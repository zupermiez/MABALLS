"""
Fold a base-mounted Motive rigid body into the existing base<-mocap
calibration so the rig can be physically moved without redoing the full
calibrate_frames.py sweep - see CLAUDE.md "remember calibration relative to
a base-mounted rigid body" for the rationale.

Does NOT move the robot and does NOT re-solve base<-mocap - it assumes the
transform already in --transform-in (default UR10_T_base_from_mocap.json) is
currently valid (i.e. the rig hasn't moved since that file was written), and
composes it with a single stationary reading of the base rigid body's live
pose to get base<-RB (frames.compose_base_from_rigid_body): the fixed offset
from the RB's local frame to the robot base frame. That offset is invariant
to where the rig sits in the mocap volume, as long as the RB stays rigidly
fixed to the robot's base (below joint 1 - see CLAUDE.md) - so runtime code
can later recover a live base<-mocap every tick via
frames.base_from_mocap_via_rigid_body(), instead of trusting a static file.

The only "calibration" happening here is averaging the RB's pose over a
stationary window for noise reduction - same stability-gated capture idea as
calibrate_frames.py's pose sampling, just for one point instead of ~20-40.

This script's own self-check (comparing the composed-then-decomposed result
back against the loaded transform) only proves the algebra round-trips - it
CANNOT catch a wrong quaternion axis/sign convention, since decomposing with
the same inputs used to compose will match by construction. Real validation
needs an independent check: after this runs, probe a point whose base-frame
position is known some other way (e.g. re-run calibrate_frames.py's
verify_live() against a live TCP marker) and confirm predicted-vs-actual
still agrees - do this BEFORE moving the rig, and again AFTER, once you trust
the base RB has stayed put.
"""
import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from natnet import NatNetClient, DataFrame

from frames import (quat_to_matrix, compose_base_from_rigid_body,
                     base_from_mocap_via_rigid_body)

DEFAULT_DURATION = 3.0     # s, stationary capture window to average over
STD_LIMIT = 0.003          # m - reject if position std-dev over the window exceeds this
MIN_VALID_RATIO = 0.9      # fraction of frames during the window that must be tracking_valid


class CaptureState:
    def __init__(self, target_id):
        self.target_id = target_id
        self.candidate_ids = []
        self.samples = []  # (pos: Vec3, rot: Vec4, valid: bool)
        self.capturing = False


def make_handler(state: CaptureState):
    def handle_frame(frame: DataFrame) -> None:
        ids = [rb.id_num for rb in frame.rigid_bodies]
        state.candidate_ids = ids
        rb = next((r for r in frame.rigid_bodies if r.id_num == state.target_id), None)
        if rb is None:
            return
        valid = True if rb.tracking_valid is None else rb.tracking_valid
        if state.capturing:
            state.samples.append((rb.pos, rb.rot, valid))
    return handle_frame


def average_pose(samples):
    """Average position (mean) and orientation (sign-aligned mean + renormalize -
    valid for a near-constant orientation like a stationary rigid body; NOT a
    general quaternion-averaging method) over the valid samples in a capture window.
    Returns (pos_avg, quat_avg, pos_std, valid_ratio)."""
    valid_samples = [(p, r) for p, r, v in samples if v]
    valid_ratio = len(valid_samples) / len(samples) if samples else 0.0
    if not valid_samples:
        return None, None, None, valid_ratio

    positions = np.array([p for p, r in valid_samples])
    pos_avg = positions.mean(axis=0)
    pos_std = positions.std(axis=0)

    quats = np.array([r for p, r in valid_samples])
    ref = quats[0]
    signs = np.sign(quats @ ref)
    signs[signs == 0] = 1.0
    quats = quats * signs[:, None]
    quat_avg = quats.mean(axis=0)
    quat_avg = quat_avg / np.linalg.norm(quat_avg)

    return pos_avg, quat_avg, pos_std, valid_ratio


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--rigid-body-id", type=int, required=True,
                         help="NatNet id of the base-mounted rigid body (no auto-detect - other "
                              "rigid bodies, e.g. the tool RB or ball, may also be visible)")
    parser.add_argument("--transform-in", default="UR10_T_base_from_mocap.json",
                         help="Existing base<-mocap calibration, assumed still valid right now "
                              "(default UR10_T_base_from_mocap.json)")
    parser.add_argument("--out", default="UR10_T_base_from_baseRB.json", help="Output JSON path")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                         help=f"Seconds to average the base RB's pose over (default {DEFAULT_DURATION})")
    parser.add_argument("--std-limit", type=float, default=STD_LIMIT,
                         help=f"Reject if position std-dev over the window exceeds this, meters "
                              f"(default {STD_LIMIT})")
    parser.add_argument("--min-valid-ratio", type=float, default=MIN_VALID_RATIO,
                         help=f"Minimum fraction of frames that must be tracking_valid (default {MIN_VALID_RATIO})")
    args = parser.parse_args()

    transform_in = json.loads(Path(args.transform_in).read_text())
    R_base_mocap = np.array(transform_in["R"])
    t_base_mocap = np.array(transform_in["t"])

    state = CaptureState(args.rigid_body_id)
    client = NatNetClient(server_ip_address=args.server_ip, local_ip_address=args.local_ip,
                           use_multicast=not args.unicast)
    client.on_data_frame_received_event.handlers.append(make_handler(state))

    with client:
        client.run_async()
        print(f"Waiting to see rigid body id={args.rigid_body_id}...")
        start = time.monotonic()
        while not state.samples and args.rigid_body_id not in state.candidate_ids:
            if time.monotonic() - start > 10.0:
                raise SystemExit(
                    f"Never saw rigid body id={args.rigid_body_id} (saw: {state.candidate_ids}). "
                    "Check it's created in Motive and streaming."
                )
            time.sleep(0.1)

        print(f"Capturing {args.duration}s of rigid body id={args.rigid_body_id} - keep it stationary...")
        state.capturing = True
        time.sleep(args.duration)
        state.capturing = False

    pos_avg, quat_avg, pos_std, valid_ratio = average_pose(state.samples)
    print(f"n_samples={len(state.samples)}  valid_ratio={valid_ratio:.2f}  "
          f"pos_std_mm={(pos_std * 1000).round(2).tolist() if pos_std is not None else None}")

    if pos_avg is None:
        raise SystemExit("No valid samples captured - rigid body was never tracking_valid.")
    if valid_ratio < args.min_valid_ratio:
        raise SystemExit(
            f"REJECTED: valid_ratio={valid_ratio:.2f} < {args.min_valid_ratio} - check for occlusion."
        )
    if np.any(pos_std > args.std_limit):
        raise SystemExit(
            f"REJECTED: position std-dev {(pos_std * 1000).round(2).tolist()} mm exceeds "
            f"{args.std_limit * 1000:.1f}mm limit - rigid body wasn't actually stationary, or "
            "the mount is loose/vibrating."
        )

    R_mocap_rb = quat_to_matrix(quat_avg)
    t_mocap_rb = pos_avg

    R_base_rb, t_base_rb = compose_base_from_rigid_body(R_base_mocap, t_base_mocap, R_mocap_rb, t_mocap_rb)

    # Algebra self-check only - see module docstring for what this does and doesn't prove.
    R_check, t_check = base_from_mocap_via_rigid_body(R_base_rb, t_base_rb, R_mocap_rb, t_mocap_rb)
    r_err = np.max(np.abs(R_check - R_base_mocap))
    t_err = np.max(np.abs(t_check - t_base_mocap))
    print(f"algebra round-trip check: max R err={r_err:.2e}  max t err={t_err:.2e} (should be ~1e-10)")
    if r_err > 1e-6 or t_err > 1e-6:
        raise SystemExit("Round-trip check failed - this is a bug, not a calibration problem. Do not use the output.")

    out = {
        "R": R_base_rb.tolist(),
        "t": t_base_rb.tolist(),
        "rigid_body_id": args.rigid_body_id,
        "n_samples": len(state.samples),
        "valid_ratio": valid_ratio,
        "pos_std_m": pos_std.tolist(),
        "rb_pos_mocap": t_mocap_rb.tolist(),
        "rb_quat_mocap": quat_avg.tolist(),
        "source_transform_file": args.transform_in,
        "source_transform_rmse_m": transform_in.get("rmse_m"),
        "source_transform_created_utc": transform_in.get("created_utc"),
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(f"\nSaved base<-rigid_body transform to {args.out}")
    print(
        "\nThis composed transform is only as good as the algebra + the source transform's own "
        f"accuracy ({transform_in.get('rmse_m', '?')}m rmse) - it has NOT been independently "
        "validated against ground truth (see module docstring). Before trusting it: re-run "
        "calibrate_frames.py's live verification (or an equivalent known-point check) and confirm "
        "predicted-vs-actual still agrees, ideally both before and after physically moving the rig."
    )


if __name__ == "__main__":
    main()
