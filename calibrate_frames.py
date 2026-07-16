"""
Solves for T_base<-mocap: the rigid transform (rotation R, translation t)
mapping a point in Motive's world frame to the UR12e's base frame -
p_base = R @ p_mocap + t - via frames.umeyama_rigid_transform().

Procedure (see CLAUDE.md "Catch Integration" for the full rationale): visit
~20-40 poses spanning the reachable workspace with a single physical point -
the box-centroid TCP - and at each pose record where the robot's own forward
kinematics says that point is (p_robot, base frame, via rtde_receive) and
where Motive says the same point is (p_mocap, mocap frame, via NatNet). The
two point clouds are then fed to the SVD-based fit.

This only works if p_robot and p_mocap are the *same physical point* at every
pose - i.e. the robot's configured TCP must be the box's marker-rigid-body
centroid, not the flange. This script sends set_tcp() (over the raw
URScript-over-socket path, same as jog_ur_raw.py/ur_goto_raw.py) to do that,
using --tcp-offset (default: measured from a 30(w) x 23(h) x 24(d) cm
cardboard box, flange mounted flush/centered on the 24cm-deep back face, so
the geometric center sits 12cm straight out along the flange's Z axis, X/Y=0).
It is the operator's responsibility to have already set the Motive rigid
body's pivot to that same physical point (Rigid Body Properties -> Translate
Pivot) - this script only sanity-checks that set_tcp() visibly changed the
reported pose, not that the two points actually coincide; that's what the
live verification pass at the end is for.

Single-person workflow: this script never commands robot motion. Move the
arm by hand (freedrive) or with a jog script running separately, hold it
still, then press Enter here to sample - no need to be at the pendant and
the keyboard at the same time.
"""

import argparse
import json
import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rtde_receive
from natnet import NatNetClient, DataFrame

from frames import umeyama_rigid_transform, mocap_point_to_base

ROBOT_IP = "192.168.20.1"
SECONDARY_PORT = 30002  # raw URScript-over-socket - see CLAUDE.md

# Measured 2026-07-13: 30(w) x 23(h) x 24(d) cm cardboard box, flange mounted
# flush and centered on the 24cm-deep back face -> geometric center sits at
# half the depth straight out along the flange's local Z axis, X/Y=0 since
# centered. Must match wherever the Motive rigid body's pivot is set to.
TCP_OFFSET = (0.0, 0.0, 0.12, 0.0, 0.0, 0.0)  # x,y,z (m), rx,ry,rz (rad)

DEFAULT_NUM_SAMPLES = 25
MIN_SAMPLES = 8            # hard floor - below this the SVD fit is poorly conditioned
RECOMMENDED_SAMPLES = 20   # CLAUDE.md's "20-40 spanning the workspace" guidance
SAMPLE_WINDOW = 0.6        # seconds averaged per captured pose
POLL_INTERVAL = 0.02       # s, ~50Hz robot-side polling during a capture window
STATIONARY_SPEED = 0.01    # m/s - peak TCP speed above this during a capture -> reject & retry
MIN_MOCAP_VALID_RATIO = 0.9   # fraction of frames during the window that must be tracking_valid
MOCAP_STD_LIMIT = 0.005    # m - std-dev of captured mocap positions above this -> reject & retry
OUTLIER_FACTOR = 3.0       # flag a sample as a possible outlier above this multiple of the RMSE
OUTLIER_ABS = 0.02         # ...or above this absolute residual (m), whichever is larger

STATE_LOCK = threading.Lock()

INSTRUCTIONS = """
Frame calibration - Motive (mocap) <-> UR12e base frame
--------------------------------------------------------
For each sample: move the arm (freedrive, or a jog script in another
terminal) so the box sits at a new pose spanning the workspace, hold it
still, then press Enter here. This script only reads state - it never
drives the robot - so one person can do the whole thing alone.

Type 'q' instead of Enter to stop early (once >= {min_samples} samples are
in). Bad captures (robot moved, rigid body occluded/unstable) are rejected
automatically with a reason - just retry the same pose.
""".format(min_samples=MIN_SAMPLES)


def send_urscript(script, timeout=5.0):
    with socket.create_connection((ROBOT_IP, SECONDARY_PORT), timeout=timeout) as s:
        s.sendall(script.encode("utf-8"))


def set_tcp_script(offset):
    off = "p[" + ",".join(f"{v:.6f}" for v in offset) + "]"
    return f"""def prog():
  set_tcp({off})
end
prog()
"""


class MocapState:
    def __init__(self):
        self.target_id = None
        self.candidate_ids = []
        self.latest_pos = None
        self.latest_valid = False
        self.capturing = False
        self.capture_samples = []  # (x,y,z) if valid, None if not - filled while capturing=True


def make_handler(state: MocapState, forced_id):
    def handle_frame(frame: DataFrame) -> None:
        with STATE_LOCK:
            if state.target_id is None:
                ids = [rb.id_num for rb in frame.rigid_bodies]
                if forced_id is not None:
                    state.target_id = forced_id
                elif len(ids) == 1:
                    state.target_id = ids[0]
                else:
                    state.candidate_ids = ids
                    return

            rb = next((r for r in frame.rigid_bodies if r.id_num == state.target_id), None)
            if rb is None:
                return

            valid = True if rb.tracking_valid is None else rb.tracking_valid
            state.latest_pos = rb.pos
            state.latest_valid = valid
            if state.capturing:
                state.capture_samples.append(rb.pos if valid else None)

    return handle_frame


def wait_for_rigid_body(state: MocapState, timeout=10.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        with STATE_LOCK:
            target_id = state.target_id
            candidates = list(state.candidate_ids)
        if target_id is not None:
            return target_id
        if candidates:
            raise SystemExit(
                f"Multiple rigid bodies seen: {candidates}. Re-run with --rigid-body-id <id>."
            )
        time.sleep(0.1)
    raise SystemExit("No rigid body seen within timeout - check Motive streaming/tracking.")


def capture_robot_window(rtde_r, duration, poll_interval):
    positions = []
    max_speed = 0.0
    end = time.monotonic() + duration
    while time.monotonic() < end:
        pose = rtde_r.getActualTCPPose()
        speed = rtde_r.getActualTCPSpeed()
        lin_speed = (speed[0] ** 2 + speed[1] ** 2 + speed[2] ** 2) ** 0.5
        positions.append(pose[:3])
        max_speed = max(max_speed, lin_speed)
        time.sleep(poll_interval)
    return np.array(positions), max_speed


def capture_mocap_window(state: MocapState, duration):
    with STATE_LOCK:
        state.capture_samples = []
        state.capturing = True
    time.sleep(duration)
    with STATE_LOCK:
        state.capturing = False
        raw = list(state.capture_samples)
    total = len(raw)
    valid = [s for s in raw if s is not None]
    ratio = (len(valid) / total) if total else 0.0
    positions = np.array(valid) if valid else None
    return positions, ratio


def capture_pair(rtde_r, state, duration, poll_interval):
    mocap_result = {}

    def _mocap():
        positions, ratio = capture_mocap_window(state, duration)
        mocap_result["positions"] = positions
        mocap_result["ratio"] = ratio

    thread = threading.Thread(target=_mocap)
    thread.start()
    robot_positions, max_speed = capture_robot_window(rtde_r, duration, poll_interval)
    thread.join()

    return robot_positions, max_speed, mocap_result["positions"], mocap_result["ratio"]


def prompt_and_sample(rtde_r, state, index, target_n, collected_so_far):
    while True:
        print(f"\n[{index}/{target_n}] Move to the next pose and hold still, then press Enter "
              f"to sample (or 'q' to stop - have {collected_so_far}, need >= {MIN_SAMPLES}).")
        line = input("> ").strip().lower()
        if line == "q":
            return "stop"

        robot_positions, max_speed, mocap_positions, mocap_ratio = capture_pair(
            rtde_r, state, SAMPLE_WINDOW, POLL_INTERVAL
        )

        if max_speed > STATIONARY_SPEED:
            print(f"  REJECTED: robot was moving during capture (peak speed "
                  f"{max_speed * 1000:.1f} mm/s > {STATIONARY_SPEED * 1000:.0f} mm/s threshold). "
                  f"Let go of freedrive / stop jogging fully, then retry.")
            continue

        if mocap_positions is None or mocap_ratio < MIN_MOCAP_VALID_RATIO:
            got = f"{mocap_ratio * 100:.0f}%" if mocap_positions is not None else "no frames received"
            print(f"  REJECTED: rigid body tracking was invalid too often during capture "
                  f"({got} valid, need >= {MIN_MOCAP_VALID_RATIO * 100:.0f}%). Check for "
                  f"occlusion or a lost NatNet connection, then retry.")
            continue

        mocap_std = mocap_positions.std(axis=0)
        if np.any(mocap_std > MOCAP_STD_LIMIT):
            print(f"  REJECTED: mocap position was unstable during capture (std="
                  f"{[round(v * 1000, 2) for v in mocap_std]} mm, limit "
                  f"{MOCAP_STD_LIMIT * 1000:.0f} mm) - possible partial occlusion or box wobble. Retry.")
            continue

        p_robot = robot_positions.mean(axis=0)
        p_mocap = mocap_positions.mean(axis=0)
        print(f"  OK  p_robot=({p_robot[0]:+.4f},{p_robot[1]:+.4f},{p_robot[2]:+.4f})  "
              f"p_mocap=({p_mocap[0]:+.4f},{p_mocap[1]:+.4f},{p_mocap[2]:+.4f})  "
              f"mocap_valid={mocap_ratio * 100:.0f}%")
        return p_robot, p_mocap


def load_resume_samples(path, expected_rigid_body_id, expected_tcp_offset):
    """Load (p_robot, p_mocap) pairs from a previous run's --out JSON, so a
    session can continue past sample N instead of starting over. Refuses to
    mix samples collected against a different rigid body or a different TCP
    offset - either would make old and new p_robot values incomparable."""
    try:
        data = json.loads(Path(path).read_text())
    except FileNotFoundError:
        raise SystemExit(f"--resume file not found: {path}")

    saved_id = data.get("rigid_body_id")
    if saved_id is not None and saved_id != expected_rigid_body_id:
        raise SystemExit(
            f"Refusing to resume: {path} was collected against rigid body id={saved_id}, "
            f"but this run is tracking id={expected_rigid_body_id}. Re-run with "
            f"--rigid-body-id {saved_id} to match, or start fresh without --resume."
        )

    saved_offset = data.get("tcp_offset")
    if saved_offset is not None and any(
        abs(a - b) > 1e-6 for a, b in zip(saved_offset, expected_tcp_offset)
    ):
        raise SystemExit(
            f"Refusing to resume: {path} was collected with --tcp-offset {saved_offset}, "
            f"but this run is using {expected_tcp_offset}. p_robot values aren't comparable "
            f"across different TCP offsets - pass --tcp-offset {saved_offset} to match, or "
            f"start fresh without --resume."
        )

    return [(np.array(s["p_robot"]), np.array(s["p_mocap"])) for s in data["samples"]]


def collect_samples(rtde_r, state, target_n, existing_pairs=None):
    pairs = list(existing_pairs) if existing_pairs else []
    if pairs:
        print(f"Resuming with {len(pairs)} existing sample(s) - continuing from "
              f"sample {len(pairs) + 1}.")
    if len(pairs) >= target_n:
        print(f"Already have {len(pairs)} samples, >= target of {target_n} - skipping "
              f"new collection (pass --num-samples to collect more).")
        return pairs

    index = len(pairs) + 1
    while True:
        result = prompt_and_sample(rtde_r, state, index, target_n, len(pairs))
        if result == "stop":
            if len(pairs) < MIN_SAMPLES:
                print(f"Need at least {MIN_SAMPLES} samples, have {len(pairs)} - keep going.")
                continue
            break
        pairs.append(result)
        index += 1
        if len(pairs) >= target_n:
            break
    if len(pairs) < RECOMMENDED_SAMPLES:
        print(f"\nNOTE: only {len(pairs)} samples collected - CLAUDE.md recommends 20-40 spanning "
              f"the workspace for a reliable fit. Consider collecting more next time.")
    return pairs


def fit_and_report(pairs):
    p_mocap = np.array([p[1] for p in pairs])
    p_robot = np.array([p[0] for p in pairs])

    R, t = umeyama_rigid_transform(p_mocap, p_robot)
    predicted = mocap_point_to_base(p_mocap, R, t)
    errors = np.linalg.norm(predicted - p_robot, axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))

    print(f"\nFit complete: {len(pairs)} samples, RMSE = {rmse * 1000:.2f} mm")
    outlier_limit = max(OUTLIER_FACTOR * rmse, OUTLIER_ABS)
    for i, e in enumerate(errors):
        flag = "  <-- possible outlier, consider re-collecting this pose" if e > outlier_limit else ""
        print(f"  sample {i + 1:2d}: residual = {e * 1000:6.2f} mm{flag}")

    if rmse > 0.015:
        print("\nWARNING: RMSE is high relative to the <1.5mm published mocap-robot benchmark "
              "(CLAUDE.md). Most likely cause: the TCP offset (--tcp-offset) and the Motive "
              "rigid body's pivot don't actually point at the same physical spot - double check "
              "both before trusting this transform.")

    return R, t, rmse, errors


def verify_live(rtde_r, state, R, t, duration):
    print("\nVerification: move the arm around (freedrive/jog) and watch predicted vs actual "
          "agree - this is the real check that the transform (and the TCP/pivot alignment behind "
          "it) is correct, not just that the fit converged. Ctrl-C to stop.")
    start = time.monotonic()
    try:
        while duration is None or time.monotonic() - start < duration:
            actual = np.array(rtde_r.getActualTCPPose()[:3])
            with STATE_LOCK:
                mocap_pos, valid = state.latest_pos, state.latest_valid
            if mocap_pos is not None and valid:
                predicted = mocap_point_to_base(np.array(mocap_pos), R, t)
                err = np.linalg.norm(predicted - actual)
                print(f"\rpredicted=({predicted[0]:+.4f},{predicted[1]:+.4f},{predicted[2]:+.4f})  "
                      f"actual=({actual[0]:+.4f},{actual[1]:+.4f},{actual[2]:+.4f})  "
                      f"error={err * 1000:6.1f} mm   ", end="", flush=True)
            else:
                print("\rwaiting for valid rigid body tracking...                                  ",
                      end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--rigid-body-id", type=int, default=None,
                         help="NatNet rigid body id to track (auto-selected if only one is visible)")
    parser.add_argument("--num-samples", type=int, default=DEFAULT_NUM_SAMPLES,
                         help=f"Target number of calibration poses (default {DEFAULT_NUM_SAMPLES})")
    parser.add_argument("--tcp-offset", type=float, nargs=6, default=list(TCP_OFFSET),
                         metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                         help="Box-centroid TCP offset from the flange, meters/radians "
                              f"(default {list(TCP_OFFSET)}, measured from the 30x23x24cm box)")
    parser.add_argument("--skip-set-tcp", action="store_true",
                         help="Don't send set_tcp() - use if the box-centroid TCP is already "
                              "active (e.g. configured on the pendant instead)")
    parser.add_argument("--out", default="T_base_from_mocap.json", help="Output JSON path")
    parser.add_argument("--resume", default=None,
                         help="Continue from a previous run's output JSON (e.g. --out's default "
                              "path) instead of starting over - same rigid body and TCP offset "
                              "required, and it's fine to pass the same path as --out")
    parser.add_argument("--skip-verify", action="store_true", help="Skip the live verification pass at the end")
    parser.add_argument("--verify-duration", type=float, default=None,
                         help="Seconds to run the verification pass (default: until Ctrl-C)")
    args = parser.parse_args()

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    if not args.skip_set_tcp:
        before = rtde_r.getActualTCPPose()
        send_urscript(set_tcp_script(args.tcp_offset))
        time.sleep(0.2)  # let the controller apply it before the next read
        after = rtde_r.getActualTCPPose()
        print(f"set_tcp({list(args.tcp_offset)}) sent.")
        print(f"  TCP pose before: {[round(v, 4) for v in before]}")
        print(f"  TCP pose after:  {[round(v, 4) for v in after]}")
        if all(abs(a - b) < 1e-4 for a, b in zip(before, after)):
            print("  NOTE: pose didn't change - harmless if this offset was already active from "
                  "a previous run (re-sending the same value is a no-op). If this is the *first* "
                  "time this offset has been applied this session and it still didn't move, "
                  "check the active TCP on the pendant (Installation -> TCP Configuration).")
    else:
        print("Skipping set_tcp() - assuming the box-centroid TCP is already active.")

    state = MocapState()
    client = NatNetClient(
        server_ip_address=args.server_ip,
        local_ip_address=args.local_ip,
        use_multicast=not args.unicast,
    )
    client.on_data_frame_received_event.handlers.append(make_handler(state, args.rigid_body_id))

    with client:
        client.run_async()
        try:
            print("\nWaiting for Motive rigid body...")
            target_id = wait_for_rigid_body(state)
            print(f"Tracking rigid body id={target_id}")

            existing_pairs = []
            if args.resume:
                existing_pairs = load_resume_samples(args.resume, target_id, args.tcp_offset)

            print(INSTRUCTIONS)
            pairs = collect_samples(rtde_r, state, args.num_samples, existing_pairs=existing_pairs)
            R, t, rmse, errors = fit_and_report(pairs)

            out = {
                "R": R.tolist(),
                "t": t.tolist(),
                "rmse_m": rmse,
                "n_samples": len(pairs),
                "tcp_offset": list(args.tcp_offset),
                "rigid_body_id": target_id,
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "samples": [
                    {"p_robot": p_robot.tolist(), "p_mocap": p_mocap.tolist(), "residual_m": float(err)}
                    for (p_robot, p_mocap), err in zip(pairs, errors)
                ],
            }
            Path(args.out).write_text(json.dumps(out, indent=2))
            print(f"\nSaved transform to {args.out}")

            if not args.skip_verify:
                verify_live(rtde_r, state, R, t, args.verify_duration)
        finally:
            client.stop_async()

    rtde_r.disconnect()


if __name__ == "__main__":
    main()
