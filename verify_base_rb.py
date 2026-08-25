"""
Live validation for calibrate_base_rb.py's output.

Every tick: recompute base<-mocap from the base rigid body's LIVE pose
(frames.base_from_mocap_via_rigid_body), use it to predict the TCP marker's
base-frame position, and compare against actual FK (rtde_receive) - the same
predicted-vs-actual check calibrate_frames.py's verify_live() does for the
static transform, but exercising the new live-recomputed one instead. This
is the independent check calibrate_base_rb.py's own algebra self-check
CANNOT provide (see that script's docstring) - it uses a second, separately
measured point (the TCP marker) as ground truth, not the same base-RB sample
used to build the transform.

Run this BEFORE moving the rig - the error should closely match whatever
calibrate_frames.py's own verification already showed (same order as its
fit rmse). Then physically move the robot+base-RB assembly and run it AGAIN,
without rerunning calibrate_frames.py - agreement after a real move is the
actual proof this works.

Reuses calibrate_frames.py's marker-tracking handlers so the TCP marker is
tracked identically to how it was during calibration (same tracking_mode,
read from the source transform file referenced in --base-rb-transform).
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rtde_receive
from natnet import NatNetClient, DataFrame

from frames import mocap_point_to_base, base_from_mocap_via_rigid_body, quat_to_matrix, rotvec_to_matrix
from calibrate_frames import (MocapState, make_marker_handler, make_handler,
                               STATE_LOCK, ROBOT_IP, apply_and_verify_tcp)

# Re-exported under its own name so importers (e.g. catch.py, tracking the ball under
# its own separate lock from live_trajectory.py) don't confuse this lock - which
# protects BaseRBState - with any other STATE_LOCK in scope.
BASE_RB_LOCK = STATE_LOCK


class BaseRBState:
    def __init__(self):
        self.latest_pos = None
        self.latest_rot = None
        self.latest_valid = False


def make_base_rb_handler(state: BaseRBState, target_id):
    def handle_frame(frame: DataFrame) -> None:
        rb = next((r for r in frame.rigid_bodies if r.id_num == target_id), None)
        if rb is None:
            return
        valid = True if rb.tracking_valid is None else rb.tracking_valid
        with STATE_LOCK:
            state.latest_pos = rb.pos
            state.latest_rot = rb.rot
            state.latest_valid = valid
    return handle_frame


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--base-rb-transform", default="UR10_T_base_from_baseRB.json",
                         help="Output of calibrate_base_rb.py")
    parser.add_argument("--tcp-rigid-body-id", type=int, default=None,
                         help="Track this rigid body id as ground truth instead of the marker mode "
                              "recorded in the source transform file - use when the calibration-time "
                              "point (e.g. a lone unlabeled marker) is no longer physically present "
                              "and a proper Rigid Body asset now sits on the tool instead. Only valid "
                              "as ground truth if its pivot is at the SAME physical point the "
                              "calibration's tool-offset fit was solved for - a different pivot (e.g. "
                              "the RB's default marker centroid vs. the exact old marker location) "
                              "will show up as extra error that isn't a calibration problem.")
    parser.add_argument("--duration", type=float, default=None, help="Seconds (default: until Ctrl-C)")
    parser.add_argument("--use-static-transform", action="store_true",
                         help="Control test: skip the base RB entirely and use --transform-in's static "
                              "base<-mocap transform directly (same thing calibrate_frames.py's own "
                              "verify_live() does). Since the base RB is physically stationary, the "
                              "live-derived transform is essentially constant too - so a pose-dependent "
                              "error (grows/shrinks as the arm moves, not a fixed bias) can't be a bug "
                              "in the base-RB composition code, which doesn't vary with arm pose either "
                              "way. This flag isolates that: same marker setup, base RB fully out of "
                              "the picture - if the same pattern shows up, it's the marker, not the code.")
    parser.add_argument("--transform-in", default="UR10_T_base_from_mocap.json",
                         help="With --use-static-transform: the base<-mocap file to use directly")
    parser.add_argument("--skip-set-tcp", action="store_true",
                         help="Don't send set_tcp() - use if the calibration's box-centroid TCP is "
                              "already known to be active. By default this script sets it explicitly "
                              "(same as calibrate_frames.py) since getActualTCPPose() is meaningless as "
                              "ground truth otherwise - a stale/different active TCP offset is a known "
                              "failure mode here (see CLAUDE.md, 2026-07-17) and produces exactly this "
                              "kind of orientation-dependent error.")
    args = parser.parse_args()

    if args.use_static_transform:
        source = json.loads(Path(args.transform_in).read_text())
        R_base_rb = t_base_rb = base_rb_id = None  # unused in this mode
    else:
        base_rb_data = json.loads(Path(args.base_rb_transform).read_text())
        R_base_rb = np.array(base_rb_data["R"])
        t_base_rb = np.array(base_rb_data["t"])
        base_rb_id = base_rb_data["rigid_body_id"]
        source = json.loads(Path(base_rb_data["source_transform_file"]).read_text())

    fit_mode = source.get("fit_mode")
    d = np.array(source["marker_offset_tool_m"]) if fit_mode == "tool_offset" else np.zeros(3)
    tracking_mode = source["tracking_mode"]

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    if not args.skip_set_tcp:
        apply_and_verify_tcp(rtde_r, source["tcp_offset"])
    else:
        print("Skipping set_tcp() - assuming the calibration's box-centroid TCP is already active.")

    marker_state = MocapState()
    rb_state = BaseRBState()

    client = NatNetClient(server_ip_address=args.server_ip, local_ip_address=args.local_ip,
                           use_multicast=not args.unicast)
    if args.tcp_rigid_body_id is not None:
        client.on_data_frame_received_event.handlers.append(
            make_handler(marker_state, args.tcp_rigid_body_id))
        tracking_mode = f"rigid_body (forced, id={args.tcp_rigid_body_id})"
    elif tracking_mode.startswith("unlabeled_marker_"):
        marker_source = tracking_mode.replace("unlabeled_marker_", "").replace("_", "-")
        client.on_data_frame_received_event.handlers.append(
            make_marker_handler(marker_state, None, marker_source))
    elif tracking_mode == "rigid_body":
        client.on_data_frame_received_event.handlers.append(
            make_handler(marker_state, source["rigid_body_id"]))
    else:
        raise SystemExit(f"tracking_mode={tracking_mode!r} not wired up in this verify script yet "
                          "(asset_marker source - add a handler if needed)")
    if not args.use_static_transform:
        client.on_data_frame_received_event.handlers.append(make_base_rb_handler(rb_state, base_rb_id))

    mode_label = "STATIC transform (control test)" if args.use_static_transform else f"base RB id={base_rb_id}"
    print(f"{mode_label}, TCP marker via {tracking_mode}. "
          "Comparing predicted vs actual (FK) - Ctrl-C to stop.")
    with client:
        client.run_async()
        start = time.monotonic()
        try:
            while args.duration is None or time.monotonic() - start < args.duration:
                actual_pose = rtde_r.getActualTCPPose()
                actual = np.array(actual_pose[:3])
                if np.any(d):
                    actual = actual + rotvec_to_matrix(actual_pose[3:6]) @ d

                with STATE_LOCK:
                    marker_pos, marker_valid = marker_state.latest_pos, marker_state.latest_valid
                    rb_pos, rb_rot, rb_valid = rb_state.latest_pos, rb_state.latest_rot, rb_state.latest_valid

                tracking_ok = marker_valid and (args.use_static_transform or rb_valid)
                if tracking_ok:
                    if args.use_static_transform:
                        R_t, t_t = np.array(source["R"]), np.array(source["t"])
                    else:
                        R_mocap_rb = quat_to_matrix(rb_rot)
                        R_t, t_t = base_from_mocap_via_rigid_body(R_base_rb, t_base_rb, R_mocap_rb, np.array(rb_pos))
                    predicted = mocap_point_to_base(np.array(marker_pos), R_t, t_t)
                    err = np.linalg.norm(predicted - actual)
                    print(f"\rpredicted=({predicted[0]:+.4f},{predicted[1]:+.4f},{predicted[2]:+.4f})  "
                          f"actual=({actual[0]:+.4f},{actual[1]:+.4f},{actual[2]:+.4f})  "
                          f"error={err * 1000:6.1f} mm   ", end="", flush=True)
                else:
                    print(f"\rwaiting for valid tracking (marker={marker_valid} rb={rb_valid})..."
                          "                                ", end="", flush=True)
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass
    print()


if __name__ == "__main__":
    main()
