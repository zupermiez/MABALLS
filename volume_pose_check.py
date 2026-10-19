"""
volume_pose_check.py - slow, step-by-step real-arm check of the poses demo.py's catch
VOLUME (--catch-volume-height / --volume-max-tilt) can command, BEFORE throwing at it.

New risk vs the plane-only catch: targets up to 0.5m above the wait height with the
box mouth tilted up to 90deg. Things to watch per pose: robot-side IK finds a
solution near the current joints (no big configuration flip), wrist2 stays away from
0/180deg (wrist singularity), and the box clears the forearm/wrist housings.

Each pose: printed, then ENTER to move (s = skip, q = quit), slow movej (IK resolved
on the robot with qnear = current joints), safety-mode check, joints printed, then
back to the wait pose. Hand on the E-stop. Default speed 0.4 rad/s.

  python3 volume_pose_check.py [--height 0.5] [--speed 0.4] [--accel 0.5]
"""

import argparse
import json
import math
import sys
import time

import numpy as np
import rtde_receive

from calibrate_frames import send_urscript, set_tcp_script
from demo import (DEFAULT_WAIT_POSE, check_catch_envelope, tilt_follow_orientation, yaw_follow_orientation)
from ur_goto_raw import ROBOT_IP, movej_script, movej_to_pose_script, send_script, wait_for_stop

DEG = math.pi / 180


def build_poses(wait_pose, height):
    """(label, pose) list: volume corners x mouth tilts. "out" = away from the base,
    the direction a ball thrown at the arm comes FROM; "side" = along the tangent."""
    wait = np.array(wait_pose[:3])
    r = float(np.hypot(wait[0], wait[1]))
    az0 = math.atan2(wait[1], wait[0])
    spots = [("wait xy, top", az0, wait[2] + height),
             ("wait xy, mid", az0, wait[2] + height / 2),
             ("az +40, top", az0 + 40 * DEG, wait[2] + height),
             ("az -40, top", az0 - 40 * DEG, wait[2] + height)]
    tilts = [("upright", None, 0), ("out 45", "out", 45), ("out 90", "out", 90),
             ("side +60", "side", 60), ("side -60", "side", -60)]
    poses = []
    for sname, az, z in spots:
        p = np.array([r * math.cos(az), r * math.sin(az), z])
        rv = yaw_follow_orientation(list(wait_pose), wait, p)
        for tname, kind, deg in tilts:
            if kind is None:
                rv_t = rv
            else:
                h = np.array([math.cos(az), math.sin(az), 0.0]) if kind == "out" \
                    else np.array([-math.sin(az), math.cos(az), 0.0]) * math.copysign(1, deg)
                a = abs(deg) * DEG
                mouth = math.cos(a) * np.array([0, 0, 1.0]) + math.sin(a) * h
                # Ball arriving opposite the mouth; the direction-dependent limits
                # (catch_feasibility.TILT_LIMIT_*) clamp these exactly as a real catch would.
                rv_t = tilt_follow_orientation(rv, -mouth, math.pi / 2, p)
            poses.append((f"{sname:13s} | {tname}", [float(p[0]), float(p[1]), float(p[2])] + list(rv_t)))
    return poses


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height", type=float, default=0.5)
    ap.add_argument("--speed", type=float, default=0.4, help="rad/s")
    ap.add_argument("--accel", type=float, default=0.5, help="rad/s^2")
    ap.add_argument("--transform-file", default="UR10_T_base_from_mocap.json", help="only for tcp_offset")
    args = ap.parse_args()

    wait_pose = list(DEFAULT_WAIT_POSE)
    poses = build_poses(wait_pose, args.height)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    tcp_offset = json.load(open(args.transform_file))["tcp_offset"]
    print(f"set_tcp {tcp_offset}  |  {len(poses)} poses  |  movej v={args.speed} a={args.accel}")
    if input("Hand on the E-stop? Type 'yes' to start: ").strip() != "yes":
        return
    send_urscript(set_tcp_script(tcp_offset))
    time.sleep(0.5)

    send_script(movej_to_pose_script(wait_pose, list(rtde_r.getActualQ()), args.speed, args.accel))
    wait_for_stop(rtde_r, is_joint=True)
    q_wait = list(rtde_r.getActualQ())
    print("at wait pose, joints(deg):", np.round(np.degrees(q_wait), 1).tolist())

    results = []
    for label, pose in poses:
        env = check_catch_envelope(np.array(pose[:3]), np.array(wait_pose[:3]))
        print(f"\n{label}   pose={np.round(pose, 3).tolist()}   envelope: {env or 'ok'}")
        ans = input("  ENTER = move, s = skip, q = quit: ").strip().lower()
        if ans == "q":
            break
        if ans == "s":
            continue
        q0 = list(rtde_r.getActualQ())
        send_script(movej_to_pose_script(pose, q0, args.speed, args.accel))
        settled = wait_for_stop(rtde_r, is_joint=True)
        mode = rtde_r.getSafetyMode()
        q = list(rtde_r.getActualQ())
        tcp = list(rtde_r.getActualTCPPose())
        pos_err = float(np.linalg.norm(np.array(tcp[:3]) - np.array(pose[:3])))
        dq = np.degrees(np.abs(np.array(q) - np.array(q_wait)))
        wrist2 = math.degrees(q[4])
        flags = []
        if mode != 1:
            flags.append(f"SAFETY MODE {mode}")
        if not settled:
            flags.append("did not settle")
        if pos_err > 0.01:
            flags.append(f"pos error {pos_err * 100:.1f}cm (IK fail / didn't move?)")
        if abs(math.sin(q[4])) < math.sin(15 * DEG):
            flags.append(f"wrist2={wrist2:.0f}deg near singularity")
        if dq.max() > 150:
            flags.append(f"joint {int(dq.argmax())} moved {dq.max():.0f}deg from wait (config flip?)")
        print(f"  joints(deg)={np.round(np.degrees(q), 1).tolist()}  max joint change {dq.max():.0f}deg  "
              f"-> {'; '.join(flags) if flags else 'OK'}")
        note = input("  any contact/clearance issue? (ENTER = none, or type a note): ").strip()
        results.append((label, flags, note))
        if mode != 1:
            print("  Fault - clear it on the pendant (or ur_status.py --clear), then rerun. Stopping.")
            break
        send_script(movej_script(q_wait, args.speed, args.accel))
        wait_for_stop(rtde_r, is_joint=True, current=q, target=q_wait)

    print("\nSUMMARY")
    for label, flags, note in results:
        print(f"  {label}: {'; '.join(flags) if flags else 'OK'}{('  NOTE: ' + note) if note else ''}")


if __name__ == "__main__":
    main()
