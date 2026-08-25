import argparse
import socket
import time

import rtde_receive

ROBOT_IP = "192.168.20.1"
SECONDARY_PORT = 30002   # raw URScript-over-socket, same path validated in jog_ur_raw.py
DEFAULT_SPEED = 0.1      # m/s (movel) or rad/s (movej)
DEFAULT_ACCEL = 0.3      # m/s^2 (movel) or rad/s^2 (movej)
MAX_WAIT = 15.0          # seconds - safety timeout for the completion poll below
SETTLE_SPEED = 0.001     # m/s (or rad/s) - below this counts as "stopped"
SETTLE_TICKS = 5         # consecutive slow samples required before declaring "done"
MOVE_START_SPEED = 0.005  # m/s (or rad/s) - clearly above sensor noise; below this,
                           # motion hasn't genuinely begun yet (see wait_for_stop)
NOOP_TOL = 0.001          # m (or rad) - target already reached, no need to wait for motion

# Defense-in-depth after a real incident 2026-07-10: a code bug (bare list
# instead of a p[] pose literal) sent the robot toward a wildly wrong target
# and had to be stopped with the physical E-stop. A bug in *this* script could
# happen again in some other form - this clamp is a second, independent check
# on the actual resolved target immediately before sending, regardless of
# what produced it, so a future bug is caught here even if it isn't caught by
# whatever logic computed the target.
MAX_TRANSLATION_STEP = 0.15   # m, per axis and combined - well above a normal
                                # jog nudge but far below "wrong target" scale
MAX_JOINT_STEP = 0.5           # rad, per joint


def send_script(script, timeout=5.0):
    with socket.create_connection((ROBOT_IP, SECONDARY_PORT), timeout=timeout) as s:
        s.sendall(script.encode("utf-8"))


def movel_absolute_script(pose, speed, accel):
    pose_str = "p[" + ",".join(f"{v:.6f}" for v in pose) + "]"
    return f"""def prog():
  movel({pose_str}, a={accel}, v={speed})
end
prog()
"""


def resolve_relative_pose(current_pose, delta):
    # Deliberately computed here in Python, not via URScript's pose_trans(start,
    # delta) or by building a pose expression on the robot side. Two real bugs
    # found the hard way on 2026-07-10:
    # (1) pose_trans(from, delta) composes `delta` in the FIRST pose's own
    #     (rotated) local frame, so a "+Z" delta gets silently rotated into a
    #     mix of base X/Y/Z whenever the tool isn't axis-aligned with base -
    #     a requested [0,0,+0.03,0,0,0] delta produced dx=+0.0094,
    #     dy=-0.0281, dz=-0.0049, nowhere close to pure +Z.
    # (2) the fix-attempt built the target as a plain URScript list
    #     `[x,y,z,rx,ry,rz]` instead of a pose literal `p[x,y,z,rx,ry,rz]` -
    #     movel() needs an actual pose type, and passing a bare list produced
    #     a wildly wrong, large, fast motion that triggered a real emergency
    #     stop. Computing the absolute target here and reusing the
    #     already-proven movel_absolute_script (which correctly emits
    #     `p[...]`) avoids inventing any new URScript syntax at all - this is
    #     the exact code path that worked correctly on the very first test.
    dx, dy, dz = delta[0], delta[1], delta[2]
    x, y, z, rx, ry, rz = current_pose
    return [x + dx, y + dy, z + dz, rx, ry, rz]


def movej_script(joints, speed, accel):
    q_str = "[" + ",".join(f"{v:.6f}" for v in joints) + "]"
    return f"""def prog():
  movej({q_str}, a={accel}, v={speed})
end
prog()
"""


def movej_to_pose_script(pose, qnear, speed, accel):
    """movej to a Cartesian pose, resolved to joints ON THE ROBOT via get_inverse_kin(
    pose, qnear=...) with qnear = the arm's actual current joints (read via rtde_receive
    in Python, passed down as a plain literal - never computed as a URScript expression,
    per the existing "resolve values in Python" rule). qnear makes the IK solver pick the
    solution nearest the arm's real starting configuration, and movej's bounded
    accelerate-cruise-decelerate joint profile (see spin_base.py's v2 fix) replaces
    movel's straight-line Cartesian interpolation - which forces an unpredictable, and
    potentially very large/fast, joint sweep (worst case: the base joint) whenever the
    arm's actual current configuration is far from what a straight Cartesian line to the
    target would assume. Real 2026-07-16 incident: the arm was left ~180deg off (base
    joint) from a prior session, catch.py's movel-based approach to the wait pose tried
    to hold a straight Cartesian line through that mismatch, and the resulting base-joint
    sweep tripped a protective stop.
    """
    pose_str = "p[" + ",".join(f"{v:.6f}" for v in pose) + "]"
    qnear_str = "[" + ",".join(f"{v:.6f}" for v in qnear) + "]"
    return f"""def prog():
  q_target = get_inverse_kin({pose_str}, qnear={qnear_str})
  movej(q_target, a={accel}, v={speed})
end
prog()
"""


def check_move_size(current, target, is_joint, force):
    deltas = [t - c for t, c in zip(target, current)]
    if is_joint:
        offenders = [(i, d) for i, d in enumerate(deltas) if abs(d) > MAX_JOINT_STEP]
        limit_desc = f"{MAX_JOINT_STEP} rad per joint"
    else:
        offenders = [(i, d) for i, d in enumerate(deltas[:3]) if abs(d) > MAX_TRANSLATION_STEP]
        limit_desc = f"{MAX_TRANSLATION_STEP} m per axis"
    if offenders and not force:
        details = ", ".join(f"axis {i}: {d:+.4f}" for i, d in offenders)
        raise SystemExit(
            f"REFUSING to send: computed move exceeds the safety clamp ({limit_desc}) - {details}. "
            f"This is the exact kind of jump that caused a real E-stop on 2026-07-10 - double check "
            f"the target before overriding with --force."
        )


def wait_for_stop(rtde_r, is_joint, current=None, target=None):
    """Poll until the arm settles at near-zero speed, or MAX_WAIT elapses.

    CB3 UR10 bring-up 2026-08-24: found that on this (slower-to-start) CB3
    controller, the settle check could declare "done" up to ~0.6-0.8s before
    the robot had even begun moving - motion hadn't started yet, so speed read
    near-zero and satisfied the "stopped" streak instantly, reporting "settled
    cleanly" for a move that (as of that instant) hadn't happened yet. This
    wasn't visible on the faster-to-start e-Series box this pattern was
    originally written against. Fix: don't start counting the settle streak
    until real motion (speed > MOVE_START_SPEED) has actually been observed -
    unless the target was already reached (no motion needed), checked via
    `current`/`target` up front so a genuine no-op move doesn't just time out.
    """
    if current is not None and target is not None:
        deltas = [t - c for t, c in zip(target, current)]
        if not is_joint:
            deltas = deltas[:3]
        if max(abs(d) for d in deltas) < NOOP_TOL:
            return True
    getter = rtde_r.getActualQd if is_joint else rtde_r.getActualTCPSpeed
    slow_streak = 0
    started = False
    start = time.time()
    while time.time() - start < MAX_WAIT:
        peak = max(abs(v) for v in getter())
        if not started:
            if peak > MOVE_START_SPEED:
                started = True
        elif peak < SETTLE_SPEED:
            slow_streak += 1
            if slow_streak >= SETTLE_TICKS:
                return True
        else:
            slow_streak = 0
        time.sleep(0.05)
    return False


def main():
    parser = argparse.ArgumentParser(description="Send the UR12e to a location via raw URScript-over-socket (no ur_rtde).")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--pose", type=float, nargs=6, metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                        help="absolute target TCP pose (m, rad axis-angle)")
    group.add_argument("--relative", type=float, nargs=6, metavar=("DX", "DY", "DZ", "DRX", "DRY", "DRZ"),
                        help="TCP pose offset from current position (m, rad)")
    group.add_argument("--joints", type=float, nargs=6, metavar=("J0", "J1", "J2", "J3", "J4", "J5"),
                        help="absolute target joint angles (rad)")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help=f"m/s or rad/s (default {DEFAULT_SPEED})")
    parser.add_argument("--accel", type=float, default=DEFAULT_ACCEL, help=f"m/s^2 or rad/s^2 (default {DEFAULT_ACCEL})")
    parser.add_argument("--force", action="store_true",
                         help="bypass the safety clamp on move size - only use after double-checking the printed target")
    args = parser.parse_args()

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    is_joint = args.joints is not None

    if is_joint:
        current = rtde_r.getActualQ()
        print("before joints:", [round(v, 4) for v in current])
        target = args.joints
        check_move_size(current, target, is_joint=True, force=args.force)
        script = movej_script(target, args.speed, args.accel)
    elif args.pose is not None:
        current = rtde_r.getActualTCPPose()
        print("before pose:", [round(v, 4) for v in current])
        target = args.pose
        check_move_size(current, target, is_joint=False, force=args.force)
        script = movel_absolute_script(target, args.speed, args.accel)
    else:
        if any(v != 0.0 for v in args.relative[3:]):
            print("NOTE: --relative only applies the X/Y/Z translation in base frame; "
                  "the RX/RY/RZ components you passed are ignored (orientation is kept "
                  "unchanged) - see resolve_relative_pose comment for why.")
        current = rtde_r.getActualTCPPose()
        print("before pose:", [round(v, 4) for v in current])
        target = resolve_relative_pose(current, args.relative)
        print("computed absolute target:", [round(v, 4) for v in target])
        check_move_size(current, target, is_joint=False, force=args.force)
        script = movel_absolute_script(target, args.speed, args.accel)

    print("sending move command...", flush=True)
    send_script(script)

    print("waiting for motion to complete (or timeout)...", flush=True)
    reached_stop = wait_for_stop(rtde_r, is_joint, current=current, target=target)

    if is_joint:
        print("after joints:", [round(v, 4) for v in rtde_r.getActualQ()])
    else:
        print("after pose:", [round(v, 4) for v in rtde_r.getActualTCPPose()])
    print("settled cleanly" if reached_stop else f"WARNING: did not settle within {MAX_WAIT}s - still moving or move failed (check pendant)")

    rtde_r.disconnect()


if __name__ == "__main__":
    main()
