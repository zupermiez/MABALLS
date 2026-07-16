import argparse
import time

import rtde_receive

from ur_goto_raw import (
    ROBOT_IP,
    check_move_size,
    movel_absolute_script,
    send_script,
    wait_for_stop,
)


def distance(a, b):
    return sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])) ** 0.5


def move_to(rtde_r, target, speed, accel, current, force):
    check_move_size(current, target, is_joint=False, force=force)
    send_script(movel_absolute_script(target, speed, accel))
    settled = wait_for_stop(rtde_r, is_joint=False)
    return rtde_r.getActualTCPPose(), settled


def main():
    parser = argparse.ArgumentParser(
        description="Loop the UR12e between two fixed TCP poses to test speed. "
                    "Get pose values with ur_get_pose.py. Ctrl-C stops after the "
                    "current move finishes - the robot just holds there, no further "
                    "commands are sent."
    )
    parser.add_argument("--pose-a", type=float, nargs=6, required=True, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--pose-b", type=float, nargs=6, required=True, metavar=("X", "Y", "Z", "RX", "RY", "RZ"))
    parser.add_argument("--speed", type=float, default=0.1, help="m/s - start low, increase between runs (default 0.1)")
    parser.add_argument("--accel", type=float, default=0.3, help="m/s^2 (default 0.3)")
    parser.add_argument("--pause", type=float, default=0.3, help="seconds to dwell at each endpoint (default 0.3)")
    parser.add_argument("--cycles", type=int, default=0, help="number of A<->B round trips, 0 = run until Ctrl-C (default 0)")
    parser.add_argument("--force-start", action="store_true",
                         help="the safety clamp (see ur_goto_raw.py) will likely refuse the very first move to "
                              "pose A, since A is probably a real distance away from wherever the robot happens "
                              "to be sitting right now - pass this once you've confirmed pose A is correct")
    args = parser.parse_args()

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    ab_dist = distance(args.pose_a, args.pose_b)
    print(f"A<->B straight-line distance: {ab_dist:.3f} m")
    print(f"speed={args.speed} m/s, accel={args.accel} m/s^2 "
          f"-> naive leg time ~{ab_dist / args.speed:.2f}s (ignoring accel ramp)")

    current = rtde_r.getActualTCPPose()
    print("moving to start (pose A)...")
    current, settled = move_to(rtde_r, args.pose_a, args.speed, args.accel, current, force=args.force_start)
    print("at pose A:", [round(v, 4) for v in current], "- settled" if settled else "- WARNING: did not settle")

    cycle = 0
    try:
        while args.cycles == 0 or cycle < args.cycles:
            time.sleep(args.pause)
            t0 = time.time()
            current, settled = move_to(rtde_r, args.pose_b, args.speed, args.accel, current, force=True)
            dt = time.time() - t0
            print(f"cycle {cycle + 1}: A->B in {dt:.2f}s" + ("" if settled else " - WARNING: did not settle"))

            time.sleep(args.pause)
            t0 = time.time()
            current, settled = move_to(rtde_r, args.pose_a, args.speed, args.accel, current, force=True)
            dt = time.time() - t0
            print(f"cycle {cycle + 1}: B->A in {dt:.2f}s" + ("" if settled else " - WARNING: did not settle"))

            cycle += 1
    except KeyboardInterrupt:
        print("\nStopping - robot finishes its current move and holds, no further commands sent.")
    finally:
        rtde_r.disconnect()


if __name__ == "__main__":
    main()
