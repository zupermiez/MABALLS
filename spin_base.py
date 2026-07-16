"""
spin_base.py - sweep the UR12e's base joint (J1) back and forth as fast as possible,
to see/measure how fast that looks. Not a catch tool - standalone speed
visualization, nothing else.

v2 (2026-07-16): v1 spun continuously in one direction with speedj() and hit a real
protective stop at 359 degrees of travel - that's a JOINT POSITION LIMIT fault, not a
speed fault (constant-speed motion in one direction was always going to walk into the
joint's range limit eventually, however slow). Fixed two ways at once by switching to
a bounded movej() sweep between two endpoints straddling the joint's STARTING position
(read live via RTDE, not assumed) rather than an open-ended spin:
  1. Bounded and inset from the limits - default total sweep width is 180 degrees
     (+-90 deg from wherever J1 started), which leaves a huge margin versus the ~359
     degrees of one-directional travel that was needed to fault last time.
  2. movej()'s own trapezoidal profile decelerates to a stop AT each endpoint before
     reversing - "full speed at the ends" simply doesn't happen, it's what the
     accelerate-cruise-decelerate profile means; no separate ramp logic needed.

Same self-stopping principle as jog_ur_raw.py/v1: every sent program is bounded and
does not depend on this process staying alive - a movej() naturally stops when it
arrives, and Ctrl-C sends an explicit stopj() on top. Additionally checks
getSafetyMode() (not just joint speed) while waiting for each leg to arrive, because
a joint frozen by a fault reads the same near-zero speed as one that arrived normally
- this project already hit that exact false-positive once in catch.py's move_to()
(see docs/debug_log.md 2026-07-15) and it applies here identically. On a detected
fault this script stops and reports it - it does NOT try to auto-clear and keep
sweeping, unlike catch.py's fault recovery: retrying the same motion that just faulted
near a limit is exactly the wrong instinct for a script whose whole point is probing
how far/fast it can go.
"""

import argparse
import math
import time

import rtde_receive

from ur_goto_raw import ROBOT_IP, movej_script, send_script

MAX_BASE_SPEED = 2.0944   # rad/s = 120 deg/s (CLAUDE.md hardware section)
DEFAULT_ACCEL = 3.0       # rad/s^2 - unverified for joint space (see module docstring
                           # in v1 history), controller assumed to clamp internally
SETTLE_SPEED = 0.01       # rad/s - below this on J1 counts as "arrived"
SETTLE_TICKS = 3          # consecutive slow polls required before declaring arrival
POLL_INTERVAL = 0.05      # s
ARRIVAL_TIMEOUT = 5.0     # s - safety backstop per leg

SAFETY_MODE_NORMAL = 1  # ur_rtde Robot State enum: 1=NORMAL, anything else needs a human


def check_fault(rtde_r) -> "str | None":
    mode = rtde_r.getSafetyMode()
    if mode != SAFETY_MODE_NORMAL:
        return f"safety_mode={mode} (not NORMAL) - robot is stopped/faulted"
    return None


def stop_script(accel: float) -> str:
    return f"""def prog():
  stopj({accel:.4f})
end
prog()
"""


def wait_for_arrival(rtde_r, label: str, speed: float, accel: float):
    """Poll until J1 settles near-zero speed, or a fault/timeout is hit.
    Returns (arrived: bool, fault: Optional[str])."""
    slow_streak = 0
    start = time.time()
    while time.time() - start < ARRIVAL_TIMEOUT:
        fault = check_fault(rtde_r)
        if fault is not None:
            return False, fault
        q1 = rtde_r.getActualQ()[0]
        qd1 = rtde_r.getActualQd()[0]
        print(f"\r{label}: J1={math.degrees(q1):+7.2f} deg   speed={math.degrees(qd1):+7.1f} deg/s   ",
              end="", flush=True)
        if abs(qd1) < SETTLE_SPEED:
            slow_streak += 1
            if slow_streak >= SETTLE_TICKS:
                return True, None
        else:
            slow_streak = 0
        time.sleep(POLL_INTERVAL)
    return False, None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--sweep-degrees", type=float, default=180.0,
                         help="total sweep width in degrees, centered on J1's position at startup (default 180)")
    parser.add_argument("--speed", type=float, default=MAX_BASE_SPEED,
                         help=f"rad/s for J1 (default {MAX_BASE_SPEED:.4f} = 120 deg/s, the documented max)")
    parser.add_argument("--accel", type=float, default=DEFAULT_ACCEL, help=f"rad/s^2 (default {DEFAULT_ACCEL})")
    parser.add_argument("--duration", type=float, default=None,
                         help="stop automatically after N seconds (default: run until Ctrl-C)")
    parser.add_argument("--robot-ip", default=ROBOT_IP, help="UR12e controller IP")
    args = parser.parse_args()

    rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    current_q = list(rtde_r.getActualQ())
    half_range = math.radians(args.sweep_degrees) / 2.0
    center = current_q[0]

    q_a = list(current_q)
    q_a[0] = center - half_range
    q_b = list(current_q)
    q_b[0] = center + half_range

    print(f"Base sweep: {args.sweep_degrees:.0f} deg wide, centered on J1's current position "
          f"({math.degrees(center):+.1f} deg) -> endpoints {math.degrees(q_a[0]):+.1f} deg / "
          f"{math.degrees(q_b[0]):+.1f} deg.")
    print(f"speed={args.speed:.4f} rad/s ({math.degrees(args.speed):.1f} deg/s)  accel={args.accel} rad/s^2")
    print("*** Make sure the workspace is clear - the arm WILL sweep back and forth. ***")
    input("Press Enter to start, Ctrl-C to stop once running...")

    targets = [q_a, q_b]
    idx = 0
    start = time.time()
    try:
        while args.duration is None or time.time() - start < args.duration:
            target = targets[idx % 2]
            leg_start = time.time()
            send_script(movej_script(target, args.speed, args.accel))
            arrived, fault = wait_for_arrival(rtde_r, f"leg {idx + 1}", args.speed, args.accel)
            if fault is not None:
                print(f"\n\n!!! FAULT DETECTED: {fault}")
                print("Stopping - not retrying. Clear it (ur_status.py --clear or the pendant), "
                      "confirm J1's actual position, before running this again.")
                send_script(stop_script(args.accel))
                return
            leg_time = time.time() - leg_start
            print(f"  -> arrived (leg {leg_time:.2f}s)" if arrived else "  -> timed out waiting to arrive")
            idx += 1
    except KeyboardInterrupt:
        pass
    finally:
        send_script(stop_script(args.accel))
        rtde_r.disconnect()
        print("\nStopped.")


if __name__ == "__main__":
    main()
