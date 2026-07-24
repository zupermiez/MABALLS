"""
Click-through calibration pose replay with a HARDCODED, curated pose list -
no dependency on any calibrate_frames.py output file (unlike
replay_calibration_poses.py, which this is a sibling of and shares almost all
of its safety machinery with - see that file for the fuller history of the
issues below).

Where these poses came from: started from the 25 poses in the 2026-07-20
calibration run (T_base_from_mocap.json, 4.25mm fit RMSE), then replaced the
6 that either errored outright or needed the soft-stop during a real replay
session on 2026-07-23 (original indices 6, 7, 9, 17, 18, 19 - see git history
for the exact log). Diagnosis: the failures clustered at small radial reach
from the base (poses 9 and 19 were r=0.22m and r=0.19m from the base Z axis -
right in among whatever is mounted near the base) and one at negative base-Z
(pose 18, below the base plane). The replacements deliberately favor larger
reach (~0.6-1.0m) and higher/positive Z (~0.6-1.0m above the base), reusing
the ORIENTATION of a nearby-azimuth pose that's already proven reachable
(position drives the Umeyama fit; a proven orientation nearby is a reasonable
starting guess for reachability, not a formal guarantee). This is a
heuristic, not a proof - r/z aren't a perfect predictor (a couple of the
kept original poses have small r too and were fine), which is exactly why
this script keeps every safety mechanism below rather than trusting the
poses blindly. Validate these for real before trusting them: run this,
soft-stop/skip anything that looks wrong, and treat repeated failures at the
same pose as a sign to edit POSES below, not to retry harder.

Round 2 (2026-07-23, second real run): poses 6/7/9/18/19 from round 1 all
succeeded. Newly replaced: 15, 17, 24 - all three failed on the VIA-POSE leg
(not the pose's own target), either as a slow settle timeout (settled=False,
modest pos_err 40-675mm - plausibly just MAX_WAIT=15s being too short for a
large joint excursion at this script's conservative default speed, not
necessarily a near-collision) or, for 17, an actual soft stop. Caveat worth
knowing: the failing leg each time was the return trip FROM THE PRECEDING
POSE (14, 16, 23 respectively) TO the via-pose - so the real culprit may be
those predecessor poses' landing configuration, not 15/17/24's own targets.
Replacing 15/17/24 changes what comes right after the same predecessors, so
if the same via-pose leg keeps failing again, try editing pose 14/16/23
instead (or lowering --speed/--accel, or raising ur_goto_raw.MAX_WAIT) rather
than re-replacing 15/17/24 again.

Round 2 also flagged 6 and 19: both reached their own target fine on a
retry, but the via-pose leg preceding them needed that retry (not a clean
first attempt) - replaced again on the policy that ANY did-not-reach event,
even one a retry later resolved, marks a pose as unreliable rather than
proven. Per the caveat above, this may really be about poses 5/18's landing
configuration (the predecessors of 6/19) rather than 6/19's own targets - if
the new points still need a retry, that's the next place to look.

Round 3 (2026-07-24, third real run): the predicted recurrence happened -
7 different via-pose legs failed (predecessors 5, 6, 14, 15, 16, 18, 23),
including one that needed a manual SOFT STOP (pos_err=1738mm). Root cause
identified: every via-pose move calls get_inverse_kin(via_pose,
qnear=<arm's live joints>), and qnear is different every time (whatever
branch the PRECEDING pose's own IK solve happened to land in). Since
get_inverse_kin picks the solution nearest THAT PARTICULAR qnear per joint,
the via-pose - despite being a single fixed Cartesian target - can resolve to
a completely different joint branch move to move, occasionally one far
enough away to require a huge/fast joint excursion (the "wrist or base spins
360deg" symptom) or blow through MAX_WAIT. Replacing "weird" poses (rounds 1
and 2) only ever relocates which predecessor triggers this - it doesn't fix
it, which is why it came back a third time.

Fix: qnear for the via-pose is now cached, not re-read from the arm's live
joints, every time. The first successful via-pose arrival in a session has
its joints (wrap_to_pi'd) stashed as `via_qnear_cache` and reused as the IK
seed for every subsequent via-pose move - so the via-pose always resolves to
the SAME branch for the rest of the session, regardless of what branch the
previous pose left the arm in. Calibration-pose targets are unaffected (they
still seed from the arm's live joints, which is correct - each is only
visited once, seeded from wherever the arm legitimately is).

Same operating model as replay_calibration_poses.py:
- Press Enter to move to the next pose, 'q' to stop early, SPACE at any point
  during a move to soft-stop it (uploads stopj() to the robot's secondary
  interface, preempting whatever movej is running - no protective stop / E-stop
  needed for a self-collision that's visibly about to happen).
- Every move is routed through a fixed, known-clear via-pose (default:
  catch.py's DEFAULT_WAIT_POSE) instead of jumping directly between two
  arbitrary poses - reduces (does not eliminate) the chance of a wild IK
  branch swing sweeping the tool through the arm itself, since movej has no
  self-collision checking of its own.
- qnear for get_inverse_kin is built from wrap_to_pi(getActualQ()), not the
  raw current joint value, so repeated moves can't ratchet a joint's
  numeric representation toward its physical +-360deg limit.
- Arrival is independently verified (position + orientation tolerance), not
  inferred from speed settling to ~0 alone - a protective stop also freezes
  speed at ~0, which looks identical to "arrived" otherwise. A move that
  didn't actually get there (fault, soft stop, or no reachable/safe IK
  branch) stops the script and offers retry/skip/quit - it never silently
  continues to the next pose.
- Dashboard calls survive the pendant being switched to Local Control
  (reconnect-and-retry instead of the uncaught "Connection reset by peer"
  crash seen for real on 2026-07-23) - the script waits for Remote Control
  and safety mode NORMAL before/after every move instead of dying.

Edit POSES below directly to drop/add/adjust points - there's no file to keep
in sync.
"""
import argparse
import contextlib
import os
import select
import sys
import termios
import time
import tty

import dashboard_client
import numpy as np
import rtde_receive

from calibrate_frames import apply_and_verify_tcp
from catch import DEFAULT_WAIT_POSE
from ur_goto_raw import (movej_to_pose_script, send_script, ROBOT_IP,
                          MAX_WAIT, SETTLE_SPEED, SETTLE_TICKS)

# Box-centroid TCP offset (matches calibrate_frames.py's current TCP_OFFSET) -
# these poses were recorded/derived against this offset, so it must be active
# for the movej targets to land on the same physical points.
TCP_OFFSET = (0.0, 0.0, 0.0725, 0.0, 0.0, 0.0)

# x, y, z (m), rx, ry, rz (rad) - box-centroid TCP pose in the robot base frame.
# See module docstring for where these came from and which 6 were replaced.
POSES = [
    (0.0318, -1.3281, 0.3507, 0.1054, -0.5756, 0.2629),    # 1
    (-0.198, -0.4602, -0.0684, 1.723, -0.2364, -0.3137),   # 2
    (-0.672, -0.9757, -0.2435, 0.3459, -1.4385, -0.6411),  # 3
    (-0.139, -0.0766, 1.2017, -2.2869, 0.0267, -0.1306),   # 4
    (-1.1329, -0.3032, 0.0451, 1.8591, -1.4872, -1.2541),  # 5
    (-0.6511, 0.5464, 0.85, 0.216, -0.14, -1.83),          # 6  replaced again (round-1 point needed a via-pose retry)
    (0.7987, 0.2907, 0.9, -1.549, 0.564, 0.626),           # 7  replaced 2026-07-24 (round-1 point at az=60deg
                                                              #    was inside the control-box keepout wedge -
                                                              #    same reach/z/orientation, rotated to az=20deg)
    (0.6239, 0.4823, 1.1776, -1.5488, 0.5639, 0.6261),     # 8
    (0.6364, 0.6364, 0.95, -1.549, 0.564, 0.626),          # 9  replaced (was r=0.22, very close to base)
    (0.5719, -0.0915, 0.8851, -0.4125, -0.7137, 1.4582),   # 10
    (0.118, -0.1464, 0.6092, -0.6301, -0.2513, 0.2998),    # 11
    (0.7439, -1.0139, 0.4146, 0.1102, -0.5492, 0.7234),    # 12
    (-0.2432, -0.2923, -0.246, 0.1583, -1.9716, -2.3491),  # 13
    (-0.4279, -0.812, 0.5863, 2.4177, -1.0533, 0.0359),    # 14
    (-0.425, -0.7361, 0.75, 2.4177, -1.0533, 0.0359),      # 15 replaced 2026-07-23 run 2 (skipped: via-pose settle failed)
    (-0.763, -0.5594, -0.475, 0.7358, -0.2813, -0.8986),   # 16
    (0.45, -0.7794, 0.75, 0.11, -0.549, 0.723),            # 17 replaced 2026-07-23 run 2 (soft-stopped, skipped)
    (-0.8457, -0.3078, 0.6, 1.859, -1.487, -1.254),        # 18 replaced (was z=-0.20, below base)
    (0.0, -0.85, 0.8, 0.105, -0.576, 0.263),               # 19 replaced again (round-1 point needed a via-pose retry)
    (0.1779, -0.3689, 1.1401, -1.0791, -1.6152, 1.9368),   # 20
    (-0.2892, 0.2834, 0.4916, -1.092, -1.4774, 0.6974),    # 21
    (-0.4768, -0.1359, -0.1622, 1.3692, -0.9431, -0.641),  # 22
    (-1.2201, 0.2229, 0.0502, 0.2159, -0.1398, -1.8296),   # 23
    (-0.5541, -0.7092, 0.8, 0.158, -1.972, -2.349),        # 24 replaced 2026-07-23 run 2 (skipped: via-pose settle failed)
    (-0.4223, -0.8801, 0.2017, 0.8143, -0.2096, -0.5174),  # 25
]

DEFAULT_SPEED = 0.5   # rad/s - supervised replay, not a race; tune with --speed
DEFAULT_ACCEL = 1.0   # rad/s^2
POS_TOL = 0.01         # m - arrival check
ROT_TOL = 0.05         # rad - arrival check (rotation vector norm difference)
JOINT_LIMIT_ASSUMED = 2 * np.pi   # rad - UR default per-joint software limit (+-360deg)
JOINT_LIMIT_MARGIN = np.radians(60)   # warn once a joint is within this of the assumed limit
STOP_KEY = " "         # soft-stop key, same convention as jog_ur_raw.py
STOP_DECEL = 2.0       # rad/s^2 - joint deceleration for the soft stop

# Control-box keepout (2026-07-24, real repeated near-collisions requiring a manual
# soft stop - see module docstring's Round 3 note). User-confirmed geometry: the box
# sits ~180deg from the wait pose's own azimuth (opposite side of the base), ~30cm
# below the base motor, ~45cm wide - radial distance from the base axis wasn't
# measured, so this errs wide (40deg half-width) rather than precise. ENDPOINT CHECK
# ONLY: there's no local forward-kinematics/path-simulation in this codebase, so this
# can't see a dangerous SWEEP between two individually-safe endpoints (which is what
# actually happened - both pose 16 and the via-pose sit outside this band, but the
# joint-space path between them, before the via-pose qnear-caching fix above, could
# still cross it). Treat this as one more layer, not a substitute for watching the
# arm / keeping SPACE ready near the pose-16-to-via-pose transition specifically.
BOX_KEEPOUT_HALF_WIDTH_DEG = 40.0


def pose_azimuth_deg(pose):
    return float(np.degrees(np.arctan2(pose[1], pose[0])))


def wrap_deg_pm180(d):
    return ((d + 180.0) % 360.0) - 180.0


def check_keepout(target_pose, box_az_deg, half_width_deg=BOX_KEEPOUT_HALF_WIDTH_DEG):
    """None if target_pose's azimuth safely clears the control box, else a reason
    string. See BOX_KEEPOUT_HALF_WIDTH_DEG above for what this does and doesn't catch."""
    tgt_az = pose_azimuth_deg(target_pose)
    d = wrap_deg_pm180(tgt_az - box_az_deg)
    if abs(d) <= half_width_deg:
        return (f"azimuth {tgt_az:.0f}deg is within {half_width_deg:.0f}deg of the control box "
                f"(~{box_az_deg:.0f}deg, ~180deg from the via-pose) - refusing to send, see "
                f"BOX_KEEPOUT_HALF_WIDTH_DEG in the source")
    return None


def stopj_script(decel):
    return f"""def prog():
  stopj({decel})
end
prog()
"""


@contextlib.contextmanager
def cbreak_mode(fd):
    """Temporarily puts the terminal in cbreak mode (single keypresses visible
    immediately, no Enter needed) - same technique as jog_ur_raw.py. Only used
    around the "wait for the move to finish" window, not around input()
    prompts, so normal line-editing keeps working everywhere else."""
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def read_keys_nonblocking(fd):
    keys = []
    while select.select([fd], [], [], 0)[0]:
        chunk = os.read(fd, 1)
        if not chunk:
            break
        keys.append(chunk.decode(errors="ignore"))
    return keys


def wait_for_stop_or_soft_stop(rtde_r):
    """Like ur_goto_raw.wait_for_stop, but also watches stdin (in cbreak mode)
    for STOP_KEY every tick - if pressed, immediately uploads stopj_script()
    to the robot's secondary interface, which PREEMPTS whatever movej is
    currently running (a fresh script upload always replaces the running
    program - same mechanism catch.py's re-aim preemption uses) and
    decelerates it to a clean stop, no protective stop / E-stop needed for a
    self-collision that's visibly about to happen. Returns (settled,
    soft_stopped)."""
    fd = sys.stdin.fileno()
    slow_streak = 0
    soft_stopped = False
    start = time.time()
    with cbreak_mode(fd):
        while time.time() - start < MAX_WAIT:
            for key in read_keys_nonblocking(fd):
                if key == STOP_KEY and not soft_stopped:
                    print("\n  SOFT STOP - sending stopj()...")
                    send_script(stopj_script(STOP_DECEL))
                    soft_stopped = True
            speed = rtde_r.getActualQd()
            if max(abs(v) for v in speed) < SETTLE_SPEED:
                slow_streak += 1
                if slow_streak >= SETTLE_TICKS:
                    return True, soft_stopped
            else:
                slow_streak = 0
            time.sleep(0.05)
    return False, soft_stopped


def wrap_to_pi(angles):
    """Canonicalize each joint angle into (-pi, pi] - see module docstring's
    "Joint wind-up" note in replay_calibration_poses.py. Used to build the
    qnear handed to get_inverse_kin so the IK solver's per-joint "nearest"
    search can't ratchet a joint's value toward its physical limit across a
    sequence of moves."""
    return [((a + np.pi) % (2 * np.pi)) - np.pi for a in angles]


def warn_if_near_joint_limit(joints):
    for i, q in enumerate(joints):
        if abs(q) > JOINT_LIMIT_ASSUMED - JOINT_LIMIT_MARGIN:
            print(f"  WARNING: joint {i} = {np.degrees(q):.1f}deg is within "
                  f"{np.degrees(JOINT_LIMIT_MARGIN):.0f}deg of the assumed +-360deg limit.")


def reconnect_dashboard(dash):
    """Switching the pendant to Local Control kicks the external Dashboard
    Server client - the next call on `dash` then raises. Retries connect()
    until it succeeds - it won't while still in Local Control, so this loop
    is also what makes the script just wait instead of dying."""
    while True:
        try:
            dash.disconnect()
        except Exception:
            pass
        try:
            dash.connect()
            return
        except Exception:
            time.sleep(1.0)


def dash_call(dash, method_name, *a):
    """Call a dashboard_client method, transparently reconnecting (see
    reconnect_dashboard) if the connection was dropped - every dashboard read
    in this script goes through here instead of calling `dash` directly."""
    while True:
        try:
            return getattr(dash, method_name)(*a)
        except Exception as e:
            print(f"\n!!! dashboard connection lost ({e}) - probably switched to Local Control on "
                  f"the pendant. Reconnecting...")
            reconnect_dashboard(dash)


def wait_for_remote_and_normal(dash):
    """Blocks until the robot is back in Remote Control AND safety mode is
    NORMAL, tolerating the dashboard connection itself dropping meanwhile
    (see dash_call/reconnect_dashboard) - so switching to Local Control, by
    accident or on purpose, pauses this script instead of crashing it."""
    remote = bool(dash_call(dash, "isInRemoteControl"))
    if not remote:
        print("\n!!! robot is in Local Control, not Remote Control - switch the pendant back to "
              "Remote; this will continue automatically once it does...")
    while not remote:
        time.sleep(1.0)
        remote = bool(dash_call(dash, "isInRemoteControl"))

    mode = dash_call(dash, "safetymode").strip().rsplit(":", 1)[-1].strip().upper()
    if mode != "NORMAL":
        print(f"\n!!! robot safety mode is {mode}, not NORMAL - clear it on the pendant, this will "
              f"continue automatically once it's back to NORMAL...")
    while mode != "NORMAL":
        time.sleep(1.0)
        mode = dash_call(dash, "safetymode").strip().rsplit(":", 1)[-1].strip().upper()


def move_and_verify(rtde_r, dash, target_pose, speed, accel, label, qnear=None,
                     box_az_deg=None, box_half_width_deg=BOX_KEEPOUT_HALF_WIDTH_DEG):
    """Send one movej-to-pose, then independently confirm the arm actually got
    there (position + orientation within tolerance) rather than trusting
    speed-only settling - a protective stop freezes speed at ~0 too, which
    looks identical to "arrived" otherwise. `qnear` defaults to the arm's live
    joints (previous behavior); pass a cached value for a repeated fixed-pose
    target (the via-pose) so get_inverse_kin always resolves the same branch -
    see module docstring's Round 3 note. `box_az_deg`, if set, refuses to send
    when target_pose's own azimuth is within `box_half_width_deg` of it - see
    BOX_KEEPOUT_HALF_WIDTH_DEG's docstring for what this does and doesn't
    catch. Returns (ok, actual_pose, actual_joints)."""
    wait_for_remote_and_normal(dash)  # don't even send if Local Control / a fault is already active

    if box_az_deg is not None:
        reason = check_keepout(target_pose, box_az_deg, box_half_width_deg)
        if reason is not None:
            print(f"  REFUSING to send {label}: {reason}")
            return False, rtde_r.getActualTCPPose(), rtde_r.getActualQ()

    if qnear is None:
        qnear = wrap_to_pi(rtde_r.getActualQ())  # see module docstring's "Joint wind-up" note
    send_script(movej_to_pose_script(target_pose, qnear, speed, accel))
    settled, soft_stopped = wait_for_stop_or_soft_stop(rtde_r)
    wait_for_remote_and_normal(dash)  # blocks here until any fault/Local-Control switch clears

    actual_joints = rtde_r.getActualQ()
    actual = rtde_r.getActualTCPPose()
    pos_err = float(np.linalg.norm(np.array(actual[:3]) - np.array(target_pose[:3])))
    rot_err = float(np.linalg.norm(np.array(actual[3:6]) - np.array(target_pose[3:6])))
    ok = settled and not soft_stopped and pos_err < POS_TOL and rot_err < ROT_TOL
    if not ok:
        reason = "soft-stopped by operator" if soft_stopped else f"settled={settled}"
        print(f"  DID NOT REACH {label}: pos_err={pos_err * 1000:.1f}mm rot_err={rot_err:.3f}rad "
              f"({reason}) - at {[round(v, 4) for v in actual]}")
    else:
        print(f"  at {label}: {[round(v, 4) for v in actual]}")
    warn_if_near_joint_limit(actual_joints)
    return ok, actual, actual_joints


def move_with_retry(rtde_r, dash, target_pose, speed, accel, label, qnear=None,
                     box_az_deg=None, box_half_width_deg=BOX_KEEPOUT_HALF_WIDTH_DEG):
    """Loop move_and_verify until it succeeds, or the operator chooses to skip
    or quit - never silently continues past a move that didn't actually land,
    unlike a plain settle+safety-mode check would. Returns (status,
    actual_joints) - actual_joints is only meaningful when status == "ok"."""
    while True:
        ok, actual, actual_joints = move_and_verify(rtde_r, dash, target_pose, speed, accel, label,
                                                      qnear=qnear, box_az_deg=box_az_deg,
                                                      box_half_width_deg=box_half_width_deg)
        if ok:
            return "ok", actual_joints
        choice = input("  retry (r), skip this pose (s), or quit (q)? [r] > ").strip().lower()
        if choice == "q":
            return "quit", actual_joints
        if choice == "s":
            return "skip", actual_joints
        # anything else (including empty) retries


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED,
                         help=f"movej joint speed, rad/s (default {DEFAULT_SPEED})")
    parser.add_argument("--accel", type=float, default=DEFAULT_ACCEL,
                         help=f"movej joint accel, rad/s^2 (default {DEFAULT_ACCEL})")
    parser.add_argument("--start-at", type=int, default=1,
                         help="1-based pose index to start from (resume after quitting early)")
    parser.add_argument("--skip-set-tcp", action="store_true",
                         help="don't send set_tcp() - use if the box-centroid TCP is already active")
    parser.add_argument("--via-pose", type=float, nargs=6, default=list(DEFAULT_WAIT_POSE),
                         metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                         help="known-clear intermediate pose every move routes through, to reduce "
                              f"self-collision risk (default: catch.py's DEFAULT_WAIT_POSE {DEFAULT_WAIT_POSE})")
    parser.add_argument("--no-via-pose", action="store_true",
                         help="jump directly pose-to-pose instead - only the earlier, higher "
                              "self-collision-risk behavior; not recommended, see module docstring")
    parser.add_argument("--box-keepout-half-width-deg", type=float, default=BOX_KEEPOUT_HALF_WIDTH_DEG,
                         help="refuse any target within this many degrees of the control box's azimuth "
                              f"(default {BOX_KEEPOUT_HALF_WIDTH_DEG:.0f}, ~180deg from --via-pose) - "
                              "endpoint check only, see BOX_KEEPOUT_HALF_WIDTH_DEG in the source")
    parser.add_argument("--no-box-keepout", action="store_true",
                         help="disable the control-box keepout check entirely - not recommended, "
                              "see BOX_KEEPOUT_HALF_WIDTH_DEG in the source")
    args = parser.parse_args()

    n = len(POSES)
    if not (1 <= args.start_at <= n):
        raise SystemExit(f"--start-at must be between 1 and {n}")

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    dash = dashboard_client.DashboardClient(ROBOT_IP)
    dash.connect()

    try:
        wait_for_remote_and_normal(dash)  # tolerate starting up while in Local Control too

        if not args.skip_set_tcp:
            apply_and_verify_tcp(rtde_r, TCP_OFFSET)
        else:
            print("Skipping set_tcp() - assuming the box-centroid TCP is already active.")

        via_pose = None if args.no_via_pose else list(args.via_pose)
        if via_pose is not None:
            print(f"Routing every move through the via-pose {[round(v, 3) for v in via_pose]} "
                  f"(--no-via-pose to disable - not recommended, see module docstring).")

        box_az_deg = None
        if not args.no_box_keepout:
            box_az_deg = wrap_deg_pm180(pose_azimuth_deg(args.via_pose) + 180.0)
            print(f"Control-box keepout ON: refusing any target within "
                  f"{args.box_keepout_half_width_deg:.0f}deg of azimuth {box_az_deg:.0f}deg "
                  f"(~180deg from --via-pose) - endpoint check only, see BOX_KEEPOUT_HALF_WIDTH_DEG "
                  f"in the source (--no-box-keepout to disable, not recommended).")
            flagged = [i for i, p in enumerate(POSES, start=1)
                       if check_keepout(p, box_az_deg, args.box_keepout_half_width_deg) is not None]
            if flagged:
                print(f"  NOTE: pose(s) {flagged} already fall inside the keepout wedge and will be "
                      f"refused when reached - fix POSES or pass --no-box-keepout/a smaller half-width.")

        print(f"\nLoaded {n} hardcoded poses. Replaying from pose {args.start_at}.")
        print("Press Enter to move to the next pose, 'q' to stop early, SPACE during a move to "
              "soft-stop it.\n")

        via_qnear_cache = None  # set from the first clean via-pose arrival - see module docstring's Round 3 note
        for i in range(args.start_at, n + 1):
            pose = POSES[i - 1]
            line = input(f"[{i}/{n}] Enter to move to "
                         f"({pose[0]:+.3f},{pose[1]:+.3f},{pose[2]:+.3f}) ... > ").strip().lower()
            if line == "q":
                print("Stopped early.")
                break

            if via_pose is not None:
                result, via_actual_joints = move_with_retry(rtde_r, dash, via_pose, args.speed, args.accel,
                                                              "via-pose", qnear=via_qnear_cache,
                                                              box_az_deg=box_az_deg,
                                                              box_half_width_deg=args.box_keepout_half_width_deg)
                if result == "quit":
                    print("Stopped early.")
                    break
                if result == "skip":
                    continue  # skip straight to the next pose's prompt
                if via_qnear_cache is None:
                    via_qnear_cache = wrap_to_pi(via_actual_joints)
                    print(f"  (cached via-pose IK seed: {[round(v, 4) for v in via_qnear_cache]} - every "
                          f"future via-pose move reuses this so it always resolves the same joint branch)")

            result, _ = move_with_retry(rtde_r, dash, pose, args.speed, args.accel, f"pose {i}",
                                         box_az_deg=box_az_deg, box_half_width_deg=args.box_keepout_half_width_deg)
            if result == "quit":
                print("Stopped early.")
                break

        print("\nDone.")
    finally:
        try:
            dash.disconnect()
        except Exception:
            pass
        rtde_r.disconnect()


if __name__ == "__main__":
    main()
