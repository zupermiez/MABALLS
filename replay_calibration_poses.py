"""
Re-visits the exact TCP poses recorded in a previous calibrate_frames.py run
(the fit's own samples[].tcp_pose) so a recalibration doesn't require walking
the arm through 20-40 poses by hand again - just stand at the pendant and
press Enter to retrace last time's pose set autonomously.

Only valid if the box hasn't moved on the flange since that run (these are
box-centroid TCP poses, see calibrate_frames.py) - if the mounting changed,
walk fresh poses by hand instead.

**Joint wind-up (found the hard way 2026-07-23):** get_inverse_kin(qnear=...)
picks the solution nearest qnear PER JOINT, including how many extra full
turns that joint's value carries (e.g. -350deg and +10deg are the same
physical orientation but different joint values) - if qnear is built straight
from the arm's raw current joint value every time, a joint can slowly drift
toward its actual limit (UR default is +-360deg) over a sequence of moves
even though no single move needed that. Fix: qnear is built from
wrap_to_pi(getActualQ()) - each joint's current value canonicalized into
(-pi, pi] before being handed to get_inverse_kin as the "nearest" reference.
This does NOT change which physical branch (elbow up/down, wrist flip) gets
picked - those alternatives differ by roughly pi at a given joint, well
within one period, so canonicalizing away only the excess whole turns leaves
branch selection intact - it just stops the numeric representation from
ratcheting toward the limit. Every move also warns (non-fatal) if the arm's
actual joints land within JOINT_LIMIT_MARGIN of the assumed +-360deg limit.

**Local Control disconnect (found the hard way 2026-07-23):** switching the
pendant to Local Control kicks the external Dashboard Server client, which
otherwise crashed this script outright with an uncaught "Connection reset by
peer". All dashboard calls now go through a reconnect-and-retry wrapper, and
the script waits for BOTH Remote Control and safety mode NORMAL before
sending or confirming a move - so flipping to Local Control (on purpose or by
an accidental Enter) pauses the script instead of killing it; flip back to
Remote and it continues on its own.

**Self-collision risk, and why it happens (found the hard way 2026-07-23):**
movej has NO self-collision checking of any kind - UR's safety system is
reactive (protective stop after unexpected joint torque), not a path planner
that checks the tool/arm against its own geometry before moving. On top of
that, get_inverse_kin(qnear=...) picks the IK branch NEAREST THE ARM'S
CURRENT JOINTS, not the branch that was actually used when a given pose was
originally reached by hand. Since the 20-40 calibration poses were reached by
unconstrained freedrive (different elbow-up/down, wrist-flip choices each
time), a direct joint-space movej from wherever the arm ended up after the
previous pose to the next pose's nearest IK branch has no reason to avoid
sweeping the tool/arm through itself. Slowing --speed/--accel down only
reduces impact force if that happens - it does NOT prevent it, since this is
a geometry problem, not a dynamics one.

Mitigation (not a guarantee): every move is routed THROUGH a fixed,
known-clear intermediate pose (--via-pose, defaults to catch.py's
DEFAULT_WAIT_POSE - already validated as a safe, central, non-singular
configuration) instead of jumping directly between two arbitrary calibration
poses. Both hops (current->via, via->target) start or end at a pose that's
already known to be clear, which is far less likely to produce a wild branch
swing than an arbitrary-to-arbitrary jump - but it is still a straight joint
interpolation with no path collision-checking, so watch the arm and be ready
to hit SPACE (see below) - or the pendant's stop - the moment it looks wrong.

**Soft stop: press SPACE at any time while a move is running** to abort it
without reaching for the physical E-stop. It uploads a stopj() script to the
robot's secondary interface, which preempts whatever movej is currently
running (any new script upload replaces the running program - the same
mechanism catch.py's re-aim preemption uses) and decelerates cleanly, no
protective stop involved. The interrupted move is then reported as not
reached and you get the usual retry/skip/quit choice.

Moves via movej_to_pose_script (ur_goto_raw.py): IK is solved robot-side
against qnear = the arm's actual current joints (canonicalized, see "Joint
wind-up" above), so each move is a bounded joint move from wherever the arm
actually is, not a straight-line movel
Cartesian sweep (see that function's docstring - movel between far-apart
poses is the known base-joint-sweep hazard, CLAUDE.md 2026-07-16). Deliberately
has NO independent distance clamp like ur_goto_raw's check_move_size - the
whole point is autonomously replaying poses that were all real, already-
reached positions from a working calibration run, not an operator-supplied
target that could be a typo.

**Via-pose branch instability (found the hard way 2026-07-24, sibling
auto_calibration_points.py):** because qnear for the via-pose is normally the
arm's LIVE joints, and those differ after every calibration pose (whatever
branch that pose's own IK solve landed in), get_inverse_kin can resolve the
SAME fixed via-pose target to a different joint branch move to move -
occasionally one far enough away to require a large/fast joint excursion (a
wrist or base joint appearing to spin ~360deg) or blow through MAX_WAIT,
sometimes bad enough to need the soft stop below. Fix: the first successful
via-pose arrival in a session has its joints cached and reused as the qnear
seed for every later via-pose move, so it always resolves the same branch
regardless of the preceding pose's landing config. Calibration-pose targets
are unaffected - each is only visited once, so seeding from the arm's live
joints is still correct there.

Arrival is independently verified against the commanded target (position +
orientation), not inferred from speed settling to ~0 alone - a protective
stop also freezes speed at ~0, which looks identical to "arrived" to a
speed-only check (the same class of bug documented in catch.py's
check_safety_mode()/move_to()). A move that didn't actually get there (fault,
or no reachable/safe IK branch) stops the script and asks you to retry, skip,
or quit - it does NOT silently continue to the next pose.

Press Enter to move to the next pose, 'q' to stop early (Ctrl-C also works,
same as any other script here - it just leaves the arm holding at whatever
pose it last reached), SPACE at any point during a move for the soft stop
above. If a move ends in a protective stop (or anything else that isn't
safety mode NORMAL), the script notices via the Dashboard Server and waits
for you to clear it by hand on the pendant before offering the
retry/skip/quit choice.
"""
import argparse
import contextlib
import json
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
                          MAX_WAIT, SETTLE_SPEED, SETTLE_TICKS, MOVE_START_SPEED)

DEFAULT_SPEED = 0.5   # rad/s - supervised replay, not a race; tune with --speed
DEFAULT_ACCEL = 1.0   # rad/s^2
POS_TOL = 0.01         # m - arrival check
ROT_TOL = 0.05         # rad - arrival check (rotation vector norm difference)
JOINT_LIMIT_ASSUMED = 2 * np.pi   # rad - UR default per-joint software limit (+-360deg)
JOINT_LIMIT_MARGIN = np.radians(60)   # warn once a joint is within this of the assumed limit
STOP_KEY = " "         # soft-stop key, same convention as jog_ur_raw.py
STOP_DECEL = 2.0       # rad/s^2 - joint deceleration for the soft stop

# Control-box keepout (2026-07-24, real repeated near-collisions requiring a manual
# soft stop on the sibling auto_calibration_points.py - see that file's module
# docstring). User-confirmed geometry: the box sits ~180deg from the wait pose's own
# azimuth (opposite side of the base), ~30cm below the base motor, ~45cm wide -
# radial distance from the base axis wasn't measured, so this errs wide (40deg
# half-width) rather than precise. ENDPOINT CHECK ONLY: there's no local forward-
# kinematics/path-simulation in this codebase, so this can't see a dangerous SWEEP
# between two individually-safe endpoints. Treat this as one more layer, not a
# substitute for watching the arm / keeping SPACE ready.
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
    soft_stopped).

    Doesn't start counting the settle streak until real motion has actually
    been observed (speed > MOVE_START_SPEED) - see ur_goto_raw.wait_for_stop's
    docstring (CB3 UR10 bring-up 2026-08-24) for why an unguarded version can
    false-positive "settled" before a slower-to-start controller has begun
    moving at all."""
    fd = sys.stdin.fileno()
    slow_streak = 0
    started = False
    soft_stopped = False
    start = time.time()
    with cbreak_mode(fd):
        while time.time() - start < MAX_WAIT:
            for key in read_keys_nonblocking(fd):
                if key == STOP_KEY and not soft_stopped:
                    print("\n  SOFT STOP - sending stopj()...")
                    send_script(stopj_script(STOP_DECEL))
                    soft_stopped = True
            peak = max(abs(v) for v in rtde_r.getActualQd())
            if not started:
                if peak > MOVE_START_SPEED:
                    started = True
            elif peak < SETTLE_SPEED:
                slow_streak += 1
                if slow_streak >= SETTLE_TICKS:
                    return True, soft_stopped
            else:
                slow_streak = 0
            time.sleep(0.05)
    return False, soft_stopped


def wrap_to_pi(angles):
    """Canonicalize each joint angle into (-pi, pi] - see module docstring's
    "Joint wind-up" note. Used to build the qnear handed to get_inverse_kin so
    the IK solver's per-joint "nearest" search can't ratchet a joint's value
    toward its physical limit across a sequence of moves."""
    return [((a + np.pi) % (2 * np.pi)) - np.pi for a in angles]


def warn_if_near_joint_limit(joints):
    for i, q in enumerate(joints):
        if abs(q) > JOINT_LIMIT_ASSUMED - JOINT_LIMIT_MARGIN:
            print(f"  WARNING: joint {i} = {np.degrees(q):.1f}deg is within "
                  f"{np.degrees(JOINT_LIMIT_MARGIN):.0f}deg of the assumed +-360deg limit.")


def load_poses(path):
    data = json.loads(open(path).read())
    poses = [s["tcp_pose"] for s in data["samples"] if s.get("tcp_pose") is not None]
    if not poses:
        raise SystemExit(
            f"{path} has no samples with a recorded tcp_pose (older calibration runs without "
            f"the tool-offset joint solve don't record one) - nothing to replay."
        )
    return poses, data.get("tcp_offset")


def reconnect_dashboard(dash):
    """Switching the pendant to Local Control kicks the external Dashboard
    Server client - the next call on `dash` then raises (seen live 2026-07-23:
    "RuntimeError: Connection reset by peer" from safetymode(), which crashed
    the script outright before this fix). Retries connect() until it succeeds
    - it won't while still in Local Control, so this loop is also what makes
    the script just wait instead of dying."""
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
    wait_for_stop's speed-only settle check - a protective stop freezes speed
    at ~0 too, which looks identical to "arrived" otherwise (see module
    docstring). `qnear` defaults to the arm's live joints (previous behavior);
    pass a cached value for a repeated fixed-pose target (the via-pose) so
    get_inverse_kin always resolves the same branch - see module docstring's
    "Via-pose branch instability" note. `box_az_deg`, if set, refuses to send
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
    parser.add_argument("--transform-in", default="UR10_T_base_from_mocap.json",
                         help="calibration output JSON to replay poses from (default UR10_T_base_from_mocap.json)")
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

    poses, tcp_offset = load_poses(args.transform_in)
    n = len(poses)
    if not (1 <= args.start_at <= n):
        raise SystemExit(f"--start-at must be between 1 and {n}")

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    dash = dashboard_client.DashboardClient(ROBOT_IP)
    dash.connect()

    try:
        wait_for_remote_and_normal(dash)  # tolerate starting up while in Local Control too

        if not args.skip_set_tcp:
            if tcp_offset is None:
                print("NOTE: no tcp_offset recorded in the transform file - skipping set_tcp(); "
                      "make sure the box-centroid TCP is already active or these poses won't line up.")
            else:
                apply_and_verify_tcp(rtde_r, tcp_offset)
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
            flagged = [i for i, p in enumerate(poses, start=1)
                       if check_keepout(p, box_az_deg, args.box_keepout_half_width_deg) is not None]
            if flagged:
                print(f"  NOTE: pose(s) {flagged} already fall inside the keepout wedge and will be "
                      f"refused when reached - pass --no-box-keepout/a smaller half-width if that's wrong.")

        print(f"\nLoaded {n} poses from {args.transform_in}. Replaying from pose {args.start_at}.")
        print("Press Enter to move to the next pose, 'q' to stop early, SPACE during a move to "
              "soft-stop it.\n")

        via_qnear_cache = None  # set from the first clean via-pose arrival - see module docstring
        for i in range(args.start_at, n + 1):
            pose = poses[i - 1]
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
