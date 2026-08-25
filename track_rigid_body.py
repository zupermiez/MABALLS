"""
First real slice of the mocap -> transform -> motion pipeline (CLAUDE.md's
"Catch Integration" plan) - the "static-target dry run" step, made continuous:
instead of a single one-shot movel, this keeps the tool positioned under
whatever gets tracked, updating live.

Watches Motive for a rigid body OTHER than the arm's own tracked box (whose
id is read straight from T_base_from_mocap.json's "rigid_body_id" field -
that's exactly the id to exclude, since it's the box on the flange, not a
target). Whichever other rigid body Motive reports becomes the tracked
target: its live position is transformed through the calibrated
T_base<-mocap into the robot's base frame, and the tool is continuously
commanded to a point --below meters straight down (base-frame -Z, i.e.
ordinary Z-up "down" - resolved *after* the mocap->base transform, so it
doesn't matter which way Motive's own Y-up axes point) from that centroid.

Continuous tracking here means a streamed speedl() velocity command, refreshed every
tick, rather than repeated discrete movel sends (the first version of this script) or
full servoj streaming (see CLAUDE.md's v2 note - still the eventual plan, this is a
cheaper intermediate step). A resent movel restarts a fresh trapezoidal accel/decel
profile from scratch every tick, which is what made the movel version visibly jagged -
the arm kept getting yanked out of its ramp before reaching cruise speed. speedl
doesn't have that problem: each tick sends a proportional-control velocity (capped at
--speed, ramped via --accel) towards the target with a self-expiring duration
(comfortably longer than one tick), and as long as the next tick's speedl arrives
before that duration elapses, the robot smoothly re-ramps to the new velocity instead
of first decelerating to zero. Sending a fresh program to the robot's secondary client
interface (port 30002) pre-empts whatever's running from the previous send - the same
mechanism jog_ur_raw.py already relies on for its keyboard jog bursts, just refreshed
continuously here instead of per keypress.

SAFETY: ur_goto_raw.py's one-shot moves refuse an oversized jump and require a
deliberate --force, because a human is reviewing that one decision. A continuous
unattended loop has no such per-tick review; instead the commanded velocity magnitude
is inherently bounded by --speed every tick (no unbounded jumps are possible the way a
one-shot movel could have one). A separate, much larger sanity distance
(SANITY_MAX_DISTANCE) still hard-stops the loop outright - that's specifically to catch
bad mocap data (e.g. a rigid body falsely reporting tracking_valid=True with a
stale/garbage position - CLAUDE.md notes tracking_valid can't always be trusted to flip
on occlusion), not to bound ordinary operation. On any skip condition (out of reach,
bad data, stale tracking) the loop sends an explicit stopl() rather than merely
withholding the next speedl - since a live velocity command is already in flight,
staying silent would let the robot coast on the last (now-distrusted) velocity until
its speedl duration expires.

Separately, every tick also checks live joint angles and speeds (rtde_r.getActualQ()/
getActualQd(), the same RTDE connection already open for TCP pose reads) against
--joint-limit-deg and the per-joint --base-shoulder-speed-limit-deg-s /
--elbow-wrist-speed-limit-deg-s (see check_joint_margins()/check_joint_speed_margins()),
stopping outright once any joint is within its --joint-stop-margin-deg or
--joint-speed-stop-margin-deg-s, warning earlier at the corresponding --*-warn-margin.
This is what used to show up as the robot silently locking with a "joint near/at
limit" popup. Neither check is direction- or motion-aware - the position check can't
tell whether the commanded motion is driving further into a limit or pulling back out,
and the speed check only sees the joint speed the controller has already reached this
tick, not what a not-yet-sent speedl would produce (both would need the arm's
Jacobian) - so together they're a coarse stop-and-let-the-operator-jog-clear backstop,
not smooth avoidance.

This is a deliberate, consented-to-risk test run - the operator is expected
to be at the physical E-stop the whole time it runs. Ctrl-C stops the
script and sends a stop command, but the E-stop is the real safety net.
"""

import argparse
import json
import threading
import time
from pathlib import Path

import numpy as np
import rtde_receive
from natnet import NatNetClient, DataFrame

from frames import mocap_point_to_base
from calibrate_frames import send_urscript, set_tcp_script, ROBOT_IP
from jog_ur_raw import stop_script

DEFAULT_SPEED = 0.1          # m/s - max commanded speedl velocity magnitude, matches
                              # ur_goto_raw.py's conservative default
DEFAULT_ACCEL = 0.3          # m/s^2 - speedl ramp rate
DEFAULT_BELOW = 0.20         # m below the tracked centroid, base frame -Z
DEFAULT_GAIN = 2.0           # 1/s - proportional gain: commanded speed = min(--speed,
                              # gain * distance-to-target). Higher = snappier tracking
                              # but more overshoot risk; lower = smoother but laggier.
UPDATE_RATE = 10.0           # Hz - how often a new speedl target is sent. Raised from
                              # the movel version's 5Hz: speedl streaming wants a tighter
                              # refresh than that, since a late tick coasts on its last
                              # velocity for up to its speedl duration before the
                              # controller's own default deceleration would kick in.
DEADBAND = 0.005             # m - inside this, command zero velocity instead of chasing
                              # positional noise
SANITY_MAX_DISTANCE = 1.5    # m - beyond this, treat the target as bad data, not a
                              # legitimate move, and stop instead of chasing it
MAX_REACH = 1.1              # m from the base origin - a soft, approximate sanity check,
                              # NOT a real IK-feasibility test (that's future catch.py work,
                              # per CLAUDE.md). Conservative vs. the UR12e's published 1.3m
                              # reach, to leave margin for the box's 12cm TCP offset (reach
                              # specs are normally to the standard flange) and for the fact
                              # that near the true limit the arm runs out of orientation
                              # freedom / approaches singularities well before full extension.
                              # Without this, an out-of-reach target would just get chased
                              # (at up to --speed) until the controller itself faults on an
                              # unreachable request - this check means we stop before that
                              # point instead of relying on the robot's own protective stop
                              # as the only backstop.
LOST_TRACKING_TIMEOUT = 1.0  # s - stop sending updates once the target goes stale

JOINT_NAMES = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
JOINT_LIMIT_DEG = 363.0      # deg, +/- from zero, same for all 6 joints - read off the
                              # pendant's Installation -> Safety -> Joint Limits screen.
                              # Override with --joint-limit-deg if that config changes.
JOINT_WARN_MARGIN_DEG = 15.0 # deg from the limit - print a warning but keep moving
JOINT_STOP_MARGIN_DEG = 5.0  # deg from the limit - stop outright, same treatment as
                              # MAX_REACH/SANITY_MAX_DISTANCE above. Deliberately not
                              # direction-aware (that would need the arm's Jacobian to
                              # know whether the commanded speedl is driving a joint
                              # further into its limit or pulling it back out) - this
                              # is a coarse "stop and let the operator jog clear"
                              # backstop, not smooth avoidance. Good enough to turn a
                              # silent protective-stop lockup into an early, legible
                              # warning naming the actual joint.

# Per-joint max speed, deg/s - also read off the pendant's Installation -> Safety ->
# Joint Limits screen (it lists a speed limit alongside the position limit). Same
# base/shoulder vs. elbow+wrists grouping as reported there; override via
# --base-shoulder-speed-limit-deg-s / --elbow-wrist-speed-limit-deg-s.
DEFAULT_BASE_SHOULDER_SPEED_LIMIT_DEG_S = 131.0
DEFAULT_ELBOW_WRIST_SPEED_LIMIT_DEG_S = 191.0
JOINT_SPEED_WARN_MARGIN_DEG_S = 20.0
JOINT_SPEED_STOP_MARGIN_DEG_S = 8.0  # unlike the position check, this one is inherently
                                      # reactive, not predictive: getActualQd() reports
                                      # the joint speed the controller has *already*
                                      # commanded/achieved this tick, not what a
                                      # not-yet-sent speedl would produce (that would
                                      # need the arm's Jacobian - same limitation as the
                                      # position check above). Useful for catching a
                                      # joint that's been ramping up across several
                                      # ticks (e.g. near a singularity, where a small
                                      # Cartesian speedl maps to a large joint speed)
                                      # before the controller's own speed-limit
                                      # protective stop trips on the next one.

STATE_LOCK = threading.Lock()

INSTRUCTIONS = """
Continuous rigid-body tracking test
------------------------------------
Moves the tool to {below:.2f}m below whatever new rigid body Motive picks up
(any id other than the arm's own tracked box), continuously, via a streamed
speedl() velocity command refreshed every tick.

This is a real, continuous, unattended motion loop. Make sure the workspace
is clear and you are ready at the physical E-stop before starting - Ctrl-C
stops the script and sends a stop command, but the E-stop is the real
safety net if anything looks wrong.
"""


def load_transform(path):
    data = json.loads(Path(path).read_text())
    R = np.array(data["R"])
    t = np.array(data["t"])
    return R, t, data["tcp_offset"], data["rigid_body_id"]


class TargetState:
    def __init__(self):
        self.target_id = None
        self.candidate_ids = []
        self.latest_pos = None
        self.latest_valid = False
        self.latest_t = None


def make_handler(state: TargetState, tool_rb_id, forced_target_id):
    def handle_frame(frame: DataFrame) -> None:
        with STATE_LOCK:
            if state.target_id is None:
                if forced_target_id is not None:
                    state.target_id = forced_target_id
                else:
                    others = [rb.id_num for rb in frame.rigid_bodies if rb.id_num != tool_rb_id]
                    if len(others) == 1:
                        state.target_id = others[0]
                    elif len(others) > 1:
                        state.candidate_ids = others
                    if state.target_id is None:
                        return

            rb = next((r for r in frame.rigid_bodies if r.id_num == state.target_id), None)
            if rb is None:
                return
            state.latest_pos = rb.pos
            state.latest_valid = True if rb.tracking_valid is None else rb.tracking_valid
            state.latest_t = time.monotonic()

    return handle_frame


def wait_for_target(state: TargetState, timeout=30.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        with STATE_LOCK:
            target_id = state.target_id
            candidates = list(state.candidate_ids)
        if target_id is not None:
            return target_id
        if candidates:
            raise SystemExit(
                f"Multiple new rigid bodies seen: {candidates}. Re-run with "
                f"--target-rigid-body-id <id> to pick one."
            )
        time.sleep(0.1)
    raise SystemExit("No rigid body other than the tool appeared within timeout - "
                      "check Motive is tracking a second object.")


def check_joint_margins(q_rad, limit_deg, warn_margin_deg, stop_margin_deg):
    """Compare live joint angles (rad, from rtde_r.getActualQ()) against a symmetric
    +/-limit_deg range. Returns (stop, warnings): stop is True if any joint is within
    stop_margin_deg of its limit (caller should halt motion), warnings is a list of
    human-readable strings for any joint within warn_margin_deg (including stop-margin
    joints) for status-line reporting."""
    stop = False
    warnings = []
    for name, q in zip(JOINT_NAMES, q_rad):
        deg = np.degrees(q)
        margin = limit_deg - abs(deg)
        if margin <= warn_margin_deg:
            warnings.append(f"{name}={deg:+.1f}deg ({margin:.1f} from limit)")
        if margin <= stop_margin_deg:
            stop = True
    return stop, warnings


def check_joint_speed_margins(qd_rad_s, limits_deg_s, warn_margin_deg_s, stop_margin_deg_s):
    """Same shape as check_joint_margins(), but for live joint speed (rad/s, from
    rtde_r.getActualQd()) against a per-joint deg/s limit list (see
    DEFAULT_*_SPEED_LIMIT_DEG_S)."""
    stop = False
    warnings = []
    for name, qd, limit in zip(JOINT_NAMES, qd_rad_s, limits_deg_s):
        deg_s = np.degrees(qd)
        margin = limit - abs(deg_s)
        if margin <= warn_margin_deg_s:
            warnings.append(f"{name}={deg_s:+.1f}deg/s ({margin:.1f} from {limit:.0f} limit)")
        if margin <= stop_margin_deg_s:
            stop = True
    return stop, warnings


def speedl_script(velocity, accel, duration):
    """Cartesian speedl() command with a self-expiring duration - deliberately has no
    trailing stopl() (unlike jog_ur_raw.py's key-burst version): a fresh speedl sent
    before `duration` elapses smoothly re-ramps the commanded velocity via `accel`
    instead of first decelerating to zero, which is what removes the movel-restart jag
    this replaced. See module docstring."""
    vec = "[" + ",".join(f"{v:.5f}" for v in velocity) + "]"
    return f"""def prog():
  speedl({vec}, a={accel}, t={duration})
end
prog()
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--transform", default="UR10_T_base_from_mocap.json",
                         help="Calibration file written by calibrate_frames.py")
    parser.add_argument("--target-rigid-body-id", type=int, default=None,
                         help="Force which rigid body to track (auto-detected if only one "
                              "other than the tool's box is visible)")
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--below", type=float, default=DEFAULT_BELOW,
                         help=f"Meters straight down (base frame -Z) from the target's "
                              f"centroid (default {DEFAULT_BELOW})")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="m/s, max commanded velocity")
    parser.add_argument("--accel", type=float, default=DEFAULT_ACCEL, help="m/s^2, speedl ramp rate")
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                         help="1/s, proportional gain from position error to commanded speed")
    parser.add_argument("--rate", type=float, default=UPDATE_RATE, help="Target-update rate, Hz")
    parser.add_argument("--joint-limit-deg", type=float, default=JOINT_LIMIT_DEG,
                         help="deg, +/- from zero, same for all 6 joints (from the pendant's "
                              "Installation -> Safety -> Joint Limits screen)")
    parser.add_argument("--joint-warn-margin-deg", type=float, default=JOINT_WARN_MARGIN_DEG,
                         help="deg from a joint limit at which to print a warning but keep moving")
    parser.add_argument("--joint-stop-margin-deg", type=float, default=JOINT_STOP_MARGIN_DEG,
                         help="deg from a joint limit at which to stop outright")
    parser.add_argument("--base-shoulder-speed-limit-deg-s", type=float,
                         default=DEFAULT_BASE_SHOULDER_SPEED_LIMIT_DEG_S,
                         help="deg/s speed limit for the base and shoulder joints")
    parser.add_argument("--elbow-wrist-speed-limit-deg-s", type=float,
                         default=DEFAULT_ELBOW_WRIST_SPEED_LIMIT_DEG_S,
                         help="deg/s speed limit for the elbow and wrist1/2/3 joints")
    parser.add_argument("--joint-speed-warn-margin-deg-s", type=float,
                         default=JOINT_SPEED_WARN_MARGIN_DEG_S,
                         help="deg/s from a joint speed limit at which to print a warning")
    parser.add_argument("--joint-speed-stop-margin-deg-s", type=float,
                         default=JOINT_SPEED_STOP_MARGIN_DEG_S,
                         help="deg/s from a joint speed limit at which to stop outright")
    args = parser.parse_args()
    joint_speed_limits_deg_s = (
        [args.base_shoulder_speed_limit_deg_s] * 2 + [args.elbow_wrist_speed_limit_deg_s] * 4
    )

    R, t, tcp_offset, tool_rb_id = load_transform(args.transform)
    print(f"Loaded transform from {args.transform} (calibrated against tool rigid body id={tool_rb_id})")

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    before = rtde_r.getActualTCPPose()
    send_urscript(set_tcp_script(tcp_offset))
    time.sleep(0.2)
    after = rtde_r.getActualTCPPose()
    print(f"set_tcp({tcp_offset}) sent. before={[round(v, 4) for v in before]} "
          f"after={[round(v, 4) for v in after]}")
    # No separate "hold this orientation" target needed below: each speedl commands
    # zero angular velocity, which holds whatever orientation the arm has right now.

    print(INSTRUCTIONS.format(below=args.below))
    input("Press Enter once the workspace is clear and you're ready at the E-stop...")

    state = TargetState()
    client = NatNetClient(
        server_ip_address=args.server_ip,
        local_ip_address=args.local_ip,
        use_multicast=not args.unicast,
    )
    client.on_data_frame_received_event.handlers.append(
        make_handler(state, tool_rb_id, args.target_rigid_body_id)
    )

    with client:
        client.run_async()
        try:
            print("\nWaiting for a rigid body other than the tool to appear in Motive...")
            target_id = wait_for_target(state)
            print(f"Tracking rigid body id={target_id} - {args.below * 100:.0f}cm below its centroid.\n")

            # duration passed to each speedl - comfortably longer than one tick so a
            # slightly-late next send doesn't let the controller's own default
            # deceleration kick in before this loop's explicit re-ramp arrives.
            speedl_duration = max(2.0 / args.rate, 0.2)
            moving = False  # whether a live (non-stopped) speedl is currently in flight
            try:
                while True:
                    tick_start = time.monotonic()
                    with STATE_LOCK:
                        pos, valid, seen_t = state.latest_pos, state.latest_valid, state.latest_t

                    stale = seen_t is None or (time.monotonic() - seen_t) > LOST_TRACKING_TIMEOUT
                    current_pos = np.array(rtde_r.getActualTCPPose()[:3])
                    pos_stop, pos_warnings = check_joint_margins(
                        rtde_r.getActualQ(), args.joint_limit_deg,
                        args.joint_warn_margin_deg, args.joint_stop_margin_deg,
                    )
                    speed_stop, speed_warnings = check_joint_speed_margins(
                        rtde_r.getActualQd(), joint_speed_limits_deg_s,
                        args.joint_speed_warn_margin_deg_s, args.joint_speed_stop_margin_deg_s,
                    )
                    joint_stop = pos_stop or speed_stop
                    joint_warnings = pos_warnings + speed_warnings

                    if joint_stop:
                        # Takes priority over the mocap-target logic below regardless of
                        # whether the target itself is valid - a joint sitting this close
                        # to its position or speed limit is a robot-state fact, not a
                        # mocap-quality one, and the whole point is to stop *before* the
                        # controller's own protective stop does (see check_joint_margins()/
                        # check_joint_speed_margins()).
                        print(f"\rSTOPPED: joint limit close - {', '.join(joint_warnings)}"
                              f"                                        ",
                              end="", flush=True)
                        if moving:
                            try:
                                send_urscript(stop_script())
                            except Exception:
                                pass
                            moving = False
                    elif valid and not stale:
                        p_base = mocap_point_to_base(np.array(pos), R, t)
                        target_pos = p_base.copy()
                        target_pos[2] -= args.below

                        # Base frame origin is (0, 0, 0) by definition, so this is
                        # straightforwardly distance-from-base - a soft reach check (see
                        # MAX_REACH) checked *before* the bad-data check below, since a
                        # genuinely out-of-reach target isn't necessarily a big jump from
                        # the arm's current position (e.g. already sitting near max reach).
                        distance_from_base = np.linalg.norm(target_pos)
                        distance_from_current = np.linalg.norm(target_pos - current_pos)

                        if distance_from_base > MAX_REACH:
                            print(f"\rSKIPPED: target {distance_from_base:.2f}m from robot base - "
                                  f"beyond the {MAX_REACH:.2f}m soft reach limit, stopping.        ",
                                  end="", flush=True)
                            if moving:
                                send_urscript(stop_script())
                                moving = False
                        elif distance_from_current > SANITY_MAX_DISTANCE:
                            print(f"\rSKIPPED: target {distance_from_current:.2f}m from current pose - "
                                  f"treating as bad data, stopping.                         ",
                                  end="", flush=True)
                            if moving:
                                send_urscript(stop_script())
                                moving = False
                        else:
                            error = target_pos - current_pos
                            dist = np.linalg.norm(error)
                            if dist < DEADBAND:
                                velocity = np.zeros(3)
                            else:
                                speed_mag = min(args.speed, dist * args.gain)
                                velocity = error / dist * speed_mag
                            full_velocity = list(velocity) + [0.0, 0.0, 0.0]
                            try:
                                send_urscript(speedl_script(full_velocity, args.accel, speedl_duration))
                                moving = True
                            except OSError as e:
                                print(f"\rWARNING: failed to send speedl ({e}) - skipping this tick.",
                                      end="", flush=True)
                            warn_suffix = f"  JOINT WARN: {', '.join(joint_warnings)}" if joint_warnings else ""
                            print(f"\rtarget=({target_pos[0]:+.3f},{target_pos[1]:+.3f},{target_pos[2]:+.3f})  "
                                  f"current=({current_pos[0]:+.3f},{current_pos[1]:+.3f},{current_pos[2]:+.3f})  "
                                  f"dist={distance_from_current * 100:5.1f}cm  "
                                  f"vel={np.linalg.norm(velocity) * 100:4.1f}cm/s{warn_suffix}   ",
                                  end="", flush=True)
                    else:
                        if moving:
                            try:
                                send_urscript(stop_script())
                            except Exception:
                                pass
                            moving = False
                        print("\rtarget lost/stale - stopped, holding position...                    ",
                              end="", flush=True)

                    elapsed = time.monotonic() - tick_start
                    time.sleep(max(0.0, 1.0 / args.rate - elapsed))
            except KeyboardInterrupt:
                pass
        finally:
            client.stop_async()

    print("\nStopping...")
    try:
        send_urscript(stop_script())
    except Exception:
        pass
    rtde_r.disconnect()


if __name__ == "__main__":
    main()
