"""
catch.py - the conductor: perception -> prediction -> feasibility gate -> REAL arm motion.

This is the first script in the project that actually MOVES THE ARM AT A THROWN
BALL. Everything upstream (release detection, trajectory fit, frame transform,
feasibility model) is reused unchanged from live_trajectory.py / trajectory.py /
frames.py / catch_feasibility.py; this file only adds the two things those didn't
do: (1) pre-position the arm at a fixed wait pose on a catch plane, and (2) when a
throw is predicted to be catchable, fire ONE max-speed movel to the intercept.

Design (per CLAUDE.md "Catch Integration"):
  - Race, not pursuit. The arm waits pre-positioned on a horizontal catch plane
    ~0.6m in front of the base (its fastest, most repeatable, non-singular zone -
    see the 2026-07-15 catch-plane-height analysis in docs/debug_log.md). A throw
    only ever requires a SHORT in-plane slide from the wait pose to the crossing
    point, never a full-workspace traverse.
  - Robot position ALWAYS comes from RTDE forward kinematics (getActualTCPPose),
    never mocap. Rationale: this is validated against a looping Motive *replay* of
    a recorded throw, in which the tool rigid body is frozen at its recorded spot -
    so a mocap tool position would be meaningless. (This is the opposite choice
    from catch_feasibility.py's optional --tool-rigid-body-id, which exists for the
    powered-off case; here the arm is powered and really moving.)
  - Commit rule: fire the INSTANT the feasibility gate passes (feasible OR "possibly
    catch" within the box/funnel tolerance) and enough samples exist (--commit-samples)
    - do NOT wait for the prediction to stabilize first. Changed 2026-07-15 (user
    directive, after catch_log analysis of a real session): the gate is almost always
    true on the very first eligible tick, when the most time budget remains, and had
    already degraded to a miss by the time --stability-window predictions accumulated
    on nearly half the recorded throws - waiting for stability was discarding the best
    opportunity, not improving it. Still prefers the --stability-window average as the
    commit TARGET POINT when it's already available (free noise reduction - pred_window
    fills regardless of this gate), it just never blocks the commit DECISION on it.

SAFETY. A catch move is inherently larger than ur_goto_raw.py's 0.15m/axis clamp,
so this script does NOT reuse that clamp or spam --force. Instead every commanded
target is checked against a dedicated, conservative catch envelope
(check_catch_envelope): a base-frame reach band, a z band that keeps the tool off
the platform deck / floor and out of the overhead singularity, and a hard cap on
how far the target may be from the wait pose. A target failing ANY of these is
refused and no motion is sent. Motion uses the proven raw-URScript-over-socket path
(port 30002), fire-and-forget, same as ur_goto_raw.py - there is no persistent
ur_rtde control session here (rtde_receive is read-only), so the "never force-kill
a control session" hazard does not apply; Ctrl-C sends a stopl and exits.

Run `python3 catch.py --help`. Start with `--dry-run` (no motion at all - logs
every decision) to validate the wait pose, derived plane, and gating against your
replay before enabling motion.
"""

import argparse
import json
import math
import os
import socket
import time
from collections import deque
from typing import List, Optional

import numpy as np
import rtde_receive
from natnet import NatNetClient, DataFrame

from live_trajectory import STATE_LOCK, SharedState, add_release_detection_args, make_handler
from trajectory import AXIS_NAMES
from frames import mocap_point_to_base
from ur_goto_raw import ROBOT_IP, SECONDARY_PORT, send_script, movel_absolute_script
from calibrate_frames import send_urscript, set_tcp_script
from catch_feasibility import (
    MoveTimeModel, fit_move_time_model, latest_speed_char_json, load_transform,
    check_feasibility, format_result, MIN_SAMPLES_FOR_CHECK, DEFAULT_CRUISE_SPEED,
    DEFAULT_BOX_RADIUS, ROBOTMODE_IDLE,
)

# --- Catch-motion safety envelope (base frame). Deliberately tighter than
# catch_feasibility.py's *read* envelope: this one gates real motion. ---
# CATCH_MIN_REACH raised 0.35->0.45m 2026-07-15: a real protective stop was traced
# (via catch_logs/ analysis) to a committed catch move targeting reach=0.370m - just
# 2cm inside the old floor. Across two recorded sessions every commit that completed
# normally landed at reach>=0.538m; the one fault was the only commit below that,
# and the arm only completed ~28% of the commanded move before the fault froze it
# (see docs/debug_log.md 2026-07-15 "closer than preset position" for the full
# analysis). 0.45m gives real margin on both sides: ~8cm clear of the observed
# fault (vs. the old floor's mere 2cm), and ~5cm below DEFAULT_WAIT_POSE's own reach
# (0.5004m, computed - NOT 0.50m flat, which would leave the wait pose only 0.4mm
# inside its own envelope and one calibration nudge from rejecting itself). The true
# safe boundary between 0.37 and 0.538m is uncharacterized (no joint-angle telemetry
# was logged for either the incident or the successes); this trades away that slice
# of workspace rather than guess at it.
CATCH_MIN_REACH = 0.45   # m from base - inside this is near-singular / too close to the body
CATCH_MAX_REACH = 1.20   # m from base - was 1.00; user directive 2026-07-15: attempt catches out to 1.20m
CATCH_Z_MIN = -0.25      # m base-frame - below this the tool reaches down toward the 1m platform deck / floor
CATCH_Z_MAX = 0.55       # m base-frame - above this heads toward the overhead shoulder singularity (slow, imprecise)
# MAX_CATCH_MOVE (was 0.60m, capped distance from the wait pose) removed 2026-07-15 per
# user directive - only the reach/z band above now bounds a catch target, so any target
# within CATCH_MAX_REACH is attempted regardless of distance from the wait pose.

# Default wait pose, full 6-DOF (base frame) - box taught upright, captured via
# ur_get_pose.py 2026-07-15. Was position-only (0.0, -0.60, 0.10) with orientation
# taken from wherever the arm happened to be at startup ("pre-orient the funnel by
# hand") - that made the wait pose depend on whatever freedrive session came before
# it. Now fully fixed: every run drives to this exact pose regardless of how the arm
# was left. Override with --wait-pose to teach a different one.
#
# Re-taught 2026-07-15 (2nd time) - the first taught pose sat right on the wrist
# singularity (wrist2/J4 within ~1-5 deg of 0 deg), which caused two real protective
# stops (C153A3, wrist joint 1 path deviation) once catch moves got braver (wider
# reach, no move-distance cap, lower margin - see docs/debug_log.md). This pose has
# wrist2 = +85.5 deg, near the best-conditioned point away from both singular values
# (0 deg and 180 deg) - confirmed via ur_get_pose.py joint readout before saving.
DEFAULT_WAIT_POSE = (0.1139, -0.4686, 0.1335, 1.5840, -0.0824, -0.0573)

# --record output goes here, not cwd - keeps the repo root from filling up with one
# file per session the way speed_char_*.json/png already do.
CATCH_LOG_DIR = "catch_logs"


def rnd(x, nd: int = 4):
    """Round floats (recursively through lists/tuples/ndarrays/dicts) for compact JSON.

    Passes bools/strs/None/ints through unchanged - only float-ish values get
    truncated, since those are what bloat a JSONL log with noise digits.
    """
    if x is None or isinstance(x, bool):
        return x
    if isinstance(x, dict):
        return {k: rnd(v, nd) for k, v in x.items()}
    if isinstance(x, (list, tuple, np.ndarray)):
        return [rnd(v, nd) for v in x]
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        return round(float(x), nd)
    return x


class Recorder:
    """Appends one compact JSON object per line to --record's log file: every
    feasibility tick, gate/commit/refuse decision, throw start/end, and robot move
    this conductor makes. Deliberately flat and unpretty-printed (not a nested/
    human-formatted log) so a later "what happened on throw N" question can be
    answered by grepping/jq-filtering this file for `"throw":N` rather than reading
    a whole run - the point is being cheap for an agent to search, not to look nice.

    `t` is the NatNet/Motive sample timestamp (the same clock a Motive replay of
    this session re-streams) and `throw` is a 1-based ordinal - both are offered as
    correlation keys against a Motive replay of the same run, since it isn't known
    up front whether Motive's replay preserves original frame timestamps exactly or
    rebases them from zero; `throw` (count the Nth throw in the replay, in order) is
    the robust fallback either way.

    Flushed after every line (not buffered) so a protective stop or Ctrl-C never
    loses the tail of a session - that's exactly the run you'd want to debug most.
    """

    def __init__(self, path: Optional[str]):
        self.f = open(path, "a") if path else None
        self.throw = 0

    def log(self, ev: str, t: Optional[float] = None, **fields) -> None:
        if self.f is None:
            return
        rec = {"wall": round(time.time(), 3), "t": rnd(t), "throw": self.throw, "ev": ev}
        for k, v in fields.items():
            rec[k] = rnd(v)
        self.f.write(json.dumps(rec, separators=(",", ":")) + "\n")
        self.f.flush()

    def close(self) -> None:
        if self.f is not None:
            self.f.close()


def _verdict(r) -> str:
    """Compact string form of a FeasibilityResult's outcome, for the recorded log."""
    if r.crossing_t is None:
        return "no_crossing"
    if not r.reachable:
        return "unreachable"
    if r.feasible:
        return "catch"
    if r.possible:
        return "possible"
    return "miss"


def stopl_script(decel: float = 3.0) -> str:
    return f"def prog():\n  stopl({decel})\nend\nprog()\n"


def check_catch_envelope(target_xyz: np.ndarray) -> Optional[str]:
    """Return None if `target_xyz` (base frame) is a safe catch target, else a reason string.

    This is the single reviewed 'clamp-off' path CLAUDE.md requires for catch moves -
    every commanded target passes through here immediately before any motion is sent,
    regardless of what produced it, so a bad fit or a code bug can't fling the arm.
    """
    reach = float(np.linalg.norm(target_xyz))
    if not (CATCH_MIN_REACH <= reach <= CATCH_MAX_REACH):
        return f"reach {reach:.2f}m outside catch band [{CATCH_MIN_REACH:.2f},{CATCH_MAX_REACH:.2f}]m"
    z = float(target_xyz[2])
    if not (CATCH_Z_MIN <= z <= CATCH_Z_MAX):
        return f"height z={z:+.2f}m outside catch band [{CATCH_Z_MIN:+.2f},{CATCH_Z_MAX:+.2f}]m (deck/singularity guard)"
    return None


def derive_catch_plane(wait_xyz: np.ndarray, R: np.ndarray, t_vec: np.ndarray) -> float:
    """Mocap up-axis (Y) value of the horizontal plane through the wait height.

    The trajectory fit and its plane-crossing solver live in the mocap frame, so we
    inverse-transform the base-frame wait position back to mocap and read its up-axis
    (Y) component. Base Z ~= mocap Y in this rig (calibration R), so a constant-height
    base plane is a constant-Y mocap plane to within the calibration's ~1 deg tilt.
    """
    p_mocap = R.T @ (wait_xyz - t_vec)
    return float(p_mocap[1])  # mocap Y = up


# UR safety-mode enum (ur_rtde Robot State docs): 1=NORMAL, everything else is some
# form of reduced/stopped/faulted state. This cell has no configured reduced-speed
# safety zones, so NORMAL is the only mode expected during ordinary operation -
# anything else means a human needs to look at the robot.
SAFETY_MODE_NORMAL = 1


def check_safety_mode(rtde_r) -> Optional[str]:
    """None if the robot's safety mode is NORMAL, else a description of the fault.

    A protective stop (or any other non-NORMAL safety mode) freezes the robot in
    place - getActualTCPSpeed() then reads ~0 forever, indistinguishable from
    "arrived and stopped" to any check that only watches speed. This is exactly how
    a real 2026-07-15 incident went undetected: a catch movel was aborted early by a
    protective stop (traced via catch_logs/ - the arm completed only ~28% of the
    commanded move), and the very next 'return to wait' move then silently reported
    settled=True after 0.4s, because the frozen arm's zero speed looked identical to
    a successful, quick arrival - the script had no idea the robot was actually stuck
    until the user noticed the fault and cleared it ~30s later. See
    docs/debug_log.md 2026-07-15 for the full trace. Call this anywhere a move's
    success is judged from TCP speed alone.
    """
    mode = rtde_r.getSafetyMode()
    if mode != SAFETY_MODE_NORMAL:
        return f"safety_mode={mode} (not NORMAL) - robot is stopped/faulted, not actually moving"
    return None


def move_to(pose: List[float], speed: float, accel: float, rtde_r, settle_timeout: float = 8.0):
    """Blocking movel to an absolute base-frame pose. Returns (settled, fault):
    fault is a description string (and settled forced False) if a non-NORMAL safety
    mode is observed at any point in the wait - see check_safety_mode()."""
    send_script(movel_absolute_script(pose, speed, accel))
    slow_streak = 0
    start = time.time()
    while time.time() - start < settle_timeout:
        fault = check_safety_mode(rtde_r)
        if fault is not None:
            return False, fault
        if max(abs(v) for v in rtde_r.getActualTCPSpeed()) < 0.002:
            slow_streak += 1
            if slow_streak >= 5:
                return True, None
        else:
            slow_streak = 0
        time.sleep(0.05)
    return False, None


def halt_on_fault(rec: "Recorder", fault: str) -> None:
    """Log and stop hard on a detected robot fault - never try to auto-resume.

    Consistent with this project's established recovery path (ur_status.py --clear /
    the pendant - see CLAUDE.md 'Key safety rules'): a human needs to look at the
    robot, confirm where it actually is, and clear the fault before anything sends
    it another command.
    """
    rec.log("fault", reason=fault)
    raise SystemExit(
        f"\n!!! ROBOT FAULT DETECTED: {fault}\n"
        "The robot is not in normal operation - a protective stop or other safety "
        "event has frozen it. Clear it (ur_status.py --clear or the pendant), confirm "
        "the arm's actual position, then restart catch.py.\n"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--rigid-body-id", type=int, default=None, help="NatNet rigid body id of the ball")

    add_release_detection_args(parser)

    # Wait pose / catch plane -----------------------------------------------------
    parser.add_argument("--wait-pose", type=float, nargs=6, default=None,
                        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                        help="Full 6-DOF wait pose in base frame (from ur_get_pose.py). If omitted, "
                             f"defaults to the taught pose {DEFAULT_WAIT_POSE} (box upright) - fixed "
                             "regardless of the arm's current position/orientation at startup.")
    parser.add_argument("--catch-value", type=float, default=None,
                        help="Mocap up-axis (Y) value of the catch plane. Default: derived from the wait "
                             "pose height (recommended - keeps the plane through the wait pose).")

    # Transform / model -----------------------------------------------------------
    parser.add_argument("--transform-file", default="T_base_from_mocap.json",
                        help="T_base<-mocap from calibrate_frames.py")
    parser.add_argument("--speed-char-json", default=None,
                        help="speed_char.py JSON for the move-time model (default: newest in cwd)")
    parser.add_argument("--cruise-speed", type=float, default=DEFAULT_CRUISE_SPEED,
                        help=f"m/s assumed cruise speed for feasibility (default {DEFAULT_CRUISE_SPEED})")
    parser.add_argument("--box-radius", type=float, default=DEFAULT_BOX_RADIUS,
                        help=f"m funnel forgiveness radius; a short-but-close move within this is still "
                             f"attempted (default {DEFAULT_BOX_RADIUS})")
    parser.add_argument("--margin", type=float, default=0.03,
                        help="s spare-time margin for a *full* WOULD-CATCH (default 0.03, was 0.10 - lower "
                             "than the dry-run tool since we also attempt POSSIBLY-catch)")

    # Motion ----------------------------------------------------------------------
    parser.add_argument("--speed", type=float, default=1.5, help="m/s commanded for the CATCH movel (controller clamps; default 1.5)")
    parser.add_argument("--accel", type=float, default=6.0, help="m/s^2 for the catch movel (default 6.0)")
    parser.add_argument("--approach-speed", type=float, default=0.4, help="m/s for the (slower) move to/return-to wait pose")
    parser.add_argument("--approach-accel", type=float, default=1.0, help="m/s^2 for the approach/return moves")

    # Commit / trust --------------------------------------------------------------
    parser.add_argument("--commit-samples", type=int, default=40,
                        help="min flight samples before a real catch may be fired (default 40, was 50 ~ 0.33s @120Hz)")
    parser.add_argument("--stability-window", type=int, default=3,
                        help="when this many recent catch-point predictions already agree within "
                             "--drift-tol, their average is used as the commit TARGET POINT (free "
                             "noise reduction) - does NOT gate whether/when a commit fires, which "
                             "happens on the first feasible-or-possible tick regardless (default 3)")
    parser.add_argument("--drift-tol", type=float, default=0.08,
                        help="m agreement threshold for the --stability-window average to be used as "
                             "the commit target point (default 0.08) - does not delay commit timing")

    parser.add_argument("--dry-run", action="store_true",
                        help="Log every decision but send NO motion at all (not even the initial positioning). "
                             "Use first to validate wait pose / plane / gating against your replay.")
    parser.add_argument("--record", action="store_true",
                        help="Record every feasibility tick, gate/commit/refuse decision, throw "
                             "start/end, and robot move to a compact JSONL log file "
                             "(catch_logs/catch_log_<timestamp>.jsonl, auto-named) for later "
                             "analysis against a Motive replay of the same session - see CLAUDE.md "
                             "'Run recording'.")
    parser.add_argument("--yes", action="store_true", help="Skip the pre-motion confirmation prompt")
    parser.add_argument("--poll-hz", type=float, default=20.0, help="Feasibility-check/print rate during flight")
    parser.add_argument("--robot-ip", default=ROBOT_IP, help="UR12e controller IP")
    args = parser.parse_args()

    record_path = None
    if args.record:
        os.makedirs(CATCH_LOG_DIR, exist_ok=True)
        record_path = os.path.join(CATCH_LOG_DIR, f"catch_log_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    rec = Recorder(record_path)
    if record_path:
        print(f"recording decisions to {record_path} (jq/grep by \"throw\":N to find a specific throw)")

    speed_char_json = args.speed_char_json or latest_speed_char_json()
    if speed_char_json is None:
        raise SystemExit("No speed_char_*.json found and --speed-char-json not given. Run speed_char.py first.")
    model = fit_move_time_model(speed_char_json, args.cruise_speed)
    R, t_vec, calib_rmse, calib_created = load_transform(args.transform_file)
    with open(args.transform_file) as f:
        tcp_offset = json.load(f)["tcp_offset"]

    print(f"connecting to robot at {args.robot_ip} ...")
    rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)

    # The R/t transform above was calibrated with the box/funnel-centroid TCP
    # active (see calibrate_frames.py), not the flange - every target this
    # script sends is in that centroid's coordinates. set_tcp() is a
    # controller-side runtime setting that is NOT guaranteed to still be
    # active from a prior script's run, so it must be (re-)sent here too
    # (same pattern as track_rigid_body.py). Without this, movel targets are
    # interpreted against the flange instead, landing the catch tool short by
    # exactly the TCP offset - this was a real bug: balls were hitting the
    # wrist/last joint ~12cm short of the funnel.
    before = rtde_r.getActualTCPPose()
    send_urscript(set_tcp_script(tcp_offset))
    time.sleep(0.2)
    after = rtde_r.getActualTCPPose()
    print(f"set_tcp({tcp_offset}) sent. before={[round(v, 4) for v in before]} "
          f"after={[round(v, 4) for v in after]}")

    robot_mode = rtde_r.getRobotMode()
    frac = rtde_r.getTargetSpeedFraction()
    if robot_mode < ROBOTMODE_IDLE:
        raise SystemExit(
            f"Robot not powered (robotmode={robot_mode}, need >= {ROBOTMODE_IDLE}). Initialize/brake-release "
            f"on the pendant. (RTDE would also return a phantom all-zeros pose - see docs/debug_log.md 2026-07-15.)"
        )

    # --- Resolve the wait pose (6-DOF, base frame) - fixed regardless of where the
    # arm currently is, so a prior freedrive session never changes it. ---
    current_pose = list(rtde_r.getActualTCPPose())
    wait_pose = list(args.wait_pose) if args.wait_pose is not None else list(DEFAULT_WAIT_POSE)
    wait_xyz = np.array(wait_pose[:3])

    # Sanity: the wait pose itself must sit inside the catch envelope (reach/z bands).
    wait_reason = check_catch_envelope(wait_xyz)
    if wait_reason is not None:
        raise SystemExit(f"Wait pose is outside the catch envelope ({wait_reason}). Pick a --wait-pose in the reachable band.")

    catch_axis_idx = 1  # mocap Y (up)
    catch_value = args.catch_value if args.catch_value is not None else derive_catch_plane(wait_xyz, R, t_vec)

    print("=" * 78)
    print("CATCH - REAL ARM MOTION" + ("  [DRY RUN - no motion]" if args.dry_run else ""))
    print("=" * 78)
    print(f"transform: {args.transform_file} (rmse={calib_rmse * 1000:.1f}mm)  robotmode={robot_mode}")
    print(f"move-time model: accel={model.accel:.2f} m/s^2 latency={model.latency*1000:.0f}ms cruise={model.v_max:.2f} m/s")
    print(f"wait pose (base): pos=({wait_xyz[0]:+.3f},{wait_xyz[1]:+.3f},{wait_xyz[2]:+.3f}) reach={np.linalg.norm(wait_xyz):.2f}m  "
          f"orient=({wait_pose[3]:+.3f},{wait_pose[4]:+.3f},{wait_pose[5]:+.3f})")
    print(f"catch plane (mocap): {AXIS_NAMES[catch_axis_idx]} = {catch_value:.4f}")
    print(f"catch envelope: reach[{CATCH_MIN_REACH},{CATCH_MAX_REACH}]m  z[{CATCH_Z_MIN:+.2f},{CATCH_Z_MAX:+.2f}]m  "
          f"(no cap on distance from wait pose)")
    print(f"catch movel: v={args.speed} m/s a={args.accel} m/s^2   |   approach: v={args.approach_speed} a={args.approach_accel}")
    print(f"commit: n>={args.commit_samples} AND (feasible OR possibly-catch) - fires on the FIRST "
          f"qualifying tick; uses the last {args.stability_window}-prediction average (agreeing within "
          f"{args.drift_tol}m) as the target point when already available, else the raw current prediction")
    if frac < 0.99:
        print(f"WARNING: pendant speed slider at {frac*100:.0f}% - catch moves will be capped there.")
    print("=" * 78)

    rec.log("run_start",
            dry_run=args.dry_run, robot_ip=args.robot_ip,
            transform_file=args.transform_file, calib_rmse_m=calib_rmse, calib_created=calib_created,
            tcp_offset=tcp_offset, robot_mode=robot_mode, speed_fraction=frac,
            wait_pose=wait_pose, catch_axis=AXIS_NAMES[catch_axis_idx], catch_value=catch_value,
            catch_envelope={"reach_min": CATCH_MIN_REACH, "reach_max": CATCH_MAX_REACH,
                            "z_min": CATCH_Z_MIN, "z_max": CATCH_Z_MAX},
            move_time_model={"accel": model.accel, "latency": model.latency, "v_max": model.v_max,
                             "residual_rms": model.residual_rms, "n_legs": model.n_legs},
            speed_char_json=speed_char_json,
            catch_speed=args.speed, catch_accel=args.accel,
            approach_speed=args.approach_speed, approach_accel=args.approach_accel,
            commit_samples=args.commit_samples, stability_window=args.stability_window,
            drift_tol=args.drift_tol, margin=args.margin, box_radius=args.box_radius,
            cruise_speed=args.cruise_speed)

    if not args.dry_run:
        initial_sweep = float(np.linalg.norm(np.array(current_pose[:3]) - wait_xyz))
        print(f"\n*** THE ROBOT WILL MOVE. *** First it drives to the wait pose ({initial_sweep:.2f}m away, "
              f"at {args.approach_speed} m/s), then")
        print("fires fast catch moves toward thrown balls. Clear the area and keep the E-stop in hand.")
        if not args.yes:
            if input("Type 'go' to arm motion (anything else aborts): ").strip().lower() != "go":
                raise SystemExit("aborted.")
        print("\nmoving to wait pose...")
        t0 = time.time()
        settled, fault = move_to(wait_pose, args.approach_speed, args.approach_accel, rtde_r)
        rec.log("move", purpose="initial_wait_pose", target=wait_pose, speed=args.approach_speed,
                accel=args.approach_accel, settled=settled, fault=fault, duration_s=time.time() - t0)
        if fault is not None:
            halt_on_fault(rec, fault)
        if not settled:
            raise SystemExit("Did not reach the wait pose (timeout). Check the pendant / remote-control mode.")
        print("at wait pose. Ready - throw the ball. Ctrl-C to stop.\n")
    else:
        print("\n[dry-run] not moving. Feasibility from the arm's CURRENT pose. Throw the ball. Ctrl-C to stop.\n")

    # --- state ---
    s = SharedState()
    if args.rigid_body_id is not None:
        s.target_id = args.rigid_body_id

    client = NatNetClient(server_ip_address=args.server_ip, local_ip_address=args.local_ip,
                          use_multicast=not args.unicast)
    client.on_data_frame_received_event.handlers.append(make_handler(s, args))

    last_state = "idle"
    attempted = False           # fired a catch for the current throw already
    refuse_logged = False       # throttle envelope-refusal spam within one throw
    pred_window: deque = deque(maxlen=args.stability_window)

    def stable() -> Optional[np.ndarray]:
        """Mean predicted catch point if the window is full and agrees within drift-tol, else None."""
        if len(pred_window) < args.stability_window:
            return None
        arr = np.array(pred_window)
        mean = arr.mean(axis=0)
        if float(np.max(np.linalg.norm(arr - mean, axis=1))) > args.drift_tol:
            return None
        return mean

    with client:
        client.run_async()
        try:
            while True:
                # Catches a fault from the fire-and-forget catch movel too (that path
                # sends via raw send_script(), not move_to(), so it has no built-in
                # settle/fault check of its own) - within one poll interval of it
                # happening, not retroactively at throw_end. Skipped in --dry-run:
                # no motion is ever sent there, so an unrelated fault shouldn't
                # interrupt a pure perception-testing session.
                if not args.dry_run:
                    fault = check_safety_mode(rtde_r)
                    if fault is not None:
                        halt_on_fault(rec, fault)

                with STATE_LOCK:
                    target_id = s.target_id
                    candidate_ids = list(s.candidate_ids)
                    state = s.state
                    flight_buffer = list(s.flight_buffer)
                    history_head = s.history[0] if s.history else None

                if target_id is None:
                    if candidate_ids:
                        print(f"Multiple rigid bodies {candidate_ids}; re-run with --rigid-body-id <id>.")
                    time.sleep(1.0 / args.poll_hz)
                    continue

                if state == "flight" and last_state == "idle":
                    print(f"--- throw detected (rigid body {target_id}) ---")
                    attempted = False
                    refuse_logged = False
                    pred_window.clear()
                    rec.throw += 1
                    rec.log("throw_start", t=flight_buffer[-1].t if flight_buffer else None,
                            rigid_body_id=target_id, arm_tcp=list(rtde_r.getActualTCPPose()))

                if state == "flight" and not attempted and len(flight_buffer) >= MIN_SAMPLES_FOR_CHECK:
                    current_tcp_xyz = np.array(rtde_r.getActualTCPPose()[:3])  # RTDE FK only - never mocap
                    result = check_feasibility(
                        flight_buffer, catch_axis_idx, catch_value, R, t_vec, current_tcp_xyz,
                        model, CATCH_MIN_REACH, CATCH_MAX_REACH, args.margin, args.box_radius,
                    )
                    if result.crossing_t is not None:
                        pred_window.append(result.catch_point_base)
                        print(format_result(result))

                        gate = result.feasible or result.possible
                        trusted = stable()
                        # Fire the INSTANT the gate is true - do not wait for the stability
                        # window to fill first. 2026-07-15 user directive, after catch_log
                        # analysis of a real 15-throw session: the gate is usually true on the
                        # very first eligible tick (n==commit_samples, the most time budget
                        # available) and had degraded to a miss by the 3rd tick (the earliest
                        # `stable` can go True) on 7 of 15 throws - the predicted catch point
                        # kept sliding further from the arm as the fit refined, while
                        # time-to-impact only ever shrinks. Waiting for stability was
                        # discarding the best (often only) opportunity, not improving it. Still
                        # use the stability-window average as the commit POINT when it's
                        # already available (free noise reduction, pred_window fills
                        # regardless of this gate) - it just never blocks the DECISION.
                        commit_point = trusted if trusted is not None else result.catch_point_base
                        rec.log("tick", t=flight_buffer[-1].t, n=result.n_samples,
                                verdict=_verdict(result), reach=result.reach, from_tcp=current_tcp_xyz,
                                target=result.catch_point_base, dist_to_go=result.move_dist,
                                move_time=result.move_time, t_impact=result.time_to_impact,
                                margin=result.margin, shortfall=result.shortfall,
                                stable=trusted is not None, trusted_point=trusted,
                                commit_ready=len(flight_buffer) >= args.commit_samples)

                        if gate and len(flight_buffer) >= args.commit_samples:
                            target_pose = [float(commit_point[0]), float(commit_point[1]), float(commit_point[2]),
                                           wait_pose[3], wait_pose[4], wait_pose[5]]
                            reason = check_catch_envelope(commit_point)
                            if reason is not None:
                                if not refuse_logged:
                                    print(f"    >> REFUSED (envelope): {reason} - no motion sent")
                                    refuse_logged = True
                                rec.log("refuse", t=flight_buffer[-1].t, reason=reason, target=commit_point)
                            else:
                                verdict = "CATCH" if result.feasible else "POSSIBLY"
                                stability_note = "" if trusted is not None else "  [instant, pred_window not full yet]"
                                print(f"    >> COMMIT ({verdict}){stability_note}: movel to "
                                      f"({commit_point[0]:+.3f},{commit_point[1]:+.3f},{commit_point[2]:+.3f}) "
                                      f"v={args.speed} a={args.accel}"
                                      + ("   [dry-run: not sent]" if args.dry_run else ""))
                                rec.log("commit", t=flight_buffer[-1].t,
                                        verdict="catch" if result.feasible else "possible",
                                        target_pose=target_pose, speed=args.speed, accel=args.accel,
                                        stable=trusted is not None, dry_run=args.dry_run)
                                if not args.dry_run:
                                    send_script(movel_absolute_script(target_pose, args.speed, args.accel))
                                attempted = True
                    else:
                        rec.log("tick", t=flight_buffer[-1].t, n=result.n_samples, verdict="no_crossing",
                                note=result.note)

                if state == "idle" and last_state == "flight":
                    if history_head is not None:
                        print(f"--- throw ended ({history_head.reason}), dur={history_head.duration:.2f}s "
                              f"n={history_head.samples} peak={history_head.peak_speed:.2f}m/s ---")
                    end_tcp = list(rtde_r.getActualTCPPose())
                    rec.log("throw_end",
                            reason=history_head.reason if history_head else None,
                            duration=history_head.duration if history_head else None,
                            samples=history_head.samples if history_head else None,
                            peak_speed=history_head.peak_speed if history_head else None,
                            attempted=attempted, arm_tcp_at_end=end_tcp)
                    if attempted and not args.dry_run:
                        print("returning to wait pose...")
                        t0 = time.time()
                        settled, fault = move_to(wait_pose, args.approach_speed, args.approach_accel, rtde_r)
                        rec.log("move", purpose="return_to_wait", target=wait_pose,
                                speed=args.approach_speed, accel=args.approach_accel,
                                settled=settled, fault=fault, duration_s=time.time() - t0)
                        if fault is not None:
                            halt_on_fault(rec, fault)
                        if not settled:
                            print("WARNING: did not settle at wait pose within timeout (no fault reported) - "
                                  "check the arm before the next throw.")
                        print("at wait pose.\n")
                    else:
                        print()

                last_state = state
                time.sleep(1.0 / args.poll_hz)
        except KeyboardInterrupt:
            print("\nCtrl-C - sending stopl.")
            rec.log("run_end", reason="keyboard_interrupt")
            if not args.dry_run:
                try:
                    send_script(stopl_script())
                except Exception:
                    pass
        finally:
            client.stop_async()
            rtde_r.disconnect()
            rec.close()


if __name__ == "__main__":
    main()
