"""
track_ball_servo.py - first perception-driven use of the v2 servoj streaming layer
(ur_servo.py): the tool continuously follows the tracked ball's live position,
retargeting every tick instead of committing to one discrete move.

This is the mocap-in-the-loop successor to `ur_servo.py --bench` (canned motion,
no perception) and the streaming successor to `track_rigid_body.py` (mocap in the
loop, but driven by repeated speedl velocity commands). Relative to those two it
changes exactly one thing each time, on purpose - if it misbehaves, --bench
already told you whether the transport is sound, and track_rigid_body.py already
told you whether the transform is sound.

    track_ball.py            plot the raw ball position, no robot
    ur_servo.py --bench      servoj streaming, no perception
    track_rigid_body.py      perception -> speedl velocity commands
    track_ball_servo.py      perception -> servoj setpoint stream   <- this file
    catch.py (v2, later)     the same stream, driven by a PREDICTED intercept

WHAT THIS IS AND ISN'T. It follows the ball's CURRENT position - it is a pursuit,
not the race catch.py runs. A genuinely thrown ball cannot be pursued: it crosses
the workspace far faster than a ~1.2 m/s arm can reposition (CLAUDE.md hardware
section), and this script's rate limiter is deliberately set slower still. So the
intended exercise is a ball moved by HAND, slowly, through the catch zone. What
that actually validates is the thing v2 needs and nothing else has measured yet:
how well the arm tracks a continuously-moving mocap-derived setpoint, and with
how much lag. That lag number is the missing input to CLAUDE.md's step (2)
feasibility work - a moving-start intercept is only worth building if the arm can
follow a moving setpoint at all.

Do NOT throw balls at this script. It has no release detection, no prediction, no
feasibility gate, and no intercept logic - it would chase a throw at 25cm/s and
achieve nothing except being pointed the wrong way.

SAFETY. All three of ur_servo.py's layers apply (rate limiter, robot-side
watchdog, IK guard); on top of them this file adds, every tick:
  - a soft MAX_REACH ceiling on the desired target AND again on the rate-limited
    setpoint actually sent - track_rigid_body.py's model, reused here rather than
    catch.py's check_catch_envelope(). check_catch_envelope()'s reach/z/azimuth
    bands are tuned for a COMMITTED high-speed catch move (e.g. its 0.55m reach
    floor exists to keep a box swung at ~1 m/s clear of the arm's own body); this
    script is slow, hand-guided, perception-only tracking with no commit and no
    catch band, and reusing that gate here just held the arm any time the ball
    was moved anywhere near the base - exactly where a hand naturally holds it.
    No floor is enforced, matching track_rigid_body.py precedent (proven working):
    the joint-margin checks below are what actually guard against driving the box
    into the arm's own body at close reach.
  - track_rigid_body.py's joint position/speed margin checks, which turn a silent
    "joint near limit" controller lockup into an early, legible stop.
  - a bad-mocap-data sanity distance, and a stale-tracking timeout.
Every one of those failure paths sends an explicit hold (servo=0 -> the robot
stops and idles) rather than merely withholding packets - going silent for
--sock-timeout would end the robot's program entirely, which is a much bigger
event than "hold still for a moment".

The arm must already be near the wait pose when this starts (drive it there with
ur_goto_raw.py first) - see --start-tolerance. This is a deliberate,
consented-to-risk motion test: the operator is expected to be at the physical
E-stop the whole time. Ctrl-C or Enter stops cleanly.

Start with --dry-run: full perception + transform + envelope + limiter, printing
the setpoints it would stream, with no program ever sent to the robot.
"""

import argparse
import json
import os
import select
import sys
import threading
import time
from typing import List, Optional

import numpy as np
import rtde_receive
import dashboard_client
from natnet import NatNetClient, DataFrame

from frames import mocap_point_to_base, base_from_mocap_via_rigid_body, quat_to_matrix
from track_rigid_body import (
    load_transform, check_joint_margins, check_joint_speed_margins,
    JOINT_LIMIT_DEG, JOINT_WARN_MARGIN_DEG, JOINT_STOP_MARGIN_DEG,
    DEFAULT_BASE_SHOULDER_SPEED_LIMIT_DEG_S, DEFAULT_ELBOW_WRIST_SPEED_LIMIT_DEG_S,
    JOINT_SPEED_WARN_MARGIN_DEG_S, JOINT_SPEED_STOP_MARGIN_DEG_S,
    MAX_REACH,
)
from catch import (
    DEFAULT_WAIT_POSE, check_safety_mode, LastNormal, yaw_follow_orientation,
    resolve_live_base_rb_transform,
)
from verify_base_rb import BaseRBState, make_base_rb_handler, BASE_RB_LOCK
from ur_servo import RateLimiter, ServoStream, add_servo_args
from speed_char import linear_speed

# The ball's rigid-body id. Defaults to 3 for the same reason catch.py does:
# with base and tool rigid bodies also in the scene, auto-selection is no longer
# reliable (CLAUDE.md, 2026-07-17).
DEFAULT_BALL_RB_ID = 3

DEFAULT_BELOW = 0.20          # m below the ball centroid, base frame -Z - same
                               # semantics as track_rigid_body.py: park the box's
                               # mouth under the target rather than inside it.
DEADBAND = 0.004              # m - inside this, stop chasing positional noise
SANITY_MAX_DISTANCE = 1.5     # m from the current TCP - beyond this, treat the
                               # target as bad mocap data, not a real move (see
                               # CLAUDE.md: tracking_valid can't always be trusted
                               # to flip false on occlusion)
LOST_TRACKING_TIMEOUT = 0.35  # s - tighter than track_rigid_body.py's 1.0s: a
                               # streaming controller should notice a dropout
                               # within a few ticks, not a third of a second
SAFETY_CHECK_INTERVAL = 0.25  # s - the dashboard safety query is a blocking
                               # round-trip, far too slow to run every tick at
                               # 125Hz, so it is decimated to this instead
DEFAULT_START_TOLERANCE = 0.25  # m from the wait pose the arm may start at
STATUS_PRINT_INTERVAL = 0.25  # s between status-line refreshes. Every tick is
                               # RECORDED; only the console is throttled. Kept
                               # this slow because a \r-updated line still lands
                               # as a separate scrollback entry (only the
                               # on-screen line is overwritten) - at 125Hz an
                               # unthrottled print would bury the session, the
                               # same trap calibrate_frames.verify_live() hit on
                               # 2026-07-20.

LOG_DIR = "servo_logs"

STATE_LOCK = threading.Lock()

INSTRUCTIONS = """
Continuous ball tracking via servoj streaming
---------------------------------------------
The tool will FOLLOW the tracked ball (rigid body {rb}) continuously, staying
{below:.2f}m below it, at up to {speed:.2f} m/s.

Move the ball BY HAND, slowly. Do not throw it - this script pursues the ball's
current position and has no prediction of any kind.

This is real, continuous, unattended motion. Make sure the workspace is clear
and you are at the physical E-stop before starting. Enter or Ctrl-C stops.
"""


class BallState:
    """Latest ball observation, written by the NatNet socket-recv thread.

    Deliberately only the LATEST sample, not a buffer: this script pursues the
    current position and never fits anything, so there is nothing to accumulate.
    (That is also why the CLAUDE.md rule about keeping heavy fits off the
    socket-recv thread has nothing to bite on here - the callback does no work.)
    """

    def __init__(self):
        self.pos: Optional[tuple] = None
        self.valid = False
        self.seen_wall: Optional[float] = None
        self.sample_t: Optional[float] = None
        self.frames = 0
        self.candidates: List[int] = []


def make_handler(state: BallState, ball_id: int):
    def handle_frame(frame: DataFrame) -> None:
        with STATE_LOCK:
            state.frames += 1
            rb = next((r for r in frame.rigid_bodies if r.id_num == ball_id), None)
            if rb is None:
                state.candidates = [r.id_num for r in frame.rigid_bodies]
                return
            state.pos = rb.pos
            state.valid = True if rb.tracking_valid is None else rb.tracking_valid
            state.seen_wall = time.monotonic()
            state.sample_t = frame.suffix.timestamp

    return handle_frame


def enter_pressed() -> bool:
    """Non-blocking check for Enter on stdin - a select() poll rather than a
    reader thread, so nothing is left holding stdin after the loop exits (same
    approach and reason as catch.py's)."""
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        sys.stdin.readline()
        return True
    return False


class Recorder:
    """One compact JSON object per line, or a no-op when --record is absent.

    Exists because the measurement this script is FOR - how far the arm lags a
    continuously moving setpoint - can only be made after the fact, by diffing
    the commanded setpoint against the arm's actual pose tick by tick. Same
    format and spirit as catch.py's catch_logs/ (which made a whole session
    researchable without going back into Motive replay).
    """

    def __init__(self, path: Optional[str]):
        self.f = None
        if path:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            self.f = open(path, "w", buffering=1)

    def log(self, ev: str, **fields) -> None:
        if self.f is None:
            return
        rec = {"ev": ev, "wall": round(time.time(), 4)}
        for k, v in fields.items():
            if isinstance(v, np.ndarray):
                v = [round(float(x), 5) for x in v]
            elif isinstance(v, (list, tuple)):
                v = [round(float(x), 5) if isinstance(x, (int, float, np.floating)) else x for x in v]
            elif isinstance(v, (float, np.floating)):
                v = round(float(v), 5)
            rec[k] = v
        self.f.write(json.dumps(rec) + "\n")

    def close(self) -> None:
        if self.f is not None:
            self.f.close()
            self.f = None


def build_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--transform", default="UR10_T_base_from_mocap.json",
                   help="Calibration file written by calibrate_frames.py")
    p.add_argument("--base-rb-transform", default="UR10_T_base_from_baseRB.json",
                   help="Path to calibrate_base_rb.py's output (default UR10_T_base_from_baseRB.json, "
                        "on by default since 2026-08-24 - pass an empty string to fall back to the "
                        "static --transform instead). When set, base<-mocap is recomputed every tick "
                        "from a second tracked rigid body mounted on the robot's fixed base "
                        "(frames.base_from_mocap_via_rigid_body) instead of loaded once from "
                        "--transform - matches catch.py/demo.py, and lets the rig be physically "
                        "repositioned without rerunning calibrate_frames.py, as long as that rigid "
                        "body stays fixed to the base.")
    p.add_argument("--rigid-body-id", type=int, default=DEFAULT_BALL_RB_ID,
                   help=f"Ball rigid-body id (default {DEFAULT_BALL_RB_ID})")
    p.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    p.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP on the mocap subnet")
    p.add_argument("--unicast", action="store_true")
    p.add_argument("--below", type=float, default=DEFAULT_BELOW,
                   help=f"m below the ball centroid, base frame -Z (default {DEFAULT_BELOW})")
    p.add_argument("--wait-pose", type=float, nargs=6, default=list(DEFAULT_WAIT_POSE),
                   help="Reference pose: orientation held, position used as the envelope's azimuth reference")
    p.add_argument("--start-tolerance", type=float, default=DEFAULT_START_TOLERANCE,
                   help="m - refuse to start if the arm is further than this from the wait pose")
    p.add_argument("--max-reach", type=float, default=MAX_REACH,
                   help=f"m from the base - soft ceiling only, no floor (default {MAX_REACH}, "
                        f"track_rigid_body.py's value; NOT catch.py's catch-band reach)")
    p.add_argument("--yaw-follow", action="store_true",
                   help="Pan the tool orientation about base Z with the target's azimuth "
                        "(catch.py's default; opt-in here until validated in streaming)")
    p.add_argument("--dry-run", action="store_true",
                   help="Full pipeline, printing setpoints, with NO program sent and no motion")
    p.add_argument("--record", nargs="?", const="", default=None,
                   help=f"Write a per-tick JSONL to {LOG_DIR}/ (optionally give a path)")
    p.add_argument("--joint-limit-deg", type=float, default=JOINT_LIMIT_DEG)
    p.add_argument("--joint-warn-margin-deg", type=float, default=JOINT_WARN_MARGIN_DEG)
    p.add_argument("--joint-stop-margin-deg", type=float, default=JOINT_STOP_MARGIN_DEG)
    p.add_argument("--base-shoulder-speed-limit-deg-s", type=float,
                   default=DEFAULT_BASE_SHOULDER_SPEED_LIMIT_DEG_S)
    p.add_argument("--elbow-wrist-speed-limit-deg-s", type=float,
                   default=DEFAULT_ELBOW_WRIST_SPEED_LIMIT_DEG_S)
    p.add_argument("--joint-speed-warn-margin-deg-s", type=float, default=JOINT_SPEED_WARN_MARGIN_DEG_S)
    p.add_argument("--joint-speed-stop-margin-deg-s", type=float, default=JOINT_SPEED_STOP_MARGIN_DEG_S)
    add_servo_args(p)
    return p.parse_args()


def main():
    args = build_args()
    joint_speed_limits = ([args.base_shoulder_speed_limit_deg_s] * 2
                          + [args.elbow_wrist_speed_limit_deg_s] * 4)
    wait_pose = list(args.wait_pose)
    wait_xyz = np.array(wait_pose[:3])
    dt = 1.0 / args.rate

    R, t_vec, tcp_offset, tool_rb_id = load_transform(args.transform)
    if args.rigid_body_id == tool_rb_id:
        raise SystemExit(
            f"--rigid-body-id {args.rigid_body_id} is the ARM's own tool rigid body (per "
            f"{args.transform}). The arm would chase itself. Pass the ball's id instead."
        )

    # base_rb_mode: recompute base<-mocap every tick from a base-mounted rigid body's
    # LIVE pose instead of trusting the static --transform file, same as catch.py/
    # demo.py (see resolve_live_base_rb_transform's docstring - a real incident there
    # was exactly this file's stale-transform symptom: rig physically moved after
    # calibration, static R/t silently wrong, target lands nowhere near the ball).
    base_rb_mode = bool(args.base_rb_transform)
    R_base_rb = t_base_rb = base_rb_id = None
    if base_rb_mode:
        with open(args.base_rb_transform) as f:
            base_rb_data = json.load(f)
        R_base_rb = np.array(base_rb_data["R"])
        t_base_rb = np.array(base_rb_data["t"])
        base_rb_id = base_rb_data["rigid_body_id"]
        if args.rigid_body_id == base_rb_id:
            raise SystemExit(
                f"--rigid-body-id {args.rigid_body_id} is the base's own rigid body (per "
                f"{args.base_rb_transform}). Pass the ball's id instead."
            )
        print(f"[LIVE] getting one settled reading of base rigid body id={base_rb_id} now, so the "
              f"start pose isn't derived from a stale static transform...")
        R, t_vec = resolve_live_base_rb_transform(args.server_ip, args.local_ip, args.unicast,
                                                   base_rb_id, R_base_rb, t_base_rb)
        print(f"transform: LIVE via base RB id={base_rb_id} ({args.base_rb_transform}), "
              f"tool rigid body id={tool_rb_id}, tcp_offset={tcp_offset}")
    else:
        print(f"transform: {args.transform} (tool rigid body id={tool_rb_id}, tcp_offset={tcp_offset})")

    record_path = None
    if args.record is not None:
        record_path = args.record or os.path.join(
            LOG_DIR, f"servo_log_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    rec = Recorder(record_path)
    if record_path:
        print(f"recording to {record_path}")

    rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    dash = dashboard_client.DashboardClient(args.robot_ip)
    dash.connect()
    last_normal = LastNormal()

    start_pose = list(rtde_r.getActualTCPPose())
    print(f"arm start pose: {[round(v, 4) for v in start_pose]}")
    # Refuse to start from an arbitrary configuration. The envelope's azimuth band
    # and the rate limiter are both defined relative to the wait pose, and the
    # limiter seeds itself from wherever the arm IS - starting far away would mean
    # a long, unreviewed opening traverse before tracking even begins. Driving to
    # the wait pose is left to ur_goto_raw.py on purpose: that is a single move a
    # human reviews, which is exactly the right shape for it.
    start_offset = float(np.linalg.norm(np.array(start_pose[:3]) - wait_xyz))
    if start_offset > args.start_tolerance:
        raise SystemExit(
            f"Arm is {start_offset:.2f}m from the wait pose (tolerance {args.start_tolerance:.2f}m).\n"
            f"Drive it there first, e.g.:\n"
            f"  python3 ur_goto_raw.py --pose {' '.join(f'{v:.4f}' for v in wait_pose)} --speed 0.3 --accel 1.0\n"
            f"(add --force only after checking the printed target)."
        )

    fault = check_safety_mode(rtde_r, dash, last_normal)
    if fault is not None:
        raise SystemExit(f"Robot is not in a NORMAL safety mode: {fault}\n"
                         f"Clear it on the pendant (or ur_status.py --clear) first.")

    state = BallState()
    client = NatNetClient(server_ip_address=args.server_ip,
                          local_ip_address=args.local_ip,
                          use_multicast=not args.unicast)
    client.on_data_frame_received_event.handlers.append(make_handler(state, args.rigid_body_id))
    rb_state = BaseRBState()
    if base_rb_mode:
        client.on_data_frame_received_event.handlers.append(make_base_rb_handler(rb_state, base_rb_id))

    print(INSTRUCTIONS.format(rb=args.rigid_body_id, below=args.below, speed=args.max_speed))
    if args.dry_run:
        print(">>> DRY RUN: no program will be sent, the arm will not move. <<<\n")
    else:
        if input("Type 'go' once the workspace is clear and you're at the E-stop: ").strip().lower() != "go":
            raise SystemExit("aborted.")

    limiter = RateLimiter(start_pose[:3], args.max_speed, args.max_accel)
    stream: Optional[ServoStream] = None
    rec.log("run_start", transform=args.transform, base_rb_mode=base_rb_mode,
            base_rb_transform=args.base_rb_transform, base_rb_id=base_rb_id,
            ball_rb=args.rigid_body_id,
            wait_pose=wait_pose, below=args.below, rate=args.rate,
            max_speed=args.max_speed, max_accel=args.max_accel,
            servo_dt=args.servo_dt, lookahead=args.lookahead, gain=args.gain,
            yaw_follow=args.yaw_follow, dry_run=args.dry_run)

    ticks = 0
    late = 0
    holding = True          # start held: nothing is commanded until a good target arrives
    last_safety_check = 0.0
    last_print = 0.0
    stop_reason = None
    # Real achieved speed/accel, off getActualTCPSpeed() (same source speed_char.py
    # reads its true peak from) - NOT the same thing as --max-speed/--max-accel,
    # which only bound the COMMANDED setpoint. The rate limiter, servoj's own gain,
    # and the ball's own hand-guided motion can all keep the arm well under its
    # cap; this is how to see what actually happened.
    peak_speed = 0.0
    peak_accel = 0.0
    prev_speed_vec = None
    prev_speed_t = 0.0

    with client:
        client.run_async()
        try:
            if not args.dry_run:
                stream = ServoStream(tcp_offset, args.robot_ip, args.host_ip, args.host_port,
                                     args.servo_dt, args.lookahead, args.gain,
                                     args.stop_accel, args.sock_timeout)
                stream.start()

            while True:
                tick_start = time.monotonic()
                ticks += 1

                if enter_pressed():
                    stop_reason = "user_enter"
                    break

                # Decimated: the dashboard query is a blocking round-trip, so at
                # 125Hz it cannot run every tick. A protective stop that froze the
                # arm mid-servo would otherwise be indistinguishable from "the arm
                # is tracking badly" - the same masking that hid a real fault for
                # ~30s in the 2026-07-15 incident (docs/debug_log.md).
                if not args.dry_run and tick_start - last_safety_check >= SAFETY_CHECK_INTERVAL:
                    last_safety_check = tick_start
                    fault = check_safety_mode(rtde_r, dash, last_normal)
                    if fault is not None:
                        stop_reason = f"robot_fault: {fault}"
                        rec.log("fault", reason=fault)
                        break

                with STATE_LOCK:
                    pos, valid, seen_wall, sample_t = (
                        state.pos, state.valid, state.seen_wall, state.sample_t)
                    candidates = list(state.candidates)

                # Recompute base<-mocap from the base RB's LIVE pose (catch.py/demo.py
                # precedent). If it's not valid this tick (occlusion), keep the last-good
                # R/t rather than falling back to anything - stale-but-consistent for one
                # tick beats a discontinuous jump.
                if base_rb_mode:
                    with BASE_RB_LOCK:
                        rb_pos, rb_rot, rb_valid = rb_state.latest_pos, rb_state.latest_rot, rb_state.latest_valid
                    if rb_valid:
                        R_mocap_rb = quat_to_matrix(rb_rot)
                        R, t_vec = base_from_mocap_via_rigid_body(R_base_rb, t_base_rb, R_mocap_rb, np.array(rb_pos))

                actual_pose = rtde_r.getActualTCPPose()
                actual_xyz = np.array(actual_pose[:3])
                actual_speed_vec = rtde_r.getActualTCPSpeed()
                actual_speed = linear_speed(actual_speed_vec)
                actual_accel = 0.0
                if prev_speed_vec is not None and tick_start > prev_speed_t:
                    actual_accel = linear_speed(
                        [(actual_speed_vec[i] - prev_speed_vec[i]) / (tick_start - prev_speed_t)
                         for i in range(3)])
                prev_speed_vec, prev_speed_t = actual_speed_vec, tick_start
                peak_speed = max(peak_speed, actual_speed)
                peak_accel = max(peak_accel, actual_accel)

                hold_reason = None
                stale = seen_wall is None or (tick_start - seen_wall) > LOST_TRACKING_TIMEOUT
                if pos is None:
                    hold_reason = (f"ball rigid body {args.rigid_body_id} not seen"
                                   + (f" (visible ids: {candidates})" if candidates else ""))
                elif not valid:
                    hold_reason = "tracking_valid=False"
                elif stale:
                    hold_reason = "tracking stale"

                pos_stop, pos_warn = check_joint_margins(
                    rtde_r.getActualQ(), args.joint_limit_deg,
                    args.joint_warn_margin_deg, args.joint_stop_margin_deg)
                speed_stop, speed_warn = check_joint_speed_margins(
                    rtde_r.getActualQd(), joint_speed_limits,
                    args.joint_speed_warn_margin_deg_s, args.joint_speed_stop_margin_deg_s)
                joint_warnings = pos_warn + speed_warn
                if pos_stop or speed_stop:
                    # Takes priority over anything mocap says: a joint this close to
                    # its limit is a robot-state fact, and the whole point is to stop
                    # before the controller's own protective stop does.
                    hold_reason = f"joint limit close - {', '.join(joint_warnings)}"

                desired = None
                if hold_reason is None:
                    p_base = mocap_point_to_base(np.array(pos), R, t_vec)
                    desired = p_base.copy()
                    desired[2] -= args.below
                    if float(np.linalg.norm(desired - actual_xyz)) > SANITY_MAX_DISTANCE:
                        hold_reason = (f"target {np.linalg.norm(desired - actual_xyz):.2f}m from the "
                                       f"arm - treating as bad mocap data")
                    else:
                        reach = float(np.linalg.norm(desired))
                        if reach > args.max_reach:
                            hold_reason = f"reach {reach:.2f}m beyond the {args.max_reach:.2f}m soft reach limit"

                if hold_reason is not None:
                    # An explicit hold, never silence: going quiet for --sock-timeout
                    # would end the robot's program outright, which is a far larger
                    # event than pausing. Re-seed the limiter from the arm's real
                    # position so resuming doesn't step from a stale setpoint.
                    if not holding:
                        limiter.reset(actual_xyz)
                        holding = True
                    if stream is not None:
                        stream.send(list(limiter.cmd) + wait_pose[3:6], servo=False)
                    rec.log("hold", reason=hold_reason, actual=actual_xyz, t=sample_t,
                            actual_speed=actual_speed, actual_accel=actual_accel)
                    if tick_start - last_print >= STATUS_PRINT_INTERVAL:
                        last_print = tick_start
                        print(f"\rHOLD: {hold_reason[:70]:<70}  "
                              f"speed={actual_speed:.2f}m/s peak={peak_speed:.2f}m/s{'':10}",
                              end="", flush=True)
                else:
                    holding = False
                    err = float(np.linalg.norm(desired - limiter.cmd))
                    target = limiter.cmd if err < DEADBAND else desired
                    cmd_xyz = limiter.step(target, dt)

                    # Belt-and-braces: the limiter output is what actually gets sent,
                    # so it is re-checked independently of the desired-target check
                    # above. A limiter bug (or a reset from an already-bad position)
                    # cannot slip a target past both.
                    cmd_reach = float(np.linalg.norm(cmd_xyz))
                    if cmd_reach > args.max_reach:
                        limiter.reset(actual_xyz)
                        holding = True
                        if stream is not None:
                            stream.send(list(actual_xyz) + wait_pose[3:6], servo=False)
                        rec.log("hold", reason=f"limited setpoint reach {cmd_reach:.2f}m beyond "
                                f"{args.max_reach:.2f}m soft reach limit", cmd=cmd_xyz, t=sample_t)
                        print(f"\rHOLD: limited setpoint reach {cmd_reach:.2f}m beyond "
                              f"{args.max_reach:.2f}m soft reach limit{'':30}", end="", flush=True)
                    else:
                        orient = (yaw_follow_orientation(wait_pose, wait_xyz, cmd_xyz)
                                  if args.yaw_follow else wait_pose[3:6])
                        setpoint = list(cmd_xyz) + list(orient)
                        if stream is not None:
                            stream.send(setpoint, servo=True)
                        lag = float(np.linalg.norm(actual_xyz - cmd_xyz))
                        rec.log("tick", t=sample_t, ball=np.array(pos), desired=desired,
                                cmd=cmd_xyz, actual=actual_xyz, lag=lag,
                                to_go=float(np.linalg.norm(desired - cmd_xyz)),
                                actual_speed=actual_speed, actual_accel=actual_accel)
                        if tick_start - last_print >= STATUS_PRINT_INTERVAL:
                            last_print = tick_start
                            warn = f"  JOINT WARN: {', '.join(joint_warnings)}" if joint_warnings else ""
                            print(f"\rcmd=({cmd_xyz[0]:+.3f},{cmd_xyz[1]:+.3f},{cmd_xyz[2]:+.3f})  "
                                  f"to_go={np.linalg.norm(desired - cmd_xyz) * 100:5.1f}cm  "
                                  f"lag={lag * 1000:5.1f}mm  reach={np.linalg.norm(cmd_xyz):.2f}m  "
                                  f"speed={actual_speed:.2f}m/s peak={peak_speed:.2f}m/s  "
                                  f"accel_peak={peak_accel:.1f}m/s2  "
                                  f"late={late}{warn}        ", end="", flush=True)

                sleep_for = dt - (time.monotonic() - tick_start)
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    late += 1
        except KeyboardInterrupt:
            stop_reason = "keyboard_interrupt"
        finally:
            print(f"\n\nstopping ({stop_reason or 'unknown'})...")
            if stream is not None:
                stream.stop()
            rec.log("run_end", reason=stop_reason or "unknown", ticks=ticks, late=late,
                    peak_speed=peak_speed, peak_accel=peak_accel)
            rec.close()
            client.stop_async()
            rtde_r.disconnect()
            dash.disconnect()

    print(f"{ticks} ticks, {late} late ({100.0 * late / max(ticks, 1):.1f}%), "
          f"mocap frames seen: {state.frames}")
    print(f"peak ACTUAL TCP speed: {peak_speed:.2f} m/s, peak ACTUAL accel: {peak_accel:.1f} m/s^2 "
          f"(measured via getActualTCPSpeed(), not the commanded --max-speed/--max-accel caps)")
    if record_path:
        print(f"log: {record_path}")


if __name__ == "__main__":
    main()
