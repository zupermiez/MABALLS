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
    --catch-move servo is a further exception (2026-07-22): it commits at
    --early-commit-samples (default 5, not --commit-samples' 40) with the feasibility
    gate bypassed entirely, since its continuous retargeting corrects the noisy early
    guess every tick afterward - see --early-commit-samples' help for the reasoning.

SAFETY. A catch move is inherently larger than ur_goto_raw.py's 0.15m/axis clamp,
so this script does NOT reuse that clamp or spam --force. Instead every commanded
target is checked against a dedicated, conservative catch envelope
(check_catch_envelope): a base-frame reach band, a z band that keeps the tool off
the platform deck / floor and out of the overhead singularity, and a hard cap on
how far the target may be from the wait pose. A target failing ANY of these is
refused and no motion is sent. Motion uses the proven raw-URScript-over-socket path
(port 30002), fire-and-forget, same as ur_goto_raw.py - there is no persistent
ur_rtde control session here (rtde_receive is read-only), so the "never force-kill
a control session" hazard does not apply; Enter (clean stop) or Ctrl-C (emergency
abort) both send a stopl and exit the same way.

A detected robot fault (protective stop etc.) no longer hard-kills the session, but
this script does NOT auto-clear it either - wait_for_fault_clear() just polls and
blocks until a human clears it on the pendant (or via ur_status.py --clear), then
drives back to the wait pose and resumes automatically. Deliberately no
dashboard-server auto-unlock here: clearing a fault is a decision that belongs to
whoever is standing next to the arm, not a script. Ctrl-C still works at any point,
including while waiting on a fault.

Run `python3 catch.py --help`. Start with `--dry-run` (no motion at all - logs
every decision) to validate the wait pose, derived plane, and gating against your
replay before enabling motion.
"""

import argparse
import json
import math
import os
import re
import select
import socket
import subprocess
import sys
import time
import traceback
from collections import deque
from typing import List, Optional

import beep
import dashboard_client
import numpy as np
import rtde_receive
from natnet import NatNetClient, DataFrame

from live_trajectory import STATE_LOCK, SharedState, add_release_detection_args, make_handler
from trajectory import AXIS_NAMES, trim_ghost_tail
from frames import mocap_point_to_base, quat_to_matrix, base_from_mocap_via_rigid_body
from ur_goto_raw import (ROBOT_IP, SECONDARY_PORT, send_script, movel_absolute_script,
                          movej_to_pose_script, movej_script, wait_for_stop)
from calibrate_frames import send_urscript, set_tcp_script
from verify_base_rb import BaseRBState, make_base_rb_handler, BASE_RB_LOCK
from calibrate_base_rb import CaptureState, make_handler as make_base_rb_capture_handler, average_pose
from ur_servo import (
    RateLimiter, OrientationRateLimiter, ServoStream, HOST_IP as SERVO_HOST_IP,
    HOST_PORT as SERVO_HOST_PORT, DEFAULT_GAIN as SERVO_DEFAULT_GAIN,
    DEFAULT_LOOKAHEAD as SERVO_DEFAULT_LOOKAHEAD, DEFAULT_SERVO_DT, DEFAULT_SOCK_TIMEOUT,
    DEFAULT_STOP_ACCEL, rotvec_to_quat, quat_to_rotvec, quat_multiply,
)
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
#
# --- 2026-08-25: raised 0.45->0.55m, user directive after a real near-self-collision
# on the UR10 (had to e-stop). That session (catch_logs/catch_log_20260825_131509.jsonl,
# throw 4) never actually sent a target under 0.4504m - check_catch_envelope() held the
# 0.45m floor exactly, refusing every retarget candidate below it (one predicted point
# at 0.408m was rejected outright) - but the arm still got close enough to worry about.
# Root cause: `reach` here is a bare distance from the base ORIGIN to the commanded TCP
# point - it doesn't inflate for the box's own geometry (box_radius, ~0.15m) or account
# for tool orientation (tilt/yaw-follow can point the box back toward the arm's own
# links at a given XYZ), so 0.45m of "reach" did not translate to 0.45m of real
# clearance. Raising the floor is a blunt fix for that gap, not a geometric one - still
# no swept-volume/orientation-aware check exists. 0.55m stays well clear of
# DEFAULT_WAIT_POSE's own reach on this arm (0.641m), so the wait pose can't reject
# itself the way the 2026-07-15 note above once had to watch for.
CATCH_MIN_REACH = 0.55   # m from base - inside this is near-singular / too close to the body
CATCH_MAX_REACH = 1.20   # m from base - was 1.00; user directive 2026-07-15: attempt catches out to 1.20m
# CATCH_Z_MIN raised -0.25->0.119 2026-07-27: a real collision (paint scraped off the
# wrist3 housing against the base's mounting stand, C157A1 fault) was traced to a
# commit/return-to-wait target that dropped to z=-0.09m - the old floor let the tool
# go 39cm below DEFAULT_WAIT_POSE's z (0.139m). User confirms (from the physical rig)
# that AT the wait-pose height, close reach only risks brushing the base joint itself
# (minor); going even 5cm below that height is what brings the tool into the stand's
# footprint. New floor is DEFAULT_WAIT_POSE z minus a 2cm buffer under that 5cm
# danger mark (0.139 - 0.02 = 0.119), applied as a flat cutoff regardless of reach -
# this trades away catching low throws far from the base (which were never actually
# near the stand) for a much simpler, physically-grounded rule. CATCH_MIN_REACH left
# unchanged per the same reasoning: it isn't what caused this incident (raised
# separately, see the 2026-08-25 note above).
#
# --- 2026-08-24 UR10 (CB3) migration: overwritten in place for the new arm/location,
# per user directive (git-recoverable, not branched - see docs/ur10_migration_roadmap.md).
# CATCH_MIN_REACH/CATCH_MAX_REACH left at the UR12e's values (same 1300mm reach spec,
# user directive to reuse). CATCH_Z_MIN set to 0.05m ("5cm off the arm's base") - a
# placeholder, not survey-derived like the UR12e's 0.119m was; the UR12e's floor only
# exists because of a real collision with THAT stand's specific geometry, so this
# number needs the same kind of physical-clearance check once the new mount/stand is
# actually looked at, not just assumed safe by analogy. CATCH_Z_MAX set equal to
# CATCH_MAX_REACH so it never binds ("no z max needed") - 3D reach (sqrt(x^2+y^2+z^2)
# <= CATCH_MAX_REACH) already bounds |z| on its own, so this is a no-op cap rather than
# a removed check.
CATCH_Z_MIN = 0.15       # m base-frame - raised from 0.05 placeholder after physical check at the
                          # arm on 2026-08-24 (user observed clearance at 0.05, moved floor up for margin)
CATCH_Z_MAX = CATCH_MAX_REACH  # m base-frame - effectively uncapped, see 2026-08-24 note above
# MAX_CATCH_MOVE (was 0.60m, capped distance from the wait pose) removed 2026-07-15 per
# user directive - only the reach/z band above now bounds a catch target, so any target
# within CATCH_MAX_REACH is attempted regardless of distance from the wait pose.

# Robot-side watchdog for demo.py's own stream, deliberately a bit larger than
# ur_servo.DEFAULT_SOCK_TIMEOUT (0.3s) - NOT a change to that shared library
# default (other scripts still get 0.3s). 2026-08-26 real incident: a single
# slow tick anywhere in this loop (a blocking Dashboard Server round-trip is
# the leading suspect - see check_safety_mode()'s call site) exceeded 0.3s
# while the robot was never actually faulted, tripping the robot-side watchdog
# and silently ending its program with no fault ever recorded. This buys the
# host more slack against exactly that kind of one-off stall without touching
# fault-detection cadence (SAFETY_CHECK_MIN_INTERVAL_S, deliberately unchanged
# - see its own comment) or weakening the actual host-crash protection much:
# a genuinely dead host is still caught within half a second.
DEMO_SOCK_TIMEOUT_S = 0.5

# A servo_hold (see emit_setpoint) that outlives this is worth escalating loudly: it
# means the limiter's own reach/z braking isn't resolving the violation on its own tick
# over tick, which is exactly the silent-stall shape of the 2026-07-27 incident. Same
# duration as DEMO_SOCK_TIMEOUT_S - by then it's taken as long as plain
# silence would need to kill the setpoint stream outright.
SERVO_HOLD_STUCK_S = DEMO_SOCK_TIMEOUT_S

# Chase-abort (2026-07-27): a committed throw whose feasibility margin stays negative
# and keeps failing to improve, tick over tick, is one that was never catchable - the
# continuous servo retarget (and, for movel/movej, re-aim) would otherwise keep pulling
# the setpoint toward a target that's still nominally inside the reach/z/azimuth
# envelope (check_catch_envelope) but whose TIME budget is only getting worse, chasing
# a degrading/noisy fit right up to the moment of impact. Real incident 2026-07-27: a
# throw ~70-90cm outside the reachable zone the whole flight (margin -0.3s to -0.9s,
# never improving) was chased this way into a hard robot-side fault (C306A3, invalid
# accel target) and a full power/brake cutoff (C305A15/16/17) - see docs/debug_log.md.
# Once the post-commit margin has gone this long without improving, the throw is
# abandoned: the setpoint simply stops being updated (holds where it is) for the rest
# of that throw. This is a strictly more conservative behavior than today's (which
# chases until the ball lands or a fault intervenes), so it defaults on.
# Wall-clock seconds, NOT a tick count (that's how this was originally written - 8
# ticks, "~0.16s at the default --poll-hz 50" - but --poll-hz's real default in servo
# mode is --servo-rate, 125, so 8 ticks was already only ~64ms; and a raw tick count
# silently redefines itself every time the loop rate changes, which is exactly the
# footgun --servo-rate/--poll-hz decoupling (2026-07-27, see docs/debug_log.md) was
# introduced to remove). Tracked via a monotonic start-of-streak timestamp instead of
# a counter - see non_improving_since below - so the tolerance means the same thing
# at 125Hz, 250Hz, or 500Hz.
ABORT_NON_IMPROVING_S = 0.16    # short, but long enough that normal per-tick fit noise
                                # doesn't false-trigger
ABORT_MARGIN_EPS = 0.005        # s - a margin improvement smaller than this still counts
                                # as "not improving" (ignores sub-tick noise in the fit)

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
# Re-taught again 2026-07-17 night shift (same orientation, adjusted x/y) as the new
# operating-point pose used alongside --catch-move movej/--yaw-follow.
#
# 2026-08-24 UR10 (CB3) migration: overwritten in place with this arm's current pose
# at the new location (captured via ur_get_pose.py, user directive - just wherever the
# arm happened to be, not a deliberately-taught singularity-avoiding pose like the
# UR12e one above was). Not yet checked for proximity to a wrist/shoulder singularity -
# do that before relying on it for real catch moves.
DEFAULT_WAIT_POSE = (0.1583, -0.5783, 0.2267, 1.1951, -1.1218, 1.1215)

# --record output goes here, not cwd - keeps the repo root from filling up with one
# file per session the way speed_char_*.json/png already do.
CATCH_LOG_DIR = "catch_logs"

# ~/.ssh/config alias set up for the UR controller (key-auth, see docs/debug_log.md) -
# used ONLY by wrap_up_session()/pull_robot_session_logs(), which runs after the
# NatNet client and RTDE connection are fully torn down. Never called from the hot
# loop - see the module docstring's threading rule.
ROBOT_SSH_HOST = "ur12e"
ROBOT_LOG_SESSIONS_DIR = os.path.join("robot_logs", "sessions")

# How often to remind the user we're still waiting on a fault to be cleared, and how
# often to re-check (see wait_for_fault_clear).
FAULT_WAIT_POLL_S = 0.5
FAULT_WAIT_REMINDER_S = 15.0

# --- --catch-move servo (2026-07-20) -------------------------------------------
# Base-joint rate cap for the servo rate limiter, deg/s. The measured cause of the
# recurring C153A0 protective stops was a Cartesian path demanding base-joint speed
# = TCP speed / reach, exceeding the 120 deg/s limit on side targets (137-195 deg/s
# at the real faulting commits). movej avoided this by planning in joint space;
# servoj does NOT, because the stream still prescribes a Cartesian path - so the
# constraint is imposed host-side by RateLimiter's tangential cap. 110 leaves ~8%
# margin under the documented limit.
SERVO_BASE_RATE_DEG_S = 110.0
# The safety-mode check is a blocking Dashboard round-trip. At --poll-hz 50 it ran
# once per tick; servo mode polls at 125Hz, where that would be 125 round-trips a
# second. Capping it at 50Hz keeps the existing behaviour identical at the old poll
# rate while stopping it from scaling with the servo rate.
SAFETY_CHECK_MIN_INTERVAL_S = 0.02

# 2026-08-26: a servo stream can die from a plain send failure (robot-side watchdog
# ended its own program) with NO safety-mode fault ever surfacing - the fault-clear
# branch above is the only thing that reopens the stream, and it never runs in that
# case, so the arm silently sat frozen for the rest of a real session's throw 2. See
# the no-fault reconnect at this file's main loop (searches for this constant). Not
# per-tick (that would resend the URScript program in a tight retry storm if the
# robot stays unreachable) - once per this many seconds is plenty since a healthy
# reconnect is near-instant and a failed one already blocks ~8s (ServoStream.start's
# accept_timeout) on its own.
STREAM_RECONNECT_MIN_INTERVAL_S = 1.0

# --- Idle wobble (2026-08-25, demo.py-only, --catch-move servo only) -----------
# Purely cosmetic "catcher waiting" tell so an audience can see the stream is
# live and not frozen: while genuinely idle, servo_target orbits a small circle
# around the wait pose instead of sitting dead still. NOT yet run on the real
# arm when written - started opt-in per the convention used throughout this
# file (--tilt-follow/--reaim-preempt/--catch-move servo itself all began
# opt-in), and promoted to ON BY DEFAULT 2026-09-11 on user request; disable
# with --no-idle-wobble.
#
# Deliberately slow/small enough to sit far under the servo RateLimiter's own
# speed/accel caps regardless of which regime (chase vs. return) happens to be
# active when idle - the wobble must never be what binds those, since a bound
# wobble would fight the limiter's real job (bounding CATCH motion) for no
# reason. At the defaults below: peak speed = 2*pi*r/T = 2*pi*0.075/6.0 =~
# 0.079 m/s, centripetal accel = r*omega^2 =~ 0.082 m/s^2 - both a small
# fraction of even the slowest catch cap (--servo-max-speed 1.2,
# --servo-max-accel 6.0, and less again after --servo-return-mult scales them
# down on the idle/return leg), so the limiter tracks the circle with
# negligible lag rather than fighting it.
IDLE_WOBBLE_DIAMETER_M = 0.15   # ~15cm circle, per user directive
IDLE_WOBBLE_PERIOD_S = 6.0      # seconds per full lap - slow, visible, low-energy


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
    feasibility tick, gate/commit/refuse decision, throw start/end, robot move, and
    (as a "throw_samples" event) the raw (t,x,y,z) ball trajectory for the throw this
    conductor makes. Deliberately flat and unpretty-printed (not a nested/
    human-formatted log) so a later "what happened on throw N" question can be
    answered by grepping/jq-filtering this file for `"throw":N` rather than reading
    a whole run - the point is being cheap for an agent to search, not to look nice.
    The one exception is "throw_samples", which is intentionally bulkier (the full
    per-frame trajectory) so a session can be researched later from this file alone -
    no Motive replay needed - but is its own event type so a plain grep/jq filter for
    the small event types (tick/commit/throw_end/...) doesn't have to wade through it.

    `t` is the NatNet/Motive sample timestamp (the same clock a Motive replay of
    this session re-streams) and `throw` is a 1-based ordinal - both are offered as
    correlation keys against a Motive replay of the same run, since it isn't known
    up front whether Motive's replay preserves original frame timestamps exactly or
    rebases them from zero; `throw` (count the Nth throw in the replay, in order) is
    the robust fallback either way.

    The very first line of every log is always a "run_start" event carrying the
    complete resolved config for that run (every flag, default or overridden alike -
    see main()'s rec.log("run_start", ...) call). Defaults change over time (e.g.
    --tilt-follow/--min-release-dist/--reaim-preempt all changed default-vs-override
    status across sessions on 2026-07-20) and are NOT visible from tick/commit/
    throw_end lines alone - a session-to-session comparison (or catch-rate diff) that
    skips diffing run_start against the CURRENT argparse defaults will silently
    misattribute behavior to the wrong cause. Always read run_start first.

    `robot_clock` is `rtde_r.getTimestamp()` - the UR controller's own "time elapsed
    since the controller was started" counter, sampled at the same instant as `wall`
    on every line (once `attach_rtde()` has been called - see main()). This is the
    EXACT same clock as the "Timestamp [s]" column in a flight report's
    `realtimedata.csv` (both come from the controller's real-time process), so any
    JSONL line can be matched to a row in a pulled `recording*.zip` by robot_clock
    directly, with no wall<->robot-clock offset to reverse-engineer - see
    docs/robot_log_formats.md. Added 2026-07-16 after a real investigation
    (docs/debug_log.md) had to derive that offset empirically, by finding one known
    event and back-solving, because nothing recorded both clocks at once.

    Flushed after every line (not buffered) so a protective stop or Ctrl-C never
    loses the tail of a session - that's exactly the run you'd want to debug most.
    """

    def __init__(self, path: Optional[str]):
        self.f = open(path, "a") if path else None
        self.throw = 0
        self.rtde_r = None  # set via attach_rtde() once the RTDE connection exists

    def attach_rtde(self, rtde_r) -> None:
        """Call once, right after connecting - see the robot_clock note above. Not
        passed to __init__ because Recorder is constructed before the RTDE
        connection exists (so run_start can log record-file setup itself)."""
        self.rtde_r = rtde_r

    def log(self, ev: str, t: Optional[float] = None, **fields) -> None:
        if self.f is None:
            return
        robot_clock = self.rtde_r.getTimestamp() if self.rtde_r is not None else None
        rec = {"wall": round(time.time(), 3), "robot_clock": rnd(robot_clock),
               "t": rnd(t), "throw": self.throw, "ev": ev}
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


# Cheap proxy for "that was a long reach, not a precision miss" - straight-line
# distance from the wait pose to the committed target, base-frame, no fit/lookup
# involved. Doesn't need to be precise (see explain_miss), just cheap: one
# np.linalg.norm on two points explain_miss already has in hand.
EXCUSE_FAR_THROW_M = 0.35


def explain_miss(guard_reason: Optional[str], attempted: bool, abandoned: bool,
                  refuse_reason: Optional[str], refuse_target: Optional[np.ndarray],
                  result, history_head, ball_last_dist: Optional[float],
                  committed_target: Optional[np.ndarray] = None,
                  wait_xyz: Optional[np.ndarray] = None) -> str:
    """demo.py only: a best-guess, data-backed one-liner for why a throw wasn't
    caught - pure console flavor for the demo audience, not a control input or
    anything logged as ground truth (the real mechanics are already in
    guard_reason/refuse_reason/result/rec.log). Follows the same decision
    chain the main loop itself used to reach "no catch", in order: release
    guard (never even eligible to commit) -> envelope refusal (commit was
    tried and rejected) -> no plane crossing / lost tracking (nothing to
    commit to) -> chase-abort (committed, then fell behind) -> far throw
    (committed, arrived, but the target was a long reach from wait pose) ->
    plain miss distance (committed, arrived, still missed). Deliberately
    blunt, second-person, and short - "you threw too far", not "target reach
    exceeded envelope bound" - keep new branches in that voice."""
    if guard_reason is not None:
        if "hand/handled" in guard_reason:
            return "that was a handoff, not a throw."
        return "that one wasn't even thrown at me."

    if not attempted:
        if refuse_reason is not None and refuse_target is not None:
            reach = float(np.linalg.norm(refuse_target))
            z = float(refuse_target[2])
            if reach > CATCH_MAX_REACH:
                return "you threw too far."
            if reach < CATCH_MIN_REACH:
                return "you threw too close."
            if z > CATCH_Z_MAX:
                return "you threw too high."
            if z < CATCH_Z_MIN:
                return "you threw too low."
            return "you threw too far to the side."
        if result is not None and result.crossing_t is None:
            return "that throw never reached my catch height."
        if history_head is not None and history_head.reason == "lost tracking":
            return "I lost sight of the ball."
        return "I didn't get a good read on that one."

    if abandoned:
        return "you threw that too fast."

    if (committed_target is not None and wait_xyz is not None
            and float(np.linalg.norm(committed_target - wait_xyz)) > EXCUSE_FAR_THROW_M):
        return "you threw too far."

    if ball_last_dist is not None:
        return "close, but I missed it."
    return "not sure what happened - just missed it."


# Real bug found 2026-08-25 (first UR10 session with base_rb_mode live):
# Motive's rigid-body solver can briefly latch onto a stray reflective point
# once the ball's own markers are occluded entering the box, and keeps
# reporting tracking_valid=True the whole time (the same caveat already known
# for flight-end detection - see live_trajectory.py's stream comments). Two
# real throws that session landed the ball in the box, then had raw_samples
# end with an instantaneous ~1.1m jump to the SAME frozen point
# (-0.709, 0.203, -0.467 mocap, both throws, within a few mm) instead of
# continuing the ball's smooth path - so the caught/miss heuristic below,
# which trusted raw_samples[-1] outright, measured "ball last seen" against a
# ghost point and logged a false miss on two good catches. Ghost-tail
# detection itself now lives in trajectory.trim_ghost_tail() (imported above,
# with its MAX_BALL_JUMP_M threshold) - shared with catch.py,
# visualize_trajectory.py, and anything else that draws or measures against a
# sample buffer's tail.


def last_plausible_ball_sample(raw_samples: list):
    """Return the raw_samples entry to trust as "where the ball was last seen" -
    the true last sample, unless an implausible single-frame jump (see
    trajectory.trim_ghost_tail) occurred somewhere in the tail, in which case
    return the sample right before the most recent such jump instead of the
    ghost point(s) after it. No-op (returns raw_samples[-1]) on a normal,
    continuously-tracked flight."""
    trimmed = trim_ghost_tail(raw_samples)
    return trimmed[-1] if trimmed else None


def stopl_script(decel: float = 3.0) -> str:
    return f"def prog():\n  stopl({decel})\nend\nprog()\n"


# --- Small quaternion helpers for --yaw-follow (rotate the wait orientation about
# base Z by the target's azimuth delta). Quaternion-based on purpose: direct
# axis-angle composition via rotation matrices needs the fragile theta~pi
# edge case handled; quaternions don't. Helpers (rotvec_to_quat etc.) live in
# ur_servo.py, imported above - OrientationRateLimiter needs the same math,
# not duplicated here anymore. ---

def yaw_follow_orientation(wait_pose: List[float], wait_xyz: np.ndarray,
                           target_xyz: np.ndarray) -> List[float]:
    """Rotate the wait pose's orientation about base Z by the azimuth delta from
    the wait position to the catch target, so the (rotationally symmetric,
    mouth-up) catch tool pans WITH the base instead of the wrist fighting to hold
    a fixed world orientation through the sweep. For an upward-facing box/funnel,
    yaw about vertical is a free variable - using it keeps the wrist configuration
    constant relative to the arm's own plane, which is exactly the well-conditioned
    thing. Motivation is the 2026-07-16 fault data: fault rate rose monotonically
    with target azimuth swing (3% under 10deg, 62% at 20-35deg) - see
    docs/debug_log.md 2026-07-17.
    """
    az_wait = math.atan2(float(wait_xyz[1]), float(wait_xyz[0]))
    az_tgt = math.atan2(float(target_xyz[1]), float(target_xyz[0]))
    d_az = math.atan2(math.sin(az_tgt - az_wait), math.cos(az_tgt - az_wait))
    q_yaw = np.array([math.cos(d_az / 2), 0.0, 0.0, math.sin(d_az / 2)])
    q_wait = rotvec_to_quat(wait_pose[3:6])
    rv = quat_to_rotvec(quat_multiply(q_yaw, q_wait))
    return [float(rv[0]), float(rv[1]), float(rv[2])]


def tilt_follow_orientation(base_rv: List[float], impact_vel_base: np.ndarray,
                            max_tilt_rad: float) -> List[float]:
    """Tilt the tool's mouth INTO the incoming ball trajectory: rotate the given
    orientation (world frame) so the mouth normal (assumed base +Z, mouth-up)
    leans toward the direction the ball arrives FROM (-v_impact), capped at
    max_tilt_rad. Functional motivation: a slanted approach sees the mouth
    aperture foreshortened by cos(incidence angle) - tilting recovers effective
    aperture exactly where the error budget is tightest. Demo motivation: the
    wrist visibly 'reaches into' the throw (yaw-follow alone deliberately
    KEEPS wrist joints still - see yaw_follow_orientation - which is why
    2026-07-17's yaw-follow sessions showed no visible wrist action).
    EXPERIMENTAL (2026-07-18 night shift, opt-in via --tilt-follow): not yet
    validated on the real arm - IK reachability of tilted poses near the
    envelope edge is the thing to watch first."""
    v = np.asarray(impact_vel_base, dtype=float)
    vn = float(np.linalg.norm(v))
    if vn < 1e-6:
        return list(base_rv)
    n_des = -v / vn                      # unit vector pointing back along the incoming path
    z = np.array([0.0, 0.0, 1.0])
    cosang = float(np.clip(np.dot(z, n_des), -1.0, 1.0))
    full = math.acos(cosang)
    axis = np.cross(z, n_des)
    an = float(np.linalg.norm(axis))
    if an < 1e-9 or full < 1e-6:
        return list(base_rv)             # ball falling straight down - mouth-up already ideal
    angle = min(full, max_tilt_rad)
    q_tilt = np.concatenate([[math.cos(angle / 2)], (axis / an) * math.sin(angle / 2)])
    q_base = rotvec_to_quat(base_rv)
    rv = quat_to_rotvec(quat_multiply(q_tilt, q_base))
    return [float(rv[0]), float(rv[1]), float(rv[2])]


def check_release_guard(flight_buffer: List, R: np.ndarray, t_vec: np.ndarray,
                        min_release_dist: float, max_away_deg: float = 100.0,
                        min_horiz_speed: float = 1.0) -> Optional[str]:
    """None if this throw's release looks like a real throw AT the robot, else a
    reason string - evaluated once per throw, right at release detection, before
    any feasibility tick may commit.

    Two independent checks, both designed against the 161 recorded real throws of
    2026-07-16 (docs/debug_log.md 2026-07-17):
    - Release origin distance: every real throw released >=1.1m (horizontal) from
      the base (p5=1.14m, median 2.16m); the events under 1.0m were all a hand
      handling the ball near the robot - including the one that made the arm
      commit to a 125deg-azimuth target and self-collide (session 151342 throw 9,
      "grabbed the ball out of the box"). A person simply cannot be throwing from
      inside the arm's own workspace.
    - Direction: a throw with real horizontal speed (> min_horiz_speed) whose
      velocity points >max_away_deg away from the base direction is moving AWAY
      from the robot - never catchable, but its fit can still produce a
      plane-crossing behind/beside the robot and commit the arm toward it. Real
      throws' p90 was 27deg; 100deg keeps every plausibly-at-the-robot throw.
    """
    n = min(len(flight_buffer), 10)
    if n < 3:
        return None  # not enough to judge - the commit gate needs 40+ samples anyway
    p0 = mocap_point_to_base(np.array([flight_buffer[0].x, flight_buffer[0].y, flight_buffer[0].z]), R, t_vec)
    p1 = mocap_point_to_base(np.array([flight_buffer[n - 1].x, flight_buffer[n - 1].y, flight_buffer[n - 1].z]), R, t_vec)
    dist0 = float(np.linalg.norm(p0[:2]))
    if dist0 < min_release_dist:
        return (f"release only {dist0:.2f}m (horizontal) from the robot base "
                f"(< {min_release_dist:.2f}m) - a hand/handled ball, not a throw")
    dt = flight_buffer[n - 1].t - flight_buffer[0].t
    if dt <= 0:
        return None
    vh = (p1[:2] - p0[:2]) / dt
    vh_n = float(np.linalg.norm(vh))
    if vh_n >= min_horiz_speed:
        to_base = -p0[:2] / (dist0 + 1e-9)
        cos = float(np.dot(vh, to_base)) / (vh_n + 1e-9)
        ang = math.degrees(math.acos(max(-1.0, min(1.0, cos))))
        if ang > max_away_deg:
            return (f"ball moving {ang:.0f}deg away from the robot at {vh_n:.1f}m/s "
                    f"(> {max_away_deg:.0f}deg) - not a throw at the arm")
    return None


# Max azimuth deviation of a catch target from the wait pose's azimuth. The
# envelope's reach/z bands are rotationally symmetric, so without this a target
# BEHIND the robot passes them - a real 2026-07-16 incident (session 151342 throw 9,
# a hand grabbing the ball out of the box being detected as a "throw") committed the
# arm to azimuth ~158deg away from the wait pose and it self-collided at the elbow.
# Real committed throws all fell within 44deg of the wait azimuth. The release guard
# (check_release_guard) should catch the false-release upstream; this is the
# belt-and-suspenders backstop at the single reviewed motion gate.
#
# 2026-08-24 UR10 migration: reused unchanged, per user directive. Same caveat as
# CATCH_Z_MIN above - this number isn't a joint/geometry constant, it's "how far
# around the OLD stand could the arm swing before hitting something" for that specific
# mount. Carrying it over is only actually safe once someone's eyeballed there's
# nothing equivalently close within 75deg of the new wait pose's azimuth.
CATCH_MAX_AZIMUTH_DEG = 75.0


def check_catch_envelope(target_xyz: np.ndarray, wait_xyz: Optional[np.ndarray] = None,
                         max_azimuth_deg: float = CATCH_MAX_AZIMUTH_DEG) -> Optional[str]:
    """Return None if `target_xyz` (base frame) is a safe catch target, else a reason string.

    This is the single reviewed 'clamp-off' path CLAUDE.md requires for catch moves -
    every commanded target passes through here immediately before any motion is sent,
    regardless of what produced it, so a bad fit or a code bug can't fling the arm.
    Pass `wait_xyz` to also enforce the azimuth band (see CATCH_MAX_AZIMUTH_DEG).
    """
    reach = float(np.linalg.norm(target_xyz))
    if not (CATCH_MIN_REACH <= reach <= CATCH_MAX_REACH):
        return f"reach {reach:.2f}m outside catch band [{CATCH_MIN_REACH:.2f},{CATCH_MAX_REACH:.2f}]m"
    z = float(target_xyz[2])
    if not (CATCH_Z_MIN <= z <= CATCH_Z_MAX):
        return f"height z={z:+.2f}m outside catch band [{CATCH_Z_MIN:+.2f},{CATCH_Z_MAX:+.2f}]m (deck/singularity guard)"
    if wait_xyz is not None:
        az_wait = math.atan2(float(wait_xyz[1]), float(wait_xyz[0]))
        az_tgt = math.atan2(float(target_xyz[1]), float(target_xyz[0]))
        d_az = math.degrees(abs(math.atan2(math.sin(az_tgt - az_wait), math.cos(az_tgt - az_wait))))
        if d_az > max_azimuth_deg:
            return (f"target azimuth {d_az:.0f}deg from the wait pose (> {max_azimuth_deg:.0f}deg) - "
                    f"behind/beside the demo corridor, refusing")
    return None


def clamp_to_envelope(p: np.ndarray, margin: float = 0.0) -> np.ndarray:
    """Project a point into the catch envelope's reach and z bands.

    Exists because the envelope is NOT CONVEX: it is an annulus (reach 0.55-1.20m)
    intersected with a z band and an azimuth wedge, so a straight line between two
    perfectly valid points can leave it. Concretely, sweeping the setpoint from the
    wait pose (reach 0.73m) to a low-reach side target cuts the corner and dips
    under the 0.55m inner bound partway across.

    That matters only for streaming. A discrete movel/movej is checked once, at its
    endpoint, and the controller owns the path in between; a servo setpoint IS the
    path, so every intermediate point gets checked too - and refusing them would
    freeze the arm mid-catch, which is both useless and worse than the transient
    dip it is avoiding. So intermediate points are projected back onto the band
    rather than rejected: motion stays continuous and always inside the envelope.

    Only reach and z are projected. Azimuth is not, deliberately: it is bounded by
    the endpoints (both already checked upstream), and "fixing" an azimuth
    violation by rotating a point would move the tool somewhere nobody asked for.
    check_catch_envelope() still runs on the result as the real gate - this is a
    projection, not a substitute for the check.

    Order matters: z is clamped first and then held FIXED while the horizontal
    component alone is scaled to satisfy the reach band. Scaling the whole vector
    for reach (the obvious one-liner) would drag z back out of the band it was
    just clamped into, so the two constraints would fight.
    """
    # Land a micron INSIDE each bound, never exactly on it. Clamping to a bound and
    # then testing against that same bound with <= is fragile: measured, the exact
    # projection came out 5.6e-17 m under CATCH_MIN_REACH and was refused by the
    # gate it had just been projected to satisfy. A micron is ~11 orders of
    # magnitude above that error and physically meaningless next to a 4.25mm
    # calibration.
    eps = 1e-6
    p = np.asarray(p, dtype=float).copy()
    p[2] = float(np.clip(p[2], CATCH_Z_MIN + eps, CATCH_Z_MAX - eps))
    z = float(p[2])
    h = float(np.hypot(p[0], p[1]))
    # With z fixed, reach^2 = h^2 + z^2, so the reach band becomes a band on h.
    h_min, h_max = reach_band_at_z(z)
    if h < 1e-9:
        return p  # on the base axis; no horizontal direction to scale (degenerate,
                  # unreachable by interpolating two in-envelope points - the gate
                  # below still refuses it)
    h_c = min(max(h, h_min), h_max)
    if h_c != h:
        p[0] *= h_c / h
        p[1] *= h_c / h
    return p


def reach_band_at_z(z: float) -> tuple:
    """Horizontal-radius band [h_min, h_max] at base-frame height `z` that keeps
    3D reach inside [CATCH_MIN_REACH, CATCH_MAX_REACH].

    Shared by clamp_to_envelope (projects a single point) and the servo
    RateLimiter's `reach_bounds` hook (brakes toward this band every tick, see
    ur_servo.RateLimiter's class docstring) - both need exactly this per-z
    projection of the spherical reach shell onto a horizontal-radius interval.
    """
    eps = 1e-6
    h_min = math.sqrt(max((CATCH_MIN_REACH + eps) ** 2 - z * z, 0.0))
    h_max = math.sqrt(max((CATCH_MAX_REACH - eps) ** 2 - z * z, 0.0))
    return h_min, h_max



def idle_wobble_offset(now_t: float, diameter: float = IDLE_WOBBLE_DIAMETER_M,
                       period: float = IDLE_WOBBLE_PERIOD_S) -> np.ndarray:
    """Base-frame (dx, dy, 0) offset for the idle 'catcher waiting' animation - a
    point on a `diameter`-wide circle, phase driven directly by wall-clock
    monotonic time rather than a tracked start-time/phase variable, so pausing
    the wobble (e.g. during pending_dump) and resuming it later just picks the
    circle back up wherever it currently sits instead of jumping to a fixed
    start angle.

    Horizontal only (z always 0) and deliberate: derive_catch_plane() fixes the
    catch plane's height from wait_xyz's height ONCE at startup, so any z
    component here would need to feed back into that plane - staying
    horizontal keeps the wobble fully decoupled from the catch geometry.
    Added onto wait_xyz by the caller; never used as an absolute point.
    """
    omega = 2.0 * math.pi / period
    ang = omega * now_t
    r = diameter / 2.0
    return np.array([r * math.cos(ang), r * math.sin(ang), 0.0])


def derive_catch_plane(wait_xyz: np.ndarray, R: np.ndarray, t_vec: np.ndarray) -> float:
    """Mocap up-axis (Y) value of the horizontal plane through the wait height.

    The trajectory fit and its plane-crossing solver live in the mocap frame, so we
    inverse-transform the base-frame wait position back to mocap and read its up-axis
    (Y) component. Base Z ~= mocap Y in this rig (calibration R), so a constant-height
    base plane is a constant-Y mocap plane to within the calibration's ~1 deg tilt.
    """
    p_mocap = R.T @ (wait_xyz - t_vec)
    return float(p_mocap[1])  # mocap Y = up


def resolve_live_base_rb_transform(server_ip, local_ip, unicast, base_rb_id, R_base_rb, t_base_rb,
                                    timeout=10.0, settle_duration=1.0, std_limit=0.003):
    """Connect to Motive just long enough to get a SETTLED reading of the base rigid
    body, and derive base<-mocap from it, BEFORE the main NatNet client/loop exist.
    Without this, derive_catch_plane() below would use whatever R,t got loaded from
    the static --transform-file - fine normally, but wrong after the rig has been
    physically moved (the whole point of --base-rb-transform), since that static file
    reflects wherever the rig was when it was captured. Real incident 2026-07-24: rig
    moved, catch.py run without --base-rb-transform at all (separate bug, just a
    missing flag) - this fixes the deeper latent one that would have hit on the next
    run even with the flag correctly passed.

    Averages `settle_duration` seconds of samples once the RB is first seen valid,
    via calibrate_base_rb.py's own average_pose() - NOT just the first valid frame.
    Real incident 2026-07-27: after physically rotating the rig, a one-shot first
    frame here was caught still-settling and biased catch_value by ~0.2m for an
    entire session, even though the main loop's per-tick R,t (line ~1695, sampled
    continuously) had by then converged to the true pose - the frozen catch_value
    and the live per-tick transform silently disagreed, and every catch target
    landed ~0.2m below the wait pose. See docs/debug_log.md 2026-07-27.

    A short-lived second NatNet client is safe here: multicast supports multiple
    simultaneous local subscribers (confirmed live 2026-07-24 while debugging a
    stray-marker issue - two independent clients on this same machine each got a
    full, undisturbed copy of the stream)."""
    state = CaptureState(base_rb_id)
    client = NatNetClient(server_ip_address=server_ip, local_ip_address=local_ip, use_multicast=not unicast)
    client.on_data_frame_received_event.handlers.append(make_base_rb_capture_handler(state))
    with client:
        client.run_async()
        state.capturing = True
        start = time.monotonic()
        first_valid_at = None
        while time.monotonic() - start < timeout:
            if any(v for _, _, v in state.samples) and first_valid_at is None:
                first_valid_at = time.monotonic()
            if first_valid_at is not None and time.monotonic() - first_valid_at >= settle_duration:
                break
            time.sleep(0.02)
        state.capturing = False
    if first_valid_at is None:
        raise SystemExit(f"Never saw a valid reading of base rigid body id={base_rb_id} within {timeout}s - "
                          "check Motive streaming and that the rigid body is visible.")
    pos_avg, quat_avg, pos_std, valid_ratio = average_pose(state.samples)
    if pos_avg is None:
        raise SystemExit(f"base rigid body id={base_rb_id} had no valid samples in the {settle_duration}s settle window.")
    if np.any(pos_std > std_limit):
        raise SystemExit(f"base rigid body id={base_rb_id} pose unstable over the {settle_duration}s settle "
                          f"window (pos_std={pos_std}, limit={std_limit}) - the rig is still moving/settling "
                          "(e.g. right after being bumped or repositioned). Wait for it to stop before "
                          "starting catch.py.")
    R_mocap_rb = quat_to_matrix(quat_avg)
    return base_from_mocap_via_rigid_body(R_base_rb, t_base_rb, R_mocap_rb, pos_avg)


class LastNormal:
    """Mutable box holding the wall-clock time check_safety_mode() last observed
    the robot in a NORMAL safety mode. Threaded through every check_safety_mode()
    call site (move_to(), wait_for_fault_clear(), the main loop) so a detected
    fault can be logged with an honest bound on when it actually started, instead
    of just whichever `throw` counter happens to be current at DETECTION time.

    Added 2026-07-16: catch_log analysis of a real session found `check_safety_mode`
    detection lagging the robot's true fault trigger (per its own flight-report
    telemetry) by several seconds on every incident - long enough to land on the
    NEXT throw's tick, silently mislabeling which throw actually caused the fault.
    The dashboard-based check above should make that lag much smaller, but this is
    the honest fix: record the uncertainty window explicitly rather than trust a
    single `throw` number. See docs/debug_log.md 2026-07-16.
    """

    def __init__(self):
        self.wall = time.time()


def check_safety_mode(rtde_r, dash, last_normal: "LastNormal") -> Optional[str]:
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

    Checks the Dashboard Server (port 29999, a stateless per-request query - see
    ur_status.py) as the primary source of truth, not rtde_r.getSafetyMode() alone.
    2026-07-16 real incident: on all 4 protective stops in one session, rtde_r's
    cached safety-mode register kept reading NORMAL for ~9s after the robot's own
    flight-report telemetry proved it was already frozen in PROTECTIVE_STOP - so
    move_to() silently reported settled=True/fault=None for a move the robot never
    actually made. rtde_r.getSafetyMode() is still read and returned for logging/
    comparison (a divergence between the two is itself worth knowing about), but the
    dashboard reading is what decides the return value.

    Updates `last_normal.wall` to now whenever NORMAL is observed - see LastNormal.
    """
    dash_mode = dash.safetymode()  # e.g. "Safetymode: NORMAL"
    rtde_mode = rtde_r.getSafetyMode()
    dash_normal = dash_mode.strip().rsplit(":", 1)[-1].strip().upper() == "NORMAL"
    if not dash_normal:
        return (f"safety_mode={dash_mode.strip()} (dashboard) / rtde={rtde_mode} - "
                f"robot is stopped/faulted, not actually moving")
    last_normal.wall = time.time()
    return None


def move_to(pose: List[float], speed: float, accel: float, rtde_r, dash, last_normal: "LastNormal",
            settle_timeout: float = 8.0):
    """Blocking movej to an absolute base-frame pose. speed/accel are rad/s, rad/s^2
    (joint-space) - NOT the m/s, m/s^2 of a movel. Resolved to joints robot-side via
    get_inverse_kin(pose, qnear=<arm's actual current joints>) - see
    movej_to_pose_script's docstring for why this replaced a straight movel: this is
    the wait-pose approach/return path (large, arbitrary-starting-configuration moves),
    not the short fire-and-forget catch movel, and a movel's straight-line Cartesian
    interpolation can force an unpredictable, large/fast joint sweep (e.g. the base
    joint) whenever the arm's real starting configuration is far from what the
    straight line assumes - a real 2026-07-16 protective stop traced to exactly that.
    Returns (settled, fault): fault is a description string (and settled forced False)
    if a non-NORMAL safety mode is observed at any point in the wait - see
    check_safety_mode().

    CB3 UR10 bring-up 2026-08-24: this settle check must not start counting
    "stopped" ticks until real motion (TCP speed > MOVE_START_SPEED) has been
    observed - on a controller with a slower script-parse/motion-start delay
    than the e-Series box this was validated against (~0.6-0.8s measured on
    CB3), the old unguarded version could read near-zero speed for the first
    few ticks *before the move had even begun* and declare settled instantly,
    reporting arrival at a pose the arm hadn't moved toward yet. See
    ur_goto_raw.wait_for_stop's docstring for the full story - same bug, same
    fix, independently duplicated here because move_to() has its own
    check_safety_mode() polling woven into the loop."""
    qnear = list(rtde_r.getActualQ())
    current_pose = rtde_r.getActualTCPPose()
    if max(abs(t - c) for t, c in zip(pose[:3], current_pose[:3])) < 0.001:
        return True, None
    send_script(movej_to_pose_script(pose, qnear, speed, accel))
    slow_streak = 0
    started = False
    start = time.time()
    while time.time() - start < settle_timeout:
        fault = check_safety_mode(rtde_r, dash, last_normal)
        if fault is not None:
            return False, fault
        peak = max(abs(v) for v in rtde_r.getActualTCPSpeed())
        if not started:
            if peak > 0.005:
                started = True
        elif peak < 0.002:
            slow_streak += 1
            if slow_streak >= 5:
                return True, None
        else:
            slow_streak = 0
        time.sleep(0.05)
    return False, None


# demo.py-only (2026-08-25): post-catch "dump the ball out" trick. Base joint
# rotates DEMO_DUMP_BASE_DEG ("right"); sign is a guess (negative = clockwise
# viewed from above, base-frame Z-up right-hand rule) - not yet verified on the
# real arm. If the first live run swings the wrong way, just flip this sign.
DEMO_DUMP_BASE_DEG = -90.0
DEMO_DUMP_WRIST_DEG = 180.0   # wrist3 (last joint) - spins the tool to tip the box out
DEMO_DUMP_SPEED = 1.5         # rad/s, joint-space
DEMO_DUMP_ACCEL = 1.0         # rad/s^2
DEMO_DUMP_ARRIVAL_TOL_M = 0.03  # how close to the wait pose (xyz only) before the dump fires

# 2026-08-26 real incident: TARGET (TARGET_RB_ID) was never actually tracked all
# session (Motive's "never sighted" default pose - see CLAUDE.md - reads as a
# real but wrong, permanently-out-of-band position, not `find_target_pose()`
# returning None), so pending_dump never cleared after any of the session's 7
# catches - and since idle wobble is gated on `not pending_dump` (see its call
# site), the wobble never ran once, all session, despite being enabled. The
# TARGET search itself is deliberately unbounded (see the 2026-08-25 fix
# comment at pending_dump's arrival check) - a real prop can take a few
# seconds to nudge into view - so this does NOT shorten that search or clear
# pending_dump early. It only lets the cosmetic wobble resume after this many
# seconds of not finding TARGET, sharing the wait pose with an occasional (not
# guaranteed) search attempt whenever a wobble lap happens to swing back near
# center - an acceptable trade for a feature that has no reliability
# requirement, versus leaving it silently dead for an entire session.
# Shortened 4.0->1.0 2026-08-26 (real-session feedback: even 4s read as a
# noticeable dead pause before the "catcher waiting" tell kicked in) - purely a
# tunable tradeoff, not a technical floor. Lower still (even 0) is safe; it
# only trades away how long a real TARGET prop gets sat still for before a
# wobble lap starts interrupting the arrival-window search.
DUMP_SEARCH_WOBBLE_GRACE_S = 1.0


def dump_ball_sequence(rtde_r, dash, last_normal: "LastNormal") -> bool:
    """Blocking joint-space trick run once the arm is back at the wait pose after
    a guessed catch: rotate the base DEMO_DUMP_BASE_DEG, then wrist3
    DEMO_DUMP_WRIST_DEG to tip the ball out of the box, then movej back to the
    joints it started from (== the wait pose, since the caller only triggers this
    once the arm has arrived there). Aborts and returns False the instant a fault
    is detected, same convention as move_to()'s settled flag - the caller is
    expected to fall into wait_for_fault_clear() same as any other move here."""
    start_q = list(rtde_r.getActualQ())

    q1 = list(start_q)
    q1[0] += math.radians(DEMO_DUMP_BASE_DEG)
    send_script(movej_script(q1, DEMO_DUMP_SPEED, DEMO_DUMP_ACCEL))
    wait_for_stop(rtde_r, is_joint=True, current=start_q, target=q1)
    if check_safety_mode(rtde_r, dash, last_normal) is not None:
        return False

    q2 = list(q1)
    q2[5] += math.radians(DEMO_DUMP_WRIST_DEG)
    send_script(movej_script(q2, DEMO_DUMP_SPEED, DEMO_DUMP_ACCEL))
    wait_for_stop(rtde_r, is_joint=True, current=q1, target=q2)
    if check_safety_mode(rtde_r, dash, last_normal) is not None:
        return False

    send_script(movej_script(start_q, DEMO_DUMP_SPEED, DEMO_DUMP_ACCEL))
    wait_for_stop(rtde_r, is_joint=True, current=q2, target=start_q)
    return check_safety_mode(rtde_r, dash, last_normal) is None


# demo.py-only (2026-08-25): dump onto a second tracked rigid body ("TARGET",
# Motive asset id TARGET_RB_ID). If TARGET isn't currently tracked, or fails
# the same envelope check every other catch target goes through, nothing is
# dumped at all - the arm just stays at the wait pose it already returned to
# (user directive: no more "dump in place" fallback - dump_ball_sequence()
# above is now unused dead code, kept in case that trick is wanted again).
# check_catch_envelope() is the one reviewed clamp-off path for arbitrary
# targets (CLAUDE.md), so this reuses it rather than inventing a second,
# unreviewed reach check - its reach band already tops out at CATCH_MAX_REACH
# (1.20m), which is the "within 120cm of base joint" limit asked for.
TARGET_RB_ID = 8


def find_target_pose(target_rb_state: "BaseRBState", R: np.ndarray, t_vec: np.ndarray) -> Optional[np.ndarray]:
    """Base-frame xyz of the TARGET rigid body, or None if it isn't currently
    tracked (never seen this session, or occluded right now)."""
    with BASE_RB_LOCK:
        pos, valid = target_rb_state.latest_pos, target_rb_state.latest_valid
    if not valid or pos is None:
        return None
    return mocap_point_to_base(np.array(pos), R, t_vec)


def target_hover_pose(target_xyz: np.ndarray, wait_pose: List[float]) -> List[float]:
    """The actual move target for dump_ball_on_target(): TARGET's x/y, z held
    exactly at the wait pose's own z - never TARGET's z, and never higher/lower
    than the wait pose (2026-08-25 user directive: stay in the wait pose's
    plane the whole time, only x/y moves). Split out so the caller can run
    this exact pose (not TARGET's raw position) through check_catch_envelope()
    before committing to the move."""
    return [float(target_xyz[0]), float(target_xyz[1]), float(wait_pose[2]),
            wait_pose[3], wait_pose[4], wait_pose[5]]


def target_aim_pose(target_xyz: np.ndarray, wait_pose: List[float], wait_xyz: np.ndarray) -> List[float]:
    """Intermediate pose for dump_ball_on_target(): same horizontal reach and z
    as the wait pose itself - only the azimuth changes, rotated to face
    TARGET - with orientation carried along via yaw_follow_orientation (keeps
    the wrist configuration constant relative to the arm's own plane, so IK
    resolves to "rotate the base, nothing else" instead of picking a different
    elbow/wrist branch).

    2026-08-25 user directive: TARGET can be anywhere around the robot, unlike
    a live catch target where CATCH_MAX_AZIMUTH_DEG's +-75deg band is a real,
    unrelaxed safety limit (see check_catch_envelope). The dump move is instead
    made safe by splitting it into two legs - rotate to this pose first (same
    reach as the wait pose, so the arm sweeps no wider than it already sits at
    rest), THEN extend outward from here to the real hover_pose (reach only
    changes, azimuth is already correct) - rather than by bounding azimuth at
    all. Same mirrored pair on the way back (retract to this pose, then rotate
    back to wait_pose) - see dump_ball_on_target().
    """
    wait_h = float(np.hypot(wait_xyz[0], wait_xyz[1]))
    az_tgt = math.atan2(float(target_xyz[1]), float(target_xyz[0]))
    orient = yaw_follow_orientation(wait_pose, wait_xyz, target_xyz)
    return [wait_h * math.cos(az_tgt), wait_h * math.sin(az_tgt), float(wait_pose[2])] + orient


def wait_until_actually_stopped(rtde_r, timeout: float = 2.0,
                                 settle_speed: float = 0.005, settle_ticks: int = 5) -> bool:
    """Block until the arm's real TCP speed reads near-zero for `settle_ticks`
    consecutive polls, or `timeout` elapses. Returns whether it actually settled.

    Unlike move_to()/wait_for_stop(), this does NOT wait for motion to start
    first - it's for confirming the arm is ALREADY at rest before sending the
    NEXT command, not for judging whether a just-sent move arrived.

    2026-08-25 real incident this exists for: dump_ball_on_target()'s first
    move_to() call used to fire immediately after stop_servo_stream() cut the
    live servo stream - but the arm was still mid-return-to-wait-pose (still
    ~7cm out, servo_cmd showed real velocity right up to the same tick) when
    the stream was torn down. move_to()'s "has real motion started" check saw
    that leftover/decelerating velocity, and as it decayed within the
    required settle window, move_to() declared the hover move "arrived" after
    only 0.4s - every other identical leg that session took 2.2-3.1s - while
    the arm was still ~0.66m short of hover_pose. The dump trick then tilted
    and poured right there (next to the wait pose) instead of over TARGET.
    Call this after tearing down the stream and before the first move_to()
    that follows it, so leftover motion can never again masquerade as the
    next move's own.
    """
    slow_streak = 0
    start = time.time()
    while time.time() - start < timeout:
        peak = max(abs(v) for v in rtde_r.getActualTCPSpeed())
        if peak < settle_speed:
            slow_streak += 1
            if slow_streak >= settle_ticks:
                return True
        else:
            slow_streak = 0
        time.sleep(0.05)
    return False


def dump_ball_on_target(rec: "Recorder", rtde_r, dash, last_normal: "LastNormal", aim_pose: List[float],
                         hover_pose: List[float], wait_pose: List[float], args) -> bool:
    """Blocking: move the box centroid to `hover_pose` (base frame, x/y over
    TARGET, z held exactly at the wait pose's own z - see target_hover_pose(),
    already envelope-checked by the caller), tilt wrist3 DEMO_DUMP_WRIST_DEG to
    pour the ball onto it, movej literally back to the pre-tilt joints, then
    movej back to `wait_pose`. Same fault-abort convention as
    dump_ball_sequence(). Every leg is a plain movej (either a literal joint
    target, or move_to()'s pose-resolved-to-joints-via-IK) - no movel anywhere
    in this trick, on purpose (2026-08-25 user directive after the incident
    below: keep it plain).

    Rotate-then-extend (2026-08-25 user directive): TARGET can be at ANY
    azimuth around the robot, including well outside CATCH_MAX_AZIMUTH_DEG's
    +-75deg live-catch band - the caller deliberately skips that check for the
    dump target (see target_hover_pose()'s call site) and passes `aim_pose`
    (target_aim_pose(): same reach/z as the wait pose, rotated to TARGET's
    azimuth) instead. Both the outbound and return trips detour through
    aim_pose, splitting what would otherwise be one movej that changes reach
    AND azimuth at once into two: rotate while still retracted at the wait
    pose's own reach (can't sweep any wider than the arm already sits at
    rest), then extend outward once already pointed the right way (reach only
    changes, azimuth doesn't) - and the mirror image coming back. This is what
    replaces the azimuth band for this path, rather than the move just being
    unbounded.

    Logs actual TCP pose after every leg individually (`post_catch_dump_hover`/
    `_tilt`/`_untilt`/`_return`) - 2026-08-25: the old version logged only one
    aggregate pass/fail for the whole function, so a real incident (the return
    leg swept the arm "somewhere crazy") left no trace of which leg failed or
    where the arm actually went. This is that fix.

    Return leg: also 2026-08-25, same real incident. The previous version went
    straight from the tilted pose (q_tilt, wrist3 +DEMO_DUMP_WRIST_DEG off
    nominal) into move_to(wait_pose, ...), which resolves via
    get_inverse_kin(wait_pose, qnear=q_tilt) - a starting joint configuration
    nothing had ever validated a return move from, and on the real run it did
    not cleanly unwind. Fix: movej literally back to q_hover first (the exact
    joints recorded right after the hover move arrived, before the tilt) - a
    guaranteed retrace of the tilt, same trick dump_ball_sequence() (dead
    code, above) always used - THEN do the normal move_to(wait_pose, ...) from
    q_hover, a normal, already-proven-safe starting configuration for that
    call. Never let move_to() resolve fresh IK from the twisted q_tilt again.

    Hover leg: 2026-08-25, a third real incident, same session type as the two
    above but a different mechanism - see wait_until_actually_stopped()'s
    docstring. This function's first move (to hover_pose) is always called
    immediately after the live servo stream was just torn down, so it opens
    with a blocking wait for that leftover motion to actually die down before
    ever calling move_to() - otherwise move_to()'s settle-detection can mistake
    the stream's dying velocity for the new move's own and report "arrived"
    almost instantly, while the arm is still wherever the stream left it.
    """
    if not wait_until_actually_stopped(rtde_r):
        rec.log("move", purpose="post_catch_dump_prestart_timeout",
                note="arm still moving 2s after the servo stream was torn down - proceeding anyway")

    def log_leg(purpose: str, settled: bool, fault: Optional[str]) -> None:
        try:
            actual_tcp = list(rtde_r.getActualTCPPose())
        except Exception:
            actual_tcp = None
        rec.log("move", purpose=purpose, settled=settled, fault=fault, actual_tcp=actual_tcp)

    settled, fault = move_to(aim_pose, args.approach_speed, args.approach_accel, rtde_r, dash, last_normal)
    log_leg("post_catch_dump_aim", settled, fault)
    if fault is not None or not settled:
        return False

    settled, fault = move_to(hover_pose, args.approach_speed, args.approach_accel, rtde_r, dash, last_normal)
    log_leg("post_catch_dump_hover", settled, fault)
    if fault is not None or not settled:
        return False

    q_hover = list(rtde_r.getActualQ())
    q_tilt = list(q_hover)
    q_tilt[5] += math.radians(DEMO_DUMP_WRIST_DEG)
    send_script(movej_script(q_tilt, DEMO_DUMP_SPEED, DEMO_DUMP_ACCEL))
    wait_for_stop(rtde_r, is_joint=True, current=q_hover, target=q_tilt)
    fault = check_safety_mode(rtde_r, dash, last_normal)
    log_leg("post_catch_dump_tilt", fault is None, fault)
    if fault is not None:
        return False

    # Literal unwind - NOT fresh IK - see docstring. Guaranteed retrace of the
    # tilt leg above, so it can't pick a different IK branch or sweep
    # somewhere unplanned the way move_to(wait_pose, ...) did from q_tilt in
    # the real incident this replaced.
    send_script(movej_script(q_hover, DEMO_DUMP_SPEED, DEMO_DUMP_ACCEL))
    wait_for_stop(rtde_r, is_joint=True, current=q_tilt, target=q_hover)
    fault = check_safety_mode(rtde_r, dash, last_normal)
    log_leg("post_catch_dump_untilt", fault is None, fault)
    if fault is not None:
        return False

    settled, fault = move_to(aim_pose, args.approach_speed, args.approach_accel, rtde_r, dash, last_normal)
    log_leg("post_catch_dump_retract", settled, fault)
    if fault is not None or not settled:
        return False

    settled, fault = move_to(wait_pose, args.approach_speed, args.approach_accel, rtde_r, dash, last_normal)
    log_leg("post_catch_dump_return", settled, fault)
    return fault is None and settled


def wait_for_fault_clear(rec: "Recorder", fault: str, fault_count: int, rtde_r, dash, last_normal: "LastNormal",
                          wait_pose: List[float], args) -> int:
    """Block until a detected robot fault (protective stop etc.) is cleared, then
    drive back to the wait pose and resume - instead of the old halt_on_fault()
    hard-kill. Deliberately does NOT auto-clear the fault itself (no dashboard-server
    unlockProtectiveStop() etc.) - that's cleared by hand, on the pendant or via
    ur_status.py --clear, by whoever is standing next to the arm. This just polls
    check_safety_mode() and waits, so the session survives a fault instead of dying.

    Ctrl-C works throughout (it's just time.sleep() in a loop, no exception
    swallowed here) if you'd rather abort than wait. Full forensic detail of the
    fault lives robot-side regardless (log_history.txt, polyscope.log, an
    auto-generated flight report zip - see docs/debug_log.md) and gets pulled by
    wrap_up_session() at the end of the run either way.
    """
    while True:
        fault_count += 1
        now = time.time()
        rec.log("fault", reason=fault, fault_count=fault_count, detected_wall=now,
                last_known_normal_wall=last_normal.wall, max_undetected_s=now - last_normal.wall,
                note="fault actually started sometime in (last_known_normal_wall, detected_wall] - "
                     "'throw' above is whichever throw was current at DETECTION time, not necessarily "
                     "the one that caused it")
        print(f"\n!!! ROBOT FAULT: {fault}")
        if now - last_normal.wall > 1.0:
            print(f"    (undetected for up to {now - last_normal.wall:.1f}s - last confirmed NORMAL "
                  f"at {time.strftime('%H:%M:%S', time.localtime(last_normal.wall))})")
        print("    clear it on the pendant (or ur_status.py --clear) - waiting...")
        waited = 0.0
        last_reminder = 0.0
        while check_safety_mode(rtde_r, dash, last_normal) is not None:
            time.sleep(FAULT_WAIT_POLL_S)
            waited += FAULT_WAIT_POLL_S
            if waited - last_reminder >= FAULT_WAIT_REMINDER_S:
                print(f"    still waiting for the fault to be cleared ({waited:.0f}s)...")
                last_reminder = waited
        rec.log("fault_cleared", fault_count=fault_count, waited_s=waited)
        print(f"    cleared after {waited:.0f}s.")
        if args.dry_run:
            break
        print("    returning to wait pose...")
        settled, fault2 = move_to(wait_pose, args.approach_speed, args.approach_accel, rtde_r, dash, last_normal)
        rec.log("move", purpose="post_fault_recovery", target=wait_pose,
                speed=args.approach_speed, accel=args.approach_accel, settled=settled, fault=fault2)
        if fault2 is not None:
            fault = fault2  # faulted again immediately (e.g. still on an obstruction) - wait again
            continue
        if not settled:
            print("    WARNING: did not settle at wait pose within timeout after recovery.")
        break
    print("    resumed. ready for next throw.\n")
    return fault_count


def enter_pressed() -> bool:
    """Non-blocking: True if the user hit Enter on stdin since the last call.

    Used as the normal/clean way to stop a session (in addition to Ctrl-C, which
    still works as an emergency abort - both paths converge on the same
    stopl+teardown+wrap-up code). select() on stdin rather than a background thread
    blocked on input() specifically so nothing is left reading stdin afterward -
    wrap_up_session() below also calls input(), and two readers racing on the same
    stdin would be a real bug, not just untidy.
    """
    if not sys.stdin.isatty():
        return False
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        sys.stdin.readline()
        return True
    return False


def pull_robot_session_logs(start_wall: float, end_wall: float, out_dir: str) -> List[str]:
    """Pull the slice of the robot's own log_history.txt / polyscope.log covering
    this session's wall-clock window (both files carry real timestamps - see
    CLAUDE.md 'Run recording'), plus any flight-report zip auto-generated during it
    (created on a fault - see docs/debug_log.md; only the last 5 are kept, oldest
    evicted on the next trigger, so pulling promptly matters). One SSH round-trip,
    called only from wrap_up_session() after the session's NatNet/RTDE connections
    are already closed - see the module docstring's threading rule for why nothing
    like this may run near the hot loop.

    Best-effort: a slow/unreachable robot SSH link just skips this with a warning,
    it never blocks or discards the name/description the user already typed.

    Returns the local basenames of any flight-report zips actually pulled (possibly
    empty) - used by build_flight_report_manifest() to link each one back to the
    catch_log fault event that (most likely) triggered it.
    """
    start_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(start_wall))
    end_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(end_wall + 1))
    os.makedirs(out_dir, exist_ok=True)
    remote_cmd = (
        f"awk -F' :: ' -v s='{start_str}' -v e='{end_str}' "
        f"'$3>=s && $3<=e' /root/log_history.txt; "
        f"echo '===SPLIT==='; "
        f"awk -v s='{start_str}' -v e='{end_str}' "
        f"'{{ts=$1\" \"$2}} ts>=s && ts<=e' /root/polyscope.log; "
        f"echo '===SPLIT==='; "
        f"find /root/flightreports -name '*.zip' -newermt '{start_str}'"
    )
    try:
        result = subprocess.run(
            ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", ROBOT_SSH_HOST, remote_cmd],
            capture_output=True, text=True, timeout=15,
        )
        if result.returncode != 0:
            print(f"  (robot log pull failed: {result.stderr.strip()[:200]})")
            return []
        parts = result.stdout.split("===SPLIT===\n")
        log_history_slice = parts[0] if len(parts) > 0 else ""
        polyscope_slice = parts[1] if len(parts) > 1 else ""
        new_reports = [l.strip() for l in parts[2].splitlines()] if len(parts) > 2 else []
        with open(os.path.join(out_dir, "log_history_slice.txt"), "w") as f:
            f.write(log_history_slice)
        with open(os.path.join(out_dir, "polyscope_slice.txt"), "w") as f:
            f.write(polyscope_slice)
        pulled_basenames = []
        for remote_path in new_reports:
            if remote_path:
                r = subprocess.run(
                    ["scp", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes", f"{ROBOT_SSH_HOST}:{remote_path}", out_dir],
                    capture_output=True, timeout=60,
                )
                if r.returncode == 0:
                    pulled_basenames.append(os.path.basename(remote_path))
        print(f"  pulled robot logs -> {out_dir} "
              f"({log_history_slice.count(chr(10))} log_history lines, "
              f"{polyscope_slice.count(chr(10))} polyscope lines, "
              f"{len(pulled_basenames)} flight report(s))")
        return pulled_basenames
    except Exception as e:
        print(f"  (robot log pull skipped: {e})")
        return []


# UR flight-report zips are named recording<YYYYMMDD>_<HH>_<MM>_<SS>.zip, where the
# embedded timestamp is the incident trigger time (verified against polyscope.log's
# own "Flight reporter triggered" line - matches to the millisecond). See
# build_flight_report_manifest().
FLIGHT_REPORT_NAME_RE = re.compile(r"recording(\d{4})(\d{2})(\d{2})_(\d{2})_(\d{2})_(\d{2})\.zip")

# Widened past the ~9s worst-case detection lag seen in one real session (see
# LastNormal's docstring) - matching is "nearest fault event within this window",
# not "must be near-instant", precisely because that lag is real and can vary.
FLIGHT_REPORT_MATCH_TOLERANCE_S = 30.0


def build_flight_report_manifest(out_dir: str, pulled_basenames: List[str], record_path: Optional[str]) -> None:
    """Write robot_logs/sessions/<...>/flight_reports.json: for each flight-report
    zip pulled into this session, its parsed trigger time and (if within
    FLIGHT_REPORT_MATCH_TOLERANCE_S) the nearest catch_log "fault" event's
    fault_count/detected_wall/max_undetected_s - so "which fault does this zip's
    realtimedata.csv belong to" is answered by reading a few lines here, not by
    eyeballing filenames against JSONL timestamps by hand (see docs/debug_log.md
    2026-07-16, which had to do exactly that once).

    Best-effort and silent-if-nothing-to-do: no report zips or no --record means an
    empty/absent manifest, not an error.
    """
    if not pulled_basenames:
        return
    faults = []
    if record_path and os.path.exists(record_path):
        with open(record_path) as f:
            for line in f:
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if rec.get("ev") == "fault":
                    faults.append(rec)

    manifest = []
    for basename in pulled_basenames:
        m = FLIGHT_REPORT_NAME_RE.match(basename)
        if not m:
            manifest.append({"zip": basename, "trigger_wall": None, "matched_fault": None,
                              "note": "filename didn't match the expected recording_YYYYMMDD_HH_MM_SS.zip pattern"})
            continue
        year, month, day, hh, mm, ss = m.groups()
        trigger_struct = time.strptime(f"{year}-{month}-{day} {hh}:{mm}:{ss}", "%Y-%m-%d %H:%M:%S")
        trigger_wall = time.mktime(trigger_struct)

        best, best_gap = None, None
        for flt in faults:
            gap = abs(flt.get("detected_wall", flt["wall"]) - trigger_wall)
            if best_gap is None or gap < best_gap:
                best, best_gap = flt, gap
        matched = None
        if best is not None and best_gap <= FLIGHT_REPORT_MATCH_TOLERANCE_S:
            matched = {"fault_count": best.get("fault_count"), "throw_at_detection": best.get("throw"),
                       "detected_wall": best.get("detected_wall", best["wall"]),
                       "last_known_normal_wall": best.get("last_known_normal_wall"),
                       "gap_to_trigger_s": round(best_gap, 3)}
        manifest.append({
            "zip": basename,
            "trigger_wall": trigger_wall,
            "trigger_iso": time.strftime("%Y-%m-%d %H:%M:%S", trigger_struct),
            "matched_fault": matched,
        })

    with open(os.path.join(out_dir, "flight_reports.json"), "w") as f:
        json.dump(manifest, f, indent=2)
    print(f"  flight report manifest -> {out_dir}/flight_reports.json "
          f"({sum(1 for m in manifest if m['matched_fault'])}/{len(manifest)} matched to a logged fault)")


def wrap_up_session(session_start: float, session_end: float, throws: int,
                     record_path: Optional[str], args,
                     catches: int = 0, attempts_ended: int = 0) -> None:
    """End-of-session bookkeeping: prompt for an optional name + a free-text
    description of how the session went, then pull the robot's own logs for that
    time window. Called once, after the NatNet/RTDE connections are torn down (see
    main()) - deliberately outside the hot loop/try-finally so none of this (input()
    prompts, an SSH round-trip) can add latency to the live trajectory/feasibility
    calculation.
    """
    if args.no_wrapup:
        return
    print("\n" + "=" * 78)
    try:
        name = input("Session name (optional, Enter to skip): ").strip()
    except EOFError:
        name = ""
    try:
        description = input("Description - what happened, how did the session go? ").strip()
    except EOFError:
        description = ""
    ts = time.strftime("%Y%m%d_%H%M%S", time.localtime(session_start))
    out_dir = os.path.join(ROBOT_LOG_SESSIONS_DIR, f"{ts}_{name}" if name else ts)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "notes.txt"), "w") as f:
        f.write(f"name: {name or '(none)'}\n")
        f.write(f"start: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(session_start))}\n")
        f.write(f"end: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(session_end))}\n")
        f.write(f"duration_s: {session_end - session_start:.1f}\n")
        f.write(f"throws: {throws}\n")
        f.write(f"caught: {catches}/{attempts_ended} attempted (heuristic: ball last seen <30cm from tool)\n")
        f.write(f"dry_run: {args.dry_run}\n")
        if record_path:
            f.write(f"catch_log: {record_path}\n")
        f.write(f"\ndescription:\n{description or '(none)'}\n")
    print(f"session notes -> {out_dir}/notes.txt")
    if args.pull_robot_logs:
        print("pulling robot logs for this session's time window...")
        pulled_basenames = pull_robot_session_logs(session_start, session_end, out_dir)
        build_flight_report_manifest(out_dir, pulled_basenames, record_path)
    else:
        print("  (skipping robot log pull - pass --pull-robot-logs once SSH access to the "
              "controller works; see docs/ur10_migration_roadmap.md)")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--rigid-body-id", type=int, default=3,
                        help="NatNet rigid body id of the ball (default 3, the standard rig with base+tool "
                             "RBs also in the scene - auto-select only works with a single tracked body present)")

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
    parser.add_argument("--transform-file", default="UR10_T_base_from_mocap.json",
                        help="T_base<-mocap from calibrate_frames.py (default UR10_T_base_from_mocap.json, "
                             "the UR10 migration rig calibration since 2026-08-24 - the old UR12e rig's "
                             "T_base_from_mocap_v2.json is still there, pass it explicitly to go back)")
    parser.add_argument("--base-rb-transform", default="UR10_T_base_from_baseRB.json",
                        help="Path to calibrate_base_rb.py's output (default UR10_T_base_from_baseRB.json, "
                             "on since 2026-08-24 - pass an empty string to fall back to the static "
                             "--transform-file instead). When set, base<-mocap is recomputed every "
                             "tick from a second tracked rigid body mounted on the robot's fixed base "
                             "(frames.base_from_mocap_via_rigid_body) instead of loaded once from "
                             "--transform-file - lets the whole rig be physically repositioned between "
                             "sessions without rerunning calibrate_frames.py, as long as that rigid "
                             "body stays fixed to the base. --transform-file is still used for the "
                             "catch plane / TCP offset / startup banner either way.")
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
    parser.add_argument("--speed", type=float, default=1.2, help="m/s commanded for the CATCH movel (controller clamps; default 1.2, the 2026-07-17 operating point)")
    parser.add_argument("--accel", type=float, default=4.0, help="m/s^2 for the catch movel (default 4.0, the 2026-07-17 operating point)")
    parser.add_argument("--catch-move", choices=("movel", "movej", "servo"), default="servo",
                        help="Motion primitive for the catch move. servo (default since 2026-07-27, in daily "
                             "use since - continuous retargeting via servoj streaming, see the --servo-* "
                             "flags) commits at one tick's latency and keeps pulling the setpoint toward "
                             "the refined intercept, rather than firing a single discrete move. movej moves "
                             "in joint space (IK resolved robot-side via get_inverse_kin qnear=current "
                             "joints, same proven path as the wait-pose approach) - it CANNOT violate joint "
                             "limits by construction, fixing the side-throw protective stops movel was "
                             "causing. movel holds a straight Cartesian line - but to a side target that "
                             "forces base-joint speed = TCP speed / reach, which exceeds the 120deg/s base "
                             "limit whenever the azimuth swing is large (the 2026-07-16 fault data: 3%% "
                             "faults <10deg swing, 62%% at 20-35deg). movej/movel use --catch-joint-speed/"
                             "--catch-joint-accel or --speed/--accel respectively, not the --servo-* flags.")

    # Servo streaming (--catch-move servo) ----------------------------------------
    parser.add_argument("--servo-max-speed", type=float, default=1.2,
                        help="m/s cap for the servo setpoint stream (default 1.2 since 2026-09-11, user "
                             "request - back up to the movel/movej operating point, and still under the "
                             "~1.6 m/s ceiling measured on this UR10 with track_ball_servo.py on "
                             "2026-08-26; was 0.8 from 2026-07-27). This is enforced host-side by "
                             "ur_servo.RateLimiter, NOT by servoj, which has no speed limit of its own.")
    parser.add_argument("--servo-max-accel", type=float, default=6.0,
                        help="m/s^2 cap for the servo setpoint stream (default 6.0 since 2026-09-11, user "
                             "request; was 4.0, itself raised from 2.0 on 2026-07-21 to match --accel's "
                             "movel/movej operating point). Also sets the "
                             "deceleration-aware approach: commanded speed never exceeds "
                             "sqrt(2*a*distance_remaining), so the setpoint cannot overshoot the "
                             "intercept.")
    parser.add_argument("--servo-return-mult", type=float, default=0.7,
                        help="multiplier on --servo-max-speed/--servo-max-accel/--servo-base-rate-deg-s "
                             "applied ONLY while the stream is driving back to the wait pose (idle before "
                             "a throw, and after a throw ends) - not while tracking/chasing a live ball, "
                             "which stays at the normal capped speed. Default 0.7 (2026-08-26, down from "
                             "1.0: user asked for the return leg to run 30% slower than normal chase "
                             "speed - it has no accuracy requirement, so there's no reason for it to be "
                             "quick. History: was 2.0 [2x faster] until 2026-08-25, when it was halved "
                             "back down to 1.0 [normal speed].")
    parser.add_argument("--servo-base-rate-deg-s", type=float, default=SERVO_BASE_RATE_DEG_S,
                        help=f"deg/s cap on the base joint implied by lateral setpoint motion (default "
                             f"{SERVO_BASE_RATE_DEG_S}). This is the servo-mode replacement for movej's "
                             f"structural immunity to the side-throw C153A0 fault - servoj does NOT get "
                             f"that for free. Only the tangential component is capped; radial and "
                             f"vertical motion don't load the base joint.")
    parser.add_argument("--servo-orient-max-rate-deg-s", type=float, default=None,
                        help="deg/s cap on the streamed ORIENTATION (yaw-follow/tilt-follow), via "
                             "ur_servo.OrientationRateLimiter (default: same as --servo-base-rate-deg-s). "
                             "Added 2026-07-22: servo_orient used to be sent completely unrate-limited "
                             "alongside a rate-limited position - on commit it could snap the whole "
                             "wait-pose->target yaw delta (up to +/-75deg) in one tick. yaw_follow_orientation "
                             "rotates about base Z by design (keeps the wrist still), so that snap landed "
                             "almost entirely on the BASE joint - suspected root cause of both the "
                             "'oscillating to come to a stop' behavior and the base-joint accel-limit "
                             "protective stops seen in real 2026-07-21/22 sessions, neither fixed by tuning "
                             "--servo-max-speed/-accel since those never touched this channel. Defaults to "
                             "the same budget as --servo-base-rate-deg-s since yaw-follow's rotation and "
                             "position's azimuthal sweep are largely redundant descriptions of the same "
                             "base-joint motion, not independent - see OrientationRateLimiter for the math.")
    parser.add_argument("--servo-orient-max-accel-deg-s2", type=float, default=None,
                        help="deg/s^2 cap for the orientation stream (default: rate/0.15, same 'full rate "
                             "in 0.15s' convention as --servo-base-rate-deg-s's implicit accel)")
    parser.add_argument("--servo-rate", type=float, default=125.0,
                        help="Hz setpoint send rate in servo mode (default 125, the rate validated by "
                             "ur_servo.py --bench at 0 late ticks). Also becomes the default --poll-hz - "
                             "the two are one loop in servo mode, not independent (emit_setpoint() must "
                             "run every iteration). check_feasibility()'s fit/solve step is cached against "
                             "flight_buffer growth (Motive tops out at 120Hz) and the chase-abort timer is "
                             "wall-clock, not tick-counted, so raising this above 125 no longer wastes cycles "
                             "re-fitting stale data or silently shrinks the abort tolerance - untested above "
                             "125 on the real arm as of 2026-07-27, though; validate with --bench first.")
    parser.add_argument("--servo-gain", type=float, default=SERVO_DEFAULT_GAIN,
                        help=f"servoj gain, range [100,2000] (default {SERVO_DEFAULT_GAIN:.0f}). Higher "
                             f"tracks harder but risks vibration - lower this first if the arm buzzes.")
    parser.add_argument("--servo-lookahead", type=float, default=SERVO_DEFAULT_LOOKAHEAD,
                        help=f"servoj lookahead_time, range [0.03,0.2] (default {SERVO_DEFAULT_LOOKAHEAD})")
    parser.add_argument("--servo-host-ip", default=SERVO_HOST_IP,
                        help="This machine's IP on the ROBOT subnet - the robot dials it to open the "
                             "setpoint stream (not the mocap-subnet --local-ip)")
    parser.add_argument("--servo-host-port", type=int, default=SERVO_HOST_PORT,
                        help="Host-side TCP port the robot connects back to")
    parser.add_argument("--servo-freeze-dist", type=float, default=0.03,
                        help="m (default 0.03): once the arm is within this distance of the current "
                             "servo target, a still-converging prediction was found repeatedly nudging "
                             "the setpoint in small corrective reversals right through the settle phase - "
                             "the real servoj tracking lag this builds up (measured ~80mm) coincided with "
                             "4-5 real C153A0 protective stops in one 2026-07-21 session, all at exactly "
                             "this finish phase. Below this distance, a retarget is only accepted if it "
                             "clears --reaim-min (same 'meaningful movement' bar used for logging) - "
                             "otherwise the last commit is held so the arm can actually settle. Does not "
                             "touch the catch envelope/reach check, so full reach is unaffected - this "
                             "only governs when the last few cm of fine retargeting happen.")
    parser.add_argument("--no-idle-wobble", action="store_true",
                        help="Turn OFF the 'catcher waiting' animation (2026-08-25, servo mode only): by "
                             "default (on since 2026-09-11, user request) the tool orbits the wait pose in "
                             "a small circle while genuinely idle instead of sitting dead still - a visible "
                             "tell that the stream is live. Slow/small by design (see "
                             "IDLE_WOBBLE_DIAMETER_M/PERIOD_S) so it sits far under the servo limiter's "
                             "own speed/accel caps and never touches the catch envelope, feasibility, or "
                             "commit path - a throw still overwrites servo_target the instant it commits.")
    parser.add_argument("--idle-wobble-diameter", type=float, default=IDLE_WOBBLE_DIAMETER_M, metavar="M",
                        help=f"Idle wobble circle diameter, meters (default {IDLE_WOBBLE_DIAMETER_M}).")
    parser.add_argument("--idle-wobble-period", type=float, default=IDLE_WOBBLE_PERIOD_S, metavar="S",
                        help=f"Idle wobble seconds per lap (default {IDLE_WOBBLE_PERIOD_S}). Smaller = "
                             "faster/more energetic; keep an eye on peak speed (pi*diameter/period) and "
                             "centripetal accel (diameter*2*pi^2/period^2) staying well under "
                             "--servo-max-speed/--servo-max-accel if you change either default.")
    parser.add_argument("--catch-joint-speed", type=float, default=2.0,
                        help="rad/s leading-joint speed for --catch-move movej (default 2.0 ~ 115deg/s, "
                             "just under the 120deg/s base/shoulder limit)")
    parser.add_argument("--catch-joint-accel", type=float, default=5.0,
                        help="rad/s^2 for --catch-move movej (default 5.0 - conservative, characterize upward)")
    parser.add_argument("--no-yaw-follow", action="store_true",
                        help="Disable yaw-follow (on by default since 2026-07-17). Yaw-follow rotates the "
                             "catch-target orientation about base Z by the target's azimuth delta from the "
                             "wait pose, so the (rotationally symmetric, mouth-up) tool pans with the base "
                             "instead of the wrist fighting to hold a fixed world orientation through a side "
                             "sweep. Paired with --catch-move movej.")
    parser.add_argument("--tilt-follow", type=float, default=20.0, metavar="DEG",
                        help="Tilt the tool mouth into the incoming ball trajectory by up to DEG degrees "
                             "(default 20, on by daily use since 2026-07-27; pass 0 to disable). Recovers "
                             "the aperture lost to a slanted approach (cos(incidence)) and makes the wrist "
                             "visibly track the throw - yaw-follow alone keeps wrist joints deliberately "
                             "still, which is why no wrist motion was visible on 2026-07-17. Composes on "
                             "top of yaw-follow. Watch IK reachability near the envelope edge.")
    parser.add_argument("--approach-speed", type=float, default=3.0,
                        help="rad/s (joint-space - this move is a movej, not a movel, see move_to()) "
                             "for the move to/return-to wait pose (default 3.0, doubled 2026-07-27 from "
                             "the 2026-07-17 1.5 rad/s point to make the return-to-wait leg faster. Safe "
                             "to raise past the 120deg/s ~ 2.09rad/s base/shoulder max unlike a movel's "
                             "--speed: movej is joint-space, so the controller clamps whichever joint is "
                             "leading to its own physical/safety limit rather than faulting - a move only "
                             "actually reaches 3.0 rad/s when a faster joint (elbow/wrist, 180deg/s ~ "
                             "3.14rad/s max) is leading.)")
    parser.add_argument("--approach-accel", type=float, default=2.0,
                        help="rad/s^2 (joint-space) for the approach/return moves (default 2.0, doubled "
                             "2026-07-27 alongside --approach-speed so the higher cap is actually reachable "
                             "rather than accel-limited the whole move)")

    # Release guard ----------------------------------------------------------------
    parser.add_argument("--min-release-dist", type=float, default=1.0,
                        help="m (horizontal, base frame) a release must originate from to be treated as a "
                             "real throw - blocks hand-near-robot false releases (a real 2026-07-16 "
                             "self-collision started as one). Every real recorded throw released >=1.1m out; "
                             "0 disables. See check_release_guard().")
    parser.add_argument("--max-away-deg", type=float, default=100.0,
                        help="deg - a release moving more than this far off the toward-the-robot direction "
                             "(with real horizontal speed) is ignored as not-a-throw-at-the-arm "
                             "(real throws' p90 was 27deg). See check_release_guard().")

    # Re-aim ------------------------------------------------------------------------
    parser.add_argument("--no-reaim", action="store_true",
                        help="Disable post-commit re-aiming. By default, once the committed move has "
                             "finished (arm at rest) and the ball is still in flight, the loop keeps "
                             "refitting and - if the refined predicted catch point has drifted >"
                             "--reaim-min from the committed target, the correction move still fits the "
                             "time budget, and the new target passes the same catch envelope - sends a "
                             "short correction move. Fires only from rest (never preempts a running "
                             "move), so it uses the exact same primitive/safety path as the commit move. "
                             "Data motivation: commits fire at ~43 samples where prediction error is "
                             "6-17cm; by 67-80 samples it is 2-5cm (docs/debug_log.md 2026-07-17).")
    parser.add_argument("--reaim-min", type=float, default=0.02,
                        help="m minimum drift of the refined prediction from the committed target before "
                             "a correction move is worth sending (default 0.02)")
    parser.add_argument("--reaim-max-count", type=int, default=3,
                        help="max correction moves per throw (default 3)")
    parser.add_argument("--reaim-preempt", action="store_true",
                        help="EXPERIMENTAL (2026-07-18, off by default, not yet real-arm validated): "
                             "allow a re-aim correction to PREEMPT the still-running catch move instead "
                             "of waiting for the arm to settle. Data motivation: on 2026-07-17, re-aim "
                             "fired on only 2 of 98 attempts - 69 blocked because the arm never settled "
                             "before impact, 27 because no time remained after settling - so the "
                             "designed noise-repair loop was effectively dormant and every early-commit "
                             "error stayed locked in. The correction program starts with an explicit "
                             "stopj() so the preemption decelerates deliberately, but replacing a "
                             "running program mid-move is exactly the kind of thing to validate at low "
                             "--speed/--catch-joint-speed on the real arm before trusting. First "
                             "validation run: watch for protective stops at the preemption instant.")
    parser.add_argument("--reaim-preempt-min", type=float, default=0.05,
                        help="m minimum drift before a PREEMPTING correction (while the arm is still "
                             "moving) is sent - deliberately larger than --reaim-min so marginal "
                             "corrections don't repeatedly interrupt a good move (default 0.05)")

    # Commit / trust --------------------------------------------------------------
    parser.add_argument("--commit-samples", type=int, default=40,
                        help="min flight samples before a real catch may be fired (default 40, was 50 ~ 0.33s @120Hz). "
                             "Ignored in --catch-move servo - see --early-commit-samples.")
    parser.add_argument("--early-commit-samples", type=int, default=5,
                        help="servo mode ONLY (2026-07-22): min flight samples before the stream starts "
                             "driving toward the predicted intercept (default 5, vs --commit-samples' 40 for "
                             "movel/movej). Also drops the feasible-or-possible gate for this first move - it "
                             "fires as soon as a plane crossing exists and clears the envelope/release-guard "
                             "checks, same as any other tick. Safe only because servo mode's existing "
                             "continuous retargeting (every tick, unconditional on feasibility - see the "
                             "'attempted and servo_mode' branch below) keeps pulling the noisy early guess "
                             "toward the true intercept as more samples arrive; movel/movej have no such "
                             "correction and must keep waiting for --commit-samples instead. Must be >=3 "
                             "(fit_trajectory's hard floor).")
    parser.add_argument("--stability-window", type=int, default=3,
                        help="when this many recent catch-point predictions already agree within "
                             "--drift-tol, their average is used as the commit TARGET POINT (free "
                             "noise reduction) - does NOT gate whether/when a commit fires, which "
                             "happens on the first feasible-or-possible tick regardless (default 3)")
    parser.add_argument("--drift-tol", type=float, default=0.08,
                        help="m agreement threshold for the --stability-window average to be used as "
                             "the commit target point (default 0.08) - does not delay commit timing")

    # Visualization ------------------------------------------------------------
    parser.add_argument("--no-plot", action="store_true",
                        help="Skip the live throw-plot window. By default (on since 2026-07-27) a live "
                             "pop-up window (throw_plot.ThrowPlotWindow, same TkAgg backend as "
                             "visualize_trajectory.py) opens and updates once after each throw with a "
                             "2-panel view (top-down X-Y + reach/height side profile) of the ball's actual "
                             "trajectory, the robot's actual TCP path, and how the commit/re-aim/"
                             "servo-retarget target guesses converged toward the true catch point (colored "
                             "light->dark by fraction of the throw observed when each guess was made). The "
                             "window is updated synchronously from this poll loop (an interactive matplotlib "
                             "backend can't safely be driven from a background thread) - every artist is "
                             "pre-allocated once at startup and only mutated in place per throw, which "
                             "measured ~90-130ms per update (comfortable margin under DEMO_SOCK_TIMEOUT_S's "
                             "0.5s robot-side watchdog) even in --catch-move servo. Data collection itself (per-tick TCP pose, "
                             "per-guess target) is free the rest of the time - it reuses values the "
                             "feasibility/commit logic already computes every tick regardless of this flag. "
                             "Closing the window just stops future updates; the session keeps running.")

    parser.add_argument("--dry-run", action="store_true",
                        help="Log every decision but send NO motion at all (not even the initial positioning). "
                             "Use first to validate wait pose / plane / gating against your replay.")
    parser.add_argument("--no-record", action="store_true",
                        help="Skip recording. By default (on since 2026-07-27) every feasibility tick, "
                             "gate/commit/refuse decision, throw start/end, robot move, and the raw "
                             "per-throw ball trajectory is written to a compact JSONL log file "
                             "(catch_logs/catch_log_<timestamp>.jsonl, auto-named) for later analysis - "
                             "no Motive replay needed, the raw trajectory is in the log itself - see "
                             "CLAUDE.md 'Run recording'.")
    parser.add_argument("--yes", action="store_true", help="Skip the pre-motion confirmation prompt")
    parser.add_argument("--poll-hz", type=float, default=None,
                        help="Feasibility-check rate during flight (default 50, or --servo-rate when "
                             "--catch-move servo - in servo mode this loop also emits the setpoint "
                             "stream, so the two rates are the same thing. Was 20 - each poll interval"
                             "is pure decision latency before the commit fires: at 20Hz that cost 25ms mean/"
                             "50ms worst, ~3-6cm of arm travel. The full-flight fit is a few np.polyfit "
                             "calls, microseconds - 50Hz is still nowhere near the hot NatNet thread's "
                             "budget. Console prints are throttled separately, see PRINT_MIN_INTERVAL_S.)")
    parser.add_argument("--robot-ip", default=ROBOT_IP, help="UR12e controller IP")
    parser.add_argument("--no-wrapup", action="store_true",
                        help="Skip the end-of-session name/description prompt and robot log pull")
    parser.add_argument("--pull-robot-logs", action="store_true",
                        help="Also SSH-pull the robot controller's own log_history.txt/polyscope.log/flight-"
                             "report slice for this session (see pull_robot_session_logs()). Off by default "
                             "since 2026-08-25: the new UR10 controller doesn't have this laptop's key "
                             "authorized yet (SSH access there is still a roadmap TODO, see "
                             "docs/ur10_migration_roadmap.md), so it was falling back to an interactive "
                             "password prompt nobody has the password for - a long hang on every session end, "
                             "not a quick skip. The name/description prompt and notes.txt still always run; "
                             "this flag only gates the extra SSH round-trip on top of that. BatchMode=yes on "
                             "the ssh/scp calls themselves (see pull_robot_session_logs()) is a second, "
                             "independent guard so re-enabling this can never hang on a password prompt again "
                             "either, even by accident.")
    parser.add_argument("--no-beep", action="store_true",
                        help="Disable audio cues for predict/reaim/catch/miss events")
    args = parser.parse_args()
    args.plot = not args.no_plot  # --plot renamed to on-by-default --no-plot 2026-07-27; args.plot kept as the internal flag
    args.idle_wobble = not args.no_idle_wobble  # same pattern, 2026-09-11: flag inverted to on-by-default, internal name kept

    servo_mode = args.catch_move == "servo"
    if servo_mode and args.early_commit_samples < 3:
        raise SystemExit(f"--early-commit-samples must be >=3 (fit_trajectory needs 3 points), "
                          f"got {args.early_commit_samples}")
    # Effective sample floor for both "start evaluating" (replaces MIN_SAMPLES_FOR_CHECK)
    # and "may commit" (replaces --commit-samples) - servo mode uses its own much lower
    # floor since it alone can correct a noisy early guess every tick afterward.
    commit_samples_eff = args.early_commit_samples if servo_mode else args.commit_samples
    min_check_samples = commit_samples_eff if servo_mode else MIN_SAMPLES_FOR_CHECK
    if args.servo_orient_max_rate_deg_s is None:
        args.servo_orient_max_rate_deg_s = args.servo_base_rate_deg_s
    if args.servo_orient_max_accel_deg_s2 is None:
        args.servo_orient_max_accel_deg_s2 = args.servo_orient_max_rate_deg_s / 0.15
    if args.poll_hz is None:
        # In servo mode this loop IS the setpoint stream, so it must run at the
        # servo rate; otherwise keep the long-standing 50Hz default.
        args.poll_hz = args.servo_rate if servo_mode else 50.0

    record_path = None
    if not args.no_record:
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

    base_rb_mode = bool(args.base_rb_transform)  # "" (--base-rb-transform "") opts out of the on-by-default file
    R_base_rb = t_base_rb = base_rb_id = None
    if base_rb_mode:
        with open(args.base_rb_transform) as f:
            base_rb_data = json.load(f)
        R_base_rb = np.array(base_rb_data["R"])
        t_base_rb = np.array(base_rb_data["t"])
        base_rb_id = base_rb_data["rigid_body_id"]
        print(f"[EXPERIMENTAL] base<-mocap will be recomputed every tick from rigid body "
              f"id={base_rb_id} (--base-rb-transform {args.base_rb_transform}), not loaded static from "
              f"{args.transform_file}. Not yet validated on the real arm - see --base-rb-transform help.")
        print(f"[EXPERIMENTAL] getting one live reading of rigid body id={base_rb_id} now, so the catch "
              f"plane below is derived from the rig's CURRENT position rather than {args.transform_file}'s "
              f"static (possibly stale, e.g. after a rig move) snapshot...")
        R, t_vec = resolve_live_base_rb_transform(args.server_ip, args.local_ip, args.unicast, base_rb_id,
                                                   R_base_rb, t_base_rb)
        print("[EXPERIMENTAL] live base<-mocap resolved.")

    print(f"connecting to robot at {args.robot_ip} ...")
    rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    rec.attach_rtde(rtde_r)  # every JSONL line from here on carries robot_clock too
    dash = dashboard_client.DashboardClient(args.robot_ip)
    dash.connect()
    last_normal = LastNormal()  # see its docstring - bounds an undetected-fault window
    fault_count = 0  # total faults waited-out this session - see wait_for_fault_clear

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

    # Payload/CoG is set once via the pendant's Installation -> Payload Estimation
    # wizard instead of being sent by this script every run (2026-07-16, user
    # decision - "it never changes"). Deliberately different from set_tcp() above:
    # nothing else in this toolchain sets a different payload the way
    # calibrate_frames.py sets a different tcp_offset during calibration, and the
    # pendant's value is saved in the installation file (survives reboots), so
    # there's no "prior script left the wrong value" hazard to guard against here.
    # A wrong payload/CoG was the traced root cause of the 2026-07-16 base-joint
    # protective stops (docs/debug_log.md) - if they recur, check the pendant
    # value first before assuming this script's motion parameters are at fault.

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

    # --plot: a live pop-up window (throw_plot.ThrowPlotWindow), not a saved file - see
    # its module docstring for why it has to be driven synchronously from THIS thread
    # (main) rather than a background worker. Built here (as soon as wait_xyz/the catch
    # envelope constants are known) so it's already on screen during the "type 'go'"
    # confirmation prompt below, not just once the first throw happens.
    plot_window = None
    if args.plot:
        from throw_plot import ThrowPlotWindow
        plot_window = ThrowPlotWindow(wait_xyz, dict(
            reach_min=CATCH_MIN_REACH, reach_max=CATCH_MAX_REACH,
            z_min=CATCH_Z_MIN, z_max=CATCH_Z_MAX, max_azimuth_deg=CATCH_MAX_AZIMUTH_DEG,
        ))
        print("live throw-plot window open (updates once per throw; close it any time - the session keeps running)")

    print("=" * 78)
    print("CATCH - REAL ARM MOTION" + ("  [DRY RUN - no motion]" if args.dry_run else ""))
    print("=" * 78)
    transform_desc = (f"LIVE via base RB id={base_rb_id} ({args.base_rb_transform}, "
                      f"static fallback {args.transform_file} rmse={calib_rmse * 1000:.1f}mm) [EXPERIMENTAL]"
                      if base_rb_mode else f"{args.transform_file} (rmse={calib_rmse * 1000:.1f}mm)")
    print(f"transform: {transform_desc}  robotmode={robot_mode}")
    print(f"move-time model: accel={model.accel:.2f} m/s^2 latency={model.latency*1000:.0f}ms cruise={model.v_max:.2f} m/s")
    print(f"wait pose (base): pos=({wait_xyz[0]:+.3f},{wait_xyz[1]:+.3f},{wait_xyz[2]:+.3f}) reach={np.linalg.norm(wait_xyz):.2f}m  "
          f"orient=({wait_pose[3]:+.3f},{wait_pose[4]:+.3f},{wait_pose[5]:+.3f})")
    print(f"catch plane (mocap): {AXIS_NAMES[catch_axis_idx]} = {catch_value:.4f}")
    print(f"catch envelope: reach[{CATCH_MIN_REACH},{CATCH_MAX_REACH}]m  z[{CATCH_Z_MIN:+.2f},{CATCH_Z_MAX:+.2f}]m  "
          f"azimuth +/-{CATCH_MAX_AZIMUTH_DEG:.0f}deg of wait pose  (no cap on distance from wait pose)")
    if servo_mode:
        print(f"catch SERVO STREAM [EXPERIMENTAL]: v<={args.servo_max_speed} m/s a<={args.servo_max_accel} m/s^2 "
              f"base<={args.servo_base_rate_deg_s:.0f} deg/s   |   orient<={args.servo_orient_max_rate_deg_s:.0f} "
              f"deg/s a<={args.servo_orient_max_accel_deg_s2:.0f} deg/s^2")
        print(f"  stream: {args.servo_rate:.0f}Hz setpoints -> {args.servo_host_ip}:{args.servo_host_port}, "
              f"servoj gain={args.servo_gain:.0f} lookahead={args.servo_lookahead}   |   "
              f"approach movej: v={args.approach_speed} rad/s a={args.approach_accel} rad/s^2")
        print(f"  continuous retargeting: after commit the setpoint follows the refined prediction every "
              f"tick (--reaim* flags do not apply)")
    elif args.catch_move == "movej":
        print(f"catch movej: v={args.catch_joint_speed} rad/s a={args.catch_joint_accel} rad/s^2 "
              f"(joint-space, --catch-move movej)   |   "
              f"approach movej: v={args.approach_speed} rad/s a={args.approach_accel} rad/s^2")
    else:
        print(f"catch movel: v={args.speed} m/s a={args.accel} m/s^2   |   "
              f"approach movej: v={args.approach_speed} rad/s a={args.approach_accel} rad/s^2")
    reaim_desc = "n/a (servo mode retargets continuously)" if servo_mode else "off" if args.no_reaim else (
        f"ON (drift>{args.reaim_min*100:.0f}cm, max {args.reaim_max_count}/throw, "
        + (f"PREEMPT allowed >={args.reaim_preempt_min*100:.0f}cm [EXPERIMENTAL]" if args.reaim_preempt
           else "from rest only") + ")")
    print(f"yaw-follow: {'ON' if (not args.no_yaw_follow) else 'off'}   "
          f"tilt-follow: {('%.0fdeg [EXPERIMENTAL]' % args.tilt_follow) if args.tilt_follow > 0 else 'off'}   "
          f"re-aim: {reaim_desc}"
          f"   release guard: dist>={args.min_release_dist}m, away<={args.max_away_deg:.0f}deg")
    if servo_mode:
        print(f"commit: n>={commit_samples_eff} AND a plane crossing exists (feasibility gate BYPASSED - "
              f"--early-commit-samples) - fires on the FIRST qualifying tick; continuous retargeting then "
              f"pulls the noisy early guess toward the true intercept every tick")
    else:
        print(f"commit: n>={commit_samples_eff} AND (feasible OR possibly-catch) - fires on the FIRST "
              f"qualifying tick; uses the last {args.stability_window}-prediction average (agreeing within "
              f"{args.drift_tol}m) as the target point when already available, else the raw current prediction")
    if frac < 0.99:
        print(f"WARNING: pendant speed slider at {frac*100:.0f}% - catch moves will be capped there.")
    print("=" * 78)

    rec.log("run_start",
            dry_run=args.dry_run, robot_ip=args.robot_ip,
            transform_file=args.transform_file, calib_rmse_m=calib_rmse, calib_created=calib_created,
            base_rb_mode=base_rb_mode, base_rb_transform=args.base_rb_transform, base_rb_id=base_rb_id,
            tcp_offset=tcp_offset, robot_mode=robot_mode, speed_fraction=frac,
            wait_pose=wait_pose, catch_axis=AXIS_NAMES[catch_axis_idx], catch_value=catch_value,
            catch_envelope={"reach_min": CATCH_MIN_REACH, "reach_max": CATCH_MAX_REACH,
                            "z_min": CATCH_Z_MIN, "z_max": CATCH_Z_MAX,
                            "max_azimuth_deg": CATCH_MAX_AZIMUTH_DEG},
            move_time_model={"accel": model.accel, "latency": model.latency, "v_max": model.v_max,
                             "residual_rms": model.residual_rms, "n_legs": model.n_legs},
            speed_char_json=speed_char_json,
            catch_speed=args.speed, catch_accel=args.accel,
            catch_move=args.catch_move, catch_joint_speed=args.catch_joint_speed,
            catch_joint_accel=args.catch_joint_accel, yaw_follow=(not args.no_yaw_follow),
            reaim=not args.no_reaim, reaim_min=args.reaim_min, reaim_max_count=args.reaim_max_count,
            reaim_preempt=args.reaim_preempt, reaim_preempt_min=args.reaim_preempt_min,
            tilt_follow_deg=args.tilt_follow,
            min_release_dist=args.min_release_dist, max_away_deg=args.max_away_deg,
            approach_speed=args.approach_speed, approach_accel=args.approach_accel,
            commit_samples=args.commit_samples, early_commit_samples=args.early_commit_samples,
            commit_samples_eff=commit_samples_eff, stability_window=args.stability_window,
            drift_tol=args.drift_tol, margin=args.margin, box_radius=args.box_radius,
            cruise_speed=args.cruise_speed, poll_hz=args.poll_hz,
            servo=({"max_speed": args.servo_max_speed, "max_accel": args.servo_max_accel,
                    "base_rate_deg_s": args.servo_base_rate_deg_s, "rate": args.servo_rate,
                    "gain": args.servo_gain, "lookahead": args.servo_lookahead,
                    "freeze_dist": args.servo_freeze_dist,
                    "orient_max_rate_deg_s": args.servo_orient_max_rate_deg_s,
                    "orient_max_accel_deg_s2": args.servo_orient_max_accel_deg_s2,
                    "host": f"{args.servo_host_ip}:{args.servo_host_port}"} if servo_mode else None),
            idle_wobble=(args.idle_wobble and servo_mode),
            idle_wobble_diameter_m=args.idle_wobble_diameter, idle_wobble_period_s=args.idle_wobble_period)

    if not args.dry_run:
        initial_sweep = float(np.linalg.norm(np.array(current_pose[:3]) - wait_xyz))
        current_q_deg = [math.degrees(q) for q in rtde_r.getActualQ()]
        print(f"\n*** THE ROBOT WILL MOVE. *** First it drives to the wait pose ({initial_sweep:.2f}m TCP-straight-"
              f"line distance away) via a bounded movej at {args.approach_speed} rad/s - joint target resolved "
              f"robot-side (get_inverse_kin) from the arm's ACTUAL current joints "
              f"{[f'{v:+.0f}' for v in current_q_deg]} deg, not assumed. Then")
        if servo_mode:
            print("brings up a CONTINUOUS servoj setpoint stream and holds the wait pose with it - the arm")
            print("stays under live servo control for the WHOLE session, not just during a catch. Motion is")
            print(f"bounded host-side to {args.servo_max_speed} m/s / {args.servo_max_accel} m/s^2 / "
                  f"{args.servo_base_rate_deg_s:.0f} deg/s base. EXPERIMENTAL - never run on the real arm before.")
        else:
            print("fires fast catch moves toward thrown balls. Clear the area and keep the E-stop in hand.")
        if not args.yes:
            if input("Type 'go' to arm motion (anything else aborts): ").strip().lower() != "go":
                raise SystemExit("aborted.")
        print("\nmoving to wait pose...")
        t0 = time.time()
        settled, fault = move_to(wait_pose, args.approach_speed, args.approach_accel, rtde_r, dash, last_normal)
        rec.log("move", purpose="initial_wait_pose", target=wait_pose, speed=args.approach_speed,
                accel=args.approach_accel, settled=settled, fault=fault, duration_s=time.time() - t0)
        if fault is not None:
            fault_count = wait_for_fault_clear(rec, fault, fault_count, rtde_r, dash, last_normal, wait_pose, args)
            settled = True  # wait_for_fault_clear already drove to the wait pose once cleared
        if not settled:
            raise SystemExit("Did not reach the wait pose (timeout). Check the pendant / remote-control mode.")
        print("at wait pose. Ready - throw the ball. Press Enter (or Ctrl-C) to stop.\n")
    else:
        print("\n[dry-run] not moving. Feasibility from the arm's CURRENT pose. Throw the ball. "
              "Press Enter (or Ctrl-C) to stop.\n")

    # --- servo stream (--catch-move servo) ------------------------------------
    # Brought up AFTER the initial movej to the wait pose, never before: sending
    # any script to :30002 replaces the running program, so a movej issued while
    # the stream is live would silently kill it. Every send_script()/send_urscript()
    # call site in this file is therefore either before start_servo_stream() or
    # after the stream has already been torn down - if you add another, keep that
    # invariant.
    stream: Optional[ServoStream] = None
    limiter: Optional[RateLimiter] = None
    orient_limiter: Optional[OrientationRateLimiter] = None
    servo_dt = 1.0 / args.poll_hz
    servo_target = wait_xyz.copy()          # base-frame xyz the stream is driving toward
    servo_orient = list(wait_pose[3:6])     # orientation the stream is driving toward
    last_tick_mono = None                   # monotonic time of the previous emission
    last_servo_logged = None                # last servo_cmd position written to the log
    # Declared HERE, above start_servo_stream(), not down with the loop's other
    # counters: that helper assigns both, and it runs before the loop-state block.

    def set_servo_return_mode(returning: bool):
        """Switch the live RateLimiter between the normal catch-tracking cap and the
        faster --servo-return-mult cap used only for the wait-pose return leg (idle
        before a throw, and after one ends). No-op before the limiter exists (dry-run
        construction applies its own initial call right after building it)."""
        if limiter is None:
            return
        mult = args.servo_return_mult if returning else 1.0
        limiter.max_speed = args.servo_max_speed * mult
        limiter.max_accel = args.servo_max_accel * mult
        if limiter.max_base_rate is not None:
            limiter.max_base_rate = math.radians(args.servo_base_rate_deg_s) * mult
            limiter.max_base_accel = limiter.max_base_rate / 0.15

    def start_servo_stream():
        """(Re)open the stream and seed both limiters from where the arm actually is.

        Seeding from the measured pose rather than the wait pose matters on the
        restart-after-fault path: the arm may be anywhere, and a limiter seeded at
        a stale position would make its first step a jump rather than a bounded
        crawl - defeating the one layer that makes streaming safe at all. Same
        reasoning applies to orientation (OrientationRateLimiter), added 2026-07-22.
        """
        nonlocal stream, limiter, orient_limiter, servo_target, servo_orient, last_tick_mono, last_servo_logged
        st = ServoStream(tcp_offset, args.robot_ip, args.servo_host_ip, args.servo_host_port,
                         DEFAULT_SERVO_DT, args.servo_lookahead, args.servo_gain,
                         DEFAULT_STOP_ACCEL, DEMO_SOCK_TIMEOUT_S)
        st.start()
        here = list(rtde_r.getActualTCPPose())
        stream = st
        limiter = RateLimiter(clamp_to_envelope(np.array(here[:3])),
                              args.servo_max_speed, args.servo_max_accel,
                              math.radians(args.servo_base_rate_deg_s),
                              reach_bounds=reach_band_at_z, z_bounds=(CATCH_Z_MIN, CATCH_Z_MAX))
        orient_limiter = OrientationRateLimiter(here[3:6], math.radians(args.servo_orient_max_rate_deg_s),
                                                math.radians(args.servo_orient_max_accel_deg_s2))
        servo_target = wait_xyz.copy()
        servo_orient = list(wait_pose[3:6])
        last_tick_mono = None
        last_servo_logged = None
        set_servo_return_mode(True)  # (re)opening the stream always targets the wait pose first
        rec.log("servo_stream", state="up", seeded_at=here[:3])

    def stop_servo_stream(reason: str):
        nonlocal stream, limiter, orient_limiter
        if stream is not None:
            stream.stop()
            rec.log("servo_stream", state="down", reason=reason, sent=stream.sent)
            stream = None
            limiter = None
            orient_limiter = None

    def emit_setpoint():
        """Send exactly one setpoint. Must run on EVERY loop iteration.

        A nested function rather than inline code at the bottom of the loop
        specifically because the loop has several `continue` paths (no rigid body
        yet, post-fault resume, throw_end dedup) that would otherwise skip it.
        Skipping is not a missed update - it is silence, and --sock-timeout of
        silence makes the robot end its program and stop. The no-rigid-body path
        can spin indefinitely, so that one would reliably kill the stream.
        No-op unless the stream is up, so non-servo modes pay nothing.
        """
        nonlocal servo_target, servo_hold_logged, servo_hold_since, servo_hold_escalated, \
            last_tick_mono, last_servo_logged
        if limiter is None:
            return  # not servo mode (or the stream is down after a fault)

        # Real elapsed time, not the nominal period: the loop sleeps a fixed
        # 1/poll_hz WITHOUT subtracting its own work, so the true tick is always
        # longer than nominal and a nominal dt would make the limiter's speed cap
        # systematically wrong. Clamped because dt multiplies straight into step
        # size - a loop stalled by a fault wait or a long log write must not be
        # able to convert that pause into one huge jump.
        now_t = time.monotonic()
        dt_meas = servo_dt if last_tick_mono is None else now_t - last_tick_mono
        last_tick_mono = now_t
        dt_eff = min(max(dt_meas, 0.25 * servo_dt), 2.0 * servo_dt)

        # The reach/z envelope is enforced INSIDE the limiter (its reach_bounds/
        # z_bounds, braking per-axis in cylindrical state - see
        # ur_servo.RateLimiter's class docstring) - deliberately not by editing the
        # result here. Post-hoc edits are not rate-limited and destroy the
        # acceleration guarantee: 60+ m/s^2 measured against a 2.0 cap. Nothing
        # touches limiter's internal state from outside.
        cmd_xyz = limiter.step(servo_target, dt_eff)
        # Orientation rate-limited the same way position is (2026-07-22, fixing the
        # bug documented at length in OrientationRateLimiter's class docstring -
        # servo_orient used to be sent raw here, which could snap the whole
        # wait-pose->commit yaw delta onto the base joint in one tick).
        cmd_orient = list(orient_limiter.step(servo_orient, dt_eff))
        # Independent backstop on what is actually about to be sent, so a limiter
        # bug or a bad seed cannot slip past both this and the target checks
        # upstream. One degree of azimuth slack: the commit gate upstream is the
        # real azimuth decision, and this check runs on intermediate path points
        # where float noise at exactly the boundary would otherwise freeze the arm
        # mid-catch for no reason.
        env = check_catch_envelope(cmd_xyz, wait_xyz, CATCH_MAX_AZIMUTH_DEG + 1.0)
        if env is None:
            # stream is None in --dry-run: the limiter, clamp and envelope check all
            # still run (so a replay session validates exactly the setpoint path a
            # live run would take), only the send is skipped.
            alive = stream.send(list(cmd_xyz) + cmd_orient, servo=True) if stream else True
            servo_hold_since = None
            servo_hold_escalated = False
        else:
            here_pose = list(rtde_r.getActualTCPPose())
            here_xyz = np.array(here_pose[:3])
            # Reset the limiter's internal position/velocity state to where the robot
            # actually is, so next tick's step() recomputes fresh from a real, in-bounds
            # starting point - but do NOT touch servo_target. servo_target is the
            # caller's real destination (wait pose or a catch intercept); overwriting it
            # with "wherever we are right now" used to abandon that destination
            # silently and permanently (2026-07-27 incident: a hold fired once during
            # return-to-wait, servo_target got reset to the hold position, and the arm
            # just sat there - parked right next to the base's mounting stand - for the
            # rest of the throw instead of continuing home; see docs/debug_log.md).
            # Leaving servo_target alone lets the limiter's own reach/z braking
            # (reach_bounds/z_bounds, see ur_servo.RateLimiter) try again toward the
            # real target every tick instead of giving up after the first violation.
            limiter.reset(here_xyz)
            # Freeze orientation at its actual current value too, same reasoning as
            # the position freeze just above - otherwise orientation would keep
            # advancing toward servo_orient while position is stuck on an envelope
            # hold, desyncing the two the same coupling problem this class exists
            # to avoid.
            orient_limiter.reset(here_pose[3:6])
            cmd_orient = here_pose[3:6]
            alive = stream.send(here_pose[:3] + cmd_orient, servo=False) if stream else True
            if not servo_hold_logged:
                print(f"    >> SERVO HOLD (envelope): {env}")
                servo_hold_logged = True
            if servo_hold_since is None:
                servo_hold_since = now_t
            hold_duration = now_t - servo_hold_since
            rec.log("servo_hold", reason=env, cmd=cmd_xyz, duration_s=round(hold_duration, 3))
            # A hold that outlives SERVO_HOLD_STUCK_S means the limiter's own braking
            # isn't resolving this tick over tick - loud, not just the one-line
            # "SERVO HOLD" print above, since this is the failure mode that produced a
            # silent multi-hundred-ms stall next to the base in the 2026-07-27 incident.
            if not servo_hold_escalated and hold_duration >= SERVO_HOLD_STUCK_S:
                print(f"    !! servo hold stuck for {hold_duration:.1f}s (envelope: {env}) - "
                      f"limiter's reach/z braking isn't clearing this on its own")
                rec.log("servo_hold_stuck", reason=env, duration_s=round(hold_duration, 3))
                servo_hold_escalated = True

        # Record the commanded setpoint whenever it has actually moved. Distance-
        # gated rather than time-gated: it logs densely through a catch (which is
        # what you want to reconstruct afterwards) and goes quiet at the wait pose,
        # instead of emitting 125 near-identical lines a second all session.
        if last_servo_logged is None or float(np.linalg.norm(cmd_xyz - last_servo_logged)) >= 0.005:
            rec.log("servo_cmd", cmd=cmd_xyz, target=servo_target, cmd_orient=cmd_orient,
                    target_orient=servo_orient, dt=dt_eff, actual=list(rtde_r.getActualTCPPose()[:3]))
            last_servo_logged = cmd_xyz.copy()
        if not alive:
            # The far end went away without the safety check having noticed yet
            # (protective stop, or the program killed on the pendant). Don't keep
            # writing into a dead pipe - drop the stream and let the decimated
            # safety check drive recovery on a later tick.
            print("\n!!! servo stream died (robot-side program gone) - "
                  "waiting for the safety check to confirm and recover.")
            rec.log("servo_stream", state="died", reason="send failed")
            stop_servo_stream("send failed")

    if servo_mode and not args.dry_run:
        print("bringing up the servo setpoint stream...")
        start_servo_stream()
        print("stream up - the arm is now holding the wait pose under servoj.\n")
    elif servo_mode:
        # Dry-run: build both limiters but no stream. emit_setpoint() then runs the
        # whole setpoint path - rate limit, envelope clamp, envelope gate, logging -
        # and only skips the send, so a Motive replay validates exactly what a live
        # run would command without the arm moving at all.
        dry_run_pose = list(rtde_r.getActualTCPPose())
        limiter = RateLimiter(clamp_to_envelope(np.array(dry_run_pose[:3])),
                              args.servo_max_speed, args.servo_max_accel,
                              math.radians(args.servo_base_rate_deg_s),
                              reach_bounds=reach_band_at_z, z_bounds=(CATCH_Z_MIN, CATCH_Z_MAX))
        orient_limiter = OrientationRateLimiter(dry_run_pose[3:6], math.radians(args.servo_orient_max_rate_deg_s),
                                                math.radians(args.servo_orient_max_accel_deg_s2))
        set_servo_return_mode(True)  # starts targeting the wait pose, same as the live path
        print("[dry-run] servo setpoint path active (limiter + envelope), nothing sent.\n")

    # --- state ---
    s = SharedState()
    if args.rigid_body_id is not None:
        s.target_id = args.rigid_body_id

    client = NatNetClient(server_ip_address=args.server_ip, local_ip_address=args.local_ip,
                          use_multicast=not args.unicast)
    client.on_data_frame_received_event.handlers.append(make_handler(s, args))
    rb_state = BaseRBState()
    if base_rb_mode:
        client.on_data_frame_received_event.handlers.append(make_base_rb_handler(rb_state, base_rb_id))
    target_rb_state = BaseRBState()  # demo.py-only: tracks the "TARGET" rigid body (see find_target_pose)
    client.on_data_frame_received_event.handlers.append(make_base_rb_handler(target_rb_state, TARGET_RB_ID))

    last_state = "idle"
    attempted = False           # fired a catch for the current throw already
    refuse_logged = False       # throttle envelope-refusal spam within one throw
    guard_reason = None         # non-None: this throw failed the release guard, never commit
    committed_target = None     # base-frame xyz the last commit/re-aim was sent to
    reaim_count = 0             # correction moves sent for the current throw
    prev_margin = None          # result.margin on the last post-commit tick (abort trend, see below)
    non_improving_since = None  # monotonic time the current flat-or-worse-margin streak started, or None
    feasibility_cache: dict = {}  # per-throw cache for check_feasibility's fit/solve step - see its docstring
    abandoned = False           # True once the current throw's chase has been given up on
    last_result = None          # most recent FeasibilityResult computed this throw - feeds explain_miss()
    last_refuse_reason = None   # check_catch_envelope() reason text from the last refusal this throw
    last_refuse_target = None   # base-frame xyz that refusal was against - feeds explain_miss()
    catches = 0                 # session tally: attempted throws whose ball was last seen at the tool
    attempts_ended = 0          # attempted throws that reached throw_end (denominator for the tally)
    last_print_wall = 0.0       # console print throttle (ticks are recorded regardless)
    last_safety_check = 0.0     # monotonic time of the last dashboard safety query
    last_stream_reconnect_attempt = 0.0  # monotonic time of the last no-fault stream reopen attempt
    servo_hold_logged = False   # throttle envelope-hold spam within one throw
    servo_hold_since = None     # monotonic time the current unbroken hold streak started, or None
    servo_hold_escalated = False  # throttle the "stuck" escalation to once per hold streak
    last_flight_logged = None   # FlightRecord already given a throw_end/throw_samples (dedup
                                # between the normal flight->idle path and the post-fault path)
    throw_tcp_trace: List[tuple] = []   # --plot only: [(n_ball_samples, tcp_xyz), ...] this throw
    throw_guesses: List[dict] = []      # --plot only: [{"n","point","kind","verdict"}, ...] this throw
    pending_dump = False         # demo.py only: caught_guess was True for the last throw - once the
                                  # arm is actually back at the wait pose, look for TARGET and dump
    pending_dump_since = None   # monotonic time pending_dump last became True - see DUMP_SEARCH_WOBBLE_GRACE_S
    dump_skip_logged = False    # throttle "TARGET out of range" spam while waiting on one dump opportunity
    PRINT_MIN_INTERVAL_S = 0.08  # ~12 lines/s max during flight - readable at --poll-hz 50
    # A ball that disappears within this of the tool was (almost certainly) swallowed
    # by the box - it occludes its own markers. Validated against all 74 classifiable
    # committed throws of 2026-07-16: agrees with the session notes at ~91% catch
    # rate; misses were last seen 1.5m+ away. See docs/debug_log.md 2026-07-17.
    CAUGHT_LAST_SEEN_DIST_M = 0.30
    pred_window: deque = deque(maxlen=args.stability_window)

    def catch_move_script(pose: List[float], preempt: bool = False) -> str:
        """The one place a catch/correction move gets turned into URScript - movel
        (straight Cartesian line, current default) or movej (joint-space, immune to
        the side-target base-joint speed violation) per --catch-move. With
        preempt=True (--reaim-preempt corrections only) the program leads with an
        explicit stopj() so preempting a still-running move decelerates
        deliberately rather than relying on the controller's implicit
        program-replacement stop."""
        if args.catch_move == "movej":
            qnear = list(rtde_r.getActualQ())
            script = movej_to_pose_script(pose, qnear, args.catch_joint_speed, args.catch_joint_accel)
        else:
            script = movel_absolute_script(pose, args.speed, args.accel)
        if preempt:
            script = script.replace("def prog():\n", f"def prog():\n  stopj({args.catch_joint_accel})\n", 1)
        return script

    def target_orientation(point: np.ndarray, impact_vel=None) -> List[float]:
        if (not args.no_yaw_follow):
            rv = yaw_follow_orientation(wait_pose, wait_xyz, point)
        else:
            rv = [wait_pose[3], wait_pose[4], wait_pose[5]]
        if args.tilt_follow > 0.0 and impact_vel is not None:
            rv = tilt_follow_orientation(rv, impact_vel, math.radians(args.tilt_follow))
        return rv

    def stable() -> Optional[np.ndarray]:
        """Mean predicted catch point if the window is full and agrees within drift-tol, else None."""
        if len(pred_window) < args.stability_window:
            return None
        arr = np.array(pred_window)
        mean = arr.mean(axis=0)
        if float(np.max(np.linalg.norm(arr - mean, axis=1))) > args.drift_tol:
            return None
        return mean

    session_start_wall = time.time()
    stop_reason = None
    with client:
        client.run_async()
        try:
            while True:
                if enter_pressed():
                    stop_reason = "user_enter"
                    break

                # Cheap (~0.1ms) - run every iteration regardless of which branch/
                # continue below fires, so the window stays responsive to resize/
                # close between throws instead of only reacting at the next
                # plot_window.update() (which may be many seconds away).
                if plot_window is not None:
                    plot_window.pump()

                # Catches a fault from the fire-and-forget catch movel too (that path
                # sends via raw send_script(), not move_to(), so it has no built-in
                # settle/fault check of its own) - within one poll interval of it
                # happening, not retroactively at throw_end. Skipped in --dry-run:
                # no motion is ever sent there, so an unrelated fault shouldn't
                # interrupt a pure perception-testing session.
                now_mono = time.monotonic()
                if not args.dry_run and now_mono - last_safety_check >= SAFETY_CHECK_MIN_INTERVAL_S:
                    last_safety_check = now_mono
                    fault = check_safety_mode(rtde_r, dash, last_normal)
                    if fault is not None:
                        # A protective stop kills the robot-side servo program, so the
                        # stream is already dead here - tear it down explicitly before
                        # wait_for_fault_clear(), whose recovery movej goes out over
                        # :30002 and must not race a half-open stream.
                        stop_servo_stream("fault")
                        fault_count = wait_for_fault_clear(rec, fault, fault_count, rtde_r, dash, last_normal, wait_pose, args)
                        # A flight that ended while the arm was frozen/waiting never
                        # reaches the normal flight->idle logging below (last_state is
                        # force-reset here), which made faulted throws the one class
                        # with NO throw_end/throw_samples in the log - exactly the
                        # throws forensics needs most (the 151342 self-collision throw
                        # could not be reconstructed because of this). Capture it now.
                        with STATE_LOCK:
                            hh = s.history[0] if s.history else None
                        if hh is not None and hh is not last_flight_logged:
                            rec.log("throw_end", reason=f"{hh.reason} (logged post-fault)",
                                    duration=hh.duration, samples=hh.samples,
                                    peak_speed=hh.peak_speed, attempted=attempted,
                                    guarded=guard_reason is not None, reaims=reaim_count,
                                    arm_tcp_at_end=list(rtde_r.getActualTCPPose()),
                                    ball_last_dist_m=None, caught_guess=None)
                            if hh.raw_samples:
                                rec.log("throw_samples", t=hh.raw_samples[0].t,
                                        raw=[[smp.t, smp.x, smp.y, smp.z] for smp in hh.raw_samples])
                            last_flight_logged = hh
                        attempted = False
                        refuse_logged = False
                        servo_hold_logged = False
                        servo_hold_since = None
                        servo_hold_escalated = False
                        guard_reason = None
                        committed_target = None
                        reaim_count = 0
                        prev_margin = None
                        non_improving_since = None
                        pending_dump = False  # fault preempts any dump owed for the throw that just ended
                        pending_dump_since = None
                        dump_skip_logged = False
                        feasibility_cache = {}
                        abandoned = False
                        last_result = None
                        last_refuse_reason = None
                        last_refuse_target = None
                        pred_window.clear()
                        throw_tcp_trace = []
                        throw_guesses = []
                        last_state = "idle"
                        # wait_for_fault_clear() has driven back to the wait pose with a
                        # movej; only now is it safe to re-open the stream (see the
                        # :30002-preempts-the-running-program invariant above).
                        if servo_mode and not args.dry_run:
                            print("    re-opening the servo stream...")
                            try:
                                start_servo_stream()
                            except SystemExit as e:
                                # ServoStream.start() raises SystemExit if the robot
                                # never dials back within accept_timeout. A REAL fault
                                # is exactly the case where this is most likely (robot
                                # not yet back in RUNNING mode, Remote Control dropped,
                                # etc.) and exactly the case whose logs matter most -
                                # letting this SystemExit propagate uncaught used to
                                # skip the try/finally's wrap_up_session() call below
                                # entirely (SystemExit isn't KeyboardInterrupt, so the
                                # loop's `except KeyboardInterrupt` didn't catch it,
                                # and wrap_up_session() sits after the try/finally on
                                # purpose - see its call site), so a real fault's own
                                # log_history.txt/polyscope.log/flight report was never
                                # pulled. End the session cleanly instead so the normal
                                # finally + wrap_up_session still run. Real 2026-07-27
                                # incident - see docs/debug_log.md.
                                print(f"    !! could not re-open servo stream: {e}")
                                print("    ending session here so the fault's logs can still be pulled.")
                                rec.log("servo_stream", state="reopen_failed", reason=str(e))
                                stop_reason = "servo_reconnect_failed"
                                break
                        emit_setpoint()
                        continue
                    elif stream is None and servo_mode:
                        # 2026-08-26 real incident: the stream can also die from a plain
                        # send failure (robot-side watchdog ended its own program) with
                        # the robot's safety mode never actually leaving NORMAL - the
                        # branch above is the ONLY other thing that reopens the stream,
                        # and it never runs when check_safety_mode() reports no fault, so
                        # a session could sit here for the rest of its life with the arm
                        # silently frozen (throw 2 of that session: full decision pipeline
                        # ran normally, zero setpoints ever reached the robot). Rate-limited
                        # (STREAM_RECONNECT_MIN_INTERVAL_S) rather than every tick, so a
                        # genuinely unreachable robot doesn't get hammered with a fresh
                        # program upload every ~8ms - a failed attempt already costs ~8s
                        # of its own (ServoStream.start's accept_timeout) on top of that.
                        if now_mono - last_stream_reconnect_attempt >= STREAM_RECONNECT_MIN_INTERVAL_S:
                            last_stream_reconnect_attempt = now_mono
                            print("    servo stream is down (no fault detected) - attempting to reopen...")
                            try:
                                start_servo_stream()
                                print("    servo stream reopened.")
                            except SystemExit as e:
                                # Same trap as the fault-recovery reconnect above, but NOT
                                # treated as session-ending here: unlike that path, there is
                                # no confirmed real fault, so this may well be transient
                                # (the robot momentarily busy, a network blip) - just log and
                                # let the next cooldown tick try again rather than tearing
                                # down a session over what might resolve on its own.
                                print(f"    !! could not reopen servo stream: {e}")
                                rec.log("servo_stream", state="reopen_failed", reason=str(e))
                        emit_setpoint()
                        continue

                with STATE_LOCK:
                    target_id = s.target_id
                    candidate_ids = list(s.candidate_ids)
                    state = s.state
                    flight_buffer = list(s.flight_buffer)
                    history_head = s.history[0] if s.history else None

                # EXPERIMENTAL (--base-rb-transform, 2026-07-23): recompute base<-mocap
                # from the base rigid body's LIVE pose instead of the static R/t loaded
                # at startup. If the RB isn't valid this tick (occlusion), deliberately
                # keep the last-good R/t rather than falling back to something - going
                # stale-but-consistent for one tick is safer than a discontinuous jump.
                if base_rb_mode:
                    with BASE_RB_LOCK:
                        rb_pos, rb_rot, rb_valid = rb_state.latest_pos, rb_state.latest_rot, rb_state.latest_valid
                    if rb_valid:
                        R_mocap_rb = quat_to_matrix(rb_rot)
                        R, t_vec = base_from_mocap_via_rigid_body(R_base_rb, t_base_rb, R_mocap_rb, np.array(rb_pos))

                if target_id is None:
                    if candidate_ids:
                        print(f"Multiple rigid bodies {candidate_ids}; re-run with --rigid-body-id <id>.")
                    emit_setpoint()
                    time.sleep(1.0 / args.poll_hz)
                    continue

                if state == "flight" and last_state == "idle":
                    print(f"--- throw detected (rigid body {target_id}) ---")
                    attempted = False
                    refuse_logged = False
                    servo_hold_logged = False
                    servo_hold_since = None
                    servo_hold_escalated = False
                    pending_dump = False  # a new throw preempts any dump still owed - box is occupied again
                    pending_dump_since = None
                    dump_skip_logged = False
                    committed_target = None
                    reaim_count = 0
                    prev_margin = None
                    non_improving_since = None
                    feasibility_cache = {}
                    abandoned = False
                    last_result = None
                    last_refuse_reason = None
                    last_refuse_target = None
                    pred_window.clear()
                    throw_tcp_trace = []
                    throw_guesses = []
                    rec.throw += 1
                    if base_rb_mode and args.catch_value is None:
                        # Re-derive against the CURRENT live transform (just refreshed above)
                        # rather than trusting whatever resolve_live_base_rb_transform() saw
                        # before the loop even started - self-corrects if the rig gets bumped
                        # mid-session, and if that startup reading was itself still-settling
                        # (the 2026-07-27 incident this guards against - see
                        # resolve_live_base_rb_transform's docstring).
                        new_catch_value = derive_catch_plane(wait_xyz, R, t_vec)
                        if abs(new_catch_value - catch_value) > 0.01:
                            print(f"    catch plane drifted: {catch_value:.4f} -> {new_catch_value:.4f} "
                                  f"(mocap {AXIS_NAMES[catch_axis_idx]}) - re-deriving from live base RB")
                        catch_value = new_catch_value
                    rec.log("throw_start", t=flight_buffer[-1].t if flight_buffer else None,
                            rigid_body_id=target_id, arm_tcp=list(rtde_r.getActualTCPPose()),
                            catch_value=catch_value)
                    # Release guard: judged once, on the first ~10 samples, before any
                    # feasibility tick may commit - see check_release_guard().
                    guard_reason = check_release_guard(flight_buffer, R, t_vec,
                                                       args.min_release_dist, args.max_away_deg)
                    if guard_reason is not None:
                        print(f"    >> GUARDED (no commit this throw): {guard_reason}")
                        rec.log("guard", t=flight_buffer[-1].t if flight_buffer else None,
                                reason=guard_reason)

                if state == "flight" and guard_reason is None and len(flight_buffer) >= min_check_samples:
                    current_tcp_xyz = np.array(rtde_r.getActualTCPPose()[:3])  # RTDE FK only - never mocap
                    if args.plot:
                        throw_tcp_trace.append((len(flight_buffer), current_tcp_xyz.copy()))
                    result = check_feasibility(
                        flight_buffer, catch_axis_idx, catch_value, R, t_vec, current_tcp_xyz,
                        model, CATCH_MIN_REACH, CATCH_MAX_REACH, args.margin, args.box_radius,
                        cache=feasibility_cache,
                    )
                    last_result = result  # feeds explain_miss() at throw_end regardless of outcome below
                    if result.crossing_t is not None:
                        pred_window.append(result.catch_point_base)
                        now_wall = time.time()
                        if now_wall - last_print_wall >= PRINT_MIN_INTERVAL_S:
                            print(format_result(result))
                            last_print_wall = now_wall

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
                        # The early commit's target IS noisy (6-17cm at ~43 samples vs 2-5cm
                        # at 67-80) - that's what the post-commit re-aim below repairs, from
                        # rest, once better data exists.
                        # Servo mode (2026-07-22): commit_samples_eff is --early-commit-samples
                        # (default 5), not --commit-samples, AND `gate` is bypassed below (`or
                        # servo_mode`) - the first move fires on any valid plane crossing that
                        # clears the envelope/release-guard, trusting the continuous retarget
                        # branch further down (attempted and servo_mode) to pull a noisy 5-sample
                        # guess toward the true intercept every tick after. movel/movej have no
                        # such correction, so they still wait for the full gate + 40 samples.
                        commit_point = trusted if trusted is not None else result.catch_point_base
                        rec.log("tick", t=flight_buffer[-1].t, n=result.n_samples,
                                verdict=_verdict(result), reach=result.reach, from_tcp=current_tcp_xyz,
                                target=result.catch_point_base, dist_to_go=result.move_dist,
                                move_time=result.move_time, t_impact=result.time_to_impact,
                                margin=result.margin, shortfall=result.shortfall,
                                stable=trusted is not None, trusted_point=trusted,
                                post_commit=attempted,
                                commit_ready=len(flight_buffer) >= commit_samples_eff)

                        # Chase-abort trend check - see ABORT_NON_IMPROVING_S above. Only
                        # meaningful once a catch has actually been committed; a negative margin
                        # before that is just the normal wait for the gate to open. Tracked as a
                        # wall-clock streak duration (not a tick count) so the tolerance means the
                        # same ~0.16s regardless of --poll-hz/--servo-rate.
                        if attempted and not abandoned and result.margin is not None:
                            if result.margin < 0 and (prev_margin is None
                                                       or result.margin <= prev_margin + ABORT_MARGIN_EPS):
                                if non_improving_since is None:
                                    non_improving_since = now_mono
                            else:
                                non_improving_since = None
                            prev_margin = result.margin
                            if non_improving_since is not None and now_mono - non_improving_since >= ABORT_NON_IMPROVING_S:
                                abandoned = True
                                streak_s = now_mono - non_improving_since
                                print(f"    >> ABORT: margin not improving for {streak_s:.2f}s "
                                      f"(last margin={result.margin:+.3f}s) - holding, no longer chasing this throw")
                                rec.log("abort", t=flight_buffer[-1].t, n=result.n_samples,
                                        margin=result.margin, streak_s=round(streak_s, 3))

                        if not attempted and (gate or servo_mode) and len(flight_buffer) >= commit_samples_eff:
                            orient = target_orientation(commit_point, result.impact_vel_base)
                            target_pose = [float(commit_point[0]), float(commit_point[1]), float(commit_point[2]),
                                           orient[0], orient[1], orient[2]]
                            reason = check_catch_envelope(commit_point, wait_xyz)
                            if reason is not None:
                                last_refuse_reason = reason  # feeds explain_miss() at throw_end
                                last_refuse_target = np.array(commit_point, dtype=float)
                                if not refuse_logged:
                                    print(f"    >> REFUSED (envelope): {reason} - no motion sent")
                                    refuse_logged = True
                                rec.log("refuse", t=flight_buffer[-1].t, reason=reason, target=commit_point)
                            else:
                                # "EARLY" only reachable via the servo gate bypass above -
                                # movel/movej always have feasible or possible true here.
                                verdict = "CATCH" if result.feasible else "POSSIBLY" if result.possible else "EARLY"
                                stability_note = "" if trusted is not None else "  [instant, pred_window not full yet]"
                                print(f"    >> COMMIT ({verdict}){stability_note}: {args.catch_move} to "
                                      f"({commit_point[0]:+.3f},{commit_point[1]:+.3f},{commit_point[2]:+.3f}) "
                                      + (f"v={args.catch_joint_speed}rad/s a={args.catch_joint_accel}rad/s^2"
                                         if args.catch_move == "movej" else f"v={args.speed} a={args.accel}")
                                      + ("   [dry-run: not sent]" if args.dry_run else ""))
                                if not args.no_beep:
                                    beep.play("predict")
                                rec.log("commit", t=flight_buffer[-1].t,
                                        verdict="catch" if result.feasible else "possible" if result.possible else "early",
                                        target_pose=target_pose, speed=args.speed, accel=args.accel,
                                        move_kind=args.catch_move, yaw_follow=(not args.no_yaw_follow),
                                        stable=trusted is not None, dry_run=args.dry_run)
                                if servo_mode:
                                    # No program send, no socket connect, no move to
                                    # start - just a new destination for the stream
                                    # that is already running. This is the whole point
                                    # of servo mode: commit latency is one tick.
                                    set_servo_return_mode(False)  # back to normal chase speed - no longer returning
                                    servo_target = np.array(commit_point, dtype=float)
                                    servo_orient = orient
                                elif not args.dry_run:
                                    send_script(catch_move_script(target_pose))
                                committed_target = np.array(commit_point, dtype=float)
                                attempted = True
                                if args.plot:
                                    throw_guesses.append({"n": result.n_samples,
                                                         "point": np.array(commit_point, dtype=float),
                                                         "kind": "commit", "verdict": verdict})

                        elif attempted and not abandoned and servo_mode and committed_target is not None:
                            # Continuous retargeting - servo mode's replacement for the
                            # whole re-aim mechanism. Re-aim existed because a discrete
                            # move had to FINISH before another could sensibly start,
                            # which on 2026-07-17 data meant it fired on 2 of 98
                            # attempts (docs/debug_log.md 2026-07-18 section 2). Here
                            # there is no move to finish: the setpoint simply follows
                            # the refined prediction, every tick, for free. No drift
                            # threshold, no count limit, no preemption question - those
                            # were all artefacts of the discrete-move model.
                            new_point = trusted if trusted is not None else result.catch_point_base
                            env = check_catch_envelope(np.array(new_point), wait_xyz)
                            if env is None:
                                drift = float(np.linalg.norm(np.array(new_point) - committed_target))
                                # Finish-phase freeze (2026-07-21): see --servo-freeze-dist help.
                                # Once the arm is nearly at the current target, only take a new
                                # retarget if it clears the same drift bar used for logging below -
                                # otherwise hold the last commit instead of chasing sub-cm prediction
                                # jitter that the real servoj stream can't fully track at speed.
                                # Envelope/reach check above is untouched, so this never shrinks reach.
                                dist_to_target = float(np.linalg.norm(np.array(new_point) - current_tcp_xyz))
                                near_finish = dist_to_target < args.servo_freeze_dist
                                if not near_finish or drift >= args.reaim_min:
                                    servo_target = np.array(new_point, dtype=float)
                                    servo_orient = target_orientation(np.array(new_point), result.impact_vel_base)
                                    committed_target = np.array(new_point, dtype=float)
                                    if drift >= args.reaim_min:
                                        # Logged only on meaningful movement: at 125Hz an
                                        # unconditional event per tick would bury the log.
                                        rec.log("retarget", t=flight_buffer[-1].t, n=result.n_samples,
                                                drift=drift, target=new_point,
                                                t_impact=result.time_to_impact)
                                        if not args.no_beep:
                                            beep.play("reaim")
                                        if args.plot:
                                            throw_guesses.append({"n": result.n_samples,
                                                                 "point": np.array(new_point, dtype=float),
                                                                 "kind": "retarget", "verdict": None})

                        elif (attempted and not abandoned and not args.no_reaim and committed_target is not None
                              and reaim_count < args.reaim_max_count):
                            # Post-commit re-aim: the commit above fired at the earliest
                            # eligible tick, on a deliberately-early (noisy) prediction. The
                            # fit keeps refining while the arm travels/waits - if the arm has
                            # already ARRIVED (at rest; this never preempts a running move)
                            # and the refined prediction has drifted meaningfully off the
                            # committed target with enough time left to correct, send one
                            # short correction move through the exact same envelope check and
                            # motion primitive as the commit itself.
                            settled = args.dry_run or max(
                                abs(v) for v in rtde_r.getActualTCPSpeed()) < 0.01
                            new_point = trusted if trusted is not None else result.catch_point_base
                            drift = float(np.linalg.norm(np.array(new_point) - committed_target))
                            # --reaim-preempt (2026-07-18): the settle-first design turned out
                            # effectively dormant on real data (2 firings in 98 attempts; the
                            # arm's travel time consumed the whole remaining flight) - opting in
                            # lets a big-enough drift interrupt the running move instead, at a
                            # deliberately higher threshold (--reaim-preempt-min).
                            preempting = (not settled) and args.reaim_preempt
                            min_drift = args.reaim_min if settled else args.reaim_preempt_min
                            if (settled or preempting) and drift >= min_drift:
                                corr_dist = float(np.linalg.norm(np.array(new_point) - current_tcp_xyz))
                                corr_time = model.estimate(corr_dist)
                                if (result.time_to_impact is not None
                                        and result.time_to_impact > corr_time + args.margin
                                        and check_catch_envelope(np.array(new_point), wait_xyz) is None):
                                    orient = target_orientation(np.array(new_point), result.impact_vel_base)
                                    corr_pose = [float(new_point[0]), float(new_point[1]), float(new_point[2]),
                                                 orient[0], orient[1], orient[2]]
                                    reaim_count += 1
                                    print(f"    >> RE-AIM #{reaim_count}{' (preempt)' if preempting else ''}: "
                                          f"drift {drift*100:.1f}cm, "
                                          f"{args.catch_move} to ({new_point[0]:+.3f},{new_point[1]:+.3f},"
                                          f"{new_point[2]:+.3f}), corr_time={corr_time:.2f}s "
                                          f"t_impact={result.time_to_impact:.2f}s"
                                          + ("   [dry-run: not sent]" if args.dry_run else ""))
                                    if not args.no_beep:
                                        beep.play("reaim")
                                    rec.log("reaim", t=flight_buffer[-1].t, n=result.n_samples,
                                            count=reaim_count, drift=drift, target_pose=corr_pose,
                                            corr_time=corr_time, t_impact=result.time_to_impact,
                                            preempt=preempting,
                                            stable=trusted is not None, dry_run=args.dry_run)
                                    if not args.dry_run:
                                        send_script(catch_move_script(corr_pose, preempt=preempting))
                                    committed_target = np.array(new_point, dtype=float)
                                    if args.plot:
                                        throw_guesses.append({"n": result.n_samples,
                                                             "point": np.array(new_point, dtype=float),
                                                             "kind": "reaim", "verdict": None})
                    else:
                        rec.log("tick", t=flight_buffer[-1].t, n=result.n_samples, verdict="no_crossing",
                                note=result.note)

                if state == "idle" and last_state == "flight":
                    if history_head is not None:
                        print(f"--- throw ended ({history_head.reason}), dur={history_head.duration:.2f}s "
                              f"n={history_head.samples} peak={history_head.peak_speed:.2f}m/s ---")
                    end_tcp = list(rtde_r.getActualTCPPose())
                    # Catch/miss heuristic: a caught ball disappears INTO the box (its own
                    # markers get occluded), so "last seen right at the tool, then gone" is
                    # the catch signature - a missed ball is last seen sailing past/landing
                    # 1.5m+ away. Validated against the 74 classifiable committed throws of
                    # 2026-07-16 (docs/debug_log.md 2026-07-17). A heuristic for the tally/
                    # log, not a control input.
                    caught_guess = None
                    ball_last_dist = None
                    excuse = None  # feeds plot_window's excuse_text - stays None on a catch (no excuse needed)
                    if history_head is not None and history_head is last_flight_logged:
                        # already logged by the post-fault capture above - don't double-log
                        last_state = state
                        emit_setpoint()
                        time.sleep(1.0 / args.poll_hz)
                        continue
                    last_flight_logged = history_head
                    if history_head is not None and history_head.raw_samples:
                        last_s = last_plausible_ball_sample(history_head.raw_samples)
                        last_base = mocap_point_to_base(np.array([last_s.x, last_s.y, last_s.z]), R, t_vec)
                        ball_last_dist = float(np.linalg.norm(last_base - np.array(end_tcp[:3])))
                        if attempted:
                            caught_guess = ball_last_dist < CAUGHT_LAST_SEEN_DIST_M
                            attempts_ended += 1
                            if caught_guess:
                                catches += 1
                                print(f"    CATCH! (ball last seen {ball_last_dist*100:.0f}cm from tool)"
                                      f"   session: {catches}/{attempts_ended} attempted")
                                if not args.no_beep:
                                    beep.play("catch")
                            else:
                                print(f"    missed - ball last seen {ball_last_dist:.2f}m from tool"
                                      f"   session: {catches}/{attempts_ended} attempted")
                                if not args.no_beep:
                                    beep.play("miss")
                                # Excuse goes to the plot window (excuse_text), not the console -
                                # see explain_miss()'s docstring. Still recorded in throw_end below
                                # so a session can be researched later without needing to have had
                                # the plot window open.
                                excuse = explain_miss(guard_reason, attempted, abandoned,
                                                       last_refuse_reason, last_refuse_target,
                                                       last_result, history_head, ball_last_dist,
                                                       committed_target, wait_xyz)
                        else:
                            # Never committed at all (guarded/refused/no crossing) - still a
                            # miss from the demo audience's point of view, just explain why
                            # the arm never even tried.
                            excuse = explain_miss(guard_reason, attempted, abandoned,
                                                   last_refuse_reason, last_refuse_target,
                                                   last_result, history_head, ball_last_dist)
                    rec.log("throw_end",
                            reason=history_head.reason if history_head else None,
                            duration=history_head.duration if history_head else None,
                            samples=history_head.samples if history_head else None,
                            peak_speed=history_head.peak_speed if history_head else None,
                            attempted=attempted, arm_tcp_at_end=end_tcp,
                            guarded=guard_reason is not None, reaims=reaim_count,
                            ball_last_dist_m=ball_last_dist, caught_guess=caught_guess, excuse=excuse)
                    # The actual ground-truth ball trajectory for this throw, not just
                    # its summary stats - a separate event (not folded into throw_end)
                    # so a plain grep for throw_end stays small/scannable while this
                    # bulkier line is opt-in to look at. This is what lets a session be
                    # researched later from catch_logs/ alone, with no need to go back
                    # into Motive replay - see FlightRecord.raw_samples' docstring for
                    # why it has to be captured here (history_head) and not from
                    # flight_buffer, which is already empty by this point.
                    if history_head is not None and history_head.raw_samples:
                        rec.log("throw_samples", t=history_head.raw_samples[0].t,
                                raw=[[s.t, s.x, s.y, s.z] for s in history_head.raw_samples])
                    if plot_window is not None and history_head is not None and history_head.raw_samples:
                        # Trimmed, not raw: a ghost-marker tail (see trim_ghost_tail's
                        # docstring) is real, unedited data and belongs in the JSONL log
                        # above, but has no business being drawn as part of the ball's
                        # path - it isn't where the ball went.
                        plot_samples = trim_ghost_tail(history_head.raw_samples)
                        ball_base = np.array([
                            mocap_point_to_base(np.array([s.x, s.y, s.z]), R, t_vec)
                            for s in plot_samples
                        ])
                        ball_t = np.array([s.t for s in plot_samples])
                        plot_window.update(
                            rec.throw, ball_base, ball_t, throw_tcp_trace, throw_guesses,
                            meta=dict(attempted=attempted, caught_guess=caught_guess,
                                     ball_last_dist_m=ball_last_dist, duration=history_head.duration,
                                     samples=history_head.samples, catch_move=args.catch_move,
                                     dry_run=args.dry_run, reason=history_head.reason, excuse=excuse,
                                     session_catches=catches, session_attempts=attempts_ended),
                        )
                    if servo_mode:
                        # Just re-aim the stream at the wait pose - no blocking movej,
                        # so the loop stays live for the next throw while the arm is
                        # still drifting home. Feasibility always reads the arm's ACTUAL
                        # TCP, so a throw arriving mid-return is handled correctly rather
                        # than against a stale "we are at the wait pose" assumption.
                        set_servo_return_mode(True)  # no longer chasing - the return leg gets the faster cap
                        servo_target = wait_xyz.copy()
                        servo_orient = list(wait_pose[3:6])
                        if attempted:
                            rec.log("move", purpose="return_to_wait", target=wait_pose,
                                    move_kind="servo", settled=None, fault=None)
                            print("returning to wait pose (servo)...\n")
                            if caught_guess:
                                pending_dump = True  # fires once the shared arrival check below sees it get there
                                pending_dump_since = now_mono
                        else:
                            print()  # never left the wait pose; nothing to announce
                    elif attempted and not args.dry_run:
                        print("returning to wait pose...")
                        t0 = time.time()
                        settled, fault = move_to(wait_pose, args.approach_speed, args.approach_accel, rtde_r, dash, last_normal)
                        rec.log("move", purpose="return_to_wait", target=wait_pose,
                                speed=args.approach_speed, accel=args.approach_accel,
                                settled=settled, fault=fault, duration_s=time.time() - t0)
                        if fault is not None:
                            fault_count = wait_for_fault_clear(rec, fault, fault_count, rtde_r, dash, last_normal, wait_pose, args)
                            settled = True  # wait_for_fault_clear already drove to the wait pose once cleared
                        if not settled:
                            print("WARNING: did not settle at wait pose within timeout (no fault reported) - "
                                  "check the arm before the next throw.")
                        print("at wait pose.\n")
                        if caught_guess:
                            pending_dump = True  # already at (or near) the wait pose - the check below fires it next tick
                            pending_dump_since = now_mono
                    else:
                        print()

                # demo.py only: the last throw was a guessed catch and the arm has been
                # pointed back at the wait pose (servo: async re-aim above; movel/movej:
                # already blocked until settled). Wait for it to actually be there -
                # don't trust the "just sent the move" tick, since in servo mode that's
                # still wherever the arm was chasing the ball from - before running the
                # dump trick, so the base/wrist rotation starts from a known pose.
                if state == "idle" and pending_dump and not args.dry_run:
                    current_xyz = np.array(rtde_r.getActualTCPPose()[:3])
                    if float(np.linalg.norm(current_xyz - wait_xyz)) < DEMO_DUMP_ARRIVAL_TOL_M:
                        # Search for TARGET every tick while parked here, not just once on
                        # arrival - it's a physical prop someone may still be nudging into
                        # range, and the arm has nothing else to do at the wait pose anyway
                        # (arriving early is free, same as everywhere else in this codebase).
                        # 2026-08-25 fix: an earlier version gave up on the dump permanently
                        # the instant the first check saw TARGET out of range (e.g. a real
                        # session logged "target azimuth 81deg... refusing" and never tried
                        # again even though the prop was visibly still being moved closer).
                        # pending_dump now only clears once a dump is actually attempted;
                        # a new throw starting still preempts it unconditionally (above).
                        # Envelope-check the ACTUAL move target (xy from TARGET, z from the
                        # wait pose - see target_hover_pose(), z deliberately never comes
                        # from TARGET's own reading), not TARGET's raw position - CLAUDE.md's
                        # one reviewed clamp-off path is meant to see exactly what's about to
                        # be sent. No wait_xyz here, deliberately: unlike a live catch target,
                        # TARGET can sit at any azimuth (2026-08-25 user directive) - the
                        # rotate-then-extend move built into dump_ball_on_target() (via
                        # aim_pose) is what makes an arbitrary azimuth safe here, so the
                        # +-75deg CATCH_MAX_AZIMUTH_DEG band a live catch still enforces
                        # doesn't apply to this path. Reach/z still do.
                        target_xyz = find_target_pose(target_rb_state, R, t_vec)
                        hover_pose = None if target_xyz is None else target_hover_pose(target_xyz, wait_pose)
                        aim_pose = None if target_xyz is None else target_aim_pose(target_xyz, wait_pose, wait_xyz)
                        target_reason = None if hover_pose is None else check_catch_envelope(np.array(hover_pose[:3]))
                        if hover_pose is not None and target_reason is None:
                            pending_dump = False
                            pending_dump_since = None
                            dump_skip_logged = False
                            print(f"    TARGET seen at ({target_xyz[0]:+.3f},{target_xyz[1]:+.3f}) - "
                                  f"moving over it to dump...")
                            if servo_mode:
                                stop_servo_stream("dump")
                            rec.log("move", purpose="post_catch_dump_target", target=hover_pose, aim=aim_pose)
                            dumped = dump_ball_on_target(rec, rtde_r, dash, last_normal, aim_pose, hover_pose, wait_pose, args)
                            rec.log("move", purpose="post_catch_dump", settled=dumped)
                            if not dumped:
                                fault = check_safety_mode(rtde_r, dash, last_normal)
                                if fault is not None:
                                    fault_count = wait_for_fault_clear(rec, fault, fault_count, rtde_r, dash, last_normal, wait_pose, args)
                            if servo_mode:
                                set_servo_return_mode(True)
                                servo_target = wait_xyz.copy()
                                servo_orient = list(wait_pose[3:6])
                                # Same SystemExit trap as the post-fault-clear reconnect
                                # above, and for the same reason: this reconnect is most
                                # likely to fail exactly when something just went wrong
                                # (e.g. the E-STOP a dump-trick fault can prompt a bystander
                                # to hit) - letting it propagate uncaught skips the
                                # try/finally's wrap_up_session() call, silently losing the
                                # SSH log pull for the fault that matters most. This call
                                # site was missed by the 2026-07-27 fix (only the other one
                                # was wrapped) - found 2026-08-25 when it happened for real.
                                try:
                                    start_servo_stream()
                                except SystemExit as e:
                                    print(f"    !! could not re-open servo stream: {e}")
                                    print("    ending session here so the fault's logs can still be pulled.")
                                    rec.log("servo_stream", state="reopen_failed", reason=str(e))
                                    stop_reason = "servo_reconnect_failed"
                                    break
                            print("ball dumped, back at wait pose.\n")
                        else:
                            # No TARGET in range (not tracked, or fails the envelope - e.g.
                            # too far away) - 2026-08-25 user directive: don't dump anywhere
                            # in that case, just stay at the wait pose the normal
                            # return-to-wait already reached. No motion, no stream touched.
                            # pending_dump stays True: keep retrying every tick (throttled
                            # print only) so a TARGET moved into range a moment later still
                            # gets the dump, instead of this being a one-shot check.
                            reason = "not tracked" if target_xyz is None else target_reason
                            if not dump_skip_logged:
                                # Logged once per skip streak, not every tick - this wait is
                                # unbounded (only a new throw cancels it), unlike the bounded
                                # per-throw ticks the "log every tick" convention elsewhere
                                # assumes.
                                print(f"    TARGET {reason} - waiting at wait pose for it to "
                                      f"come into range (ball not dumped yet)...\n")
                                rec.log("move", purpose="post_catch_dump_skipped", reason=reason)
                                dump_skip_logged = True

                # Idle wobble (--idle-wobble): only while genuinely idle, servo mode is
                # up, and not (freshly) mid-dump-trick - pending_dump's arrival check
                # needs the arm to actually reach wait_xyz (within
                # DEMO_DUMP_ARRIVAL_TOL_M, 3cm), which a 15cm-diameter orbit would only
                # satisfy in passing. Every other idle tick (including the async
                # return-to-wait leg right after a throw) gets it - servo_target
                # already gets re-seeded to wait_xyz.copy() at each of those
                # transitions above, so adding the offset here every tick is the
                # entire mechanism; nothing upstream needs to know this exists. A
                # throw starting overwrites servo_target itself (state != "idle" at
                # that point), so the wobble stops being applied the same tick it
                # commits - no separate stop/interrupt logic needed.
                #
                # 2026-08-26 fix: pending_dump used to suppress wobble unconditionally
                # for as long as it stayed True - which the 2026-08-25 fix made
                # unbounded (retries forever, see that comment at pending_dump's
                # arrival check) so TARGET could be nudged into view late. A real
                # session where TARGET was never tracked at all (Motive's
                # never-sighted phantom pose - reads as a real but permanently
                # out-of-band position, not "not tracked") hit exactly that: every
                # catch set pending_dump and it never once cleared, so the wobble
                # never ran, all session. DUMP_SEARCH_WOBBLE_GRACE_S lets the wobble
                # resume after a few seconds of not finding TARGET without touching
                # the search itself (still unbounded, still every tick) - it just
                # shares the wait pose with an occasional, not guaranteed, search
                # attempt whenever a wobble lap happens to swing back near center.
                dump_wobble_ok = not pending_dump or (
                    pending_dump_since is not None
                    and now_mono - pending_dump_since >= DUMP_SEARCH_WOBBLE_GRACE_S)
                if args.idle_wobble and servo_mode and state == "idle" and dump_wobble_ok:
                    servo_target = wait_xyz + idle_wobble_offset(
                        now_mono, args.idle_wobble_diameter, args.idle_wobble_period)

                last_state = state
                # Last thing in the loop so the setpoint reflects the newest
                # decision. Every `continue` above calls it too - see emit_setpoint().
                emit_setpoint()
                time.sleep(1.0 / args.poll_hz)
        except KeyboardInterrupt:
            stop_reason = "keyboard_interrupt"
        except SystemExit as e:
            # Catch-all for a SystemExit escaping from somewhere in the loop that
            # wasn't given its own specific handler (like the two servo-reconnect
            # sites above) - e.g. a future one, or a helper's own safety-clamp
            # SystemExit (see ur_goto_raw.check_move_size). Never let this reach
            # bare `python3 demo.py` uncaught: same 2026-07-27/2026-08-25 bug
            # class either way - it skips straight past wrap_up_session() below,
            # losing the session folder for the exact run whose logs matter most.
            print(f"\n!!! uncaught SystemExit: {e}")
            stop_reason = f"crashed: {e}"
        except Exception as e:
            # Catch-all crash guard (2026-08-25 user directive: "make it so even on
            # not clean exit a new session appears in the folder"). Anything
            # unexpected here - a real bug, a dropped NatNet/RTDE connection, etc -
            # used to propagate straight out of main(), skipping wrap_up_session()
            # entirely below and leaving that session's name/description prompt and
            # (once re-enabled) robot log pull never run - exactly the session most
            # worth having notes on. Print the full traceback (this is NOT silent -
            # the bug still needs fixing) but fall through to the normal
            # finally+wrap_up_session path instead of dying.
            print(f"\n!!! unhandled exception in main loop - session ending abnormally:")
            traceback.print_exc()
            stop_reason = f"crashed: {type(e).__name__}: {e}"
        finally:
            print(f"\nstopping ({stop_reason or 'unknown'}) - sending stopl.")
            if attempts_ended:
                print(f"session tally: {catches}/{attempts_ended} attempted throws caught "
                      f"(heuristic - ball last seen <{CAUGHT_LAST_SEEN_DIST_M*100:.0f}cm from tool)")
            rec.log("run_end", reason=stop_reason or "unknown",
                    catches=catches, attempts_ended=attempts_ended,
                    servo_setpoints_sent=(stream.sent if stream is not None else None))
            # Stream down FIRST: stop() asks the robot to stopj and end its program,
            # and the stopl below is a fresh script on :30002 that would otherwise
            # preempt it mid-shutdown. Best-effort - if this raises, the robot's own
            # read-timeout watchdog reaches the same state within --sock-timeout.
            try:
                stop_servo_stream(stop_reason or "unknown")
            except Exception:
                pass
            if not args.dry_run:
                try:
                    send_script(stopl_script())
                except Exception:
                    pass
            # Each wrapped individually and best-effort (2026-08-25, same crash-
            # guard directive as above): one of these raising used to take out the
            # rest of teardown AND skip wrap_up_session() below with it - a
            # connection that's already dead (which is plausible exactly when the
            # loop above crashed) shouldn't cost the session folder too.
            for teardown_step in (client.stop_async, rtde_r.disconnect, dash.disconnect, rec.close):
                try:
                    teardown_step()
                except Exception as e:
                    print(f"  (teardown step {teardown_step.__qualname__} failed: {e})")
            if plot_window is not None:
                # Deliberately not closed here - left open (already showing the last
                # throw's plot, up to date as of its own update() call) so it's still
                # visible during wrap_up_session()'s name/description prompts below.
                # A non-blocking Tk window doesn't keep the process alive on its own
                # (nothing here ever called mainloop()), so it just disappears with
                # the rest of the process at exit - nothing to explicitly tear down.
                print("throw-plot window left open (showing the last throw).")

    # Deliberately outside the try/finally above and after the NatNet/RTDE
    # connections are fully closed - see wrap_up_session()'s docstring and the
    # module docstring's threading rule.
    wrap_up_session(session_start_wall, time.time(), rec.throw, record_path, args,
                    catches=catches, attempts_ended=attempts_ended)


if __name__ == "__main__":
    main()
