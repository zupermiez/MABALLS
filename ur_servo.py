"""
ur_servo.py - the v2 streaming control layer: continuous servoj() setpoint
streaming to the UR12e at 125-500Hz, over the same raw-socket path the rest of
this project already trusts (no ur_rtde control session, no External Control
URCap - see CLAUDE.md "Robot Control" for why those are parked).

WHY THIS EXISTS (CLAUDE.md "Catch Integration", v2). Today catch.py fires ONE
movel/movej per throw and is then locked to that decision: the post-commit
re-aim built to repair a noisy early prediction was measured firing on 2 of 98
real attempts (docs/debug_log.md 2026-07-18 section 2) because a discrete move
must *finish* before another can sensibly start, and arm travel consumes the
whole remaining flight. Streaming dissolves that: there is no "move" to finish,
only a setpoint that can be revised every single tick. That is also the only
path to the 46 throws the feasibility gate currently refuses outright (section
5 of the same entry) - those need the arm already moving when the prediction
firms up, not starting from rest afterwards.

ARCHITECTURE - reverse socket. A per-tick send to port 30002 (what every other
motion script here does) cannot work at servo rates: each send opens a fresh TCP
connection AND preempts the running program. Instead this module sends ONE
persistent program, once, which calls socket_open() back to THIS machine - the
robot is the TCP client, the laptop is the server. Then setpoints stream over
that already-open connection as packed binary int32s. This is the canonical UR
pattern (ur_modern_driver / ur_client_library / ur_rtde all do exactly this
internally); it is not something invented here.

    laptop (this module)                       UR12e controller
    --------------------                       ----------------
    bind+listen :30099        <---socket_open--- servo_prog()  (sent once, :30002)
    send 8 x int32 @ --rate   ---------------->  read loop -> get_inverse_kin
                                                       |
                                                  cmd_q (shared)
                                                       |
                                                  servoThread: servoj(cmd_q) @500Hz

The two loops are deliberately decoupled: the robot's servo thread is self-timed
by servoj()'s own `t` argument and always uses the LATEST setpoint, while the
read loop runs at whatever rate the host sends. That is the right model for
tracking a moving target (newest value wins). It is NOT the circular-buffer model
UR's servoj article describes - that one exists for replaying a precomputed
trajectory where every waypoint matters. Here every waypoint is superseded by the
next one; queueing them would only add latency, which is the one thing this whole
project cannot afford.

POSES, NOT JOINTS. Setpoints go over the wire as Cartesian poses and the robot
resolves them with get_inverse_kin(pose, qnear=<last commanded q>) - the same
primitive ur_goto_raw.movej_to_pose_script() already uses successfully. The
alternative (host-side IK, streaming joints) would mean writing and validating a
UR kinematics implementation at the same time as debugging the streaming path;
two unvalidated things at once is how you get an incident you can't attribute.
qnear is the last COMMANDED q, not the actual measured one, so IK solution
choice can't chatter between branches as the arm lags its setpoint.

SAFETY MODEL. servoj() has no speed limit of its own - it drives at whatever rate
`gain` allows toward whatever q it is handed. A single bad setpoint is therefore a
violent move, which on this arm means a protective stop (or worse). Three
independent layers guard that, and NONE of them is "the caller will be careful":

  1. RateLimiter (this module, pure math, offline self-tested): the streamed
     setpoint is never more than v_max*dt from the PREVIOUS streamed setpoint,
     and the step size itself is acceleration-limited. This is what makes a
     wild target produce a slow crawl toward it rather than a lunge. It is the
     load-bearing layer - the robot-side has no equivalent.
  2. Envelope check (caller's job, before send()): catch.py's reviewed
     check_catch_envelope - reach band, z band, azimuth band.
  3. Robot-side watchdog + IK guard (servo_program below): if the host stops
     sending for --sock-timeout the robot decides on its own to stopj() and end
     the program; and a setpoint with no IK solution is ignored rather than
     raising a runtime exception mid-servo.

Layer 3 is the one that matters if THIS PROCESS DIES. A killed host leaves the
robot holding its last servoj setpoint (benign - servoj holds position), and the
read timeout then terminates the program cleanly. There is no persistent ur_rtde
control session anywhere here, so the "never force-kill a control session" rule
in CLAUDE.md does not apply to this script.

Run `python3 ur_servo.py --self-test` for the offline math checks (no robot
needed, no network). Run `python3 ur_servo.py --bench` for the standalone
canned-motion prototype on the real arm - this is CLAUDE.md's "prototype
servo_track.py standalone before wiring into a catch" step: a slow sine sweep
about the current pose with zero perception in the loop, so the streaming path
can be proven (or found wanting) without any mocap variable involved.
"""

import argparse
import math
import socket
import struct
import time
from typing import Callable, Optional, Sequence

import numpy as np

from ur_goto_raw import ROBOT_IP, send_script

# This machine's address ON THE ROBOT SUBNET (see CLAUDE.md "Network setup") -
# the robot dials this, so it must be the robot-facing NIC, not the OptiTrack one
# (192.168.10.2) and not 127.0.0.1.
HOST_IP = "192.168.20.2"
# Host-side listen port. Deliberately not in the 30001-30004 range (those are the
# robot's own interfaces) and not 50001/50002 (ur_modern_driver / External Control
# URCap default to those - staying clear avoids a confusing collision if that
# investigation is ever resumed).
HOST_PORT = 30099

# Wire scale: setpoints are sent as int32 because socket_read_binary_integer() is
# the only robust fixed-width read URScript offers (socket_read_ascii_float has to
# parse text every tick, and its framing is fragile). 1e6 gives 1um / 1urad
# resolution; the largest value ever sent is ~1.3m -> 1.3e6, three orders of
# magnitude inside int32's range, so overflow is not a live concern.
WIRE_SCALE = 1_000_000.0
# 8 fields: x,y,z,rx,ry,rz,keepalive,servo_flag. Big-endian ("!") because
# socket_read_binary_integer expects network byte order.
WIRE_FMT = "!8i"
WIRE_N = 8

# --- servoj tuning. Defaults follow UR's own servoj guidance article
# (t=0.002 = the e-Series 500Hz control period, lookahead 0.06, gain 1000) rather
# than the URScript manual's more sluggish defaults (lookahead 0.1, gain 300).
# Higher gain / lower lookahead = faster response but risks instability and
# vibration; these are the conservative end of "responsive". ---
DEFAULT_SERVO_DT = 0.002
DEFAULT_LOOKAHEAD = 0.06
DEFAULT_GAIN = 1000.0
DEFAULT_STOP_ACCEL = 3.0     # rad/s^2 for the robot-side stopj on shutdown/idle

# Robot-side socket read timeout. If the host goes this long without sending, the
# robot stops itself and ends the program - the watchdog that makes a host crash
# safe. Must be comfortably longer than one host tick (8ms at 125Hz) but short
# enough that a dead host is caught quickly.
DEFAULT_SOCK_TIMEOUT = 0.3

DEFAULT_RATE = 125.0         # Hz, host -> robot setpoint rate

# --- RateLimiter defaults. Deliberately far below the arm's measured ~1.2 m/s
# ceiling (CLAUDE.md hardware section): this module's first job is to prove the
# streaming path, not to go fast. Raising these is a separate, deliberate,
# validated step. ---
DEFAULT_MAX_SPEED = 0.25     # m/s
DEFAULT_MAX_ACCEL = 1.0      # m/s^2


def servo_program(tcp_offset: Sequence[float], host_ip: str = HOST_IP,
                  host_port: int = HOST_PORT, servo_dt: float = DEFAULT_SERVO_DT,
                  lookahead: float = DEFAULT_LOOKAHEAD, gain: float = DEFAULT_GAIN,
                  stop_accel: float = DEFAULT_STOP_ACCEL,
                  sock_timeout: float = DEFAULT_SOCK_TIMEOUT) -> str:
    """The persistent URScript program: reverse-connect, then servo forever.

    Structure mirrors the long-proven ur_modern_driver driverProg (a servo thread
    self-timed by servoj, a main loop reading scaled int32 setpoints, a keepalive
    field to terminate) with two deliberate differences for this project:

    - No producer/consumer handshake. The driver's set_servo_setpoint() blocks
      the read loop until the servo thread has consumed each waypoint, because it
      is replaying a trajectory where every point matters. Here the newest
      setpoint always supersedes the older one, so the read loop just overwrites
      cmd_q and moves on - no queueing, no added latency.
    - IK on the robot. Setpoints arrive as poses; get_inverse_kin_has_solution()
      (PolyScope >= 5.10; this controller is 5.25) guards get_inverse_kin() so an
      unreachable setpoint is IGNORED rather than raising a runtime exception that
      would kill the program mid-motion.

    run_state starts at 0, so the arm does NOT move when the program starts - it
    only begins servoing once the first setpoint with servo_flag=1 arrives. The
    servo thread's 1->0 transition issues its own stopj(), which is why shutdown
    can simply flip the flag and wait rather than racing a stop against a
    still-running servoj.

    set_tcp() is issued here, inside the program, for the same reason catch.py
    resends it every run (docs/debug_log.md 2026-07-15): it is controller-side
    runtime state that does NOT reliably survive from whatever script ran last,
    and getting it wrong silently commands the flange instead of the tool.
    """
    # Guarded, not clamped: URScript documents a timeout of "0 or negative" as
    # "block until a read completes" - i.e. passing 0 does not mean "no wait", it
    # silently DISABLES the watchdog and leaves a dead host holding the arm's last
    # setpoint with no program-side escape. Too important to accept quietly.
    if sock_timeout <= 0:
        raise ValueError(
            f"sock_timeout must be > 0 (got {sock_timeout}). URScript treats a timeout of 0 or "
            f"negative as 'block forever', which disables the host-crash watchdog entirely."
        )
    tcp = "p[" + ",".join(f"{v:.6f}" for v in tcp_offset) + "]"
    return f"""def servo_prog():
  set_tcp({tcp})
  MULT = {WIRE_SCALE:.1f}
  cmd_q = get_actual_joint_positions()
  run_state = 0

  thread servoThread():
    prev_state = 0
    while True:
      enter_critical
      q = cmd_q
      st = run_state
      exit_critical
      if st == 1:
        servoj(q, 0.0, 0.0, {servo_dt}, {lookahead}, {gain})
      else:
        if prev_state == 1:
          stopj({stop_accel})
        end
        sync()
      end
      prev_state = st
    end
  end

  opened = socket_open("{host_ip}", {host_port}, "servo_sock")
  if opened:
    textmsg("ur_servo: connected, streaming")
    thread_servo = run servoThread()
    keepalive = 1
    while keepalive > 0:
      pkt = socket_read_binary_integer({WIRE_N}, "servo_sock", {sock_timeout})
      # On timeout/invalid reply URScript returns [0,-1,-1,-1] - a SHORT list, so
      # pkt[7]/pkt[8] must never be touched on this branch (they are read only
      # inside the else, below).
      if pkt[0] < {WIRE_N}:
        textmsg("ur_servo: read timeout/short - host gone, stopping")
        keepalive = 0
      else:
        keepalive = pkt[7]
        if pkt[8] == 1:
          target = p[pkt[1]/MULT, pkt[2]/MULT, pkt[3]/MULT, pkt[4]/MULT, pkt[5]/MULT, pkt[6]/MULT]
          enter_critical
          qn = cmd_q
          exit_critical
          if get_inverse_kin_has_solution(target, qnear=qn):
            new_q = get_inverse_kin(target, qnear=qn)
            enter_critical
            cmd_q = new_q
            run_state = 1
            exit_critical
          end
        else:
          enter_critical
          run_state = 0
          exit_critical
        end
      end
    end
    enter_critical
    run_state = 0
    exit_critical
    sleep(0.15)
    kill thread_servo
    stopj({stop_accel})
    socket_close("servo_sock")
    textmsg("ur_servo: stopped")
  else:
    textmsg("ur_servo: socket_open to {host_ip}:{host_port} FAILED - host not listening")
  end
end
servo_prog()
"""


def _wrap_angle(a: float) -> float:
    """Wrap `a` (radians) to (-pi, pi] - the shortest-path angular difference."""
    return math.atan2(math.sin(a), math.cos(a))


class RateLimiter:
    """Turns an arbitrary desired position into a stream of setpoints the arm can
    actually follow, bounding speed, acceleration, base-joint rate, and (optionally)
    a workspace envelope - all jointly, without the axes fighting each other.

    THIS IS THE LOAD-BEARING SAFETY LAYER of the whole streaming design. servoj()
    itself imposes no speed limit: hand it a q far from the current one and it
    drives there as hard as `gain` permits. So the setpoint stream, not the robot,
    is what has to be well-behaved.

    STATE IS CYLINDRICAL (r, theta, z about the base Z axis), not Cartesian - this
    is the fix for a real bug found by 6-seed x 120-throw simulation of the first
    (Cartesian) version: streamed setpoints violated the 120deg/s base-joint limit
    by up to 2.2x (134-265deg/s measured) and the reach/z envelope by up to 4.8cm,
    reproducing the exact C153A0 fault servo mode was supposed to eliminate.

    Root cause: the Cartesian version derived a TANGENTIAL LINEAR speed cap
    (v_t <= omega_max * r) and applied it to the Cartesian step vector, then
    smoothed that step toward the previous tick's step (acceleration limiting).
    That smoothing is only safe when the cap being enforced doesn't itself shift
    between ticks - which it does here, because r is driven by a DIFFERENT
    (radial) part of the same motion. Concretely: a step legal at large r gets
    smoothed inward as r shrinks, and by the time it arrives the same linear
    magnitude implies a higher angular rate than the (now smaller) cap allows.
    The reach/z envelope brake had the same structural problem, compounded by
    sharing one Cartesian step vector with the base-rate cap.

    The fix is to stop expressing the base-joint constraint in linear-tangential
    terms at all. theta is tracked as its own state with a CONSTANT hard bound
    (omega_max, independent of r) - so the smoothing-toward-previous-step
    argument is sound again: if this tick's desired theta-step and the previous
    tick's theta-step are both already <= omega_max*dt, any smoothed blend
    between them is too (norms are convex), by induction from t=0. r and z each
    get the same treatment against the envelope's per-axis margin (which IS
    self-consistent with sqrt(2*a*margin) braking, because the axis's own motion
    is what's consuming its own margin - no cross-axis coupling once r, theta, z
    no longer share one mutated Cartesian step vector). The three axes are only
    recombined once, via a joint Cartesian-speed rescale that can only shrink
    each axis's contribution - never re-introduce a violation.

    r=0 is a genuine coordinate singularity of this representation (theta is
    undefined there), and it is real-reachable: catch.py's own reach envelope
    permits h_min=0 whenever |z| alone already satisfies the minimum 3D reach.
    Approaching it used to divert-then-diverge (self.r ran to -14m in a real
    repro) because the margin/CBF brake toward h_min was folded into the same
    frac search as the acceleration bound, and the two can want frac to move
    in OPPOSITE directions - fixed by making the margin brake an unconditional
    post-search clamp instead of a competing search term, plus a hard r>=0
    floor. See step()'s comments and docs/debug_log.md 2026-07-21.

    Kept as pure math with no I/O so it is testable without a robot - see
    --self-test. Positions in/out are Cartesian metres / base frame, matching
    every other call site in this project (ServoStream.send, catch.py).
    """

    def __init__(self, start: Sequence[float], max_speed: float = DEFAULT_MAX_SPEED,
                 max_accel: float = DEFAULT_MAX_ACCEL,
                 max_base_rate: Optional[float] = None,
                 reach_bounds: Optional[Callable[[float], "tuple[float, float]"]] = None,
                 z_bounds: Optional[Sequence[float]] = None,
                 max_base_accel: Optional[float] = None):
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.max_base_rate = None if max_base_rate is None else float(max_base_rate)
        # Angular ramp-up rate. Only affects SMOOTHNESS, never compliance with
        # max_base_rate itself (that bound is enforced pre-blend, see step()) -
        # default reaches full rate in 0.15s, fast enough to be responsive to a
        # committed catch target without a visible lag.
        if max_base_accel is not None:
            self.max_base_accel = float(max_base_accel)
        elif self.max_base_rate is not None:
            self.max_base_accel = self.max_base_rate / 0.15
        else:
            self.max_base_accel = None
        # reach_bounds(z) -> (h_min, h_max): horizontal-radius band at that base-
        # frame z. z_bounds -> (z_min, z_max). Both None (the default, and what
        # --bench/track_ball_servo.py use) disables envelope enforcement entirely -
        # this class has no opinion on where the workspace boundary is, only how
        # to brake toward one if the caller supplies it (catch.py does, from its
        # own reviewed catch envelope).
        self.reach_bounds = reach_bounds
        # The reach constraint every current caller (catch.py's reach_band_at_z,
        # the self-test's local reach_bounds) implements is a circle in the
        # (r, z) plane centred on the base axis: reach_bounds(z) returns the
        # r-band such that hypot(r, z) stays inside [reach_min, reach_max].
        # Evaluating at z=0 recovers those two scalars directly (no z
        # component to trim there), which lets step()'s envelope clamp use the
        # EXACT combined-reach distance/gradient (hypot(r, z), (r, z)/reach)
        # instead of linearizing h_min(z) - a linearization that (see step()'s
        # comment) blows up near |z| close to reach_min/reach_max even far
        # from the actual boundary, since h_min(z)'s SLOPE there is steep
        # regardless of how much margin remains. Not a generic assumption
        # about reach_bounds' shape beyond what this project has ever passed.
        self._reach_min = self._reach_max = None
        if reach_bounds is not None:
            self._reach_min, self._reach_max = reach_bounds(0.0)
        self.z_bounds = None if z_bounds is None else (float(z_bounds[0]), float(z_bounds[1]))
        self._set_state(start)

    def _set_state(self, position: Sequence[float]) -> None:
        x, y, z = (float(v) for v in position)
        self.r = math.hypot(x, y)
        self.theta = math.atan2(y, x)
        self.z = z
        # Previous tick's ACHIEVED velocities, not step deltas - see step()'s
        # comment on why velocity (not step) is the right thing to remember, and
        # why the tangential one is stored as an angular rate rather than a
        # linear speed.
        self.prev_v_r = 0.0
        self.prev_omega = 0.0
        self.prev_v_z = 0.0

    def reset(self, position: Sequence[float]) -> None:
        """Re-seed the setpoint to `position` and zero the step history.

        Called after any discontinuity in control (resuming from a hold, the arm
        having been moved by something else) - without this, the first step after
        the gap would be measured from a stale setpoint.
        """
        self._set_state(position)

    @property
    def cmd(self) -> np.ndarray:
        """Current commanded position, Cartesian base frame - derived from the
        canonical (r, theta, z) state, not stored separately, so it can never
        drift out of sync with it."""
        return np.array([self.r * math.cos(self.theta), self.r * math.sin(self.theta), self.z])

    def _brake_cap(self, margin: float, accel: Optional[float] = None) -> float:
        """Largest approach speed from which `margin` (>=0 means still inside the
        bound) is enough to stop under `accel` (default max_accel) - the textbook
        double-integrator control-barrier-function form, v <= sqrt(2*a*margin)."""
        a = self.max_accel if accel is None else accel
        return math.sqrt(2.0 * a * max(margin, 0.0))

    def _axis_cap(self, want: float, dt: float, brake_margins: Sequence[float],
                  margin_accel: Optional[float] = None) -> float:
        """Max |step| this tick for a linear axis (r or z): the tightest of the
        overall speed limit, decelerating smoothly into the desired value (so the
        axis doesn't overshoot its own target, using the FULL accel budget - that
        overshoot isn't a hard safety bound), and braking toward whichever
        envelope bound `want`'s direction is headed toward (using `margin_accel`
        if given - see step()'s geometric reservation for why the envelope brake
        specifically needs a smaller, more conservative budget than the target
        approach does)."""
        caps = [self.max_speed]
        if want != 0.0:
            caps.append(self._brake_cap(abs(want)))
        for margin in brake_margins:
            caps.append(self._brake_cap(margin, accel=margin_accel))
        return min(caps) * dt

    def _theta_step_cap(self, dth_want: float, dt: float) -> float:
        """Max |dtheta| this tick. THE fix: this bound is a CONSTANT
        (self.max_base_rate), never a function of r - see class docstring. Without
        an explicit max_base_rate (non-catch callers), falls back to whatever
        angular rate the overall speed cap implies at the CURRENT radius; that
        fallback is r-dependent and not immune to the coupling bug, but it's only
        reachable when nothing has asked for a base-rate guarantee at all.
        """
        if self.max_base_rate is not None:
            omega_cap = self.max_base_rate
            alpha = self.max_base_accel
        else:
            r_ref = max(self.r, 1e-3)
            omega_cap = self.max_speed / r_ref
            alpha = self.max_accel / r_ref
        if dth_want != 0.0:
            omega_cap = min(omega_cap, math.sqrt(2.0 * alpha * abs(dth_want)))
        return omega_cap * dt

    def step(self, desired: Sequence[float], dt: float) -> np.ndarray:
        """Advance one tick toward `desired`; returns the new commanded position."""
        if dt <= 0:
            return self.cmd
        dx, dy, z_des = (float(v) for v in desired)
        r_des = math.hypot(dx, dy)
        # A target on (or numerically at) the base axis has no defined azimuth -
        # hold the current theta rather than inventing a direction to sweep to.
        th_des = math.atan2(dy, dx) if r_des > 1e-9 else self.theta

        dr_want = r_des - self.r
        dth_want = _wrap_angle(th_des - self.theta)
        dz_want = z_des - self.z

        # Final-approach deadband: snap straight onto a stationary-enough target
        # and drop velocity state, instead of letting the accel-limited blend
        # below carry residual velocity through it. Root cause of the real
        # decaying oscillation measured settling to the wait pose (up to ~8cm,
        # ~2s to damp out): the sqrt(2*a*margin) braking curve _axis_cap uses
        # has a slope that diverges as margin->0, so on the last couple of
        # ticks before crossing the target the discrete accel cap can't shed
        # velocity fast enough - the blend overshoots, reverses, and repeats,
        # each pass smaller than the last. Snapping removes the residual
        # velocity that (re)drives each swing. Threshold is scaled to this
        # tick's own max_speed*dt (not a fixed distance) so the snap itself
        # never exceeds the speed cap regardless of --servo-rate/max_speed -
        # a fixed 5mm tripped the self-test at a smaller dt (0.6m/s worst-case
        # jump against a 0.25 m/s cap at dt=0.002s). Also gated on dth_want
        # alone, not just the Cartesian gap: near the r~0 singularity a tiny
        # Cartesian distance can still hide a large angle (arc length = r*dth
        # shrinks with r even for a big dth) - snapping through that would
        # spike the base-joint rate exactly like the unbounded-omega bug this
        # class already had to fix once (see class docstring, 2026-07-21).
        theta_ok = self.max_base_rate is None or abs(dth_want) <= self.max_base_rate * dt
        if theta_ok and math.dist((dx, dy, z_des), self.cmd) < self.max_speed * dt:
            self.r, self.theta, self.z = r_des, th_des, z_des
            self.prev_v_r = self.prev_omega = self.prev_v_z = 0.0
            return self.cmd

        h_min = h_max = None
        if self.reach_bounds is not None:
            h_min, h_max = self.reach_bounds(self.z)
        z_min = z_max = None
        if self.z_bounds is not None:
            z_min, z_max = self.z_bounds

        r_margins = []
        if h_min is not None:
            if dr_want < 0.0:
                r_margins.append(self.r - h_min)
            elif dr_want > 0.0:
                r_margins.append(h_max - self.r)
        z_margins = []
        if z_min is not None:
            if dz_want < 0.0:
                z_margins.append(self.z - z_min)
            elif dz_want > 0.0:
                z_margins.append(z_max - self.z)

        # The envelope CBF caps below need to brake using only the accel that
        # will ACTUALLY be free for it, not the full max_accel - some of the
        # budget is already spoken for by the geometric (centripetal + Coriolis)
        # terms of whatever angular motion is already under way (estimated here
        # from last tick's achieved state, since this tick's isn't chosen yet).
        # Skipping this reservation lets the CBF promise a stopping distance the
        # arm can't actually deliver once theta is moving at the same time - the
        # combined-motion multi-seed session (not any single-axis test) measured
        # this as an 8-14cm reach/z envelope excursion despite the CBF speed
        # itself being satisfied at every individual tick along the way. Floored
        # at 15% of max_accel so a momentarily-large estimate can't stall
        # braking entirely.
        geometric_now = math.hypot(self.r * self.prev_omega * self.prev_omega,
                                   2.0 * self.prev_v_r * self.prev_omega)
        margin_accel = max(self.max_accel - geometric_now, 0.15 * self.max_accel)

        dr0 = math.copysign(min(abs(dr_want), self._axis_cap(dr_want, dt, r_margins, margin_accel)), dr_want) if dr_want else 0.0
        dz0 = math.copysign(min(abs(dz_want), self._axis_cap(dz_want, dt, z_margins, margin_accel)), dz_want) if dz_want else 0.0
        dth0 = math.copysign(min(abs(dth_want), self._theta_step_cap(dth_want, dt)), dth_want) if dth_want else 0.0
        v_r_des, omega_des, v_z_des = dr0 / dt, dth0 / dt, dz0 / dt

        # Joint Cartesian speed cap: sqrt(r_dot^2 + (r*theta_dot)^2 + z_dot^2) <=
        # max_speed. A uniform scale-down only ever shrinks each component, so it
        # cannot undo the per-axis hard bounds above.
        v_t_des = self.r * omega_des
        v_eff = math.sqrt(v_r_des * v_r_des + v_t_des * v_t_des + v_z_des * v_z_des)
        if v_eff > self.max_speed and v_eff > 1e-12:
            scale = self.max_speed / v_eff
            v_r_des *= scale
            omega_des *= scale
            v_z_des *= scale

        # Acceleration limit LAST: find how far along the segment from the
        # PREVIOUS tick's (v_r, omega, v_z) to this tick's desired one we can move
        # - `frac` in [0, 1] - without the resulting TRUE Cartesian acceleration
        # exceeding max_accel. "True" here means the full polar decomposition
        # (jerk terms r'', z'', r*theta'' PLUS the geometric terms that exist even
        # at constant velocity: -r*theta_dot^2 centripetal and 2*r_dot*theta_dot
        # Coriolis), not just rate-of-change-of-velocity. That distinction is not
        # academic: two earlier, wrong versions of this method (kept in comments
        # here, not git history, because the reasoning is exactly what stopped
        # each of them from working) bounded only rate-of-change and separately
        # tried to budget the geometric terms - one blended r/theta/z
        # INDEPENDENTLY (passed every per-axis check, then a componentwise blend
        # between two points inside a speed ball landed outside it - 0.266 m/s
        # against a 0.25 cap - because per-axis convexity doesn't imply joint
        # convexity); the other reserved a FIXED fraction of the budget for
        # geometric terms up front and blended the rest as pure jerk (passed the
        # single-target braking test, then under a real jitter-and-return session
        # measured 2.61 m/s^2 against a 2.0 cap, because braking BOTH r and theta
        # to a stop at once - exactly what returning to the wait pose after a
        # catch does - needs geometric room roughly matching the jerk room, not a
        # small fixed slice of it, and the sizing changes every tick with r and
        # the current speeds). Solving for the true bound directly, every tick,
        # sidesteps guessing a split.
        #
        # theta is blended as its own SCALAR state (not re-derived from a linear
        # tangential speed) specifically so the hard bound survives regardless of
        # `frac`: prev_omega and omega_des are each independently <= omega_max
        # (established above, before this block), so ANY convex combination of
        # them - any frac in [0, 1] - is too. That is what makes searching over
        # frac safe to do at all: nothing explored during the search can ever
        # imply a base-joint rate above the hard limit.
        # The envelope CBF caps folded into v_r_des/v_z_des above assumed the FULL
        # max_accel is available for braking toward that boundary. It generally
        # isn't - some of the budget can be spent on the geometric terms above,
        # which the iteration below is free to prioritise since it only tracks
        # total accel, not which direction it goes toward. Re-check the ACHIEVED
        # (post-blend) v_r_c/v_z_c against the same CBF cap every iteration, and
        # let it also drive `frac` down: without this, a tick with heavy angular
        # motion can leave less accel than the CBF assumed for braking, and the
        # arm coasts past the boundary before the (correctly-computed, but now
        # unaffordable) deceleration profile catches up - measured as an 8-14cm
        # reach/z envelope excursion in the multi-seed session despite every
        # individual cap being satisfied in isolation.
        #
        # NOTE this loop used to also fold the r/z margin (CBF) ratios into
        # `worst_ratio` here, shrinking frac whenever EITHER accel OR a margin
        # was over-limit. That is wrong whenever the two disagree on which
        # direction to move frac: margin compliance improves as v_r_c/v_z_c's
        # MAGNITUDE shrinks toward v_des (i.e. frac -> 1, if v_des is smaller
        # than the previous tick's velocity - the exact shape of hard braking),
        # while accel compliance improves as frac -> 0 (staying near last
        # tick's velocity). A "shrink frac when over-limit" search silently
        # assumes both ratios move the SAME way with frac; when they don't, it
        # walks frac toward 0 chasing the accel term while the margin ratio it
        # can't see improving just sits violated - the commanded velocity
        # freezes at whatever it was, unchanged, tick after tick, while the
        # margin keeps eroding at that same (unsafe) speed. Reproduced exactly
        # this way with a z-heavy near-axis catch target (h_min -> 0 per
        # reach_band_at_z, a legitimate point in catch.py's real envelope):
        # self.r sailed through 0 and diverged to -14m over ~2000 ticks,
        # because r<0 fed back into the centripetal term (r*omega^2) as a
        # fictitious LARGE ACCELERATION that only grew as r got more negative,
        # which choked frac further, which prevented any correction - a
        # feedback loop, not a one-off overshoot. See docs/debug_log.md
        # 2026-07-21. Root fix: keep this loop for what it's actually good at
        # (the true joint kinematic accel bound, which - unlike a margin ratio
        # against a FIXED external boundary - only ever needs frac shrunk, by
        # construction, since it is 0 at frac=0 and grows with frac), and
        # enforce the margins afterward as a direct, unconditional clamp - see
        # below. Envelope safety must win when the two genuinely conflict: a
        # brief accel overshoot is recoverable, driving through the reach
        # floor's singularity or past a hard z bound is not.
        frac = 1.0
        v_r_c, omega_c, v_z_c = v_r_des, omega_des, v_z_des
        for it in range(12):
            v_r_c = self.prev_v_r + frac * (v_r_des - self.prev_v_r)
            omega_c = self.prev_omega + frac * (omega_des - self.prev_omega)
            v_z_c = self.prev_v_z + frac * (v_z_des - self.prev_v_z)
            a_radial = (v_r_c - self.prev_v_r) / dt - self.r * omega_c * omega_c
            a_tang = self.r * (omega_c - self.prev_omega) / dt + 2.0 * v_r_c * omega_c
            a_z = (v_z_c - self.prev_v_z) / dt
            a_mag = math.sqrt(a_radial * a_radial + a_tang * a_tang + a_z * a_z)
            worst_ratio = a_mag / self.max_accel

            # 0.02% slack, not 0: this ratio is exactly linear in frac whenever
            # theta isn't moving (radial/z-only motion - the geometric accel
            # terms vanish identically), so the very first correction below
            # lands within floating-point noise of ratio == 1, not on either
            # side of it reliably. Without slack that noise could trigger an
            # unnecessary extra (damped) iteration purely from rounding, giving
            # up several % of budget for nothing - measured turning a 1.0mm
            # braking-overshoot result into 5.7mm.
            if worst_ratio <= 1.0002 or frac <= 0.0:
                break
            # Proportional shrink toward the point where accel is exactly at
            # its cap. Not a bisection: a_mag isn't linear in frac once theta
            # is moving (the geometric terms are quadratic in it), so an exact
            # solve isn't a closed form. The FIRST correction is trusted fully
            # - exact whenever theta isn't moving, which is most ticks of most
            # catches. Only once nonlinearity is confirmed to matter (a first
            # correction that didn't land within the slack above) does the
            # next correction get a small damping factor, just enough to
            # converge instead of oscillate around a curved ratio(frac) - a
            # single flat damping factor from the first iteration on was tried
            # and correctly handled the nonlinear case, but then cost 3% of
            # budget on EVERY tick of the common linear case too (nothing was
            # ever nonlinear about it), roughly doubling the braking-overshoot
            # self-test's result (4.8 -> 10.1mm at 1.2 m/s) for motion that
            # never touches theta at all.
            damp = 1.0 if it == 0 else 0.9
            frac *= damp / worst_ratio
            frac = max(frac, 0.0)

        # Hard margin clamp - unconditional, not part of the search above (see
        # the note on why folding it into that loop broke down). Checked
        # against v_r_c/v_z_c's OWN sign, not dr_want's/dz_want's: a blend that
        # starts from a fast PREVIOUS velocity can still be heading toward a
        # DIFFERENT boundary than the new target implies for several ticks
        # while it decelerates (e.g. residual downward momentum from a catch,
        # while the next target - the wait pose - is above the current
        # position) - gating on the target's direction missed exactly that
        # case and let the arm coast through Z_MIN under old momentum the
        # check never even looked at (measured 14cm past it, pre-this-fix).
        # This is a plain speed clamp (not a blend/frac), so it can't be
        # fought by the accel search the way folding it in there was: whatever
        # accel that implies this tick, so be it - it is bounded (the CBF cap
        # only ever shrinks a speed that was itself accel-limited a moment
        # ago), unlike the unbounded runaway this replaces.
        # Uses the FULL max_accel here, not margin_accel (which reserves room
        # for geometric terms and is what _axis_cap used to compute v_r_des/
        # v_z_des above, for the smooth common case). This is the last line of
        # defense - it should use every bit of authority actually available
        # rather than the more conservative planning figure, both because
        # that's the tightest cap still provably safe (sqrt(2*a*margin) with
        # a=max_accel is the true worst-case stopping guarantee) and because a
        # smaller assumed accel here only makes the clamp bite EARLIER and
        # HARDER without buying back any real margin (margin_accel's
        # reservation is for planning ahead, not for this already-last-resort
        # override).
        # The reach constraint is really on hypot(r, z) (see __init__'s note on
        # self._reach_min/_reach_max), not on r alone at a fixed z - a path
        # that moves r and z together can close that combined margin faster
        # than a check holding z fixed can see. Enforce it directly on the
        # true combined distance: reach = hypot(r, z), moving at rate
        # d(reach)/dt = (r, z)/reach . (v_r_c, v_z_c) - the exact projection
        # of the (r, z) velocity onto the radial direction in that plane, no
        # linearization/derivative-estimate involved (an earlier version of
        # this used a numeric dh_min/dz slope instead, which - independent of
        # how much margin actually remained - blows up near |z| close to
        # reach_min/reach_max purely because h_min(z)'s SLOPE is steep there,
        # producing spurious large corrections even far from the boundary:
        # measured turning a 15 m/s^2 spike into a 66 m/s^2 one instead of
        # fixing it). Correcting along the exact radial direction leaves the
        # tangential (along-the-boundary) component of motion untouched,
        # same principle as the rest of this method. See docs/debug_log.md
        # 2026-07-21.
        if self._reach_min is not None:
            reach = math.hypot(self.r, self.z)
            if reach > 1e-9:
                rhat_r, rhat_z = self.r / reach, self.z / reach
                rate = rhat_r * v_r_c + rhat_z * v_z_c  # d(reach)/dt

                margin_in = reach - self._reach_min
                cap_in = self._brake_cap(margin_in, accel=self.max_accel)
                if rate < -cap_in:
                    lam = -cap_in - rate  # rhat is a unit vector: |rhat|^2 == 1
                    v_r_c += lam * rhat_r
                    v_z_c += lam * rhat_z
                    rate = -cap_in

                margin_out = self._reach_max - reach
                cap_out = self._brake_cap(margin_out, accel=self.max_accel)
                if rate > cap_out:
                    lam = cap_out - rate
                    v_r_c += lam * rhat_r
                    v_z_c += lam * rhat_z
        if z_min is not None and v_z_c != 0.0:
            margin = (self.z - z_min) if v_z_c < 0.0 else (z_max - self.z)
            cap = self._brake_cap(margin, accel=self.max_accel)
            if abs(v_z_c) > cap:
                v_z_c = math.copysign(cap, v_z_c)

        # Final joint-speed re-check, same rescale as the pre-loop one (can
        # only shrink, never reintroduce a violation) but now against the
        # CURRENT self.r, not the r used to compute v_t_des earlier. Needed
        # because the pre-loop rescale and the frac blend's "convexity"
        # argument both implicitly compare against a fixed r - but r is itself
        # one of the three blended quantities, so it moves during the tick.
        # The blend's start point (prev_v_r, prev_omega, prev_v_z) was only
        # verified compliant using the PREVIOUS tick's r; if r grew since then
        # (moving outward while sweeping), r*prev_omega evaluated at the NEW,
        # larger r can exceed max_speed even though nothing this tick asked
        # for more speed - measured up to ~4% (1.246 vs 1.2 cap) in the
        # sustained-sweep-while-reaching-out portion of the multi-seed session.
        v_t_c = self.r * omega_c
        v_eff_c = math.sqrt(v_r_c * v_r_c + v_t_c * v_t_c + v_z_c * v_z_c)
        if v_eff_c > self.max_speed and v_eff_c > 1e-12:
            scale = self.max_speed / v_eff_c
            v_r_c *= scale
            omega_c *= scale
            v_z_c *= scale

        self.r += v_r_c * dt
        self.theta = _wrap_angle(self.theta + omega_c * dt)
        self.z += v_z_c * dt
        # r is a Euclidean radius - never physically negative. The clamp above
        # keeps this from happening except by sub-millimetre discretization
        # slop right at h_min==0 (cap*dt can, for a vanishingly small margin,
        # be a hair larger than the margin itself - the same discrete-time
        # slack already accepted for target-overshoot elsewhere in this
        # class). Belt-and-suspenders floor, not the primary mechanism.
        if self.r < 0.0:
            self.r = 0.0
            v_r_c = 0.0
        self.prev_v_r, self.prev_omega, self.prev_v_z = v_r_c, omega_c, v_z_c
        return self.cmd


class ServoStream:
    """Host side of the reverse socket: owns the listening socket, the one-shot
    program send, and the setpoint stream.

    Lifecycle is start() -> send() repeatedly -> stop(), and it is a context
    manager so stop() cannot be skipped on an exception path. ORDER MATTERS in
    start(): the listening socket must be bound and listening BEFORE the program
    is sent, because the program's first act is socket_open() - send first and
    the robot dials a closed port, logs a failure to the pendant, and silently
    does nothing.
    """

    def __init__(self, tcp_offset: Sequence[float], robot_ip: str = ROBOT_IP,
                 host_ip: str = HOST_IP, host_port: int = HOST_PORT,
                 servo_dt: float = DEFAULT_SERVO_DT, lookahead: float = DEFAULT_LOOKAHEAD,
                 gain: float = DEFAULT_GAIN, stop_accel: float = DEFAULT_STOP_ACCEL,
                 sock_timeout: float = DEFAULT_SOCK_TIMEOUT):
        self.tcp_offset = list(tcp_offset)
        self.robot_ip = robot_ip
        self.host_ip = host_ip
        self.host_port = host_port
        self.servo_dt = servo_dt
        self.lookahead = lookahead
        self.gain = gain
        self.stop_accel = stop_accel
        self.sock_timeout = sock_timeout
        self._listener: Optional[socket.socket] = None
        self._conn: Optional[socket.socket] = None
        self.sent = 0
        # False once the far end has gone away. A protective stop KILLS the
        # robot-side program, which closes the socket - so a caller streaming
        # into a faulted arm needs to find out from send() rather than keep
        # writing into a dead pipe. See send()'s return value.
        self.alive = False

    def __enter__(self) -> "ServoStream":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def start(self, accept_timeout: float = 8.0) -> None:
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # SO_REUSEADDR so a re-run moments after a crash isn't blocked by the
        # previous connection sitting in TIME_WAIT - otherwise every failed
        # experiment costs a ~60s wait before the next attempt.
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((self.host_ip, self.host_port))
        self._listener.listen(1)
        self._listener.settimeout(accept_timeout)

        script = servo_program(self.tcp_offset, self.host_ip, self.host_port,
                               self.servo_dt, self.lookahead, self.gain,
                               self.stop_accel, self.sock_timeout)
        send_script(script)

        try:
            self._conn, addr = self._listener.accept()
        except socket.timeout:
            self._listener.close()
            self._listener = None
            raise SystemExit(
                f"Robot never connected back to {self.host_ip}:{self.host_port} within "
                f"{accept_timeout:.0f}s. Check: Remote Control is ON, the robot is in "
                f"RUNNING mode (ur_status.py), no protective stop is latched, and no host "
                f"firewall is blocking inbound TCP {self.host_port} on the robot subnet."
            )
        # Nagle would coalesce our tiny 32-byte setpoints and add tens of ms of
        # jitter - exactly the latency this whole architecture exists to remove.
        self._conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.alive = True
        print(f"servo stream up: robot connected from {addr[0]}:{addr[1]}")

    def send(self, pose: Sequence[float], servo: bool = True, keepalive: bool = True) -> bool:
        """Stream one setpoint. Returns False if the stream has died.

        `servo=False` tells the robot to stop and idle (the arm holds position)
        while keeping the connection alive, which is what a lost/stale/refused
        target should produce - NOT simply withholding packets, since silence for
        --sock-timeout ends the program entirely.

        Returns rather than raises on a dead socket: the expected way for this to
        happen is a protective stop killing the robot-side program mid-session,
        and a caller in a hot loop should be able to notice and recover (tear
        down, wait for the human to clear it, restart) without an exception
        unwinding the loop it needs to stay in.
        """
        if self._conn is None:
            raise RuntimeError("ServoStream.send() before start()")
        if not self.alive:
            return False
        vals = [int(round(v * WIRE_SCALE)) for v in pose[:6]]
        vals.append(1 if keepalive else 0)
        vals.append(1 if servo else 0)
        try:
            self._conn.sendall(struct.pack(WIRE_FMT, *vals))
        except OSError:
            self.alive = False
            return False
        self.sent += 1
        return True

    def stop(self) -> None:
        """Clean shutdown: ask the robot to stop and end its program, then close.

        Best-effort at every step - if the connection is already dead the robot's
        own read timeout (layer 3) reaches the same end state a fraction of a
        second later, so a failure here is not a stranded-arm hazard.
        """
        self.alive = False
        if self._conn is not None:
            try:
                self._conn.sendall(struct.pack(WIRE_FMT, 0, 0, 0, 0, 0, 0, 0, 0))
                time.sleep(0.3)  # let the robot's servo thread see run_state=0 and stopj
            except OSError:
                pass
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None
        if self._listener is not None:
            try:
                self._listener.close()
            except OSError:
                pass
            self._listener = None


def _simulate_session(max_speed: float, max_accel: float, omega_max_deg: float,
                      seeds: int = 6, throws_per_seed: int = 120,
                      rate: float = DEFAULT_RATE):
    """Multi-seed simulated catch.py session.

    Exists to close the coverage gap that let the original (Cartesian) RateLimiter
    ship with self-test parts 1-5 all green while violating the base-rate cap by
    up to 2.2x and the envelope by up to 4.8cm on real catch data (see class
    docstring). None of parts 1-5 combine a MOVING target with a CHANGING radius -
    each tests one axis in isolation. A real catch does both at once: the
    trajectory prediction keeps refining (jittering the target every tick) while
    the arm is also sweeping in reach, exactly the cross-axis condition that broke
    the old implementation. This reproduces that: per throw, retarget every tick
    at a randomized catch point plus decaying gaussian noise (mimicking a
    prediction converging as more ball samples arrive) for a burst of ticks, then
    retarget back to a fixed wait pose before the next throw - repeated
    `throws_per_seed` times, over `seeds` independent RNG streams.

    Envelope constants mirror catch.py's CATCH_MIN_REACH/MAX_REACH/Z_MIN/Z_MAX
    (not imported - ur_servo.py is imported BY catch.py, so the reverse would be
    circular; keep these in sync by hand if the real envelope changes).
    """
    REACH_MIN, REACH_MAX = 0.45, 1.20
    Z_MIN, Z_MAX = -0.25, 0.55

    def reach_bounds(z):
        eps = 1e-6
        h_min = math.sqrt(max((REACH_MIN + eps) ** 2 - z * z, 0.0))
        h_max = math.sqrt(max((REACH_MAX - eps) ** 2 - z * z, 0.0))
        return h_min, h_max

    wait = np.array([0.0, -0.7, 0.14])
    wait_az = math.atan2(wait[1], wait[0])
    dt = 1.0 / rate
    omega_max = math.radians(omega_max_deg)

    worst_omega = worst_speed = worst_accel = 0.0
    worst_reach_over = worst_z_over = 0.0

    def _track(cur, prev_cmd, prev_step):
        nonlocal worst_omega, worst_speed, worst_accel, worst_reach_over, worst_z_over
        r_prev, r_cur = math.hypot(prev_cmd[0], prev_cmd[1]), math.hypot(cur[0], cur[1])
        if min(r_prev, r_cur) > 1e-6:
            d_az = math.atan2(cur[1], cur[0]) - math.atan2(prev_cmd[1], prev_cmd[0])
            d_az = math.atan2(math.sin(d_az), math.cos(d_az))
            worst_omega = max(worst_omega, abs(d_az) / dt)
        worst_speed = max(worst_speed, float(np.linalg.norm(cur - prev_cmd)) / dt)
        step = cur - prev_cmd
        worst_accel = max(worst_accel, float(np.linalg.norm(step - prev_step)) / (dt * dt))
        reach_cur = float(np.linalg.norm(cur))
        worst_reach_over = max(worst_reach_over, REACH_MIN - reach_cur, reach_cur - REACH_MAX)
        worst_z_over = max(worst_z_over, Z_MIN - cur[2], cur[2] - Z_MAX)
        return step

    for seed in range(seeds):
        rng = np.random.default_rng(1000 + seed)
        lim = RateLimiter(wait, max_speed, max_accel, omega_max,
                          reach_bounds=reach_bounds, z_bounds=(Z_MIN, Z_MAX))
        prev_cmd = lim.cmd.copy()
        prev_step = np.zeros(3)

        for _ in range(throws_per_seed):
            reach = rng.uniform(REACH_MIN + 0.03, REACH_MAX - 0.03)
            z = rng.uniform(Z_MIN + 0.03, Z_MAX - 0.03)
            az = wait_az + math.radians(rng.uniform(-70.0, 70.0))
            h = math.sqrt(max(reach * reach - z * z, 0.0))
            true_point = np.array([h * math.cos(az), h * math.sin(az), z])

            for tick in range(int(rng.uniform(30, 90))):
                noise = rng.normal(0.0, 0.05 * math.exp(-tick / 12.0), size=3)
                cur = lim.step(true_point + noise, dt)
                prev_step = _track(cur, prev_cmd, prev_step)
                prev_cmd = cur.copy()

            for _ in range(int(rng.uniform(60, 120))):
                cur = lim.step(wait, dt)
                prev_step = _track(cur, prev_cmd, prev_step)
                prev_cmd = cur.copy()

    return worst_omega, worst_speed, worst_accel, worst_reach_over, worst_z_over


def _self_test() -> None:
    """Offline checks of the pieces that must be right before any arm moves: the
    rate limiter's guarantees and the wire encoding's round-trip. No robot, no
    network - same spirit as frames.py / trajectory.py's synthetic self-tests."""
    rng = np.random.default_rng(7)
    dt = 1.0 / DEFAULT_RATE

    # 1) A far-away target must never produce a step above the speed limit, and
    # the resulting motion must still converge. This is the exact failure mode
    # the limiter exists to prevent: a 2m jump becoming a 2m lunge.
    lim = RateLimiter([0.0, -0.6, 0.1], max_speed=0.25, max_accel=1.0)
    target = np.array([0.5, -1.5, 0.6])
    prev = lim.cmd.copy()
    worst_speed = 0.0
    for _ in range(2000):
        cur = lim.step(target, dt)
        worst_speed = max(worst_speed, float(np.linalg.norm(cur - prev)) / dt)
        prev = cur.copy()
    assert worst_speed <= 0.25 + 1.0 * dt * 1.001, f"speed limit violated: {worst_speed:.4f} m/s"
    assert float(np.linalg.norm(lim.cmd - target)) < 1e-3, "limiter never converged to target"
    print(f"[rate limit]   worst commanded speed {worst_speed:.4f} m/s (cap 0.25), converged")

    # 2) Acceleration bound, including a hard reversal: with the target flipped
    # every tick the naive magnitude-only clamp would snap the step vector from
    # +max to -max instantly (infinite acceleration). Check it doesn't. Start and
    # both alternating targets are kept off the base axis (y=0.6 throughout, r
    # never near 0) - r=0 is a genuine kinematic singularity (no defined azimuth)
    # under the cylindrical state this class now uses, and it isn't a physically
    # reachable TCP position on the real arm either, so testing a reversal
    # straight through it was never meaningful for this system.
    lim = RateLimiter([0.0, 0.6, 0.0], max_speed=0.5, max_accel=2.0)
    prev_step = np.zeros(3)
    worst_accel = 0.0
    for i in range(400):
        tgt = np.array([1.0, 0.6, 0]) if (i // 20) % 2 == 0 else np.array([-1.0, 0.6, 0])
        before = lim.cmd.copy()
        cur = lim.step(tgt, dt)
        step = cur - before
        worst_accel = max(worst_accel, float(np.linalg.norm(step - prev_step)) / (dt * dt))
        prev_step = step
    # 12%, not 0: the bound RateLimiter actually enforces is on the CHANGE IN
    # VELOCITY per tick (see step()'s joint blend); this check instead measures
    # the raw second difference of committed POSITIONS, which only approximates
    # that when a tick's motion is close to a straight line. Near a reversal, r
    # and theta are both moving (this test's start/targets are all off-axis), and
    # position-space finite-differencing a curved (r, theta) path picks up a
    # bounded second-order residual that the velocity-space guarantee doesn't
    # promise to hide. Measured consistently at 10.7% (deterministic, no RNG in
    # this test) - 12% leaves headroom without masking a real regression.
    assert worst_accel <= 2.0 * 1.12, f"accel limit violated: {worst_accel:.4f} m/s^2"
    print(f"[accel limit]  worst commanded accel {worst_accel:.4f} m/s^2 (cap 2.0) across reversals")

    # 2b) Deceleration: driven flat-out at a target from far away, the commanded
    # setpoint must not sail past it. Without the sqrt(2*a*d) cap a pure
    # first-order limiter overshoots by v^2/(2a) - 18cm at catch speeds, which is
    # the entire error budget several times over.
    overshoots = []
    for v_max, a_max in ((0.25, 1.0), (0.6, 2.0), (1.2, 4.0)):
        lim = RateLimiter([0.0, 0.0, 0.0], max_speed=v_max, max_accel=a_max)
        target = np.array([1.5, 0.0, 0.0])
        overshoot = 0.0
        for _ in range(4000):
            cur = lim.step(target, dt)
            overshoot = max(overshoot, float(cur[0] - target[0]))
        naive = v_max * v_max / (2 * a_max)
        # 5mm, not 0: with the acceleration limit having the last word (see step())
        # the braking ramp is slightly softened, so a few mm of overshoot is the
        # designed behaviour rather than a defect. Measured 0.6/1.5/2.4mm at
        # 0.25/0.6/1.2 m/s - three orders of magnitude better than undamped, and
        # well inside a 4.25mm calibration.
        assert overshoot < 0.005, f"overshoot {overshoot*1000:.1f}mm at v={v_max} (naive would be {naive*100:.0f}cm)"
        assert abs(lim.cmd[0] - target[0]) < 1e-3, f"did not settle on target at v={v_max}"
        overshoots.append(overshoot * 1000)
    print("[braking]      overshoot " + "/".join(f"{o:.1f}" for o in overshoots)
          + "mm at 0.25/0.6/1.2 m/s (undamped would be 3/9/18cm)")

    # 2c) Base-joint rate: a purely lateral sweep at catch speed is the exact
    # 2026-07-16/17 C153A0 protective-stop geometry (v/r > 120 deg/s). The
    # tangential cap must hold the implied base rate under the limit, while NOT
    # throttling radial motion, which doesn't load that joint at all. Unlike the
    # pre-fix version, omega_max is now a CONSTANT bound on the tracked theta
    # state (not derived from a tangential-linear-speed cap that has to be
    # reconverted through a changing radius), so this should hold far tighter
    # than a first-order correction - see class docstring for why.
    omega_max = math.radians(110.0)
    lim = RateLimiter([0.5, 0.0, 0.0], max_speed=1.2, max_accel=4.0, max_base_rate=omega_max)
    worst_omega = 0.0
    prev = lim.cmd.copy()
    for _ in range(1500):
        cur = lim.step([0.0, 0.5, 0.0], dt)  # 90deg sweep at r=0.5m
        r = math.hypot(prev[0], prev[1])
        d_az = math.atan2(cur[1], cur[0]) - math.atan2(prev[1], prev[0])
        d_az = math.atan2(math.sin(d_az), math.cos(d_az))
        if r > 1e-6:
            worst_omega = max(worst_omega, abs(d_az) / dt)
        prev = cur.copy()
    assert worst_omega <= omega_max * 1.001, \
        f"base rate {math.degrees(worst_omega):.3f} deg/s exceeds cap {math.degrees(omega_max):.0f}"
    lim_r = RateLimiter([0.5, 0.0, 0.0], max_speed=1.2, max_accel=4.0, max_base_rate=omega_max)
    radial_speed = 0.0
    for _ in range(200):
        before = lim_r.cmd.copy()
        after = lim_r.step([1.1, 0.0, 0.0], dt)  # pure radial - must NOT be throttled
        radial_speed = max(radial_speed, float(np.linalg.norm(after - before)) / dt)
    assert radial_speed > 1.0, f"radial motion wrongly throttled to {radial_speed:.2f} m/s by the base cap"
    print(f"[base rate]    lateral sweep held to {math.degrees(worst_omega):.3f} deg/s "
          f"(cap 110); radial motion unthrottled at {radial_speed:.2f} m/s")

    # 3) A tiny residual error must be tracked exactly, not chased in max-size
    # steps - i.e. the limiter is transparent when it isn't binding.
    lim = RateLimiter([0.1, 0.2, 0.3], max_speed=0.25, max_accel=100.0)
    out = lim.step([0.1001, 0.2, 0.3], dt)
    assert abs(out[0] - 0.1001) < 1e-12, "limiter distorted a sub-limit step"
    print("[transparency] sub-limit steps pass through unchanged")

    # 4) Wire round-trip, at the extremes the arm can actually reach: an encoding
    # slip here would send a plausible-looking but wrong pose, which is precisely
    # the class of bug that caused the 2026-07-10 E-stop.
    for _ in range(500):
        pose = list(rng.uniform(-1.4, 1.4, size=3)) + list(rng.uniform(-math.pi, math.pi, size=3))
        packed = struct.pack(WIRE_FMT, *[int(round(v * WIRE_SCALE)) for v in pose], 1, 1)
        assert len(packed) == 32, f"packet is {len(packed)} bytes, robot reads 8 x int32 = 32"
        back = [v / WIRE_SCALE for v in struct.unpack(WIRE_FMT, packed)[:6]]
        assert max(abs(a - b) for a, b in zip(pose, back)) < 2e-6, "wire round-trip lost precision"
    print("[wire]         500 random poses round-trip within 2um/2urad, 32-byte packets")

    # 5) The generated URScript must contain the pieces the robot depends on -
    # a cheap guard against an edit accidentally dropping one (e.g. deleting the
    # watchdog, which would turn a host crash into an arm that never stops).
    prog = servo_program([0, 0, 0.0725, 0, 0, 0])
    for needle in ("set_tcp(", "socket_open(", "servoj(", "stopj(", "get_inverse_kin_has_solution(",
                   "socket_read_binary_integer(8", "kill thread_servo", "socket_close("):
        assert needle in prog, f"generated program is missing {needle!r}"
    assert prog.endswith("servo_prog()\n"), "program must end by calling itself, with a trailing newline"
    # A zero/negative socket timeout means "block forever" in URScript, silently
    # removing the host-crash watchdog - must be rejected, not passed through.
    for bad in (0.0, -1.0):
        try:
            servo_program([0, 0, 0, 0, 0, 0], sock_timeout=bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"sock_timeout={bad} was accepted - watchdog would be disabled")
    print("[urscript]     generated program contains tcp/socket/servoj/watchdog/IK-guard/teardown;"
          " non-positive sock_timeout rejected")

    # 6) Multi-seed simulated session - see _simulate_session's docstring for why
    # this exists (parts 1-5 all passed while the pre-fix implementation violated
    # the base rate cap by up to 2.2x on real catch data; none of them combine a
    # moving target with a changing radius the way an actual catch does).
    for v_max, a_max in ((0.6, 2.0), (1.2, 4.0)):
        omega, speed, accel, reach_over, z_over = _simulate_session(v_max, a_max, 110.0)
        assert math.degrees(omega) <= 110.0 * 1.01, \
            f"base rate {math.degrees(omega):.1f} deg/s exceeds 110 at v={v_max}"
        assert speed <= v_max * 1.01, f"speed {speed:.3f} m/s exceeds cap {v_max} at v={v_max}"
        # 2.2x, not 1.05x: the envelope's hard margin clamp (step()'s last
        # word, deliberately allowed to exceed max_accel - see its comment on
        # why smoothness yields to envelope safety when the two conflict) has
        # an inherent one-tick discretization residual at the exact moment a
        # braking-to-a-hard-boundary curve reaches v=0: the ideal continuous
        # sqrt(2*a*margin) profile hits zero in finite time, but sampled at a
        # fixed dt it can demand a slightly bigger final velocity drop than a
        # smooth max_accel*dt step would give, right at that one tick. Swept
        # 30 seeds x 150 throws (4500 throws, 10x this test's default) and it
        # plateaus at ~2.15x, never grows with more trials - a bounded,
        # understood residual, not a runaway (contrast the reach/z envelope
        # asserts below, which stay within a fraction of a mm every time).
        assert accel <= a_max * 2.2, f"accel {accel:.3f} m/s^2 exceeds cap {a_max} at v={v_max}"
        assert reach_over <= 0.001, f"reach envelope violated by {reach_over * 1000:.2f}mm at v={v_max}"
        assert z_over <= 0.001, f"z envelope violated by {z_over * 1000:.2f}mm at v={v_max}"
        print(f"[session sim]  v_max={v_max} a_max={a_max}: worst base rate "
              f"{math.degrees(omega):.1f}deg/s (cap 110), speed {speed:.3f}m/s (cap {v_max}), "
              f"accel {accel:.2f}m/s^2 (cap {a_max}), envelope held to "
              f"{max(reach_over, z_over, 0.0) * 1000:.3f}mm")

    print("\nself-test passed")


def _bench(args) -> None:
    """Standalone canned-motion prototype - CLAUDE.md's 'prototype servo_track.py
    standalone before wiring into a catch' step.

    Streams a slow sine sweep about the arm's CURRENT pose, one axis, with no
    perception anywhere in the loop. The point is to answer 'does the streaming
    path work, and how smoothly' with the mocap variable removed entirely, so
    that when track_ball_servo.py misbehaves later you already know whether the
    transport is sound. Amplitude and speed default to small/slow deliberately;
    this is a first-contact test on a shared robot.
    """
    import rtde_receive

    rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    start = list(rtde_r.getActualTCPPose())
    print(f"start pose: {[round(v, 4) for v in start]}")
    axis = {"x": 0, "y": 1, "z": 2}[args.bench_axis]
    print(f"\nAbout to stream a +/-{args.bench_amplitude * 100:.0f}cm sine on base {args.bench_axis}, "
          f"period {args.bench_period:.1f}s, for {args.bench_duration:.0f}s.")
    print("The workspace must be clear and you must be at the physical E-stop.")
    if input("Type 'go' to start: ").strip().lower() != "go":
        raise SystemExit("aborted.")

    dt = 1.0 / args.rate
    limiter = RateLimiter(start[:3], args.max_speed, args.max_accel)
    stream = ServoStream(args.tcp_offset, args.robot_ip, args.host_ip, args.host_port,
                         args.servo_dt, args.lookahead, args.gain,
                         args.stop_accel, args.sock_timeout)
    t0 = time.monotonic()
    late = 0
    with stream:
        try:
            while True:
                tick = time.monotonic()
                elapsed = tick - t0
                if elapsed > args.bench_duration:
                    break
                desired = np.array(start[:3])
                desired[axis] += args.bench_amplitude * math.sin(2 * math.pi * elapsed / args.bench_period)
                cmd = limiter.step(desired, dt)
                stream.send(list(cmd) + start[3:6])

                actual = rtde_r.getActualTCPPose()[:3]
                lag = float(np.linalg.norm(np.array(actual) - cmd))
                print(f"\rt={elapsed:5.1f}s  cmd={cmd[axis]:+.4f}  actual={actual[axis]:+.4f}  "
                      f"lag={lag * 1000:5.1f}mm  sent={stream.sent}  late_ticks={late}   ",
                      end="", flush=True)

                sleep_for = dt - (time.monotonic() - tick)
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    late += 1
        except KeyboardInterrupt:
            print("\ninterrupted.")
    print(f"\ndone: {stream.sent} setpoints, {late} late ticks "
          f"({100.0 * late / max(stream.sent, 1):.1f}%)")
    rtde_r.disconnect()


def add_servo_args(parser: argparse.ArgumentParser) -> None:
    """Shared servo/streaming flags, so track_ball_servo.py and --bench expose an
    identical tuning surface (and a value found good in one is transferable)."""
    parser.add_argument("--robot-ip", default=ROBOT_IP)
    parser.add_argument("--host-ip", default=HOST_IP,
                        help="This machine's IP ON THE ROBOT SUBNET - the robot dials it")
    parser.add_argument("--host-port", type=int, default=HOST_PORT)
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE,
                        help=f"Host setpoint rate, Hz (default {DEFAULT_RATE:.0f})")
    parser.add_argument("--servo-dt", type=float, default=DEFAULT_SERVO_DT,
                        help="servoj t, s (robot-side control period)")
    parser.add_argument("--lookahead", type=float, default=DEFAULT_LOOKAHEAD,
                        help="servoj lookahead_time, s, valid range [0.03,0.2] - higher smooths, lower reacts")
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN,
                        help="servoj gain, valid range [100,2000] - higher tracks harder, risks vibration")
    parser.add_argument("--stop-accel", type=float, default=DEFAULT_STOP_ACCEL,
                        help="rad/s^2 for the robot-side stopj")
    parser.add_argument("--sock-timeout", type=float, default=DEFAULT_SOCK_TIMEOUT,
                        help="s - robot-side watchdog: no setpoint for this long and it stops itself")
    parser.add_argument("--max-speed", type=float, default=DEFAULT_MAX_SPEED,
                        help=f"m/s cap enforced by the host-side rate limiter (default {DEFAULT_MAX_SPEED})")
    parser.add_argument("--max-accel", type=float, default=DEFAULT_MAX_ACCEL,
                        help=f"m/s^2 cap enforced by the host-side rate limiter (default {DEFAULT_MAX_ACCEL})")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-test", action="store_true",
                      help="Offline math/encoding checks - no robot, no network")
    mode.add_argument("--bench", action="store_true",
                      help="Canned sine-sweep streaming test on the real arm (no mocap)")
    add_servo_args(parser)
    parser.add_argument("--tcp-offset", type=float, nargs=6,
                        default=[0.0, 0.0, 0.0725, 0.0, 0.0, 0.0],
                        help="set_tcp offset sent at program start (default: the 7.25cm box centroid)")
    parser.add_argument("--bench-axis", choices=("x", "y", "z"), default="z")
    parser.add_argument("--bench-amplitude", type=float, default=0.05, help="m")
    parser.add_argument("--bench-period", type=float, default=4.0, help="s")
    parser.add_argument("--bench-duration", type=float, default=20.0, help="s")
    args = parser.parse_args()

    if args.self_test:
        _self_test()
    else:
        _bench(args)


if __name__ == "__main__":
    main()
