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
from typing import Optional, Sequence

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


class RateLimiter:
    """Turns an arbitrary desired position into a stream of setpoints the arm can
    actually follow, bounding both speed and acceleration.

    THIS IS THE LOAD-BEARING SAFETY LAYER of the whole streaming design. servoj()
    itself imposes no speed limit: hand it a q far from the current one and it
    drives there as hard as `gain` permits. So the setpoint stream, not the robot,
    is what has to be well-behaved - and the only way to guarantee that is to make
    each setpoint a bounded step from the PREVIOUS SETPOINT (never from the arm's
    measured position, which would let error accumulate into a lunge whenever the
    arm lags).

    Speed is bounded by construction: |step| <= max_speed*dt. Acceleration is
    bounded by limiting how much the step VECTOR may change between ticks
    (|step_k - step_{k-1}| <= max_accel*dt^2), which also removes the direction-
    reversal snap you get from clamping magnitude alone.

    Kept as pure math with no I/O so it is testable without a robot - see
    --self-test. Everything here is in metres / base frame.
    """

    def __init__(self, start: Sequence[float], max_speed: float = DEFAULT_MAX_SPEED,
                 max_accel: float = DEFAULT_MAX_ACCEL):
        self.cmd = np.asarray(start, dtype=float).copy()
        self.prev_step = np.zeros(3)
        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)

    def reset(self, position: Sequence[float]) -> None:
        """Re-seed the setpoint to `position` and zero the step history.

        Called after any discontinuity in control (resuming from a hold, the arm
        having been moved by something else) - without this, the first step after
        the gap would be measured from a stale setpoint.
        """
        self.cmd = np.asarray(position, dtype=float).copy()
        self.prev_step = np.zeros(3)

    def step(self, desired: Sequence[float], dt: float) -> np.ndarray:
        """Advance one tick toward `desired`; returns the new commanded position."""
        desired = np.asarray(desired, dtype=float)
        max_step = self.max_speed * dt
        max_step_delta = self.max_accel * dt * dt

        want = desired - self.cmd
        want_norm = float(np.linalg.norm(want))
        step = want if want_norm <= max_step else want * (max_step / want_norm)

        # Acceleration limit, applied to the step vector rather than its magnitude
        # so that a reversal (step flipping sign at full speed) is also bounded.
        delta = step - self.prev_step
        delta_norm = float(np.linalg.norm(delta))
        if delta_norm > max_step_delta:
            step = self.prev_step + delta * (max_step_delta / delta_norm)

        self.cmd = self.cmd + step
        self.prev_step = step
        return self.cmd.copy()


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
        print(f"servo stream up: robot connected from {addr[0]}:{addr[1]}")

    def send(self, pose: Sequence[float], servo: bool = True, keepalive: bool = True) -> None:
        """Stream one setpoint. `servo=False` tells the robot to stop and idle
        (the arm holds position) while keeping the connection alive, which is what
        a lost/stale/refused target should produce - NOT simply withholding
        packets, since silence for --sock-timeout ends the program entirely."""
        if self._conn is None:
            raise RuntimeError("ServoStream.send() before start()")
        vals = [int(round(v * WIRE_SCALE)) for v in pose[:6]]
        vals.append(1 if keepalive else 0)
        vals.append(1 if servo else 0)
        self._conn.sendall(struct.pack(WIRE_FMT, *vals))
        self.sent += 1

    def stop(self) -> None:
        """Clean shutdown: ask the robot to stop and end its program, then close.

        Best-effort at every step - if the connection is already dead the robot's
        own read timeout (layer 3) reaches the same end state a fraction of a
        second later, so a failure here is not a stranded-arm hazard.
        """
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
    assert worst_speed <= 0.25 * 1.001, f"speed limit violated: {worst_speed:.4f} m/s"
    assert float(np.linalg.norm(lim.cmd - target)) < 1e-3, "limiter never converged to target"
    print(f"[rate limit]   worst commanded speed {worst_speed:.4f} m/s (cap 0.25), converged")

    # 2) Acceleration bound, including a hard reversal: with the target flipped
    # every tick the naive magnitude-only clamp would snap the step vector from
    # +max to -max instantly (infinite acceleration). Check it doesn't.
    lim = RateLimiter([0.0, 0.0, 0.0], max_speed=0.5, max_accel=2.0)
    prev_step = np.zeros(3)
    worst_accel = 0.0
    for i in range(400):
        tgt = np.array([1.0, 0, 0]) if (i // 20) % 2 == 0 else np.array([-1.0, 0, 0])
        before = lim.cmd.copy()
        cur = lim.step(tgt, dt)
        step = cur - before
        worst_accel = max(worst_accel, float(np.linalg.norm(step - prev_step)) / (dt * dt))
        prev_step = step
    assert worst_accel <= 2.0 * 1.001, f"accel limit violated: {worst_accel:.4f} m/s^2"
    print(f"[accel limit]  worst commanded accel {worst_accel:.4f} m/s^2 (cap 2.0) across reversals")

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
