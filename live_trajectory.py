"""
Live release detection + trajectory prediction for a tracked rigid body.

Wires the NatNet stream (see CLAUDE.md for the confirmed multicast setup) into
trajectory.py's quadratic fit, closing the loop that was previously just a
synthetic self-test.

Design note - why release detection is automatic, not a button press: a human
button press has ~150-250ms of reaction latency, which is large relative to a
short throw and would corrupt exactly the samples needed to seed the fit. Free
flight has a distinctive, cheap-to-check signature instead: acceleration on
one axis settles to -9.81 m/s^2 and ~0 on the other two (trajectory.py already
computes this per fit via `up_axis`/residual_rms). We watch a short rolling
window and require several consecutive fits to look ballistic before trusting
it - this rejects the transient false-positives a hand's wind-up can produce
(e.g. a swing that briefly, coincidentally, matches gravity on one axis).

Threading note: NatNetClient runs packet parsing and this module's frame
callback on the same dedicated socket-recv thread (verified by reading
nat_net_client.py). At 120Hz the inter-packet budget is ~8.3ms, so the
callback (`handle_frame`) only ever fits a small fixed-size window
(`--window-short` samples) for release/landing checks - it never re-fits the
full, growing flight buffer. That heavier fit (up to a few hundred points)
runs only in the 10Hz display loop, off the hot thread - same split
live_view.py uses between its callback and its render loop.
"""

import argparse
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional

from natnet import NatNetClient, DataFrame
from rich.live import Live
from rich.table import Table
from rich.console import Group
from rich.panel import Panel
from rich.text import Text

from trajectory import G, AXIS_NAMES, Sample, TrajectoryFit, fit_trajectory

STATE_LOCK = threading.Lock()

# A backward timestamp jump bigger than this is treated as the data source
# restarting (e.g. a looped Motive playback), not a stale/duplicate packet -
# real jitter/reordering never approaches this, only a genuine discontinuity.
STREAM_RESTART_GAP = 0.5

# Reason string for a flight that ended by the object slowing to a stop on a
# surface. Named so finalize_flight() can key the post-landing refractory off it
# without the string drifting from where it's produced in the callback.
LANDING_REASON = "stopped (caught/landed)"


def looks_ballistic(fit: TrajectoryFit, accel_tol: float, residual_max: float) -> bool:
    """True if a fit's acceleration matches free-fall: -G on one axis, ~0 on the rest."""
    if fit.residual_rms > residual_max:
        return False
    up = fit.axes[fit.up_axis]
    if abs(up.a - (-G)) > accel_tol:
        return False
    for i, axis in enumerate(fit.axes):
        if i != fit.up_axis and abs(axis.a) > accel_tol:
            return False
    return True


def window_speed(window: Deque[Sample]) -> float:
    """Average speed (m/s) across a short window, from its first to last sample."""
    w0, w1 = window[0], window[-1]
    dt = w1.t - w0.t
    if dt <= 0:
        return 0.0
    return ((w1.x - w0.x) ** 2 + (w1.y - w0.y) ** 2 + (w1.z - w0.z) ** 2) ** 0.5 / dt


@dataclass
class FlightRecord:
    reason: str
    duration: float
    samples: int
    peak_height_rise: float
    peak_speed: float
    residual_rms: float


@dataclass
class SharedState:
    target_id: Optional[int] = None
    candidate_ids: List[int] = field(default_factory=list)
    state: str = "idle"  # "idle" | "flight"

    short_window: Deque[Sample] = field(default_factory=lambda: deque())
    consecutive_good: int = 0

    flight_buffer: List[Sample] = field(default_factory=list)
    flight_max_speed: float = 0.0
    low_speed_count: int = 0
    lost_count: int = 0

    last_sample_t: Optional[float] = None
    last_pos: Optional[tuple] = None
    last_valid: bool = False
    last_marker_error: Optional[float] = None

    # Sample-time (not wall-clock) before which release detection is suppressed.
    # Set when a flight ends by landing, so the ball's own bounce off the floor -
    # a genuine low free-flight arc that a speed threshold alone can't always
    # reject - can't be re-detected as a fresh throw. See finalize_flight().
    refractory_until: Optional[float] = None

    history: Deque[FlightRecord] = field(default_factory=lambda: deque(maxlen=8))

    fps_window: Deque[tuple] = field(default_factory=lambda: deque(maxlen=180))


def finalize_flight(s: SharedState, reason: str, floor_refractory: float = 0.0) -> None:
    buf = s.flight_buffer
    # A bounce is a real ballistic arc off the floor right after the ball lands,
    # so it can pass the ballistic+speed release gate and get mis-detected as a
    # new throw. Only a landing can be followed by a bounce, so arm a short
    # refractory (in sample-time) after a landing that suppresses re-detection -
    # regardless of how energetic the bounce happens to be.
    if reason == LANDING_REASON and floor_refractory > 0.0 and buf:
        s.refractory_until = buf[-1].t + floor_refractory
    if len(buf) >= 3:
        fit = fit_trajectory(buf)
        up = fit.up_axis
        peak_rise = max(sample_axis(b, up) for b in buf) - sample_axis(buf[0], up)
        s.history.appendleft(
            FlightRecord(
                reason=reason,
                duration=buf[-1].t - buf[0].t,
                samples=len(buf),
                peak_height_rise=peak_rise,
                peak_speed=s.flight_max_speed,
                residual_rms=fit.residual_rms,
            )
        )
    s.state = "idle"
    s.flight_buffer = []
    s.flight_max_speed = 0.0
    s.low_speed_count = 0
    s.lost_count = 0
    s.short_window.clear()
    s.consecutive_good = 0


def sample_axis(sample: Sample, axis: int) -> float:
    return (sample.x, sample.y, sample.z)[axis]


def make_handler(s: SharedState, args):
    window_short = args.window_short

    def handle_frame(frame: DataFrame) -> None:
        with STATE_LOCK:
            s.fps_window.append((time.monotonic(), frame.prefix.frame_number))

            if s.target_id is None:
                ids = [rb.id_num for rb in frame.rigid_bodies]
                if len(ids) == 1:
                    s.target_id = ids[0]
                else:
                    s.candidate_ids = ids
                return

            rb = next((r for r in frame.rigid_bodies if r.id_num == s.target_id), None)
            if rb is None:
                return

            t = frame.suffix.timestamp
            valid = True if rb.tracking_valid is None else rb.tracking_valid
            s.last_valid = valid
            s.last_pos = rb.pos
            s.last_marker_error = rb.marker_error

            stream_restarted = (
                s.last_sample_t is not None and t < s.last_sample_t - STREAM_RESTART_GAP
            )
            if stream_restarted:
                # Timestamp jumped far backward - e.g. a looped Motive playback
                # restarting, not a stale/duplicate packet. Treat as a fresh
                # start rather than silently dropping every subsequent frame
                # for looking "older" than the last one before the loop point.
                if s.state == "flight":
                    finalize_flight(s, "stream restarted")
                s.short_window.clear()
                s.consecutive_good = 0
                s.last_sample_t = None
                s.refractory_until = None

            if not valid or (s.last_sample_t is not None and t <= s.last_sample_t):
                if s.state == "idle":
                    s.short_window.clear()
                    s.consecutive_good = 0
                else:
                    s.lost_count += 1
                    if s.lost_count >= args.lost_frames:
                        finalize_flight(s, "lost tracking", args.floor_refractory)
                return

            s.last_sample_t = t
            sample = Sample(t, *rb.pos)

            if s.refractory_until is not None and t >= s.refractory_until:
                s.refractory_until = None

            if s.state == "idle":
                s.short_window.append(sample)
                if len(s.short_window) > window_short:
                    s.short_window.popleft()
                if s.refractory_until is not None:
                    # In the post-landing refractory: keep filling the window so
                    # detection resumes cleanly once it lapses, but never declare
                    # a release (this is where a floor bounce would slip through).
                    s.consecutive_good = 0
                elif len(s.short_window) == window_short:
                    try:
                        fit = fit_trajectory(list(s.short_window))
                    except ValueError:
                        fit = None
                    speed = window_speed(s.short_window)
                    ballistic = fit is not None and looks_ballistic(fit, args.accel_tol, args.residual_max)
                    # Ballistic acceleration alone isn't enough: a ball settling
                    # near the ground after landing/bouncing can produce a brief,
                    # very clean-looking free-flight window (small hop, low
                    # residual) purely by chance. A real release always has
                    # non-trivial velocity, so require that too - this is what
                    # separates an actual throw from a slow post-impact bounce.
                    good = ballistic and speed >= args.min_release_speed
                    if good:
                        s.consecutive_good += 1
                    else:
                        s.consecutive_good = 0
                    if getattr(args, "debug_release", False) and fit is not None:
                        up = fit.axes[fit.up_axis]
                        others = [fit.axes[i].a for i in range(3) if i != fit.up_axis]
                        print(
                            f"[release-scan] t={t:.3f} pos=({sample.x:+.3f},{sample.y:+.3f},{sample.z:+.3f}) "
                            f"speed={speed:.2f}m/s up_axis={AXIS_NAMES[fit.up_axis]} "
                            f"a_up={up.a:+.2f} a_other={others[0]:+.2f},{others[1]:+.2f} "
                            f"residual={fit.residual_rms * 1000:.2f}mm ballistic={ballistic} good={good} "
                            f"consecutive_good={s.consecutive_good}"
                        )
                    if s.consecutive_good >= args.consecutive:
                        s.state = "flight"
                        s.flight_buffer = list(s.short_window)
                        s.flight_max_speed = 0.0
                        s.low_speed_count = 0
                        s.lost_count = 0
                        s.consecutive_good = 0
                        s.short_window.clear()
            else:  # flight
                s.lost_count = 0
                s.flight_buffer.append(sample)
                if len(s.flight_buffer) > args.max_flight_samples:
                    s.flight_buffer.pop(0)

                prev = s.flight_buffer[-2] if len(s.flight_buffer) >= 2 else None
                if prev is not None:
                    dt = sample.t - prev.t
                    if dt > 0:
                        speed = (
                            (sample.x - prev.x) ** 2
                            + (sample.y - prev.y) ** 2
                            + (sample.z - prev.z) ** 2
                        ) ** 0.5 / dt
                        s.flight_max_speed = max(s.flight_max_speed, speed)
                        if speed < args.speed_stop:
                            s.low_speed_count += 1
                        else:
                            s.low_speed_count = 0
                        if s.low_speed_count >= args.stop_frames:
                            finalize_flight(s, LANDING_REASON, args.floor_refractory)
                            return

                if sample.t - s.flight_buffer[0].t > args.max_flight_duration:
                    finalize_flight(s, "timeout", args.floor_refractory)

    return handle_frame


def compute_rates(s: SharedState):
    samples = list(s.fps_window)
    if len(samples) < 2:
        return 0.0, 0.0
    t0, f0 = samples[0]
    t1, f1 = samples[-1]
    dt = t1 - t0
    if dt <= 0:
        return 0.0, 0.0
    return (len(samples) - 1) / dt, (f1 - f0) / dt


def build_display(s: SharedState, args) -> Group:
    with STATE_LOCK:
        target_id = s.target_id
        candidate_ids = list(s.candidate_ids)
        state = s.state
        last_pos = s.last_pos
        last_valid = s.last_valid
        last_marker_error = s.last_marker_error
        flight_buffer = list(s.flight_buffer)
        flight_max_speed = s.flight_max_speed
        history = list(s.history)
        packet_fps, motive_fps = compute_rates(s)

    header = Panel(
        f"target rigid body id: {target_id if target_id is not None else '(not yet selected)'}   "
        f"stream fps: [bold]{packet_fps:5.1f}[/bold]   motive fps: [bold]{motive_fps:5.1f}[/bold]",
        title="Live Trajectory Predictor",
    )

    if target_id is None:
        if candidate_ids:
            msg = (
                f"Multiple rigid bodies seen: {candidate_ids}. "
                f"Re-run with --rigid-body-id <id> to pick one."
            )
        else:
            msg = "Waiting for first frame..."
        return Group(header, Panel(msg))

    state_color = "yellow" if state == "flight" else "green"
    state_text = Text(f"STATE: {state.upper()}", style=f"bold {state_color}")

    status_lines = [state_text]
    if last_pos is not None:
        x, y, z = last_pos
        status_lines.append(
            Text(f"pos=({x:+.4f}, {y:+.4f}, {z:+.4f})  valid={last_valid}  "
                 f"marker_error={last_marker_error if last_marker_error is not None else '-'}")
        )

    panels = [header, Panel(Group(*status_lines), title="Status")]

    if state == "flight" and len(flight_buffer) >= 3:
        fit = fit_trajectory(flight_buffer)
        last_t = flight_buffer[-1].t
        elapsed = last_t - flight_buffer[0].t
        v = [fit.axes[i].v0 + fit.axes[i].a * elapsed for i in range(3)]
        speed_now = (v[0] ** 2 + v[1] ** 2 + v[2] ** 2) ** 0.5

        pred_lines = [
            Text(f"samples={len(flight_buffer)}  elapsed={elapsed:.3f}s  "
                 f"residual_rms={fit.residual_rms * 1000:.2f}mm  up_axis={AXIS_NAMES[fit.up_axis]}"),
            Text(f"velocity=({v[0]:+.2f}, {v[1]:+.2f}, {v[2]:+.2f}) m/s  "
                 f"|v|={speed_now:.2f}  peak_speed_seen={flight_max_speed:.2f}"),
        ]

        ahead_t = last_t + args.predict_ahead
        px, py, pz = fit.position(ahead_t)
        pred_lines.append(
            Text(f"+{args.predict_ahead:.2f}s predicted pos=({px:+.3f}, {py:+.3f}, {pz:+.3f})")
        )

        up = fit.axes[fit.up_axis]
        if up.a < 0:
            apex_dt = -up.v0 / up.a
            apex_t = fit.t0 + apex_dt
            if apex_t > last_t:
                apex_pos = fit.position(apex_t)
                pred_lines.append(
                    Text(f"apex in {apex_t - last_t:.3f}s at height={apex_pos[fit.up_axis]:+.3f}")
                )

        if args.catch_axis is not None:
            axis_idx = AXIS_NAMES.index(args.catch_axis)
            crossing_t = fit.time_of_plane_crossing(axis_idx, args.catch_value, after_t=last_t)
            if crossing_t is not None:
                cx, cy, cz = fit.position(crossing_t)
                pred_lines.append(
                    Text(
                        f"catch plane {args.catch_axis}={args.catch_value}: "
                        f"in {crossing_t - last_t:.3f}s at pos=({cx:+.3f}, {cy:+.3f}, {cz:+.3f})",
                        style="bold cyan",
                    )
                )
            else:
                pred_lines.append(Text(f"catch plane {args.catch_axis}={args.catch_value}: no crossing predicted"))

        panels.append(Panel(Group(*pred_lines), title="Prediction"))

    hist_table = Table(title="Recent flights")
    for col in ("reason", "duration", "samples", "peak_rise", "peak_speed", "residual_rms"):
        hist_table.add_column(col)
    for rec in history:
        hist_table.add_row(
            rec.reason,
            f"{rec.duration:.3f}s",
            str(rec.samples),
            f"{rec.peak_height_rise:+.3f}m",
            f"{rec.peak_speed:.2f}m/s",
            f"{rec.residual_rms * 1000:.2f}mm",
        )
    panels.append(hist_table)

    return Group(*panels)


def add_release_detection_args(parser: argparse.ArgumentParser) -> None:
    """Flags shared by every tool that consumes the release/flight-end state machine
    in make_handler() - factored out so tuning here (see CLAUDE.md "Release
    detection") can't silently drift between live_trajectory.py and other consumers
    (e.g. catch_feasibility.py) that need the exact same ballistic-detection behavior.
    """
    parser.add_argument("--window-short", type=int, default=10, help="Samples in the release-detection window")
    parser.add_argument("--consecutive", type=int, default=3, help="Consecutive ballistic windows required to declare release")
    parser.add_argument("--accel-tol", type=float, default=6.0, help="m/s^2 tolerance around gravity/zero")
    parser.add_argument("--residual-max", type=float, default=0.02, help="Max fit residual RMS (m) to trust as ballistic")
    parser.add_argument("--min-release-speed", type=float, default=3.0,
                         help="m/s a short window must average to be considered a real release, "
                              "not a settling/bounce that coincidentally looks ballistic. A floor "
                              "bounce is lower-energy than the lofted throw that caused it; 3.0 sits "
                              "between a measured ~2.2 m/s bounce and the ~4.9 m/s minimum release "
                              "speed of a >1m-apex lofted throw")
    parser.add_argument("--floor-refractory", type=float, default=0.5,
                         help="s after a flight ends by landing during which release detection is "
                              "suppressed, so the ball's bounce off the floor can't be re-detected "
                              "as a new throw regardless of how energetic the bounce is (0 disables)")

    parser.add_argument("--speed-stop", type=float, default=0.3, help="m/s below which the object is considered stopped")
    parser.add_argument("--stop-frames", type=int, default=5, help="Consecutive low-speed samples to end a flight")
    parser.add_argument("--lost-frames", type=int, default=8, help="Consecutive invalid/missing samples to end a flight")
    parser.add_argument("--max-flight-duration", type=float, default=3.0, help="Safety timeout (s) per flight")
    parser.add_argument("--max-flight-samples", type=int, default=400, help="Safety cap on flight buffer length")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--rigid-body-id", type=int, default=None, help="NatNet rigid body id to track")

    add_release_detection_args(parser)

    parser.add_argument("--predict-ahead", type=float, default=0.15, help="Seconds ahead to show a predicted position")
    parser.add_argument("--catch-axis", choices=AXIS_NAMES, default=None, help="Axis of the catch plane")
    parser.add_argument("--catch-value", type=float, default=None, help="Value of the catch plane on --catch-axis")

    parser.add_argument("--duration", type=float, default=None, help="Exit automatically after N seconds")
    parser.add_argument("--debug-release", action="store_true",
                         help="Print every idle-state short-window ballistic check (accel/residual/pass-fail)")
    args = parser.parse_args()

    if (args.catch_axis is None) != (args.catch_value is None):
        parser.error("--catch-axis and --catch-value must be given together")

    s = SharedState()
    if args.rigid_body_id is not None:
        s.target_id = args.rigid_body_id

    client = NatNetClient(
        server_ip_address=args.server_ip,
        local_ip_address=args.local_ip,
        use_multicast=not args.unicast,
    )
    client.on_data_frame_received_event.handlers.append(make_handler(s, args))

    with client:
        client.run_async()
        start = time.monotonic()
        try:
            with Live(refresh_per_second=10) as live:
                while args.duration is None or (time.monotonic() - start) < args.duration:
                    live.update(build_display(s, args))
                    time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            client.stop_async()


if __name__ == "__main__":
    main()
