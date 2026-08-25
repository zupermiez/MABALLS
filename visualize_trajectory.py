"""
Real-time visualization of a thrown rigid body's trajectory vs. the script's
own prediction, for eyeballing how good the catch-point prediction is.

Kept as a separate script from live_trajectory.py on purpose - that one stays
lean for real robot-control usage, this one adds matplotlib rendering and
extra bookkeeping that would slow down the hot NatNet callback if merged in.
It reuses live_trajectory's SharedState/make_handler for release and landing
detection directly (same threading split: the NatNet callback only runs the
cheap release/landing state machine, all fit_trajectory/plotting work happens
in the render loop, off the socket thread).

Confidence trigger ("flash when the landing prediction is trustworthy"): there
is no ground truth available live, so this uses prediction stability instead
of a fixed sample count - it keeps recomputing the predicted landing point
every render tick and declares it converged once the last `--stability-window`
predictions agree within `--drift-tol` of each other. A fixed-sample-count
threshold was tried and rejected: the number found in the sample-count-sweep
experiment (~67 samples) was calibrated on one specific throw's speed/distance
and won't generalize to faster, slower, or shorter throws.
"""

import argparse
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

from natnet import NatNetClient

import live_trajectory as lt
from trajectory import AXIS_NAMES, fit_trajectory, trim_ghost_tail

ACTUAL_COLOR = "#2a78d6"
LOCKED_PRED_COLOR = "#d03b3b"
FLASH_COLOR = "#fff3b0"
NORMAL_FACECOLOR = None  # set from the figure's default at startup

# Snapshot curves (prediction using only the first N samples of the current
# flight, captured every --snapshot-step samples) are colored by how far into
# the flight they were taken. A continuous colormap (tried first: viridis)
# looked too similar step-to-step with only ~6-8 snapshots per flight -
# neighboring shades of the same hue are hard to tell apart at a glance, which
# defeats the point of watching them separate out. Using hand-picked,
# maximally-distinct hues instead (loosely ordered warm/cool for a rough
# "early vs late" feel, but contrast is what matters here, not a gradient).
# Avoids pure blue/red - those are already ACTUAL_COLOR/LOCKED_PRED_COLOR.
SNAPSHOT_COLORS = [
    "#8e44ad",  # purple
    "#27ae60",  # green
    "#f1c40f",  # yellow
    "#e67e22",  # orange
    "#795548",  # brown
    "#e84393",  # pink
    "#7f8c8d",  # grey
    "#2d3436",  # near-black
]


def build_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--server-ip", default="192.168.10.1")
    p.add_argument("--local-ip", default="192.168.10.2")
    p.add_argument("--unicast", action="store_true")
    p.add_argument("--rigid-body-id", type=int, default=None)

    p.add_argument("--window-short", type=int, default=10)
    p.add_argument("--consecutive", type=int, default=3)
    p.add_argument("--accel-tol", type=float, default=6.0)
    p.add_argument("--residual-max", type=float, default=0.02)
    p.add_argument("--min-release-speed", type=float, default=3.0,
                    help="m/s a short window must average to be considered a real release, "
                         "not a settling/bounce that coincidentally looks ballistic")
    p.add_argument("--floor-refractory", type=float, default=0.5,
                    help="s after a landing during which release detection is suppressed, so a "
                         "floor bounce can't be re-detected as a new throw (0 disables)")
    p.add_argument("--speed-stop", type=float, default=0.3)
    p.add_argument("--stop-frames", type=int, default=5)
    p.add_argument("--lost-frames", type=int, default=8)
    p.add_argument("--max-flight-duration", type=float, default=3.0)
    p.add_argument("--max-flight-samples", type=int, default=400)

    p.add_argument("--catch-axis", choices=AXIS_NAMES, default="y",
                    help="Axis of the ground/catch plane (default: y, Motive's up axis)")
    p.add_argument("--catch-value", type=float, default=0.0,
                    help="Ground/catch plane value on --catch-axis (default: 0.0)")

    p.add_argument("--min-fit-samples", type=int, default=16,
                    help="Minimum flight samples before attempting a landing prediction")
    p.add_argument("--stability-window", type=int, default=4,
                    help="Consecutive predictions checked for convergence")
    p.add_argument("--drift-tol", type=float, default=0.05,
                    help="Max spread (m) within the stability window to declare converged")
    p.add_argument("--predict-step", type=int, default=8,
                    help="Re-evaluate the landing prediction every this many new flight samples "
                         "(sample-count-driven, not render-tick-driven, so convergence timing "
                         "doesn't depend on how fast the plot happens to redraw)")
    p.add_argument("--flash-duration", type=float, default=0.4,
                    help="Seconds the background flashes once converged")
    p.add_argument("--snapshot-step", type=int, default=20,
                    help="Capture a prediction snapshot (using only the first N flight samples) "
                         "every this many samples, for comparing early-vs-late prediction quality")
    p.add_argument("--max-snapshots", type=int, default=8,
                    help="Cap on snapshots per flight (pre-allocates this many plot lines). "
                         "Also sets the color-gradient span, so pick close to the number of "
                         "snapshots a typical flight actually produces (samples/snapshot-step)")
    p.add_argument("--fps", type=float, default=15.0, help="Render loop rate")
    p.add_argument("--debug", action="store_true", help="Print per-tick diagnostics to the terminal")
    p.add_argument("--debug-release", action="store_true",
                    help="Print every idle-state short-window ballistic check (accel/residual/pass-fail)")
    return p.parse_args()


class VizState:
    """Render-loop-only bookkeeping (single-threaded, no lock needed - only
    the main/render thread ever touches this)."""

    def __init__(self):
        self.last_seen_state = "idle"
        self.converged = False
        self.locked_curve = None  # (top_xz, side_dist_y) sampled points
        self.locked_landing = None  # (x, y, z, t)
        self.recent_predictions = deque()
        self.flash_start = None
        self.bounds_top = None  # [xmin,xmax,zmin,zmax]
        self.bounds_side = None  # [dmin,dmax,ymin,ymax]
        self.tick_count = 0
        self.rate_check_wall = None
        self.snapshots = []  # list of (n_samples, (top_xz, side_dist_y))
        self.next_snapshot_n = None  # set from args.snapshot_step on reset
        self.next_predict_n = None  # set from args.min_fit_samples on reset
        self.bounds_streak = None  # [dmin,dmax,ymin,ymax], actual-path bounds only
        self.locked_at_n = None  # sample count at which convergence locked


def predict_landing(flight_samples, catch_axis_idx, catch_value):
    if len(flight_samples) < 3:
        return None
    fit = fit_trajectory(flight_samples)
    last_t = flight_samples[-1].t
    crossing_t = fit.time_of_plane_crossing(catch_axis_idx, catch_value, after_t=last_t)
    if crossing_t is None:
        return None
    pos = fit.position(crossing_t)
    return fit, crossing_t, pos


def sample_curve(fit, t_start, t_end, n=40):
    """Sample fit.position over [t_start, t_end] for drawing a smooth curve."""
    if t_end <= t_start:
        t_end = t_start + 1e-3
    pts = []
    for i in range(n + 1):
        t = t_start + (t_end - t_start) * i / n
        pts.append(fit.position(t))
    return pts


def curve_pair(fit, t_start, t_end, x0, z0, catch_axis, n=40):
    """Sample a curve and project it into (top xz) and (side dist,val) form."""
    pts = sample_curve(fit, t_start, t_end, n=n)
    xs = [p[0] for p in pts]
    zs = [p[2] for p in pts]
    dists = [((p[0] - x0) ** 2 + (p[2] - z0) ** 2) ** 0.5 for p in pts]
    vals = [p[1] if catch_axis == "y" else (p[0] if catch_axis == "x" else p[2]) for p in pts]
    return (xs, zs), (dists, vals)


def step_prediction(flight, v, args, catch_axis_idx, x0, z0):
    """Advance convergence bookkeeping (mutates `v`), catching up through
    every --predict-step sample-count boundary reached since the last call -
    not just one evaluation per render tick. Convergence has to be judged by
    how many actual flight samples have accumulated, not by how often we
    happen to redraw: render fps varies with plotting cost (more panels/
    artists means slower redraws), and evaluating once per tick silently made
    the lock decision depend on how expensive the plot was to draw rather
    than on the data itself - a flight that ends before enough *render ticks*
    (not enough *samples*) had a chance to run would simply never lock, even
    though the underlying fit was ready. Mirrors capture_snapshots' pattern,
    which was already sample-count-driven and unaffected by this.

    Returns extra_stats_lines for the latest (highest-N) prediction this
    tick. Once converged, `v` holds `locked_curve`/`locked_landing`/
    `locked_at_n` and this returns [] from then on.
    """
    stats = []
    while not v.converged and v.next_predict_n <= len(flight):
        n = v.next_predict_n
        v.next_predict_n += args.predict_step

        result = predict_landing(flight[:n], catch_axis_idx, args.catch_value)
        if result is None:
            continue
        fit, crossing_t, pos = result

        v.recent_predictions.append((pos[0], pos[2]))
        if len(v.recent_predictions) > args.stability_window:
            v.recent_predictions.popleft()

        stats = [f"tentative landing=({pos[0]:+.2f},{pos[2]:+.2f}) in {crossing_t - flight[n - 1].t:.2f}s (N={n})"]

        if len(v.recent_predictions) == args.stability_window:
            pts = list(v.recent_predictions)
            spread = max(((ax - bx) ** 2 + (az - bz) ** 2) ** 0.5 for ax, az in pts for bx, bz in pts)
            stats.append(f"spread={spread*100:.2f}cm (need <={args.drift_tol*100:.1f}cm)")
            if spread <= args.drift_tol:
                v.converged = True
                v.flash_start = time.monotonic()
                v.locked_landing = (pos[0], pos[1], pos[2], crossing_t)
                v.locked_curve = curve_pair(fit, flight[0].t, crossing_t, x0, z0, args.catch_axis, n=60)
                v.locked_at_n = n
                if getattr(args, "debug", False):
                    print(f"*** LOCKED IN *** at N={n} samples, spread={spread*100:.2f}cm "
                          f"landing=({pos[0]:+.2f},{pos[2]:+.2f}) in {crossing_t - flight[n - 1].t:.2f}s")

    return stats


def capture_snapshots(flight, v, args, catch_axis_idx, x0, z0):
    """Every --snapshot-step samples, freeze a prediction curve computed from
    only the *first* N samples of this flight (not the latest N) - this
    directly answers "what would the prediction have looked like with only
    this much of the throw seen so far", for comparing early-vs-late
    prediction quality against each other and the eventual locked curve.
    """
    while v.next_snapshot_n <= len(flight) and len(v.snapshots) < args.max_snapshots:
        n = v.next_snapshot_n
        v.next_snapshot_n += args.snapshot_step
        if n < args.min_fit_samples:
            continue  # too few samples yet for a meaningful fit - skip, don't stall
        result = predict_landing(flight[:n], catch_axis_idx, args.catch_value)
        if result is not None:
            fit, crossing_t, _pos = result
            curve = curve_pair(fit, flight[0].t, crossing_t, x0, z0, args.catch_axis, n=40)
            v.snapshots.append((n, curve))
            if getattr(args, "debug", False):
                print(f"  snapshot N={n}: landing=({_pos[0]:+.2f},{_pos[2]:+.2f})")


def expand_bounds(bounds, xs, ys, margin=0.1):
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    if bounds is None:
        bounds = [xmin, xmax, ymin, ymax]
    bounds[0] = min(bounds[0], xmin)
    bounds[1] = max(bounds[1], xmax)
    bounds[2] = min(bounds[2], ymin)
    bounds[3] = max(bounds[3], ymax)
    return bounds


def apply_bounds(ax, bounds, margin_frac=0.15):
    if bounds is None:
        return
    xmin, xmax, ymin, ymax = bounds
    dx = (xmax - xmin) or 1.0
    dy = (ymax - ymin) or 1.0
    ax.set_xlim(xmin - dx * margin_frac, xmax + dx * margin_frac)
    ax.set_ylim(ymin - dy * margin_frac, ymax + dy * margin_frac)


def main():
    args = build_args()
    catch_axis_idx = AXIS_NAMES.index(args.catch_axis)

    s = lt.SharedState()
    if args.rigid_body_id is not None:
        s.target_id = args.rigid_body_id
    v = VizState()

    client = NatNetClient(
        server_ip_address=args.server_ip,
        local_ip_address=args.local_ip,
        use_multicast=not args.unicast,
    )
    client.on_data_frame_received_event.handlers.append(lt.make_handler(s, args))

    fig = plt.figure(figsize=(17, 6.3))
    gs = fig.add_gridspec(2, 3, height_ratios=[10, 1], hspace=0.35)
    ax_top = fig.add_subplot(gs[0, 0])
    ax_side = fig.add_subplot(gs[0, 1])
    ax_streak = fig.add_subplot(gs[0, 2])
    ax_legend = fig.add_subplot(gs[1, :])
    ax_legend.axis("off")
    fig.suptitle("Thrown rigid body: actual path vs. predicted landing", fontsize=12)
    global NORMAL_FACECOLOR
    NORMAL_FACECOLOR = fig.get_facecolor()

    ax_top.set_title("Top view (x - z)")
    ax_top.set_xlabel("x (m)")
    ax_top.set_ylabel("z (m)")
    ax_side.set_title("Side view (distance from release - height)")
    ax_side.set_xlabel("horizontal distance from release (m)")
    ax_side.set_ylabel(f"{args.catch_axis} (m)")
    ax_streak.set_title("Parabola coverage by snapshot")
    ax_streak.set_xlabel("horizontal distance from release (m)")
    ax_streak.set_ylabel(f"{args.catch_axis} (m)")

    (actual_top,) = ax_top.plot([], [], "-", color=ACTUAL_COLOR, linewidth=2, label="actual")
    (locked_pred_top,) = ax_top.plot([], [], "-", color=LOCKED_PRED_COLOR, linewidth=2.2, label="locked")
    (landing_top,) = ax_top.plot([], [], "x", color=LOCKED_PRED_COLOR, markersize=12, markeredgewidth=3)

    (actual_side,) = ax_side.plot([], [], "-", color=ACTUAL_COLOR, linewidth=2, label="actual")
    (locked_pred_side,) = ax_side.plot([], [], "-", color=LOCKED_PRED_COLOR, linewidth=2.2, label="locked")
    (landing_side,) = ax_side.plot([], [], "x", color=LOCKED_PRED_COLOR, markersize=12, markeredgewidth=3)

    # Reference trail (full actual path so far) drawn beneath every streak
    # segment, so any stretch not yet covered by a snapshot still reads as
    # "real path, just not snapshotted yet" rather than empty space.
    (actual_streak,) = ax_streak.plot([], [], "-", color=ACTUAL_COLOR, linewidth=2, zorder=0)

    # Pre-allocated pool of snapshot lines (reused across flights) rather than
    # creating new Line2D artists every tick - snapshot count varies per
    # flight, but FuncAnimation redraws are cheapest when artists are stable.
    snapshot_top_pool = [ax_top.plot([], [], "--", linewidth=1.3)[0] for _ in range(args.max_snapshots)]
    snapshot_side_pool = [ax_side.plot([], [], "--", linewidth=1.3)[0] for _ in range(args.max_snapshots)]
    # Streak lines are drawn thick and solid, shortest (lowest N) on top of
    # longest: fixed zorder set once here, high for low i (short streak, drawn
    # first in capture order) so a shorter streak never gets buried by a
    # later, longer one covering the exact same ground - only the longer
    # streak's *uncovered tail* stays visible, peeking out past the shorter one.
    snapshot_streak_pool = [
        ax_streak.plot([], [], "-", linewidth=4, solid_capstyle="round",
                        zorder=args.max_snapshots - i)[0]
        for i in range(args.max_snapshots)
    ]

    stats_text = fig.text(0.5, 0.02, "", ha="center", va="bottom", fontsize=9, family="monospace")

    def snapshot_color(i):
        # Fixed index into SNAPSHOT_COLORS (not scaled by max_snapshots) so a
        # given snapshot's color is always the same regardless of how many
        # this particular flight ends up producing.
        return SNAPSHOT_COLORS[i % len(SNAPSHOT_COLORS)]

    def rebuild_legend():
        handles = [
            Line2D([0], [0], color=ACTUAL_COLOR, lw=2, label="actual"),
        ]
        for i, (n, _curve) in enumerate(v.snapshots):
            handles.append(Line2D([0], [0], color=snapshot_color(i), lw=1.3, ls="--", label=f"N={n}"))
        if v.converged:
            handles.append(Line2D([0], [0], color=LOCKED_PRED_COLOR, lw=2.2, label=f"locked (N={v.locked_at_n})"))
        ax_legend.legend(
            handles=handles, loc="center", ncol=min(len(handles), 8),
            frameon=False, fontsize=8, handlelength=1.8, columnspacing=1.2,
        )

    def reset_for_new_throw():
        v.converged = False
        v.locked_curve = None
        v.locked_landing = None
        v.locked_at_n = None
        v.recent_predictions.clear()
        v.bounds_top = None
        v.bounds_side = None
        v.bounds_streak = None
        v.snapshots = []
        v.next_snapshot_n = args.snapshot_step
        v.next_predict_n = args.min_fit_samples
        for artist in (actual_top, locked_pred_top, landing_top,
                       actual_side, locked_pred_side, landing_side, actual_streak,
                       *snapshot_top_pool, *snapshot_side_pool, *snapshot_streak_pool):
            artist.set_data([], [])
        rebuild_legend()

    v.next_snapshot_n = args.snapshot_step
    v.next_predict_n = args.min_fit_samples

    def update(_frame):
        now = time.monotonic()
        v.tick_count += 1
        if args.debug:
            if v.rate_check_wall is None:
                v.rate_check_wall = now
            elif now - v.rate_check_wall >= 1.0:
                print(f"[render rate] {v.tick_count / (now - v.rate_check_wall):.1f} ticks/sec "
                      f"(target {args.fps})")
                v.tick_count = 0
                v.rate_check_wall = now

        with lt.STATE_LOCK:
            state = s.state
            # Trimmed, not raw: Motive can briefly latch onto a stray reflective
            # point once the ball's own markers are occluded (typically entering
            # the catch box at the end of a real flight) and keeps reporting a
            # frozen "ghost" position at tracking_valid=True - see
            # trajectory.trim_ghost_tail's docstring. Trimming here (not in
            # live_trajectory.py's shared state) keeps it a display-only concern:
            # the buffer other consumers see is untouched, and this loop's own
            # fit/prediction calls below (step_prediction/capture_snapshots) get
            # the benefit too since they're passed this same local `flight`.
            flight = trim_ghost_tail(list(s.flight_buffer))
            history = list(s.history)
            last_pos = s.last_pos
            last_valid = s.last_valid

        if state == "flight" and v.last_seen_state == "idle":
            reset_for_new_throw()
            if args.debug:
                print(f"[{now:.2f}] NEW THROW: release detected, clearing plot")
        elif state == "idle" and v.last_seen_state == "flight" and args.debug:
            reason = history[0].reason if history else "?"
            print(f"[{now:.2f}] FLIGHT ENDED: reason={reason}")
        v.last_seen_state = state

        x0 = flight[0].x if flight else None
        z0 = flight[0].z if flight else None

        def dist_from_release(sample):
            return ((sample.x - x0) ** 2 + (sample.z - z0) ** 2) ** 0.5

        stats_lines = [f"state={state}"]

        if flight:
            actual_top.set_data([smp.x for smp in flight], [smp.z for smp in flight])
            dists = [dist_from_release(smp) for smp in flight]
            ys = [getattr(smp, args.catch_axis) for smp in flight]
            actual_side.set_data(dists, ys)
            actual_streak.set_data(dists, ys)
            v.bounds_top = expand_bounds(v.bounds_top, [smp.x for smp in flight], [smp.z for smp in flight])
            v.bounds_side = expand_bounds(v.bounds_side, dists, ys)
            v.bounds_streak = expand_bounds(v.bounds_streak, dists, ys)

            elapsed = flight[-1].t - flight[0].t
            stats_lines.append(f"samples={len(flight)} elapsed={elapsed:.3f}s")

            n_snapshots_before = len(v.snapshots)
            capture_snapshots(flight, v, args, catch_axis_idx, x0, z0)
            for i, (n, ((sxs, szs), (sdists, svals))) in enumerate(v.snapshots):
                snapshot_top_pool[i].set_data(sxs, szs)
                snapshot_top_pool[i].set_color(snapshot_color(i))
                snapshot_side_pool[i].set_data(sdists, svals)
                snapshot_side_pool[i].set_color(snapshot_color(i))
                # Snapshot curves (esp. early ones) can extend well beyond the
                # actual trail seen so far - bounds must include them too, or
                # matplotlib clips the curve to the trail's tiny viewport.
                v.bounds_top = expand_bounds(v.bounds_top, sxs, szs)
                v.bounds_side = expand_bounds(v.bounds_side, sdists, svals)
                # Streak = the *actual* (not predicted) path, truncated to
                # this snapshot's first N samples - how much of the real
                # parabola that snapshot needed to make its prediction.
                snapshot_streak_pool[i].set_data(dists[:n], ys[:n])
                snapshot_streak_pool[i].set_color(snapshot_color(i))
            if len(v.snapshots) != n_snapshots_before:
                stats_lines.append(f"snapshots={[n for n, _ in v.snapshots]}")
                rebuild_legend()

            if not v.converged:
                extra_stats = step_prediction(flight, v, args, catch_axis_idx, x0, z0)
                stats_lines.extend(extra_stats)
                if v.converged:
                    rebuild_legend()

            if v.converged and v.locked_curve is not None:
                (lx, lz), (ld, ly) = v.locked_curve
                locked_pred_top.set_data(lx, lz)
                locked_pred_side.set_data(ld, ly)
                v.bounds_top = expand_bounds(v.bounds_top, lx, lz)
                v.bounds_side = expand_bounds(v.bounds_side, ld, ly)
                lx0, ly0, lz0, lt_ = v.locked_landing
                landing_top.set_data([lx0], [lz0])
                landing_dist = ((lx0 - x0) ** 2 + (lz0 - z0) ** 2) ** 0.5
                landing_val = ly0 if args.catch_axis == "y" else (lx0 if args.catch_axis == "x" else lz0)
                landing_side.set_data([landing_dist], [landing_val])
                stats_lines.append(
                    f"LOCKED landing=({lx0:+.2f},{lz0:+.2f}) predicted_t={lt_ - flight[-1].t:+.2f}s from now"
                )

        elif history:
            rec = history[0]
            stats_lines.append(f"last flight: {rec.reason} dur={rec.duration:.2f}s samples={rec.samples}")

        if last_pos is not None:
            stats_lines.append(f"raw pos=({last_pos[0]:+.3f},{last_pos[1]:+.3f},{last_pos[2]:+.3f}) valid={last_valid}")

        stats_text.set_text("   |   ".join(stats_lines))
        if args.debug and flight:
            print("  " + " | ".join(stats_lines))

        apply_bounds(ax_top, v.bounds_top)
        apply_bounds(ax_side, v.bounds_side)
        apply_bounds(ax_streak, v.bounds_streak)

        if v.flash_start is not None and time.monotonic() - v.flash_start < args.flash_duration:
            fig.patch.set_facecolor(FLASH_COLOR)
        else:
            fig.patch.set_facecolor(NORMAL_FACECOLOR)

        return (actual_top, locked_pred_top, landing_top,
                actual_side, locked_pred_side, landing_side, actual_streak,
                *snapshot_top_pool, *snapshot_side_pool, *snapshot_streak_pool)

    with client:
        client.run_async()
        anim = FuncAnimation(fig, update, interval=1000.0 / args.fps, blit=False, cache_frame_data=False)
        try:
            plt.show()
        finally:
            client.stop_async()


if __name__ == "__main__":
    main()
