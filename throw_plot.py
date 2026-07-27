"""
Live, persistent per-throw plot window for catch.py: a single pop-up window
(not a saved image) that updates once after each throw to show the ball's
actual trajectory, the robot's actual TCP path, and how the commit/re-aim/
servo-retarget target guesses converged toward the true catch point.

Why a persistent window updated in place, not a fresh figure/image per throw
(first version of this file did exactly that, saved to PNG): the user wants
the running visual dashboard visualize_trajectory.py gives you, not a file to
go open. That constrains the implementation more than it first looks like it
should:

  - Interactive matplotlib backends (TkAgg here - confirmed the only one this
    machine has, and the same one visualize_trajectory.py already relies on)
    are NOT thread-safe. The window has to be created and updated from
    catch.py's main thread - there is no safe way to hand this off to a
    background worker thread the way a save-to-PNG design could. ThrowPlotWindow
    is therefore driven synchronously from the poll loop, once per throw.
  - That matters because --catch-move servo holds its setpoint stream open
    with only ur_servo.DEFAULT_SOCK_TIMEOUT=0.3s of silence tolerance. A naive
    "clear the axes and replot everything" update measured ~100ms - not over
    budget, but closer to it than comfortable. Pre-allocating every artist
    once at startup and mutating them in place each update (set_data/
    set_offsets/set_segments, never recreating a Line2D/LineCollection/
    PathCollection/legend/colorbar - the same technique visualize_trajectory.py's
    FuncAnimation loop already uses, for the same reason) measured ~90-130ms for
    the full window (2 axes, pre-allocated line/collection artists, 4 scatter
    collections, a shared colorbar, a legend, relim+autoscale on both axes every
    update) - still a comfortable ~2.5x under the 0.3s ceiling, just not the
    near-zero cost a minimal prototype (2 lines, no legend/colorbar) measured at
    ~50-65ms. The time-colored LineCollection scheme (2026-07-27, see below) adds
    two more pre-allocated collections and two more scatter series but does the
    same kind of per-update work (set_segments/set_array instead of set_data, no
    new artist created per update) - expected to stay in the same ballpark, but
    this hasn't been freshly re-measured against a live throw; re-check if
    --catch-move servo starts showing setpoint-stream timeouts after this change.
  - pump() is a bare flush_events() (~0.1ms, measured) meant to be called
    every poll-loop tick regardless of whether a throw just ended, so the
    window keeps responding to resize/close between throws (which can be many
    seconds apart) instead of only reacting the next time update() runs.

Both update() and pump() catch and swallow exceptions from a closed/dead
window (self.closed) rather than raising - a user closing the plot window
must not take the catch session down with it.

**Time-colored flight (2026-07-27):** ball path, arm TCP path, and every
commit/re-aim/retarget marker share ONE colormap (plasma: purple -> magenta ->
orange -> yellow) normalized 0->1 over *that throw's own* elapsed flight time
(release -> impact), not absolute seconds - so "purple" always means "early in
this throw" and "orange/red" always means "late in this throw", regardless of
how long the throw took. That shared, throw-relative clock is what makes the
robot's move legible against the ball's flight: a purple segment on the arm's
path happened at the same moment as the purple segment on the ball's path.
Line *style* (solid ball, dashed arm) carries the ball-vs-arm distinction
instead, since color no longer can. `event_scat_{top,side}` is new: it plots
NOT the predicted catch point (that's `guess_scat`, unchanged in kind) but the
ball's own actual position at the moment of each commit/re-aim/retarget - a
literal mark on the ball's trajectory line at the time the robot moved,
color-matched to the same clock. `n` (a sample count recorded by catch.py,
cheap to grab mid-loop) is mapped to elapsed-flight-fraction via the raw
ball-sample timestamps (`ball_t`, passed into update() alongside `ball_base`)
rather than assumed evenly spaced - correct even if the 120Hz stream hiccups.
"""

from typing import Dict, List, Tuple

import matplotlib.pyplot as plt  # interactive backend (TkAgg) - a real pop-up window, not headless
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.lines import Line2D
import numpy as np

BALL_COLOR = "#2a78d6"        # ball start/end marker identity (blue) - the path itself is time-colored
ROBOT_COLOR = "#1baf7a"       # arm TCP end marker identity (aqua) - the path itself is time-colored
ENVELOPE_COLOR = "#c3c2b7"    # muted chrome - reference geometry, not data
WAIT_COLOR = "#898781"        # muted ink
BASE_COLOR = "#0b0b0b"        # primary ink
GUESS_LINE_COLOR = "#8a3210"  # neutral connector between successive predicted catch-point guesses
NEUTRAL_INK = "#52514e"       # legend swatches for line *style* (color there varies per-throw for real data)

STATUS_CAUGHT = "#0ca30c"
STATUS_MISSED = "#d03b3b"
STATUS_NEUTRAL = "#898781"

# Single perceptual ramp shared by the ball path, arm path, and every decision
# marker: purple (release) -> magenta/red -> orange -> yellow (impact). Fixed
# 0..1 domain = fraction of THIS throw's flight elapsed, not absolute seconds,
# so the full color range is always used regardless of throw duration - see
# module docstring "Time-colored flight".
TIME_CMAP = plt.get_cmap("plasma")
TIME_NORM = Normalize(vmin=0.0, vmax=1.0)


def _circle(radius: float, n: int = 100) -> Tuple[np.ndarray, np.ndarray]:
    theta = np.linspace(0, 2 * np.pi, n)
    return radius * np.cos(theta), radius * np.sin(theta)


def _status(meta: Dict) -> Tuple[str, str]:
    if meta.get("dry_run"):
        return "DRY RUN", STATUS_NEUTRAL
    if not meta.get("attempted"):
        return f"not attempted ({meta.get('reason', '?')})", STATUS_NEUTRAL
    caught = meta.get("caught_guess")
    dist = meta.get("ball_last_dist_m")
    if caught is True:
        return (f"CAUGHT (ball last seen {dist * 100:.0f}cm from tool)" if dist is not None
                else "CAUGHT"), STATUS_CAUGHT
    if caught is False:
        return (f"MISSED (ball last seen {dist:.2f}m from tool)" if dist is not None
                else "MISSED"), STATUS_MISSED
    return "attempted (outcome unknown)", STATUS_NEUTRAL


def _elapsed_fracs(sample_t: np.ndarray) -> np.ndarray:
    """0..1 fraction-of-flight-elapsed for each raw ball sample timestamp,
    release=0, last sample=1. Uses the real timestamps (not an assumed evenly
    -spaced index) so it stays correct through any capture hiccup."""
    n = len(sample_t)
    if n < 2:
        return np.zeros(n)
    span = sample_t[-1] - sample_t[0]
    if span <= 0:
        return np.zeros(n)
    return np.clip((sample_t - sample_t[0]) / span, 0.0, 1.0)


def _n_to_frac(n: int, fracs: np.ndarray) -> float:
    """Map a recorded ball-sample count (tcp_trace/guess 'n', catch.py's
    len(flight_buffer) at that instant) to that same sample's elapsed-flight
    fraction, so the arm's path/markers land on the exact same clock as the
    ball's own path."""
    if len(fracs) == 0:
        return 0.0
    idx = int(np.clip(n - 1, 0, len(fracs) - 1))
    return float(fracs[idx])


def _segments(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """(N,) x (N,) -> (N-1, 2, 2) segment array for a LineCollection."""
    if len(x) < 2:
        return np.empty((0, 2, 2))
    pts = np.column_stack([x, y]).reshape(-1, 1, 2)
    return np.concatenate([pts[:-1], pts[1:]], axis=1)


class ThrowPlotWindow:
    """A persistent, non-blocking pop-up window - see module docstring for why
    it has to be built this way. Construct once at session start (after the
    wait pose / catch envelope are resolved), call update() once per throw at
    throw_end, and pump() every poll-loop tick. Must all happen on the main
    thread."""

    def __init__(self, wait_xyz: np.ndarray, envelope: Dict):
        self.closed = False
        self.wait_xyz = np.asarray(wait_xyz, dtype=float)
        self.envelope = envelope

        plt.ion()
        self.fig = plt.figure(figsize=(12.5, 6.8))
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.fig.canvas.manager.set_window_title("catch.py - throw plot")
        self.fig.subplots_adjust(top=0.86, bottom=0.06)
        gs = self.fig.add_gridspec(2, 2, height_ratios=[10, 1], hspace=0.32, wspace=0.28)
        self.ax_top = self.fig.add_subplot(gs[0, 0])
        self.ax_side = self.fig.add_subplot(gs[0, 1])
        ax_legend = self.fig.add_subplot(gs[1, :])
        ax_legend.axis("off")

        self.ax_top.set_title("Top-down (base X–Y)")
        self.ax_top.set_xlabel("x (m)")
        self.ax_top.set_ylabel("y (m)")
        self.ax_top.set_aspect("equal", adjustable="datalim")
        self.ax_side.set_title("Reach profile (radial distance – height)")
        self.ax_side.set_xlabel("distance from base (m)")
        self.ax_side.set_ylabel("z (m, base frame)")
        for ax in (self.ax_top, self.ax_side):
            ax.margins(0.15)
            ax.grid(True, linewidth=0.4, alpha=0.35)

        self.title_text = self.fig.suptitle("catch.py — waiting for the first throw...",
                                            color=STATUS_NEUTRAL, fontsize=13, fontweight="bold", y=0.975)
        self.stats_text = self.fig.text(0.5, 0.905, "", ha="center", va="top", fontsize=8,
                                        family="monospace", color="#52514e")

        # --- static geometry (catch envelope, wait pose, base) - drawn once, ---
        # --- never touched again: these don't change throw to throw. ---
        reach_min, reach_max = envelope["reach_min"], envelope["reach_max"]
        z_min, z_max = envelope["z_min"], envelope["z_max"]
        wait_az = float(np.arctan2(self.wait_xyz[1], self.wait_xyz[0]))
        az_span = np.radians(envelope.get("max_azimuth_deg", 75.0))
        cx_min, cy_min = _circle(reach_min)
        cx_max, cy_max = _circle(reach_max)
        self.ax_top.plot(cx_min, cy_min, "--", color=ENVELOPE_COLOR, linewidth=1, zorder=1)
        self.ax_top.plot(cx_max, cy_max, "--", color=ENVELOPE_COLOR, linewidth=1, zorder=1)
        for sign in (-1, 1):
            ray_az = wait_az + sign * az_span
            self.ax_top.plot([0, reach_max * np.cos(ray_az)], [0, reach_max * np.sin(ray_az)],
                             ":", color=ENVELOPE_COLOR, linewidth=1, zorder=1)
        self.ax_top.plot(0, 0, "^", color=BASE_COLOR, markersize=10, zorder=6)
        self.ax_top.plot(self.wait_xyz[0], self.wait_xyz[1], "s", color=WAIT_COLOR, markersize=8, zorder=6)
        self.ax_side.add_patch(plt.Rectangle((reach_min, z_min), reach_max - reach_min, z_max - z_min,
                                             fill=False, linestyle="--", edgecolor=ENVELOPE_COLOR,
                                             linewidth=1, zorder=1))
        wait_reach = float(np.hypot(self.wait_xyz[0], self.wait_xyz[1]))
        self.ax_side.plot(wait_reach, self.wait_xyz[2], "s", color=WAIT_COLOR, markersize=8, zorder=6)

        # --- dynamic artists, pre-allocated once, mutated every update() ---
        # Ball and arm paths are LineCollections (not Line2D) so each segment can
        # carry its own color off the shared TIME_CMAP/TIME_NORM clock - solid for
        # the ball, dashed for the arm, since color no longer distinguishes them.
        self.ball_line_top = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                            linewidths=2.4, capstyle="round", zorder=3)
        self.ax_top.add_collection(self.ball_line_top)
        self.ball_line_side = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                             linewidths=2.4, capstyle="round", zorder=3)
        self.ax_side.add_collection(self.ball_line_side)
        (self.ball_top_start,) = self.ax_top.plot([], [], "o", color=BALL_COLOR, markersize=7, zorder=4,
                                                  markerfacecolor="white", markeredgewidth=1.6)
        (self.ball_top_end,) = self.ax_top.plot([], [], "x", color=BALL_COLOR, markersize=10,
                                                markeredgewidth=2.4, zorder=4)
        (self.ball_side_start,) = self.ax_side.plot([], [], "o", color=BALL_COLOR, markersize=7, zorder=4,
                                                    markerfacecolor="white", markeredgewidth=1.6)
        (self.ball_side_end,) = self.ax_side.plot([], [], "x", color=BALL_COLOR, markersize=10,
                                                  markeredgewidth=2.4, zorder=4)

        self.arm_line_top = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                           linewidths=2.6, linestyles="--", capstyle="round", zorder=3)
        self.ax_top.add_collection(self.arm_line_top)
        self.arm_line_side = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                            linewidths=2.6, linestyles="--", capstyle="round", zorder=3)
        self.ax_side.add_collection(self.arm_line_side)
        (self.arm_top_end,) = self.ax_top.plot([], [], "o", color=ROBOT_COLOR, markersize=8, zorder=5)
        (self.arm_side_end,) = self.ax_side.plot([], [], "o", color=ROBOT_COLOR, markersize=8, zorder=5)

        # Predicted catch-point guesses (a point out in space, ahead of the ball -
        # NOT the same as event_scat below, which marks the ball's own position).
        (self.guess_line_top,) = self.ax_top.plot([], [], ":", color=GUESS_LINE_COLOR, alpha=0.5,
                                                   linewidth=1, zorder=4)
        (self.guess_line_side,) = self.ax_side.plot([], [], ":", color=GUESS_LINE_COLOR, alpha=0.5,
                                                     linewidth=1, zorder=4)
        self.guess_scat_top = self.ax_top.scatter([], [], c=[], cmap=TIME_CMAP, norm=TIME_NORM,
                                                   s=80, edgecolors="white", linewidths=0.9, zorder=7)
        self.guess_scat_side = self.ax_side.scatter([], [], c=[], cmap=TIME_CMAP, norm=TIME_NORM,
                                                     s=80, edgecolors="white", linewidths=0.9, zorder=7)
        (self.guess_star_top,) = self.ax_top.plot([], [], "*", color=GUESS_LINE_COLOR, markersize=16,
                                                  markeredgecolor="white", markeredgewidth=0.8, zorder=8)
        (self.guess_star_side,) = self.ax_side.plot([], [], "*", color=GUESS_LINE_COLOR, markersize=16,
                                                    markeredgecolor="white", markeredgewidth=0.8, zorder=8)

        # The "stripe": the ball's own actual position at the moment of each
        # commit/re-aim/retarget, marked directly on its path - this is what
        # answers "when did the move happen, on the ball's own flight". Diamond,
        # bigger + heavier edge for the initial commit than for later re-aims/
        # retargets, colored on the same shared clock as everything else here.
        self.event_scat_top = self.ax_top.scatter([], [], c=[], cmap=TIME_CMAP, norm=TIME_NORM,
                                                   marker="D", s=[], edgecolors="black",
                                                   linewidths=1.1, zorder=6)
        self.event_scat_side = self.ax_side.scatter([], [], c=[], cmap=TIME_CMAP, norm=TIME_NORM,
                                                     marker="D", s=[], edgecolors="black",
                                                     linewidths=1.1, zorder=6)

        sm = plt.cm.ScalarMappable(cmap=TIME_CMAP, norm=TIME_NORM)
        sm.set_array([])
        cbar = self.fig.colorbar(sm, ax=[self.ax_top, self.ax_side], orientation="horizontal",
                                 fraction=0.05, pad=0.12, aspect=40)
        cbar.set_label("time through this throw's flight (release -> impact) — "
                       "shared by the ball path, arm path, and move markers", fontsize=8)
        cbar.ax.tick_params(labelsize=7)

        handles = [
            Line2D([0], [0], color=NEUTRAL_INK, lw=2.4, ls="-", label="ball path (solid; color = time)"),
            Line2D([0], [0], color=NEUTRAL_INK, lw=2.6, ls="--", label="arm TCP path (dashed; same time scale)"),
            Line2D([0], [0], marker="D", color="none", markerfacecolor=NEUTRAL_INK, markeredgecolor="black",
                  markersize=8, label="commit / re-aim / retarget (marked on ball path)"),
            Line2D([0], [0], color=GUESS_LINE_COLOR, lw=1, ls=":", marker="*", markersize=10,
                  label="predicted catch point (converging guesses)"),
            Line2D([0], [0], marker="s", color="none", markerfacecolor=WAIT_COLOR, markersize=8, label="wait pose"),
            Line2D([0], [0], marker="^", color="none", markerfacecolor=BASE_COLOR, markersize=8, label="base"),
            Line2D([0], [0], color=ENVELOPE_COLOR, lw=1, ls="--", label="catch envelope"),
        ]
        ax_legend.legend(handles=handles, loc="center", ncol=4,
                        frameon=False, fontsize=8, handlelength=1.8, columnspacing=1.3)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def _on_close(self, _event) -> None:
        self.closed = True

    def pump(self) -> None:
        """Call every poll-loop tick (cheap, ~0.1ms measured with nothing
        pending) so the window responds to resize/close between throws
        instead of only at the next update()."""
        if self.closed:
            return
        try:
            self.fig.canvas.flush_events()
        except Exception:
            self.closed = True

    def _clear_event_scat(self) -> None:
        self.event_scat_top.set_offsets(np.empty((0, 2)))
        self.event_scat_top.set_array(np.array([]))
        self.event_scat_top.set_sizes(np.array([]))
        self.event_scat_side.set_offsets(np.empty((0, 2)))
        self.event_scat_side.set_array(np.array([]))
        self.event_scat_side.set_sizes(np.array([]))

    def update(self, throw_no: int, ball_base: np.ndarray, ball_t: np.ndarray,
              tcp_trace: List[Tuple[int, np.ndarray]], guesses: List[Dict], meta: Dict) -> None:
        """Mutate the pre-allocated artists for this throw and redraw.
        Synchronous, ~90-130ms measured pre-2026-07-27 (see module docstring for
        why that's an acceptable one-time cost, once per throw, even in
        --catch-move servo; not freshly re-measured since).

        ball_t: raw per-sample timestamps for ball_base, same order/length -
        the clock every other time-colored artist here (arm path, commit/re-aim/
        retarget markers, predicted-catch-point guesses) gets mapped onto via
        _n_to_frac(), so a color match anywhere on this plot means "same moment
        in this throw's flight."
        """
        if self.closed:
            return
        ball_base = np.asarray(ball_base, dtype=float)
        ball_t = np.asarray(ball_t, dtype=float)
        fracs = _elapsed_fracs(ball_t)

        status_label, status_color = _status(meta)
        self.title_text.set_text(f"Throw #{throw_no} — {status_label}")
        self.title_text.set_color(status_color)

        commit_note = ""
        commits = [g for g in guesses if g.get("kind") == "commit"]
        if commits and len(fracs) and len(ball_t) >= 2:
            idx0 = int(np.clip(commits[0]["n"] - 1, 0, len(fracs) - 1))
            commit_t = float(ball_t[idx0] - ball_t[0])
            commit_note = f"  commit@{commit_t:.2f}s({fracs[idx0] * 100:.0f}% of flight)"

        dur, samples = meta.get("duration"), meta.get("samples")
        stats = (f"n_samples={samples}  duration={dur:.2f}s  " if dur is not None else "") \
            + f"catch_move={meta.get('catch_move')}  n_guesses={len(guesses)}{commit_note}"
        self.stats_text.set_text(stats)

        if len(ball_base):
            bx, by, bz = ball_base[:, 0], ball_base[:, 1], ball_base[:, 2]
            b_reach = np.hypot(bx, by)
            seg_colors = fracs[:-1] if len(fracs) > 1 else np.zeros(0)
            self.ball_line_top.set_segments(_segments(bx, by))
            self.ball_line_top.set_array(seg_colors)
            self.ball_line_side.set_segments(_segments(b_reach, bz))
            self.ball_line_side.set_array(seg_colors)
            self.ball_top_start.set_data([bx[0]], [by[0]])
            self.ball_top_end.set_data([bx[-1]], [by[-1]])
            self.ball_side_start.set_data([b_reach[0]], [bz[0]])
            self.ball_side_end.set_data([b_reach[-1]], [bz[-1]])
        else:
            self.ball_line_top.set_segments(np.empty((0, 2, 2)))
            self.ball_line_top.set_array(np.zeros(0))
            self.ball_line_side.set_segments(np.empty((0, 2, 2)))
            self.ball_line_side.set_array(np.zeros(0))
            for artist in (self.ball_top_start, self.ball_top_end,
                          self.ball_side_start, self.ball_side_end):
                artist.set_data([], [])

        if tcp_trace:
            tcp_xyz = np.array([p for _n, p in tcp_trace], dtype=float)
            tcp_fracs = np.array([_n_to_frac(n, fracs) for n, _p in tcp_trace])
            tx, ty, tz = tcp_xyz[:, 0], tcp_xyz[:, 1], tcp_xyz[:, 2]
            t_reach = np.hypot(tx, ty)
            arm_seg_colors = tcp_fracs[:-1] if len(tcp_fracs) > 1 else np.zeros(0)
            self.arm_line_top.set_segments(_segments(tx, ty))
            self.arm_line_top.set_array(arm_seg_colors)
            self.arm_line_side.set_segments(_segments(t_reach, tz))
            self.arm_line_side.set_array(arm_seg_colors)
            self.arm_top_end.set_data([tx[-1]], [ty[-1]])
            self.arm_side_end.set_data([t_reach[-1]], [tz[-1]])
        else:
            self.arm_line_top.set_segments(np.empty((0, 2, 2)))
            self.arm_line_top.set_array(np.zeros(0))
            self.arm_line_side.set_segments(np.empty((0, 2, 2)))
            self.arm_line_side.set_array(np.zeros(0))
            self.arm_top_end.set_data([], [])
            self.arm_side_end.set_data([], [])

        if guesses:
            gxyz = np.array([g["point"] for g in guesses], dtype=float)
            g_fracs = np.array([_n_to_frac(g["n"], fracs) for g in guesses])
            gx, gy, gz = gxyz[:, 0], gxyz[:, 1], gxyz[:, 2]
            g_reach = np.hypot(gx, gy)
            self.guess_line_top.set_data(gx, gy)
            self.guess_line_side.set_data(g_reach, gz)
            self.guess_scat_top.set_offsets(np.column_stack([gx, gy]))
            self.guess_scat_top.set_array(g_fracs)
            self.guess_scat_side.set_offsets(np.column_stack([g_reach, gz]))
            self.guess_scat_side.set_array(g_fracs)
            self.guess_star_top.set_data([gx[-1]], [gy[-1]])
            self.guess_star_side.set_data([g_reach[-1]], [gz[-1]])

            if len(ball_base):
                idxs = np.clip(np.array([g["n"] for g in guesses], dtype=int) - 1, 0, len(ball_base) - 1)
                ex, ey, ez = ball_base[idxs, 0], ball_base[idxs, 1], ball_base[idxs, 2]
                e_reach = np.hypot(ex, ey)
                sizes = np.array([130 if g.get("kind") == "commit" else 55 for g in guesses])
                self.event_scat_top.set_offsets(np.column_stack([ex, ey]))
                self.event_scat_top.set_array(g_fracs)
                self.event_scat_top.set_sizes(sizes)
                self.event_scat_side.set_offsets(np.column_stack([e_reach, ez]))
                self.event_scat_side.set_array(g_fracs)
                self.event_scat_side.set_sizes(sizes)
            else:
                self._clear_event_scat()
        else:
            self.guess_line_top.set_data([], [])
            self.guess_line_side.set_data([], [])
            self.guess_scat_top.set_offsets(np.empty((0, 2)))
            self.guess_scat_top.set_array(np.array([]))
            self.guess_scat_side.set_offsets(np.empty((0, 2)))
            self.guess_scat_side.set_array(np.array([]))
            self.guess_star_top.set_data([], [])
            self.guess_star_side.set_data([], [])
            self._clear_event_scat()

        # relim() only walks Line2D/Patch artists, not collections - every
        # LineCollection/scatter's extent has to be fed in explicitly or a throw
        # with movement only in one of them would never rescale to show it.
        top_collections = (self.guess_scat_top, self.event_scat_top, self.ball_line_top, self.arm_line_top)
        side_collections = (self.guess_scat_side, self.event_scat_side, self.ball_line_side, self.arm_line_side)
        for ax, collections in ((self.ax_top, top_collections), (self.ax_side, side_collections)):
            ax.relim()
            for artist in collections:
                ax.update_datalim(artist.get_datalim(ax.transData))
            ax.autoscale_view()

        try:
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
        except Exception:
            self.closed = True

    def close(self) -> None:
        if not self.closed:
            try:
                plt.close(self.fig)
            except Exception:
                pass
            self.closed = True
