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

# Light theme (2026-08-25, replacing the earlier dark theme): chart chrome/ink
# pulled from the dataviz skill's validated light-mode tokens (references/
# palette.md), not eyeballed - surfaces, ink, and gridline all come from the
# same "Chart chrome & ink" light column; categorical marks use each slot's
# light step instead of its dark step.
PAGE_PLANE = "#f9f9f7"        # figure background (validated token)
CHART_SURFACE = "#fcfcfb"     # axes panel background, one step off the page for separation
INK_PRIMARY = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"         # axis/tick labels - same value in both modes by design
GRIDLINE = "#e1e0d9"
AXIS_LINE = "#c3c2b7"

BALL_COLOR = "#2a78d6"        # categorical slot 1 (blue), light step - ball start/end marker identity
ROBOT_COLOR = "#1baf7a"       # categorical slot 3 (aqua), light step - arm TCP end marker identity
ENVELOPE_COLOR = INK_MUTED    # reference geometry, not data - muted so it recedes behind real data
WAIT_COLOR = INK_MUTED
BASE_COLOR = "#4a3aa7"        # categorical slot 7 (violet), light step
GUESS_LINE_COLOR = "#eb6834"  # categorical slot 2 (orange), light step - connector between predicted catch-point guesses
NEUTRAL_INK = INK_SECONDARY   # legend swatches for line *style* (color there varies per-throw for real data)

STATUS_CAUGHT = "#0ca30c"     # status "good" - fixed hex, same on both surfaces (3.27:1 on light)
STATUS_MISSED = "#e34948"     # categorical slot 8 (red), light step
STATUS_NEUTRAL = INK_MUTED
FRAME_COLOR = INK_SECONDARY   # solid gray for the physical mount frame - distinct from the
                               # dashed, lighter ENVELOPE_COLOR so a real structure doesn't
                               # read as an abstract limit

# Aluminium arm-mount frame (2026-08-25, user-supplied measurements): a rectangular
# frame the arm bolts to, floor-standing. Two "rails" run the 160cm length (left/
# right, 80cm apart); two run the 80cm width (front/back, 160cm apart). The arm base
# center sits inside this footprint, off-center, at the given offsets from the left
# and back rails - NOT at the frame's own center. Orientation in the base frame isn't
# separately known, so the frame is squared to the wait-pose azimuth (self.wait_az):
# the length rails (back->front) run exactly along it, matching how a frame like this
# is actually installed - square to the robot's working direction, not rotated to
# force an exact centroid hit. (An earlier version instead solved for the rotation
# that put the wait pose exactly over the frame's centroid; the arm's off-center
# mounting made that centroid direction diagonal in the frame's own local axes, so
# forcing it to match wait_az visibly tilted the rails - see docs/debug_log.md.)
# See _mount_frame_corners_base_xy().
MOUNT_FRAME_LENGTH_M = 1.60          # long rails (left/right)
MOUNT_FRAME_WIDTH_M = 0.80           # short rails (front/back)
MOUNT_FRAME_HEIGHT_M = 0.75          # frame top above the floor
MOUNT_ARM_FROM_LEFT_M = 0.52         # arm base center, across the width, from the left rail
MOUNT_ARM_FROM_BACK_M = 0.26         # arm base center, along the length, from the back rail

# The physical cardboard catch box mounted on the tool (2026-08-25 user
# measurement, replacing the earlier reach/z "catch envelope" rectangle that
# used to occupy this spot on the side panel - that rectangle was the arm's
# abstract feasibility limits, not the box, and was mislabeled as one).
# Side-view profile only (what the box looks like face-on in ax_side):
# BOX_WIDTH_M along the approach direction, BOX_HEIGHT_M vertically - centered
# on the wait pose, since that's where the box sits at rest.
BOX_WIDTH_M = 0.15
BOX_HEIGHT_M = 0.13

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


def _forward(x, y, wait_az: float):
    """Signed distance along the wait-pose azimuth direction, for the ax_side
    panel. A linear projection (x*cos+y*sin), not radial distance
    (hypot(x,y)) - radial distance folds a ball's path back on itself
    whenever it passes near the base's vertical axis, which reads as the
    trajectory "curving backward" even though the underlying motion is a
    straight-ish line. This stays linear, so ax_side shows a real,
    unfolded view of the throw instead of a derived reach metric."""
    return x * np.cos(wait_az) + y * np.sin(wait_az)


def _mount_frame_corners_base_xy(wait_az: float) -> np.ndarray:
    """4 corners of the aluminium mount frame's footprint, in base-frame X,Y
    (meters), with the arm base at the origin - order: left-back, right-back,
    right-front, left-front (closed by re-adding corner 0 at draw time).

    2026-08-25: squared to wait_az (see the module comment above this
    function for the earlier centroid-solve attempt this replaced). User
    flagged that as still visibly tilted in the top-down plot - wait_az
    isn't exactly -90 degrees, so squaring to it left a few degrees of tilt.
    Squared to straight down (base -Y, azimuth -90 degrees) instead - `wait_az`
    is kept as a parameter (still used elsewhere in this file) but no longer
    drives this rotation."""
    arm_local = np.array([MOUNT_ARM_FROM_LEFT_M, MOUNT_ARM_FROM_BACK_M])
    straight_down_az = -np.pi / 2
    theta = straight_down_az - np.pi / 2  # local +length axis (0, 1) -> straight down
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    corners_local = np.array([
        [0.0, 0.0],
        [MOUNT_FRAME_WIDTH_M, 0.0],
        [MOUNT_FRAME_WIDTH_M, MOUNT_FRAME_LENGTH_M],
        [0.0, MOUNT_FRAME_LENGTH_M],
    ])
    return (corners_local - arm_local) @ rot.T


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
        self.fig.patch.set_facecolor(PAGE_PLANE)
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.fig.canvas.manager.set_window_title("catch.py - throw plot")
        self.fig.subplots_adjust(top=0.80, bottom=0.03)
        gs = self.fig.add_gridspec(2, 2, height_ratios=[10, 2], hspace=0.32, wspace=0.28)
        self.ax_top = self.fig.add_subplot(gs[0, 0])
        self.ax_side = self.fig.add_subplot(gs[0, 1])
        ax_legend = self.fig.add_subplot(gs[1, :])
        ax_legend.axis("off")
        ax_legend.set_facecolor(PAGE_PLANE)

        self.ax_top.set_title("Top-down (base X–Y)", color=INK_PRIMARY, family="monospace", fontsize=10)
        self.ax_top.set_xlabel("x (m)")
        self.ax_top.set_ylabel("y (m)")
        self.ax_top.set_aspect("equal", adjustable="datalim")
        self.ax_side.set_title("Side view (along approach direction – height)", color=INK_PRIMARY,
                               family="monospace", fontsize=10)
        self.ax_side.set_xlabel("distance from base, along wait-pose direction (m)")
        self.ax_side.set_ylabel("z (m, base frame)")
        # Equal aspect on ax_side too (ax_top already had it) - without this the
        # envelope rectangle (a real 0.75m-reach x 0.43m-tall box) gets stretched
        # to whatever ratio best fills the panel for THIS throw's own data range,
        # so the same fixed real-world box can look tall on one throw and wide on
        # another. Locking the aspect makes it always render at its true, wider-
        # than-tall proportions regardless of what data happens to be in frame.
        self.ax_side.set_aspect("equal", adjustable="datalim")
        for ax in (self.ax_top, self.ax_side):
            ax.margins(0.15)
            ax.set_facecolor(CHART_SURFACE)
            ax.grid(True, linewidth=0.5, alpha=0.6, color=GRIDLINE)
            ax.xaxis.label.set_color(INK_SECONDARY)
            ax.yaxis.label.set_color(INK_SECONDARY)
            ax.tick_params(colors=INK_MUTED, labelsize=8)
            for spine_name, spine in ax.spines.items():
                if spine_name in ("top", "right"):
                    spine.set_visible(False)
                else:
                    spine.set_color(AXIS_LINE)

        self.title_text = self.fig.suptitle("catch.py — waiting for the first throw...",
                                            color=STATUS_NEUTRAL, fontsize=13, fontweight="bold",
                                            family="monospace", y=0.975)

        # Excuse callout (demo.py's explain_miss(), 2026-08-25): a plain-language
        # "why not" for the audience, replacing the console print it started as -
        # a pop-up window is what the audience is actually looking at, so that's
        # where the line belongs. Speech-bubble styling (rounded box, italic,
        # quoted) reads as commentary distinct from the technical title/stats
        # lines around it. Hidden (set_visible(False) in update()) whenever
        # there's nothing to say - a catch, or a plain catch.py session that
        # never populates meta["excuse"] at all.
        self.excuse_text = self.fig.text(0.5, 0.90, "", ha="center", va="top", fontsize=11.5,
                                         family="sans-serif", style="italic", color=INK_PRIMARY,
                                         wrap=True,
                                         bbox=dict(boxstyle="round,pad=0.5", facecolor=CHART_SURFACE,
                                                  edgecolor=AXIS_LINE, linewidth=1))
        self.excuse_text.set_visible(False)

        # Session scoreboard (2026-08-25): a persistent running tally, not a
        # per-throw stat, so it lives in its own corner badge rather than the
        # stats line that gets fully overwritten every throw. Text set once
        # here so the badge chrome is visible even before the first throw
        # resolves, same "the UI already looks finished at startup" idea as
        # the title's own "waiting for the first throw..." placeholder.
        self.score_text = self.fig.text(0.985, 0.94, "CATCHES  0 / 0", ha="right", va="top",
                                        fontsize=11, family="monospace", fontweight="bold",
                                        color=INK_PRIMARY,
                                        bbox=dict(boxstyle="round,pad=0.45", facecolor=CHART_SURFACE,
                                                 edgecolor=ROBOT_COLOR, linewidth=1.4))

        self.stats_text = self.fig.text(0.5, 0.855, "", ha="center", va="top", fontsize=8,
                                        family="monospace", color=INK_SECONDARY)

        # --- static geometry (catch envelope, wait pose, base) - drawn once, ---
        # --- never touched again: these don't change throw to throw. ---
        reach_min, reach_max = envelope["reach_min"], envelope["reach_max"]
        wait_az = float(np.arctan2(self.wait_xyz[1], self.wait_xyz[0]))
        self.wait_az = wait_az  # reused in update() to project x,y onto this direction for ax_side
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
        wait_fwd = _forward(self.wait_xyz[0], self.wait_xyz[1], wait_az)
        # The box (see BOX_WIDTH_M/BOX_HEIGHT_M above) - solid, not dashed, same
        # "real physical structure" treatment as the mount frame below, centered
        # on the wait pose since that's where it sits at rest.
        self.ax_side.add_patch(plt.Rectangle(
            (wait_fwd - BOX_WIDTH_M / 2, self.wait_xyz[2] - BOX_HEIGHT_M / 2),
            BOX_WIDTH_M, BOX_HEIGHT_M,
            fill=False, linestyle="-", edgecolor=FRAME_COLOR, linewidth=1.3, zorder=1))
        self.ax_side.plot(wait_fwd, self.wait_xyz[2], "s", color=WAIT_COLOR, markersize=8, zorder=6)

        # Mount frame - real aluminium structure the arm bolts to, not a derived
        # limit, so solid FRAME_COLOR rather than envelope's dashed ENVELOPE_COLOR.
        # Base-frame z=0 is the mounting surface (frame top) by UR convention, so
        # the floor is at z=-MOUNT_FRAME_HEIGHT_M.
        frame_corners = _mount_frame_corners_base_xy(wait_az)
        fx = np.append(frame_corners[:, 0], frame_corners[0, 0])
        fy = np.append(frame_corners[:, 1], frame_corners[0, 1])
        self.ax_top.plot(fx, fy, "-", color=FRAME_COLOR, linewidth=1.3, zorder=1)
        frame_fwd = _forward(frame_corners[:, 0], frame_corners[:, 1], wait_az)
        floor_z = -MOUNT_FRAME_HEIGHT_M
        self.ax_side.add_patch(plt.Rectangle((frame_fwd.min(), floor_z),
                                             frame_fwd.max() - frame_fwd.min(), MOUNT_FRAME_HEIGHT_M,
                                             fill=False, linestyle="-", edgecolor=FRAME_COLOR,
                                             linewidth=1.3, zorder=1))

        # --- dynamic artists, pre-allocated once, mutated every update() ---
        # Ball and arm paths are LineCollections (not Line2D) so each segment can
        # carry its own color off the shared TIME_CMAP/TIME_NORM clock - solid for
        # the ball, dashed for the arm, since color no longer distinguishes them.
        #
        # Each gets a "glow" twin: a wider, low-alpha copy of the same segments/
        # colors drawn just underneath (lower zorder, added first) - a cheap way
        # to fake a neon/emissive line against the black surface (thick soft halo
        # + thin crisp core). Glow twins are mutated in lockstep with their crisp
        # counterpart in update() (same set_segments/set_array calls, same cost
        # class) - roughly doubles the per-update array-set work for these four
        # artists specifically, which is negligible next to the draw itself; see
        # module docstring for the overall per-update latency budget.
        self.ball_glow_top = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                            linewidths=7.0, alpha=0.35, capstyle="round", zorder=2)
        self.ax_top.add_collection(self.ball_glow_top)
        self.ball_glow_side = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                             linewidths=7.0, alpha=0.35, capstyle="round", zorder=2)
        self.ax_side.add_collection(self.ball_glow_side)
        self.ball_line_top = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                            linewidths=2.2, capstyle="round", zorder=3)
        self.ax_top.add_collection(self.ball_line_top)
        self.ball_line_side = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                             linewidths=2.2, capstyle="round", zorder=3)
        self.ax_side.add_collection(self.ball_line_side)
        (self.ball_top_start,) = self.ax_top.plot([], [], "o", color=BALL_COLOR, markersize=7, zorder=4,
                                                  markerfacecolor="white", markeredgewidth=1.6)
        (self.ball_top_end,) = self.ax_top.plot([], [], "x", color=BALL_COLOR, markersize=10,
                                                markeredgewidth=2.4, zorder=4)
        (self.ball_side_start,) = self.ax_side.plot([], [], "o", color=BALL_COLOR, markersize=7, zorder=4,
                                                    markerfacecolor="white", markeredgewidth=1.6)
        (self.ball_side_end,) = self.ax_side.plot([], [], "x", color=BALL_COLOR, markersize=10,
                                                  markeredgewidth=2.4, zorder=4)

        self.arm_glow_top = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                           linewidths=7.5, alpha=0.30, capstyle="round", zorder=2)
        self.ax_top.add_collection(self.arm_glow_top)
        self.arm_glow_side = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                            linewidths=7.5, alpha=0.30, capstyle="round", zorder=2)
        self.ax_side.add_collection(self.arm_glow_side)
        self.arm_line_top = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                           linewidths=2.4, linestyles="--", capstyle="round", zorder=3)
        self.ax_top.add_collection(self.arm_line_top)
        self.arm_line_side = LineCollection([], cmap=TIME_CMAP, norm=TIME_NORM,
                                            linewidths=2.4, linestyles="--", capstyle="round", zorder=3)
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
                                                   marker="D", s=[], edgecolors=INK_PRIMARY,
                                                   linewidths=1.1, zorder=6)
        self.event_scat_side = self.ax_side.scatter([], [], c=[], cmap=TIME_CMAP, norm=TIME_NORM,
                                                     marker="D", s=[], edgecolors=INK_PRIMARY,
                                                     linewidths=1.1, zorder=6)

        sm = plt.cm.ScalarMappable(cmap=TIME_CMAP, norm=TIME_NORM)
        sm.set_array([])
        cbar = self.fig.colorbar(sm, ax=[self.ax_top, self.ax_side], orientation="horizontal",
                                 fraction=0.05, pad=0.12, aspect=40)
        cbar.set_label("time through this throw's flight (release -> impact) — "
                       "shared by the ball path, arm path, and move markers",
                       fontsize=8, color=INK_SECONDARY)
        cbar.ax.tick_params(labelsize=7, colors=INK_MUTED)
        cbar.outline.set_edgecolor(AXIS_LINE)

        # Every marker that actually appears on the axes gets its own entry here -
        # a marker with nothing in the legend is what caused the "what's the blue
        # cross / blue diamond" confusion (2026-07-28): the X and the plain circle
        # markers had no legend line at all, and the diamond's swatch was a flat
        # gray that didn't warn the reader its real color means "time", not
        # "identity" (a diamond from early in a throw renders in plasma's blue/
        # violet start - easy to mistake for a fixed "ball-blue" color, since
        # BALL_COLOR is also blue). Fix: label every marker, and spell out
        # "color = time" wherever a time-colored swatch could otherwise read as
        # an identity color.
        commit_swatch = TIME_CMAP(0.12)
        reaim_swatch = TIME_CMAP(0.75)
        handles = [
            Line2D([0], [0], color=NEUTRAL_INK, lw=2.4, ls="-", label="ball path (color = time, see bar below)"),
            Line2D([0], [0], color=NEUTRAL_INK, lw=2.6, ls="--", label="arm TCP path (dashed; same time scale)"),
            Line2D([0], [0], marker="o", color="none", markerfacecolor="white", markeredgecolor=BALL_COLOR,
                  markeredgewidth=1.6, markersize=8, label="release (ball, first sample)"),
            Line2D([0], [0], marker="x", color=BALL_COLOR, markeredgewidth=2.2, markersize=9,
                  label="ball last seen (flight end)"),
            Line2D([0], [0], marker="o", color="none", markerfacecolor=ROBOT_COLOR, markersize=8,
                  label="arm TCP, end of throw"),
            Line2D([0], [0], marker="D", color="none", markerfacecolor=commit_swatch, markeredgecolor=INK_PRIMARY,
                  markeredgewidth=1.1, markersize=10, label="commit (large ◆, color = time)"),
            Line2D([0], [0], marker="D", color="none", markerfacecolor=reaim_swatch, markeredgecolor=INK_PRIMARY,
                  markeredgewidth=1.1, markersize=6, label="re-aim / retarget (small ◆, color = time)"),
            Line2D([0], [0], color=GUESS_LINE_COLOR, lw=1, ls=":", marker="*", markersize=10,
                  label="predicted catch point (converging guesses)"),
            Line2D([0], [0], marker="s", color="none", markerfacecolor=WAIT_COLOR, markersize=8, label="wait pose"),
            Line2D([0], [0], marker="^", color="none", markerfacecolor=BASE_COLOR, markersize=8, label="robot base"),
            Line2D([0], [0], color=ENVELOPE_COLOR, lw=1, ls="--", label="catch envelope"),
            Line2D([0], [0], color=FRAME_COLOR, lw=1.3, ls="-", label="box (side view, at wait pose)"),
            Line2D([0], [0], color=FRAME_COLOR, lw=1.3, ls="-", label="mount frame (aluminium, to floor)"),
        ]
        ax_legend.legend(handles=handles, loc="center", ncol=4, labelcolor=INK_SECONDARY,
                        frameon=False, fontsize=8, handlelength=1.8, columnspacing=1.3, labelspacing=1.1)

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

        excuse = meta.get("excuse")
        if excuse:
            self.excuse_text.set_text(f"Excuse for not getting the ball: {excuse}")
            self.excuse_text.set_color(status_color)
            self.excuse_text.get_bbox_patch().set_edgecolor(status_color)
            self.excuse_text.set_visible(True)
        else:
            self.excuse_text.set_visible(False)

        session_catches = meta.get("session_catches")
        session_attempts = meta.get("session_attempts")
        if session_catches is not None and session_attempts is not None:
            pct = f"  ({100 * session_catches / session_attempts:.0f}%)" if session_attempts else ""
            self.score_text.set_text(f"CATCHES  {session_catches} / {session_attempts}{pct}")

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
            b_fwd = _forward(bx, by, self.wait_az)
            seg_colors = fracs[:-1] if len(fracs) > 1 else np.zeros(0)
            top_segs, side_segs = _segments(bx, by), _segments(b_fwd, bz)
            for line in (self.ball_line_top, self.ball_glow_top):
                line.set_segments(top_segs)
                line.set_array(seg_colors)
            for line in (self.ball_line_side, self.ball_glow_side):
                line.set_segments(side_segs)
                line.set_array(seg_colors)
            self.ball_top_start.set_data([bx[0]], [by[0]])
            self.ball_top_end.set_data([bx[-1]], [by[-1]])
            self.ball_side_start.set_data([b_fwd[0]], [bz[0]])
            self.ball_side_end.set_data([b_fwd[-1]], [bz[-1]])
        else:
            for line in (self.ball_line_top, self.ball_glow_top, self.ball_line_side, self.ball_glow_side):
                line.set_segments(np.empty((0, 2, 2)))
                line.set_array(np.zeros(0))
            for artist in (self.ball_top_start, self.ball_top_end,
                          self.ball_side_start, self.ball_side_end):
                artist.set_data([], [])

        if tcp_trace:
            tcp_xyz = np.array([p for _n, p in tcp_trace], dtype=float)
            tcp_fracs = np.array([_n_to_frac(n, fracs) for n, _p in tcp_trace])
            tx, ty, tz = tcp_xyz[:, 0], tcp_xyz[:, 1], tcp_xyz[:, 2]
            t_fwd = _forward(tx, ty, self.wait_az)
            arm_seg_colors = tcp_fracs[:-1] if len(tcp_fracs) > 1 else np.zeros(0)
            arm_top_segs, arm_side_segs = _segments(tx, ty), _segments(t_fwd, tz)
            for line in (self.arm_line_top, self.arm_glow_top):
                line.set_segments(arm_top_segs)
                line.set_array(arm_seg_colors)
            for line in (self.arm_line_side, self.arm_glow_side):
                line.set_segments(arm_side_segs)
                line.set_array(arm_seg_colors)
            self.arm_top_end.set_data([tx[-1]], [ty[-1]])
            self.arm_side_end.set_data([t_fwd[-1]], [tz[-1]])
        else:
            for line in (self.arm_line_top, self.arm_glow_top, self.arm_line_side, self.arm_glow_side):
                line.set_segments(np.empty((0, 2, 2)))
                line.set_array(np.zeros(0))
            self.arm_top_end.set_data([], [])
            self.arm_side_end.set_data([], [])

        if guesses:
            gxyz = np.array([g["point"] for g in guesses], dtype=float)
            g_fracs = np.array([_n_to_frac(g["n"], fracs) for g in guesses])
            gx, gy, gz = gxyz[:, 0], gxyz[:, 1], gxyz[:, 2]
            g_fwd = _forward(gx, gy, self.wait_az)
            self.guess_line_top.set_data(gx, gy)
            self.guess_line_side.set_data(g_fwd, gz)
            self.guess_scat_top.set_offsets(np.column_stack([gx, gy]))
            self.guess_scat_top.set_array(g_fracs)
            self.guess_scat_side.set_offsets(np.column_stack([g_fwd, gz]))
            self.guess_scat_side.set_array(g_fracs)
            self.guess_star_top.set_data([gx[-1]], [gy[-1]])
            self.guess_star_side.set_data([g_fwd[-1]], [gz[-1]])

            if len(ball_base):
                idxs = np.clip(np.array([g["n"] for g in guesses], dtype=int) - 1, 0, len(ball_base) - 1)
                ex, ey, ez = ball_base[idxs, 0], ball_base[idxs, 1], ball_base[idxs, 2]
                e_fwd = _forward(ex, ey, self.wait_az)
                sizes = np.array([130 if g.get("kind") == "commit" else 55 for g in guesses])
                self.event_scat_top.set_offsets(np.column_stack([ex, ey]))
                self.event_scat_top.set_array(g_fracs)
                self.event_scat_top.set_sizes(sizes)
                self.event_scat_side.set_offsets(np.column_stack([e_fwd, ez]))
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

        # ax_top's catch-envelope wedge (circles + azimuth rays) is symmetric
        # left-right about the base origin's x=0 line by construction, but a
        # lopsided throw (release point far off to one side) pulls autoscale's x
        # range unevenly, making the wedge look off-center even though it isn't.
        # Re-center x on 0 (keep the same span, just balanced) - doesn't clip
        # anything, just repositions the same view. Only x needs this: y doesn't
        # need to be symmetric for the wedge to look symmetric, and forcing both
        # fights the equal-aspect autoscaling above (ax.set_aspect("equal",
        # adjustable="datalim") recomputes y from x's span on its own at draw
        # time - setting y explicitly here just gets silently overridden anyway).
        xlim = self.ax_top.get_xlim()
        rx = max(abs(xlim[0]), abs(xlim[1]))
        self.ax_top.set_xlim(-rx, rx)

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
