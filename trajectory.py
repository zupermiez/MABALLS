"""
Projectile-motion trajectory fitting for a tracked ball.

Fits an independent quadratic (constant-acceleration) polynomial per axis to a
buffer of (t, x, y, z) samples via least squares, then uses the fit to predict
position at a future time or find when the ball crosses a target plane (e.g. the
robot's catch plane).

Motive's default world frame is Y-up, not Z-up like many robotics stacks - don't
assume which axis is "up" without checking Motive's actual axis convention. This
module doesn't assume it either: instead of hardcoding gravity onto one axis, it
fits acceleration on all three axes independently and reports which one is
closest to -9.81 m/s^2. That both identifies the up axis empirically and sanity
-checks the fit - real projectile motion should show ~9.81 on exactly one axis
and ~0 on the other two; if it doesn't, something's wrong (bad samples, ball not
actually in free flight, wrong units, etc).
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

G = 9.81
AXIS_NAMES = ("x", "y", "z")


@dataclass
class Sample:
    t: float
    x: float
    y: float
    z: float


# Motive "ghost marker" filter (2026-08-25, found via catch.py's forensic log
# analysis - see docs/debug_log.md 2026-08-25). Motive's rigid-body solver can
# briefly latch onto a stray reflective point once the ball's own markers are
# occluded (typically entering the catch box at the end of a real flight), and
# keeps reporting tracking_valid=True while frozen at that point - not the
# already-documented "never sighted this session" (0,0,0)+identity default,
# but a *different*, arbitrary, environment-specific fixed point (one real
# session saw two throws both end with a jump to the same frozen mocap point,
# within a few mm, well away from the ball's actual path). Any consumer that
# trusts the tail of a sample buffer at face value - a post-hoc "where was the
# ball last seen" check, or a live plot drawing the buffer as it grows - draws
# or measures against that ghost point instead of the real trajectory.
#
# A real ball at 120fps never moves more than this in one frame, even well
# above any speed thrown here, so any single-frame jump past it is Motive's
# solver re-latching onto something else, not real motion.
MAX_BALL_JUMP_M = 0.5


def trim_ghost_tail(samples: Sequence[Sample], max_jump_m: float = MAX_BALL_JUMP_M) -> List[Sample]:
    """Return `samples` with any trailing ghost-marker run trimmed off - the
    prefix up to and including the last sample before the most recent
    implausible single-frame jump (see MAX_BALL_JUMP_M), or every sample
    unchanged if no such jump exists anywhere in the tail.

    Deliberately a *display/post-hoc* filter, not something to splice into
    live release/flight-end detection: the ghost point only ever shows up
    after real flight has effectively ended (markers occluded), so it never
    affects an in-flight catch decision - only what gets drawn or measured
    against afterward. Callers that need the raw, unfiltered buffer (e.g. the
    JSONL trajectory dump `catch.py`/`demo.py` write for later forensic
    research - the ghost point itself was found by looking at that raw data)
    should keep using the untrimmed samples; this is only for what a human
    looks at live.

    Only trims a trailing run, on purpose: the only observed cause (marker
    occlusion at the end of flight, once the ball is caught/landed) only ever
    produces a tail artifact, so a single backward scan for the last
    disqualifying jump is enough - no need to hunt for jumps buried mid-flight.
    """
    if not samples:
        return []
    trusted_idx = len(samples) - 1
    for i in range(len(samples) - 1, 0, -1):
        a, b = samples[i - 1], samples[i]
        jump = math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2)
        if jump > max_jump_m:
            trusted_idx = i - 1
            break
    return list(samples[: trusted_idx + 1])


@dataclass
class AxisFit:
    p0: float
    v0: float
    a: float  # fitted acceleration

    def position(self, dt: float) -> float:
        return self.p0 + self.v0 * dt + 0.5 * self.a * dt * dt


@dataclass
class TrajectoryFit:
    t0: float  # absolute time the fit's dt=0 corresponds to
    axes: Tuple[AxisFit, AxisFit, AxisFit]  # x, y, z
    up_axis: int  # 0/1/2, whichever axis's fitted accel is closest to -G
    residual_rms: float  # fit quality across all axes, meters

    def position(self, t: float) -> Tuple[float, float, float]:
        dt = t - self.t0
        return tuple(axis.position(dt) for axis in self.axes)

    def time_of_plane_crossing(
        self, axis: int, value: float, after_t: float = 0.0
    ) -> Optional[float]:
        """Solve position(axis) == value for absolute t >= after_t.

        Returns the earliest valid crossing time, or None if the trajectory
        never reaches that value after after_t.
        """
        fit = self.axes[axis]
        a, b, c = 0.5 * fit.a, fit.v0, fit.p0 - value
        roots_dt = _solve_quadratic(a, b, c)
        candidates = [self.t0 + r for r in roots_dt if self.t0 + r >= after_t]
        return min(candidates) if candidates else None


def _solve_quadratic(a: float, b: float, c: float) -> List[float]:
    if abs(a) < 1e-12:
        if abs(b) < 1e-12:
            return []
        return [-c / b]
    disc = b * b - 4 * a * c
    if disc < 0:
        return []
    sq = disc**0.5
    return [(-b - sq) / (2 * a), (-b + sq) / (2 * a)]


def fit_trajectory(samples: Sequence[Sample]) -> TrajectoryFit:
    if len(samples) < 3:
        raise ValueError("Need at least 3 samples to fit a quadratic trajectory")

    t0 = samples[0].t
    ts = np.array([s.t - t0 for s in samples])
    coords = np.array([[s.x, s.y, s.z] for s in samples])

    axes = []
    residuals_sq_sum = 0.0
    for i in range(3):
        c2, c1, c0 = np.polyfit(ts, coords[:, i], deg=2)
        axes.append(AxisFit(p0=c0, v0=c1, a=2 * c2))
        pred = np.polyval((c2, c1, c0), ts)
        residuals_sq_sum += float(np.sum((pred - coords[:, i]) ** 2))

    up_axis = min(range(3), key=lambda i: abs(axes[i].a - (-G)))
    residual_rms = (residuals_sq_sum / (3 * len(samples))) ** 0.5

    return TrajectoryFit(t0=t0, axes=tuple(axes), up_axis=up_axis, residual_rms=residual_rms)


if __name__ == "__main__":
    # Self-test with synthetic projectile data (no live camera/ball needed yet).
    import random

    rng = random.Random(42)
    t_start = 100.0
    p0 = {"x": 0.0, "y": 1.0, "z": 0.0}
    v0 = {"x": 3.0, "y": 4.0, "z": 0.5}  # y is "up" in this synthetic throw
    noise_m = 0.0005  # ~0.5mm, roughly matching Flex 13 rigid-body accuracy
    fps = 120

    samples = []
    for i in range(30):
        dt = i / fps
        t = t_start + dt
        x = p0["x"] + v0["x"] * dt + rng.gauss(0, noise_m)
        y = p0["y"] + v0["y"] * dt - 0.5 * G * dt * dt + rng.gauss(0, noise_m)
        z = p0["z"] + v0["z"] * dt + rng.gauss(0, noise_m)
        samples.append(Sample(t, x, y, z))

    fit = fit_trajectory(samples)

    print(f"detected up_axis = {fit.up_axis} ({AXIS_NAMES[fit.up_axis]}), expected 1 (y)")
    print(f"residual_rms = {fit.residual_rms * 1000:.3f} mm")
    for name, axis in zip(AXIS_NAMES, fit.axes):
        print(f"  {name}: p0={axis.p0:+.4f} v0={axis.v0:+.4f} a={axis.a:+.4f}")

    last_t = samples[-1].t
    catch_t = fit.time_of_plane_crossing(axis=1, value=0.5, after_t=last_t)
    if catch_t is not None:
        pos = fit.position(catch_t)
        print(
            f"predicted y=0.5 crossing at t={catch_t:.4f} "
            f"({catch_t - last_t:.4f}s from last sample), pos={tuple(round(v, 4) for v in pos)}"
        )
    else:
        print("no future crossing found for y=0.5")

    assert fit.up_axis == 1, "expected y to be identified as the up axis"
    assert fit.residual_rms < 0.01, "fit residual too large"
    assert catch_t is not None

    # trim_ghost_tail: a clean flight is returned unchanged; a flight whose tail
    # instantaneously jumps to a frozen ghost point and stays there is trimmed
    # back to the last real sample before the jump.
    trimmed_clean = trim_ghost_tail(samples)
    assert trimmed_clean == samples, "clean flight should pass through unchanged"

    ghost = Sample(t=samples[-1].t + 1 / fps, x=5.0, y=5.0, z=5.0)  # far from real path
    haunted = samples + [ghost, ghost, ghost]  # solver latches on and stays there
    trimmed = trim_ghost_tail(haunted)
    assert trimmed == samples, "ghost tail should be trimmed back to the real samples"
    print("trim_ghost_tail: clean flight preserved, ghost tail trimmed")

    print("\nself-test passed")
