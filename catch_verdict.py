"""
catch_verdict.py - post-flight caught/missed judgement (2026-10-16).

The original heuristic (catch.py/demo.py CAUGHT_LAST_SEEN_DIST_M) decides at the
instant the flight ends: ball last seen <30cm from the tool => caught. That's
clean at that instant (real catches 2-16cm, misses 0.65m+ over 572 logged
throws), but blind to what happens NEXT - a ball that rattles in the box and hops
back out, or rolls off the rim, was already counted as a catch. October demo
sessions showed a lot of balls bouncing back up out of the box (see
docs/debug_log.md 2026-10-16), so this keeps watching for POST_WINDOW_S after the
flight ends and overturns the verdict on contrary evidence.

Pure logic, no I/O. The caller keeps feeding tool positions while the window is
open (RTDE FK, one cheap read per main-loop tick) and, once due(), hands over the
ball samples Motive delivered after the flight ended (SharedState.recent_samples,
appended losslessly by the frame callback - no per-tick work on the hot thread).

Rules, in order (best guess, not yet validated against ground truth):
  1. MISS  - the ball is seen, after the flight ended, >FAR_M from the tool for
             >=MIN_FAR_SAMPLES samples (escaped the box / landed elsewhere).
  2. CATCH - old rule said caught and nothing contradicts it: "seen in box" if
             post samples sit near the tool, else "vanished into box".
  3. CATCH - old rule said missed, but the ball rides along with the moving tool
             afterwards (rescues a ghost-marker or occlusion false miss).
  4. MISS  - otherwise (last seen away from the tool, nothing new).
Ghost-marker guard: Motive can latch the ball's rigid body onto a stray fixed
point once the real markers are occluded (trajectory.trim_ghost_tail). A post
sample that jumps further from the previous trusted point than a real ball could
have moved is dropped, and a perfectly static far-away run that only appeared
after a gap is treated as ghost-like and ignored for rule 1.
"""

import bisect
from typing import List, Optional, Sequence, Tuple

import numpy as np

POST_WINDOW_S = 1.2        # how long after flight end to keep watching
NEAR_M = 0.20              # ball within this of the TCP (box centroid) = in/at the box
FAR_M = 0.30               # ball beyond this of the TCP = definitely not in the box
MIN_FAR_SAMPLES = 3        # far sightings needed to overturn a catch (~25ms @120Hz)
MIN_RIDE_SAMPLES = 10      # near sightings needed to rescue an old-rule miss
MIN_RIDE_TOOL_TRAVEL_M = 0.05  # ...while the tool itself moved this far (riding, not coincidence)
MAX_JUMP_BASE_M = 0.5      # same per-frame ghost bound as trajectory.MAX_BALL_JUMP_M
MAX_BALL_SPEED = 6.0       # m/s - generous bound on a ball leaving the box
GHOST_STATIC_M = 0.005     # a far run whose spread stays under this is ghost-like...
GHOST_GAP_S = 0.05         # ...if it only appeared after at least this long unseen
UPWARD_EXIT_VZ = 0.5       # m/s - last flight samples rising this fast near the box = "unsure"


class PostFlightWatch:
    """One pending verdict. `ctx` is opaque caller state needed to announce it."""

    def __init__(self, end_t: float, start_mono: float, clock_off: Optional[float],
                 last_base: np.ndarray, tail_base: np.ndarray, tail_t: Sequence[float],
                 old_caught: bool, old_dist: float, ctx: dict):
        self.end_t = end_t                  # mocap time of the last flight sample
        self.start_mono = start_mono
        # monotonic - mocap_t, so tool readings (monotonic) can be placed on the
        # mocap clock. Falls back to "flight just ended now" if never measured.
        self.clock_off = clock_off if clock_off is not None else start_mono - end_t
        self.last_base = np.asarray(last_base, dtype=float)
        self.tail_base = np.asarray(tail_base, dtype=float)
        self.tail_t = list(tail_t)
        self.old_caught = bool(old_caught)
        self.old_dist = old_dist
        self.ctx = ctx
        self.tool_t: List[float] = []
        self.tool_xyz: List[Tuple[float, float, float]] = []

    def add_tool(self, now_mono: float, tcp_xyz) -> None:
        self.tool_t.append(now_mono - self.clock_off)
        self.tool_xyz.append(tuple(float(v) for v in tcp_xyz[:3]))

    def due(self, now_mono: float) -> bool:
        return now_mono - self.start_mono >= POST_WINDOW_S

    def _tool_at(self, t: float) -> Optional[np.ndarray]:
        if not self.tool_t:
            return None
        i = bisect.bisect_left(self.tool_t, t)
        if i <= 0:
            return np.array(self.tool_xyz[0])
        if i >= len(self.tool_t):
            return np.array(self.tool_xyz[-1])
        j = i if self.tool_t[i] - t < t - self.tool_t[i - 1] else i - 1
        return np.array(self.tool_xyz[j])

    def judge(self, post: Sequence[Tuple[float, np.ndarray]]) -> dict:
        """`post`: [(mocap_t, base_xyz)] samples seen after end_t, in time order."""
        post = [(t, np.asarray(p, dtype=float)) for t, p in post
                if self.end_t < t <= self.end_t + POST_WINDOW_S]

        # Ghost filter: chain from the last trusted flight point.
        prev_p, prev_t = self.last_base, self.end_t
        accepted, n_jump = [], 0
        for t, p in post:
            gap = t - prev_t
            if np.linalg.norm(p - prev_p) > MAX_JUMP_BASE_M + MAX_BALL_SPEED * max(gap, 0.0):
                n_jump += 1
                continue
            accepted.append((t, p, gap))
            prev_p, prev_t = p, t

        dists = []
        for t, p, gap in accepted:
            tool = self._tool_at(t)
            dists.append(float(np.linalg.norm(p - tool)) if tool is not None else None)

        far_idx = [i for i, d in enumerate(dists) if d is not None and d > FAR_M]
        near_idx = [i for i, d in enumerate(dists) if d is not None and d < NEAR_M]

        # Ghost-like far run: static, and it only appeared after a gap.
        n_ghost_static = 0
        if far_idx:
            run = np.array([accepted[i][1] for i in far_idx])
            first_gap = accepted[far_idx[0]][2]
            if len(run) >= 2 and np.ptp(run, axis=0).max() < GHOST_STATIC_M and first_gap >= GHOST_GAP_S:
                n_ghost_static = len(far_idx)
                far_idx = []

        tool_travel = 0.0
        if near_idx:
            tn = [self._tool_at(accepted[i][0]) for i in near_idx]
            tool_travel = float(np.linalg.norm(tn[-1] - tn[0])) if tn[0] is not None else 0.0

        upward_exit = False
        if len(self.tail_t) >= 4:
            dt = self.tail_t[-1] - self.tail_t[-4]
            if dt > 0:
                vz = (self.tail_base[-1][2] - self.tail_base[-4][2]) / dt
                upward_exit = bool(vz > UPWARD_EXIT_VZ and self.old_dist is not None and self.old_dist < FAR_M)

        if len(far_idx) >= MIN_FAR_SAMPLES:
            caught = False
            far_max = max(dists[i] for i in far_idx)
            why = f"ball seen {far_max:.2f}m from the box after the flight - didn't stay in"
        elif self.old_caught:
            caught = True
            if near_idx:
                why = "ball seen sitting in the box"
            elif upward_exit:
                why = "ball vanished at the box, but was rising - unsure"
            else:
                why = "ball vanished into the box"
        elif len(near_idx) >= MIN_RIDE_SAMPLES and tool_travel >= MIN_RIDE_TOOL_TRAVEL_M:
            caught = True
            why = "ball rode along with the box after the flight"
        else:
            caught = False
            why = (f"ball last seen {self.old_dist:.2f}m from the box" if self.old_dist is not None
                   else "ball not seen at the box")

        return dict(caught=caught, why=why, old_caught=self.old_caught, n_post=len(post),
                    n_accepted=len(accepted), n_jump_rejected=n_jump, n_ghost_static=n_ghost_static,
                    n_far=len(far_idx), n_near=len(near_idx), tool_travel_m=tool_travel,
                    upward_exit=upward_exit, n_tool=len(self.tool_t),
                    far_max_m=max((dists[i] for i in far_idx), default=None))


if __name__ == "__main__":
    # Offline self-test: synthetic cases for each rule.
    def mk(old_caught, old_dist=0.05, post=(), tool_path=None, rising=False):
        tail_t = [0.0, 0.01, 0.02, 0.03]
        vz = 1.0 if rising else -3.0
        tail = np.array([[0.5, 0.0, 0.3 + vz * t] for t in tail_t])
        w = PostFlightWatch(0.03, 100.0, 100.0 - 0.03, tail[-1], tail, tail_t, old_caught, old_dist, {})
        for k in range(150):
            t = 0.03 + k * 0.008
            w.add_tool(t + w.clock_off, tool_path(t) if tool_path else (0.5, 0.0, 0.3))
        return w.judge(post)

    still = lambda t: (0.5, 0.0, 0.3)
    moving = lambda t: (0.5 - 0.3 * t, 0.0, 0.3)
    in_box = [(0.03 + k / 120, np.array(moving(0.03 + k / 120))) for k in range(1, 60)]
    hop_out = [(0.03 + k / 120, np.array([0.5 + 0.02 * k, 0.0, 0.3 - 0.01 * k])) for k in range(1, 60)]
    ghost = [(0.5 + k / 120, np.array([2.0, 1.0, 0.1])) for k in range(20)]

    cases = [
        ("hidden in box", mk(True), True),
        ("seen in box, tool moving", mk(True, post=in_box, tool_path=moving), True),
        ("hopped out", mk(True, post=hop_out), False),
        ("ghost point far away", mk(True, post=ghost), True),
        ("old miss, nothing seen", mk(False, old_dist=1.3), False),
        ("old miss, rode with box", mk(False, old_dist=0.4, post=in_box, tool_path=moving), True),
        ("rising at box, vanished", mk(True, rising=True), True),
    ]
    ok = True
    for name, r, want in cases:
        good = r["caught"] == want
        ok &= good
        print(f"{'ok ' if good else 'BAD'} {name:28s} caught={r['caught']}  {r['why']}")
    raise SystemExit(0 if ok else 1)
