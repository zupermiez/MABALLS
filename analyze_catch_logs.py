"""
Offline analysis of catch.py --record JSONL logs - the tool behind the 2026-07-17
night-shift review (docs/debug_log.md 2026-07-17, docs/architecture_review_2026-07-17.md).

Reads one or more catch_logs/catch_log_*.jsonl files (default: all non-dry-run logs)
and prints, per the full set:
  - session/throw/commit/fault counts and the commit-verdict mix
  - estimated catch rate (last-seen-near-tool heuristic - same one catch.py now
    prints live as `caught_guess`)
  - fault rate vs commit-target azimuth swing from the wait pose (the movel
    side-throw C153A0 signature - see CLAUDE.md Key safety rules)
  - fault rate vs catch accel/speed and vs --catch-move kind (for movej-vs-movel
    comparison sessions)
  - prediction-accuracy-vs-sample-count table, replaying each recorded trajectory's
    quadratic fit against its own recorded catch-plane crossing (only throws that
    visibly crossed the plane can be scored - caught balls occlude first)
  - commit-time margins/distances and the re-aim events actually fired

Usage:
  python3 analyze_catch_logs.py                      # all recorded real sessions
  python3 analyze_catch_logs.py catch_logs/catch_log_20260717_*.jsonl   # a subset
"""

import glob
import json
import math
import sys
from collections import defaultdict

import numpy as np

from frames import mocap_point_to_base

CAUGHT_LAST_SEEN_DIST_M = 0.30  # keep in sync with catch.py's tally heuristic
AZIMUTH_BINS = [(0, 10), (10, 20), (20, 35), (35, 60), (60, 180)]
ACCURACY_NS = [30, 40, 50, 67, 80, 100]


def load_events(path):
    events = []
    with open(path) as f:
        for line in f:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events


def true_crossing(raw, catch_value):
    """(t, mocap point) where the recorded trajectory actually crossed the catch
    plane (descending), or None - only observable for balls that stayed visible."""
    for i in range(1, len(raw)):
        y0, y1 = raw[i - 1][2], raw[i][2]
        if y0 > catch_value >= y1:
            f = (y0 - catch_value) / (y0 - y1)
            p0 = np.array(raw[i - 1][1:4])
            p1 = np.array(raw[i][1:4])
            return raw[i - 1][0] + f * (raw[i][0] - raw[i - 1][0]), p0 + f * (p1 - p0)
    return None


def predict_crossing(raw, catch_value, after_t):
    """Same free per-axis quadratic + plane solve catch.py uses, on a sample prefix."""
    ts = np.array([r[0] for r in raw])
    t0 = raw[0][0]
    tt = ts - t0
    P = np.array([r[1:4] for r in raw])
    coef = [np.polyfit(tt, P[:, i], 2) for i in range(3)]
    c2, c1, c0 = coef[1]
    a, b, c = c2, c1, c0 - catch_value
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    roots = [(-b - disc**0.5) / (2 * a), (-b + disc**0.5) / (2 * a)]
    cands = [r for r in roots if t0 + r >= after_t]
    if not cands:
        return None
    dt = min(cands)
    return np.array([np.polyval(cc, dt) for cc in coef])


def pct(a, q):
    return a[min(len(a) - 1, int(len(a) * q))]


def main():
    paths = sys.argv[1:] or sorted(glob.glob("catch_logs/catch_log_*.jsonl"))
    with open("T_base_from_mocap.json") as f:
        tf = json.load(f)
    R, t_vec = np.array(tf["R"]), np.array(tf["t"])

    sessions = 0
    throws = commits = faults = guards = reaims = 0
    verdicts = defaultdict(int)
    move_kinds = defaultdict(lambda: [0, 0])   # kind -> [commits, fault-attributed]
    accel_kinds = defaultdict(lambda: [0, 0])
    az_bins = defaultdict(lambda: [0, 0])
    caught = missed = 0
    margins, dists, t_impacts, commit_ns = [], [], [], []
    acc_errs = defaultdict(list)

    for path in paths:
        events = load_events(path)
        run = next((e for e in events if e["ev"] == "run_start"), None)
        if run is None or run.get("dry_run"):
            continue
        sessions += 1
        catch_value = run.get("catch_value")
        wait = run.get("wait_pose", [0] * 6)[:3]
        az_wait = math.atan2(wait[1], wait[0])
        accel = run.get("catch_accel")
        fault_walls = [e.get("last_known_normal_wall", e["wall"])
                       for e in events if e["ev"] == "fault"]
        faults += len(fault_walls)

        by_throw = defaultdict(lambda: defaultdict(list))
        for e in events:
            by_throw[e["throw"]][e["ev"]].append(e)

        for tn, evs in sorted(by_throw.items()):
            if tn == 0 or not evs.get("throw_start"):
                continue
            throws += 1
            guards += len(evs.get("guard", []))
            reaims += len(evs.get("reaim", []))
            commit = evs["commit"][0] if evs.get("commit") else None
            end = evs["throw_end"][0] if evs.get("throw_end") else None
            samp = evs["throw_samples"][0] if evs.get("throw_samples") else None
            raw = samp.get("raw", []) if samp else []

            if raw and catch_value is not None and len(raw) >= 60:
                tc = true_crossing(raw, catch_value)
                if tc is not None:
                    t_true, p_true = tc
                    for N in ACCURACY_NS:
                        if len(raw) < N or raw[N - 1][0] >= t_true:
                            continue
                        pred = predict_crossing(raw[:N], catch_value, raw[N - 1][0])
                        if pred is not None:
                            acc_errs[N].append(float(np.linalg.norm(pred - p_true)))

            if commit:
                commits += 1
                verdicts[commit.get("verdict")] += 1
                kind = commit.get("move_kind", "movel")
                tgt = commit.get("target_pose", [None] * 3)[:3]
                cw = commit["wall"]
                faulted = any(cw - 1 <= fw <= cw + 4 for fw in fault_walls)
                move_kinds[kind][0] += 1
                move_kinds[kind][1] += faulted
                accel_kinds[accel][0] += 1
                accel_kinds[accel][1] += faulted
                if tgt[0] is not None:
                    az = math.atan2(tgt[1], tgt[0])
                    d_az = math.degrees(abs(math.atan2(math.sin(az - az_wait),
                                                       math.cos(az - az_wait))))
                    for lo, hi in AZIMUTH_BINS:
                        if lo <= d_az < hi:
                            az_bins[(lo, hi)][0] += 1
                            az_bins[(lo, hi)][1] += faulted
                # commit-tick stats: the tick at/just before the commit timestamp
                ct = commit.get("t")
                tick = None
                for tk in evs.get("tick", []):
                    if tk.get("t") is not None and ct is not None and tk["t"] <= ct + 1e-9:
                        tick = tk
                if tick:
                    if tick.get("margin") is not None:
                        margins.append(tick["margin"])
                    if tick.get("dist_to_go") is not None:
                        dists.append(tick["dist_to_go"])
                    if tick.get("t_impact") is not None:
                        t_impacts.append(tick["t_impact"])
                    if tick.get("n") is not None:
                        commit_ns.append(tick["n"])
                # catch classification (prefer catch.py's own live guess when present)
                if end is not None:
                    cg = end.get("caught_guess")
                    if cg is None and raw:
                        last = mocap_point_to_base(np.array(raw[-1][1:4]), R, t_vec)
                        arm = end.get("arm_tcp_at_end")
                        if arm:
                            cg = float(np.linalg.norm(last - np.array(arm[:3]))) < CAUGHT_LAST_SEEN_DIST_M
                    if cg is True:
                        caught += 1
                    elif cg is False:
                        missed += 1

    print(f"sessions={sessions} throws={throws} commits={commits} "
          f"({100 * commits / max(1, throws):.0f}%) faults={faults} guards={guards} reaims={reaims}")
    print(f"commit verdicts: {dict(verdicts)}")
    if caught + missed:
        print(f"catch rate on classifiable commits: {caught}/{caught + missed} "
              f"({100 * caught / (caught + missed):.0f}%)")
    if margins:
        for name, arr in (("margin(s)", margins), ("dist(m)", dists),
                          ("t_impact(s)", t_impacts), ("n@commit", commit_ns)):
            a = sorted(arr)
            print(f"  commit {name}: min={a[0]:.3f} med={a[len(a) // 2]:.3f} "
                  f"p90={pct(a, 0.9):.3f} max={a[-1]:.3f}")
    print("\nfault rate vs commit-target azimuth swing from wait pose:")
    for b in AZIMUTH_BINS:
        n, f = az_bins[b][0], az_bins[b][1]
        if n:
            print(f"  {b[0]:3d}-{b[1]:3d} deg: {f:2d}/{n:3d} ({100 * f / n:3.0f}%)")
    print("fault rate by move kind:")
    for k, (n, f) in sorted(move_kinds.items()):
        print(f"  {k}: {f}/{n} ({100 * f / max(1, n):.0f}%)")
    print("fault rate by catch accel:")
    for k, (n, f) in sorted(accel_kinds.items(), key=lambda kv: (kv[0] is None, kv[0])):
        print(f"  accel={k}: {f}/{n} ({100 * f / max(1, n):.0f}%)")
    if acc_errs:
        print("\nprediction error vs sample count (throws with an observable crossing):")
        print("   n | med(cm) | p90(cm) | count")
        for N in ACCURACY_NS:
            e = sorted(acc_errs[N])
            if e:
                print(f" {N:3d} | {100 * e[len(e) // 2]:7.1f} | {100 * pct(e, 0.9):7.1f} | {len(e)}")


if __name__ == "__main__":
    main()
