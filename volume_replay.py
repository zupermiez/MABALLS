"""
volume_replay.py - offline checks for demo.py's catch VOLUME (--catch-volume-height).

No robot, no Motive. Two modes:

  python3 volume_replay.py --self-test
      Synthetic throws through the real transform/wait pose: plane-only behaviour is
      unchanged at height 0, a flat throw whose plane crossing is out of reach gets
      caught higher up with a big tilt, no point on a steeply rising leg is ever
      chosen, plus timing of the solve (must stay far below a 125Hz tick).

  python3 volume_replay.py [catch_logs/*.jsonl ...]
      Re-runs every recorded throw (throw_samples) through the plane-only solver and
      the volume solver, from the arm sitting at the wait pose with the mouth up,
      at a fixed sample count (--at-n, default 40 = MIN_SAMPLES_FOR_CHECK), and
      reports how many throws each would rate feasible / possible. Transform, wait
      pose, catch plane, move-time model and margins all come from each log's own
      run_start line, except the transform: the static file a run names can be stale
      (the stand moves; live runs use the base-RB transform), so each session's real
      transform is recovered from its own logged ticks (session_transform()).
"""

import argparse
import glob
import json
import math
import sys
import time

import numpy as np

from catch_feasibility import (FeasibilityResult, MoveTimeModel, check_feasibility, load_transform,
                               MIN_SAMPLES_FOR_CHECK)
from trajectory import Sample

G = 9.81
DEG = math.pi / 180


def rotvec_to_matrix(rv):
    rv = np.asarray(rv, dtype=float)
    th = float(np.linalg.norm(rv))
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)


def volume_cfg(height, wait_xyz, max_tilt_deg=90.0, orient=True, rate_deg=110.0, accel_deg=733.3):
    cfg = {"height": height, "max_tilt_rad": max_tilt_deg * DEG, "wait_xyz": wait_xyz,
           "max_azimuth_deg": 75.0, "zero_dist": 0.40, "full_dist": 0.60}   # demo.py defaults
    if orient:
        cfg.update(mouth_normal_now=np.array([0.0, 0.0, 1.0]), orient_rate=rate_deg * DEG,
                   orient_accel=accel_deg * DEG)
    return cfg


def run(samples, catch_value, R, t_vec, tcp, model, margin, box, volume):
    return check_feasibility(samples, 1, catch_value, R, t_vec, tcp, model, 0.55, 1.20, margin, box,
                             cache={}, volume=volume)


# --------------------------------------------------------------------------- self-test

def synth(R, t_vec, p0_base, v0_base, n=40, dt=1 / 120):
    """Ballistic samples generated in the BASE frame (Z up), handed over in mocap."""
    out = []
    for i in range(n):
        tt = i * dt
        p = np.array(p0_base) + np.array(v0_base) * tt + np.array([0, 0, -0.5 * G * tt * tt])
        pm = R.T @ (p - t_vec)
        out.append(Sample(100.0 + tt, float(pm[0]), float(pm[1]), float(pm[2])))
    return out


def synth_through(R, t_vec, release_xy, hit, T, vz_hit, n=40):
    """Throw from release_xy that passes through base point `hit` T seconds after
    the first sample, moving vertically at vz_hit there."""
    hit = np.asarray(hit, dtype=float)
    vz0 = vz_hit + G * T
    z0 = hit[2] - vz0 * T + 0.5 * G * T * T
    vxy = (hit[:2] - np.asarray(release_xy, dtype=float)) / T
    return synth(R, t_vec, [release_xy[0], release_xy[1], z0], [vxy[0], vxy[1], vz0], n=n)


def self_test(transform_file):
    R, t_vec, _, _ = load_transform(transform_file)
    wait = np.array([0.1583, -0.5783, 0.2267])
    catch_value = float((R.T @ (wait - t_vec))[1])        # mocap-Y of the plane through the wait height
    model = MoveTimeModel(accel=4.5576, latency=0.1116, v_max=1.3, residual_rms=0.055, n_legs=40)
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok &= bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {name} {detail}")

    # (1) height 0 == old behaviour, bit for bit.
    # Lob: released ~0.46m above the plane, lands on it 10cm from the wait point after 1.4s.
    lob = synth_through(R, t_vec, [0.3, -2.8], wait + [0.0, -0.1, 0.0], 1.4, -7.2)
    a = run(lob, catch_value, R, t_vec, wait, model, 0.03, 0.15, None)
    b = run(lob, catch_value, R, t_vec, wait, model, 0.03, 0.15, volume_cfg(0.0, wait))
    check("height 0 identical to plane-only", a.crossing_t is not None
          and np.allclose(a.catch_point_base, b.catch_point_base) and a.margin == b.margin and b.vol_dz is None)

    # (2) lofted throw landing near the wait pose: plane feasible -> volume must be too
    # (its margin also pays for wrist-tilt time, which the plane-only number doesn't).
    v = run(lob, catch_value, R, t_vec, wait, model, 0.03, 0.15, volume_cfg(0.5, wait))
    check("lofted throw: plane feasible and volume feasible", a.feasible and v.feasible,
          f"(plane {a.margin:+.3f}s at {np.round(a.catch_point_base, 2)}, volume {v.margin:+.3f}s "
          f"+{v.vol_dz * 100:.0f}cm tilt {v.tilt_deg:.0f}deg)")

    # (3) flatter throw (3 m/s horizontal) released 0.5m above the plane from 3m out:
    # passes 0.3m above the wait point after 1.0s, then reaches the plane ~16cm further
    # in toward the base - inside the 0.55m reach floor, so the plane can't take it.
    flat = synth_through(R, t_vec, [0.1583, -3.578], wait + [0.0, 0.0, 0.3], 1.0, -5.4)
    p = run(flat, catch_value, R, t_vec, wait, model, 0.03, 0.15, None)
    v = run(flat, catch_value, R, t_vec, wait, model, 0.03, 0.15, volume_cfg(0.5, wait))
    check("flat throw: plane unreachable or infeasible", (not p.reachable) or (not p.feasible),
          f"(plane reach {p.reach:.2f}m margin {p.margin:+.3f}s)" if p.crossing_t else "(no crossing)")
    check("flat throw: volume catches it higher, tilted", v.vol_dz is not None and v.vol_dz > 0.03
          and v.tilt_deg > 15 and (v.feasible or v.possible),
          f"(+{(v.vol_dz or 0) * 100:.0f}cm tilt {(v.tilt_deg or 0):.0f}deg margin {v.margin:+.3f}s "
          f"reach {v.reach:.2f}m)")
    check("chosen point inside the slab", v.catch_point_base is not None
          and wait[2] - 0.02 <= v.catch_point_base[2] <= wait[2] + 0.52)

    # (4) never choose a point while the ball is still climbing steeper than the tilt cap.
    up = synth(R, t_vec, [0.2, -1.4, 0.25], [0.0, 1.0, 4.0], n=10)
    v = run(up, catch_value, R, t_vec, wait, model, 0.03, 0.15, volume_cfg(0.5, wait))
    check("rising ball: chosen impact velocity is not climbing", v.impact_vel_base is None
          or v.impact_vel_base[2] <= 1e-6, f"(vz={None if v.impact_vel_base is None else round(float(v.impact_vel_base[2]), 2)})")
    v45 = run(flat, catch_value, R, t_vec, wait, model, 0.03, 0.15, volume_cfg(0.5, wait, max_tilt_deg=45))
    check("tilt cap respected", v45.tilt_deg is None or v45.tilt_deg <= 45 + 1e-6)

    # (5) hysteresis: identical inputs twice with a shared cache keep the same choice.
    cache = {}
    r1 = check_feasibility(flat, 1, catch_value, R, t_vec, wait, model, 0.55, 1.2, 0.03, 0.15, cache=cache,
                           volume=volume_cfg(0.5, wait))
    r2 = check_feasibility(flat, 1, catch_value, R, t_vec, wait + 0.01, model, 0.55, 1.2, 0.03, 0.15,
                           cache=cache, volume=volume_cfg(0.5, wait))
    check("hysteresis keeps the choice under a 1cm arm move", r1.crossing_t == r2.crossing_t)

    # (5b) height near the base: a throw passing 0.3m up at only 0.38m from the base
    # axis must not be taken that high (inside zero_dist 0.40: no height allowed).
    near = synth_through(R, t_vec, [0.12, -3.4], np.array([0.1, -0.37, wait[2] + 0.3]), 1.0, -5.4)
    v = run(near, catch_value, R, t_vec, wait, model, 0.03, 0.15, volume_cfg(0.5, wait))
    ok_near = v.catch_point_base is None or (
        v.catch_point_base[2] - wait[2] <= 0.5 * np.clip((np.hypot(*v.catch_point_base[:2]) - 0.40) / 0.20, 0, 1) + 0.02)
    check("height limited near the base", ok_near,
          "(no point)" if v.catch_point_base is None else
          f"(chose +{(v.catch_point_base[2] - wait[2]) * 100:.0f}cm at {np.hypot(*v.catch_point_base[:2]):.2f}m "
          f"from base{'' if v.vol_dz is not None else ', plane fallback'})")

    # (6) timing: full solve (new sample) and cached per-tick choice.
    reps = 300
    t0 = time.perf_counter()
    for _ in range(reps):
        run(flat, catch_value, R, t_vec, wait, model, 0.03, 0.15, None)
    plane_ms = (time.perf_counter() - t0) / reps * 1e3
    t0 = time.perf_counter()
    for _ in range(reps):
        run(flat, catch_value, R, t_vec, wait, model, 0.03, 0.15, volume_cfg(0.5, wait))
    vol_ms = (time.perf_counter() - t0) / reps * 1e3
    cache = {}
    cfg = volume_cfg(0.5, wait)
    check_feasibility(flat, 1, catch_value, R, t_vec, wait, model, 0.55, 1.2, 0.03, 0.15, cache=cache, volume=cfg)
    t0 = time.perf_counter()
    for _ in range(reps):
        check_feasibility(flat, 1, catch_value, R, t_vec, wait, model, 0.55, 1.2, 0.03, 0.15, cache=cache,
                          volume=cfg)
    tick_ms = (time.perf_counter() - t0) / reps * 1e3
    check("solve time", vol_ms < 1.0 and tick_ms < 0.3,
          f"(plane {plane_ms:.3f}ms, volume new-sample {vol_ms:.3f}ms, volume cached tick {tick_ms:.3f}ms; "
          f"125Hz tick = 8ms)")
    print("SELF-TEST", "PASSED" if ok else "FAILED")
    return ok


# --------------------------------------------------------------------------- replay

def kabsch(P, Q):
    """Best-fit rigid (R, t) with Q ~= P @ R.T + t."""
    pc, qc = P.mean(axis=0), Q.mean(axis=0)
    U, _, Vt = np.linalg.svd((P - pc).T @ (Q - qc))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, qc - pc @ R.T


def session_transform(path):
    """Recover the base<-mocap transform a session ACTUALLY used (live base-RB, not the
    static file it names) from its own logs: every plane-mode tick logs the base-frame
    target, and the same plane crossing recomputed from throw_samples in mocap gives the
    other half of the pair. Returns (R, t, rms_m, n_pairs) or None."""
    from trajectory import fit_trajectory
    ticks, raws, planes = [], {}, {}
    with open(path) as f:
        for line in f:
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                continue
            k = ev.get("ev")
            if k == "tick" and ev.get("target") is not None and ev.get("vol_dz") is None:
                ticks.append(ev)
            elif k == "throw_samples":
                raws[ev["throw"]] = ev.get("raw") or []
            elif k == "throw_start" and ev.get("catch_value") is not None:
                planes[ev["throw"]] = ev["catch_value"]
    P, Q = [], []
    for ev in ticks[::4]:
        raw, cv = raws.get(ev["throw"]), planes.get(ev["throw"])
        if not raw or cv is None or ev["n"] > len(raw) or ev["n"] < 3:
            continue
        smp = [Sample(*s[:4]) for s in raw[:ev["n"]]]
        fit = fit_trajectory(smp)
        tc = fit.time_of_plane_crossing(1, cv, after_t=smp[-1].t)
        if tc is None:
            continue
        P.append(fit.position(tc))
        Q.append(ev["target"])
    if len(P) < 10:
        return None
    P, Q = np.array(P), np.array(Q)
    keep = np.ones(len(P), bool)
    for _ in range(3):  # drop pairs a later tick's rounding/outlier made inconsistent
        R, t = kabsch(P[keep], Q[keep])
        err = np.linalg.norm(P @ R.T + t - Q, axis=1)
        keep = err < max(0.01, 3 * np.median(err))
    return R, t, float(np.sqrt(np.mean(err[keep] ** 2))), int(keep.sum())


def replay(paths, at_n, height, max_tilt, orient=True):
    tot = {"throws": 0, "plane_ok": 0, "plane_poss": 0, "vol_ok": 0, "vol_poss": 0, "rescued": 0,
           "lost": 0}
    dzs, tilts, gains = [], [], []
    for path in paths:
        run_start = None
        with open(path) as f:
            for line in f:
                try:
                    ev = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if ev.get("ev") == "run_start":
                    run_start = ev
                    est = session_transform(path)
                    if est is None:
                        print(f"  {path}: too few plane-mode ticks to recover its transform - skipped")
                        run_start = None
                        continue
                    R, t_vec, rms, npairs = est
                    print(f"  {path}: transform recovered from {npairs} logged ticks, rms {rms * 1000:.1f}mm")
                    wait = np.array(ev["wait_pose"][:3])
                    m = ev["move_time_model"]
                    model = MoveTimeModel(m["accel"], m["latency"], m["v_max"], m["residual_rms"], m["n_legs"])
                    margin = ev.get("margin", 0.03)
                    box = ev.get("box_radius", 0.15)
                    so = ev.get("servo") or {}
                    rate = so.get("orient_max_rate_deg_s", 110.0) if isinstance(so, dict) else 110.0
                    acc = so.get("orient_max_accel_deg_s2", 733.3) if isinstance(so, dict) else 733.3
                    plane_default = ev.get("catch_value")
                    continue
                if ev.get("ev") == "throw_start" and ev.get("catch_value") is not None:
                    plane_default = ev["catch_value"]
                    continue
                if ev.get("ev") != "throw_samples" or run_start is None:
                    continue
                raw = ev.get("raw") or []
                if len(raw) < at_n:
                    continue
                cv = plane_default
                samples = [Sample(*s[:4]) for s in raw[:at_n]]
                try:
                    p = run(samples, cv, R, t_vec, wait, model, margin, box, None)
                    v = run(samples, cv, R, t_vec, wait, model, margin, box,
                            volume_cfg(height, wait, max_tilt, orient, rate, acc))
                except Exception as e:  # degenerate fits - skip, but say so
                    print(f"  skip {path} throw {ev.get('throw')}: {e}")
                    continue
                tot["throws"] += 1
                p_ok = bool(p.crossing_t is not None and p.reachable and p.feasible)
                p_poss = bool(p.crossing_t is not None and p.reachable and (p.feasible or p.possible))
                v_ok = bool(v.crossing_t is not None and v.reachable and v.feasible)
                v_poss = bool(v.crossing_t is not None and v.reachable and (v.feasible or v.possible))
                tot["plane_ok"] += p_ok
                tot["plane_poss"] += p_poss
                tot["vol_ok"] += v_ok
                tot["vol_poss"] += v_poss
                tot["rescued"] += v_poss and not p_poss
                tot["lost"] += p_poss and not v_poss
                if v.vol_dz is not None:
                    dzs.append(v.vol_dz)
                    tilts.append(v.tilt_deg)
                    if p.margin is not None and p.reachable:
                        gains.append(v.margin - p.margin)
    n = tot["throws"]
    if n == 0:
        print("no usable throws")
        return
    print(f"{n} recorded throws, judged at n={at_n} samples from the wait pose "
          f"(volume +{height:.2f}m, tilt<={max_tilt:.0f}deg, tilt time {'counted' if orient else 'IGNORED'}):")
    print(f"  plane : feasible {tot['plane_ok']:3d} ({tot['plane_ok'] / n:4.0%})  "
          f"feasible-or-possible {tot['plane_poss']:3d} ({tot['plane_poss'] / n:4.0%})")
    print(f"  volume: feasible {tot['vol_ok']:3d} ({tot['vol_ok'] / n:4.0%})  "
          f"feasible-or-possible {tot['vol_poss']:3d} ({tot['vol_poss'] / n:4.0%})")
    print(f"  rescued by volume: {tot['rescued']}   lost vs plane: {tot['lost']}")
    if dzs:
        dz = np.array(dzs)
        print(f"  chosen height above plane: median {np.median(dz) * 100:.0f}cm, "
              f"{np.mean(dz > 0.05):.0%} above +5cm, max {dz.max() * 100:.0f}cm")
        print(f"  chosen tilt: median {np.median(tilts):.0f}deg, p90 {np.percentile(tilts, 90):.0f}deg, "
              f"max {max(tilts):.0f}deg")
    if gains:
        print(f"  margin gain vs plane (both reachable): median {np.median(gains) * 1e3:+.0f}ms")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="*", help="catch_log_*.jsonl files (default: all of catch_logs/)")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--transform-file", default="UR10_T_base_from_mocap.json")
    ap.add_argument("--at-n", type=int, default=MIN_SAMPLES_FOR_CHECK)
    ap.add_argument("--height", type=float, default=0.5)
    ap.add_argument("--max-tilt", type=float, default=90.0)
    ap.add_argument("--no-orient-time", action="store_true",
                    help="what-if: ignore the wrist-tilt time (as if orientation were free)")
    args = ap.parse_args()
    if args.self_test:
        sys.exit(0 if self_test(args.transform_file) else 1)
    replay(args.logs or sorted(glob.glob("catch_logs/*.jsonl")), args.at_n, args.height, args.max_tilt,
           not args.no_orient_time)


if __name__ == "__main__":
    main()
