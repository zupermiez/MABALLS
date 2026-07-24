"""
Quantify what "let the arm also move up" would buy, using real recorded throws.

Question: for throws in catch_logs/*.jsonl, if instead of intercepting at the
single fixed catch plane (catch_value, wait height) the system searched candidate
higher intercept points earlier in the flight, would more throws clear the
feasibility margin, and how much extra height/lead time does that take?

Method: refit each throw's full raw ball trajectory (post-hoc, all samples - a
best-case ground-truth parabola, isolating the geometry question from perception
accuracy), then for a sweep of candidate mocap-Y (up) plane heights above and
below the run's actual catch_value, solve time_of_plane_crossing, transform to
base frame, and evaluate the same MoveTimeModel/envelope logic catch_feasibility.py
uses. Decision/move-start time is fixed per throw at the recorded commit tick (or,
if absent, the 40th sample) so all candidate heights are compared from the same
information/start point - isolating the effect of catch height by itself.
"""
import glob
import json
import sys

import numpy as np

sys.path.insert(0, "/home/erkka/codeprojects/OPTITRACK")
from trajectory import Sample, fit_trajectory, AXIS_NAMES
from frames import mocap_point_to_base
from catch_feasibility import MoveTimeModel

CATCH_MIN_REACH, CATCH_MAX_REACH = 0.45, 1.20
CATCH_Z_MIN, CATCH_Z_MAX = -0.25, 0.55
CATCH_MAX_AZIMUTH_DEG = 75.0
MIN_SAMPLES_FOR_CHECK = 40

with open("/home/erkka/codeprojects/OPTITRACK/T_base_from_mocap.json") as f:
    _t = json.load(f)
R = np.array(_t["R"])
t_vec = np.array(_t["t"])


def envelope_ok(p_base, wait_xyz):
    reach = np.linalg.norm(p_base)
    if not (CATCH_MIN_REACH <= reach <= CATCH_MAX_REACH):
        return False
    if not (CATCH_Z_MIN <= p_base[2] <= CATCH_Z_MAX):
        return False
    az_wait = np.arctan2(wait_xyz[1], wait_xyz[0])
    az_tgt = np.arctan2(p_base[1], p_base[0])
    d_az = np.degrees(abs(np.arctan2(np.sin(az_tgt - az_wait), np.cos(az_tgt - az_wait))))
    return bool(d_az <= CATCH_MAX_AZIMUTH_DEG)


def analyze_file(path):
    evs = [json.loads(l) for l in open(path) if l.strip()]
    run = next((e for e in evs if e["ev"] == "run_start"), None)
    if run is None or run.get("dry_run"):
        return []
    wait_xyz = np.array(run["wait_pose"][:3])
    catch_value = run["catch_value"]
    mtm = run["move_time_model"]
    model = MoveTimeModel(accel=mtm["accel"], latency=mtm["latency"], v_max=mtm["v_max"],
                           residual_rms=mtm.get("residual_rms", 0.0), n_legs=mtm.get("n_legs", 0))

    by_throw = {}
    for e in evs:
        by_throw.setdefault(e["throw"], {}).setdefault(e["ev"], []).append(e)

    rows = []
    for tn, kinds in sorted(by_throw.items()):
        if tn == 0 or "throw_samples" not in kinds:
            continue
        raw = kinds["throw_samples"][0]["raw"]
        if len(raw) < 45:
            continue
        samples = [Sample(t=r[0], x=r[1], y=r[2], z=r[3]) for r in raw]
        fit = fit_trajectory(samples)

        decision_t = samples[MIN_SAMPLES_FOR_CHECK - 1].t
        commit_ev = kinds.get("commit", [None])[0]
        # Where the move starts from: at commit_ev-time the arm is still parked
        # at the wait pose for the vast majority of real throws (the whole point
        # of "go and wait" - see CLAUDE.md governing insight), so wait_xyz is the
        # right start point for this geometry sweep regardless of whether this
        # particular throw actually committed.
        start_xyz = wait_xyz

        best = None
        baseline = None
        for h in np.arange(catch_value - 0.05, catch_value + 0.65, 0.02):
            ct = fit.time_of_plane_crossing(1, float(h), after_t=decision_t)
            if ct is None:
                continue
            p_mocap = np.array(fit.position(ct))
            p_base = mocap_point_to_base(p_mocap, R, t_vec)
            ok = envelope_ok(p_base, wait_xyz)
            dist = float(np.linalg.norm(p_base - start_xyz))
            move_time = model.estimate(dist)
            t_impact = ct - decision_t
            margin = t_impact - move_time
            rec = dict(h=float(h), margin=margin, ok=ok, dist=dist, t_impact=t_impact,
                       move_time=move_time, reach=float(np.linalg.norm(p_base)), z=float(p_base[2]))
            if abs(h - catch_value) < 0.011:
                baseline = rec
            if ok and (best is None or margin > best["margin"]):
                best = rec

        if baseline is None:
            continue
        rows.append(dict(
            session=path.split("catch_log_")[-1].replace(".jsonl", ""), throw=tn,
            base_margin=baseline["margin"], base_ok=bool(baseline["ok"]),
            base_feasible=bool(baseline["ok"] and baseline["margin"] >= 0),
            best_margin=best["margin"] if best else None,
            best_h=best["h"] if best else None,
            best_feasible=bool(best is not None and best["margin"] >= 0),
            dh=(best["h"] - catch_value) if best else None,
            committed=bool(kinds.get("commit")),
            verdict=commit_ev.get("verdict") if commit_ev else None,
            caught=kinds.get("throw_end", [{}])[0].get("caught_guess"),
        ))
    return rows


all_rows = []
for path in sorted(glob.glob("/home/erkka/codeprojects/OPTITRACK/catch_logs/*.jsonl")):
    all_rows.extend(analyze_file(path))

print(f"analyzed {len(all_rows)} throws with >=45 raw samples across all sessions\n")

n = len(all_rows)
base_feas = sum(r["base_feasible"] for r in all_rows)
best_feas = sum(r["best_feasible"] for r in all_rows)
flipped = [r for r in all_rows if r["best_feasible"] and not r["base_feasible"]]
lost = [r for r in all_rows if r["base_feasible"] and not r["best_feasible"]]

print(f"feasible at FIXED catch plane:        {base_feas}/{n} ({100*base_feas/n:.0f}%)")
print(f"feasible at BEST searched height:     {best_feas}/{n} ({100*best_feas/n:.0f}%)")
print(f"newly unlocked by searching height:   {len(flipped)}")
print(f"lost (shouldn't happen, sanity check): {len(lost)}")

if flipped:
    dhs = [r["dh"] for r in flipped]
    margin_gain = [r["best_margin"] - r["base_margin"] for r in flipped]
    print(f"\nof the {len(flipped)} newly-unlocked throws:")
    print(f"  extra height needed (dh, m): min={min(dhs):.2f} med={sorted(dhs)[len(dhs)//2]:.2f} max={max(dhs):.2f}")
    print(f"  margin gain (s):             min={min(margin_gain):.2f} med={sorted(margin_gain)[len(margin_gain)//2]:.2f} max={max(margin_gain):.2f}")

# Among throws that were feasible at the fixed plane already, how much margin
# does the best height search buy on top (headroom / robustness question)?
already = [r for r in all_rows if r["base_feasible"]]
if already:
    gains = [r["best_margin"] - r["base_margin"] for r in already]
    dhs2 = [r["dh"] for r in already]
    print(f"\nof the {len(already)} ALREADY-feasible throws, searching height still gains:")
    print(f"  margin gain (s): min={min(gains):.2f} med={sorted(gains)[len(gains)//2]:.2f} max={max(gains):.2f}")
    print(f"  at dh (m):       med={sorted(dhs2)[len(dhs2)//2]:.2f}")

# Distribution of base_margin among currently-infeasible throws - are they close
# misses (small negative margin, good candidates for a height fix) or hopeless?
infeas = [r for r in all_rows if not r["base_feasible"]]
if infeas:
    margins = sorted(r["base_margin"] for r in infeas)
    print(f"\n{len(infeas)} infeasible-at-fixed-plane throws, base_margin distribution (s):")
    print(f"  min={margins[0]:.2f} p25={margins[len(margins)//4]:.2f} med={margins[len(margins)//2]:.2f} "
          f"p75={margins[3*len(margins)//4]:.2f} max={margins[-1]:.2f}")
    still_bad = [r for r in infeas if not r["best_feasible"] and r["best_margin"] is not None]
    if still_bad:
        m2 = sorted(r["best_margin"] for r in still_bad)
        print(f"  of those STILL infeasible even at best height ({len(still_bad)}): "
              f"best_margin med={m2[len(m2)//2]:.2f}")

print("\n--- restricted to throws catch.py actually COMMITTED to ---")
committed_rows = [r for r in all_rows if r["committed"]]
nc = len(committed_rows)
if nc:
    bf = sum(r["base_feasible"] for r in committed_rows)
    bef = sum(r["best_feasible"] for r in committed_rows)
    flip2 = [r for r in committed_rows if r["best_feasible"] and not r["base_feasible"]]
    print(f"n={nc}  fixed-plane feasible={bf} ({100*bf/nc:.0f}%)  best-height feasible={bef} ({100*bef/nc:.0f}%)  "
          f"newly unlocked={len(flip2)}")
    caught_known = [r for r in committed_rows if r["caught"] is not None]
    if caught_known:
        print(f"  of {len(caught_known)} with a known outcome: "
              f"caught={sum(1 for r in caught_known if r['caught'])} "
              f"missed={sum(1 for r in caught_known if not r['caught'])}")
        missed_rows = [r for r in caught_known if not r["caught"]]
        if missed_rows:
            flip3 = [r for r in missed_rows if r["best_feasible"] and not r["base_feasible"]]
            print(f"  of {len(missed_rows)} MISSED real catches, height-search would have newly "
                  f"unlocked feasibility for {len(flip3)}")

with open("/home/erkka/codeprojects/OPTITRACK/analysis/vertical_catch_rows.json", "w") as f:
    json.dump(all_rows, f, indent=1)
print("\nsaved analysis/vertical_catch_rows.json")
