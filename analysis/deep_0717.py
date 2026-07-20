"""Deep-dive: (1) why re-aim never fires, (2) prediction-error evolution vs n
on missed throws (how much a later correction would have bought), (3) caught-
ball last-distance proxy for movej vs movel accuracy, (4) guard/shortfall
margins on non-committed throws."""
import json, glob, math
import numpy as np

T = json.load(open("/home/erkka/codeprojects/OPTITRACK/T_base_from_mocap.json"))
R = np.array(T["R"]); tv = np.array(T["t"])
def to_base(p): return R @ np.asarray(p) + tv

def plane_crossing(raw, plane_y):
    for i in range(1, len(raw)):
        t0,x0,y0,z0 = raw[i-1]; t1,x1,y1,z1 = raw[i]
        if y0 >= plane_y >= y1 and y0 != y1:
            a = (y0-plane_y)/(y0-y1)
            return (t0+a*(t1-t0), np.array([x0+a*(x1-x0), plane_y, z0+a*(z1-z0)]))
    return None

MARGIN = 0.03
sessions = sorted(glob.glob("/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260717_*.jsonl"))

reaim_block = dict(no_postcommit_ticks=0, not_settled=0, drift_small=0, no_time=0,
                   envelope=0, fired=0, no_crossing_post=0)
# For "not settled" we can't see TCP speed; infer settled = |from_tcp - committed| < 0.03
evol = []   # (rel_n, err) for missed throws: prediction error vs samples
late_fix = []  # per missed throw: (err_at_commit, err_at_last_feasible_correction, t_impact_then)
caught_dist = {"movej": [], "movel": []}
guard_dists = []
shortfalls = []

for path in sessions:
    lines = [json.loads(l) for l in open(path)]
    cfg = lines[0]
    plane = cfg["catch_value"]
    throws = {}
    for ev in lines:
        n = ev.get("throw", 0)
        if n: throws.setdefault(n, []).append(ev)
    for n, evs in throws.items():
        by = {}
        for e in evs: by.setdefault(e["ev"], []).append(e)
        commits = by.get("commit", []); ends = by.get("throw_end", [])
        ticks = by.get("tick", []); samples = by.get("throw_samples", [])
        reaims = by.get("reaim", [])
        end = ends[0] if ends else None
        for g in by.get("guard", []):
            r = g["reason"]
            if "release only" in r:
                guard_dists.append(float(r.split("release only ")[1].split("m")[0]))
        for tk in ticks:
            if tk.get("shortfall") not in (None, 0.0) and not tk.get("post_commit"):
                shortfalls.append(tk["shortfall"])
        if not commits or end is None: continue
        mv = cfg["catch_move"]
        if end.get("caught_guess") and end.get("ball_last_dist_m") is not None:
            caught_dist[mv].append(end["ball_last_dist_m"])

        committed = np.array(commits[0]["target_pose"][:3])
        ct = commits[0]["t"]
        post = [tk for tk in ticks if tk.get("post_commit") and tk.get("t", 0) > ct]
        if not post:
            reaim_block["no_postcommit_ticks"] += 1
        else:
            fired = bool(reaims)
            if fired:
                reaim_block["fired"] += 1
            else:
                # classify the DOMINANT blocker across post-commit ticks
                cls = []
                cur = committed
                for tk in post:
                    tgt = tk.get("target"); fr = tk.get("from_tcp")
                    ti = tk.get("t_impact"); mt = tk.get("move_time")
                    if tgt is None: cls.append("no_crossing_post"); continue
                    settled = fr is not None and float(np.linalg.norm(np.array(fr)-cur)) < 0.03
                    drift = float(np.linalg.norm(np.array(tgt)-cur))
                    corr_ok = ti is not None and mt is not None and ti > mt + MARGIN
                    if not settled: cls.append("not_settled")
                    elif drift < 0.02: cls.append("drift_small")
                    elif not corr_ok: cls.append("no_time")
                    else: cls.append("envelope")
                from collections import Counter
                # blocker = what stopped the LAST tick that had drift>=2cm, else most common
                pick = Counter(cls).most_common(1)[0][0]
                big = [c for c,d in zip(cls,[float(np.linalg.norm(np.array(tk.get("target"))-committed)) if tk.get("target") else 0 for tk in post]) if d>=0.02]
                if big: pick = Counter(big).most_common(1)[0][0]
                reaim_block[pick] += 1

        # prediction evolution for missed throws
        if end.get("caught_guess") is False and samples and samples[0].get("raw"):
            cr = plane_crossing(samples[0]["raw"], plane)
            if cr:
                t_cross, xm = cr
                xb = to_base(xm)
                pts = [(tk["n"], tk["t"], np.array(tk["target"]), tk.get("from_tcp"), tk.get("move_time"))
                       for tk in ticks if tk.get("target") is not None]
                if pts:
                    for nn, tt, tgt, fr, mt in pts:
                        evol.append((nn, float(np.linalg.norm(tgt - xb))))
                    e_commit = next((float(np.linalg.norm(t2-xb)) for n2,t2q,t2,f2,m2 in
                                     [(p[0],p[1],p[2],p[3],p[4]) for p in pts] if t2q<=ct), None)
                    # error at commit tick = last tick <= ct
                    pre = [p for p in pts if p[1] <= ct]
                    e_commit = float(np.linalg.norm(pre[-1][2]-xb)) if pre else None
                    # last tick where a correction from the COMMITTED point would still land:
                    # corr distance = |target - committed|, corr feasible if t_cross - t > model-ish time
                    best = None
                    for nn, tt, tgt, fr, mt in pts:
                        t_left = t_cross - tt
                        corr_d = float(np.linalg.norm(tgt - committed))
                        corr_t = 0.11 + corr_d/1.3 + 0.3  # latency + cruise + ramp fudge
                        if t_left > corr_t:
                            best = (nn, float(np.linalg.norm(tgt - xb)), t_left)
                    if e_commit is not None:
                        late_fix.append((e_commit, best))

print("RE-AIM blocker per attempted throw:", reaim_block)
print("\nGuard 'release only Xm' distances:", sorted(guard_dists))
sf = np.array(shortfalls)
print(f"\nshortfall (pre-commit infeasible ticks): n={len(sf)} p25={np.percentile(sf,25):.2f} "
      f"median={np.median(sf):.2f} p75={np.percentile(sf,75):.2f} (meters short at impact)")
for mv, d in caught_dist.items():
    if d:
        print(f"\ncaught ball_last_dist ({mv}): n={len(d)} median={np.median(d)*100:.1f}cm "
              f"mean={np.mean(d)*100:.1f}cm p90={np.percentile(d,90)*100:.1f}cm")

print("\nPrediction-error evolution on MISSED throws (error vs n, all missed):")
evol = np.array(evol)
for lo, hi in [(35,45),(45,55),(55,65),(65,75),(75,90),(90,120)]:
    sel = evol[(evol[:,0]>=lo)&(evol[:,0]<hi)]
    if len(sel):
        print(f"  n {lo:3d}-{hi:3d}: median err {np.median(sel[:,1])*100:5.1f}cm  (ticks={len(sel)})")

print("\nMissed throws: err at commit -> err at last still-correctable tick:")
for e_c, best in late_fix:
    if best:
        print(f"  commit err {e_c*100:5.1f}cm -> n={best[0]:3d} err {best[1]*100:5.1f}cm (t_left {best[2]:.2f}s)")
    else:
        print(f"  commit err {e_c*100:5.1f}cm -> no correctable window")
