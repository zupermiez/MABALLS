"""Where did never-committed throws (feasibility-gate refusals) actually cross
the catch plane, in base frame, relative to the wait pose? And what was their
flight time / time-to-impact at first tick? -> tells us whether a different
wait pose / plane would convert refusals into attempts."""
import json, glob
import numpy as np

base="/home/erkka/codeprojects/OPTITRACK/"
T=json.load(open(base+"T_base_from_mocap.json"))
R0,t0=np.array(T["R"]),np.array(T["t"])
def to_base(p): return R0@np.asarray(p)+t0

rows=[]
for path in sorted(glob.glob(base+"catch_logs/catch_log_20260717_*.jsonl")):
    lines=[json.loads(l) for l in open(path)]
    cfg=lines[0]; plane=cfg["catch_value"]; wait=np.array(cfg["wait_pose"][:3])
    throws={}
    for ev in lines:
        n=ev.get("throw",0)
        if n: throws.setdefault(n,[]).append(ev)
    for n,evs in throws.items():
        by={}
        for e in evs: by.setdefault(e["ev"],[]).append(e)
        if by.get("commit") or by.get("guard"): continue
        ticks=by.get("tick",[]); samples=by.get("throw_samples",[])
        end=by.get("throw_end",[None])[0]
        if not ticks or not samples or not samples[0].get("raw"): continue
        raw=np.array(samples[0]["raw"])
        # recorded descending crossing (no arm contact for refused throws)
        cross=None
        for i in range(1,len(raw)):
            if raw[i-1,2]>=plane>=raw[i,2] and raw[i-1,2]!=raw[i,2]:
                a=(raw[i-1,2]-plane)/(raw[i-1,2]-raw[i,2])
                cross=raw[i-1,1:4]+a*(raw[i,1:4]-raw[i-1,1:4]); break
        verdicts=set(tk.get("verdict") for tk in ticks)
        ti=[tk.get("t_impact") for tk in ticks if tk.get("t_impact")]
        dur=end.get("duration") if end else None
        cb=to_base(cross) if cross is not None else None
        rows.append(dict(sess=path.split("_")[-1][:6], n=n, cross=cb,
                          d_wait=float(np.linalg.norm(cb-wait)) if cb is not None else None,
                          reach=float(np.linalg.norm(cb)) if cb is not None else None,
                          t_impact_first=ti[0] if ti else None, dur=dur,
                          verdicts=",".join(sorted(v for v in verdicts if v))))

print(f"{len(rows)} never-committed (non-guarded) throws with data")
withc=[r for r in rows if r["cross"] is not None]
print(f"{len(withc)} actually crossed the plane; of those:")
d=np.array([r["d_wait"] for r in withc]); rc=np.array([r["reach"] for r in withc])
ti=np.array([r["t_impact_first"] for r in withc if r["t_impact_first"]])
print(f"  dist from wait pose: median={np.median(d):.2f}m p25={np.percentile(d,25):.2f} p75={np.percentile(d,75):.2f} max={d.max():.2f}")
print(f"  reach from base:     median={np.median(rc):.2f}m p25={np.percentile(rc,25):.2f} p75={np.percentile(rc,75):.2f}")
print(f"  in-envelope reach (0.45-1.2): {np.sum((rc>=0.45)&(rc<=1.2))}/{len(rc)}")
print(f"  t_impact at first tick: median={np.median(ti):.2f}s p25={np.percentile(ti,25):.2f} p75={np.percentile(ti,75):.2f}")
mean_c=np.mean(np.array([r["cross"] for r in withc]),axis=0)
print(f"  mean crossing point: ({mean_c[0]:+.2f},{mean_c[1]:+.2f},{mean_c[2]:+.2f})  (wait pose ~(0.04,-0.72,0.14))")
# how many would be feasible if the arm STARTED at the crossing-cloud centroid?
feas=0
for r in withc:
    dd=float(np.linalg.norm(r["cross"]-mean_c))
    if r["t_impact_first"] and 0.45<=r["reach"]<=1.2:
        mt=0.11+dd/1.3+0.35  # latency+cruise+ramp approx
        if r["t_impact_first"]>mt+0.03 or dd- (1.3*(r["t_impact_first"]-0.11-0.35))<=0.15: feas+=1
print(f"  rough: feasible-if-waiting-at-centroid: {feas}/{len(withc)}")
never=[r for r in rows if r["cross"] is None]
print(f"\n{len(never)} never crossed the plane at all (too low/flat or ended early)")
from collections import Counter
print("verdict mixes:", Counter(r["verdicts"] for r in rows).most_common())
