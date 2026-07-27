"""For every ATTEMPTED missed throw: did the ball deviate from ballistic flight
(hit the box rim / arm) before its recorded plane crossing?  Method: fit a
quadratic per axis to a clean mid-flight window (n=15..45), extrapolate to the
catch plane -> 'ballistic truth'.  Compare (a) committed target vs ballistic
truth (real prediction error), (b) recorded crossing vs ballistic truth
(deflection magnitude), and (c) min distance ball-to-committed-target."""
import json, glob
import pathlib
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

T = json.load(open(REPO_ROOT / "T_base_from_mocap.json"))
R = np.array(T["R"]); tv = np.array(T["t"])
def to_base(p): return R @ np.asarray(p) + tv

def plane_crossing_from_fit(coef, t0, t1, plane):
    # coef: per-axis quad [a,b,c] in t. Solve y(t)=plane for t in [t0, t1+2]
    a,b,c = coef[1]
    A,B,C = a, b, c-plane
    if abs(A) < 1e-9: return None
    disc = B*B - 4*A*C
    if disc < 0: return None
    r1 = (-B - np.sqrt(disc))/(2*A); r2 = (-B + np.sqrt(disc))/(2*A)
    cands = [r for r in (r1,r2) if t0 < r < t1 + 2.0]
    # descending crossing: dy/dt < 0
    cands = [r for r in cands if 2*a*r + b < 0]
    return min(cands) if cands else None

def recorded_crossing(raw, plane):
    for i in range(1, len(raw)):
        t0,x0,y0,z0 = raw[i-1]; t1,x1,y1,z1 = raw[i]
        if y0 >= plane >= y1 and y0 != y1:
            a=(y0-plane)/(y0-y1)
            return (t0+a*(t1-t0), np.array([x0+a*(x1-x0), plane, z0+a*(z1-z0)]))
    return None

sessions = sorted(glob.glob(str(REPO_ROOT / "catch_logs/catch_log_20260717_*.jsonl")))
print(f"{'sess':>6} {'thr':>3} {'pred_err_ballistic':>18} {'deflection':>10} {'min_d_tgt':>9} {'kink?':>6} verdict")
rows=[]
for path in sessions:
    lines=[json.loads(l) for l in open(path)]
    cfg=lines[0]; plane=cfg["catch_value"]
    throws={}
    for ev in lines:
        n=ev.get("throw",0)
        if n: throws.setdefault(n,[]).append(ev)
    for n,evs in sorted(throws.items()):
        by={}
        for e in evs: by.setdefault(e["ev"],[]).append(e)
        end=by.get("throw_end",[None])[0]; commits=by.get("commit",[])
        samples=by.get("throw_samples",[])
        reaims=by.get("reaim",[])
        if not end or not commits or end.get("caught_guess") is not False: continue
        if not samples or not samples[0].get("raw"): continue
        raw=np.array(samples[0]["raw"])
        committed=np.array((reaims[-1]["target_pose"] if reaims else commits[0]["target_pose"])[:3])
        # clean mid-flight ballistic fit (samples 15..45, before any arm contact plausible)
        w = raw[15:45] if len(raw)>=45 else raw[10:max(len(raw)-5,13)]
        if len(w)<10: continue
        tt=w[:,0]-w[0,0]
        coef=[np.polyfit(tt,w[:,1+ax],2) for ax in range(3)]
        tc = plane_crossing_from_fit(coef, tt[0], raw[-1,0]-w[0,0], plane)
        rec = recorded_crossing(raw, plane)
        if tc is None: continue
        bal = np.array([np.polyval(coef[ax], tc) for ax in range(3)])
        bal_b = to_base(bal)
        pred_err = float(np.linalg.norm(bal_b-committed))
        defl = float(np.linalg.norm(to_base(rec[1])-bal_b)) if rec else None
        # min dist ball to committed target (base frame) over flight
        pb = np.array([to_base(p) for p in raw[:,1:4]])
        dmin = float(np.min(np.linalg.norm(pb-committed,axis=1)))
        # kink: max deviation of recorded samples from ballistic fit AFTER fit window
        t_all = raw[:,0]-w[0,0]
        pred_all = np.stack([np.polyval(coef[ax], t_all) for ax in range(3)],axis=1)
        dev = np.linalg.norm(raw[:,1:4]-pred_all,axis=1)
        post = dev[45:] if len(raw)>45 else dev[-5:]
        kink = float(np.max(post)) if len(post) else 0.0
        sess=path.split("_")[-1][:6]
        verdict = "RIM-HIT/deflected" if (defl and defl>0.25) or dmin<0.20 else ("clean miss" if pred_err>0.15 else "near miss")
        rows.append((sess,n,pred_err,defl,dmin,kink,verdict))
        print(f"{sess:>6} {n:3d} {pred_err*100:17.1f}cm {defl*100 if defl else -1:9.1f}cm "
              f"{dmin*100:8.1f}cm {kink*100:5.1f}cm {verdict}")

pe=[r[2] for r in rows]
print(f"\nBallistic prediction error on missed throws: n={len(pe)} median={np.median(pe)*100:.1f}cm "
      f"p25={np.percentile(pe,25)*100:.1f} p75={np.percentile(pe,75)*100:.1f}")
print("dmin<20cm (ball basically reached the box):", sum(1 for r in rows if r[4]<0.20), "/", len(rows))
