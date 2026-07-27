"""Frame-shift test: (1) direction/magnitude of marker-cal vs old-cal
disagreement in the catch zone; (2) per-session mean error VECTOR
(ballistic-truth crossing minus committed target) for attempted throws,
morning (pre camera reconfig) vs evening (post) - a frame shift appears as a
consistent evening-only bias matching (1). Uses caught throws' last-seen
vector too as a weaker check."""
import json, glob
import pathlib
import numpy as np

base = str(pathlib.Path(__file__).resolve().parent.parent) + "/"
d0 = json.load(open(base+"T_base_from_mocap.json"))
d1 = json.load(open(base+"T_base_from_mocap_marker.json"))
R0,t0 = np.array(d0["R"]), np.array(d0["t"])
R1,t1 = np.array(d1["R"]), np.array(d1["t"])

grid=[]
for x in np.linspace(-0.3,0.5,5):
    for y in np.linspace(-1.1,-0.4,5):
        for z in np.linspace(0.0,0.4,3):
            grid.append([x,y,z])
grid=np.array(grid)
Pm=(R0.T@(grid-t0).T).T
delta=(R1@Pm.T).T+t1-grid
print("marker1-vs-old offset in catch zone: mean vector "
      f"({delta[:,0].mean()*1000:+.0f},{delta[:,1].mean()*1000:+.0f},{delta[:,2].mean()*1000:+.0f})mm "
      f"|mean|={np.linalg.norm(delta.mean(0))*1000:.0f}mm  spread(std)={np.std(delta,axis=0).round(3)*1000}")

def to_base(p): return R0@np.asarray(p)+t0
def ballistic_cross(raw, plane):
    if len(raw)<45: w=raw[10:max(len(raw)-5,13)]
    else: w=raw[15:45]
    if len(w)<10: return None
    tt=w[:,0]-w[0,0]
    coef=[np.polyfit(tt,w[:,1+ax],2) for ax in range(3)]
    a,b,c=coef[1]; A,B,C=a,b,c-plane
    if abs(A)<1e-9: return None
    disc=B*B-4*A*C
    if disc<0: return None
    roots=[(-B-np.sqrt(disc))/(2*A),(-B+np.sqrt(disc))/(2*A)]
    roots=[r for r in roots if tt[0]<r<raw[-1,0]-w[0,0]+2.0 and 2*a*r+b<0]
    if not roots: return None
    tc=min(roots)
    return np.array([np.polyval(coef[ax],tc) for ax in range(3)])

morning=[]; evening=[]
for path in sorted(glob.glob(base+"catch_logs/catch_log_20260717_*.jsonl")):
    lines=[json.loads(l) for l in open(path)]
    cfg=lines[0]; plane=cfg["catch_value"]
    hh=int(path.split("_")[-1][:2]); mm=int(path.split("_")[-1][2:4])
    is_evening = hh>=15
    throws={}
    for ev in lines:
        n=ev.get("throw",0)
        if n: throws.setdefault(n,[]).append(ev)
    for n,evs in throws.items():
        by={}
        for e in evs: by.setdefault(e["ev"],[]).append(e)
        end=by.get("throw_end",[None])[0]; commits=by.get("commit",[])
        samples=by.get("throw_samples",[]); reaims=by.get("reaim",[])
        if not end or not commits: continue
        if not samples or not samples[0].get("raw"): continue
        raw=np.array(samples[0]["raw"])
        cross=ballistic_cross(raw, plane)
        if cross is None: continue
        committed=np.array((reaims[-1]["target_pose"] if reaims else commits[0]["target_pose"])[:3])
        errv = to_base(cross)-committed
        (evening if is_evening else morning).append(errv)

for lab, arr in [("morning (old cam config)", morning), ("evening (new cam config)", evening)]:
    a=np.array(arr)
    if not len(a): continue
    m=a.mean(0)
    print(f"\n{lab}: n={len(a)}")
    print(f"  mean error vector (truth - target): ({m[0]*1000:+.0f},{m[1]*1000:+.0f},{m[2]*1000:+.0f})mm |mean|={np.linalg.norm(m)*1000:.0f}mm")
    print(f"  per-axis std: {np.std(a,axis=0).round(3)*1000}")
    print(f"  |err| median: {np.median(np.linalg.norm(a,axis=1))*1000:.0f}mm")
