"""Calibration forensics on T_base_from_mocap{,_marker,_marker2}.json:
1. refit rigid (Umeyama, s=1) - confirm stored RMSE
2. fit SIMILARITY (free scale) - a scale != 1 means mocap volume scale error
3. fit full AFFINE - if affine collapses RMSE, the volume is nonrigidly distorted
4. leave-one-out crossval of the rigid fit
5. cross-consistency: marker1 vs marker2 vs old transform in the catch zone
6. residual-vs-position correlation."""
import json
import numpy as np

def load(p):
    d = json.load(open("/home/erkka/codeprojects/OPTITRACK/" + p))
    P = np.array([s["p_mocap"] for s in d["samples"]])
    Q = np.array([s["p_robot"] for s in d["samples"]])
    return d, P, Q

def umeyama(P, Q, with_scale=False):
    mp, mq = P.mean(0), Q.mean(0)
    Pc, Qc = P-mp, Q-mq
    S = Qc.T @ Pc / len(P)
    U, D, Vt = np.linalg.svd(S)
    sgn = np.sign(np.linalg.det(U @ Vt))
    E = np.diag([1,1,sgn])
    R = U @ E @ Vt
    s = (np.trace(np.diag(D) @ E) / np.mean(np.sum(Pc**2,axis=1))) if with_scale else 1.0
    t = mq - s * R @ mp
    res = Q - (s * (R @ P.T).T + t)
    rmse = float(np.sqrt(np.mean(np.sum(res**2,axis=1))))
    return R, t, s, rmse, res

def affine(P, Q):
    A = np.hstack([P, np.ones((len(P),1))])
    X, *_ = np.linalg.lstsq(A, Q, rcond=None)
    res = Q - A @ X
    return X, float(np.sqrt(np.mean(np.sum(res**2,axis=1)))), res

for name in ["T_base_from_mocap.json", "T_base_from_mocap_marker.json", "T_base_from_mocap_marker2.json"]:
    d, P, Q = load(name)
    R,t,s,rmse,res = umeyama(P,Q)
    Rs,ts,ss,rmses,_ = umeyama(P,Q,with_scale=True)
    X, rmsea, resa = affine(P,Q)
    # leave-one-out
    loo=[]
    for i in range(len(P)):
        m = np.ones(len(P),bool); m[i]=False
        Ri,ti,_,_,_ = umeyama(P[m],Q[m])
        loo.append(np.linalg.norm(Q[i]-(Ri@P[i]+ti)))
    # singular values of the affine linear part = per-axis scales
    U2,D2,V2 = np.linalg.svd(X[:3])
    print(f"\n== {name}  n={len(P)}  stored_rmse={d['rmse_m']*1000:.1f}mm  tcp_offset={d['tcp_offset'][:3]}")
    print(f"  rigid refit rmse = {rmse*1000:6.1f}mm   per-sample max={np.max(np.linalg.norm(res,axis=1))*1000:.1f}mm")
    print(f"  similarity fit:  scale={ss:.4f}  rmse={rmses*1000:6.1f}mm")
    print(f"  affine fit:      rmse={rmsea*1000:6.1f}mm  axis scales={D2.round(4)}")
    print(f"  leave-one-out:   rmse={np.sqrt(np.mean(np.array(loo)**2))*1000:6.1f}mm")
    # residual vs position correlation (rigid)
    r_norm = np.linalg.norm(res,axis=1)
    for ax,axn in enumerate("xyz"):
        c = np.corrcoef(P[:,ax], r_norm)[0,1]
        print(f"  corr(|res|, mocap_{axn}) = {c:+.2f}", end="")
    print()

# cross-consistency in the catch zone
d0,P0,Q0 = load("T_base_from_mocap.json")
d1,P1,Q1 = load("T_base_from_mocap_marker.json")
d2,P2,Q2 = load("T_base_from_mocap_marker2.json")
R0,t0 = np.array(d0["R"]), np.array(d0["t"])
R1,t1 = np.array(d1["R"]), np.array(d1["t"])
R2,t2 = np.array(d2["R"]), np.array(d2["t"])
# catch zone in base frame: reach 0.45-1.0, z 0.1-0.4 around wait pose azimuth.
# sample mocap points by inverse-transforming a base-frame grid through T0
grid=[]
for x in np.linspace(-0.3,0.5,5):
    for y in np.linspace(-1.1,-0.4,5):
        for z in np.linspace(0.0,0.4,3):
            grid.append([x,y,z])
grid=np.array(grid)
Pm = (R0.T @ (grid - t0).T).T   # mocap coords of catch-zone points
d01 = np.linalg.norm((R1@Pm.T).T+t1 - grid, axis=1)
d02 = np.linalg.norm((R2@Pm.T).T+t2 - grid, axis=1)
d12 = np.linalg.norm(((R1@Pm.T).T+t1) - ((R2@Pm.T).T+t2), axis=1)
print(f"\nCatch-zone disagreement (75 grid points):")
print(f"  old vs marker1: median={np.median(d01)*1000:.0f}mm max={d01.max()*1000:.0f}mm")
print(f"  old vs marker2: median={np.median(d02)*1000:.0f}mm max={d02.max()*1000:.0f}mm")
print(f"  marker1 vs marker2: median={np.median(d12)*1000:.0f}mm max={d12.max()*1000:.0f}mm")
# NOTE: old vs marker comparisons include the tool-offset difference (tcp 0.12 vs 0.01)
# only if the calibrations tracked different physical points - they map mocap->TCP for
# their OWN tcp setting, so a grid point maps to "where that calibration thinks this
# mocap point is in base frame" - directly comparable.
