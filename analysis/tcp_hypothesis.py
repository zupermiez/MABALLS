"""Discriminate marker-cal failure modes using the old (catch-validated)
transform as ground truth for the marker's base-frame position:
  diff_i = p_robot_i - (R0 @ p_mocap_i + t0)
- constant |diff| ~ 0.1m, varying direction  -> robot reported a point ~10cm
  from the marker in the FLANGE frame (stale/wrong TCP offset, or marker not
  where assumed) - |R_flange d| = |d| is pose-invariant.
- small scattered |diff| -> mocap-side error instead."""
import json
import numpy as np

base="/home/erkka/codeprojects/OPTITRACK/"
d0=json.load(open(base+"T_base_from_mocap.json"))
R0,t0=np.array(d0["R"]),np.array(d0["t"])
for name in ["T_base_from_mocap_marker.json","T_base_from_mocap_marker2.json"]:
    d=json.load(open(base+name))
    P=np.array([s["p_mocap"] for s in d["samples"]])
    Q=np.array([s["p_robot"] for s in d["samples"]])
    marker_base=(R0@P.T).T+t0
    diff=Q-marker_base
    mag=np.linalg.norm(diff,axis=1)
    print(f"\n== {name} (tcp_offset={d['tcp_offset'][:3]})")
    print("  |p_robot - marker_true| per sample (mm):", (mag*1000).round(0))
    print(f"  mean={mag.mean()*1000:.0f}mm  std={mag.std()*1000:.0f}mm  "
          f"min={mag.min()*1000:.0f}  max={mag.max()*1000:.0f}")
    print("  mean diff vector:", (diff.mean(0)*1000).round(0), "mm")
    # if it's a constant flange-frame offset d with varying orientation, the
    # base-frame diff vectors should have ~constant magnitude but varied direction:
    dirs = diff/mag[:,None]
    print(f"  direction spread: mean pairwise angle = "
          f"{np.degrees(np.mean(np.arccos(np.clip(dirs@dirs.T,-1,1))[np.triu_indices(len(dirs),1)])):.0f}deg")
