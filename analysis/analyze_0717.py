"""Forensic analysis of the 2026-07-17 catch sessions.

For each session: outcomes per throw; for attempted throws, reconstruct the
ball's ACTUAL catch-plane crossing from throw_samples (mocap frame, plane
y=catch_value_mocap), transform to base, and compare against (a) the final
committed target and (b) the arm TCP at throw end. Also compute yaw-follow
azimuth deltas actually commanded, and classify why non-attempted throws
never committed.
"""
import json, glob, math, sys
import numpy as np

sys.path.insert(0, "/home/erkka/codeprojects/OPTITRACK")
# transform used by ALL sessions (confirmed from run_start)
T = json.load(open("/home/erkka/codeprojects/OPTITRACK/T_base_from_mocap.json"))
R = np.array(T["R"]); t = np.array(T["t"])

def fmt(v):
    return f"{v*100:5.1f}cm" if isinstance(v,(int,float)) else "  n/a "

def to_base(p):
    return R @ np.asarray(p) + t

def plane_crossing(raw, plane_y, falling_only=True):
    """raw: [[t,x,y,z],...] mocap frame. Find crossing of y=plane_y while
    descending (interpolated). Returns (t_cross, xyz_mocap) or None."""
    best = None
    for i in range(1, len(raw)):
        t0, x0, y0, z0 = raw[i-1]
        t1, x1, y1, z1 = raw[i]
        if y0 >= plane_y >= y1 and y0 != y1:  # descending through the plane
            a = (y0 - plane_y) / (y0 - y1)
            best = (t0 + a*(t1-t0),
                    np.array([x0 + a*(x1-x0), plane_y, z0 + a*(z1-z0)]))
            # keep FIRST descending crossing (the catchable one)
            return best
    return best

sessions = sorted(glob.glob("/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260717_*.jsonl"))

overall = []
for path in sessions:
    lines = [json.loads(l) for l in open(path)]
    cfg = lines[0]
    throws = {}
    for ev in lines:
        n = ev.get("throw", 0)
        if n == 0: continue
        throws.setdefault(n, []).append(ev)
    name = path.split("_",3)[-1].replace(".jsonl","")
    plane_mocap = None
    # derive mocap plane value: catch_value is logged in run_start (mocap frame per derive_catch_plane)
    plane_mocap = cfg.get("catch_value")
    print(f"\n=== {name}  move={cfg['catch_move']} yawf={cfg['yaw_follow']} "
          f"wait={cfg['wait_pose'][:3]} appr={cfg['approach_speed']} throws={len(throws)}")
    wait_xyz = np.array(cfg["wait_pose"][:3])
    az_wait = math.atan2(wait_xyz[1], wait_xyz[0])

    for n in sorted(throws):
        evs = throws[n]
        by = {}
        for e in evs: by.setdefault(e["ev"], []).append(e)
        commits = by.get("commit", [])
        reaims = by.get("reaim", [])
        guards = by.get("guard", [])
        refuses = by.get("refuse", [])
        ends = by.get("throw_end", [])
        samples = by.get("throw_samples", [])
        ticks = by.get("tick", [])
        end = ends[0] if ends else None
        attempted = end.get("attempted") if end else bool(commits)
        caught = end.get("caught_guess") if end else None
        row = dict(session=name, throw=n, move=cfg["catch_move"], yawf=cfg["yaw_follow"],
                   attempted=attempted, caught=caught, reaims=len(reaims),
                   guarded=bool(guards), refused=bool(refuses))
        # why no commit?
        if not attempted:
            if guards: row["why"] = "guard: " + guards[0]["reason"][:60]
            elif refuses: row["why"] = "refuse: " + refuses[0]["reason"][:60]
            else:
                # look at ticks: was gate ever true? commit_ready?
                verdicts = [tk.get("verdict") for tk in ticks]
                ncr = [tk for tk in ticks if tk.get("commit_ready")]
                gate_true = [tk for tk in ticks if tk.get("verdict") in ("catch","possible")]
                gate_ready = [tk for tk in ticks if tk.get("verdict") in ("catch","possible") and tk.get("commit_ready")]
                if not ticks: row["why"] = "no ticks (flight too short/no fit)"
                elif all(v == "no_crossing" for v in verdicts): row["why"] = "no plane crossing"
                elif not gate_true: row["why"] = "gate never true: " + ",".join(sorted(set(v for v in verdicts if v)))[:60]
                elif not ncr: row["why"] = f"never reached commit_samples (max n={max(tk.get('n',0) for tk in ticks)})"
                elif not gate_ready: row["why"] = "gate true only before commit_samples"
                else: row["why"] = "UNEXPLAINED (gate+ready but no commit)"
                # add shortfall info for infeasible
                sf = [tk.get("shortfall") for tk in ticks if tk.get("shortfall") is not None]
                if sf: row["why"] += f" shortfall_min={min(sf):.2f}s"
        else:
            final_target = (reaims[-1]["target_pose"] if reaims else commits[0]["target_pose"]) if commits else None
            row["final_target"] = final_target
            row["commit_n"] = None
            # find n at commit: last tick before commit time
            if commits:
                ct = commits[0]["t"]
                pre = [tk for tk in ticks if tk["t"] <= ct]
                if pre: row["commit_n"] = pre[-1].get("n")
            # yaw delta
            if final_target:
                az_t = math.atan2(final_target[1], final_target[0])
                row["d_az_deg"] = math.degrees(math.atan2(math.sin(az_t-az_wait), math.cos(az_t-az_wait)))
            # actual crossing
            if samples and samples[0].get("raw") and final_target:
                cr = plane_crossing(samples[0]["raw"], plane_mocap)
                if cr:
                    xb = to_base(cr[1])
                    ft = np.array(final_target[:3])
                    tcp = np.array(end["arm_tcp_at_end"][:3]) if end and end.get("arm_tcp_at_end") else None
                    row["pred_err_m"] = float(np.linalg.norm(xb - ft))          # prediction vs truth
                    row["pred_err_xy"] = float(np.linalg.norm(xb[:2] - ft[:2]))
                    if tcp is not None:
                        row["arm_err_m"] = float(np.linalg.norm(ft - tcp))       # did arm reach target
                        row["total_err_m"] = float(np.linalg.norm(xb - tcp))     # truth vs arm
                        row["total_err_xy"] = float(np.linalg.norm(xb[:2] - tcp[:2]))
                else:
                    row["pred_err_m"] = None  # ball never descended through plane (caught above? occluded)
        overall.append(row)
        flag = ("CAUGHT" if caught else ("miss " if caught is False else "  -  "))
        extra = ""
        if attempted:
            extra = (f" n@commit={row.get('commit_n')} reaims={len(reaims)} d_az={row.get('d_az_deg',float('nan')):+5.1f}deg"
                     f" pred_err={fmt(row.get('pred_err_m'))} arm_err={fmt(row.get('arm_err_m'))}"
                     f" total={fmt(row.get('total_err_m'))} totalXY={fmt(row.get('total_err_xy'))}")
        else:
            extra = " " + row.get("why","")
        print(f"  throw {n:2d} {'ATT' if attempted else '   '} {flag}{extra}")

def fmt(v):
    return f"{v*100:5.1f}cm" if isinstance(v,(int,float)) else "  n/a "

# --- aggregates ---
import statistics
def agg(rows, key):
    vals = [r[key] for r in rows if isinstance(r.get(key),(int,float))]
    if not vals: return "n/a"
    return f"n={len(vals)} median={statistics.median(vals)*100:.1f}cm mean={np.mean(vals)*100:.1f}cm p90={np.percentile(vals,90)*100:.1f}cm"

att = [r for r in overall if r["attempted"]]
print("\n\n======== AGGREGATES ========")
for grp_name, grp in [("movej throws", [r for r in att if r["move"]=="movej"]),
                       ("movel+yawf (155142)", [r for r in att if r["move"]=="movel" and r["yawf"]]),
                       ("movel baseline (morning)", [r for r in att if r["move"]=="movel" and not r["yawf"]])]:
    if not grp: continue
    c = sum(1 for r in grp if r["caught"]); tot = sum(1 for r in grp if r["caught"] is not None)
    print(f"\n{grp_name}: attempts={len(grp)} caught={c}/{tot}")
    print("  prediction err (final target vs true crossing):", agg(grp,"pred_err_m"))
    print("  arm err (target vs TCP at end):               ", agg(grp,"arm_err_m"))
    print("  TOTAL err (true crossing vs TCP, 3D):         ", agg(grp,"total_err_m"))
    print("  TOTAL err (true crossing vs TCP, XY):         ", agg(grp,"total_err_xy"))
    da = [abs(r.get("d_az_deg",0)) for r in grp if r.get("d_az_deg") is not None]
    if da: print(f"  |d_az|: median={statistics.median(da):.1f}deg max={max(da):.1f}deg")
    ra = [r["reaims"] for r in grp]
    print(f"  reaims per attempt: mean={np.mean(ra):.2f} with>=1: {sum(1 for x in ra if x)}/{len(ra)}")

# caught vs missed total error
for lab, rows in [("caught", [r for r in att if r["caught"]]), ("missed", [r for r in att if r["caught"] is False])]:
    print(f"\n{lab}: ", agg(rows, "total_err_xy"), " | pred:", agg(rows,"pred_err_m"), " | arm:", agg(rows,"arm_err_m"))

# non-attempted reasons histogram
from collections import Counter
non = [r for r in overall if not r["attempted"]]
print(f"\nnon-attempted throws: {len(non)}/{len(overall)}")
cnt = Counter((r.get("why","?").split(" shortfall")[0]) for r in non)
for k,v in cnt.most_common(): print(f"  {v:3d}  {k}")
