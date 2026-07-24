"""Forensic analysis of the early-commit servo feature (dcb9f5d, 2026-07-22/23).

Splits catch_logs/*.jsonl into two groups by the presence of run_start's
"early_commit_samples" field (the feature's actual on-wire signature - it was
added mid-day 2026-07-22, so file naming alone can't be trusted as the cutoff):

  PRE  = servo mode, full feasibility gate + 40-sample commit (2026-07-21 all
         day, 2026-07-22 morning through 11:47)
  POST = servo mode, feasibility gate BYPASSED, 5-sample early commit
         (2026-07-22 14:05 onward, all of 2026-07-23)

Prints throw/commit/catch-rate/fault counts for both groups, a verdict
breakdown (catch/possible/early - "early" is only reachable via the bypass,
so it's the direct measure of what the feature newly unlocks), and dumps a
JSON side-car of every committed throw's target point + outcome + verdict for
plot_catch_area_2.py to render.
"""
import glob
import json
import math
import sys
from collections import defaultdict

import numpy as np

sys.path.insert(0, "/home/erkka/codeprojects/OPTITRACK")

PRE_FILES = sorted(glob.glob("/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260721_*.jsonl")) + [
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_105755.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_105903.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_111640.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_113113.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_114222.jsonl",
]
POST_FILES = [
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_140544.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_164459.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_172652.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_173706.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_173719.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260722_173904.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260723_103647.jsonl",
    "/home/erkka/codeprojects/OPTITRACK/catch_logs/catch_log_20260723_105022.jsonl",
]


def load(path):
    return [json.loads(l) for l in open(path) if l.strip()]


def analyze_group(name, files):
    sessions = 0
    throws = commits = 0
    verdicts = defaultdict(int)
    caught = missed = unknown = 0
    caught_by_verdict = defaultdict(lambda: [0, 0])  # verdict -> [caught, total classifiable]
    guards = refuses_total = retargets = 0
    faults = []
    n_at_commit = []
    rows = []  # per-throw records for the plot

    for path in files:
        evs = load(path)
        run = next((e for e in evs if e["ev"] == "run_start"), None)
        if run is None or run.get("dry_run"):
            continue
        sessions += 1
        wait_xyz = run.get("wait_pose", [0] * 6)[:3]
        session_name = path.split("catch_log_")[-1].replace(".jsonl", "")

        fault_events = [e for e in evs if e["ev"] == "fault"]
        faults.extend({"session": session_name, **e} for e in fault_events)
        fault_walls = [e["wall"] for e in fault_events]

        by_throw = defaultdict(lambda: defaultdict(list))
        for e in evs:
            by_throw[e["throw"]][e["ev"]].append(e)

        for tn, kinds in sorted(by_throw.items()):
            if tn == 0 or not kinds.get("throw_start"):
                continue
            throws += 1
            guards += len(kinds.get("guard", []))
            refuses_total += len(kinds.get("refuse", []))
            retargets += len(kinds.get("retarget", []))
            commit = kinds["commit"][0] if kinds.get("commit") else None
            end = kinds["throw_end"][0] if kinds.get("throw_end") else None

            if commit:
                commits += 1
                v = commit.get("verdict", "?")
                verdicts[v] += 1
                tgt = commit.get("target_pose", [None] * 3)[:3]
                cg = end.get("caught_guess") if end else None
                if cg is True:
                    caught += 1
                    caught_by_verdict[v][0] += 1
                elif cg is False:
                    missed += 1
                else:
                    unknown += 1
                if cg is not None:
                    caught_by_verdict[v][1] += 1
                ticks = kinds.get("tick", [])
                ct = commit.get("t")
                pre = [tk for tk in ticks if tk.get("t") is not None and ct is not None and tk["t"] <= ct + 1e-9]
                n_commit = pre[-1].get("n") if pre else None
                if n_commit is not None:
                    n_at_commit.append(n_commit)
                faulted_near = any(abs(commit["wall"] - fw) <= 4.0 for fw in fault_walls)
                rows.append(dict(
                    session=session_name, throw=tn, verdict=v,
                    target_xy=tgt[:2] if tgt[0] is not None else None,
                    caught=cg, faulted_near=faulted_near,
                    n_commit=n_commit,
                    peak_speed=end.get("peak_speed") if end else None,
                    reaims=end.get("reaims") if end else None,
                ))

    print(f"\n=== {name} ({sessions} sessions, files: {len(files)}) ===")
    print(f"throws={throws} commits={commits} ({100*commits/max(1,throws):.0f}% commit rate) "
          f"guards={guards} refuses={refuses_total} retargets={retargets}")
    print(f"verdict breakdown: {dict(verdicts)}")
    classifiable = caught + missed
    if classifiable:
        print(f"catch rate (classifiable): {caught}/{classifiable} ({100*caught/classifiable:.0f}%)  "
              f"[{unknown} unclassifiable]")
    for v, (c, t) in sorted(caught_by_verdict.items()):
        if t:
            print(f"  verdict={v:9s}: caught {c}/{t} ({100*c/t:.0f}%)")
    if n_at_commit:
        a = sorted(n_at_commit)
        print(f"n@commit: min={a[0]} med={a[len(a)//2]} max={a[-1]}")
    if faults:
        print(f"faults: {len(faults)}")
        for f in faults:
            print(f"  session={f['session']} throw={f['throw']} reason={f['reason'][:90]}")
    return rows, faults


if __name__ == "__main__":
    pre_rows, pre_faults = analyze_group("PRE (feasibility-gated servo, 07-21 & 07-22 morning)", PRE_FILES)
    post_rows, post_faults = analyze_group("POST (early-commit servo, 07-22 afternoon & 07-23)", POST_FILES)

    with open("/home/erkka/codeprojects/OPTITRACK/analysis/early_commit_rows.json", "w") as f:
        json.dump({"pre": pre_rows, "post": post_rows,
                    "pre_faults": pre_faults, "post_faults": post_faults}, f, indent=1)
    print("\nsaved analysis/early_commit_rows.json")
