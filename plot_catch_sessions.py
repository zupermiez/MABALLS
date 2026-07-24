"""General-purpose catch-zone plotter: reads catch_logs/*.jsonl directly (no
analysis-script/JSON intermediate needed - point it at a glob and it draws) and
scatters each committed throw's target point over the real catch envelope
(same geometry catch.py enforces: reach_band_at_z + CATCH_MAX_AZIMUTH_DEG).

One panel, everything:
    python3 plot_catch_sessions.py

One session / one day (glob matches the catch_log_<timestamp>.jsonl filenames,
matched against just the <timestamp> part, e.g. "20260723*" or "20260723_1050*"):
    python3 plot_catch_sessions.py --sessions "20260723*"

Whole history, one calendar day per panel:
    python3 plot_catch_sessions.py --by-day

Side-by-side comparison of two named groups of sessions (repeat --panel,
comma-separate multiple globs per panel) - e.g. reproducing plot_catch_area_2.py:
    python3 plot_catch_sessions.py \\
        --panel "PRE=20260721*,20260722_105755,20260722_105903,20260722_111640,20260722_113113,20260722_114222" \\
        --panel "POST=20260722_140544,20260722_164459,20260722_172652,20260722_173706,20260722_173719,20260722_173904,20260723_103647,20260723_105022"

Filter by verdict or exclude faulted-near throws:
    python3 plot_catch_sessions.py --verdict catch,possible
    python3 plot_catch_sessions.py --exclude-faulted

Tweak freely - this is meant to be a quick base to edit, not a finished tool.
"""
import argparse
import fnmatch
import glob
import json
import math
import os

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

from catch import CATCH_MAX_AZIMUTH_DEG, DEFAULT_WAIT_POSE, reach_band_at_z

LOG_DIR = os.path.join(os.path.dirname(__file__), "catch_logs")

VERDICT_MARKER = {"catch": "o", "possible": "s", "early": "*"}
OUTCOME_COLOR = {True: "#16a34a", False: "#dc2626", None: "#94a3b8"}


def session_name(path):
    return os.path.basename(path).replace("catch_log_", "").replace(".jsonl", "")


def load_throws(paths):
    """One row per committed throw across the given session files."""
    rows = []
    for path in paths:
        evs = [json.loads(l) for l in open(path) if l.strip()]
        run = next((e for e in evs if e["ev"] == "run_start"), None)
        if run is None or run.get("dry_run"):
            continue
        fault_walls = [e["wall"] for e in evs if e["ev"] == "fault"]

        by_throw = {}
        for e in evs:
            by_throw.setdefault(e["throw"], {}).setdefault(e["ev"], []).append(e)

        sname = session_name(path)
        for tn, kinds in sorted(by_throw.items()):
            if tn == 0 or not kinds.get("commit"):
                continue
            commit = kinds["commit"][0]
            end = kinds["throw_end"][0] if kinds.get("throw_end") else None
            tgt = commit.get("target_pose")
            if not tgt:
                continue
            rows.append(dict(
                session=sname, throw=tn, verdict=commit.get("verdict", "?"),
                target_xy=tgt[:2], caught=end.get("caught_guess") if end else None,
                faulted_near=any(abs(commit["wall"] - fw) <= 4.0 for fw in fault_walls),
            ))
    return rows


def resolve_globs(patterns):
    all_files = sorted(glob.glob(os.path.join(LOG_DIR, "catch_log_*.jsonl")))
    by_name = {session_name(p): p for p in all_files}
    matched = []
    for pat in patterns:
        hits = [p for name, p in by_name.items() if fnmatch.fnmatch(name, pat)]
        if not hits:
            print(f"warning: pattern {pat!r} matched no sessions")
        matched.extend(hits)
    return sorted(set(matched))


def draw_envelope(ax, wait_xyz, title):
    z = wait_xyz[2]
    wait_az_deg = math.degrees(math.atan2(wait_xyz[1], wait_xyz[0]))
    h_min, h_max = reach_band_at_z(z)
    az_lo, az_hi = wait_az_deg - CATCH_MAX_AZIMUTH_DEG, wait_az_deg + CATCH_MAX_AZIMUTH_DEG

    thetas = np.radians(np.linspace(az_lo, az_hi, 100))
    outer = np.stack([h_max * np.cos(thetas), h_max * np.sin(thetas)], axis=1)
    inner = np.stack([h_min * np.cos(thetas[::-1]), h_min * np.sin(thetas[::-1])], axis=1)
    ax.add_patch(patches.Polygon(np.concatenate([outer, inner]), closed=True, facecolor="#3b82f6",
                                  alpha=0.15, edgecolor="#1d4ed8", linewidth=2))
    for r, ls in ((h_min, ":"), (h_max, "--")):
        ax.add_patch(patches.Circle((0, 0), r, fill=False, edgecolor="#94a3b8", linestyle=ls, linewidth=1))
    ax.plot(0, 0, marker="s", markersize=12, color="#111827", zorder=5)
    for az in (az_lo, az_hi):
        rad = math.radians(az)
        ax.plot([0, h_max * 1.15 * math.cos(rad)], [0, h_max * 1.15 * math.sin(rad)],
                 color="#6b7280", linestyle="--", linewidth=1)
    ax.plot(wait_xyz[0], wait_xyz[1], marker="o", markersize=8, color="#dc2626", zorder=5)
    ax.annotate("wait pose", (wait_xyz[0], wait_xyz[1]), textcoords="offset points",
                xytext=(8, -14), fontsize=8, color="#dc2626")

    lim = h_max * 1.3
    ax.set_aspect("equal")
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_xlabel("base-frame X (m)")
    ax.set_ylabel("base-frame Y (m)")
    ax.set_title(title)
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.5)


def scatter_rows(ax, rows):
    for r in rows:
        x, y = r["target_xy"]
        marker = VERDICT_MARKER.get(r["verdict"], "o")
        color = OUTCOME_COLOR.get(r["caught"])
        size = 130 if r["verdict"] == "early" else 55
        edge = "black" if r["faulted_near"] else "none"
        lw = 1.4 if r["faulted_near"] else 0
        ax.scatter(x, y, marker=marker, s=size, c=color, edgecolors=edge, linewidths=lw,
                   alpha=0.85, zorder=6)


def legend_handles():
    return [
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#16a34a", markersize=10, label="caught"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#dc2626", markersize=10, label="missed"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#94a3b8", markersize=10, label="unclassified"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="gray", markersize=10, label="verdict=catch"),
        plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="gray", markersize=9, label="verdict=possible"),
        plt.Line2D([0], [0], marker="*", color="w", markerfacecolor="gray", markersize=16, label="verdict=early"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="none", markeredgecolor="black",
                   markeredgewidth=1.4, markersize=10, label="fault within ±4s of commit"),
    ]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sessions", default="*", help="comma-separated glob(s) over session timestamps "
                     "(default: everything in catch_logs/)")
    ap.add_argument("--panel", action="append", default=None,
                     help="name=glob[,glob...] - repeat for multiple side-by-side panels; overrides --sessions")
    ap.add_argument("--by-day", action="store_true", help="one panel per calendar day (YYYYMMDD prefix) found")
    ap.add_argument("--verdict", default=None, help="comma-separated verdicts to include (default: all)")
    ap.add_argument("--exclude-faulted", action="store_true", help="drop throws faulted within +/-4s of commit")
    ap.add_argument("--wait-pose", nargs=6, type=float, default=list(DEFAULT_WAIT_POSE),
                     help="override wait pose (default: catch.py's DEFAULT_WAIT_POSE)")
    ap.add_argument("--title", default="Catch zone: committed-throw locations")
    ap.add_argument("--out", default="catch_sessions.png")
    args = ap.parse_args()

    wait_xyz = np.array(args.wait_pose[:3])
    verdict_filter = set(args.verdict.split(",")) if args.verdict else None

    if args.panel:
        panels = []
        for spec in args.panel:
            name, _, globs = spec.partition("=")
            panels.append((name, resolve_globs(globs.split(","))))
    elif args.by_day:
        all_files = sorted(glob.glob(os.path.join(LOG_DIR, "catch_log_*.jsonl")))
        days = sorted({session_name(p)[:8] for p in all_files})
        panels = [(day, resolve_globs([f"{day}*"])) for day in days]
    else:
        panels = [("all sessions", resolve_globs(args.sessions.split(",")))]

    fig, axes = plt.subplots(1, len(panels), figsize=(8 * len(panels), 8), squeeze=False)
    axes = axes[0]

    for ax, (name, paths) in zip(axes, panels):
        rows = load_throws(paths)
        if verdict_filter:
            rows = [r for r in rows if r["verdict"] in verdict_filter]
        if args.exclude_faulted:
            rows = [r for r in rows if not r["faulted_near"]]
        draw_envelope(ax, wait_xyz, f"{name}\n({len(paths)} sessions, {len(rows)} commits)")
        scatter_rows(ax, rows)

    fig.legend(handles=legend_handles(), loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle(args.title, fontsize=13)
    fig.tight_layout(rect=[0, 0.06, 1, 0.96])
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
