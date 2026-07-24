"""catch_area_2.png - the catch-zone diagram (same geometry as draw_catch_area.py:
catch.py's actual reach/azimuth envelope, not eyeballed) with real committed-throw
locations overlaid, split PRE vs POST the 2026-07-22/23 early-commit servo feature.

Reads analysis/early_commit_rows.json (produced by analysis/analyze_early_commit.py).
Run from the repo root: python3 plot_catch_area_2.py
"""
import json
import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from catch import CATCH_MAX_AZIMUTH_DEG, reach_band_at_z, DEFAULT_WAIT_POSE

with open("analysis/early_commit_rows.json") as f:
    data = json.load(f)

wait_xyz = np.array(DEFAULT_WAIT_POSE[:3])
z = wait_xyz[2]
wait_az_deg = math.degrees(math.atan2(wait_xyz[1], wait_xyz[0]))
h_min, h_max = reach_band_at_z(z)
az_lo, az_hi = wait_az_deg - CATCH_MAX_AZIMUTH_DEG, wait_az_deg + CATCH_MAX_AZIMUTH_DEG

fig, axes = plt.subplots(1, 2, figsize=(16, 8))

VERDICT_MARKER = {"catch": "o", "possible": "s", "early": "*"}
OUTCOME_COLOR = {True: "#16a34a", False: "#dc2626", None: "#94a3b8"}


def draw_zone(ax, title):
    thetas = np.radians(np.linspace(az_lo, az_hi, 100))
    outer = np.stack([h_max * np.cos(thetas), h_max * np.sin(thetas)], axis=1)
    inner = np.stack([h_min * np.cos(thetas[::-1]), h_min * np.sin(thetas[::-1])], axis=1)
    ring_pts = np.concatenate([outer, inner])
    ax.add_patch(patches.Polygon(ring_pts, closed=True, facecolor="#3b82f6", alpha=0.15,
                                  edgecolor="#1d4ed8", linewidth=2, label="catch envelope"))
    for r, ls in ((h_min, ":"), (h_max, "--")):
        ax.add_patch(patches.Circle((0, 0), r, fill=False, edgecolor="#94a3b8", linestyle=ls, linewidth=1))
    ax.plot(0, 0, marker="s", markersize=12, color="#111827", zorder=5)
    for az, ls in ((az_lo, "--"), (az_hi, "--")):
        rad = math.radians(az)
        ax.plot([0, h_max * 1.15 * math.cos(rad)], [0, h_max * 1.15 * math.sin(rad)],
                color="#6b7280", linestyle=ls, linewidth=1)
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


def scatter_rows(ax, rows, verdict_filter=None):
    for r in rows:
        if r["target_xy"] is None:
            continue
        if verdict_filter and r["verdict"] not in verdict_filter:
            continue
        x, y = r["target_xy"]
        marker = VERDICT_MARKER.get(r["verdict"], "o")
        color = OUTCOME_COLOR.get(r["caught"])
        size = 130 if r["verdict"] == "early" else 55
        edge = "black" if r.get("faulted_near") else "none"
        lw = 1.4 if r.get("faulted_near") else 0
        ax.scatter(x, y, marker=marker, s=size, c=color, edgecolors=edge, linewidths=lw,
                   alpha=0.85, zorder=6)


draw_zone(axes[0], f"PRE early-commit\n(feasibility-gated servo, 07-21 & 07-22 AM, n={len(data['pre'])} commits)")
scatter_rows(axes[0], data["pre"])

draw_zone(axes[1], f"POST early-commit\n(gate bypassed, 07-22 PM & 07-23, n={len(data['post'])} commits)")
scatter_rows(axes[1], data["post"])

legend_elems = [
    plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#16a34a", markersize=10, label="caught"),
    plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#dc2626", markersize=10, label="missed"),
    plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#94a3b8", markersize=10, label="unclassified"),
    plt.Line2D([0], [0], marker="*", color="w", markerfacecolor="gray", markersize=16,
               label="verdict=EARLY (only fires via gate bypass)"),
    plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="gray", markersize=10, label="verdict=catch"),
    plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="gray", markersize=9, label="verdict=possible"),
    plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="none", markeredgecolor="black",
               markeredgewidth=1.4, markersize=10, label="fault within ±4s of commit"),
]
fig.legend(handles=legend_elems, loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, -0.02))
fig.suptitle("Catch zone: committed-throw locations before vs after the early-commit servo feature", fontsize=13)
fig.tight_layout(rect=[0, 0.06, 1, 0.96])

out = "catch_area_2.png"
fig.savefig(out, dpi=150, bbox_inches="tight")
print(f"saved {out}")
