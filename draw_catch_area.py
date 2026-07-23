"""Throwaway top-down diagram of the floor area to tape as the catch zone.

Draws in the robot BASE FRAME (top-down, looking down the base +Z axis), using
the same constants/functions catch.py actually enforces - not eyeballed numbers.
Re-run any time those constants change (wait pose, azimuth band, reach envelope)
to get an up-to-date picture. Run from the repo root: python3 draw_catch_area.py
"""
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

from catch import CATCH_MAX_AZIMUTH_DEG, reach_band_at_z, DEFAULT_WAIT_POSE

wait_xyz = np.array(DEFAULT_WAIT_POSE[:3])
z = wait_xyz[2]
wait_az_deg = math.degrees(math.atan2(wait_xyz[1], wait_xyz[0]))
h_min, h_max = reach_band_at_z(z)
az_lo, az_hi = wait_az_deg - CATCH_MAX_AZIMUTH_DEG, wait_az_deg + CATCH_MAX_AZIMUTH_DEG

fig, ax = plt.subplots(figsize=(8, 8))
ax.set_aspect("equal")

# Ring sector (catch zone): fill the wedge between h_min and h_max, az_lo..az_hi
thetas = np.radians(np.linspace(az_lo, az_hi, 100))
outer = np.stack([h_max * np.cos(thetas), h_max * np.sin(thetas)], axis=1)
inner = np.stack([h_min * np.cos(thetas[::-1]), h_min * np.sin(thetas[::-1])], axis=1)
ring_pts = np.concatenate([outer, inner])
ax.add_patch(patches.Polygon(ring_pts, closed=True, facecolor="#3b82f6", alpha=0.25,
                              edgecolor="#1d4ed8", linewidth=2, label="catch zone (tape this)"))

# Full reference circles (dashed, faint) for the inner/outer radius, all the way around
for r, ls in ((h_min, ":"), (h_max, "--")):
    ax.add_patch(patches.Circle((0, 0), r, fill=False, edgecolor="#94a3b8", linestyle=ls, linewidth=1))

# Robot base marker + azimuth reference lines
ax.plot(0, 0, marker="s", markersize=14, color="#111827", zorder=5)
ax.annotate("robot base\n(pedestal center)", (0, 0), textcoords="offset points",
            xytext=(0, 14), ha="center", fontsize=9)

for az, label, style in ((wait_az_deg, "wait-pose centerline", "-"),
                          (az_lo, f"{-CATCH_MAX_AZIMUTH_DEG:.0f} deg", "--"),
                          (az_hi, f"+{CATCH_MAX_AZIMUTH_DEG:.0f} deg", "--")):
    rad = math.radians(az)
    ax.plot([0, h_max * 1.15 * math.cos(rad)], [0, h_max * 1.15 * math.sin(rad)],
            color="#6b7280", linestyle=style, linewidth=1)

# Wait pose point itself (where the parked funnel hovers)
ax.plot(wait_xyz[0], wait_xyz[1], marker="o", markersize=8, color="#dc2626", zorder=5)
ax.annotate("parked funnel\n(plumb this to the floor\nfor your centerline anchor)",
            (wait_xyz[0], wait_xyz[1]), textcoords="offset points", xytext=(10, -25),
            fontsize=8, color="#dc2626")

# Radius labels along the centerline
rad_c = math.radians(wait_az_deg)
for r in (h_min, h_max):
    ax.annotate(f"{r:.2f} m", (r * math.cos(rad_c), r * math.sin(rad_c)),
                textcoords="offset points", xytext=(6, 6), fontsize=9, color="#1d4ed8")

lim = h_max * 1.3
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_xlabel("base-frame X (m)")
ax.set_ylabel("base-frame Y (m)")
ax.set_title(f"Catch zone, top-down (base frame)\n"
             f"ring {h_min:.2f}-{h_max:.2f} m, +/-{CATCH_MAX_AZIMUTH_DEG:.0f} deg off wait-pose azimuth "
             f"({wait_az_deg:.0f} deg)")
ax.legend(loc="upper right", fontsize=8)
ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.5)

out = "catch_area.png"
fig.savefig(out, dpi=150, bbox_inches="tight")
print(f"saved {out}")
