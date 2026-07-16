"""
Minimal reference viewer: plots the tracked rigid body's raw position in the
same two 2D views as visualize_trajectory.py (top-down x-z, side view of
height vs. distance from the last clear point) - nothing else. No release
detection, no fitting, no prediction. Useful as a fast, low-overhead sanity
check of the raw tracking data on its own.

Press space to clear the plot.
"""

import argparse
import threading

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from natnet import NatNetClient

AXIS_NAMES = ("x", "y", "z")

LOCK = threading.Lock()
POINTS = []  # list of (x, y, z), appended by the NatNet callback
TARGET_ID = None
CANDIDATE_IDS = []


def build_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--server-ip", default="192.168.10.1")
    p.add_argument("--local-ip", default="192.168.10.2")
    p.add_argument("--unicast", action="store_true")
    p.add_argument("--rigid-body-id", type=int, default=None)
    p.add_argument("--up-axis", choices=AXIS_NAMES, default="y", help="Axis used as height in the side view")
    p.add_argument("--fps", type=float, default=30.0)
    return p.parse_args()


def make_handler(args):
    def handle_frame(frame):
        global TARGET_ID
        with LOCK:
            if TARGET_ID is None:
                ids = [rb.id_num for rb in frame.rigid_bodies]
                if len(ids) == 1:
                    TARGET_ID = ids[0]
                else:
                    CANDIDATE_IDS[:] = ids
                return
            rb = next((r for r in frame.rigid_bodies if r.id_num == TARGET_ID), None)
            if rb is None:
                return
            valid = True if rb.tracking_valid is None else rb.tracking_valid
            if not valid:
                return
            POINTS.append(rb.pos)

    return handle_frame


def main():
    args = build_args()
    if args.rigid_body_id is not None:
        global TARGET_ID
        TARGET_ID = args.rigid_body_id
    up_idx = AXIS_NAMES.index(args.up_axis)

    client = NatNetClient(
        server_ip_address=args.server_ip,
        local_ip_address=args.local_ip,
        use_multicast=not args.unicast,
    )
    client.on_data_frame_received_event.handlers.append(make_handler(args))

    fig, (ax_top, ax_side) = plt.subplots(1, 2, figsize=(12, 5.5))
    ax_top.set_title("Top view (x - z)")
    ax_top.set_xlabel("x (m)")
    ax_top.set_ylabel("z (m)")
    ax_side.set_title("Side view (distance from last clear - height)")
    ax_side.set_xlabel("horizontal distance (m)")
    ax_side.set_ylabel(f"{args.up_axis} (m)")
    fig.suptitle("Raw ball position (space to clear)", fontsize=12)

    (top_line,) = ax_top.plot([], [], "-o", color="#2a78d6", markersize=2, linewidth=1.5)
    (side_line,) = ax_side.plot([], [], "-o", color="#2a78d6", markersize=2, linewidth=1.5)

    def on_key(event):
        if event.key == " ":
            with LOCK:
                POINTS.clear()

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        with LOCK:
            pts = list(POINTS)

        if not pts:
            return top_line, side_line

        x0, y0, z0 = pts[0]
        xs = [p[0] for p in pts]
        zs = [p[2] for p in pts]
        dists = [((p[0] - x0) ** 2 + (p[2] - z0) ** 2) ** 0.5 for p in pts]
        vals = [p[up_idx] for p in pts]

        top_line.set_data(xs, zs)
        side_line.set_data(dists, vals)

        ax_top.relim()
        ax_top.autoscale_view()
        ax_side.relim()
        ax_side.autoscale_view()

        return top_line, side_line

    with client:
        client.run_async()
        anim = FuncAnimation(fig, update, interval=1000.0 / args.fps, blit=False, cache_frame_data=False)
        try:
            plt.show()
        finally:
            client.stop_async()


if __name__ == "__main__":
    main()
