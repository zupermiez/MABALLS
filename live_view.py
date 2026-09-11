"""
Live terminal view of NatNet rigid bodies + markers + measured frame rate.

Builds on the working setup documented in CLAUDE.md (Motive on 192.168.10.1,
multicast, NatNet 4.2). Uses the `natnet` PyPI client for parsing (don't hand-roll
a NatNet parser - see CLAUDE.md for why) and `rich` for the live display.
"""

import argparse
import threading
import time
from collections import deque
from typing import Optional

from natnet import NatNetClient, DataFrame
from rich.live import Live
from rich.table import Table
from rich.console import Group
from rich.panel import Panel

FPS_WINDOW = deque(maxlen=180)  # (wall_time, frame_number) samples for rate calc
STATE_LOCK = threading.Lock()
# Optional[...] rather than `DataFrame | None`: this annotation is evaluated at
# runtime (module-level AnnAssign), and PEP 604 unions need Python 3.10+ - this
# is the repo's only 3.10-ism, and it blocked the whole script on an Ubuntu
# 20.04 (Python 3.8) demo laptop.
LATEST_FRAME: Optional[DataFrame] = None


def handle_frame(frame: DataFrame) -> None:
    global LATEST_FRAME
    with STATE_LOCK:
        LATEST_FRAME = frame
        FPS_WINDOW.append((time.monotonic(), frame.prefix.frame_number))


def compute_rates():
    """Returns (packet_fps, motive_fps) measured over the trailing window."""
    with STATE_LOCK:
        samples = list(FPS_WINDOW)
    if len(samples) < 2:
        return 0.0, 0.0
    t0, f0 = samples[0]
    t1, f1 = samples[-1]
    dt = t1 - t0
    if dt <= 0:
        return 0.0, 0.0
    packet_fps = (len(samples) - 1) / dt
    motive_fps = (f1 - f0) / dt
    return packet_fps, motive_fps


def build_display(server_ip: str, local_ip: str, protocol_version) -> Group:
    with STATE_LOCK:
        frame = LATEST_FRAME

    packet_fps, motive_fps = compute_rates()

    header = Panel(
        f"server={server_ip}  local={local_ip}  protocol={protocol_version}\n"
        f"stream fps (packets/sec received): [bold]{packet_fps:5.1f}[/bold]   "
        f"motive fps (frame-number delta/sec): [bold]{motive_fps:5.1f}[/bold]",
        title="NatNet Live View",
    )

    if frame is None:
        return Group(header, Panel("Waiting for first frame..."))

    rb_table = Table(title=f"Rigid Bodies ({len(frame.rigid_bodies)})")
    for col in ("id", "x", "y", "z", "valid", "mean_error"):
        rb_table.add_column(col)
    for rb in frame.rigid_bodies:
        x, y, z = rb.pos
        rb_table.add_row(
            str(rb.id_num),
            f"{x:+.4f}",
            f"{y:+.4f}",
            f"{z:+.4f}",
            "yes" if rb.tracking_valid else "NO",
            f"{rb.marker_error:.5f}" if rb.marker_error is not None else "-",
        )

    ms_table = Table(title=f"Marker Sets ({len(frame.marker_sets)})")
    for col in ("name", "n_markers"):
        ms_table.add_column(col)
    for ms in frame.marker_sets:
        ms_table.add_row(ms.model_name, str(len(ms.marker_pos_list)))

    unlabeled = frame.unlabeled_marker_pos
    unlabeled_table = Table(title=f"Unlabeled Markers ({len(unlabeled)})")
    for col in ("#", "x", "y", "z"):
        unlabeled_table.add_column(col)
    for i, (x, y, z) in enumerate(unlabeled[:10]):
        unlabeled_table.add_row(str(i), f"{x:+.4f}", f"{y:+.4f}", f"{z:+.4f}")
    if len(unlabeled) > 10:
        unlabeled_table.caption = f"... and {len(unlabeled) - 10} more"

    return Group(header, rb_table, ms_table, unlabeled_table)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument(
        "--duration", type=float, default=None,
        help="Exit automatically after N seconds (default: run until Ctrl+C)",
    )
    args = parser.parse_args()

    client = NatNetClient(
        server_ip_address=args.server_ip,
        local_ip_address=args.local_ip,
        use_multicast=not args.unicast,
    )
    client.on_data_frame_received_event.handlers.append(handle_frame)

    with client:
        client.run_async()
        start = time.monotonic()
        try:
            with Live(refresh_per_second=10) as live:
                while args.duration is None or (time.monotonic() - start) < args.duration:
                    live.update(build_display(args.server_ip, args.local_ip, client.protocol_version))
                    time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            client.stop_async()

    packet_fps, motive_fps = compute_rates()
    print(f"\nFinal: stream_fps={packet_fps:.1f} motive_fps={motive_fps:.1f}")
    if LATEST_FRAME is not None:
        for rb in LATEST_FRAME.rigid_bodies:
            print(f"  rigid_body id={rb.id_num} pos={rb.pos} valid={rb.tracking_valid}")


if __name__ == "__main__":
    main()
