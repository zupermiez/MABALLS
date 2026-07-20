"""
Standalone mocap-only accuracy/noise check for a stationary marker - deliberately
NOT part of calibrate_frames.py. No robot connection, no set_tcp(), no TCP offset
at all: while a physical marker sits still, positional standard deviation over
time IS the tracking noise, full stop. This isolates whatever noise floor Motive/
NatNet gives for a given tracking source before blaming (or clearing) the robot
side of calibrate_frames.py's pipeline - see the RMSE investigation in
docs/debug_log.md and CLAUDE.md "Catch Integration".

Compares two sources for the same stationary setup, side by side:
  - a named Marker Set (frame.marker_sets, e.g. "Markerset 002") - a raw named
    grouping of markers, distinct from both a Rigid Body and a Motive "Marker"
    asset (see calibrate_frames.py's docstring for those two).
  - the plain unlabeled marker list (frame.unlabeled_marker_pos) - the same
    field live_view.py's "Unlabeled Markers" table reads.

Usage: python3 check_marker_accuracy.py --duration 15
(hold everything completely still for the duration - any real movement shows
up indistinguishably from tracking noise in this measurement)
"""

import argparse
import time

import numpy as np
from natnet import NatNetClient, DataFrame


def summarize(label, samples, total_frames):
    n = len(samples)
    pct = f"{n / total_frames * 100:.0f}%" if total_frames else "n/a"
    print(f"\n{label}: valid in {n}/{total_frames} frames ({pct})")
    if not samples:
        print("  no valid samples captured")
        return
    arr = np.array(samples)
    mean = arr.mean(axis=0)
    std = arr.std(axis=0)
    spread = arr.max(axis=0) - arr.min(axis=0)
    print(f"  mean pos = ({mean[0]:+.4f},{mean[1]:+.4f},{mean[2]:+.4f})")
    print(f"  per-axis std = ({std[0] * 1000:.2f},{std[1] * 1000:.2f},{std[2] * 1000:.2f}) mm   "
          f"3D std = {np.linalg.norm(std) * 1000:.2f} mm")
    print(f"  max-min spread per axis = ({spread[0] * 1000:.2f},{spread[1] * 1000:.2f},"
          f"{spread[2] * 1000:.2f}) mm")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--duration", type=float, default=10.0, help="seconds to sample (default 10)")
    parser.add_argument("--marker-set-name", default="Markerset 002",
                         help="frame.marker_sets model_name to track (default 'Markerset 002')")
    args = parser.parse_args()

    total_frames = 0
    ms_samples = []
    ul_samples = []

    def handle(frame: DataFrame) -> None:
        nonlocal total_frames
        total_frames += 1

        ms = next((m for m in frame.marker_sets if m.model_name == args.marker_set_name), None)
        if ms is not None and len(ms.marker_pos_list) == 1:
            ms_samples.append(ms.marker_pos_list[0])

        ul = frame.unlabeled_marker_pos
        if len(ul) == 1:
            ul_samples.append(ul[0])

    client = NatNetClient(server_ip_address=args.server_ip, local_ip_address=args.local_ip,
                           use_multicast=not args.unicast)
    client.on_data_frame_received_event.handlers.append(handle)
    print(f"Sampling for {args.duration:.0f}s - hold everything completely still...")
    with client:
        client.run_async()
        time.sleep(args.duration)
        client.stop_async()

    print(f"\n{total_frames} frames received")
    summarize(f"'{args.marker_set_name}' marker set (frame.marker_sets)", ms_samples, total_frames)
    summarize("unlabeled_marker_pos (exactly 1 present)", ul_samples, total_frames)

    if ms_samples and ul_samples:
        print("\nLower 3D std / tighter max-min spread = the more trustworthy source to "
              "calibrate against. If they're both sub-mm, the RMSE problem is elsewhere "
              "(offset measurement, or which physical point is actually being tracked) - "
              "not raw-marker noise.")


if __name__ == "__main__":
    main()
