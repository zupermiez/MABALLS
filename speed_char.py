"""
Characterize the UR12e's *achievable* TCP speed vs the *commanded* speed, to
find the real ceiling (CLAUDE.md: ~1 m/s effective, confirmed empirically -
this script is how you re-derive that number and watch it plateau live).

Why this shape of experiment (from the 2026-07-14 request):
  - A `movel` only reaches its commanded cruise speed if the move is long
    enough to finish accelerating before it has to start decelerating. A short
    move is a triangular velocity profile that never touches the commanded v.
    So for each commanded speed we run several moves of INCREASING distance
    ("further and further") all starting from the same point (pose A), and take
    the fastest peak achieved across them - the longest move is the one that
    actually reaches cruise.
  - Then bump the commanded speed (default +0.20 m/s) and repeat.
  - Acceleration is pinned HIGH (default 40 m/s^2) so accel is never the
    bottleneck. The controller clamps this to its own true max internally;
    setting it high just guarantees the arm ramps as fast as the hardware
    allows, so what limits the measured peak is the velocity ceiling we're
    hunting, not the ramp and not the distance. (CLAUDE.md: no clean published
    accel ceiling; pushing accel past a few x the speed has diminishing returns
    but is accepted - a real minimum ramp time exists regardless.)

The headline plot is commanded speed (x) vs measured peak TCP speed (y), one
line per move distance, with a dashed ideal y=x. Where the top line falls away
from y=x and flattens is the speed ceiling: bigger speed commands stop buying
bigger actual speed. A CLI table with the same numbers prints regardless, so
`--no-plot` is a first-class mode if matplotlib ever bottlenecks the loop.

Motion path is the proven raw-URScript-over-socket route (same as ur_loop.py /
ur_goto_raw.py); rtde_receive is read-only (TCP speed/pose). No ur_rtde control
session is held, so there's no stranded-realtime-thread hazard on Ctrl-C.

SAFETY MODEL: every move is a pure translation along the A->B segment you
captured with ur_get_pose.py (orientation held at A's). Both endpoints are
operator-verified reachable/non-singular, and no target ever leaves that
segment, so the per-axis clamp in ur_goto_raw.py is deliberately bypassed here
(like ur_loop.py's force=True legs) - the segment IS the safety envelope. The
one move that could surprise you is the very first one, from wherever the arm
is now to pose A; that's gated behind --force-start, same as ur_loop.py.
Operator is expected to be at the physical E-stop the whole run.

Usage:
  python3 speed_char.py --pose-a <6 vals> --pose-b <6 vals> --force-start
Get the two poses by jogging the arm to each end of a long, clear, straight,
non-singular path and running ur_get_pose.py.
"""
import argparse
import json
import math
import time
from datetime import datetime

import numpy as np
import rtde_receive

from ur_goto_raw import ROBOT_IP, movel_absolute_script, send_script

# --- leg measurement tuning ---
MOVE_START_SPEED = 0.02   # m/s - above this, the leg is considered to have begun moving
SETTLE_SPEED = 0.005      # m/s - below this counts as stopped (a touch looser than
                          #       ur_goto_raw's 0.001 so high-accel stop jitter doesn't hang us)
SETTLE_TICKS = 8          # consecutive slow samples before declaring "arrived"
START_TIMEOUT = 3.0       # s - if it hasn't started moving by now, something's wrong (slider? protective stop?)
LEG_TIMEOUT = 15.0        # s - hard backstop for one leg
POLL_DT = 0.002           # s - ~500 Hz, matches the e-series RTDE stream rate


def linear_speed(tcp_speed):
    """Magnitude of the translational part of getActualTCPSpeed() (ignores rotational)."""
    return math.sqrt(tcp_speed[0] ** 2 + tcp_speed[1] ** 2 + tcp_speed[2] ** 2)


def target_at(pose_a, direction, dist):
    """Point `dist` metres from A along the A->B direction, keeping A's orientation."""
    return [pose_a[i] + direction[i] * dist for i in range(3)] + list(pose_a[3:])


def stop_script():
    return "def prog():\n  stopl(3.0)\nend\nprog()\n"


def run_move(rtde_r, target, speed, accel, capture_peak):
    """
    Fire one movel and poll the TCP speed at ~500 Hz until it settles.
    Returns dict with peak linear speed and timing. Reused for the homing moves
    too (capture_peak=False just means we don't care about the peak there).
    """
    send_script(movel_absolute_script(target, speed, accel))
    t_send = time.perf_counter()
    started = False
    t_start = None
    peak = 0.0
    slow = 0
    while True:
        now = time.perf_counter()
        v = linear_speed(rtde_r.getActualTCPSpeed())
        if v > peak:
            peak = v
        if not started:
            if v > MOVE_START_SPEED:
                started, t_start = True, now
            elif now - t_send > START_TIMEOUT:
                return {"peak": peak, "move_time": None, "flight_time": None,
                        "started": False, "settled": False}
        else:
            if v < SETTLE_SPEED:
                slow += 1
                if slow >= SETTLE_TICKS:
                    return {"peak": peak, "move_time": now - t_send,
                            "flight_time": now - t_start, "started": True, "settled": True}
            else:
                slow = 0
        if now - t_send > LEG_TIMEOUT:
            return {"peak": peak, "move_time": now - t_send,
                    "flight_time": (now - t_start) if t_start else None,
                    "started": started, "settled": False}
        time.sleep(POLL_DT)


def ascii_bar(value, scale, width=40):
    n = int(round((value / scale) * width)) if scale > 0 else 0
    n = max(0, min(width, n))
    return "#" * n + " " * (width - n)


def make_plot():
    try:
        import matplotlib.pyplot as plt
        plt.ion()
        fig, ax = plt.subplots(figsize=(8, 6))
        return plt, fig, ax
    except Exception as e:
        print(f"[plot] matplotlib unavailable ({e}) - continuing CLI-only")
        return None, None, None


def redraw(plt, ax, series, distances, axis_max):
    ax.clear()
    for di, d in enumerate(distances):
        if not series[di]:
            continue
        xs = [p[0] for p in series[di]]
        ys = [p[1] for p in series[di]]
        ax.plot(xs, ys, marker="o", label=f"{d:.2f} m")
    ax.plot([0, axis_max], [0, axis_max], "k--", alpha=0.4, label="ideal (achieved = commanded)")
    ax.set_xlim(0, axis_max)
    ax.set_ylim(0, axis_max)
    ax.set_xlabel("commanded TCP speed (m/s)")
    ax.set_ylabel("measured peak TCP speed (m/s)")
    ax.set_title("UR12e commanded vs achieved TCP speed\n(top line flattening = real speed ceiling)")
    ax.legend(title="move distance", fontsize=8, loc="upper left")
    ax.grid(True, alpha=0.3)
    plt.pause(0.001)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pose-a", type=float, nargs=6, required=True,
                   metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                   help="start pose - the fixed point every measured move launches from")
    p.add_argument("--pose-b", type=float, nargs=6, required=True,
                   metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                   help="far endpoint - defines the direction and the max reachable distance")
    p.add_argument("--v-start", type=float, default=0.2, help="first commanded speed, m/s (default 0.2)")
    p.add_argument("--v-step", type=float, default=0.2, help="commanded-speed increment per iteration, m/s (default 0.2)")
    p.add_argument("--v-max", type=float, default=1.6, help="hard cap on commanded speed, m/s (default 1.6)")
    p.add_argument("--accel", type=float, default=40.0,
                   help="commanded accel, m/s^2 (default 40 - high on purpose so accel never bottlenecks; controller clamps it)")
    p.add_argument("--n-dist", type=int, default=5, help="distances tried per commanded speed (default 5)")
    p.add_argument("--d-min", type=float, default=0.10, help="shortest test distance, m (default 0.10)")
    p.add_argument("--home-speed", type=float, default=0.3, help="speed for returning to A between legs, m/s (default 0.3)")
    p.add_argument("--home-accel", type=float, default=1.5, help="accel for returning to A between legs, m/s^2 (default 1.5)")
    p.add_argument("--dwell", type=float, default=0.25, help="pause at A before each measured leg, s (default 0.25)")
    p.add_argument("--plateau-tol", type=float, default=0.03,
                   help="if best achieved speed grows by less than this per step for --plateau-count steps, stop (m/s, default 0.03)")
    p.add_argument("--plateau-count", type=int, default=2, help="consecutive flat steps to declare the ceiling (default 2)")
    p.add_argument("--force-start", action="store_true",
                   help="acknowledge the initial move from the arm's current position to pose A (required, same gate as ur_loop.py)")
    p.add_argument("--no-plot", action="store_true", help="skip the live graph, CLI table only")
    args = p.parse_args()

    a = list(args.pose_a)
    b = list(args.pose_b)
    seg = np.array(b[:3]) - np.array(a[:3])
    L = float(np.linalg.norm(seg))
    if L < args.d_min:
        raise SystemExit(f"A->B distance {L:.3f} m is shorter than --d-min {args.d_min} m - pick a longer segment.")
    if L < 0.25:
        print(f"WARNING: A->B is only {L:.3f} m. Reaching high cruise speeds needs room; "
              f"the top line may plateau on distance, not the real speed ceiling.")
    direction = (seg / L).tolist()
    distances = np.linspace(args.d_min, L, args.n_dist).tolist()

    if not args.force_start:
        raise SystemExit(
            "Refusing to start: the first move goes from the arm's current position to pose A, "
            "which may be far away. Confirm pose A is correct, then pass --force-start."
        )

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    # CLAUDE.md: a silently-reduced pendant speed slider caps actual speed and
    # would corrupt this entire experiment - check it before trusting any number.
    frac = rtde_r.getTargetSpeedFraction()
    if frac < 0.99:
        print(f"WARNING: pendant speed slider at {frac*100:.0f}% - it silently caps TCP speed. "
              f"Set it to 100% or every measured peak here is meaningless.")

    print(f"A->B segment: {L:.3f} m")
    print(f"test distances: {[round(d,3) for d in distances]} m")
    print(f"commanded speeds: {args.v_start:.2f} -> {args.v_max:.2f} m/s in {args.v_step:.2f} steps, accel={args.accel} m/s^2")
    print("moving to pose A to begin...", flush=True)
    home = run_move(rtde_r, a, args.home_speed, args.home_accel, capture_peak=False)
    if not home["settled"]:
        print("WARNING: did not settle at A cleanly - check the pendant before continuing.")

    plt = fig = ax = None
    if not args.no_plot:
        plt, fig, ax = make_plot()
    plot_on = plt is not None

    series = [[] for _ in distances]   # series[distance_index] -> list of (v_cmd, peak)
    all_legs = []                      # full log for JSON (feeds distance->move_time later too)
    envelope = []                      # (v_cmd, best peak across distances)
    axis_max = args.v_max + 0.1

    v = args.v_start
    prev_best = None
    flat_streak = 0
    interrupted = False
    try:
        while v <= args.v_max + 1e-9:
            print(f"\n=== commanded {v:.2f} m/s ===")
            best = 0.0
            for di, d in enumerate(distances):
                # always relaunch from A
                run_move(rtde_r, a, args.home_speed, args.home_accel, capture_peak=False)
                if args.dwell > 0:
                    time.sleep(args.dwell)
                tgt = target_at(a, direction, d)
                leg = run_move(rtde_r, tgt, v, args.accel, capture_peak=True)
                peak = leg["peak"]
                best = max(best, peak)
                series[di].append((v, peak))
                all_legs.append({"v_cmd": v, "distance": d, "peak": peak,
                                 "move_time": leg["move_time"], "flight_time": leg["flight_time"],
                                 "started": leg["started"], "settled": leg["settled"]})
                flag = "" if leg["started"] else "  <-- NEVER MOVED (slider? protective stop?)"
                mt = f"{leg['move_time']:.2f}s" if leg["move_time"] else "  -  "
                print(f"  d={d:0.3f} m  peak={peak:0.3f} m/s  t={mt}{flag}")
                if plot_on:
                    redraw(plt, ax, series, distances, axis_max)

            ratio = best / v if v > 0 else 0.0
            envelope.append((v, best))
            print(f"  -> best achieved {best:0.3f} m/s  (ratio {ratio:0.2f})  "
                  f"|{ascii_bar(best, axis_max)}|")

            if prev_best is not None and (best - prev_best) < args.plateau_tol:
                flat_streak += 1
            else:
                flat_streak = 0
            prev_best = best
            if flat_streak >= args.plateau_count:
                print(f"\nPLATEAU: commanding faster stopped raising the achieved speed "
                      f"(<{args.plateau_tol} m/s gain for {args.plateau_count} steps). "
                      f"Effective TCP ceiling ~= {best:0.3f} m/s. Stopping sweep.")
                break
            v += args.v_step
    except KeyboardInterrupt:
        interrupted = True
        print("\nCtrl-C - sending stopl to decelerate, then holding.")
        try:
            send_script(stop_script())
        except Exception:
            pass
    finally:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_json = f"speed_char_{stamp}.json"
        with open(out_json, "w") as f:
            json.dump({"created": datetime.now().isoformat(), "robot_ip": ROBOT_IP,
                       "pose_a": a, "pose_b": b, "segment_len": L, "distances": distances,
                       "accel": args.accel, "v_start": args.v_start, "v_step": args.v_step,
                       "v_max": args.v_max, "speed_fraction": frac,
                       "envelope": envelope, "legs": all_legs, "interrupted": interrupted}, f, indent=2)
        print(f"\nsaved data -> {out_json}")
        if plot_on:
            redraw(plt, ax, series, distances, axis_max)
            out_png = f"speed_char_{stamp}.png"
            fig.savefig(out_png, dpi=120)
            print(f"saved plot -> {out_png}")
        # envelope recap - the CLI version of the money graph
        print("\ncommanded -> best achieved:")
        for vc, bp in envelope:
            print(f"  {vc:0.2f} m/s -> {bp:0.3f} m/s  (ratio {bp/vc:0.2f})  |{ascii_bar(bp, axis_max)}|")
        rtde_r.disconnect()
        if plot_on and not args.no_plot:
            print("\nclose the plot window to exit.")
            try:
                plt.ioff()
                plt.show()
            except Exception:
                pass


if __name__ == "__main__":
    main()
