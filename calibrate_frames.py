"""
Solves for T_base<-mocap: the rigid transform (rotation R, translation t)
mapping a point in Motive's world frame to the UR12e's base frame -
p_base = R @ p_mocap + t - via frames.umeyama_rigid_transform().

Procedure (see CLAUDE.md "Catch Integration" for the full rationale): visit
~20-40 poses spanning the reachable workspace with a single physical point -
the box-centroid TCP - and at each pose record where the robot's own forward
kinematics says that point is (p_robot, base frame, via rtde_receive) and
where Motive says the same point is (p_mocap, mocap frame, via NatNet). The
two point clouds are then fed to the SVD-based fit.

This only works if p_robot and p_mocap are the *same physical point* at every
pose - i.e. the robot's configured TCP must be the box's marker-rigid-body
centroid, not the flange. This script sends set_tcp() (over the raw
URScript-over-socket path, same as jog_ur_raw.py/ur_goto_raw.py) to do that,
using --tcp-offset (default: measured from the current 15.5(w) x 14.5(d) cm
box, flange mounted flush/centered on the 14.5cm-deep back face, so the
geometric center sits 7.25cm straight out along the flange's Z axis, X/Y=0 -
swapped in 2026-07-20 from an earlier 30x23x24cm box, see CLAUDE.md).
It is the operator's responsibility to have already set the Motive rigid
body's pivot to that same physical point (Rigid Body Properties -> Translate
Pivot) - this script only sanity-checks that set_tcp() visibly changed the
reported pose, not that the two points actually coincide; that's what the
live verification pass at the end is for.

--unlabeled-marker: track a raw unlabeled Motive marker instead of a rigid
body - for a calibration point mounted with too few markers (<3) to form a
Rigid Body asset (e.g. a single marker precisely placed at/near the flange
TCP, see CLAUDE.md "Catch Integration"). Two DIFFERENT NatNet fields can both
plausibly mean "unlabeled marker" and they are NOT confirmed to be the same
population of points:
  - --marker-source unlabeled-list (default): frame.unlabeled_marker_pos, a
    bare position list with no id/occlusion metadata - "valid" means exactly
    one entry this frame. This is the SAME field live_view.py's "Unlabeled
    Markers" table reads, so it's the one actually confirmed (by eye) to show
    your physical marker.
  - --marker-source labeled-markers: frame.labeled_markers (NatNet's per-marker
    stream, protocol >=2.3) filtered to entries with the .unlabeled flag set -
    has an id_num and an explicit occluded flag, which unlabeled-list lacks,
    but its scope is less certain: "labeled_markers" may only cover markers
    Motive has associated with some defined marker set/asset, in which case a
    persistently-mislabeled marker belonging to an unrelated existing asset
    could satisfy "exactly one unlabeled entry" every frame and get tracked
    instead of your real marker - consistently, not sporadically, which would
    look exactly like the marker-source, not an offset/mounting problem. Use
    --list-markers to compare both sources side by side before trusting either.
Neither source gives a rigid body's Motive-side identity persistence across a
whole session, so by default this just requires *exactly one* candidate
present each frame and rejects/treats-as-occluded any frame where that's not
true (zero, or more than one - e.g. a stray reflection); existing per-capture
rejection (stationary-speed/valid-ratio/std) then catches it like any other
bad capture. --marker-id (labeled-markers source only) selects a specific id
instead of requiring exactly one candidate - only safe if you've confirmed
that id is stable across your session via --list-markers.

--asset-marker: track a marker belonging to a Motive "Marker" asset (as
opposed to a "Rigid Body" asset, which needs >=3 markers) - this gets Motive's
asset-level position solving/refinement even for a single physical marker,
unlike either --unlabeled-marker source above which are raw/unrefined. Reads
frame.assets (NatNet's generic asset stream, protocol >=4.1), matching an
Asset by --asset-id (auto-selected if exactly one Asset is present) and
requiring it to expose exactly one AssetMarkerData in .markers.

Tool-offset joint solve (default on, --no-solve-offset to disable): every
sample now records the full 6D TCP pose, and the fit jointly estimates the
tracked point's fixed offset in the TOOL frame alongside the transform
(frames.fit_transform_with_tool_offset). Consequence: the marker/pivot does
NOT need to be precisely at the TCP anymore - anywhere rigid on the tool
works - and a wrong --tcp-offset or a stale controller-side set_tcp() can no
longer silently poison the fit (the exact failure of the 2026-07-17 marker
runs: a stale 12cm box TCP produced 80mm RMSE that looked like tracking
noise; see docs/debug_log.md 2026-07-18). The solved |d| is printed as a
mounting sanity check. set_tcp() is additionally verified with a wiggle test
at startup (apply_and_verify_tcp) so an ignored write aborts loudly instead.

Single-person workflow: this script never commands robot motion. Move the
arm by hand (freedrive) or with a jog script running separately, hold it
still, then press Enter here to sample - no need to be at the pendant and
the keyboard at the same time.
"""

import argparse
import json
import socket
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rtde_receive
from natnet import NatNetClient, DataFrame

from frames import (umeyama_rigid_transform, mocap_point_to_base,
                    fit_transform_with_tool_offset, rotvec_to_matrix)

ROBOT_IP = "192.168.20.1"
SECONDARY_PORT = 30002  # raw URScript-over-socket - see CLAUDE.md

# Measured 2026-07-20: 15.5(w) x 14.5(d) cm box, flange mounted flush and
# centered on the 14.5cm-deep back face -> geometric center sits at half the
# depth (7.25cm) straight out along the flange's local Z axis, X/Y=0 since
# centered. Must match wherever the Motive rigid body's pivot is set to.
# (Was 0.12 / a 30x23x24cm box through 2026-07-19 - see CLAUDE.md.)
TCP_OFFSET = (0.0, 0.0, 0.0725, 0.0, 0.0, 0.0)  # x,y,z (m), rx,ry,rz (rad)

DEFAULT_NUM_SAMPLES = 25
MIN_SAMPLES = 8            # hard floor - below this the SVD fit is poorly conditioned
RECOMMENDED_SAMPLES = 20   # CLAUDE.md's "20-40 spanning the workspace" guidance
SAMPLE_WINDOW = 0.6        # seconds averaged per captured pose
POLL_INTERVAL = 0.02       # s, ~50Hz robot-side polling during a capture window
STATIONARY_SPEED = 0.01    # m/s - peak TCP speed above this during a capture -> reject & retry
MIN_MOCAP_VALID_RATIO = 0.9   # fraction of frames during the window that must be tracking_valid
MOCAP_STD_LIMIT = 0.005    # m - std-dev of captured mocap positions above this -> reject & retry
OUTLIER_FACTOR = 3.0       # flag a sample as a possible outlier above this multiple of the RMSE
OUTLIER_ABS = 0.02         # ...or above this absolute residual (m), whichever is larger

STATE_LOCK = threading.Lock()

INSTRUCTIONS = """
Frame calibration - Motive (mocap) <-> UR12e base frame
--------------------------------------------------------
For each sample: move the arm (freedrive, or a jog script in another
terminal) so the box sits at a new pose spanning the workspace, hold it
still, then press Enter here. This script only reads state - it never
drives the robot - so one person can do the whole thing alone.

Type 'q' instead of Enter to stop early (once >= {min_samples} samples are
in). Bad captures (robot moved, rigid body occluded/unstable) are rejected
automatically with a reason - just retry the same pose.
""".format(min_samples=MIN_SAMPLES)


def send_urscript(script, timeout=5.0):
    with socket.create_connection((ROBOT_IP, SECONDARY_PORT), timeout=timeout) as s:
        s.sendall(script.encode("utf-8"))


def set_tcp_script(offset):
    off = "p[" + ",".join(f"{v:.6f}" for v in offset) + "]"
    return f"""def prog():
  set_tcp({off})
end
prog()
"""


class MocapState:
    def __init__(self):
        self.target_id = None
        self.candidate_ids = []
        self.latest_pos = None
        self.latest_valid = False
        self.capturing = False
        self.capture_samples = []  # (x,y,z) if valid, None if not - filled while capturing=True


def make_handler(state: MocapState, forced_id):
    def handle_frame(frame: DataFrame) -> None:
        with STATE_LOCK:
            if state.target_id is None:
                ids = [rb.id_num for rb in frame.rigid_bodies]
                if forced_id is not None:
                    state.target_id = forced_id
                elif len(ids) == 1:
                    state.target_id = ids[0]
                else:
                    state.candidate_ids = ids
                    return

            rb = next((r for r in frame.rigid_bodies if r.id_num == state.target_id), None)
            if rb is None:
                return

            valid = True if rb.tracking_valid is None else rb.tracking_valid
            state.latest_pos = rb.pos
            state.latest_valid = valid
            if state.capturing:
                state.capture_samples.append(rb.pos if valid else None)

    return handle_frame


def wait_for_rigid_body(state: MocapState, timeout=10.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        with STATE_LOCK:
            target_id = state.target_id
            candidates = list(state.candidate_ids)
        if target_id is not None:
            return target_id
        if candidates:
            raise SystemExit(
                f"Multiple rigid bodies seen: {candidates}. Re-run with --rigid-body-id <id>."
            )
        time.sleep(0.1)
    raise SystemExit("No rigid body seen within timeout - check Motive streaming/tracking.")


def make_marker_handler(state: MocapState, forced_marker_id, source):
    """Like make_handler() but for a raw unlabeled marker instead of a rigid
    body - see module docstring for the two possible NatNet sources (--marker-
    source) and why neither locks onto an id up front the way rigid-body mode
    does."""
    def handle_frame(frame: DataFrame) -> None:
        with STATE_LOCK:
            if source == "unlabeled-list":
                # frame.unlabeled_marker_pos: bare positions, no id/occlusion -
                # the field live_view.py's "Unlabeled Markers" table reads.
                positions = frame.unlabeled_marker_pos
                state.candidate_ids = list(range(len(positions)))
                pos = positions[0] if len(positions) == 1 else None
                valid = pos is not None
            else:
                candidates = [m for m in frame.labeled_markers if m.unlabeled]
                state.candidate_ids = [m.id_num for m in candidates]
                if forced_marker_id is not None:
                    m = next((mk for mk in candidates if mk.id_num == forced_marker_id), None)
                elif len(candidates) == 1:
                    m = candidates[0]
                else:
                    m = None  # zero or >1 unlabeled markers this instant - ambiguous/missing
                pos = m.pos if (m is not None and not m.occluded) else None
                valid = pos is not None

            if pos is not None:
                state.latest_pos = pos
            state.latest_valid = valid
            if state.capturing:
                state.capture_samples.append(pos if valid else None)

    return handle_frame


def wait_for_marker(state: MocapState, forced_marker_id, timeout=10.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        with STATE_LOCK:
            valid = state.latest_valid
            candidates = list(state.candidate_ids)
        if valid:
            return
        if forced_marker_id is None and len(candidates) > 1:
            raise SystemExit(
                f"Multiple unlabeled markers seen: {candidates}. Re-run with --marker-id <id> "
                "(labeled-markers source only - verify it's stable first with --list-markers), "
                "or remove the extra reflective points from the volume."
            )
        time.sleep(0.1)
    raise SystemExit(
        "No (single) unlabeled marker seen within timeout - check Motive streaming and that "
        "exactly one unlabeled marker is visible (or pass --marker-id)."
    )


def make_asset_marker_handler(state: MocapState, forced_asset_id):
    """Tracks a single AssetMarkerData from frame.assets - a Motive "Marker"
    asset (not "Rigid Body", no 3-marker minimum) gets asset-level position
    solving unlike the raw --unlabeled-marker sources. AssetMarkerData has no
    occluded-flag helper property in this natnet library version (unlike
    LabeledMarker) - marker_params uses the same NatNet bitfield convention,
    bit 0 = occluded, so it's masked directly here."""
    def handle_frame(frame: DataFrame) -> None:
        with STATE_LOCK:
            assets = frame.assets
            state.candidate_ids = [a.asset_id for a in assets]
            if forced_asset_id is not None:
                asset = next((a for a in assets if a.asset_id == forced_asset_id), None)
            elif len(assets) == 1:
                asset = assets[0]
            else:
                asset = None  # zero or >1 assets - ambiguous/missing, pass --asset-id

            pos = None
            if asset is not None and len(asset.markers) == 1:
                marker = asset.markers[0]
                occluded = bool(marker.marker_params & 0x01)
                if not occluded:
                    pos = marker.pos

            valid = pos is not None
            if pos is not None:
                state.latest_pos = pos
            state.latest_valid = valid
            if state.capturing:
                state.capture_samples.append(pos if valid else None)

    return handle_frame


def wait_for_asset(state: MocapState, forced_asset_id, timeout=10.0):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        with STATE_LOCK:
            valid = state.latest_valid
            candidates = list(state.candidate_ids)
        if valid:
            return
        if forced_asset_id is None and len(candidates) > 1:
            raise SystemExit(
                f"Multiple assets seen: {candidates}. Re-run with --asset-id <id>."
            )
        time.sleep(0.1)
    raise SystemExit(
        "No single-marker asset seen within timeout - check Motive streaming and that the "
        "asset really has exactly one marker (or pass --asset-id)."
    )


def list_markers(server_ip, local_ip, unicast, duration=5.0):
    """Standalone sanity check before trusting --unlabeled-marker or --asset-marker:
    prints frame.unlabeled_marker_pos, frame.labeled_markers (.unlabeled), and
    frame.assets side by side for a few seconds, so you can see directly whether
    the two --unlabeled-marker sources agree (same count/position) and confirm
    an id looks stable, instead of assuming they refer to the same physical point."""
    def handle(frame: DataFrame) -> None:
        ul = frame.unlabeled_marker_pos
        lm = [m for m in frame.labeled_markers if m.unlabeled]
        ul_str = ", ".join(f"({p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f})" for p in ul) or "(none)"
        lm_str = ", ".join(f"id={m.id_num} pos=({m.pos[0]:+.4f},{m.pos[1]:+.4f},{m.pos[2]:+.4f}) "
                            f"occluded={m.occluded}" for m in lm) or "(none)"
        print(f"[{frame.prefix.frame_number}] unlabeled_marker_pos: {ul_str}")
        print(f"    labeled_markers(.unlabeled): {lm_str}")
        for a in frame.assets:
            markers_str = ", ".join(f"marker_id={m.marker_id} pos=({m.pos[0]:+.4f},{m.pos[1]:+.4f},"
                                     f"{m.pos[2]:+.4f}) occluded={bool(m.marker_params & 0x01)}"
                                     for m in a.markers) or "(no markers)"
            print(f"    asset id={a.asset_id}: {markers_str}")

    client = NatNetClient(server_ip_address=server_ip, local_ip_address=local_ip,
                           use_multicast=not unicast)
    client.on_data_frame_received_event.handlers.append(handle)
    with client:
        client.run_async()
        time.sleep(duration)
        client.stop_async()


def apply_and_verify_tcp(rtde_r, tcp_offset):
    """set_tcp() the intended offset and PROVE the controller applied it, via a
    wiggle test: temporarily set the offset +5cm along tool Z, confirm the
    reported TCP pose moved by exactly that much, then restore. Exists because
    of the 2026-07-17 marker-calibration failure (docs/debug_log.md
    2026-07-18): the old before/after print could not distinguish "offset was
    already active" from "set_tcp writes are being ignored" (e.g. Remote
    Control off), and an ignored write left a stale 12cm box TCP silently
    poisoning every sample by ~10cm. No motion involved - set_tcp only changes
    how the controller REPORTS the (stationary) pose."""
    WIGGLE = 0.05
    intended = list(tcp_offset)
    wiggled = list(tcp_offset)
    wiggled[2] += WIGGLE

    send_urscript(set_tcp_script(intended))
    time.sleep(0.3)
    base_pose = np.array(rtde_r.getActualTCPPose()[:3])
    send_urscript(set_tcp_script(wiggled))
    time.sleep(0.3)
    wiggle_pose = np.array(rtde_r.getActualTCPPose()[:3])
    send_urscript(set_tcp_script(intended))
    time.sleep(0.3)
    restored_pose = np.array(rtde_r.getActualTCPPose()[:3])

    moved = float(np.linalg.norm(wiggle_pose - base_pose))
    restored = float(np.linalg.norm(restored_pose - base_pose))
    print(f"set_tcp({intended}) sent; wiggle test: +{WIGGLE * 1000:.0f}mm tool-Z probe moved the "
          f"reported TCP by {moved * 1000:.1f}mm (expect ~{WIGGLE * 1000:.0f}), restore delta "
          f"{restored * 1000:.2f}mm (expect ~0).")
    if not (abs(moved - WIGGLE) < 0.01 and restored < 0.002):
        raise SystemExit(
            "set_tcp() writes are NOT taking effect on the controller (reported pose did not track "
            "the wiggle). Calibrating now would record p_robot for whatever stale TCP is active - "
            "the exact failure that poisoned the 2026-07-17 marker calibration. Check Remote "
            "Control is ON and the pendant's Installation -> TCP, then retry."
        )


def capture_robot_window(rtde_r, duration, poll_interval):
    """Poll the robot for `duration` seconds. Also detects a dead/stale RTDE
    connection (e.g. after a protective stop takes down the controller and
    reboots it): `getActualTCPPose()` does NOT raise or block when the
    connection has dropped - it just keeps returning the last cached packet
    forever, silently. That exact failure poisoned a real 2026-07-20
    calibration run (samples read a frozen p_robot while p_mocap kept
    changing, wrecking the RMSE from ~5mm to 350mm) - see docs/debug_log.md.
    Two independent checks, either one being bad marks the window stale:
    isConnected() (the library's own liveness flag) and the RTDE timestamp
    actually advancing over the window (catches a connection that reports
    connected but has stopped receiving fresh packets)."""
    positions = []
    poses = []
    max_speed = 0.0
    start_ts = rtde_r.getTimestamp()
    end = time.monotonic() + duration
    while time.monotonic() < end:
        pose = rtde_r.getActualTCPPose()
        speed = rtde_r.getActualTCPSpeed()
        lin_speed = (speed[0] ** 2 + speed[1] ** 2 + speed[2] ** 2) ** 0.5
        positions.append(pose[:3])
        poses.append(pose)
        max_speed = max(max_speed, lin_speed)
        time.sleep(poll_interval)
    end_ts = rtde_r.getTimestamp()
    stale = (not rtde_r.isConnected()) or (end_ts <= start_ts)
    # Full 6D pose for the tool-offset joint solve (fit_transform_with_tool_offset):
    # position is averaged as before, but the rotation vector is taken from the
    # middle sample rather than averaged - rotvecs near |rv|=pi can flip sign
    # between samples, which would average to garbage; the arm is stationary
    # (enforced by the caller's speed check) so any single sample is fine.
    mid_pose = poses[len(poses) // 2]
    return np.array(positions), max_speed, np.array(mid_pose), stale


def capture_mocap_window(state: MocapState, duration):
    with STATE_LOCK:
        state.capture_samples = []
        state.capturing = True
    time.sleep(duration)
    with STATE_LOCK:
        state.capturing = False
        raw = list(state.capture_samples)
    total = len(raw)
    valid = [s for s in raw if s is not None]
    ratio = (len(valid) / total) if total else 0.0
    positions = np.array(valid) if valid else None
    return positions, ratio


def capture_pair(rtde_r, state, duration, poll_interval):
    mocap_result = {}

    def _mocap():
        positions, ratio = capture_mocap_window(state, duration)
        mocap_result["positions"] = positions
        mocap_result["ratio"] = ratio

    thread = threading.Thread(target=_mocap)
    thread.start()
    robot_positions, max_speed, tcp_pose, stale = capture_robot_window(rtde_r, duration, poll_interval)
    thread.join()

    return robot_positions, max_speed, tcp_pose, stale, mocap_result["positions"], mocap_result["ratio"]


def prompt_and_sample(rtde_r, state, index, target_n, collected_so_far):
    while True:
        print(f"\n[{index}/{target_n}] Move to the next pose and hold still, then press Enter "
              f"to sample (or 'q' to stop - have {collected_so_far}, need >= {MIN_SAMPLES}).")
        line = input("> ").strip().lower()
        if line == "q":
            return "stop"

        robot_positions, max_speed, tcp_pose, stale, mocap_positions, mocap_ratio = capture_pair(
            rtde_r, state, SAMPLE_WINDOW, POLL_INTERVAL
        )

        if stale:
            print("  REJECTED: the RTDE connection to the robot looks dead (not connected, or no "
                  "fresh packets arrived during the capture window) - readings would be a frozen "
                  "cached pose, not the arm's real position. This happens after a protective stop "
                  "reboots the controller. DO NOT keep sampling: Ctrl-C out of this script entirely "
                  "and rerun it fresh (add --resume <your --out file> to continue from the samples "
                  "you already have instead of starting over).")
            continue

        if max_speed > STATIONARY_SPEED:
            print(f"  REJECTED: robot was moving during capture (peak speed "
                  f"{max_speed * 1000:.1f} mm/s > {STATIONARY_SPEED * 1000:.0f} mm/s threshold). "
                  f"Let go of freedrive / stop jogging fully, then retry.")
            continue

        if mocap_positions is None or mocap_ratio < MIN_MOCAP_VALID_RATIO:
            got = f"{mocap_ratio * 100:.0f}%" if mocap_positions is not None else "no frames received"
            print(f"  REJECTED: rigid body/marker tracking was invalid too often during capture "
                  f"({got} valid, need >= {MIN_MOCAP_VALID_RATIO * 100:.0f}%). Check for "
                  f"occlusion, a stray second unlabeled marker, or a lost NatNet connection, "
                  f"then retry.")
            continue

        mocap_std = mocap_positions.std(axis=0)
        if np.any(mocap_std > MOCAP_STD_LIMIT):
            print(f"  REJECTED: mocap position was unstable during capture (std="
                  f"{[round(v * 1000, 2) for v in mocap_std]} mm, limit "
                  f"{MOCAP_STD_LIMIT * 1000:.0f} mm) - possible partial occlusion or box wobble. Retry.")
            continue

        p_robot = robot_positions.mean(axis=0)
        p_mocap = mocap_positions.mean(axis=0)
        print(f"  OK  p_robot=({p_robot[0]:+.4f},{p_robot[1]:+.4f},{p_robot[2]:+.4f})  "
              f"p_mocap=({p_mocap[0]:+.4f},{p_mocap[1]:+.4f},{p_mocap[2]:+.4f})  "
              f"mocap_valid={mocap_ratio * 100:.0f}%")
        return p_robot, p_mocap, tcp_pose


def load_resume_samples(path, expected_rigid_body_id, expected_tcp_offset, expected_tracking_mode):
    """Load (p_robot, p_mocap) pairs from a previous run's --out JSON, so a
    session can continue past sample N instead of starting over. Refuses to
    mix samples collected against a different rigid body or a different TCP
    offset - either would make old and new p_robot values incomparable."""
    try:
        data = json.loads(Path(path).read_text())
    except FileNotFoundError:
        raise SystemExit(f"--resume file not found: {path}")

    saved_mode = data.get("tracking_mode", "rigid_body")
    if saved_mode != expected_tracking_mode:
        raise SystemExit(
            f"Refusing to resume: {path} was collected in '{saved_mode}' mode, but this run is "
            f"'{expected_tracking_mode}'. p_mocap values come from different tracking paths - "
            f"match the mode (pass/drop --unlabeled-marker to match), or start fresh."
        )

    saved_id = data.get("rigid_body_id")
    if saved_mode == "rigid_body" and saved_id is not None and saved_id != expected_rigid_body_id:
        raise SystemExit(
            f"Refusing to resume: {path} was collected against rigid body id={saved_id}, "
            f"but this run is tracking id={expected_rigid_body_id}. Re-run with "
            f"--rigid-body-id {saved_id} to match, or start fresh without --resume."
        )
    elif saved_mode != "rigid_body" and saved_id is not None and saved_id != expected_rigid_body_id:
        print(f"NOTE: {path} was first collected with id={saved_id} ('{saved_mode}' mode), this "
              f"run saw id={expected_rigid_body_id} - an unlabeled marker/asset id isn't "
              f"guaranteed stable across sessions (see module docstring), so this is not treated "
              f"as an error, but double check it's really the same physical marker before trusting "
              f"the combined fit.")

    saved_offset = data.get("tcp_offset")
    if saved_offset is not None and any(
        abs(a - b) > 1e-6 for a, b in zip(saved_offset, expected_tcp_offset)
    ):
        raise SystemExit(
            f"Refusing to resume: {path} was collected with --tcp-offset {saved_offset}, "
            f"but this run is using {expected_tcp_offset}. p_robot values aren't comparable "
            f"across different TCP offsets - pass --tcp-offset {saved_offset} to match, or "
            f"start fresh without --resume."
        )

    pairs = [(np.array(s["p_robot"]), np.array(s["p_mocap"]),
              np.array(s["tcp_pose"]) if s.get("tcp_pose") is not None else None)
             for s in data["samples"]]
    if any(p[2] is None for p in pairs):
        print(f"NOTE: {path} predates full-TCP-pose recording - the tool-offset joint solve "
              f"(see fit_and_report) will only use the samples that have one.")
    return pairs


def running_fit_estimate(pairs, solve_offset=True):
    """Cheap RMSE from whatever samples have been collected SO FAR, printed
    after every capture during collect_samples() so a badly-cooked run is
    visible at sample 8 instead of only after committing to the full 20-40
    (see fit_and_report for the final, fully-reported fit - this is a rough
    preview using the same math, minus the per-sample residual printout).

    IMPORTANT asymmetry: with few points a rigid transform is close to
    exactly-fit (3 points is the hard minimum and fits ~perfectly even on bad
    data), so RMSE here is systematically OPTIMISTIC at low N and only
    approaches the true noise floor as samples accumulate - same overfitting
    shape as trajectory.py's residual_rms (see CLAUDE.md). Consequence: a BAD
    number this early is trustworthy (something's really wrong), a GOOD
    number is not proof of anything yet - don't stop early just because this
    looks clean.
    """
    p_mocap = np.array([p[1] for p in pairs])
    p_robot = np.array([p[0] for p in pairs])
    R, t = umeyama_rigid_transform(p_mocap, p_robot)
    predicted = mocap_point_to_base(p_mocap, R, t)
    rmse = float(np.sqrt(np.mean(np.sum((predicted - p_robot) ** 2, axis=1))))

    rmse_joint = None
    if solve_offset:
        posed = [p for p in pairs if p[2] is not None]
        if len(posed) >= MIN_SAMPLES:
            pm = np.array([p[1] for p in posed])
            tcp_poses = np.array([p[2] for p in posed])
            _, _, _, rmse_joint = fit_transform_with_tool_offset(pm, tcp_poses)
    return rmse, rmse_joint


def collect_samples(rtde_r, state, target_n, existing_pairs=None, solve_offset=True):
    pairs = list(existing_pairs) if existing_pairs else []
    if pairs:
        print(f"Resuming with {len(pairs)} existing sample(s) - continuing from "
              f"sample {len(pairs) + 1}.")
    if len(pairs) >= target_n:
        print(f"Already have {len(pairs)} samples, >= target of {target_n} - skipping "
              f"new collection (pass --num-samples to collect more).")
        return pairs

    index = len(pairs) + 1
    while True:
        result = prompt_and_sample(rtde_r, state, index, target_n, len(pairs))
        if result == "stop":
            if len(pairs) < MIN_SAMPLES:
                print(f"Need at least {MIN_SAMPLES} samples, have {len(pairs)} - keep going.")
                continue
            break
        pairs.append(result)
        index += 1
        if len(pairs) >= 4:
            rmse, rmse_joint = running_fit_estimate(pairs, solve_offset=solve_offset)
            joint_str = f", tool-offset RMSE = {rmse_joint * 1000:.1f} mm" if rmse_joint is not None else ""
            hint = " (few samples - optimistic, not a verdict)" if len(pairs) < MIN_SAMPLES else ""
            print(f"  running fit check @ {len(pairs)} samples: plain RMSE = {rmse * 1000:.1f} mm"
                  f"{joint_str}{hint}")
        if len(pairs) >= target_n:
            break
    if len(pairs) < RECOMMENDED_SAMPLES:
        print(f"\nNOTE: only {len(pairs)} samples collected - CLAUDE.md recommends 20-40 spanning "
              f"the workspace for a reliable fit. Consider collecting more next time.")
    return pairs


def fit_and_report(pairs, solve_offset=True):
    """Plain Umeyama fit, plus (when full TCP poses were recorded and
    solve_offset is on) the tool-offset joint solve - which additionally
    estimates the marker/pivot's fixed offset `d` in the TOOL frame, making the
    result immune to a wrong --tcp-offset or a stale controller-side set_tcp().
    That exact failure poisoned the 2026-07-17 single-marker calibrations with
    a hidden ~10cm offset (80mm RMSE, undetectable from residuals alone) - see
    docs/debug_log.md 2026-07-18.

    Returns (R, t, rmse, errors, fit_info) - fit_info describes which fit the
    returned R/t came from plus both RMSEs and the solved offset, for the
    output JSON."""
    p_mocap = np.array([p[1] for p in pairs])
    p_robot = np.array([p[0] for p in pairs])

    R, t = umeyama_rigid_transform(p_mocap, p_robot)
    predicted = mocap_point_to_base(p_mocap, R, t)
    errors = np.linalg.norm(predicted - p_robot, axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))
    fit_info = {"fit_mode": "rigid", "rigid_rmse_m": rmse}

    print(f"\nPlain rigid fit: {len(pairs)} samples, RMSE = {rmse * 1000:.2f} mm")
    outlier_limit = max(OUTLIER_FACTOR * rmse, OUTLIER_ABS)
    for i, e in enumerate(errors):
        flag = "  <-- possible outlier, consider re-collecting this pose" if e > outlier_limit else ""
        print(f"  sample {i + 1:2d}: residual = {e * 1000:6.2f} mm{flag}")

    posed = [p for p in pairs if p[2] is not None]
    if solve_offset and len(posed) >= MIN_SAMPLES:
        pm = np.array([p[1] for p in posed])
        tcp_poses = np.array([p[2] for p in posed])
        R_j, t_j, d, rmse_j = fit_transform_with_tool_offset(pm, tcp_poses)
        print(f"\nTool-offset joint solve ({len(posed)} samples with full TCP pose):")
        print(f"  RMSE = {rmse_j * 1000:.2f} mm (vs {rmse * 1000:.2f} mm plain)")
        print(f"  solved marker/pivot offset in TOOL frame d = "
              f"({d[0] * 1000:+.1f}, {d[1] * 1000:+.1f}, {d[2] * 1000:+.1f}) mm, |d| = "
              f"{np.linalg.norm(d) * 1000:.1f} mm")
        print("  Sanity-check |d| against where the marker/pivot physically sits relative to the")
        print("  ACTIVE TCP: near zero if your --tcp-offset was right, and a large |d| means the")
        print("  configured TCP did NOT match the marker (e.g. stale set_tcp) - the joint fit has")
        print("  absorbed that error, but do figure out where it came from.")
        if np.linalg.norm(d) > 0.03:
            print(f"  WARNING: |d| = {np.linalg.norm(d) * 1000:.0f} mm - the tracked point was far "
                  f"from the configured TCP. The joint fit corrects for it, but a plain fit of this "
                  f"same data would have been badly poisoned.")
        # The joint fit's R/t is the genuine mocap->base rigid transform (d is solved
        # out separately, not baked into R/t) - exactly what catch.py needs, since it
        # applies R/t to the BALL's mocap position, which has no tool-frame offset to
        # correct for. Applied to the MARKER's own mocap position instead, R/t predicts
        # the marker's base-frame position (p_tcp + R_tool@d), NOT the TCP position
        # itself - see verify_live()'s docstring for why that distinction matters.
        R, t = R_j, t_j
        # Per-sample residuals of the joint model against the recorded TCP positions.
        # Kept aligned with `pairs`: a resumed old-format sample without a stored TCP
        # pose can't have the marker offset applied, so its residual is computed
        # against p_robot directly and still CONTAINS the marker-offset error - fine
        # for display, and the joint fit itself never used those samples.
        marker_pred_posed = mocap_point_to_base(pm, R, t)
        tool_R = np.stack([rotvec_to_matrix(p[3:6]) for p in tcp_poses])
        joint_err = np.linalg.norm(
            marker_pred_posed - (tcp_poses[:, :3] + np.einsum("nij,j->ni", tool_R, d)), axis=1)
        errors = []
        ji = 0
        for p in pairs:
            if p[2] is not None:
                errors.append(float(joint_err[ji]))
                ji += 1
            else:
                errors.append(float(np.linalg.norm(mocap_point_to_base(p[1], R, t) - p[0])))
        errors = np.array(errors)
        rmse = rmse_j
        fit_info = {"fit_mode": "tool_offset", "rigid_rmse_m": fit_info["rigid_rmse_m"],
                    "tool_offset_rmse_m": rmse_j, "marker_offset_tool_m": d.tolist(),
                    "n_pose_samples": len(posed)}
    elif solve_offset:
        print(f"\n(tool-offset joint solve skipped: only {len(posed)} samples carry a full TCP "
              f"pose, need >= {MIN_SAMPLES})")

    if rmse > 0.015:
        print("\nWARNING: RMSE is high relative to the <1.5mm published mocap-robot benchmark "
              "(CLAUDE.md). With the tool-offset solve active this is no longer explainable by a "
              "TCP/pivot mismatch - remaining suspects: mocap volume calibration quality (re-wand), "
              "marker centroid bias, or the arm moving during captures.")

    return R, t, rmse, errors, fit_info


def verify_live(rtde_r, state, R, t, duration, d=None):
    """`R, t` (from the tool-offset joint solve, when active) predict the
    MARKER's base-frame position (p_tcp + R_tool @ d), not the TCP itself -
    see fit_transform_with_tool_offset(). Comparing that straight against
    getActualTCPPose() is only valid when d~=0 (plain fit); otherwise the two
    points are legitimately |d| apart by construction and the "error" printed
    is just |d|, not a calibration problem (this is what produced a
    confusing ~92mm live "error" on 2026-07-20 despite a 4.25mm fit RMSE -
    see docs/debug_log.md). Adding R_tool_current @ d onto `actual` here
    makes the comparison apples-to-apples again; with d=None/zero this is a
    no-op and matches the old behavior."""
    print("\nVerification: move the arm around (freedrive/jog) and watch predicted vs actual "
          "agree - this is the real check that the transform (and the TCP/pivot alignment behind "
          "it) is correct, not just that the fit converged. Ctrl-C to stop.")
    d = np.zeros(3) if d is None else np.asarray(d, dtype=float)
    start = time.monotonic()
    try:
        while duration is None or time.monotonic() - start < duration:
            actual_pose = rtde_r.getActualTCPPose()
            actual = np.array(actual_pose[:3])
            if np.any(d):
                actual = actual + rotvec_to_matrix(actual_pose[3:6]) @ d
            with STATE_LOCK:
                mocap_pos, valid = state.latest_pos, state.latest_valid
            if mocap_pos is not None and valid:
                predicted = mocap_point_to_base(np.array(mocap_pos), R, t)
                err = np.linalg.norm(predicted - actual)
                print(f"\rpredicted=({predicted[0]:+.4f},{predicted[1]:+.4f},{predicted[2]:+.4f})  "
                      f"actual=({actual[0]:+.4f},{actual[1]:+.4f},{actual[2]:+.4f})  "
                      f"error={err * 1000:6.1f} mm   ", end="", flush=True)
            else:
                print("\rwaiting for valid tracking...                                             ",
                      end="", flush=True)
            # 2Hz, not 10Hz: each \r-updated print still lands as a separate entry in
            # the terminal's scrollback (only the on-screen line is overwritten), so
            # 10Hz flooded scrollback history with thousands of near-duplicate lines
            # during a normal Ctrl-C-terminated verify run (2026-07-20 user report).
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    print()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--rigid-body-id", type=int, default=None,
                         help="NatNet rigid body id to track (auto-selected if only one is visible). "
                              "Mutually exclusive with --unlabeled-marker/--asset-marker.")
    parser.add_argument("--unlabeled-marker", action="store_true",
                         help="Track a raw unlabeled marker instead of a rigid body - for a "
                              "calibration point with too few markers (<3) to form a Rigid Body "
                              "asset in Motive. See module docstring for the two possible sources "
                              "(--marker-source) and the identity caveat.")
    parser.add_argument("--marker-source", choices=("unlabeled-list", "labeled-markers"),
                         default="unlabeled-list",
                         help="With --unlabeled-marker: which NatNet field to read. "
                              "unlabeled-list (default) = frame.unlabeled_marker_pos, the same field "
                              "live_view.py's 'Unlabeled Markers' table shows - confirmed to match "
                              "what you actually see. labeled-markers = frame.labeled_markers "
                              "filtered to .unlabeled, which has id/occlusion metadata but an "
                              "uncertain scope - see module docstring. Use --list-markers to compare.")
    parser.add_argument("--marker-id", type=int, default=None,
                         help="With --unlabeled-marker --marker-source labeled-markers: select a "
                              "specific marker id instead of requiring exactly one unlabeled marker "
                              "in the volume. Only safe if you've confirmed this id is stable across "
                              "your session - check with --list-markers first. Not usable with the "
                              "unlabeled-list source (no ids exist there).")
    parser.add_argument("--asset-marker", action="store_true",
                         help="Track a Motive 'Marker' asset (not 'Rigid Body', no 3-marker "
                              "minimum) via frame.assets - gets asset-level position refinement "
                              "unlike --unlabeled-marker's raw sources. Mutually exclusive with "
                              "--rigid-body-id/--unlabeled-marker.")
    parser.add_argument("--asset-id", type=int, default=None,
                         help="With --asset-marker: which Asset id to track (auto-selected if only "
                              "one is present)")
    parser.add_argument("--list-markers", action="store_true",
                         help="Print frame.unlabeled_marker_pos, frame.labeled_markers(.unlabeled), "
                              "and frame.assets side by side for a few seconds and exit - a sanity "
                              "check before trusting --unlabeled-marker or --asset-marker (confirms "
                              "there's really only one candidate, that the sources agree, or that an "
                              "id you plan to pin looks stable).")
    parser.add_argument("--num-samples", type=int, default=DEFAULT_NUM_SAMPLES,
                         help=f"Target number of calibration poses (default {DEFAULT_NUM_SAMPLES})")
    parser.add_argument("--tcp-offset", type=float, nargs=6, default=list(TCP_OFFSET),
                         metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
                         help="Box-centroid TCP offset from the flange, meters/radians "
                              f"(default {list(TCP_OFFSET)}, measured from the 15.5x14.5cm box)")
    parser.add_argument("--no-solve-offset", action="store_true",
                         help="Disable the tool-offset joint solve and save the plain rigid fit. "
                              "By default the fit jointly estimates the marker/pivot's fixed offset "
                              "in the tool frame from the recorded full TCP poses, which makes the "
                              "transform immune to a wrong/stale TCP offset (the failure that "
                              "poisoned the 2026-07-17 marker calibrations - see debug_log.md).")
    parser.add_argument("--skip-set-tcp", action="store_true",
                         help="Don't send set_tcp() - use if the box-centroid TCP is already "
                              "active (e.g. configured on the pendant instead)")
    parser.add_argument("--out", default="UR10_T_base_from_mocap.json", help="Output JSON path")
    parser.add_argument("--resume", default=None,
                         help="Continue from a previous run's output JSON (e.g. --out's default "
                              "path) instead of starting over - same rigid body and TCP offset "
                              "required, and it's fine to pass the same path as --out")
    parser.add_argument("--skip-verify", action="store_true", help="Skip the live verification pass at the end")
    parser.add_argument("--verify-duration", type=float, default=None,
                         help="Seconds to run the verification pass (default: until Ctrl-C)")
    args = parser.parse_args()

    if sum([args.unlabeled_marker, args.asset_marker, args.rigid_body_id is not None]) > 1:
        raise SystemExit("--rigid-body-id, --unlabeled-marker, and --asset-marker are mutually exclusive.")
    if args.marker_id is not None and not args.unlabeled_marker:
        raise SystemExit("--marker-id only applies with --unlabeled-marker.")
    if args.marker_id is not None and args.marker_source == "unlabeled-list":
        raise SystemExit("--marker-id needs --marker-source labeled-markers (unlabeled-list has no ids).")
    if args.asset_id is not None and not args.asset_marker:
        raise SystemExit("--asset-id only applies with --asset-marker.")

    if args.list_markers:
        list_markers(args.server_ip, args.local_ip, args.unicast)
        return

    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

    if not args.skip_set_tcp:
        apply_and_verify_tcp(rtde_r, args.tcp_offset)
    else:
        print("Skipping set_tcp() - assuming the box-centroid TCP is already active.")

    state = MocapState()
    client = NatNetClient(
        server_ip_address=args.server_ip,
        local_ip_address=args.local_ip,
        use_multicast=not args.unicast,
    )
    if args.unlabeled_marker:
        client.on_data_frame_received_event.handlers.append(
            make_marker_handler(state, args.marker_id, args.marker_source))
        tracking_mode = f"unlabeled_marker_{args.marker_source.replace('-', '_')}"
    elif args.asset_marker:
        client.on_data_frame_received_event.handlers.append(make_asset_marker_handler(state, args.asset_id))
        tracking_mode = "asset_marker"
    else:
        client.on_data_frame_received_event.handlers.append(make_handler(state, args.rigid_body_id))
        tracking_mode = "rigid_body"

    with client:
        client.run_async()
        try:
            if args.unlabeled_marker:
                print("\nWaiting for a single unlabeled marker..."
                      if args.marker_id is None else f"\nWaiting for unlabeled marker id={args.marker_id}...")
                wait_for_marker(state, args.marker_id)
                with STATE_LOCK:
                    seen_ids = list(state.candidate_ids)
                target_id = args.marker_id if args.marker_id is not None else (seen_ids[0] if seen_ids else None)
                id_label = "index" if args.marker_source == "unlabeled-list" else "id"
                print(f"Tracking unlabeled marker (source={args.marker_source}) {id_label}={target_id} "
                      "(identity re-checked every frame, not locked - see module docstring)")
            elif args.asset_marker:
                print("\nWaiting for a single-marker Asset..."
                      if args.asset_id is None else f"\nWaiting for asset id={args.asset_id}...")
                wait_for_asset(state, args.asset_id)
                with STATE_LOCK:
                    seen_ids = list(state.candidate_ids)
                target_id = args.asset_id if args.asset_id is not None else (seen_ids[0] if seen_ids else None)
                print(f"Tracking asset id={target_id}")
            else:
                print("\nWaiting for Motive rigid body...")
                target_id = wait_for_rigid_body(state)
                print(f"Tracking rigid body id={target_id}")

            existing_pairs = []
            if args.resume:
                existing_pairs = load_resume_samples(args.resume, target_id, args.tcp_offset, tracking_mode)

            print(INSTRUCTIONS)
            pairs = collect_samples(rtde_r, state, args.num_samples, existing_pairs=existing_pairs,
                                     solve_offset=not args.no_solve_offset)
            R, t, rmse, errors, fit_info = fit_and_report(pairs, solve_offset=not args.no_solve_offset)

            out = {
                "R": R.tolist(),
                "t": t.tolist(),
                "rmse_m": rmse,
                "n_samples": len(pairs),
                "tcp_offset": list(args.tcp_offset),
                "rigid_body_id": target_id,
                "tracking_mode": tracking_mode,
                "created_utc": datetime.now(timezone.utc).isoformat(),
                **fit_info,
                "samples": [
                    {"p_robot": p_robot.tolist(), "p_mocap": p_mocap.tolist(),
                     "tcp_pose": (tcp_pose.tolist() if tcp_pose is not None else None),
                     "residual_m": float(err)}
                    for (p_robot, p_mocap, tcp_pose), err in zip(pairs, errors)
                ],
            }
            Path(args.out).write_text(json.dumps(out, indent=2))
            print(f"\nSaved transform to {args.out}")

            if not args.skip_verify:
                verify_live(rtde_r, state, R, t, args.verify_duration,
                            d=fit_info.get("marker_offset_tool_m"))
        finally:
            client.stop_async()

    rtde_r.disconnect()


if __name__ == "__main__":
    main()
