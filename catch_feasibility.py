"""
Console-only "would the arm have made this catch?" reality-check tool.

Does NOT move the robot. It tracks a thrown ball live (reusing
live_trajectory.py's release detection + trajectory.py's quadratic fit),
computes the one intercept point on a fixed catch plane, transforms it into
the robot's base frame via the existing calibration (calibrate_frames.py's
T_base_from_mocap.json + frames.py), and asks: could a movel from the arm's
*current* pose reach that point before the ball does? It prints a verdict and
a margin (seconds to spare, or seconds it would have missed by) so you can
throw the ball around and sanity-check the whole perception -> feasibility
pipeline before any code is allowed to touch the arm.

Physics, not vibes - the distance -> move_time model:
CLAUDE.md's Catch Integration plan calls for "a distance->move_time lookup
from ur_loop.py's measured per-leg timing - that curve is the feasibility
oracle." speed_char.py already logs exactly that (its `legs` list, comment:
"feeds distance->move_time later too"). A trapezoidal velocity profile
(accelerate to a cruise speed, cruise, decelerate; or a triangular profile if
the move is too short to reach cruise) predicts:

    move_time(d) = d/v + v/a                    (trapezoid, d >= v^2/a)
    move_time(d) = 2*sqrt(d/a)                   (triangle,  d <  v^2/a)

for cruise speed v and acceleration a. Rearranged, `move_time - d/v = v/a` is
linear in `v` for a fixed `a`, so `a` (and a constant startup/comms latency
`c`) are recovered from the recorded legs by one least-squares fit:
`y = move_time - d/v_achieved`, `x = v_achieved`, `y = x/a + c`. Fit against
the 2026-07-14 speed_char run: a ~= 4.6 m/s^2, c ~= 0.11s, residual RMS ~55ms
(max ~170ms) - the physics model's imperfection (CLAUDE.md notes the real
ceiling varies with pose/direction) becomes part of the required safety
margin below, not something the model pretends not to have.

Catch plane stays in mocap-frame axis/value (--catch-axis/--catch-value),
exactly like live_trajectory.py's --catch-axis/--catch-value - this reuses
trajectory.py's time_of_plane_crossing() solver as-is (it only knows how to
solve in the frame the fit was computed in). Only the resulting single 3D
catch *point* gets transformed into the robot base frame, not the whole fit.

Threading: identical split to live_trajectory.py - make_handler()'s callback
runs on the NatNet socket-recv thread and only ever touches the small
short-detection window; the full-flight fit and every feasibility
calculation here run in this module's own poll loop, off the hot thread.

Output is plain incremental console prints (not a redrawing Live dashboard)
on purpose: the point of this tool is throwing the ball repeatedly and
reading back a scrolling history of verdicts, which a screen-clearing TUI
would erase after every throw.
"""

import argparse
import glob
import json
import math
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import rtde_receive
from natnet import NatNetClient, DataFrame

from live_trajectory import STATE_LOCK, SharedState, add_release_detection_args, make_handler
from trajectory import AXIS_NAMES, Sample, fit_trajectory
from frames import mocap_point_to_base
from ur_goto_raw import ROBOT_IP
from calibrate_frames import send_urscript, set_tcp_script

# ur_rtde robotmode enum: motors are only powered (so getActualTCPPose/getActualQ
# report the *real* arm pose) from IDLE upward. Below this - POWER_OFF, BOOTING,
# etc. - RTDE hands back a default all-zeros joint vector, whose forward
# kinematics is a phantom pose out near max reach, NOT where the arm is. Trusting
# it silently made every catch look like a ~1.3m move from the wrong place. See
# docs/debug_log.md 2026-07-15.
ROBOTMODE_IDLE = 5

# Latest position of the arm's end-effector ("tool") rigid body, captured off the
# NatNet thread by its own handler (independent of the ball target selection).
TOOL_LOCK = threading.Lock()


@dataclass
class ToolState:
    latest_pos: Optional[tuple] = None
    latest_valid: bool = False


def make_tool_handler(tool_state: ToolState, tool_id: int):
    """A second, independent NatNet handler that only tracks the tool rigid body.

    Kept separate from live_trajectory.make_handler (which owns the *ball* target
    and its release detection) so the two never interfere - NatNetClient fans each
    frame out to every registered handler.
    """
    def handle_frame(frame: DataFrame) -> None:
        rb = next((r for r in frame.rigid_bodies if r.id_num == tool_id), None)
        if rb is None:
            return
        valid = True if rb.tracking_valid is None else rb.tracking_valid
        with TOOL_LOCK:
            tool_state.latest_pos = rb.pos
            tool_state.latest_valid = valid

    return handle_frame

# CLAUDE.md "Load-bearing accuracy number": predicted landing position doesn't
# stabilize below 5cm error until ~67 samples (~0.55s @ 120Hz) - this is the
# earliest point a predicted catch point should be trusted at all, rounded up.
MIN_SAMPLES_FOR_CHECK = 40

# CLAUDE.md hardware section: commanded speed is delivered 1:1 up to ~1.2 m/s
# (the reliable ceiling); above that it rolls off and becomes pose/direction-
# dependent. Assume a real catch move would be commanded at/above this and
# use it as the model's cruise speed - conservative relative to the ~1.32
# peak seen on the best-case segment.
# Was 1.2 - user directive 2026-07-15: 1.2 is the conservative/lower-end
# number, the arm can sometimes reach ~1.3 (see speed_char.py peak), so bias
# the model's assumed cruise speed up toward that.
DEFAULT_CRUISE_SPEED = 1.3

# UR12e reach is 1300mm (CLAUDE.md hardware section). Stay off both ends:
# near the base (singularity risk, CLAUDE.md "home position is a
# singularity") and right at the mechanical limit.
DEFAULT_MIN_REACH = 0.25
DEFAULT_MAX_REACH = 1.25

# Catch tool forgiveness radius: the box end-effector is 30cm wide, so it still
# covers the target if the tool center ends up up to half that width short of
# where it was aiming - even a move that can't fully *arrive* before impact
# still travels most of the way there. Matches CLAUDE.md's end-effector note
# ("10-15cm effective radius").
DEFAULT_BOX_RADIUS = 0.15


@dataclass
class MoveTimeModel:
    """Trapezoidal-profile move-time estimate, fit from a speed_char.py JSON run."""

    accel: float          # m/s^2, fit
    latency: float        # s, fit constant offset (comms + controller start-up)
    v_max: float           # m/s, assumed cruise speed for a catch move
    residual_rms: float   # s, fit quality against the legs it was trained on
    n_legs: int

    def estimate(self, distance: float) -> float:
        if distance <= 0:
            return self.latency
        d_accel = self.v_max**2 / (2 * self.accel)
        if distance >= 2 * d_accel:
            t = (distance - 2 * d_accel) / self.v_max + 2 * (self.v_max / self.accel)
        else:
            t = 2 * math.sqrt(distance / self.accel)
        return t + self.latency

    def estimate_array(self, distance: np.ndarray) -> np.ndarray:
        """Vectorized estimate() - same trapezoid/triangle split, for the catch-volume
        solver scoring many candidate points per tick."""
        d = np.maximum(np.asarray(distance, dtype=float), 0.0)
        d_accel = self.v_max**2 / (2 * self.accel)
        trap = (d - 2 * d_accel) / self.v_max + 2 * (self.v_max / self.accel)
        tri = 2 * np.sqrt(d / self.accel)
        return np.where(d >= 2 * d_accel, trap, tri) + self.latency

    def reachable_distance(self, time_budget: float) -> float:
        """Inverse of estimate(): how far the arm could get in `time_budget` seconds.

        Used for the box-tolerance "possibly catch" check - even a move that can't
        fully arrive before impact still gets the tool *some* distance along the
        way, and a wide catch tool forgives the rest. Same trapezoid/triangle
        split as estimate(), solved for distance instead of time.
        """
        tau = time_budget - self.latency
        if tau <= 0:
            return 0.0
        tau_cross = 2 * self.v_max / self.accel  # time to reach cruise speed (accel+decel)
        if tau <= tau_cross:
            return self.accel * tau * tau / 4.0  # triangular profile, never reaches cruise
        return self.v_max * tau - self.v_max**2 / self.accel  # trapezoid, cruise-limited


def fit_move_time_model(path: str, v_max: float) -> MoveTimeModel:
    with open(path) as f:
        data = json.load(f)

    xs: List[float] = []
    ys: List[float] = []
    for leg in data.get("legs", []):
        if not (leg.get("started") and leg.get("settled")):
            continue
        v, d, t = leg.get("peak"), leg.get("distance"), leg.get("move_time")
        if not v or t is None or d is None or v <= 0:
            continue
        xs.append(v)
        ys.append(t - d / v)

    if len(xs) < 6:
        raise ValueError(
            f"only {len(xs)} usable (started+settled) legs in {path} - need >=6 to fit "
            f"a move-time model. Re-run speed_char.py or pass a different --speed-char-json."
        )

    xs_a = np.array(xs)
    ys_a = np.array(ys)
    A = np.vstack([xs_a, np.ones_like(xs_a)]).T
    inv_a, latency = np.linalg.lstsq(A, ys_a, rcond=None)[0]
    if inv_a <= 0:
        raise ValueError(
            f"fitted acceleration from {path} came out non-positive (inv_a={inv_a:.4f}) - "
            f"bad/degenerate speed_char data, not usable as a feasibility model."
        )
    accel = 1.0 / inv_a
    resid = ys_a - (xs_a * inv_a + latency)
    residual_rms = float(np.sqrt(np.mean(resid**2)))

    return MoveTimeModel(accel=accel, latency=latency, v_max=v_max, residual_rms=residual_rms, n_legs=len(xs))


def latest_speed_char_json() -> Optional[str]:
    candidates = sorted(glob.glob("speed_char_*.json"))
    return candidates[-1] if candidates else None


def load_transform(path: str):
    with open(path) as f:
        data = json.load(f)
    R = np.array(data["R"])
    t = np.array(data["t"])
    return R, t, data.get("rmse_m"), data.get("created_utc")


@dataclass
class FeasibilityResult:
    n_samples: int
    crossing_t: Optional[float]
    time_to_impact: Optional[float]
    catch_point_base: Optional[np.ndarray]
    reach: Optional[float]
    reachable: Optional[bool]
    move_dist: Optional[float]
    move_time: Optional[float]
    margin: Optional[float]
    feasible: Optional[bool]
    current_tcp: Optional[np.ndarray] = None  # base-frame arm position the move was measured FROM
    shortfall: Optional[float] = None  # m the tool would still be short of the target at impact, if not feasible
    possible: Optional[bool] = None    # not fully feasible, but within box_radius of the target at impact
    impact_vel_base: Optional[np.ndarray] = None  # ball velocity (m/s, base frame) AT the plane crossing
    note: Optional[str] = None
    # Catch-volume mode only (volume_height > 0 and a valid candidate was chosen):
    vol_dz: Optional[float] = None      # m the chosen point sits above the catch plane (0 = old plane catch)
    tilt_deg: Optional[float] = None    # mouth tilt the chosen point needs (already capped at max_tilt)
    n_cand: Optional[int] = None        # valid volume candidates this tick
    orient_time: Optional[float] = None  # s the wrist needs to reach that tilt (part of `margin`)


def _solve_intercept(
    flight_buffer: List[Sample],
    catch_axis_idx: int,
    catch_value: float,
    R: np.ndarray,
    t_vec: np.ndarray,
    min_reach: float,
    max_reach: float,
    volume: Optional[dict] = None,
) -> dict:
    """The part of feasibility-checking that only changes when the ball fit does:
    fit the trajectory, solve for the catch-plane crossing, and transform the
    intercept point into the base frame. Split out of check_feasibility() so a
    caller polling faster than new mocap samples arrive (Motive tops out at
    120Hz) can cache this dict across ticks where len(flight_buffer) hasn't
    grown, instead of re-running an unchanged fit - see check_feasibility's
    `cache` parameter and catch.py's per-tick use of it."""
    fit = fit_trajectory(flight_buffer)
    last_t = flight_buffer[-1].t
    crossing_t = fit.time_of_plane_crossing(catch_axis_idx, catch_value, after_t=last_t)
    if crossing_t is None:
        return {"n": len(flight_buffer), "crossing_t": None,
                "note": f"trajectory never crosses catch plane {AXIS_NAMES[catch_axis_idx]}={catch_value}"}

    catch_point_mocap = np.array(fit.position(crossing_t))
    catch_point_base = mocap_point_to_base(catch_point_mocap, R, t_vec)
    # Ball velocity at impact (central difference on the fit - avoids assuming the
    # AxisFit coefficient layout), rotated to base frame. Consumed by catch.py's
    # --tilt-follow to aim the tool mouth into the incoming trajectory.
    eps = 0.01
    vel_mocap = (np.array(fit.position(crossing_t + eps)) - np.array(fit.position(crossing_t - eps))) / (2 * eps)
    impact_vel_base = R @ vel_mocap
    reach = float(np.linalg.norm(catch_point_base))
    reachable = min_reach <= reach <= max_reach
    solved = {
        "n": len(flight_buffer), "last_t": last_t, "crossing_t": crossing_t,
        "catch_point_base": catch_point_base, "impact_vel_base": impact_vel_base,
        "reach": reach, "reachable": reachable, "note": None,
    }
    if volume is not None and volume.get("height", 0.0) > 0.0:
        solved["vol"] = _solve_volume_candidates(fit, last_t, catch_axis_idx, catch_value, R, t_vec,
                                                 min_reach, max_reach, volume)
    return solved


# Catch volume (2026-10-19): instead of the single point where the ball crosses the
# catch plane, score every point of the flight that lies inside the slab
# [plane, plane + height] and pick the one the arm can reach with the most time to
# spare (move time AND wrist-tilt time both count). VOLUME_N_CANDIDATES times are
# sampled from now to the downward plane crossing - ~15ms apart on a 1s flight,
# one numpy pass, cached per new mocap sample like the plane solve.
VOLUME_N_CANDIDATES = 64
VOLUME_HYSTERESIS_S = 0.03   # keep last tick's choice unless another beats it by this much margin
# A point ABOVE the plane (more than VOLUME_PLANE_BAND_M up) is only eligible if the arm
# is at most this far short of it. Without this, a hopeless throw (5.4 m/s, ~0.5s short
# from the first tick, 2026-10-19) picked the "least hopeless" point - high and close to
# the base - and early-commit chased it there before chase-abort fired: the brave lunge
# that needed an E-stop. Hopeless throws now behave exactly like plane-only.
VOLUME_MAX_DEFICIT_S = 0.25
VOLUME_PLANE_BAND_M = 0.02

# Direction-dependent mouth-tilt limits (2026-10-19 volume_pose_check.py on the real arm).
# "Out" = mouth toward the thrower, i.e. away from the base at the target's azimuth: the
# wrist folds back toward the forearm, and out 90 made the box HIT the forearm (twice);
# out 45 cleared at every tested spot. Side +/-60 cleared everywhere. "In" (toward the
# base) was never tested, so it keeps the old validated --tilt-follow 20. Mixed
# directions are limited by the ellipse through these axis limits (clamp_tilt) - the
# tilt is scaled down along its own direction, never rotated to a new direction.
TILT_LIMIT_OUT_DEG = 45.0
TILT_LIMIT_IN_DEG = 20.0
TILT_LIMIT_SIDE_DEG = 60.0


def clamp_tilt(tau: np.ndarray, out_unit: np.ndarray, max_tilt_rad: float) -> np.ndarray:
    """Clamp horizontal tilt vector(s) `tau` (direction the mouth leans, length = tilt
    angle in rad; shape (2,) or (N,2)) to the direction-dependent limits above and to
    `max_tilt_rad` overall. `out_unit` = unit horizontal vector(s) pointing away from
    the base at the catch point. Shared by the volume solver and demo.py's commanded
    orientation (tilt_follow_orientation) so both agree on what's allowed."""
    tau = np.atleast_2d(np.asarray(tau, dtype=float))
    out = np.atleast_2d(np.asarray(out_unit, dtype=float))
    t_out = np.sum(tau * out, axis=1)
    t_side = out[:, 0] * tau[:, 1] - out[:, 1] * tau[:, 0]
    lim_o = np.where(t_out >= 0, math.radians(TILT_LIMIT_OUT_DEG), math.radians(TILT_LIMIT_IN_DEG))
    r = np.sqrt((t_out / lim_o) ** 2 + (t_side / math.radians(TILT_LIMIT_SIDE_DEG)) ** 2)
    tau = tau * np.where(r > 1.0, 1.0 / np.maximum(r, 1e-12), 1.0)[:, None]
    mag = np.linalg.norm(tau, axis=1)
    return tau * np.where(mag > max_tilt_rad, max_tilt_rad / np.maximum(mag, 1e-12), 1.0)[:, None]


def volume_height_at(r_h, volume: dict):
    """Max height above the catch plane allowed at horizontal distance r_h (m) from the
    base: 0 inside volume["zero_dist"], full volume["height"] beyond volume["full_dist"],
    linear between. Missing keys = full height everywhere. Shared with demo.py's
    envelope ceiling so the solver and the final gate agree."""
    h = float(volume["height"])
    z0, zf = volume.get("zero_dist"), volume.get("full_dist")
    if z0 is None or zf is None or zf <= z0:
        return h + 0.0 * np.asarray(r_h, dtype=float)
    return h * np.clip((np.asarray(r_h, dtype=float) - z0) / (zf - z0), 0.0, 1.0)


def _solve_volume_candidates(fit, last_t: float, catch_axis_idx: int, catch_value: float,
                             R: np.ndarray, t_vec: np.ndarray, min_reach: float, max_reach: float,
                             volume: dict) -> Optional[dict]:
    """Fit-dependent half of the catch-volume solve (positions, velocities, needed
    tilt, envelope validity per candidate time). The arm-position-dependent half
    (distance/move time/margin/choice) runs every tick in _choose_volume_point()."""
    t_end = fit.time_of_descending_crossing(catch_axis_idx, catch_value, after_t=last_t)
    if t_end is None or t_end <= last_t:
        return None
    ts = np.linspace(last_t, t_end, VOLUME_N_CANDIDATES)
    p_mocap, v_mocap = fit.sample(ts)
    p = mocap_point_to_base(p_mocap, R, t_vec)
    v = v_mocap @ R.T
    z_lo = float(p[-1, 2])                     # plane height in base frame (the last sample IS the crossing)
    # Height allowed above the plane shrinks near the base (2026-10-19 user request: high
    # targets close in fold the elbow a lot). Full height from full_dist outward, none
    # inside zero_dist, linear in between - horizontal distance from the base axis.
    z_hi = z_lo + volume_height_at(np.hypot(p[:, 0], p[:, 1]), volume)

    # Needed mouth tilt: angle between base +Z and the direction the ball comes FROM.
    vn = np.linalg.norm(v, axis=1)
    inc = -v / np.maximum(vn, 1e-9)[:, None]
    full = np.arccos(np.clip(inc[:, 2], -1.0, 1.0))
    max_tilt = float(volume["max_tilt_rad"])
    horiz = inc[:, :2]
    hn = np.linalg.norm(horiz, axis=1)
    h_unit = np.where(hn[:, None] > 1e-9, horiz / np.maximum(hn, 1e-9)[:, None], 0.0)
    pn = np.linalg.norm(p[:, :2], axis=1)
    out_unit = p[:, :2] / np.maximum(pn, 1e-9)[:, None]
    tau = clamp_tilt(full[:, None] * h_unit, out_unit, max_tilt)   # allowed tilt (see TILT_LIMIT_*)
    tilt = np.linalg.norm(tau, axis=1)
    tau_dir = tau / np.maximum(tilt, 1e-12)[:, None]
    n_des = np.column_stack([np.sin(tilt)[:, None] * tau_dir, np.cos(tilt)])  # commanded mouth normal

    reach = np.linalg.norm(p, axis=1)
    valid = ((reach >= min_reach) & (reach <= max_reach)
             & (p[:, 2] >= z_lo - 1e-6) & (p[:, 2] <= z_hi)
             & (full <= math.pi / 2 + 1e-6)    # never catch a ball still climbing (coming up from below)
             & (ts > last_t))
    wait_xyz = volume.get("wait_xyz")
    if wait_xyz is not None and volume.get("max_azimuth_deg") is not None:
        az_w = math.atan2(float(wait_xyz[1]), float(wait_xyz[0]))
        d_az = np.degrees(np.abs(np.arctan2(np.sin(np.arctan2(p[:, 1], p[:, 0]) - az_w),
                                            np.cos(np.arctan2(p[:, 1], p[:, 0]) - az_w))))
        valid &= d_az <= float(volume["max_azimuth_deg"])
    return {"ts": ts, "p": p, "v": v, "tilt": tilt, "n_des": n_des, "reach": reach,
            "valid": valid, "z_lo": z_lo}


def _choose_volume_point(vol: dict, last_t: float, current_tcp_xyz: np.ndarray, model: MoveTimeModel,
                         required_margin: float, volume: dict, prev_t: Optional[float]) -> Optional[int]:
    """Per-tick half of the volume solve: margin for every valid candidate from the
    arm's CURRENT position (and current mouth direction), best one wins, with
    VOLUME_HYSTERESIS_S stickiness so the target doesn't hop between heights on fit
    noise. Returns the candidate index (also leaves margins etc. in `vol`), or None."""
    valid = vol["valid"]
    if not valid.any():
        return None
    d = np.linalg.norm(vol["p"] - current_tcp_xyz, axis=1)
    move_t = model.estimate_array(d)
    orient_t = np.zeros_like(move_t)
    n_now = volume.get("mouth_normal_now")
    rate, acc = volume.get("orient_rate"), volume.get("orient_accel")
    if n_now is not None and rate and acc:
        ang = np.arccos(np.clip(vol["n_des"] @ np.asarray(n_now, dtype=float), -1.0, 1.0))
        orient_t = np.where(ang >= rate * rate / acc, ang / rate + rate / acc, 2 * np.sqrt(ang / acc))
    margin = (vol["ts"] - last_t) - np.maximum(move_t, orient_t) - required_margin
    above = (vol["p"][:, 2] - vol["z_lo"]) > VOLUME_PLANE_BAND_M
    eligible = valid & ~(above & (margin < -VOLUME_MAX_DEFICIT_S))
    if not eligible.any():
        return None                       # -> plain plane result, as if the volume were off
    score = np.where(eligible, margin, -np.inf)
    best = int(np.argmax(score))
    if prev_t is not None:
        idx = np.flatnonzero(eligible)
        j = int(idx[np.argmin(np.abs(vol["ts"][idx] - prev_t))])
        if score[j] >= score[best] - VOLUME_HYSTERESIS_S:
            best = j
    vol["_d"], vol["_move_t"], vol["_orient_t"], vol["_margin"] = d, move_t, orient_t, margin
    return best


def check_feasibility(
    flight_buffer: List[Sample],
    catch_axis_idx: int,
    catch_value: float,
    R: np.ndarray,
    t_vec: np.ndarray,
    current_tcp_xyz: np.ndarray,
    model: MoveTimeModel,
    min_reach: float,
    max_reach: float,
    required_margin: float,
    box_radius: float,
    cache: Optional[dict] = None,
    volume: Optional[dict] = None,
) -> FeasibilityResult:
    """`cache`, if given, is a caller-owned dict this function reads and updates.
    When cache["n"] already equals len(flight_buffer) - i.e. no new mocap sample
    has arrived since the last call - the fit/plane-crossing solve (_solve_intercept,
    the expensive ~0.3ms part) is skipped and last tick's solved intercept is
    reused; current_tcp_xyz-dependent quantities (move_dist/move_time/margin/...)
    are still recomputed every call regardless, since those depend on the arm's
    live position, which keeps changing tick over tick even between new ball
    samples. Pass a fresh {} per throw (not shared across throws) and None for
    call sites (e.g. this module's own --live console tool) that don't need the
    optimization.

    `volume`, if given with height > 0, switches on the catch volume (see
    _solve_volume_candidates): keys height (m above the plane), max_tilt_rad, and
    optionally wait_xyz + max_azimuth_deg (envelope wedge), mouth_normal_now (unit
    base-frame vector the box mouth currently faces) + orient_rate/orient_accel
    (rad/s, rad/s^2 - the wrist-tilt time model). Falls back to the plain plane
    result when no volume point is valid."""
    if cache is not None and cache.get("n") == len(flight_buffer):
        solved = cache
    else:
        solved = _solve_intercept(flight_buffer, catch_axis_idx, catch_value, R, t_vec, min_reach, max_reach,
                                  volume)
        if cache is not None:
            prev_t = cache.get("vol_prev_t")
            cache.clear()
            cache.update(solved)
            if prev_t is not None:
                cache["vol_prev_t"] = prev_t

    vol = solved.get("vol")
    if vol is not None:
        prev_t = cache.get("vol_prev_t") if cache is not None else None
        best = _choose_volume_point(vol, solved["last_t"], current_tcp_xyz, model, required_margin,
                                    volume, prev_t)
        if best is not None:
            if cache is not None:
                cache["vol_prev_t"] = float(vol["ts"][best])
            return _volume_result(len(flight_buffer), solved["last_t"], vol, best, current_tcp_xyz,
                                  model, box_radius)

    if solved["crossing_t"] is None:
        return FeasibilityResult(
            n_samples=len(flight_buffer), crossing_t=None, time_to_impact=None,
            catch_point_base=None, reach=None, reachable=None, move_dist=None,
            move_time=None, margin=None, feasible=False, current_tcp=current_tcp_xyz,
            note=solved["note"],
        )

    crossing_t = solved["crossing_t"]
    last_t = solved["last_t"]
    catch_point_base = solved["catch_point_base"]
    impact_vel_base = solved["impact_vel_base"]
    reach = solved["reach"]
    reachable = solved["reachable"]

    move_dist = float(np.linalg.norm(catch_point_base - current_tcp_xyz))
    move_time = model.estimate(move_dist)
    time_to_impact = crossing_t - last_t
    margin = time_to_impact - move_time - required_margin
    feasible = bool(reachable and margin >= 0)

    # Even when not fully feasible, the tool travels *some* distance toward the
    # target in the time actually available - a wide catch tool (box_radius)
    # forgives the rest. Use the raw time_to_impact (not margin-reduced) since
    # the box tolerance is doing the safety-margin job here, not a time buffer.
    shortfall = None
    possible = False
    if reachable:
        reachable_dist = model.reachable_distance(time_to_impact)
        shortfall = max(0.0, move_dist - reachable_dist)
        possible = (not feasible) and shortfall <= box_radius

    note = None if reachable else f"target {reach:.2f}m from base, outside [{min_reach:.2f}, {max_reach:.2f}]m"

    return FeasibilityResult(
        n_samples=len(flight_buffer), crossing_t=crossing_t, time_to_impact=time_to_impact,
        catch_point_base=catch_point_base, reach=reach, reachable=reachable, move_dist=move_dist,
        move_time=move_time, margin=margin, feasible=feasible, current_tcp=current_tcp_xyz,
        shortfall=shortfall, possible=possible, impact_vel_base=impact_vel_base, note=note,
    )


def _volume_result(n: int, last_t: float, vol: dict, i: int, current_tcp_xyz: np.ndarray,
                   model: MoveTimeModel, box_radius: float) -> FeasibilityResult:
    time_to_impact = float(vol["ts"][i] - last_t)
    margin = float(vol["_margin"][i])
    move_dist = float(vol["_d"][i])
    feasible = margin >= 0
    shortfall = max(0.0, move_dist - model.reachable_distance(time_to_impact))
    return FeasibilityResult(
        n_samples=n, crossing_t=float(vol["ts"][i]), time_to_impact=time_to_impact,
        catch_point_base=vol["p"][i].copy(), reach=float(vol["reach"][i]), reachable=True,
        move_dist=move_dist, move_time=float(vol["_move_t"][i]), margin=margin, feasible=feasible,
        current_tcp=current_tcp_xyz, shortfall=shortfall,
        possible=(not feasible) and shortfall <= box_radius, impact_vel_base=vol["v"][i].copy(),
        vol_dz=float(vol["p"][i, 2] - vol["z_lo"]), tilt_deg=float(math.degrees(vol["tilt"][i])),
        n_cand=int(vol["valid"].sum()), orient_time=float(vol["_orient_t"][i]),
    )


def format_result(r: FeasibilityResult) -> str:
    if r.crossing_t is None:
        return f"n={r.n_samples:3d}  no predicted catch-plane crossing ({r.note}) -> MISS"

    px, py, pz = r.catch_point_base
    frm = ""
    if r.current_tcp is not None:
        fx, fy, fz = r.current_tcp
        frm = f"from=({fx:+.3f},{fy:+.3f},{fz:+.3f}) "
    base = (
        f"n={r.n_samples:3d}  {frm}target_base=({px:+.3f},{py:+.3f},{pz:+.3f}) reach={r.reach:.2f}m  "
        f"dist_to_go={r.move_dist:.3f}m  t_impact={r.time_to_impact:+.3f}s  move_time={r.move_time:.3f}s  "
        f"margin={r.margin:+.3f}s"
    )
    if r.vol_dz is not None:
        base += f"  vol: +{r.vol_dz * 100:.0f}cm tilt={r.tilt_deg:.0f}deg ({r.n_cand} cand)"
    if not r.reachable:
        return f"{base}  -> MISS (unreachable: {r.note})"
    if r.feasible:
        return f"{base}  -> WOULD CATCH, {r.margin:.3f}s to spare"
    if r.possible:
        return f"{base}  -> POSSIBLY CATCH, tool would fall {r.shortfall * 100:.1f}cm short (within box tolerance)"
    return f"{base}  -> WOULD MISS by {-r.margin:.3f}s (tool would fall {r.shortfall * 100:.1f}cm short)"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-ip", default="192.168.10.1", help="Motive host IP")
    parser.add_argument("--local-ip", default="192.168.10.2", help="This machine's IP")
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")
    parser.add_argument("--rigid-body-id", type=int, default=None, help="NatNet rigid body id of the ball")
    parser.add_argument("--tool-rigid-body-id", type=int, default=None,
                         help="NatNet rigid body id of the arm's end-effector/tool. If given, the arm's "
                              "current position is read from THIS mocap body (transformed to base) instead "
                              "of robot FK - the ground-truth TCP, and the only trustworthy source when the "
                              "arm is powered off (RTDE returns a phantom all-zeros pose then). "
                              "Falls back to robot FK via RTDE if omitted.")

    add_release_detection_args(parser)

    parser.add_argument("--catch-axis", choices=AXIS_NAMES, required=True, help="Axis of the catch plane (mocap frame)")
    parser.add_argument("--catch-value", type=float, required=True, help="Value of the catch plane on --catch-axis (mocap frame)")

    parser.add_argument("--transform-file", default="UR10_T_base_from_mocap.json",
                         help="T_base<-mocap from calibrate_frames.py (default: UR10_T_base_from_mocap.json)")
    parser.add_argument("--speed-char-json", default=None,
                         help="speed_char.py output JSON to fit the move-time model from "
                              "(default: newest speed_char_*.json in the current directory)")
    parser.add_argument("--cruise-speed", type=float, default=DEFAULT_CRUISE_SPEED,
                         help=f"m/s assumed cruise speed for a hypothetical catch move (default {DEFAULT_CRUISE_SPEED})")
    parser.add_argument("--min-reach", type=float, default=DEFAULT_MIN_REACH, help="m, closest a target may be to the base")
    parser.add_argument("--max-reach", type=float, default=DEFAULT_MAX_REACH, help="m, farthest a target may be from the base")
    parser.add_argument("--margin", type=float, default=0.20,
                         help="s, required spare time beyond the raw move_time-vs-time_to_impact race "
                              "before calling it feasible (default 0.20 - covers the move-time model's "
                              "own fit error, ~170ms worst-case, plus a buffer)")
    parser.add_argument("--box-radius", type=float, default=DEFAULT_BOX_RADIUS,
                         help=f"m, catch-tool forgiveness radius - a move that can't fully arrive in time "
                              f"but gets the tool within this distance of the target at impact is reported "
                              f"'POSSIBLY CATCH' instead of a flat miss (default {DEFAULT_BOX_RADIUS} - half "
                              f"the 30cm box width)")

    parser.add_argument("--poll-hz", type=float, default=15.0, help="Feasibility-check/print rate during flight")
    parser.add_argument("--robot-ip", default=ROBOT_IP, help="UR12e controller IP (read-only RTDE)")
    parser.add_argument("--duration", type=float, default=None, help="Exit automatically after N seconds")
    parser.add_argument("--debug-release", action="store_true", help="Print every idle-state release-detection check")
    args = parser.parse_args()

    speed_char_json = args.speed_char_json or latest_speed_char_json()
    if speed_char_json is None:
        raise SystemExit(
            "No speed_char_*.json found in the current directory and --speed-char-json not given. "
            "Run speed_char.py first (see CLAUDE.md 'Robot Control') to characterize the move-time model."
        )
    model = fit_move_time_model(speed_char_json, args.cruise_speed)
    R, t_vec, calib_rmse, calib_created = load_transform(args.transform_file)
    with open(args.transform_file) as f:
        tcp_offset = json.load(f)["tcp_offset"]

    print(f"connecting to robot at {args.robot_ip} (read-only RTDE)...")
    rtde_r = rtde_receive.RTDEReceiveInterface(args.robot_ip)
    frac = rtde_r.getTargetSpeedFraction()

    # Decide where the arm's *current* position comes from. RTDE forward
    # kinematics is only trustworthy when the arm is powered (robotmode >= IDLE);
    # below that the joints read all-zeros and getActualTCPPose() is a phantom
    # pose near max reach - measuring a catch move from there is meaningless.
    use_tool_rb = args.tool_rigid_body_id is not None
    robot_mode = rtde_r.getRobotMode()
    if not use_tool_rb and robot_mode < ROBOTMODE_IDLE:
        raise SystemExit(
            f"Robot is not powered (robotmode={robot_mode}, need >= {ROBOTMODE_IDLE}/IDLE). "
            f"RTDE would return a default all-zeros joint pose, so every feasibility check "
            f"would be measured from a phantom location ~1.3m from where the arm actually is. "
            f"Power the arm on (pendant: initialize/brake-release), OR pass "
            f"--tool-rigid-body-id <id> to read the arm's true TCP from an end-effector mocap "
            f"rigid body instead (works with the arm off)."
        )

    if not use_tool_rb:
        # R/t was calibrated with the box/funnel-centroid TCP active, not the
        # flange (see calibrate_frames.py) - getActualTCPPose() below is only
        # measuring the same physical point the transform expects if that TCP
        # is (re-)sent here too, same as catch.py/track_rigid_body.py. Without
        # this, every printed reach/dist_to_go/margin number here is silently
        # off by the TCP offset.
        before = rtde_r.getActualTCPPose()
        send_urscript(set_tcp_script(tcp_offset))
        time.sleep(0.2)
        after = rtde_r.getActualTCPPose()
        print(f"set_tcp({tcp_offset}) sent. before={[round(v, 4) for v in before]} "
              f"after={[round(v, 4) for v in after]}")

    tool_state = ToolState()

    print("=" * 78)
    print("CATCH FEASIBILITY CHECK - console only, robot will NOT move")
    print("=" * 78)
    print(f"transform: {args.transform_file} (calibrated {calib_created}, rmse={calib_rmse * 1000:.1f}mm)")
    print(f"move-time model: fit from {speed_char_json} ({model.n_legs} legs) "
          f"accel={model.accel:.2f} m/s^2  latency={model.latency * 1000:.0f}ms  "
          f"residual_rms={model.residual_rms * 1000:.0f}ms  cruise_speed={model.v_max:.2f} m/s")
    if use_tool_rb:
        print(f"arm position source: mocap tool rigid body id={args.tool_rigid_body_id} "
              f"(ground truth, transformed to base) - robotmode={robot_mode}")
    else:
        print(f"arm position source: robot FK via RTDE (robotmode={robot_mode}, powered)")
    print(f"reach envelope: [{args.min_reach:.2f}, {args.max_reach:.2f}] m from base")
    print(f"required safety margin: {args.margin:.2f}s (on top of the raw move_time-vs-time_to_impact race)")
    print(f"box tolerance: {args.box_radius * 100:.0f}cm - a short-but-close move within this is 'POSSIBLY CATCH'")
    print(f"catch plane (mocap frame): {args.catch_axis} = {args.catch_value}")
    print(f"minimum samples before evaluating a throw: {MIN_SAMPLES_FOR_CHECK}")
    if frac < 0.99:
        print(f"WARNING: pendant speed slider at {frac * 100:.0f}% - a real catch move would be capped there "
              f"too; this feasibility check assumes {model.v_max:.2f} m/s is actually achievable.")
    print("=" * 78)
    print("Throw the ball. Ctrl-C to stop.\n")

    catch_axis_idx = AXIS_NAMES.index(args.catch_axis)

    s = SharedState()
    if args.rigid_body_id is not None:
        s.target_id = args.rigid_body_id

    client = NatNetClient(
        server_ip_address=args.server_ip,
        local_ip_address=args.local_ip,
        use_multicast=not args.unicast,
    )
    client.on_data_frame_received_event.handlers.append(make_handler(s, args))
    if use_tool_rb:
        client.on_data_frame_received_event.handlers.append(
            make_tool_handler(tool_state, args.tool_rigid_body_id)
        )

    last_state = "idle"
    last_verdict: Optional[FeasibilityResult] = None
    last_reported_id = None
    crossing_ever_found = False
    stop_checking = False

    with client:
        client.run_async()
        start = time.monotonic()
        try:
            while args.duration is None or (time.monotonic() - start) < args.duration:
                with STATE_LOCK:
                    target_id = s.target_id
                    candidate_ids = list(s.candidate_ids)
                    state = s.state
                    flight_buffer = list(s.flight_buffer)
                    history_head = s.history[0] if s.history else None

                if target_id is None:
                    if candidate_ids and candidate_ids != last_reported_id:
                        print(f"Multiple rigid bodies seen: {candidate_ids}. "
                              f"Re-run with --rigid-body-id <id> to pick the ball.")
                        last_reported_id = candidate_ids
                    time.sleep(1.0 / args.poll_hz)
                    continue

                if state == "flight" and last_state == "idle":
                    print(f"--- throw detected (rigid body {target_id}) ---")
                    last_verdict = None
                    crossing_ever_found = False
                    stop_checking = False

                if state == "flight" and len(flight_buffer) >= MIN_SAMPLES_FOR_CHECK and not stop_checking:
                    if use_tool_rb:
                        with TOOL_LOCK:
                            tool_pos, tool_valid = tool_state.latest_pos, tool_state.latest_valid
                        if tool_pos is None or not tool_valid:
                            # Can't measure the move without knowing where the arm is.
                            # Skip this tick rather than fabricate a distance.
                            time.sleep(1.0 / args.poll_hz)
                            last_state = state
                            continue
                        current_tcp_xyz = mocap_point_to_base(np.array(tool_pos), R, t_vec)
                    else:
                        current_tcp_xyz = np.array(rtde_r.getActualTCPPose()[:3])
                    result = check_feasibility(
                        flight_buffer, catch_axis_idx, args.catch_value, R, t_vec, current_tcp_xyz,
                        model, args.min_reach, args.max_reach, args.margin, args.box_radius,
                    )
                    if result.crossing_t is not None:
                        # A real crossing exists in the future relative to the latest sample -
                        # this is the only case worth remembering as "the" verdict for this
                        # flight; don't let a later None (see below) clobber it.
                        crossing_ever_found = True
                        print(format_result(result))
                        last_verdict = result
                    elif crossing_ever_found:
                        # The predicted crossing time has now passed relative to the latest
                        # sample - the trajectory DID cross the plane, we're just past it. Not
                        # "never crosses" (that phrasing only fits the branch below). The
                        # decision window has closed; stop re-checking and keep last_verdict
                        # (the last real margin) as the answer instead of overwriting it.
                        print(f"n={result.n_samples:3d}  catch plane already passed - "
                              f"locking final verdict from the last valid check above")
                        stop_checking = True
                    else:
                        # Genuinely hasn't crossed (yet, or possibly ever) - e.g. plane is
                        # above the trajectory's apex, or the fit doesn't reach it. Keep
                        # checking each tick in case a refining fit finds it later.
                        print(format_result(result))

                if state == "idle" and last_state == "flight":
                    if history_head is not None:
                        print(
                            f"--- throw ended ({history_head.reason}), "
                            f"duration={history_head.duration:.3f}s samples={history_head.samples} "
                            f"peak_speed={history_head.peak_speed:.2f}m/s ---"
                        )
                    if last_verdict is None:
                        print(f"    (never reached {MIN_SAMPLES_FOR_CHECK} samples - too short/fast to evaluate)")
                    else:
                        print(f"    final verdict: {format_result(last_verdict)}")
                    print()

                last_state = state
                time.sleep(1.0 / args.poll_hz)
        except KeyboardInterrupt:
            pass
        finally:
            client.stop_async()
            rtde_r.disconnect()


if __name__ == "__main__":
    main()
