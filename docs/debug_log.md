# RoboBaseball debug log

Detailed history: bugs found, experiments run, and dead ends explored. This file is
**not** auto-loaded into every session (unlike `CLAUDE.md` at the repo root) - read it
when you need the full story behind a decision in `CLAUDE.md`, or when debugging
something that might be a repeat of a past issue. `CLAUDE.md` holds only the current,
load-bearing state; this file holds the "how we got there."

## OptiTrack / NatNet

**Frame rate stuck at ~7/sec, resolved 2026-07-09**: the first test had a whole bag of
spare markers placed next to the rigid body (not noise/reflections - real markers,
intentionally there), showing up as 12 unlabeled markers. That bag was removed before
the second test, which came back at a full 120 Hz on both stream fps (packets/sec
received) and Motive fps (frame-number delta/sec), with 0 unlabeled markers. Plausible
that the extra ~12 markers added enough point-cloud reconstruction load to slow things
down, but this is correlation from one before/after comparison, not confirmed
causation - if frame rate drops again, don't assume "too many markers" is
automatically the cause. Check (in order): Motive's Status Panel for measured vs
configured rate, Log Pane for dropped-frame warnings, Windows Task Manager per-core CPU
(reconstruction is often single-core bound), USB hub/cabling bandwidth, and marker
count/clutter in the capture volume (intentional or not).

## Trajectory Fitting

Free-flight-start heuristic design rationale: automatic release detection, not a
button - a button's ~150-250ms human reaction latency would corrupt exactly the
samples needed to seed the fit. Continuously fits a short rolling window and requires
it to look ballistic (acceleration ~-9.81 on one axis, ~0 on the other two, low
residual) for several consecutive windows before declaring release.

**Tuning history for release detection** (current defaults are in CLAUDE.md):
started at `--window-short 6` (50ms) - too short, fitted acceleration too noisy at
realistic Flex 13 noise levels (~0.5mm/sample) since acceleration is a second
derivative that amplifies position noise. Moved to 10 samples (83ms), which held.
`--accel-tol` started at 1.5 m/s² and was validated against a real thrown
shirt-wrapped rigid body (5 throws, 2026-07-09, no changes needed) - but a second,
different clean single-throw recording **failed to detect the actual throw at all**,
only catching a secondary post-landing bounce ~3s later. Added `--debug-release` to
`live_trajectory.py`/`visualize_trajectory.py` to print every idle-state ballistic
check frame-by-frame. That showed the real release *did* briefly look ballistic (5.4
m/s, accel in tolerance) but never sustained 3 consecutive good windows - short-window
horizontal acceleration on this object had p50≈3 m/s², p90≈5 m/s², occasionally 7+,
all above the 1.5 default (residual stayed sub-mm, so `--residual-max` was never the
limiter). Meanwhile a slow post-impact bounce (~1.1-1.4 m/s) passed the old tight
tolerance easily since a small bounce is, locally, genuine free flight too - so it won
the race and got misdetected as "the throw," matching the first recording's symptom.
Swept `--accel-tol` 1.5-10 in 0.5 steps: real in-flight windows pass reliably by
tol≈4-6, windup/backswing stays clean (zero false-goods) up to tol≈6, leaks through at
6.5+. Landed on **6.0** with margin both sides. Tolerance alone doesn't fix bounce
misdetection (looser tolerance makes a clean slow bounce *easier* to pass), so also
added `--min-release-speed` (2.0 m/s default) - measured ~4.7-5.5 m/s at real release
vs ~1.1-1.8 m/s in the bounce, clean separation. With both fixes: release triggers
within ~30ms of real release, bounce no longer preempts it.

One observed quirk, not fixed: on a throw deliberately thrown too high (tracking
effectively stops mid-flight in Motive's own view), `rb.tracking_valid` apparently
never flips to `False` in the recorded data - position holds/jumps instead of
reporting invalid. The "lost tracking" end condition never fires; that flight only
closes via the 3s `--max-flight-duration` safety timeout (bounded, not stuck, but
`--lost-frames` occlusion detection can't be trusted to key off `tracking_valid` alone
on this dataset).

**Timestamp-monotonicity bug, found and fixed 2026-07-09**: the guard meant to drop
stale/duplicate packets didn't account for Motive's looped playback making
`frame.suffix.timestamp` jump *backward* at each loop restart (e.g. ~3537.8s back to
~3473.8s). The guard correctly dropped that one bad frame, but never updated
`last_sample_t` - so every later frame in the new loop pass (all less than the
previous pass's peak) also looked like a stale duplicate and got silently dropped,
killing detection for the rest of the run. Fixed by treating a backward jump bigger
than `STREAM_RESTART_GAP` (0.5s) as a stream restart - finalize any in-progress flight
and resume fresh - rather than a duplicate to discard. Will recur any time a recording
is looped for testing, not just on first real deployment.

**Sample-count-vs-accuracy experiment (2026-07-09)**: using a clean 4s single-throw
recording, measured how sample count in `fit_trajectory` affects predicted
ground-plane crossing accuracy vs. the actual observed landing point. Charts:
https://claude.ai/code/artifact/8f9e0bc5-854a-4d33-a910-2f8d1c0db06f. Landing-position
error is wild with few samples (up to ~3.3m off at N=12, ~0.09s observed) and doesn't
stabilize below 5cm until ~67 samples (~0.55s), out of a ~0.92s usable flight window
(this ~67-sample/~0.55s number is now load-bearing for the catching-latency budget -
see CLAUDE.md Open Questions).

Counterintuitive finding: fit `residual_rms` **increases** with more samples (0.7mm at
N=8 up to 9.3mm at N=111) even as the landing prediction gets far more accurate. Few
points let almost any quadratic fit nearly exactly, but velocity/acceleration is
nearly unconstrained, so the extrapolation is a coin flip. More points force the fit
to reconcile real deviations (drag, spin-wobble, sensor noise), so residual climbs
while extrapolation tightens. Don't use `residual_rms` as a trust proxy for
predictions - low residual on a short window means overfitting, not confidence.

**`visualize_trajectory.py` lock-condition investigation, 2026-07-09**: the live
prediction was never locking even after the release-detection fix above. Cross-checked
tentative predictions against the real observed touchdown (`x=+0.825, z=+1.471`) -
predictions were already accurate to 1-2cm well before flight end (e.g. `(+0.84,+1.48)`
from ~0.8s of flight onward). The lock condition was failing on measured *precision*,
not accuracy: `--stability-window 6` kept the last 6 predictions and required all
pairwise distances under `--drift-tol`, but early convergence still held 1-2 older,
noisier predictions alongside converged ones, inflating `spread` longer than needed.
Shrinking to `--stability-window 4` let old outliers age out faster without weakening
the agreement check. Combined with loosening `--drift-tol` from 2cm to 5cm (this
object's short-window fits are noisier than the earlier shirt-wrapped rigid body), both
now defaults, locked correctly on all throws tested - landing consistently locked at
`(+0.84,+1.48)`, matching real touchdown within ~1-2cm, with enough runway (locking
~0.85-1.0s into a ~1.15-1.2s flight) for the flash to be visible.

Confidence-trigger design rationale: no ground truth available live, so can't just
check "is error under 5cm." Uses **prediction stability** instead - keeps the last N
predictions, locks in once they agree within `--drift-tol`. A fixed sample-count
threshold (hardcoding ~67 from the experiment above) was considered and rejected -
calibrated on one throw's specific speed/distance, wouldn't generalize. Validated
against synthetic data and the real recorded throw (ground truth used only as an
offline check): converged around 0.5-0.65s of clean flight with 2-4cm actual error,
consistent with the sample-count sweep despite a completely different,
ground-truth-free trigger condition.

**Bug found and fixed 2026-07-09**: on real "big slow parabola" throws, only the
actual (blue) path ever rendered, plus occasionally a short straight orange stub -
never a full predicted arc. Root cause: axis bounds were computed only from actual
samples seen so far, never from the predicted curve. Matplotlib clips line artists to
the current axes viewport, and a high/long throw's predicted landing point can be
*meters* beyond where the still-short actual trail has reached early in flight -
confirmed empirically (22 samples in: actual trail x∈[0.25,0.51], predicted curve
reached x=2.02, almost entirely outside the box). Fixed by expanding tracked bounds
with the live and locked prediction curves' own extents too, not just the actual
trail. Added `--debug` for this kind of live troubleshooting (release/landing
transitions, measured render-loop rate, sample count/prediction/spread per tick).

**Snapshot/streak comparison, 2026-07-09**: added to make "how many samples does the
prediction actually need" visually inspectable live. Every `--snapshot-step` samples
(default 20), freezes a prediction curve from only the first N samples, color-coded,
kept on screen for the rest of the flight. A third panel shows the same idea against
the actual path, layered thickest/shortest-N on top of longest-N underneath (fixed
per-slot zorder) so shorter streaks aren't buried. Colors: tried a continuous colormap
(viridis) first, but with only ~6-8 snapshots per flight neighboring shades were too
close to tell apart - switched to a hand-picked maximally-distinct color list, fixed
per slot index. `--max-snapshots` (default 8) sets pooled line-artist count per view -
keep close to `flight_samples / snapshot_step` or later snapshots silently stop being
captured.

**Regression found while adding the third panel**: render throughput dropped from
~15Hz to ~6-9Hz (a third fully-decorated Axes plus ~24 more pooled line artists roughly
doubles per-frame redraw cost under `blit=False` - not worth chasing with `blit=True`
here since axis limits change every tick via `apply_bounds`, and blitting's cached-
background approach doesn't handle that cleanly). This mattered beyond cosmetics:
prediction convergence (`step_prediction`) was evaluated exactly once per render tick,
so fewer ticks in the same ~1.1s flight meant fewer chances to fill the stability
window before landing - confirmed directly, two throws that used to lock cleanly
stopped locking (spread stuck at 22cm/12cm) once the third panel dropped fps. This was
a latent fragility in the original design (convergence timing implicitly coupled to
render speed) that the extra panel simply exposed. Fixed by making `step_prediction`
sample-count-driven exactly like `capture_snapshots` already was - a `--predict-step`
(default 8) catch-up loop evaluates every reachable N boundary in one call, so a
slow-drawing tick still processes all intervening samples at once. Convergence timing
is now governed by the 120Hz data stream, not plot cost. Re-verified: still ~6-7Hz
render rate, but 3/3 throws locked again (N=120, 128, 136).

## Robot Control

### Architecture decision (2026-07-09/10)

Decided against raw URScript sockets and against ROS2
(`Universal_Robots_ROS2_Driver`) for driving the UR12e, in favor of `ur_rtde`.
URScript alone has no clean way to stream a continuously-updating target at high rate
without reimplementing RTDE by hand. ROS2 adds a DDS/controller-manager/MoveIt layer
between the trajectory predictor and the arm - more latency and moving parts than a
single reactive-catch demo needs. **This decision was later complicated**: `ur_rtde`
has not been gotten working on this specific robot/URCap combination (see below), and
raw URScript-over-socket - the exact approach initially rejected - turned out to be
the first (and so far only) confirmed-working control path. Current state and the
path forward are in CLAUDE.md; this section is the history of how that happened.

### ur_rtde troubleshooting saga (2026-07-10, ultimately unresolved)

**Confirmed working pieces:** Network link, Remote Control mode, `RTDEReceiveInterface`
(read-only state) - solid via `ur_rtde_test.py`. Dashboard Server (port 29999) via
`dashboard_client.DashboardClient` - fully reliable: `robotmode()`, `safetymode()`,
`programState()`, `isInRemoteControl()`, `closeSafetyPopup()`, `unlockProtectiveStop()`,
`stop()`, `play()`, `powerOn()`, `brakeRelease()`. The pendant's own Play button is
greyed out by design whenever Remote Control is on (expected UR behavior, not a bug) -
program start/stop must go through the Dashboard Server instead. External Control
URCap `v1.0.5` installs and runs: a pendant program with the node can be loaded and
`play()`'d remotely, and `RTDEControlInterface(ROBOT_IP, flags=FLAG_USE_EXT_UR_CAP)`
connects and holds a stable session (ran a bounded 150-tick/~3s zero-velocity `speedL`
loop with no fault).

Root-caused two distinct causes of protective stop `C271A1` ("runtime is too much
behind"), both client-side bugs, not robot/firmware faults - see
[[robot_control_testing_safety]] memory for full detail: (1) force-killing a script
mid-session via SIGTERM/timeout skips Python's `finally` cleanup, stranding the
robot's real-time thread; (2) a control loop printing/flushing every tick at full
control rate (50Hz) can jitter enough to trip the same fault - fixed in `jog_ur.py` by
decoupling HUD print rate (10Hz) from control loop rate.

An earlier "no inverse kinematics solution for this move" error turned out to be a
singularity (robot at home position), not a real fault - reproduced even jogging from
the pendant directly, no RTDE/script involved. Freedriving off home position fixed it
immediately.

**Confirmed NOT working:**
- Headless mode (`FLAG_UPLOAD_SCRIPT`, default, no URCap): connects to RTDE data fine,
  but the uploaded control script never signals it's running -
  `"Failed to start control script, before timeout of 5 seconds"`, 100% reproducible,
  independent of `FLAG_UPPER_RANGE_REGISTERS`. Root cause not identified.
- `FLAG_UPPER_RANGE_REGISTERS`: source-verified no-op on this controller.
  `rtde_control_interface.cpp`'s `setupRecipes()` hardcodes `register_offset_ = 0` for
  any PolyScope `5.23+` (this robot is `5.25`) regardless of the flag - only prints a
  warning. Using it anyway produced erratic ~1.7s per-`speedL`-call latency and an
  immediate "can't reach requested pose" fault - actively worse than default flags.
- `FLAG_USE_EXT_UR_CAP` with default (lower-range) registers - the best config found -
  connects cleanly, every `speedL()` call returns instantly (~0.0ms, no error) - but
  the real-motion test (`ur_move_test.py`, 0.02 m/s for 1s) produced zero measurable
  TCP displacement.

**Follow-up session, root cause narrowed but not fixed:** Systematically ruled out
every remaining hypothesis. Speed scaling is not the cause -
`ur_speedscale_diag.py` sampled `getSpeedScaling()` *during* an active 2s `speedL`
stream and got a solid `1.000` the whole time, with `isConnected()`/
`isProgramRunning()`/`getRobotStatus()` all healthy - yet TCP position was
bit-identical across 100 ticks. `moveL` (blocking) behaves inconsistently depending on
the `frequency` constructor arg: with default (`-1.0`, auto-detect), `moveL` returns
almost instantly printing `RTDEControlInterface: RTDE control script is not running!`
(a real, specific warning - also appears in `ur_rtde` GitLab issue #96, there
triggered by `stopL()` after `speedL()`; here it fired on the very first `moveL`
call). But with `frequency=500.0` explicit (matching the official docs example
verbatim), `moveL` instead hung indefinitely - had to be force-killed (`SIGKILL`)
after 25+s, since a blocking pybind11 C++ call doesn't return control to Python to
process a pending signal, so even the SIGTERM handler that works fine in `jog_ur.py`
never got a chance to run (see [[robot_control_testing_safety]]). Robot itself stayed
`NORMAL` safety mode throughout both - purely a stuck client, not a stuck robot. Each
time a test script's connection ended (clean or forced), the pendant logged a
"Receive program failed: Connection refused" popup as the External Control node tried
to auto-reconnect and found nothing listening - confirmed benign every time via
`ur_status.py` (safetymode stayed NORMAL), just meant `ur_play.py` needed rerunning.

**Conclusion:** looks like a genuine `ur_rtde` 1.6.3 <-> External Control URCap
`v1.0.5` compatibility issue, not a config mistake - registers, speed scaling, and
frequency were all systematically ruled out. Untried candidates for a future session:
an older URCap release (pre-1.0.5), a different `ur_rtde` build (GitLab master vs. the
1.6.3 PyPI release), whether `servoL`/`servoJ` behave differently from `speedL`/
`moveL`, or whether `ur_rtde`'s own unmodified stock example scripts behave
differently from our custom scripts (isolates "our code" vs. "library/robot version
mismatch"). Deprioritized after raw URScript-over-socket (below) gave a working
fallback, but still needed eventually for high-rate (500Hz) streaming control - a
hand-rolled persistent-socket streaming loop (URScript opening its own socket back to
the laptop) is the other alternative if `ur_rtde` remains unresolved.

### Raw URScript-over-socket breakthrough (2026-07-10)

After the `ur_rtde` dead end, tried bypassing it entirely: open a plain TCP socket to
`192.168.20.1:30002` (the secondary client interface) and send a small self-contained
URScript program as text, which the controller parses and runs directly. First two
attempts (`ur_raw_script_test.py` doing a `movel` Z round-trip, `ur_raw_jointmove_test.py`
doing a `movej` wrist round-trip) looked like they'd failed - a before/after pose/joint
diff showed ~zero net change - but the robot actually visibly moved both times. The
test design was flawed, not the motion: both scripts deliberately move out and back to
the exact original pose/joints, so a before/after delta of a closed round-trip is
always ~0 by construction even when real motion happened in between. Confirmed
properly with a non-round-trip single burst (`speedl([0,0,0.03,0,0,0], a=0.5, t=0.2)`
then `stopl(1.0)`): produced a real, measured +5.5mm Z displacement (expected ~6mm for
0.03 m/s over 0.2s accounting for accel ramp), with a clean `[0,0,0,0,0,0]` stop
afterward and `NORMAL` safety throughout.

`jog_ur_raw.py` built on this: same WASD/RF/IJKLUO/space/quit key scheme as
`jog_ur.py`, but instead of a persistent `ur_rtde` session, every control tick opens a
fresh socket to port 30002 and sends `speedl(velocity, a=ACCELERATION, t=NUDGE_DURATION)`
immediately followed by `stopl(STOP_DECEL)` in the same script - self-stopping by
design so a crash of the Python process mid-jog just leaves the robot finishing its
current bounded burst and stopping normally, never coasting or orphaning a real-time
thread. Motion is stepped/pulsed rather than smooth as a tradeoff.

### E-stop incident: ur_goto_raw.py scripting bug (2026-07-10)

Built `ur_goto_raw.py` to send absolute/relative target poses or joint angles (not
just velocity jogs) via `movel`/`movej`, since point-to-point goto is faster and a
better fit for "give the robot a location" than pulsed jogging. Two bugs, found in
sequence, on the live arm:

1. First version's `--relative` used URScript's `pose_trans(start_pose, delta)` to
   compute the target - this composes `delta` in the **tool's own local/rotated
   frame**, not base frame. A requested `[0,0,+0.03,0,0,0]` (pure +Z) produced a real
   but wrong displacement: dx=+0.0094, dy=-0.0281, dz=-0.0049. No fault, just the
   wrong direction - caught by comparing before/after pose.
2. The fix-attempt rebuilt the target as a **plain URScript list**
   `[start_pose[0]+dx, ...]` instead of a proper pose literal
   `p[start_pose[0]+dx, ...]`. `movel()` requires an actual `pose` type - passing a
   bare list produced a wildly wrong, large, fast motion (in one test: ~58cm in X,
   ~33cm drop in Z, large orientation change, versus a requested 3cm pure-Z move).
   **The user had to hit the physical E-stop** to halt it - `safetymode`/
   `safetystatus` read `ROBOT_EMERGENCY_STOP` afterward (not a routine protective
   stop). No injury/collision, arm position confirmed safe after stopping, e-stop
   released once the user confirmed - but this was a real safety event, not a
   near-miss caught in review.

**Fix**: don't compute the target pose via URScript expressions or `pose_trans` at
all - read the current pose with the already-proven read-only `rtde_receive`
interface, compute the absolute target in **Python** (plain float arithmetic,
position components only, orientation unchanged), and hand that finished absolute
pose to the exact `movel_absolute_script` code path that worked correctly on the very
first raw-script test. Also added a second, independent safety clamp
(`check_move_size` in `ur_goto_raw.py`) that refuses to send any move whose resolved
target implies more than 0.15m per translation axis or 0.5 rad per joint versus the
robot's current position, unless overridden with `--force`. Retested afterward with
the same small relative move: `after pose` matched the computed target exactly (Z
+0.03m, nothing else changed), `NORMAL` safety throughout.

**Lesson**: don't build pose/joint math as URScript expressions embedded in an
f-string and trust the syntax is right - verify function signatures before every new
primitive (as done for `speedl`/`stopl` before writing `jog_ur_raw.py`), and prefer
resolving target values in Python and sending them as plain literals over having the
robot-side script compute anything non-trivial. See [[robot_control_testing_safety]]
for the general pattern.

### Speed/accel limits research (2026-07-11)

UR12e's official tech spec page states "TCP Speed: Approx. 1 m/s" as the headline
figure (not just a cautious safety default) - separate from the UR10e hardware's raw
mechanical/marketing figure of 4 m/s (freedrive/unloaded, not relevant to commanded
moves). Joint speed: base/shoulder/elbow max 180°/s, wrist joints max 360°/s. No clean
published hard acceleration ceiling exists; UR's own script defaults are `a=1.2 m/s²`
for `movel` (just a typical example value). UR forum reports indicate acceleration and
velocity aren't independent - there's a real minimum ramp time regardless of how high
`a` is set (users report ~0.3-0.4s once `a` is already several multiples of `v`), so
pushing `--accel` far beyond `--speed` buys little past roughly 2-4x. Empirically
confirmed on this robot: commanding `--speed 10 --accel 10` did not achieve anything
close to that - consistent with the ~1 m/s documented ceiling. Also worth checking:
the pendant speed-slider override (`rtde_receive.getTargetSpeedFraction()`) multiplies
whatever speed is commanded - seen sitting at 0.2 (20%) in one earlier session, which
would silently cap all motion regardless of `--speed`; confirm it's at 1.0 before
concluding a speed ceiling is a hardware/safety limit rather than the slider.

### Speed characterization with peak measurement (2026-07-14) — supersedes the ~1 m/s conclusion above

Built `speed_char.py` to measure *achieved* TCP speed properly. Key methodological fix
over the 2026-07-11 note: read the true **peak** off `getActualTCPSpeed()` at ~500 Hz,
rather than inferring speed from `ur_loop.py` leg *timing* (which gives the ramp-diluted
*average*). For each commanded speed it runs several increasing-distance moves from a
fixed pose A and takes the fastest peak (a short move is a triangular profile that never
reaches cruise, so distance has to be long enough), with `--accel 40` pinned high so
accel is never the limiter (controller clamps it internally).

Run on a 0.967 m segment (`--pose-a -0.4799 -0.0621 0.5540 1.5716 -0.6653 -0.7089
--pose-b -1.0796 0.6455 0.2816 1.4842 -0.7987 -0.5374`), slider confirmed 100%:

| commanded | best achieved | ratio |
|-----------|---------------|-------|
| 0.20 | 0.203 | 1.02 |
| 0.40 | 0.404 | 1.01 |
| 0.60 | 0.605 | 1.01 |
| 0.80 | 0.806 | 1.01 |
| 1.00 | 1.005 | 1.01 |
| 1.20 | 1.202 | 1.00 |
| 1.40 | 1.324 | 0.95 |
| 1.60 | 1.324 | 0.83 |

Findings:
- **Commanded speed is delivered 1:1 up to ~1.2 m/s; peak achieved ~1.32 m/s.** So the
  real ceiling is ~30% above the datasheet's "approx. 1 m/s" — which is a *nominal/
  typical* figure, not a hard cap (confirmed by fetching the official UR10e SW5.22
  manual: "Tool: Approx. 1 m/s"). Nothing anomalous; the arm is doing what its joint
  limits allow.
- **The ceiling is configuration-dependent, not a constant.** At commanded 1.4, the
  0.75 m leg peaked 1.324 but the *longer* 0.967 m leg peaked *lower* at 1.204 — the
  controller scales the whole move down to keep the worst-case joint under its angular
  limit, and the far end of this path needs faster joint motion per unit TCP speed. So
  "max TCP speed" is a function of pose/direction; expect a different number elsewhere.
- **Why 2026-07-11 concluded ~1 m/s:** that was `ur_loop.py` leg-average. E.g. the
  0.967 m move commanded at 1.0 m/s took 1.30 s → 0.74 m/s average despite a 1.005 m/s
  peak. Peak ≠ average. Use `speed_char.py` for peak; use timing/`ur_loop.py` for the
  ramp-inclusive move *times* that actually gate catch feasibility.
- **Joint-speed spec correction:** official SW5.22 manual says base & shoulder **120°/s**,
  elbow & wrists **180°/s** — the old "180/360" figures in these docs were wrong (likely
  a UR10-non-e or marketing sheet). CLAUDE.md hardware section updated.
- Raw data/plot saved as `speed_char_20260714_143238.{json,png}`.

## Release detection: floor bounce mis-detected as a new throw (2026-07-15)

**Symptom:** a looped Motive playback of a single throw that bounces off the floor made
`visualize_trajectory.py` and `catch_feasibility.py` report **two** throws per loop —
the real throw plus a spurious one from the bounce. `--min-release-speed 2.0` was
supposed to reject exactly this (a bounce is locally genuine free-flight) but this
bounce was fast enough to slip through.

**Investigation (evidence, not guesswork):** captured 25s of the live multicast stream
to JSON and replayed the ball rigid body (id 3) through the *real*
`live_trajectory.make_handler` state machine. Per loop cycle it declared 2 releases:
- real throw: release at y≈0.9m, window speed ~6.0 m/s, apex y=2.28m, rise **+1.76m**,
  peak_speed 12.7 m/s.
- bounce: release at **floor level y≈0.05m**, window speed **~2.2 m/s** (just over the
  2.0 threshold), apex y=0.33m, rise **+0.05m**, peak_speed 1.9 m/s.

So the bounce passes the ballistic accel/residual check (it *is* a real low arc) and
squeaked past the old speed gate. The two clean discriminators: the bounce is far
lower-energy (2.2 vs 6.0 m/s release, 5cm vs 176cm rise) and always **follows a
landing**.

**Fix (two independent guards, both on by default):**
- `--min-release-speed` 2.0 → **3.0**. A floor bounce is lower-energy than the lofted
  throw that produced it; 3.0 sits between the measured ~2.2 m/s bounce and the ~4.9 m/s
  minimum release speed of a >1m-apex lofted throw (the demo requires lofted throws).
- new `--floor-refractory` (**0.5s**): after a flight ends by landing
  (`LANDING_REASON`, keyed in `finalize_flight`), release detection is suppressed for
  that long, so a rebound can't re-trigger *regardless of its energy*. The short window
  keeps filling during the refractory so detection resumes cleanly once it lapses.

**Confirmation:** replaying the recording with each guard alone AND both together gives
exactly **3 releases (the throws), 0 bounces** (was 6). Re-ran the real detector with
default args against the **live** loop for 24s → exactly 3 releases, all real throws
(rise +1.76m), 0 bounces. Real-throw detection unchanged (same release pos/rise/speed).

Guards live in the shared `live_trajectory` detector + `add_release_detection_args`
(so `catch_feasibility` inherits them); `visualize_trajectory`'s duplicated arg list
updated to match. Scratchpad capture/replay harnesses were throwaway; the live loop is
the reproduction if this recurs.

## 2026-07-15 — `catch_feasibility` "128cm short" on a clearly-catchable throw = phantom power-off TCP

Symptom: a looped Motive playback of a good lofted throw the arm should easily catch
(~30cm move from where the tool sat in Motive) reported `dist_to_go≈1.28m` on every
tick, verdict `WOULD MISS by ~1s (tool would fall ~128cm short)`, `move_time` stuck
~1.44s regardless of throw. Trajectory viz showed the predicted catch point was
accurate.

Root cause: **the arm was `POWER_OFF`.** With motors unpowered, `rtde_receive`
returns a default **all-joints-zero** vector, whose forward kinematics is a phantom
TCP stretched straight out at reach ~1.22m (`(-1.184,-0.289,+0.059)` base). The catch
point was correct (`(0.000,-0.789,+0.101)`, reach 0.80). `check_feasibility` differenced
the correct catch point against the phantom pose → 1.28m. A rigid transform preserves
distance, so the giveaway was that the *real* tool rigid body (mocap id=1, stationary)
transformed to `(+0.372,-0.710,+0.108)` base — **0.38m** from the catch point, matching
the eyeballed ~30cm. The move-time model, transform (27.7mm), trajectory fit, and
plane-crossing solver were all fine; the single bad input was `current_tcp_xyz`.
Confirmed by powering the arm on → verdict immediately flipped to `POSSIBLY CATCH`.

Diagnosis tell: `getActualQ()` returned exactly `[0,0,0,0,0,0]` — a real arm is never
at exact all-zeros; that's the "RTDE handed back a default, not the true state"
signature. `dashboard robotmode` = `POWER_OFF`.

Fix (`catch_feasibility.py`):
- Guard: read `getRobotMode()` at startup; if `< ROBOTMODE_IDLE (5)` and no tool RB
  given, `SystemExit` with an explicit "arm not powered → phantom pose" message rather
  than silently trusting FK.
- New `--tool-rigid-body-id`: read the arm's true TCP from an end-effector mocap rigid
  body (transformed to base via the same calibration) instead of RTDE FK — ground truth,
  and valid with the arm powered off. Own independent NatNet handler (`make_tool_handler`
  + `TOOL_LOCK`), separate from the ball target/release detection.
- Output now prints `from=(x,y,z)` (the base-frame position the move was measured from)
  and the arm-position source in the header, so a wrong origin is visible immediately.

## 2026-07-15 — `catch.py` built (first real arm-motion catch conductor)

New script `catch.py`: pre-positions the arm at a wait pose (~0.6m reach, catch
height ~1.1m floor = base-z ~+0.1), derives the horizontal catch plane through that
height, tracks a thrown ball (reusing live_trajectory release detection + trajectory
fit + catch_feasibility.check_feasibility), and fires ONE max-speed `movel` over the
raw-URScript-over-socket path (port 30002, ur_goto_raw idiom) when the predicted catch
point is stable (last N agree within --drift-tol) AND the gate passes (feasible OR
possibly-catch). Returns to the wait pose after each attempt.

Design constraints driving it:
- **Robot position from RTDE FK only, never mocap.** It's validated against a looping
  Motive *replay* of a recorded throw, in which the tool rigid body is frozen at its
  recorded spot - a mocap tool position would be meaningless. (Opposite choice from
  catch_feasibility's --tool-rigid-body-id, which is for the powered-off dry-run case.)
- **Own catch envelope** (`check_catch_envelope`), not ur_goto_raw's 0.15m clamp: reach
  0.35-1.0m, base-z -0.25..+0.55m (deck/singularity guard), and ≤0.6m from the wait pose
  (a catch is a short in-plane slide; a larger target = bad fit → refuse, no motion).
  This is CLAUDE.md's single reviewed "clamp-off" path.
- `--dry-run` (no motion, logs every decision) + a 'go' confirmation before first move.

Offline validation against the looped throw: default wait (0,-0.6,0.10) → derived catch
plane mocap Y=1.1029, which is within **2.5mm** of where the recorded ball actually
crosses (Y=1.1004); recorded catch point (0,-0.789,0.101) passes the envelope; required
move from wait = **0.189m** (short slide, easily catchable). Not yet run on the live arm.

## 2026-07-15 — `catch.py` first live-replay test: ball consistently hit the wrist,
## ~15cm short of the funnel = missing `set_tcp()`

Symptom (reported after the first real run against a Motive replay of two recorded
throws): the arm consistently drove to a point ~15cm *inward* of where the funnel
should have been — close enough to the wrist/last joint (wrist 3) that the ball hit
the arm itself instead of landing in the funnel, on both throws.

Root cause: **`catch.py` never sent `set_tcp()`.** `T_base_from_mocap.json` was
calibrated (`calibrate_frames.py`) with the box/funnel-centroid TCP active
(`tcp_offset = [0,0,0.12,0,0,0]`, box mounted flush at the flange) — every `p_robot`
sample used to fit `R,t` is therefore the *funnel-tip* location, not the flange, so
every target this transform produces is in funnel-tip coordinates. `set_tcp()` is a
controller-side runtime setting, not something baked into the calibration file or
carried by `movel()` itself — it must be (re-)sent by whatever script is about to
command motion. `track_rigid_body.py` already does this (loads `tcp_offset` from the
transform JSON, sends `set_tcp()` at startup) and was "reported working well"; `catch.py`
imported `catch_feasibility.load_transform()`, which silently drops the `tcp_offset`
field from the JSON, and never called `set_tcp()` at all — so it moved the *flange* to
where the funnel tip should have been, undershooting every catch by the full 12cm
offset (the flange sits 12cm back from the funnel along tool Z; matches the reported
~15cm within the noise of the box's real dimensions/mount vs. the nominal 12cm).

Confirmed empirically: added a `set_tcp()` call in `catch.py` (same pattern as
`track_rigid_body.py`) and printed `getActualTCPPose()` before/after. First run showed
`before=(-0.000,-0.600,+0.100)` → `after=(+0.118,-0.616,+0.113)` — an ~12cm shift on
exactly the axis/scale predicted, from a robot that had been sitting at the flange-TCP
wait-pose coordinates. Re-ran `catch.py --dry-run` against the same live replay
afterward: `target_base` values across ticks were stable and consistent throw-to-throw
(e.g. two separate throws both settling near `(+0.04,-0.69,+0.10)`), matching the
`COMMIT (CATCH)` numbers a healthy run should produce.

Fix: `catch.py` now reads `tcp_offset` directly out of the transform JSON and sends
`set_tcp()` right after connecting (before the `robotmode` check, mirroring
`track_rigid_body.py`), printing before/after `getActualTCPPose()` for visibility.
Applied the identical fix to `catch_feasibility.py` (read-only diagnostic companion) —
it had the same gap: no `set_tcp()`, so its printed `dist_to_go`/`margin`/verdict numbers
were silently measured from the flange whenever the arm is powered and `--tool-rigid-body-id`
isn't used. Not yet fixed at the source: `catch_feasibility.load_transform()` still drops
`tcp_offset` from its return value — both call sites now read it separately from the JSON
instead of through that helper, which is a minor duplication worth cleaning up if a third
caller appears.

Lesson: `set_tcp()` is **not persistent across a robot power cycle**, but it *is*
persistent across separate script invocations while the controller stays powered (a
later run's "before" pose already reflected an earlier script's `set_tcp()`) — don't
assume a freshly-started script inherits the calibration's TCP just because some
other script set it earlier in the session; every script that computes or consumes a
base-frame pose against the calibrated transform must send `set_tcp()` itself.

## 2026-07-15 — `catch.py` braver tuning fixed the default wait-pose orientation to a
## pose that happened to sit on the wrist singularity → two protective stops (C153A3)

Symptom: after fixing the wait-pose determinism bug above (position AND orientation
now fixed regardless of prior freedrive) and separately loosening the catch envelope
(`CATCH_MAX_REACH` 1.00→1.20m, `MAX_CATCH_MOVE` 0.60m cap removed entirely, `--margin`
0.10→0.03, `--commit-samples` 50→40, `DEFAULT_CRUISE_SPEED` 1.2→1.3 - all per user
request, same session), a throw triggered a real protective stop:
`C153A3: Position deviates from the path detected by the wrist joint 1`. Second
occurrence. The pendant's plain "enable" did not clear it; a full controller restart
did (by resetting joint state), which masked the real cause rather than fixing it.

Root cause: **the taught wait-pose orientation sat almost exactly on the UR wrist
singularity.** Diagnosed by comparing `ur_get_pose.py`'s joint readout at the taught
wait pose vs. immediately after the fault, at nearly the *same TCP pose*:

| joint | taught | post-fault | diff |
|---|---|---|---|
| J3 (wrist1) | +287.8° | +147.5° | **-140.4°** |
| J4 (wrist2) | **+5.5°** | **+0.9°** | -4.5° |
| J5 (wrist3) | -349.2° | -249.6° | **+99.6°** |

Wrist1/wrist3 differing by 140°/100° while the TCP pose is nearly identical is the
textbook signature of a UR wrist singularity, which occurs specifically at
**wrist2 (J4) ≈ 0° or ±180°** (the point where the wrist1 and wrist3 axes become
collinear and the Jacobian goes singular - a small commanded change in orientation
requires enormous wrist1/wrist3 joint velocity to track exactly). The taught wait
pose had J4 at +5.5°, well inside that danger zone. Every catch move launched *from*
that pose starts right at the ill-conditioned point, and the same-session envelope
widening made moves bigger/more varied (no more 0.6m move cap, reach out to 1.2m,
committing earlier/less certainly) - exactly what's more likely to force a wrist1/
wrist3 branch flip the real hardware can't track fast enough, tripping the wrist-1
path-deviation monitor. This is the risk CLAUDE.md's Open Questions already flagged
as untested ("a fixed A↔B loop test doesn't exercise the direction-change-heavy
motion a real catch trajectory will need").

Fix: re-taught the wait pose via freedrive + `ur_get_pose.py`, this time checking the
joint readout (not just the visual "looks upright" check) before accepting it. First
re-teach attempt still had J4 at -3.9° (still on the singularity, just a different
sign) - visual orientation is not a proxy for wrist2 angle, since wrist2 can sit at
0° in many different-looking tool orientations. Second attempt landed J4 at **+85.5°**
- near the best-conditioned point, maximally far from both singular values (0°/180°).
New `DEFAULT_WAIT_POSE` in `catch.py`: `(0.1139, -0.4686, 0.1335, 1.5840, -0.0824,
-0.0573)`.

Recovery note: `ur_status.py --clear` is the correct recovery path for a protective
stop (closes the safety popup, unlocks the protective stop, powers on + brake-releases
if needed via the Dashboard Server) - a full restart isn't necessary and only
resets joint state without addressing the geometric cause.

Lesson: **when teaching any pose the robot will move *from* (a wait pose, a
calibration anchor, etc.), check the joint angles, not just the Cartesian pose or how
it looks.** A pose can look fine and sit exactly on a singularity - TCP position/
orientation alone doesn't reveal it. `ur_get_pose.py` already prints joint angles;
the fix here was actually reading them (wrist2 specifically) before accepting a taught
pose, not just eyeballing the arm.

## 2026-07-15 — `catch.py --record` first real use: stability-window commit gate was
## silently discarding "possibly catch" opportunities on ~half of a 15-throw session

Symptom (user-reported): watching the console during a real throwing session, "possibly
catch" verdicts kept appearing but the arm often never moved for that throw - it would
print `possible` a couple of times, then `miss`, with no `COMMIT` line and no motion.

Diagnosis: recorded the session with `catch.py --record` (added earlier this session)
and matched the JSONL (`catch_log_20260715_162926.jsonl`, 15 throws, 3 commits) against
the Motive replay of the same run. Throw 2 is the cleanest example:

| tick | n | verdict | shortfall | stable | dist_to_go | t_impact |
|---|---|---|---|---|---|---|
| 1 | 43 | possible | 3.8cm (within 15cm box) | False | 0.386m | 0.664s |
| 2 | 50 | possible | 14.5cm (barely within box) | False | 0.433m | 0.615s |
| 3 | 56 | **miss** | 21.5cm (now outside box) | **True** | 0.450m | 0.567s |

`catch.py`'s commit rule required BOTH `gate` (feasible-or-possible on the current
tick) AND `trusted` (the last `--stability-window`=3 predictions agreeing within
`--drift-tol`, from `stable()`). `stable()` can't return non-`None` until the
`pred_window` deque has 3 entries, i.e. not before the 3rd eligible tick. But across
those first 3 ticks the predicted catch point kept sliding *further* from the arm's
current position as the fit refined (`dist_to_go` grew 0.386→0.450m, `t_impact` shrank
0.664→0.567s) - so the tick where `stable` first goes `True` is systematically also the
tick where the box-tolerance window has already closed. Checked programmatically
against the whole session, not just by eye: old code committed on only 3 of 15 throws
(1, 5, 10 - the ones with enough margin to survive 3 ticks unscathed). Simulating "fire
on the first feasible-or-possible tick" against the same recorded ticks would have
raised that to 10 of 15 - 6 throws (2, 4, 8, 11, 13, 15) were squashed by exactly the
early-possible-then-stability-lag pattern above; throws 3, 6, 7, 9, 14 never became
feasible/possible under either rule (genuinely unreachable or too-late targets, a
different/structural issue, correctly unaffected).

One throw (12) is a useful cautionary case rather than a clean win: after a long
`no_crossing` gap the fit was visibly still chaotic - `dist_to_go` swung
97m→9.4m→2.6m→0.88m→0.21m→0.11m→0.32m→0.47m→0.66m across consecutive ticks - and the
old stability gate correctly stayed `False` throughout, refusing to commit to any of
it. The new instant-fire rule would have committed on the first `catch` tick in that
swing (`dist_to_go=0.88m`, itself sandwiched between wildly different neighbors) -
exactly the kind of still-converging, uncorroborated prediction the stability check
existed to filter. Firing on it isn't obviously wrong (the envelope clamp still bounds
where it can go, and it's one throw out of 15), but it's the real shape of the
tradeoff being accepted below, not a hypothetical one.

Root cause: the stability check was designed to filter a still-noisy early prediction,
but in a race where the time budget only ever shrinks, *waiting* for it is strictly
worse than acting on the first passing tick - it can only cost margin, never gain any.
`--commit-samples` (40) is already below the "prediction <5cm error" load-bearing
number (~67 samples, see `CLAUDE.md` Trajectory Fitting) - a deliberate earlier
brave-tuning tradeoff - so the stability window was added on top to partially
compensate, but ended up defeating its own purpose once the shrinking-margin dynamic
was accounted for.

Fix (user directive: "always send it if it's ever possible"): commit fires on the
FIRST tick where `gate` is true and `n >= commit_samples`, full stop. The
stability-window average is still used as the commit *target point* when it happens to
already be available (pred_window keeps accumulating regardless of this gate, so it's
free), falling back to the raw current-tick `catch_point_base` otherwise - so the
change loses zero smoothing when 3 ticks are cheaply available, it just stops blocking
the decision on waiting for them. Implemented in `catch.py`'s flight-tick handler;
`--stability-window`/`--drift-tol` help text and the module docstring's "Commit rule"
updated to match. Simulated against this session's recorded ticks (programmatic replay,
not by eye): commits go from 3→10 of 15 throws - 6 clean recoveries of the
early-possible-then-squashed pattern above, plus throw 12's noisier case discussed
above, with no change to the genuinely unreachable/no-crossing throws.

Tradeoff made explicit: a commit can now fire on a still-converging n=40 prediction
with zero stability corroboration, which the old gate was there to prevent. The
existing `check_catch_envelope` clamp is unchanged and still bounds every commit to the
reach/z safety band regardless - worst case of a bad early commit is a plausible arm
move to a safe-but-wrong point (still a miss), not an unsafe one. Not yet validated
against a fresh live session with the fix in place - do that next, ideally with
`--record` again so the before/after commit rate is directly comparable.

## 2026-07-15 — first live session with the instant-fire fix: real protective stop on
## a catch move that targeted reach=0.370m, plus a masked ~30s undetected-fault window

Validated the instant-fire fix (previous entry) with two real recorded sessions
(`catch_logs/catch_log_20260715_165200.jsonl`, 8 throws, and
`catch_log_20260715_165504.jsonl`, 12 throws - moved into the new `catch_logs/`
directory this session). User confirmed it's "a lot better" (session 1: 6 of 8 threw
committed a catch attempt, vs. the old ~3-of-15 rate). Session 2 hit a real fault:
user heard a sound and had to manually unlock the robot mid-session.

**Finding the fault in the log.** Cross-checked every commit's target reach against
`throw_end`'s `arm_tcp_at_end` (both logged fields) across both sessions - 13 of 14
commits matched their target exactly (arm completed the move); throw 2 of session
165504 did not:

```
throw_start  arm_tcp reach=0.500 (wait pose)
tick n=41    verdict=catch  target reach=0.370  dist_to_go=0.148  margin=+0.078  stable=False
commit       target=(0.034,-0.344,0.132)  v=1.5 m/s  a=6.0 m/s^2
throw_end    arm_tcp_at_end=(0.083,-0.443,0.146)  reach=0.474   <- NOT the commanded target
move return_to_wait  settled=True  duration_s=0.40  (suspiciously fast)
```

The commit fired on the very first eligible tick (n=41 - this is the instant-fire fix
working as designed) at reach=0.370m, just 2cm inside the *old* `CATCH_MIN_REACH`=0.35m
floor - explicitly commented as "near-singular / too close to the body" territory even
before this incident. The commanded displacement from the wait pose was 0.148m, but the
arm's actual end-of-throw position was only 0.042m from the wait pose - **~28% of the
intended move** - and its orientation also stopped partway between the wait-pose
orientation and the commanded one. That's the signature of a trajectory aborted
mid-flight, not one that arrived and settled normally: the same fault family as the
2026-07-15 wrist-singularity incident above (fast movel into an ill-conditioned,
folded-up configuration forces a joint velocity/path-tracking rate the controller
can't sustain), this time on the opposite workspace boundary (near-base/reach-floor
instead of wrist orientation).

**A second, independent bug this exposed.** The `return_to_wait` move immediately
after throw 2 logged `settled=True` after only 0.40s - suspiciously close to the
minimum possible (5 consecutive <2mm/s polls at 50ms = 0.25s, plus overhead). Checking
the gap to the next throw: `throw_start` for throw 3 didn't fire until **30.8 seconds
later**. That reconciles completely: the robot was actually frozen by the protective
stop for that whole ~31s (not moving at all), and `move_to()`'s settle check - which
only ever watched `getActualTCPSpeed()` - couldn't tell the difference between "arrived
and stopped" and "frozen because the controller rejected the command while faulted."
Both read as sustained near-zero TCP speed. So the script silently logged success and
would have kept running (and could have tried to fire more catch moves at an
unresponsive, faulted arm) had a throw come in during that window - only the user
manually noticing the fault and clearing it (via the pendant, presumably - not
`ur_status.py --clear`, since nothing in the log shows a script restart) got it moving
again in time for throw 3, which then completed normally.

**Fixes (both in `catch.py`):**
1. `CATCH_MIN_REACH` raised 0.35→0.45m. Empirical, not theoretical: every commit that
   completed normally across both sessions landed at reach≥0.538m; the one fault was
   the only commit below that. 0.45m gives ~8cm margin above the observed fault (vs.
   the old floor's 2cm) and ~5cm margin below `DEFAULT_WAIT_POSE`'s own reach
   (0.5004m, computed precisely - a flat 0.50m floor was tried first and rejected
   because it left the wait pose only 0.4mm inside its own envelope, one calibration
   nudge from rejecting itself on startup). The true safe boundary between 0.37m and
   0.538m is uncharacterized - no joint-angle telemetry was logged for either the
   fault or the successes, so this trades away that slice of workspace rather than
   guess at exactly where it's safe.
2. New `check_safety_mode(rtde_r)` (checks `getSafetyMode() == 1`/NORMAL) wired into
   three places: inside `move_to()`'s settle-wait loop (catches a fault during the
   initial approach or a return-to-wait move, replacing the speed-only check that
   missed this one), and once per iteration of the main poll loop (catches a fault
   during the fire-and-forget catch movel itself, which has no blocking wait of its
   own to instrument). Any non-NORMAL mode now logs a `fault` event and hard-stops the
   script via `halt_on_fault()` - no auto-resume attempt, matching this project's
   established recovery path (a human clears the stop via the pendant or
   `ur_status.py --clear`, confirms the arm's real position, then restarts). Skipped
   in `--dry-run`, which never sends motion so an unrelated fault shouldn't interrupt
   a pure perception-testing session.

Not yet validated against a fresh live session with both fixes in place - the reach
floor removes this specific target from ever being commanded again, and the fault
detector should turn any *future* incident into an immediate, loud stop instead of a
silent ~30s blind spot, but neither has been exercised live yet.
