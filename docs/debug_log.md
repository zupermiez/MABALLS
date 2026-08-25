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

## 2026-07-16 — SSH access to the UR controller, robot-side log pulling, and a
## `catch.py` session-robustness pass (Enter-to-stop, fault-wait instead of
## hard-halt, session wrap-up, raw trajectory capture)

**SSH access.** Confirmed working: `root@192.168.20.1`, default creds `root`/`easybot`
per UR's docs (this rig's password had already been changed). Two real gotchas hit
getting it going:
1. SSH is **off by default** on e-series - has to be enabled once on the pendant
   (`Settings → Security → Secure Shell`), and that page is greyed out while the
   pendant is in Remote Control mode (same "one source of control" ISO reasoning as
   the Play-button lockout) - had to flip the physical mode switch to Local first.
2. `ssh-copy-id` installed the key correctly (confirmed via `~/.ssh/authorized_keys`
   on the robot, correct permissions), but a bare `ssh root@192.168.20.1` still
   prompted for a password. `ssh -v` showed why: the server's own
   `Authentications that can continue: password` never even listed `publickey` -
   `/etc/ssh/sshd_config` had `PubkeyAuthentication no` baked into UR's factory
   image. Flipped to `yes` + `systemctl restart sshd` (safe - only bounces the SSH
   daemon, no interaction with `urcontrol`/the real-time motion process) and key auth
   worked immediately. A `~/.ssh/config` alias (`ur12e`) with an explicit
   `IdentityFile` was also needed since the key wasn't one of OpenSSH's
   default-named files (`id_ed25519` etc.) - without that, a bare `ssh`/`scp` never
   even offers the key.

**What's actually on the robot, once in.** `/root/log_history.txt` is the machine
log behind PolyScope's Log tab - compact `::`-delimited lines, timestamped, with
cryptic codes (`C153A0` etc.) decodable via `/root/GUI/bundle/errorcodes-*.jar`
(pulled out and saved to `docs/ur_error_codes_en.properties` - just grep it, e.g.
`C153A0` = "Detected by the Base joint... robot could not follow the path"; `C286A1`
turned out to be a benign motor-encoder homing message, not a fault, despite sitting
next to real fault codes). `/root/polyscope.log` is the fuller GUI-side log, already
human-readable including explicit `Flight reporter triggered, safety mode:
PROTECTIVE_STOP` lines. `/root/flightreports/*.zip` are UR's own auto-generated
incident bundles - `summary.log` (incident time, program state, active TCP
offset/payload, URCap versions) plus full `log_history.txt`/`polyscope.log`
snapshots and per-joint binary telemetry (`robot_metrics/joint*.bin`) - the richest
single artifact for debugging a real fault. **Caught live**: watched a new report
evict an old one mid-session - **only the last 5 are kept**, oldest deleted on every
new trigger, not disk-space-driven (4.9GB free at the time). Pulled all 5
then-current reports into `robot_logs/flightreports/` (gitignored, local) before they
could rotate away, including the 2026-07-15 13:45/13:49/15:41/16:55 recordings that
correspond to the incidents already narrated earlier in this file - those now have
real forensic backing instead of just notes.

**`catch.py` changes** (user directives, this session):
1. **Enter-to-stop, alongside Ctrl-C.** `enter_pressed()` does a non-blocking
   `select()` on stdin each poll tick rather than a background thread blocked on
   `input()` - deliberate, because a lingering blocked `input()` thread would race
   the wrap-up prompts (below) on the same stdin once the loop actually exits.
2. **Fault handling rewritten twice in one session.** First pass added
   `recover_from_fault()`: auto-clear via the Dashboard Server (mirroring
   `ur_status.py --clear`'s chain exactly) + drive back to the wait pose + resume,
   capped at N consecutive auto-clears before hard-exiting. User's actual intent,
   stated after seeing this: they want to clear the fault **themselves** on the
   pendant, not have the script do it - "I want to be able to clear the alarm from
   the robot and keep going." Replaced with `wait_for_fault_clear()`: no
   dashboard-server calls at all, just polls `check_safety_mode()` and blocks
   (printing a reminder every 15s) until a human clears it, then drives back to the
   wait pose and resumes automatically. This fully supersedes the `halt_on_fault()`
   description earlier in this file (2026-07-15 entry above) - that function no
   longer exists. Both versions share the reasoning that it's safe to not preserve
   much fault detail in-process, since the robot's own logs (above) capture full
   forensic detail regardless and get pulled at session end either way (next point).
3. **`wrap_up_session()`** - once the NatNet/RTDE connections are fully torn down
   (deliberately outside the hot loop/`try-finally`, so none of this can add latency
   to the live trajectory/feasibility calculation), prompts for an optional session
   name + free-text description, then `pull_robot_session_logs()` does one SSH
   round-trip pulling that session's wall-clock-timestamp-filtered slice of
   `log_history.txt`/`polyscope.log` (awk range filter on the embedded timestamps,
   not a byte-offset diff - simpler, and needs no start-of-session SSH call at all)
   plus any flight report zip triggered during it, into
   `robot_logs/sessions/<timestamp>_<name>/`. Runs on every exit path.
4. **Raw per-throw trajectory capture.** Investigating "should Motive's own data get
   transferred off the Windows PC" surfaced a real gap: the actual ball position
   samples (`trajectory.Sample(t,x,y,z)`) were never being persisted anywhere.
   Worse, they're **not safely recoverable from `SharedState.flight_buffer` at
   `throw_end`** - `finalize_flight()` (the NatNet callback thread) resets
   `flight_buffer` to `[]` in the same atomic step that sets `state = "idle"`, so by
   the time the main poll loop (a separate, slower loop) notices the idle
   transition and reads `s.flight_buffer`, it's already empty. Fixed at the source:
   `FlightRecord` (in `live_trajectory.py`, shared by `catch.py`/
   `catch_feasibility.py`/`visualize_trajectory.py`) gained a `raw_samples` field,
   populated inside `finalize_flight()` itself - the only point in time it's
   actually available - and logged by `catch.py` as a new `throw_samples` JSONL
   event. Verified end-to-end with a synthetic flight (`finalize_flight()` →
   `FlightRecord.raw_samples` populated, `flight_buffer` empty immediately after,
   confirming the race) and a real pull of live robot logs. Net effect: a session
   recorded with `--record` no longer needs a Motive replay to answer "what did the
   ball actually do on throw N" - full raw trajectory is in the JSONL. Full `.tak`
   transfer (camera/marker-level data) stays a manual, occasional thing - not worth
   automating a Windows-side pull for something only needed for rare, deep
   debugging, unlike the robot logs which the project already leans on constantly.

## 2026-07-16 (session "thu", 12:55-12:59) — recurring `C153A0`/`C157A0` base-joint
## protective stops, root-caused via flight-report telemetry, plus a real fault-
## detection bug found along the way

**Symptom**: 4 protective stops in a 12-throw session (`robot_logs/sessions/
20260716_125504_thu/`), user-reported as "kept hitting some error code very often
after catching the ball." `log_history_slice.txt` showed `C153A0` (x3, "position
deviates from path, detected by the Base joint") and `C157A0` (x1, "collision
detected... Base joint") - decoded via `docs/ur_error_codes_en.properties`. UR's own
suggestion text for both: "check payload, center of gravity and acceleration
settings."

**Root cause, found by going past the log text into the flight reports' own
`realtimedata.csv`** (500Hz joint/TCP telemetry UR auto-captures ~25s before/~5s
after every incident, in each `recording*.zip`): all 4 trips show the *identical*
signature - a clean, deliberately-accelerating move starting exactly at the wait
pose, heading toward a plausible catch-envelope target, tripping the base joint
240-450ms in, at only ~1.0-1.3 m/s of the commanded 1.5 m/s / 6.0 m/s². This is not
a physical impact (smooth accel ramp, not a jolt; user confirmed no collision/human
contact from the Motive side either) - it's the catch movel's own commanded
acceleration exceeding what the controller's dynamic model will tolerate. Directly
confirmed for the 4th incident (12:59:00, throw 11) against a real logged `commit`
event with matching speed/accel params 0.69s earlier. The robot's payload was
configured as **1.00kg, CoG=[0,0,0]** (a symmetric point mass at the flange) -
almost certainly wrong for an offset funnel/tool, which is exactly the mismatch that
makes the expected-vs-actual torque model diverge under a fast movel.

**A second, independent bug found while tracing this**: matching each physical
trigger (from `log_history`/flight-report timestamps) against `catch.py`'s own
JSONL `fault`/`fault_cleared` events revealed `check_safety_mode()` (backed by
`rtde_r.getSafetyMode()`) didn't register any of the 4 faults until a suspiciously
consistent **~8.87s** after the real trigger - and for throw 11 specifically, the
logged `move` event for "return to wait" reported `settled: True, fault: None`
while the robot's own telemetry proves it was frozen in `PROTECTIVE_STOP` that
entire time. This is the exact "silently stuck arm" bug already described earlier
in this file (2026-07-15) as fixed - it recurred. Root cause not confirmed (no live
robot access during this investigation), but the pattern (an RTDE-cached register
lagging by many seconds after a protective stop) matches known `ur_rtde` staleness
issues with `RTDEReceiveInterface` around controller-side faults.

**First three incidents' triggering movel never fully identified.** Only the 4th
incident has a matching `commit` in the JSONL; the first three have no commit or
move logged in the many seconds beforehand, despite matching telemetry signatures.
User confirmed no second `catch.py`/control-script instance was running and no
physical contact occurred. Given the fault-detection lag bug above, the most likely
explanation is that these *were* real catch movels from throws that DID commit, but
whose `commit` events landed at a wall-clock time this analysis didn't check
closely enough while the session's `check_safety_mode()` was itself blind for
several seconds - not fully closed out, flagged here rather than re-litigated.

**Fixes applied to `catch.py`/`calibrate_frames.py`** (not yet validated on the real
robot - start with `--dry-run`, then a deliberate bench test before trusting live):
1. **Payload/CoG fix.** First pass added `--payload-mass`/`--payload-cog` as
   *required* CLI args plus a new `set_payload_script()` (`calibrate_frames.py`,
   mirrors `set_tcp_script()`) sent at startup, mirroring how `set_tcp()` has to be
   resent every run because other scripts leave a different value behind. User's
   actual intent, stated after seeing this: payload doesn't need resending every
   run the way TCP does, because nothing else in this toolchain ever sets a
   different one - "I just put it on the pendant since it never changes." Reverted:
   `catch.py` no longer touches payload/CoG at all; it's set once via the pendant's
   Installation → Payload Estimation wizard and trusted to persist (saved in the
   installation file). `set_payload_script()` removed from `calibrate_frames.py` as
   dead code. If `C153A0`/`C157A0` recurs, check the pendant's payload value first.
2. **`check_safety_mode()` now checks the Dashboard Server (port 29999,
   `dashboard_client.DashboardClient.safetymode()`, stateless per-request query -
   same mechanism `ur_status.py` already uses) as the primary source of truth**,
   not `rtde_r.getSafetyMode()` alone - the latter is still read and included in the
   fault string for comparison/diagnosis. Threaded a persistent `dash` connection
   (connected at startup alongside `rtde_r`, disconnected in the same `finally` as
   `rtde_r.disconnect()`) through `move_to()`/`wait_for_fault_clear()`/the main loop.
3. Not changed: `--accel` (still defaults to 6.0 m/s², already user-configurable) -
   recommend testing at a lower value once the payload/CoG fix is in place, since
   that may turn out to be sufficient on its own.

## 2026-07-17 (night shift) — quantitative analysis of all 23 real 2026-07-16
## catch sessions (196 throws), root cause of the remaining protective stops,
## and a batch of offline-validated catch.py changes

Analysis scripts lived in the session scratchpad (not committed); every number
below is reproducible from `catch_logs/catch_log_20260716_*.jsonl` alone.

**Dataset**: 23 non-dry-run sessions, 196 throws, 108 commits (55%), 33 fault
events. Ball peak speeds 2.8–9.5 m/s (median 5.3), flight durations median 1.05s.
Commits fired at median n=43 samples (min 40 = `--commit-samples`), median
time-to-impact 0.63s, median move distance 0.31m. 78/108 commits were "possible"
(box-forgiveness) vs 30 full "catch". The `--stability-window` average was used as
the commit target **0 of 108 times** — commits fire on the first eligible tick,
when `pred_window` necessarily has 1 entry, so that code path was dead in practice
(it now matters again for the post-commit re-aim, see below).

**Estimated catch rate: ~91% of committed throws** (67/74 classifiable). A caught
ball is *occluded by the box* — its throw ends "lost tracking" with the ball last
seen <30cm from the tool; missed balls were last seen 1.5m+ away. This
last-seen-near-tool heuristic matched the session notes well and is now printed
live per throw (see changes).

**Remaining protective stops are geometric, not payload.** After the 14:34
payload fix, fault rate vs the commit target's azimuth swing from the wait pose
(accel<=4 sessions): 0% under 10°, 17% at 10–20°, 36% at 20–35°, 50% above 35°.
(Pre-fix accel=6 sessions: 3%/27%/62%/67%.) Mechanism: a movel holds a straight
Cartesian line at commanded TCP speed; at reach r that demands base-joint speed
~v/r — at the observed faulting commits (r=0.44–0.79m, v=1.1–1.5 m/s) that is
137–195°/s, over the 120°/s base joint limit → C153A0 "position deviates from
path". This is why lowering `--accel` helped but never fixed side throws: it's a
*velocity* violation on a *path*, not an acceleration problem. A movej cannot
violate joint limits by construction (the controller plans in joint space), so
`--catch-move movej` (+ `--yaw-follow`, below) is the fix to test on the arm.

**Prediction accuracy vs sample count** (replayed from `throw_samples` against
each trajectory's actual recorded plane crossing, 76 throws): free-quadratic fit
median/p90 error = 10.4/23.5cm at n=30, 6.4/16.9 at n=40, 2.7/4.7 at n=67,
1.6/2.8 at n=80. Confirms CLAUDE.md's "~67 samples for <5cm" number on real data.
Tested alternatives — **both worse, do not "improve" the fit this way**:
gravity-locked to -9.81 on mocap Y (5.1/9.7cm at n=67) and locked to the
empirical median -9.73 (6.0/10.9 at n=67). The free fit absorbs drag, calibration
tilt and marker-centroid wobble into its fitted accel; constraining it to physics
adds bias that outweighs the variance saved, already by n≈40. (Full-flight fitted
Y-accel across 147 flights: median -9.73, std 4.46 — the spread is why.)

**Re-aim opportunity**: between the commit (n≈43) and n=80 the predicted catch
point moves median 4.4cm / p90 10.7cm — real error that vanishes if re-aimed. At
n=67 there is still median 0.42s (p10 0.33s) to impact; the arm is usually
already at/near the committed point (arriving early is the design), so the
correction is a short move from rest. This is the data behind the new re-aim
behavior (fires only from rest, never preempts).

**False-release hazard quantified**: real throws release >=1.1m (horizontal, p5;
median 2.16m) from the base moving toward it (angle p90 = 27°). The 8 recorded
events under 1.0m / >100° away were all a hand handling the ball near the robot —
the same class as the 15:13 self-collision (session 151342 throw 9: ball grabbed
out of the box was detected as a throw, committed the arm to a target 158° behind
the wait azimuth, elbow self-collided). That throw's own trajectory was never
recorded because the fault path skipped throw_end/throw_samples logging — also
fixed.

**catch.py changes (offline-validated; NOT yet run against the real arm)**:
1. **Release guard** (`check_release_guard`, default on): release must originate
   >=1.0m (horizontal) from the base (`--min-release-dist`) and not be moving
   >100° away from it (`--max-away-deg`). Replayed over all 161 recorded throws:
   blocks exactly the 8 hand/away events, passes everything else. Guarded throws
   log a `guard` event and never commit.
2. **Azimuth band in `check_catch_envelope`** (`CATCH_MAX_AZIMUTH_DEG=75°` from
   the wait-pose azimuth, on for every commit/re-aim when `wait_xyz` is passed):
   backstop for targets behind/beside the demo corridor — the 151342 target
   (125° swing) is refused by unit test; all real committed targets (max 44°) pass.
3. **Post-commit re-aim** (default on, `--no-reaim`): after the committed move
   settles (TCP speed <0.01, never preempting motion) the loop keeps refitting;
   if the refined prediction drifts >=`--reaim-min` (2cm) off the committed
   target, the correction still fits the time budget, and the new target passes
   the envelope, it sends a short correction move (same primitive as the commit,
   max `--reaim-max-count` 3/throw, logged as `reaim` events).
4. **`--catch-move movej` + `--catch-joint-speed/--catch-joint-accel`** (default
   still movel): catch move via `movej_to_pose_script` (IK robot-side, qnear =
   actual joints) — the anti-C153A0 option for side throws. **`--yaw-follow`**:
   rotates the target orientation about base Z by the target's azimuth delta so
   the mouth-up box pans with the base instead of the wrist fighting to hold a
   fixed world orientation (quaternion composition, unit-tested to 1e-15 against
   rotation matrices). Both default OFF until tested on the arm.
5. **`--poll-hz` 20 → 50** (each poll interval is pure decision latency before
   the commit; 20Hz cost 25ms mean / 50ms worst ≈ 3–6cm of arm travel). Console
   prints throttled to ~12/s so the terminal stays readable; ticks recorded
   regardless.
6. **Live catch/miss tally**: throw_end now logs/prints `caught_guess` +
   `ball_last_dist_m` (last-seen-near-tool heuristic, <30cm) and a running
   session score; totals go in run_end and the wrap-up notes.txt.
7. **Faulted throws now get their `throw_end`/`throw_samples` logged** (captured
   post-fault-clear, deduped via the FlightRecord identity) — previously the one
   class of throw with no trajectory in the log was exactly the one that faulted.

Re-aim replay caveat: only 6 committed throws had an observable "true" crossing
to score against (caught balls occlude before crossing the plane), so the direct
replay (3 improved / 2 worsened) is weak evidence either way; the strong evidence
is the 76-throw accuracy-vs-n table above (error strictly shrinks with n, med
6.4cm→1.6cm from commit-time to n=80).

## 2026-07-17 (later) — night-shift changes promoted to defaults; open question on movej accuracy

Ran a first real session with the whole night-shift batch (release guard,
azimuth band, re-aim, `--catch-move movej`, `--yaw-follow`, `--poll-hz 50`) live
on the arm: `python3 catch.py --record --accel 4.0 --speed 1.2 --rigid-body-id 3
--catch-move movej --yaw-follow --wait-pose 0.042 -0.716 0.139 1.584 -0.0824
-0.0573 --approach-speed 1.5`. No new faults observed, so all of it — plus that
exact operating point — is now the script's default (see CLAUDE.md "Catch
Integration" and "Status"): `--catch-move movej`, `--yaw-follow` default on
(`movel`/`--no-yaw-follow` still available as escape hatches), `--accel 4.0`,
`--speed 1.2`, `--approach-speed 1.5`, wait pose `(0.042, -0.716, 0.139, 1.584,
-0.0824, -0.0573)`, `--rigid-body-id` defaulting to `3`.

**Open question, not yet root-caused**: operator impression from this session
that catch accuracy was a bit worse than under the old `movel` default. Nothing
quantified yet — no side-by-side A/B on matched throws, just a feel during the
session. Candidate causes worth checking before trusting movej for a tighter
tool than the current funnel:
- `catch_move_script`'s movej path resolves IK via `get_inverse_kin
  qnear=current joints` robot-side — if the returned elbow/wrist configuration
  isn't quite the one assumed elsewhere (e.g. `yaw_follow_orientation`'s
  rotation math, or the re-aim correction move recomputing IK from a different
  `qnear` after the arm has moved), the final TCP pose could differ slightly
  from the intended Cartesian target even though the *commanded* pose is
  identical to what movel would have received.
- Re-aim (`reaim` correction sends) still uses whatever `catch_move_script`
  resolves at correction time — worth checking whether a movej correction from
  a different qnear than the original commit converges to the same pose or
  drifts.
- Could also just be movej's cruise/blend profile reaching the target pose with
  a different final-approach velocity than movel, interacting with however "hit
  the target" was being judged (JSONL commit vs re-aim pose diff, not yet
  compared to actual TCP-at-catch from `throw_end`).
Next step: replay/compare `throw_end`'s actual TCP pose vs the committed
target pose across this session's JSONL, movej throws vs a matched movel
baseline, before concluding there's a real effect.

## 2026-07-18 (night shift) — full forensic pass on the 2026-07-17 sessions, marker-calibration root cause, re-aim found dormant

All numbers reproducible from `catch_logs/catch_log_20260717_*.jsonl` alone via
the scripts now in `analysis/` (analyze_0717.py, deep_0717.py, kinks_0717.py,
calib_forensics.py, tcp_hypothesis.py, bias_0717.py, refused_map.py). 12
sessions, 160 throws, 98 committed attempts, 75 caught of 93 classifiable
(81%). Sessions 10:52–10:56 ran movel/no-yaw-follow (old wait pose), 11:02–15:48
movej+yaw-follow, 15:51 movel+yaw-follow. All sessions all day ran the OLD
07-14 transform (`T_base_from_mocap.json`, run_start confirms) — the new
marker calibrations were never used live.

### 1. The misses are rim hits, not wild misses
For every attempted throw the ball's true catch-plane crossing was
reconstructed two ways: from the raw recorded samples (recorded crossing) and
from a clean mid-flight ballistic fit (samples 15–45, extrapolated to the
plane — immune to post-contact deflection). Result: on 14 of 17 measurable
missed throws the ball got within 13–21cm of the committed target center
(min ball-to-target distance) and its recorded trajectory shows a violent
post-contact kink — i.e. it REACHED the box and bounced off the rim/out
(matches the two session notes "went in but bounced out"). Ballistic
prediction error on missed throws: median 13.1cm (p25 9.7, p75 19.4). The
box's effective radius (~15cm) sits mid-distribution: the current system's
total error budget lands balls ON THE RIM, and rim hits mostly bounce out.
Two consequences, in order of leverage:
- A modestly wider and/or energy-absorbing mouth (net/foam rim/deeper funnel)
  converts most rim hits to catches: would have been ~89/98 (91%) vs 75/98.
- Cutting the residual 10–20cm error (re-aim actually firing + tighter
  calibration) does the same in software.
Important correction to an earlier read: a naive "prediction error" computed
against the RECORDED crossing (median ~45cm, non-converging with n) is
garbage on committed throws — the recorded crossing is post-deflection. Use
the ballistic reconstruction.

### 2. Arm error is zero; re-aim is dormant
On every measurable miss the arm's TCP at throw_end was at the committed
target to ~0.0cm — misses are 100% prediction, 0% motion. The re-aim
mechanism built to repair exactly this fired on only 2 of 98 attempts.
Blocker classification per attempt (from post-commit ticks): 69 "arm never
settled in time" (travel consumed the remaining flight), 27 "settled but no
time budget left". The settle-first design is structurally too late at these
flight times — hence `--reaim-preempt` (below).

### 3. movej accuracy question: RESOLVED — no regression
- Caught-ball proximity proxy (ball_last_dist on caught throws): movej median
  9.9cm (n=52) vs movel 11.2cm (n=23) — movej marginally TIGHTER.
- Catch rates: movej 52/63 (83%); morning movel 8/9; evening movel+yaw-follow
  15/21 (71%, and that session's notes say harder throws + it was last).
- Ballistic prediction errors don't differ by move kind.
Operator impression of worse movej accuracy is not supported; the "worse"
feel came from rim-out misses, which are throw/prediction-driven. movej
stays a sound default.

### 4. Yaw-follow worked — and is invisible by design
Commanded orientations DID rotate (|d_az| median ~9–11°, max 41° across
committed throws; the 15:51 movel session also ran with it on). No visible
wrist motion is the DESIGN: yaw-follow keeps the wrist joints still relative
to the arm's plane (the base pans, the tool orientation follows), where
no-yaw-follow would make the wrist counter-rotate to hold world orientation.
For visible wrist action that's also functional, `--tilt-follow DEG` (new,
opt-in) tilts the mouth into the incoming trajectory (recovers
cos(incidence) aperture loss; e.g. a 37° incidence throw gets the full 20°
cap and comes down to 17°).

### 5. Why 62/160 throws never committed
- 46 feasibility-gate refusals: they crossed the plane a median 0.51m from
  the wait pose with median 0.56s to impact at the first tick — a ~0.7s move
  against 0.56s of warning. Median shortfall 0.41m: NOT marginal; these are
  genuinely too fast/flat for a ~1.25m/s arm from a standing start. Even
  waiting at the refusal-cloud centroid would rescue only ~7/44. The honest
  lever is throw discipline (loft) — or v2 mid-flight streaming.
- 10 release-guard rejections, of which 5 released at 0.89–0.99m — right
  under the 1.0m threshold and plausibly REAL close-range throws (the other
  5: 0.46–0.79m + wrong-direction, correctly rejected). If throwing from
  close is desired, `--min-release-dist 0.85` looks safe (the azimuth
  envelope still backstops); not changed as default.
- 2 azimuth refusals (targets 118°/170° behind), 2 too-short flights, 1 no
  plane crossing. Wait pose vs all-throw crossing centroid: crossings center
  ~(-0.12,-0.91) vs wait (0.04,-0.72) — shifting the wait pose ~15–20cm
  toward -x/-y would center the workload slightly; small win, worth a try.

### 6. Camera reconfig did NOT shift the mocap frame
Evening (post-reconfig) throws show no directional bias shift vs morning
(mean error vector +32,-17,0 → +25,0,0 mm; medians 93→105mm) and nothing
resembling the 69mm marker-vs-old-transform offset. The old transform stayed
valid all day. (This is also what acquits the camera reconfig in the
marker-calibration failure below.)

### 7. Single-marker calibration root cause: a stale/wrong TCP poisoned p_robot
The two marker runs (15:02/15:17, RMSE 80.3/74.2mm) are internally awful but
agree with EACH OTHER to 13mm median in the catch zone while both sitting
69mm from the (still-valid) old transform → a systematic shared by both
runs, not noise. Discriminator: using the old transform as ground truth for
the marker's true base position, |p_robot − marker_true| = 99±23mm (run 1)
/ 93±31mm (run 2) with direction swinging ~75° across poses — a FIXED ~10cm
offset in the FLANGE frame between the point the robot reported and the
physical marker. The marker sat at flange+1–1.7cm; the box TCP is 12cm:
the effective TCP during p_robot reads was ~10cm out along tool Z, i.e. the
z=0.01/0.017 set_tcp either never took effect or didn't describe the real
marker. Umeyama can neither detect nor absorb an orientation-dependent
error, so it smeared into 80mm RMSE. Not a mocap problem, not the camera
reconfig, not single-marker noise. Fixes implemented (all offline-tested,
`python3 frames.py` self-test reproduces the exact failure synthetically:
plain fit 80mm → joint solve 0.85mm, offset recovered to 0.07mm):
- `frames.fit_transform_with_tool_offset()`: joint solve of (R, t) AND the
  marker's fixed tool-frame offset d from full 6D TCP poses. Marker
  placement precision no longer matters at all — anywhere rigid on the tool
  works — and the printed |d| doubles as a mounting sanity check.
- `calibrate_frames.py` now records the full TCP pose per sample, runs the
  joint solve by default (`--no-solve-offset` reverts), and
  `apply_and_verify_tcp()` wiggle-tests set_tcp at startup (+5cm tool-Z
  probe must move the reported pose) so an ignored write aborts loudly.
- Old-format JSONs still resume (offset solve skips pose-less samples).
Expectation for the next calibration run: the same 25-pose procedure should
now land at or below the old 27.7mm — likely well below, since the joint
solve also absorbs the box-pivot-vs-TCP mismatch that plausibly dominated
the ORIGINAL 27.7mm (its similarity-fit scale 0.987 hints some volume-level
error too; if RMSE stays >15mm after the joint solve, re-wand the volume).

### 8. New catch.py flags (both opt-in, neither real-arm validated)
- `--reaim-preempt` (+ `--reaim-preempt-min`, default 5cm): re-aim may
  replace the running catch move (program starts with an explicit stopj()).
  Validate at low speed first; watch for protective stops at the preemption
  instant. This is the targeted fix for finding #2.
- `--tilt-follow DEG`: see #4. Watch IK reachability near the envelope edge.
`impact_vel_base` (ball velocity at plane crossing, base frame) was added to
`FeasibilityResult` to support tilt-follow.

## 2026-07-20 — v2 servoj streaming layer built and validated on the real arm

First working piece of CLAUDE.md's step (5): continuous setpoint streaming,
replacing the commit-then-locked single-move model. Two new files,
`ur_servo.py` (transport) and `track_ball_servo.py` (perception in the loop).
Both worked on the first real-arm run.

### Why the obvious approach can't work
Every existing motion script opens a fresh TCP connection to port 30002 and
sends a complete program. At servo rates that fails twice over: a connection
per tick, and each send *preempts* the program still running from the previous
tick. So the per-tick-send pattern this repo has used everywhere else does not
extend to 125Hz — it is not a tuning problem, it is structural.

The fix is the **reverse socket**: send ONE persistent program, once, which
calls `socket_open()` back to the laptop. The robot is the TCP client, the
laptop the server; setpoints then stream over that already-open connection.
This is what ur_modern_driver / ur_client_library / ur_rtde all do internally,
and UR's own `servoj` documentation describes it explicitly: *"x is a pose
variable with target cartesian positions, received over a socket or RTDE
registers."* Sidesteps the parked `ur_rtde` control-session issue entirely —
`rtde_receive` stays read-only, as everywhere else here.

### Three URScript details that would each have caused a real incident
Verified against the PolyScope 5 Script Directory PDF rather than assumed
(this controller is PolyScope **5.25.0.130258**):
1. `socket_read_binary_integer(number, socket_name, timeout)` — a timeout of
   **0 or negative means "block until a read completes"**, NOT "return
   immediately". Passing 0 would silently disable the host-crash watchdog and
   leave a dead host holding the arm's last setpoint with no program-side
   escape. `servo_program()` now raises on any non-positive timeout, with a
   self-test asserting it.
2. On timeout the same call returns **`[0,-1,-1,-1]` — a SHORT list**, not a
   zero-filled one of the requested length. Indexing `pkt[7]`/`pkt[8]` on that
   branch is a runtime error mid-servo, so those fields are read only inside
   the `pkt[0] >= 8` branch.
3. `get_inverse_kin_has_solution(pose, qnear, ...)` — confirmed parameter name
   and position from the manual. Guards `get_inverse_kin` so an unreachable
   setpoint is ignored rather than raising an exception that kills the program
   while the arm is moving. Available since PolyScope 5.10.

### Design choices worth not re-litigating
- **Latest-wins, not a circular buffer.** UR's servoj article recommends
  buffering waypoints; that advice is for replaying a precomputed trajectory
  where every point matters. Here every setpoint is superseded by the next, so
  queueing would only add latency. The robot's servo thread is self-timed by
  `servoj`'s own `t` and always reads the newest `cmd_q`.
- **Poses over the wire, IK on the robot.** Streaming joints would mean writing
  and validating a UR kinematics implementation at the same time as debugging
  the transport — two unvalidated things at once, and an incident you can't
  attribute. `qnear` is the last COMMANDED q, not the measured one, so IK
  solution choice can't chatter between branches as the arm lags.
- **Rate limiter is the load-bearing safety layer.** `servoj` has NO speed
  limit of its own: hand it a distant q and it drives there as hard as `gain`
  allows. So the setpoint STREAM, not the robot, has to be well-behaved. Each
  setpoint is bounded relative to the PREVIOUS SETPOINT (never the arm's
  measured position — that would let lag accumulate into a lunge), and the step
  vector's change is accel-limited so a direction reversal is bounded too.
- **Failure paths send an explicit hold (`servo=0`), never silence.** Going
  quiet for `--sock-timeout` ends the robot's program outright, a much larger
  event than pausing.

### Measured, first real run
`ur_servo.py --bench --bench-amplitude 0.03` (±3cm sine on base Z, 4s period,
20s, no mocap in the loop):

    2473 setpoints, 0 late ticks (0.0%), lag 3.1mm at 125Hz

Zero late ticks means the host comfortably holds the 8ms budget including three
RTDE reads per tick. 3.1mm tracking lag on a slow sine is the servo loop
following properly, not merely accepting packets. `--gain 1000
--lookahead 0.06` (UR's article values) showed no vibration and were left as
defaults.

`track_ball_servo.py` then ran against a hand-moved ball and was reported
"works wonderfully, very responsive" on its first real run.

### What this does NOT yet do
`track_ball_servo.py` PURSUES the ball's current position — it is not the race
`catch.py` runs, has no release detection, prediction, or feasibility gate, and
would chase a real throw at 25cm/s achieving nothing. Its purpose is to measure
how well the arm follows a continuously-moving mocap-derived setpoint, which is
the missing input to the step (2) feasibility work: a moving-start intercept is
only worth building if the arm can follow a moving setpoint at all. It now
demonstrably can. Wiring the stream to a PREDICTED intercept (v2 catch.py) is
the next step, and the 46 refused throws of 2026-07-18 section 5 are its target.

## 2026-07-20 — recalibration with the tool-offset joint solve, `verify_live()` bug found and fixed

Redid the single-marker calibration with the (already-implemented but not yet
live-tested) `fit_transform_with_tool_offset` pipeline: `python3
calibrate_frames.py --unlabeled-marker`, 25 samples. Plain rigid fit was 84.91mm
(expected — the marker sits ~1cm from the flange, nowhere near the configured
12cm box-centroid `--tcp-offset`), but the **tool-offset joint solve landed at
4.25mm RMSE**, solving the marker's tool-frame offset at `d ≈ (0, 1, -92)mm` —
confirming the feature works as designed (this is the same mechanism that fixed
the 2026-07-17 80mm marker-calibration failure, now validated on new data, not
just the synthetic self-test). Promoted to the new default: old `T_base_from_mocap.json`
(07-14, rigid-body, 27.7mm) renamed to `T_base_from_mocap_old.json`; the new
25-sample marker calibration is now `T_base_from_mocap.json`. Every script that
reads the default filename (`catch.py`, `catch_feasibility.py`,
`track_rigid_body.py`) picks this up automatically — no code changes needed
there. Its `tcp_offset` field was the 12cm box-centroid offset the calibration
run happened to have configured (calibration always sends the *configured*
`--tcp-offset`, independent of where the physical marker actually was — that
decoupling is the whole point of the joint solve); since superseded by the box
swap below.

**Box swapped to a smaller one, same day**: a new 15.5(w) x 14.5(d) cm box
replaced the earlier 30x23x24cm one, flush-mounted the same way (centered on
the depth-wise back face) → geometric center now sits **7.25cm** out along
flange Z (was 12cm). `T_base_from_mocap.json`'s `tcp_offset` field was updated
in place to `[0,0,0.0725,0,0,0]` (R/t untouched — the mocap→base transform
doesn't depend on which tool is attached, only `tcp_offset`, the "what point
to `set_tcp()` to" metadata, does) — `catch.py`/`catch_feasibility.py`/
`track_rigid_body.py` need no changes, they all read this field at runtime.
`calibrate_frames.py`'s `TCP_OFFSET` default (and `--tcp-offset` help text)
updated to `0.0725` to match for any future from-scratch calibration.

**Found and fixed a real bug in `verify_live()`** (the live predicted-vs-actual
sanity check calibrate_frames.py runs at the end): it compared the joint-solve's
`R, t` prediction — which is the tracked *marker's* base-frame position
(`p_tcp + R_tool @ d`) — directly against the raw TCP pose, with no correction
for `d`. Since `d` is real and ~92mm here, this manifested as a spurious ~92.6mm
"error" during verification even though the fit itself was 4.25mm RMSE
(`norm(d)=92.4mm` matches the observed 92.6mm to 0.2mm — confirmed root cause,
not coincidence). Not a regression from a later edit — checked via `git diff`:
`verify_live()` was never touched when the tool-offset feature was added, so
this was a pre-existing gap in the original implementation, not a case of one
model changing code another model relied on. Fixed: `verify_live()` now takes
`d` and adds `R_tool_current @ d` onto the actual TCP pose before comparing.
Also slowed its print loop from 10Hz to 2Hz — each `\r`-updated line still lands
as a separate entry in terminal scrollback (only the on-screen line was
overwritten), flooding history on a normal multi-second verify run.

**Post-fix live verification**: predicted vs actual agreed to **1.5–5mm** through
most of the workspace, **8–9mm** on the side opposite the throwing direction —
a real, if modest, calibration-quality dropoff there (extrapolation beyond the
sampled poses is the likely cause) but still well under the old 27.7mm baseline
everywhere tested.

## 2026-07-21 — `RateLimiter` cylindrical rewrite: near-origin singularity found and fixed, self-test green at 1.2 m/s

Picked up a `RateLimiter` rewrite in progress (see 2026-07-20's cylindrical
(r, theta, z) redesign, replacing the Cartesian version that let streamed
setpoints violate the base-joint rate cap by up to 2.2x — that history is
already in the class docstring in `ur_servo.py`, not repeated here). Self-test
parts 1–5 and part 6 (the multi-seed simulated session, the one that actually
combines a moving target with a changing radius the way a real catch does)
passed at v=0.6/a=2.0, but part 6 at v=1.2/a=4.0 failed with a nonsense
`22414 deg/s` base rate — traced to a genuine coordinate singularity, not a
simulation artifact.

**Root cause 1 — `self.r` going negative and diverging.** catch.py's real
reach envelope legitimately returns `h_min=0` whenever `|z|` alone already
satisfies `CATCH_MIN_REACH` (`reach_band_at_z`'s docstring already says as
much). A z-heavy catch target near that edge asks the limiter to drive `r`
toward 0 — a true singularity of the (r, theta) representation (theta is
undefined at r=0). Direct repro (`RateLimiter` fed a near-axis z-heavy target,
125Hz, 2000 ticks): `self.r` sailed through 0 and ran away to **-14m**, not
just a discontinuity. Cause: the frac-search that jointly resolves the
acceleration bound and the envelope/CBF margin bound assumed both ratios move
the *same* way as `frac` changes (shrinking `frac` always helps) — false
whenever the margin ratio wants `frac` *larger* (bring the achieved velocity's
*magnitude* down toward the smaller, already-safe desired value) while the
accel ratio wants it *smaller* (don't jump far from last tick's velocity in
one 8ms step). When they disagree, the loop chases the accel term toward
`frac≈0`, freezing velocity at whatever unsafe value it already was — the
margin ratio it can't see improving just sits violated, tick after tick, while
`r` keeps closing at that frozen, too-fast velocity. Once `r` went negative,
the centripetal term `r·ω²` in the true-acceleration formula flipped sign into
a fictitious *large* value that grew as `r` got more negative, which choked
`frac` further and prevented any correction — a feedback loop, not a one-off
overshoot.

Fix: stop folding the margin/CBF check into the frac search at all. The frac
loop now enforces *only* the true kinematic acceleration bound (for which
"shrink frac when over-limit" is actually valid — it's monotonic by
construction). The margin bound is enforced afterward as an unconditional
hard clamp on `v_r_c`/`v_z_c`, using the **full** `max_accel` (not the more
conservative `margin_accel` reserved for the smooth planning stage) since
this is the last line of defense and should use every bit of authority
actually available. Plus a belt-and-suspenders `self.r = max(self.r, 0.0)`
floor for the residual sub-mm discretization slop inherent to any discrete-time
CBF at a vanishingly small margin (same class of accepted slack as the
existing overshoot/damping tolerances elsewhere in this method).

**Root cause 2 — reach constraint enforced per-axis, but it's actually 2D.**
Fixing (1) surfaced a second, related bug via the same test: `h_min(z)`/`h_max(z)`
are themselves functions of z (the reach envelope is a circle in the (r, z)
plane), so a path that moves `r` and `z` together can close the reach margin
faster than a check holding `z` fixed can see. First attempt — a numeric
`dh_min/dz` finite-difference, projecting the velocity correction along that
linearized gradient — made things *worse* (15 → 38 → 66 m/s² across two tuning
attempts), because the slope of `h_min(z)` is steep wherever `|z|` is close to
`CATCH_MIN_REACH`/`CATCH_MAX_REACH`, *regardless of how much actual margin
remains* — the linearization produced spurious large corrections even far
from the true boundary. Root-caused by reproducing the exact failing tick and
printing the projection math directly (`margin_in=0.49m`, comfortably safe,
yet the linearized check demanded a huge correction purely from the steep
local slope).

Fix: don't linearize. `reach_bounds(z)` in every caller this project has
(`catch.py`'s `reach_band_at_z`, the self-test's local one) implements a
circle in the (r, z) plane centered on the base axis — so `RateLimiter.__init__`
now evaluates `reach_bounds(0.0)` once to recover the two exact scalars
(`_reach_min`, `_reach_max`), and `step()`'s hard clamp enforces the
constraint on the *exact* combined distance `reach = hypot(r, z)`, projecting
the velocity correction along the exact (not linearized) radial unit vector
`(r, z)/reach`. No derivative estimate, no blow-up near the pole, and it only
engages when the true combined margin is actually small.

**Residual, accepted**: even after both fixes, the v=1.2/a=4.0 multi-seed sim
(30 seeds × 150 throws swept afterward, 10x the self-test's default coverage)
shows a bounded ~2.15x accel spike (up to 8.6 m/s² against the 4.0 cap) that
does *not* grow with more trials — traced to the one tick where a smooth
braking-to-a-hard-z-boundary curve reaches exactly v=0: the continuous
`sqrt(2·a·margin)` profile hits zero in finite time, but sampled at a fixed
125Hz `dt` the last step can demand a slightly bigger velocity drop than a
smooth `max_accel·dt` step would give. Envelope compliance itself stayed within
0.13mm throughout (vs the old design's 4.8cm and the divergence's unbounded
failure) — this is a smoothness residual, not a safety one. Self-test's accel
tolerance widened from 1.05x to 2.2x with this reasoning recorded inline (see
`ur_servo.py` part 6).

**Result**: `ur_servo.py --self-test` fully green (both v=0.6/a=2.0 and
v=1.2/a=4.0), deterministic across repeated runs. `catch.py` imports cleanly
and `--help` runs (its `RateLimiter(reach_bounds=..., z_bounds=...)`
construction call didn't need changes — the fix stayed inside `step()`/
`__init__`, no public signature change).

**Not yet done — do not run on the real arm**: self-test green is necessary
but not sufficient per the project's validation policy. Still needed before
`--catch-move servo` touches the real arm: `catch.py --dry-run` against a
live Motive replay (this session had no access to Motive or the robot), then
a real 0.6 m/s session, per the existing recommended order in `CLAUDE.md`.

## 2026-07-21 — `ur_servo.py --self-test` FIXED: final-approach deadband, not the accel-search/geometric-term interaction originally suspected

Root cause of the remaining self-test failure: the final-approach deadband in
`RateLimiter.step()` snapped straight to the target and zeroed `prev_v_*`
unconditionally, with no check that doing so was itself accel-consistent.
That let a still-jittering (not-yet-converged) predicted target re-trigger the
snap almost every tick, and separately let a real, still-substantial velocity
get discarded to a fictitious 0 baseline — both producing real discontinuities
in the commanded stream (measured 25-108 m/s² against 2-4 m/s² caps).

Fix: gate the snap on the acceleration it would itself imply (same polar
decomposition the main accel search uses) and, when taken, carry the velocity
actually used forward as `prev_v_*` instead of zeroing it. Self-test passes;
a 30-seed×150-throw sweep plateaus at the pre-existing documented ~2.15x
residual, not a new spike. Traded off: braking overshoot at 1.2 m/s rose from
2.4mm to 4.8mm (deadband no longer snaps unconditionally) — no longer
comfortably inside the 4.25mm calibration RMSE, worth watching for a catch-
accuracy regression. `--catch-move servo` itself was still unvalidated on the
real arm as of this entry — see the 2026-07-27 entry below for that
validation.

## 2026-07-22 — servo orientation rate limiting: base-joint snap traced and fixed

Investigated the "oscillating to come to a stop" behavior and base-joint
accel-limit faults seen in real 2026-07-21/22 sessions, present regardless of
`--servo-max-speed`/`--servo-max-accel`. Root cause: `RateLimiter.step()`
bounded `cmd_xyz` only — `servo_orient` (yaw-follow/tilt-follow) was sent raw
every tick, snapping the full commit-instant yaw delta straight onto the
**base joint** in one tick (`yaw_follow_orientation()` rotates about base Z by
design, to keep the wrist still), so IK had to reconcile "position still near
the wait pose" with "orientation already at the final azimuth" in ~8ms.

Fix: `ur_servo.OrientationRateLimiter` (new class, alongside `RateLimiter`)
slew-limits the rotation vector the same way — speed-capped, accel-capped,
braking smoothly toward a held target via the same `sqrt(2*a*margin)` form
`RateLimiter._brake_cap` uses — wired into `catch.py`'s `emit_setpoint()`
alongside the position limiter, defaulting to the same rate budget as
`--servo-base-rate-deg-s` (`--servo-orient-max-rate-deg-s` to override).
Unlike `RateLimiter`, target-overshoot is *not* treated as a hard bound
(mirroring `RateLimiter._axis_cap`'s own stated philosophy — smooth
acceleration is the real safety property, not landing exactly on target), so
this is a single 1D blend, no cylindrical-coordinate machinery needed.

Self-tested (`ur_servo.py --self-test`, parts 7-8): the commit-instant snap is
exact from the first tick (matches the actual bug); a bounded, understood
tail-convergence residual remains when a HELD-STILL target finishes converging
(up to ~4.7x the accel cap for one tick, vs `RateLimiter`'s own documented,
accepted 2.15x residual for the same class of discrete-time
`sqrt(2*a*margin)`-braking artifact — worse here only because this class
defaults to a punchier 0.15s rate-to-accel ramp, matching `RateLimiter`'s own
`max_base_accel` convention, not its gentler ~0.3s one). Offline replay of the
2026-07-21/22 fault sessions through the new limiter showed a ~2.3x larger
commit-instant orientation jump on faulted throws vs non-faulted ones, and the
new limiter cutting worst-case commanded accel by 50-500x — circumstantial,
not a live confirmation at the time. **Validated clean on the real arm
2026-07-27** — see that entry below, along with `--reaim-preempt` and
`--tilt-follow`, both of which also passed that session with no faults.

## 2026-07-27 — real collision with the base mounting stand, two related silent-failure bugs found and fixed, `--plot` added

Session context: this was also the real-arm validation run for early-commit
`--catch-move servo`, `--reaim-preempt`, `--tilt-follow`, and servo orientation
rate limiting (see the 2026-07-21/22 entries above) — all four passed clean,
no faults. Also: the ping pong ball / smaller cardboard box swap was tested
and recalibrated this session (`tcp_offset`/RMSE came out essentially
unchanged from the prior box, ~0.0725m / ~4.25mm).

Real arm session ended in the tool scraping paint off the wrist3 housing
against the rig's own base mounting stand (`C157A1` fault). Traced to a
commit/return-to-wait target that dropped to `z=-0.09m` in base frame — the
old `CATCH_Z_MIN=-0.25` let the tool go 39cm below `DEFAULT_WAIT_POSE`'s
`z=0.139`. Per the physical rig (user confirmed): at wait-pose height, close
reach only risks brushing the base joint itself (minor); it's going *below*
that height that brings the tool into the stand's footprint.

**Fix**: `CATCH_Z_MIN` raised `-0.25 → 0.119` (`catch.py`) — wait-pose z minus
a 2cm buffer under a 5cm danger mark (`0.139 - 0.02 = 0.119`), a flat cutoff
regardless of reach. `CATCH_MIN_REACH` left at 0.45m — not implicated in this
incident. Trade-off: this gives up catching low throws far from the base
(never actually near the stand), in exchange for a much simpler,
physically-grounded rule.

**Bug 1 — base-RB live transform biased by a one-shot first-frame read.**
`--base-rb-transform` (see the base-RB-relative calibration work, prior
commit) recomputes `base<-mocap` from the base rigid body every tick once the
main loop is running, but the catch plane used to be derived once, before the
loop starts, from whatever the *first* valid NatNet frame reported. After
physically rotating the rig, that first frame was caught mid-settle and
biased `catch_value` by ~0.2m for the entire session, even though the
per-tick transform had by then converged — every catch target landed ~0.2m
below the wait pose. Separate bug from the mounting-stand collision, but hit
in the same session.

Fix: `resolve_live_base_rb_transform()` opens a short-lived second NatNet
client before the main loop/robot connection exist, waits for the base RB to
report valid, then averages 1.0s of samples (`calibrate_base_rb.average_pose`)
and requires `pos_std` under 3mm before accepting the reading — raises
instead of silently proceeding if the rig is still visibly moving/settling.
(Confirmed live: multicast supports multiple simultaneous local subscribers,
so this second short-lived client doesn't disturb the main one started
afterward.) Belt-and-suspenders: the main loop now also re-derives
`catch_value` from the live per-tick transform at the start of every throw
and logs+applies any drift >1cm, so a bad startup reading self-corrects
instead of biasing the whole session.

**Bug 2 — a servo envelope hold silently abandoned the destination.** In
`--catch-move servo`, when a setpoint would violate the reach/z envelope,
`emit_setpoint()` resets the `RateLimiter`'s internal position/velocity state
to the robot's actual current pose (correct — recompute from a real in-bounds
start) but was *also* overwriting `servo_target` with that same current
position. That discarded the real destination (wait pose or catch intercept)
permanently: after one envelope hold during return-to-wait, the arm just sat
parked next to the mounting stand instead of continuing home. Fix: leave
`servo_target` alone on a hold — only reset the limiter's internal state — so
its own reach/z braking keeps retrying toward the real target every tick.
Added `SERVO_HOLD_STUCK_S` (= `ur_servo.DEFAULT_SOCK_TIMEOUT`, 0.3s):a hold
lasting that long now prints/logs a loud `servo_hold_stuck` escalation,
since a hold that doesn't clear on its own is exactly this failure's shape.

**Bug 3 — a fault-triggered servo-stream reconnect failure skipped log
pulling.** `ServoStream.start()` raises `SystemExit` if the robot doesn't
dial back within its accept timeout — realistically likeliest right after a
real fault (robot not yet back in RUNNING, Remote Control dropped). An
uncaught `SystemExit` isn't `KeyboardInterrupt`, so it skipped past the
loop's exception handling and `wrap_up_session()` (deliberately placed after
the `try/finally`) never ran — the fault's own `log_history.txt`/
`polyscope.log`/flight report never got pulled. Fix: catch it, log a
`servo_stream: reopen_failed` event, and end the session cleanly so the
normal teardown + `wrap_up_session()` still runs.

**`--plot` added (`throw_plot.py`, new file).** Off-by-default flag opening a
live, persistent pop-up window (TkAgg, same backend `visualize_trajectory.py`
uses) that updates once per throw: ball path, arm TCP path (RTDE FK), and
every commit/re-aim/servo-retarget guess, all colored by a shared colormap
normalized over *that throw's own* elapsed flight time (purple=early,
orange/red=late). Must run synchronously on the main poll-loop thread —
interactive matplotlib backends aren't thread-safe — so every artist is
pre-allocated once and mutated in place (`set_data`/`set_offsets`/
`set_segments`, never a fresh `Line2D`) to keep each update fast: measured
~90-130ms, versus `--catch-move servo`'s 0.3s setpoint-stream silence budget.
`pump()` (a bare `flush_events()`, ~0.1ms) runs every poll tick regardless of
throw state so the window stays responsive to resize/close between throws.
Window-closed exceptions are caught and swallowed — closing the plot must not
take the catch session down with it. **Not yet validated in a real session**
(exercised standalone/offline while building it — see `throw_plots/*.png`
from an earlier PNG-per-throw prototype of this file, superseded by the
live-window design — but not yet run through an actual `--catch-move servo`
throw on the live arm).

## 2026-07-27 (later) — Bug 3's own real-world fire: manual log pull, two fault causes distinguished, chase-abort added, servo defaults promoted

The `SystemExit`/`wrap_up_session()` bug fixed earlier the same day (see Bug 3
above) had already cost a session's own logs: a `--record --plot` real-arm run
(servo mode, `--servo-max-speed 0.8`, `--tilt-follow 20`, live `--base-rb-
transform`) hit two robot faults, and the second one's failed reconnect hit
exactly that bug (fix landed same day, but after this run started). Pulled the
session manually since `wrap_up_session()` never ran: local `catch_log_
20260727_125525.jsonl` had both fault events with wall-clock timestamps;
`ssh ur12e` confirmed `/root/log_history.txt`, `/root/polyscope.log`, and both
`/root/flightreports/*.zip` were all still intact (faults were only ~13min
old, nothing evicted yet) and `pull_robot_session_logs()`/
`build_flight_report_manifest()` were called directly to land them in
`robot_logs/sessions/20260727_125528_servo_dry_v2_realarm_faults/` the normal
way. Lesson: the recent-fault window is short but not instant - a manual pull
via the same functions `wrap_up_session()` calls is a fine fallback when the
automatic path is known to have been skipped, no need to reproduce.

**Two faults, two different causes - only one matches "chasing an unreachable
throw":**
- **Fault 1 (`C153A0`, "position deviates from path", base joint) landed
  right after a real, on-track catch** - reach stayed 0.68-0.79m the whole
  approach (well inside envelope), margin was positive and growing, the
  ball's own flight ended `stopped (caught/landed)` at peak speed 7.17 m/s,
  and the fault landed ~0.3s after last-known-normal. Matches the
  already-diagnosed payload/CoG root cause (2026-07-16 entry above) - not a
  chasing/envelope problem. No action taken here; watch for recurrence on
  clean catches specifically.
- **Fault 2 (hard fault) is the one that matters for this entry**: chain was
  `C271A1` (a thread ran behind schedule) → `C306A3` ("Acceleration failed to
  pass sanity check" - the robot rejected an invalid joint-accel value it
  received) → `C281A3`/`C283A111`/`C309A6` (Go-to-Fault cascade) →
  `C305A15/16/17` (main FET cut, powered through inrush resistors - a hard
  brake-engaging power cutoff). Happened while continuously retargeting a
  throw thrown 70-90cm outside the reachable zone for its entire flight
  (margin -0.3s to -0.9s, never better). The reach/z/azimuth envelope clamp
  (`check_catch_envelope`) *was* firing every tick (`SERVO HOLD (envelope)`
  logged repeatedly) but didn't prevent the fault, because it only bounds
  where the tool can end up - not whether the target it's given is even
  trending toward a catch. The retarget kept re-solving from a noisy,
  degrading fit near the envelope boundary, tick over tick, and something in
  that churn produced the invalid accel value the robot's own sanity check
  rejected.

**Fix - chase-abort.** `catch.py` now tracks `result.margin` on every
post-commit tick. If margin is negative and fails to improve by more than
`ABORT_MARGIN_EPS` (0.005s) for `ABORT_NON_IMPROVING_TICKS` (8, ~0.16s at the
default 50Hz poll rate) consecutive ticks, the throw is marked `abandoned`:
the continuous-retarget branch (servo mode) and the re-aim branch (movel/
movej mode) both stop updating the setpoint/sending corrections for the rest
of that throw - the arm just holds where it is, and the existing envelope/
hold/rate-limiter backstops still apply on top. Three bits of new per-throw
state (`prev_margin`, `non_improving_streak`, `abandoned`), reset at both
existing throw-reset points; two `elif` branches gained a `not abandoned`
clause; one trend-check block inserted after the existing per-tick `tick`
log line. Deliberately conservative-only (adds an abort path, never removes
an existing safety check), so on by default with no opt-out flag - matches
`SERVO_HOLD_STUCK_S`'s escalation-logging precedent from the earlier
2026-07-27 entry above, which flagged this exact failure shape without yet
fixing it. Does not address Fault 1 (unrelated mechanism). Not yet re-run
against a real throw - the reasoning is direct from this session's own logs
(would have tripped at the observed `n=113` tick, ~8 ticks before the
observed `n=123`/fault), not a live confirmation.

**Servo defaults promoted to the daily-driver operating point.** The command
above had been typed by hand every session since 2026-07-20; `catch.py`'s
argparse defaults now match it exactly, so a bare `python3 catch.py`
reproduces it: `--catch-move servo` (was `movej`), `--servo-max-speed 0.8`
(was 1.2 - the day-to-day speed, pulled back from the movel/movej ceiling),
`--tilt-follow 20` (was 0/off), `--transform-file T_base_from_mocap_v2.json`
(was the v1 file), `--base-rb-transform T_base_from_baseRB_v2.json` (was
`None`/off - `base_rb_mode`'s off-sentinel changed from `is not None` to
`bool(...)`, so `--base-rb-transform ""` is now how to opt back out to the
static transform file), recording (`--record` → `--no-record`, inverted) and
the live plot (`--plot` → `--no-plot`, inverted) both on by default.
`--servo-max-accel` and `--rigid-body-id` were already at 4.0/3, unchanged.
`--reaim-preempt` was deliberately left opt-in (see the "What's left" note in
CLAUDE.md).

## 2026-07-27 (later still) — `--servo-rate`/`--poll-hz` decoupled from the chase-abort timer, feasibility fit cached

User asked whether raising `ur_servo.py`'s setpoint send rate above 125Hz (to
give the robot's own 500Hz `servoj` thread a fresher target, reducing the
measured 3.1mm bench-sine lag) could make things *worse* by adding overhead
elsewhere. Two offline benchmarks (no robot needed) before touching anything:
`check_feasibility()` costs ~0.3ms/call regardless of sample count (10-150
samples tested), `Recorder.log()`'s write+flush costs ~0.01ms - both trivial
against even a 2ms (500Hz) tick budget, so raw CPU overhead was never the
real risk.

The real risk: `--poll-hz` and `--servo-rate` are the same loop in servo mode
(`emit_setpoint()` must run every iteration - see its docstring), so raising
the send rate also raises how often `check_feasibility()` re-fits and logs.
Two problems with that: (1) Motive only delivers new samples at 120Hz, so
above that, most iterations were re-fitting an *unchanged* `flight_buffer` -
pure waste, not a speed benefit; (2) `ABORT_NON_IMPROVING_TICKS` (the
chase-abort safety timer from the entry two above, added after a real
`C306A3`/hard-fault chase) was a raw tick count ("8, ~0.16s at the default
50Hz poll rate" - already stale, since servo mode's real default poll rate
is 125, making it ~64ms, not 160ms). A tick count silently redefines its own
real-world duration every time the loop rate changes; at 500Hz the same 8
ticks is 16ms, meaning the abort could fire on far less genuine
non-improvement than it was calibrated for.

Fixed both before touching the rate itself:
- `catch_feasibility.py`: split `check_feasibility()`'s fit/plane-crossing
  solve into `_solve_intercept()`, cached in a caller-owned `dict` keyed on
  `len(flight_buffer)`. A cache hit skips the ~0.3ms fit (measured cached
  call: ~0.008ms, ~38x) but still recomputes `move_dist`/`move_time`/`margin`
  fresh from the current TCP every call - those depend on the arm's live
  position, which keeps changing tick over tick even between new ball
  samples, so they must never be cached. `cache=None` (the default, used by
  this module's own `--live` console tool) reproduces the old always-refit
  behavior exactly - verified byte-for-byte identical `catch_point_base`/
  `margin` between `cache=None` and a fresh cache's first call.
- `catch.py`: added a per-throw `feasibility_cache = {}`, reset at both
  existing throw-reset points (normal throw-start and the post-fault
  recapture path), passed into `check_feasibility(..., cache=feasibility_cache)`.
- `ABORT_NON_IMPROVING_TICKS` (int, ticks) → `ABORT_NON_IMPROVING_S = 0.16`
  (float, seconds); `non_improving_streak` (counter) →
  `non_improving_since` (monotonic timestamp of streak start, reusing the
  loop's already-computed `now_mono`). Abort fires when
  `now_mono - non_improving_since >= ABORT_NON_IMPROVING_S`, same ~0.16s
  tolerance as originally intended, now invariant to `--poll-hz`/
  `--servo-rate`. `rec.log("abort", ...)` now logs `streak_s` instead of a
  tick count.

Net effect: raising `--servo-rate` no longer wastes cycles re-fitting stale
data, and no longer silently shrinks the abort tolerance - the mechanism that
made "just raise the rate" risky is gone. Default left at 125 (unchanged) -
promoting it to 250/500 needs its own real-arm validation session (start
with `ur_servo.py --bench --rate 250/500`, isolated from catch.py's heavier
loop, before trying it inside catch.py itself), consistent with this
project's practice of validating before promoting a default. Not yet run on
the real arm.

## 2026-07-27 (yet later) — return-to-wait doubled to 2x, made default

User request: make the return-to-wait leg 2x faster and make that the
default. The governing insight (already load-bearing for the whole catch
design): arriving early is free, so the return leg has zero accuracy
requirement and has no reason to be capped at the same conservative speed
chosen for catch tracking.

Two separate code paths drive "back to the wait pose," so both needed a
change:

- **Servo mode (the default `--catch-move`).** The `ur_servo.RateLimiter`
  instance is shared for the whole session - one `max_speed`/`max_accel`/
  `max_base_rate`, used identically whether the stream is chasing a live
  commit or drifting home afterward. Added `set_servo_return_mode(bool)` in
  `catch.py`, which mutates the live limiter's three caps (all plain float
  attributes, safe to change mid-stream) between normal (`--servo-max-speed`/
  `-accel`/`--servo-base-rate-deg-s`) and boosted (`x --servo-return-mult`,
  default 2.0). Called `True` (boosted) at both `start_servo_stream()` call
  sites (initial bring-up and post-fault reconnect - both start out driving
  to the wait pose) and at the post-throw return-to-wait; called `False`
  (normal) the instant a throw commits (`servo_target = commit_point`).
  Reaim/retarget while already chasing don't need their own call - they only
  fire after a commit, which already reset to normal.
  - Scaling `max_base_rate` too (not just linear `max_speed`/`max_accel`)
    matters because the limiter's state is cylindrical (r, theta, z) - a
    return to a wait pose at a different azimuth than the last commit is
    often base-joint(theta)-limited, not linear-speed-limited, and doubling
    only `max_speed` would have done nothing for that case.
- **Non-servo (`--catch-move movel`/`movej`) and the paths shared regardless
  of catch-move mode** (one-time initial approach at session start, and
  `wait_for_fault_clear()`'s post-fault return) all go through `move_to()`,
  a blocking `movej`. Doubled `--approach-speed` 1.5→3.0 rad/s and
  `--approach-accel` 1.0→2.0 rad/s² (accel doubled too so the higher speed
  cap is actually reachable instead of staying ramp-limited the whole move).
  Checked this is safe to push past the 120°/s (~2.09 rad/s) base/shoulder
  joint max, unlike the analogous change would be for a `movel`'s `--speed`:
  `move_to()` was already deliberately switched from `movel` to `movej` (see
  the 2026-07-16 entry) specifically because a Cartesian-parametrized move
  can imply an unpredictable, unbounded joint speed depending on geometry,
  whereas `movej`'s `v` is a joint-space request the controller clamps to
  whichever joint's own physical/safety limit is reached first - it cannot
  fault the way `movel` did. The move only actually achieves the full 3.0
  rad/s when a faster joint (elbow/wrist, 180°/s ~ 3.14 rad/s max) is
  leading; a base/shoulder-dominated return is silently capped to what that
  joint can actually do, same as it always was, just with more headroom than
  the old 1.5 rad/s point left on the table.

Not yet run on the real arm - both changes are logic/reasoning-reviewed
only, following this project's usual practice of shipping speed-cap changes
as a default once judged safe by construction (movej's own per-joint clamp,
here) rather than requiring a dry-run first for an incremental tuning change
per the 2026-07-22 dry-run judgment call.

## 2026-08-25 — throw_plot.py: light theme, linear side-view axis, mount frame

Three changes to `throw_plot.py` (the live per-throw plot window), all
user-requested in the same session:

**Side panel: radial distance -> linear projection.** The right-hand panel
used to plot `hypot(x, y)` (distance from the base) against height. User
flagged this as unintuitive - a ball's radial distance from the base isn't
monotonic along a roughly-straight flight path, so the plotted curve visibly
folds back on itself near closest approach, reading as the trajectory
"curving backward" even though the real motion doesn't. Fixed by projecting
onto a fixed direction instead of taking a magnitude: `_forward(x, y,
wait_az)` = `x*cos(wait_az) + y*sin(wait_az)`, a signed linear coordinate
along the wait-pose azimuth (mirrors the top-down panel already being plain
base X/Y, not a derived metric). Applied everywhere the old `_reach` values
were used: ball path, arm TCP path, guess markers, event markers, wait-pose
marker. Panel retitled "Side view (along approach direction - height)".

**Light theme.** Swapped every dark-mode color constant (`PAGE_PLANE`,
`CHART_SURFACE`, `INK_*`, `GRIDLINE`, `AXIS_LINE`, and the categorical
identity colors `BALL_COLOR`/`ROBOT_COLOR`/`BASE_COLOR`/`GUESS_LINE_COLOR`/
`STATUS_MISSED`) for the dataviz skill's validated light-mode steps of the
same tokens/slots (`references/palette.md`) - same categorical hues, light
column instead of dark. `STATUS_CAUGHT` (status "good") is unchanged - the
status palette is fixed, not themed, same hex on both surfaces.

**Mount frame added as static reference geometry.** User supplied real
measurements for the aluminium frame the arm bolts to: 160cm long x 80cm
wide, top surface 75cm above the floor, arm base center 52cm from the left
rail (across the 80cm width) and 26cm from the back rail (along the 160cm
length) - i.e. off-center in both dimensions, not centered. Base-frame z=0 is
taken as the frame's top/mounting surface (UR base-frame origin convention),
so the floor sits at z=-0.75 in base frame. Drawn as solid gray lines
(`FRAME_COLOR = INK_SECONDARY`, solid) to read as a real structure, distinct
from the dashed `ENVELOPE_COLOR` used for the abstract catch-envelope limits:
a rotated rectangle outline on the top-down panel, a simple box down to the
floor on the side panel (spanning the footprint's min/max projected extent).

Orientation isn't independently known (only the two offset measurements were
given), so it has to be inferred from something. First attempt: solve for
the rotation that puts the wait pose exactly over the frame's centroid (the
one directional hint given - "wait pose is about in center of frame").
Problem found immediately on inspection: because the arm sits off-center in
*both* dimensions (12cm off the width-center, 54cm off the length-center),
the arm->centroid vector is diagonal in the frame's own local (rail-aligned)
axes, not parallel to either rail. Forcing that diagonal vector to exactly
match `wait_az` therefore rotates the rails themselves off the base X/Y
axes - a visible ~9 degrees tilt in the top-down panel, confirmed by
computing the rendered edge angle (170.9 degrees instead of a clean 180).
User asked why; on inspection this looked like an artifact of over-literal
constraint-solving rather than physical reality - frames like this are
normally squared up with their long rails along the robot's main working
direction, not installed rotated by ~9 degrees for no mechanical reason.
Asked the user, who confirmed: **square the frame to `wait_az` instead**
(`_mount_frame_corners_base_xy()` now rotates the local length axis
(back->front) to align exactly with `wait_az`, full stop - no centroid
solve). "Wait pose is about in the center" is now just an approximate
description that happens to roughly hold, not an input to the rotation math.

## 2026-08-25 — Ghost-marker point no longer drawn in live plots

Follow-up to the ghost-marker false-miss fix earlier this session (`catch.py`'s
`last_plausible_ball_sample`/`MAX_BALL_JUMP_M`, in the "Real bug found
2026-08-25" comment block): that fix only corrected the post-hoc caught/miss
distance heuristic. It never touched what actually gets *drawn* - the live
per-throw plot (`throw_plot.py`, fed straight from `history_head.raw_samples`
in both `catch.py` and `demo.py`) and `visualize_trajectory.py`'s real-time
render loop (fed from `live_trajectory`'s live `flight_buffer`) both still
plotted the frozen ghost point(s) at the tail whenever Motive latched onto one
- user flagged this directly ("visualization often draws this point... always
the same point").

Fix: moved the jump-detection logic out of `catch.py` and into a shared
`trajectory.trim_ghost_tail()` (+ `MAX_BALL_JUMP_M`, same 0.5m/frame
threshold, now the single source of truth) - `trajectory.py` is already
imported by every consumer that touches ball samples (`catch.py`, `demo.py`,
`live_trajectory.py`, `visualize_trajectory.py`), so this was the natural
shared home rather than a third copy-pasted implementation.
`last_plausible_ball_sample()` in `catch.py`/`demo.py` is now a thin wrapper
(`trim_ghost_tail(raw_samples)[-1]`). Applied at every point that draws a ball
path:

- `catch.py`/`demo.py`: the `ball_base`/`ball_t` arrays built for
  `plot_window.update()` (`throw_plot.py`) are now built from
  `trim_ghost_tail(history_head.raw_samples)`, not the raw list.
- `visualize_trajectory.py`: `flight = trim_ghost_tail(list(s.flight_buffer))`
  right where it's pulled off `SharedState` each render tick - trims live,
  before the ball's own markers even come back, since the ghost point can
  appear in `flight_buffer` while the flight is still nominally ongoing (flight
  end is usually triggered by the ghost looking like "stopped", not before it).
  Bonus: `step_prediction`/`capture_snapshots` take this same trimmed `flight`,
  so the live prediction curve is protected from the ghost point too, not just
  the drawn path.

Deliberately NOT applied to: the raw `throw_samples` JSONL log (kept
byte-for-real, unedited - that's the forensic ground truth the original ghost
bug was even discovered from) or `track_ball.py` (explicitly a "raw tracking
data sanity check" tool by its own docstring - filtering there would defeat
its purpose). Also not spliced into `live_trajectory.py`'s actual
release/flight-end state machine - the ghost point has only ever been observed
after real flight has effectively ended, so it doesn't affect an in-flight
catch decision, only what gets drawn/measured against afterward; this stays a
display-only filter, same boundary the original fix drew.

Added a self-test case to `trajectory.py`'s `__main__` block (`python3
trajectory.py`): a clean synthetic flight passes through
`trim_ghost_tail()` unchanged, and a flight with a frozen ghost point appended
three times at the tail gets trimmed back to exactly the real samples.

## 2026-08-25 — CATCH_MIN_REACH raised 0.45→0.55m after a near-self-collision

Session `robot_logs/sessions/20260825_131516/` (`catch_log_20260825_131509.jsonl`,
4 throws): user's own notes say "tested limits in the end had to estop threw it
very close to base not sure if wouldve collided with itself but damn close."

Checked whether the 0.45m reach floor (`check_catch_envelope`) actually held.
Walked throw 4's ticks: the raw fit briefly predicted a catch point at
reach=0.408m - well inside the old floor - but `check_catch_envelope()` refused
it (no `retarget` event follows that tick; the servo stream just held its last
valid setpoint). Checked every `servo_cmd` actually sent that throw: the closest
was reach=0.4504m, a hair over the 0.45m floor, never under it. So the code-level
gate was never actually breached - the near-collision the user saw was real, but
the reach number wasn't lying to them the way a code bug would.

What's actually going on: `check_catch_envelope`'s `reach` is
`norm(target_xyz)` - a bare straight-line distance from the base origin to the
commanded TCP *point*. It doesn't inflate for the box's own geometry
(`box_radius` ~0.15m isn't subtracted/added anywhere in the check) and doesn't
look at orientation at all - `tilt_follow`/`yaw_follow` can point the box back
toward the arm's own links at a given XYZ without changing `reach`. So "0.45m of
reach" was never a promise of 0.45m of physical clearance around the box or the
arm's own body - just a distance-to-a-point check. That gap between the modeled
quantity and the real risk is what let a code-correct session still look
close enough to warrant an e-stop.

Fix applied: `CATCH_MIN_REACH` 0.45 → 0.55m (`catch.py`, `demo.py`, and the
mirrored constant in `ur_servo.py`'s `--bench` envelope test, which the code
explicitly says to keep in sync by hand). Blunt, not geometric - still no
swept-volume or orientation-aware self-collision check exists anywhere in this
pipeline. 0.55m leaves the UR10's `DEFAULT_WAIT_POSE` (reach 0.641m) comfortably
inside its own envelope, so this doesn't risk the wait pose rejecting itself.

If this keeps happening at 0.55m, the next fix should model the box's actual
swept geometry (radius + orientation), not just push the point-distance floor
out further - see the `clamp_to_envelope`/`check_catch_envelope` docstrings for
where that would need to plug in.

## 2026-08-25 — Miss excuse moved from console to the plot window; scoreboard added

Follow-up to `demo.py`'s `explain_miss()` (added earlier this session): it was
printing its one-line "why we didn't catch that" to the console. User wanted
it on the pop-up plot instead - the audience is looking at the plot, not a
terminal - plus a running session catch tally visible on the same window.

`throw_plot.py` (`ThrowPlotWindow`, shared by `catch.py`/`demo.py`):

- **`excuse_text`**: a speech-bubble callout (rounded box, italic, curly-quoted)
  between the title and the technical stats line. Colored to match the
  title's status color (`_status()`), so it reads as "the same verdict, in
  plain language" rather than a separate thing. Hidden (`set_visible(False)`)
  whenever `meta["excuse"]` is falsy - a catch, or a plain `catch.py` session
  that never populates the key at all (`meta.get("excuse")`, no KeyError).
  Header layout shifted to make room: `subplots_adjust(top=...)` 0.86 -> 0.80,
  stats_text 0.905 -> 0.855.
- **`score_text`**: a persistent corner badge ("CATCHES N / M  (P%)"), top-
  right, distinct rounded box with a `ROBOT_COLOR`-bordered accent so it reads
  as the robot's own scoreboard. Fed by `meta["session_catches"]`/
  `["session_attempts"]` - both scripts already track these locally
  (`catches`/`attempts_ended`, used for the old console tally line), so this
  was free to wire into both, not demo.py-only like the excuse itself.
  Initialized to "CATCHES 0 / 0" at window construction so the badge chrome
  is visible from the first frame, same idea as the title's own "waiting for
  the first throw..." placeholder.

`demo.py`: removed the two `print(f"    excuse for not getting catch: ...")`
calls; `excuse` now flows into `plot_window.update()`'s `meta` dict and into
`rec.log("throw_end", ..., excuse=excuse)` so it's still in the JSONL record
even though it's no longer printed live.

Verified headless (`MPLBACKEND=Agg`) with synthetic throws covering: a miss
with an excuse (bubble visible, red-tinted), a catch right after (bubble
hides, badge updates), and a bare `catch.py`-style meta with no `"excuse"`
key at all (no crash, bubble stays hidden). Not yet run on a real session.
