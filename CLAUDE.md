# Project: RoboBaseball

For the detailed history behind any decision below (bugs found, experiments run, dead
ends) see `docs/debug_log.md` - it's not auto-loaded, so consult it deliberately when
you need the full story rather than duplicating it here. Keep this file to current,
load-bearing state only.

## Git

**Never `git push` unless explicitly instructed in that conversation** — a prior
commit approval does not carry forward to later ones. **Never add Claude as
co-author on commits in this repo** (no `Co-Authored-By: Claude` line) — user
directive, 2026-07-16.

## Concept

A demo showing Universal Robots + OptiTrack working together in real time: humans throw
objects (starting with tennis balls) at a UR12e arm, and the arm catches them. Goal is
maximum "wow" factor as a technology demo, not industrial usefulness.

Working pipeline sketch:
1. OptiTrack Flex 13 cameras track a thrown ball's 3D position at high frame rate.
2. Software fits/predicts the ball's trajectory (projectile motion, drag optional) and
   extrapolates a catch point + time.
3. A controller streams a target pose to the UR12e (via URScript/RTDE) fast enough for the
   arm to move an end effector (net, cup, or soft gripper) into the intercept point before
   impact.
4. Bonus: OptiTrack could also track the arm/end-effector itself (rigid body markers) for
   closed-loop correction instead of trusting UR forward kinematics alone.

This is not yet a settled architecture — treat the above as a first-pass idea to be
revised as we prototype.

## Hardware Reference

### Universal Robots UR12e (arm)
- Rebrand of the UR10e (2024) — same hardware, renamed to reflect true payload. Any
  UR10e docs/URCaps/accessories apply unchanged.
- Payload: 12.5 kg, Reach: 1300 mm, Repeatability: ±0.05 mm, 6 DOF
- Joint speed (official UR10e manual, SW5.22): base & shoulder max **120°/s**, elbow
  & all three wrists max **180°/s**. (An earlier version of this file said
  180°/360°/s — that was wrong, likely from a UR10-non-e or marketing sheet.)
- **TCP speed: no fixed cap — it's joint-speed-limited, configuration-dependent,
  and varies by pose/direction** (not a single constant). The datasheet's "Tool:
  approx. 1 m/s" is nominal, not a ceiling; the separate "4 m/s" figure is
  unloaded/freedrive capability, not achievable via commanded moves. Measured
  peak TCP speed (`speed_char.py`, reads true peak off `getActualTCPSpeed()`,
  not leg-average): commanded speed delivered **1:1 up to ~1.2 m/s**, peak
  achieved **~1.32 m/s** in the best configuration. Use `speed_char.py` for
  peak speed; use `ur_loop.py`/leg timing for the ramp-inclusive *move times*
  that actually gate catch feasibility (peak ≠ average — a leg can have a
  1.3 m/s peak but a 0.74 m/s timed average). Accel: no published hard
  ceiling; `speed_char.py` pins `--accel 40` so accel never bottlenecks —
  pushing `--accel` past ~2-4x commanded speed has diminishing returns. Full
  measurement + reconciliation with the datasheet: `docs/debug_log.md` 2026-07-14.
- Footprint: Ø190 mm base, weighs 33.5 kg, IP54, PLd Category 3 safety functions
- Control: URScript, RTDE for external streaming control, URCaps for extensions
- **Catching implication**: TCP speed (~1.2 m/s reliably commanded, ~1.3 peak) is still
  the binding constraint — a tennis ball thrown at any real pace covers the workspace far
  faster than the arm can reposition. ~30% more reach-rate than the earlier ~1 m/s budget,
  but the story is unchanged: throw speed/distance limits, minimized end-to-end reaction
  latency, and/or a forgiving catch tool (net/funnel) over pinpoint placement. What
  actually gates feasibility is ramp-inclusive *move time* over a distance, not peak speed.

### OptiTrack Flex 13 (motion capture camera)
- Resolution 1280×1024, global shutter, 30–120 FPS adjustable, 8.3 ms system latency,
  56° FOV, sub-mm marker accuracy (~±0.24mm single marker, ~±0.05mm calibrated rigid body)
- USB 2.0 (data/sync/power), Motive software (NatNet SDK for streaming)
- **Catching implication**: OptiTrack (120 FPS, ~8ms latency) is not expected to be the
  bottleneck — robot reaction/motion time is. Multiple Flex 13 units needed (min. 3,
  realistically more) for reliable 3D triangulation across throw + catch zone.

## Development Environment

Motive runs on a separate Windows 10 PC, connected to this Ubuntu laptop via direct
Ethernet (no router/switch), static IPs:
- Windows 10 PC (Motive): `192.168.10.1`
- Ubuntu laptop (interface `enp0s31f6`): `192.168.10.2`

**Confirmed working NatNet config**: Motive → Streaming pane: Enable, Local Interface
`192.168.10.1`, Transmission Type **Multicast** (`239.255.42.99`, port 1511 data / 1510
command, Motive defaults). Windows Firewall: inbound UDP 1510/1511 allowed, network
profile Private. Use multicast, not unicast — unicast requires implementing NatNet's
connect handshake for no benefit with one client. Tested against Motive `3.4.1.1`,
NatNet protocol `4.2.0.0`.

Python client: PyPI `natnet` (0.2.0, github.com/TimSchneider42/python-natnet-client),
pinned in `requirements.txt`. `natnet_test.py` (repo root) is a minimal working
example. Don't hand-roll a NatNet byte parser — NatNet 4.1+ added a `_dataset_size`
uint32 field after every section's element count, easy to miss by hand; this library
handles it correctly, version-adaptively.

`live_view.py` (repo root): real-time terminal dashboard (`rich`) showing stream fps
vs Motive fps, rigid bodies, marker sets, unlabeled markers. `python3 live_view.py`
(Ctrl+C to stop, `--duration N` for bounded run).

**Ball marker approach**: wrap the tennis ball entirely in retroreflective tape (not
discrete stick-on markers) — a fully-coated sphere has the same angle-independent
centroid property as OptiTrack's standard marker balls (holds only if the wrap stays
smooth/seamless — wrinkles reintroduce off-axis bias). Only position is needed, not
orientation, so a single tracked point suffices. Needs Motive's marker size threshold
raised (default expects ~9-25mm, a tennis ball is ~67mm). In Motive: select the
tracked point → right-click → create Rigid Body or single-point Marker asset.

## Trajectory Fitting

`trajectory.py` (repo root): fits an independent quadratic (constant-acceleration) per
axis to a buffer of (t, x, y, z) samples via least-squares (`np.polyfit`), predicts
future position or solves for when the trajectory crosses a target plane. Doesn't
hardcode which axis is "up" — Motive's default world frame is Y-up (unlike Z-up
robotics convention), so it fits acceleration on all 3 axes and picks whichever is
closest to -9.81 m/s² as `up_axis` (doubles as a sanity check on sample quality). Run
`python3 trajectory.py` for a synthetic-data self-test.

`live_trajectory.py` (repo root): connects to Motive, tracks a rigid body
(auto-selected if only one present, else `--rigid-body-id`), runs a terminal monitor
showing live state (idle/flight), fitted velocity, short-horizon prediction, apex, and
(with `--catch-axis`/`--catch-value`) predicted catch-plane crossing + throw history.
`python3 live_trajectory.py`.

**Release detection** (automatic, not a button — human reaction latency would corrupt
the samples needed to seed the fit): fits a short rolling window and requires it to
look ballistic for several consecutive windows. **Current defaults**: `--window-short
10` (83ms @ 120Hz — shorter was too noisy, acceleration is a 2nd derivative that
amplifies position noise), `--consecutive 3`, `--accel-tol 6.0` (raised from an
initial 1.5 after real-data testing), `--min-release-speed 3.0` m/s and
`--floor-refractory 0.5` s — the two guards against a **floor bounce being detected as
a new throw**. A bounce is a genuine ballistic arc, so acceleration+residual alone
can't reject it; both guards exploit that a bounce is lower-energy than, and follows, a
landing. `--min-release-speed` (raised from 2.0 — a real recorded bounce hopped at
~2.2 m/s, above the old threshold) sits between that bounce and the ~4.9 m/s minimum
release speed of a >1m-apex lofted throw. `--floor-refractory` suppresses release
detection for that many seconds after a flight ends by landing, so a rebound can't
re-trigger regardless of its energy (each guard independently removed the mis-detection
on the bounce recording; both are on by default). Full tuning history in
`docs/debug_log.md`.

Flight-ending logic treats "stopped" (caught/landed), "lost tracking" (occlusion), and
`--max-flight-duration` timeout as three distinct, separately-logged end conditions.
Note: `tracking_valid` can't always be trusted to flip `False` on occlusion (seen
holding/jumping instead on one real high-throw recording) — the timeout is the
reliable backstop. Separately: a rigid body that's in Motive's asset list but hasn't
appeared in a tracked frame yet this session reports `pos=(0,0,0)`, `rot`=identity,
`tracking_valid=True` (confirmed live 2026-07-15 via raw NatNet capture) — not a bug,
just Motive's default-until-first-sighting value; don't mistake it for a phantom-pose
issue like the RTDE one below.

**Threading**: NatNetClient runs packet parsing and the frame callback on the same
socket-recv thread, so the callback only fits the small fixed-size short window — the
larger full-flight fit runs only in the 10Hz display loop, off the hot thread. Don't
move the full fit into the callback even if it profiles fast enough today.

**Load-bearing accuracy number**: predicted landing position doesn't stabilize below
5cm error until ~67 samples (~0.55s of observation) — the earliest point a predicted
catch point should be trusted enough to commit the arm, not just "as soon as a fit
exists." Don't use `residual_rms` as a trust/confidence proxy — it *increases* with
more samples even as prediction accuracy improves (few points overfit a clean-looking
but under-constrained parabola).

`visualize_trajectory.py` (repo root): separate from `live_trajectory.py` on purpose —
that one stays lean for eventual robot-control use, this one carries matplotlib
rendering. Two 2D panels (top-down x-z, height vs. horizontal distance) rather than a
rotating 3D plot. Confidence trigger for "flash when trustworthy": **prediction
stability**, not a ground-truth check (none available live) — keeps the last
`--stability-window 4` predictions, locks in once they agree within `--drift-tol
0.05`m, freezes the curve and flashes. `--snapshot-step 20` freezes/color-codes
prediction curves at fixed sample counts for visual "how many samples needed"
inspection. `--debug` prints release/landing transitions, render-loop rate, and
per-tick sample count/prediction/spread. Run with `python3 visualize_trajectory.py`.

## Robot Control

**Current status: real motion is confirmed working via raw URScript sent directly
over the robot's secondary client interface (port 30002) — no `ur_rtde`, no External
Control URCap.** `ur_rtde` (the originally intended approach for high-rate streaming
control) has not been gotten working on this robot/URCap combination — investigation
paused, not abandoned; see `docs/debug_log.md` for the full troubleshooting history
and untried next steps. Raw URScript-over-socket is what to use for anything on the
real arm right now.

### Network setup (confirmed working)
Second physical link, separate from the OptiTrack cable: UR12e control box to laptop
via USB-Ethernet adapter (`enxd0c0bf2dd1ed`), own subnet:
- UR12e controller: static `192.168.20.1` (pendant: `Settings → System → Network`)
- Laptop (`enxd0c0bf2dd1ed`): static `192.168.20.2/24`, no gateway
- **Remote Control mode must be ON** (`Settings → System → Remote Control` + pendant
  mode switch top-right set to "Remote") — without it the controller rejects
  externally-sourced writes. Note: the pendant's own Play button is greyed out by
  design whenever Remote Control is on — program start/stop must go through the
  Dashboard Server (`ur_status.py`/`ur_play.py`) instead.
- `ur_rtde==1.6.3` pinned in `requirements.txt` (used for read-only state today, plus
  parked `ur_rtde`-based control scripts)
- **SSH access confirmed working (2026-07-16)**: `root@192.168.20.1`, key auth via
  `~/.ssh/config` alias `ur12e` (`ssh ur12e`). Must be enabled once on the pendant
  first (`Settings → Security → Secure Shell`, off by default) and needs
  `IdentitiesOnly yes` + an explicit `IdentityFile` in the ssh config if the key isn't
  one of OpenSSH's default-named files — a bare `ssh` silently falls back to password
  otherwise. Also needs `PubkeyAuthentication yes` in the robot's own
  `/etc/ssh/sshd_config` (UR's factory image ships this **disabled**, `no`). Used for
  pulling the robot's own logs, never for anything on the hot control path: see
  `catch.py`'s `wrap_up_session()`/`pull_robot_session_logs()`. The controller's own
  logs live at `/root/log_history.txt` (compact `::`-delimited events, codes decoded
  by `docs/ur_error_codes_en.properties`, extracted from the robot's own
  `errorcodes-*.jar`) and `/root/polyscope.log` (human-readable, includes explicit
  `PROTECTIVE_STOP` lines) — both real timestamps, filterable by wall-clock window.
  `/root/flightreports/*.zip` are UR's own auto-generated incident bundles (full
  per-joint telemetry, triggered on any fault) but **only the last 5 are kept, oldest
  evicted on the next trigger** — pull promptly after anything worth keeping.
  **This is root on a shared robot controller other people also use — be careful.**
  Treat every SSH session as strictly read-only unless the task explicitly calls for
  a change: pull/read logs freely, but do not delete, move, or edit anything on the
  robot (including its own log files, configs, or `sshd_config`) unless specifically
  instructed to. The `PubkeyAuthentication`/`sshd_config` edit above was one
  deliberate, user-confirmed exception, not a standing license to reconfigure the
  box — don't repeat that pattern on other files without being asked again.

### Working scripts (repo root)
- `ur_status.py` — status + `--clear` recovery (robotmode/safetymode/programState via
  Dashboard Server, port 29999)
- `ur_play.py` — remotely trigger a loaded/paused pendant program
- `ur_get_pose.py` — one-shot print of current TCP pose + joint angles (read-only,
  via `rtde_receive`) — use this to find real coordinates: freedrive/pendant-jog the
  arm to a spot, run this, copy the printed `--pose`/`--joints` values
- `jog_ur_raw.py` — interactive keyboard teleop (WASD/RF translate, IJKLUO rotate,
  space=stop, q/Esc=quit) via raw URScript-over-socket. Each held key sends a short,
  self-stopping `speedl()`+`stopl()` burst — motion is stepped/pulsed, not smooth, but
  a crash mid-jog just leaves the robot finishing its current bounded burst safely.
- `ur_goto_raw.py` — send the robot to an absolute/relative target via `movel`
  (`--pose`/`--relative`) or `movej` (`--joints`), full speed the whole way (no
  pulsing). Has a safety clamp (`check_move_size`) refusing moves >0.15m/axis or
  >0.5rad/joint from current position unless `--force`d — this exists because of a
  real incident, see below.
- `ur_loop.py` — loops the arm between two fixed poses (`--pose-a`/`--pose-b`,
  from `ur_get_pose.py`) for speed testing; `--speed`/`--accel` to tune, prints
  measured per-leg timing. Ctrl-C stops cleanly (robot finishes current move, holds).
  `--force-start` needed on the first move to pose A (same safety clamp as above).
- `speed_char.py` — characterizes *achieved* peak TCP speed vs *commanded* speed to
  find the real velocity ceiling. Sweeps commanded speed upward; at each speed runs
  several increasing-distance moves from a fixed pose A (a short move never reaches
  cruise), takes the fastest peak, pins `--accel` high so accel never bottlenecks, and
  reads true peak off `getActualTCPSpeed()`. Live plot (commanded-vs-achieved, one line
  per distance, dashed y=x ideal) + CLI table (`--no-plot`), auto-stops on plateau,
  saves JSON+PNG. Same segment-as-safety-envelope model as `ur_loop.py` (`--pose-a`/
  `--pose-b` on a long clear non-singular path, `--force-start` for the first move).
  Produced the 2026-07-14 speed numbers above.
- `ur_servo.py` — **the v2 streaming control layer (2026-07-20, validated).**
  Continuous `servoj` setpoint streaming at 125-500Hz via a **reverse socket**:
  ONE persistent URScript program is sent to :30002, and *it* calls
  `socket_open()` back to this laptop (robot = TCP client, laptop = server);
  setpoints then stream as packed int32s over that open connection. A per-tick
  send cannot work at servo rates — each send opens a fresh connection and
  preempts the running program — so do NOT "fix" this back into the
  send-a-script-per-tick shape every other motion script here uses. Robot-side:
  a `servoj` thread self-timed by its own `t`, always using the latest setpoint
  (latest-wins, deliberately NOT the circular buffer UR's servoj article
  describes — that's for replaying precomputed trajectories, and queueing here
  would only add latency). Poses go over the wire, `get_inverse_kin` runs on the
  robot. Exports `RateLimiter`, `ServoStream`, `add_servo_args` for reuse.
  `--self-test` = offline math/encoding checks, no robot. `--bench` = canned
  sine sweep with no perception in the loop (this is the "prototype
  `servo_track.py` standalone first" step). Measured first run: **2473
  setpoints, 0 late ticks, 3.1mm lag at 125Hz**.
- `track_ball_servo.py` — first perception-driven use of the above: the tool
  continuously follows the tracked ball, retargeting every tick. Reported
  working well / "very responsive" on its first real run (2026-07-20). It
  **pursues the ball's current position** — no release detection, no prediction,
  no feasibility gate, so do NOT throw at it; it exists to measure how well the
  arm follows a moving mocap-derived setpoint. `--dry-run` runs the full
  pipeline with no program sent; `--record` writes per-tick JSONL to
  `servo_logs/` (gitignored) for lag analysis.
- `jog_ur.py`, `ur_move_test.py`, `ur_movel_test.py`, `ur_speedscale_diag.py`,
  `ur_rtde_diag.py`, `ur_freq_test.py`, `ur_latency_probe.py` — `ur_rtde`-based
  scripts from the paused investigation; not currently working end-to-end, kept for
  when that's revisited (see `docs/debug_log.md`).

### Key safety rules
- **`--dry-run` before a real session is a judgment call, not a blanket rule**
  (2026-07-22 user directive, superseding the earlier reflexive "dry-run
  first, then a real session" phrasing elsewhere in this file — those are
  leftover instances of the old default, not a standing requirement). Reach
  for `--dry-run` when a change has a REALISTIC chance of something novel
  going wrong: a new motion primitive, a materially different commit/gate
  timing, a bigger commanded speed/accel/reach envelope, anything touching
  the safety-critical path for the first time. Skip it for incremental
  tuning, logging/analysis-only changes, or anything where the worst
  realistic outcome is a protective stop — a protective stop is an
  acceptable, expected backstop here, not an incident to engineer around.
  User has never actually run `--dry-run` before a first real test
  themselves. Don't re-insert "dry-run first" as reflexive caveat text on
  every change going forward; only flag it when the risk genuinely warrants it.
- **Never force-kill (SIGTERM/`timeout`) a script holding an active `ur_rtde`
  control session** — skips Python's `finally` cleanup, strands the robot's real-time
  thread, causes a protective stop on the next run. See
  [[robot_control_testing_safety]] memory.
- **`servoj` has NO speed limit of its own** — hand it a joint target far from
  the current one and it drives there as hard as `gain` allows, which on this arm
  means a protective stop or worse. So the setpoint STREAM, not the robot, is
  what has to be well-behaved: `ur_servo.RateLimiter` bounds every setpoint
  relative to the **previous setpoint** (never the arm's measured position —
  that lets lag accumulate into a lunge) and accel-limits the step vector so
  direction reversals are bounded too. Any new streaming code must go through it.
  Corollary: a streaming script's failure paths must send an explicit hold
  (`servo=0`), never just stop sending — silence for `--sock-timeout` ends the
  robot's program outright. And never pass `sock_timeout<=0`: URScript reads that
  as "block forever", silently deleting the host-crash watchdog (guarded, raises).
- **The robot controller is shared — other people use it too.** SSH access (see
  "Network setup" above) is root, on a real, currently-in-use controller. Default to
  read-only over that link (pulling/reading logs is fine and expected); don't
  delete, move, or modify anything on the robot — its files, configs, running
  state — unless the task specifically calls for it.
- **Blocking `ur_rtde` calls (`moveL`/`moveJ`) aren't interruptible via SIGTERM** —
  a blocking pybind11 C++ call doesn't return control to Python to process a pending
  signal. Use `timeout -s KILL` or plan to manually `kill -9` + recheck status.
- **Never build a target pose/joint value as a URScript expression in an f-string and
  trust the syntax** — resolve it in Python (via `rtde_receive`) and send a plain,
  already-computed literal instead. A bug here (bare list instead of a `p[...]` pose
  literal) caused a real emergency stop. Always double-check function signatures
  before using a new URScript primitive.
- **A `movel` to a side target is a joint-speed violation waiting to happen**: a
  straight Cartesian line at TCP speed v and reach r demands base-joint speed ~v/r —
  at the real faulting commits (r=0.44–0.79m, v=1.1–1.5 m/s) that's 137–195°/s, over
  the 120°/s base limit → C153A0. Measured on the 2026-07-16 sessions: fault rate vs
  target azimuth swing from the wait pose (accel≤4) was 0% <10°, 36% at 20–35°, 50%
  >35°. Lowering `--accel` never fixes this (it's a velocity-on-path violation, not
  acceleration). `catch.py --catch-move movej` (+ `--yaw-follow`) is the fix — a
  movej plans in joint space and cannot violate joint limits — validated on the
  real arm and on by default (see Catch Integration). See `docs/debug_log.md` 2026-07-17.
- **Home position is a singularity** — pendant jogging (and any move command) can
  throw a false "no IK solution" error there. Freedrive off home position first before
  assuming it's a real fault.
- Before assuming a speed ceiling is a hardware/safety limit, check the pendant
  speed-slider override (`rtde_receive.getTargetSpeedFraction()`) isn't silently
  capping things — seen at 20% in one past session.
- **A move that ends by faulting (protective stop, etc.) looks identical to a move
  that ended by arriving** to any check that only watches `getActualTCPSpeed()` — a
  frozen-by-fault arm reads the same near-zero speed as one that landed on target.
  `catch.py`'s `move_to()`/`check_safety_mode()` guard against this by also checking
  `rtde_receive.getSafetyMode()` (must be `1`/NORMAL); a real 2026-07-15 incident
  (see `docs/debug_log.md`) went undetected for ~30s without this check. Any new
  motion code that judges success from speed/position alone should do the same.
- **A detected fault no longer kills `catch.py`'s session, but it also does NOT
  auto-clear itself** (2026-07-16 user directive — a prior version auto-cleared via
  the Dashboard Server; removed on request, clearing a fault is a human decision, not
  a script's). `wait_for_fault_clear()` blocks, polling `getSafetyMode()`, until
  *you* clear it on the pendant (or `ur_status.py --clear`), then drives back to the
  wait pose and resumes on its own. Ctrl-C still aborts at any point, including
  mid-wait.

## Catch Integration (perception → robot)

Plan for connecting the two working-but-separate pieces (trajectory prediction, robot
motion) into an actual catch. Full rationale in the 2026-07-13 design discussion; keep
this section as the load-bearing summary for implementation.

**Governing insight — don't track the ball with the arm.** Because effective TCP speed
is only ~1.2 m/s (measured; see hardware section), treat the catch as a *race, not a
pursuit*: compute ONE intercept point on a
fixed catch surface, decide if the arm can beat the ball there, and if so go and wait
(arriving early is free — the ball comes to the tool). Shrink dimensionality by fixing a
**catch plane** inside the reachable, non-singular workspace AND the mocap volume;
reuse `trajectory.py`'s existing "trajectory ∩ plane → (point, time)" solver as the
intercept solver.

**Coordinate frames — one rigid transform `T_base←mocap`.** Everything hinges on the
6-DOF transform from Motive's Y-up world frame to the robot Z-up base frame. Do NOT
hand-code the Y-up/Z-up axis swap — *measure* the full rotation via calibration.
- Method: **point-set registration (Umeyama/Kabsch)**, SVD-based, ~10 lines of NumPy.
  Command the arm through ~20-40 poses spanning the workspace (well off the home
  singularity); at each, record `p_robot` from FK (`rtde_receive.getActualTCPPose()`,
  as in `ur_get_pose.py`) and `p_mocap` from a marker at the TCP. Solve the two point
  clouds for the best-fit rigid transform. Published mocap-robot setups hit <1.5mm RMSE.
- The marker must sit *on* the TCP point (marker-post tip = TCP, or fit sphere center +
  set TCP offset to match) — a 5mm offset is a 5mm systematic miss on every catch.

**Markers on the robot — yes, two rigid bodies (both non-coplanar + asymmetric so
Motive resolves a unique orientation):**
- **Base RB** (on base/mounting plate): makes `T_base←mocap` a live, continuously-
  measured transform instead of a static assumption — self-updates if the rig is bumped
  or Motive re-origins. Calibrate once relative to this RB; never recalibrate unless
  markers move relative to the base.
- **Tool/wrist RB** (on the end-effector, offset to avoid occluding/being occluded by
  the ball): ground-truth TCP position in flight for closed-loop correction and for
  verifying the tool actually reached the commanded catch point. Put it on the funnel
  rim so you verify the mouth, not the flange.

**Timing budget is the whole game — measure, don't guess.** Race is `time_to_impact`
(from the fit) vs `time_to_arrive` (arm move time). Dominant terms: the ~0.55s / ~67-
sample observation delay before prediction is <5cm (existing load-bearing number), and
robot motion time (distance ÷ ~1 m/s + accel ramp). Build a **`distance→move_time`
lookup** from `ur_loop.py`'s measured per-leg timing — that curve is the feasibility
oracle. Commit rule: `if intercept_reachable and move_time(dist) < time_to_impact -
margin: go` — fire on the FIRST qualifying tick, not once a stability check also
agrees (see the 2026-07-15 revision below: waiting for stability was found to
systematically discard the best/only opportunity, since margin only ever shrinks).
Consequence: needs lofted throws with flight times comfortably above ~1s; bound throw
speed/distance deliberately.

**Control primitive — start with a single `movel`.** Because arriving early is free,
v1 needs no moving-setpoint tracking: as soon as the move is feasible, fire ONE `movel`
to the intercept over the proven raw-URScript-over-socket path, arrive early, wait.

**v2 streaming — BUILT AND VALIDATED 2026-07-20, see `ur_servo.py` below.** (The
earlier sketch here said "stream servoj over port 30003"; that was wrong in an
important way. A per-tick send to *any* of the robot's ports cannot work — each
send opens a fresh connection AND preempts the still-running program. The
program is sent once, over 30002, and the robot dials back to us. See
`ur_servo.py`'s docstring and `docs/debug_log.md` 2026-07-20.)

**End effector — net/funnel with a wide mouth** converts the few-cm error budget into
forgiveness (10-15cm effective radius). Rigid cup / pinpoint gripper is v3 hero-mode.

**Safety on the catch move:** a catch move is inherently larger than
`ur_goto_raw.py`'s 0.15m/axis clamp — do NOT just spam `--force`. The catch code path
needs its own bounded, IK-feasible, non-singular, pre-checked workspace envelope;
treat "clamp off" as one deliberate reviewed path, not a sprinkled flag.

**Pieces to build:** `calibrate_frames.py` ✅ (Umeyama routine → saves `T_base←mocap`
R/t to JSON) · `frames.py` ✅ (pure `mocap_point→base_point`, testable, no I/O) ·
`catch.py` 🔨 BUILT, dry-run validated against a live Motive replay (conductor: NatNet →
existing release detect + fit → trajectory ∩ catch-plane → `frames.transform` →
feasibility gate → real `movel` over raw URScript-over-socket). Pre-positions at a wait
pose (~0.6m reach, catch height ~1.1m floor) at startup, derives the horizontal catch
plane through that height, and fires ONE max-speed `movel` per throw on the FIRST tick
where the gate passes (feasible OR possibly-catch) — NOT once a stability check also
agrees; see "Run recording" below for why that changed 2026-07-15. Stopped with
**Enter (clean stop) or Ctrl-C (emergency abort)** — both converge on the same
stopl+teardown path (2026-07-16; Enter uses a non-blocking `select()` on stdin each
tick, not a thread, so nothing is left reading stdin when the end-of-session prompts
below run). A detected robot fault no longer kills the session — see "Key safety
rules" `wait_for_fault_clear()`. **Robot position
comes from RTDE FK only, never mocap** — it's
validated against a looping Motive *replay* where the tool RB is frozen. Own
conservative catch envelope (`check_catch_envelope`: reach 0.45–1.20m, base-z
−0.25…+0.55m, azimuth ±75° of the wait pose, no cap on distance from wait pose) is
the single reviewed "clamp-off"
path — no `--force` spam. Reach floor raised 0.35→0.45m 2026-07-15 after a real
protective stop (see "Run recording" below and `docs/debug_log.md`). Azimuth band
added 2026-07-17 after a real self-collision: a hand grabbing the ball out of the
box was detected as a throw and committed the arm to a target 158° behind the wait
azimuth. Same incident class also spawned `check_release_guard()` (default on): a
release originating <1.0m (horizontal) from the base, or moving >100° away from
it, is logged as a `guard` event and never commits — thresholds set from all 161
recorded real throws (release ≥1.1m p5, toward-robot angle ≤27° p90), which all
pass. **Post-commit re-aim** (default on, `--no-reaim`): once the committed
move has settled (never preempting motion), the loop keeps refitting and sends
a short envelope-checked correction move when the refined prediction drifts
≥2cm and the time budget allows (≤3 per throw, `reaim` events in the JSONL).
Found effectively dormant on real data (2 firings in 98 attempts — travel time
eats the settle window); `--reaim-preempt` (opt-in, unvalidated) interrupts the
running move on ≥5cm drift instead (leads with `stopj()`). `--catch-move movej`
+ `--yaw-follow` are the fix for the side-throw protective stops (see Key
safety rules), on by default (`--catch-move movel` / `--no-yaw-follow` still
available) — confirmed no catch-accuracy regression vs `movel`
(`docs/debug_log.md` 2026-07-18 §3). `--poll-hz` default 50 (poll interval is
pure decision latency); console prints throttled, ticks all recorded. `throw_end`
logs a `caught_guess` (ball last seen <30cm from tool ⇒ swallowed by the box)
plus a live session tally, and faulted throws also get their
`throw_end`/`throw_samples` logged post-fault-clear. `--dry-run` logs every
decision with zero motion; confirmation prompt before the first move. Keep the
heavy fit off the socket-recv thread (existing threading rule). **Must
`set_tcp()` the calibrated `tcp_offset` at startup** — it's a controller-side
runtime setting, not implied by the transform file or `movel()`, and does NOT
reliably carry over from whatever a previous script set it to (a missed
`set_tcp()` once silently commanded the flange instead of the funnel,
undershooting every catch by the offset — see `docs/debug_log.md` 2026-07-15).
Current `tcp_offset` is `0.0725`m (box centroid), read from
`T_base_from_mocap.json` at runtime — don't hardcode it. `track_rigid_body.py`
also does this correctly.

**Payload/CoG is set once via the pendant's Installation → Payload Estimation
wizard, not by `catch.py`** — user decision, "it never changes." Unlike
`tcp_offset` (overwritten by `calibrate_frames.py`, so `catch.py` must resend
it every run), nothing else in this toolchain touches payload, so there's no
"prior script left the wrong value" hazard here. A wrong payload/CoG (factory
default: symmetric point mass, mismatched with the real offset funnel) was the
traced root cause of a recurring `C153A0`/`C157A0` base-joint protective stop —
see `docs/debug_log.md` 2026-07-16. If it recurs, check the pendant value
before assuming this script's motion parameters are at fault.
`check_safety_mode()` also cross-checks the Dashboard Server, not
`rtde_r.getSafetyMode()` alone (that investigation found the RTDE-based check
lagging a real protective stop by ~9s).

**Run recording (`catch.py --record`)** — off by default; when passed, writes
`catch_logs/catch_log_<timestamp>.jsonl` (dir auto-created), one compact JSON
object per line covering every feasibility tick, gate/commit/refuse decision,
throw start/end, robot move, and the raw per-throw ball trajectory
(`run_start`/`throw_start`/`tick`/`commit`/`refuse`/`throw_end`/`throw_samples`/
`move`/`run_end` event types, see `Recorder`/`rnd()` in `catch.py`).
`throw_samples` is captured in `finalize_flight()` (`live_trajectory.py`) into
`FlightRecord.raw_samples`, not read back out of `SharedState.flight_buffer`
later — that buffer is already reset to `[]` by the same function by the time
any poll-loop consumer would notice. Each line carries a 1-based `"throw"`
ordinal (filter with `jq 'select(.throw==N)'`) and `t`, the raw NatNet/Motive
sample timestamp — within one continuous Motive session (no relaunch), a
JSONL line's `t` lines up exactly with the timestamp seen on replay of the
same session (confirmed 2026-07-15); throw-ordinal is the fallback across a
relaunch. The point: a session can be researched later from `catch_logs/`
alone, with no need to go back into Motive replay. `throw_end` includes the
arm's actual TCP pose, so a committed catch's target can be diffed against
where the tool ended up. See `docs/debug_log.md` 2026-07-15 for the
tick-by-tick analysis that motivated the current commit rule (fire on the
first qualifying tick, not a stability window).

**Session wrap-up (`wrap_up_session()`, 2026-07-16, on by default — `--no-wrapup` to
skip)**: once the NatNet/RTDE connections are fully torn down (deliberately outside
the hot loop and its `try/finally` — nothing here may add latency to the live
trajectory/feasibility calculation), prompts for an optional session name and a
free-text description of how it went, then `pull_robot_session_logs()` does one SSH
round-trip pulling exactly that session's wall-clock-timestamp-filtered slice of the
robot's own `log_history.txt`/`polyscope.log` (see "SSH access" under Robot Control)
plus any flight-report zip triggered during it. Lands in
`robot_logs/sessions/<timestamp>_<name>/` (gitignored, same pattern as
`catch_logs/`): `notes.txt` (name/description/start/end/throw count) +
`log_history_slice.txt` + `polyscope_slice.txt` + any pulled flight report. Runs on
every exit path, including Ctrl-C.

**Recommended order (status as of 2026-07-20):**
- (1) ✅ **frame registration + rigid bodies — DONE.** `calibrate_frames.py` +
  `frames.py` work; current `T_base_from_mocap.json` (single-marker +
  tool-offset joint solve) is **4.25mm fit RMSE**, live-verified to **1.5–9mm**
  across the workspace (old rigid-body calibration kept as
  `T_base_from_mocap_old.json`, 27.7mm). `track_rigid_body.py` closes the full
  spatial pipeline (mocap → transform → live motion) and validates the
  transform end to end — reported working well. Recalibration history:
  `docs/debug_log.md` 2026-07-20.
- (2) 🔄 **latency/feasibility characterization — IN PROGRESS.** Speed ceiling measured
  (`speed_char.py`, see hardware section: 1:1 to ~1.2 m/s, ~1.3 peak, joint-limited).
  Still needed: the `distance→move_time` curve (the feasibility oracle) and one full
  perception→motion latency measurement.
- (3) ~ static-target dry run — largely covered by `track_rigid_body.py`; a funnel-on-a-
  held-ball run through the actual `catch.py` path is still worth doing once it exists.
- (4) 🔨 v1 catch — `catch.py` BUILT (lofted throws, fixed plane, single `movel`, funnel,
  pre-positioned wait pose). Needs a live-throw validation run; start with `--dry-run`.
- (5) 🔨 v2 servoj streaming — **transport DONE and validated 2026-07-20**
  (`ur_servo.py`, `track_ball_servo.py`; 0 late ticks, 3.1mm lag at 125Hz).
  **Early commit implemented 2026-07-22**: `--catch-move servo` now starts
  driving toward the predicted intercept at `--early-commit-samples` (default
  5, vs `--commit-samples`' 40 for movel/movej) with the feasibility gate
  bypassed for that first move — it fires on any valid plane crossing that
  clears the envelope/release-guard, and the existing continuous-retarget
  branch (`catch.py`, "attempted and servo_mode") pulls the noisy early guess
  toward the true intercept every tick after. This is what dissolves the
  commit-then-locked model and targets the throws the feasibility gate used to
  refuse outright. **Not yet validated on the real arm** — this changes commit
  timing materially (bypasses the feasibility gate for servo's first move), so
  worth a `--dry-run` before a real session per the Key safety rules judgment
  call above. Then tighter margins + smaller tools.

**Reference prior art / methods:**
- EPFL/LASA "Catching Objects in Flight" (Kim & Billard) — canonical mocap + fast arm +
  early trajectory model; confirms predict-early/refine-on-approach:
  https://actu.epfl.ch/news/ultra-fast-the-robotic-arm-can-catch-objects-on-th/
- Robot Anticipation Learning System (RALS) — reads thrower's arm pre-release for a
  head-start (v3 idea): https://www.mdpi.com/2218-6581/10/4/113
- "Finding the Kinematic Base Frame of a Robot by Hand-Eye Calibration Using 3D Position
  Data" — the base-frame registration problem:
  https://www.researchgate.net/publication/291417721
- UR "Trajectory improvements for servoj()":
  https://www.universal-robots.com/articles/ur/programming/trajectory-improvements-for-servoj/
- Smooth real-time target-update on UR (servoj buffering pitfalls):
  https://dof.robotiq.com/discussion/1255/
- sawUniversalRobot — realtime script interface over port 30003:
  https://github.com/jhu-saw/sawUniversalRobot

## Open Questions / To Revisit
- How many Flex 13 units are available, and can they cover both the throw's mid-flight
  trajectory and the arm's catch zone?
- What end effector catches the ball (rigid cup/funnel vs. soft net vs. passive
  compliant catch) — precision vs. forgiveness tradeoff given ~1.2 m/s TCP speed limit.
- RTDE control loop rate and achievable end-to-end latency (perception → prediction →
  robot command → robot motion) — not yet measured end-to-end with real
  perception+prediction in the loop.
- Whether markers go on the ball itself (retroreflective sphere on/in a tennis ball) or
  tracking is passive/markerless — markered is far easier and more reliable to start.
- Safety: throwing objects at a moving cobot arm needs a real risk assessment even
  though UR e-series has built-in safety functions; likely need a cage/soft barrier or
  restricted demo zone regardless of "collaborative" rating.
- Torque limits on sudden/hard trajectories: a fixed A↔B loop test doesn't exercise
  the direction-change-heavy motion a real catch trajectory will need — revisit once
  real trajectory prediction is driving the arm.
- `ur_rtde`/External Control URCap root cause still open (deprioritized, not urgent —
  raw URScript-over-socket is a working fallback for now). See `docs/debug_log.md`.
- **`--reaim-preempt` and `--tilt-follow` need real-arm validation** (both
  opt-in): preempt replaces a running move (validate at low speed, watch for protective
  stops at the preemption instant); tilt-follow needs an IK-reachability check near the
  envelope edge. Motivations + data: `docs/debug_log.md` 2026-07-18 §2/§4/§8.
- **`ur_servo.py --self-test` FIXED (2026-07-21)** — root cause was the final-approach
  deadband in `RateLimiter.step()` (not the accel-search/geometric-term interaction
  originally suspected): it snapped straight to the target and zeroed `prev_v_*`
  unconditionally, with no check that doing so was itself accel-consistent. That let
  a still-jittering (not-yet-converged) predicted target re-trigger the snap almost
  every tick, and separately let a real, still-substantial velocity get discarded to
  a fictitious 0 baseline, both producing real discontinuities in the commanded
  stream (measured 25-108 m/s² against 2-4 m/s² caps). Fix: gate the snap on the
  acceleration it would itself imply (same polar decomposition the main accel search
  uses) and, when taken, carry the velocity actually used forward as `prev_v_*`
  instead of zeroing it. Self-test passes; 30-seed×150-throw sweep plateaus at the
  pre-existing documented ~2.15x residual, not a new spike. Traded off: braking
  overshoot at 1.2 m/s rose from 2.4mm to 4.8mm (deadband no longer snaps
  unconditionally) — no longer comfortably inside the 4.25mm calibration RMSE, worth
  watching if catch accuracy regresses. `--catch-move servo` is still otherwise
  unvalidated on the real arm; real-arm protective stops remain a backstop.
- **Servo orientation rate limiting — IMPLEMENTED 2026-07-22, not yet validated
  on the real arm.** Root cause (see the prior entry, kept for the diagnosis):
  `ur_servo.RateLimiter.step()` bounded `cmd_xyz` only; `servo_orient` (yaw-
  follow/tilt-follow) was sent raw every tick, snapping the full commit-instant
  yaw delta straight onto the **base joint** in one tick — `yaw_follow_orientation()`
  rotates about base Z by design (keeps the wrist still), so IK had to reconcile
  "position still near the wait pose" with "orientation already at the final
  azimuth" in ~8ms. Suspected root cause of both the "oscillating to come to a
  stop" behavior and the base-joint accel-limit faults seen in real 2026-07-21/22
  sessions, regardless of `--servo-max-speed`/`--servo-max-accel` (neither
  touched this channel). Fix: `ur_servo.OrientationRateLimiter` (new class,
  alongside `RateLimiter`) slew-limits the rotation vector the same way —
  speed-capped, accel-capped, braking smoothly toward a held target via the
  same `sqrt(2*a*margin)` form `RateLimiter._brake_cap` uses — wired into
  `catch.py`'s `emit_setpoint()` alongside the position limiter, defaulting to
  the same rate budget as `--servo-base-rate-deg-s` (`--servo-orient-max-rate-deg-s`
  to override). Unlike `RateLimiter`, target-overshoot is *not* treated as a
  hard bound (mirroring `RateLimiter._axis_cap`'s own stated philosophy — smooth
  acceleration is the real safety property, not landing exactly on target), so
  the implementation is a single 1D blend, no cylindrical-coordinate machinery
  needed. Self-tested (`ur_servo.py --self-test`, parts 7–8): the commit-instant
  snap is exact from the first tick (matches the actual bug); a bounded,
  understood tail-convergence residual remains when a HELD-STILL target finishes
  converging (up to ~4.7x the accel cap for one tick, vs `RateLimiter`'s own
  documented, accepted 2.15x residual for the same class of discrete-time
  `sqrt(2*a*margin)`-braking artifact — worse here only because this class
  defaults to a punchier 0.15s rate-to-accel ramp, matching `RateLimiter`'s own
  `max_base_accel` convention, not its gentler ~0.3s one). Not yet validated on
  the real arm — offline replay of the 2026-07-22 fault sessions through the new
  limiter (see that day's conversation) showed a ~2.3x larger commit-instant
  orientation jump on faulted throws vs non-faulted ones and the new limiter
  cutting worst-case commanded accel by 50-500x, but that's circumstantial, not
  a live confirmation.

## Status (2026-07-20)
Real motion works (raw URScript-over-socket). Trajectory fitting/prediction
works against recorded and live OptiTrack data. Spatial pipeline connected end
to end: frame registration done (4.25mm fit RMSE — see Catch Integration step
1), `track_rigid_body.py` continuously drives the arm to park under a tracked
rigid body via `frames.py` — proven and reported working well. v2 servoj
streaming transport also built and validated (`ur_servo.py`/
`track_ball_servo.py`, see step 5).

**`catch.py` is catching real thrown balls.** Best measured rate: **81%**
(75/93 committed throws, 2026-07-18 forensic pass over a 160-throw day). Most
misses are rim hits (median prediction error ~13cm), not wild misses, so a
wider/energy-absorbing mouth is currently the single biggest catch-rate lever.
Current default operating point: `--catch-move movej --yaw-follow` (fixes
side-throw protective stops — see Key safety rules), `--accel 4.0 --speed 1.2
--approach-speed 1.5`, `--poll-hz 50`, `--rigid-body-id 3`, wait pose
`(0.042, -0.716, 0.139, 1.584, -0.0824, -0.0573)`.

Full session-by-session history (release guard, re-aim, movej/yaw-follow
rollout, the calibration root-cause chase, box swap, v2 streaming validation)
is in `docs/debug_log.md`, dated 2026-07-16 through 2026-07-20 — consult it
for the "why" behind any of the current defaults above.
