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
- **TCP speed: no fixed cap — it's joint-speed-limited and configuration-dependent.**
  The datasheet's "Tool: approx. 1 m/s" is a *nominal/typical* number, not a hard
  ceiling; the separate "4 m/s" figure is unloaded/freedrive mechanical capability, not
  achievable via commanded moves. **Measured empirically 2026-07-14 with `speed_char.py`**
  (which reads true *peak* TCP speed off `getActualTCPSpeed()`, not leg-average):
  - Commanded speed is delivered **1:1 up to ~1.2 m/s** (achieved/commanded ratio
    1.00–1.02), then rolls off; peak achieved was **~1.32 m/s** in the best
    configuration along the test path.
  - The ceiling **varies along the path**: the same commanded 1.4 m/s peaked 1.324 on a
    0.75 m leg but only 1.204 on a longer 0.97 m leg — because the controller scales the
    whole move down to keep the worst-case joint under its angular limit. So "the max
    TCP speed" is a function of pose/direction, not a constant; expect a different number
    on a different segment.
  - **This supersedes the old "confirmed ~1 m/s" note.** That came from `ur_loop.py` leg
    *timing*, i.e. *average* speed over a move including accel/decel ramps — always below
    peak (e.g. a 0.97 m move commanded at 1.0 m/s took 1.30 s → 0.74 m/s average despite
    a 1.005 m/s peak). Peak ≠ average; use `speed_char.py` for peak, timing/`ur_loop.py`
    for the ramp-inclusive move times that actually gate catch feasibility.
  - Accel: no clean published hard ceiling; `speed_char.py` pins `--accel 40` so accel
    never bottlenecks (controller clamps it internally). Pushing `--accel` past ~2-4x the
    commanded speed has diminishing returns (real minimum ramp time regardless).
  - Full run + reconciliation with the docs: `docs/debug_log.md` (2026-07-14).
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
  acceleration). `catch.py --catch-move movej` (+ `--yaw-follow`) is the designed fix —
  a movej plans in joint space and cannot violate joint limits — pending real-arm
  validation. See `docs/debug_log.md` 2026-07-17.
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
pass. **Post-commit re-aim** (2026-07-17, default on, `--no-reaim`): commits fire
at ~43 samples where prediction error is still 6–17cm (vs 2–5cm at 67–80 samples,
measured on real throws); once the committed move has settled (never preempting
motion) the loop keeps refitting and sends a short envelope-checked correction
move when the refined prediction drifts ≥2cm and the time budget allows (≤3 per
throw, `reaim` events in the JSONL). **2026-07-18 finding: the settle-first
design is effectively dormant on real data** (2 firings in 98 attempts —
travel time eats the window); `--reaim-preempt` (opt-in, unvalidated) lets a
≥5cm drift interrupt the running move instead (leads with `stopj()`). `--catch-move movej` + `--yaw-follow` are
the fix for the side-throw protective stops (see Key safety rules) — **both
promoted to default-on 2026-07-17** after a first real session confirmed no
new faults (`--catch-move movel` / `--no-yaw-follow` still available). Possible
regression to watch: an unconfirmed operator impression that catch accuracy got
slightly worse with movej on — see "movej accuracy" in Open Questions. `--poll-hz` default 50 (was 20 — poll interval is
pure decision latency); console prints throttled, ticks all recorded. throw_end
now logs a `caught_guess` (ball last seen <30cm from tool ⇒ swallowed by the box;
matched session notes at ~91% on 2026-07-16 data) plus a live session tally, and
faulted throws now get their `throw_end`/`throw_samples` logged post-fault-clear
too (previously they were the one class with no trajectory in the log). `--dry-run`
logs every decision with zero motion; confirmation prompt
before the first move. Keep the heavy fit off the socket-recv thread (existing
threading rule). **Must `set_tcp()` the calibrated `tcp_offset` at startup** — it's a
controller-side runtime setting, not implied by the transform file or `movel()`, and
does NOT reliably carry over from whatever a previous script set it to; a real
2026-07-15 bug (see `docs/debug_log.md`) had `catch.py`/`catch_feasibility.py` silently
commanding the *flange* to the funnel's calibrated target, undershooting every catch by
the 12cm `tcp_offset` — ball hit the wrist, ~15cm short of the funnel. Fixed in both
files; `track_rigid_body.py` already did this correctly.

**Payload/CoG is set once via the pendant's Installation → Payload Estimation
wizard, not by `catch.py`** (2026-07-16) — user decision, "it never changes."
Unlike `tcp_offset` (which genuinely gets overwritten by other scripts, e.g.
`calibrate_frames.py` during calibration, so `catch.py` must resend it every run),
nothing else in this toolchain sets a different payload, and the pendant's value is
saved in the installation file — no "prior script left the wrong value" hazard to
guard against here. A wrong payload/CoG (previously the factory default: a
symmetric point mass, mismatched with the real offset funnel) is the traced root
cause of a recurring `C153A0`/`C157A0` base-joint protective stop — see
`docs/debug_log.md` 2026-07-16. If it recurs, check the pendant value first before
assuming this script's motion parameters are at fault. `check_safety_mode()` also
now cross-checks the Dashboard Server (not `rtde_r.getSafetyMode()` alone), after
that same investigation found the RTDE-based fault check lagging a real protective
stop by ~9s in one session. Neither fix has been validated on the real arm yet —
dry-run first.

**Run recording (`catch.py --record`)** — off by default; when passed, writes
`catch_logs/catch_log_<timestamp>.jsonl` (dir auto-created) with one compact JSON object per line covering
every feasibility tick, gate/commit/refuse decision, throw start/end, robot move, and
(2026-07-16) the raw per-throw ball trajectory
(`run_start`/`throw_start`/`tick`/`commit`/`refuse`/`throw_end`/`throw_samples`/`move`/`run_end`
event types, see `Recorder`/`rnd()` in `catch.py`). `throw_samples` carries every
`(t,x,y,z)` sample of that throw's flight — captured in `finalize_flight()`
(`live_trajectory.py`) into `FlightRecord.raw_samples`, not read back out of
`SharedState.flight_buffer` later, because that buffer is already reset to `[]` by
the same function by the time any poll-loop consumer notices the state change. The
point: **a session can now be researched later from `catch_logs/` alone, with no
need to go back into Motive replay** — replay is still what you'd use for
camera/marker-level debugging, not for "what did the ball actually do on throw N."
Each line carries a 1-based
`"throw"` ordinal — this is the intended key for "what happened on throw N": filter
the log to `"throw":N` (`jq 'select(.throw==10)'` or plain grep) and cross-reference
against the Nth throw in a Motive replay of the same session, counting in the same
order. Lines also carry `t` (the raw NatNet/Motive sample timestamp) as a second,
now-confirmed correlation key: verified live 2026-07-15 (raw NatNet capture across a
replay loop boundary) that Motive's "play recording" re-streams each take's *original*
per-frame `timestamp`/`frame_number` verbatim, looping back to the exact same starting
value every cycle rather than rebasing to zero — frame_number and timestamp reset
together, and the reset delta matched the take's own length to the millisecond. So
within one continuous Motive session (no relaunch between the live `catch.py --record`
run and the later replay), a JSONL line's `t` lines up exactly with the timestamp seen
during replay, down to the sample — not just "same ordinal throw." `throw`-index
counting remains the robust fallback if Motive was restarted in between (untested
whether the timestamp epoch survives a relaunch). Every `tick` includes the predicted
catch point, distance-
to-go, move_time, time-to-impact and margin (i.e. the feasibility gate's own
reasoning at that instant); `throw_end` includes the arm's actual TCP pose at that
moment, so a committed catch's target can be diffed against where the tool actually
ended up. **First real use, 2026-07-15**: recorded a 15-throw session; only 3 throws
ever committed, even though several more logged `possible`/`catch` ticks that never
turned into a `movel` — root-caused via the JSONL to the (since-removed)
stability-window commit gate racing a shrinking time budget. Replaying the recorded
ticks against the fixed rule raises commits to 10 of 15; see the "Commit rule" change
above and `docs/debug_log.md` 2026-07-15 for the full tick-by-tick analysis (including
one case, throw 12, where the new rule commits to a still-noisy prediction the old gate
correctly rejected — a real tradeoff, not a pure win).

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
- (1) ✅ **frame registration + rigid bodies — DONE, recalibrated 2026-07-20.**
  `calibrate_frames.py` + `frames.py` work; current default `T_base_from_mocap.json`
  (25 samples, single-marker + tool-offset joint solve, **4.25 mm fit RMSE**, live
  verify pass agreeing to **1.5–9mm**) supersedes the old 07-14 rigid-body calibration
  (27.7mm, kept as `T_base_from_mocap_old.json`). `track_rigid_body.py` closes the
  full spatial pipeline (mocap → transform → live motion, tool parks under a tracked
  body) — this is the continuous form of step (3), and it validates the transform end to
  end. **Reported working well.**
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
  Still to do: drive the stream from a PREDICTED intercept rather than the
  ball's current position, which is what dissolves the commit-then-locked model
  and targets the 46 throws the feasibility gate currently refuses outright.
  Then tighter margins + smaller tools.

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
- ~~`--catch-move movej` accuracy~~ **RESOLVED 2026-07-18**: full-log analysis of all
  2026-07-17 sessions found NO movej regression — caught-ball proximity was marginally
  *tighter* under movej (9.9 vs 11.2cm median), catch rates and prediction errors don't
  differ by move kind. The "felt worse" sessions were rim-out misses (prediction-side,
  throw-dependent). movej stays the default. See `docs/debug_log.md` 2026-07-18 §3.
- **`--reaim-preempt` and `--tilt-follow` need real-arm validation** (2026-07-18, both
  opt-in): preempt replaces a running move (validate at low speed, watch for protective
  stops at the preemption instant); tilt-follow needs an IK-reachability check near the
  envelope edge. Motivations + data: `docs/debug_log.md` 2026-07-18 §2/§4/§8.

## Status (2026-07-20)
Real motion works (raw URScript-over-socket, 2026-07-10/11). Trajectory
fitting/prediction works against recorded and live OptiTrack data.

**Spatial pipeline connected end to end:** frame registration done
(`calibrate_frames.py` → `T_base_from_mocap.json`, recalibrated 2026-07-20 to
4.25mm fit RMSE — see "2026-07-20" below), and `track_rigid_body.py` continuously
drives the arm to park under a tracked rigid body via `frames.py` — mocap →
transform → live motion is proven and reported working well.

**`catch.py` IS CATCHING REAL THROWN BALLS.** 2026-07-16 afternoon: 23 real
sessions, 196 throws, 108 commits, an estimated **~91% catch rate on committed
throws** (last-seen-near-tool heuristic vs session notes). Full quantitative
analysis of those logs: `docs/debug_log.md` 2026-07-17. The recurring pain was
protective stops on side throws — root-caused to movel base-joint speed violation
(see Key safety rules), payload/CoG having fixed only the straight-ahead cases.
Practical operating point found that day: `--accel 3.0–4.0`, `--speed 1.1–1.5`.

**2026-07-17 night-shift pass on `catch.py`**: release guard + envelope azimuth
band (kills the false-release/self-collision class), post-commit re-aim (default
on — fixes the committed-target-is-noisy problem), `--catch-move movej`/
`--yaw-follow`, `--poll-hz` 50, live catch/miss tally, faulted throws now logged.
Details in "Catch Integration" and `docs/debug_log.md` 2026-07-17. Also that
entry's negative result: do NOT constrain the trajectory fit to gravity —
measured worse than the free quadratic on real throws.

**All of the above promoted to defaults same day**, after a first real session
confirmed them: `--catch-move movej` and `--yaw-follow` are now ON by default
(`--no-yaw-follow` to disable; plain `movel` still available via
`--catch-move movel`) — the fix for side-throw protective stops. New default
operating point: `--accel 4.0 --speed 1.2 --approach-speed 1.5`, wait pose
re-taught to `(0.042, -0.716, 0.139, 1.584, -0.0824, -0.0573)`, and
`--rigid-body-id` now defaults to `3` (the standard ball-RB id once base/tool RBs
are also in the scene — auto-select no longer reliable with 3 rigid bodies
present). See the "movej accuracy" entry in Open Questions — an unconfirmed
operator impression that catches got slightly less accurate with movej on.

~~Loose end worth tightening before any smaller-than-box catch tool: calibration
RMSE is 27.7 mm~~ **DONE 2026-07-20** — recalibrated to 4.25mm, see below.

**2026-07-18 night shift — forensic pass on all 2026-07-17 logs + calibration
root cause** (full numbers `docs/debug_log.md` 2026-07-18; analysis scripts in
`analysis/`): day totals 160 throws, 98 commits, 75/93 caught (81%). Key
findings: (1) misses are RIM HITS — 14/17 measurable misses put the ball
within 13–21cm of the box center; true prediction error is median 13cm, i.e.
the error budget lands balls on the rim — a wider/energy-absorbing mouth is
the single biggest catch-rate lever (~91% would have been caught); (2) re-aim
was found effectively DORMANT (2 firings/98 attempts — arm travel eats the
settle window) — `--reaim-preempt` is the designed fix, opt-in, unvalidated;
(3) movej accuracy question resolved, no regression (see Open Questions);
(4) yaw-follow worked (up to 41° commanded) — no visible wrist motion is by
design (wrist joints deliberately held still); `--tilt-follow DEG` (opt-in)
gives functional, visible wrist action instead; (5) the single-marker
calibration failure (80mm RMSE) was a stale/wrong TCP during `p_robot` reads
(~10cm fixed flange-frame offset, provable from the data), NOT a mocap
problem — `calibrate_frames.py` now records full TCP poses, jointly solves
the marker's tool-frame offset (`frames.fit_transform_with_tool_offset`,
default on), and wiggle-tests `set_tcp()` at startup, making marker placement
precision irrelevant; redo the calibration with the new pipeline (expect
≤27.7mm, likely much less — **confirmed 2026-07-20, see below**); (6) the
after-lunch camera reconfig did NOT shift the mocap frame — the 07-14
transform stayed valid all day.

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
decoupling is the whole point of the joint solve); **since superseded, see
below — a smaller box went on the same day and the field was updated to
match.**

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

**v2 streaming works.** `ur_servo.py` + `track_ball_servo.py` (2026-07-20):
continuous `servoj` setpoint streaming over a reverse socket, validated on the
real arm on the first try — `--bench` measured **2473 setpoints, 0 late ticks,
3.1mm lag at 125Hz**, and `track_ball_servo.py` follows a hand-moved ball
"very responsively". This is the architectural unlock for step (5): there is no
longer a discrete move that must *finish* before the arm can be retargeted,
which is what made the post-commit re-aim dormant (2 firings in 98 attempts).
What remains is driving the stream from a **predicted intercept** instead of the
ball's current position. Full write-up, including the three URScript gotchas
that would each have caused a real incident, in `docs/debug_log.md` 2026-07-20.
