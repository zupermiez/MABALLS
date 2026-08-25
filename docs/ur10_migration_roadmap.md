# UR10 (CB3) migration roadmap — 2026-08-24

Goal for today: get the ball-catch pipeline running on a **classic CB3 UR10**, mounted
at a **new location** (not the current UR12e rig), not yet networked, target = a real
arm catching real thrown balls.

## Live status (updated as we go)

- **Network: UP.** Arm is on `192.168.20.1` (same subnet number the UR12e used, but
  now over the laptop's built-in port instead of the dedicated USB adapter — no
  separate adapter is connected today). Laptop already has `192.168.20.2` on
  `enp0s31f6`, so no laptop-side network config was needed. All 5 UR ports open
  (29999/30001/30002/30003/30004), RTDE connects.
- **Model confirmed:** UR10 CB3, firmware/PolyScope **3.15.8** (Jun 2022 build).
  Recent enough for full RTDE support with `ur_rtde==1.6.3`.
- **Health check (RTDE, read-only):** robotmode RUNNING (7), safetymode NORMAL (1),
  target speed fraction 1.0 (pendant slider not capping speed), current pose is a
  normal bent configuration — not at the home singularity.
- **Joint speed limits — RESOLVED.** Checked the official UR10/CB3 technical
  specifications (three independent sources on the actual manual's spec table, not
  the PolyScope-configurable safety "Joint Limits" page): **120°/s base & shoulder,
  180°/s elbow + all three wrists — identical to the UR12e.** The "Key differences"
  section below is now out of date on this point: `catch.py`'s existing
  speed/accel defaults do not need re-deriving for the joint-limit reason. (The
  "180°/360°" number CLAUDE.md flagged as a possible UR10-non-e artifact isn't this
  robot's spec either — still unexplained, but no longer relevant here.)
- Payload (10kg vs UR12e's 12.5kg) and reach (1300mm, same) also confirmed via the
  official spec, not just the earlier web-search summary.
- **Real bug found and fixed: false-positive "settled" on CB3.** First raw
  URScript-over-socket test move (`ur_goto_raw.py`) reported "settled cleanly"
  with the arm still at its pre-move pose - it had commanded the move correctly,
  but the shared settle-detection pattern (`wait_for_stop` / `move_to` / etc.:
  count N consecutive near-zero-speed samples, declare done) has no guard
  against the robot simply not having started moving yet. CB3's script-parse/
  motion-start latency measured ~0.6-0.8s here, comfortably longer than the
  ~250ms settle window - so it declared arrival before the move had begun. This
  pattern was duplicated in **6 places**: `ur_goto_raw.py` (`wait_for_stop`),
  **`catch.py`'s `move_to()`** (the wait-pose/return-path mover - this is the
  one that actually mattered for catching), `spin_base.py`
  (`wait_for_arrival`), `replay_calibration_poses.py` and
  `auto_calibration_points.py` (`wait_for_stop_or_soft_stop`, both identical
  copies). `speed_char.py` already had the correct guard (a `started` flag
  gated on a speed threshold) - used as the reference shape for the fix. All
  six now require observed motion (peak speed above a small "moving" threshold)
  before counting the settle streak, with a fast-path for genuine no-op moves
  (already at target). Verified fixed with a real move on the arm: `after pose`
  now correctly reflects the commanded target instead of the stale pre-move
  reading. **Lesson for the rest of today:** anything that infers "arrived" from
  speed alone, anywhere in this codebase, was validated against the UR12e's
  timing and should be treated as unverified on CB3 until exercised - the servo
  streaming path (`ur_servo.py`) is a different mechanism (continuous setpoints,
  not one-shot settle-detection) so likely isn't exposed to this exact bug, but
  hasn't been checked either.

**Honest risk read first:** every piece of this stack — network, robot-speed
assumptions, catch envelope, frame calibration, servo tuning — was hand-tuned against
*this specific* UR12e sitting in *this specific* spot, over several real sessions
(2026-07-13 → 07-27, see `docs/debug_log.md`). A new robot generation *and* a new
physical location means almost none of that carries over unchanged. The two biggest
time sinks are (a) standing up a whole new OptiTrack calibration volume from scratch
and (b) re-validating servo/joint-speed behavior on CB3 hardware, which is a
different (older, less capable) realtime control board than the UR12e's. Both gate
everything after them. Real catching today is achievable but tight — budget the
afternoon for phases 7-11, and have a fallback (movej mode, lower catch rate, fewer
throws) ready rather than insisting on full servo+tilt-follow parity with the old rig.

## ⚠️ Real incident: protective stops not triggering, stand physically moved (2026-08-24)

First real `catch.py --catch-move servo` session on this arm: it caught throws and
returned to wait pose very fast - fast enough to physically shift the mounting stand
on the ground - and **never threw a protective stop or any pendant fault** the whole
time, even under motion clearly harder/faster than what routinely faulted the UR12e
at similar or lower commanded speeds. This invalidates a load-bearing assumption from
CLAUDE.md's dry-run judgment call ("a protective stop is an acceptable, expected
backstop here, not an incident to engineer around") - that was true for the UR12e's
tuned e-Series PLd Category 3 safety functions specifically, and has NOT been shown
true here. Until this is understood, treat this controller as having no reliable
motion backstop: a real collision may go straight to hardware damage instead of
faulting safely.

Leading hypothesis, unconfirmed: CB3's Safety Configuration (pendant: Installation ->
Safety) may be at wide-open factory defaults rather than the deliberately-tuned limits
the UR12e had configured - **check that screen before doing anything else with servo
mode on this arm.** Also unconfirmed: whether CB3's safety monitoring is simply less
sensitive/differently-implemented than e-Series regardless of configuration.

Mitigations: servo speed/accel caps have already been lowered below the
UR12e-validated defaults (`--servo-max-speed 0.8 --servo-max-accel 4.0`) - done, no
need to re-flag this every session. Still open: stand needs physical securing (was
not anchored, moved under load) before trusting fast motion near it again, and the
safety-configuration question above (wide-open CB3 defaults vs. UR12e's tuned
limits) is still unconfirmed - worth a real check next time someone's on-site with
the pendant, not an every-session blocker.

## Key differences from the UR12e setup (verify, don't assume)

- **Joint speed limits are unverified for this unit.** Do not reuse the UR12e's
  120°/s (base/shoulder) / 180°/s (elbow+wrists) numbers. CLAUDE.md already has one
  documented case of this exact confusion (an earlier wrong note said 180°/360°/s,
  "likely from a UR10-non-e or marketing sheet" — i.e. that number may actually
  belong to a CB3 UR10 like this one, or may not). Pull the real max joint speeds
  from this controller's own manual/pendant "About" screen before setting any
  `--speed`/`--catch-joint-speed`/servo-rate default.
- **No e-Series "Remote Control" mode.** That toggle (`Settings → System → Remote
  Control` + pendant switch) is an e-Series/PolyScope 5 feature the UR12e setup
  depends on. CB3/PolyScope 3.x has no equivalent — external script/RTDE control
  isn't gated the same way. Check on-site how this controller actually behaves when
  a script is sent externally while a program might be loaded on the pendant.
- **CB3 realtime board is weaker.** `ur_servo.py`'s gain/lookahead/rate defaults
  (125 Hz, `SERVO_BASE_RATE_DEG_S=110`, etc.) were tuned and validated only on the
  UR12e's e-Series control box. Re-validate with `ur_servo.py --bench` in isolation
  before trusting servo mode in `catch.py` — CB3 may need a lower rate or different
  gain to avoid jitter/lag turning into a fault.
- **RTDE/firmware version.** `ur_rtde==1.6.3` should negotiate down to older
  controllers, but RTDE needs a reasonably recent CB3 firmware (roughly ≥3.3). Check
  the installed PolyScope version once reachable; if it's very old, RTDE (read-only
  pose/status) may not be available at all.
- **SSH/log layout may differ or be absent.** The `/root/log_history.txt` +
  `/root/polyscope.log` pull (`wrap_up_session()`) is confirmed only for this
  controller's e-Series OS. Not blocking for today — treat as best-effort, lower
  priority than getting the arm moving.
- **Payload:** CB3 UR10 nominal payload is ~10kg vs the UR12e's 12.5kg — irrelevant
  for a ping-pong-ball box, but the Payload Estimation wizard still has to be redone
  on this controller for this tool (never copied automatically, and this is the
  parameter directly implicated in the recurring `C153A0`/`C157A0` protective stops
  on the current rig — see CLAUDE.md).
- **Reach (~1300mm) and flange (ISO 9409-1-50-4-M6) are likely the same** as the
  UR12e, so the existing box/bracket should mount mechanically without redesign —
  confirm on-site, but don't budget engineering time here unless it doesn't fit.

## Code changes needed before touching the new arm

`ROBOT_IP = "192.168.20.1"` is hardcoded (no CLI override) in four files:
`ur_goto_raw.py`, `jog_ur_raw.py`, `calibrate_frames.py`, `track_rigid_body.py`.
`catch.py`, `catch_feasibility.py`, `ur_servo.py`, `spin_base.py` already accept
`--robot-ip`. Fastest safe fix: in those four files, change the constant to read an
env var with the old value as fallback:

```python
import os
ROBOT_IP = os.environ.get("UR_ROBOT_IP", "192.168.20.1")
```

Then `export UR_ROBOT_IP=192.168.30.1` (or whatever the new arm's IP ends up being)
for the session. This avoids silently commanding the wrong robot if both are ever
reachable at once, without a bigger refactor today.

Also: don't overwrite `T_base_from_mocap_v2.json` / `T_base_from_baseRB_v2.json` /
the calibration output for the UR12e — have `calibrate_frames.py` write to new
filenames (e.g. `T_base_from_mocap_ur10.json`) so the existing rig's calibration
stays intact if you go back to it later. Same for any new `DEFAULT_WAIT_POSE` and
catch-envelope constants in `catch.py` — don't overwrite the UR12e's tuned values in
place; branch them (a `--robot-profile` flag or a second constants block) so this
isn't a one-way door.

## Phased plan

**0. Recon (30 min)** — once the arm is powered/reachable on the pendant network
   screen: read off PolyScope/firmware version, real joint-speed limits, and
   whether Remote Control/equivalent exists. This gates several later defaults.

**1. Physical + network bring-up (45-90 min)** — secure mount, power, dedicated
   Ethernet link (recommend a **new subnet**, e.g. `192.168.30.x`, rather than
   reusing `.20.x`, so both arms' configs can coexist without reconfiguring
   anything). Static IP on the pendant. Confirm `ping`, Dashboard Server (29999),
   RTDE, and port 30002 are all reachable from the laptop.

**2. Code prep (30 min, parallel with #1)** — the env-var IP fix above; branch
   calibration/constants filenames; sanity-read `ur_status.py`/`ur_goto_raw.py`
   against the new IP once reachable.

**3. Safety basics (30-45 min)** — freedrive off home (singularity) before jogging;
   set payload/CoG via the pendant wizard for the box+ball tool; confirm Dashboard
   commands (`ur_status.py`-equivalent) behave the same on CB3.

**4. Validate raw URScript-over-socket path (30-45 min)** — small moves via
   `jog_ur_raw.py` / `ur_goto_raw.py` (`check_move_size`'s clamp still applies)
   before anything larger. This is the path everything else builds on.

**5. OptiTrack bring-up at the new location (60-90 min)** — camera placement,
   Motive calibration wand routine, ground plane, verify enough Flex 13 coverage
   for *both* the throw zone and catch zone (open question already flagged in
   CLAUDE.md — now doubly relevant with a brand-new volume). Sanity check with
   `live_view.py`.

**6. Rigid bodies (20 min)** — attach base RB + tool RB to the new arm/mount
   (non-coplanar, asymmetric). Confirm spare marker sets exist — if not, this
   blocks everything downstream of it.

**7. Frame registration (45-60 min)** — `calibrate_frames.py` sweep of ~20-40
   poses. No `DEFAULT_WAIT_POSE` exists yet for this arm, so the sweep needs a
   manually-jogged safe starting/via pose first (chicken-and-egg — move slowly,
   watch for singularities). Recalibrate `tcp_offset` for the box on this flange —
   don't reuse `0.0725m`, remeasure.

**8. Derive new wait pose + catch envelope (20-30 min)** — pick a safe, central,
   non-singular wait pose at the intended catch height. Set initial
   `CATCH_Z_MIN`/reach/azimuth bounds **conservatively**, based on a physical
   survey of the new mount/stand geometry — the UR12e's `CATCH_Z_MIN=0.119` exists
   *because of a real collision with that specific stand*; don't copy the number,
   copy the method (look at what's near the arm at low z, set the floor above it).

**9. servoj bench validation on CB3 (30-45 min)** — `ur_servo.py --bench` in
   isolation, starting at a conservative rate (try below 125Hz if CB3 shows jitter),
   before wiring into `catch.py`. If this doesn't stabilize in time, fall back to
   `--catch-move movej --yaw-follow` for today's demo — it's the proven, joint-space-
   safe path from the original UR12e bring-up and doesn't depend on CB3 realtime
   performance being as good as the e-Series box.

**10. Pursuit sanity check (30 min)** — `track_ball_servo.py` (or the movej
    equivalent) to confirm end-to-end tracking responsiveness before attempting a
    real release-to-catch sequence.

**11. First real catches** — `catch.py --dry-run` first here: this genuinely
    qualifies under the project's own dry-run judgment call (new arm, new
    envelope, first time on this safety-critical path), unlike routine tuning
    changes on the existing rig. Review the decision log, then go live starting in
    `movej --yaw-follow` mode with a conservative envelope before opting into
    servo/tilt-follow, which is only validated on the old arm.

**12. Iterate** — same loop as the original 2026-07 history: watch misses, nudge
    envelope/timing, log sessions (`--record`) so today's tuning is reconstructable
    later the same way `catch_logs/` works for the UR12e.

## Quick-reference issue list

- CB3 joint-speed limits: unverified, do not reuse UR12e's 120/180 numbers blind.
- No e-Series Remote Control toggle on CB3 — access model differs, verify on-site.
- CB3 realtime board likely weaker — servo rate/gain probably needs to drop from
  UR12e-validated values; `movej --yaw-follow` is the safe fallback if servo isn't
  stable in time.
- Confirm RTDE works at all on this controller's firmware version.
- SSH/log-pull layout may differ from the e-Series controller — low priority today.
- New mount geometry is unknown — `CATCH_Z_MIN` and reach bounds must be surveyed
  and set conservatively, not copied from the UR12e.
- Four scripts hardcode the UR12e's IP — fix before running anything against the
  new arm (see Code changes above).
- Flange/bracket fit: probably compatible (same ISO standard), confirm physically;
  `tcp_offset` must be remeasured regardless.
- Flex 13 coverage for an entirely new volume is an open question, not just a
  reuse-existing-cameras job.
- Payload/CoG wizard must be redone on this controller for this tool — never
  copied automatically, and this exact miss caused a recurring protective stop on
  the current rig.
- `calibrate_frames.py`'s pose sweep needs a temporary hand-picked safe via-pose
  before `DEFAULT_WAIT_POSE` exists for this arm.
