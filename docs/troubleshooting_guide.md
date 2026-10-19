# Troubleshooting & orientation guide

For someone who has never touched this repo before and something isn't working.
Read top to bottom once; after that, use it as a lookup table. This duplicates a
few facts from `CLAUDE.md`/`docs/demo_setup_new_laptop.md` on purpose — this file
is meant to be self-contained enough to paste whole into an AI chat as context
when asking for help.

If you're setting up a **brand-new laptop** from scratch, use
`docs/demo_setup_new_laptop.md` instead — it's the install/network walkthrough.
This file is for "it's installed, something's wrong, what do I try."

## What this project does

Humans throw a ping-pong ball at a Universal Robots arm. OptiTrack cameras
track the ball's 3D position; software fits a trajectory, decides where/when
the ball crosses a fixed "catch plane," and streams the arm to that point fast
enough to catch it in a small cardboard box mounted on the flange. Two separate
systems have to be alive and talking to the laptop at once: **Motive**
(Windows PC, camera tracking) and the **UR10 robot controller**.

```
[Flex 13 cameras] --USB--> [Windows PC: Motive] --NatNet/UDP multicast--> [laptop]
                                                                              |
                                                                       trajectory.py
                                                                       (fit + predict)
                                                                              |
                                                                        catch.py / demo.py
                                                                        (feasibility + move)
                                                                              |
                                                            --URScript/servoj over TCP--> [UR10 controller] --> arm
```

`demo.py` is the one to run for an actual demo session — it's `catch.py` plus
idle-wobble-while-waiting and other demo polish, same core pipeline. If in
doubt which to run, run `demo.py`.

## Confirmed run procedure — this exact laptop/rig

This is the actual, currently-working sequence an operator uses (not a generic
setup walkthrough — the laptop and network are already configured). If you
followed this exactly and it still didn't work, skip to the health-check and
symptom sections below.

**1. OptiTrack (Windows PC):**
- Turn Motive on and confirm you can see the marker rig (the one worn near/on
  the hand) in Motive's 3D view before throwing anything. **Only the taped
  ping-pong ball tracks/throws reliably — a bare or different ball won't.**
- Open Motive's streaming settings and check: **Streaming → NatNet → Enable**
  is on, and **Local Interface shows `192.168.10.1`** — not `127.0.0.1`
  (loopback). This is almost always already on; if it isn't (or shows
  loopback), unplug and replug the Ethernet cable, then restart Motive.
- **Known quirk on this laptop: pinging the Motive PC (`192.168.20.1` from the
  Windows side / `192.168.10.1`) does not work, even when everything is fine
  — the NatNet data still comes through regardless.** Don't treat a failed
  `ping` to the Motive PC as evidence anything is broken; use `live_view.py`
  (see below) as the real test, not `ping`.

**2. Robot (UR pendant):** power the robot on and release the brakes ("put it
into power"). That's it — this controller has no e-Series Remote Control
toggle to flip, and no program needs to be loaded or playing; the arm is
driven by raw scripts sent to it, not a pendant program.

**3. Laptop:**
```bash
# open a terminal: Ctrl+Alt+T
cd ~/MABALLS && source .venv/bin/activate && python3 demo.py
```
Type `go` and press Enter when prompted — the arm should move to the wait
pose and start. In the **same terminal**, later runs just need `python3
demo.py` again (the venv is already active for that shell).

There are many flags for tuning; the two operators reach for most to get more
visible motion are `--servo-max-speed` and `--servo-max-accel` (current
defaults 1.2 m/s / 6.0 m/s² — see the numbers table below for the measured
ceiling before pushing these up).

**To stop:** press Enter. It then prompts for a **session name** and
**description** — both can be left blank by pressing Enter again; they only
label the saved log files from that session, nothing else depends on them.

## The IDs and numbers that matter

These are baked into `demo.py`/`catch.py` as defaults and into the calibration
JSON files. If Motive's asset list doesn't match these, either fix it in
Motive or override on the command line.

| Thing | Value | Where it's used |
|---|---|---|
| **Ball** rigid body ID (Motive) | **3** | `--rigid-body-id` on `demo.py`/`catch.py`/`live_trajectory.py`/`track_ball_servo.py` |
| **Robot base** rigid body ID (Motive) | **7** | baked into `UR10_T_base_from_baseRB.json`, not a CLI flag |
| **TARGET** rigid body ID (Motive) — the prop the arm dumps a caught ball onto | **8** | not a CLI flag |
| Tool/wrist rigid body (used only during calibration, not at runtime) | id `0` in the calibration file's `rigid_body_id` field, but that field means "unlabeled marker," not a real Motive asset id — see `tracking_mode: unlabeled_marker_unlabeled_list` in the transform JSON | `UR10_T_base_from_mocap.json` |
| Motive PC (Windows) | `192.168.10.1` | `--server-ip` |
| Laptop, Motive-side | `192.168.10.2` | `--local-ip` |
| UR10 controller | `192.168.20.1` | `--robot-ip` |
| Laptop, robot-side | `192.168.20.2` | fixed, also where the robot's servo stream dials back to |
| Robot ports | 29999 (Dashboard), 30001-30004 (primary/secondary/realtime/RTDE) | — |
| Servo callback port (robot → laptop) | `30099` | `ur_servo.SERVO_HOST_PORT` |
| NatNet multicast | `239.255.42.99`, data UDP `1511`, command UDP `1510` | Motive Streaming pane |
| Default `tcp_offset` (box centroid along flange Z) | `0.0725` m | read from the transform JSON at runtime, not hardcoded |
| Wait pose (UR10 rig) | `(0.1583, -0.5783, 0.2267, 1.1951, -1.1218, 1.1215)` | `DEFAULT_WAIT_POSE` in `catch.py`/`demo.py` |
| Catch envelope | reach `[0.55, 1.20]` m, z `[0.15, 1.20]` m, azimuth `±75°` of wait pose | `CATCH_MIN_REACH`/`CATCH_MAX_REACH`/`CATCH_Z_MIN`/`CATCH_Z_MAX`/`CATCH_MAX_AZIMUTH_DEG` |
| Default transform files | `UR10_T_base_from_mocap.json` (static) + `UR10_T_base_from_baseRB.json` (live, on by default via `--base-rb-transform`) | `--transform-file` / `--base-rb-transform` |
| Servo defaults | 125 Hz, `--servo-max-speed 1.2`, `--servo-max-accel 6.0` (below the ~1.6 m/s / ~32 m/s² measured hard ceiling on this arm) | `demo.py` |

**There is more than one `T_base_from_*.json` in the repo root** — `T_base_from_mocap.json`,
`T_base_from_mocap_v2.json`, `T_base_from_baseRB.json`, `T_base_from_baseRB_v2.json`
(no `UR10_` prefix) are all leftover calibration from the **old UR12e rig at the old
location** — do not use them. Only `UR10_T_base_from_mocap.json` and
`UR10_T_base_from_baseRB.json` are current. If a script complains about a pose that
looks physically wrong (e.g. "Wait pose is outside the catch envelope"), check which
transform file it actually loaded before assuming the envelope constants are wrong.

## Before touching anything: check what's actually alive

Don't guess — these are all quick, read-only, and safe to run at any time.

```bash
source .venv/bin/activate         # forgetting this is the #1 cause of ModuleNotFoundError

# OptiTrack side
python3 live_view.py --duration 10      # frame rate + rigid body list. Confirms Motive is streaming.

# Robot side
python3 ur_status.py                    # want robotmode 7 (RUNNING), safetymode 1 (NORMAL)
python3 ur_get_pose.py                  # prints current TCP pose - proves RTDE reads work
ping -c2 192.168.20.1                   # robot reachable at all?
nc -zv 192.168.20.1 29999 30002 30004   # dashboard / URScript / RTDE ports open?
```

**Don't use `ping` to check the Motive PC** — on this laptop, pinging
`192.168.10.1` fails even on a perfectly healthy connection, while NatNet data
keeps flowing fine. `live_view.py` is the real test for that side; a failed
ping to the Motive PC alone means nothing here.

If both halves check out independently, run the full pipeline with nothing at
risk: `python3 demo.py --dry-run` (or `catch.py --dry-run`) — full perception
and decision logic, zero robot motion, logs every tick.

## Symptom → cause → fix

| Symptom | Likely cause | Fix |
|---|---|---|
| `ModuleNotFoundError: No module named ...` | venv not active | `source .venv/bin/activate` in the repo root |
| `live_view.py` shows 0 fps / no rigid bodies | Motive's NatNet streaming is off, or Local Interface reverted to loopback (`127.0.0.1`) instead of `192.168.10.1` | In Motive: Streaming → NatNet → Enable, Local Interface = `192.168.10.1`. If it's already set correctly but still not streaming, unplug/replug the Ethernet cable on the Windows PC and restart Motive — this fixes it every time it's been seen. Do **not** diagnose this by pinging the Motive PC — see the ping quirk above |
| Rigid bodies show up but the ball is never detected/tracked | Ball's Motive asset id isn't `3`, or you're not throwing the taped ping-pong ball | Pass `--rigid-body-id N`, or rename the asset in Motive to id 3. Throwing works reliably only with the specific taped ping-pong ball — a bare ball or a different object won't track cleanly |
| Ball tracked but release never detected (arm never reacts) | Throw too slow/short, or occluded at release | Release detection needs release speed ≥3.0 m/s and a lofted arc; make sure cameras cover the full throw start point, not just mid-flight |
| `Never saw a valid reading of base rigid body id=7 within Ns` | Base RB occluded, deleted, or recreated in Motive | Check it's visible in `live_view.py`; **never delete/recreate the base RB asset** — that invalidates `UR10_T_base_from_baseRB.json` and needs a re-run of `calibrate_base_rb.py`. Re-origining the volume or bumping the rig is fine, deleting the asset is not. |
| `Wait pose is outside the catch envelope` | Robot base moved relative to the base RB, or wrong `--transform-file`/`--base-rb-transform` in use, or you're accidentally loading a non-`UR10_`-prefixed transform file | Confirm the printed config dump names `UR10_T_base_from_mocap.json` / `UR10_T_base_from_baseRB.json`; if the rig was physically moved, recalibrate rather than editing the envelope constants |
| `Did not reach the wait pose (timeout)` | Robot not powered, brakes not released, or a fault/dialog blocking the pendant | On the pendant: power on, release brakes, dismiss any dialog/fault. That's the entire robot-side procedure on this controller — no Remote Control toggle to set, no program to load or play |
| RTDE connect hangs / `ur_status.py` never returns | Robot not powered, wrong cable in switch, laptop missing `192.168.20.2` | `ip -brief addr`, `ping 192.168.20.1`, check physical cabling |
| Arm reaches wait pose, then everything freezes — no catches, no faults | Servo stream never opened: robot couldn't dial back to `192.168.20.2:30099` | Check `ufw`/firewall allows inbound TCP 30099; confirm the address is actually on the NIC |
| Protective stop on nearly every catch attempt | Wrong payload/CoG on the pendant (Installation → Payload Estimation) — the #1 root cause seen historically (`C153A0`/`C157A0`) | Set payload/CoG for the box+ball tool on the pendant. This is never set by any script — check it before touching speed/accel parameters |
| Fault clears itself unexpectedly / doesn't clear at all | By design, `catch.py`/`demo.py` never auto-clears a fault — a human must clear it (pendant, or `ur_status.py --clear`) | Clear on the pendant or run `python3 ur_status.py --clear`; the session resumes on its own once safetymode goes back to NORMAL (1) |
| Session dies with an uncaught traceback instead of the normal Ctrl-C/Enter teardown | Real bug, or a fault happened during a servo-reconnect attempt | Check `robot_logs/sessions/<latest>/` if `wrap_up_session()` ran; if it didn't run at all, the session logs weren't pulled — note the approximate wall-clock time and pull `/root/log_history.txt`/`/root/polyscope.log` from the robot manually over SSH (`ssh ur12e`-style alias, read-only) |
| Catch rate seems low / lots of rim hits, not wild misses | Historically normal — best measured rate is ~81% and most misses were rim hits from prediction error, not gross misses | Check throw quality (lofted, >1s flight, roughly toward the box) before assuming code regressed; `--plot` shows commit points vs. actual ball path per throw |
| Late ticks / jittery tracking in `ur_servo.py --bench` | Laptop can't keep up at the configured servo rate, often on battery power | Run on AC power, drop `--no-plot` (biggest single per-tick CPU cost), then try `--servo-rate 100` |
| No audio cues (beep/double-blip/cha-ching) | `aplay`/`paplay` missing | `sudo apt install alsa-utils`; harmless without it, just silent |
| Plot window (`--plot`) doesn't open, matplotlib backend error | Missing Tk | `sudo apt install python3-tk` |
| `TypeError: unsupported operand type(s) for |` in `live_view.py` | Clone predates commit `97eb00a` (needed Python 3.10 syntax) | `git pull` |
| A move looks like it "arrived" but the arm never actually moved (or moved to the wrong place) | Either a genuine settle-detection edge case, or the arm faulted and froze — `getActualTCPSpeed()` alone can't tell the two apart | Any current motion code also checks `getSafetyMode()`; if you're calling raw RTDE yourself, do the same rather than trusting speed-only "arrived" checks |
| Robot commanded to the flange position instead of the box's mouth (undershoots every catch by ~7cm) | `set_tcp()` wasn't (re)sent for this session, or another script left a different TCP offset installed | `catch.py`/`demo.py` sends this every run automatically — if calling lower-level scripts (`track_rigid_body.py`, `track_ball_servo.py`) directly, verify they log the `tcp_offset` they loaded |

## Real UR error codes seen in this project (from `docs/ur_error_codes_en.properties`)

| Code | Meaning | Notes from this project's history |
|---|---|---|
| `C153A0`/`C153A1`/etc. | "Position deviates from path," detected by the named joint (A0=base … A5=wrist3) | Root cause here has usually been wrong payload/CoG on the pendant, or (on the old UR12e rig) a `movel` demanding a joint speed over its limit on a side target — `movej --yaw-follow` avoids the latter class entirely |
| `C157A0`/`C157A1`/etc. | "Collision detected by joint" | Same payload/CoG root cause as above; `A1` also seen from a real physical collision (tool vs. base mounting stand) — see `CATCH_Z_MIN` history in `CLAUDE.md` |
| `C306A3` | "Acceleration failed to pass sanity check" | Seen chasing a throw that was continuously outside the reachable envelope the whole flight — this is what "chase-abort" (margin-trend abort) in `catch.py`/`demo.py` now prevents |

Full explanations for any code: `docs/ur_error_codes_en.properties` (search `CODE_<num>`).
`docs/robot_log_formats.md` explains how to correlate `catch_logs/*.jsonl` timestamps
against the robot's own `/root/log_history.txt`/`/root/polyscope.log` if you need to
dig into a specific incident after the fact.

## Key scripts, what each is actually for

| Script | Purpose |
|---|---|
| `demo.py` | Full demo: perception → prediction → catch, plus idle wobble/polish. **Run this for an actual session.** |
| `catch.py` | Same core pipeline, without the demo polish — useful for isolating whether a problem is in the core loop or in `demo.py`'s extras |
| `live_view.py` | Read-only Motive/NatNet dashboard — frame rate, rigid bodies, markers. First thing to run if OptiTrack seems dead |
| `live_trajectory.py` | Tracks one rigid body (the ball), shows fitted velocity/prediction, no robot involved |
| `visualize_trajectory.py` | matplotlib version of the above, for visually inspecting prediction convergence/quality |
| `ur_status.py` | Robot status (robotmode/safetymode/programState); `--clear` clears a fault and re-releases brakes |
| `ur_get_pose.py` | One-shot current TCP pose + joints, read-only |
| `ur_goto_raw.py` | Send the arm to an absolute/relative pose or joint target, with a safety clamp |
| `jog_ur_raw.py` | Interactive keyboard teleop for manual positioning |
| `ur_servo.py` | The streaming-control layer `catch.py`/`demo.py` build on. `--self-test` (offline), `--bench` (canned sine sweep, no perception) — use this to isolate "is the servo transport OK" from "is the whole catch pipeline OK" |
| `track_ball_servo.py` | Pure pursuit of the ball's current position, no prediction/catch logic — good for checking raw tracking responsiveness without risking a catch attempt |
| `calibrate_frames.py` | Rebuilds the base ↔ mocap transform (only needed after a real physical change, not routine use) |
| `calibrate_base_rb.py` | Rebuilds the base-rigid-body-relative transform (needed if the base RB asset is ever recreated in Motive) |
| `analyze_catch_logs.py`, `plot_catch_sessions.py`, `throw_plot.py` | Offline analysis of recorded `catch_logs/*.jsonl` sessions |

## Safety reminders (do not skip these when debugging live)

- Keep a hand near the E-stop any time the arm is powered and a script is running near it.
- A protective stop is an expected, safe backstop on this arm's tuned safety config —
  **but on this specific UR10 CB3 controller, that assumption was found NOT reliably
  true** (a session ran hard/fast motion with no fault at all, see
  `docs/ur10_migration_roadmap.md` "Real incident" section) — don't assume a stop
  will always catch a mistake here the way it would on the old UR12e.
- Never `kill -9` / SIGTERM a script mid-session — it skips cleanup and can leave the
  robot's realtime thread stranded, causing a fault on the *next* run. Always stop
  with Enter (clean) or Ctrl-C (abort) — both are handled the same way by `catch.py`/`demo.py`.
- A detected fault does not auto-clear. Clearing it is a human decision — do it on the
  pendant or with `ur_status.py --clear` once it's actually safe to resume.
- Don't `git push` unless explicitly asked to in that conversation, and don't delete/
  modify anything on the robot controller over SSH unless the task specifically calls
  for it (it's a shared, real controller).

## If you're still stuck

1. Re-run the two independent health checks above (Motive side, robot side) and
   confirm which one is actually failing — most reported "it doesn't work" issues
   are one side being unreachable, not a logic bug.
2. `--dry-run` the full pipeline to separate "perception+decision" problems from
   "robot motion" problems.
3. Check `catch_logs/*.jsonl` (if `--record` was on) for the actual tick-by-tick
   decisions of the last session — `jq 'select(.throw==N)' catch_logs/<file>.jsonl`.
4. Check `docs/debug_log.md` — chronological history of every bug found on this
   project so far, dated, with root causes. Search it by symptom before assuming
   something is new.
5. Check `docs/ur10_migration_roadmap.md` if the issue looks specific to this
   arm/location vs. general pipeline logic.
6. If pasting this whole file to an AI for help, also paste: the exact command you
   ran, the full error/traceback, and the output of `python3 ur_status.py` and
   `python3 live_view.py --duration 5` from the same session.
