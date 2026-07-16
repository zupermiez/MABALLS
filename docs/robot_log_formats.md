# Robot log formats - reference

Evergreen reference material (unlike `debug_log.md`, which is a chronological
incident history) for the log/telemetry formats this project pulls off the UR
controller and writes into `catch_logs/*.jsonl` / `robot_logs/sessions/<...>/`.
Consult this when correlating an incident across files instead of re-deriving it
from scratch - see `docs/debug_log.md` 2026-07-16 for the investigation that made
clear this was needed (a wall-clock<->robot-clock offset had to be reverse-engineered
by hand because nothing recorded both at once).

## Clocks - three of them, now two are directly correlatable

| Clock | Where it appears | Notes |
|---|---|---|
| **wall** | `catch_log*.jsonl`'s `wall` field, `log_history.txt` field 3, `polyscope.log`'s leading timestamp | Real wall-clock (`time.time()` / local system clock). The one a human reads. |
| **robot_clock** | `catch_log*.jsonl`'s `robot_clock` field (added 2026-07-16), `realtimedata.csv`'s `Timestamp [s]` column | UR controller's own "time elapsed since the controller was started" counter (`rtde_receive.getTimestamp()`). **Same source as the CSV column** - a JSONL line's `robot_clock` can be matched directly to a `realtimedata.csv` row with no conversion. Sessions recorded before 2026-07-16 don't have this field; for those, derive the offset empirically (find one event visible in both, e.g. a `Safety mode` transition vs. a `fault` event, and subtract) - see `docs/debug_log.md` 2026-07-16 for a worked example. |
| **t** (NatNet) | `catch_log*.jsonl`'s `t` field | Motive/NatNet sample timestamp - the clock a Motive *replay* of the session re-streams verbatim (confirmed 2026-07-15, see `docs/debug_log.md`). Unrelated to the other two; only useful against a Motive replay of the same take, or as a `throw`-ordinal cross-check. |

**Rule of thumb**: correlating `catch_log*.jsonl` against a flight report's
`realtimedata.csv`? Use `robot_clock`/`Timestamp [s]` directly (exact, no math).
Correlating against `log_history.txt`/`polyscope.log`? Use `wall`. Correlating
against a Motive replay? Use `t`.

## `log_history.txt` - the robot's own machine log (`/root/log_history.txt`)

`::`-delimited lines. Empirically observed structure (not from UR's own docs - UR
doesn't publish this format, so treat field names below as best-effort, not
authoritative):

```
3.5 :: 0001d21h13m04.729s :: 2026-07-16 12:55:58.702 :: -3 :: C0A0:7 :: null :: 1 :: prog :: Program prog started :: null
3.5 :: 0001d21h13m05.075s :: 2026-07-16 12:55:59.001 :: -3 :: C153A0:6 :: null :: 2 ::  ::  :: 0
```

| # | Field | Confidence | Notes |
|---|---|---|---|
| 1 | version-ish (`3.5`) | low | constant across a session; probably a log-format/firmware tag |
| 2 | controller uptime (`0001d21h13m04.729s`) | high | days/hours/minutes/seconds since controller boot - **not** the same clock as `robot_clock`/`Timestamp [s]` above (that one resets on controller restart of the real-time process, this is boot uptime) |
| 3 | **wall-clock timestamp** | high | use this one for correlation - see Clocks table |
| 4 | severity (`-3`, `-2`, ...) | medium | `-3` seen on routine program-state and fault lines; `-2` seen on what looks like a state-cleared/recovery line (e.g. after a fault clears) |
| 5 | **`CODE:subcode`** | high | decode the `CODE_NNN` part via `docs/ur_error_codes_en.properties` (e.g. `grep CODE_153 docs/ur_error_codes_en.properties`); `Cxxx` lines with no matching real fault code (e.g. `C0A0`) are routine program-state transitions, not errors - confirmed by cross-referencing against `polyscope.log`'s plain-English lines at the same wall time |
| 6-10 | assorted params/source/message | varies | when field 8 is `prog` and field 9 is a plain-English message (`Program prog started`/`paused`/`stopped`), it's a routine program-state change, not a fault - these fire on every `send_script()` call this project makes (each is a fresh URScript program named `prog`, see `stopl_script()`/`movel_absolute_script()`/`movej_to_pose_script()`) |

For an actual fault, cross-reference the same wall-clock timestamp in
`polyscope.log`, which has the human-readable equivalent (e.g. `Flight reporter
triggered, safety mode: PROTECTIVE_STOP`) and is much less ambiguous to grep.

## `realtimedata.csv` (inside each `recording*.zip` flight report)

**Never `Read()` this file directly** - it's ~15000 rows x ~140 columns
(~25-30MB) per incident. Always slice it with a script (pandas/csv module,
row-indexed or column-filtered) and only pull the specific rows/columns you need
into context.

- **500 Hz** (2ms between rows), auto-captured by UR as a ring buffer around every
  incident. Empirically measured shape (consistent across 4 samples in the
  2026-07-16 investigation): **~24.9s of data before the trigger, ~5.1s after** -
  the `Safety mode` column transition (see below) is not at the start or end of the
  file, it's ~83% through it.
- The **trigger row** is the first row where the `Safety mode` column changes away
  from `1.0` (NORMAL). Find it by scanning for the first transition, not by
  assuming it's at a fixed row index (a prior fault clearing mid-file, e.g. a
  `3.0 -> 1.0` transition right at the start of a recording that begins moments
  after the previous fault was cleared, can appear before the real trigger).
- Columns worth knowing for a fault post-mortem (exact header names, case-sensitive):
  - `Timestamp [s]` - robot_clock, see Clocks table above.
  - `Safety mode` - enum: `1`=NORMAL, `3`=PROTECTIVE_STOP (others exist - REDUCED,
    RECOVERY, SAFEGUARD_STOP, *_EMERGENCY_STOP, VIOLATION, FAULT - see `ur_rtde`'s
    `getSafetyMode()` docs for the full list).
  - `Actual position j0..j5 [rad]` / `Target position j0..j5 [rad]` - the gap
    between these is the following/path-deviation error that trips codes like 153.
  - `Actual velocity j0..j5 [rad/s]`, `Actual current j0..j5 [A]` - a clean,
    monotonic accel/decel ramp with proportionate current draw indicates a genuine
    commanded move; a step-change spike in current with no matching velocity
    change would indicate a real physical impact (not seen in the 2026-07-16
    incidents - all four showed the clean-ramp signature).
  - `Actual TCP pose x/y/z [m]` / `Target TCP pose x/y/z [m]` - target pose is the
    *instantaneous interpolated setpoint*, not the move's final destination; only
    useful for direction-of-travel / reach at a given row, not "where was it headed."
  - `Actual TCP velocity x/y/z [m/s]` - use `sqrt(vx^2+vy^2+vz^2)` for a scalar
    TCP speed; useful for finding a move's onset (first row after a run of ~0) and
    peak speed reached before a trip.
  - `Program state` / `Script control line` - change together whenever a new
    `send_script()`-sent program starts executing; useful for confirming a motion
    burst in the telemetry lines up with a specific script send.
  - `TCP force x/y/z [N]` - a *virtual*, model-estimated force (no physical F/T
    sensor on this rig), derived from the controller's joint torque + payload/CoG
    model - so it's only as accurate as that model. Large constant offsets here are
    consistent with a wrong payload/CoG (gravity component reads wrong), not
    necessarily an external push.

## `flight_reports.json` (added 2026-07-16, per `robot_logs/sessions/<...>/`)

Written by `build_flight_report_manifest()` in `catch.py` at the end of a
`--record` session with faults. One entry per pulled `recording*.zip`, with its
parsed trigger time and (if within `FLIGHT_REPORT_MATCH_TOLERANCE_S`, currently
30s) the nearest `catch_log*.jsonl` `"fault"` event - `fault_count`,
`throw_at_detection` (the throw counter at DETECTION time - see the caveat below),
`detected_wall`, `gap_to_trigger_s`. Read this first when investigating a session
with faults; it answers "which zip goes with which JSONL fault line" without
eyeballing timestamps.

## `catch_log*.jsonl` fault-event caveat

A `"fault"` event's `throw` field is whichever throw was in progress when
`check_safety_mode()` detected the fault - **not necessarily the throw that
caused it**, if detection lagged the real trigger (see `LastNormal` in `catch.py`
and `docs/debug_log.md` 2026-07-16, where this lag was ~9s on every incident in one
session, long enough to land on the next throw entirely). Each `fault` event also
carries `detected_wall`, `last_known_normal_wall`, and `max_undetected_s` - the
fault actually happened sometime in `(last_known_normal_wall, detected_wall]`, and
that whole window (not just the `throw` number) is what should be checked against
`realtimedata.csv`/`log_history.txt` when root-causing.
