# Architecture review: the whole catch pipeline, ordered by lowest-hanging fruit

*2026-07-17 night shift. Every number in here is measured — from the 23 real sessions
of 2026-07-16 (196 throws, 108 commits, 33 faults, `catch_logs/`), the robot's own
flight-report telemetry, or offline replay of the logged trajectories. Analysis
detail: `docs/debug_log.md` 2026-07-17.*

## Where the system actually is

It works. ~91% of committed throws were caught (67/74 classifiable by the
last-seen-near-tool heuristic, consistent with the session notes). The three real
problems, in order of demo damage:

1. **Protective stops on side throws** (demo killers — each needs a human at the
   pendant). Root-caused: not payload (that fixed the straight-ahead ones), but
   `movel`'s Cartesian line demanding base-joint speed `v/reach` = 137–195°/s
   against a 120°/s limit. Fault rate vs target azimuth swing (accel≤4): 0% <10°,
   36% at 20–35°, 50% >35°.
2. **Commit targets are noisy.** Commits fire at n≈43 samples (correct decision —
   earliest = most budget) but the *target* at n=43 has 6.4cm median / 17cm p90
   error, vs 1.6/2.8cm at n=80. The stability-averaging that was supposed to help
   was used 0/108 times (it can't be full on the first eligible tick). The wide box
   forgave this; anything smaller won't.
3. **False releases near the robot** (the "grabbed the ball out of the box →
   self-collision" incident). Release detection is kinematic only; it has no idea
   whether the "throw" originated from a hand 0.5m from the base.

Everything below is ordered by value ÷ effort.

---

## Tier 0 — implemented tonight, just needs shakedown (effort: run a session)

All offline-validated against the logged real throws; **none run on the real arm
yet**. Defaults chosen so the first run behaves like yesterday plus safety:

| Change | What it buys | On by default? |
|---|---|---|
| Release guard (`check_release_guard`) | Blocks the self-collision class. Replayed on all 161 logged throws: blocks exactly the 8 hand/away events, passes every real throw | yes |
| Envelope azimuth band (±75° of wait pose) | Backstop: targets behind/beside the corridor structurally refused (the 158° self-collision target now fails by unit test) | yes |
| Post-commit re-aim | Repairs the noisy n=43 target from rest once n≈60–80 data exists (median 4.4cm / p90 10.7cm correction, median 0.42s budget remaining at n=67). Never preempts a moving arm | yes (`--no-reaim`) |
| `--poll-hz` 50 (was 20) | 15–40ms less decision latency ≈ 2–5cm of arm travel, free | yes |
| Live catch/miss tally + `caught_guess` in JSONL | Instant feedback per throw + session score in notes.txt (also a demo scoreboard data source) | yes |
| Faulted throws now logged (`throw_end`/`throw_samples` post-fault) | The class of throw you most need forensics on finally has data | yes |
| `--catch-move movej` + `--yaw-follow` | See Tier 1 | **no** — test deliberately |

Suggested first command (behaves like yesterday's best config + the new safety):

```
python3 catch.py --record --accel 4.0 --speed 1.2
```

## Tier 1 — one robot session of testing, highest payoff

**1. `--catch-move movej --yaw-follow` — the protective-stop fix.**
A movej plans in joint space and *cannot* violate joint limits; `--yaw-follow`
rotates the box's (free) vertical-axis orientation with the target azimuth so the
wrist stops fighting the sweep. Expect side throws at 20–35° swing to go from ~36%
fault to ~0, and to arrive *faster* (base joint at full 115°/s instead of a scaled
Cartesian crawl). Test plan: park at wait pose, throw deliberately to the side at
increasing angles, compare faults + arrival vs movel. If it holds, make it the
default and note the movel numbers in the log. Note the feasibility model still
assumes movel timing (conservative for azimuth sweeps — fine).

**2. Re-teach the wait pose at the data's center of mass.**
Current wait reach is 0.48m; the geometric median of all 108 real commit targets is
`(0.042, -0.716, 0.139)` — reach 0.72m. Waiting there cuts the median catch move
from 0.311m to 0.190m, i.e. **~113ms less move time on every throw** — pure margin,
converts "possible" commits into full catches. Try
`--wait-pose 0.042 -0.716 0.139 1.584 -0.0824 -0.0573` (same taught orientation),
and check the joints there with `ur_get_pose.py` first (keep wrist2 well off 0°,
same reason the pose was re-taught on 2026-07-15).

**3. Faster return-to-wait.**
`--approach-speed 0.5` rad/s (29°/s) makes the between-throws reset sluggish — it
caps demo cadence and audience energy. 1.0–1.2 rad/s is still ≤ half the joint
limit. Pure flag test, no code.

## Tier 2 — a day of work, transforms what the demo can claim

**4. Recalibrate `T_base←mocap` properly (27.7mm RMSE → target <10mm).**
The calibration error is now the *largest single term* in the error budget (bigger
than the n=80 prediction error). Likely dominated by the TCP marker not sitting
exactly on the TCP point. Do: (a) verify/measure `tcp_offset` by a pivot test
(rotate the wrist about a fixed tool tip; the marker should not translate — solve
the offset that minimizes its motion), (b) 40+ poses spanning the *actual catch
envelope* (not a generic cube), (c) sanity-check by commanding the tool to a
marker on the floor. Published mocap↔robot rigs hit <1.5mm; even 8–10mm here
unlocks Tier 3's smaller vessels.

**5. Error budget → shrink the catch vessel in public steps.**
Current per-catch lateral error stack (RSS, p90-ish): calibration 28mm +
commit-target error 169mm (n=43, no re-aim) + arm settle ~0 ≈ needs the 150mm box
radius. With re-aim (n≈80: 28mm + 28mm ≈ 40mm) a **12–16cm-diameter bowl** is
already defensible; with recalibration (10mm + 28mm ≈ 30mm) a **coffee-cup-class
(9–10cm) target** becomes a realistic hero shot on lofted throws. Demo-wise this is
the cheapest "impossible-looking" upgrade there is: same system, visibly tiny
target. Stage it: box → bowl → cup, and let the audience see the swap.

**6. `servo_track.py` — servoj streaming over port 30003 (the v2 unlock).**
Re-aim (Tier 0) is discrete corrections from rest; servoj at 125–500Hz makes the
correction *continuous*: commit early exactly as now, then glide the setpoint as
the fit refines — no program restarts, no rest requirement, buttery on camera, and
it retires the movel side-throw problem a second way (stream small deltas). Known
pitfalls to design for (CLAUDE.md refs): steady send cadence, lookahead/gain
tuning, buffer growth under speed scaling. Prototype standalone against a slow
moving target first (`track_rigid_body.py` is the harness pattern). This is the
single biggest *architectural* step left; everything else is parameter- or
bolt-on-level.

## Tier 3 — the showbiz tier (mostly free, given the above)

The system's actual superpower is **precognition**: it knows where the ball will
land ~0.5s before it does, and the arm is *already waiting* when the ball arrives.
Waiting is not a limitation to hide — it's the most impressive-looking thing the
system does. Sell it:

- **Audience prediction screen (near-free, biggest wow-per-hour).**
  `visualize_trajectory.py` already draws live trajectory + frozen prediction +
  confidence flash. Point it at a projector/TV behind the robot: audience sees the
  predicted landing point lock in *while the ball is still rising*, then watches
  the arm be there. Add the committed target + a "LOCKED" flash on `commit` (data
  is already in the tick stream). This makes the invisible intelligence legible —
  the difference between "robot moved" and "robot *knew*".
- **Scoreboard/streak** — the tally is now computed live; put `CATCHES: 7/8` on
  that same screen. Crowds count streaks.
- **Commit sound cue** — one `aplay` beep on commit ("lock-on"), a second tone on
  catch. Trivial, disproportionate effect: the audience *hears* the decision being
  made mid-flight, before the catch proves it.
- **Audience throws.** The release guard + azimuth band + envelope now make
  stranger throws structurally safe(r): bad throws get *refused*, not chased. A
  refusal is itself a demo beat — show "TOO FAST / OUT OF REACH" on screen when the
  gate says no. The robot visibly declining an impossible ball reads as judgment,
  not failure.
- **Choreograph the reset.** After a catch: slow tilt of the box toward the
  audience (show the ball), then a movej to a "dump" pose over a return chute/ramp
  that rolls the ball back to the thrower, then back to wait. ~20 lines using the
  existing primitives, turns each catch into a complete little scene and solves
  ball-retrieval (which also caused the self-collision incident — nobody reaches
  into the workspace anymore).
- **Call-your-shot variant** (needs Tier 2 #4/#5): tape three colored zones on the
  floor plane... skip — better: three differently-sized vessels swapped on the
  tool; "now the cup." Size escalation is the cleanest impressiveness narrative.

### The Robotiq 2F-85: what it can and cannot do here

Numbers first: 85mm stroke, closing speed 20–150 mm/s ⇒ full close ≈ **0.57s**;
even the last 20mm of travel costs ~130ms. A tennis ball (67mm, arriving at 4–6
m/s) transits the finger plane in ~15ms. Timing a mid-air pinch means starting the
close so the gap passes through ~70mm exactly at ball arrival: the tolerance
window is (85−70)mm ÷ 150mm/s ≈ **100ms**, while late-flight arrival-time
prediction is good to ~10–30ms. So a mid-air snatch is *marginally* physically
possible but the failure mode is a ball ricocheting off closing steel fingers —
low percentage, and a miss looks bad. **Do not make it the primary catch.**

What the gripper IS worth mounting for:

1. **Catch-then-grip theater (guaranteed win).** Gripper fingers hold a shallow
   soft cone/funnel (3D-printed, fingers as the funnel's ribs, or funnel between
   the fingers). Ball lands in the funnel exactly as today → gripper closes 20mm
   → ball is *held*. Robot turns, presents the ball to the audience, drops it in
   the return chute. Every element already works; the close happens after the
   catch, so its 0.57s is irrelevant. To the audience, "the robot caught it in a
   box" just became "the robot caught it and held it up."
2. **Shrinking the target with fingers open** — fingers-open + small cup insert is
   itself a small catch vessel (Tier 3 size-escalation prop).
3. **Hero finale (optional):** the timed mid-air pinch as an explicitly framed
   "one in five" stunt after the reliable set — a miss is drama, a hit is legend.
   Only worth attempting after servoj (Tier 2 #6) tightens arrival timing.

Mind the payload math when mounting it: 2F-85 ≈ 0.9kg + coupling + tool — redo the
pendant Payload Estimation wizard (a wrong CoG is the proven C153A0 trigger).

---

## Full pipeline walkthrough (state, verdict, residual risk)

1. **Cameras / Motive (8× Flex 13 @120Hz).** Healthy; 8 cameras is generous for
   one corridor + catch zone. Checks worth 10 minutes: rigid-body *smoothing must
   be 0* for the ball asset (Motive smoothing = hidden latency; it defaults on for
   some asset types), exposure short enough for a 9 m/s ball (no blur-stretched
   centroids), and camera placement biased so ≥3 cameras see the *first 0.4s* of
   flight — release-side coverage is worth more than catch-side (commit happens at
   n≈43; the catch zone only needs enough coverage to not lose the ball early).
   The occlusion-on-catch "problem" is now a feature (it's the catch detector).
2. **NatNet ingest.** Multicast, dedicated link, library-parsed — correct and
   boring. Callback stays skinny (existing threading rule). No changes.
3. **Release detection.** Tuned and battle-tested (bounce guards etc.). Its one
   blind spot — *who/where* threw — is now covered by the release guard. No
   further work until a real false-negative shows up.
4. **Trajectory fit.** Free per-axis quadratic on the full buffer. **Measured
   optimal — leave it alone.** Gravity-locked and empirical-g-locked variants both
   lose to it on real throws (bias > variance saved, already at n=40); drag
   modeling is unnecessary because the free fit absorbs it. The real lever was
   never the fitter, it's *when you re-consult it* (re-aim / servoj).
5. **Intercept solve (plane crossing).** Closed-form, correct. A future refinement
   once catches move off a single plane: solve against the *tool-mouth disc* in 3D
   rather than an infinite plane — only matters for cup-class targets.
6. **Frame transform + TCP offset.** Umeyama transform is the right method; its
   27.7mm RMSE is now the top error term → Tier 2 #4. `set_tcp` re-send every run
   is correct and stays (real bug class, twice).
7. **Feasibility gate + commit rule.** First-qualifying-tick commit is
   data-vindicated (55% commit rate, and the 2026-07-15 analysis already showed
   stability-gating threw away the best window). Move-time model residual ~55ms is
   inside `--margin`+box tolerance. With the wait pose moved (Tier 1 #2) expect
   the possible→catch verdict mix to improve on its own.
8. **Motion primitive.** movel: proven but geometrically fault-prone sideways →
   movej/yaw-follow (Tier 1 #1) → servoj (Tier 2 #6). That's the whole motion
   roadmap in one line.
9. **Safety/recovery.** Envelope (now with azimuth band) + guard + dashboard-based
   fault detect + wait-for-human-clear is a sound demo posture. Residual gap:
   `move_to`'s settle detection can't distinguish "arrived" from "stalled at low
   speed without fault" — acceptable, backstopped by the timeout.
10. **Logging/forensics.** JSONL + robot-clock correlation + auto-pulled robot logs
    + (now) faulted-throw trajectories + catch tally. This layer is genuinely
    excellent — it's why tonight's entire analysis was possible without touching
    Motive. Keep feeding it.

## Do-NOT list (measured dead ends)

- Don't constrain the fit to gravity (or to empirical g) — measured worse.
- Don't gate commits on prediction stability — re-litigated 2026-07-15, stands.
- Don't fight side-throw stops with lower `--accel` — it's a velocity/geometry
  problem; 3.0 already cost catches and still faulted at 20°+ swings.
- Don't resurrect `ur_rtde` control for v2 — servoj over raw 30003 needs nothing
  from it.
- Don't attempt mid-air gripper pinches as the main act (math above).

## Suggested morning order

1. `git diff` tonight's changes (uncommitted on top of `6228f18`), skim, commit if
   happy.
2. Real session, yesterday's config + new defaults: `--record --accel 4.0 --speed 1.2`
   — confirm guard/re-aim/tally behave, watch for one full-session zero-fault run.
3. Same session, second half: add `--catch-move movej --yaw-follow`, throw
   deliberately sideways, compare.
4. Try the re-taught wait pose (`ur_get_pose.py` joint check first).
5. If all green: bump `--approach-speed`, then start Tier 2.
