# Running `demo.py` on a fresh Ubuntu 22 laptop

Everything needed to take a clean Ubuntu 22.04 machine, plug it into the switch, and
run the ball-catching demo — written so someone other than the author can set it up
and show the demo.

Rough time: **1–2 hours** the first time, most of it network + physical checks.
Software install is ~15 minutes and has no compile steps.

Scope note: this covers the **UR10 (CB3) rig at the new location**, which is what
`demo.py`'s defaults point at. SSH to the robot controller is **not** used and not
needed (`--pull-robot-logs` is off by default there).

---

## 0. What you need

- Ubuntu 22.04 laptop. **One Ethernet port is enough** — everything goes through the
  switch.
- The Ethernet switch + 3 cables: laptop, Motive PC, UR10 control box.
- Windows PC running Motive, cameras calibrated, with these assets in the scene
  (the ids matter — they are baked into `demo.py`'s defaults and the calibration files):
  - **ball** rigid body → id `3`
  - **robot base** rigid body → id `7`
  - **TARGET** rigid body (the prop the arm dumps the ball onto) → id `8`
- UR10 CB3 + teach pendant, cardboard-box tool mounted, taped ping pong ball.
- The E-stop, in hand, for the whole session.

---

## 1. Install the software (~15 min)

```bash
sudo apt update
sudo apt install -y git python3-venv python3-pip python3-tk alsa-utils netcat-openbsd

git clone https://github.com/zupermiez/MABALLS.git ~/MABALLS
cd ~/MABALLS

python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

Why those apt packages: `python3-tk` is required for the live throw-plot window
(matplotlib TkAgg — the only backend confirmed working in this project), and
`alsa-utils` gives `aplay` for the audio cues in `beep.py` (it looks for `paplay` or
`aplay` and stays silent if neither exists).

Nothing compiles: `ur_rtde==1.6.3` ships a prebuilt cp310 wheel, which is exactly
Ubuntu 22.04's system Python (3.10). All project code is 3.10-compatible.

**Verify, no hardware needed:**

```bash
python3 -c "import natnet, rtde_receive, dashboard_client, matplotlib, numpy; print('deps ok')"
python3 trajectory.py        # offline self-test of the trajectory fit
python3 ur_servo.py --self-test   # offline math/encoding checks, no robot
```

Remember: **every session starts with `source .venv/bin/activate`** in the repo
directory. If a script dies with `ModuleNotFoundError`, that's almost always the
missing step.

---

## 2. Network (~10 min — this is the part that actually bites)

One flat wire through the switch carrying **two subnets**. The laptop holds an
address in each, on the same interface:

| Device | Address |
|---|---|
| Motive PC (Windows) | `192.168.10.1` |
| **Laptop** | `192.168.10.2/24` **and** `192.168.20.2/24` (same NIC) |
| UR10 control box | `192.168.20.1` |

All three of those are hardcoded defaults in the code. `192.168.20.2` matters twice:
it's how you reach the robot, and it's the address the robot **dials back to** to open
the servo setpoint stream (`ur_servo.HOST_IP`, port `30099`).

Create one NetworkManager profile with both addresses:

```bash
IFACE=$(ip -o link show | awk -F': ' '$2 ~ /^en/ {print $2; exit}')
echo "using interface: $IFACE"

sudo nmcli con add type ethernet ifname "$IFACE" con-name robobaseball \
  ipv4.method manual \
  ipv4.addresses 192.168.10.2/24,192.168.20.2/24 \
  ipv4.never-default yes \
  ipv6.method ignore
sudo nmcli con up robobaseball
```

`ipv4.never-default yes` keeps Wi-Fi as the default route, so plugging in the switch
doesn't take your internet away.

**Verify (all three must pass before going further):**

```bash
ip -brief addr            # the interface should list BOTH 192.168.10.2 and 192.168.20.2
ping -c2 192.168.10.1     # Motive PC
ping -c2 192.168.20.1     # robot
nc -zv 192.168.20.1 29999 30002 30004   # dashboard / URScript / RTDE ports
```

Firewall: `sudo ufw status` should say inactive (Ubuntu's default). If it is active,
allow inbound **UDP 1510 and 1511** (NatNet multicast from Motive) and **TCP 30099**
(the robot's callback for the servo stream) — otherwise the mocap stream is silent and
servo mode never starts.

---

## 3. Motive PC checklist (no changes needed if nobody touched it)

Motive → Streaming pane: **Enable**, Local Interface `192.168.10.1`, Transmission Type
**Multicast** (`239.255.42.99`, data port 1511, command port 1510). Windows Firewall:
inbound UDP 1510/1511 allowed, network profile Private.

**Verify from the laptop:**

```bash
python3 live_view.py --duration 10
```

You should see a live frame rate and the rigid bodies listed. If the ball's id is not
`3`, or base is not `7`, or TARGET is not `8`, either fix the ids in Motive or pass
`--rigid-body-id N` to `demo.py` (ball only — base and TARGET ids are not CLI flags).

**Do not delete and recreate the base rigid body asset in Motive.** The committed
calibration (`UR10_T_base_from_baseRB.json`) describes the robot base *relative to that
asset's own frame*; recreating the asset invalidates it and requires re-running
`calibrate_base_rb.py`. Re-calibrating the camera volume / moving the robot is fine —
the transform is resolved live from the base rigid body every run, precisely so the rig
can be bumped or re-origined without a resweep.

---

## 4. Robot checklist

On the pendant:

1. Power on, **release brakes**. That's enough — CB3 has no e-Series "Remote Control"
   toggle, and this project drives the arm by sending raw URScript to port 30002, so no
   program needs to be loaded or playing.
2. Confirm **payload and CoG** are set for the box tool (Installation → Payload). A
   wrong payload is the known root cause of recurring `C153A0`/`C157A0` protective
   stops in this project — it is not something `demo.py` sets.
3. Clear the space around the arm. Keep the E-stop in hand.

**Verify from the laptop:**

```bash
python3 ur_status.py     # want robotmode 7 (RUNNING) and safetymode 1 (NORMAL)
python3 ur_get_pose.py   # prints the current TCP pose — proves RTDE reads work
```

If safetymode isn't 1, clear the fault on the pendant (or `python3 ur_status.py --clear`).

---

## 5. Optional one-time check: can this laptop hold the servo rate

`demo.py` streams setpoints at 125 Hz. A slower laptop is the one thing a machine swap
can genuinely regress, and it shows up as late ticks:

```bash
python3 ur_servo.py --bench
```

Healthy looks like the validated reference run: a few thousand setpoints, **0 late
ticks**, a few mm of lag. Run it with the area clear — this moves the arm through a
canned sine sweep.

---

## 6. Run the demo

```bash
cd ~/MABALLS
source .venv/bin/activate
python3 demo.py --no-wrapup
```

`--no-wrapup` skips the end-of-session name/description prompt — nice for a guest
operator. Drop it if you want the session notes written to `robot_logs/sessions/`.

What you'll see, in order:

1. A config dump: transform source, wait pose, catch envelope, servo stream settings.
2. The live throw-plot window opens (ball path, arm path, commit points — one update
   per throw). Closing it does **not** stop the session.
3. A prompt: **type `go`** to arm motion (anything else aborts).
4. The arm does one `movej` to the wait pose, then brings up the continuous servo
   stream and holds that pose — it stays under live servo control for the whole
   session, not just during a catch.
5. `at wait pose. Ready - throw the ball.`

Then just throw. Lofted throws, roughly toward the box, flight time comfortably over a
second. Audio cues tell you what it decided (a single beep = it sees a trajectory,
double blip = re-aiming, "cha-ching" = it thinks it caught the ball). After a caught
ball, the arm rotates to the **TARGET** prop and tips the ball out, then returns to the
wait pose.

**Stopping:** press **Enter** for a clean stop, **Ctrl-C** for an emergency abort —
both stop the arm and tear down the same way. Never `kill -9` it.

If the arm protective-stops, the session does **not** die: it waits for a human to
clear the fault on the pendant (or `ur_status.py --clear` from another terminal), then
drives back to the wait pose and resumes on its own.

Useful overrides:

| Flag | Effect |
|---|---|
| `--dry-run` | Full pipeline, zero motion. Good for checking mocap + gating without the arm. |
| `--yes` | Skip the `go` prompt. |
| `--no-plot` | No plot window (slightly less CPU). |
| `--no-record` | Don't write `catch_logs/*.jsonl`. |
| `--catch-move movej --yaw-follow` | The older joint-space path; fall back here if servo mode misbehaves. |
| `--rigid-body-id N` | If the ball's Motive id isn't 3. |

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `ModuleNotFoundError` | Forgot `source .venv/bin/activate`. |
| Plot window never appears, matplotlib backend error | `sudo apt install python3-tk`. |
| No sound cues | `sudo apt install alsa-utils` (harmless without it). |
| `live_view.py` shows 0 fps / no frames | Wrong subnet or Motive streaming off. Check `ping 192.168.10.1`, Motive's Streaming pane, Windows firewall, and that the laptop really has `192.168.10.2`. |
| Rigid bodies listed but ball never detected | Ball id isn't 3 → `--rigid-body-id N`; or the tape wrap is too wrinkled/small for Motive's marker threshold. |
| `Never saw a valid reading of base rigid body id=7` | The base rigid body isn't tracked (occluded markers, or the asset was renamed/recreated in Motive). |
| RTDE connect fails / `ur_status.py` hangs | Robot not powered, wrong cable in the switch, or laptop missing `192.168.20.2`. |
| Arm moves to wait pose then everything freezes, no catches | Servo stream never opened — the robot couldn't reach `192.168.20.2:30099`. Check ufw and that the address is on the NIC. |
| `Wait pose is outside the catch envelope` | The robot base moved relative to the base rigid body, or the wrong transform file is in use. Needs re-calibration, not a flag. |
| Protective stop on almost every catch | Check the pendant's payload/CoG setting first (section 4.2) before touching speeds. |

---

## 8. Don't change these without reading first

- The catch envelope constants in `demo.py` (`CATCH_MIN_REACH`, `CATCH_Z_MIN`, …).
  Each number exists because of a real collision or protective stop — see CLAUDE.md's
  "Key safety rules".
- `--servo-max-speed` / `--servo-max-accel` (0.8 / 4.0). Deliberately below this arm's
  measured ceiling.
- The wait pose. It is this rig's surveyed, non-singular, collision-checked pose.

For the reasoning behind any current default, see `docs/debug_log.md` and
`docs/ur10_migration_roadmap.md`.
