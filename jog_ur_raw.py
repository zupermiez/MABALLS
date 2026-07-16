import os
import select
import socket
import sys
import termios
import time
import tty

ROBOT_IP = "192.168.20.1"
SECONDARY_PORT = 30002   # secondary client interface - accepts URScript program text directly,
                          # no ur_rtde / External Control URCap involved (see CLAUDE.md
                          # "Raw URScript-over-socket" section - this is the one control
                          # path confirmed to produce real motion on this robot so far)

LINEAR_SPEED = 0.05    # m/s, translation jog speed
ANGULAR_SPEED = 0.2    # rad/s, rotation jog speed
ACCELERATION = 0.5     # m/s^2 (and rad/s^2) - speedl ramp rate
STOP_DECEL = 1.0       # m/s^2 - deceleration rate for the explicit stop at the end of
                        # every burst, and whenever keys are released
NUDGE_DURATION = 0.2   # seconds each speedl burst commands before this script calls
                        # stopl() itself - deliberately self-stopping so the robot never
                        # keeps moving after a single script finishes, even if this
                        # Python process dies mid-jog (see robot_control_testing_safety
                        # memory: never rely on an external process to be the only thing
                        # standing between "held key" and "robot decelerates")

# Same repeat-key-detection approach as jog_ur.py: terminals only report keydown, so
# "still held" is inferred from how recently a repeat/press arrived.
HOLD_TIMEOUT = 0.3

KEY_MAP = {
    "d": (0, +1), "a": (0, -1),   # X
    "w": (1, +1), "s": (1, -1),   # Y
    "r": (2, +1), "f": (2, -1),   # Z
    "l": (3, +1), "j": (3, -1),   # RX
    "i": (4, +1), "k": (4, -1),   # RY
    "o": (5, +1), "u": (5, -1),   # RZ
}
AXIS_NAMES = ["X", "Y", "Z", "RX", "RY", "RZ"]
QUIT_KEYS = {"q", "\x1b"}   # q or Esc
STOP_KEY = " "

INSTRUCTIONS = """
UR12e keyboard jog - raw URScript-over-socket (no ur_rtde)
------------------------------------------------------------
Translation:  W/S = Y+/-   A/D = X-/+   R/F = Z+/-
Rotation:     I/K = RY+/-  J/L = RX-/+  U/O = RZ-/+
Space = stop     Q / Esc / Ctrl-C = quit

Each held key sends a short, self-stopping speedl() burst directly to the
robot's secondary client interface (port 30002). There is no persistent
control session (unlike jog_ur.py's ur_rtde-based approach, which still
isn't confirmed working) - every burst decelerates to a stop on its own
before this script sends the next one, so motion is stepped/pulsed rather
than perfectly smooth, and a crash of this script mid-jog just means the
current burst finishes and stops normally.

Make sure the workspace is clear. Press Enter to start.
"""


def read_available_keys(fd):
    keys = []
    while select.select([fd], [], [], 0)[0]:
        chunk = os.read(fd, 1)
        if not chunk:
            break
        keys.append(chunk.decode(errors="ignore"))
    return keys


def send_script(script, timeout=2.0):
    with socket.create_connection((ROBOT_IP, SECONDARY_PORT), timeout=timeout) as s:
        s.sendall(script.encode("utf-8"))


def nudge_script(velocity):
    vec = "[" + ",".join(f"{v:.5f}" for v in velocity) + "]"
    return f"""def prog():
  speedl({vec}, a={ACCELERATION}, t={NUDGE_DURATION})
  stopl({STOP_DECEL})
end
prog()
"""


def stop_script():
    return f"""def prog():
  stopl({STOP_DECEL})
end
prog()
"""


def main():
    print(INSTRUCTIONS)
    input()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_seen = {k: 0.0 for k in KEY_MAP}
    was_active = False

    try:
        tty.setcbreak(fd)
        print("Connected (no persistent session - sends per-burst). Jogging...")
        while True:
            tick_start = time.time()

            for key in read_available_keys(fd):
                if key in QUIT_KEYS:
                    return
                if key == STOP_KEY:
                    last_seen = {k: 0.0 for k in KEY_MAP}
                elif key in KEY_MAP:
                    last_seen[key] = tick_start

            velocity = [0.0] * 6
            active_axes = set()
            for key, (axis, sign) in KEY_MAP.items():
                if tick_start - last_seen[key] <= HOLD_TIMEOUT:
                    speed = LINEAR_SPEED if axis < 3 else ANGULAR_SPEED
                    velocity[axis] += sign * speed
                    active_axes.add(AXIS_NAMES[axis])

            axes_str = ",".join(sorted(active_axes)) if active_axes else "-"
            sys.stdout.write(f"\ractive: {axes_str:<12}")
            sys.stdout.flush()

            if active_axes:
                send_script(nudge_script(velocity))
                was_active = True
            elif was_active:
                send_script(stop_script())
                was_active = False
            else:
                time.sleep(0.05)
    finally:
        try:
            send_script(stop_script())
        except Exception:
            pass
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nStopped.")


if __name__ == "__main__":
    main()
