import os
import select
import signal
import sys
import termios
import time
import tty

import rtde_control
import rtde_receive

ROBOT_IP = "192.168.20.1"

LINEAR_SPEED = 0.05    # m/s, translation jog speed
ANGULAR_SPEED = 0.2    # rad/s, rotation jog speed
ACCELERATION = 0.5     # m/s^2 (and rad/s^2) - speedL ramp rate
CONTROL_HZ = 50
DISPLAY_HZ = 10        # HUD print rate - decoupled from CONTROL_HZ so terminal
                        # I/O (which can block unpredictably) never delays a
                        # speedL tick

# Terminals only report keydown (auto-repeat), never keyup, so "is this key
# still held" is inferred from how recently a repeat/press arrived. This
# must comfortably exceed the OS's initial repeat delay (~0.3-0.5s on most
# Linux desktops) or a genuinely held key will look "released" right before
# repeat kicks in. Keep jog speeds low above so the resulting worst-case
# coast (a few hundred ms) stays centimeter-scale, not a safety issue.
HOLD_TIMEOUT = 0.3     # seconds

# axis index into the 6-vector [x, y, z, rx, ry, rz], and sign
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
UR12e keyboard jog
------------------
Translation:  W/S = Y+/-   A/D = X-/+   R/F = Z+/-
Rotation:     I/K = RY+/-  J/L = RX-/+  U/O = RZ-/+
Space = stop     Q / Esc / Ctrl-C = quit

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


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    # Python only unwinds try/finally on SIGINT (Ctrl-C) by default; a bare
    # SIGTERM (e.g. from `timeout`, a process manager, or `kill`) would skip
    # our speedStop()/disconnect() cleanup and leave the robot's real-time
    # thread waiting on a client that vanished - convert it into a normal
    # KeyboardInterrupt so cleanup always runs.
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    print(INSTRUCTIONS)
    input()

    # Uses the External Control URCap (started from the pendant) rather than
    # ur_rtde's headless script-upload mode - more version-stable across
    # PolyScope releases. Requires the URCap program to already be running
    # and waiting on the pendant before this connects.
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP, flags=rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    print(f"Connected to {ROBOT_IP}. Jogging...")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_seen = {k: 0.0 for k in KEY_MAP}
    period = 1.0 / CONTROL_HZ
    display_period = 1.0 / DISPLAY_HZ
    last_display = 0.0

    try:
        tty.setcbreak(fd)
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

            rtde_c.speedL(velocity, ACCELERATION)

            if tick_start - last_display >= display_period:
                last_display = tick_start
                pose = rtde_r.getActualTCPPose()
                pose_str = " ".join(f"{v:+.3f}" for v in pose)
                axes_str = ",".join(sorted(active_axes)) if active_axes else "-"
                sys.stdout.write(f"\rTCP [{pose_str}]  active: {axes_str:<12}")
                sys.stdout.flush()

            elapsed = time.time() - tick_start
            time.sleep(max(0.0, period - elapsed))
    finally:
        rtde_c.speedStop()
        rtde_c.disconnect()
        rtde_r.disconnect()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nStopped, disconnected.")


if __name__ == "__main__":
    main()
