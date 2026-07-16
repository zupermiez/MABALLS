import signal
import time

import rtde_control
import rtde_receive

ROBOT_IP = "192.168.20.1"
SPEED = 0.02        # m/s, deliberately slow
ACCELERATION = 0.1  # m/s^2
Z_DELTA = 0.02       # m, small bounded up-then-back move


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


# Convert SIGTERM into a normal KeyboardInterrupt so a force-kill (e.g. from
# `timeout`) still unwinds the finally block below instead of stranding the
# robot's real-time thread - see CLAUDE.md robot_control_testing_safety notes.
signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

print("waiting for URCap connection...", flush=True)
flags = rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP, flags=flags)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
print("CONNECTED", flush=True)

try:
    start_pose = rtde_r.getActualTCPPose()
    print("start pose:", [round(v, 4) for v in start_pose], flush=True)

    target_up = list(start_pose)
    target_up[2] += Z_DELTA

    print(f"moveL +{Z_DELTA} m in Z...", flush=True)
    rtde_c.moveL(target_up, SPEED, ACCELERATION)
    mid_pose = rtde_r.getActualTCPPose()
    print("mid pose:", [round(v, 4) for v in mid_pose], flush=True)

    print("moveL back to start...", flush=True)
    rtde_c.moveL(start_pose, SPEED, ACCELERATION)
    end_pose = rtde_r.getActualTCPPose()
    print("end pose:", [round(v, 4) for v in end_pose], flush=True)

    z_delta_out = mid_pose[2] - start_pose[2]
    z_delta_back = end_pose[2] - mid_pose[2]
    print(f"Z moved out by {z_delta_out:+.4f} m, back by {z_delta_back:+.4f} m", flush=True)
finally:
    rtde_c.stopL()
    rtde_c.disconnect()
    rtde_r.disconnect()
    print("disconnected cleanly", flush=True)
