import signal

import rtde_control
import rtde_receive

ROBOT_IP = "192.168.20.1"
SPEED = 0.02
ACCELERATION = 0.1
Z_DELTA = 0.02


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

print("waiting for URCap connection...", flush=True)
flags = rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP, 500.0, flags)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
print("CONNECTED", flush=True)

try:
    start_pose = rtde_r.getActualTCPPose()
    print("start pose:", [round(v, 4) for v in start_pose], flush=True)

    target_up = list(start_pose)
    target_up[2] += Z_DELTA

    print(f"moveL +{Z_DELTA} m in Z (explicit freq=500)...", flush=True)
    rtde_c.moveL(target_up, SPEED, ACCELERATION)
    mid_pose = rtde_r.getActualTCPPose()
    print("mid pose:", [round(v, 4) for v in mid_pose], flush=True)

    print("moveL back to start...", flush=True)
    rtde_c.moveL(start_pose, SPEED, ACCELERATION)
    end_pose = rtde_r.getActualTCPPose()
    print("end pose:", [round(v, 4) for v in end_pose], flush=True)

    print(f"Z delta out: {mid_pose[2] - start_pose[2]:+.4f} m", flush=True)
    print(f"Z delta back: {end_pose[2] - mid_pose[2]:+.4f} m", flush=True)
finally:
    rtde_c.stopL()
    rtde_c.disconnect()
    rtde_r.disconnect()
    print("disconnected cleanly", flush=True)
