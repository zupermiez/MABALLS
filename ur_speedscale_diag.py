import signal
import time

import rtde_control
import rtde_receive

ROBOT_IP = "192.168.20.1"
SPEED = 0.02
ACCELERATION = 0.1
CONTROL_HZ = 50
DURATION = 2.0


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

print("waiting for URCap connection...", flush=True)
flags = rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP, flags=flags)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
print("CONNECTED", flush=True)

period = 1.0 / CONTROL_HZ
try:
    start_pose = rtde_r.getActualTCPPose()
    print("start pose:", [round(v, 4) for v in start_pose], flush=True)

    n_ticks = int(DURATION * CONTROL_HZ)
    for i in range(n_ticks):
        t0 = time.time()
        rtde_c.speedL([0.0, 0.0, SPEED, 0.0, 0.0, 0.0], ACCELERATION)
        if i % 10 == 0:
            print(
                f"tick={i} speedScaling={rtde_r.getSpeedScaling():.3f} "
                f"targetSpeedFraction={rtde_r.getTargetSpeedFraction():.3f} "
                f"pose_z={rtde_r.getActualTCPPose()[2]:.4f}",
                flush=True,
            )
        time.sleep(max(0.0, period - (time.time() - t0)))

    end_pose = rtde_r.getActualTCPPose()
    print("end pose:", [round(v, 4) for v in end_pose], flush=True)
    print(f"Z delta: {end_pose[2] - start_pose[2]:+.4f} m", flush=True)
finally:
    rtde_c.speedStop()
    rtde_c.disconnect()
    rtde_r.disconnect()
    print("disconnected cleanly", flush=True)
