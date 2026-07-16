import time

import rtde_control
import rtde_receive

ROBOT_IP = "192.168.20.1"
SPEED = 0.02   # m/s, deliberately slow for a first real-motion confirmation
ACCELERATION = 0.3
CONTROL_HZ = 50

print("waiting for URCap connection...", flush=True)
flags = rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP, flags=flags)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
print("CONNECTED", flush=True)

period = 1.0 / CONTROL_HZ
start_pose = rtde_r.getActualTCPPose()
print("start pose:", [round(v, 4) for v in start_pose], flush=True)


def run_for(seconds, velocity):
    n_ticks = int(seconds * CONTROL_HZ)
    for _ in range(n_ticks):
        t0 = time.time()
        rtde_c.speedL(velocity, ACCELERATION)
        time.sleep(max(0.0, period - (time.time() - t0)))


print("moving +Z for 1s...", flush=True)
run_for(1.0, [0.0, 0.0, SPEED, 0.0, 0.0, 0.0])
mid_pose = rtde_r.getActualTCPPose()
print("mid pose:", [round(v, 4) for v in mid_pose], flush=True)

print("moving -Z for 1s (returning)...", flush=True)
run_for(1.0, [0.0, 0.0, -SPEED, 0.0, 0.0, 0.0])

rtde_c.speedStop()
end_pose = rtde_r.getActualTCPPose()
print("end pose:", [round(v, 4) for v in end_pose], flush=True)

z_delta_out = mid_pose[2] - start_pose[2]
z_delta_back = end_pose[2] - mid_pose[2]
print(f"Z moved out by {z_delta_out:+.4f} m, back by {z_delta_back:+.4f} m", flush=True)

rtde_c.disconnect()
rtde_r.disconnect()
print("disconnected cleanly", flush=True)
