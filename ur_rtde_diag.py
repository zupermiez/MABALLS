import time

import rtde_control
import rtde_receive

ROBOT_IP = "192.168.20.1"

print("waiting for URCap connection...", flush=True)
flags = rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP, flags=flags)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
print("CONNECTED", flush=True)

for i in range(10):
    print(
        f"t={i * 0.5:.1f}s "
        f"isConnected={rtde_c.isConnected()} "
        f"isProgramRunning={rtde_c.isProgramRunning()} "
        f"robotStatus={rtde_c.getRobotStatus()}",
        flush=True,
    )
    time.sleep(0.5)

rtde_c.disconnect()
rtde_r.disconnect()
print("disconnected cleanly", flush=True)
