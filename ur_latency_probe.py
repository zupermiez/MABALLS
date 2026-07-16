import time

import rtde_control
import rtde_receive

ROBOT_IP = "192.168.20.1"
N_CALLS = 20
MAX_WALL_SECONDS = 10.0   # hard abort regardless of N_CALLS - a real wall-clock bound

print("waiting for URCap connection...", flush=True)
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP, flags=rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
print("CONNECTED", flush=True)

latencies = []
test_start = time.time()
for i in range(N_CALLS):
    if time.time() - test_start > MAX_WALL_SECONDS:
        print(f"ABORT: hit {MAX_WALL_SECONDS}s wall-clock limit after {i} calls", flush=True)
        break
    t0 = time.time()
    rtde_c.speedL([0.0] * 6, 0.3)
    dt = time.time() - t0
    latencies.append(dt)
    print(f"call {i}: {dt * 1000:.1f} ms", flush=True)

rtde_c.speedStop()
rtde_c.disconnect()
rtde_r.disconnect()

if latencies:
    print(f"min={min(latencies)*1000:.1f}ms max={max(latencies)*1000:.1f}ms "
          f"avg={sum(latencies)/len(latencies)*1000:.1f}ms", flush=True)
print("disconnected cleanly", flush=True)
