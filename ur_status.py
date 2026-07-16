import sys
import time

import dashboard_client

ROBOT_IP = "192.168.20.1"
POWER_ON_TIMEOUT = 10   # seconds to wait for robotmode to report IDLE after powerOn()


def report(client):
    print(f"robotmode:      {client.robotmode()}")
    print(f"safetymode:     {client.safetymode()}")
    print(f"safetystatus:   {client.safetystatus()}")
    print(f"programState:   {client.programState()}")
    print(f"loadedProgram:  {client.getLoadedProgram()}")
    print(f"isInRemoteCtl:  {client.isInRemoteControl()}")


client = dashboard_client.DashboardClient(ROBOT_IP)
client.connect()

print("--- status ---")
report(client)

if "--clear" in sys.argv:
    print("\n--- clearing ---")
    print(client.closeSafetyPopup())
    try:
        print(client.unlockProtectiveStop())
    except Exception as e:
        print(f"unlockProtectiveStop failed (needs 5s since the stop): {e}")
    print(client.stop())

    if "POWER_OFF" in client.robotmode():
        print(client.powerOn())
        start = time.time()
        while "IDLE" not in client.robotmode() and time.time() - start < POWER_ON_TIMEOUT:
            time.sleep(0.5)

    if "IDLE" in client.robotmode():
        print(client.brakeRelease())
        start = time.time()
        while "RUNNING" not in client.robotmode() and time.time() - start < POWER_ON_TIMEOUT:
            time.sleep(0.5)

    print("\n--- status after clear ---")
    report(client)

client.disconnect()
