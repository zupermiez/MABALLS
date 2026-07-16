import dashboard_client

ROBOT_IP = "192.168.20.1"

client = dashboard_client.DashboardClient(ROBOT_IP)
client.connect()
print("loadedProgram:", client.getLoadedProgram())
print(client.play())
print("programState:", client.programState())
client.disconnect()
