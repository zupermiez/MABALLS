import rtde_receive

ROBOT_IP = "192.168.20.1"

rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
pose = rtde_r.getActualTCPPose()
q = rtde_r.getActualQ()
rtde_r.disconnect()

print("TCP pose (x,y,z,rx,ry,rz):", " ".join(f"{v:.4f}" for v in pose))
print("Joints (rad):             ", " ".join(f"{v:.4f}" for v in q))
print()
print("Copy-paste ready:")
print("  --pose   " + " ".join(f"{v:.4f}" for v in pose))
print("  --joints " + " ".join(f"{v:.4f}" for v in q))
