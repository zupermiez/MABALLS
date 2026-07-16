import time

from natnet import NatNetClient, DataFrame

SERVER_IP = "192.168.10.1"
LOCAL_IP = "192.168.10.2"
DURATION = 8


def handle_frame(frame: DataFrame):
    for rb in frame.rigid_bodies:
        print(
            f"frame={frame.prefix.frame_number} rb_id={rb.id_num} "
            f"pos={tuple(round(v, 4) for v in rb.pos)} "
            f"rot={tuple(round(v, 4) for v in rb.rot)} "
            f"valid={rb.tracking_valid} error={rb.marker_error}"
        )


client = NatNetClient(server_ip_address=SERVER_IP, local_ip_address=LOCAL_IP, use_multicast=True)
client.on_data_frame_received_event.handlers.append(handle_frame)

with client:
    print(f"Connected. Server info: {client.server_info}")
    print(f"Protocol version: {client.protocol_version}")
    client.run_async()
    time.sleep(DURATION)
    client.stop_async()
