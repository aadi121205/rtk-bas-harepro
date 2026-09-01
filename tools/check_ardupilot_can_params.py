from pymavlink import mavutil
import time

names = [
    "CAN_P1_DRIVER", "CAN_P2_DRIVER",
    "CAN_D1_PROTOCOL", "CAN_D2_PROTOCOL",
    "CAN_D1_UC_NODE", "CAN_D2_UC_NODE",
]

def main():
    conn = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    conn.wait_heartbeat(timeout=10)
    print("connected, sys", conn.target_system, "comp", conn.target_component)

    for name in names:
        conn.mav.param_request_read_send(
            conn.target_system, conn.target_component,
            name.encode('utf-8'), -1
        )
        msg = conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg is None:
            print(f"{name}: NO RESPONSE (param may not exist on this firmware)")
        else:
            pname = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode('utf-8', 'ignore')
            print(f"{pname.strip():20s} = {msg.param_value}")

if __name__ == '__main__':
    main()
