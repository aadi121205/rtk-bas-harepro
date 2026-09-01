from pymavlink import mavutil
import time

def try_bus(conn, bus_param):
    print(f"\n--- requesting CAN_FORWARD on bus {bus_param} ---")
    conn.mav.command_long_send(
        conn.target_system, conn.target_component,
        mavutil.mavlink.MAV_CMD_CAN_FORWARD, 0,
        bus_param, 0, 0, 0, 0, 0, 0)
    ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print("ACK:", ack)

    count = 0
    start = time.time()
    while time.time() - start < 5:
        m = conn.recv_match(type=['CAN_FRAME', 'CANFD_FRAME'], blocking=True, timeout=0.5)
        if m:
            count += 1
            if count <= 5:
                print("  frame:", m)
    print(f"  total frames in 5s on bus {bus_param}: {count}")

def main():
    conn = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    conn.wait_heartbeat(timeout=10)
    print("sys", conn.target_system, "comp", conn.target_component)

    try_bus(conn, 1)  # CAN1
    try_bus(conn, 2)  # CAN2

if __name__ == '__main__':
    main()
