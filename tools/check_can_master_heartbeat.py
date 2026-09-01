import sys
from pymavlink import mavutil

def main():
    conn = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    print("waiting for heartbeat...")
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        print("NO HEARTBEAT")
        return
    print(f"Heartbeat: type={hb.type} autopilot={hb.autopilot} sys={conn.target_system} comp={conn.target_component}")

if __name__ == '__main__':
    main()
