from pymavlink import mavutil
import time
from collections import Counter

def decode(can_id):
    # DroneCAN v0 / legacy UAVCAN message frame encoding:
    # bits 28-24: priority, bits 23-8: data_type_id (16 bits), bit7: service-not-message flag
    # bits 6-0: source node id (for message frames)
    priority = (can_id >> 24) & 0x1F
    service_flag = (can_id >> 7) & 0x1  # for message frames this should be part of dtid range; approximate
    data_type_id = (can_id >> 8) & 0xFFFF
    source_node = can_id & 0x7F
    return priority, data_type_id, source_node

def main():
    conn = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    conn.wait_heartbeat(timeout=10)

    for bus_param, label in [(1, "CAN1"), (2, "CAN2")]:
        conn.mav.command_long_send(conn.target_system, conn.target_component,
                                    mavutil.mavlink.MAV_CMD_CAN_FORWARD, 0,
                                    bus_param, 0,0,0,0,0,0)
        conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2)

        dtid_counter = Counter()
        node_ids_by_dtid = {}
        start = time.time()
        while time.time() - start < 6:
            m = conn.recv_match(type='CAN_FRAME', blocking=True, timeout=0.3)
            if m is None:
                continue
            prio, dtid, node = decode(m.id)
            dtid_counter[dtid] += 1
            node_ids_by_dtid.setdefault(dtid, set()).add(node)

        print(f"\n=== {label} (bus_param={bus_param}) ===")
        print("Top data_type_ids seen (count, dtid, node_ids):")
        for dtid, cnt in dtid_counter.most_common(10):
            print(f"  dtid={dtid:5d} count={cnt:4d} nodes={sorted(node_ids_by_dtid[dtid])}")
        # NodeStatus is dtid 341
        if 341 in node_ids_by_dtid:
            print(f"  -> NodeStatus (341) seen from node ids: {sorted(node_ids_by_dtid[341])}")

if __name__ == '__main__':
    main()
