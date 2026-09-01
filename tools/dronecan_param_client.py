#!/usr/bin/env python3
"""
Minimal DroneCAN parameter GetSet client that bridges via an ArduPilot
autopilot's MAVLink CAN_FRAME forwarding, bypassing the `dronecan` library's
broken multiprocessing transport (see README gotcha #4).

Uses the `dronecan` library ONLY for correct DSDL encoding/decoding
(Transfer.to_frames() / Transfer.from_frames()) -- never for its own
transport/driver. Sending/receiving raw CAN_FRAME messages is done directly
via pymavlink, which is verified reliable in this environment.

Must set MAVLINK20=1 and MAVLINK_DIALECT=ardupilotmega in the environment
BEFORE importing pymavlink, otherwise it silently falls back to MAVLink v1.0
which doesn't have CAN_FRAME at all.
"""
import os
os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")

import argparse
import time

import dronecan
from dronecan import uavcan
import dronecan.transport as dct
from pymavlink import mavutil


def get_param(conn, bus, source_node, dest_node, name, timeout=5):
    return _getset(conn, bus, source_node, dest_node, name, value=None, timeout=timeout)


def set_param(conn, bus, source_node, dest_node, name, int_value, timeout=5):
    return _getset(conn, bus, source_node, dest_node, name, value=int_value, timeout=timeout)


def _getset(conn, bus, source_node, dest_node, name, value, timeout):
    req = uavcan.protocol.param.GetSet.Request(name=name.encode())
    if value is not None:
        req.value.integer_value = value

    transfer_id = int(time.time()) & 0x1F
    xfer = dct.Transfer(
        transfer_id=transfer_id,
        source_node_id=source_node,
        dest_node_id=dest_node,
        payload=req,
        request_not_response=True,
        service_not_message=True,
    )

    for frame in xfer.to_frames():
        data = bytes(frame.bytes)
        conn.mav.can_frame_send(
            conn.target_system, conn.target_component,
            bus, len(data), frame.message_id, list(data) + [0] * (8 - len(data))
        )

    # Listen for the response: service response frames from dest_node to
    # source_node, same service type id, request_not_response=False
    resp_dtid = dronecan.uavcan.protocol.param.GetSet.Request().transfer_id if False else None
    tm = dct.TransferManager()
    start = time.time()
    while time.time() - start < timeout:
        m = conn.recv_match(type="CAN_FRAME", blocking=True, timeout=0.3)
        if m is None:
            continue
        can_id = m.id
        service_flag = (can_id >> 7) & 0x1
        if not service_flag:
            continue
        req_resp_flag = (can_id >> 15) & 0x1
        dtid = (can_id >> 16) & 0xFF
        dst = (can_id >> 8) & 0x7F
        src = can_id & 0x7F
        if req_resp_flag != 0:  # 1 = request, we want responses (0)
            continue
        if src != dest_node or dst != source_node:
            continue

        frame = dct.Frame(message_id=can_id, data=bytes(m.data[:m.len]))
        frames = tm.receive_frame(frame)
        if frames is None:
            continue

        resp_xfer = dct.Transfer(
            source_node_id=src, dest_node_id=dst,
            request_not_response=False, service_not_message=True,
        )
        resp_xfer.data_type_id = dtid
        try:
            resp_xfer.from_frames(frames)
        except dct.TransferError as e:
            print(f"  (transfer reassembly error, ignoring: {e})")
            continue
        return resp_xfer.payload

    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM1", help="autopilot MAVLink serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--bus", type=int, default=0, help="0=CAN1, 1=CAN2")
    ap.add_argument("--source-node", type=int, default=99)
    ap.add_argument("--dest-node", type=int, required=True, help="target DroneCAN node id (e.g. HerePro's node id)")
    ap.add_argument("--name", required=True, help="parameter name, e.g. CAN_NODE")
    ap.add_argument("--set", type=int, default=None, help="if given, SET the parameter to this integer value; otherwise GET only")
    args = ap.parse_args()

    conn = mavutil.mavlink_connection(args.port, baud=args.baud, dialect="ardupilotmega")
    print("waiting for autopilot heartbeat...")
    conn.wait_heartbeat(timeout=10)
    print(f"connected: sys={conn.target_system}")

    conn.mav.command_long_send(conn.target_system, conn.target_component,
                                mavutil.mavlink.MAV_CMD_CAN_FORWARD, 0,
                                args.bus + 1, 0, 0, 0, 0, 0, 0)
    conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)

    if args.set is not None:
        print(f"SETTING node {args.dest_node} param '{args.name}' = {args.set} ...")
        resp = set_param(conn, args.bus, args.source_node, args.dest_node, args.name, args.set)
    else:
        print(f"GETTING node {args.dest_node} param '{args.name}' ...")
        resp = get_param(conn, args.bus, args.source_node, args.dest_node, args.name)

    if resp is None:
        print("NO RESPONSE (timed out)")
    else:
        print("Response:", resp)


if __name__ == "__main__":
    main()
