#!/usr/bin/env python3
"""
Check current HerePro base status (fix, satellites, survey-in progress, RTCM3 flow)
without disturbing the running ntrip_caster.py.

If the caster is running, connects to it as an NTRIP client (127.0.0.1:2101 by
default) so it doesn't fight the caster for the serial port. If the caster is
NOT running, pass --serial to read the port directly instead.
"""
import argparse
import socket
import struct
import time


def find_ubx_msgs(data, cls, mid):
    out = []
    i, n = 0, len(data)
    while i < n - 8:
        if data[i] == 0xB5 and data[i + 1] == 0x62 and data[i + 2] == cls and data[i + 3] == mid:
            length = data[i + 4] | (data[i + 5] << 8)
            if i + 6 + length + 2 <= n:
                out.append(data[i + 6:i + 6 + length])
            i += 6 + length
        else:
            i += 1
    return out


def capture_via_caster(host, port, mount, seconds):
    s = socket.create_connection((host, port), timeout=5)
    req = f"GET /{mount} HTTP/1.1\r\nUser-Agent: StatusCheck/1.0\r\nAccept: */*\r\n\r\n"
    s.sendall(req.encode())
    s.settimeout(1)
    total = b""
    start = time.time()
    while time.time() - start < seconds:
        try:
            chunk = s.recv(4096)
            if chunk:
                total += chunk
        except socket.timeout:
            continue
    s.close()
    if total.startswith(b"ICY 200 OK\r\n\r\n"):
        total = total[len(b"ICY 200 OK\r\n\r\n"):]
    return total


def capture_via_serial(device, baud, seconds):
    import serial
    ser = serial.Serial(device, baudrate=baud, timeout=1)
    time.sleep(0.3)
    ser.reset_input_buffer()
    total = b""
    start = time.time()
    while time.time() - start < seconds:
        c = ser.read(4000)
        if c:
            total += c
    ser.close()
    return total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1", help="ntrip_caster.py host (default: local)")
    ap.add_argument("--port", type=int, default=2101)
    ap.add_argument("--mount", default="HEREPRO")
    ap.add_argument("--serial", default=None, help="read this serial device directly instead of the caster")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=int, default=20)
    args = ap.parse_args()

    if args.serial:
        data = capture_via_serial(args.serial, args.baud, args.seconds)
    else:
        data = capture_via_caster(args.host, args.port, args.mount, args.seconds)

    print(f"{len(data)} bytes captured over {args.seconds}s")

    pvts = find_ubx_msgs(data, 0x01, 0x07)
    if pvts:
        p = pvts[-1]
        _, _, fixType, _, _, numSV = struct.unpack_from('<IiBBBB', p, 12)
        lon, lat, height, _ = struct.unpack_from('<iiii', p, 24)
        hAcc, vAcc = struct.unpack_from('<II', p, 40)
        print(f"NAV-PVT ({len(pvts)} msgs): fixType={fixType} numSV={numSV} "
              f"lat={lat/1e7:.7f} lon={lon/1e7:.7f} hAcc={hAcc/1000:.2f}m")
    else:
        print("no NAV-PVT seen in this window")

    svins = find_ubx_msgs(data, 0x01, 0x3B)
    if svins:
        p = svins[-1]
        _, dur = struct.unpack_from('<II', p, 4)
        meanAcc, obs = struct.unpack_from('<II', p, 28)
        valid, active = struct.unpack_from('<BB', p, 36)
        print(f"NAV-SVIN ({len(svins)} msgs): duration={dur}s observations={obs} "
              f"meanAcc={meanAcc/10000:.3f}m valid={valid} active={active}")
    else:
        print("no NAV-SVIN seen in this window")

    d3 = data.count(b'\xd3')
    print(f"RTCM3 preamble (0xD3) byte count: {d3}")


if __name__ == "__main__":
    main()
