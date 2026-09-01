import serial, time, struct

PORT = '/dev/ttyACM2'

def ubx_checksum(body):
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b

def ubx_frame(cls, mid, payload):
    body = bytes([cls, mid, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + payload
    a, b = ubx_checksum(body)
    return bytes([0xB5, 0x62]) + body + bytes([a, b])

def valset_u1(key, value, layers=0x01):
    payload = bytes([0x00, layers, 0x00, 0x00]) + struct.pack('<I', key) + struct.pack('<B', value)
    return ubx_frame(0x06, 0x8A, payload)

def valset_u4(key, value, layers=0x01):
    payload = bytes([0x00, layers, 0x00, 0x00]) + struct.pack('<I', key) + struct.pack('<I', value)
    return ubx_frame(0x06, 0x8A, payload)

def send_and_check(ser, frame, label):
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(0.3)
    resp = ser.read(200)
    ack = bytes([0xB5,0x62,0x05,0x01]) in resp
    nak = bytes([0xB5,0x62,0x05,0x00]) in resp
    status = "ACK" if ack else ("NAK" if nak else "NO-RESPONSE")
    print(f"{label:45s} -> {status}")
    return status

# Keys (u-blox F9/X-gen VALSET keyspace, 0x20... = U1, 0x40... = U4)
KEY_TMODE_MODE            = 0x20030001
KEY_TMODE_SVIN_MIN_DUR    = 0x40030010
KEY_TMODE_SVIN_ACC_LIMIT  = 0x40030011

RTCM_USB_KEYS = {
    "RTCM_1005 (station ARP)":        0x209102BE,
    "RTCM_1077 (GPS MSM7)":           0x209102CE,
    "RTCM_1097 (Galileo MSM7)":       0x20910318,
    "RTCM_1127 (BeiDou MSM7)":        0x209102D8,
}

def main():
    ser = serial.Serial(PORT, baudrate=115200, timeout=1)
    time.sleep(0.3)

    print("=== Survey-in / TMODE config ===")
    send_and_check(ser, valset_u1(KEY_TMODE_MODE, 1), "TMODE_MODE=1 (survey-in)")
    send_and_check(ser, valset_u4(KEY_TMODE_SVIN_MIN_DUR, 60), "SVIN_MIN_DUR=60s")
    send_and_check(ser, valset_u4(KEY_TMODE_SVIN_ACC_LIMIT, 20000), "SVIN_ACC_LIMIT=2.0m")

    print("\n=== RTCM3 output on USB, rate=1 (every epoch) ===")
    for label, key in RTCM_USB_KEYS.items():
        send_and_check(ser, valset_u1(key, 1), label)

    print("\n=== Listening for RTCM3 (0xD3 preamble) traffic for 10s ===")
    ser.reset_input_buffer()
    total = b""
    start = time.time()
    while time.time() - start < 10:
        c = ser.read(2000)
        if c:
            total += c
    print(f"{len(total)} bytes received")
    d3_count = total.count(b'\xd3')
    ubx_count = total.count(b'\xb5\x62')
    print(f"0xD3 (RTCM3 preamble candidate) bytes: {d3_count}")
    print(f"UBX sync count: {ubx_count}")

    ser.close()

if __name__ == '__main__':
    main()
