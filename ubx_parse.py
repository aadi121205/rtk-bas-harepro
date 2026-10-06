"""Shared UBX message parsing helpers (NAV-PVT, NAV-SVIN) and RTCM3 frame
counting, used by ntrip_caster.py's live TUI and check_status.py."""
import struct


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


def decode_nav_pvt(payload):
    _, _, fixType, _, _, numSV = struct.unpack_from('<IiBBBB', payload, 12)
    lon, lat, height, _ = struct.unpack_from('<iiii', payload, 24)
    hAcc, vAcc = struct.unpack_from('<II', payload, 40)
    return {
        "fixType": fixType,
        "numSV": numSV,
        "lat": lat / 1e7,
        "lon": lon / 1e7,
        "height_m": height / 1000,
        "hAcc_m": hAcc / 1000,
        "vAcc_m": vAcc / 1000,
    }


def decode_nav_svin(payload):
    _, dur = struct.unpack_from('<II', payload, 4)
    meanAcc, obs = struct.unpack_from('<II', payload, 28)
    valid, active = struct.unpack_from('<BB', payload, 36)
    return {
        "duration_s": dur,
        "observations": obs,
        "meanAcc_m": meanAcc / 10000,
        "valid": bool(valid),
        "active": bool(active),
    }


def count_rtcm3_frames(data):
    """Structurally-plausible RTCM3 frame count (preamble 0xD3 + 10-bit length)."""
    count = 0
    i, n = 0, len(data)
    while i < n - 3:
        if data[i] == 0xD3:
            length = ((data[i + 1] & 0x03) << 8) | data[i + 2]
            total_len = 3 + length + 3
            if i + total_len <= n:
                count += 1
                i += total_len
                continue
        i += 1
    return count


def _make_crc24q_table():
    table = []
    for i in range(256):
        crc = i << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        table.append(crc & 0xFFFFFF)
    return table


_CRC24Q = _make_crc24q_table()


def crc24q(data):
    crc = 0
    for b in data:
        crc = ((crc << 8) & 0xFFFFFF) ^ _CRC24Q[(crc >> 16) ^ b]
    return crc


def split_frames(buf):
    """Pull checksum-valid UBX and RTCM3 frames off the front of a stream buffer.

    Returns (frames, consumed): frames are ("ubx", (cls, id), payload) or
    ("rtcm", msg_type, frame_bytes); consumed is how many leading bytes of buf
    are done with (frames + junk). An incomplete frame at the end is left
    unconsumed for the next call, so each frame is reported exactly once.
    """
    frames = []
    i, n = 0, len(buf)
    while i < n:
        b0 = buf[i]
        if b0 == 0xB5:
            if i + 6 > n:
                break
            if buf[i + 1] != 0x62:
                i += 1
                continue
            length = buf[i + 4] | (buf[i + 5] << 8)
            end = i + 8 + length
            if end > n:
                if length > 8192:  # implausible length: junk, not a frame
                    i += 1
                    continue
                break
            ck_a = ck_b = 0
            for x in buf[i + 2:end - 2]:
                ck_a = (ck_a + x) & 0xFF
                ck_b = (ck_b + ck_a) & 0xFF
            if (ck_a, ck_b) == (buf[end - 2], buf[end - 1]):
                frames.append(("ubx", (buf[i + 2], buf[i + 3]), bytes(buf[i + 6:end - 2])))
                i = end
                continue
            i += 1
        elif b0 == 0xD3:
            if i + 3 > n:
                break
            if buf[i + 1] & 0xFC:  # 6 reserved bits must be zero
                i += 1
                continue
            length = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
            end = i + 6 + length
            if end > n:
                break
            if length >= 2 and crc24q(buf[i:end - 3]) == int.from_bytes(buf[end - 3:end], "big"):
                frames.append(("rtcm", (buf[i + 3] << 4) | (buf[i + 4] >> 4), bytes(buf[i:end])))
                i = end
                continue
            i += 1
        else:
            i += 1
    return frames, i


FIX_TYPE_NAMES = {
    0: "NO FIX",
    1: "DEAD RECKONING",
    2: "2D",
    3: "3D",
    4: "3D+DR",
    5: "TIME ONLY",
}
