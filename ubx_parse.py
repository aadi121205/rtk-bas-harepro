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


def extract_frames(buf, rtcm_types=None):
    """Single pass over a stream buffer. Returns (ubx_msgs, rtcm_count, consumed)
    where ubx_msgs is a list of (cls, mid, payload) for each *complete* UBX
    frame and consumed is how many leading bytes are fully processed (safe to
    drop). A partial frame at the end is left unconsumed so it is completed by
    the next read -- and complete frames are never seen twice."""
    ubx, rtcm = [], 0
    i, n = 0, len(buf)
    while i < n:
        b = buf[i]
        if b == 0xB5:
            if i + 6 > n:
                break
            if buf[i + 1] == 0x62:
                length = buf[i + 4] | (buf[i + 5] << 8)
                if i + 8 + length > n:
                    if length > 2048:  # implausible, treat as noise
                        i += 1
                        continue
                    break
                ubx.append((buf[i + 2], buf[i + 3], bytes(buf[i + 6:i + 6 + length])))
                i += 8 + length
                continue
        elif b == 0xD3:
            if i + 3 > n:
                break
            if (buf[i + 1] & 0xFC) == 0:  # 6 reserved bits must be zero
                length = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
                total = 6 + length
                if i + total > n:
                    break
                rtcm += 1
                if rtcm_types is not None:  # message number = first 12 bits of payload
                    rtcm_types[(buf[i + 3] << 4) | (buf[i + 4] >> 4)] += 1
                i += total
                continue
        i += 1
    return ubx, rtcm, i


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


def decode_rxm_rawx(payload):
    """Return [{gnssId, svId, sigId, cno}] for each measurement in UBX-RXM-RAWX."""
    n = payload[11]
    out = []
    for i in range(n):
        o = 16 + 32 * i
        if o + 32 > len(payload):
            break
        out.append({"gnssId": payload[o + 20], "svId": payload[o + 21],
                    "sigId": payload[o + 22], "cno": payload[o + 26]})
    return out


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


FIX_TYPE_NAMES = {
    0: "NO FIX",
    1: "DEAD RECKONING",
    2: "2D",
    3: "3D",
    4: "3D+DR",
    5: "TIME ONLY",
}
