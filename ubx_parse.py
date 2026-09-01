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


FIX_TYPE_NAMES = {
    0: "NO FIX",
    1: "DEAD RECKONING",
    2: "2D",
    3: "3D",
    4: "3D+DR",
    5: "TIME ONLY",
}
