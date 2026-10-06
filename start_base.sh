#!/usr/bin/env bash
# Start the HerePro NTRIP base, sample its position for a chosen time, then
# print the exact NTRIP settings/commands a rover needs. Optionally compares
# the sampled position against a fiducial (known reference point).
#
# Usage: ./start_base.sh                     (asks for everything)
#        ./start_base.sh 300                 (sample 300 s, still asks about fiducial)
#        ./start_base.sh -t 5m -f 28.7485915,77.1171407,176.43
#        ./start_base.sh -t 300 -F           (no fiducial, no prompts)
#
#   -t TIME          sampling time in seconds, or Nm for minutes
#   -f LAT,LON,H     use (and save) this fiducial; H is ellipsoidal height in m
#   -F               don't use a fiducial this run (saved one is left alone)
#
# The fiducial is stored in ~/.herepro_fiducial.json, the same file
# ntrip_caster.py's TUI uses ('f' key / --fiducial), so they stay in sync.
# It is only a reference to measure error against -- it does NOT change the
# base position the HerePro broadcasts (that comes from its survey-in).
#
# The caster keeps running in the background after this script exits.
# If a caster is already listening on the port (e.g. the TUI in another
# terminal), it is reused instead of starting a second one.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="$REPO/.venv/bin/python"
PORT=2101
MOUNT=HEREPRO
LOG="$HOME/.herepro_caster.log"
PIDFILE="$HOME/.herepro_caster.pid"
FID_FILE="$HOME/.herepro_fiducial.json"
BY_ID_GLOB="/dev/serial/by-id/usb-CubePilot_HerePro_*-if01"

die() { echo "ERROR: $*" >&2; exit 1; }

port_listening() { [[ -n "$(ss -Hltn "sport = :$PORT")" ]]; }

SERVICE=herepro-ntrip
service_active() { systemctl --user is-active --quiet "$SERVICE" 2>/dev/null; }

# Normalise "LAT,LON,H" (commas and/or spaces) -> "lat,lon,h"; fails if invalid.
parse_fid() {
    "$PY" -c '
import sys
v = [float(x) for x in sys.argv[1].replace(",", " ").split()]
assert len(v) == 3, "need exactly 3 numbers"
assert -90 <= v[0] <= 90 and -180 <= v[1] <= 180, "lat/lon out of range"
print("%r,%r,%r" % tuple(v))' "$1" 2>/dev/null
}

# Saved fiducial as "lat,lon,h", or nothing if there isn't a valid one.
read_fid_file() {
    [[ -f "$FID_FILE" ]] || return 0
    "$PY" -c '
import json, sys
d = json.load(open(sys.argv[1]))
print("%r,%r,%r" % (d["lat"], d["lon"], d["height_m"]))' "$FID_FILE" 2>/dev/null || true
}

fid_file_age() {
    "$PY" -c '
import json, sys, time
t = json.load(open(sys.argv[1])).get("set_at")
print(time.strftime("%Y-%m-%d %H:%M", time.localtime(t)) if t else "unknown time")' "$FID_FILE" 2>/dev/null || echo "unknown time"
}

# Same JSON shape as ntrip_caster.py's save_fiducial().
save_fid() {
    "$PY" -c '
import json, sys, time
lat, lon, h = (float(x) for x in sys.argv[2].split(","))
json.dump({"lat": lat, "lon": lon, "height_m": h, "hAcc_m": None, "set_at": time.time()},
          open(sys.argv[1], "w"), indent=2)' "$FID_FILE" "$1"
}

ask_fid() {
    local raw parsed
    echo "  Enter as LAT,LON,HEIGHT -- decimal degrees, HEIGHT = ellipsoidal (HAE) metres,"
    echo "  NOT mean-sea-level (the receiver reports HAE; here HAE is ~40-50 m below MSL)."
    while true; do
        read -rp "  Fiducial: " raw || die "no input"
        if parsed=$(parse_fid "$raw"); then
            FID="$parsed"; return
        fi
        echo "  Couldn't parse '$raw' -- example: 28.7485915,77.1171407,176.43"
    done
}

# --- args ------------------------------------------------------------------
SECS=""
FID=""          # fiducial used this run, "lat,lon,h"; empty = none
FID_MODE=ask    # ask | given | none
while getopts ":t:f:Fh" opt; do
    case $opt in
        t) SECS="$OPTARG" ;;
        f) FID=$(parse_fid "$OPTARG") || die "bad -f '$OPTARG' (want LAT,LON,HEIGHT)"; FID_MODE=given ;;
        F) FID_MODE=none ;;
        h) sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) die "unknown option -$OPTARG (see -h)" ;;
    esac
done
shift $((OPTIND - 1))
[[ -z "$SECS" && $# -gt 0 ]] && SECS="$1"

# --- preflight -------------------------------------------------------------
[[ -x "$PY" ]] || die "no venv at $REPO/.venv -- run: python3 -m venv .venv && .venv/bin/pip install -r requirements.txt"

# --- ask for sampling time -------------------------------------------------
if [[ -z "$SECS" ]]; then
    read -rp "Sampling time (seconds, or e.g. 5m) [120]: " SECS || true
    SECS="${SECS:-120}"
fi
if [[ "$SECS" =~ ^([0-9]+)m$ ]]; then
    SECS=$(( BASH_REMATCH[1] * 60 ))
fi
[[ "$SECS" =~ ^[0-9]+$ && "$SECS" -gt 0 ]] || die "sampling time must be a positive number of seconds (or Nm), got '$SECS'"

# --- fiducial ----------------------------------------------------------------
FID_CHANGED=0
case $FID_MODE in
    given)
        save_fid "$FID"; FID_CHANGED=1
        echo "Fiducial set from -f: $FID (saved to $FID_FILE)" ;;
    none)
        echo "Fiducial: not used this run." ;;
    ask)
        SAVED=$(read_fid_file)
        if [[ -n "$SAVED" ]]; then
            echo "Fiducial present: $SAVED  (set $(fid_file_age))"
            read -rp "[U]se it, enter a [N]ew one, or [C]lear it? [U]: " ans || true
            case "${ans,,}" in
                ""|u*) FID="$SAVED" ;;
                n*)    ask_fid; save_fid "$FID"; FID_CHANGED=1; echo "  Saved." ;;
                c*)    rm -f "$FID_FILE"; FID_CHANGED=1; echo "  Fiducial cleared." ;;
                *)     die "answer U, N or C" ;;
            esac
        else
            echo "No fiducial saved."
            read -rp "Do you have a fiducial (known reference point) to enter? [y/N]: " ans || true
            if [[ "${ans,,}" == y* ]]; then
                ask_fid; save_fid "$FID"; FID_CHANGED=1; echo "  Saved."
            fi
        fi ;;
esac

# --- start (or reuse) the caster -------------------------------------------
if port_listening; then
    if service_active; then
        echo "Caster already running as the $SERVICE service -- reusing it."
    else
        echo "Caster already listening on :$PORT -- reusing it."
    fi
    if [[ $FID_CHANGED == 1 ]]; then
        echo "  Note: that caster loaded the fiducial at its startup; its own log/TUI won't"
        echo "  show the change until restarted. This script's result below does use it."
        service_active && echo "  Restart it with:  systemctl --user restart $SERVICE"
    fi
else
    compgen -G "$BY_ID_GLOB" >/dev/null || die "HerePro not found ($BY_ID_GLOB). Plug it in / check the cable."
    echo "Starting caster in background (log: $LOG)"
    cd "$REPO"
    nohup "$PY" -u ntrip_caster.py --no-ui --port "$PORT" --mount "$MOUNT" >"$LOG" 2>&1 &
    echo $! >"$PIDFILE"
    for _ in $(seq 1 20); do port_listening && break; sleep 0.5; done
    port_listening || die "caster didn't start, see $LOG"
fi

# --- sample through the caster (read-only, never touches the serial port) --
echo "Sampling for ${SECS}s... (Ctrl-C to stop early; the caster keeps running)"
OFFER_SAVE=0
[[ -z "$FID" && $FID_MODE == ask && -t 0 ]] && OFFER_SAVE=1
"$PY" - "$PORT" "$MOUNT" "$SECS" "$REPO" "$FID" "$OFFER_SAVE" "$FID_FILE" <<'PYEOF'
import json, socket, struct, sys, time
port, mount, secs, repo = int(sys.argv[1]), sys.argv[2], int(sys.argv[3]), sys.argv[4]
fid = tuple(float(x) for x in sys.argv[5].split(",")) if sys.argv[5] else None
offer_save, fid_file = sys.argv[6] == "1", sys.argv[7]
sys.path.insert(0, repo)
from geo import enu_offset

FIX = {0: "NO FIX", 1: "DR", 2: "2D", 3: "3D", 4: "3D+DR", 5: "TIME"}

def crc24q(data):
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF

s = socket.create_connection(("127.0.0.1", port), timeout=5)
s.sendall(f"GET /{mount} HTTP/1.0\r\nUser-Agent: NTRIP start_base.sh\r\n\r\n".encode())
hdr = b""
while b"\r\n\r\n" not in hdr:
    chunk = s.recv(1024)
    if not chunk:
        sys.exit("caster closed the connection")
    hdr += chunk
if b"200" not in hdr.split(b"\r\n", 1)[0]:
    sys.exit(f"caster refused mount /{mount}: {hdr.splitlines()[0]!r}")
buf = bytearray(hdr.split(b"\r\n\r\n", 1)[1])

pvt = svin = None
points = []          # (lat, lon, h) from 3D fixes only
rtcm = {}            # message type -> count
start = last_print = time.time()
s.settimeout(1)
try:
    while time.time() - start < secs:
        try:
            data = s.recv(4096)
            if not data:
                print("caster closed the connection early"); break
            buf += data
        except socket.timeout:
            pass

        # consume complete UBX / RTCM3 frames from the front of the buffer
        i = 0
        while i < len(buf):
            if buf[i] == 0xB5 and i + 1 < len(buf) and buf[i + 1] == 0x62:
                if i + 6 > len(buf): break
                n = buf[i + 4] | buf[i + 5] << 8
                if i + 8 + n > len(buf): break
                frame = buf[i + 2:i + 6 + n]
                a = b = 0
                for x in frame:
                    a = (a + x) & 0xFF; b = (b + a) & 0xFF
                if (a, b) != (buf[i + 6 + n], buf[i + 7 + n]):
                    i += 1; continue
                cls, mid, p = buf[i + 2], buf[i + 3], bytes(buf[i + 6:i + 6 + n])
                if (cls, mid) == (0x01, 0x07) and n >= 92:
                    fix, sv = p[20], p[23]
                    lon, lat, h = struct.unpack_from("<iii", p, 24)
                    hacc, vacc = struct.unpack_from("<II", p, 40)
                    pvt = dict(fix=fix, sv=sv, lat=lat / 1e7, lon=lon / 1e7, h=h / 1000, hacc=hacc / 1000, vacc=vacc / 1000)
                    if fix >= 3:
                        points.append((pvt["lat"], pvt["lon"], pvt["h"]))
                elif (cls, mid) == (0x01, 0x3B) and n >= 40:
                    dur = struct.unpack_from("<I", p, 8)[0]
                    acc, obs = struct.unpack_from("<II", p, 28)
                    svin = dict(dur=dur, acc=acc / 10000, obs=obs, valid=bool(p[36]), active=bool(p[37]))
                i += 8 + n
            elif buf[i] == 0xD3:
                if i + 3 > len(buf): break
                n = (buf[i + 1] & 0x03) << 8 | buf[i + 2]
                if i + 6 + n > len(buf): break
                body = bytes(buf[i:i + 3 + n])
                if n >= 2 and crc24q(body) == int.from_bytes(buf[i + 3 + n:i + 6 + n], "big"):
                    mtype = buf[i + 3] << 4 | buf[i + 4] >> 4
                    rtcm[mtype] = rtcm.get(mtype, 0) + 1
                    i += 6 + n
                else:
                    i += 1
            else:
                i += 1
        del buf[:i]
        if len(buf) > 65536:
            del buf[:-4096]

        now = time.time()
        if now - last_print >= 5:
            last_print = now
            p_txt = f"{FIX.get(pvt['fix'], pvt['fix'])} sats={pvt['sv']} hAcc={pvt['hacc']:.2f}m" if pvt else "no fix msg yet"
            s_txt = (f"survey-in {'VALID' if svin['valid'] else 'running'} acc={svin['acc']:.3f}m ({svin['dur']}s)"
                     if svin else "no survey-in msg yet")
            f_txt = ""
            if fid and pvt and pvt["fix"] >= 3:
                f_txt = f" | vs fiducial {enu_offset(*fid, pvt['lat'], pvt['lon'], pvt['h'])['dist_2d_m'] * 100:.0f}cm 2D"
            print(f"  [{now - start:4.0f}/{secs}s] {p_txt} | {s_txt} | rtcm msgs={sum(rtcm.values())}{f_txt}", flush=True)
except KeyboardInterrupt:
    print("\nstopped early")
elapsed = max(time.time() - start, 1e-9)

print("\n================ SAMPLING RESULT ================")
if points:
    n = len(points)
    mlat = sum(p[0] for p in points) / n
    mlon = sum(p[1] for p in points) / n
    mh = sum(p[2] for p in points) / n
    spread = (sum(enu_offset(mlat, mlon, mh, *p)["dist_3d_m"] ** 2 for p in points) / n) ** 0.5
    print(f" Averaged position : {mlat:.8f}, {mlon:.8f}  h={mh:.3f} m  ({n} fixes over {elapsed:.0f}s)")
    print(f" Spread (RMS 3D)   : {spread * 100:.1f} cm")
    if fid:
        o = enu_offset(*fid, mlat, mlon, mh)
        print(f" Fiducial          : {fid[0]:.8f}, {fid[1]:.8f}  h={fid[2]:.3f} m")
        print(f" Avg vs fiducial   : E {o['east_m']:+.3f} m  N {o['north_m']:+.3f} m  U {o['up_m']:+.3f} m")
        print(f"                     error {o['dist_2d_m'] * 100:.1f} cm horizontal, {o['dist_3d_m'] * 100:.1f} cm 3D")
    elif offer_save:
        try:
            with open("/dev/tty") as tty:
                print(" Save this averaged position as the fiducial? [y/N]: ", end="", flush=True)
                if tty.readline().strip().lower().startswith("y"):
                    json.dump({"lat": mlat, "lon": mlon, "height_m": mh, "hAcc_m": None, "set_at": time.time()},
                              open(fid_file, "w"), indent=2)
                    print(f" Fiducial saved to {fid_file} ({n}-fix average, spread {spread * 100:.1f} cm)")
        except OSError:
            pass
else:
    print(" Averaged position : none -- no 3D fix during sampling (antenna sky view?)")
if pvt:
    print(f" Last fix          : {FIX.get(pvt['fix'], pvt['fix'])}, {pvt['sv']} sats, hAcc {pvt['hacc']:.2f} m")
if svin:
    print(f" Survey-in         : {'VALID' if svin['valid'] else 'NOT VALID YET'}  acc {svin['acc']:.3f} m, {svin['dur']}s, {svin['obs']} obs")
else:
    print(" Survey-in         : no NAV-SVIN seen")
if rtcm:
    types = ", ".join(f"{t}({c / elapsed:.1f}/s)" for t, c in sorted(rtcm.items()))
    print(f" RTCM3 messages    : {types}")
else:
    print(" RTCM3 messages    : none")

warn = []
if not points:
    warn.append("no 3D fix")
if svin and not svin["valid"]:
    warn.append("survey-in not converged yet -- rover will not get RTK FIXED until it is")
if 1005 not in rtcm and 1006 not in rtcm:
    warn.append("no RTCM 1005/1006 (base position) -- rover can't use corrections without it; usually appears once survey-in is VALID")
for w in warn:
    print(f" WARNING: {w}")
PYEOF

# --- print the NTRIP connection info ---------------------------------------
mapfile -t IPS < <(ip -4 -o addr show scope global \
    | awk '$2 !~ /^(docker|br-|veth|virbr)/ {split($4, a, "/"); print $2 " " a[1]}')

echo
echo "================ NTRIP: give this to the rover ================"
for entry in "${IPS[@]}"; do
    IF="${entry%% *}"; IP="${entry##* }"
    echo
    echo "--- via $IF ($IP) ---"
    echo " URL            : ntrip://$IP:$PORT/$MOUNT   (no username/password)"
    echo " Mission Planner: Setup > Optional > RTK/GPS Inject -> NTRIP"
    echo "                  http://$IP:$PORT/$MOUNT"
    echo " QGC            : Host $IP  Port $PORT  Mountpoint $MOUNT  (user/pass blank)"
    echo " MAVProxy       : module load ntrip"
    echo "                  ntrip set caster $IP"
    echo "                  ntrip set port $PORT"
    echo "                  ntrip set mountpoint $MOUNT"
    echo "                  ntrip start"
    echo " str2str test   : str2str -in ntrip://$IP:$PORT/$MOUNT -out file://rtcm_test.rtcm3"
done
[[ ${#IPS[@]} -gt 0 ]] || echo " (no network interface with an IPv4 address found)"

echo
if service_active; then
    echo "Caster is the $SERVICE service (logs: journalctl --user -u $SERVICE -f)."
    echo "Stop it with:  systemctl --user stop $SERVICE"
elif [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    echo "Caster running in background (pid $(cat "$PIDFILE"), log $LOG)."
    echo "Stop it with:  kill \$(cat $PIDFILE)"
else
    echo "Caster was already running before this script; stop it where you started it."
fi
