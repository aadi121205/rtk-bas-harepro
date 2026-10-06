#!/usr/bin/env python3
"""
NTRIP caster that bridges a local serial RTCM3 source (HerePro base, USB
CDC-ACM) to any number of NTRIP clients (NTRIP v1-style, widely compatible).

Auto-reconnects if the USB device drops and re-enumerates (which happens
often with this hardware -- see project README gotcha #5), and shows a live
terminal UI with fix/survey-in/RTCM status by default.

Usage:
  python3 ntrip_caster.py                          # auto-discover device, TUI on
  python3 ntrip_caster.py --serial /dev/ttyACM0     # pin a specific device
  python3 ntrip_caster.py --no-ui                   # headless (for systemd/logs)

Settings can also come from the environment (see systemd/base.env.example):
NTRIP_PORT, NTRIP_MOUNT, HEREPRO_SERIAL, HEREPRO_BAUD, FIDUCIAL, STATUS_S.
Command-line flags override them.
"""

import argparse
import curses
import glob
import json
import logging
import os
import socket
import sys
import threading
import time

import serial

from ubx_parse import split_frames, decode_nav_pvt, decode_nav_svin, FIX_TYPE_NAMES
from geo import enu_offset

BY_ID_GLOB = "/dev/serial/by-id/usb-CubePilot_HerePro_*-if01"
FIDUCIAL_FILE = os.path.expanduser("~/.herepro_fiducial.json")
CLIENT_SEND_TIMEOUT_S = 2  # a stalled NTRIP client is dropped instead of blocking the serial reader

log = logging.getLogger("ntrip_caster")


def discover_serial_device():
    matches = sorted(glob.glob(BY_ID_GLOB))
    return matches[0] if matches else None


def load_fiducial():
    try:
        with open(FIDUCIAL_FILE) as f:
            return json.load(f)
    except Exception:
        return None


def save_fiducial(lat, lon, height, hAcc=None):
    data = {"lat": lat, "lon": lon, "height_m": height, "hAcc_m": hAcc, "set_at": time.time()}
    try:
        with open(FIDUCIAL_FILE, "w") as f:
            json.dump(data, f, indent=2)
    except Exception:
        pass
    return data


def clear_fiducial():
    try:
        os.remove(FIDUCIAL_FILE)
    except Exception:
        pass


def finalize_sample(points, started_at):
    """Average a list of (lat, lon, height_m) samples and report the spread
    (std dev of each point's 3D distance from the mean) as a quality metric."""
    n = len(points)
    mean_lat = sum(p[0] for p in points) / n
    mean_lon = sum(p[1] for p in points) / n
    mean_height = sum(p[2] for p in points) / n

    if n > 1:
        dists = [enu_offset(mean_lat, mean_lon, mean_height, lat, lon, h)["dist_3d_m"] for lat, lon, h in points]
        spread_m = (sum(d * d for d in dists) / n) ** 0.5
    else:
        spread_m = None

    return {
        "lat": mean_lat, "lon": mean_lon, "height_m": mean_height,
        "count": n, "duration_s": time.time() - started_at,
        "spread_m": spread_m, "completed_at": time.time(),
    }


class SharedState:
    def __init__(self, mount, port, fiducial=None):
        self.lock = threading.Lock()
        self.mount = mount
        self.port = port
        self.connected = False
        self.serial_path = None
        self.last_error = None
        self.total_bytes = 0
        self.bytes_this_second = 0
        self.bytes_per_sec = 0
        self.rtcm_frames_this_second = 0
        self.rtcm_frames_per_sec = 0
        self.rtcm_types = set()      # RTCM3 message types seen since the device (re)connected
        self.last_pvt = None
        self.last_pvt_age = None
        self.last_svin = None
        self.client_count = 0
        self.client_addrs = []
        self.started_at = time.time()
        self.fiducial = fiducial
        self.pvt_seq = 0  # bumped each time a new NAV-PVT is decoded

        # occupation sampling (time- or count-based averaging session)
        self.sampling = False
        self.sample_mode = None      # "time" or "count"
        self.sample_target = None    # seconds (time) or sample count (count)
        self.sample_started_at = None
        self.sample_points = []      # list of (lat, lon, height_m)
        self.last_sample = None      # finalized {lat, lon, height_m, count, duration_s, spread_m, completed_at}

    def snapshot(self):
        with self.lock:
            return dict(
                connected=self.connected,
                serial_path=self.serial_path,
                last_error=self.last_error,
                total_bytes=self.total_bytes,
                bytes_per_sec=self.bytes_per_sec,
                rtcm_frames_per_sec=self.rtcm_frames_per_sec,
                rtcm_types=sorted(self.rtcm_types),
                last_pvt=self.last_pvt,
                last_pvt_age=(time.time() - self.last_pvt_age) if self.last_pvt_age else None,
                last_svin=self.last_svin,
                client_count=self.client_count,
                client_addrs=list(self.client_addrs),
                uptime=time.time() - self.started_at,
                mount=self.mount,
                port=self.port,
                fiducial=dict(self.fiducial) if self.fiducial else None,
                pvt_seq=self.pvt_seq,
                sampling=self.sampling,
                sample_mode=self.sample_mode,
                sample_target=self.sample_target,
                sample_elapsed=(time.time() - self.sample_started_at) if self.sample_started_at else None,
                sample_count=len(self.sample_points),
                last_sample=dict(self.last_sample) if self.last_sample else None,
            )


def rate_ticker(state, stop_event):
    """Every second, roll the this-second counters into a rate and reset them."""
    while not stop_event.is_set():
        time.sleep(1)
        with state.lock:
            state.bytes_per_sec = state.bytes_this_second
            state.bytes_this_second = 0
            state.rtcm_frames_per_sec = state.rtcm_frames_this_second
            state.rtcm_frames_this_second = 0


def _sync_client_state(state, clients):
    """Mirror the live client list into state; caller holds clients_lock."""
    with state.lock:
        state.client_count = len(clients)
        state.client_addrs = [peer for _, peer in clients]


def _record_pvt(state, payload):
    """Decode a NAV-PVT, log fix changes, feed an active occupation sample. Caller holds state.lock."""
    prev = state.last_pvt
    p = decode_nav_pvt(payload)
    state.last_pvt = p
    state.last_pvt_age = time.time()
    state.pvt_seq += 1
    if prev is None or prev["fixType"] != p["fixType"]:
        log.info("fix: %s -> %s (%d sats, hAcc %.2f m)",
                 FIX_TYPE_NAMES.get(prev["fixType"], prev["fixType"]) if prev else "none",
                 FIX_TYPE_NAMES.get(p["fixType"], p["fixType"]), p["numSV"], p["hAcc_m"])
    if state.sampling:
        state.sample_points.append((p["lat"], p["lon"], p["height_m"]))
        done = False
        if state.sample_mode == "count" and len(state.sample_points) >= state.sample_target:
            done = True
        elif state.sample_mode == "time" and (time.time() - state.sample_started_at) >= state.sample_target:
            done = True
        if done:
            state.last_sample = finalize_sample(state.sample_points, state.sample_started_at)
            state.sampling = False


def _record_svin(state, payload):
    """Decode a NAV-SVIN and log when survey-in becomes (or stops being) valid. Caller holds state.lock."""
    prev = state.last_svin
    s = decode_nav_svin(payload)
    state.last_svin = s
    if s["valid"] and not (prev and prev["valid"]):
        log.info("survey-in VALID: acc %.3f m after %d s, %d observations", s["meanAcc_m"], s["duration_s"], s["observations"])
    elif prev and prev["valid"] and not s["valid"]:
        log.warning("survey-in no longer valid (receiver restarted it?)")


def serial_reader(args, clients, clients_lock, stop_event, state):
    buf = bytearray()
    last_problem = None  # log each distinct "can't read the device" reason once, not every second

    def problem(msg, detail):
        nonlocal last_problem
        with state.lock:
            state.connected = False
            state.last_error = msg
        if msg != last_problem:
            log.warning("%s (%s) -- retrying every 1 s", msg, detail)
            last_problem = msg

    while not stop_event.is_set():
        path = args.serial or discover_serial_device()
        if not path:
            problem("no HerePro device found", args.serial or BY_ID_GLOB)
            time.sleep(1)
            continue

        try:
            ser = serial.Serial(path, baudrate=args.baud, timeout=1)
        except Exception as e:
            problem(f"open failed: {e}", path)
            time.sleep(1)
            continue

        last_problem = None
        buf.clear()
        log.info("HerePro connected: %s (%s)", path, os.path.realpath(path))
        with state.lock:
            state.connected = True
            state.serial_path = path
            state.last_error = None

        while not stop_event.is_set():
            try:
                data = ser.read(2048)
            except Exception as e:
                log.warning("HerePro lost: %s", path)
                problem(f"read error: {e}", path)
                with state.lock:
                    # nothing is being sent now, and a replug reboots the HerePro anyway:
                    # survey-in and RTCM output start over
                    state.rtcm_types = set()
                    state.last_pvt = None
                    state.last_svin = None
                break

            if not data:
                continue

            with clients_lock:
                dead = []
                for c in clients:
                    try:
                        c[0].sendall(data)
                    except Exception:
                        dead.append(c)
                for d in dead:
                    clients.remove(d)
                    try:
                        d[0].close()
                    except Exception:
                        pass
                    log.info("NTRIP client disconnected: %s (%d connected)", d[1], len(clients))
                if dead:
                    _sync_client_state(state, clients)

            buf.extend(data)
            frames, used = split_frames(buf)
            del buf[:used]
            if len(buf) > 8192:
                del buf[:-1024]  # never happens with real frames; guards against a garbage stream
            with state.lock:
                state.total_bytes += len(data)
                state.bytes_this_second += len(data)
                for kind, key, payload in frames:
                    try:
                        if kind == "rtcm":
                            state.rtcm_frames_this_second += 1
                            if key not in state.rtcm_types:
                                state.rtcm_types.add(key)
                                note = " -- base position, rovers can now use the corrections" if key in (1005, 1006) else ""
                                log.info("RTCM3 %d now being sent%s (types: %s)", key, note,
                                         " ".join(map(str, sorted(state.rtcm_types))))
                        elif key == (0x01, 0x07):
                            _record_pvt(state, payload)
                        elif key == (0x01, 0x3B):
                            _record_svin(state, payload)
                    except Exception:
                        log.debug("could not decode %s %s", kind, key, exc_info=True)

        try:
            ser.close()
        except Exception:
            pass


def handle_client(conn, addr, mount, clients, clients_lock, state):
    try:
        conn.settimeout(5)
        request = b""
        while b"\r\n\r\n" not in request and len(request) < 4096:
            chunk = conn.recv(1024)
            if not chunk:
                conn.close()
                return
            request += chunk
        first_line = request.split(b"\r\n", 1)[0].decode(errors="replace")

        if first_line.startswith("GET / "):
            sourcetable = (
                f"STR;{mount};HerePro RTK Base;RTCM 3;;2;GPS+GAL+BDS;HerePro;IND;0.00;0.00;0;0;sNTRIP;none;N;N;0;\r\n"
                "ENDSOURCETABLE\r\n"
            )
            resp = (
                "SOURCETABLE 200 OK\r\nServer: SimplePyCaster\r\nContent-Type: text/plain\r\n"
                f"Content-Length: {len(sourcetable)}\r\n\r\n" + sourcetable
            )
            conn.sendall(resp.encode())
            conn.close()
            return

        if f"GET /{mount}" not in first_line:
            log.warning("NTRIP client %s:%d asked for unknown mount: %s", addr[0], addr[1], first_line)
            conn.sendall(b"ICY 404 Not Found\r\n\r\n")
            conn.close()
            return

        conn.settimeout(CLIENT_SEND_TIMEOUT_S)
        conn.sendall(b"ICY 200 OK\r\n\r\n")
        peer = f"{addr[0]}:{addr[1]}"
        with clients_lock:
            clients.append((conn, peer))
            _sync_client_state(state, clients)
            n = len(clients)
        log.info("NTRIP client connected: %s (%d connected)", peer, n)
    except Exception:
        try:
            conn.close()
        except Exception:
            pass


def accept_loop(srv, mount, clients, clients_lock, state, stop_event):
    srv.settimeout(1)
    while not stop_event.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        t = threading.Thread(target=handle_client, args=(conn, addr, mount, clients, clients_lock, state), daemon=True)
        t.start()


def fmt_age(seconds):
    if seconds is None:
        return "never"
    if seconds < 2:
        return "just now"
    return f"{seconds:.0f}s ago"


TIME_PRESETS = {ord('1'): 10, ord('2'): 30, ord('3'): 60, ord('4'): 120, ord('5'): 300}


def draw_tui(stdscr, state, stop_event, default_sample_count=50):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(500)
    flash_msg = [None, 0]  # [text, expiry_time]

    while not stop_event.is_set():
        c = stdscr.getch()
        if c in (ord('q'), ord('Q')):
            stop_event.set()
            break
        elif c in (ord('f'), ord('F')):
            with state.lock:
                pvt = state.last_pvt
                sample = state.last_sample
            if sample:
                fid = save_fiducial(sample["lat"], sample["lon"], sample["height_m"])
                with state.lock:
                    state.fiducial = fid
                spread_txt = f", spread {sample['spread_m']*100:.1f}cm" if sample["spread_m"] is not None else ""
                flash_msg[0] = f"Fiducial set from {sample['count']}-sample average{spread_txt}."
            elif pvt:
                fid = save_fiducial(pvt["lat"], pvt["lon"], pvt["height_m"], pvt["hAcc_m"])
                with state.lock:
                    state.fiducial = fid
                flash_msg[0] = "Fiducial set from current (single) position -- consider an averaged sample ('1'-'5' or 'r') for less noise."
            else:
                flash_msg[0] = "Can't set fiducial: no fix yet."
            flash_msg[1] = time.time() + 4
        elif c in (ord('c'), ord('C')):
            clear_fiducial()
            with state.lock:
                state.fiducial = None
            flash_msg[0] = "Fiducial cleared."
            flash_msg[1] = time.time() + 3
        elif c in TIME_PRESETS:
            secs = TIME_PRESETS[c]
            with state.lock:
                if not state.sampling:
                    state.sampling = True
                    state.sample_mode = "time"
                    state.sample_target = secs
                    state.sample_started_at = time.time()
                    state.sample_points = []
            flash_msg[0] = f"Started {secs}s timed sample."
            flash_msg[1] = time.time() + 3
        elif c in (ord('r'), ord('R')):
            with state.lock:
                if not state.sampling:
                    state.sampling = True
                    state.sample_mode = "count"
                    state.sample_target = default_sample_count
                    state.sample_started_at = time.time()
                    state.sample_points = []
            flash_msg[0] = f"Started {default_sample_count}-fix sample."
            flash_msg[1] = time.time() + 3
        elif c in (ord('x'), ord('X')):
            with state.lock:
                if state.sampling and state.sample_points:
                    state.last_sample = finalize_sample(state.sample_points, state.sample_started_at)
                state.sampling = False
            flash_msg[0] = "Sample stopped/finalized early."
            flash_msg[1] = time.time() + 3

        s = state.snapshot()
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        def line(y, text, attr=0):
            if 0 <= y < h:
                stdscr.addnstr(y, 0, text, max(w - 1, 0), attr)

        line(0, " HerePro RTK Base -> NTRIP  ".center(w - 1, "="), curses.A_BOLD)

        conn_str = "CONNECTED" if s["connected"] else "DISCONNECTED / RECONNECTING"
        conn_attr = curses.A_BOLD | (curses.color_pair(1) if s["connected"] else curses.color_pair(2))
        line(2, f" Serial device : {s['serial_path'] or '(searching...)'}")
        line(3, f" Link status   : {conn_str}", conn_attr)
        if s["last_error"]:
            line(4, f" Last error    : {s['last_error']}")

        line(6, f" NTRIP mount   : /{s['mount']}  port {s['port']}   clients connected: {s['client_count']}")
        if s["client_addrs"]:
            line(7, f" Clients       : {', '.join(s['client_addrs'][-4:])}")

        line(9, " --- GNSS fix (NAV-PVT) ---", curses.A_UNDERLINE)
        pvt = s["last_pvt"]
        if pvt:
            fix_name = FIX_TYPE_NAMES.get(pvt["fixType"], str(pvt["fixType"]))
            fix_attr = curses.color_pair(1) if pvt["fixType"] >= 3 else curses.color_pair(2)
            line(10, f" Fix type      : {fix_name}   Satellites: {pvt['numSV']}", fix_attr)
            line(11, f" Position      : {pvt['lat']:.7f}, {pvt['lon']:.7f}   alt {pvt['height_m']:.1f}m")
            line(12, f" Accuracy      : hAcc {pvt['hAcc_m']:.2f}m  vAcc {pvt['vAcc_m']:.2f}m")
            line(13, f" Last update   : {fmt_age(s['last_pvt_age'])}")
        else:
            line(10, " (no NAV-PVT received yet)")

        line(15, " --- Survey-in (NAV-SVIN) ---", curses.A_UNDERLINE)
        svin = s["last_svin"]
        if svin:
            status = "VALID (converged)" if svin["valid"] else ("IN PROGRESS" if svin["active"] else "STOPPED / NOT VALID")
            status_attr = curses.color_pair(1) if svin["valid"] else curses.color_pair(2)
            line(16, f" Status        : {status}", status_attr)
            line(17, f" Duration      : {svin['duration_s']}s   Observations: {svin['observations']}")
            line(18, f" Current acc   : {svin['meanAcc_m']:.3f}m")
        else:
            line(16, " (no NAV-SVIN received yet)")

        line(20, " --- Data flow ---", curses.A_UNDERLINE)
        line(21, f" Throughput    : {s['bytes_per_sec']} bytes/sec  ({s['bytes_per_sec']*8} bps)")
        types_txt = " ".join(map(str, s["rtcm_types"])) or "none yet"
        base_txt = "" if (1005 in s["rtcm_types"] or 1006 in s["rtcm_types"]) else "  (no 1005 base position yet)"
        line(22, f" RTCM3 frames  : {s['rtcm_frames_per_sec']}/sec   types: {types_txt}{base_txt}")
        line(23, f" Total received: {s['total_bytes']:,} bytes   uptime {s['uptime']:.0f}s")

        line(25, " --- Occupation sample (averaged position) ---", curses.A_UNDERLINE)
        if s["sampling"]:
            if s["sample_mode"] == "time":
                progress = f"{s['sample_elapsed']:.0f}s / {s['sample_target']}s"
            else:
                progress = f"{s['sample_count']} / {s['sample_target']} fixes"
            line(26, f" SAMPLING ({s['sample_mode']})... {progress}   [x to stop early]", curses.A_BOLD | curses.color_pair(2))
        elif s["last_sample"]:
            ls = s["last_sample"]
            spread_txt = f"{ls['spread_m']*100:.1f}cm" if ls["spread_m"] is not None else "n/a"
            line(26, f" Last sample   : {ls['count']} fixes over {ls['duration_s']:.0f}s, spread \xb1{spread_txt}")
            line(27, f"                 {ls['lat']:.7f}, {ls['lon']:.7f}  alt {ls['height_m']:.2f}m")
        else:
            line(26, " No sample yet. Press 1-5 for a 10/30/60/120/300s timed average, or 'r' for a fixed-count average.")

        line(29, " --- Fiducial / correction from reference point ---", curses.A_UNDERLINE)
        fid = s["fiducial"]
        if fid:
            age_s = time.time() - fid["set_at"]
            line(30, f" Fiducial      : {fid['lat']:.7f}, {fid['lon']:.7f}  alt {fid['height_m']:.2f}m  (set {age_s/60:.0f}m ago)")
            if pvt:
                off = enu_offset(fid["lat"], fid["lon"], fid["height_m"], pvt["lat"], pvt["lon"], pvt["height_m"])
                err_attr = curses.color_pair(1) if off["dist_3d_m"] < 0.10 else (
                    curses.color_pair(2) if off["dist_3d_m"] < 0.5 else curses.color_pair(3))
                line(31, f" Live offset   : E {off['east_m']:+.3f}m  N {off['north_m']:+.3f}m  U {off['up_m']:+.3f}m")
                line(32, f" Error vs fid. : {off['dist_2d_m']*100:.1f}cm horizontal, {off['dist_3d_m']*100:.1f}cm 3D", err_attr)
            else:
                line(31, " (no live fix yet to compare)")
        else:
            line(30, " No fiducial set. Press 'f' to set one (uses last sample average if present, else live fix).")

        if flash_msg[0] and time.time() < flash_msg[1]:
            line(h - 2, f" >> {flash_msg[0]}", curses.A_BOLD)

        line(h - 1, " q=quit  1-5/r=sample(time/count)  x=stop  f=set fiducial  c=clear fiducial ".center(w - 1, "-"))
        stdscr.refresh()


def run_tui(state, stop_event, default_sample_count=50):
    def _main(stdscr):
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_GREEN, -1)
        curses.init_pair(2, curses.COLOR_YELLOW, -1)
        curses.init_pair(3, curses.COLOR_RED, -1)
        draw_tui(stdscr, state, stop_event, default_sample_count)
    curses.wrapper(_main)


def run_headless(state, stop_event, every=2.0):
    while not stop_event.is_set():
        s = state.snapshot()
        pvt = s["last_pvt"]
        svin = s["last_svin"]
        pvt_str = f"fix={FIX_TYPE_NAMES.get(pvt['fixType'], pvt['fixType'])} sats={pvt['numSV']} hAcc={pvt['hAcc_m']:.1f}m" if pvt else "no fix yet"
        svin_str = f"svin_acc={svin['meanAcc_m']:.2f}m valid={svin['valid']}" if svin else "no survey-in data yet"
        fid_str = ""
        if s["fiducial"] and pvt:
            off = enu_offset(s["fiducial"]["lat"], s["fiducial"]["lon"], s["fiducial"]["height_m"],
                              pvt["lat"], pvt["lon"], pvt["height_m"])
            fid_str = f" | err_vs_fiducial={off['dist_3d_m']*100:.1f}cm"
        base = "yes" if (1005 in s["rtcm_types"] or 1006 in s["rtcm_types"]) else "no"
        log.info("%s clients=%d %dB/s rtcm=%d/s base_pos=%s | %s | %s%s",
                 "UP" if s["connected"] else "DOWN", s["client_count"], s["bytes_per_sec"],
                 s["rtcm_frames_per_sec"], base, pvt_str, svin_str, fid_str)
        stop_event.wait(every)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=int(os.getenv("NTRIP_PORT", "2101")))
    ap.add_argument("--mount", default=os.getenv("NTRIP_MOUNT", "HEREPRO"))
    ap.add_argument("--serial", default=os.getenv("HEREPRO_SERIAL") or None,
                     help="serial device path; default: auto-discover via /dev/serial/by-id")
    ap.add_argument("--baud", type=int, default=int(os.getenv("HEREPRO_BAUD", "115200")))
    ap.add_argument("--no-ui", action="store_true", help="headless mode: periodic status lines instead of curses TUI")
    ap.add_argument("--status-every", type=float, default=float(os.getenv("STATUS_S", "2")),
                     help="headless mode: seconds between status lines (default: 2)")
    ap.add_argument("--fiducial", default=os.getenv("FIDUCIAL") or None, metavar="LAT,LON,HEIGHT_M",
                     help="set a known reference point at startup (e.g. --fiducial 28.748599,77.117063,201.5); "
                          "persists to ~/.herepro_fiducial.json. Omit to reuse a previously-saved fiducial, if any.")
    ap.add_argument("--sample-count", type=int, default=50,
                     help="default fix count for the 'r' (count-based) occupation sample in the TUI (default: 50)")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(message)s", datefmt="%H:%M:%S")

    fiducial = load_fiducial()
    if args.fiducial:
        try:
            lat, lon, height = (float(x) for x in args.fiducial.split(","))
            fiducial = save_fiducial(lat, lon, height)
        except ValueError:
            log.error("--fiducial / FIDUCIAL must be LAT,LON,HEIGHT_M (got: %r)", args.fiducial)
            return 1

    state = SharedState(args.mount, args.port, fiducial=fiducial)
    clients = []
    clients_lock = threading.Lock()
    stop_event = threading.Event()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind(("0.0.0.0", args.port))
    except OSError as e:
        log.error("can't listen on port %d: %s -- is another caster already running? "
                  "(check: systemctl --user status herepro-ntrip)", args.port, e)
        return 1
    srv.listen(8)
    log.info("NTRIP caster on 0.0.0.0:%d/%s, serial %s", args.port, args.mount, args.serial or f"auto ({BY_ID_GLOB})")
    if fiducial:
        log.info("fiducial: %.8f, %.8f, %.3f m", fiducial["lat"], fiducial["lon"], fiducial["height_m"])

    use_tui = not args.no_ui and sys.stdout.isatty()
    if use_tui:
        logging.getLogger().handlers = [logging.NullHandler()]  # curses owns the screen

    threading.Thread(target=serial_reader, args=(args, clients, clients_lock, stop_event, state), daemon=True).start()
    threading.Thread(target=accept_loop, args=(srv, args.mount, clients, clients_lock, state, stop_event), daemon=True).start()
    threading.Thread(target=rate_ticker, args=(state, stop_event), daemon=True).start()

    if use_tui:
        try:
            run_tui(state, stop_event, args.sample_count)
        except KeyboardInterrupt:
            pass
    else:
        try:
            run_headless(state, stop_event, args.status_every)
        except KeyboardInterrupt:
            pass

    stop_event.set()
    srv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
