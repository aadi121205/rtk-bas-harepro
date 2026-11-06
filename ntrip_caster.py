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
"""

import argparse
import curses
import glob
import json
import os
import socket
import sys
import threading
import time
import collections
import csv

import serial

from ubx_parse import extract_frames, decode_nav_pvt, decode_nav_svin, decode_rxm_rawx, FIX_TYPE_NAMES
from skyplot import SkyModel, draw_skyplot
from geo import enu_offset

BY_ID_GLOB = "/dev/serial/by-id/usb-CubePilot_HerePro_*-if01"
FIDUCIAL_FILE = os.path.expanduser("~/.herepro_fiducial.json")


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
        self.last_pvt = None
        self.last_pvt_age = None
        self.last_svin = None
        self.log_path = None
        self.rtcm_types_last = {}
        self.rtcm_types = collections.Counter()   # RTCM msg number -> frames this second
        self.tracked = {}          # (gnssId, svId) -> max C/N0 from latest RXM-RAWX
        self.tracked_at = None
        self.client_count = 0
        self.client_addrs = []
        self.started_at = time.time()
        self.fiducial = fiducial
        self.pvt_seq = 0  # bumped each time a new NAV-PVT is decoded

        # occupation sampling (time- or count-based averaging session)
        self.sampling = False
        self.sample_live_spread = None  # running RMS spread (m) for "spread" mode
        self._spread_checked_at = 0
        self.sample_mode = None      # "time", "count" or "spread"
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
                last_pvt=self.last_pvt,
                last_pvt_age=(time.time() - self.last_pvt_age) if self.last_pvt_age else None,
                last_svin=self.last_svin,
                tracked=dict(self.tracked) if self.tracked_at and time.time() - self.tracked_at < 5 else {},
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
                sample_live_spread=self.sample_live_spread,
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
            state.rtcm_types_last = dict(state.rtcm_types)
            state.rtcm_types.clear()


def data_logger(state, stop_event, log_dir):
    """1 Hz CSV of fix quality, link health and RTCM message mix, plus an event
    log, so noise on the rover can be correlated with what the base was doing."""
    os.makedirs(log_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(log_dir, f"herepro_{stamp}.csv")
    cols = ["utc", "connected", "fixType", "numSV", "hAcc_m", "vAcc_m", "lat", "lon", "height_m",
            "svin_active", "svin_valid", "svin_dur_s", "svin_meanAcc_m", "bytes_per_s", "rtcm_frames_per_s",
            "clients", "trk_GPS", "trk_GAL", "trk_BDS", "trk_GLO", "trk_total", "rtcm_types"]
    state.log_path = path
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        last_clients = None
        while not stop_event.is_set():
            time.sleep(1)
            with state.lock:
                pvt, svin = state.last_pvt, state.last_svin
                types = dict(state.rtcm_types_last)
                fresh = state.tracked_at and time.time() - state.tracked_at < 5
                trk = [sum(1 for (g, _) in state.tracked if g == gid) if fresh else "" for gid in (0, 2, 3, 6)]
                trk_total = len(state.tracked) if fresh else ""
                row = [time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), int(state.connected),
                       pvt["fixType"] if pvt else "", pvt["numSV"] if pvt else "",
                       f"{pvt['hAcc_m']:.3f}" if pvt else "", f"{pvt['vAcc_m']:.3f}" if pvt else "",
                       f"{pvt['lat']:.9f}" if pvt else "", f"{pvt['lon']:.9f}" if pvt else "",
                       f"{pvt['height_m']:.3f}" if pvt else "",
                       int(svin["active"]) if svin else "", int(svin["valid"]) if svin else "",
                       svin["duration_s"] if svin else "", f"{svin['meanAcc_m']:.4f}" if svin else "",
                       state.bytes_per_sec, state.rtcm_frames_per_sec, state.client_count, *trk, trk_total,
                       " ".join(f"{k}:{v}" for k, v in sorted(types.items()))]
            w.writerow(row)
            f.flush()


def serial_reader(args, clients, clients_lock, stop_event, state):
    buf = bytearray()
    while not stop_event.is_set():
        path = args.serial or discover_serial_device()
        if not path:
            with state.lock:
                state.connected = False
                state.last_error = "no HerePro device found"
            time.sleep(1)
            continue

        try:
            ser = serial.Serial(path, baudrate=args.baud, timeout=1)
        except Exception as e:
            with state.lock:
                state.connected = False
                state.last_error = f"open failed: {e}"
            time.sleep(1)
            continue

        with state.lock:
            state.connected = True
            state.serial_path = path
            state.last_error = None

        while not stop_event.is_set():
            try:
                data = ser.read(2048)
            except Exception as e:
                with state.lock:
                    state.connected = False
                    state.last_error = f"read error: {e}"
                break

            if not data:
                continue

            with clients_lock:
                dead = []
                for c in clients:
                    try:
                        c.sendall(data)
                    except Exception:
                        dead.append(c)
                for d in dead:
                    clients.remove(d)
                    try:
                        d.close()
                    except Exception:
                        pass
                if dead:
                    with state.lock:
                        state.client_count = len(clients)

            buf.extend(data)
            msgs, rtcm_count, consumed = extract_frames(buf, state.rtcm_types)
            del buf[:consumed]  # each frame is processed exactly once
            if len(buf) > 8192:
                buf.clear()
            pvts = [p for c, m, p in msgs if c == 0x01 and m == 0x07]
            svins = [p for c, m, p in msgs if c == 0x01 and m == 0x3B]
            rawxs = [p for c, m, p in msgs if c == 0x02 and m == 0x15]
            with state.lock:
                state.total_bytes += len(data)
                state.bytes_this_second += len(data)
                state.rtcm_frames_this_second += rtcm_count
                if pvts:
                    try:
                        for pl in pvts:
                            state.last_pvt = decode_nav_pvt(pl)
                            state.last_pvt_age = time.time()
                            state.pvt_seq += 1
                            if state.sampling:
                                p = state.last_pvt
                                state.sample_points.append((p["lat"], p["lon"], p["height_m"]))
                                done = False
                                if state.sample_mode == "count" and len(state.sample_points) >= state.sample_target:
                                    done = True
                                elif state.sample_mode == "time" and (time.time() - state.sample_started_at) >= state.sample_target:
                                    done = True
                                elif state.sample_mode == "spread":
                                    now = time.time()
                                    if now - state._spread_checked_at >= 1 and len(state.sample_points) >= SPREAD_MIN_FIXES:
                                        state._spread_checked_at = now
                                        cur = finalize_sample(state.sample_points, state.sample_started_at)
                                        state.sample_live_spread = cur["spread_m"]
                                        if cur["spread_m"] is not None and cur["spread_m"] < state.sample_target:
                                            done = True
                                    if now - state.sample_started_at >= SPREAD_MAX_S:
                                        done = True
                                if done:
                                    state.last_sample = finalize_sample(state.sample_points, state.sample_started_at)
                                    state.sampling = False
                    except Exception:
                        pass
                if rawxs:
                    try:
                        trk = {}
                        for meas in decode_rxm_rawx(rawxs[-1]):
                            k = (meas["gnssId"], meas["svId"])
                            trk[k] = max(trk.get(k, 0), meas["cno"])
                        state.tracked = trk
                        state.tracked_at = time.time()
                    except Exception:
                        pass
                if svins:
                    try:
                        state.last_svin = decode_nav_svin(svins[-1])
                    except Exception:
                        pass

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
            conn.sendall(b"ICY 404 Not Found\r\n\r\n")
            conn.close()
            return

        conn.settimeout(0.5)  # bounded sends: a stalled client is dropped, never blocks the serial reader
        conn.sendall(b"ICY 200 OK\r\n\r\n")
        with clients_lock:
            clients.append(conn)
        with state.lock:
            state.client_count = len(clients)
            state.client_addrs.append(f"{addr[0]}:{addr[1]}")
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
        with clients_lock:
            with state.lock:
                state.client_count = len(clients)


def fmt_age(seconds):
    if seconds is None:
        return "never"
    if seconds < 2:
        return "just now"
    return f"{seconds:.0f}s ago"


TIME_PRESETS = {ord('1'): 10, ord('2'): 30, ord('3'): 60, ord('4'): 120, ord('5'): 300,
                ord('6'): 600, ord('7'): 1200, ord('8'): 1800}
SPREAD_TARGET_M = 0.5      # 'a': keep averaging until the RMS spread is below this...
SPREAD_MIN_FIXES = 30      # ...but not before this many fixes (spread of a few points means nothing)
SPREAD_MAX_S = 1800        # ...and give up (finalize what we have) after 30 min


def prompt_line(stdscr, prompt):
    """Blocking single-line text prompt on the bottom row. Returns text, or None on Esc."""
    h, w = stdscr.getmaxyx()
    buf = ""
    curses.curs_set(1)
    stdscr.timeout(-1)
    try:
        while True:
            stdscr.move(h - 2, 0)
            stdscr.clrtoeol()
            stdscr.addnstr(h - 2, 0, (prompt + buf)[-(w - 1):], w - 1, curses.A_BOLD)
            stdscr.refresh()
            ch = stdscr.getch()
            if ch in (10, 13, curses.KEY_ENTER):
                return buf
            if ch == 27:
                return None
            if ch in (curses.KEY_BACKSPACE, 127, 8):
                buf = buf[:-1]
            elif 32 <= ch < 127:
                buf += chr(ch)
    finally:
        curses.curs_set(0)
        stdscr.timeout(500)


def draw_tui(stdscr, state, stop_event, default_sample_count=50):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(500)
    flash_msg = [None, 0]  # [text, expiry_time]
    sky = SkyModel()
    sky.start()
    show_sky = False

    while not stop_event.is_set():
        c = stdscr.getch()
        if c in (ord('q'), ord('Q')):
            stop_event.set()
            break
        elif c in (ord('s'), ord('S')):
            show_sky = not show_sky
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
        elif c in (ord('t'), ord('T')):
            text = prompt_line(stdscr, "Fiducial LAT,LON,HEIGHT_M (Enter=ok, Esc=cancel): ")
            if text is None:
                flash_msg[0] = "Fiducial entry cancelled."
            else:
                try:
                    lat, lon, height = (float(x) for x in text.replace(" ", "").split(","))
                    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                        raise ValueError
                    fid = save_fiducial(lat, lon, height)
                    with state.lock:
                        state.fiducial = fid
                    flash_msg[0] = f"Fiducial set (typed): {lat:.7f}, {lon:.7f}, {height:.2f}m"
                except ValueError:
                    flash_msg[0] = "Invalid fiducial; expected LAT,LON,HEIGHT_M e.g. 28.748599,77.117063,201.5"
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
        elif c in (ord('a'), ord('A')):
            with state.lock:
                if not state.sampling:
                    state.sampling = True
                    state.sample_mode = "spread"
                    state.sample_target = SPREAD_TARGET_M
                    state.sample_started_at = time.time()
                    state.sample_points = []
                    state.sample_live_spread = None
                    state._spread_checked_at = 0
            flash_msg[0] = f"Averaging until spread < {SPREAD_TARGET_M} m (max {SPREAD_MAX_S // 60} min)."
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

        if show_sky:
            draw_skyplot(stdscr, sky, s["last_pvt"], s["tracked"])
            if flash_msg[0] and time.time() < flash_msg[1]:
                line(h - 2, f" >> {flash_msg[0]}", curses.A_BOLD)
            line(h - 1, " q=quit  s=back to status ".center(w - 1, "-"))
            stdscr.refresh()
            continue

        conn_str = "CONNECTED" if s["connected"] else "DISCONNECTED / RECONNECTING"
        conn_attr = curses.A_BOLD | (curses.color_pair(1) if s["connected"] else curses.color_pair(2))
        line(2, f" Serial device : {s['serial_path'] or '(searching...)'}")
        line(3, f" Link status   : {conn_str}", conn_attr)
        if s["last_error"]:
            line(4, f" Last error    : {s['last_error']}")

        line(6, f" NTRIP mount   : /{s['mount']}  port {s['port']}   clients connected: {s['client_count']}")
        if s["client_addrs"]:
            line(7, f" Recent clients: {', '.join(s['client_addrs'][-4:])}")

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
        line(22, f" RTCM3 frames  : {s['rtcm_frames_per_sec']}/sec")
        line(23, f" Total received: {s['total_bytes']:,} bytes   uptime {s['uptime']:.0f}s")

        line(25, " --- Occupation sample (averaged position) ---", curses.A_UNDERLINE)
        if s["sampling"]:
            if s["sample_mode"] == "spread":
                live = s["sample_live_spread"]
                live_txt = f"{live:.2f}m" if live is not None else f"(need {SPREAD_MIN_FIXES} fixes)"
                progress = f"spread {live_txt} -> target < {s['sample_target']}m, {s['sample_elapsed']:.0f}s / {SPREAD_MAX_S}s max, {s['sample_count']} fixes"
            elif s["sample_mode"] == "time":
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

        line(h - 1, " q=quit  s=sky plot  1-8=timed(10s..30m)  a=until<0.5m  r=count  x=stop  f=set fiducial  t=type fiducial  c=clear fiducial ".center(w - 1, "-"))
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


def run_headless(state, stop_event):
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
        print(f"[{time.strftime('%H:%M:%S')}] {'UP' if s['connected'] else 'DOWN'} "
              f"clients={s['client_count']} {s['bytes_per_sec']}B/s rtcm={s['rtcm_frames_per_sec']}/s | {pvt_str} | {svin_str}{fid_str}",
              flush=True)
        time.sleep(2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=2101)
    ap.add_argument("--mount", default="HEREPRO")
    ap.add_argument("--serial", default=None, help="serial device path; default: auto-discover via /dev/serial/by-id")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-ui", action="store_true", help="headless mode: periodic status lines instead of curses TUI")
    ap.add_argument("--fiducial", default=None, metavar="LAT,LON,HEIGHT_M",
                     help="set a known reference point at startup (e.g. --fiducial 28.748599,77.117063,201.5); "
                          "persists to ~/.herepro_fiducial.json. Omit to reuse a previously-saved fiducial, if any.")
    ap.add_argument("--log-dir", default="logs", help="directory for the 1 Hz CSV log (default: ./logs); use --no-log to disable")
    ap.add_argument("--no-log", action="store_true", help="disable CSV logging")
    ap.add_argument("--sample-count", type=int, default=50,
                     help="default fix count for the 'r' (count-based) occupation sample in the TUI (default: 50)")
    args = ap.parse_args()

    fiducial = load_fiducial()
    if args.fiducial:
        try:
            lat, lon, height = (float(x) for x in args.fiducial.split(","))
            fiducial = save_fiducial(lat, lon, height)
        except ValueError:
            print(f"--fiducial must be LAT,LON,HEIGHT_M (got: {args.fiducial!r})", file=sys.stderr)
            sys.exit(1)

    state = SharedState(args.mount, args.port, fiducial=fiducial)
    clients = []
    clients_lock = threading.Lock()
    stop_event = threading.Event()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(8)

    threading.Thread(target=serial_reader, args=(args, clients, clients_lock, stop_event, state), daemon=True).start()
    threading.Thread(target=accept_loop, args=(srv, args.mount, clients, clients_lock, state, stop_event), daemon=True).start()
    if not args.no_log:
        threading.Thread(target=data_logger, args=(state, stop_event, args.log_dir), daemon=True).start()
    threading.Thread(target=rate_ticker, args=(state, stop_event), daemon=True).start()

    if not args.no_ui and sys.stdout.isatty():
        try:
            run_tui(state, stop_event, args.sample_count)
        except KeyboardInterrupt:
            pass
    else:
        try:
            run_headless(state, stop_event)
        except KeyboardInterrupt:
            pass

    stop_event.set()
    srv.close()


if __name__ == "__main__":
    main()
