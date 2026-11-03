"""Sky plot (azimuth/elevation "horizon projection") for the TUI.

The HerePro's USB stream carries UBX-RXM-RAWX (which satellites it is actually
tracking + C/N0) but not NAV-SAT, so it never reports azimuth/elevation. We
get geometry by predicting every constellation satellite with SGP4 from public
orbital elements (CelesTrak, cached on disk) for the receiver's own position,
then overlaying what the receiver is really tracking.

Accuracy is a degree or so -- plenty for "where is it in the sky / is the
receiver ignoring a satellite that is clearly overhead".

Satellite identity: GPS (PRN) and BeiDou (Cxx) element sets carry the id that
matches the receiver's svId, so those are labelled and matched exactly.
Galileo / GLONASS element sets don't carry a usable svId, so those are drawn
unlabelled (lowercase letter) and only counted, not matched.
"""
import curses
import json
import math
import os
import re
import threading
import time
import urllib.request
from datetime import datetime, timezone

from geo import lla_to_ecef, ecef_delta_to_enu

CACHE_DIR = os.path.expanduser("~/.cache/herepro")
CACHE_MAX_AGE_S = 6 * 3600
GROUPS = ("gps-ops", "glo-ops", "galileo", "beidou")

# gnssId (UBX numbering) -> (letter, name)
GNSS_INFO = {0: ("G", "GPS"), 2: ("E", "GAL"), 3: ("C", "BDS"), 6: ("R", "GLO")}
GROUP_GNSS = {"gps-ops": 0, "galileo": 2, "beidou": 3, "glo-ops": 6}


def _label_prn(gnss, name):
    if gnss == 0:
        m = re.search(r"PRN\s*(\d+)", name)
    elif gnss == 3:
        m = re.search(r"\(C(\d+)\)", name)
    else:
        return None
    return int(m.group(1)) if m else None


def _gmst(dt):
    """Greenwich mean sidereal time (rad) for a UTC datetime."""
    jd = dt.timestamp() / 86400.0 + 2440587.5
    t = (jd - 2451545.0) / 36525.0
    g = (67310.54841 + (876600 * 3600 + 8640184.812866) * t
         + 0.093104 * t * t - 6.2e-6 * t ** 3)
    return math.radians((g % 86400) / 240.0)


class SkyModel:
    def __init__(self):
        self.sats = []        # [(gnss, prn|None, Satrec)]
        self.status = "orbit data: loading..."
        self._lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._load, daemon=True).start()

    def _fetch_group(self, group):
        os.makedirs(CACHE_DIR, exist_ok=True)
        path = os.path.join(CACHE_DIR, f"omm_{group}.json")
        fresh = os.path.exists(path) and time.time() - os.path.getmtime(path) < CACHE_MAX_AGE_S
        if not fresh:
            try:
                url = f"https://celestrak.org/NORAD/elements/gp.php?GROUP={group}&FORMAT=json"
                with urllib.request.urlopen(url, timeout=20) as r:
                    data = r.read()
                json.loads(data)  # validate before replacing the cache
                with open(path, "wb") as f:
                    f.write(data)
            except Exception:
                if not os.path.exists(path):
                    raise
        with open(path) as f:
            return f.read()

    def _load(self):
        try:
            from sgp4 import omm
            from sgp4.api import Satrec
        except ImportError:
            self.status = "orbit data: install sgp4 (pip install sgp4)"
            return
        sats, errors = [], 0
        for group in GROUPS:
            try:
                text = self._fetch_group(group)
                for fields in json.loads(text):
                    sat = Satrec()
                    omm.initialize(sat, fields)
                    gnss = GROUP_GNSS[group]
                    sats.append((gnss, _label_prn(gnss, fields.get("OBJECT_NAME", "")), sat))
            except Exception:
                errors += 1
        with self._lock:
            self.sats = sats
        self.status = (f"orbit data: {len(sats)} sats" if sats else "orbit data: unavailable (offline? no cache)") \
            + (f" ({errors} group(s) failed)" if errors and sats else "")

    def predict(self, lat, lon, height_m, now=None):
        """Return [(gnss, prn|None, az_deg, el_deg)] for satellites above the horizon."""
        now = now or datetime.now(timezone.utc)
        with self._lock:
            sats = self.sats
        if not sats:
            return []
        from sgp4.api import jday
        jd, fr = jday(now.year, now.month, now.day, now.hour, now.minute, now.second + now.microsecond / 1e6)
        g = _gmst(now)
        cg, sg = math.cos(g), math.sin(g)
        ox, oy, oz = lla_to_ecef(lat, lon, height_m)
        out = []
        for gnss, prn, sat in sats:
            err, r, _v = sat.sgp4(jd, fr)
            if err:
                continue
            x, y, z = r[0] * 1000, r[1] * 1000, r[2] * 1000   # TEME km -> m
            ex, ey, ez = cg * x + sg * y, -sg * x + cg * y, z   # TEME -> ECEF (GMST spin)
            e, n, u = ecef_delta_to_enu(ex - ox, ey - oy, ez - oz, lat, lon)
            rng = math.sqrt(e * e + n * n + u * u)
            el = math.degrees(math.asin(u / rng))
            if el < 0:
                continue
            out.append((gnss, prn, math.degrees(math.atan2(e, n)) % 360, el))
        return out


def draw_skyplot(stdscr, sky, pvt, tracked):
    """tracked: {(gnssId, svId): max_cno} from the latest RXM-RAWX."""
    h, w = stdscr.getmaxyx()

    def put(y, x, text, attr=0):
        if 0 <= y < h - 2 and 0 <= x < w - 1:
            stdscr.addnstr(y, x, text, w - 1 - x, attr)

    put(1, 1, " --- Sky plot (horizon projection, N up) ---", curses.A_UNDERLINE)
    if not pvt or pvt["fixType"] < 2:
        put(3, 2, "Waiting for a position fix to place satellites...")
        put(5, 2, sky.status)
        return
    sats = sky.predict(pvt["lat"], pvt["lon"], pvt["height_m"])
    if not sats:
        put(3, 2, "No satellites to draw -- " + sky.status, curses.color_pair(2) | curses.A_BOLD)
        put(4, 2, "(run the caster with .venv/bin/python ntrip_caster.py so sgp4 is available)", curses.color_pair(2))

    ry = max(4, min((h - 7) // 2, 14))
    rx = ry * 2
    cy, cx = 3 + ry, 2 + rx + 2
    # elevation rings: 0 (horizon), 30, 60 degrees
    for el_ring in (0, 30, 60):
        r = (90 - el_ring) / 90.0
        for a in range(0, 360, 4 if el_ring == 0 else 6):
            x = cx + int(round(rx * r * math.sin(math.radians(a))))
            y = cy - int(round(ry * r * math.cos(math.radians(a))))
            put(y, x, "." if el_ring == 0 else "·", curses.A_DIM)
    put(cy, cx, "+", curses.A_DIM)
    put(cy - ry - 1, cx, "N", curses.A_BOLD)
    put(cy + ry + 1, cx, "S", curses.A_BOLD)
    put(cy, cx - rx - 2, "W", curses.A_BOLD)
    put(cy, cx + rx + 2, "E", curses.A_BOLD)

    have_trk = bool(tracked)
    stats = {g: [0, 0] for g in GNSS_INFO}    # gnss -> [tracked, visible(predicted)]
    rows = []
    for gnss, prn, az, el in sats:
        letter = GNSS_INFO[gnss][0]
        cno = tracked.get((gnss, prn)) if prn is not None else None
        stats[gnss][1] += 1
        if prn is not None:
            if cno is not None:
                stats[gnss][0] += 1
        r = (90 - el) / 90.0
        x = cx + int(round(rx * r * math.sin(math.radians(az))))
        y = cy - int(round(ry * r * math.cos(math.radians(az))))
        if prn is None:
            put(y, x, letter.lower(), curses.A_DIM)
        elif not have_trk:
            rows.append((0, y, x, f"{letter}{prn:02d}", curses.A_BOLD))
        elif cno is None:
            rows.append((0, y, x, f"{letter}{prn:02d}", curses.color_pair(3)))
        else:
            col = curses.color_pair(1) if cno >= 35 else curses.color_pair(2)
            rows.append((1, y, x, f"{letter}{prn:02d}", col | curses.A_BOLD))
    for _, y, x, label, attr in sorted(rows):   # tracked drawn last (on top)
        put(y, x - 1, label, attr)

    # side panel
    px = cx + rx + 8
    put(3, px, "Constellation   tracked / predicted above horizon", curses.A_BOLD)
    row = 4
    for gnss, (letter, name) in GNSS_INFO.items():
        trk_all = sorted(s for (g, s) in tracked if g == gnss)
        vis = stats[gnss][1]
        if not have_trk:
            txt = f" - / {vis:2d}"
        elif gnss in (0, 3):
            txt = f"{stats[gnss][0]:2d} / {vis:2d}"
        else:
            txt = f"{len(trk_all):2d} / {vis:2d}"
        attr = curses.color_pair(3) | curses.A_BOLD if (have_trk and vis > 0 and not trk_all) else 0
        note = "  <-- NOT TRACKED (enable in GPS_GNSS_MODE)" if attr else ""
        put(row, px, f"{letter} {name}        {txt}{note}", attr)
        row += 1
    row += 1
    if not have_trk:
        put(row, px, f"No RXM-RAWX in stream: prediction only. Receiver uses {pvt['numSV']} SVs.", curses.color_pair(2)); row += 1
        put(row, px, "Compare that to the predicted total above.", curses.color_pair(2)); row += 2
    put(row, px, "Legend:", curses.A_BOLD); row += 1
    put(row, px, "G05 green  tracked, C/N0 >= 35 dB-Hz", curses.color_pair(1)); row += 1
    put(row, px, "G05 yellow tracked, C/N0 < 35 dB-Hz", curses.color_pair(2)); row += 1
    put(row, px, "G05 red    above horizon, NOT tracked", curses.color_pair(3)); row += 1
    put(row, px, "e r  dim   Galileo/GLONASS prediction (no id match)", curses.A_DIM); row += 2
    put(row, px, "Rings: horizon / 30 / 60 deg, centre = zenith", curses.A_DIM); row += 1
    put(row, px, sky.status, curses.A_DIM)
