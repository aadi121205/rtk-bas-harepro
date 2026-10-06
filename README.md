# HerePro RTK Base -> NTRIP

Turns a CubePilot HerePro (used as a ground base station) into an NTRIP caster
reachable over the network, by reading its USB serial output directly and
re-serving it as RTCM3 corrections.

## Current status (as of setup)

- HerePro parameters `B_ENABLE=1`, `SERIAL_PASS1=1`, and **`CAN_NODE=100`**
  are written to the HerePro's own flash (done via Mission Planner's UAVCAN
  page, node 124 at the time, one-time CAN connection to a Cube each time).
  These persist across power cycles.
- **`CAN_NODE=100` removes the CAN-master-at-boot requirement entirely.**
  Confirmed by testing: HerePro plugged into USB with *zero* Cube/autopilot
  present anywhere -- it still came up producing real UBX (NAV-PVT, NAV-SVIN,
  RXM-RAWX) with no "waiting for dynamic node ID allocation" hang. It now
  self-assigns node ID 100 at boot via the `CAN_NODE` static-ID parameter
  (see `Tools/AP_Periph/can.cpp` in ArduPilot -- `g.can_node` bypasses DNA
  entirely when set). No CAN master, adapter, or Cube needed ever again for
  base-only USB operation.
- With `B_ENABLE`/`SERIAL_PASS1` set, on every boot the HerePro runs
  survey-in automatically and streams UBX **and** RTCM3 correction messages
  out its USB serial port -- no per-boot manual configuration needed.
- `ntrip_caster.py` reads that serial port and re-serves it as an NTRIP
  stream over TCP. Run by hand it only lives as long as your terminal; install
  the `herepro-ntrip` systemd user service (see "Running it as a service"
  below) to have it start at boot and stay up.

## Hard-won gotchas (don't relearn these)

1. **~~The HerePro needs a DroneCAN master present at boot~~ -- FIXED.**
   Without a static node ID, power-cycling with no CAN link (just USB) gets
   it stuck printing "Waiting for dynamic node ID allocation..." forever and
   it never activates UBX/RTCM output -- true for the first several days of
   this project. Fixed permanently by setting `CAN_NODE=100` (see "Current
   status" above) -- confirmed working standalone with zero CAN hardware
   present. If you're setting this up fresh: still need one *temporary* CAN
   connection to a Cube to write `B_ENABLE`, `SERIAL_PASS1`, and `CAN_NODE`
   once each, but after that the Cube is never needed again.
2. **This USB serial port is a one-way mirror, not real two-way UBX
   passthrough.** Any bytes written into it (polls, UBX-CFG-VALSET, etc.)
   seem to halt the periodic broadcast until the next power cycle. Don't
   write to the port while `ntrip_caster.py` (or anything else) depends on
   its live output -- `tools/configure_base_valset.py` is kept here for
   reference but isn't a reliable way to configure the receiver; use Mission
   Planner's UAVCAN parameter page instead.
3. **Only one process can usefully read the serial port at a time.** To
   check status while `ntrip_caster.py` is running, connect to the caster
   itself as an NTRIP client (`check_status.py` does this by default) rather
   than opening `/dev/ttyACMx` directly -- two readers fight over the same
   bytes.
4. **The `dronecan` Python library's multiprocessing-based CAN driver is
   broken on very new Python versions** (hit this on 3.14 -- `multiprocessing`
   internals changed enough that its subprocess I/O silently fails). Raw
   MAVLink `CAN_FRAME` send/recv via `pymavlink` directly works fine and is
   what `tools/raw_can_frames.py` / `tools/decode_can_ids.py` use instead.
5. **The USB tty device node number changes on every reconnect**
   (`/dev/ttyACM0` -> `/dev/ttyACM1` -> ...). Check `/dev/serial/by-id/` for
   the stable `usb-CubePilot_HerePro_<serial>-if01` symlink, or re-run
   `ls -la /dev/serial/by-id/` after any replug.
6. **`MAV_CMD_CAN_FORWARD` + MAVLink `CAN_FRAME` only reliably worked
   RECEIVE-side from this machine, not TX/inject.** `tools/raw_can_frames.py`
   and `tools/decode_can_ids.py` can *read* real bus traffic fine. But sending
   a `CAN_FRAME` (`tools/dronecan_param_client.py`, used to try writing
   `CAN_NODE` without Mission Planner) never got a response or even a bus
   echo of the sent frame -- tried both MAVLink component IDs (0 and 1),
   confirmed the connection was stable (steady ~2Hz heartbeats from two
   components) and confirmed the DroneCAN transfer encoding was correct
   (used the `dronecan` library's own `Transfer.to_frames()`/`from_frames()`
   for that part, not hand-rolled). Never root-caused whether this is an
   ArduPilot firmware limitation, a missing precondition, or something else.
   **For any DroneCAN parameter WRITE (not just read), use Mission Planner's
   UAVCAN page on Windows -- that path is proven reliable (used successfully
   for `SERIAL_PASS1` and `B_ENABLE`).** Don't burn more time on the raw-frame
   TX path without new information.
   Also note: importing `pymavlink` needs `MAVLINK20=1` and
   `MAVLINK_DIALECT=ardupilotmega` set in the environment *before* import --
   otherwise it silently falls back to MAVLink v1.0, which doesn't have
   `CAN_FRAME` at all (id 386, a v2-only message), and every CAN-related
   send/recv method just silently doesn't exist.

## Usage

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt

# start the caster with the live TUI (run this in YOUR terminal, not backgrounded --
# curses needs a real tty). Device path is auto-discovered via /dev/serial/by-id,
# no need to pass --serial unless you want to pin a specific one.
.venv/bin/python ntrip_caster.py

# headless mode instead (for systemd, logging, or a backgrounded/non-interactive
# shell that has no real tty -- auto-detected and used automatically in that case,
# or force it explicitly). Settings also come from env vars (NTRIP_PORT,
# NTRIP_MOUNT, HEREPRO_SERIAL, HEREPRO_BAUD, FIDUCIAL, STATUS_S):
.venv/bin/python ntrip_caster.py --no-ui

# in another terminal: one-shot status check without disturbing the caster
.venv/bin/python check_status.py --seconds 20
```

The TUI shows: serial device + link status (auto-reconnects if the HerePro
drops off USB and re-enumerates -- very common with this hardware, see
gotcha #5), NTRIP mountpoint/port/connected client count, live NAV-PVT (fix
type, satellite count, position, accuracy), live NAV-SVIN (survey-in
progress/accuracy/valid), data throughput (bytes/sec, RTCM3 frames/sec), an
occupation-sample averager, and a fiducial/correction panel. Press `q` to
quit. Headless mode prints the same info (plus fiducial error, if set) as
periodic log lines instead, for non-interactive use.

**Occupation sampling and fiducial correction** (for checking real
ground-truth error, e.g. against a known survey point or another receiver):
- `1`-`5` in the TUI: start a timed average (10/30/60/120/300s) of incoming
  fixes.
- `r`: start a fixed-count average (default 50 fixes; change with
  `--sample-count N`).
- `x`: stop/finalize the current sample early (uses whatever was collected).
- Averaging is plain arithmetic mean of lat/lon/height across the session,
  with the spread reported as the RMS 3D distance (via `geo.py`'s WGS84
  ECEF/ENU math) of each sample from that mean -- a real precision estimate,
  not just the receiver's own self-reported accuracy.
- `f`: set the fiducial (reference point). Uses the last completed sample
  average if one exists (much less noisy), otherwise falls back to the
  single live fix. Persists to `~/.herepro_fiducial.json` and is reloaded
  automatically on the next run (or set directly via
  `--fiducial LAT,LON,HEIGHT_M`, e.g. from a known surveyed benchmark).
- `c`: clear the fiducial.
- Once a fiducial is set, the TUI continuously shows the live fix's East/
  North/Up offset and 2D/3D error (cm) relative to it -- e.g. to occupy a
  known point, average a sample there, set it as fiducial, then move off and
  back to see how repeatably the base+rover solution returns to that exact
  spot, or compare against a surveyed truth coordinate directly.

Connect a rover/NTRIP client to `ntrip://<this-host-ip>:2101/HEREPRO`, no
credentials needed. On the LAN use the machine's local IP; over Tailscale use
its Tailscale IP.

## Files

- `ntrip_caster.py` -- the caster + live TUI. Auto-discovers the HerePro's
  serial device (via `/dev/serial/by-id`, no more chasing `/dev/ttyACMx`
  numbers), auto-reconnects if the device drops and re-enumerates, serves
  RTCM3 to any number of NTRIP clients (NTRIP v1-style `ICY 200 OK`
  handshake, works with Mission Planner, QGC, `str2str`, u-center, etc.), and
  shows live status via curses (or headless periodic log lines with
  `--no-ui` / when there's no real tty).
- `start_base.sh` -- one-command start: asks for a sampling time and
  (optionally) a fiducial, starts the caster or reuses a running one (incl. the
  service), averages the position, checks survey-in and that RTCM 1005 (base
  position) is being sent, then prints the exact NTRIP settings for Mission
  Planner / QGC / MAVProxy / str2str for each network interface. `-h` for flags.
- `systemd/` -- `herepro-ntrip.service` user unit, `base.env.example`
  settings, and deploy steps (`systemd/README.md`).
- `ubx_parse.py` -- shared UBX NAV-PVT / NAV-SVIN decoding, RTCM3 frame
  counting, and `split_frames()` (checksum-checked incremental stream parser
  used by `ntrip_caster.py`), used by both `ntrip_caster.py` and `check_status.py`.
- `geo.py` -- WGS84 ECEF/ENU math (lat/lon/height -> East/North/Up offset
  between two points), used for the fiducial correction display and
  occupation-sample spread calculation.
- `check_status.py` -- one-shot status check: decodes live NAV-PVT / NAV-SVIN
  / RTCM3 presence from either the running caster or a serial port directly.
  `ntrip_caster.py`'s own TUI now covers this continuously, so this is mostly
  useful for quick one-off checks or scripting.
- `tools/check_can_master_heartbeat.py` -- confirms a Cube/autopilot is alive
  over MAVLink before trying anything CAN-related.
- `tools/check_ardupilot_can_params.py` -- reads `CAN_P*_DRIVER` /
  `CAN_D*_PROTOCOL` / `CAN_D*_UC_NODE` off the autopilot to confirm CAN is
  actually enabled.
- `tools/raw_can_frames.py` -- enables MAVLink CAN-forwarding
  (`MAV_CMD_CAN_FORWARD`) and dumps raw `CAN_FRAME` traffic; the reliable way
  to confirm CAN bus traffic exists when the `dronecan` library isn't
  cooperating.
- `tools/decode_can_ids.py` -- decodes DroneCAN v0 message CAN IDs (priority,
  data type ID, source node) from raw frames to identify which node IDs are
  present and what they're broadcasting (e.g. NodeStatus = dtid 341).
- `tools/discover_can_nodes_dronecan.py` -- the "proper" way to enumerate
  DroneCAN nodes via the `dronecan` library's `make_node()` + NodeStatus
  handler. Kept for reference; didn't work in this environment (see gotcha
  #4) even though the underlying bus was fine.
- `tools/configure_base_valset.py` -- attempts to configure survey-in /
  RTCM3 output via raw `UBX-CFG-VALSET` messages sent directly over the USB
  serial port. Kept for reference; unreliable here (see gotcha #2). Setting
  `B_ENABLE=1` via Mission Planner's DroneCAN parameter page turned out to be
  sufficient on its own.
- `tools/dronecan_param_client.py` -- hand-rolled DroneCAN GetSet client that
  encodes/decodes transfers correctly via the `dronecan` library but sends
  raw frames through MAVLink `CAN_FRAME` directly (bypassing `dronecan`'s
  broken transport, see gotcha #4). GET/SET CLI for any node's parameter by
  name. Read path untested-successful (never got a response to confirm);
  TX/write path did not work in this environment (see gotcha #6). Kept for
  reference / a future retry, not currently a working substitute for Mission
  Planner.

## Running it as a service (survives reboot)

`systemd/` has a user unit (`herepro-ntrip.service`) plus a settings file
(`base.env.example`) -- deploy steps in [`systemd/README.md`](systemd/README.md).
Once installed it starts at boot (no login needed), waits for the HerePro,
reconnects after replugs, and logs to `journalctl --user -u herepro-ntrip -f`.
`start_base.sh` detects the running service and reuses it.
