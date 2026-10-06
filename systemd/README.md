# Deploying the NTRIP caster (systemd, user unit)

Not installed by anything in this repo — do it on the base-station computer.

```bash
git clone https://github.com/aadi121205/rtk-bas-harepro.git ~/rtk-bas-harepro   # unit expects that path
cd ~/rtk-bas-harepro
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
mkdir -p ~/.config/herepro ~/.config/systemd/user
cp systemd/base.env.example ~/.config/herepro/base.env       # edit it
cp systemd/herepro-ntrip.service ~/.config/systemd/user/
sudo usermod -aG uucp "$USER"       # serial port access: uucp on Arch, dialout on Ubuntu/Debian; log out + in
systemctl --user daemon-reload
systemctl --user enable --now herepro-ntrip.service
sudo loginctl enable-linger "$USER" # start at boot without a login session
```

* `herepro-ntrip.service` runs from boot. It does not need the HerePro plugged in: it logs
  "no HerePro device found" once and picks the device up when it appears, and again after every
  replug (the `/dev/ttyACMx` number changes, the `/dev/serial/by-id` name does not).
* It only ever *reads* the HerePro's USB port (writing to it stops the receiver's output until a
  power cycle, see the main README).
* Rovers connect to `ntrip://<this-host-ip>:2101/HEREPRO`, no credentials. `./start_base.sh` prints
  the exact settings for Mission Planner / QGC / MAVProxy / str2str and checks survey-in + RTCM 1005.
* Logs: `journalctl --user -u herepro-ntrip -f` — device found/lost, fix changes, survey-in VALID,
  each new RTCM message type (1005 = base position: rovers can't RTK without it), clients
  connecting/disconnecting, plus a status line every `STATUS_S` seconds.
* Change settings: edit `~/.config/herepro/base.env`, then `systemctl --user restart herepro-ntrip`
  (also after changing the fiducial).
* Stop / don't start at boot: `systemctl --user stop herepro-ntrip` / `systemctl --user disable herepro-ntrip`.
  Stop it before running `ntrip_caster.py` by hand (the TUI) — only one caster can hold port 2101.
* Update: `cd ~/rtk-bas-harepro && git pull && systemctl --user restart herepro-ntrip`.
