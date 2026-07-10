# Pixhawk ↔ Jetson Connection Test

Minimal, no-GUI test of the MAVLink link between the companion computer and the
flight controller. No QGroundControl or IDE required — just Python and a USB cable.

## Current hardware setup

| Component | Details |
|---|---|
| Companion computer | NVIDIA Jetson Orin Nano Developer Kit |
| OS | Ubuntu 24.04.4 LTS (L4T R39.2, kernel 6.8 tegra) |
| Flight controller | Pixhawk 2.4.8 (FMUv2, shows as `26ac:0011 3D Robotics PX4 FMU v2.x`) |
| Firmware | PX4 v1.13.3 |
| Connection | USB → `/dev/ttyACM0` (baud rate is ignored on USB CDC) |
| Ground station | None installed (this repo replaces QGC for basic checks) |

Other serial ports available on the Jetson if you later wire TELEM2 to the
40-pin header: `/dev/ttyTHS1` and `/dev/ttyTHS2` (UARTs, typically 57600 or
921600 baud — must match the `SER_TEL2_BAUD` PX4 parameter).

## First-time setup

```bash
bash setup.sh
```

This does three things (asks for your sudo password once):

1. Adds your user to the `dialout` group — required to open `/dev/ttyACM0`.
   Permanent after the next logout/login or reboot.
2. Grants immediate access to the port so you can test right away.
3. Creates a `venv/` virtual environment and installs `pymavlink`
   (Ubuntu 24.04 blocks system-wide `pip install`, hence the venv).

## Running the test

```bash
./venv/bin/python check_pixhawk.py
```

or, with the venv activated (`source venv/bin/activate`):

```bash
python check_pixhawk.py
```

Options:

```bash
python check_pixhawk.py --device /dev/ttyTHS1 --baud 921600   # via TELEM2 UART
python check_pixhawk.py --timeout 20                          # slower heartbeat wait
```

## What the script checks

1. **Serial port opens** — cable present, permissions OK.
2. **Heartbeat** — the autopilot is alive and speaking MAVLink; prints
   system/component ID, autopilot type, vehicle type, armed state.
3. **Firmware version** — requests `AUTOPILOT_VERSION` (should report 1.13.3).
4. **Telemetry** — battery voltage (`SYS_STATUS`), GPS fix and satellite count
   (`GPS_RAW_INT`, skipped gracefully if no GPS module), attitude from the IMU
   (`ATTITUDE`).
5. **Parameter read** — reads `SYS_AUTOSTART` to prove two-way communication.

Exit code is `0` when everything passes, `1` otherwise — safe to use in scripts.

Expected output on a healthy USB-powered bench setup:

```
[PASS] Heartbeat from system 1 component 1
[INFO]   Autopilot: MAV_AUTOPILOT_PX4   Vehicle type: MAV_TYPE_QUADROTOR
[PASS] Firmware version: 1.13.3
[PASS] Battery: 0.00 V  (no battery / USB power only)
...
All good: the Jetson can talk to the Pixhawk. ✅
```

## Troubleshooting

- **`Permission denied` on `/dev/ttyACM0`** — log out/in (or reboot) so the
  `dialout` group takes effect, or re-run `setup.sh`.
- **Device missing** — check `ls /dev/ttyACM*` and `lsusb | grep 26ac`.
  Try a different USB cable (data lines required, not charge-only).
- **No heartbeat** — wait ~30 s after plugging in for PX4 to boot; make sure
  nothing else (e.g. MAVProxy) already has the port open.

## Next steps

- MAVProxy (`pip install MAVProxy` in the venv) for an interactive shell / to
  forward MAVLink over the network to QGC on another machine.
- ROS 2 + micro-XRCE-DDS or MAVROS for actual companion-computer control.
