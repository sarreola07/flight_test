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

> Reinstalling the Jetson (e.g. moving to NVMe)? The camera environment and
> venvs live outside this repo — see [docs/REINSTALL.md](docs/REINSTALL.md) to
> rebuild everything from a fresh clone.

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

![check_pixhawk.py output](docs/check_pixhawk.png)

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

## Missions (missions.py)

Interactive bench-test program:

```bash
./venv/bin/python missions.py
```

It first asks whether the propellers are removed, then shows a menu that **loops
until you press `q`** — after each mission it returns to the menu so you can run
another. Press `p` to re-declare the props state (to switch between motor tests
and flight without restarting). It offers:

| Mission | What it does | Requires |
|---|---|---|
| 1 | Spins each motor one at a time, 3 s each at 15% throttle | Props **OFF** |
| 2 | Spins all motors together for 3 s at 15% throttle | Props **OFF** |
| 3 | Arms, takes off to 3 ft (0.91 m), hovers, lands, disarms | Props **ON**, GPS position fix, extra `FLY` confirmation |
| 4 | Manual RC flight: switches to Stabilized, arms, then monitors while you fly with the transmitter | Props **ON**, RC link up, extra `FLY` confirmation |
| 5 | Camera tracking display — shows the OAK-D's live person X/Y/Z. No flight. | AI Camera toggle started |
| 6 | LoRa remote — runs missions from LoRa commands (see table below) | LoRa module on `/dev/ttyUSB0` |

Safety interlocks: motor tests are locked out while props are on, and flight is
locked out while props are off. Ctrl-C sends a disarm in missions 1–3; in
mission 4 it only stops the monitor — the RC pilot keeps control and disarms
with the sticks (throttle low + yaw left).

### Camera + LoRa architecture

Two processes talk over a local UDP socket, which keeps the camera's DepthAI
environment separate from the flight code's venv:

```
OAK-D  --USB-->  camera_publisher.py (depthai-env)  --UDP 127.0.0.1:5005-->  missions.py (venv)
LoRa   --/dev/ttyUSB0 serial-->                                              missions.py (pyserial)
Pixhawk--/dev/ttyACM0 MAVLink-->                                             missions.py (pymavlink)
```

- **`camera_publisher.py`** runs the OAK-D person detector headlessly (in
  `~/oak_drone_project/depthai-env`) and broadcasts the nearest person's
  `{x, y, z, conf}` in metres. It is started/stopped **on demand** via
  `ai_camera.sh` (see below) — never at boot.
- **Option 5** subscribes to that UDP stream and prints live coordinates — pure
  telemetry, never touches the flight controller. (Autonomous *follow-flight* is
  intentionally not wired up on this PX4 vehicle yet; see Notes.)
- **Option 6** turns LoRa packets (`{"msg": "N"}`) into missions, under the **same
  props interlock** you declare at startup:

  | LoRa `msg` | Action | Allowed when |
  |---|---|---|
  | `1` | sequential motor test | props **OFF** |
  | `2` | all-motor test | props **OFF** |
  | `3` | camera display (30 s) | always |
  | `4` | flight: arm / takeoff / land | props **ON** |

  Because a LoRa sender can't type the `FLY` prompt, command `4` flies with **no
  local confirmation** — it is still gated by props-ON and the Pixhawk's own
  pre-arm checks (which currently need a GPS fix). LoRa is **not** auto-started on
  boot; it only runs after you pick option 6.

The mission menu with props off, and the interlock refusing a flight mission
without props:

![missions menu](docs/missions_menu.png)
![safety interlock](docs/safety_lockout.png)

Mission 4 verifying the RC link before flight (aborted at the FLY prompt):

![mission 4 RC check](docs/mission4_rc_check.png)

Notes for this vehicle (PX4 v1.13.3, FMUv2):

- Motor tests use `MAV_CMD_DO_MOTOR_TEST`; the safety switch must be pressed
  (solid LED) and a battery connected, or the FC rejects the command.
- Mission 3 uses PX4's AUTO.TAKEOFF/AUTO.LAND modes. PX4's own preflight and
  arm-time checks must pass before it will arm; when it refuses, the script
  prints the FC's exact reason ("FC says: ...").
- This vehicle is configured as airframe `SYS_AUTOSTART=6001` (DJI F550
  hexarotor), safety switch bypassed (`CBRK_IO_SAFETY=22027`), arming without
  GPS allowed (`COM_ARM_WO_GPS=1`).
- FMUv2 quirk: PX4 v1.13 on this board doesn't run `load_mon`, so the
  "No CPU load information" preflight check fails out of the box. We set
  `COM_CPU_MAX=-1` to disable that check.

## Install: desktop shortcuts

```bash
bash install.sh
```

No sudo, no systemd — this only installs two user-level Desktop shortcuts:

1. **Hexacopter Mission** — opens a terminal running the mission menu
   (`run_missions.sh` → `missions.py`).
2. **AI Camera (toggle)** — starts/stops the OAK-D tracker on demand.

## AI camera toggle (decoupled from the drone link)

The AI camera tracking is **optional and manual**, deliberately separate from the
core MAVLink/telemetry background services. Toggling it starts or stops
`camera_publisher.py` as a plain user process that only reads the USB camera and
publishes on UDP 5005 — it **never opens `/dev/ttyACM0`**, so it cannot restart
the drone or interrupt background communications.

Double-click the **AI Camera (toggle)** Desktop icon to flip it on/off (you get a
desktop notification either way), or from a terminal:

```bash
./ai_camera.sh            # toggle: start if stopped, stop if running
./ai_camera.sh start      # explicit start (headless)
./ai_camera.sh stop       # explicit stop
./ai_camera.sh status     # RUNNING (PID) or stopped
./ai_camera.sh preview    # open a live window to visually check the camera
```

State and logs live in `~/.local/state/ai-camera/` (`camera.pid`, `camera.log`).

### Visual check — preview window

To *see* the camera feed with the detected person boxed and its X/Y/Z distance
drawn on it, double-click **AI Camera (preview)** or run `./ai_camera.sh preview`.
Press **q** in the window (or Ctrl-C) to close it. The preview still publishes
coordinates on UDP 5005, so mission option 5 works while it is open.

The OAK-D allows only one owner at a time, so `preview` first stops the headless
tracker if it is running; start it again with the toggle when you are done.

> The camera is **not** a boot service. If you want the core MAVLink/telemetry
> link to come up automatically at boot instead, that belongs in its own systemd
> unit; to share the Pixhawk serial port with `missions.py`, front it with a
> MAVLink router (e.g. `mavlink-routerd`) so one owner holds `/dev/ttyACM0` and
> everything else connects over UDP.

## Follow-me code from classmates (`hexacopter-follow/`)

The `hexacopter-follow/` folder is a separate project (a classmate's git repo) built
for the **ArduPilot SITL simulator**, not this PX4 vehicle — its GUIDED-mode flight
paths do not run on PX4 as-is. It is kept on disk but git-ignored by this repo. The
only piece reused here is the camera → UDP idea, reimplemented safely in
`camera_publisher.py`. Porting its autonomous person-following to PX4 (OFFBOARD mode
+ geofence/failsafe hardening) is future work.

## Next steps

- MAVProxy (`pip install MAVProxy` in the venv) for an interactive shell / to
  forward MAVLink over the network to QGC on another machine.
- ROS 2 + micro-XRCE-DDS or MAVROS for actual companion-computer control.
