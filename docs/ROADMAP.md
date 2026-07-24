# Venator roadmap — laptop-commanded follow-me drone

End goal: command the drone from any laptop over LoRa, with **zero interaction on
the Jetson**, including a "fly to these coordinates" mission.

## Status

| Phase | What | State |
|---|---|---|
| 0 | NVMe install + restore project (see [REINSTALL.md](REINSTALL.md)) | ✅ done |
| 1 | C2 protocol ([PROTOCOL.md](PROTOCOL.md), `c2_protocol.py`) | ✅ done |
| 2 | Portable laptop client + mock + CI to build `.exe`/`.app` | 🔵 in progress |
| 3 | Bidirectional Heltec firmware (half-duplex transceiver) | ✅ written — you flash + two-terminal test |
| 4 | Jetson C2 server + systemd boot service (zero-touch) | ✅ working over real LoRa (motor test); boot service + rich menu done |
| 5 | GPS + waypoint flight | 🔵 upload + flight trigger + two-step arm + link-loss RTL built & bench-tested; real outdoor flight pending a fix |
| — | Mission 1: hover + detect people + land | 🔵 built & bench-tested (mock); hover 2 m on GPS, person detection over LoRa; real flight outdoors, true 3 ft with lidar |
| 6 | Polish: auto-launch agents, saved "places", browser GUI, OFFBOARD follow | ⬜ |

## Phase details

- **1 — Protocol** *(done):* versioned, newline-JSON messages; handshake, menu,
  run, waypoint upload, ACK/retransmit. Shared by client and Jetson server.
- **2 — Client** *(in progress):* `gcs_client.py` auto-detects the Heltec
  (CP210x VID), handshakes, renders the Jetson's menu, uploads waypoints. Runs
  today with `--mock` (no hardware). The GitHub Actions workflow builds
  `VenatorGCS.exe` and `.app` — download from the run's Artifacts.
- **3 — Firmware:** merge the two working sketches into one half-duplex
  transceiver (transparent line bridge, same 915 MHz/SF7/syncword). Flash both
  sticks.
- **4 — Jetson server** *(server done):* `jetson_c2_server.py` owns the LoRa
  serial and the Pixhawk; serves the menu, runs missions (reusing `missions.py`),
  streams ACK/DONE. Defaults to a **mock FC** and **props OFF** (safe); `--real`
  connects the Pixhawk, `--props-on` enables flight. Tested end-to-end against the
  real client over a loopback — props + GPS gates verified. `systemd/venator-c2.service`
  is the zero-touch boot unit (enable deliberately). Still TODO: two-step arm
  confirm, link-loss → RTL, and threaded execution so ABORT/heartbeat work mid-mission.
- **5 — Waypoint flight:** fit a GPS (M8N → Pixhawk GPS port; `EKF2_AID_MASK=1`
  already set). Upload via `MISSION_ITEM_INT` → `AUTO.MISSION`, altitude cap +
  RTL-last, bench-validate acceptance, then fly outdoors.
- **6 — Polish:** zero-click launch agents on your own Win/Mac, a saved places
  library, a browser GUI, and the OFFBOARD-mode camera-follow port.

## Open decisions (needed by Phase 4/5)

1. Link-loss during flight: RTL or land-in-place? (default: RTL)
2. Geofence radius + altitude cap.
3. Coordinates: free-typed, saved "places", or both?
4. Client UI: CLI now, browser GUI later.
