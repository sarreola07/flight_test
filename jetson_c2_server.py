#!/usr/bin/env python3
"""
Venator Jetson C2 server — the drone side of the LoRa command link.

Reads C2 protocol messages from the LoRa stick (/dev/ttyUSB0), serves the
mission menu, runs missions on the Pixhawk, and ACKs back to the laptop client.
The laptop never touches the drone directly — it only speaks the protocol.

    # safe: mock flight controller, no hardware touched (default)
    python3 jetson_c2_server.py

    # real: connect the Pixhawk and actually run missions
    ./venv/bin/python jetson_c2_server.py --real --props-on

Safety: defaults to a MOCK flight controller and props-OFF. Flight missions are
rejected unless props are declared ON (--props-on) and the FC's own checks pass
(e.g. a GPS fix). Motor tests require props OFF. Nothing arms on startup.
"""
import argparse
import glob
import json
import socket
import sys
import time

import c2_protocol as p

MAX_ALT_M = 50.0        # reject uploaded waypoints above this relative altitude
LINK_LOSS_S = 4.0       # in flight, no client ping for this long -> RTL failsafe
FLIGHT_MAX_S = 600.0    # hard cap on a monitored flight -> RTL
STATUS_INTERVAL_S = 1.0 # how often to push in-flight status to the client

# --- Mission 1: hover & detect ---
# GPS-safe hover altitude. A true 3 ft (0.91 m) hold is unsafe on GPS alone
# (several-metre vertical error); set this to 0.91 once the downward lidar is in.
HOVER_ALT_M = 2.0
HOVER_TIME_S = 60.0     # how long to hover and detect people
CAMERA_UDP_PORT = 5005  # OAK-D publisher (ai_camera.sh) broadcasts person X/Y/Z here

# --- Mission 2: follow-me (OFFBOARD velocity control) ---
FOLLOW_DIST_M = 3.0     # keep this distance from the person
FOLLOW_DIST_DEAD = 0.5  # deadband on distance (m)
FOLLOW_X_DEAD = 0.3     # deadband on horizontal offset (m) before yawing
FOLLOW_FWD_MPS = 0.5    # forward/back speed
FOLLOW_YAW_RPS = 0.35   # yaw rate (rad/s, ~20 deg/s)
CAMERA_LOSS_S = 1.0     # no person for this long -> hover in place (don't drift)


def find_fc_device(default="/dev/ttyACM0"):
    """Locate the Pixhawk robustly, so a USB replug (ttyACM0->ttyACM1) can't
    break the link. Prefer the stable by-id symlink (survives renumbering and
    the v2->v3 reflash, since the name still contains PX4/FMU), then any ttyACM."""
    for pat in ("/dev/serial/by-id/*PX4*", "/dev/serial/by-id/*FMU*",
                "/dev/serial/by-id/*Pixhawk*"):
        matches = sorted(glob.glob(pat))
        if matches:
            return matches[0]
    acms = sorted(glob.glob("/dev/ttyACM*"))
    return acms[0] if acms else default


def find_lora_device(default="/dev/ttyUSB0"):
    """Locate the Heltec LoRa stick robustly (it's a CP2102 USB-UART), so a
    replug (ttyUSB0->ttyUSB1) can't kill the service."""
    for pat in ("/dev/serial/by-id/*CP2102*", "/dev/serial/by-id/*Silicon_Labs*"):
        matches = sorted(glob.glob(pat))
        if matches:
            return matches[0]
    usbs = sorted(glob.glob("/dev/ttyUSB*"))
    return usbs[0] if usbs else default

# Reuse the tested mission logic and MAVLink helpers from missions.py
try:
    import missions
    from pymavlink import mavutil
except ImportError:
    missions = None
    mavutil = None


# --------------------------------------------------------------------------
# Flight-controller abstraction — mock (safe) or real (pymavlink via missions).
# --------------------------------------------------------------------------
class MockFC:
    """A fake flight controller so the server runs and is testable with no drone.

    Includes a tiny flight simulator (arm -> climb -> hover -> land/RTL -> disarm)
    so the flight trigger and link-loss failsafe can be exercised without a drone.
    Pass gps=True to simulate a 3D fix (needed to test the flight path).
    """
    name = "mock"

    def __init__(self, gps=False):
        self._gps = gps
        self._armed = False
        self._mode = "HOLD"
        self._t_arm = 0.0
        self._t_land = None
        self._has_mission = False

    def status(self):
        return {"armed": self._armed, "gps": "3D fix" if self._gps else "no fix", "batt": 11.4}

    def has_gps_fix(self):
        return self._gps

    def run_mission(self, mid, motors=6):
        time.sleep(0.1)
        return True, "ok (mock)"

    def upload_waypoints(self, wps):
        self._has_mission = len(wps) > 0
        return len(wps), 0, "stored (mock)"

    def has_mission(self):
        return self._has_mission

    # -- flight simulation --
    def arm(self, timeout=8):
        self._armed = True
        self._t_arm = time.time()
        self._t_land = None
        return True

    def disarm(self):
        self._armed = False
        return True

    def set_flight_mode(self, mode):
        self._mode = mode
        if mode in ("LAND", "RTL") and self._t_land is None:
            self._t_land = time.time()
        return True

    def send_velocity(self, vx, vy, vz, yaw_rate):
        self._last_vel = (vx, vy, vz, yaw_rate)   # recorded for tests

    def flight_state(self):
        now = time.time()
        # AUTO.MISSION completes on its own ~4 s after takeoff (mock). TAKEOFF/HOLD
        # (Mission 1) keeps hovering until the server explicitly commands LAND/RTL.
        if (self._armed and self._t_land is None and self._mode == "MISSION"
                and now - self._t_arm > 4.0):
            self._t_land = now
        # simulate touchdown + auto-disarm ~1.5 s after landing starts
        if self._armed and self._t_land and now - self._t_land > 1.5:
            self._armed = False
        if not self._armed:
            return {"armed": False, "alt": 0.0, "mode": self._mode}
        if self._t_land:
            alt = max(0.0, 1.5 - (now - self._t_land)) * 2.0
        else:
            alt = min(3.0, (now - self._t_arm) * 1.5)  # climb to 3 m
        return {"armed": True, "alt": round(alt, 2), "mode": self._mode}

    def close(self):
        pass


class RealFC:
    """Real flight controller: connects the Pixhawk and runs missions.py logic."""
    name = "real"

    def __init__(self, device, baud):
        self.master = missions.connect(device, baud)

    def status(self):
        armed = False
        gps = "no fix"
        batt = 0.0
        hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        g = self.master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if g:
            fixes = {0: "no gps", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK"}
            gps = fixes.get(g.fix_type, str(g.fix_type))
        s = self.master.recv_match(type="SYS_STATUS", blocking=True, timeout=2)
        if s:
            batt = round(s.voltage_battery / 1000.0, 2)
        return {"armed": armed, "gps": gps, "batt": batt}

    def has_gps_fix(self):
        g = self.master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        return bool(g and g.fix_type >= 3)

    def has_mission(self):
        self.master.mav.mission_request_list_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        c = self.master.recv_match(type="MISSION_COUNT", blocking=True, timeout=3)
        return bool(c and c.count > 0)

    # -- flight --
    def arm(self, timeout=8):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        end = time.time() + timeout
        while time.time() < end:
            hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return True
        return False

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        return True

    def set_flight_mode(self, mode):
        # mode is a px4_map key: "MISSION", "RTL", "LAND", "TAKEOFF", "OFFBOARD"
        self.master.set_mode_px4(*mavutil.px4_map[mode])
        return True

    def send_velocity(self, vx, vy, vz, yaw_rate):
        """Body-frame velocity + yaw-rate setpoint (for OFFBOARD following).
        Type mask uses vx,vy,vz and yaw_rate; ignores position, accel, and yaw."""
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000011111000111,
            0, 0, 0,            # position (ignored)
            vx, vy, vz,         # velocity (forward, right, down)
            0, 0, 0,            # acceleration (ignored)
            0, yaw_rate)        # yaw (ignored), yaw_rate

    def flight_state(self):
        hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        armed = bool(hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
        mode = mavutil.mode_string_v10(hb) if hb else "?"
        alt = 0.0
        g = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if g:
            alt = round(g.relative_alt / 1000.0, 2)
        return {"armed": armed, "alt": alt, "mode": mode}

    def run_mission(self, mid, motors=6):
        if mid == 1:
            missions.mission_1(self.master, motors)
        elif mid == 2:
            missions.mission_2(self.master, motors)
        elif mid == 3:
            missions.mission_3(self.master, require_confirm=False)
        else:
            return False, f"mission {mid} not runnable remotely"
        return True, "done"

    def upload_waypoints(self, wps):
        """Validate waypoints, build a takeoff→waypoints→RTL mission, upload to PX4.

        Returns (accepted_count, rejected_count, note). Uploading works without a
        GPS fix (PX4 just stores the plan); it only flies once armed with a fix.
        """
        valid, rejected = [], 0
        for wp in wps:
            try:
                lat, lon, alt = float(wp[0]), float(wp[1]), float(wp[2])
            except (TypeError, ValueError, IndexError):
                rejected += 1
                continue
            if not (-90 <= lat <= 90) or not (-180 <= lon <= 180) or (lat == 0 and lon == 0):
                rejected += 1
                continue
            if not (0 < alt <= MAX_ALT_M):
                rejected += 1
                continue
            valid.append((lat, lon, alt))

        if not valid:
            return 0, rejected, "no valid waypoints (check lat/lon and 0<alt<=%g m)" % MAX_ALT_M

        items = self._build_mission(valid)
        result = self._upload_mission(items)
        if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            return len(valid), rejected, \
                "PX4 accepted mission ({} items: takeoff + {} wp + RTL)".format(len(items), len(valid))
        name = (mavutil.mavlink.enums["MAV_MISSION_RESULT"][result].name
                if result is not None else "no MISSION_ACK (timeout)")
        return 0, len(wps), "PX4 rejected mission: {}".format(name)

    def _build_mission(self, valid):
        """takeoff (to first wp alt) -> NAV_WAYPOINTs -> RTL.

        Positional items use the global relative-alt frame; RTL has no position,
        so it must use MAV_FRAME_MISSION or PX4 rejects the whole mission as
        UNSUPPORTED (verified on this FMUv2 / PX4 1.13.3).
        """
        REL = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        MISSION = mavutil.mavlink.MAV_FRAME_MISSION
        items = []

        def item(seq, cmd, frame=REL, x=0, y=0, z=0.0, p1=0, p2=0, p3=0, p4=0):
            return dict(seq=seq, frame=frame, command=cmd, current=1 if seq == 0 else 0,
                        autocontinue=1, p1=p1, p2=p2, p3=p3, p4=p4, x=x, y=y, z=z)

        seq = 0
        items.append(item(seq, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, z=valid[0][2]))
        for lat, lon, alt in valid:
            seq += 1
            items.append(item(seq, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                              x=int(lat * 1e7), y=int(lon * 1e7), z=alt, p2=2.0))  # 2 m accept radius
        seq += 1
        items.append(item(seq, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, frame=MISSION))
        return items

    def _upload_mission(self, items, timeout=15):
        """Run the MAVLink mission upload handshake. Returns the MISSION_ACK type."""
        m = self.master
        m.mav.mission_count_send(m.target_system, m.target_component,
                                 len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        start = time.time()
        while time.time() - start < timeout:
            msg = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                               blocking=True, timeout=2)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ACK":
                return msg.type
            seq = msg.seq
            if seq >= len(items):
                continue
            it = items[seq]
            m.mav.mission_item_int_send(
                m.target_system, m.target_component,
                it["seq"], it["frame"], it["command"], it["current"], it["autocontinue"],
                it["p1"], it["p2"], it["p3"], it["p4"], it["x"], it["y"], it["z"],
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        return None

    def clear_mission(self):
        """Erase the uploaded mission from the FC. Returns True if acknowledged."""
        m = self.master
        m.mav.mission_clear_all_send(m.target_system, m.target_component,
                                     mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        ack = m.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
        return bool(ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED)

    def close(self):
        try:
            self.master.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# The C2 server: protocol handling + mission dispatch.
# --------------------------------------------------------------------------
class C2Server:
    def __init__(self, fc, props_off=True, motors=6, log=print):
        self.fc = fc
        self.props_off = props_off
        self.motors = motors
        self.log = log
        self._wp = []
        # A validated flight waiting for CONFIRM: None / "waypoints" / "mission1" / "mission2"
        self._pending = None

    def menu(self):
        return [
            {"id": 1, "name": "Sequential motor test", "needs": "props_off"},
            {"id": 2, "name": "All-motor test", "needs": "props_off"},
            {"id": 3, "name": "Fly to waypoints", "needs": "props_on", "gps": True},
            {"id": 4, "name": "Mission 1: hover + detect + land", "needs": "props_on",
             "gps": True, "fly": "mission1"},
            {"id": 5, "name": "Mission 2: follow person", "needs": "props_on",
             "gps": True, "fly": "mission2"},
        ]

    def _validate(self, mid):
        """Return (accepted, reason) for a RUN request under the safety gates."""
        item = next((i for i in self.menu() if i["id"] == mid), None)
        if item is None:
            return False, "unknown mission"
        if item.get("needs") == "props_off" and not self.props_off:
            return False, "props are ON — motor tests need props OFF"
        if item.get("needs") == "props_on" and self.props_off:
            return False, "props are OFF — flight needs props ON"
        if item.get("gps") and not self.fc.has_gps_fix():
            return False, "no GPS fix"
        return True, "ok"

    # -- message handling -----------------------------------------------------
    def handle(self, msg):
        """Return a list of immediate reply messages for a non-RUN message."""
        t = msg.get("t")
        seq = msg.get("seq", 0)
        if t == p.HELLO:
            st = self.fc.status()
            return [p.message(p.HELLO_ACK, seq, proto=p.PROTO_VERSION,
                              fc=self.fc.name, props_off=self.props_off, **st)]
        if t == p.GET_MENU:
            # Stream the menu one item per packet — a full menu in a single packet
            # exceeds the LoRa limit (~255 B) and the firmware's 240-byte buffer.
            # Each item (with its props/GPS flags) fits comfortably.
            items = self.menu()
            return [p.message(p.MENU, seq, item=it, last=(i == len(items) - 1))
                    for i, it in enumerate(items)]
        if t == p.PING:
            return [p.message(p.PONG, seq)]
        if t == p.WP_BEGIN:
            self._wp = []
            return [p.message(p.ACK, seq, ready=True)]
        if t == p.WP:
            self._wp.append((msg.get("lat"), msg.get("lon"), msg.get("alt")))
            return [p.message(p.ACK, seq, i=msg.get("i"))]
        if t == p.WP_END:
            up, rej, note = self.fc.upload_waypoints(self._wp)
            return [p.message(p.ACK, seq, uploaded=up, rejected=rej, note=note)]
        if t == p.FLY_REQ:
            ok, reason = self._validate_fly()
            self._pending = "waypoints" if ok else None
            return [p.message(p.ACK, seq, accepted=ok, confirm_required=ok, reason=reason)]
        return []

    def _validate_fly(self):
        """Safety gate for arming and flying the uploaded mission."""
        if self.props_off:
            return False, "props are OFF — flight needs props ON"
        if not self.fc.has_gps_fix():
            return False, "no GPS 3D fix"
        if not self.fc.has_mission():
            return False, "no mission uploaded — send waypoints first"
        return True, "ready — arm + fly takeoff->waypoints->RTL"

    def run_and_report(self, msg, send):
        """Handle a RUN. Motor tests execute immediately; flight missions are
        *prepared* and wait for a CONFIRM (two-step arm)."""
        seq = msg.get("seq", 0)
        mid = msg.get("id")
        accepted, reason = self._validate(mid)
        if not accepted:
            send(p.message(p.ACK, seq, id=mid, accepted=False, reason=reason))
            self.log(f"RUN {mid} rejected: {reason}")
            return
        item = next((i for i in self.menu() if i["id"] == mid), {})
        fly = item.get("fly")
        if fly in ("mission1", "mission2"):
            self._pending = fly
            if fly == "mission1":
                reason = ("Mission 1: takeoff to {:g} m, hover {:d} s + detect people, "
                          "land in place. Confirm to launch.".format(HOVER_ALT_M, int(HOVER_TIME_S)))
            else:
                reason = ("Mission 2: takeoff to {:g} m, then FOLLOW the person "
                          "(keep {:g} m). Send STOP to land. Confirm to launch.".format(
                              HOVER_ALT_M, FOLLOW_DIST_M))
            send(p.message(p.ACK, seq, id=mid, accepted=True, confirm_required=True, reason=reason))
            self.log(f"{fly} prepared — awaiting CONFIRM")
            return
        send(p.message(p.ACK, seq, id=mid, accepted=True, reason=reason))
        self.log(f"RUN {mid} accepted — executing")
        try:
            ok, result = self.fc.run_mission(mid, self.motors)
        except Exception as exc:
            ok, result = False, f"error: {exc}"
        send(p.message(p.DONE, seq, id=mid, result=result if ok else f"FAILED: {result}"))

    # -- camera (person detection during Mission 1) --------------------------
    def _open_camera(self):
        """UDP socket to receive the OAK-D publisher's person X/Y/Z, or None."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setblocking(False)
            s.bind(("127.0.0.1", CAMERA_UDP_PORT))
            return s
        except OSError:
            return None

    @staticmethod
    def _read_person(sock):
        """Drain the camera socket, return the latest person dict or None."""
        if sock is None:
            return None
        latest = None
        for _ in range(20):
            try:
                data, _addr = sock.recvfrom(1024)
            except (BlockingIOError, OSError):
                break
            try:
                latest = json.loads(data.decode())
            except (ValueError, UnicodeDecodeError):
                pass
        return latest

    def run_mission1(self, link, send):
        """Mission 1: arm -> takeoff -> hover HOVER_TIME_S while reporting detected
        people -> land in place. Same link-loss RTL / abort failsafes as run_flight."""
        if self._pending != "mission1":
            send(p.message(p.ACK, 0, accepted=False, reason="no confirmed flight pending"))
            return
        self._pending = None
        cam = self._open_camera()

        send(p.message(p.LOG, 0, text="Mission 1: arming"))
        self.log("MISSION1: arming")
        try:
            if not self.fc.arm():
                send(p.message(p.DONE, 0, id="mission1", result="FAILED: arm rejected"))
                return
            send(p.message(p.LOG, 0, text="armed — takeoff to {:g} m".format(HOVER_ALT_M)))
            self.fc.set_flight_mode("TAKEOFF")
        except Exception as exc:
            send(p.message(p.DONE, 0, id="mission1", result=f"FAILED: {exc}"))
            return

        phase = "takeoff"
        hover_start = None
        people_seen = 0
        person_present = False
        last_ping = time.time()
        last_status = 0.0
        rtl = False
        deadline = time.time() + FLIGHT_MAX_S
        while time.time() < deadline:
            now = time.time()
            msg = link.poll()
            if msg:
                mt = msg.get("t")
                if mt == p.PING:
                    last_ping = now
                    send(p.message(p.PONG, msg.get("seq", 0)))
                elif mt == p.ABORT and not rtl:
                    send(p.message(p.LOG, 0, text="ABORT — returning home"))
                    self.fc.set_flight_mode("RTL")
                    rtl = True
            if not rtl and now - last_ping > LINK_LOSS_S:
                self.log("MISSION1: LINK LOST -> RTL")
                send(p.message(p.LOG, 0, text="LINK LOST — returning home"))
                self.fc.set_flight_mode("RTL")
                rtl = True

            st = self.fc.flight_state()
            person = self._read_person(cam)
            if person is not None:
                if not person_present:
                    people_seen += 1
                person_present = True
            else:
                person_present = False

            if not rtl:
                if phase == "takeoff" and st["alt"] >= HOVER_ALT_M * 0.9:
                    phase = "hover"
                    hover_start = now
                    send(p.message(p.LOG, 0, text="hovering — detecting people"))
                elif phase == "hover" and hover_start and now - hover_start > HOVER_TIME_S:
                    phase = "land"
                    self.fc.set_flight_mode("LAND")
                    send(p.message(p.LOG, 0, text="hover complete — landing in place"))

            if now - last_status >= STATUS_INTERVAL_S:
                send(p.message(p.STATUS, 0, rtl=rtl, phase="RTL" if rtl else phase,
                               person=person_present, seen=people_seen,
                               pz=round(person["z"], 2) if person else None, **st))
                last_status = now

            if not st["armed"]:
                if cam:
                    cam.close()
                send(p.message(p.DONE, 0, id="mission1",
                               result="landed & disarmed — {} person-detection(s){}".format(
                                   people_seen, " (RTL)" if rtl else "")))
                self.log("MISSION1: landed & disarmed")
                return
            time.sleep(0.15)

        if cam:
            cam.close()
        self.fc.set_flight_mode("RTL")
        send(p.message(p.DONE, 0, id="mission1", result="time cap -> RTL"))

    @staticmethod
    def _follow_setpoint(person):
        """person X (right+)/Z (forward dist) -> (vx forward, yaw_rate). vz=0 (hold alt)."""
        z = person.get("z", FOLLOW_DIST_M)
        x = person.get("x", 0.0)
        vx = 0.0
        if z > FOLLOW_DIST_M + FOLLOW_DIST_DEAD:
            vx = FOLLOW_FWD_MPS
        elif z < FOLLOW_DIST_M - FOLLOW_DIST_DEAD:
            vx = -FOLLOW_FWD_MPS
        yaw = 0.0
        if x > FOLLOW_X_DEAD:
            yaw = FOLLOW_YAW_RPS
        elif x < -FOLLOW_X_DEAD:
            yaw = -FOLLOW_YAW_RPS
        return vx, yaw

    def run_mission2(self, link, send):
        """Mission 2: arm -> takeoff -> OFFBOARD follow the person, keeping
        FOLLOW_DIST_M, until the client sends STOP (land) / ABORT (RTL) / link loss
        (RTL). No person in view -> hover in place (never drift)."""
        if self._pending != "mission2":
            send(p.message(p.ACK, 0, accepted=False, reason="no confirmed flight pending"))
            return
        self._pending = None
        cam = self._open_camera()

        send(p.message(p.LOG, 0, text="Mission 2: arming"))
        self.log("MISSION2: arming")
        try:
            if not self.fc.arm():
                send(p.message(p.DONE, 0, id="mission2", result="FAILED: arm rejected"))
                return
            send(p.message(p.LOG, 0, text="armed — takeoff to {:g} m".format(HOVER_ALT_M)))
            self.fc.set_flight_mode("TAKEOFF")
        except Exception as exc:
            send(p.message(p.DONE, 0, id="mission2", result=f"FAILED: {exc}"))
            return

        last_ping = time.time()
        last_status = 0.0
        rtl = False
        ending = None            # None -> flying; "STOP"/"ABORT"/"LINKLOSS"
        offboard = False
        last_person_t = 0.0
        deadline = time.time() + FLIGHT_MAX_S
        while time.time() < deadline:
            now = time.time()
            msg = link.poll()
            if msg:
                mt = msg.get("t")
                if mt == p.PING:
                    last_ping = now
                    send(p.message(p.PONG, msg.get("seq", 0)))
                elif mt == p.STOP and ending is None:
                    ending = "STOP"
                    send(p.message(p.LOG, 0, text="STOP — landing in place"))
                    self.fc.set_flight_mode("LAND")
                elif mt == p.ABORT and ending is None:
                    ending, rtl = "ABORT", True
                    send(p.message(p.LOG, 0, text="ABORT — returning home"))
                    self.fc.set_flight_mode("RTL")
            if ending is None and now - last_ping > LINK_LOSS_S:
                ending, rtl = "LINKLOSS", True
                self.log("MISSION2: LINK LOST -> RTL")
                send(p.message(p.LOG, 0, text="LINK LOST — returning home"))
                self.fc.set_flight_mode("RTL")

            st = self.fc.flight_state()
            person = self._read_person(cam)
            following = False
            if ending is None:
                # once at altitude, enter OFFBOARD and stream velocity setpoints
                if not offboard and st["alt"] >= HOVER_ALT_M * 0.9:
                    for _ in range(10):
                        self.fc.send_velocity(0, 0, 0, 0)
                        time.sleep(0.02)
                    self.fc.set_flight_mode("OFFBOARD")
                    offboard = True
                    send(p.message(p.LOG, 0, text="following — send STOP to land"))
                if offboard:
                    if person is not None:
                        last_person_t = now
                        vx, yaw = self._follow_setpoint(person)
                        following = True
                    else:
                        vx, yaw = 0.0, 0.0        # no person -> hover, don't drift
                    self.fc.send_velocity(vx, 0, 0, yaw)

            if now - last_status >= STATUS_INTERVAL_S:
                send(p.message(p.STATUS, 0, rtl=rtl,
                               phase=("RTL" if rtl else (ending or ("follow" if offboard else "takeoff"))),
                               person=person is not None, following=following,
                               pz=round(person["z"], 2) if person else None, **st))
                last_status = now

            if not st["armed"]:
                if cam:
                    cam.close()
                how = {"STOP": "landed (stopped)", "ABORT": "landed (RTL)",
                       "LINKLOSS": "landed (link-loss RTL)"}.get(ending, "landed & disarmed")
                send(p.message(p.DONE, 0, id="mission2", result=how))
                self.log("MISSION2: landed & disarmed")
                return
            time.sleep(0.1)

        if cam:
            cam.close()
        self.fc.set_flight_mode("RTL")
        send(p.message(p.DONE, 0, id="mission2", result="time cap -> RTL"))

    def run_flight(self, link, send):
        """CONFIRM received: arm + AUTO.MISSION, then monitor with a link-loss
        RTL failsafe until the drone lands and disarms. `link` is polled for the
        client's in-flight pings and ABORT; `send` pushes status/log back."""
        if self._pending != "waypoints":
            send(p.message(p.ACK, 0, accepted=False, reason="no confirmed flight pending"))
            return
        self._pending = None

        send(p.message(p.LOG, 0, text="arming..."))
        self.log("FLIGHT: arming")
        try:
            if not self.fc.arm():
                send(p.message(p.DONE, 0, id="fly", result="FAILED: arm rejected (pre-arm checks)"))
                return
            send(p.message(p.LOG, 0, text="armed — AUTO.MISSION, taking off"))
            self.log("FLIGHT: armed, AUTO.MISSION")
            self.fc.set_flight_mode("MISSION")
        except Exception as exc:
            send(p.message(p.DONE, 0, id="fly", result=f"FAILED: {exc}"))
            return

        last_ping = time.time()
        last_status = 0.0
        rtl = False
        deadline = time.time() + FLIGHT_MAX_S
        while time.time() < deadline:
            now = time.time()
            msg = link.poll()
            if msg:
                mt = msg.get("t")
                if mt == p.PING:
                    last_ping = now
                    send(p.message(p.PONG, msg.get("seq", 0)))
                elif mt == p.ABORT and not rtl:
                    self.log("FLIGHT: ABORT -> RTL")
                    send(p.message(p.LOG, 0, text="ABORT — returning home"))
                    self.fc.set_flight_mode("RTL")
                    rtl = True
            if not rtl and now - last_ping > LINK_LOSS_S:
                self.log("FLIGHT: LINK LOST -> RTL")
                send(p.message(p.LOG, 0, text="LINK LOST — returning home"))
                self.fc.set_flight_mode("RTL")
                rtl = True

            st = self.fc.flight_state()
            if now - last_status >= STATUS_INTERVAL_S:
                send(p.message(p.STATUS, 0, rtl=rtl, **st))
                last_status = now
            if not st["armed"]:
                send(p.message(p.DONE, 0, id="fly",
                               result="landed & disarmed" + (" (RTL)" if rtl else "")))
                self.log("FLIGHT: landed & disarmed")
                return
            time.sleep(0.15)

        self.log("FLIGHT: time cap reached -> RTL")
        self.fc.set_flight_mode("RTL")
        send(p.message(p.DONE, 0, id="fly", result="flight time cap -> RTL"))


# --------------------------------------------------------------------------
# LoRa serial transport (transparent line bridge on the other end).
# --------------------------------------------------------------------------
class LoRaLink:
    def __init__(self, port, baud):
        import serial
        # exclusive=True so nothing else can grab the LoRa port and corrupt the link
        self.ser = serial.Serial(port, baud, timeout=0.2, exclusive=True)
        self._buf = ""

    def send(self, msg):
        self.ser.write(p.encode(msg).encode("utf-8"))

    def poll(self):
        try:
            data = self.ser.read(256).decode("utf-8", errors="replace")
        except Exception:
            return None
        if data:
            self._buf += data
        if "\n" not in self._buf:
            return None
        line, self._buf = self._buf.split("\n", 1)
        return p.decode(line)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def serve(server, link, log=print):
    """Main loop: pump protocol messages between the LoRa link and the server."""
    log(f"C2 server up ({server.fc.name} FC, props {'OFF' if server.props_off else 'ON'}). "
        f"Waiting for the laptop...")
    while True:
        msg = link.poll()
        if msg is None:
            time.sleep(0.02)
            continue
        t = msg.get("t")
        if t == p.RUN:
            server.run_and_report(msg, link.send)
        elif t == p.CONFIRM:
            if server._pending == "mission1":
                server.run_mission1(link, link.send)
            elif server._pending == "mission2":
                server.run_mission2(link, link.send)
            else:
                server.run_flight(link, link.send)
        else:
            for reply in server.handle(msg):
                link.send(reply)


def wait_for_fc(link, args):
    """Retry the Pixhawk connection forever. While it's missing, keep answering
    the LoRa link so the operator sees WHY the drone isn't ready (instead of
    silence when the Pixhawk USB is unplugged or still booting)."""
    last_note = 0.0
    while True:
        device = find_fc_device() if args.fc_device == "auto" else args.fc_device
        try:
            print("FC device: {}".format(device), flush=True)
            return RealFC(device, args.fc_baud)
        except SystemExit:
            pass  # missions.connect() exits on no-heartbeat; treat as retry
        except Exception as exc:
            print("FC connect failed: {}".format(exc), flush=True)
        if time.time() - last_note > 30:
            print("Pixhawk not connected — retrying every 5 s...", flush=True)
            last_note = time.time()
        end = time.time() + 5
        while time.time() < end:
            msg = link.poll()
            if msg is None:
                time.sleep(0.05)
                continue
            t = msg.get("t")
            seq = msg.get("seq", 0)
            if t == p.HELLO:
                link.send(p.message(p.HELLO_ACK, seq, proto=p.PROTO_VERSION,
                                    fc="disconnected", props_off=True,
                                    armed=False, gps="n/a", batt=0))
                link.send(p.message(p.LOG, 0,
                                    text="Pixhawk NOT connected to the Jetson — check its USB cable"))
            elif t == p.PING:
                link.send(p.message(p.PONG, seq))
            elif t == p.GET_MENU:
                link.send(p.message(p.MENU, seq, item=None, last=True))
                link.send(p.message(p.LOG, 0, text="no missions — Pixhawk not connected"))


def main():
    ap = argparse.ArgumentParser(description="Venator Jetson C2 server")
    ap.add_argument("--lora-port", default="auto",
                    help="LoRa stick serial port, or 'auto' to detect by stable ID")
    ap.add_argument("--lora-baud", type=int, default=115200)
    ap.add_argument("--fc-device", default="auto",
                    help="Pixhawk serial port, or 'auto' to detect by stable ID")
    ap.add_argument("--fc-baud", type=int, default=115200)
    ap.add_argument("--motors", type=int, default=6)
    ap.add_argument("--real", action="store_true",
                    help="connect the real Pixhawk (default is a safe mock FC)")
    ap.add_argument("--props-on", action="store_true",
                    help="declare props ON (enables flight missions, disables motor tests)")
    args = ap.parse_args()

    # Line-buffer stdout so logs reach the journal immediately under systemd.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, OSError):
        pass

    # Open the LoRa link FIRST so the operator gets answers even when the
    # flight controller is missing (previously this crash-looped in silence).
    lora_port = find_lora_device() if args.lora_port == "auto" else args.lora_port
    print("LoRa device: {}".format(lora_port), flush=True)
    try:
        link = LoRaLink(lora_port, args.lora_baud)
    except Exception as e:
        sys.exit(f"Could not open LoRa port {lora_port}: {e}")

    if args.real:
        if missions is None:
            sys.exit("pymavlink/missions unavailable — run with the venv, or drop --real")
        fc = wait_for_fc(link, args)
    else:
        fc = MockFC()

    server = C2Server(fc, props_off=not args.props_on, motors=args.motors)

    try:
        serve(server, link)
    except KeyboardInterrupt:
        print("\nC2 server stopped.")
    finally:
        link.close()
        fc.close()


if __name__ == "__main__":
    main()
