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
import sys
import time

import c2_protocol as p

MAX_ALT_M = 50.0   # reject uploaded waypoints above this relative altitude

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
    """A fake flight controller so the server runs and is testable with no drone."""
    name = "mock"

    def status(self):
        return {"armed": False, "gps": "no fix", "batt": 11.4}

    def has_gps_fix(self):
        return False

    def run_mission(self, mid, motors=6):
        time.sleep(0.1)
        return True, "ok (mock)"

    def upload_waypoints(self, wps):
        return len(wps), 0, "stored (mock — no GPS, would not fly)"

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

    def menu(self):
        return [
            {"id": 1, "name": "Sequential motor test", "needs": "props_off"},
            {"id": 2, "name": "All-motor test", "needs": "props_off"},
            {"id": 3, "name": "Auto takeoff & land (3 ft)", "needs": "props_on", "gps": True},
            {"id": 4, "name": "Fly to waypoints", "needs": "props_on", "gps": True},
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
        return []

    def run_and_report(self, msg, send):
        """Handle a RUN: ACK, execute (blocking), then DONE — via send(msg)."""
        seq = msg.get("seq", 0)
        mid = msg.get("id")
        accepted, reason = self._validate(mid)
        send(p.message(p.ACK, seq, id=mid, accepted=accepted, reason=reason))
        if not accepted:
            self.log(f"RUN {mid} rejected: {reason}")
            return
        self.log(f"RUN {mid} accepted — executing")
        try:
            ok, result = self.fc.run_mission(mid, self.motors)
        except Exception as exc:
            ok, result = False, f"error: {exc}"
        send(p.message(p.DONE, seq, id=mid, result=result if ok else f"FAILED: {result}"))


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
        if msg.get("t") == p.RUN:
            server.run_and_report(msg, link.send)
        else:
            for reply in server.handle(msg):
                link.send(reply)


def main():
    ap = argparse.ArgumentParser(description="Venator Jetson C2 server")
    ap.add_argument("--lora-port", default="/dev/ttyUSB0")
    ap.add_argument("--lora-baud", type=int, default=115200)
    ap.add_argument("--fc-device", default="/dev/ttyACM0")
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

    if args.real:
        if missions is None:
            sys.exit("pymavlink/missions unavailable — run with the venv, or drop --real")
        fc = RealFC(args.fc_device, args.fc_baud)
    else:
        fc = MockFC()

    server = C2Server(fc, props_off=not args.props_on, motors=args.motors)

    try:
        link = LoRaLink(args.lora_port, args.lora_baud)
    except Exception as e:
        sys.exit(f"Could not open LoRa port {args.lora_port}: {e}")

    try:
        serve(server, link)
    except KeyboardInterrupt:
        print("\nC2 server stopped.")
    finally:
        link.close()
        fc.close()


if __name__ == "__main__":
    main()
