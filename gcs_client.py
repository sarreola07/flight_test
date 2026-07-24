#!/usr/bin/env python3
"""
Venator GCS — portable laptop client (Windows / Mac / Linux).

Plug in the Heltec LoRa stick, run this, and it links to the Jetson and shows
the mission menu the Jetson advertises. One option lets you send waypoints for
the drone to fly to.

    python gcs_client.py            # auto-detect the LoRa stick
    python gcs_client.py --port COM5
    python gcs_client.py --mock     # no hardware: demo against a fake Jetson

Packaged into VenatorGCS(.exe/.app) by the GitHub Actions workflow.
"""
import argparse
import sys
import threading
import time

import c2_protocol as p

CP210X_VID = 0x10C4   # Heltec V3 onboard USB-serial (Silicon Labs CP2102)

C_OK = "\033[92m"
C_WARN = "\033[93m"
C_ERR = "\033[91m"
C_DIM = "\033[90m"
C_OFF = "\033[0m"


# --------------------------------------------------------------------------
# Transports: something that can send() and poll() protocol messages.
# --------------------------------------------------------------------------
class SerialTransport:
    """Talk protocol lines over the LoRa serial link (real hardware)."""

    def __init__(self, port, baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self._buf = ""

    def send(self, m):
        self.ser.write(p.encode(m).encode("utf-8"))

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


class MockTransport:
    """In-process fake Jetson so the client runs with no hardware."""

    def __init__(self):
        self.jetson = MockJetson()
        self._out = []          # messages queued for the client to poll()

    def send(self, m):
        self._out.extend(self.jetson.handle(m))

    def poll(self):
        return self._out.pop(0) if self._out else None

    def close(self):
        pass


class MockJetson:
    """Minimal fake of the Jetson C2 server for offline demos."""

    MENU = [
        {"id": 1, "name": "Sequential motor test", "needs": "props_off"},
        {"id": 2, "name": "All-motor test",        "needs": "props_off"},
        {"id": 3, "name": "Fly to waypoints",      "needs": "props_on", "gps": True},
        {"id": 4, "name": "Mission 1: hover + detect + land", "needs": "props_on", "gps": True},
        {"id": 5, "name": "Mission 2: follow person", "needs": "props_on", "gps": True},
    ]

    def handle(self, m):
        t = m.get("t")
        seq = m.get("seq", 0)
        if t == p.HELLO:
            return [p.message(p.HELLO_ACK, seq, proto=p.PROTO_VERSION,
                              armed=False, gps="no fix", batt=11.4)]
        if t == p.GET_MENU:
            return [p.message(p.MENU, seq, item=it, last=(i == len(self.MENU) - 1))
                    for i, it in enumerate(self.MENU)]
        if t == p.PING:
            return [p.message(p.PONG, seq)]
        if t == p.RUN:
            mid = m.get("id")
            item = next((i for i in self.MENU if i["id"] == mid), None)
            if item is None:
                return [p.message(p.ACK, seq, id=mid, accepted=False, reason="unknown mission")]
            if item.get("gps"):
                return [p.message(p.ACK, seq, id=mid, accepted=False,
                                  reason="no GPS fix (mock)")]
            return [p.message(p.ACK, seq, id=mid, accepted=True),
                    p.message(p.DONE, seq, id=mid, result="ok (mock)")]
        if t == p.WP_BEGIN:
            self._wp = []
            return [p.message(p.ACK, seq, ready=True)]
        if t == p.WP:
            self._wp.append(m)
            return [p.message(p.ACK, seq, i=m.get("i"))]
        if t == p.WP_END:
            n = len(getattr(self, "_wp", []))
            return [p.message(p.ACK, seq, uploaded=n, rejected=0,
                              note="stored (mock — no GPS, would not fly)")]
        if t == p.FLY_REQ:
            return [p.message(p.ACK, seq, accepted=False, reason="no GPS fix (mock)")]
        return []


# --------------------------------------------------------------------------
# Client
# --------------------------------------------------------------------------
def autodetect_port():
    """Return the serial device of the first CP210x (Heltec) found, or None."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    for pi in list_ports.comports():
        if pi.vid == CP210X_VID:
            return pi.device
    return None


class Client:
    def __init__(self, transport):
        self.tx = transport
        self.seq = 0

    def _next_seq(self):
        self.seq += 1
        return self.seq

    def request(self, m, want, timeout=5.0):
        """Send a message and wait for a reply of type in `want` (set/str)."""
        want = {want} if isinstance(want, str) else set(want)
        self.tx.send(m)
        end = time.time() + timeout
        while time.time() < end:
            r = self.tx.poll()
            if r is None:
                time.sleep(0.02)
                continue
            if r.get("t") == p.LOG:
                print(f"{C_DIM}  drone: {r.get('text','')}{C_OFF}")
                continue
            if r.get("t") in want:
                return r
        return None

    def handshake(self):
        print("Linking to drone...")
        r = self.request(p.message(p.HELLO, self._next_seq()), p.HELLO_ACK, timeout=8)
        if r is None:
            print(f"{C_ERR}No response from the Jetson. Is it powered and in range?{C_OFF}")
            return False
        print(f"{C_OK}Connected ✓{C_OFF}  proto v{r.get('proto')}  "
              f"armed={r.get('armed')}  gps={r.get('gps')}  batt={r.get('batt')} V")
        return True

    def get_menu(self):
        # The menu is streamed one item per packet (each fits a LoRa frame), so
        # collect MENU messages until the one flagged last.
        self.tx.send(p.message(p.GET_MENU, self._next_seq()))
        items = []
        end = time.time() + 8
        while time.time() < end:
            r = self.tx.poll()
            if r is None:
                time.sleep(0.02)
                continue
            if r.get("t") == p.MENU:
                if r.get("item"):
                    items.append(r["item"])
                if r.get("last"):
                    break
        return items

    def run_mission(self, mid):
        r = self.request(p.message(p.RUN, self._next_seq(), id=mid), {p.ACK}, timeout=8)
        if r is None:
            print(f"{C_ERR}No ACK — command may not have arrived.{C_OFF}")
            return
        if not r.get("accepted", False):
            print(f"{C_WARN}Rejected: {r.get('reason','?')}{C_OFF}")
            return
        print(f"{C_OK}Accepted.{C_OFF} Running...")
        d = self.request(p.message(p.PING, self.seq), {p.DONE}, timeout=60)
        if d:
            print(f"{C_OK}Done:{C_OFF} {d.get('result','')}")

    def upload_waypoints(self):
        try:
            n = int(input("How many waypoints? ").strip())
        except ValueError:
            print("Not a number."); return
        wps = []
        for i in range(n):
            raw = input(f"  #{i+1} as lat,lon,alt(m): ").strip()
            try:
                lat, lon, alt = (float(x) for x in raw.split(","))
            except ValueError:
                print("  bad format, expected e.g. 37.4208,-122.0841,30"); return
            wps.append((lat, lon, alt))
        print(f"Sending {len(wps)} waypoints...")
        self.request(p.message(p.WP_BEGIN, self._next_seq(), count=len(wps)), {p.ACK})
        for i, (lat, lon, alt) in enumerate(wps):
            self.request(p.message(p.WP, self._next_seq(), i=i, lat=lat, lon=lon, alt=alt), {p.ACK})
        r = self.request(p.message(p.WP_END, self._next_seq()), {p.ACK})
        if not r or not r.get("uploaded"):
            print(f"{C_ERR}Upload failed:{C_OFF} {r.get('note','') if r else 'no response'}")
            return
        print(f"{C_OK}Upload result:{C_OFF} {r.get('uploaded')} stored, "
              f"{r.get('rejected')} rejected. {r.get('note','')}")
        self.fly_uploaded_mission()

    def fly_uploaded_mission(self):
        """Two-step arm+fly of the uploaded mission, then monitor the flight."""
        if input("\nArm and FLY this mission now? (type FLY, else skip): ").strip().lower() != "fly":
            print("Mission stored on the drone; not flying now.")
            return
        # step 1 — ask the drone if it's safe to fly
        ack = self.request(p.message(p.FLY_REQ, self._next_seq()), {p.ACK}, timeout=8)
        if not ack or not ack.get("accepted"):
            print(f"{C_WARN}Cannot fly: {ack.get('reason') if ack else 'no response'}{C_OFF}")
            return
        # step 2 — explicit second confirmation
        print(f"{C_WARN}SAFETY: {ack.get('reason')}{C_OFF}")
        print(f"{C_WARN}The drone will ARM and TAKE OFF. Ctrl-C during flight = abort (return home).{C_OFF}")
        if input("Type LAUNCH to arm and take off: ").strip().lower() != "launch":
            print("Aborted — not flying.")
            return
        self.tx.send(p.message(p.CONFIRM, self._next_seq()))
        self.monitor_flight()

    def mission_1_flow(self, mid):
        """Mission 1: server prepares (takeoff/hover/detect/land), two-step confirm."""
        ack = self.request(p.message(p.RUN, self._next_seq(), id=mid), {p.ACK}, timeout=10)
        if not ack or not ack.get("accepted"):
            print(f"{C_WARN}Cannot start Mission 1: {ack.get('reason') if ack else 'no response'}{C_OFF}")
            return
        print(f"{C_WARN}SAFETY: {ack.get('reason')}{C_OFF}")
        print(f"{C_DIM}Start the AI camera first (./ai_camera.sh start) so it can detect people.{C_OFF}")
        print(f"{C_WARN}The drone will ARM and TAKE OFF. Ctrl-C during flight = abort (return home).{C_OFF}")
        if input("Type LAUNCH to arm and take off: ").strip().lower() != "launch":
            print("Aborted — not flying.")
            return
        self.tx.send(p.message(p.CONFIRM, self._next_seq()))
        self.monitor_flight()

    def mission_2_flow(self, mid):
        """Mission 2 (follow-me): prepare, two-step confirm, then follow monitor."""
        ack = self.request(p.message(p.RUN, self._next_seq(), id=mid), {p.ACK}, timeout=10)
        if not ack or not ack.get("accepted"):
            print(f"{C_WARN}Cannot start Mission 2: {ack.get('reason') if ack else 'no response'}{C_OFF}")
            return
        print(f"{C_WARN}SAFETY: {ack.get('reason')}{C_OFF}")
        print(f"{C_DIM}Start the AI camera first (./ai_camera.sh start) so it can see the person.{C_OFF}")
        print(f"{C_WARN}The drone will ARM, TAKE OFF, and FOLLOW. Ctrl-C = abort (RTL).{C_OFF}")
        if input("Type LAUNCH to arm and take off: ").strip().lower() != "launch":
            print("Aborted — not flying.")
            return
        self.tx.send(p.message(p.CONFIRM, self._next_seq()))
        self.monitor_follow()

    def monitor_follow(self):
        """Follow monitor: pings + live status; type Enter to STOP (land), Ctrl-C to abort (RTL)."""
        print(f"{C_OK}Following.{C_OFF} Press Enter to STOP (land in place); Ctrl-C to abort (return home).")
        stop_evt = threading.Event()
        threading.Thread(target=lambda: (sys.stdin.readline(), stop_evt.set()), daemon=True).start()
        last_ping = 0.0
        try:
            while True:
                now = time.time()
                if stop_evt.is_set():
                    stop_evt.clear()
                    print(f"{C_WARN}STOP — landing in place...{C_OFF}")
                    self.tx.send(p.message(p.STOP, self._next_seq()))
                if now - last_ping > 1.0:
                    self.tx.send(p.message(p.PING, self._next_seq()))
                    last_ping = now
                r = self.tx.poll()
                if r is None:
                    time.sleep(0.05)
                    continue
                t = r.get("t")
                if t == p.STATUS:
                    who = "person YES" if r.get("person") else "person no"
                    if r.get("pz"):
                        who += f" @ {r.get('pz')} m"
                    foll = " following" if r.get("following") else ""
                    tag = "  [RTL]" if r.get("rtl") else ""
                    print(f"{C_OK}alt {r.get('alt')} m  {r.get('phase')}  {who}{foll}{tag}{C_OFF}")
                elif t == p.LOG:
                    print(f"{C_DIM}  drone: {r.get('text','')}{C_OFF}")
                elif t == p.DONE:
                    print(f"{C_OK}Mission 2 complete:{C_OFF} {r.get('result','')}")
                    return
        except KeyboardInterrupt:
            print(f"\n{C_WARN}ABORT — returning home...{C_OFF}")
            self.tx.send(p.message(p.ABORT, self._next_seq()))
            end = time.time() + 90
            while time.time() < end:
                r = self.tx.poll()
                if r and r.get("t") == p.DONE:
                    print(f"{C_OK}Mission 2 complete:{C_OFF} {r.get('result','')}")
                    return
                time.sleep(0.1)

    def monitor_flight(self):
        """Send flight heartbeats, show live status, Ctrl-C to abort (RTL)."""
        print(f"{C_OK}Launch confirmed.{C_OFF} Monitoring flight — Ctrl-C to abort (return home).")
        last_ping = 0.0
        try:
            while True:
                now = time.time()
                if now - last_ping > 1.0:
                    self.tx.send(p.message(p.PING, self._next_seq()))
                    last_ping = now
                r = self.tx.poll()
                if r is None:
                    time.sleep(0.05)
                    continue
                t = r.get("t")
                if t == p.STATUS:
                    tag = "  [RTL]" if r.get("rtl") else ""
                    phase = f"  {r['phase']}" if r.get("phase") else ""
                    who = ""
                    if r.get("person") is not None:
                        pz = r.get("pz")
                        who = ("  person YES" + (f" @ {pz} m" if pz else "")) if r["person"] else "  person no"
                        who += f" (seen {r.get('seen', 0)})"
                    print(f"{C_OK}alt {r.get('alt')} m  mode {r.get('mode')}{phase}  "
                          f"armed {r.get('armed')}{who}{tag}{C_OFF}")
                elif t == p.LOG:
                    print(f"{C_DIM}  drone: {r.get('text','')}{C_OFF}")
                elif t == p.DONE:
                    print(f"{C_OK}Flight complete:{C_OFF} {r.get('result','')}")
                    return
        except KeyboardInterrupt:
            print(f"\n{C_WARN}ABORT — telling the drone to return home...{C_OFF}")
            self.tx.send(p.message(p.ABORT, self._next_seq()))
            end = time.time() + 90
            while time.time() < end:
                r = self.tx.poll()
                if r and r.get("t") == p.DONE:
                    print(f"{C_OK}Flight complete:{C_OFF} {r.get('result','')}")
                    return
                time.sleep(0.1)


def flag_label(item):
    bits = []
    if item.get("needs") == "props_off":
        bits.append("props OFF")
    elif item.get("needs") == "props_on":
        bits.append("props ON")
    if item.get("gps"):
        bits.append("needs GPS")
    return f"  {C_DIM}({', '.join(bits)}){C_OFF}" if bits else ""


def menu_loop(client):
    items = client.get_menu()
    if not items:
        print(f"{C_ERR}The drone sent no menu.{C_OFF}")
        return
    while True:
        print("\n=== Missions ===")
        for it in items:
            print(f"  {it['id']}) {it['name']}{flag_label(it)}")
        print("  q) Quit")
        choice = input("\nSelect: ").strip().lower()
        if choice == "q":
            print("Bye.")
            return
        if not choice.isdigit():
            print("Enter a number or q."); continue
        mid = int(choice)
        item = next((i for i in items if i["id"] == mid), None)
        if item is None:
            print("No such mission."); continue
        name = item["name"].lower()
        if name.startswith("fly to waypoints"):
            client.upload_waypoints()          # collect + upload + two-step fly
        elif name.startswith("mission 1"):
            client.mission_1_flow(mid)          # prepare + two-step fly (hover+detect)
        elif name.startswith("mission 2"):
            client.mission_2_flow(mid)          # prepare + two-step fly (follow-me)
        else:
            client.run_mission(mid)             # motor tests etc.


def main():
    ap = argparse.ArgumentParser(description="Venator GCS — portable LoRa client")
    ap.add_argument("--port", help="serial port (default: auto-detect the Heltec)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mock", action="store_true", help="run against a fake Jetson (no hardware)")
    args = ap.parse_args()

    print("=" * 48)
    print(" Venator GCS — drone command over LoRa")
    print("=" * 48)

    if args.mock:
        print(f"{C_WARN}[mock mode] no hardware — talking to a simulated Jetson{C_OFF}")
        tx = MockTransport()
    else:
        port = args.port or autodetect_port()
        if not port:
            print(f"{C_ERR}No LoRa stick found.{C_OFF} Plug in the Heltec, or pass --port, "
                  f"or use --mock to demo without hardware.")
            return 1
        print(f"Using LoRa stick on {port}")
        try:
            tx = SerialTransport(port, args.baud)
        except Exception as e:
            print(f"{C_ERR}Could not open {port}: {e}{C_OFF}")
            return 1

    client = Client(tx)
    try:
        if client.handshake():
            menu_loop(client)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        tx.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
