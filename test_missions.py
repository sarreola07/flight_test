#!/usr/bin/env python3
"""
Regression tests for the flight missions (mock flight controller — nothing flies).

    ./venv/bin/python test_missions.py

Covers the paths that must not break before a field test:
  * menu integrity + packet sizes small enough for the LoRa firmware buffer
  * props interlock (flight vs motor tests) and the runtime props toggle
  * Mission 1: takeoff -> hover + person detection -> land
  * Mission 2: takeoff -> OFFBOARD follow -> STOP -> land
  * link-loss -> RTL failsafe on both missions
  * client tolerates corrupted menu packets instead of crashing
"""
import json
import socket
import sys
import threading
import time

import c2_protocol as c2
import gcs_client as gc
import jetson_c2_server as srv

FAILURES = []


def check(label, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + label + (f"   {detail}" if detail else ""))
    if not cond:
        FAILURES.append(label)


class FakeLink:
    """Stands in for the LoRa link. Pings for `ping_for` s, then goes silent.
    Optionally injects a STOP after `stop_at` seconds."""

    def __init__(self, ping_for=100.0, stop_at=None):
        self.t0 = time.time()
        self.ping_for = ping_for
        self.stop_at = stop_at
        self.sent = []
        self._stopped = False

    def poll(self):
        el = time.time() - self.t0
        if self.stop_at is not None and el > self.stop_at and not self._stopped:
            self._stopped = True
            return c2.message(c2.STOP, 0)
        return c2.message(c2.PING, 0) if el < self.ping_for else None

    def send(self, m):
        self.sent.append(m)

    def texts(self):
        return [m.get("text", "") for m in self.sent if m["t"] == c2.LOG]

    def phases(self):
        out = []
        for m in self.sent:
            if m["t"] == c2.STATUS and m.get("phase") not in out:
                out.append(m.get("phase"))
        return out

    def done(self):
        d = [m for m in self.sent if m["t"] == c2.DONE]
        return d[-1]["result"] if d else None


def person_injector(stop_evt):
    """Publish OAK-D-style person packets to the camera UDP port."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while not stop_evt.is_set():
        s.sendto(json.dumps({"x": 0.5, "y": 0.0, "z": 4.0, "conf": 0.9}).encode(),
                 ("127.0.0.1", srv.CAMERA_UDP_PORT))
        time.sleep(0.1)
    s.close()


def make_server(gps=True, props_off=False, mission=False):
    # Sample status faster than the default 1 s so short test flights capture
    # every phase transition.
    srv.STATUS_INTERVAL_S = 0.25
    fc = srv.MockFC(gps=gps)
    if mission:
        fc.upload_waypoints([(37.42, -122.08, 5.0)])
    return srv.C2Server(fc, props_off=props_off)


# ---------------------------------------------------------------- menu / packets
def test_menu():
    print("\n[menu + LoRa packet safety]")
    s = make_server()
    msgs = s.handle(c2.message(c2.GET_MENU, 1))
    sizes = [len(c2.encode(m)) for m in msgs]
    check("every menu item has id and name",
          all(isinstance(m["item"].get("id"), int)
              and isinstance(m["item"].get("name"), str) for m in msgs))
    check("exactly one packet flagged last",
          sum(1 for m in msgs if m.get("last")) == 1)
    check("each packet < 200 B (firmware buffer is ~256 B)",
          max(sizes) < 200, f"largest {max(sizes)} B")
    check("no server-internal keys leak to the wire",
          not any(k in m["item"] for m in msgs for k in ("fly", "action")))
    check("sends are paced to avoid buffer overflow",
          srv.LoRaLink.MIN_SEND_GAP_S >= 0.2, f"{srv.LoRaLink.MIN_SEND_GAP_S}s gap")


# ---------------------------------------------------------------- props gating
def test_props():
    print("\n[props interlock + runtime toggle]")
    s = make_server(props_off=False)
    check("props default to ON (flight ready out of the box)", s.props_off is False)
    for mid, label in ((4, "Mission 1"), (5, "Mission 2")):
        out = []
        s.run_and_report(c2.message(c2.RUN, 1, id=mid), out.append)
        check(f"{label} accepted with props ON", out[-1].get("accepted") is True,
              out[-1].get("reason", "")[:44])
    out = []
    s.run_and_report(c2.message(c2.RUN, 1, id=1), out.append)
    check("motor test refused with props ON", out[-1].get("accepted") is False)
    out = []
    s.run_and_report(c2.message(c2.RUN, 1, id=6), out.append)
    check("props toggle flips state", s.props_off is True, out[-1].get("result", ""))
    out = []
    s.run_and_report(c2.message(c2.RUN, 1, id=4), out.append)
    check("Mission 1 refused after toggling props OFF", out[-1].get("accepted") is False)


def test_fly_gates():
    print("\n[waypoint flight gates]")
    cases = [
        (make_server(gps=False, props_off=True, mission=True), "props OFF"),
        (make_server(gps=False, props_off=False, mission=True), "no GPS"),
        (make_server(gps=True, props_off=False, mission=False), "no mission uploaded"),
    ]
    for s, why in cases:
        r = s.handle(c2.message(c2.FLY_REQ, 1))[0]
        check(f"FLY_REQ refused: {why}", r.get("accepted") is False, r.get("reason", ""))
    s = make_server(gps=True, props_off=False, mission=True)
    r = s.handle(c2.message(c2.FLY_REQ, 1))[0]
    check("FLY_REQ accepted when all conditions met", r.get("accepted") is True)
    check("acceptance demands a second confirmation", r.get("confirm_required") is True)


# ---------------------------------------------------------------- missions
def run_mission(which, ping_for=100.0, stop_at=None, people=True):
    s = make_server()
    out = []
    s.run_and_report(c2.message(c2.RUN, 1, id=4 if which == 1 else 5), out.append)
    link = FakeLink(ping_for=ping_for, stop_at=stop_at)
    stop_evt = threading.Event()
    if people:
        threading.Thread(target=person_injector, args=(stop_evt,), daemon=True).start()
    (s.run_mission1 if which == 1 else s.run_mission2)(link, link.send)
    stop_evt.set()
    return s, link


def test_mission1():
    print("\n[Mission 1: hover + detect + land]")
    srv.HOVER_TIME_S = 2.0
    s, link = run_mission(1)
    check("phases takeoff -> hover -> land",
          link.phases() == ["takeoff", "hover", "land"], str(link.phases()))
    check("PX4 told the intended takeoff altitude",
          getattr(s.fc, "_takeoff_alt", None) == srv.HOVER_ALT_M,
          f"MIS_TAKEOFF_ALT={getattr(s.fc, '_takeoff_alt', None)}")
    check("people were detected and counted",
          any(m.get("seen", 0) > 0 for m in link.sent if m["t"] == c2.STATUS))
    check("ends landed & disarmed", "landed" in (link.done() or ""), link.done())

    print("\n[Mission 1: link lost mid-flight]")
    srv.LINK_LOSS_S = 2.0
    _, link = run_mission(1, ping_for=1.0)
    check("link loss announced", any("LINK LOST" in t for t in link.texts()))
    check("returned home (RTL)", "RTL" in (link.done() or ""), link.done())


def test_mission2():
    print("\n[Mission 2: follow person]")
    for x, z, wvx, wyaw in ((0.0, 4.0, +1, 0), (0.0, 2.0, -1, 0),
                            (0.0, 3.0, 0, 0), (0.6, 3.0, 0, +1), (-0.6, 3.0, 0, -1)):
        vx, yaw = srv.C2Server._follow_setpoint({"x": x, "z": z})
        ok = ((vx > 0) == (wvx > 0)) and ((vx < 0) == (wvx < 0)) \
             and ((yaw > 0) == (wyaw > 0)) and ((yaw < 0) == (wyaw < 0))
        check(f"controller x={x:+.1f} z={z:.1f} -> vx={vx:+.2f} yaw={yaw:+.2f}", ok)

    # stop late enough that the drone reaches altitude and actually follows first
    s, link = run_mission(2, stop_at=3.0)
    check("entered follow phase", "follow" in link.phases(), str(link.phases()))
    check("actively followed the person",
          any(m.get("following") for m in link.sent if m["t"] == c2.STATUS))
    check("velocity setpoints were streamed", getattr(s.fc, "_last_vel", None) is not None,
          str(getattr(s.fc, "_last_vel", None)))
    check("STOP landed it in place", "stopped" in (link.done() or ""), link.done())

    print("\n[Mission 2: link lost while following]")
    _, link = run_mission(2, ping_for=1.0)
    check("returned home (RTL)", "RTL" in (link.done() or ""), link.done())

    print("\n[Mission 2: camera loss -> hover, never drift]")
    s, link = run_mission(2, stop_at=2.0, people=False)
    check("no person -> zero velocity commanded",
          getattr(s.fc, "_last_vel", (1, 0, 0, 1))[0] == 0.0
          and getattr(s.fc, "_last_vel", (1, 0, 0, 1))[3] == 0.0,
          str(getattr(s.fc, "_last_vel", None)))


# ---------------------------------------------------------------- client robustness
def test_client_robustness():
    print("\n[client survives corrupted LoRa packets]")

    class BadTx:
        def __init__(self):
            self.out = []

        def send(self, m):
            if m.get("t") == c2.GET_MENU:
                self.out = [
                    c2.message(c2.MENU, 1, item={"id": 1, "name": "Sequential motor test"}),
                    c2.message(c2.MENU, 1, item={"id": 6}),          # corrupt: no name
                    c2.message(c2.MENU, 1, item="garbage"),           # corrupt: not a dict
                    c2.message(c2.MENU, 1, item={"id": 1, "name": "Sequential motor test"}),  # dup
                    c2.message(c2.MENU, 1, item={"id": 4, "name": "Mission 1"}, last=True),
                ]

        def poll(self):
            return self.out.pop(0) if self.out else None

        def close(self):
            pass

    items = gc.Client(BadTx()).get_menu()
    check("corrupted items dropped", [i["id"] for i in items] == [1, 4], str(items))
    try:
        for it in items:
            f"{it.get('id')}) {it.get('name')}{gc.flag_label(it)}"
        check("menu renders without crashing", True)
    except Exception as exc:
        check("menu renders without crashing", False, str(exc))


if __name__ == "__main__":
    test_menu()
    test_props()
    test_fly_gates()
    test_mission1()
    test_mission2()
    test_client_robustness()
    print("\n" + "=" * 52)
    if FAILURES:
        print(f"{len(FAILURES)} FAILURE(S): " + "; ".join(FAILURES))
        sys.exit(1)
    print("ALL CHECKS PASSED")
