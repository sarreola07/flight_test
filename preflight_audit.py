#!/usr/bin/env python3
"""
Pre-flight audit — read-only health report from the Pixhawk.

Checks the things that block or endanger a first flight: sensor health,
compass setup, GPS, flight-mode switch assignments, failsafe actions, and
battery. Sends no commands and never arms.

    sudo systemctl stop venator-c2
    ./venv/bin/python preflight_audit.py
    sudo systemctl start venator-c2
"""
import glob
import struct
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink missing — run with ./venv/bin/python")

OK = "\033[92m[ OK ]\033[0m"
BAD = "\033[91m[FAIL]\033[0m"
INF = "\033[94m[INFO]\033[0m"
WRN = "\033[93m[WARN]\033[0m"

FLTMODES = {
    -1: "Unassigned", 0: "Manual", 1: "Altitude", 2: "Position", 3: "Mission",
    4: "Hold/Loiter", 5: "Return(RTL)", 6: "Acro", 7: "Offboard", 8: "Stabilized",
    9: "Rattitude", 10: "Takeoff", 11: "Land", 12: "Follow Me", 13: "Precision Land",
}
RCL_ACT = {0: "Disabled", 1: "Hold", 2: "Return(RTL)", 3: "Land", 5: "Terminate", 6: "Lockdown"}
MAG_ROT = {-1: "not set", 0: "ROLL_0 (forward, default)", 2: "YAW_90", 4: "YAW_180", 6: "YAW_270"}


def find_fc():
    for pat in ("/dev/serial/by-id/*PX4*", "/dev/serial/by-id/*FMU*"):
        m = sorted(glob.glob(pat))
        if m:
            return m[0]
    a = sorted(glob.glob("/dev/ttyACM*"))
    return a[0] if a else "/dev/ttyACM0"


def readp(m, name, timeout=2.5):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if p and p.param_id.strip("\x00") == name:
            if p.param_type in (mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT32):
                return struct.unpack("<i", struct.pack("<f", p.param_value))[0]
            return round(p.param_value, 4)
    return None


def main():
    dev = find_fc()
    print(f"{INF} Connecting to {dev} ...")
    m = mavutil.mavlink_connection(dev, baud=115200)
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    if m.wait_heartbeat(timeout=10) is None:
        sys.exit(f"{BAD} No heartbeat (is venator-c2 still running?)")
    print(f"{OK} Connected (system {m.target_system})")

    # ---------- sensor health ----------
    print(f"\n=== Sensor health (what preflight is complaining about) ===")
    s = m.recv_match(type="SYS_STATUS", blocking=True, timeout=5)
    if s:
        bad = []
        for bit, e in sorted(mavutil.mavlink.enums["MAV_SYS_STATUS_SENSOR"].items()):
            if bit == 0:
                continue
            name = e.name.replace("MAV_SYS_STATUS_SENSOR_", "").replace("MAV_SYS_STATUS_", "")
            present = s.onboard_control_sensors_present & bit
            enabled = s.onboard_control_sensors_enabled & bit
            health = s.onboard_control_sensors_health & bit
            if present and enabled and not health:
                bad.append(name)
        if bad:
            for b in bad:
                print(f"  {BAD} unhealthy: {b}")
        else:
            print(f"  {OK} all enabled sensors healthy")
        print(f"  {INF} battery: {s.voltage_battery/1000.0:.2f} V")

    # ---------- compass ----------
    print(f"\n=== Compass / magnetometer ===")
    found = 0
    for i in range(4):
        mid = readp(m, f"CAL_MAG{i}_ID")
        if mid:
            found += 1
            rot = readp(m, f"CAL_MAG{i}_ROT")
            prio = readp(m, f"CAL_MAG{i}_PRIO")
            off = readp(m, f"CAL_MAG{i}_XOFF")
            calibrated = off is not None and off != 0.0
            kind = "external (GPS mast)" if (rot is not None and rot >= 0) else "internal (on the FC)"
            print(f"  {INF} MAG{i}: id={mid}  {kind}")
            print(f"        rotation={MAG_ROT.get(rot, rot)}  priority={prio}  "
                  f"offsets {'present (calibrated)' if calibrated else 'ZERO -> NOT CALIBRATED'}")
    if found == 0:
        print(f"  {BAD} no magnetometer registered — compass not detected")
    elif found > 1:
        print(f"  {WRN} {found} compasses registered. On a Pixhawk 2.4.8 the internal one is")
        print(f"       noisy; if calibration keeps failing, disable it and use the GPS mast only.")
    print(f"  {INF} SYS_HAS_MAG = {readp(m, 'SYS_HAS_MAG')}  (1 = a compass is required)")

    # ---------- GPS ----------
    print(f"\n=== GPS ===")
    g = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=5)
    fixes = {0: "no GPS", 1: "NO FIX", 2: "2D fix", 3: "3D fix", 4: "DGPS", 5: "RTK-float", 6: "RTK-fixed"}
    if g:
        mark = OK if g.fix_type >= 3 else WRN
        print(f"  {mark} fix: {fixes.get(g.fix_type, g.fix_type)}   satellites: {g.satellites_visible}")
        if g.fix_type < 3:
            print(f"  {INF} expected indoors — needs open sky (1-5 min cold start outside)")
    else:
        print(f"  {BAD} no GPS_RAW_INT — module not talking")

    # ---------- flight modes ----------
    print(f"\n=== Flight-mode switch (channel {readp(m, 'RC_MAP_FLTMODE')}) ===")
    stab_pos = []
    for i in range(1, 7):
        v = readp(m, f"COM_FLTMODE{i}")
        if v is None:
            continue
        label = FLTMODES.get(v, v)
        if v == 8:
            stab_pos.append(i)
        mark = OK if v == 8 else INF
        print(f"  {mark} switch position {i}: {label}")
    if stab_pos:
        print(f"  {OK} Stabilized is on position(s) {stab_pos} -> use this for manual RC flight")
    else:
        print(f"  {WRN} No switch position is set to Stabilized. Assign one in QGC")
        print(f"       (Vehicle Setup -> Flight Modes) for manual no-GPS flight.")

    # ---------- safety / failsafe ----------
    print(f"\n=== Safety & failsafe ===")
    kill = readp(m, "RC_MAP_KILL_SW")
    print(f"  {OK if kill else WRN} RC_MAP_KILL_SW = {kill}"
          f"  {'(kill switch mapped)' if kill else '(NO kill switch — map one in QGC)'}")
    rcl = readp(m, "NAV_RCL_ACT")
    print(f"  {INF} NAV_RCL_ACT (RC loss)   = {rcl} -> {RCL_ACT.get(rcl, rcl)}")
    if rcl == 2:
        print(f"  {WRN} RC-loss action is RTL, which needs GPS. For no-GPS manual flight set it")
        print(f"       to Land (3) so a lost transmitter gives a controlled descent.")
    dll = readp(m, "NAV_DLL_ACT")
    print(f"  {INF} NAV_DLL_ACT (datalink)  = {dll} -> {RCL_ACT.get(dll, dll)}")
    print(f"  {INF} COM_ARM_WO_GPS = {readp(m, 'COM_ARM_WO_GPS')}  (1 = manual arming without GPS allowed)")
    print(f"  {INF} SYS_AUTOSTART  = {readp(m, 'SYS_AUTOSTART')}  (6001 = DJI F550 hexa)")
    print(f"  {INF} COM_CPU_MAX    = {readp(m, 'COM_CPU_MAX')}  (-1 = CPU-load check disabled)")

    m.close()
    print(f"\n{INF} Audit complete — read-only, nothing was changed.")


if __name__ == "__main__":
    main()
