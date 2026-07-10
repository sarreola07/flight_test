#!/usr/bin/env python3
"""
Pixhawk 2.4.8 (PX4 v1.13.3) connection test.

Connects to the flight controller over MAVLink, waits for a heartbeat,
and prints firmware version, flight mode, battery, GPS and attitude data
so you can confirm the link is healthy without QGroundControl.

Usage:
    python3 check_pixhawk.py                      # defaults to /dev/ttyACM0
    python3 check_pixhawk.py --device /dev/ttyTHS1 --baud 921600
"""

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink is not installed. Run: source venv/bin/activate  (see README)")

PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
INFO = "\033[94m[INFO]\033[0m"


def decode_fw_version(version_int):
    """Decode the FLIGHT_SW_VERSION field of AUTOPILOT_VERSION (major.minor.patch)."""
    major = (version_int >> 24) & 0xFF
    minor = (version_int >> 16) & 0xFF
    patch = (version_int >> 8) & 0xFF
    return f"{major}.{minor}.{patch}"


def main():
    parser = argparse.ArgumentParser(description="Test MAVLink connection to Pixhawk")
    parser.add_argument("--device", default="/dev/ttyACM0", help="serial device (default: /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="baud rate (ignored on USB, default: 115200)")
    parser.add_argument("--timeout", type=int, default=10, help="seconds to wait for heartbeat")
    args = parser.parse_args()

    results = []

    # 1. Open the serial port
    print(f"{INFO} Connecting to {args.device} @ {args.baud} baud ...")
    try:
        master = mavutil.mavlink_connection(args.device, baud=args.baud)
    except Exception as e:
        print(f"{FAIL} Could not open {args.device}: {e}")
        print("      Is the Pixhawk plugged in? Are you in the 'dialout' group? (see README)")
        sys.exit(1)
    results.append(("Serial port opened", True))

    # 2. Announce ourselves, then wait for a heartbeat.
    # PX4 v1.13+ keeps the USB port silent until it detects MAVLink traffic
    # from the host, so we must send a GCS heartbeat first.
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb = master.wait_heartbeat(timeout=args.timeout)
    if hb is None:
        print(f"{FAIL} No heartbeat within {args.timeout}s. Check cable/power and device path.")
        sys.exit(1)
    autopilot = mavutil.mavlink.enums["MAV_AUTOPILOT"][hb.autopilot].name
    vehicle = mavutil.mavlink.enums["MAV_TYPE"][hb.type].name
    print(f"{PASS} Heartbeat from system {master.target_system} component {master.target_component}")
    print(f"{INFO}   Autopilot: {autopilot}   Vehicle type: {vehicle}")
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print(f"{INFO}   Armed: {armed}")
    results.append(("Heartbeat received", True))

    # 3. Ask for the firmware version
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    msg = master.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=5)
    if msg:
        print(f"{PASS} Firmware version: {decode_fw_version(msg.flight_sw_version)}")
        results.append(("Firmware version read", True))
    else:
        print(f"{FAIL} No AUTOPILOT_VERSION reply (link still OK, but version unknown)")
        results.append(("Firmware version read", False))

    # 4. Sample telemetry for a few seconds
    print(f"{INFO} Listening for telemetry (5 s) ...")
    wanted = {"SYS_STATUS": None, "GPS_RAW_INT": None, "ATTITUDE": None}
    deadline = time.time() + 5
    while time.time() < deadline and None in wanted.values():
        msg = master.recv_match(type=list(wanted), blocking=True, timeout=1)
        if msg and wanted.get(msg.get_type()) is None:
            wanted[msg.get_type()] = msg

    sysstat = wanted["SYS_STATUS"]
    if sysstat:
        volts = sysstat.voltage_battery / 1000.0
        print(f"{PASS} Battery: {volts:.2f} V" + ("  (no battery / USB power only)" if volts < 1 else ""))
        results.append(("SYS_STATUS (battery)", True))
    else:
        print(f"{FAIL} No SYS_STATUS message received")
        results.append(("SYS_STATUS (battery)", False))

    gps = wanted["GPS_RAW_INT"]
    if gps:
        fix_names = {0: "no GPS", 1: "no fix", 2: "2D fix", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}
        print(f"{PASS} GPS: {fix_names.get(gps.fix_type, gps.fix_type)}, satellites: {gps.satellites_visible}")
        results.append(("GPS_RAW_INT", True))
    else:
        print(f"{INFO} No GPS_RAW_INT (normal if no GPS module is connected)")
        results.append(("GPS_RAW_INT", None))

    att = wanted["ATTITUDE"]
    if att:
        import math
        print(f"{PASS} Attitude: roll {math.degrees(att.roll):+.1f}°  pitch {math.degrees(att.pitch):+.1f}°  yaw {math.degrees(att.yaw):+.1f}°")
        results.append(("ATTITUDE (IMU alive)", True))
    else:
        print(f"{FAIL} No ATTITUDE message received")
        results.append(("ATTITUDE (IMU alive)", False))

    # 5. Read one parameter to prove two-way communication
    master.mav.param_request_read_send(
        master.target_system, master.target_component, b"SYS_AUTOSTART", -1)
    p = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
    if p:
        # PX4 packs integer params byte-wise into the float field
        if p.param_type in (mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                            mavutil.mavlink.MAV_PARAM_TYPE_UINT32):
            import struct
            value = struct.unpack("<i", struct.pack("<f", p.param_value))[0]
        else:
            value = p.param_value
        print(f"{PASS} Parameter read: {p.param_id} = {value}")
        results.append(("Parameter read (two-way link)", True))
    else:
        print(f"{FAIL} Parameter read failed")
        results.append(("Parameter read (two-way link)", False))

    master.close()

    # Summary
    print("\n=== Summary ===")
    hard_fail = False
    for name, ok in results:
        mark = PASS if ok else (INFO if ok is None else FAIL)
        print(f"  {mark} {name}")
        if ok is False:
            hard_fail = True
    if hard_fail:
        print("\nSome checks failed — see messages above.")
        sys.exit(1)
    print("\nAll good: the Jetson can talk to the Pixhawk. ✅")


if __name__ == "__main__":
    main()
