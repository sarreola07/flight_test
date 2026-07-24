#!/usr/bin/env python3
"""
RC transmitter pre-flight check.

Shows live stick values from the Pixhawk so you can confirm every channel is
mapped and moving the right way BEFORE you fly. Also reports RC link health,
the channel mapping PX4 is using, and whether arming without GPS is allowed.

    sudo systemctl stop venator-c2     # free the Pixhawk serial port
    ./venv/bin/python check_rc.py
    sudo systemctl start venator-c2    # hand it back when done

Read-only: sends no commands, never arms.
"""
import argparse
import glob
import struct
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink missing — run with ./venv/bin/python")

PASS = "\033[92m[ OK ]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
INFO = "\033[94m[INFO]\033[0m"
WARN = "\033[93m[WARN]\033[0m"

# PX4 RC_MAP_* params: which RC channel drives each function
MAPS = [
    ("RC_MAP_THROTTLE", "throttle"),
    ("RC_MAP_ROLL", "roll"),
    ("RC_MAP_PITCH", "pitch"),
    ("RC_MAP_YAW", "yaw"),
    ("RC_MAP_FLTMODE", "flight-mode switch"),
    ("RC_MAP_KILL_SW", "KILL switch"),
    ("RC_MAP_ARM_SW", "arm switch"),
    ("RC_MAP_RETURN_SW", "return/RTL switch"),
]


def find_fc(default="/dev/ttyACM0"):
    for pat in ("/dev/serial/by-id/*PX4*", "/dev/serial/by-id/*FMU*"):
        m = sorted(glob.glob(pat))
        if m:
            return m[0]
    acms = sorted(glob.glob("/dev/ttyACM*"))
    return acms[0] if acms else default


def read_param(m, name, timeout=3):
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if p and p.param_id.strip("\x00") == name:
            if p.param_type in (mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                                mavutil.mavlink.MAV_PARAM_TYPE_UINT32):
                return struct.unpack("<i", struct.pack("<f", p.param_value))[0]
            return p.param_value
    return None


def bar(value, width=20):
    """Render 1000..2000 us as a small ASCII bar."""
    if value is None or value == 0:
        return "-" * width
    frac = max(0.0, min(1.0, (value - 1000) / 1000.0))
    n = int(frac * width)
    return "#" * n + "." * (width - n)


def rc_healthy(m):
    s = m.recv_match(type="SYS_STATUS", blocking=True, timeout=3)
    if s is None:
        return None
    bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
    return bool(s.onboard_control_sensors_enabled & bit
                and s.onboard_control_sensors_health & bit)


def main():
    ap = argparse.ArgumentParser(description="RC transmitter pre-flight check")
    ap.add_argument("--device", default="auto")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=int, default=30, help="live monitor duration")
    args = ap.parse_args()

    dev = find_fc() if args.device == "auto" else args.device
    print(f"{INFO} Connecting to {dev} ...")
    m = mavutil.mavlink_connection(dev, baud=args.baud)
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    if m.wait_heartbeat(timeout=10) is None:
        sys.exit(f"{FAIL} No heartbeat — is another program using the Pixhawk? "
                 f"(sudo systemctl stop venator-c2)")
    print(f"{PASS} Connected (system {m.target_system})\n")

    # --- 1. RC link health ---
    h = rc_healthy(m)
    if h:
        print(f"{PASS} RC receiver healthy — transmitter is ON and bound")
    else:
        print(f"{FAIL} RC receiver NOT healthy — transmitter off, unbound, or in failsafe")
        print(f"{WARN} Turn the transmitter on before continuing.")

    # --- 2. arming rules ---
    wo_gps = read_param(m, "COM_ARM_WO_GPS")
    print(f"{PASS if wo_gps == 1 else WARN} COM_ARM_WO_GPS = {wo_gps}"
          f"  ({'can arm without GPS (manual RC flight OK)' if wo_gps == 1 else 'GPS REQUIRED to arm'})")
    rc_mode = read_param(m, "COM_RC_IN_MODE")
    print(f"{INFO} COM_RC_IN_MODE = {rc_mode}  (0=RC only, 1=joystick, 2=both, 3=keep first, 4=sticks disabled)")
    if rc_mode == 4:
        print(f"{FAIL} Stick input is DISABLED — RC will not fly the drone.")

    # --- 3. channel mapping ---
    print(f"\n{INFO} Channel mapping (0 = not assigned):")
    mapping = {}
    for pname, label in MAPS:
        v = read_param(m, pname)
        mapping[label] = v
        mark = PASS if (v or 0) > 0 else (WARN if label in
                                          ("KILL switch", "arm switch", "return/RTL switch") else FAIL)
        print(f"  {mark} {label:20s} -> channel {v}")
    if not mapping.get("KILL switch"):
        print(f"{WARN} No KILL switch mapped. Strongly recommended: assign one in QGroundControl")
        print(f"{WARN} (Radio setup) so you can cut motors instantly.")

    # --- 4. live sticks ---
    print(f"\n{INFO} Live sticks for {args.seconds}s — move each one and watch it respond.")
    print(f"{INFO} Throttle should read ~1000 at the bottom and ~2000 at the top;")
    print(f"{INFO} roll/pitch/yaw should sit ~1500 centred. Flip the mode switch too.\n")
    seen_min = {}
    seen_max = {}
    end = time.time() + args.seconds
    last_draw = 0.0
    while time.time() < end:
        msg = m.recv_match(type="RC_CHANNELS", blocking=True, timeout=1)
        if msg is None:
            continue
        chans = [getattr(msg, f"chan{i}_raw", 0) for i in range(1, 9)]
        for i, c in enumerate(chans):
            if c:
                seen_min[i] = min(seen_min.get(i, c), c)
                seen_max[i] = max(seen_max.get(i, c), c)
        if time.time() - last_draw < 0.2:
            continue
        last_draw = time.time()
        labels = {}
        for label, ch in mapping.items():
            if ch and 1 <= ch <= 8:
                labels[ch - 1] = label
        lines = []
        for i, c in enumerate(chans):
            name = labels.get(i, "")
            lines.append(f"  ch{i+1} {c:4d} |{bar(c)}| {name}")
        rssi = getattr(msg, "rssi", 255)
        print("\033[2J\033[H" + f"{INFO} RC live (rssi {rssi})   Ctrl-C to stop early\n"
              + "\n".join(lines))

    # --- 5. movement summary ---
    print(f"\n{INFO} Movement summary (channels you actually moved):")
    moved = 0
    for i in sorted(seen_min):
        span = seen_max[i] - seen_min[i]
        if span > 100:
            moved += 1
            print(f"  {PASS} ch{i+1}: {seen_min[i]}..{seen_max[i]}  (span {span})")
        else:
            print(f"  {INFO} ch{i+1}: {seen_min[i]}..{seen_max[i]}  (barely moved)")
    if moved >= 4:
        print(f"\n{PASS} At least 4 channels moved — sticks are reaching the flight controller.")
    else:
        print(f"\n{WARN} Fewer than 4 channels moved. Move all sticks + switches and re-run,")
        print(f"{WARN} or calibrate the radio in QGroundControl.")
    m.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
