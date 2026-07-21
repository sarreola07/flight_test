#!/usr/bin/env python3
"""
Bench-test missions for the Pixhawk 2.4.8 (PX4 v1.13.3) hexarotor.

Missions:
  1) Spin each motor one at a time for 3 s (props MUST be off)
  2) Spin all motors together for 3 s   (props MUST be off)
  3) Arm, take off to 3 ft, land, disarm (props on — REAL FLIGHT)
  4) RC manual flight: Stabilized mode, arm, you fly with the
     transmitter while this script monitors (props on — REAL FLIGHT)
  5) Camera tracking display: show the OAK-D person X/Y/Z (no flight)
  6) LoRa remote: run missions from LoRa commands (respects props gate)

Usage:
    ./venv/bin/python missions.py
    ./venv/bin/python missions.py --device /dev/ttyACM0 --motors 6
"""

import argparse
import json
import socket
import sys
import time

from lora_helper import listen as listen_lora

try:
    from pymavlink import mavutil
except ImportError:
    sys.exit("pymavlink is not installed. Run: ./venv/bin/python missions.py (see README)")

PASS = "\033[92m[ OK ]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
INFO = "\033[94m[INFO]\033[0m"
WARN = "\033[93m[WARN]\033[0m"

TAKEOFF_ALT_M = 0.91          # 3 feet
MOTOR_TEST_THROTTLE = 15      # percent — low, just enough to spin
MOTOR_TEST_SECONDS = 3

# Camera publisher (camera_publisher.py) broadcasts person coordinates here
CAMERA_UDP_IP = "127.0.0.1"
CAMERA_UDP_PORT = 5005

# LoRa RX module (CP210x USB-serial)
LORA_PORT = "/dev/ttyUSB0"
LORA_BAUD = 115200


def connect(device, baud, timeout=15):
    print(f"{INFO} Connecting to {device} @ {baud} ...")
    master = mavutil.mavlink_connection(device, baud=baud)
    # PX4 v1.13 keeps USB silent until it hears MAVLink from us
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    if master.wait_heartbeat(timeout=timeout) is None:
        sys.exit(f"{FAIL} No heartbeat from the flight controller.")
    print(f"{PASS} Connected (system {master.target_system})")
    return master


def drain_statustext(master, seconds=0.5):
    """Print any STATUSTEXT messages (arming denials, safety warnings...)."""
    end = time.time() + seconds
    while time.time() < end:
        msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.2)
        if msg:
            print(f"{INFO}   FC says: {msg.text}")


def send_command(master, command, *params, name=""):
    """Send a COMMAND_LONG and wait for its ACK. Returns True on MAV_RESULT_ACCEPTED.

    STATUSTEXT messages arriving alongside the ACK carry the FC's reason for a
    rejection (e.g. "Arming denied! ..."), so collect and print them too.
    """
    p = list(params) + [0] * (7 - len(params))
    master.mav.command_long_send(
        master.target_system, master.target_component, command, 0, *p)
    ack = None
    end = time.time() + 3
    while time.time() < end:
        msg = master.recv_match(type=["COMMAND_ACK", "STATUSTEXT"],
                                blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            print(f"{INFO}   FC says: {msg.text}")
        elif msg.command == command:
            ack = msg
            break
    if ack is None:
        print(f"{FAIL} {name}: no acknowledgement")
        return False
    if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        result = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name
        print(f"{FAIL} {name}: rejected ({result})")
        drain_statustext(master, 3.0)
        return False
    return True


def is_armed(master):
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    return bool(hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def motor_test(master, motor, seconds, throttle):
    """Spin one motor (1-based index) via MAV_CMD_DO_MOTOR_TEST."""
    return send_command(
        master, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        motor,                                            # param1: motor instance (1-based)
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,      # param2: throttle type
        throttle,                                         # param3: throttle value
        seconds,                                          # param4: timeout
        0,                                                # param5: motor count (0 = just this one)
        mavutil.mavlink.MOTOR_TEST_ORDER_BOARD,           # param6: test order
        name=f"motor {motor}")


def mission_1(master, motors):
    print(f"\n{INFO} Mission 1: each motor for {MOTOR_TEST_SECONDS}s at "
          f"{MOTOR_TEST_THROTTLE}% throttle, one after another.")
    for m in range(1, motors + 1):
        print(f"{INFO} Motor {m}/{motors} ...")
        if not motor_test(master, m, MOTOR_TEST_SECONDS, MOTOR_TEST_THROTTLE):
            print(f"{WARN} Stopping. (Is the safety switch pressed? Battery connected?)")
            return
        time.sleep(MOTOR_TEST_SECONDS + 1)
    print(f"{PASS} Mission 1 complete — all {motors} motors tested.")


def mission_2(master, motors):
    print(f"\n{INFO} Mission 2: all {motors} motors together for "
          f"{MOTOR_TEST_SECONDS}s at {MOTOR_TEST_THROTTLE}% throttle.")
    ok = True
    for m in range(1, motors + 1):
        ok &= motor_test(master, m, MOTOR_TEST_SECONDS, MOTOR_TEST_THROTTLE)
    if not ok:
        print(f"{WARN} Some motors were rejected. (Safety switch? Battery?)")
        return
    time.sleep(MOTOR_TEST_SECONDS + 1)
    print(f"{PASS} Mission 2 complete.")


def wait_altitude(master, target_m, timeout=30):
    """Watch relative altitude until target reached or timeout. Returns reached alt."""
    best = 0.0
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"],
                                blocking=True, timeout=1)
        if msg is None:
            continue
        alt = (msg.relative_alt / 1000.0 if msg.get_type() == "GLOBAL_POSITION_INT"
               else -msg.z)
        best = max(best, alt)
        print(f"\r{INFO} Altitude: {alt:5.2f} m", end="", flush=True)
        if alt >= target_m * 0.9:
            print()
            return best
    print()
    return best


def mission_3(master, require_confirm=True):
    print(f"\n{INFO} Mission 3: arm → take off to {TAKEOFF_ALT_M:.2f} m (3 ft) → land → disarm.")
    print(f"{WARN} THIS FLIES THE DRONE. Clear the area. Be ready to cut power.")
    if require_confirm:
        if input("Type FLY to continue, anything else to abort: ").strip().lower() != "fly":
            print(f"{INFO} Aborted.")
            return
    else:
        print(f"{WARN} Triggered remotely by LoRa — no local confirmation. Arming in 3 s.")
        time.sleep(3)

    # AUTO.TAKEOFF needs a position estimate (GPS/optical flow); check before arming
    print(f"{INFO} Checking position estimate ...")
    if master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=3) is None:
        print(f"{FAIL} No position estimate — PX4 cannot do an automatic takeoff.")
        print(f"{WARN} This vehicle has no GPS/optical flow. Fit a GPS module and go")
        print(f"{WARN} outdoors, or fly manually with the RC transmitter instead.")
        return

    # Takeoff altitude used by PX4's AUTO.TAKEOFF mode
    master.mav.param_set_send(
        master.target_system, master.target_component,
        b"MIS_TAKEOFF_ALT", TAKEOFF_ALT_M, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    master.recv_match(type="PARAM_VALUE", blocking=True, timeout=3)

    print(f"{INFO} Arming ...")
    if not send_command(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1,
                        name="arm"):
        print(f"{WARN} PX4 refused to arm — its preflight checks decide, not us.")
        print(f"{WARN} The 'FC says' line above is the exact reason. Common causes:")
        print(f"{WARN} no RC/joystick input, no position estimate, sensors not calibrated.")
        return
    print(f"{PASS} Armed.")

    print(f"{INFO} Switching to AUTO.TAKEOFF ...")
    master.set_mode_px4(*mavutil.px4_map["TAKEOFF"])
    reached = wait_altitude(master, TAKEOFF_ALT_M, timeout=30)
    if reached < TAKEOFF_ALT_M * 0.5:
        print(f"{WARN} Takeoff did not progress (reached {reached:.2f} m). Landing now.")
    else:
        print(f"{PASS} Reached {reached:.2f} m. Hovering 3 s ...")
        time.sleep(3)

    print(f"{INFO} Switching to AUTO.LAND ...")
    master.set_mode_px4(*mavutil.px4_map["LAND"])

    print(f"{INFO} Waiting for landing + auto-disarm (up to 60 s) ...")
    end = time.time() + 60
    while time.time() < end:
        if not is_armed(master):
            print(f"{PASS} Landed and disarmed. Mission 3 complete.")
            return
    print(f"{WARN} Still armed after 60 s — sending disarm.")
    send_command(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, name="disarm")


def rc_link_ok(master):
    """True when the RC receiver is enabled and healthy (transmitter bound, no failsafe)."""
    s = master.recv_match(type="SYS_STATUS", blocking=True, timeout=5)
    if s is None:
        return False
    bit = mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER
    return bool(s.onboard_control_sensors_enabled & bit
                and s.onboard_control_sensors_health & bit)


def mission_4(master):
    print(f"\n{INFO} Mission 4: manual RC flight in Stabilized mode.")
    print(f"{INFO} You fly with the transmitter; this script arms, then only watches.")
    print(f"{WARN} THIS FLIES THE DRONE. Clear the area. Be ready to cut power.")

    print(f"{INFO} Checking RC link ...")
    if not rc_link_ok(master):
        print(f"{FAIL} RC link is down (transmitter off, unbound, or in failsafe).")
        print(f"{WARN} Turn the transmitter on and re-run.")
        return
    print(f"{PASS} RC link is up.")

    if input("Type FLY to continue, anything else to abort: ").strip().lower() != "fly":
        print(f"{INFO} Aborted.")
        return

    print(f"{INFO} Switching to STABILIZED ...")
    master.set_mode_px4(*mavutil.px4_map["STABILIZED"])
    time.sleep(1)

    print(f"{INFO} Arming (keep throttle LOW) ...")
    if not send_command(master, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1,
                        name="arm"):
        print(f"{WARN} PX4 refused to arm — see the 'FC says' line above.")
        return
    print(f"{PASS} Armed. Motors at idle — you have the controls.")
    print(f"{INFO} Monitoring until you land and disarm (throttle low + yaw left,")
    print(f"{INFO} or let PX4 auto-disarm after landing). Ctrl-C stops ONLY the")
    print(f"{INFO} monitor — it will NOT disarm the drone.\n")

    try:
        last_beat = 0.0
        while True:
            now = time.time()
            if now - last_beat > 1.0:
                master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                          mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_beat = now
            msg = master.recv_match(type=["HEARTBEAT", "VFR_HUD", "SYS_STATUS", "STATUSTEXT"],
                                    blocking=True, timeout=2)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "STATUSTEXT":
                print(f"\n{INFO} FC says: {msg.text}")
            elif t == "HEARTBEAT":
                if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                    print(f"\n{PASS} Disarmed — mission 4 complete.")
                    return
            elif t == "VFR_HUD":
                print(f"\r{INFO} alt {msg.alt:7.1f} m  climb {msg.climb:+5.1f} m/s  "
                      f"throttle {msg.throttle:3d}%   ", end="", flush=True)
    except KeyboardInterrupt:
        print(f"\n{WARN} Monitor stopped. Drone may still be ARMED — you have the RC.")
        print(f"{WARN} Land and disarm with the transmitter (throttle low + yaw left).")


def listen_camera_coordinates(seconds=None):
    """Show live person X/Y/Z from the OAK-D camera publisher (UDP). No flight.

    seconds=None runs until Ctrl-C (menu option 5); a number bounds the run
    (used by the LoRa dispatcher so it doesn't block the listener forever).
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((CAMERA_UDP_IP, CAMERA_UDP_PORT))
    except OSError as exc:
        print(f"{FAIL} Cannot bind UDP {CAMERA_UDP_IP}:{CAMERA_UDP_PORT}: {exc}")
        print(f"{WARN} Another camera listener may already be running.")
        return
    sock.settimeout(1.0)
    print(f"{INFO} Listening for OAK-D coordinates on {CAMERA_UDP_IP}:{CAMERA_UDP_PORT}")
    print(f"{INFO} Stand in view of the camera. Ctrl-C to stop." if seconds is None
          else f"{INFO} Showing camera data for {seconds}s ...")

    end = None if seconds is None else time.time() + seconds
    last_z = None
    warned_idle = False
    try:
        while end is None or time.time() < end:
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                if not warned_idle:
                    print(f"{WARN} No detections yet — is oak-camera running, and is "
                          f"someone in frame?")
                    warned_idle = True
                continue
            try:
                c = json.loads(data.decode())
                x, y, z = float(c["x"]), float(c["y"]), float(c["z"])
            except (ValueError, KeyError, json.JSONDecodeError):
                continue
            warned_idle = False
            trend = ""
            if last_z is not None:
                if z < last_z - 0.05:
                    trend = "  (approaching)"
                elif z > last_z + 0.05:
                    trend = "  (moving away)"
            last_z = z
            print(f"{PASS} Person X:{x:+.2f}m  Y:{y:+.2f}m  Z:{z:.2f}m{trend}")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    print(f"{INFO} Camera display finished.")


def normalize_lora_msg(msg):
    """LoRa senders sometimes deliver '1.0' for 1; coerce to the bare digit."""
    text = str(msg).strip()
    if text.endswith(".0") and text[:-2].isdigit():
        text = text[:-2]
    return text


def handle_lora_msg(master, msg, props_off, motors):
    """Dispatch one LoRa command under the same props interlock as the menu."""
    msg = normalize_lora_msg(msg)
    # command -> (label, handler, needs_props_off, needs_props_on)
    table = {
        "1": ("sequential motor test", lambda: mission_1(master, motors), True, False),
        "2": ("all-motor test", lambda: mission_2(master, motors), True, False),
        "3": ("camera display (30 s)", lambda: listen_camera_coordinates(seconds=30), False, False),
        "4": ("flight: arm/takeoff/land", lambda: mission_3(master, require_confirm=False), False, True),
    }
    if msg not in table:
        print(f"{WARN} LoRa: unknown command {msg!r} — ignored")
        return
    label, handler, need_off, need_on = table[msg]
    if need_off and not props_off:
        print(f"{FAIL} LoRa {msg} ({label}) blocked: props are ON — motor tests need props OFF.")
        return
    if need_on and props_off:
        print(f"{FAIL} LoRa {msg} ({label}) blocked: props are OFF — flight needs props ON.")
        return
    print(f"{INFO} LoRa {msg} → {label}")
    try:
        handler()
        print(f"{PASS} LoRa {msg} finished")
    except Exception as exc:
        print(f"{FAIL} LoRa {msg} failed: {exc}")


def run_lora_listener(master, props_off, motors):
    """Menu option 6: turn LoRa packets into mission commands."""
    print(f"\n{INFO} LoRa remote listener on {LORA_PORT} @ {LORA_BAUD}.")
    print(f"{INFO} Command map: 1=seq motors, 2=all motors, 3=camera, 4=flight.")
    if props_off:
        print(f"{PASS} Props OFF — LoRa may run 1, 2, 3. Flight (4) is blocked.")
    else:
        print(f"{WARN} Props ON — LoRa may run 3, 4. Motor tests (1, 2) are blocked.")
        print(f"{WARN} LoRa command 4 will FLY the drone with no local confirmation.")
    listen_lora(port=LORA_PORT, baud=LORA_BAUD,
                on_msg=lambda m: handle_lora_msg(master, m, props_off, motors))


def ask_props_state():
    """Ask whether the props are removed. Returns True if props are OFF."""
    while True:
        ans = input("\nAre the propellers REMOVED from the motors? (yes/no): ").strip().lower()
        if ans in ("yes", "y"):
            return True
        if ans in ("no", "n"):
            return False


def run_selected(choice, props_off, args):
    """Run one mission. Option 5 needs no flight controller; the rest connect."""
    if choice == "5":
        listen_camera_coordinates()
        return

    master = connect(args.device, args.baud)
    try:
        if choice == "1":
            mission_1(master, args.motors)
        elif choice == "2":
            mission_2(master, args.motors)
        elif choice == "3":
            mission_3(master)
        elif choice == "4":
            mission_4(master)
        elif choice == "6":
            run_lora_listener(master, props_off, args.motors)
    except KeyboardInterrupt:
        print(f"\n{WARN} Interrupted — sending disarm just in case.")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    finally:
        master.close()


def main():
    parser = argparse.ArgumentParser(description="Pixhawk bench-test missions")
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--motors", type=int, default=6, help="motor count (hexarotor = 6)")
    args = parser.parse_args()

    print("=" * 52)
    print(" Pixhawk missions — PX4 v1.13.3 hexarotor bench test")
    print("=" * 52)

    props_off = ask_props_state()

    # Menu loops until the user quits with 'q'. Options 5 (camera) and 6 (LoRa)
    # are always available; the LoRa dispatcher enforces the props interlock too.
    while True:
        if props_off:
            print(f"\n{PASS} Props OFF — motor tests (1, 2) available; flight (3, 4) locked out.")
            allowed = {"1", "2", "5", "6"}
        else:
            print(f"\n{WARN} Props ON — flight (3, 4) available; motor tests (1, 2) locked out.")
            allowed = {"3", "4", "5", "6"}

        print("\n  1) Test each motor for 3 s, one at a time")
        print("  2) Test all motors at the same time")
        print("  3) Arm, take off 3 ft, land, shut off (automatic — needs GPS)")
        print("  4) Manual RC flight in Stabilized mode (script arms + monitors)")
        print("  5) Camera tracking display — show OAK-D person X/Y/Z (no flight)")
        print("  6) LoRa remote — run missions from LoRa commands")
        print("  p) Change props state (re-declare)")
        print("  q) Quit")

        choice = input("\nSelect mission (or q to quit): ").strip().lower()

        if choice == "q":
            print("Bye.")
            return
        if choice == "p":
            props_off = ask_props_state()
            continue
        if choice not in ("1", "2", "3", "4", "5", "6"):
            print(f"{WARN} Enter 1–6, p, or q.")
            continue
        if choice not in allowed:
            need = "ON" if props_off else "OFF"
            print(f"{FAIL} Mission {choice} needs props {need}. "
                  f"Press 'p' to re-declare, or change the props first.")
            continue

        run_selected(choice, props_off, args)
        print(f"\n{INFO} Mission finished — returning to menu (press q to quit).")


if __name__ == "__main__":
    main()
