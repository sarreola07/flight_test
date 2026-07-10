#!/usr/bin/env python3
"""
Bench-test missions for the Pixhawk 2.4.8 (PX4 v1.13.3) hexarotor.

Missions:
  1) Spin each motor one at a time for 3 s (props MUST be off)
  2) Spin all motors together for 3 s   (props MUST be off)
  3) Arm, take off to 3 ft, land, disarm (props on — REAL FLIGHT)

Usage:
    ./venv/bin/python missions.py
    ./venv/bin/python missions.py --device /dev/ttyACM0 --motors 6
"""

import argparse
import sys
import time

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


def mission_3(master):
    print(f"\n{INFO} Mission 3: arm → take off to {TAKEOFF_ALT_M:.2f} m (3 ft) → land → disarm.")
    print(f"{WARN} THIS FLIES THE DRONE. Clear the area. Be ready to cut power.")
    if input("Type FLY to continue, anything else to abort: ").strip().lower() != "fly":
        print(f"{INFO} Aborted.")
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


def main():
    parser = argparse.ArgumentParser(description="Pixhawk bench-test missions")
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--motors", type=int, default=6, help="motor count (hexarotor = 6)")
    args = parser.parse_args()

    print("=" * 52)
    print(" Pixhawk missions — PX4 v1.13.3 hexarotor bench test")
    print("=" * 52)

    props_off = None
    while props_off is None:
        ans = input("\nAre the propellers REMOVED from the motors? (yes/no): ").strip().lower()
        if ans in ("yes", "y"):
            props_off = True
        elif ans in ("no", "n"):
            props_off = False

    if props_off:
        print(f"\n{PASS} Props off — motor tests available. Flight (3) is locked out.")
        allowed = {"1", "2"}
    else:
        print(f"\n{WARN} Props ON — motor tests are locked out for safety.")
        print(f"{WARN} Only mission 3 (flight) is available.")
        allowed = {"3"}

    print("\n  1) Test each motor for 3 s, one at a time")
    print("  2) Test all motors at the same time")
    print("  3) Arm, take off 3 ft, land, shut off")
    print("  q) Quit")

    while True:
        choice = input("\nSelect mission: ").strip().lower()
        if choice == "q":
            print("Bye.")
            return
        if choice in ("1", "2", "3"):
            break
        print("Enter 1, 2, 3 or q.")

    if choice not in allowed:
        if props_off:
            sys.exit(f"{FAIL} Mission 3 needs props ON. Re-run after fitting props.")
        sys.exit(f"{FAIL} Motor tests need props OFF. Remove them and re-run.")

    master = connect(args.device, args.baud)
    try:
        if choice == "1":
            mission_1(master, args.motors)
        elif choice == "2":
            mission_2(master, args.motors)
        else:
            mission_3(master)
    except KeyboardInterrupt:
        print(f"\n{WARN} Interrupted — sending disarm just in case.")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    finally:
        master.close()


if __name__ == "__main__":
    main()
