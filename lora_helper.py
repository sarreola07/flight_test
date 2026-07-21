"""
LoRa serial listener — receives JSON commands from the RX module.

Connection: LoRa RX module -> Jetson USB, /dev/ttyUSB0 @ 115200 baud

Expected payload (one JSON object per line):
  {"msg": "1"}  sequential motor test
  {"msg": "2"}  simultaneous motor test
  {"msg": "3"}  camera tracking display
  {"msg": "4"}  flight (arm / takeoff / land)
"""

import json
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required for LoRa. Run: pip install pyserial", flush=True)
    sys.exit(1)

# ----------------------------------------------------------------------
# TUNABLE PARAMETERS
# ----------------------------------------------------------------------
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
# ----------------------------------------------------------------------


def listen(port=DEFAULT_PORT, baud=DEFAULT_BAUD, on_msg=None):
    """
    Open the LoRa serial port, parse JSON lines, and call on_msg with the
    value of the "msg" field. Blocks until KeyboardInterrupt.
    """
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as exc:
        print(f"Cannot open LoRa port {port} @ {baud}: {exc}", flush=True)
        sys.exit(1)

    print(f"Listening on {port} @ {baud} baud  (Ctrl+C to stop)", flush=True)
    print('Expecting JSON like {"msg": "1"}', flush=True)
    print("Ready — waiting for LoRa packets...", flush=True)

    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue

            ts = time.strftime("%H:%M:%S")

            try:
                payload = json.loads(text)
            except json.JSONDecodeError:
                print(f"[{ts}] ignored (invalid JSON): {text}", flush=True)
                continue

            if not isinstance(payload, dict) or "msg" not in payload:
                print(f"[{ts}] ignored (missing msg field): {text}", flush=True)
                continue

            msg = str(payload["msg"]).strip()
            print(f"[{ts}] msg={msg!r}", flush=True)

            if on_msg is not None:
                try:
                    on_msg(msg)
                except Exception as exc:
                    print(f"[{ts}] command error: {exc}", flush=True)
    except KeyboardInterrupt:
        print("\nLoRa listener stopped.", flush=True)
    finally:
        ser.close()
