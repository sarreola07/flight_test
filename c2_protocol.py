"""
Venator C2 protocol — shared by the laptop client (GCS) and the Jetson server.

Wire format: one JSON object per line (newline-delimited). Messages are kept
small so each fits in a single LoRa packet (well under ~200 bytes). Every
message has a type "t" and a sequence number "seq"; reliability (ACK +
retransmit) is handled by the callers, not here.

Stdlib only, so this imports unchanged on the Jetson and on Windows/Mac.
"""
import json

PROTO_VERSION = 1

# --- message types -----------------------------------------------------------
HELLO     = "HELLO"      # client -> jetson : link request (handshake)
HELLO_ACK = "HELLO_ACK"  # jetson -> client : link up + drone status
GET_MENU  = "GET_MENU"   # client -> jetson : request the mission list
MENU      = "MENU"       # jetson -> client : the mission list
RUN       = "RUN"        # client -> jetson : run mission {id}
ACK       = "ACK"        # jetson -> client : accepted / rejected (+reason)
DONE      = "DONE"       # jetson -> client : mission finished (+result)
WP_BEGIN  = "WP_BEGIN"   # client -> jetson : start a waypoint upload {count}
WP        = "WP"         # client -> jetson : one waypoint {i,lat,lon,alt}
WP_END    = "WP_END"     # client -> jetson : end of waypoint upload
PING      = "PING"       # either way : liveness check
PONG      = "PONG"       # either way : liveness reply
ABORT     = "ABORT"      # client -> jetson : stop the current mission
LOG       = "LOG"        # jetson -> client : a text line to show
STATUS    = "STATUS"     # jetson -> client : lite status (armed, gps, batt)


def encode(m: dict) -> str:
    """dict -> one compact JSON line (newline-terminated)."""
    return json.dumps(m, separators=(",", ":")) + "\n"


def decode(line: str):
    """One line -> dict, or None if it is not a protocol message we understand."""
    if line is None:
        return None
    line = line.strip()
    if not line:
        return None
    try:
        m = json.loads(line)
    except (json.JSONDecodeError, ValueError):
        return None
    if isinstance(m, dict) and "t" in m:
        return m
    return None


def message(t, seq=0, **fields):
    """Build a protocol message dict."""
    m = {"t": t, "seq": seq}
    m.update(fields)
    return m
