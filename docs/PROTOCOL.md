# Venator C2 protocol (v1)

The command-and-control protocol between the **laptop client** (`gcs_client.py`)
and the **Jetson server** (Phase 4). Defined once in [`c2_protocol.py`](../c2_protocol.py)
and imported by both, so they can never drift.

## Wire format

- One JSON object per line, newline-delimited (`\n`).
- Kept small so each message fits one LoRa packet (< ~200 bytes).
- Every message has `t` (type) and `seq` (sequence number).

Example:

```
{"t":"HELLO","seq":1}
{"t":"HELLO_ACK","seq":1,"proto":1,"armed":false,"gps":"no fix","batt":11.4}
```

## Message types

| Type | Direction | Fields | Meaning |
|---|---|---|---|
| `HELLO` | client → jetson | — | link request (handshake) |
| `HELLO_ACK` | jetson → client | `proto,armed,gps,batt` | link up + status |
| `GET_MENU` | client → jetson | — | request the mission list |
| `MENU` | jetson → client | `item,last` | one mission `{id,name,needs,gps}`; streamed one per packet, `last:true` on the final |
| `RUN` | client → jetson | `id` | run a mission |
| `ACK` | jetson → client | `accepted,reason` / `i` / `uploaded` | accepted/rejected reply |
| `DONE` | jetson → client | `id,result` | mission finished |
| `WP_BEGIN` | client → jetson | `count` | start a waypoint upload |
| `WP` | client → jetson | `i,lat,lon,alt` | one waypoint |
| `WP_END` | client → jetson | — | end of upload |
| `PING`/`PONG` | either | — | liveness |
| `ABORT` | client → jetson | — | stop current mission |
| `LOG` | jetson → client | `text` | line to display |
| `STATUS` | jetson → client | `armed,gps,batt` | lite status |

## Flows

**Handshake / "connection established":** client sends `HELLO`, waits for
`HELLO_ACK`. That reply is the moment the menu is requested.

**Menu:** client sends `GET_MENU`; the server streams one `MENU` message per
mission (each fits a LoRa packet), the last flagged `last:true`. A full menu in a
single message would exceed the ~255 B LoRa / 240 B firmware limit.

**Run a mission:** `RUN{id}` → `ACK{accepted}`; if accepted, a later `DONE`.
Flight missions are rejected (`accepted:false, reason`) until PX4's own checks
pass (e.g. a GPS fix).

**Waypoint upload:** `WP_BEGIN{count}` → `WP{i,lat,lon,alt}` × N (each ACKed) →
`WP_END` → `ACK{uploaded,rejected}`. The Jetson validates coordinates, builds a
PX4 mission, and (Phase 5, with GPS) flies it.

## Reliability

LoRa drops packets, so callers use **stop-and-wait**: send one message, wait for
its ACK, retransmit on timeout, and dedupe by `seq`. Half-duplex firmware means
one side talks at a time — the request/response shape keeps that collision-free.

## Firmware note (Phase 3)

The bidirectional Heltec firmware should be a **transparent line bridge**: send
each serial line over the air verbatim and print each received payload to serial
verbatim. Then `SerialTransport` in the client speaks this protocol directly with
no wrapping. (The original one-way firmware wrapped lines as `{"msg":"..."}`;
the transparent bridge drops that wrapper.)
