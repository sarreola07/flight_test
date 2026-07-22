# LoRa firmware (Heltec WiFi LoRa 32 V3)

## What this is

`LoRa_Transceiver/LoRa_Transceiver.ino` is a **half-duplex transparent bridge**:
a line sent to USB serial goes out over 915 MHz LoRa verbatim, and any packet
received over the air is printed to USB serial verbatim. This lets the Venator
C2 protocol (newline-delimited JSON, see [../docs/PROTOCOL.md](../docs/PROTOCOL.md))
flow in **both directions**.

It replaces the original one-way sketches (`LoRa_TX.ino` + `LoRa_RX.ino`), which
could only send *or* receive — not enough for the "Jetson sends the menu, laptop
sends the choice" handshake.

## Flash it on BOTH sticks

Both the Jetson-side stick and the laptop-side stick run the **same** sketch.

1. Arduino IDE → install the **Heltec ESP32** board package and the **Heltec
   ESP32 LoRaWan** library (same as your originals — they already compile).
2. Board: **Heltec WiFi LoRa 32(V3)**.
3. Open `LoRa_Transceiver/LoRa_Transceiver.ino`, select the stick's port, Upload.
4. Repeat for the second stick.

Radio parameters (must be identical on both, and they are): 915 MHz, SF7,
syncword `0x12`, 14 dBm.

## Test the two-way link (before touching the drone)

1. Plug both sticks into one computer (or two).
2. Open a serial monitor on each at **115200 baud**, line ending = **Newline**.
3. Type a line into stick A's monitor and press Enter → it appears on stick B.
4. Type a line into stick B → it appears on stick A.

If both directions work, the link is ready for the GCS client and the Jetson
C2 server. A quick protocol sanity check: type

```
{"t":"HELLO","seq":1}
```

into one monitor; the other should print that exact line.

## Why a transparent bridge (not the old `{"msg":...}` wrapper)

The original TX sketch wrapped each line as `{"msg":"..."}`. The transceiver
sends lines through untouched so the JSON protocol arrives exactly as sent —
the client and Jetson server do all the parsing. Keep the radio a dumb pipe;
the intelligence lives on the computers at each end.
