/*
 * Venator LoRa transceiver — half-duplex transparent bridge.
 *
 * Flash this SAME sketch on BOTH Heltec WiFi LoRa 32 V3 sticks (the one on the
 * Jetson and the one on the laptop). It replaces the old one-way LoRa_TX.ino
 * and LoRa_RX.ino so the link works in BOTH directions on each stick.
 *
 * Behaviour — a transparent line bridge:
 *   - A line typed/sent to USB serial (ending in '\n') is transmitted over
 *     915 MHz LoRa verbatim.
 *   - Any packet received over the air is printed to USB serial verbatim,
 *     followed by a newline.
 *
 * That lets the Venator C2 protocol (newline-delimited JSON) pass straight
 * through in both directions. Radio parameters are identical to the original
 * working sketches (915 MHz, SF7, syncword 0x12), so range/behaviour are
 * unchanged — only the direction handling is new.
 *
 * NOTE: this compiles against the Heltec ESP32 LoRaWan library, same as your
 * originals. It has not been compiled here (no ESP32 toolchain on the Jetson) —
 * flash it and run the two-terminal test in firmware/README.md.
 */
#include "LoRaWan_APP.h"

#define RF_FREQUENCY     915000000   // Hz  (must match on both sticks)
#define TX_OUTPUT_POWER  14          // dBm
#define LORA_SF          7           // spreading factor
#define LORA_SYNCWORD    0x12
#define MAX_LINE         240         // keep under the LoRa payload limit (255)

static RadioEvents_t RadioEvents;

static volatile bool txBusy = false;     // true while a packet is transmitting
static char   line[MAX_LINE + 1];        // USB-serial line being assembled
static size_t lineLen  = 0;
static bool   lineReady = false;         // a full line is waiting to send

static void startRx() {
    Radio.Rx(0);                         // continuous receive
}

static void sendLine(const char *buf, size_t len) {
    txBusy = true;
    Radio.Send((uint8_t *)buf, len);     // leaves Rx; onTxDone returns to Rx
}

// ---- radio callbacks --------------------------------------------------------
void onTxDone(void)    { txBusy = false; startRx(); }
void onTxTimeout(void) { txBusy = false; startRx(); }
void onRxTimeout(void) { startRx(); }
void onRxError(void)   { startRx(); }

void onRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    for (uint16_t i = 0; i < size; i++) {
        Serial.write(payload[i]);        // print received bytes verbatim
    }
    Serial.write('\n');
    startRx();                           // re-arm the receiver
}

// ---- setup / loop -----------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    RadioEvents.TxDone    = onTxDone;
    RadioEvents.TxTimeout = onTxTimeout;
    RadioEvents.RxDone    = onRxDone;
    RadioEvents.RxTimeout = onRxTimeout;
    RadioEvents.RxError   = onRxError;
    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);
    // Identical TX + RX config to the original working sketches.
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, 0, LORA_SF, 1, 8,
                      false, true, 0, 0, false, 3000);
    Radio.SetRxConfig(MODEM_LORA, 0, LORA_SF, 1, 0, 8, 0,
                      false, 0, true, 0, 0, false, true);
    Radio.SetSyncWord(LORA_SYNCWORD);

    startRx();                           // default state: listening
    Serial.println("Venator LoRa transceiver ready (half-duplex bridge).");
}

void loop() {
    Radio.IrqProcess();

    // Assemble one line from USB serial without blocking. We pause reading once
    // a full line is ready so nothing is dropped while a previous send is in
    // flight (the protocol is stop-and-wait, so this rarely stalls).
    while (Serial.available() && !lineReady) {
        char c = (char)Serial.read();
        if (c == '\n') {
            if (lineLen > 0) { line[lineLen] = '\0'; lineReady = true; }
        } else if (c != '\r' && lineLen < MAX_LINE) {
            line[lineLen++] = c;
        }
    }

    // Transmit the assembled line once the radio is idle.
    if (lineReady && !txBusy) {
        sendLine(line, lineLen);
        lineLen = 0;
        lineReady = false;
    }
}
