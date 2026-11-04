/*
 * =============================================================================
 *  Transmitter.ino
 *  DCS Project - ESP32 + SX1278 (RA-02) 433 MHz LoRa control link
 *
 *  ROLE: Ground controller. Reads the joystick (aileron / elevator), the rudder
 *        potentiometer and the joystick push-button, sends command packets over
 *        LoRa, and (optionally) listens for receiver acknowledgements.
 * -----------------------------------------------------------------------------
 *  EDUCATIONAL, GROUND-BASED PROTOTYPE ONLY. This is NOT a flight-certified or
 *  safety-rated controller and must not be used to control a real aircraft.
 * -----------------------------------------------------------------------------
 *  Board     : ESP32 Dev Module (ESP32 DevKit V1 / ESP32-WROOM-32)
 *  Libraries : LoRa (Sandeep Mistry), Adafruit SSD1306, Adafruit GFX
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =============================================================================
//  PIN MAP  (matches the project wiring tables - do not change unless rewired)
// =============================================================================
// SX1278 RA-02 (VSPI bus)
static const int PIN_LORA_SCK   = 18;
static const int PIN_LORA_MISO  = 19;
static const int PIN_LORA_MOSI  = 23;
static const int PIN_LORA_NSS   = 5;
static const int PIN_LORA_RESET = 14;
static const int PIN_LORA_DIO0  = 26;  // wired as specified; this sketch polls the radio over SPI

// Controls
static const int PIN_JOY_VRX    = 34;  // Aileron  (roll)  - ADC1_CH6, input-only pin
static const int PIN_JOY_VRY    = 35;  // Elevator (pitch) - ADC1_CH7, input-only pin
static const int PIN_JOY_SW     = 33;  // Joystick push-button, active LOW (internal pull-up)
static const int PIN_RUDDER_POT = 32;  // Rudder   (yaw)   - ADC1_CH4, potentiometer wiper

// OLED (I2C)
static const int     PIN_OLED_SDA  = 21;
static const int     PIN_OLED_SCL  = 22;
static const uint8_t OLED_I2C_ADDR = 0x3C;
static const int     OLED_WIDTH    = 128;
static const int     OLED_HEIGHT   = 64;

// =============================================================================
//  SHARED RADIO + PROTOCOL DEFINITIONS
//  *** Keep this block IDENTICAL in Transmitter.ino and Receiver.ino ***
// =============================================================================
static const long     LORA_FREQUENCY_HZ         = 433000000L; // 433 MHz
static const long     LORA_BANDWIDTH_HZ         = 125000L;    // 125 kHz
static const int      LORA_SPREADING_FACTOR     = 7;          // SF7..SF12 (SF6 needs implicit header - not used)
static const int      LORA_CODING_RATE_DENOM    = 5;          // 5..8  ->  coding rate 4/5..4/8
static const int      LORA_SYNC_WORD            = 0x24;       // private sync word (avoid 0x34 = LoRaWAN)
static const long     LORA_PREAMBLE_SYMBOLS     = 8;
static const int      LORA_TX_POWER_DBM         = 10;         // PA_BOOST range 2..20 dBm; keep low on the bench
static const long     LORA_SPI_FREQUENCY_HZ     = 1000000L;   // 1 MHz: robust over breadboard jumpers

static const uint8_t  PROTO_MAGIC               = 0xDC;       // packet identifier
static const uint8_t  PROTO_VERSION             = 0x01;
static const uint8_t  PKT_TYPE_COMMAND          = 0x01;
static const uint8_t  PKT_TYPE_ACK              = 0x02;
static const uint8_t  CMD_PACKET_LEN            = 11;         // bytes (see packet table)
static const uint8_t  ACK_PACKET_LEN            = 12;         // bytes
static const uint8_t  MAX_LORA_PAYLOAD          = 32;         // RX buffer; longer packets are rejected
static const uint8_t  CMD_FLAG_BUTTON_PRESSED   = 0x01;
static const uint8_t  CMD_FLAG_ACK_REQUEST      = 0x02;
static const uint8_t  CMD_FLAGS_DEFINED_MASK    = 0x03;       // all other flag bits must be 0
static const uint8_t  ACK_STATUS_SERVOS_OK      = 0x01;
static const uint8_t  ACK_STATUS_OLED_OK        = 0x02;
static const uint8_t  ACK_STATUS_LINK_RECOVERED = 0x04;       // RX had a LINK LOST since its previous ACK
static const uint8_t  CONTROL_MIN_DEG           = 0;
static const uint8_t  CONTROL_MAX_DEG           = 180;
static const uint8_t  CONTROL_CENTER_DEG        = 90;
static const uint32_t ACK_REPLY_DELAY_MS        = 8;          // RX waits this long before replying with an ACK
// ======================= END OF SHARED DEFINITIONS ===========================

// =============================================================================
//  TRANSMITTER CONFIGURATION
// =============================================================================
static const uint32_t TX_INTERVAL_MS        = 50;    // command period (auto-stretched if airtime is too long)
static const uint32_t TX_MIN_IDLE_GAP_MS    = 5;     // minimum radio-idle gap between packets
static const uint32_t TX_STARTUP_HOLD_MS    = 1000;  // show RADIO READY briefly before first packet
static const bool     ENABLE_ACK            = true;  // false = pure one-way link (recommended for first test)
static const uint8_t  ACK_EVERY_N_PACKETS   = 5;     // request an ACK on every Nth command packet
static const uint32_t ACK_WINDOW_MARGIN_MS  = 30;    // listen time beyond (reply delay + ACK airtime)
static const uint32_t ACK_LINK_LOST_MS      = 1000;  // no valid ACK for this long -> "NO ACK/LINK LOST"
static const uint32_t INPUT_SAMPLE_MS       = 10;    // control sampling period (100 Hz)
static const int      ADC_OVERSAMPLE        = 4;     // analogRead() calls averaged per sample
static const float    FILTER_ALPHA          = 0.35f; // EMA weight of newest sample (1.0 = no filtering)
static const uint32_t BUTTON_DEBOUNCE_MS    = 30;
static const uint32_t OLED_REFRESH_MS       = 200;
static const uint32_t OLED_REFRESH_COST_MS  = 30;    // approx. blocking time of one display() at 400 kHz
static const uint32_t SLOW_WORK_STARVE_MS   = 1000;  // force OLED/Serial work if starved this long
static const uint32_t SERIAL_DEBUG_MS       = 500;
static const uint32_t RADIO_RETRY_MS        = 2000;

// ---------------- Input calibration (edit after reading Serial output) -------
struct AxisCalibration {
  int  rawMin;       // ADC reading at one end of travel (0..4095)
  int  rawCenter;    // ADC reading at rest / centre
  int  rawMax;       // ADC reading at the other end of travel
  int  deadbandRaw;  // +/- ADC counts around rawCenter treated as exact centre
  bool reverse;      // true = invert direction
  int  outMinDeg;    // command at full negative deflection (0..90)
  int  outMaxDeg;    // command at full positive deflection (90..180)
};

// Typical values for a 3.3 V joystick on the ESP32 ADC; calibrate yours.
AxisCalibration calAileron  = { 50, 1880, 4045, 80, false, 0, 180 };
AxisCalibration calElevator = { 50, 1880, 4045, 80, false, 0, 180 };
AxisCalibration calRudder   = { 50, 2048, 4045, 40, false, 0, 180 };

// =============================================================================
//  TYPES AND GLOBAL STATE
// =============================================================================
enum AxisIndex : uint8_t { AXIS_AILERON = 0, AXIS_ELEVATOR = 1, AXIS_RUDDER = 2 };
enum TxRadioState : uint8_t { TXS_IDLE, TXS_SENDING, TXS_ACK_LISTEN };
enum TxLinkStatus : uint8_t { TXL_RADIO_FAIL, TXL_RADIO_READY, TXL_TRANSMITTING, TXL_ACK_RECEIVED, TXL_NO_ACK };

struct ControlState {
  float   filteredRaw[3];
  int     lastRaw[3];
  uint8_t commandDeg[3];
  bool    filterPrimed;
};

struct ButtonState {
  bool     stablePressed;
  bool     lastRawPressed;
  uint32_t lastChangeMs;
};

struct AckPacket {
  uint16_t ackedSeq;
  uint8_t  status;
  int8_t   rssiDbm;       // RSSI of our command as measured by the receiver
  int8_t   snrQuarterDb;  // SNR x4 as measured by the receiver
  uint16_t rxValidCount;  // receiver's accepted-packet counter (mod 65536)
};

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

bool oledOk  = false;
bool radioOk = false;

ControlState controls;
ButtonState  button = { false, false, 0 };

TxRadioState radioState            = TXS_IDLE;
bool         currentPacketWantsAck = false;
uint16_t     nextSeq               = 0;
uint16_t     lastSentSeq           = 0;
uint16_t     awaitingAckSeq        = 0;
uint8_t      packetsSinceAckReq    = 0;

uint32_t txStartedMs = 0, ackListenStartMs = 0, nextTxDueMs = 0;
uint32_t effectiveTxIntervalMs = TX_INTERVAL_MS;
uint32_t cmdAirtimeMs = 0, ackAirtimeMs = 0, ackWindowMs = 0, txWatchdogMs = 0;

// Statistics
uint32_t packetsSent = 0, txBusySkips = 0, txTimeouts = 0;
uint32_t ackRequested = 0, ackReceived = 0, ackMissed = 0, ackStale = 0, badRxPackets = 0;
uint32_t consecutiveAckMisses = 0;
bool     hasTransmitted = false, everAcked = false;
uint32_t firstTxDoneMs = 0, lastAckMs = 0;
AckPacket lastAck = { 0, 0, 0, 0, 0 };
int      lastAckLocalRssi = 0;
float    lastAckLocalSnr  = 0.0f;

uint32_t lastSampleMs = 0, lastOledMs = 0, lastSerialMs = 0;
uint32_t lastRadioRetryMs = 0, lastSlowWorkMs = 0;

// =============================================================================
//  SHARED HELPER FUNCTIONS  (identical in both sketches)
// =============================================================================

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final XOR).
// Self-test: CRC of ASCII "123456789" = 0x29B1.
uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
      else              crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

uint8_t clampDegrees(int value) {
  if (value < (int)CONTROL_MIN_DEG) return CONTROL_MIN_DEG;
  if (value > (int)CONTROL_MAX_DEG) return CONTROL_MAX_DEG;
  return (uint8_t)value;
}

// LoRa time-on-air (Semtech SX1276/77/78 datasheet formula): explicit header,
// payload CRC on, low-data-rate optimisation when symbol time > 16 ms.
float loraAirtimeMs(uint8_t payloadLen) {
  const float symbolMs       = (float)(1UL << LORA_SPREADING_FACTOR) * 1000.0f / (float)LORA_BANDWIDTH_HZ;
  const int   lowDataRateOpt = (symbolMs > 16.0f) ? 1 : 0;
  const int   crcOn          = 1;
  const int   implicitHeader = 0;
  const int   cr             = LORA_CODING_RATE_DENOM - 4;
  const int   numerator      = 8 * (int)payloadLen - 4 * LORA_SPREADING_FACTOR + 28 + 16 * crcOn - 20 * implicitHeader;
  const int   denominator    = 4 * (LORA_SPREADING_FACTOR - 2 * lowDataRateOpt);
  int extraSymbols = 0;
  if (numerator > 0) extraSymbols = ((numerator + denominator - 1) / denominator) * (cr + 4);
  const float preambleMs = ((float)LORA_PREAMBLE_SYMBOLS + 4.25f) * symbolMs;
  return preambleMs + (8.0f + (float)extraSymbols) * symbolMs;
}

// Copies the current received packet out of the radio FIFO.
// Bytes beyond maxLen are drained and discarded. Returns bytes stored.
int readPacketBytes(uint8_t *buf, int maxLen) {
  int n = 0;
  while (LoRa.available()) {
    int b = LoRa.read();
    if (b < 0) break;
    if (n < maxLen) buf[n++] = (uint8_t)b;
  }
  return n;
}

// Configures the SX1278 identically on both ends. Returns false if not found.
bool initLoRaRadio() {
  LoRa.setPins(PIN_LORA_NSS, PIN_LORA_RESET, PIN_LORA_DIO0);
  LoRa.setSPI(SPI);
  LoRa.setSPIFrequency(LORA_SPI_FREQUENCY_HZ);
  if (!LoRa.begin(LORA_FREQUENCY_HZ)) return false;
  LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setCodingRate4(LORA_CODING_RATE_DENOM);
  LoRa.setPreambleLength(LORA_PREAMBLE_SYMBOLS);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER_DBM, PA_OUTPUT_PA_BOOST_PIN);  // RA-02 uses PA_BOOST
  LoRa.enableCrc();                                            // SX1278 hardware payload CRC
  LoRa.idle();
  return true;
}
// ===================== END OF SHARED HELPER FUNCTIONS ========================

// =============================================================================
//  CALIBRATION / INPUT
// =============================================================================
void validateCalibration(const char *name, const AxisCalibration &c) {
  bool ok = true;
  if (c.rawMin < 0 || c.rawMax > 4095)                          ok = false;
  if (!(c.rawMin < c.rawCenter - c.deadbandRaw))                ok = false;
  if (!(c.rawCenter + c.deadbandRaw < c.rawMax))                ok = false;
  if (c.outMinDeg < 0 || c.outMinDeg > (int)CONTROL_CENTER_DEG) ok = false;
  if (c.outMaxDeg > 180 || c.outMaxDeg < (int)CONTROL_CENTER_DEG) ok = false;
  Serial.printf("[CAL] %s: min=%d center=%d max=%d deadband=%d rev=%d out=%d..%d -> %s\n",
                name, c.rawMin, c.rawCenter, c.rawMax, c.deadbandRaw, c.reverse ? 1 : 0,
                c.outMinDeg, c.outMaxDeg, ok ? "OK" : "CHECK VALUES (output is still clamped to 0..180)");
}

float readAdcAveraged(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_OVERSAMPLE; i++) sum += (uint32_t)analogRead(pin);
  return (float)sum / (float)ADC_OVERSAMPLE;
}

// Piecewise mapping: rawMin..centre -> outMin..90, centre..rawMax -> 90..outMax.
// Guarantees that the calibrated centre gives exactly 90 degrees.
uint8_t mapAxisToDegrees(float raw, const AxisCalibration &c) {
  const float lowEdge  = (float)(c.rawCenter - c.deadbandRaw);
  const float highEdge = (float)(c.rawCenter + c.deadbandRaw);
  float n = 0.0f;  // normalised deflection, -1..+1
  if (raw < lowEdge) {
    const float span = lowEdge - (float)c.rawMin;
    n = (span > 1.0f) ? -(lowEdge - raw) / span : -1.0f;
  } else if (raw > highEdge) {
    const float span = (float)c.rawMax - highEdge;
    n = (span > 1.0f) ? (raw - highEdge) / span : 1.0f;
  }
  if (n < -1.0f) n = -1.0f;
  if (n >  1.0f) n =  1.0f;
  if (c.reverse) n = -n;

  const float center = (float)CONTROL_CENTER_DEG;
  float deg = (n < 0.0f) ? center + n * (center - (float)c.outMinDeg)
                         : center + n * ((float)c.outMaxDeg - center);
  long r = lroundf(deg);
  if (r < c.outMinDeg) r = c.outMinDeg;
  if (r > c.outMaxDeg) r = c.outMaxDeg;
  return clampDegrees((int)r);
}

void sampleControls() {
  const int pins[3] = { PIN_JOY_VRX, PIN_JOY_VRY, PIN_RUDDER_POT };
  const AxisCalibration *cals[3] = { &calAileron, &calElevator, &calRudder };
  for (uint8_t i = 0; i < 3; i++) {
    const float raw = readAdcAveraged(pins[i]);
    controls.lastRaw[i] = (int)lroundf(raw);
    if (!controls.filterPrimed) controls.filteredRaw[i] = raw;
    else controls.filteredRaw[i] += FILTER_ALPHA * (raw - controls.filteredRaw[i]);
    controls.commandDeg[i] = mapAxisToDegrees(controls.filteredRaw[i], *cals[i]);
  }
  controls.filterPrimed = true;
}

void updateButton(uint32_t now) {
  const bool rawPressed = (digitalRead(PIN_JOY_SW) == LOW);
  if (rawPressed != button.lastRawPressed) {
    button.lastRawPressed = rawPressed;
    button.lastChangeMs = now;
  } else if (rawPressed != button.stablePressed && (now - button.lastChangeMs) >= BUTTON_DEBOUNCE_MS) {
    button.stablePressed = rawPressed;
    Serial.printf("[BTN] Joystick button %s\n", rawPressed ? "PRESSED" : "RELEASED");
  }
}

// =============================================================================
//  PACKETS
// =============================================================================
void buildCommandPacket(uint8_t *f, uint16_t seq, const uint8_t deg[3], uint8_t flags) {
  f[0] = PROTO_MAGIC;
  f[1] = PROTO_VERSION;
  f[2] = PKT_TYPE_COMMAND;
  f[3] = (uint8_t)(seq & 0xFF);
  f[4] = (uint8_t)(seq >> 8);
  f[5] = clampDegrees(deg[AXIS_AILERON]);
  f[6] = clampDegrees(deg[AXIS_ELEVATOR]);
  f[7] = clampDegrees(deg[AXIS_RUDDER]);
  f[8] = (uint8_t)(flags & CMD_FLAGS_DEFINED_MASK);
  const uint16_t crc = crc16Ccitt(f, CMD_PACKET_LEN - 2);
  f[9]  = (uint8_t)(crc & 0xFF);
  f[10] = (uint8_t)(crc >> 8);
}

// Returns nullptr if valid, otherwise a short reason string.
const char *decodeAckPacket(const uint8_t *b, int storedLen, int reportedLen, AckPacket &out) {
  if (reportedLen != ACK_PACKET_LEN || storedLen != ACK_PACKET_LEN) return "bad length";
  const uint16_t rxCrc = (uint16_t)b[10] | ((uint16_t)b[11] << 8);
  if (crc16Ccitt(b, ACK_PACKET_LEN - 2) != rxCrc) return "CRC mismatch";
  if (b[0] != PROTO_MAGIC)   return "bad identifier";
  if (b[1] != PROTO_VERSION) return "bad version";
  if (b[2] != PKT_TYPE_ACK)  return "not an ACK";
  out.ackedSeq     = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
  out.status       = b[5];
  out.rssiDbm      = (int8_t)b[6];
  out.snrQuarterDb = (int8_t)b[7];
  out.rxValidCount = (uint16_t)b[8] | ((uint16_t)b[9] << 8);
  return nullptr;
}

// =============================================================================
//  RADIO STATE MACHINE (non-blocking)
// =============================================================================
void computeTiming() {
  cmdAirtimeMs = (uint32_t)ceilf(loraAirtimeMs(CMD_PACKET_LEN));
  ackAirtimeMs = (uint32_t)ceilf(loraAirtimeMs(ACK_PACKET_LEN));
  const uint32_t minInterval = cmdAirtimeMs + TX_MIN_IDLE_GAP_MS;
  effectiveTxIntervalMs = (TX_INTERVAL_MS > minInterval) ? TX_INTERVAL_MS : minInterval;
  ackWindowMs  = ACK_REPLY_DELAY_MS + ackAirtimeMs + ACK_WINDOW_MARGIN_MS;
  txWatchdogMs = 2 * cmdAirtimeMs + 100;

  Serial.printf("[RADIO] %ld Hz, BW %ld Hz, SF%d, CR 4/%d, sync 0x%02X, preamble %ld, %d dBm, HW CRC on\n",
                LORA_FREQUENCY_HZ, LORA_BANDWIDTH_HZ, LORA_SPREADING_FACTOR, LORA_CODING_RATE_DENOM,
                LORA_SYNC_WORD, LORA_PREAMBLE_SYMBOLS, LORA_TX_POWER_DBM);
  Serial.printf("[RADIO] Airtime: command %lu ms, ACK %lu ms\n",
                (unsigned long)cmdAirtimeMs, (unsigned long)ackAirtimeMs);
  if (effectiveTxIntervalMs != TX_INTERVAL_MS) {
    Serial.printf("[RADIO] WARNING: TX_INTERVAL_MS=%lu is shorter than airtime; using %lu ms\n",
                  (unsigned long)TX_INTERVAL_MS, (unsigned long)effectiveTxIntervalMs);
  }
  if (ENABLE_ACK) {
    Serial.printf("[RADIO] ACK every %u packets, listen window %lu ms (worst command gap ~%lu ms)\n",
                  (unsigned)ACK_EVERY_N_PACKETS, (unsigned long)ackWindowMs,
                  (unsigned long)(cmdAirtimeMs + ackWindowMs));
  } else {
    Serial.println(F("[RADIO] ACKs disabled: one-way control link"));
  }
}

void startCommandTransmission(uint32_t now) {
  // beginPacket() returns 0 if the radio is still transmitting.
  if (LoRa.beginPacket() != 1) {
    txBusySkips++;
    nextTxDueMs = now + 2;
    return;
  }

  bool wantAck = false;
  if (ENABLE_ACK && ACK_EVERY_N_PACKETS > 0) {
    packetsSinceAckReq++;
    if (packetsSinceAckReq >= ACK_EVERY_N_PACKETS) {
      wantAck = true;
      packetsSinceAckReq = 0;
    }
  }

  uint8_t flags = 0;
  if (button.stablePressed) flags |= CMD_FLAG_BUTTON_PRESSED;
  if (wantAck)              flags |= CMD_FLAG_ACK_REQUEST;

  const uint16_t seq = nextSeq++;
  uint8_t frame[CMD_PACKET_LEN];
  buildCommandPacket(frame, seq, controls.commandDeg, flags);

  LoRa.write(frame, CMD_PACKET_LEN);
  LoRa.endPacket(true);  // async: returns immediately; SX1278 transmits on its own

  currentPacketWantsAck = wantAck;
  if (wantAck) {
    awaitingAckSeq = seq;
    ackRequested++;
  }
  lastSentSeq = seq;
  txStartedMs = now;
  nextTxDueMs = now + effectiveTxIntervalMs;
  radioState  = TXS_SENDING;
}

void onTransmissionComplete(uint32_t now) {
  packetsSent++;  // radio finished sending; this does NOT mean the receiver got it
  if (!hasTransmitted) {
    hasTransmitted = true;
    firstTxDoneMs = now;
  }
  if (currentPacketWantsAck) {
    LoRa.parsePacket();  // switch to single-shot receive mode immediately
    ackListenStartMs = now;
    radioState = TXS_ACK_LISTEN;
  } else {
    radioState = TXS_IDLE;
  }
}

void handleAckPacket(int reportedLen, uint32_t now) {
  uint8_t buf[MAX_LORA_PAYLOAD];
  const int stored = readPacketBytes(buf, MAX_LORA_PAYLOAD);
  const int   rssi = LoRa.packetRssi();
  const float snr  = LoRa.packetSnr();

  AckPacket ack;
  const char *err = decodeAckPacket(buf, stored, reportedLen, ack);
  if (err != nullptr) {
    badRxPackets++;
    Serial.printf("[ACK] Rejected packet: %s (len=%d)\n", err, reportedLen);
    return;  // keep listening; parsePacket() re-arms RX next loop
  }
  if (ack.ackedSeq != awaitingAckSeq) {
    ackStale++;
    Serial.printf("[ACK] Stale ACK for seq %u (waiting for %u)\n",
                  (unsigned)ack.ackedSeq, (unsigned)awaitingAckSeq);
    return;
  }

  ackReceived++;
  consecutiveAckMisses = 0;
  everAcked = true;
  lastAckMs = now;
  lastAck = ack;
  lastAckLocalRssi = rssi;
  lastAckLocalSnr  = snr;
  if (ack.status & ACK_STATUS_LINK_RECOVERED) {
    Serial.println(F("[ACK] Receiver reports it recovered from LINK LOST"));
  }
  LoRa.idle();
  radioState = TXS_IDLE;
}

void serviceRadio(uint32_t now) {
  switch (radioState) {
    case TXS_IDLE:
      if ((int32_t)(now - nextTxDueMs) >= 0) startCommandTransmission(now);
      break;

    case TXS_SENDING:
      // Non-blocking "TX finished?" probe: LoRa.beginPacket() returns 0 while
      // the SX1278 is still in TX mode and 1 once it has returned to standby.
      if (LoRa.beginPacket() == 1) {
        onTransmissionComplete(now);
      } else if (now - txStartedMs > txWatchdogMs) {
        txTimeouts++;
        Serial.println(F("[LoRa] TX watchdog: transmission did not complete, forcing idle"));
        LoRa.idle();
        radioState = TXS_IDLE;
      }
      break;

    case TXS_ACK_LISTEN: {
      const int len = LoRa.parsePacket();  // non-blocking poll
      if (len > 0) handleAckPacket(len, now);
      if (radioState == TXS_ACK_LISTEN && (now - ackListenStartMs) >= ackWindowMs) {
        ackMissed++;
        consecutiveAckMisses++;
        if (consecutiveAckMisses == 1 || (consecutiveAckMisses % 10) == 0) {
          Serial.printf("[ACK] No ACK for seq %u (%lu consecutive misses)\n",
                        (unsigned)awaitingAckSeq, (unsigned long)consecutiveAckMisses);
        }
        LoRa.idle();
        radioState = TXS_IDLE;
      }
      break;
    }
  }
}

void retryRadioInit(uint32_t now) {
  if (now - lastRadioRetryMs < RADIO_RETRY_MS) return;
  lastRadioRetryMs = now;
  radioOk = initLoRaRadio();
  if (radioOk) {
    Serial.println(F("[LoRa] Radio initialised on retry"));
    radioState = TXS_IDLE;
    nextTxDueMs = millis();
  } else {
    Serial.println(F("[LoRa] Init failed again - check 3.3V, GND, NSS=5, RST=14, SCK/MISO/MOSI"));
  }
}

// =============================================================================
//  STATUS, DISPLAY, SERIAL
// =============================================================================
TxLinkStatus currentLinkStatus(uint32_t now) {
  if (!radioOk)        return TXL_RADIO_FAIL;
  if (!hasTransmitted) return TXL_RADIO_READY;
  if (!ENABLE_ACK)     return TXL_TRANSMITTING;
  if (everAcked && (now - lastAckMs) <= ACK_LINK_LOST_MS)      return TXL_ACK_RECEIVED;
  if (!everAcked && (now - firstTxDoneMs) <= ACK_LINK_LOST_MS) return TXL_TRANSMITTING;
  return TXL_NO_ACK;
}

const char *linkStatusText(TxLinkStatus s) {
  switch (s) {
    case TXL_RADIO_FAIL:   return "LORA INIT FAIL";
    case TXL_RADIO_READY:  return "RADIO READY";
    case TXL_TRANSMITTING: return ENABLE_ACK ? "TRANSMITTING" : "TRANSMITTING 1-WAY";
    case TXL_ACK_RECEIVED: return "ACK RECEIVED";
    case TXL_NO_ACK:       return "NO ACK/LINK LOST";
  }
  return "?";
}

// OLED refresh and long Serial prints block for tens of ms, so they run only
// when they cannot delay a scheduled transmission or an ACK receive window.
bool slowWorkAllowed(uint32_t now) {
  if (!radioOk) return true;
  switch (radioState) {
    case TXS_SENDING:
      return !currentPacketWantsAck &&
             (now - txStartedMs + OLED_REFRESH_COST_MS) <= effectiveTxIntervalMs;
    case TXS_IDLE:
      return ((int32_t)(nextTxDueMs - now) >= (int32_t)OLED_REFRESH_COST_MS) ||
             (now - lastSlowWorkMs) >= SLOW_WORK_STARVE_MS;
    case TXS_ACK_LISTEN:
    default:
      return false;
  }
}

void drawControlBar(int y, const char *label, uint8_t deg) {
  char text[10];
  snprintf(text, sizeof(text), "%s %3u", label, (unsigned)deg);
  display.setCursor(0, y);
  display.print(text);

  const int barX = 48, barW = 80, barH = 7, innerW = barW - 2;
  const int centerX = barX + 1 + (int)((long)CONTROL_CENTER_DEG * (innerW - 1) / 180);
  const int posX    = barX + 1 + (int)((long)deg * (innerW - 1) / 180);
  display.drawRect(barX, y, barW, barH, SSD1306_WHITE);
  if (posX >= centerX) display.fillRect(centerX, y + 1, posX - centerX + 1, barH - 2, SSD1306_WHITE);
  else                 display.fillRect(posX, y + 1, centerX - posX + 1, barH - 2, SSD1306_WHITE);
}

void drawDisplay(uint32_t now) {
  char line[24];
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (hasTransmitted) snprintf(line, sizeof(line), "DCS TX    SEQ %5u", (unsigned)lastSentSeq);
  else                snprintf(line, sizeof(line), "DCS TX    SEQ -----");
  display.setCursor(0, 0);
  display.print(line);

  drawControlBar(10, "AIL", controls.commandDeg[AXIS_AILERON]);
  drawControlBar(19, "ELE", controls.commandDeg[AXIS_ELEVATOR]);
  drawControlBar(28, "RUD", controls.commandDeg[AXIS_RUDDER]);

  if (ENABLE_ACK && everAcked) {
    snprintf(line, sizeof(line), "BTN:%s  RX:%4ddBm", button.stablePressed ? "ON " : "OFF", (int)lastAck.rssiDbm);
  } else {
    snprintf(line, sizeof(line), "BTN:%s", button.stablePressed ? "ON " : "OFF");
  }
  display.setCursor(0, 38);
  display.print(line);

  if (ENABLE_ACK) snprintf(line, sizeof(line), "TX:%-6lu ACK:%lu", (unsigned long)packetsSent, (unsigned long)ackReceived);
  else            snprintf(line, sizeof(line), "TX:%-6lu", (unsigned long)packetsSent);
  display.setCursor(0, 47);
  display.print(line);

  // Inverted status bar
  display.fillRect(0, 56, OLED_WIDTH, 8, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(2, 56);
  display.print(linkStatusText(currentLinkStatus(now)));
  display.setTextColor(SSD1306_WHITE);

  display.display();
}

void printDebug(uint32_t now) {
  Serial.printf("[TX] raw A=%4d E=%4d R=%4d | cmd A=%3u E=%3u R=%3u | btn=%s | %s\n",
                controls.lastRaw[AXIS_AILERON], controls.lastRaw[AXIS_ELEVATOR], controls.lastRaw[AXIS_RUDDER],
                (unsigned)controls.commandDeg[AXIS_AILERON], (unsigned)controls.commandDeg[AXIS_ELEVATOR],
                (unsigned)controls.commandDeg[AXIS_RUDDER], button.stablePressed ? "PRESSED" : "released",
                linkStatusText(currentLinkStatus(now)));
  Serial.printf("     sent=%lu ackReq=%lu ackOK=%lu ackMiss=%lu stale=%lu badRx=%lu busySkip=%lu txTimeout=%lu\n",
                (unsigned long)packetsSent, (unsigned long)ackRequested, (unsigned long)ackReceived,
                (unsigned long)ackMissed, (unsigned long)ackStale, (unsigned long)badRxPackets,
                (unsigned long)txBusySkips, (unsigned long)txTimeouts);
  if (everAcked) {
    Serial.printf("     lastACK seq=%u %lums ago | RSSI@RX=%d dBm SNR@RX=%.2f dB rxValid=%u st=0x%02X | RSSI@TX=%d dBm SNR@TX=%.2f dB\n",
                  (unsigned)lastAck.ackedSeq, (unsigned long)(now - lastAckMs), (int)lastAck.rssiDbm,
                  lastAck.snrQuarterDb / 4.0f, (unsigned)lastAck.rxValidCount, (unsigned)lastAck.status,
                  lastAckLocalRssi, lastAckLocalSnr);
  }
}

void initOled() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR, true, false);
  if (!oledOk) {
    Serial.println(F("[OLED] Init FAILED (check SDA=21, SCL=22, 3.3V, address 0x3C). Continuing without display."));
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextWrap(false);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("DCS LoRa TX"));
  display.println(F("Starting radio..."));
  display.display();
  Serial.println(F("[OLED] OK"));
}

// =============================================================================
//  SETUP / LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);  // one-time, lets the USB serial port settle
  Serial.println(F("\n=== DCS LoRa TRANSMITTER ==="));
  Serial.println(F("Educational ground prototype - not for real aircraft."));

  const uint8_t crcTest[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
  Serial.printf("[SELFTEST] CRC16 = 0x%04X (expected 0x29B1) -> %s\n", crc16Ccitt(crcTest, 9),
                crc16Ccitt(crcTest, 9) == 0x29B1 ? "PASS" : "FAIL");

  pinMode(PIN_JOY_SW, INPUT_PULLUP);
  button.lastRawPressed = button.stablePressed = (digitalRead(PIN_JOY_SW) == LOW);

  analogReadResolution(12);         // 0..4095
  analogSetAttenuation(ADC_11db);   // ~0..3.1 V input range
  validateCalibration("Aileron ", calAileron);
  validateCalibration("Elevator", calElevator);
  validateCalibration("Rudder  ", calRudder);
  controls.filterPrimed = false;
  sampleControls();

  initOled();

  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  computeTiming();
  radioOk = initLoRaRadio();
  lastRadioRetryMs = millis();
  if (radioOk) Serial.println(F("[LoRa] SX1278 initialised - RADIO READY"));
  else         Serial.println(F("[LoRa] Init FAILED - will retry every 2 s"));

  nextTxDueMs = millis() + TX_STARTUP_HOLD_MS;
  if (oledOk) drawDisplay(millis());
}

void loop() {
  const uint32_t now = millis();

  updateButton(now);

  if (now - lastSampleMs >= INPUT_SAMPLE_MS) {
    lastSampleMs = now;
    sampleControls();
  }

  if (radioOk) serviceRadio(now);
  else         retryRadioInit(now);

  // At most one slow job per pass, and only when it cannot disturb the radio.
  const uint32_t t = millis();
  if (slowWorkAllowed(t)) {
    if (oledOk && (t - lastOledMs) >= OLED_REFRESH_MS) {
      lastOledMs = t;
      lastSlowWorkMs = t;
      drawDisplay(t);
    } else if ((t - lastSerialMs) >= SERIAL_DEBUG_MS) {
      lastSerialMs = t;
      lastSlowWorkMs = t;
      printDebug(t);
    }
  }
}
