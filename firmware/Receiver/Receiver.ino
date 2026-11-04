/*
 * =============================================================================
 *  Receiver.ino
 *  DCS Project - ESP32 + SX1278 (RA-02) 433 MHz LoRa control link
 *
 *  ROLE: Simulated drone. Receives and validates command packets, drives three
 *        servos (aileron / elevator / rudder), shows link data on the OLED,
 *        replies with ACKs when asked, and goes to neutral on link loss.
 * -----------------------------------------------------------------------------
 *  EDUCATIONAL, GROUND-BASED PROTOTYPE ONLY. This is NOT a flight-certified or
 *  safety-rated controller and must not be used to control a real aircraft.
 * -----------------------------------------------------------------------------
 *  Board     : ESP32 Dev Module (ESP32 DevKit V1 / ESP32-WROOM-32)
 *  Libraries : LoRa (Sandeep Mistry), Adafruit SSD1306, Adafruit GFX, ESP32Servo
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <LoRa.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// =============================================================================
//  PIN MAP  (matches the project wiring tables - do not change unless rewired)
// =============================================================================
// SX1278 RA-02 (VSPI bus)
static const int PIN_LORA_SCK   = 18;
static const int PIN_LORA_MISO  = 19;
static const int PIN_LORA_MOSI  = 23;
static const int PIN_LORA_NSS   = 5;
static const int PIN_LORA_RESET = 14;
static const int PIN_LORA_DIO0  = 26;  // LoRa DIO0 ONLY - never used for a servo

// Servo signal outputs
static const int PIN_SERVO_AILERON  = 25;
static const int PIN_SERVO_ELEVATOR = 27;
static const int PIN_SERVO_RUDDER   = 32;

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
//  RECEIVER CONFIGURATION
// =============================================================================
static const uint32_t LINK_TIMEOUT_MS      = 500;   // no valid command for this long -> failsafe neutral
static const bool     ACK_REPLY_ENABLED    = true;  // reply when the transmitter requests an ACK
static const uint16_t RESYNC_AFTER_OLD_PKTS = 10;   // accept "older" seq after this many in a row (TX reboot)

// Servo outputs (index 0 = aileron, 1 = elevator, 2 = rudder)
static const uint8_t SERVO_NEUTRAL_DEG[3]   = { 90, 90, 90 };
static const uint8_t SERVO_LIMIT_MIN_DEG[3] = { 0, 0, 0 };        // mechanical travel limits
static const uint8_t SERVO_LIMIT_MAX_DEG[3] = { 180, 180, 180 };
static const int     SERVO_PULSE_MIN_US     = 1000;  // pulse width at 0 deg  (widen carefully, e.g. 500)
static const int     SERVO_PULSE_MAX_US     = 2000;  // pulse width at 180 deg (widen carefully, e.g. 2400)
static const int     SERVO_PWM_HZ           = 50;

static const uint32_t OLED_REFRESH_MS      = 200;
static const uint32_t RX_QUIET_GAP_MS      = 100;   // timed slow work only if no packet for this long
static const uint32_t RADIO_READY_SHOW_MS  = 1500;  // show RADIO READY this long after init
static const uint32_t SERIAL_SUMMARY_MS    = 1000;
static const uint32_t RADIO_RETRY_MS       = 2000;

// =============================================================================
//  TYPES AND GLOBAL STATE
// =============================================================================
enum RxLinkStatus : uint8_t { RXL_RADIO_FAIL, RXL_RADIO_READY, RXL_WAITING, RXL_LINK_OK, RXL_LINK_LOST };

enum RejectReason : uint8_t {
  REJ_NONE, REJ_LENGTH, REJ_CRC, REJ_MAGIC, REJ_VERSION, REJ_TYPE,
  REJ_FLAGS, REJ_RANGE, REJ_DUPLICATE, REJ_OLD_SEQ
};

struct CommandPacket {
  uint16_t seq;
  uint8_t  deg[3];
  uint8_t  flags;
};

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
Servo servos[3];
static const int SERVO_PINS[3] = { PIN_SERVO_AILERON, PIN_SERVO_ELEVATOR, PIN_SERVO_RUDDER };

bool oledOk = false, radioOk = false, servosOk = false;

uint8_t  appliedDeg[3] = { 90, 90, 90 };
bool     failsafeActive = true;        // true until the first valid packet
bool     linkLostSinceLastAck = false;
bool     haveLastSeq = false;
uint16_t lastSeq = 0;
uint16_t consecutiveOldSeq = 0;
bool     lastButtonPressed = false;

// Statistics
uint32_t validCount = 0, invalidCount = 0, lostCount = 0, duplicateCount = 0;
uint32_t resyncCount = 0, linkLossEvents = 0, ackSent = 0, ackFailures = 0;
int      lastRssi = 0;
float    lastSnr  = 0.0f;

uint32_t lastValidMs = 0, lastPacketActivityMs = 0, failsafeStartMs = 0;
uint32_t radioReadyMs = 0, lastRadioRetryMs = 0, lastOledMs = 0, lastSerialMs = 0;
uint32_t validAtLastSummary = 0;

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
//  SERVOS
// =============================================================================
void writeServo(uint8_t i, uint8_t deg) {
  if (deg < SERVO_LIMIT_MIN_DEG[i]) deg = SERVO_LIMIT_MIN_DEG[i];
  if (deg > SERVO_LIMIT_MAX_DEG[i]) deg = SERVO_LIMIT_MAX_DEG[i];
  const long us = SERVO_PULSE_MIN_US +
                  ((long)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * deg + 90) / 180;
  if (servos[i].attached()) servos[i].writeMicroseconds((int)us);
  appliedDeg[i] = deg;
}

void applyNeutral() {
  for (uint8_t i = 0; i < 3; i++) writeServo(i, SERVO_NEUTRAL_DEG[i]);
}

void validateServoConfig() {
  const char *names[3] = { "Aileron", "Elevator", "Rudder" };
  for (uint8_t i = 0; i < 3; i++) {
    const bool ok = SERVO_LIMIT_MIN_DEG[i] <= SERVO_NEUTRAL_DEG[i] &&
                    SERVO_NEUTRAL_DEG[i] <= SERVO_LIMIT_MAX_DEG[i] &&
                    SERVO_LIMIT_MAX_DEG[i] <= CONTROL_MAX_DEG;
    Serial.printf("[SERVO] %-8s GPIO%d neutral=%u limits=%u..%u -> %s\n", names[i], SERVO_PINS[i],
                  (unsigned)SERVO_NEUTRAL_DEG[i], (unsigned)SERVO_LIMIT_MIN_DEG[i],
                  (unsigned)SERVO_LIMIT_MAX_DEG[i], ok ? "OK" : "CHECK CONFIG");
  }
  if (SERVO_PULSE_MIN_US >= SERVO_PULSE_MAX_US) Serial.println(F("[SERVO] WARNING: pulse min >= max"));
}

void initServos() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servosOk = true;
  for (uint8_t i = 0; i < 3; i++) {
    servos[i].setPeriodHertz(SERVO_PWM_HZ);
    servos[i].attach(SERVO_PINS[i], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    if (!servos[i].attached()) {
      servosOk = false;
      Serial.printf("[SERVO] Attach FAILED on GPIO%d\n", SERVO_PINS[i]);
    }
  }
  applyNeutral();  // safe neutral before the radio is even started
  validateServoConfig();
}

// =============================================================================
//  PACKET VALIDATION
// =============================================================================
const char *reasonText(RejectReason r) {
  switch (r) {
    case REJ_NONE:      return "ok";
    case REJ_LENGTH:    return "bad length";
    case REJ_CRC:       return "CRC mismatch";
    case REJ_MAGIC:     return "bad identifier";
    case REJ_VERSION:   return "bad version";
    case REJ_TYPE:      return "not a command";
    case REJ_FLAGS:     return "undefined flag bits set";
    case REJ_RANGE:     return "control value out of range";
    case REJ_DUPLICATE: return "duplicate sequence";
    case REJ_OLD_SEQ:   return "old / out-of-order sequence";
  }
  return "?";
}

RejectReason decodeCommandPacket(const uint8_t *b, int storedLen, int reportedLen, CommandPacket &out) {
  if (reportedLen != CMD_PACKET_LEN || storedLen != CMD_PACKET_LEN) return REJ_LENGTH;
  const uint16_t rxCrc = (uint16_t)b[9] | ((uint16_t)b[10] << 8);
  if (crc16Ccitt(b, CMD_PACKET_LEN - 2) != rxCrc) return REJ_CRC;
  if (b[0] != PROTO_MAGIC)      return REJ_MAGIC;
  if (b[1] != PROTO_VERSION)    return REJ_VERSION;
  if (b[2] != PKT_TYPE_COMMAND) return REJ_TYPE;
  if (b[8] & (uint8_t)~CMD_FLAGS_DEFINED_MASK) return REJ_FLAGS;
  for (uint8_t i = 0; i < 3; i++) {
    if (b[5 + i] < CONTROL_MIN_DEG || b[5 + i] > CONTROL_MAX_DEG) return REJ_RANGE;
  }
  out.seq   = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
  out.deg[0] = b[5];
  out.deg[1] = b[6];
  out.deg[2] = b[7];
  out.flags = b[8];
  return REJ_NONE;
}

// 16-bit serial-number arithmetic. Updates sequence state only on accept.
RejectReason checkSequence(uint16_t seq) {
  if (!haveLastSeq || failsafeActive) {
    if (haveLastSeq) {
      resyncCount++;
      Serial.printf("[RX] Sequence resynchronised at %u\n", (unsigned)seq);
    }
    haveLastSeq = true;
    lastSeq = seq;
    consecutiveOldSeq = 0;
    return REJ_NONE;
  }
  const uint16_t diff = (uint16_t)(seq - lastSeq);
  if (diff == 0) {
    duplicateCount++;
    return REJ_DUPLICATE;
  }
  if (diff < 0x8000) {
    if (diff > 1) {
      lostCount += (uint32_t)(diff - 1);
      Serial.printf("[RX] Packet loss: %u missing (seq %u -> %u)\n",
                    (unsigned)(diff - 1), (unsigned)lastSeq, (unsigned)seq);
    }
    lastSeq = seq;
    consecutiveOldSeq = 0;
    return REJ_NONE;
  }
  consecutiveOldSeq++;
  if (consecutiveOldSeq >= RESYNC_AFTER_OLD_PKTS) {  // e.g. transmitter rebooted quickly
    resyncCount++;
    Serial.printf("[RX] Sequence resynchronised at %u (transmitter restart?)\n", (unsigned)seq);
    lastSeq = seq;
    consecutiveOldSeq = 0;
    return REJ_NONE;
  }
  return REJ_OLD_SEQ;
}

// =============================================================================
//  ACKNOWLEDGEMENTS
// =============================================================================
void buildAckPacket(uint8_t *f, uint16_t seq, int rssi, float snr) {
  long r = rssi;
  if (r < -128) r = -128;
  if (r > 127)  r = 127;
  long q = lroundf(snr * 4.0f);
  if (q < -128) q = -128;
  if (q > 127)  q = 127;

  uint8_t status = 0;
  if (servosOk)             status |= ACK_STATUS_SERVOS_OK;
  if (oledOk)               status |= ACK_STATUS_OLED_OK;
  if (linkLostSinceLastAck) status |= ACK_STATUS_LINK_RECOVERED;

  const uint16_t valid16 = (uint16_t)(validCount & 0xFFFF);
  f[0] = PROTO_MAGIC;
  f[1] = PROTO_VERSION;
  f[2] = PKT_TYPE_ACK;
  f[3] = (uint8_t)(seq & 0xFF);
  f[4] = (uint8_t)(seq >> 8);
  f[5] = status;
  f[6] = (uint8_t)(int8_t)r;
  f[7] = (uint8_t)(int8_t)q;
  f[8] = (uint8_t)(valid16 & 0xFF);
  f[9] = (uint8_t)(valid16 >> 8);
  const uint16_t crc = crc16Ccitt(f, ACK_PACKET_LEN - 2);
  f[10] = (uint8_t)(crc & 0xFF);
  f[11] = (uint8_t)(crc >> 8);
}

// Sent only straight after a command that requested it. The transmitter is
// listening at that moment and will not transmit until its ACK window closes.
void sendAck(uint16_t seq, int rssi, float snr) {
  uint8_t frame[ACK_PACKET_LEN];
  buildAckPacket(frame, seq, rssi, snr);
  delay(ACK_REPLY_DELAY_MS);  // short, bounded: gives the TX time to enter receive mode
  if (LoRa.beginPacket() != 1) {
    ackFailures++;
    return;
  }
  LoRa.write(frame, ACK_PACKET_LEN);
  LoRa.endPacket();  // blocks for the ACK airtime only (~41 ms at SF7/BW125)
  ackSent++;
  linkLostSinceLastAck = false;
}

// =============================================================================
//  RECEIVE PATH
// =============================================================================
void handleIncomingPacket(int reportedLen, uint32_t now) {
  uint8_t buf[MAX_LORA_PAYLOAD];
  const int   stored = readPacketBytes(buf, MAX_LORA_PAYLOAD);
  const int   rssi   = LoRa.packetRssi();
  const float snr    = LoRa.packetSnr();
  lastPacketActivityMs = now;

  CommandPacket cmd;
  RejectReason reason = decodeCommandPacket(buf, stored, reportedLen, cmd);
  if (reason == REJ_NONE) reason = checkSequence(cmd.seq);

  if (reason != REJ_NONE) {
    invalidCount++;
    Serial.printf("[RX] Rejected: %s (len=%d, RSSI=%d dBm, SNR=%.2f dB)\n",
                  reasonText(reason), reportedLen, rssi, snr);
    LoRa.parsePacket();  // re-arm receive immediately
    return;
  }

  // ---- Valid command: apply it ----
  validCount++;
  lastValidMs = now;
  lastRssi = rssi;
  lastSnr = snr;
  lastButtonPressed = (cmd.flags & CMD_FLAG_BUTTON_PRESSED) != 0;

  if (failsafeActive) {
    failsafeActive = false;
    if (validCount == 1) Serial.println(F("[RX] First valid packet - LINK OK"));
    else Serial.printf("[RX] LINK RESTORED after %lu ms\n", (unsigned long)(now - failsafeStartMs));
  }
  for (uint8_t i = 0; i < 3; i++) writeServo(i, cmd.deg[i]);

  if (ACK_REPLY_ENABLED && (cmd.flags & CMD_FLAG_ACK_REQUEST)) {
    sendAck(cmd.seq, rssi, snr);
  }

  LoRa.parsePacket();  // re-arm receive BEFORE any slow OLED/Serial work
}

void checkLinkTimeout(uint32_t now) {
  if (!failsafeActive && (now - lastValidMs) > LINK_TIMEOUT_MS) {
    failsafeActive = true;
    linkLostSinceLastAck = true;
    linkLossEvents++;
    failsafeStartMs = now;
    applyNeutral();
    Serial.printf("[RX] LINK LOST: no valid command for %lu ms -> servos to neutral\n",
                  (unsigned long)LINK_TIMEOUT_MS);
  }
}

void retryRadioInit(uint32_t now) {
  if (now - lastRadioRetryMs < RADIO_RETRY_MS) return;
  lastRadioRetryMs = now;
  radioOk = initLoRaRadio();
  if (radioOk) {
    radioReadyMs = millis();
    LoRa.parsePacket();
    Serial.println(F("[LoRa] Radio initialised on retry"));
  } else {
    Serial.println(F("[LoRa] Init failed again - check 3.3V, GND, NSS=5, RST=14, SCK/MISO/MOSI"));
  }
}

// =============================================================================
//  STATUS, DISPLAY, SERIAL
// =============================================================================
RxLinkStatus currentLinkStatus(uint32_t now) {
  if (!radioOk) return RXL_RADIO_FAIL;
  if (validCount == 0) return ((now - radioReadyMs) < RADIO_READY_SHOW_MS) ? RXL_RADIO_READY : RXL_WAITING;
  return failsafeActive ? RXL_LINK_LOST : RXL_LINK_OK;
}

const char *linkStatusText(RxLinkStatus s) {
  switch (s) {
    case RXL_RADIO_FAIL:  return "LORA INIT FAIL";
    case RXL_RADIO_READY: return "RADIO READY";
    case RXL_WAITING:     return "WAITING";
    case RXL_LINK_OK:     return "LINK OK";
    case RXL_LINK_LOST:   return "LINK LOST";
  }
  return "?";
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

  if (haveLastSeq) snprintf(line, sizeof(line), "DCS RX    SEQ %5u", (unsigned)lastSeq);
  else             snprintf(line, sizeof(line), "DCS RX    SEQ -----");
  display.setCursor(0, 0);
  display.print(line);

  drawControlBar(10, "AIL", appliedDeg[0]);  // values actually applied to the servos
  drawControlBar(19, "ELE", appliedDeg[1]);
  drawControlBar(28, "RUD", appliedDeg[2]);

  if (validCount > 0) snprintf(line, sizeof(line), "RSSI %4d SNR %5.1f", lastRssi, lastSnr);
  else                snprintf(line, sizeof(line), "RSSI ---- SNR  ---");
  display.setCursor(0, 38);
  display.print(line);

  snprintf(line, sizeof(line), "PKT %-6lu LOST %lu", (unsigned long)validCount, (unsigned long)lostCount);
  display.setCursor(0, 47);
  display.print(line);

  const RxLinkStatus st = currentLinkStatus(now);
  display.fillRect(0, 56, OLED_WIDTH, 8, SSD1306_WHITE);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(2, 56);
  display.print(linkStatusText(st));
  if (st == RXL_LINK_OK && lastButtonPressed) {
    display.setCursor(OLED_WIDTH - 20, 56);
    display.print(F("BTN"));
  }
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void printSummary(uint32_t now) {
  const uint32_t newValid = validCount - validAtLastSummary;
  validAtLastSummary = validCount;
  const float lossPct = (validCount + lostCount) ? (100.0f * lostCount / (float)(validCount + lostCount)) : 0.0f;
  Serial.printf("[RX] %s | valid=%lu (+%lu/s) lost=%lu (%.1f%%) invalid=%lu dup=%lu | RSSI=%d dBm SNR=%.2f dB | "
                "A=%u E=%u R=%u btn=%d | acks=%lu linkLoss=%lu\n",
                linkStatusText(currentLinkStatus(now)), (unsigned long)validCount, (unsigned long)newValid,
                (unsigned long)lostCount, lossPct, (unsigned long)invalidCount, (unsigned long)duplicateCount,
                lastRssi, lastSnr, (unsigned)appliedDeg[0], (unsigned)appliedDeg[1], (unsigned)appliedDeg[2],
                lastButtonPressed ? 1 : 0, (unsigned long)ackSent, (unsigned long)linkLossEvents);
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
  display.println(F("DCS LoRa RX"));
  display.println(F("Servos neutral"));
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
  Serial.println(F("\n=== DCS LoRa RECEIVER (simulated drone) ==="));
  Serial.println(F("Educational ground prototype - not for real aircraft."));

  initServos();  // neutral first

  const uint8_t crcTest[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
  Serial.printf("[SELFTEST] CRC16 = 0x%04X (expected 0x29B1) -> %s\n", crc16Ccitt(crcTest, 9),
                crc16Ccitt(crcTest, 9) == 0x29B1 ? "PASS" : "FAIL");

  initOled();

  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  Serial.printf("[RADIO] %ld Hz, BW %ld Hz, SF%d, CR 4/%d, sync 0x%02X, preamble %ld, HW CRC on\n",
                LORA_FREQUENCY_HZ, LORA_BANDWIDTH_HZ, LORA_SPREADING_FACTOR, LORA_CODING_RATE_DENOM,
                LORA_SYNC_WORD, LORA_PREAMBLE_SYMBOLS);
  Serial.printf("[RADIO] Airtime: command %.1f ms, ACK %.1f ms; link timeout %lu ms\n",
                loraAirtimeMs(CMD_PACKET_LEN), loraAirtimeMs(ACK_PACKET_LEN), (unsigned long)LINK_TIMEOUT_MS);

  radioOk = initLoRaRadio();
  lastRadioRetryMs = millis();
  if (radioOk) {
    radioReadyMs = millis();
    LoRa.parsePacket();  // arm receive
    Serial.println(F("[LoRa] SX1278 initialised - RADIO READY, waiting for packets"));
  } else {
    Serial.println(F("[LoRa] Init FAILED - servos held neutral, retrying every 2 s"));
  }
  if (oledOk) drawDisplay(millis());
}

void loop() {
  uint32_t now = millis();
  bool handledPacket = false;

  if (radioOk) {
    const int len = LoRa.parsePacket();  // non-blocking; also re-arms single-shot RX
    if (len > 0) {
      handleIncomingPacket(len, now);
      handledPacket = true;
    }
  } else {
    retryRadioInit(now);
  }

  now = millis();
  checkLinkTimeout(now);

  // Slow work runs right after a packet (RX already re-armed and the next
  // packet takes ~41 ms on air) or when the channel has been quiet.
  const bool quiet = (now - lastPacketActivityMs) >= RX_QUIET_GAP_MS;
  if (handledPacket || quiet || !radioOk) {
    if (oledOk && (now - lastOledMs) >= OLED_REFRESH_MS) {
      lastOledMs = now;
      drawDisplay(now);
    } else if ((now - lastSerialMs) >= SERIAL_SUMMARY_MS) {
      lastSerialMs = now;
      printSummary(now);
    }
  }
}
