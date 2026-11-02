// LD2450 radar -> USB serial for the ESP32-S3-Zero.
//
// Reads the radar's 30-byte frames and prints one line per frame over USB:
//     D,x1,y1,v1,x2,y2,v2,x3,y3,v3
// (mm, mm, cm/s for each of up to 3 targets; an absent target is all zeros).
// viewer.py reads these lines and draws the bird's-eye view.

#include <Arduino.h>

// ============================ Wiring ============================
//   Radar 5V  -> board 5V pin
//   Radar GND -> board GND
//   Radar TX  -> RADAR_RX_PIN (GPIO4)
//   Radar RX  -> RADAR_TX_PIN (GPIO5)
static const int RADAR_RX_PIN = 4;
static const int RADAR_TX_PIN = 5;
static const uint32_t RADAR_BAUD = 256000;

// Frame: AA FF 03 00 | 3 targets x 8 bytes | 55 CC  = 30 bytes
static const uint8_t HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
static const uint8_t FOOTER[2] = {0x55, 0xCC};
static const size_t FRAME_LEN = 30;

struct Target {
  bool present;
  int16_t x;      // mm, + right / - left of sensor
  int16_t y;      // mm, distance in front of sensor
  int16_t speed;  // cm/s
};

// LD2450 signed encoding: bit15 = 1 -> positive, 0 -> negative, low 15 bits = magnitude
static int16_t decodeSigned(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  int16_t mag = raw & 0x7FFF;
  return (raw & 0x8000) ? mag : -mag;
}

static bool parseFrame(const uint8_t *buf, Target *t) {
  for (int i = 0; i < 4; i++) if (buf[i] != HEADER[i]) return false;
  if (buf[28] != FOOTER[0] || buf[29] != FOOTER[1]) return false;

  for (int n = 0; n < 3; n++) {
    const uint8_t *p = buf + 4 + n * 8;
    t[n].x = decodeSigned(p[0], p[1]);
    t[n].y = decodeSigned(p[2], p[3]);
    t[n].speed = decodeSigned(p[4], p[5]);
    t[n].present = !(p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 0 &&
                     p[4] == 0 && p[5] == 0 && p[6] == 0 && p[7] == 0);
  }
  return true;
}

static uint8_t frameBuf[FRAME_LEN];
static size_t frameIdx = 0;

static uint32_t lastBeat = 0;
static uint32_t bytesSeen = 0;
static uint32_t framesOk = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // never block the loop waiting for a USB host to read

  // Wait up to 3 s for a USB serial connection, then carry on regardless
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) {
    delay(10);
  }

  Serial1.setRxBufferSize(2048);
  Serial1.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
}

void loop() {
  // Status line, ignored by viewer.py but handy in a plain serial monitor.
  // If "radar bytes" never climbs, the radar isn't talking: check wiring and power.
  uint32_t now = millis();
  if (now - lastBeat > 1000) {
    lastBeat = now;
    Serial.printf("alive | radar bytes: %u | good frames: %u\n", bytesSeen, framesOk);
  }

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    bytesSeen++;

    // Resync: only start collecting when the header bytes arrive in order
    if (frameIdx < 4) {
      if (b == HEADER[frameIdx]) {
        frameBuf[frameIdx++] = b;
      } else if (b == HEADER[0]) {
        frameBuf[0] = b;
        frameIdx = 1;
      } else {
        frameIdx = 0;
      }
      continue;
    }

    frameBuf[frameIdx++] = b;

    if (frameIdx == FRAME_LEN) {
      frameIdx = 0;
      Target t[3];
      if (parseFrame(frameBuf, t)) {
        framesOk++;
        Serial.printf("D,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                      t[0].present ? t[0].x : 0, t[0].present ? t[0].y : 0, t[0].present ? t[0].speed : 0,
                      t[1].present ? t[1].x : 0, t[1].present ? t[1].y : 0, t[1].present ? t[1].speed : 0,
                      t[2].present ? t[2].x : 0, t[2].present ? t[2].y : 0, t[2].present ? t[2].speed : 0);
      }
    }
  }
}
