// LD2450 radar + 1.3" OLED bird's-eye display for the ESP32-S3-Zero.
//
// Shows up to 3 tracked targets on the OLED and also streams every radar frame over
// USB serial as "D,..." lines, so viewer.py works with this build too.

#include <Arduino.h>
#include <U8g2lib.h>

// ============================ Wiring ============================
// LD2450 radar
//   Radar 5V  -> board 5V pin
//   Radar GND -> board GND
//   Radar TX  -> RADAR_RX_PIN (GPIO4)
//   Radar RX  -> RADAR_TX_PIN (GPIO5)
// 1.3" I2C OLED
//   OLED VCC  -> board 3V3   (NOT 5V)
//   OLED GND  -> board GND   (a real GND pin, never a GPIO)
//   OLED SDA  -> OLED_SDA_PIN (GPIO1)
//   OLED SCL  -> OLED_SCL_PIN (GPIO3)
static const int RADAR_RX_PIN = 4;
static const int RADAR_TX_PIN = 5;
static const uint32_t RADAR_BAUD = 256000;

static const int OLED_SDA_PIN = 1;
static const int OLED_SCL_PIN = 3;

// 0-255. Lower = dimmer, less current, longer panel life.
static const uint8_t OLED_CONTRAST = 100;

// Most 1.3" OLEDs are SH1106. If yours is an SSD1306, swap to the second line.
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL_PIN, OLED_SDA_PIN);

// ======================= Radar frame parsing =======================
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

// ============================ Tracking ============================
// The sensor reorders its 3 slots freely and drops targets on sideways motion,
// so keep identity by nearest match, smooth jitter, and hold lost targets briefly.
static const float ALPHA = 0.45f;          // smoothing: lower = smoother but laggier
static const float GATE_MM = 900.0f;       // max jump between frames for the same target
static const uint32_t HOLD_MS = 1000;      // keep a lost target this long
static const uint32_t LOST_AFTER_MS = 250; // draw as hollow ring after this long unseen
static const uint8_t MIN_HITS = 2;         // frames before a new target is shown

struct Track {
  bool active;
  float x, y;
  uint8_t hits;
  uint32_t lastSeen;
};
static Track tracks[3];

struct Det { float x, y; };

static void updateTracks(const Det *dets, int nDet, uint32_t now) {
  bool detUsed[3] = {false, false, false};
  bool trkUsed[3] = {false, false, false};

  // Greedy nearest-neighbour matching, closest pair first
  for (int iter = 0; iter < 3; iter++) {
    float best = GATE_MM;
    int bt = -1, bd = -1;
    for (int t = 0; t < 3; t++) {
      if (!tracks[t].active || trkUsed[t]) continue;
      for (int d = 0; d < nDet; d++) {
        if (detUsed[d]) continue;
        float dist = hypotf(tracks[t].x - dets[d].x, tracks[t].y - dets[d].y);
        if (dist < best) { best = dist; bt = t; bd = d; }
      }
    }
    if (bt < 0) break;
    Track &tr = tracks[bt];
    tr.x += ALPHA * (dets[bd].x - tr.x);
    tr.y += ALPHA * (dets[bd].y - tr.y);
    if (tr.hits < 255) tr.hits++;
    tr.lastSeen = now;
    trkUsed[bt] = true;
    detUsed[bd] = true;
  }

  // Unmatched detections start new tracks in a free slot
  for (int d = 0; d < nDet; d++) {
    if (detUsed[d]) continue;
    for (int t = 0; t < 3; t++) {
      if (!tracks[t].active) {
        tracks[t] = {true, dets[d].x, dets[d].y, 1, now};
        break;
      }
    }
  }
}

static void pruneTracks(uint32_t now) {
  for (int t = 0; t < 3; t++) {
    if (!tracks[t].active) continue;
    uint32_t limit = (tracks[t].hits >= MIN_HITS) ? HOLD_MS : 300;  // ghosts die fast
    if (now - tracks[t].lastSeen > limit) tracks[t].active = false;
  }
}

// ============================= Display =============================
// 128x64 screen: bottom-centre is the sensor, 64 px = 5000 mm both ways,
// so the full width is +/-5 m and the full height is 5 m ahead.
static const float SCREEN_RANGE_MM = 5000.0f;
static const int OX = 64;
static const int OY = 63;
static const float PX_PER_MM = 64.0f / SCREEN_RANGE_MM;

static void drawScreen(uint32_t now, bool radarAlive) {
  u8g2.clearBuffer();

  // Field-of-view edges (+/-60 deg from straight ahead).
  // u8g2 takes unsigned coordinates, so endpoints must stay on screen (negative values
  // wrap around and draw the line in the wrong place). The edge hits the side of the
  // screen first: 63 px sideways = 36 px up (63 * tan 30 deg).
  const int ex = 63;
  const int ey = 36;
  u8g2.drawLine(OX, OY, OX - ex, OY - ey);
  u8g2.drawLine(OX, OY, OX + ex, OY - ey);

  // Range arcs at 2 m and 4 m
  const int r2 = (int)(2000.0f * PX_PER_MM + 0.5f);
  const int r4 = (int)(4000.0f * PX_PER_MM + 0.5f);
  u8g2.drawCircle(OX, OY, r2, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
  u8g2.drawCircle(OX, OY, r4, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);

  // Sensor marker
  u8g2.drawTriangle(OX, OY - 4, OX - 3, OY, OX + 3, OY);

  // Targets
  float nearest = -1.0f;
  int shown = 0;
  for (int t = 0; t < 3; t++) {
    const Track &tr = tracks[t];
    if (!tr.active || tr.hits < MIN_HITS) continue;
    shown++;
    int px = OX + (int)(tr.x * PX_PER_MM);
    int py = OY - (int)(tr.y * PX_PER_MM);
    px = constrain(px, 3, 124);
    py = constrain(py, 3, 60);
    bool lost = (now - tr.lastSeen) > LOST_AFTER_MS;
    if (lost) u8g2.drawCircle(px, py, 3);
    else      u8g2.drawDisc(px, py, 3);

    float d = hypotf(tr.x, tr.y) / 1000.0f;
    if (nearest < 0 || d < nearest) nearest = d;
  }

  u8g2.setFont(u8g2_font_5x7_tr);
  if (!radarAlive) {
    u8g2.drawStr(22, 30, "NO RADAR DATA");
  } else {
    char buf[16];
    if (nearest >= 0) snprintf(buf, sizeof(buf), "%.2fm", nearest);
    else              snprintf(buf, sizeof(buf), "clear");
    u8g2.drawStr(0, 7, buf);
    snprintf(buf, sizeof(buf), "n=%d", shown);
    u8g2.drawStr(110, 7, buf);
  }

  u8g2.sendBuffer();
}

// ============================== Main ==============================
static uint8_t frameBuf[FRAME_LEN];
static size_t frameIdx = 0;

static uint32_t lastBeat = 0;
static uint32_t lastDraw = 0;
static uint32_t lastFrameMs = 0;
static uint32_t bytesSeen = 0;
static uint32_t framesOk = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);  // never block the loop waiting for a USB host to read

  u8g2.begin();
  u8g2.setContrast(OLED_CONTRAST);
  u8g2.setBusClock(400000);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(20, 30, "LD2450 radar");
  u8g2.drawStr(30, 44, "starting...");
  u8g2.sendBuffer();

  // Wait up to 3 s for a USB serial connection, then carry on regardless
  uint32_t start = millis();
  while (!Serial && millis() - start < 3000) {
    delay(10);
  }

  // Large RX buffer so UART data survives while the OLED redraws
  Serial1.setRxBufferSize(2048);
  Serial1.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
}

void loop() {
  uint32_t now = millis();

  // Status line, ignored by viewer.py but handy in a plain serial monitor
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
        lastFrameMs = millis();

        // Raw line for viewer.py: D,x1,y1,v1,x2,y2,v2,x3,y3,v3  (absent target = zeros)
        Serial.printf("D,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                      t[0].present ? t[0].x : 0, t[0].present ? t[0].y : 0, t[0].present ? t[0].speed : 0,
                      t[1].present ? t[1].x : 0, t[1].present ? t[1].y : 0, t[1].present ? t[1].speed : 0,
                      t[2].present ? t[2].x : 0, t[2].present ? t[2].y : 0, t[2].present ? t[2].speed : 0);

        // Feed the on-board tracker
        Det dets[3];
        int n = 0;
        for (int i = 0; i < 3; i++) {
          if (t[i].present) dets[n++] = {(float)t[i].x, (float)t[i].y};
        }
        updateTracks(dets, n, lastFrameMs);
      }
    }
  }

  now = millis();
  pruneTracks(now);

  // Redraw at ~12 Hz
  if (now - lastDraw >= 80) {
    lastDraw = now;
    bool alive = (framesOk > 0) && (now - lastFrameMs < 1500);
    drawScreen(now, alive);
  }
}
