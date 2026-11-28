/*
 * Servos.h — 2 servo qua hub Ohstem driverV2 (I2C 0x54)
 *   Servo 1 (S1): GẮP bóng   — servo góc, mở / đóng
 *   Servo 2 (S2): CỬA NẠP bóng — servo 180°, chỉ mở khi bánh bắn đủ rpm
 *
 * Nói chuyện thẳng với hub qua I2C (cùng thanh ghi với thư viện OhStemRobotics: WHO_AM_I 42, firmware 40/41, servo S1..S4
 * ở 24/26/28/30, giá trị 16 bit góc 0..180). Không cần cài thư viện OhStemRobotics nữa: bản 0.2.0 của thư viện dùng
 * ledcAttach (chỉ có ở core ESP32 3.x) nên không biên dịch được trên board Bluepad32 4.1.0 (core 2.x).
 */
#pragma once
#include <Wire.h>
#include "config.h"
#include "orc_protocol.h"
#include "Common.h"
#include "Shooter.h"

namespace hub {
const uint8_t REG_MOTOR_INDEX = 16, REG_FW = 40, REG_WHOAMI = 42;
const uint8_t REG_SERVO[4] = {24, 26, 28, 30};
const uint8_t S1 = 0, S2 = 1, ALL_MOTORS = 63;

static bool readReg(uint8_t reg, uint8_t &v) {
  Wire.beginTransmission(ORC_ADDR_OHSTEM_HUB);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)ORC_ADDR_OHSTEM_HUB, (size_t)1, true) != 1) { while (Wire.available()) Wire.read(); return false; }
  v = Wire.read();
  return true;
}
static void write16(uint8_t reg, int16_t a) {
  Wire.beginTransmission(ORC_ADDR_OHSTEM_HUB);
  Wire.write(reg); Wire.write((uint8_t)(a & 0xFF)); Wire.write((uint8_t)((a >> 8) & 0xFF));
  Wire.endTransmission();
}
static void write16x2(uint8_t reg, int16_t a, int16_t b) {
  Wire.beginTransmission(ORC_ADDR_OHSTEM_HUB);
  Wire.write(reg);
  Wire.write((uint8_t)(a & 0xFF)); Wire.write((uint8_t)((a >> 8) & 0xFF));
  Wire.write((uint8_t)(b & 0xFF)); Wire.write((uint8_t)((b >> 8) & 0xFF));
  Wire.endTransmission();
}
// Wire đã được begin() ở setup (SDA/SCL của ESP32#1), ở đây không begin lại.
static bool begin() {
  uint8_t who = 0;
  if (!readReg(REG_WHOAMI, who) || who != ORC_ADDR_OHSTEM_HUB) return false;
  write16x2(REG_MOTOR_INDEX, ALL_MOTORS, 0);        // tắt các cổng motor của hub (không dùng)
  return true;
}
static String firmware() {
  uint8_t mi = 0, ma = 0;
  readReg(REG_FW, mi); readReg(REG_FW + 1, ma);
  return String(ma) + "." + String(mi);
}
static void servo(uint8_t idx, int deg) { write16(REG_SERVO[idx], (int16_t)constrain(deg, 0, 180)); }
}  // namespace hub

bool hubOk = false;
bool gripClosed = false;
int guideSpeedPct = 0;
uint32_t guideStopAtMs = 0;       // 0 = không hẹn giờ
static int lastGripDeg = -1, lastGuideDeg = -1;

void servosBegin() {
  hubOk = hub::begin();
  if (!hubOk) {
    Log.println("[SERVO] Không thấy hub Ohstem driverV2 (0x54)! Servo sẽ không chạy.");
    return;
  }
  Log.printf("[SERVO] Hub OK, firmware %s\n", hub::firmware().c_str());
}

static void gripWrite(int deg) {
  if (!hubOk || deg == lastGripDeg) return;
  hub::servo(hub::S1, deg);
  lastGripDeg = deg;
}
static void guideWrite(int deg) {
  if (!hubOk || deg == lastGuideDeg) return;
  hub::servo(hub::S2, deg);
  lastGuideDeg = deg;
}

void gripSet(bool close) {
  gripClosed = close;
  gripWrite(close ? GRIP_CLOSE_DEG : GRIP_OPEN_DEG);
}
void gripToggle() { gripSet(!gripClosed); }

// Servo 2 (180°) = CỬA NẠP BÓNG vào bánh bắn. guideSet(pct != 0) = bật chế độ nạp, guideSet(0) = tắt (cửa đóng).
// Khi đang nạp: chỉ mở cửa khi bánh bắn ĐÃ ĐỦ rpm (shooterAtSpeed), mở GUIDE_OPEN_MS cho 1 quả qua rồi đóng,
// chờ bánh bắn hồi lại đủ rpm mới mở tiếp. durationMs > 0: tự tắt chế độ nạp sau bấy lâu.
static bool guideGateOpen = false;
static uint32_t guideGateMs = 0;
static bool guideWaitLogged = false;

void guideSet(int pct, uint32_t durationMs = 0) {
  guideSpeedPct = (pct != 0) ? 1 : 0;
  guideStopAtMs = (pct != 0 && durationMs > 0) ? millis() + durationMs : 0;
  if (!guideSpeedPct) { guideGateOpen = false; guideWrite(GUIDE_CLOSED_DEG); }
  guideWaitLogged = false;
}

void servosUpdate() {
  if (guideStopAtMs && (int32_t)(millis() - guideStopAtMs) >= 0) guideSet(0);
  if (!guideSpeedPct) return;
  uint32_t now = millis();
  if (guideGateOpen) {
    if (now - guideGateMs >= GUIDE_OPEN_MS) { guideGateOpen = false; guideGateMs = now; guideWrite(GUIDE_CLOSED_DEG); }
  } else if (now - guideGateMs >= GUIDE_OPEN_MS) {         // đóng ít nhất bấy lâu để rpm kịp tụt khi bóng đi qua
    if (shooterOn && shooterAtSpeed) {
      guideGateOpen = true; guideGateMs = now; guideWrite(GUIDE_OPEN_DEG);
      guideWaitLogged = false;
    } else if (!guideWaitLogged) {
      guideWaitLogged = true;
      Log.println(shooterOn ? "[NẠP] Chờ bánh bắn đủ rpm mới mở cửa" : "[NẠP] Bánh bắn đang tắt (bấm ✕) — cửa nạp giữ đóng");
    }
  }
}

void servosInitialPose() {
  gripSet(!GRIP_OPEN_AT_BOOT);
  guideSet(0);
}
