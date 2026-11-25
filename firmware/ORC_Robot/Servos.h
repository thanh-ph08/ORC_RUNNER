/*
 * Servos.h — 2 servo qua hub Ohstem driverV2 (thư viện OhStemRobotics, I2C 0x54)
 *   Servo 1 (S1): GẮP bóng   — servo góc, mở / đóng
 *   Servo 2 (S2): DẪN bóng   — servo xoay liên tục 360°, tốc độ -100..100 %
 */
#pragma once
#include <Wire.h>
#include <OhStemRobotics.h>
#include "config.h"
#include "orc_protocol.h"
#include "Common.h"

ohstem::robotics::MotorDriverV2 ohstemHub(Wire, ORC_ADDR_OHSTEM_HUB);
ohstem::robotics::RobotServo gripServo(ohstemHub, ohstem::robotics::S1, 180);
ohstem::robotics::RobotServo guideServo(ohstemHub, ohstem::robotics::S2, 180);

bool hubOk = false;
bool gripClosed = false;
int guideSpeedPct = 0;
uint32_t guideStopAtMs = 0;       // 0 = không hẹn giờ
static int lastGripDeg = -1, lastGuideDeg = -1;

void servosBegin() {
  hubOk = ohstemHub.begin();
  if (!hubOk) {
    Serial.println("[SERVO] Không thấy hub Ohstem driverV2 (0x54)! Servo sẽ không chạy.");
    return;
  }
  Serial.printf("[SERVO] Hub OK, firmware %s\n", ohstemHub.firmwareVersion().c_str());
  gripServo.begin();
  guideServo.begin();
}

static void gripWrite(int deg) {
  if (!hubOk || deg == lastGripDeg) return;
  gripServo.setAngle(deg);
  lastGripDeg = deg;
}
static void guideWrite(int deg) {
  if (!hubOk || deg == lastGuideDeg) return;
  guideServo.setAngle(deg);
  lastGuideDeg = deg;
}

void gripSet(bool close) {
  gripClosed = close;
  gripWrite(close ? GRIP_CLOSE_DEG : GRIP_OPEN_DEG);
}
void gripToggle() { gripSet(!gripClosed); }

// Servo 360°: 90° = đứng yên, dưới 90° quay một chiều, trên 90° quay chiều kia (cùng quy ước RobotServo::spin)
// pct: -100..100. durationMs > 0: tự dừng sau bấy lâu.
void guideSet(int pct, uint32_t durationMs = 0) {
  pct = constrain(pct, -100, 100);
  guideSpeedPct = pct;
  guideStopAtMs = (pct != 0 && durationMs > 0) ? millis() + durationMs : 0;
  int deg = (pct == 0) ? 90 + GUIDE_STOP_TRIM_DEG : (int)lroundf(90.0f + GUIDE_STOP_TRIM_DEG - pct * 0.9f);
  guideWrite(constrain(deg, 0, 180));
}

void servosUpdate() {
  if (guideStopAtMs && (int32_t)(millis() - guideStopAtMs) >= 0) guideSet(0);
}

void servosInitialPose() {
  gripSet(!GRIP_OPEN_AT_BOOT);
  guideSet(0);
}
