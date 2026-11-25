/*
 * Drive.h — di chuyển 4 bánh mecanum (4 motor qua L298N #1, #2)
 * Xe không có encoder bánh: dùng feedforward PWM = KS + KV*|v|; vị trí lấy từ Pico (chuột optical flow + BMI160).
 * Hệ robot: x = phía trước, y = bên trái, w ngược chiều kim đồng hồ.
 */
#pragma once
#include "config.h"
#include "Common.h"
#include "Motors.h"
#include "Odometry.h"

static float battV = 0;

float batteryComp() {
  if (BATT_PIN < 0) return 1.0f;
  float v = analogReadMilliVolts(BATT_PIN) / 1000.0f * BATT_DIVIDER;
  battV = (battV < 1.0f) ? v : battV * 0.95f + v * 0.05f;
  if (battV < 5.0f) return 1.0f;
  return clampf(BATT_NOMINAL_V / battV, 0.8f, 1.3f);
}

void stopDrive() {
  for (int i = M_FL; i <= M_RR; i++) motorSet(i, 0);
}

// Lệnh vận tốc hệ ROBOT: vx tới (mm/s), vy sang trái (mm/s), w ngược chiều kim đồng hồ (rad/s)
void driveRobot(float vx, float vy, float w) {
  const float k = WHEEL_LX_MM + WHEEL_LY_MM;
  float v[4];
  v[M_FL] = vx - vy - k * w;
  v[M_FR] = vx + vy + k * w;
  v[M_RL] = vx + vy - k * w;
  v[M_RR] = vx - vy + k * w;
  float mx = 0;
  for (int i = 0; i < 4; i++) mx = fmaxf(mx, fabsf(v[i]));
  if (mx > WHEEL_MAX_MMS) {                           // co cả vector để không lệch hướng
    float s = WHEEL_MAX_MMS / mx;
    for (int i = 0; i < 4; i++) v[i] *= s;
  }
  float comp = batteryComp();
  for (int i = 0; i < 4; i++) {
    float a = fabsf(v[i]);
    int pwm = (a < 2.0f) ? 0 : (int)((KS_PWM[i] + KV_PWM_PER_MMS[i] * a) * comp);
    motorSet(i, v[i] >= 0 ? pwm : -pwm);
  }
}

// Lệnh vận tốc hệ SÂN (dùng yaw từ Pico)
void driveField(float vx, float vy, float w) {
  float c = cosf(poseTh), s = sinf(poseTh);
  driveRobot(c * vx + s * vy, -s * vx + c * vy, w);
}

// ---- Công tắc chạm (nếu có) ----
void touchBegin() {
  for (int s = 0; s < 4; s++)
    for (int k = 0; k < 2; k++) {
      int p = TOUCH_PIN[s][k];
      if (p >= 0) pinMode(p, (p >= 34) ? INPUT : INPUT_PULLUP);
    }
}
static bool touchPinActive(int p) {
  if (p < 0) return false;
  return digitalRead(p) == (TOUCH_ACTIVE_LOW ? LOW : HIGH);
}
int touchCount(int side) { return (TOUCH_PIN[side][0] >= 0) + (TOUCH_PIN[side][1] >= 0); }
int touchPressed(int side) { return touchPinActive(TOUCH_PIN[side][0]) + touchPinActive(TOUCH_PIN[side][1]); }
