/*
 * Lift.h — nâng hạ (L298N #4, motor 1), ĐƠN GIẢN, không PID
 *
 * Lái tay (MAN): giữ D-pad ↑ = LIFT_PWM_UP, ↓ = LIFT_PWM_DOWN, thả = LIFT_PWM_HOLD nếu vừa nâng (giữ khỏi tụt), 0 nếu vừa hạ.
 *                Không cần encoder.
 * Tự động (AUTO): chạy về chiều cao đích theo encoder (ESP32#2): xa thì PWM đầy, gần (< 30 mm) thì nửa PWM, tới nơi (< 5 mm) thì giữ.
 *                Quá 5 s chưa tới nơi, hoặc mất encoder => tắt motor (chống cháy khi kẹt / sai chiều encoder).
 */
#pragma once
#include "config.h"
#include "Common.h"
#include "Motors.h"
#include "Encoders.h"

float liftTargetMM = 0;
float liftPosMM = 0;
bool liftFault = false;
int liftPwmOut = 0;
static int32_t liftZeroCounts = 0;
static bool liftHeldUp = false;           // đang ở trên cao => thả nút thì nuôi LIFT_PWM_HOLD
static uint32_t liftMoveSince = 0;

static const float LIFT_TOL_MM = 5.0f, LIFT_SLOW_MM = 30.0f;
static const uint32_t LIFT_AUTO_TIMEOUT_MS = 5000;

// ---- Đếm encoder có dấu ----
int32_t liftCount = 0;
static int32_t liftRawLast = 0;
static bool liftRawInit = false;
static int liftDirSign = 1;

static void liftTrackCount() {
#if LIFT_ENC_SINGLE
  int32_t raw = encLiftCount;
  if (!liftRawInit) { liftRawLast = raw; liftRawInit = true; }
  int32_t d = raw - liftRawLast;
  liftRawLast = raw;
  if (d < 0 || d > 2000) d = 0;
  liftCount += liftDirSign * d;
#else
  liftCount = LIFT_ENC_SIGN * encLiftCount;
#endif
}

void liftBegin() {}

// Coi vị trí hiện tại là 0 mm (gõ 'l')
void liftZeroHere() {
  liftTrackCount();
  liftZeroCounts = liftCount;
  liftTargetMM = 0; liftPosMM = 0;
  liftFault = false; liftHeldUp = false;
  Log.println("[LIFT] Đặt 0 mm tại vị trí hiện tại");
}

void liftSetTarget(float mm) {
  float t = clampf(mm, 0.0f, LIFT_MAX_MM);
  if (fabsf(t - liftTargetMM) > 0.01f) { liftFault = false; liftMoveSince = 0; }
  liftTargetMM = t;
}

bool liftBusy() {
  if (liftFault || !encFresh()) return false;
  return fabsf(liftTargetMM - liftPosMM) > LIFT_TOL_MM;
}

void liftUpdate(float dt) {
  (void)dt;
  liftTrackCount();
  liftPosMM = (float)(liftCount - liftZeroCounts) / LIFT_COUNTS_PER_MM;

  int pwm = 0;
  if (robotMode == RM_AUTO) {
    float err = liftTargetMM - liftPosMM;
    if (!encFresh() || liftFault) {
      pwm = 0;
    } else if (fabsf(err) > LIFT_TOL_MM) {
      float k = fabsf(err) < LIFT_SLOW_MM ? 0.5f : 1.0f;
      pwm = err > 0 ? (int)(k * LIFT_PWM_UP) : -(int)(k * LIFT_PWM_DOWN);
      if (!liftMoveSince) liftMoveSince = millis();
      if (millis() - liftMoveSince > LIFT_AUTO_TIMEOUT_MS) {
        liftFault = true; pwm = 0;
        Log.println("[LIFT] Quá 5 s chưa tới chiều cao đích: kẹt hoặc encoder sai => tắt motor lift");
      }
    } else {
      liftMoveSince = 0;
      pwm = (liftTargetMM > LIFT_TOL_MM) ? LIFT_PWM_HOLD : 0;
    }
    liftHeldUp = liftTargetMM > LIFT_TOL_MM;
  } else {
    if (liftOpenDir > 0)      { pwm = LIFT_PWM_UP;   liftHeldUp = true; }
    else if (liftOpenDir < 0) { pwm = -LIFT_PWM_DOWN; liftHeldUp = false; }
    else                      pwm = liftHeldUp ? LIFT_PWM_HOLD : 0;
    liftTargetMM = liftPosMM;
    liftFault = false;
  }
  liftDirSign = pwm > 0 ? 1 : (pwm < 0 ? -1 : liftDirSign);
  liftPwmOut = pwm;
  motorSet(M_LIFT, LIFT_DIR * pwm);
}
