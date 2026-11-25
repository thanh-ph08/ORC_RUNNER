/*
 * Lift.h — cơ cấu nâng hạ (L298N #4, motor 1) bằng PID vị trí + encoder (ESP32#2)
 *
 * Vị trí 0 mm = vị trí lúc cấp điện (đáy). PID có tích phân + PWM nuôi (LIFT_HOLD_PWM) để GIỮ vị trí chống trọng lực.
 * Thả D-pad thì đích = vị trí hiện tại => lift đứng giữ yên.
 * Bảo vệ: PWM lớn mà không nhúc nhích quá LIFT_STALL_MS, hoặc mất encoder => ngắt motor (liftFault) cho tới khi có lệnh mới.
 */
#pragma once
#include "config.h"
#include "Common.h"
#include "Motors.h"
#include "PidCtrl.h"
#include "Encoders.h"

float liftTargetMM = 0;
float liftPosMM = 0, liftVelMMS = 0;
bool liftFault = false;
int liftPwmOut = 0;
static int32_t liftZeroCounts = 0;
static Pid liftPid;
static uint32_t liftStallSince = 0;

void liftBegin() {
  liftPid.set(LIFT_KP, LIFT_KI, 0.0f, LIFT_I_MAX_PWM);   // D tính riêng theo vận tốc đo (LIFT_KD)
}

// Coi vị trí hiện tại là 0 mm
void liftZeroHere() {
  liftZeroCounts = encLiftCount;
  liftTargetMM = 0; liftPosMM = 0;
  liftPid.reset();
  liftFault = false;
  Log.println("[LIFT] Đặt 0 mm tại vị trí hiện tại");
}

static void liftMeasure() {
  liftPosMM = LIFT_ENC_SIGN * (float)(encLiftCount - liftZeroCounts) / LIFT_COUNTS_PER_MM;
  liftVelMMS = LIFT_ENC_SIGN * (float)encLiftCps / LIFT_COUNTS_PER_MM;
}

void liftSetTarget(float mm) {
  float t = clampf(mm, 0.0f, LIFT_MAX_MM);
  if (fabsf(t - liftTargetMM) > 0.01f) liftFault = false;     // lệnh mới => thử lại
  liftTargetMM = t;
}

// Đi theo hướng dir (+1 lên, -1 xuống) với tốc độ jog; không cho đích chạy quá xa vị trí thật
void liftJog(int dir, float dt) {
  float t = liftTargetMM + dir * LIFT_JOG_MMS * dt;
  t = clampf(t, liftPosMM - LIFT_MAX_LEAD_MM, liftPosMM + LIFT_MAX_LEAD_MM);
  liftSetTarget(t);
}
// Thả nút: giữ đúng vị trí đang đứng
void liftHoldHere() { liftSetTarget(liftPosMM); }

bool liftBusy() {
  if (liftFault || !encFresh()) return false;     // lỗi thì đừng bắt auto chờ vô ích
  return fabsf(liftTargetMM - liftPosMM) > LIFT_TOL_MM || fabsf(liftVelMMS) > 15.0f;
}

void liftUpdate(float dt) {
  liftMeasure();
  if (!encFresh()) {
    liftPid.reset();
    liftPwmOut = 0;
    motorSet(M_LIFT, 0);
    return;
  }
  if (liftFault) { liftPid.reset(); liftPwmOut = 0; motorSet(M_LIFT, 0); return; }

  float err = liftTargetMM - liftPosMM;
  float hold = (liftTargetMM > 5.0f) ? LIFT_HOLD_PWM : 0.0f;
  // D theo vận tốc đo (không theo sai số) để lệnh nhảy không bị giật
  float pwm = liftPid.update(err, dt, false) + hold - LIFT_KD * liftVelMMS;
  pwm = clampf(pwm, -(float)LIFT_PWM_MAX_DOWN, (float)LIFT_PWM_MAX_UP);

  // sát đáy, đích ~0: không ép motor vào cữ chặn
  if (liftTargetMM < 2.0f && liftPosMM < 6.0f) { pwm = fminf(pwm, 0.0f); if (liftPosMM < 2.0f) pwm = 0; liftPid.integ = 0; }
  liftPwmOut = (int)pwm;
  motorSet(M_LIFT, LIFT_DIR * liftPwmOut);

  // chống kẹt
  if (fabsf(pwm) > 0.7f * (pwm > 0 ? LIFT_PWM_MAX_UP : LIFT_PWM_MAX_DOWN) && fabsf(liftVelMMS) < 3.0f) {
    if (!liftStallSince) liftStallSince = millis();
    if (millis() - liftStallSince > LIFT_STALL_MS) {
      liftFault = true;
      motorSet(M_LIFT, 0);
      Log.println("[LIFT] Kẹt / sai chiều encoder (LIFT_ENC_SIGN, LIFT_DIR) => ngắt motor. Ra lệnh mới để thử lại");
    }
  } else liftStallSince = 0;
}
