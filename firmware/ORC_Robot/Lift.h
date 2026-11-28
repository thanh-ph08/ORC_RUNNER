/*
 * Lift.h — cơ cấu nâng hạ (L298N #4, motor 1) bằng PID vị trí + encoder (ESP32#2)
 *
 * Vị trí 0 mm = vị trí lúc cấp điện (đáy). PID có tích phân + PWM nuôi (LIFT_HOLD_PWM) để GIỮ vị trí chống trọng lực.
 * Thả D-pad thì đích = vị trí hiện tại => lift đứng giữ yên.
 * Encoder 1 kênh (LM393, LIFT_ENC_SINGLE 1): chiều lấy theo lệnh motor, nên PWM nuôi LIFT_HOLD_PWM phải đủ giữ lift không tự trôi xuống.
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

// Số đếm có dấu của lift. Encoder 2 kênh: lấy thẳng từ ESP32#2. Encoder 1 kênh (LIFT_ENC_SINGLE): ESP32#2 chỉ đếm cạnh
// (luôn tăng), nên cộng/trừ theo chiều của lệnh motor gần nhất khác 0.
int32_t liftCount = 0;
static int32_t liftRawLast = 0;
static bool liftRawInit = false;
static int liftDirSign = 1;          // +1 = đang lệnh đi lên, -1 = đi xuống

static void liftTrackCount() {
#if LIFT_ENC_SINGLE
  int32_t raw = encLiftCount;
  if (!liftRawInit) { liftRawLast = raw; liftRawInit = true; }
  int32_t d = raw - liftRawLast;
  liftRawLast = raw;
  if (d < 0 || d > 2000) d = 0;      // ESP32#2 vừa khởi động lại / số vô lý: bỏ qua, không làm nhảy vị trí
  liftCount += liftDirSign * d;
#else
  liftCount = LIFT_ENC_SIGN * encLiftCount;
#endif
}

void liftBegin() {
  liftPid.set(LIFT_KP, LIFT_KI, 0.0f, LIFT_I_MAX_PWM);   // D tính riêng theo vận tốc đo (LIFT_KD)
}

// Coi vị trí hiện tại là 0 mm
void liftZeroHere() {
  liftTrackCount();
  liftZeroCounts = liftCount;
  liftTargetMM = 0; liftPosMM = 0;
  liftPid.reset();
  liftFault = false;
  Log.println("[LIFT] Đặt 0 mm tại vị trí hiện tại");
}

static void liftMeasure() {
  liftTrackCount();
  liftPosMM = (float)(liftCount - liftZeroCounts) / LIFT_COUNTS_PER_MM;
#if LIFT_ENC_SINGLE
  liftVelMMS = liftDirSign * fabsf((float)encLiftCps) / LIFT_COUNTS_PER_MM;
#else
  liftVelMMS = LIFT_ENC_SIGN * (float)encLiftCps / LIFT_COUNTS_PER_MM;
#endif
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

// Thử motor lift TRỰC TIẾP (bỏ qua encoder, PID, chống kẹt): lệnh Serial 'u' = lên, 'n' = xuống, 0.4 s
static int liftTestPwm = 0;
static uint32_t liftTestUntil = 0;
static int32_t liftTestStartCount = 0;
void liftTest(int dir) {
  liftTestPwm = dir > 0 ? LIFT_PWM_MAX_UP : -LIFT_PWM_MAX_DOWN;
  liftTestUntil = millis() + 400;
  liftTestStartCount = liftCount;
  Log.printf("[LIFT] THỬ motor %s, PWM %d trong 0.4 s (bỏ qua encoder). Encoder ESP32#2: %s\n",
             dir > 0 ? "LÊN" : "XUỐNG", liftTestPwm, encFresh() ? "OK" : "MẤT");
}

void liftUpdate(float dt) {
  liftMeasure();
  if (liftTestUntil) {
    if ((int32_t)(millis() - liftTestUntil) < 0) { motorSet(M_LIFT, LIFT_DIR * liftTestPwm); return; }
    motorSet(M_LIFT, 0);
    liftTestUntil = 0;
    Log.printf("[LIFT] Thử xong: encoder đếm được %ld count (%.1f mm). 0 count = encoder không nhận xung; số âm khi thử LÊN = sai chiều\n",
               (long)(liftCount - liftTestStartCount), (liftCount - liftTestStartCount) / LIFT_COUNTS_PER_MM);
    liftFault = false; liftPid.reset(); liftTargetMM = liftPosMM;
  }
  if (!encFresh()) {
    static uint32_t warnMs = 0;
    if (liftTargetMM != liftPosMM && millis() - warnMs > 2000) { warnMs = millis(); Log.println("[LIFT] Không chạy: mất encoder ESP32#2 (0x42). Lift chỉ chạy khi có encoder"); }
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
  if (liftPwmOut > 0) liftDirSign = 1; else if (liftPwmOut < 0) liftDirSign = -1;   // chiều đếm cho encoder 1 kênh
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
