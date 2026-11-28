/*
 * Shooter.h — bánh bắn banh (L298N #3, motor 2) giữ tốc độ bằng PID + encoder (ESP32#2)
 *
 * pwm = KFF * rpm_lệnh + PID(rpm_lệnh - rpm_đo)
 * Bảo vệ: mất encoder, hoặc PWM gần trần mà tốc độ đo được vẫn ~0 (sai chiều encoder / đứt dây) => tắt motor.
 */
#pragma once
#include "config.h"
#include "Common.h"
#include "Motors.h"
#include "PidCtrl.h"
#include "Encoders.h"

bool shooterOn = false;
float shooterTargetRpm = SHOOTER_RPM_DEFAULT;   // tốc độ muốn bắn khi bật
float shooterCmdRpm = 0;                        // lệnh sau bộ dốc
float shooterRpm = 0;                           // tốc độ đo được
bool shooterAtSpeed = false;
bool shooterFault = false;
int shooterPwmOut = 0;
static Pid shooterPid;
static uint32_t shooterInBandSince = 0, shooterSatSince = 0;

void shooterBegin() {
  shooterPid.set(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_I_MAX_PWM);
}

void shooterSetRpm(float rpm) { shooterTargetRpm = clampf(rpm, SHOOTER_RPM_MIN, SHOOTER_RPM_MAX); }
void shooterAdjust(float deltaRpm) { shooterSetRpm(shooterTargetRpm + deltaRpm); }
void shooterEnable(bool on) {
  shooterOn = on;
  if (on) { shooterFault = false; shooterSatSince = 0; }
}
void shooterToggle() { shooterEnable(!shooterOn); }

void shooterUpdate(float dt) {
  shooterRpm = SHOOTER_ENC_SIGN * (float)encShooterCps / SHOOTER_CPR * 60.0f;

  float want = (shooterOn && !shooterFault) ? shooterTargetRpm : 0.0f;
  float step = SHOOTER_RAMP_RPM_S * dt;
  if (shooterCmdRpm < want) shooterCmdRpm = fminf(shooterCmdRpm + step, want);
  else shooterCmdRpm = fmaxf(shooterCmdRpm - step, want);

  if (pidOff) {                                   // KHÔNG PID: PWM = KFF × rpm, không cần encoder
    shooterFault = false;
    shooterPid.reset();
    shooterPwmOut = (shooterCmdRpm < 1.0f) ? 0 : (int)clampf(SHOOTER_KFF * shooterCmdRpm, 0.0f, (float)PWM_MAX);
    shooterAtSpeed = shooterOn && fabsf(shooterCmdRpm - shooterTargetRpm) < 1.0f;
    motorSet(M_SHOOTER, SHOOTER_DIR * shooterPwmOut);
    return;
  }
  if (!encFresh() || shooterCmdRpm < 1.0f) {
    if (!encFresh() && shooterOn && !shooterFault) {
      shooterFault = true;
      shooterOn = false;
      Log.println("[SHOOTER] Mất encoder (ESP32#2) => tắt bắn");
    }
    shooterPid.reset();
    shooterPwmOut = 0;
    shooterAtSpeed = false;
    motorSet(M_SHOOTER, 0);
    return;
  }

  float raw = SHOOTER_KFF * shooterCmdRpm + shooterPid.update(shooterCmdRpm - shooterRpm, dt, false);
  float pwm = clampf(raw, 0.0f, (float)PWM_MAX);
  shooterPwmOut = (int)pwm;
  motorSet(M_SHOOTER, SHOOTER_DIR * shooterPwmOut);

  // phát hiện sai dây / sai chiều: PWM gần trần mà gần như không quay
  if (pwm > 0.9f * PWM_MAX && fabsf(shooterRpm) < 0.1f * shooterCmdRpm) {
    if (!shooterSatSince) shooterSatSince = millis();
    if (millis() - shooterSatSince > 1500) {
      shooterFault = true;
      shooterOn = false;
      Log.println("[SHOOTER] PWM tối đa mà không quay: kiểm tra encoder (SHOOTER_ENC_SIGN) / dây motor => tắt bắn");
    }
  } else shooterSatSince = 0;

  float tol = fmaxf(SHOOTER_TOL_RPM, 0.04f * shooterTargetRpm);
  bool inBand = shooterOn && fabsf(shooterTargetRpm - shooterRpm) < tol && fabsf(shooterCmdRpm - shooterTargetRpm) < 1.0f;
  if (inBand) {
    if (!shooterInBandSince) shooterInBandSince = millis();
    shooterAtSpeed = (millis() - shooterInBandSince) >= SHOOTER_AT_SPEED_MS;
  } else {
    shooterInBandSince = 0;
    shooterAtSpeed = false;
  }
}
