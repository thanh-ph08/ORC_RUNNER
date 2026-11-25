/*
 * Manual.h — CHẠY THỦ CÔNG bằng tay cầm PS4/PS5
 *
 *   Cần trái ........ chạy tới/lùi/ngang        Cần phải (ngang) .. xoay        Giữ L2 .. chạy chậm
 *   D-pad ↑ / ↓ ..... nâng / hạ lift (giữ), thả nút thì lift đứng giữ vị trí (PID)
 *   D-pad ← / → ..... giảm / tăng tốc độ bắn (± SHOOTER_RPM_STEP)
 *   △ Triangle ...... bật/tắt INTAKE            Giữ R1 ........ đảo chiều intake (xả kẹt)
 *   ✕ Cross ......... bật/tắt BẮN (PID tốc độ)
 *   ◯ Circle ........ DẪN bóng (servo 2, giữ để quay)
 *   □ Square ........ GẮP bóng (servo 1, bấm đổi gắp/nhả)
 *   L3 .............. bật/tắt lái theo sân     R3 ............ bật/tắt giữ hướng bằng yaw
 *   Share ........... đặt toạ độ về điểm xuất phát
 *   Giữ L1+R1 rồi bấm Options ... CHẠY TỰ ĐỘNG     PS / Share / gạt mạnh cần ... huỷ tự động
 */
#pragma once
#include "config.h"
#include "Common.h"
#include "Pad.h"
#include "Odometry.h"
#include "Drive.h"
#include "Intake.h"
#include "Shooter.h"
#include "Lift.h"
#include "Servos.h"
#include "Auto.h"

bool fieldCentric = false, headingHold = HEADING_HOLD_DEFAULT;
float holdTarget = START_TH_RAD;
uint32_t lastTurnMs = 0;
static uint32_t padLostSince = 0;
static bool prevLiftBtn = false;

void failsafeStopMechanisms() {
  intakeCommand(0);
  shooterEnable(false);
  guideSet(0);
}

void manualUpdate(float dt) {
  if (!padNow.ok) {
    stopDrive();
    if (!padLostSince) padLostSince = millis();
    if (millis() - padLostSince > 300) failsafeStopMechanisms();
    return;
  }
  padLostSince = 0;

  if (padNow.l1 && padNow.r1 && PRESSED(start)) { startAuto(); return; }
  if (PRESSED(thumbL)) { fieldCentric = !fieldCentric; Serial.printf("[MAN] Lái theo sân: %s\n", fieldCentric ? "BẬT" : "TẮT"); }
  if (PRESSED(thumbR)) { headingHold = !headingHold; holdTarget = poseTh; Serial.printf("[MAN] Giữ hướng: %s\n", headingHold ? "BẬT" : "TẮT"); }
  if (PRESSED(share)) resetPoseToStart();

  // ---- Di chuyển ----
  float sp = padNow.l2 ? MANUAL_SLOW : 1.0f;
  float stickF = -padNow.ly, stickL = -padNow.lx, turn = -padNow.rx;
  float vx, vy;
  if (fieldCentric) {
    float a = (DRIVER_VIEW_DEG - 90.0f) * DEG_TO_RAD;
    float sxR = -stickL, syU = stickF;
    float fx = cosf(a) * sxR - sinf(a) * syU;
    float fy = sinf(a) * sxR + cosf(a) * syU;
    float c = cosf(poseTh), s = sinf(poseTh);
    vx = (c * fx + s * fy) * MANUAL_MAX_MMS * sp;
    vy = (-s * fx + c * fy) * MANUAL_MAX_MMS * sp;
  } else {
    vx = stickF * MANUAL_MAX_MMS * sp;
    vy = stickL * MANUAL_MAX_MMS * sp;
  }
  float w;
  if (turn != 0) {
    w = turn * MANUAL_MAX_W * sp;
    lastTurnMs = millis();
    holdTarget = poseTh;
  } else if (headingHold && poseFresh()) {
    if (millis() - lastTurnMs < 250) holdTarget = poseTh;
    float hasMove = (vx != 0 || vy != 0) ? 1.0f : 0.0f;
    w = hasMove * clampf(HOLD_KP * wrapPi(holdTarget - poseTh), -MANUAL_MAX_W, MANUAL_MAX_W);
  } else {
    w = 0;
  }
  if (vx == 0 && vy == 0 && w == 0) stopDrive();
  else driveRobot(vx, vy, w);

  // ---- Intake ----
  if (PRESSED(triangle)) intakeToggle();
  if (INTAKE_REVERSE_WITH_R1) intakeSetReverse(padNow.r1 && !padNow.l1);

  // ---- Shooter ----
  if (PRESSED(cross)) { shooterToggle(); Serial.printf("[SHOOT] %s, mục tiêu %.0f rpm\n", shooterOn ? "BẬT" : "TẮT", shooterTargetRpm); }
  if (PRESSED(left))  { shooterAdjust(-SHOOTER_RPM_STEP); Serial.printf("[SHOOT] mục tiêu %.0f rpm\n", shooterTargetRpm); }
  if (PRESSED(right)) { shooterAdjust(+SHOOTER_RPM_STEP); Serial.printf("[SHOOT] mục tiêu %.0f rpm\n", shooterTargetRpm); }

  // ---- Servo ----
  if (PRESSED(square)) gripToggle();
  if (GUIDE_HOLD_TO_RUN) {
    if (PRESSED(circle)) guideSet(GUIDE_SPEED_PCT);
    if (!padNow.circle && padPrev.circle) guideSet(0);
  } else if (PRESSED(circle)) {
    guideSet(guideSpeedPct != 0 ? 0 : GUIDE_SPEED_PCT);
  }

  // ---- Lift: giữ D-pad thì đi, thả thì đứng giữ ----
  bool liftBtn = padNow.up != padNow.down;
  if (padNow.up && !padNow.down) liftJog(+1, dt);
  else if (padNow.down && !padNow.up) liftJog(-1, dt);
  else if (prevLiftBtn && !liftBtn) liftHoldHere();
  prevLiftBtn = liftBtn;
}
