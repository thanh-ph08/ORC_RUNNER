/*
 * Intake.h — motor hút bóng (L298N #3, motor 1). Bật/tắt bằng tam giác; giữ R1 để đảo chiều xả kẹt.
 */
#pragma once
#include "config.h"
#include "Common.h"
#include "Motors.h"

bool intakeEnabled = false;      // trạng thái bật/tắt (do nút / auto)
bool intakeReverse = false;      // đảo chiều tạm thời

void intakeApply() {
  int dir = intakeReverse ? -1 : (intakeEnabled ? 1 : 0);
  motorSet(M_INTAKE, dir * INTAKE_PWM);
}
void intakeSetEnabled(bool on) { intakeEnabled = on; intakeApply(); }
void intakeToggle() { intakeSetEnabled(!intakeEnabled); }
void intakeSetReverse(bool rev) { if (rev != intakeReverse) { intakeReverse = rev; intakeApply(); } }
// Dùng cho auto: dir = 1 bật, 0 tắt, -1 đảo chiều (giữ đến khi lệnh khác)
void intakeCommand(int dir) {
  intakeReverse = (dir < 0);
  intakeEnabled = (dir > 0);
  intakeApply();
}
