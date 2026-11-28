/*
 * Auto.h — CHẠY TỰ ĐỘNG theo path_data.h (tạo bởi app ORC Path Planner)
 *
 * Mỗi bước trong STEPS[]:
 *   FOLLOW  : bám quỹ đạo TRAJ[a..b] = feedforward vận tốc + PID vị trí x, y, θ (toạ độ từ RPi)
 *   LIFT    : đưa lift tới p1 mm (PID encoder), chờ hoặc chạy song song
 *   WAIT    : chờ p1 ms
 *   WALL    : chạy chậm vào tường (công tắc chạm hoặc theo thời gian) => đặt lại toạ độ + góc
 *   INTAKE  : p1 = 1 bật, 0 tắt, -1 đảo chiều
 *   SHOOTER : p1 = rpm (0 = tắt); không async thì chờ tới khi đủ tốc độ
 *   GRIP    : p1 = 1 gắp (đóng), 0 nhả (mở)
 *   GUIDE   : nạp bóng (servo 2 là cửa), p1 ≠ 0 = bật nạp, 0 = tắt; p2 = thời gian nạp ms (0 = tới khi tắt).
 *             Cửa chỉ mở khi bánh bắn đủ rpm, mỗi lần 1 quả => nhớ bật bánh bắn (bước SHOOTER) trước.
 *   CUSTOM  : gọi runCustomAction(id, tham số) — tự viết ở cuối file này
 *
 * Không có encoder bánh nên quỹ đạo có "đồng hồ co giãn": robot tụt lại xa điểm đích
 * thì đồng hồ chạy chậm lại / dừng, chờ robot bắt kịp (không bị cắt góc).
 * Mất tín hiệu RPi (toạ độ) => dừng ngay.
 */
#pragma once
#include "path_data.h"
#include "config.h"
#include "Common.h"
#include "Odometry.h"
#include "Drive.h"
#include "Pad.h"
#include "Intake.h"
#include "Shooter.h"
#include "Lift.h"
#include "Servos.h"

// ============================ THAM SỐ PID BÁM PATH ============================
const float AUTO_KP_XY = 3.0f;      // (mm/s) cho mỗi mm sai số
const float AUTO_KI_XY = 0.8f;
const float AUTO_KD_XY = 0.0f;
const float AUTO_I_MAX_MMS = 150.0f;
const float AUTO_KP_TH = 4.0f;      // (rad/s) cho mỗi rad sai số
const float AUTO_KI_TH = 0.5f;
const float AUTO_I_MAX_W = 0.6f;
const float AUTO_I_ZONE_MM = 40.0f;
const float AUTO_VMAX_MMS = 1000.0f;
const float FF_LEAD_S = 0.08f;      // bù trễ motor
const float AUTO_WMAX = 5.0f;

const float SLOW_START_MM = 40.0f, SLOW_FULL_MM = 120.0f;
const float END_TOL_MM = 12.0f, END_TOL_RAD = 2.0f * DEG_TO_RAD;
const uint32_t END_TIMEOUT_MS = 1500;
const float APPROACH_TOL_MM = 25.0f, APPROACH_TOL_RAD = 4.0f * DEG_TO_RAD;
const float APPROACH_VMAX_MMS = 350.0f;
const uint32_t APPROACH_TIMEOUT_MS = 3000;
const float WALL_SPEED_MMS = 180.0f;
const uint32_t WALL_PUSH_MS = 300;
const uint32_t LIFT_WAIT_TIMEOUT_MS = 6000;
const uint32_t SHOOTER_WAIT_TIMEOUT_MS = 4000;
// Giữ chỗ khi đứng chờ (WAIT / nâng lift / chờ bắn / gắp / hành động riêng): bị xe khác đẩy lệch thì tự chạy về
const bool  HOLD_ENABLE = true;
const float HOLD_START_MM = 20.0f, HOLD_START_RAD = 3.0f * DEG_TO_RAD;   // lệch quá mức này thì bắt đầu kéo về
const float HOLD_STOP_MM = 8.0f,   HOLD_STOP_RAD = 1.5f * DEG_TO_RAD;    // về tới trong mức này thì đứng yên (trễ để không rung)
const float HOLD_VMAX_MMS = 400.0f;
const bool AUTO_RESET_POSE_ON_START = true;   // bấm chạy tự động => đặt toạ độ = điểm xuất phát

// ============================ TRẠNG THÁI =============================
enum AutoPhase { AP_IDLE, AP_APPROACH, AP_FOLLOW, AP_SETTLE, AP_WAIT, AP_LIFT, AP_WALL_DRIVE, AP_WALL_PUSH,
                 AP_SHOOTER, AP_TIMER, AP_CUSTOM, AP_DONE };
const char *PHASE_NAME[] = {"idle", "approach", "follow", "settle", "wait", "lift", "wall", "push",
                            "shooter", "timer", "custom", "done"};

AutoPhase autoPhase = AP_IDLE;
int autoStep = 0;
uint32_t phaseStartMs = 0, autoStartMs = 0, timerMs = 0;
float trajIdx = 0;
bool lastFollowContinue = false;
int32_t lastFollowEnd = -1;
float iX = 0, iY = 0, iTh = 0, prevEx = 0, prevEy = 0;
float wallTargetTh = 0;
int asyncCustomId = -1; float asyncCustomParam = 0;
bool holdActive = false;                    // đang kéo về chỗ cũ (giữ chỗ khi đứng chờ)

bool runCustomAction(int id, float param, bool start);
void beginStep();

static char statusBuf[72];
const char *autoStatus() {
  if (robotMode != RM_AUTO) return "";
  snprintf(statusBuf, sizeof(statusBuf), "bước %d/%d %s%s idx=%.0f", autoStep, STEP_COUNT, PHASE_NAME[autoPhase], holdActive ? " (về chỗ)" : "", trajIdx);
  return statusBuf;
}

void setPhase(int p) { autoPhase = (AutoPhase)p; phaseStartMs = millis(); }
uint32_t phaseMs() { return millis() - phaseStartMs; }
void resetPid() { iX = iY = iTh = 0; prevEx = prevEy = 0; }

// Góc pháp tuyến hướng RA NGOÀI sân của từng tường: 0 trái(x=0), 1 phải(x=W), 2 dưới(y=0), 3 trên(y=H)
float wallNormal(int wall) {
  switch (wall) { case 0: return PI; case 1: return 0; case 2: return -HALF_PI; default: return HALF_PI; }
}
// Hướng của từng mặt robot trong hệ robot: 0 trước, 1 trái, 2 sau, 3 phải
float sideAngle(int side) {
  switch (side) { case 0: return 0; case 1: return HALF_PI; case 2: return PI; default: return -HALF_PI; }
}
void setCoordFromWall(int wall, float perp) {
  float x = poseX, y = poseY;
  switch (wall) {
    case 0: x = perp; break;
    case 1: x = FIELD_W_MM - perp; break;
    case 2: y = perp; break;
    default: y = FIELD_H_MM - perp; break;
  }
  setPose(x, y, poseTh);
}

void trajAt(float fi, TrajPt &out) {
  int i0 = (int)floorf(fi);
  if (i0 < 0) i0 = 0;
  if (i0 >= TRAJ_COUNT - 1) { out = TRAJ[TRAJ_COUNT - 1]; return; }
  float g = fi - i0;
  const TrajPt &A = TRAJ[i0], &B = TRAJ[i0 + 1];
  out.x = A.x + g * (B.x - A.x);   out.y = A.y + g * (B.y - A.y);
  out.th = A.th + g * (B.th - A.th);
  out.vx = A.vx + g * (B.vx - A.vx); out.vy = A.vy + g * (B.vy - A.vy); out.w = A.w + g * (B.w - A.w);
}

void poseControl(float tx, float ty, float tth, float ffx, float ffy, float ffw, float dt, float vlimit) {
  float ex = tx - poseX, ey = ty - poseY, eth = wrapPi(tth - poseTh);
  if (AUTO_KI_XY > 0 && hypotf(ex, ey) < AUTO_I_ZONE_MM) {
    iX = clampf(iX + ex * dt, -AUTO_I_MAX_MMS / AUTO_KI_XY, AUTO_I_MAX_MMS / AUTO_KI_XY);
    iY = clampf(iY + ey * dt, -AUTO_I_MAX_MMS / AUTO_KI_XY, AUTO_I_MAX_MMS / AUTO_KI_XY);
  }
  if (AUTO_KI_TH > 0) iTh = clampf(iTh + eth * dt, -AUTO_I_MAX_W / AUTO_KI_TH, AUTO_I_MAX_W / AUTO_KI_TH);
  float dEx = (ex - prevEx) / dt, dEy = (ey - prevEy) / dt;
  prevEx = ex; prevEy = ey;
  float vx = ffx + AUTO_KP_XY * ex + AUTO_KI_XY * iX + AUTO_KD_XY * dEx;
  float vy = ffy + AUTO_KP_XY * ey + AUTO_KI_XY * iY + AUTO_KD_XY * dEy;
  float w = ffw + AUTO_KP_TH * eth + AUTO_KI_TH * iTh;
  float sp = hypotf(vx, vy);
  if (sp > vlimit) { vx *= vlimit / sp; vy *= vlimit / sp; }
  w = clampf(w, -AUTO_WMAX, AUTO_WMAX);
  driveField(vx, vy, w);
}

float holdX = 0, holdY = 0, holdTh = 0;    // chỗ cần giữ khi đứng chờ
uint32_t holdPushCount = 0;

// Ghi nhớ chỗ đứng hiện tại làm điểm giữ
void holdSetHere() { holdX = poseX; holdY = poseY; holdTh = poseTh; holdActive = false; }
void holdSet(float x, float y, float th) { holdX = x; holdY = y; holdTh = th; holdActive = false; }

// Gọi mỗi chu kỳ thay cho stopDrive() khi đứng chờ
void holdPosition(float dt) {
  if (!HOLD_ENABLE) { stopDrive(); return; }
  float e = hypotf(holdX - poseX, holdY - poseY), eth = fabsf(wrapPi(holdTh - poseTh));
  if (!holdActive && (e > HOLD_START_MM || eth > HOLD_START_RAD)) {
    holdActive = true; holdPushCount++;
    resetPid();
    Log.printf("[AUTO] Bị đẩy lệch %.0f mm / %.1f° -> chạy về chỗ cũ\n", e, eth * RAD_TO_DEG);
  }
  if (holdActive && e < HOLD_STOP_MM && eth < HOLD_STOP_RAD) { holdActive = false; resetPid(); }
  if (holdActive) poseControl(holdX, holdY, holdTh, 0, 0, 0, dt, HOLD_VMAX_MMS);
  else stopDrive();
}

// ============================ ĐIỀU KHIỂN =============================
void startAuto() {
  if (STEP_COUNT == 0) { Log.println("[AUTO] path_data.h trống"); return; }
  if (pidOff) { Log.println("[AUTO] Không bắt đầu: đang ở chế độ KHÔNG PID, bấm Options để bật lại PID"); return; }
  if (!poseFresh()) { Log.println("[AUTO] Không bắt đầu: chưa có toạ độ từ RPi"); return; }
  if (poseFlags & POSE_FLAG_CALIBRATING) { Log.println("[AUTO] Không bắt đầu: RPi đang cân gyro, giữ yên xe"); return; }
  if (AUTO_RESET_POSE_ON_START) setPose(START_X_MM, START_Y_MM, START_TH_RAD);
  robotMode = RM_AUTO;
  autoStep = 0;
  lastFollowContinue = false; lastFollowEnd = -1;
  asyncCustomId = -1;
  autoStartMs = millis();
  resetPid();
  holdSetHere();
  setPadColor(0, 0, 255);
  Log.printf("[AUTO] BẮT ĐẦU path \"%s\"\n", PATH_NAME);
  beginStep();
}

void stopAuto(const char *why) {
  stopDrive();
  robotMode = RM_MANUAL;
  setPhase(AP_IDLE);
  asyncCustomId = -1;
  liftSetTarget(liftPosMM);          // giữ nguyên chiều cao đang đứng
  setPadColor(0, 255, 0);
  Log.printf("[AUTO] DỪNG: %s (%.1f s)\n", why, (millis() - autoStartMs) / 1000.0f);
}

void nextStep() { autoStep++; beginStep(); }

void beginStep() {
  if (autoStep >= STEP_COUNT) {
    stopDrive();
    setPhase(AP_DONE);
    Log.printf("[AUTO] HOÀN THÀNH sau %.1f s\n", (millis() - autoStartMs) / 1000.0f);
    return;
  }
  const AutoStep &s = STEPS[autoStep];
  const bool async = (s.flags & SF_ASYNC) != 0;
  switch (s.type) {
    case ST_FOLLOW: {
      bool contiguous = lastFollowContinue && s.a == lastFollowEnd;
      lastFollowContinue = false;
      if (contiguous) {
        if (trajIdx < s.a) trajIdx = s.a;
        setPhase(AP_FOLLOW);
      } else {
        trajIdx = s.a;
        resetPid();
        const TrajPt &p = TRAJ[s.a];
        bool far = hypotf(p.x - poseX, p.y - poseY) > APPROACH_TOL_MM || fabsf(wrapPi(p.th - poseTh)) > APPROACH_TOL_RAD;
        setPhase(far ? AP_APPROACH : AP_FOLLOW);
      }
      break;
    }
    case ST_LIFT:
      liftSetTarget(s.p1);
      if (async) { nextStep(); return; }
      stopDrive();
      setPhase(AP_LIFT);
      break;
    case ST_WAIT:
      stopDrive();
      setPhase(AP_WAIT);
      break;
    case ST_WALL: {
      float raw = wallNormal(s.b) - sideAngle(s.a);
      wallTargetTh = poseTh + wrapPi(raw - poseTh);
      setPhase(AP_WALL_DRIVE);
      break;
    }
    case ST_INTAKE:
      intakeCommand((int)lroundf(s.p1));
      nextStep();
      return;
    case ST_SHOOTER:
      if (s.p1 <= 0.5f) { shooterEnable(false); nextStep(); return; }
      shooterSetRpm(s.p1);
      shooterEnable(true);
      if (async) { nextStep(); return; }
      stopDrive();
      setPhase(AP_SHOOTER);
      break;
    case ST_GRIP:
      gripSet(s.p1 > 0.5f);
      if (async) { nextStep(); return; }
      stopDrive();
      timerMs = GRIP_SETTLE_MS;
      setPhase(AP_TIMER);
      break;
    case ST_GUIDE: {
      uint32_t dur = (s.p2 > 0) ? (uint32_t)s.p2 : 0;
      guideSet((int)lroundf(s.p1), dur);
      if (async || dur == 0) { nextStep(); return; }
      stopDrive();
      timerMs = dur;
      setPhase(AP_TIMER);
      break;
    }
    case ST_CUSTOM:
      if (async) {
        asyncCustomId = s.a; asyncCustomParam = s.p1;
        if (runCustomAction(s.a, s.p1, true)) asyncCustomId = -1;
        nextStep();
        return;
      }
      stopDrive();
      setPhase(AP_CUSTOM);
      if (runCustomAction(s.a, s.p1, true)) { nextStep(); return; }
      break;
    default:
      nextStep();
      return;
  }
}

void autoUpdate(float dt) {
  // Huỷ: PS / Share, hoặc người lái gạt mạnh cần
  if (padNow.ok && (PRESSED(ps) || PRESSED(share))) { stopAuto("huỷ bằng tay cầm"); return; }
  if (padNow.ok && (fabsf(padNow.lx) > 0.7f || fabsf(padNow.ly) > 0.7f || fabsf(padNow.rx) > 0.7f)) {
    stopAuto("người lái gạt cần"); return;
  }
  if (!poseFresh()) { stopAuto("mất toạ độ từ RPi"); return; }
  if (asyncCustomId >= 0 && runCustomAction(asyncCustomId, asyncCustomParam, false)) asyncCustomId = -1;
  if (autoPhase == AP_DONE) { stopAuto("xong path"); return; }

  const AutoStep &s = STEPS[autoStep];
  switch (autoPhase) {
    case AP_APPROACH: {
      const TrajPt &p = TRAJ[s.a];
      poseControl(p.x, p.y, p.th, 0, 0, 0, dt, APPROACH_VMAX_MMS);
      bool ok = hypotf(p.x - poseX, p.y - poseY) < APPROACH_TOL_MM && fabsf(wrapPi(p.th - poseTh)) < APPROACH_TOL_RAD;
      if (ok || phaseMs() > APPROACH_TIMEOUT_MS) { trajIdx = s.a; setPhase(AP_FOLLOW); }
      break;
    }
    case AP_FOLLOW: {
      TrajPt t;
      trajAt(trajIdx, t);
      float err = hypotf(t.x - poseX, t.y - poseY);
      float scale = 1.0f - clampf((err - SLOW_START_MM) / (SLOW_FULL_MM - SLOW_START_MM), 0.0f, 1.0f);
      trajIdx += dt / TRAJ_DT_S * scale;
      if (trajIdx >= s.b) {
        trajIdx = s.b;
        if (s.flags & SF_CONTINUE) {
          lastFollowContinue = true; lastFollowEnd = s.b;
          nextStep();
          return;
        }
        setPhase(AP_SETTLE);
        resetPid();
      }
      trajAt(trajIdx, t);
      TrajPt ff;
      trajAt(fminf(trajIdx + FF_LEAD_S / TRAJ_DT_S, (float)s.b), ff);
      poseControl(t.x, t.y, t.th, ff.vx * scale, ff.vy * scale, ff.w * scale, dt, AUTO_VMAX_MMS);
      break;
    }
    case AP_SETTLE: {
      const TrajPt &p = TRAJ[s.b];
      poseControl(p.x, p.y, p.th, 0, 0, 0, dt, APPROACH_VMAX_MMS);
      bool ok = hypotf(p.x - poseX, p.y - poseY) < END_TOL_MM && fabsf(wrapPi(p.th - poseTh)) < END_TOL_RAD;
      if (ok || phaseMs() > END_TIMEOUT_MS) { stopDrive(); resetPid(); holdSet(p.x, p.y, p.th); nextStep(); }
      break;
    }
    case AP_WAIT:
      holdPosition(dt);
      if (phaseMs() >= (uint32_t)s.p1) nextStep();
      break;
    case AP_LIFT:
      holdPosition(dt);
      if (!liftBusy() || phaseMs() > LIFT_WAIT_TIMEOUT_MS) nextStep();
      break;
    case AP_SHOOTER:
      holdPosition(dt);
      if (shooterAtSpeed || shooterFault || phaseMs() > SHOOTER_WAIT_TIMEOUT_MS) nextStep();
      break;
    case AP_TIMER:
      holdPosition(dt);
      if (phaseMs() >= timerMs) nextStep();
      break;
    case AP_WALL_DRIVE: {
      int side = s.a, n = touchCount(side), hit = touchPressed(side);
      float dir = sideAngle(side);
      float sp = hit ? 0.6f * WALL_SPEED_MMS : WALL_SPEED_MMS;
      float w = hit ? 0 : clampf(AUTO_KP_TH * wrapPi(wallTargetTh - poseTh), -2.0f, 2.0f);
      driveRobot(sp * cosf(dir), sp * sinf(dir), w);
      bool done = (n > 0 && hit >= n);
      uint32_t timeout = (uint32_t)s.p1;
      if (done || (n == 0 && phaseMs() > timeout)) setPhase(AP_WALL_PUSH);   // không có công tắc: ép theo thời gian
      else if (phaseMs() > timeout) {
        if (hit) setPhase(AP_WALL_PUSH);
        else { stopDrive(); Log.println("[AUTO] Ép tường: không chạm, bỏ qua sửa vị trí"); nextStep(); }
      }
      break;
    }
    case AP_WALL_PUSH: {
      float dir = sideAngle(s.a);
      driveRobot(0.5f * WALL_SPEED_MMS * cosf(dir), 0.5f * WALL_SPEED_MMS * sinf(dir), 0);
      if (phaseMs() >= WALL_PUSH_MS) {
        stopDrive();
        float dist = (s.a == 0 || s.a == 2) ? ROBOT_LEN_MM / 2 : ROBOT_WID_MM / 2;
        float oldX = poseX, oldY = poseY, oldTh = poseTh;
        setPose(poseX, poseY, wallTargetTh);
        setCoordFromWall(s.b, dist);
        Log.printf("[AUTO] Ép tường: sửa dx=%.0f dy=%.0f dθ=%.1f°\n", poseX - oldX, poseY - oldY,
                      (poseTh - oldTh) * RAD_TO_DEG);
        resetPid();
        holdSetHere();
        nextStep();
      }
      break;
    }
    case AP_CUSTOM:
      holdPosition(dt);
      if (runCustomAction(s.a, s.p1, false)) nextStep();
      break;
    default:
      stopDrive();
      break;
  }
}

// =====================================================================
// ===================== HÀNH ĐỘNG RIÊNG (TỰ VIẾT) =====================
// =====================================================================
// Gọi với start = true một lần khi bắt đầu, sau đó start = false mỗi chu kỳ (10 ms).
// Trả về true khi xong. KHÔNG dùng delay() ở đây.
// id và tham số đặt trong app (hành động "★ Hành động riêng").
bool runCustomAction(int id, float param, bool start) {
  static uint32_t t0 = 0;
  if (start) t0 = millis();
  switch (id) {
    case 1:   // Ví dụ: chờ param ms
      return millis() - t0 >= (uint32_t)param;

    case 2:   // Ví dụ: nạp bóng trong param ms (mặc định 1500): cửa mở mỗi khi bánh bắn đủ rpm
      if (start) guideSet(1, param > 0 ? (uint32_t)param : 1500);
      return millis() - t0 >= (param > 0 ? (uint32_t)param : 1500);

    default:
      Log.printf("[AUTO] Hành động riêng id=%d chưa được viết\n", id);
      return true;
  }
}
