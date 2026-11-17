/*
 * Auto.ino — FILE PHỤ: CHẠY TỰ ĐỘNG theo path_data.h
 *
 * Mỗi bước trong STEPS[] (tạo bởi app):
 *   FOLLOW : bám quỹ đạo TRAJ[a..b] = feedforward vận tốc + PID vị trí x, y, θ
 *   LIFT   : đưa lift tới p1 mm (có thể chạy song song)
 *   WAIT   : chờ p1 ms
 *   WALL   : chạy chậm vào tường tới khi công tắc chạm => đặt lại toạ độ + góc
 *   TOF    : đo khoảng cách tới tường => sửa một toạ độ
 *   CUSTOM : gọi runCustomAction(id, tham số) — tự viết ở cuối file này
 *
 * Không có encoder nên quỹ đạo có "đồng hồ co giãn": robot tụt lại xa điểm đích
 * thì đồng hồ chạy chậm lại / dừng, chờ robot bắt kịp (không bị cắt góc).
 */

// ============================ THAM SỐ PID ============================
const float AUTO_KP_XY = 3.0f;      // (mm/s) cho mỗi mm sai số
const float AUTO_KI_XY = 0.8f;
const float AUTO_KD_XY = 0.0f;
const float AUTO_I_MAX_MMS = 150.0f;   // giới hạn phần I (mm/s)
const float AUTO_KP_TH = 4.0f;      // (rad/s) cho mỗi rad sai số
const float AUTO_KI_TH = 0.5f;
const float AUTO_I_MAX_W = 0.6f;       // rad/s
const float AUTO_I_ZONE_MM = 40.0f;    // chỉ tích phân khi sai số nhỏ hơn mức này (chống tích luỹ quá đà)
const float AUTO_VMAX_MMS = 1000.0f;   // trần tốc độ tịnh tiến khi bám path
const float FF_LEAD_S = 0.08f;         // bù trễ motor: lấy vận tốc feedforward sớm hơn 80 ms
const float AUTO_WMAX = 5.0f;          // rad/s

// Đồng hồ co giãn: sai số > START thì bắt đầu chậm, >= FULL thì dừng hẳn đồng hồ
const float SLOW_START_MM = 40.0f, SLOW_FULL_MM = 120.0f;

// Kết thúc một đoạn có dừng
const float END_TOL_MM = 12.0f, END_TOL_RAD = 2.0f * DEG_TO_RAD;
const uint32_t END_TIMEOUT_MS = 1500;

// Tiến tới điểm đầu đoạn (khi đang lệch, VD sau khi ép tường)
const float APPROACH_TOL_MM = 25.0f, APPROACH_TOL_RAD = 4.0f * DEG_TO_RAD;
const float APPROACH_VMAX_MMS = 350.0f;
const uint32_t APPROACH_TIMEOUT_MS = 3000;

// Ép tường
const float WALL_SPEED_MMS = 180.0f;
const uint32_t WALL_PUSH_MS = 300;
// ToF
const float TOF_MAX_ANGLE_DEG = 15.0f;
const int   TOF_SAMPLES = 3;
const uint32_t TOF_TIMEOUT_MS = 500;
// Lift
const uint32_t LIFT_WAIT_TIMEOUT_MS = 10000;

const bool AUTO_RESET_POSE_ON_START = true;   // bấm chạy tự động => đặt toạ độ = điểm xuất phát

// ============================ TRẠNG THÁI =============================
enum AutoPhase { AP_IDLE, AP_APPROACH, AP_FOLLOW, AP_SETTLE, AP_WAIT, AP_LIFT, AP_WALL_DRIVE, AP_WALL_PUSH,
                 AP_TOF, AP_CUSTOM, AP_DONE };
const char *PHASE_NAME[] = {"idle", "approach", "follow", "settle", "wait", "lift", "wall", "push", "tof", "custom", "done"};

AutoPhase autoPhase = AP_IDLE;
int autoStep = 0;
uint32_t phaseStartMs = 0, autoStartMs = 0;
float trajIdx = 0;
bool lastFollowContinue = false;
int32_t lastFollowEnd = -1;
float iX = 0, iY = 0, iTh = 0, prevEx = 0, prevEy = 0;
// WALL / TOF
float wallTargetTh = 0;
int tofCount = 0; float tofSum = 0; uint32_t tofLastStamp = 0;
// CUSTOM chạy song song (1 suất)
int asyncCustomId = -1; float asyncCustomParam = 0;

bool runCustomAction(int id, float param, bool start);
void beginStep();

static char statusBuf[48];
const char *autoStatus() {
  if (mode != MODE_AUTO) return "";
  snprintf(statusBuf, sizeof(statusBuf), "bước %d/%d %s idx=%.0f", autoStep, STEP_COUNT, PHASE_NAME[autoPhase], trajIdx);
  return statusBuf;
}

void setPhase(int p) { autoPhase = (AutoPhase)p; phaseStartMs = millis(); }   // int: tránh lỗi prototype của Arduino IDE
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
// Đặt 1 toạ độ khi biết khoảng cách vuông góc từ tâm robot tới tường
void setCoordFromWall(int wall, float perp) {
  switch (wall) {
    case 0: poseX = perp; break;
    case 1: poseX = FIELD_W_MM - perp; break;
    case 2: poseY = perp; break;
    default: poseY = FIELD_H_MM - perp; break;
  }
}
float coordForWall(int wall) { return (wall <= 1) ? poseX : poseY; }

// Lấy điểm quỹ đạo tại chỉ số thực (nội suy)
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

// PID vị trí + feedforward, xuất lệnh hệ sân
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

// ============================ ĐIỀU KHIỂN =============================
void startAuto() {
  if (STEP_COUNT == 0) { Serial.println("[AUTO] path_data.h trống"); return; }
  if (AUTO_RESET_POSE_ON_START) setPose(START_X_MM, START_Y_MM, START_TH_RAD);
  mode = MODE_AUTO;
  autoStep = 0;
  lastFollowContinue = false; lastFollowEnd = -1;
  asyncCustomId = -1;
  autoStartMs = millis();
  resetPid();
  setPadColor(0, 0, 255);
  Serial.printf("[AUTO] BẮT ĐẦU path \"%s\"\n", PATH_NAME);
  beginStep();
}

void stopAuto(const char *why) {
  stopMotors();
  mode = MODE_MANUAL;
  setPhase(AP_IDLE);
  asyncCustomId = -1;
  setPadColor(0, 255, 0);
  Serial.printf("[AUTO] DỪNG: %s (%.1f s)\n", why, (millis() - autoStartMs) / 1000.0f);
}

void nextStep() {
  autoStep++;
  beginStep();
}

void beginStep() {
  if (autoStep >= STEP_COUNT) {
    stopMotors();
    setPhase(AP_DONE);
    Serial.printf("[AUTO] HOÀN THÀNH sau %.1f s\n", (millis() - autoStartMs) / 1000.0f);
    return;
  }
  const AutoStep &s = STEPS[autoStep];
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
      if (s.flags & SF_ASYNC) { nextStep(); return; }
      setPhase(AP_LIFT);
      break;
    case ST_WAIT:
      stopMotors();
      setPhase(AP_WAIT);
      break;
    case ST_WALL: {
      float raw = wallNormal(s.b) - sideAngle(s.a);
      wallTargetTh = poseTh + wrapPi(raw - poseTh);    // góc mục tiêu gần góc hiện tại nhất
      setPhase(AP_WALL_DRIVE);
      break;
    }
    case ST_TOF:
      stopMotors();
      tofCount = 0; tofSum = 0;
      tofLastStamp = (s.a >= 0 && s.a < TOF_COUNT) ? tofStamp[s.a] : 0;
      setPhase(AP_TOF);
      break;
    case ST_CUSTOM:
      if (s.flags & SF_ASYNC) {
        asyncCustomId = s.a; asyncCustomParam = s.p1;
        if (runCustomAction(s.a, s.p1, true)) asyncCustomId = -1;
        nextStep();
        return;
      }
      stopMotors();
      setPhase(AP_CUSTOM);
      if (runCustomAction(s.a, s.p1, true)) { nextStep(); return; }
      break;
    default:
      nextStep();
      return;
  }
}

void autoUpdate(float dt) {
  // Huỷ: O / PS, hoặc người lái gạt mạnh cần
  if (padNow.ok && (PRESSED(b) || PRESSED(ps))) { stopAuto("huỷ bằng tay cầm"); return; }
  if (padNow.ok && (fabsf(padNow.lx) > 0.7f || fabsf(padNow.ly) > 0.7f || fabsf(padNow.rx) > 0.7f)) {
    stopAuto("người lái gạt cần"); return;
  }
  if (asyncCustomId >= 0 && runCustomAction(asyncCustomId, asyncCustomParam, false)) asyncCustomId = -1;
  if (autoPhase == AP_DONE) { stopMotors(); stopAuto("xong path"); return; }

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
      if (ok || phaseMs() > END_TIMEOUT_MS) { stopMotors(); resetPid(); nextStep(); }
      break;
    }
    case AP_WAIT:
      stopMotors();
      if (phaseMs() >= (uint32_t)s.p1) nextStep();
      break;
    case AP_LIFT:
      stopMotors();
      if (!liftBusy() || phaseMs() > LIFT_WAIT_TIMEOUT_MS) nextStep();
      break;
    case AP_WALL_DRIVE: {
      int side = s.a, n = touchCount(side), hit = touchPressed(side);
      float dir = sideAngle(side);
      float sp = hit ? 0.6f * WALL_SPEED_MMS : WALL_SPEED_MMS;
      // chưa chạm: giữ hướng; đã chạm 1 bên: thả hướng cho xe tự xoay áp sát tường
      float w = hit ? 0 : clampf(AUTO_KP_TH * wrapPi(wallTargetTh - poseTh), -2.0f, 2.0f);
      driveRobot(sp * cosf(dir), sp * sinf(dir), w);
      bool done = (n > 0 && hit >= n);
      uint32_t timeout = (uint32_t)s.p1;
      if (done || (n == 0 && phaseMs() > timeout)) setPhase(AP_WALL_PUSH);   // không có công tắc: ép theo thời gian
      else if (phaseMs() > timeout) {
        if (hit) setPhase(AP_WALL_PUSH);
        else { stopMotors(); Serial.println("[AUTO] Ép tường: không chạm, bỏ qua sửa vị trí"); nextStep(); }
      }
      break;
    }
    case AP_WALL_PUSH: {
      float dir = sideAngle(s.a);
      driveRobot(0.5f * WALL_SPEED_MMS * cosf(dir), 0.5f * WALL_SPEED_MMS * sinf(dir), 0);
      if (phaseMs() >= WALL_PUSH_MS) {
        stopMotors();
        float dist = (s.a == 0 || s.a == 2) ? ROBOT_LEN_MM / 2 : ROBOT_WID_MM / 2;
        float oldX = poseX, oldY = poseY, oldTh = poseTh;
        setPose(poseX, poseY, wallTargetTh);
        setCoordFromWall(s.b, dist);
        Serial.printf("[AUTO] Ép tường: sửa dx=%.0f dy=%.0f dθ=%.1f°\n", poseX - oldX, poseY - oldY,
                      (poseTh - oldTh) * RAD_TO_DEG);
        resetPid();
        nextStep();
      }
      break;
    }
    case AP_TOF: {
      stopMotors();
      int i = s.a;
      if (i < 0 || i >= TOF_COUNT || !tofOk[i]) { Serial.println("[AUTO] ToF không sẵn sàng"); nextStep(); break; }
      if (tofStamp[i] != tofLastStamp) {
        tofLastStamp = tofStamp[i];
        if (tofValid[i]) { tofSum += tofMM[i]; tofCount++; }
      }
      if (tofCount >= TOF_SAMPLES || phaseMs() > TOF_TIMEOUT_MS) {
        if (tofCount > 0) {
          float d = tofSum / tofCount;
          float dirErr = wrapPi(poseTh + sideAngle(TOF_SIDE[i]) - wallNormal(s.b));
          if (fabsf(dirErr) > TOF_MAX_ANGLE_DEG * DEG_TO_RAD) {
            Serial.printf("[AUTO] ToF%d: lệch %.0f° so với tường, bỏ qua\n", i, dirErr * RAD_TO_DEG);
          } else {
            float perp = (d + TOF_OFFSET_MM[i]) * cosf(dirErr);
            float before = coordForWall(s.b);
            float ox = poseX, oy = poseY;
            setCoordFromWall(s.b, perp);
            float corr = coordForWall(s.b) - before;
            if (fabsf(corr) > s.p1) {
              poseX = ox; poseY = oy;
              Serial.printf("[AUTO] ToF%d: sửa %.0f mm quá lớn, bỏ qua\n", i, corr);
            } else {
              Serial.printf("[AUTO] ToF%d: d=%.0f mm, sửa %.0f mm\n", i, d, corr);
            }
          }
        } else {
          Serial.printf("[AUTO] ToF%d: không có số đo hợp lệ\n", i);
        }
        resetPid();
        nextStep();
      }
      break;
    }
    case AP_CUSTOM:
      stopMotors();
      if (runCustomAction(s.a, s.p1, false)) nextStep();
      break;
    default:
      stopMotors();
      break;
  }
}

// =====================================================================
// ===================== HÀNH ĐỘNG RIÊNG (TỰ VIẾT) =====================
// =====================================================================
// Gọi với start = true một lần khi bắt đầu, sau đó start = false mỗi chu kỳ (5 ms).
// Trả về true khi xong. KHÔNG dùng delay() ở đây.
// id và tham số đặt trong app (hành động "★ Hành động riêng").
bool runCustomAction(int id, float param, bool start) {
  static uint32_t t0 = 0;
  if (start) t0 = millis();
  switch (id) {
    case 1:   // Ví dụ: chờ param ms (thay bằng: bật motor kẹp, bắn, nhả vật…)
      return millis() - t0 >= (uint32_t)param;

    case 2:   // Ví dụ: nâng/hạ lift thêm param mm so với hiện tại, xong khi lift dừng
      if (start) liftSetTarget(liftTargetMM + param);
      return !liftBusy();

    default:
      Serial.printf("[AUTO] Hành động riêng id=%d chưa được viết\n", id);
      return true;
  }
}
