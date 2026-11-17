/*
 * ORC_Robot.ino — FILE CHÍNH: phần cứng + CHẠY THỦ CÔNG (tay PS4/PS5 qua Bluepad32)
 * File phụ  Auto.ino      : CHẠY TỰ ĐỘNG theo path_data.h
 * Dữ liệu   path_data.h   : tạo bằng app ORC Path Planner (nút "Tải path_data.h"), chép đè vào thư mục này
 *
 * Phần cứng: ESP32 thường + 4 motor mecanum (H-bridge, 2 chân PWM/motor), PMW3901 (SPI),
 *            ICM-20948 (I2C), ToF VL53L1X (I2C, XSHUT), công tắc chạm, stepper NEMA17 cho lift.
 *
 * Board: "ESP32 Dev Module" trong gói "esp32_bluepad32" (Boards Manager URL:
 *   https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json)
 * Thư viện (Library Manager): FastAccelStepper, "SparkFun 9DoF IMU Breakout - ICM 20948",
 *   "Bitcraze PMW3901", "VL53L1X" (của Pololu).
 *
 * Tay cầm (chế độ THỦ CÔNG):
 *   Cần trái: chạy tới/lùi/ngang     Cần phải (ngang): xoay      Giữ L2: chạy chậm
 *   R1 / L1: nâng / hạ lift (giữ)    D-pad lên/xuống: mức lift   X (Cross): lift về 0
 *   △: bật/tắt lái theo sân          □: bật/tắt giữ hướng (IMU)  R3: homing lift
 *   Share/Create: đặt lại toạ độ về điểm xuất phát
 *   GIỮ L1 + R1 rồi bấm Options: CHẠY TỰ ĐỘNG     O (Circle) / PS: dừng tự động
 * Serial 115200: r = về điểm xuất phát, a = chạy tự động, s = dừng, t = bật/tắt in toạ độ,
 *   f = in số đếm chuột quang (hiệu chỉnh), g = in góc gyro cộng dồn (hiệu chỉnh), z = xoá bộ đếm hiệu chỉnh
 * Hệ toạ độ: gốc góc dưới-trái sân, x sang phải, y lên (mm); θ ngược chiều kim đồng hồ, 0 = hướng +x.
 * Hệ robot: x = phía trước, y = bên trái.
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Bluepad32.h>
#include <FastAccelStepper.h>
#include <ICM_20948.h>
#include <Bitcraze_PMW3901.h>
#include <VL53L1X.h>
#include "path_data.h"

// =====================================================================
// ============================ CẤU HÌNH ===============================
// =====================================================================

// ---- Motor (mỗi motor 2 chân PWM: IN1/IN2) ----
// L298N: cắm jumper ENA/ENB.  BTS7960: R_EN, L_EN nối VCC logic.  TB6612: PWMA/PWMB nối 3.3V, STBY lên 3.3V.
enum { FL = 0, FR = 1, RL = 2, RR = 3 };            // trước-trái, trước-phải, sau-trái, sau-phải
const int  MOTOR_PIN_A[4]  = {25, 27, 32, 4};
const int  MOTOR_PIN_B[4]  = {26, 14, 33, 13};
const bool MOTOR_INVERT[4] = {false, true, false, true};  // đảo nếu bánh quay ngược khi đẩy cần tới
const int  PWM_FREQ = 20000, PWM_BITS = 10, PWM_MAX = 1023;

// ---- Hình học robot (PHẢI khớp ô Dài/Rộng trong app) ----
const float ROBOT_LEN_MM = 400.0f;   // trước–sau, tính cả cản
const float ROBOT_WID_MM = 400.0f;   // trái–phải, tính cả cản
const float WHEEL_LX_MM  = 150.0f;   // nửa khoảng cách trục trước–trục sau
const float WHEEL_LY_MM  = 160.0f;   // nửa khoảng cách bánh trái–bánh phải

// ---- Feedforward motor (không encoder): PWM = KS + KV * |v| ----
// KS: PWM nhỏ nhất để bánh bắt đầu quay khi xe đặt trên sân. KV: đo bằng lệnh 'f' khi chạy thẳng.
const float KS_PWM[4]        = {70, 70, 70, 70};
const float KV_PWM_PER_MMS[4] = {0.95f, 0.95f, 0.95f, 0.95f};
const float WHEEL_MAX_MMS    = 1000.0f;   // tốc độ bánh tối đa (mm/s) ứng với PWM_MAX

// ---- Pin (bù sụt áp) ----
const int   BATT_PIN      = 36;      // -1 nếu không đo
const float BATT_DIVIDER  = 5.0f;    // hệ số cầu chia áp (VD 40k/10k => 5.0)
const float BATT_NOMINAL_V = 12.0f;  // điện áp lúc hiệu chỉnh KS/KV

// ---- I2C / SPI ----
const int I2C_SDA = 21, I2C_SCL = 22;
const int FLOW_CS = 5;               // PMW3901: SCK 18, MISO 19, MOSI 23, CS 5

// ---- Chuột quang PMW3901 (đặt cách sàn >= 80 mm, nên có đèn LED chiếu xiên) ----
const float FLOW_MM_PER_COUNT = 0.20f;  // HIỆU CHỈNH: đẩy xe đúng 1000 mm, gõ 'f', lấy 1000 / số đếm
const bool  FLOW_SWAP_XY = false;       // đẩy xe TỚI mà số đếm tăng ở trục y => đặt true
const int   FLOW_SIGN_X = 1, FLOW_SIGN_Y = 1;  // đẩy TỚI phải ra x dương, đẩy sang TRÁI phải ra y dương
const float FLOW_POS_X_MM = 0.0f, FLOW_POS_Y_MM = 0.0f;  // vị trí cảm biến so với tâm robot (x trước, y trái)
const float FLOW_MAX_JUMP_MM = 40.0f;   // bỏ qua giá trị nhảy bất thường (mỗi 10 ms)

// ---- IMU ICM-20948 ----
const bool  ICM_AD0 = true;         // true: địa chỉ 0x69, false: 0x68
const int   GYRO_SIGN = 1;          // quay xe NGƯỢC chiều kim đồng hồ mà θ giảm => đặt -1
const float GYRO_SCALE = 1.000f;    // HIỆU CHỈNH: quay đúng 10 vòng, gõ 'g', lấy 3600 / số độ in ra

// ---- ToF VL53L1X (PHẢI khớp ô "Vị trí ToF" trong app) ----
const int   TOF_COUNT = 2;
const int   TOF_XSHUT[TOF_COUNT]     = {2, 15};
const int   TOF_SIDE[TOF_COUNT]      = {0, 3};        // 0 trước, 1 trái, 2 sau, 3 phải
const float TOF_OFFSET_MM[TOF_COUNT] = {190, 190};    // từ tâm robot tới mặt cảm biến

// ---- Công tắc chạm để ép tường: [mặt robot][2 công tắc], -1 = không có ----
// Chân 34–39 không có trở kéo lên trong: cần trở 10k kéo lên 3.3V.
const int  TOUCH_PIN[4][2] = { {-1, -1},   // trước
                               {-1, -1},   // trái
                               {34, 35},   // sau
                               {-1, -1} }; // phải
const bool TOUCH_ACTIVE_LOW = true;

// ---- Lift (NEMA17 + hộp số hành tinh + cuộn dây) ----
const int   LIFT_STEP = 16, LIFT_DIR = 17, LIFT_EN = -1;
const bool  LIFT_DIR_HIGH_UP = true;
const int   LIFT_HOME_PIN = 39;          // công tắc đáy lift, -1 nếu không có (khi đó bật máy lúc lift ở đáy)
const float LIFT_STEPS_PER_MM = 80.0f;   // = (200 × vi bước × tỉ số hộp số) / (mm dây cuộn mỗi vòng tang)
const float LIFT_MAX_MM = 900.0f;
const uint32_t LIFT_SPEED_HZ = 4000;     // bước/s
const int32_t  LIFT_ACCEL = 8000;        // bước/s²
const uint32_t LIFT_HOME_SPEED_HZ = 1500;
const float LIFT_JOG_MMS = 200.0f;
const float LIFT_LEVELS_MM[] = {0, 150, 400, 650, 880};
const int   LIFT_LEVEL_COUNT = sizeof(LIFT_LEVELS_MM) / sizeof(LIFT_LEVELS_MM[0]);

// ---- Lái thủ công ----
const float MANUAL_MAX_MMS = 900.0f;
const float MANUAL_MAX_W   = 4.0f;     // rad/s
const float MANUAL_SLOW    = 0.35f;
const float STICK_DEADBAND = 0.08f;
const float DRIVER_VIEW_DEG = 90.0f;   // lái theo sân: người lái nhìn theo hướng này (90 = nhìn về +y)
const float HOLD_KP = 5.0f;            // giữ hướng: rad/s trên mỗi rad lệch

const uint32_t CONTROL_PERIOD_US = 5000;   // vòng điều khiển 200 Hz
const uint32_t IMU_PERIOD_US     = 2000;   // 500 Hz
const uint32_t FLOW_PERIOD_US    = 10000;  // 100 Hz

// =====================================================================
// ============================= BIẾN TOÀN CỤC =========================
// =====================================================================
enum RobotMode { MODE_MANUAL = 0, MODE_AUTO = 1 };
RobotMode mode = MODE_MANUAL;

struct PadState {
  bool ok;
  float lx, ly, rx, ry;          // -1..1, ly/ry: âm = đẩy lên
  bool a, b, x, y, l1, r1, l2, r2, thumbR, up, down, start, share, ps;
};
PadState padNow = {}, padPrev = {};
#define PRESSED(f) (padNow.f && !padPrev.f)

ControllerPtr pad = nullptr;

// Vị trí robot (hệ sân)
float poseX = START_X_MM, poseY = START_Y_MM, poseTh = START_TH_RAD;
float velX = 0, velY = 0;              // mm/s (hệ sân, ước lượng)
float gyroBias = 0, gyroRate = 0;      // dps, rad/s
float flowLastTh = START_TH_RAD;
uint32_t lastImuUs = 0, lastFlowUs = 0;
bool imuOk = false, flowOk = false;
float rawFlowX = 0, rawFlowY = 0, gyroAccumDeg = 0;   // để hiệu chỉnh

ICM_20948_I2C icm;
Bitcraze_PMW3901 flow(FLOW_CS);
VL53L1X tof[TOF_COUNT];
bool tofOk[TOF_COUNT] = {};
float tofMM[TOF_COUNT] = {};
bool tofValid[TOF_COUNT] = {};
uint32_t tofStamp[TOF_COUNT] = {};

FastAccelStepperEngine stepperEngine;
FastAccelStepper *lift = nullptr;
float liftTargetMM = 0;
bool liftHoming = false;
uint32_t liftHomingStart = 0;
int liftLevel = 0;

bool fieldCentric = false, headingHold = true;
float holdTarget = START_TH_RAD;
uint32_t lastTurnMs = 0;
bool telemetryOn = true;
float battV = 0;

// Hàm ở Auto.ino
void startAuto();
void stopAuto(const char *why);
void autoUpdate(float dt);
const char *autoStatus();

// =====================================================================
// ============================== TIỆN ÍCH =============================
// =====================================================================
float wrapPi(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float deadband(float v) {
  if (fabsf(v) < STICK_DEADBAND) return 0;
  return (v - copysignf(STICK_DEADBAND, v)) / (1.0f - STICK_DEADBAND);
}
void setPose(float x, float y, float th) {
  poseX = x; poseY = y; poseTh = th;
  flowLastTh = th; holdTarget = th;
}
void resetPoseToStart() {
  setPose(START_X_MM, START_Y_MM, START_TH_RAD);
  Serial.println("[POSE] Đặt lại về điểm xuất phát");
}

// =====================================================================
// =============================== MOTOR ===============================
// =====================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach((pin), PWM_FREQ, PWM_BITS)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((pin), (duty))
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup((ch), PWM_FREQ, PWM_BITS); ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((ch), (duty))
#endif

void setupMotors() {
  for (int i = 0; i < 4; i++) {
    PWM_ATTACH(MOTOR_PIN_A[i], i * 2);
    PWM_ATTACH(MOTOR_PIN_B[i], i * 2 + 1);
    PWM_WRITE(MOTOR_PIN_A[i], i * 2, 0);
    PWM_WRITE(MOTOR_PIN_B[i], i * 2 + 1, 0);
  }
}

// pwm: -PWM_MAX..PWM_MAX
void setMotorPWM(int i, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (MOTOR_INVERT[i]) pwm = -pwm;
  if (pwm >= 0) {
    PWM_WRITE(MOTOR_PIN_A[i], i * 2, pwm);
    PWM_WRITE(MOTOR_PIN_B[i], i * 2 + 1, 0);
  } else {
    PWM_WRITE(MOTOR_PIN_A[i], i * 2, 0);
    PWM_WRITE(MOTOR_PIN_B[i], i * 2 + 1, -pwm);
  }
}

void stopMotors() {
  for (int i = 0; i < 4; i++) setMotorPWM(i, 0);
}

float batteryComp() {
  if (BATT_PIN < 0) return 1.0f;
  float v = analogReadMilliVolts(BATT_PIN) / 1000.0f * BATT_DIVIDER;
  battV = (battV < 1.0f) ? v : battV * 0.95f + v * 0.05f;
  if (battV < 5.0f) return 1.0f;                       // chưa nối dây đo pin
  return clampf(BATT_NOMINAL_V / battV, 0.8f, 1.3f);
}

// Lệnh vận tốc hệ ROBOT: vx tới (mm/s), vy sang trái (mm/s), w ngược chiều kim đồng hồ (rad/s)
void driveRobot(float vx, float vy, float w) {
  const float k = WHEEL_LX_MM + WHEEL_LY_MM;
  float v[4];
  v[FL] = vx - vy - k * w;
  v[FR] = vx + vy + k * w;
  v[RL] = vx + vy - k * w;
  v[RR] = vx - vy + k * w;
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
    setMotorPWM(i, v[i] >= 0 ? pwm : -pwm);
  }
}

// Lệnh vận tốc hệ SÂN
void driveField(float vx, float vy, float w) {
  float c = cosf(poseTh), s = sinf(poseTh);
  driveRobot(c * vx + s * vy, -s * vx + c * vy, w);
}

// =====================================================================
// ============================== CẢM BIẾN =============================
// =====================================================================
void setupImu() {
  for (int tries = 0; tries < 5 && !imuOk; tries++) {
    icm.begin(Wire, ICM_AD0);
    imuOk = (icm.status == ICM_20948_Stat_Ok);
    if (!imuOk) delay(200);
  }
  if (!imuOk) { Serial.println("[IMU] Không tìm thấy ICM-20948!"); return; }
  ICM_20948_fss_t fss;
  fss.a = gpm4;
  fss.g = dps1000;
  icm.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);
  Serial.println("[IMU] Đang đo độ lệch gyro, GIỮ YÊN XE...");
  double sum = 0; int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    if (icm.dataReady()) { icm.getAGMT(); sum += icm.gyrZ(); n++; }
    delay(2);
  }
  gyroBias = n ? (float)(sum / n) : 0;
  Serial.printf("[IMU] OK, bias = %.3f dps (%d mẫu)\n", gyroBias, n);
  lastImuUs = micros();
}

void updateImu(uint32_t now) {
  if (!imuOk || now - lastImuUs < IMU_PERIOD_US) return;
  if (!icm.dataReady()) return;
  icm.getAGMT();
  float dt = (now - lastImuUs) * 1e-6f;
  lastImuUs = now;
  if (dt > 0.05f) dt = 0.05f;
  float raw = icm.gyrZ();
  float dps = (raw - gyroBias) * GYRO_SIGN * GYRO_SCALE;
  // Xe đứng yên trong chế độ thủ công: từ từ học lại bias để giảm trôi
  bool idle = (mode == MODE_MANUAL) && fabsf(dps) < 1.5f && fabsf(velX) < 5 && fabsf(velY) < 5 &&
              (!padNow.ok || (padNow.lx == 0 && padNow.ly == 0 && padNow.rx == 0));
  if (idle) gyroBias += 0.0005f * (raw - gyroBias);
  gyroRate = dps * DEG_TO_RAD;
  poseTh += gyroRate * dt;
  gyroAccumDeg += dps * dt;
}

void setupFlow() {
  SPI.begin();
  flowOk = flow.begin();
  Serial.println(flowOk ? "[FLOW] PMW3901 OK" : "[FLOW] Không tìm thấy PMW3901!");
  lastFlowUs = micros();
}

void updateFlow(uint32_t now) {
  if (!flowOk || now - lastFlowUs < FLOW_PERIOD_US) return;
  float dts = (now - lastFlowUs) * 1e-6f;
  lastFlowUs = now;
  int16_t dx = 0, dy = 0;
  flow.readMotionCount(&dx, &dy);
  float sx = FLOW_SWAP_XY ? dy : dx;
  float sy = FLOW_SWAP_XY ? dx : dy;
  sx *= FLOW_SIGN_X;
  sy *= FLOW_SIGN_Y;
  rawFlowX += sx; rawFlowY += sy;
  float mx = sx * FLOW_MM_PER_COUNT, my = sy * FLOW_MM_PER_COUNT;
  float dth = wrapPi(poseTh - flowLastTh);
  flowLastTh = poseTh;
  if (hypotf(mx, my) > FLOW_MAX_JUMP_MM) return;     // nhiễu, bỏ qua
  // Bù chuyển động do xoay khi cảm biến không nằm ở tâm robot
  float cx = mx + dth * FLOW_POS_Y_MM;
  float cy = my - dth * FLOW_POS_X_MM;
  float thm = poseTh - 0.5f * dth;
  float c = cosf(thm), s = sinf(thm);
  float fx = c * cx - s * cy, fy = s * cx + c * cy;
  poseX += fx;
  poseY += fy;
  velX = 0.7f * velX + 0.3f * fx / dts;
  velY = 0.7f * velY + 0.3f * fy / dts;
}

void setupTof() {
  for (int i = 0; i < TOF_COUNT; i++) { pinMode(TOF_XSHUT[i], OUTPUT); digitalWrite(TOF_XSHUT[i], LOW); }
  delay(10);
  for (int i = 0; i < TOF_COUNT; i++) {
    digitalWrite(TOF_XSHUT[i], HIGH);
    delay(10);
    tof[i].setTimeout(100);
    if (!tof[i].init()) { Serial.printf("[TOF] Cảm biến #%d lỗi!\n", i); continue; }
    tof[i].setAddress(0x30 + i);
    tof[i].setDistanceMode(VL53L1X::Long);
    tof[i].setMeasurementTimingBudget(33000);
    tof[i].startContinuous(40);
    tofOk[i] = true;
    Serial.printf("[TOF] #%d OK\n", i);
  }
}

void updateTof() {
  for (int i = 0; i < TOF_COUNT; i++) {
    if (!tofOk[i] || !tof[i].dataReady()) continue;
    uint16_t d = tof[i].read(false);
    tofValid[i] = (tof[i].ranging_data.range_status == VL53L1X::RangeValid);
    tofMM[i] = d;
    tofStamp[i] = millis();
  }
}

void setupTouch() {
  for (int s = 0; s < 4; s++)
    for (int k = 0; k < 2; k++) {
      int p = TOUCH_PIN[s][k];
      if (p < 0) continue;
      pinMode(p, (p >= 34) ? INPUT : INPUT_PULLUP);
    }
  if (LIFT_HOME_PIN >= 0) pinMode(LIFT_HOME_PIN, (LIFT_HOME_PIN >= 34) ? INPUT : INPUT_PULLUP);
}

bool pinActive(int p) {
  if (p < 0) return false;
  return digitalRead(p) == (TOUCH_ACTIVE_LOW ? LOW : HIGH);
}
// Số công tắc có trên một mặt, và số công tắc đang bị chạm
int touchCount(int side) { return (TOUCH_PIN[side][0] >= 0) + (TOUCH_PIN[side][1] >= 0); }
int touchPressed(int side) { return pinActive(TOUCH_PIN[side][0]) + pinActive(TOUCH_PIN[side][1]); }

// =====================================================================
// ================================ LIFT ===============================
// =====================================================================
void setupLift() {
  stepperEngine.init();
  lift = stepperEngine.stepperConnectToPin(LIFT_STEP);
  if (!lift) { Serial.println("[LIFT] Không khởi tạo được stepper!"); return; }
  lift->setDirectionPin(LIFT_DIR, LIFT_DIR_HIGH_UP);
  if (LIFT_EN >= 0) { lift->setEnablePin(LIFT_EN); lift->setAutoEnable(true); }
  lift->setSpeedInHz(LIFT_SPEED_HZ);
  lift->setAcceleration(LIFT_ACCEL);
  lift->setCurrentPosition(0);
}

float liftPosMM() { return lift ? lift->getCurrentPosition() / LIFT_STEPS_PER_MM : 0; }
bool liftBusy() { return lift && (liftHoming || lift->isRunning()); }

void liftSetTarget(float mm) {
  liftTargetMM = clampf(mm, 0, LIFT_MAX_MM);
  if (lift && !liftHoming) lift->moveTo(lroundf(liftTargetMM * LIFT_STEPS_PER_MM));
}

void liftStartHoming() {
  if (!lift || LIFT_HOME_PIN < 0) return;
  liftHoming = true;
  liftHomingStart = millis();
  lift->setSpeedInHz(LIFT_HOME_SPEED_HZ);
  lift->runBackward();
  Serial.println("[LIFT] Homing...");
}

void liftUpdate() {
  if (!lift) return;
  bool atHome = pinActive(LIFT_HOME_PIN);
  if (liftHoming) {
    if (atHome) {
      lift->forceStopAndNewPosition(0);
      liftHoming = false; liftTargetMM = 0; liftLevel = 0;
      lift->setSpeedInHz(LIFT_SPEED_HZ);
      Serial.println("[LIFT] Homing xong");
    } else if (millis() - liftHomingStart > 10000) {
      lift->forceStop();
      liftHoming = false;
      lift->setSpeedInHz(LIFT_SPEED_HZ);
      Serial.println("[LIFT] Homing thất bại (quá 10 s)");
    }
    return;
  }
  // An toàn: đang hạ mà chạm công tắc đáy => dừng, đặt về 0
  if (atHome && lift->isRunning() && lift->getCurrentPosition() > lroundf(liftTargetMM * LIFT_STEPS_PER_MM)) {
    lift->forceStopAndNewPosition(0);
    liftTargetMM = 0;
  }
}

// =====================================================================
// ============================== TAY CẦM ==============================
// =====================================================================
void onConnectedController(ControllerPtr ctl) {
  if (pad == nullptr) {
    pad = ctl;
    Serial.println("[PAD] Đã kết nối tay cầm");
    ctl->setColorLED(0, 255, 0);
  } else {
    Serial.println("[PAD] Đã có tay cầm, bỏ qua tay thứ hai");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  if (pad == ctl) {
    pad = nullptr;
    Serial.println("[PAD] Mất kết nối tay cầm!");
  }
}

void readGamepad() {
  padPrev = padNow;
  PadState s = {};
  if (pad && pad->isConnected() && pad->isGamepad()) {
    s.ok = true;
    s.lx = deadband(pad->axisX() / 512.0f);
    s.ly = deadband(pad->axisY() / 512.0f);
    s.rx = deadband(pad->axisRX() / 512.0f);
    s.ry = deadband(pad->axisRY() / 512.0f);
    s.a = pad->a(); s.b = pad->b(); s.x = pad->x(); s.y = pad->y();
    s.l1 = pad->l1(); s.r1 = pad->r1(); s.l2 = pad->l2(); s.r2 = pad->r2();
    s.thumbR = pad->thumbR();
    uint8_t d = pad->dpad();
    s.up = d & 0x01; s.down = d & 0x02;
    uint16_t m = pad->miscButtons();
    s.ps = m & 0x01; s.share = m & 0x02; s.start = m & 0x04;
  }
  padNow = s;
}

void setPadColor(uint8_t r, uint8_t g, uint8_t b) {
  if (pad && pad->isConnected()) pad->setColorLED(r, g, b);
}

// =====================================================================
// ============================ CHẠY THỦ CÔNG ==========================
// =====================================================================
void manualUpdate(float dt) {
  if (!padNow.ok) { stopMotors(); return; }

  if (padNow.l1 && padNow.r1 && PRESSED(start)) { startAuto(); return; }
  if (PRESSED(y)) { fieldCentric = !fieldCentric; Serial.printf("[MAN] Lái theo sân: %s\n", fieldCentric ? "BẬT" : "TẮT"); }
  if (PRESSED(x)) { headingHold = !headingHold; holdTarget = poseTh; Serial.printf("[MAN] Giữ hướng: %s\n", headingHold ? "BẬT" : "TẮT"); }
  if (PRESSED(share)) resetPoseToStart();

  float sp = padNow.l2 ? MANUAL_SLOW : 1.0f;
  float stickF = -padNow.ly, stickL = -padNow.lx, turn = -padNow.rx;
  float vx, vy;
  if (fieldCentric) {
    // vector cần gạt trong hệ người lái -> hệ sân -> hệ robot
    float a = (DRIVER_VIEW_DEG - 90.0f) * DEG_TO_RAD;
    float sxR = -stickL, syU = stickF;                 // phải, lên
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
  } else if (headingHold && imuOk) {
    if (millis() - lastTurnMs < 250) holdTarget = poseTh;   // chờ xe hết quán tính quay
    float hasMove = (vx != 0 || vy != 0) ? 1.0f : 0.0f;
    w = hasMove * clampf(HOLD_KP * wrapPi(holdTarget - poseTh), -MANUAL_MAX_W, MANUAL_MAX_W);
  } else {
    w = 0;
  }

  if (vx == 0 && vy == 0 && w == 0) stopMotors();
  else driveRobot(vx, vy, w);

  // ---- Lift ----
  if (PRESSED(thumbR)) liftStartHoming();
  if (padNow.r1 && !padNow.l1) liftSetTarget(liftTargetMM + LIFT_JOG_MMS * dt);
  if (padNow.l1 && !padNow.r1) liftSetTarget(liftTargetMM - LIFT_JOG_MMS * dt);
  if (PRESSED(up))   { liftLevel = min(liftLevel + 1, LIFT_LEVEL_COUNT - 1); liftSetTarget(LIFT_LEVELS_MM[liftLevel]); }
  if (PRESSED(down)) { liftLevel = max(liftLevel - 1, 0); liftSetTarget(LIFT_LEVELS_MM[liftLevel]); }
  if (PRESSED(a))    { liftLevel = 0; liftSetTarget(0); }
}

// =====================================================================
// ========================= SERIAL & TELEMETRY ========================
// =====================================================================
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r': resetPoseToStart(); break;
      case 'a': startAuto(); break;
      case 's': if (mode == MODE_AUTO) stopAuto("lệnh Serial"); stopMotors(); break;
      case 't': telemetryOn = !telemetryOn; break;
      case 'f': Serial.printf("[CAL] Chuột quang: x = %.0f, y = %.0f count\n", rawFlowX, rawFlowY); break;
      case 'g': Serial.printf("[CAL] Gyro cộng dồn: %.1f độ\n", gyroAccumDeg); break;
      case 'z': rawFlowX = rawFlowY = gyroAccumDeg = 0; Serial.println("[CAL] Đã xoá bộ đếm"); break;
      default: break;
    }
  }
}

void telemetry() {
  static uint32_t last = 0;
  if (!telemetryOn || millis() - last < 200) return;
  last = millis();
  Serial.printf("x=%7.1f y=%7.1f th=%7.2f° | v=%5.0f | lift=%5.1f | tof:", poseX, poseY, poseTh * RAD_TO_DEG,
                hypotf(velX, velY), liftPosMM());
  for (int i = 0; i < TOF_COUNT; i++) Serial.printf(" %4.0f%s", tofMM[i], tofValid[i] ? "" : "?");
  Serial.printf(" | chạm sau:%d | %s | %s\n", touchPressed(2), mode == MODE_AUTO ? "AUTO" : "MAN", autoStatus());
}

// =====================================================================
// ============================= SETUP / LOOP ==========================
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n=== ORC Robot — path: %s (%d mẫu, %d bước) ===\n", PATH_NAME, TRAJ_COUNT, STEP_COUNT);
  setupMotors();
  stopMotors();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  setupTouch();
  setupTof();
  setupFlow();
  setupImu();
  setupLift();
  if (BATT_PIN >= 0) analogReadResolution(12);
  setPose(START_X_MM, START_Y_MM, START_TH_RAD);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys();   // bỏ comment 1 lần nếu cần ghép lại tay cầm từ đầu
  Serial.println("Ghép tay PS4: giữ Share + PS. Tay PS5: giữ Create + PS.");

  if (LIFT_HOME_PIN >= 0) liftStartHoming();
}

void loop() {
  BP32.update();
  uint32_t now = micros();
  updateImu(now);
  updateFlow(now);
  updateTof();
  handleSerial();

  static uint32_t lastCtrl = micros();
  if (now - lastCtrl >= CONTROL_PERIOD_US) {
    float dt = (now - lastCtrl) * 1e-6f;
    lastCtrl = now;
    if (dt > 0.05f) dt = 0.05f;
    readGamepad();
    if (mode == MODE_AUTO) autoUpdate(dt);
    else manualUpdate(dt);
    liftUpdate();
  }
  telemetry();
}
