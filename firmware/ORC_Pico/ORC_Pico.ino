/*
 * ORC_Pico.ino — Raspberry Pi Pico (RP2040) = "RPi" của xe. I2C SLAVE 0x43 nói chuyện với ESP32#1.
 *
 * Việc của Pico:
 *   1. Đọc BMI160 (I2C1) -> bộ lọc Mahony (gyro + gia tốc) -> góc YAW
 *   2. Đọc chuột USB (optical flow) bằng PIO-USB host -> dịch chuyển (count)
 *   3. Ghép lại: dùng yaw để (a) xoay dịch chuyển từ hệ xe sang hệ sân, (b) trừ phần chuyển động giả do xe XOAY
 *      khi chuột không gắn đúng tâm quay (FLOW_POS_X/Y_MM) => toạ độ x, y, yaw (hệ sân)
 *   4. Gửi PosePacket (orc_protocol.h) cho ESP32#1 mỗi khi nó đọc
 *
 * ====== Nạp code (Arduino IDE) ======
 *   Boards Manager: "Raspberry Pi Pico/RP2040" của Earle Philhower (arduino-pico). Board: Raspberry Pi Pico.
 *   Tools > CPU Speed       = 120 MHz   (bắt buộc cho PIO-USB; 240 MHz cũng được)
 *   Tools > USB Stack       = Adafruit TinyUSB
 *   Library Manager: "Adafruit TinyUSB Library" (>= 3.0) và "Pico PIO USB" (Pico-PIO-USB của sekigon-gonnoc).
 *   Serial Monitor (USB của Pico) 115200 để xem debug.
 *
 * ====== Đấu dây ======
 *   I2C slave (nối ESP32#1):  SDA = GP4, SCL = GP5, GND chung (bus chung với ESP32#2 + hub Ohstem, trở kéo lên 3.3V)
 *   BMI160 (I2C1):            SDA = GP6, SCL = GP7, 3V3, GND. Nằm PHẲNG, trục Z hướng LÊN. SDO nối GND => 0x68, nối 3V3 => 0x69.
 *   Chuột USB (PIO-USB):      D+ = GP2, D- = GP3 (D- PHẢI là chân kế tiếp D+), cấp 5V cho chuột từ nguồn 5V,
 *                             mỗi chân D+/D- thêm trở 15k xuống GND. Mạch USB-A của board Ohstem có thể dùng chân khác: sửa PIN_USB_DP.
 *   Chuột nên là chuột có cảm biến optical (không phải laser), 800–1600 CPI, báo cáo 1000 Hz; gắn cách mặt sân 1–3 mm (chuột bàn thường
 *   cần sát mặt). Chuột báo cáo dạng "boot protocol" chỉ có dx, dy 8 bit nên mỗi gói tối đa ±127 count: để không tràn thì polling nhanh (1000 Hz).
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "hardware/clocks.h"
#include "pio_usb.h"
#include "Adafruit_TinyUSB.h"
#include "orc_protocol.h"

// =====================================================================
// ============================ CẤU HÌNH ===============================
// =====================================================================
const int PIN_I2C_SLAVE_SDA = 4, PIN_I2C_SLAVE_SCL = 5;     // I2C0 (slave, nối ESP32#1)
const int PIN_IMU_SDA = 6, PIN_IMU_SCL = 7;                  // I2C1 (master, BMI160)
const int PIN_USB_DP = 2;                                    // D+ ; D- = GP3

const uint8_t BMI160_ADDR = 0x69;       // 0x68 nếu SDO nối GND

// ---- Chuột optical flow ----
const float FLOW_MM_PER_COUNT = 0.60f;  // HIỆU CHỈNH: đẩy xe đúng 1000 mm, đọc rawX/rawY ('f' ở ESP32#1), lấy 1000 / số đếm
const bool  FLOW_SWAP_XY = false;       // đẩy xe TỚI mà số đếm tăng ở trục y => đặt true
const int   FLOW_SIGN_X = 1, FLOW_SIGN_Y = 1;   // đẩy TỚI phải ra x dương, đẩy sang TRÁI phải ra y dương
const float FLOW_POS_X_MM = 0.0f, FLOW_POS_Y_MM = 0.0f;  // vị trí chuột so với TÂM QUAY (x trước, y trái) — đo bằng thước
const float FLOW_MAX_JUMP_MM = 40.0f;   // bỏ qua dịch chuyển nhảy bất thường trong 1 chu kỳ (5 ms)

// ---- BMI160 ----
const int   GYRO_SIGN = 1;              // quay xe NGƯỢC chiều kim đồng hồ (nhìn từ trên) mà yaw GIẢM => -1
const float GYRO_SCALE = 1.000f;        // HIỆU CHỈNH: quay xe đúng 10 vòng, đọc gyroDeg ('g'), lấy 3600 / số độ
const float MAHONY_KP = 1.5f;           // độ tin gia tốc kế (chỉ sửa nghiêng); 0.5–5
const float ACCEL_G_MIN = 0.85f, ACCEL_G_MAX = 1.15f;  // chỉ tin gia tốc khi |a| ≈ 1 g (xe đang tăng tốc thì bỏ qua)
const uint32_t CAL_MS = 1500;           // thời gian đo độ lệch gyro lúc bật máy / khi có lệnh

const uint32_t IMU_PERIOD_US  = 2000;   // 500 Hz
const uint32_t FLOW_PERIOD_US = 5000;   // 200 Hz
const uint32_t PKT_PERIOD_US  = 4000;

// =====================================================================
// ===================== BMI160 (tự viết, không cần thư viện) ==========
// =====================================================================
static bool bmiWrite(uint8_t reg, uint8_t v) {
  Wire1.beginTransmission(BMI160_ADDR);
  Wire1.write(reg); Wire1.write(v);
  return Wire1.endTransmission() == 0;
}
static bool bmiRead(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire1.beginTransmission(BMI160_ADDR);
  Wire1.write(reg);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((uint8_t)BMI160_ADDR, (size_t)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire1.read();
  return true;
}

bool imuOk = false;
static bool bmiBegin() {
  Wire1.setSDA(PIN_IMU_SDA);
  Wire1.setSCL(PIN_IMU_SCL);
  Wire1.begin();
  Wire1.setClock(400000);
  uint8_t id = 0;
  if (!bmiRead(0x00, &id, 1) || id != 0xD1) {
    Serial.printf("[IMU] Không thấy BMI160 (đọc ID = 0x%02X, cần 0xD1)\n", id);
    return false;
  }
  bmiWrite(0x7E, 0xB6); delay(100);        // soft reset
  bmiWrite(0x7E, 0x11); delay(20);         // accel normal
  bmiWrite(0x7E, 0x15); delay(100);        // gyro normal
  bmiWrite(0x40, 0x2B);                    // ACC_CONF: 800 Hz, normal filter
  bmiWrite(0x41, 0x05);                    // ACC_RANGE ±4 g   (8192 LSB/g)
  bmiWrite(0x42, 0x2B);                    // GYR_CONF: 800 Hz, normal filter
  bmiWrite(0x43, 0x01);                    // GYR_RANGE ±1000 dps (32.8 LSB/dps)
  delay(50);
  return true;
}
// gx,gy,gz: dps (chưa trừ bias) ; ax,ay,az: g
static bool bmiReadAll(float &gx, float &gy, float &gz, float &ax, float &ay, float &az) {
  uint8_t b[12];
  if (!bmiRead(0x0C, b, 12)) return false;
  auto s16 = [&](int i) { return (int16_t)((b[i + 1] << 8) | b[i]); };
  gx = s16(0) / 32.8f; gy = s16(2) / 32.8f; gz = s16(4) / 32.8f;
  ax = s16(6) / 8192.0f; ay = s16(8) / 8192.0f; az = s16(10) / 8192.0f;
  return true;
}

// =====================================================================
// ======================== BỘ LỌC MAHONY 6 TRỤC =======================
// =====================================================================
struct Mahony {
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
  // g*: rad/s ; a*: g (sẽ chuẩn hoá) ; kp: hệ số tin gia tốc
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt, float kp, bool useAccel) {
    if (useAccel) {
      float n = sqrtf(ax * ax + ay * ay + az * az);
      if (n > 1e-6f) {
        ax /= n; ay /= n; az /= n;
        float hvx = q1 * q3 - q0 * q2;
        float hvy = q0 * q1 + q2 * q3;
        float hvz = q0 * q0 - 0.5f + q3 * q3;
        float ex = ay * hvz - az * hvy;
        float ey = az * hvx - ax * hvz;
        float ez = ax * hvy - ay * hvx;
        gx += 2.0f * kp * ex; gy += 2.0f * kp * ey; gz += 2.0f * kp * ez;
      }
    }
    gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    float r = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= r; q1 *= r; q2 *= r; q3 *= r;
  }
  float yaw() const { return atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)); }
  // Xoay quanh trục Z của sân sao cho yaw = newYaw (giữ nguyên độ nghiêng)
  void setYaw(float newYaw) {
    float d = newYaw - yaw();
    float c = cosf(0.5f * d), s = sinf(0.5f * d);
    float w = c * q0 - s * q3, x = c * q1 - s * q2, y = c * q2 + s * q1, z = c * q3 + s * q0;
    q0 = w; q1 = x; q2 = y; q3 = z;
  }
};
Mahony ahrs;

static inline float wrapPi(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

// =====================================================================
// ====================== CHUỘT USB (PIO-USB host, core 1) =============
// =====================================================================
Adafruit_USBH_Host USBHost;
volatile int32_t mouseAccX = 0, mouseAccY = 0;     // chỉ core 1 ghi
volatile bool mouseConnected = false;
volatile uint32_t mouseLastReportMs = 0;

void setup1() {
  uint32_t hz = clock_get_hz(clk_sys);
  if (hz != 120000000UL && hz != 240000000UL) {
    // PIO-USB cần CPU 120/240 MHz: vào Tools > CPU Speed
    while (true) { Serial.printf("LỖI: CPU = %lu Hz, cần 120 MHz (Tools > CPU Speed)\n", (unsigned long)hz); delay(1000); }
  }
  pio_usb_configuration_t cfg = PIO_USB_DEFAULT_CONFIG;
  cfg.pin_dp = PIN_USB_DP;
  USBHost.configure_pio_usb(1, &cfg);
  USBHost.begin(1);
}
void loop1() { USBHost.task(); }

extern "C" {
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  (void)desc_report; (void)desc_len;
  if (tuh_hid_interface_protocol(dev_addr, instance) == HID_ITF_PROTOCOL_MOUSE) {
    tuh_hid_set_protocol(dev_addr, instance, HID_PROTOCOL_BOOT);   // ép dạng boot: byte0 nút, byte1 dx, byte2 dy
    mouseConnected = true;
    tuh_hid_receive_report(dev_addr, instance);
  }
}
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  (void)dev_addr; (void)instance;
  mouseConnected = false;
}
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  if (tuh_hid_interface_protocol(dev_addr, instance) == HID_ITF_PROTOCOL_MOUSE && len >= 3) {
    mouseAccX = mouseAccX + (int8_t)report[1];
    mouseAccY = mouseAccY + (int8_t)report[2];
    mouseLastReportMs = millis();
  }
  tuh_hid_receive_report(dev_addr, instance);
}
}

// =====================================================================
// ========================= TRẠNG THÁI TOẠ ĐỘ =========================
// =====================================================================
float poseX = 0, poseY = 0, poseYaw = 0;
float velX = 0, velY = 0, gyroRateRad = 0;
float gyroBias[3] = {0, 0, 0};                      // dps
int32_t rawX = 0, rawY = 0;                         // tổng count thô (để hiệu chỉnh)
float gyroAccumDeg = 0;
uint8_t setAck = 0, seqNo = 0;

bool calibrating = true;
uint32_t calStartMs = 0;
double calSum[3] = {0, 0, 0};
int calCount = 0;

int32_t lastMouseX = 0, lastMouseY = 0;
float flowLastYaw = 0;
uint32_t lastImuUs = 0, lastFlowUs = 0, lastPktUs = 0;
uint32_t lastMotionMs = 0;

// lệnh từ ESP32#1 (ghi trong ngắt I2C, xử lý trong loop)
volatile bool cmdSetPose = false, cmdCalGyro = false, cmdClearRaw = false;
volatile float cmdX = 0, cmdY = 0, cmdYaw = 0;

static PosePacket txPkt;

void onReceiveCb(int n) {
  uint8_t buf[16];
  int i = 0;
  while (Wire.available() && i < 16) buf[i++] = Wire.read();
  while (Wire.available()) Wire.read();
  if (i < 1) return;
  if (buf[0] == FLOW_CMD_SET_POSE && i >= 13) {
    float v[3];
    memcpy(v, buf + 1, 12);
    cmdX = v[0]; cmdY = v[1]; cmdYaw = v[2];
    cmdSetPose = true;
  } else if (buf[0] == FLOW_CMD_CAL_GYRO) cmdCalGyro = true;
  else if (buf[0] == FLOW_CMD_CLEAR_RAW) cmdClearRaw = true;
}

void onRequestCb() {
  Wire.write((const uint8_t *)&txPkt, sizeof(txPkt));
}

static void startCalibration() {
  calibrating = true;
  calStartMs = millis();
  calSum[0] = calSum[1] = calSum[2] = 0;
  calCount = 0;
}

static void buildPacket() {
  PosePacket p;
  p.seq = ++seqNo;
  p.flags = (imuOk ? POSE_FLAG_IMU_OK : 0) | (mouseConnected ? POSE_FLAG_MOUSE_OK : 0) |
            (calibrating ? POSE_FLAG_CALIBRATING : 0);
  p.setAck = setAck;
  p.x = poseX; p.y = poseY; p.yaw = poseYaw;
  p.vx = velX; p.vy = velY; p.w = gyroRateRad;
  p.rawX = rawX; p.rawY = rawY;
  p.gyroDeg = gyroAccumDeg;
  p.crc = orcCrc8((uint8_t *)&p, sizeof(p) - 1);
  noInterrupts();
  txPkt = p;
  interrupts();
}

// =====================================================================
// ============================== SETUP ================================
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.setSDA(PIN_I2C_SLAVE_SDA);
  Wire.setSCL(PIN_I2C_SLAVE_SCL);
  Wire.onReceive(onReceiveCb);
  Wire.onRequest(onRequestCb);
  Wire.begin((uint8_t)ORC_ADDR_FLOW);

  imuOk = bmiBegin();
  memset(&txPkt, 0, sizeof(txPkt));
  buildPacket();
  startCalibration();
  lastImuUs = lastFlowUs = lastPktUs = micros();
  Serial.println("ORC_Pico sẵn sàng. Giữ yên xe ~1.5 s để đo độ lệch gyro.");
}

// =====================================================================
// =============================== VÒNG LẶP ============================
// =====================================================================
static void imuStep(uint32_t nowUs) {
  float dt = (nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;
  float gx, gy, gz, ax, ay, az;
  if (!imuOk || !bmiReadAll(gx, gy, gz, ax, ay, az)) return;

  if (calibrating) {
    calSum[0] += gx; calSum[1] += gy; calSum[2] += gz; calCount++;
    ahrs.update(0, 0, 0, ax, ay, az, dt, 8.0f, true);          // gom tư thế nghiêng nhanh khi đứng yên
    if (millis() - calStartMs >= CAL_MS && calCount > 50) {
      for (int i = 0; i < 3; i++) gyroBias[i] = (float)(calSum[i] / calCount);
      calibrating = false;
      lastMouseX = mouseAccX; lastMouseY = mouseAccY;           // bỏ qua chuột lúc cân
      flowLastYaw = ahrs.yaw();
      Serial.printf("[IMU] bias = %.3f %.3f %.3f dps (%d mẫu)\n", gyroBias[0], gyroBias[1], gyroBias[2], calCount);
    }
    return;
  }

  float rx = (gx - gyroBias[0]) * GYRO_SIGN * GYRO_SCALE;
  float ry = (gy - gyroBias[1]) * GYRO_SIGN * GYRO_SCALE;
  float rz = (gz - gyroBias[2]) * GYRO_SIGN * GYRO_SCALE;

  // Xe đứng yên (gyro + chuột không động đậy) => từ từ học lại bias để giảm trôi
  bool still = fabsf(rx) < 1.5f && fabsf(ry) < 1.5f && fabsf(rz) < 1.5f && (millis() - lastMotionMs) > 300;
  if (still) {
    gyroBias[0] += 0.0005f * (gx - gyroBias[0]);
    gyroBias[1] += 0.0005f * (gy - gyroBias[1]);
    gyroBias[2] += 0.0005f * (gz - gyroBias[2]);
  }

  float an = sqrtf(ax * ax + ay * ay + az * az);
  bool useAcc = an > ACCEL_G_MIN && an < ACCEL_G_MAX;
  ahrs.update(rx * DEG_TO_RAD, ry * DEG_TO_RAD, rz * DEG_TO_RAD, ax, ay, az, dt, MAHONY_KP, useAcc);
  poseYaw = ahrs.yaw();
  gyroRateRad = rz * DEG_TO_RAD;
  gyroAccumDeg += rz * dt;
}

static void flowStep(uint32_t nowUs) {
  float dts = (nowUs - lastFlowUs) * 1e-6f;
  lastFlowUs = nowUs;
  if (calibrating) return;
  int32_t cx = mouseAccX, cy = mouseAccY;
  int32_t dx = cx - lastMouseX, dy = cy - lastMouseY;
  lastMouseX = cx; lastMouseY = cy;

  float dth = wrapPi(poseYaw - flowLastYaw);
  flowLastYaw = poseYaw;
  if (dx != 0 || dy != 0 || fabsf(dth) > 1e-4f) lastMotionMs = millis();

  float sx = FLOW_SWAP_XY ? dy : dx;
  float sy = FLOW_SWAP_XY ? dx : dy;
  sx *= FLOW_SIGN_X; sy *= FLOW_SIGN_Y;
  rawX += (int32_t)sx; rawY += (int32_t)sy;
  float mx = sx * FLOW_MM_PER_COUNT, my = sy * FLOW_MM_PER_COUNT;
  if (hypotf(mx, my) > FLOW_MAX_JUMP_MM) return;         // nhiễu, bỏ qua

  // Chuột cách tâm quay (px, py): khi xe xoay dth, chuột "trượt" (-dth*py, +dth*px) dù tâm xe đứng yên => trừ đi
  float tx = mx + dth * FLOW_POS_Y_MM;
  float ty = my - dth * FLOW_POS_X_MM;
  // xoay từ hệ xe sang hệ sân bằng góc giữa chu kỳ
  float thm = poseYaw - 0.5f * dth;
  float c = cosf(thm), s = sinf(thm);
  float fx = c * tx - s * ty, fy = s * tx + c * ty;
  poseX += fx;
  poseY += fy;
  if (dts > 1e-4f) {
    velX = 0.7f * velX + 0.3f * fx / dts;
    velY = 0.7f * velY + 0.3f * fy / dts;
  }
}

void loop() {
  uint32_t now = micros();

  // ---- lệnh từ ESP32#1 ----
  if (cmdSetPose) {
    float x = cmdX, y = cmdY, yaw = cmdYaw;
    cmdSetPose = false;
    poseX = x; poseY = y;
    ahrs.setYaw(yaw);
    poseYaw = ahrs.yaw();
    flowLastYaw = poseYaw;
    lastMouseX = mouseAccX; lastMouseY = mouseAccY;
    velX = velY = 0;
    setAck++;
  }
  if (cmdCalGyro) { cmdCalGyro = false; startCalibration(); Serial.println("[IMU] Đo lại bias gyro"); }
  if (cmdClearRaw) { cmdClearRaw = false; rawX = rawY = 0; gyroAccumDeg = 0; }

  if (now - lastImuUs >= IMU_PERIOD_US) imuStep(now);
  if (now - lastFlowUs >= FLOW_PERIOD_US) flowStep(now);
  if (now - lastPktUs >= PKT_PERIOD_US) { lastPktUs = now; buildPacket(); }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("x=%7.1f y=%7.1f yaw=%7.2f° | chuột:%s raw=(%ld,%ld) | IMU:%s%s\n", poseX, poseY, poseYaw * RAD_TO_DEG,
                  mouseConnected ? "OK" : "chưa cắm", (long)rawX, (long)rawY, imuOk ? "OK" : "LỖI", calibrating ? " (đang cân gyro)" : "");
  }
}
