/*
 * Imu.h — ESP32#1 đọc MPU6050 (0x68, nằm sẵn trên Control Hub, chung bus I2C với hub servo) rồi gửi mẫu thô sang Raspberry Pi
 *         qua UART. Pi KHÔNG đọc I2C nữa, nên không còn hai master tranh nhau trên một bus.
 *
 * Khung gửi (FLOW_CMD_IMU 0x22, 22 byte): u32 micros | gx gy gz ax ay az (int16, đã đổi sang little-endian) | int16 temp thô | u8 số lần reset chip | u16 số lỗi I2C
 * Thang đo: gyro ±1000 dps (32.8 LSB/dps), accel ±4 g (8192 LSB/g), giống bản trước trên Pi.
 */
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "orc_protocol.h"

static void rpiSend(const uint8_t *payload, uint8_t len);   // Odometry.h

#ifndef IMU_ADDR
#define IMU_ADDR 0x68
#endif
#ifndef IMU_PERIOD_US
#define IMU_PERIOD_US 10000        // 100 Hz
#endif

struct ImuState {
  bool ok = false;
  uint8_t resets = 0;
  uint16_t err = 0;
  uint32_t sent = 0;
  uint8_t failRun = 0;
  uint8_t goodRun = 0;
  uint32_t lastUs = 0, lastRecoverMs = 0;
} imuS;

static bool imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool imuReadRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint16_t)IMU_ADDR, (size_t)n, true) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool imuBegin() {
#if !IMU_READ_ON_ESP32
  return false;                              // Pi tự đọc IMU
#endif
  uint8_t who;
  if (!imuReadRegs(0x75, &who, 1)) { Log.println("[IMU] Không đọc được MPU6050 (0x68) trên bus I2C"); imuS.ok = false; return false; }
  if (who != 0x68 && who != 0x69 && who != 0x70 && who != 0x71 && who != 0x72 && who != 0x73 && who != 0x98) {
    Log.printf("[IMU] WHO_AM_I = 0x%02X, không phải MPU6050\n", who); imuS.ok = false; return false;
  }
  bool ok = imuWrite(0x6B, 0x80); delay(100);   // reset thiết bị
  ok &= imuWrite(0x6B, 0x01); delay(50);         // thoát ngủ, xung nhịp PLL theo gyro X
  ok &= imuWrite(0x68, 0x07); delay(100);        // reset đường tín hiệu
  ok &= imuWrite(0x19, 0x00);                    // SMPLRT_DIV = 0
  ok &= imuWrite(0x1A, 0x03);                    // DLPF ~44 Hz
  ok &= imuWrite(0x1B, 0x10);                    // gyro ±1000 dps
  ok &= imuWrite(0x1C, 0x08);                    // accel ±4 g
  delay(100);
  imuS.ok = ok;
  if (ok) Log.println("[IMU] MPU6050 OK (đọc ở ESP32#1, gửi sang Pi qua UART)");
  return ok;
}

static bool imuConfigOk() {
  uint8_t a, b, c;
  if (!imuReadRegs(0x6B, &a, 1) || !imuReadRegs(0x1B, &b, 1) || !imuReadRegs(0x1C, &c, 1)) return true;   // lỗi bus thoáng qua
  return a == 0x01 && b == 0x10 && c == 0x08;
}

static void imuRecover() {
  imuS.lastRecoverMs = millis();
  imuS.failRun = 0;
  if (imuBegin()) { imuS.resets++; Log.println("[IMU] Chip bị reset giữa chừng -> đã khởi tạo lại"); }
}

// Gọi trong loop() thật thường xuyên; tự giữ nhịp IMU_PERIOD_US.
void imuStep() {
#if !IMU_READ_ON_ESP32
  return;
#endif
  uint32_t now = micros();
  if (now - imuS.lastUs < IMU_PERIOD_US) return;
  imuS.lastUs = now;
  if (Serial2.availableForWrite() < 28) return;                     // đường UART đang đầy: bỏ mẫu này
  if (!imuS.ok) {                                                  // chưa có chip: thử lại mỗi 1 s
    if (millis() - imuS.lastRecoverMs >= 1000) { imuS.lastRecoverMs = millis(); imuBegin(); }
    return;
  }
  uint8_t b[14];
  if (!imuReadRegs(0x3B, b, 14)) {
    imuS.err++;
    if (++imuS.failRun >= 25 && millis() - imuS.lastRecoverMs > 300) imuRecover();
    return;
  }
  bool allZero = true, allFF = true;
  for (uint8_t i = 0; i < 14; i++) { if (i != 6 && i != 7) { if (b[i]) allZero = false; if (b[i] != 0xFF) allFF = false; } }
  if (allZero || allFF) {                                          // chip ngủ / bus chết
    imuS.err++;
    if (++imuS.failRun >= 25 && millis() - imuS.lastRecoverMs > 300) imuRecover();
    return;
  }
  imuS.failRun = 0;
  if (++imuS.goodRun >= 20) {                                      // ~0.2 s: kiểm tra chip còn giữ cấu hình không
    imuS.goodRun = 0;
    if (!imuConfigOk()) { imuS.err++; imuRecover(); return; }
  }
  // MPU6050: big-endian, thứ tự ax ay az temp gx gy gz -> gửi gx gy gz ax ay az temp (little-endian)
  auto be = [&](uint8_t i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  int16_t v[7] = {be(8), be(10), be(12), be(0), be(2), be(4), be(6)};
  uint8_t f[22];
  f[0] = FLOW_CMD_IMU;
  memcpy(f + 1, &now, 4);
  memcpy(f + 5, v, 14);
  f[19] = imuS.resets;
  memcpy(f + 20, &imuS.err, 2);
  rpiSend(f, sizeof(f));
  imuS.sent++;
}
