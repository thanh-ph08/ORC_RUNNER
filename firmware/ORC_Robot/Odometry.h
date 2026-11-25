/*
 * Odometry.h — đọc toạ độ x, y, yaw từ RPi qua I2C (RPi lo BMI160 + chuột optical flow)
 */
#pragma once
#include <Wire.h>
#include "path_data.h"
#include "config.h"
#include "orc_protocol.h"
#include "Common.h"

// Toạ độ robot (hệ sân, mm / rad). Gốc góc dưới-trái sân, x sang phải, y lên, yaw ngược chiều kim đồng hồ.
float poseX = START_X_MM, poseY = START_Y_MM, poseTh = START_TH_RAD;
float velX = 0, velY = 0, gyroRate = 0;            // mm/s, mm/s, rad/s
int32_t rawFlowX = 0, rawFlowY = 0;                 // số đếm chuột thô (hiệu chỉnh)
float gyroAccumDeg = 0;                             // góc gyro cộng dồn (hiệu chỉnh)
uint8_t poseFlags = 0;
uint32_t poseLastOkMs = 0, poseErrCount = 0, poseOkCount = 0;
bool poseEverOk = false;

static uint8_t poseAckSeen = 0, poseAckWanted = 0;
static bool poseAwaitAck = false;
static uint32_t poseAwaitMs = 0;

// Đọc 1 gói từ slave. true nếu đủ byte.
bool i2cReadPacket(uint8_t addr, uint8_t *buf, size_t len) {
  size_t n = Wire.requestFrom((uint16_t)addr, (size_t)len, true);
  if (n != len) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool poseFresh() { return poseEverOk && (millis() - poseLastOkMs) < SENSOR_STALE_MS; }

static bool flowSend(const uint8_t *data, size_t len) {
  for (int tries = 0; tries < 3; tries++) {
    Wire.beginTransmission(ORC_ADDR_FLOW);
    Wire.write(data, len);
    if (Wire.endTransmission() == 0) return true;
  }
  return false;
}

// Đặt toạ độ (cả trên RPi lẫn bản sao ở ESP32#1)
void setPose(float x, float y, float th) {
  poseX = x; poseY = y; poseTh = th;
  uint8_t buf[13];
  buf[0] = FLOW_CMD_SET_POSE;
  memcpy(buf + 1, &x, 4); memcpy(buf + 5, &y, 4); memcpy(buf + 9, &th, 4);
  if (flowSend(buf, sizeof(buf))) {
    poseAckWanted = (uint8_t)(poseAckSeen + 1);
    poseAwaitAck = true;
    poseAwaitMs = millis();
  } else {
    Serial.println("[POSE] Không gửi được lệnh đặt toạ độ tới RPi!");
  }
}
void resetPoseToStart() {
  setPose(START_X_MM, START_Y_MM, START_TH_RAD);
  Serial.println("[POSE] Đặt lại về điểm xuất phát");
}
void flowCalibrateGyro() { uint8_t c = FLOW_CMD_CAL_GYRO; flowSend(&c, 1); Serial.println("[POSE] RPi đo lại gyro: GIỮ YÊN XE ~2 s"); }
void flowClearRaw()      { uint8_t c = FLOW_CMD_CLEAR_RAW; flowSend(&c, 1); }

// Gọi mỗi vòng điều khiển
void odometryUpdate() {
  PosePacket p;
  if (!i2cReadPacket(ORC_ADDR_FLOW, (uint8_t *)&p, sizeof(p)) || orcCrc8((uint8_t *)&p, sizeof(p) - 1) != p.crc) {
    poseErrCount++;
    return;
  }
  poseAckSeen = p.setAck;
  if (poseAwaitAck) {
    if (p.setAck == poseAckWanted) poseAwaitAck = false;
    else if (millis() - poseAwaitMs > 200) { poseAwaitAck = false; Serial.println("[POSE] RPi không xác nhận lệnh đặt toạ độ"); }
    else return;                                   // gói cũ, chờ RPi áp dụng toạ độ mới
  }
  poseX = p.x; poseY = p.y; poseTh = p.yaw;
  velX = p.vx; velY = p.vy; gyroRate = p.w;
  rawFlowX = p.rawX; rawFlowY = p.rawY; gyroAccumDeg = p.gyroDeg;
  poseFlags = p.flags;
  poseLastOkMs = millis();
  poseEverOk = true;
  poseOkCount++;
}
