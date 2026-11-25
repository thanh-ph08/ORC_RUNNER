/*
 * Odometry.h — nhận toạ độ x, y, yaw từ Raspberry Pi qua UART (Pi lo BMI160 + chuột optical flow)
 *
 * Pi tự đẩy gói 0xAA 0x55 + PosePacket(40 byte) khoảng 200 lần/giây; ESP32 chỉ đọc, kiểm tra CRC rồi dùng gói mới nhất.
 * Lệnh gửi cho Pi (đặt toạ độ, đo lại gyro, xoá bộ đếm) đi theo khung 0xA5 0x5A + len + payload + crc8.
 */
#pragma once
#include <Arduino.h>
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

bool poseFresh() { return poseEverOk && (millis() - poseLastOkMs) < SENSOR_STALE_MS; }

void odometryBegin() {
  Serial2.setRxBufferSize(1024);
  Serial2.begin(RPI_BAUD, SERIAL_8N1, RPI_RX_PIN, RPI_TX_PIN);
}

static void rpiSend(const uint8_t *payload, uint8_t len) {
  uint8_t head[3] = {ORC_CMD_SYNC0, ORC_CMD_SYNC1, len};
  Serial2.write(head, 3);
  Serial2.write(payload, len);
  Serial2.write(orcCrc8(payload, len));
}

// Đặt toạ độ (cả trên Pi lẫn bản sao ở ESP32#1)
void setPose(float x, float y, float th) {
  poseX = x; poseY = y; poseTh = th;
  uint8_t buf[13];
  buf[0] = FLOW_CMD_SET_POSE;
  memcpy(buf + 1, &x, 4); memcpy(buf + 5, &y, 4); memcpy(buf + 9, &th, 4);
  rpiSend(buf, sizeof(buf));
  poseAckWanted = (uint8_t)(poseAckSeen + 1);
  poseAwaitAck = true;
  poseAwaitMs = millis();
}
void resetPoseToStart() {
  setPose(START_X_MM, START_Y_MM, START_TH_RAD);
  Serial.println("[POSE] Đặt lại về điểm xuất phát");
}
void flowCalibrateGyro() { uint8_t c = FLOW_CMD_CAL_GYRO; rpiSend(&c, 1); Serial.println("[POSE] Pi đo lại gyro: GIỮ YÊN XE ~2 s"); }
void flowClearRaw()      { uint8_t c = FLOW_CMD_CLEAR_RAW; rpiSend(&c, 1); }

static void applyPose(const PosePacket &p) {
  poseAckSeen = p.setAck;
  if (poseAwaitAck) {
    if (p.setAck == poseAckWanted) poseAwaitAck = false;
    else if (millis() - poseAwaitMs > 200) { poseAwaitAck = false; Serial.println("[POSE] Pi không xác nhận lệnh đặt toạ độ"); }
    else return;                                   // gói cũ, chờ Pi áp dụng toạ độ mới
  }
  poseX = p.x; poseY = p.y; poseTh = p.yaw;
  velX = p.vx; velY = p.vy; gyroRate = p.w;
  rawFlowX = p.rawX; rawFlowY = p.rawY; gyroAccumDeg = p.gyroDeg;
  poseFlags = p.flags;
  poseLastOkMs = millis();
  poseEverOk = true;
  poseOkCount++;
}

// Gọi mỗi vòng điều khiển: đọc hết byte đang chờ, tìm khung, kiểm tra CRC
void odometryUpdate() {
  static uint8_t buf[sizeof(PosePacket)];
  static uint8_t state = 0, idx = 0;                // 0: chờ SYNC0, 1: chờ SYNC1, 2: đang nhận gói
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    if (state == 0) { if (b == ORC_POSE_SYNC0) state = 1; }
    else if (state == 1) {
      if (b == ORC_POSE_SYNC1) { state = 2; idx = 0; }
      else state = (b == ORC_POSE_SYNC0) ? 1 : 0;
    } else {
      buf[idx++] = b;
      if (idx == sizeof(PosePacket)) {
        state = 0;
        PosePacket p;
        memcpy(&p, buf, sizeof(p));
        if (orcCrc8(buf, sizeof(p) - 1) == p.crc) applyPose(p);
        else poseErrCount++;
      }
    }
  }
}
