/*
 * Odometry.h — nhận toạ độ x, y, yaw từ Raspberry Pi (Pi lo MPU6050 + chuột optical flow)
 *
 * PI_LINK_I2C = 1 (mặc định, xem config.h): bus I2C riêng, Pi master ghi PosePacket vào ESP32#1 (slave 0x43 trên Wire1),
 * Pi đọc lại PiStatus (lệnh đặt toạ độ / đo gyro / xoá bộ đếm + log). PI_LINK_I2C = 0: UART như bên dưới.
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

static uint8_t poseAckSeen = 0, poseAckAtSet = 0;  // setAck của Pi: lần mới nhất / lúc ra lệnh đặt toạ độ
static bool poseAckSynced = false;                 // đã nhận ít nhất 1 gói (biết setAck hiện tại của Pi)
static bool poseAwaitAck = false;
static uint32_t poseAwaitMs = 0;

bool poseFresh() { return poseEverOk && (millis() - poseLastOkMs) < SENSOR_STALE_MS; }

#if PI_LINK_I2C
#include <Wire.h>
static volatile uint8_t plFlags = 0, plCntSet = 0, plCntCal = 0, plCntClear = 0, plBootId = 0;
static volatile float plSetX = 0, plSetY = 0, plSetYaw = 0;
static volatile bool plRxNew = false;
static volatile uint32_t plReads = 0, plWrites = 0;
static uint8_t plRxBuf[sizeof(PosePacket)];
static portMUX_TYPE plMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t piLinkReads()  { return plReads; }
uint32_t piLinkWrites() { return plWrites; }

static volatile uint8_t plWant = 0;                // 0 = lần đọc tới trả PiStatus, 1 = trả ImuLink
static uint8_t plImuBuf[sizeof(ImuLink)];           // gói IMU mới nhất (Imu.h điền), mặc định magic = 0 => Pi bỏ qua
static volatile uint8_t plLogAck = 0;              // logSeq mà Pi báo đã nhận (gửi kèm mỗi lần Pi ghi)
// Callback I2C (chạy trong luồng/ISR của driver): KHÔNG gọi Log, delay hay Serial ở đây.
static void plOnReceive(int n) {
  uint8_t tmp[sizeof(PosePacket) + 2];               // [0xA7] + PosePacket + logAck
  int k = 0;
  while (Wire1.available()) {
    uint8_t b = (uint8_t)Wire1.read();
    if (k < (int)sizeof(tmp)) tmp[k++] = b;
  }
  plWrites++;
  if (k >= 1 && tmp[0] == PILINK_W_IMU) { plWant = 1; return; }
  plWant = 0;
  if (k == (int)sizeof(tmp) && tmp[0] == PILINK_W_POSE) {
    portENTER_CRITICAL_SAFE(&plMux);
    memcpy(plRxBuf, tmp + 1, sizeof(PosePacket));
    plRxNew = true;
    plLogAck = tmp[sizeof(tmp) - 1];
    portEXIT_CRITICAL_SAFE(&plMux);
  }
}

// Gói trạng thái dựng SẴN ở vòng lặp chính (odometryUpdate), callback chỉ chép ra — làm nặng trong callback
// thì FIFO phát của ESP32 cạn trước khi kịp nạp (Pi chỉ nhận được byte đầu, phần còn lại 0xFF).
// Log đi theo kiểu "gửi - chờ xác nhận": giữ nguyên mảnh hiện tại cho tới khi Pi báo đã nhận (logAck == logSeq)
// rồi mới lấy mảnh mới. Nhờ vậy driver slave có trả gói cũ / trả trễ một nhịp thì log cũng không mất, không lặp.
static PiStatus plStBuf;
static volatile bool plStConsumed = true;          // true = gói đã được Pi đọc, cần dựng gói mới
static uint8_t plLogSeq = 0, plLogLen = 0;
static uint8_t plLogBuf[sizeof(((PiStatus *)0)->log)];

static void plBuildStatus() {
  PiStatus st;
  memset(&st, 0, sizeof(st));
  st.magic = PILINK_STATUS_MAGIC;
  st.flags = plFlags;
  st.cntSetPose = plCntSet; st.cntCalGyro = plCntCal; st.cntClearRaw = plCntClear;
  st.bootId = plBootId;
  st.setX = plSetX; st.setY = plSetY; st.setYaw = plSetYaw;
  if (plLogLen == 0 || plLogAck == plLogSeq) {      // mảnh trước đã được Pi nhận (hoặc chưa có mảnh): lấy mảnh mới
    plLogLen = (uint8_t)Log.take(plLogBuf, sizeof(plLogBuf));
    if (plLogLen) plLogSeq++;
  }
  st.logLen = plLogLen;
  st.logSeq = plLogSeq;
  memcpy(st.log, plLogBuf, plLogLen);
  st.crc = orcCrc8((const uint8_t *)&st, sizeof(st) - 1);
  portENTER_CRITICAL(&plMux);
  plStBuf = st;
  plStConsumed = false;
  portEXIT_CRITICAL(&plMux);
}

static void plOnRequest() {
  uint8_t out[sizeof(PiStatus)];
  portENTER_CRITICAL_SAFE(&plMux);
  if (plWant == 1) memcpy(out, plImuBuf, sizeof(out));
  else { memcpy(out, &plStBuf, sizeof(out)); plStConsumed = true; }
  portEXIT_CRITICAL_SAFE(&plMux);
  Wire1.write(out, sizeof(out));
  plReads++;
}

void odometryBegin() {
  plBootId = (uint8_t)(esp_random() & 0xFF);
  Wire1.onReceive(plOnReceive);
  Wire1.onRequest(plOnRequest);
  bool ok = Wire1.begin((uint8_t)ORC_PILINK_ADDR, PILINK_SDA, PILINK_SCL, 100000);
  plBuildStatus();
  Log.printf("[LINK] I2C slave 0x%02X (Pi master) trên GPIO%d/GPIO%d: %s\n", ORC_PILINK_ADDR, PILINK_SDA, PILINK_SCL, ok ? "OK" : "LỖI khởi tạo");
}

// Imu.h gọi khi có gói IMU mới
static void piLinkSetImu(const ImuLink &m) {
  portENTER_CRITICAL(&plMux);
  memcpy(plImuBuf, &m, sizeof(m));
  portEXIT_CRITICAL(&plMux);
}
static void rpiSend(const uint8_t *, uint8_t) {}     // không dùng ở chế độ I2C (giữ cho Imu.h / Log.h)
#else
uint32_t piLinkReads()  { return 0; }
uint32_t piLinkWrites() { return 0; }

void odometryBegin() {
  Serial2.setRxBufferSize(1024);
  Serial2.begin(RPI_BAUD, SERIAL_8N1, RPI_RX_PIN, RPI_TX_PIN);
}

static void piLinkSetImu(const ImuLink &) {}
static void rpiSend(const uint8_t *payload, uint8_t len) {
  uint8_t head[3] = {ORC_CMD_SYNC0, ORC_CMD_SYNC1, len};
  Serial2.write(head, 3);
  Serial2.write(payload, len);
  Serial2.write(orcCrc8(payload, len));
}
#endif

// Đặt toạ độ (cả trên Pi lẫn bản sao ở ESP32#1)
void setPose(float x, float y, float th) {
  poseX = x; poseY = y; poseTh = th;
#if PI_LINK_I2C
  plSetX = x; plSetY = y; plSetYaw = th;
  plCntSet = (uint8_t)(plCntSet + 1);
  plFlags |= PILINK_F_SETPOSE;                       // xoá khi Pi xác nhận (setAck trong PosePacket)
#else
  uint8_t buf[13];
  buf[0] = FLOW_CMD_SET_POSE;
  memcpy(buf + 1, &x, 4); memcpy(buf + 5, &y, 4); memcpy(buf + 9, &th, 4);
  rpiSend(buf, sizeof(buf));
#endif
  poseAckAtSet = poseAckSeen;                       // Pi áp dụng xong thì setAck sẽ khác giá trị này
  poseAwaitAck = true;
  poseAwaitMs = millis();
}
void resetPoseToStart() {
  setPose(START_X_MM, START_Y_MM, START_TH_RAD);
  Log.println("[POSE] Đặt lại về điểm xuất phát");
}
#if PI_LINK_I2C
void flowCalibrateGyro() { plCntCal = (uint8_t)(plCntCal + 1); Log.println("[POSE] Pi đo lại gyro: GIỮ YÊN XE ~2 s"); }
void flowClearRaw()      { plCntClear = (uint8_t)(plCntClear + 1); }
#else
void flowCalibrateGyro() { uint8_t c = FLOW_CMD_CAL_GYRO; rpiSend(&c, 1); Log.println("[POSE] Pi đo lại gyro: GIỮ YÊN XE ~2 s"); }
void flowClearRaw()      { uint8_t c = FLOW_CMD_CLEAR_RAW; rpiSend(&c, 1); }
#endif

static void applyPose(const PosePacket &p) {
  if (!poseAckSynced) {                            // gói đầu tiên (ESP32#1 hoặc Pi vừa khởi động): lấy mốc setAck của Pi
    poseAckSynced = true;
    poseAckSeen = poseAckAtSet = p.setAck;
    poseAwaitMs = millis();
  }
  poseAckSeen = p.setAck;
  poseLastOkMs = millis();                         // liên kết còn sống (kể cả khi đang chờ Pi xác nhận)
  if (poseAwaitAck) {
    if (p.setAck != poseAckAtSet) {                // Pi đã áp dụng lệnh đặt toạ độ mới nhất
      poseAwaitAck = false;
#if PI_LINK_I2C
      plFlags &= (uint8_t)~PILINK_F_SETPOSE;
#endif
    } else if (millis() - poseAwaitMs > 500) {
      poseAwaitAck = false;
#if PI_LINK_I2C
      plFlags &= (uint8_t)~PILINK_F_SETPOSE;
#endif
      Log.println("[POSE] Pi không xác nhận lệnh đặt toạ độ (0.5 s)");
    } else return;                                 // gói cũ, giữ toạ độ vừa đặt cho tới khi Pi áp dụng
  }
  poseX = p.x; poseY = p.y; poseTh = p.yaw;
  velX = p.vx; velY = p.vy; gyroRate = p.w;
  rawFlowX = p.rawX; rawFlowY = p.rawY; gyroAccumDeg = p.gyroDeg;
  poseFlags = p.flags;
  poseEverOk = true;
  poseOkCount++;
}

// Gọi mỗi vòng điều khiển
#if PI_LINK_I2C
void odometryUpdate() {
  if (plStConsumed) plBuildStatus();               // chuẩn bị gói cho lần Pi đọc tiếp theo
  if (!plRxNew) return;
  PosePacket p;
  portENTER_CRITICAL(&plMux);
  memcpy(&p, plRxBuf, sizeof(p));
  plRxNew = false;
  portEXIT_CRITICAL(&plMux);
  if (orcCrc8((const uint8_t *)&p, sizeof(p) - 1) == p.crc) applyPose(p);
  else poseErrCount++;
}
#else
// đọc hết byte đang chờ, tìm khung, kiểm tra CRC
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
#endif
