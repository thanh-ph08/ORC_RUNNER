/*
 * orc_protocol.h — giao thức giữa 3 board (I2C + UART). FILE NÀY GIỐNG HỆT NHAU ở 2 thư mục
 * (ORC_Robot, ORC_Encoder). Bản Python của Pi (ORC_Pi/orc_pi.py) tự khai báo lại cùng giao thức; sửa ở đây thì sửa cả ở đó.
 *
 *   ESP32#1 (ORC_Robot)   = I2C MASTER, đọc mọi thứ, điều khiển motor
 *   ESP32#2 (ORC_Encoder) = I2C slave 0x42, đọc 2 encoder (bắn banh + nâng hạ)
 *   RPi     (ORC_Pi)      = nối ESP32#1 bằng bus I2C riêng (Pi master, ESP32#1 slave 0x43; hoặc UART 921600 nếu PI_LINK_I2C 0), MPU6050 + chuột optical flow => toạ độ x, y, yaw
 *   Hub Ohstem driverV2   = I2C slave 0x54, chỉ dùng cho servo (thư viện OhStemRobotics)
 *
 * Mọi số nhiều byte là little-endian (ESP32 và RP2040 đều little-endian nên chép thẳng struct).
 */
#pragma once
#include <stdint.h>

#define ORC_ADDR_OHSTEM_HUB 0x54
#define ORC_ADDR_ENCODER    0x42
#define ORC_ADDR_FLOW       0x43   // (không dùng nữa — Pi nói chuyện qua UART)

// ---- UART giữa ESP32#1 và Raspberry Pi ----
//  Pi -> ESP32 : 0xAA 0x55 + PosePacket (40 byte, có CRC bên trong)      ~200 gói/giây
//  ESP32 -> Pi : 0xA5 0x5A + len + payload[len] + crc8(payload)           payload = lệnh FLOW_CMD_*
#define ORC_UART_BAUD     921600
#define ORC_POSE_SYNC0    0xAA
#define ORC_POSE_SYNC1    0x55
#define ORC_CMD_SYNC0     0xA5
#define ORC_CMD_SYNC1     0x5A

// ---- ESP32#2: master ghi 1 byte lệnh ----
#define ENC_CMD_RESET_SHOOTER 1
#define ENC_CMD_RESET_LIFT    2
#define ENC_CMD_RESET_ALL     3

// ---- RPi: lệnh gửi từ ESP32#1 (byte đầu tiên của payload) ----
#define FLOW_CMD_SET_POSE   0x10   // + float x_mm, float y_mm, float yaw_rad  (đặt toạ độ)
#define FLOW_CMD_CAL_GYRO   0x11   // đo lại độ lệch gyro (xe phải đứng yên ~1.5 s)
#define FLOW_CMD_CLEAR_RAW  0x12   // xoá bộ đếm thô (rawX, rawY, gyroDeg) để hiệu chỉnh
#define FLOW_CMD_LOG        0x20   // + text UTF-8 (tối đa 30 byte): một mảnh log của ESP32#1, Pi ghép theo dấu xuống dòng
#define FLOW_CMD_PING       0x21   // nhịp tim 1 lần/giây: Pi biết đường ESP32#1 -> Pi còn sống
#define FLOW_CMD_IMU        0x22   // ESP32#1 -> Pi: mẫu MPU6050 thô (xem Imu.h): u32 micros, 6×int16 (gx gy gz ax ay az), int16 temp, u8 reset, u16 lỗi

#define POSE_FLAG_IMU_OK   0x01
#define POSE_FLAG_MOUSE_OK 0x02
#define POSE_FLAG_CALIBRATING 0x04

#pragma pack(push, 1)
struct EncoderPacket {           // 18 byte
  uint8_t seq;
  int32_t shooterCount;          // xung x4 (tích luỹ)
  int32_t liftCount;
  int32_t shooterCps;            // xung/giây (đã lọc)
  int32_t liftCps;
  uint8_t crc;                   // CRC-8 của 17 byte đầu
};

struct PosePacket {              // 40 byte
  uint8_t seq;
  uint8_t flags;                 // POSE_FLAG_*
  uint8_t setAck;                // tăng 1 mỗi lần RPi nhận xong lệnh SET_POSE
  float x, y, yaw;               // mm, mm, rad — hệ sân, yaw ngược chiều kim đồng hồ
  float vx, vy, w;               // mm/s, mm/s (hệ sân), rad/s
  int32_t rawX, rawY;            // số đếm chuột thô (để hiệu chỉnh mm/count)
  float gyroDeg;                 // góc gyro cộng dồn (để hiệu chỉnh GYRO_SCALE)
  uint8_t crc;
};
#pragma pack(pop)

static inline uint8_t orcCrc8(const uint8_t *d, uint8_t n) {   // CRC-8, đa thức 0x07
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// ---- Liên kết Pi <-> ESP32#1 bằng I2C riêng (PI_LINK_I2C = 1 trong config.h) ----
// Bus riêng: Pi là MASTER (bus 3, GPIO5/6), ESP32#1 là SLAVE 0x43 trên Wire1 (GPIO18/19). Không dính bus của hub/ESP32#2/IMU.
//  Pi -> ESP32#1 : Pi ghi 42 byte = PILINK_W_POSE + PosePacket(40 byte, có CRC) + logAck (logSeq Pi đã nhận)  ~100 lần/giây
//  ESP32#1 -> Pi : Pi đọc 32 byte = PiStatus (lệnh đang chờ + 1 mảnh log)
#define ORC_PILINK_ADDR     0x43
#define PILINK_W_POSE       0xA7
#define PILINK_STATUS_MAGIC 0xA8
#define PILINK_F_SETPOSE    0x01   // đang chờ Pi xác nhận lệnh đặt toạ độ (setX, setY, setYaw)
#pragma pack(push, 1)
struct PiStatus {                // 32 byte
  uint8_t magic;                 // PILINK_STATUS_MAGIC
  uint8_t flags;                 // PILINK_F_*
  uint8_t cntSetPose;            // tăng 1 mỗi lần ESP32#1 ra lệnh đặt toạ độ
  uint8_t cntCalGyro;            // tăng 1 mỗi lần ra lệnh đo lại gyro
  uint8_t cntClearRaw;           // tăng 1 mỗi lần ra lệnh xoá bộ đếm thô
  uint8_t bootId;                // số ngẫu nhiên mỗi lần ESP32#1 khởi động (Pi biết ESP32 vừa reset)
  float setX, setY, setYaw;      // toạ độ của lệnh đặt toạ độ mới nhất (mm, mm, rad)
  uint8_t logLen;                // số byte log hợp lệ (0..11)
  uint8_t logSeq;                // tăng 1 mỗi mảnh log MỚI (Pi bỏ qua mảnh trùng khi đọc nhanh hơn ESP32 dựng gói)
  uint8_t log[11];               // một mảnh log (text UTF-8)
  uint8_t crc;                   // crc8 của 31 byte đầu
};
#pragma pack(pop)
