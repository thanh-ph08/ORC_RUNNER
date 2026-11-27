/*
 * config.h — TOÀN BỘ chân + hằng số hiệu chỉnh của ESP32#1. Chỉ cần sửa file này khi đổi dây.
 */
#pragma once

// 1 = mỗi lần bật ESP32 xoá khoá Bluetooth cũ, tay cầm phải ghép lại (giữ Share + PS). Dùng khi mới thử / ghép lỗi.
// Khi đã ghép ổn, đổi về 0 để tay cầm tự nối lại sau khi bật xe.
#define PAD_FORGET_KEYS_ON_BOOT 1
#include <stdint.h>

// =====================================================================
// ===== 4 mạch L298N, mỗi mạch 4 chân IN1..IN4 (jumper ENA/ENB giữ nguyên) =====
// PWM đặt thẳng lên chân IN, chân IN còn lại của motor giữ LOW.
//   L298N #1: IN1 IN2 IN3 IN4 = 32 33 25 26  -> FL (32,33)  FR (25,26)
//   L298N #2:                   27 14 12 13  -> RL (27,14)  RR (12,13)
//   L298N #3:                   15  2  4 16  -> INTAKE (15,2)  SHOOTER (4,16)
//   L298N #4:                   17  5 18 19  -> LIFT (17,5)    (18,19 còn trống)
// Lưu ý: GPIO 12, 2, 15, 5 là chân "strapping" của ESP32. Nếu board không boot / không nạp được
// thì rút dây các chân này ra lúc cấp nguồn/nạp code, hoặc đổi sang chân khác.
// =====================================================================
enum MotorId { M_FL = 0, M_FR, M_RL, M_RR, M_INTAKE, M_SHOOTER, M_LIFT, M_SPARE, MOTOR_COUNT };
const int  MOTOR_IN1[MOTOR_COUNT] = {32, 25, 27, 12, 15,  4, 17, -1};
const int  MOTOR_IN2[MOTOR_COUNT] = {33, 26, 14, 13,  2, 16,  5, -1};
// Đảo chiều nếu motor quay ngược. Xe: đẩy cần tới thì cả 4 bánh phải quay tới.
const bool MOTOR_INVERT[MOTOR_COUNT] = {false, true, false, true,   false, false, false, false};
const int  PWM_FREQ = 5000;          // L298N chậm, đừng đặt cao (1–10 kHz)
const int  PWM_BITS = 10;
const int  PWM_MAX  = 1023;

// =====================================================================
// ===== UART tới Raspberry Pi (toạ độ x, y, yaw). Nối chéo: Pi TXD (GPIO14) -> RX, Pi RXD (GPIO15) <- TX, GND chung =====
// GPIO34 chỉ nhận vào nên dùng làm RX; đừng dùng GPIO1/3 (UART0, dính cổng USB nạp code).
// =====================================================================
const int      RPI_RX_PIN = 34, RPI_TX_PIN = 23;
const uint32_t RPI_BAUD   = 921600;

// ===== Liên kết Pi <-> ESP32#1 =====
//  1 = I2C riêng: Pi master (GPIO5 = SDA chân 29, GPIO6 = SCL chân 31) <-> ESP32#1 slave 0x43 trên Wire1 (GPIO18 = SDA, GPIO19 = SCL).
//      Cần trở kéo lên 4.7k về 3.3V trên cả SDA và SCL (một bộ duy nhất), GND chung. Không dùng UART2 nữa.
//  0 = UART2 như cũ (dây ở RPI_RX_PIN / RPI_TX_PIN phía trên)
#define PI_LINK_I2C 1
const int PILINK_SDA = 18, PILINK_SCL = 19;

// =====================================================================
// ===== I2C (ESP32#1 là master): ESP32#2 (encoder) và hub Ohstem (servo) cùng 1 bus =====
// Cần trở kéo lên 3.3V (2.2k–4.7k) trên SDA/SCL. Nếu hay lỗi đọc, hạ I2C_CLOCK xuống 100000.
// =====================================================================
const int      I2C_SDA = 21, I2C_SCL = 22;
const uint32_t I2C_CLOCK = 100000;   // 100 kHz: ổn định hơn khi Bluetooth (Bluepad32) đang chạy; tăng lên 400000 nếu bus sạch
// IMU (MPU6050 0x68 trên Control Hub): 1 = ESP32#1 đọc rồi đưa sang Pi (mặc định; Pi đặt IMU_TYPE = "esp32", KHÔNG đụng bus này);
// 0 = Pi tự đọc I2C — chỉ dùng khi Pi nối IMU trên bus RIÊNG, vì chung bus với ESP32#1 là 2 master tranh nhau (lỗi, treo 1 s)
#define IMU_READ_ON_ESP32 1
const uint32_t SENSOR_STALE_MS = 80;   // quá lâu không đọc được => coi là mất tín hiệu

// =====================================================================
// ===== Hình học robot (PHẢI khớp ô Dài/Rộng trong app, mm = cm × 10) =====
// =====================================================================
const float ROBOT_LEN_MM = 400.0f;
const float ROBOT_WID_MM = 400.0f;
const float WHEEL_LX_MM  = 150.0f;   // nửa khoảng cách trục trước–trục sau
const float WHEEL_LY_MM  = 160.0f;   // nửa khoảng cách bánh trái–bánh phải

// ---- Feedforward motor xe (xe không có encoder): PWM = KS + KV * |v| ----
const float KS_PWM[4]         = {110, 110, 110, 110};   // PWM nhỏ nhất để bánh nhúc nhích (L298N hao ~2 V nên lớn hơn H-bridge khác)
const float KV_PWM_PER_MMS[4] = {0.90f, 0.90f, 0.90f, 0.90f};
const float WHEEL_MAX_MMS     = 1000.0f;                // tốc độ bánh tối đa ứng với PWM_MAX
const int   BATT_PIN       = -1;     // chân đo pin (cầu chia áp), -1 = không đo
const float BATT_DIVIDER   = 5.0f;
const float BATT_NOMINAL_V = 12.0f;

// =====================================================================
// ===== Công tắc chạm để ép tường (chưa có => -1, ép tường sẽ chạy theo thời gian) =====
// [mặt robot: 0 trước, 1 trái, 2 sau, 3 phải][2 công tắc]
// =====================================================================
const int  TOUCH_PIN[4][2] = { {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1} };
const bool TOUCH_ACTIVE_LOW = true;

// =====================================================================
// ===== INTAKE (L298N #3, motor 1) =====
// =====================================================================
const int  INTAKE_PWM = 800;              // 0..1023
const bool INTAKE_REVERSE_WITH_R1 = true; // giữ R1 để đảo chiều xả kẹt

// =====================================================================
// ===== SHOOTER (L298N #3, motor 2) — PID tốc độ bằng encoder =====
// Encoder Hall của GA25 (ESP32#2 GPIO8/9), đọc x4. Đã bỏ hộp số => encoder nằm ngay trục bánh bắn.
// GA25 thường 11 xung/vòng => 44 count/vòng (có loại 7 hoặc 12 xung => 28 / 48).
// KIỂM TRA: quay tay bánh bắn đúng 10 vòng, gõ 'e', lấy count / 10.
// =====================================================================
const float SHOOTER_CPR          = 44.0f;     // count / vòng của bánh bắn
const int   SHOOTER_ENC_SIGN     = 1;         // -1 nếu bật bắn mà rpm đo được ÂM
const int   SHOOTER_DIR          = 1;         // -1 nếu bánh bắn quay ngược (hoặc đảo MOTOR_INVERT)
const float SHOOTER_RPM_DEFAULT  = 3000.0f;   // tốc độ bắn mặc định
const float SHOOTER_RPM_STEP     = 100.0f;    // D-pad trái/phải đổi tốc độ bắn mỗi lần bấm
const float SHOOTER_RPM_MIN      = 500.0f, SHOOTER_RPM_MAX = 8000.0f;
const float SHOOTER_RAMP_RPM_S   = 6000.0f;   // tốc độ tăng/giảm lệnh (chống sụt áp, giật)
const float SHOOTER_KFF = 0.15f;              // PWM trên mỗi rpm (đo: PWM_MAX / rpm tối đa khi không tải)
const float SHOOTER_KP  = 0.10f;              // PWM trên mỗi rpm sai
const float SHOOTER_KI  = 0.50f;              // PWM trên mỗi (rpm·s)
const float SHOOTER_KD  = 0.0f;
const float SHOOTER_I_MAX_PWM = 400.0f;       // chặn phần tích phân
const float SHOOTER_TOL_RPM   = 120.0f;       // coi là "đã đủ tốc độ" khi |sai| < max(TOL, 4%)
const uint32_t SHOOTER_AT_SPEED_MS = 250;     // phải giữ trong dung sai bấy lâu

// =====================================================================
// ===== LIFT nâng hạ (L298N #4, motor 1) — PID vị trí bằng encoder =====
// Lúc cấp điện coi như lift đang ở ĐÁY (= 0 mm). Gõ 'l' ở Serial để đặt lại 0 tại chỗ.
// =====================================================================
// Encoder lift: đĩa rãnh trên trục NEMA 17 + LM393, 1 kênh (ESP32#2 GPIO7) => ESP32#2 chỉ đếm cạnh, không biết chiều.
// LIFT_ENC_SINGLE 1: ESP32#1 lấy chiều theo lệnh motor lift (lệnh lên => count tăng). Lift tự trôi xuống khi motor tắt sẽ bị đếm sai chiều.
#define LIFT_ENC_SINGLE 1
const float LIFT_COUNTS_PER_MM = 4.0f;        // HIỆU CHỈNH: nâng đúng 100 mm, đọc count (gõ 'e'), chia 100 (mỗi rãnh = 2 count)
const int   LIFT_ENC_SIGN      = 1;           // chỉ dùng khi LIFT_ENC_SINGLE 0 (encoder 2 kênh): -1 nếu nâng lên mà count GIẢM
const int   LIFT_DIR           = 1;           // -1 nếu bấm lên mà lift đi xuống (hoặc đảo MOTOR_INVERT)
const float LIFT_MAX_MM        = 450.0f;      // hành trình tối đa
const float LIFT_JOG_MMS       = 120.0f;      // tốc độ nâng/hạ khi giữ D-pad (mm/s)
const float LIFT_MAX_LEAD_MM   = 25.0f;       // đích không được vượt xa vị trí thật quá mức này (an toàn)
const float LIFT_KP = 14.0f;                  // PWM trên mỗi mm sai
const float LIFT_KI = 8.0f;                   // PWM trên mỗi (mm·s) — giữ chống trọng lực
const float LIFT_KD = 0.6f;                   // PWM trên mỗi mm/s
const float LIFT_I_MAX_PWM = 350.0f;
const int   LIFT_PWM_MAX_UP   = 900;          // giới hạn PWM khi đi lên
const int   LIFT_PWM_MAX_DOWN = 600;          // giới hạn PWM khi đi xuống (nhẹ hơn vì có trọng lực)
const float LIFT_HOLD_PWM  = 120.0f;          // PWM nuôi sẵn để giữ tải khi đang ở trên cao
const float LIFT_TOL_MM    = 3.0f;            // coi là "tới nơi" khi |sai| < giá trị này
const uint32_t LIFT_STALL_MS = 1500;          // PWM lớn mà không nhúc nhích bấy lâu => ngắt motor (chống cháy)

// =====================================================================
// ===== SERVO qua hub Ohstem driverV2 (thư viện OhStemRobotics) =====
// Servo 1 = gắp bóng (góc), Servo 2 = dẫn bóng (xoay liên tục 360°)
// =====================================================================
const bool GRIP_OPEN_AT_BOOT = true;
const int  GRIP_OPEN_DEG  = 100;    // chỉnh theo cơ khí
const int  GRIP_CLOSE_DEG = 40;
const uint32_t GRIP_SETTLE_MS = 350;      // thời gian chờ khi auto gắp/nhả
const int  GUIDE_SPEED_PCT = 100;         // 1..100, chiều quay: đổi dấu nếu quay ngược
const int  GUIDE_STOP_TRIM_DEG = 0;       // servo 360° không đứng yên ở đúng 90°: chỉnh ±vài độ cho hết trôi
const bool GUIDE_HOLD_TO_RUN = true;      // true: giữ nút tròn thì quay; false: bấm bật/tắt

// =====================================================================
// ===== Lái thủ công =====
// =====================================================================
const float MANUAL_MAX_MMS = 900.0f;
const float MANUAL_MAX_W   = 4.0f;       // rad/s
const float MANUAL_SLOW    = 0.35f;      // giữ L2: chạy chậm
const float STICK_DEADBAND = 0.08f;
const float DRIVER_VIEW_DEG = 90.0f;     // lái theo sân: người lái nhìn về hướng này (90 = nhìn về +y)
const float HOLD_KP = 5.0f;              // giữ hướng bằng yaw
const bool  HEADING_HOLD_DEFAULT = true;

const uint32_t CONTROL_PERIOD_US = 10000;   // vòng điều khiển 100 Hz (mỗi vòng đọc I2C 2 board)
