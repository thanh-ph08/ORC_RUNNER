/*
 * ORC_Robot.ino — ESP32#1 (I2C MASTER, board "esp32_bluepad32 > ESP32 Dev Module")
 *
 * Mỗi cơ cấu một file (cùng thư mục):
 *   config.h     chân + hằng số hiệu chỉnh (chỉ sửa chỗ này)
 *   Motors.h     L298N × 4 (PWM trên IN1/IN2)
 *   Drive.h      4 bánh mecanum
 *   Odometry.h   toạ độ x, y, yaw từ RPi (UART)
 *   Imu.h        đọc MPU6050 (I2C 0x68) rồi gửi mẫu thô sang RPi qua UART
 *   Encoders.h   encoder từ ESP32#2
 *   Intake.h     hút bóng
 *   Shooter.h    bắn banh, PID tốc độ
 *   Lift.h       nâng hạ, PID vị trí
 *   Servos.h     gắp (servo 1) + dẫn bóng (servo 2, 360°) qua hub Ohstem driverV2
 *   Pad.h        tay cầm Bluepad32          Manual.h  chạy thủ công        Auto.h  chạy tự động
 *   path_data.h  đường đi do app tạo ra (chép đè vào đây)
 *
 * Thư viện: OhStemRobotics (https://github.com/truongphat2307/ohstem-orc-hub-robotics, chép vào Arduino/libraries)
 * Serial 115200: r = về điểm xuất phát, a = chạy tự động, s = dừng, t = bật/tắt in toạ độ,
 *   f = in số đếm chuột thô, g = in góc gyro cộng dồn, z = xoá bộ đếm hiệu chỉnh, c = RPi đo lại gyro,
 *   e = in count encoder, l = đặt lift 0 mm tại chỗ, +/- = tốc độ bắn ± 100 rpm, x = bật/tắt bắn
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Bluepad32.h>
#include <OhStemRobotics.h>

#include "path_data.h"
#include "config.h"
#include "orc_protocol.h"
#include "Common.h"
#include "Log.h"
#include "PidCtrl.h"
#include "Motors.h"
#include "Odometry.h"
#include "Imu.h"
#include "Encoders.h"
#include "Drive.h"
#include "Intake.h"
#include "Shooter.h"
#include "Lift.h"
#include "Servos.h"
#include "Pad.h"
#include "Auto.h"
#include "Manual.h"

static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'i': {   // quét I2C: ESP32#2 = 0x42, hub Ohstem = 0x54
        Log.println("[I2C] Quét bus (lỗi 2 = trống, 5 = timeout):");
        int found = 0;
        for (uint8_t ad = 1; ad < 127; ad++) {
          Wire.beginTransmission(ad);
          uint8_t er = Wire.endTransmission();
          if (er == 0) { Log.printf("  thấy thiết bị 0x%02X\n", ad); found++; }
          else if (er != 2) Log.printf("  lỗi %u tại 0x%02X\n", er, ad);
        }
        Log.printf("[I2C] Xong, %d thiết bị\n", found);
        break;
      }
      case 'r': resetPoseToStart(); break;
      case 'a': startAuto(); break;
      case 's': if (robotMode == RM_AUTO) stopAuto("lệnh Serial"); stopDrive(); break;
      case 't': telemetryOn = !telemetryOn; break;
      case 'f': Log.printf("[CAL] Chuột quang: x = %ld, y = %ld count\n", (long)rawFlowX, (long)rawFlowY); break;
      case 'g': Log.printf("[CAL] Gyro cộng dồn: %.1f độ\n", gyroAccumDeg); break;
      case 'z': flowClearRaw(); Log.println("[CAL] Đã xoá bộ đếm"); break;
      case 'c': flowCalibrateGyro(); break;
      case 'e': Log.printf("[ENC] shooter=%ld (%.0f rpm)  lift=%ld count (%.1f mm)\n", (long)encShooterCount, shooterRpm,
                              (long)encLiftCount, liftPosMM); break;
      case 'l': liftZeroHere(); break;
      case '+': shooterAdjust(+SHOOTER_RPM_STEP); Log.printf("[SHOOT] %.0f rpm\n", shooterTargetRpm); break;
      case '-': shooterAdjust(-SHOOTER_RPM_STEP); Log.printf("[SHOOT] %.0f rpm\n", shooterTargetRpm); break;
      case 'x': shooterToggle(); break;
      default: break;
    }
  }
}

static void telemetry() {
  static uint32_t last = 0;
  if (!telemetryOn || millis() - last < 250) return;
  last = millis();
  Log.printf("x=%6.0f y=%6.0f th=%6.1f° | shoot %s %4.0f/%4.0f rpm%s | lift %5.1f→%5.1f mm%s | gắp:%s dẫn:%d | lỗi gói Pi=%lu, I2C enc=%lu | %s %s\n",
                poseX, poseY, poseTh * RAD_TO_DEG,
                shooterOn ? "ON " : "off", shooterRpm, shooterTargetRpm, shooterFault ? " LỖI" : "",
                liftPosMM, liftTargetMM, liftFault ? " LỖI" : "",
                gripClosed ? "đóng" : "mở", guideSpeedPct,
                (unsigned long)poseErrCount, (unsigned long)encErrCount,
                robotMode == RM_AUTO ? "AUTO" : "MAN", autoStatus());
  if (!poseFresh()) Log.println("  ! mất Raspberry Pi (toạ độ) — kiểm tra dây TX/RX và orc_pi.py");
  if (!encFresh()) Log.println("  ! mất ESP32#2 (encoder)");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Log.println("\n=== ORC Robot — ESP32#1 ===");

  motorsBegin();
  Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK);
  Wire.setTimeOut(15);                      // ms — slave treo thì không kẹt cả vòng điều khiển
  odometryBegin();                          // UART2 tới Raspberry Pi
  servosBegin();                            // hub.begin() gọi lại Wire.begin(), nên đặt lại clock sau đó
  Wire.setClock(I2C_CLOCK);
  imuBegin();                               // MPU6050 trên Control Hub: ESP32#1 là master duy nhất của bus
  touchBegin();
  shooterBegin();
  liftBegin();
  servosInitialPose();

  delay(100);
  encodersUpdate();
  odometryUpdate();
  liftZeroHere();
  setPose(START_X_MM, START_Y_MM, START_TH_RAD);
  holdTarget = START_TH_RAD;

  Log.printf("Bluepad32 firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* bd = BP32.localBdAddress();
  Log.printf("BD Addr: %02X:%02X:%02X:%02X:%02X:%02X\n", bd[0], bd[1], bd[2], bd[3], bd[4], bd[5]);
  BP32.setup(&onConnectedController, &onDisconnectedController);
#if PAD_FORGET_KEYS_ON_BOOT
  BP32.forgetBluetoothKeys();               // xoá khoá cũ: mỗi lần bật phải ghép lại (Share + PS), nhưng ghép chắc chắn nhất
#endif
  BP32.enableVirtualDevice(false);          // không tạo "chuột ảo" từ touchpad PS4/PS5
  Log.println("Ghép tay PS4: giữ Share + PS. Tay PS5: giữ Create + PS.");
}

void loop() {
  BP32.update();
  Log.pump();
  imuStep();
  handleSerial();

  static uint32_t lastCtrl = micros();
  uint32_t now = micros();
  if (now - lastCtrl >= CONTROL_PERIOD_US) {
    float dt = (now - lastCtrl) * 1e-6f;
    lastCtrl = now;
    if (dt > 0.05f) dt = 0.05f;

    odometryUpdate();
    encodersUpdate();
    padRead();
    if (robotMode == RM_AUTO) autoUpdate(dt);
    else manualUpdate(dt);
    shooterUpdate(dt);
    liftUpdate(dt);
    servosUpdate();
  }
  telemetry();
}
