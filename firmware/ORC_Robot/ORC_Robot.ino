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
 * Thư viện: không cần cài thêm (hub servo Ohstem điều khiển thẳng qua I2C trong Servos.h)
 * Serial 115200: r = về điểm xuất phát, a = chạy tự động, s = dừng, t = bật/tắt in toạ độ,
 *   f = in số đếm chuột thô, g = in góc gyro cộng dồn, z = xoá bộ đếm hiệu chỉnh, c = RPi đo lại gyro,
 *   e = in count encoder, l = đặt lift 0 mm tại chỗ, +/- = tốc độ bắn ± 100 rpm, x = bật/tắt bắn
 */
#include <Arduino.h>
#if __has_include(<esp_system.h>)
#include <esp_system.h>
#endif
#include <Wire.h>
#include <math.h>
#include <Bluepad32.h>

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
      case 'e': Log.printf("[ENC] shooter=%ld (%.0f rpm)  lift=%ld count (%.1f mm, ESP32#2 đếm %ld cạnh)\n", (long)encShooterCount, shooterRpm,
                              (long)(liftCount - liftZeroCounts), liftPosMM, (long)encLiftCount); break;
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
  if (!telemetryOn || millis() - last < 1000) return;   // 1 s/dòng: log qua I2C tới Pi không bị tràn
  last = millis();
  Log.printf("x=%6.0f y=%6.0f th=%6.1f° | shoot %s %4.0f/%4.0f rpm%s | lift %5.1f→%5.1f mm%s | gắp:%s nạp:%d | lỗi gói Pi=%lu, I2C enc=%lu | %s %s\n",
                poseX, poseY, poseTh * RAD_TO_DEG,
                shooterOn ? "ON " : "off", shooterRpm, shooterTargetRpm, shooterFault ? " LỖI" : "",
                liftPosMM, liftTargetMM, liftFault ? " LỖI" : "",
                gripClosed ? "đóng" : "mở", guideSpeedPct,
                (unsigned long)poseErrCount, (unsigned long)encErrCount,
                robotMode == RM_AUTO ? "AUTO" : (pidOff ? "MAN KHÔNG-PID" : "MAN"), autoStatus());
  if (!poseFresh()) {
#if PI_LINK_I2C
    Log.printf("  ! mất Raspberry Pi (toạ độ) — I2C: Pi đã đọc %lu lần, ghi %lu lần. Kiểm tra dây GPIO18/19 <-> Pi GPIO5/6, trở kéo lên 4.7k, GND, và orc_pi.py\n",
               (unsigned long)piLinkReads(), (unsigned long)piLinkWrites());
#else
    Log.println("  ! mất Raspberry Pi (toạ độ) — kiểm tra dây TX/RX và orc_pi.py");
#endif
  }
  if (!encFresh()) Log.println("  ! mất ESP32#2 (encoder)");
}

// Nếu một slave đang giữ SDA ở mức thấp (do master reset giữa lúc truyền), bơm tối đa 9 xung SCL cho nó nhả SDA rồi gửi STOP.
static void i2cBusRecover() {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, OUTPUT);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(10);
  bool stuck = digitalRead(I2C_SDA) == LOW;
  for (int i = 0; i < 9 && digitalRead(I2C_SDA) == LOW; i++) {
    digitalWrite(I2C_SCL, LOW);  delayMicroseconds(10);
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(10);
  }
  pinMode(I2C_SDA, OUTPUT);                 // STOP: SDA lên cao khi SCL đang cao
  digitalWrite(I2C_SDA, LOW);  delayMicroseconds(10);
  digitalWrite(I2C_SCL, HIGH); delayMicroseconds(10);
  digitalWrite(I2C_SDA, HIGH); delayMicroseconds(10);
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  Serial.printf("[I2C] SDA %s lúc khởi động%s\n", stuck ? "BỊ KẸT THẤP" : "bình thường",
                stuck && digitalRead(I2C_SDA) == HIGH ? " -> đã gỡ" : (stuck ? " -> VẪN KẸT, kiểm tra nguồn/GND/dây của các board I2C" : ""));
}

RTC_NOINIT_ATTR static uint32_t lastStage;           // phần đang chạy — còn giữ sau khi watchdog reset, để biết treo ở đâu
RTC_NOINIT_ATTR static uint32_t wdtMagic;            // = WDT_MAGIC nếu lần reset trước do watchdog tự viết bên dưới
#define WDT_MAGIC 0xC0DE7EEDu
#include <esp_timer.h>
// Watchdog tự viết (không phụ thuộc core): timer nền kiểm tra mỗi 0.5 s, loop() đứng yên > 5 s thì ghi dấu rồi esp_restart().
static volatile uint32_t loopAliveMs = 0;
static void swWdtCheck(void *) {
  uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
  if (loopAliveMs && now - loopAliveMs > 5000) { wdtMagic = WDT_MAGIC; esp_restart(); }
}
static void swWdtBegin() {
  static esp_timer_handle_t h;
  esp_timer_create_args_t a = {};
  a.callback = swWdtCheck;
  a.name = "loopwdt";
  if (esp_timer_create(&a, &h) == ESP_OK) esp_timer_start_periodic(h, 500000);
}
// ---- Đo thời gian từng phần của vòng lặp: in "[LOOP] ..." mỗi 2 s để biết phần nào làm chậm ----
enum { T_PAD, T_LOGIMU, T_ODO, T_ENC, T_CTRL, T_SERVO, T_TELE, T_N };
static const char *T_NAME[T_N] = {"BP32", "log/imu/serial", "pi-link", "encoder(ESP32#2)", "điều khiển", "servo(hub)", "telemetry"};

void setup() {
  Serial.begin(115200);
  delay(300);
  Log.println("\n=== ORC Robot — ESP32#1 ===");
#if __has_include(<esp_system.h>)
  {
    esp_reset_reason_t r = esp_reset_reason();
    const char *why = r == ESP_RST_POWERON ? "bật nguồn / cắm điện (bình thường)" :
                      r == ESP_RST_BROWNOUT ? "BROWNOUT: SỤT NGUỒN (nguồn yếu / động cơ-servo kéo sập 3V3)" :
                      r == ESP_RST_PANIC ? "PANIC: code bị lỗi (crash)" :
                      r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT ? "WATCHDOG: vòng lặp bị treo" :
                      r == ESP_RST_SW ? "khởi động lại bằng phần mềm" :
                      r == ESP_RST_EXT ? "chân EN / nút reset" : "khác";
    Log.printf("[BOOT] lý do khởi động lại: %s (mã %d)\n", why, (int)r);
    bool hung = (r == ESP_RST_SW && wdtMagic == WDT_MAGIC) || r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT;
    if (hung) Log.printf("[BOOT] lần trước vòng lặp TREO > 5 s, kẹt ở phần: %s\n",
                         (lastStage >= 1 && lastStage <= T_N) ? T_NAME[lastStage - 1] : "không rõ");
    lastStage = 0; wdtMagic = 0;
  }
#endif

  motorsBegin();
  i2cBusRecover();                          // gỡ kẹt bus (slave giữ SDA thấp sau khi ESP32 reset giữa chừng)
  Wire.begin(I2C_SDA, I2C_SCL, I2C_CLOCK);
  Wire.setTimeOut(15);                      // ms — slave treo thì không kẹt cả vòng điều khiển
  odometryBegin();                          // liên kết tới Raspberry Pi (I2C slave trên Wire1, hoặc UART2 nếu PI_LINK_I2C 0)
  servosBegin();                            // hub.begin() gọi lại Wire.begin(), nên đặt lại clock + timeout sau đó
  Wire.setClock(I2C_CLOCK);
  Wire.setTimeOut(15);
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
  swWdtBegin();                             // vòng lặp kẹt > 5 s thì tự reset, lần sau in "[BOOT] lần trước vòng lặp TREO ..."
}

static uint32_t tMax[T_N], loopCount = 0, ctrlCount = 0, loopMaxUs = 0;
#define TIMED(id, code) do { lastStage = (id) + 1; uint32_t _t0 = micros(); code; uint32_t _d = micros() - _t0; if (_d > tMax[id]) tMax[id] = _d; } while (0)

static void loopReport() {
  static uint32_t last = 0;
  if (millis() - last < 2000) return;
  float sec = (millis() - last) / 1000.0f;
  last = millis();
  int worst = 0;
  for (int i = 1; i < T_N; i++) if (tMax[i] > tMax[worst]) worst = i;
  Log.printf("[LOOP] %lu vòng/s, %lu nhịp điều khiển/s, vòng chậm nhất %.1f ms | chậm nhất: %s %.1f ms (enc %.1f, servo %.1f, BP32 %.1f) | I2C Pi: đọc %lu ghi %lu\n",
             (unsigned long)(loopCount / sec), (unsigned long)(ctrlCount / sec), loopMaxUs / 1000.0f,
             T_NAME[worst], tMax[worst] / 1000.0f, tMax[T_ENC] / 1000.0f, tMax[T_SERVO] / 1000.0f, tMax[T_PAD] / 1000.0f,
             (unsigned long)piLinkReads(), (unsigned long)piLinkWrites());
  loopCount = ctrlCount = loopMaxUs = 0;
  for (int i = 0; i < T_N; i++) tMax[i] = 0;
}

void loop() {
  loopAliveMs = (uint32_t)(esp_timer_get_time() / 1000);
  uint32_t loopT0 = micros();
  TIMED(T_PAD, BP32.update());
  TIMED(T_LOGIMU, { Log.pump(); imuStep(); handleSerial(); });

  static uint32_t lastCtrl = micros();
  uint32_t now = micros();
  if (now - lastCtrl >= CONTROL_PERIOD_US) {
    float dt = (now - lastCtrl) * 1e-6f;
    lastCtrl = now;
    if (dt > 0.05f) dt = 0.05f;
    ctrlCount++;

    TIMED(T_ODO, odometryUpdate());
    TIMED(T_ENC, encodersUpdate());
    TIMED(T_CTRL, {
      padRead();
      if (robotMode == RM_AUTO) autoUpdate(dt);
      else manualUpdate(dt);
      shooterUpdate(dt);
      liftUpdate(dt);
    });
    TIMED(T_SERVO, servosUpdate());
  }
  TIMED(T_TELE, telemetry());
  loopReport();
  loopCount++;
  uint32_t d = micros() - loopT0;
  if (d > loopMaxUs) loopMaxUs = d;
}
