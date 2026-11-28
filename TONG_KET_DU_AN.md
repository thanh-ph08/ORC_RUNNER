# ORC Runner — tổng kết dự án

Cập nhật: 28/11/2026 · Repo: `thanh-ph08/ORC_RUNNER` · Commit mới nhất: `ba6e2ce`

Robot thi đấu ORC (sân "Explorer Offline 2026–2027", 364 × 364 cm, tường cao 2 cm) chạy được cả **lái tay** bằng tay cầm PS4/PS5 lẫn **tự động** theo đường đi vẽ trên app web. Firmware đã biên dịch thật trên core `esp32_bluepad32` 4.1.0: 0 lỗi, 0 cảnh báo. Phần lớn hệ số hiệu chỉnh vẫn là số đoán, chưa đo trên xe.

---

## 1. Phần cứng

| Khối | Thiết bị | Vai trò |
|---|---|---|
| ESP32#1 | ESP32 Dev Module (Bluepad32) | Bộ não: tay cầm, điều khiển motor, bám path, giữ chỗ, lift, bắn, servo |
| ESP32#2 | OhStem ORC Control Hub rev D (ESP32-S3) | Đọc 2 encoder, gửi cho ESP32#1 qua I2C (slave 0x42). Có sẵn MPU6050 0x68 |
| Raspberry Pi 3B+ | `rpiorz`, Python | Tính toạ độ x, y, góc từ chuột optical USB + IMU; trang web giám sát cổng 8080 |
| Hub servo | OhStem driverV2 (I2C 0x54) | Servo 1 = gắp bóng, Servo 2 = cửa nạp bóng (180°) |
| Driver motor | 4 × L298N | 7 động cơ |
| Encoder bắn | Hall của motor GA25 (đã bỏ hộp số) | 44 count/vòng |
| Encoder lift | NEMA 17 làm máy phát, 2 phase qua 2 LM393 | 2 kênh lệch 90°, 200 count/vòng trục NEMA |
| Định vị | Chuột optical USB (cắm Pi) + MPU6050 trên Control Hub | Toạ độ cho chạy tự động |

Đã bỏ: cảm biến ToF VL53L0X (tường sân chỉ 2 cm), Raspberry Pico (thay bằng Pi 3B+), BMI160 (nhiễu khi đặt gần L298N, đổi sang MPU6050).

## 2. Kiến trúc và luồng dữ liệu

```
MPU6050 (Control Hub) ──bus 1──► ESP32#1 ──mẫu IMU──────────► Pi
ESP32#2 (encoder) ──────bus 1──► ESP32#1                       │ Pi ghép IMU + chuột
Hub servo 0x54 ◄────────bus 1─── ESP32#1                       │ => x, y, góc
Chuột USB ─────────────────────────────────────────────────► Pi
ESP32#1 ◄──────── bus I2C riêng (x, y, góc 100 lần/s) ──────── Pi
   └─► bám path / giữ chỗ / lift / bắn / servo / 7 motor
```

- **Bus 1** (ESP32#1 GPIO21/22): ESP32#1 là master **duy nhất**. Pi không được nối vào bus này.
- **Bus liên kết Pi ↔ ESP32#1**: Pi master (bus I2C phần mềm số 3, GPIO5/6) ↔ ESP32#1 slave 0x43 (Wire1, GPIO18/19). Trở kéo lên 4.7 kΩ về 3.3 V.
  - Pi ghi 42 byte: toạ độ (40 byte có CRC) + số mảnh log đã nhận.
  - Pi đọc 32 byte trạng thái: lệnh đặt toạ độ / đo gyro / xoá bộ đếm + một mảnh log.
  - Pi đọc 32 byte IMU: 2 mẫu MPU6050 mới nhất (ESP32#1 đọc 200 Hz).
  - Mọi gói đều có CRC-8. Bộ đếm lệnh chỉ nhận khi tăng, log dùng cơ chế gửi rồi chờ xác nhận, nên gói cũ hay gói trễ không gây lỗi.
- **Log của ESP32#1** đi qua liên kết này và hiện trên trang web của Pi: `http://rpiorz.local:8080` (lọc ESP32#1).

## 3. Đấu dây

**Motor (ESP32#1 → L298N)**

| L298N | OUT1/OUT2 (IN1, IN2) | OUT3/OUT4 (IN3, IN4) |
|---|---|---|
| #1 | Bánh trước trái (GPIO32, 33) | Bánh trước phải (GPIO25, 26) |
| #2 | Bánh sau trái (GPIO27, 14) | Bánh sau phải (GPIO12, 13) |
| #3 | Intake (GPIO15, 2) | Bánh bắn (GPIO4, 16) |
| #4 | Linear slide (GPIO17, 5) | Trống (GPIO18/19 dùng cho Pi) |

Giữ jumper ENA/ENB, GND chung. GPIO12, 2, 15, 5 là chân strapping. L298N chịu khoảng 2 A mỗi kênh, motor FTC kẹt ăn 9–11 A, nên cần tản nhiệt.

**Encoder (ESP32#2)**: bắn A → GPIO8, B → GPIO9; lift phase A → GPIO7, phase B → GPIO6; I2C SDA 11 / SCL 12. Cấp nguồn cảm biến 3.3 V.

**Pi**: liên kết GPIO5 (chân 29) / GPIO6 (chân 31) + GND. Trong `/boot/firmware/config.txt` có dòng `dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=5,i2c_gpio_scl=6,i2c_gpio_delay_us=2`.

Bảng đấu dây đầy đủ có trong Sheet "Đấu dây L298N — ORC Runner".

## 4. Tay cầm

| Nút | Chức năng |
|---|---|
| Cần trái / cần phải | Chạy tới–lùi–ngang / xoay. Giữ **R2**: chạy chậm |
| **L2** | Bật / tắt **chạy tự động** |
| D-pad ↑ / ↓ | Nâng / hạ lift (PID giữ vị trí khi thả) |
| D-pad ← / → | Giảm / tăng rpm bắn ±100 |
| ✕ | Bật / tắt bánh bắn |
| △ / giữ R1 | Bật / tắt intake / đảo chiều xả kẹt |
| ◯ (giữ) | Nạp bóng: cửa chỉ mở khi bánh bắn đủ rpm, mỗi lần 1 quả |
| □ | Gắp / nhả |
| L3 / R3 | Lái theo sân / giữ hướng bằng gyro |
| Share | Đặt toạ độ về điểm xuất phát (đang tự động thì huỷ) |
| **Options** | Bật / tắt **chế độ không PID**: lift chạy thẳng theo D-pad, bánh bắn chạy theo PWM, không cần encoder (đèn cam) |
| PS / gạt mạnh cần | Huỷ tự động |

Đèn tay cầm: xanh lá = lái tay, cam = không PID, xanh dương = tự động. Mất tay cầm thì xe dừng; sau 0.3 s tắt intake, bánh bắn và cửa nạp.

## 5. Chạy tự động

- **App ORC Path Planner** (`index.html`, GitHub Pages): vẽ sân, vùng tránh, điểm path, các bước (lift, chờ, bắn, gắp, nạp, intake, ép tường, hành động riêng), rồi xuất `path_data.h` chép vào `firmware/ORC_Robot/`.
- **Bám path**: quỹ đạo = feedforward vận tốc + PID x, y, góc. Có "đồng hồ co giãn": khi xe tụt lại hoặc bị đẩy lệch, điểm đích chờ xe bắt kịp, nên không cắt góc.
- **Giữ chỗ**: lúc đứng chờ (WAIT, lift, bắn, gắp, nạp) mà bị xe khác đẩy lệch quá 20 mm hoặc 3° thì xe tự chạy về.
- **Ép tường**: chạy vào tường để sửa lại toạ độ (hiện chạy theo thời gian, không dùng công tắc chạm).
- Tự động dừng khi mất toạ độ từ Pi, khi người lái huỷ, hoặc khi hết path.

## 6. Chỉnh ở đâu

| Chỉnh ở | Gồm | Sau khi sửa |
|---|---|---|
| App trên GitHub | Sân, path, các bước, kích thước xe, V max, ω max | Tải `path_data.h`, nạp lại ESP32#1 |
| `firmware/ORC_Robot/config.h` | Chân, chiều motor, feedforward, lift, bắn, intake, servo, lái tay | Nạp lại ESP32#1 |
| `firmware/ORC_Robot/Auto.h` | PID bám path, dung sai, giữ chỗ, ép tường | Nạp lại ESP32#1 |
| `firmware/ORC_Encoder/ORC_Encoder.ino` | Chân encoder, 1 hay 2 phase lift | Nạp lại ESP32#2 |
| `firmware/ORC_Pi/orc_pi.py` | Tỉ lệ / chiều chuột, gyro | `sudo systemctl restart orc-pi` |

Danh sách đầy đủ từng hằng số và cách đo nằm trong Doc "ORC Runner — danh sách cần chỉnh theo thực tế".

## 7. Lịch sử

| Ngày | Việc chính |
|---|---|
| Trước 17/11 | App path planner và bản firmware chạy tự động đầu tiên |
| 17/11 | Bỏ cảm biến ToF |
| 25/11 | Viết lại firmware: mỗi cơ cấu một file, 3 board. Thay Pico bằng Pi 3B+. BMI160 lỗi nhiều, đổi sang MPU6050 |
| 26/11 | Đổi chân ESP32#2 theo Control Hub. Trang web giám sát + log ESP32#1. Sửa lỗi kẹt cân gyro, chuột nhận nhầm HDMI-CEC. Gỡ kẹt bus I2C khi khởi động. Thử UART Pi ↔ ESP32#1 |
| 27/11 | UART chỉ chạy một chiều, chuyển sang **liên kết I2C riêng**. Sửa driver slave (trả gói thiếu byte, gói cũ, log vụn). Thêm watchdog và đo thời gian vòng lặp: tìm ra Pi và ESP32#1 tranh bus 1, làm encoder kẹt 1 s → chuyển IMU cho ESP32#1 đọc. Thêm giữ chỗ khi bị đụng |
| 28/11 | Encoder mới (GA25 cho bắn, NEMA 17 + 2 LM393 cho lift). Bỏ thư viện OhStemRobotics (không biên dịch trên core Bluepad32). Chế độ không PID (Options). Servo 2 thành cửa nạp chờ đủ rpm. L2 = tự động. Rà soát logic, sửa 4 lỗi, biên dịch thật được |

## 8. Các vấn đề đã gặp và cách giải

| Vấn đề | Nguyên nhân | Cách giải |
|---|---|---|
| IMU trên Pi báo `Remote I/O error`, ESP32#1 treo 1 s mỗi lần đọc encoder | Pi và ESP32#1 cùng làm master trên bus 1 | ESP32#1 đọc IMU, Pi lấy qua liên kết riêng |
| UART Pi → ESP32#1 không chạy | Chưa xác định (chiều ngược lại chạy được) | Chuyển sang I2C riêng trên GPIO18/19 |
| Pi chỉ nhận byte đầu (`0xa8` rồi toàn `0xff`) | Callback slave làm việc nặng, FIFO cạn | Dựng sẵn gói ở vòng lặp chính |
| Log vụn, lặp chữ | Driver slave trả gói cũ / trễ một nhịp | Số thứ tự mảnh log + gửi rồi chờ xác nhận |
| `ledcAttach was not declared` | Thư viện OhStemRobotics 0.2.0 cần core 3.x | Ghi thẳng thanh ghi hub 0x54 trong `Servos.h` |
| `enableLoopWDT` không có | Core Bluepad32 | Watchdog tự viết bằng `esp_timer` |
| Toạ độ trôi khi xe đứng yên (300,300 → −273,2237) | Chưa rõ: chuột hoặc `FLOW_MM_PER_COUNT` | **Chưa xử lý**, cần đo |
| Linear slide không chạy, kể cả ở chế độ không PID | Nhiều khả năng phần cứng: L298N #4, nguồn, dây GPIO17/5 | **Chưa xử lý**, cần thử motor trực tiếp |

## 9. Việc còn lại

1. **Linear slide**: kiểm tra L298N #4, nguồn 12 V, jumper ENA, dây GPIO17/GPIO5. Kiểm tra module có phải WROVER không (WROVER không dùng được GPIO16/17). Thử nối motor thẳng vào nguồn.
2. **Toạ độ**: để xe đứng yên 60 s thử trôi; đo `FLOW_MM_PER_COUNT` (đẩy xe 1 m), chiều trục chuột, `GYRO_SCALE` (xoay 10 vòng).
3. **Đo hệ số**: `SHOOTER_CPR` (quay tay 10 vòng), `SHOOTER_KFF`, `LIFT_COUNTS_PER_MM` (nâng 100 mm), feedforward bánh `KS_PWM` / `KV_PWM_PER_MMS`, góc servo gắp và cửa nạp.
4. **Chỉnh PID**: lift, bánh bắn, bám path (`AUTO_KP_XY` …), giữ chỗ.
5. **Path thi thật** trong app (hiện là path `demo`), đúng điểm xuất phát và kích thước xe.
6. Trước khi thi: `PAD_FORGET_KEYS_ON_BOOT 0`, `sudo systemctl enable orc-pi`, kiểm tra brownout khi chạy hết công suất.
7. Dọn dẹp: README còn đoạn hướng dẫn cũ (UART, Pi đọc IMU); các file thử UART không còn dùng.

## 10. File chính

| File | Nội dung |
|---|---|
| `index.html` | App ORC Path Planner |
| `firmware/ORC_Robot/` | ESP32#1: `config.h`, `Auto.h`, `Manual.h`, `Lift.h`, `Shooter.h`, `Servos.h`, `Odometry.h`, `Imu.h`, `Log.h`, `path_data.h`… |
| `firmware/ORC_Encoder/ORC_Encoder.ino` | ESP32#2: đọc encoder |
| `firmware/ORC_Pi/orc_pi.py`, `web_ui.py` | Pi: toạ độ + trang web giám sát |
| `firmware/ORC_Pi/LENH_PI.md` | Lệnh hay dùng trên Pi |
