# ORC_RUNNER — Path Planner + firmware robot ORC

App web để vẽ đường chạy tự động cho robot mecanum ORC, và firmware cho 3 board trên xe (mỗi cơ cấu một file).

```
index.html                       ← app (deploy lên GitHub Pages)
examples/                        ← path mẫu (demo.json, sa bàn ORC 2026)
firmware/
  ORC_Robot/    ESP32#1 — MASTER: điều khiển motor, tay cầm, chạy tự động
    ORC_Robot.ino    file chính: setup/loop, Serial
    config.h         ★ TẤT CẢ chân + hằng số hiệu chỉnh
    Motors.h         4 mạch L298N, PWM trên chân IN
    Drive.h          4 bánh mecanum
    Odometry.h       toạ độ x, y, yaw đọc từ Pico
    Encoders.h       encoder đọc từ ESP32#2
    Intake.h         hút bóng
    Shooter.h        bắn banh — PID tốc độ (encoder)
    Lift.h           nâng hạ — PID vị trí (encoder) + giữ tải
    Servos.h         gắp (servo 1) + dẫn bóng (servo 2, 360°) — hub Ohstem driverV2
    Pad.h  Manual.h  Auto.h     tay cầm / chạy thủ công / chạy tự động
    PidCtrl.h  Common.h  orc_protocol.h
    path_data.h      đường đi do app tạo ra (chép đè vào đây)
  ORC_Encoder/  ESP32#2 — I2C slave 0x42: đếm 2 encoder (bắn banh, nâng hạ)
  ORC_Pico/     Pico (RPi) — I2C slave 0x43: BMI160 + chuột optical flow => x, y, yaw
```

`orc_protocol.h` có mặt giống hệt trong 3 thư mục (Arduino IDE không dùng chung file giữa các sketch). Sửa một nơi thì chép sang 2 nơi còn lại.

## 1. Deploy app lên GitHub Pages
1. Repo này: https://github.com/thanh-ph08/ORC_RUNNER
2. Vào **Settings → Pages → Build and deployment**: Source = *Deploy from a branch*, Branch = `main`, thư mục `/ (root)` → Save.
3. Đợi khoảng 1 phút, app chạy ở https://thanh-ph08.github.io/ORC_RUNNER/.

App là một file HTML duy nhất, không cần server, chạy được cả trên điện thoại. Path tự lưu trong trình duyệt. Muốn chuyển sang máy khác thì dùng "Lưu JSON".

## 2. Dùng app
**Mọi kích thước trong app tính bằng cm.** File `path_data.h` vẫn xuất ra mm cho firmware.

### Sa bàn (tab "Sa bàn")
- **📄 Nhập file sa bàn (PDF / ảnh)**
  - Với file PDF in sa bàn, app tự lấy kích thước sân theo khổ giấy (VD 364 × 364 cm), dùng hình làm nền, và tự nhận các ô màu đặc thành **vùng**.
  - Với ảnh PNG/JPG, nhập kích thước sân trước rồi mới nhập ảnh.
  - File PDF rất nặng (ảnh 8000 × 8000) nên nhập trên máy tính sẽ chắc ăn hơn điện thoại.
- **Sa bàn ORC 2026 có sẵn**: nạp ngay sa bàn Explorer Offline 2026–2027 đã tách sẵn 34 vùng (file `examples/orc-2026-sa-ban.json`).
- Mỗi vùng chọn một trong ba loại:
  - **Bỏ qua**: không làm gì.
  - **⛔ Tránh**: xe không được đi vào, tính cả *khoảng cách an toàn* xung quanh.
  - **★ Vật**: mục tiêu.

  Các vùng cùng màu có hàng "tất cả" để đổi loại một lần.
- Nút **▦ Vùng** trên sân: chạm vào một vùng để chọn và sửa nó, kéo trên sân để vẽ vùng mới.
- **Viền an toàn quanh tường** (mặc định 3 cm) hiện là dải cam đứt nét. Thân xe (tính theo góc θ) không được lấn vào viền, trừ điểm có hành động ép tường.
- **Tự lùi điểm vào trong** (bật sẵn): khi thêm, kéo hoặc nhập toạ độ một điểm mà xe chạm viền hay vùng tránh, app tự dời điểm tới vị trí hợp lệ gần nhất. Nút **↩ Lùi tất cả điểm vào trong** dùng sau khi đổi kích thước xe hoặc độ rộng viền.
- **⛔ Tự né vùng tránh**: app tự chèn điểm đi lướt để đường đi vòng qua các vùng tránh (dùng thuật toán tìm đường A*). Những đoạn vẫn còn chạm vùng tránh sẽ hiện chấm đỏ và cảnh báo trong tab Xuất.

### Kích thước xe (tab "Robot")
- Nhập dài × rộng xe (cm), tính cả phần khung nhô ra. App dùng kích thước này để kiểm tra va chạm và đặt điểm trước vật.
- Hai số này phải khớp `ROBOT_LEN_MM` / `ROBOT_WID_MM` trong `ORC_Robot.ino` (mm = cm × 10).

### Điểm & hành động (tab "Điểm")
- **＋ Điểm**: chạm lên sân để thêm điểm tiếp theo. Điểm #0 là vị trí xuất phát.
- Kéo điểm để di chuyển, kéo nút cam **θ** để xoay hướng xe.
- **🎯 Đặt điểm trước vật**: đặt điểm sát một vật đã chọn, mặt trước xe quay vào vật, chừa khoảng hở như cài trong tab Robot.
- **Dừng tại điểm**: nếu bỏ chọn, xe đi lướt qua điểm đó theo đường cong.
- **Hành động** tại mỗi điểm:
  | Hành động | Ở điểm Dừng | Ở điểm đi qua |
  |---|---|---|
  | ⬆ Lift (cm) | chờ xong hoặc chạy song song | chạy song song |
  | ⏱ Chờ (ms) | ✓ | ✗ |
  | ▮ Ép tường | ✓ (sửa toạ độ + góc) | ✗ |
  | ★ Hành động riêng (id, tham số) | ✓ | chạy song song |
| ⟳ Intake (bật / tắt / đảo) | ✓ | ✓ |
| 🎯 Bắn banh (rpm, 0 = tắt) | chờ đủ tốc độ hoặc chạy song song | chạy song song |
| ✊ Gắp / nhả (servo 1) | chờ 0,35 s hoặc song song | chạy song song |
| ➜ Dẫn bóng (servo 2: % và ms) | chờ hoặc chạy song song | chạy song song |
- Nút ▶ phía dưới sân mô phỏng xe chạy. Màu đường đi thể hiện tốc độ: xanh là chậm, đỏ là nhanh.
- Tab **Xuất**: nút **⬇ Tải path_data.h** rồi chép đè vào `firmware/ORC_Robot/path_data.h`. Nút **⇋ Lật trái/phải** tạo path cho phía sân đối diện.

Hệ toạ độ: gốc ở góc dưới-trái sân, x sang phải, y lên trên. θ = 0 là hướng +x, chiều dương ngược chiều kim đồng hồ.

## 3. Phần cứng và đấu dây

Kiến trúc: ESP32#1 là I2C master (SDA 21, SCL 22), cùng một bus với ESP32#2 (0x42), Pico (0x43) và hub Ohstem driverV2 (0x54). Cần trở kéo lên 3.3V (2.2k–4.7k) trên SDA/SCL, GND chung tất cả board.

### Motor — 4 mạch L298N, PWM trên IN, jumper ENA/ENB giữ nguyên
| Mạch | IN1 IN2 IN3 IN4 (ESP32#1) | Motor |
|---|---|---|
| L298N #1 | 32 33 25 26 | IN1/IN2 = bánh trước-trái (FL), IN3/IN4 = trước-phải (FR) |
| L298N #2 | 27 14 12 13 | sau-trái (RL), sau-phải (RR) |
| L298N #3 | 15 2 4 16 | intake (15, 2), bắn banh (4, 16) |
| L298N #4 | 17 5 18 19 | nâng hạ (17, 5); 18, 19 để trống |

Bảng này chỉ là cách mình ghép 4 hàng chân bạn đưa vào từng motor; đổi trong `MOTOR_IN1/MOTOR_IN2` ở `config.h`. GPIO 12, 2, 15, 5 là chân strapping của ESP32: nếu board không boot hoặc không nạp được thì rút các dây đó ra lúc cấp nguồn.

### Encoder (ESP32#2) — "Rotary Encoder 100 xung 5VDC"
Dây đỏ = nguồn, đen = GND, 2 dây còn lại = A, B (collector hở). **Cấp nguồn 3.3V** để tín hiệu ra 3.3V; cấp 5V thì tín hiệu 5V sẽ vượt mức chịu của chân ESP32. Chân tạm: bắn banh A/B = 32/33, nâng hạ A/B = 25/26, sửa ở đầu `ORC_Encoder.ino`. Đọc x4 => 400 count/vòng.

### Pico
| Chức năng | Chân Pico |
|---|---|
| I2C slave (nối ESP32#1) | SDA GP4, SCL GP5 |
| BMI160 (I2C1) | SDA GP6, SCL GP7 (nằm phẳng, trục Z hướng lên) |
| Chuột USB (PIO-USB) | D+ GP2, D- GP3, 5V cho chuột, 15k xuống GND ở mỗi chân D+/D- |

### Servo
Hub Ohstem **driverV2** (I2C 0x54) với thư viện OhStemRobotics: servo 1 = gắp bóng (S1), servo 2 = dẫn bóng 360° (S2).

## 4. Nạp code

**ESP32#1 (`ORC_Robot`)**
1. Arduino IDE → Preferences → Additional Boards Manager URLs, thêm:
   `https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`
2. Boards Manager: cài **esp32_bluepad32**. Board: **ESP32 Dev Module** (nhóm *ESP32 + Bluepad32*).
3. Thư viện **OhStemRobotics**: tải https://github.com/truongphat2307/ohstem-orc-hub-robotics, chép thư mục vào `Documents/Arduino/libraries/`.
4. Sửa `config.h` nếu cần, nạp. Ghép tay cầm: PS4 giữ **Share + PS**, PS5 giữ **Create + PS**.

**ESP32#2 (`ORC_Encoder`)**: board ESP32 Dev Module (core esp32 hoặc esp32_bluepad32 đều được), không cần thư viện. Chưa nối ESP32#1 vẫn mở Serial Monitor được: xoay encoder bằng tay sẽ thấy count thay đổi.

**Pico (`ORC_Pico`)**
1. Boards Manager: **Raspberry Pi Pico/RP2040** (Earle Philhower). Board: Raspberry Pi Pico.
2. Tools → **CPU Speed = 120 MHz**, Tools → **USB Stack = Adafruit TinyUSB**.
3. Library Manager: **Adafruit TinyUSB Library** (≥ 3.0) và **Pico PIO USB**.
4. Nạp. Serial Monitor (USB của Pico, 115200) in toạ độ, trạng thái chuột và IMU.

## 5. Tay cầm

| Nút | Chức năng |
|---|---|
| Cần trái / cần phải | chạy / xoay |
| Giữ L2 | chạy chậm |
| D-pad ↑ / ↓ | nâng / hạ lift (giữ). Thả nút: lift đứng giữ vị trí bằng PID |
| D-pad ← / → | giảm / tăng tốc độ bắn 100 rpm |
| △ | bật/tắt intake (giữ R1: đảo chiều xả kẹt) |
| ✕ | bật/tắt bắn banh (PID giữ rpm) |
| ◯ | dẫn bóng (servo 2, giữ để quay; đổi `GUIDE_HOLD_TO_RUN` để thành bật/tắt) |
| □ | gắp / nhả (servo 1, bấm đổi trạng thái) |
| L3 / R3 | bật/tắt lái theo sân / giữ hướng bằng yaw |
| Share | đặt toạ độ về điểm xuất phát |
| **Giữ L1+R1, bấm Options** | **chạy tự động** |
| PS / Share / gạt mạnh cần | huỷ tự động |

Mất tay cầm quá 0,3 s: xe dừng, intake, bắn, dẫn bóng tắt, lift giữ nguyên.

Serial Monitor của ESP32#1 (115200): `r` về điểm xuất phát, `a` chạy tự động, `s` dừng, `t` bật/tắt in, `f` / `g` / `z` / `c` hiệu chỉnh chuột và gyro, `e` in count encoder, `l` đặt lift 0 mm tại chỗ, `+` / `-` / `x` chỉnh và bật/tắt bắn.

## 6. Hiệu chỉnh (theo thứ tự)

1. **Chiều motor xe**: đẩy cần tới thì cả 4 bánh quay tới, bánh nào ngược thì đổi `MOTOR_INVERT`.
2. **Pico**: mở Serial Monitor Pico, giữ yên xe 1,5 s để đo bias gyro. Cắm chuột, thấy `chuột:OK`.
3. **Chuột optical flow** (hằng số ở đầu `ORC_Pico.ino`): gõ `z` ở ESP32#1, đẩy xe đúng 1000 mm, gõ `f`. `FLOW_MM_PER_COUNT = 1000 / số đếm`. Đẩy tới phải ra x dương, đẩy sang trái phải ra y dương; không thì chỉnh `FLOW_SWAP_XY` / `FLOW_SIGN_*`. Đo vị trí chuột so với tâm quay của xe và điền `FLOW_POS_X_MM` / `FLOW_POS_Y_MM`: thiếu bước này, xe xoay tại chỗ sẽ bị tính là đi.
4. **Gyro**: `z`, quay xe đúng 10 vòng, `g`. `GYRO_SCALE = 3600 / số độ`. Quay ngược chiều kim đồng hồ mà yaw giảm thì `GYRO_SIGN = -1`.
5. **KS / KV xe** (`config.h`): KS là PWM nhỏ nhất làm xe nhúc nhích, `KV ≈ (PWM − KS) / v`.
6. **PID bám path** (đầu `Auto.h`): tăng `AUTO_KP_XY` tới khi bám tốt mà không rung, thêm `AUTO_KI_XY` nếu dừng thiếu vài mm.
7. **Encoder bắn banh**: bật bắn, nếu rpm đo là âm thì `SHOOTER_ENC_SIGN = -1`. Nếu motor quay ngược thì `SHOOTER_DIR = -1`. Đo rpm tối đa không tải để đặt `SHOOTER_KFF = PWM_MAX / rpm tối đa`, rồi tăng `SHOOTER_KP`, sau đó `SHOOTER_KI` tới khi rpm bám nhanh mà không dao động. Nếu PWM chạm trần mà không quay 1,5 s, firmware tự tắt bắn.
8. **Lift**: lúc cấp điện lift phải ở đáy. Nâng tay (hoặc D-pad) đúng 100 mm, gõ `e`, `LIFT_COUNTS_PER_MM = count / 100`. Count giảm khi nâng lên thì `LIFT_ENC_SIGN = -1`; bấm lên mà lift đi xuống thì `LIFT_DIR = -1`. Chỉnh `LIFT_HOLD_PWM` cho vừa đủ giữ tải, rồi `LIFT_KP`, `LIFT_KI`. Lift kẹt quá 1,5 s thì firmware ngắt motor (liftFault), ra lệnh mới để thử lại.
9. **Servo**: chỉnh `GRIP_OPEN_DEG`, `GRIP_CLOSE_DEG`, và `GUIDE_STOP_TRIM_DEG` cho servo 360° đứng yên.

## 7. Giao thức I2C (`orc_protocol.h`)

- ESP32#2 (0x42): ESP32#1 đọc `EncoderPacket` (18 byte: seq, count và count/s của 2 encoder, CRC-8). Ghi 1 byte để xoá count.
- Pico (0x43): ESP32#1 đọc `PosePacket` (40 byte: x, y, yaw, vx, vy, w, cờ trạng thái, bộ đếm thô để hiệu chỉnh, CRC-8). Ghi `0x10 + x,y,yaw` để đặt toạ độ, `0x11` đo lại gyro, `0x12` xoá bộ đếm thô.
- Gói sai CRC hoặc không đủ byte bị bỏ qua và đếm lỗi (hiện trong dòng telemetry). Mất Pico quá 80 ms thì tự động dừng; mất ESP32#2 thì shooter và lift tắt motor. Nếu hay lỗi, hạ `I2C_CLOCK` xuống 100000.

## 8. Viết hành động riêng
Cuối file `Auto.h`, hàm `runCustomAction(id, param, start)`: thêm một `case` cho mỗi id bạn đặt trong app. Hàm trả về `true` khi xong. Không dùng `delay()` trong hàm này. Các cơ cấu đã có hàm sẵn: `intakeCommand`, `shooterSetRpm` / `shooterEnable`, `liftSetTarget`, `gripSet`, `guideSet`.
