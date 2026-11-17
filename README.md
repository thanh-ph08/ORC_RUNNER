# ORC_RUNNER — Path Planner + firmware ESP32

App web để vẽ đường chạy tự động cho robot mecanum ORC. Code ESP32 gồm 2 file: chạy thủ công và chạy tự động.

```
index.html                  ← app (deploy lên GitHub Pages)
examples/demo.json          ← path mẫu (mở bằng nút "📂 Mở JSON")
examples/orc-2026-sa-ban.json ← sa bàn ORC 2026 (364×364 cm) đã tách vùng
firmware/ORC_Robot/
  ORC_Robot.ino             ← FILE CHÍNH: phần cứng + chạy thủ công
  Auto.ino                  ← FILE PHỤ: chạy tự động
  path_data.h               ← dữ liệu path, app tạo ra, chép đè vào đây
```

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
  | 📏 ToF | ✓ (sửa 1 toạ độ) | ✗ |
  | ★ Hành động riêng (id, tham số) | ✓ | chạy song song |
- Nút ▶ phía dưới sân mô phỏng xe chạy. Màu đường đi thể hiện tốc độ: xanh là chậm, đỏ là nhanh.
- Tab **Xuất**: nút **⬇ Tải path_data.h** rồi chép đè vào `firmware/ORC_Robot/`. Nút **⇋ Lật trái/phải** tạo path cho phía sân đối diện.

Hệ toạ độ: gốc ở góc dưới-trái sân, x sang phải, y lên trên. θ = 0 là hướng +x, chiều dương ngược chiều kim đồng hồ.

## 3. Nạp code ESP32
1. Arduino IDE → Preferences → Additional Boards Manager URLs, thêm:
   `https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json`
2. Boards Manager: cài **esp32_bluepad32**. Chọn board **ESP32 Dev Module** trong mục *ESP32 + Bluepad32*.
3. Library Manager: cài **FastAccelStepper**, **SparkFun 9DoF IMU Breakout - ICM 20948**, **Bitcraze PMW3901**, **VL53L1X** (của *Pololu*, không phải bản của ST).
4. Sửa phần `CẤU HÌNH` đầu file `ORC_Robot.ino` (chân, kích thước robot, ToF, công tắc chạm, lift), rồi nạp.
5. Ghép tay cầm: PS4 giữ **Share + PS**, PS5 giữ **Create + PS**.

### Nút trên tay cầm
| Nút | Chức năng |
|---|---|
| Cần trái / cần phải | chạy / xoay |
| Giữ L2 | chạy chậm |
| R1 / L1 | nâng / hạ lift |
| D-pad ↑↓ | lên/xuống mức lift có sẵn |
| ✕ | lift về 0 |
| R3 | homing lift |
| △ | bật/tắt lái theo sân |
| □ | bật/tắt giữ hướng bằng IMU |
| Share/Create | đặt toạ độ về điểm xuất phát |
| **Giữ L1+R1, bấm Options** | **chạy tự động** |
| ◯ / PS / gạt mạnh cần | huỷ tự động |

## 4. Hiệu chỉnh (bắt buộc, theo thứ tự)
Mở Serial Monitor ở 115200 baud. Gõ `t` để tắt/bật in toạ độ, `z` để xoá bộ đếm hiệu chỉnh.
1. **Chiều motor**: đẩy cần tới thì cả 4 bánh phải quay tới. Bánh nào quay ngược thì đổi `MOTOR_INVERT`. Đẩy cần sang trái mà xe chạy sang phải thì đảo dây hoặc kiểm tra lại chiều lắp bánh mecanum (các con lăn phải tạo thành hình chữ X khi nhìn từ trên xuống).
2. **Chuột quang**: gõ `z`, đẩy tay xe tới đúng 1000 mm, gõ `f`. Đặt `FLOW_MM_PER_COUNT = 1000 / số đếm`. Đẩy tới phải ra x dương, đẩy sang trái phải ra y dương; nếu không, chỉnh `FLOW_SWAP_XY` / `FLOW_SIGN_*`.
   Sai 3% ở bước này sẽ làm xe lệch khoảng 7 cm sau 2–3 m. Đây là hệ số quan trọng nhất.
3. **Gyro**: gõ `z`, quay xe đúng 10 vòng, gõ `g`. Đặt `GYRO_SCALE = 3600 / số độ`. Quay ngược chiều kim đồng hồ mà θ giảm thì đặt `GYRO_SIGN = -1`.
4. **KS / KV**: KS là PWM nhỏ nhất làm xe nhúc nhích. Cho xe chạy thẳng với PWM cố định, đo vận tốc (dòng `v=` trong log), rồi tính `KV ≈ (PWM − KS) / v`.
5. **PID** (đầu file `Auto.ino`): tăng `AUTO_KP_XY` tới khi xe bám path tốt mà không rung. Tăng `AUTO_KI_XY` nếu xe dừng thiếu vài mm. `FF_LEAD_S` bù độ trễ của motor.

## 5. Viết hành động riêng
Cuối file `Auto.ino`, hàm `runCustomAction(id, param, start)`: thêm một `case` cho mỗi id bạn đặt trong app. Hàm trả về `true` khi xong. Không dùng `delay()` trong hàm này.
