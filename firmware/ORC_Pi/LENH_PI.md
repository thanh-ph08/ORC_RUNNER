# Lệnh dùng cho Raspberry Pi (user `rpi`, repo ở `~/ORC_RUNNER`)

## Cập nhật code từ GitHub
```
cd ~/ORC_RUNNER && git pull
sudo systemctl restart orc-pi        # nếu đang chạy bằng dịch vụ
```
Nếu `git pull` báo xung đột vì bạn đã sửa file: `git stash && git pull && git stash pop`

## Chạy tay (để xem log trực tiếp)
```
sudo systemctl stop orc-pi           # dừng dịch vụ trước, vì dịch vụ giữ cổng serial và chuột
cd ~/ORC_RUNNER/firmware/ORC_Pi
sudo python3 orc_pi.py               # Ctrl+C để dừng
sudo python3 orc_pi.py --imu-test    # đo thô IMU 60 giây (xe để YÊN), xem gyro/nhiệt độ/lỗi/reset
sudo systemctl start orc-pi          # chạy lại dịch vụ khi xong
```

## Tự chạy khi bật Pi (systemd)
```
cd ~/ORC_RUNNER/firmware/ORC_Pi
sudo cp orc-pi.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now orc-pi   # bật tự chạy và chạy ngay
systemctl status orc-pi              # xem trạng thái
journalctl -u orc-pi -f              # xem log trực tiếp (Ctrl+C để thoát)
journalctl -u orc-pi -b              # log từ lúc bật máy
sudo systemctl restart orc-pi        # khởi động lại dịch vụ
sudo systemctl disable --now orc-pi  # tắt tự chạy
```

## Trang web giám sát
Mở trên điện thoại / máy tính cùng Wi-Fi: `http://rpiorz.local:8080` hoặc `http://<IP của Pi>:8080`
```
hostname -I                          # xem IP của Pi
```

## Kiểm tra phần cứng
```
ls -l /dev/serial0                   # phải trỏ tới ttyAMA0
sudo i2cdetect -y 1                  # I2C bus 1 (GPIO2/3)
sudo i2cdetect -y 3                  # bus riêng GPIO5/6 (nếu đã bật dtoverlay i2c-gpio)
sudo evtest                          # chọn thiết bị chuột, di chuyển phải thấy REL_X / REL_Y
ls /dev/input/                       # danh sách thiết bị nhập
vcgencmd get_throttled               # 0x0 = nguồn ổn; khác 0 = Pi thiếu điện / quá nóng
vcgencmd measure_temp                # nhiệt độ CPU
```

## Cấu hình hệ thống (`/boot/firmware/config.txt`)
```
sudo nano /boot/firmware/config.txt
```
Các dòng cần có ở cuối file:
```
dtoverlay=disable-bt
enable_uart=1
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=5,i2c_gpio_scl=6,i2c_gpio_delay_us=2
```
Sau khi sửa: `sudo reboot`. Tắt máy an toàn: `sudo shutdown now`.
Nếu I2C hay lỗi, bỏ hoặc hạ dòng `dtparam=i2c_arm_baudrate=400000` (nếu có) xuống 100000.

## Cài lần đầu
```
sudo apt update && sudo apt install -y git python3-serial python3-evdev python3-pip i2c-tools evtest
sudo pip3 install smbus2 --break-system-packages
git clone https://github.com/thanh-ph08/ORC_RUNNER.git
```

## Sửa hằng số trong `orc_pi.py`
```
nano ~/ORC_RUNNER/firmware/ORC_Pi/orc_pi.py
```
Các hằng hay sửa: `IMU_TYPE` ("mpu6050" / "bmi160"), `MPU6050_ADDR`, `IMU_BUS`, `WEB_PORT`, `FLOW_MM_PER_COUNT`,
`FLOW_POS_X_MM` / `FLOW_POS_Y_MM`, `GYRO_SIGN`, `GYRO_SCALE`. Sửa xong chạy lại `sudo systemctl restart orc-pi`.

## Lệnh gõ trong Serial Monitor của ESP32#1 (115200)
`r` đặt lại vị trí xuất phát · `a` chạy tự động · `s` dừng · `t` bật/tắt telemetry · `f` số đếm chuột · `g` gyro cộng dồn ·
`z` xoá bộ đếm · `c` đo lại bias gyro · `e` encoder · `l` đặt 0 cho nâng hạ · `+` `-` đổi rpm bắn · `x` bật/tắt bắn · `i` quét I2C

## Liên kết I2C tới ESP32#1 (bus 3)
```
sudo i2cdetect -y 3                    # thấy 43 khi ESP32#1 đang chạy bản mới (PI_LINK_I2C 1)
sudo i2cdetect -y 1                    # bus của IMU và hub: 68 (IMU), 54 (hub servo), 42 (ESP32#2)
```
Trên Pi cần `dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=5,i2c_gpio_scl=6,i2c_gpio_delay_us=2` trong `/boot/firmware/config.txt`, rồi reboot. Dây: Pi GPIO5 (chân 29) với ESP32#1 GPIO18, Pi GPIO6 (chân 31) với ESP32#1 GPIO19, GND chung, trở kéo lên 4.7k về 3.3V trên SDA và SCL.
