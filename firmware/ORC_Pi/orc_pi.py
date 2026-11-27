#!/usr/bin/env python3
"""
orc_pi.py — Raspberry Pi 3B+ : đọc cảm biến, tính toạ độ, gửi cho ESP32#1 qua UART.

Việc của Pi:
  1. Đọc MPU6050/BMI160 (I2C bus 3 phần mềm, GPIO5/6) -> bộ lọc Mahony (gyro + gia tốc) -> góc YAW
  2. Đọc chuột USB (optical flow) bằng evdev -> dịch chuyển (count)
  3. Ghép: dùng yaw để (a) xoay dịch chuyển từ hệ xe sang hệ sân, (b) trừ phần chuyển động giả khi xe XOAY mà chuột
     không gắn đúng tâm quay (FLOW_POS_X/Y_MM) => toạ độ x, y, yaw (hệ sân)
  4. Đẩy PosePacket cho ESP32#1 qua UART khoảng 200 lần/giây, và nhận lệnh (đặt toạ độ, đo lại gyro) từ ESP32#1

====== Đấu dây (số theo GPIO/BCM) ======
  UART tới ESP32#1:  Pi TXD GPIO14 (chân 8)  -> ESP32 GPIO34
                     Pi RXD GPIO15 (chân 10) <- ESP32 GPIO23          GND chung. Logic 3.3V cả hai phía.
  IMU (I2C bus 3): SDA GPIO5 (chân 29), SCL GPIO6 (chân 31), 3V3, GND. Nằm PHẲNG, trục Z hướng LÊN.
                     SDO nối GND => 0x68, nối 3V3 => 0x69.
  Chuột:             cắm cổng USB của Pi (chuột optical, 800–1600 CPI, cách mặt sân 1–3 mm).

====== Cài đặt (xem README, mục "Cài Raspberry Pi") ======
  sudo apt install -y python3-serial python3-evdev i2c-tools python3-pip
  sudo pip3 install smbus2 --break-system-packages
  raspi-config: bật I2C, bật Serial Port phần cứng (TẮT login shell qua serial); config.txt: dtoverlay=disable-bt
  sudo python3 orc_pi.py        (cần root để đọc chuột)

Gõ Ctrl+C để thoát. Mỗi 0,5 s in toạ độ ra màn hình.
"""
import collections
import math
import os
import queue
import struct
import sys
import threading
import time

import serial
from smbus2 import SMBus, i2c_msg
from web_ui import install_log_tee, start_web
import evdev
from evdev import ecodes

# =====================================================================
# ============================ CẤU HÌNH ===============================
# =====================================================================
LINK = "i2c"                        # liên kết Pi <-> ESP32#1: "i2c" = bus riêng (Pi master, ESP32#1 slave 0x43), "uart" = dây UART cũ
LINK_BUS = 3                        # bus I2C riêng cho liên kết: dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=5,i2c_gpio_scl=6 (GPIO5 = SDA chân 29, GPIO6 = SCL chân 31)
LINK_ADDR = 0x43                    # địa chỉ ESP32#1 (ORC_PILINK_ADDR)
LINK_PERIOD = 0.01                  # 100 lần/giây: ghi toạ độ + đọc trạng thái
SERIAL_PORT = "/dev/serial0"        # chỉ dùng khi LINK = "uart"
SERIAL_BAUD = 921600
IMU_BUS = 1                         # bus I2C phần cứng của Pi (GPIO2 = SDA chân 3, GPIO3 = SCL chân 5); đổi 3 nếu tách bus riêng bằng dtoverlay
WEB_PORT = 8080                    # trang giám sát http://<tên-máy>.local:8080 ; 0 = tắt
IMU_TYPE = "mpu6050"               # "mpu6050"/"bmi160" = Pi tự đọc I2C; "esp32" = ESP32#1 đọc rồi gửi qua UART (cần đặt IMU_READ_ON_ESP32 1 trong config.h)
MPU6050_ADDR = 0x68                # 0x69 nếu chân AD0 nối 3V3
BMI160_ADDR = 0x69                 # 0x68 nếu SDO nối GND

# ---- Chuột optical flow ----
MOUSE_DEVICE = None                # None = tự tìm chuột; hoặc "/dev/input/event3"
FLOW_MM_PER_COUNT = 0.60           # HIỆU CHỈNH: đẩy xe đúng 1000 mm, đọc rawX/rawY ('f' ở ESP32#1), lấy 1000 / số đếm
FLOW_SWAP_XY = False               # đẩy xe TỚI mà số đếm tăng ở trục y => True
FLOW_SIGN_X, FLOW_SIGN_Y = 1, 1    # đẩy TỚI phải ra x dương, đẩy sang TRÁI phải ra y dương
FLOW_POS_X_MM, FLOW_POS_Y_MM = 0.0, 0.0   # vị trí chuột so với TÂM QUAY (x trước, y trái) — đo bằng thước
FLOW_MAX_JUMP_MM = 40.0            # bỏ qua dịch chuyển nhảy bất thường trong 1 chu kỳ

# ---- BMI160 ----
GYRO_SIGN = 1                      # quay xe NGƯỢC chiều kim đồng hồ (nhìn từ trên) mà yaw GIẢM => -1
GYRO_SCALE = 1.000                 # HIỆU CHỈNH: quay xe đúng 10 vòng, đọc gyroDeg ('g'), lấy 3600 / số độ
MAHONY_KP = 1.5                    # độ tin gia tốc kế (chỉ sửa nghiêng); 0.5–5
ACCEL_G_MIN, ACCEL_G_MAX = 0.85, 1.15   # chỉ tin gia tốc khi |a| ≈ 1 g
CAL_S = 2.0                        # thời gian đo độ lệch gyro lúc bật / khi có lệnh (sau 0.4 s bỏ qua đầu)
CAL_SKIP_S = 0.4                   # bỏ qua đoạn đầu khi vừa reset cảm biến / vừa chạm tay vào xe
CAL_MOTION_DPS = 0.35              # 2 nửa thời gian đo lệch nhau quá mức này => xe bị động đậy, đo lại
GYRO_SPIKE_DPS = 300.0             # mẫu gyro nhảy quá mức này so với mẫu trước (2 ms) => coi là nhiễu bus, loại
BIAS_FAST_S = 8.0                  # trong khoảng này sau khi đo xong, học lại bias nhanh hơn
BIAS_TAU_FAST_S, BIAS_TAU_S = 1.5, 8.0

IMU_PERIOD = 0.002                 # 500 Hz (Python thực tế có thể thấp hơn, dt được đo thật)
FLOW_PERIOD = 0.005
PKT_PERIOD = 0.005

# ---- Giao thức (khớp orc_protocol.h) ----
POSE_SYNC = b"\xAA\x55"           # Pi -> ESP32 : sync + PosePacket(40 byte)
CMD_SYNC = (0xA5, 0x5A)           # ESP32 -> Pi : sync + len + payload + crc8(payload)
FLOW_CMD_SET_POSE = 0x10
FLOW_CMD_CAL_GYRO = 0x11
FLOW_CMD_CLEAR_RAW = 0x12
FLOW_CMD_LOG = 0x20                # ESP32#1 -> Pi: mảnh log (text)
FLOW_CMD_PING = 0x21               # ESP32#1 -> Pi: nhịp tim 1 Hz
FLOW_CMD_IMU = 0x22                # ESP32#1 -> Pi: mẫu MPU6050 thô
PILINK_W_POSE = 0xA7              # Pi ghi vào ESP32#1: 0xA7 + PosePacket(40)   (khớp orc_protocol.h)
PILINK_STATUS_MAGIC = 0xA8        # Pi đọc từ ESP32#1: PiStatus 32 byte
PILINK_F_SETPOSE = 0x01
POSE_FLAG_IMU_OK = 0x01
POSE_FLAG_MOUSE_OK = 0x02
POSE_FLAG_CALIBRATING = 0x04
POSE_FMT = "<BBB6f2ifB"            # seq, flags, setAck, x y yaw vx vy w, rawX rawY, gyroDeg, crc  = 40 byte
assert struct.calcsize(POSE_FMT) == 40


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# =====================================================================
# ============================== BMI160 ===============================
# =====================================================================
class Bmi160:
    NAME = "BMI160"
    GYRO_LSB = 32.8            # LSB/dps ở ±1000 dps
    ACC_LSB = 8192.0           # LSB/g ở ±4 g
    CHECKS = ((0x41, 0x05), (0x43, 0x01))     # (thanh ghi, giá trị đã ghi): mất cấu hình = chip đã reset
    CHECK_EVERY = 20           # kiểm tra cấu hình sau mỗi N mẫu tốt (~0.1 s)

    def __init__(self, bus_no=IMU_BUS, addr=BMI160_ADDR):
        if not os.path.exists(f"/dev/i2c-{bus_no}"):
            print(f"[IMU] CẢNH BÁO: chưa có /dev/i2c-{bus_no} (chưa thêm dtoverlay i2c-gpio trong config.txt), "
                  f"tạm dùng bus 1 — cảm biến phải nằm ở GPIO2/3. Nên tách bus riêng, xem README.")
            bus_no = 1
        self.bus = SMBus(bus_no)
        self.addr = addr
        self.err = 0            # số lần đọc I2C lỗi (OSError)
        self.glitch = 0         # số mẫu bị loại vì giá trị vô lý (nhiễu bus I2C)
        self.last = None        # mẫu gyro hợp lệ gần nhất
        self.rej_run = 0        # số mẫu liên tiếp bị loại
        self.fail_run = 0       # số lần đọc liên tiếp lỗi / toàn 0 (chip chết, bus treo)
        self.resets = 0         # số lần chip phải khởi tạo lại
        self.last_recover = 0.0
        self.n_ok = 0

    def begin(self) -> bool:
        try:
            cid = self.bus.read_byte_data(self.addr, 0x00)
        except OSError as e:
            print(f"[IMU] Không đọc được {self.NAME}: {e}")
            return False
        if cid != 0xD1:
            print(f"[IMU] Sai ID 0x{cid:02X}, cần 0xD1")
            return False
        w = lambda r, v: self.bus.write_byte_data(self.addr, r, v)
        w(0x7E, 0xB6); time.sleep(0.1)     # soft reset
        w(0x7E, 0x11); time.sleep(0.02)    # accel normal
        w(0x7E, 0x15); time.sleep(0.1)     # gyro normal
        w(0x40, 0x2B)                      # ACC_CONF 800 Hz
        w(0x41, 0x05)                      # ±4 g   (8192 LSB/g)
        w(0x42, 0x2B)                      # GYR_CONF 800 Hz
        w(0x43, 0x01)                      # ±1000 dps (32.8 LSB/dps)
        time.sleep(0.05)
        return True

    def config_ok(self) -> bool:
        """Chip còn giữ cấu hình đã ghi? Sai = chip đã bị reset (dữ liệu vẫn ra nhưng sai thang đo)."""
        try:
            for reg, want in self.CHECKS:
                if self.bus.read_byte_data(self.addr, reg) != want:
                    return False
        except OSError:
            return True        # lỗi bus thoáng qua, để lần sau
        return True

    def _fail(self):
        self.fail_run += 1
        if self.fail_run >= 25 and time.monotonic() - self.last_recover > 0.3:
            self.recover()

    def recover(self) -> bool:
        """Khởi tạo lại chip sau khi nó bị reset / bus treo."""
        self.last_recover = time.monotonic()
        self.fail_run = 0
        self.last = None
        try:
            ok = self.begin()
        except OSError:
            ok = False
        if ok:
            self.resets += 1
        return ok

    def read_temp(self):
        """Nhiệt độ chip [°C] (thanh ghi 0x20/0x21)"""
        try:
            lo, hi = self.bus.read_i2c_block_data(self.addr, 0x20, 2)
        except OSError:
            return float("nan")
        t = struct.unpack("<h", bytes([lo, hi]))[0]
        return 23.0 + t / 512.0

    def _raw(self):
        """-> (gx, gy, gz, ax, ay, az) số nguyên thô; ném OSError nếu bus lỗi"""
        b = self.bus.read_i2c_block_data(self.addr, 0x0C, 12)
        return struct.unpack("<6h", bytes(b))

    def read(self):
        """-> (gx, gy, gz [dps], ax, ay, az [g]) hoặc None"""
        try:
            v = self._raw()
        except OSError:
            self.err += 1
            self._fail()
            return None
        if not any(v) or all(x == -1 for x in v):   # toàn 0 / toàn 0xFF = chip ngủ (vừa reset) hoặc bus chết
            self.glitch += 1
            self._fail()
            return None
        self.fail_run = 0
        self.n_ok += 1
        if self.n_ok % self.CHECK_EVERY == 0 and not self.config_ok():
            self.glitch += 1
            if time.monotonic() - self.last_recover > 0.3:
                self.recover()
            return None
        g, a = self.GYRO_LSB, self.ACC_LSB
        s = (v[0] / g, v[1] / g, v[2] / g, v[3] / a, v[4] / a, v[5] / a)
        # Loại mẫu hỏng do nhiễu/va chạm trên bus I2C: tràn thang đo, toàn 0, hoặc nhảy vọt không thể có
        bad = any(x in (-32768, 32767) for x in v) or not any(v) \
            or abs(s[3]) > 4.5 or abs(s[4]) > 4.5 or abs(s[5]) > 4.5
        if not bad and self.last is not None:
            bad = max(abs(s[i] - self.last[i]) for i in range(3)) > GYRO_SPIKE_DPS
        if bad and self.rej_run < 3:           # 3 mẫu hỏng liên tiếp => coi là chuyển động thật, chấp nhận lại
            self.rej_run += 1
            self.glitch += 1
            return None
        self.rej_run = 0
        self.last = s
        return s


class Mpu6050(Bmi160):
    """MPU6050 (GY-521): cùng giao diện với Bmi160, dữ liệu big-endian từ 0x3B."""
    NAME = "MPU6050"
    GYRO_LSB = 32.8            # ±1000 dps
    ACC_LSB = 8192.0           # ±4 g
    CHECKS = ((0x6B, 0x01), (0x1B, 0x10), (0x1C, 0x08))
    WHO_OK = (0x68, 0x69, 0x70, 0x71, 0x72, 0x73, 0x98)   # MPU6050 và vài bản clone/MPU6500

    def __init__(self, bus_no=IMU_BUS, addr=MPU6050_ADDR):
        super().__init__(bus_no, addr)

    def begin(self) -> bool:
        try:
            who = self.bus.read_byte_data(self.addr, 0x75)
        except OSError as e:
            print(f"[IMU] Không đọc được {self.NAME}: {e}")
            return False
        if who not in self.WHO_OK:
            print(f"[IMU] WHO_AM_I = 0x{who:02X}, không phải MPU6050 (cần 0x68)")
            return False
        w = lambda r, v: self.bus.write_byte_data(self.addr, r, v)
        w(0x6B, 0x80); time.sleep(0.1)     # reset thiết bị
        w(0x6B, 0x01); time.sleep(0.05)    # thoát chế độ ngủ, xung nhịp PLL theo gyro X
        w(0x68, 0x07); time.sleep(0.1)     # reset đường tín hiệu gyro/accel/temp
        w(0x19, 0x00)                      # SMPLRT_DIV = 0 (1 kHz)
        w(0x1A, 0x03)                      # DLPF ~44 Hz: lọc rung/nhiễu
        w(0x1B, 0x10)                      # gyro ±1000 dps (32.8 LSB/dps)
        w(0x1C, 0x08)                      # accel ±4 g (8192 LSB/g)
        time.sleep(0.1)
        return True

    def _raw(self):
        b = self.bus.read_i2c_block_data(self.addr, 0x3B, 14)
        ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", bytes(b))
        return (gx, gy, gz, ax, ay, az)

    def read_temp(self):
        try:
            hi, lo = self.bus.read_i2c_block_data(self.addr, 0x41, 2)
        except OSError:
            return float("nan")
        return struct.unpack(">h", bytes([hi, lo]))[0] / 340.0 + 36.53


class EspImu:
    """MPU6050 do ESP32#1 đọc (cùng bus I2C với hub), gửi mẫu thô qua UART. Pi không đụng I2C => không tranh bus."""
    NAME = "MPU6050 (qua ESP32#1)"
    GYRO_LSB = 32.8
    ACC_LSB = 8192.0

    def __init__(self):
        self.q = collections.deque(maxlen=64)
        self.err = 0            # lỗi I2C đếm ở phía ESP32#1
        self.resets = 0         # số lần ESP32#1 phải khởi tạo lại chip
        self.glitch = 0         # mẫu bị loại vì giá trị vô lý
        self.last = None
        self.rej_run = 0
        self.last_ts = None
        self.last_dt = 0.01
        self.temp = float("nan")
        self.rx_t = 0.0         # lần cuối nhận mẫu
        self.n_rx = 0

    def begin(self) -> bool:
        return True

    def push(self, p: bytes):
        """p = payload sau byte lệnh: u32 µs | 6×int16 | int16 temp | u8 reset | u16 lỗi  (20 byte)"""
        if len(p) < 21:
            return
        ts, = struct.unpack("<I", p[0:4])
        v = struct.unpack("<6h", p[4:16])
        t, = struct.unpack("<h", p[16:18])
        self.resets = p[18]
        self.err, = struct.unpack("<H", p[19:21])
        self.temp = t / 340.0 + 36.53
        self.rx_t = time.monotonic()
        self.n_rx += 1
        self.q.append((ts, v))

    def age(self):
        return None if self.n_rx == 0 else time.monotonic() - self.rx_t

    def read_temp(self):
        return self.temp

    def read(self):
        """-> (gx, gy, gz [dps], ax, ay, az [g]) hoặc None; self.last_dt = giây giữa 2 mẫu (theo đồng hồ của ESP32#1)"""
        if not self.q:
            return None
        ts, v = self.q.popleft()
        if self.last_ts is not None:
            dt = ((ts - self.last_ts) & 0xFFFFFFFF) * 1e-6
            self.last_dt = dt if 0.0005 < dt < 0.05 else 0.01
        self.last_ts = ts
        g, a = self.GYRO_LSB, self.ACC_LSB
        s = (v[0] / g, v[1] / g, v[2] / g, v[3] / a, v[4] / a, v[5] / a)
        bad = any(x in (-32768, 32767) for x in v) or abs(s[3]) > 4.5 or abs(s[4]) > 4.5 or abs(s[5]) > 4.5
        if not bad and self.last is not None:
            bad = max(abs(s[i] - self.last[i]) for i in range(3)) > GYRO_SPIKE_DPS
        if bad and self.rej_run < 3:
            self.rej_run += 1
            self.glitch += 1
            return None
        self.rej_run = 0
        self.last = s
        return s


def make_imu():
    t = IMU_TYPE.lower()
    if t == "esp32":
        return EspImu()
    return Mpu6050() if t == "mpu6050" else Bmi160()


# =====================================================================
# ======================== BỘ LỌC MAHONY 6 TRỤC =======================
# =====================================================================
class Mahony:
    def __init__(self):
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0

    def update(self, gx, gy, gz, ax, ay, az, dt, kp, use_accel):
        """g*: rad/s ; a*: g"""
        if use_accel:
            n = math.sqrt(ax * ax + ay * ay + az * az)
            if n > 1e-6:
                ax, ay, az = ax / n, ay / n, az / n
                q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
                hvx = q1 * q3 - q0 * q2
                hvy = q0 * q1 + q2 * q3
                hvz = q0 * q0 - 0.5 + q3 * q3
                ex = ay * hvz - az * hvy
                ey = az * hvx - ax * hvz
                ez = ax * hvy - ay * hvx
                gx += 2.0 * kp * ex
                gy += 2.0 * kp * ey
                gz += 2.0 * kp * ez
        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt
        qa, qb, qc = self.q0, self.q1, self.q2
        self.q0 += -qb * gx - qc * gy - self.q3 * gz
        self.q1 += qa * gx + qc * gz - self.q3 * gy
        self.q2 += qa * gy - qb * gz + self.q3 * gx
        self.q3 += qa * gz + qb * gy - qc * gx
        r = 1.0 / math.sqrt(self.q0 ** 2 + self.q1 ** 2 + self.q2 ** 2 + self.q3 ** 2)
        self.q0 *= r; self.q1 *= r; self.q2 *= r; self.q3 *= r

    def yaw(self):
        return math.atan2(2.0 * (self.q0 * self.q3 + self.q1 * self.q2), 1.0 - 2.0 * (self.q2 ** 2 + self.q3 ** 2))

    def set_yaw(self, new_yaw):
        d = new_yaw - self.yaw()
        c, s = math.cos(0.5 * d), math.sin(0.5 * d)
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        self.q0 = c * q0 - s * q3
        self.q1 = c * q1 - s * q2
        self.q2 = c * q2 + s * q1
        self.q3 = c * q3 + s * q0


# =====================================================================
# ============================== CHUỘT ================================
# =====================================================================
class MouseReader(threading.Thread):
    """Chạy nền, cộng dồn REL_X / REL_Y. Tự tìm lại chuột nếu rút ra cắm vào."""

    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.acc_x = 0
        self.acc_y = 0
        self.connected = False
        self.warned = False

    BAD_NAMES = ("hdmi", "vc4", "cec", "keyboard", "kbd", "touchpad")   # thiết bị ảo của Pi / bàn phím có REL_X giả

    @staticmethod
    def find():
        paths = [MOUSE_DEVICE] if MOUSE_DEVICE else evdev.list_devices()
        for p in paths:
            try:
                d = evdev.InputDevice(p)
                caps = d.capabilities()
                rel = caps.get(ecodes.EV_REL, [])
                keys = caps.get(ecodes.EV_KEY, [])
                name = (d.name or "").lower()
                ok = ecodes.REL_X in rel and ecodes.REL_Y in rel
                if not MOUSE_DEVICE:               # tự dò: bỏ thiết bị HDMI-CEC (vc4-hdmi) và thứ không phải chuột thật
                    ok = ok and ecodes.BTN_LEFT in keys and not any(b in name for b in MouseReader.BAD_NAMES)
                if ok:
                    return d
                d.close()
            except OSError:
                continue
        return None

    def read_acc(self):
        with self.lock:
            return self.acc_x, self.acc_y

    def run(self):
        while True:
            dev = self.find()
            if dev is None:
                self.connected = False
                if not self.warned:
                    self.warned = True
                    names = []
                    for p in evdev.list_devices():
                        try:
                            names.append(f"{p}={evdev.InputDevice(p).name}")
                        except OSError:
                            pass
                    print("[MOUSE] Không thấy chuột USB (loại vc4-hdmi). Cắm chuột optical vào cổng USB của Pi. Thiết bị hiện có: "
                          + (", ".join(names) or "không có"))
                time.sleep(1.0)
                continue
            self.warned = False
            print(f"[MOUSE] Dùng {dev.path} ({dev.name})")
            try:
                dev.grab()                      # không cho con trỏ hệ điều hành nhúc nhích theo
            except OSError:
                pass
            self.connected = True
            try:
                for ev in dev.read_loop():
                    if ev.type == ecodes.EV_REL:
                        if ev.code == ecodes.REL_X:
                            with self.lock:
                                self.acc_x += ev.value
                        elif ev.code == ecodes.REL_Y:
                            with self.lock:
                                self.acc_y += ev.value
            except OSError:
                pass
            self.connected = False
            print("[MOUSE] Mất chuột, tìm lại...")


# =====================================================================
# ============================== CHÍNH ================================
# =====================================================================
class Robot:
    def __init__(self):
        self.imu = make_imu()
        self.imu_ok = self.imu.begin()
        self.ahrs = Mahony()
        self.mouse = MouseReader()
        self.mouse.start()

        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = 0.0
        self.w = 0.0
        self.bias = [0.0, 0.0, 0.0]
        self.raw_x = self.raw_y = 0
        self.gyro_deg = 0.0
        self.set_ack = 0
        self.seq = 0

        self.calibrating = True
        self.cal_start = time.monotonic()
        self.cal_sum = [0.0, 0.0, 0.0]
        self.cal_n = 0
        self.cal_a = [0.0, 0.0, 0.0]      # tổng nửa đầu thời gian đo
        self.cal_na = 0
        self.cal_b = [0.0, 0.0, 0.0]      # tổng nửa sau
        self.cal_nb = 0
        self.cal_done_t = 0.0
        self.seen_resets = 0
        self.cal_warn_t = 0.0
        self.esp_last = 0.0            # lần cuối nhận bất cứ gì từ ESP32#1
        self.esp_pings = 0
        self.esp_buf = bytearray()
        self.imu_n = 0                 # đếm mẫu để tính tần số đọc IMU (hiển thị trên trang web)
        self.imu_hz = 0.0
        self.imu_t0 = time.monotonic()
        self.imu_temp = float("nan")
        self.imu_temp_t = 0.0
        self.last_mouse = (0, 0)
        self.flow_last_yaw = 0.0
        self.last_motion = 0.0
        self.t_imu = self.t_flow = self.t_pkt = time.perf_counter()

    # ---- lệnh từ ESP32#1 ----
    def handle_rx(self, rx: bytes):
        self.esp_last = time.monotonic()
        if rx and rx[0] == FLOW_CMD_PING:
            self.esp_pings += 1
            return
        if rx and rx[0] == FLOW_CMD_IMU:
            if isinstance(self.imu, EspImu):
                self.imu.push(rx[1:])
            return
        if rx and rx[0] == FLOW_CMD_LOG:               # ghép các mảnh thành dòng, in ra (trang web đọc từ đây)
            self.esp_buf += rx[1:]
            while True:
                k = self.esp_buf.find(b"\n")
                if k < 0:
                    if len(self.esp_buf) > 300:        # dòng dài bất thường / mất dấu xuống dòng: đẩy ra luôn
                        k = len(self.esp_buf)
                    else:
                        break
                line = bytes(self.esp_buf[:k]).decode("utf-8", "replace").strip()
                del self.esp_buf[:k + 1]
                if line:
                    print("[ESP32#1] " + line)
            return
        i = 0
        while i < len(rx):
            cmd = rx[i]
            if cmd == FLOW_CMD_SET_POSE and i + 13 <= len(rx):
                x, y, yaw = struct.unpack("<3f", rx[i + 1:i + 13])
                self.x, self.y = x, y
                self.ahrs.set_yaw(yaw)
                self.yaw = self.ahrs.yaw()
                self.flow_last_yaw = self.yaw
                self.last_mouse = self.mouse.read_acc()
                self.vx = self.vy = 0.0
                self.set_ack = (self.set_ack + 1) & 0xFF
                i += 13
            elif cmd == FLOW_CMD_CAL_GYRO:
                self.start_calibration()
                i += 1
            elif cmd == FLOW_CMD_CLEAR_RAW:
                self.raw_x = self.raw_y = 0
                self.gyro_deg = 0.0
                i += 1
            else:
                i += 1

    def start_calibration(self):
        self.calibrating = True
        self.cal_start = time.monotonic()
        self.cal_sum = [0.0, 0.0, 0.0]
        self.cal_n = 0
        self.cal_a = [0.0, 0.0, 0.0]
        self.cal_na = 0
        self.cal_b = [0.0, 0.0, 0.0]
        self.cal_nb = 0
        print("[IMU] Đo bias gyro, GIỮ YÊN XE")

    # ---- IMU ----
    def imu_step(self, now):
        dt = min(now - self.t_imu, 0.05)
        self.t_imu = now
        if not self.imu_ok:
            return
        s = self.imu.read()
        if isinstance(self.imu, EspImu):
            dt = self.imu.last_dt                      # dt đo bằng đồng hồ ESP32#1, không bị Python/UART làm lệch
        if self.imu.resets != self.seen_resets:
            self.seen_resets = self.imu.resets
            still = time.monotonic() - self.last_motion > 1.0 and not self.calibrating
            print("[IMU] Chip bị reset giữa chừng (nghi sụt nguồn/nhiễu) -> " +
                  ("đo lại bias, GIỮ YÊN XE" if still else "giữ bias cũ vì xe đang động, yaw có thể lệch"))
            if still:
                self.start_calibration()
        if s is None:
            return
        gx, gy, gz, ax, ay, az = s
        mono = time.monotonic()
        self.imu_n += 1
        if mono - self.imu_t0 >= 1.0:
            self.imu_hz = self.imu_n / (mono - self.imu_t0)
            self.imu_n, self.imu_t0 = 0, mono
        if mono - self.imu_temp_t >= 2.0:      # đọc nhiệt độ ở luồng chính (không đọc từ luồng web, tránh 2 luồng chung bus I2C)
            self.imu_temp_t = mono
            self.imu_temp = self.imu.read_temp()
        if self.calibrating:
            self.ahrs.update(0, 0, 0, ax, ay, az, dt, 8.0, True)       # gom tư thế nghiêng nhanh khi đứng yên
            el = mono - self.cal_start - CAL_SKIP_S
            if el >= 0:
                tgt, n = (self.cal_a, 'a') if el < CAL_S / 2 else (self.cal_b, 'b')
                tgt[0] += gx; tgt[1] += gy; tgt[2] += gz
                if n == 'a':
                    self.cal_na += 1
                else:
                    self.cal_nb += 1
            if el >= CAL_S and (self.cal_na <= 20 or self.cal_nb <= 20):
                # hết giờ mà chưa đủ mẫu (cảm biến lỗi/đứt giữa chừng) -> đo lại từ đầu, tránh kẹt "đang cân" mãi
                if mono - self.cal_warn_t > 5.0:
                    self.cal_warn_t = mono
                    print("[IMU] Chưa đủ mẫu để đo bias (cảm biến/ESP32#1 chưa gửi dữ liệu?), đo lại — giữ yên xe")
                self.start_calibration()
                return
            if el >= CAL_S and self.cal_na > 20 and self.cal_nb > 20:
                ma = [v / self.cal_na for v in self.cal_a]
                mb = [v / self.cal_nb for v in self.cal_b]
                if max(abs(ma[i] - mb[i]) for i in range(3)) > CAL_MOTION_DPS:
                    print("[IMU] Xe bị động đậy lúc đo bias, đo lại — giữ yên xe")
                    self.start_calibration()
                    return
                n = self.cal_na + self.cal_nb
                self.bias = [(self.cal_a[i] + self.cal_b[i]) / n for i in range(3)]
                self.cal_n = n
                self.calibrating = False
                self.cal_done_t = mono
                self.last_mouse = self.mouse.read_acc()
                self.flow_last_yaw = self.ahrs.yaw()
                print(f"[IMU] bias = {self.bias[0]:.3f} {self.bias[1]:.3f} {self.bias[2]:.3f} dps ({n} mẫu)")
            return
        rx = (gx - self.bias[0]) * GYRO_SIGN * GYRO_SCALE
        ry = (gy - self.bias[1]) * GYRO_SIGN * GYRO_SCALE
        rz = (gz - self.bias[2]) * GYRO_SIGN * GYRO_SCALE
        # xe đứng yên => học lại bias (nhanh trong vài giây đầu, sau đó chậm) để giảm trôi
        fast = mono - self.cal_done_t < BIAS_FAST_S
        lim = 2.5 if fast else 1.5
        if abs(rx) < lim and abs(ry) < lim and abs(rz) < lim and mono - self.last_motion > 0.3:
            a = dt / (BIAS_TAU_FAST_S if fast else BIAS_TAU_S)
            for i, g in enumerate((gx, gy, gz)):
                self.bias[i] += a * (g - self.bias[i])
        an = math.sqrt(ax * ax + ay * ay + az * az)
        use_acc = ACCEL_G_MIN < an < ACCEL_G_MAX
        d2r = math.pi / 180.0
        self.ahrs.update(rx * d2r, ry * d2r, rz * d2r, ax, ay, az, dt, MAHONY_KP, use_acc)
        self.yaw = self.ahrs.yaw()
        self.w = rz * d2r
        self.gyro_deg += rz * dt

    # ---- chuột + toạ độ ----
    def flow_step(self, now):
        dts = now - self.t_flow
        self.t_flow = now
        if self.calibrating:
            # đang cân gyro: chưa tính toạ độ, nhưng vẫn đếm chuột thô để kiểm tra chuột độc lập với IMU
            cx, cy = self.mouse.read_acc()
            sx, sy = (cy - self.last_mouse[1], cx - self.last_mouse[0]) if FLOW_SWAP_XY else (cx - self.last_mouse[0], cy - self.last_mouse[1])
            self.raw_x += int(sx * FLOW_SIGN_X)
            self.raw_y += int(sy * FLOW_SIGN_Y)
            self.last_mouse = (cx, cy)
            return
        cx, cy = self.mouse.read_acc()
        dx, dy = cx - self.last_mouse[0], cy - self.last_mouse[1]
        self.last_mouse = (cx, cy)
        dth = wrap_pi(self.yaw - self.flow_last_yaw)
        self.flow_last_yaw = self.yaw
        if dx or dy or abs(dth) > 1e-4:
            self.last_motion = time.monotonic()
        sx, sy = (dy, dx) if FLOW_SWAP_XY else (dx, dy)
        sx *= FLOW_SIGN_X
        sy *= FLOW_SIGN_Y
        self.raw_x += int(sx)
        self.raw_y += int(sy)
        mx, my = sx * FLOW_MM_PER_COUNT, sy * FLOW_MM_PER_COUNT
        if math.hypot(mx, my) > FLOW_MAX_JUMP_MM:
            return
        # chuột cách tâm quay (px, py): xe xoay dth thì chuột "trượt" (-dth*py, +dth*px) dù tâm xe đứng yên => trừ đi
        tx = mx + dth * FLOW_POS_Y_MM
        ty = my - dth * FLOW_POS_X_MM
        thm = self.yaw - 0.5 * dth
        c, s = math.cos(thm), math.sin(thm)
        fx, fy = c * tx - s * ty, s * tx + c * ty
        self.x += fx
        self.y += fy
        if dts > 1e-4:
            self.vx = 0.7 * self.vx + 0.3 * fx / dts
            self.vy = 0.7 * self.vy + 0.3 * fy / dts

    def packet(self) -> bytes:
        flags = (POSE_FLAG_IMU_OK if self.imu_ok else 0) | (POSE_FLAG_MOUSE_OK if self.mouse.connected else 0) \
            | (POSE_FLAG_CALIBRATING if self.calibrating else 0)
        self.seq = (self.seq + 1) & 0xFF
        body = struct.pack(POSE_FMT[:-1], self.seq, flags, self.set_ack, self.x, self.y, self.yaw,
                           self.vx, self.vy, self.w, self.raw_x, self.raw_y, self.gyro_deg)
        return body + bytes([crc8(body)])


class PiLink(threading.Thread):
    """Liên kết I2C riêng tới ESP32#1 (Pi master, ESP32#1 slave 0x43). Chạy nền ~100 Hz:
       ghi PosePacket, đọc PiStatus (lệnh đặt toạ độ / đo gyro / xoá bộ đếm + mảnh log) rồi đẩy vào hàng đợi cho vòng chính xử lý."""

    def __init__(self, bot, q):
        super().__init__(daemon=True)
        self.bot, self.q = bot, q
        self.bus = None
        self.ok = 0
        self.err = 0
        self.bad = 0
        self.last_boot = None
        self.last_set = None
        self.last_cal = None
        self.last_clear = None
        self.last_logseq = None
        self.fresh = 0
        self.dup = 0
        self.stat_t = time.monotonic()
        self.warn_t = 0.0

    def _warn(self, msg):
        t = time.monotonic()
        if t - self.warn_t > 5.0:
            self.warn_t = t
            print(msg)

    def _open(self):
        if not os.path.exists(f"/dev/i2c-{LINK_BUS}"):
            self._warn(f"[LINK] CHƯA CÓ /dev/i2c-{LINK_BUS}: thêm vào /boot/firmware/config.txt dòng "
                       f"dtoverlay=i2c-gpio,bus={LINK_BUS},i2c_gpio_sda=5,i2c_gpio_scl=6,i2c_gpio_delay_us=2 rồi reboot")
            return False
        self.bus = SMBus(LINK_BUS)
        return True

    def handle(self, st: bytes):
        if len(st) != 32 or st[0] != PILINK_STATUS_MAGIC or crc8(st[:31]) != st[31]:
            self.bad += 1
            self._warn(f"[LINK] ESP32#1 trả dữ liệu sai (magic/CRC), {self.bad} lần — kiểm tra dây GPIO5/6 <-> GPIO18/19, trở kéo lên, GND")
            return
        self.ok += 1
        bot = self.bot
        bot.esp_last = time.monotonic()
        bot.esp_pings += 1
        flags, c_set, c_cal, c_clear, boot = st[1], st[2], st[3], st[4], st[5]
        if boot != self.last_boot:                       # ESP32#1 vừa khởi động lại (hoặc lần đọc đầu): đồng bộ lại bộ đếm
            if self.last_boot is not None:
                print("[LINK] ESP32#1 vừa khởi động lại")
            self.last_boot = boot
            self.last_set = None
            self.last_cal, self.last_clear = c_cal, c_clear
        if (flags & PILINK_F_SETPOSE) and c_set != self.last_set:
            self.last_set = c_set
            self.q.put(bytes([FLOW_CMD_SET_POSE]) + st[6:18])
        if c_cal != self.last_cal:
            self.last_cal = c_cal
            self.q.put(bytes([FLOW_CMD_CAL_GYRO]))
        if c_clear != self.last_clear:
            self.last_clear = c_clear
            self.q.put(bytes([FLOW_CMD_CLEAR_RAW]))
        n, seq = st[18], st[19]
        if 0 < n <= 11:
            if seq != self.last_logseq:                  # mảnh log mới
                self.last_logseq = seq
                self.q.put(bytes([FLOW_CMD_LOG]) + st[20:20 + n])
                self.fresh += 1
            else:
                self.dup += 1                            # đọc nhanh hơn ESP32 dựng gói: bỏ mảnh trùng

    def run(self):
        while True:
            t0 = time.perf_counter()
            try:
                if self.bus is None and not self._open():
                    time.sleep(1.0)
                    continue
                self.bus.i2c_rdwr(i2c_msg.write(LINK_ADDR, bytes([PILINK_W_POSE]) + self.bot.packet()))
                rd = i2c_msg.read(LINK_ADDR, 32)
                self.bus.i2c_rdwr(rd)
                self.handle(bytes(rd))
                if time.monotonic() - self.stat_t > 10.0:
                    self.stat_t = time.monotonic()
                    print(f"[LINK] 10 s qua: đọc đúng {self.ok} lần, sai {self.bad}, lỗi bus {self.err}; mảnh log mới {self.fresh}, trùng {self.dup}")
            except OSError as e:
                self.err += 1
                self._warn(f"[LINK] Không nói chuyện được với ESP32#1 (0x{LINK_ADDR:02X} trên bus {LINK_BUS}): {e} "
                           f"— ESP32#1 phải đang chạy bản mới (PI_LINK_I2C 1), dây GPIO18/19, trở kéo lên 4.7k, GND chung. Tổng lỗi {self.err}")
                time.sleep(0.05)
            dt = time.perf_counter() - t0
            if dt < LINK_PERIOD:
                time.sleep(LINK_PERIOD - dt)


class CmdParser:
    """Tách khung lệnh từ ESP32#1: A5 5A len payload crc8(payload)"""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(bytes(CMD_SYNC))
            if i < 0:
                del self.buf[:max(0, len(self.buf) - 1)]      # giữ lại 1 byte phòng khi sync bị cắt đôi
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 3:
                break
            n = self.buf[2]
            if n == 0 or n > 32:                               # độ dài vô lý => sync giả
                del self.buf[:2]
                continue
            if len(self.buf) < 3 + n + 1:
                break
            payload = bytes(self.buf[3:3 + n])
            if crc8(payload) == self.buf[3 + n]:
                out.append(payload)
                del self.buf[:3 + n + 1]
            else:
                del self.buf[:2]
        return out


def imu_test(seconds=60):
    """python3 orc_pi.py --imu-test : đo thô IMU để biết gyro tự trôi hay do code. Để xe YÊN."""
    imu = make_imu()
    if isinstance(imu, EspImu):
        print("IMU_TYPE = esp32: IMU do ESP32#1 đọc nên không đo trực tiếp từ Pi được. Chạy bình thường rồi xem trang web / log;\n"
              "muốn thử I2C ngay trên Pi thì tạm đặt IMU_TYPE = \"mpu6050\" (và Pi phải nối trực tiếp vào IMU).")
        return
    if not imu.begin():
        return
    print("Giây | gyro thô gx gy gz (dps, trung bình 1 s) | |a| (g) | nhiệt độ | mẫu | nhiễu | lỗi")
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        t0 = time.monotonic()
        sg, sa, n = [0.0, 0.0, 0.0], 0.0, 0
        while time.monotonic() - t0 < 1.0:
            r = imu.read()
            if r:
                sg[0] += r[0]; sg[1] += r[1]; sg[2] += r[2]
                sa += math.sqrt(r[3] ** 2 + r[4] ** 2 + r[5] ** 2)
                n += 1
            time.sleep(0.002)
        if n:
            print(f"{int(time.monotonic() - t_end + seconds):3d} | {sg[0]/n:7.3f} {sg[1]/n:7.3f} {sg[2]/n:7.3f} | {sa/n:.3f} | "
                  f"{imu.read_temp():.1f} °C | {n} | {imu.glitch} | {imu.err} | reset={imu.resets}")


def main():
    if "--imu-test" in sys.argv:
        imu_test()
        return
    ser = None
    if LINK != "i2c":
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0, write_timeout=0.05)
        except serial.SerialException as e:
            sys.exit(f"Không mở được {SERIAL_PORT}: {e}\nKiểm tra: raspi-config bật Serial Port phần cứng, tắt login shell, dtoverlay=disable-bt")
    parser = CmdParser()
    try:                                   # ưu tiên thời gian thực cho vòng lặp
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20))
    except (PermissionError, AttributeError, OSError):
        pass

    install_log_tee()
    bot = Robot()
    if WEB_PORT:
        start_web(bot, WEB_PORT)
    link_q = queue.Queue()
    link = None
    if LINK == "i2c":
        link = PiLink(bot, link_q)
        link.start()
        print("ORC_Pi sẵn sàng, liên kết I2C tới ESP32#1 (bus %d, 0x%02X). Giữ yên xe ~%.1f s để đo bias gyro." % (LINK_BUS, LINK_ADDR, CAL_S))
    else:
        print("ORC_Pi sẵn sàng, UART %s %d baud. Giữ yên xe ~%.1f s để đo bias gyro." % (SERIAL_PORT, SERIAL_BAUD, CAL_S))
    pkt = bot.packet()
    last_print = 0.0
    try:
        while True:
            now = time.perf_counter()
            if now - bot.t_imu >= (0.0 if isinstance(bot.imu, EspImu) else IMU_PERIOD):
                bot.imu_step(now)
            if now - bot.t_flow >= FLOW_PERIOD:
                bot.flow_step(now)
            if link is not None:
                while True:                        # lệnh / log từ ESP32#1 do luồng I2C gom về, xử lý ở luồng chính
                    try:
                        bot.handle_rx(link_q.get_nowait())
                    except queue.Empty:
                        break
            else:
                if now - bot.t_pkt >= PKT_PERIOD:
                    bot.t_pkt = now
                    pkt = bot.packet()
                    try:
                        ser.write(POSE_SYNC + pkt)
                    except serial.SerialTimeoutException:
                        pass                       # ESP32 chưa đọc kịp, bỏ gói này
                data = ser.read(256)
                if data:
                    for payload in parser.feed(data):
                        bot.handle_rx(payload)
            t = time.monotonic()
            if t - last_print > 0.5:
                last_print = t
                print(f"x={bot.x:7.1f} y={bot.y:7.1f} yaw={math.degrees(bot.yaw):7.2f}° | chuột:{'OK' if bot.mouse.connected else 'chưa cắm'} "
                      f"raw=({bot.raw_x},{bot.raw_y}) | IMU:{'OK' if bot.imu_ok else 'LỖI'}{' (đang cân gyro)' if bot.calibrating else ''}"
                      f" nhiễu={bot.imu.glitch} lỗiI2C={bot.imu.err} reset={bot.imu.resets}")
            time.sleep(0.0005)
    except KeyboardInterrupt:
        pass
    finally:
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    main()
