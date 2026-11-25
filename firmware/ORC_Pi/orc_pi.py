#!/usr/bin/env python3
"""
orc_pi.py — Raspberry Pi 3B+ : đọc cảm biến, tính toạ độ, gửi cho ESP32#1 qua UART.

Việc của Pi:
  1. Đọc BMI160 (I2C bus 3 phần mềm, GPIO5/6) -> bộ lọc Mahony (gyro + gia tốc) -> góc YAW
  2. Đọc chuột USB (optical flow) bằng evdev -> dịch chuyển (count)
  3. Ghép: dùng yaw để (a) xoay dịch chuyển từ hệ xe sang hệ sân, (b) trừ phần chuyển động giả khi xe XOAY mà chuột
     không gắn đúng tâm quay (FLOW_POS_X/Y_MM) => toạ độ x, y, yaw (hệ sân)
  4. Đẩy PosePacket cho ESP32#1 qua UART khoảng 200 lần/giây, và nhận lệnh (đặt toạ độ, đo lại gyro) từ ESP32#1

====== Đấu dây (số theo GPIO/BCM) ======
  UART tới ESP32#1:  Pi TXD GPIO14 (chân 8)  -> ESP32 GPIO34
                     Pi RXD GPIO15 (chân 10) <- ESP32 GPIO23          GND chung. Logic 3.3V cả hai phía.
  BMI160 (I2C bus 3): SDA GPIO5 (chân 29), SCL GPIO6 (chân 31), 3V3, GND. Nằm PHẲNG, trục Z hướng LÊN.
                     SDO nối GND => 0x68, nối 3V3 => 0x69.
  Chuột:             cắm cổng USB của Pi (chuột optical, 800–1600 CPI, cách mặt sân 1–3 mm).

====== Cài đặt (xem README, mục "Cài Raspberry Pi") ======
  sudo apt install -y python3-serial python3-evdev i2c-tools python3-pip
  sudo pip3 install smbus2 --break-system-packages
  raspi-config: bật I2C, bật Serial Port phần cứng (TẮT login shell qua serial); config.txt: dtoverlay=disable-bt
  sudo python3 orc_pi.py        (cần root để đọc chuột)

Gõ Ctrl+C để thoát. Mỗi 0,5 s in toạ độ ra màn hình.
"""
import math
import os
import struct
import sys
import threading
import time

import serial
from smbus2 import SMBus
import evdev
from evdev import ecodes

# =====================================================================
# ============================ CẤU HÌNH ===============================
# =====================================================================
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUD = 921600
IMU_BUS = 3                         # bus I2C phần mềm riêng GPIO5/6 (dtoverlay=i2c-gpio,bus=3,...), tách khỏi ESP32
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
    def __init__(self, bus_no=IMU_BUS, addr=BMI160_ADDR):
        self.bus = SMBus(bus_no)
        self.addr = addr
        self.err = 0            # số lần đọc I2C lỗi (OSError)
        self.glitch = 0         # số mẫu bị loại vì giá trị vô lý (nhiễu bus I2C)
        self.last = None        # mẫu gyro hợp lệ gần nhất
        self.rej_run = 0        # số mẫu liên tiếp bị loại

    def begin(self) -> bool:
        try:
            cid = self.bus.read_byte_data(self.addr, 0x00)
        except OSError as e:
            print(f"[IMU] Không đọc được BMI160: {e}")
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

    def read(self):
        """-> (gx, gy, gz [dps], ax, ay, az [g]) hoặc None"""
        try:
            b = self.bus.read_i2c_block_data(self.addr, 0x0C, 12)
        except OSError:
            self.err += 1
            return None
        v = struct.unpack("<6h", bytes(b))
        s = (v[0] / 32.8, v[1] / 32.8, v[2] / 32.8, v[3] / 8192.0, v[4] / 8192.0, v[5] / 8192.0)
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

    @staticmethod
    def find():
        paths = [MOUSE_DEVICE] if MOUSE_DEVICE else evdev.list_devices()
        for p in paths:
            try:
                d = evdev.InputDevice(p)
                rel = d.capabilities().get(ecodes.EV_REL, [])
                if ecodes.REL_X in rel and ecodes.REL_Y in rel:
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
                time.sleep(1.0)
                continue
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
        self.imu = Bmi160()
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
        self.last_mouse = (0, 0)
        self.flow_last_yaw = 0.0
        self.last_motion = 0.0
        self.t_imu = self.t_flow = self.t_pkt = time.perf_counter()

    # ---- lệnh từ ESP32#1 ----
    def handle_rx(self, rx: bytes):
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
        if s is None:
            return
        gx, gy, gz, ax, ay, az = s
        mono = time.monotonic()
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


def main():
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0, write_timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"Không mở được {SERIAL_PORT}: {e}\nKiểm tra: raspi-config bật Serial Port phần cứng, tắt login shell, dtoverlay=disable-bt")
    parser = CmdParser()
    try:                                   # ưu tiên thời gian thực cho vòng lặp
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20))
    except (PermissionError, AttributeError, OSError):
        pass

    bot = Robot()
    print("ORC_Pi sẵn sàng, UART %s %d baud. Giữ yên xe ~%.1f s để đo bias gyro." % (SERIAL_PORT, SERIAL_BAUD, CAL_S))
    pkt = bot.packet()
    last_print = 0.0
    try:
        while True:
            now = time.perf_counter()
            if now - bot.t_imu >= IMU_PERIOD:
                bot.imu_step(now)
            if now - bot.t_flow >= FLOW_PERIOD:
                bot.flow_step(now)
            if now - bot.t_pkt >= PKT_PERIOD:
                bot.t_pkt = now
                pkt = bot.packet()
                try:
                    ser.write(POSE_SYNC + pkt)
                except serial.SerialTimeoutException:
                    pass                           # ESP32 chưa đọc kịp, bỏ gói này
            data = ser.read(256)
            if data:
                for payload in parser.feed(data):
                    bot.handle_rx(payload)
            t = time.monotonic()
            if t - last_print > 0.5:
                last_print = t
                print(f"x={bot.x:7.1f} y={bot.y:7.1f} yaw={math.degrees(bot.yaw):7.2f}° | chuột:{'OK' if bot.mouse.connected else 'chưa cắm'} "
                      f"raw=({bot.raw_x},{bot.raw_y}) | IMU:{'OK' if bot.imu_ok else 'LỖI'}{' (đang cân gyro)' if bot.calibrating else ''}"
                      f" nhiễu={bot.imu.glitch} lỗiI2C={bot.imu.err}")
            time.sleep(0.0005)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
