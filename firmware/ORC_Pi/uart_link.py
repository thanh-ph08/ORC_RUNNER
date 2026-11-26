"""
uart_link.py — thử UART hai chiều với ESP32#1 (nạp ORC_UartTest.ino vào ESP32#1). Dừng dịch vụ trước:  sudo systemctl stop orc-pi
    sudo python3 uart_link.py [baud]
Pi gửi 20 khung/giây sang ESP32 và nghe ESP32; mỗi giây in:
  ESP32 -> Pi : số khung đúng / sai đã nhận, seq cuối
  Pi -> ESP32 : số khung mà ESP32 BÁO là đã nhận đúng từ Pi (đọc từ chính khung ESP32 gửi về)
"""
import sys, time, threading, struct
import serial

PORT = "/dev/serial0"
BAUD = int(sys.argv[1]) if len(sys.argv) > 1 else 921600


def crc8(data: bytes) -> int:
    c = 0
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def make_frame(seq: int) -> bytes:
    p = bytes([0x31, seq & 0xFF])
    return bytes([0xAA, 0x55, len(p)]) + p + bytes([crc8(p)])


class Parser:
    """A5 5A len payload crc8"""
    def __init__(self):
        self.buf = bytearray()
        self.ok = self.bad = self.bytes = 0

    def feed(self, data: bytes):
        self.bytes += len(data)
        self.buf += data
        out = []
        while len(self.buf) >= 4:
            if self.buf[0] == 0xA5 and self.buf[1] == 0x5A and 1 <= self.buf[2] <= 32:
                n = self.buf[2]
                if len(self.buf) < 3 + n + 1:
                    break
                p, c = bytes(self.buf[3:3 + n]), self.buf[3 + n]
                del self.buf[:3 + n + 1]
                if crc8(p) == c:
                    self.ok += 1
                    out.append(p)
                else:
                    self.bad += 1
            else:
                del self.buf[0]
        return out


def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.02, write_timeout=0.2)
    print(f"{PORT} @ {BAUD}. Ctrl+C để dừng.")
    stop = threading.Event()
    seq = [0]

    def tx():
        while not stop.is_set():
            try:
                ser.write(make_frame(seq[0]))
            except serial.SerialException:
                pass
            seq[0] += 1
            time.sleep(0.05)
    threading.Thread(target=tx, daemon=True).start()

    ps = Parser()
    esp_got = 0
    last_seq = None
    t_print = time.time() + 1
    try:
        while True:
            for p in ps.feed(ser.read(256)):
                if p[0] == 0x30 and len(p) >= 8:
                    last_seq = p[1]
                    esp_got = struct.unpack("<I", p[2:6])[0]
            if time.time() >= t_print:
                t_print += 1
                a = f"ESP32->Pi: {ps.ok} khung đúng, {ps.bad} sai, {ps.bytes} byte, seq cuối {last_seq}"
                b = f"Pi->ESP32: Pi đã gửi {seq[0]} khung, ESP32 báo nhận đúng {esp_got}"
                print(a + "   |   " + b)
                if ps.bytes == 0:
                    print("   ! chưa nhận byte nào từ ESP32: kiểm tra ESP32 GPIO23 -> Pi chân 10, GND chung, và ESP32 đang chạy ORC_UartTest")
                elif seq[0] > 40 and esp_got == 0:
                    print("   ! ESP32 nghe được Pi? Chưa: kiểm tra Pi chân 8 -> ESP32 GPIO34 (đừng đấu chéo/đảo)")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        ser.close()


if __name__ == "__main__":
    main()
