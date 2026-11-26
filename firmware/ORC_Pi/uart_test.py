"""
uart_test.py — kiểm tra dây UART giữa Raspberry Pi và ESP32#1, KHÔNG cần chạy orc_pi.py.
Dừng dịch vụ trước:  sudo systemctl stop orc-pi

  sudo python3 uart_test.py loop [baud]   # nối tạm chân TX (GPIO14, chân 8) với RX (GPIO15, chân 10) của Pi bằng 1 sợi dây; bỏ hết dây tới ESP32
  sudo python3 uart_test.py rx     # nghe ESP32#1 trong 8 s (ESP32 phải đang chạy bản ORC_Robot mới): kỳ vọng thấy khung A5 5A
  sudo python3 uart_test.py tx     # Pi gửi liên tục khung toạ độ giả 10 s; ESP32 sẽ ngừng báo "mất Raspberry Pi" nếu nhận được
"""
import sys, time, struct
import serial

PORT = "/dev/serial0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 921600   # vd: sudo python3 uart_test.py loop 115200

def crc8(b):
    c = 0
    for x in b:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c

mode = sys.argv[1] if len(sys.argv) > 1 else "rx"
ser = serial.Serial(PORT, BAUD, timeout=0.2)
print(f"{PORT} @ {BAUD}")

if mode == "loop":
    ser.reset_input_buffer()
    msg = bytes(range(32))
    ok = 0
    for i in range(20):
        ser.write(msg)
        time.sleep(0.05)
        got = ser.read(64)
        ok += got == msg
    print(f"Loopback đúng {ok}/20 lần.", "=> cổng UART của Pi HOẠT ĐỘNG tốt" if ok >= 18 else
          "=> cổng UART của Pi LỖI: kiểm tra config.txt (enable_uart=1, dtoverlay=disable-bt), raspi-config tắt login shell, và dây nối TX-RX")
elif mode == "rx":
    buf = bytearray(); total = 0; frames = 0; bad = 0; kinds = {}
    t_end = time.time() + 8
    while time.time() < t_end:
        d = ser.read(256)
        total += len(d); buf += d
        while len(buf) >= 4:
            if buf[0] == 0xA5 and buf[1] == 0x5A and 1 <= buf[2] <= 32:
                n = buf[2]
                if len(buf) < 3 + n + 1:
                    break
                p = bytes(buf[3:3 + n]); c = buf[3 + n]
                if crc8(p) == c:
                    frames += 1; kinds[p[0]] = kinds.get(p[0], 0) + 1
                else:
                    bad += 1
                del buf[:3 + n + 1]
            else:
                del buf[0]
    print(f"Nhận {total} byte, {frames} khung đúng CRC, {bad} khung sai CRC. Loại khung (0x20=log, 0x21=ping, 0x22=imu): {kinds}")
    if total == 0:
        print("=> KHÔNG có byte nào: sai dây (TX ESP32 = GPIO23 phải vào RX Pi = GPIO15 chân 10), thiếu GND chung, hoặc ESP32 chưa chạy bản mới")
    elif frames == 0:
        print("=> Có byte nhưng không ra khung: sai baud (cần 921600) hoặc cổng UART của Pi là mini-UART không ổn định (cần dtoverlay=disable-bt)")
    else:
        print("=> ESP32#1 -> Pi HOẠT ĐỘNG")
elif mode == "tx":
    body = bytes(39)                       # gói toạ độ giả toàn số 0 (hợp lệ: CRC đúng) => ESP32 sẽ hiện x=0 y=0 th=0
    frame = b"\xAA\x55" + body + bytes([crc8(body)])
    t_end = time.time() + 10
    n = 0
    while time.time() < t_end:
        ser.write(frame); n += 1; time.sleep(0.01)
    print(f"Đã gửi {n} khung giả. Nhìn Serial Monitor ESP32#1 (115200 baud): nếu nhận được thì dòng '! mất Raspberry Pi' biến mất "
          "và dòng x=... đổi thành x=0 y=0 th=0.0 (có thể kèm '[POSE] Pi không xác nhận lệnh đặt toạ độ'). Báo 'lỗi gói Pi' tăng = byte tới nhưng sai CRC/nhiễu.")
