/*
 * ORC_UartTest.ino — CHỈ để thử đường UART giữa ESP32#1 và Raspberry Pi. Không tay cầm, motor, I2C, Bluepad32.
 * Board: "ESP32 Dev Module" (hoặc esp32_bluepad32 > ESP32 Dev Module). Serial Monitor 115200.
 *
 * Dây:  ESP32 GPIO23 (TX2) -> Pi chân 10 (GPIO15, RXD)     ESP32 GPIO34 (RX2) <- Pi chân 8 (GPIO14, TXD)     GND chung
 *
 * ESP32 mỗi 100 ms gửi khung  A5 5A len payload crc8  (payload: 0x30, seq, số khung ĐÚNG đã nhận từ Pi, số khung SAI) sang Pi.
 * Pi (uart_link.py) mỗi 200 ms gửi khung  AA 55 len payload crc8 (payload: 0x31, seq) sang ESP32.
 * Mỗi khi nhận ĐÚNG 1 khung từ Pi, đèn D2 (GPIO2) chớp 1 lần (khung sai hoặc không có gì thì không chớp). Pi gửi 5 khung/giây nên đèn chớp 5 lần/giây.
 * Mỗi 0.5 s ESP32 in lên Serial Monitor số khung nhận được từ Pi. Mỗi 1 s Pi in số khung nhận từ ESP32 và số khung ESP32 báo đã nhận từ Pi.
 */
#include <Arduino.h>

static const int RX_PIN = 34, TX_PIN = 23;
static const uint32_t BAUD = 921600;
static const int LED_PIN = 2;                 // D2 (đèn trên board ESP32 DevKit thường ở GPIO2): chớp 1 lần mỗi khi nhận ĐÚNG 1 khung từ Pi
static const uint32_t LED_MS = 40;            // thời gian sáng mỗi lần chớp
static uint32_t ledOffAt = 0;
static volatile uint32_t rxEdges = 0;          // số lần chân RX (GPIO34) đổi từ cao xuống thấp: >0 nghĩa là CÓ tín hiệu điện từ dây Pi tới, dù UART có giải mã được hay không
static void IRAM_ATTR onRxEdge() { rxEdges++; }

static uint8_t crc8(const uint8_t *d, size_t n) {          // CRC-8 đa thức 0x07 (giống orc_protocol.h)
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

static uint32_t rxBytes = 0, rxOk = 0, rxBad = 0, txFrames = 0;
static uint8_t lastPiSeq = 0, txSeq = 0;
static uint32_t lastPiMs = 0;

static void sendFrame() {
  uint8_t p[8];
  p[0] = 0x30; p[1] = txSeq++;
  memcpy(p + 2, &rxOk, 4);
  uint16_t bad = (uint16_t)rxBad;
  memcpy(p + 6, &bad, 2);
  uint8_t head[3] = {0xA5, 0x5A, sizeof(p)};
  Serial2.write(head, 3);
  Serial2.write(p, sizeof(p));
  Serial2.write(crc8(p, sizeof(p)));
  txFrames++;
}

static void parseRx() {                                    // AA 55 len payload crc8
  static uint8_t st = 0, len = 0, idx = 0, buf[32];
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    rxBytes++;
    switch (st) {
      case 0: if (b == 0xAA) st = 1; break;
      case 1: st = (b == 0x55) ? 2 : (b == 0xAA ? 1 : 0); break;
      case 2: if (b >= 1 && b <= sizeof(buf)) { len = b; idx = 0; st = 3; } else st = 0; break;
      case 3: buf[idx++] = b; if (idx == len) st = 4; break;
      case 4:
        if (crc8(buf, len) == b && buf[0] == 0x31) { rxOk++; lastPiSeq = buf[1]; lastPiMs = millis(); digitalWrite(LED_PIN, HIGH); ledOffAt = millis() + LED_MS; }
        else rxBad++;
        st = 0;
        break;
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ORC_UartTest (ESP32) ===");
  Serial2.setRxBufferSize(1024);
  Serial2.begin(BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  attachInterrupt(digitalPinToInterrupt(RX_PIN), onRxEdge, FALLING);
  Serial.printf("UART2: RX=GPIO%d (nhận từ Pi), TX=GPIO%d (gửi sang Pi), %lu baud\n", RX_PIN, TX_PIN, (unsigned long)BAUD);
}

void loop() {
  parseRx();
  static uint32_t tTx = 0, tPrint = 0;
  uint32_t now = millis();
  if (ledOffAt && (int32_t)(now - ledOffAt) >= 0) { digitalWrite(LED_PIN, LOW); ledOffAt = 0; }   // tắt đèn sau LED_MS, không dùng delay
  if (now - tTx >= 100) { tTx = now; sendFrame(); }
  if (now - tPrint >= 500) {
    tPrint = now;
    Serial.printf("[ESP->Pi] gửi %lu khung | [chân GPIO%d] mức=%d, cạnh xuống=%lu | [Pi->ESP] %lu byte, %lu khung đúng, %lu sai, seq cuối %u, cách đây %lu ms\n",
                  (unsigned long)txFrames, RX_PIN, digitalRead(RX_PIN), (unsigned long)rxEdges, (unsigned long)rxBytes,
                  (unsigned long)rxOk, (unsigned long)rxBad, lastPiSeq, (unsigned long)(lastPiMs ? now - lastPiMs : 0));
    if (rxEdges == 0)
      Serial.println("   ! chân RX KHÔNG thấy tín hiệu nào: dây Pi chân 8 -> ESP32 GPIO34 chưa tới/đứt/đấu nhầm, hoặc thiếu GND chung, hoặc uart_link.py chưa chạy");
    else if (rxBytes == 0)
      Serial.println("   ! có tín hiệu trên chân RX nhưng UART không nhận byte: kiểm tra tốc độ (921600) hoặc nhiễu");
    else if (rxOk == 0)
      Serial.println("   ! nhận byte nhưng không ra khung đúng: sai giao thức/CRC hoặc nhiễu");
  }
}
