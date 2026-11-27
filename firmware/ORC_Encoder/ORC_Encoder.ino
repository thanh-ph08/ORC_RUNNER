/*
 * ORC_Encoder.ino — ESP32#2 (I2C SLAVE 0x42): đọc 2 encoder quadrature, gửi về ESP32#1
 *
 *   Encoder 1: bánh bắn banh — encoder Hall của động cơ GA25, 2 kênh A/B (GPIO8, GPIO9), đọc x4.
 *   Encoder 2: nâng hạ — NEMA 17 quay theo trục lift làm "máy phát": điện áp 2 cuộn dây (phase A, B) qua 2 bộ so sánh LM393
 *              ra 2 xung vuông lệch nhau sẵn 90° (D0 -> GPIO7, D0 thứ 2 -> GPIO6) => đọc x4, biết chiều.
 *              NEMA 1.8° = 50 chu kỳ điện / vòng => 200 count / vòng trục NEMA. Đứng yên hoặc quay rất chậm thì gần như không có xung.
 *              LIFT_QUAD 0: chỉ dùng 1 phase (GPIO7), ESP32#1 lấy chiều theo lệnh motor lift.
 *
 *   !! CẤP NGUỒN CẢ HAI CẢM BIẾN 3.3V. Cấp 5V thì tín hiệu ra 5V, ESP32-S3 không chịu được.
 *   !! Chân dùng INPUT_PULLUP nội; nếu xung yếu/nhiễu thì thêm trở 10k kéo lên 3.3V.
 *
 * Board: "ESP32S3 Dev Module" (OhStem ORC Control Hub rev D). Tools → USB CDC On Boot: Enabled để Serial Monitor hoạt động qua cổng USB-C.
 * Chân I2C: SDA 11, SCL 12 (nối chung bus với ESP32#1, RPi, hub Ohstem; cần trở kéo lên 3.3V trên bus).
 *
 * Bắn: x4 => 4 × số xung/vòng của encoder (GA25 thường 11 xung/vòng ở trục motor, nhân tỉ số hộp số nếu đo ở trục ra).
 * Lift: 2 phase => 4 count mỗi chu kỳ điện (200 / vòng NEMA 1.8°); 1 phase => 2 count mỗi chu kỳ. ESP32#2 chỉ đếm + tính tốc độ (count/s);
 * đổi sang rpm / mm do ESP32#1 làm (config.h).
 */
#include <Arduino.h>
#include <Wire.h>
#include "orc_protocol.h"

// ============================ CẤU HÌNH ============================
// (Chân tạm — sửa lại cho đúng dây của bạn.)
// Board: OhStem ORC Control Hub rev D (ESP32-S3-WROOM). Trên S3, GPIO26–32 là flash, không dùng được.
// Chân dưới đây là chân Grove của board: A0=GPIO1, A1=GPIO2, D3=GPIO6, D4=GPIO7 (đối chiếu chữ in trên board).
const int SHOOTER_PIN_A = 8, SHOOTER_PIN_B = 9;     // encoder GA25: A -> GPIO8, B -> GPIO9
const int LIFT_PIN      = 7;                        // LM393 (phase A của NEMA) D0 -> GPIO7
// 1 = dùng cả 2 phase (LM393 thứ 2 ở GPIO6) => đọc x4, biết chiều. 0 = chỉ phase A. Phải khớp LIFT_ENC_SINGLE ở ESP32#1 (1 <-> 0).
#define LIFT_QUAD 1
const int LIFT_PIN_B    = 6;                        // LM393 (phase B của NEMA) D0 -> GPIO6
const uint32_t LIFT_MIN_EDGE_US = 150;              // chỉ dùng khi LIFT_QUAD 0: bỏ cạnh rung của bộ so sánh; tăng nếu đếm thừa
const int I2C_SDA = 11, I2C_SCL = 12;   // ESP32#2: SDA = GPIO11, SCL = GPIO12 (ESP32#1 vẫn dùng 21/22, mỗi board chọn chân riêng)
const uint32_t I2C_CLOCK = 100000;     // khớp ESP32#1

const uint32_t VEL_PERIOD_MS = 10;      // chu kỳ tính tốc độ
const float    VEL_FILTER    = 0.35f;   // 0..1, càng lớn càng bám nhanh nhưng nhiễu hơn

// ============================ ENCODER ============================
struct Quad {
  int pinA, pinB;
  volatile int32_t count;
  volatile uint8_t state;
};
Quad shooterEnc = {SHOOTER_PIN_A, SHOOTER_PIN_B, 0, 0};

// Lift: 1 kênh, đếm mọi cạnh (lên + xuống), bỏ cạnh rung
struct Pulse { int pin; volatile int32_t count; volatile uint32_t lastUs; };
#if LIFT_QUAD
Quad liftEnc = {LIFT_PIN, LIFT_PIN_B, 0, 0};
#else
Pulse liftEnc = {LIFT_PIN, 0, 0};
#endif

// Bảng trạng thái x4: chỉ số = (trạng thái cũ << 2) | trạng thái mới; giá trị -1/0/+1
static const int8_t QTAB[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void IRAM_ATTR quadIsr(void *arg) {
  Quad *q = (Quad *)arg;
  uint8_t s = (uint8_t)((digitalRead(q->pinA) << 1) | digitalRead(q->pinB));
  q->count += QTAB[((q->state << 2) | s) & 0x0F];
  q->state = s;
}

void quadBegin(Quad &q) {
  pinMode(q.pinA, INPUT_PULLUP);
  pinMode(q.pinB, INPUT_PULLUP);
  q.state = (uint8_t)((digitalRead(q.pinA) << 1) | digitalRead(q.pinB));
  attachInterruptArg(digitalPinToInterrupt(q.pinA), quadIsr, &q, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(q.pinB), quadIsr, &q, CHANGE);
}

void IRAM_ATTR pulseIsr(void *arg) {
  Pulse *p = (Pulse *)arg;
  uint32_t t = micros();
  if (t - p->lastUs < LIFT_MIN_EDGE_US) return;
  p->lastUs = t;
  p->count++;
}

void pulseBegin(Pulse &p) {
  pinMode(p.pin, INPUT_PULLUP);
  attachInterruptArg(digitalPinToInterrupt(p.pin), pulseIsr, &p, CHANGE);
}

// ============================ I2C SLAVE ============================
static EncoderPacket pkt;                 // gói đang phát (cập nhật trong loop)
static portMUX_TYPE pktMux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t seq = 0;
static volatile uint32_t reqCount = 0, rcvCount = 0;   // đếm số lần ESP32#1 đọc / ghi tới (để biết bus I2C có tới được không)

void onRequestCb() {
  reqCount++;
  portENTER_CRITICAL(&pktMux);
  EncoderPacket copy = pkt;
  portEXIT_CRITICAL(&pktMux);
  Wire.write((const uint8_t *)&copy, sizeof(copy));
}

void onReceiveCb(int n) {
  rcvCount++;
  while (Wire.available()) {
    uint8_t cmd = Wire.read();
    if (cmd == ENC_CMD_RESET_SHOOTER || cmd == ENC_CMD_RESET_ALL) shooterEnc.count = 0;
    if (cmd == ENC_CMD_RESET_LIFT || cmd == ENC_CMD_RESET_ALL) liftEnc.count = 0;
  }
}

void setup() {
  Serial.begin(115200);
  quadBegin(shooterEnc);
#if LIFT_QUAD
  quadBegin(liftEnc);
#else
  pulseBegin(liftEnc);
#endif

#if defined(CONFIG_IDF_TARGET_ESP32)
  // ESP32 thường: GPIO6–11 nối với flash của chip, dùng làm I2C sẽ làm board treo/khởi động lại liên tục.
  if ((I2C_SDA >= 6 && I2C_SDA <= 11) || (I2C_SCL >= 6 && I2C_SCL <= 11))
    Serial.println("!! CẢNH BÁO: GPIO6–11 là chân flash của ESP32 thường. Đổi I2C_SDA/I2C_SCL sang 21/22.");
#endif
  Wire.onReceive(onReceiveCb);
  Wire.onRequest(onRequestCb);
  bool i2cOk = Wire.begin((uint8_t)ORC_ADDR_ENCODER, I2C_SDA, I2C_SCL, I2C_CLOCK);
  if (!i2cOk) Serial.println("!! Wire.begin (slave) THẤT BẠI — kiểm tra chân SDA/SCL và core ESP32");

  memset(&pkt, 0, sizeof(pkt));
  pkt.crc = orcCrc8((uint8_t *)&pkt, sizeof(pkt) - 1);
  Serial.println("ORC_Encoder (ESP32#2) sẵn sàng, I2C 0x42");
}

void loop() {
  static uint32_t lastMs = 0;
  static int32_t lastShooter = 0, lastLift = 0;
  static float cpsShooter = 0, cpsLift = 0;

  uint32_t now = millis();
  if (now - lastMs >= VEL_PERIOD_MS) {
    float dt = (now - lastMs) * 1e-3f;
    lastMs = now;
    int32_t s = shooterEnc.count, l = liftEnc.count;
    cpsShooter += VEL_FILTER * ((s - lastShooter) / dt - cpsShooter);
    cpsLift    += VEL_FILTER * ((l - lastLift) / dt - cpsLift);
    lastShooter = s; lastLift = l;

    EncoderPacket p;
    p.seq = ++seq;
    p.shooterCount = s; p.liftCount = l;
    p.shooterCps = (int32_t)lroundf(cpsShooter);
    p.liftCps = (int32_t)lroundf(cpsLift);
    p.crc = orcCrc8((uint8_t *)&p, sizeof(p) - 1);
    portENTER_CRITICAL(&pktMux);
    pkt = p;
    portEXIT_CRITICAL(&pktMux);
  }

  // In ra Serial 2 lần/giây để kiểm tra chiều quay khi chưa nối ESP32#1
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    Serial.printf("shooter=%ld (%ld cps)  lift=%ld cạnh (%ld cps, chân %d) | I2C: đọc=%lu ghi=%lu\n", (long)shooterEnc.count,
                  (long)lroundf(cpsShooter), (long)liftEnc.count, (long)lroundf(cpsLift), digitalRead(LIFT_PIN),
                  (unsigned long)reqCount, (unsigned long)rcvCount);
  }
}
