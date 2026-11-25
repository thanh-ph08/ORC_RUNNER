/*
 * Log.h — log của ESP32#1: in ra Serial (USB) VÀ gửi sang Raspberry Pi qua UART để hiện trên trang web giám sát.
 *
 * Dùng Log.print / Log.println / Log.printf như Serial. Log chỉ đi vào bộ đệm vòng 2 KB, Log.pump() (gọi trong loop)
 * mới gửi dần từng khung nhỏ qua Serial2 và KHÔNG chặn vòng điều khiển; đầy thì bỏ phần cũ nhất.
 * Cứ 1 giây còn gửi thêm 1 khung PING để Pi biết đường ESP32#1 -> Pi đang sống.
 */
#pragma once
#include <Arduino.h>
#include "orc_protocol.h"

static void rpiSend(const uint8_t *payload, uint8_t len);   // định nghĩa trong Odometry.h

class LogTee : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    Serial.write(b, n);
    portENTER_CRITICAL(&mux_);
    for (size_t i = 0; i < n; i++) {
      size_t nh = (head_ + 1) % N;
      if (nh == tail_) { tail_ = (tail_ + 1) % N; dropped++; }
      ring_[head_] = b[i];
      head_ = nh;
    }
    portEXIT_CRITICAL(&mux_);
    return n;
  }

  // Gọi trong loop(): gửi tối đa 1 khung mỗi 5 ms, chỉ khi Serial2 còn chỗ.
  void pump() {
    uint32_t now = millis();
    if (now - lastHb_ >= 1000) {
      uint8_t hb = FLOW_CMD_PING;
      if (Serial2.availableForWrite() >= 6) { rpiSend(&hb, 1); lastHb_ = now; }
    }
    if (micros() - lastSend_ < 5000) return;
    uint8_t buf[31];
    size_t n = 0;
    portENTER_CRITICAL(&mux_);
    size_t t = tail_;
    while (t != head_ && n < 30) { buf[1 + n++] = ring_[t]; t = (t + 1) % N; }
    portEXIT_CRITICAL(&mux_);
    if (n == 0) return;
    if (Serial2.availableForWrite() < (int)(n + 6)) return;
    buf[0] = FLOW_CMD_LOG;
    rpiSend(buf, (uint8_t)(n + 1));
    lastSend_ = micros();
    portENTER_CRITICAL(&mux_);
    tail_ = t;                       // chỉ xoá sau khi đã gửi (đơn luồng gửi nên t vẫn đúng trừ khi bộ đệm tràn)
    portEXIT_CRITICAL(&mux_);
  }

  uint32_t dropped = 0;

 private:
  static const size_t N = 2048;
  uint8_t ring_[N];
  volatile size_t head_ = 0, tail_ = 0;
  uint32_t lastHb_ = 0, lastSend_ = 0;
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};
static LogTee Log;
