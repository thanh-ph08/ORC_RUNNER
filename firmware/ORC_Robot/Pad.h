/*
 * Pad.h — tay cầm PS4/PS5 qua Bluepad32
 * Bluepad32: a() = ✕ (Cross), b() = ◯ (Circle), x() = □ (Square), y() = △ (Triangle)
 */
#pragma once
#include <Bluepad32.h>
#include "config.h"
#include "Common.h"

struct PadState {
  bool ok;
  float lx, ly, rx, ry;          // -1..1, ly/ry: âm = đẩy lên
  bool cross, circle, square, triangle;
  bool l1, r1, l2, r2, thumbL, thumbR, up, down, left, right, start, share, ps;
};
PadState padNow = {}, padPrev = {};
#define PRESSED(f) (padNow.f && !padPrev.f)

static ControllerPtr pad = nullptr;

void onConnectedController(ControllerPtr ctl) {
  if (pad == nullptr) {
    pad = ctl;
    ControllerProperties pr = ctl->getProperties();
    Serial.printf("[PAD] Đã kết nối tay cầm: %s (VID=0x%04x PID=0x%04x)\n", ctl->getModelName().c_str(), pr.vendor_id, pr.product_id);
    ctl->setColorLED(0, 255, 0);
  } else {
    Serial.println("[PAD] Đã có tay cầm, bỏ qua tay thứ hai");
  }
}
void onDisconnectedController(ControllerPtr ctl) {
  if (pad == ctl) {
    pad = nullptr;
    Serial.println("[PAD] Mất kết nối tay cầm!");
  }
}

static float stickDeadband(float v) {
  if (fabsf(v) < STICK_DEADBAND) return 0;
  return (v - copysignf(STICK_DEADBAND, v)) / (1.0f - STICK_DEADBAND);
}

void padRead() {
  padPrev = padNow;
  PadState s = {};
  if (pad && pad->isConnected() && pad->isGamepad()) {
    s.ok = true;
    s.lx = stickDeadband(pad->axisX() / 512.0f);
    s.ly = stickDeadband(pad->axisY() / 512.0f);
    s.rx = stickDeadband(pad->axisRX() / 512.0f);
    s.ry = stickDeadband(pad->axisRY() / 512.0f);
    s.cross = pad->a(); s.circle = pad->b(); s.square = pad->x(); s.triangle = pad->y();
    s.l1 = pad->l1(); s.r1 = pad->r1(); s.l2 = pad->l2(); s.r2 = pad->r2();
    s.thumbL = pad->thumbL(); s.thumbR = pad->thumbR();
    uint8_t d = pad->dpad();
    s.up = d & 0x01; s.down = d & 0x02; s.right = d & 0x04; s.left = d & 0x08;
    uint16_t m = pad->miscButtons();
    s.ps = m & 0x01; s.share = m & 0x02; s.start = m & 0x04;
  }
  padNow = s;
}

void setPadColor(uint8_t r, uint8_t g, uint8_t b) {
  if (pad && pad->isConnected()) pad->setColorLED(r, g, b);
}
