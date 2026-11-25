/*
 * PidCtrl.h — PID dùng chung cho shooter (tốc độ) và lift (vị trí)
 * Đầu ra cùng đơn vị với kp*err (ở đây là PWM). Có chống tích phân quá đà và D theo sai số.
 */
#pragma once
#include "Common.h"

struct Pid {
  float kp = 0, ki = 0, kd = 0;
  float iMax = 0;            // giới hạn |ki * integ| (đơn vị đầu ra)
  float integ = 0, prevErr = 0;
  bool first = true;

  void set(float p, float i, float d, float imax) { kp = p; ki = i; kd = d; iMax = imax; reset(); }
  void reset() { integ = 0; prevErr = 0; first = true; }

  // freezeI = true: không tích phân (khi đầu ra đang chạm trần)
  float update(float err, float dt, bool freezeI = false) {
    if (dt <= 0) return kp * err;
    if (!freezeI && ki > 0) {
      integ += err * dt;
      float lim = iMax / ki;
      integ = clampf(integ, -lim, lim);
    }
    float d = first ? 0 : (err - prevErr) / dt;
    first = false;
    prevErr = err;
    return kp * err + ki * integ + kd * d;
  }
};
