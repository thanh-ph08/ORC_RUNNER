/*
 * Common.h — tiện ích chung + trạng thái toàn cục của robot
 */
#pragma once
#include <Arduino.h>
#include <math.h>

enum RobotMode { RM_MANUAL = 0, RM_AUTO = 1 };
RobotMode robotMode = RM_MANUAL;
bool telemetryOn = true;

static inline float wrapPi(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline float signf(float v) { return v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f); }
