/*
 * Motors.h — điều khiển 7 motor qua 4 mạch L298N (PWM trên IN1/IN2, ENA/ENB cắm jumper)
 */
#pragma once
#include "config.h"
#include "Common.h"

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach((pin), PWM_FREQ, PWM_BITS)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((pin), (duty))
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup((ch), PWM_FREQ, PWM_BITS); ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((ch), (duty))
#endif

void motorsBegin() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (MOTOR_IN1[i] < 0 || MOTOR_IN2[i] < 0) continue;
    PWM_ATTACH(MOTOR_IN1[i], i * 2);
    PWM_ATTACH(MOTOR_IN2[i], i * 2 + 1);
    PWM_WRITE(MOTOR_IN1[i], i * 2, 0);
    PWM_WRITE(MOTOR_IN2[i], i * 2 + 1, 0);
  }
}

// pwm: -PWM_MAX..PWM_MAX (dương = IN1 có xung, âm = IN2 có xung)
void motorSet(int id, int pwm) {
  if (id < 0 || id >= MOTOR_COUNT || MOTOR_IN1[id] < 0 || MOTOR_IN2[id] < 0) return;
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (MOTOR_INVERT[id]) pwm = -pwm;
  if (pwm >= 0) {
    PWM_WRITE(MOTOR_IN1[id], id * 2, pwm);
    PWM_WRITE(MOTOR_IN2[id], id * 2 + 1, 0);
  } else {
    PWM_WRITE(MOTOR_IN1[id], id * 2, 0);
    PWM_WRITE(MOTOR_IN2[id], id * 2 + 1, -pwm);
  }
}
