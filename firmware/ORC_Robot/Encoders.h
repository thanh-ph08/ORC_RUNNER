/*
 * Encoders.h — đọc encoder bắn banh + nâng hạ từ ESP32#2 qua I2C
 */
#pragma once
#include <Wire.h>
#include "config.h"
#include "orc_protocol.h"
#include "Common.h"
#include "Odometry.h"   // i2cReadPacket

int32_t encShooterCount = 0, encLiftCount = 0;   // count thô (tích luỹ từ lúc ESP32#2 bật)
int32_t encShooterCps = 0, encLiftCps = 0;       // count/giây
uint32_t encLastOkMs = 0, encErrCount = 0;
bool encEverOk = false;

bool encFresh() { return encEverOk && (millis() - encLastOkMs) < SENSOR_STALE_MS; }

void encodersUpdate() {
  EncoderPacket p;
  if (!i2cReadPacket(ORC_ADDR_ENCODER, (uint8_t *)&p, sizeof(p)) || orcCrc8((uint8_t *)&p, sizeof(p) - 1) != p.crc) {
    encErrCount++;
    return;
  }
  encShooterCount = p.shooterCount; encLiftCount = p.liftCount;
  encShooterCps = p.shooterCps;     encLiftCps = p.liftCps;
  encLastOkMs = millis();
  encEverOk = true;
}
