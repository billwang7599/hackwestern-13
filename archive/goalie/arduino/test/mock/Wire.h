// Host mock of Wire with a fake MPU6050 whose accel registers the test sets.
#pragma once
#include "Arduino.h"

#define WIRE_HAS_TIMEOUT 1

struct MockMpu {
  bool present = false;
  uint8_t addr = 0x68;
  uint8_t regs[128] = {0};
  void setAccel(int16_t x, int16_t y, int16_t z) {
    regs[0x3B] = (uint8_t)(x >> 8); regs[0x3C] = (uint8_t)x;
    regs[0x3D] = (uint8_t)(y >> 8); regs[0x3E] = (uint8_t)y;
    regs[0x3F] = (uint8_t)(z >> 8); regs[0x40] = (uint8_t)z;
  }
};
extern MockMpu mock_mpu;

class MockWire {
  uint8_t txAddr = 0, txBuf[8] = {0}, txLen = 0, ptr = 0, rx[8] = {0}, rxLen = 0, rxPos = 0;
 public:
  void begin() {}
  void setClock(uint32_t) {}
  void setWireTimeout(uint32_t, bool) {}
  void beginTransmission(uint8_t a) { txAddr = a; txLen = 0; }
  size_t write(uint8_t b) { if (txLen < sizeof txBuf) txBuf[txLen++] = b; return 1; }
  uint8_t endTransmission(bool = true) {
    if (!mock_mpu.present || txAddr != mock_mpu.addr) return 2;  // NACK
    if (txLen >= 1) ptr = txBuf[0];
    if (txLen >= 2) mock_mpu.regs[ptr & 127] = txBuf[1];
    return 0;
  }
  uint8_t requestFrom(uint8_t a, uint8_t n) {
    if (!mock_mpu.present || a != mock_mpu.addr || n > sizeof rx) return 0;
    for (uint8_t i = 0; i < n; i++) rx[i] = mock_mpu.regs[(ptr + i) & 127];
    rxLen = n; rxPos = 0;
    return n;
  }
  int read() { return rxPos < rxLen ? rx[rxPos++] : -1; }
};
extern MockWire Wire;
