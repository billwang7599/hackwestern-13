// Host mock of Wire with a fake MPU6050 (accel 0x3B.., gyro 0x43..).
#pragma once
#include "Arduino.h"

#define WIRE_HAS_TIMEOUT 1

struct MockMpu {
  bool present = false;
  uint8_t addr = 0x68;
  uint8_t regs[128] = {0};
  void set16(uint8_t r, int16_t v) { regs[r] = (uint8_t)(v >> 8); regs[r + 1] = (uint8_t)v; }
  void setAccel(int16_t x, int16_t y, int16_t z) { set16(0x3B, x); set16(0x3D, y); set16(0x3F, z); }
  void setGyro(int16_t x, int16_t y, int16_t z) { set16(0x43, x); set16(0x45, y); set16(0x47, z); }
};
extern MockMpu mock_mpu;

class MockWire {
  uint8_t txAddr = 0, txBuf[8] = {0}, txLen = 0, ptr = 0, rx[16] = {0}, rxLen = 0, rxPos = 0;
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
