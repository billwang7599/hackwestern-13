// Host mock of the AVR EEPROM library (get/put only).
#pragma once
#include <cstring>
#include <cstdint>

struct MockEEPROM {
  uint8_t data[1024];
  MockEEPROM() { memset(data, 0xFF, sizeof data); }
  template <typename T> T &get(int addr, T &t) { memcpy(&t, data + addr, sizeof(T)); return t; }
  template <typename T> const T &put(int addr, const T &t) { memcpy(data + addr, &t, sizeof(T)); return t; }
  void clear() { memset(data, 0xFF, sizeof data); }
};
extern MockEEPROM EEPROM;
