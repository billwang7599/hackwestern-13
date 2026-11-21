// Host mock of the AVR EEPROM library (1 KB, survives setup() like the real one).
#pragma once
#include <cstring>
#include <cstdint>
struct MockEeprom {
  uint8_t mem[1024];
  MockEeprom() { memset(mem, 0xFF, sizeof mem); }   // erased EEPROM reads 0xFF
  template <typename T> T &get(int addr, T &t) { memcpy(&t, mem + addr, sizeof t); return t; }
  template <typename T> const T &put(int addr, const T &t) { memcpy(mem + addr, &t, sizeof t); return t; }
  void clear() { memset(mem, 0xFF, sizeof mem); }
};
extern MockEeprom EEPROM;
