// Minimal host-side mock of the Arduino core, just enough for tars.ino.
// Time is simulated (mock_now_us), pins are recorded, Serial is a pair of
// in-memory buffers. Used only by test_tars.cpp.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <string>
#include <deque>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define HEX 16
#define DEC 10
#define A0 14
#define A1 15
#define A2 16
#define A3 17

struct __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper *>(s))

extern uint64_t mock_now_us;
extern int mock_pin_mode[32];
extern int mock_pin_digital[32];   // last digitalWrite level, -1 = never
extern int mock_pin_pwm[32];       // last analogWrite value, -1 = digital mode

inline unsigned long millis() { return (unsigned long)(uint32_t)(mock_now_us / 1000); }
inline unsigned long micros() { return (unsigned long)(uint32_t)mock_now_us; }
inline void delay(unsigned long ms) { mock_now_us += (uint64_t)ms * 1000; }
inline void pinMode(uint8_t p, uint8_t m) { mock_pin_mode[p] = m; }
inline void digitalWrite(uint8_t p, uint8_t v) { mock_pin_digital[p] = v; mock_pin_pwm[p] = -1; }
// Like the real core: 0 and 255 become plain digital writes.
inline void analogWrite(uint8_t p, int v) {
  if (v <= 0) { digitalWrite(p, LOW); return; }
  if (v >= 255) { digitalWrite(p, HIGH); mock_pin_pwm[p] = 255; return; }
  mock_pin_pwm[p] = v; mock_pin_digital[p] = -1;
}

class MockSerial {
 public:
  std::deque<char> in;
  std::string out;
  void begin(long) {}
  int available() { return (int)in.size(); }
  int availableForWrite() { return 63; }
  int read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return (unsigned char)c; }
  void feed(const std::string &s) { for (char c : s) in.push_back(c); }
  void print(const char *s) { out += s; }
  void print(const __FlashStringHelper *s) { out += reinterpret_cast<const char *>(s); }
  void print(unsigned long v, int base = DEC) {
    char b[24]; snprintf(b, sizeof b, base == HEX ? "%lX" : "%lu", v); out += b;
  }
  void print(unsigned int v, int base = DEC) { print((unsigned long)v, base); }
  void print(long v, int base = DEC) {
    if (base == DEC && v < 0) { out += "-"; print((unsigned long)(-v), base); }
    else print((unsigned long)v, base);
  }
  void print(int v, int base = DEC) { print((long)v, base); }
  void print(uint8_t v, int base = DEC) { print((unsigned long)v, base); }
  template <typename T> void println(T v) { print(v); out += "\r\n"; }
  template <typename T> void println(T v, int base) { print(v, base); out += "\r\n"; }
};
extern MockSerial Serial;
