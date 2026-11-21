// Host unit tests for goalie.ino (line parser, watchdog, flag, stepper, HIT).
//   make -C goalie/arduino/test
// The sketch is compiled as-is against mock/Arduino.h + mock/Wire.h.
#include "Arduino.h"
#include "Wire.h"

uint64_t mock_now_us = 0;
int mock_pin_mode[32];
int mock_pin_digital[32];
int mock_pin_pwm[32];
MockSerial Serial;
MockWire Wire;
MockMpu mock_mpu;

#include "../goalie/goalie.ino"

#include <cstdlib>

static int failures = 0, checks = 0;
#define CHECK(cond)                                                          \
  do {                                                                       \
    checks++;                                                                \
    if (!(cond)) { failures++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// Reset every piece of sketch + mock state, then run setup().
static void boot(bool withMpu) {
  mock_now_us = 1000000;  // start at t=1 s so time math isn't trivially 0
  for (int i = 0; i < 32; i++) { mock_pin_mode[i] = -1; mock_pin_digital[i] = -1; mock_pin_pwm[i] = -1; }
  Serial.in.clear(); Serial.out.clear();
  mock_mpu = MockMpu();
  mock_mpu.present = withMpu;
  mock_mpu.setAccel(0, 0, 4096);  // 1 g on Z
  lineLen = 0; lineOverflow = false;
  motorAActive = false; lastMMs = 0;
  celebrating = false;
  stepPos = stepTarget = 0; stepPhase = 0; stepEnergized = false; lastStepUs = 0; stepArrivedMs = 0;
  mpuAddr = 0; lastAccelUs = 0; baselineMg = 1000; hitWindowOpen = false; hitPeakMg = 0;
  lastHitMs = 0; hitEverSent = false;
  setup();
}

// Run loop() for ms of simulated time in 100 us ticks.
static void run(uint32_t ms) {
  uint64_t end = mock_now_us + (uint64_t)ms * 1000;
  while (mock_now_us < end) { loop(); mock_now_us += 100; }
}
static void send(const char *s) { Serial.feed(s); loop(); }

// Motor A as a signed pwm value from the pin state (2-PWM driver).
static int motorA() {
  auto lvl = [](int p) { return mock_pin_pwm[p] >= 0 ? mock_pin_pwm[p] : (mock_pin_digital[p] == HIGH ? 255 : 0); };
  int a = lvl(PIN_A_IN1), b = lvl(PIN_A_IN2);
  if (a && b) return 0;  // brake
  return a - b;
}
static int motorB() {
  auto lvl = [](int p) { return mock_pin_pwm[p] >= 0 ? mock_pin_pwm[p] : (mock_pin_digital[p] == HIGH ? 255 : 0); };
  int a = lvl(PIN_B_IN1), b = lvl(PIN_B_IN2);
  if (a && b) return 0;
  return a - b;
}
static int coils() {
  int c = 0;
  for (int i = 0; i < 4; i++) if (mock_pin_digital[PIN_STEP[i]] == HIGH) c |= 1 << i;
  return c;
}
static int countOf(const std::string &hay, const char *needle) {
  int n = 0; size_t pos = 0;
  while ((pos = hay.find(needle, pos)) != std::string::npos) { n++; pos++; }
  return n;
}

static void test_boot() {
  puts("boot");
  boot(false);
  CHECK(Serial.out.find("# no accelerometer") != std::string::npos);
  CHECK(Serial.out.find("READY\r\n") != std::string::npos);
  CHECK(Serial.out.rfind("READY") > Serial.out.find("#"));
  CHECK(motorA() == 0 && motorB() == 0 && coils() == 0);
  boot(true);
  CHECK(Serial.out.find("# mpu6050 at 0x68") != std::string::npos);
  CHECK(mock_mpu.regs[0x1C] == 0x10);  // +-8g
  CHECK(mpuAddr == 0x68);
  CHECK(baselineMg > 990 && baselineMg < 1010);
  mock_mpu.addr = 0x69;
  boot(true);
  // boot() rebuilt mock_mpu; set 0x69 and re-init
  mock_mpu.addr = 0x69; mpuAddr = 0; CHECK(accelInit() && mpuAddr == 0x69);
}

static void test_motor_and_parser() {
  puts("motor + parser");
  boot(false);
  send("M 0.5\n");   CHECK(motorA() == 128);
  send("M -0.5\n");  CHECK(motorA() == -128);
  send("M 1\n");     CHECK(motorA() == 255);
  send("M 5\n");     CHECK(motorA() == 255);       // clamped
  send("M -7.5\n");  CHECK(motorA() == -255);
  send("M 0.01\n");  CHECK(motorA() == 0 && !motorAActive);  // deadband
  CHECK(mock_pin_digital[PIN_A_IN1] == HIGH && mock_pin_digital[PIN_A_IN2] == HIGH);  // brake
  send("m 0.25\r");  CHECK(motorA() == 64);        // lowercase, CR-only terminator
  send("M 0.3\r\n"); CHECK(motorA() == 77);        // CRLF
  send("  M   -0.3  \n"); CHECK(motorA() == -77);  // extra whitespace
  // Garbage must be ignored and leave the motor as it was.
  const char *bad[] = {"MX 1\n", "M abc\n", "M 0.5 junk\n", "M\n", "M nan\n", "Q 1\n",
                       "S 1\n", "N 1.5\n", "N\n", "C x\n", "\n", "   \n", "Z 3\n"};
  for (const char *b : bad) { send(b); CHECK(motorA() == -77); }
  // Overlong line: whole line discarded, next line still works.
  std::string longl = "M 0.9"; longl += std::string(60, '0'); longl += "\n";
  send(longl.c_str()); CHECK(motorA() == -77);
  // Binary garbage in a line: discarded.
  std::string bin = "M 0.9"; bin += '\x01'; bin += "\n";
  send(bin.c_str()); CHECK(motorA() == -77);
  send("M 0.9\n"); CHECK(motorA() == 230);
  // A line split across several reads is assembled.
  send("M -0."); CHECK(motorA() == 230);
  send("5\n");   CHECK(motorA() == -128);
  // Many lines in one burst (200 Hz from the Pi, buffered).
  send("M 0.1\nM 0.2\nM 0.4\n"); CHECK(motorA() == 102);
}

static void test_watchdog() {
  puts("watchdog");
  boot(false);
  send("M 0.6\n");
  run(150); CHECK(motorA() == 153);
  run(60);  CHECK(motorA() == 0 && !motorAActive);   // >200 ms since M
  // Refreshing M at 200 Hz keeps it running indefinitely.
  send("M 0.6\n");
  for (int i = 0; i < 400; i++) { run(5); send("M 0.6\n"); }
  CHECK(motorA() == 153);
  // Garbage lines don't feed the watchdog.
  for (int i = 0; i < 10; i++) { run(30); send("M bogus\n"); }
  CHECK(motorA() == 0);
}

static void test_stop_and_celebrate() {
  puts("stop + celebrate");
  boot(false);
  send("C 300\n"); CHECK(motorB() == 204 && celebrating);
  run(290);        CHECK(motorB() == 204);
  run(20);         CHECK(motorB() == 0 && !celebrating);
  send("C 50000\n"); CHECK(celebrateDurMs == CELEBRATE_MAX_MS);
  send("C 0\n");   CHECK(motorB() == 0 && !celebrating);
  send("C 1000\n");
  send("M 0.5\n");
  send("N 500\n"); run(50);
  CHECK(motorB() != 0 && motorA() != 0 && stepPos > 0);
  send("S\n");
  long held = stepPos;
  CHECK(motorA() == 0 && motorB() == 0 && !celebrating && coils() == 0);
  run(100); CHECK(stepPos == held);   // needle halted
  // M still works after S.
  send("M -1\n"); CHECK(motorA() == -255);
}

static void test_stepper() {
  puts("stepper");
  boot(false);
  send("N 10\n");
  // First tick only energizes the current phase, then one step per interval.
  CHECK(coils() == HALF_STEP[0] && stepPos == 0);
  uint64_t t0 = mock_now_us;
  int seenPhases = 0; long last = 0;
  while (stepPos != 10 && mock_now_us - t0 < 1000000) {
    loop(); mock_now_us += 50;
    if (stepPos != last) {
      last = stepPos;
      CHECK(coils() == HALF_STEP[stepPos & 7]);
      seenPhases++;
    }
  }
  CHECK(stepPos == 10 && seenPhases == 10);
  double elapsed_ms = (mock_now_us - t0) / 1000.0;
  // 10 steps at 1400 us each (+1 energize interval) ~= 14-15.5 ms
  CHECK(elapsed_ms >= 14.0 && elapsed_ms <= 15.6);
  CHECK(coils() != 0);
  run(25); CHECK(coils() == 0 && !stepEnergized);   // released when idle
  // Rate limit: ~714 steps/s.
  send("N 1010\n"); run(1000);
  CHECK(stepPos > 690 && stepPos < 730);
  run(1000); CHECK(stepPos == 1010);
  run(30);   CHECK(coils() == 0);
  // Backwards, then Z.
  send("N -5\n"); run(2000); CHECK(stepPos == -5);
  CHECK(stepPhase == (uint8_t)((-5 % 8 + 8) % 8));
  send("Z\n"); CHECK(stepPos == 0 && stepTarget == 0);
  send("N 3\n"); run(20); CHECK(stepPos == 3);
}

static void test_hit() {
  puts("hit detect");
  boot(true);
  run(500);
  CHECK(countOf(Serial.out, "HIT") == 0);           // steady 1 g: no hits
  Serial.out.clear();
  // 10 ms spike: 1 g + 0.5 g bump, peaking at +1.2 g.
  mock_mpu.setAccel(0, 0, 4096 + 2048); run(4);
  mock_mpu.setAccel(0, 0, 4096 + 4915); run(4);
  mock_mpu.setAccel(0, 0, 4096 + 2048); run(2);
  mock_mpu.setAccel(0, 0, 4096); run(30);
  CHECK(countOf(Serial.out, "HIT ") == 1);
  int mag = atoi(Serial.out.c_str() + Serial.out.find("HIT ") + 4);
  CHECK(mag >= 1150 && mag <= 1250);
  // Second spike within 250 ms of the first HIT: suppressed.
  mock_mpu.setAccel(0, 0, 4096 + 8192); run(6);
  mock_mpu.setAccel(0, 0, 4096); run(100);
  CHECK(countOf(Serial.out, "HIT ") == 1);
  // After the rate limit, a new spike is reported again.
  run(200);
  mock_mpu.setAccel(4096 * 2, 0, 4096); run(6);
  mock_mpu.setAccel(0, 0, 4096); run(30);
  CHECK(countOf(Serial.out, "HIT ") == 2);
  // Small vibration below threshold: nothing.
  for (int i = 0; i < 100; i++) { mock_mpu.setAccel(0, 0, 4096 + (i & 1 ? 800 : -800)); run(2); }
  mock_mpu.setAccel(0, 0, 4096); run(300);
  CHECK(countOf(Serial.out, "HIT ") == 2);
  // Tilted sensor (gravity on X now): baseline magnitude is unchanged, no HIT.
  mock_mpu.setAccel(4096, 0, 0); run(500);
  CHECK(countOf(Serial.out, "HIT ") == 2);
  // Sensor unplugged mid-run: reads fail, loop keeps going, no crash.
  mock_mpu.present = false;
  send("M 0.5\n"); run(50); CHECK(motorA() == 128);
}

int main() {
  test_boot();
  test_motor_and_parser();
  test_watchdog();
  test_stop_and_celebrate();
  test_stepper();
  test_hit();
  printf("%d/%d checks passed\n", checks - failures, checks);
  return failures ? 1 : 0;
}
