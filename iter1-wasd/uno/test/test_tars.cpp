// Host unit tests for tars.ino (no Arduino needed).
//   make -C iter1-wasd/uno/test
// The sketch is compiled as-is against mock/Arduino.h + mock/Wire.h.
#include "Arduino.h"
#include "Wire.h"
#include "EEPROM.h"

uint64_t mock_now_us = 0;
int mock_pin_input[32];
int mock_tone_hz[32];
MockEeprom EEPROM;
int mock_pin_mode[32];
int mock_pin_digital[32];
int mock_pin_pwm[32];
MockSerial Serial;
MockWire Wire;
MockMpu mock_mpu;

#include "../tars/tars.ino"

#include <cmath>
#include <cstdlib>
#include <set>
#include <vector>

static int failures = 0, checks = 0;
#define CHECK(cond)                                                          \
  do {                                                                       \
    checks++;                                                                \
    if (!(cond)) { failures++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

// Which accelerometer the "with sensor" boots use (main() runs the sensor tests
// once per type). 0 = MPU6050 @0x68, 1 = ADXL345 @0x53.
static int g_sensor = 0;
static const char *sensorName() { return g_sensor ? "adxl345" : "mpu6050"; }
static uint8_t sensorAddr() { return g_sensor ? 0x53 : 0x68; }

static void boot(bool withMpu, int addrOverride = -1, bool keepEeprom = false) {
  mock_now_us = 1000000;
  for (int i = 0; i < 32; i++) {
    mock_pin_mode[i] = -1; mock_pin_digital[i] = -1; mock_pin_pwm[i] = -1;
    mock_pin_input[i] = HIGH; mock_tone_hz[i] = 0;   // button released (pull-up)
  }
  if (!keepEeprom) EEPROM.clear();
  Serial.in.clear(); Serial.out.clear();
  mock_mpu = MockMpu();
  mock_mpu.present = withMpu;
  if (g_sensor) mock_mpu.makeAdxl();
  if (addrOverride >= 0) mock_mpu.addr = (uint8_t)addrOverride;
  mock_mpu.setAccel(0, 0, 4096);  // 1 g on Z = upright
  lineLen = 0; lineOverflow = false;
  driveActive = false; lastDMs = 0;
  for (auto &a : arms) a = Arm();
  accelType = ACCEL_NONE; accelAddr = 0; accelLsbPerG = MPU_LSB_PER_G; lastAccelUs = 0;
  upX = upY = 0; upZ = 1; fX = fY = 0; fZ = 1; tiltDeg = 0;
  tipped = false; tipSinceMs = 0; uprightSinceMs = 0; lastTipMsgMs = 0;
  baselineMg = 1000; bumpWindowOpen = false; bumpPeakMg = 0; lastBumpMs = 0; bumpEverSent = false;
  setup();
}

static void run(uint32_t ms) {
  uint64_t end = mock_now_us + (uint64_t)ms * 1000;
  while (mock_now_us < end) { loop(); mock_now_us += 100; }
}
static void send(const char *s) { Serial.feed(s); loop(); }

static int lvl(int p) { return mock_pin_pwm[p] >= 0 ? mock_pin_pwm[p] : (mock_pin_digital[p] == HIGH ? 255 : 0); }
// motorL/R: the commanded wheel speed on a 0..255 scale, so the same drive tests
// run against DC PWM and stepper velocity targets.
#if WHEEL_TYPE == WHEELS_STEPPER
static int motorL() { return (int)lround(wheels[0].vel / wheelMaxSps() * 255.0f); }
static int motorR() { return (int)lround(wheels[1].vel / wheelMaxSps() * 255.0f); }
#elif DRIVER_TYPE == DRIVER_2PWM
static int motorL() { int a = lvl(PIN_L_IN1), b = lvl(PIN_L_IN2); return (a && b) ? 0 : a - b; }
static int motorR() { int a = lvl(PIN_R_IN1), b = lvl(PIN_R_IN2); return (a && b) ? 0 : a - b; }
#else
static int enIn(int en, int in1, int in2) {
  int e = lvl(en), a = lvl(in1), b = lvl(in2);
  if (!e || (a && b) || (!a && !b)) return 0;
  return a ? e : -e;
}
static int motorL() { return enIn(PIN_L_EN, PIN_L_IN1, PIN_L_IN2); }
static int motorR() { return enIn(PIN_R_EN, PIN_R_IN1, PIN_R_IN2); }
#endif
// Steady-state reading of motorL()/motorR() for "D d" (no invert, gain 1): the same
// 0..255 scale for every build. DC builds map |d| to DC_MIN_DUTY..DC_MAX_DUTY.
static int pwmOf(float d) {
  d = d < -1 ? -1 : d > 1 ? 1 : d;
  if (fabsf(d) < DRIVE_DEADBAND) return 0;
#if WHEEL_TYPE == WHEELS_STEPPER
  return (int)lround(d * 255.0f);
#else
  float s = shape(d, false, 1.0f);
  int p = (int)(fabsf(s) * 255.0f + 0.5f);
  return s < 0 ? -p : p;
#endif
}
static const int P50 = pwmOf(0.5f);
static void run(uint32_t ms);
// DC builds kick a wheel from rest for DC_KICK_MS: wait it out before reading the steady duty.
static void settle() { if (WHEEL_TYPE == WHEELS_DC && DC_KICK_MS > 0) run(DC_KICK_MS + 1); }
static int wcoils(int w) {
#if WHEEL_TYPE == WHEELS_STEPPER
  int c = 0;
  for (int k = 0; k < 4; k++) if (mock_pin_digital[PIN_WHEEL[w][k]] == HIGH) c |= 1 << k;
  return c;
#else
  (void)w; return 0;
#endif
}
static int coils(int arm) {
  int c = 0;
  for (int k = 0; k < 4; k++) if (mock_pin_digital[PIN_ARM[arm][k]] == HIGH) c |= 1 << k;
  return c;
}
static int countOf(const std::string &hay, const char *needle) {
  int n = 0; size_t pos = 0;
  while ((pos = hay.find(needle, pos)) != std::string::npos) { n++; pos++; }
  return n;
}
static long degSteps(float d) { return lround(d * 4096.0f / 360.0f); }
static void tilt(float deg) {  // rotate gravity about X by deg
  float r = deg * 3.14159265f / 180.0f;
  mock_mpu.setAccel(0, (int16_t)(sinf(r) * 4096), (int16_t)(cosf(r) * 4096));
}

static void test_boot() {
  puts("boot");
  boot(false);
  CHECK(Serial.out.find("# no accelerometer") != std::string::npos);
  CHECK(Serial.out.find("READY\r\n") != std::string::npos);
  CHECK(Serial.out.rfind("READY") > Serial.out.find("#"));
  CHECK(motorL() == 0 && motorR() == 0 && wcoils(0) == 0 && wcoils(1) == 0);
  for (int a = 0; a < ARM_COUNT; a++) {
    CHECK(coils(a) == 0);
    for (int k = 0; k < 4; k++) CHECK(mock_pin_mode[PIN_ARM[a][k]] == OUTPUT);
  }
  if (PIN_BUTTON != NO_PIN) CHECK(mock_pin_mode[PIN_BUTTON] == INPUT_PULLUP);
  if (PIN_LED != NO_PIN) CHECK(mock_pin_mode[PIN_LED] == OUTPUT && mock_pin_digital[PIN_LED] == LOW);

  boot(true);
  {
    char want[32];
    snprintf(want, sizeof want, "# %s at 0x%X", sensorName(), sensorAddr());
    CHECK(Serial.out.find(want) != std::string::npos);
  }
  CHECK(Serial.out.find("# upright captured") != std::string::npos);
  CHECK(accelAddr == sensorAddr());
  CHECK(accelType == (g_sensor ? ACCEL_ADXL345 : ACCEL_MPU6050));
  CHECK(fabsf(baselineMg - 1000.0f) < 5.0f);   // 1 g read back correctly
  CHECK(fabsf(upZ - 1.0f) < 1e-3f);

  // Pins: every pin in use in this build is distinct, and none is serial/I2C.
  std::vector<int> used = {0, 1, 18 /*A4*/, 19 /*A5*/};
#if WHEEL_TYPE == WHEELS_STEPPER
  for (int w = 0; w < 2; w++) for (int k = 0; k < 4; k++) used.push_back(PIN_WHEEL[w][k]);
#else
  used.insert(used.end(), {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2});
#if DRIVER_TYPE == DRIVER_EN_IN
  used.insert(used.end(), {PIN_L_EN, PIN_R_EN});
#endif
#endif
  for (int a = 0; a < ARM_COUNT; a++) for (int k = 0; k < 4; k++) used.push_back(PIN_ARM[a][k]);
  for (int p : {PIN_BUTTON, PIN_LED, PIN_BUZZER}) if (p != NO_PIN) used.push_back(p);
  std::set<int> uniq(used.begin(), used.end());
  CHECK(uniq.size() == used.size());
  for (int p : used) CHECK(p >= 0 && p <= 19);
}

static void test_drive() {
  puts("drive");
  boot(false);
  send("D 0.5 -0.5\n"); settle();
  CHECK(motorL() == P50 && motorR() == -P50);
  send("D 2 -3\n"); settle();
  CHECK(motorL() == pwmOf(1) && motorR() == pwmOf(-1));
  send("D 0.01 -0.02\n");
  CHECK(motorL() == 0 && motorR() == 0);
  CHECK(!driveActive);
  send("d 0.3 0.3\r"); settle();
  CHECK(motorL() == pwmOf(0.3f) && motorR() == pwmOf(0.3f));
  send("D -1 1\r\n"); settle();
  CHECK(motorL() == pwmOf(-1) && motorR() == pwmOf(1));
#if CAR == CAR_CUSTOM
  CHECK(pwmOf(0.5f) == 128 && pwmOf(1) == 255 && pwmOf(0.3f) == 77);   // the old linear map
#endif
}

static void test_malformed() {
  puts("malformed");
  boot(false);
  send("D 0.4 0.4\n"); settle();
  int l = motorL();
  const char *bad[] = {"D 0.9\n", "D 0.9 0.9 0.9 0.9\n", "DX 1 1\n", "D a b\n", "D 0.9 0.9x\n",
                       "D nan 0.9\n", "D 0.900000000000000000000000000000000000000000 1\n",
                       "\x01D 0.9 0.9\n", "Q 1\n", "S 1\n", "A 2 10\n", "A 0\n", "A -1 30\n"};
  for (const char *b : bad) { send(b); CHECK(motorL() == l); }
  CHECK(arms[0].target == 0 && arms[1].target == 0);
  // a split line still works
  send("D 0."); send("2 0.2\n");
  CHECK(motorL() == pwmOf(0.2f));
}

static void test_watchdog() {
  puts("watchdog");
  boot(false);
  send("D 0.5 0.5\n");
  run(150);
  CHECK(motorL() == P50);
  run(100);
  CHECK(motorL() == 0 && motorR() == 0);
  // fed every 50 ms -> keeps running
  for (int i = 0; i < 20; i++) { send("D 0.5 0.5\n"); run(50); }
  CHECK(motorL() == P50);
  // junk doesn't feed it
  for (int i = 0; i < 5; i++) { send("D junk\n"); run(50); }
  CHECK(motorL() == 0);
}

static void test_arms() {
#if ARM_COUNT < 2
  return;
#endif
  puts("arms");
  boot(false);
  send("A 0 90\n");                       // default 45 dps -> 2 s
  CHECK(arms[0].target == degSteps(90));
  run(1000);
  CHECK(labs(arms[0].pos - degSteps(45)) < 30);
  CHECK(coils(0) != 0);
  run(1200);
  CHECK(arms[0].pos == degSteps(90));
  run(30);
  CHECK(coils(0) == 0);                   // released when idle

  // both at once, speed clamp to 60 dps -> 90 deg in 1.5 s
  send("A 0 0 200\n");
  send("A 1 -90 200\n");
  run(800);
  CHECK(arms[0].pos < degSteps(90) && arms[0].pos > 0);
  CHECK(arms[1].pos < 0 && arms[1].pos > degSteps(-90));
  run(900);
  CHECK(arms[0].pos == 0 && arms[1].pos == degSteps(-90));

  // arm 1 is mirrored: a positive move steps its phase backwards
  boot(false);
  send("A 1 10\n");
  run(3);                                 // energize + first step
  run(30);
  CHECK(arms[1].pos > 0);
  CHECK(arms[1].phase == (uint8_t)((8 - (arms[1].pos % 8)) % 8));
  send("A 0 10\n");
  run(30);
  CHECK(arms[0].phase == (uint8_t)(arms[0].pos % 8));

  // limit clamp
  send("A 0 500\n");
  CHECK(arms[0].target == degSteps(135));
  send("A 0 -500\n");
  CHECK(arms[0].target == degSteps(-135));

  // X halts, H homes, Z zeroes
  boot(false);
  send("A 0 90 60\n");
  run(500);
  send("X\n");
  long p = arms[0].pos;
  run(100);
  CHECK(arms[0].pos == p && p > 0);
  CHECK(coils(0) == 0);
  send("H\n");
  CHECK(arms[0].target == 0);
  run(1500);
  CHECK(arms[0].pos == 0);
  send("A 0 30 60\n");
  run(700);
  send("Z\n");
  CHECK(arms[0].pos == 0 && arms[0].target == 0);

  // S stops drive and halts arms
  send("D 0.5 0.5\n");
  send("A 1 90\n");
  run(300);
  send("S\n");
  long p1 = arms[1].pos;
  CHECK(motorL() == 0 && arms[1].target == p1);
  run(200);
  CHECK(arms[1].pos == p1);
}

static void test_status() {
  puts("status");
  boot(true);
  Serial.out.clear();
  send("?\n");
  CHECK(Serial.out.find(std::string("# arms 0 0 tilt 0 tipped 0 accel ") + sensorName()) != std::string::npos);
#if WHEEL_TYPE == WHEELS_STEPPER
  CHECK(Serial.out.find("wheels 0 0 move -") != std::string::npos);
#endif
  if (ARM_COUNT == 0) return;
  send("A 0 -20 60\n");
  run(600);
  Serial.out.clear();
  send("?\n");
  CHECK(Serial.out.find("# arms -20 0") != std::string::npos);
}

static void test_tip() {
  puts("tip");
  boot(true);
  run(100);
  send("A 0 90\n");
  for (int i = 0; i < 4; i++) { send("D 0.5 0.5\n"); run(40); }
  Serial.out.clear();
  tilt(60);
  send("D 0.5 0.5\n");
  run(120);                                // filter + hold not done yet
  CHECK(!tipped && motorL() == P50);
  CHECK(Serial.out.find("T ") == std::string::npos);
  send("D 0.5 0.5\n");
  run(150);
  CHECK(tipped);
  CHECK(Serial.out.find("T 6") != std::string::npos || Serial.out.find("T 5") != std::string::npos);
  CHECK(motorL() == 0 && motorR() == 0);
  CHECK(arms[0].target == arms[0].pos);   // arms halted
  // refuses to drive or move arms while tipped
  send("D 0.5 0.5\n");
  CHECK(motorL() == 0);
  send("A 0 10\n");
  CHECK(arms[0].target == arms[0].pos);
  Serial.out.clear();
  run(1100);
  CHECK(countOf(Serial.out, "T ") == 2);  // repeated every 500 ms
  // back upright: needs 500 ms under 25 deg
  tilt(0);
  run(300);
  CHECK(tipped);
  run(500);
  CHECK(!tipped);
  CHECK(Serial.out.find("# upright\r\n") != std::string::npos);
  send("D 0.5 0.5\n"); settle();
  CHECK(motorL() == P50);

  // a short tilt glitch (100 ms) is not a tip
  boot(true);
  run(50);
  Serial.out.clear();
  tilt(70);
  run(100);
  tilt(0);
  run(500);
  CHECK(!tipped && Serial.out.find("T ") == std::string::npos);

  // U recaptures upright: a robot standing on a slope isn't "tipped"
  boot(true);
  tilt(50);
  send("U\n");
  run(1000);
  CHECK(!tipped);
  CHECK(Serial.out.find("T ") == std::string::npos);
}

static void test_bump() {
  puts("bump");
  boot(true);
  run(200);
  Serial.out.clear();
  mock_mpu.setAccel(0, 0, 4096 * 3);       // 3 g spike, 10 ms
  run(10);
  mock_mpu.setAccel(0, 0, 4096);
  run(30);
  CHECK(countOf(Serial.out, "B ") == 1);
  int mg = atoi(Serial.out.c_str() + Serial.out.find("B ") + 2);
  CHECK(mg > 1800 && mg < 2100);
  CHECK(!tipped);
  // a second spike within 250 ms is rate limited
  mock_mpu.setAccel(0, 0, 4096 * 3);
  run(10);
  mock_mpu.setAccel(0, 0, 4096);
  run(50);
  CHECK(countOf(Serial.out, "B ") == 1);
  run(300);
  mock_mpu.setAccel(4096 * 2, 0, 4096);
  run(10);
  mock_mpu.setAccel(0, 0, 4096);
  run(30);
  CHECK(countOf(Serial.out, "B ") == 2);
  // small vibration (0.5 g) is not a bump
  Serial.out.clear();
  for (int i = 0; i < 50; i++) { mock_mpu.setAccel(0, 0, (i & 1) ? 4096 + 2048 : 4096 - 2048); run(5); }
  mock_mpu.setAccel(0, 0, 4096);
  CHECK(Serial.out.find("B ") == std::string::npos);
  // static tilt is not a bump (|a| is still 1 g)
  Serial.out.clear();
  tilt(30);
  run(500);
  CHECK(Serial.out.find("B ") == std::string::npos && !tipped);
}

static void test_no_mpu() {
  puts("no accelerometer / unplugged");
  boot(false);
  mock_mpu.setAccel(0, 0, 4096 * 4);
  run(500);
  CHECK(Serial.out.find("B ") == std::string::npos && Serial.out.find("T ") == std::string::npos);
  boot(true);
  run(50);
  mock_mpu.present = false;               // wire falls out mid-run
  send("D 0.5 0.5\n");
  run(100);
  CHECK(motorL() == P50);
  send("A 0 10 60\n");
  run(300);
  CHECK(arms[0].pos == (ARM_COUNT ? degSteps(10) : 0));
}

static void test_sensor_detect() {
  puts("sensor detection");
  int saved = g_sensor;
  g_sensor = 0;
  boot(true, 0x69);
  CHECK(accelType == ACCEL_MPU6050 && accelAddr == 0x69);
  CHECK(Serial.out.find("# mpu6050 at 0x69") != std::string::npos);
  g_sensor = 1;
  boot(true, 0x1D);
  CHECK(accelType == ACCEL_ADXL345 && accelAddr == 0x1D);
  CHECK(Serial.out.find("# adxl345 at 0x1D") != std::string::npos);
  // ADXL345 config registers as specified
  CHECK(mock_mpu.regs[0x2D] == 0x08 && mock_mpu.regs[0x31] == 0x0B);
  // something at 0x53 that isn't an ADXL345 (wrong DEVID) is not used
  boot(true);
  boot(false);
  mock_mpu.present = true; mock_mpu.adxl = true; mock_mpu.addr = 0x53; mock_mpu.regs[0] = 0x00;
  setup();
  CHECK(accelType == ACCEL_NONE && Serial.out.find("# no accelerometer") != std::string::npos);
  // little-endian decode with negative values
  g_sensor = 1;
  boot(true);
  mock_mpu.setAccel(-4096, 2048, -1024);
  int16_t ax, ay, az;
  CHECK(accelRead(&ax, &ay, &az));
  CHECK(ax == -256 && ay == 128 && az == -64);
  g_sensor = saved;
}

static void test_overflow() {
  puts("overflow");
  boot(false);
  std::string longline(80, '1');
  send(("D " + longline + " 1\n").c_str());
  CHECK(motorL() == 0);
  send("D 0.5 0.5\n"); settle();
  CHECK(motorL() == P50);
}


// ------------------------------------------------------------------ stepper wheels
static bool contains(const char *needle) { return Serial.out.find(needle) != std::string::npos; }
// Runs until `needle` shows up in Serial.out (or maxMs). Returns elapsed ms, -1 on timeout.
static long runUntil(const char *needle, uint32_t maxMs) {
  uint64_t start = mock_now_us;
  while (mock_now_us - start < (uint64_t)maxMs * 1000) {
    if (contains(needle)) return (long)((mock_now_us - start) / 1000);
    loop();
    mock_now_us += 100;
  }
  return contains(needle) ? (long)maxMs : -1;
}
static float lastAmount(const char *prefix) {   // "DONE M " -> the number after it
  size_t p = Serial.out.rfind(prefix);
  return p == std::string::npos ? NAN : (float)atof(Serial.out.c_str() + p + strlen(prefix));
}

#if WHEEL_TYPE == WHEELS_STEPPER
static long mmSteps(float mm) { return lround(mm * 4075.7728f / (3.14159265f * wheelDiamMm)); }
static long degWheelSteps(float deg) { return mmSteps(3.14159265f * wheelTrackMm * deg / 360.0f); }

static void test_gear_ratio() {
  puts("stepper: gear ratio");
  double ratio = (32.0 / 9) * (22.0 / 11) * (26.0 / 9) * (31.0 / 10);
  CHECK(fabs(ratio * 64 - WHEEL_STEPS_PER_REV) < 0.01);
  CHECK(fabsf(wheelMaxSps() - 815.15f) < 0.1f);         // 12 RPM
}

static void test_wheel_velocity() {
  puts("stepper: D velocity, ramp, watchdog, release");
  boot(false);
  // ramp: 50 ms after D 1 1 the rate is ~150 sps, not the 815 max
  send("D 1 1\n");
  run(50);
  CHECK(wheels[0].rate > 130 && wheels[0].rate < 170);
  CHECK(wcoils(0) != 0 && wcoils(1) != 0);
  for (int i = 0; i < 10; i++) { send("D 1 1\n"); run(50); }   // 0.55 s: at full speed
  CHECK(fabsf(wheels[0].rate - wheelMaxSps()) < 1);
  long p0 = wheels[0].pos, q0 = wheels[1].pos;
  for (int i = 0; i < 20; i++) { send("D 1 1\n"); run(50); }   // 1 s steady
  long n = wheels[0].pos - p0;
  CHECK(labs(n - 815) <= 10);                                 // average rate kept exact
  CHECK(wheels[1].pos - q0 == n);                             // both wheels in step
  // right wheel is mirrored: + steps walk its phase backwards
  CHECK(wheels[0].phase == (uint8_t)(((wheels[0].pos % 8) + 8) % 8));
  CHECK(wheels[1].phase == (uint8_t)(((-wheels[1].pos % 8) + 8) % 8));
  // watchdog: no D for 200 ms -> hard stop, coils off 100 ms later
  run(150);
  CHECK(wheels[0].rate > 0);
  run(100);
  CHECK(wheels[0].rate == 0 && wheels[0].vel == 0 && !driveActive);
  long hold = wheels[0].pos;
  run(150);
  CHECK(wheels[0].pos == hold && wcoils(0) == 0 && wcoils(1) == 0);
  // D 0 0 ramps down (not a hard stop) and reversal goes through zero
  for (int i = 0; i < 12; i++) { send("D 0.5 0.5\n"); run(50); }
  CHECK(fabsf(wheels[0].rate - 0.5f * wheelMaxSps()) < 1);
  send("D 0 0\n");
  run(30);
  CHECK(wheels[0].rate > 200 && wheels[0].rate < 0.5f * wheelMaxSps());   // ramping, not dead stop
  send("S\n");

  // hold-to-drive feel: full speed -> D 0 0 reaches 0 within 150 ms, and the
  // step interval never jumps (no step "lost" to a sudden stop)
  for (int i = 0; i < 12; i++) { send("D 1 1\n"); run(50); }
  CHECK(fabsf(wheels[0].rate - wheelMaxSps()) < 1);
  send("D 0 0\n");
  uint64_t t0 = mock_now_us;
  float prev = wheels[0].rate, maxDrop = 0;
  while (wheels[0].rate > 0 && mock_now_us - t0 < 400000) {
    loop(); mock_now_us += 100;
    if (prev - wheels[0].rate > maxDrop) maxDrop = prev - wheels[0].rate;
    prev = wheels[0].rate;
  }
  long stopMs = (long)((mock_now_us - t0) / 1000);
  CHECK(stopMs >= 120 && stopMs <= 150);
  CHECK(maxDrop <= 6000 * 0.0011f + 1);                      // never more than one ramp tick
  // reversal ramps through zero: braking at the decel rate, then accelerating
  for (int i = 0; i < 12; i++) { send("D 0.5 0.5\n"); run(50); }
  send("D -0.5 -0.5\n");
  bool sawZero = false;
  long posAtZero = 0;
  for (int i = 0; i < 400; i++) {
    run(1);
    if (wheels[0].rate == 0 && !sawZero) { sawZero = true; posAtZero = wheels[0].pos; }
    if (i % 50 == 0) send("D -0.5 -0.5\n");
  }
  CHECK(sawZero);
  CHECK(fabsf(wheels[0].rate + 0.5f * wheelMaxSps()) < 1);
  CHECK(wheels[0].pos < posAtZero);                            // now going backwards
  for (int i = 0; i < 12; i++) { send("D -0.5 0.5\n"); run(50); }
  CHECK(wheels[0].rate < -400 && wheels[1].rate > 400);       // spin in place
  send("S\n");
  CHECK(wheels[0].rate == 0 && wheels[1].rate == 0);
}

static void test_wheel_moves() {
  puts("stepper: M / R exact moves");
  boot(false);
  Serial.out.clear();
  send("M 100\n");
  CHECK(moveKind == 'M' && wheels[0].target == mmSteps(100) && wheels[1].target == mmSteps(100));
  long maxSkew = 0;
  uint64_t t0 = mock_now_us;
  while (!contains("DONE") && mock_now_us - t0 < 5000000ULL) {
    loop(); mock_now_us += 100;
    long sk = labs(wheels[0].pos - wheels[1].pos);
    if (sk > maxSkew) maxSkew = sk;
  }
  long ms = (long)((mock_now_us - t0) / 1000);
  CHECK(contains("DONE M 100.0\r\n"));
  CHECK(wheels[0].pos == mmSteps(100) && wheels[1].pos == mmSteps(100));   // exact
  CHECK(maxSkew <= 1);                                         // wheels in sync
  CHECK(ms > 2600 && ms < 3200);                               // ~2.85 s at 12 RPM
  CHECK(moveKind == 0 && wcoils(0) != 0);
  run(120);
  CHECK(wcoils(0) == 0 && wcoils(1) == 0);                     // released when done
  CHECK(wheels[0].pos == mmSteps(100) && wheels[1].pos == mmSteps(100));   // no creep after DONE

  // R +30 = clockwise: left forward, right back
  Serial.out.clear();
  long l0 = wheels[0].pos, r0 = wheels[1].pos;
  send("R 30\n");
  long t = runUntil("DONE", 3000);
  CHECK(contains("DONE R 30.0\r\n"));
  CHECK(wheels[0].pos - l0 == degWheelSteps(30) && wheels[1].pos - r0 == -degWheelSteps(30));
  CHECK(t > 500 && t < 1200);
  Serial.out.clear();
  l0 = wheels[0].pos; r0 = wheels[1].pos;
  send("r -90 0.5\n");                                         // left, half speed
  runUntil("DONE", 6000);
  CHECK(contains("DONE R -90.0\r\n"));
  CHECK(wheels[0].pos - l0 == -degWheelSteps(90) && wheels[1].pos - r0 == degWheelSteps(90));
  Serial.out.clear();
  l0 = wheels[0].pos;
  send("M -50\n");
  runUntil("DONE", 4000);
  CHECK(contains("DONE M -50.0\r\n") && wheels[0].pos - l0 == -mmSteps(50));
  // half speed is slower
  Serial.out.clear();
  send("M 100 0.5\n");
  long slow = runUntil("DONE", 9000);
  CHECK(slow > 4500 && contains("DONE M 100.0"));
  // zero-length move finishes at once; clamps
  Serial.out.clear();
  send("M 0\n");
  CHECK(contains("DONE M 0.0\r\n") && moveKind == 0);
  send("M 5000\n");
  CHECK(moveReq == 1000.0f && wheels[0].target - wheels[0].start == mmSteps(1000));
  send("R -1000\n");
  CHECK(contains("ABORT M ") && contains(" replaced"));
  CHECK(moveKind == 'R' && moveReq == -360.0f);
  send("S\n");
  // malformed
  Serial.out.clear();
  const char *bad[] = {"M\n", "M 1 2 3\n", "M x\n", "R\n", "MX 10\n", "M 10 abc\n"};
  for (const char *b : bad) { send(b); CHECK(moveKind == 0); }
  CHECK(Serial.out.empty());
}

static void test_wheel_aborts() {
  puts("stepper: aborts (S, D, tip, link) and keepalives");
  boot(false);
  Serial.out.clear();
  send("M 100\n");
  run(1000);
  send("S\n");
  CHECK(contains("ABORT M "));
  float a = lastAmount("ABORT M ");
  CHECK(a > 10 && a < 60);
  CHECK(contains(" stop\r\n") && moveKind == 0 && wheels[0].rate == 0);
  long p = wheels[0].pos;
  run(200);
  CHECK(wheels[0].pos == p);

  // typed on the bench (no D stream): no watchdog / link abort mid-move;
  // D 0 0 is a keepalive, non-zero D aborts and takes over
  boot(false);
  Serial.out.clear();
  send("R 90\n");
  run(800);
  CHECK(moveKind == 'R');
  send("D 0 0\n");
  run(50);
  CHECK(moveKind == 'R' && !contains("ABORT"));
  send("D 0.5 0.5\n");
  CHECK(contains("ABORT R ") && contains(" drive\r\n") && moveKind == 0);
  CHECK(wheels[0].vel > 0 && driveActive);
  send("S\n");

  // Pi streaming D: the move runs while D 0 0 keeps coming, aborts when it stops
  boot(false);
  for (int i = 0; i < 5; i++) { send("D 0 0\n"); run(20); }
  Serial.out.clear();
  send("M 50\n");
  CHECK(moveLinkGuard);
  long t = 0;
  while (!contains("DONE") && t < 3000) { send("D 0.000 0.000\n"); run(20); t += 20; }
  CHECK(contains("DONE M 50.0"));
  Serial.out.clear();
  send("M 100\n");
  run(300);
  CHECK(moveKind == 'M');
  run(600);                                                   // Pi went quiet
  CHECK(contains(" link\r\n") && moveKind == 0 && wheels[0].rate == 0);

  // tip mid-move
  boot(true);
  run(50);
  Serial.out.clear();
  send("M 200\n");
  run(500);
  tilt(60);
  run(400);
  CHECK(tipped && contains(" tipped\r\n") && moveKind == 0);
  CHECK(Serial.out.find("ABORT M") < Serial.out.find("T "));
  Serial.out.clear();
  send("M 10\n");
  CHECK(contains("ABORT M 0.0 tipped") && moveKind == 0);
}

static void test_wheel_cal() {
  puts("stepper: K calibration in EEPROM");
  boot(false);
  CHECK(contains("# wheels diam 65.0 track 100.0 steps/rev 4075.8"));
  Serial.out.clear();
  send("K 70 120\n");
  CHECK(contains("# wheels diam 70.0 track 120.0") && wheelDiamMm == 70.0f);
  send("K 5 120\n");                                          // out of range: ignored
  send("K 70\n");
  CHECK(wheelDiamMm == 70.0f && wheelTrackMm == 120.0f);
  boot(false, -1, true);                                       // reboot keeps EEPROM
  CHECK(wheelDiamMm == 70.0f && wheelTrackMm == 120.0f);
  CHECK(contains("# wheels diam 70.0 track 120.0"));
  Serial.out.clear();
  send("M 100\n");
  CHECK(wheels[0].target == mmSteps(100) && mmSteps(100) == lround(100 * 4075.7728f / (3.14159265f * 70)));
  runUntil("DONE", 4000);
  CHECK(contains("DONE M 100.0"));
  boot(false);                                                 // erased EEPROM -> defaults
  CHECK(wheelDiamMm == 65.0f && wheelTrackMm == 100.0f);
}
#endif

#if WHEEL_TYPE == WHEELS_DC
static bool braked(int ch) {        // both bridge inputs equal and the motor not driven
#if DRIVER_TYPE == DRIVER_EN_IN
  int en = ch ? PIN_R_EN : PIN_L_EN, a = ch ? PIN_R_IN1 : PIN_L_IN1, b = ch ? PIN_R_IN2 : PIN_L_IN2;
  return lvl(en) == 255 && lvl(a) == 255 && lvl(b) == 255;   // L298N: EN high, IN1 = IN2 = fast stop
#else
  int a = ch ? PIN_R_IN1 : PIN_L_IN1, b = ch ? PIN_R_IN2 : PIN_L_IN2;
  return lvl(a) == 255 && lvl(b) == 255;
#endif
}
static const int KICK = (int)(DC_KICK_DUTY * 255.0f + 0.5f);
static const long SETTLE = DC_KICK_MS > 0 ? DC_KICK_MS + 1 : 0;   // what settle() runs

static void test_dc_speed() {
  puts("dc: duty map, speed cap, kick");
  boot(false);
  // |d| -> DC_MIN_DUTY..DC_MAX_DUTY, monotonic, D 1 = the cap
  int prev = 0;
  bool mono = true;
  for (int i = 1; i <= 20; i++) { int p = pwmOf(i / 20.0f); if (p < prev) mono = false; prev = p; }
  CHECK(mono);
  CHECK(pwmOf(1) == (int)(DC_MAX_DUTY * 255.0f + 0.5f));
  CHECK(pwmOf(0.031f) == (int)((DC_MIN_DUTY + (DC_MAX_DUTY - DC_MIN_DUTY) * 0.031f) * 255.0f + 0.5f));
  CHECK(pwmOf(0.02f) == 0);
  send("D 5 5\n"); settle();
  CHECK(motorL() == pwmOf(1) && motorR() == pwmOf(1));      // never above the cap
  send("S\n");
  if (DC_KICK_MS == 0) {
    send("D 0.3 0.3\n");
    CHECK(motorL() == pwmOf(0.3f));                          // custom builds: no kick
    send("S\n");
    return;
  }
  // from rest: kick, then the real duty after DC_KICK_MS
  send("D 0.3 0.3\n");
  CHECK(motorL() == KICK && motorR() == KICK);
  run(DC_KICK_MS - 3);
  send("D 0.3 0.3\n");                                       // a repeat doesn't restart the kick
  CHECK(motorL() == KICK);
  run(4);
  CHECK(motorL() == pwmOf(0.3f) && motorR() == pwmOf(0.3f));
  // speeding up in the same direction: no kick
  send("D 0.6 0.6\n");
  CHECK(motorL() == pwmOf(0.6f));
  // reversing one side: that side kicks backwards, the other doesn't
  send("D -0.6 0.6\n");
  CHECK(motorL() == -KICK && motorR() == pwmOf(0.6f));
  run(DC_KICK_MS + 1);
  CHECK(motorL() == pwmOf(-0.6f));
  // S in the middle of a kick: braked at once, and the kick never comes back
  send("S\n");
  send("D 0.3 0.3\n");
  CHECK(motorL() == KICK);
  send("S\n");
  CHECK(motorL() == 0 && motorR() == 0 && braked(0) && braked(1));
  run(DC_KICK_MS + 20);
  CHECK(motorL() == 0 && braked(0));
  // watchdog during a kick
  send("D 0.3 0.3\n");
  run(250);
  CHECK(motorL() == 0 && !driveActive && braked(0));
}

static void test_dc_moves() {
  puts("dc: timed M / R");
  boot(false);
  CHECK(contains("WHEELS DC 150.0 120.0\r\n"));
  CHECK(Serial.out.find("WHEELS DC") < Serial.out.find("READY"));   // the Pi learns before READY
  Serial.out.clear();
  send("M 300\n");                                           // 300 / 150 mm/s = 2.0 s
  CHECK(moveKind == 'M' && moveDurMs == 2000);
  settle();
  CHECK(motorL() == pwmOf(1) && motorR() == pwmOf(1));       // forward at the cap
  CHECK(!driveActive);
  long t = runUntil("DONE", 4000);
  CHECK(contains("DONE M 300.0\r\n"));
  CHECK(t + SETTLE >= 1999 && t + SETTLE <= 2002);         // 2.0 s from the M
  CHECK(moveKind == 0 && braked(0) && braked(1));
  run(300);
  CHECK(braked(0) && braked(1));                             // stays stopped, no watchdog games

  Serial.out.clear();
  send("R 90\n");                                            // 90 / 120 deg/s = 750 ms
  settle();
  CHECK(motorL() == pwmOf(1) && motorR() == pwmOf(-1));      // + = clockwise: left fwd, right back
  t = runUntil("DONE", 2000);
  CHECK(contains("DONE R 90.0\r\n") && t + SETTLE >= 749 && t + SETTLE <= 752);
  Serial.out.clear();
  send("r -45 1.00\n");                                      // speed 1 (what tars_motor sends) = D 1
  settle();
  CHECK(motorL() == pwmOf(-1) && motorR() == pwmOf(1) && moveDurMs == 375);
  runUntil("DONE", 2000);
  CHECK(contains("DONE R -45.0\r\n"));
  Serial.out.clear();
  send("M -150\n");
  settle();
  CHECK(motorL() == pwmOf(-1) && motorR() == pwmOf(-1));
  runUntil("DONE", 2000);
  CHECK(contains("DONE M -150.0\r\n") && braked(0));

  // zero-length, clamps, replace
  Serial.out.clear();
  send("M 0\n");
  CHECK(contains("DONE M 0.0\r\n") && moveKind == 0 && braked(0));
  send("M 5000\n");
  CHECK(moveReq == 1000.0f && moveDurMs == 6667);
  run(100);
  send("R -1000\n");
  CHECK(contains("ABORT M 15.0 replaced\r\n"));
  CHECK(moveKind == 'R' && moveReq == -360.0f && moveDurMs == 3000);
  send("S\n");
  CHECK(contains("ABORT R ") && contains(" stop\r\n") && braked(0) && braked(1));
  // malformed
  Serial.out.clear();
  const char *bad[] = {"M\n", "M 1 2 3\n", "M x\n", "R\n", "MX 10\n", "M 10 abc\n"};
  for (const char *b : bad) { send(b); CHECK(moveKind == 0); }
  CHECK(Serial.out.empty());
}

// The slow pivot: R with speed < 0.99 spins at DC_SLOW_TURN_DUTY, timed by deg_s_slow.
static void test_dc_slow_turn() {
  puts("dc: slow R (speed < 1): own duty, own rate, kick, stop wins");
  static const int SLOW = (int)(DC_SLOW_TURN_DUTY * 255.0f + 0.5f);
  CHECK(DC_SLOW_TURN_DUTY > DC_MIN_DUTY && DC_SLOW_TURN_DUTY <= DC_MAX_DUTY);   // turns, under the cap
  CHECK(DC_DEG_S_SLOW_DEFAULT < DC_DEG_S_DEFAULT);
  boot(false);
  char want[48];
  snprintf(want, sizeof want, "WHEELS DCSLOW %.1f\r\n", DC_DEG_S_SLOW_DEFAULT);
  CHECK(contains(want));
  // WHEELS DC stays exactly two numbers (the Pi rejects extra fields), DCSLOW follows, both before READY
  CHECK(contains("WHEELS DC 150.0 120.0\r\nWHEELS DCSLOW "));
  CHECK(Serial.out.find("WHEELS DCSLOW") < Serial.out.find("READY"));
  Serial.out.clear();
  send("R 90 0.40\n");
  uint32_t want_ms = (uint32_t)(90.0f / DC_DEG_S_SLOW_DEFAULT * 1000.0f + 0.5f);
  CHECK(moveKind == 'R' && moveDurMs == want_ms);
  CHECK(motorL() == (DC_KICK_MS ? KICK : SLOW) && motorR() == -(DC_KICK_MS ? KICK : SLOW));   // kick from rest
  settle();
  CHECK(motorL() == SLOW && motorR() == -SLOW);              // + = clockwise, at the slow duty
  CHECK(SLOW < pwmOf(1));                                    // slower than the D 1 turn
  long t = runUntil("DONE", 5000);
  CHECK(contains("DONE R 90.0\r\n") && t + SETTLE >= (long)want_ms - 1 && t + SETTLE <= (long)want_ms + 2);
  CHECK(braked(0) && braked(1));
  // left turn
  Serial.out.clear();
  send("R -30 0.4\n");
  settle();
  CHECK(motorL() == -SLOW && motorR() == SLOW);
  runUntil("DONE", 3000);
  CHECK(contains("DONE R -30.0\r\n"));
  // S mid slow turn: braked at once, reports by time
  Serial.out.clear();
  send("R 120 0.4\n");
  run(500);
  send("S\n");
  CHECK(braked(0) && braked(1) && moveKind == 0 && contains("ABORT R ") && contains(" stop\r\n"));
  float a = lastAmount("ABORT R ");
  CHECK(a > 0.5f * DC_DEG_S_SLOW_DEFAULT - 1 && a < 0.5f * DC_DEG_S_SLOW_DEFAULT + 1);
  // watchdog-style: a non-zero D takes over
  Serial.out.clear();
  send("R 90 0.4\n");
  run(100);
  send("D 0.5 0.5\n");
  CHECK(contains(" drive\r\n") && moveKind == 0);
  send("S\n");
  // M ignores the speed (always D 1); R at speed 1 / no speed stays fast
  send("M 150 0.4\n");
  CHECK(moveDurMs == 1000);
  send("R 90 1\n");
  CHECK(moveDurMs == 750);
  send("R 90\n");
  CHECK(moveDurMs == 750);
  send("S\n");
}

static void test_dc_aborts() {
  puts("dc: M/R aborts (S, D, link, tip) and keepalives");
  boot(false);
  Serial.out.clear();
  // S mid-move brakes AT ONCE (same loop) and reports how far it got by time
  send("M 300\n");
  run(500);
  CHECK(motorL() == pwmOf(1));
  send("S\n");
  CHECK(motorL() == 0 && motorR() == 0 && braked(0) && braked(1));
  CHECK(contains("ABORT M ") && contains(" stop\r\n") && moveKind == 0);
  float a = lastAmount("ABORT M ");
  CHECK(a > 74 && a < 76);                                   // 500 ms of 2000 ms
  run(2000);
  CHECK(braked(0) && !contains("DONE"));                     // the old timer never fires

  // S during the start kick
  Serial.out.clear();
  send("R 90\n");
  CHECK(motorL() == KICK || DC_KICK_MS == 0);
  send("S\n");
  CHECK(braked(0) && braked(1) && contains("ABORT R 0.0 stop"));
  run(DC_KICK_MS + 10);
  CHECK(braked(0) && braked(1));

  // typed on the bench (no D stream): no link abort; D 0 0 is a keepalive;
  // a non-zero D aborts and takes over
  boot(false);
  Serial.out.clear();
  send("R 90\n");
  run(600);
  CHECK(moveKind == 'R');
  send("D 0 0\n");
  run(50);
  CHECK(moveKind == 'R' && !contains("ABORT") && motorL() == pwmOf(1));
  send("D 0.5 0.5\n");
  CHECK(contains("ABORT R ") && contains(" drive\r\n") && moveKind == 0);
  settle();
  CHECK(motorL() == P50 && motorR() == P50 && driveActive);
  send("S\n");

  // Pi streaming D: the move runs to DONE while D 0 0 keeps coming, aborts when it stops
  boot(false);
  for (int i = 0; i < 5; i++) { send("D 0 0\n"); run(20); }
  Serial.out.clear();
  send("M 150\n");
  CHECK(moveLinkGuard);
  long t = 0;
  while (!contains("DONE") && t < 3000) { send("D 0.000 0.000\n"); run(20); t += 20; }
  CHECK(contains("DONE M 150.0") && t >= 980 && t <= 1020);
  Serial.out.clear();
  send("M 300\n");
  run(300);
  CHECK(moveKind == 'M');
  run(600);                                                  // Pi went quiet
  CHECK(contains(" link\r\n") && moveKind == 0 && braked(0) && braked(1));
  a = lastAmount("ABORT M ");
  CHECK(a > 70 && a < 80);                                   // aborted ~501 ms after the last line

  // tip mid-move
  boot(true);
  run(50);
  Serial.out.clear();
  send("M 300\n");
  run(500);
  tilt(60);
  run(400);
  CHECK(tipped && contains(" tipped\r\n") && moveKind == 0 && braked(0));
  CHECK(Serial.out.find("ABORT M") < Serial.out.find("T "));
  Serial.out.clear();
  send("M 10\n");
  CHECK(contains("ABORT M 0.0 tipped") && moveKind == 0 && braked(0));
}

static void test_dc_cal() {
  puts("dc: V speed calibration in EEPROM");
  boot(false);
  CHECK(dcMmS == DC_MM_S_DEFAULT && dcDegS == DC_DEG_S_DEFAULT);
  Serial.out.clear();
  send("V\n");
  CHECK(Serial.out.find("WHEELS DC 150.0 120.0\r\nWHEELS DCSLOW ") == 0);
  send("V 200 90\n");                                        // two values: slow rate kept
  CHECK(contains("WHEELS DC 200.0 90.0\r\n") && dcMmS == 200.0f && dcDegS == 90.0f);
  CHECK(dcDegSSlow == DC_DEG_S_SLOW_DEFAULT);
  Serial.out.clear();
  send("V 5 90\n"); send("V 200\n"); send("V 200 90 1\n"); send("V 200 3000\n"); send("V 200 90 40 5\n");
  CHECK(Serial.out.empty() && dcMmS == 200.0f && dcDegS == 90.0f);   // bad values ignored
  CHECK(dcDegSSlow == DC_DEG_S_SLOW_DEFAULT);
  send("V 200 90 45\n");
  CHECK(contains("WHEELS DC 200.0 90.0\r\nWHEELS DCSLOW 45.0\r\n") && dcDegSSlow == 45.0f);
  send("V 200 90\n");
  CHECK(dcDegSSlow == 45.0f);
  boot(false, -1, true);
  CHECK(dcDegSSlow == 45.0f && contains("WHEELS DCSLOW 45.0"));
  send("R 90 0.4\n");
  CHECK(moveDurMs == 2000);
  send("S\n");
  Serial.out.clear();
  send("K 70 120\n"); send("K\n");
  CHECK(Serial.out.empty());                                 // K is stepper-only
  boot(false, -1, true);                                     // reboot keeps EEPROM
  CHECK(dcMmS == 200.0f && dcDegS == 90.0f && contains("WHEELS DC 200.0 90.0"));
  Serial.out.clear();
  send("M 300\n");
  CHECK(moveDurMs == 1500);
  send("R 90\n");
  CHECK(moveDurMs == 1000);
  send("S\n");
  boot(false);                                               // erased EEPROM -> defaults
  CHECK(dcMmS == 150.0f && dcDegS == 120.0f);
  // a stepper build's K record in EEPROM (other magic) is not read as DC speeds
  struct { uint16_t magic; float d, t; } k = {0x7A12, 300.0f, 400.0f};
  EEPROM.put(0, k);
  boot(false, -1, true);
  CHECK(dcMmS == 150.0f && dcDegS == 120.0f);
  // a v1 record (before the slow rate existed) still loads; slow rate = default
  struct { uint16_t magic; float mm, deg; } v1 = {0x7DC1, 180.0f, 100.0f};
  boot(false);
  EEPROM.put(0, v1);
  boot(false, -1, true);
  CHECK(dcMmS == 180.0f && dcDegS == 100.0f && dcDegSSlow == DC_DEG_S_SLOW_DEFAULT);
}
#endif

#if CAR == CAR_SM11
static void test_sm11_pins() {
  puts("sm11: kit pin map");
  // github.com/mbof/sm11 sm11_pinout.h
  CHECK(PIN_L_EN == 10 && PIN_L_IN1 == 8 && PIN_L_IN2 == 9);
  CHECK(PIN_R_EN == 5 && PIN_R_IN1 == 6 && PIN_R_IN2 == 7);
  CHECK(PIN_BUZZER == 12 && PIN_BUTTON == 13 && PIN_LED == 3);
  CHECK(!BUZZER_PASSIVE);                                    // no tone() -> Timer2 free
  CHECK(ARM_COUNT == 0 && DRIVER_TYPE == DRIVER_EN_IN && WHEEL_TYPE == WHEELS_DC);
  // EN pins must be hardware PWM: D5/D6 Timer0, D9/D10 Timer1, D3/D11 Timer2
  for (int p : {(int)PIN_L_EN, (int)PIN_R_EN}) CHECK(p == 5 || p == 6 || p == 9 || p == 10 || p == 3 || p == 11);
  // the cap is a real cap and the minimum sits under it
  CHECK(DC_MAX_DUTY < 1.0f && DC_MIN_DUTY > 0.0f && DC_MIN_DUTY < DC_MAX_DUTY && DC_KICK_DUTY > DC_MAX_DUTY);
  boot(false);
  CHECK(mock_pin_mode[13] == INPUT_PULLUP && mock_pin_mode[3] == OUTPUT && mock_pin_mode[12] == OUTPUT);
  CHECK(braked(0) && braked(1));                             // boots braked
  // untouched kit pins: IR D2, right LED D4, ultrasonic/line A0-A3
  for (int p : {2, 4, 14, 15, 16, 17}) CHECK(mock_pin_mode[p] == -1);
}
#endif

#if ARM_COUNT == 1
static void test_one_arm() {
  puts("one arm (A0-A3)");
  boot(false);
  send("A 0 30 60\n");
  send("A 1 30 60\n");                                         // no arm 1: ignored
  run(700);
  CHECK(arms[0].pos == degSteps(30) && arms[1].target == 0);
  send("M 20\n");
  runUntil("DONE", 2000);
  CHECK(contains("DONE M 20.0"));
}
#endif

// ------------------------------------------------------------------ button / LED / buzzer
static void test_io() {
  puts("button / LED / buzzer");
  boot(false);
  if (PIN_BUTTON != NO_PIN) {
    Serial.out.clear();
    // short press with contact bounce
    for (int i = 0; i < 6; i++) { mock_pin_input[PIN_BUTTON] = (i & 1) ? HIGH : LOW; run(3); }
    mock_pin_input[PIN_BUTTON] = LOW;
    run(300);
    CHECK(!contains("BTN"));                                   // nothing until release
    mock_pin_input[PIN_BUTTON] = HIGH;
    run(10);
    CHECK(!contains("BTN"));                                   // debounce
    run(40);
    CHECK(countOf(Serial.out, "BTN TEACH\r\n") == 1);
    // a 10 ms glitch is not a press
    Serial.out.clear();
    mock_pin_input[PIN_BUTTON] = LOW; run(10); mock_pin_input[PIN_BUTTON] = HIGH; run(100);
    CHECK(Serial.out.empty());
    // long press: FORGET at 2 s while held, and no TEACH on release
    mock_pin_input[PIN_BUTTON] = LOW;
    run(1900);
    CHECK(!contains("BTN"));
    run(200);
    CHECK(countOf(Serial.out, "BTN FORGET\r\n") == 1);
    run(1000);
    mock_pin_input[PIN_BUTTON] = HIGH;
    run(100);
    CHECK(countOf(Serial.out, "BTN") == 1);
    // button works while the wheels are busy
    Serial.out.clear();
#if WHEEL_TYPE == WHEELS_STEPPER
    send("M 50\n");
#endif
    mock_pin_input[PIN_BUTTON] = LOW; run(100); mock_pin_input[PIN_BUTTON] = HIGH; run(100);
    CHECK(contains("BTN TEACH"));
    send("S\n");
  }
  if (PIN_LED != NO_PIN) {
    send("L 1\n"); run(5);
    CHECK(mock_pin_digital[PIN_LED] == HIGH);
    send("L 0\n"); run(5);
    CHECK(mock_pin_digital[PIN_LED] == LOW);
    send("L 2\n");
    int changes = 0, last = -1;
    for (int i = 0; i < 2000; i++) { run(1); if (mock_pin_digital[PIN_LED] != last) { changes++; last = mock_pin_digital[PIN_LED]; } }
    CHECK(changes >= 4 && changes <= 5);                      // 500 ms half-period
    send("L 3\n");
    changes = 0;
    for (int i = 0; i < 2000; i++) { run(1); if (mock_pin_digital[PIN_LED] != last) { changes++; last = mock_pin_digital[PIN_LED]; } }
    CHECK(changes >= 19 && changes <= 21);                    // 100 ms half-period
    send("L 4\n"); send("L 1.5\n"); send("L -1\n"); send("L\n");
    CHECK(ledPattern == 3);                                    // bad patterns ignored
  }
  if (PIN_BUZZER != NO_PIN) {
    send("B 3\n");
    int rises = 0, last = mock_pin_digital[PIN_BUZZER], onMs = 0;
    for (int i = 0; i < 1500; i++) {
      run(1);
      int v = mock_pin_digital[PIN_BUZZER];
      if (v == HIGH) onMs++;
      if (v == HIGH && last != HIGH) rises++;
      last = v;
    }
    CHECK(rises == 2 && onMs >= 230 && onMs <= 245);          // 1st beep starts at once: 3 x 80 ms
    CHECK(mock_pin_digital[PIN_BUZZER] == LOW && beepsLeft == 0);
    send("B 5\n"); run(100);
    send("B 0\n"); run(5);                                     // silence
    CHECK(mock_pin_digital[PIN_BUZZER] == LOW && beepsLeft == 0);
    send("B 99\n");
    CHECK(beepsLeft == 9);                                     // capped
    send("B 0\n");
    send("B x\n"); send("B 2 3\n");
    CHECK(beepsLeft == 0);
  }
  // L / B never move anything
  CHECK(motorL() == 0);
}

int main() {
  test_drive();
  test_malformed();
  test_watchdog();
  test_arms();
  test_overflow();
  test_sensor_detect();
  test_io();
#if WHEEL_TYPE == WHEELS_STEPPER
  test_gear_ratio();
  test_wheel_velocity();
  test_wheel_moves();
  test_wheel_cal();
  g_sensor = 0;
  test_wheel_aborts();
#else
  test_dc_speed();
  test_dc_moves();
  test_dc_slow_turn();
  test_dc_cal();
  g_sensor = 0;
  test_dc_aborts();
#endif
#if CAR == CAR_SM11
  test_sm11_pins();
#endif
#if ARM_COUNT == 1
  test_one_arm();
#endif
  for (g_sensor = 0; g_sensor < 2; g_sensor++) {
    printf("== accelerometer: %s ==\n", sensorName());
    test_boot();
    test_status();
    test_tip();
    test_bump();
    test_no_mpu();
  }
  printf("%d/%d checks passed  [car=%s wheels=%s arms=%d driver=%d]\n", checks - failures, checks,
         CAR == CAR_SM11 ? "sm11" : "custom", WHEEL_TYPE == WHEELS_STEPPER ? "stepper" : "dc", ARM_COUNT, DRIVER_TYPE);
  return failures ? 1 : 0;
}
