// Host unit tests for tars.ino (no Arduino needed).
//   make -C uno/test
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

#include "../tars/tars.ino"

#include <cmath>
#include <cstdlib>

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

static void boot(bool withMpu, int addrOverride = -1) {
  mock_now_us = 1000000;
  for (int i = 0; i < 32; i++) { mock_pin_mode[i] = -1; mock_pin_digital[i] = -1; mock_pin_pwm[i] = -1; }
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
#if DRIVER_TYPE == DRIVER_2PWM
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
  CHECK(motorL() == 0 && motorR() == 0 && coils(0) == 0 && coils(1) == 0);
  for (int k = 0; k < 4; k++) CHECK(mock_pin_mode[PIN_ARM[1][k]] == OUTPUT);

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

  // Pins: arm 1 must not clash with the drive or I2C/serial pins in this mode.
  int used[] = {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2, 0, 1, 18 /*A4*/, 19 /*A5*/,
#if DRIVER_TYPE == DRIVER_EN_IN
                PIN_L_EN, PIN_R_EN,
#endif
  };
  for (int a = 0; a < 2; a++)
    for (int k = 0; k < 4; k++)
      for (int u : used) CHECK(PIN_ARM[a][k] != u);
}

static void test_drive() {
  puts("drive");
  boot(false);
  send("D 0.5 -0.5\n");
  CHECK(motorL() == 128 && motorR() == -128);
  send("D 2 -3\n");
  CHECK(motorL() == 255 && motorR() == -255);
  send("D 0.01 -0.02\n");
  CHECK(motorL() == 0 && motorR() == 0);
  CHECK(!driveActive);
  send("d 0.3 0.3\r");
  CHECK(motorL() == 77 && motorR() == 77);
  send("D -1 1\r\n");
  CHECK(motorL() == -255 && motorR() == 255);
}

static void test_malformed() {
  puts("malformed");
  boot(false);
  send("D 0.4 0.4\n");
  int l = motorL();
  const char *bad[] = {"D 0.9\n", "D 0.9 0.9 0.9 0.9\n", "DX 1 1\n", "D a b\n", "D 0.9 0.9x\n",
                       "D nan 0.9\n", "D 0.900000000000000000000000000000000000000000 1\n",
                       "\x01D 0.9 0.9\n", "Q 1\n", "S 1\n", "A 2 10\n", "A 0\n", "A -1 30\n"};
  for (const char *b : bad) { send(b); CHECK(motorL() == l); }
  CHECK(arms[0].target == 0 && arms[1].target == 0);
  // a split line still works
  send("D 0."); send("2 0.2\n");
  CHECK(motorL() == 51);
}

static void test_watchdog() {
  puts("watchdog");
  boot(false);
  send("D 0.5 0.5\n");
  run(150);
  CHECK(motorL() == 128);
  run(100);
  CHECK(motorL() == 0 && motorR() == 0);
  // fed every 50 ms -> keeps running
  for (int i = 0; i < 20; i++) { send("D 0.5 0.5\n"); run(50); }
  CHECK(motorL() == 128);
  // junk doesn't feed it
  for (int i = 0; i < 5; i++) { send("D junk\n"); run(50); }
  CHECK(motorL() == 0);
}

static void test_arms() {
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
  CHECK(!tipped && motorL() == 128);
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
  send("D 0.5 0.5\n");
  CHECK(motorL() == 128);

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
  CHECK(motorL() == 128);
  send("A 0 10 60\n");
  run(300);
  CHECK(arms[0].pos == degSteps(10));
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
  send("D 0.5 0.5\n");
  CHECK(motorL() == 128);
}

int main() {
  test_drive();
  test_malformed();
  test_watchdog();
  test_arms();
  test_overflow();
  test_sensor_detect();
  for (g_sensor = 0; g_sensor < 2; g_sensor++) {
    printf("== accelerometer: %s ==\n", sensorName());
    test_boot();
    test_status();
    test_tip();
    test_bump();
    test_no_mpu();
  }
  printf("%d/%d checks passed\n", checks - failures, checks);
  return failures ? 1 : 0;
}
