// Host unit tests for balancer.ino: filter, PID, cutoff/re-arm, commands, X mode.
//   make -C balancer/uno/test
// The sketch is compiled as-is against mock/Arduino.h, mock/Wire.h, mock/EEPROM.h.
#include "Arduino.h"
#include "Wire.h"
#include "EEPROM.h"

uint64_t mock_now_us = 0;
int mock_pin_mode[32];
int mock_pin_digital[32];
int mock_pin_pwm[32];
MockSerial Serial;
MockWire Wire;
MockMpu mock_mpu;
MockEEPROM EEPROM;

#include "../balancer/balancer.ino"

#include <cmath>
#include <cstdlib>

static int failures = 0, checks = 0;
#define CHECK(cond)                                                                       \
  do {                                                                                    \
    checks++;                                                                             \
    if (!(cond)) { failures++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((double)(a) - (double)(b)) <= (tol))

// Default mounting: X up, Z forward, pitch rate = -gyro Y.
static void setTilt(float deg, float rateDps = 0, float gyroBiasDps = 0) {
  float r = deg * 3.14159265f / 180.0f;
  mock_mpu.setAccel((int16_t)(std::cos(r) * 8192), 0, (int16_t)(-std::sin(r) * 8192));
  mock_mpu.setGyro(0, (int16_t)(-(rateDps + gyroBiasDps) * 65.5f), 0);
}

static void resetSketch() {
  lineLen = 0; lineOverflow = false;
  kp = DEF_KP; ki = DEF_KI; kd = DEF_KD; ks = DEF_KS;
  uprightOffset = 0; gyroBias = 0; mpuAddr = 0;
  extMode = false; extPitch = extRate = 0; lastPMs = 0; extEverSeen = false;
  pitchRaw = accPitch = pitch = rate = 0; filterSeeded = false;
  state = ST_DISARMED; armed = false; uprightSinceMs = 0; uprightTiming = false;
  integ = speedEst = u = 0; turnCmd = leanCmd = 0; lastCmdMs = 0;
  lastLoopUs = 0; lastTelemMs = 0;
}

static void boot(bool withMpu, bool clearEeprom = true, float tilt = 0, float bias = 0) {
  mock_now_us = 1000000;
  for (int i = 0; i < 32; i++) { mock_pin_mode[i] = -1; mock_pin_digital[i] = -1; mock_pin_pwm[i] = -1; }
  Serial.in.clear(); Serial.out.clear();
  mock_mpu = MockMpu();
  mock_mpu.present = withMpu;
  setTilt(tilt, 0, bias);
  if (clearEeprom) EEPROM.clear();
  resetSketch();
  setup();
}

static void run(uint32_t ms) {
  uint64_t end = mock_now_us + (uint64_t)ms * 1000;
  while (mock_now_us < end) { loop(); mock_now_us += 100; }
}
static void send(const char *s) { Serial.feed(s); loop(); }

static int level(int p) { return mock_pin_pwm[p] >= 0 ? mock_pin_pwm[p] : (mock_pin_digital[p] == HIGH ? 255 : 0); }
static int motorL() { return level(PIN_L_IN1) - level(PIN_L_IN2); }
static int motorR() { return level(PIN_R_IN1) - level(PIN_R_IN2); }
static bool has(const char *s) { return Serial.out.find(s) != std::string::npos; }

// Hold upright long enough to start balancing.
static void standUp() { setTilt(0); run(1200); }

static void test_boot() {
  puts("boot");
  boot(true);
  CHECK(has("# mpu6050 at 0x68"));
  CHECK(has("READY\r\n"));
  CHECK(motorL() == 0 && motorR() == 0);
  CHECK(state == ST_WAITING);  // AUTO_ARM
  CHECK(mock_mpu.regs[0x1B] == 0x08 && mock_mpu.regs[0x1C] == 0x08 && mock_mpu.regs[0x6B] == 0x01);

  boot(false);
  CHECK(has("NOIMU"));
  CHECK(has("READY"));
  run(2000);
  CHECK(state != ST_BALANCING);
  CHECK(motorL() == 0 && motorR() == 0);

  // MPU on the alternate address
  mock_now_us = 1000000; Serial.out.clear(); resetSketch();
  mock_mpu = MockMpu(); mock_mpu.present = true; mock_mpu.addr = 0x69; setTilt(0);
  setup();
  CHECK(has("# mpu6050 at 0x69"));
}

static void test_gyro_bias() {
  puts("gyro bias calibrated at boot");
  boot(true, true, 0, 3.0f);   // 3 deg/s bias, robot still
  NEAR(gyroBias, 3.0, 0.05);
  run(200);
  NEAR(rate, 0.0, 0.05);
  NEAR(pitch, 0.0, 0.1);
}

static void test_filter() {
  puts("complementary filter");
  boot(true);
  send("S\n");
  // Static tilt: converges to accel angle.
  setTilt(10);
  run(3000);
  NEAR(pitch, 10.0, 0.2);
  NEAR(accPitch, 10.0, 0.1);
  // Accel disturbance (wheels lurch) with zero rate: filtered pitch barely moves in 50 ms.
  setTilt(0); run(3000);
  NEAR(pitch, 0.0, 0.2);
  mock_mpu.setAccel(8192, 0, (int16_t)(-0.5f * 8192));  // 0.5 g fake forward reading
  mock_mpu.setGyro(0, 0, 0);
  run(50);
  CHECK(std::fabs(accPitch) > 20);
  CHECK(std::fabs(pitch) < 6.0f && std::fabs(pitch) < 0.25f * std::fabs(accPitch));
  // Gyro rate integrates: 20 deg/s for 100 ms with accel at 0 -> ~+2 deg (minus a bit of pull-back).
  setTilt(0); run(3000);
  mock_mpu.setGyro(0, (int16_t)(-20 * 65.5f), 0);
  run(100);
  CHECK(pitch > 1.2f && pitch < 2.1f);
  NEAR(rate, 20.0, 0.1);
}

static void test_arm_and_pid() {
  puts("arm / upright hold / PID sign");
  boot(true);
  setTilt(0);
  run(500);
  CHECK(state == ST_WAITING);
  CHECK(!has("UP"));
  run(700);
  CHECK(state == ST_BALANCING);
  CHECK(has("UP\r\n"));
  // Leaning forward -> both wheels forward.
  setTilt(2); run(100);
  CHECK(motorL() > 0 && motorR() > 0);
  CHECK(motorL() == motorR());
  // Leaning back -> backward.
  setTilt(-2); run(300);
  CHECK(motorL() < 0 && motorR() < 0);
  // Upright & still -> output ~0, inside deadband -> motors off.
  setTilt(0); run(2000);
  CHECK(std::fabs(u) < DEADBAND);
  CHECK(motorL() == 0);
  // D term: rate alone gives kd*rate.
  send("G 0 0 0.01\n");
  mock_mpu.setGyro(0, (int16_t)(-10 * 65.5f), 0);
  loop(); mock_now_us += 5000; loop();
  NEAR(u, 0.01 * rate + 0 * pitch, 1e-4);
  CHECK(u > 0.09f);
}

static void test_min_duty() {
  puts("deadband / min duty kick");
  CHECK(shape(0.001f) == 0.0f);
  NEAR(shape(0.01f), MIN_DUTY + (1 - MIN_DUTY) * 0.01f, 1e-6);
  NEAR(shape(-0.5f), -(MIN_DUTY + (1 - MIN_DUTY) * 0.5f), 1e-6);
  NEAR(shape(2.0f), 1.0f, 1e-6);
  boot(true);
  standUp();
  send("G 0.01 0 0\n");
  setTilt(1.0f); run(3000);   // u = 0.01 -> pwm (0.1 + 0.9*0.01)*255 ~ 28
  CHECK(motorL() >= 26 && motorL() <= 30);
}

static void test_integral() {
  puts("integral + clamp");
  boot(true);
  standUp();
  send("G 0 1 0\n");
  setTilt(2); run(1000);           // e = 2 for ~1 s -> integ ~ 2
  CHECK(u > 1.5f * 0.25f && u <= I_LIMIT + 1e-4);
  run(5000);
  NEAR(u, I_LIMIT, 1e-3);           // clamped
  send("G 0 1 0\n");               // gain change resets integral
  CHECK(std::fabs(integ) < 0.02f);
}

static void test_cutoff_and_rearm() {
  puts("tip-over cutoff + re-arm after 1 s upright");
  boot(true);
  standUp();
  CHECK(state == ST_BALANCING);
  setTilt(40); run(1500);
  CHECK(state == ST_FALLEN);
  CHECK(has("FALL "));
  CHECK(motorL() == 0 && motorR() == 0);
  // Briefly upright then knocked again: no restart.
  setTilt(0); run(600);
  setTilt(20); run(100);
  CHECK(state == ST_FALLEN);
  CHECK(motorL() == 0);
  // Held upright 1 s: restarts.
  Serial.out.clear();
  setTilt(0); run(1300);
  CHECK(state == ST_BALANCING);
  CHECK(has("UP"));
}

static void test_stop_arm() {
  puts("S disarms, A arms");
  boot(true);
  standUp();
  setTilt(3); run(50);
  CHECK(motorL() != 0);
  send("S\n");
  CHECK(state == ST_DISARMED);
  CHECK(motorL() == 0 && motorR() == 0);
  setTilt(0); run(3000);
  CHECK(state == ST_DISARMED);       // stays off upright
  send("a\n");                       // lowercase works
  CHECK(state == ST_WAITING);
  run(1200);
  CHECK(state == ST_BALANCING);
}

static void test_turn_lean_watchdog() {
  puts("T / V + watchdog");
  boot(true);
  standUp();
  send("G 0 0 0\n");
  send("T 1\n");
  run(20);
  CHECK(motorL() > 0 && motorR() < 0);   // + = turn right: left fwd, right back
  NEAR(turnCmd, 1.0, 1e-6);
  send("T 5\n");
  NEAR(turnCmd, 1.0, 1e-6);               // clamped
  send("V 9\n");
  NEAR(leanCmd, MAX_LEAN_BIAS, 1e-6);
  send("V -2\n");
  NEAR(leanCmd, -2, 1e-6);
  // keep alive
  for (int i = 0; i < 10; i++) { send("T 0.5\n"); run(100); }
  NEAR(turnCmd, 0.5, 1e-6);
  NEAR(leanCmd, -2, 1e-6);
  CHECK(state == ST_BALANCING);
  // silence -> steering and lean drop to 0, balancing continues
  run(400);
  CHECK(turnCmd == 0 && leanCmd == 0);
  CHECK(state == ST_BALANCING);
  // lean bias shifts the target: pitch 0 with V -2 -> e = +2 -> forward
  send("G 0.05 0 0\n");
  send("V -2\n");
  run(20);
  NEAR(u, 0.1, 0.01);
}

static void test_speed_loop() {
  puts("ks speed loop pulls target back");
  boot(true);
  standUp();
  send("G 0.05 0 0 2\n");
  NEAR(ks, 2, 1e-6);
  setTilt(4); run(2000);                // sustained forward output -> speedEst > 0
  CHECK(speedEst > 0.1f);
  CHECK(u > 0.05f * 4);                 // target moved back -> larger error
}

static void test_parsing() {
  puts("parsing robustness");
  boot(true);
  send("T\n"); CHECK(turnCmd == 0);
  send("T abc\n"); CHECK(turnCmd == 0);
  send("TX 1\n"); CHECK(turnCmd == 0);
  send("T 0.3 0.4\n"); CHECK(turnCmd == 0);
  send("T 0.3x\n"); CHECK(turnCmd == 0);
  send("G 1 2\n"); NEAR(kp, DEF_KP, 1e-9);
  send("G 1 2 3 4 5\n"); NEAR(kp, DEF_KP, 1e-9);
  send("\x01\x02T 0.5\n"); CHECK(turnCmd == 0);          // binary garbage drops the line
  send("T 0.000000000000000000000000000000000000001\n"); // long line dropped
  send("  t   0.25 \r"); NEAR(turnCmd, 0.25, 1e-6);
  send("S extra\n"); CHECK(state != ST_DISARMED);
  Serial.out.clear();
  send("?\n"); CHECK(has("# kp "));
}

static void test_eeprom() {
  puts("Z offset + W save + reload");
  boot(true);
  send("S\n");
  setTilt(4); run(3000);
  send("Z\n");
  NEAR(uprightOffset, 4.0, 0.1);
  run(500);
  NEAR(pitch, 0.0, 0.2);
  send("G 0.07 0.01 0.004 1.5\n");
  send("W\n");
  CHECK(has("# saved"));
  boot(true, false, 4);
  NEAR(kp, 0.07, 1e-6); NEAR(ki, 0.01, 1e-6); NEAR(kd, 0.004, 1e-6); NEAR(ks, 1.5, 1e-6);
  NEAR(uprightOffset, 4.0, 0.1);
  run(1200);
  CHECK(state == ST_BALANCING);    // 4 deg raw is "upright" now
  boot(true, true);
  NEAR(kp, DEF_KP, 1e-9);          // cleared EEPROM -> defaults
}

static void test_ext_mode() {
  puts("X mode: external P lines");
  boot(false);                       // no IMU at all
  send("X 1\n");
  CHECK(extMode);
  CHECK(has("# ext 1"));
  run(2000);
  CHECK(state == ST_WAITING);        // no P yet
  for (int i = 0; i < 120; i++) { send("P 0.5 0\n"); run(10); }
  CHECK(state == ST_BALANCING);
  send("G 0.05 0 0.001\n");
  send("P 2 10\n");
  run(6);
  NEAR(pitch, 2.0, 1e-4);
  NEAR(u, 0.05 * 2 + 0.001 * 10, 1e-4);
  CHECK(motorL() > 0);
  // P stops -> STALE, motors off within ~100 ms
  run(150);
  CHECK(has("STALE"));
  CHECK(state == ST_FALLEN);
  CHECK(motorL() == 0 && motorR() == 0);
  // External cutoff
  for (int i = 0; i < 120; i++) { send("P 0 0\n"); run(10); }
  CHECK(state == ST_BALANCING);
  send("P 50 0\n"); run(10);
  CHECK(state == ST_FALLEN);
  // Z in X mode takes the external pitch as upright
  send("P 3 0\n");
  send("Z\n");
  NEAR(uprightOffset, 3.0, 1e-4);
  // Back to onboard
  send("X 0\n");
  CHECK(!extMode);
}

static void test_imu_loss() {
  puts("IMU lost mid-balance -> stop");
  boot(true);
  standUp();
  CHECK(state == ST_BALANCING);
  mock_mpu.present = false;
  run(20);
  CHECK(state == ST_FALLEN);
  CHECK(motorL() == 0);
  CHECK(has("NOIMU"));
}

static void test_telemetry() {
  puts("telemetry at ~20 Hz");
  boot(true);
  Serial.out.clear();
  run(1000);
  int n = 0; size_t pos = 0;
  while ((pos = Serial.out.find("# t ", pos)) != std::string::npos) { n++; pos++; }
  CHECK(n >= 19 && n <= 21);
  CHECK(has("# t W "));
}

static void test_loop_rate() {
  puts("control loop at 200 Hz");
  boot(true);
  standUp();
  // count controlStep calls by watching speedEst evolve? simpler: run 1 s and count telemetry already done;
  // here check the scheduler keeps up with a 100 us tick
  uint32_t before = lastLoopUs;
  run(1000);
  uint32_t diff = lastLoopUs - before;
  CHECK(diff >= 995000 && diff <= 1000000);
}

int main() {
  test_boot();
  test_gyro_bias();
  test_filter();
  test_arm_and_pid();
  test_min_duty();
  test_integral();
  test_cutoff_and_rearm();
  test_stop_arm();
  test_turn_lean_watchdog();
  test_speed_loop();
  test_parsing();
  test_eeprom();
  test_ext_mode();
  test_imu_loss();
  test_telemetry();
  test_loop_rate();
  printf("%d/%d checks passed\n", checks - failures, checks);
  return failures ? 1 : 0;
}
