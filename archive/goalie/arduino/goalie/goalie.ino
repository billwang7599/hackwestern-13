/*
 * Beat the Goalie: Arduino Uno motor/sensor co-processor.
 *
 *   Pi 5 (QNX goalie_motor) --USB serial 115200 8N1--> this Uno
 *     -> H-bridge ch A -> DC motor #1 (goalie paddle)
 *     -> H-bridge ch B -> DC motor #2 (celebration flag)
 *     -> ULN2003       -> 28BYJ-48 stepper (scoreboard needle)
 *     <- MPU6050 (I2C) -> hit / save detection
 *
 * Protocol (binding contract: goalie/docs/architecture.md, "Link 2"):
 *   Pi -> Uno (ASCII lines, '\n' terminated; '\r' also accepted; no replies):
 *     M <duty>   goalie motor, float in [-1, 1], + moves paddle towards +x
 *     S          stop all motors
 *     N <steps>  needle to absolute step position (non-blocking)
 *     C <ms>     spin the flag motor for ms, then stop (non-blocking)
 *     Z          zero the needle at its current position
 *   Uno -> Pi:
 *     READY          once after boot
 *     HIT <mag_mg>   accel spike above threshold, peak deviation in milli-g,
 *                    at most one per 250 ms
 *     A <mg>         (DEBUG only, <= 10 Hz) accel magnitude
 *     # ...          comment/diagnostic line. The Pi ignores lines starting
 *                    with '#'.
 *
 * Safety: no M for 200 ms -> goalie motor stops (Uno-side watchdog, on top of
 * the Pi's own watchdog). Nothing in loop() blocks: serial is drained a byte at
 * a time, the stepper and flag are time-scheduled, I2C has a timeout.
 *
 * Bench test: open the Serial Monitor at 115200 with line ending "Newline"
 * and type e.g.  M 0.5   (it will stop after 200 ms, that's the watchdog;
 * use goalie/arduino/serial_test.py for a sustained sweep).
 */

#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <math.h>

// =============================================================================
// CONFIG: edit this block on the bench. Everything else should not need changes.
// =============================================================================

// ---- Motor driver type -------------------------------------------------------
// How to tell which board you have (read the chip markings with a phone camera):
//   DRIVER_2PWM  : L9110S / HG7881 (two tiny 8-pin chips, header "B-IA B-IB GND
//                  VCC A-IA A-IB"), DRV8833 (pins AIN1 AIN2 BIN1 BIN2, often a
//                  "SLP/nSLEEP" pin: tie it HIGH), MX1508 (tiny red/blue board,
//                  IN1..IN4, no enable). Rule: 4 control inputs, NO enable pins.
//   DRIVER_EN_IN : L298N (big chip with heatsink, jumpers on ENA/ENB, IN1..IN4),
//                  TB6612FNG (PWMA AIN1 AIN2 / PWMB BIN1 BIN2 / STBY: tie STBY
//                  HIGH). Rule: an enable/PWM pin + 2 direction pins per channel.
//                  On an L298N, REMOVE the ENA/ENB jumpers so the Uno can PWM them.
#define DRIVER_2PWM  1
#define DRIVER_EN_IN 2
#define DRIVER_TYPE  DRIVER_2PWM

// ---- Pins ---------------------------------------------------------------------
// PWM pin choice: Uno pins 5/6 run on Timer0, which also drives millis()/micros();
// we keep them free. Channel A uses Timer1 (pins 9, 10), channel B uses Timer2
// (pins 3, 11). Nothing else here uses Timer1/Timer2 (no Servo, no tone()).
// Serial uses 0/1, I2C uses A4 (SDA) / A5 (SCL), the stepper uses A0-A3.
#if DRIVER_TYPE == DRIVER_2PWM
// Two PWM inputs per channel. Forward: IN1=PWM, IN2=LOW. Reverse: swapped.
const uint8_t PIN_A_IN1 = 9;    // L9110S A-IA / DRV8833 AIN1 / MX1508 IN1
const uint8_t PIN_A_IN2 = 10;   // L9110S A-IB / DRV8833 AIN2 / MX1508 IN2
const uint8_t PIN_B_IN1 = 3;    // L9110S B-IA / DRV8833 BIN1 / MX1508 IN3
const uint8_t PIN_B_IN2 = 11;   // L9110S B-IB / DRV8833 BIN2 / MX1508 IN4
#elif DRIVER_TYPE == DRIVER_EN_IN
// One PWM enable + two direction pins per channel.
const uint8_t PIN_A_EN  = 9;    // L298N ENA / TB6612 PWMA
const uint8_t PIN_A_IN1 = 7;    // L298N IN1 / TB6612 AIN1
const uint8_t PIN_A_IN2 = 8;    // L298N IN2 / TB6612 AIN2
const uint8_t PIN_B_EN  = 3;    // L298N ENB / TB6612 PWMB
const uint8_t PIN_B_IN1 = 4;    // L298N IN3 / TB6612 BIN1
const uint8_t PIN_B_IN2 = 12;   // L298N IN4 / TB6612 BIN2
#else
#error "DRIVER_TYPE must be DRIVER_2PWM or DRIVER_EN_IN"
#endif

// ULN2003 IN1..IN4 (A0..A3 used as digital pins).
const uint8_t PIN_STEP[4] = {A0, A1, A2, A3};

// ---- Motor tuning ---------------------------------------------------------------
// INVERT: flip if "M 0.5" moves the paddle towards -x (or swap the motor wires).
const bool  MOTOR_A_INVERT   = false;
const bool  MOTOR_B_INVERT   = false;
// |duty| below DEADBAND -> motor off (kills hum/creep from tiny commands).
const float MOTOR_A_DEADBAND = 0.02f;
// MIN_DUTY: output duty that just overcomes static friction. Commands above the
// deadband are mapped |d| -> MIN_DUTY + (1 - MIN_DUTY) * |d| so small commands
// still move.
// Left at 0 by default: if the Pi controller already does deadband
// compensation, doing it here too would double it. Measure on the bench
// (slowly raise "M x" until the paddle starts moving) and set it here OR on the
// Pi, not both.
const float MOTOR_A_MIN_DUTY = 0.0f;
// Brake (both inputs high / EN high + IN1=IN2) on stop instead of coasting.
// Braking stops the paddle faster; coasting is gentler on the gearbox.
const bool  MOTOR_A_BRAKE_ON_STOP = true;

const float    CELEBRATE_DUTY      = 0.8f;
#define CELEBRATE_WIGGLE_MS 0                // >0: flip flag direction every N ms
const uint16_t CELEBRATE_MAX_MS    = 10000;  // clamp for C <ms>

const uint16_t MOTOR_WATCHDOG_MS = 200;      // contract: no M for 200 ms -> stop

// ---- Stepper (28BYJ-48 + ULN2003) -----------------------------------------------
// Half-step mode: ~4096 half-steps per output revolution (4076 measured on many).
const uint16_t STEP_INTERVAL_US = 1400;      // ~714 half-steps/s (spec: 600-800)
const uint16_t STEP_RELEASE_MS  = 20;        // de-energize coils this long after arriving
const bool     STEP_INVERT      = false;

// ---- Accelerometer (MPU6050) ------------------------------------------------------
// ADXL345 swap note: different chip, different map. I2C address 0x53 (SDO low)
// or 0x1D, DEVID reg 0x00 = 0xE5, POWER_CTL 0x2D = 0x08 to measure,
// DATA_FORMAT 0x31 = 0x0A (+-8g, full-res, 3.9 mg/LSB), data at 0x32..0x37
// LITTLE-endian (X0,X1,...). Change accelInit()/accelRead() and ACCEL_LSB_PER_G
// (256 in full-res mode).
const uint8_t  MPU_ADDR_0 = 0x68;            // AD0 low (default on GY-521)
const uint8_t  MPU_ADDR_1 = 0x69;            // AD0 high
const float    ACCEL_LSB_PER_G   = 4096.0f;  // +-8g range
const uint16_t ACCEL_PERIOD_US   = 2000;     // 500 Hz sampling
const uint16_t HIT_THRESHOLD_MG  = 600;      // tune with DEBUG on: watch "# peak"
const uint16_t HIT_WINDOW_MS     = 15;       // collect the peak over this window
const uint16_t HIT_RATE_LIMIT_MS = 250;      // contract: <= 1 HIT per 250 ms
const float    BASELINE_ALPHA    = 0.002f;   // slow gravity-baseline EMA (~1 s at 500 Hz)

// ---- Debug ----------------------------------------------------------------------
// 1: send "A <mg>" and "# peak <mg>" at 10 Hz for threshold tuning.
#define DEBUG 0
const uint16_t DEBUG_PERIOD_MS = 100;

const long SERIAL_BAUD = 115200;

// =============================================================================
// End of CONFIG
// =============================================================================

// Forward declarations (arduino-cli generates these, a host C++ build doesn't).
void motorWrite(uint8_t ch, float duty, bool brake);
void motorAStop();
void stopAll();
void handleLine(char *line);
void serialPoll();
void stepperPoll();
void celebratePoll();
void watchdogPoll();
bool accelInit();
bool accelRead(int16_t *ax, int16_t *ay, int16_t *az);
void accelPoll();

// ---- State ----------------------------------------------------------------------
static char     lineBuf[32];
static uint8_t  lineLen = 0;
static bool     lineOverflow = false;

static bool     motorAActive = false;
static uint32_t lastMMs = 0;

static bool     celebrating = false;
static uint32_t celebrateStartMs = 0;
static uint32_t celebrateDurMs = 0;

static long     stepPos = 0;          // current absolute half-step position
static long     stepTarget = 0;
static uint8_t  stepPhase = 0;        // 0..7 index into HALF_STEP
static bool     stepEnergized = false;
static uint32_t lastStepUs = 0;
static uint32_t stepArrivedMs = 0;

static const uint8_t HALF_STEP[8] = {
  0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,
};

static uint8_t  mpuAddr = 0;          // 0 = no accelerometer
static uint32_t lastAccelUs = 0;
static float    baselineMg = 1000.0f;
static bool     hitWindowOpen = false;
static uint32_t hitWindowStartMs = 0;
static uint16_t hitPeakMg = 0;
static uint32_t lastHitMs = 0;
static bool     hitEverSent = false;
#if DEBUG
static uint32_t lastDebugMs = 0;
static uint16_t debugPeakMg = 0;
static uint16_t lastMagMg = 0;
#endif

// ---- Motors ---------------------------------------------------------------------
static uint8_t dutyToPwm(float d) {
  if (d < 0) d = -d;
  if (d > 1) d = 1;
  return (uint8_t)(d * 255.0f + 0.5f);
}

// duty in [-1, 1] (already shaped). brake only matters when duty == 0.
void motorWrite(uint8_t ch, float duty, bool brake) {
  uint8_t pwm = dutyToPwm(duty);
#if DRIVER_TYPE == DRIVER_2PWM
  uint8_t in1 = ch == 0 ? PIN_A_IN1 : PIN_B_IN1;
  uint8_t in2 = ch == 0 ? PIN_A_IN2 : PIN_B_IN2;
  if (pwm == 0) {
    // both HIGH = brake (short motor), both LOW = coast
    digitalWrite(in1, brake ? HIGH : LOW);
    digitalWrite(in2, brake ? HIGH : LOW);
  } else if (duty > 0) {
    digitalWrite(in2, LOW);
    analogWrite(in1, pwm);
  } else {
    digitalWrite(in1, LOW);
    analogWrite(in2, pwm);
  }
#else
  uint8_t en  = ch == 0 ? PIN_A_EN  : PIN_B_EN;
  uint8_t in1 = ch == 0 ? PIN_A_IN1 : PIN_B_IN1;
  uint8_t in2 = ch == 0 ? PIN_A_IN2 : PIN_B_IN2;
  if (pwm == 0) {
    // Brake: EN high + IN1 = IN2 = HIGH (short brake on both L298N and
    // TB6612; TB6612 treats LOW/LOW as coast). Coast: EN low, inputs low.
    digitalWrite(in1, brake ? HIGH : LOW);
    digitalWrite(in2, brake ? HIGH : LOW);
    digitalWrite(en, brake ? HIGH : LOW);
  } else {
    digitalWrite(in1, duty > 0 ? HIGH : LOW);
    digitalWrite(in2, duty > 0 ? LOW : HIGH);
    analogWrite(en, pwm);
  }
#endif
}

// Contract duty -> shaped output duty for the goalie motor.
static float shapeA(float d) {
  if (d > 1) d = 1;
  if (d < -1) d = -1;
  float mag = d < 0 ? -d : d;
  if (mag < MOTOR_A_DEADBAND) return 0.0f;
  // Identity when MIN_DUTY == 0; otherwise [0, 1] -> [MIN_DUTY, 1].
  float out = MOTOR_A_MIN_DUTY + (1.0f - MOTOR_A_MIN_DUTY) * mag;
  if (d < 0) out = -out;
  if (MOTOR_A_INVERT) out = -out;
  return out;
}

static void motorASet(float d) {
  float out = shapeA(d);
  motorWrite(0, out, MOTOR_A_BRAKE_ON_STOP);
  motorAActive = out != 0.0f;
}

void motorAStop() {
  motorWrite(0, 0.0f, MOTOR_A_BRAKE_ON_STOP);
  motorAActive = false;
}

static void motorBStop() {
  motorWrite(1, 0.0f, false);
  celebrating = false;
}

static void stepRelease() {
  for (uint8_t i = 0; i < 4; i++) digitalWrite(PIN_STEP[i], LOW);
  stepEnergized = false;
}

void stopAll() {
  motorAStop();
  motorBStop();
  stepTarget = stepPos;     // halt the needle where it is
  stepRelease();
}

// ---- Parsing ----------------------------------------------------------------------
// Parse exactly one number after the command letter; reject trailing garbage.
static bool parseFloatArg(const char *s, float *out) {
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return false;
  char *end;
  double v = strtod(s, &end);
  if (end == s) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end) return false;
  if (v != v) return false;  // NaN
  *out = (float)v;
  return true;
}

static bool parseLongArg(const char *s, long *out) {
  while (*s == ' ' || *s == '\t') s++;
  if (!*s) return false;
  char *end;
  long v = strtol(s, &end, 10);
  if (end == s) return false;
  while (*end == ' ' || *end == '\t') end++;
  if (*end) return false;
  *out = v;
  return true;
}

static bool onlySpace(const char *s) {
  while (*s == ' ' || *s == '\t') s++;
  return *s == 0;
}

// One complete line (no terminator). Unknown/malformed lines are ignored:
// the protocol is fire-and-forget and the Pi never waits for a reply.
void handleLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32;  // Serial Monitor convenience
  if (!cmd) return;
  const char *arg = line + 1;
  if (*arg && *arg != ' ' && *arg != '\t') return;  // e.g. "MX 1": garbage

  switch (cmd) {
    case 'M': {
      float d;
      if (!parseFloatArg(arg, &d)) return;
      lastMMs = millis();
      motorASet(d);
      break;
    }
    case 'S':
      if (onlySpace(arg)) stopAll();
      break;
    case 'N': {
      long n;
      if (!parseLongArg(arg, &n)) return;
      stepTarget = n;
      break;
    }
    case 'C': {
      long ms;
      if (!parseLongArg(arg, &ms)) return;
      if (ms <= 0) { motorBStop(); return; }
      if (ms > CELEBRATE_MAX_MS) ms = CELEBRATE_MAX_MS;
      celebrateStartMs = millis();
      celebrateDurMs = (uint32_t)ms;
      celebrating = true;
      motorWrite(1, MOTOR_B_INVERT ? -CELEBRATE_DUTY : CELEBRATE_DUTY, false);
      break;
    }
    case 'Z':
      if (onlySpace(arg)) { stepPos = 0; stepTarget = 0; }
      break;
    default:
      break;
  }
}

// Drain whatever is in the RX buffer without ever waiting.
void serialPoll() {
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') {
      if (!lineOverflow && lineLen > 0) {
        lineBuf[lineLen] = 0;
        handleLine(lineBuf);
      }
      lineLen = 0;
      lineOverflow = false;
    } else if (lineOverflow) {
      // drop until end of line
    } else if (c < 0x20 || c > 0x7E) {
      lineOverflow = true;  // binary garbage: discard the whole line
    } else if (lineLen >= sizeof(lineBuf) - 1) {
      lineOverflow = true;  // too long: discard the whole line
    } else {
      lineBuf[lineLen++] = (char)c;
    }
  }
}

// ---- Watchdog / flag / stepper ---------------------------------------------------
void watchdogPoll() {
  if (motorAActive && (uint32_t)(millis() - lastMMs) > MOTOR_WATCHDOG_MS) {
    motorAStop();
  }
}

void celebratePoll() {
  if (!celebrating) return;
  uint32_t el = millis() - celebrateStartMs;
  if (el >= celebrateDurMs) { motorBStop(); return; }
#if CELEBRATE_WIGGLE_MS > 0
  bool flip = ((el / CELEBRATE_WIGGLE_MS) & 1) != 0;
  float d = flip ? -CELEBRATE_DUTY : CELEBRATE_DUTY;
  if (MOTOR_B_INVERT) d = -d;
  motorWrite(1, d, false);
#endif
}

static void stepApply() {
  uint8_t pat = HALF_STEP[stepPhase];
  for (uint8_t i = 0; i < 4; i++) digitalWrite(PIN_STEP[i], (pat >> i) & 1 ? HIGH : LOW);
  stepEnergized = true;
}

void stepperPoll() {
  uint32_t nowUs = micros();
  if (stepPos == stepTarget) {
    if (stepEnergized && (uint32_t)(millis() - stepArrivedMs) >= STEP_RELEASE_MS) stepRelease();
    return;
  }
  if (stepEnergized && (uint32_t)(nowUs - lastStepUs) < STEP_INTERVAL_US) return;
  if (!stepEnergized) {
    // Re-energize on the current phase first so the rotor doesn't skip.
    stepApply();
    lastStepUs = nowUs;
    return;
  }
  int8_t dir = stepTarget > stepPos ? 1 : -1;
  stepPos += dir;
  int8_t phaseDir = STEP_INVERT ? -dir : dir;
  stepPhase = (uint8_t)((stepPhase + 8 + phaseDir) & 7);
  stepApply();
  lastStepUs = nowUs;
  if (stepPos == stepTarget) stepArrivedMs = millis();
}

// ---- Accelerometer ---------------------------------------------------------------
static bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool accelInit() {
  uint8_t addr = 0;
  if (mpuProbe(MPU_ADDR_0)) addr = MPU_ADDR_0;
  else if (mpuProbe(MPU_ADDR_1)) addr = MPU_ADDR_1;
  if (!addr) return false;
  // PWR_MGMT_1: wake, PLL with X gyro clock. ACCEL_CONFIG: +-8g. CONFIG: DLPF 184 Hz.
  if (!i2cWrite(addr, 0x6B, 0x01)) return false;
  if (!i2cWrite(addr, 0x1C, 0x10)) return false;
  if (!i2cWrite(addr, 0x1A, 0x01)) return false;
  mpuAddr = addr;
  return true;
}

bool accelRead(int16_t *ax, int16_t *ay, int16_t *az) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) b[i] = (uint8_t)Wire.read();
  *ax = (int16_t)((b[0] << 8) | b[1]);
  *ay = (int16_t)((b[2] << 8) | b[3]);
  *az = (int16_t)((b[4] << 8) | b[5]);
  return true;
}

// Deviation of |a| from a slow baseline (≈1 g, tracks sensor offset/tilt).
// Above threshold opens a short window, the window's peak is sent as HIT.
void accelPoll() {
  if (!mpuAddr) return;
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastAccelUs) < ACCEL_PERIOD_US) return;
  lastAccelUs = nowUs;

  int16_t ax, ay, az;
  if (!accelRead(&ax, &ay, &az)) return;
  float x = ax, y = ay, z = az;
  float magMg = sqrtf(x * x + y * y + z * z) * (1000.0f / ACCEL_LSB_PER_G);
  float devF = magMg - baselineMg;
  if (devF < 0) devF = -devF;
  uint16_t dev = devF > 60000.0f ? 60000 : (uint16_t)devF;
  uint32_t nowMs = millis();

  if (dev < HIT_THRESHOLD_MG) baselineMg += BASELINE_ALPHA * (magMg - baselineMg);

#if DEBUG
  lastMagMg = magMg > 60000.0f ? 60000 : (uint16_t)magMg;
  if (dev > debugPeakMg) debugPeakMg = dev;
#endif

  bool cooling = hitEverSent && (uint32_t)(nowMs - lastHitMs) < HIT_RATE_LIMIT_MS;
  if (!hitWindowOpen) {
    if (dev >= HIT_THRESHOLD_MG && !cooling) {
      hitWindowOpen = true;
      hitWindowStartMs = nowMs;
      hitPeakMg = dev;
    }
  } else {
    if (dev > hitPeakMg) hitPeakMg = dev;
    if ((uint32_t)(nowMs - hitWindowStartMs) >= HIT_WINDOW_MS) {
      Serial.print(F("HIT "));
      Serial.println(hitPeakMg);
      hitWindowOpen = false;
      lastHitMs = nowMs;
      hitEverSent = true;
    }
  }
}

#if DEBUG
static void debugPoll() {
  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastDebugMs) < DEBUG_PERIOD_MS) return;
  lastDebugMs = nowMs;
  if (!mpuAddr || Serial.availableForWrite() < 24) return;  // never block on TX
  Serial.print(F("A "));
  Serial.println(lastMagMg);
  Serial.print(F("# peak "));
  Serial.println(debugPeakMg);
  debugPeakMg = 0;
}
#endif

// ---- Arduino entry points --------------------------------------------------------
void setup() {
#if DRIVER_TYPE == DRIVER_EN_IN
  pinMode(PIN_A_EN, OUTPUT);
  pinMode(PIN_B_EN, OUTPUT);
#endif
  pinMode(PIN_A_IN1, OUTPUT);
  pinMode(PIN_A_IN2, OUTPUT);
  pinMode(PIN_B_IN1, OUTPUT);
  pinMode(PIN_B_IN2, OUTPUT);
  for (uint8_t i = 0; i < 4; i++) pinMode(PIN_STEP[i], OUTPUT);
  stopAll();

  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);  // a loose I2C wire must never hang loop()
#endif
  if (accelInit()) {
    // Seed the gravity baseline so the first samples aren't counted as hits.
    float sum = 0;
    uint8_t n = 0;
    for (uint8_t i = 0; i < 16; i++) {
      int16_t ax, ay, az;
      if (accelRead(&ax, &ay, &az)) {
        float x = ax, y = ay, z = az;
        sum += sqrtf(x * x + y * y + z * z) * (1000.0f / ACCEL_LSB_PER_G);
        n++;
      }
      delay(2);
    }
    if (n) baselineMg = sum / n;
    Serial.print(F("# mpu6050 at 0x"));
    Serial.println(mpuAddr, HEX);
  } else {
    Serial.println(F("# no accelerometer found, HIT disabled"));
  }

  Serial.println(F("READY"));
}

void loop() {
  serialPoll();
  watchdogPoll();
  celebratePoll();
  stepperPoll();
  accelPoll();
#if DEBUG
  debugPoll();
#endif
}
