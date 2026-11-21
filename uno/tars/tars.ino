/*
 * TARS: Arduino Uno motor/arm/sensor co-processor (replaces the Pico).
 *
 *   Pi 5 (QNX tars_motor -b uno) --USB serial 115200 8N1--> this Uno
 *     -> H-bridge ch A -> left drive motor   (DC gearmotor under middle slab 1)
 *     -> H-bridge ch B -> right drive motor  (DC gearmotor under middle slab 2)
 *     -> ULN2003 #0    -> 28BYJ-48 arm 0 (left outer slab)
 *     -> ULN2003 #1    -> 28BYJ-48 arm 1 (right outer slab)
 *     <- accelerometer (I2C: ADXL345 / GY-291, or MPU6050 / GY-521) -> tip + bump
 *
 * The Uno is deliberately dumb: gestures (wave, point, ...) are sequences of
 * A commands run by the Pi. The Uno only does things that must be local and
 * fast: PWM, stepping, the drive watchdog and the tip/bump safety stop.
 *
 * Protocol (ASCII lines, '\n' or '\r' terminated, no replies to commands):
 *   Pi -> Uno
 *     D <l> <r>            drive, floats in [-1, 1]. Must be resent < 200 ms
 *                          (watchdog) - the Pi sends it every 20 ms tick.
 *     S                    stop drive motors AND halt both arms where they are
 *     A <arm> <deg> [dps]  arm 0|1 to absolute angle (deg, clamped to
 *                          +-ARM_LIMIT_DEG) at dps deg/s (default
 *                          ARM_DEFAULT_DPS, max ARM_MAX_DPS). Non-blocking;
 *                          both arms can move at once.
 *     H                    both arms to 0 deg at the default speed
 *     Z                    define the arms' current positions as 0 deg
 *     X                    halt both arms (drive unaffected)
 *     U                    "this is upright": recapture the tilt reference
 *     ?                    print a "# ..." status line (bench debugging)
 *   Uno -> Pi
 *     READY                once after boot (mid-run = the Uno rebooted; arm
 *                          zero is lost, the Pi resets its arm model to 0)
 *     T <deg>              tipped: tilt from upright > TIP_DEG for TIP_HOLD_MS.
 *                          Repeated every TIP_REPEAT_MS while tipped. The Uno
 *                          stops the drive motors, halts the arms and ignores
 *                          D until upright again.
 *     B <mg>               bump: |accel| spike above BUMP_THRESHOLD_MG (peak
 *                          deviation in milli-g), at most one per 250 ms
 *     # ...                diagnostics. The Pi ignores lines starting with '#'.
 *
 * Bench test: Serial Monitor at 115200, line ending "Newline". Try "A 0 90",
 * "A 1 -45 20", "H", "?". "D 0.5 0.5" only runs for 200 ms (the watchdog).
 */

#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <math.h>

// =============================================================================
// CONFIG: edit this block on the bench. Everything else should not need changes.
// =============================================================================

// ---- Motor driver type -------------------------------------------------------
// Read the chip markings (phone camera + zoom):
//   DRIVER_2PWM  : L9110S / HG7881, DRV8833 (tie nSLEEP high), MX1508.
//                  4 control inputs, NO enable pins.
//   DRIVER_EN_IN : L298N (big chip + heatsink; REMOVE the ENA/ENB jumpers),
//                  TB6612FNG (tie STBY high). An enable/PWM pin + 2 direction
//                  pins per channel.
#define DRIVER_2PWM  1
#define DRIVER_EN_IN 2
#ifndef DRIVER_TYPE
#define DRIVER_TYPE  DRIVER_2PWM
#endif

// ---- Pins ---------------------------------------------------------------------
// PWM only on Timer1 (9, 10) and Timer2 (3, 11); pins 5/6 (Timer0, millis())
// are used as plain digital outputs for arm 1, which is fine.
// Serial = 0/1, I2C = A4/A5, arm 0 = A0-A3, arm 1 = D4-D7 IN BOTH MODES, so
// switching driver type never means rewiring the arms.
#if DRIVER_TYPE == DRIVER_2PWM
const uint8_t PIN_L_IN1 = 9;    // L9110S A-IA / DRV8833 AIN1 / MX1508 IN1
const uint8_t PIN_L_IN2 = 10;   // L9110S A-IB / DRV8833 AIN2 / MX1508 IN2
const uint8_t PIN_R_IN1 = 3;    // L9110S B-IA / DRV8833 BIN1 / MX1508 IN3
const uint8_t PIN_R_IN2 = 11;   // L9110S B-IB / DRV8833 BIN2 / MX1508 IN4
// free: D2, D8, D12, D13
#elif DRIVER_TYPE == DRIVER_EN_IN
const uint8_t PIN_L_EN  = 9;    // L298N ENA / TB6612 PWMA
const uint8_t PIN_L_IN1 = 8;    // L298N IN1 / TB6612 AIN1
const uint8_t PIN_L_IN2 = 12;   // L298N IN2 / TB6612 AIN2
const uint8_t PIN_R_EN  = 3;    // L298N ENB / TB6612 PWMB
const uint8_t PIN_R_IN1 = 2;    // L298N IN3 / TB6612 BIN1
const uint8_t PIN_R_IN2 = 11;   // L298N IN4 / TB6612 BIN2
// free: D10, D13
#else
#error "DRIVER_TYPE must be DRIVER_2PWM or DRIVER_EN_IN"
#endif

const uint8_t PIN_ARM[2][4] = {
  {A0, A1, A2, A3},             // arm 0: ULN2003 #0 IN1..IN4
  {4, 5, 6, 7},                 // arm 1: ULN2003 #1 IN1..IN4
};

// ---- Drive tuning ---------------------------------------------------------------
// INVERT: flip if "D 0.5 0.5" spins that wheel backwards (or swap its wires).
const bool  LEFT_INVERT  = false;
const bool  RIGHT_INVERT = false;
const float DRIVE_DEADBAND = 0.03f;  // |d| below this -> off (no hum)
// MIN_DUTY: duty that just starts the wheel turning on the floor. Commands
// above the deadband map |d| -> MIN_DUTY + (1 - MIN_DUTY) * |d|. Measure it:
// send "D x x" repeatedly, raise x until the robot creeps. 0 = no remap.
const float DRIVE_MIN_DUTY = 0.0f;
const bool  BRAKE_ON_STOP  = true;   // short-brake on stop/watchdog vs coast
const uint16_t DRIVE_WATCHDOG_MS = 200;

// ---- Arms (28BYJ-48 + ULN2003) --------------------------------------------------
// Half-step mode: 4096 half-steps per output revolution (4076 on some units).
const float    ARM_STEPS_PER_REV = 4096.0f;
const float    ARM_MAX_DPS       = 60.0f;   // ~700 half-steps/s, the motor's limit at 5 V
const float    ARM_DEFAULT_DPS   = 45.0f;
const float    ARM_LIMIT_DEG     = 135.0f;  // never command beyond +-this
// The two steppers face opposite directions, so one is mirrored: with these
// defaults "+deg" swings both slabs forward. Flip on the bench if not.
const bool     ARM_INVERT[2]     = {false, true};
const uint16_t ARM_RELEASE_MS    = 20;      // coils off this long after arriving
// true: keep coils energized when idle (~240 mA each, gets warm). The 64:1
// gearbox usually holds a centre-pivoted slab with the coils off.
const bool     ARM_HOLD          = false;

// ---- Accelerometer -------------------------------------------------------------
// Auto-detected at boot, first match wins; the boot "#" line says which:
//   MPU6050 (GY-521)  at 0x68 (AD0 low) or 0x69, +-8g, 4096 LSB/g
//   ADXL345 (GY-291)  at 0x53 (SDO low, the usual) or 0x1D (SDO high),
//                     checked via DEVID (reg 0x00 == 0xE5), full-res +-16g, 256 LSB/g
// Only acceleration is used, so tip/bump logic is identical for both.
// GY-291 wiring: VCC 3.3V or 5V (board has a regulator), GND, SDA A4, SCL A5,
// CS -> 3.3V (selects I2C; most GY-291 boards pull it up already).
const uint8_t  MPU_ADDR_0  = 0x68;
const uint8_t  MPU_ADDR_1  = 0x69;
const uint8_t  ADXL_ADDR_0 = 0x53;
const uint8_t  ADXL_ADDR_1 = 0x1D;
const float    MPU_LSB_PER_G  = 4096.0f;
const float    ADXL_LSB_PER_G = 256.0f;    // 3.9 mg/LSB in full-res mode
const uint16_t ACCEL_PERIOD_US    = 5000;     // 200 Hz
const float    TILT_ALPHA         = 0.1f;     // tilt low-pass (~50 ms)
const float    TIP_DEG            = 40.0f;    // tilted further than this ...
const uint16_t TIP_HOLD_MS        = 150;      // ... for this long = tipped
const float    UPRIGHT_DEG        = 25.0f;    // back under this ...
const uint16_t UPRIGHT_HOLD_MS    = 500;      // ... for this long = upright again
const uint16_t TIP_REPEAT_MS      = 500;
const uint16_t BUMP_THRESHOLD_MG  = 1200;     // tune: driving vibration vs a real bump
const uint16_t BUMP_WINDOW_MS     = 15;
const uint16_t BUMP_RATE_LIMIT_MS = 250;
const float    BASELINE_ALPHA     = 0.005f;   // slow |a| baseline (~1 s at 200 Hz)

const long SERIAL_BAUD = 115200;

// =============================================================================
// End of CONFIG
// =============================================================================

void motorWrite(uint8_t ch, float duty, bool brake);
void driveSet(float l, float r);
void driveStop();
void armsHalt();
void stopAll();
void handleLine(char *line);
void serialPoll();
void watchdogPoll();
void armPoll(uint8_t i);
bool accelInit();
bool accelRead(int16_t *ax, int16_t *ay, int16_t *az);
void accelPoll();
void captureUpright();

// ---- State ----------------------------------------------------------------------
static char     lineBuf[40];
static uint8_t  lineLen = 0;
static bool     lineOverflow = false;

static bool     driveActive = false;
static uint32_t lastDMs = 0;

struct Arm {
  long     pos;         // half-steps, 0 = home
  long     target;
  uint8_t  phase;       // 0..7
  bool     energized;
  uint32_t intervalUs;
  uint32_t lastStepUs;
  uint32_t arrivedMs;
};
static Arm arms[2];

static const uint8_t HALF_STEP[8] = {
  0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,
};

#define ACCEL_NONE 0
#define ACCEL_MPU6050 1
#define ACCEL_ADXL345 2
static uint8_t  accelType = ACCEL_NONE;
static uint8_t  accelAddr = 0;        // 0 = no accelerometer
static float    accelLsbPerG = MPU_LSB_PER_G;
static uint32_t lastAccelUs = 0;
static float    upX = 0, upY = 0, upZ = 1;   // upright gravity direction (unit)
static float    fX = 0, fY = 0, fZ = 1;      // filtered gravity (g)
static float    tiltDeg = 0;
static bool     tipped = false;
static uint32_t tipSinceMs = 0;       // when tilt first exceeded TIP_DEG (0 = not)
static uint32_t uprightSinceMs = 0;   // while tipped: when tilt went below UPRIGHT_DEG
static uint32_t lastTipMsgMs = 0;
static float    baselineMg = 1000.0f;
static bool     bumpWindowOpen = false;
static uint32_t bumpWindowStartMs = 0;
static uint16_t bumpPeakMg = 0;
static uint32_t lastBumpMs = 0;
static bool     bumpEverSent = false;

// ---- Drive motors ---------------------------------------------------------------
static uint8_t dutyToPwm(float d) {
  if (d < 0) d = -d;
  if (d > 1) d = 1;
  return (uint8_t)(d * 255.0f + 0.5f);
}

// ch 0 = left, 1 = right. duty already shaped. brake only matters at duty 0.
void motorWrite(uint8_t ch, float duty, bool brake) {
  uint8_t pwm = dutyToPwm(duty);
#if DRIVER_TYPE == DRIVER_2PWM
  uint8_t in1 = ch == 0 ? PIN_L_IN1 : PIN_R_IN1;
  uint8_t in2 = ch == 0 ? PIN_L_IN2 : PIN_R_IN2;
  if (pwm == 0) {
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
  uint8_t en  = ch == 0 ? PIN_L_EN  : PIN_R_EN;
  uint8_t in1 = ch == 0 ? PIN_L_IN1 : PIN_R_IN1;
  uint8_t in2 = ch == 0 ? PIN_L_IN2 : PIN_R_IN2;
  if (pwm == 0) {
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

static float shape(float d, bool invert) {
  if (d > 1) d = 1;
  if (d < -1) d = -1;
  float mag = d < 0 ? -d : d;
  if (mag < DRIVE_DEADBAND) return 0.0f;
  float out = DRIVE_MIN_DUTY + (1.0f - DRIVE_MIN_DUTY) * mag;
  if (d < 0) out = -out;
  return invert ? -out : out;
}

void driveSet(float l, float r) {
  float ol = shape(l, LEFT_INVERT), orr = shape(r, RIGHT_INVERT);
  motorWrite(0, ol, BRAKE_ON_STOP);
  motorWrite(1, orr, BRAKE_ON_STOP);
  driveActive = ol != 0.0f || orr != 0.0f;
}

void driveStop() {
  motorWrite(0, 0.0f, BRAKE_ON_STOP);
  motorWrite(1, 0.0f, BRAKE_ON_STOP);
  driveActive = false;
}

// ---- Arms -------------------------------------------------------------------------
static float stepsPerDeg() { return ARM_STEPS_PER_REV / 360.0f; }

static void armRelease(uint8_t i) {
  for (uint8_t k = 0; k < 4; k++) digitalWrite(PIN_ARM[i][k], LOW);
  arms[i].energized = false;
}

static void armApply(uint8_t i) {
  uint8_t pat = HALF_STEP[arms[i].phase];
  for (uint8_t k = 0; k < 4; k++) digitalWrite(PIN_ARM[i][k], (pat >> k) & 1 ? HIGH : LOW);
  arms[i].energized = true;
}

static void armMove(uint8_t i, float deg, float dps) {
  if (deg > ARM_LIMIT_DEG) deg = ARM_LIMIT_DEG;
  if (deg < -ARM_LIMIT_DEG) deg = -ARM_LIMIT_DEG;
  if (!(dps > 0)) dps = ARM_DEFAULT_DPS;
  if (dps > ARM_MAX_DPS) dps = ARM_MAX_DPS;
  if (dps < 1.0f) dps = 1.0f;
  arms[i].target = lround(deg * stepsPerDeg());
  arms[i].intervalUs = (uint32_t)(1000000.0f / (dps * stepsPerDeg()));
  if (arms[i].pos == arms[i].target) arms[i].arrivedMs = millis();
}

void armsHalt() {
  for (uint8_t i = 0; i < 2; i++) {
    arms[i].target = arms[i].pos;
    arms[i].arrivedMs = millis();
  }
}

void stopAll() {
  driveStop();
  armsHalt();
}

void armPoll(uint8_t i) {
  Arm &a = arms[i];
  uint32_t nowUs = micros();
  if (a.pos == a.target) {
    if (!ARM_HOLD && a.energized && (uint32_t)(millis() - a.arrivedMs) >= ARM_RELEASE_MS) armRelease(i);
    return;
  }
  if (!a.energized) {
    // Re-energize on the current phase first so the rotor doesn't skip.
    armApply(i);
    a.lastStepUs = nowUs;
    return;
  }
  if ((uint32_t)(nowUs - a.lastStepUs) < a.intervalUs) return;
  int8_t dir = a.target > a.pos ? 1 : -1;
  a.pos += dir;
  int8_t pd = ARM_INVERT[i] ? -dir : dir;
  a.phase = (uint8_t)((a.phase + 8 + pd) & 7);
  armApply(i);
  a.lastStepUs = nowUs;
  if (a.pos == a.target) a.arrivedMs = millis();
}

// ---- Parsing ----------------------------------------------------------------------
static const char *skipSpace(const char *s) {
  while (*s == ' ' || *s == '\t') s++;
  return s;
}

// Parse up to max floats separated by whitespace; reject trailing garbage.
// Returns the count parsed, or -1 on garbage.
static int parseFloats(const char *s, float *out, int max) {
  int n = 0;
  s = skipSpace(s);
  while (*s) {
    if (n >= max) return -1;
    char *end;
    double v = strtod(s, &end);
    if (end == s || v != v) return -1;
    if (*end && *end != ' ' && *end != '\t') return -1;
    out[n++] = (float)v;
    s = skipSpace(end);
  }
  return n;
}

static void printStatus() {
  Serial.print(F("# arms "));
  Serial.print((long)lround(arms[0].pos / stepsPerDeg()));
  Serial.print(F(" "));
  Serial.print((long)lround(arms[1].pos / stepsPerDeg()));
  Serial.print(F(" tilt "));
  Serial.print((long)lround(tiltDeg));
  Serial.print(F(" tipped "));
  Serial.print((int)tipped);
  Serial.print(F(" accel "));
  Serial.println(accelType == ACCEL_MPU6050 ? F("mpu6050") : accelType == ACCEL_ADXL345 ? F("adxl345") : F("none"));
}

// One complete line (no terminator). Malformed lines are ignored.
void handleLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32;  // Serial Monitor convenience
  if (!cmd) return;
  const char *arg = line + 1;
  if (*arg && *arg != ' ' && *arg != '\t') return;  // e.g. "DX 1": garbage
  float v[3];
  int n = parseFloats(arg, v, 3);
  if (n < 0) return;

  switch (cmd) {
    case 'D':
      if (n != 2) return;
      lastDMs = millis();
      if (tipped) { driveStop(); return; }  // refuse to drive while tipped over
      driveSet(v[0], v[1]);
      break;
    case 'S':
      if (n == 0) stopAll();
      break;
    case 'A': {
      if (n < 2) return;
      if (v[0] != 0.0f && v[0] != 1.0f) return;
      if (tipped) return;
      armMove((uint8_t)v[0], v[1], n == 3 ? v[2] : ARM_DEFAULT_DPS);
      break;
    }
    case 'H':
      if (n == 0 && !tipped) { armMove(0, 0, ARM_DEFAULT_DPS); armMove(1, 0, ARM_DEFAULT_DPS); }
      break;
    case 'Z':
      if (n == 0) for (uint8_t i = 0; i < 2; i++) { arms[i].pos = 0; arms[i].target = 0; }
      break;
    case 'X':
      if (n == 0) armsHalt();
      break;
    case 'U':
      if (n == 0) captureUpright();
      break;
    case '?':
      if (n == 0) printStatus();
      break;
    default:
      break;
  }
}

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
      lineOverflow = true;
    } else if (lineLen >= sizeof(lineBuf) - 1) {
      lineOverflow = true;
    } else {
      lineBuf[lineLen++] = (char)c;
    }
  }
}

void watchdogPoll() {
  if (driveActive && (uint32_t)(millis() - lastDMs) > DRIVE_WATCHDOG_MS) driveStop();
}

// ---- Accelerometer ---------------------------------------------------------------
static bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool i2cReadReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
  return true;
}

static bool mpuInit(uint8_t addr) {
  if (!i2cProbe(addr)) return false;
  if (!i2cWrite(addr, 0x6B, 0x01)) return false;  // PWR_MGMT_1: wake, PLL clock
  if (!i2cWrite(addr, 0x1C, 0x10)) return false;  // ACCEL_CONFIG: +-8g
  if (!i2cWrite(addr, 0x1A, 0x03)) return false;  // CONFIG: DLPF ~44 Hz
  accelType = ACCEL_MPU6050; accelAddr = addr; accelLsbPerG = MPU_LSB_PER_G;
  return true;
}

static bool adxlInit(uint8_t addr) {
  uint8_t id;
  if (!i2cProbe(addr) || !i2cReadReg(addr, 0x00, &id, 1) || id != 0xE5) return false;
  if (!i2cWrite(addr, 0x2C, 0x0B)) return false;  // BW_RATE: 200 Hz output rate
  if (!i2cWrite(addr, 0x31, 0x0B)) return false;  // DATA_FORMAT: full-res, +-16g
  if (!i2cWrite(addr, 0x2D, 0x08)) return false;  // POWER_CTL: measure
  accelType = ACCEL_ADXL345; accelAddr = addr; accelLsbPerG = ADXL_LSB_PER_G;
  return true;
}

bool accelInit() {
  accelType = ACCEL_NONE;
  accelAddr = 0;
  return mpuInit(MPU_ADDR_0) || mpuInit(MPU_ADDR_1) ||
         adxlInit(ADXL_ADDR_0) || adxlInit(ADXL_ADDR_1);
}

// Raw counts; scale with accelLsbPerG.
bool accelRead(int16_t *ax, int16_t *ay, int16_t *az) {
  uint8_t b[6];
  if (accelType == ACCEL_MPU6050) {
    if (!i2cReadReg(accelAddr, 0x3B, b, 6)) return false;  // ACCEL_XOUT_H.., big-endian
    *ax = (int16_t)((b[0] << 8) | b[1]);
    *ay = (int16_t)((b[2] << 8) | b[3]);
    *az = (int16_t)((b[4] << 8) | b[5]);
    return true;
  }
  if (accelType == ACCEL_ADXL345) {
    if (!i2cReadReg(accelAddr, 0x32, b, 6)) return false;  // DATAX0.., little-endian
    *ax = (int16_t)((b[1] << 8) | b[0]);
    *ay = (int16_t)((b[3] << 8) | b[2]);
    *az = (int16_t)((b[5] << 8) | b[4]);
    return true;
  }
  return false;
}

// Average a few samples and call that direction "upright".
void captureUpright() {
  if (!accelAddr) return;
  float sx = 0, sy = 0, sz = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < 16; i++) {
    int16_t ax, ay, az;
    if (accelRead(&ax, &ay, &az)) { sx += ax; sy += ay; sz += az; n++; }
    delay(2);
  }
  if (!n) return;
  float m = sqrtf(sx * sx + sy * sy + sz * sz);
  if (m < 1.0f) return;
  upX = sx / m; upY = sy / m; upZ = sz / m;
  float g = m / n / accelLsbPerG;
  fX = upX * g; fY = upY * g; fZ = upZ * g;
  baselineMg = g * 1000.0f;
  tiltDeg = 0;
  tipped = false;
  tipSinceMs = 0;
  Serial.println(F("# upright captured"));
}

static void sendTip() {
  Serial.print(F("T "));
  Serial.println((long)lround(tiltDeg));
  lastTipMsgMs = millis();
}

void accelPoll() {
  if (!accelAddr) return;
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastAccelUs) < ACCEL_PERIOD_US) return;
  lastAccelUs = nowUs;

  int16_t ax, ay, az;
  if (!accelRead(&ax, &ay, &az)) return;
  const float k = 1.0f / accelLsbPerG;
  float x = ax * k, y = ay * k, z = az * k;   // g
  float magMg = sqrtf(x * x + y * y + z * z) * 1000.0f;
  uint32_t nowMs = millis();

  // ---- bump: |a| spike relative to a slow baseline ----
  float devF = magMg - baselineMg;
  if (devF < 0) devF = -devF;
  uint16_t dev = devF > 60000.0f ? 60000 : (uint16_t)devF;
  if (dev < BUMP_THRESHOLD_MG) baselineMg += BASELINE_ALPHA * (magMg - baselineMg);
  bool cooling = bumpEverSent && (uint32_t)(nowMs - lastBumpMs) < BUMP_RATE_LIMIT_MS;
  if (!bumpWindowOpen) {
    if (dev >= BUMP_THRESHOLD_MG && !cooling) {
      bumpWindowOpen = true;
      bumpWindowStartMs = nowMs;
      bumpPeakMg = dev;
    }
  } else {
    if (dev > bumpPeakMg) bumpPeakMg = dev;
    if ((uint32_t)(nowMs - bumpWindowStartMs) >= BUMP_WINDOW_MS) {
      Serial.print(F("B "));
      Serial.println(bumpPeakMg);
      bumpWindowOpen = false;
      lastBumpMs = nowMs;
      bumpEverSent = true;
    }
  }

  // ---- tilt: angle between filtered gravity and the upright reference ----
  fX += TILT_ALPHA * (x - fX);
  fY += TILT_ALPHA * (y - fY);
  fZ += TILT_ALPHA * (z - fZ);
  float fm = sqrtf(fX * fX + fY * fY + fZ * fZ);
  if (fm < 0.3f) return;  // free fall / garbage: no opinion
  float c = (fX * upX + fY * upY + fZ * upZ) / fm;
  if (c > 1) c = 1;
  if (c < -1) c = -1;
  tiltDeg = acosf(c) * 57.29578f;

  if (!tipped) {
    if (tiltDeg > TIP_DEG) {
      if (!tipSinceMs) tipSinceMs = nowMs ? nowMs : 1;
      if ((uint32_t)(nowMs - tipSinceMs) >= TIP_HOLD_MS) {
        tipped = true;
        uprightSinceMs = 0;
        stopAll();
        sendTip();
      }
    } else {
      tipSinceMs = 0;
    }
  } else {
    if (tiltDeg < UPRIGHT_DEG) {
      if (!uprightSinceMs) uprightSinceMs = nowMs ? nowMs : 1;
      if ((uint32_t)(nowMs - uprightSinceMs) >= UPRIGHT_HOLD_MS) {
        tipped = false;
        tipSinceMs = 0;
        Serial.println(F("# upright"));
      }
    } else {
      uprightSinceMs = 0;
      if ((uint32_t)(nowMs - lastTipMsgMs) >= TIP_REPEAT_MS) sendTip();
    }
  }
}

// ---- Arduino entry points --------------------------------------------------------
void setup() {
#if DRIVER_TYPE == DRIVER_EN_IN
  pinMode(PIN_L_EN, OUTPUT);
  pinMode(PIN_R_EN, OUTPUT);
#endif
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  for (uint8_t i = 0; i < 2; i++) {
    for (uint8_t k = 0; k < 4; k++) pinMode(PIN_ARM[i][k], OUTPUT);
    arms[i].pos = arms[i].target = 0;
    arms[i].phase = 0;
    arms[i].intervalUs = (uint32_t)(1000000.0f / (ARM_DEFAULT_DPS * stepsPerDeg()));
    armRelease(i);
  }
  driveStop();

  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);  // a loose I2C wire must never hang loop()
#endif
  if (accelInit()) {
    Serial.print(accelType == ACCEL_MPU6050 ? F("# mpu6050 at 0x") : F("# adxl345 at 0x"));
    Serial.println(accelAddr, HEX);
    captureUpright();  // boot it standing up, or send U later
  } else {
    Serial.println(F("# no accelerometer found, tip/bump disabled"));
  }
  Serial.println(F("READY"));
}

void loop() {
  serialPoll();
  watchdogPoll();
  armPoll(0);
  armPoll(1);
  accelPoll();
}
