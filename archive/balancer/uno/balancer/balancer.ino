/*
 * Two-wheel self-balancer: Arduino Uno balance loop.
 *
 *   MPU6050 (I2C, A4/A5) -> complementary filter -> PID on pitch -> H-bridge -> 2 DC motors
 *   Pi 5 (QNX supervisor) or a laptop/Serial Monitor -> USB serial 115200 -> this Uno
 *
 * The loop runs entirely here at 200 Hz. The Pi only steers, tunes and supervises.
 * It stands up on its own with nothing connected: power on, hold it upright and
 * still for 1 s, let go.
 *
 * Serial protocol (ASCII lines, '\n' or '\r' terminated, lowercase ok):
 *   Host -> Uno
 *     T <turn>          steering, -1..1 (+ = turn right). Watchdog: 0 after 300 ms
 *     V <lean_deg>      lean bias in degrees, clamped +-MAX_LEAN_BIAS (+ = drift
 *                       forward). Watchdog: 0 after 300 ms. No encoders, so this is
 *                       "lean a bit", not a real speed.
 *     G kp ki kd [ks]   live gains (see "Gains" below)
 *     Z                 robot held still at its balance point: recalibrate gyro bias
 *                       and set "upright" to the current angle (~0.5 s, motors off)
 *     W                 save gains + upright offset to EEPROM (survive reset)
 *     S                 stop + disarm (motors off until A)
 *     A                 arm (it starts balancing once held upright for 1 s)
 *     X [0|1]           1 = external tilt mode (iPhone path), 0 = onboard MPU6050
 *     P <pitch> <rate>  external tilt in deg and deg/s (+ = leaning forward),
 *                       only used in X 1 mode, must arrive at least every 100 ms
 *     ?                 print config
 *   Uno -> host
 *     READY                     once after boot
 *     UP                        started balancing
 *     FALL <pitch>              tipped past CUTOFF_DEG, motors off
 *     STALE                     X mode: P lines stopped, motors off
 *     NOIMU                     no MPU6050 found (use X mode or fix wiring)
 *     # t <st> <pitch> <rate> <u> <acc_pitch>   telemetry at 20 Hz
 *           st: D disarmed, W waiting upright, B balancing, F fallen/stale,
 *           pitch = filtered pitch minus upright offset (deg), rate (deg/s),
 *           u = PID output (-1..1), acc_pitch = accel-only pitch (deg, raw)
 *     # ...                     any other comment line (acks, config dump)
 *
 * Gains (output u is motor duty -1..1):
 *   u = kp*e + ki*integral(e) + kd*rate,   e = pitch - target (deg)
 *   target = V - ks*speed_est, speed_est = low-pass of u (no encoders: the duty we
 *   have been applying is our best guess of how fast the wheels are going).
 *   ks = 0 to start; raise it to stop the slow run-away drift.
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <math.h>

// =============================================================================
// CONFIG: edit this block on the bench.
// =============================================================================

// ---- Motor driver (same block as goalie.ino) -----------------------------------
//   DRIVER_2PWM  : L9110S / HG7881 / DRV8833 / MX1508: 4 inputs, no enable pins.
//   DRIVER_EN_IN : L298N (remove ENA/ENB jumpers), TB6612FNG (tie STBY high):
//                  PWM enable + 2 direction pins per channel.
#define DRIVER_2PWM  1
#define DRIVER_EN_IN 2
#define DRIVER_TYPE  DRIVER_2PWM

// Pins 5/6 (Timer0) are avoided: they share the timer with millis()/micros().
#if DRIVER_TYPE == DRIVER_2PWM
const uint8_t PIN_L_IN1 = 9;    // channel A = LEFT wheel
const uint8_t PIN_L_IN2 = 10;
const uint8_t PIN_R_IN1 = 3;    // channel B = RIGHT wheel
const uint8_t PIN_R_IN2 = 11;
#elif DRIVER_TYPE == DRIVER_EN_IN
const uint8_t PIN_L_EN  = 9;
const uint8_t PIN_L_IN1 = 7;
const uint8_t PIN_L_IN2 = 8;
const uint8_t PIN_R_EN  = 3;
const uint8_t PIN_R_IN1 = 4;
const uint8_t PIN_R_IN2 = 12;
#else
#error "DRIVER_TYPE must be DRIVER_2PWM or DRIVER_EN_IN"
#endif

// Flip if "+duty" spins that wheel backwards (or swap its two motor wires).
const bool  LEFT_INVERT  = false;
const bool  RIGHT_INVERT = false;
// |u| below DEADBAND -> motors off. Above it, |u| maps to MIN_DUTY..1 so tiny
// corrections still overcome gear slop + static friction. Measure MIN_DUTY on the
// bench: raise duty until the wheel just starts turning (TUNING.md step 3).
const float DEADBAND = 0.005f;
const float MIN_DUTY = 0.10f;
const float TURN_MAX = 0.25f;           // duty added/subtracted per side at T = +-1

// ---- IMU mounting ----------------------------------------------------------------
// Axis indices: 0 = X, 1 = Y, 2 = Z as printed on the GY-521 board.
// Default: board stands VERTICAL on the body, X arrow pointing UP, Z (the chip
// face) pointing FORWARD. Then pitch = atan2(-acc_fwd, acc_up), rate = -gyro Y
// (leaning forward, gravity shows up as NEGATIVE on the forward axis).
// Check with telemetry (TUNING.md step 2): lean forward -> acc_pitch AND pitch go
// positive, rate positive while tipping forward. Fix with the axis/sign below.
const uint8_t ACC_FWD_AXIS = 2;  const int8_t ACC_FWD_SIGN = 1;
const uint8_t ACC_UP_AXIS  = 0;  const int8_t ACC_UP_SIGN  = 1;
const uint8_t GYRO_AXIS    = 1;  const int8_t GYRO_SIGN    = -1;

// ---- Loop / filter / safety ------------------------------------------------------
const uint16_t LOOP_US          = 5000;    // 200 Hz
const float    COMP_ALPHA       = 0.98f;   // gyro weight (time constant ~0.25 s)
const float    CUTOFF_DEG       = 35.0f;   // fallen beyond this
const float    UPRIGHT_DEG      = 5.0f;    // must be within this ...
const uint16_t UPRIGHT_HOLD_MS  = 1000;    // ... for this long to (re)start
const uint16_t CMD_WATCHDOG_MS  = 300;     // T/V older than this -> 0
const uint16_t EXT_STALE_MS     = 100;     // X mode: no P for this long -> stop
const float    MAX_LEAN_BIAS    = 5.0f;    // |V| clamp, deg
const float    I_LIMIT          = 0.5f;    // |ki*integral| clamp (duty)
const float    SPEED_TAU_S      = 0.5f;    // speed_est low-pass time constant
const uint16_t TELEM_MS         = 50;      // 20 Hz
const bool     AUTO_ARM         = true;    // armed at boot (stand it up and go)

// ---- Default gains (overridden by EEPROM after W) ----------------------------------
// From balancer/sim (DC gearmotor model, 0.9 kg, CoM 0.25 m, 90 mm wheels).
const float DEF_KP = 0.060f;   // duty per deg
const float DEF_KI = 0.000f;   // duty per deg*s
const float DEF_KD = 0.0030f;  // duty per deg/s
const float DEF_KS = 0.0f;     // deg of lean per unit of speed_est

const long SERIAL_BAUD = 115200;

// =============================================================================
// End of CONFIG
// =============================================================================

const uint8_t MPU_ADDR_0 = 0x68, MPU_ADDR_1 = 0x69;
const float ACC_LSB_PER_G   = 8192.0f;   // +-4 g
const float GYRO_LSB_PER_DPS = 65.5f;    // +-500 deg/s
const float DT = LOOP_US / 1e6f;
const uint32_t EE_MAGIC = 0xBA1A0001UL;

enum State : uint8_t { ST_DISARMED, ST_WAITING, ST_BALANCING, ST_FALLEN };

struct Saved { uint32_t magic; float kp, ki, kd, ks, offset; };

// Forward declarations (arduino-cli generates these, a host C++ build doesn't).
void handleLine(char *line);
void serialPoll();
bool imuInit();
bool imuRead(float *accFwd, float *accUp, float *gyro);
void calibrate(bool setOffset);
void motorsOff();
void setMotors(float l, float r);
void controlStep();
void telemetry();

// ---- State ------------------------------------------------------------------------
static char     lineBuf[40];
static uint8_t  lineLen = 0;
static bool     lineOverflow = false;

static float kp = DEF_KP, ki = DEF_KI, kd = DEF_KD, ks = DEF_KS;
static float uprightOffset = 0;   // deg, raw pitch when balanced
static float gyroBias = 0;        // deg/s

static uint8_t  mpuAddr = 0;
static bool     extMode = false;
static float    extPitch = 0, extRate = 0;
static uint32_t lastPMs = 0;
static bool     extEverSeen = false;

static float    pitchRaw = 0;     // filtered, before offset
static float    accPitch = 0;
static float    pitch = 0;        // filtered - offset
static float    rate = 0;
static bool     filterSeeded = false;

static State    state = ST_DISARMED;
static bool     armed = false;
static uint32_t uprightSinceMs = 0;
static bool     uprightTiming = false;

static float    integ = 0;
static float    speedEst = 0;
static float    u = 0;
static float    turnCmd = 0, leanCmd = 0;
static uint32_t lastCmdMs = 0;

static uint32_t lastLoopUs = 0;
static uint32_t lastTelemMs = 0;

// ---- Motors --------------------------------------------------------------------------
static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void motorWrite(uint8_t ch, float duty) {
  if (duty > 1) duty = 1;
  if (duty < -1) duty = -1;
  uint8_t pwm = (uint8_t)((duty < 0 ? -duty : duty) * 255.0f + 0.5f);
#if DRIVER_TYPE == DRIVER_2PWM
  uint8_t in1 = ch == 0 ? PIN_L_IN1 : PIN_R_IN1;
  uint8_t in2 = ch == 0 ? PIN_L_IN2 : PIN_R_IN2;
  if (pwm == 0) {                // coast (both low): free wheel when off
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
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
    digitalWrite(en, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, duty > 0 ? HIGH : LOW);
    digitalWrite(in2, duty > 0 ? LOW : HIGH);
    analogWrite(en, pwm);
  }
#endif
}

// Deadband + minimum-duty kick (hides gearbox slop / stiction).
static float shape(float d) {
  float m = d < 0 ? -d : d;
  if (m < DEADBAND) return 0.0f;
  if (m > 1) m = 1;
  float out = MIN_DUTY + (1.0f - MIN_DUTY) * m;
  return d < 0 ? -out : out;
}

void setMotors(float l, float r) {
  float sl = shape(l), sr = shape(r);
  motorWrite(0, LEFT_INVERT ? -sl : sl);
  motorWrite(1, RIGHT_INVERT ? -sr : sr);
}

void motorsOff() {
  motorWrite(0, 0);
  motorWrite(1, 0);
}

// ---- IMU ----------------------------------------------------------------------------
static bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool imuInit() {
  uint8_t addr = 0;
  if (probe(MPU_ADDR_0)) addr = MPU_ADDR_0;
  else if (probe(MPU_ADDR_1)) addr = MPU_ADDR_1;
  if (!addr) return false;
  // Wake (PLL X gyro), DLPF ~44 Hz, gyro +-500 dps, accel +-4 g.
  if (!i2cWrite(addr, 0x6B, 0x01)) return false;
  if (!i2cWrite(addr, 0x1A, 0x03)) return false;
  if (!i2cWrite(addr, 0x1B, 0x08)) return false;
  if (!i2cWrite(addr, 0x1C, 0x08)) return false;
  mpuAddr = addr;
  return true;
}

// accFwd/accUp in g, gyro (pitch rate, bias NOT removed) in deg/s.
bool imuRead(float *accFwd, float *accUp, float *gyro) {
  if (!mpuAddr) return false;
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpuAddr, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (uint8_t i = 0; i < 14; i++) b[i] = (uint8_t)Wire.read();
  int16_t a[3], g[3];
  for (uint8_t i = 0; i < 3; i++) {
    a[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    g[i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]);
  }
  *accFwd = ACC_FWD_SIGN * a[ACC_FWD_AXIS] / ACC_LSB_PER_G;
  *accUp  = ACC_UP_SIGN  * a[ACC_UP_AXIS]  / ACC_LSB_PER_G;
  *gyro   = GYRO_SIGN    * g[GYRO_AXIS]    / GYRO_LSB_PER_DPS;
  return true;
}

static const float RAD2DEG = 57.2957795f;

// Blocking (~0.5 s), motors off. setOffset: also take the current angle as upright.
void calibrate(bool setOffset) {
  motorsOff();
  if (extMode) {
    if (setOffset && extEverSeen) uprightOffset = extPitch;
    return;
  }
  if (!mpuAddr) return;
  float gs = 0, ps = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < 100; i++) {
    float f, up, g;
    if (imuRead(&f, &up, &g)) {
      gs += g;
      ps += atan2f(-f, up) * RAD2DEG;
      n++;
    }
    delay(5);
  }
  if (!n) return;
  gyroBias = gs / n;
  if (setOffset) uprightOffset = ps / n;
  pitchRaw = ps / n;
  filterSeeded = true;
}

static void loadEeprom() {
  Saved s;
  EEPROM.get(0, s);
  if (s.magic != EE_MAGIC) return;
  if (s.kp != s.kp || s.ki != s.ki || s.kd != s.kd || s.ks != s.ks || s.offset != s.offset) return;
  kp = s.kp; ki = s.ki; kd = s.kd; ks = s.ks; uprightOffset = s.offset;
}

static void saveEeprom() {
  Saved s = {EE_MAGIC, kp, ki, kd, ks, uprightOffset};
  EEPROM.put(0, s);
}

// ---- Parsing ------------------------------------------------------------------------
// Parse up to maxN floats; returns how many, or -1 on garbage.
static int parseFloats(const char *s, float *out, int maxN) {
  int n = 0;
  for (;;) {
    while (*s == ' ' || *s == '\t') s++;
    if (!*s) return n;
    if (n >= maxN) return -1;
    char *end;
    double v = strtod(s, &end);
    if (end == s || v != v) return -1;
    out[n++] = (float)v;
    s = end;
    if (*s && *s != ' ' && *s != '\t') return -1;
  }
}

static void printConfig() {
  Serial.print(F("# kp ")); Serial.print(kp, 4);
  Serial.print(F(" ki ")); Serial.print(ki, 4);
  Serial.print(F(" kd ")); Serial.print(kd, 5);
  Serial.print(F(" ks ")); Serial.print(ks, 3);
  Serial.print(F(" offset ")); Serial.print(uprightOffset, 2);
  Serial.print(F(" bias ")); Serial.print(gyroBias, 2);
  Serial.print(F(" ext ")); Serial.print(extMode ? 1 : 0);
  Serial.print(F(" imu ")); Serial.println(mpuAddr ? 1 : 0);
}

static void enterFallen() {
  state = ST_FALLEN;
  motorsOff();
  integ = 0;
  speedEst = 0;
  u = 0;
  uprightTiming = false;
}

void handleLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32;
  if (!cmd) return;
  const char *arg = line + 1;
  if (*arg && *arg != ' ' && *arg != '\t') return;
  float v[4];
  int n = parseFloats(arg, v, 4);
  if (n < 0) return;
  uint32_t now = millis();

  switch (cmd) {
    case 'T':
      if (n != 1) return;
      turnCmd = clampf(v[0], -1, 1);
      lastCmdMs = now;
      break;
    case 'V':
      if (n != 1) return;
      leanCmd = clampf(v[0], -MAX_LEAN_BIAS, MAX_LEAN_BIAS);
      lastCmdMs = now;
      break;
    case 'G':
      if (n < 3) return;
      kp = v[0]; ki = v[1]; kd = v[2];
      if (n == 4) ks = v[3];
      integ = 0;
      printConfig();
      break;
    case 'Z':
      if (n != 0) return;
      if (state == ST_BALANCING) enterFallen();
      calibrate(true);
      if (state == ST_FALLEN) state = armed ? ST_WAITING : ST_DISARMED;
      uprightTiming = false;
      printConfig();
      break;
    case 'W':
      if (n != 0) return;
      saveEeprom();
      Serial.println(F("# saved"));
      break;
    case 'S':
      if (n != 0) return;
      armed = false;
      enterFallen();
      state = ST_DISARMED;
      turnCmd = leanCmd = 0;
      break;
    case 'A':
      if (n != 0) return;
      armed = true;
      if (state == ST_DISARMED) { state = ST_WAITING; uprightTiming = false; }
      break;
    case 'X': {
      if (n > 1) return;
      bool want = n == 0 ? true : v[0] != 0;
      if (want != extMode) {
        if (state == ST_BALANCING) enterFallen();
        extMode = want;
        extEverSeen = false;
        filterSeeded = false;
      }
      Serial.print(F("# ext ")); Serial.println(extMode ? 1 : 0);
      break;
    }
    case 'P':
      if (n != 2) return;
      extPitch = v[0];
      extRate = v[1];
      lastPMs = now;
      extEverSeen = true;
      break;
    case '?':
      printConfig();
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
    } else if (c < 0x20 || c > 0x7E || lineLen >= sizeof(lineBuf) - 1) {
      lineOverflow = true;
    } else {
      lineBuf[lineLen++] = (char)c;
    }
  }
}

// ---- Control ------------------------------------------------------------------------
// One 200 Hz tick: estimate pitch, run the state machine and the PID.
void controlStep() {
  uint32_t now = millis();

  // 1) Pitch estimate
  bool ok = false;
  if (extMode) {
    if (extEverSeen && (uint32_t)(now - lastPMs) <= EXT_STALE_MS) {
      pitchRaw = extPitch;
      rate = extRate;
      accPitch = extPitch;
      ok = true;
    } else if (state == ST_BALANCING) {
      enterFallen();
      Serial.println(F("STALE"));
    }
  } else {
    float f, up, g;
    if (imuRead(&f, &up, &g)) {
      accPitch = atan2f(-f, up) * RAD2DEG;
      rate = g - gyroBias;
      if (!filterSeeded) { pitchRaw = accPitch; filterSeeded = true; }
      else pitchRaw = COMP_ALPHA * (pitchRaw + rate * DT) + (1.0f - COMP_ALPHA) * accPitch;
      ok = true;
    } else if (state == ST_BALANCING) {
      enterFallen();          // lost the sensor: never drive blind
      Serial.println(F("NOIMU"));
    }
  }
  pitch = pitchRaw - uprightOffset;

  // 2) Command watchdog: steering/lean only, balancing continues
  if ((uint32_t)(now - lastCmdMs) > CMD_WATCHDOG_MS) { turnCmd = 0; leanCmd = 0; }

  // 3) State machine
  float ap = pitch < 0 ? -pitch : pitch;
  if (state == ST_BALANCING) {
    if (ap > CUTOFF_DEG) {
      enterFallen();
      Serial.print(F("FALL "));
      Serial.println(pitch, 1);
    }
  } else if (state == ST_WAITING || state == ST_FALLEN) {
    if (!armed) state = ST_DISARMED;
    else if (ok && ap < UPRIGHT_DEG) {
      if (!uprightTiming) { uprightTiming = true; uprightSinceMs = now; }
      else if ((uint32_t)(now - uprightSinceMs) >= UPRIGHT_HOLD_MS) {
        state = ST_BALANCING;
        integ = 0; speedEst = 0; u = 0;
        Serial.println(F("UP"));
      }
    } else {
      uprightTiming = false;
    }
  }

  if (state != ST_BALANCING) { motorsOff(); u = 0; return; }

  // 4) PID (+ encoder-less speed loop)
  float target = leanCmd - ks * speedEst;
  float e = pitch - target;
  if (ki != 0) {
    integ += e * DT;
    float lim = I_LIMIT / (ki < 0 ? -ki : ki);
    integ = clampf(integ, -lim, lim);
  }
  u = clampf(kp * e + ki * integ + kd * rate, -1, 1);
  speedEst += (u - speedEst) * (DT / SPEED_TAU_S);

  float t = turnCmd * TURN_MAX;
  setMotors(u + t, u - t);
}

static char stateChar() {
  switch (state) {
    case ST_DISARMED: return 'D';
    case ST_WAITING: return 'W';
    case ST_BALANCING: return 'B';
    default: return 'F';
  }
}

void telemetry() {
  uint32_t now = millis();
  if ((uint32_t)(now - lastTelemMs) < TELEM_MS) return;
  lastTelemMs = now;
  if (Serial.availableForWrite() < 40) return;   // never block the loop on TX
  Serial.print(F("# t "));
  Serial.print(stateChar());
  Serial.print(' ');
  Serial.print(pitch, 2);
  Serial.print(' ');
  Serial.print(rate, 1);
  Serial.print(' ');
  Serial.print(u, 3);
  Serial.print(' ');
  Serial.println(accPitch, 1);
}

// ---- Arduino entry points ---------------------------------------------------------------
void setup() {
#if DRIVER_TYPE == DRIVER_EN_IN
  pinMode(PIN_L_EN, OUTPUT);
  pinMode(PIN_R_EN, OUTPUT);
#endif
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  motorsOff();

  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);  // a loose I2C wire must never hang loop()
#endif
  loadEeprom();
  if (imuInit()) {
    Serial.print(F("# mpu6050 at 0x"));
    Serial.println(mpuAddr, HEX);
    calibrate(false);   // gyro bias only: keep it still for 0.5 s after power-on
  } else {
    Serial.println(F("NOIMU"));
    Serial.println(F("# no MPU6050 at 0x68/0x69: check A4/A5, or use X mode"));
  }
  armed = AUTO_ARM;
  state = armed ? ST_WAITING : ST_DISARMED;
  printConfig();
  Serial.println(F("READY"));
  lastLoopUs = micros();
}

void loop() {
  serialPoll();
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) >= LOOP_US) {
    lastLoopUs += LOOP_US;
    if ((uint32_t)(nowUs - lastLoopUs) >= LOOP_US) lastLoopUs = nowUs;  // overran: resync
    controlStep();
    telemetry();
  }
}
