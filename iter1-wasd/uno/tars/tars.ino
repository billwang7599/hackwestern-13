/*
 * TARS: Arduino Uno wheel/arm/sensor/IO co-processor.
 *
 *   Pi 5 (QNX tars_motor -b uno) --USB serial 115200 8N1--> this Uno
 *     CAR = CAR_SM11 (default): Kuman SM11 smart car kit, DC TT gearmotors on the
 *       kit's L298N-style board (EN + IN1/IN2 per side), kit button/LED/buzzer.
 *       !! UNPLUG THE KIT'S BLUETOOTH MODULE: it sits on D0/D1, which is the USB
 *       !! serial link to the Pi. With it plugged in the Pi's commands get garbled.
 *       Pin map + sources: docs/PI_TEST.md "Kuman SM11".
 *     CAR = CAR_CUSTOM: choose WHEEL_TYPE / DRIVER_TYPE yourself:
 *       WHEEL_TYPE = WHEELS_STEPPER (the custom default):
 *         -> ULN2003 #W0 -> 28BYJ-48 left wheel
 *         -> ULN2003 #W1 -> 28BYJ-48 right wheel
 *       WHEEL_TYPE = WHEELS_DC:
 *         -> H-bridge ch A/B -> left/right DC gearmotor
 *     -> ULN2003 #0/#1 -> 28BYJ-48 arms (ARM_COUNT, see the pin map)
 *     <- accelerometer (I2C: ADXL345 / GY-291, or MPU6050 / GY-521) -> tip + bump
 *     <- TEACH push button, -> status LED, -> buzzer (on-robot UI, no laptop)
 *
 * The Uno is deliberately dumb: gestures and vision run on the Pi. The Uno only
 * does things that must be local and fast: stepping, ramps, exact moves, the
 * drive watchdog, the tip/bump safety stop, and the button/LED/buzzer.
 *
 * Protocol (ASCII lines, '\n' or '\r' terminated):
 *   Pi -> Uno
 *     D <l> <r>            drive, floats in [-1, 1] = fraction of max wheel speed.
 *                          Must be resent < 200 ms (watchdog); the Pi sends it
 *                          every 20 ms tick. Stepper wheels ramp (WHEEL_ACCEL_SPS2).
 *                          DC: |d| maps to duty DC_MIN_DUTY..DC_MAX_DUTY (the speed
 *                          cap), with a DC_KICK_MS kick from rest.
 *     M <mm> [speed]       drive straight <mm> (+ fwd, - back, clamped
 *                          +-MOVE_MAX_MM), speed = fraction of max (0.1..1).
 *                          Self-terminating -> "DONE M <mm>".
 *                          DC (no encoders): TIMED open loop at D 1 for
 *                          |mm| / mm_s (see V); speed is accepted but ignored.
 *     R <deg> [speed]      spin in place <deg>, + = clockwise seen from above
 *                          (turn RIGHT), - = left. Clamped +-TURN_MAX_DEG.
 *                          -> "DONE R <deg>". DC: timed, |deg| / deg_s; with
 *                          speed < 0.99 a SLOW turn: DC_SLOW_TURN_DUTY (after the
 *                          kick), timed |deg| / deg_s_slow (tars_motor's slow pivot).
 *                          During M/R: "D 0 0" lines are keepalives (ignored);
 *                          a non-zero D, S, a new M/R, tip-over, or (if the Pi was
 *                          streaming D when the move began) 500 ms of serial
 *                          silence abort it -> "ABORT <M|R> <done> <why>".
 *                          DC reports the amount it was TOLD to do (by time), not
 *                          what the floor did: the Pi re-plans from the iPhone pose.
 *     K [diam_mm track_mm] stepper only: set + save (EEPROM) wheel diameter and
 *                          track width; no args = print them.
 *     V [mm_s deg_s [deg_s_slow]]  DC only: set + save (EEPROM) the speeds measured
 *                          at D 1 (straight mm/s, spin deg/s) and the slow-turn spin
 *                          deg/s (omitted = keep it); no args = print them.
 *                          Answered with the WHEELS lines. Stepper builds ignore it.
 *     S                    stop wheels NOW, abort any M/R, halt arms
 *     A <arm> <deg> [dps]  arm 0|1 to absolute angle (only arms < ARM_COUNT)
 *     H / Z / X            arms home / zero here / halt
 *     U                    "this is upright": recapture the tilt reference
 *     L <p>                status LED: 0 off, 1 on, 2 slow blink, 3 fast blink
 *     B <n>                n short beeps (0 = silence, max 9), non-blocking
 *     ?                    print a "# ..." status line (bench debugging)
 *   Uno -> Pi
 *     WHEELS DC <mm_s> <deg_s>   DC builds: at boot (before READY) and after V.
 *                          tars_motor uses it for pose + move estimates. Stepper
 *                          builds never send it (the Pi assumes steppers).
 *     WHEELS DCSLOW <deg_s_slow> right after every WHEELS DC: the slow-turn rate
 *                          (a separate line so older Pi code, which ignores it,
 *                          still parses WHEELS DC).
 *     READY                once after boot (mid-run = the Uno rebooted)
 *     T <deg>              tipped (repeated every TIP_REPEAT_MS while tipped)
 *     B <mg>               bump (at most one per 250 ms)
 *     DONE <M|R> <amount>  exact move finished (mm or deg actually moved)
 *     ABORT <M|R> <amount> <why>   why = stop|drive|replaced|tipped|link
 *     BTN TEACH            TEACH button short press (released before 2 s)
 *     BTN FORGET           TEACH button held 2 s (sent while still held)
 *     # ...                diagnostics. The Pi ignores lines starting with '#'.
 *
 * Bench test: Serial Monitor at 115200, line ending "Newline". Try "M 100",
 * "R 90", "R -90", "K" (or "V" on DC), "L 2", "B 3", "?". "D 0.5 0.5" only runs 200 ms.
 * NOT compiled for AVR yet (no arduino-cli on the dev Mac): only the host tests
 * in iter1-wasd/uno/test have built it.
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <stdlib.h>
#include <math.h>

// =============================================================================
// CONFIG: edit this block on the bench. Everything else should not need changes.
// =============================================================================

// ---- Which robot ----------------------------------------------------------------
//   CAR_SM11   : Kuman SM11 kit (the default). Sets WHEEL_TYPE, DRIVER_TYPE, pins.
//   CAR_CUSTOM : the hand-built TARS; pick WHEEL_TYPE / DRIVER_TYPE below
//                (default: 28BYJ-48 stepper wheels).
// Passing -DWHEEL_TYPE=... or -DDRIVER_TYPE=... (the host tests do) means CUSTOM.
#define WHEELS_DC      1
#define WHEELS_STEPPER 2
#define DRIVER_2PWM    1
#define DRIVER_EN_IN   2
#define CAR_CUSTOM     0
#define CAR_SM11       1
#ifndef CAR
#if defined(WHEEL_TYPE) || defined(DRIVER_TYPE)
#define CAR CAR_CUSTOM
#else
#define CAR CAR_SM11
#endif
#endif
#if CAR == CAR_SM11
#if (defined(WHEEL_TYPE) && WHEEL_TYPE != WHEELS_DC) || (defined(DRIVER_TYPE) && DRIVER_TYPE != DRIVER_EN_IN)
#error "CAR_SM11 is DC wheels on an EN+IN driver; use CAR=CAR_CUSTOM for anything else"
#endif
#ifndef WHEEL_TYPE
#define WHEEL_TYPE  WHEELS_DC
#endif
#ifndef DRIVER_TYPE
#define DRIVER_TYPE DRIVER_EN_IN
#endif
#ifndef ARM_COUNT
#define ARM_COUNT   0          // no free pins worth having: the kit's sensors use them
#endif
#if ARM_COUNT != 0
#error "CAR_SM11 has no pins for arms (ARM_COUNT 0)"
#endif
#endif

// ---- What drives the wheels (CAR_CUSTOM) ------------------------------------------
#ifndef WHEEL_TYPE
#define WHEEL_TYPE WHEELS_STEPPER
#endif

// ---- Arms --------------------------------------------------------------------
// An Uno has 16 free I/O pins (D2-D13, A0-A3; D0/D1 = serial, A4/A5 = I2C).
// Stepper wheels (8) + button + LED + buzzer leave room for ONE arm (A0-A3).
#ifndef ARM_COUNT
#if WHEEL_TYPE == WHEELS_STEPPER
#define ARM_COUNT 0
#else
#define ARM_COUNT 2
#endif
#endif
#if WHEEL_TYPE == WHEELS_STEPPER && ARM_COUNT > 1
#error "stepper wheels leave pins for at most one arm (ARM_COUNT 0 or 1)"
#endif

// ---- DC motor driver type (WHEELS_DC only) ------------------------------------
//   DRIVER_2PWM  : L9110S / HG7881, DRV8833 (tie nSLEEP high), MX1508.
//   DRIVER_EN_IN : L298N (REMOVE the ENA/ENB jumpers), TB6612FNG (tie STBY high).
#ifndef DRIVER_TYPE
#define DRIVER_TYPE  DRIVER_2PWM
#endif

const uint8_t NO_PIN = 255;

// ---- Pins ---------------------------------------------------------------------
#if CAR == CAR_SM11
// Kuman SM11 stock wiring, from github.com/mbof/sm11 sm11_pinout.h (Motor(enable,
// go, back): go HIGH = forward). Left EN on D10 is why the Servo library (Timer1 =
// D9/D10 PWM) kills left speed control: we don't use the servo.
//   D2 IR receiver, D4 right LED, A0/A1 ultrasonic trig/echo, A2/A3 line sensors:
//   left plugged in, unused. A4/A5 IR proximity sensors: UNPLUG them if you fit an
//   MPU6050/ADXL345 (A4/A5 are I2C SDA/SCL); without one, tip/bump is just off.
const uint8_t PIN_L_EN  = 10;   // ENA/ENB: Timer1 PWM, 490 Hz
const uint8_t PIN_L_IN1 = 8;    // left "go"
const uint8_t PIN_L_IN2 = 9;    // left "back"
const uint8_t PIN_R_EN  = 5;    // Timer0 PWM, 980 Hz (the sides differ: see DC_*_GAIN)
const uint8_t PIN_R_IN1 = 6;    // right "go"
const uint8_t PIN_R_IN2 = 7;    // right "back"
const uint8_t PIN_BUTTON = 13;  // the kit's push button ("KEY", to GND): TEACH button
const uint8_t PIN_LED    = 3;   // the kit's left LED (D13 is the button, can't be an output)
const uint8_t PIN_BUZZER = 12;  // the kit's buzzer: active, HIGH = on (no tone(), no Timer2)
#elif WHEEL_TYPE == WHEELS_STEPPER
const uint8_t PIN_WHEEL[2][4] = {
  {8, 9, 10, 11},               // left wheel:  ULN2003 #W0 IN1..IN4
  {4, 5, 6, 7},                 // right wheel: ULN2003 #W1 IN1..IN4
};
const uint8_t PIN_BUTTON = 2;   // TEACH button to GND (internal pull-up)
const uint8_t PIN_LED    = 13;  // status LED (the Uno's own "L" LED too)
const uint8_t PIN_BUZZER = 12;  // buzzer + to D12, - to GND
// free: D3 (and A0-A3 when ARM_COUNT is 0)
#elif DRIVER_TYPE == DRIVER_2PWM
const uint8_t PIN_L_IN1 = 9;    // L9110S A-IA / DRV8833 AIN1 / MX1508 IN1
const uint8_t PIN_L_IN2 = 10;
const uint8_t PIN_R_IN1 = 3;
const uint8_t PIN_R_IN2 = 11;
const uint8_t PIN_BUTTON = 2;
const uint8_t PIN_LED    = 13;
const uint8_t PIN_BUZZER = 12;
// free: D8
#elif DRIVER_TYPE == DRIVER_EN_IN
const uint8_t PIN_L_EN  = 9;
const uint8_t PIN_L_IN1 = 8;
const uint8_t PIN_L_IN2 = 12;
const uint8_t PIN_R_EN  = 3;
const uint8_t PIN_R_IN1 = 2;
const uint8_t PIN_R_IN2 = 11;
const uint8_t PIN_BUTTON = 10;
const uint8_t PIN_LED    = 13;
const uint8_t PIN_BUZZER = NO_PIN;  // no pin left
#else
#error "DRIVER_TYPE must be DRIVER_2PWM or DRIVER_EN_IN"
#endif

const uint8_t PIN_ARM[2][4] = {
  {A0, A1, A2, A3},             // arm 0: ULN2003 #0 IN1..IN4
  {4, 5, 6, 7},                 // arm 1: ULN2003 #1 IN1..IN4 (DC wheels only)
};

// ---- Stepper wheels (28BYJ-48 + ULN2003, half-step) ----------------------------
// Gear train (32/9)(22/11)(26/9)(31/10) = 63.684:1, NOT the 64:1 on the label,
// so 64 half-steps x 63.684 = 4075.77 half-steps per wheel turn. A few units are
// a true 64:1 (4096). If a 1 m drive is ~5 mm off after diameter calibration,
// try 4096.
const float    WHEEL_STEPS_PER_REV = 4075.7728f;
const float    WHEEL_DIAM_MM_DEFAULT  = 65.0f;   // calibrate: K <diam> <track>
const float    WHEEL_TRACK_MM_DEFAULT = 100.0f;  // centre-to-centre of the tyres
const float    WHEEL_MAX_RPM    = 12.0f;   // 10-15 at 5 V. Lower it if it buzzes/stalls
const float    WHEEL_ACCEL_SPS2 = 3000.0f; // half-steps/s^2: 0 -> 12 RPM in ~0.27 s
// Slowing down (key released, D 0 0, reversing through zero) uses a harder
// ramp: 12 RPM -> 0 in ~0.14 s. The 64:1 gearbox makes the robot's inertia
// tiny at the rotor, so this doesn't lose steps; raise toward 3000 if it does.
const float    WHEEL_DECEL_SPS2 = 6000.0f;
const float    WHEEL_MIN_SPS    = 100.0f;  // crawl speed at the end of an exact move
// The wheels face opposite directions, so one is mirrored: "D 0.5 0.5" should
// roll both forward. Flip on the bench if not.
const bool     WHEEL_INVERT[2]  = {false, true};
const uint16_t WHEEL_RELEASE_MS = 100;     // coils off this long after the last step
const bool     WHEEL_HOLD       = false;   // true: keep coils on when idle (hot)
const float    MOVE_MAX_MM      = 1000.0f;
const float    TURN_MAX_DEG     = 360.0f;
const uint16_t MOVE_LINK_MS     = 500;     // Pi went silent mid-move -> abort

// ---- DC drive tuning (WHEELS_DC only) --------------------------------------------
// D l r: |d| in (DRIVE_DEADBAND, 1] -> duty DC_MIN_DUTY .. DC_MAX_DUTY (linear).
//   DC_MIN_DUTY: lowest duty that keeps a ROLLING wheel turning (TT motors stall below).
//   DC_MAX_DUTY: the SPEED CAP. D 1 = this duty. Everything above the Uno (goto gains,
//     the 0.55-0.6 m lidar_guard stop zone, pet/vision D values) was tuned for the
//     ~41 mm/s steppers; see docs/PI_TEST.md "Kuman SM11" for why the cap is
//     ~150 mm/s and how to check it with "V" after wheel_cal.py.
//   DC_KICK_*: from rest (or reversing) a wheel gets DC_KICK_DUTY for DC_KICK_MS to
//     break static friction, then its real duty. S / watchdog / tip end a kick at once.
// DC_*_GAIN: trim a side that runs faster (the SM11's two EN pins have different PWM
//   frequencies, 490 vs 980 Hz). < 1 slows that side. Check with "M 1000" on the floor.
// DC_SLOW_TURN_DUTY: "R <deg> <speed>" with speed < DC_SLOW_BELOW spins at this duty
//   (after the usual kick) instead of D 1, timed by deg_s_slow. That's the slow pivot
//   tars_motor asks for (TURN speed 0.40: goto's turns, and any TURN allowed while the
//   iPhone guard says BLOCKED). A literal 40 % of the cap would stall TT motors, and an
//   in-place skid-steer turn needs more torque than rolling straight, so it's a fixed
//   duty between DC_MIN_DUTY and the cap. If "R 90 0.4" stalls on the floor, raise it.
#if CAR == CAR_SM11
const bool  LEFT_INVERT   = false;   // flip if "D 0.5 0.5" turns that wheel backwards
const bool  RIGHT_INVERT  = false;
const float DC_MIN_DUTY   = 0.30f;
const float DC_MAX_DUTY   = 0.45f;
const float DC_KICK_DUTY  = 0.80f;
const uint16_t DC_KICK_MS = 40;
const float DC_SLOW_TURN_DUTY = 0.36f;   // GUESS: check with wheel_cal.py dc-spin --slow
#else
const bool  LEFT_INVERT   = false;
const bool  RIGHT_INVERT  = false;
const float DC_MIN_DUTY   = 0.0f;    // custom DC builds: the old linear 0..1 map, no kick
const float DC_MAX_DUTY   = 1.0f;
const float DC_KICK_DUTY  = 0.0f;
const uint16_t DC_KICK_MS = 0;
const float DC_SLOW_TURN_DUTY = 0.40f;
#endif
const float DC_LEFT_GAIN  = 1.0f;
const float DC_RIGHT_GAIN = 1.0f;
const float DC_SLOW_BELOW = 0.99f;   // R speed below this = slow turn (DC builds)
// DC M/R timing: speeds at D 1 (and the slow-turn rate) until "V" saves measured ones.
const float DC_MM_S_DEFAULT  = 150.0f;
const float DC_DEG_S_DEFAULT = 120.0f;
#if CAR == CAR_SM11
const float DC_DEG_S_SLOW_DEFAULT = 60.0f;   // GUESS (duty 0.36 vs the 0.45 cap): measure it
#else
const float DC_DEG_S_SLOW_DEFAULT = 48.0f;   // 0.40 x 120 on the linear custom map: a guess
#endif
const float DRIVE_DEADBAND = 0.03f;  // |d| below this -> off (both wheel types)
const bool  BRAKE_ON_STOP  = true;
const uint16_t DRIVE_WATCHDOG_MS = 200;
const bool  LED_INVERT    = false;   // true if the LED lights when its pin is LOW

// ---- Arms (28BYJ-48 + ULN2003) --------------------------------------------------
const float    ARM_STEPS_PER_REV = 4096.0f;
const float    ARM_MAX_DPS       = 60.0f;
const float    ARM_DEFAULT_DPS   = 45.0f;
const float    ARM_LIMIT_DEG     = 135.0f;
const bool     ARM_INVERT[2]     = {false, true};
const uint16_t ARM_RELEASE_MS    = 20;
const bool     ARM_HOLD          = false;

// ---- Button / LED / buzzer --------------------------------------------------------
const uint16_t BTN_DEBOUNCE_MS = 30;
const uint16_t BTN_LONG_MS     = 2000;   // held this long -> BTN FORGET
const uint16_t LED_SLOW_MS     = 500;    // half-period of pattern 2
const uint16_t LED_FAST_MS     = 100;    // half-period of pattern 3
const uint16_t BEEP_ON_MS      = 80;
const uint16_t BEEP_OFF_MS     = 120;
// false: ACTIVE buzzer (beeps by itself when its pin is HIGH; has a sticker /
// sealed back). true: PASSIVE piezo (open back, needs a tone; uses tone()).
const bool     BUZZER_PASSIVE  = false;
const uint16_t BUZZER_HZ       = 2700;

// ---- Accelerometer -------------------------------------------------------------
const uint8_t  MPU_ADDR_0  = 0x68;
const uint8_t  MPU_ADDR_1  = 0x69;
const uint8_t  ADXL_ADDR_0 = 0x53;
const uint8_t  ADXL_ADDR_1 = 0x1D;
const float    MPU_LSB_PER_G  = 4096.0f;
const float    ADXL_LSB_PER_G = 256.0f;
const uint16_t ACCEL_PERIOD_US    = 5000;     // 200 Hz
const float    TILT_ALPHA         = 0.1f;
const float    TIP_DEG            = 40.0f;
const uint16_t TIP_HOLD_MS        = 150;
const float    UPRIGHT_DEG        = 25.0f;
const uint16_t UPRIGHT_HOLD_MS    = 500;
const uint16_t TIP_REPEAT_MS      = 500;
const uint16_t BUMP_THRESHOLD_MG  = 1200;
const uint16_t BUMP_WINDOW_MS     = 15;
const uint16_t BUMP_RATE_LIMIT_MS = 250;
const float    BASELINE_ALPHA     = 0.005f;

const long SERIAL_BAUD = 115200;

// =============================================================================
// End of CONFIG
// =============================================================================

void driveSet(float l, float r);
void driveStop();
void armsHalt();
void stopAll();
void handleLine(char *line);
void serialPoll();
void watchdogPoll();
bool accelInit();
bool accelRead(int16_t *ax, int16_t *ay, int16_t *az);
void accelPoll();
void captureUpright();
void ioPoll();

// ---- State ----------------------------------------------------------------------
static char     lineBuf[40];
static uint8_t  lineLen = 0;
static bool     lineOverflow = false;

static bool     driveActive = false;
static uint32_t lastDMs = 0;
static bool     dEver = false;           // a D line has been seen since boot
static uint32_t lastLineMs = 0;          // any valid command line

static const uint8_t HALF_STEP[8] = {
  0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,
};

struct Arm {
  long     pos;
  long     target;
  uint8_t  phase;
  bool     energized;
  uint32_t intervalUs;
  uint32_t lastStepUs;
  uint32_t arrivedMs;
};
static Arm arms[2];

#if WHEEL_TYPE == WHEELS_STEPPER
struct Wheel {
  long     pos;          // half-steps since boot (+ = forward for this wheel)
  long     start;        // pos when the current exact move began
  long     target;       // exact-move target
  float    rate;         // current signed half-steps/s (ramped)
  float    vel;          // velocity-mode target, signed half-steps/s
  uint8_t  phase;
  bool     energized;
  uint32_t lastStepUs;
  uint32_t lastActiveMs;
};
static Wheel    wheels[2];
static char     moveKind = 0;            // 0 = velocity mode, 'M' or 'R' = exact move
static float    moveReq = 0;
static float    moveMaxSps = 0;
static bool     moveLinkGuard = false;
static uint32_t lastRampUs = 0;
static float    wheelDiamMm = WHEEL_DIAM_MM_DEFAULT;
static float    wheelTrackMm = WHEEL_TRACK_MM_DEFAULT;

struct WheelCal { uint16_t magic; float diam, track; };
const uint16_t WHEEL_CAL_MAGIC = 0x7A12;
const int      WHEEL_CAL_ADDR  = 0;
#endif

// button / LED / buzzer
static bool     btnRaw = false, btnStable = false;   // true = pressed
static uint32_t btnChangeMs = 0, btnDownMs = 0;
static bool     btnLongSent = false;
static uint8_t  ledPattern = 0;
static uint8_t  beepsLeft = 0;
static bool     beepOn = false;
static uint32_t beepEdgeMs = 0;

#define ACCEL_NONE 0
#define ACCEL_MPU6050 1
#define ACCEL_ADXL345 2
static uint8_t  accelType = ACCEL_NONE;
static uint8_t  accelAddr = 0;
static float    accelLsbPerG = MPU_LSB_PER_G;
static uint32_t lastAccelUs = 0;
static float    upX = 0, upY = 0, upZ = 1;
static float    fX = 0, fY = 0, fZ = 1;
static float    tiltDeg = 0;
static bool     tipped = false;
static uint32_t tipSinceMs = 0;
static uint32_t uprightSinceMs = 0;
static uint32_t lastTipMsgMs = 0;
static float    baselineMg = 1000.0f;
static bool     bumpWindowOpen = false;
static uint32_t bumpWindowStartMs = 0;
static uint16_t bumpPeakMg = 0;
static uint32_t lastBumpMs = 0;
static bool     bumpEverSent = false;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

// =================================================================== wheels: DC
#if WHEEL_TYPE == WHEELS_DC
static uint8_t dutyToPwm(float d) {
  if (d < 0) d = -d;
  if (d > 1) d = 1;
  return (uint8_t)(d * 255.0f + 0.5f);
}

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

// D fraction -> signed duty: deadband, then DC_MIN_DUTY..DC_MAX_DUTY, then side gain.
static float shape(float d, bool invert, float gain) {
  d = clampf(d, -1, 1);
  float mag = d < 0 ? -d : d;
  if (mag < DRIVE_DEADBAND) return 0.0f;
  float out = (DC_MIN_DUTY + (DC_MAX_DUTY - DC_MIN_DUTY) * mag) * gain;
  if (out > 1.0f) out = 1.0f;
  if (d < 0) out = -out;
  return invert ? -out : out;
}

static float    dcDuty[2];               // steady signed duty per channel (0 = braked)
static bool     dcKicking[2];
static uint32_t dcKickStartMs[2];

// Set channel ch to signed duty. From rest or on a reversal: kick first.
static void dcApply(uint8_t ch, float duty) {
  float prev = dcDuty[ch];
  dcDuty[ch] = duty;
  if (duty == 0.0f) {
    dcKicking[ch] = false;
    motorWrite(ch, 0.0f, BRAKE_ON_STOP);
    return;
  }
  bool fromRest = prev == 0.0f || (prev > 0) != (duty > 0);
  if (DC_KICK_MS > 0 && fromRest) {
    float mag = duty < 0 ? -duty : duty;
    float k = mag > DC_KICK_DUTY ? mag : DC_KICK_DUTY;
    dcKicking[ch] = true;
    dcKickStartMs[ch] = millis();
    motorWrite(ch, duty > 0 ? k : -k, false);
  } else if (!dcKicking[ch]) {
    motorWrite(ch, duty, false);
  }
}

static void dcKickPoll() {
  for (uint8_t ch = 0; ch < 2; ch++) {
    if (dcKicking[ch] && (uint32_t)(millis() - dcKickStartMs[ch]) >= DC_KICK_MS) {
      dcKicking[ch] = false;
      motorWrite(ch, dcDuty[ch], BRAKE_ON_STOP);
    }
  }
}

static void dcSet(float l, float r) {
  dcApply(0, shape(l, LEFT_INVERT, DC_LEFT_GAIN));
  dcApply(1, shape(r, RIGHT_INVERT, DC_RIGHT_GAIN));
}

// ---- DC exact moves: timed, open loop (no encoders) ----
static char     moveKind = 0;            // 0 = velocity mode, 'M' or 'R' = timed move
static float    moveReq = 0;
static uint32_t moveStartMs = 0;
static uint32_t moveDurMs = 0;
static bool     moveLinkGuard = false;
static float    dcMmS = DC_MM_S_DEFAULT;
static float    dcDegS = DC_DEG_S_DEFAULT;
static float    dcDegSSlow = DC_DEG_S_SLOW_DEFAULT;

// EEPROM: v1 (0x7DC1) had mmS, degS; v2 (0x7DC2) appends degSSlow. A v1 record still
// loads (slow rate = default), so reflashing doesn't lose a calibration.
struct DcCal { uint16_t magic; float mmS, degS, degSSlow; };
const uint16_t DC_CAL_MAGIC_V1 = 0x7DC1;  // != the stepper's WHEEL_CAL magic
const uint16_t DC_CAL_MAGIC    = 0x7DC2;
const int      DC_CAL_ADDR  = 0;
static bool dcRateOk(float v) { return v >= 20 && v <= 2000; }
static bool dcCalOk(float mmS, float degS) { return dcRateOk(mmS) && dcRateOk(degS); }

// Two lines: "WHEELS DC <mm_s> <deg_s>" (the Pi's parser rejects extra fields, so it
// stays as it was) then "WHEELS DCSLOW <deg_s_slow>" (older Pi code ignores it).
static void dcCalPrint() {
  Serial.print(F("WHEELS DC "));
  Serial.print(dcMmS, 1);
  Serial.print(F(" "));
  Serial.println(dcDegS, 1);
  Serial.print(F("WHEELS DCSLOW "));
  Serial.println(dcDegSSlow, 1);
}

// Amount "done" so far, by time, signed like the request.
static float moveDone() {
  uint32_t el = millis() - moveStartMs;
  if (el >= moveDurMs) return moveReq;
  return moveReq * (float)el / (float)moveDurMs;
}

static void moveFinish(bool ok, const char *why) {
  if (!moveKind) return;
  float done = ok ? moveReq : moveDone();
  char kind = moveKind;
  moveKind = 0;
  dcApply(0, 0.0f);                       // brake first: done or aborted
  dcApply(1, 0.0f);
  Serial.print(ok ? F("DONE ") : F("ABORT "));
  Serial.print(kind);
  Serial.print(F(" "));
  if (ok) {
    Serial.println(done, 1);
  } else {
    Serial.print(done, 1);
    Serial.print(F(" "));
    Serial.println(why);
  }
}

static void moveAbort(const char *why) { moveFinish(false, why); }

// Spin at a fixed duty (not the D map): +1 = clockwise. Invert/gain like shape().
static void dcSpinDuty(float duty, float dir) {
  float l = dir * duty * DC_LEFT_GAIN, r = -dir * duty * DC_RIGHT_GAIN;
  l = clampf(l, -1, 1); r = clampf(r, -1, 1);
  dcApply(0, LEFT_INVERT ? -l : l);
  dcApply(1, RIGHT_INVERT ? -r : r);
}

static void moveStart(char kind, float amount, float speed) {
  // DC: M always runs at D 1. R runs at D 1, or slow (DC_SLOW_TURN_DUTY, timed by
  // deg_s_slow) when speed < DC_SLOW_BELOW: tars_motor's slow pivot.
  bool slow = kind == 'R' && speed > 0 && speed < DC_SLOW_BELOW;
  if (moveKind) moveFinish(false, "replaced");
  if (kind == 'M') amount = clampf(amount, -MOVE_MAX_MM, MOVE_MAX_MM);
  else amount = clampf(amount, -TURN_MAX_DEG, TURN_MAX_DEG);
  float rate = kind == 'M' ? dcMmS : slow ? dcDegSSlow : dcDegS;
  float mag = amount < 0 ? -amount : amount;
  driveActive = false;
  moveKind = kind;
  moveReq = amount;
  moveStartMs = millis();
  moveDurMs = (uint32_t)(mag / rate * 1000.0f + 0.5f);
  moveLinkGuard = dEver && (uint32_t)(millis() - lastDMs) < 1000;   // same rule as steppers
  if (moveDurMs == 0) { moveFinish(true, ""); return; }
  float s = amount < 0 ? -1.0f : 1.0f;
  if (kind == 'M') dcSet(s, s);
  else if (slow) dcSpinDuty(DC_SLOW_TURN_DUTY, s);
  else dcSet(s, -s);                      // R +: left fwd, right back = clockwise
}

void driveSet(float l, float r) {
  bool nonzero = !(fabsf(l) < DRIVE_DEADBAND && fabsf(r) < DRIVE_DEADBAND);
  if (moveKind) {
    if (!nonzero) return;                 // keepalive during a timed move
    moveAbort("drive");
  }
  dcSet(l, r);
  driveActive = dcDuty[0] != 0.0f || dcDuty[1] != 0.0f;
}

// Immediate stop (S, watchdog, tip): abort a move, brake both sides, end any kick.
void driveStop() {
  moveAbort("stop");
  dcApply(0, 0.0f);
  dcApply(1, 0.0f);
  driveActive = false;
}

static void wheelsPoll() {
  if (moveKind) {
    if ((uint32_t)(millis() - moveStartMs) >= moveDurMs) moveFinish(true, "");
    else if (moveLinkGuard && (uint32_t)(millis() - lastLineMs) > MOVE_LINK_MS) moveAbort("link");
  }
  dcKickPoll();
}

static void dcCalLoad() {
  DcCal c;
  EEPROM.get(DC_CAL_ADDR, c);
  dcMmS = DC_MM_S_DEFAULT; dcDegS = DC_DEG_S_DEFAULT; dcDegSSlow = DC_DEG_S_SLOW_DEFAULT;
  if ((c.magic == DC_CAL_MAGIC || c.magic == DC_CAL_MAGIC_V1) && dcCalOk(c.mmS, c.degS)) {
    dcMmS = c.mmS;
    dcDegS = c.degS;
    if (c.magic == DC_CAL_MAGIC && dcRateOk(c.degSSlow)) dcDegSSlow = c.degSSlow;
  }
}

static void wheelsSetup() {
  moveKind = 0;
  moveReq = 0;
  moveLinkGuard = false;
  for (uint8_t i = 0; i < 2; i++) { dcDuty[i] = 0; dcKicking[i] = false; }
  dcCalLoad();
#if DRIVER_TYPE == DRIVER_EN_IN
  pinMode(PIN_L_EN, OUTPUT);
  pinMode(PIN_R_EN, OUTPUT);
#endif
  pinMode(PIN_L_IN1, OUTPUT);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN1, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  driveStop();
}

// =================================================================== wheels: stepper
#else
float wheelMaxSps() { return WHEEL_MAX_RPM * WHEEL_STEPS_PER_REV / 60.0f; }
float wheelStepsPerMm() { return WHEEL_STEPS_PER_REV / (3.14159265f * wheelDiamMm); }

static void wheelRelease(uint8_t i) {
  for (uint8_t k = 0; k < 4; k++) digitalWrite(PIN_WHEEL[i][k], LOW);
  wheels[i].energized = false;
}

static void wheelApply(uint8_t i) {
  uint8_t pat = HALF_STEP[wheels[i].phase];
  for (uint8_t k = 0; k < 4; k++) digitalWrite(PIN_WHEEL[i][k], (pat >> k) & 1 ? HIGH : LOW);
  wheels[i].energized = true;
}

static float shapeSps(float d) {
  d = clampf(d, -1, 1);
  if ((d < 0 ? -d : d) < DRIVE_DEADBAND) return 0.0f;
  return d * wheelMaxSps();
}

// Amount actually moved in the current exact move, in mm (M) or deg (R),
// signed like the request.
static float moveDone() {
  float s = (labs(wheels[0].pos - wheels[0].start) + labs(wheels[1].pos - wheels[1].start)) * 0.5f;
  float mm = s / wheelStepsPerMm();
  float amt = moveKind == 'R' ? mm * 360.0f / (3.14159265f * wheelTrackMm) : mm;
  return moveReq < 0 ? -amt : amt;
}

static void moveFinish(bool ok, const char *why) {
  if (!moveKind) return;
  float done = moveDone();
  Serial.print(ok ? F("DONE ") : F("ABORT "));
  Serial.print(moveKind);
  Serial.print(F(" "));
  if (ok) {
    Serial.println(done, 1);
  } else {
    Serial.print(done, 1);
    Serial.print(F(" "));
    Serial.println(why);
  }
  moveKind = 0;
  for (uint8_t i = 0; i < 2; i++) {
    wheels[i].target = wheels[i].pos;
    wheels[i].vel = 0;
    wheels[i].rate = 0;            // done: already at target; abort: hard stop
  }
}

static void moveAbort(const char *why) { moveFinish(false, why); }

static void moveStart(char kind, float amount, float speed) {
  if (moveKind) moveFinish(false, "replaced");
  if (kind == 'M') amount = clampf(amount, -MOVE_MAX_MM, MOVE_MAX_MM);
  else amount = clampf(amount, -TURN_MAX_DEG, TURN_MAX_DEG);
  if (!(speed > 0)) speed = 1.0f;
  speed = clampf(speed, 0.1f, 1.0f);
  float mm = kind == 'M' ? amount : 3.14159265f * wheelTrackMm * amount / 360.0f;
  long steps = lround(mm * wheelStepsPerMm());
  long d[2] = {steps, kind == 'M' ? steps : -steps};   // R +: left fwd, right back = clockwise
  for (uint8_t i = 0; i < 2; i++) {
    wheels[i].vel = 0;
    wheels[i].start = wheels[i].pos;
    wheels[i].target = wheels[i].pos + d[i];
  }
  driveActive = false;
  moveKind = kind;
  moveReq = amount;
  moveMaxSps = speed * wheelMaxSps();
  // The Pi streams D every 20 ms; if it was doing so, silence mid-move means
  // the Pi died -> abort. Typed on the Serial Monitor: no guard.
  moveLinkGuard = dEver && (uint32_t)(millis() - lastDMs) < 1000;
  if (steps == 0) moveFinish(true, "");
}

void driveSet(float l, float r) {
  float vl = shapeSps(l), vr = shapeSps(r);
  if (moveKind) {
    if (vl == 0.0f && vr == 0.0f) return;   // keepalive during an exact move
    moveAbort("drive");
  }
  wheels[0].vel = vl;
  wheels[1].vel = vr;
  driveActive = vl != 0.0f || vr != 0.0f;
}

// Immediate stop (S, watchdog, tip). Steppers this light stop dead fine.
void driveStop() {
  moveAbort("stop");
  for (uint8_t i = 0; i < 2; i++) { wheels[i].vel = 0; wheels[i].rate = 0; }
  driveActive = false;
}

static float toward(float cur, float tgt, float dv) {
  float d = tgt - cur;
  return d > dv ? cur + dv : d < -dv ? cur - dv : tgt;
}

// Speed up at dvUp, slow down at dvDown. A reversal brakes to exactly 0 first
// (at dvDown), then speeds up the other way on the next ramp ticks.
static float ramp(float cur, float tgt, float dvUp, float dvDown) {
  if ((cur > 0 && tgt < cur && tgt >= 0) || (cur < 0 && tgt > cur && tgt <= 0))
    return toward(cur, tgt, dvDown);                 // same direction, slower
  if ((cur > 0 && tgt < 0) || (cur < 0 && tgt > 0)) {
    float n = toward(cur, 0, dvDown);                // reversing: brake to 0
    return n;
  }
  return toward(cur, tgt, dvUp);                     // from 0 or speeding up
}

static void wheelStep(uint8_t i, uint32_t nowUs) {
  Wheel &w = wheels[i];
  float r = w.rate, mag = r < 0 ? -r : r;
  bool want = mag >= 5.0f;
  if (moveKind) {
    long rem = w.target - w.pos;
    if (rem == 0 || (rem > 0) != (r > 0)) want = false;
  }
  if (!want) {
    if (!WHEEL_HOLD && w.energized && (uint32_t)(millis() - w.lastActiveMs) >= WHEEL_RELEASE_MS)
      wheelRelease(i);
    return;
  }
  w.lastActiveMs = millis();
  if (!w.energized) {               // re-energize on the current phase first
    wheelApply(i);
    w.lastStepUs = nowUs;
    return;
  }
  uint32_t interval = (uint32_t)(1000000.0f / mag);
  uint32_t since = nowUs - w.lastStepUs;
  if (since < interval) return;
  // Keep the average rate exact despite loop jitter, but don't "catch up" a burst.
  w.lastStepUs = since > 2 * interval ? nowUs : w.lastStepUs + interval;
  int8_t dir = r > 0 ? 1 : -1;
  w.pos += dir;
  int8_t pd = WHEEL_INVERT[i] ? -dir : dir;
  w.phase = (uint8_t)((w.phase + 8 + pd) & 7);
  wheelApply(i);
}

static void wheelsPoll() {
  uint32_t nowUs = micros();
  uint32_t dtUs = nowUs - lastRampUs;
  if (dtUs >= 1000) {                // ramp at 1 kHz
    lastRampUs = nowUs;
    if (dtUs > 20000) dtUs = 20000;
    float dv = WHEEL_ACCEL_SPS2 * dtUs * 1e-6f;
    float dvDown = WHEEL_DECEL_SPS2 * dtUs * 1e-6f;
    for (uint8_t i = 0; i < 2; i++) {
      Wheel &w = wheels[i];
      float des = w.vel;
      if (moveKind) {
        long rem = w.target - w.pos;
        long ar = rem < 0 ? -rem : rem;
        if (ar == 0) {
          des = 0;
        } else {
          float v = sqrtf(2.0f * WHEEL_ACCEL_SPS2 * ar);   // can still stop in time
          v = clampf(v, WHEEL_MIN_SPS, moveMaxSps);
          des = rem > 0 ? v : -v;
        }
      }
      w.rate = ramp(w.rate, des, dv, dvDown);
      if (moveKind && w.target == w.pos) w.rate = 0;
    }
  }
  if (moveKind) {
    if (wheels[0].pos == wheels[0].target && wheels[1].pos == wheels[1].target) {
      moveFinish(true, "");
    } else if (moveLinkGuard && (uint32_t)(millis() - lastLineMs) > MOVE_LINK_MS) {
      moveAbort("link");
    }
  }
  wheelStep(0, nowUs);
  wheelStep(1, nowUs);
}

static void wheelCalPrint() {
  Serial.print(F("# wheels diam "));
  Serial.print(wheelDiamMm, 1);
  Serial.print(F(" track "));
  Serial.print(wheelTrackMm, 1);
  Serial.print(F(" steps/rev "));
  Serial.println(WHEEL_STEPS_PER_REV, 1);
}

static void wheelsSetup() {
  for (uint8_t i = 0; i < 2; i++) {
    for (uint8_t k = 0; k < 4; k++) pinMode(PIN_WHEEL[i][k], OUTPUT);
    wheels[i] = Wheel();
    wheelRelease(i);
  }
  moveKind = 0;
  moveReq = 0;
  moveLinkGuard = false;
  lastRampUs = micros();
  WheelCal c;
  EEPROM.get(WHEEL_CAL_ADDR, c);
  if (c.magic == WHEEL_CAL_MAGIC && c.diam >= 10 && c.diam <= 300 && c.track >= 20 && c.track <= 500) {
    wheelDiamMm = c.diam;
    wheelTrackMm = c.track;
  } else {
    wheelDiamMm = WHEEL_DIAM_MM_DEFAULT;
    wheelTrackMm = WHEEL_TRACK_MM_DEFAULT;
  }
}
#endif

// ============================================================================ arms
static float stepsPerDeg() { return ARM_STEPS_PER_REV / 360.0f; }

#if ARM_COUNT > 0
static void armRelease(uint8_t i) {
  for (uint8_t k = 0; k < 4; k++) digitalWrite(PIN_ARM[i][k], LOW);
  arms[i].energized = false;
}

static void armApply(uint8_t i) {
  uint8_t pat = HALF_STEP[arms[i].phase];
  for (uint8_t k = 0; k < 4; k++) digitalWrite(PIN_ARM[i][k], (pat >> k) & 1 ? HIGH : LOW);
  arms[i].energized = true;
}

static void armPoll(uint8_t i) {
  Arm &a = arms[i];
  uint32_t nowUs = micros();
  if (a.pos == a.target) {
    if (!ARM_HOLD && a.energized && (uint32_t)(millis() - a.arrivedMs) >= ARM_RELEASE_MS) armRelease(i);
    return;
  }
  if (!a.energized) {
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
#endif

static void armMove(uint8_t i, float deg, float dps) {
  if (i >= ARM_COUNT) return;
  deg = clampf(deg, -ARM_LIMIT_DEG, ARM_LIMIT_DEG);
  if (!(dps > 0)) dps = ARM_DEFAULT_DPS;
  dps = clampf(dps, 1.0f, ARM_MAX_DPS);
  arms[i].target = lround(deg * stepsPerDeg());
  arms[i].intervalUs = (uint32_t)(1000000.0f / (dps * stepsPerDeg()));
  if (arms[i].pos == arms[i].target) arms[i].arrivedMs = millis();
}

void armsHalt() {
  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    arms[i].target = arms[i].pos;
    arms[i].arrivedMs = millis();
  }
}

void stopAll() {
  driveStop();
  armsHalt();
}

// ====================================================== button / LED / buzzer
static void buzzerWrite(bool on) {
  if (PIN_BUZZER == NO_PIN) return;
  if (BUZZER_PASSIVE) { if (on) tone(PIN_BUZZER, BUZZER_HZ); else noTone(PIN_BUZZER); }
  else digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

static void beepStart(uint8_t n) {
  if (n > 9) n = 9;
  beepsLeft = n;
  beepOn = n > 0;
  beepEdgeMs = millis();
  buzzerWrite(beepOn);
}

void ioPoll() {
  uint32_t now = millis();
  // button: debounced, short press on release, long press while held
  if (PIN_BUTTON != NO_PIN) {
    bool raw = digitalRead(PIN_BUTTON) == LOW;
    if (raw != btnRaw) { btnRaw = raw; btnChangeMs = now; }
    if (btnRaw != btnStable && (uint32_t)(now - btnChangeMs) >= BTN_DEBOUNCE_MS) {
      btnStable = btnRaw;
      if (btnStable) {
        btnDownMs = now;
        btnLongSent = false;
      } else if (!btnLongSent) {
        Serial.println(F("BTN TEACH"));
      }
    }
    if (btnStable && !btnLongSent && (uint32_t)(now - btnDownMs) >= BTN_LONG_MS) {
      btnLongSent = true;
      Serial.println(F("BTN FORGET"));
    }
  }
  // LED
  if (PIN_LED != NO_PIN) {
    bool on = ledPattern == 1 ||
              (ledPattern == 2 && (now / LED_SLOW_MS) % 2 == 0) ||
              (ledPattern == 3 && (now / LED_FAST_MS) % 2 == 0);
    digitalWrite(PIN_LED, on != LED_INVERT ? HIGH : LOW);
  }
  // buzzer
  if (beepsLeft) {
    uint32_t el = now - beepEdgeMs;
    if (beepOn && el >= BEEP_ON_MS) {
      beepOn = false;
      buzzerWrite(false);
      beepEdgeMs = now;
      beepsLeft--;
    } else if (!beepOn && el >= BEEP_OFF_MS) {
      beepOn = true;
      buzzerWrite(true);
      beepEdgeMs = now;
    }
  }
}

static void ioSetup() {
  if (PIN_BUTTON != NO_PIN) pinMode(PIN_BUTTON, INPUT_PULLUP);
  if (PIN_LED != NO_PIN) { pinMode(PIN_LED, OUTPUT); digitalWrite(PIN_LED, LED_INVERT ? HIGH : LOW); }
  if (PIN_BUZZER != NO_PIN) { pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW); }
  btnRaw = btnStable = btnLongSent = false;
  btnChangeMs = btnDownMs = 0;
  ledPattern = 0;
  beepsLeft = 0;
  beepOn = false;
}

// ========================================================================= parsing
static const char *skipSpace(const char *s) {
  while (*s == ' ' || *s == '\t') s++;
  return s;
}

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
  Serial.print(accelType == ACCEL_MPU6050 ? F("mpu6050") : accelType == ACCEL_ADXL345 ? F("adxl345") : F("none"));
#if WHEEL_TYPE == WHEELS_STEPPER
  Serial.print(F(" wheels "));
  Serial.print(wheels[0].pos);
  Serial.print(F(" "));
  Serial.print(wheels[1].pos);
  Serial.print(F(" move "));
  if (moveKind) Serial.println(moveKind); else Serial.println(F("-"));
#else
  Serial.print(F(" wheels dc "));
  Serial.print(dcDuty[0], 2);
  Serial.print(F(" "));
  Serial.print(dcDuty[1], 2);
  Serial.print(F(" move "));
  if (moveKind) Serial.println(moveKind); else Serial.println(F("-"));
#endif
}

void handleLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  char cmd = line[0];
  if (cmd >= 'a' && cmd <= 'z') cmd -= 32;
  if (!cmd) return;
  const char *arg = line + 1;
  if (*arg && *arg != ' ' && *arg != '\t') return;
  float v[3];
  int n = parseFloats(arg, v, 3);
  if (n < 0) return;
  lastLineMs = millis();

  switch (cmd) {
    case 'D':
      if (n != 2) return;
      lastDMs = millis();
      dEver = true;
      if (tipped) { driveStop(); return; }
      driveSet(v[0], v[1]);
      break;
    case 'M':
    case 'R':
      if (n < 1 || n > 2) return;
      if (tipped) {
        Serial.print(F("ABORT ")); Serial.print(cmd); Serial.println(F(" 0.0 tipped"));
        return;
      }
      moveStart(cmd, v[0], n == 2 ? v[1] : 1.0f);
      break;
    case 'V':
#if WHEEL_TYPE == WHEELS_DC
      if (n == 2 || n == 3) {                    // V mm_s deg_s keeps the slow rate
        if (!dcCalOk(v[0], v[1]) || (n == 3 && !dcRateOk(v[2]))) return;
        dcMmS = v[0];
        dcDegS = v[1];
        if (n == 3) dcDegSSlow = v[2];
        DcCal c = {DC_CAL_MAGIC, dcMmS, dcDegS, dcDegSSlow};
        EEPROM.put(DC_CAL_ADDR, c);
      } else if (n != 0) {
        return;
      }
      dcCalPrint();
#endif
      break;
    case 'K':
#if WHEEL_TYPE == WHEELS_STEPPER
      if (n == 2) {
        if (v[0] < 10 || v[0] > 300 || v[1] < 20 || v[1] > 500) return;
        wheelDiamMm = v[0];
        wheelTrackMm = v[1];
        WheelCal c = {WHEEL_CAL_MAGIC, wheelDiamMm, wheelTrackMm};
        EEPROM.put(WHEEL_CAL_ADDR, c);
      } else if (n != 0) {
        return;
      }
      wheelCalPrint();
#endif
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
    case 'L':
      if (n == 1 && v[0] >= 0 && v[0] <= 3 && v[0] == (float)(int)v[0]) ledPattern = (uint8_t)v[0];
      break;
    case 'B':
      if (n == 1 && v[0] >= 0 && v[0] <= 99 && v[0] == (float)(int)v[0]) beepStart((uint8_t)v[0]);
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

// =================================================================== accelerometer
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
  if (!i2cWrite(addr, 0x6B, 0x01)) return false;
  if (!i2cWrite(addr, 0x1C, 0x10)) return false;
  if (!i2cWrite(addr, 0x1A, 0x03)) return false;
  accelType = ACCEL_MPU6050; accelAddr = addr; accelLsbPerG = MPU_LSB_PER_G;
  return true;
}

static bool adxlInit(uint8_t addr) {
  uint8_t id;
  if (!i2cProbe(addr) || !i2cReadReg(addr, 0x00, &id, 1) || id != 0xE5) return false;
  if (!i2cWrite(addr, 0x2C, 0x0B)) return false;
  if (!i2cWrite(addr, 0x31, 0x0B)) return false;
  if (!i2cWrite(addr, 0x2D, 0x08)) return false;
  accelType = ACCEL_ADXL345; accelAddr = addr; accelLsbPerG = ADXL_LSB_PER_G;
  return true;
}

bool accelInit() {
  accelType = ACCEL_NONE;
  accelAddr = 0;
  return mpuInit(MPU_ADDR_0) || mpuInit(MPU_ADDR_1) ||
         adxlInit(ADXL_ADDR_0) || adxlInit(ADXL_ADDR_1);
}

bool accelRead(int16_t *ax, int16_t *ay, int16_t *az) {
  uint8_t b[6];
  if (accelType == ACCEL_MPU6050) {
    if (!i2cReadReg(accelAddr, 0x3B, b, 6)) return false;
    *ax = (int16_t)((b[0] << 8) | b[1]);
    *ay = (int16_t)((b[2] << 8) | b[3]);
    *az = (int16_t)((b[4] << 8) | b[5]);
    return true;
  }
  if (accelType == ACCEL_ADXL345) {
    if (!i2cReadReg(accelAddr, 0x32, b, 6)) return false;
    *ax = (int16_t)((b[1] << 8) | b[0]);
    *ay = (int16_t)((b[3] << 8) | b[2]);
    *az = (int16_t)((b[5] << 8) | b[4]);
    return true;
  }
  return false;
}

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
  float x = ax * k, y = ay * k, z = az * k;
  float magMg = sqrtf(x * x + y * y + z * z) * 1000.0f;
  uint32_t nowMs = millis();

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

  fX += TILT_ALPHA * (x - fX);
  fY += TILT_ALPHA * (y - fY);
  fZ += TILT_ALPHA * (z - fZ);
  float fm = sqrtf(fX * fX + fY * fY + fZ * fZ);
  if (fm < 0.3f) return;
  float c = (fX * upX + fY * upY + fZ * upZ) / fm;
  c = clampf(c, -1, 1);
  tiltDeg = acosf(c) * 57.29578f;

  if (!tipped) {
    if (tiltDeg > TIP_DEG) {
      if (!tipSinceMs) tipSinceMs = nowMs ? nowMs : 1;
      if ((uint32_t)(nowMs - tipSinceMs) >= TIP_HOLD_MS) {
        tipped = true;
        uprightSinceMs = 0;
        moveAbort("tipped");
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

// ========================================================== Arduino entry points
void setup() {
  wheelsSetup();
  for (uint8_t i = 0; i < 2; i++) {
    arms[i] = Arm();
    arms[i].intervalUs = (uint32_t)(1000000.0f / (ARM_DEFAULT_DPS * stepsPerDeg()));
  }
#if ARM_COUNT > 0
  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    for (uint8_t k = 0; k < 4; k++) pinMode(PIN_ARM[i][k], OUTPUT);
    armRelease(i);
  }
#endif
  ioSetup();
  driveActive = false;
  dEver = false;
  lastDMs = lastLineMs = 0;

  Serial.begin(SERIAL_BAUD);

  Wire.begin();
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  if (accelInit()) {
    Serial.print(accelType == ACCEL_MPU6050 ? F("# mpu6050 at 0x") : F("# adxl345 at 0x"));
    Serial.println(accelAddr, HEX);
    captureUpright();
  } else {
    Serial.println(F("# no accelerometer found, tip/bump disabled"));
  }
#if WHEEL_TYPE == WHEELS_STEPPER
  wheelCalPrint();
#else
  dcCalPrint();                           // tells tars_motor these are DC wheels
#endif
  Serial.println(F("READY"));
}

void loop() {
  serialPoll();
  watchdogPoll();
  wheelsPoll();
#if ARM_COUNT > 0
  for (uint8_t i = 0; i < ARM_COUNT; i++) armPoll(i);
#endif
  accelPoll();
  ioPoll();
}
