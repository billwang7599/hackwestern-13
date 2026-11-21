# Goalie Uno firmware

`goalie/goalie.ino` turns an Arduino Uno R3 into the goalie's motor/sensor
co-processor. It takes line commands from the Pi over USB serial (115200 8N1)
and drives:

- **DC motor #1**, the goalie paddle (H-bridge channel A)
- **DC motor #2**, the celebration flag (H-bridge channel B)
- **28BYJ-48 stepper** via a ULN2003 board, the scoreboard needle
- **MPU6050** accelerometer for hit/save detection, reported as `HIT <mg>`

The protocol is defined in [`../docs/architecture.md`](../docs/architecture.md)
("Link 2"). Summary:

| Pi -> Uno | |
|---|---|
| `M <duty>` | goalie motor, `-1..1`, `+` = paddle towards `+x`. The Pi sends it at 200 Hz. |
| `S` | stop everything (both motors, needle halts, coils released) |
| `N <steps>` | needle to absolute half-step position (non-blocking, ~714 steps/s) |
| `C <ms>` | spin the flag motor for `ms` (max 10000), then stop. `C 0` stops it. |
| `Z` | needle: current position becomes 0 |

| Uno -> Pi | |
|---|---|
| `READY` | once after boot |
| `HIT <mg>` | accel spike, peak deviation from 1 g in milli-g, ≤ 1 per 250 ms |
| `A <mg>` | only with `DEBUG 1`: accel magnitude at 10 Hz |
| `# ...` | diagnostics (e.g. `# no accelerometer found, HIT disabled`). **The Pi ignores lines starting with `#`.** |

The Uno never replies to commands. Malformed lines, overlong lines (>31 chars)
and lines with binary bytes are dropped silently. `\n`, `\r\n` and bare `\r`
all end a line, and lowercase commands work, which helps in the Serial Monitor.

**Watchdog:** if no valid `M` arrives for 200 ms, the goalie motor stops
(brakes). The Pi has its own 300 ms watchdog upstream. Garbage lines do not
feed the watchdog.

## 1. Identify your motor driver

Read the chip markings (a phone camera on macro helps) and count the control
pins:

| You see | Type | Set in CONFIG |
|---|---|---|
| Two small 8-pin chips "L9110S"/"9110", header `B-IA B-IB GND VCC A-IA A-IB` | 2-PWM | `DRIVER_2PWM` (default) |
| "DRV8833" board, pins `AIN1 AIN2 BIN1 BIN2`, maybe `SLP`/`EEP` | 2-PWM | `DRIVER_2PWM`, tie SLP/EEP to 5 V |
| Tiny board with `IN1..IN4`, no enable pins, often "MX1508" | 2-PWM | `DRIVER_2PWM` |
| Big chip with heatsink "L298N", jumpers on `ENA`/`ENB` | EN+IN | `DRIVER_EN_IN`, **remove the ENA/ENB jumpers** |
| "TB6612FNG", pins `PWMA AIN1 AIN2 PWMB BIN1 BIN2 STBY` | EN+IN | `DRIVER_EN_IN`, tie STBY to 5 V |

Rule of thumb: 4 control inputs and no enable pin means 2-PWM. An enable/PWM pin
plus two direction pins per motor means EN+IN.

The "1.3 V" printed on the team's board is probably a minimum or voltage-drop
rating. Check the chip's datasheet for the real motor voltage range.

## 2. Wiring

### Pin map

PWM pins avoid Timer0 (pins 5/6), which runs `millis()`/`micros()`. Channel A
uses Timer1 (pins 9/10), channel B uses Timer2 (pins 3/11). The sketch uses no
Servo or `tone()`, so both timers are free.

#### `DRIVER_2PWM` (default: L9110S / DRV8833 / MX1508)

| Uno | Driver | Function |
|---|---|---|
| D9 (PWM) | A-IA / AIN1 / IN1 | goalie motor A |
| D10 (PWM) | A-IB / AIN2 / IN2 | goalie motor A |
| D3 (PWM) | B-IA / BIN1 / IN3 | flag motor B |
| D11 (PWM) | B-IB / BIN2 / IN4 | flag motor B |
| GND | GND | **common ground** |
| - | VCC / VM | motor supply + (battery pack), not the Uno |

Motor outputs: goalie motor on `MOTOR A` / `AO1 AO2` / `OUT1 OUT2`, flag motor
on the B outputs.

#### `DRIVER_EN_IN` (L298N / TB6612FNG)

| Uno | L298N | TB6612FNG | Function |
|---|---|---|---|
| D9 (PWM) | ENA | PWMA | goalie speed |
| D7 | IN1 | AIN1 | goalie direction |
| D8 | IN2 | AIN2 | goalie direction |
| D3 (PWM) | ENB | PWMB | flag speed |
| D4 | IN3 | BIN1 | flag direction |
| D12 | IN4 | BIN2 | flag direction |
| 5V | (+5V logic, if the 5V-EN jumper is off) | VCC (logic) | logic supply |
| - | - | STBY -> 5V | enable |
| GND | GND | GND | **common ground** |

#### ULN2003 + 28BYJ-48

| Uno | ULN2003 |
|---|---|
| A0 | IN1 |
| A1 | IN2 |
| A2 | IN3 |
| A3 | IN4 |
| GND | `-` |
| - | `+` -> 5 V **from the motor/5 V supply**, not the Uno 5V pin |

Plug the stepper's white connector into the ULN2003 board. Half-step mode
gives about 4096 half-steps per output revolution (4076 on many units), so
`N 1024` is about a quarter turn. The coils are switched off 20 ms after the
needle arrives, so the motor stays cool, and the gearbox holds the needle in
place.

#### MPU6050 (GY-521 breakout)

| Uno | MPU6050 |
|---|---|
| 5V | VCC (the GY-521 has its own 3.3 V regulator) |
| GND | GND |
| A4 | SDA |
| A5 | SCL |
| - | AD0: leave open/GND (0x68). Tied high, the sketch also finds 0x69. |

Mount it **on the goal frame/backboard**, not on the paddle. On the paddle, the
paddle's own swings look like hits. The sketch runs at ±8 g and samples at
500 Hz. It tracks a slow gravity baseline, so tilt and offset don't matter.
A spike more than `HIT_THRESHOLD_MG` (default 600 mg) from the baseline opens
a 15 ms window, and the window's peak is reported as `HIT <mg>`. I2C has a 3 ms
timeout, so a loose wire can't freeze the loop.

**ADXL345 instead of MPU6050:** different address (0x53, or 0x1D with SDO high)
and registers: `POWER_CTL 0x2D = 0x08`, `DATA_FORMAT 0x31 = 0x0A` (±8 g,
full-res, 256 LSB/g), data at `0x32..0x37` **little-endian**. Edit
`accelInit()`, `accelRead()` and `ACCEL_LSB_PER_G`. The notes are in the CONFIG
block.

### Power (read this)

- The motors need **their own supply** (e.g. a 4xAA pack or a 2S pack through
  the driver's VM/VCC/12V terminal). **Never power motors from the Uno's 5V
  pin.** Stall current resets the Uno, or worse, kills its regulator or the
  laptop/Pi USB port.
- **Common ground**: the Uno GND, the driver GND, the ULN2003 `-` and the
  battery `-` must all connect.
- Power the ULN2003 `+` from a separate 5 V source (or the battery through a
  5 V regulator). The 28BYJ-48 draws about 240 mA with coils on.
- The Uno itself is powered over USB from the Pi.
- Check the motor driver's max voltage before choosing the battery. L9110S
  boards are rated 2.5-12 V at about 800 mA per channel.

## 3. Upload

### Arduino IDE

Open `goalie/arduino/goalie/goalie.ino`, then choose Tools -> Board -> Arduino
Uno, pick the port, and click Upload.

### arduino-cli

```sh
brew install arduino-cli          # once
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno goalie/arduino/goalie
arduino-cli board list            # find the port, e.g. /dev/cu.usbmodem1101
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/cu.usbmodem1101 goalie/arduino/goalie
arduino-cli monitor -p /dev/cu.usbmodem1101 -c baudrate=115200
```

Build size (arduino:avr 1.8.8): about 10.6 KB flash (33%) and 505 B RAM (24%).

## 4. Bench test

### Serial Monitor (by hand)

Use 115200 baud with line ending **Newline**. You should see `READY`,
preceded by `# mpu6050 at 0x68` or `# no accelerometer found, HIT disabled`.

| Type | Expect |
|---|---|
| `M 0.5` | goalie motor twitches towards +x for **~200 ms, then stops**. That's the watchdog: by hand you can't send 200 lines/s. Use `serial_test.py` for sustained motion. |
| `M -0.5` | same, the other way. If `+` goes towards `-x`, set `MOTOR_A_INVERT = true` (or swap the motor wires). |
| `C 1000` | flag motor spins for 1 s |
| `N 2048` | needle turns about half a turn, then the ULN2003 LEDs go dark |
| `N 0` | needle returns |
| `Z` | the current needle position becomes 0 |
| `S` | everything stops |
| tap the frame | `HIT <mg>` |

### `serial_test.py` (scripted sweep)

```sh
pip install pyserial              # requirement: pyserial>=3.5
python goalie/arduino/serial_test.py /dev/cu.usbmodem1101
python goalie/arduino/serial_test.py /dev/cu.usbmodem1101 --max 0.8 --seconds 3
python goalie/arduino/serial_test.py /dev/cu.usbmodem1101 --no-motor   # stepper/flag/HIT only
```

The script sends a 200 Hz `M` sweep (0 -> +max -> -max -> 0), checks the
watchdog, runs the flag and the needle, then listens for HITs. It prints every
line from the Uno, and Ctrl-C sends `S`.

### Tuning on the bench

1. **Direction**: `+` duty must move the paddle towards `+x` (right in the
   overhead camera, goal at the bottom). Fix it with `MOTOR_A_INVERT`.
2. **Static friction**: raise `--max` (or type `M x` lines) until the paddle
   just starts moving. Put that value in `MOTOR_A_MIN_DUTY` **or** in the Pi
   controller's deadband compensation, not both. The default here is 0.
3. **Hit threshold**: set `#define DEBUG 1` and re-upload. Watch `# peak <mg>`
   while the motor sweeps (that's noise) and while you flick a ping-pong ball
   at the paddle (that's a hit). Set `HIT_THRESHOLD_MG` between the two.
4. **Brake vs coast**: `MOTOR_A_BRAKE_ON_STOP = true` stops the paddle faster
   when duty is 0 or the watchdog trips.

## 5. Host unit tests (no Arduino needed)

```sh
make -C goalie/arduino/test
```

This compiles `goalie.ino` unchanged against a mock Arduino core
(`test/mock/Arduino.h`, `test/mock/Wire.h` with a fake MPU6050), with ASan and
UBSan on. It tests the line parser (garbage, overlong, split and burst lines,
CR/CRLF), the motor pin patterns and deadband, the 200 ms watchdog, `S`, the
`C` timing, the stepper (phase sequence, ~714 steps/s, coil release, `Z`),
and HIT detection (threshold, peak, 250 ms rate limit, tilt immunity, a
sensor unplugged mid-run).
