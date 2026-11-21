# TARS on the Pi: hands-on test checklist

For someone at the robot with a fresh clone who hasn't seen this repo before.
Work top to bottom and tick things off. Each step says what you should see.
If you don't see it, go to [Troubleshooting](#7-troubleshooting) before moving on.

**Everything on the robot is QNX native:** `tars_motor` is C, built with `qcc`
(§1a). There's no Python on the Pi. Python only runs on the laptop (teleop).

**What you're testing:** a two-wheel rover. Each wheel is a **28BYJ-48 stepper
on a ULN2003 board**, driven by an **Arduino Uno**. The Uno is plugged into a
**Raspberry Pi 5 running QNX** over USB. On the Pi, `tars_motor` listens on TCP
port 5555, and a laptop on the same Wi-Fi drives it with WASD.

```
laptop (teleop.py) --Wi-Fi TCP :5555--> Pi 5 / QNX (tars_motor) --USB serial--> Uno --> 2x ULN2003 --> 2x 28BYJ-48
                                                                                  <-- MPU6050 (I2C), TEACH button, LED, buzzer
```

> **The robot is now the Kuman SM11 smart car kit** (DC TT motors, the kit's driver board,
> no encoders). The firmware's default build is for it: wiring, unplug list, power and first
> checks are in **[§2b Kuman SM11](#2b-kuman-sm11-smart-car-the-default-build)**. The 28BYJ-48
> stepper rover below is still a build option (`-DWHEEL_TYPE=2`).

Nothing here has run on real hardware yet. All of it has only been tested
against simulators on a Mac. Expect to find the first real bugs, and write
down what you see.

---

## 0. Get the code

```sh
git clone https://github.com/billwang7599/hackwestern-13.git
cd hackwestern-13                       # everything is on main; no branch switch needed
```

You need:

| Thing | For |
|---|---|
| A Mac or Linux laptop with Docker Desktop | building `tars_motor` for QNX |
| QNX SDP 8.0 installed in `~/qnx800`, licence in `~/.qnx` | the QNX compiler (`qcc`). See README §1 for the free licence and install |
| Arduino IDE 2.x **or** `arduino-cli` | flashing the Uno |
| Python 3.10+ | the teleop on the laptop |
| The Pi 5 booted from the QNX Quick Start (QSTI) image, on the same Wi-Fi as the laptop, with SSH working | running `tars_motor` |

Find the Pi's IP from the hotspot's client list, then check SSH works:
`ssh qnxuser@<pi-ip>`. Use that IP wherever this guide says `<pi-ip>`.

---

## 1. Build and deploy

### 1a. `tars_motor` (QNX binary) — on the laptop

```sh
docker/run.sh build                  # once: builds the tars-qnx800 image (~minutes)
docker/run.sh qcc --version          # sanity: prints a qcc/gcc version
make -C iter1-wasd/pi                           # cross-compiles in Docker -> iter1-wasd/pi/build/tars_motor
file iter1-wasd/pi/build/tars_motor             # expect: ELF 64-bit LSB ... ARM aarch64
```

If `qcc` is on your PATH (you sourced `~/qnx800/qnxsdp-env.sh` on a Linux
laptop), `make -C iter1-wasd/pi` uses it directly with no Docker.

Deploy it. `PI_HOST` defaults to `qnxuser@tars.local`; the file goes to `/data/home/qnxuser/tars_motor`:

```sh
make -C iter1-wasd/pi deploy PI_HOST=qnxuser@<pi-ip>
```

**Optional first check with no hardware:** `make -C iter1-wasd/pi host && make -C iter1-wasd/pi test`
builds a Mac/Linux version and runs the server tests (about a minute). It
should end with `270/270 checks passed`.

### 1b. Uno firmware — on the laptop, Uno plugged into the laptop

The sketch is `iter1-wasd/uno/tars/tars.ino`. The default build is the **Kuman SM11 kit**
(`CAR_SM11`, §2b). For the 28BYJ-48 stepper rover in §2, add `#define WHEEL_TYPE WHEELS_STEPPER`
at the top of the sketch (or `--build-property "build.extra_flags=-DWHEEL_TYPE=2"` with
arduino-cli). **SM11: unplug the kit's Bluetooth module before uploading** (it sits on D0/D1).

**Arduino IDE:** open `iter1-wasd/uno/tars/tars.ino`, then *Tools → Board → Arduino Uno*
and *Tools → Port → the usbmodem / usbserial port*. Click Upload.

**arduino-cli:**
```sh
arduino-cli core install arduino:avr
arduino-cli board list                                   # find the port
arduino-cli compile --fqbn arduino:avr:uno iter1-wasd/uno/tars
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/cu.usbmodemXXXX iter1-wasd/uno/tars
```
The sketch has never been compiled by avr-gcc, only by a host compiler
against a mock Arduino core. If the compile fails, send the error to the lead.

### 1c. Laptop teleop

```sh
cd laptop
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt          # pygame-ce (+ anthropic, unused here)
cd ..
```

### 1d. Pi on Wi-Fi

Use a **phone hotspot** (venue Wi-Fi usually has a captive
portal or client isolation, and then the laptop can't reach the Pi). Put the
laptop on the same hotspot. Before booting the QNX Quick Start (QSTI) image, put
the SD card in the laptop and, on the FAT32 boot partition, edit or create
`wpa_supplicant.conf`:
```
network={
    ssid="YourHotspotName"
    psk="hotspot-password"
    key_mgmt=WPA-PSK
}
```
Boot the Pi with no USB keyboard or mouse plugged in. Use 2.4 GHz /
"Maximize Compatibility" on an iPhone hotspot if it doesn't join.
Edits to this file also apply on later boots.

**2. Find the Pi's IP.** Check the hotspot's connected-devices list, or with a
screen/serial console on the Pi run `ifconfig bcm0` (bcm0 is the Wi-Fi
interface; it should show an `inet` address). Then:
```sh
ssh qnxuser@<pi-ip>          # QSTI default user qnxuser (password qnxuser, see the QSTI docs)
```

**3. Run `tars_motor`.** Plug the Uno into the Pi's blue USB port, deploy (§1a), then:
```sh
ssh qnxuser@<pi-ip> '/data/home/qnxuser/tars_motor -b uno'     # default /dev/serusb1; use -d /dev/serusbN if it's elsewhere
```
If it can't open the Uno, run `ls /dev/ser*` on the Pi and see "No serial device on the Pi" in §7.

**4. Drive from the laptop** (wheels in the air first):
```sh
python3 iter1-wasd/laptop/tars_client.py <pi-ip>     # ping + STATUS: expect 'backend': 'uno', 'ready': 1
python3 iter1-wasd/laptop/teleop.py <pi-ip>          # click the window, hold W/A/S/D, Space = stop
```
`wheel_cal.py` (§3 step 5) works the same way: `TARS_HOST=<pi-ip> python3 iter1-wasd/laptop/wheel_cal.py move 100`.

---

## 2. Wiring

**Power first.** The steppers get their **own 5 V supply**, for example a USB
power bank with a cut cable, or a 5 V / 2 A buck converter. **Never** power them
from the Uno's 5 V pin: two moving steppers draw about 0.5 A and will brown out
or reset the Uno. **Connect all grounds together:** stepper supply GND, both
ULN2003 `-` pins, and Uno GND.

| Uno pin | Goes to | Notes |
|---|---|---|
| D8, D9, D10, D11 | **Left** wheel ULN2003 IN1, IN2, IN3, IN4 | keep that order |
| D4, D5, D6, D7 | **Right** wheel ULN2003 IN1, IN2, IN3, IN4 | keep that order |
| — | both ULN2003 `+` → **separate 5 V**, `-` → common GND | the 5-pin motor plug goes straight into the board |
| A4 (SDA), A5 (SCL) | MPU6050 / GY-521 SDA, SCL | VCC → Uno 5 V (the GY-521 has a regulator), GND → GND. Address 0x68 (AD0 low) |
| D2 | TEACH push button, other leg → GND | internal pull-up, no resistor needed |
| D13 | status LED via 220–330 Ω → GND | the Uno's own "L" LED shows the same thing |
| D12 | buzzer `+`, `-` → GND | active buzzer (sealed back, beeps on DC). For a passive piezo, set `BUZZER_PASSIVE = true` |
| D0, D1 | leave free | USB serial to the Pi |
| D3, A0–A3 | free | A0–A3 = optional single arm stepper (`ARM_COUNT 1`) |
| USB-B | **Pi 5 blue USB 3 port** | power + data for the Uno |

Mount the MPU6050 **flat and low on the chassis**. The Uno treats whatever
orientation it has at boot as "upright", so power up with the robot standing
on its wheels.

**Which wheel is left?** Left and right are as seen from behind the robot,
looking forward. The right motor faces the other way, so the firmware mirrors
it (`WHEEL_INVERT = {false, true}`).

---

## 2b. Kuman SM11 smart car (the default build)

The kit: Uno R3 + the kit's motor driver board (EN + IN1/IN2 per side, L298N-style), TT gear
motors, buzzer, LEDs, push button, IR receiver, ultrasonic on a servo, line sensors, IR
proximity sensors, Bluetooth module, battery box. **No wheel encoders**, so `MOVE`/`TURN` are
**timed** (D 1 for |mm| / mm_s), not counted. The Pi re-plans from the iPhone pose after every
leg, which is what corrects the error (sim numbers below).

**Unplug / leave alone:**
- **!! The Bluetooth module. !!** It is wired to D0/D1, the Uno's USB serial link to the Pi.
  Plugged in, it garbles the Pi's commands (and uploads fail). Pull it off the shield.
- The **IR proximity sensors on A4/A5** if you fit an MPU6050/ADXL345 (A4/A5 are I2C SDA/SCL).
  Without an accelerometer, tip/bump detection is simply off (it says so at boot).
- The **servo**: not used. The Servo library would take Timer1 and kill the PWM on D10 (left
  EN), which is the "no speed control on one side" problem Mitov hit. The ultrasonic just
  faces forward and isn't read either.
- IR receiver (D2), right LED (D4), ultrasonic (A0/A1), line sensors (A2/A3): stay plugged
  in, unused.

**Pin map** (firmware `CAR_SM11` block). Source: the community SM11 sketch
[github.com/mbof/sm11 `sm11_pinout.h`](https://github.com/mbof/sm11) and its `motor.cpp`
(`Motor(enable, go, back)`, go HIGH = forward), cross-checked by the
[Mitov SM11 programming post](http://labpacks.blogspot.com/2018/07/programming-kuman-smart-car-arduino.html)
(Servo library breaks speed control on one side = an EN pin on Timer1).

| Uno pin | Kit part | Our use | Confirmed? |
|---|---|---|---|
| D10 | left motor ENABLE (PWM, Timer1) | left speed | mbof sketch |
| D8 / D9 | left motor GO / BACK | IN1 / IN2 | mbof sketch |
| D5 | right motor ENABLE (PWM, Timer0) | right speed | mbof sketch |
| D6 / D7 | right motor GO / BACK | IN1 / IN2 | mbof sketch |
| D12 | buzzer (active, HIGH = on) | `BEEP` | mbof sketch |
| D13 | push button "KEY" (to GND, pull-up) | TEACH button | mbof sketch |
| D3 | left LED | status `LED` (D4 right LED unused) | mbof sketch; **polarity unverified** (`LED_INVERT`) |
| D2 | IR receiver | unused | mbof sketch |
| A0 / A1 | ultrasonic TRIG / ECHO | unused | mbof sketch |
| A2 / A3 | line sensors R / L | unused | mbof sketch |
| A4 / A5 | IR proximity R / L | **I2C if you add an accelerometer: unplug them** | mbof sketch |
| D0 / D1 | Bluetooth module | **USB serial to the Pi: unplug Bluetooth** | mbof README |
| servo signal | ? | unused | **unverified** (no source gave the pin) |

**Unverified kit facts** (no source we could reach confirmed them; check your kit): the driver
chip (L298N vs L293D-style board), 2WD vs 4WD (with 4WD each side's two motors share a
channel; the firmware doesn't care), the battery (2×18650 is typical), and whether the driver
board feeds the Uno through VIN or the 5 V pin.

**ENA/ENB jumpers:** if the kit's driver is an L298N module with ENA/ENB jumper caps, the caps
must be **off** and ENA/ENB wired to D10/D5 (stock SM11 wiring). With a cap on, that side runs
at full speed whatever the PWM says: the speed cap is gone. Test: `D 0.3 0.3` must be clearly
slower than `D 1 1`, and even `D 1 1` is a walking-pace crawl (the cap, below).

**Power:**
- The **kit's battery** runs the motors and the Uno (through the kit board). Charge it: the
  speed (and the calibration) drops with the battery voltage.
- The **Pi 5 needs its own 5 V supply**: a USB-C PD 5 V / 5 A supply is ideal. On a 3 A supply
  the Pi caps its USB ports at 600 mA total, which the Uno + webcam + mic can exceed.
- **Common ground:** the Uno's USB cable to the Pi already joins the grounds. Don't add any
  other wire from the kit to the Pi. If the kit board feeds the Uno's **5 V pin** directly (not
  VIN / the barrel jack), that fights the Pi's USB 5 V: then cut/unplug that 5 V wire and let
  the Pi's USB power the Uno (the kit battery still runs the motors, grounds stay joined).

**Speed cap: ~150 mm/s at `D 1`** (`DC_MAX_DUTY`, a duty guess for TT motors; measure it).
Everything above the Uno was tuned on the ~41 mm/s steppers, so we checked how fast goto can go:
- `iter2-autonomy/goto/tests/test_dc_car.py --sweep` (2D sim, visual GOTO, 16 random starts per
  row): 100, 150 and 200 mm/s arrive like the stepper reference (16/16, stops 0.37–0.56 m, none
  under 0.30 m). 300 mm/s: first stop at 0.25 m and misses. 450 mm/s: 14/16 exact, 5/16 with
  under-turning.
- lidar_guard's stop zone is 0.55–0.6 m. Worst case before the wheels stop: a 150 ms-stale LiDAR
  sample + 100 ms gate heartbeat + a 20 ms tick + the Uno's brake ≈ 0.3 s. That's ~45 mm at
  150 mm/s (~90 mm at 300) plus a few cm of braking.
- 150 leaves ~30% under the 200 mm/s edge for battery voltage (8.4 V fresh vs ~7 V flat).
After `wheel_cal.py dc-move`, if the measured speed isn't 100–200 mm/s, change `DC_MAX_DUTY`,
re-flash and calibrate again. `DC_MIN_DUTY` (0.30) is the slowest a rolling wheel keeps turning;
`DC_KICK_DUTY`/`DC_KICK_MS` (0.8 for 40 ms) get a wheel going from rest.

**First flash (wheels in the air, then on the floor).** Serial Monitor at 115200, "Newline":
1. Reset. Expect `WHEELS DC 150.0 120.0`, then `READY`. Nothing garbled (garbled = Bluetooth
   still plugged in).
2. `D 0.5 0.5` (it only runs 200 ms: the watchdog; type it again or use teleop): **both wheels
   forward.** A wheel backwards → flip `LEFT_INVERT` / `RIGHT_INVERT` in the SM11 block. `D 0.5 0`
   moves the right wheel → left/right are swapped: swap the L and R pin groups.
3. `D 0.3 0.3` from rest: both wheels start (short kick, then slow). One doesn't → raise
   `DC_MIN_DUTY` or `DC_KICK_DUTY`. `S` stops at once (brake).
4. On the floor: `M 300` → ~2 s forward, then `DONE M 300.0`. `R 90` → spins **right**
   (clockwise from above) ~0.75 s, then `DONE R 90.0`. `R -90` spins left. `S` mid-move →
   `ABORT M <done> stop` and it stops dead.
5. It curves on `M 1000` → the faster side's `DC_LEFT_GAIN` / `DC_RIGHT_GAIN` below 1 (the two
   EN pins have different PWM frequencies, 490 vs 980 Hz, so some pull is normal).
6. On the Pi: `tars_motor` logs `co-processor has DC wheels: 150.0 mm/s, 120.0 deg/s` and
   `STATUS` shows `wheels=dc`. Then calibrate (Step 5, DC).

**How sloppy can it be?** Timed moves on a real floor will be off. In the sims, goto still
arrives with ±25% MOVE/TURN error plus 15°/m drift:
- `test_dc_car.py` (visual GOTO, in `make test`; numbers from a 16-start run): exact timing
  16/16; ±15% (after calibration) 32/32 over two rows; ±25%: 60/64 over four rows. The misses are all clean `not_found` (BEEP 3): a TURN that really
  does 75% turns 12 × 22.5° = 270°, and with the 60° FOV that leaves a blind wedge.
  **Keep TURN error under ~15%** (calibrate `dc-spin`) and the search covers 360°.
- `test_fullstack_dc.py` (map-guided, real `tars_motor` + `goto`, fake iPhone/guard/YOLO):
  obstacle, moved object, visual fallback × 5 error rows: 64/65. The miss: goto's object memory
  merges sightings by `tars_motor`'s dead-reckoned x/y, which drifts metres on a DC car, so a
  moved bin refreshes the old entry instead of downgrading it (it still docks; fix pending in
  `goto_core.c`).
- `tars_motor`'s own `POSE` is dead-reckoned from commanded amounts and is **not trustworthy**
  on the DC car. Use the iPhone pose.

---

## 3. Bring-up, step by step

### Step 1 — Uno alone, from the laptop's serial monitor

Wiring: the Uno on the laptop's USB, the stepper 5 V supply on, and the
**robot up on a box with the wheels in the air**.

Open the Serial Monitor at **115200 baud**, line ending **"Newline"**.
Press the Uno's reset button. Expected output:

```
# mpu6050 at 0x68               (or "# no accelerometer found, tip/bump disabled")
# upright captured
# wheels diam 65.0 track 100.0 steps/rev 4075.8
READY
```

Send these one at a time:

| Type | Expect |
|---|---|
| `?` | `# arms 0 0 tilt 0 tipped 0 accel mpu6050 wheels 0 0 move -` |
| `M 100` | both wheels turn **forward** for about 2.7 s, then `DONE M 100.0`. The ULN2003 LEDs flicker while it moves and go dark ~0.1 s after |
| `M -50` | both turn backward, then `DONE M -50.0` |
| `R 30` | left wheel forward, right wheel back (turning **right**), ~0.9 s, then `DONE R 30.0` |
| `R -90` | the opposite way, then `DONE R -90.0` |
| `M 300` then `S` while it moves | it stops at once: `ABORT M <how far> stop` |
| `D 0.5 0.5` | both wheels spin **for only 0.2 s**. That's the watchdog, and it's correct; the Pi resends `D` every 20 ms |
| `L 2` / `L 3` / `L 0` | LED slow blink / fast blink / off |
| `B 3` | three short beeps |
| press TEACH briefly | `BTN TEACH` (on release) |
| hold TEACH 2 s | `BTN FORGET` (while still held) |
| `K` | `# wheels diam 65.0 track 100.0 steps/rev 4075.8` |

If a wheel goes the wrong way, see [Troubleshooting](#7-troubleshooting).

### Step 2 — Uno on the Pi, `tars_motor` running

Move the Uno's USB cable to the Pi's **blue** USB port.

```sh
ssh qnxuser@<pi-ip>
ls /dev/ser*                       # note what's there; unplug/replug the Uno and look again
```
The new entry is the Uno, most likely **`/dev/serusb1`**, which is also the
default. If no new device appears, see Troubleshooting ("no serial device").

```sh
/data/home/qnxuser/tars_motor -b uno -d /dev/serusb1
```
Expected:
```
[uno] opened /dev/serusb1 @115200
tars_motor: backend=uno dev=/dev/serusb1 port=5555 watchdog=500ms prio=30
[ctl] STOP (watchdog)                  <- normal: no client yet, wheels held at 0
[uno] # mpu6050 at 0x68                <- opening the port rebooted the Uno (~1.5 s)
[uno] # upright captured
[uno] # wheels diam 65.0 track 100.0 steps/rev 4075.8
[ctl] co-processor READY
```
You might also see `can't get SCHED_RR prio 30 (need root?)`, which is fine.
Leave this running. Ctrl-C stops it and the motors.

It's fine to start `tars_motor` **before** plugging in the Uno: it logs
`[uno] /dev/serusb1 not there yet ... retrying every 1000 ms` and connects
when the Uno appears. Unplugging the Uno logs `[uno] lost ...`, and
replugging reconnects on its own.

(Same thing from the laptop: `make -C iter1-wasd/pi run PI_HOST=qnxuser@<pi-ip> UART=/dev/serusb1`.)

### Step 3 — STATUS from the laptop

```sh
export TARS_HOST=<pi-ip>
python3 iter1-wasd/laptop/tars_client.py $TARS_HOST
```
Expect `ping N ms` and a dict containing `'backend': 'uno'`, `'ready': 1`,
`'tipped': False`, `'move': 'idle'`, `'wheel_d': 65.0`.

Raw alternative: `nc <pi-ip> 5555`, then type `STATUS`, `LED 2`, `BEEP 2`, `TURN 30`.
Each line gets an `OK ...` reply.

### Step 4 — WASD teleop (wheels still in the air first, then on the floor)

```sh
cd laptop && . .venv/bin/activate
python teleop.py $TARS_HOST          # a small window opens: CLICK IT to give it focus
```
- Hold **W**: both wheels forward. **Release**: they stop within about 0.15 s.
- Hold **S** to go back. Hold **A** / **D** to spin left / right in place.
- **W+A** / **W+D** arc. **W+S** together stop it.
- **+ / -** change speed: 1.0 = 12 RPM, the stepper maximum. The window shows L/R, speed and connection.
- **Space** stops. Clicking away from the window also stops. **x** or **Esc** quits.

Without the window: `python teleop.py --terminal $TARS_HOST`. It stutters,
because terminals don't report key release.

### Step 5 — Calibration (on the floor)

Use a hard, flat floor and a tape measure. Run these from `iter1-wasd/laptop/`.

**SM11 / DC car** (timed moves: calibrate speeds, with the battery you'll demo on):
1. `python wheel_cal.py dc-move` → `MOVE 1000`. Tape the real distance, type it in. It scales and
   saves mm/s (`SPEEDCAL` → the Uno's EEPROM). Run it again: it should land within ~5%. Outside
   100–200 mm/s → change `DC_MAX_DUTY` (§2b).
2. Put tape along the nose, `python wheel_cal.py dc-spin` → `TURN 360`. Type how far it really
   turned (e.g. 300). Repeat until it's within ~15° (goto's search needs < ~15% error).
3. `python wheel_cal.py show`. (Bench alternative: `V <mm_s> <deg_s>` in the Serial Monitor.)

**Stepper rover:**

1. **Wheel diameter:** mark the start, then `python wheel_cal.py move 1000`.
   Measure the real distance in mm. Then
   `new_diam = 65 × measured / 1000` and save it with
   `python wheel_cal.py set <new_diam> 100`.
   Repeat until it's within about 1 cm.
2. **Track width:** put a strip of tape on the floor along the robot's nose,
   then run `python wheel_cal.py turns 12 30`. That's 12 × 30°, which should
   be exactly one full turn. Estimate how far it actually turned in total (T°).
   Then `new_track = track × 360 / T` (it over-turned → smaller track), and run
   `python wheel_cal.py set <diam> <new_track>`.
3. `python wheel_cal.py show`. The values are saved in the Uno's EEPROM and
   survive power-off.

### Step 6 — 2-minute floor load test

Put the robot fully loaded (Pi, battery, camera and body on board) on the floor
you'll demo on.

1. `python wheel_cal.py move 500`, twice. It should go straight with no
   buzzing, and `done` should be 500 each time.
2. In teleop at speed 1.0, drive forward and back and spin for about a minute.
   Listen: smooth whirring is fine. **Buzzing, clicking or a wheel that stops
   while its LEDs still flicker means skipped steps**. See Troubleshooting.
3. Repeat at speed 0.6. If 1.0 skipped steps but 0.6 doesn't, set
   `WHEEL_MAX_RPM` lower (step 7).
4. Feel the motors and ULN2003 chips. Warm is fine; too hot to touch is not.
   Coils switch off when idle (`WHEEL_HOLD = false`).
5. Tip the robot past ~40° and hold it there. The wheels stop, `STATUS`
   shows `tipped=1`, and driving answers `ERR tipped`. Stand it up and press
   **r** in the teleop (RESET).

Expected speed: about **4 cm/s** at 12 RPM with 65 mm wheels, so crossing
1 m takes about 25 s. Bigger wheels go faster but push less.

### Step 7 — voice (speech commands, all on the Pi)

Details and the full command list are in [`iter2-autonomy/voice/README.md`](../iter2-autonomy/voice/README.md).
Everything here runs on QNX in C; no laptop is needed once it's deployed.

**7a. Mic check first (10 min). If this fails, voice is off; tell the lead.**
```sh
make -C iter2-autonomy/voice mic_test                          # laptop: QNX build (Docker + ~/qnx800), no whisper needed
ssh qnxuser@<pi-ip> 'mkdir -p voice'
scp iter2-autonomy/voice/build/qnx/mic_test qnxuser@<pi-ip>:voice/
ssh qnxuser@<pi-ip>
cd voice
./mic_test -l                                   # lists cards/devices; the C270 should show up with [capture]
./mic_test                                      # records 3 s from the default capture device -> mic.wav
./mic_test -d 1,0                               # ...or pick card 1 device 0 from the list
```
You should see a level meter jump while you talk and `VERDICT: OK`. Then listen to it on the laptop:
`scp qnxuser@<pi-ip>:voice/mic.wav . && afplay mic.wav` (Linux: `aplay mic.wav`).
- `can't open the mic`: is io-audio running (`pidin | grep io-audio`)? Is the webcam on a **blue USB 3 port**? Try every card from `-l`.
- `pure silence`: wrong device, or the mic is muted. `WEAK`: speak closer; a headset/lapel USB mic is much better on a loud floor.
- Write the working `-d card,dev` down. It goes into `voice.conf` as `input = qnx:card,dev`.

**7b. Build and deploy voice**
```sh
iter2-autonomy/voice/fetch_whisper.sh && iter2-autonomy/voice/get_model.sh    # laptop, once: whisper.cpp v1.9.5 source + 31 MB model
make -C iter2-autonomy/voice                                   # QNX build of whisper.cpp + tars_voice (a few minutes the first time)
make -C iter2-autonomy/voice deploy PI_HOST=qnxuser@<pi-ip>    # -> /data/home/qnxuser/voice/ (binary, model, voice.conf, start script)
```
This QNX build has never been run. If it fails, see "Not tested yet" in `iter2-autonomy/voice/README.md` for likely fixes.

**7c. Try it** (`tars_motor` running from Step 2, robot on blocks first)
```sh
cd ~/voice
./tars_voice -c voice.conf --dry                # prints what it heard + what it WOULD send; nothing moves
./tars_voice -c voice.conf                      # for real
```
Say, about 30 cm from the mic: **"TARS, turn left"** → `[heard] "TARS, turn left." ... TURN -30`, the
robot beeps and turns. Then try "TARS, go forward", "TARS, turn right ninety", **"Stop!"** (no "TARS" needed),
and "TARS, go to the trash can" (→ `[goto] GOTO trash can`; it just logs `unreachable` until the go-to
controller on port 5570 is running, and that's fine). Speech without "TARS" is ignored. **Press the Uno
button** and you have 4 s to give a command without saying "TARS" (use this on a noisy floor).

Write down the `stt=…ms` numbers from the `[heard]` lines (whisper time per command) with and without
YOLO running. They go in the CPU table in PLAN.md.

**7d. Keep it running / at boot:** `./start_voice.sh &` restarts it if it crashes and logs to
`~/voice/logs/voice.log`. For boot, add this to the startup script after the `tars_motor` line (§5):
`(cd /data/home/qnxuser/voice && ./start_voice.sh >/dev/null 2>&1 &)`

### Step 8: go-to ("TARS, go to the trash can")

Needs Steps 2–5 working. Details are in `iter2-autonomy/goto/README.md`.

1. **On the laptop, build:**
   - `iter2-autonomy/goto/get_model.sh` writes `iter2-autonomy/goto/models/yolo_world.onnx` and `labels.txt`.
   - `make -C iter2-autonomy/goto` builds `goto` with qcc.
   - `make -C iter2-autonomy/goto detect ORT_ROOT=...` needs ONNX Runtime built for QNX first (see `iter2-autonomy/goto/README.md`).
   - `make -C iter2-autonomy/goto deploy PI_HOST=qnxuser@<pi-ip>` copies everything to the Pi.
2. **Camera alone:** run `./detect --camera camapi:1 --nomodel --stdout --no-udp --frames 20`. You should get 20 lines with the right `w`/`h`.
   - If `camera_open failed`: check that the QNX `sensor` service is running and that its config lists the USB webcam.
3. **Model alone:** run `./detect --camera camapi:1 --stdout --no-udp` and hold a cup or bottle in view. You should see a `"label":"cup"` box, and stderr prints fps every 5 s. Expect about 5–10 fps.
4. **Wheels in the air:** start `tars_motor`, `./goto -c goto.conf -o objects.txt &` and `./detect &`, then from the laptop:
   `printf 'GOTO cup\n' | nc <pi-ip> 5570`. It beeps once.
   - With the cup out of view: the wheels do 30° turns. After 12 of them (a full circle) it beeps three times.
   - With the cup in view: the wheels spin forward, and the faster side is the side the cup is on.
5. **On the floor:** put a trash can 1–2 m away and `GOTO trash can`. It should turn toward it, crawl up (about 4 cm/s) and stop about 0.5 m away with one beep.
   - **Watch:** `printf 'STATUS\n' | nc <pi-ip> 5570` shows `state= h= bearing=`.
   - **If it stops too early or too late:** change `arrive_h_frac` in `goto.conf`. It's the box height ÷ frame height at the spot where you want it to stop.
6. **Stop test:** during an approach, `printf 'S\n' | nc <pi-ip> 5555`. The wheels stop at once, and goto STATUS shows `result=cancelled:stop`.
7. **Object memory** (`iter2-autonomy/goto/README.md`, "Object memory"):
   - `printf 'POSE\n' | nc <pi-ip> 5555` → `OK x y hdg epoch moving`. Drive 1 m with WASD and check `x` says about 1000. Spin 360° and check `hdg` comes back near 0. Write down how far off it is: that's the drift.
   - `printf 'SCAN\n' | nc <pi-ip> 5570` (or say "TARS, look around"): 12 × 30° turns, then `printf 'MEMORY\n' | nc <pi-ip> 5570` lists e.g. `trash can#1 fresh ... est_dist=`. Compare `est_dist` with a tape measure. If it's way off, set `cam_height_mm` / `cam_pitch_deg` in `goto.conf`.
   - Turn the robot away by hand with WASD, then `GOTO trash can`: the first move should be one TURN straight toward it, not a 30° search.
   - It's saved in `/data/home/qnxuser/goto/memory.txt`. **If you carry the robot somewhere else, say "TARS, forget everything" or send `FORGET` to 5570**, because the remembered positions are relative to where it was.

---

## 4. Useful commands (TCP, port 5555)

| Command | Does |
|---|---|
| `D l r` | wheel speeds −1…1 (1 = 12 RPM), resend every < 500 ms |
| `S` | stop now, aborts any MOVE/TURN |
| `MOVE <mm> [speed]` | exactly mm forward (−: back); replies `OK <id> <est_ms>` at once; poll `STATUS` `move=` |
| `TURN <deg> [speed]` | spin exactly; **+ = right**, − = left |
| `WHEELCAL [diam track]` | read / set wheel calibration (mm) |
| `LED off\|teach\|rec\|ok\|no\|forget` or `LED 0..3`, `BEEP n` | status LED (teach = slow blink, rec/ok = solid, no/forget = fast blink) / n beeps |
| `STATUS` | everything: `why=`, `tipped=`, `move=`, `move_done=`, `btn_teach=`, `btn_forget=` … |
| `RESET` | clear tip-over latch |
| `WHO <name>` | label this client in the log (voice sends `WHO voice`) |
| `MODE gesture\|follow\|toggle\|teach` | request for the vision app (shown in `STATUS mode_req= mode_seq=` and in the reply to `VIS`) |

Several clients can be connected at once (voice, vision, YOLO go-to, teleop). **`S` from any client
stops everything**, and for 1.5 s (`-H` ms) other clients' non-zero `D` is refused (`ERR held`), so a
steering stream can't undo someone else's stop. A client disconnecting only stops a drive or move
**it** started. `STATUS` shows `clients=` and `hold_ms=`.

## 5. Start at boot (optional)

`tars_motor` is safe to start with nothing connected. With no client the
wheels are held stopped (watchdog), and a missing Uno is retried every second.
To start it at boot, add this to the image's startup script:

```sh
/data/home/qnxuser/tars_motor -b uno -d /dev/serusb1 > /tmp/tars_motor.log 2>&1 &
```

Voice (Step 7d) starts after it: `(cd /data/home/qnxuser/voice && ./start_voice.sh >/dev/null 2>&1 &)`.

The right startup file depends on the QSTI image. Look for `/etc/rc.d/rc.local`
or ask the lead. Not tested yet.

## 6. Change firmware settings

Everything is in the `CONFIG` block at the top of `iter1-wasd/uno/tars/tars.ino`. Edit,
re-upload (§1b), and remember the Uno must be on the laptop to flash it.

| Setting | Default | When to change |
|---|---|---|
| `WHEEL_MAX_RPM` | 12 | skipped steps/buzzing at full speed → 10 (also set `WHEEL_MAX_RPM` in `iter1-wasd/laptop/teleop.py` and `iter1-wasd/pi/src/wheels.h` so displays/estimates match) |
| `WHEEL_ACCEL_SPS2` / `WHEEL_DECEL_SPS2` | 3000 / 6000 | skips when starting / stopping → lower them |
| `WHEEL_INVERT` | `{false, true}` | a wheel goes the wrong way |
| `WHEEL_STEPS_PER_REV` | 4075.77 | after diameter calibration a 1 m drive is still ~5 mm off → try 4096 (some motors are a true 64:1) |
| `BUZZER_PASSIVE` | false | passive piezo (silent with the default) |

---

## 7. Troubleshooting

**Stepper buzzes or vibrates but doesn't turn (or turns erratically)**
- Wrong coil order: IN1–IN4 must go to the pins in the table order. The four
  LEDs on the ULN2003 should light in a rotating A→B→C→D pattern during `M 100`.
- Too fast for the load: lower `WHEEL_MAX_RPM` to 10 or 8, or try `MOVE 100 0.5`
  (half speed) to confirm.
- Weak supply: measure the stepper 5 V **while moving**. Below ~4.6 V means the
  supply or wires are too thin.
- Ground: the stepper supply GND must connect to Uno GND.

**One wheel goes the wrong way**
- In `iter1-wasd/uno/tars/tars.ino` flip that wheel's entry in `WHEEL_INVERT`, then
  re-upload. Test with `M 100`: both forward.
- Both wheels backward: flip both. `R 30` must then turn **right** (clockwise from above).

**Turns go the wrong way but straight lines are fine**
- Left and right are swapped: swap the two ULN2003 input cables (D8–D11 ↔ D4–D7).

**No serial device on the Pi** (`ls /dev/ser*` shows nothing new)
- Use the blue USB port and a known-good data cable (some cables are charge-only).
- Check that a USB serial driver is running: `pidin ar | grep -i serusb`. If
  `devc-serusb` exists on the image but isn't running, start it by hand.
- Clone Unos with a **CH340** chip need a different driver than genuine
  Unos (CDC-ACM). Look at the chip next to the USB socket.
- Fallback: header UART. Pi pin 8 → Uno D0; Uno D1 → 1 kΩ/2 kΩ divider → Pi
  pin 10; GND–GND; then `-d /dev/ser1` (see docs/tars_build.md §6).

**Wheels stop by themselves while driving** (`STATUS` `why=watchdog`)
- The laptop stopped sending: the teleop window lost focus (click it), or
  Wi-Fi dropped (check `ping`, expect < 100 ms).
- On the Uno's own monitor `D` only lasts 200 ms. That's by design.

**`ERR tipped ...`**: the robot was over 40° for 150 ms. Stand it up, wait 1 s,
press **r** (RESET). If it says tipped while standing, the MPU6050 was
mounted tilted at boot. Send `LEVEL` while it's upright.

**`ERR bumped ...`**: a jolt over 1.2 g. Driving resumes after 1 s. If normal
driving triggers it, raise `BUMP_THRESHOLD_MG`.

**MOVE ends `aborted`**: `STATUS` `move_why=` says why:
- `stop`: someone sent `S`
- `drive`: a non-zero `D` took over
- `tipped` / `bump`
- `link`: the Pi stopped talking mid-move
- `timeout`: the Uno never answered. It's probably running old firmware.
- `nosteppers`: the Uno has the DC-motor build. Re-flash the default sketch.
- `reboot`: the Uno reset mid-move. This usually means a brown-out, so check the stepper supply.

**Uno keeps rebooting** (`READY again` in the Pi log): the steppers are on
the Uno's 5 V, or there's no common ground. See §2.
