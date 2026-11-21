# Building TARS (wheeled, stands on its own, stepper arms)

A desk-scale TARS: four tall foam-board slabs. The **two middle slabs** are the
body and sit on a base with two drive wheels and a caster, so it stands on its
own with no balancing loop. The **two outer slabs** are the arms. Each one hangs
from a 28BYJ-48 stepper and swings forward/back for gestures (wave, point,
shrug, nod, the split "stance", fold).

```
  front view                         side view (arm swung +30 deg)
  ┌──┐┌──┐┌──┐┌──┐                         ╱╲
  │  ││  ││  ││  │   30 cm                 ╱  ╲ <- outer slab, pivot at its middle
  │A0││B ││B ││A1│                    ┌──╳──┐    (stepper shaft = ╳)
  │  ││  ││  ││  │                    │  B  │ <- middle slab
  │  ││  ││  ││  │                    │     │
  └──┘│  ││  │└──┘  <- arms clear     ├─────┤ <- base plate
      └┬─┘└─┬┘        the floor 1 cm  (W)   (c)  W = drive wheel, c = caster
      (W)  (W)
```

## 1. Slabs and dimensions

| Part | Size | Notes |
|---|---|---|
| Middle slabs (×2) | 30 × 7.5 cm, 5 mm foam board | glue them together **with a 2–3 cm spacer box between them** to hold the electronics |
| Outer slabs / arms (×2) | 26 × 7.5 cm | shorter so they clear the floor when they swing |
| Base plate | ~12 × 10 cm, stiff cardboard or 2 layers of foam board | under the middle slabs, carries motors, caster, battery |
| Depth of all slabs | 3–4 cm boxes (fold foam board into a hollow box) or flat 5 mm boards | hollow boxes look more like TARS but weigh more |

Keep the **arm slabs light** (see the torque section). A flat 26 × 7.5 cm piece of
5 mm foam board weighs about 6–8 g. Paint and tape add more than you'd think.

## 2. Wheels, caster, standing on its own

- The two DC gearmotors mount **under the base plate**, shafts pointing out
  sideways, one under each middle slab. Hot glue plus a zip tie around each
  gearbox holds them well enough for a demo.
- **Wheels:** your parts list doesn't include any. Options: the gearmotors' own
  wheels if they came with some, bottle caps or 3D-printed hubs with a rubber-band
  tyre, or two CDs glued to a hub. About 6–7 cm diameter suits a 30 cm robot.
- **Caster:** one ball caster or a smooth sliding foot (a plastic bead, the
  rounded back of a spoon, a furniture slider) at the back of the base plate,
  so it rests on three points: wheel, wheel, caster. Add a second one at the
  front if it rocks forward when braking.
- **Centre of mass low:** put the battery pack flat on the base plate, as low and
  as central as possible. 30 cm tall on a 10 cm footprint tips easily. The
  accelerometer will catch it, but it's better if it doesn't fall.
- **Arms do not touch the floor.** Their pivot is at mid-height of the body
  (~16 cm up), so a 26 cm arm hangs with its bottom ~3 cm above the floor and
  never drags whatever angle it's at.

> The brief said "outer slabs rest on casters/feet". Swinging arms can't also be
> the feet, so here the body stands on its own base plate and the arms hang free.
> If you'd rather have the outer slabs carry weight, the arms can only do small
> moves. Keep the base plate either way.

## 3. Arms: 28BYJ-48 mounting and torque limit

- Glue (or screw, using the two mounting ears) each stepper to the **outside
  face of a middle slab**, shaft pointing outward, at **~16 cm height** (the
  middle of the arm).
- **Hub:** the shaft is 5 mm with two flats. Push it into a tight hole in a
  bottle cap, a 3D-printed hub or a stack of 3 card circles, and glue it. Then
  glue the arm slab to the hub **at the arm's own centre**.
- **Torque budget:** the 28BYJ-48 makes only about **0.3 kg·cm** (less at
  speed). If an arm is pivoted at its centre, gravity barely loads it, so
  nearly all of that goes into moving it. Pivoted at one end, a 10 g, 26 cm slab
  needs 10 g × 13 cm = 0.13 kg·cm just to hold it horizontal, and any decoration
  makes the motor skip steps. **Keep arms light and pivot them near their centre of mass.**
- **Speed:** about 60°/s at most (the firmware caps it at 60°/s, default 45°/s).
  A wave takes ~6.5 s. That's slow, but in character for TARS.
- **Zero:** the steppers have no end-stop. Before powering up, hang both arms
  straight (flush with the body). Power-up position = 0°. If they drift, move
  them back by hand and send `ZERO`. Do the same if the Uno ever reboots (the
  Pi logs "co-processor READY again").
- **Direction:** `+deg` should swing the top of the slab forward on both sides.
  Arm 1 is mirrored in firmware (`ARM_INVERT = {false, true}`). If one goes the
  wrong way, flip its entry.

## 4. Power (read this before plugging anything in)

| Load | Supply | Why |
|---|---|---|
| 2 DC gearmotors | **separate motor battery** (4×AA = 6 V, or a pack matching the motors) into the H-bridge's motor input | motor current spikes reset the Uno and the Pi if they share a supply |
| 2 × ULN2003 + 28BYJ-48 (5 V, ~240 mA each while moving) | **separate 5 V**: a USB power bank with a cut USB cable, or a 5 V buck from the motor battery | **never from the Uno's 5 V pin**. Two steppers is ~0.5 A, more than the Uno's USB or regulator should supply |
| Uno | USB from the Pi (that's also the data link) | |
| Accelerometer | Uno 3.3 V (GY-291/ADXL345) or 5 V (GY-521/MPU6050 has its own regulator) | a few mA |
| Pi 5 | official 27 W USB-C supply | on the desk, tethered. A Pi 5 on a power bank browns out under load |

**Common ground:** connect the GND of the motor battery, the stepper 5 V supply,
the H-bridge, both ULN2003 boards and the Uno together. Without it the logic
signals have no reference and the motors twitch or don't move.

The H-bridge's "1.3 V" marking is probably a saturation-voltage spec, not the
motor voltage. Check the chip markings before you pick a battery (see
`iter1-wasd/uno/tars/tars.ino` CONFIG block for how to identify the driver type).

## 5. Uno wiring

### Stepper wheels (the default build, `WHEEL_TYPE WHEELS_STEPPER`)

The two 28BYJ-48s drive the wheels (exact moves and turns), so there are **no
stepper arms** by default. 8 wheel pins + button + LED + buzzer leave room for
at most one arm (`ARM_COUNT 1`, on A0–A3). Full checklist: [PI_TEST.md](PI_TEST.md).

| Uno pin | To |
|---|---|
| D8, D9, D10, D11 | left wheel ULN2003 IN1–IN4 |
| D4, D5, D6, D7 | right wheel ULN2003 IN1–IN4 |
| A4 / A5 | accelerometer SDA / SCL |
| D2 | TEACH button → GND (internal pull-up) |
| D13 | status LED (+ 220 Ω) → GND; also the on-board "L" LED |
| D12 | buzzer + (active by default; `BUZZER_PASSIVE` for a piezo) |
| D0, D1 | USB serial to the Pi |
| free | D3, A0–A3 (one optional arm) |

ULN2003 `+` go to a **separate 5 V supply (≥1 A)**, never the Uno's 5 V pin, with all grounds common.
Wheel geometry: `K <diam_mm> <track_mm>` on the serial monitor (or `WHEELCAL` via the
Pi) saves it in EEPROM. 28BYJ-48 gear ratio is 63.684:1 → **4075.77 half-steps/rev**,
not 4096. At 12 RPM a 65 mm wheel does ~4 cm/s; 100 mm wheels ~6 cm/s.

### DC wheels (`WHEEL_TYPE WHEELS_DC`, the original build)

Button D2 / LED D13 / buzzer D12 in 2-PWM mode; button D10 / LED D13 / no buzzer in EN+IN mode.

Firmware: `iter1-wasd/uno/tars/tars.ino`. Pins are set in its CONFIG block. Arm 1 uses
**D4–D7 in both driver modes**, so switching driver type never means rewiring the arms.

**Default: DRIVER_2PWM** (L9110S / HG7881, DRV8833, MX1508: four inputs, no enable pins)

| Uno pin | To | Function |
|---|---|---|
| D9 | driver A-IA / AIN1 / IN1 | left wheel |
| D10 | driver A-IB / AIN2 / IN2 | left wheel |
| D3 | driver B-IA / BIN1 / IN3 | right wheel |
| D11 | driver B-IB / BIN2 / IN4 | right wheel |
| A0, A1, A2, A3 | ULN2003 #0 IN1–IN4 | left arm (arm 0) |
| D4, D5, D6, D7 | ULN2003 #1 IN1–IN4 | right arm (arm 1) |
| A4 (SDA), A5 (SCL) | accelerometer SDA, SCL | tip / bump |
| D0, D1 | (USB serial to the Pi) | keep free |
| free | D2, D8, D12, D13 | |

**DRIVER_EN_IN** (L298N: remove the ENA/ENB jumpers; TB6612FNG: tie STBY high)

| Uno pin | To | Function |
|---|---|---|
| D9 (PWM) | ENA / PWMA | left speed |
| D8, D12 | IN1, IN2 / AIN1, AIN2 | left direction |
| D3 (PWM) | ENB / PWMB | right speed |
| D2, D11 | IN3, IN4 / BIN1, BIN2 | right direction |
| A0–A3, D4–D7, A4/A5 | same as above | arms, accelerometer |
| free | D10, D13 | |

PWM is only on Timer1 (D9/D10) and Timer2 (D3/D11). D5/D6 (Timer0, which runs
`millis()`) are used as plain digital outputs for arm 1, which is fine.

**Accelerometer:** the firmware auto-detects, first match wins, and the
boot line `# adxl345 at 0x53` or `# mpu6050 at 0x68` says which it found:

| Board | Address | Notes |
|---|---|---|
| ADXL345 / GY-291 ("3-axis digital accelerometer") | 0x53 (SDO→GND, the usual), 0x1D (SDO→VCC) | CS must be high for I2C (most boards pull it up). Full-res ±16 g |
| MPU6050 / GY-521 | 0x68 (AD0 low), 0x69 | ±8 g |

Mount it **flat on the base plate or low on the body**, not on an arm, or arm
swings will register as bumps. Boot TARS **standing upright**: the firmware
takes the boot orientation as "upright". Otherwise send `LEVEL` once it's standing.

**ULN2003 boards:** `+` to the separate 5 V, `-` to common GND. The 5-pin
plug goes straight into the 28BYJ-48.

## 6. Pi ↔ Uno link (QNX)

- **USB (default):** plug the Uno's USB into the Pi 5. QNX needs a USB CDC-ACM
  serial driver for it to show up (the Uno R3's 16U2 chip is CDC-ACM, and
  clones with a CH340 chip are a different driver). The device name is
  **unknown until you try**: `/dev/serusb1` is a guess and is the default for
  `-b uno`. On the Pi: `ls /dev/ser*` before and after plugging in. Then
  `make -C iter1-wasd/pi run UART=/dev/serusbN`.
  - If nothing appears, check `pidin ar | grep -i usb` and whether
    `devc-serusb` is running/available on your image (start it by hand if it
    exists but isn't running).
  - Opening the port reboots the Uno (DTR). It says `READY` about 1.5 s later.
    The Pi just keeps sending.
- **Fallback: header UART.** Pi pin 8 (GPIO14 TXD) → Uno D0 (RX). Uno D1 (TX) →
  **voltage divider** → Pi pin 10 (GPIO15 RXD): Uno TX → 1 kΩ → junction →
  2 kΩ → GND, junction → Pi RX (5 V → 3.3 V; **the Pi's GPIO is not 5 V tolerant**).
  Pi pin 6 GND ↔ Uno GND. The Pi's 3.3 V TX is high enough for the Uno.
  Then use `-d /dev/ser1` (or whichever `/dev/serN` is the header UART, see the
  README checklist). With this wiring, **unplug the Uno's USB** (it shares D0/D1)
  and disconnect D0 while uploading a sketch.

## 7. Bring-up order on the bench

1. **Uno alone, USB to the Mac:** upload `iter1-wasd/uno/tars/tars.ino`, open the Serial
   Monitor at 115200 with "Newline" line endings. Check the boot lines (`# adxl345 at 0x53`
   or `# mpu6050 at 0x68`, then `READY`). Type `?`.
2. **Arms:** with 5 V on the ULN2003 boards, `A 0 45`, `A 1 45`, `H`. Both
   should swing forward and back. Flip `ARM_INVERT` if one goes backwards.
3. **Wheels** (robot on a box, wheels in the air): `D 0.5 0.5` then quickly
   again (the 200 ms watchdog stops it). Both should spin forward. Flip
   `LEFT_INVERT`/`RIGHT_INVERT` if not. Raise `D x x` until it creeps on
   the floor and put that in `DRIVE_MIN_DUTY`.
4. **Tip:** tilt the robot past ~40° and hold it: `T <deg>` lines, motors stop.
   Stand it up and after 0.5 s you get `# upright`.
5. **Bump:** tap the body. You should see `B <mg>`. Drive on carpet/desk and
   make sure driving alone doesn't trigger it. Raise `BUMP_THRESHOLD_MG` if it does.
6. **Pi:** `make -C iter1-wasd/pi deploy run`, then `python iter1-wasd/laptop/teleop.py`.
