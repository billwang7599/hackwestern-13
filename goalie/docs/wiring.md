# Wiring

```
 Laptop ──Wi-Fi/Ethernet TCP :5556──> Pi 5 (QNX) ──USB (or UART)──> Uno ──> driver ──> goalie motor (A)
 (webcam on the laptop)                                               │              └─> flag motor (B)
                                                                      ├──> ULN2003 ──> 28BYJ-48 (needle)
                                                                      └──> MPU6050 (on the goal frame)
 Battery pack ──> driver VM/VCC (motors)        5 V supply ──> ULN2003 +
 ALL GROUNDS TOGETHER: battery -, driver GND, ULN2003 -, Uno GND
```

The full per-board detail (how to identify the driver chip, EN+IN variant,
ADXL345 swap) is in [`arduino/README.md`](../arduino/README.md). This page is
the bench summary.

## Uno pin map (default `DRIVER_2PWM`: L9110S / DRV8833 / MX1508)

| Uno pin | Goes to | Function |
|---|---|---|
| D9 (PWM, Timer1) | driver A-IA / AIN1 / IN1 | goalie motor A |
| D10 (PWM, Timer1) | driver A-IB / AIN2 / IN2 | goalie motor A |
| D3 (PWM, Timer2) | driver B-IA / BIN1 / IN3 | flag motor B |
| D11 (PWM, Timer2) | driver B-IB / BIN2 / IN4 | flag motor B |
| A0, A1, A2, A3 | ULN2003 IN1..IN4 | 28BYJ-48 scoreboard needle |
| A4 (SDA), A5 (SCL) | MPU6050 SDA, SCL | hit detection (I2C 0x68, 0x69 also found) |
| 5V | MPU6050 VCC (GY-521 has its own 3.3 V regulator) | logic only |
| GND | driver GND, ULN2003 `-`, MPU6050 GND, battery `-` | **common ground** |
| USB | Pi 5 USB port | power + serial 115200 8N1 |

Pins 5/6 are avoided on purpose (Timer0 runs `millis()`).

**L298N / TB6612FNG (`DRIVER_EN_IN`):** goalie = EN D9, IN D7/D8; flag = EN D3,
IN D4/D12. On the L298N remove the ENA/ENB jumpers; on the TB6612 tie STBY to 5 V.
Change `DRIVER_TYPE` in the sketch's CONFIG block.

## Power

- **Motors get their own supply** (4xAA or a 2S pack) into the driver's
  VM/VCC terminal. Never power motors from the Uno 5V pin: stall current resets
  the Uno (you'll see a mid-run `READY` in the Pi log) or damages the USB port.
- **ULN2003 `+` from a separate 5 V** (or the battery through a 5 V regulator).
  The 28BYJ-48 draws ~240 mA with coils on.
- **Common ground** between battery `-`, driver GND, ULN2003 `-` and Uno GND.
  Without it the PWM signals have no reference and the motor twitches randomly.
- Check the driver's voltage rating before picking the battery (L9110S:
  2.5-12 V, ~800 mA/channel). The "1.3 V" printed on the board is probably not
  the motor voltage.
- Mount the MPU6050 on the **goal frame**, not the paddle (the paddle's own
  swings look like hits).

## Pi 5 <-> Uno link

### Primary: USB serial

Plug the Uno into a Pi 5 USB port (blue USB 3 ports preferred). On QNX the
Uno's ATmega16U2 (CDC-ACM) should appear as `/dev/serusb1`, which is the
`goalie_motor` default (`-d`). **Not verified on the QNX 8.0 Pi 5 image yet.**
Check on the Pi:

```sh
ls /dev/ser*                # with and without the Uno plugged in
pidin ar | grep -i usb      # io-usb-otg and devc-serusb must be running
```

If nothing appears, start the USB serial driver (`devc-serusb &`) or use the
UART fallback below. Opening the port resets the Uno; it sends `READY` ~1.5 s
later. Motor commands before that are lost, which is harmless (sent at 200 Hz).

### Fallback: UART pins (if USB serial doesn't enumerate)

| Uno | Pi 5 header | Note |
|---|---|---|
| TX (D1) | **through a divider** -> GPIO15 / RXD (pin 10) | Uno TX is 5 V, the Pi is 3.3 V only |
| RX (D0) | GPIO14 / TXD (pin 8) | 3.3 V from the Pi is read as HIGH by the Uno: no level shifter needed |
| GND | GND (pin 6) | common ground |

Divider on the Uno TX line: **Uno TX -> 1 kΩ -> Pi RX, and Pi RX -> 2 kΩ -> GND**
(5 V × 2/3 = 3.3 V; 10 k/20 k also works at 115200). Then run
`goalie_motor -d /dev/ser1` (check the device name on your image). Power the
Uno from its own USB supply or barrel jack in this case, and unplug D0/D1 while
uploading sketches (they share the USB serial lines).

## Laptop <-> Pi

TCP port 5556 over the same network (Wi-Fi or a direct Ethernet cable). Set
`GOALIE_HOST` on the laptop if `goalie.local` doesn't resolve:
`export GOALIE_HOST=192.168.1.42`.
