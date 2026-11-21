# TARS motor co-processor: Raspberry Pi Pico (MicroPython) + TB6612FNG.
#
# Reads lines from the Pi 5 over UART0 and drives two DC motors.
#   "D <l> <r>"   drive, floats in [-1, 1]
#   "S"           stop
#   "G <name>"    gesture on the servo (optional)
# If nothing arrives for WATCHDOG_MS, the motors stop (the Pi resends D at 50 Hz).
#
# ---- PIN CHOICES (change here) -------------------------------------------
#  UART0  TX=GP0 -> Pi 5 GPIO15/RXD (pin 10)
#         RX=GP1 <- Pi 5 GPIO14/TXD (pin 8)          GND <-> GND (pin 6)
#  TB6612FNG:  PWMA=GP2  AIN1=GP3  AIN2=GP4  STBY=GP5
#              BIN1=GP6  BIN2=GP7  PWMB=GP8
#  Servo (slab gesture, optional): GP15 (50 Hz). Power it from 5-6 V, not the Pico.
#  LED: onboard ("LED"), on while motors are commanded.
# ---------------------------------------------------------------------------
from machine import Pin, PWM, UART
import time

UART_ID, UART_TX, UART_RX, BAUD = 0, 0, 1, 115200
PWMA, AIN1, AIN2, STBY, BIN1, BIN2, PWMB = 2, 3, 4, 5, 6, 7, 8
SERVO_PIN = 15
MOTOR_PWM_HZ = 20_000
WATCHDOG_MS = 300
DEADBAND = 0.05          # ignore tiny commands (motors just hum)
INVERT_LEFT = False      # flip if a wheel spins the wrong way
INVERT_RIGHT = False

uart = UART(UART_ID, baudrate=BAUD, tx=Pin(UART_TX), rx=Pin(UART_RX))
led = Pin("LED", Pin.OUT)
stby = Pin(STBY, Pin.OUT, value=1)


class Motor:
    def __init__(self, pwm_pin, in1, in2, invert):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(MOTOR_PWM_HZ)
        self.in1 = Pin(in1, Pin.OUT, value=0)
        self.in2 = Pin(in2, Pin.OUT, value=0)
        self.invert = invert
        self.set(0)

    def set(self, v):
        v = max(-1.0, min(1.0, -v if self.invert else v))
        if abs(v) < DEADBAND:
            self.in1.value(0)
            self.in2.value(0)            # coast; use 1/1 for brake
            self.pwm.duty_u16(0)
            return
        self.in1.value(1 if v > 0 else 0)
        self.in2.value(0 if v > 0 else 1)
        self.pwm.duty_u16(int(abs(v) * 65535))


left = Motor(PWMA, AIN1, AIN2, INVERT_LEFT)
right = Motor(PWMB, BIN1, BIN2, INVERT_RIGHT)

servo = PWM(Pin(SERVO_PIN))
servo.freq(50)


def servo_deg(deg):
    # 0..180 deg -> 0.5..2.5 ms pulse
    us = 500 + int(max(0, min(180, deg)) * 2000 / 180)
    servo.duty_ns(us * 1000)


def stop():
    left.set(0)
    right.set(0)
    led.value(0)


def gesture(name):
    # Blocking is OK-ish here: keep gestures short (< WATCHDOG_MS between moves
    # doesn't matter because motors are stopped during a gesture).
    stop()
    if name == "wave":
        for d in (60, 120, 60, 120, 90):
            servo_deg(d)
            time.sleep_ms(200)
    elif name == "nod":
        for d in (70, 90):
            servo_deg(d)
            time.sleep_ms(250)
    else:
        servo_deg(90)


def handle(line):
    parts = line.split()
    if not parts:
        return
    cmd = parts[0].upper()
    if cmd == "D" and len(parts) == 3:
        try:
            l, r = float(parts[1]), float(parts[2])
        except ValueError:
            return
        left.set(l)
        right.set(r)
        led.value(1 if (abs(l) > DEADBAND or abs(r) > DEADBAND) else 0)
    elif cmd == "S":
        stop()
    elif cmd == "G" and len(parts) >= 2:
        gesture(parts[1])


def main():
    stop()
    servo_deg(90)
    buf = b""
    last = time.ticks_ms()
    while True:
        n = uart.any()
        if n:
            buf += uart.read(n)
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                try:
                    handle(raw.decode().strip())
                except Exception as e:  # never let a bad line kill the loop
                    print("bad line:", raw, e)
                last = time.ticks_ms()
            if len(buf) > 128:
                buf = b""
        if time.ticks_diff(time.ticks_ms(), last) > WATCHDOG_MS:
            stop()
            last = time.ticks_ms()   # don't spam stop()
        time.sleep_ms(2)


try:
    main()
finally:
    stop()
    stby.value(0)
