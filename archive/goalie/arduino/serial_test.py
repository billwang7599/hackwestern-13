#!/usr/bin/env python3
"""Bench test for the goalie Uno firmware (laptop or Pi, no QNX needed).

    pip install pyserial
    python serial_test.py /dev/cu.usbmodem1101            # macOS
    python serial_test.py /dev/ttyACM0 --sweep-only       # Linux
    python serial_test.py PORT --no-motor                 # just stepper/flag/HIT

What it does, printing every line the Uno sends (READY / HIT / A / # ...):
  1. waits for READY (opening the port resets the Uno)
  2. goalie motor sweep: M from 0 -> +max -> 0 -> -max -> 0 at 200 Hz, like the Pi
  3. watchdog check: one M 0.5, then silence -> motor must stop by itself ~200 ms later
  4. flag: C 800
  5. needle: N 1024 (quarter turn), N 0, then Z
  6. listens for HITs for a few seconds: tap the paddle / table
Ctrl-C at any point sends S (stop all).
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

RATE_HZ = 200


class Uno:
    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ready = threading.Event()
        self.hits = 0
        self._stop = False
        self._t = threading.Thread(target=self._reader, daemon=True)
        self._t.start()

    def _reader(self) -> None:
        buf = b""
        while not self._stop:
            try:
                buf += self.ser.read(64)
            except (serial.SerialException, OSError, TypeError) as e:
                if not self._stop:
                    print(f"!! serial error: {e}")
                return
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip()
                if not line:
                    continue
                if line == "READY":
                    self.ready.set()
                elif line.startswith("HIT "):
                    self.hits += 1
                print(f"   uno> {line}")

    def send(self, line: str) -> None:
        self.ser.write((line + "\n").encode("ascii"))

    def close(self) -> None:
        try:
            self.send("S")
            time.sleep(0.05)
        finally:
            self._stop = True
            self._t.join(timeout=0.5)  # reader exits within one read timeout
            self.ser.close()


def sweep(uno: Uno, max_duty: float, seconds: float) -> None:
    """Triangle 0 -> +max -> 0 -> -max -> 0, sent at the Pi's control rate."""
    n = int(seconds * RATE_HZ)
    t0 = time.monotonic()
    for i in range(n + 1):
        p = i / n  # 0..1
        tri = 4 * p if p < 0.25 else 2 - 4 * p if p < 0.75 else 4 * p - 4
        uno.send(f"M {max_duty * tri:.3f}")
        # absolute-time pacing so the rate holds even if a write is slow
        next_t = t0 + (i + 1) / RATE_HZ
        time.sleep(max(0.0, next_t - time.monotonic()))
    uno.send("M 0")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--max", type=float, default=0.6, help="peak sweep duty (default 0.6)")
    ap.add_argument("--seconds", type=float, default=4.0, help="sweep duration")
    ap.add_argument("--no-motor", action="store_true", help="skip the goalie motor sweep")
    ap.add_argument("--sweep-only", action="store_true")
    ap.add_argument("--listen", type=float, default=5.0, help="seconds to listen for HITs at the end")
    a = ap.parse_args()

    uno = Uno(a.port, a.baud)
    try:
        print(">> waiting for READY (the Uno resets when the port opens)...")
        if not uno.ready.wait(4.0):
            print("!! no READY within 4 s: wrong port/baud, or the Uno was already running."
                  " Continuing anyway.")

        if not a.no_motor:
            print(f">> sweep: M 0 -> {a.max} -> -{a.max} -> 0 over {a.seconds:.1f}s at {RATE_HZ} Hz."
                  " The paddle should swing towards +x first.")
            sweep(uno, a.max, a.seconds)
            time.sleep(0.3)
        if a.sweep_only:
            return 0

        if not a.no_motor:
            print(">> watchdog: one 'M 0.5' then silence. Motor should twitch and stop by itself (~200 ms).")
            uno.send("M 0.5")
            time.sleep(1.0)

        print(">> flag: C 800")
        uno.send("C 800")
        time.sleep(1.2)

        print(">> needle: N 1024 (~quarter turn), back to N 0, then Z")
        uno.send("N 1024")
        time.sleep(2.0)
        uno.send("N 0")
        time.sleep(2.0)
        uno.send("Z")

        print(f">> tap the paddle/table for {a.listen:.0f}s: expect 'HIT <mg>' lines")
        time.sleep(a.listen)
        print(f">> done. HITs seen: {uno.hits}")
    except KeyboardInterrupt:
        print("\n>> interrupted, stopping")
    finally:
        uno.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
