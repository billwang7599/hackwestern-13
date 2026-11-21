"""Nintendo Switch Pro controller / Joy-Con (Bluetooth) -> human goalie.

Pair the controller with the Mac first (System Settings > Bluetooth; Pro
controller: hold the sync button on top; Joy-Con: hold the small sync button
on the rail). Then:

    sw = SwitchInput()           # never raises; sw.available is False without a pad
    sw.poll()                    # once per camera frame (must be the main thread on macOS)
    sw.x                         # left stick x scaled to [-GOAL_HALF_W, +GOAL_HALF_W]
    sw.toggled                   # True once each time the toggle button is pressed

    python switch_input.py       # debug: prints axes/buttons so you can find indices

SDL button numbering differs between the Pro controller, single Joy-Cons and
SDL versions, so the toggle button and stick axis are parameters (defaults:
any of buttons 0/1 = A/B on a Pro controller, axis 0 = left stick x).
"""
from __future__ import annotations

import os
import sys

from field import GOAL_HALF_W

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")            # don't fight OpenCV for the window system
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

DEADZONE = 0.12


class SwitchInput:
    def __init__(self, axis: int = 0, toggle_buttons=(0, 1), invert: bool = False):
        self.axis, self.toggle_buttons, self.invert = axis, tuple(toggle_buttons), invert
        self.available = False
        self.name = ""
        self.x = 0.0
        self.toggled = False
        self._pg = None
        self._js = None
        try:
            import pygame
            pygame.display.init()
            pygame.joystick.init()
            self._pg = pygame
            self._open()
        except Exception as e:      # no pygame, no SDL, no permission...
            print(f"[switch] controller support unavailable: {e}")

    def _open(self):
        pg = self._pg
        if pg.joystick.get_count() == 0:
            return
        self._js = pg.joystick.Joystick(0)
        self._js.init()
        self.name = self._js.get_name()
        self.available = True
        print(f"[switch] using controller: {self.name}")

    def poll(self) -> None:
        """Pump SDL events, update x and toggled. Cheap; call every frame."""
        self.toggled = False
        pg = self._pg
        if pg is None:
            return
        try:
            for ev in pg.event.get():
                if ev.type == pg.JOYDEVICEADDED and not self.available:
                    self._open()
                elif ev.type == pg.JOYDEVICEREMOVED:
                    self.available, self._js = False, None
                    print("[switch] controller disconnected")
                elif ev.type == pg.JOYBUTTONDOWN and ev.button in self.toggle_buttons:
                    self.toggled = True
            if self._js is not None and self.axis < self._js.get_numaxes():
                v = self._js.get_axis(self.axis)
                v = 0.0 if abs(v) < DEADZONE else v
                self.x = max(-1.0, min(1.0, -v if self.invert else v)) * GOAL_HALF_W
        except Exception as e:
            print(f"[switch] poll error: {e}")
            self.available = False


if __name__ == "__main__":
    import time
    sw = SwitchInput()
    if not sw.available:
        print("no controller found (pair it over Bluetooth, then re-run); waiting for one to connect...")
    js_last = None
    try:
        while True:
            sw.poll()
            if sw._js is not None:
                js = sw._js
                axes = " ".join(f"{js.get_axis(i):+.2f}" for i in range(js.get_numaxes()))
                btn = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
                line = f"axes [{axes}] buttons {btn} -> x={sw.x:+.3f}{'  TOGGLE' if sw.toggled else ''}"
                if line != js_last:
                    print(line)
                    js_last = line
            time.sleep(0.02)
    except KeyboardInterrupt:
        sys.exit(0)
