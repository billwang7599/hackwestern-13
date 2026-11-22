"""Drive QNXplorer with a Nintendo Switch controller (Pro Controller or a Joy-Con pair).

    pip install pygame            # once (SDL2 inside reads Switch controllers on macOS)
    python switch_teleop.py [host]
    python switch_teleop.py --show   # print raw axis/button numbers, no robot

Pair the controller in macOS Bluetooth settings first (Pro Controller: hold the
small sync button on top until the lights run).

  left stick      drive (up = forward, sideways = turn)
  ZR held         turbo (full speed), ZL held = slow
  A wave   B nod   X point   Y shrug   L stance   R fold
  d-pad left/right  left arm  -/+ 15 deg     d-pad up/down  right arm +/- 15 deg
  minus  RESET (after a tip-over)     plus  HOME arms     home  quit

Button numbers depend on the SDL version and the controller; if a button does
the wrong thing, run --show, press it, and fix BUTTONS / AXES below.
If pygame or a controller is missing this prints why and exits 0 (no-op).
"""
from __future__ import annotations

import sys
import time

from tars_client import DEFAULT_HOST, LEFT, RIGHT, Tars, TarsError

# SDL2 GameController order (what pygame 2 reports for a Switch Pro Controller
# through SDL's HIDAPI driver). Edit after running --show if yours differ.
BUTTONS = {
    "A": 0, "B": 1, "X": 2, "Y": 3,
    "MINUS": 4, "HOME": 5, "PLUS": 6,
    "LSTICK": 7, "RSTICK": 8, "L": 9, "R": 10,
    "UP": 11, "DOWN": 12, "LEFT": 13, "RIGHT": 14, "CAPTURE": 15,
}
AXES = {"LX": 0, "LY": 1, "RX": 2, "RY": 3, "ZL": 4, "ZR": 5}

DEADZONE = 0.12
BASE_SPEED = 0.6
ARM_STEP = 15.0
SEND_HZ = 20

ACTIONS = {          # button -> (kind, arg)
    "A": ("gesture", "wave"), "B": ("gesture", "nod"),
    "X": ("gesture", "point"), "Y": ("gesture", "shrug"),
    "L": ("gesture", "stance"), "R": ("gesture", "fold"),
    "LEFT": ("arm", (LEFT, -1)), "RIGHT": ("arm", (LEFT, +1)),
    "UP": ("arm", (RIGHT, +1)), "DOWN": ("arm", (RIGHT, -1)),
    "MINUS": ("reset", None), "PLUS": ("home", None), "HOME": ("quit", None),
}


def dz(v: float) -> float:
    if abs(v) < DEADZONE:
        return 0.0
    return (abs(v) - DEADZONE) / (1 - DEADZONE) * (1 if v > 0 else -1)


def mix(x: float, y: float, speed: float) -> tuple[float, float]:
    """Arcade drive: stick up (y<0 in SDL) = forward, x = turn."""
    fwd, turn = -y, x * 0.7
    l, r = fwd + turn, fwd - turn
    m = max(1.0, abs(l), abs(r))
    return l / m * speed, r / m * speed


def open_joystick():
    try:
        import os
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame
    except ImportError:
        print("switch_teleop: pygame not installed (pip install pygame); nothing to do")
        return None, None
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("switch_teleop: no controller found (pair it in Bluetooth settings); nothing to do")
        return pygame, None
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"controller: {js.get_name()} ({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)")
    return pygame, js


def show(pygame, js) -> None:
    print("press buttons / move sticks, Ctrl-C to quit")
    try:
        while True:
            for ev in pygame.event.get():
                if ev.type == pygame.JOYBUTTONDOWN:
                    print(f"button {ev.button}")
                elif ev.type == pygame.JOYAXISMOTION and abs(ev.value) > 0.5:
                    print(f"axis {ev.axis} = {ev.value:+.2f}")
                elif ev.type == pygame.JOYHATMOTION:
                    print(f"hat {ev.hat} = {ev.value}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


def axis(js, name: str) -> float:
    i = AXES[name]
    return js.get_axis(i) if i < js.get_numaxes() else 0.0


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    pygame, js = open_joystick()
    if js is None:
        return
    if "--show" in sys.argv:
        show(pygame, js)
        return
    host = args[0] if args else DEFAULT_HOST
    by_number = {n: name for name, n in BUTTONS.items()}
    arm_deg = [0.0, 0.0]
    last_err = ""
    with Tars(host) as t:
        print(f"connected to {host} ({t.ping():.0f} ms)")
        print(__doc__.split("\n\n")[3])
        period = 1.0 / SEND_HZ
        while True:
            t0 = time.monotonic()
            try:
                for ev in pygame.event.get():
                    if ev.type == pygame.JOYBUTTONDOWN:
                        kind, arg = ACTIONS.get(by_number.get(ev.button, ""), (None, None))
                    elif ev.type == pygame.JOYHATMOTION:   # some SDL builds report the d-pad as a hat
                        hx, hy = ev.value
                        kind, arg = {(-1, 0): ACTIONS["LEFT"], (1, 0): ACTIONS["RIGHT"],
                                     (0, 1): ACTIONS["UP"], (0, -1): ACTIONS["DOWN"]}.get((hx, hy), (None, None))
                    else:
                        continue
                    if kind == "quit":
                        return
                    if kind == "gesture":
                        print(f"gesture {arg} (~{t.gesture(arg):.1f} s)")
                    elif kind == "arm":
                        a, sign = arg
                        arm_deg[a] = t.arm(a, arm_deg[a] + sign * ARM_STEP)
                        print(f"arms L={arm_deg[0]:+.0f} R={arm_deg[1]:+.0f}")
                    elif kind == "home":
                        t.home()
                        arm_deg = [0.0, 0.0]
                    elif kind == "reset":
                        t.reset()
                        print("reset OK")
                speed = BASE_SPEED
                if axis(js, "ZR") > 0.5:
                    speed = 1.0
                elif axis(js, "ZL") > 0.5:
                    speed = 0.3
                t.drive(*mix(dz(axis(js, "LX")), dz(axis(js, "LY")), speed))
                last_err = ""
            except TarsError as e:
                if str(e) != last_err:
                    print(f"! {e}")
                    last_err = str(e)
                t.ping()
            time.sleep(max(0.0, period - (time.monotonic() - t0)))


if __name__ == "__main__":
    main()
