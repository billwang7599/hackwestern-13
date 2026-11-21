"""Hold-to-drive WASD teleop for TARS (laptop -> Wi-Fi -> tars_motor on the Pi).

    pip install -r requirements.txt          # pygame
    TARS_HOST=<pi-ip> python teleop.py       # or: python teleop.py <pi-ip>
    python teleop.py --terminal <pi-ip>      # old terminal mode (no pygame)

A small window opens; click it so it has keyboard focus.

  hold W / S      both wheels forward / back
  hold A / D      spin in place left / right
  W+A, W+D        arc (inner wheel slower); S+A, S+D the same backwards
  W+S, A+D        cancel each other
  release         stop at once (sends S)
  space           stop        + / -   speed (1.0 = the stepper max RPM)
  r               RESET after a tip-over     l   LEVEL ("it's upright now")
  x / Esc         quit (sends S)

While any drive key is held it sends "D l r" at 20 Hz, which keeps the Pi's
500 ms and the Uno's 200 ms watchdogs fed. Losing window focus = stop.

Why a window: a terminal only sees key-repeat characters, never key-up, and
macOS waits ~0.5 s before repeating, so terminal driving stutters and stops
late. pygame gets real keydown/keyup events.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from tars_client import DEFAULT_HOST, LEFT, RIGHT, Tars, TarsError

WHEEL_MAX_RPM = 12.0          # must match WHEEL_MAX_RPM in iter1-wasd/uno/tars/tars.ino (D 1 1 = this)
SEND_HZ = 20.0
ARC_INNER = 0.4               # inner wheel speed / outer wheel speed in an arc
SPEED_STEP = 0.1
DRIVE_KEYS = ("w", "a", "s", "d")


class HoldDrive:
    """Held keys -> the D/S command stream. Pure logic, time passed in (testable).

    Every method returns the protocol lines to send, in order ("D l r" / "S").
    """

    def __init__(self, speed: float = 0.8, hz: float = SEND_HZ, arc_inner: float = ARC_INNER):
        self.held: set[str] = set()
        self.speed = speed
        self.period = 1.0 / hz
        self.arc_inner = arc_inner
        self.last_send = -1e9
        self.moving = False           # last thing we sent was a non-zero D

    # -- mixing -------------------------------------------------------------
    def wheels(self) -> tuple[float, float]:
        fwd = ("w" in self.held) - ("s" in self.held)      # W+S cancel
        turn = ("d" in self.held) - ("a" in self.held)     # A+D cancel; + = right
        v = self.speed
        if fwd and not turn:
            return fwd * v, fwd * v
        if turn and not fwd:
            return turn * v, -turn * v                     # A: left back, right fwd
        if fwd and turn:                                   # arc: inner wheel slower
            outer, inner = fwd * v, fwd * v * self.arc_inner
            return (outer, inner) if turn > 0 else (inner, outer)
        return 0.0, 0.0

    @staticmethod
    def d_line(l: float, r: float) -> str:
        return f"D {l:.3f} {r:.3f}"

    def _emit(self, t: float) -> list[str]:
        """Send the current state now (on any change)."""
        l, r = self.wheels()
        if l == 0.0 and r == 0.0:
            if self.moving:
                self.moving = False
                return ["S"]
            return []
        self.moving = True
        self.last_send = t
        return [self.d_line(l, r)]

    # -- inputs -------------------------------------------------------------
    def key_down(self, k: str, t: float) -> list[str]:
        if k not in DRIVE_KEYS or k in self.held:
            return []
        self.held.add(k)
        return self._emit(t)

    def key_up(self, k: str, t: float) -> list[str]:
        if k not in self.held:
            return []
        self.held.discard(k)
        return self._emit(t)

    def stop(self, t: float) -> list[str]:
        """Space, focus lost, quit: forget held keys, always send S."""
        self.held.clear()
        self.moving = False
        return ["S"]

    def change_speed(self, delta: float, t: float) -> list[str]:
        self.speed = round(min(1.0, max(SPEED_STEP, self.speed + delta)), 2)
        return self._emit(t) if self.moving else []

    def tick(self, t: float) -> list[str]:
        """Call often (every frame). Resends D at `hz` while moving."""
        if self.moving and t - self.last_send >= self.period - 1e-6:
            l, r = self.wheels()
            self.last_send = t
            return [self.d_line(l, r)]
        return []


# ---------------------------------------------------------------- link to the Pi
class Link:
    """Sends lines, reconnects on its own, remembers the last error for the HUD."""

    def __init__(self, host: str):
        self.host = host
        self.t: Tars | None = None
        self.status = "connecting"
        self.err = ""
        self.next_try = 0.0

    def ensure(self) -> bool:
        if self.t:
            return True
        if time.monotonic() < self.next_try:
            return False
        self.next_try = time.monotonic() + 1.0
        try:
            self.t = Tars(self.host, timeout=1.0).connect()
            self.status = f"connected ({self.t.ping():.0f} ms)"
            self.err = ""
            return True
        except OSError as e:
            self.t = None
            self.status = f"no connection: {e}"
            return False

    def send(self, line: str) -> None:
        if not self.ensure():
            return
        try:
            self.t.send(line)
            if self.err.startswith(("tipped", "bumped")) and line.startswith("D"):
                self.err = ""
        except TarsError as e:           # tipped / bumped: keep going, show why
            self.err = str(e)
        except OSError as e:
            self.status = f"lost: {e}"
            self.t = None

    def close(self) -> None:
        if self.t:
            try:
                self.t.close()            # sends S
            except OSError:
                pass
        self.t = None


# ---------------------------------------------------------------- pygame window
def run_window(host: str, speed: float) -> None:
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((460, 240))
    pygame.display.set_caption(f"TARS teleop - {host}")
    font = pygame.font.SysFont("menlo,monospace", 16)
    big = pygame.font.SysFont("menlo,monospace", 26, bold=True)
    clock = pygame.time.Clock()
    drive = HoldDrive(speed=speed)
    link = Link(host)
    link.ensure()
    names = {pygame.K_w: "w", pygame.K_a: "a", pygame.K_s: "s", pygame.K_d: "d"}

    def out(lines: list[str]) -> None:
        for line in lines:
            link.send(line)

    running = True
    try:
        while running:
            now = time.monotonic()
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                elif ev.type == pygame.KEYDOWN:
                    if ev.key in names:
                        out(drive.key_down(names[ev.key], now))
                    elif ev.key in (pygame.K_x, pygame.K_ESCAPE):
                        running = False
                    elif ev.key == pygame.K_SPACE:
                        out(drive.stop(now))
                    elif ev.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        out(drive.change_speed(+SPEED_STEP, now))
                    elif ev.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        out(drive.change_speed(-SPEED_STEP, now))
                    elif ev.key == pygame.K_r:
                        out(drive.stop(now)); link.send("RESET")
                    elif ev.key == pygame.K_l:
                        link.send("LEVEL")
                elif ev.type == pygame.KEYUP and ev.key in names:
                    out(drive.key_up(names[ev.key], now))
                elif ev.type == pygame.WINDOWFOCUSLOST:
                    if drive.held or drive.moving:
                        out(drive.stop(now))
            out(drive.tick(now))
            if not link.t:
                link.ensure()

            l, r = drive.wheels()
            screen.fill((18, 18, 24))
            col = (90, 220, 120) if link.t else (230, 90, 90)
            rows = [
                (big, f"L {l:+.2f}   R {r:+.2f}", (240, 240, 240)),
                (font, f"speed {drive.speed:.1f} = {drive.speed * WHEEL_MAX_RPM:.1f} RPM   (+/-)", (200, 200, 200)),
                (font, "keys  " + (" ".join(sorted(k.upper() for k in drive.held)) or "-"), (200, 200, 200)),
                (font, f"{host}: {link.status}", col),
                (font, f"! {link.err}" if link.err else "", (240, 180, 60)),
                (font, "WASD hold  space stop  r reset  x/Esc quit", (120, 120, 140)),
            ]
            y = 14
            for f, text, c in rows:
                screen.blit(f.render(text, True, c), (14, y))
                y += 40 if f is big else 30
            pygame.display.flip()
            clock.tick(100)               # 100 fps poll: key events land within 10 ms
    finally:
        out(drive.stop(time.monotonic()))
        link.close()
        pygame.quit()


# ---------------------------------------------------------------- terminal fallback
def run_terminal(host: str, speed: float) -> None:
    """Old mode: key-repeat in a terminal. Stops KEY_HOLD_S after the last repeat."""
    import select
    import termios
    import tty

    KEY_HOLD_S = 0.6                  # > macOS initial key-repeat delay
    ARM_STEP = 15.0
    gestures = {"g": "wave", "p": "point", "u": "shrug", "n": "nod", "t": "stance", "f": "fold"}
    arm_keys = {"q": (LEFT, -1), "e": (LEFT, +1), "z": (RIGHT, -1), "c": (RIGHT, +1)}

    def say(msg: str) -> None:
        print(f"\r{msg:<60}", end="", flush=True)

    print(f"connecting to {host}...")
    with Tars(host) as t:
        print(f"connected ({t.ping():.0f} ms). wasd hold, space stop, +/- speed, x quit")
        arm_deg = [0.0, 0.0]
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        cmd = (0.0, 0.0)
        last_key = 0.0
        last_err = ""
        try:
            while True:
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                try:
                    if r:
                        k = sys.stdin.read(1).lower()
                        last_key = time.monotonic()
                        if k == "x":
                            break
                        elif k == " ":
                            cmd = (0.0, 0.0)
                            t.stop()
                        elif k in gestures:
                            say(f"{gestures[k]} (~{t.gesture(gestures[k]):.1f} s)")
                        elif k in arm_keys:
                            arm, sign = arm_keys[k]
                            arm_deg[arm] = t.arm(arm, arm_deg[arm] + sign * ARM_STEP)
                        elif k == "0":
                            t.home(); arm_deg = [0.0, 0.0]
                        elif k == "r":
                            t.reset(); say("reset OK")
                        elif k == "l":
                            t.level(); say("level captured")
                        elif k in "+=":
                            speed = min(1.0, speed + SPEED_STEP); say(f"speed {speed:.1f}")
                        elif k == "-":
                            speed = max(SPEED_STEP, speed - SPEED_STEP); say(f"speed {speed:.1f}")
                        elif k in "wasd":
                            hd = HoldDrive(speed=speed)
                            hd.held.add(k)
                            cmd = hd.wheels()
                    if time.monotonic() - last_key > KEY_HOLD_S:
                        cmd = (0.0, 0.0)
                    t.drive(*cmd)
                    last_err = ""
                except TarsError as e:
                    if str(e) != last_err:
                        say(f"! {e}")
                        last_err = str(e)
                    cmd = (0.0, 0.0)
                    t.ping()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
            print("\nstopping")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("host", nargs="?", default=DEFAULT_HOST, help="Pi address (default $TARS_HOST or tars.local)")
    ap.add_argument("--terminal", action="store_true", help="terminal key-repeat mode (no pygame)")
    ap.add_argument("--speed", type=float, default=0.8, help="start speed, fraction of max RPM")
    a = ap.parse_args()
    if a.terminal:
        run_terminal(a.host, a.speed)
        return
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    try:
        import pygame  # noqa: F401
    except ImportError:
        sys.exit("pygame missing: pip install -r requirements.txt  (or use --terminal)")
    run_window(a.host, a.speed)


if __name__ == "__main__":
    main()
