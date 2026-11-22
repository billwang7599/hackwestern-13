"""SDL keyboard intent. Key release, lost focus, or guard denial requires a new press."""
import time


class KeyboardDrive:
    def __init__(self):
        self.key = None
        self.shifted = False
        self.expires = 0.0
        self.claimed = False
        self.generation = 0

    def stop(self):
        self.shifted = False
        self.key = None
        self.claimed = True

    def event(self, kind, key=0, repeat=False, window_event=0, modifiers=0):
        if kind == 0x200 and window_event in (7, 13, 14):  # minimize/focus lost/close
            self.stop()
        elif kind == 0x301 and self.shifted and key in (0x400000e1, 0x400000e5):
            self.stop()
        elif kind == 0x301 and key in (ord('w'), ord('a'), ord('s'), ord('d')):
            self.stop()
        elif kind == 0x300 and not repeat:
            if key in (32, 27):
                self.stop()
            elif key in (ord('w'), ord('a'), ord('s'), ord('d')):
                self.shifted = bool(modifiers & 3) and not bool(modifiers & 0xfc0)
                self.expires = time.monotonic() + 5
                self.key = key
                self.claimed = True
                self.generation += 1

    def state(self, permitted):
        if not permitted or (self.shifted and time.monotonic() >= self.expires):
            self.stop()
        if self.key is None:
            return 0.0, 0.0
        return {ord('w'): (1.0, 1.0), ord('s'): (-1.0, -1.0),
                ord('a'): (-1.0, 1.0), ord('d'): (1.0, -1.0)}[self.key]
