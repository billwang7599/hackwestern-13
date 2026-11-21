"""SDL keyboard intent. Key release, lost focus, or guard denial requires a new press."""
class KeyboardDrive:
    def __init__(self):
        self.key = None
        self.claimed = False
        self.generation = 0

    def stop(self):
        self.key = None
        self.claimed = True

    def event(self, kind, key=0, repeat=False, window_event=0):
        if kind == 0x200 and window_event in (7, 13, 14):  # minimize/focus lost/close
            self.stop()
        elif kind == 0x301 and key in (ord('w'), ord('a'), ord('s'), ord('d')):
            self.stop()
        elif kind == 0x300 and not repeat:
            if key in (32, 27):
                self.stop()
            elif key in (ord('w'), ord('a'), ord('s'), ord('d')):
                self.key = key
                self.claimed = True
                self.generation += 1

    def state(self, permitted):
        if not permitted:
            self.stop()
        if self.key is None:
            return 0.0, 0.0
        return {ord('w'): (.25, .25), ord('s'): (-.25, -.25),
                ord('a'): (-.25, .25), ord('d'): (.25, -.25)}[self.key]
