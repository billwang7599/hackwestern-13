"""Focus-free input, isolated from the native motor/guard authority."""
from pathlib import Path
import subprocess
import threading
import time
from control import log_input
from keyboard_drive import KeyboardDrive

class HeadlessKeyboard:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.drive = KeyboardDrive()
        self.lock = threading.Lock()
        self.latest = (0, 0, 0)
        self.previous = 0
        self.last_keys = []
        self.last_press = []
        self.last_event = 0.0
        self.events = 0
        self.armed = False
        self.started = 0.0
        self.proc = None
        self.error = 'Waiting for keyboard'

    def run(self):
        binary = Path(__file__).resolve().parents[2] / 'iter1-wasd/pi/build/keyboard_hid'
        try:
            self.proc = subprocess.Popen(['sudo', '-n', str(binary)], stdout=subprocess.PIPE)
            for line in self.proc.stdout:
                parts = line.split()
                if len(parts) != 3: raise ValueError('Invalid keyboard record')
                stamp, attached, mask = map(int, parts)
                if attached not in (0, 1) or not 0 <= mask <= 63:
                    raise ValueError('Invalid keyboard state')
                with self.lock:
                    self.latest = stamp, attached, mask
            self.error = 'Keyboard helper stopped'
        except (OSError, ValueError) as error:
            self.error = str(error)
        finally:
            with self.lock: self.latest = (0, 0, 0)

    def read(self):
        # Snapshot the heartbeat before reading the clock. A map snapshot can
        # take longer than a keyboard heartbeat between headless loop stages.
        with self.lock:
            stamp, attached, mask = self.latest
            return self.clock(), stamp, attached, mask

    def sample(self, permitted):
        now, stamp, attached, mask = self.read()
        fresh = bool(attached and stamp and 0 <= now - stamp / 1e6 < .3)
        # Neutral report is required after attach, stale input, or a stop.
        if not fresh:
            if self.drive.claimed: self.drive.stop()
            self.armed = False
            self.previous = mask
            return False
        self.error = None
        if mask != self.previous:
            self.events += 1
            keys = [name for bit, name in ((1,'W'),(2,'S'),(4,'A'),(8,'D'),(16,'STOP'),(32,'F')) if mask & bit]
            self.last_keys = keys or ['RELEASE']
            log_input("keyboard", {"keys": self.last_keys, "permitted": bool(permitted)})
            if keys: self.last_press = keys
            self.last_event = now
        if mask == 0:
            self.armed = True
            self.started = 0
            if self.previous: self.drive.stop()
        elif mask & 16:
            self.drive.stop(); self.armed = False
        elif mask == 32 and self.previous != mask:
            self.drive.stop(); self.drive.claimed = False; self.armed = False
        elif mask in (1, 2, 4, 8):
            if self.armed and mask != self.previous:
                key = {1:'w', 2:'s', 4:'a', 8:'d'}[mask]
                self.drive.event(0x300, ord(key))
                self.started = now
            # A wireless receiver may keep reporting a lost radio key: bound each hold.
            if not permitted or (self.started and now-self.started >= 2):
                self.drive.stop(); self.armed = False
        else:
            self.drive.stop(); self.armed = False
        self.previous = mask
        return fresh

    def take_phone_control(self):
        self.drive.stop()
        self.drive.claimed = False
        self.armed = False

    def snapshot(self):
        now, stamp, attached, mask = self.read()
        fresh = bool(attached and stamp and 0 <= now - stamp / 1e6 < .3)
        keys = [name for bit, name in ((1,'W'),(2,'S'),(4,'A'),(8,'D'),(16,'STOP'),(32,'F')) if mask & bit]
        return dict(connected=fresh, keys=keys if fresh else [], last_keys=self.last_keys, last_press=self.last_press,
                    event_count=self.events, last_event_age_ms=round((now-self.last_event)*1000) if self.last_event else None,
                    owner='keyboard' if self.drive.claimed else 'phone',
                    active=chr(self.drive.key).upper() if self.drive.key else None,
                    error=self.error, max_hold_s=2)

    def close(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try: self.proc.wait(timeout=1)
            except subprocess.TimeoutExpired: self.proc.kill()
