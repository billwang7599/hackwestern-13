#!/usr/bin/env python3
"""Virtual SM11 through the REAL host tars_motor (unchanged, -b uno -d <pty>).

    python3 bench/tests/test_car.py          (builds bench/car + iter1-wasd/pi host)

What runs: iter1-wasd/uno/tars/tars.ino (CAR_SM11) compiled for the Mac with the
firmware test mocks, bench/car/physics.h, and tars_motor_host. Every port is free
(picked at start). Laptop only: none of this has touched a real Uno, car or Pi.
"""
from __future__ import annotations

import json
import math
import os
import select
import signal
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
BENCH = os.path.dirname(HERE)
REPO = os.path.dirname(BENCH)
sys.path.insert(0, os.path.join(BENCH, "car"))
import launch as car  # noqa: E402

LAPTOP = os.path.join(REPO, "iter1-wasd", "laptop")

fails = 0
total = 0


def check(ok: bool, what: str) -> None:
    global fails, total
    total += 1
    fails += 0 if ok else 1
    print(("  ok   " if ok else "  FAIL ") + what, flush=True)


def free_port(kind=socket.SOCK_STREAM) -> int:
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def load() -> str:
    return os.getloadavg().__repr__()


class Rig:
    """vuno + tars_motor on free ports, a truth listener, a client connection."""

    def __init__(self, seed: int = 1, watchdog_ms: int = 300):
        self.tr = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tr.bind(("127.0.0.1", 0))
        self.ctl_port = free_port(socket.SOCK_DGRAM)
        self.port = free_port()
        self.log = open(os.path.join(tempfile.gettempdir(), f"test_car_{self.port}.log"), "w")
        self.car, self.dev = car.start("sm11", self.tr.getsockname()[1], self.ctl_port, seed,
                                       link=os.path.join(tempfile.gettempdir(), f"tars-vuno-test-{self.ctl_port}"),
                                       log=self.log)
        self.motor = car.start_motor(self.dev, self.port, watchdog_ms=watchdog_ms, log=self.log)
        self.connect()
        self.last = None

    def connect(self):
        self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=3)
        self.f = self.sock.makefile("rw", encoding="ascii", newline="\n")
        self.cmd("WHO test_car")

    def cmd(self, line: str) -> str:
        self.f.write(line + "\n")
        self.f.flush()
        return self.f.readline().strip()

    def status(self) -> dict:
        r = self.cmd("STATUS").split()
        return dict(kv.split("=", 1) for kv in r if "=" in kv)

    def truth(self) -> dict:
        """Latest truth datagram (drains the queue)."""
        d = None
        self.tr.setblocking(False)
        try:
            while True:
                d = json.loads(self.tr.recv(65536))
        except BlockingIOError:
            pass
        if d is None:
            self.tr.settimeout(1.0)
            d = json.loads(self.tr.recv(65536))
        self.last = d
        return d

    def ctl(self, msg: dict) -> dict:
        return car.ctl(msg, self.ctl_port)

    def wait(self, pred, timeout: float, poll: float = 0.02):
        """Poll pred() (also keeps the tars_motor link talking) until true or timeout.
        Returns seconds taken, or None."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if pred():
                return time.time() - t0
            time.sleep(poll)
        return None

    def stopped(self) -> bool:
        d = self.truth()
        return abs(d["wl"]) < 0.005 and abs(d["wr"]) < 0.005

    def exact(self, line: str, timeout: float = 20.0) -> tuple[dict, float, float]:
        """MOVE/TURN through tars_motor, keep polling STATUS (its client watchdog),
        return (final status, metres moved (signed along the start heading), heading
        change in degrees (+ = counter-clockwise, unwrapped))."""
        d0 = self.truth()
        x0, y0, th0 = d0["x"], d0["y"], d0["th"]
        rep = self.cmd(line)
        assert rep.startswith("OK"), rep
        acc, prev = 0.0, th0
        st = {}
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.02)
            self.cmd("PING")              # tars_motor's 500 ms deadman, as tars_client does
            d = self.truth()
            dth = math.atan2(math.sin(d["th"] - prev), math.cos(d["th"] - prev))
            acc += dth
            prev = d["th"]
            st = self.status()
            if st.get("move") not in ("running", "queued", "starting") and self.stopped():
                break
        time.sleep(0.15)                  # let it settle (braking)
        d = self.truth()
        acc += math.atan2(math.sin(d["th"] - prev), math.cos(d["th"] - prev))
        dist = (d["x"] - x0) * math.cos(th0) + (d["y"] - y0) * math.sin(th0)
        return st, dist, math.degrees(acc)

    def drive_for(self, l: float, r: float, secs: float) -> None:
        t0 = time.time()
        while time.time() - t0 < secs:
            self.cmd(f"D {l} {r}")
            time.sleep(0.02)

    def close(self):
        try:
            self.cmd("S")
            self.sock.close()
        except Exception:
            pass
        car.stop()
        self.log.close()


def run_wheel_cal(rig: Rig, args: list[str], measure) -> tuple[int, str]:
    """Run the REAL iter1-wasd/laptop/wheel_cal.py against the sim. When it asks
    'how far did it REALLY go', answer with measure() (the truth)."""
    env = dict(os.environ, TARS_HOST="127.0.0.1", TARS_PORT=str(rig.port), PYTHONUNBUFFERED="1")
    p = subprocess.Popen([sys.executable, "wheel_cal.py", *args], cwd=LAPTOP, env=env,
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    out = b""
    t0 = time.time()
    answered = False
    while time.time() - t0 < 40:
        r, _, _ = select.select([p.stdout], [], [], 0.05)
        if r:
            ch = os.read(p.stdout.fileno(), 4096)
            if not ch:
                break
            out += ch
        if not answered and b"REALLY go" in out:
            time.sleep(0.2)
            p.stdin.write(f"{measure():.1f}\n".encode())
            p.stdin.flush()
            answered = True
    try:
        p.wait(5)
    except subprocess.TimeoutExpired:
        p.kill()
    return p.returncode if p.returncode is not None else -9, out.decode(errors="replace")


def main() -> int:
    car.build()
    car.build_motor()
    print(f"load {load()}", flush=True)
    rig = Rig(seed=7)
    try:
        # ---- READY handshake (power-on boot, then the DTR reset when tars_motor opens it)
        t = rig.wait(lambda: rig.status().get("ready") == "1", 6.0)
        st = rig.status()
        check(t is not None and st.get("wheels") == "dc" and st.get("dc_mm_s") == "150.0",
              f"READY + WHEELS DC through the pty in {t and round(t, 2)} s (ready={st.get('ready')} wheels={st.get('wheels')})")
        check(rig.truth()["boots"] == 2, "firmware booted twice: power-on, then DTR reset on open")

        # ---- D drives, S stops
        rig.drive_for(0.6, 0.6, 1.0)
        d = rig.truth()
        check(d["v"] > 0.05 and d["pwm_l"] > 0 and d["pwm_r"] > 0,
              f"D 0.6 0.6 drives: v {d['v']:.3f} m/s, pwm {d['pwm_l']}/{d['pwm_r']}")
        t0 = time.time()
        rig.cmd("S")
        t = rig.wait(rig.stopped, 1.0, 0.005)
        check(t is not None and t < 0.25, f"S: wheels stopped (< 5 mm/s) in {t and round(t, 3)} s (limit 0.25 s, load {load()})")

        # ---- D once, then silence: tars_motor's watchdog (-w 300) stops it
        rig.cmd("D 1 1")
        t0 = time.time()
        rig.wait(lambda: rig.truth()["v"] > 0.03, 1.0, 0.01)
        t = None
        while time.time() - t0 < 2.0:              # silence on TCP; only read UDP truth
            if rig.stopped():
                t = time.time() - t0
                break
            time.sleep(0.005)
        check(t is not None and t < 0.7, f"one D then silence: tars_motor watchdog (300 ms) stopped it in {t and round(t, 3)} s")

        # ---- frozen Pi (SIGSTOP tars_motor mid-drive): the firmware's own 200 ms watchdog
        rig.drive_for(1, 1, 0.6)
        os.kill(rig.motor.pid, signal.SIGSTOP)
        t0 = time.time()
        t = None
        while time.time() - t0 < 2.0:
            if rig.stopped():
                t = time.time() - t0
                break
            time.sleep(0.005)
        os.kill(rig.motor.pid, signal.SIGCONT)
        check(t is not None and t < 0.45, f"tars_motor frozen: Uno firmware watchdog (200 ms) stopped it in {t and round(t, 3)} s")
        rig.cmd("S")
        time.sleep(0.3)

        # ---- LED / BEEP reach the kit pins (D3 LED, D12 buzzer)
        rig.cmd("LED ok")
        led = rig.wait(lambda: rig.truth()["led"] == 1, 1.5)
        rig.cmd("LED off")
        rig.cmd("BEEP 2")
        buzz = rig.wait(lambda: rig.truth()["buzz"] == 1, 1.0, 0.01)
        check(led is not None and buzz is not None, f"LED ok lights D3 ({led is not None}), BEEP 2 sounds D12 ({buzz is not None})")

        # ---- the true D 1 speeds of this profile (what calibration should find)
        rig.drive_for(1, 1, 2.0)
        v_true = sum(rig.truth()["v"] for _ in range(5) if not time.sleep(0.02)) / 5
        rig.cmd("S"); time.sleep(0.4)
        rig.drive_for(1, -1, 2.0)
        w_true = abs(sum(rig.truth()["w"] for _ in range(5) if not time.sleep(0.02)) / 5)
        rig.cmd("S"); time.sleep(0.4)
        print(f"  ..   true D 1 speeds: {v_true * 1000:.1f} mm/s straight, {math.degrees(w_true):.1f} deg/s spin", flush=True)

        # ---- calibration loop: the real wheel_cal.py, answering its prompt with the truth
        for i in range(2):
            mark = rig.truth()
            x0, y0 = mark["x"], mark["y"]
            rc, out = run_wheel_cal(rig, ["dc-move", "1000"],
                                    lambda: 1000 * math.hypot(rig.truth()["x"] - x0, rig.truth()["y"] - y0))
            line = [l for l in out.splitlines() if l.startswith("saved")]
            print(f"  ..   dc-move #{i + 1}: {line[0] if line else out[-300:]!r}", flush=True)
        st = rig.status()
        mm_s = float(st["dc_mm_s"])
        check(rc == 0 and abs(mm_s - v_true * 1000) / (v_true * 1000) < 0.06,
              f"wheel_cal.py dc-move x2 recovers mm/s: {mm_s:.1f} vs true {v_true * 1000:.1f} (tol 6%)")

        for i in range(2):
            acc = {"a": 0.0, "prev": rig.truth()["th"]}
            # heading change measured live (the tool's TURN runs while we poll)
            import threading
            stop = threading.Event()

            def track():
                while not stop.is_set():
                    th = rig.truth()["th"]
                    acc["a"] += math.atan2(math.sin(th - acc["prev"]), math.cos(th - acc["prev"]))
                    acc["prev"] = th
                    time.sleep(0.01)
            th_ = threading.Thread(target=track, daemon=True)
            th_.start()
            rc, out = run_wheel_cal(rig, ["dc-spin", "360"], lambda: abs(math.degrees(acc["a"])))
            stop.set(); th_.join()
            line = [l for l in out.splitlines() if l.startswith("saved")]
            print(f"  ..   dc-spin #{i + 1}: {line[0] if line else out[-300:]!r}", flush=True)
        st = rig.status()
        deg_s = float(st["dc_deg_s"])
        check(rc == 0 and abs(deg_s - math.degrees(w_true)) / math.degrees(w_true) < 0.10,
              f"wheel_cal.py dc-spin x2 recovers deg/s: {deg_s:.1f} vs true {math.degrees(w_true):.1f} (tol 10%)")

        # ---- MOVE 500 / TURN 90 on the gain-matched, calibrated profile
        st, dist, dth = rig.exact("MOVE 500")
        check(st.get("move") == "done" and abs(dist - 0.5) < 0.04 and abs(dth) < 3,
              f"MOVE 500 (matched, calibrated): {dist * 1000:.0f} mm, heading {dth:+.1f} deg ({st.get('move')})")
        st, dist, dth = rig.exact("TURN 90")
        check(st.get("move") == "done" and abs(-dth - 90) < 10,
              f"TURN 90 (right = clockwise): turned {-dth:.1f} deg ({st.get('move')}, tol 10)")
        # ---- mismatched sides drift
        rig.ctl({"set": {"gain_r": 0.8}})
        st, dist, dth = rig.exact("MOVE 500")
        check(dth < -5, f"gain_r 0.8: MOVE 500 curves right by {-dth:.1f} deg (> 5)")
        rig.ctl({"set": {"gain_r": 1.0}})

        # ---- tip: the firmware's own tilt detection on the simulated MPU6050
        rig.ctl({"fault": "tip"})
        t = rig.wait(lambda: rig.status().get("tipped") == "1", 2.0)
        rig.drive_for(0.8, 0.8, 0.4)
        d = rig.truth()
        check(t is not None and d["pwm_l"] == 0 and d["pwm_r"] == 0,
              f"tip fault: tars_motor tipped=1 in {t and round(t, 2)} s, D refused (pwm {d['pwm_l']}/{d['pwm_r']})")
        r = rig.cmd("RESET")
        check(r.startswith("ERR still tipped"), f"RESET refused while the Uno still reports T: {r!r}")
        rig.ctl({"fault": "clear"})
        # the firmware needs 500 ms upright, then tars_motor's TIP_CLEAR_MS since the last T
        t = rig.wait(lambda: rig.cmd("RESET").startswith("OK") and rig.status().get("tipped") == "0", 5.0, 0.2)
        check(t is not None, f"fault cleared: upright, RESET accepted, tipped=0 after {t and round(t, 2)} s")
        rig.cmd("S")

        # ---- wall: drive into it, the firmware feels the bump
        d = rig.truth()
        hx, hy = math.cos(d["th"]), math.sin(d["th"])
        cx, cy = d["x"] + 0.35 * hx, d["y"] + 0.35 * hy          # a wall 35 cm ahead
        rig.ctl({"walls": [[cx - hy, cy + hx, cx + hy, cy - hx]]})
        b0 = int(rig.status().get("bumps", "0"))
        t0 = time.time()
        hit = None
        while time.time() - t0 < 5:
            rig.cmd("D 1 1")
            if rig.truth()["contact"] and hit is None:
                hit = time.time() - t0
            if int(rig.status().get("bumps", "0")) > b0:
                break
            time.sleep(0.02)
        rig.cmd("S")
        b1 = int(rig.status().get("bumps", "0"))
        check(hit is not None and b1 > b0, f"wall hit: contact after {hit and round(hit, 2)} s, tars_motor bumps {b0}->{b1}")
        rig.ctl({"walls": []})
        time.sleep(1.2)

        # ---- brownout: the Uno resets, tars_motor sees READY again
        r0 = int(rig.status()["ready"])
        rig.ctl({"fault": "brownout", "v": 4.0, "ms": 300})
        t = rig.wait(lambda: int(rig.status()["ready"]) > r0, 4.0)
        check(t is not None, f"brownout 4.0 V: Uno reset, READY again in {t and round(t, 2)} s (ready {r0}->{rig.status()['ready']})")

        # ---- unplug: the pty disappears, comes back under a new name, tars_motor reconnects
        r0 = int(rig.status()["ready"])
        old = rig.truth()["pty"]
        rig.ctl({"fault": "unplug", "ms": 800})
        t = rig.wait(lambda: int(rig.status()["ready"]) > r0, 8.0, 0.05)
        new = rig.truth()["pty"]
        check(t is not None, f"unplug 0.8 s: tars_motor reconnected and got READY in {t and round(t, 2)} s ({old} -> {new})")
        rig.drive_for(0.6, 0.6, 0.5)
        check(rig.truth()["v"] > 0.03, "drives again after the reconnect")
        rig.cmd("S")

        # ---- control port sanity
        r = rig.ctl({"get": "profile"})
        check(r.get("ok") and abs(r["profile"]["kv_mps_per_v"] - 0.147) < 1e-9, "control: get profile")
        r = rig.ctl({"set": {"nope": 1}})
        check(not r.get("ok") and "unknown key" in r.get("error", ""), "control: unknown key rejected")
    finally:
        rig.close()
    print(f"load {load()}")
    try:
        print(f"virtual Uno main-loop stalls > 50 ms: {rig.last['stalls']} (max {rig.last['stall_max_ms']} ms)")
    except Exception:
        pass
    print(f"{total - fails}/{total} checks passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
