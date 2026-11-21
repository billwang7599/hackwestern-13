"""Check that balance/micropython/balance.py matches the C library.

    python check_parity.py

Feeds both the same random IMU/setpoint stream (through arming, running, falling
and re-arming) and compares every output. C uses float32, Python float64, so a
small tolerance is allowed. Then runs the same sim scenario on both backends.
"""
import math
import random
import sys

from controller import Controller
from params import robot_for_motor
from simulate import run_scenario


def stream_test(n=20000, seed=3):
    rng = random.Random(seed)
    c, py = Controller("c"), Controller("py")
    worst = 0.0
    pitch = 0.0
    for k in range(n):
        # random walk in pitch, occasionally past the cutoff, then back upright
        if k % 4000 < 300:
            pitch = 0.0
        pitch += rng.gauss(0, 0.004)
        if k % 4000 == 2500:
            pitch = 0.9        # fall
        g = rng.gauss(0, 0.2)
        af, au = -9.81 * math.sin(pitch) + rng.gauss(0, 0.3), 9.81 * math.cos(pitch)
        ws = None if k % 2 else rng.gauss(0, 0.2)
        enable = 0 if k < 200 else 1
        sp = (rng.uniform(-0.3, 0.3), rng.uniform(-1, 1)) if k % 500 < 250 else (0.0, 0.0)
        dt = 0.0025 + rng.gauss(0, 0.0002)
        a = c.step(g, af, au, pitch, ws, sp[0], sp[1], enable, dt)
        b = py.step(g, af, au, pitch, ws, sp[0], sp[1], enable, dt)
        if a[5] != b[5] or a[6] != b[6]:
            print(f"state mismatch at step {k}: C={a[5:]} py={b[5:]}")
            return False
        for x, y in zip(a[:5], b[:5]):
            worst = max(worst, abs(x - y) / (1.0 + abs(y)))
    print(f"stream: {n} steps, states identical, worst relative diff {worst:.2e}")
    return worst < 1e-3


def sim_test():
    ok = True
    for motor in ("nema17", "tt"):
        p = robot_for_motor(motor)
        mc, _ = run_scenario(p, backend="c")
        mp, _ = run_scenario(p, backend="py")
        # after a fall the motion is chaotic, so only compare tilt while upright
        d = 0.0 if mc["fell"] else abs(mc["max_tilt_deg"] - mp["max_tilt_deg"])
        same = mc["passed"] == mp["passed"] and mc["fell"] == mp["fell"] and d < 0.5
        ok &= same
        print(f"sim {motor}: C max tilt {mc['max_tilt_deg']:.2f} deg ({'PASS' if mc['passed'] else 'FAIL'}), "
              f"py {mp['max_tilt_deg']:.2f} deg ({'PASS' if mp['passed'] else 'FAIL'}) -> "
              f"{'match' if same else 'MISMATCH'}")
    return ok


if __name__ == "__main__":
    ok = stream_test() and sim_test()
    print("parity OK" if ok else "parity FAILED")
    sys.exit(0 if ok else 1)
