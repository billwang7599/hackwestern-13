"""Classification time per frame (features + k-NN over learned poses), Python and C++.

For the judges' slide. MediaPipe inference is separate (the app logs infer=).

    python3 iter3-pet/vision/tests/bench_learn.py
"""
from __future__ import annotations

import os
import random
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "common"), HERE]

import gesture as g  # noqa: E402
import handgen as h  # noqa: E402
import learn as L  # noqa: E402
import test_parity as tp  # noqa: E402


def main() -> int:
    cfg = g.load_config()
    aw = cfg["learn_angle_weight"]
    exe = tp.build()
    rows = []
    for n_poses in (1, 4, 8):
        rng = random.Random(n_poses)
        b = L.Brain(cfg)
        poses = (h.NOVEL * 2)[:n_poses]
        for i, p in enumerate(poses):
            raw = [L.features(h.make_hand(p, "up", cx=rng.uniform(.3, .7), size=rng.uniform(.12, .3),
                                          jitter_deg=rng.uniform(-30, 30), noise=0.004, rng=rng), h.ASPECT, aw)
                   for _ in range(int(cfg["learn_frames"]))]
            b.learner.add(f"pose{i + 1}", "spin", raw)
        lm = h.make_hand("rock", "up", noise=0.004, rng=rng)
        reps = 300
        t0 = time.perf_counter()
        for _ in range(reps):
            b.learner.match(L.features(lm, h.ASPECT, aw))
        py_us = (time.perf_counter() - t0) / reps * 1e6
        with tempfile.TemporaryDirectory(dir=tp.BUILD) as d:
            jp = os.path.join(d, "db.json")
            b.save(jp)
            pts = " ".join(f"{x!r} {y!r} {z!r}" for x, y, z in lm)
            scr = f"C {tp.CONF}\nLOAD {jp}\nBENCH 5000 {h.ASPECT!r} {pts}\n"
            out = subprocess.run([exe], input=scr, capture_output=True, text=True, check=True).stdout
            cpp_us = float([ln for ln in out.splitlines() if ln.startswith("BENCH")][0].split()[1])
        n_ex = len(b.learner.classes[0].examples)
        rows.append((n_poses, n_ex, py_us, cpp_us))
    print("learned poses | examples/pose | Python us/frame | C++ (-O1) us/frame")
    for r in rows:
        print(f"{r[0]:13d} | {r[1]:13d} | {r[2]:15.0f} | {r[3]:18.1f}")
    print("(this Mac; the Pi 5 is roughly 2-4x slower. MediaPipe inference itself is milliseconds.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
