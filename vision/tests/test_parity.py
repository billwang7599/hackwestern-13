"""Python vs C++ parity: the QNX C++ app and the Python apps must behave the same.

Builds tests/parity_main.cpp with the host C++ compiler, generates random
sessions of synthetic hands (all poses, moving around, dropouts, camera stalls,
mode toggles, several config variants), runs both implementations, and diffs
every label, mode and command line.

    python3 vision/tests/test_parity.py
"""
from __future__ import annotations

import os
import random
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "common"), HERE]

import gesture as g  # noqa: E402
import handgen as h  # noqa: E402

CONF = os.path.abspath(os.path.join(HERE, "..", "gesture.conf"))
BUILD = os.path.join(HERE, "build")


def build() -> str:
    cxx = os.environ.get("CXX") or shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if not cxx:
        raise SystemExit("no C++ compiler found")
    os.makedirs(BUILD, exist_ok=True)
    exe = os.path.join(BUILD, "parity_main")
    subprocess.run([cxx, "-std=c++17", "-O1", "-Wall", "-Wextra", "-Werror",
                    os.path.join(HERE, "parity_main.cpp"), "-o", exe], check=True)
    return exe


POSES = [(p, d) for (p, d) in h.EXPECTED] + [("none", "up")]


def session(rng: random.Random, n_frames: int):
    """Yield script events: ('F', t, aspect, landmarks|None) / ('T', t)."""
    t = 0.0
    aspect = rng.choice([640 / 480, 16 / 9, 1.0])
    pose = rng.choice(POSES)
    cx, cy, size = 0.5, 0.5, 0.15
    for _ in range(n_frames):
        r = rng.random()
        if r < 0.08:
            pose = rng.choice(POSES)
        elif r < 0.10:                                   # long peace hold to toggle modes
            pose = ("peace", "up")
        cx = min(0.9, max(0.1, cx + rng.gauss(0, 0.03)))
        cy = min(0.8, max(0.2, cy + rng.gauss(0, 0.02)))
        size = min(0.45, max(0.04, size + rng.gauss(0, 0.01)))
        if rng.random() < 0.01:                           # camera stall
            t += rng.uniform(300, 900)
        dt = rng.uniform(25, 110)                         # 9-40 fps, jittery
        # ticks between frames (the runner's 20 ms timer)
        k = t + rng.uniform(5, 20)
        while k < t + dt:
            yield ("T", round(k, 3))
            k += rng.uniform(15, 25)
        t += dt
        if pose[0] == "none" or rng.random() < 0.03:
            yield ("F", round(t, 3), aspect, None)
        else:
            lm = h.make_hand(pose[0], pose[1], cx=cx, cy=cy, size=size,
                             jitter_deg=rng.uniform(-50, 50), noise=rng.choice([0, 0.003, 0.01]),
                             rng=rng, aspect=aspect)
            yield ("F", round(t, 3), aspect, lm)


VARIANTS = [
    {},
    {"hold_repeat": 0, "point_left_is_left": 0, "flip": 0},
    {"stop_frames": 3, "follow_invert_turn": 1, "follow_max_back": 0, "mode_frames": 3},
    {"stable_frames": 2, "cooldown_ms": 0, "follow_slew_per_s": 50, "follow_send_ms": 40},
]


def run_python(conf_path: str, events) -> list[str]:
    c = g.Controller(g.load_config(conf_path))
    out = []
    for ev in events:
        if ev[0] == "T":
            out += [f"> {x.text}|{x.reason}" for x in c.tick(ev[1])]
        else:
            _, t, aspect, lm = ev
            label = g.classify(lm, aspect, c.cfg)
            cmds = c.on_frame(t, label, g.hand_features(lm, aspect))
            out.append(f"L {label} {c.mode}")
            out += [f"> {x.text}|{x.reason}" for x in cmds]
    return out


def script(conf_path: str, events) -> str:
    lines = [f"C {conf_path}"]
    for ev in events:
        if ev[0] == "T":
            lines.append(f"T {ev[1]!r}")
        else:
            _, t, aspect, lm = ev
            if lm is None:
                lines.append(f"F {t!r} {aspect!r} 0")
            else:
                pts = " ".join(f"{x!r} {y!r} {z!r}" for x, y, z in lm)
                lines.append(f"F {t!r} {aspect!r} {len(lm)} {pts}")
    return "\n".join(lines) + "\n"


def write_conf(tmp: str, over: dict) -> str:
    cfg = g.load_config(CONF, over)
    p = os.path.join(tmp, f"v{abs(hash(tuple(sorted(over.items()))))}.conf")
    with open(p, "w") as f:
        for k, v in cfg.items():
            f.write(f"{k} = {v!r}\n")
    return p


def main() -> int:
    exe = build()
    total_lines = total_cmds = 0
    labels_seen, reasons_seen, modes_seen = set(), set(), set()
    with tempfile.TemporaryDirectory(dir=BUILD) as tmp:
        for vi, over in enumerate(VARIANTS):
            conf = write_conf(tmp, over)
            for seed in range(6):
                rng = random.Random(1000 * vi + seed)
                events = list(session(rng, 1500))
                py = run_python(conf, events)
                cpp = subprocess.run([exe], input=script(conf, events), capture_output=True,
                                     text=True, check=True).stdout.splitlines()
                if py != cpp:
                    for i, (a, b) in enumerate(zip(py, cpp)):
                        if a != b:
                            print(f"MISMATCH variant {vi} seed {seed} line {i}:\n  py : {a}\n  cpp: {b}")
                            print("  context:", py[max(0, i - 4):i])
                            return 1
                    print(f"MISMATCH variant {vi} seed {seed}: length {len(py)} vs {len(cpp)}")
                    return 1
                total_lines += len(py)
                for ln in py:
                    if ln.startswith("L "):
                        _, lab, mode = ln.split()
                        labels_seen.add(lab)
                        modes_seen.add(mode)
                    else:
                        total_cmds += 1
                        reasons_seen.add(ln.rsplit("|", 1)[1])
    print(f"parity OK: {len(VARIANTS)} configs x 6 sessions, {total_lines} lines, {total_cmds} commands identical")
    print(f"  labels covered : {sorted(labels_seen)}")
    print(f"  modes covered  : {sorted(modes_seen)}")
    print(f"  reasons covered: {sorted(reasons_seen)}")
    missing = set(g.LABELS) - labels_seen
    must = {"palm", "lost", "camera_timeout", "done", "forward", "back", "turn_left",
            "turn_right", "resend", "follow", "mode_gesture", "mode_follow"}
    if missing or must - reasons_seen or modes_seen != {"gesture", "follow"}:
        print(f"coverage gap: labels {missing}, reasons {must - reasons_seen}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
