"""Python vs C++ parity: the QNX C++ app and the Python apps must behave the same.

Builds tests/parity_main.cpp with the host C++ compiler, generates random
sessions of synthetic hands (all poses, moving around, dropouts, camera stalls,
mode toggles, several config variants), runs both implementations, and diffs
every label, mode and command line. A second set of sessions exercises
teach-on-the-spot (common/learn.py vs learn.hpp): long held poses so the TEACH
sign triggers, novel poses get recorded, gestures assign actions, palm cancels,
Uno button presses, forget, and a save -> load round trip across languages.
A third set runs PET mode (common/pet.py vs pet.hpp): waves, come-here
approaches, thumbs, peace flashes and holds, fists, long empty stretches
(boredom, sleep), camera stalls and server-side stops.

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

import json  # noqa: E402

import gesture as g  # noqa: E402
import handgen as h  # noqa: E402
import learn as L  # noqa: E402

CONF = os.path.abspath(os.path.join(HERE, "..", "gesture.conf"))
BUILD = os.path.join(HERE, "build")


def build() -> str:
    cxx = os.environ.get("CXX") or shutil.which("c++") or shutil.which("clang++") or shutil.which("g++")
    if not cxx:
        raise SystemExit("no C++ compiler found")
    os.makedirs(BUILD, exist_ok=True)
    exe = os.path.join(BUILD, "parity_main")
    subprocess.run([cxx, "-std=c++17", "-O1", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror",
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


TEACH_POSES = ([("shaka", "up")] * 4 + [(p, "up") for p in h.NOVEL] * 2
               + [("palm", "up"), ("fist", "up"), ("peace", "up"), ("thumb", "up"),
                  ("point", "left"), ("point", "right"), ("none", "up")])


def teach_session(rng: random.Random, n_chunks: int, save_dir: str, tag: str):
    """Chunks of one pose held 0.2-4 s, so TEACH holds, recordings and picks happen."""
    t = 0.0
    aspect = rng.choice([640 / 480, 16 / 9])
    for ci in range(n_chunks):
        pose = rng.choice(TEACH_POSES)
        r = rng.random()
        if r < 0.03:
            yield ("B", round(t, 3), "teach")
        elif r < 0.045:
            yield ("B", round(t, 3), "forget")
        if ci == n_chunks // 2:
            yield ("SAVE", os.path.join(save_dir, f"{tag}_py.json"), os.path.join(save_dir, f"{tag}_cpp.json"))
            yield ("LOAD", os.path.join(save_dir, f"{tag}_py.json"))
        hold = rng.uniform(200, 4000)
        end = t + hold
        cx, size = rng.uniform(0.3, 0.7), rng.uniform(0.12, 0.3)
        while t < end:
            dt = rng.uniform(30, 90)
            k = t + rng.uniform(5, 20)
            while k < t + dt:
                yield ("T", round(k, 3))
                k += rng.uniform(15, 25)
            t += dt
            if pose[0] == "none" or rng.random() < 0.02:
                yield ("F", round(t, 3), aspect, None)
            else:
                lm = h.make_hand(pose[0], pose[1], cx=cx, size=size, jitter_deg=rng.uniform(-20, 20),
                                 noise=0.004, rng=rng, aspect=aspect)
                yield ("F", round(t, 3), aspect, lm)
    yield ("DB",)


def run_python(conf_path: str, events) -> list[str]:
    b = L.Brain(g.load_config(conf_path))
    out = []

    def dump(cmds):
        out.extend(f"> {x.text}|{x.reason}" for x in cmds)
        out.extend(f"E {e}" for e in b.events)
        b.events = []
        out.extend(f"P {e}" for e in b.pet.events)
        b.pet.events = []

    for ev in events:
        if ev[0] == "T":
            dump(b.tick(ev[1]))
        elif ev[0] == "B":
            dump(b.button(ev[1], ev[2]))
        elif ev[0] == "K":
            b.server_stop(ev[1])
            dump([])
        elif ev[0] == "SAVE":
            b.save(ev[1])
        elif ev[0] == "LOAD":
            out.append(f"LOADED {b.load(ev[1])}")
        elif ev[0] == "DB":
            out.extend("DB %s %s %.9f %d" % (c.name, c.action, c.thr, len(c.examples))
                       for c in b.learner.classes)
        else:
            _, t, aspect, lm = ev
            cmds = b.on_frame(t, lm, aspect)
            out.append(f"L {b.label} {b.ctl.mode} {b.state} {b.pet.state} {b.pet.energy:.6f}")
            dump(cmds)
    return out


def script(conf_path: str, events) -> str:
    lines = [f"C {conf_path}"]
    for ev in events:
        if ev[0] == "T":
            lines.append(f"T {ev[1]!r}")
        elif ev[0] == "B":
            lines.append(f"B {ev[1]!r} {ev[2]}")
        elif ev[0] == "K":
            lines.append(f"K {ev[1]!r}")
        elif ev[0] == "SAVE":
            lines.append(f"SAVE {ev[2]}")
        elif ev[0] == "LOAD":
            lines.append(f"LOAD {ev[1]}")
        elif ev[0] == "DB":
            lines.append("DB")
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
                        _, lab, mode = ln.split()[:3]
                        labels_seen.add(lab)
                        modes_seen.add(mode)
                    elif ln.startswith("> "):
                        total_cmds += 1
                        reasons_seen.add(ln.rsplit("|", 1)[1])
    print(f"parity OK: {len(VARIANTS)} configs x 6 sessions, {total_lines} lines, {total_cmds} commands identical")
    print(f"  labels covered : {sorted(labels_seen)}")
    print(f"  modes covered  : {sorted(modes_seen)}")
    print(f"  reasons covered: {sorted(reasons_seen)}")
    missing = set(g.LABELS) - labels_seen
    must = {"palm", "lost", "camera_timeout", "done", "forward", "back", "turn_left",
            "turn_right", "resend", "follow", "mode_gesture", "mode_follow"}
    if missing or must - reasons_seen or not {"gesture", "follow", "pet"} <= modes_seen:
        print(f"coverage gap: labels {missing}, reasons {must - reasons_seen}")
        return 1
    return teach_parity(exe)


def teach_parity(exe: str) -> int:
    """Teach sessions through both Brains; also check the C++ and Python JSON files agree."""
    total = 0
    events_seen: set[str] = set()
    reasons_seen: set[str] = set()
    with tempfile.TemporaryDirectory(dir=BUILD) as tmp:
        for vi, over in enumerate([{}, {"feedback_led_beep": 0, "learn_assign_frames": 3}]):
            conf = write_conf(tmp, over)
            for seed in range(8):
                rng = random.Random(5000 + 100 * vi + seed)
                tag = f"v{vi}s{seed}"
                events = list(teach_session(rng, 90, tmp, tag))
                py = run_python(conf, events)
                # the C++ side saves to *_cpp.json, then (like Python) loads the Python file
                cpp = subprocess.run([exe], input=script(conf, events), capture_output=True,
                                     text=True, check=True).stdout.splitlines()
                if py != cpp:
                    for i, (a, b) in enumerate(zip(py, cpp)):
                        if a != b:
                            print(f"TEACH MISMATCH variant {vi} seed {seed} line {i}:\n  py : {a}\n  cpp: {b}")
                            print("  context:", py[max(0, i - 6):i])
                            return 1
                    print(f"TEACH MISMATCH variant {vi} seed {seed}: length {len(py)} vs {len(cpp)}")
                    return 1
                with open(os.path.join(tmp, f"{tag}_py.json")) as f:
                    jp = json.load(f)
                with open(os.path.join(tmp, f"{tag}_cpp.json")) as f:
                    jc = json.load(f)
                if jp != jc:
                    print(f"JSON MISMATCH variant {vi} seed {seed}")
                    return 1
                total += len(py)
                for ln in py:
                    if ln.startswith("E "):
                        events_seen.add(" ".join(ln.split()[1:3]))
                    elif ln.startswith("> "):
                        reasons_seen.add(ln.rsplit("|", 1)[1].split("@")[0])
    print(f"teach parity OK: 2 configs x 8 sessions, {total} lines identical, saved JSON identical")
    print(f"  teach events : {sorted(events_seen)}")
    print(f"  reasons      : {sorted(reasons_seen)}")
    need = {"teach start", "record", "learned pose1", "reject too_similar", "cancel palm", "forget 0",
            "recorded 40"}
    have = {e for e in events_seen} | {" ".join(e.split()[:2]) for e in events_seen}
    missing = {n for n in need if not any(e.startswith(n) for e in have)}
    if missing:
        print(f"teach coverage gap: {missing}")
        return 1
    return pet_parity(exe)


PET_CHUNKS = (["wave"] * 3 + ["still_palm"] * 2 + ["come"] * 3 + ["thumbs_up"] * 3
              + ["thumbs_down"] * 2 + ["peace_flash"] * 3 + ["peace_hold"] + ["fist_hold"] * 2
              + ["watch"] * 4 + ["none_short"] * 3 + ["none_long"] * 2 + ["novel"] * 2 + ["sleep_wave"] * 2)


def pet_session(rng: random.Random, n_chunks: int):
    """Chunks of one pet behaviour each, starting in PET mode (pet_start = 1)."""
    t = 0.0
    aspect = rng.choice([640 / 480, 16 / 9])

    def frames(ms, pose, d="up", cx=lambda k: 0.5, size=lambda k: 0.15, drop=0.02):
        nonlocal t
        end, k = t + ms, 0
        while t < end:
            dt = rng.uniform(40, 90)
            q = t + rng.uniform(5, 20)
            while q < t + dt:
                yield ("T", round(q, 3))
                q += rng.uniform(15, 25)
            t += dt
            if rng.random() < 0.004:                                    # server refuses a D
                yield ("K", round(t - 1.0, 3))
            if pose == "none" or rng.random() < drop:
                yield ("F", round(t, 3), aspect, None)
            else:
                lm = h.make_hand(pose, d, cx=min(0.9, max(0.1, cx(k))), size=size(k),
                                 jitter_deg=rng.uniform(-15, 15), noise=0.003, rng=rng, aspect=aspect)
                yield ("F", round(t, 3), aspect, lm)
            k += 1

    for _ in range(n_chunks):
        kind = rng.choice(PET_CHUNKS)
        c0 = rng.uniform(0.25, 0.75)
        if rng.random() < 0.03:                                         # camera stall
            t += rng.uniform(450, 1500)
        if kind == "wave":
            amp, per = rng.uniform(0.06, 0.15), rng.randint(4, 8)
            yield from frames(rng.uniform(900, 2500), "palm", cx=lambda k: c0 + amp * (1 if (k // per) % 2 else -1))
        elif kind == "still_palm":
            yield from frames(rng.uniform(300, 2500), "palm", cx=lambda k: c0)
        elif kind == "come":
            s0 = rng.uniform(0.05, 0.12)
            grow = rng.uniform(0.0, 0.012)
            yield from frames(rng.uniform(800, 6000), "point", "up", cx=lambda k: c0, size=lambda k: s0 + grow * k)
        elif kind == "thumbs_up":
            yield from frames(rng.uniform(300, 1500), "thumb", "up", cx=lambda k: c0)
        elif kind == "thumbs_down":
            yield from frames(rng.uniform(300, 1500), "thumb", "down", cx=lambda k: c0)
        elif kind == "peace_flash":
            yield from frames(rng.uniform(150, 900), "peace", cx=lambda k: c0)
        elif kind == "peace_hold":
            yield from frames(rng.uniform(1500, 3500), "peace", cx=lambda k: c0)
        elif kind == "sleep_wave":                                      # play dead, then wave hello
            yield from frames(rng.uniform(2000, 3000), "fist", cx=lambda k: c0)
            yield from frames(rng.uniform(200, 1500), "none")
            amp, per = rng.uniform(0.08, 0.15), rng.randint(4, 7)
            yield from frames(rng.uniform(1200, 2200), "palm", cx=lambda k: c0 + amp * (1 if (k // per) % 2 else -1))
        elif kind == "fist_hold":
            yield from frames(rng.uniform(600, 2500), "fist", cx=lambda k: c0)
        elif kind == "watch":
            v = rng.uniform(-0.02, 0.02)
            yield from frames(rng.uniform(500, 4000), rng.choice(["fist", "point", "rock"]),
                              rng.choice(["up", "left", "right"]), cx=lambda k: c0 + v * k)
        elif kind == "none_short":
            yield from frames(rng.uniform(200, 3000), "none")
        elif kind == "none_long":
            yield from frames(rng.uniform(4000, 25000), "none")
        else:
            yield from frames(rng.uniform(300, 2000), rng.choice(h.NOVEL), cx=lambda k: c0)


def pet_parity(exe: str) -> int:
    total = 0
    pet_events, reasons_seen, states_seen = set(), set(), set()
    variants = [{"pet_start": 1},
                {"pet_start": 1, "feedback_led_beep": 0, "stop_frames": 2, "pet_energy_start": 0.9},
                {"pet_start": 1, "pet_seed": 99, "pet_wave_swings": 3, "pet_gap_frames": 0,
                 "pet_energy_decay_per_s": 0.2}]
    with tempfile.TemporaryDirectory(dir=BUILD) as tmp:
        for vi, over in enumerate(variants):
            conf = write_conf(tmp, over)
            for seed in range(8):
                rng = random.Random(9000 + 100 * vi + seed)
                events = list(pet_session(rng, 120))
                py = run_python(conf, events)
                cpp = subprocess.run([exe], input=script(conf, events), capture_output=True,
                                     text=True, check=True).stdout.splitlines()
                if py != cpp:
                    for i, (a, b) in enumerate(zip(py, cpp)):
                        if a != b:
                            print(f"PET MISMATCH variant {vi} seed {seed} line {i}:\n  py : {a}\n  cpp: {b}")
                            print("  context:", py[max(0, i - 8):i])
                            return 1
                    print(f"PET MISMATCH variant {vi} seed {seed}: length {len(py)} vs {len(cpp)}")
                    return 1
                total += len(py)
                for ln in py:
                    if ln.startswith("P "):
                        pet_events.add(ln[2:])
                    elif ln.startswith("> "):
                        reasons_seen.add(ln.rsplit("|", 1)[1])
                    elif ln.startswith("L "):
                        states_seen.add(ln.split()[4])
    print(f"pet parity OK: {len(variants)} configs x 8 sessions, {total} lines identical")
    print(f"  pet states : {sorted(states_seen)}")
    print(f"  pet events : {sorted(pet_events)}")
    print(f"  reasons    : {sorted(r for r in reasons_seen if r.startswith('pet') or r in ('palm', 'lost', 'camera_timeout', 'mode_gesture'))}")
    need_states = {"idle", "watch", "come", "stay", "greet", "happy", "sad", "trick", "look", "sleep"}
    need_events = {"enter", "wave", "come", "arrived", "come limit", "sleep fist", "sleep bored",
                   "wake palm", "leave peace", "trick", "happy", "sad", "greet", "look"}
    need_reasons = {"pet_stream", "pet_greet", "pet_happy", "pet_sad", "pet_trick", "pet_look",
                    "pet_arrived", "pet_settle", "pet_sleep", "palm", "lost", "camera_timeout"}
    gap = (need_states - states_seen, need_events - pet_events, need_reasons - reasons_seen)
    if any(gap):
        print(f"pet coverage gap: states {gap[0]}, events {gap[1]}, reasons {gap[2]}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
