#!/usr/bin/env python3
"""Whisper repeat-loop filter (stt_dedupe in src/stt.c), via `build/stt_cli --dedupe TEXT`.

    make -C iter2-autonomy/voice test-stt        (also part of `make test`; no model needed)

tiny.en sometimes loops ("Tars, turn left, turn left, turn left, ..."), which turned clean
commands into rejections. Checks: the loops the bench voice lab saw collapse to one copy, normal
commands pass through unchanged, and (fuzz) the filter never adds a word and never changes the
safety-stop verdict: a stop in, a stop out; no stop in, no stop out.
"""
from __future__ import annotations

import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
VOICE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(VOICE, "intent"))
import intent as I  # noqa: E402

CLI = os.path.join(VOICE, "build", "stt_cli")
checks = failures = 0


def check(cond: bool, what: str) -> None:
    global checks, failures
    checks += 1
    if not cond:
        failures += 1
        print(f"  FAIL {what}")


def dedupe(t: str) -> str:
    return subprocess.run([CLI, "--dedupe", t], capture_output=True, text=True, check=True).stdout.rstrip("\n")


def main() -> int:
    if not os.path.exists(CLI):
        sys.exit(f"missing {CLI}: make -C iter2-autonomy/voice host")
    loops = {   # real stt_cli outputs from the bench say clips (before the fix)
        "Tars, turn left, turn left, turn left, turn left, turn left, turn left, turn left,": "Tars, turn left,",
        "Tars, turn lift, turn lift, turn, turn lift, turn, turn, turn, turn, turn, turn": "Tars, turn lift",
        "Tars, go to the laptop, go to the laptop, go to the laptop, go to the laptop, go to the": "Tars, go to the laptop",
        "freeze, freeze, freeze, freeze, freeze, freeze, freeze, freeze": "freeze",
        "stop, stop, stop, stop.": "stop.",
        "TARS, turn. Go back half a meter. TARS, turn. TARS, turn.": "TARS, turn. Go back half a meter.",
        "Tars, turn right ninety, turn right ninety, turn": "Tars, turn right ninety",
        "Tars go to the chair the chair": "Tars go to the chair",
        # the old filter cut everything after the first repeated sentence, stop included
        "Turn left. Turn left. Stop!": "Turn left. Stop!",
        "Tars, go forward. Tars, go forward. Wait!": "Tars, go forward. Wait!",
    }
    for t, want in loops.items():
        got = dedupe(t)
        check(got == want, f"loop {t!r}: want {want!r}, got {got!r}")
    m = I.load_model()
    for t in loops:                     # and the collapsed text decides like one clean copy would
        d = I.decide(m, dedupe(t), require_wake=False)
        check(d.action != "reject" or "lift" in t, f"collapsed loop still understood {t!r}: {d}")
    same = ["Tars, go forward two steps.", "Tars, turn right one eighty.", "TARS, go to the trash can.",
            "Tars, go back to where the bin was.", "Stop!", "TARS, turn left 90 degrees.", "", "Tars.",
            "Tars, forward half a metre.", "TARS, what do you remember?"]
    for t in same:
        check(dedupe(t) == t, f"unchanged {t!r} -> {dedupe(t)!r}")
    # stop words survive any collapse
    for t in ("stop stop stop", "Stop. Stop. Stop.", "turn left, turn left, stop", "whoa, whoa, whoa",
              "Tars, no, no, no, no", "halt halt", "go go go stop stop"):
        o = dedupe(t)
        for wake in (False, True):
            toks = I.normalize(o)
            check(I.safety_stop(toks, wake) == I.safety_stop(I.normalize(t), wake), f"stop kept {t!r} -> {o!r}")
    # fuzz: never adds a word, never changes the stop verdict, never longer
    rng = random.Random(3)
    vocab = ("tars turn left right go to the forward back stop no wait chair bin laptop one two "
             "ninety meter steps freeze whoa halt step top a").split()
    seps = [" ", ", ", ". ", "! ", "? "]
    bad = 0
    for _ in range(600):
        unit = [rng.choice(vocab) for _ in range(rng.randint(1, 4))]
        words = [rng.choice(vocab) for _ in range(rng.randint(0, 3))] + unit * rng.randint(1, 5) + unit[:rng.randint(0, len(unit))]
        t = "".join(w + rng.choice(seps) for w in words).strip()
        o = dedupe(t)
        ti, oi = I.normalize(t), I.normalize(o)
        ok = (len(o) <= len(t) and set(oi) <= set(ti)
              and all(I.safety_stop(oi, w) == I.safety_stop(ti, w) for w in (False, True)))
        if not ok:
            bad += 1
            if bad <= 5:
                print(f"  FUZZ {t!r} -> {o!r}")
    check(bad == 0, f"fuzz: {600 - bad}/600 keep the words and the stop verdict")
    print(f"{checks - failures}/{checks} checks passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
