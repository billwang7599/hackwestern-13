#!/usr/bin/env python3
"""Intent classifier tests (Mac/Linux host; standard library only).

    make -C iter2-autonomy/voice test-intent        # builds build/intent_cli, then runs this

  1. normalisation, wake word, safety words, number/slot parser (unit tests)
     1b. regressions found by the bench voice lab (bare "turn", "the left one", ...)
  2. held-out accuracy on TEST templates (phrasings never seen in training),
     per-class precision/recall, confusion matrix, accept/reject rates
  3. hand-written realistic whisper transcripts (data/realistic.tsv)
  4. STOP recall: every stop sentence must end in "S" (100% required)
  5. Python <-> C parity on every sentence above + fuzz strings
  6. latency (C and Python)

--markdown prints the tables for iter2-autonomy/voice/README.md.
"""
from __future__ import annotations

import os
import random
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
VOICE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(VOICE, "intent"))
import gen_data  # noqa: E402
import intent as I  # noqa: E402

CLI = os.path.join(VOICE, "build", "intent_cli")
MD = "--markdown" in sys.argv
checks = failures = 0


def check(cond: bool, what: str) -> None:
    global checks, failures
    checks += 1
    if not cond:
        failures += 1
        print(f"  FAIL {what}")


def first_line(d: I.Decision) -> str:
    """First real command: skips the "@goto CANCEL" that precedes manual moves,
    and drops the "@goto " routing prefix ("GOTO chair")."""
    for l in d.lines:
        if l != I.CANCEL:
            return l[len(I.GOTO_PREFIX):] if l.startswith(I.GOTO_PREFIX) else l
    return "-"


# ------------------------------------------------------------------ 1. units
def test_units(m: I.Model) -> None:
    print("units: normalize / wake / safety / slots")
    check(I.normalize("TARS, turn left 90°.") == ["tars", "turn", "left", "90"], "normalize punct + degree sign")
    check(I.normalize("Tar’s go 1.5m!") == ["tars", "go", "1.5", "m"], "curly apostrophe, decimal, 1.5m split")
    check(I.normalize("don't  STOP...") == ["dont", "stop"], "apostrophe dropped, dots")
    check(I.normalize("v1.2.3") == ["v", "1.2.3"], "dotted digits kept as one token")
    check(I.normalize("café ok") == ["caf", "ok"], "non-ASCII -> space")
    for w in ("tars", "stars", "cars", "tires", "taurus", "tarz", "bars"):
        check(I.is_wake(w), f"wake {w}")
    for w in ("turn", "go", "the", "car", "stop"):
        check(not I.is_wake(w), f"not wake {w}")
    check(I.strip_wake(["hey", "tars", "go"]) == (True, ["go"]), "hey tars")
    check(I.strip_wake(["go", "tars"]) == (False, ["go", "tars"]), "wake must lead")
    for w in ("stop", "stopped", "stops", "stob", "stap", "halt", "hault", "freeze", "frees", "breeze", "whoa", "woah", "stoop"):
        check(I.strong_stop_word(w), f"strong stop {w}")
    for w in ("step", "steps", "top", "half", "shop", "start", "store", "story", "forward", "what", "who", "hold"):
        check(not I.strong_stop_word(w), f"not strong stop {w}")
    check(I.safety_stop(["no"], True) and not I.safety_stop(["no"], False), "'no' needs the wake word")
    check(I.safety_stop(["please", "stop"], False), "'stop' works without wake word")
    objs = I.load_objects()
    names = [c for c, _ in objs]
    for want in ("trash can", "chair", "person", "backpack", "water bottle", "door", "table", "laptop", "cup", "box"):
        check(want in names, f"object {want} in objects.txt")
    check(I.find_object(objs, ["the", "garbage", "can"]) == "trash can", "multi-word alias beats 'can'")
    check(I.find_object(objs, ["water", "bottle"]) == "water bottle", "water bottle, not just water")
    check(I.find_object(objs, ["coffee", "cup"]) == "cup", "coffee cup -> cup")
    check(I.find_object(objs, ["the", "kitchen"]) is None, "unknown object -> None")
    check(I.decide(m, "tars go to the chair").lines == [I.GOTO_PREFIX + "GOTO chair", "BEEP 1"], "goto lines")
    check(I.decide(m, "tars stop").lines == ["S", I.CANCEL], "stop: S to tars_motor + CANCEL to go-to")
    check(I.decide(m, "tars go forward").lines[0] == I.CANCEL, "manual move cancels go-to first")
    d = I.decide(m, "tars go to the kitchen")
    check(d.action == "reject" and d.why == "no_object" and d.lines == ["LED no", "BEEP 3"], f"unknown object rejected {d}")

    slot_cases = {
        "tars go forward": "MOVE 300", "tars go back": "MOVE -300",
        "tars turn left": "TURN -30", "tars turn right": "TURN 30",
        "tars turn left ninety": "TURN -90", "tars turn right 45 degrees": "TURN 45",
        "tars turn right forty five degrees": "TURN 45", "tars turn left one eighty degrees": "TURN -180",
        "tars turn right one hundred and eighty degrees": "TURN 180", "tars turn left a hundred and twenty": "TURN -120",
        "tars go forward two hundred and fifty millimeters": "MOVE 250",
        "tars turn right a quarter turn": "TURN 90", "tars turn left a little": "TURN -15",
        "tars turn right 720 degrees": "TURN 360", "tars turn right 2 meters": "TURN 30",
        "tars go forward half a metre": "MOVE 500", "tars go forward one meter": "MOVE 1000",
        "tars go forward two steps": "MOVE 200", "tars go forward 1.5 meters": "MOVE 1000",
        "tars go forward 0.5 m": "MOVE 500", "tars go forward 50 centimeters": "MOVE 500",
        "tars go forward fifty": "MOVE 500", "tars go forward 3": "MOVE 300",
        "tars go forward 250": "MOVE 250", "tars go forward a foot": "MOVE 305",
        "tars go forward a bit": "MOVE 100", "tars back up one and a half feet": "MOVE -458",
        "tars go back to meters": "MOVE -1000", "tars move forward 100 millimeters": "MOVE 100",
        "tars go forward three hundred millimeters": "MOVE 300", "tars go back a hundred": "MOVE -1000",
        "tars turn around": "TURN 180", "tars spin around": "TURN 180", "tars do a full spin": "TURN 360",
        "tars status": "STATUS",
        "tars go to the trash can": "GOTO trash can", "tars drive to the chair": "GOTO chair",
        "tars find the door": "GOTO door", "tars head over to that bin": "GOTO trash can",
        "tars go to the garbage": "GOTO trash can", "tars go to the trashcan": "GOTO trash can",
        "tars go to the garbage can": "GOTO trash can", "tars find the water bottle": "GOTO water bottle",
        "tars go to the bottle": "GOTO water bottle", "tars find my back pack": "GOTO backpack",
        "tars go to the backpack": "GOTO backpack", "tars go to the computer": "GOTO laptop",
        "tars go to the desk": "GOTO table", "tars find a mug": "GOTO cup", "tars go to the boxes": "GOTO box",
        "tars go to that guy": "GOTO person", "tars go to the chare": "GOTO chair",
        "tars go to the kitchen": "LED no", "tars find the giraffe": "LED no",
        "tars look around": "SCAN", "tars what do you see": "SCAN", "tars scan the room": "SCAN",
        "tars scan": "SCAN", "tars take a look around": "SCAN",
        "tars forget everything": "FORGET", "tars clear your memory": "FORGET",
        "tars forget the bin": "FORGET trash can", "tars forget where the chair is": "FORGET chair",
        "tars what do you remember": "MEMORY SAY", "tars what's in your memory": "MEMORY SAY",
        "tars tell me what you remember": "MEMORY SAY",
        "tars go back to where the bin was": "RECALL trash can", "tars go back to the chair": "RECALL chair",
        "tars return to the trash can": "RECALL trash can", "tars go to where the backpack was": "RECALL backpack",
        "tars go back to where the giraffe was": "LED no",
    }
    for text, want in slot_cases.items():
        d = I.decide(m, text)
        check(first_line(d) == want, f"slot {text!r}: want {want}, got {d.lines} ({d.intent} {d.prob:.2f})")
    return list(slot_cases)


# ------------------------------------------------------------------ 1b. regressions (bench voice lab)
MOTION = ("MOVE", "TURN", I.GOTO_PREFIX + "GOTO", I.GOTO_PREFIX + "RECALL")


def test_regressions(m: I.Model) -> list[str]:
    """Bugs found by the laptop bench voice lab. Each must stay fixed, in Python and C (parity)."""
    print("regressions: bare fragments")
    texts = []
    # 1. a motion verb with no direction must never move ("TARS, turn." used to give TURN 180)
    bare = ["TARS, turn.", "tars turn", "Tars, turn!", "tars turn turn", "tars turn turn turn",
            "tars turn the", "tars turn it", "tars turn now", "tars uh turn", "tars turning",
            "tars move", "tars move it", "tars move it move it", "tars just move", "tars move now please",
            "tars rotate", "tars rotate it", "tars, tern.", "tars go to", "tars go to the"]
    for t in bare:
        d = I.decide(m, t)
        check(d.action == "reject" and d.lines == ["LED no", "BEEP 3"] and not any(l.startswith(MOTION) for l in d.lines),
              f"bare fragment rejected {t!r}: {d}")
    check(I.decide(m, "tars turn").why == "bare", "bare turn: why=bare")
    # ...but a stop inside one still stops, and real commands with the same verbs still work
    for t in ("tars turn stop", "turn turn stop", "tars move no", "tars rotate wait"):
        check(I.decide(m, t).lines == ["S", I.CANCEL], f"stop wins over bare {t!r}")
    still = {"tars turn left": "TURN -30", "tars turn right": "TURN 30", "tars turn around": "TURN 180",
             "tars move forward": "MOVE 300", "tars move back": "MOVE -300", "tars rotate left": "TURN -30",
             "tars spin": "TURN 360", "TARS. Go!": "MOVE 300", "tars back up a bit": "MOVE -100",
             "tars move to the chair": "GOTO chair", "tars turn to the left": "TURN -30"}
    for t, want in still.items():
        d = I.decide(m, t)
        check(first_line(d) == want, f"not bare {t!r}: want {want}, got {d.lines} ({d.intent} {d.prob:.2f})")
    return texts + bare + list(still)


# ------------------------------------------------------------------ 2. held-out accuracy
def evaluate(m: I.Model, rows) -> None:
    K = len(I.INTENTS)
    conf = [[0] * K for _ in range(K)]
    ok_accept = wrong_accept = rejected = 0
    for text, intent, _ in rows:
        _, toks = I.strip_wake(I.normalize(text))
        p = I.predict(m, toks)
        k = max(range(K), key=lambda i: p[i])
        conf[I.INTENTS.index(intent)][k] += 1
        d = I.decide(m, text, require_wake=False)
        if d.action in ("command", "stop"):
            if d.intent == intent:
                ok_accept += 1
            else:
                wrong_accept += 1
        else:
            rejected += 1
    n = len(rows)
    acc = sum(conf[i][i] for i in range(K)) / n
    print(f"held-out (unseen templates): {n} sentences, classifier accuracy {acc:.3f}")
    print(f"  full decision: correct action {ok_accept / n:.3f}, WRONG action {wrong_accept / n:.3f}, "
          f"rejected/ignored {rejected / n:.3f}")
    check(acc >= 0.80, f"held-out accuracy >= 0.80 (got {acc:.3f})")
    # wrong actions on non-unknown, non-stop rows; an unknown row that becomes S is fine
    check(wrong_accept / n <= 0.08, f"wrong-action rate <= 8% (got {wrong_accept / n:.3f})")

    lines = ["| intent | test sentences | precision | recall |", "|---|---|---|---|"]
    for i, name in enumerate(I.INTENTS):
        tp = conf[i][i]
        col = sum(conf[r][i] for r in range(K))
        row = sum(conf[i])
        lines.append(f"| {name} | {row} | {tp / col if col else 0:.2f} | {tp / row if row else 0:.2f} |")
    lines.append(f"| **all** | **{n}** | | **accuracy {acc:.3f}** |")
    short = ["fwd", "back", "left", "right", "around", "stop", "goto", "spin", "status", "scan", "forget", "mem", "recall", "unk"]
    cm = ["| true \\ predicted | " + " | ".join(short) + " |", "|---" * (K + 1) + "|"]
    for i in range(K):
        cm.append(f"| **{short[i]}** | " + " | ".join(str(conf[i][j]) if conf[i][j] else "·" for j in range(K)) + " |")
    if MD:
        print("\n" + "\n".join(lines) + "\n\nConfusion matrix (classifier only, before thresholds):\n\n" + "\n".join(cm) + "\n")
    else:
        print("\n".join("  " + l for l in lines))
        print("  confusion (rows = true):")
        print("\n".join("  " + l for l in cm))
    return acc, ok_accept / n, wrong_accept / n, rejected / n


# ------------------------------------------------------------------ 3. realistic
def load_realistic():
    out = []
    with open(os.path.join(VOICE, "intent", "data", "realistic.tsv")) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            out.append(tuple(line.rstrip("\n").split("\t")))
    return out


def test_realistic(m: I.Model):
    rows = load_realistic()
    good = wrong_motion = 0
    for text, want, cmd in rows:
        d = I.decide(m, text)
        got = d.intent if d.action in ("command", "stop") else d.action
        ok = (got == want or (want == "reject" and d.action in ("reject", "ignore"))) and (cmd == "-" or first_line(d) == cmd)
        good += ok
        moved = any(l.startswith(("MOVE", "TURN", I.GOTO_PREFIX + "GOTO")) for l in d.lines)
        if moved and not ok:
            wrong_motion += 1
            print(f"  WRONG MOTION {text!r}: want {want} {cmd}, got {d.lines}")
        elif not ok:
            print(f"  (safe miss) {text!r}: want {want} {cmd}, got {d.action}/{d.intent} {d.prob:.2f}")
    print(f"realistic hand-written: {good}/{len(rows)} exact; wrong motion: {wrong_motion}")
    check(good >= 0.9 * len(rows), f"realistic >= 90% ({good}/{len(rows)})")
    check(wrong_motion == 0, "no wrong motion on realistic set")
    return good, len(rows), [r[0] for r in rows]


# ------------------------------------------------------------------ 4. stop recall
def test_stop_recall(m: I.Model, gen_rows, real_rows):
    stops = [t for t, c, _ in gen_rows if c == "stop"] + [r[0] for r in real_rows if r[1] == "stop"]
    missed = [t for t in stops if I.decide(m, t).lines != ["S", I.CANCEL]]
    for t in missed[:10]:
        print(f"  STOP MISSED: {t!r}")
    print(f"stop recall: {len(stops) - len(missed)}/{len(stops)}")
    check(not missed, f"stop recall 100% ({len(stops) - len(missed)}/{len(stops)})")
    # and stop words inside other sentences still stop
    for t in ("tars go forward stop", "TARS turn left no stop", "stars, go to the chair. STOP!", "forward stop"):
        check(I.decide(m, t).lines[0] == "S", f"stop wins in {t!r}")
    return len(stops), len(missed)


# ------------------------------------------------------------------ 5. parity
def run_cli(texts, *args):
    r = subprocess.run([CLI, *args], input="\n".join(texts) + "\n", capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, r.stderr
    return r.stdout.splitlines()


def py_line(m, text, require_wake=True):
    d = I.decide(m, text, require_wake)
    return [d.action, d.intent, d.prob, int(d.wake), d.why, " | ".join(d.lines)]


def test_parity(m: I.Model, texts):
    rng = random.Random(5)
    alphabet = "abcdefghijklmnopqrstuvwxyz  ...,'!?0123456789°é"
    fuzz = ["".join(rng.choice(alphabet) for _ in range(rng.randint(0, 40))) for _ in range(300)]
    fuzz += ["tars " + f for f in fuzz[:150]]
    texts = [t.replace("\n", " ") for t in texts + fuzz]
    for args, rw in (((), True), (("--any",), False)):
        out = run_cli(texts, *args)
        check(len(out) == len(texts), "cli line count")
        bad = 0
        for t, line in zip(texts, out):
            c = line.split("\t")
            p = py_line(m, t, rw)
            same = (c[0] == p[0] and c[1] == p[1] and abs(float(c[2]) - p[2]) < 1e-4
                    and c[3] == str(p[3]) and c[4] == p[4] and c[5] == p[5])
            if not same:
                bad += 1
                if bad <= 5:
                    print(f"  PARITY {t!r}\n    C : {c}\n    py: {p}")
        print(f"parity ({'wake required' if rw else 'any'}): {len(texts) - bad}/{len(texts)} identical")
        check(bad == 0, f"Python/C parity {'wake' if rw else 'any'}")
    return len(texts)


# ------------------------------------------------------------------ 6. latency
def test_latency(m: I.Model, texts):
    out = run_cli(texts, "--bench", "20")
    c_us = float(out[0].split()[0])
    t0 = time.perf_counter()
    for t in texts[:300]:
        I.decide(m, t)
    py_us = (time.perf_counter() - t0) / min(300, len(texts)) * 1e6
    print(f"latency per sentence (this Mac): C {c_us:.1f} us, Python {py_us:.0f} us")
    check(c_us < 1000, "C decision under 1 ms")
    return c_us, py_us


if __name__ == "__main__":
    if not os.path.exists(CLI):
        sys.exit(f"{CLI} missing: run make -C iter2-autonomy/voice intent_cli")
    model = I.load_model()
    print(f"model: threshold {model.threshold}, stop_threshold {model.stop_threshold}")
    slot_texts = test_units(model) + test_regressions(model)
    gen = gen_data.generate(30, seed=13)
    test_rows = [r for r in gen if r[2] == "test"]
    acc, okr, wr, rj = evaluate(model, test_rows)
    good, nreal, real_texts = test_realistic(model)
    nstop, nmiss = test_stop_recall(model, gen, load_realistic())
    texts = [t for t, _, _ in test_rows] + real_texts + slot_texts
    npar = test_parity(model, texts)
    c_us, py_us = test_latency(model, texts)
    if MD:
        print(f"\nSummary: held-out accuracy {acc:.3f} on {len(test_rows)} sentences; full decision "
              f"correct {okr:.3f} / wrong {wr:.3f} / rejected {rj:.3f}; realistic {good}/{nreal}; "
              f"stop recall {nstop - nmiss}/{nstop}; parity {npar}x2 identical; C {c_us:.1f} us, Python {py_us:.0f} us.")
    print(f"{checks - failures}/{checks} checks passed")
    sys.exit(1 if failures else 0)
