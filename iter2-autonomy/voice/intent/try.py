#!/usr/bin/env python3
"""Type a sentence, see what the robot would do. No robot, mic or whisper needed.

    python3 iter2-autonomy/voice/intent/try.py                  # interactive
    python3 iter2-autonomy/voice/intent/try.py "tars turn lift 90"   # one-shot
    python3 iter2-autonomy/voice/intent/try.py --any            # don't require the wake word

Prints the intent, its confidence, the next two guesses, and the exact lines
that would go to tars_motor. Uses the current intent_model.h (the same weights
as the C code on the Pi).
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import intent as I  # noqa: E402


def show(m: I.Model, text: str, require_wake: bool) -> None:
    toks = I.normalize(text)
    wake, rest = I.strip_wake(toks)
    d = I.decide(m, text, require_wake)
    print(f"  words   : {' '.join(rest) or '-'}   (wake word: {'yes' if wake else 'NO'})")
    if rest:
        p = I.predict(m, rest)
        top = sorted(range(len(p)), key=lambda k: -p[k])[:3]
        print("  guesses : " + ", ".join(f"{I.INTENTS[k]} {p[k]:.2f}" for k in top)
              + f"   (threshold {m.threshold:.2f})")
    what = {"ignore": "does nothing", "stop": "STOPS", "command": "does it", "reject": "doesn't understand"}[d.action]
    print(f"  robot   : {what}  [{d.action}, {d.why}]")
    print(f"  sends   : {' | '.join(d.lines) if d.lines else '(nothing)'}")


def main() -> None:
    args = [a for a in sys.argv[1:] if a != "--any"]
    require_wake = "--any" not in sys.argv
    m = I.load_model()
    if args:
        show(m, " ".join(args), require_wake)
        return
    print("Type what you'd say to the robot (e.g. 'tars turn left ninety'). Ctrl-D to quit.")
    if require_wake:
        print("(the wake word 'tars' is required; start with --any to skip it)")
    while True:
        try:
            text = input("> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if text.strip():
            show(m, text, require_wake)


if __name__ == "__main__":
    main()
