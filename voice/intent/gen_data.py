"""Generate intent training/test sentences that look like whisper.cpp output.

The phrasings (templates) come from phrases.txt (humans edit that file). Templates are split 80/20 by a
stable hash: the TEST templates are never seen in training, so the held-out
accuracy measures new phrasings, not memorised ones. On top of that every
sentence gets whisper-style damage:

  - wake word: "TARS," / "Tars." / "hey tars" / misheard ("stars", "cars",
    "tires", "taurus", "tar's") / missing
  - homophones: forward -> "for word", left -> "lift", right -> "write",
    back -> "bag", turn -> "tern", around -> "a round", ...
  - dropped words, merged words ("turnleft"), letter typos, fillers ("uh",
    "please"), punctuation and capitals ("Turn left.")
  - numbers as digits or words ("90", "ninety", "90°", "45 degrees")

Output: TSV lines  text<TAB>intent<TAB>split   (split = train | test)
"""
from __future__ import annotations

import random
import sys
import zlib

import os

PHRASES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "phrases.txt")
INTENT_NAMES = ["forward", "back", "left", "right", "turn_around", "stop",
                "goto", "spin", "status", "unknown"]
# goto templates: {o} is filled with an alias from objects.txt (most of the
# time) or with a thing that's NOT in the list (still intent goto; the robot then
# says "don't know that object" with LED no + BEEP 3).
NOT_OBJECTS = ["kitchen", "window", "wall", "stairs", "car", "tree", "plant", "couch", "fridge",
               "printer", "sign", "stage", "corner", "lamp", "phone", "whiteboard", "elevator", "moon"]


def load_phrases(path: str = PHRASES) -> dict[str, list[str]]:
    """phrases.txt -> {intent: [phrase, ...]}. Raises ValueError with the line
    number on a bad line, an unknown intent, or a phrase listed twice."""
    out: dict[str, list[str]] = {k: [] for k in INTENT_NAMES}
    seen: dict[str, tuple[str, int]] = {}
    errors = []
    with open(path, encoding="utf-8") as f:
        for ln, line in enumerate(f, 1):
            line = line.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            if "\t" not in line:
                errors.append(f"{path}:{ln}: expected intent<TAB>phrase, got {line!r}")
                continue
            intent, phrase = (x.strip() for x in line.split("\t", 1))
            phrase = " ".join(phrase.lower().split())
            if intent not in out:
                errors.append(f"{path}:{ln}: unknown intent {intent!r} (use one of {', '.join(INTENT_NAMES)})")
                continue
            if not phrase:
                errors.append(f"{path}:{ln}: empty phrase")
                continue
            if phrase in seen:
                other, oln = seen[phrase]
                if other != intent:
                    errors.append(f"{path}:{ln}: {phrase!r} is listed as {intent} here and as {other} on line {oln}")
                continue
            seen[phrase] = (intent, ln)
            out[intent].append(phrase)
    for k, v in out.items():
        if not v:
            errors.append(f"{path}: no phrases for intent {k!r}")
    if errors:
        raise ValueError("\n".join(errors))
    return out


TURN_AMOUNTS = ["90", "ninety", "45", "forty five", "45 degrees", "90 degrees", "ninety degrees",
                "30 degrees", "thirty degrees", "a bit", "a little", "one eighty", "180",
                "fifteen degrees", "60", "sixty degrees", "20 degrees", "120 degrees", "90°",
                "10 degrees", "a quarter turn", "135"]
DIST_AMOUNTS = ["a meter", "one meter", "half a metre", "half a meter", "two steps", "one step",
                "three steps", "50 centimeters", "fifty centimeters", "1 meter", "30 cm",
                "a foot", "two feet", "20 centimeters", "a bit", "a little", "100 millimeters",
                "1.5 meters", "one and a half meters", "five steps", "10 cm", "0.5 m"]

WAKE = ["tars", "TARS,", "Tars,", "Tars.", "TARS.", "hey tars", "hey TARS,", "okay tars",
        "Stars,", "stars", "Cars,", "cars", "tires", "Taurus,", "tar's", "Tarz,", "Tars!",
        "towers", "tours", "Tons,", "Tones,"]

HOMO = {
    "forward": ["for word", "four word", "foreword", "forwards", "for ward", "foward", "forword"],
    "forwards": ["for words", "forward"],
    "left": ["lift", "lef", "laughed", "lest", "loft"],
    "right": ["write", "rite", "ride", "white", "bright", "wright", "rights"],
    "back": ["bag", "pack", "bach", "black", "bad", "backs"],
    "turn": ["tern", "turned", "burn", "torn", "tun", "turns"],
    "around": ["a round", "round", "arround"],
    "go": ["goal", "go,", "goes", "ago"],
    "follow": ["hollow", "fallow", "follo", "following"],
    "spin": ["spinning", "pin", "spun", "spend"],
    "stop": ["stopped", "stops", "stob", "stopping", "stap", "stahp"],
    "halt": ["halts", "hault", "halted"],
    "freeze": ["freezes", "frees", "breeze", "frieze"],
    "chair": ["share", "cheer", "chairs", "chare"],
    "trash": ["trish", "crash", "trashcan"],
    "bottle": ["bottel", "battle", "model"],
    "laptop": ["lap top", "laptops"],
    "backpack": ["back pack", "backpacks"],
    "box": ["bocks", "boxes", "fox"],
    "door": ["dour", "doors", "the door"],
    "table": ["tables", "cable"],
    "cup": ["cop", "cups"],
    "bin": ["been", "ben", "bean"],
    "find": ["fine", "found", "fined"],
    "status": ["stratus", "state us", "statues"],
    "gestures": ["gesture", "just yours", "jesters", "gestured"],
    "gesture": ["jester", "gestures"],
    "learn": ["learned", "lauren", "learns"],
    "reverse": ["reverb", "reversed"],
    "rotate": ["rotated", "row tate"],
    "move": ["moved", "moves", "moo"],
    "ahead": ["a head", "had"],
    "degrees": ["degree", "decrees", "degrease"],
    "meter": ["metre", "meters", "metal"],
    "ninety": ["90", "nineteen"],
}
KEY_WORDS = {"left", "right", "back", "forward", "forwards", "spin", "around", "round", "to",
             "find", "status", "report", "circle", "reverse", "backwards", "backward", "ahead"}
FILLERS = ["uh", "um", "please", "now", "okay", "so", "like", "just", "hey", "can you", "could you"]
# For the stop class: damage that keeps a stop word recognisable (whisper rarely
# mangles "stop" further than this; the safety pass must catch all of them).
STOP_DAMAGE = {"stop": ["stop", "Stop!", "stopped", "stops", "stob", "stopping", "stap", "stahp", "STOP"],
               "halt": ["halt", "halts", "hault"], "freeze": ["freeze", "frees", "frieze", "freezes"]}


def split_of(intent: str, template: str) -> str:
    if len(template.split()) == 1:
        return "train"            # single words are the core vocabulary: always learn them
    return "test" if zlib.crc32(f"{intent}|{template}".encode()) % 5 == 0 else "train"


_OBJECTS: list[list[str]] = []


def random_object(rng: random.Random) -> str:
    """Pick an object uniformly, then one of its names (so every object is seen
    equally often, however many aliases it has)."""
    if not _OBJECTS:
        import intent as I
        _OBJECTS.extend(al for _, al in I.load_objects())
    return rng.choice(rng.choice(_OBJECTS))


def fill(t: str, rng: random.Random) -> str:
    if "{o}" in t:
        o = random_object(rng) if rng.random() < 0.8 else rng.choice(NOT_OBJECTS)
        t = t.replace("{o}", o)
    if "{n}" in t:
        t = t.replace("{n}", rng.choice(TURN_AMOUNTS))
    if "{d}" in t:
        t = t.replace("{d}", rng.choice(DIST_AMOUNTS))
    return t


def typo(w: str, rng: random.Random) -> str:
    if len(w) < 4:
        return w
    i = rng.randrange(1, len(w) - 1)
    op = rng.random()
    if op < 0.4:
        return w[:i] + w[i + 1:]
    if op < 0.7:
        return w[:i] + w[i] + w[i:]
    return w[:i] + rng.choice("aeioustrnl") + w[i + 1:]


def damage(text: str, intent: str, rng: random.Random, level: float) -> str:
    words = text.split()
    out = []
    for w in words:
        lw = w.lower()
        if intent == "stop" and lw in STOP_DAMAGE:
            out.append(rng.choice(STOP_DAMAGE[lw]))
            continue
        if lw in HOMO and rng.random() < 0.35 * level and intent != "stop":
            out.append(rng.choice(HOMO[lw]))
            continue
        if intent != "stop" and rng.random() < 0.04 * level:
            out.append(typo(w, rng))
            continue
        out.append(w)
    # drop a filler-ish word (never the only word, never a stop word)
    # (only in 3+ word phrases, never a key word: dropping "around" from "turn
    # around" would teach the model that a bare "turn" means turn around)
    if len(out) > 2 and rng.random() < 0.12 * level and intent != "stop":
        k = rng.randrange(len(out))
        if out[k].lower().strip(",.") not in KEY_WORDS:
            out.pop(k)
    # merge two words
    if len(out) >= 2 and rng.random() < 0.06 * level and intent != "stop":
        k = rng.randrange(len(out) - 1)
        out[k:k + 2] = [out[k] + out[k + 1]]
    if rng.random() < 0.15 * level:
        out.insert(rng.randrange(len(out) + 1), rng.choice(FILLERS))
    s = " ".join(out)
    r = rng.random()
    if r < 0.3:
        s = s[:1].upper() + s[1:]
    elif r < 0.4:
        s = s.upper()
    s += rng.choice(["", "", ".", ".", "!", "?", ","])
    return s


def with_wake(s: str, intent: str, rng: random.Random) -> str:
    # The stop class must work with no wake word only via strong stop words;
    # the weak ones ("no", "wait", ...) need the wake word, so always add it.
    p_none = 0.1
    if intent == "stop":
        import intent as I
        if not any(I.strong_stop_word(t) for t in I.normalize(s)):
            p_none = 0.0           # weak stop words only count after the wake word
    if rng.random() < p_none:
        return s
    return f"{rng.choice(WAKE)} {s}"


def generate(per_template: int = 30, seed: int = 13) -> list[tuple[str, str, str]]:
    rng = random.Random(seed)
    rows = []
    for intent, temps in load_phrases().items():
        for t in temps:
            sp = split_of(intent, t)
            n = per_template if intent not in ("unknown",) else per_template // 2
            if len(t.split()) == 1 and intent == "unknown":
                n *= 3                 # bare "turn", "move", "what": easily outvoted by "turn left" etc.
            for _ in range(n):
                level = rng.choice([0.0, 0.5, 1.0, 1.0, 1.5])
                s = damage(fill(t, rng), intent, rng, level)
                rows.append((with_wake(s, intent, rng), intent, sp))
    # word salad from non-command words -> unknown
    salad = ("the a robot is this that cool nice demo judges table pizza hackathon laptop phone "
             "camera hello everyone thanks okay yes what how why when people team code demo time "
             "coffee water project battery cable wire table chair door window").split()
    for i in range(600):
        k = rng.randint(1, 6)
        s = " ".join(rng.choice(salad) for _ in range(k))
        rows.append((with_wake(s, "unknown", rng), "unknown", "test" if i % 5 == 0 else "train"))
    return rows


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    for text, intent, sp in generate(n):
        print(f"{text}\t{intent}\t{sp}")
