"""Text -> robot command, reference implementation (Python).

This file is the SPEC. intent.c implements exactly the same steps and
tests/test_intent.py checks they agree on every test sentence (parity).

    whisper text
      -> normalize()            lowercase ASCII words, "90°" -> "90", "2m" -> "2 m"
      -> strip_wake()           "tars"/"stars"/"cars"... as the first word -> wake=True
      -> safety_stop()          stop/halt/freeze (fuzzy) -> S, no matter what else
      -> features() + model     hashed char 2-4 grams + words -> logistic regression
      -> parse_amount()         "ninety", "half a metre", "two steps", "45 degrees"
      -> decide()               Decision(action, lines to send to tars_motor)

Python standard library only at runtime (numpy is only for training, train.py).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

INTENTS = ["forward", "back", "left", "right", "turn_around", "stop",
           "follow", "gestures", "teach", "spin", "status", "unknown"]
N_FEAT = 4096                       # hashed feature bins (power of two)

DEFAULT_TURN_DEG = 30
DEFAULT_MOVE_MM = 300
STEP_MM = 100                       # "two steps" = 200 mm
SMALL_MM, SMALL_DEG = 100, 15       # "a bit", "a little"
MAX_MM, MAX_DEG = 1000, 360         # tars_motor clamps MOVE/TURN to these too

# ---------------------------------------------------------------- normalize
def normalize(text: str) -> list[str]:
    """Lowercase ASCII tokens. Letters and digits are kept, an apostrophe is
    dropped ("tar's" -> "tars"), a '.' between two digits is kept ("1.5"),
    a digit/letter boundary splits ("90deg" -> "90 deg"), anything else
    (punctuation, '°', non-ASCII bytes) is a space."""
    b = text.encode("utf-8", "replace")
    out = []
    prev = ""                                   # class of the last char written: d, a or ""
    n = len(b)
    skip = 0
    for i in range(n):
        if skip:
            skip -= 1
            continue
        if b[i] == 0xE2 and i + 2 < n and b[i + 1] == 0x80 and b[i + 2] in (0x98, 0x99):
            skip = 2                            # UTF-8 curly apostrophe: drop like "'"
            continue
        c = chr(b[i]) if b[i] < 128 else " "
        if "A" <= c <= "Z":
            c = c.lower()
        if c == "'":
            continue
        if c == "." and prev == "d" and i + 1 < n and chr(b[i + 1]).isdigit() and b[i + 1] < 128:
            out.append(".")
            continue                            # prev stays "d"
        if c.isdigit():
            cls = "d"
        elif "a" <= c <= "z":
            cls = "a"
        else:
            if prev:
                out.append(" ")
            prev = ""
            continue
        if prev and prev != cls:
            out.append(" ")
        out.append(c)
        prev = cls
    return "".join(out).split()


def edit_distance(a: str, b: str, cap: int = 3) -> int:
    """Levenshtein distance, stops counting at cap."""
    if abs(len(a) - len(b)) >= cap:
        return cap
    prev = list(range(len(b) + 1))
    for i in range(1, len(a) + 1):
        cur = [i] + [0] * len(b)
        for j in range(1, len(b) + 1):
            cur[j] = min(prev[j] + 1, cur[j - 1] + 1, prev[j - 1] + (a[i - 1] != b[j - 1]))
        prev = cur
    return min(prev[-1], cap)

# ---------------------------------------------------------------- wake word
WAKE_WORDS = {"tars", "tarz", "tar", "tares", "tarr", "stars", "star", "taurus", "torres",
              "tires", "tyres", "tours", "towers", "cars", "tarps", "tass", "tas", "task",
              "tarts", "tarse", "chars", "dars", "thars", "tahrs", "tarsh", "tarzan"}
WAKE_LEAD = {"hey", "ok", "okay", "yo", "hi", "uh", "um", "oh", "so", "a"}


def is_wake(tok: str) -> bool:
    return tok in WAKE_WORDS or (len(tok) >= 3 and edit_distance(tok, "tars", 2) <= 1)


def strip_wake(toks: list[str]) -> tuple[bool, list[str]]:
    """Wake word = first token, or second after hey/ok/..."""
    for i in range(min(2, len(toks))):
        if is_wake(toks[i]):
            return True, toks[i + 1:]
        if toks[i] not in WAKE_LEAD:
            break
    return False, toks

# ---------------------------------------------------------------- safety pass
STRONG_ROOTS = ("stop", "halt", "freeze", "whoa")         # fuzzy, work without the wake word
STRONG_EXTRA = {"stopit", "stopp", "stahp", "stawp", "frees", "freese", "freez", "woah",
                "halted", "stopping", "stopped", "freezing", "froze"}
NOT_STOP = {"step", "top", "shop", "atop", "half", "hall", "salt", "malt", "hat", "alt",
            "who", "whom", "what", "sop", "spot", "stomp"}
WEAK_STOP = {"no", "nope", "nah", "wait", "hold", "enough", "cancel", "brake", "stay"}  # need the wake word


def strong_stop_word(tok: str) -> bool:
    if tok in NOT_STOP or len(tok) < 3:
        return False
    if tok in STRONG_EXTRA:
        return True
    for r in STRONG_ROOTS:
        if tok == r or tok.startswith(r) or edit_distance(tok, r, 2) <= 1:
            return True
    return False


def safety_stop(toks: list[str], wake: bool) -> bool:
    if any(strong_stop_word(t) for t in toks):
        return True
    return wake and any(t in WEAK_STOP for t in toks)

# ---------------------------------------------------------------- features
def fnv1a(data: bytes) -> int:
    h = 2166136261
    for x in data:
        h = ((h ^ x) * 16777619) & 0xFFFFFFFF
    return h


def features(toks: list[str]) -> dict[int, float]:
    """Hashed char 2..4-grams of ' '+text+' ' plus whole words (prefixed 0x01),
    counts, L2-normalised. Returns {bin: value}."""
    s = (" " + " ".join(toks) + " ").encode("ascii")
    f: dict[int, float] = {}
    for n in (2, 3, 4):
        for i in range(len(s) - n + 1):
            k = fnv1a(s[i:i + n]) & (N_FEAT - 1)
            f[k] = f.get(k, 0.0) + 1.0
    for t in toks:
        k = fnv1a(b"\x01" + t.encode("ascii")) & (N_FEAT - 1)
        f[k] = f.get(k, 0.0) + 1.0
    norm = math.sqrt(sum(v * v for v in f.values())) or 1.0
    return {k: v / norm for k, v in f.items()}

# ---------------------------------------------------------------- numbers / slots
ONES = {"zero": 0, "oh": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
        "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12,
        "thirteen": 13, "fourteen": 14, "fifteen": 15, "sixteen": 16, "seventeen": 17,
        "eighteen": 18, "nineteen": 19}
TENS = {"twenty": 20, "thirty": 30, "forty": 40, "fourty": 40, "fifty": 50, "sixty": 60,
        "seventy": 70, "eighty": 80, "ninety": 90}
SOUNDALIKE = {"to": 2, "too": 2, "for": 4, "won": 1, "ate": 8}    # only right before a unit
UNITS = {"degree": "deg", "degrees": "deg", "deg": "deg", "degs": "deg",
         "metre": "m", "metres": "m", "meter": "m", "meters": "m", "m": "m",
         "centimetre": "cm", "centimetres": "cm", "centimeter": "cm", "centimeters": "cm", "cm": "cm",
         "millimetre": "mm", "millimetres": "mm", "millimeter": "mm", "millimeters": "mm", "mm": "mm",
         "foot": "ft", "feet": "ft", "ft": "ft", "inch": "in", "inches": "in",
         "step": "step", "steps": "step"}


def _num_token(t: str):
    """Value of one digit token ("45", "1.5"), else None."""
    if t and (t[0].isdigit()):
        try:
            return float(t)
        except ValueError:
            return None
    return None


def _after_hundred(toks: list[str], j: int, v: int) -> tuple[int, int]:
    """'(one) hundred [and] eighty [five]' -> add the tens/ones after 'hundred'."""
    n = len(toks)
    k = j + 1 if j < n and toks[j] == "and" else j
    if k < n and toks[k] in TENS:
        v, k = v + TENS[toks[k]], k + 1
        if k < n and toks[k] in ONES and 0 < ONES[toks[k]] < 10:
            v, k = v + ONES[toks[k]], k + 1
        return v, k
    if k < n and toks[k] in ONES and ONES[toks[k]] > 0:
        return v + ONES[toks[k]], k + 1
    return v, j


def parse_amount(toks: list[str]) -> tuple[float | None, str | None]:
    """First amount in the sentence -> (value, unit). unit is deg|m|cm|mm|ft|in|step|small|None.
    value None = no amount. 'a bit'/'a little' -> (1, 'small')."""
    n = len(toks)
    i = 0
    while i < n:
        t = toks[i]
        if t in ("bit", "little", "tad", "smidge") and i > 0 and toks[i - 1] in ("a", "little"):
            return 1.0, "small"
        val, j = None, i
        d = _num_token(t)
        if d is not None:
            val, j = d, i + 1
        elif t in ONES or t in TENS:
            v = 0
            if t in TENS:
                v, j = TENS[t], i + 1
                if j < n and toks[j] in ONES and 0 < ONES[toks[j]] < 10:
                    v, j = v + ONES[toks[j]], j + 1
            else:
                v, j = ONES[t], i + 1
                if 0 < v < 10 and j < n and (toks[j] in TENS or toks[j] in ("hundred",)):
                    if toks[j] == "hundred":
                        v, j = _after_hundred(toks, j + 1, v * 100)
                    else:                                   # "one eighty" = 180
                        v, j = v * 100 + TENS[toks[j]], j + 1
                        if j < n and toks[j] in ONES and 0 < ONES[toks[j]] < 10:
                            v, j = v + ONES[toks[j]], j + 1
            if j < n and toks[j] == "hundred" and v < 10:
                v, j = _after_hundred(toks, j + 1, v * 100)
            val = float(v)
        elif t == "hundred":
            v, j = _after_hundred(toks, i + 1, 100)
            val = float(v)
        elif t == "half" or t == "quarter":
            val, j = (0.5 if t == "half" else 0.25), i + 1
            if j < n and toks[j] in ("a", "an"):
                j += 1
        elif t in ("a", "an") and i + 1 < n and toks[i + 1] in UNITS:
            val, j = 1.0, i + 1
        elif t in SOUNDALIKE and i + 1 < n and toks[i + 1] in UNITS and toks[i + 1] not in ("m",):
            val, j = float(SOUNDALIKE[t]), i + 1
        if val is not None:
            if j + 2 < n and toks[j] == "and" and toks[j + 1] == "a" and toks[j + 2] == "half":
                val, j = val + 0.5, j + 3                   # "one and a half"
            elif j + 1 < n and toks[j] == "and" and toks[j + 1] == "half":
                val, j = val + 0.5, j + 2
            unit = UNITS.get(toks[j]) if j < n else None
            if unit is None and j + 1 < n and toks[j] in ("a", "of") and toks[j + 1] in UNITS:
                unit = UNITS[toks[j + 1]]                   # "half a metre"
            return val, unit
        i += 1
    return None, None


def move_mm(val: float | None, unit: str | None) -> int:
    if val is None:
        return DEFAULT_MOVE_MM
    if unit == "small":
        mm = SMALL_MM
    elif unit == "m":
        mm = val * 1000
    elif unit == "cm":
        mm = val * 10
    elif unit == "mm":
        mm = val
    elif unit == "ft":
        mm = val * 305
    elif unit == "in":
        mm = val * 25
    elif unit == "step":
        mm = val * STEP_MM
    elif unit == "deg":
        return DEFAULT_MOVE_MM
    elif val <= 10:
        mm = val * STEP_MM            # bare small number = steps
    elif val <= 200:
        mm = val * 10                 # bare 11..200 = cm
    else:
        mm = val                      # bare > 200 = mm
    mm = int(round(mm))
    if mm <= 0:
        return DEFAULT_MOVE_MM
    return min(mm, MAX_MM)


def turn_deg(val: float | None, unit: str | None) -> int:
    if val is None:
        return DEFAULT_TURN_DEG
    if unit == "small":
        return SMALL_DEG
    if unit not in (None, "deg"):
        return DEFAULT_TURN_DEG
    if val < 1:                        # "half" -> half a turn, "quarter" -> 90
        val = val * 360
    d = int(round(val))
    if d <= 0:
        return DEFAULT_TURN_DEG
    return min(d, MAX_DEG)

# ---------------------------------------------------------------- model + decision
@dataclass
class Model:
    W: list[list[float]]                 # [K][N_FEAT]
    b: list[float]
    threshold: float
    stop_threshold: float


def load_model(path: str | None = None) -> Model:
    """Parse the generated C header (intent_model.h), so Python and C share
    exactly the same rounded float weights."""
    import re
    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "intent_model.h")
    with open(path) as f:
        src = f.read()
    names = re.search(r"intent_names\[INTENT_N\] = \{(.*?)\};", src, re.S).group(1)
    assert [x.strip().strip('"') for x in names.split(",")] == INTENTS, "intent list changed: retrain"
    assert int(re.search(r"#define INTENT_N_FEAT (\d+)", src).group(1)) == N_FEAT
    thr = float(re.search(r"#define INTENT_THRESHOLD ([\d.]+)f", src).group(1))
    sthr = float(re.search(r"#define INTENT_STOP_THRESHOLD ([\d.]+)f", src).group(1))
    num = lambda blob: [float(x.rstrip("f")) for x in re.findall(r"-?[\d.]+(?:e[-+]?\d+)?f", blob)]
    b = num(re.search(r"intent_b\[INTENT_N\] = \{(.*?)\};", src, re.S).group(1))
    wblob = re.search(r"intent_W\[INTENT_N\]\[INTENT_N_FEAT\] = \{(.*)\};", src, re.S).group(1)
    wblob = re.sub(r"/\*.*?\*/", "", wblob)
    flat = num(wblob)
    assert len(b) == len(INTENTS) and len(flat) == len(INTENTS) * N_FEAT
    W = [flat[k * N_FEAT:(k + 1) * N_FEAT] for k in range(len(INTENTS))]
    return Model(W, b, thr, sthr)


def predict(model: Model, toks: list[str]) -> list[float]:
    """Softmax probabilities, float32-free reference (C uses float; parity tol 1e-4)."""
    f = features(toks)
    z = [model.b[k] + sum(model.W[k][i] * v for i, v in f.items()) for k in range(len(INTENTS))]
    m = max(z)
    e = [math.exp(x - m) for x in z]
    s = sum(e)
    return [x / s for x in e]


@dataclass
class Decision:
    action: str                    # ignore | stop | command | reject
    intent: str = "-"
    prob: float = 0.0
    wake: bool = False
    why: str = ""                  # safety | classifier | low_conf | unknown | no_wake | empty
    lines: list[str] = field(default_factory=list)    # to tars_motor, in order


def command_lines(intent: str, toks: list[str]) -> list[str]:
    val, unit = parse_amount(toks)
    if intent == "forward":
        return [f"MOVE {move_mm(val, unit)}", "BEEP 1"]
    if intent == "back":
        return [f"MOVE -{move_mm(val, unit)}", "BEEP 1"]
    if intent == "left":
        return [f"TURN -{turn_deg(val, unit)}", "BEEP 1"]
    if intent == "right":
        return [f"TURN {turn_deg(val, unit)}", "BEEP 1"]
    if intent == "turn_around":
        return ["TURN 180", "BEEP 1"]
    if intent == "spin":
        return ["TURN 360", "BEEP 1"]
    if intent == "follow":
        return ["MODE follow", "BEEP 1"]
    if intent == "gestures":
        return ["MODE gesture", "BEEP 1"]
    if intent == "teach":
        return ["MODE teach", "BEEP 1"]
    if intent == "status":
        return ["STATUS", "BEEP 2"]
    return []


def decide(model: Model, text: str, require_wake: bool = True) -> Decision:
    toks = normalize(text)
    wake, rest = strip_wake(toks)
    if safety_stop(rest, wake):
        return Decision("stop", "stop", 1.0, wake, "safety", ["S"])
    if require_wake and not wake:
        return Decision("ignore", wake=wake, why="no_wake")
    if not rest:
        return Decision("ignore", wake=wake, why="empty")
    p = predict(model, rest)
    k = max(range(len(p)), key=lambda i: p[i])
    intent = INTENTS[k]
    if intent == "stop" and p[k] >= model.stop_threshold:
        return Decision("stop", intent, p[k], wake, "classifier", ["S"])
    if intent == "unknown" or p[k] < model.threshold:
        return Decision("reject", intent, p[k], wake, "unknown" if intent == "unknown" else "low_conf",
                        ["LED no", "BEEP 3"])
    return Decision("command", intent, p[k], wake, "classifier", command_lines(intent, rest))


if __name__ == "__main__":
    import sys
    mdl = load_model()
    for line in sys.stdin:
        d = decide(mdl, line.rstrip("\n"))
        print(f"{d.action}\t{d.intent}\t{d.prob:.4f}\t{int(d.wake)}\t{d.why}\t{' | '.join(d.lines)}")
