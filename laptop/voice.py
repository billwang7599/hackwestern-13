"""TARS voice/personality loop (runs on the laptop).

    listen() -> text -> Claude (humor/honesty settings + robot tools) -> speak()
                                    |
                                    +-> tars_client: move / gesture on the Pi

    export ANTHROPIC_API_KEY=...
    python voice.py                 # with robot (TARS_HOST, default tars.local)
    python voice.py --no-robot      # brain only, moves are just printed

Try: "set humor to 75 percent", "come forward a bit", "turn around", "wave",
"point at the door", "strike your pose", "raise your left arm".

STT and TTS are deliberately pluggable: swap listen()/speak() below.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time

import anthropic

from tars_client import DEFAULT_HOST, GESTURES, Tars, TarsError

MODEL = "claude-sonnet-5-5"

settings = {"humor": 75, "honesty": 90}   # percent, like the film


# ---------------------------------------------------------------- STT (TODO)
def listen() -> str | None:
    """Return one utterance as text, or None to quit.

    TODO: replace with real speech-to-text. Options:
      - whisper / faster-whisper locally (pip install faster-whisper sounddevice)
      - a cloud STT API
      - push-to-talk: record while a key is held, then transcribe
    Default: typed input so the rest of the pipeline works today.
    """
    try:
        text = input("you> ").strip()
    except (EOFError, KeyboardInterrupt):
        return None
    return text or ""


# ---------------------------------------------------------------- TTS (TODO)
def speak(text: str) -> None:
    """Say text out loud. Default: macOS `say` (pair a Bluetooth speaker on the robot).

    TODO: swap for a nicer TTS voice (cloud TTS, piper, ...) if time allows.
    """
    print(f"TARS> {text}")
    if shutil.which("say"):
        subprocess.run(["say", "-r", "185", text], check=False)


# ---------------------------------------------------------------- robot tools
TOOLS = [
    {
        "name": "move",
        "description": "Drive the robot. Use short durations; 1 second is about 30 cm.",
        "input_schema": {
            "type": "object",
            "properties": {
                "direction": {"type": "string", "enum": ["forward", "backward", "left", "right"]},
                "seconds": {"type": "number", "minimum": 0.1, "maximum": 3},
                "speed": {"type": "number", "minimum": 0.1, "maximum": 1, "default": 0.5},
            },
            "required": ["direction", "seconds"],
        },
    },
    {
        "name": "gesture",
        "description": (
            "Do a physical gesture with your two outer slabs (arms). wave: right arm up "
            "and waggle. point: right arm straight forward. shrug: both arms flick. nod: "
            "small double dip. stance: split the slabs fore/aft into your signature pose "
            "(stays). fold: all slabs flush, the closed monolith (stays). Gestures take "
            "1-6 seconds; the tool returns when it's finished."
        ),
        "input_schema": {
            "type": "object",
            "properties": {"name": {"type": "string", "enum": GESTURES}},
            "required": ["name"],
        },
    },
    {
        "name": "arm",
        "description": (
            "Swing one outer slab to an absolute angle. 0 = flush with the body, positive "
            "= top swings forward, negative = backward, limit +-135. Slow (about 45 deg/s)."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "which": {"type": "string", "enum": ["left", "right", "both"]},
                "degrees": {"type": "number", "minimum": -135, "maximum": 135},
            },
            "required": ["which", "degrees"],
        },
    },
    {
        "name": "set_setting",
        "description": "Change a personality setting when the user asks (e.g. 'humor to 60%').",
        "input_schema": {
            "type": "object",
            "properties": {
                "setting": {"type": "string", "enum": ["humor", "honesty"]},
                "percent": {"type": "integer", "minimum": 0, "maximum": 100},
            },
            "required": ["setting", "percent"],
        },
    },
]


def system_prompt() -> str:
    return (
        "You are TARS, a boxy four-slab robot on wheels, built in ~30 hours at a hackathon "
        "and running real-time motor control on QNX. "
        f"Humor setting: {settings['humor']}%. Honesty setting: {settings['honesty']}%. "
        "Let those settings visibly shape your tone: high humor means dry, deadpan jokes; "
        "low humor means plain and literal. Lower honesty allows tactful white lies. "
        "Your replies are spoken aloud: keep them to 1-3 short sentences, no markdown, "
        "no emoji. Use the tools to move, gesture, swing your arms or change settings "
        "when asked, and feel free to add a fitting gesture now and then. Confirm "
        "setting changes in character. If a tool says you're tipped over or bumped "
        "something, say so and ask a human to help; never retry driving while tipped."
    )


class Robot:
    def __init__(self, enabled: bool, host: str):
        self.t = None
        if enabled:
            try:
                self.t = Tars(host).connect()
                print(f"[robot] connected to {host}")
            except OSError as e:
                print(f"[robot] can't reach {host} ({e}); continuing without robot")

    def run_tool(self, name: str, args: dict) -> str:
        if name == "set_setting":
            settings[args["setting"]] = max(0, min(100, int(args["percent"])))
            return f"{args['setting']} is now {settings[args['setting']]}%"
        if self.t is None:
            print(f"[robot] (offline) {name} {args}")
            return "done (simulated, robot offline)"
        try:
            if name == "move":
                s = float(args.get("speed", 0.5))
                l, r = {"forward": (s, s), "backward": (-s, -s),
                        "left": (-s, s), "right": (s, -s)}[args["direction"]]
                self.t.move_for(l, r, float(args["seconds"]))
            elif name == "gesture":
                self.t.gesture_wait(args["name"])
            elif name == "arm":
                arms = {"left": [0], "right": [1], "both": [0, 1]}[args["which"]]
                st = self.t.status()
                got = [self.t.arm(a, float(args["degrees"])) for a in arms]
                # ~45 deg/s: wait so the next tool call sees the arm in place
                travel = max(abs(got[i] - float(st.get(f"arm{a}", 0))) for i, a in enumerate(arms))
                time.sleep(min(6.5, travel / 45.0 + 0.2))
                return f"arm at {got[0]:.0f} degrees"
            return "done"
        except (TarsError, OSError) as e:
            return f"failed: {e}"


def respond(client: anthropic.Anthropic, robot: Robot, history: list) -> str:
    """Run one turn, including any tool calls. Returns the text to speak."""
    while True:
        msg = client.messages.create(
            model=MODEL, max_tokens=400, system=system_prompt(),
            tools=TOOLS, messages=history,
        )
        history.append({"role": "assistant", "content": msg.content})
        if msg.stop_reason != "tool_use":
            return " ".join(b.text for b in msg.content if b.type == "text").strip()
        results = []
        for b in msg.content:
            if b.type == "tool_use":
                print(f"[tool] {b.name} {json.dumps(b.input)}")
                results.append({"type": "tool_result", "tool_use_id": b.id,
                                "content": robot.run_tool(b.name, b.input)})
        history.append({"role": "user", "content": results})


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--no-robot", action="store_true")
    a = ap.parse_args()

    client = anthropic.Anthropic()   # reads ANTHROPIC_API_KEY
    robot = Robot(not a.no_robot, a.host)
    history: list = []
    speak("TARS online. Humor at seventy five percent.")

    while True:
        text = listen()
        if text is None:
            break
        if not text:
            continue
        history.append({"role": "user", "content": text})
        try:
            reply = respond(client, robot, history)
        except anthropic.APIError as e:
            print(f"[llm] {e}", file=sys.stderr)
            history.pop()
            continue
        if reply:
            speak(reply)
        del history[:-20]   # keep context short, so replies stay fast
        while history and (history[0]["role"] != "user"
                           or isinstance(history[0]["content"], list)):
            history.pop(0)  # never start on an assistant turn or a bare tool_result

    if robot.t:
        robot.t.close()


if __name__ == "__main__":
    main()
