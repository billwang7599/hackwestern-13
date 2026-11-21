"""TARS voice/personality loop (runs on the laptop).

    listen() -> text -> Claude (humor/honesty settings + robot tools) -> speak()
                                    |
                                    +-> tars_client: move / gesture on the Pi

    export ANTHROPIC_API_KEY=...
    python voice.py                 # with robot (TARS_HOST, default tars.local)
    python voice.py --no-robot      # brain only, moves are just printed

Try: "set humor to 75 percent", "come forward a bit", "turn around", "wave".

STT and TTS are deliberately pluggable: swap listen()/speak() below.
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys

import anthropic

from tars_client import DEFAULT_HOST, Tars, TarsError

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
        "description": "Do a physical gesture with the slabs.",
        "input_schema": {
            "type": "object",
            "properties": {"name": {"type": "string", "enum": ["wave", "nod"]}},
            "required": ["name"],
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
        "no emoji. Use the tools to move, gesture or change settings when asked; "
        "confirm setting changes in character."
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
                self.t.gesture(args["name"])
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
