"""Goalkeeper trash talk: on each save/goal, a one-line taunt from Claude, spoken
with macOS `say`. Runs in a background thread and never blocks the camera loop.

    t = Taunter()                 # ANTHROPIC_API_KEY from env; canned lines without it
    t.update(status["saves"], status["goals"])   # call whenever STATUS is polled
    t.enabled = False             # mute

Events are derived from the saves/goals counters going up (the first update
just sets the baseline; a RESET to lower counters re-baselines silently).
Rate-limited (one taunt per MIN_GAP_S, extra events while busy are dropped).
Offline fallbacks: canned lines when there is no key, no network, or an error.
"""
from __future__ import annotations

import os
import queue
import random
import shutil
import subprocess
import threading
import time

MODEL = "claude-sonnet-5-5"     # same as laptop/voice.py
MIN_GAP_S = 4.0

SYSTEM = ("You are GOALBOT, a smug little cardboard robot goalkeeper running on a "
          "real-time QNX brain at a hackathon. You trash-talk the humans shooting ping-pong "
          "balls at you. Reply with ONE short spoken line, max 14 words, no quotes, no emoji, "
          "PG, playful not mean.")

CANNED = {
    "save": ["Denied. My reflexes run on a real-time kernel.",
             "Nice try, carbon-based life form.",
             "Two hundred hertz of pure disrespect.",
             "Was that a shot or a suggestion?",
             "I predicted that before you blinked."],
    "goal": ["Lucky. I'm filing a bug report.",
             "That one was a scheduling anomaly.",
             "Enjoy it. I'm recalibrating.",
             "Fine. One for the humans. Don't get used to it.",
             "My paddle was buffering. Totally unfair."],
}


def speak(text: str) -> None:
    print(f"GOALBOT> {text}", flush=True)
    if shutil.which("say"):
        subprocess.run(["say", "-r", "190", text], check=False)


class Taunter:
    def __init__(self, enabled: bool = True, use_llm: bool | None = None,
                 speaker=speak, min_gap_s: float = MIN_GAP_S, llm=None):
        self.enabled = enabled
        self.speaker = speaker
        self.min_gap_s = min_gap_s
        self.last_text = ""
        self.last_t = 0.0
        self._base = None
        self._q: queue.Queue = queue.Queue(maxsize=1)
        self._last_spoken = -1e9
        self._llm = llm                      # callable(event, saves, goals) -> str, for tests
        if self._llm is None and (use_llm if use_llm is not None else bool(os.environ.get("ANTHROPIC_API_KEY"))):
            self._llm = self._claude
        self._client = None
        threading.Thread(target=self._worker, daemon=True, name="taunt").start()

    # -- called from the camera loop (cheap) -------------------------------
    def update(self, saves, goals) -> str | None:
        """Feed the latest counters. Returns 'save'/'goal' if an event fired."""
        if not isinstance(saves, int) or not isinstance(goals, int):
            return None
        prev, self._base = self._base, (saves, goals)
        if prev is None or saves < prev[0] or goals < prev[1]:
            return None
        ev = "save" if saves > prev[0] else "goal" if goals > prev[1] else None
        if ev and self.enabled:
            try:
                self._q.put_nowait((ev, saves, goals))
            except queue.Full:
                pass                          # busy talking: drop it
        return ev

    # -- background --------------------------------------------------------
    def _worker(self):
        while True:
            ev, saves, goals = self._q.get()
            if time.monotonic() - self._last_spoken < self.min_gap_s:
                continue
            text = None
            if self._llm:
                try:
                    text = self._llm(ev, saves, goals)
                except Exception as e:        # network, auth, rate limit...
                    print(f"[taunt] Claude unavailable ({type(e).__name__}), using canned line")
            text = (text or random.choice(CANNED[ev])).strip().strip('"')
            self.last_text, self.last_t = text, time.monotonic()
            self._last_spoken = time.monotonic()
            try:
                self.speaker(text)
            except Exception as e:
                print(f"[taunt] speak failed: {e}")

    def _claude(self, ev, saves, goals) -> str:
        if self._client is None:
            import anthropic
            self._client = anthropic.Anthropic(timeout=4.0, max_retries=0)
        what = ("You just SAVED a shot." if ev == "save" else "The human just SCORED on you.")
        msg = self._client.messages.create(
            model=MODEL, max_tokens=60, system=SYSTEM,
            messages=[{"role": "user", "content": f"{what} Score: robot {saves} saves, humans {goals} goals."}])
        return "".join(b.text for b in msg.content if b.type == "text").split("\n")[0]


if __name__ == "__main__":
    t = Taunter()
    t.update(0, 0)
    t.update(1, 0)
    time.sleep(6)
    t.update(1, 1)
    time.sleep(6)
