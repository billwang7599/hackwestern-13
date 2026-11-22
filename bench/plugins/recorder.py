"""bench plugin `recorder`: every event (bench.tap) to JSONL, and display-only replay.

Recording: one file per session in bench/recordings/ (gitignored), one line per event:
  {"t": <seconds since the recording started>, "topic": "...", "data": ...}
Recording starts with the bench (POST /api/recorder/record {"on": false} to pause) and starts
a new file on {"new": true}.

Replay re-publishes a recording to the browser with its original timing. **Replay never sends
anything to the robot**: this module doesn't import or call the robot plugin's send; it only
calls bench.publish. While replaying, every replayed event carries "_replay": true, the robot
plugin's own status/goto publishing pauses (so the views show the recording, not live data),
replayed events are not recorded again, and topic `replay` says so (the panel shows a banner).
STOP keeps working during replay (it goes to the live robot, as always).
"""
from __future__ import annotations

import json
import os
import re
import threading
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIR = os.path.join(HERE, "recordings")
SKIP_TOPICS = {"replay"}          # the recorder's own control topic

_bench = None
_rec = {"on": False, "path": None, "f": None, "t0": 0.0, "n": 0, "bytes": 0}
_lock = threading.Lock()
_play = {"active": False, "stop": threading.Event(), "name": None, "pos": 0.0, "len": 0.0, "speed": 1.0}


def _new_file() -> None:
    os.makedirs(DIR, exist_ok=True)
    with _lock:
        if _rec["f"]:
            _rec["f"].close()
        name = time.strftime("bench-%Y%m%d-%H%M%S") + ".jsonl"
        path = os.path.join(DIR, name)
        i = 1
        while os.path.exists(path):
            path = os.path.join(DIR, name.replace(".jsonl", f"-{i}.jsonl"))
            i += 1
        _rec.update(path=path, f=open(path, "a", buffering=1), t0=time.monotonic(), n=0, bytes=0)


def _tap(topic: str, data) -> None:
    if topic in SKIP_TOPICS or (isinstance(data, dict) and data.get("_replay")):
        return
    with _lock:
        if not _rec["on"] or not _rec["f"]:
            return
        line = json.dumps({"t": round(time.monotonic() - _rec["t0"], 3), "topic": topic, "data": data},
                          separators=(",", ":")) + "\n"
        _rec["f"].write(line)
        _rec["n"] += 1
        _rec["bytes"] += len(line)


def replaying() -> bool:
    return _play["active"]


def state() -> dict:
    with _lock:
        return {"recording": _rec["on"], "file": os.path.basename(_rec["path"] or ""), "events": _rec["n"],
                "bytes": _rec["bytes"], "replay": {k: _play[k] for k in ("active", "name", "pos", "len", "speed")}}


def recordings() -> list[dict]:
    if not os.path.isdir(DIR):
        return []
    out = []
    for f in sorted(os.listdir(DIR), reverse=True):
        if f.endswith(".jsonl"):
            p = os.path.join(DIR, f)
            out.append({"name": f, "bytes": os.path.getsize(p), "mtime": os.path.getmtime(p)})
    return out


def _safe_path(name: str) -> str | None:
    if not re.fullmatch(r"[\w.\-]+\.jsonl", name or ""):
        return None
    p = os.path.join(DIR, name)
    return p if os.path.isfile(p) else None


def _pub_state() -> None:
    _bench.publish("replay", {"active": _play["active"], "name": _play["name"], "pos": round(_play["pos"], 1),
                              "len": round(_play["len"], 1), "speed": _play["speed"]})


def _replay(path: str, speed: float, topics: set | None) -> None:
    """re-publish with original timing. Only bench.publish: nothing reaches the robot."""
    with open(path) as f:
        evs = []
        for line in f:
            try:
                e = json.loads(line)
                if topics is None or e.get("topic") in topics:
                    evs.append(e)
            except ValueError:
                continue
    _play.update(active=True, name=os.path.basename(path), pos=0.0, len=evs[-1]["t"] if evs else 0.0, speed=speed)
    _play["stop"].clear()
    _pub_state()
    _bench.log("replay", f"REPLAY {os.path.basename(path)} ({len(evs)} events, {_play['len']:.1f} s) - display only, nothing is sent")
    start = time.monotonic()
    last_state = 0.0
    try:
        for e in evs:
            due = start + e["t"] / speed
            while True:
                left = due - time.monotonic()
                if _play["stop"].wait(min(max(left, 0), 0.25)):
                    return
                if left <= 0.25:
                    break
            data = e.get("data")
            if isinstance(data, dict):
                data = dict(data, _replay=True)
            else:
                data = {"_replay": True, "value": data}
            _bench.publish(e["topic"], data, sticky=e["topic"] not in ("cmd", "log"))
            _play["pos"] = e["t"]
            if time.monotonic() - last_state > 0.5:
                last_state = time.monotonic()
                _pub_state()
    finally:
        _play["active"] = False
        _pub_state()
        _bench.log("replay", "replay ended: live data again")


def start_replay(name: str, speed: float = 1.0, topics: list[str] | None = None) -> dict:
    p = _safe_path(name)
    if not p:
        return {"error": "no such recording"}
    if _play["active"]:
        stop_replay()
        time.sleep(0.3)
    speed = min(20.0, max(0.1, float(speed)))
    threading.Thread(target=_replay, args=(p, speed, set(topics) if topics else None), daemon=True,
                     name="replay").start()
    return {"replaying": name, "speed": speed}


def stop_replay() -> None:
    _play["stop"].set()


def setup(bench) -> None:
    global _bench
    _bench = bench
    bench.tap(_tap)
    if not getattr(bench.args, "no_record", False):
        _rec["on"] = True
        _new_file()
    robot = bench.plugins.get("robot")
    if robot and getattr(robot, "link", None) and robot.link():
        robot.link().paused = replaying
    else:
        # robot may load after us (alphabetical: recorder < robot): hook up lazily
        def hook():
            for _ in range(50):
                r = bench.plugins.get("robot")
                if r and r.link():
                    r.link().paused = replaying
                    return
                time.sleep(0.1)
        threading.Thread(target=hook, daemon=True).start()

    def r_state(q, body):
        return 200, dict(state(), recordings=recordings()[:50])

    def r_record(q, body):
        d = json.loads(body or b"{}") or {}
        if d.get("new"):
            _new_file()
        if "on" in d:
            if d["on"] and not _rec["f"]:
                _new_file()
            _rec["on"] = bool(d["on"])
        return 200, state()

    def r_replay(q, body):
        d = json.loads(body or b"{}") or {}
        r = start_replay(d.get("name", ""), d.get("speed", 1.0), d.get("topics"))
        return (404 if "error" in r else 200), r

    def r_stop(q, body):
        stop_replay()
        return 200, {"ok": True}

    def r_get(q, body):
        p = _safe_path(q.get("name", ""))
        if not p:
            return 404, {"error": "no such recording"}
        with open(p, "rb") as f:
            return 200, (f.read(), "application/x-ndjson")

    bench.route("GET", "/api/recorder/state", r_state)
    bench.route("POST", "/api/recorder/record", r_record)
    bench.route("POST", "/api/recorder/replay", r_replay)
    bench.route("POST", "/api/recorder/stop", r_stop)
    bench.route("GET", "/api/recorder/file", r_get)
    _pub_state()


def shutdown() -> None:
    stop_replay()
    with _lock:
        if _rec["f"]:
            _rec["f"].close()
            _rec["f"] = None
