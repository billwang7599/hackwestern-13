#!/usr/bin/env python3
"""Regenerate recordings/sample-sim-room.jsonl: the sim phone in the default room on the
scripted scan path, every 10th scene (28 scenes, < 1 MB). SYNTHETIC, not a phone capture.
    python3 bench/phone/make_sample.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from phone import sim  # noqa: E402

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings", "sample-sim-room.jsonl")
os.makedirs(os.path.dirname(out), exist_ok=True)
phone, path = sim.SimPhone(seed=1, session="sample-sim-room"), sim.ScriptedPath()
with open(out, "w") as f:
    f.write(json.dumps({"bench_phone_recording": 1, "source": "sim (synthetic, make_sample.py)",
                        "created": "generated"}) + "\n")
    for k in range(0, 281, 10):
        t = k / 10
        f.write(json.dumps({"t": t, "scene": phone.scene(1000.0 + t, *path.pose(t))},
                           separators=(",", ":")) + "\n")
print(out, os.path.getsize(out), "bytes")
