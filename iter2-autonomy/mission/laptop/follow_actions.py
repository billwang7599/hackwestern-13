"""Runs on the Pi over SSH stdin. Follows both journals across creation and rotation."""
import os
import time
from pathlib import Path

root = Path(os.getenv("TARS_ACTION_DIR", Path.home()))
paths = [root / f"tars-{name}-actions.log" for name in ("input", "motor", "nav")]
streams = {}
first = set(paths)
print("Watching motor commands, replies and navigation events on the Pi (UTC).", flush=True)
while True:
    lines = []
    for path in paths:
        stream = streams.get(path)
        if stream:
            lines.extend(stream.readlines())
        try:
            info = path.stat()
            if stream and os.path.samestat(info, os.fstat(stream.fileno())):
                continue
            new = path.open(errors="replace")
        except FileNotFoundError:
            first.discard(path)
            continue
        if stream:
            lines.extend(stream.readlines())
            stream.close()
        streams[path] = new
        if path in first and info.st_size > 8192:
            new.seek(info.st_size - 8192)
            new.readline()
        first.discard(path)
        lines.extend(new.readlines())
    for line in sorted(lines, key=lambda item: item.partition(" ")[0]):
        print(line, end="")
    time.sleep(0.2)
