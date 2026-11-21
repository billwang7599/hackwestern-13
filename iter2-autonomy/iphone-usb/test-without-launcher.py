"""Briefly pause QNX's USB auto-configurer while testing usbmuxd, then resume it."""
import os
from pathlib import Path
import signal
import subprocess
import sys

if os.geteuid() != 0:
    raise SystemExit('Run with sudo')
pid = int(sys.argv[1])
here = Path(__file__).resolve().parent
process = None
os.kill(pid, signal.SIGSTOP)
try:
    process = subprocess.Popen(['sh', str(here / 'run-usbmuxd.sh')])
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
finally:
    os.kill(pid, signal.SIGCONT)
    print('USB launcher resumed', flush=True)
