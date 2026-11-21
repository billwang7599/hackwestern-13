#!/usr/bin/env python3
"""Start / stop the virtual SM11 (bench/car/build/vuno). Laptop bench only.

For other bench parts (run_sim.py):

    sys.path.insert(0, "bench/car"); import launch as car
    proc, dev = car.start("sm11", truth_port=5590, ctl_port=5591, seed=1)
    motor = car.start_motor(dev, port=5555)        # optional: host tars_motor -b uno -d dev
    ...
    car.stop()                                    # stops everything started here

`dev` is a stable symlink to the pty (default <tempdir>/tars-vuno-<ctl_port>). Give THAT to
tars_motor's -d: after an "unplug" fault the pty is re-created under a new name and the
link is moved, so tars_motor's normal reconnect finds it.

CLI:
    python3 bench/car/launch.py [--profile sm11] [--truth 5590] [--ctl 5591] [--seed 1]
                                [--link PATH] [--motor PORT]   # also start tars_motor
    python3 bench/car/launch.py ctl '{"fault": "tip"}'          # send one control datagram
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
VUNO = os.path.join(HERE, "build", "vuno")
MOTOR = os.path.join(REPO, "iter1-wasd", "pi", "build", "tars_motor_host")
PROFILES = os.path.join(HERE, "profiles")

_procs: list[subprocess.Popen] = []


def build() -> None:
    """make -C bench/car (cheap when up to date)."""
    subprocess.run(["make", "-s", "-C", HERE], check=True)


def build_motor() -> None:
    subprocess.run(["make", "-s", "-C", os.path.join(REPO, "iter1-wasd", "pi"), "host"], check=True,
                   stdout=subprocess.DEVNULL)


def profile_path(profile: str) -> str:
    if os.path.sep in profile or profile.endswith(".json"):
        return profile
    return os.path.join(PROFILES, profile + ".json")


def start(profile: str = "sm11", truth_port: int = 5590, ctl_port: int = 5591, seed: int = 1,
          link: str | None = None, log=None, extra: list[str] | None = None,
          timeout: float = 5.0) -> tuple[subprocess.Popen, str]:
    """Start the virtual car. Returns (proc, device path for tars_motor -d).
    truth_port 0 = no default truth destination (subscribe with {"sub": port} instead).
    log: file object for the car's stderr (default: inherit)."""
    build()
    link = link or os.path.join(tempfile.gettempdir(), f"tars-vuno-{ctl_port}")
    cmd = [VUNO, "--profile", profile_path(profile), "--truth", str(truth_port),
           "--ctl", str(ctl_port), "--seed", str(seed), "--link", link, *(extra or [])]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=log, text=True)
    deadline = time.time() + timeout
    got = None
    while time.time() < deadline:
        line = p.stdout.readline()
        if not line:
            break
        if line.startswith("LINK "):
            got = line.split(" ", 1)[1].strip()
            break
    if not got:
        p.kill()
        raise RuntimeError(f"vuno didn't start (exit {p.poll()}): {' '.join(cmd)}")
    _procs.append(p)
    return p, got


def start_motor(dev: str, port: int = 5555, guard_port: int = 0, watchdog_ms: int = 500,
                log=None, extra: list[str] | None = None) -> subprocess.Popen:
    """Host tars_motor, UNCHANGED, on the virtual (or real) Uno: -b uno -d dev."""
    build_motor()
    p = subprocess.Popen([MOTOR, "-b", "uno", "-d", dev, "-p", str(port), "-r", "0",
                          "-g", str(guard_port), "-w", str(watchdog_ms), *(extra or [])],
                         stdout=log or subprocess.DEVNULL, stderr=log or subprocess.DEVNULL)
    deadline = time.time() + 5
    while time.time() < deadline:
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.3).close()
            break
        except OSError:
            if p.poll() is not None:
                raise RuntimeError(f"tars_motor exited {p.returncode}")
            time.sleep(0.05)
    else:
        p.kill()
        raise RuntimeError(f"tars_motor never listened on {port}")
    _procs.append(p)
    return p


def stop(proc: subprocess.Popen | None = None) -> None:
    """Stop one process, or (no argument) everything start()/start_motor() started."""
    targets = [proc] if proc else list(reversed(_procs))
    for p in targets:
        if p.poll() is None:
            p.send_signal(signal.SIGTERM)
            try:
                p.wait(2)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        if p in _procs:
            _procs.remove(p)


def ctl(msg: dict, ctl_port: int = 5591, timeout: float = 1.0) -> dict:
    """Send one control datagram and return the car's reply."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(msg).encode(), ("127.0.0.1", ctl_port))
        data, _ = s.recvfrom(65536)
        return json.loads(data)
    finally:
        s.close()


def main() -> int:
    if len(sys.argv) >= 3 and sys.argv[1] == "ctl":
        port = int(sys.argv[3]) if len(sys.argv) > 3 else 5591
        print(json.dumps(ctl(json.loads(sys.argv[2]), port)))
        return 0
    ap = argparse.ArgumentParser(description="virtual SM11 (real tars.ino + physics on a pty)")
    ap.add_argument("--profile", default="sm11")
    ap.add_argument("--truth", type=int, default=5590)
    ap.add_argument("--ctl", type=int, default=5591)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--link", default=None)
    ap.add_argument("--motor", type=int, default=0, help="also start host tars_motor on this port")
    a = ap.parse_args()
    proc, dev = start(a.profile, a.truth, a.ctl, a.seed, a.link)
    print(f"virtual SM11 on {dev} (truth -> udp {a.truth}, control <- udp {a.ctl})", flush=True)
    if a.motor:
        start_motor(dev, a.motor)
        print(f"tars_motor -b uno -d {dev} on tcp {a.motor}", flush=True)
    else:
        print(f"now: iter1-wasd/pi/build/tars_motor_host -b uno -d {dev}", flush=True)
    try:
        proc.wait()
    except KeyboardInterrupt:
        pass
    finally:
        stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
