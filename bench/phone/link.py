#!/usr/bin/env python3
"""Phone-in-the-loop link (bench only): the phone's LiDAR and camera streams onto this Mac's localhost.

    python3 bench/phone/link.py --source usb --ldr 5562:5562 --rgb 5563:5563 --nav 5572
    python3 bench/phone/link.py --source tcp:127.0.0.1 --ldr L:D --rgb L:D --nav N    (fake phone, tests)

What `iproxy -l -s 127.0.0.1 P:P` does on the Pi (service.py), plus one tee:

  phone 5562 (LDR1 safety: u32 BE length + LDR1, client ACKs 0x01) -> 127.0.0.1:<ldr local> -> lidar_guard
      every phone->guard frame is also copied to goto's nav port over TCP (u32 BE length + LDR1)
  phone 5563 (RGB1 JPEG frames)                                     -> 127.0.0.1:<rgb local> -> rgb_broker

Why the tee: on the Pi, lidar_guard --nav-port tees LDR1 to goto over UDP. A full phone packet is
44 + 1024 x 12 = 12332 bytes, over macOS's default UDP datagram cap (net.inet.udp.maxdgram 9216), so on a
Mac the biggest (most useful) frames would vanish. goto also takes LDR1 on TCP, so the bench sends it
there. Bytes to lidar_guard are passed through untouched, ACKs included; the tee never blocks them
(a 2-deep queue, oldest dropped). goto validates every packet itself (nav.c), as it does on the robot.

Each local port takes one client at a time (the phone serves one client per stream).
Stats go to stdout every 5 s (the run_sim log) and to --stats-file as JSON if given.
"""
from __future__ import annotations

import argparse
import json
import os
import queue
import socket
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import usbmux  # noqa: E402

MAX_FRAME = 2_000_012


class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.v = {"ldr_frames": 0, "ldr_bytes_max": 0, "ldr_last": 0.0, "rgb_bytes": 0, "rgb_last": 0.0,
                  "nav_sent": 0, "nav_dropped": 0, "nav_connected": False, "phone": "not connected yet",
                  "ldr_client": False, "rgb_client": False}

    def add(self, **kw):
        with self.lock:
            for k, val in kw.items():
                if isinstance(val, (int, float)) and not isinstance(val, bool) and k.endswith(("_frames", "_bytes", "_sent", "_dropped")):
                    self.v[k] += val
                else:
                    self.v[k] = val

    def snap(self) -> dict:
        with self.lock:
            return dict(self.v)


class NavTee:
    """LDR1 frames -> goto's nav port, TCP, length-prefixed. Never blocks the caller."""

    def __init__(self, port: int, stats: Stats):
        self.port, self.stats = port, stats
        self.q: queue.Queue = queue.Queue(maxsize=2)
        threading.Thread(target=self._run, daemon=True, name="nav-tee").start()

    def put(self, frame: bytes):
        while True:
            try:
                self.q.put_nowait(frame)
                return
            except queue.Full:
                try:
                    self.q.get_nowait()
                    self.stats.add(nav_dropped=1)
                except queue.Empty:
                    pass

    def _run(self):
        s = None
        while True:
            frame = self.q.get()
            for _ in range(2):
                try:
                    if s is None:
                        s = socket.create_connection(("127.0.0.1", self.port), timeout=1)
                        s.settimeout(1)
                        self.stats.add(nav_connected=True)
                    s.sendall(frame)
                    self.stats.add(nav_sent=1)
                    break
                except OSError:
                    if s is not None:
                        s.close()
                    s = None
                    self.stats.add(nav_connected=False)
            else:
                self.stats.add(nav_dropped=1)
                time.sleep(0.2)


def make_source(spec: str, serial: str | None):
    if spec == "usb":
        return lambda port: usbmux.connect_phone(port, serial=serial)
    if spec.startswith("tcp:"):
        host = spec[4:]

        def tcp(port):
            s = socket.create_connection((host, port), timeout=2)
            s.settimeout(None)
            return s
        return tcp
    raise SystemExit(f"--source usb | tcp:HOST, got {spec!r}")


def _exact(s: socket.socket, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        b = s.recv(n - len(out))
        if not b:
            raise ConnectionError("closed")
        out += b
    return bytes(out)


class Link:
    """127.0.0.1:local <-> phone:device. framed=True: phone->client is u32-length frames, each also given to tap."""

    def __init__(self, name, local, device, connect, stats: Stats, tap=None):
        self.name, self.device, self.connect, self.stats, self.tap = name, device, connect, stats, tap
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", local))
        self.srv.listen(1)
        self.last_err = ""
        threading.Thread(target=self._accept, daemon=True, name=f"link-{name}").start()

    def _note(self, msg: str):
        if msg != self.last_err:          # a refused phone port retries ~10/s: say it once per change
            self.last_err = msg
            print(f"link {self.name}: {msg}", flush=True)
            self.stats.add(phone=msg)

    def _accept(self):
        while True:
            client, _ = self.srv.accept()
            try:
                phone = self.connect(self.device)
            except (OSError, usbmux.UsbmuxError) as e:
                self._note(f"phone port {self.device}: {e}")
                client.close()
                time.sleep(0.3)
                continue
            self._note(f"phone port {self.device} connected")
            self.stats.add(**{f"{self.name}_client": True})
            up = threading.Thread(target=self._up, args=(client, phone), daemon=True)
            up.start()
            try:
                self._down(phone, client)
            except (OSError, ConnectionError, ValueError) as e:
                self._note(f"phone port {self.device} closed ({e})")
            finally:
                for s in (phone, client):
                    try:
                        s.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
                    s.close()
                self.stats.add(**{f"{self.name}_client": False})

    def _up(self, client, phone):
        """client -> phone (ACKs): byte copy"""
        try:
            while True:
                b = client.recv(4096)
                if not b:
                    break
                phone.sendall(b)
        except OSError:
            pass
        try:
            phone.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    def _down(self, phone, client):
        """phone -> client: whole frames (u32 BE length + body), forwarded unchanged"""
        while True:
            head = _exact(phone, 4)
            n, = struct.unpack("!I", head)
            if n > MAX_FRAME:
                raise ValueError(f"frame {n} bytes")
            frame = head + _exact(phone, n)
            client.sendall(frame)
            now = time.time()
            if self.name == "ldr":
                self.stats.add(ldr_frames=1, ldr_last=now)
                with self.stats.lock:
                    self.stats.v["ldr_bytes_max"] = max(self.stats.v["ldr_bytes_max"], len(frame))
                if self.tap and frame[4:8] == b"LDR1":
                    self.tap(frame)
            else:
                self.stats.add(rgb_bytes=len(frame), rgb_last=now)
                with self.stats.lock:
                    self.stats.v["rgb_frames"] = self.stats.v.get("rgb_frames", 0) + 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="phone streams -> localhost (+ LDR1 tee to goto over TCP)")
    ap.add_argument("--source", default="usb", help="usb (this Mac's usbmuxd) | tcp:HOST (a fake phone)")
    ap.add_argument("--serial", default=None, help="usb: pick this phone (default: the first one)")
    ap.add_argument("--ldr", required=True, metavar="LOCAL:DEVICE", help="LDR1 safety stream (phone 5562)")
    ap.add_argument("--rgb", default="", metavar="LOCAL:DEVICE", help="RGB1 camera stream (phone 5563)")
    ap.add_argument("--nav", type=int, default=0, help="goto nav port (TCP): copy each LDR1 frame there")
    ap.add_argument("--stats-file", default="")
    a = ap.parse_args(argv)
    connect = make_source(a.source, a.serial)
    stats = Stats()
    if a.source == "usb":
        try:
            devs = usbmux.list_devices()
            print(f"link: usbmuxd sees {len(devs)} device(s): " +
                  ", ".join(f"{d.get('serial')} ({d.get('connection')})" for d in devs), flush=True)
        except usbmux.UsbmuxError as e:
            print(f"link: {e}", flush=True)
    tee = NavTee(a.nav, stats) if a.nav else None
    lp, dp = (int(x) for x in a.ldr.split(":"))
    Link("ldr", lp, dp, connect, stats, tap=tee.put if tee else None)
    if a.rgb:
        lp, dp = (int(x) for x in a.rgb.split(":"))
        Link("rgb", lp, dp, connect, stats)
    print(f"link: up ({a.source}); ldr {a.ldr}, rgb {a.rgb or '-'}, nav tee TCP {a.nav or '-'}", flush=True)
    t_print = 0.0
    while True:
        time.sleep(1)
        st = stats.snap()
        if a.stats_file:
            try:
                with open(a.stats_file + ".tmp", "w") as f:
                    json.dump(st, f)
                os.replace(a.stats_file + ".tmp", a.stats_file)
            except OSError:
                pass
        if time.monotonic() - t_print >= 5:
            t_print = time.monotonic()
            now = time.time()
            age = lambda t: f"{now - t:.1f}s ago" if t else "never"  # noqa: E731
            print(f"link: LDR1 {st['ldr_frames']} frames (last {age(st['ldr_last'])}, max {st['ldr_bytes_max']} B), "
                  f"nav tee sent {st['nav_sent']} dropped {st['nav_dropped']}, RGB {st.get('rgb_frames', 0)} frames "
                  f"(last {age(st['rgb_last'])}); {st['phone']}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
