"""Minimal stdlib usbmuxd client: reach a TCP port on a USB-attached iPhone from this Mac.

macOS runs usbmuxd itself (unix socket /var/run/usbmuxd), so no iproxy/libimobiledevice
install is needed. This speaks the plist protocol: ListDevices, then Connect(DeviceID,
port); on Result 0 the same unix socket becomes a raw byte tunnel to the phone's port.

`Forwarder` listens on 127.0.0.1:<local> and tunnels each accepted connection to the
phone's <device_port>, exactly what `iproxy -l -s 127.0.0.1 local:device` does on the Pi.
So Andrew's receivers (mapping.SceneFeed on 5561, motion on 5557) run unchanged here.

Equivalent with the brew tool, if installed: `iproxy -l -s 127.0.0.1 5595:5561`.
"""
from __future__ import annotations

import plistlib
import socket
import struct
import threading

SOCKET_PATH = "/var/run/usbmuxd"
HEADER = struct.Struct("<IIII")  # total length, version 1, message 8 (plist), tag
RESULTS = {0: "ok", 1: "bad command", 2: "bad device (unplugged?)", 3: "connection refused "
           "(is MotionUSB open with Start pressed?)", 6: "bad version"}


class UsbmuxError(Exception):
    pass


class NoDevice(UsbmuxError):
    pass


def _open(path: str) -> socket.socket:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(3)
    try:
        s.connect(path)
    except OSError as e:
        s.close()
        raise UsbmuxError(f"usbmuxd not reachable at {path}: {e}") from e
    return s


def _exact(s: socket.socket, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        chunk = s.recv(n - len(out))
        if not chunk:
            raise UsbmuxError("usbmuxd closed the connection")
        out += chunk
    return bytes(out)


def _request(s: socket.socket, msg: dict, tag: int = 1) -> dict:
    msg = {"ClientVersionString": "tars-bench", "ProgName": "tars-bench",
           "kLibUSBMuxVersion": 3, **msg}
    body = plistlib.dumps(msg)
    s.sendall(HEADER.pack(HEADER.size + len(body), 1, 8, tag) + body)
    length, _version, _kind, _tag = HEADER.unpack(_exact(s, HEADER.size))
    if not HEADER.size <= length <= 1 << 22:
        raise UsbmuxError(f"bad usbmuxd reply length {length}")
    try:
        return plistlib.loads(_exact(s, length - HEADER.size))
    except Exception as e:  # plistlib raises several types
        raise UsbmuxError(f"bad usbmuxd reply: {e}") from e


def list_devices(path: str = SOCKET_PATH) -> list[dict]:
    """[{id, serial, connection, product_id}], USB devices first. [] if none."""
    with _open(path) as s:
        reply = _request(s, {"MessageType": "ListDevices"})
    out = []
    for d in reply.get("DeviceList", []) or []:
        p = d.get("Properties", {}) or {}
        out.append({"id": d.get("DeviceID", p.get("DeviceID")), "serial": p.get("SerialNumber", ""),
                    "connection": p.get("ConnectionType", "?"), "product_id": p.get("ProductID")})
    out.sort(key=lambda d: d["connection"] != "USB")
    return out


def pick_device(path: str = SOCKET_PATH, serial: str | None = None) -> dict:
    devs = [d for d in list_devices(path) if serial in (None, "", d["serial"])]
    usb = [d for d in devs if d["connection"] == "USB"]
    if not usb:
        raise NoDevice("no iPhone on USB" + (f" with serial {serial}" if serial else "")
                       + (" (only Wi-Fi pairings seen)" if devs else "")
                       + ": plug it in, unlock it, tap Trust")
    return usb[0]


def connect(device_id: int, port: int, path: str = SOCKET_PATH) -> socket.socket:
    """A socket tunnelled to <port> on the phone. Raises UsbmuxError with a readable reason."""
    s = _open(path)
    try:
        swapped = ((port & 0xFF) << 8) | (port >> 8)  # usbmuxd wants network byte order
        reply = _request(s, {"MessageType": "Connect", "DeviceID": device_id, "PortNumber": swapped})
        n = reply.get("Number", -1)
        if n != 0:
            raise UsbmuxError(f"phone port {port}: {RESULTS.get(n, f'result {n}')}")
        s.settimeout(None)
        return s
    except BaseException:
        s.close()
        raise


def connect_phone(port: int, path: str = SOCKET_PATH, serial: str | None = None) -> socket.socket:
    return connect(pick_device(path, serial)["id"], port, path)


class Forwarder:
    """127.0.0.1:<local> -> phone:<device_port>. Each accepted client gets its own tunnel.
    state: 'listening' | 'connected' | 'error: ...'. Stop with close()."""

    def __init__(self, local_port: int, device_port: int, path: str = SOCKET_PATH,
                 serial: str | None = None):
        self.device_port, self.path, self.serial = device_port, path, serial
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", local_port))
        self.srv.listen(2)
        self.port = self.srv.getsockname()[1]
        self.state = "listening"
        self.bytes_in = 0
        self.closed = threading.Event()
        self._socks: list[socket.socket] = []
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while not self.closed.is_set():
            try:
                client, _ = self.srv.accept()
            except OSError:
                return
            try:
                phone = connect_phone(self.device_port, self.path, self.serial)
            except UsbmuxError as e:
                self.state = f"error: {e}"
                client.close()
                continue
            self.state = "connected"
            self._socks += [client, phone]
            threading.Thread(target=self._pipe, args=(phone, client, True), daemon=True).start()
            threading.Thread(target=self._pipe, args=(client, phone, False), daemon=True).start()

    def _pipe(self, src, dst, inbound):
        try:
            while True:
                data = src.recv(65536)
                if not data:
                    break
                if inbound:
                    self.bytes_in += len(data)
                dst.sendall(data)
        except OSError:
            pass
        finally:
            for s in (src, dst):
                try:
                    s.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                s.close()
            if inbound and not self.closed.is_set():
                self.state = "listening (phone side closed)"

    def close(self):
        self.closed.set()
        self.srv.close()
        for s in self._socks:
            try:
                s.close()
            except OSError:
                pass


if __name__ == "__main__":
    try:
        devs = list_devices()
    except UsbmuxError as e:
        raise SystemExit(str(e))
    print(devs or "no devices (plug in the iPhone, unlock, Trust)")
