"""One-time bridge for a running pre-viewer dashboard with an unsaved scan.

Reads only the existing outbound 127.0.0.1:5559 phone-status stream on QNX.
Exits with the old dashboard. Subsequent launches serve Room View directly.
"""
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from room_view import serve_room


def read_packets(stream):
    header = stream.read(24)
    orders = {b'\xd4\xc3\xb2\xa1': '<', b'\xa1\xb2\xc3\xd4': '>',
              b'\x4d\x3c\xb2\xa1': '<', b'\xa1\xb2\x3c\x4d': '>'}
    if header[:4] not in orders or len(header) != 24:
        raise ValueError('Missing pcap header')
    order = orders[header[:4]]
    linktype = struct.unpack_from(order+'I', header, 20)[0] & 0xffff
    if linktype not in (0, 1, 108):
        raise ValueError(f'Unsupported loopback datalink {linktype}')
    offset = 14 if linktype == 1 else 4
    while (record := stream.read(16)):
        if len(record) != 16:
            return
        length = struct.unpack_from(order+'I', record, 8)[0]
        if length > 262144:
            raise ValueError('Bad capture length')
        packet = stream.read(length)
        ip = packet[offset:]
        if len(ip) < 40 or ip[0] >> 4 != 4 or ip[9] != 6:
            continue
        tcp = ip[(ip[0] & 15)*4:struct.unpack_from('!H', ip, 2)[0]]
        if len(tcp) >= 20:
            port, dest, seq = struct.unpack_from('!HHI', tcp)
            data = tcp[(tcp[12] >> 4)*4:]
            if dest == 5559 and data:
                yield port, seq, data


def mirror(stream, link):
    pending = bytearray()
    source = expected = None
    synced = False
    for port, seq, data in read_packets(stream):
        if source == port and expected is not None:
            delta = ((seq-expected+2**31) % 2**32)-2**31
            if delta < 0:
                data = data[-delta:]
                if not data:
                    continue
                seq = expected
        if port != source or seq != expected:
            pending.clear()
            synced = False
        source, expected = port, (seq+len(data)) % 2**32
        pending.extend(data)
        while (end := pending.find(b'\n')) >= 0:
            line = bytes(pending[:end])
            del pending[:end+1]
            if synced or line.startswith(b'{'):
                try:
                    value = json.loads(line)
                    if value.get('type') == 'status' and isinstance(value.get('surroundings'), dict):
                        with link.lock:
                            link.latest = value
                            link.room_published = time.monotonic()
                except (AttributeError, ValueError, UnicodeError):
                    pass
            synced = True
        if len(pending) > 524288:
            pending.clear()
            synced = False


if __name__ == '__main__':
    pid = int(sys.argv[1])
    stop = threading.Event()
    link = SimpleNamespace(latest={}, room_published=0, lock=threading.Lock())
    capture = subprocess.Popen(['sudo','-n','/system/bin/tcpdump','-n','-U','-s','0','-i','lo0',
                                '-w','-', 'ip and tcp dst port 5559 and src host 127.0.0.1 and dst host 127.0.0.1'],
                               stdout=subprocess.PIPE)
    def monitor():
        while not stop.wait(1):
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                capture.terminate()
                return
    threading.Thread(target=monitor, daemon=True).start()
    threading.Thread(target=serve_room, args=(link,stop), daemon=True).start()
    try:
        mirror(capture.stdout, link)
    finally:
        stop.set()
        capture.terminate()
        capture.wait()
