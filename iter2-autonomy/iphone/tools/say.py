#!/usr/bin/env python3
"""Make the mounted iPhone speak: send one line to the speech link (TCP 5573).

    python3 iphone/tools/say.py "I remember 2 things."            # SAY line
    python3 iphone/tools/say.py --urgent "Stopping."              # JSON, interrupts
    python3 iphone/tools/say.py --hush                            # stop + clear queue
    python3 iphone/tools/say.py --host 127.0.0.1 --port 5573 ...  # defaults shown

On the Pi, 127.0.0.1:5573 is forwarded to the phone by iproxy (service.py).
On the Mac with the app running in the iOS Simulator, 127.0.0.1 reaches it too.
Prints the app's reply (OK say / OK hush / ERR ...) and exits 0 only on OK.
"""
import argparse
import json
import socket
import sys


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('text', nargs='*', help='what to say')
    ap.add_argument('--urgent', action='store_true', help='interrupt current speech (JSON form)')
    ap.add_argument('--json', action='store_true', help='send the JSON form even for normal priority')
    ap.add_argument('--hush', action='store_true', help='stop speaking and clear the queue')
    ap.add_argument('--raw', help='send this exact line (for testing garbage)')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5573)
    ap.add_argument('--timeout', type=float, default=3.0)
    a = ap.parse_args()

    text = ' '.join(a.text)
    if a.raw is not None:
        line = a.raw
    elif a.hush:
        line = 'HUSH'
    elif not text:
        ap.error('nothing to say')
    elif a.urgent or a.json:
        line = json.dumps({'type': 'say', 'text': text, 'priority': 'urgent' if a.urgent else 'normal'})
    else:
        line = 'SAY ' + ' '.join(text.split())

    with socket.create_connection((a.host, a.port), timeout=a.timeout) as sock:
        sock.sendall(line.encode() + b'\n')
        reply = b''
        while not reply.endswith(b'\n'):
            chunk = sock.recv(256)
            if not chunk:
                break
            reply += chunk
    reply = reply.decode(errors='replace').strip()
    print(reply or '(no reply)')
    return 0 if reply.startswith('OK') else 1


if __name__ == '__main__':
    sys.exit(main())
