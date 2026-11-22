#!/usr/bin/env python3
"""Pretend to be the phone's robot mic (MIC1 on TCP 5577): tars_voice (`input = phone`) dials
in and hears a WAV file or raw audio from stdin. A debugging tool for the Pi or a Mac:
it checks tars_voice's phone input without the iPhone app. Stdlib only.

    python3 iter2-autonomy/iphone/tools/mic_stream.py cmd.wav            # 16 kHz mono 16-bit WAV
    python3 iter2-autonomy/iphone/tools/mic_stream.py cmd.wav --loop 3
    ffmpeg -f avfoundation -i ":0" -ac 1 -ar 16000 -f s16le - | \\
        python3 iter2-autonomy/iphone/tools/mic_stream.py -                # Mac mic, live
    python3 iter2-autonomy/iphone/tools/mic_stream.py --port 5599 cmd.wav  # tars_voice --input phone:5599

On the Pi, stop iproxy's 5577 forward first (or use another port): this listens on 5577 itself.
Protocol: iter2-autonomy/voice/src/audio.h.
"""
import argparse
import socket
import struct
import sys
import time
import wave

HEADER = b"MIC1 16000 1 s16le\n"
CHUNK = 640                                  # 20 ms of 16 kHz s16


def pkt(kind: bytes, payload: bytes = b"") -> bytes:
    return kind + struct.pack("<H", len(payload)) + payload


def load_wav(path: str) -> bytes:
    with wave.open(path) as w:
        if (w.getframerate(), w.getnchannels(), w.getsampwidth()) != (16000, 1, 2):
            sys.exit(f"{path}: need 16 kHz mono 16-bit (afconvert -f WAVE -d LEI16@16000 -c 1 in out.wav)")
        return w.readframes(w.getnframes())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", help="WAV file, or - for raw s16le 16 kHz mono on stdin")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5577)
    ap.add_argument("--loop", type=int, default=1, help="play the WAV this many times (2 s gaps)")
    args = ap.parse_args()

    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(1)
    print(f"waiting for tars_voice on {args.host}:{args.port} ...", flush=True)
    c, peer = srv.accept()
    print(f"tars_voice connected from {peer[0]}:{peer[1]}", flush=True)
    c.sendall(HEADER + pkt(b"U"))
    silence = b"\0" * (2 * 16000 * 2)
    try:
        if args.source == "-":
            while True:
                data = sys.stdin.buffer.read(CHUNK)
                if not data:
                    break
                c.sendall(pkt(b"A", data[: len(data) & ~1]))
        else:
            audio = load_wav(args.source)
            for i in range(args.loop):
                for part in (silence[:16000], audio, silence):
                    for o in range(0, len(part), CHUNK):
                        c.sendall(pkt(b"A", part[o:o + CHUNK]))
                        time.sleep(0.02)                 # real time
                print(f"played {i + 1}/{args.loop}", flush=True)
        c.sendall(pkt(b"M"))
        time.sleep(0.5)
    except (BrokenPipeError, ConnectionResetError):
        print("tars_voice hung up", flush=True)
    c.close()
    srv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
