"""Non-critical native inference process adapter; always pulls the latest RGB."""
import argparse
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import time
from rgb_broker import exact, MAX_PACKET

HERE = Path(__file__).resolve().parent


def result_line(proc, timeout):
    deadline, data = time.monotonic() + timeout, bytearray()
    while time.monotonic() < deadline:
        ready, _, _ = select.select([proc.stdout], [], [], max(0, deadline - time.monotonic()))
        if not ready:
            break
        part = os.read(proc.stdout.fileno(), 65536)
        if not part:
            raise ConnectionError('Native worker exited')
        data.extend(part)
        if len(data) > 100_000:
            raise ValueError('Oversized inference result')
        if data.endswith(b'\n'):
            try:
                value = json.loads(data)
            except RecursionError as error:
                raise ValueError('Nested inference result') from error
            if not isinstance(value, dict) or type(value.get('id')) is not int or not 0 <= value['id'] <= 0xffffffff:
                raise ValueError('Invalid inference result object or ID')
            return value
    raise TimeoutError('Native inference exceeded bounded deadline')


def write_job(proc, wire, timeout=2):
    deadline, offset = time.monotonic() + timeout, 0
    while offset < len(wire):
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([], [proc.stdin], [], remaining)[1]:
            raise TimeoutError('Native input stalled')
        try:
            sent = os.write(proc.stdin.fileno(), wire[offset:])
        except BlockingIOError:
            continue
        if sent <= 0:
            raise ConnectionError('Native input disconnected')
        offset += sent


def main():
    def stop(*_):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--depth', action='store_true')
    args = parser.parse_args()
    name, model, status_port = ('ai_depth', 'midas-small.onnx', 5567) if args.depth else ('ai_detector', 'yolox_tiny.onnx', 5565)
    command = [str(HERE / 'build' / name), '--model', str(HERE / 'models' / model), '--threads', '1']
    proc, previous, job = None, -1, 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as status:
        # QNX/FreeBSD defaults UDP send space to 9216 bytes. MiDaS results
        # are larger; increase only this bounded loopback socket's buffer.
        status.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 262144)
        status.setblocking(False)
        def publish(value):
            value['engine'] = 'QNX ONNX Runtime'
            value['kind'] = value.get('kind', 'object_detection')
            payload = json.dumps(value, allow_nan=False, separators=(',', ':')).encode()
            if len(payload) > 60_000:
                raise ValueError('Inference status exceeds UDP limit')
            try:
                status.sendto(payload, ('127.0.0.1', status_port))
            except OSError as error:
                print(f'{name} status send failed ({len(payload)} bytes): {error}', flush=True)
        try:
            while True:
                try:
                    with socket.create_connection(('127.0.0.1', 5566), timeout=1) as broker:
                        broker.settimeout(1)
                        broker.sendall(b'GET\n')
                        size, = struct.unpack('!I', exact(broker, 4))
                        if size == 0:
                            time.sleep(0.1)
                            continue
                        if not 16 <= size <= MAX_PACKET:
                            raise ValueError('Invalid broker frame length')
                        packet = exact(broker, size)
                    if packet[:4] != b'RGB1':
                        raise ValueError('Invalid broker frame')
                    frame_id, = struct.unpack('!Q', packet[4:12])
                    if frame_id == previous:
                        time.sleep(0.05)
                        continue
                    previous = frame_id
                    if proc is None or proc.poll() is not None:
                        proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
                        os.set_blocking(proc.stdin.fileno(), False)
                    job = (job + 1) & 0xffffffff
                    jpeg = packet[12:]
                    started = time.monotonic()
                    wire = struct.pack('!II', job, len(jpeg)) + jpeg
                    # Large pipe writes can be partial; never silently truncate a job.
                    write_job(proc, wire)
                    value = result_line(proc, 8 if args.depth else 4)
                    if value.get('id') != job:
                        raise ValueError('Native inference ID mismatch')
                    value.update(frame_id=str(frame_id), live='error' not in value,
                                 round_trip_ms=round((time.monotonic() - started) * 1000, 2))
                    publish(value)
                    # Low-rate, low-priority depth cannot consume all spare CPU.
                    if args.depth:
                        time.sleep(0.5)
                except (OSError, ValueError, TimeoutError) as error:
                    publish({'live': False, 'error': str(error)[:200]})
                    if proc is not None:
                        proc.kill()
                        proc.wait()
                        proc = None
                    time.sleep(1)
        finally:
            if proc is not None:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()


if __name__ == '__main__':
    main()
