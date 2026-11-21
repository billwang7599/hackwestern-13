"""Read MotionUSB JSON lines through iproxy. No motor control."""
import argparse
import json
import math
import socket
import time


def validate(sample):
    if sample.get('version') != 1 or type(sample.get('seq')) is not int:
        raise ValueError('Unsupported sample format')
    for field in ('accel_time_s', 'gyro_time_s'):
        if not isinstance(sample.get(field), (int, float)) or not math.isfinite(sample[field]):
            raise ValueError('Invalid sensor timestamp')
    for field in ('accel_m_s2', 'gyro_rad_s'):
        values = sample.get(field)
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError('Invalid vector')
        if not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values):
            raise ValueError('Non-finite sensor value')
    return sample


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=5557)
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    with socket.create_connection(('127.0.0.1', args.port), timeout=3) as sock:
        sock.settimeout(1)
        start = last_print = time.monotonic()
        previous = None
        count = gaps = 0
        buffer = b''
        print('Connected to local USB proxy; waiting for phone samples', flush=True)
        while time.monotonic() - start < args.seconds:
            data = sock.recv(4096)
            if not data:
                raise RuntimeError('Phone stream disconnected')
            buffer += data
            if len(buffer) > 65536:
                raise ValueError('Oversized stream buffer')
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                sample = validate(json.loads(line))
                count += 1
                if previous is not None:
                    if sample['seq'] <= previous:
                        raise ValueError('Non-increasing sequence')
                    gaps += sample['seq'] - previous - 1
                previous = sample['seq']
                now = time.monotonic()
                if count == 1 or now - last_print >= 0.25:
                    print(f"samples={count} gaps={gaps} accel={sample['accel_m_s2']} gyro={sample['gyro_rad_s']}", flush=True)
                    last_print = now
        print(f'Received {count} samples in {time.monotonic()-start:.2f}s; skipped sequence numbers={gaps}')
        print('Receipt rate is not an end-to-end latency measurement.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        raise SystemExit(f'STREAM STOPPED: {error}')
