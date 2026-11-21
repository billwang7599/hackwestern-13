"""Original right Joy-Con -> UDP raw IMU telemetry. No motor control."""
import argparse
import socket
import struct
import time


def samples(report):
    if len(report) < 49 or report[0] != 0x30:
        return []
    return [struct.unpack_from('<6h', report, 13 + i * 12) for i in range(3)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host', default='172.23.43.185')
    p.add_argument('--port', type=int, default=5556)
    p.add_argument('--simulate', action='store_true')
    p.add_argument('--seconds', type=float, default=0, help='0 runs until Ctrl-C')
    p.add_argument('--list', action='store_true', help='List paired right Joy-Cons')
    args = p.parse_args()
    device = None
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        destination = (socket.gethostbyname(args.host), args.port)
        try:
            if not args.simulate:
                import hid
                devices = hid.enumerate(0x057e, 0x2007)
                if args.list:
                    for d in devices:
                        print(d)
                    if not devices:
                        print('No original right Joy-Con found. Pair Joy-Con (R) in Bluetooth settings.')
                    return
                if len(devices) != 1:
                    raise RuntimeError(f'Found {len(devices)} right Joy-Cons; connect exactly one.')
                device = hid.device()
                device.open_path(devices[0]['path'])
                # Enable IMU, then request full input reports. Require acknowledgements.
                for counter, (command, value) in enumerate(((0x40, 1), (0x03, 0x30))):
                    packet = bytes([1, counter]) + bytes.fromhex('0001404000014040') + bytes([command, value])
                    device.write(packet)
                    deadline = time.monotonic() + 3
                    while True:
                        reply = device.read(64, 100)
                        if len(reply) >= 15 and reply[0] == 0x21 and reply[14] == command:
                            if not reply[13] & 0x80:
                                raise RuntimeError('Joy-Con rejected IMU setup')
                            break
                        if time.monotonic() > deadline:
                            raise RuntimeError('Joy-Con setup timed out; reconnect and close other controller apps.')
            start = last = time.monotonic()
            seq = 0
            print(f'Sending {"SIMULATED" if args.simulate else "Joy-Con"} raw counts to {destination}; Ctrl-C stops.')
            while not args.seconds or time.monotonic() - start < args.seconds:
                if args.simulate:
                    batch = [(0, 0, 4096, 0, 0, 0)] * 3
                    timer = seq % 256
                    time.sleep(0.015)
                else:
                    report = device.read(64, 100)
                    batch = samples(report)
                    if not batch:
                        if time.monotonic() - last > 1:
                            raise RuntimeError('No IMU reports for 1 second; relay stopped.')
                        continue
                    timer = report[1]
                last = time.monotonic()
                stamp = time.monotonic_ns()
                for index, values in enumerate(batch):
                    line = f'IMU1 {seq} {stamp} {timer} {index} ' + ' '.join(map(str, values)) + '\n'
                    sock.sendto(line.encode('ascii'), destination)
                    seq += 1
        finally:
            if device is not None:
                device.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ImportError) as exc:
        raise SystemExit(str(exc))
