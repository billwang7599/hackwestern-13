"""Bounded latest RGB frame broker. Neither AI consumer can block LiDAR."""
import argparse
import socket
import struct
import threading
import time

MAX_PACKET = 2_000_012


def exact(sock, length):
    data = bytearray()
    while len(data) < length:
        part = sock.recv(length - len(data))
        if not part:
            raise ConnectionError('RGB disconnected')
        data.extend(part)
    return bytes(data)


def read_packet(sock):
    size, = struct.unpack('!I', exact(sock, 4))
    if not 16 <= size <= MAX_PACKET:
        raise ValueError('Invalid RGB size')
    packet = exact(sock, size)
    if packet[:4] != b'RGB1' or packet[12:14] != b'\xff\xd8' or packet[-2:] != b'\xff\xd9':
        raise ValueError('Invalid RGB envelope')
    return packet


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--phone-port', type=int, default=5563, help='phone RGB stream on 127.0.0.1 (default 5563)')
    ap.add_argument('--port', type=int, default=5566, help='serve GET on 127.0.0.1 (default 5566)')
    args = ap.parse_args(argv)
    lock = threading.Lock()
    latest = [None, 0.0]
    slots = threading.BoundedSemaphore(4)

    def capture():
        while True:
            try:
                with socket.create_connection(('127.0.0.1', args.phone_port), timeout=2) as phone:
                    phone.settimeout(3)
                    previous = -1
                    while True:
                        packet = read_packet(phone)
                        frame_id, = struct.unpack('!Q', packet[4:12])
                        if frame_id <= previous:
                            raise ValueError('Non-increasing RGB frame')
                        previous = frame_id
                        with lock:
                            latest[:] = [packet, time.monotonic()]
                        phone.sendall(b'\x01')
            except (OSError, ValueError) as error:
                with lock:
                    latest[:] = [None, 0.0]
                print('RGB broker: ' + str(error), flush=True)
                time.sleep(1)

    def reply(client):
        try:
            with client:
                client.settimeout(1)
                if exact(client, 4) != b'GET\n':
                    return
                with lock:
                    packet, received = latest
                if packet is None or time.monotonic() - received > 1.5:
                    client.sendall(struct.pack('!I', 0))
                else:
                    client.sendall(struct.pack('!I', len(packet)) + packet)
        except OSError:
            pass
        finally:
            slots.release()

    threading.Thread(target=capture, daemon=True).start()
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', args.port))
        server.listen(4)
        while True:
            client, _ = server.accept()
            if slots.acquire(blocking=False):
                threading.Thread(target=reply, args=(client,), daemon=True).start()
            else:
                client.close()


if __name__ == '__main__':
    main()
