"""Wi-Fi stand-in for iproxy: 127.0.0.1:<port> on the Pi -> <phone>:<port>.

Usage: wifi_proxy.py <phone-host> <port> [port ...]

Every local consumer keeps connecting to 127.0.0.1 exactly as it does through
iproxy; only the transport changes. A dropped phone closes the local side, so
the consumers see the same disconnect they see when USB is pulled.
"""
import socket
import sys
import threading


def pipe(src, dst):
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except OSError:
        pass
    finally:
        for sock in (src, dst):
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


def handle(local, host, port):
    try:
        remote = socket.create_connection((host, port), timeout=2)
    except OSError:
        local.close()
        return
    remote.settimeout(None)
    for sock in (local, remote):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    threading.Thread(target=pipe, args=(remote, local), daemon=True).start()
    pipe(local, remote)
    local.close()
    remote.close()


def serve(host, port):
    listener = socket.socket()
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(('127.0.0.1', port))
    listener.listen(8)
    while True:
        local, _ = listener.accept()
        threading.Thread(target=handle, args=(local, host, port), daemon=True).start()


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    host, ports = sys.argv[1], [int(p) for p in sys.argv[2:]]
    for port in ports[1:]:
        threading.Thread(target=serve, args=(host, port), daemon=True).start()
    print(f'Forwarding 127.0.0.1:{",".join(map(str, ports))} -> {host} over Wi-Fi', flush=True)
    serve(host, ports[0])


if __name__ == '__main__':
    main()
