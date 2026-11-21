#!/usr/bin/env python3
"""Bounded loopback-only UDP size probe. Does not touch stack-global settings."""
import errno
import json
import socket
import time


def probe(label, send_buffer=None, receive_buffer=None):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender, \
         socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(('127.0.0.1', 0))
        receiver.settimeout(0.3)
        sender.setblocking(False)
        setup_errors = []
        for sock, option, requested in ((sender, socket.SO_SNDBUF, send_buffer),
                                        (receiver, socket.SO_RCVBUF, receive_buffer)):
            if requested is not None:
                try:
                    sock.setsockopt(socket.SOL_SOCKET, option, requested)
                except OSError as error:
                    setup_errors.append({'option': option, 'errno': error.errno, 'error': str(error)})
        for length in (8192, 40000, 60000):
            payload = bytes([length % 251]) * length
            record = {'mode': label, 'bytes': length,
                      'sndbuf': sender.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF),
                      'rcvbuf': receiver.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF),
                      'received': False}
            if setup_errors:
                record['setup_errors'] = setup_errors
            start = time.monotonic()
            try:
                record['sent'] = sender.sendto(payload, receiver.getsockname())
            except OSError as error:
                record.update(errno=error.errno, errno_name=errno.errorcode.get(error.errno), error=str(error))
            else:
                try:
                    data, _ = receiver.recvfrom(65535)
                    record.update(received=len(data) == length and data == payload, received_bytes=len(data))
                except OSError as error:
                    record.update(receive_errno=error.errno, receive_error=str(error))
            record['elapsed_ms'] = round((time.monotonic() - start) * 1000, 3)
            print(json.dumps(record, sort_keys=True), flush=True)


if __name__ == '__main__':
    probe('defaults')
    probe('sndbuf_65536', send_buffer=65536)
    probe('sndbuf_65536_rcvbuf_131072', send_buffer=65536, receive_buffer=131072)
