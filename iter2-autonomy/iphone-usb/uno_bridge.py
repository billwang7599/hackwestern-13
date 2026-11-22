"""Expose the Uno R3 CDC data interface as the motor service's serial port."""
import ctypes as C
import os
import pty
import select
import signal
import time
import termios
import tty
from pathlib import Path

lib = C.CDLL(str(Path(__file__).resolve().parent / 'install/lib/libusb-1.0.so'))
P = C.c_void_p
def bind(name, result, *args):
    fn = getattr(lib, 'libusb_' + name)
    fn.restype, fn.argtypes = result, args
    return fn

init = bind('init', C.c_int, C.POINTER(P))
finish = bind('exit', None, P)
devices = bind('get_device_list', C.c_ssize_t, P, C.POINTER(C.POINTER(P)))
free_devices = bind('free_device_list', None, C.POINTER(P), C.c_int)
descriptor = bind('get_device_descriptor', C.c_int, P, P)
open_device = bind('open', C.c_int, P, C.POINTER(P))
close = bind('close', None, P)
claim = bind('claim_interface', C.c_int, P, C.c_int)
control = bind('control_transfer', C.c_int, P,C.c_uint8,C.c_uint8,C.c_uint16,C.c_uint16,P,C.c_uint16,C.c_uint)
bulk = bind('bulk_transfer', C.c_int,P,C.c_uint8,P,C.c_int,C.POINTER(C.c_int),C.c_uint)
path = str(Path(__file__).resolve().parent.parent / 'dashboard/state/uno-serial')

def bridge(handle):
    if claim(handle,0) or claim(handle,1):
        raise OSError('Uno CDC interface busy')
    coding = (115200).to_bytes(4,'little') + bytes([0,0,8])
    if control(handle,0x21,0x20,0,0,coding,7,500)!=7:
        raise OSError('Uno line coding failed')
    for state in (0, 3):
        rc = control(handle, 0x21, 0x22, state, 0, None, 0, 500)
        if rc != 0:
            raise OSError(f'Uno CDC control-line change failed ({rc})')
        if state == 0:
            time.sleep(.1)
    # DTR resets even an externally powered Uno. Keep application traffic out
    # of the bootloader until the sketch has had time to start.
    time.sleep(2)
    master, slave = pty.openpty()
    target = os.ttyname(slave)
    try:
        tty.setraw(slave, termios.TCSANOW)
        # QNX can recycle a PTY with bytes left in its output queue.
        termios.tcflush(slave, termios.TCIOFLUSH)
        os.set_blocking(master, False)
        os.chmod(target,0o660)
        os.chown(target,int(os.environ['SUDO_UID']),int(os.environ['SUDO_GID']))
        if os.path.islink(path): os.unlink(path)
        os.symlink(target,path)
        print('Uno connected via '+target,flush=True)
        buf=C.create_string_buffer(128)
        count=C.c_int()
        tx = b""
        rx = b""
        stalled = None
        awaiting_reply = None
        tx_bytes = rx_bytes = 0
        while True:
            if not tx and select.select([master], [], [], 0)[0]:
                tx = os.read(master, 64)
            if tx:
                rc = bulk(handle, 0x04, tx, len(tx), C.byref(count), 50)
                if rc not in (0, -7):
                    raise OSError(f'Uno write failed ({rc})')
                if b'?' in tx[:count.value] and awaiting_reply is None:
                    awaiting_reply = time.monotonic()
                tx_bytes += count.value
                tx = tx[count.value:]
                if count.value:
                    stalled = None
                elif stalled is None:
                    stalled = time.monotonic()
                elif time.monotonic() - stalled >= 1:
                    raise OSError('Uno write stalled')
            if not rx:
                rc = bulk(handle, 0x83, buf, 128, C.byref(count), 10)
                if rc not in (0, -7):
                    raise OSError(f'Uno read failed ({rc})')
                rx = buf.raw[:count.value]
                if rx:
                    awaiting_reply = None
                    rx_bytes += len(rx)
            if awaiting_reply is not None and time.monotonic() - awaiting_reply > 6:
                raise OSError(f'Uno USB reply timeout (sent {tx_bytes}, received {rx_bytes} bytes)')
            if rx:
                try:
                    rx = rx[os.write(master, rx):]
                except BlockingIOError:
                    time.sleep(.005)
    finally:
        if os.path.islink(path) and os.readlink(path)==target: os.unlink(path)
        os.close(master); os.close(slave)

def connect_once(session=bridge):
    ctx = P()
    rc = init(C.byref(ctx))
    if rc:
        raise OSError(f'USB initialization failed ({rc})')
    try:
        entries = C.POINTER(P)()
        count = devices(ctx, C.byref(entries))
        if count < 0:
            raise OSError(f'USB enumeration failed ({count})')
        try:
            for i in range(count):
                raw = C.create_string_buffer(18)
                rc = descriptor(entries[i], raw)
                if rc:
                    raise OSError(f'USB descriptor read failed ({rc})')
                vendor = int.from_bytes(raw.raw[8:10], 'little')
                product = int.from_bytes(raw.raw[10:12], 'little')
                if vendor not in (0x2341, 0x2a03) or product != 0x0043:
                    continue
                handle = P()
                rc = open_device(entries[i], C.byref(handle))
                if rc:
                    raise OSError(f'Uno {vendor:04x}:{product:04x} enumerated, open failed ({rc})')
                try:
                    session(handle)
                finally:
                    close(handle)
                return
            raise OSError(f'Waiting for Uno R3: QNX USB scan found {count} device(s)')
        finally:
            free_devices(entries, 1)
    finally:
        finish(ctx)


def stop(signum, frame):
    raise SystemExit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, stop)
    if os.path.islink(path):
        os.unlink(path)
    previous = None
    while True:
        try:
            connect_once()
        except OSError as error:
            message = str(error)
            if message != previous:
                print(message, flush=True)
                previous = message
        time.sleep(2)
