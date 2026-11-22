"""Probe an idle Uno with exclusive USB access. Reads status, never programs flash."""
import ctypes as C
import time

import uno_bridge as usb


def probe(handle):
    for interface in (0, 1):
        if usb.claim(handle, interface):
            raise RuntimeError('Uno USB interface busy')
    coding = (115200).to_bytes(4, 'little') + bytes([0, 0, 8])
    if usb.control(handle, 0x21, 0x20, 0, 0, coding, 7, 500) != 7:
        raise RuntimeError('SET_LINE_CODING failed')
    current = C.create_string_buffer(7)
    if usb.control(handle, 0xa1, 0x21, 0, 0, current, 7, 500) != 7 or current.raw != coding:
        raise RuntimeError(f'Unexpected CDC line coding: {current.raw.hex()}')
    print('USB converter confirmed 115200 8N1', flush=True)
    for state in (0, 3):
        if usb.control(handle, 0x21, 0x22, state, 0, None, 0, 500):
            raise RuntimeError('DTR failed')
        time.sleep(.10)
    buf, count = C.create_string_buffer(64), C.c_int()

    def exchange(query, seconds):
        result = bytearray()
        rc = usb.bulk(handle, 0x04, query, len(query), C.byref(count), 200)
        if rc or count.value != len(query):
            raise RuntimeError(f'USB OUT failed: {rc}, bytes={count.value}')
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rc = usb.bulk(handle, 0x83, buf, len(buf), C.byref(count), 15)
            if rc not in (0, -7):
                raise RuntimeError(f'USB IN failed: {rc}')
            result.extend(buf.raw[:count.value])
        return bytes(result)

    sync = b''
    for _ in range(4):
        sync += exchange(b'0 ', .12)  # STK_GET_SYNC, CRC_EOP
        if b'\x14\x10' in sync:
            break
    print('Optiboot sync:', b'\x14\x10' in sync, 'bytes received:', len(sync), flush=True)
    time.sleep(2)  # Allow the bootloader watchdog to start the sketch.
    reply = exchange(b'\n?\n', 1)
    print('Sketch status:', b'# arms ' in reply, 'bytes received:', len(reply), flush=True)


if __name__ == '__main__':
    usb.connect_once(probe)
