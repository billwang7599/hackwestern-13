"""Expose the Uno R3 CDC data interface as the motor service's serial port."""
import ctypes as C
import os
import pty
import select
import time
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
open_uno = bind('open_device_with_vid_pid', P, P, C.c_uint16, C.c_uint16)
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
    control(handle,0x21,0x22,0,0,None,0,500)
    time.sleep(.1)
    control(handle,0x21,0x22,3,0,None,0,500)
    master, slave = pty.openpty()
    target = os.ttyname(slave)
    try:
        tty.setraw(slave)
        os.chmod(target,0o660)
        os.chown(target,int(os.environ['SUDO_UID']),int(os.environ['SUDO_GID']))
        if os.path.islink(path): os.unlink(path)
        os.symlink(target,path)
        print('Uno connected via '+target,flush=True)
        buf=C.create_string_buffer(128)
        count=C.c_int()
        while True:
            if select.select([master],[],[],0)[0]:
                data=os.read(master,512)
                if bulk(handle,0x04,data,len(data),C.byref(count),50) or count.value!=len(data):
                    raise OSError('Uno write failed')
            rc=bulk(handle,0x83,buf,128,C.byref(count),10)
            if rc not in (0,-7): raise OSError('Uno read failed '+str(rc))
            if count.value: os.write(master,buf.raw[:count.value])
    finally:
        if os.path.islink(path) and os.readlink(path)==target: os.unlink(path)
        os.close(master); os.close(slave)

if __name__=='__main__':
    ctx=P()
    if init(C.byref(ctx)): raise SystemExit('USB initialization failed')
    try:
        handle=open_uno(ctx,0x2341,0x0043)
        if not handle: raise SystemExit('Uno absent')
        try: bridge(handle)
        finally: close(handle)
    finally: finish(ctx)
