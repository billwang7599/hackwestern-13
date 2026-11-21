"""Native process integration: one grid on screen and in A*, plus safe recovery.
Uses a disconnected fake motor endpoint; no hardware, USB, or production ports.
"""
from pathlib import Path
import copy
import json
import socket
import struct
import subprocess
import tempfile
import time
import unittest

from native_map import NativeGrid, validate_snapshot
from mapping import Map, SceneFeed

ROOT = Path(__file__).resolve().parents[2]


def port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0)); return s.getsockname()[1]


class NativeMapTests(unittest.TestCase):
    def setUp(self):
        self.ctl, self.nav, self.det, self.motor = [port() for _ in range(4)]
        self.log = tempfile.TemporaryFile()
        self.proc = subprocess.Popen([str(ROOT/'iter2-autonomy/goto/build/goto_host'),
            '-p',str(self.ctl),'-n',str(self.nav),'-u',str(self.det),
            '-m',f'127.0.0.1:{self.motor}','-M','-'],stdout=self.log,stderr=self.log)
        self.client = NativeGrid(self.ctl)
        self.client.set_session('test-room')
        end=time.monotonic()+3
        while time.monotonic()<end:
            self.client.poll()
            if self.client.latest is not None: break
            time.sleep(.03)
        self.assertIsNotNone(self.client.latest)
        self.seq=0
        self.udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    def tearDown(self):
        self.udp.close(); self.proc.terminate()
        try: self.proc.wait(timeout=3)
        except subprocess.TimeoutExpired: self.proc.kill(); self.proc.wait()
        self.log.close()

    def packet(self, points, flags=7):
        self.seq += 1
        return b'LDR1'+struct.pack('!IQII5f', self.seq,int(time.monotonic()*1e6),flags,len(points),
                                  0, .25, 0, 0, 0)+b''.join(struct.pack('!3f',*p) for p in points)

    def send(self, points, flags=7, wire=None):
        wire = wire or self.packet(points,flags)
        self.udp.sendto(wire,('127.0.0.1',self.nav))
        time.sleep(.025)
        self.client.poll()
        return wire

    def native(self):
        with socket.create_connection(('127.0.0.1',self.ctl),timeout=2) as s:
            s.sendall(b'NAV SNAPSHOT\n'); return json.loads(s.makefile('rb').readline(65537))

    def test_recovery_and_exact_dashboard_grid(self):
        cell=100*200+100
        mapper=Map(native=self.client)
        self.send([[.05,.5,.05]])
        raw=self.native(); shown=mapper.snapshot()
        self.assertEqual(shown['grid'],raw['grid'])
        self.assertEqual(shown['grid_version'],raw['version'])
        self.assertEqual(shown['grid'][cell],'2')
        self.assertEqual(shown['grid_source'],'native-nav')
        # A local inferred grid or loaded preview cannot overwrite native occupancy.
        mapper.grid=bytearray([1])*40000
        mapper.path=[[8,8]]
        self.assertEqual(mapper.snapshot()['grid'][cell],'2')
        self.assertEqual(mapper.snapshot()['path'],raw['path'])
        wire=self.send([[.05,0,.05]]*20)
        for _ in range(5): self.send([],wire=wire)
        self.assertEqual(mapper.snapshot()['grid'][cell],'2')
        for _ in range(6):
            time.sleep(.16); self.send([[.05,0,.05]])
        raw=self.native(); shown=mapper.snapshot()
        self.assertEqual(raw['grid'][cell],'1')
        self.assertEqual(shown['grid'],raw['grid'])
        self.assertEqual(shown['pose'],raw['pose'])
        self.assertTrue(shown['native_map_live'])
        # New obstacle is immediate, even with floor endpoints in that cell.
        self.send([[.05,0,.05],[.05,.5,.05]])
        self.assertEqual(mapper.snapshot()['grid'][cell],'2')

    def test_disconnect_never_falls_back_to_python_map(self):
        self.send([[.05,0,.05]])
        mapper=Map(native=self.client)
        mapper.grid=bytearray([1])*40000
        self.proc.terminate(); self.proc.wait(timeout=3)
        self.client.poll()
        snap=mapper.snapshot()
        self.assertEqual(set(snap['grid']),{'0'})
        self.assertFalse(snap['native_map_live'])
        self.assertEqual(snap['path'],[])

    def test_session_and_explicit_reset_clear_authority(self):
        self.send([[.05,.5,.05]])
        epoch=self.native()['epoch']
        self.client.set_session('another-room'); self.client.poll()
        raw=self.native()
        self.assertGreater(raw['epoch'],epoch)
        self.assertEqual(set(raw['grid']),{'0'})
        self.assertFalse(raw['live'])
        self.send([[.05,.5,.05]])
        self.client.reset(); self.client.poll()
        self.assertEqual(set(self.native()['grid']),{'0'})

    def test_invalid_snapshot_and_stale_feed(self):
        self.send([[.05,0,.05]])
        good=self.native()
        for key,value in [('grid','1'*39999),('path',[[float('nan'),0]]),('pose',[0,0]),('size',201)]:
            bad=copy.deepcopy(good);bad[key]=value
            with self.assertRaises(ValueError): validate_snapshot(bad)
        time.sleep(.6); self.client.poll()
        self.assertFalse(self.client.snapshot()['native_map_live'])
        self.assertEqual(self.client.snapshot()['path'],[])

    def test_slow_display_cannot_block_control(self):
        with socket.socket() as slow:
            slow.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024)
            slow.connect(('127.0.0.1',self.ctl))
            slow.sendall(b'NAV SNAPSHOT\n'*100)
            time.sleep(.05)
            started=time.monotonic()
            with socket.create_connection(('127.0.0.1',self.ctl),timeout=.5) as control:
                control.settimeout(.5);control.sendall(b'CANCEL\n')
                self.assertEqual(control.makefile('rb').readline(),b'OK\n')
            self.assertLess(time.monotonic()-started,.5)

    def test_runtime_feed_always_uses_native_authority(self):
        self.assertIsInstance(SceneFeed().mapper.native,NativeGrid)

if __name__=='__main__': unittest.main()
