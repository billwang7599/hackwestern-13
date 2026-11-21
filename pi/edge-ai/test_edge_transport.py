"""Bounds and fail-closed status checks for the independent RGB/AI adapters."""
import contextlib
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import time
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / 'dashboard'))
from rgb_broker import MAX_PACKET, read_packet
from inference_worker import result_line, write_job
from edge_status import EdgeStatus


class FragmentSocket:
    def __init__(self, data, fragment=1):
        self.data, self.fragment, self.reads = data, fragment, 0

    def recv(self, size):
        self.reads += 1
        part = self.data[:min(size, self.fragment)]
        self.data = self.data[len(part):]
        return part


def wire(jpeg=b'\xff\xd8\xff\xd9', frame_id=123):
    payload = b'RGB1' + struct.pack('!Q', frame_id) + jpeg
    return struct.pack('!I', len(payload)) + payload


@contextlib.contextmanager
def worker(script):
    proc = subprocess.Popen([sys.executable, '-c', script], stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
    try:
        yield proc
    finally:
        if proc.poll() is None:
            proc.kill()
        proc.wait(timeout=3)
        for pipe in (proc.stdin, proc.stdout, proc.stderr):
            pipe.close()


class RGBPacketTests(unittest.TestCase):
    def test_fragmented_rgb_frame_retains_64bit_source_id(self):
        source_id = 2**63 + 11
        packet = read_packet(FragmentSocket(wire(frame_id=source_id), 1))
        self.assertEqual(packet[:4], b'RGB1')
        self.assertEqual(struct.unpack('!Q', packet[4:12])[0], source_id)
        self.assertEqual(packet[12:], b'\xff\xd8\xff\xd9')

    def test_size_bound_rejects_header_without_waiting_for_body(self):
        for size in (0, 15, MAX_PACKET + 1, 2**32 - 1):
            with self.subTest(size=size):
                sock = FragmentSocket(struct.pack('!I', size), 4)
                with self.assertRaises(ValueError):
                    read_packet(sock)
                self.assertEqual(sock.reads, 1)

    def test_truncated_frame_raises_disconnect(self):
        for data in (b'', b'\x00\x00', wire()[:-1]):
            with self.subTest(data=data):
                with self.assertRaises(ConnectionError):
                    read_packet(FragmentSocket(data, 2))

    def test_invalid_magic_or_jpeg_envelope_rejected(self):
        for payload in (b'RGB2' + struct.pack('!Q', 1) + b'\xff\xd8\xff\xd9',
                        b'RGB1' + struct.pack('!Q', 1) + b'bad jpeg'):
            with self.subTest(payload=payload):
                with self.assertRaises(ValueError):
                    read_packet(FragmentSocket(struct.pack('!I', len(payload)) + payload, 7))


class WorkerResultTests(unittest.TestCase):
    def test_fragmented_native_result(self):
        script = "import os,time; os.write(1,b'{\"id\":7,'); time.sleep(.01); os.write(1,b'\"objects\":[]}\\n')"
        with worker(script) as proc:
            self.assertEqual(result_line(proc, 2), {'id': 7, 'objects': []})

    def test_worker_result_timeout_is_total_deadline(self):
        with worker('import time; time.sleep(5)') as proc:
            start = time.monotonic()
            with self.assertRaises(TimeoutError):
                result_line(proc, .05)
            self.assertLess(time.monotonic() - start, 1)

    def test_oversized_result_is_rejected(self):
        with worker("import os; os.write(1,b'x'*100001+b'\\n')") as proc:
            with self.assertRaises(ValueError):
                result_line(proc, 2)

    def test_partial_line_on_worker_exit_is_not_a_result(self):
        with worker("import os; os.write(1,b'{\"id\":1')") as proc:
            with self.assertRaises(ConnectionError):
                result_line(proc, 2)

    def test_json_array_is_not_a_native_result(self):
        with worker("print('[]')") as proc:
            with self.assertRaises(ValueError):
                result_line(proc, 2)

    def test_invalid_native_ids_are_rejected(self):
        for identifier in (None, True, -1, 2**32, 1.5, "7"):
            with self.subTest(identifier=identifier):
                line = json.dumps({'id': identifier})
                with worker('print(' + repr(line) + ')') as proc:
                    with self.assertRaises(ValueError):
                        result_line(proc, 2)

    def test_multiple_results_for_one_job_are_rejected(self):
        with worker("import os; os.write(1,b'{\"id\":1}\\n{\"id\":2}\\n')") as proc:
            with self.assertRaises(ValueError):
                result_line(proc, 2)


class WorkerInputTests(unittest.TestCase):
    def test_stalled_native_stdin_has_bounded_deadline(self):
        with worker('import time; time.sleep(5)') as proc:
            os.set_blocking(proc.stdin.fileno(), False)
            started = time.monotonic()
            with self.assertRaises(TimeoutError):
                write_job(proc, b'x' * 2_000_000, timeout=.05)
            self.assertLess(time.monotonic() - started, 1)

    def test_partial_pipe_writes_preserve_all_job_bytes(self):
        payload = bytes(range(256)) * 1024
        script = ("import sys,json,hashlib; data=sys.stdin.buffer.read(262144); "
                  "print(json.dumps({'id':5,'bytes':len(data),'sha':hashlib.sha256(data).hexdigest()}))")
        with worker(script) as proc:
            os.set_blocking(proc.stdin.fileno(), False)
            write_job(proc, payload, timeout=3)
            value = result_line(proc, 3)
        import hashlib
        self.assertEqual(value['bytes'], len(payload))
        self.assertEqual(value['sha'], hashlib.sha256(payload).hexdigest())


class EdgeFreshnessTests(unittest.TestCase):
    def test_missing_guard_defaults_to_no_permit(self):
        status = EdgeStatus().snapshot()
        self.assertFalse(status['safety']['permit'])
        self.assertFalse(status['safety']['live'])
        self.assertEqual(status['safety']['verdict'], 'STALE')
        self.assertIsNone(status['safety']['status_age_ms'])

    def test_guard_expires_independently_of_fresh_ai(self):
        edge = EdgeStatus()
        edge.latest = {'safety': ({'verdict': 'CLEAR', 'permit': True, 'connected': True}, 99.6),
                       'ai': ({'live': True, 'objects': [{'label': 'chair'}]}, 99.9),
                       'depth': ({'live': True, 'inverse_depth': [5.0]}, 99.9)}
        with patch('edge_status.time.monotonic', return_value=100):
            status = edge.snapshot()
        self.assertFalse(status['safety']['permit'])
        self.assertEqual(status['safety']['verdict'], 'STALE')
        self.assertTrue(status['ai']['live'])
        self.assertTrue(status['depth']['live'])
        self.assertTrue(edge.latest['safety'][0]['permit'], 'snapshot must not mutate stored status')

    def test_ai_failure_does_not_change_fresh_guard(self):
        edge = EdgeStatus()
        edge.latest = {'safety': ({'verdict': 'CLEAR', 'permit': True, 'connected': True}, 99.9),
                       'ai': ({'live': False, 'error': 'worker stopped'}, 99.99)}
        with patch('edge_status.time.monotonic', return_value=100):
            status = edge.snapshot()
        self.assertTrue(status['safety']['permit'])
        self.assertTrue(status['safety']['live'])
        self.assertFalse(status['ai']['live'])

    def test_reported_guard_disconnect_cannot_retain_permit(self):
        edge = EdgeStatus()
        for field in ('live', 'connected'):
            edge.latest = {'safety': ({'verdict': 'CLEAR', 'permit': True, field: False}, 99.9)}
            with patch('edge_status.time.monotonic', return_value=100):
                status = edge.snapshot()
            self.assertFalse(status['safety']['permit'], field)


if __name__ == '__main__':
    unittest.main()
