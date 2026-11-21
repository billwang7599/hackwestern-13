#!/usr/bin/env python3
"""Fault-injection demo: synthetic LiDAR + real QNX YOLO process, no motors.

Uses private localhost ports so the installed robot service is not interrupted.
A passing run demonstrates this observed test, not hard real-time certification.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent


def clear_packet(seq):
    points = [(x, 0.0, z) for x in (-.3, 0.0, .3) for z in (.25, .4, .6, .8, 1.0)]
    body = struct.pack('!4sIQII5f', b'LDR1', seq, int(time.monotonic() * 1_000_000),
                       7, len(points), 0, .5, 0, 0, 0)
    body += b''.join(struct.pack('!3f', *point) for point in points)
    return struct.pack('!I', len(body)) + body


class Experiment:
    def __init__(self, args):
        self.args = args
        self.cleanup = threading.Event()
        self.stop_feeding = threading.Event()
        self.feeding_stopped = threading.Event()
        self.lock = threading.Lock()
        self.records = deque(maxlen=256)
        self.errors = deque(maxlen=12)
        self.ai_lines = 0
        self.ai_last = None
        self.depth_lines = 0
        self.depth_last = None
        self.last_send = None
        self.guard = self.ai = self.depth = None
        self.connection = None
        self.threads = []
        self.server = socket.socket()
        self.server.bind(('127.0.0.1', args.port))
        self.server.listen(1)
        self.server.settimeout(3)
        self.status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.status.bind(('127.0.0.1', args.status_port))
        self.status.settimeout(.1)

    def thread(self, fn, *args):
        worker = threading.Thread(target=fn, args=args, daemon=True)
        self.threads.append(worker)
        worker.start()

    def record_status(self):
        while not self.cleanup.is_set():
            try:
                message = self.status.recv(4096)
                value = json.loads(message)
                if value.get('type') != 'guard':
                    raise ValueError('Unexpected UDP status type')
                with self.lock:
                    self.records.append((time.monotonic(), value))
            except socket.timeout:
                continue
            except (OSError, ValueError) as error:
                if not self.cleanup.is_set():
                    with self.lock:
                        self.errors.append('status: ' + str(error))
                return

    def feed_sensor(self):
        seq = 0
        next_send = time.monotonic()
        try:
            while not self.cleanup.is_set() and not self.stop_feeding.is_set():
                seq += 1
                self.connection.sendall(clear_packet(seq))
                with self.lock:
                    self.last_send = time.monotonic()
                if self.connection.recv(1) != b'\x01':
                    raise RuntimeError('Guard did not ACK synthetic LiDAR')
                next_send += .05
                self.stop_feeding.wait(max(0, next_send - time.monotonic()))
        except (OSError, RuntimeError) as error:
            if not self.cleanup.is_set():
                with self.lock:
                    self.errors.append('sensor: ' + str(error))
        finally:
            # Keep TCP open while no more frames arrive, to exercise age expiry
            # rather than the guard's earlier disconnect interlock.
            self.feeding_stopped.set()

    def drain(self, stream, name):
        while True:
            # Bound each line even if an external program misbehaves.
            limit = 131072 if name == 'depth_stdout' else 16384
            line = stream.readline(limit + 1)
            if not line:
                return
            if len(line) > limit:
                with self.lock:
                    self.errors.append(name + ': oversized diagnostic line')
                return
            if name == 'ai_stdout':
                try:
                    value = json.loads(line)
                    if value.get('model') != 'YOLOX-Nano' or not isinstance(value.get('objects'), list):
                        raise ValueError('Unexpected detector output')
                    with self.lock:
                        self.ai_lines += 1
                        self.ai_last = value
                except (ValueError, AttributeError) as error:
                    with self.lock:
                        self.errors.append(name + ': ' + str(error))
            elif name == 'depth_stdout':
                try:
                    value = json.loads(line)
                    depths = value.get('inverse_depth')
                    if (value.get('model') != 'MiDaS-small' or
                            value.get('kind') != 'relative_inverse_depth' or
                            value.get('depth_width') != 64 or value.get('depth_height') != 64 or
                            not isinstance(depths, list) or len(depths) != 4096 or
                            not all(type(d) in (int, float) and math.isfinite(d) and 0 <= d <= 1e7 for d in depths)):
                        raise ValueError('Unexpected relative-depth output')
                    # Retain only metadata, not a sequence of depth maps.
                    summary = {key: value.get(key) for key in
                               ('model', 'runtime', 'inference_ms', 'processing_ms')}
                    with self.lock:
                        self.depth_lines += 1
                        self.depth_last = summary
                except (ValueError, AttributeError) as error:
                    with self.lock:
                        self.errors.append(name + ': ' + str(error))
            else:
                # Native scheduling/runtime notes are retained only for failures.
                with self.lock:
                    self.errors.append(name + ': ' + line.decode(errors='replace').strip()[:300])

    def latest(self):
        with self.lock:
            return self.records[-1] if self.records else None

    def wait_clear(self, timeout=1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            latest = self.latest()
            if latest and latest[1].get('permit') and latest[1].get('verdict') == 'CLEAR':
                return latest
            time.sleep(.01)
        raise RuntimeError('Synthetic sensor did not establish CLEAR')

    def phase(self, name, duration=.35):
        beginning = time.monotonic()
        while time.monotonic() - beginning < duration:
            latest = self.latest()
            if self.guard.poll() is not None:
                raise RuntimeError('LiDAR guard exited during ' + name)
            if latest and time.monotonic() - latest[0] > .2:
                raise RuntimeError('Guard heartbeat disappeared during ' + name)
            time.sleep(.01)
        with self.lock:
            rows = [value for arrived, value in self.records if arrived >= beginning]
        if len(rows) < 3:
            raise RuntimeError('Too few guard heartbeats during ' + name)
        if any(value.get('verdict') != 'CLEAR' or not value.get('permit') or
               value.get('sample_age_ms', 1000) >= 150 for value in rows):
            raise RuntimeError('Clearance/freshness changed during ' + name + ': ' + json.dumps(rows[-1]))
        if rows[-1]['frames'] <= rows[0]['frames'] or rows[-1]['ticks'] <= rows[0]['ticks']:
            raise RuntimeError('Guard stopped consuming/ticking during ' + name)
        return dict(phase=name, heartbeats=len(rows), ticks_advanced=rows[-1]['ticks'] - rows[0]['ticks'],
                    frames_advanced=rows[-1]['frames'] - rows[0]['frames'],
                    maximum_sample_age_ms=round(max(r['sample_age_ms'] for r in rows), 3),
                    max_lateness_ms=rows[-1]['max_lateness_ms'],
                    missed_ticks=rows[-1]['missed_ticks'], verdict='CLEAR')

    def run(self):
        started = time.monotonic()
        required = [self.args.guard, self.args.ai, self.args.model, self.args.image]
        if self.args.with_depth:
            required += [self.args.depth, self.args.depth_model]
        for file in required:
            if not file.is_file():
                raise RuntimeError('Required real artifact missing: ' + str(file))
        self.thread(self.record_status)
        self.guard = subprocess.Popen([str(self.args.guard), '--port', str(self.server.getsockname()[1]),
                                       '--status-port', str(self.status.getsockname()[1])],
                                      stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                      stderr=subprocess.PIPE)
        self.thread(self.drain, self.guard.stderr, 'guard_stderr')
        self.connection, _ = self.server.accept()
        self.connection.settimeout(.2)
        self.thread(self.feed_sensor)
        self.wait_clear()
        phases = [self.phase('before_ai')]
        self.ai = subprocess.Popen([str(self.args.ai), '--model', str(self.args.model),
                                    '--threads', '1', '--image', str(self.args.image), '--repeat', '100'],
                                   stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.thread(self.drain, self.ai.stdout, 'ai_stdout')
        self.thread(self.drain, self.ai.stderr, 'ai_stderr')
        if self.args.with_depth:
            self.depth = subprocess.Popen([str(self.args.depth), '--model', str(self.args.depth_model),
                                           '--threads', '1', '--image', str(self.args.image), '--repeat', '100'],
                                          stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.thread(self.drain, self.depth.stdout, 'depth_stdout')
            self.thread(self.drain, self.depth.stderr, 'depth_stderr')
        ready_seconds = 6 if self.args.with_depth else 4
        deadline = time.monotonic() + ready_seconds
        while True:
            with self.lock:
                completed = self.ai_lines
                depth_completed = self.depth_lines
            if completed and (not self.args.with_depth or depth_completed):
                break
            if self.ai.poll() is not None:
                raise RuntimeError('Real AI exited before producing inference output')
            if self.depth is not None and self.depth.poll() is not None:
                raise RuntimeError('Real depth worker exited before both AI workers became ready')
            if time.monotonic() > deadline:
                raise RuntimeError('No real inference result from every enabled AI worker within ' + str(ready_seconds) + ' seconds')
            latest = self.latest()
            if not latest or time.monotonic() - latest[0] > .2 or not latest[1].get('permit'):
                raise RuntimeError('Guard freshness/clearance lost during AI initialization')
            time.sleep(.01)
        phases.append(self.phase('ai_running', .35))
        if self.ai.poll() is not None:
            raise RuntimeError('AI finished before SIGSTOP test; cannot claim paused-process isolation')
        os.kill(self.ai.pid, signal.SIGSTOP)
        deadline = time.monotonic() + .5
        stopped = False
        while time.monotonic() < deadline:
            pid, status = os.waitpid(self.ai.pid, os.WNOHANG | os.WUNTRACED)
            if pid and os.WIFSTOPPED(status):
                stopped = True
                break
            if pid:
                raise RuntimeError('AI exited instead of stopping')
            time.sleep(.01)
        if not stopped:
            raise RuntimeError('Could not observe AI stopped state')
        phases.append(self.phase('ai_sigstop', .5))
        os.kill(self.ai.pid, signal.SIGKILL)
        self.ai.wait(timeout=1)
        phases.append(self.phase('ai_killed', .35))
        if self.depth is not None:
            if self.depth.poll() is not None:
                raise RuntimeError('Depth worker exited before its SIGSTOP test')
            os.kill(self.depth.pid, signal.SIGSTOP)
            deadline = time.monotonic() + .5
            depth_stopped = False
            while time.monotonic() < deadline:
                pid, status = os.waitpid(self.depth.pid, os.WNOHANG | os.WUNTRACED)
                if pid and os.WIFSTOPPED(status):
                    depth_stopped = True
                    break
                if pid:
                    raise RuntimeError('Depth worker exited instead of stopping')
                time.sleep(.01)
            if not depth_stopped:
                raise RuntimeError('Could not observe depth worker stopped state')
            phases.append(self.phase('depth_sigstop', .5))
            os.kill(self.depth.pid, signal.SIGKILL)
            self.depth.wait(timeout=1)
            phases.append(self.phase('depth_killed', .35))
        self.stop_feeding.set()
        if not self.feeding_stopped.wait(.4):
            raise RuntimeError('Synthetic sensor did not stop')
        with self.lock:
            last_send = self.last_send
        deadline = last_send + .25
        stale = None
        while time.monotonic() < deadline:
            latest = self.latest()
            if latest and latest[0] > last_send and latest[1].get('verdict') == 'STALE':
                stale = latest
                break
            time.sleep(.005)
        if stale is None or stale[1].get('permit'):
            raise RuntimeError('LiDAR silence did not revoke clearance within250ms')
        delay_ms = (stale[0] - last_send) * 1000
        if not 145 <= stale[1].get('sample_age_ms', -1) <= 250:
            raise RuntimeError('Stale result did not exercise sample-age watchdog')
        with self.lock:
            ai_last, ai_lines = self.ai_last, self.ai_lines
        result = dict(passed=True, sensor='synthetic LDR1 floor returns at20Hz; no motors',
                    ai='actual YOLOX-Nano inference on supplied image', ai_completed=ai_lines,
                    ai_runtime=ai_last.get('runtime'), ai_last_inference_ms=ai_last.get('inference_ms'),
                    phases=phases, lidar_silence_to_observed_stale_ms=round(delay_ms, 3),
                    guard_reported_stale_age_ms=stale[1]['sample_age_ms'],
                    scheduling={key: stale[1].get(key) for key in
                                ('policy', 'priority', 'requested_priority', 'requested_cpu', 'runmask',
                                 'schedule_error', 'affinity_error')},
                    elapsed_s=round(time.monotonic() - started, 3),
                    limitation='Observed process-isolation test, not a hard real-time guarantee or physical LiDAR test')
        if self.args.with_depth:
            with self.lock:
                depth_last, depth_lines = self.depth_last, self.depth_lines
            result.update(depth='actual MiDaS-small relative-depth inference on supplied image',
                          depth_completed=depth_lines, depth_runtime=depth_last.get('runtime'),
                          depth_last_inference_ms=depth_last.get('inference_ms'))
        return result

    def close(self):
        self.cleanup.set()
        self.stop_feeding.set()
        if self.connection:
            self.connection.close()
        self.server.close()
        self.status.close()
        for process in (self.ai, self.depth, self.guard):
            if process and process.poll() is None:
                # Kill a potentially SIGSTOPped AI; SIGTERM alone would wait.
                if process is not self.guard:
                    process.kill()
                else:
                    process.terminate()
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1)
        for worker in self.threads:
            worker.join(timeout=.15)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--guard', type=Path, default=HERE / 'build/lidar_guard')
    parser.add_argument('--ai', type=Path, default=HERE / 'build/ai_detector')
    parser.add_argument('--model', type=Path, default=HERE / 'models/yolox_nano.onnx')
    parser.add_argument('--image', type=Path, default=HERE / 'models/dog.jpg')
    parser.add_argument('--with-depth', action='store_true', help='Also run and fault-inject the actual MiDaS worker')
    parser.add_argument('--depth', type=Path, default=HERE / 'build/ai_depth')
    parser.add_argument('--depth-model', type=Path, default=HERE / 'models/midas-small.onnx')
    parser.add_argument('--port', type=int, default=0, help='Private synthetic-sensor TCP port;0 chooses free')
    parser.add_argument('--status-port', type=int, default=0, help='Private guard UDP port;0 chooses free')
    args = parser.parse_args()
    required = [args.guard, args.ai, args.model, args.image]
    if args.with_depth:
        required += [args.depth, args.depth_model]
    for file in required:
        if not file.is_file():
            print(json.dumps(dict(passed=False, error='Required real artifact missing: ' + str(file))))
            return 1
    if not 0 <= args.port <= 65535 or not 0 <= args.status_port <= 65535:
        parser.error('Test ports must be between0 and65535')
    try:
        experiment = Experiment(args)
    except OSError as error:
        print(json.dumps(dict(passed=False, error='Test socket setup: ' + str(error))))
        return 1
    try:
        result = experiment.run()
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        with experiment.lock:
            diagnostics = list(experiment.errors)
        print(json.dumps(dict(passed=False, error=str(error), diagnostics=diagnostics), indent=2))
        return 1
    finally:
        experiment.close()
    print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
