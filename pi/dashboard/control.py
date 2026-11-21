"""USB return channel and leased command bridge to the independently guarded motor server."""
import json
import math
import queue
import socket
import threading
import time


def parse_command(message):
    if not isinstance(message, dict):
        raise ValueError('Expected command object')
    op = message.get('op')
    if op in ('start', 'stop', 'halt', 'map_save', 'map_load', 'map_clear', 'control_phone'):
        return {'op': op}
    if op == 'select_target':
        target = message.get('id')
        if not isinstance(target, str) or not 1 <= len(target) <= 64:
            raise ValueError('Invalid target id')
        return dict(op=op, id=target)
    if op == 'drive':
        values = [message.get('left'), message.get('right')]
        if not all(type(v) in (int, float) and math.isfinite(v) and -1 <= v <= 1 for v in values):
            raise ValueError('Invalid debug drive values')
        return dict(op=op, left=values[0], right=values[1])
    raise ValueError('Unknown command')


class ControlLink:
    def __init__(self, port=5559):
        self.port = port
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.commands = queue.Queue(maxsize=16)
        self.latest = {}
        self.connected = False

    def publish(self, status):
        with self.lock:
            self.latest = status

    def run(self):
        while not self.stop.is_set():
            try:
                with socket.create_connection(('127.0.0.1', self.port), timeout=2) as sock:
                    sock.settimeout(0.1)
                    self.connected = True
                    buf = b''
                    last_send = 0
                    while not self.stop.is_set():
                        now = time.monotonic()
                        if now - last_send >= 0.2:
                            with self.lock:
                                data = json.dumps(self.latest, allow_nan=False).encode() + b'\n'
                            sock.sendall(data)
                            last_send = now
                        try:
                            data = sock.recv(4096)
                        except socket.timeout:
                            continue
                        if not data:
                            raise ConnectionError('Phone status channel disconnected')
                        buf += data
                        if len(buf) > 8192:
                            raise ValueError('Oversized command')
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            command = parse_command(json.loads(line))
                            # A full queue closes this connection and clears drive intent.
                            self.commands.put_nowait(command)
            except (OSError, ValueError, queue.Full):
                pass
            finally:
                self.connected = False
                while True:
                    try:
                        self.commands.get_nowait()
                    except queue.Empty:
                        break
            self.stop.wait(1)


class DebugDrive:
    """Bounded phone intent; MotorLink sends it to the native motor authority."""
    def __init__(self):
        self.left = self.right = 0.0
        self.expires = 0.0
        self.generation = 0

    def command(self, command, now):
        if command['op'] == 'drive':
            self.generation += 1
            self.left, self.right = command['left'], command['right']
            self.expires = now + 0.5
        elif command['op'] in ('halt', 'stop'):
            self.left = self.right = 0.0

    def state(self, now, connected):
        if not connected or now >= self.expires:
            self.left = self.right = 0.0
        return self.left, self.right


class MotorLink:
    """Best-effort phone command bridge; the native gate is the final authority.

    A stopped dashboard cannot keep driving: this mailbox expires in 150 ms.
    A native refusal or TCP reconnect consumes the command generation, so
    clearing an obstacle never resurrects an old phone button press.
    """
    def __init__(self, port=5555):
        self.port = port
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.requested = (0.0, 0.0)
        self.generation = 0
        self.denied_generation = -1
        self.lease_generation = -1
        self.lease_epoch = 0
        self.expires = 0.0
        self.latest = {}
        self.received = 0.0
        self.connected = False
        self.error = 'Motor server not connected'

    def update(self, left, right, generation):
        with self.lock:
            self.requested = (left, right)
            self.generation = generation
            self.expires = time.monotonic() + 0.15

    def desired(self):
        with self.lock:
            if time.monotonic() >= self.expires or self.generation == self.denied_generation:
                return (0.0, 0.0), self.generation
            return self.requested, self.generation

    @staticmethod
    def parse_status(line):
        fields = line.split()
        if len(fields) < 7 or fields[0] != 'OK':
            raise ValueError('Invalid native motor status')
        output = [float(fields[1]), float(fields[2])]
        if not all(math.isfinite(v) and -1 <= v <= 1 for v in output):
            raise ValueError('Invalid motor output')
        values = dict(item.split('=', 1) for item in fields[6:] if '=' in item)
        backend = values.get('backend', 'unknown')
        return dict(output=output, backend=backend, mode='stub' if backend == 'stub' else 'hardware',
                    inhibited=fields[5] != '0', reason=values.get('why', 'unknown'),
                    guard_enabled=values.get('guard_enabled') == '1',
                    guard_permit=values.get('guard_permit') == '1',
                    guard_age_ms=int(values.get('guard_age_ms', '-1')),
                    guard_epoch=int(values.get('guard_epoch', '0')),
                    arduino_ready_count=int(values.get('ready', '0')),
                    move=values.get('move', 'unknown'))

    def snapshot(self):
        with self.lock:
            value = dict(self.latest)
            fresh = self.received and time.monotonic() - self.received < 0.4
            connected = self.connected and bool(fresh)
            value.update(connected=connected, requested=list(self.requested),
                         command_error=self.error,
                         status_age_ms=round((time.monotonic() - self.received) * 1000) if self.received else None)
            if not connected:
                value.update(inhibited=True, reason='motor_disconnected', output=[0, 0])
            elif self.denied_generation == self.generation:
                value.update(inhibited=True, reason='new_command_required')
            return value

    def run(self):
        while not self.stop.is_set():
            try:
                with socket.create_connection(('127.0.0.1', self.port), timeout=0.2) as sock:
                    sock.settimeout(0.15)
                    with sock.makefile('rb') as stream:
                        def exchange(command):
                            sock.sendall(command.encode('ascii') + b'\n')
                            response = stream.readline(2048)
                            if not response.endswith(b'\n'):
                                raise ValueError('Missing/oversized motor response')
                            return response.decode('ascii').strip()
                        exchange('S')
                        initial = self.parse_status(exchange('STATUS'))
                        if not initial['guard_enabled']:
                            raise ValueError('Native motor guard is disabled')
                        with self.lock:
                            self.connected = True
                            self.latest, self.received = initial, time.monotonic()
                            self.denied_generation = self.generation
                        while not self.stop.is_set():
                            (left, right), generation = self.desired()
                            with self.lock:
                                if (left or right) and self.lease_generation != generation:
                                    self.lease_generation = generation
                                    self.lease_epoch = self.latest.get('guard_epoch', 0)
                                epoch = self.lease_epoch
                            command = f'DL {epoch} {left:.3f} {right:.3f}' if left or right else 'S'
                            reply = exchange(command)
                            with self.lock:
                                self.error = None if reply.startswith('OK') else reply
                                if not reply.startswith('OK'):
                                    self.denied_generation = generation
                            status = self.parse_status(exchange('STATUS'))
                            if not status['guard_enabled']:
                                # This integration never drives an unguarded server.
                                exchange('S')
                                raise ValueError('Native motor guard is disabled')
                            with self.lock:
                                self.latest, self.received = status, time.monotonic()
                                if status['inhibited'] and (left or right):
                                    self.denied_generation = generation
                            self.stop.wait(0.05)
                        exchange('S')
            except (OSError, ValueError, UnicodeError) as error:
                with self.lock:
                    self.error = str(error)
            finally:
                with self.lock:
                    self.connected = False
                    self.denied_generation = self.generation
            self.stop.wait(0.5)
