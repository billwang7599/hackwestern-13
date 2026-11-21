"""Detached QNX headless stack. start/stop/status require no display."""
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
USB = HERE.parent / 'iphone-usb'
EDGE = HERE.parent / 'edge-ai'
STATE = HERE / 'state'
ADDRESS = ('127.0.0.1', 5560)


def request(command):
    with socket.create_connection(ADDRESS, timeout=2) as sock:
        sock.settimeout(3)
        sock.sendall(command.encode() + b'\n')
        return sock.recv(8192).decode().strip()


def serve():
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    env = dict(os.environ, LD_LIBRARY_PATH=str(USB / 'install/lib'))
    children = {}
    backend = os.environ.get('TARS_MOTOR_BACKEND', 'stub')
    if backend not in ('stub', 'uno', 'pico'):
        raise ValueError('TARS_MOTOR_BACKEND must be stub, uno, or pico')
    motor_command = [str(HERE.parents[1] / 'iter1-wasd/pi/build/tars_motor'), '-b', backend, '-l', '-g', '5568']
    if backend != 'stub':
        device = os.environ.get('TARS_MOTOR_DEVICE')
        if not device or not (device.startswith('/dev/') or device == str(STATE / 'uno-serial')):
            raise ValueError('Hardware mode requires an explicit TARS_MOTOR_DEVICE=/dev/...')
        motor_command += ['-d', device]
    print('Motor backend: ' + backend + (' (no physical motor output)' if backend == 'stub' else ''), flush=True)
    commands = {
        'usbmuxd': ['sudo', '-n', 'sh', str(USB / 'run-usbmuxd.sh')],
        **{f'proxy{p}': [str(USB / 'install/bin/iproxy'), '-l', '-s', '127.0.0.1', f'{p}:{p}'] for p in (5557, 5558, 5559, 5561, 5562, 5563, 5573)},
        **({'uno_bridge': ['sudo', '-n', sys.executable, str(USB / 'uno_bridge.py')]}
           if backend == 'uno' and device == str(STATE / 'uno-serial') else {}),
        'motor': motor_command,
        'guard': [str(EDGE / 'build/lidar_guard'), '--motor-port', '5568', '--fusion-port', '5569'],
        'rgb': [sys.executable, str(EDGE / 'rgb_broker.py')],
        'ai': [sys.executable, str(EDGE / 'inference_worker.py')],
        **({'depth': [sys.executable, str(EDGE / 'inference_worker.py'), '--depth']}
           if os.environ.get('TARS_ENABLE_MIDAS') == '1' else {}),
        'headless': [sys.executable, str(HERE / 'headless.py')],
    }
    def launch(name):
        children[name] = subprocess.Popen(commands[name], env=env, stdin=subprocess.DEVNULL)
        print(f'Started {name}: {children[name].pid}', flush=True)
    def terminate(proc):
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2)

    with socket.socket() as admin:
        admin.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        admin.bind(ADDRESS)
        admin.listen(4)
        # Refuse to compete with an independently running proxy.
        for port in (5555, 5557, 5558, 5559, 5561, 5562, 5563, 5573):
            with socket.socket() as probe:
                if probe.connect_ex(('127.0.0.1', port)) == 0:
                    raise RuntimeError(f'Port {port} already in use; stop the existing proxy first')
        subprocess.run(['sudo', '-n', 'sh', str(USB / 'configure-runtime.sh')], check=True)
        try:
            for name in commands:
                launch(name)
            last_probe = time.monotonic()
            while running:
                if select.select([admin], [], [], 0.5)[0]:
                    client, _ = admin.accept()
                    with client:
                        client.settimeout(1)
                        try:
                            command = client.recv(32).strip()
                            if command in (b'gui', b'headless'):
                                terminate(children['headless'])
                                filename = 'dashboard.py' if command == b'gui' else 'headless.py'
                                commands['headless'] = [sys.executable, str(HERE / filename)]
                                launch('headless')
                            if command == b'stop':
                                running = False
                            client.sendall(json.dumps({k: {'pid': v.pid, 'running': v.poll() is None} for k, v in children.items()}).encode() + b'\n')
                        except OSError:
                            pass
                for name, proc in list(children.items()):
                    if running and proc.poll() is not None:
                        if name == 'headless' and commands[name][-1].endswith('/dashboard.py'):
                            commands[name] = [sys.executable, str(HERE / 'headless.py')]
                        # Avoid rapid restart loops on a persistent error.
                        time.sleep(1)
                        launch(name)
                if running and time.monotonic() - last_probe >= 10:
                    last_probe = time.monotonic()
                    try:
                        result = subprocess.run([str(USB / 'install/bin/idevice_id'), '-l'], env=env,
                                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=3)
                        if not result.stdout.strip():
                            # Old QNX libusb has no usable hotplug support. A fresh
                            # daemon re-enumerates without its polling reference leak.
                            terminate(children['usbmuxd'])
                            launch('usbmuxd')
                    except subprocess.TimeoutExpired:
                        terminate(children['usbmuxd'])
                        launch('usbmuxd')
        finally:
            for name in reversed(commands):
                if name in children:
                    terminate(children[name])


def main():
    command = sys.argv[1] if len(sys.argv) > 1 else 'status'
    if command == 'run':
        serve()
    elif command == 'start':
        try:
            print('Already running: ' + request('status'))
            return
        except OSError:
            pass
        STATE.mkdir(parents=True, exist_ok=True)
        with (STATE / 'headless-service.log').open('ab') as log:
            proc = subprocess.Popen([sys.executable, __file__, 'run'], stdin=subprocess.DEVNULL,
                                    stdout=log, stderr=log, start_new_session=True)
        for _ in range(30):
            time.sleep(0.1)
            if proc.poll() is not None:
                raise SystemExit(f'Start failed; see {STATE}/headless-service.log')
            try:
                status = request('status')
                if 'headless' in status:
                    print(status)
                    return
            except OSError:
                pass
        raise SystemExit(f'Still starting; check {STATE}/headless-service.log')
    elif command in ('status', 'stop', 'gui', 'headless'):
        try:
            print(request(command))
        except OSError as error:
            raise SystemExit(f'Service unavailable: {error}')
    else:
        raise SystemExit('Usage: service.py start|stop|status|gui|headless')


if __name__ == '__main__':
    main()
