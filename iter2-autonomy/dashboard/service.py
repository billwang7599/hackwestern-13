"""Detached QNX headless stack. start/stop/status require no display."""
import json
import os
from pathlib import Path
import secrets
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


def _nav_dir():
    nav = HERE.parent / 'goto'
    return nav if nav.is_dir() else HERE.parents[1] / 'iter2-autonomy/goto'


def demo_preflight():
    """TARS_DEMO=1: everything real or nothing. Sets the demo defaults in os.environ and returns the list of
    problems (empty = go). No stub motors, autonomy on, YOLO on the phone camera, voice, the live view."""
    for key, want in (('TARS_ENABLE_GOTO', '1'), ('TARS_ENABLE_VOICE', '1')):
        if os.environ.get(key, want) != want:
            return [f'TARS_DEMO=1 needs {key}={want} (it is {os.environ[key]})']
        os.environ[key] = want
    problems = []
    if os.environ.get('TARS_MOTOR_BACKEND', 'stub') == 'stub':
        problems.append('motors would be the stub (no wheels): set TARS_MOTOR_BACKEND=uno TARS_MOTOR_DEVICE=/dev/...')
    if not os.environ.get('TARS_MOTOR_DEVICE'):
        problems.append('no TARS_MOTOR_DEVICE=/dev/... (the Uno\'s serial device)')
    nav = _nav_dir()
    for f, how in (('build/goto', 'sh iter2-autonomy/goto/build-native.sh'),
                   ('build/detect', 'sh iter2-autonomy/goto/build-native.sh (needs edge-ai/setup-qnx.sh first)'),
                   ('models/yolo_world.onnx', 'iter2-autonomy/goto/get_model.sh on the laptop, copy models/ over'),
                   ('models/labels.txt', 'comes with yolo_world.onnx')):
        if not (nav / f).is_file():
            problems.append(f'no {nav / f}: {how}')
    voice = [Path(os.environ['TARS_VOICE_DIR']) / 'tars_voice', Path(os.environ['TARS_VOICE_DIR']) / 'build/native/tars_voice'] \
        if os.environ.get('TARS_VOICE_DIR') else \
        [HERE.parent / 'voice/build/native/tars_voice', Path('/data/home/qnxuser/voice/tars_voice')]
    if not any(p.is_file() for p in voice):
        problems.append('no tars_voice: make -C iter2-autonomy/voice native')
    if len(os.environ.get('TARS_BENCH_TOKEN', '')) < 12:
        problems.append('no live view: set TARS_BENCH_TOKEN=<12+ characters> (open http://<pi-ip>:8770/?token=...)')
    return problems


def demo_clear_memory():
    """Start the demo with no remembered objects: goto's memory file from earlier tests is moved aside
    (memory.txt -> memory.txt.before-demo-<time>), never deleted, so it can be put back."""
    nav = _nav_dir()
    path = 'memory.txt'
    try:
        for line in (nav / 'goto.conf').read_text().splitlines():
            k, _, v = line.split('#', 1)[0].partition('=')
            if k.strip() == 'memory_path' and v.strip():
                path = v.strip()
    except OSError:
        pass
    if path == '-':
        return
    mem = Path(path) if Path(path).is_absolute() else nav / path
    if mem.is_file():
        aside = mem.with_name(mem.name + time.strftime('.before-demo-%Y%m%d-%H%M%S'))
        mem.rename(aside)
        print(f'Demo: goto starts with no memory ({mem} moved to {aside.name})', flush=True)


def serve():
    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    if os.environ.get('TARS_DEMO') == '1':
        problems = demo_preflight()
        if problems:
            raise ValueError('TARS_DEMO=1 refused:\n  ' + '\n  '.join(problems))
        demo_clear_memory()
        print('DEMO: real motors, autonomy, YOLO on the phone camera, voice, live view', flush=True)
    env = dict(os.environ, LD_LIBRARY_PATH=str(USB / 'install/lib'))
    children = {}
    backend = os.environ.get('TARS_MOTOR_BACKEND', 'stub')
    if backend not in ('stub', 'uno', 'pico', 'gpio'):
        raise ValueError('TARS_MOTOR_BACKEND must be stub, uno, pico, or gpio')
    flat_layout = HERE.parent.name == 'pi'
    motor_dir = HERE.parent if flat_layout else HERE.parents[1] / 'iter1-wasd/pi'
    phone_ports = (5557, 5558, 5559, 5561, 5562, 5563)
    if not flat_layout:
        phone_ports += (5573, 5577)    # 5573 TARS speech -> phone, 5577 phone mic (AirPods) -> tars_voice
    motor_command = [str(motor_dir / 'build/tars_motor'), '-b', backend, '-l', '-g', '5568']
    if backend == 'gpio':
        # Pi GPIO straight into the L298N (no Uno). TARS_MOTOR_DEVICE = options, e.g. "invl,min=0.4".
        device = os.environ.get('TARS_MOTOR_DEVICE', '')
        if device:
            motor_command += ['-d', device]
    elif backend != 'stub':
        device = os.environ.get('TARS_MOTOR_DEVICE')
        if not device or not (device.startswith('/dev/') or device == str(STATE / 'uno-serial')):
            raise ValueError('Hardware mode requires an explicit TARS_MOTOR_DEVICE=/dev/...')
        motor_command += ['-d', device]
    print('Motor backend: ' + backend + (' (no physical motor output)' if backend == 'stub' else ''), flush=True)
    # Wi-Fi phone link: TARS_PHONE_HOST, or the address in state/phone-host.
    phone_host = os.environ.get('TARS_PHONE_HOST', '').strip()
    if not phone_host and (STATE / 'phone-host').is_file():
        phone_host = (STATE / 'phone-host').read_text().strip()
    if phone_host:
        phone_link = {'wifi_proxy': [sys.executable, str(USB / 'wifi_proxy.py'), phone_host,
                                     *map(str, phone_ports)]}
    else:
        phone_link = {
            'usbmuxd': ['sudo', '-n', 'sh', str(USB / 'run-usbmuxd.sh')],
            **{f'proxy{p}': [str(USB / 'install/bin/iproxy'), '-l', '-s', '127.0.0.1', f'{p}:{p}'] for p in phone_ports},
        }
    print('Phone link: ' + (f'Wi-Fi {phone_host}' if phone_host else 'USB'), flush=True)
    commands = {
        **phone_link,
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
    # The installed flat Pi layout keeps navigation under iter2-autonomy.
    nav = HERE.parent / 'goto'
    if not nav.is_dir():
        nav = HERE.parents[1] / 'iter2-autonomy/goto'
    keyboard_auth = secrets.token_hex(16)
    nav_mode = env.get('TARS_ENABLE_GOTO', 'map')
    commands['guard'] += ['--identify-obstacles', '1']
    # One source of truth for the car's size: goto.conf (car_length_m / car_width_m / safety_margin_m /
    # guard_stop_m). goto prints the matching lidar_guard corridor flags; without a goto build the
    # guard keeps its defaults (0.45 / 0.6).
    try:
        flags = subprocess.run([str(nav / 'build/goto'), '-c', str(nav / 'goto.conf'), '--guard-flags'],
                               capture_output=True, text=True, timeout=5).stdout.split()
        if len(flags) == 4 and flags[0] == '--half-width' and flags[2] == '--stop-distance':
            commands['guard'] += flags
            print('Guard corridor from goto.conf: ' + ' '.join(flags), flush=True)
    except (OSError, subprocess.SubprocessError):
        pass
    if nav_mode != '0':
        commands['guard'] += ['--nav-port', '5572']
        commands['goto'] = [str(nav / 'build/goto'), '-c', str(nav / 'goto.conf'),
                            '-o', str(nav / 'objects.txt')]
        if nav_mode == 'map':
            # Add the map to the manual-drive profile without enabling autonomy.
            commands['goto'] += ['-m', '127.0.0.1:0']
        elif nav_mode == '1':
            commands['guard'] += ['--require-clear', '1']
        else:
            raise ValueError('TARS_ENABLE_GOTO must be 0, 1 or map')
    # Real-robot mode for the laptop sim's pieces (bench/run_sim.py fakes these three):
    #  detect: YOLO-World on the mounted iPhone's camera (rgb_broker 5566) -> goto detections 5571.
    #          Only with autonomy on. TARS_DETECT_ROTATE turns the phone's sensor-orientation
    #          frames upright (default 270 = 90 deg counter-clockwise, unverified: check with
    #          `build/detect --camera broker --nomodel --frames 1 --save-frame up.ppm --rotate 270`).
    #  voice:  tars_voice from TARS_VOICE_DIR (make -C iter2-autonomy/voice deploy), TARS_ENABLE_VOICE=1.
    #          Don't also run its start_voice.sh.
    #  relay:  mission_relay for the laptop bench (--target pi), only when MISSION_KEY is set.
    cwds = {}
    if nav_mode == '1':
        detect, model = nav / 'build/detect', nav / 'models/yolo_world.onnx'
        if detect.is_file() and model.is_file():
            commands['detect'] = [str(detect), '--camera', 'broker:5566', '--model', str(model),
                                  '--labels', str(nav / 'models/labels.txt'),
                                  '--rotate', os.environ.get('TARS_DETECT_ROTATE', '270'),
                                  '--threads', os.environ.get('TARS_DETECT_THREADS', '1'),
                                  '--fps', os.environ.get('TARS_DETECT_FPS', '4'), '--quiet']
            # docs/ARCHITECTURE.md CPU table: YOLOX is off while YOLO-World runs.
            if os.environ.get('TARS_KEEP_YOLOX') != '1':
                del commands['ai']
                print('detect on: YOLOX worker off (TARS_KEEP_YOLOX=1 keeps it)', flush=True)
        else:
            print(f'No {detect} or {model}: GOTO will stop with "no camera frames"', flush=True)
    if os.environ.get('TARS_ENABLE_VOICE') == '1':
        # 1. native build in the repo (make -C iter2-autonomy/voice native): run from voice/, where
        #    voice.conf and models/ are; 2. the SDP deploy folder (make deploy); TARS_VOICE_DIR overrides.
        repo_voice = HERE.parent / 'voice'
        if os.environ.get('TARS_VOICE_DIR'):
            voice_dir = Path(os.environ['TARS_VOICE_DIR'])
            binary = voice_dir / 'tars_voice'
            if not binary.is_file():
                binary = voice_dir / 'build/native/tars_voice'
        elif (repo_voice / 'build/native/tars_voice').is_file():
            voice_dir, binary = repo_voice, repo_voice / 'build/native/tars_voice'
        else:
            voice_dir = Path('/data/home/qnxuser/voice')
            binary = voice_dir / 'tars_voice'
        if not binary.is_file():
            raise ValueError('TARS_ENABLE_VOICE=1 but no tars_voice: run make -C iter2-autonomy/voice native '
                             '(or set TARS_VOICE_DIR)')
        if not (voice_dir / 'models/ggml-tiny.en-q5_1.bin').is_file():
            raise ValueError(f'TARS_ENABLE_VOICE=1 but no {voice_dir}/models/ggml-tiny.en-q5_1.bin '
                             '(iter2-autonomy/voice/get_model.sh)')
        commands['voice'] = [str(binary), '-c', 'voice.conf']
        cwds['voice'] = str(voice_dir)
    # The bench's live view ON the robot: the sim's 2D room page drawn from the real robot (bench/plugins/live.py),
    # opened from a laptop at http://<pi-ip>:8770/?token=<TARS_BENCH_TOKEN>. The page can drive, hence the token.
    bench = HERE.parents[1] / 'bench/bench.py'
    if env.get('TARS_BENCH_TOKEN'):
        if len(env['TARS_BENCH_TOKEN']) < 12:
            raise ValueError('TARS_BENCH_TOKEN must be at least 12 characters')
        if not bench.is_file():
            raise ValueError(f'TARS_BENCH_TOKEN set but no {bench}')
        commands['bench'] = [sys.executable, str(bench), '--target', 'robot', '--bind', '0.0.0.0',
                             '--port', env.get('TARS_BENCH_PORT', '8770')]
    relay = HERE.parent / 'mission/build/mission_relay'
    if env.get('MISSION_KEY'):
        if not relay.is_file():
            raise ValueError(f'MISSION_KEY set but no {relay} (make -C iter2-autonomy/mission native)')
        commands['relay'] = [str(relay)]

    def launch(name):
        child_env = env
        if name in ('motor', 'headless'):
            child_env = dict(env, TARS_KEYBOARD_AUTH=keyboard_auth)
        elif name == 'bench':
            child_env = dict(env, BENCH_TOKEN=env['TARS_BENCH_TOKEN'])   # via env, not argv (ps shows argv)
        elif name == 'goto':
            preload = str(nav / 'build/loopback_bind.so')
            if not os.path.isfile(preload):
                raise RuntimeError('Missing QNX goto loopback binding library')
            child_env = dict(env, LD_PRELOAD=preload + (':' + env['LD_PRELOAD'] if env.get('LD_PRELOAD') else ''))
        children[name] = subprocess.Popen(commands[name], env=child_env, stdin=subprocess.DEVNULL,
                                          cwd=cwds.get(name))
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
        for port in (5555, *phone_ports, *((5570, 5571, 5572) if 'goto' in commands else ()),
                     *((5574,) if 'voice' in commands else ()), *((5575,) if 'relay' in commands else ()),
                     *((int(env.get('TARS_BENCH_PORT', '8770')),) if 'bench' in commands else ())):
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
                if running and 'usbmuxd' in children and time.monotonic() - last_probe >= 10:
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
    if command == 'stop':
        STATE.mkdir(parents=True, exist_ok=True)
        (STATE / 'paused').touch()
    elif command == 'start':
        (STATE / 'paused').unlink(missing_ok=True)
    if command == 'run':
        serve()
    elif command == 'start':
        try:
            print('Already running: ' + request('status'))
            return
        except OSError:
            pass
        if os.environ.get('TARS_DEMO') == '1':
            problems = demo_preflight()
            if problems:
                raise SystemExit('TARS_DEMO=1 refused (nothing started):\n  ' + '\n  '.join(problems))
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
        raise SystemExit('Usage: service.py start|stop|status|gui|headless\n'
                         'Wi-Fi phone: echo <phone-ip> > state/phone-host (rm it for USB), then restart')


if __name__ == '__main__':
    main()
