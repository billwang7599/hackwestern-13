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
    flat_layout = HERE.parent.name == 'pi'
    motor_dir = HERE.parent if flat_layout else HERE.parents[1] / 'iter1-wasd/pi'
    phone_ports = (5557, 5558, 5559, 5561, 5562, 5563)
    if not flat_layout:
        phone_ports += (5573, 5577)    # 5573 TARS speech -> phone, 5577 phone mic (AirPods) -> tars_voice
    motor_command = [str(motor_dir / 'build/tars_motor'), '-b', backend, '-l', '-g', '5568']
    if backend != 'stub':
        device = os.environ.get('TARS_MOTOR_DEVICE')
        if not device or not (device.startswith('/dev/') or device == str(STATE / 'uno-serial')):
            raise ValueError('Hardware mode requires an explicit TARS_MOTOR_DEVICE=/dev/...')
        motor_command += ['-d', device]
    print('Motor backend: ' + backend + (' (no physical motor output)' if backend == 'stub' else ''), flush=True)
    commands = {
        'usbmuxd': ['sudo', '-n', 'sh', str(USB / 'run-usbmuxd.sh')],
        **{f'proxy{p}': [str(USB / 'install/bin/iproxy'), '-l', '-s', '127.0.0.1', f'{p}:{p}'] for p in phone_ports},
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
    relay = HERE.parent / 'mission/build/mission_relay'
    if env.get('MISSION_KEY'):
        if not relay.is_file():
            raise ValueError(f'MISSION_KEY set but no {relay} (make -C iter2-autonomy/mission native)')
        commands['relay'] = [str(relay)]

    def launch(name):
        child_env = env
        if name in ('motor', 'headless'):
            child_env = dict(env, TARS_KEYBOARD_AUTH=keyboard_auth)
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
                     *((5574,) if 'voice' in commands else ()), *((5575,) if 'relay' in commands else ())):
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
