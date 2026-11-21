"""Host integration test: python3 iter1-wasd/pi/test_imu_receiver.py /path/to/imu_receiver."""
import socket
import subprocess
import sys
import time

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
process = subprocess.Popen([sys.argv[1], str(port)], stdout=subprocess.PIPE, text=True)
try:
    assert 'C++ receiver running' in process.stdout.readline()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for packet in (b'garbage', b'IMU1 0 1 0 0 99999 0 0 0 0 0\n',
                       b'IMU1 1 1 0 0 0 0 4096 -12 0 7\n'):
            sock.sendto(packet, ('127.0.0.1', port))
    time.sleep(0.4)
finally:
    process.terminate()
    output, _ = process.communicate(timeout=3)
assert 'samples=1 seq=1 accel=0,0,4096 gyro=-12,0,7' in output, output
assert 'samples=2' not in output, output
assert 'STALE:' in output, output
print('PASS: valid telemetry, malformed/range rejection, stale timeout')
