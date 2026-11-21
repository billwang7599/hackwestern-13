"""Boot supervisor for the QNX headless stack. Run as qnxuser."""
import fcntl
import json
import os
import signal
import subprocess
import sys
import time

from service import HERE, STATE, request

def main():
    STATE.mkdir(parents=True, exist_ok=True)
    lock = (STATE / 'watchdog.lock').open('a')
    try:
        fcntl.lockf(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        return
    env = dict(os.environ, TARS_MOTOR_BACKEND='uno',
               TARS_MOTOR_DEVICE=str(STATE / 'uno-serial'))
    proc = None
    misses = 0
    while True:
        time.sleep(3)
        if (STATE / 'paused').exists():
            continue
        try:
            healthy = 'headless' in json.loads(request('status'))
        except (OSError, ValueError):
            healthy = False
        misses = 0 if healthy else misses + 1
        if misses < 3:
            continue
        if proc is not None:
            # This session contains only our service and its workers.
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try: os.killpg(proc.pid, sig)
                except ProcessLookupError: break
                time.sleep(2)
            proc.poll()
        print('Robot service unavailable for three checks; starting', flush=True)
        with (STATE / 'headless-service.log').open('ab') as log:
            proc = subprocess.Popen([sys.executable, str(HERE / 'service.py'), 'run'],
                                    env=env, start_new_session=True, stdout=log, stderr=log)
        misses = 0

if __name__ == '__main__':
    main()
