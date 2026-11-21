#!/bin/sh
# Start tars_voice on the Pi (QNX) and keep it running: restart after a crash
# with a 1 s .. 10 s backoff. Logs to logs/voice.log. Normal priority on
# purpose: whisper must never starve tars_motor's SCHED_RR control loop.
#
#   ./start_voice.sh            run in the foreground (Ctrl-C stops)
#   ./start_voice.sh &          background (e.g. from the boot script)
#   ./start_voice.sh stop       stop a running one
#
# Boot: add this line to the image's startup script (for example
# /etc/rc.d/rc.local, or the QSTI image's post-startup hook), AFTER tars_motor:
#   (cd /data/home/qnxuser/voice && ./start_voice.sh >/dev/null 2>&1 &)
# NOT YET RUN on QNX: plain POSIX sh only, but which boot file your image uses is unverified.
cd "$(dirname "$0")" || exit 1
PIDFILE=logs/start_voice.pid
mkdir -p logs

if [ "${1:-}" = "stop" ]; then
    [ -f "$PIDFILE" ] && kill "$(cat "$PIDFILE")" 2>/dev/null
    slay -f tars_voice 2>/dev/null || pkill tars_voice 2>/dev/null
    rm -f "$PIDFILE"
    echo "stopped"
    exit 0
fi

echo $$ > "$PIDFILE"
trap 'rm -f "$PIDFILE"; kill $child 2>/dev/null; exit 0' INT TERM
delay=1
while :; do
    start=$(date +%s)
    echo "$(date) starting tars_voice" >> logs/voice.log
    ./tars_voice -c voice.conf >> logs/voice.log 2>&1 &
    child=$!
    wait $child
    rc=$?
    echo "$(date) tars_voice exited ($rc); restarting in ${delay}s" >> logs/voice.log
    [ $(( $(date +%s) - start )) -gt 30 ] && delay=1        # ran a while: reset backoff
    sleep $delay
    delay=$(( delay * 2 )); [ $delay -gt 10 ] && delay=10
done
