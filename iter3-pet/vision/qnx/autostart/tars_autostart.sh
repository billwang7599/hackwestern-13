#!/bin/sh
# TARS on QNX, no laptop: start the camera service (optional), tars_motor and the
# gesture app, restart each if it crashes, log to $LOG_DIR. POSIX sh (QNX ksh ok).
#
#   sh tars_autostart.sh start      # (what the boot hook runs) returns immediately
#   sh tars_autostart.sh stop
#   sh tars_autostart.sh status
#   sh tars_autostart.sh run        # foreground, for debugging (Ctrl-C stops all)
#
# Settings: tars.env next to this script (cp tars.env.example tars.env; deploy.sh
# never overwrites it). Boot hook: install_autostart.sh.
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ -f "$HERE/tars.env" ]; then . "$HERE/tars.env"; else . "$HERE/tars.env.example"; fi
PIDDIR="$LOG_DIR/pids"
mkdir -p "$LOG_DIR" "$PIDDIR"

log() { echo "$(date '+%Y-%m-%d %H:%M:%S') [autostart] $*" >> "$LOG_DIR/autostart.log"; }

rotate() {   # keep each log under ~2 MB: file -> file.1
    f="$1"
    if [ -f "$f" ] && [ "$(wc -c < "$f")" -gt 2000000 ]; then mv -f "$f" "$f.1"; fi
}

# supervise <name> <command...>: run forever, restart on exit with backoff
supervise() {
    name="$1"; shift
    backoff=1
    while :; do
        rotate "$LOG_DIR/$name.log"
        echo "$(date '+%Y-%m-%d %H:%M:%S') ---- start: $*" >> "$LOG_DIR/$name.log"
        started=$(date +%s)
        [ -f "$PIDDIR/stopping" ] && break
        "$@" < /dev/null >> "$LOG_DIR/$name.log" 2>&1 &
        echo $! > "$PIDDIR/$name.child"
        wait $!
        rc=$?
        ran=$(( $(date +%s) - started ))
        [ -f "$PIDDIR/stopping" ] && break
        log "$name exited rc=$rc after ${ran}s; restarting in ${backoff}s"
        if [ "$ran" -gt 60 ]; then backoff=1; fi
        sleep "$backoff"
        [ -f "$PIDDIR/stopping" ] && break
        backoff=$(( backoff * 2 )); [ "$backoff" -gt 10 ] && backoff=10
    done
}

gesture_cmd() {
    if [ "$GESTURE_APP" = cpp ]; then
        echo "$VISION/qnx/cpp/tars_gesture_cpu --model=$VISION/models/hand_landmarker.task --conf=$VISION/gesture.conf --learned=$LEARNED $GESTURE_ARGS"
    else
        echo "python3 -u $VISION/qnx/python/tars_gesture.py --learned $LEARNED --console off $GESTURE_ARGS"
    fi
}

start_all() {
    rm -f "$PIDDIR/stopping"
    sleep "${BOOT_DELAY:-0}"
    if [ -n "$CAMERA_CMD" ]; then
        if pidin -f a 2>/dev/null | grep -q '[s]ensor'; then
            log "camera service already running"
        else
            log "starting camera service: $CAMERA_CMD"
            $CAMERA_CMD >> "$LOG_DIR/camera.log" 2>&1
            sleep 2
        fi
    fi
    [ -x "$MOTOR_BIN" ] || log "WARNING: $MOTOR_BIN missing (make -C pi deploy)"
    supervise motor $MOTOR_BIN $MOTOR_ARGS &
    echo $! > "$PIDDIR/motor.sup"
    sleep 1                                  # let tars_motor open :5555 (the app retries anyway)
    # shellcheck disable=SC2046
    supervise gesture $(gesture_cmd) &
    echo $! > "$PIDDIR/gesture.sup"
    log "started (motor sup $(cat "$PIDDIR/motor.sup"), gesture sup $(cat "$PIDDIR/gesture.sup"))"
}

stop_all() {
    touch "$PIDDIR/stopping"
    for n in gesture motor; do
        for k in child sup; do
            f="$PIDDIR/$n.$k"
            [ -f "$f" ] && kill "$(cat "$f")" 2>/dev/null
            rm -f "$f"
        done
    done
    log "stopped"
}

case "${1:-start}" in
    start)
        if [ -f "$PIDDIR/motor.sup" ] && kill -0 "$(cat "$PIDDIR/motor.sup")" 2>/dev/null; then
            echo "already running (sh $0 stop first)"; exit 0
        fi
        log "boot start"
        start_all
        echo "TARS started; logs in $LOG_DIR" ;;
    run)
        trap 'stop_all; exit 0' INT TERM
        start_all
        tail -f "$LOG_DIR/gesture.log" ;;
    stop)  stop_all; echo "stopped" ;;
    status)
        for n in motor gesture; do
            if [ -f "$PIDDIR/$n.child" ] && kill -0 "$(cat "$PIDDIR/$n.child")" 2>/dev/null; then
                echo "$n: running (pid $(cat "$PIDDIR/$n.child"))"
            else echo "$n: NOT running"; fi
        done
        tail -n 5 "$LOG_DIR/gesture.log" 2>/dev/null ;;
    *) echo "usage: $0 start|stop|status|run"; exit 2 ;;
esac
