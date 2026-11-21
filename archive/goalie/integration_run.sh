#!/usr/bin/env bash
# Full no-hardware loop: vision/fake_ball.py (fake camera + fake paddle)
#   -> TCP -> REAL pi/goalie_motor (host build, stub backend) -> motor duties
#   -> back into the fake paddle. Writes docs/integration_run.txt.
#
#   cd goalie && ./integration_run.sh
#
# Needs: make -C pi host (done here), vision/.venv (see vision/README.md).
set -u
cd "$(dirname "$0")"
OUT=docs/integration_run.txt
PY=vision/.venv/bin/python
BIN=pi/build/goalie_motor_host
LOGDIR=pi/build
make -s -C pi host >/dev/null || exit 1
[ -x "$PY" ] || { echo "missing $PY (python3 -m venv vision/.venv && vision/.venv/bin/pip install -r vision/requirements.txt)"; exit 1; }

# Fake paddle ~ the sim's pivot paddle at 100 RPM: 12 cm arm -> tip speed
# 100*2pi/60*0.12 = 1.26 m/s, motor time constant ~45 ms (sim tuned()).
VMAX=1.26
TAU=0.045
SRV_FLAGS="--rpm 100 --tau 0.045 --delay-ms 40"   # fake camera latency is 40 ms

PORT=$(( 50000 + RANDOM % 10000 ))
SP=""
start_server() {   # $1 = log name, rest = extra flags
  local log="$LOGDIR/$1.log"; shift
  "$BIN" -b stub -p "$PORT" $SRV_FLAGS "$@" >"$log" 2>&1 &
  SP=$!
  sleep 0.4
}
stop_server() { kill -INT "$SP" 2>/dev/null; wait "$SP" 2>/dev/null; }
trap 'stop_server' EXIT

fb() {   # fake_ball scenario: label, then fake_ball args
  local label="$1"; shift
  echo
  echo "### $label"
  echo "\$ fake_ball.py $*"
  (cd vision && .venv/bin/python fake_ball.py --host 127.0.0.1 --port "$PORT" --seed 1 "$@" 2>&1) \
    > "$LOGDIR/fb_last.txt"
  grep -E "^shot" "$LOGDIR/fb_last.txt" | head -3
  echo "  ..."
  grep -E "shots at|^server:" "$LOGDIR/fb_last.txt"
  (cd vision && .venv/bin/python - "$PORT" <<'EOF'
import sys
from goalie_link import GoalieLink
l = GoalieLink("127.0.0.1", int(sys.argv[1]))
st = l.status()
keys = ["mode", "sched", "emulated", "ctl_kind", "saves", "goals", "hits", "jit_avg_us",
        "jit_max_us", "loop_hz", "model_rpm", "kp", "kd"]
print("STATUS: " + " ".join(f"{k}={st.get(k)}" for k in keys))
l.close()
EOF
  )
}

{
echo "Beat the Goalie: integration run (no hardware)"
echo "date: $(date '+%Y-%m-%d %H:%M %Z')   host: $(uname -sm)   git: $(git describe --always --dirty 2>/dev/null)"
echo
echo "Loop: vision/fake_ball.py (30 fps, 40 ms latency, 3 mm noise, 2% dropouts, 7 cm blade)"
echo "  -> TCP F lines -> REAL C server pi/build/goalie_motor_host -b stub $SRV_FLAGS"
echo "  -> OK <target> <duty> -> fake paddle (vmax $VMAX m/s at duty 1, tau $TAU s ~ 100 RPM pivot)"
echo "Each scenario: 20 shots, same seed. LINUX = EMULATED (SCHED_OTHER + random 0-30 ms stalls)."

start_server observer
fb "A. 1.0 m/s casual roll, QNX, observer controller"        --shots 20 --speed 1.0 --mode QNX --paddle-vmax $VMAX --paddle-tau $TAU
fb "B. 2.0 m/s, QNX, observer"                              --shots 20 --speed 2.0 --mode QNX --paddle-vmax $VMAX --paddle-tau $TAU
fb "C. 2.0 m/s, LINUX (emulated), observer"                 --shots 20 --speed 2.0 --mode LINUX --paddle-vmax $VMAX --paddle-tau $TAU
fb "D. 3.0 m/s, QNX, observer"                              --shots 20 --speed 3.0 --mode QNX --paddle-vmax $VMAX --paddle-tau $TAU
fb "E. 3.0 m/s, LINUX (emulated), observer"                 --shots 20 --speed 3.0 --mode LINUX --paddle-vmax $VMAX --paddle-tau $TAU
fb "F. slow motor (~30 RPM: vmax 0.38 m/s), 1.5 m/s, QNX"   --shots 20 --speed 1.5 --mode QNX --paddle-vmax 0.38 --paddle-tau $TAU
stop_server

echo
echo "### Sample of stub motor commands (server log, scenario A start; the stub prints M only when it changes)"
grep -E "^\[stub\] (M|C|N|S)" "$LOGDIR/observer.log" | head -25
echo "... $(grep -c '^\[stub\] M' "$LOGDIR/observer.log") distinct M lines in total"
echo
echo "### Server events (saves/goals/watchdog/mode), first 15"
grep -E "^\[(game|ctl)\]" "$LOGDIR/observer.log" | head -15

start_server pd --ctl pd
fb "G. legacy camera-PD controller (--ctl pd), 2.0 m/s, QNX"  --shots 20 --speed 2.0 --mode QNX --paddle-vmax $VMAX --paddle-tau $TAU
fb "H. legacy camera-PD controller (--ctl pd), 3.0 m/s, QNX"  --shots 20 --speed 3.0 --mode QNX --paddle-vmax $VMAX --paddle-tau $TAU
stop_server

cat <<'NOTES'

### How to read this
- Real-time run over TCP (server threads, OS scheduling, Python timing): the
  same seed does NOT give identical results run to run. Expect +-1-2 shots per
  scenario; a 90% vs 100% difference at 2 m/s is within that noise.
- "N shots at": fake_ball's ground truth. "server:"/"STATUS:": the C server's
  own counters (vision-only save detection: the fake world sends no Uno HIT).
  At 3 m/s the server can miss a save (ball moves 10 cm per frame, so the
  bounce-back is seen late); on hardware the Uno's accelerometer HIT covers it.
- The fake paddle is a linear first-order motor with a 7 cm blade and no
  deadband/friction; the C observer assumes the sim's pivot model (friction
  offset 0.10, deadband comp 0.12), so it is deliberately a bit mismatched.
  The save rates show the software loop works end to end; they are NOT a
  hardware prediction. Use goalie/sim with your measured RPM for that.
- LINUX mode is an emulation (SCHED_OTHER + injected 0-30 ms stalls).
NOTES
} 2>&1 | tee "$OUT"
echo
echo "wrote $OUT"
