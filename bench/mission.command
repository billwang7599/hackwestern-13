#!/bin/bash
# Double-click (Finder) or run: the bench as mission control for the REAL robot.
#   bench/mission.command [pi-host[:5575]]       default $TARS_HOST or tars.local
# The relay key: $MISSION_KEY, else ~/.tars_mission_key (one line), else asked here (not echoed).
# Then open http://127.0.0.1:8770 (opened for you). Ctrl-C to quit; the robot carries on without it.
set -eu
cd "$(dirname "$0")/.."
host="${1:-${TARS_HOST:-tars.local}}"
if [ -z "${MISSION_KEY:-}" ] && [ -r "$HOME/.tars_mission_key" ]; then
    MISSION_KEY="$(head -n1 "$HOME/.tars_mission_key")"
fi
if [ -z "${MISSION_KEY:-}" ]; then
    read -r -s -p "mission_relay key for $host: " MISSION_KEY; echo
fi
export MISSION_KEY
( sleep 2; open "http://127.0.0.1:8770/" 2>/dev/null || true ) &
exec python3 bench/bench.py --target pi --relay "$host"
