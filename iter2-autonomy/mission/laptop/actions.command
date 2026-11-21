#!/bin/sh
# Open directly in Terminal, or pass a different user@host on the command line.
set -eu
cd "$(dirname "$0")"
exec ssh -o ControlMaster=auto -o ControlPath=/tmp/hw13-actions-%C -o ServerAliveInterval=15 -o ServerAliveCountMax=3 "${1:-qnxuser@172.23.43.185}" 'python3 -u -' < follow_actions.py
