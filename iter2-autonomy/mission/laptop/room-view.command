#!/bin/sh
# The dashboard remains private to the Pi and this laptop.
set -eu
host=${1:-qnxuser@172.23.43.185}
echo 'Room view: http://127.0.0.1:5580/  (Ctrl-C to disconnect)'
exec ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -o ServerAliveCountMax=3 \
    -L 127.0.0.1:5580:127.0.0.1:5580 "$host"
