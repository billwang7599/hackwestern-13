#!/bin/sh
# Launch on the Pi itself; USB bridge and both iproxy processes must be running.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec python3 "$HERE/dashboard.py" --paused "$@"
