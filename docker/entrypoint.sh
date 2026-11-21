#!/usr/bin/env bash
# Sources qnxsdp-env.sh, then runs the given command (default: bash).
set -e
if [ -f "$QNX_SDP/qnxsdp-env.sh" ]; then
    # shellcheck disable=SC1091
    . "$QNX_SDP/qnxsdp-env.sh" >/dev/null
else
    echo "!! $QNX_SDP/qnxsdp-env.sh not found." >&2
    echo "!! Install SDP 8.0 + 'QNX SDP Host Tools (Linux 64-bit)' to ~/qnx800 on the Mac" >&2
    echo "!! (or set QNX_SDP_HOST=/path/to/qnx800 before docker/run.sh)." >&2
fi
exec "$@"
