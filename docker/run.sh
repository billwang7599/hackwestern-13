#!/usr/bin/env bash
# QNX build container helper (run on the Mac).
#
#   docker/run.sh build          # build the image (once)
#   docker/run.sh                # interactive shell, QNX env sourced, repo at its Mac path
#   docker/run.sh make -C pi     # run one command in the container
#
# Env overrides:
#   QNX_SDP_HOST  path to SDP on the Mac (default ~/qnx800)
#   QNX_LICENSE   path to license dir on the Mac (default ~/.qnx)
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
IMAGE=tars-qnx800
QNX_SDP_HOST="${QNX_SDP_HOST:-$HOME/qnx800}"
QNX_LICENSE="${QNX_LICENSE:-$HOME/.qnx}"

if [ "${1:-}" = "build" ]; then
    exec docker build --platform linux/amd64 -t "$IMAGE" "$HERE"
fi

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo ">> image $IMAGE missing, building..."
    docker build --platform linux/amd64 -t "$IMAGE" "$HERE"
fi

[ -d "$QNX_SDP_HOST" ] || { echo "!! SDP not found at $QNX_SDP_HOST" >&2; exit 1; }

# Mount the repo AND the SDP at the SAME absolute paths as on the Mac:
#  - qnxsdp-env.sh may contain the absolute install path
#  - compile_commands.json generated in here is then valid for clangd on the Mac
MOUNTS=(-v "$REPO:$REPO" -v "$QNX_SDP_HOST:$QNX_SDP_HOST")
# qcc checks the license in ~/.qnx; mount it at root's home in the container.
[ -d "$QNX_LICENSE" ] && MOUNTS+=(-v "$QNX_LICENSE:/root/.qnx")
# Let `make deploy` inside the container use your Mac's SSH keys (read-only).
[ -d "$HOME/.ssh" ] && MOUNTS+=(-v "$HOME/.ssh:/root/.ssh:ro")

TTY=()
[ -t 0 ] && TTY=(-it)
ENVS=(-e QNX_SDP="$QNX_SDP_HOST")
[ -n "${PI_HOST:-}" ] && ENVS+=(-e PI_HOST="$PI_HOST")

[ $# -eq 0 ] && set -- bash
exec docker run --rm ${TTY[@]+"${TTY[@]}"} --platform linux/amd64 \
    "${MOUNTS[@]}" -w "$REPO" \
    "${ENVS[@]}" \
    "$IMAGE" "$@"
