#!/bin/sh
# Fetch the pinned whisper.cpp source into voice/third_party/whisper.cpp (not committed).
# Pinned: v1.9.5 = commit d1be6fde11ac6e0407606b4e42fe72d34add8037.
set -eu
cd "$(dirname "$0")"
TAG=v1.9.5
COMMIT=d1be6fde11ac6e0407606b4e42fe72d34add8037
DIR=third_party/whisper.cpp
if [ -d "$DIR/.git" ]; then
    have=$(git -C "$DIR" rev-parse HEAD)
    if [ "$have" = "$COMMIT" ]; then echo "ok: $DIR at $TAG ($COMMIT)"; exit 0; fi
    echo "$DIR is at $have, want $COMMIT; delete it and rerun" >&2; exit 1
fi
mkdir -p third_party
git clone --depth 1 --branch "$TAG" https://github.com/ggml-org/whisper.cpp.git "$DIR"
have=$(git -C "$DIR" rev-parse HEAD)
[ "$have" = "$COMMIT" ] || { echo "tag $TAG moved: got $have, want $COMMIT" >&2; exit 1; }
echo "ok: $DIR at $TAG ($COMMIT)"
