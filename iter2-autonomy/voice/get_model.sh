#!/bin/sh
# Download the whisper.cpp speech model into iter2-autonomy/voice/models/ and check its SHA-256.
#
#   iter2-autonomy/voice/get_model.sh            ggml-tiny.en-q5_1.bin (~31 MB, default, see PLAN.md)
#   iter2-autonomy/voice/get_model.sh --base     ggml-base.en-q5_1.bin (~57 MB, fallback: better, ~2-3x slower)
#
# Source: the official whisper.cpp Hugging Face repo. SHAs are the LFS oids
# published there (https://huggingface.co/ggerganov/whisper.cpp/raw/main/<file>).
# Models are not committed (see AGENTS.md).
set -eu
cd "$(dirname "$0")"
mkdir -p models

name=tiny.en-q5_1
sha=c77c5766f1cef09b6b7d47f21b546cbddd4157886b3b5d6d4f709e91e66c7c2b
if [ "${1:-}" = "--base" ]; then
    name=base.en-q5_1
    sha=4baf70dd0d7c4247ba2b81fafd9c01005ac77c2f9ef064e00dcf195d0e2fdd2f
fi
file="models/ggml-$name.bin"
url="https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-$name.bin"

sha256() {
    if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | cut -d' ' -f1
    elif command -v shasum >/dev/null 2>&1; then shasum -a 256 "$1" | cut -d' ' -f1
    else openssl dgst -sha256 "$1" | sed 's/.*= //'; fi
}

if [ -f "$file" ] && [ "$(sha256 "$file")" = "$sha" ]; then
    echo "ok: $file already present (sha256 matches)"
    exit 0
fi
echo "downloading $url"
if command -v curl >/dev/null 2>&1; then curl -fL --retry 3 -o "$file.part" "$url"
else wget -O "$file.part" "$url"; fi
got=$(sha256 "$file.part")
if [ "$got" != "$sha" ]; then
    echo "SHA-256 MISMATCH for $file: got $got, want $sha" >&2
    rm -f "$file.part"
    exit 1
fi
mv "$file.part" "$file"
echo "ok: $file ($(wc -c < "$file") bytes, sha256 verified)"
