#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DIR="$ROOT/torch/checkpoints"
FILE="$DIR/ssdlite320_mobilenet_v3_large_coco-a79551df.pth"
URL="https://download.pytorch.org/models/ssdlite320_mobilenet_v3_large_coco-a79551df.pth"
SHA="a79551df90c79834bcd3bb3845ef9d966b5449a3a9b2833ae8404778ca5d65d2"
mkdir -p "$DIR"
if [[ -f "$FILE" ]] && [[ "$(sha256sum "$FILE" | awk '{print $1}')" == "$SHA" ]]; then
  echo "Semantic COCO model already valid: $FILE"
  exit 0
fi
tmp="$FILE.part"
rm -f "$tmp"
curl -fL --retry 3 --progress-bar "$URL" -o "$tmp"
actual="$(sha256sum "$tmp" | awk '{print $1}')"
[[ "$actual" == "$SHA" ]] || { echo "SHA256 mismatch: $actual" >&2; rm -f "$tmp"; exit 1; }
mv "$tmp" "$FILE"
echo "Semantic COCO model ready: $FILE"
