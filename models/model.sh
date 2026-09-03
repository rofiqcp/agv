#!/usr/bin/env bash
set -euo pipefail

MODEL_URL="https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt"
EXPECTED_SHA256="f2a8c8374203ae3e67ff9c184e931f763957de92a993b23269e4e721627f1f8c"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODEL_PATH="${SCRIPT_DIR}/yolopv2.pt"
TMP_PATH="${MODEL_PATH}.part"
FORCE="${1:-}"

verify_model() {
  [[ -f "$MODEL_PATH" ]] || return 1
  local actual
  actual="$(sha256sum "$MODEL_PATH" | awk '{print $1}')"
  [[ "$actual" == "$EXPECTED_SHA256" ]]
}

if [[ "$FORCE" != "--force" ]] && verify_model; then
  echo "YOLOPv2 model already valid: $MODEL_PATH"
  exit 0
fi

rm -f "$TMP_PATH"
echo "Downloading official YOLOPv2 model..."
if command -v curl >/dev/null 2>&1; then
  curl -fL --retry 3 --progress-bar "$MODEL_URL" -o "$TMP_PATH"
elif command -v wget >/dev/null 2>&1; then
  wget --tries=3 --show-progress -O "$TMP_PATH" "$MODEL_URL"
else
  echo "ERROR: curl or wget is required." >&2
  exit 1
fi

actual_sha256="$(sha256sum "$TMP_PATH" | awk '{print $1}')"
if [[ "$actual_sha256" != "$EXPECTED_SHA256" ]]; then
  echo "ERROR: SHA256 mismatch." >&2
  echo "Expected: $EXPECTED_SHA256" >&2
  echo "Actual:   $actual_sha256" >&2
  rm -f "$TMP_PATH"
  exit 1
fi

mv "$TMP_PATH" "$MODEL_PATH"
echo "Model ready: $MODEL_PATH"
echo "SHA256: $EXPECTED_SHA256"
