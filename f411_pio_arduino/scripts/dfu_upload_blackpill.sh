#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DFU="$ROOT_DIR/.pio/tools/dfu-util-blackpill/bin/dfu-util"
IMAGE="${1:-}"
[[ -f "$IMAGE" ]] || { echo "Firmware not found: $IMAGE" >&2; exit 2; }
"$ROOT_DIR/scripts/build_dfu_util_blackpill.sh"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
MANIFEST="$TMP/manifest.bin"; ZERO="$TMP/manifest_invalid.bin"; READBACK="$TMP/readback.bin"
python3 "$ROOT_DIR/scripts/make_app_manifest.py" "$IMAGE" "$MANIFEST"
python3 - <<PY
from pathlib import Path
Path("$ZERO").write_bytes(bytes(32))
PY
SIZE=$(stat -c %s "$IMAGE")
echo "[USB-DFU] transactional update: invalidate -> app -> readback -> manifest"
"$DFU" -a 0 -d 0483:df11 -s 0x08060000 -D "$ZERO"
"$DFU" -a 0 -d 0483:df11 -s 0x08008000 -D "$IMAGE"
"$DFU" -a 0 -d 0483:df11 -s "0x08008000:${SIZE}" -U "$READBACK"
cmp "$IMAGE" "$READBACK"
echo "[USB-DFU] app readback verified (${SIZE} bytes)"
"$DFU" -a 0 -d 0483:df11 -s 0x08060000:leave -D "$MANIFEST"
echo "[USB-DFU] manifest committed last; bootloader may start app"
