#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DFU="$ROOT_DIR/.pio/tools/dfu-util-blackpill/bin/dfu-util"
IMAGE="${1:-}"
[[ -f "$IMAGE" ]] || { echo "Firmware not found: $IMAGE" >&2; exit 2; }
"$ROOT_DIR/scripts/build_dfu_util_blackpill.sh"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
MANIFEST="$TMP/manifest.bin"; ZERO="$TMP/manifest_invalid.bin"; READBACK="$TMP/readback.bin"; RAW="$TMP/app_raw.bin"
python3 - <<PY2
from pathlib import Path
d=Path("$IMAGE").read_bytes()
if len(d)>=16 and d[-8:-5] == b"UFD" and d[-5] == 16:
    d=d[:-16]
Path("$RAW").write_bytes(d)
print(f"[USB-DFU] normalized raw app bytes={len(d)}")
PY2
python3 "$ROOT_DIR/scripts/make_app_manifest.py" "$RAW" "$MANIFEST"
python3 - <<PY
from pathlib import Path
Path("$ZERO").write_bytes(bytes(32))
PY
SIZE=$(stat -c %s "$RAW")
echo "[USB-DFU] transactional update: invalidate -> app -> readback -> manifest"
"$DFU" -a 0 -d 0483:df11 -s 0x08060000 -D "$ZERO"
"$DFU" -a 0 -d 0483:df11 -s 0x08008000 -D "$RAW"
"$DFU" -a 0 -d 0483:df11 -s "0x08008000:${SIZE}" -U "$READBACK"
cmp "$RAW" "$READBACK"
echo "[USB-DFU] app readback verified (${SIZE} bytes)"
"$DFU" -a 0 -d 0483:df11 -s 0x08060000 -D "$MANIFEST"
"$DFU" -a 0 -d 0483:df11 -s 0x08000000:4:leave -U "$TMP/bootword.bin"
echo "[USB-DFU] manifest committed last; leave forced through bootloader @0x08000000"
