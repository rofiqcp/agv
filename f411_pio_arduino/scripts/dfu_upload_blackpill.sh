#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DFU="$ROOT_DIR/.pio/tools/dfu-util-blackpill/bin/dfu-util"
IMAGE="${1:-}"
[[ -f "$IMAGE" ]] || { echo "Firmware not found: $IMAGE" >&2; exit 2; }
"$ROOT_DIR/scripts/build_dfu_util_blackpill.sh"
TMP="$(mktemp -d)"
cleanup() { rm -rf "$TMP"; }
trap cleanup EXIT INT TERM
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

run_dfu() {
  local label="$1"; shift
  local attempt rc=1
  for attempt in 1 2 3; do
    echo "[USB-DFU] ${label} attempt=${attempt}/3"
    if "$DFU" "$@"; then return 0; else rc=$?; fi
    if (( attempt < 3 )); then
      sleep "0.$((attempt * 3))"
    fi
  done
  echo "[USB-DFU] ${label} failed after retries rc=${rc}" >&2
  return "$rc"
}

echo "[USB-DFU] transactional update: invalidate -> app -> readback -> commit manifest -> leave bootloader -> CDC verify"
run_dfu invalidate-manifest -a 0 -d 0483:df11 -s 0x08060000 -D "$ZERO"
run_dfu write-app -a 0 -d 0483:df11 -s 0x08008000 -D "$RAW"
run_dfu readback-app -a 0 -d 0483:df11 -s "0x08008000:${SIZE}" -U "$READBACK"
cmp "$RAW" "$READBACK"
echo "[USB-DFU] app readback verified (${SIZE} bytes)"
# Commit validity metadata LAST, but do not use its address as the DfuSe :leave
# address. STM32 ROM DFU uses the current DfuSe address as the jump target.
run_dfu commit-manifest -a 0 -d 0483:df11 -s 0x08060000 -D "$MANIFEST"

CDC="/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_CDC_in_FS_Mode_338133833134-if00"
leave_bootloader() {
  local attempt rc=1 tick
  for attempt in 1 2 3; do
    echo "[USB-DFU] leave-via-bootloader attempt=${attempt}/3 address=0x08000000"
    set +e
    # DfuSe command mode: SET_ADDRESS(0x08000000) + zero-length DNLOAD leave.
    # No -D/-U file is supplied, therefore bootloader flash is never rewritten.
    "$DFU" -a 0 -d 0483:df11 -s 0x08000000:leave
    rc=$?
    set -e
    if (( rc == 0 )); then return 0; fi
    for tick in $(seq 1 30); do
      if [[ -e "$CDC" ]]; then
        echo "[USB-DFU] leave returned rc=${rc}, but runtime CDC is back; accepting reset success"
        return 0
      fi
      sleep 0.10
    done
    # If DFU is already gone, do not blindly issue another command to a new device.
    if ! lsusb -d 0483:df11 >/dev/null 2>&1; then
      echo "[USB-DFU] ROM DFU disappeared after leave; waiting for runtime CDC"
      return 0
    fi
    if (( attempt < 3 )); then sleep "0.$((attempt * 3))"; fi
  done
  echo "[USB-DFU] leave-via-bootloader failed after retries rc=${rc}" >&2
  return "$rc"
}
leave_bootloader

CDC_OK=0
for _ in $(seq 1 150); do
  if [[ -e "$CDC" ]]; then CDC_OK=1; break; fi
  sleep 0.10
done
if (( CDC_OK == 0 )); then
  echo "[USB-DFU] ERROR: flash verified but F411 CDC did not return after DFU leave/reset" >&2
  echo "[USB-DFU] Recovery: press NRST once (do not hold BOOT0); future uploads use DNLOAD+leave." >&2
  exit 12
fi
echo "[USB-DFU] manifest committed last; DFU reset complete; CDC returned"
