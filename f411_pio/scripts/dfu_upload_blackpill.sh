#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILDER="$ROOT_DIR/scripts/build_dfu_util_blackpill.sh"
DFU="$ROOT_DIR/.pio/tools/dfu-util-blackpill/bin/dfu-util"
IMAGE="${1:-}"

[[ -n "$IMAGE" ]] || { echo "Usage: $0 <firmware.bin>" >&2; exit 2; }
[[ -f "$IMAGE" ]] || { echo "Firmware not found: $IMAGE" >&2; exit 2; }

"$BUILDER"

echo "[USB-DFU] Uploading with STM32F411 poll-timeout workaround"
exec "$DFU" -a 0 -d 0483:df11 -s 0x08000000:leave -D "$IMAGE"
