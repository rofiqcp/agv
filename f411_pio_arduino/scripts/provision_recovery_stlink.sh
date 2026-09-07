#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OPENOCD="$HOME/.platformio/packages/tool-openocd/bin/openocd"
BOOT="$ROOT/bootloader/.pio/build/f411_recovery_boot/firmware.bin"
APP="$ROOT/.pio/build/blackpill_f411ce_stlink/firmware.bin"
MANIFEST="$ROOT/.pio/build/blackpill_f411ce_stlink/manifest.bin"
cd "$ROOT/bootloader"; pio run -e f411_recovery_boot
cd "$ROOT"; pio run -e blackpill_f411ce_stlink
python3 scripts/make_app_manifest.py "$APP" "$MANIFEST"
echo "[STLINK] provisioning bootloader + relocated app + manifest"
"$OPENOCD" -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 500; init; reset halt; \
      flash write_image erase $BOOT 0x08000000 bin; verify_image $BOOT 0x08000000 bin; \
      flash write_image erase $APP 0x08008000 bin; verify_image $APP 0x08008000 bin; \
      flash write_image erase $MANIFEST 0x08060000 bin; verify_image $MANIFEST 0x08060000 bin; \
      reset run; shutdown"
echo "[STLINK] provision complete"
