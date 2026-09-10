#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${AGV_ROOT:-$(cd -- "$SCRIPT_DIR/../../.." && pwd)}"
SRC="$ROOT/src/stmf4/systemd/agv-esc.service"
DST="$HOME/.config/systemd/user/agv-esc.service"
mkdir -p "$(dirname "$DST")"
TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT
python3 - "$SRC" "$TMP" "$ROOT" <<'PYGEN'
from pathlib import Path
import sys
src, dst, root = map(Path, sys.argv[1:])
text = src.read_text()
if '@AGV_ROOT@' not in text:
    raise SystemExit('service template missing @AGV_ROOT@')
dst.write_text(text.replace('@AGV_ROOT@', str(root.resolve())))
PYGEN
install -m 0644 "$TMP" "$DST"
systemctl --user daemon-reload
systemctl --user enable agv-esc.service
echo "Installed user service: $DST"
echo "AGV_ROOT: $ROOT"
echo "Note: starts at user login; system-wide boot-before-login requires administrator-managed service or linger."
