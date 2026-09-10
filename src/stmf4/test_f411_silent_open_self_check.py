#!/usr/bin/env python3
from pathlib import Path
s=(Path(__file__).resolve().parent/'src/stmf4_hmi_bridge.cpp').read_text()
for tok in ['serial_opened_at_', 'first response timeout', 'silent_open_failures_', 'last_rx_.time_since_epoch().count() == 0', 'closeSerial("first response timeout")']:
    if tok not in s: raise SystemExit('FAIL missing '+tok)
print('F411_SILENT_OPEN_SELF_CHECK_PASS')
