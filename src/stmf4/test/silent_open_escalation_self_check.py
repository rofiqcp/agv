#!/usr/bin/env python3
from pathlib import Path
s=(Path(__file__).parents[1]/'src/stmf4_hmi_bridge.cpp').read_text()
for tok in ['serial_opened_at_','silent_open_failures_ >= 3U','sendLine("USB:RECOVER", 0)','std::chrono::seconds(8)','/stmf4/usb/status','USB:STATUS','usb_recovery_requests_']:
    assert tok in s, tok
assert 'closeSerial("first response timeout")' in s
print('F411_SILENT_OPEN_ESCALATION_SELF_CHECK_PASS')
