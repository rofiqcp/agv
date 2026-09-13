#!/usr/bin/env python3
from pathlib import Path
WS=Path(__file__).resolve().parents[3]
bridge=(WS/'src/stmf4/src/stmf4_hmi_bridge.cpp').read_text()
f4=(WS/'F4gateway/src/main.cpp').read_text()
web=(WS/'src/navigation/web/static/app.js').read_text()
def need(token, corpus, message=None):
    if token not in corpus:
        raise AssertionError(message or token)
for token in ('manual_command_lease_sec','manualLeaseFresh','manualInitialMotionAllowed','manual direction reversal requires explicit STOP first','manual command lease expired','last_drive_command_time_'):
    need(token, bridge)
need('manual_command_lease_sec_{0.30}', bridge, 'manual lease default must leave timer margin below 350 ms acceptance')
for token in ('gDriveStopPending','gNavStopPending','sendSafetyControlLine("CMD:DRIVE:STOP")','sendSafetyControlLine("CMD:NAV:STOP")','serviceSafetyControlTx','DRIVE_LEASE_REFRESH_MS = 100U'):
    need(token, f4)
for token in ('navigationMotionGateReady','motion_localization_ready','planning_ready','mapPoseFresh()'):
    need(token, bridge)
if 'NEO:LED:AUTO' in bridge:
    raise AssertionError('LED AUTO reconnect still present')
if 'wait_for_service(250ms)' in bridge:
    raise AssertionError('blocking Nav2 wait remains')
for token in ('hmiControlHold','setInterval','100','hmiControlRelease','visibilitychange','STEER_STOP'):
    need(token, web)
need('resetTransportEpochState', bridge)
need('neo3CommandAllowed', bridge)
need('time_standard != 2', bridge, 'GNSS timestamp must only trust UTC absolute epoch')

usb=(WS/'F4gateway/src/usb/UsbCdcPort.cpp').read_text()
critical=usb[usb.find('bool UsbCdcPort::writeLineCritical'):usb.find('#ifdef HMI_TEST_HOOKS')]
if 'HAL_Delay' in critical:
    raise AssertionError('critical USB path still blocks on HAL_Delay')
for token in ('::fsync(tmp_fd)', 'fs::rename(temp, target', '::fsync(dir_fd)', 'waypointPoseSane'):
    need(token, bridge)
print('STAGE1_SAFETY_INTEGRITY_SELF_CHECK_PASS')
