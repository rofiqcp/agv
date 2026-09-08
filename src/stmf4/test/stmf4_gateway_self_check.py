#!/usr/bin/env python3
"""Static contract for the F411 single-USB hardware gateway."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parents[1]
bridge = (ROOT / 'src/stmf4_hmi_bridge.cpp').read_text(encoding='utf-8')
hmi = (yaml.safe_load((ROOT / 'config/hmi.yaml').read_text()) or {})['stmf4_hmi_bridge']['ros__parameters']
vesc_h = (WS / 'f411_pio_arduino/src/VescGateway.h').read_text(encoding='utf-8')
vesc_cpp = (WS / 'f411_pio_arduino/src/VescGateway.cpp').read_text(encoding='utf-8')
neo_h = (WS / 'f411_pio_arduino/src/Neo3Sensors.h').read_text(encoding='utf-8')
neo_cpp = (WS / 'f411_pio_arduino/src/Neo3Sensors.cpp').read_text(encoding='utf-8')
main = (WS / 'f411_pio_arduino/src/main.cpp').read_text(encoding='utf-8')

def require(ok, msg):
    if not ok:
        raise AssertionError(msg)

# Physical pin contract: no collision between VESC, NEO3 UART and NEO3 I2C.
require('Uart uart_{PB7, PB6}' in vesc_h, 'VESC must use F411 USART1 RX=PB7 TX=PB6')
require('Uart gnss_serial_{PA3, PA2}' in neo_h, 'NEO3 GNSS must use USART2 RX=PA3 TX=PA2')
require('Wire.setSDA(PB9)' in neo_cpp and 'Wire.setSCL(PB8)' in neo_cpp,
        'NEO3 IST8310 I2C must use SDA=PB9 SCL=PB8')
require('istWriteAt(addr, IST8310_CTRL2, 0x01)' in neo_cpp and
        'IST8310_ADDR_MIN = 0x0C' in neo_h and 'IST8310_ADDR_MAX = 0x0F' in neo_h,
        'IST8310 must reset/probe the PX4-observed 0x0C..0x0F hot-plug address range')
require('IST8310J_WHOAMI = 0xA3' in neo_h and
        'who == IST8310_WHOAMI || who == IST8310J_WHOAMI' in neo_cpp,
        'IST8310 probe must accept both IST8310 and IST8310J device IDs')
for token in ('IST8310_MAX_RAW_XY', 'IST8310_MAX_RAW_Z', 'const int16_t z = static_cast<int16_t>(-z_sensor)',
              'avg != 0x24', 'pd != 0xC0'):
    require(token in neo_cpp, f'IST8310 robustness contract missing: {token}')
import re
f411_m = re.search(r'kBaud\s*=\s*(115200|1000000)', vesc_h)
f103_m = re.search(r'F103_VESC_UART_BAUD\s+(115200|1000000)u', (WS / 'hoverboard-firmware-hack-FOC/Src/vesc/f103_boot_layout.h').read_text(encoding='utf-8'))
require(f411_m is not None and f103_m is not None, 'F411/F103 VESC baud must be explicit 115200 or 1000000')
require(f411_m.group(1) == f103_m.group(1), 'F411 and F103 internal VESC UART baud must match')
require('gVesc.begin()' in main and 'gVesc.poll()' in main, 'F411 VESC gateway lifecycle missing')
require('VESC:MODE:RUNTIME' in vesc_cpp and 'VESC:MODE:MAINTENANCE' in vesc_cpp,
        'F411 VESC runtime/maintenance ownership missing')

# ROS must be the single USB CDC owner and safely multiplex raw VESC bytes.
for token in ('/stmf4/vesc/runtime_tx', '/stmf4/vesc/maintenance_tx', '/stmf4/vesc/rx',
              '/stmf4/vesc/mode', '/stmf4/vesc/status', '/stmf4/vesc/connected',
              '/stmf4/vesc/mode', 'VESC:STATUS'):
    require(token in bridge, f'ROS F411 VESC bridge contract missing: {token}')
require('mode != "RUNTIME" && mode != "NORMAL" && mode != "MAINTENANCE"' in bridge and
        'sendLine(std::string("VESC:MODE:") + route)' in bridge,
        'ROS F411 bridge must validate and forward RUNTIME/MAINTENANCE ownership dynamically')
require('VESC:MODE:RUNTIME' in vesc_cpp and 'VESC:MODE:MAINTENANCE' in vesc_cpp,
        'F411 firmware must explicitly implement both VESC ownership modes')
require('Serial.availableForWrite()' in vesc_cpp and 'usb_drop_frames_' in vesc_cpp,
        'F411 USB CDC telemetry must be bounded/nonblocking')
for token in ('kRuntimeNoValidFrameRecoverMs', 'recoverRuntimeUart', 'recovery_streak_', 'ever_valid_frame_', 'NVIC_SystemReset'):
    require(token in vesc_h + vesc_cpp, f'F411 VESC recovery contract missing: {token}')
for token in ('HardwareTimer *gAppWatchdogTimer', 'TIM11', 'APP_WATCHDOG_TIMEOUT_MS',
              'gMainLoopHeartbeatMs = HAL_GetTick()', 'stopAppWatchdog()', 'NVIC_SystemReset'):
    require(token in main, f'F411 application watchdog contract missing: {token}')
require(str(hmi.get('serial_device', '')).lower() == 'auto', 'F411 serial_device must default to fail-safe auto discovery')
require('STMICROELECTRONICS' in bridge and 'F411' in bridge and 'CDC' in bridge,
        'F411 auto-discovery identity guard missing')
require(float(hmi.get('vesc_transport_timeout_sec', 0.0)) > 0.0, 'VESC transport watchdog missing')
require(int(hmi.get('serial_baud', 0)) == 1000000, 'Mini-PC<->F411 USB CDC host setting must stay 1 Mbaud')
require(int(hmi.get('vesc_uart_baud_expected', 0)) == int(f411_m.group(1)), 'ROS expected internal VESC baud must match firmware source')
require('vesc_uart_baud_active_' in bridge and 'payload.find("baud=")' in bridge,
        'ROS bridge must adapt internal VESC pacing to F411-reported baud without changing host USB')
require('rx_count > 0UL' in bridge, 'VESC connected state must require real F103 RX traffic')
require('const bool qualified_fix = receiver_valid && coordinates_valid' in bridge,
        'M9N no-fix telemetry must separate receiver link from fusion-valid LLH')
require('quality.data[44] = qualified_fix ? 1.0 : 0.0' in bridge,
        'M9N quality fix-valid flag must use qualified_fix')
require('qualified_fix && velocity_valid' in bridge,
        'M9N velocity must never publish into ROS without qualified position fix')
require('pvt_.received_ms != 0U' in neo_cpp and 'gnssReady(uint32_t now_ms)' in neo_cpp,
        'F411 must distinguish M9N streaming/connected from GNSS ready/fix')
require('const bool pvt_stream_fresh' in neo_cpp and 'if (!pvt_stream_fresh)' in neo_cpp,
        'M9N UBX configuration retry must follow stream freshness, not GNSS fix')

# F411 first-stage recovery bootloader contract.
pio = (WS / 'f411_pio_arduino/platformio.ini').read_text(encoding='utf-8')
boot = (WS / 'f411_pio_arduino/bootloader/src/main.c').read_text(encoding='utf-8')
uploader = (WS / 'f411_pio_arduino/scripts/dfu_upload_blackpill.sh').read_text(encoding='utf-8')
require('board_build.flash_offset = 0x8000' in pio and 'board_upload.maximum_size = 393216' in pio,
        'F411 application must stay relocated behind 32-KiB first-stage bootloader')
for token in ('APP_BASE', 'MANIFEST_ADDR', 'MANIFEST_MAGIC', 'crc32_bytes', 'jump_system_dfu', 'application_valid'):
    require(token in boot, f'F411 recovery bootloader contract missing: {token}')
require('0x08008000' in uploader and '0x08060000' in uploader and 'readback verified' in uploader and
        '0x08000000:leave' in uploader, 'transactional DFU updater must preserve bootloader and verify app')

print('PASS stmf4_gateway_self_check')
print('pins: VESC PB6/PB7 | NEO3 I2C PB8/PB9 | GNSS PA2/PA3 | one F411 USB CDC')
