#!/usr/bin/env python3
"""Static contract for the F411 single-USB hardware gateway."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parents[1]
bridge = (ROOT / 'src/stmf4_hmi_bridge.cpp').read_text(encoding='utf-8')
hmi = (yaml.safe_load((ROOT / 'config/hmi.yaml').read_text()) or {})['stmf4_hmi_bridge']['ros__parameters']
vesc_h = (WS / 'stm32f401/src/VescGateway.h').read_text(encoding='utf-8')
vesc_cpp = (WS / 'stm32f401/src/VescGateway.cpp').read_text(encoding='utf-8')
neo_h = (WS / 'stm32f401/src/Neo3Sensors.h').read_text(encoding='utf-8')
neo_cpp = (WS / 'stm32f401/src/Neo3Sensors.cpp').read_text(encoding='utf-8')
main = (WS / 'stm32f401/src/main.cpp').read_text(encoding='utf-8')

def require(ok, msg):
    if not ok:
        raise AssertionError(msg)

# Physical pin contract: no collision between VESC, NEO3 UART and NEO3 I2C.
require('Uart uart_{PB7, PB6}' in vesc_h, 'VESC must use F411 USART1 RX=PB7 TX=PB6')
require('Uart gnss_serial_{PA3, PA2}' in neo_h, 'NEO3 GNSS must use USART2 RX=PA3 TX=PA2')
require('Wire.setSDA(PB9)' in neo_cpp and 'Wire.setSCL(PB8)' in neo_cpp,
        'NEO3 IST8310 I2C must use SDA=PB9 SCL=PB8')
require('static constexpr uint32_t kBaud = 1000000' in vesc_h, 'VESC UART must remain 1000000')
require('gVesc.begin()' in main and 'gVesc.poll()' in main, 'F411 VESC gateway lifecycle missing')
require('VESC:MODE:RUNTIME' in vesc_cpp and 'VESC:MODE:MAINTENANCE' in vesc_cpp,
        'F411 VESC runtime/maintenance ownership missing')

# ROS must be the single USB CDC owner and safely multiplex raw VESC bytes.
for token in ('/stmf4/vesc/runtime_tx', '/stmf4/vesc/maintenance_tx', '/stmf4/vesc/rx',
              '/stmf4/vesc/mode', '/stmf4/vesc/status', '/stmf4/vesc/connected',
              'VESC:MODE:RUNTIME', 'VESC:STATUS'):
    require(token in bridge, f'ROS F411 VESC bridge contract missing: {token}')
require(float(hmi.get('vesc_transport_timeout_sec', 0.0)) > 0.0, 'VESC transport watchdog missing')
require('rx_count > 0UL' in bridge, 'VESC connected state must require real F103 RX traffic')
print('PASS stmf4_gateway_self_check')
print('pins: VESC PB6/PB7 | NEO3 I2C PB8/PB9 | GNSS PA2/PA3 | one F411 USB CDC')
