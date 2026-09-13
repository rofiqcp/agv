#!/usr/bin/env python3
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
WS=ROOT.parents[1]
F4=WS/'F4gateway'
bridge=(ROOT/'src/stmf4_hmi_bridge.cpp').read_text()
main=(F4/'src/main.cpp').read_text()
neo=(F4/'src/Neo3ProSensors.cpp').read_text()
neo_h=(F4/'src/Neo3ProSensors.h').read_text()
pio=(F4/'platformio.ini').read_text()
board=(F4/'src/BoardSupport.cpp').read_text()
hmi=(yaml.safe_load((ROOT/'config/hmi.yaml').read_text()) or {})['stmf4_hmi_bridge']['ros__parameters']
def req(v,m):
    if not v: raise AssertionError(m)
req('F4_ESC_GATEWAY=0' in pio or '-DF4_ESC_GATEWAY=0' in pio,'production F4 must not own ESC')
req(int(hmi['serial_baud'])==1000000,'NUC<->F411 CDC must remain 1 Mbaud')
req(str(hmi.get('serial_device','')).lower()=='auto','serial selector must use identity discovery')
for t in ('HOST:HELLO:', 'ACK:HOST:SESSION:', 'NEO:STATUS', 'USB:STATUS'):
    req(t in bridge+main,t+' session contract missing')
req('NEO:LED:AUTO' not in bridge,'reconnect must not mutate NEO LED state')
for t in ('manual_command_lease_sec','manualLeaseFresh','navigationMotionGateReady','resetTransportEpochState'):
    req(t in bridge,t+' safety/integrity contract missing')
req('wait_for_service(250ms)' not in bridge,'Nav2 cancel must not block serial executor')
for t in ('CMD:DRIVE:STOP','writeLineHighPriority','serviceSafetyControlTx','serviceManualDriveLease'):
    req(t in main,t+' F4 safety-control path missing')
for t in ('GPIO_PIN_10','EXTI15_10_IRQn'):
    req(t in board,t+' MCP interrupt path missing')
for t in ('Board_MonotonicMicros64()', 'RAW_CAN_QUEUE_CAPACITY = 128U','can_health_tec_'):
    req(t in neo+neo_h,t+' NEO transport contract missing')
print('PASS stmf4_gateway_self_check HMI+NEO3PRO production')
