#!/usr/bin/env python3
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
app = (root/'web/static/app.js').read_text(encoding='utf-8')
vesc = (root/'web/static/vesc_workbench.js').read_text(encoding='utf-8')
server = (root/'web/web_server.cpp').read_text(encoding='utf-8')

def require(ok, msg):
    if not ok:
        print('FAIL:', msg)
        sys.exit(1)

require('eventTimer_.setInterval(20);' in server,
        'SSE delta broadcaster must run at 50 Hz')
require('highRateTelemetry' in server and 'sensorQos' in server,
        'high-rate ESC telemetry must use low-latency best-effort QoS')
require("STICKY_STREAMS=new Set(['foc_telemetry'" in app,
        'frontend telemetry must preserve last-known-good objects')
require('mergeSticky(prev,next)' in app and 'queueRender()' in app,
        'SSE merge must coalesce rendering without dropping latest state')
require('v5RuntimeCache={1:{},2:{}}' in vesc,
        'dual ESC runtime telemetry must have per-motor last-known-good cache')
require("a.type==='number')a.blur()" in app,
        'mouse wheel must not accidentally tune focused numeric inputs')
require('param-slider' in app and 'bindParamNumber' in app,
        'numeric tuning parameters must expose slider + keyboard-capable number input')
print('PASS web_realtime_contract_self_check')
print('contract: ROS ingest 50 Hz | UI coalesced | ESC LKG cache | safe parameter controls')
