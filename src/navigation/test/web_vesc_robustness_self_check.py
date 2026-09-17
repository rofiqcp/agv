from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
html = (ROOT / "web/static/index.html").read_text(encoding="utf-8")
js = (ROOT / "web/static/vesc_workbench.js").read_text(encoding="utf-8")
ack_js = (ROOT / "web/static/esc.js").read_text(encoding="utf-8")
css = (ROOT / "web/static/esc.css").read_text(encoding="utf-8")
web = (ROOT / "web/web_server.cpp").read_text(encoding="utf-8")
ESC = ROOT.parent / "esc"
bridge = (ESC / "src/vesc_tool_bridge.cpp").read_text(encoding="utf-8")
cfg = (ESC / "config/vesc_tool.yaml").read_text(encoding="utf-8")

def req(ok, msg):
    if not ok:
        raise AssertionError(msg)

order = [html.index(f'id="{x}"') for x in (
    "vescGraphSection", "vescControlSection", "vescMotorSetupSection",
    "vescUtilitySection", "vescLiveSection")]
req(order == sorted(order), "ESC page must be Graph -> Controls -> Motor Setup -> Utilities -> Telemetry")
req("DUAL VESC 6.00 • F411 GATEWAY" not in html, "ESC header must not hard-code F411 as physical transport")
for token in ("SAFE_STOP:BOTH", "KEEPALIVE:WEB", "DETECT:ALL:",
              "DETECT:RL:", "DETECT:FLUX_OPEN:", "v5RenderDetect",
              "v5RenderIntegrity", "v5ApplyEffectiveLimits", "V5_FAULT_NAMES"):
    req(token in js, f"browser ESC robustness contract missing: {token}")
for token in ("vescDetectAll", "vescDetectRlLeft", "vescDetectRlRight",
              "vescDetectFluxLeft", "vescDetectFluxRight", "vescIntegritySection",
              "vescEffectiveLimits",
              "vescDetectEncoderSetup", "vescDetectHallSetup"):
    req(f'id="{token}"' in html, f"VESC setup control missing: {token}")
for token in ("web_lease_timeout_ms", "webLeaseTick", "web_lease_expired_maintenance_latched",
              "webMotorHeartbeatTick", "clearWebMotorHeartbeat", "web_motor_heartbeat_",
              "tcp_keepalive_period_ms", "tcpKeepaliveTick", "COMM_ALIVE",
              "HB_VERSION = 2",
              "cancelWebDetectionIfActive", "max_abs_current_a",
              "SAFE_STOP:BOTH", "COMM_DETECT_MOTOR_R_L",
              "COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP",
              "COMM_DETECT_APPLY_ALL_FOC", "/esc/vesc/detect_state"):
    req(token in bridge, f"VESC bridge safety/detect contract missing: {token}")
req("web_lease_timeout_ms: 5000" in cfg, "Web maintenance lease timeout must tolerate browser scheduling jitter")
req("tcp_keepalive_period_ms: 100" in cfg, "VESC Tool TCP keepalive fallback missing from config")
req("SET:POS:1" in js,
    "Web maintenance must match VESC Tool and send standard COMM_SET_POS for LEFT position")
req('parts[1] == "STEER"' not in bridge and "HB_SET_STEERING_DEG" not in bridge,
    "Web/bridge must not expose a second custom steering command path")
req(all(x in html for x in ("ackScaleVescLeftPos","ackScaleVescCenterPos","ackScaleVescRightPos","ackCaptureLeftPos","ackCaptureCenterPos","ackCaptureRightPos")),
    "ROS Web must expose three-point physical-to-VESC-POS calibration/capture controls")
req("steering_vesc_pos_left_deg" in ack_js and "COMM_SET_POS" in html,
    "ROS Web Ackermann mapping must stage raw VESC POS calibration and document COMM_SET_POS")
req('id="vescLeftPosInput" type="number" min="0" max="360"' in html,
    "Web VESC Tool workbench must expose the standard 0..360 degree position domain")
req("Position 0–360°" in html and "Position 0/180/360" in html,
    "Web UI must document the VESC Tool 0/180/360 steering adapter")
req("activePage==='esc'" not in js[js.rfind("setInterval"):],
    "Web maintenance lease renewal must survive navigation between ESC subviews")
req('{"/esc/vesc/detect_state", "vesc_detect_state"}' in web,
    "Web server must expose VESC detection state")
req(".vesc-motor-setup" in css and ".vesc-detect-result" in css,
    "VESC motor setup styling missing")
print("PASS web_vesc_robustness_self_check")
print("order: scope -> direct controls -> FOC setup -> utilities -> telemetry")
print("safety: atomic stop + browser lease + VESC ALIVE refresh + fail-closed maintenance owner")
