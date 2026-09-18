#!/usr/bin/env python3
"""Dependency-light ESC command/serial/odometry safety contract."""
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]


def fail(message: str) -> None:
    print(f"FAIL: {message}")
    raise SystemExit(1)


ack = (yaml.safe_load((ROOT / "config/ackermann.yaml").read_text()) or {})[
    "esc_ackermann"]["ros__parameters"]
teleop = yaml.safe_load((ROOT / "config/teleop.yaml").read_text()) or {}
shared = teleop["/**"]["ros__parameters"]
vehicle = yaml.safe_load((ROOT.parent / "navigation/config/vehicle.yaml").read_text())[
    "vehicle"]["ros__parameters"]
source = (ROOT / "src/ackermann_controller_server.cpp").read_text()
launch = (ROOT / "launch/esc.launch.py").read_text()

# Runtime-routing keys are launch-owned so commissioning overrides cannot be
# shadowed by the exact esc_ackermann YAML node scope. Direct executable runs
# still receive safe C++ defaults, while autonomous.launch routes all motion
# through cmd_vel_router -> velocity_smoother -> /cmd_vel.
for key in ("nav2_topic", "teleop_topic", "teleop_source_topic", "active_source_topic",
            "require_autonomy_gate", "transport_mode", "serial_device", "serial_enabled"):
    if key in ack:
        fail(f"runtime routing key must not be node-scoped in ackermann.yaml: {key}")
for token in (
    'DeclareLaunchArgument("nav2_topic", default_value="/cmd_vel")',
    'DeclareLaunchArgument("teleop_topic", default_value="/cmd_vel/teleop")',
    'DeclareLaunchArgument("teleop_source_topic", default_value="/teleop/active_source")',
    'DeclareLaunchArgument("transport_mode", default_value="direct_vesc")',
    'DeclareLaunchArgument("serial_enabled", default_value="true")',
    '"nav2_topic": LaunchConfiguration("nav2_topic")',
    '"teleop_topic": LaunchConfiguration("teleop_topic")',
    '"teleop_source_topic": LaunchConfiguration("teleop_source_topic")',
    '"transport_mode": LaunchConfiguration("transport_mode")',
    'DeclareLaunchArgument("integration_bypass", default_value="false")',
    '"serial_enabled": ParameterValue(PythonExpression([',
    'LaunchConfiguration("integration_bypass")',
):
    if token not in launch:
        fail(f"ESC launch runtime-routing contract missing: {token}")
if ack.get("output_topic") != "/cmd_vel/actuator":
    fail("actuator diagnostic output topic is invalid")
if float(ack.get("command_watchdog_sec", 99.0)) >= float(ack.get("nav2_timeout_sec", 0.0)):
    fail("serial command watchdog must be tighter than Nav2 source timeout")
if float(ack.get("serial_tx_rate_hz", 0.0)) != float(ack.get("command_rate_hz", 0.0)):
    fail("ROS command and STM transmit rates must match")
if int(ack.get("serial_baud", 0)) != 921600:
    fail("direct ESC USB-UART must remain 921600 baud")
if abs(float(ack.get("drive_erpm_per_mps", 0.0)) -
       float(vehicle.get("drive_erpm_per_mps", -1.0))) > 1e-9:
    fail("ESC drive_erpm_per_mps must match vehicle.yaml SSOT")
if abs(float(vehicle.get("wheel_radius_m", 0.0)) - 0.145) > 1e-9:
    fail("vehicle wheel radius SSOT must remain 0.145 m")
# Direct native VESC runtime is one physical USB-UART owner.
for token in ("appendVescFrame", "appendVescSetPos", "appendVescSetErpm",
              "appendVescValuesRequest", "appendSteeringCalibrationRequest",
              "buildNativeRuntimeBatch", "drainDirectRx", "TIOCEXCL",
              '"/esc/vesc/direct_rx"', '"/esc/vesc/direct_connected"'):
    if token not in source:
        fail(f"direct native VESC transport missing: {token}")
if 'transport_mode must be direct_vesc' not in source:
    fail("legacy F411 ESC transport still accepted")

for token in ("driveErpmPerMps", "rightCommandLimitErpm", "rightErpmFor",
              "return drive_erpm_per_mps_",
              "raw_commissioning_enabled", "rawCommissioningSnapshot",
              "clampCommissioningSteeringDeg", "stm32_right_erpm", "RAW_COMMISSIONING",
              "physicalToVescPositionDeg", "vescPositionToPhysicalDeg",
              "std::vector<std::uint8_t> payload{kVescSetPos}"):
    if token not in source:
        fail(f"native VESC eRPM conversion contract missing: {token}")
if "1a86_USB_Serial" not in str(ack.get("serial_auto_id_contains", "")):
    fail("direct VESC serial selector missing")
if str(ack.get("serial_auto_path_contains", "")).strip():
    fail("direct-serial recovery must not use topology-dependent physical by-path fallback")
if 'declare_parameter<std::string>("serial_auto_path_contains", "")' not in source:
    fail("ESC source default must keep physical by-path fallback disabled")
if "configured selectors cannot resolve" not in source:
    fail("ESC auto-routing must fail closed when identity/path are unresolved")
if "no unique serial candidate" not in source:
    fail("ESC must expose a clear diagnostic when serial identity is unresolved")
if "errno == EAGAIN || errno == EWOULDBLOCK" not in source or "would_block_retries" not in source:
    fail("ESC nonblocking USB-UART TX must tolerate bounded transient EAGAIN")
if "serial_active_path_" not in source or '<< " path="' not in source:
    fail("ESC status must report the active physical serial path")
for token in ("TIOCEXCL", "safe_stop_requested_", "SAFE SHUTDOWN", "rclcpp::on_shutdown", "buildNativeSafeStopBatch"):
    if token not in source:
        fail(f"ESC Ctrl+C/USB safe-shutdown contract missing: {token}")
if float(shared.get("speed_max", -1.0)) != float(ack.get("manual_speed_max_mps", -2.0)):
    fail("shared teleop speed limit must match actuator manual_speed_max_mps")

for token in (
    "if (estop)",
    "const bool hmi_manual = hmi_fresh && hmi_active",
    "if (teleop_active)",
    "if (teleop_hold)",
    "const bool unified_cmd_gate_ok = routed_teleop ||",
    "routed_web_trial && !gate_ok",
    "if (nav2_fresh && unified_cmd_gate_ok)",
    "PERCEPTION:",
    "age > command_watchdog_sec_",
    "crc16Ccitt",
    "makeVescFrame",
    "appendVescSetPos",
    "appendVescSetErpm",
    "wrapRightMotor",
    "appendVescValuesRequest",
    "buildNativeRuntimeBatch",
    "consumeVescRxBytes",
    "maintenance_mode_active_",
    "ack_fresh && left_ready && right_ready && !firmware_failsafe",
    "odom.twist.twist.linear.x = drive_mps",
    'create_publisher<std_msgs::msg::Float64>("/esc/kinematic_yaw_rate_rps"',
):
    if token not in source:
        fail(f"ESC runtime safety behavior missing: {token}")
if "age >= 0.0 && age <= timeout_sec" not in source:
    fail("ESC command freshness must reject negative age after a clock jump")
for token in ("physicalToVescPositionDeg", "vescPositionToPhysicalDeg",
              "steering_vesc_pos_left_deg_", "steering_vesc_pos_center_deg_",
              "steering_vesc_pos_right_deg_"):
    if token not in source:
        fail(f"native VESC steering 0/180/360 mapping missing: {token}")
if '<< " stm_range=+/-" << steering_max_deg_' not in source:
    fail("ESC status must expose the internal steering range separately from wheel angle")
teleop_source = (ROOT / "src/motor_teleop.cpp").read_text()
teleop_params = teleop["motor_teleop"]["ros__parameters"]
for token in ("/teleop/joystick_connected", "/teleop/joystick_status", "publish_gamepad_link(false)"):
    if token not in teleop_source:
        fail(f"joystick observability contract missing: {token}")
for token in ("require_neutral_rearm(dev)", "validate_axis_motion",
              "set_gamepad_connected(true, 0.0, 0.0)", "[JOY-SAFETY]"):
    if token not in teleop_source:
        fail(f"joystick fail-closed HID safety missing: {token}")
if float(teleop_params.get("post_limit_neutral_rearm_sec", 0.0)) < 0.5:
    fail("joystick fault neutral re-arm must be >= 0.5 s")
if abs(float(teleop_params.get("speed_initial", -1.0)) - float(teleop_params.get("speed_min", -2.0))) > 1e-9:
    fail("teleop must start at velocity level 1")
if abs(float(teleop_params.get("yaw_initial_deg_s", -1.0)) - float(teleop_params.get("yaw_min_deg_s", -2.0))) > 1e-9:
    fail("teleop must start at steering level 1")
if not (0.10 <= float(teleop_params.get("button_axis_delta_guard", 0.0)) <= 0.50):
    fail("legacy button-coincident axis delta threshold must remain bounded")
if float(teleop_params.get("axis_full_scale_jump_guard", 0.0)) < 0.85:
    fail("joystick full-scale jump guard is too permissive")
if "if (mag < axis_full_scale_jump_guard_) intermediate_seen = true;" not in teleop_source:
    fail("direct neutral->full-scale EV_ABS must not self-authorize as an intermediate sample")
for token in ("dev.forward_intermediate_seen = false", "dev.yaw_intermediate_seen = false",
              "dev.forward_intermediate_seen = true", "dev.yaw_intermediate_seen = true"):
    if token not in teleop_source:
        fail(f"event-batch intermediate-axis proof missing: {token}")
if not (0.015 <= float(teleop_params.get("axis_motion_confirm_sec", 0.0)) <= 0.03):
    fail("joystick motion onset confirmation is too short")
if float(teleop_params.get("button_debounce_sec", 99.0)) > 0.04:
    fail("joystick level-button debounce must remain <=40 ms for responsive click-by-click control")
if float(teleop_params.get("limit_button_axis_guard_sec", 99.0)) != 0.0:
    fail("normal LB/LT/RB/RT must not freeze valid stick motion; button axis guard must be disabled")
if 'if (pressed) dev.button_press_pending[code] = true;' not in teleop_source:
    fail("joystick quick-tap rising edge must be queued independently of release")
if 'dev.button_press_pending[code] = pressed;' in teleop_source:
    fail("joystick release must not erase a queued quick-tap")
if 'service_limit_button_repeat' in teleop_source:
    fail("level buttons must be edge-triggered, never auto-repeat while held")
if not (0.05 <= float(teleop_params.get("axis_event_start_window_sec", 0.0)) <= 0.30):
    fail("joystick EV_ABS onset window must remain tight")
for token in ("last_forward_abs_event", "last_yaw_abs_event",
              "last_forward_abs_value", "last_yaw_abs_value",
              "event_value != snapshot_value", "snapshot-only onset",
              "trusted_forward", "trusted_yaw", "button-coincident jump",
              "service_limit_button_edges", "SYN_DROPPED -> force 0,0"):
    if token not in teleop_source:
        fail(f"joystick event-level safety missing: {token}")
button_handler = teleop_source.split("void handle_gamepad_button", 1)[1].split("void service_limit_button_edges", 1)[0]
if "require_neutral_rearm" in button_handler:
    fail("normal LB/LT/RB/RT edge must not trigger neutral re-arm")
if "service_limit_button_repeat" in teleop_source:
    fail("held limit buttons must not auto-repeat levels")
if "Independent domains: LB+RB may be clicked together and both update once." not in teleop_source:
    fail("LB/RB independent simultaneous click contract missing")
if 'service_pair(BTN_TL, BTN_TL2' not in teleop_source or 'service_pair(BTN_TR, BTN_TR2' not in teleop_source:
    fail("shoulder buttons must be serviced as two independent domains")
if 'set_gamepad_connected(true, dev.trusted_forward, dev.trusted_yaw)' not in teleop_source:
    fail("forward+yaw axes must be published together as one joystick state")
open_wait = teleop_source.split("init_button_snapshot(dev);", 1)[1].split("if (same_reconnect)", 1)[0]
if "set_gamepad_connected(false, 0.0, 0.0)" not in open_wait:
    fail("USB receiver open must remain fail-closed until centered controller state is validated")
if 'RCLCPP_WARN(get_logger(), "[TTY] /dev/tty tidak dapat dibuka' in teleop_source:
    fail("headless /dev/tty fallback must not be reported as a warning")
if "TransformBroadcaster" in source or "sendTransform" in source:
    fail("ESC must not publish odom->base TF; local EKF owns that transform")
if source.count("::open(") != 1:
    fail("ESC serial device must have exactly one open owner")
tool_source = (ROOT / "src/vesc_tool_bridge.cpp").read_text()
if "motor_teleop" not in launch or "ackermann_controller_server" not in launch or "vesc_tool_bridge" not in launch:
    fail("ESC launch must contain teleop, Ackermann runtime, and VESC maintenance bridge")
for token in ("maintenance_owner_topic_", "maintenance_owner_sub_",
              "maintenance_mode_active_", "direct_enter_safe_stop_pending_",
              "if (maintenance != active) return",
              "if (maintenance) direct_runtime_tx_queue_.clear()",
              "else direct_maintenance_tx_queue_.clear()",
              "if (!maintenance && now_steady >= next_tx)",
              "effective_source = maintenance_active ? maintenance_owner",
              "sendRuntimeSafeStop();", "publishActive(true);",
              "sendMaintenanceSafeStop(); transition_ = Transition::EXIT_STOP"):
    if token not in source and token not in tool_source:
        fail(f"maintenance ownership/data-flow contract missing: {token}")
if source.find('if (!maintenance && now_steady >= next_tx)') < 0:
    fail("runtime SET_POS/SET_RPM batch must be completely disabled during maintenance")
if tool_source.find('sendRuntimeSafeStop();') > tool_source.find('publishActive(true);'):
    fail("maintenance entry must safe-stop runtime before publishing maintenance_active")

for token in ("/esc/vesc/maintenance_tx", "/esc/vesc/direct_rx", "/esc/vesc/direct_connected",
              "/esc/vesc/maintenance_active", "/esc/vesc/maintenance_owner",
              "MODE:MAINTENANCE", "COMM_GET_MCCONF", "COMM_DETECT_HALL_FOC", "COMM_DETECT_ENCODER",
              "COMM_TERMINAL_CMD", "127.0.0.1", "tcp_port", "python_tcp_port", "PYTHON_MAINTENANCE",
              "SOCK_NONBLOCK", "TCP_NODELAY", "tcp_client_fd_", "python_tcp_client_fd_",
              "sendMaintenanceSafeStop", "command_rejected_python_has_priority",
              "command_rejected_tcp_client_owns_maintenance", "SAFE_STOP:BOTH", "KEEPALIVE:WEB",
              "webLeaseTick", "cancelWebDetectionIfActive", "/esc/vesc/detect_state",
              "COMM_DETECT_MOTOR_R_L", "COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP",
              "COMM_DETECT_APPLY_ALL_FOC"):
    if token not in tool_source:
        fail(f"VESC maintenance feature missing: {token}")

print("PASS ESC runtime contract")
print("priority: E_STOP > PYTHON_MAINTENANCE > VESC_TOOL > MANUAL(HMI/ROSWEB/TELEOP freshest) > PERCEPTION > gated NAV2 > IDLE")
print("fusion authority: ESC longitudinal speed only; kinematic yaw is diagnostic")

assert "maintenance_active" in (ROOT / "src/vesc_tool_bridge.cpp").read_text()

assert "!tcp_probe_pending_" in (ROOT / "src/vesc_tool_bridge.cpp").read_text()
