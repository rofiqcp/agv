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
source = (ROOT / "src/ackermann_controller_server.cpp").read_text()
launch = (ROOT / "launch/esc.launch.py").read_text()

# Runtime-routing keys are launch-owned so commissioning overrides cannot be
# shadowed by the exact esc_ackermann YAML node scope. Direct executable runs
# still receive safe C++ defaults, while autonomous.launch routes all motion
# through cmd_vel_router -> velocity_smoother -> /cmd_vel.
for key in ("nav2_topic", "teleop_topic", "active_source_topic",
            "require_autonomy_gate", "serial_device", "serial_enabled"):
    if key in ack:
        fail(f"runtime routing key must not be node-scoped in ackermann.yaml: {key}")
for token in (
    'DeclareLaunchArgument("nav2_topic", default_value="/cmd_vel")',
    'DeclareLaunchArgument("teleop_topic", default_value="/cmd_vel/teleop")',
    'DeclareLaunchArgument("serial_enabled", default_value="true")',
    '"nav2_topic": LaunchConfiguration("nav2_topic")',
    '"teleop_topic": LaunchConfiguration("teleop_topic")',
    '"serial_enabled": ParameterValue(LaunchConfiguration("serial_enabled"), value_type=bool)',
):
    if token not in launch:
        fail(f"ESC launch runtime-routing contract missing: {token}")
if ack.get("output_topic") != "/cmd_vel/actuator":
    fail("actuator diagnostic output topic is invalid")
if float(ack.get("command_watchdog_sec", 99.0)) >= float(ack.get("nav2_timeout_sec", 0.0)):
    fail("serial command watchdog must be tighter than Nav2 source timeout")
if float(ack.get("serial_tx_rate_hz", 0.0)) != float(ack.get("command_rate_hz", 0.0)):
    fail("ROS command and STM transmit rates must match")
if int(ack.get("serial_baud", 0)) != 115200:
    fail("STM protocol baud must remain 115200")
if "Prolific_Technology_Inc._USB-Serial_Controller" not in str(ack.get("serial_auto_id_contains", "")):
    fail("ESC PL2303 by-id selector is invalid")
if not str(ack.get("serial_auto_path_contains", "")).strip():
    fail("ESC must retain a non-empty physical by-path fallback")
if source.find('directory_iterator("/dev/serial/by-id"') > source.find('directory_iterator("/dev/serial/by-path"'):
    fail("ESC must evaluate unique by-id before physical by-path fallback")
if "configured selectors cannot resolve" not in source:
    fail("ESC auto-routing must fail closed when identity/path are unresolved")
if "no unique serial candidate" not in source:
    fail("ESC must expose a clear diagnostic when serial identity is unresolved")
if "errno == EAGAIN || errno == EWOULDBLOCK" not in source or "would_block_retries" not in source:
    fail("ESC nonblocking USB-UART TX must tolerate bounded transient EAGAIN")
if "serial_active_path_" not in source or '<< " path="' not in source:
    fail("ESC status must report the active physical serial path")
for token in ("TIOCEXCL", "safe_stop_requested_", "SAFE SHUTDOWN", "rclcpp::on_shutdown", "Never tcdrain()"):
    if token not in source:
        fail(f"ESC Ctrl+C/USB safe-shutdown contract missing: {token}")
if float(shared.get("speed_max", -1.0)) != float(ack.get("speed_max", -2.0)):
    fail("shared teleop and actuator speed limits disagree")

for token in (
    "if (estop)",
    "if (teleop_fresh && (teleop_active || teleop_hold))",
    "if (nav2_fresh && gate_ok)",
    "age > command_watchdog_sec_",
    "crc16Ccitt",
    "ack_fresh && left_ready && right_ready && !firmware_failsafe",
    "odom.twist.twist.linear.x = drive_mps",
    'create_publisher<std_msgs::msg::Float64>("/esc/kinematic_yaw_rate_rps"',
):
    if token not in source:
        fail(f"ESC runtime safety behavior missing: {token}")
if "age >= 0.0 && age <= timeout_sec" not in source:
    fail("ESC command freshness must reject negative age after a clock jump")
if "physical_deg / operational_deg" not in source or "(-physical_deg) / operational_deg" not in source:
    fail("uncalibrated wheel span must map to the full STM/FOC +/-90 protocol span")
if '<< " stm_range=+/-" << steering_max_deg_' not in source:
    fail("ESC status must expose the STM/FOC protocol range separately from wheel angle")
teleop_source = (ROOT / "src/motor_teleop.cpp").read_text()
if 'RCLCPP_WARN(get_logger(), "[TTY] /dev/tty tidak dapat dibuka' in teleop_source:
    fail("headless /dev/tty fallback must not be reported as a warning")
if "TransformBroadcaster" in source or "sendTransform" in source:
    fail("ESC must not publish odom->base TF; local EKF owns that transform")
if source.count("::open(") != 1:
    fail("ESC serial device must have exactly one open owner")
if "exactly two nodes" not in launch and "exactly two" not in launch:
    fail("ESC launch ownership is not documented")
if "motor_teleop" not in launch or "ackermann_controller_server" not in launch:
    fail("ESC launch must contain teleop plus the single serial actuator owner")

print("PASS ESC runtime contract")
print("priority: E_STOP > TELEOP > gated NAV2 > IDLE")
print("fusion authority: ESC longitudinal speed only; kinematic yaw is diagnostic")
