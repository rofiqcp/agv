#!/usr/bin/env python3
"""Static regression guard for Stage-1 hardware/steering/odometry foundation."""
from __future__ import annotations
from gui_source_helper import read_gui_source

import math
import re
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2]
NAV = ROOT / "navigation"
ESC = ROOT / "esc"


def load_params(path: Path, node: str) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return data[node]["ros__parameters"]


def require(cond: bool, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)


vehicle = load_params(NAV / "config/vehicle.yaml", "vehicle")
navcore = load_params(NAV / "config/navigation_core.yaml", "navigation_core")
esc = load_params(ESC / "config/ackermann.yaml", "esc_ackermann")
gnss = load_params(NAV / "config/gnss.yaml", "data_cuav_node")
imu = load_params(NAV / "config/imu.yaml", "data_imu_node")
hmi = load_params(ROOT / "stmf4/config/hmi.yaml", "stmf4_hmi_bridge")
launch_text = (NAV / "launch/autonomous.launch.py").read_text(encoding="utf-8")

# 1) Production routing: NEO-3/IST8310 + F103 VESC share one identity-checked
# F411 CDC gateway; IMU remains direct CP2102. Legacy direct serial is by-id only.
identities = {
    "ESC": esc["serial_auto_id_contains"],
    "GNSS": gnss["auto_port_id_contains"],
    "IMU": imu["auto_port_id_contains"],
}
require(identities["ESC"] == "Prolific_Technology_Inc._USB-Serial_Controller", "ESC PL2303 identity changed")
require(identities["GNSS"] == "1a86_USB_Serial", "GNSS CH340 identity changed")
require(identities["IMU"] == "Silicon_Labs_CP2102", "IMU CP2102 identity changed")
require(len(set(identities.values())) == len(identities), "serial by-id selectors overlap")
selectors = {
    "ESC_RECOVERY": esc["serial_auto_path_contains"],
    "GNSS_RECOVERY": gnss["auto_port_path_contains"],
    "IMU": imu["auto_port_path_contains"],
}
require(str(hmi.get("serial_device", "")).lower() == "auto", "F411 gateway must use auto identity discovery")
require("DeclareLaunchArgument('gnss_source', default_value='stm32'" in launch_text, "GNSS must default to F411/stm32")
require("DeclareLaunchArgument('esc_transport_mode', default_value='stm32'" in launch_text, "ESC must default to F411/stm32")
require(not any(str(v).strip() for v in selectors.values()), "physical by-path fallback must be disabled")

# 2) Calibration state must be explicit, but this regression test must remain
# valid both before and after real field commissioning.
for key in (
    "steering_calibration_valid",
    "steering_circle_calibration_valid",
    "drive_odometry_calibration_valid",
):
    require(isinstance(vehicle.get(key), bool), f"{key} must exist and be boolean")

# 3) Geometry must remain physically self-consistent. Before circle
# certification we enforce the safe analytical fallback. After certification,
# measured geometry is allowed as long as it remains positive and sane.
left = abs(math.degrees(float(vehicle["measured_left_steering_limit_rad"])))
right = abs(math.degrees(float(vehicle["measured_right_steering_limit_rad"])))
op = abs(math.degrees(float(vehicle["operational_steering_angle_rad"])))
require(op > 0.0, "operational steering must be positive")
require(op <= min(left, right) + 1e-9, "operational steering exceeds mechanical limit")
require(abs(float(esc["steering_physical_operational_limit_deg"]) - op) < 1e-6,
        "ESC and vehicle physical operating limit disagree")

wb = float(vehicle["effective_wheelbase_m"])
track = float(vehicle["track_width_m"])
actual_r = float(vehicle["minimum_turning_radius_m"])
require(wb > 0.0 and track > 0.0 and actual_r > 0.0,
        "Ackermann geometry must be strictly positive")

if not vehicle["steering_circle_calibration_valid"]:
    margin = 1.0 + float(vehicle["turning_radius_safety_margin_pct"]) / 100.0
    expected_r = (0.5 * track + wb / math.tan(math.radians(op))) * margin
    require(abs(expected_r - actual_r) < 1e-6,
            f"uncalibrated minimum radius inconsistent: expected {expected_r}, got {actual_r}")
else:
    source = str(vehicle.get("steering_calibration_source", ""))
    require("circle" in source.lower() or str(vehicle.get("steering_circle_calibration_saved_at", "")).strip(),
            "circle-valid state needs traceable measured-circle provenance")

require(float(vehicle["max_yaw_rate_rps"]) <= float(vehicle["max_forward_speed_mps"]) / actual_r + 1e-9,
        "vehicle yaw-rate exceeds Ackermann kinematic ceiling")

# 4) Autonomous actuation must fail closed until Stage-1 calibration is saved.
for key in (
    "require_steering_calibration_for_autonomy",
    "require_steering_circle_calibration_for_autonomy",
    "require_drive_odometry_calibration_for_autonomy",
):
    require(navcore.get(key) is True, f"{key} must default true")

nav_cpp = (NAV / "src/navigation_core.cpp").read_text(encoding="utf-8")
for token in (
    "require_steering_calibration_ && !steering_calibration_validated_",
    "require_steering_circle_calibration_ && !steering_circle_calibration_validated_",
    "require_drive_odometry_calibration_ && !drive_odometry_calibration_validated_",
):
    require(token in nav_cpp, f"NavigationCore gate missing: {token}")

launch = (NAV / "launch/autonomous.launch.py").read_text(encoding="utf-8")
for token in (
    "steering_calibration_validated': steering_calibration_default",
    "steering_circle_calibration_validated': steering_circle_calibration_default",
    "drive_odometry_calibration_validated': drive_odometry_calibration_default",
):
    require(token in launch, f"launch does not inject vehicle calibration state: {token}")

# 5) ESC fallback must never use +/-80 protocol span as uncalibrated physical steering.
esc_cpp = (ESC / "src/ackermann_controller_server.cpp").read_text(encoding="utf-8")
require("return steering_max_deg_;\n  }\n\n  double clampPhysicalSteeringDeg" not in esc_cpp,
        "uncalibrated physical steering still falls back to legacy protocol span")
require("std::abs(steering_physical_operational_limit_deg_)" in esc_cpp,
        "safe physical operating fallback is not enforced in ESC")

# 5b) All serial identity owners must fail closed on ambiguous matches.
gnss_cpp = (NAV / "src/gnss_node.cpp").read_text(encoding="utf-8")
imu_cpp = (NAV / "src/imu_node.cpp").read_text(encoding="utf-8")
for label, source, token in (
    ("ESC", esc_cpp, "matches.size() == 1U"),
    ("GNSS", gnss_cpp, "matches.size() == 1U"),
    ("IMU", imu_cpp, "matches.size() == 1U"),
):
    require(token in source, f"{label} identity matching must fail closed when ambiguous")

# 6) Known Qt5/GCC build regressions from the field build log must stay removed.
gui = read_gui_source(NAV)
for bad in (
    "while (s.startsWith('_')) s.remove(0,1); while",
    "if(!v.isValid()||v.isNull())return YAML::Node(); const int t=",
    'for(const QString&k:{"',
    "targets[d.name()]={{",
    "all[d.name()]={{",
    'auto*reload=new QPushButton("Reload YAML")',
    'auto*revoke=new QPushButton("Revoke Safety")',
    "projectPixel(points_.value(layer).value",
):
    require(bad not in gui, f"known agv_gui build anti-pattern reintroduced: {bad}")
require("bool reload(){" in gui, "YamlStore::reload must report parse success/failure")

print("STAGE1 FOUNDATION SELF-CHECK: PASS")
print("serial route: F411 CDC shared GNSS/IST8310+VESC | IMU CP2102 | physical by-path disabled")
print(f"safe fallback steering: +/-{op:.3f} deg")
print(f"safe fallback turning radius: {actual_r:.6f} m")
