#!/usr/bin/env python3
"""Repository-wide static regression audit that does not require ROS 2.

The hardware/ROS integration still has to be validated on the target mini-PC.
This check catches source/config defects that previously escaped narrow unit tests:
duplicate YAML/ROS parameters, incomplete launch wiring, unsafe command ownership,
and accidental changes to the ESC/GNSS/IMU localization contract.
"""
from __future__ import annotations

import ast
from collections import Counter
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT.parent


def fail(message: str) -> None:
    print(f"FAIL: {message}")
    raise SystemExit(1)


class UniqueKeyLoader(yaml.SafeLoader):
    pass


def construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f"duplicate YAML key {key!r} at line {key_node.start_mark.line + 1}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_unique_mapping)


# Every YAML must parse without duplicate keys. Silent duplicate-key overwrite is
# particularly dangerous for calibration and safety thresholds.
for path in sorted(SRC.rglob("*.yaml")):
    try:
        yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueKeyLoader)
    except Exception as error:
        fail(f"invalid/ambiguous YAML {path.relative_to(SRC)}: {error}")


# Launch files must be valid Python and every LaunchConfiguration in each top-level
# launch must have a declaration in that same file.
for path in sorted(SRC.rglob("*.launch.py")):
    source = path.read_text(encoding="utf-8")
    try:
        ast.parse(source, filename=str(path))
    except SyntaxError as error:
        fail(f"launch syntax error {path.relative_to(SRC)}: {error}")
    declarations = set(re.findall(
        r"DeclareLaunchArgument\(\s*['\"]([^'\"]+)['\"]", source))
    references = set(re.findall(
        r"LaunchConfiguration\(\s*['\"]([^'\"]+)['\"]", source))
    missing = sorted(references - declarations)
    if missing:
        fail(f"undeclared launch arguments in {path.relative_to(SRC)}: {missing}")


# Package manifests must remain valid XML.
for path in sorted(SRC.glob("*/package.xml")):
    try:
        ET.parse(path)
    except ET.ParseError as error:
        fail(f"invalid package.xml {path.relative_to(SRC)}: {error}")


# rclcpp throws when the same parameter is declared twice in one node.
declare_pattern = re.compile(
    r"declare_parameter(?:<[^\n;()]*>)?\s*\(\s*['\"]([^'\"]+)['\"]")
for path in sorted(SRC.rglob("*.cpp")):
    names = declare_pattern.findall(path.read_text(encoding="utf-8", errors="ignore"))
    duplicates = sorted(name for name, count in Counter(names).items() if count > 1)
    if duplicates:
        fail(f"duplicate rclcpp parameters in {path.relative_to(SRC)}: {duplicates}")


autonomous = (ROOT / "launch/autonomous.launch.py").read_text(encoding="utf-8")
gui = (ROOT / "launch/gui.launch.py").read_text(encoding="utf-8")
navigation_core = (ROOT / "src/navigation_core.cpp").read_text(encoding="utf-8")
trajectory = (ROOT / "src/trajectory_safety_supervisor.cpp").read_text(encoding="utf-8")
localization = (ROOT / "src/localization_core.cpp").read_text(encoding="utf-8")
imu_source = (ROOT / "src/imu_node.cpp").read_text(encoding="utf-8")
cpu_source = (SRC / "perception/src/astra_yolop_cpu_pt_node.cpp").read_text(encoding="utf-8")
gpu_source = (SRC / "perception/src/astra_yolop_gpu_node.cpp").read_text(encoding="utf-8")

for source, label in ((autonomous, "autonomous"), (gui, "gui")):
    for token in ("perception_mode", "engine_path", "pt_model_path"):
        if token not in source:
            fail(f"{label} launch missing perception option {token}")

for token in (
    "condition=IfCondition(perception_cpu_enabled)",
    "condition=IfCondition(perception_gpu_enabled)",
    "camera_only_node",
    "else '/cmd_vel_nav_raw'",
    "'lane_safety_enabled': ParameterValue(lane_enabled",
):
    if token not in autonomous:
        fail(f"autonomous launch safety/backend wiring missing: {token}")

for token in (
    "camera_metric_calibration_validated_ &&",
    "CALIBRATION_REQUIRED_PASSTHROUGH",
):
    if token not in cpu_source or token not in gpu_source:
        fail(f"CPU/GPU metric authority gate missing: {token}")
for token in (
    "mixRecenterCommand",
    r'\"decision\"',
    r'\"recenter_blocked\"',
    "strict_camera_mode_",
    "CAMERA_MODE_MISMATCH",
):
    if token not in cpu_source:
        fail(f"CPU perception parity/strict-mode contract missing: {token}")

for token in (
    "lane_safety_enabled_ && lane_control_fresh && lane_recenter_blocked",
    "lane_safety_enabled_ && !path_obstacle_near && lane_fresh && lane_valid",
    "lane_safety_enabled_ && stop_on_lane_lost_ && lane_fresh",
):
    if token not in trajectory:
        fail(f"lane-safety OFF isolation missing: {token}")

# Exactly one final /cmd_vel publisher exists per collision-monitor mode.
for token in (
    "if (collision_monitor_enabled_)",
    "final_sub_ = create_subscription<geometry_msgs::msg::Twist>",
    "final_pub_ = create_publisher<geometry_msgs::msg::Twist>",
    "if (!collision_monitor_enabled_ && final_pub_) final_pub_->publish(command)",
):
    if token not in navigation_core:
        fail(f"final command ownership contract missing: {token}")
if "estop_ || !esc_ready_ || !map_ready_" not in navigation_core:
    fail("autonomous motion gate must require a fresh ready ESC link")

# ROS 2 Humble collision-monitor parses polygon vertices from a string and uses
# max_points (newer Nav2 releases renamed this to min_points). A YAML numeric
# array is syntactically valid but fails parameter-type validation at runtime.
for path in sorted((ROOT / "config").glob("collision_monitor*.yaml")):
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    monitor = config["collision_monitor"]["ros__parameters"]
    for polygon_name in monitor.get("polygons", []):
        polygon = monitor.get(polygon_name, {})
        if not isinstance(polygon.get("points"), str):
            fail(f"Humble collision polygon must be a string: {path.name}.{polygon_name}.points")
        if "max_points" not in polygon or "min_points" in polygon:
            fail(f"Humble collision threshold must use max_points: {path.name}.{polygon_name}")


ekf = yaml.safe_load((ROOT / "config/ekf.yaml").read_text(encoding="utf-8"))
local_ekf = ekf["ekf_filter_node_odom"]["ros__parameters"]
global_ekf = ekf["ekf_filter_node_map"]["ros__parameters"]
enabled = lambda values: {index for index, value in enumerate(values) if value}
if local_ekf.get("odom0") != "/esc/odom" or enabled(local_ekf.get("odom0_config", [])) != {6, 11}:
    fail("local EKF optional ESC vx+kinematic yaw-rate fusion missing")
if local_ekf["twist0"] != "/gnss/base_velocity_fusion" or enabled(local_ekf["twist0_config"]) != {6}:
    fail("local EKF must fuse independent GNSS base vx")
if local_ekf["imu0"] != "/imu/data" or enabled(local_ekf["imu0_config"]) != {5, 11}:
    fail("local EKF IMU must fuse absolute yaw + gyro-Z")
if global_ekf["odom0"] != "/odometry/gnss_map" or enabled(global_ekf["odom0_config"]) != {0, 1}:
    fail("global EKF must fuse GNSS map x/y")
# Global COG architecture: GNSS x/y + vx, COG absolute yaw, IMU gyro-Z continuity.
if global_ekf["twist0"] != "/gnss/base_velocity_fusion" or enabled(global_ekf["twist0_config"]) != {6}:
    fail("global EKF must fuse GNSS base vx only when COG is active")
if global_ekf["pose0"] != "/gnss/cog_heading_fusion" or enabled(global_ekf["pose0_config"]) != {5}:
    fail("global EKF must fuse COG absolute yaw through pose0")
if global_ekf["imu0"] != "/imu/data" or enabled(global_ekf["imu0_config"]) != {11}:
    fail("global EKF must fuse IMU gyro-Z (vyaw) only when COG is active")
if local_ekf.get("publish_tf") is not True or global_ekf.get("publish_tf") is not False:
    fail("TF ownership must remain local EKF odom->base + LocalizationCore map->odom")

loc_cfg = yaml.safe_load((ROOT / "config/localization_cpp.yaml").read_text(encoding="utf-8"))[
    "localization_core"]["ros__parameters"]
if loc_cfg.get("gnss_require_measurement_timestamp") is not True:
    fail("GNSS measurement timestamps must stay mandatory")
if loc_cfg.get("require_gnss_velocity_certification_for_fusion") is not False:
    fail("localization bootstrap must not require ESC-dependent GNSS velocity certification")
if loc_cfg.get("enable_global_gnss_velocity_fusion") is not True:
    fail("GNSS vx/vyaw fusion must be enabled")
if loc_cfg.get("enable_global_gnss_cog_fusion") is not True:
    fail("GNSS COG absolute-yaw fusion must be enabled for production heading")
if float(loc_cfg.get("cog_min_forward_speed_mps", 0.0)) < 0.35:
    fail("COG low-speed gate must stay conservative (>=0.35 m/s)")
if float(loc_cfg.get("cog_max_sacc_mps", 999.0)) > 0.3:
    fail("COG speed-accuracy gate must stay conservative (<=0.3 m/s)")
for token in (
    "validVelocityCovarianceUnlocked",
    "gnssMeasurementStampUsableUnlocked",
    "velocityCertificationAllowsFusionUnlocked",
    "cogCertificationAllowsFusionUnlocked",
):
    if token not in localization:
        fail(f"GNSS fusion integrity implementation missing: {token}")

for token in (
    "require_fresh_gyro_for_imu_publish_ && !gyro_fresh",
    "msg.angular_velocity_covariance[0] = -1.0",
    "std::clamp<int64_t>",
    "IMU covariance diagonal must be finite and strictly positive",
    "EKF yaw now comes from the absolute IMU orientation quaternion",
):
    if token not in imu_source:
        fail(f"IMU freshness/parameter guard missing: {token}")


# Validate static-map references and raster headers without loading the large image.
for map_yaml in sorted((ROOT / "maps").rglob("*.yaml")):
    data = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
    image_name = data.get("image")
    if not image_name:
        fail(f"map YAML has no image: {map_yaml.relative_to(ROOT)}")
    image_path = (map_yaml.parent / image_name).resolve()
    if not image_path.is_file() or image_path.stat().st_size <= 16:
        fail(f"map raster missing/empty: {image_path}")
    with image_path.open("rb") as handle:
        if handle.read(2) not in {b"P2", b"P5"}:
            fail(f"map raster is not a valid PGM header: {image_path}")

print("PASS full-stack static re-audit")
print("launch/YAML/XML/parameter uniqueness: PASS")
print("command ownership + lane OFF isolation: PASS")
print("local/global fusion GNSS=vx+vyaw, IMU=yaw; map TF ownership: PASS")
