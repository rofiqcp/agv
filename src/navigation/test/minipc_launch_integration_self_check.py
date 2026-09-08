#!/usr/bin/env python3
"""Static integration contract for the Mini-PC autonomous/gui entry points."""
from __future__ import annotations
from gui_source_helper import read_gui_source
import ast
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = ROOT.parent
ESC = WORKSPACE_SRC / "esc"
PERCEPTION = WORKSPACE_SRC / "perception"
errors: list[str] = []

def need(cond: bool, message: str) -> None:
    if not cond:
        errors.append(message)

def params(path: Path, node: str) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return data[node]["ros__parameters"]

def enabled(v):
    return {i for i, x in enumerate(v or []) if bool(x)}

need(not (ROOT / "launch/minipc.launch.py").exists(), "minipc.launch.py must be removed")
for name in ("autonomous.launch.py", "gui.launch.py"):
    path = ROOT / "launch" / name
    need(path.is_file(), f"{name} missing")
    if path.is_file():
        src = path.read_text(encoding="utf-8")
        try:
            ast.parse(src)
        except SyntaxError as exc:
            errors.append(f"{name} syntax: {exc}")
        for token in ("perception_mode", "pt_model_path", "engine_path", "esc_serial_enabled",
                      "gnss_port", "imu_port", "rgb_device"):
            need(token in src, f"{name} missing {token}")

launch = (ROOT / "launch/autonomous.launch.py").read_text(encoding="utf-8")
need("camera_only_node" in launch and "camera_only_enabled" in launch,
     "perception_mode=off must launch camera_only_node")
need("perception_cpu_node" in launch and "pt_model_path" in launch,
     "CPU direct .pt backend missing")
need("importlib.util.find_spec" not in launch, "autonomous launch must not probe Python ONNX/Torch modules")
need("_discover_cpu_model" in launch and "YOLOPV2_PT_PATH" in launch,
     "Mini-PC portable PT discovery missing")
need("DeclareLaunchArgument('mode', default_value='web'" in launch,
     "autonomous mode selector must default to web")
need("mode not in {'rviz', 'gui', 'web'}" in launch,
     "autonomous mode validation contract missing")
need("condition=IfCondition(mode_gui)" in launch and "executable='agv_gui'" in launch,
     "mode=gui native GUI condition missing")
need("condition=IfCondition(mode_web)" in launch and "executable='agv_web_gui'" in launch,
     "mode=web web GUI condition missing")
need("condition=IfCondition(mode_rviz)" in launch and "executable='rviz2'" in launch,
     "mode=rviz standalone RViz condition missing")
need("condition=IfCondition(LaunchConfiguration('start_web_gui'))" not in launch,
     "legacy start_web_gui must not override authoritative mode selector")
need("condition=IfCondition(LaunchConfiguration('enable_rviz'))" not in launch,
     "legacy enable_rviz must not override authoritative mode selector")

perception_cmake = (PERCEPTION / "CMakeLists.txt").read_text(encoding="utf-8")
need("find_package(Torch QUIET)" in perception_cmake and "PERCEPTION_CPU_AVAILABLE" in perception_cmake, "LibTorch capability detection missing")
need("add_executable(perception_cpu_node src/astra_yolop_cpu_pt_node.cpp)" in perception_cmake,
     "direct PT CPU target missing")
need("add_executable(camera_only_node src/camera_only_node.cpp)" in perception_cmake,
     "camera-only target missing")
need("RENAME perception_cpu_node" not in perception_cmake,
     "legacy Python PT->ONNX launcher still owns perception_cpu_node")

per_cfg = params(PERCEPTION / "config/astra_yolop_gpu.yaml", "perception")
need(per_cfg.get("perception_mode") == "off", "Mini-PC perception_mode YAML must default OFF for lazy YOLO toggle")
need(per_cfg.get("pt_model_path") == "auto", "PT path YAML must use portable auto discovery")
need(per_cfg.get("inference_enabled") is False, "YOLO inference must default OFF")
need(int(per_cfg.get("cpu_threads", -1)) == 2, "CPU threads YAML must default to bounded 2-thread Mini-PC budget")

# Sensor fusion contract.
ekf = yaml.safe_load((ROOT / "config/ekf.yaml").read_text(encoding="utf-8"))
local = ekf["ekf_filter_node_odom"]["ros__parameters"]
global_ = ekf["ekf_filter_node_map"]["ros__parameters"]
need(local.get("odom0") == "/esc/odom" and enabled(local.get("odom0_config")) == {6},
     "local EKF must fuse ESC vx only; kinematic yaw-rate is diagnostic")
need(local.get("twist0") == "/gnss/base_velocity_fusion" and enabled(local.get("twist0_config")) == {6},
     "local EKF must fuse independent GNSS vx")
need(local.get("imu0") == "/imu/data" and enabled(local.get("imu0_config")) == {11} and local.get("imu0_relative") is True,
     "local EKF must fuse IMU gyro-Z only")
need(global_.get("odom0") == "/odometry/gnss_map" and enabled(global_.get("odom0_config")) == {0, 1},
     "global EKF must fuse GNSS map x/y")
need(global_.get("twist0") == "/gnss/base_velocity_fusion" and enabled(global_.get("twist0_config")) == {6},
     "global EKF must fuse GNSS vx")
need(global_.get("pose0") == "/gnss/cog_heading_fusion" and enabled(global_.get("pose0_config")) == {5},
     "global EKF COG heading source invalid")
need(global_.get("pose1") == "/heading/validated_fusion" and enabled(global_.get("pose1_config")) == {5},
     "global EKF validated heading source invalid")
need("pose2" not in global_, "global EKF must not fuse raw second magnetic heading directly")
need(global_.get("imu0") == "/imu/data" and enabled(global_.get("imu0_config")) == {11} and global_.get("imu0_relative") is True,
     "global EKF must fuse IMU gyro-Z only")

esc = params(ESC / "config/ackermann.yaml", "esc_ackermann")
esc_launch = (ESC / "launch/esc.launch.py").read_text(encoding="utf-8")
need("serial_enabled" not in esc, "ESC serial_enabled must be launch-owned, not node-scoped YAML")
need('DeclareLaunchArgument("serial_enabled", default_value="true")' in esc_launch and
     '"serial_enabled": ParameterValue(LaunchConfiguration("serial_enabled"), value_type=bool)' in esc_launch,
     "ESC launch-owned serial_enabled contract missing")

gui = read_gui_source(ROOT)
need("perception.ros__parameters.perception_mode" in gui, "GUI perception mode YAML selector missing")
need("Restore EKF Sensor Policy" in gui, "GUI estimator policy restore action missing")

if errors:
    print("AUTONOMOUS/GUI INTEGRATION SELF-CHECK: FAIL")
    for item in errors:
        print(" -", item)
    sys.exit(1)
print("AUTONOMOUS/GUI INTEGRATION SELF-CHECK: PASS")
