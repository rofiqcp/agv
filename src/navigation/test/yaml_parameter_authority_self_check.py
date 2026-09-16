#!/usr/bin/env python3
"""Fail if a runtime ROS setting exists only in source/launch instead of YAML."""
from pathlib import Path
import re
import yaml

AGV = Path(__file__).resolve().parents[3]
SRC = AGV / "src"
META = SRC / "navigation/web/config/ui_parameter_metadata.yaml"

# 1) Every declared runtime parameter must have a YAML authority somewhere under src.
yaml_param_names = set()
for path in SRC.rglob("*.yaml"):
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except Exception:
        continue
    def walk(node, stack=()):
        if isinstance(node, dict):
            for key, value in node.items():
                nxt = stack + (str(key),)
                if "ros__parameters" in nxt and len(nxt) > nxt.index("ros__parameters") + 1:
                    yaml_param_names.add(nxt[-1])
                walk(value, nxt)
        elif isinstance(node, list):
            for value in node:
                walk(value, stack)
    walk(data)

decl = re.compile(r"declare_parameter(?:<[^>]+>)?\s*\(\s*[\"']([^\"']+)[\"']")
missing = []
for path in SRC.rglob("*"):
    if path.suffix not in {".cpp", ".hpp", ".py"} or "test" in path.parts:
        continue
    text = path.read_text(errors="ignore")
    for name in decl.findall(text):
        if name not in yaml_param_names:
            missing.append(f"{path.relative_to(AGV)}:{name}")
assert not missing, "Declared parameters without YAML authority:\n" + "\n".join(missing)

# 2) Every ROS-Web registered YAML leaf must have complete metadata, so it is
# visible and can be staged/validated (or explicitly calibration-generated).
registered = {
    "vehicle": "src/navigation/config/vehicle.yaml",
    "navigation_core": "src/navigation/config/navigation_core.yaml",
    "nav2": "src/navigation/config/nav2_ackermann.yaml",
    "ekf": "src/navigation/config/ekf.yaml",
    "localization": "src/navigation/config/localization_cpp.yaml",
    "gnss": "src/navigation/config/gnss.yaml",
    "imu": "src/navigation/config/imu.yaml",
    "mag_heading": "src/navigation/config/mag_heading.yaml",
    "imu_speed": "src/navigation/config/imu_speed.yaml",
    "stage3": "src/navigation/config/stage3_navigation.yaml",
    "trajectory_safety": "src/navigation/config/trajectory_safety.yaml",
    "collision": "src/navigation/config/collision_monitor_production.yaml",
    "mppi_closed_loop": "src/navigation/config/mppi_closed_loop.yaml",
    "gui": "src/navigation/config/gui_calibration.yaml",
    "esc": "src/esc/config/ackermann.yaml",
    "teleop": "src/esc/config/teleop.yaml",
    "foc_thesis": "src/esc/config/foc_thesis.yaml",
    "vesc_tool": "src/esc/config/vesc_tool.yaml",
    "hmi": "src/stmf4/config/hmi.yaml",
    "perception": "src/perception/config/astra_yolop_gpu.yaml",
    "bbox_calibration": "src/perception/config/bbox_obstacle_calibration.yaml",
}
metadata = (yaml.safe_load(META.read_text()) or {}).get("parameters", {}) or {}

def leaves(node, prefix=""):
    if isinstance(node, dict):
        for key, value in node.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            yield from leaves(value, path)
    else:
        yield prefix

meta_missing = []
for file_key, rel in registered.items():
    data = yaml.safe_load((AGV / rel).read_text()) or {}
    for path in leaves(data):
        identity = f"{file_key}:{path}"
        row = metadata.get(identity, {})
        if not row or row.get("metadata_complete") is not True:
            meta_missing.append(identity)
assert not meta_missing, "ROS Web metadata incomplete:\n" + "\n".join(meta_missing)

# 3) Main launch must load custom-node settings from YAML rather than literal maps.
auto = (SRC / "navigation/launch/autonomous.launch.py").read_text()
required = [
    "parameters=[navigation_core_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],",
    "parameters=[camera_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],",
]
for token in required:
    assert token in auto, f"YAML-backed launch contract missing: {token}"
for forbidden in [
    "'score_threshold': 0.20",
    "'inference_hz': 1.0",
    "'camera_jpeg_fps': 5.0",
    "'source_topic': '/navigation/cmd_mux/source'",
]:
    assert forbidden not in auto, f"Hardcoded runtime setting returned to autonomous.launch.py: {forbidden}"
assert "def _yaml_ros_param(path: str, node_name: str, key: str):" in auto
assert ".get(key, default)" not in auto

print(f"PASS YAML authority: {len(yaml_param_names)} parameter names; {len(metadata)} ROS-Web metadata entries")
