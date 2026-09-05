#!/usr/bin/env python3
from gui_source_helper import read_gui_source
"""Static contract checks for Precision Part 2 localization/calibration changes."""
from pathlib import Path
import sys
try:
    import yaml
except Exception as exc:
    print(f"FAIL: python3-yaml required: {exc}", file=sys.stderr)
    raise SystemExit(2)

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parent

def fail(msg: str):
    print("FAIL:", msg, file=sys.stderr)
    raise SystemExit(1)

def load(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}

loc_cfg = load(ROOT / "config/localization_cpp.yaml")["localization_core"]["ros__parameters"]
esc_cfg = load(WS / "esc/config/ackermann.yaml")["esc_ackermann"]["ros__parameters"]
loc_cpp = (ROOT / "src/localization_core.cpp").read_text(encoding="utf-8")
gnss_cpp = (ROOT / "src/gnss_node.cpp").read_text(encoding="utf-8")
esc_cpp = (WS / "esc/src/ackermann_controller_server.cpp").read_text(encoding="utf-8")
gui = read_gui_source(ROOT) + (ROOT / 'gui/agv_gui_specs.hpp').read_text(encoding='utf-8')

# GNSS NAV-PVT COG/headAcc must be exposed append-only, but yaw correction stays safe-off by default.
for token in ["head_acc_raw", "heading_accuracy_rad_", "courseAccuracyRad", "course_enu_rad"]:
    if token not in gnss_cpp + loc_cpp:
        fail(f"GNSS course/head accuracy contract missing: {token}")
if loc_cfg.get("enable_gnss_course_yaw_correction", True):
    fail("single-antenna COG yaw correction must default OFF before field qualification")
for key in ["cog_min_forward_speed_mps", "cog_max_sacc_mps", "cog_max_heading_accuracy_rad",
            "cog_max_local_yaw_rate_rps", "cog_max_innovation_rad", "cog_yaw_alpha",
            "cog_max_yaw_step_rad", "gnss_antenna_x_m", "gnss_antenna_y_m"]:
    if key not in loc_cfg:
        fail(f"localization config missing {key}")

# Multi-point calibration must reject weak geometry and bad residuals.
for key in ["map_calibration_min_baseline_m", "map_calibration_min_geometry_score", "map_calibration_max_rmse_m"]:
    if key not in loc_cfg:
        fail(f"map calibration quality guard missing {key}")
for token in ["calibrationGeometryUnlocked", "geometry_score", "geometry_baseline", "map_calibration_max_rmse_m_"]:
    if token not in loc_cpp:
        fail(f"map calibration implementation missing {token}")

# Ackermann yaw is model-derived and must be separate from IMU-derived local yaw.
if loc_cfg.get("esc_kinematic_yaw_rate_topic") != "/esc/kinematic_yaw_rate_rps":
    fail("explicit ESC kinematic yaw diagnostic topic missing from localization config")
for token in ["/esc/kinematic_yaw_rate_rps", "yaw_rate_kinematic_pub_", "esc_kinematic_yaw_rate_rps_",
              "yaw_model_imu_residual", "ackermann_w"]:
    if token not in esc_cpp + loc_cpp:
        fail(f"Ackermann-vs-IMU residual contract missing {token}")

# Adaptive odometry covariance must exist in both config and runtime.
for key in ["odom_v_variance_base", "odom_v_variance_rpm_error_gain",
            "odom_yaw_variance_base", "odom_yaw_variance_steer_gain",
            "odom_yaw_rate_variance_base"]:
    if key not in esc_cfg:
        fail(f"ESC adaptive covariance YAML missing {key}")
    if key not in esc_cpp:
        fail(f"ESC adaptive covariance runtime missing {key}")

# GUI must provide data-driven calibration and reproducibility audit.
for token in ["class ImuCalibrationPage", "class GnssCalibrationPage",
              "Verifikasi Runtime", "void getParameters", "session_id", "reload()",
              "Ackermann w", "model-IMU residual"]:
    if token not in gui:
        fail(f"GUI Part2 feature missing {token}")

# Runtime and GUI should resolve a common active config root.
for rel in ["launch/autonomous.launch.py", "launch/imu.launch.py", "launch/gnss.launch.py", "launch/gui.launch.py"]:
    text = (ROOT / rel).read_text(encoding="utf-8")
    if "AGV_CONFIG_DIR" not in text or "_active_config_dir" not in text:
        fail(f"{rel}: active navigation config root contract missing")
for env_name in ["AGV_CONFIG_DIR", "AGV_ESC_CONFIG_DIR", "AGV_PERCEPTION_CONFIG_DIR"]:
    if env_name not in gui:
        fail(f"GUI does not honor {env_name}")
esc_launch = (WS / "esc/launch/esc.launch.py").read_text(encoding="utf-8")
if "AGV_ESC_CONFIG_DIR" not in esc_launch or "ackermann.yaml" not in esc_launch:
    fail("ESC launch does not honor writable active config root")
auto_launch = (ROOT / "launch/autonomous.launch.py").read_text(encoding="utf-8")
if "AGV_PERCEPTION_CONFIG_DIR" not in auto_launch:
    fail("autonomous launch does not honor perception active config root")

# Revised local EKF ownership: GNSS and ESC supply independent vx; IMU supplies relative yaw + gyro-Z.
ekf = load(ROOT / "config/ekf.yaml")
local = ekf["ekf_filter_node_odom"]["ros__parameters"]
if local.get("twist0") != "/gnss/base_velocity_fusion" or local.get("imu0") != "/imu/data":
    fail("local EKF must use gated GNSS velocity plus IMU")
if [i for i,v in enumerate(local["twist0_config"]) if v] != [6]:
    fail("local EKF must take independent vx from GNSS velocity fusion")
if [i for i,v in enumerate(local["imu0_config"]) if v] != [5, 11]:
    fail("local EKF must take relative yaw + gyro-Z from IMU")
if local.get('imu0_relative') is not True:
    fail("local EKF IMU yaw must remain relative")
if local.get('odom0') != '/esc/odom' or {i for i,v in enumerate(local.get('odom0_config',[])) if v} != {6}:
    fail("local EKF must accept optional ESC vx only; kinematic yaw-rate stays diagnostic")

print("PASS precision_part2_self_check")
