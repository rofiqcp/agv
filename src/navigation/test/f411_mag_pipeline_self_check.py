#!/usr/bin/env python3
"""Static contract for F411 GNSS/IST8310 -> magnetic heading -> global EKF pipeline."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
launch = (ROOT / "launch/autonomous.launch.py").read_text(encoding="utf-8")
ekf = yaml.safe_load((ROOT / "config/ekf.yaml").read_text(encoding="utf-8"))
mag = yaml.safe_load((ROOT / "config/mag_heading.yaml").read_text(encoding="utf-8"))

def req(ok, msg):
    if not ok:
        raise AssertionError(msg)

req("DeclareLaunchArgument('hmi_port', default_value='auto')" in launch, "F411 launch selector must default to auto")
req("mag_heading_fusion = Node(" in launch, "mag_heading_fusion node definition missing")
req("delayed_ekf = TimerAction(period=5.0, actions=[local_ekf, global_ekf])" in launch, "delayed EKF startup contract missing")
req("delayed_ekf, localization_core, mag_heading_fusion, imu_speed_diagnostic" in launch, "delayed EKF / mag fusion actions are not in LaunchDescription")
req("'gnss_source'" in launch and "stm32=NEO3 via HMI USB CDC" in launch, "STM32 NEO3 launch contract missing")
mp = mag['mag_heading_fusion']['ros__parameters']
req(mp['neo3_mag_topic'] == '/neo3/mag', 'NEO3 magnetometer topic mismatch')
req(mp['imu_mag_topic'] == '/imu/mag', 'IMU magnetometer topic mismatch')
req(mp['map_yaw_topic'] == '/localization/map_yaw_from_enu', 'map yaw dependency mismatch')
g = ekf['ekf_filter_node_map']['ros__parameters']
req(g['pose1'] == '/neo3/mag_heading_fusion', 'global EKF NEO3 magnetic heading input mismatch')
req(g['pose2'] == '/imu/mag_heading_fusion', 'global EKF IMU magnetic heading input mismatch')
req(g['pose0'] == '/gnss/cog_heading_fusion', 'global EKF COG heading input mismatch')
print('PASS f411_mag_pipeline_self_check')
