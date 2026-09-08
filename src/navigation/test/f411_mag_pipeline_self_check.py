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
req("delayed_ekf = TimerAction(period=20.0, actions=[local_ekf, global_ekf])" in launch, "delayed EKF startup contract missing")
req("delayed_ekf, localization_core, mag_heading_fusion, imu_speed_diagnostic" in launch, "delayed EKF / mag fusion actions are not in LaunchDescription")
req("'gnss_source'" in launch and "stm32=NEO3 via HMI USB CDC" in launch, "STM32 NEO3 launch contract missing")
mp = mag['mag_heading_fusion']['ros__parameters']
req(mp['neo3_mag_topic'] == '/neo3/mag', 'NEO3 magnetometer topic mismatch')
req(mp['imu_mag_topic'] == '/imu/mag', 'IMU magnetometer topic mismatch')
req(mp['map_yaw_topic'] == '/localization/map_yaw_from_enu', 'map yaw dependency mismatch')
req(mp.get('neo3_mag_yaw_sign') == 1.0, '8-direction calibration requires corrected NEO3 yaw sign')
req(mp.get('neo3_planar_calibration_enabled') is True, 'NEO3 planar calibration must be enabled')
req(len(mp.get('neo3_mag_bias_xy_ut', [])) == 2, 'NEO3 XY bias calibration missing')
req(len(mp.get('neo3_mag_matrix_xy', [])) == 4, 'NEO3 XY calibration matrix missing')
req(mp.get('neo3_heading_lut_enabled') is True, 'NEO3 8-direction heading LUT must be enabled')
req(len(mp.get('neo3_heading_lut_input_rad', [])) == 8, 'NEO3 heading LUT must contain 8 knots')
req(len(mp.get('neo3_heading_lut_correction_rad', [])) == 8, 'NEO3 correction LUT must contain 8 values')
cal = yaml.safe_load((ROOT / 'config/heading_8dir_calibration.yaml').read_text(encoding='utf-8'))['heading_8dir_calibration']
req(cal.get('valid') is True and cal.get('validation', {}).get('pass') is True, '8-direction calibration evidence must be valid')
req(float(cal['validation']['static_heading_max_abs_error_deg']) < 5.0, '8-direction replay must remain inside consensus gate')
g = ekf['ekf_filter_node_map']['ros__parameters']
req(g['pose1'] == '/heading/validated_fusion', 'global EKF validated heading input mismatch')
req('pose2' not in g, 'global EKF must not fuse raw second magnetic heading directly')
req(mp['neo3_heading_topic'] == '/neo3/mag_heading_fusion', 'NEO3 diagnostic heading topic mismatch')
req(mp['imu_heading_topic'] == '/imu/mag_heading_fusion', 'Yahboom magnetic diagnostic heading topic mismatch')
req(g['pose0'] == '/gnss/cog_heading_fusion', 'global EKF COG heading input mismatch')
print('PASS f411_mag_pipeline_self_check')
