#!/usr/bin/env python3
"""Static contract for F411 GNSS/IST8310 -> magnetic heading -> global EKF pipeline."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
launch = (ROOT / "launch/autonomous.launch.py").read_text(encoding="utf-8")
ekf = yaml.safe_load((ROOT / "config/ekf.yaml").read_text(encoding="utf-8"))
mag = yaml.safe_load((ROOT / "config/mag_heading.yaml").read_text(encoding="utf-8"))
imu = yaml.safe_load((ROOT / "config/imu.yaml").read_text(encoding="utf-8"))
imu_src = (ROOT / "src/imu_node.cpp").read_text(encoding="utf-8")

def req(ok, msg):
    if not ok:
        raise AssertionError(msg)

req("DeclareLaunchArgument('hmi_port', default_value='auto')" in launch, "F411 launch selector must default to auto")
req("mag_heading_fusion = Node(" in launch, "mag_heading_fusion node definition missing")
req("delayed_ekf = TimerAction(period=20.0, actions=[local_ekf, global_ekf])" in launch, "delayed EKF startup contract missing")
for action in ("delayed_ekf", "localization_core", "sensor_contract_monitor",
               "precision_localization_monitor", "mag_heading_fusion", "imu_speed_diagnostic"):
    req(action in launch, f"autonomous LaunchDescription missing action {action}")
return_idx = launch.find("return LaunchDescription")
order = [launch.find(token, return_idx) for token in
         ("delayed_ekf", "localization_core", "sensor_contract_monitor",
          "precision_localization_monitor", "mag_heading_fusion", "imu_speed_diagnostic")]
req(all(i >= 0 for i in order) and order == sorted(order),
    "localization/safety/magnetic actions are not ordered deterministically in LaunchDescription")
req("'gnss_source'" in launch and "stm32=NEO3 via HMI USB CDC" in launch, "STM32 NEO3 launch contract missing")
mp = mag['mag_heading_fusion']['ros__parameters']
req(mp['neo3_mag_topic'] == '/neo3/mag', 'NEO3 magnetometer topic mismatch')
req(mp['imu_mag_topic'] == '/imu/mag', 'IMU magnetometer topic mismatch')
req(mp['map_yaw_topic'] == '/localization/map_yaw_from_enu', 'map yaw dependency mismatch')
req(mp.get('enable_imu_mag_heading') is True, 'Yahboom calibrated MAG diagnostic must be enabled after 8-direction validation')
req(mp.get('imu_raw_mag_topic') == '/imu/mag_raw_lsb', 'Yahboom calibration must consume manufacturer raw LSB topic')
req(mp.get('imu_planar_calibration_enabled') is True, 'Yahboom planar calibration must be enabled')
req(len(mp.get('imu_mag_bias_xy_lsb', [])) == 2, 'Yahboom XY hard-iron bias missing')
req(len(mp.get('imu_mag_matrix_xy_per_lsb', [])) == 4, 'Yahboom XY soft-iron matrix missing')
req(mp.get('imu_heading_lut_enabled') is True, 'Yahboom 8-direction heading LUT must be enabled')
req(len(mp.get('imu_heading_lut_input_rad', [])) == 8 and len(mp.get('imu_heading_lut_correction_rad', [])) == 8, 'Yahboom LUT must contain 8 knots/corrections')
ycal = yaml.safe_load((ROOT / 'config/yahboom_mag_calibration.yaml').read_text(encoding='utf-8'))['yahboom_mag_calibration']
req(ycal.get('valid') is True and ycal.get('validation', {}).get('pass') is True, 'Yahboom 8-direction calibration evidence must pass')
req(float(ycal['validation']['rms_error_deg']) < 3.0 and float(ycal['validation']['max_abs_error_deg']) < 5.0, 'Yahboom replay must remain inside calibration gates')
ip = imu['data_imu_node']['ros__parameters']
req(ip.get('baudrate') == 921600 and ip.get('auto_baud') is True, 'Yahboom serial must prefer/probe 921600 baud')
req(ip.get('publish_rate_hz') == 50 and ip.get('output_rate_code') == 8, 'Yahboom hardware/ROS rate must be 50 Hz')
req(ip.get('output_content_mask') == 30, 'Yahboom stream must contain ACC+GYRO+ANGLE+MAG')
req(ip.get('configure_algorithm_on_connect') is True and ip.get('algorithm_mode') == 1, 'Yahboom must run 6-axis relative-yaw mode while MAG is calibrated externally')
req(ip.get('vector_x_sign') == -1.0 and ip.get('vector_y_sign') == -1.0 and ip.get('vector_z_sign') == 1.0, 'Yahboom vectors must apply Rz(pi): [-X,-Y,+Z] into REP-103 body frame')
req(ip.get('yaw_sign') == 1.0, 'Yahboom gyro/yaw sign must follow REP-103: +Z is CCW')
req(ip.get('publish_mag_tesla') is False and float(ip.get('mag_scale_tesla_per_lsb', -1.0)) == 0.0, 'Do not publish fabricated Tesla units for Yahboom raw LSB')
req('imu/mag_raw_lsb' in imu_src and 'vector_z_sign_ * gz_' in imu_src, 'Yahboom raw MAG/body-frame and gyro-Z contract missing in driver')
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
