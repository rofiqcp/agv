#!/usr/bin/env python3
"""Static contract: NEO3 Pro RM3100 is the sole magnetometer."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
mag = yaml.safe_load((ROOT / 'config/mag_heading.yaml').read_text())['mag_heading_fusion']['ros__parameters']
imu = yaml.safe_load((ROOT / 'config/imu.yaml').read_text())['data_imu_node']['ros__parameters']
ekf_all = yaml.safe_load((ROOT / 'config/ekf.yaml').read_text())
ekf = ekf_all['ekf_filter_node_map']['ros__parameters']
ekf_local = ekf_all['ekf_filter_node_odom']['ros__parameters']
node = (ROOT / 'src/mag_heading_fusion_node.cpp').read_text()
imu_src = (ROOT / 'src/imu_node.cpp').read_text()
web = (ROOT / 'web/web_server.cpp').read_text()

def req(ok, msg):
    if not ok:
        raise AssertionError(msg)

req(mag['rm3100_mag_topic'] == '/neo3pro/mag', 'RM3100 raw topic mismatch')
req(mag['heading_topic'] == '/neo3pro/mag_heading_fusion', 'RM3100 heading topic mismatch')
req(mag['calibrated_mag_topic'] == '/neo3pro/mag_calibrated', 'RM3100 calibrated topic mismatch')
req(mag['validated_heading_topic'] == '/heading/validated_fusion', 'validated heading topic mismatch')
req(mag['rm3100_calibration_owner'] in ('ros_host', 'ap_periph'), 'calibration owner invalid')
req(mag['rm3100_calibration_owner'] == 'ros_host', 'production RM3100 calibration must be owned by ROS host')
req(mag['rm3100_full_calibration_enabled'] is True, 'production RM3100 Full-3D calibration must be enabled')
req(mag['rm3100_planar_calibration_enabled'] is False, 'legacy planar calibration must be disabled after Full-3D apply')
req(mag['rm3100_heading_lut_enabled'] is False, 'legacy heading LUT must be disabled after Full-3D apply')
req(len(mag['rm3100_mag_bias_xyz_ut']) == 3, 'full-3D bias incomplete')
req(len(mag['rm3100_mag_matrix_3x3']) == 9, 'full-3D matrix incomplete')
req(any(abs(float(v)) > 1e-9 for v in mag['rm3100_mag_bias_xyz_ut']), 'Full-3D bias still identity/zero')
identity=[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
req(any(abs(float(a)-b) > 1e-6 for a,b in zip(mag['rm3100_mag_matrix_3x3'],identity)), 'Full-3D matrix still identity')
req(len(mag['rm3100_mag_bias_xy_ut']) == 2, 'planar bias incomplete')
req(len(mag['rm3100_mag_matrix_xy']) == 4, 'planar matrix incomplete')
req(int(imu['output_content_mask']) == 14, 'Yahboom RSW must be ACC+GYRO+ANGLE only (0x000E)')
req('/imu/mag' not in node, 'heading node must not subscribe Yahboom magnetometer')
req('imu_mag_heading' not in node, 'legacy Yahboom magnetic heading remains')
req('case 0x54' in imu_src, 'driver must explicitly handle legacy MAG frame')
req('legacy magnetometer frame intentionally ignored' in imu_src, 'legacy MAG frame must be ignored')
req('/heading/validated_fusion' in [v for k,v in ekf.items() if str(k).startswith('pose') and isinstance(v,str)], 'global EKF validated heading missing')
req('/heading/validated_fusion' not in [v for k,v in ekf_local.items() if (str(k).startswith('pose') or str(k).startswith('odom') or str(k).startswith('imu')) and isinstance(v,str)], 'local EKF must stay continuous odom/gyro and must not consume absolute magnetic yaw directly')
req('/neo3pro/mag' in web, 'ROS Web RM3100 binding missing')
req('/neo3/mag' not in web, 'legacy /neo3/mag binding remains')
req('/imu/mag' not in web, 'legacy Yahboom MAG binding remains')
req('/neo3pro/mag_heading_fusion' in web, 'RM3100 heading binding missing')
req('/neo3pro/mag_heading_valid' in web, 'RM3100 heading-valid binding missing')
req('if (!calibration_applied_) return;' in node, 'validated heading must fail closed until calibration is applied')
req('const bool validated_live=live && calibration_applied_;' in node, 'validated freshness must require applied calibration')
req('rm3100_calibration_applied=' in node, 'calibration-applied status telemetry missing')

print('PASS f411_mag_pipeline_self_check: RM3100-only magnetic authority')
