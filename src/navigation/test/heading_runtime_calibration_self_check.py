#!/usr/bin/env python3
"""Fail if active heading runtime drifts from the latest accepted 360deg calibration."""
from pathlib import Path
import math
import yaml

NAV_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = NAV_ROOT.parents[1]
CAL = WORKSPACE / 'calibration' / 'heading_360_latest.yaml'
CFG = NAV_ROOT / 'config' / 'mag_heading.yaml'
SRC = NAV_ROOT / 'src' / 'mag_heading_fusion_node.cpp'


def require(cond, msg):
    if not cond:
        raise SystemExit('HEADING_RUNTIME_CALIBRATION_FAIL: ' + msg)


def same(a, b, tol=1e-12):
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(same(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isfinite(float(a)) and math.isfinite(float(b)) and abs(float(a) - float(b)) <= tol
    return a == b


require(CAL.is_file(), f'latest calibration artifact missing: {CAL}')
cal = yaml.safe_load(CAL.read_text())['heading_360_calibration']
cfg = yaml.safe_load(CFG.read_text())['mag_heading_fusion']['ros__parameters']
require(cal.get('valid') is True and cal.get('ready_for_stage2') is True, 'latest calibration is not accepted')
require(cal.get('calibration_owner') == 'ros_host', 'calibration owner must be ros_host')
require(cal['yahboom']['validation']['pass'] is True, 'Yahboom calibration validation failed')
require(cal['neo3']['validation']['pass'] is True, 'RM/NEO3 calibration validation failed')

pairs = {
    'imu_mag_bias_xy_lsb': cal['yahboom']['bias'],
    'imu_mag_matrix_xy_per_lsb': cal['yahboom']['matrix'],
    'imu_mag_yaw_sign': cal['yahboom']['yaw_sign'],
    'imu_mag_yaw_offset_rad': cal['yahboom']['yaw_offset_rad'],
    'imu_heading_lut_input_rad': cal['yahboom']['heading_lut_input_rad'],
    'imu_heading_lut_correction_rad': cal['yahboom']['heading_lut_correction_rad'],
    'neo3_mag_bias_xy_ut': cal['neo3']['bias'],
    'neo3_mag_matrix_xy': cal['neo3']['matrix'],
    'neo3_mag_yaw_sign': cal['neo3']['yaw_sign'],
    'neo3_mag_yaw_offset_rad': cal['neo3']['yaw_offset_rad'],
    'neo3_heading_lut_input_rad': cal['neo3']['heading_lut_input_rad'],
    'neo3_heading_lut_correction_rad': cal['neo3']['heading_lut_correction_rad'],
}
for key, expected in pairs.items():
    require(key in cfg, f'active runtime parameter missing: {key}')
    require(same(cfg[key], expected), f'{key} differs from latest calibration')

require(cfg.get('imu_planar_calibration_enabled') is True, 'Yahboom planar calibration not enabled')
require(cfg.get('imu_heading_lut_enabled') is True, 'Yahboom heading LUT not enabled')
require(cfg.get('neo3_planar_calibration_enabled') is True or cfg.get('neo3_full_calibration_enabled') is True,
        'RM/NEO3 calibration not enabled')
require(cfg.get('neo3_heading_lut_enabled') is True, 'RM/NEO3 heading LUT not enabled')
require(cfg.get('neo3_calibration_owner') == 'ros_host' and cfg.get('neo3_calibration_ownership_verified') is True,
        'RM/NEO3 calibration ownership is not verified')

src = SRC.read_text()
for token in (
    'x-imu_bias_x_lsb_', 'imu_m00_*bx + imu_m01_*by', 'applyImuHeadingLut(yaw_enu)',
    'mx - neo_bias_x_ut_', 'neo_m00_ * bx + neo_m01_ * by', 'applyNeoHeadingLut(yaw_enu)',
    'imu_heading_pub_->publish', 'neo_heading_pub_)->publish',
):
    require(token in src, f'calibration application code missing: {token}')

print('HEADING_RUNTIME_CALIBRATION_PASS',
      f"yah_rms={cal['yahboom']['validation']['fit_rms_error_deg']:.3f}deg",
      f"rm_rms={cal['neo3']['validation']['fit_rms_error_deg']:.3f}deg",
      f"source={cal.get('raw_csv','--')}")
