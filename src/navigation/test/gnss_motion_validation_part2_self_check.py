#!/usr/bin/env python3
from gui_source_helper import read_gui_source
"""Static contract checks for GNSS Motion Validation Part 2."""
from pathlib import Path
import sys, yaml
ROOT=Path(__file__).resolve().parents[1]
def fail(m): print('FAIL:',m,file=sys.stderr); raise SystemExit(1)
def load(p): return yaml.safe_load(p.read_text()) or {}
cfg=load(ROOT/'config/localization_cpp.yaml')['localization_core']['ros__parameters']
cpp=(ROOT/'src/localization_core.cpp').read_text()
gui = read_gui_source(ROOT) + (ROOT / 'gui/agv_gui_specs.hpp').read_text(encoding='utf-8')
ekf=(ROOT/'config/ekf.yaml').read_text()
bag=(ROOT/'tools/rosbag_regression.py').read_text()
cm=(ROOT/'CMakeLists.txt').read_text()
required={
 'gnss_velocity_topic':'/gnss/vel','gnss_velocity_fit_topic':'/gnss/velocity_position_fit',
 'gnss_motion_history_sec':6.0,'gnss_sync_max_gap_sec':0.30,'gnss_velocity_timeout_sec':0.80,
 'gnss_fit_timeout_sec':4.0,'gnss_velocity_min_validation_speed_mps':0.15,
 'gnss_speed_consistency_max_mps':0.20,'gnss_fit_speed_residual_max_mps':0.30,
 'wheel_gnss_slip_residual_mps':0.25,'cog_valid_hold_sec':1.5,'cog_invalid_hold_sec':0.5,
}
for k,v in required.items():
    if cfg.get(k)!=v: fail(f'{k}={cfg.get(k)!r}, expected {v!r}')
for token in [
 'LocalMotionSample','localStateAtUnlocked','mapHeadingForLocalStateUnlocked',
 'stampOrNow(msg->header.stamp)','gnss_map_yaw_at_measurement_rad_',
 'gnss_map_vx_mps_ = ant_map_x + omega * ry',
 'gnss_map_vy_mps_ = ant_map_y - omega * rx',
 'const double omega = gnss_yaw_rate_valid_ ? gnss_yaw_rate_rps_ : 0.0',
 '"/gnss/vel_map"','"/gnss/base_velocity"','"/gnss/motion_validation"',
 '"/gnss/velocity_qualified"','"/gnss/cog_qualified"','"/gnss/speed_residual"',
 'cog_motion_qualified_','wheel_slip_motion_detected_',
 'fix.header.stamp.sec != 0','RawMapSample{raw_map_base, quality_.hacc_m, gnss_stamp}',
]:
    if token not in cpp: fail(f'localization Part 2 contract missing: {token}')
# Raw Part-2 validation topics must never be connected directly to robot_localization.
# Part 3 is allowed to add dedicated *_fusion topics that are published only after qualification.
ekf_data=load(ROOT/'config/ekf.yaml')
all_values=[]
def collect(obj):
    if isinstance(obj,dict):
        for v in obj.values(): collect(v)
    elif isinstance(obj,list):
        for v in obj: collect(v)
    elif isinstance(obj,str): all_values.append(obj)
collect(ekf_data)
for raw_topic in ['/gnss/base_velocity','/gnss/vel_map','/gnss/vel','/gnss/cog_heading']:
    if raw_topic in all_values: fail(f'Part 2 raw-topic isolation violated: {raw_topic} wired directly into EKF')
for token in ['gnss_motion_validation','gnss_vel_map','gnss_base_vel','Motion Validation',
              'GNSS↔odom sync gap','Wheel-GNSS slip residual','yaw_at_measurement_rad']:
    if token not in gui: fail(f'GUI Part 2 contract missing: {token}')
for token in ['/gnss/vel_map','/gnss/base_velocity','/gnss/motion_validation','/gnss/velocity_qualified','/gnss/cog_qualified']:
    if token not in bag: fail(f'rosbag Part 2 topic missing: {token}')
if 'find_package(builtin_interfaces REQUIRED)' not in cm:
    fail('builtin_interfaces direct dependency missing')
print('PASS gnss_motion_validation_part2_self_check')
