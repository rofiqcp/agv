#!/usr/bin/env python3
from gui_source_helper import read_gui_source
"""Regression guard for the revised GNSS/IMU localization foundation."""
from pathlib import Path
import math
import yaml

ROOT=Path(__file__).resolve().parents[1]
SRC_ROOT=ROOT.parent

def load_params(path,node):
    return (yaml.safe_load(path.read_text(encoding='utf-8')) or {})[node]['ros__parameters']
def require(ok,msg):
    if not ok: raise AssertionError(msg)
def enabled(cfg): return {i for i,v in enumerate(cfg or []) if v}

loc=load_params(ROOT/'config/localization_cpp.yaml','localization_core')
imu=load_params(ROOT/'config/imu.yaml','data_imu_node')
vehicle=load_params(ROOT/'config/vehicle.yaml','vehicle')
navcore=load_params(ROOT/'config/navigation_core.yaml','navigation_core')
ekf=yaml.safe_load((ROOT/'config/ekf.yaml').read_text())
local=ekf['ekf_filter_node_odom']['ros__parameters']
global_=ekf['ekf_filter_node_map']['ros__parameters']

# 1) TF and sensor ownership: no ESC dependency in estimator startup.
require(local.get('publish_tf') is True,'Local EKF must own odom->base TF')
require(global_.get('publish_tf') is False,'Global EKF must not publish map->odom TF')
require(local.get('odom0')=='/esc/odom' and enabled(local.get('odom0_config'))=={6},'Local EKF optional ESC odom must remain vx-only; steering/encoder yaw is not heading authority')
require(local.get('twist0')=='/gnss/base_velocity_fusion' and enabled(local.get('twist0_config'))=={6},
        'Local EKF must use independent GNSS vx')
require(local.get('imu0')=='/imu/data' and enabled(local.get('imu0_config'))=={5,11} and local.get('imu0_relative') is True,
        'Local EKF must use relative IMU yaw + gyro-Z')
require(global_.get('odom0')=='/odometry/gnss_map' and enabled(global_.get('odom0_config'))=={0,1},
        'Global EKF GNSS x/y source changed')
require(global_.get('twist0')=='/gnss/base_velocity_fusion' and enabled(global_.get('twist0_config'))=={6},
        'Global EKF must use GNSS vx only; yaw-rate authority comes from IMU gyro')
require(global_.get('imu0')=='/imu/data' and enabled(global_.get('imu0_config'))=={5,11} and global_.get('imu0_relative') is True,
        'Global EKF must use relative IMU yaw + gyro-Z')
for key, topic in (('pose0','/gnss/cog_heading_fusion'),
                   ('pose1','/neo3/mag_heading_fusion'),
                   ('pose2','/imu/mag_heading_fusion')):
    require(global_.get(key)==topic and enabled(global_.get(key+'_config'))=={5},
            f'Global EKF absolute heading source invalid: {key}={topic}')

# 2) Integrity gates remain, while velocity bootstrap no longer depends on ESC certification.
require(loc.get('gnss_require_measurement_timestamp') is True,'GNSS timestamp must be mandatory')
for key in ('gnss_max_future_stamp_sec','gnss_max_measurement_age_sec','gnss_max_stamp_regression_sec','gnss_quality_timeout_sec'):
    value=float(loc[key]); require(math.isfinite(value) and value>=0.0,f'bad {key}')
require(0.0<float(loc['gnss_velocity_covariance_min_variance'])<float(loc['gnss_velocity_covariance_max_variance']),
        'GNSS covariance bounds invalid')
require(loc.get('require_gnss_velocity_certification_for_fusion') is False,
        'GNSS velocity bootstrap must not require ESC-dependent field certification')
require(loc.get('enable_global_gnss_velocity_fusion') is True,'GNSS velocity fusion must be enabled')
require(isinstance(loc.get('enable_global_gnss_cog_fusion'),bool),'COG fusion flag must be boolean')
require(loc.get('enable_gnss_course_yaw_correction') is False,'legacy GNSS yaw correction must stay off')
require(float(loc.get('cog_min_forward_speed_mps',999.0)) <= 0.18,
        'COG heading qualification must be reachable at Stage-3 commissioning speed')
require(float(loc.get('global_ekf_yaw_max_innovation_rad',0.0)) >= math.pi/2,
        'COG correction must recover a >=90deg bad startup heading seed')
require(loc.get('anchor_init_requires_strict') is False,'map display bootstrap should allow sane degraded GNSS')
for key in ('gnss_yaw_rate_min_speed_mps','gnss_yaw_rate_max_abs_rps','gnss_yaw_rate_filter_alpha',
            'gnss_yaw_rate_min_variance','gnss_yaw_rate_max_variance'):
    require(key in loc,f'missing GNSS vyaw setting {key}')

# 3) Field evidence is still retained for commissioning/autonomy, not estimator existence.
for key in ('gnss_velocity_calibration_valid','gnss_cog_calibration_valid'):
    require(isinstance(loc.get(key),bool),f'{key} must be boolean')
for key in ('stage2_min_velocity_epochs','stage2_min_cog_epochs','stage2_min_velocity_qualified_ratio',
            'stage2_min_cog_qualified_ratio','stage2_max_sync_gap_p95_sec','stage2_max_wheel_gnss_residual_p95_mps'):
    require(key in loc,f'persistent Stage-2 evidence missing: {key}')

# 4) IMU calibration remains an autonomy interlock. Raw IMU orientation is fused only as
# relative yaw + gyro-Z; COG and the two tilt-compensated magnetometers are independent absolute yaw sources.
require(imu.get('require_fresh_gyro_for_imu_publish') is False,'IMU orientation must survive stale gyro when yaw is valid')
for key in ('stationary_calibration_valid','stationary_calibration_saved_at','stationary_calibration_sample_count',
            'stationary_calibration_duration_sec','stationary_calibration_gyro_z_std_rps','stationary_calibration_accel_norm_error_mps2'):
    require(key in imu,f'IMU evidence missing: {key}')
require(navcore.get('require_imu_calibration_for_autonomy') is True,'NavigationCore must require IMU calibration')
nav_cpp=(ROOT/'src/navigation_core.cpp').read_text()
require('require_imu_calibration_ && !imu_calibration_validated_' in nav_cpp,'NavigationCore IMU interlock missing')

# 5) Source-level deadlock break and GNSS vyaw derivation.
cpp=(ROOT/'src/localization_core.cpp').read_text()
for token in ('qualityFreshUnlocked','validVelocityCovarianceUnlocked','gnssMeasurementStampUsableUnlocked',
              'imu_yaw_unavailable','have_local_motion_at_gnss_ = localStateAtUnlocked',
              'No rejection: this is the expected state when ESC is disabled/offline',
              'course_enu - *last_gnss_velocity_course_enu_rad_',
              'gnss_yaw_rate_variance_','degraded_anchor_ok','DEGRADED_BOOTSTRAP'):
    require(token in cpp,f'LocalizationCore revised contract missing: {token}')
require('local_odom_sync_gap' not in cpp,'old local-odom hard rejection must not return')
raw_start=cpp.find('const bool raw_cog_candidate')
raw_end=cpp.find('const auto t = now();',raw_start)
require(raw_start>=0 and raw_end>raw_start,'raw COG qualification block missing')
raw_block=cpp[raw_start:raw_end]
require('gnss_velocity_qualified_' not in raw_block,'COG bootstrap must not depend on heading-projected velocity qualification')
require('wheel_slip_motion_detected_' not in raw_block,'COG bootstrap must not depend on heading-projected wheel-slip residual')

# 6) GUI has current localization controls and YAML autosave/live ESC sync.
gui=read_gui_source(ROOT)+(ROOT/'gui/agv_gui_specs.hpp').read_text()
for token in ('gnss_yaw_rate_min_speed_mps','anchor_init_requires_strict','Aktifkan serial ESC',
              'runtime.live_apply_yaml','bab4MenuTabs','navigation_menu.active_leaf'):
    require(token in gui,f'GUI revised feature missing: {token}')

for key in ('steering_calibration_valid','steering_circle_calibration_valid','drive_odometry_calibration_valid'):
    require(isinstance(vehicle.get(key),bool),f'vehicle {key} missing/not bool')

print('STAGE2 LOCALIZATION FOUNDATION SELF-CHECK: PASS')
print('GNSS=vx/COG | dual magnetic absolute yaw | IMU=relative yaw+gyro-Z | ESC vx-only auxiliary')
