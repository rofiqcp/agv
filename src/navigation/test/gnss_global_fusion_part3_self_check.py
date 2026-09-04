#!/usr/bin/env python3
from gui_source_helper import read_gui_source
"""Static contract for GNSS vx/vyaw + IMU yaw fusion and map TF ownership."""
from pathlib import Path
import sys, yaml
ROOT=Path(__file__).resolve().parents[1]
def fail(m): print('FAIL:',m,file=sys.stderr); raise SystemExit(1)
def load(p): return yaml.safe_load(p.read_text()) or {}
def enabled(cfg): return [i for i,v in enumerate(cfg or []) if v]
loc=load(ROOT/'config/localization_cpp.yaml')['localization_core']['ros__parameters']
stage3=load(ROOT/'config/stage3_navigation.yaml')['stage3_navigation']['ros__parameters']
ekf=load(ROOT/'config/ekf.yaml')
g=ekf['ekf_filter_node_map']['ros__parameters']; l=ekf['ekf_filter_node_odom']['ros__parameters']
cpp=(ROOT/'src/localization_core.cpp').read_text(encoding='utf-8')
gui=read_gui_source(ROOT)+(ROOT/'gui/agv_gui_specs.hpp').read_text(encoding='utf-8')
bag=(ROOT/'tools/rosbag_regression.py').read_text(encoding='utf-8')

if loc.get('gnss_velocity_fusion_topic')!='/gnss/base_velocity_fusion': fail('velocity fusion topic mismatch')
if not loc.get('enable_global_gnss_velocity_fusion',False): fail('GNSS vx/vyaw fusion must be enabled')
if loc.get('require_gnss_velocity_certification_for_fusion',True): fail('startup GNSS velocity cannot depend on ESC-based certification')
if loc.get('enable_global_gnss_cog_fusion') is not True: fail('GNSS COG absolute yaw fusion must be enabled for production heading')
if loc.get('enable_gnss_course_yaw_correction',True): fail('legacy GNSS direct yaw correction must remain disabled')
cog_min=float(loc.get('cog_min_forward_speed_mps',0.0))
commissioning_cap=float(stage3.get('commissioning_speed_cap_mps',0.0))
validation_min=float(loc.get('gnss_velocity_min_validation_speed_mps',0.0))
if cog_min < validation_min: fail('COG gate must not be below GNSS velocity validation speed')
if cog_min > commissioning_cap: fail(f'COG gate {cog_min} m/s is unreachable at commissioning cap {commissioning_cap} m/s')
if float(loc.get('global_ekf_yaw_max_innovation_rad',0.0)) < 1.57079632679:
    fail('global COG yaw correction must recover >=90deg startup heading error')
if float(loc.get('cog_max_sacc_mps',999.0)) > 0.3: fail('COG speed-accuracy gate must stay conservative (<=0.3 m/s)')
for key in ('gnss_yaw_rate_min_speed_mps','gnss_yaw_rate_max_abs_rps','gnss_yaw_rate_filter_alpha',
            'gnss_yaw_rate_min_variance','gnss_yaw_rate_max_variance'):
    if key not in loc: fail(f'missing GNSS vyaw parameter {key}')

if l.get('publish_tf') is not True: fail('local EKF must publish odom->base TF')
if g.get('publish_tf') is not False: fail('global EKF must not compete for map->odom TF')
# Local EKF: short-term odom continuity — wheel/GNSS vx + IMU gyro-Z only.
# Magnetic absolute yaw is a startup seed in LocalizationCore, not a continuous EKF input.
if l.get('twist0')!='/gnss/base_velocity_fusion' or enabled(l.get('twist0_config')) != [6]:
    fail('local EKF must fuse independent GNSS vx')
if l.get('imu0')!='/imu/data' or enabled(l.get('imu0_config')) != [11]:
    fail('local EKF must fuse IMU gyro-Z only; magnetic absolute yaw must not be fused continuously')
if l.get('odom0')!='/esc/odom' or enabled(l.get('odom0_config')) != [6]:
    fail('local EKF wheel odometry must remain auxiliary vx-only to avoid steering-slip yaw authority')
# Global EKF (COG active): GNSS x/y + GNSS vx, COG absolute yaw, IMU vyaw continuity.
if g.get('odom0')!='/odometry/gnss_map' or enabled(g.get('odom0_config')) != [0,1]:
    fail('global EKF must fuse GNSS absolute x/y')
if g.get('twist0')!='/gnss/base_velocity_fusion' or enabled(g.get('twist0_config')) != [6]:
    fail('global EKF must fuse GNSS vx only (yaw-rate authority moved to IMU gyro with COG active)')
if g.get('pose0')!='/gnss/cog_heading_fusion' or enabled(g.get('pose0_config')) != [5]:
    fail('global EKF must fuse GNSS COG absolute yaw via pose0 (yaw only)')
if g.get('imu0')!='/imu/data' or enabled(g.get('imu0_config')) != [11]:
    fail('global EKF must fuse IMU vyaw-only (gyro continuity) when COG is active')
if any(isinstance(v,str) and v=='/gnss/cog_heading_fusion' for v in g.values()):
    pass  # expected: pose0 is the COG topic

raw_start=cpp.find('const bool raw_cog_candidate')
raw_end=cpp.find('const auto t = now();', raw_start)
if raw_start < 0 or raw_end < 0: fail('raw COG qualification block missing')
raw_block=cpp[raw_start:raw_end]
if 'gnss_velocity_qualified_' in raw_block:
    fail('COG heading bootstrap must not depend on base-frame velocity qualification')
if 'wheel_slip_motion_detected_' in raw_block:
    fail('COG heading bootstrap must not depend on heading-projected wheel-slip residual')
for token in [
    'publishGatedFusionMeasurementsUnlocked','gnss_velocity_qualified_',
    'gnss_yaw_rate_rps_','gnss_yaw_rate_variance_','gnss_yaw_rate_valid_',
    'course_enu - *last_gnss_velocity_course_enu_rad_',
    'imu_yaw_unavailable','have_local_motion_at_gnss_ = localStateAtUnlocked',
    'No rejection: this is the expected state when ESC is disabled/offline',
    'gnss_velocity_fusion_pub_->publish','"/gnss/fusion_status"',
]:
    if token not in cpp: fail(f'C++ fusion contract missing: {token}')
for topic in ['/gnss/base_velocity_fusion','/gnss/fusion_status']:
    if topic not in bag: fail(f'rosbag topic missing: {topic}')
for token in ['gnss_yaw_rate_min_speed_mps','enable_global_gnss_velocity_fusion','enable_global_gnss_cog_fusion']:
    if token not in gui: fail(f'GUI fusion setting missing: {token}')
print('PASS GNSS COG + gyro heading fusion contract')
