#!/usr/bin/env python3
from pathlib import Path
import math,yaml
ROOT=Path(__file__).resolve().parents[1]; SRC=ROOT.parent

def req(ok,msg):
    if not ok: raise AssertionError(msg)
def ena(v): return {i for i,x in enumerate(v or []) if x}
def load(name,node): return yaml.safe_load((ROOT/'config'/name).read_text())[node]['ros__parameters']

ekf=yaml.safe_load((ROOT/'config/ekf.yaml').read_text()); lo=ekf['ekf_filter_node_odom']['ros__parameters']; gl=ekf['ekf_filter_node_map']['ros__parameters']
nav=yaml.safe_load((ROOT/'config/nav2_ackermann.yaml').read_text()); nc=load('navigation_core.yaml','navigation_core'); loc=load('localization_cpp.yaml','localization_core'); st3=load('stage3_navigation.yaml','stage3_navigation')
navcpp=(ROOT/'src/navigation_core.cpp').read_text(); loccpp=(ROOT/'src/localization_core.cpp').read_text(); router=(ROOT/'src/cmd_vel_router.cpp').read_text(); esc=(SRC/'esc/src/ackermann_controller_server.cpp').read_text(); certify=(ROOT/'tools/stage3_certify.py').read_text(); mag=(ROOT/'src/mag_heading_fusion_node.cpp').read_text()
# EKF ownership / source matrix.
req(lo['world_frame']=='odom' and lo['base_link_frame']=='base_footprint' and lo.get('publish_tf') is True,'local EKF must own odom->base_footprint')
req(gl['world_frame']=='map' and gl['base_link_frame']=='base_footprint' and gl.get('publish_tf') is False,'global EKF must not own map->odom TF')
req(lo['odom0']=='/esc/odom' and ena(lo['odom0_config'])=={6},'local ESC input must be vx-only')
req(lo['twist0']=='/gnss/base_velocity_fusion' and ena(lo['twist0_config'])=={6},'local GNSS input must be qualified vx-only')
req(lo['imu0']=='/imu/data' and ena(lo['imu0_config'])=={11},'local IMU must be gyro-Z only')
req(gl['odom0']=='/odometry/gnss_map' and ena(gl['odom0_config'])=={0,1},'global GNSS position must be map x/y only')
req(gl['pose0']=='/gnss/cog_heading_fusion' and ena(gl['pose0_config'])=={5},'global pose0 must be qualified COG yaw')
req(gl['pose1']=='/heading/validated_fusion' and ena(gl['pose1_config'])=={5},'global pose1 must be validated heading only')
req(gl['imu0']=='/imu/data' and ena(gl['imu0_config'])=={11},'global IMU must be gyro-Z only')
req('pose2' not in gl and '/imu/mag_heading_fusion' not in str(gl) and '/neo3/mag_heading_fusion' not in str(gl),'raw magnetic headings must never enter EKF directly')
# TF owner remains exactly split.
for token in ('tf_broadcaster_','tf.header.frame_id = map_frame_','tf.child_frame_id = odom_frame_','tf_broadcaster_->sendTransform(tf)'):
    req(token in loccpp,f'LocalizationCore map->odom contract missing: {token}')
# Nav2 timing: EKF must update faster than controller and MPPI dt must match controller period.
cf=float(nav['controller_server']['ros__parameters']['controller_frequency']); mppi=nav['controller_server']['ros__parameters']['FollowPath']; dt=float(mppi['model_dt'])
req(float(lo['frequency'])>=cf and float(gl['frequency'])>=cf,'EKF rate must not be slower than MPPI controller')
req(abs(dt-1.0/cf)<1e-9,'MPPI model_dt must equal controller period')
req(float(nav['velocity_smoother']['ros__parameters']['smoothing_frequency'])>=cf,'velocity smoother must not be slower than controller')
# Ackermann geometry consistency between planner/controller.
planner=nav['planner_server']['ros__parameters']['GridBased']; r1=float(planner['minimum_turning_radius']); r2=float(mppi['AckermannConstraints']['min_turning_r'])
req(abs(r1-r2)<1e-6 and r1>0,'planner and MPPI minimum turning radius must match')
# Localization integrity stays strict and certified.
req(loc.get('require_gnss_velocity_certification_for_fusion') is True and loc.get('require_gnss_cog_certification_for_fusion') is True,'GNSS vx/COG fusion must remain certification-gated')
req(int(loc['strict_min_satellites'])>=8 and float(loc['strict_max_dop'])<=2.0 and float(loc['strict_max_hacc_m'])<=2.5,'strict GNSS acquire thresholds were weakened')
req(loc.get('use_imu_initial_heading') is False,'raw Yahboom ANGLE must never seed map yaw')
# Motion gate requires calibration + Stage3 and has defense-in-depth production checks.
for key in ('require_steering_calibration_for_autonomy','require_steering_circle_calibration_for_autonomy','require_drive_odometry_calibration_for_autonomy','require_imu_calibration_for_autonomy','require_stage3_production_certification','require_collision_monitor_for_production','require_perception_for_production'):
    req(nc.get(key) is True,f'NavigationCore safety gate disabled: {key}')
for token in ('require_stage3_production_certification_ && !stage3_production_certified_','require_collision_monitor_for_production_','production_perception_age'):
    req(token in navcpp,f'production runtime defense missing: {token}')
# Two independent downstream gate consumers must remain.
req('/system/autonomy_motion_allowed' in router and 'autonomy_gate_seen_ && autonomy_gate_' in router,'cmd_vel_router autonomy gate missing')
req('/system/autonomy_motion_allowed' in esc and 'require_autonomy_gate' in esc and 'autonomy_gate_seen_ && autonomy_gate_' in esc,'ESC autonomy gate missing')
# Stage3 normal certification path must require real production safety evidence.
for token in ("collision_monitor_enabled","trajectory_safety_calibration_valid","collision_monitor_calibration_valid","fault_injection_calibration_valid","camera_metric_calibration_validated"):
    req(token in certify,f'Stage3 finalize prerequisite missing: {token}')
if st3.get('production_autonomy_certified'):
    req(nc.get('collision_monitor_enabled') is True,'certified production cannot run with collision monitor disabled')
    req(st3.get('trajectory_safety_calibration_valid') and st3.get('collision_monitor_calibration_valid'),'certified production requires trajectory/collision evidence')
# Yahboom calibrated MAG remains diagnostic only.
req('/imu/mag_raw_lsb' in mag and '/imu/mag_heading_fusion' in mag,'Yahboom calibrated diagnostic pipeline missing')
print('PASS ekf_nav2_gate_chain_self_check')
print(f'EKF local/global={lo["frequency"]}/{gl["frequency"]} Hz | MPPI={cf} Hz dt={dt:.3f}s | TF ownership and dual actuator gates PASS')
