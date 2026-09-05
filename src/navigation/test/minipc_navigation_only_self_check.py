#!/usr/bin/env python3
"""Static contract for camera-only OFF mode and ESC-optional localization."""
from pathlib import Path
import sys, yaml
ROOT=Path(__file__).resolve().parents[1]; WS=ROOT.parent

def fail(m): print('FAIL:',m); raise SystemExit(1)
def params(path,node): return (yaml.safe_load(path.read_text()) or {})[node]['ros__parameters']
def enabled(cfg): return {i for i,v in enumerate(cfg or []) if v}

autonomous=(ROOT/'launch/autonomous.launch.py').read_text(); gui=(ROOT/'launch/gui.launch.py').read_text()
if (ROOT/'launch/minipc.launch.py').exists(): fail('minipc.launch.py must not exist')
for label,source in (('autonomous',autonomous),('gui',gui)):
    for token in ('perception_mode','engine_path','pt_model_path'):
        if token not in source: fail(f'{label} missing perception option {token}')
if 'camera_only_node' not in autonomous: fail('OFF mode camera-only backend missing')
if 'importlib.util.find_spec' in autonomous: fail('autonomous CPU launch still performs legacy Python dependency probe')
for token in ('condition=IfCondition(perception_cpu_enabled)','condition=IfCondition(perception_gpu_enabled)',
              "executable='perception_cpu_node'","executable='perception_node'",
              "executable='camera_only_node'"):
    if token not in autonomous: fail(f'selectable perception contract missing: {token}')

if "DeclareLaunchArgument('esc_serial_enabled', default_value='true')" not in autonomous:
    fail('autonomous esc_serial_enabled arg missing')
if 'serial_enabled' not in (WS/'esc/launch/esc.launch.py').read_text(): fail('ESC launch does not forward serial_enabled')

ekf=yaml.safe_load((ROOT/'config/ekf.yaml').read_text()); local=ekf['ekf_filter_node_odom']['ros__parameters']; global_=ekf['ekf_filter_node_map']['ros__parameters']
if local.get('odom0')!='/esc/odom' or enabled(local.get('odom0_config'))!={6}: fail('local EKF ESC odom must remain vx-only')
if local.get('twist0')!='/gnss/base_velocity_fusion' or enabled(local.get('twist0_config'))!={6}: fail('local EKF must use independent GNSS vx')
if local.get('imu0')!='/imu/data' or enabled(local.get('imu0_config'))!={5,11} or local.get('imu0_relative') is not True: fail('local EKF must use relative IMU yaw+gyro-Z')
if global_.get('odom0')!='/odometry/gnss_map' or enabled(global_.get('odom0_config'))!={0,1}: fail('global GNSS x/y missing')
if global_.get('twist0')!='/gnss/base_velocity_fusion' or enabled(global_.get('twist0_config'))!={6}: fail('global GNSS vx missing')
for key,topic in (('pose0','/gnss/cog_heading_fusion'),('pose1','/neo3/mag_heading_fusion'),('pose2','/imu/mag_heading_fusion')):
    if global_.get(key)!=topic or enabled(global_.get(key+'_config'))!={5}: fail(f'global absolute heading source missing: {key}')
if global_.get('imu0')!='/imu/data' or enabled(global_.get('imu0_config'))!={5,11} or global_.get('imu0_relative') is not True: fail('global relative IMU yaw+gyro-Z missing')
print('PASS autonomous/gui camera-only OFF + ESC-optional localization contract')
