#!/usr/bin/env python3
"""Static regression contract for 2026-09-10 navigation deep-audit implementation."""
from pathlib import Path
import math
import yaml

ROOT=Path(__file__).resolve().parents[1]
WS=ROOT.parents[1]

def need(cond,msg):
    if not cond:
        raise SystemExit('FAIL: '+msg)

def text(rel): return (WS/rel).read_text(encoding='utf-8')
def params(rel,node):
    data=yaml.safe_load(text(rel)) or {}
    return data[node]['ros__parameters']

imu=text('src/navigation/src/imu_node.cpp')
imuh=text('src/navigation/include/imu/imu_node.hpp')
navlaunch=text('src/navigation/launch/autonomous.launch.py')
navcore=text('src/navigation/src/navigation_core.cpp')
loc=text('src/navigation/src/localization_core.cpp')
esc=text('src/esc/src/ackermann_controller_server.cpp')
stm=text('src/stmf4/src/stmf4_hmi_bridge.cpp')
f4=text('F4gateway/src/Neo3Sensors.cpp')
cmake=text('src/navigation/CMakeLists.txt')

for token in ('packetStampNow(', '/imu/timing_status', 'msg.header.stamp = measurement_stamp',
              'parsed_type == 0x52', 'component_sync_max_gap_sec_'):
    need(token in imu or token in imuh, 'IMU measurement-time contract missing '+token)

vehicle=params('src/navigation/config/vehicle.yaml','vehicle')
ack=params('src/esc/config/ackermann.yaml','esc_ackermann')
need(vehicle['drive_odometry_calibration_valid'] is False, 'uncertified drive calibration must remain fail-closed')
need(abs(float(vehicle['drive_odometry_calibration_scale'])-float(ack['drive_odometry_calibration_scale']))<1e-12,
     'ESC and vehicle drive scales split-brain')
need('_materialize_nav2_vehicle_ssot' in navlaunch and '_validate_vehicle_runtime_contract' in navlaunch,
     'vehicle SSOT launch materialization/validation missing')
need('vehicle_drive_odometry_scale' in navlaunch and 'vehicle_speed_max_mps' in navlaunch and
     'vehicle_wheelbase_m' in navlaunch and 'vehicle_track_width_m' in navlaunch,
     'ESC vehicle SSOT launch overrides missing')

nav2=yaml.safe_load(text('src/navigation/config/nav2_ackermann.yaml'))
follow=nav2['controller_server']['ros__parameters']['FollowPath']
smoother=nav2['velocity_smoother']['ros__parameters']
need(abs(float(smoother['max_velocity'][2])-float(vehicle['max_yaw_rate_rps']))<1e-9,
     'velocity smoother yaw cap differs from vehicle SSOT')
need(abs(float(nav2['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance'])-
         float(vehicle['standard_xy_goal_tolerance_m']))<1e-9,
     'baseline goal tolerance must be standard tier')
need('precision_controller_frequency_hz' in vehicle and float(vehicle['precision_costmap_resolution_m'])<=0.05,
     'precision profile parameters missing')
need('precision_mode' in navlaunch and 'precision_localization_monitor' in navlaunch,
     'precision tier launch/gate missing')
need('precision_mode_ && !precision_localization_ready_' in navcore,
     'precision motion gate is not fail-closed')

sensor=text('src/navigation/src/sensor_contract_monitor.cpp')
for topic in ('/imu/data','/gnss/fix_raw','/gnss/vel','/esc/odom'):
    need(topic in sensor, 'sensor publisher ownership topic missing '+topic)
need('get_publishers_info_by_topic' in sensor and 'infos.size() == 1U' in sensor,
     'exactly-one publisher assertion missing')
need('sensor_contract_monitor' in cmake and 'sensor_contract_monitor' in navlaunch,
     'sensor contract monitor not built/launched')

need('SENSOR_PROTOCOL_VERSION = 2U' in f4 and 'sensorCrc16Ccitt' in f4 and 'gnss_sequence_' in f4,
     'F411 GNSS v2 CRC/sequence contract missing')
need('neo3_require_protocol_crc' in stm and 'validateNeo3Protocol' in stm and 'neo3_sequence_gaps_' in stm,
     'host GNSS v2 CRC/sequence validation missing')
need('quality.data.assign(50, nan)' in stm and 'quality.data[20] = 0.0' in stm and
     'quality.data[21] = 0.0' in stm and 'quality.data[24] = 3.0' in stm and
     'quality.data[49] = velocity_valid ? 1.0 : 0.0' in stm,
     'F411 /gnss/quality canonical 0..44 semantic parity / append-only extension missing')
need('Canonical transport-invariant append-only layout' in text('src/navigation/src/gnss_node.cpp'),
     'direct USB GNSS quality canonical-layout declaration missing')

for token in ('wheel_gnss_nis_gate_', 'cog_nis_gate_', '/localization/innovation_status', '/localization/wheel_slip'):
    need(token in loc, 'innovation/NIS/slip contract missing '+token)
need('wheel_slip_covariance_multiplier_' in esc and 'v_var *= wheel_slip_covariance_multiplier_' in esc,
     'wheel slip covariance inflation missing')

ekf=yaml.safe_load(text('src/navigation/config/ekf.yaml'))
need(float(ekf['ekf_filter_node_odom']['ros__parameters']['frequency'])>=30.0,
     'local EKF experiment rate must be >=30 Hz')
need(float(ekf['ekf_filter_node_map']['ros__parameters']['frequency'])>=10.0,
     'global EKF rate unexpectedly reduced')

absolute=params('src/navigation/config/absolute_localization.yaml','precision_localization_monitor')
need(float(absolute['max_position_std_m'])<=0.05 and float(absolute['max_yaw_std_rad'])<=math.radians(2.01),
     'precision absolute-pose quality gate too loose')
need('precision_localization_monitor' in cmake, 'precision monitor not compiled')

for tool in ('navigation_calibration.py','navigation_metrics.py','planar_eskf_ab.py',
             'navigation_fault_suite.py','certification_manifest.py','rosbag_regression.py'):
    p=WS/'src/navigation/tools'/tool
    need(p.is_file() and p.stat().st_size>100, 'missing validation tool '+tool)
    need(tool in cmake, 'validation tool not installed '+tool)

need('camera_metric_calibration_validated_ &&' in text('src/perception/src/astra_yolop_cpu_pt_node.cpp'),
     'CPU camera metric authority fail-closed missing')
need('camera_metric_calibration_validated_ &&' in text('src/perception/src/astra_yolop_gpu_node.cpp'),
     'GPU camera metric authority fail-closed missing')

print('PASS navigation_deep_audit_impl_self_check')
print('P0 measurement/SSOT/ownership/GNSS protocol: PASS')
print('P2 NIS/slip/30Hz local estimator tooling: PASS')
print('P3/P4 precision tier fail-closed + Nav2 consistency: PASS')
print('P1/P5 evidence tooling + production perception authority: PASS')
