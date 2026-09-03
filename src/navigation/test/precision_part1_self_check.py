#!/usr/bin/env python3
from gui_source_helper import read_gui_source
from pathlib import Path
import math
import sys
try:
    import yaml
except Exception as exc:
    print(f'FAIL: python3-yaml required: {exc}', file=sys.stderr); sys.exit(2)

root = Path(__file__).resolve().parents[1]
ws = root.parent

def load(path):
    return yaml.safe_load(Path(path).read_text())

def near(a,b,eps=1e-9):
    return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=eps)

def fail(msg):
    print('FAIL:', msg); sys.exit(1)

v = load(root/'config/vehicle.yaml')['vehicle']['ros__parameters']
esc = load(ws/'esc/config/ackermann.yaml')['esc_ackermann']['ros__parameters']
nc = load(root/'config/navigation_core.yaml')['navigation_core']['ros__parameters']
nav2 = load(root/'config/nav2_ackermann.yaml')
per = load(ws/'perception/config/astra_yolop_gpu.yaml')['perception']['ros__parameters']
traj = load(root/'config/trajectory_safety.yaml')['trajectory_safety_supervisor']['ros__parameters']
imu = load(root/'config/imu.yaml')['data_imu_node']['ros__parameters']

# Exact physical authority.
for name, actual, expected in [
    ('ESC wheelbase', esc['wheelbase_m'], v['wheelbase_m']),
    ('ESC track', esc['track_width_m'], v['track_width_m']),
    ('NavigationCore wheelbase', nc['wheelbase_m'], v['wheelbase_m']),
    ('NavigationCore track', nc['track_width_m'], v['track_width_m']),
    ('NavigationCore operational steering', nc['max_steering_angle_rad'], v['operational_steering_angle_rad']),
    ('Perception wheelbase', per['wheelbase_m'], v['wheelbase_m']),
    ('Perception physical width', per['lane_vehicle_width_m'], v['total_width_m']),
    ('Smac min turn', nav2['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius'], v['minimum_turning_radius_m']),
    ('MPPI min turn', nav2['controller_server']['ros__parameters']['FollowPath']['AckermannConstraints']['min_turning_r'], v['minimum_turning_radius_m']),
]:
    if not near(actual, expected, 1e-7): fail(f'{name}: {actual} != {expected}')
if nav2['local_costmap']['local_costmap']['ros__parameters']['footprint'] != v['footprint']:
    fail('local costmap footprint != vehicle authority')
if nav2['global_costmap']['global_costmap']['ros__parameters']['footprint'] != v['footprint']:
    fail('global costmap footprint != vehicle authority')

# Downstream operating limits must not exceed the certified ceiling.
ctrl = nav2['controller_server']['ros__parameters']['FollowPath']
sm = nav2['velocity_smoother']['ros__parameters']
checks = [
    ('NavigationCore forward', nc['max_forward_speed_mps'], v['max_forward_speed_mps']),
    ('NavigationCore reverse', nc['max_reverse_speed_mps'], v['max_reverse_speed_mps']),
    ('NavigationCore yaw', nc['max_yaw_rate_rps'], v['max_yaw_rate_rps']),
    ('MPPI forward', ctrl['vx_max'], v['max_forward_speed_mps']),
    ('MPPI yaw', ctrl['wz_max'], v['max_yaw_rate_rps']),
    ('MPPI accel', ctrl['ax_max'], v['max_accel_mps2']),
    ('Smoother forward', sm['max_velocity'][0], v['max_forward_speed_mps']),
    ('Smoother yaw', abs(sm['max_velocity'][2]), v['max_yaw_rate_rps']),
    ('Smoother accel', sm['max_accel'][0], v['max_accel_mps2']),
    ('Smoother yaw accel', sm['max_accel'][2], v['max_yaw_accel_rps2']),
    ('Trajectory yaw', traj['maximum_yaw_rate_rps'], v['max_yaw_rate_rps']),
]
for name, actual, ceiling in checks:
    if float(actual) > float(ceiling) + 1e-9: fail(f'{name}: {actual} exceeds {ceiling}')
if float(ctrl['ax_min']) < float(v['max_decel_mps2']) - 1e-9:
    fail('MPPI decel exceeds certified magnitude')
if float(sm['max_decel'][0]) < float(v['max_decel_mps2']) - 1e-9:
    fail('Smoother decel exceeds certified magnitude')
if float(sm['max_decel'][2]) < -float(v['max_yaw_accel_rps2']) - 1e-9:
    fail('Smoother yaw decel exceeds certified magnitude')

# IMU absolute-yaw contract: a fresh orientation may publish even if gyro is stale.
# GNSS now owns vyaw, so suppressing valid orientation on stale gyro would deadlock EKF startup.
if imu.get('require_fresh_gyro_for_imu_publish', True): fail('fresh gyro gate must be false when IMU owns yaw only')
if not 0.05 <= float(imu.get('gyro_packet_timeout_sec', 0)) <= 2.0: fail('gyro timeout invalid')
imu_cpp=(root/'src/imu_node.cpp').read_text()
imu_hpp=(root/'include/imu/imu_node.hpp').read_text()
for token in ['last_gyro_packet_time_', 'gyro_packet_timeout_sec_', 'require_fresh_gyro_for_imu_publish_', 'if (publishImu())', 'EKF yaw now comes from the absolute IMU orientation quaternion']:
    if token not in imu_cpp + imu_hpp: fail(f'IMU freshness source missing {token}')

# No dead GUI goal checker path remains.
gui=read_gui_source(root) + (root/'gui/agv_gui_specs.hpp').read_text()
if 'general_goal_checker' in gui: fail('dead general_goal_checker GUI key remains')
for good in ['controller_server.ros__parameters.goal_checker.xy_goal_tolerance',
             'controller_server.ros__parameters.goal_checker.yaw_goal_tolerance']:
    if good not in gui: fail(f'missing GUI goal key {good}')

# Collision Monitor is preview-only and opt-in in Part 1.
for launch_name in ['autonomous.launch.py','gui.launch.py']:
    text=(root/'launch'/launch_name).read_text()
    if 'enable_collision_monitor' not in text: fail(f'{launch_name}: collision arg missing')
    if "enable_collision_monitor', default_value='true'" in text or 'enable_collision_monitor", default_value="true"' in text:
        fail(f'{launch_name}: collision preview must not default ON')
if nc.get('collision_monitor_enabled', True): fail('NavigationCore collision ownership must remain false in Part1')
coll=load(root/'config/collision_monitor.yaml')['collision_monitor']['ros__parameters']
if coll['cmd_vel_out_topic'] == '/cmd_vel': fail('Part1 collision preview must not own /cmd_vel')

print('PASS precision_part1_self_check')
