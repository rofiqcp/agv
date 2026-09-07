#!/usr/bin/env python3
from gui_source_helper import read_gui_source
from pathlib import Path
import math
import re
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parent

def load(path):
    return yaml.safe_load(Path(path).read_text())

def fail(msg):
    print('FAIL:', msg)
    sys.exit(1)

def near(a, b, eps=1e-7):
    return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=eps)

vehicle = load(ROOT/'config/vehicle.yaml')['vehicle']['ros__parameters']
nav2 = load(ROOT/'config/nav2_ackermann.yaml')
nc = load(ROOT/'config/navigation_core.yaml')['navigation_core']['ros__parameters']
traj = load(ROOT/'config/trajectory_safety.yaml')['trajectory_safety_supervisor']['ros__parameters']
esc = load(WS/'esc/config/ackermann.yaml')['esc_ackermann']['ros__parameters']
gnss = load(ROOT/'config/gnss.yaml')['data_cuav_node']['ros__parameters']
imu = load(ROOT/'config/imu.yaml')['data_imu_node']['ros__parameters']
hmi = load(WS/'stmf4/config/hmi.yaml')['stmf4_hmi_bridge']['ros__parameters']
launch = (ROOT/'launch/autonomous.launch.py').read_text()

# Production transport is one identity-checked F411 CDC gateway for NEO-3/IST8310
# and F103 VESC, plus one direct CP2102 IMU. Legacy CH340/PL2303 routes are
# recovery-only and must never use physical by-path fallback because USB topology
# can be re-used by the F411 after reflashing/replugging.
if str(hmi.get('serial_device','')).lower() != 'auto': fail('F411 gateway must default to auto identity discovery')
if "DeclareLaunchArgument('gnss_source', default_value='stm32'" not in launch: fail('GNSS production source must be F411/stm32')
if "DeclareLaunchArgument('esc_transport_mode', default_value='stm32'" not in launch: fail('ESC production transport must be F411/stm32')
expected_ids = {
    'ESC_RECOVERY': (esc['serial_auto_id_contains'], 'Prolific_Technology_Inc._USB-Serial_Controller'),
    'GNSS_RECOVERY': (gnss['auto_port_id_contains'], '1a86_USB_Serial'),
    'IMU': (imu['auto_port_id_contains'], 'Silicon_Labs_CP2102'),
}
for name, (actual, wanted) in expected_ids.items():
    if wanted not in str(actual): fail(f'{name} serial id selector {actual!r} does not contain {wanted!r}')
expected_paths = {
    'ESC_RECOVERY': str(esc['serial_auto_path_contains']),
    'GNSS_RECOVERY': str(gnss['auto_port_path_contains']),
    'IMU': str(imu['auto_port_path_contains']),
}
if any(value.strip() for value in expected_paths.values()):
    fail(f'physical by-path fallback must be disabled: {expected_paths}')

all_text = '\n'.join(
    p.read_text(errors='ignore') for p in WS.rglob('*')
    if p.is_file()
    and 'docs' not in p.parts
    and p.suffix.lower() in {'.cpp','.hpp','.h','.yaml','.yml','.py','.xml','.txt'}
)
obsolete_selector = 'FTDI_' + 'FT232R_USB_UART_A5069RR4'
if obsolete_selector in all_text:
    fail('obsolete FTDI selector remains in project')

# Source fallbacks must agree with YAML so a missing params file cannot swap sensors.
gnss_cpp = (ROOT/'src/gnss_node.cpp').read_text()
imu_cpp = (ROOT/'src/imu_node.cpp').read_text()
esc_cpp = (WS/'esc/src/ackermann_controller_server.cpp').read_text()
for text, token, name in [
    (gnss_cpp, '"auto_port_id_contains", "1a86_USB_Serial"', 'GNSS recovery'),
    (imu_cpp, '"auto_port_id_contains", "Silicon_Labs_CP2102"', 'IMU'),
    (esc_cpp, '"serial_auto_id_contains", "Prolific_Technology_Inc._USB-Serial_Controller"', 'ESC recovery'),
    (gnss_cpp, '"auto_port_path_contains", ""', 'GNSS no by-path'),
    (imu_cpp, '"auto_port_path_contains", ""', 'IMU no by-path'),
    (esc_cpp, '"serial_auto_path_contains", ""', 'ESC no by-path'),
]:
    if token not in text: fail(f'{name} source fallback does not match production contract')

# Geometry must be self-consistent even before physical commissioning.
L = float(vehicle['effective_wheelbase_m'])
track = float(vehicle['track_width_m'])
op = float(vehicle['operational_steering_angle_rad'])
if not (L > 0 and track > 0 and 0 < op < math.pi/2): fail('vehicle geometry invalid')
expected_r = (0.5 * track + L / math.tan(op)) * 1.10
if not vehicle['steering_circle_calibration_valid']:
    if float(vehicle['minimum_turning_radius_m']) + 1e-6 < expected_r:
        fail(f'uncalibrated minimum turning radius {vehicle["minimum_turning_radius_m"]} is smaller than safe theoretical {expected_r}')
    if 'uncalibrated' not in str(vehicle['steering_calibration_source']):
        fail('uncalibrated steering must be explicitly marked in steering_calibration_source')

rmin = float(vehicle['minimum_turning_radius_m'])
vehicle_yaw_cap = float(vehicle['max_forward_speed_mps']) / rmin
if float(vehicle['max_yaw_rate_rps']) > vehicle_yaw_cap + 1e-7:
    fail('vehicle max_yaw_rate_rps violates v/R minimum-turning-radius ceiling')

planner_r = nav2['planner_server']['ros__parameters']['GridBased']['minimum_turning_radius']
ctrl = nav2['controller_server']['ros__parameters']['FollowPath']
sm = nav2['velocity_smoother']['ros__parameters']
for label, val in [('Smac', planner_r), ('MPPI', ctrl['AckermannConstraints']['min_turning_r'])]:
    if not near(val, rmin): fail(f'{label} minimum turning radius != vehicle authority')
if not near(nc['max_steering_angle_rad'], vehicle['operational_steering_angle_rad']):
    fail('NavigationCore steering cap must use operational, not mechanical, angle')

nav_yaw_cap = float(ctrl['vx_max']) / rmin
for label, val in [
    ('MPPI', ctrl['wz_max']),
    ('velocity_smoother max', sm['max_velocity'][2]),
    ('velocity_smoother min', abs(sm['min_velocity'][2])),
    ('trajectory safety', traj['maximum_yaw_rate_rps']),
]:
    if float(val) > nav_yaw_cap + 1e-7:
        fail(f'{label} yaw limit {val} exceeds autonomous v/R cap {nav_yaw_cap}')

freq = float(nav2['controller_server']['ros__parameters']['controller_frequency'])
dt = float(ctrl['model_dt'])
if not near(dt, 1.0/freq, 1e-9):
    fail(f'MPPI model_dt={dt} must equal controller period={1.0/freq} for this Humble configuration')

# Recalibration must invalidate stale circle certification and update all relevant command gates.
gui = read_gui_source(ROOT)
for token in [
    'invalidateCircleCalibration()',
    'propagateKinematicAuthority(',
    'velocity_smoother.ros__parameters.max_velocity',
    'trajectory_safety_supervisor.ros__parameters.maximum_yaw_rate_rps',
    'theoretical_part1_recalibration_required',
    'theoretical_part2_recalibration_required',
]:
    if token not in gui: fail(f'GUI steering authority propagation missing {token}')

print('PASS project_consistency_self_check')
print('serial route: NEO-3/IST8310 + VESC via identity-checked F411 CDC | IMU via CP2102 | legacy physical by-path DISABLED')
print(f'uncalibrated geometry: Rmin={rmin:.6f} m | vehicle yaw cap={vehicle["max_yaw_rate_rps"]:.6f} rad/s | autonomous yaw cap={nav_yaw_cap:.6f} rad/s')
