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

# Current deployment uses three distinct USB-UART families. Stable by-id
# identity is primary; physical by-path remains only a deterministic fallback.
expected_ids = {
    'ESC': (esc['serial_auto_id_contains'], 'Prolific_Technology_Inc._USB-Serial_Controller'),
    'GNSS': (gnss['auto_port_id_contains'], '1a86_USB_Serial'),
    'IMU': (imu['auto_port_id_contains'], 'Silicon_Labs_CP2102'),
}
for name, (actual, wanted) in expected_ids.items():
    if wanted not in str(actual): fail(f'{name} serial id selector {actual!r} does not contain {wanted!r}')
if len({str(x[0]) for x in expected_ids.values()}) != 3:
    fail('serial by-id selectors are not unique')
expected_paths = {
    'ESC': str(esc['serial_auto_path_contains']),
    'GNSS': str(gnss['auto_port_path_contains']),
    'IMU': str(imu['auto_port_path_contains']),
}
if any(not value.strip() for value in expected_paths.values()):
    fail('serial physical-path fallback is empty')
if len(set(expected_paths.values())) != 3:
    fail('serial physical-path fallbacks are not unique')

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
    (gnss_cpp, '"auto_port_id_contains", "1a86_USB_Serial"', 'GNSS'),
    (imu_cpp, '"auto_port_id_contains", "Silicon_Labs_CP2102"', 'IMU'),
    (esc_cpp, '"serial_auto_id_contains", "Prolific_Technology_Inc._USB-Serial_Controller"', 'ESC'),
]:
    if token not in text: fail(f'{name} source fallback does not match YAML')

if gnss_cpp.find('globPattern("/dev/serial/by-id/*")') > gnss_cpp.find('globPattern("/dev/serial/by-path/*")'):
    fail('GNSS must prefer by-id before by-path fallback')
if imu_cpp.find('globPattern("/dev/serial/by-id/*")') > imu_cpp.find('globPattern("/dev/serial/by-path/*")'):
    fail('IMU must prefer by-id before by-path fallback')
if esc_cpp.find('directory_iterator("/dev/serial/by-id"') > esc_cpp.find('directory_iterator("/dev/serial/by-path"'):
    fail('ESC must prefer by-id before by-path fallback')

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
print(f'serial paths: ESC={esc["serial_auto_path_contains"]} | GNSS={gnss["auto_port_path_contains"]} | IMU={imu["auto_port_path_contains"]}')
print(f'uncalibrated geometry: Rmin={rmin:.6f} m | vehicle yaw cap={vehicle["max_yaw_rate_rps"]:.6f} rad/s | autonomous yaw cap={nav_yaw_cap:.6f} rad/s')
