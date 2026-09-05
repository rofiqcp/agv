#!/usr/bin/env python3
from gui_source_helper import read_gui_source
from pathlib import Path
import sys, yaml
root=Path(__file__).resolve().parents[1]
esc_cpp=root.parent/'esc/src/ackermann_controller_server.cpp'
esc_yaml=root.parent/'esc/config/ackermann.yaml'
veh_yaml=root/'config/vehicle.yaml'
text=esc_cpp.read_text(); gui=read_gui_source(root)
required=[
 'steering_physical_calibration_enabled',
 'steering_physical_left_limit_deg',
 'steering_physical_right_limit_deg',
 'steering_physical_operational_limit_deg',
 'operationalPhysicalLimitDeg()',
 'clampPhysicalSteeringDeg',
 'physical_deg / steering_physical_right_limit_deg_',
 'physical_deg / steering_physical_left_limit_deg_',
 'return std::min(steering_max_deg_, std::abs(steering_physical_right_limit_deg_))',
 'return -std::min(steering_max_deg_, std::abs(steering_physical_left_limit_deg_))',
]
for token in required:
    assert token in text, token
assert 'Target sudut roda fisik' in gui
assert 'Sudut fisik KIRI' in gui and 'Sudut fisik KANAN' in gui
assert 'RODA DALAM' in gui
assert 'serial_left_max_deg"]=80.0' not in gui, 'GUI must not overwrite protocol fallback as physical angle'
assert 'steering_physical_calibration_enabled"]=true' in gui
assert 'measured_left_steering_limit_rad' in gui and 'measured_right_steering_limit_rad' in gui
assert 'operational_steering_angle_rad' in gui
assert 'minimum_turning_radius_m' in gui
with open(esc_yaml) as f: e=yaml.safe_load(f)['esc_ackermann']['ros__parameters']
assert e['steering_physical_calibration_enabled'] is False
assert e['steering_physical_left_limit_deg'] < 0 < e['steering_physical_right_limit_deg']
assert 0 < e['steering_physical_operational_limit_deg'] <= min(abs(e['steering_physical_left_limit_deg']), e['steering_physical_right_limit_deg'])
with open(veh_yaml) as f: v=yaml.safe_load(f)['vehicle']['ros__parameters']
for k in ['measured_left_steering_limit_rad','measured_right_steering_limit_rad','max_steering_angle_rad','operational_steering_angle_rad','steering_calibration_source']:
    assert k in v, k
print('PASS steering physical Part 1 contract')
