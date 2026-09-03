#!/usr/bin/env python3
from gui_source_helper import read_gui_source
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[1]
esc_cpp=(root.parent/'esc/src/ackermann_controller_server.cpp').read_text()
gui=read_gui_source(root)
with open(root.parent/'esc/config/ackermann.yaml') as f:
    e=yaml.safe_load(f)['esc_ackermann']['ros__parameters']
with open(root/'config/vehicle.yaml') as f:
    v=yaml.safe_load(f)['vehicle']['ros__parameters']
for k in [
 'steering_physical_lut_enabled','steering_lut_physical_deg',
 'steering_lut_command_increasing_deg','steering_lut_command_decreasing_deg',
 'steering_lut_feedback_increasing_deg','steering_lut_feedback_decreasing_deg',
 'steering_lut_direction_deadband_deg','steering_lut_calibration_saved_at']:
    assert k in e, k
assert e['steering_physical_lut_enabled'] is False
n=len(e['steering_lut_physical_deg'])
assert n>=5
for k in ['steering_lut_command_increasing_deg','steering_lut_command_decreasing_deg','steering_lut_feedback_increasing_deg','steering_lut_feedback_decreasing_deg']:
    assert len(e[k])==n
for token in [
 'validateSteeringLut','interpolateMonotonic','updateSteeringLutDirection',
 'steering_lut_motion_direction_','steering_lut_feedback_increasing_deg_',
 'steering_lut_feedback_decreasing_deg_','steering_physical_lut_enabled_ && steering_physical_lut_valid_']:
    assert token in esc_cpp, token
for token in [
 'Multi-Point LUT + Hysteresis / Backlash','Sweep KIRI → KANAN','Sweep KANAN → KIRI',
 'Capture Titik LUT','TERAPKAN LUT PART 2','steering_lut_physical_deg',
 'steering_lut_command_increasing_deg','steering_lut_command_decreasing_deg',
 'steering_lut_feedback_increasing_deg','steering_lut_feedback_decreasing_deg',
 'steering_calibration.lut_part2_draft','Ekspor LUT CSV']:
    assert token in gui, token
assert 'steering_lut_point_count' in v
assert 'steering_lut_max_command_hysteresis_deg' in v
assert 'steering_lut_max_feedback_hysteresis_deg' in v
assert 'QVariant::List' in gui and 'PARAMETER_DOUBLE_ARRAY' in gui
print('PASS steering physical Part 2 LUT contract')
