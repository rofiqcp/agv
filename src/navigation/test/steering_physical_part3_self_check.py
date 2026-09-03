#!/usr/bin/env python3
from gui_source_helper import read_gui_source
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[1]
gui=read_gui_source(root)
spec=(root/'gui/agv_gui_specs.hpp').read_text(encoding='utf-8')
with open(root/'config/vehicle.yaml',encoding='utf-8') as f:
    v=yaml.safe_load(f)['vehicle']['ros__parameters']
for k in ['effective_wheelbase_m','effective_wheelbase_left_m','effective_wheelbase_right_m',
          'turning_radius_left_m','turning_radius_right_m','turning_radius_safety_margin_pct',
          'steering_circle_calibration_valid','steering_circle_trial_count_left',
          'steering_circle_trial_count_right','steering_circle_fit_rmse_m',
          'steering_circle_calibration_saved_at','minimum_turning_radius_source']:
    assert k in v,k
assert isinstance(v['steering_circle_calibration_valid'],bool)
for token in ['Part 3 — Circle Test & Kinematic Geometry Certification','Analisis Circle Test',
              'TERAPKAN GEOMETRI PART 3','CircleFit circleFit()const','effective_wheelbase_m',
              'turning_radius_left_m','turning_radius_right_m','measured_circle_part3',
              'den/(2.0*std::abs(y))','ri*std::tan','std::max(out.leftRadius,out.rightRadius)',
              'circleRmseMax_']:
    assert token in gui,token
assert 'Physical wheelbase' in spec and 'Effective kinematic wheelbase' in spec
# Physical wheelbase must not be synchronized directly to ESC after Part 3 split.
assert 'double physicalWb=vg("wheelbase_m",.70),wb=vg("effective_wheelbase_m",physicalWb)' in gui
print('PASS steering physical Part 3 circle/geometry contract')
