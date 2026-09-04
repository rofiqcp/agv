#!/usr/bin/env python3
from pathlib import Path
import re, yaml
ROOT=Path(__file__).resolve().parents[3]
urdf=(ROOT/'src/navigation/urdf/dimensions.xacro').read_text()
src=(ROOT/'src/perception/src/astra_yolop_cpu_pt_node.cpp').read_text()
app=(ROOT/'src/navigation/web/static/app.js').read_text()
with open(ROOT/'src/perception/config/astra_yolop_gpu.yaml') as f:
    cfg=yaml.safe_load(f)['perception']['ros__parameters']
def val(name):
    m=re.search(rf'name="{name}" value="([0-9.\-]+)"',urdf)
    assert m, name
    return float(m.group(1))
assert abs(val('total_width')-cfg['lane_vehicle_width_m']) < 1e-9
expected_h=val('vehicle_top_from_ground')+0.036
assert abs(expected_h-cfg['lane_corridor_camera_height_m']) < 1e-9
assert cfg['lane_corridor_camera_pitch_deg']==0.0
assert cfg['lane_corridor_center_offset_px']==0.0
assert cfg['lane_corridor_left_offset_px']==0.0
assert cfg['lane_corridor_right_offset_px']==0.0
assert 'laneSafetyLineAtRow' in src
assert 'cv::line(image, top_left, top_right' not in src
assert 'cv::line(image, bottom_left, bottom_right' not in src
assert 'TRAPEZOID' not in src
assert 'result.left_status = "UNKNOWN"' in src
assert 'result.left_valid ? laneCorridorStatus(true, result.left_gap_px) : "GREEN"' in src
for token in ['lane_corridor_camera_height_m','lane_corridor_camera_pitch_deg','lane_corridor_safety_margin_m','lane_corridor_center_offset_px','lane_corridor_left_offset_px','lane_corridor_right_offset_px']:
    assert token in app, token
print(f'PASS lane safety geometry: width={cfg["lane_vehicle_width_m"]:.3f}m camera_h={expected_h:.3f}m FOV=intrinsics two-lines + calibration offsets')
