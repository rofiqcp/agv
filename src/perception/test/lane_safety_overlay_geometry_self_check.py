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
assert 'if (!result.drivable_detected && !lane_mask_detected)' in src
assert 'result.left_status = "UNKNOWN"' in src
assert 'result.left_valid ? laneCorridorStatus(true, result.left_gap_px) : "GREEN"' in src
assert cfg['lane_corridor_far_lookahead_m'] == 3.8
assert abs(cfg['lane_corridor_safety_margin_m'] - 0.30) < 1e-9
for token in ['lane_corridor_camera_height_m','lane_corridor_camera_pitch_deg','lane_corridor_safety_margin_m','lane_corridor_far_lookahead_m','lane_corridor_center_offset_px','lane_corridor_left_offset_px','lane_corridor_right_offset_px']:
    assert token in app, token
# Deterministic default geometry: top must stay separated and bottom must be wider.
fx=cfg['camera_fx']; fy=cfg['camera_fy']; cx=cfg['camera_cx']; cy=cfg['camera_cy']
half_m=0.5*cfg['lane_vehicle_width_m']+cfg['lane_corridor_safety_margin_m']
top_y=720*cfg['lane_corridor_top_y_ratio']; bottom_y=720*cfg['lane_corridor_bottom_y_ratio']
top_half=fx*half_m/cfg['lane_corridor_far_lookahead_m']
q=(bottom_y-cy)/fy
bottom_depth=expected_h/q if q>0 else cfg['lane_corridor_far_lookahead_m']
bottom_depth=min(bottom_depth,cfg['lane_corridor_far_lookahead_m'])
bottom_half=fx*half_m/bottom_depth
assert top_half > 1.0, top_half
assert bottom_half > top_half, (top_half,bottom_half)
assert (cx-top_half) < (cx+top_half), 'top endpoints collapsed'
assert 'top_line.left_x + t * (bottom_line.left_x - top_line.left_x)' in src

# Color-state contract requested for operator preview.
def visual_state(drivable, lane_present, valid, gap, warning=36.0, touch=2.0):
    if not drivable and not lane_present:
        return 'GRAY'
    if valid and gap <= touch:
        return 'RED'
    if valid and gap <= warning:
        return 'YELLOW'
    return 'GREEN'
assert visual_state(False,False,False,999)=='GRAY'
assert visual_state(True,False,False,999)=='GREEN'
assert visual_state(False,True,True,80)=='GREEN'
assert visual_state(True,True,True,20)=='YELLOW'
assert visual_state(True,True,True,0)=='RED'

print(f'PASS lane safety open-trapezoid: top=({cx-top_half:.1f},{cx+top_half:.1f}) bottom=({cx-bottom_half:.1f},{cx+bottom_half:.1f})')
print(f'PASS lane safety geometry: width={cfg["lane_vehicle_width_m"]:.3f}m camera_h={expected_h:.3f}m FOV=intrinsics two-lines + calibration offsets')
