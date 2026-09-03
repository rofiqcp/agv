#!/usr/bin/env python3
from pathlib import Path
import math
import yaml

ROOT = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((ROOT / 'config' / 'trajectory_safety.yaml').read_text())['trajectory_safety_supervisor']['ros__parameters']
VEHICLE_W = float(CFG['vehicle_width_m'])
SIDE_MARGIN = float(CFG['avoidance_side_margin_m'])
REQUIRED = VEHICLE_W + 2.0 * SIDE_MARGIN
HARD = float(CFG['hard_stop_path_distance_m'])
IMMEDIATE = float(CFG['immediate_command_hard_stop_m'])


def free_corridor(left_boundary, right_boundary, obs_right, obs_left):
    left_available = max(0.0, left_boundary - obs_left)
    right_available = max(0.0, obs_right - right_boundary)
    return left_available >= REQUIRED, right_available >= REQUIRED, left_available, right_available


def decide(path_distance, command_distance, left_free, right_free, drivable=True):
    if command_distance is not None and command_distance <= IMMEDIATE:
        return 'IMMEDIATE_COMMAND_HARD_STOP'
    if path_distance is None:
        return 'NAV2_PASS'
    if path_distance <= HARD:
        if not drivable:
            return 'DRIVABLE_SPACE_UNKNOWN_STOP'
        if not (left_free or right_free):
            return 'NO_SAFE_CORRIDOR_STOP'
        if command_distance is not None and command_distance <= HARD:
            return 'AVOIDANCE_NOT_ENGAGED_STOP'
        return 'AVOIDANCE_PASS'
    if path_distance < float(CFG['slow_path_distance_m']):
        return 'SEARCH_AVOIDANCE' if (left_free or right_free) else 'PATH_OBSTACLE_SLOW_SEARCH'
    return 'NAV2_PASS'


# Case 1: parked car dekat kamera tetapi di luar future path -> tidak ikut safety stop.
assert decide(None, None, False, False) == 'NAV2_PASS'

# Case 2: obstacle tepat di path, kiri cukup 1.4 m, kanan 0.65 m -> bypass kiri tersedia.
lf, rf, la, ra = free_corridor(2.2, -0.85, -0.20, 0.80)
assert REQUIRED == 1.10
assert lf and not rf, (lf, rf, la, ra)
assert decide(0.80, None, lf, rf) == 'AVOIDANCE_PASS'

# Case 3: ruang kiri/kanan tidak cukup -> stop sebagai pilihan terakhir.
lf, rf, _, _ = free_corridor(0.95, -0.90, -0.20, 0.20)
assert not lf and not rf
assert decide(0.75, None, lf, rf) == 'NO_SAFE_CORRIDOR_STOP'

# Case 4: ruang ada, tetapi command MPPI masih menuju obstacle dekat -> jangan menerobos.
lf, rf, _, _ = free_corridor(2.1, -2.0, -0.25, 0.65)
assert lf or rf
assert decide(0.80, 0.80, lf, rf) == 'AVOIDANCE_NOT_ENGAGED_STOP'

# Case 5: near-field/current swept command sangat dekat selalu emergency stop walaupun samping bebas.
assert decide(0.50, 0.50, True, True) == 'IMMEDIATE_COMMAND_HARD_STOP'

print('PASS avoidance contract')
print(f'required_corridor={REQUIRED:.2f}m hard={HARD:.2f}m immediate={IMMEDIATE:.2f}m')
