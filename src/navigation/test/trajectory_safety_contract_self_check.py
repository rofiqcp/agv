#!/usr/bin/env python3
"""Offline contract checks for trajectory-safety geometry/configuration.

This does not require ROS 2. It guards the intended geometry contract used by
trajectory_safety_supervisor.cpp, especially the parked-car-across-a-turn case.
"""
from __future__ import annotations

import math
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
CFG = yaml.safe_load((ROOT / "config" / "trajectory_safety.yaml").read_text())[
    "trajectory_safety_supervisor"
]["ros__parameters"]


def point_to_corridor(obstacle, path, half_width, horizon):
    best_lat = math.inf
    best_s = math.inf
    cumulative = 0.0
    for a, b in zip(path, path[1:]):
        vx, vy = b[0] - a[0], b[1] - a[1]
        seg = math.hypot(vx, vy)
        if seg <= 1e-9:
            continue
        if cumulative > horizon:
            break
        wx, wy = obstacle[0] - a[0], obstacle[1] - a[1]
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / (vx * vx + vy * vy)))
        px, py = a[0] + t * vx, a[1] + t * vy
        lat = math.hypot(obstacle[0] - px, obstacle[1] - py)
        along = cumulative + t * seg
        if along <= horizon and lat < best_lat:
            best_lat, best_s = lat, along
        cumulative += seg
    return best_lat <= half_width and best_s <= horizon, best_s, best_lat


vehicle_half = 0.5 * float(CFG["vehicle_width_m"])
collision_half = vehicle_half + float(CFG["trajectory_lateral_margin_m"])
planning_half = vehicle_half + float(CFG["planning_lateral_margin_m"])
horizon = float(CFG["path_horizon_m"])

# Future path curves left after the first metre.
path = [
    (0.0, 0.0),
    (0.8, 0.0),
    (1.2, 0.15),
    (1.55, 0.45),
    (1.75, 0.90),
    (1.80, 1.50),
    (1.80, 2.20),
]

# Close in Euclidean image/ground sense, but on the outside of the turn.
parked_across_turn = (1.85, -0.85)
relevant, _, lateral = point_to_corridor(parked_across_turn, path, collision_half, horizon)
assert not relevant, ("parked car across turn must not hard-stop", lateral, collision_half)

# Obstacle physically on the future path must be relevant.
on_path = (0.70, 0.0)
relevant, along, _ = point_to_corridor(on_path, path, collision_half, horizon)
assert relevant and along <= float(CFG["hard_stop_path_distance_m"])

# Wider planning envelope may retain nearby object while narrow collision tube does not.
near_path = (1.15, -0.55)
narrow, _, _ = point_to_corridor(near_path, path, collision_half, horizon)
wide, _, _ = point_to_corridor(near_path, path, planning_half, horizon)
assert not narrow and wide

# Current-command emergency horizon must stay shorter than normal future-path horizon.
assert float(CFG["command_collision_horizon_m"]) < float(CFG["slow_path_distance_m"])
assert float(CFG["immediate_command_hard_stop_m"]) <= float(CFG["command_collision_horizon_m"])

# Camera health and plan/obstacle stream are fail-closed in final autonomous config.
assert CFG["require_camera_connected"] is True
assert CFG["require_camera_health"] is True
assert CFG["require_plan_when_moving"] is True
assert CFG["require_obstacle_stream_when_moving"] is True

# Lane safety is an independent opt-in. Its default must never alter obstacle-
# only trajectory safety, and every lane-driven command branch is source-gated.
assert CFG["lane_safety_enabled"] is False
source = (ROOT / "src" / "trajectory_safety_supervisor.cpp").read_text()
for guarded_branch in (
    "lane_safety_enabled_ && lane_control_fresh && lane_recenter_blocked",
    "lane_safety_enabled_ && !path_obstacle_near && lane_fresh && lane_valid",
    "lane_safety_enabled_ && stop_on_lane_lost_ && lane_fresh",
):
    assert guarded_branch in source, f"lane OFF isolation missing: {guarded_branch}"

print("PASS trajectory safety contract")
