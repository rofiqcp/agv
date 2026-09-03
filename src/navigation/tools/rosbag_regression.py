#!/usr/bin/env python3
"""Safe rosbag helper for AGV regression / replay evidence.

Commands:
  rosbag_regression.py record --name straight_01
  rosbag_regression.py play BAG_DIR
  rosbag_regression.py info BAG_DIR

Record includes localization, commands, ESC feedback, IMU/GNSS and safety state.
Playback never starts the vehicle by itself; keep ESC disarmed for offline replay.
"""
from __future__ import annotations
import argparse, os, subprocess
from datetime import datetime
from pathlib import Path

TOPICS = [
    "/tf", "/tf_static", "/imu/data", "/gnss/fix_raw", "/gnss/quality",
    "/gnss/vel", "/gnss/velocity_position_fit", "/gnss/vel_map", "/gnss/base_velocity",
    "/gnss/base_velocity_fusion", "/gnss/cog_heading_fusion",
    "/gnss/motion_validation", "/gnss/velocity_qualified", "/gnss/cog_qualified",
    "/gnss/velocity_fusion_active", "/gnss/cog_fusion_active", "/gnss/fusion_status",
    "/gnss/speed_residual", "/gnss/course_residual", "/gnss/state", "/gnss/motion_diagnostics",
    "/esc/odom", "/odometry/filtered", "/odometry/filtered_map",
    "/cmd_vel_nav_raw", "/cmd_vel/autonomy_pre_smoother", "/cmd_vel/pre_smoother", "/cmd_vel/autonomy_integrated",
    "/cmd_vel/nav2_pre_collision", "/cmd_vel", "/cmd_vel/actuator",
    "/esc/drive_target_mps", "/esc/drive_actual_mps",
    "/esc/steering_target_rad", "/esc/steering_actual_rad",
    "/navigation/mppi_closed_loop/status", "/navigation/velocity_smoother/qualification",
    "/navigation/trajectory_safety_state", "/collision_monitor/state",
    "/system/localization_state", "/system/planning_localization_ready",
    "/system/motion_localization_ready", "/system/autonomy_motion_allowed",
    "/system/sensor_status", "/system/gnss_status", "/system/imu_status",
    "/system/ekf_local_status", "/system/ekf_global_status", "/system/pose_estimator",
    "/esc/kinematic_yaw_rate_rps", "/navigation/goal_state",
    "/plan", "/controller_server/transformed_global_plan",
]

def run(cmd):
    print("+", " ".join(cmd)); return subprocess.call(cmd)

def main():
    ap=argparse.ArgumentParser(); sub=ap.add_subparsers(dest="cmd",required=True)
    r=sub.add_parser("record"); r.add_argument("--name",default="agv_regression"); r.add_argument("--root",default="~/.ros/agv_bags")
    p=sub.add_parser("play"); p.add_argument("bag"); p.add_argument("--rate",type=float,default=1.0)
    i=sub.add_parser("info"); i.add_argument("bag")
    args=ap.parse_args()
    if args.cmd=="record":
        root=Path(args.root).expanduser(); root.mkdir(parents=True,exist_ok=True)
        bag=root/f"{args.name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        return run(["ros2","bag","record","-o",str(bag),*TOPICS])
    if args.cmd=="play":
        print("OFFLINE REPLAY: keep ESC disarmed / physical autonomy gate closed.")
        return run(["ros2","bag","play",str(Path(args.bag).expanduser()),"--rate",str(args.rate)])
    return run(["ros2","bag","info",str(Path(args.bag).expanduser())])

if __name__=="__main__": raise SystemExit(main())
