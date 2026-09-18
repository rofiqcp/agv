#!/usr/bin/env python3
"""Minimal Goal Pose field monitor for AGV Nav2 tests."""
from __future__ import annotations
import os, sys, shlex
from pathlib import Path

ROOT = Path('/home/sirobo/agv')

def _ensure_ros_python() -> None:
    try:
        import rclpy  # noqa: F401
        return
    except ModuleNotFoundError:
        pass
    if os.environ.get('_GOALPOSE_ROS_SOURCED') == '1':
        raise
    argv = shlex.join([sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]])
    cmd = (
        'source /opt/ros/humble/setup.bash && '
        f'source {shlex.quote(str(ROOT / "install/setup.bash"))} && '
        'export _GOALPOSE_ROS_SOURCED=1; '
        'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}; '
        'export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}; '
        f'exec {argv}'
    )
    os.execv('/bin/bash', ['bash', '-lc', cmd])

_ensure_ros_python()

import argparse, csv, json, math, signal, subprocess, time
from datetime import datetime
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import Buffer, TransformListener


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class GoalPoseMonitor(Node):
    def __init__(self, writer, csv_file, period: float):
        super().__init__('goalpose_monitor')
        self.writer, self.csv_file = writer, csv_file
        self.period = max(0.05, period)
        self.erpm = math.nan
        self.speed = math.nan
        self.yaw_rate = math.nan
        self.goal: Optional[PoseStamped] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, '/esc/foc/telemetry', self.on_foc, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/gnss/quality', self.on_quality, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/navigation/goal_request', self.on_goal, 10)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.on_goal, 10)
        self.create_timer(self.period, self.on_timer)

    def on_foc(self, msg: String) -> None:
        try:
            self.erpm = float(json.loads(msg.data).get('right', {}).get('erpm', math.nan))
        except Exception:
            pass

    def on_quality(self, msg: Float64MultiArray) -> None:
        if len(msg.data) > 6:
            self.speed = float(msg.data[6])

    def on_imu(self, msg: Imu) -> None:
        self.yaw_rate = float(msg.angular_velocity.z)

    def on_goal(self, msg: PoseStamped) -> None:
        self.goal = msg

    def on_timer(self) -> None:
        dist = math.nan
        yaw_err = math.nan
        if self.goal is not None:
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
                x = float(tf.transform.translation.x)
                y = float(tf.transform.translation.y)
                yaw = yaw_from_quat(tf.transform.rotation)
                gx = float(self.goal.pose.position.x)
                gy = float(self.goal.pose.position.y)
                dx, dy = gx - x, gy - y
                dist = math.hypot(dx, dy)
                target_bearing = math.atan2(dy, dx) if dist > 1.0e-6 else yaw
                yaw_err = wrap_pi(target_bearing - yaw)
            except Exception:
                pass
        now = datetime.now().isoformat(timespec='milliseconds')
        self.writer.writerow([
            now, f'{self.speed:.6f}', f'{self.erpm:.3f}',
            f'{self.yaw_rate:.6f}', f'{yaw_err:.6f}', f'{dist:.6f}'
        ])
        self.csv_file.flush()
        yaw_deg = math.degrees(yaw_err) if math.isfinite(yaw_err) else math.nan
        print(
            f'm/s {self.speed:6.3f} | ERPM {self.erpm:7.0f} | '
            f'yawrate {self.yaw_rate:+7.3f} rad/s | '
            f'yawerr {yaw_deg:+7.2f} deg | goal {dist:6.2f} m',
            flush=True)


def parse_args():
    p = argparse.ArgumentParser(description='Minimal Nav2 Goal Pose field monitor.')
    p.add_argument('--no-launch', action='store_true')
    p.add_argument('--print-period', type=float, default=0.20)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = ROOT / 'tools' / 'record' / stamp
    out_dir.mkdir(parents=True, exist_ok=True)
    (ROOT / 'log').mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / 'goalpose.csv'
    log_path = ROOT / 'log' / f'{stamp}.txt'
    launch_proc: Optional[subprocess.Popen] = None
    if not args.no_launch:
        launch_cmd = (
            f'cd {shlex.quote(str(ROOT))} && source /opt/ros/humble/setup.bash && '
            f'source {shlex.quote(str(ROOT / "install/setup.bash"))} && '
            'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}; '
            f'ros2 launch navigation autonomous.launch.py 2>&1 | tee {shlex.quote(str(log_path))} >/dev/null'
        )
        launch_proc = subprocess.Popen(
            ['/bin/bash', '-lc', launch_cmd], preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    rclpy.init()
    stop_requested = {'value': False}
    def _stop(_signum, _frame):
        stop_requested['value'] = True
    signal.signal(signal.SIGTERM, _stop)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, _stop)

    with csv_path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['time_iso','gnss_mps','right_erpm','yaw_rate_rps','yaw_error_to_goal_rad','goal_distance_m'])
        node = GoalPoseMonitor(w, f, args.print_period)
        try:
            while rclpy.ok() and not stop_requested['value']:
                rclpy.spin_once(node, timeout_sec=0.10)
                if launch_proc is not None and launch_proc.poll() is not None:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    if launch_proc is not None and launch_proc.poll() is None:
        try:
            os.killpg(launch_proc.pid, signal.SIGINT)
            launch_proc.wait(timeout=8.0)
        except Exception:
            try:
                os.killpg(launch_proc.pid, signal.SIGTERM)
            except Exception:
                pass
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
