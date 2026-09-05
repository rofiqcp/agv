#!/usr/bin/env python3
"""Regression check for Humble robot_localization + requested sensor ownership."""
from pathlib import Path
import math
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "ekf.yaml"
N = 15

def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)

def enabled(values):
    return {i for i, value in enumerate(values) if value}

def main() -> None:
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8")) or {}
    for node in ("ekf_filter_node_odom", "ekf_filter_node_map"):
        params = data.get(node, {}).get("ros__parameters", {})
        if not params:
            fail(f"missing {node}.ros__parameters")
        for key, value in params.items():
            if not key.endswith("_config"):
                continue
            if not isinstance(value, list) or len(value) != N or not all(type(v) is bool for v in value):
                fail(f"{node}.{key} must contain exactly {N} booleans")
        q = params.get("process_noise_covariance")
        if not isinstance(q, list) or len(q) != N * N:
            fail(f"{node}.process_noise_covariance must contain {N*N} values")
        if not all(type(v) is not bool and isinstance(v, (int, float)) and math.isfinite(float(v)) for v in q):
            fail(f"{node}.process_noise_covariance contains invalid values")
        for row in range(N):
            if float(q[row * N + row]) <= 0.0:
                fail(f"{node} Q[{row},{row}] must be > 0")
            for col in range(row + 1, N):
                if abs(float(q[row*N+col]) - float(q[col*N+row])) > 1e-12:
                    fail(f"{node} Q is not symmetric at [{row},{col}]")
        print(f"PASS {node}: Humble matrix/config shape valid")

    local = data["ekf_filter_node_odom"]["ros__parameters"]
    global_ = data["ekf_filter_node_map"]["ros__parameters"]
    if local.get("publish_tf") is not True or global_.get("publish_tf") is not False:
        fail("TF ownership must be local EKF odom->base, LocalizationCore map->odom")
    if local.get("odom0") != "/esc/odom" or enabled(local.get("odom0_config", [])) != {6}:
        fail("local EKF must fuse ESC longitudinal vx only")
    if local.get("twist0") != "/gnss/base_velocity_fusion" or enabled(local.get("twist0_config", [])) != {6}:
        fail("local EKF must fuse independent GNSS vx")
    if (local.get("imu0") != "/imu/data" or enabled(local.get("imu0_config", [])) != {5, 11} or
            local.get("imu0_relative") is not True):
        fail("local EKF must fuse relative IMU yaw + gyro-Z")
    if global_.get("odom0") != "/odometry/gnss_map" or enabled(global_.get("odom0_config", [])) != {0, 1}:
        fail("global EKF must fuse GNSS map x/y")
    if global_.get("twist0") != "/gnss/base_velocity_fusion" or enabled(global_.get("twist0_config", [])) != {6}:
        fail("global EKF must fuse GNSS forward velocity only")
    for key, topic in (("pose0", "/gnss/cog_heading_fusion"),
                       ("pose1", "/neo3/mag_heading_fusion"),
                       ("pose2", "/imu/mag_heading_fusion")):
        if global_.get(key) != topic or enabled(global_.get(key + "_config", [])) != {5}:
            fail(f"global EKF absolute heading source invalid: {key}")
    if (global_.get("imu0") != "/imu/data" or enabled(global_.get("imu0_config", [])) != {5, 11} or
            global_.get("imu0_relative") is not True):
        fail("global EKF must fuse relative IMU yaw + gyro-Z")
    print("PASS EKF sensor ownership: ESC/GNSS=vx; COG+dual-mag=absolute yaw; IMU=relative yaw+gyro-Z")

if __name__ == "__main__":
    main()
