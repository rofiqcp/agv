#!/usr/bin/env python3
"""Read-only Stage-2 commissioning check for final GNSS/IMU EKF ownership.

Final contract:
  local EKF  : GNSS vx + GNSS-derived vyaw + IMU absolute yaw
  global EKF : GNSS map x/y + same GNSS vx/vyaw + IMU absolute yaw
  TF         : local EKF owns odom->base; LocalizationCore owns map->odom
  ESC        : optional for localization; still required before actuator motion.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import yaml


def find_workspace(explicit: str | None) -> Path:
    if explicit:
        p = Path(explicit).expanduser().resolve()
        if (p / "src/navigation/config").is_dir():
            return p
        raise FileNotFoundError(f"workspace tidak memiliki src/navigation/config: {p}")
    for origin in (Path.cwd().resolve(), Path(__file__).resolve()):
        for p in (origin, *origin.parents):
            if (p / "src/navigation/config").is_dir():
                return p
    raise FileNotFoundError("workspace ROS tidak ditemukan; gunakan --workspace")


def params(path: Path, node: str) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return dict(data[node]["ros__parameters"])


def enabled(cfg: list[Any] | None) -> set[int]:
    return {i for i, value in enumerate(cfg or []) if bool(value)}


def yn(v: bool) -> str:
    return "PASS" if v else "WAIT"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", default=None)
    ap.add_argument("--require-ready", action="store_true",
                    help="return non-zero while field evidence is incomplete")
    a = ap.parse_args()
    ws = find_workspace(a.workspace)
    nav = ws / "src/navigation"

    vehicle = params(nav / "config/vehicle.yaml", "vehicle")
    imu = params(nav / "config/imu.yaml", "data_imu_node")
    loc = params(nav / "config/localization_cpp.yaml", "localization_core")
    navcore = params(nav / "config/navigation_core.yaml", "navigation_core")
    ekf = yaml.safe_load((nav / "config/ekf.yaml").read_text(encoding="utf-8")) or {}
    local = ekf["ekf_filter_node_odom"]["ros__parameters"]
    global_ = ekf["ekf_filter_node_map"]["ros__parameters"]

    errors: list[str] = []
    waits: list[str] = []

    def require(ok: bool, msg: str) -> None:
        if not ok:
            errors.append(msg)

    def readiness(ok: bool, msg: str) -> None:
        if not ok:
            waits.append(msg)

    # Estimator ownership.
    require(local.get("publish_tf") is True and local.get("world_frame") == "odom",
            "Local EKF harus publish odom→base dengan world_frame=odom")
    require("odom0" not in local,
            "Local EKF tidak boleh bergantung pada /esc/odom")
    require(local.get("twist0") == "/gnss/base_velocity_fusion" and
            enabled(local.get("twist0_config")) == {6, 11},
            "Local EKF harus fusion GNSS vx+vyaw")
    require(local.get("imu0") == "/imu/data" and enabled(local.get("imu0_config")) == {5},
            "Local EKF harus fusion IMU yaw absolut saja")

    require(global_.get("publish_tf") is False and global_.get("world_frame") == "map",
            "Global EKF publish_tf harus false dengan world_frame=map")
    require(global_.get("odom0") == "/odometry/gnss_map" and
            enabled(global_.get("odom0_config")) == {0, 1},
            "Global EKF harus fusion GNSS map x/y")
    if loc.get("enable_global_gnss_cog_fusion"):
        require(global_.get("twist0") == "/gnss/base_velocity_fusion" and
                enabled(global_.get("twist0_config")) == {6},
                "Global EKF (COG aktif) harus fusion GNSS vx saja; yaw-rate authority ke IMU gyro")
    else:
        require(global_.get("twist0") == "/gnss/base_velocity_fusion" and
                enabled(global_.get("twist0_config")) == {6, 11},
                "Global EKF (COG off) harus fusion GNSS vx+vyaw")
    if loc.get("enable_global_gnss_cog_fusion"):
        require(global_.get("imu0") == "/imu/data" and
                enabled(global_.get("imu0_config")) == {11},
                "Global EKF harus fusion IMU vyaw-only saat COG aktif")
        require(global_.get("pose0") == "/gnss/cog_heading_fusion" and
                enabled(global_.get("pose0_config")) == {5},
                "Global EKF harus fusion GNSS COG absolute yaw via pose0")
    else:
        require(global_.get("imu0") == "/imu/data" and
                enabled(global_.get("imu0_config")) == {5},
                "Global EKF harus fusion IMU yaw absolut saat COG off")
        require(not any(isinstance(v, str) and "/gnss/cog_heading_fusion" in v
                        for v in global_.values()),
                "GNSS COG absolute yaw tidak boleh masuk global EKF saat COG off")

    # GNSS vyaw integrity and low-speed de-weighting.
    require(loc.get("enable_global_gnss_velocity_fusion") is True,
            "GNSS vx fusion harus aktif agar estimator tidak tergantung ESC")
    require(loc.get("enable_global_gnss_cog_fusion") is True,
            "GNSS absolute COG yaw fusion harus ON untuk heading sesuai arah gerak")
    require(loc.get("enable_gnss_course_yaw_correction") is False,
            "legacy direct COG yaw correction harus OFF")
    require(loc.get("gnss_require_measurement_timestamp") is True,
            "measurement timestamp gate harus aktif")
    require(float(loc.get("gnss_max_measurement_age_sec", 0.0)) > 0.0,
            "max measurement age harus positif")
    require(float(loc.get("gnss_quality_timeout_sec", 0.0)) > 0.0,
            "GNSS quality timeout harus positif")
    min_cov = float(loc.get("gnss_velocity_covariance_min_variance", 0.0))
    max_cov = float(loc.get("gnss_velocity_covariance_max_variance", 0.0))
    require(math.isfinite(min_cov) and math.isfinite(max_cov) and 0.0 < min_cov < max_cov,
            "batas covariance GNSS velocity tidak valid")
    yaw_min_var = float(loc.get("gnss_yaw_rate_min_variance", 0.0))
    yaw_max_var = float(loc.get("gnss_yaw_rate_max_variance", 0.0))
    require(0.0 < yaw_min_var < yaw_max_var,
            "batas covariance GNSS vyaw tidak valid")
    require(float(loc.get("gnss_yaw_rate_min_speed_mps", 0.0)) > 0.0,
            "low-speed gate GNSS vyaw harus > 0")
    require(imu.get("require_fresh_gyro_for_imu_publish") is False,
            "fresh IMU quaternion yaw tidak boleh diblok oleh gyro packet")
    require(navcore.get("require_imu_calibration_for_autonomy") is True,
            "autonomy harus memerlukan IMU stationary calibration")

    # Field evidence. These gates affect autonomous approval, not estimator startup.
    steer = bool(vehicle.get("steering_calibration_valid", False))
    circle = bool(vehicle.get("steering_circle_calibration_valid", False))
    drive = bool(vehicle.get("drive_odometry_calibration_valid", False))
    imu_ok = bool(imu.get("stationary_calibration_valid", False))
    vel_cert = bool(loc.get("gnss_velocity_calibration_valid", False))

    readiness(steer, "steering physical calibration belum valid")
    readiness(circle, "circle/effective geometry calibration belum valid")
    readiness(drive, "drive odometry calibration belum valid (tidak menghalangi EKF, tetapi diperlukan untuk actuator validation)")
    readiness(imu_ok, "IMU stationary calibration belum valid")
    readiness(vel_cert, "GNSS Doppler vx/vyaw field validation belum valid")

    print("STAGE-2 GNSS/IMU EKF PREFLIGHT")
    print(f"workspace : {ws}")
    print("ownership : GNSS x/y + vx/vyaw | IMU yaw | ESC optional localization")
    for name, value in (
        ("steering physical", steer), ("circle geometry", circle),
        ("drive odometry", drive), ("IMU stationary", imu_ok),
        ("GNSS vx/vyaw validation", vel_cert),
    ):
        print(f"  {name:28s}: {yn(value)}")
    print(f"  GNSS velocity fusion         : {bool(loc.get('enable_global_gnss_velocity_fusion', False))}")
    print(f"  GNSS absolute COG yaw fusion : {bool(loc.get('enable_global_gnss_cog_fusion', False))}")

    if errors:
        print("\nCONFIG ERROR:")
        for e in errors:
            print(f"  - {e}")
        return 2
    if waits:
        print("\nCONFIG SAFE; ESTIMATOR BOLEH RUN, FIELD COMMISSIONING BELUM LENGKAP:")
        for w in waits:
            print(f"  - {w}")
        return 3 if a.require_ready else 0
    print("\nSTAGE-2 READY: estimator ownership dan field validation PASS.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
