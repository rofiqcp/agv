#!/usr/bin/env python3
"""Read-only Stage-2 commissioning check for the current dual-EKF architecture.

Current contract:
  local EKF  : ESC wheel vx + independent GNSS vx + IMU gyro-Z
  global EKF : GNSS map x/y + GNSS vx + COG/validated absolute yaw + IMU gyro-Z
  TF         : local EKF owns odom->base; LocalizationCore owns map->odom
  ESC        : vx-only auxiliary source; never a yaw/heading authority.

Field completion is intentionally stricter than estimator startup.  A short
commissioning calibration may keep the stack operable, but it cannot satisfy
Stage-2 physical certification.
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
    mag = params(nav / "config/mag_heading.yaml", "mag_heading_fusion")
    mppi_supervisor = params(nav / "config/mppi_closed_loop.yaml", "mppi_closed_loop_supervisor")
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

    # Estimator ownership: match the current runtime and stage2 foundation guard.
    require(local.get("publish_tf") is True and local.get("world_frame") == "odom",
            "Local EKF harus publish odom→base dengan world_frame=odom")
    require(local.get("odom0") == "/esc/odom" and enabled(local.get("odom0_config")) == {6},
            "Local EKF harus memakai /esc/odom vx-only; wheel yaw bukan heading authority")
    require(local.get("twist0") == "/gnss/base_velocity_fusion" and
            enabled(local.get("twist0_config")) == {6},
            "Local EKF harus fusion GNSS vx sebagai aid independen")
    require(local.get("imu0") == "/imu/data" and enabled(local.get("imu0_config")) == {11} and
            local.get("imu0_relative") is True,
            "Local EKF harus fusion IMU gyro-Z saja")

    require(global_.get("publish_tf") is False and global_.get("world_frame") == "map",
            "Global EKF publish_tf harus false dengan world_frame=map")
    require(global_.get("odom0") == "/odometry/gnss_map" and
            enabled(global_.get("odom0_config")) == {0, 1},
            "Global EKF harus fusion GNSS map x/y")
    require(global_.get("twist0") == "/gnss/base_velocity_fusion" and
            enabled(global_.get("twist0_config")) == {6},
            "Global EKF harus fusion GNSS vx saja")
    require(global_.get("imu0") == "/imu/data" and enabled(global_.get("imu0_config")) == {11} and
            global_.get("imu0_relative") is True,
            "Global EKF harus fusion IMU gyro-Z saja")
    require(global_.get("pose0") == "/gnss/cog_heading_fusion" and
            enabled(global_.get("pose0_config")) == {5},
            "Global EKF harus fusion GNSS COG absolute yaw via pose0")
    require(global_.get("pose1") == "/heading/validated_fusion" and
            enabled(global_.get("pose1_config")) == {5},
            "Global EKF harus fusion validated heading consensus via pose1")
    require("pose2" not in global_, "Global EKF tidak boleh mempunyai raw magnetic heading authority kedua")

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
    require(imu.get("require_fresh_gyro_for_imu_publish") is True,
            "IMU publish harus tetap berada pada fresh gyro measurement epoch")
    require(float(local.get("frequency", 0.0)) >= 25.0,
            "Local EKF Stage-2 harus >=25 Hz (target runtime 30 Hz)")
    require(float(global_.get("frequency", 0.0)) >= 8.0,
            "Global EKF Stage-2 harus >=8 Hz")
    require(float(mppi_supervisor.get("min_odom_rate_hz", 0.0)) >= 25.0,
            "MPPI CLOSED_LOOP qualification harus memerlukan >=25 Hz odometry feedback")
    require(navcore.get("require_imu_calibration_for_autonomy") is True,
            "autonomy harus memerlukan IMU commissioning calibration")
    require(mag.get("motor_current_topic") == "/esc/motor_current_abs_a",
            "magnetic EMI qualification harus memakai canonical ESC motor-current topic")
    for key in ("field_calibration_valid", "field_calibration_stationary_duration_sec",
                "field_calibration_six_position_valid", "field_calibration_motor_interference_valid"):
        require(key in imu, f"IMU Stage-2 field evidence key hilang: {key}")
    for key in ("field_qualification_valid", "enable_current_emi_gate", "current_emi_gate_threshold_a"):
        require(key in mag, f"magnetic Stage-2 field evidence key hilang: {key}")

    # Field evidence. These gates are stricter than estimator startup.
    steer = bool(vehicle.get("steering_calibration_valid", False))
    circle = bool(vehicle.get("steering_circle_calibration_valid", False))
    drive = bool(vehicle.get("drive_odometry_calibration_valid", False))
    imu_commissioning = bool(imu.get("stationary_calibration_valid", False))
    imu_field = bool(imu.get("field_calibration_valid", False)) and imu.get("calibration_level") == "field_certified"
    mag_field = bool(mag.get("field_qualification_valid", False))
    control_field = bool(vehicle.get("control_field_calibration_valid", False))
    vel_cert = bool(loc.get("gnss_velocity_calibration_valid", False))
    cog_cert = bool(loc.get("gnss_cog_calibration_valid", False))

    readiness(steer, "steering physical calibration belum valid")
    readiness(circle, "circle/effective geometry calibration belum valid")
    readiness(drive, "drive odometry calibration belum valid")
    readiness(imu_field, "IMU long-stationary + six-position + EMI field calibration belum valid")
    readiness(mag_field, "magnetometer 8-direction/current interference qualification belum valid")
    readiness(control_field, "steering/yaw step-slalom-circle field tuning belum valid")
    readiness(vel_cert, "GNSS Doppler velocity field validation belum valid")
    readiness(cog_cert, "GNSS COG heading field validation belum valid")

    if bool(imu.get("field_calibration_valid", False)):
        require(imu.get("calibration_level") == "field_certified", "field IMU valid tetapi calibration_level bukan field_certified")
        require(float(imu.get("field_calibration_stationary_duration_sec", 0.0)) >= 1200.0, "field IMU valid tanpa >=1200 s stationary evidence")
        require(bool(imu.get("field_calibration_six_position_valid", False)), "field IMU valid tanpa six-position PASS")
        require(bool(imu.get("field_calibration_motor_interference_valid", False)), "field IMU valid tanpa motor-interference PASS")
    if mag_field:
        require(bool(str(mag.get("field_qualification_evidence_sha256", "")).strip()), "mag field valid tanpa evidence hash")
    if control_field:
        require(bool(str(vehicle.get("control_field_evidence_sha256", "")).strip()), "control field valid tanpa evidence hash")

    print("STAGE-2 PHYSICAL + EKF/NAV2/CONTROL PREFLIGHT")
    print(f"workspace : {ws}")
    print("ownership : local[ESC vx + GNSS vx + IMU wz] | global[GNSS xy/vx + COG/validated yaw + IMU wz]")
    for name, value in (
        ("steering physical", steer), ("circle geometry", circle),
        ("drive odometry", drive), ("IMU commissioning", imu_commissioning),
        ("IMU field certified", imu_field), ("mag field qualified", mag_field),
        ("control field tuned", control_field), ("GNSS velocity validation", vel_cert),
        ("GNSS COG validation", cog_cert),
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
