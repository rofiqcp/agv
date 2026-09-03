#!/usr/bin/env python3
"""Read-only preflight for the final Mini-PC AGV profile.

Final estimator ownership:
  * GNSS position -> global x/y
  * GNSS Doppler velocity -> vx + GNSS-derived vyaw
  * IMU quaternion -> absolute yaw
  * ESC -> actuator/feedback only; serial may be disabled/offline without
    preventing map/localization from running.
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any

import yaml


def find_workspace(explicit: str | None) -> Path:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser().resolve())
    cwd = Path.cwd().resolve()
    candidates.extend((cwd, *cwd.parents))
    here = Path(__file__).resolve()
    candidates.extend((here, *here.parents))
    for candidate in candidates:
        if (candidate / "src/navigation/config").is_dir() and (candidate / "src/esc/config").is_dir():
            return candidate
    raise FileNotFoundError("workspace ROS tidak ditemukan; gunakan --workspace")


def load_params(path: Path, node: str) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return dict(data[node]["ros__parameters"])


def enabled_indices(config: list[Any] | None) -> set[int]:
    return {i for i, value in enumerate(config or []) if bool(value)}


def command_lines(command: list[str], timeout: float = 8.0) -> list[str]:
    try:
        result = subprocess.run(
            command, check=False, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return []
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def one_serial(selector: str, *, by_path: bool = False) -> list[Path]:
    root = Path("/dev/serial/by-path" if by_path else "/dev/serial/by-id")
    if not selector or not root.is_dir():
        return []
    return sorted(p for p in root.iterdir() if selector in p.name)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace")
    parser.add_argument("--runtime", action="store_true",
                        help="also inspect currently running ROS nodes/topics")
    parser.add_argument("--require-hardware", action="store_true",
                        help="require GNSS+IMU identities; ESC remains optional")
    parser.add_argument("--require-esc", action="store_true",
                        help="also require one matching ESC serial identity")
    parser.add_argument("--expect-esc-node", action="store_true",
                        help="when --runtime, require /esc_ackermann node")
    args = parser.parse_args()

    workspace = find_workspace(args.workspace)
    nav = workspace / "src/navigation"
    esc_dir = workspace / "src/esc"
    errors: list[str] = []
    waits: list[str] = []

    autonomous = (nav / "launch/autonomous.launch.py").read_text(encoding="utf-8")
    gui = (nav / "launch/gui.launch.py").read_text(encoding="utf-8")
    if (nav / "launch/minipc.launch.py").exists():
        errors.append("minipc.launch.py harus dihapus; gunakan autonomous.launch.py atau gui.launch.py")
    for label, source in (("autonomous", autonomous), ("gui", gui)):
        for token in ("perception_mode", "esc_serial_enabled", "gnss_port", "imu_port", "pt_model_path"):
            if token not in source:
                errors.append(f"{label} launch belum expose {token}")
    if "camera_only_node" not in autonomous:
        errors.append("perception_mode=off belum menjalankan camera_only_node")
    if "importlib.util.find_spec" in autonomous:
        errors.append("autonomous launch masih melakukan dependency probe Python lama")

    ekf = yaml.safe_load((nav / "config/ekf.yaml").read_text(encoding="utf-8")) or {}
    local = ekf["ekf_filter_node_odom"]["ros__parameters"]
    global_ = ekf["ekf_filter_node_map"]["ros__parameters"]
    loc = load_params(nav / "config/localization_cpp.yaml", "localization_core")
    # COG migration contract is validated together with EKF ownership below.

    if local.get("publish_tf") is not True or local.get("world_frame") != "odom":
        errors.append("EKF lokal harus menjadi owner odom→base_footprint")
    if "odom0" in local:
        errors.append("EKF lokal masih bergantung pada ESC odometry")
    if local.get("twist0") != "/gnss/base_velocity_fusion" or enabled_indices(local.get("twist0_config")) != {6, 11}:
        errors.append("EKF lokal harus memakai GNSS vx+vyaw")
    if local.get("imu0") != "/imu/data" or enabled_indices(local.get("imu0_config")) != {5}:
        errors.append("EKF lokal harus memakai IMU yaw absolut saja")

    if global_.get("publish_tf") is not False or global_.get("world_frame") != "map":
        errors.append("EKF global tidak boleh publish map→odom TF")
    if global_.get("odom0") != "/odometry/gnss_map" or enabled_indices(global_.get("odom0_config")) != {0, 1}:
        errors.append("EKF global harus memakai GNSS map x/y")
    if global_.get("twist0") != "/gnss/base_velocity_fusion" or enabled_indices(global_.get("twist0_config")) != {6}:
        errors.append("EKF global harus memakai GNSS vx saja (COG fusion memindah yaw-rate ke IMU gyro)")
    if global_.get("imu0") != "/imu/data":
        errors.append("EKF global IMU source changed")
    if loc.get("enable_global_gnss_cog_fusion"):
        if enabled_indices(global_.get("imu0_config")) != {11}:
            errors.append("EKF global harus memakai IMU vyaw-only ketika COG fusion aktif")
        if global_.get("pose0") != "/gnss/cog_heading_fusion":
            errors.append("EKF global harus memiliki pose0 COG ketika COG fusion aktif")
        if enabled_indices(global_.get("pose0_config")) != {5}:
            errors.append("EKF global COG harus yaw-only")
    else:
        if enabled_indices(global_.get("imu0_config")) != {5}:
            errors.append("EKF global harus memakai IMU yaw absolut saja ketika COG fusion off")
        if any(isinstance(v, str) and "/gnss/cog_heading_fusion" in v for v in global_.values()):
            errors.append("COG masih masuk EKF global padahal COG fusion off")

    vehicle = load_params(nav / "config/vehicle.yaml", "vehicle")
    imu = load_params(nav / "config/imu.yaml", "data_imu_node")
    esc = load_params(esc_dir / "config/ackermann.yaml", "esc_ackermann")
    gnss = load_params(nav / "config/gnss.yaml", "data_cuav_node")

    if loc.get("enable_global_gnss_velocity_fusion") is not True:
        errors.append("GNSS velocity fusion harus ON agar vx tersedia tanpa ESC")
    if loc.get("enable_global_gnss_cog_fusion") is not True:
        errors.append("GNSS COG absolute yaw harus ON untuk heading global yang sesuai arah gerak")
    if loc.get("enable_gnss_course_yaw_correction") is not False:
        errors.append("legacy direct GNSS COG yaw correction harus OFF")
    if float(loc.get("gnss_yaw_rate_min_speed_mps", 0.0)) <= 0.0:
        errors.append("GNSS vyaw low-speed gate tidak valid")
    if float(loc.get("gnss_yaw_rate_max_variance", 0.0)) <= float(loc.get("gnss_yaw_rate_min_variance", 0.0)):
        errors.append("GNSS vyaw covariance range tidak valid")
    if imu.get("require_fresh_gyro_for_imu_publish") is not False:
        errors.append("IMU absolute yaw masih dipaksa bergantung pada gyro freshness")
    if float(imu.get("orientation_publish_timeout_sec", 999.0)) >= float(imu.get("orientation_packet_timeout_sec", 0.0)):
        errors.append("freshness heading IMU tidak lebih ketat dari recovery timeout")

    gates = {
        "steering physical": bool(vehicle.get("steering_calibration_valid", False)),
        "circle geometry": bool(vehicle.get("steering_circle_calibration_valid", False)),
        "drive odometry": bool(vehicle.get("drive_odometry_calibration_valid", False)),
        "IMU stationary": bool(imu.get("stationary_calibration_valid", False)),
    }
    for name, ready in gates.items():
        if not ready:
            waits.append(f"kalibrasi {name} belum PASS")

    selectors = {
        # Stable USB identities are primary. Current hardware is distinct:
        # GNSS=CH340, IMU=CP2102, ESC=PL2303. by-path is fallback only.
        "ESC": (str(esc.get("serial_auto_id_contains", "")), str(esc.get("serial_auto_path_contains", ""))),
        "GNSS": (str(gnss.get("auto_port_id_contains", "")), str(gnss.get("auto_port_path_contains", ""))),
        "IMU": (str(imu.get("auto_port_id_contains", "")), str(imu.get("auto_port_path_contains", ""))),
    }
    hardware_rows: list[tuple[str, str, str, list[Path]]] = []
    for name, (id_selector, path_selector) in selectors.items():
        matches = one_serial(id_selector, by_path=False)
        kind, selector = "by-id", id_selector
        if len(matches) != 1:
            matches = one_serial(path_selector, by_path=True)
            kind, selector = "by-path-fallback", path_selector
        hardware_rows.append((name, kind, selector, matches))

    print("MINI-PC AGV PREFLIGHT")
    print(f"workspace: {workspace}")
    print("profile  : autonomous/gui; OFF=camera-only; ESC UART optional")
    print("fusion   : GNSS=x/y(global)+vx | COG=yaw(abs) | IMU=vyaw(continuity) | ESC=actuator/feedback only")
    print("TF       : LocalizationCore map→odom | local EKF odom→base_footprint")
    print("\nCALIBRATION GATES")
    for name, ready in gates.items():
        print(f"  {name:20s}: {'PASS' if ready else 'WAIT'}")
    print("\nSERIAL IDENTITY")
    def serial_rw(matches: list[Path]) -> bool:
        if len(matches) != 1:
            return False
        try:
            target = matches[0].resolve(strict=True)
        except OSError:
            return False
        return os.access(target, os.R_OK | os.W_OK)

    for name, kind, selector, matches in hardware_rows:
        resolved = str(matches[0].resolve()) if len(matches) == 1 else "--"
        access = "RW" if serial_rw(matches) else "NO-RW"
        state = "PASS" if len(matches) == 1 and serial_rw(matches) else ("OPTIONAL" if name == "ESC" and len(matches) != 1 else "WAIT")
        print(f"  {name:4s}: {state:8s} access={access:5s} {kind}={selector!r} -> {resolved}")
        if len(matches) == 1 and not serial_rw(matches):
            waits.append(f"permission serial {name} belum read/write untuk user aktif: {resolved}")

    hardware_required_ok = all(
        len(matches) == 1 and serial_rw(matches)
        for name, _kind, _selector, matches in hardware_rows if name in {"GNSS", "IMU"})
    esc_ok = next(
        (len(matches) == 1 and serial_rw(matches)
         for name, _kind, _selector, matches in hardware_rows if name == "ESC"), False)

    if args.runtime:
        if not shutil.which("ros2"):
            errors.append("ros2 CLI tidak ditemukan untuk --runtime")
        else:
            nodes = set(command_lines(["ros2", "node", "list"]))
            topics = set(command_lines(["ros2", "topic", "list"]))
            required_nodes = {
                "/data_cuav_node", "/data_imu_node", "/ekf_filter_node_odom",
                "/ekf_filter_node_map", "/localization_core", "/map_server",
                "/controller_server", "/planner_server", "/bt_navigator",
                "/velocity_smoother", "/navigation_core",
            }
            if args.expect_esc_node:
                required_nodes.add("/esc_ackermann")
            required_topics = {
                "/map", "/imu/data", "/gnss/fix_raw", "/gnss/base_velocity_fusion",
                "/odometry/filtered", "/odometry/filtered_map",
                "/robot_description", "/system/localization_state",
                "/system/autonomy_motion_allowed",
            }
            for missing in sorted(required_nodes - nodes):
                errors.append(f"runtime node hilang: {missing}")
            for missing in sorted(required_topics - topics):
                errors.append(f"runtime topic hilang: {missing}")
            print(f"\nRUNTIME: nodes={len(nodes)} topics={len(topics)}")

    if errors:
        print("\nCONFIG/RUNTIME ERROR")
        for error in errors:
            print(f"  - {error}")
        return 2
    if args.require_hardware and not hardware_required_ok:
        print("\nWAIT: GNSS/IMU serial belum lengkap.")
        return 3
    if args.require_esc and not esc_ok:
        print("\nWAIT: ESC serial diminta tetapi identity belum ditemukan.")
        return 3
    if waits:
        print("\nCONFIG PASS; COMMISSIONING MASIH WAIT")
        for wait in waits:
            print(f"  - {wait}")
    else:
        print("\nCONFIG + CALIBRATION GATES PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
