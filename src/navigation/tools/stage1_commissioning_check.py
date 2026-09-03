#!/usr/bin/env python3
"""Stage-1 on-robot preflight for serial identity, steering and odometry commissioning.

Run after `source install/setup.bash`. The script is intentionally read-only.
It reports what remains before autonomous motion can be unlocked.
"""
from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Dict, Tuple
import yaml


def load_params(path: Path, node: str) -> dict:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get(node, {}).get("ros__parameters", {})


def find_serial(selector: str, by_path: bool = False) -> list[Path]:
    root = Path("/dev/serial/by-path" if by_path else "/dev/serial/by-id")
    if not root.is_dir():
        return []
    return sorted(p for p in root.iterdir() if selector in p.name)


def resolved(path: Path) -> str:
    try:
        return str(path.resolve(strict=True))
    except OSError:
        return "<unresolved>"


def yn(v: bool) -> str:
    return "PASS" if v else "WAIT"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", default=os.environ.get("AGV_WORKSPACE", ""),
                    help="ROS workspace root; default auto-detect from current directory")
    args = ap.parse_args()

    ws = Path(args.workspace).expanduser().resolve() if args.workspace else Path.cwd().resolve()
    if (ws.name == "src"):
        ws = ws.parent
    while ws != ws.parent and not (ws / "src/navigation/config/vehicle.yaml").exists():
        ws = ws.parent
    nav = ws / "src/navigation"
    esc_dir = ws / "src/esc"
    if not nav.exists() or not esc_dir.exists():
        print("FAIL: workspace tidak ditemukan. Jalankan dari root workspace atau gunakan --workspace.")
        return 2

    vehicle = load_params(nav / "config/vehicle.yaml", "vehicle")
    gnss = load_params(nav / "config/gnss.yaml", "data_cuav_node")
    imu = load_params(nav / "config/imu.yaml", "data_imu_node")
    esc = load_params(esc_dir / "config/ackermann.yaml", "esc_ackermann")

    selectors: Dict[str, Tuple[str, str]] = {
        "ESC": (str(esc.get("serial_auto_id_contains", "")), str(esc.get("serial_auto_path_contains", ""))),
        "GNSS": (str(gnss.get("auto_port_id_contains", "")), str(gnss.get("auto_port_path_contains", ""))),
        "IMU": (str(imu.get("auto_port_id_contains", "")), str(imu.get("auto_port_path_contains", ""))),
    }

    print("=== STAGE-1 HARDWARE ROUTING (by-id primary, by-path fallback) ===")
    found: Dict[str, Tuple[Path, str] | None] = {}
    identity_ok = True
    for name, (id_selector, path_selector) in selectors.items():
        matches = find_serial(id_selector, by_path=False)
        source = "by-id"
        if len(matches) != 1:
            matches = find_serial(path_selector, by_path=True)
            source = "by-path-fallback"
        if len(matches) != 1:
            identity_ok = False
            found[name] = None
            print(f"{name:4s}: FAIL id={id_selector!r} path={path_selector!r} matches={len(matches)}")
            for p in matches:
                print(f"      - {p} -> {resolved(p)}")
        else:
            found[name] = (matches[0], resolved(matches[0]))
            print(f"{name:4s}: PASS [{source}] {matches[0]} -> {found[name][1]}")

    resolved_nodes = [v[1] for v in found.values() if v is not None]
    if len(resolved_nodes) != len(set(resolved_nodes)):
        identity_ok = False
        print("FAIL: dua selector serial resolve ke device /dev yang sama.")

    print("\n=== STAGE-1 CALIBRATION GATES ===")
    steer_ok = bool(vehicle.get("steering_calibration_valid", False))
    circle_ok = bool(vehicle.get("steering_circle_calibration_valid", False))
    drive_ok = bool(vehicle.get("drive_odometry_calibration_valid", False))
    print(f"Steering physical Part 1/2 : {yn(steer_ok)}")
    print(f"Circle geometry Part 3     : {yn(circle_ok)}")
    print(f"Drive odometry scale       : {yn(drive_ok)}")

    op = math.degrees(abs(float(vehicle.get("operational_steering_angle_rad", 0.0))))
    rmin = float(vehicle.get("minimum_turning_radius_m", 0.0))
    vmax = float(vehicle.get("max_forward_speed_mps", 0.0))
    yaw = float(vehicle.get("max_yaw_rate_rps", 0.0))
    print(f"Operational steering       : +/-{op:.3f} deg")
    print(f"Minimum turning radius     : {rmin:.6f} m")
    print(f"Yaw limit / kinematic cap  : {yaw:.6f} / {(vmax/rmin if rmin>0 else float('nan')):.6f} rad/s")

    ready = identity_ok and steer_ok and circle_ok and drive_ok
    print("\n=== RESULT ===")
    if ready:
        print("PASS: Stage-1 foundation lengkap. Autonomous gate boleh dibuka setelah runtime verification.")
        return 0
    print("WAIT: Stage-1 belum lengkap. Autonomous motion sengaja tetap fail-closed.")
    if not steer_ok:
        print("  1. GUI Steering: capture LEFT/CENTER/RIGHT, Apply Part 1; lanjutkan LUT Part 2 bila digunakan.")
    if not drive_ok:
        print("  2. GUI Odom: lakukan straight-run terukur dan Apply Drive Calibration.")
    if not circle_ok:
        print("  3. GUI Odom/Circle: minimal 2 trial kiri + 2 kanan, lalu Apply Part 3.")
    if not identity_ok:
        print("  0. Benahi USB identity/by-id atau fallback by-path; jangan pakai ttyUSB index tetap.")
    return 3


if __name__ == "__main__":
    raise SystemExit(main())
