#!/usr/bin/env python3
"""Read-only serial preflight for the production F411 ROS architecture.

Production routing:
  * STM32F411 USB CDC (0483:5740) is the shared hardware gateway for
    CUAV NEO-3 GNSS + IST8310 and the STM32F103 VESC UART transport.
  * Yahboom IMU remains a direct CP2102 serial device.
  * Legacy direct CH340 GNSS / PL2303 ESC selectors are recovery-only and
    intentionally have no physical by-path fallback, preventing topology swaps.

No serial device is opened and no command is transmitted.
"""
from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path

import yaml

EXPECTED = {
    "F411": ("0483", "5740"),
    "IMU": ("10c4", "ea60"),
}


def params(path: Path, node: str) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return dict(data[node]["ros__parameters"])


def props(dev: Path) -> dict[str, str]:
    try:
        text = subprocess.check_output(
            ["udevadm", "info", "--query=property", f"--name={dev}"],
            text=True, stderr=subprocess.STDOUT, timeout=3.0)
    except (OSError, subprocess.SubprocessError):
        return {}
    out: dict[str, str] = {}
    for line in text.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k] = v
    return out


def by_id_matches(predicate) -> list[Path]:
    root = Path("/dev/serial/by-id")
    if not root.is_dir():
        return []
    return sorted(p for p in root.iterdir() if predicate(p.name.upper()))


def resolve_one(label: str, matches: list[Path], expected: tuple[str, str], failures: list[str]) -> Path | None:
    if len(matches) != 1:
        failures.append(f"{label}: expected exactly one by-id endpoint, found {len(matches)}")
        print(f"{label:4s}: FAIL by-id matches={len(matches)}")
        return None
    try:
        target = matches[0].resolve(strict=True)
    except OSError as exc:
        failures.append(f"{label}: unresolved symlink: {exc}")
        print(f"{label:4s}: FAIL unresolved {matches[0]}")
        return None
    u = props(target)
    actual = (u.get("ID_VENDOR_ID", "").lower(), u.get("ID_MODEL_ID", "").lower())
    rw = os.access(target, os.R_OK | os.W_OK)
    if actual != expected:
        failures.append(f"{label}: VID:PID {actual[0]}:{actual[1]} != {expected[0]}:{expected[1]}")
    if not rw:
        failures.append(f"{label}: {target} is not read/write for current user")
    ok = actual == expected and rw
    print(f"{label:4s}: {'PASS' if ok else 'FAIL'} dev={target} VID:PID={actual[0]}:{actual[1]} RW={'yes' if rw else 'no'}")
    return target


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", default=os.environ.get("AGV_ROOT", str(Path.home() / "agv")))
    args = ap.parse_args()
    ws = Path(args.workspace).expanduser().resolve()

    hmi = params(ws / "src/stmf4/config/hmi.yaml", "stmf4_hmi_bridge")
    imu = params(ws / "src/navigation/config/imu.yaml", "data_imu_node")
    gnss_legacy = params(ws / "src/navigation/config/gnss.yaml", "data_cuav_node")
    esc_legacy = params(ws / "src/esc/config/ackermann.yaml", "esc_ackermann")
    launch = (ws / "src/navigation/launch/autonomous.launch.py").read_text(encoding="utf-8")

    failures: list[str] = []
    if str(hmi.get("serial_device", "")).lower() != "auto":
        failures.append("F411 HMI/GNSS/VESC gateway selector must default to auto")
    if "DeclareLaunchArgument('gnss_source', default_value='stm32'" not in launch:
        failures.append("production GNSS launch source is not stm32")
    if "DeclareLaunchArgument('esc_transport_mode', default_value='stm32'" not in launch:
        failures.append("production ESC transport is not stm32")
    if str(gnss_legacy.get("auto_port_path_contains", "")).strip():
        failures.append("legacy GNSS direct serial must not use physical by-path fallback")
    if str(esc_legacy.get("serial_auto_path_contains", "")).strip():
        failures.append("legacy ESC direct serial must not use physical by-path fallback")
    if str(imu.get("auto_port_path_contains", "")).strip():
        failures.append("IMU must not use topology-dependent physical by-path fallback")

    print("SERIAL PREFLIGHT — PRODUCTION F411 ARCHITECTURE — READ ONLY")
    print("route: NEO-3 GNSS + IST8310 -> F411 CDC -> ROS; ROS VESC -> F411 -> F103; IMU -> CP2102")
    f411 = by_id_matches(lambda n: "STMICROELECTRONICS" in n and "F411" in n and "CDC" in n)
    imu_sel = str(imu.get("auto_port_id_contains", "")).upper()
    imu_matches = by_id_matches(lambda n: bool(imu_sel) and imu_sel in n)
    f411_dev = resolve_one("F411", f411, EXPECTED["F411"], failures)
    imu_dev = resolve_one("IMU", imu_matches, EXPECTED["IMU"], failures)
    if f411_dev is not None and imu_dev is not None and f411_dev == imu_dev:
        failures.append("F411 gateway and IMU resolve to the same tty")

    print("RECOVERY-ONLY selectors:")
    print(f"  GNSS direct by-id={gnss_legacy.get('auto_port_id_contains','')!r} by-path=DISABLED")
    print(f"  ESC  direct by-id={esc_legacy.get('serial_auto_id_contains','')!r} by-path=DISABLED")

    if failures:
        print("RESULT: FAIL")
        for item in failures:
            print(f"  - {item}")
        return 2
    print("RESULT: PASS — production F411/IMU identities are unambiguous and topology-safe")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
