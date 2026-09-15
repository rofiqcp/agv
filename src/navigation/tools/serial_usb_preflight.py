#!/usr/bin/env python3
"""Read-only serial preflight for the production F411 ROS architecture.

Production routing:
  * STM32F411 USB CDC (0483:5740) owns HMI + NEO-3/NEO3PRO GNSS/MAG.
  * STM32F103/VESC is direct on the dedicated CH340 USB-UART (1a86:7523).
  * Yahboom IMU remains a direct CP2102 serial device.
  * Direct GNSS USB recovery requires an explicit non-ESC port.

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
    "ESC": ("1a86", "7523"),
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
        failures.append("F411 HMI/GNSS selector must default to auto")
    if "DeclareLaunchArgument('gnss_source', default_value='stm32'" not in launch:
        failures.append("production GNSS launch source is not stm32")
    if "DeclareLaunchArgument('esc_transport_mode', default_value='direct_vesc'" not in launch:
        failures.append("production ESC transport is not direct_vesc")
    if str(gnss_legacy.get("auto_port_path_contains", "")).strip():
        failures.append("GNSS explicit USB recovery must not use physical by-path fallback")
    if str(esc_legacy.get("serial_auto_path_contains", "")).strip():
        failures.append("ESC direct serial must not use physical by-path fallback")
    if str(imu.get("auto_port_path_contains", "")).strip():
        failures.append("IMU must not use topology-dependent physical by-path fallback")

    print("SERIAL PREFLIGHT — DIRECT ESC + F411 GNSS/HMI — READ ONLY")
    print("route: GNSS/HMI -> F411 CDC | ESC/F103 -> CH340 direct | IMU -> CP2102")
    f411 = by_id_matches(lambda n: "STMICROELECTRONICS" in n and "F411" in n and "CDC" in n)
    esc_sel = str(esc_legacy.get("serial_auto_id_contains", "")).upper()
    imu_sel = str(imu.get("auto_port_id_contains", "")).upper()
    esc_matches = by_id_matches(lambda n: bool(esc_sel) and esc_sel in n)
    imu_matches = by_id_matches(lambda n: bool(imu_sel) and imu_sel in n)
    f411_dev = resolve_one("F411", f411, EXPECTED["F411"], failures)
    esc_dev = resolve_one("ESC", esc_matches, EXPECTED["ESC"], failures)
    imu_dev = resolve_one("IMU", imu_matches, EXPECTED["IMU"], failures)
    resolved = [d for d in (f411_dev, esc_dev, imu_dev) if d is not None]
    if len(set(resolved)) != len(resolved):
        failures.append("F411, ESC, and IMU must resolve to distinct tty devices")

    if str(gnss_legacy.get('auto_port_id_contains','')) != 'EXPLICIT_GNSS_USB_PORT_REQUIRED':
        failures.append("GNSS USB auto-selector must be disabled because CH340 belongs to ESC")
    print("ROUTING selectors:")
    print(f"  ESC  direct by-id={esc_legacy.get('serial_auto_id_contains','')!r} by-path=DISABLED")
    print("  GNSS direct USB: explicit port required; auto discovery disabled")

    if failures:
        print("RESULT: FAIL")
        for item in failures:
            print(f"  - {item}")
        return 2
    print("RESULT: PASS — F411, direct ESC, and IMU identities are distinct and topology-safe")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
