#!/usr/bin/env python3
"""Read-only USB serial routing preflight for GNSS, IMU and ESC.

No serial device is opened and no actuator/sensor command is transmitted.
The check validates stable by-id selectors, physical fallback consistency,
permissions, VID:PID, and uniqueness of the resolved devices.
"""
from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path
from typing import Dict, Tuple

import yaml

EXPECTED_VIDPID = {
    "GNSS": ("1a86", "7523"),   # CH340
    "IMU":  ("10c4", "ea60"),   # CP2102
    "ESC":  ("067b", "2303"),   # Prolific PL2303
}


def load_params(path: Path, node: str) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return dict(data[node]["ros__parameters"])


def matches(root: Path, selector: str) -> list[Path]:
    if not selector or not root.is_dir():
        return []
    return sorted(p for p in root.iterdir() if selector in p.name)


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
            key, value = line.split("=", 1)
            out[key] = value
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", default="/home/otomasi/ros")
    args = ap.parse_args()
    ws = Path(args.workspace).expanduser().resolve()

    gnss = load_params(ws / "src/navigation/config/gnss.yaml", "data_cuav_node")
    imu = load_params(ws / "src/navigation/config/imu.yaml", "data_imu_node")
    esc = load_params(ws / "src/esc/config/ackermann.yaml", "esc_ackermann")
    selectors: Dict[str, Tuple[str, str]] = {
        "GNSS": (str(gnss.get("auto_port_id_contains", "")), str(gnss.get("auto_port_path_contains", ""))),
        "IMU": (str(imu.get("auto_port_id_contains", "")), str(imu.get("auto_port_path_contains", ""))),
        "ESC": (str(esc.get("serial_auto_id_contains", "")), str(esc.get("serial_auto_path_contains", ""))),
    }

    by_id = Path("/dev/serial/by-id")
    by_path = Path("/dev/serial/by-path")
    failures: list[str] = []
    warnings: list[str] = []
    resolved: dict[str, Path] = {}

    print("USB SERIAL PREFLIGHT — READ ONLY")
    print("authority: unique by-id -> by-path fallback -> fail closed")
    for name in ("GNSS", "IMU", "ESC"):
        id_sel, path_sel = selectors[name]
        ids = matches(by_id, id_sel)
        paths = matches(by_path, path_sel)
        source = "by-id"
        selected: Path | None = ids[0] if len(ids) == 1 else None
        if selected is None and len(paths) == 1:
            selected = paths[0]
            source = "by-path-fallback"
        if selected is None:
            failures.append(f"{name}: no unique route (by-id={len(ids)}, by-path={len(paths)})")
            print(f"{name:4s}: FAIL by-id={len(ids)} by-path={len(paths)}")
            continue

        try:
            target = selected.resolve(strict=True)
        except OSError as exc:
            failures.append(f"{name}: selected symlink unresolved: {exc}")
            print(f"{name:4s}: FAIL unresolved {selected}")
            continue
        resolved[name] = target
        rw = os.access(target, os.R_OK | os.W_OK)
        if not rw:
            failures.append(f"{name}: {target} is not read/write for current user")

        u = props(target)
        actual = (u.get("ID_VENDOR_ID", ""), u.get("ID_MODEL_ID", ""))
        expected = EXPECTED_VIDPID[name]
        if actual != expected:
            failures.append(f"{name}: VID:PID {actual[0]}:{actual[1]} != {expected[0]}:{expected[1]}")

        # A configured physical fallback must never point at a different device.
        if len(paths) == 1:
            try:
                fallback_target = paths[0].resolve(strict=True)
                if fallback_target != target:
                    failures.append(f"{name}: by-path fallback points to {fallback_target}, primary points to {target}")
            except OSError:
                warnings.append(f"{name}: by-path fallback currently unresolved; by-id remains primary")
        elif len(paths) == 0:
            warnings.append(f"{name}: configured by-path fallback is not currently present")
        else:
            failures.append(f"{name}: by-path fallback is ambiguous ({len(paths)} matches)")

        print(
            f"{name:4s}: {'PASS' if rw and actual == expected else 'FAIL'} "
            f"source={source:16s} dev={target} VID:PID={actual[0]}:{actual[1]} RW={'yes' if rw else 'no'}")

    if len(set(resolved.values())) != len(resolved):
        failures.append("two logical devices resolve to the same tty")

    if warnings:
        print("WARNINGS")
        for item in warnings:
            print(f"  - {item}")
    if failures:
        print("RESULT: FAIL")
        for item in failures:
            print(f"  - {item}")
        return 2
    print("RESULT: PASS — GNSS/IMU/ESC identities are unique and safe")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
