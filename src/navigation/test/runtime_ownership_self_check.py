#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
launch = (root / "launch" / "autonomous.launch.py").read_text(encoding="utf-8")
web = (root / "web" / "web_server.cpp").read_text(encoding="utf-8")

checks = {
    "semantic detector has explicit opt-in": "enable_semantic_calibration" in launch,
    "semantic condition uses explicit opt-in": "LaunchConfiguration('enable_semantic_calibration')" in launch,
    "web GUI has process singleton lock": "QLockFile instanceLock" in web,
    "duplicate web GUI is rejected": "Web GUI instance kedua ditolak" in web,
    "web GUI lock uses stable name": "agv_web_gui.lock" in web,
}

failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
if failed:
    raise SystemExit("runtime ownership contract failed: " + ", ".join(failed))
print("runtime ownership contract PASS")
