#!/usr/bin/env python3
from pathlib import Path

src = (Path(__file__).resolve().parents[1] / "src" / "ackermann_controller_server.cpp").read_text(encoding="utf-8")
required = {
    "pair skew metric": "pair_skew_ms" in src,
    "pair max skew metric": "pair_max_skew_ms" in src,
    "pair complete counter": "values_pair_complete_count_" in src,
    "pair reject counter": "values_pair_reject_count_" in src,
    "left feedback age": "left_age_ms" in src,
    "right feedback age": "right_age_ms" in src,
    "rejected pair resets mask": "values_pair_mask_ = 0U;" in src,
}
failed = [name for name, ok in required.items() if not ok]
for name, ok in required.items():
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
if failed:
    raise SystemExit("VESC transport observability contract failed: " + ", ".join(failed))
print("VESC transport observability contract PASS")
