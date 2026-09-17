#!/usr/bin/env python3
from pathlib import Path

src = (Path(__file__).resolve().parents[1] / "src" / "ackermann_controller_server.cpp").read_text(encoding="utf-8")
required = {
    "VESC standard SET_POS command": "constexpr std::uint8_t kVescSetPos = 9;" in src and "appendVescSetPos" in src,
    "ROS physical to VESC POS calibration": "physicalToVescPositionDeg" in src and "steering_vesc_pos_left_deg" in src and "steering_vesc_pos_center_deg" in src and "steering_vesc_pos_right_deg" in src,
    "VESC POS to ROS physical feedback": "vescPositionToPhysicalDeg" in src and "measured_steering_vesc_pos_deg_" in src,
    "runtime has no custom steering command": "appendVescSetSteeringDeg" not in src and "kHbSetSteeringDeg" not in src,
    "GET_VALUES position is raw steering authority": "position_deg = static_cast<double>(readI32Be(&p[32])) / 1000000.0" in src and "measured_steering_vesc_pos_deg_ = std::clamp(position_deg" in src,
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
