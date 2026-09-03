#!/usr/bin/env python3
from gui_source_helper import read_gui_source
"""Static contract checks for GNSS Driver V2 Part 1.

Part 1 must improve receiver observability and diagnostics without changing EKF fusion.
"""
from pathlib import Path
import sys
try:
    import yaml
except Exception as exc:
    print(f"FAIL: python3-yaml required: {exc}", file=sys.stderr)
    raise SystemExit(2)

ROOT = Path(__file__).resolve().parents[1]

def fail(msg: str):
    print("FAIL:", msg, file=sys.stderr)
    raise SystemExit(1)

def load(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}

cfg = load(ROOT / "config/gnss.yaml")["data_cuav_node"]["ros__parameters"]
cpp = (ROOT / "src/gnss_node.cpp").read_text(encoding="utf-8")
hpp = (ROOT / "include/gnss/gnss_node.hpp").read_text(encoding="utf-8")
gui = read_gui_source(ROOT) + (ROOT / 'gui/agv_gui_specs.hpp').read_text(encoding='utf-8')
ekf = (ROOT / "config/ekf.yaml").read_text(encoding="utf-8")

required_cfg = {
    "configure_navigation_rate": True,
    "navigation_rate_hz": 10.0,
    "poll_nav_cov": True,
    "nav_cov_poll_rate_hz": 5.0,
    "poll_nav_dop": True,
    "nav_dop_poll_rate_hz": 2.0,
    "configure_dynamic_model": True,
    "dynamic_model": "automotive",
    "timestamp_mode": "auto",
    "position_fit_window_sec": 3.0,
    "position_fit_min_samples": 5,
}
for key, expected in required_cfg.items():
    if cfg.get(key) != expected:
        fail(f"gnss.yaml {key}={cfg.get(key)!r}, expected {expected!r}")

# Official M9 CFG-RATE keys and NAV message IDs.
for token in [
    "0x30210001u", "0x30210002u",  # CFG-RATE-MEAS/NAV
    "0x20110021u",  # CFG-NAVSPG-DYNMODEL
    "sendUbxMessage(0x01, 0x36, {})",  # NAV-COV poll
    "sendUbxMessage(0x01, 0x04, {})",  # NAV-DOP poll
    "msg_id == 0x36", "msg_id == 0x04",
]:
    if token not in cpp:
        fail(f"GNSS V2 protocol contract missing: {token}")

# NAV-PVT full motion/time fields must be parsed at canonical offsets.
for token in [
    "&payload[0]", "&payload[12]", "&payload[16]",  # iTOW, tAcc, nano
    "&payload[44]",  # vAcc
    "&payload[48]", "&payload[52]", "&payload[56]",  # velN/E/D
    "&payload[60]", "&payload[64]", "&payload[68]", "&payload[72]",  # gSpeed/headMot/sAcc/headAcc
    "&payload[78]",  # flags3
]:
    if token not in cpp:
        fail(f"NAV-PVT field offset contract missing: {token}")

# Velocity must use velE/N/D directly, not only gSpeed/course reconstruction.
for token in ["vel_e_mps_", "vel_n_mps_", "vel_d_mps_", "NED Down -> ENU Up"]:
    if token not in cpp + hpp:
        fail(f"direct Doppler velocity contract missing: {token}")

# Epoch/timestamp discipline.
for token in ["updatePvtRate", "ubx_duplicate_itow_count_", "ubx_out_of_order_itow_count_",
              "makeMeasurementStamp", "ubx_utc", "itow_aligned", "pending_ubx_publish_", "stamp_ns > arrival.nanoseconds()"]:
    if token not in cpp + hpp:
        fail(f"measurement-time/epoch contract missing: {token}")


# Extended NAV-PVT flags/diagnostics and invalid-LLH gate.
for token in ["last_ubx_flags_", "last_ubx_invalid_llh_", "ubx-invalid-llh",
              "head_vehicle_rad_", "mag_declination_rad_", "carrier_solution", "last_correction_age_code"]:
    if token not in cpp + hpp + gui:
        fail(f"extended NAV-PVT diagnostic contract missing: {token}")

# NAV-COV must be NED->ENU transformed for position and velocity covariance.
for token in ["nav_cov_.pos_ee", "nav_cov_.pos_nn", "-nav_cov_.pos_ed",
              "nav_cov_.vel_ee", "nav_cov_.vel_nn", "-nav_cov_.vel_ed"]:
    if token not in cpp:
        fail(f"NAV-COV NED->ENU contract missing: {token}")

# Position-fit diagnostic must exist but must not enter EKF yet.
for token in ["gnss/velocity_position_fit", "publishPositionFit", "position_fit_min_baseline_m_",
              "speed_fit_minus_doppler_mps", "course_fit_minus_cog_rad"]:
    if token not in cpp + hpp:
        fail(f"multi-point position-fit contract missing: {token}")
if "/gnss/vel" in ekf or "velocity_position_fit" in ekf:
    fail("Part 1 isolation violated: GNSS velocity must not be fused into EKF yet")

# GUI must expose and log the V2 fields.
for token in ["d.resize(std::max<size_t>(45", "itow_ms", "vacc_m", "vel_e_mps", "pvt_rate_hz",
              "nav_cov_pos_valid", "gnss_fix_ok", "gnss_vel_fit", "Timestamp mode", "Position Fit"]:
    if token not in gui:
        fail(f"GUI GNSS V2 contract missing: {token}")

# auto_baud YAML knob must finally control runtime probing.
if 'declare_parameter<bool>("auto_baud", false)' not in cpp or "isAutoPort() && auto_baud_enabled_" not in cpp:
    fail("auto_baud parameter is still dead/misleading")

print("PASS gnss_driver_v2_part1_self_check")
