#!/usr/bin/env python3
"""Shared schema/math for AGV direct-serial and ROS data loggers."""
from __future__ import annotations
import math
from typing import Any, Optional, Tuple

PI = math.pi
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
G = 9.80665
NAN = float("nan")

UNIFIED_FIELDS = [
    "source_mode", "sample_seq", "wall_utc_ns", "utc_iso8601", "ros_now_ns", "monotonic_ns",
    "measurement_stamp_ns", "measurement_stamp_sec", "measurement_stamp_nanosec",
    "f411_mcu_ms", "dronecan_network_us", "dronecan_gnss_us", "dronecan_time_standard", "timestamp_source_code",
    "x_m", "y_m", "xy_source", "gnss_east_m", "gnss_north_m",
    "longitude_deg", "latitude_deg", "altitude_m", "hacc_m", "vacc_m", "fix_type", "fix_valid",
    "satellites", "satellites_visible", "gnss_ground_speed_mps", "gnss_course_deg",
    "gnss_vel_n_mps", "gnss_vel_e_mps", "gnss_vel_d_mps", "gnss_sacc_mps", "gnss_course_acc_deg",
    "gnss_pdop", "gnss_rate_hz",
    "yaw_neo3_deg", "yaw_neo3_source", "yaw_neo3_valid",
    "neo3_mag_x_ut", "neo3_mag_y_ut", "neo3_mag_z_ut", "neo3_mag_norm_ut", "neo3_node_id",
    "yaw_yahboom_deg", "yaw_yahboom_source", "yaw_yahboom_valid",
    "yah_mag_raw_x_lsb", "yah_mag_raw_y_lsb", "yah_mag_raw_z_lsb",
    "yah_mag_corrected_x", "yah_mag_corrected_y", "yahboom_corrected_norm",
    "yaw_inertial_deg", "yaw_inertial_source", "yaw_inertial_valid", "gyro_integrated_yaw_deg",
    "yaw_gnss_heading_deg", "yaw_imu_orientation_deg", "map_yaw_from_enu_deg",
    "imu_acc_raw_x", "imu_acc_raw_y", "imu_acc_raw_z",
    "imu_acc_x_mps2", "imu_acc_y_mps2", "imu_acc_z_mps2", "imu_acc_norm_mps2",
    "imu_gyro_raw_x", "imu_gyro_raw_y", "imu_gyro_raw_z",
    "imu_gyro_x_dps", "imu_gyro_y_dps", "imu_gyro_z_dps",
    "imu_gyro_x_rps", "imu_gyro_y_rps", "imu_gyro_z_rps", "gyro_z_rps",
    "imu_angle_raw_roll", "imu_angle_raw_pitch", "imu_angle_raw_yaw",
    "imu_roll_deg", "imu_pitch_deg", "imu_accgyro_yaw_deg",
    "baro_pressure_pa", "baro_temperature_k",
    "neo3_crc_ok", "yahboom_checksum_ok", "bad_neo3_crc", "bad_yahboom_checksum",
    "f4_line_count", "gnss_count", "neo3_mag_count", "imu_acc_count", "imu_gyro_count", "imu_angle_count", "imu_mag_count",
    "raw_message_count", "raw_reference",
]

def norm_angle(v: float) -> float:
    return math.atan2(math.sin(v), math.cos(v)) if math.isfinite(v) else NAN

def wrap360(v: float) -> float:
    return v % 360.0 if math.isfinite(v) else NAN

def deg(v: float) -> float:
    return math.degrees(v) if math.isfinite(v) else NAN

def finite(v: Any, default: float = NAN) -> float:
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except Exception:
        return default

def crc16_ccitt(text: str) -> int:
    crc = 0xFFFF
    for byte in text.encode("utf-8"):
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def parse_v2_payload(payload: str) -> Optional[list[str]]:
    try:
        body, version, supplied = payload.rsplit(",", 2)
        if version != "2" or int(supplied) != crc16_ccitt(body):
            return None
        return body.split(",")
    except Exception:
        return None

def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
    return ((n + alt_m) * cl * math.cos(lon), (n + alt_m) * cl * math.sin(lon), (n * (1.0 - WGS84_E2) + alt_m) * sl)

def enu_from_origin(lat_deg: float, lon_deg: float, alt_m: float, origin) -> Tuple[float, float, float]:
    lat0, lon0, _alt0, ecef0 = origin
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    dx, dy, dz = x - ecef0[0], y - ecef0[1], z - ecef0[2]
    lat, lon = math.radians(lat0), math.radians(lon0)
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    east = -so * dx + co * dy
    north = -sl * co * dx - sl * so * dy + cl * dz
    up = cl * co * dx + cl * so * dy + sl * dz
    return east, north, up

def apply_lut(yaw: float, inputs: list, corrections: list) -> float:
    if not (math.isfinite(yaw) and len(inputs) >= 2 and len(inputs) == len(corrections)):
        return norm_angle(yaw)
    two_pi = 2.0 * PI
    y = yaw % two_pi
    pts = sorted(((float(a) % two_pi, float(c)) for a, c in zip(inputs, corrections)), key=lambda p: p[0])
    ext = pts + [(pts[0][0] + two_pi, pts[0][1])]
    if y < pts[0][0]:
        y += two_pi
    for (a0, c0), (a1, c1) in zip(ext, ext[1:]):
        if a0 <= y <= a1:
            t = 0.0 if a1 == a0 else (y - a0) / (a1 - a0)
            return norm_angle(yaw + c0 + t * (c1 - c0))
    return norm_angle(yaw)
