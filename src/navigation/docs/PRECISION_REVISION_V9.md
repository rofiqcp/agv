> **HISTORICAL / SUPERSEDED (2026-08-27):** bagian arsitektur fusion di dokumen ini merekam tahap implementasi sebelumnya. Kontrak runtime final adalah **GNSS x/y + GNSS vx/vyaw, IMU absolute yaw, ESC optional untuk localization**. Gunakan `MINIPC_NAV2_NO_CAMERA_COMMISSIONING.md` sebagai authority saat deploy.

# Precision Revision V9 — 2026-08-22

## Applied
- Local EKF now fuses ESC longitudinal velocity with real IMU gyro-Z; ESC kinematic yaw-rate is no longer double-counted as a measurement.
- MPPI visualization defaults OFF for deployment efficiency.
- Goal checker defaults tightened to 0.75 m / 30 deg for M9N commissioning and remains GUI-tunable.
- `vehicle.yaml` is exposed in GUI as the authoritative vehicle-geometry calibration surface.
- Navigation safety/calibration YAML is no longer silently overridden by `autonomous.launch.py`.
- Every diagnostic/tuning CSV now receives a `.meta.json` sidecar containing exact YAML values and SHA-256 hashes for reproducible thesis experiments.
- GUI exposes IMU-EKF settings, vehicle geometry, goal precision, MPPI visualization, velocity smoother feedback mode, and report session ID.

## Physical validation still required
1. Steering LEFT/CENTER/RIGHT capture must be performed on the real vehicle.
2. Gyro sign must be checked against a known left/right turn before autonomous driving.
3. GNSS stationary scatter determines final goal tolerance; do not reduce below measured GPS capability.
4. CLOSED_LOOP velocity smoother should only be enabled after odometry latency/rate validation.
