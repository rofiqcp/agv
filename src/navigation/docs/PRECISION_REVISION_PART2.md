> **HISTORICAL / SUPERSEDED (2026-08-27):** bagian arsitektur fusion di dokumen ini merekam tahap implementasi sebelumnya. Kontrak runtime final adalah **GNSS x/y + GNSS vx/vyaw, IMU absolute yaw, ESC optional untuk localization**. Gunakan `MINIPC_NAV2_NO_CAMERA_COMMISSIONING.md` sebagai authority saat deploy.

# Precision Revision — Part 2

Part 2 concentrates on measurement quality and localization precision after the Part 1 configuration/freshness fixes.

## Implemented

- Data-driven IMU stationary calibration wizard: gyro bias, accelerometer bias, and covariance recommendations are computed from fresh ROS samples and can be applied to YAML. Repeated calibration adds measured residuals to the existing correction instead of accidentally replacing the previous bias.
- GNSS stationary qualification wizard: RMS, CEP50, CEP95, maximum radius, drift, hAcc, DOP, and satellite statistics are recorded to CSV. A separate straight-run COG qualification mode measures gated epochs, course accuracy, course-vs-local-yaw bias, and centered P95 residual.
- UBX NAV-PVT `headMot` and `headAcc` are exposed as ENU course and course uncertainty. Single-antenna COG yaw correction is implemented but **disabled by default** until field qualification.
- COG correction is gated by forward motion, GNSS speed accuracy, heading accuracy, low yaw-rate, GNSS quality, innovation, and a bounded correction step.
- GNSS antenna lever-arm X/Y is used by LocalizationCore when converting antenna position to `base_footprint` position.
- Multi-point map calibration now validates baseline, 2-D geometry score, and RMSE before accepting a solution.
- ESC publishes explicit `/esc/kinematic_yaw_rate_rps`. This is model-derived from measured speed plus calibrated steering and is diagnostic-only. Local EKF yaw remains IMU gyro-Z.
- Wheel-slip/mismatch diagnostics compare fresh Ackermann kinematic yaw with IMU gyro instead of comparing the IMU-driven EKF back to the same IMU.
- ESC odometry covariance adapts to drive tracking error and steering magnitude.
- GUI report CSV metadata now includes session ID and reloads YAML before hashing/snapshotting.
- GUI has a Config-vs-Runtime parameter audit using ROS parameter services.
- `AGV_CONFIG_DIR` is supported by GUI and navigation launch files so source/installed YAML divergence can be avoided.

## Safe commissioning order

1. Complete steering calibration from Part 1.
2. Run IMU stationary capture for 30–60 s and apply the resulting bias/covariance.
3. Restart IMU and EKF; use **Verify Runtime** and require CONFIG == RUNTIME.
4. Run straight and left/right arc tests while observing gyro-Z, Ackermann yaw-rate, and their residual.
5. Measure the physical GNSS antenna X/Y offset from `base_footprint` and save it.
6. Run GNSS stationary capture in open sky for at least 60 s.
7. Capture multiple map calibration points with a wide 2-D spatial distribution; do not use nearly collinear points.
8. Leave COG yaw correction OFF initially. Qualify it using forward, mostly straight runs; enable only if course accuracy and residual plots are stable at the actual operating speed.

## Important limitation

The revision is statically validated in this workspace. Final bias, covariance, antenna offsets, GNSS quality thresholds, and COG suitability must be measured on the physical AGV. Part 2 intentionally does not invent calibration values.
