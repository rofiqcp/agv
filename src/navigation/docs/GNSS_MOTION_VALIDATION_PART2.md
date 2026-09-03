> **HISTORICAL / SUPERSEDED (2026-08-27):** bagian arsitektur fusion di dokumen ini merekam tahap implementasi sebelumnya. Kontrak runtime final adalah **GNSS x/y + GNSS vx/vyaw, IMU absolute yaw, ESC optional untuk localization**. Gunakan `MINIPC_NAV2_NO_CAMERA_COMMISSIONING.md` sebagai authority saat deploy.

# GNSS Motion Validation Part 2

Part 2 validates GNSS motion before any new GNSS velocity/yaw fusion is enabled in `robot_localization`.

## Runtime path

`/gnss/vel` (ENU antenna Doppler velocity) is synchronized against a short history of `/odometry/filtered` using the GNSS measurement timestamp. The synchronized local yaw and yaw-rate are then used to:

1. rotate ENU velocity into the calibrated map frame;
2. compensate GNSS antenna lever-arm rotational velocity (`v_base = v_ant - omega x r`);
3. rotate map velocity into `base_footprint`;
4. compare base longitudinal velocity with local wheel/ESC velocity;
5. compare NAV-PVT COG with the Doppler vector course and, when fresh, the multi-point position-fit course.

Published diagnostics:

- `/gnss/vel_map`
- `/gnss/base_velocity`
- `/gnss/motion_validation`
- `/gnss/velocity_qualified`
- `/gnss/cog_qualified`
- `/gnss/speed_residual`
- `/gnss/course_residual`

`COG` qualification includes hold/release hysteresis and rejects near-stop, reverse, excessive yaw-rate, poor sAcc/headAcc, inconsistent vector course, position-fit disagreement, and wheel-slip state.

## Important isolation rule

Part 2 intentionally does **not** add `/gnss/base_velocity`, `/gnss/vel_map`, or COG to `ekf.yaml`. Local EKF remains ESC longitudinal velocity + IMU gyro-Z, while global EKF remains GNSS map position only. Part 3 may enable GNSS motion fusion only after field qualification.

## Field commissioning sequence

1. Verify PVT approximately 5 Hz and measurement timestamp age.
2. Drive straight forward at stable speed for at least 15–30 s.
3. Confirm `/gnss/velocity_qualified=true` is sustained.
4. Confirm body lateral velocity remains near zero.
5. Compare wheel vs GNSS base `vx`; investigate residuals above threshold.
6. Wait for valid multi-point fit and compare fit speed/course.
7. Confirm `/gnss/cog_qualified=true` only during stable forward/straight motion.
8. Repeat with left/right turns: COG qualification should drop when yaw-rate exceeds the gate.
9. Lift/spin drive wheel: wheel-slip diagnostic should assert while GNSS base velocity remains near zero.
10. Keep `enable_gnss_course_yaw_correction=false` until these tests pass.
