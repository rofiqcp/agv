> **HISTORICAL / SUPERSEDED (2026-08-27):** bagian arsitektur fusion di dokumen ini merekam tahap implementasi sebelumnya. Kontrak runtime final adalah **GNSS x/y + GNSS vx/vyaw, IMU absolute yaw, ESC optional untuk localization**. Gunakan `MINIPC_NAV2_NO_CAMERA_COMMISSIONING.md` sebagai authority saat deploy.

# GNSS Driver V2 — Part 1

Part 1 hardens the u-blox NEO-M9N/CUAV GNSS input before any new GNSS motion measurement is fused into the EKF.

## Implemented

- UBX NAV-PVT parsing expanded to iTOW, UTC validity/time accuracy, ellipsoid/MSL height, hAcc/vAcc, velN/E/D, flags/flags2/flags3, gSpeed, headMot/headVeh validity, sAcc/headAcc, pDOP, differential/carrier-solution state, correction age, authenticated-time flag, and magnetic-declination diagnostics.
- Receiver navigation rate is configured in RAM with `CFG-RATE-MEAS` / `CFG-RATE-NAV`; default target is 5 Hz.
- NAV-COV and NAV-DOP are polled and matched to NAV-PVT using iTOW.
- NavSatFix and GNSS velocity use the NAV-COV matrix when a matching valid epoch is available. NED covariance is transformed to ENU.
- `/gnss/vel` now uses receiver-provided velE/velN/velD directly. gSpeed/headMot remain independent cross-check values.
- Duplicate and out-of-order iTOW epochs are rejected from publication and counted.
- Measurement timestamp mode supports UBX UTC when fully resolved and the ROS wall-time numeric epoch is consistent, then falls back to an iTOW-aligned ROS timeline. The iTOW fallback is clamped against future timestamps to avoid future-TF faults; arrival-time mode remains available for troubleshooting.
- A weighted multi-point position regression publishes diagnostic `/gnss/velocity_position_fit` only after the baseline is large relative to hAcc. It is deliberately **not fused into EKF** in Part 1.
- `/gnss/quality` remains append-only: indices 0..8 are unchanged; Driver V2 fields are appended.
- GUI shows actual PVT rate, iTOW, timestamp source/age, velE/N/D, NAV-COV validity, extended DOP and position-fit residuals. CSV runs include the same fields.
- `auto_baud` is now a real runtime parameter instead of a dead YAML key.

## Safety boundary

Part 1 does not add `/gnss/vel` or COG to `robot_localization`. Local EKF remains ESC longitudinal velocity + IMU gyro-Z, and global EKF remains GNSS position only. Motion fusion is intentionally deferred until the new measurements are field-qualified.

## Field acceptance checks

1. NAV-PVT actual rate should settle near the configured 5 Hz.
2. iTOW must be monotonic with no persistent duplicate/out-of-order counters.
3. `gSpeed` should approximately match `hypot(velE, velN)` during straight motion.
4. `headMot` converted to ENU should agree with `atan2(velN, velE)` when speed/headAcc are valid.
5. NAV-COV position/velocity flags should appear valid in open sky; fallback hAcc/sAcc covariance remains available otherwise.
6. Position-fit velocity should only become valid after a sufficiently long baseline and is expected to be noisier than Doppler velocity on M9N at low speed.

- NAV-PVT `flags3.invalidLlh` is now an explicit autonomous-quality rejection condition even when the numeric latitude/longitude fields look finite.
