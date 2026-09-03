# DEPRECATED ARCHITECTURE NOTE

Dokumen ini merekam arsitektur sebelum final integration 2026-08-27. Jangan gunakan tabel sensor/launch di bawah sebagai konfigurasi aktif. Kontrak final: GNSS x/y + vx+vyaw, IMU absolute yaw, ESC optional untuk localization. Lihat `MINIPC_NAV2_NO_CAMERA_COMMISSIONING.md` dan `README_MINIPC_FINAL.md`.

# GNSS Global Fusion Part 3

## Scope
Part 3 connects only **qualified** GNSS motion measurements to the global `robot_localization` EKF. Raw Part-2 topics remain diagnostic-only. Local EKF remains GNSS-independent (`/esc/odom` vx + `/imu/data` gyro-Z).

## Safety defaults
Both fusion switches default to `false` because real-vehicle Part-2 qualification has not yet been executed:

- `enable_global_gnss_velocity_fusion: false`
- `enable_global_gnss_cog_fusion: false`

Enable them from the GNSS GUI only after the corresponding `velocity_qualified` / `cog_qualified` status is stable during a field test, then restart `localization_core` and `ekf_filter_node_map`.

## Data flow

```text
UBX NAV-PVT velE/N + NAV-COV
        |
        v
ENU -> map -> base + lever-arm correction @ GNSS measurement time
        |
        +--> /gnss/base_velocity              (diagnostic)
        |
Part-2 velocity qualification
        |
        v
/gnss/base_velocity_fusion  -- vx only --> Global EKF

IMU /imu/data ---------------- gyro-Z ------> Global EKF

UBX headMot/headAcc
        |
Part-2 COG qualification + hysteresis
        |
        v
/gnss/cog_heading_fusion --- absolute yaw --> Global EKF

/odometry/gnss_map ----------- x/y ---------> Global EKF
                                              |
                                              v
                                     /odometry/filtered_map
                                              |
                             gated, bounded yaw correction
                                              |
                                              v
                                          map -> odom
```

`ekf_filter_node_map.publish_tf=false` remains mandatory. `LocalizationCore` is still the only owner of `map -> odom`.

## Dedicated fusion topics
- `/gnss/base_velocity_fusion` (`TwistWithCovarianceStamped`): published only if velocity fusion is enabled and Part-2 velocity qualification passes. `ekf.yaml` fuses only body `vx`.
- `/gnss/cog_heading_fusion` (`PoseWithCovarianceStamped`, `frame_id=map`): published only if COG fusion is enabled and COG qualification passes. Only yaw is fused.
- `/gnss/velocity_fusion_active`, `/gnss/cog_fusion_active`: freshness-aware state.
- `/gnss/fusion_status`: enabled/active/global-yaw-fresh state for GUI/reporting.

## Covariance
Velocity uses the transformed NAV-COV/body covariance but clamps `vx` variance between:
- `gnss_velocity_fusion_min_variance`
- `gnss_velocity_fusion_max_variance`

COG yaw variance uses `headAcc^2`, clamped between:
- `gnss_cog_fusion_min_variance_rad2`
- `gnss_cog_fusion_max_variance_rad2`

COG is never published at rest/reverse/large yaw-rate/poor GNSS quality because Part-2 qualification blocks it.

## Temporal consistency hardening
Part 3 also fixes a remaining position-correction timing issue. The candidate `map -> odom` transform now pairs the GNSS absolute pose with **local odometry interpolated at the GNSS measurement timestamp**, rather than current odometry. This prevents GNSS/serial latency from appearing as a false translation while driving.

## Global yaw ownership
When `enable_global_gnss_cog_fusion=true`, legacy `enable_gnss_course_yaw_correction` must remain false. The code enforces a single yaw path:

`COG -> gated pose -> Global EKF (+ gyro-Z) -> bounded map->odom yaw correction`.

Global EKF yaw is only used after a COG fusion publication has actually been followed by a fresh Global EKF output. If COG becomes stale, the absolute-yaw correction stops; local EKF continues continuous dead-reckoning.

## Field commissioning order
1. Keep both Part-3 fusion switches OFF.
2. Run Part-2 straight velocity qualification; save CSV/rosbag.
3. Verify wheel-vs-GNSS residual, NAV-COV, sync gap, no unexplained lateral velocity.
4. Enable GNSS velocity fusion in GUI, restart `localization_core` + global EKF, repeat straight/circle/dropout tests.
5. Run COG straight qualification; confirm COG-vs-Doppler/position-fit residual and `headAcc` are stable.
6. Enable COG fusion, keep legacy direct COG OFF, restart localization/global EKF.
7. Test headings in multiple map directions, GNSS dropout, stationary periods, turns, and reconnect.
8. Only retain fusion if Global EKF output improves repeatability without TF jumps.

## Do not change
- Do not add GNSS to Local EKF.
- Do not fuse GNSS lateral `vy` initially; use it as slip/quality diagnostic.
- Do not fuse `headVeh` as body yaw on NEO-M9N single-antenna.
- Do not allow Global EKF to publish TF.
