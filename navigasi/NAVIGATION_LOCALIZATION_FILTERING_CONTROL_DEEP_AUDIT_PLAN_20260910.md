# AGV Navigation — Deep Audit Sensor Acquisition, Calibration, Scaling, Filtering, Localization, Nav2, and Control

**Date:** 2026-09-10  
**Target workspace:** `/home/sirobo/agv`  
**Reference repo directory:** `/home/sirobo/agv/navigasi/reference_repos/`  
**Intended report path on NUC:** `/home/sirobo/agv/navigasi/NAVIGATION_LOCALIZATION_FILTERING_CONTROL_DEEP_AUDIT_PLAN_20260910.md`

## 1. Executive verdict

The current navigation stack already has a comparatively strong fail-closed architecture: one `map->odom` owner, local and global `robot_localization` EKFs, GNSS quality gating, measurement-time handling for the F411 GNSS path, bounded map correction, explicit calibration/certification flags, Ackermann-aware MPPI, a single velocity command chain, lower-level yaw-rate feedback, and multiple static self-checks.

However, the system is **not yet precision-certified**. The main blockers are not “which Kalman filter to use,” but measurement integrity, physical calibration, and accuracy-model consistency:

1. **Absolute accuracy target mismatch:** Nav2 is configured for `xy_goal_tolerance = 0.08 m`, while autonomous GNSS admission still allows `hAcc <= 2.5 m`. A standalone NEO-M9N-class GNSS solution cannot by itself support repeatable 8 cm absolute goal accuracy. RTK GNSS, LiDAR map localization, visual-inertial localization, or another surveyed absolute reference is required if 8–10 cm is a real production requirement.
2. **Physical calibration flags remain false:** `vehicle.yaml` marks drive-odometry scale, physical steering calibration, and circle/turning-radius calibration invalid. Until these are measured, MPPI turning radius, wheel kinematics, odometry scale, and closed-loop control are not production-identifiable.
3. **Split calibration source:** `src/esc/config/ackermann.yaml` contains a provisional `drive_odometry_calibration_scale = 1.5268972865`, while `src/navigation/config/vehicle.yaml` has `drive_odometry_calibration_scale = 1.0` and `drive_odometry_calibration_valid = false`. The runtime gate is fail-closed, but the estimator can still consume a provisionally scaled `/esc/odom`. This must become one source of truth.
4. **IMU timestamp quality is weaker than GNSS timestamp quality:** the WIT IMU parser scales values correctly, but `/imu/data` is stamped with host `now()` at publish time even though accel, gyro and orientation arrive in distinct serial packets. The F411 GNSS path, in contrast, maps MCU measurement milliseconds into ROS time. Fusion precision is limited by the weakest timing contract.
5. **Filter tuning is mostly hand-configured rather than identified from data:** process noise, sensor covariance and rejection thresholds are structurally reasonable but need Allan/stationary statistics, straight/circle/figure-eight trials, innovation statistics, and bag replay evidence.
6. **Nav2/control rates and tolerances are not internally consistent for 8 cm operation:** local/global EKFs publish at 10 Hz, MPPI is 8 Hz, while the vehicle can travel roughly 12.5 cm per controller cycle at 1 m/s. The velocity smoother’s angular bounds (`±1.396 rad/s`) also exceed the vehicle’s modeled maximum yaw rate (`0.6259 rad/s`).
7. **Runtime proof is currently incomplete:** during this audit, the active ROS graph contained the gateway/ESC/Web stack, but `/imu/data`, the EKFs, `localization_core`, Nav2 and the autonomy gate were not running. Therefore this report distinguishes code/config evidence from actual runtime proof.
8. **One unrelated full-stack regression remains:** the localization-focused static self-checks passed, but `full_stack_reaudit_self_check.py` failed at the camera metric authority/perception safety contract. That should block production certification even though it is not a localization-math failure.

## 2. Scope and method

Audited code/configuration paths include at least:

- `src/navigation/src/imu_node.cpp`
- `src/navigation/src/gnss_node.cpp`
- `src/navigation/src/localization_core.cpp`
- `src/navigation/src/mag_heading_fusion_node.cpp`
- `src/navigation/src/vehicle_dynamics_observer.cpp`
- `src/navigation/src/navigation_core.cpp`
- `src/navigation/src/cmd_vel_router.cpp`
- `src/navigation/src/mppi_closed_loop_supervisor.cpp`
- `src/navigation/config/imu.yaml`
- `src/navigation/config/imu_calibration.yaml`
- `src/navigation/config/gnss.yaml`
- `src/navigation/config/ekf.yaml`
- `src/navigation/config/localization_cpp.yaml`
- `src/navigation/config/mag_heading.yaml`
- `src/navigation/config/vehicle.yaml`
- `src/navigation/config/nav2_ackermann.yaml`
- `src/navigation/config/mppi_closed_loop.yaml`
- `src/navigation/config/precision.yaml`
- `src/navigation/launch/autonomous.launch.py`
- `src/esc/src/ackermann_controller_server.cpp`
- `src/esc/config/ackermann.yaml`
- `src/stmf4/src/stmf4_hmi_bridge.cpp`
- `F4gateway/src/Neo3Sensors.cpp`
- calibration artifacts under `calibration/`
- selected navigation self-checks under `src/navigation/test/`

The audit followed the real data path rather than only reviewing algorithms in isolation:

`physical sensor -> MCU/serial parser -> raw/scaled ROS message -> timestamp/frame/covariance -> calibration/gating -> local fusion -> global fusion/map alignment -> Nav2 state -> controller -> command conditioning -> actuator -> feedback -> estimator/control diagnostics`.

## 3. Current sensor-to-control data graph

### 3.1 IMU path

`WIT/Yahboom serial -> imu_node.cpp -> /imu/data -> local EKF gyro-Z + global EKF gyro-Z + yaw-rate feedback + LocalizationCore/MagHeading diagnostics`

Additional raw/diagnostic outputs include raw sensor vectors and magnetometer data where configured.

### 3.2 GNSS path — F411 default

`NEO3/u-blox -> F4gateway Neo3Sensors.cpp -> compact SENS:GNSS/SENS:GNSSF + MCU millis -> stmf4_hmi_bridge.cpp -> /gnss/fix_raw + /gnss/fix + /gnss/vel + /gnss/quality + /gnss/state -> LocalizationCore -> gated velocity/COG + /odometry/gnss_map -> global EKF -> bounded map->odom correction`.

The launch file explicitly selects exactly one GNSS transport (`gnss_source=stm32|usb`), so the duplicate-topic risk has already been addressed architecturally at launch level.

### 3.3 GNSS path — direct USB fallback

`u-blox USB -> gnss_node.cpp -> UBX NAV-PVT/NAV-COV/NAV-DOP + NMEA fallback -> ROS measurement timestamps/covariance -> same LocalizationCore contract`.

The direct driver is richer than the compact F411 transport because it can retain exact NAV-COV and UTC/iTOW information.

### 3.4 Wheel/steering/actuator feedback path

`VESC/F103/F411 telemetry -> esc_ackermann -> calibrated drive speed + physical steering angle -> /esc/odom + /esc/drive_actual_mps + /esc/steering_actual_rad + /esc/kinematic_yaw_rate_rps`.

Local EKF intentionally fuses only longitudinal velocity from `/esc/odom`, not the kinematic odom yaw. IMU gyro-Z owns short-term yaw dynamics, which avoids directly double-counting steering-model yaw inside the EKF.

### 3.5 State estimation path

**Local EKF (`odom` world):**
- `/esc/odom`: `vx` only.
- `/imu/data`: yaw rate (`wz`) only.
- `/gnss/base_velocity_fusion`: `vx` only and only after custom qualification/certification.
- publishes `odom->base_footprint` and `/odometry/filtered`.

**Global EKF (`map` world):**
- `/odometry/gnss_map`: x/y absolute position.
- `/gnss/base_velocity_fusion`: longitudinal velocity.
- `/gnss/cog_heading_fusion`: absolute COG yaw when motion-observable.
- `/heading/validated_fusion`: independent validated heading consensus.
- `/imu/data`: yaw rate.
- `publish_tf=false`; `LocalizationCore` remains the only `map->odom` TF owner.

This TF ownership design is correct and should be preserved.

## 4. Deep audit — acquisition, raw data, scaling and calibration

### 4.1 IMU decoding/scaling

Observed decoding in `imu_node.cpp`:

- accel packet `0x51`: `int16 / 32768 * 16 g * 9.80665` -> m/s².
- gyro packet `0x52`: `int16 / 32768 * 2000 deg/s`, then converts to rad/s for ROS.
- angle packet `0x53`: `int16 / 32768 * 180 deg`, then quaternion in radians.
- quaternion packet `0x59`: supported as fallback.
- magnetometer packet `0x54`: preserved separately; Tesla publishing is disabled unless a positive calibrated scale exists.
- mounting signs and accel/gyro biases are applied before ROS publication.
- stale gyro/accel fields are marked unavailable using the `sensor_msgs/Imu` covariance convention instead of publishing false zeroes.

**Assessment:** unit scaling and unavailable-field semantics are good.

**P0 timing defect:** one composite `/imu/data` message is stamped with host publish `now()`. Because the latest accel, gyro and orientation may have different packet times, the message timestamp does not strictly describe each component measurement. This is acceptable for low-accuracy visualization, but not ideal for precise EKF/control phase alignment.

**Required change:** timestamp gyro at packet reception using a monotonic-to-ROS mapping; maintain per-field measurement time; publish the fusion IMU message on fresh gyro epochs and only include accel/orientation if their age is bounded. If the IMU exposes a hardware timestamp, use it. Never “freshen” an old gyro sample by publishing it with a new host timestamp.

### 4.2 IMU calibration

`imu.yaml` records a stationary calibration as valid, but the saved gyro-Z standard deviation is `0.0`. That may be real quantization, an insufficient sample path, or a calibration-tool bookkeeping issue. A true stochastic covariance should not be assumed from that single value.

`imu_calibration.yaml` also contains materially different legacy settings (baud, axis inversion, yaw offset, tiny covariance). It should be treated as a calibration-tool profile only and must never accidentally become a runtime configuration source.

**Required calibration evidence:**

- 20–30 min stationary bag at normal operating temperature.
- stationary mean/std and Allan deviation for gyro and accel.
- six-position accelerometer check for scale and cross-axis/mount error.
- repeat after motor electronics are powered and steering/drive actuate, to measure EMI/vibration coupling.
- temperature sweep if operating temperature changes significantly.

### 4.3 F411 GNSS measurement time and scaling

`stmf4_hmi_bridge.cpp` uses `stampFromMcuMillis()` to:

- map the F411 millisecond counter into ROS clock time,
- unwrap forward progression,
- tolerate small out-of-order packets,
- re-anchor after MCU reboot/long gaps,
- prevent future-dated measurements,
- maintain non-decreasing published stamps.

This is a strong implementation and should be retained.

F411 GNSS publication correctly converts:

- N/E/D velocity -> ROS ENU x/y/z (`E`, `N`, `-D`).
- NED course -> ENU yaw (`pi/2 - course_NED`).
- hAcc/vAcc/sAcc into message covariance estimates.

The compact frame explicitly does **not** transport full NAV-COV position covariance; it uses scalar hAcc/vAcc and sAcc instead. That is reasonable for the compact gateway but loses cross-axis covariance and richer receiver diagnostics.

**Upgrade recommendation:** extend the compact gateway protocol version to include at minimum receiver epoch id/iTOW, NAV-COV validity + horizontal covariance/correlation where available, time-accuracy/UTC-valid flags, and explicit protocol version/CRC. Do not break the existing parser without a negotiated version field.

### 4.4 Direct USB GNSS path

`gnss_node.cpp` is one of the stronger parts of the stack:

- prefers UBX NAV-PVT atomically,
- matches NAV-COV/NAV-DOP by iTOW,
- converts NED covariance to ENU,
- uses Doppler velocity directly,
- gates fix quality,
- has an NMEA fallback,
- constructs measurement timestamps from resolved UTC or an iTOW-aligned clock and prevents future dating,
- tracks measurement age and timestamp source.
**Recommendation:** use this behavior as the reference contract for the F411 path. The two transports should be semantically equivalent at `/gnss/*`, differing only in transport richness.

### 4.5 Magnetometers and heading

The system already avoids blindly using magnetic yaw as the primary local yaw source. `mag_heading_fusion` combines two independently calibrated heading sources and requires consensus; GNSS COG is used only when motion makes heading observable.

This is directionally correct. Remaining risks:

- planar hard/soft iron/LUT calibration cannot fully model 3D magnetic distortion;
- steering/traction current can create state-dependent magnetic bias;
- single-antenna GNSS COG is unobservable at standstill and weak at very low speed.

**Required production rule:** magnetic heading may seed/realign only under a validated field norm/consensus and low-current/low-dynamics condition. During motion, prefer gyro propagation plus qualified GNSS COG or a stronger absolute-yaw source. For high-precision production, dual-antenna RTK heading is preferable to relying on chassis magnetometers.

### 4.6 Drive speed scaling and odometry

`ackermann_controller_server.cpp` converts measured drive eRPM into ground speed through a fixed `drive_erpm_per_mps` baseline and a multiplicative `drive_odometry_calibration_scale`.

The current ESC config contains a provisional scale derived from one 2000 eRPM trial, while `vehicle.yaml` intentionally remains unvalidated. A single-point scale does not identify offset, direction asymmetry, speed dependence, tire deformation, slip, or repeatability.

**Required fit:** collect at least four speeds in forward and reverse over surveyed distances. Fit `v_ground = a * eRPM + b` and test whether `b` is statistically negligible. If forward/reverse or low/high-speed residuals differ materially, use a piecewise or signed model rather than forcing one multiplicative coefficient.

### 4.7 Steering scaling/geometry

`vehicle.yaml` still uses theoretical 0.7 m wheelbase and 1.597679 m minimum turning radius, with steering physical/circle calibration invalid.

MPPI’s Ackermann motion model depends directly on this radius. A wrong radius produces systematic path-tracking error even if localization were perfect.

**Required calibration:** measured physical wheel angle LUT in increasing/decreasing directions, center hysteresis, left/right circle radii, effective left/right wheelbase, steering actuator delay/time constant, and repeatability after recentering.

## 5. Deep audit — covariance, filtering and observability

### 5.1 Local EKF architecture

The local EKF is intentionally simple: wheel longitudinal speed + IMU yaw rate + certified GNSS longitudinal velocity. This is a good starting architecture for an Ackermann ground vehicle because it limits correlated inputs.

Problems to address:

- 10 Hz output is low for a 1 m/s vehicle targeting centimeter-scale final behavior.
- process noise is static and not yet supported by innovation/consistency evidence.
- `robot_localization` does not explicitly estimate IMU gyro/accel biases as states in the same way a dedicated error-state INS filter would.
- longitudinal wheel speed alone cannot constrain lateral slip; the nonholonomic assumption is implicit rather than explicitly monitored as a measurement.

**Recommendation:** first improve timing and physical calibration, then increase local estimator output to 30–50 Hz if CPU measurements support it. Do not raise rates before fixing timestamp semantics.

### 5.2 Global EKF architecture

Global EKF inputs are sensibly separated into position, velocity, absolute heading, and yaw-rate. The custom `LocalizationCore` owns gating and `map->odom` correction, which prevents the global EKF from publishing a competing transform.

Potential improvement:

- compute and expose innovations/NIS for each absolute source;
- adapt GNSS measurement covariance based on receiver quality and consistency, not only static clamps;
- prevent a re-acquired noisy GNSS fix from pulling `map->odom` before a stable hold window;
- retain bounded correction rate and stationary-translation freeze.

### 5.3 Delayed measurements

GNSS is delayed relative to IMU and wheel sensing. `LocalizationCore` already keeps local history and interpolates the local state at GNSS measurement time for lever-arm handling—this is good.

For higher precision, copy the stronger design pattern seen in modern ESKF implementations: rewind/replay or fixed-lag smoothing for delayed GNSS, with explicit sensor time, arrival time, filter time and apply time diagnostics. This is especially valuable when gateway/USB scheduling latency varies.

### 5.4 Bias observability

A dedicated error-state filter that estimates gyro/accel bias can outperform static bias subtraction during long missions, but bias states must be observable. The system should not “learn” bias during unexcited motion just because a Kalman model allows it.

Adopt an observability gate: bias updates only when excitation and aiding geometry are sufficient. At standstill, use ZUPT/zero-innovation heading-rate constraints; during straight motion, use wheel speed + nonholonomic constraints cautiously; during turns, use heading/position aids to make gyro bias observable.

### 5.5 Slip detection

The current stack already compares wheel/GNSS speed and kinematic/IMU yaw rate, and can diagnose wheel slip while keeping high-gain map correction disabled by default. Preserve this conservative behavior.

Future adaptive covariance logic should inflate wheel-velocity variance during slip rather than simply continuing with the nominal wheel covariance.

## 6. Localization precision ceiling

### Current standalone-GNSS tier

With strict `hAcc <= 2.5 m`, the system can be made robust for meter-class global localization and much better short-term relative odometry, but **not** repeatedly certify an 8 cm absolute endpoint.

### Precision tier options

1. **RTK GNSS:** upgrade to an RTK-capable receiver, use fixed/float status, baseline corrections, and ideally dual-antenna heading. Keep wheel/IMU fusion for outages and high-rate motion.
2. **LiDAR map localization:** use KISS-ICP/NDT/GICP/slam_toolbox/Autoware-style scan matching to a surveyed point-cloud/2D map, fused with IMU/wheel. GNSS becomes global initialization/recovery rather than the centimeter-level measurement.
3. **Visual-inertial:** useful where camera geometry/lighting is reliable; requires temporal and extrinsic calibration. Treat it as another local odometry source and retain independent global anchoring.
4. **Hybrid:** RTK + LiDAR/visual + wheel/IMU offers the strongest failover and observability but also the highest integration cost.

**Recommendation for this AGV:** if repeatable outdoor 8–10 cm absolute goal accuracy is mandatory, prioritize RTK GNSS and/or LiDAR map localization before spending time on finer EKF matrix tuning.

## 7. Nav2 audit

### Good current decisions

- Ackermann motion model in MPPI.
- Smac Hybrid planning.
- a single command path through router -> velocity smoother -> `/cmd_vel` -> Ackermann actuator.
- fail-closed motion gate and independent teleop behavior.
- explicit trajectory/collision safety layers.

### Required corrections

1. **Turning radius:** do not certify MPPI until circle calibration is valid.
2. **Goal tolerance vs localization:** `0.08 m` should be used only when the absolute localizer can demonstrate better than this at p95. Otherwise expose a localization-aware goal tolerance or refuse precision mode.
3. **Costmap resolution:** 0.1 m cells are coarse relative to an 0.08 m target. For precision mode, 0.05 m may be more appropriate if CPU/memory allow and the map/sensor resolution warrants it.
4. **Rate:** 8 Hz controller at 1 m/s means 0.125 m travel per nominal cycle. After local state is 30–50 Hz and CPU is verified, evaluate MPPI at 15–20 Hz.
5. **Velocity smoother limit mismatch:** clamp angular speed to the same vehicle SSOT limit used by MPPI/Ackermann, unless a documented reason requires a wider intermediate bound.
6. **Closed-loop smoother:** keep `OPEN_LOOP` until feedback qualification passes; then evaluate closed-loop smoothing using measured `/odometry/filtered`, with stale-feedback failover.

## 8. Control audit

### Current positive structure

- actuator loop runs faster than planner/controller;
- measured yaw-rate feedback can correct steering-model error;
- command watchdogs/timeouts are present;
- teleop/autonomous ownership is explicit.

### Required model identification

Measure:

- steering command -> physical wheel-angle static map,
- steering delay and first-order time constant,
- yaw-rate response vs speed and steer,
- drive command -> eRPM -> ground-speed response,
- braking/deceleration response,
- deadband and sign asymmetry,
- center hysteresis and return-to-center bias.

Then tune yaw-rate feedforward + PI against these data. Anti-windup and stale-IMU behavior must be tested with abrupt command changes and sensor dropout.

## 9. Runtime/static evidence collected during this audit

### Runtime graph observed

Active nodes at audit time included:
- `/agv_web_gui`
- `/esc_ackermann`
- `/rt_cert_probe`
- `/stmf4_hmi_bridge`
- `/vesc_tool_bridge`

At that moment:
- `/gnss/fix_raw`: 1 publisher, `stmf4_hmi_bridge`.
- `/gnss/vel`: 1 publisher, `stmf4_hmi_bridge`.
- `/esc/odom`: 1 publisher, `esc_ackermann`.
- `/imu/data`: 0 publishers.
- `/odometry/filtered`: 0 publishers.
- `/odometry/filtered_map`: 0 publishers.
- `/gnss/base_velocity_fusion`: 0 publishers.
- `/gnss/cog_heading_fusion`: 0 publishers.
- `/cmd_vel_nav_raw`: 0 publishers.
- `/system/autonomy_motion_allowed`: 0 publishers.

Therefore no end-to-end localization accuracy claim is made from this runtime snapshot.

### Static self-checks that passed

- `ekf_humble_config_self_check.py`
- `ekf_launch_validation_self_check.py`
- `ekf_nav2_gate_chain_self_check.py`
- `gnss_driver_v2_part1_self_check.py`
- `gnss_global_fusion_part3_self_check.py`
- `gnss_motion_math_self_check.py`
- `gnss_motion_validation_part2_self_check.py`
- `f411_mag_pipeline_self_check.py`
- `nav2_velocity_chain_self_check.py`
- `precision_part1_self_check.py`
- `precision_part2_self_check.py`
- `precision_part3_self_check.py`
- `stage2_localization_foundation_self_check.py`
- `stage3_nav2_safety_self_check.py`

One broader test failed:
- `full_stack_reaudit_self_check.py`: camera metric authority/perception safety contract mismatch. Treat as an integration blocker before production certification.

## 10. 30 reference repositories cloned and what to adopt

All repositories below were URL-validated and shallow-cloned into `/home/sirobo/agv/navigasi/reference_repos/`. The listed commit is the checked-out shallow HEAD during the audit.

| # | Repository / clone folder | HEAD date / commit | Primary lesson for AGV |
|---|---|---|---|
| 1 | `ros-navigation/navigation2` / `01_navigation2` | 2026-09-09 `5e771b9f931c` | MPPI, Smac Hybrid, lifecycle, velocity smoothing, collision-monitor contracts, controller diagnostics. |
| 2 | `cra-ros-pkg/robot_localization` / `02_robot_localization` | 2026-07-16 `7dfb6aa97b20` | EKF/UKF configuration, rejection thresholds, dual-EKF GPS patterns, transform ownership. |
| 3 | `SteveMacenski/slam_toolbox` / `03_slam_toolbox` | 2026-08-17 `60fba35938fc` | 2D map localization/relocalization and Nav2 integration for LiDAR path. |
| 4 | `ros-controls/ros2_control` / `04_ros2_control` | 2026-09-08 `85f1d7669e3f` | deterministic hardware/controller boundaries, lifecycle, realtime-safe state/command interfaces. |
| 5 | `ros-controls/ros2_controllers` / `05_ros2_controllers` | 2026-09-02 `119c9a50a716` | Ackermann steering controller geometry, generated parameter validation, controller feedback patterns. |
| 6 | `ros-teleop/twist_mux` / `06_twist_mux` | 2026-07-09 `a8225e2d9bf0` | command priority/locks and explicit authority arbitration. |
| 7 | `f1tenth/f1tenth_system` / `07_f1tenth_system` | 2024-02-16 `ae64e05fbaf6` | small Ackermann vehicle system integration, deadman and sensor/drive bringup. |
| 8 | `f1tenth/vesc` / `08_f1tenth_vesc` | 2023-02-19 `e3f3084408f4` | VESC-to-odometry and Ackermann-to-VESC mapping reference. |
| 9 | `autowarefoundation/autoware_universe` / `09_autoware_universe` | 2026-09-09 `ffd562e35dcf` | production-grade localization/control decomposition, diagnostics, pose/twist fusion, vehicle state checks. |
| 10 | `locusrobotics/fuse` / `10_fuse` | 2026-06-18 `8e3f0a1fef6f` | factor-graph sensor fusion, timestamp manager, model/plugin separation, covariance handling. |
| 11 | `CCNYRoboticsLab/imu_tools` / `11_imu_tools` | 2026-09-09 `5efc1bef22d7` | Madgwick/complementary IMU filtering and ROS IMU conventions. |
| 12 | `ros-drivers/nmea_navsat_driver` / `12_nmea_navsat_driver` | 2022-11-20 `b168f499349e` | NMEA checksum/parser contracts and NavSat/Twist message semantics. |
| 13 | `KumarRobotics/ublox` / `13_ublox` | 2022-12-06 `4f107f3b8213` | u-blox receiver configuration and UBX message handling. |
| 14 | `septentrio-gnss/septentrio_gnss_driver` / `14_septentrio_gnss_driver` | 2026-08-01 `5613af2969e3` | modern GNSS/INS driver time handling, covariance, RTK status, robust communication. |
| 15 | `LORD-MicroStrain/microstrain_inertial` / `15_microstrain_inertial` | 2026-05-21 `ccc6cd5632f8` | high-quality IMU/GNSS/INS ROS publishing, diagnostics, device timestamping/configuration. |
| 16 | `ethz-asl/kalibr` / `16_kalibr` | 2024-03-08 `1f60227442d2` | temporal/extrinsic calibration methodology; useful if camera/VIO or multi-IMU is adopted. |
| 17 | `rpng/open_vins` / `17_open_vins` | 2025-11-30 `69488123ed93` | MSCKF/EKF visual-inertial design, covariance management, temporal calibration. |
| 18 | `HKUST-Aerial-Robotics/VINS-Fusion` / `18_vins_fusion` | 2021-07-26 `be55a937a574` | multi-sensor VIO and online time-offset/extrinsic calibration reference; legacy ROS caveat. |
| 19 | `TixiaoShan/LIO-SAM` / `19_lio_sam` | 2023-04-17 `0be1fbe6275f` | IMU preintegration + LiDAR factor graph + GNSS aiding concepts. |
| 20 | `hku-mars/FAST_LIO` / `20_fast_lio` | 2024-07-23 `7cc4175de6f8` | tightly-coupled iterated Kalman LiDAR-inertial odometry. |
| 21 | `PRBonn/kiss-icp` / `21_kiss_icp` | 2026-05-04 `1ffa7d7512f1` | simple robust LiDAR odometry, adaptive registration design, ROS2 support. |
| 22 | `koide3/small_gicp` / `22_small_gicp` | 2026-08-31 `2c6c7831bc1f` | efficient GICP/VGICP, rejectors, registration benchmarking. |
| 23 | `rsasaki0109/lidar_localization_ros2` / `23_lidar_localization_ros2` | 2026-08-26 `674e51390130` | ROS2 map-based NDT/GICP localization, recovery and repeatable bag evaluation. |
| 24 | `rsasaki0109/kalman_filter_localization_ros2` / `24_kalman_filter_localization_ros2` | 2026-08-08 `0a456b24f906` | highly relevant ESKF: IMU bias states, GNSS NIS gating, robust loss, lever arm, delayed rewind/replay, wheel speed/NHC/ZUPT/observability tests. |
| 25 | `cartographer-project/cartographer_ros` / `25_cartographer_ros` | 2022-10-27 `c138034db0c4` | legacy reference for pose extrapolation, time synchronization and localization mode; do not adopt as primary new stack. |
| 26 | `borglab/gtsam` / `26_gtsam` | 2026-09-09 `067ca8d0c55f` | factor graphs, IMU preintegration, incremental smoothing, robust optimization. |
| 27 | `PX4/PX4-Autopilot` / `27_px4_autopilot` | 2026-09-09 `e53ff6b3ffc3` | mature estimator aiding/innovation status, delay handling, multi-sensor health gating and rover control concepts. |
| 28 | `ArduPilot/ardupilot` / `28_ardupilot` | 2026-09-09 `4891432f35c3` | EKF3 innovation gates, sensor delays, bias process noise, GPS/mag fault checks, rover control/failsafes. |
| 29 | `koide3/glim` / `29_glim` | 2026-09-06 `2262aafa2369` | modern LiDAR/IMU factor-graph odometry/mapping and clean `map->odom->base` conventions. |
| 30 | `tier4/ndt_omp` / `30_ndt_omp_ros2` | 2026-01-08 `025f263f41b7` | ROS2-compatible accelerated NDT and covariance tools used in production-style scan matching. |

### Highest-value patterns to transplant first

Do **not** copy whole frameworks. Transplant design patterns in this order:

1. from `kalman_filter_localization_ros2`: sensor-time/arrival-time diagnostics, delayed measurement rewind/replay, bias-state/observability thinking, NIS gates, robust loss, NHC/ZUPT patterns;
2. from PX4/ArduPilot: per-aiding-source innovation health, delayed measurement model, failover and bias/noise management;
3. from Nav2/ros2_controllers: single parameter source for Ackermann geometry and controller constraints;
4. from Autoware: production diagnostics, localization quality monitors and vehicle model consistency;
5. from septentrio/microstrain drivers: timestamp/covariance/device-health semantics;
6. if LiDAR exists: KISS-ICP or NDT/GICP map localization as an independent absolute/relative localizer;
7. use GTSAM/fuse only if the problem has outgrown the dual-EKF architecture—avoid unnecessary complexity before calibration/timing is fixed.

## 11. Prioritized implementation roadmap

### P0 — measurement integrity and single-source-of-truth

**P0.1 IMU measurement timestamp contract**
- add per-packet measurement timestamps;
- publish fusion IMU on fresh gyro epoch;
- expose `sensor_time`, `arrival_time`, `publish_time`, and age diagnostics;
- reject regressing/future measurements;
- add serial jitter histogram.

**P0.2 Unify vehicle calibration SSOT**
- eliminate independent drive scale in ESC vs `vehicle.yaml`;
- load/generate Ackermann parameters from one certified vehicle calibration file;
- add startup assertion: if a runtime value differs from the certified SSOT beyond tolerance, autonomous mode must stay false.

**P0.3 Normalize motion limits**
- one source for max steering, min turning radius, max speed, max yaw rate, accel/decel/yaw accel;
- inject the same values into MPPI, smoother, ESC and safety supervisors;
- reject launch on inconsistency.

**P0.4 Sensor publisher ownership check**
- startup self-check must assert exactly one publisher for each canonical raw sensor topic before arming autonomous motion.

**P0.5 F411 GNSS protocol versioning**
- add protocol version + CRC/sequence if not already available at transport level;
- transport richer timing/covariance validity fields where bandwidth permits.

### P1 — physical calibration campaign

**P1.1 IMU**
- stationary 20–30 min;
- Allan analysis;
- six-position accel;
- motor-on/motor-off noise comparison;
- save raw bag + derived metrics + calibration hash.

**P1.2 Drive odometry**
- surveyed 10/25/50 m lines;
- >=4 command speeds each forward/reverse;
- fit slope/intercept, residual p95, direction asymmetry;
- certify only if distance error and repeatability gates pass.

**P1.3 Steering**
- increasing/decreasing LUT;
- center repeatability;
- measured left/right physical limits;
- circle tests in both directions;
- estimate effective wheelbase/turn radius and uncertainty.

**P1.4 GNSS lever arm/map alignment**
- survey antenna reference relative to `base_footprint`;
- capture 3+ well-spread surveyed map points;
- solve 2D rigid transform and report RMSE/geometry score.

**P1.5 Magnetic heading**
- repeat 8-direction calibration with propulsion off, steering active, drive active;
- create current/dynamics rejection rule;
- certify heading only where residual p95 meets threshold.

### P2 — estimator upgrade and tuning

**P2.1 Increase local estimator rate**
- after timestamp fix, test 30 Hz then 50 Hz local EKF output;
- measure CPU, callback latency, jitter and missed deadlines.

**P2.2 Data-driven covariance**
- derive IMU covariance/process noise from stationary/Allan data;
- derive wheel speed noise from repeated straight runs;
- derive steering/yaw model noise from circle/figure-eight residuals;
- preserve GNSS receiver-provided covariance where available.

**P2.3 Innovation/NIS monitor**
- publish innovation, normalized innovation squared, gate decision and covariance for GNSS pos/vel/yaw and wheel-vs-GNSS consistency;
- use chi-square-informed thresholds rather than arbitrary residuals alone.

**P2.4 Delayed measurement replay**
- evaluate fixed-lag/rewind-replay for GNSS based on repo 24 pattern;
- only adopt if replay tests show improvement over current interpolation/bounded-map-correction design.

**P2.5 Bias-aware ESKF decision gate**
- compare current dual `robot_localization` architecture against a dedicated planar ESKF with gyro bias state;
- migrate only if bag evidence shows materially lower outage drift/yaw error without robustness regression.

### P3 — absolute localization precision upgrade

Choose based on hardware:

- **RTK path:** RTK-capable receiver + corrections; fuse fixed solution, Doppler velocity and dual-antenna heading if available.
- **LiDAR path:** KISS-ICP/local odometry + NDT/GICP or slam_toolbox map localization; fuse with wheel/IMU and retain GNSS for global recovery.
- **VIO path:** Kalibr time/extrinsic calibration + OpenVINS-like VIO; retain independent global anchor.

Do not claim 8 cm global accuracy until ground-truth testing demonstrates it.

### P4 — Nav2/control retuning

- update min turning radius from measured circle calibration;
- align velocity smoother angular limits with vehicle limits;
- increase MPPI rate to 15–20 Hz only after estimator rate/CPU are ready;
- consider 0.05 m costmap resolution for precision mode;
- tune steering delay/tau in the dynamics observer;
- identify yaw-rate feedforward + PI gains from step/slalom data;
- validate closed-loop smoother only after odometry feedback qualification.

### P5 — resilience and fault injection

Automated tests must include:

- IMU disconnect/reconnect;
- GNSS dropout/reacquisition;
- F411 reboot and MCU millis re-anchor;
- out-of-order/delayed GNSS packet;
- corrupted/partial serial frame;
- static GNSS jump;
- magnetic disturbance during drive/steering current;
- wheel slip;
- stale `/odometry/filtered`;
- duplicate publisher injection;
- Nav2 lifecycle restart;
- Web/HMI restart while motion remains safely governed.

## 12. Required rosbag/ground-truth test matrix

| Test | Minimum data | Primary metrics |
|---|---|---|
| stationary 20–30 min | IMU, GNSS, all clocks | bias, Allan/noise, drift, timestamp jitter |
| straight 10/25/50 m | eRPM, wheel/steer, IMU, GNSS, EKF | distance scale, yaw drift, cross-track |
| reverse straight | same | signed scale/asymmetry |
| left/right circles | steering, IMU wz, GNSS/GT pose | radius fit, wheelbase, yaw-rate residual |
| figure-eight | all | bias observability, heading consistency |
| slalom | all + commands | delay, control bandwidth, tracking residual |
| slow precision approach | Nav2 command/state + GT | stopping error, overshoot, final yaw |
| GNSS outage | all | local drift per meter/time, recovery overshoot/settling |
| GNSS jump/outlier | all | innovation gate, max pose jump |
| IMU dropout | all | safe degradation, actuation gate response |
| F411 reboot | GNSS gateway clocks | timestamp monotonicity, reconnection latency |
| magnetic interference | mag + currents + heading refs | heading error/current correlation |

Ground truth should be RTK-fixed surveyed points, total station, motion capture, or accurately surveyed distance/heading fixtures. Do not tune against the same noisy GNSS source being evaluated.

## 13. Suggested acceptance gates

Use tiers rather than one unrealistic threshold.

### Tier 1 — robust meter-class autonomous navigation
- strict GNSS quality passes and does not flap;
- no TF discontinuity > configured bounded correction;
- local odometry straight-line distance error <= 2% p95;
- circle radius model error <= 5% p95;
- yaw-rate residual <= 0.08 rad/s p95 in normal turns;
- no autonomous motion with stale/missing critical sensor;
- timestamp age/jitter thresholds pass.

### Tier 2 — calibrated decimeter local tracking
- straight distance error <= 1% p95;
- circle radius error <= 3% p95;
- local relative pose RPE <= 0.10 m over representative short horizons;
- final local tracking error <= 0.15 m p95 with an adequate absolute reference.

### Tier 3 — precision global mode
Only with RTK/LiDAR/VIO-grade absolute localization:
- horizontal absolute pose error <= 0.05 m RMS and <= 0.10 m p95, or a project-specific tighter surveyed gate;
- final position error <= 0.08–0.10 m p95;
- yaw error <= 2 deg p95;
- repeatable across multiple days/starts and after GNSS/localizer reacquisition.

The current M9N-style standalone GNSS path should not be certified against Tier 3.

## 14. Concrete code-change checklist

### Measurement layer
- [ ] IMU packet-time timestamping.
- [ ] per-field sample age diagnostics.
- [ ] sensor source/sequence diagnostics.
- [ ] F411 GNSS protocol version/sequence/CRC review.
- [ ] exact semantic parity between STM32 and USB GNSS topics.

### Calibration/scaling layer
- [ ] one vehicle calibration SSOT.
- [ ] multi-speed drive scale fit.
- [ ] steering hysteresis LUT.
- [ ] left/right circle radius fit.
- [ ] verified GNSS lever arm.
- [ ] magnetic current-interference characterization.
- [ ] Allan/noise-derived IMU covariance.

### Fusion layer
- [ ] local EKF 30/50 Hz experiment after timestamp fix.
- [ ] innovation/NIS telemetry.
- [ ] wheel covariance inflation on slip.
- [ ] delayed GNSS rewind/replay A/B test.
- [ ] bias-aware ESKF A/B evaluation.
- [ ] startup/reacquisition hold tests.

### Nav2/control layer
- [ ] measured radius into planner/controller.
- [ ] uniform max yaw-rate across smoother/controller/ESC.
- [ ] precision-mode resolution/tolerance consistency.
- [ ] MPPI 15–20 Hz A/B test.
- [ ] steering delay/tau identification.
- [ ] yaw-rate PI retune with anti-windup evidence.
- [ ] closed-loop smoother qualification.

### Validation layer
- [ ] deterministic rosbag replay runner.
- [ ] metrics JSON/CSV/Markdown generator.
- [ ] fault-injection suite.
- [ ] ground-truth acceptance report.
- [ ] production certification manifest containing commit hashes + calibration hashes + parameter hashes.

## 15. Recommended final architecture

For the current hardware baseline:

`WIT IMU(packet-time, calibrated) + drive eRPM/steering(calibrated) -> high-rate local estimator -> odom->base`

`GNSS measurement-time + qualified COG/validated heading -> global estimator / bounded map correction -> map->odom`

`Nav2 MPPI(Ackermann parameters from certified vehicle SSOT) -> authority/safety router -> velocity smoother -> actuator controller -> yaw-rate feedback -> VESC/F103`

For true centimeter/decimeter absolute operation, insert:

`RTK and/or LiDAR map localizer -> robust global absolute pose aid`,

while retaining GNSS, IMU and wheel sensors as independent aids/fallbacks.

## 16. Do-not-do list

- Do not tune EKF process noise to “hide” bad timestamping.
- Do not reduce covariance merely to make the path look smoother.
- Do not set calibration-valid flags manually without raw trial evidence.
- Do not copy PX4/ArduPilot/Autoware EKF code wholesale into ROS; adopt their health/delay/innovation design patterns.
- Do not fuse multiple correlated representations of the same measurement without understanding correlation.
- Do not use magnetometer yaw continuously near motor current without an interference gate.
- Do not claim 8 cm absolute localization using a 2.5 m admitted GNSS solution.
- Do not raise MPPI rate until estimator timing and CPU jitter are measured.

## 17. Final priority order

1. Fix IMU measurement timing semantics.
2. Unify vehicle calibration SSOT and remove provisional split-brain scaling.
3. Complete real drive + steering + circle calibration.
4. Build bag/ground-truth metrics and innovation telemetry.
5. Retune covariance/filter rates from evidence.
6. Decide RTK/LiDAR/VIO path for the actual absolute-accuracy target.
7. Retune MPPI/smoother/yaw control using calibrated dynamics.
8. Complete dropout/reboot/fault-injection certification.
9. Resolve the unrelated perception full-stack self-check failure before production sign-off.

---

## External reference URLs

- https://github.com/ros-navigation/navigation2
- https://github.com/cra-ros-pkg/robot_localization
- https://github.com/SteveMacenski/slam_toolbox
- https://github.com/ros-controls/ros2_control
- https://github.com/ros-controls/ros2_controllers
- https://github.com/ros-teleop/twist_mux
- https://github.com/f1tenth/f1tenth_system
- https://github.com/f1tenth/vesc
- https://github.com/autowarefoundation/autoware_universe
- https://github.com/locusrobotics/fuse
- https://github.com/CCNYRoboticsLab/imu_tools
- https://github.com/ros-drivers/nmea_navsat_driver
- https://github.com/KumarRobotics/ublox
- https://github.com/septentrio-gnss/septentrio_gnss_driver
- https://github.com/LORD-MicroStrain/microstrain_inertial
- https://github.com/ethz-asl/kalibr
- https://github.com/rpng/open_vins
- https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
- https://github.com/TixiaoShan/LIO-SAM
- https://github.com/hku-mars/FAST_LIO
- https://github.com/PRBonn/kiss-icp
- https://github.com/koide3/small_gicp
- https://github.com/rsasaki0109/lidar_localization_ros2
- https://github.com/rsasaki0109/kalman_filter_localization_ros2
- https://github.com/cartographer-project/cartographer_ros
- https://github.com/borglab/gtsam
- https://github.com/PX4/PX4-Autopilot
- https://github.com/ArduPilot/ardupilot
- https://github.com/koide3/glim
- https://github.com/tier4/ndt_omp