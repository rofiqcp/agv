# Stage 3 Implementation Report — Nav2 Ackermann Outdoor + Production Safety

**Date:** 2026-08-23  
**Baseline:** `src_mobil_stage2_gnss_imu_odometry_fusion_fixed_20260823.zip`  
**Scope:** final Nav2 Ackermann outdoor commissioning layer after Stage 1 physical geometry/odometry and Stage 2 GNSS–IMU fusion.

## 1. Goals

Stage 3 makes the autonomous command chain physically feasible for Ackermann motion, aligns Smac/MPPI/smoother/safety authority, adds a low-speed commissioning mode, and makes production autonomy fail-closed until persistent evidence exists for MPPI, trajectory safety, Collision Monitor, and fault injection.

The invariant enforced at the command boundaries is:

`|omega| <= |v| / Rmin`

where `Rmin` comes from the vehicle steering/circle-calibration authority.

## 2. Major fixes

### 2.1 Dynamic Ackermann curvature clamp in NavigationCore
Previously `NavigationCore` clamped yaw rate only against a fixed maximum. At low linear speed, a fixed yaw command can imply a turning radius smaller than the physical vehicle radius. The Stage 3 implementation computes the yaw ceiling dynamically from current command speed and `minimum_turning_radius_m` before publishing downstream.

### 2.2 Dynamic curvature clamp after TrajectorySafety slowdown
TrajectorySafety could reduce `linear.x` for obstacle/lane risk without proportionally reducing `angular.z`. That changes curvature and can make a command less physically feasible while attempting to slow down. The final output is now re-clamped against `|v|/Rmin` after all speed/risk scaling.

### 2.3 One kinematic authority across calibration and tuning
Steering/circle recalibration now propagates minimum turning radius to:
- `vehicle.yaml`
- Smac Hybrid-A* `minimum_turning_radius`
- MPPI `AckermannConstraints.min_turning_r`
- `NavigationCore.minimum_turning_radius_m`
- `TrajectorySafety.minimum_turning_radius_m`

The GUI MPPI A/B/C profile apply path also updates `vx_max`, `wz_max`, velocity-smoother limits, and trajectory-safety yaw authority together so a speed profile cannot silently violate Ackermann geometry.

### 2.4 Stage 3 production gate
`NavigationCore` now supports:
- `require_stage3_production_certification`
- `stage3_production_certified`
- `stage3_commissioning_mode`
- `stage3_commissioning_speed_cap_mps`

Production baseline is fail-closed. Stage 3 field testing requires an explicit launch argument:

`stage3_commissioning_mode:=true`

The default commissioning autonomous speed cap is `0.18 m/s`. Stage 1/2 prerequisites remain required; commissioning mode is not a bypass for physical/localization validity.

### 2.5 Persistent Stage 3 evidence
New `stage3_navigation.yaml` stores commissioning thresholds and certification state for:
- MPPI profile
- optional CLOSED_LOOP velocity smoother
- trajectory safety
- Collision Monitor
- fault injection
- final production autonomy sign-off

Evidence written by `stage3_certify.py` stores the source file path and SHA-256. `stage3_commissioning_check.py` re-hashes certified evidence. Missing or changed evidence returns a configuration error.

### 2.6 Camera / Collision Monitor runtime gate included in final sign-off
Final production certification requires both the persistent camera metric certificate and the actual `NavigationCore` runtime safety authority (`require_camera_metric_calibration`, `camera_metric_calibration_validated`, `collision_monitor_enabled`). This prevents a stale GUI certificate from being treated as sufficient if the runtime safety gate is disabled.

### 2.7 GUI/report snapshot coverage
`stage3_navigation.yaml` is added to the GUI workspace file map so configuration snapshots and reports include Stage 3 state alongside Nav2/vehicle/localization configuration.

## 3. New files

- `src/navigation/config/stage3_navigation.yaml`
- `src/navigation/docs/STAGE3_NAV2_ACKERMANN_OUTDOOR_SAFETY.md`
- `src/navigation/test/stage3_nav2_safety_self_check.py`
- `src/navigation/tools/stage3_nav2_csv_analyzer.py`
- `src/navigation/tools/stage3_commissioning_check.py`
- `src/navigation/tools/stage3_certify.py`

## 4. Modified files

- `src/navigation/CMakeLists.txt`
- `src/navigation/config/navigation_core.yaml`
- `src/navigation/config/trajectory_safety.yaml`
- `src/navigation/gui/agv_gui.cpp`
- `src/navigation/launch/autonomous.launch.py`
- `src/navigation/src/navigation_core.cpp`
- `src/navigation/src/trajectory_safety_supervisor.cpp`

## 5. Stage 3 commissioning sequence

1. Finish Stage 1 physical steering, LUT/hysteresis, drive scale, and circle geometry.
2. Finish Stage 2 stationary IMU and GNSS velocity certification; COG is required only if COG fusion is enabled.
3. Certify camera metric calibration and enable the production Collision Monitor runtime gate.
4. Run Stage 3 preflight. `WAIT` is expected before field evidence; `CONFIG ERROR` must be fixed before motion.
5. Start autonomy explicitly in Stage 3 commissioning mode (0.18 m/s default cap).
6. Run MPPI profile A, then B, and C only if needed. Record the same route/test set for comparable evidence.
7. Analyze the tuning CSV with `stage3_nav2_csv_analyzer.py`; certify the selected profile only if thresholds pass.
8. Record and mark trajectory-safety and Collision Monitor evidence.
9. Perform fault injection: GNSS disconnect/reconnect, IMU dropout, ESC/Prolific disconnect, camera dropout/stale perception, Nav2 cancel, and E-STOP.
10. Run `stage3_certify.py finalize`. It succeeds only when persistent prerequisites and evidence are valid and evidence hashes still match.
11. Restart normal autonomous launch without commissioning mode and run `stage3_commissioning_check.py --require-ready`.

## 6. Baseline state distributed in the ZIP

The final ZIP deliberately retains all real-world calibration/certification flags as `false`. This is not an implementation failure: hardware measurements cannot be fabricated. The expected pre-field state is `CONFIG SAFE, FIELD COMMISSIONING BELUM LENGKAP`.

## 7. Regression result

- Python syntax: **42 PASS**
- YAML parse: **22 PASS**
- XML/Xacro/URDF parse: **16 PASS**
- Navigation Python self-checks: **20/20 PASS**
- Standalone C++ math/safety checks: **6/6 PASS** using `-std=c++17 -Wall -Wextra -Wpedantic -Werror`
- Simulated valid evidence path reaches **STAGE-3 PRODUCTION READY**
- Tampered evidence is detected and preflight returns **CONFIG ERROR**
- Active legacy FTDI selector: absent
- Fixed `/dev/ttyUSB<N>` active device selection: absent

The audit container does not contain ROS 2/colcon/Qt5, therefore the target Jetson still must run a clean `colcon build` and `colcon test` before hardware testing.

## 8. Key field commands

```bash
cd ~/Sistem-Otomasi-Car/Car-MiniPC/ros
source /opt/ros/humble/setup.bash

ros2 run navigation stage3_commissioning_check.py --workspace "$PWD"

ros2 launch navigation autonomous.launch.py stage3_commissioning_mode:=true

ros2 run navigation stage3_nav2_csv_analyzer.py /path/to/mppi_tuning.csv

ros2 run navigation stage3_certify.py --workspace "$PWD" \
  mppi --csv /path/to/mppi_tuning.csv --profile B

ros2 run navigation stage3_certify.py --workspace "$PWD" \
  mark trajectory_safety --evidence /path/to/trajectory_safety_evidence.txt

ros2 run navigation stage3_certify.py --workspace "$PWD" \
  mark collision_monitor --evidence /path/to/collision_monitor_evidence.txt

ros2 run navigation stage3_certify.py --workspace "$PWD" \
  mark fault_injection --evidence /path/to/fault_injection_evidence.txt

ros2 run navigation stage3_certify.py --workspace "$PWD" finalize
ros2 run navigation stage3_commissioning_check.py --workspace "$PWD" --require-ready
```
