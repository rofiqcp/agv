# Stage 1 — Hardware Identity, Physical Steering, and Odometry Foundation

Stage 1 is deliberately **fail-closed**. Nav2 commands may be generated for diagnostics,
but `NavigationCore` will not open `/system/autonomy_motion_allowed` until three physical
calibration gates stored in `vehicle.yaml` are valid:

1. `steering_calibration_valid: true` — generated only after physical steering Part 1/2 is applied.
2. `drive_odometry_calibration_valid: true` — generated only after measured straight-run scaling is applied.
3. `steering_circle_calibration_valid: true` — generated only after valid Part 3 circle trials are applied.

Manual steering calibration is not blocked by these gates. During steering calibration mode,
drive RPM is forced to zero and direct steering protocol commands are latched for endpoint capture.

## 1. Verify serial identity

```bash
ls -l /dev/serial/by-id/
```

Expected ownership:

- ESC: `Prolific_Technology_Inc._USB-Serial_Controller`
- GNSS: `1a86_USB_Serial`
- IMU: `Silicon_Labs_CP2102`

Run the read-only preflight:

```bash
ros2 run navigation stage1_commissioning_check.py --workspace ~/Sistem-Otomasi-Car/Car-MiniPC/ros
```

The ESC auto-selector is fail-safe: when the configured Prolific identity is absent it does **not**
fall back to arbitrary `/dev/ttyUSB*` devices.

## 2. Steering physical calibration

Before calibration, `-80..+80` is treated as the legacy STM protocol/request span, **not** as
physical wheel degrees. Autonomous motion remains blocked. The normal non-calibration path is
also clamped by the conservative physical fallback in `ackermann.yaml`.

Perform in this order from the C++ GUI:

1. Vehicle stopped, wheels unloaded only if mechanically safe.
2. Enter Steering Calibration Mode. Drive is forced to zero.
3. Physically establish CENTER, then safe LEFT and RIGHT endpoints.
4. Capture the three protocol/feedback references.
5. Enter the measured physical wheel angles; apply Part 1.
6. Perform increasing/decreasing multi-point sweep and apply Part 2 LUT when required.
7. Any re-apply of Part 1/2 invalidates old circle certification.

## 3. Linear odometry scale

Use a straight measured course. The GUI computes the robust median of `real_distance / odom_distance`
and updates the RPM-to-m/s model. On successful Apply it records:

- `drive_odometry_calibration_valid: true`
- `drive_odometry_calibration_scale`
- timestamp

Repeat in both travel directions if possible and reject trials with large lateral displacement.

## 4. Circle geometry Part 3

After steering Part 1/2 and drive scale are valid, record at least two LEFT and two RIGHT circle
trials at >=70% of the operational steering limit. The GUI estimates effective wheelbase and
center turning radius, rejects poor fits, applies safety margin, and writes the certified
`minimum_turning_radius_m` used by Smac, MPPI, velocity smoother and trajectory safety.

## 5. Restart and verify

After applying calibration:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select navigation --event-handlers console_direct+
colcon test-result --verbose
ros2 run navigation stage1_commissioning_check.py --workspace "$PWD"
```

Only after the Stage-1 preflight reports PASS should autonomous motion be tested at low speed.
