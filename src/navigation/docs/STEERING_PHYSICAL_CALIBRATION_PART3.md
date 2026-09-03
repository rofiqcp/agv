# Steering Physical Calibration Part 3 — Circle Test & Kinematic Geometry

Part 3 closes the steering calibration chain with measured vehicle motion. Part 1 separates ESC protocol values from real wheel angle. Part 2 identifies nonlinear command/feedback LUTs and hysteresis. Part 3 identifies the **effective kinematic wheelbase** and the **certified minimum turning radius** from real left/right circle trials.

## Why physical and effective wheelbase are separate

`vehicle.wheelbase_m` is the measured axle-to-axle distance and remains physical geometry. `vehicle.effective_wheelbase_m` is an identified kinematic parameter that absorbs tire compliance, steering linkage geometry and low-speed slip. ESC Ackermann odometry, NavigationCore command conversion and the perception kinematic model use the effective value. URDF/documentation keeps the physical value.

## Circle-test procedure

1. Complete Part 1 and Part 2 first.
2. Measure the physical wheel angle from the LUT-calibrated steering feedback.
3. For LEFT trials, command at least 70% of the operational steering limit and drive a slow, constant arc. Record the real endpoint `(X forward, Y left)` relative to the start pose. Positive Y is LEFT.
4. Repeat at least twice LEFT and twice RIGHT. More trials are recommended.
5. Enter the measured real endpoint and physical wheel angle in the ESC/Odometry page and save each trial.
6. Press **Analisis Circle Test**. The GUI computes center radius and effective wheelbase for each trial.
7. The fit is valid only when both directions have sufficient high-steering trials and the radius-model RMSE is below the configured limit.
8. Press **TERAPKAN GEOMETRI PART 3**. The GUI stores effective wheelbase and left/right radii in `vehicle.yaml`, propagates the kinematic wheelbase to ESC/NavigationCore/perception, and writes a conservative minimum radius to Smac Hybrid-A* and MPPI.

## Equations

For a real endpoint `(x,y)` expressed in the start frame, assuming an approximately constant-curvature arc:

`R_center = (x^2 + y^2) / (2 |y|)`

The steering topic is the calibrated **inner-wheel** angle, therefore:

`R_inner = R_center - track_width/2`

`L_effective = R_inner * tan(|delta_inner|)`

The planner radius is not the theoretical endpoint radius. It is:

`R_certified = max(median(R_left), median(R_right)) * (1 + safety_margin)`

This ensures both steering directions can execute the radius.

## Safety

Run circle tests at low speed in an open area. Do not infer geometry from `/esc/odom` alone; the endpoint must be ground truth or an independent measurement. The old three-point/LUT mappings remain the fallback if Part 3 has not been certified.
