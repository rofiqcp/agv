# Precision Revision — Part 1 (P0 Foundation)

This part intentionally fixes correctness foundations before MPPI/global-yaw tuning.

## Implemented

1. **Fail-closed IMU gyro freshness**
   - `/imu/data` is not re-stamped/published when gyro packets are stale.
   - Freshness is independent from the slower stream-recovery timeout.
   - Accel/orientation fields are also only populated while fresh.
   - All `has_*` state is cleared across USB reconnects.
2. **GUI Goal Checker key correction**
   - Removed dead `general_goal_checker.*` editor entries.
   - The only goal-tolerance editor now targets `controller_server.ros__parameters.goal_checker.*`.
3. **`vehicle.yaml` physical authority**
   - GUI edits to vehicle geometry atomically propagate exact wheelbase/track/footprint/min-turning-radius to dependent YAML files.
   - Certified speed/yaw/acceleration limits act as ceilings: downstream tuning is only clamped downward, never automatically made more aggressive.
   - `ESC speed_max` is deliberately NOT synchronized because it is a calibrated RPM↔m/s conversion scale, not merely a command ceiling.
4. **Collision Monitor ambiguity removed for commissioning**
   - Default launch is OFF. Manual preview remains available and still outputs only `/cmd_vel/collision_preview`.
5. **GUI persistence wording hardened**
   - A YAML save no longer implies a running node has consumed the value; the GUI explicitly states restart/live-apply may be required.

## Deferred to Part 2 / Part 3

- active-config root (`src` vs `install`) and runtime parameter introspection/live-apply matrix;
- automated IMU calibration wizard and covariance estimation;
- dynamic ESC/IMU covariance and slip weighting;
- gated absolute yaw (GNSS COG / calibrated magnetometer);
- production Collision Monitor command ownership after metric-camera validation;
- full report/session metadata and automated tuning sweeps.

## Safety note

Static validation cannot replace a low-speed physical commissioning test. Before autonomous motion, verify steering sign/center, IMU gyro sign/freshness, odometry scale, TF continuity, e-stop, and motion gate on the real vehicle.
