# PRECISION UPGRADE PLAN — PX4 + AUTOWARE -> ADV ROS 2

Tanggal audit: 2026-09-07
Workspace: `/home/otomasi/ros`
Status: PLAN ONLY — belum mengaktifkan algoritma baru ke actuator.

## 1. Kesimpulan audit

Target utama bukan memasukkan seluruh PX4 atau Autoware. Targetnya mengambil mekanisme yang paling meningkatkan presisi dan robustness tanpa membuat controller/fusion ganda.

Urutan pengaruh terbesar ke presisi ADV saat ini:
1. Measurement timestamp + timesync + frame/covariance consistency.
2. Kalibrasi steering fisik/LUT/hysteresis dan kalibrasi IMU nyata.
3. Transport F411<->F103 deterministic + sequence/jitter/loss proof.
4. ESC-vx + gyro-z fused twist dan stationary clamp.
5. Ackermann yaw-rate feed-forward + PI anti-windup dalam shadow mode lalu active.
6. Steering actuator delay predictor + dynamic steering offset observer.
7. Delay-aware localization diagnostics, Mahalanobis/innovation monitor, yaw-bias observer.
8. Control validator + performance analysis untuk tuning berbasis angka.
9. Semantic one-way routing dari Autoware concepts.
10. Depth swept-path AEB sebagai safety independen.

## 2. Temuan kritis source aktif

- F411 `stm32f401/src/VescGateway.h` saat ini `kBaud = 115200`.
- F103 `hoverboard-firmware-hack-FOC/Src/vesc/f103_boot_layout.h` saat ini `F103_VESC_UART_BAUD = 115200u`.
- Komentar F103 menyebut production 1 Mbaud tetapi konstanta aktual masih 115200; ini harus diperlakukan sebagai mismatch dokumentasi-vs-runtime.
- `src/navigation/config/imu.yaml`: `stationary_calibration_valid: false`, bias masih nol/default.
- `src/esc/config/ackermann.yaml`: `steering_physical_calibration_enabled: false`, `steering_physical_lut_enabled: false`.
- GNSS NEO-M9N aktif 10 Hz, sementara source PX4 UBX menyatakan M9N 8 Hz menjaga seluruh satelit; >8 Hz dapat membatasi satelit yang digunakan menjadi 16.
## 3. P0-1 — Timing, sample age, timesync, frame transform

**PX4 FROM**
- `px4/PX4-Autopilot/msg/TimesyncStatus.msg`
- `px4/PX4-Autopilot/msg/SensorGps.msg`
- `px4/PX4-Autopilot/src/modules/sensors/vehicle_imu/VehicleIMU.cpp`
- `px4/px4_ros_com/src/lib/frame_transforms.cpp`

**TO**
- `src/navigation/src/gnss_node.cpp`
- `src/navigation/src/imu_node.cpp`
- `src/navigation/src/localization_core.cpp`
- `src/stmf4/src/stmf4_hmi_bridge.cpp`
- `stm32f401/src/VescGateway.cpp/.h`

**Buat** satu kontrak: `measurement_timestamp`, `receive_timestamp`, `publish_timestamp`, sequence, age, jitter, loss, RTT/clock-offset. Pindahkan NED/ENU dan sensor/body transform ke helper tunggal beserta covariance transform. Jangan memperbaiki timestamp dengan memberi `now()` baru pada data lama.

**Mengapa P0:** GNSS, IMU, ESC yang benar tetapi masuk filter pada waktu yang salah menghasilkan pose/yaw yang terlihat noise atau terlambat. Ini tidak dapat disembuhkan hanya dengan tuning Q/R.

## 4. P0-2 — Kalibrasi sensor dan steering sebelum controller baru

**PX4 FROM:** pola sample health/clipping/rate dan calibration discipline dari `VehicleIMU.cpp`.
**AUTOWARE FROM:** `autoware_steer_offset_estimator`, `control_data_collecting_tool`.

**TO**
- `src/navigation/config/imu.yaml`
- `src/navigation/src/imu_node.cpp`
- `src/esc/config/ackermann.yaml`
- `src/esc/src/ackermann_controller_server.cpp`
- existing experiment/logger tools.

Lakukan stationary IMU calibration berulang, simpan gyro bias + measured covariance. Steering harus mempunyai center fisik, left/right physical limit, command-vs-actual LUT increasing/decreasing, backlash/hysteresis, deadband dan repeatability. Jangan aktifkan dynamic offset observer sebelum static calibration benar.
## 5. P0-3 — Transport deterministic F411 <-> F103

**PX4 FROM**
- `px4/mavlink-router/src/` untuk single physical owner/logical endpoint pattern.
- `px4/PX4-Autopilot/msg/TimesyncStatus.msg` untuk RTT/clock-offset.
- PX4 DDS topic-rate concept hanya sebagai rate-class reference.

**TO**
- `stm32f401/src/VescGateway.cpp/.h`
- `hoverboard-firmware-hack-FOC/Src/setup.c`
- `hoverboard-firmware-hack-FOC/Src/config.h`
- `hoverboard-firmware-hack-FOC/Src/vesc/f103_boot_layout.h`
- `src/esc/src/vesc_tool_bridge.cpp`

Sebelum pindah 115200 -> 1 Mbaud, buat counter CRC/frame/overrun/loss/sequence/jitter dan endurance test. Uji 1 Mbaud kedua sisi bersamaan; jangan pernah membuat satu sisi 1 M dan sisi lain 115200. Bila 1 M tidak CRC-clean dan 50/20 Hz tidak stabil, rollback terukur ke 115200 sesuai requirement commissioning.

Target rate: command/RT 50 Hz; APP 20 Hz; health 5-10 Hz. Transaction config tidak boleh menghentikan periodic telemetry. Python 65101 exclusive saat maintenance transaction; VESC Tool 65102 monitor tidak boleh otomatis mengambil ownership.

## 6. P0-4 — Fused vehicle twist

**AUTOWARE FROM**
- `autoware_core/localization/autoware_gyro_odometer/src/gyro_odometer.cpp`
- `autoware_core/localization/autoware_stop_filter/src/stop_filter.cpp`

**TO**
- buat `src/navigation/src/vehicle_twist_fusion.cpp`
- `src/navigation/src/imu_speed_diagnostic.cpp`
- `src/navigation/config/ekf.yaml`

Gunakan measured ESC longitudinal speed sebagai `vx` dan timestamp-aligned IMU gyro-Z sebagai `wz`. Existing Ackermann kinematic yaw-rate menjadi comparator/slip detector, bukan measurement kedua yang dianggap independen. Stationary clamp menggunakan existing richer gate: ESC speed + gyro + accel + hold/hysteresis.
## 7. P0-5 — PX4-like Ackermann yaw-rate closed loop

**PX4 FROM**
- `PX4-Autopilot/src/modules/rover_ackermann/AckermannRateControl/AckermannRateControl.cpp`
- `PX4-Autopilot/src/lib/pid/PID.cpp`

**TO**
- `src/esc/src/ackermann_controller_server.cpp`
- `src/esc/config/ackermann.yaml`

Formula inti: feasible yaw-rate = `min(|v|*tan(delta_max)/L, yaw_rate_limit)`; slew-limit omega setpoint; feed-forward `delta_ff=atan(L*omega/v)`; PI correction dari IMU gyro-Z; conditional anti-windup saat steering saturated. Gunakan measured ESC speed, bukan throttle estimate PX4.

Aktivasi awal hanya forward + Nav2 + IMU fresh. Reverse tetap geometry/feed-forward sampai data reverse cukup. Semua output tetap melewati physical steering clamp + LUT existing. MPPI tetap controller trajectory; PI ini hanya lower-level plant correction.

PASS: yaw-rate RMSE turun tanpa steering oscillation, integral reset pada stale/ESTOP/source change, physical steering tidak pernah melewati calibration limit.

## 8. P0-6 — Steering delay predictor

**AUTOWARE FROM:** `autoware_universe/control/autoware_mpc_lateral_controller/src/steering_predictor.cpp`.
**TO:** helper baru di package ESC/navigation + `mppi_closed_loop_supervisor.cpp`.

Identifikasi `dead_time_sec` dan `tau` dari target-vs-actual step response. Predictor orde-1 harus berjalan observer-only dahulu. Gunakan predicted steering untuk kinematic yaw-rate/safety model hanya jika predicted-vs-actual RMSE lebih baik daripada target-vs-actual RMSE.

Ini penting karena MPPI dan model Ackermann akan lebih presisi bila tidak menganggap roda mencapai target secara instan.

## 9. P0-7 — Dynamic steering zero/offset observer

**AUTOWARE FROM:** `autoware_universe/vehicle/autoware_steer_offset_estimator/src/steer_offset_estimator.cpp`.
**TO:** buat `src/esc/src/steering_offset_observer.cpp`.

Gunakan estimator scalar Kalman/RLS dengan velocity, yaw-rate dan steering actual. Gating wajib: minimum velocity, steering kecil/moderat, steering-rate rendah, gyro fresh, tidak slip, tidak hard turn. Output candidate offset, covariance, Kalman gain, residual dan confidence.

Default `observer_only=true`. Jangan menulis EEPROM/YAML otomatis. Static mechanical calibration tetap lapisan dasar; dynamic offset hanya correction kecil setelah multi-run PASS.
## 10. P0-8 — Delay-aware localization + innovation diagnostics

**AUTOWARE FROM**
- `autoware_core/localization/autoware_ekf_localizer/src/ekf_localizer.cpp`
- `.../utils/measurement.cpp`
- `.../utils/mahalanobis.cpp`

**PX4 FROM**
- `PX4-Autopilot/msg/EstimatorInnovations.msg`
- `PX4-Autopilot/msg/EstimatorGpsStatus.msg`
- `PX4-Autopilot/msg/EstimatorStatusFlags.msg`

**TO**
- `src/navigation/src/localization_core.cpp`
- `src/navigation/config/localization_cpp.yaml`
- `src/navigation/config/ekf.yaml`

Autoware EKF memperlihatkan extended-state delayed update, measurement delay gate, Mahalanobis gate, queue diagnostics dan yaw-bias state. Jangan mengganti `robot_localization` dahulu. Tahap pertama: buat external consistency/innovation monitor yang time-aligned dan yaw-bias observer; ukur apakah lag GNSS/IMU/ESC merupakan sumber error nyata.

Jika observer membuktikan delayed update memberi manfaat besar, baru evaluasi dua opsi secara A/B: konfigurasi lagged-data/history yang tersedia pada estimator existing versus EKF lokal custom/Autoware-inspired. Migrasi filter hanya boleh dilakukan bila RMSE/P95 lebih baik dan TF continuity tetap benar.

## 11. P0-9 — GNSS M9N optimization

**PX4 FROM:** `PX4-Autopilot/src/drivers/gps/devices/src/ubx.cpp`.
**TO:** `src/navigation/src/gnss_node.cpp`, `src/navigation/config/gnss.yaml`, F411 NEO3 path bila menjadi authority.

A/B 8 Hz vs 10 Hz minimal beberapa run outdoor identik. Nilai: satellites, hAcc, vAcc, pDOP/HDOP, sAcc, COG accuracy, packet jitter/loss, innovation, position/yaw RMSE. Pilih rate berdasarkan total accuracy; jangan berdasarkan Hz tertinggi.

Tambahkan NAV-STATUS dan, bila receiver mendukung, MON-RF/SEC-SIG low-rate untuk jamming/spoofing integrity. Satu GNSS authority saja boleh masuk fusion topic pada runtime.
## 12. P1-1 — Control validator dan performance analysis

**AUTOWARE FROM**
- `autoware_universe/control/autoware_control_validator/src/control_validator.cpp`
- `autoware_universe/control/autoware_control_performance_analysis/src/control_performance_analysis_core.cpp`

**TO**
- `src/navigation/src/mppi_closed_loop_supervisor.cpp`
- `src/navigation/src/trajectory_safety_supervisor.cpp`
- experiment logger/exporter.

Tambahkan latency, lateral/longitudinal error, heading error, curvature, speed error, steering error, lateral jerk, acceleration error, rollback, over-speed dan stop-point overrun. Gunakan consecutive invalid count/hysteresis. Modul ini mengukur dan memvalidasi; tidak mengendalikan steering.

Tujuan: setiap tuning MPPI, yaw PI, steering predictor dan velocity smoother dinilai dengan metric yang sama. Ini mencegah tuning berdasarkan feeling.

## 13. P1-2 — Final command safety envelope

**AUTOWARE FROM:** `autoware_universe/control/autoware_control_command_gate/src/common/control_command_filter.cpp`.
**TO:** final stage `src/esc/src/ackermann_controller_server.cpp`.

Ambil speed-dependent steering angle/rate, lateral acceleration/jerk, longitudinal accel/jerk dan actual-steering-difference limiter. Nav2 velocity_smoother tetap nominal smoother; final gate hanya safety backstop agar tidak terjadi double smoothing dan lag.

## 14. P1-3 — Structured system-identification runs

**AUTOWARE FROM:** `autoware_tools/control_data_collecting_tool/`.
**TO:** existing `/home/otomasi/ros/data`, experiment GUI/logger dan test tools.

Adaptasi desain test: straight forward/reverse, constant speed, constant acceleration/deceleration, steering steps kiri/kanan, circle, figure-eight, S-curve dan along-road. Buat coverage matrix speed x steering x steering-rate agar calibration tidak hanya bagus pada satu operating point.

Hasil dipakai untuk: steering LUT/hysteresis, dead time/tau, dynamic offset confidence, drive speed scale, yaw-rate PI gain, acceleration/jerk envelope dan MPPI validation.
## 15. P1-4 — Semantic road routing

**AUTOWARE FROM**
- `autoware_core/planning/autoware_mission_planner/src/lanelet2_plugins/default_planner.cpp`
- `autoware_core/planning/autoware_route_handler/`
- `autoware_core/common/autoware_lanelet2_utils/`

**TO**
- `src/navigation/tools/osm_pgm.py`
- buat `src/navigation/src/osm_route_planner.cpp`
- `src/navigation/src/navigation_core.cpp`
- Web/RViz map visualization.

P0 awal cukup directed OSM graph: nearest legal road, one-way, legal yaw, disconnected route reject, NavigateThroughPoses. SMAC Hybrid-A* + MPPI tetap geometric planner/controller. Lanelet2 penuh baru bila stop-line, no-entry, lane individual, parking/drop zone atau intersection semantics memang dibutuhkan.

Semantic routing terutama meningkatkan correctness dan repeatability rute; bukan pengganti perbaikan sensor/steering untuk precision tracking.

## 16. P1-5 — Processing time + readiness proof

**AUTOWARE FROM:** `autoware_universe/system/autoware_processing_time_checker/src/processing_time_checker.cpp`.
**PX4 FROM:** timestamp/rate/gap discipline dan failsafe status patterns.

**TO:** navigation, localization, perception, ESC bridge, Web diagnostics.

Ukur current/min/max/mean/P95/P99 untuk GNSS, IMU, localization, semantic route, SMAC, MPPI/safety, final command, F411 ACK dan telemetry. Pisahkan processing duration, message age, inter-arrival jitter, RTT dan end-to-end latency.

## 17. P2 — Safety yang memberi robustness, bukan precision utama

**AUTOWARE FROM:** `autoware_universe/control/autoware_autonomous_emergency_braking/src/node.cpp`.
**TO:** `trajectory_safety_supervisor.cpp` + Astra depth metric cloud.

Ambil swept ego footprint, predicted IMU/trajectory path, pointcloud height/voxel filtering dan RSS/reaction-braking distance. Jangan membuat actuator authority kedua. YOLO harus boleh OFF sementara depth safety tetap bekerja.

AEB penting untuk production safety, tetapi dikerjakan setelah timing/localization/control baseline presisi stabil agar debugging tidak tercampur.
## 18. Yang jangan dilakukan

- Jangan mengganti SMAC+MPPI dengan seluruh Autoware control/planning stack.
- Jangan menjalankan PX4 flight EKF sebagai estimator kedua paralel.
- Jangan memasang Autoware MPC lateral sebagai controller kedua di bawah MPPI.
- Jangan menambahkan full ROS speed PI bila F103/VESC speed loop sudah menjadi actuator loop; pakai diagnostic/slow trim hanya jika data membuktikan perlu.
- Jangan membuat Autoware velocity smoother kedua setelah Nav2 velocity_smoother.
- Jangan auto-persist dynamic steering offset sebelum observer multi-run stabil.
- Jangan menganggap 1 Mbaud selesai hanya karena komunikasi sesaat berhasil; harus CRC/jitter/loss/endurance PASS.
- Jangan tune Q/R untuk menutupi timestamp yang salah.
- Jangan mengaktifkan dua GNSS authority ke EKF.

## 19. Urutan implementasi final

### Stage 0 — Baseline
Record rosbag dan metrics sistem lama: straight, steering step, circle, figure-eight; simpan hash YAML/firmware. Metric minimum: position/yaw error, vx/wz error, steering target/actual, command latency, packet jitter/loss.

### Stage 1 — Instrumentation only
Implement sample timestamp, sequence, typed health, rate/jitter/loss/RTT, frame-transform tests, processing metrics. Tidak mengubah actuator/fusion.

### Stage 2 — Calibration foundation
IMU stationary calibration; GNSS 8-vs-10 Hz; steering physical calibration + hysteresis LUT; speed scale calibration. Ulangi baseline dan pilih parameter berdasarkan RMSE/P95.

### Stage 3 — Transport
Tes F411/F103 1 Mbaud kedua sisi dengan 50 Hz RT + 20 Hz APP + maintenance/VESC Tool concurrency. Enforce hanya bila endurance clean.

### Stage 4 — Twist and odometry
Vehicle twist fusion + stop clamp dalam shadow, lalu masukkan ke local EKF setelah timestamp/sign/covariance PASS. Tambah reset_counter semantics.

### Stage 5 — Steering plant identification
Run structured steering steps untuk dead-time, tau, backlash, center bias dan speed-dependent response. Steering predictor + dynamic offset tetap observer-only.

### Stage 6 — Yaw-rate controller shadow
PX4-like FF+PI dihitung dan dilog, existing steering command tetap authority. Tune P lalu I kecil dengan anti-windup.

### Stage 7 — Yaw-rate controller active low speed
Forward only, speed cap rendah. PASS bila yaw-rate/path tracking RMSE membaik tanpa oscillation atau latency regression.
### Stage 8 — Delay-aware localization
Aktifkan external innovation/Mahalanobis monitor + yaw-bias observer. Bandingkan estimator baseline dengan delay-compensated candidate menggunakan dataset yang sama. Filter authority tidak berubah sampai evidence cukup.

### Stage 9 — Control validator + final safety envelope
Control validator observer -> WARN -> autonomous gate bila sudah tervalidasi. Final command filter shadow -> enforce. Pastikan velocity_smoother tetap nominal, gate tidak menambah double-lag.

### Stage 10 — Semantic routing
Directed OSM graph -> route preview -> NavigateThroughPoses -> legal route enforcement. Uji one-way, wrong-way yaw, disconnected route dan final approach.

### Stage 11 — Depth AEB
Astra metric pointcloud -> swept footprint WARN-only -> HARD_STOP setelah false-positive/latency tests.

### Stage 12 — Regression/certification
Fault injection GNSS dropout/jump, IMU stale/spike, steering delay/offset, F411/F103 reboot, packet loss, maintenance takeover, VESC Tool monitoring dan ESTOP. Semua perubahan dibandingkan dengan Stage-0 baseline.

## 20. Definition of Done precision

Upgrade precision dianggap berhasil bila pada repeated runs yang sama:
- steering target-vs-actual RMSE/P95 membaik;
- yaw-rate target-vs-IMU RMSE/P95 membaik;
- cross-track dan heading RMSE/P95 membaik;
- stationary odometry/yaw tidak creeping;
- GNSS correction tidak menghasilkan jump tak terjelaskan;
- measurement age/jitter/loss terbukti dan tidak stale-masquerading-as-fresh;
- RT telemetry 50 Hz dan APP 20 Hz tetap stabil;
- command-to-actuator latency P95/P99 tidak memburuk;
- tidak ada oscillation baru atau excessive final-gate clipping;
- one-way route ratio 100% dan wrong-way segment 0 pada semantic-routing tests;
- E-stop dan maintenance precedence selalu deterministic.

## 21. Prioritas keputusan

Jika hanya boleh mengerjakan lima hal dulu: **timing/timesync**, **kalibrasi steering+IMU**, **transport proof**, **fused twist**, lalu **PX4-like yaw-rate FF+PI**. Setelah lima ini stabil, Autoware steering predictor/offset dan delayed-localization diagnostics akan memberi peningkatan presisi berikutnya dengan risiko jauh lebih kecil.
