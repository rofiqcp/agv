# MASTER UPGRADE PLAN — AUTOWARE + PX4 UNTUK ADV ROS 2

Tanggal konsolidasi: 2026-09-07  
Workspace aktif: `/home/otomasi/ros`  
Sumber referensi: `/home/otomasi/ros/autoware` dan `/home/otomasi/ros/px4`.

Dokumen ini menggantikan dua blueprint terpisah sebagai **satu arsitektur upgrade**. Tujuannya bukan menumpuk algoritma Autoware + PX4, tetapi memilih algoritma terbaik untuk setiap lapisan, mempertahankan Nav2/SMAC/MPPI yang sudah cocok untuk ADV, dan memastikan hanya ada **satu owner** untuk route, state, command, sensor authority, dan physical transport.

## 1. Keputusan arsitektur utama

- **Autoware dipakai untuk:** semantic road routing, control validation, final command safety envelope, diagnostics graph, processing-time supervision, swept-path/AEB concept, steering offset observer, actuator delay predictor, localization/control/planning evaluator.
- **PX4 dipakai untuk:** sensor sample/timestamp discipline, GNSS UBX integrity, IMU health/rate/jitter, frame/covariance transform discipline, odometry reset semantics, timesync/link statistics, failsafe state semantics, lower-level Ackermann yaw-rate feedback control, single-owner transport/router, fault injection/logging discipline.
- **Nav2 tetap dipakai untuk:** global geometric planning `SMAC Hybrid-A*`, local control `MPPI`, lifecycle/costmap, behavior tree, goal execution.
- **robot_localization tetap dipakai untuk EKF:** jangan mengganti EKF dengan PX4 EKF/Autoware localization stack secara penuh.
- **F411 + F103/VESC tetap actuator transport:** MiniPC -> F411 @ 1 Mbaud, F411 -> F103/VESC @ 1 Mbaud.
- **RT telemetry tetap 50 Hz, APP telemetry 20 Hz:** upgrade tidak boleh menurunkan rate atau menambah blocking timeout.
- **YOLO tidak boleh menjadi syarat emergency obstacle stop:** depth safety metric harus tetap hidup saat YOLO OFF.
- **Lanelet2 penuh belum menjadi P0:** directed OSM graph dipilih dahulu karena memberi legal one-way routing dengan perubahan minimum.

## 2. Aturan anti-tumpukan algoritma

1. Hanya satu semantic router: `osm_route_planner`; SMAC bukan semantic router, SMAC tetap geometric path planner di corridor/waypoints legal.
2. Hanya satu velocity smoothing dominan: Nav2 `velocity_smoother`; limiter downstream hanya safety backstop, bukan smoother kedua yang lebih agresif.
3. Hanya satu lower-level yaw-rate feedback controller: controller PX4-like di `ackermann_controller_server.cpp`; jangan menambah MPC lateral Autoware sebagai controller kedua.
4. Hanya satu final actuator gate: Autoware-like final command gate di `ackermann_controller_server.cpp` sebelum conversion LUT/RPM/protocol.
5. Hanya satu high-level vehicle state/readiness owner: `system_state_manager`; Web/HMI hanya mirror.
6. Hanya satu GNSS authority pada runtime: direct USB **atau** F411 bridge; jalur lain diagnostic/remap.
7. Hanya satu physical owner tiap UART F411<->F103: logical clients 65101/65102/ROS masuk melalui router, bukan membuka UART sendiri.
## 3. Aliran sistem final yang dipilih

```text
GNSS/IMU/MAG/ESC/Depth
        -> typed sensor status + measurement timestamp + health
        -> frame/time normalization
        -> vehicle_twist_fusion + stop filter
        -> local/global EKF + localization consistency monitor
        -> system_state_manager/readiness

Web/RViz/HMI Goal
        -> osm_route_planner (legal directed road)
        -> NavigateThroughPoses
        -> SMAC Hybrid-A*
        -> MPPI
        -> trajectory_safety_supervisor
        -> cmd_vel_router / authority arbitration
        -> Nav2 velocity_smoother
        -> Ackermann yaw-rate FF+PI (PX4-like, forward only initially)
        -> final command gate (Autoware-like safety envelope)
        -> steering LUT + drive conversion
        -> F411 @1M -> F103/VESC @1M -> steering + traction

Astra depth -> safety cloud -> swept-path collision -> safety veto --------^ 
Python 65101 / VESC Tool 65102 -> single-owner VESC router ----------------^ 
```

## 4. Pembagian fungsi: siapa yang menang bila Autoware dan PX4 overlap

| Fungsi | Dipilih | Alasan |
|---|---|---|
| Semantic route / one-way | Autoware pattern | PX4 tidak menyediakan mission road semantics; Autoware unggul pada topology/lane legality |
| Geometric planner/controller | Existing Nav2 | Sudah sesuai Ackermann dan tidak perlu diganti Autoware/PX4 |
| Yaw-rate lower-level steering | PX4 rover pattern | Sangat sesuai feedback gyro + measured speed pada Ackermann |
| Final speed/jerk/steer safety envelope | Autoware command gate pattern | Lebih lengkap untuk limiting command kendaraan jalan |
| System state flags | PX4 semantics | BOOT/STANDBY/READY/AUTONOMOUS/TELEOP/MAINTENANCE/FAULT/ESTOP jelas |
| Dependency readiness/hysteresis | Autoware diagnostic graph pattern | Cocok menggabungkan leaf health menjadi SYSTEM_READY |
| Sensor timing/rate/jitter | PX4 pattern | Lebih kuat pada timestamp_sample, rate, clipping, reset, link health |
| GNSS fusion qualification | PX4 integrity + Autoware GNSS poser pattern | PX4 membaca health/integrity; Autoware memberi pola covariance/freshness/frame gate |
| ESC vx + gyro wz fusion | Autoware gyro odometer pattern | Sangat langsung untuk kendaraan darat; kualitas input memakai health PX4-like |
| Stationary zeroing | Existing ZUPT + Autoware stop filter | Existing gate lebih kaya; Autoware hanya final clamp |
| Steering offset | Autoware estimator | Lebih spesifik untuk steering offset kendaraan |
| Steering delay | Autoware predictor | Dipakai observer/model, bukan controller baru |
| Depth emergency stop | Autoware AEB pattern | Swept footprint + TTC/stop distance lebih tepat |
| Link/router ownership | PX4/mavlink-router pattern | Cocok untuk banyak logical client, satu serial owner |
| Evaluator | Autoware metrics + PX4 manifest/logging | Metrics Autoware, reproducibility/log structure PX4 |
# BAGIAN A — SENSOR, TIME, FRAME, DAN DATA CONTRACT

## 5. GNSS: satu authority, parser PX4-like, fusion gate Autoware-like

### FROM — PX4
- `/home/otomasi/ros/px4/PX4-Autopilot/src/drivers/gps/devices/src/ubx.cpp`
- `/home/otomasi/ros/px4/PX4-Autopilot/src/drivers/gps/devices/src/ubx.h`
- `/home/otomasi/ros/px4/PX4-Autopilot/src/drivers/gps/gps.cpp`
- `/home/otomasi/ros/px4/PX4-Autopilot/msg/SensorGps.msg`
- `/home/otomasi/ros/px4/PX4-Autopilot/msg/EstimatorGpsStatus.msg`

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_core/sensing/autoware_gnss_poser/src/gnss_poser_node.cpp`
- `/home/otomasi/ros/autoware/autoware_core/sensing/autoware_gnss_poser/config/gnss_poser.param.yaml`

### TO — program aktif
- `/home/otomasi/ros/src/navigation/src/gnss_node.cpp`
- `/home/otomasi/ros/src/navigation/include/gnss/gnss_node.hpp`
- `/home/otomasi/ros/src/navigation/config/gnss.yaml`
- `/home/otomasi/ros/stm32f401/src/Neo3Sensors.cpp`
- `/home/otomasi/ros/src/stmf4/src/stmf4_hmi_bridge.cpp`
- `/home/otomasi/ros/src/navigation/src/localization_core.cpp`

### Yang harus diubah
1. `gnss_node.cpp` tetap menjadi UBX direct driver; jangan diganti PX4 driver mentah.
2. Tambah state konfigurasi eksplisit: `PORT_FOUND -> STREAM_VALID -> CONFIG_SENT -> ACKED -> PVT_STREAMING`.
3. NAV-PVT harus dianggap satu epoch atomik berdasarkan `iTOW`; invalid checksum/length tidak boleh mengubah state.
4. Tambah low-rate integrity: NAV-DOP, NAV-STATUS, dan bila receiver mendukung MON-RF/SEC-SIG tanpa mengganggu PVT.
5. Pisahkan `stream_connected`, `fix_valid`, dan `fusion_qualified`; CONNECTED tidak berarti boleh masuk EKF.
6. `localization_core.cpp` memakai pola GNSS Poser untuk covariance/freshness/frame gate, tetapi tetap mempertahankan UBX-specific quality yang lebih kaya.
### GNSS authority yang dipilih
- Production harus punya parameter global `gnss_authority: direct|stm32`.
- `direct`: `gnss_node.cpp` publish `/gnss/*`; bridge F411 remap ke `/gnss_stm32/*` atau tidak publish fusion topic.
- `stm32`: `stmf4_hmi_bridge.cpp` publish `/gnss/*`; direct driver OFF/remap diagnostic.
- Jangan membuat automatic failover dulu sebelum estimator reset/source-change sudah benar.

### Timestamp yang harus dibuat
- Direct USB: `measurement_stamp` dari UTC/iTOW alignment; fallback arrival hanya jika tidak ada sumber lebih baik.
- F411: compact record membawa `iTOW`, `sequence`, `mcu_receive_ms`; PC menyimpan `host_receive_time`.
- Publish `measurement_age`, `arrival_to_publish_latency`, `stamp_source`, `stamp_regression_count`, `future_stamp_count`.
- `LocalizationCore` menilai freshness dari measurement timestamp, bukan sekadar callback time.

### 8 Hz vs 10 Hz
- Jangan langsung memilih 10 Hz hanya karena lebih tinggi.
- Uji A/B 8 Hz dan 10 Hz minimal 5–10 menit outdoor dengan `numSV`, hAcc, vAcc, sAcc, DOP, missing epoch, jitter, innovation, path error.
- Jika 8 Hz memberi satellite/accuracy lebih baik tanpa menurunkan kontrol, gunakan 8 Hz; jika 10 Hz terbukti setara/lebih baik, pertahankan 10 Hz.

### Output baru
Buat typed message lokal `AgvGnssStatus.msg` di package `agv_msgs`, bukan memakai `px4_msgs` atau `autoware_msgs` langsung. Field minimal: source, measurement stamp, stream/config/fix/fusion validity, sat count, hAcc/vAcc/sAcc/DOP, COG/headAcc, spoof/jam state, checksum/drop/rate/jitter, quality bitmask, reject reason.

### Hasil upgrade
GNSS menjadi **traceable dan deterministic**: diketahui data berasal dari jalur mana, epoch mana, umur data berapa, kualitasnya apa, kenapa diterima/ditolak, dan tidak ada double publisher yang diam-diam membuat EKF menerima measurement ganda.
## 6. IMU Yahboom/WIT: pertahankan driver, upgrade sample discipline dan health

### FROM — PX4
- `/home/otomasi/ros/px4/PX4-Autopilot/src/modules/sensors/vehicle_imu/VehicleIMU.cpp`

### TO
- `/home/otomasi/ros/src/navigation/src/imu_node.cpp`
- `/home/otomasi/ros/src/navigation/include/imu/imu_node.hpp`
- `/home/otomasi/ros/src/navigation/config/imu.yaml`
- `/home/otomasi/ros/src/navigation/src/localization_core.cpp`
- `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`

### Proses upgrade
1. Saat frame WIT checksum-valid lengkap diterima, ambil timestamp frame sebelum decode.
2. Simpan timestamp terpisah untuk ACC/GYRO/ANGLE/MAG, karena paketnya tidak datang persis bersamaan.
3. `/imu/data.header.stamp` memakai sample orientation boundary yang aktual, bukan selalu publish-now.
4. Hitung age setiap field pada waktu publish; gyro stale tidak boleh memakai angka lama seolah fresh.
5. Tambah observed Hz, mean/std/P95 interval, longest gap, checksum fail, resync byte, reconnect, clipping, quaternion norm error.
6. Pertahankan covariance `[0] = -1` untuk field unavailable; jangan memalsukan measurement.
7. Stationary calibration menghitung bias/covariance nyata dan menulis output calibration hanya bila stationary gate lulus.
8. `ackermann_controller_server.cpp` hanya boleh mengaktifkan yaw-rate feedback jika gyro sample fresh dan status valid.

### Output baru
`AgvImuStatus.msg`: stream/config valid, sample ages, rate/jitter per packet type, checksum/reconnect/clipping, bias/covariance source, stationary calibration state, vibration/noise metric, quality/reason bitmask.

### Hasil upgrade
IMU tidak lagi dinilai sekadar 'port terbuka'. Sistem mengetahui apakah gyro yang dipakai controller/EKF benar-benar sample baru, rate stabil, tidak clipping, dan calibration valid.
## 7. IST8310 / magnetometer: PX4 sebagai health reference, bukan scale truth

### FROM
- `/home/otomasi/ros/px4/PX4-Autopilot/src/drivers/magnetometer/isentek/ist8310/IST8310.cpp`

### TO
- `/home/otomasi/ros/stm32f401/src/Neo3Sensors.cpp`
- `/home/otomasi/ros/stm32f401/src/Neo3Sensors.h`
- `/home/otomasi/ros/src/stmf4/src/stmf4_hmi_bridge.cpp`
- `/home/otomasi/ros/src/navigation/src/mag_heading_fusion_node.cpp`
- `/home/otomasi/ros/src/navigation/config/mag_heading.yaml`

### Yang dipakai
- State machine RESET/CONFIGURE/MEASURE/READ non-blocking.
- DRDY timeout, I2C error, register-integrity checker, consecutive failure, reset reason.
- Measurement timestamp dari F411 sampai ROS.

### Yang tidak boleh langsung disalin
PX4 memakai kira-kira `0.07576 uT/LSB`, kode ADV pernah memakai `0.30 uT/LSB`. **Tidak satu pun dijadikan benar tanpa eksperimen.** Pilih scale dari raw-XYZ 360°, hard/soft-iron calibration, field norm, pembanding heading, dan gangguan motor ON/OFF.

### Hasil
Mag menjadi sumber heading opsional yang punya confidence/disturbance status. Ia tidak boleh sendirian membuat global heading 'valid' bila calibration atau magnetic environment buruk.

## 8. Central frame/covariance transform

### FROM
- `/home/otomasi/ros/px4/px4_ros_com/include/px4_ros_com/frame_transforms.h`
- `/home/otomasi/ros/px4/px4_ros_com/src/lib/frame_transforms.cpp`

### TO / CREATE
- Existing: `/home/otomasi/ros/src/navigation/include/navigation/navigation_math.hpp`
- Disarankan tambah: `/home/otomasi/ros/src/navigation/include/navigation/frame_transforms.hpp`
- Disarankan tambah: `/home/otomasi/ros/src/navigation/src/frame_transforms.cpp`
### Fungsi yang dibuat
- `nedVectorToEnu()` / `enuVectorToNed()`.
- `frdVectorToFlu()` / `fluVectorToFrd()`.
- Quaternion equivalents.
- Covariance 3x3 NED<->ENU dan bila perlu covariance odometry 6x6.
- COG bearing -> ROS ENU yaw.

### Target pemakaian
- `gnss_node.cpp`: velocity N/E/D, NAV-COV, COG conversion.
- `stmf4_hmi_bridge.cpp`: GNSS velocity transform harus identik dengan direct path.
- `imu_node.cpp`: mount transform eksplisit; kurangi penggunaan invert flags yang tidak menjelaskan frame.
- `mag_heading_fusion_node.cpp`: mount/heading convention memakai helper yang sama.
- `localization_core.cpp`: semua sign/frame assumptions merujuk library tunggal.

### Test wajib
Unit test basis North/East/Down, yaw +90°, gyro-z kiri/kanan, covariance diagonal/off-diagonal. Physical test harus membuktikan maju = +X base_link, kiri = +yaw sesuai REP-103.

## 9. Odometry reset + sample semantics

### FROM
- `/home/otomasi/ros/px4/PX4-Autopilot/msg/versioned/VehicleOdometry.msg`

### TO
- `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`
- `/home/otomasi/ros/src/navigation/src/localization_core.cpp`

Tambahkan companion `AgvOdometryStatus.msg` dengan `measurement_stamp`, `reset_counter`, `quality`, `last_reset_reason`, sequence/epoch. Counter naik saat F411/F103 reboot, encoder/distance reset, maintenance reset, atau rebase calibration. Packet loss biasa **bukan reset**.

Saat reset terdeteksi, `ackermann_controller_server.cpp` tidak mengintegrasikan `dt` besar. `LocalizationCore::onOdom()` clear/reseed history sehingga reboot saat kendaraan diam tidak menghasilkan teleport pada odom/map.
## 10. Timesync, packet age, jitter, dan proof 50/20 Hz

### FROM — PX4
- `/home/otomasi/ros/px4/PX4-Autopilot/msg/TimesyncStatus.msg`
- `/home/otomasi/ros/px4/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml`

### TO
- `/home/otomasi/ros/stm32f401/src/VescGateway.cpp`
- `/home/otomasi/ros/stm32f401/src/VescGateway.h`
- `/home/otomasi/ros/stm32f401/src/Telemetry.h`
- `/home/otomasi/ros/stm32f401/src/Config.h`
- `/home/otomasi/ros/src/stmf4/src/stmf4_hmi_bridge.cpp`
- `/home/otomasi/ros/hoverboard-firmware-hack-FOC/Src/comms.c`
- `/home/otomasi/ros/hoverboard-firmware-hack-FOC/Src/vesc/vesc_protocol.c`

### Contract telemetry final
- Command critical: 50 Hz deterministic.
- `RT`: 50 Hz — RPM/ERPM, current, steering target/actual, drive target/actual, fault, control/ACK state.
- `APP`: 20 Hz — voltage, temperature, limits, config/controller summary.
- `HEALTH`: 5–10 Hz — sequence loss, jitter, watchdog, reset counter, endpoint/router stats, firmware identity.
- Static/config identity: event/on-change atau <=1 Hz.

Setiap class/frame minimal membawa `sequence`, sender timestamp, data-class/version/length. PC menghitung receive timestamp, inter-arrival, loss, duplicate/out-of-order, jitter P95/P99, RTT/ACK latency, queue/drop, dan clock offset bila timesync sudah tersedia.

**Prinsip timeout:** RT loop tidak boleh menunggu transaksi config sampai 10 detik. Timeout panjang hanya milik operation transaction; streaming selalu non-blocking dan setiap field mempunyai age/validity sendiri.

### Hasil upgrade
Target 50 Hz RT dan 20 Hz APP tidak lagi hanya klaim `topic hz`; tersedia bukti rate, jitter, loss, age, latency, dan failure reason dari F103 sampai MiniPC.
# BAGIAN B — LOCALIZATION DAN STATE ESTIMATION

## 11. Vehicle twist fusion: ESC `vx` + IMU gyro-Z

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_core/localization/autoware_gyro_odometer/src/gyro_odometer.cpp`
- `/home/otomasi/ros/autoware/autoware_core/localization/autoware_gyro_odometer/src/gyro_odometer_diagnostics.cpp`
- `/home/otomasi/ros/autoware/autoware_core/localization/autoware_gyro_odometer/config/gyro_odometer.param.yaml`

### TO / CREATE
- Create `/home/otomasi/ros/src/navigation/src/vehicle_twist_fusion.cpp`
- Modify `/home/otomasi/ros/src/navigation/src/localization_core.cpp`
- Modify `/home/otomasi/ros/src/navigation/config/ekf.yaml`
- Input producer `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`

### Algoritma final
1. Buffer ESC longitudinal speed dan IMU gyro-z berdasarkan **measurement stamp**.
2. Cari pair terdekat dengan `sync_max_gap_sec`; jangan fuse callback yang kebetulan datang berdekatan tetapi sample time jauh.
3. `linear.x = measured ESC speed`; `angular.z = measured gyro-z`.
4. Covariance `vx` berasal dari RPM/tracking quality; covariance `wz` dari IMU calibration/noise.
5. ESC Ackermann kinematic yaw-rate tetap comparator/slip diagnostic, **bukan measurement kedua yang dianggap independen**.
6. Jika salah satu source stale, status fused invalid; jangan menerbitkan sample lama dengan timestamp baru.
7. Setelah observer PASS, `/vehicle/twist_fused` menjadi input local EKF.

### Output
`/vehicle/twist_fused_raw`, `/vehicle/twist_fused`, `/vehicle/twist_fusion/status`.

### Hasil upgrade
EKF menerima twist yang secara fisik lebih kuat: translasi dari roda/ESC, rotasi dari gyro; model Ackermann dipakai sebagai pembanding, bukan mencampur model dengan measurement.
## 12. Stop filter / ZUPT: existing logic menang, Autoware menjadi final clamp

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_core/localization/autoware_stop_filter/src/stop_filter.cpp`
- `/home/otomasi/ros/autoware/autoware_core/localization/autoware_stop_filter/src/stop_filter_node.cpp`

### TO
- `/home/otomasi/ros/src/navigation/src/imu_speed_diagnostic.cpp`
- `/home/otomasi/ros/src/navigation/src/vehicle_twist_fusion.cpp`
- `/home/otomasi/ros/src/navigation/src/localization_core.cpp`

Jangan mengganti stationary detector existing dengan rule Autoware yang lebih sederhana. Gunakan final rule: `abs(vx_esc)<threshold AND abs(gyro_z)<threshold AND accel_stationary AND hold_time_elapsed`, optional wheel RPM near zero. Jika true, clamp `vx=vy=wz=0` dengan covariance floor realistis.

Gunakan hysteresis: enter STOP setelah stabil beberapa ratus ms; exit lebih cepat ketika speed/gyro melewati exit threshold. Satu `/vehicle/stopped` menjadi source-of-truth; `LocalizationCore` tidak menghitung ulang stationary dengan formula berbeda.

## 13. External estimator consistency / innovation monitor

### FROM — PX4
- `/home/otomasi/ros/px4/PX4-Autopilot/msg/EstimatorInnovations.msg`
- `/home/otomasi/ros/px4/PX4-Autopilot/msg/EstimatorStatusFlags.msg`

### TO
- `/home/otomasi/ros/src/navigation/src/localization_core.cpp`
- Create `/home/otomasi/ros/src/navigation/src/localization_evaluator.cpp`
- `/home/otomasi/ros/src/navigation/web/web_server.cpp`
- `/home/otomasi/ros/src/navigation/gui/agv_experiment_catalog.hpp`

Hitung residual time-aligned: GNSS position vs predicted map pose, GNSS velocity vs local velocity, COG vs yaw saat qualified, IMU gyro-z vs Ackermann model, mag heading vs predicted yaw. Simpan measurement variance, predicted variance bila tersedia, combined variance `S`, normalized test-ratio bila sah, accepted/rejected, reason.

**Jangan menyebutnya internal EKF innovation** bila bukan nilai dari robot_localization. Topic/status harus jelas misalnya `/localization/measurement_consistency` dengan `source=external_monitor`.
# BAGIAN C — SEMANTIC ROUTING DAN NAV2

## 14. Semantic road router: Autoware concept, Nav2 execution

### FROM — Autoware mission/routing
- `/home/otomasi/ros/autoware/autoware_core/planning/autoware_mission_planner/src/lanelet2_plugins/default_planner.cpp`
- `/home/otomasi/ros/autoware/autoware_core/planning/autoware_mission_planner/src/lanelet2_plugins/utility_functions.cpp`
- `/home/otomasi/ros/autoware/autoware_core/planning/autoware_mission_planner/src/mission_planner/mission_planner.cpp`
- `/home/otomasi/ros/autoware/autoware_core/planning/autoware_route_handler/src/route_handler.cpp`
- `/home/otomasi/ros/autoware/autoware_core/common/autoware_lanelet2_utils/src/topology.cpp`
- `/home/otomasi/ros/autoware/autoware_core/common/autoware_lanelet2_utils/src/nn_search.cpp`

### TO
- Modify `/home/otomasi/ros/src/navigation/tools/osm_pgm.py`
- Create `/home/otomasi/ros/src/navigation/src/osm_route_planner.cpp`
- Create `/home/otomasi/ros/src/navigation/config/osm_route_planner.yaml`
- Create `/home/otomasi/ros/src/navigation/maps/undip/undip_nav2_route_graph.json`
- Modify `/home/otomasi/ros/src/navigation/src/navigation_core.cpp`
- Modify `/home/otomasi/ros/src/navigation/web/web_server.cpp`
- Modify `/home/otomasi/ros/src/navigation/web/static/app.js`
- Modify `/home/otomasi/ros/src/navigation/gui/modules/navigation_rviz_panel.cpp`

### Fungsi yang dibuat
- `buildDirectedRouteGraph()` di generator.
- `findNearestLegalEdge()` untuk snap start/goal.
- `planLegalRoute()` untuk directed A*/Dijkstra.
- `validateRoadGoal()` untuk wrong-way/off-road/disconnected.
- `buildNavigateThroughPosesGoal()` di NavigationCore.

Graph memakai koordinat map meter dan tag `oneway`, `highway`, `lanes`, `maxspeed`, length, polyline, enabled. `oneway=yes` hanya membentuk edge legal searah.
### Proses runtime Goal Pose setelah upgrade
1. User mengirim goal x/y/yaw biasa dari Web/RViz/HMI.
2. `NavigationCore` mengecek map/localization readiness.
3. Router snap start dan goal ke edge legal terdekat dengan score jarak + heading.
4. Directed shortest path dibuat di graph OSM.
5. Polyline di-resample menjadi pose rapat dan yaw mengikuti tangent jalan.
6. Jika requested yaw melawan one-way, policy default `AUTO_CORRECT_YAW`; Web tetap menampilkan yaw asli dan yaw legal.
7. Route legal dikirim sebagai `NavigateThroughPoses`; SMAC membuat path geometrik antar pose, MPPI tetap mengontrol kendaraan.
8. Final user goal tetap disimpan; final approach dilakukan sesuai batas off-road yang diizinkan.
9. Jika tidak ada legal path dan `require_legal_route=true`, status harus `ROUTE_REJECTED`, **tidak boleh fallback diam-diam ke direct SMAC**.

### Kenapa tidak langsung Lanelet2 penuh
Occupancy map `.pgm/.yaml` sekarang masih sangat berguna untuk Nav2. Directed OSM graph memberi one-way/legal direction tanpa membawa dependency/autoware_msgs besar. Lanelet2 menjadi tahap lanjut hanya bila dibutuhkan lane individual, stop line, no-entry formal, parking/drop zone, speed zone, atau intersection semantics kompleks.

### Lanelet2 future FROM
- `/home/otomasi/ros/autoware/autoware_core/map/autoware_map_loader/src/lanelet2_map_loader/`
- `/home/otomasi/ros/autoware/autoware_lanelet2_extension/autoware_lanelet2_extension/lib/autoware_traffic_rules.cpp`
- `/home/otomasi/ros/autoware/autoware_lanelet2_extension/autoware_lanelet2_extension/lib/route_checker.cpp`

### Hasil upgrade
Goal pose sekarang tidak hanya tahu titik tujuan tetapi juga **arah jalan legal**. Nav2 tidak dibuang; semantic route menjadi constraint/mission layer di atas SMAC+MPPI, sehingga kemampuan existing tetap dipakai.
# BAGIAN D — CONTROL: MPPI TETAP, PX4 YAW-RATE DI BAWAHNYA, AUTOWARE GATE PALING AKHIR

## 15. Ackermann yaw-rate closed-loop: ambil dari PX4 rover

### FROM — PX4
- `/home/otomasi/ros/px4/PX4-Autopilot/src/modules/rover_ackermann/AckermannRateControl/AckermannRateControl.cpp`
- `/home/otomasi/ros/px4/PX4-Autopilot/src/modules/rover_ackermann/AckermannSpeedControl/AckermannSpeedControl.cpp`
- `/home/otomasi/ros/px4/PX4-Autopilot/src/modules/rover_ackermann/AckermannActControl/AckermannActControl.cpp`
- `/home/otomasi/ros/px4/PX4-Autopilot/src/lib/pid/PID.cpp`

### TO
- `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`
- `/home/otomasi/ros/src/esc/config/ackermann.yaml`
- Diagnostic consumer `/home/otomasi/ros/src/navigation/src/mppi_closed_loop_supervisor.cpp`

### Posisi algoritma dalam chain
`MPPI v/omega -> cmd_vel_router -> velocity_smoother -> ackermann_controller_server::yawRateController -> finalCommandGate -> LUT/RPM -> F411`.

Artinya PX4-like yaw controller **bukan planner dan bukan pengganti MPPI**. Ia hanya mengubah `omega_request` yang sudah dibuat MPPI menjadi physical steering yang lebih sesuai kendaraan nyata dengan feedback gyro-z.

### Fungsi lokal yang disarankan
- `computeFeasibleYawRate(v, steering_limit)`.
- `updateYawRateSetpointSlew(omega_req, dt)`.
- `computeAckermannFeedForward(v, omega_sp)`.
- `updateYawRatePi(omega_sp, omega_imu, dt)`.
- `closedLoopNavSteeringDeg(selected, dt)`.
- `resetYawRateController(reason)`.

### Rumus inti
`omega_physical_max = abs(v) * tan(delta_max) / wheelbase`. Setpoint `omega` diklem ke minimum antara limit fisik dan configured yaw-rate limit. Feed-forward center steering `delta_ff = atan(wheelbase * omega_sp / v)`. PI hanya menghasilkan koreksi physical steering, lalu hasil akhir tetap melewati physical steering calibration/LUT.
### Anti-windup dan fail-safe yaw controller
- PI aktif awal hanya forward + NAV2/autonomy + gyro fresh + speed feedback fresh.
- Reverse: feed-forward geometry saja sampai gain/sign reverse benar-benar diuji.
- Zero/very-low speed: jangan memaksa Ackermann pure rotation; reset/freeze integral.
- Source change, ESTOP, maintenance, IMU reconnect, dt invalid: reset integral dan slew state.
- Jika steering saturated dan error mendorong lebih jauh ke saturation, integral tidak boleh bertambah.
- `yaw_rate_accel_limit` adalah backstop fisik; tuning dibuat lebih longgar daripada velocity smoother agar tidak menjadi smoothing kedua yang dominan.

### Parameter baru di `src/esc/config/ackermann.yaml`
`yaw_rate_control_enabled`, `yaw_rate_shadow_mode`, `yaw_rate_kp`, `yaw_rate_ki`, `yaw_rate_integral_limit_deg`, `yaw_rate_feedback_output_limit_deg`, `yaw_rate_limit_rps`, `yaw_rate_accel_limit_rps2`, `yaw_rate_gyro_deadband_rps`, `yaw_rate_imu_timeout_sec`, `yaw_rate_speed_feedback_timeout_sec`, `yaw_rate_correction_gain`, `yaw_rate_reverse_feedback_enabled`.

### Shadow mode wajib
Tahap pertama controller menghitung `proposed_steering` tetapi actuator tetap memakai `steeringDegFor()` lama. Bandingkan proposed-vs-existing-vs-actual di log. Authority baru diberikan jika yaw tracking membaik dan steering tidak osilasi.

## 16. Final command gate: safety envelope Autoware paling akhir

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_vehicle_cmd_gate/src/vehicle_cmd_filter.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_control_command_gate/src/common/control_command_filter.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_control_command_gate/src/common/timeout_diagnostics.cpp`

### TO
- `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`
- `/home/otomasi/ros/src/esc/config/ackermann.yaml`

Final gate menerima **physical speed + physical steering candidate** setelah yaw-rate controller. Ia tidak menghitung route, tidak memilih source, dan tidak menggantikan velocity smoother.
### Urutan `applyFinalCommandLimits()`
1. Finite/NaN check.
2. Authority + source freshness + ESTOP/maintenance/fault gate.
3. Hard speed clamp.
4. Acceleration/deceleration safety clamp.
5. Jerk safety clamp.
6. Hard physical steering angle clamp.
7. Physical steering rate clamp.
8. Actual-steering-difference/saturation check.
9. Baru convert steering physical melalui LUT/hysteresis/center calibration.
10. Baru convert drive m/s ke RPM/ERPM target dan kirim ke F411.

### Anti-double-filter rule
Nav2 `velocity_smoother` tetap menentukan kenyamanan/nominal accel-decel. Final gate harus dituning sebagai **safety envelope sedikit lebih longgar** dari smoother. Dengan demikian normal command tidak terus diklem dua kali; gate hanya bekerja saat source spike, stale transition, controller error, atau physical constraint terlampaui.

Likewise, PX4-like yaw setpoint slew adalah feasibility backstop. `final_steer_rate_max` adalah actuator safety hard envelope. Jangan membuat keduanya sama ketat sehingga response steering menjadi lambat dua kali.

### Parameter final gate
`final_gate_enabled`, `final_gate_shadow_mode`, `final_speed_max_mps`, `final_accel_max_mps2`, `final_decel_max_mps2`, `final_jerk_max_mps3`, `final_steer_rate_max_deg_s`, `final_actual_steer_diff_max_deg`, `final_command_timeout_sec`.

### Output diagnostic
`/esc/final_gate/requested_*`, `/esc/final_gate/limited_*`, accel, jerk, steer_rate, clamp reason, source, timestamp. Ini penting supaya jika kendaraan terasa lambat dapat diketahui apakah penyebabnya smoother, yaw controller, final gate, atau actuator.

### Hasil upgrade control
MPPI tetap menentukan intent trajectory; velocity smoother tetap membentuk command nominal; PX4-like loop memperbaiki yaw-rate tracking kendaraan nyata; Autoware-like final gate mencegah command fisik yang tidak aman. Keempatnya berbeda fungsi dan tidak saling menumpuk.
## 17. Control validator: supervisor, bukan controller baru

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_control_validator/src/control_validator.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_control_validator/src/utils.cpp`

### TO
- `/home/otomasi/ros/src/navigation/src/mppi_closed_loop_supervisor.cpp`
- `/home/otomasi/ros/src/navigation/src/trajectory_safety_supervisor.cpp`
- `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`

Tambahkan rolling window dan consecutive-failure counter untuk speed tracking, steering tracking, yaw-rate tracking, command freshness, feedback freshness, path validity, actuator saturation, dan final-gate clamp ratio. Satu frame invalid -> WARN/counter; invalid persisten -> controlled stop + `CONTROL_INVALID`.

Validator **tidak mengoreksi steering**. Koreksi steering hanya dilakukan yaw-rate controller; validator hanya memutuskan apakah autonomous output masih layak dipercaya.

## 18. Steering offset observer: Autoware estimator, observer dulu

### FROM
- `/home/otomasi/ros/autoware/autoware_universe/vehicle/autoware_steer_offset_estimator/src/steer_offset_estimator.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/vehicle/autoware_steer_offset_estimator/src/node.cpp`

### TO / CREATE
- Create `/home/otomasi/ros/src/esc/src/steering_offset_observer.cpp`
- Modify `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`
- Modify `/home/otomasi/ros/src/navigation/config/vehicle.yaml`

Gunakan `delta_expected = atan(L*w/v)` dari fused speed + fresh gyro-z, bandingkan dengan steering actual. Update scalar Kalman/RLS hanya jika speed cukup, gyro/steer fresh, tidak slip, tidak near hard-limit, tidak hard acceleration/turn. Publish candidate offset, covariance, K, residual, confidence, gate reason.

`observer_only=true` pada tahap awal. Jangan otomatis menulis LUT/center/EEPROM. Jika offset berbeda besar antara belok kiri dan kanan, anggap masalah hysteresis/LUT, bukan zero-offset tunggal.
## 19. Steering actuator delay predictor: model observer, bukan controller kedua

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_mpc_lateral_controller/include/autoware/mpc_lateral_controller/steering_predictor.hpp`
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_mpc_lateral_controller/src/steering_predictor.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_mpc_lateral_controller/src/vehicle_model/vehicle_model_bicycle_kinematics.cpp`

### TO
- Create helper/class di package ESC, misalnya `/home/otomasi/ros/src/esc/src/steering_predictor.cpp`
- `/home/otomasi/ros/src/navigation/src/mppi_closed_loop_supervisor.cpp`
- `/home/otomasi/ros/src/navigation/config/mppi_closed_loop.yaml`

Dari step test target-vs-actual identifikasi `dead_time_sec` dan `time_constant_sec`; model orde-1 `d(delta)/dt=(delta_cmd-delta)/tau` dipakai untuk memprediksi physical steering yang benar-benar mungkin tercapai pada horizon pendek.

Output awal hanya `/esc/steering_predicted_rad`, prediction error, delay ms. Predictor boleh dipakai untuk kinematic yaw comparator setelah RMSE predicted-vs-actual terbukti lebih baik daripada raw target-vs-actual. **Actual steering tetap measurement state estimation.**

# BAGIAN E — PERCEPTION SAFETY / AEB

## 20. Depth safety independent dari YOLO

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_autonomous_emergency_braking/src/node.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_autonomous_emergency_braking/src/utils.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/control/autoware_autonomous_emergency_braking/config/autonomous_emergency_braking.param.yaml`

### TO / CREATE
- Create `/home/otomasi/ros/src/perception/src/astra_depth_safety_cloud.cpp`
- Modify `/home/otomasi/ros/src/navigation/src/trajectory_safety_supervisor.cpp`
- Modify `/home/otomasi/ros/src/navigation/config/trajectory_safety.yaml`
- Existing semantic `/home/otomasi/ros/src/perception/src/astra_yolop_cpu_pt_node.cpp` tetap terpisah.
### Proses depth safety
1. Astra depth -> XYZ/PointCloud2 di frame sensor.
2. Transform ke `base_footprint` dengan timestamp yang benar.
3. Reject ground, body robot sendiri, height di luar range, dan ROI belakang yang tidak relevan.
4. Optional voxel downsample ringan agar mini PC tidak terbebani.
5. Publish `/perception/depth_safety_points` walau YOLO OFF.
6. `trajectory_safety_supervisor` membuat predicted footprint/swept corridor dari speed + steering actual/predicted yang tervalidasi.
7. Gunakan **satu** fungsi existing `computeRequiredStopDistance(v, reaction_time, decel)` sebagai sumber braking distance; jangan membuat rumus Autoware kedua yang berbeda.
8. Collision corridor + stop distance/TTC menghasilkan `NORMAL`, `WARN`, atau `HARD_STOP`.
9. `HARD_STOP` masuk safety veto/failsafe state; tidak ada node AEB yang langsung mengirim steering/drive command sendiri.

### Failure mode
- Depth stale -> safety state DEGRADED/FAULT sesuai policy; cloud lama tidak dianggap fresh.
- YOLO OFF -> safety metric tetap aktif.
- Obstacle samping corridor -> tidak false stop.
- Obstacle depan di swept footprint -> WARN/HARD_STOP sesuai distance/TTC.
- Perception node tidak boleh mengambil command authority kecuali sebagai **veto/stop**.

### Hasil upgrade
Safety obstacle tidak lagi bergantung pada keberhasilan AI classification. Geometry depth menjadi safety primer; YOLO tetap memberi semantic class untuk behavior/visualization.
# BAGIAN F — SYSTEM STATE, DIAGNOSTICS, DAN FAILSAFE

## 21. Satu `system_state_manager`: gabungkan PX4 state semantics + Autoware dependency graph

### FROM — PX4
- `/home/otomasi/ros/px4/PX4-Autopilot/msg/FailsafeFlags.msg`
- Vehicle/status message patterns di `/home/otomasi/ros/px4/PX4-Autopilot/msg/`

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_universe/system/autoware_diagnostic_graph_aggregator/src/common/graph/graph.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/system/autoware_diagnostic_graph_aggregator/src/common/graph/logic.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/system/autoware_diagnostic_graph_aggregator/src/common/graph/diags.cpp`

### TO / CREATE
- Create `/home/otomasi/ros/src/navigation/src/system_state_manager.cpp`
- Create `/home/otomasi/ros/src/navigation/config/system_state.yaml`
- Modify `/home/otomasi/ros/src/navigation/src/navigation_core.cpp`
- Modify `/home/otomasi/ros/src/navigation/src/cmd_vel_router.cpp`
- Modify `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`
- Modify `/home/otomasi/ros/src/navigation/web/web_server.cpp`
- Modify `/home/otomasi/ros/src/navigation/launch/autonomous.launch.py`

### State tunggal
`BOOT -> INITIALIZING -> STANDBY -> READY -> AUTONOMOUS|TELEOP|MAINTENANCE`, dengan side state `DEGRADED`, `FAULT`, `ESTOP`. Setiap transition mempunyai previous state, new state, timestamp, reason, dan authority owner.

### Dependency graph tunggal
`SENSOR_READY`, `LOCALIZATION_READY`, `CONTROL_READY`, `ACTUATOR_READY`, `SAFETY_READY` dihitung dari typed leaf status + DiagnosticStatus. `SYSTEM_MOTION_READY = required readiness AND !ESTOP AND allowed mode`.

Web/HMI tidak menghitung READY sendiri. Mereka hanya render `/system/state`, `/system/readiness`, dan raw leaf detail.
### Leaf status minimum
- GNSS: link, config, fix, fusion-qualified, integrity, timestamp/rate.
- IMU: stream, gyro freshness, orientation freshness, calibration, rate/jitter.
- MAG: link, calibration, disturbance.
- Localization: local EKF, global pose, map->odom TF, timestamp integrity, dead-reckoning.
- Planning/control: semantic router, SMAC, MPPI, trajectory safety, control validator.
- Actuator: F411, F103/VESC left/right, steering encoder, drive feedback, final command freshness.
- Transport: 65101 owner, 65102 connection, RT/APP rate, packet loss, watchdog.
- Safety: global/teleop/perception ESTOP, depth stale, control invalid.

### Hysteresis/latch policy
Autoware-like timeout/hysteresis dipakai supaya satu packet terlambat tidak membuat ONLINE/OFFLINE chatter. Fault VESC tertentu boleh latch sampai clear. GNSS FIX loss dapat membuat global capability DEGRADED tanpa otomatis mematikan seluruh monitoring. Python maintenance harus menjadi `MAINTENANCE`, bukan `FAULT`.

### Authority priority final
1. ESTOP / hard safety veto.
2. Python maintenance TCP 65101 untuk exclusive command/config transaction.
3. VESC Tool 65102 untuk monitoring/config sesuai permission; koneksi monitor tidak otomatis MAINTENANCE.
4. TELEOP/HMI/Web manual.
5. Safety intervention/veto.
6. Nav2 autonomy.

`cmd_vel_router.cpp` memilih command source berdasarkan state/permission; `ackermann_controller_server.cpp` tetap memiliki independent last-line stale/ESTOP check. Dengan demikian bila state manager crash, actuator tidak otomatis terus menjalankan command lama.

### Hasil upgrade
Tidak ada lagi status `READY` yang berbeda antara ROS Web, NavigationCore, ESC, dan HMI. Semua berasal dari satu state machine dengan dependency graph, tetapi actuator tetap punya fail-safe lokal paling akhir.
## 22. Processing-time monitor: Autoware metrics + PX4 link timing

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_universe/system/autoware_processing_time_checker/src/processing_time_checker.cpp`
- `/home/otomasi/ros/autoware/autoware_universe/system/autoware_processing_time_checker/config/processing_time_checker.param.yaml`

### TO / CREATE
- Create `/home/otomasi/ros/src/navigation/src/processing_time_monitor.cpp`
- Instrument `/home/otomasi/ros/src/navigation/src/localization_core.cpp`
- Instrument `/home/otomasi/ros/src/navigation/src/navigation_core.cpp`
- Instrument `/home/otomasi/ros/src/navigation/src/trajectory_safety_supervisor.cpp`
- Instrument `/home/otomasi/ros/src/perception/src/astra_yolop_cpu_pt_node.cpp`
- Instrument `/home/otomasi/ros/src/esc/src/vesc_tool_bridge.cpp`
- Instrument `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`

Setiap producer publish duration/latency terukur; monitor menyimpan current, min, max, mean, P50/P95/P99, count. Pisahkan empat hal: **processing duration**, **message age**, **inter-arrival jitter**, dan **end-to-end latency**.

Channel wajib: GNSS processing, IMU processing, localization correction, semantic route, SMAC goal-to-first-plan, trajectory safety, YOLO preprocess/inference/postprocess, ESC request->ACK, RT/APP inter-arrival, final command pipeline.

Untuk RT 50 Hz, interval ideal sekitar 20 ms; APP 20 Hz sekitar 50 ms. Threshold readiness harus berbasis statistik age/jitter/latency, bukan satu timeout besar yang menyembunyikan lag.

### Output
`/system/processing_metrics` + `/debug/processing_time/<channel>_ms`. Web tampilkan current + P95/P99; logger menyimpan summary trial.
# BAGIAN G — VESC TRANSPORT, TCP 65101/65102, DAN SINGLE OWNER

## 23. Router transport: ambil arsitektur mavlink-router, tetap protocol VESC

### FROM — PX4 ecosystem
- `/home/otomasi/ros/px4/mavlink-router/src/mavlink-router/mainloop.cpp`
- `/home/otomasi/ros/px4/mavlink-router/src/mavlink-router/endpoint.cpp`
- `/home/otomasi/ros/px4/mavlink-router/src/mavlink-router/pollable.cpp`
- `/home/otomasi/ros/px4/mavlink-router/src/mavlink-router/timeout.cpp`

### TO
- `/home/otomasi/ros/stm32f401/src/VescGateway.cpp`
- `/home/otomasi/ros/stm32f401/src/VescGateway.h`
- `/home/otomasi/ros/src/stmf4/src/stmf4_hmi_bridge.cpp`
- `/home/otomasi/ros/src/esc/src/vesc_tool_bridge.cpp`
- `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`
- `/home/otomasi/ros/vesc_tool/tcpserversimple.cpp`
- `/home/otomasi/ros/vesc_tool/tcphub.cpp`

### Logical endpoint final
- `RUNTIME_ROS`.
- `PYTHON_MAINTENANCE_65101`.
- `VESC_TOOL_65102`.
- `LOGGER_HEALTH` read-only.

Satu physical UART owner membaca byte sekali. Logical endpoint mempunyai bounded RX/TX queue, permission, last activity, transaction owner, timeout, bytes/frames, drop/backpressure counter. Queue penuh **drop terukur**, bukan memblokir 50 Hz control loop.

### Routing rule
Periodic telemetry boleh dibagikan ke ROS/logger/VESC Tool. Response transaksi config/detect kembali hanya ke transaction owner. Python 65101 mengambil exclusive command/config ownership. VESC Tool 65102 monitoring tidak mengambil MAINTENANCE. Physical link loss membuat semua endpoint unavailable dan actuator fail-closed.
### Timeout/reconnect rule
- Endpoint TCP timeout hanya menutup endpoint itu, bukan reset UART fisik bila F411/F103 sehat.
- Python disconnect ketika memegang transaction: safe-stop bila perlu, release owner, lalu revalidate READY.
- VESC Tool disconnect: tidak mengubah READY bila hanya monitor.
- Config transaction boleh timeout beberapa detik tanpa menghentikan RT telemetry.
- RT/APP field freshness memakai age pendek sesuai rate; jangan menaikkan semua timeout menjadi 10 s untuk menghilangkan symptom lag.

### Statistik yang wajib terlihat
Per endpoint: connected, mode, owner, RX/TX bytes/s, frames/s, queue depth/max, drop, transaction timeout, malformed/CRC, reconnect. Per physical link: baud, RT/APP observed rate, jitter/loss, request->ACK latency, watchdog, reset counter, firmware/protocol version.

### Concurrency acceptance
Jalankan ROS autonomous/teleop + VESC Tool 65102 monitor + Python 65101 connect/disconnect/config transaction. PASS jika tidak ada serial contention, RT 50 Hz collapse, APP 20 Hz collapse, timeout storm, reconnect storm, atau actuator command dari non-owner.

### Hasil upgrade
65101 dan 65102 tidak lagi diperlakukan sebagai koneksi yang 'berebut serial'. Mereka menjadi logical clients di atas satu transport owner dengan hak akses eksplisit.

# BAGIAN H — TYPED DATA CONTRACT

## 24. Buat package lokal `agv_msgs`

### CREATE
- `/home/otomasi/ros/src/agv_msgs/msg/AgvGnssStatus.msg`
- `/home/otomasi/ros/src/agv_msgs/msg/AgvImuStatus.msg`
- `/home/otomasi/ros/src/agv_msgs/msg/AgvOdometryStatus.msg`
- `/home/otomasi/ros/src/agv_msgs/msg/AgvEstimatorStatus.msg`
- `/home/otomasi/ros/src/agv_msgs/msg/AgvControllerStatus.msg`
- `/home/otomasi/ros/src/agv_msgs/msg/AgvLinkStatus.msg`
- `/home/otomasi/ros/src/agv_msgs/msg/AgvFailsafeStatus.msg`

Jangan bergantung langsung pada `px4_msgs` atau `autoware_msgs`; keduanya hanya referensi desain. Typed messages publish paralel dengan String/Float64MultiArray lama sampai seluruh consumer sudah migrasi.
# BAGIAN I — EVALUATOR, LOGGING, DAN BAB IV

## 25. Metrics: Autoware evaluator + PX4 self-describing trial

### FROM — Autoware
- `/home/otomasi/ros/autoware/autoware_universe/evaluator/autoware_localization_evaluator/src/`
- `/home/otomasi/ros/autoware/autoware_universe/evaluator/autoware_control_evaluator/src/`
- `/home/otomasi/ros/autoware/autoware_universe/evaluator/autoware_planning_evaluator/src/`

### FROM — PX4 ecosystem
- `/home/otomasi/ros/px4/pyulog/pyulog/core.py`
- `/home/otomasi/ros/px4/pyulog/pyulog/ulog2csv.py`
- `/home/otomasi/ros/px4/flight-review-rs/`

### TO / CREATE
- Create `/home/otomasi/ros/src/navigation/src/localization_evaluator.cpp`
- Create `/home/otomasi/ros/src/navigation/src/control_evaluator.cpp`
- Optional create `/home/otomasi/ros/src/navigation/src/planning_evaluator.cpp`
- Modify `/home/otomasi/ros/src/navigation/tools/export_trial_artifacts.py`
- Modify `/home/otomasi/ros/src/navigation/tools/rosbag_regression.py`
- Modify `/home/otomasi/ros/src/navigation/gui/agv_experiment_catalog.hpp`
- Modify `/home/otomasi/ros/src/navigation/gui/modules/reporting_widgets.cpp`
- Modify `/home/otomasi/ros/src/navigation/web/static/export_tools.js`

### Metric localization
RMSE X/Y/position/yaw, mean/max/std/P95, convergence time, GNSS correction magnitude, covariance consistency, reject/innovation summary, stationary drift.

### Metric control
CTE, heading error, speed RMSE, steering RMSE, yaw-rate RMSE, settling time, overshoot, steering oscillation, accel/jerk peak, command->actuator latency, clamp ratio.
### Metric planning/safety
Semantic route latency/length, start/goal snap distance, corrected yaw, `legal_route_ratio`, `wrong_way_segment_count`, route discontinuity; SMAC planning latency/path length/curvature/clearance; MPPI tracking/clearance/TTC; safety minimum TTC/required decel/HARD_STOP count.

### Manifest setiap run
Wajib simpan run ID, date/time, git revision workspace, firmware F411/F103 hash/version, YAML snapshot, sensor identity/port/baud, calibration IDs, experiment/subbab, ground-truth method, operator notes, start/stop reason.

Timeseries minimal menyimpan `timestamp`, `measurement_timestamp`, source, validity, raw measurement, target/actual, state/failsafe, link age/jitter/loss. Rosbag tetap raw source utama; CSV/XLSX/PNG/JSON adalah export terstruktur.

### Satu sumber angka
Web, GUI, CSV, XLSX, PNG, JSON harus mengambil summary evaluator yang sama. Jangan menghitung RMSE satu kali di Web dan formula lain di exporter karena angka Bab IV akan berbeda.

# BAGIAN J — PATCH PLAN FILE-PER-FILE

## 26. File aktif yang sudah diverifikasi dan titik masuk upgrade

### `/home/otomasi/ros/src/esc/src/ackermann_controller_server.cpp`
Kode sekarang sudah mempunyai `selectCommand()`, `clampTwist()`, `operationalPhysicalLimitDeg()`, `clampPhysicalSteeringDeg()`, `steeringDegFor()`, LUT increasing/decreasing, dan physical-to-STM mapping. Maka upgrade **tidak boleh merombak calibration pipeline**.

Patch target:
1. Tambah IMU gyro + measured speed subscriptions/state.
2. Tambah yaw-rate controller helper sebelum `steeringDegFor()` output final untuk NAV2, dengan shadow mode.
3. Refactor `steeringDegFor()` agar geometric feed-forward dapat dipakai sebagai fallback/helper.
4. Tambah `applyFinalCommandLimits()` setelah physical steering candidate terbentuk dan sebelum LUT/STM conversion.
5. Tambah odom reset/status, typed controller status, processing-time instrumentation.
6. Pertahankan existing `selectCommand()` arbitration; high-level permission nanti dikaitkan ke `/system/state` tanpa membuat mux kedua.
### `/home/otomasi/ros/src/navigation/src/navigation_core.cpp`
Current code sudah memiliki `/navigation/goal_request` subscription (`onGoal()`), queued-goal flow, `trySendQueuedGoal()`, Nav2 lifecycle handling, dan velocity smoother lifecycle guard. Maka semantic routing harus masuk **di antara `onGoal()` dan send action**, bukan dibuat sebagai goal subscriber kedua yang berkompetisi.

Patch target:
1. Link library/class `OsmRoutePlanner`.
2. `onGoal()` tetap satu entry-point dari Web/RViz/HMI.
3. Setelah readiness/finite/map check, buat route preview/legal plan.
4. Simpan `queued_user_goal_` terpisah dari `queued_route_` dan `route_id_`.
5. Tambah action client `nav2_msgs::action::NavigateThroughPoses`.
6. Cancel/replace harus mengetahui action aktif `NavigateToPose` atau `NavigateThroughPoses`.
7. State route: `QUEUED -> ROUTING -> ROUTE_READY -> SENDING_ROUTE -> ACTIVE_ROUTE -> FINAL_APPROACH -> SUCCEEDED`.
8. `autonomousMotionReadyUnlocked()` nantinya consume `/system/readiness`, bukan merakit semua raw status sendiri.

### `/home/otomasi/ros/src/navigation/tools/osm_pgm.py`
Pertahankan generator PGM existing. Tambah parser road-properties yang reusable dan mode `--route-graph-only`. Graph output deterministic, memakai map-meter XY, dan validator menolak edge length zero, bad node reference, illegal reverse one-way, disconnected/invalid topology sesuai policy.

### `/home/otomasi/ros/src/navigation/src/localization_core.cpp`
Jangan mengganti pipeline GNSS/EKF besar. Upgrade hanya: typed GNSS/IMU status consumer, true measurement stamp gate, fused twist consumer, unified `/vehicle/stopped`, odom reset handling, external consistency monitor, standardized diagnostics/evaluator hooks.

### `/home/otomasi/ros/src/navigation/src/mppi_closed_loop_supervisor.cpp`
Tidak menjadi controller baru. Tambah control-validation window, consecutive faults, predicted steering comparator, saturation/clamp ratio, speed/steer/yaw-rate RMSE live.

### `/home/otomasi/ros/src/navigation/src/trajectory_safety_supervisor.cpp`
Existing stop-distance logic tetap source-of-truth. Tambah depth cloud + swept-footprint collision; hasil tetap satu safety state/veto.
### `/home/otomasi/ros/src/esc/src/vesc_tool_bridge.cpp`
Current code sudah mempunyai timer untuk runtime transition, polling, TCP, `pythonTcpTick()`, setup 65101/65102, dan publish mode/owner. Jangan menambah bridge baru. Refactor ownership menjadi satu enum/state machine eksplisit dengan per-endpoint bounded queue/statistics.

Patch target:
1. Pisahkan physical-link state dari logical-client state.
2. Python 65101 = exclusive transaction owner ketika command/config/detect berjalan.
3. VESC Tool 65102 = monitor by default; config write dapat meminta temporary transaction permission sesuai policy.
4. Endpoint disconnect tidak me-reset healthy physical link.
5. Poll/telemetry tetap non-blocking selama config transaction.
6. Publish `AgvLinkStatus` dan raw rate/jitter/drop counters.

### `/home/otomasi/ros/stm32f401/src/VescGateway.cpp` + `/home/otomasi/ros/stm32f401/src/VescGateway.h`
Jadikan scheduler F411 deterministic: command/RT 50 Hz, APP 20 Hz, HEALTH 5–10 Hz. Tambah sequence/timestamp/class/version; bounded TX queues; no long blocking wait. Python/VESC logical ownership dari PC diterjemahkan ke transaction routing tanpa menghambat periodic telemetry.

### `/home/otomasi/ros/stm32f401/src/Neo3Sensors.cpp` + `/home/otomasi/ros/stm32f401/src/Neo3Sensors.h`
GNSS: sequence+iTOW+receive time + health counters. IST: explicit non-blocking state machine, DRDY/I2C/config integrity. Sensor failure tidak boleh menahan VESC gateway loop.

### `/home/otomasi/ros/src/stmf4/src/stmf4_hmi_bridge.cpp`
Bridge harus preserve measurement time dari MCU, bukan overwrite semua dengan `now()`. Publish typed link/GNSS/mag health. Ia mirror system state ke HMI tetapi bukan owner autonomous safety decision.

### `/home/otomasi/ros/src/navigation/web/web_server.cpp` + `web/static/app.js`
Web menjadi visualizer/control front-end: route preview, road direction arrows, requested/legal yaw, readiness graph, processing P95/P99, typed sensor health, link owner, controller requested/limited/actual. Jangan menghitung semantic graph, EKF quality, atau READY sendiri di JavaScript.

### `/home/otomasi/ros/src/navigation/launch/autonomous.launch.py`
Node baru masuk bertahap lewat feature flag. Observer-only nodes boleh default ON setelah build stabil; module yang dapat memblokir motion default shadow/disabled sampai certified.
## 27. File baru yang memang perlu dibuat — jangan lebih dari ini tanpa alasan

| File baru | Fungsi tunggal |
|---|---|
| `src/navigation/src/osm_route_planner.cpp` | directed legal road routing; tidak publish actuator |
| `src/navigation/config/osm_route_planner.yaml` | parameter graph/snap/yaw/legal-route |
| `src/navigation/maps/undip/undip_nav2_route_graph.json` | runtime semantic graph |
| `src/navigation/src/vehicle_twist_fusion.cpp` | ESC vx + IMU wz + stationary clamp |
| `src/navigation/src/system_state_manager.cpp` | readiness graph + vehicle state/failsafe owner |
| `src/navigation/config/system_state.yaml` | required leaves, timeout, hysteresis/latch |
| `src/navigation/src/processing_time_monitor.cpp` | aggregate processing/age/jitter metrics |
| `src/navigation/src/localization_evaluator.cpp` | localization metrics |
| `src/navigation/src/control_evaluator.cpp` | control metrics |
| `src/perception/src/astra_depth_safety_cloud.cpp` | metric obstacle cloud independent YOLO |
| `src/esc/src/steering_offset_observer.cpp` | dynamic zero candidate observer |
| `src/esc/src/steering_predictor.cpp` | physical steering delay model |
| `src/agv_msgs/msg/*.msg` | typed project-specific contracts |

**Tidak perlu membuat:** controller Ackermann node kedua, AEB actuator node kedua, command mux kedua, GNSS failover node dulu, Autoware message adapter besar, PX4 DDS runtime, PX4 EKF runtime, Lanelet2 runtime P0.

## 28. CMake/package dependency minimum

`navigation`: tambah `diagnostic_msgs`, `nlohmann_json`, dan `agv_msgs` saat source masuk. Existing `rclcpp_action/nav2_msgs/nav_msgs/visualization_msgs/yaml-cpp` tetap dipakai. `OsmRoutePlanner` paling baik sebagai library yang dilink `navigation_core`, supaya satu owner `/navigation/goal_request`.

`esc`: yaw controller/final gate tidak butuh Autoware/PX4 dependency. Tambah `agv_msgs` dan `diagnostic_msgs` hanya untuk typed/status publish. Jangan link Autoware Universe ke actuator package.

`perception`: depth safety gunakan ROS sensor/point cloud stack existing; hindari dependency perception Autoware berat di mini PC.

`agv_msgs`: package interface ringan tanpa business logic.
# BAGIAN K — URUTAN IMPLEMENTASI YANG PALING AMAN

## 29. Stage 0 — baseline dan freeze measurement
Simpan git status/diff, firmware identity, YAML snapshot, 5–10 menit baseline GNSS/IMU/ESC/link, RT/APP rate, Nav2 tracking, CPU. Tidak reset/revert perubahan user. Semua perbandingan upgrade harus memakai baseline ini.

## 30. Stage 1 — data contract + sensor timing/health, tanpa mengubah fusion
Buat `agv_msgs`; upgrade IMU/GNSS/F411 timestamp, rate/jitter, parser counters, one-GNSS-authority. Legacy topic tetap publish paralel. PASS bila numeric sensor output tidak berubah salah dan tidak ada duplicate authority.

## 31. Stage 2 — frame/covariance + odom reset semantics
Buat central frame transform + unit test, migrasikan GNSS NED/ENU/covariance/COG, audit IMU/mag mounting sign, tambah odom reset counter. Belum mengubah controller.

## 32. Stage 3 — transport/timesync observability
Tambah sequence/time/class pada F411/F103 telemetry, endpoint/link stats, proof RT 50 Hz + APP 20 Hz. Refactor 65101/65102 ownership tanpa mengubah motor tuning. PASS concurrency dahulu.

## 33. Stage 4 — system state/readiness tunggal
Aktifkan `system_state_manager` observer, bandingkan dengan existing status. Setelah cocok, NavigationCore/cmd router/Web/HMI migrasi consume source tunggal. Actuator fail-safe lokal tetap dipertahankan.

## 34. Stage 5 — semantic OSM router observer
Generate route graph dari OSM lokal tanpa mengubah PGM/YAML. Preview legal route, road arrows, wrong-way detection, route latency. Existing NavigateToPose tetap authority selama observer.

## 35. Stage 6 — semantic route authority
Aktifkan `require_legal_route=true` dan `NavigateThroughPoses`. Validate cancel/replace/final approach. SMAC+MPPI tetap tidak berubah.

## 36. Stage 7 — vehicle twist fusion + stop filter observer -> EKF
Bandingkan fused ESC-vx/gyro-wz terhadap raw input. Setelah timestamp/sign/covariance PASS, masukkan ke local EKF satu perubahan pada satu waktu; jangan sekaligus retune seluruh Q/R.
## 37. Stage 8 — PX4-like yaw-rate controller shadow -> low-speed authority
Tambahkan gyro/speed feedback, feasible yaw-rate, FF+PI anti-windup. Shadow dahulu; bandingkan geometric-only vs proposed. Jika yaw tracking membaik tanpa steering oscillation, enable forward/autonomy pada speed cap rendah. Reverse tetap FF sampai diuji terpisah.

## 38. Stage 9 — Autoware-like final gate shadow -> authority
Publish requested vs limited speed/steering/accel/jerk/rate. Pastikan normal command hampir tidak diklem; tune envelope lebih longgar daripada nominal smoother. Setelah PASS, enable final gate. E-stop/stale/maintenance precedence diuji ulang.

## 39. Stage 10 — control validator + predictor + steering offset observer
Validator boleh block autonomy setelah consecutive fault threshold. Steering predictor dan dynamic offset tetap observer. Predictor baru dipakai model comparator bila RMSE membaik; offset baru boleh apply runtime setelah multi-run confidence stabil, persist paling akhir.

## 40. Stage 11 — depth safety/AEB observer -> HARD_STOP authority
Depth cloud dulu hanya visual/metrics, lalu WARN, terakhir HARD_STOP. Uji obstacle depan/samping/belakang, steering kiri/kanan, YOLO OFF, stale/noisy depth, false positive. AEB hanya veto.

## 41. Stage 12 — evaluator, fault injection, regression certification
Integrasikan localization/control/planning evaluator dengan manifest run. Buat test dropout GNSS, IMU stale, GNSS jump, mag disturbance, steering delay/backlash, wheel slip, F411/F103 reset, packet loss/jitter, 65101 takeover, 65102 monitoring, ESTOP pada semua mode.

## 42. Lanelet2 hanya Stage future
Migrasi ke Lanelet2 jika directed OSM sudah stabil tetapi requirement semantic nyata tidak dapat diwakili centerline+tags. Occupancy PGM tetap dipertahankan untuk Nav2 costmap.

# BAGIAN L — BUILD DAN VALIDASI

## 43. Build policy 50%
Mini PC 4C/4T: gunakan maksimal 2 parallel worker.

```bash
cd /home/otomasi/ros
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS='-j2 -l2'
colcon build --packages-select <package-yang-berubah> --parallel-workers 2
```

Jangan build seluruh workspace setiap patch kecil. `agv_msgs` dibuild lebih dulu sebelum package consumer.
## 44. Acceptance per subsistem

### Sensor/time
PASS: measurement stamps monotonic, stale field tidak dipakai ulang, GNSS hanya satu authority, direct/F411 frame convention sama, IMU gyro sign REP-103 benar, rate/jitter/counter terlihat.

### Localization
PASS: fused twist timestamp aligned, stationary clamp tidak chatter, odom reset tidak teleport, external residual menjelaskan GNSS/gyro/mag outlier, EKF tidak menerima stale sample bertimestamp palsu.

### Semantic routing
PASS: one-way tidak pernah dilawan, disconnected route ditolak, wrong-way yaw dikoreksi/ditolak sesuai policy, cancel/replace aman, final user goal tetap tercapai, `wrong_way_segment_count=0`.

### Control
PASS: yaw-rate proposed/active menurunkan tracking error dibanding geometric-only; no sustained steering oscillation; reverse feedback OFF sampai certified; final gate tidak membuat normal control sluggish; hard physical limit tidak pernah terlampaui.

### Safety
PASS: depth safety aktif saat YOLO OFF; obstacle corridor memicu state benar; obstacle samping tidak false stop; stale depth dikenali; HARD_STOP selalu mengalahkan autonomy/manual motion sesuai policy safety.

### Transport
PASS: MiniPC->F411 1M dan F411->F103 1M stabil; RT 50 Hz, APP 20 Hz; 65101/65102 concurrency tidak membuat timeout storm; physical serial hanya satu owner; reconnect tidak menghasilkan command stale.

### System state
PASS: setiap transition punya reason/timestamp; Python 65101 -> MAINTENANCE dan selesai -> revalidate READY; VESC Tool monitor boleh tetap READY; sensor/fault recovery mengikuti hysteresis; ESTOP zero-command terbukti.

### Evaluator
PASS: Web/GUI/export memakai summary yang sama, START/STOP trial tidak bocor data antar run, manifest menyimpan source revision/calibration/config, angka RMSE/P95 dapat direproduksi dari rosbag.

## 45. Definition of Done global
Upgrade baru boleh disebut selesai jika build/test PASS **dan behavior runtime PASS**. Copy source atau colcon build sukses saja tidak cukup. Tidak boleh ada timeout/frequency regression, command bypass, double publisher, double controller, double mux, atau status READY ganda.
# BAGIAN M — KEPUTUSAN KEEP / MERGE / REJECT / DEFER

## 46. Eliminasi duplikasi algoritma

| Kandidat | Keputusan final | Penjelasan |
|---|---|---|
| Nav2 SMAC Hybrid-A* | KEEP | Planner geometrik utama; semantic route hanya memberi corridor/through-poses |
| Nav2 MPPI | KEEP | Local trajectory controller utama |
| Nav2 velocity_smoother | KEEP | Smoother nominal utama untuk v/omega |
| PX4 AckermannRateControl | PORT | Lower-level yaw-rate feedback setelah smoother, bukan planner/controller trajectory baru |
| PX4 AckermannAttControl | REJECT dari jalur Nav2 | Outer heading loop akan menumpuk dengan MPPI; hanya opsional test/heading-hold terpisah |
| PX4 AckermannSpeedControl PI | DEFER/mostly reject | VESC/F103 sudah punya motor speed loop; hanya accel/decel pattern + optional slow trim nanti |
| PX4 AckermannActControl slew | MERGE | Physical actuator slew dimasukkan ke final gate, bukan node/filter terpisah |
| Autoware vehicle/control command gate | PORT | Menjadi satu final safety envelope setelah yaw controller |
| Autoware MPC lateral controller | REJECT sebagai controller | Jangan mengganti MPPI; hanya `SteeringPredictor`/vehicle model insight yang dipakai |
| Autoware control validator | PORT | Supervisor readiness/controlled stop, tidak menghasilkan steering |
| Autoware diagnostic graph | MERGE | Dependency logic masuk `system_state_manager` |
| PX4 failsafe flags/state | MERGE | State/mode semantics masuk `system_state_manager` yang sama |
| Autoware processing checker | MERGE | ROS processing metrics digabung dengan PX4-like link timing, satu dashboard metrics |
| PX4 EKF/ECL | REJECT runtime | `robot_localization` existing tetap EKF; PX4 hanya innovation/status discipline |
| Autoware localization stack penuh | REJECT runtime | Tidak perlu mengganti local/global localization existing |
| Autoware gyro odometer | PORT | Menjadi `vehicle_twist_fusion`, memakai timestamp/quality PX4-like |
| Autoware stop filter | MERGE | Existing stationary/ZUPT lebih kaya; Autoware hanya final zero clamp |
| Autoware GNSS poser | MERGE | Fusion quality/frame/covariance pattern; UBX driver tetap lokal/PX4-like |
| Autoware AEB | MERGE | Swept-path/TTC pattern masuk existing trajectory safety; tidak membuat actuator node kedua |
| Existing braking-distance formula | KEEP | Satu source-of-truth stop distance; jangan duplikasi rumus Autoware |
| Autoware Lanelet2 penuh | DEFER | Directed OSM graph lebih ringan untuk kebutuhan one-way saat ini |
| PX4 Micro-XRCE-DDS | REJECT sekarang | Custom VESC 1M dipertahankan; ambil rate/topic discipline saja |
| `px4_msgs`/`autoware_msgs` runtime | REJECT | Buat `agv_msgs` agar ABI/semantic project tetap independen |
# BAGIAN N — MASTER FROM -> TO MATRIX

## 47. Mapping sumber fungsi ke fungsi lokal

| FROM source/function | TO lokal | Tindakan | Hasil |
|---|---|---|---|
| Autoware `DefaultPlanner::plan()` | `osm_route_planner.cpp::planLegalRoute()` | PORT algorithm | route directed legal |
| Autoware nearest lane/centerline | `findNearestLegalEdge()/projectToRoadCenterline()` | PORT algorithm | snap start/goal legal |
| Autoware RouteHandler topology | `osm_pgm.py::buildDirectedRouteGraph()` | PORT concept | adjacency one-way/two-way |
| Autoware `is_goal_valid()` | `validateRoadGoal()` | PORT algorithm | wrong-way/offroad/disconnected rejection |
| Autoware mission route | `navigation_core.cpp` `NavigateThroughPoses` flow | ADAPT | semantic mission -> Nav2 execution |
| PX4 `AckermannRateControl` | `ackermann_controller_server.cpp` yaw helper | PORT algorithm | gyro-feedback steering correction |
| PX4 PID anti-windup | yaw-rate PI internal | PORT algorithm | no integral windup on saturation/stale |
| PX4 actuator slew | final gate steering rate | MERGE | one physical actuator slew owner |
| Autoware command filter | `applyFinalCommandLimits()` | PORT algorithm | speed/accel/jerk/steer envelope |
| Autoware control validator | `mppi_closed_loop_supervisor.cpp` | PORT pattern | persistent invalid -> controlled stop |
| Autoware steering offset | `steering_offset_observer.cpp` | PORT algorithm | candidate dynamic center correction |
| Autoware steering predictor | `steering_predictor.cpp` | PORT algorithm | model physical actuator lag |
| PX4 `VehicleIMU` health | `imu_node.cpp` | PORT pattern | sample timestamp/rate/jitter/clipping |
| PX4 UBX driver integrity | `gnss_node.cpp` + `Neo3Sensors.cpp` | PORT pattern | epoch/checksum/config/integrity diagnostics |
| Autoware GNSS Poser gate | `localization_core.cpp` | MERGE | covariance/freshness/frame fusion qualification |
| PX4 frame transforms | central `frame_transforms.*` | PORT library concept | one sign/frame implementation |
| PX4 odom reset semantics | ESC odom + LocalizationCore | PORT pattern | reboot/reset without pose jump |
| Autoware gyro odometer | `vehicle_twist_fusion.cpp` | PORT algorithm | ESC vx + gyro wz measurement |
| Autoware stop filter | twist fusion stationary clamp | MERGE | no odom creeping at stop |
| PX4 EstimatorInnovations | LocalizationCore external consistency | ADAPT | explain measurement residual/reject |
| Autoware AEB swept path | `trajectory_safety_supervisor.cpp` | PORT algorithm | depth corridor collision veto |
| Autoware evaluator | evaluator nodes/tools | PORT metrics | repeatable localization/control/planning metrics |
| PX4/ULog logging pattern | trial manifest/exporter | PORT pattern | self-describing Bab IV run |
| FROM source/function | TO lokal | Tindakan | Hasil |
|---|---|---|---|
| PX4 `FailsafeFlags` semantics | `system_state_manager.cpp` | PORT state model | one authoritative vehicle state |
| Autoware diagnostic graph | `system_state_manager.cpp` readiness graph | MERGE | dependency READY + timeout/hysteresis/latch |
| Autoware processing checker | `processing_time_monitor.cpp` | PORT metrics | current/min/max/P95/P99 processing |
| PX4 TimesyncStatus pattern | F411/bridge/link status | PORT pattern | packet age/offset/RTT/jitter |
| mavlink-router Mainloop/Endpoint | `vesc_tool_bridge.cpp` + F411 gateway | PORT architecture | one physical owner, many logical endpoints |
| PX4 DDS topic/rate discipline | VESC Telemetry scheduler | PORT concept | RT50/APP20/HEALTH5-10 deterministic classes |
| PX4 IST8310 state machine | `Neo3Sensors.cpp` | PORT pattern | non-blocking mag health/recovery |

## 48. Local file action summary

### MODIFY — critical path
- `src/navigation/src/navigation_core.cpp`: semantic route integration + state-manager readiness consumer.
- `src/navigation/src/localization_core.cpp`: measurement stamps, typed status, fused twist, reset/innovation/evaluator hooks.
- `src/navigation/src/imu_node.cpp`: sample timestamps + health.
- `src/navigation/src/gnss_node.cpp`: UBX integrity + typed health.
- `src/navigation/src/cmd_vel_router.cpp`: consume authoritative vehicle state/permissions.
- `src/navigation/src/mppi_closed_loop_supervisor.cpp`: validator/predictor metrics.
- `src/navigation/src/trajectory_safety_supervisor.cpp`: swept depth safety.
- `src/esc/src/ackermann_controller_server.cpp`: yaw-rate controller + final gate + reset/status.
- `src/esc/src/vesc_tool_bridge.cpp`: single-owner logical endpoint router.
- `src/stmf4/src/stmf4_hmi_bridge.cpp`: preserve MCU sample time + typed link/sensor health.
- `stm32f401/src/VescGateway.cpp` + `stm32f401/src/VescGateway.h`: deterministic rate classes + bounded queues/sequence/time.
- `stm32f401/src/Neo3Sensors.cpp` + `stm32f401/src/Neo3Sensors.h`: GNSS/MAG time/health state.
- `src/navigation/tools/osm_pgm.py`: directed graph output without breaking PGM generation.

### KEEP UNCHANGED IN ROLE
`SMAC`, `MPPI`, `velocity_smoother`, robot_localization EKF, steering LUT/calibration, VESC/F103 motor loop, occupancy map.
# BAGIAN O — SOP IMPLEMENTASI PRAKTIS

## 49. Sebelum patch satu stage

```bash
cd /home/otomasi/ros
git status --short
git diff -- src/navigation src/esc src/stmf4 stm32f401 hoverboard-firmware-hack-FOC > /tmp/pre_upgrade.diff
sha256sum src/navigation/maps/undip/undip_nav2_raw.osm \
          src/navigation/maps/undip/undip_nav2.pgm \
          src/navigation/maps/undip/undip_nav2.yaml
```

Jangan reset/revert workspace. Tujuan baseline hanya untuk mengetahui file mana yang disentuh stage tersebut.

## 50. Setelah semantic graph generator selesai dibuat

```bash
cd /home/otomasi/ros
python3 src/navigation/tools/osm_pgm.py \
  --input-osm src/navigation/maps/undip/undip_nav2_raw.osm \
  --route-graph-only \
  --output-dir src/navigation/maps/undip
```

PGM/YAML/raw OSM checksum harus tetap sama. Validator graph harus non-zero exit bila topology invalid.

## 51. Runtime observer commissioning
Semua feature yang berpotensi mengubah behavior pertama kali harus punya `enabled` + `shadow_mode/enforce` terpisah. Urutan observer: semantic route, processing monitor, state manager, fused twist, yaw controller, final gate, steering predictor, offset observer, depth safety.

Production authority baru diberikan satu-per-satu; jangan enable semantic route + yaw PI + final gate + AEB sekaligus karena bila behavior berubah tidak diketahui penyebabnya.
## 52. Topic/metric yang harus dipantau saat commissioning

```bash
# Route
ros2 topic echo /navigation/semantic_route/status
ros2 topic echo /navigation/semantic_route --once
ros2 topic echo /navigation/goal_state

# State/readiness
ros2 topic echo /system/state
ros2 topic echo /system/readiness
ros2 topic echo /diagnostics

# Twist/localization
ros2 topic echo /vehicle/twist_fusion/status
ros2 topic echo /vehicle/twist_fused
ros2 topic echo /vehicle/stopped

# Control
ros2 topic echo /esc/final_gate/status
ros2 topic echo /esc/drive_actual_mps
ros2 topic echo /esc/steering_actual_rad

# Timing/link
ros2 topic echo /system/processing_metrics
ros2 topic hz <topic_rt_vesc_aktual>
ros2 topic hz <topic_app_vesc_aktual>
```

Nama topic RT/APP final harus memakai nama aktual yang diexpose bridge saat implementasi, bukan membuat alias baru tanpa alasan.

## 53. Hardware validation wajib
Bench/wheel-off-ground lebih dulu untuk command path. Setelah itu low-speed field test. Test motor/steering harus mempunyai E-stop fisik siap, area bebas, speed cap, log aktif, dan satu operator yang mengontrol authority. Jangan tune PI yaw-rate atau final gate dari test high-speed pertama.
# BAGIAN P — VALIDASI DOKUMEN

## 54. Path validation 2026-09-07
Dokumen dipindai terhadap filesystem `otomasi`: 132 absolute path unik ditemukan. Seluruh path sumber/target existing yang disebut dokumen terdeteksi ada. 22 path yang belum ada adalah **target CREATE** yang memang sengaja didefinisikan untuk implementasi baru (`agv_msgs`, semantic router, fused twist, state manager, evaluators, processing monitor, steering observer/predictor, depth safety, graph/config baru).

Dengan demikian `FROM` path tidak bergantung pada path imajiner, sedangkan path yang belum ada sudah dapat dibedakan sebagai pekerjaan yang harus dibuat.

## 55. Prinsip terakhir untuk agen implementasi
Selalu mulai dari fungsi target existing, port algoritma sekecil mungkin, aktifkan observer/shadow, ukur, baru beri authority. Jika sebuah upgrade membutuhkan node kedua yang menghasilkan command untuk fungsi yang sudah memiliki owner, desain tersebut harus ditolak dan digabung ke owner yang ada.