> **HISTORICAL / SUPERSEDED (2026-08-27):** bagian arsitektur fusion di dokumen ini merekam tahap implementasi sebelumnya. Kontrak runtime final adalah **GNSS x/y + GNSS vx/vyaw, IMU absolute yaw, ESC optional untuk localization**. Gunakan `MINIPC_NAV2_NO_CAMERA_COMMISSIONING.md` sebagai authority saat deploy.

# Deep Audit & 3-Stage Implementation — GNSS + IMU + Odometry + Ackermann Outdoor

Date: 2026-08-23  
Target: ROS 2 Humble / Ackermann outdoor AGV  
Input project: `src_mobil_precision_steering_physical_part3(1).zip`

## 1. Scope audit

Audit ini memeriksa source C++, GUI Qt/C++, Python tools/tests, launch, YAML, CMake/package wiring, URDF/Xacro, map references, serial-device ownership, TF/localization architecture, steering physical calibration, Ackermann geometry, Nav2 Smac/MPPI, velocity smoother, trajectory safety, perception integration, serta regression/self-check yang tersedia.

Binary assets (STL/PGM/PNG) diinventarisasi dan referensinya diperiksa. Audit ini tidak mengklaim melakukan semantic reverse-engineering pada binary assets. Full ROS runtime build tidak dapat dijalankan di environment audit karena ROS 2/colcon/Qt development stack tidak tersedia; validasi target-side tetap wajib.

## 2. 20 reference projects paling relevan

Urutan berikut berdasarkan kecocokan terhadap kebutuhan GNSS–IMU–wheel odometry–Ackermann outdoor, bukan sekadar jumlah GitHub stars.

| # | Repository | Kegunaan untuk project ini | Yang layak diadopsi |
|---|---|---|---|
| 1 | MapIV/eagleye — https://github.com/MapIV/eagleye | Paling dekat dengan vehicle localization GNSS + IMU + wheel speed | GNSS Doppler velocity, heading/course gating, sensor time alignment, vehicle-speed fusion, quality-based localization |
| 2 | cra-ros-pkg/robot_localization — https://github.com/cra-ros-pkg/robot_localization | Baseline EKF/UKF ROS 2 | State selection, covariance discipline, dual local/global estimator pattern, REP-105 frame semantics |
| 3 | ros-navigation/navigation2 — https://github.com/ros-navigation/navigation2 | Planner/controller/safety navigation ROS 2 | Smac Hybrid-A*, MPPI Ackermann, velocity smoother, collision monitor, lifecycle/testing |
| 4 | ros-controls/ros2_controllers — https://github.com/ros-controls/ros2_controllers | Reference kinematics & odometry Ackermann | Steering-controller geometry, odometry interfaces, command timeout, front/rear track semantics |
| 5 | f1tenth/f1tenth_system — https://github.com/f1tenth/f1tenth_system | Ackermann hardware stack dengan VESC | Steering-servo calibration, speed↔ERPM mapping, wheelbase, odometry, deadman/arbitration |
| 6 | autowarefoundation/autoware_universe — https://github.com/autowarefoundation/autoware_universe | Production-style autonomous vehicle architecture | Separation sensing/localization/control, localization diagnostics, vehicle motion constraints |
| 7 | sseifsalama/ackermann_robot — https://github.com/sseifsalama/ackermann_robot | ROS 2 Ackermann end-to-end example | Wheel odom + IMU yaw-rate strategy, hardware/simulation separation, Nav2 adaptation |
| 8 | KumarRobotics/ublox — https://github.com/KumarRobotics/ublox | Mature u-blox driver reference | UBX parsing, velocity topic, covariance semantics, diagnostics/configuration |
| 9 | septentrio-gnss/septentrio_gnss_driver — https://github.com/septentrio-gnss/septentrio_gnss_driver | Strong GNSS/INS driver architecture | Timestamp handling, ENU/NED conversion, lever arm, RTK status, diagnostics, reconnect, covariance |
| 10 | ros-drivers/nmea_navsat_driver — https://github.com/ros-drivers/nmea_navsat_driver | Standard NMEA parsing fallback | Strict NMEA parsing and standard ROS message behavior |
| 11 | MapIV/rtklib_ros_bridge — https://github.com/MapIV/rtklib_ros_bridge | RTKLIB→ROS bridge | ECEF/velocity/time output pattern and RTKLIB integration if receiver upgraded |
| 12 | ros-geographic-info/geographic_info — https://github.com/ros-geographic-info/geographic_info | Geographic coordinate utilities | Geodesy and geographic message conventions |
| 13 | swri-robotics/marti_common — https://github.com/swri-robotics/marti_common | Outdoor/vehicle ROS utilities | Transform, geometry, serial, route utility design patterns |
| 14 | CCNYRoboticsLab/imu_tools — https://github.com/CCNYRoboticsLab/imu_tools | Practical IMU filtering | Complementary/Madgwick filtering and orientation handling references |
| 15 | gaowenliang/imu_utils — https://github.com/gaowenliang/imu_utils | IMU noise characterization | Allan variance / noise-density and random-walk characterization for covariance tuning |
| 16 | ethz-asl/kalibr — https://github.com/ethz-asl/kalibr | Sensor calibration reference | Extrinsic/time-offset calibration methodology; especially useful if camera-IMU fusion is added |
| 17 | Aceinna/gnss-ins-sim — https://github.com/Aceinna/gnss-ins-sim | GNSS/INS simulation | Repeatable estimator tests, sensor-error injection, trajectory validation |
| 18 | TixiaoShan/LIO-SAM — https://github.com/TixiaoShan/LIO-SAM | Lidar-IMU + GPS factor reference | Time sync, IMU extrinsic rigor, GPS-factor gating; optional future localization extension |
| 19 | HKUST-Aerial-Robotics/GVINS — https://github.com/HKUST-Aerial-Robotics/GVINS | GNSS-visual-inertial fusion | Globally consistent fusion and GNSS/IMU timing concepts if vision localization is later added |
| 20 | rpng/open_vins — https://github.com/rpng/open_vins | High-quality inertial/visual estimation reference | IMU modeling, calibration, timing, covariance, estimator validation methodology |

Additional architecture reference: Nav2 GPS tutorial: https://github.com/ros-navigation/docs.nav2.org/blob/master/tutorials/docs/navigation2_with_gps.rst

## 3. Temuan utama dari project asli

### 3.1 Serial-device ownership tidak konsisten

Project asli memiliki tiga device berbeda tetapi fallback C++ GNSS dan IMU tertukar:

- GNSS YAML: `1a86_USB_Serial`, tetapi fallback source GNSS menggunakan CP2102.
- IMU YAML: `Silicon_Labs_CP2102`, tetapi fallback source IMU menggunakan `1a86_USB_Serial`.
- ESC masih memilih FTDI `FTDI_FT232R_USB_UART_A5069RR4`.

Ini berbahaya bila params file gagal dimuat atau node dijalankan standalone. Source kini disamakan dengan YAML dan ESC dipindahkan ke Prolific.

Target ESC yang sekarang digunakan:

`/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0`

Selector runtime sengaja hanya memakai substring:

`Prolific_Technology_Inc._USB-Serial_Controller`

agar tidak bergantung pada `/dev/ttyUSBx`.

### 3.2 Bug/ketidakrapian IMU baud candidate

`imu_node.cpp` memakai `std::array<int, 8>` tetapi hanya mengisi dua nilai; enam entry menjadi `0`. Memang terskip di runtime, tetapi tidak clean. Sudah dirapikan menjadi array dua entry.

### 3.3 Logical steering protocol tercampur dengan physical wheel angle

Ini adalah temuan paling kritis. Protokol STM/ESC menggunakan unit logical/protocol yang dapat bergerak dalam rentang besar, tetapi nilai itu bukan otomatis sudut roda fisik. Project asli memasukkan sekitar ±80° sebagai `max_steering_angle_rad`, sehingga dengan `wheelbase=0.70 m` dan `track=0.48 m` radius minimum menjadi sekitar `0.363 m`—tidak masuk akal sebagai authority sebelum kalibrasi roda fisik valid.

Project sekarang memakai safe fallback sebelum Part 1/2:

- measured physical limits: ±30°
- operational limit: ±28°
- effective wheelbase: 0.70 m
- track: 0.48 m
- safety margin turning radius: 10%
- minimum turning radius fallback: `1.712159378317 m`
- vehicle max yaw ceiling at 0.5 m/s: `0.292028888392 rad/s`
- autonomous yaw ceiling at Nav2 `vx_max=0.3 m/s`: `0.175217333035 rad/s`

Rumus safe fallback:

`R_min = (track/2 + L_eff / tan(delta_operational)) * (1 + margin)`

Yaw-rate ceiling:

`|omega|max <= |v|max / R_min`

Nilai tersebut **bukan hasil kalibrasi final**; hanya authority aman sampai physical calibration dan circle-test selesai.

### 3.4 Stale circle certification sesudah steering recalibration

Sebelumnya, mengulang physical calibration Part 1 atau LUT Part 2 tidak otomatis membatalkan hasil circle-test Part 3 lama. Ini dapat membuat planner tetap memakai radius lama walaupun transfer steering sudah berubah.

Sekarang:

- apply Part 1 → circle calibration lama invalid;
- apply Part 2 → circle calibration lama invalid;
- theoretical radius dihitung ulang;
- vehicle, NavigationCore, Nav2 MPPI/Smac, velocity smoother, dan trajectory safety disinkronkan;
- Part 3 baru boleh membuat radius menjadi certified lagi.

### 3.5 Authority geometri tersebar dan sebelumnya tidak semuanya sinkron

Sebelumnya Part 3 mengubah minimum turning radius MPPI/Smac tetapi yaw limit pada velocity smoother dan trajectory-safety dapat tertinggal. `syncVehicleAuthority()` juga belum mengunci semua downstream limit.

Sekarang satu authority chain memaksa:

`physical steering -> operational steering -> Rmin -> v/R yaw cap -> planner/controller/smoother/safety`

### 3.6 Dead compatibility state pada center hold

Member `steering_center_hold_ki_`, integral-limit state, dan `last_center_hold_i_deg_` dibaca/di-reset tetapi tidak dipakai pada algoritma. Runtime center hold memang P/adaptive bounded, bukan PI. Member dead tersebut dihapus; parameter YAML deprecated tetap dideklarasikan agar file lama tidak rusak.

### 3.7 Localization architecture

Arsitektur saat ini sengaja berbeda dari tutorial dual-EKF standar:

- local EKF: ESC longitudinal `vx` + IMU `gyro-z`, publish `odom -> base_footprint`;
- global EKF: GNSS/map translation + gated GNSS longitudinal velocity + optional gated COG yaw; `publish_tf=false`;
- `LocalizationCore` custom adalah **single owner** `map -> odom`.

Ini valid dan mencegah double-TF. Jangan mengaktifkan TF global EKF bersamaan dengan `LocalizationCore`.

Kelebihan yang sudah ada:

- GNSS velocity di-time-align dengan local odom history;
- antenna lever-arm dikoreksi pada velocity (`v_base = v_ant - omega x r`);
- quality gates hAcc/DOP/sAcc/course;
- COG diblok pada low speed;
- slip residual wheel-vs-GNSS;
- hysteresis valid/invalid hold;
- global velocity/COG fusion default OFF sampai field qualification.

### 3.8 Heading awal masih calibration-sensitive

`use_imu_initial_heading=true`, sedangkan `magnetic_declination_radians=0.0`. Local EKF tidak fuse absolute orientation—ini baik selama magnetometer/heading belum terbukti—tetapi initial map heading tetap sensitif terhadap orientasi IMU awal.

Jangan mengarang nilai declination. Pilihan setelah field test:

1. kalibrasikan absolute heading + declination dan dokumentasikan;
2. gunakan COG initialization setelah straight motion pada speed cukup;
3. gunakan dual-antenna GNSS bila kebutuhan heading absolut tinggi.

### 3.9 GNSS velocity covariance sebelum fusion

Current code merotasi covariance ENU→map→body dan kemudian memberi variance clamp sebelum global fusion. Lever-arm velocity correction sudah ada. Sebelum `enable_global_gnss_velocity_fusion=true`, field test tetap perlu menguji kontribusi uncertainty yaw-rate/lever-arm dan menentukan variance floor yang cocok dari data nyata; jangan mengaktifkan fusion hanya karena topic tersedia.

### 3.10 FOC/calibration placeholders tidak boleh diisi tebakan

`foc_thesis.yaml` masih memiliki nilai `UNVERIFIED` dan `0.0` pada direction, phase order, current gain, PI current, PI position, dan safety limits. Ini bukan syntax error; ini commissioning gate. Mengisi angka fiktif justru lebih berbahaya daripada membiarkannya eksplisit belum terverifikasi.

### 3.11 Perception portability

Path TensorRT model dan beberapa converter defaults masih deployment-specific `/home/otomasi1/...`. Autonomous launch sudah mendukung override `YOLOP_ENGINE_PATH`. Path tidak diganti secara membabi buta karena dapat merupakan lokasi model target yang benar. Untuk deployment portable, pindahkan model path ke environment/service configuration di Stage 3 setelah target path dipastikan.

CUDA root sekarang dibuat `CACHE PATH`, sehingga build tidak lagi terkunci mutlak pada satu lokasi compiler.

## 4. Perubahan yang sudah diterapkan

### Serial / hardware identity
- ESC selector FTDI -> Prolific.
- GNSS fallback source -> CH340/u-blox serial identity `1a86_USB_Serial`.
- IMU fallback source -> CP2102 identity `Silicon_Labs_CP2102`.
- Runtime/source/YAML dibuat konsisten dan tidak overlap.

### Ackermann / steering authority
- logical protocol dipisahkan dari physical wheel angle;
- fallback operational steering ±28°;
- fallback Rmin aman 1.712159 m;
- vehicle/NavCore/Nav2/velocity smoother/trajectory safety disinkronkan;
- re-apply Part 1/2 invalidates Part 3 circle certification;
- Part 3 propagates yaw/radius authority ke semua downstream gate.

### Code hygiene
- IMU baud candidate array diperbaiki;
- dead center-hold compatibility state dihapus;
- runtime log/comment revision labels dirapikan;
- README logical steering disamakan dengan implementasi;
- CUDA root lebih overrideable.

### Regression tests
- semua existing Python self-check dipertahankan;
- standalone C++ math checks dipertahankan;
- ditambahkan `project_consistency_self_check.py`;
- self-check suite didaftarkan di `BUILD_TESTING` CMake agar dapat dijalankan lewat `colcon test` di mesin ROS target.

## 5. Tiga tahap implementasi

## TAHAP 1 — Foundation, hardware identity, steering/odometry authority

Status: **baseline code sudah diterapkan di ZIP revisi; target-hardware verification masih wajib.**

Tujuan: tidak ada salah-port, tidak ada geometri fiktif, satu source of truth untuk steering/kinematics, dan project bisa diuji repeatable.

Urutan target:

1. Verifikasi `/dev/serial/by-id`:
   - ESC -> Prolific;
   - GNSS -> CH340/`1a86_USB_Serial`;
   - IMU -> CP2102.
2. Jalankan build bersih `colcon build` pada ROS 2 Humble target.
3. Jalankan `colcon test --packages-select navigation`.
4. Pastikan hanya `LocalizationCore` publish `map->odom`, local EKF publish `odom->base_footprint`.
5. Steering Physical Part 1: ukur hard left, physical center, hard right.
6. Steering Part 2: ambil multi-point LUT increasing/decreasing dan hysteresis.
7. Ulangi circle Part 3 setelah Part 1/2 berubah.
8. Kalibrasi drive/RPM scale dan verifikasi odom straight-line.
9. Kalibrasi stationary IMU bias/noise dan frame sign.
10. Jangan mengaktifkan global GNSS velocity/COG fusion pada tahap ini.

PASS gate:
- serial ownership unik;
- no stale circle certificate;
- measured steering physical valid;
- odom vx direction/scale valid;
- gyro-z sign/frame valid;
- TF tree satu owner per transform;
- all regression tests PASS.

## TAHAP 2 — GNSS/IMU/odometry precision localization

Tujuan: membuktikan GNSS motion data layak sebelum difuse.

1. Rekam rosbag GNSS raw/quality/velocity, IMU, ESC odom, filtered local odom, TF.
2. Uji stationary: GNSS velocity mean/std, IMU gyro bias, position scatter.
3. Straight runs dua arah dan beberapa kecepatan.
4. Validasi timestamp GNSS vs local odom dan sync gap.
5. Validasi lever-arm sign menggunakan yaw maneuver lambat.
6. Bandingkan GNSS Doppler speed vs wheel odom; hitung residual dan slip threshold.
7. Validasi COG hanya di atas speed minimum; reject saat low-speed/turning tinggi.
8. Tuning covariance berdasarkan measured standard deviation/innovation, bukan tebakan.
9. Setelah statistik PASS, aktifkan **velocity fusion dahulu**, evaluasi.
10. Aktifkan **COG fusion belakangan** jika heading benar-benar membantu dan tidak menyebabkan jump/double-count.
11. Tetap biarkan legacy direct COG correction OFF bila global EKF COG fusion ON.

PASS gate:
- no timestamp jumps/out-of-order;
- velocity residual berada dalam threshold mayoritas run;
- COG innovation stabil saat straight motion;
- global pose tidak melompat pada GNSS quality degradation;
- map->odom correction bounded;
- local odom tetap smooth saat GNSS ditutup/disconnect.

## TAHAP 3 — Outdoor Nav2 Ackermann performance + safety certification

Tujuan: tuning planner/controller dilakukan hanya setelah geometry/localization dipercaya.

1. Sertifikasi circle test kiri/kanan dan `effective_wheelbase`.
2. Sinkronkan final Rmin ke Smac Hybrid dan MPPI Ackermann.
3. Tuning Smac penalties/lookup dengan radius final.
4. Tuning MPPI `vx_max`, acceleration, horizon, critics dari speed rendah ke target.
5. Jaga `model_dt = 1/controller_frequency` untuk configuration ini.
6. Tuning velocity smoother dan trajectory safety terhadap final `v/R` envelope.
7. Uji path: straight, S-curve, 90°, U-turn feasible, narrow passage, goal approach.
8. Uji GNSS degradation, IMU dropout, ESC disconnect, camera dropout, E-STOP.
9. Sertifikasi camera metric calibration sebelum camera-based distance dijadikan hard safety authority.
10. Centralize final TensorRT engine path via deployment environment/service config.
11. Simpan rosbag + YAML snapshot + parameter CRC untuk setiap tuning session.

PASS gate:
- planner tidak meminta curvature lebih tajam dari physical vehicle;
- MPPI tracking tidak saturate steering terus-menerus;
- no oscillatory map->odom;
- localization continues safely through brief GNSS degradation;
- safety chain stop/fail-closed sesuai requirement;
- target-specific build, tests, and outdoor regression all PASS.

## 6. Validasi yang sudah dilakukan di environment audit

PASS:

- seluruh YAML yang diperiksa dapat diparse;
- seluruh Python source/test dapat dikompilasi syntax;
- launch internal executable references ditemukan di package CMake;
- seluruh referenced navigation mesh ada;
- Xacro/XML static parse PASS;
- map YAML menunjuk file PGM yang ada;
- existing Python self-check suite PASS;
- standalone C++ self-checks compile dengan `-std=c++17 -Wall -Wextra -Wpedantic -Werror` dan PASS;
- new project consistency check PASS;
- source serial selectors konsisten dan old FTDI runtime selector tidak tersisa.

Tidak dapat dilakukan di environment audit:

- full `colcon build` ROS 2;
- full Qt GUI compile/link;
- actual Xacro ROS expansion;
- hardware serial handshake dengan Prolific/STM;
- real GNSS/IMU/ESC live topic validation;
- RViz/Nav2 runtime test;
- outdoor circle/GNSS fusion test.

Alasannya: environment audit tidak memiliki ROS 2/colcon/Qt development installation dan tidak terhubung ke hardware target.

## 7. Target-side verification commands

```bash
# 1. Device ownership
ls -l /dev/serial/by-id/

# 2. Expected ESC identity
ls -l /dev/serial/by-id/ | grep Prolific_Technology_Inc._USB-Serial_Controller

# 3. Build
cd ~/Sistem-Otomasi-Car/Car-MiniPC/ros
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 4. Tests
colcon test --packages-select navigation --event-handlers console_direct+
colcon test-result --verbose

# 5. TF ownership check after launch
ros2 run tf2_tools view_frames
ros2 topic hz /odometry/filtered
ros2 topic hz /gnss/fix_raw
ros2 topic hz /gnss/vel
ros2 topic hz /imu/data
ros2 topic hz /esc/odom

# 6. Check serial-related logs
ros2 topic echo /system/esc_connection
```

## 8. Final rules sebelum autonomous outdoor

Jangan menganggap project fully commissioned hanya karena compile PASS. Kondisi berikut harus tetap dianggap blocking sampai terukur:

- `steering_physical_calibration_enabled=false`;
- `steering_physical_lut_enabled=false`;
- `steering_circle_calibration_valid=false`;
- GNSS velocity/COG fusion masih OFF;
- camera metric calibration belum certified;
- FOC direction/phase/gains/safety limits masih `UNVERIFIED`/zero.

Prinsip utama: **kalibrasi hardware -> odometry -> IMU -> GNSS motion qualification -> fusion -> geometry certification -> planner/controller tuning -> safety regression**. Jangan memakai tuning layer atas untuk menyembunyikan error layer bawah.
