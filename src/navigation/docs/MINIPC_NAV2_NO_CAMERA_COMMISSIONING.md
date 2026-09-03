# Mini-PC Nav2 — GNSS vx/vyaw + IMU yaw, ESC Optional

Dokumen ini adalah commissioning guide Mini-PC menggunakan `autonomous.launch.py` / `gui.launch.py`.

## 1. Arsitektur localization

| Besaran | Sumber | Topic / owner |
|---|---|---|
| global `x,y` | GNSS | `/odometry/gnss_map` -> global EKF |
| body `vx` | GNSS Doppler ENU, dirotasi dengan yaw IMU | `/gnss/base_velocity_fusion` |
| `vyaw` | turunan course GNSS Doppler saat speed cukup | `/gnss/base_velocity_fusion.angular.z` |
| absolute yaw | IMU quaternion | `/imu/data` |
| `odom->base_footprint` | local EKF | `ekf_filter_node_odom` |
| `map->odom` | localization core | `localization_core` |

ESC odometry tidak digunakan sebagai input wajib EKF. Hal ini menghilangkan dependency loop lama: ESC offline -> local odom hilang -> GNSS velocity tidak bisa dirotasi -> global localization gagal.

COG GNSS masih boleh dipakai sebagai **diagnostic/qualification**, tetapi tidak menjadi absolute yaw EKF. Absolute yaw authority adalah IMU.

## 2. Build

```bash
cd /home/otomasi/ros
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select esc navigation
source install/setup.bash
```

Build perception hanya bila dibutuhkan:

```bash
colcon build --symlink-install --packages-select esc perception navigation
```

## 3. Serial Mini-PC

Default selector source:

- ESC: `Prolific_Technology_Inc._USB-Serial_Controller`
- GNSS: `1a86_USB_Serial`
- IMU: `Silicon_Labs_CP2102`

Periksa:

```bash
ls -l /dev/serial/by-id/
```

Jangan mengandalkan `/dev/ttyUSB0/1/2`, karena urutannya dapat berubah setelah reboot/replug. Bila dua adaptor mempunyai identifier yang sama, gunakan override launch dengan path `/dev/serial/by-id/...` yang spesifik atau buat udev symlink unik.

## 4. Launch profiles

GUI default:

```bash
ros2 launch navigation gui.launch.py perception_mode:=off
```

Pada GUI, tekan tombol **NAV2 MAP** di kiri atas. Visualisasi muncul di panel tengah pada subtab **NAV2 LIVE**; tidak ada window RViz kedua. Subtab **GROUND TRUTH** tetap menyediakan PGM/OSM dan tabel target.

Localization tanpa ESC UART:

```bash
ros2 launch navigation gui.launch.py perception_mode:=off esc_serial_enabled:=false
```

Tanpa node ESC sama sekali:

```bash
ros2 launch navigation gui.launch.py perception_mode:=off start_esc_ackermann:=false
```

Headless + RViz:

```bash
ros2 launch navigation autonomous.launch.py perception_mode:=off enable_rviz:=true
```

Perception sengaja default OFF. Aktifkan CPU/GPU hanya setelah model dan camera calibration siap.

## 5. Preflight

```bash
ros2 run navigation minipc_nav2_preflight.py --workspace /home/otomasi/ros
```

Setelah launch:

```bash
ros2 run navigation minipc_nav2_preflight.py \
  --workspace /home/otomasi/ros --runtime --require-hardware
```

Untuk commissioning ESC:

```bash
ros2 run navigation minipc_nav2_preflight.py \
  --workspace /home/otomasi/ros --runtime --require-hardware \
  --require-esc --expect-esc-node
```

## 6. Apa yang harus terlihat pada map

Urutan minimum:

1. `/gnss/fix_raw` dan `/gnss/vel` hidup.
2. `/imu/data` memiliki quaternion orientation valid/fresh.
3. `localization_core` menghasilkan GNSS map odometry dan GNSS body velocity.
4. local EKF menghasilkan `/odometry/filtered` dan `odom->base_footprint`.
5. global EKF menghasilkan `/odometry/filtered_map` tanpa publish TF.
6. `localization_core` menghitung/publish `map->odom`.
7. map server publish `/map`; posisi robot terlihat pada peta melalui TF chain `map->odom->base_footprint`.
8. `robot_state_publisher` publish `/robot_description`; URDF muncul pada layer `URDF Kendaraan`.
9. Setelah Goal Pose diterima, `/plan` menampilkan global path Smac Hybrid-A*, `/controller_server/transformed_global_plan` menampilkan plan yang dipakai MPPI, dan `/trajectories` menampilkan kandidat rollout MPPI tersampling.

Cek cepat:

```bash
ros2 topic hz /imu/data
ros2 topic hz /gnss/vel
ros2 topic hz /gnss/base_velocity_fusion
ros2 topic hz /odometry/filtered
ros2 topic hz /odometry/filtered_map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 topic echo /robot_description --once
ros2 topic hz /plan
ros2 topic hz /controller_server/transformed_global_plan
ros2 topic hz /trajectories
```

`/plan` dan `/trajectories` baru aktif setelah Goal Pose dikirim dan lifecycle planner/controller sudah ACTIVE.

## 7. Validasi `vx` dan `vyaw`

Straight run maju: `linear.x > 0`.

Belok kiri/CCW: tanda `angular.z` GNSS harus konsisten dengan convention ROS/IMU. `vyaw` GNSS berasal dari perubahan course dan hanya valid saat speed lebih besar dari `gnss_yaw_rate_min_speed_mps`. Di bawah threshold, source memberi covariance sangat besar agar EKF mengabaikannya.

Lakukan run kanan dan kiri serta maju dua arah sebelum menyimpulkan sign benar.

## 8. Kalibrasi GUI

GUI BAB IV menyimpan hasil ke YAML secara atomik. Kalibrasi IMU/drive dapat mencabut **evidence certification** GNSS sehingga autonomy gate kembali WAIT, tetapi tidak lagi mematikan input estimator GNSS `vx+vyaw`.

Tombol COG hanya mensertifikasi diagnostic consistency. Ia tidak mengaktifkan COG sebagai absolute yaw.

Gunakan tombol `Restore EKF Sensor Policy` bila ingin memastikan baseline:

```text
GNSS velocity fusion = ON
GNSS COG absolute yaw = OFF
legacy GNSS course yaw correction = OFF
IMU absolute yaw = ON melalui ekf.yaml
```

## 9. ESC offline

Saat `esc_serial_enabled:=false` atau serial tidak tersedia:

- localization/map tidak boleh shutdown;
- `/esc/ready` tidak boleh dipalsukan menjadi true;
- command motor tidak boleh dianggap executable;
- GUI/sensor/TF/Nav2 dapat tetap dipakai untuk diagnostic/localization.

## 10. Safety

Perception OFF tidak menyediakan deteksi obstacle dinamis berbasis kamera. Gunakan area steril, E-STOP fisik, spotter, dan kecepatan commissioning rendah. Calibration flags bawaan tetap `false` sampai ada bukti pengujian fisik.
