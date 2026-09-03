# DEPRECATED ARCHITECTURE NOTE

Dokumen ini merekam arsitektur sebelum final integration 2026-08-27. Jangan gunakan tabel sensor/launch di bawah sebagai konfigurasi aktif. Kontrak final: GNSS x/y + vx+vyaw, IMU absolute yaw, ESC optional untuk localization. Lihat `MINIPC_NAV2_NO_CAMERA_COMMISSIONING.md` dan `README_MINIPC_FINAL.md`.

# Audit Mini-PC Nav2 Tanpa Kamera — 2026-08-24

## Kesimpulan

Source sudah dipisahkan sehingga package dan proses `perception` benar-benar
opsional. `autonomous.launch.py` dan `gui.launch.py` default ke navigation-only,
tetap dapat menjalankan ESC, GNSS, IMU, dua EKF, LocalizationCore, Nav2, dan GUI
ketika package perception tidak dibangun atau tidak terpasang.

Revisi lanjutan 2026-08-25 mempertahankan package perception di workspace dengan
tiga mode runtime: OFF, CPU OpenCV-DNN/ONNX, dan GPU CUDA/TensorRT. CPU selalu
dibangun; GPU bersifat capability-gated dan tidak lagi membuat configure gagal
ketika CUDA/TensorRT tidak tersedia.

Audit source dan tes regresi lulus. Status **belum production-certified** karena
enam evidence kalibrasi fisik pada file yang diterima masih `false`: steering
physical, circle geometry, drive odometry, IMU stationary, GNSS Doppler velocity,
dan GNSS COG. Ini adalah interlock yang benar dan tidak boleh diubah menjadi
`true` tanpa pengukuran robot nyata.

## Perubahan yang diterapkan

1. `navigation/package.xml` tidak lagi mempunyai dependency wajib ke
   `perception`.
2. Kedua launch melakukan lookup package perception secara opsional. Permintaan
   `enable_perception:=true` saat package tidak tersedia menghasilkan warning
   dan fallback navigation-only, bukan menghentikan launch.
3. Default kamera/perception dan lane safety menjadi `false`; collision monitor
   berbasis kamera hanya dapat aktif jika perception tersedia dan kalibrasi
   metrik kamera valid.
4. GUI sekarang meneruskan `enable_trajectory_safety` dan
   `stage3_commissioning_mode` secara eksplisit.
5. Bootstrap path native GUI memisahkan package wajib (`navigation`, `esc`) dan
   optional (`perception`), sehingga hilangnya perception tidak mengalihkan GUI
   ke path konfigurasi yang salah.
6. IMU hanya memublikasikan quaternion sebagai heading baru bila packet ANGLE
   masih fresh (default 0.50 s). Quaternion unavailable/zero/non-finite ditolak,
   dan heading awal tidak boleh di-latch sebelum sample orientation valid cukup.
7. Ditambahkan preflight mini-PC dan regression contract untuk mode tanpa kamera
   serta kontrak runtime ESC.
8. Ditambahkan `perception_cpu_node`, launch backend `cpu|gpu`, preflight backend,
   dan gate yang mencegah output kamera belum terkalibrasi memperoleh authority
   command autonomous.

## Authority estimasi dan command

| State/fungsi | Authority | Status desain |
|---|---|---|
| Kecepatan lokal `vx` | `/esc/odom` | Satu-satunya state ESC yang difusi EKF lokal |
| Yaw-rate lokal `vyaw` | gyro-Z `/imu/data` | Pengukuran fisik utama; yaw Ackermann ESC diagnostic-only |
| Posisi global `x,y` | `/odometry/gnss_map` | GNSS absolut dengan quality, timestamp, covariance, dan map gate |
| Kecepatan global `vx` | `/gnss/base_velocity_fusion` | Doppler GNSS, hanya setelah field certification |
| Heading global `yaw` | `/gnss/cog_heading_fusion` | COG absolut saat maju lurus, hanya setelah field certification |
| Output actuator | node `esc_ackermann` | Satu UART owner; priority E-STOP > teleop > gated Nav2 > idle |

GNSS single-antenna tidak memberikan yaw-rate lokal yang andal, terutama pada
kecepatan rendah. Karena itu `vyaw` tetap dari IMU dan GNSS COG dipakai sebagai
koreksi heading absolut jangka panjang. Ini mencegah diferensiasi COG yang noisy
masuk sebagai yaw-rate palsu.

## Nav2

- Global planner: Smac Hybrid dengan motion model Dubins.
- Local controller: MPPI motion model Ackermann.
- Minimum turning radius konsisten `1.712159 m` pada planner/controller.
- Batas commissioning: `vx <= 0.18 m/s`; konfigurasi produksi saat ini
  `vx <= 0.30 m/s`, `|wz| <= 0.175217 rad/s`.
- Tidak ada rotate-in-place behavior; robot hanya mempunyai behavior `wait`.
- Rantai command navigation-only:
  `/cmd_vel_nav_raw -> velocity_smoother -> /cmd_vel_nav_smoothed ->`
  `NavigationCore -> /cmd_vel -> esc_ackermann`.
- Local costmap tetap dapat aktif tanpa pesan perception karena source kamera
  tidak mempunyai required update rate. Pada mode ini obstacle dinamis tidak
  tersedia; hanya peta statis dan footprint robot yang dapat digunakan.

## Hasil validasi di lingkungan audit

| Pemeriksaan | Hasil |
|---|---|
| 21 self-check Python navigation | PASS |
| ESC runtime contract | PASS |
| 5 self-check matematika C++17 | PASS |
| Parse seluruh XML dan YAML | PASS |
| Compile seluruh Python | PASS |
| Source delimiter/braces dan GUI migration contract | PASS |
| Preflight konfigurasi navigation-only | PASS |
| Build `colcon`/launch ROS 2 aktual | Tidak tersedia di lingkungan audit |
| Kalibrasi dan acceptance kendaraan nyata | WAIT — wajib dilakukan pada mini-PC/robot |

Tidak tersedianya ROS 2 di lingkungan audit berarti hasil ini tidak boleh disebut
sebagai bukti hardware-ready. Ikuti
`MINIPC_NAV2_NO_CAMERA_COMMISSIONING.md`, jalankan `colcon build`, preflight
`--runtime --require-hardware`, lalu simpan rosbag/evidence semua tahap sebelum
mengaktifkan fusion GNSS atau mode produksi.

## Landasan desain

- Nav2 MPPI menyediakan motion model Ackermann dan constraint minimum turning
  radius: <https://docs.nav2.org/configuration/packages/configuring-mppic.html>
- Smac Hybrid mendukung planning constrained/nonholonomic:
  <https://docs.nav2.org/configuration/packages/configuring-smac-planner.html>
- Konfigurasi state `robot_localization` mengikuti urutan
  `x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az` dan pemisahan
  `world_frame=odom` untuk filter kontinu vs `world_frame=map` untuk data absolut:
  <https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/params/ekf.yaml>
- Tanda yaw/gyro mengikuti konvensi frame ENU dan right-handed REP-103:
  <https://www.ros.org/reps/rep-0103.html>
