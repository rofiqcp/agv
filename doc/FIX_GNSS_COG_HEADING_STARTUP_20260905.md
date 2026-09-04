# Fix GNSS COG Heading Startup — 2026-09-05

## Gejala lapangan
- Arah/arrow robot saat startup dapat berbeda dari arah gerak sebenarnya.
- Saat robot digerakkan maju–mundur, jejak GNSS menunjukkan sumbu Utara–Selatan dengan benar.
- COG/Doppler GNSS menjadi referensi arah yang lebih dapat dipercaya ketika kendaraan bergerak.
- Magnetometer dapat terganggu arus motor steering/traction, sehingga tidak boleh menjadi yaw absolut kontinu saat bergerak.

## Root cause yang ditemukan
1. `cog_min_forward_speed_mps` sebelumnya 0.35 m/s, sementara commissioning speed cap hanya 0.18 m/s. COG tidak mungkin qualify pada uji pelan.
2. Maksimum yaw innovation sebelumnya 45 derajat. Startup error sekitar 90 derajat tidak mungkin dikoreksi walaupun COG benar.
3. Qualification COG bergantung pada `gnss_velocity_qualified_` dan residual wheel-slip yang dihitung memakai heading saat ini. Bila heading awal salah 90 derajat, GNSS forward tampak lateral dan COG justru ditolak.
4. IMU absolute yaw sebelumnya masih berpotensi menjadi authority kontinu; arsitektur diubah agar magnetic yaw hanya menjadi startup seed.
## Arsitektur setelah perbaikan
- Magnetic yaw: startup seed / diagnostik saat diam.
- IMU gyro-Z: kontinuitas yaw jangka pendek selama bergerak.
- GNSS COG: koreksi yaw absolut saat kendaraan maju, cukup cepat, lurus, dan kualitas receiver memenuhi gate.
- Wheel/ESC odometry: membantu `vx` saja; yaw steering/encoder tidak menjadi heading authority agar slip tidak memutar estimasi.
- Global EKF: GNSS x/y + GNSS vx + COG yaw + IMU gyro-Z.
- Local EKF: wheel/GNSS vx + IMU gyro-Z.

## Perubahan gate COG
- `cog_min_forward_speed_mps`: 0.35 -> 0.15 m/s, sehingga reachable pada commissioning cap 0.18 m/s.
- `global_ekf_yaw_max_innovation_rad`: 45 deg -> 135 deg.
- `cog_max_innovation_rad`: 45 deg -> 135 deg.
- Koreksi per update tetap dibatasi `global_ekf_yaw_max_step_rad = 0.5 deg` sehingga tidak terjadi snap heading.
- COG bootstrap tidak lagi bergantung pada proyeksi lateral berbasis heading saat ini atau wheel-slip residual.
- Gate kualitas tetap mempertahankan velocity freshness, UBX quality, forward motion, sAcc, heading accuracy, straight gyro, course consistency, dan hold timer.
## Verifikasi
- Konversi UBX `headMot` diperiksa: heading receiver (0 deg North, clockwise) dikonversi menjadi ROS ENU `yaw = 90 deg - headMot`; East=0 deg dan North=+90 deg.
- Build target `localization_core` berhasil dengan `-j1` dan exit code 0.
- Regression checks PASS:
  - `gnss_motion_math_self_check.py`
  - `gnss_motion_validation_part2_self_check.py`
  - `gnss_global_fusion_part3_self_check.py`
  - `stage2_localization_foundation_self_check.py`
- Ditambahkan regression kasus startup error 90 deg pada speed 0.18 m/s: COG harus dapat diterima dan koreksi yaw tetap rate-limited.
- Startup terbaru: magnetic seed/map yaw 81.82 deg; North ideal ROS = 90 deg.
- Saat robot diam, GNSS Doppler hanya sekitar 0–0.06 m/s dan course acak; COG dengan benar tidak dipakai pada kondisi ini.

## Safety dan status commissioning
- Stage-1 tetap fail-closed: steering physical Part 1/2, drive odometry scale, circle Part 3, dan IMU stationary calibration belum disertifikasi.
- `/system/autonomy_motion_allowed` tetap `false`; tidak ada safety gate yang dibypass.
- Verifikasi fisik COG setelah patch masih memerlukan gerak maju lurus menggunakan remote/operator. Saat COG valid, heading map diharapkan konvergen bertahap ke arah gerak GNSS.
- Launch operasional: `ros2 launch navigation autonomous.launch.py mode:=web stage3_commissioning_mode:=true`.