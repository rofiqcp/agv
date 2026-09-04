# Validasi Outdoor GNSS + IMU dan Heading Utara — 2026-09-04

## Tujuan
Validasi robot saat fisik diarahkan ke utara, prioritaskan GNSS + IMU terhadap wheel encoder yang dapat selip, pastikan kamera mirror, dan validasi target navigasi 2 m ke depan.

## Temuan awal
- GNSS outdoor sehat: 16 satelit, UBX NAV-PVT 10 Hz, hAcc sekitar 0.4–0.6 m.
- IMU saat robot diarahkan ke utara membaca sekitar +98.355 derajat ROS ENU; utara ideal adalah +90 derajat.
- `/odometry/gnss_map` mengikuti IMU sekitar +98.355 derajat.
- `/odometry/filtered_map` sebelumnya sekitar 175–180 derajat karena EKF global tidak memiliki sumber absolute yaw saat kendaraan diam.
- Kamera runtime sudah mirror: parameter `/perception flip_horizontal = true`.

## Perbaikan
- `yaw_offset_rad` IMU diubah menjadi `-0.1458222590` rad (-8.355 derajat).
- EKF lokal tetap memakai wheel odom sebagai sumber kecepatan longitudinal `vx`, tetapi yaw-rate dari `/esc/odom` tidak lagi difusi agar slip encoder/steering tidak memutar heading estimator.
- EKF global sekarang memfusi absolute yaw IMU + gyro-z; GNSS COG tetap menjadi koreksi heading saat kendaraan bergerak dan memenuhi qualification gate.
- `startup_imu_samples` dinaikkan dari 3 menjadi 50 supaya anchor `map->odom` baru dikunci setelah EKF lokal selesai konvergen ke yaw IMU.
## Hasil validasi setelah restart
Window validasi 8 detik menghasilkan:
- IMU: mean 89.780 derajat, circular std 0.000 derajat.
- `/odometry/gnss_map`: mean 89.780 derajat, circular std 0.000 derajat.
- `/odometry/filtered_map`: mean 89.780 derajat, circular std 0.000 derajat.
- GNSS: 16 satelit, DOP 1.46–1.58, hAcc 0.42–0.45 m.
- `HEADING SEED`: map yaw 89.78 derajat dan local odom yaw 89.78 derajat.
- Anchor `map->odom`: yaw 0.000 rad.

## Uji target 2 m ke depan
Pose live contoh: x=128.4666 m, y=93.7244 m, yaw=89.780 derajat.
Target 2 m ke depan: x=128.4743 m, y=95.7244 m dengan orientasi tetap 89.780 derajat.
`/compute_path_to_pose` dengan planner `GridBased`/SMAC Hybrid: ACCEPTED dan SUCCEEDED, menghasilkan 9 pose dengan planning time 0.224 s.

## Interlock keselamatan
Eksekusi gerak fisik tidak dipaksa karena sistem masih melaporkan `autonomy_ready=false` dan `motion_ready=false`; HUD juga menunjukkan `ARM DISARMED`. Localization dan Nav2 sendiri sudah READY. Interlock commissioning/kalibrasi tidak dibypass.

## Kesimpulan
Heading utara sekarang konsisten antara IMU, GNSS-map, dan global EKF. GNSS menjadi referensi posisi utama; wheel encoder hanya pendukung kecepatan longitudinal. Kamera mirror aktif. Target 2 m di depan sudah lolos validasi planner, tetapi gerak fisik menunggu seluruh safety/commissioning gate mengizinkan autonomous motion.
