# Rangkuman Tuning Utara dan Uji 5 m — 2026-09-05

## Status akhir

- Pengujian dihentikan atas permintaan operator; robot terakhir terverifikasi `drive_actual_mps=0.0` dan `steering_actual_rad=0.0`.
- Launch `autonomous.launch.py mode:=web stage3_commissioning_mode:=true` sudah dihentikan secara graceful.
- Tidak ada skrip uji 5 m/commissioning yang dibiarkan aktif.
- Uji 5 m **BELUM PASS**. Watchdog sengaja menghentikan beberapa percobaan ketika data menunjukkan kondisi belum valid.
- Safety gate autonomy tetap fail-closed; tidak ada flag kalibrasi yang dipaksa menjadi valid.

## Konvensi heading yang dipakai

- Utara fisik = `+90 deg` pada ROS ENU.
- Operator beberapa kali menempatkan body + roda benar-benar lurus ke Utara untuk referensi lapangan.
- Setelah kalibrasi magnetometer terakhir, seed startup saat referensi Utara pernah terbaca `89.66 deg`, sesuai target.
- GNSS COG tetap menjadi kandidat absolute heading saat kendaraan bergerak, tetapi pada kecepatan commissioning 0.10–0.18 m/s course accuracy masih sekitar 35–44 deg sehingga COG belum qualified.

## Temuan IMU paling penting

- Sebelum koreksi, steering kanan / yaw kinematik ESC menunjukkan yaw ROS negatif, tetapi local EKF berbasis gyro justru naik positif.
- Contoh sesudah uji: ESC kinematic odom yaw sekitar `-19 deg`, sementara local EKF sekitar `+6.6 deg`.
- Root cause: `yaw_sign` gyro-Z IMU terbalik.
- Konfigurasi diperbaiki dari `yaw_sign: +1.0` menjadi `yaw_sign: -1.0`.
- Validasi setelah fix: ketika steering aktual positif (kanan) sekitar +1.7 deg saat bergerak pelan, yaw lokal turun sekitar -0.26 deg; tanda sekarang konsisten dengan REP-103.

## Kalibrasi heading magnetometer

- Offset sebelumnya: `mag_yaw_offset_rad = 1.7116332028`.
- Berdasarkan referensi Utara fisik terbaru, diubah menjadi `1.5451826276` rad.
- Magnetometer hanya dipakai untuk startup seed / standstill reference; moving heading tetap mengandalkan gyro-Z + qualified GNSS COG karena magnetometer pernah terpengaruh arus motor.

## Temuan dan koreksi steering center

- Saat operator memastikan roda benar-benar lurus, feedback raw steering stabil sekitar `-14.14 deg`.
- Nilai ini sekarang dipakai sebagai `steering_feedback_center_reference_deg = -14.14`.
- Percobaan mengubah command center menjadi `-5.8636 deg` terbukti salah: center-hold harus menambah trim besar dan steering masih tersisa positif.
- Command center dikembalikan ke nilai lama yang benar: `steering_feedback_center_deg = -25.84 deg`.
- `steering_center_hold_feedback_deadband_deg` diperlebar dari `0.5` menjadi `1.0 deg` agar noise/backlash dekat center tidak memicu trim terus-menerus.
- Dengan command center `-25.84` dan feedback center `-14.14`, steering aktual bisa kembali terbaca `0.00 deg` saat diam.

## Respons steering stasioner

- Command sekitar `+2 deg` menghasilkan steering aktual sekitar `+1.33 deg`, drive tetap 0.
- Command `-8 deg` menghasilkan steering aktual sekitar `-1.22 deg`, drive tetap 0.
- Sisi kiri memiliki deadband/backlash lebih besar daripada sisi kanan; command kecil negatif sekitar -2..-5 deg dapat tertahan di deadband.
- Setelah command dilepas, center bisa kembali ke sekitar 0 deg.

## Pilot lurus awal

- Pilot sekitar 0.6 m: maju Utara `~0.611 m`, lateral `~0.069 m`, heading bergeser dari sekitar 93.47 deg menuju ~97.6 deg.
- Pilot heading-hold berikutnya di-abort sekitar 0.43 m karena lateral terukur ~0.262 m.
- Data ini memicu audit center steering dan tanda gyro-Z.

## Uji 5 m — percobaan map-watchdog

Data: `data/navigasi/5m_north_20260905_034209.csv` (129 sampel).

- Start heading map: `89.659 deg`.
- Selama loop command, map displacement terakhir sebelum stop: North `0.636 m`, East `-0.299 m`.
- Integrasi drive saat itu: `0.448 m`.
- Heading terakhir loop: `90.933 deg`; error hanya `+0.933 deg`.
- DOP sekitar `1.86–1.89`, hAcc sekitar `0.89–0.92 m`.
- COG receiver sekitar `91.26 deg`, tetapi course accuracy sekitar `44 deg`; `cog_qualified=0`, `cog_fusion=0`.
- Watchdog lateral berbasis map terbukti terlalu sensitif untuk short run karena hAcc mendekati 1 m; run di-abort, bukan diteruskan 5 m.

## Uji 5 m — percobaan local-odom

Data: `data/navigasi/5m_north_local_20260905_034522.csv` (621 sampel).

- Local forward terakhir: `2.592 m`.
- Local lateral terakhir: `0.207 m`.
- Drive integral terakhir: `2.802 m`.
- Progress estimator terakhir: `2.665 m`.
- Local yaw naik dari `1.520 deg` ke `6.442 deg`.
- Steering actual mencapai sekitar `5.30 deg` dengan command limit `+5 deg`.
- Map yaw terakhir: `96.194 deg`.
- DOP range `1.50–3.18`, hAcc `0.615–0.853 m`.
- Raw COG sekitar 130–150 deg dengan accuracy ~43 deg; COG tetap belum qualified/fused.
- Percobaan ini dihentikan setelah diketahui local yaw bergerak dengan tanda berlawanan terhadap yaw kinematik ESC; dari sinilah bug `yaw_sign` IMU ditemukan.

## Kesimpulan teknis hari ini

1. Heading startup Utara sudah dapat dibuat dekat 90 deg ROS.
2. Bug tanda gyro-Z sudah ditemukan dan diperbaiki (`yaw_sign=-1`).
3. Feedback center steering fisik terbaru adalah sekitar `-14.14 deg` raw.
4. Command center yang benar tetap `-25.84 deg`; jangan ganti ke -5.86 deg.
5. Steering kanan dan kiri sama-sama bisa bergerak, tetapi sisi kiri memiliki deadband/backlash lebih besar.
6. GNSS posisi absolut cukup untuk global reference, tetapi hAcc 0.6–0.9 m tidak cocok dijadikan watchdog lateral short-run 5 m.
7. GNSS COG pada 0.10–0.18 m/s belum cukup akurat; untuk qualification COG besok perlu speed/track straight yang cukup sambil tetap aman.
8. Untuk tuning besok, gunakan local EKF/gyro untuk heading jangka pendek, ESC/local odom untuk jarak relatif, dan GNSS sebagai validasi global.

## Rencana lanjutan besok

1. Start robot diam, body + roda benar-benar lurus ke Utara.
2. Verifikasi steering actual ~0 deg, raw feedback dekat -14.14 deg, gyro-Z stationary ~0.
3. Restart stack dan pastikan seed heading dekat 90 deg.
4. Lakukan short left/right steering motion untuk verifikasi tanda gyro vs kinematic yaw sekali lagi.
5. Tuning deadband/gain steering kiri-kanan dengan pilot 0.5–1.0 m, bukan langsung 5 m.
6. Validasi odometry scale dengan lintasan ukur nyata sebelum memakai jarak encoder sebagai ground truth.
7. Setelah lateral dan heading stabil, jalankan fresh 5.000 m dengan braking profile dan independent GNSS validation.
8. Baru nyatakan PASS bila jarak relatif, lateral, heading, steering response, dan safety gate semuanya memenuhi acceptance criteria.
