# Audit mounting Yahboom IMU - 2026-09-08

- Gejala awal: yaw Yahboom sekitar 348 deg sementara heading absolut IST8310 NEO3 sekitar 159 deg; selisih sekitar 171 deg.
- Uji hipotesis mounting +180 deg: yaw_offset Yahboom ditambah pi, menghasilkan yaw sekitar 168 deg; selisih ke NEO3 turun menjadi sekitar 9-10 deg.
- `map_yaw_from_enu` saat pengujian = 0, sehingga perbaikan bukan efek rotasi map.
- TF base_footprint->imu_link sebelumnya identity; pipeline ini melakukan koreksi mounting di driver agar data yang diterbitkan konsisten dengan body/REP-103.
- Untuk rotasi planar 180 deg, X/Y berbalik dan Z tetap. Karena itu invert_roll=true, invert_pitch=true, accel X/Y ikut dibalik; gyro Z tetap mengikuti yaw_sign yang sudah ada.
- Magnetometer Yahboom tidak dipakai sebagai absolute heading karena norm medan terukur sekitar 990 uT, di luar gate medan bumi 15..100 uT. IST8310 NEO3 sekitar 67 uT dan menjadi sumber magnetic heading utama.
- Local EKF memakai IMU relative (`imu0_relative=true`), sehingga odom yaw mulai 0 dan hanya merepresentasikan perubahan heading. Absolute heading tetap ditangani seed/global correction.
- Jangan fine-tune sisa offset 9-10 deg dari satu sampel statis. Validasi akhir harus memakai heading referensi fisik/COG saat GNSS qualified.

## Validasi runtime lanjutan
- Global EKF setelah koreksi mounting dan penghapusan yaw relatif IMU dari global fusion menghasilkan yaw sekitar 2.749 rad (~157.5 deg), konsisten dengan NEO3 magnetic heading sekitar 157-159 deg.
- NEO3 IST8310 valid, magnitude medan sekitar 66.2 uT dan lolos gate 15-100 uT.
- GNSS saat pengujian belum layak autonomous: sekitar 8 satelit, DOP ~5.23, hAcc ~4.31 m. Localization tetap DEGRADED_HOLD / motion_localization_ready=false sesuai fail-closed design.
- Regression tests diperbarui agar global EKF memakai IMU gyro-Z saja; absolute yaw berasal dari GNSS COG + magnetic heading. GNSS velocity fusion tetap membutuhkan field certification.
- ekf_humble_config_self_check, f411_mag_pipeline_self_check, stage2_localization_foundation_self_check, full_stack_reaudit_self_check: PASS.
- navigation build PASS dengan 2 core (taskset 0,1; parallel-workers 2).

## Upgrade UART Yahboom ke 921600 bps
- Port runtime: CP2102 `/dev/ttyUSB0` via by-id stabil.
- Protokol WIT register BAUD 0x04 diuji langsung dari terminal.
- Tahapan validasi: 9600 -> 230400 -> 460800 -> 921600 bps.
- Setiap baud menghasilkan frame WIT 0x55 dengan checksum valid.
- Baud final 921600 bps disimpan permanen ke sensor dan diverifikasi ulang setelah reopen.
- ROS disinkronkan ke 921600 pada `imu.yaml`, baseline web, `imu.launch.py`, dan `autonomous.launch.py`.
- Auto-baud recovery diaktifkan dengan kandidat 921600, 460800, 230400, 115200, 9600.
- Build navigation PASS dengan 2 core. Runtime log membuktikan `IMU stream detected ... @ 921600 (WIT frame checksum valid)`.
- Catatan: UART baud dan output rate adalah parameter terpisah. Saat audit ini RRATE masih 0x06 = 10 Hz.

## Kalibrasi diam Yahboom vs NEO3 setelah UART 921600
- Akuisisi awal 30 s; 5 s pertama dibuang. Sampel efektif: IMU 164, NEO3 heading 501, NEO3 mag 496.
- Sebelum fine alignment: Yahboom 167.9150 deg, NEO3 157.3633 deg, residual +10.5517 deg.
- Koreksi `yaw_offset_rad`: 0.3893434987 -> 0.2051809117 rad.
- Verifikasi setelah restart: Yahboom 157.3633 deg, NEO3 157.4274 deg, residual -0.0641 deg.
- Verifikasi final 15 s: residual -0.0488 deg.
- NEO3 medan magnet: 66.69 uT, std 0.39-0.48 uT; valid pada gate 15..100 uT.
- Roll/pitch Yahboom diam: -0.8961 / -1.8976 deg. Komponen gravitasi X/Y konsisten dengan tilt ini, sehingga roll/pitch tidak dinolkan secara paksa.
- Bias accel disesuaikan terhadap gravitasi terprediksi: [0.0010990764, -0.0007872455, -0.0057577635] m/s^2.
- Verifikasi final norm accel = 9.80765 m/s^2, error terhadap g = +0.00100 m/s^2.
- Gyro-Z diam mean/std = 0/0 pada resolusi keluaran sensor.
- Kontrak EKF tetap fail-safe: local EKF memakai relative yaw + gyro-Z; global EKF hanya gyro-Z dari Yahboom. Absolute yaw global tetap NEO3/COG, sehingga Yahboom tidak dapat menggeser heading global bila alignment rusak.

## Redesign yaw inertial + consensus pre-EKF (2026-09-08 malam)
- Raw WIT `ANGLE` tidak lagi diperlakukan sebagai yaw inertial murni dan tidak difusikan sebagai absolute yaw ke EKF.
- `/imu/inertial_heading` sekarang berasal dari integrasi gyro-Z, di-seed dari IST8310 yang lolos gate.
- Saat stationary: `|wz|<=0.03 rad/s`, `|norm(a)-g|<=0.15 m/s2`, hold >=2 s -> gyro-Z bias diperbarui dengan EMA.
- Tidak ada reset yaw periodik berbasis waktu. Koreksi absolute dilakukan bounded maksimal 1 deg/s.
- `/imu/mag_heading_fusion` tetap tersedia diagnostik, tetapi field real Yahboom ~773-774 uT sehingga ditolak gate 15..100 uT.
- IST8310 real ~69-70 uT dan lolos gate.
- Consensus inertial+IST8310: residual <=5 deg selama >=2 s -> `/heading/validated_fusion` + `/heading/validated=true`.
- Global EKF: GNSS x/y + GNSS vx + qualified COG + `/heading/validated_fusion` + gyro-Z.
- Local EKF: ESC vx + qualified GNSS vx + gyro-Z saja; raw absolute Yahboom yaw tidak masuk.
- `LocalizationCore use_imu_initial_heading=false`; raw WIT ANGLE tidak boleh seed map.

## Baseline stationary setelah redesign
- 30 s: inertial mean 156.381 deg, std 0.131 deg, drift end-start -0.099 deg.
- IST8310 mean 156.359 deg, std 0.250 deg.
- Residual inertial-IST8310 mean -0.0008 deg, std 0.218 deg, range -0.839..+0.708 deg.
- Gyro-Z stationary mean/std = 0/0 pada resolusi sensor.
- Runtime consensus setelah restart: valid=true, residual sekitar 0.0..0.24 deg.

## Optimasi rate dan TF global
- Yahboom UART tetap 921600 baud; RRATE dinaikkan ke 0x08 dan ROS target 50 Hz.
- Publish gate diperbaiki agar jitter 19.x ms tidak membuang setiap frame kedua.
- Rate aktual sesudah fix: 49.15 Hz; dt 20.35 +/- 2.60 ms; hanya 2 gap >40 ms dalam 10 s; tidak ada gap >100 ms.
- Ditemukan global EKF ~156.37 deg tetapi map->odom semula 0 deg karena yaw freshness terikat GNSS COG.
- LocalizationCore kini menerima `/heading/validated_fusion`; saat startup stationary melakukan one-time map->odom heading alignment, lalu koreksi berikutnya bounded.
- Verifikasi akhir: map->odom 156.266 deg, map->base_footprint 156.267 deg, global EKF 156.373 deg, validated heading ~156.47 deg.
- `/system/autonomy_motion_allowed=false`; perubahan heading/TF tidak membuka actuation.
