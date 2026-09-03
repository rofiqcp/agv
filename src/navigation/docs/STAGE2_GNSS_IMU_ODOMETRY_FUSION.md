> **HISTORICAL / SUPERSEDED (2026-08-27):** bagian arsitektur fusion di dokumen ini merekam tahap implementasi sebelumnya. Kontrak runtime final adalah **GNSS x/y + GNSS vx/vyaw, IMU absolute yaw, ESC optional untuk localization**. Gunakan `MINIPC_NAV2_NO_CAMERA_COMMISSIONING.md` sebagai authority saat deploy.

# Stage 2 — GNSS + IMU + Wheel Odometry Fusion Commissioning

## Tujuan
Stage 2 mengubah fondasi localization yang sudah tersedia menjadi **fusion yang hanya aktif setelah bukti lapangan PASS**. Prinsip utamanya:

1. `odom -> base_footprint` tetap berasal dari Local EKF (wheel odometry + IMU gyro-Z).
2. Global EKF tetap `publish_tf: false`.
3. `LocalizationCore` tetap satu-satunya owner `map -> odom`.
4. Raw GNSS velocity tidak pernah langsung menjadi input EKF; hanya `/gnss/base_velocity_fusion` yang boleh difuse.
5. COG tidak pernah langsung menjadi yaw correction bersamaan dengan gated COG EKF fusion.
6. Satu epoch GNSS yang terlihat baik **tidak cukup** untuk mengaktifkan fusion. Harus ada run-level certification persisten.

## Arsitektur

```text
ESC wheel odometry -----> Local EKF -----> /odometry/filtered -----> odom -> base_footprint
                              ^
                              |
                         IMU gyro-Z

GNSS position -----------------------------> /odometry/gnss_map ----┐
GNSS Doppler velocity -> timestamp/cov/quality gate -> body vx ----+--> Global EKF
IMU gyro-Z ---------------------------------------------------------+
qualified COG -> certification gate -> map yaw ---------------------+
                                                                    |
                                                        /odometry/filtered_map
                                                                    |
                                                         LocalizationCore
                                                                    |
                                                               map -> odom
```

## Persamaan utama

### Antenna lever arm
GNSS mengukur kecepatan pada titik antena, bukan pada origin kendaraan:

`v_ant = v_base + omega x r`

sehingga:

`v_base = v_ant - omega x r`

Untuk planar vehicle dengan `r=(rx, ry)` dan yaw-rate `wz`:

`vx_base = vx_ant + wz * ry`

`vy_base = vy_ant - wz * rx`

### Transform covariance
Covariance horizontal tidak boleh diputar dengan rotasi vektor biasa. Yang benar:

`Sigma_target = R * Sigma_source * R^T`

Stage 2 memeriksa variance finite, berada pada batas yang diizinkan, dan covariance horizontal positive-semidefinite sebelum measurement boleh qualified.

### Residual wheel vs GNSS

`e_v = v_wheel - v_gnss_base_x`

Residual ini tidak langsung dipakai untuk mengoreksi wheel odometry. Ia dipakai sebagai evidence untuk mendeteksi slip/mismatch dan untuk certification.

### COG residual

`e_psi = wrap(psi_COG - psi_DopplerVelocity)`

COG hanya layak untuk yaw saat kendaraan bergerak maju cukup cepat, yaw-rate kecil, covariance/quality baik, dan tidak ada indikasi slip.

## Gate Stage 2

### A. Prasyarat Stage 1
Sebelum GNSS motion certification:

- `steering_calibration_valid = true`
- `steering_circle_calibration_valid = true`
- `drive_odometry_calibration_valid = true`

### B. IMU stationary certification
GUI IMU sekarang menyimpan evidence:

- jumlah sampel;
- durasi capture;
- standard deviation gyro-Z;
- error mean `|a|` terhadap gravitasi;
- bias gyro/accelerometer;
- covariance hasil stationary capture.

Default gate:

- minimum 80 sampel;
- minimum 8 s;
- `gyro-Z std <= 0.03 rad/s`;
- `||mean(|a|)-g|| <= 0.75 m/s²`.

Setelah Apply, `stationary_calibration_valid=true`. Re-kalibrasi IMU otomatis mencabut sertifikasi GNSS motion karena transform/residual Stage 2 harus diuji ulang.

### C. GNSS timestamp gate
Velocity ditolak dari qualification jika:

- timestamp hilang ketika timestamp diwajibkan;
- timestamp terlalu jauh di masa depan;
- measurement terlalu tua;
- timestamp mundur lebih dari toleransi;
- timestamp duplikat.

Default:

- max future: 0.10 s;
- max age: 1.00 s;
- max regression: 0.02 s.

### D. GNSS covariance gate
Horizontal velocity covariance harus:

- finite;
- diagonal variance `>= 1e-6` dan `<= 1.0`;
- positive-semidefinite.

`NAV-COV vel valid` dan GNSS quality juga harus fresh.

### E. Run-level velocity certification
GUI `Straight Motion Run` menghitung:

- total epoch;
- ratio `velocity_qualified`;
- ratio velocity covariance valid;
- ratio GNSS quality fresh;
- P95 timestamp sync gap;
- P95 absolute wheel-vs-GNSS residual;
- P95 absolute lateral velocity pada base frame.

Default PASS:

- minimal 50 epoch;
- qualified ratio >= 0.85;
- covariance-valid ratio >= 0.85;
- quality-fresh ratio >= 0.85;
- sync-gap P95 <= 0.20 s;
- wheel-GNSS residual P95 <= 0.25 m/s;
- lateral velocity P95 <= 0.15 m/s.

Baru tombol **Certify + Enable EKF Velocity** boleh digunakan.

### F. Run-level COG certification
COG harus melewati velocity certification dan tambahan:

- minimal 30 COG residual epoch;
- `cog_qualified` ratio >= 0.70;
- COG-vs-Doppler P95 <= 10 deg.

Baru tombol **Certify + Enable EKF COG** boleh digunakan. Saat COG EKF aktif, `enable_gnss_course_yaw_correction` dipaksa false untuk mencegah double correction.

## Urutan pengujian lapangan

### 1. Preflight read-only

```bash
ros2 run navigation stage2_commissioning_check.py \
  --workspace ~/Sistem-Otomasi-Car/Car-MiniPC/ros
```

`WAIT` bukan error. Baseline memang WAIT sampai evidence lapangan tersedia.

### 2. IMU stationary

- tempatkan kendaraan di lantai datar dan tidak bergerak;
- pastikan motor/steering tidak membuat getaran;
- capture disarankan 30–60 s walaupun gate minimum 8 s;
- analisis;
- Apply hanya jika PASS;
- restart IMU, EKF, LocalizationCore, autonomous launch.

### 3. GNSS stationary evidence

- tempatkan kendaraan di area langit terbuka;
- capture stationary 1–3 menit;
- simpan CSV untuk hAcc/DOP/sAcc dan position scatter;
- ini evidence receiver; belum mengaktifkan velocity fusion.

### 4. Straight-run velocity certification

Lakukan minimal dua arah pada jalur yang sama agar mounting/heading bias mudah terlihat:

- arah A: 0.2–0.4 m/s, straight;
- arah B: 0.2–0.4 m/s, straight balik;
- hindari akselerasi keras di bagian yang dipakai sebagai evidence;
- GNSS antenna lever arm di YAML harus sesuai pengukuran fisik.

Rekam rosbag bersamaan:

```bash
ros2 run navigation rosbag_regression.py record --name stage2_velocity
```

Di GUI pilih `Straight Motion Run`, lalu `Stop + Analisis`. Hanya jika PASS, lakukan `Certify + Enable EKF Velocity` dan restart LocalizationCore + Global EKF.

### 5. Uji setelah velocity fusion

Ulang straight-run dan pastikan:

- local odom tetap smooth;
- global odom tidak meloncat;
- `/gnss/velocity_fusion_active` hanya true ketika measurement fresh dan qualified;
- GNSS dropout tidak membuat local odom berhenti;
- saat GNSS kembali, `map -> odom` terkoreksi bertahap, bukan teleport.

### 6. COG certification

Setelah velocity fusion stabil:

- jalankan straight pada beberapa heading berbeda;
- pastikan forward speed melewati minimum COG;
- hindari turn selama evidence COG;
- cek residual Doppler-course dan wheel slip;
- `Certify + Enable EKF COG` hanya jika PASS.

### 7. Fault injection

Uji minimal:

- cabut GNSS saat bergerak pelan;
- reconnect GNSS;
- buat GNSS quality buruk/tertutup sebagian;
- cabut IMU (dengan kendaraan aman/tidak autonomous);
- wheel slip terkontrol pada permukaan aman;
- timestamp stale/replay offline.

Target: local odom tetap continuity-bounded dan input global yang tidak qualified tidak diteruskan ke EKF.

## Kapan certification harus dicabut

Wajib revoke dan ulang Stage 2 bila berubah:

- drive odometry scale;
- IMU bias/orientation/mounting;
- GNSS antenna mounting atau lever arm;
- GNSS receiver/firmware/output rate yang mengubah timestamp/covariance;
- frame convention ENU/base;
- wheel radius/gear ratio yang mengubah odometry;
- perubahan besar pada steering geometry yang juga mengubah motion model pengujian.

GUI otomatis revoke saat drive scale atau IMU calibration diaplikasikan ulang. Untuk mounting/lever-arm yang diedit manual, gunakan tombol **Revoke Certifications**.

## Offline review CSV

```bash
ros2 run navigation stage2_motion_csv_analyzer.py path/to/gnss_stage2.csv \
  --localization-yaml ~/Sistem-Otomasi-Car/Car-MiniPC/ros/src/navigation/config/localization_cpp.yaml
```

Analyzer bersifat read-only dan sengaja tidak mengubah YAML. Certification tetap tindakan eksplisit operator di GUI.

## Sign-off Stage 2
Stage 2 selesai hanya bila:

- Stage 1 physical/drive gates PASS;
- IMU stationary certification PASS;
- GNSS velocity certification PASS;
- velocity fusion diuji ulang setelah restart dan stabil;
- COG certification PASS (jika COG akan dipakai);
- GNSS dropout/reconnect tidak menyebabkan pose discontinuity yang membahayakan;
- Global EKF tetap `publish_tf=false`;
- tidak ada dua publisher `map -> odom`;
- rosbag + CSV + YAML snapshot disimpan sebagai evidence.
