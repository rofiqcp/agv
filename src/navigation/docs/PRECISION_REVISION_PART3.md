# Precision Revision V9 — Part 3

Part 3 memfinalkan workflow commissioning menjadi workflow production/tuning yang reproducible tanpa mengubah prinsip sensor fusion Part 1–2.

## 1. Velocity smoother CLOSED_LOOP qualification
`mppi_closed_loop_supervisor` sekarang mengukur window feedback aktual dan menerbitkan:

- `/navigation/mppi_closed_loop/ready`
- `/navigation/velocity_smoother/closed_loop_eligible`
- `/navigation/velocity_smoother/qualification`

Qualification mempertimbangkan rate `/odometry/filtered`, jitter interval odometry, velocity RMSE, steering RMSE, yaw-rate RMSE, freshness ESC dan freshness EKF. Supervisor **tidak** mengganti mode smoother saat kendaraan bergerak. GUI hanya mengizinkan penulisan `CLOSED_LOOP` setelah eligibility PASS; perubahan tetap membutuhkan restart lifecycle velocity_smoother.

## 2. MPPI A/B/C sweep
Halaman Navigation memiliki tiga baseline profile yang sengaja dibatasi oleh `vehicle.max_forward_speed_mps`:

- A — Konservatif
- B — Balanced
- C — Responsif

`Generate Sweep Pack` membuat tiga YAML + `manifest.json`. `Apply Profile` membuat backup konfigurasi aktif sebelum menulis YAML. Setiap field-test dapat direkam menjadi CSV + metadata + automatic numeric summary.

## 3. Collision Monitor preview vs production
Dua file dipisahkan secara eksplisit:

- `collision_monitor.yaml`: preview/commissioning, output `/cmd_vel/collision_preview`.
- `collision_monitor_production.yaml`: production safety owner, output `/cmd_vel`.

`autonomous.launch.py` hanya memakai konfigurasi production ketika `enable_collision_monitor=true`. Default launch arg dibaca dari `navigation_core.yaml`, dan nilai tersebut juga diinjeksi kembali ke NavigationCore agar ownership command selalu konsisten.

Production chain:

`MPPI -> velocity_smoother -> trajectory_safety -> NavigationCore -> /cmd_vel/nav2_pre_collision -> Collision Monitor -> /cmd_vel -> ESC mux`

## 4. Camera metric safety certification
Camera GUI menambahkan gate certification. Production collision hanya disarankan aktif bila:

- geometri homography/metric valid,
- kamera connected,
- `/perception/camera_healthy=true`,
- minimal 6 validation points,
- measured RMSE <= configured threshold.

Certification menulis status persisten ke YAML dan mengaktifkan `require_camera_metric_calibration`, `camera_metric_calibration_validated`, dan `collision_monitor_enabled`. Restart autonomous launch tetap diperlukan.

## 5. Reporting BAB IV
Setiap CSV kini mendapat sidecar `.meta.json` yang mencakup:

- session/category/variant/notes,
- snapshot + SHA256 seluruh YAML,
- numeric summary setiap kolom,
- RMSE/mean-absolute/max-absolute untuk kolom error/residual,
- runtime identity (host/platform/Python, Git commit bila tersedia).

Tool `experiment_rank.py` merangkum seluruh sidecar dan menghasilkan `experiment_ranking.csv`.

## 6. Rosbag regression
`rosbag_regression.py` menyediakan daftar topic standar untuk record/replay. Replay harus dilakukan dengan ESC disarmed / physical autonomy gate tertutup.

## 7. Runtime profiling
`runtime_profile.py` merekam CPU/RAM dan, bila tersedia, output Jetson `tegrastats`, sehingga profile TUNING dan DEPLOYMENT dapat dibandingkan berdasarkan data.

## 8. Launch reliability
Destructive stale-process cleanup tidak lagi berjalan otomatis. `AGV_CLEAN_STALE_RUNTIME=1` diperlukan untuk mengaktifkannya. Ini mencegah rosbag/diagnostic process lain terbunuh hanya karena berasal dari workspace yang sama.

## Urutan validasi Part 3
1. Jalankan semua calibration Part 1–2 dan `Verify Runtime`.
2. Jalankan MPPI feedback qualification dalam beberapa straight/turn run.
3. Tetap OPEN_LOOP bila eligibility gagal.
4. Jika PASS, hentikan kendaraan, ubah smoother ke CLOSED_LOOP, restart lifecycle dan ulangi run yang sama.
5. Jalankan MPPI A/B/C dengan route dan kondisi lingkungan yang sama.
6. Gunakan CSV/meta/ranking untuk memilih profile berdasarkan error + safety, bukan hanya kecepatan.
7. Validasi camera metric terhadap titik/jarak fisik dan isi RMSE + jumlah titik.
8. Certify camera metric safety hanya setelah threshold lulus.
9. Restart autonomous launch dan verifikasi production Collision Monitor command chain.
10. Rekam rosbag regression dan runtime profile untuk profile final.
