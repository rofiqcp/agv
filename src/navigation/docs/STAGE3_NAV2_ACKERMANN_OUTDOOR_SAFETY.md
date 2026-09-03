# Stage 3 — Nav2 Ackermann Outdoor, MPPI, Smoother, Trajectory Safety, dan Production Sign-off

## Tujuan
Stage 3 adalah lapisan terakhir setelah geometri/odometri Stage 1 dan localization Stage 2 selesai. Fokusnya adalah memastikan **setiap command yang benar-benar menuju aktuator tetap feasible untuk Ackermann**, kemudian mengumpulkan evidence MPPI, trajectory-safety, Collision Monitor, dan fault injection sebelum production autonomy dibuka permanen.

## Perbaikan kinematik utama
Untuk kendaraan Ackermann dengan minimum turning radius `Rmin`, command harus selalu memenuhi:

`|omega| <= |v| / Rmin`

Batas ini sekarang diterapkan lagi pada `NavigationCore` dan `TrajectorySafetySupervisor`. Ini penting karena sebuah safety layer dapat menurunkan `v`; bila `omega` tidak ikut diturunkan maka radius command justru menjadi lebih kecil.

## Production gate dan commissioning mode
Baseline final:

- `production_autonomy_certified: false`
- `stage3_commissioning_mode:=false`

Artinya physical Nav2 motion tetap fail-closed setelah Stage 1/2 sampai Stage 3 selesai. Untuk pengujian Stage 3, operator harus eksplisit menjalankan:

```bash
ros2 launch navigation autonomous.launch.py stage3_commissioning_mode:=true
```

Commissioning mode membatasi kecepatan autonomous ke `0.18 m/s` (default) dan tetap mempertahankan seluruh gate Stage 1/2.

## Smac Hybrid-A*
Baseline production tetap `DUBIN` dan `vx_min=0`, sehingga planner/controller sama-sama forward-only. `minimum_turning_radius` Smac wajib sama dengan authority `vehicle.yaml` dan MPPI.

## MPPI
`model_dt` harus sama dengan periode controller untuk baseline ini. Profile A/B/C GUI sekarang tidak hanya mengubah `vx_max`; apply profile juga memperbarui `wz_max`, velocity smoother, dan trajectory safety agar tidak melanggar `v/Rmin`.

## Velocity smoother
OPEN_LOOP tetap default. CLOSED_LOOP hanya boleh dipakai setelah feedback supervisor menunjukkan odometry rate/latency dan RMSE lulus. Stage 3 menyimpan certification terpisah sehingga mode CLOSED_LOOP tidak boleh menjadi pilihan production hanya dari satu status sesaat.

## Safety chain

```text
Smac path
  -> MPPI /cmd_vel_nav_raw
  -> velocity_smoother /cmd_vel_nav_smoothed
  -> TrajectorySafety /cmd_vel/autonomy_integrated
  -> NavigationCore /cmd_vel/nav2_pre_collision
  -> Collision Monitor /cmd_vel
  -> ESC mux /cmd_vel/actuator
```

Tidak boleh ada publisher autonomous kedua ke `/cmd_vel`.

## Urutan commissioning
1. Jalankan `stage3_commissioning_check.py` dan pastikan hanya WAIT, bukan CONFIG ERROR.
2. Jalankan autonomy dengan `stage3_commissioning_mode:=true`.
3. Profile A: straight, gentle turn, S-curve, 90-degree feasible turn, goal approach.
4. Profile B: ulangi rute yang sama dan rekam CSV/rosbag.
5. Profile C hanya jika B stabil dan geometry limit tidak pernah tersaturasi.
6. Analisis CSV dengan `stage3_nav2_csv_analyzer.py`.
7. Simpan evidence MPPI dengan `stage3_certify.py mppi` hanya bila analyzer PASS. Tool menyimpan path evidence + SHA-256.
8. Uji obstacle on-path, obstacle outside-turn, blocked corridor, lane warning, camera stale, dan emergency stop; simpan evidence file lalu mark trajectory/collision.
9. Fault injection minimal: GNSS disconnect/reconnect, IMU dropout, ESC/Prolific disconnect, camera dropout, stale perception, Nav2 cancel, dan E-STOP. Kendaraan harus stop/fail-closed sesuai owner layer.
10. `stage3_certify.py finalize` hanya berhasil bila seluruh prerequisite persistent PASS dan file evidence masih ada dengan SHA-256 yang sama. Evidence yang diubah/dihapus setelah certification membuat preflight gagal.

## Command penting

```bash
ros2 run navigation stage3_commissioning_check.py --workspace ~/Sistem-Otomasi-Car/Car-MiniPC/ros

ros2 run navigation stage3_nav2_csv_analyzer.py /path/mppi_tuning.csv

ros2 run navigation stage3_certify.py --workspace ~/Sistem-Otomasi-Car/Car-MiniPC/ros \
  mppi --csv /path/mppi_tuning.csv --profile B

ros2 run navigation stage3_certify.py --workspace ~/Sistem-Otomasi-Car/Car-MiniPC/ros \
  mark trajectory_safety --evidence /path/trajectory_safety_test.txt

ros2 run navigation stage3_certify.py --workspace ~/Sistem-Otomasi-Car/Car-MiniPC/ros \
  mark collision_monitor --evidence /path/collision_monitor_test.txt

ros2 run navigation stage3_certify.py --workspace ~/Sistem-Otomasi-Car/Car-MiniPC/ros \
  mark fault_injection --evidence /path/fault_injection_test.txt

ros2 run navigation stage3_certify.py --workspace ~/Sistem-Otomasi-Car/Car-MiniPC/ros finalize
```

## Kriteria sign-off
Production autonomy tidak boleh disertifikasi sebelum Stage 1, IMU, GNSS velocity, camera metric, MPPI profile, trajectory safety, Collision Monitor, dan fault-injection evidence PASS. COG hanya diwajibkan bila COG fusion memang diaktifkan.


## Audit trail evidence
Setiap certification Stage 3 menyimpan lokasi evidence dan SHA-256. `stage3_commissioning_check.py` memverifikasi ulang hash tersebut. Karena path bersifat lokal robot, simpan CSV/log/rosbag evidence pada lokasi persisten dan jangan menghapusnya setelah sign-off.
