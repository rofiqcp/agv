# Audit Runtime ROS dan Integrasi Laporan BAB IV — 2026-09-04

## Ruang lingkup
Audit workspace `/home/otomasi/ros` mencakup paket `esc`, `navigation`, `perception`, dan `stmf4`, konfigurasi YAML, launch ROS 2, self-check, lifecycle runtime, serta integrasi ROS Web.

## Temuan dan perbaikan
- Ditemukan instance stack ROS lama berjalan bersamaan dan memicu konflik lock HMI/port web; proses duplikat dibersihkan sebelum pengujian ulang.
- Lifecycle `semantic_obstacle_node.py` telah dibuat aman terhadap shutdown ROS agar tidak publish setelah context invalid dan tidak melakukan shutdown ganda.
- `trajectory_safety.command_collision_horizon_m` diselaraskan menjadi 2.0 m sehingga lebih kecil daripada `slow_path_distance_m` 2.2 m dan kontrak safety kembali konsisten.
- Self-check katalog Navigasi diperbarui mengikuti katalog aktual minimal 84 leaf.
- Kontrak Mini-PC diperbarui mengikuti arsitektur lazy YOLOPv2: default OFF, inference OFF, dan CPU thread dibatasi 2.
- Sidebar Navigasi memperoleh kelompok Laporan BAB IV 4.1.1–4.4.3 yang memakai source tuning existing, recorder 1 Hz, YAML tuning, CSV/Excel, grafik, dan rekap data.
- Export Excel menambahkan kolom parameter tuning terpisah dengan style ungu agar nilai tuning mudah dibedakan dari measurement.
- Inferensi kolom laporan diperluas untuk planner success, path endpoint error, GNSS age/rate, dan residual IMU.

## Validasi sementara
- Syntax Python: PASS.
- XML package: PASS.
- YAML parse: PASS.
- Shell syntax: PASS.
- Launch Python syntax/import: PASS.
- ROS package resolution: PASS untuk `esc`, `navigation`, `perception`, `stmf4`.
- Regression self-check sebelum perubahan web terbaru: 41 PASS / 0 FAIL.

## Catatan verifikasi lanjutan
Setelah commit/sinkronisasi, wajib dijalankan kembali full self-check, build dengan maksimum sekitar 70% CPU/RAM, `colcon test`, lalu runtime `autonomous.launch.py mode:=web` dan pemeriksaan node/topic/log/resource sebelum dinyatakan siap uji penuh.
