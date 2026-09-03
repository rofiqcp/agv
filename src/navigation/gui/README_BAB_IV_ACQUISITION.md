# Akuisisi Data GUI untuk BAB IV

GUI menyediakan tiga workspace akuisisi yang dipilih melalui satu floating menu `☰` kiri atas. Menu mempunyai tab **Navigasi**, **Persepsi**, dan **ESC**, masing-masing dengan group 4.1, 4.2, dst.:

- **Akuisisi BAB IV — Navigasi**: 53 bentuk tabel dan 48 format grafik dari laporan navigasi, termasuk seluruh pengujian 4.9.
- **Akuisisi BAB IV — Persepsi**: 32 bentuk tabel dan 10 format grafik dari laporan persepsi visual.
- **Akuisisi BAB IV — ESC / FOC**: 15 bentuk tabel dan 20 format grafik dari laporan steering/FOC.

Setiap subpengujian mempunyai kolom tabel sesuai laporan, grafik live atau ringkasan, identitas variasi/kondisi, ground truth opsional, rekaman sampel mentah, dan tombol **Simpan CSV + PNG + Raw**. Hasil disimpan secara default ke `~/.ros/agv_gui_reports` bersama manifest JSON.

## Arti warna sel

- Hijau: nilai dihitung atau diambil otomatis dari topic/source yang tersedia.
- Kuning: nilai perlu diisi dari ground truth, alat ukur eksternal, atau hasil observasi operator.

Nilai berlabel "DATA ESTIMASI" di laporan hanya dipakai untuk menentukan bentuk tabel/grafik. Nilai estimasi tersebut tidak disalin sebagai hasil pengujian aktual.

## Topic penting

Navigasi memakai topic GNSS, IMU, odometry, EKF, localization, command chain, `/plan`, `/local_plan`, dan status tuning yang sudah ada. Persepsi memakai detection, obstacle metrics, lane/drivable-space, camera health, near-field, safety, dan performance topic CPU/GPU yang sudah ada.

Steering target/actual dan RPM tersedia dari topic ESC yang sudah ada. Kolom arus/tegangan FOC (`Id`, `Iq`, `Vd`, `Vq`, dan `Vbus`) akan ditandai belum tersedia sampai sistem memublikasikan `std_msgs/msg/String` pada `/esc/foc/telemetry`. GUI sengaja tidak mengarang nilai pengganti.

## Build dan run

```bash
cd /home/otomasi/mobil_stage3_ws
colcon build --packages-select navigation --symlink-install
source install/setup.bash
export QT_QPA_PLATFORM=xcb
ros2 launch navigation gui.launch.py
```

Perbaikan ini tidak mengubah `autonomous.launch.py`, konfigurasi Nav2, maupun konfigurasi RViz. Jalankan autonomous seperti biasa; paket ini hanya menimpa file GUI dan self-check terkait GUI.
