# Lane Safety V5 — Full Height Open Trapezoid

Tanggal: 2026-09-04

## Tujuan
Safety line harus berupa dua sisi trapesium terbuka yang membentang dari ujung atas preview hingga ujung bawah preview, tanpa garis horizontal atas/bawah.

## Perubahan
- `lane_corridor_top_y_ratio`: 0.50 -> 0.02.
- `lane_corridor_bottom_y_ratio`: 0.94 -> 0.98.
- Clamp top ratio diperluas agar mendukung full-height.
- Baseline YAML dan baseline reset disamakan.

## Bentuk Default
- Bagian atas tetap memiliki jarak antargaris.
- Bagian bawah lebih lebar.
- Kedua garis tidak bertemu membentuk V/segitiga.
- Tidak ada garis horizontal.

## Logika Warna
- Abu-abu: lane mask dan drivable-area mask sama-sama tidak tersedia.
- Hijau: kondisi aman.
- Kuning: lane mask mendekati safety line.
- Merah: lane mask menyentuh atau masuk ke safety line.

## Verifikasi Live
- Build package `perception` Release: PASS.
- Frame live: `/home/otomasi/ros/log/20260904_lane_safety_v5_fullheight_live.jpg`.
- Source frame: `yolop_annotated`.
- Pada scene indoor tanpa lane/drivable evidence, kedua garis tampil abu-abu.
- Garis membentang dari sekitar Y=2% hingga Y=98% frame.
- Runtime error count sebelumnya tetap 0.
- `cmd_actuator=0`, `cmd_final=0`, drive target 0, steer target 0.
- Goal tetap IDLE dan autonomy tetap false.
