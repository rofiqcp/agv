# Lane Safety V3 — Geometri & Kalibrasi

Tanggal: 2026-09-04

## Tujuan
Mengganti overlay trapesium lama menjadi dua garis safety miring yang bersih dan berbasis geometri fisik ADV.

## Perubahan Visual
- Menghapus garis horizontal atas/bawah sehingga preview tidak lagi membentuk trapesium tertutup.
- Menghapus tulisan `TRAPEZOID`, `ROAD UNKNOWN`, `L/R UNKNOWN`, dan label diagnostik lain dari frame kamera.
- Drivable tidak terdeteksi: kedua garis safety berwarna abu-abu.
- Drivable terdeteksi tanpa lane mask: garis safety hijau.
- Lane mendekati batas: kuning; lane menyentuh/menembus batas: merah sesuai sisi.

## Baseline Geometri
- Lebar kendaraan mengikuti URDF `total_width = 0.550 m` dan parameter `lane_vehicle_width_m = 0.55`.
- Tinggi kamera dari ground = `vehicle_top_from_ground 0.700 + mount 0.036 = 0.736 m`.
- Pitch baseline mengikuti URDF = `0 deg`.
- FOV menggunakan intrinsics kamera runtime/config: `fx=910`, `fy=910`, `cx=640`, `cy=360` pada 1280×720.
- Margin safety default = `0.25 m` per sisi.
- Baseline 1280×720: top row Y=360 bertemu di X=640; bottom row Y=677 sekitar X kiri=413.9 dan kanan=866.1.

## Kalibrasi Web
Parameter 4.5.2 sekarang menyediakan camera height, camera pitch, safety margin, pair center shift, left line shift, right line shift, serta top/bottom Y ratio.
Reset baseline juga diaktifkan untuk domain Persepsi agar kalibrasi dapat dikembalikan ke nilai fisik awal.

## Verifikasi
- Build `perception` Release, sequential 1 worker: PASS (`Summary: 1 package finished`).
- Self-check keseluruhan: `42 PASS, 0 FAIL`.
- Live frame: `/home/otomasi/ros/log/20260904_lane_safety_v3_live.jpg`.
- Live indoor tanpa drivable/lane: hanya dua garis miring abu-abu; tanpa garis horizontal dan tanpa label status.
- Kontrak sintetis PASS: no-drivable→GRAY, drivable/no-lane→GREEN, warning→YELLOW, touch→RED.
- Calibration shift PASS untuk center, left, dan right offset independen.
- Safety selama pengujian: Goal IDLE, autonomy false, `cmd_actuator=0`, `cmd_final=0`, drive target=0, steer target=0.
