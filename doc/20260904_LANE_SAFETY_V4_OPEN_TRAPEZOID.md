# Lane Safety V4 — Open Trapezoid

Tanggal: 2026-09-04

## Tujuan
Preview safety lane menggunakan dua garis miring kiri/kanan seperti sisi trapesium terbuka. Tidak ada garis horizontal atas/bawah dan tidak ada label diagnostik pada gambar.

## Koreksi dari V3
V3 memakai proyeksi ground langsung pada horizon. Pada pitch kamera 0 derajat, endpoint atas menghasilkan half-width 0 px sehingga kedua garis bertemu dan tampak seperti segitiga/V.

V4 memberi batas kedalaman fisik `lane_corridor_far_lookahead_m=4.0`. Endpoint atas tetap mempunyai lebar, sementara endpoint bawah tetap mengikuti tinggi kamera, pitch, intrinsic/FOV, lebar kendaraan, dan safety margin.

Default URDF/config:
- vehicle width = 0.550 m
- camera height from ground = 0.736 m
- camera pitch = 0 deg
- safety margin = 0.25 m per sisi
- far lookahead = 4.0 m
- fx/fy = 910 px pada 1280x720

Geometri default 1280x720:
- endpoint atas sekitar X 520.6 / 759.4
- endpoint bawah sekitar X 414.0 / 866.0
- kedua sisi diinterpolasi lurus agar evaluasi mask sama persis dengan garis yang digambar.
## Kontrak Warna
- GRAY: lane mask tidak ada DAN drivable-area mask tidak ada.
- GREEN: evidence tersedia dan lane tidak mendekati safety line; termasuk drivable terdeteksi tetapi lane mask tidak ada.
- YELLOW: lane mask mendekati safety line sesuai `lane_corridor_warning_gap_px`.
- RED: lane mask menyentuh/masuk safety line sesuai `lane_corridor_touch_margin_px`.

## Kalibrasi
Fine calibration tetap tersedia melalui center shift, left/right shift, camera pitch, camera height, safety margin, top/bottom Y ratio, dan far lookahead. Baseline dapat di-reset melalui konfigurasi baseline web.

## Verifikasi
- Build `perception` Release sequential 1 worker: PASS.
- Autonomous/GUI integration self-check: PASS.
- BAB IV acquisition self-check: 16 PASS.
- Lane safety geometry/color self-check: PASS.
- Runtime error count: 0.
- Live frame: dua sisi trapesium terbuka abu-abu pada scene indoor tanpa lane/drivable mask.
- GoalPose tidak dikirim.
- `cmd_actuator`, `cmd_final`, drive target, steer target tetap 0.

Bukti frame: `log/20260904_lane_safety_v4_live.jpg`.
