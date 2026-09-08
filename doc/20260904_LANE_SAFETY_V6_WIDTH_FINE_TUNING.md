# Lane Safety V6 – Fine Tuning Lebar Safety Line

Tanggal: 2026-09-04
Device: otomasi
Repo: `$AGV_ROOT`

## Tujuan
Melebarkan safety line sedikit dari baseline V5 tanpa membuat sisi atas terlalu terbuka atau mengubah bentuk open-trapezoid full-height.

## Baseline Baru
- `lane_corridor_top_y_ratio = 0.02`
- `lane_corridor_bottom_y_ratio = 0.98`
- `lane_corridor_safety_margin_m = 0.30`
- `lane_corridor_far_lookahead_m = 3.8`
- `lane_corridor_camera_height_m = 0.736`
- `lane_corridor_camera_pitch_deg = 0.0`

## Perubahan Geometri
Dibanding V5 (`margin=0.25`, `far=4.0`), lebar proyeksi bertambah secara proporsional:
- bagian atas sekitar +15.3%
- bagian bawah sekitar +9.5%

Endpoint nominal pada 1280x720:
- atas: X ≈ 502.3 sampai 777.7
- bawah: X ≈ 370.0 sampai 910.0

## Verifikasi
- Build `perception` Release + 1 worker: PASS.
- Lane geometry self-check: PASS.
- Autonomous/GUI integration self-check: PASS.
- BAB IV acquisition self-check: 16 PASS.
- Web health: `ok=true`.
- Runtime error count: 0.
- Live frame: `log/20260904_lane_safety_v6_width_live.jpg`.

## Safety
Selama build, respawn, dan live verification:
- Goal tetap `IDLE`.
- autonomy tetap `false`.
- `cmd_actuator = 0`.
- `cmd_final = 0`.
- `esc_drive_target = 0`.
- `esc_steer_target = 0`.
- Tidak ada GoalPose atau command motor nonzero yang dikirim.
