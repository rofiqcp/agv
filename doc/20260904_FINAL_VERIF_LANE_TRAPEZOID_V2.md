# Final Verification Live Lane Trapezoid v2 — 2026-09-04

## Scope
- Workspace: `$AGV_ROOT`
- Mode: `autonomous.launch.py mode:=web`
- Safety constraint: tidak mengirim GoalPose dan tidak mengirim request command motor nonzero.
- ROS runtime: `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`.

## Build
- `perception`: PASS, sequential 1 worker, selesai sekitar 1 menit.
- `navigation`: PASS, Release + sequential 1 worker, selesai 7 menit 31 detik.
- Build `RelWithDebInfo` dihentikan karena konsumsi RAM melewati target; Release menjaga resource lebih aman.
- Self-check sebelum final run: 41/41 PASS.

## Live verification
- Web server: `127.0.0.1:5000` LISTEN.
- Camera: healthy, source `yolop_annotated`.
- Inference YOLOPv2: ON untuk verifikasi visual.
- Frame bukti: `log/20260904_lane_trapezoid_v2_live.jpg` (640x360 JPEG).
- Trapezoid v2 terlihat nyata pada frame dan melebar ke bawah sesuai konfigurasi corridor.
- Overlay live: `TRAPEZOID`, `ROAD UNKNOWN`, `L UNKNOWN`, `R UNKNOWN`, `DRIVABLE UNKNOWN`.
- Status UNKNOWN sesuai kondisi kamera indoor/non-jalan; bukan fault pipeline.

## Safety/runtime result
- `cmd_actuator`: linear 0, angular 0.
- `cmd_final`: linear 0, angular 0.
- `cmd_perception_advisory`: linear 0, angular 0.
- `cmd_pre_collision`: linear 0, angular 0.
- `esc_drive_target = 0` dan `esc_steer_target = 0`.
- Navigation goal tetap `IDLE`, target `NONE`, origin `NONE`.
- `system.autonomy_ready = false` dan `system.motion_ready = false`.
- ESC drive/steer/armed/ready tidak aktif pada snapshot verifikasi.
- Raw steering protocol dapat menunjukkan offset center kalibrasi walau physical steering target = 0; ini merupakan representasi center tersimpan, bukan GoalPose atau request gerak nonzero dari operator.

## Lane state pada scene uji
- Corridor: `enabled=true`, `control_enabled=true`.
- `drivable_detected=false`, `drivable_fraction=0`, status kiri/kanan `UNKNOWN`.
- Lane state: `LANE_LOST` dengan reason `lane_geometry_invalid`, karena scene indoor tidak memiliki lane/drivable road yang valid.
- Camera/pipeline tetap sehat; tidak ditemukan `ERROR`, `FATAL`, `Traceback`, atau `CPU_PIPELINE_ERROR` pada log final.

## Artefak
- Runtime log: `log/20260904_lane_trapezoid_v2_final_live.log`.
- Build perception: `log/20260904_lane_seq_perception.log`.
- Build navigation Release: `log/20260904_lane_seq_navigation_release.log`.
- Live frame: `log/20260904_lane_trapezoid_v2_live.jpg`.
