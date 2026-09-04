# Audit Perception Toggle dan Observability — 2026-09-04

## Tujuan
Memastikan toggle YOLOPv2 pada mode web benar-benar mengaktifkan/nonaktifkan inference, kamera tetap aktif saat inference OFF, status ROS Web tidak menampilkan telemetry inference yang stale, dan build/test tetap bersih.

## Perbaikan
- `astra_yolop_cpu_pt_node.cpp`: menambahkan payload `/perception/performance` khusus camera-only dengan `backend=camera_only` dan `inference_enabled=false`.
- Payload camera-only dipublish langsung ketika parameter `inference_enabled` diubah ke OFF dan periodik selama capture camera-only.
- Payload inference CPU sekarang eksplisit membawa `inference_enabled=true`.
- `perception_safety_core_self_check.cpp`: menghilangkan warning `unused-but-set-variable` pada Release/NDEBUG tanpa mengubah logika assert.

## Verifikasi Runtime
Clean restart dilakukan pada ROS domain 42 agar tidak memakai proses lama. Hasil REST `/api/perception/inference` dan `/api/state`:
- Initial OFF: `backend=camera_only`, `inference_enabled=false`.
- ON: `backend=cpu`, `inference_enabled=true`, model TorchScript lazy-load PASS.
- OFF kembali: `backend=camera_only`, `inference_enabled=false`.
- Kamera tetap capture/preview ketika inference OFF.
- Tidak ditemukan FATAL/ERROR/process died pada log runtime clean.

## Build dan Test
- Build perception: PASS, 1 package selesai tanpa warning/error pada verifikasi akhir.
- `colcon test --packages-select perception`: 4/4 PASS.
- `colcon test-result`: 4 tests, 0 errors, 0 failures, 0 skipped.
- Full audit sebelumnya pada workspace: 4/4 package build PASS dan total 48 tests PASS.

## Status Sistem Saat Runtime
- Map dan Nav2 lifecycle berhasil aktif.
- Kamera, GNSS, dan IMU terdeteksi.
- GNSS sedang DEGRADED pada pengujian indoor/lokasi saat ini karena DOP/hAcc/satelit, sehingga motion localization gate tetap CLOSED sesuai safety design.
- ESC sempat terdeteksi pada run sebelumnya, tetapi pada clean run terakhir status USB ESC belum ready; ini merupakan kondisi hardware/runtime yang harus dipastikan sebelum uji gerak autonomous.
- Software tidak memaksa autonomous motion ketika localization/calibration/hardware gate belum memenuhi syarat.

## Catatan Operasional
Workspace menggunakan `ROS_DOMAIN_ID=42` dan `ROS_LOCALHOST_ONLY=1` untuk stack ini. Untuk inspeksi CLI, gunakan environment yang sama atau restart ROS daemon CLI agar node graph tidak terlihat kosong akibat daemon domain lama.
