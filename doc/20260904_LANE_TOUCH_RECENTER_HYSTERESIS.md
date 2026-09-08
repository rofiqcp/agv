# Lane Safety Touch → Recenter → Release

Tanggal: 2026-09-04
Device: otomasi
Repo: `$AGV_ROOT`

## Tujuan

Safety line adalah batas virtual di dalam area jalan. Lane mask normal harus berada di luar safety line. Sistem tidak boleh menunggu lane menembus jauh sebelum mulai melakukan koreksi.

Kontrak akhir:
- Lane tidak mendekati safety line: GREEN / aman, tanpa koreksi tambahan.
- Lane mendekati tetapi belum menyentuh: YELLOW / warning, tanpa steering correction.
- Lane menyentuh atau masuk safety line: RED dan recenter langsung aktif.
- Lane kiri touch/masuk: `RECENTER_RIGHT`.
- Lane kanan touch/masuk: `RECENTER_LEFT`.
- Kedua sisi touch/masuk bersamaan: fail-safe `BOTH_INTRUSION_STOP`.
- Setelah touch, recenter tetap latched sampai lane kembali keluar melewati release gap.

## Threshold Pixel

- touch gap: 2 px
- warning gap: 36 px
- release gap: 48 px
- correction gain: 0.004 m/px
- maximum correction: 0.35 m

## Perbaikan Bug

Implementasi sebelumnya menandai gap `<= touch_margin` sebagai RED, tetapi correction memakai `penetration_px=max(0,-gap)`. Akibatnya tepat saat touch (gap sekitar 0–2 px), correction masih dapat bernilai nol.

Sekarang correction menargetkan release gap:
`recovery_gap_px = max(0, release_gap_px - gap_px)`.
Dengan demikian tepat saat touch sudah dihasilkan correction positif.

Latch mencegah osilasi: setelah RED, status recovery dipertahankan YELLOW dan correction tetap aktif sampai gap mencapai 48 px. Setelah itu latch dilepas dan status kembali GREEN.

## Validasi

- `perception_safety_core_self_check`: PASS melalui compile/run langsung.
- Left touch menghasilkan angular correction negatif (belok kanan): PASS.
- Right touch menghasilkan angular correction positif (belok kiri): PASS.
- Touch menghasilkan correction > 0: PASS.
- Recovery 40 px masih latched: PASS.
- Release 48 px kembali GREEN dan correction 0: PASS.
- `lane_safety_overlay_geometry_self_check.py`: PASS.
- backend contract: PASS.
- CPU LibTorch runtime contract: PASS.
- runtime robustness contract: PASS.
- Build perception Release sequential 1 worker: PASS.
- Runtime web health `ok=true`; error count 0.
- Verifikasi dilakukan tanpa GoalPose dan tanpa command motor nonzero; drive/steer target tetap 0.

Catatan: physical steering authority tetap mengikuti gate `lane_safety_enabled` dan `camera_metric_calibration_validated` yang sudah ada. Pengujian ini tidak mengaktifkan motor.