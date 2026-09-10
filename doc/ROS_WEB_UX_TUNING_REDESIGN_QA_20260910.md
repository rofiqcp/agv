# ROS Web UX Tuning Redesign — QA Status 2026-09-10

Target: `/home/sirobo/agv/src/navigation/web`
Checklist utama: `doc/ROS_WEB_UX_TUNING_REDESIGN_PLAN.md`

## Hasil verifikasi sesi ini

- Build `navigation`: PASS (`colcon build --symlink-install --packages-select navigation`).
- ROS Web runtime di-restart dalam mode `read_only:=true` dan aktif di `127.0.0.1:5000`.
- `/api/health`: `ok=true`, `ros=true`, `read_only=true`.
- `/api/config/state` dan `/api/config/schema`: PASS.
- `/api/config/validate`: PASS dengan current-value payload dan terbukti tidak menulis YAML.
- `/api/config/apply`: HTTP 403 pada read-only, sesuai safety contract.
- `web_workspace_tabs_self_check.py`: PASS.
- `colcon test --packages-select navigation`: 52 tests, 0 failures, 0 errors, 0 skipped.
- `git diff --check`: PASS.

## Browser QA

`node scripts/qa_ros_web_playwright.js` PASS pada:
- 1920x1080 desktop
- 1366x768 laptop
- 1024x768 tablet
- 390x844 mobile
Browser QA memverifikasi tanpa error:
- true workspace tab/domain traversal
- tepat satu `.page.active`
- tidak ada document horizontal overflow
- tidak ada focusable control di hidden workspace pane
- tidak ada invisible pointer blocker besar
- TASK / TUNE / ANALYZE
- BASIC / ADVANCED / EXPERT
- Help Drawer open/close
- 0 console/page errors
- 0 HTTP >=400 (di luar resource visual yang memang dikecualikan)
- 0 request failure
- 0 POST otomatis saat UI read-only dibuka/ditelusuri

Screenshot regression tersimpan di `/tmp/agv_ros_web_qa_20260910/`:
`desktop.png`, `laptop.png`, `tablet.png`, `mobile.png`.

## Staged tuning/config semantics

- Input tuning menulis ke browser Draft (`configPending`), bukan langsung ke YAML.
- `Validate` memakai `/api/config/validate`.
- `Apply + Save` memakai batch transactional `/api/config/apply`.
- `Revert Draft` hanya membuang Draft browser.
- `Stage Baseline` membuat Draft baseline; tidak langsung menulis.
- Backend transaction mempunyai per-file backup, atomic set, rollback partial write, runtime apply/read-back, dan rollback YAML saat `RUNTIME_MISMATCH`.
## Perbaikan yang dilakukan pada sesi QA

Ditemukan empat field Basic RUN/GT locked tanpa context-help trigger. Renderer `decorateTuningFieldStates()` diperbaiki agar field non-YAML/locked juga mempunyai metadata help dan tombol `?` tanpa mengubah write path.

Verifikasi setelah fix:
- Basic visible: 8, missing help: 0.
- Advanced visible: 9, missing help: 0.
- Full Playwright di empat viewport: PASS ulang.

## Item plan yang belum selesai

CSS sudah menggunakan satu stylesheet aktif (`styles.css`) dan `vesc_mp_theme.css` sudah tidak dilink serta dihapus dari source, tetapi konsolidasi internal belum tuntas. Audit exact-selector kasar masih menemukan 374 selector berulang; `.topbar` muncul 10 kali dan sejumlah komponen/layout lain masih mempunyai beberapa generasi rule. Ini technical debt P0 pada Phase 2 dan tidak aman di-auto-dedupe karena cascade/order dapat mengubah layout commissioning.

Karena itu redesign belum boleh dinyatakan 100% selesai terhadap seluruh plan sampai CSS internal dikonsolidasikan menjadi token/layout/component/page layer yang authoritative dan seluruh QA diulang.

## Hardware QA terpisah

`STRICT_HARDWARE=1 node scripts/qa_ros_web_playwright.js` gagal pada `connected.camera=false`. Ini blocker hardware/runtime kamera, bukan kegagalan layout. Layout/read-only QA tetap PASS dan strict hardware QA sengaja dipertahankan terpisah sesuai plan.
