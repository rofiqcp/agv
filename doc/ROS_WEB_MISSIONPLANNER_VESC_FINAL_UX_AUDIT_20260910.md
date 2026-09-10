# ROS Web UX Final Audit — MissionPlanner + VESC Tool

**Tanggal:** 2026-09-10  
**Target:** `/home/sirobo/agv/src/navigation/web`  
**Acceptance source:** `doc/ROS_WEB_UX_TUNING_REDESIGN_PLAN.md`  
**Referensi lokal:** `ardupilot/MissionPlanner`, `esc/vesc_tool`

## 1. Executive Summary

Audit ini membandingkan implementasi ROS Web aktual dengan pola engineering UX yang terbukti di MissionPlanner dan VESC Tool, lalu memverifikasi runtime read-only di `localhost:5000`.

Fondasi redesign P0 sudah masuk: true workspace panes, TASK/TUNE/ANALYZE, BASIC/ADVANCED/EXPERT, reusable Help Drawer, staged config shared antara Tuning dan Configuration, no auto-write dari tuning, adapter config `schema/state/validate/apply/revert`, transaction backup/rollback, dan runtime readback status.

QA final juga diperketat dari 4 menjadi 9 viewport sesuai plan. Audit tambahan ini menemukan dan memperbaiki satu overlap nyata pada mobile 430 px: aturan akhir `.workspace-tabs{top:var(--topbar-h)}` mengalahkan breakpoint lama yang seharusnya `top:auto`, sehingga workspace tab bertabrakan dengan complexity controls. Fix eksplisit mobile telah diterapkan dan QA 9 viewport lulus.

Namun acceptance plan secara keseluruhan **belum boleh dinyatakan 100% selesai**. Masih ada gap arsitektural P1/P2 yang membutuhkan implementasi lanjutan, terutama metadata schema authoritative, modularisasi CSS/JS, guided calibration completeness, dan screenshot regression comparison otomatis.
## 2. Pola VESC Tool yang Relevan

Referensi konkret yang diaudit:
- `esc/vesc_tool/configparams.h` / `configparams.cpp`: parameter sebagai object metadata, bukan field UI yang hard-coded tersebar.
- `esc/vesc_tool/startupwizard.cpp`: `QWizard` + `QWizardPage`; title/description berasal dari `ConfigParam.longName` dan `description`.
- `esc/vesc_tool/parametereditor.cpp`: editor mengonsumsi metadata parameter termasuk long name dan description.
- `esc/vesc_tool/res/qml/Examples/ParamTableAndPlot.qml`: pemisahan parameter/config dari realtime plotting dapat dijadikan pola visual.

### Adopsi yang tepat
1. Satu metadata source untuk label, description, min/max/step, unit, options, risk, dan apply mode.
2. Guided wizard untuk pekerjaan yang punya urutan/prerequisite: steering center/endpoints, speed scale, IMU, camera metric, lane/ROI.
3. Pisahkan Live/Realtime dari Tune/Configuration dan Maintenance/Utilities.
4. Context help reusable, bukan paragraf permanen di setiap card.
5. Compare/config-difference dipakai sebagai engineering evidence, bukan hanya tombol save.

### Yang tidak diadopsi mentah
- Kepadatan desktop VESC Tool tidak cocok untuk touchscreen/operator lapangan.
- Terminal/firmware/config tidak boleh memenuhi primary Live workspace.
- Raw parameter names tidak boleh menjadi default label operator.
- Wizard tidak boleh mengaktifkan motion/safety flag hanya karena langkah visual selesai.
## 3. Pola MissionPlanner yang Relevan

Referensi konkret yang diaudit:
- `ardupilot/MissionPlanner/Controls/MavlinkNumericUpDown.cs`: range dan increment berasal dari `ParameterMetaDataRepository`.
- `ardupilot/MissionPlanner/Controls/MavlinkComboBox.cs`: enum/options berasal dari metadata parameter.
- `ardupilot/MissionPlanner/Controls/paramcompare.cs`: compare state parameter.
- `ConfigRawParams` dan changelog terkait: full parameter browser, modified/favorites, restore old value, partial-failure handling, reboot semantics.

### Adopsi yang tepat
1. Friendly/task configuration sebagai default; Full Parameters hanya Expert.
2. Type-aware editor: numeric, enum, boolean, bitmask, bukan raw text seragam.
3. Range/increment/options harus berasal dari metadata authoritative.
4. Baseline/YAML/Draft/Runtime compare dan modified/staged filter.
5. Calibration menjadi guided workflow dengan readiness/progress/error recovery.
6. Apply result harus menjelaskan runtime applied vs restart/reload required.

### Yang tidak diadopsi mentah
- Nested tabs dan desktop legacy density.
- Dialog blocking untuk aksi kecil.
- Banyak singkatan tanpa inline/context help.
- Menampilkan semua parameter hanya karena tersedia.
## 4. Gap ROS Web Saat Ini

### P0 — sudah diterapkan dan terverifikasi
- True workspace pane menggantikan primary scroll-to-anchor.
- Inactive pane memakai hidden/aria-hidden dan tidak focusable.
- TASK/TUNE/ANALYZE aktif sebagai mode workspace.
- BASIC/ADVANCED/EXPERT aktif; Expert tidak menjadi default setelah refresh.
- Help Drawer reusable dengan event delegation `data-help-key`.
- Tuning input hanya membuat Draft; tidak auto-write YAML.
- Tuning dan Configuration memakai `configPending` yang sama.
- `POST /api/config/validate` memvalidasi batch tanpa write.
- `POST /api/config/apply` membuat transaction backup, atomic writes, runtime apply/readback, rollback pada save failure/runtime mismatch.
- `POST /api/config/revert` tersedia; read-only menolak apply.
- `vesc_mp_theme.css` tidak lagi menjadi stylesheet kedua dan sudah dihapus dari static source.
- Legacy domain switch serta phase/family pseudo tabs sudah dihapus.

### P1 — belum complete
- `/api/config/schema` belum merupakan normalized metadata schema per-field yang authoritative untuk unit/range/step/options/risk/dependency/apply capability; saat ini terutama mengembalikan snapshot files + capabilities.
- Help parameter masih sebagian dibangun heuristically di frontend; belum single source of truth yang direview untuk semua Basic/Advanced parameter.
- Guided calibration belum dibuktikan lengkap untuk semua target plan: steering endpoints, speed scale, IMU/magnetometer, camera metric/homography, lane/ROI, obstacle qualification.
- Pre-apply compare sudah punya state Baseline/YAML/Draft/Runtime, tetapi belum menjadi satu diff review surface yang konsisten untuk semua batch.

### P2 — belum complete
- `styles.css` sudah satu file authoritative, tetapi source masih monolitik/append-history; belum modular token/base/layout/components/pages sesuai target arsitektur.
- `app.js` masih monolitik; TrueTabs/help/config/tuning belum dipisah menjadi modules.
- Screenshot artifacts sudah diambil, tetapi automated pixel/snapshot comparison terhadap approved baseline belum ada.
- Audit icon-only tooltip/aria dan seluruh text-only help metadata belum dibuktikan 100% coverage.
## 5. Rekomendasi Prioritas

### P0 — gate sebelum merge
1. Pertahankan QA 9 viewport sebagai mandatory CI/manual acceptance.
2. Jangan mengembalikan stylesheet override kedua; `styles.css` harus tetap satu authoritative output.
3. Semua perubahan motion/safety tetap staged dan read-only/replay tidak boleh mengirim write/motion.
4. Setiap perubahan backend config harus mempertahankan transaction backup + rollback + readback.

### P1 — implementasi berikutnya
1. Tambahkan schema metadata authoritative per parameter di backend/config metadata: `label`, `type`, `unit`, `min`, `max`, `step`, `options`, `risk`, `requires_stationary`, `apply_mode`, `dependencies`.
2. Ubah renderer Help/Tuning agar tidak lagi menebak metadata dari nama/path.
3. Buat satu Batch Diff Review sebelum Apply: Baseline | YAML | Draft | expected Runtime, berikut warning/restart semantics.
4. Selesaikan guided calibration satu per satu dengan prerequisite, progress, validation, recovery, dan evidence.
5. Tambah runtime capability enum: `dynamic`, `lifecycle_reload`, `node_restart`, `startup_only`, `read_only`.

### P2 — maintainability/ergonomics
1. Pecah CSS source ke token/layout/components/pages lalu build menjadi satu stylesheet final.
2. Pecah `app.js` minimal ke navigation/help/config-store/tuning/widgets tanpa mengubah ID/backend contract.
3. Tambahkan approved screenshot baseline dan visual-diff threshold.
4. Tambahkan automated coverage audit untuk `aria-label`, icon-only tooltip, dan help metadata Basic/Advanced.
## 6. Bukti Verifikasi Sesi Ini

Runtime:
- `127.0.0.1:5000` listen oleh `agv_web_gui`.
- `/api/health` => `ok=true`, `ros=true`, `read_only=true`.
- `/api/config/schema` menampilkan capabilities validate/atomic batch/backup/rollback/readback/revert.
- `/api/config/validate` dengan batch kosong => HTTP 400 sesuai contract.
- `/api/config/apply` pada read-only => HTTP 403 sesuai safety contract.

Static/source:
- `node --check src/navigation/web/static/app.js` PASS.
- `git diff --check` untuk ROS Web/QA PASS.
- DOM audit: 782 ID, zero duplicate ID.
- `.legacy-domain-switch` tidak ada.
- `#testPhaseTabs/#testFamilyTabs` tidak ada.
- Help Drawer dan Tuning Modebar ada.
- `web_workspace_tabs_self_check.py` PASS.

Build:
- `colcon build --symlink-install --packages-select navigation` PASS: 1 package finished.

Browser QA:
- 1920x1080, 1600x900, 1366x768, 1280x720, 1024x768, 820x1180, 768x1024, 430x932, 390x844 semuanya PASS.
- Zero console/page errors, zero unexpected HTTP >=400, zero failed requests, zero automatic POST pada read-only UI.
- Zero hidden-pane focusable, zero document horizontal overflow, zero invisible blocker, zero interactive-control overlap setelah allow-list overlay yang sah.
- Screenshot artifacts: `/tmp/agv_ros_web_qa_20260910/`.
## 7. Perubahan Low-Risk yang Diterapkan Sesi Ini

1. `scripts/qa_ros_web_playwright.js`
   - viewport matrix diperluas ke seluruh 9 resolusi mandatory plan;
   - screenshot per domain;
   - interactive overlap detector;
   - sticky first-content coverage check;
   - allow-list backdrop/overlay yang memang sah.
2. `src/navigation/web/static/styles.css`
   - fix breakpoint `max-width:560px`: `.workspace-tabs` kembali `position:relative; top:auto!important` agar rule token sticky global tidak menggeser tab ke bawah dan menabrak complexity controls.
3. Dokumen audit ini dibuat di `doc/`.

## 8. Acceptance Verdict

**P0 runtime/layout transaction redesign: LULUS pada read-only QA.**

**Full plan: BELUM 100% COMPLETE**, sehingga belum layak diberi label final-complete. Sisa nyata adalah metadata schema authoritative, coverage guided calibration, modularisasi CSS/JS, full help/accessibility coverage, dan automated screenshot baseline comparison.

Tidak ada motion command yang dikirim dalam QA sesi ini. Pengujian apply transaction aktif sengaja tidak dilakukan pada kendaraan/config produksi karena runtime berjalan `read_only=true`; yang diverifikasi adalah contract validate dan penolakan write 403, sementara source transaction/rollback berhasil dibuild.
