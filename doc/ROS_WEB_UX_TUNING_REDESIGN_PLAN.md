# ROS Web UX, Overlay, Help, dan Tuning Redesign Plan

**Project:** AGV ROS 2 Humble  
**Target:** `/home/sirobo/agv/src/navigation/web`  
**Referensi lokal:** `/home/sirobo/agv/esc/vesc_tool` dan `/home/sirobo/agv/ardupilot/MissionPlanner`  
**Tanggal audit:** 2026-09-10  
**Status:** PLAN / belum mengubah implementasi ROS Web  

---

## 1. Tujuan

Redesign ROS Web harus menghasilkan engineering console yang tetap kuat untuk commissioning, tetapi tidak membuat operator bingung karena terlalu banyak panel, tab semu, parameter mentah, atau kontrol yang tampak ganda.

Target utamanya:

1. Tidak ada panel yang menimpa panel lain secara tidak sengaja.
2. Tidak ada fungsi yang tampil dua kali tanpa alasan yang jelas.
3. Secondary tab menjadi **true workspace tab**, bukan hanya scroll-to-anchor.
4. Tuning mengikuti alur kerja manusia: **pilih tujuan → ubah parameter → lihat efek → validasi → simpan**.
5. Operator baru dapat memahami fungsi parameter/box/tombol tanpa membaca source code.
6. Engineer tetap mendapat akses lengkap ke YAML, runtime state, raw telemetry, trial, graph, dan expert parameter.
7. Tampilan mengambil pola terbaik VESC Tool dan MissionPlanner tanpa menyalin kepadatan/legacy UX keduanya.
8. Semua perubahan visual dibuktikan dengan automated overlap test dan screenshot regression.

## 2. Scope Audit yang Sudah Dilakukan

Source yang diperiksa:

- `src/navigation/web/static/index.html`
- `src/navigation/web/static/app.js`
- `src/navigation/web/static/styles.css`
- `src/navigation/web/static/vesc_mp_theme.css`
- struktur VESC Tool, terutama `configparams.*`, `widgets/paramedit*`, `widgets/helpdialog.*`, `startupwizard.*`, dan halaman connection/motor.
- struktur MissionPlanner, terutama `ConfigRawParams`, `ParameterMetaDataRepository`, `MavlinkNumericUpDown`, `MavlinkComboBox`, calibration pages, dan `ParamCompare`.

Catatan audit runtime:

- Saat audit ini `localhost:5000` **tidak sedang listen**, sehingga tidak ada klaim bahwa screenshot runtime aktif telah diverifikasi.
- Audit visual saat ini berasal dari DOM/CSS/JS source yang benar-benar terpasang.
- Plan implementasi mewajibkan browser QA saat server aktif sebagai acceptance gate.

### 2.1 Angka penting dari source sekarang

- `index.html`: 764 ID diperiksa dan **tidak ada duplicate ID**.
- `page-esc`: sekitar 13 panel, 43 button, 16 input dalam satu page.
- `page-experiments`: sekitar 12 panel, 25 button, 11 input sebelum field tuning dinamis dirender.
- `page-perception`: 8 panel, belum termasuk detail dinamis.
- `styles.css` mengulang selector `.topbar` sampai 10 kali.
- Sekitar **250 selector yang sama** didefinisikan di `styles.css` dan di-override lagi oleh `vesc_mp_theme.css`.

## 3. Temuan Utama: Penyebab UI Terasa Double / Overlay / Useless

### 3.1 Tab sekunder masih tab semu — PRIORITAS P0

`WORKSPACE_TABS` mempunyai banyak action `anchor:<page>:<section>`. `activateWorkspaceTab()` hanya membuka page lalu menjalankan `scrollIntoView()`.

Akibatnya:

- klik `Perception > Detection` tidak membuat Detection menjadi workspace tunggal;
- Camera, Lane, Planning, Calibration, Performance, Detection, Safety, dan Evidence tetap berada dalam DOM/layout page yang sama;
- klik tab memberi persepsi seperti pindah halaman, tetapi sebenarnya hanya berpindah posisi scroll;
- semakin banyak feature ditambahkan, page semakin panjang dan terasa seperti konten ganda.

**Keputusan:** semua workspace tab harus memakai state `activePane`, dan hanya pane yang relevan yang visible. Scroll-to-anchor hanya boleh dipakai untuk link dokumentasi, bukan navigasi primer.

### 3.2 CSS bertumpuk antar-generasi — PRIORITAS P0

Saat ini `styles.css` berisi banyak revisi append-only, kemudian `vesc_mp_theme.css` melakukan override besar lagi. Contoh class yang berkali-kali didefinisikan: `.topbar`, `.sidebar`, `.content`, `.panel`, `.map-layout`, `.config-workbench`, `.tuning-field`, `.rviz-stage`, dan `.workspace-tabs`.

Risikonya:

- layout final ditentukan oleh urutan file, bukan satu design contract;
- breakpoint lama dan baru dapat saling mengalahkan;
- perubahan kecil berpotensi memunculkan overlap pada resolusi lain;
- developer sulit mengetahui declaration mana yang authoritative.

**Keputusan:** setelah UI target stabil, consolidate menjadi token + layout + component + page layer yang jelas. Tidak boleh ada “V2/V3/V5 final override” sebagai mekanisme desain permanen.

### 3.3 Overlay yang memang benar vs overlay yang harus dibuang

**Overlay yang sah dan harus dipertahankan:**

- `perceptionCalOverlay` di atas camera frame, karena fungsi utamanya memang drag calibration.
- RViz/map layer controls di atas map, bila compact/collapsible.
- toast, confirm modal, dan mobile sidebar backdrop.
- context visual Map/Camera/ESC yang memakai satu viewport, **asal hanya satu pane active**.

**Syarat overlay yang sah:**

- inactive layer wajib `display:none` atau `visibility:hidden + pointer-events:none`;
- inactive layer tidak boleh menerima focus/tab;
- z-index harus berasal dari skala token, bukan angka acak;
- canvas calibration hanya menerima pointer saat calibration mode aktif;
- placeholder tidak boleh berada di atas image setelah frame valid;
- overlay harus mengikuti bounding rect media yang sebenarnya, termasuk letterbox/object-fit.

**Overlay/stacking yang harus dihilangkan atau direstruktur:**

- seluruh section dalam domain yang tetap visible saat secondary tab lain dipilih;
- dua atau lebih control bars yang menawarkan action sama pada satu context;
- sticky workspace tabs yang dapat bertabrakan dengan topbar saat breakpoint mengubah tinggi topbar;
- layer lama yang hanya disembunyikan oleh CSS override generasi terbaru tetapi masih menjadi struktur utama.

### 3.4 Hidden legacy navigation tetap menjadi technical debt

`legacy-domain-switch` memang `hidden`, tetapi masih berada di DOM. Ini bukan visual bug saat ini, namun menambah dua sumber navigasi dan state yang harus dipelihara.

**Keputusan:** setelah domain sidebar baru proven, hapus markup/event path legacy, jangan hanya menyembunyikannya.

### 3.5 Tuning page terlalu banyak tanggung jawab

Saat ini satu page tuning/experiments memuat sekaligus:

- commissioning roadmap;
- phase tabs + family tabs;
- chapter browser/search;
- selected-test header;
- procedure guide;
- preflight safety;
- run identity/recorder;
- live graph;
- YAML parameter editor;
- context visual;
- perception-specific wizard;
- result table;
- trial recap + comparison;
- raw source/audit metadata.

Semua fitur berguna, tetapi **berguna tidak berarti harus terlihat bersamaan**. Ini sumber utama cognitive overload.

**Keputusan:** ubah menjadi staged workspace dengan tiga mode utama:

1. **TASK** — pilih pekerjaan dan ikuti procedure/preflight.
2. **TUNE** — hanya parameter penting + live effect + Apply/Revert.
3. **ANALYZE** — graph, table, trial comparison, export, raw evidence.

Mode Expert dapat membuka full YAML/raw metadata tanpa memenuhi layar operator biasa.

## 4. Target Information Architecture

### 4.1 Level 1 — Domain tetap sederhana

Top domain dipertahankan karena sudah mudah dipahami:

- **Overview**
- **ESC**
- **Navigation**
- **Perception**

Status global ROS2, motion authority, dan E-stop tetap selalu terlihat di topbar.

### 4.2 Level 2 — True workspace tabs

Setiap domain hanya menampilkan satu workspace utama pada satu waktu.

**Overview:**
- Monitor
- Health
- Authority
- Events
- Replay
- Reports

**ESC:**
- Live
- Control
- Tune
- Configuration
- Diagnostics

**Navigation:**
- Map / Mission
- Sensors
- Localization
- Planner
- Controller
- Safety
- Tune

**Perception:**
- Live
- Calibration
- Detection
- Lane
- Planning
- Safety
- Performance
- Evidence

### 4.3 Level 3 — Progressive disclosure, bukan tab tambahan tanpa akhir

Di dalam workspace gunakan hanya tiga pola:

- **primary surface**: informasi/action yang diperlukan sekarang;
- **details drawer**: telemetry/detail lanjutan;
- **Expert drawer**: raw JSON, YAML path, internal topic, source metadata.

Jangan menambah level tab baru kecuali kontennya benar-benar mutually exclusive.

### 4.4 Aturan satu-fungsi-satu-tempat

Action yang sama tidak boleh muncul di beberapa lokasi aktif sekaligus.

Contoh target:

- parameter umum diedit di TUNE; Full Configuration hanya expert browser;
- recorder START/STOP hanya berada pada TASK/TUNE header yang sticky;
- export trial berada di ANALYZE, bukan juga di setiap graph dan recap kecuali export per-chart;
- perception calibration hanya di `Perception > Calibration`, bukan juga sebagai section panjang di Live;
- safety bypass hanya di Safety/Commissioning context, bukan menjadi kontrol umum di setiap page.

## 5. Redesign Tuning: Basic / Advanced / Expert

Setiap domain tuning harus memiliki complexity mode yang konsisten.

### 5.1 BASIC — default

Ditujukan untuk commissioning normal. Hanya parameter yang benar-benar menjawab tujuan user ditampilkan.

Setiap screen maksimal menampilkan:

- 3–8 parameter utama;
- satu live visual/graph utama;
- satu status target;
- satu action bar: `Apply`, `Revert`, `Validate`;
- warning jika kendaraan harus stationary/restart.

Nama parameter harus human-readable, misalnya `Lane correction strength`, bukan menjadikan YAML path sebagai judul.

### 5.2 ADVANCED

Menambah parameter pendukung, min/max, unit, runtime status, dependencies, dan graph tambahan.

### 5.3 EXPERT

Menampilkan:

- exact YAML key/path;
- source file;
- baseline/current/draft/runtime value;
- raw ROS topic/state bila relevan;
- config fingerprint;
- full parameter browser;
- developer metadata.

Expert harus opt-in dan disimpan di local UI preference, bukan menjadi default setelah refresh pada workstation operator umum.

### 5.4 Tuning dikelompokkan berdasarkan tujuan, bukan nama file YAML

**ESC**
- Steering sensor / center / limits
- Current control / FOC
- Position steering loop
- Velocity loop
- Motor limits / thermal / current
- Validation / step response

**Navigation**
- Vehicle geometry & Ackermann
- Speed calibration
- Local EKF
- Global EKF / GNSS
- Costmap
- Smac Hybrid-A*
- MPPI
- Safety / command authority

**Perception**
- Camera / inference
- Metric calibration
- Object ROI → Nav2
- Lane safety lines
- Lane proportional correction
- Trajectory safety
- Performance / qualification

User memilih “apa yang mau diperbaiki”, lalu UI menentukan parameter yang relevan. Full YAML tetap tersedia untuk engineer.

## 6. Help / Hint System — Wajib Metadata-Driven

Pola utama diambil dari VESC Tool `ConfigParam.longName + description + HelpDialog`, lalu diperluas untuk kebutuhan AGV.

### 6.1 Aturan visual

- **Setiap parameter:** satu tombol kecil `?` di samping label.
- **Setiap box/panel:** satu tombol `?` di header jika fungsi panel belum self-evident.
- **Setiap text-only action button:** wajib mempunyai help metadata; bila action tidak obvious, tampilkan affordance `?`/info di action group.
- **Button yang sudah mempunyai icon yang jelas:** tidak perlu tombol `?` visual tambahan sesuai requirement; tetap wajib `aria-label`/native tooltip untuk accessibility.
- Hover hanya bonus. Klik/tap `?` harus bekerja di touchscreen.
- Jangan menaruh paragraf bantuan panjang secara permanen di setiap card.

### 6.2 Saat hint dibuka

Gunakan satu **Help Drawer** di sisi kanan, bukan modal baru untuk setiap parameter.

Isi minimal:

1. Nama sederhana.
2. “Untuk apa?”
3. Efek jika nilai dinaikkan.
4. Efek jika nilai diturunkan.
5. Unit.
6. Range teknis / recommended range jika valid.
7. Current value, baseline, dan pending value.
8. Apakah apply runtime atau perlu restart/lifecycle reload.
9. Dependency dengan parameter lain.
10. Risiko: `INFO / CAUTION / MOTION / SAFETY-CRITICAL`.

### 6.3 Metadata schema yang disarankan

Tambahkan satu registry UI, misalnya:

```yaml
lane_corridor_correction_gain_m_per_px:
  label: Lane correction strength
  group: Lane Safety
  level: basic
  summary: Besar koreksi steering saat lane melewati safety line.
  unit: m/px
  effect_up: Koreksi menjadi lebih agresif.
  effect_down: Koreksi menjadi lebih lembut.
  risk: motion
  requires_stationary: true
  runtime_apply: true
  related:
    - lane_corridor_touch_margin_px
    - lane_corridor_release_gap_px
```

Registry dapat berada di `src/navigation/web/config/ui_parameter_metadata.yaml` atau di source schema backend. Yang penting: **satu source of truth**, tidak hardcode penjelasan yang sama di banyak template JS.

### 6.4 Help key juga berlaku untuk panel/action

Contoh key:

- `panel.perception.lane_safety`
- `panel.navigation.mppi`
- `action.config.apply_all`
- `action.perception.safety_bypass`
- `action.esc.web_maintenance`

Dengan begitu bahasa, wording, severity, dan dokumentasi dapat diperbarui tanpa mengubah layout.

## 7. Pola dari VESC Tool yang Layak Diterapkan

Audit lokal menemukan pola konkret berikut.

### 7.1 Parameter metadata sebagai sumber UI

`esc/vesc_tool/configparams.cpp` menyediakan long name, description, min, max, step, dan tipe parameter. Widget editor mengambil metadata ini, bukan membuat label/range secara terpisah.

**Terapkan:** renderer parameter ROS Web harus membangun input, slider, enum, help, unit, dan validation dari metadata yang sama.

### 7.2 Help per parameter

`widgets/parameditdouble.cpp`, `parameditint.cpp`, `parameditenum.cpp`, dll memanggil `HelpDialog::showHelp()`, yang mengambil `longName` dan `description` dari parameter.

**Terapkan:** tombol `?` per parameter + satu reusable Help Drawer.

### 7.3 Wizard untuk pekerjaan multi-step

VESC Tool memakai `QWizard` dan setup wizard untuk pekerjaan yang tidak aman bila dilakukan acak.

**Terapkan:** calibration/commissioning AGV menjadi stepper dengan prerequisites, progress, capture state, validation, dan Finish/Apply.

### 7.4 Realtime terpisah dari configuration

VESC Tool tidak memaksa parameter editor, terminal, detection wizard, dan realtime scope terlihat sekaligus.

**Terapkan:** `Live`, `Tune`, `Calibration`, dan `Utilities` menjadi surface berbeda, bukan satu page panjang.

## 8. Pola dari MissionPlanner yang Layak Diterapkan

### 8.1 Friendly configuration vs Full Parameter List

MissionPlanner mempunyai halaman konfigurasi yang task-oriented dan tetap menyediakan `ConfigRawParams` / Full Parameter List.

**Terapkan:**

- default = friendly/task tuning;
- Expert = Full Parameters;
- jangan memaksa user belajar nama YAML untuk commissioning normal.

### 8.2 Metadata range / increment / options

`MavlinkNumericUpDown` mengambil range dan increment dari `ParameterMetaDataRepository`; combo mengambil option dari metadata.

**Terapkan:** input numeric harus memperoleh min/max/step dari schema. Enum harus dropdown. Boolean harus switch. Raw text hanya fallback expert.

### 8.3 Out-of-range warning

MissionPlanner memberi warning sebelum menerima nilai di luar range.

**Terapkan:** ROS Web membedakan:

- hard invalid → tidak dapat di-stage;
- outside recommended → boleh hanya setelah warning;
- safety critical → confirmation + stationary gate;
- impossible dependency → Apply disabled dengan alasan terlihat.

### 8.4 Param compare dan favorites

`ParamCompare` serta favorite/sort pada raw parameter sangat relevan dengan staged config yang sudah mulai ada di ROS Web.

**Terapkan:** tambah compare `Baseline | Runtime | YAML | Draft`, favorite, modified, staged, dan filter level Basic/Advanced/Expert.

### 8.5 Calibration as guided workflow

MissionPlanner calibration pages expose one task at a time and show readiness/progress instead of raw parameter soup.

**Terapkan untuk AGV:**

- IMU/magnetometer calibration;
- steering center/endpoints;
- wheel speed scale;
- camera metric/homography;
- lane safety lines;
- object ROI;
- obstacle distance qualification.

### 8.6 Yang tidak boleh ditiru mentah-mentah

MissionPlanner dan VESC Tool sama-sama dapat menjadi sangat padat untuk engineer lama. ROS Web tidak boleh menyalin:

- terlalu banyak nested tabs;
- singkatan tanpa explanation;
- dialog blocking untuk setiap aksi kecil;
- layout desktop legacy yang sulit di-touch;
- semua parameter ditampilkan hanya karena tersedia.

Prinsipnya adalah mengambil **metadata, wizard, compare, task separation, dan diagnostics**, bukan kepadatan visualnya.

## 9. Matrix Redesign per Domain

### 9.1 Overview

**Pertahankan:** readiness, hardware health, data health, command authority, event timeline.  
**Ubah:** masing-masing menjadi true tab/pane, bukan semua section panjang sekaligus.  
**Gabung:** `Operational Overview` + `Navigation State` menjadi satu compact status strip ketika layar kecil.  
**Expert only:** raw channel diagnostics dan JSON.

### 9.2 ESC

Current ESC sangat lengkap tetapi terlalu panjang.

Target:

- **Live:** dual motor primary telemetry + scope.
- **Control:** LEFT steering / RIGHT velocity command dengan safety state.
- **Tune:** FOC/PID/limits yang relevan dan parameter metadata.
- **Configuration:** MC/App config + read/write/store semantics.
- **Utilities:** connection, firmware, terminal; tampil hanya saat maintenance.
- **Diagnostics:** fault history, decoded variables, compare.

Connection/FW/Terminal jangan berada di antara live graph dan motor telemetry pada workflow normal.

### 9.3 Navigation

Map/Mission tetap primary. Inspector yang sekarang sudah menggunakan true active panes dapat menjadi pola untuk domain lain.

Target side inspector:
- Vehicle
- EKF Local
- EKF Global
- Planner
- MPPI
- Costmap
- Safety
- Command Chain

Raw JSON pindah ke Expert drawer per inspector.

### 9.4 Perception

Current page paling jelas menunjukkan masalah tab-semu karena hampir semua section tetap berada pada page yang sama.

Target:

- **Live:** camera + 4–6 KPI terpenting saja.
- **Calibration:** camera yang sama + interactive lane/ROI overlay + calibration controls.
- **Detection:** object list/bboxes + class/confidence/ROI acceptance.
- **Lane:** lane state, penetration kiri/kanan, proportional correction, mux result.
- **Planning:** object points → path relevant → planning relevant → costmap handoff.
- **Safety:** trajectory gate, emergency/near-field state, bypass commissioning.
- **Performance:** FPS, latency, dropped frame, inference/postprocess.
- **Evidence:** capture/export/qualification.

Camera tidak perlu diduplikasi secara fisik antar pane. Gunakan satu `CameraViewport` component yang mode overlay-nya berubah berdasarkan active workspace.

### 9.5 Perception Calibration khusus

Interactive calibration harus mempunyai mode eksplisit:

1. `VIEW` — overlay visible, pointer disabled.
2. `EDIT LANE` — hanya handle lane aktif.
3. `EDIT OBJECT ROI` — hanya handle ROI aktif.
4. `REVIEW` — tampilkan before/current dan status validation.
5. `APPLIED` — kembali pointer-disabled.

Dengan ini canvas overlay tidak akan menangkap click saat user hanya ingin melihat live camera.

## 10. Tuning State Model yang Harus Terlihat Jelas

Salah satu sumber kebingungan tuning adalah user tidak selalu tahu nilai mana yang sedang dilihat.

Setiap parameter editable harus memiliki empat state eksplisit:

- **BASELINE** — nilai referensi awal/known-good.
- **YAML** — nilai yang tersimpan sebagai source-of-truth.
- **RUNTIME** — nilai yang benar-benar sedang dipakai node saat ini bila dapat dibaca.
- **DRAFT** — nilai yang sedang disiapkan user tetapi belum diterapkan.

UI row ideal:

`Parameter | Runtime | Draft/Input | Unit | Status | ?`

Expanded detail:

`Baseline | YAML path | Apply mode | Restart requirement | Related parameters`

Status yang diperbolehkan:

- `SYNCED`
- `DRAFT`
- `YAML SAVED`
- `RUNTIME APPLIED`
- `RESTART REQUIRED`
- `VERIFYING`
- `ERROR`
- `LOCKED`

Jangan menggunakan label `READY` untuk beberapa arti yang berbeda.

## 11. Apply / Save / Revert Semantics

Gunakan bahasa action yang konsisten di seluruh domain.

- `Stage` = simpan ke Draft saja.
- `Apply` = terapkan Draft ke runtime bila parameter mendukung dynamic update.
- `Save YAML` = commit nilai ke source configuration file secara atomik.
- `Apply + Save` = operasi transaction yang jelas, bukan dua action tersembunyi.
- `Revert Draft` = buang perubahan yang belum diterapkan.
- `Restore Baseline` = membuat Draft dari baseline; belum langsung menulis.
- `Reload Runtime` = lifecycle/restart hanya jika benar-benar dibutuhkan.

Untuk banyak parameter, default harus **batch/staged apply**, seperti workflow parameter engineering MissionPlanner, bukan menulis otomatis setiap kali input berubah.

Auto-write hanya boleh tersedia sebagai Expert preference dan OFF secara default untuk parameter motion/safety.

### 11.1 Transaction safety

Satu batch Apply harus:

1. validate type/range/dependency;
2. cek stationary/motion gate;
3. backup YAML;
4. tulis semua field secara atomik;
5. apply runtime yang didukung;
6. verify runtime readback;
7. rollback bila transaction gagal secara parsial;
8. tampilkan diff dan hasil per parameter.

## 12. CSS Architecture Baru

Target akhir jangan lagi mengandalkan dua stylesheet besar yang saling mengoreksi.

Struktur yang disarankan:

```text
static/css/
  tokens.css
  base.css
  layout.css
  components.css
  pages/
    overview.css
    navigation.css
    perception.css
    esc.css
    tuning.css
    configuration.css
```

Atau tetap build menjadi satu file final, tetapi source harus modular.

### 12.1 Layer contract

1. `tokens`: color, spacing, typography, radius, z-index, breakpoint.
2. `base`: reset, body, form primitive.
3. `layout`: shell, sidebar, topbar, workspace.
4. `component`: panel, tabs, button, field, help, table, graph.
5. `page`: hanya aturan yang benar-benar domain-specific.

Tidak boleh ada page stylesheet yang redefinisi global `.panel`, `.topbar`, `.content`, atau `.button`.

### 12.2 Z-index contract

Definisikan token tetap, misalnya:

- content: `0`
- in-canvas overlay: `10`
- sticky workspace: `20`
- topbar/sidebar: `30`
- dropdown/help popover: `40`
- toast: `50`
- modal/backdrop: `60`

Tidak boleh menambah z-index baru tanpa kategori.

### 12.3 Sticky offset contract

`workspace-tabs` sekarang memiliki fixed `top` yang dapat tidak cocok ketika topbar berubah tinggi pada breakpoint.

Target:

- expose `--topbar-h` dan `--workspace-h`;
- topbar menentukan actual token per breakpoint;
- workspace tabs menggunakan `top: var(--topbar-h)`;
- anchor/scroll margin menggunakan jumlah token yang sama;
- automated test memastikan first content row tidak tertutup sticky bars.

### 12.4 Touch target

Action penting minimum 40–44 px pada touch layout. Desktop engineering density boleh lebih rapat, tetapi click target tidak boleh hanya mengikuti glyph kecil.

## 13. JavaScript Architecture / Component Contract

`app.js` sekarang memegang terlalu banyak domain sekaligus. Refactor bertahap tanpa framework wajib.

Target module:

```text
static/js/
  state.js
  navigation.js
  help.js
  config-store.js
  tuning.js
  widgets/param-field.js
  widgets/help-drawer.js
  widgets/true-tabs.js
  pages/perception.js
  pages/navigation.js
  pages/esc.js
```

### 13.1 TrueTabs component

API minimum:

- `registerWorkspace(domain, panes)`
- `activatePane(domain, paneId)`
- set `hidden`, `aria-hidden`, dan tabIndex dengan benar;
- restore last pane per domain;
- deep-link optional via URL hash;
- tidak memakai `scrollIntoView` sebagai mekanisme navigasi primer.

### 13.2 Shared Help component

Semua `data-help-key` menggunakan satu event delegation. Jangan membuat ratusan handler per button.

## 14. Backend / API yang Perlu Dirapikan untuk UX Tuning

Pertahankan atomic YAML write yang sudah ada, tetapi expose semantics yang lebih jelas.

Endpoint target konseptual:

- `GET /api/config/schema` — metadata field, range, unit, options, help, apply mode.
- `GET /api/config/state` — baseline/YAML/runtime consistency.
- `POST /api/config/validate` — validasi draft batch tanpa menulis.
- `POST /api/config/apply` — transaction batch.
- `POST /api/config/revert` — rollback ke snapshot/baseline yang dipilih.

Tidak wajib mengganti endpoint lama sekaligus. Tambahkan adapter lebih dulu agar UI baru bisa migrasi bertahap.

### 14.1 Runtime capability metadata

Setiap parameter harus mengatakan salah satu:

- `dynamic`
- `lifecycle_reload`
- `node_restart`
- `startup_only`
- `read_only`

UI tidak boleh menebak apply mode dari nama file.

### 14.2 Dependency metadata

Contoh dependency yang harus dapat diekspresikan:

- field hanya relevan bila feature enabled;
- field B harus lebih besar daripada field A;
- lane control tidak boleh enable sebelum calibration validated;
- motion test tidak dapat start bila E-stop/telemetry/preflight belum memenuhi gate.

## 15. Automated Overlay / Duplicate UX QA

Tidak cukup mengandalkan inspeksi mata. Tambahkan browser QA khusus layout.

### 15.1 Resolusi wajib

- 1920×1080
- 1600×900
- 1366×768
- 1280×720
- 1024×768
- 820×1180
- 768×1024
- 430×932
- 390×844

### 15.2 Overlap detector

Playwright membaca `getBoundingClientRect()` semua elemen interactive/major panels lalu melaporkan intersection yang tidak berada dalam allow-list.

Allow-list hanya untuk:

- modal/backdrop;
- toast;
- map layer panel di map viewport;
- perception calibration canvas di camera viewport;
- context visual pane yang active.

Test harus gagal bila:

- button/input saling bertumpuk;
- sticky header menutup heading/form;
- panel keluar viewport horizontal;
- hidden pane masih menerima pointer/focus;
- dua context pane visible sekaligus;
- placeholder menutup media aktif.

### 15.3 Screenshot regression

Ambil screenshot per domain + per mode Basic/Advanced/Expert dan simpan artifact QA, bukan commit screenshot setiap run ke source tree.

## 16. Implementation Phases

### Phase 0 — Baseline & inventory (P0)

- Jalankan ROS Web runtime read-only.
- Capture screenshot semua domain/resolusi.
- Simpan DOM panel inventory.
- Tambah initial overlap detector.
- Catat action/parameter yang muncul di lebih dari satu active surface.

**Exit:** baseline reproducible tersedia sebelum redesign.

### Phase 1 — True workspace navigation (P0)

- Ganti semua primary `anchor:*` tabs menjadi pane state.
- Pertahankan URL/deep-link compatibility bila dibutuhkan.
- `Perception`, `ESC`, `Overview` menjadi target pertama.
- Pastikan inactive pane `hidden` dan tidak focusable.

**Exit:** satu workspace tab = satu content workspace.

### Phase 2 — CSS consolidation (P0)

- Buat token authoritative.
- Freeze penambahan override baru.
- Pindahkan style final dari dua stylesheet ke layer modular.
- Hapus selector obsolete setelah screenshot parity.
- Definisikan z-index dan sticky offsets.

**Exit:** satu selector global mempunyai satu source styling utama.

### Phase 3 — Help metadata + Help Drawer (P1)

- Tambah schema parameter/panel/action.
- Buat reusable `HelpButton` dan `HelpDrawer`.
- Migrate parameter Basic lebih dulu.
- Tambah `data-help-key` untuk box dan text-only action.
- Tambah aria-label/title untuk icon-only controls.

**Exit:** tidak ada parameter Basic tanpa explanation yang dapat dibuka dengan satu click/tap.

### Phase 4 — Tuning TASK / TUNE / ANALYZE (P1)

- Pisahkan procedure/preflight dari parameter editor.
- Pisahkan graph/trial/export ke Analyze.
- Tambah Basic/Advanced/Expert switch.
- Default auto-write YAML menjadi staged transaction untuk motion/safety parameter.
- Buat sticky pending-change bar yang compact.

**Exit:** user dapat menyelesaikan satu tuning task tanpa scroll melewati panel yang tidak diperlukan.

### Phase 5 — Guided calibration (P1)

- Steering endpoints/center wizard.
- Speed scale wizard.
- Camera metric/homography wizard.
- Lane + Object ROI direct calibration.
- Obstacle distance qualification.
- IMU/magnetometer wizard.

**Exit:** tiap calibration mempunyai prerequisite, progress, validation, save/apply, dan recovery yang jelas.

### Phase 6 — Config transaction + compare (P1)

- Tambah batch validate/apply/readback.
- Tampilkan Baseline/YAML/Runtime/Draft.
- Tambah compare dan diff sebelum apply.
- Tambah rollback hasil transaction.
- Pertahankan favorite/modified/staged filter.

**Exit:** tidak ada kondisi user mengira nilai sudah runtime padahal baru tersimpan di YAML.

### Phase 7 — Density, responsive, accessibility (P2)

- Touch target audit.
- Keyboard tab/focus audit.
- Contrast/status semantics.
- Mobile/tablet layout.
- Kurangi raw JSON dari default surface.
- Hilangkan hidden legacy DOM yang tidak lagi dipakai.

### Phase 8 — Regression & cleanup (P0 sebelum merge final)

- Browser QA seluruh resolusi.
- Screenshot comparison.
- Zero unexpected overlap.
- Zero horizontal document overflow.
- Zero duplicate active action.
- Test read-only mode dan replay mode.
- Test active runtime mode tanpa mengirim motion command otomatis.

## 17. Backlog Konkret: Keep / Merge / Move / Remove

| Area sekarang | Keputusan | Target |
|---|---|---|
| Workspace tabs berbasis `anchor:` | **REMOVE sebagai navigation behavior** | True pane activation; anchor hanya untuk deep-link/docs |
| `legacy-domain-switch` hidden | **REMOVE setelah migrasi** | Satu domain/navigation source saja |
| Perception Camera + seluruh section di bawahnya | **SPLIT** | Live / Calibration / Detection / Lane / Planning / Safety / Performance / Evidence |
| Perception Lane + Planning cards di kanan camera | **KEEP tetapi contextual** | Live summary compact; detail penuh di masing-masing pane |
| Perception calibration canvas | **KEEP** | Hanya pointer-active pada EDIT mode |
| Perception Safety Bypass di Live camera | **MOVE** | Safety / Commissioning pane |
| ESC Connection/FW/Terminal di workbench utama | **MOVE** | Utilities / Maintenance pane |
| ESC decoded telemetry detail | **MOVE/COLLAPSE** | Expert drawer / Diagnostics |
| ESC graphs + telemetry | **KEEP** | Live pane, satu primary scope per motor |
| Tuning roadmap | **KEEP, COMPACT** | Task header / progress strip |
| Phase tabs + family tabs + chapter browser | **MERGE** | Satu task navigator + search/filter |
| Trial Guide + Preflight | **MERGE** | TASK pane: procedure + readiness |
| Run Control | **KEEP** | Sticky task/run toolbar saat recording |
| Parameter editor | **KEEP/REDESIGN** | TUNE pane, Basic/Advanced/Expert |
| Context visual | **KEEP** | TUNE pane, satu active visual |
| Graph + report table + trial recap | **MOVE** | ANALYZE pane |
| Raw catalog/source metadata | **MOVE** | Expert drawer |
| Full YAML Configuration | **KEEP** | Expert engineering browser, bukan tuning default |
| Inline help paragraphs panjang | **REDUCE** | Ringkas 1-line hint + Help Drawer |

## 18. Prioritas Help Metadata

Urutan rollout agar nilai praktis muncul cepat:

**P0 — motion/safety:** E-stop-related state, ESC maintenance/control, Ackermann, MPPI command limits, lane correction, safety bypass, object ROI, trajectory safety.

**P1 — calibration:** steering endpoints, speed scale, EKF noise/rejection, GNSS gates, camera metric/homography, lane thresholds, obstacle qualification.

**P2 — diagnostics/report:** graph options, export controls, raw telemetry fields, evidence metadata.

Setiap metadata baru harus mempunyai review sederhana:

- label dapat dipahami tanpa nama source variable;
- unit benar;
- effect-up/effect-down tidak misleading;
- range berasal dari constraint nyata, bukan angka UI arbitrer;
- dependency dan restart semantics benar;
- risk classification sesuai dampak kendaraan.

## 19. Definition of Done UX

Redesign dianggap selesai hanya jika:

- satu secondary tab hanya menampilkan workspace yang dipilih;
- tidak ada unexpected overlap pada seluruh viewport QA;
- tidak ada document-level horizontal overflow;
- tidak ada action motion/safety yang muncul dua kali pada active context;
- semua parameter Basic/Advanced mempunyai help yang dapat dibuka dengan tap;
- text-only action yang tidak self-explanatory mempunyai help metadata;
- icon-only action mempunyai `aria-label`/tooltip yang jelas;
- user selalu dapat membedakan Baseline, YAML, Runtime, dan Draft;
- perubahan safety/motion tidak auto-write hanya karena input berubah;
- Apply menampilkan validation/readback result;
- Expert/raw UI tidak mengganggu default workflow;
- replay/read-only tetap memblokir write dan motion command;
- browser QA berjalan tanpa console error dan screenshot regression yang tidak dijelaskan.

## 20. Urutan File Implementasi yang Disarankan

Urutan ini meminimalkan risiko merusak runtime yang sedang dikembangkan paralel.

1. `scripts/qa_ros_web_playwright.js` / layout QA baru — buat baseline dan detector dulu.
2. `static/app.js` — tambah true workspace pane state tanpa mengubah backend.
3. `static/index.html` — beri wrapper/pane semantics dan pindahkan section, tanpa menghapus ID telemetry yang masih dipakai renderer.
4. CSS — konsolidasi setelah struktur pane stabil; jangan mengejar visual dengan override baru.
5. Help metadata registry + renderer Help Drawer.
6. Tuning renderer — Basic/Advanced/Expert + TASK/TUNE/ANALYZE.
7. Configuration transaction/readback UI.
8. Backend schema/validate/apply adapter bila metadata/runtime semantics belum tersedia.
9. Hapus legacy DOM/CSS/handler setelah coverage QA membuktikan tidak dipakai.

Saat refactor, ID telemetry dan endpoint yang sudah digunakan backend harus dipertahankan sampai migration layer selesai. Tujuannya mengubah UX tanpa memutus data contract ROS yang sudah berjalan.

## 21. Non-Goals dan Safety Guardrail

Redesign ini **tidak** bertujuan:

- menyembunyikan capability engineering; capability dipindah ke tempat yang tepat;
- mengubah algoritma Nav2/perception/ESC hanya demi tampilan;
- membuat UI otomatis memilih gain “terbaik” tanpa objective metric yang eksplisit;
- mengaktifkan calibration/safety flag hanya karena wizard selesai secara visual;
- mengirim motion command saat page dibuka, tab diganti, config dibaca, atau QA dijalankan.

Semua wizard yang dapat memengaruhi gerak harus tetap memakai existing command authority, stationary gate, E-stop, watchdog, finite-command check, dan readback/validation yang sesuai.

## 22. Contoh User Journey Target

### 22.1 Tune Lane Safety

1. Buka `Perception > Calibration`.
2. Camera viewport tampil satu kali.
3. Klik `Edit Lane` lalu drag garis kiri/kanan.
4. Help `?` menjelaskan touch margin, release gap, dan Kp.
5. Buka `Tune` untuk mengubah strength/threshold; live pane menunjukkan penetration dan correction.
6. Draft berubah tetapi kendaraan belum menerima perubahan otomatis.
7. Klik `Validate` → dependency/range/stationary gate diperiksa.
8. Klik `Apply + Save` → YAML transaction + runtime apply + readback.
9. Status menjadi `RUNTIME APPLIED` atau `RESTART REQUIRED` secara eksplisit.
10. Buka `Analyze` bila ingin merekam trial/graph/compare.

### 22.2 Tune MPPI

1. Buka `Navigation > Tune` dan pilih goal `Controller / MPPI`.
2. Basic hanya menampilkan parameter yang paling relevan untuk behavior target.
3. Hover/tap `?` memberi efek naik/turun dan dependency.
4. Context visual tetap map/path, bukan raw JSON.
5. Advanced membuka weights/constraints tambahan.
6. Analyze memperlihatkan tracking error, velocity, steering, clearance, dan compare trial.

### 22.3 ESC Commissioning

1. `ESC > Live` memastikan telemetry sehat.
2. `ESC > Control` hanya menampilkan command yang relevan dan authority state.
3. `ESC > Tune` memakai group Steering/FOC/Velocity/Limit.
4. `ESC > Utilities` baru dibuka saat perlu maintenance/terminal/firmware.
5. Expert dapat membuka decoded variable dan full config tanpa memenuhi layar operator.

## 23. Kesimpulan Audit

Masalah utama ROS Web saat ini bukan duplicate HTML ID. Masalah utamanya adalah **information architecture dan accumulated styling**: secondary tab masih scroll-to-anchor, banyak feature berguna dirender bersamaan, dan dua stylesheet besar mempunyai ratusan selector yang saling override.

Arah redesign yang direkomendasikan adalah:

**True Workspaces + Task-oriented Tuning + Metadata-driven Help + Staged Parameter Transactions + Expert Progressive Disclosure + Automated Layout QA.**

Dengan pola ini ROS Web tetap sekuat VESC Tool/MissionPlanner untuk engineer, tetapi commissioning harian lebih tenang, jelas, dan tidak membuat user harus memahami seluruh internal ROS/YAML sekaligus.
