# F4 HMI 2-Stage Source-Aligned Implementation Plan

Tanggal: 2026-09-10
Target repo: `/home/sirobo/agv/F4gateway`
Target HW: STM32F411CE + ILI9341 320x240 + XPT2046
Branch teramati: `v1`

## 1. Tujuan

Mengubah HMI F4 menjadi arsitektur operator yang sederhana:

```text
SPLASH -> HOME -> MAIN MENU -> ESC / PERSEPSI / NAVIGASI
                         \-> SERVICE MODE (hidden)
```

Implementasi dibagi hanya menjadi dua tahap besar agar perubahan UI tidak bercampur sekaligus dengan perluasan telemetry/protokol.

Tahap 1: finalisasi shell, state machine, navigation, safety behavior, dan layout dengan data yang sudah tersedia.
Tahap 2: isi seluruh 33 submenu domain, telemetry lanjutan, Service Mode lengkap, compact assets, integration dan cleanup.

## 2. Baseline source aktual yang wajib dipertahankan

Source saat audit sudah memiliki fondasi realtime yang bagus dan tidak boleh dirombak sembarangan:
- `src/main.cpp`: main loop, parser, UI state mutation, action gate, watchdog, display recovery.
- `src/UiShell.h`: renderer tunggal, dirty rendering, metric cache, domain renderer.
- `src/UiMenu.h`: tree menu, title, parent, children, edit mapping.
- `src/TouchButtons.h`: release-commit, slide-cancel, hold-to-run, touch generation.
- `include/HmiDisplay.h` + `src/HmiDisplay.cpp`: SPI ownership, batched frame transaction, recovery, touch read.
- `src/VescGateway.cpp`: transparent framed VESC gateway + safety-stop preemption + runtime/maintenance ownership.
- `src/Diagnostics.h`: UI timing, SPI/UART counters, service p95/p99/max, stack headroom.

Baseline menu saat ini masih:
`SPLASH -> OVERVIEW -> ESC / PERCEPTION / NAVIGATION / SYSTEM`.
Belum ada HOME dan MAIN MENU terpisah.

`UiState` saat ini hanya menyimpan `menu`, `selectedChild`, `editing`, `editValue`, transaksi config.
Belum ada `domainPageIndex`, `detailViewIndex`, atau state Service Mode tersembunyi.

Carousel saat ini bergerak satu child melalui `selectedChild` + `menuWindowFirst()`.
Target baru harus bergerak per grup tiga kartu: page 1 = item 0..2, page 2 = item 3..5, dst.

Telemetry saat ini masih flat di `VehicleTelemetry` dan sudah membawa data dasar ESC, GNSS, perception, Nav2 dan waypoint.
Namun belum membawa Vbus, current, duty, FOC Iq/Id, thermal, fault code, encoder raw counters, odom pose, costmap/controller detail, dan perception geometry detail.

`VescGateway` saat ini tidak melakukan semantic decode `COMM_GET_VALUES`; frame valid diteruskan mentah ke mini PC sebagai `VESC:RX:<hex>`.
Jadi tahap awal HMI tidak boleh mengasumsikan power telemetry lokal sudah tersedia.

Existing ELF: text 137108 B, data 8336 B, bss 49264 B.
App flash usable 360448 B sehingga masih ada margin, tetapi bukan alasan untuk menyimpan puluhan tile RGB565 penuh.
`src/VisualAssets.h` saat ini sudah menyimpan 3 asset RGB565 besar dengan total sekitar 38,952 B.
Menambah 40+ icon dengan cara yang sama akan menghabiskan flash secara tidak perlu.

Working tree saat audit memiliki perubahan yang belum commit pada area USB/udev:
`99-blackpill-stm32.rules`, `include/UsbCdcPort.h`, `scripts/install_stm32_udev_rules.sh`, `src/main.cpp`, `src/usb/UsbCdcPort.cpp`, `src/usb/usbd_cdc_if.cpp` serta beberapa test USB baru.
Implementasi HMI harus menghindari menimpa bagian perubahan USB tersebut dan melakukan edit `main.cpp` secara surgical.

Tiga self-check HMI/robustness saat audit semuanya PASS:
- `test/hmi_current_geometry_self_check.py`
- `test/hmi_10repo_audit_self_check.py`
- `test/spi_interrupt_display_robustness_self_check.py`

## 3. Invariant yang tidak boleh rusak

1. `Board_SetRealtimeServiceCallback()` tetap melayani safety, USB parser dan VESC selama burst TFT.
2. Renderer tetap main-loop only; tidak boleh render dari ISR/realtime callback.
3. `drawUiNow()` tetap memakai immutable snapshot `UiState` + `VehicleTelemetry`.
4. `tft.beginFrame()/endFrame()` dan SPI owner arbitration tetap dipakai.
5. Dirty rendering dan value cache tetap aktif; jangan kembali ke full-screen clear tiap telemetry update.
6. XPT2046 tetap release-commit + slide-cancel.
7. Motion test tetap hold-to-run; release/slide-out/display recovery/menu exit harus STOP.
8. Physical safety input tetap preempt queued VESC TX melalui `VescGateway::setSafetyStop()`.
9. Maintenance mode VESC tetap menunda best-effort TFT/GNSS tanpa menunda safety/USB/VESC.
10. Existing recovery/DFU/USB behavior tidak boleh ikut di-refactor hanya karena redesain HMI.
## 4. Target state machine final

State operator dibuat eksplisit dan tidak lagi bergantung pada OVERVIEW sebagai menu campuran:

```text
SPLASH
  -> HOME
      -> MAIN_MENU
          -> ESC_ROOT
          -> PERCEPTION_ROOT
          -> NAVIGATION_ROOT
      -> SYSTEM_ROOT hanya melalui hidden service entry
```

`SYSTEM_ROOT` tetap dipakai sebagai ID internal agar diagnostics lama tidak perlu dipindah sekaligus.
Ia hanya dikeluarkan dari `MAIN_MENU`, bukan dihapus.

`UiState` target minimal:
- `UiMenuId menu`
- `uint8_t pageIndex`
- `uint8_t detailViewIndex`
- `bool editing`
- `float editValue`
- config transaction fields yang sekarang sudah ada
- optional `bool serviceSession` untuk membedakan akses hidden service vs external diagnostic command.

`selectedChild` sebaiknya dihentikan setelah migrasi karena konsep final adalah page-of-three, bukan sliding selected child.
Mission waypoint selection tetap menggunakan `VehicleTelemetry::selectedWaypoint`, bukan state carousel.

## 5. TAHAP 1 — Shell final + navigation + safety-preserving migration

### 5.1 Tujuan Tahap 1

Tahap 1 menghasilkan HMI yang struktur navigasinya sudah final meskipun sebagian metric lanjutan masih `N/A`.
Tidak mengubah protokol VESC raw gateway dan tidak membutuhkan perubahan F103.
### 5.2 `src/Config.h`

Perubahan:
- tambah `HOME` dan `MAIN_MENU` tanpa merusak range ESC/PER/NAV yang dipakai helper lama;
- tambah `SoftKey::MENU` dan bila perlu `SoftKey::SERVICE_HOLD`;
- tambah `SERVICE_HOLD_MS = 3000`;
- buat layout HOME terpisah dari layout domain;
- buat helper konstanta `DOMAIN_PAGE_SIZE = 3`.

Jangan ubah pin, baud, timer, watchdog, display SPI atau touch calibration pada tahap ini.

Geometry HOME direkomendasikan:
- top status: 0..30;
- tiga summary tile: sekitar y=45..135;
- rail ESC/PER/NAV kecil: sekitar y=145..165;
- tombol MENU tunggal: y=184..235.

Geometry MAIN MENU/domain tetap memanfaatkan tiga kolom 100 px yang sudah terbukti in-bounds.

### 5.3 `src/UiMenu.h`

Ganti helper carousel sliding dengan page helper:
- `menuPageCount(count) = (count + 2) / 3`;
- `menuPageFirst(page) = page * 3`;
- `menuPageNext/pagePrev` wrap per halaman;
- `menuCardAt(root,page,slot)` mengembalikan child atau invalid.

`OVERVIEW` dipertahankan sementara hanya sebagai compatibility alias menuju HOME.
Tambahkan descriptor final 12 ESC, 9 PERCEPTION, 12 NAVIGATION sebagai child root, tetapi detail view tidak lagi dibuat sebagai nested menu terpisah.
Ini sekaligus menghindari tree terlalu dalam seperti `STEERING -> LIVE/TEST/CAL` yang ada sekarang.

Sebelum enum baru diperbanyak, ubah `isEscMenu()/isPerceptionMenu()/isNavigationMenu()` agar tidak bergantung pada range numerik enum.
Gunakan `menuDomain(id)` atau descriptor domain eksplisit supaya penambahan ID tidak menghasilkan salah klasifikasi.

Final root ordering:
- ESC: Overview, Operator Mode, Steering / Drive, FOC-Power, Motor Telemetry / Encoder, Calibration, Communication / Fault-Safety, Performance, Manual Test.
- PER: Per Overview, Camera, Detection / Lane, Drivable Area, Obstacle / Performance, Calibration, Per Test.
- NAV: Nav Overview, Localization, GNSS / IMU-MAG, EKF, Odometry / Mission, Nav2, Safety / Costmap, Path-Control, Nav Test.

### 5.4 Penyesuaian layout dari plan konseptual

Jangan memindahkan LEFT/RIGHT menjadi tombol sentuh kecil di top bar 30 px.
Source saat ini sudah memiliki footer 48 px dengan hit target kiri/kanan 76 px yang lolos geometry test dan lebih aman untuk XPT2046.

Layout final yang direkomendasikan:
- top bar: BACK/HOME kiri, title domain/detail di tengah, page/view indicator kecil;
- area tengah: tiga card atau detail metrics;
- footer: LEFT dan RIGHT besar, page/view number di tengah;
- editor/action: footer berubah menjadi LEFT/OK/RIGHT seperti implementasi sekarang.

Jadi konsistensi BACK/LEFT/RIGHT tetap tercapai secara fungsi, tetapi touch target tidak dikorbankan demi penempatan visual di header.
### 5.5 HOME baru di `src/UiShell.h`

Buat `drawHome()` terpisah dari `drawOverview()` lama.
HOME hanya menampilkan tiga informasi primer:
1. SPEED: `speedKmh` besar, fallback `driveActualMps` bila diperlukan.
2. STATE: E-STOP override > FAULT > system/vehicle state.
3. POWER: Vbus bila valid; jika field belum dikirim bridge tampilkan `N/A`, jangan hardcode 51.2 V.

Tambahkan indikator kecil:
- AUTO/MANUAL;
- ESC / PER / NAV health + freshness;
- ROS/F103 link secukupnya;
- E-STOP banner tetap prioritas tertinggi.

HOME tidak memakai carousel dan tidak mempunyai SYSTEM card.
Tombol bawah tunggal `MENU` membuka `MAIN_MENU`.
Long-hold MENU 3 s dapat menjadi entry Service Mode, tetapi hanya setelah stationary/service gate lolos.

Untuk Stage 1 boleh ditambah `vbusV` + `vbusValid` pada telemetry sebagai field forward-compatible.
Selama bridge belum mengirimkannya, kartu POWER wajib menampilkan `NO DATA/N/A` dan tidak ikut membuat domain terlihat READY.

### 5.6 MAIN MENU baru

Buat `drawMainMenu()` dengan tepat tiga kartu:
- ESC
- PERSEPSI
- NAVIGASI

Top-left menjadi HOME. Tidak ada footer carousel karena ketiga domain selalu terlihat sekaligus.
SYSTEM tetap reachable hanya melalui Service Mode/internal command, bukan kartu operator.
### 5.7 Domain root renderer

Buat satu renderer generic `drawDomainMenu()` berbasis descriptor, bukan tiga switch layout terpisah.
Input minimal: domain id, page index, descriptor cards, count, telemetry snapshot.

Per page selalu tiga slot tetap.
Jika page terakhir kurang dari tiga item, slot kosong harus dibersihkan dan tidak boleh mempunyai hit target aktif.
LEFT/RIGHT mengubah `pageIndex`, bukan `selectedChild`.
Page transition menaikkan touch generation sebelum redraw agar release dari layar lama tidak dieksekusi pada layar baru.

Status card menggunakan kombinasi:
- READY/OK;
- WARN/WAIT;
- STALE;
- FAULT;
- N/A bila metric memang belum di-bridge.

Jangan menyimpulkan READY hanya dari satu field baru.
Domain status tetap mengacu ke freshness domain + readiness gate yang authoritative.

### 5.8 Detail view state

Setiap card descriptor mempunyai `viewCount` 1..3.
Read-only detail memakai `detailViewIndex`; footer LEFT/RIGHT cycle view dalam card yang sama.
Header menampilkan contoh `STEERING 1/3`, bukan membuat child menu LIVE/CAL terpisah.

Saat BACK dari detail:
- kembali ke domain page yang melahirkan card tersebut;
- page index dipertahankan;
- detailViewIndex di-reset 0;
- semua armed touch/action dibatalkan.
### 5.9 `src/TouchButtons.h`

Pertahankan mekanisme existing yang sudah benar:
- tap commit saat RELEASE;
- slide keluar membatalkan action;
- motion baru aktif setelah HOLD 650 ms;
- motion RELEASE menghasilkan STOP;
- touch generation mencegah stale release mengeksekusi layar baru.

Tambahkan hit-test mode baru:
- HOME: hanya MENU + hidden long-hold service pada area MENU;
- MAIN_MENU: HOME + CARD_0..2;
- DOMAIN_MENU: BACK + CARD_0..2 + footer LEFT/RIGHT;
- DETAIL: BACK + footer LEFT/RIGHT;
- EDIT/ACTION: BACK + existing LEFT/OK/RIGHT;
- MANUAL_TEST: dedicated 5 controls existing.

Service hold tidak boleh memakai repeat event biasa.
Buat event hold khusus dengan threshold 3000 ms dan sekali-trigger per touch generation.

### 5.10 Safety gap yang wajib ditutup di `src/main.cpp`

Saat ini ROS offline sudah `stopAllManualTest()`, display recovery/menu exit juga stop, tetapi perubahan `escFresh=false` belum secara eksplisit menghentikan seluruh manual motion.
Tambahkan state `steeringTestRunning` agar steering hold dapat diawasi sama seperti `driveTestRunning`.

Buat satu helper `manualMotionGateValid()` berdasarkan kondisi authoritative saat ini:
ROS connected + ESC fresh + MANUAL + ESC ready + !E-stop + valid encoder untuk steering.
Setelah `updateDomainFreshness()` dan setelah command yang mengubah MODE/STATE/ESC/ENC/ESTOP, evaluasi gate.
Jika motion test aktif dan gate menjadi invalid, kirim STOP segera dan clear local running state.

Perubahan spesifik:
- `ESTOP:1` -> `stopAllManualTest()`, bukan hanya `stopDriveTest()`;
- `escFresh` transisi true->false -> `stopAllManualTest()`;
- `MODE` berubah dari MANUAL -> AUTO saat test aktif -> STOP;
- `ESC`/`ENC` readiness hilang saat relevant test aktif -> STOP;
- ROS timeout/offline -> existing STOP tetap dipertahankan;
- display fault/recovery -> existing STOP tetap dipertahankan;
- page exit/back/home -> STOP sebelum state UI berubah.

`DRIVE_TEST_MAX_MS=3000` tetap sebagai lapisan timeout tambahan, bukan primary dead-man.
STOP harus idempotent dan boleh dikirim walau local state sudah idle.

### 5.11 Service Mode minimum pada Stage 1

Keluarkan SYSTEM dari `menuChildren(MAIN_MENU)`.
Pertahankan 5 SYSTEM page yang sudah ada sebagai service subset sementara:
System Health, IO Pin Monitor, Display Pins, Link Diagnostics, Error Counters.

Touch entry hanya jika kendaraan stationary dan tidak ada navigation/manual test aktif.
Untuk entry service, gunakan gate yang mirip `motionSafeForHeavyMaintenance()` tetapi jangan mensyaratkan E-stop clear hanya untuk melihat diagnostics; E-stop aktif justru perlu bisa didiagnosis.
Semua action berat seperti TFT test/verify tetap memakai `motionSafeForHeavyMaintenance()` existing.

Auto-return 60 s yang sekarang kembali ke OVERVIEW diubah menjadi kembali ke HOME.
`GOTO:SYSTEM` boleh dipertahankan sebagai compatibility diagnostic command tetapi tidak menampilkan SYSTEM di operator menu.
### 5.12 Rendering/cache changes

Jangan membuang dirty rendering existing.
Tambahkan cache khusus `UiHomeCache`, `UiMainMenuCache`, dan cache domain-card yang menyimpan id/status/page.

`drawUiFrame()` final Stage 1 dispatch:
- HOME -> `drawHome()`;
- MAIN_MENU -> `drawMainMenu()`;
- domain root -> `drawDomainMenu()`;
- detail/editor/manual/service -> renderer sesuai descriptor/type.

Full transition tetap membersihkan hanya gap/region yang perlu, bukan `fillScreen()` tanpa alasan.
Dynamic telemetry pass hanya menggambar text/status yang berubah.
Perubahan page/menu selalu full-layout pass, telemetry update selalu bounded dynamic pass.

### 5.13 Test yang harus diperbarui pada Stage 1

Update `test/hmi_current_geometry_self_check.py`:
- HOME dan MAIN_MENU in-bounds;
- tepat tiga main-domain card;
- SYSTEM tidak berada di MAIN_MENU;
- footer page navigation tidak overlap cards;
- manual test hit boxes tetap sama/in-bounds.

Update `test/hmi_10repo_audit_self_check.py` agar kontrak lama `four overview domains reachable` diganti:
- three operator domains visible;
- SYSTEM reachable only through service path;
- stale/fresh model tetap ada;
- dead-man dan release STOP tetap ada.
Tambahkan self-check baru untuk page-of-three:
- ESC 12 card -> tepat 4 page;
- PER 9 card -> tepat 3 page;
- NAV 12 card -> tepat 4 page;
- LEFT/RIGHT wrap per page;
- blank slot tidak clickable;
- BACK mempertahankan originating page.

Tambahkan self-check fail-safe manual motion:
- ESC stale -> STOP;
- ROS loss -> STOP;
- E-stop -> STOP semua motion;
- MANUAL->AUTO -> STOP;
- encoder lost -> steering STOP;
- touch release/slide-out -> STOP.

`test/spi_interrupt_display_robustness_self_check.py` harus tetap PASS tanpa melonggarkan assertion realtime/SPI/stack.
Preview generator `test/render_hmi_current.py` diperbarui menjadi HOME, MAIN_MENU, setiap domain page, detail representative, Service subset, dan Manual Test.

### 5.14 Acceptance Tahap 1

Tahap 1 dianggap selesai hanya jika:
- Splash -> HOME -> MENU -> tiga domain bekerja deterministik;
- MAIN_MENU tepat ESC/PER/NAV;
- ESC 4 page, PER 3 page, NAV 4 page, selalu max tiga card visible;
- ROOT page bergerak tiga item sekaligus, bukan sliding satu item;
- detail view dapat cycle tanpa nested menu buntu;
- POWER yang belum ada tampil N/A, bukan angka palsu;
- SYSTEM tidak terlihat operator tetapi masih dapat diakses service;
- seluruh existing safety/realtime self-check tetap PASS;
- manual test berhenti segera pada semua gate-loss di atas;
- tidak ada perubahan pada VESC raw forwarding, pin assignment, USB recovery, bootloader/DFU.
## 6. TAHAP 2 — Telemetry lengkap + seluruh domain + Service + asset + cleanup

### 6.1 Tujuan Tahap 2

Tahap 2 mengisi shell final Tahap 1 dengan data runtime yang benar-benar authoritative dan menghapus compatibility UI lama setelah migrasi terbukti stabil.
Target akhir: 12 ESC + 9 PERCEPTION + 12 NAVIGATION + 10 SERVICE cards tanpa menambah jalur kontrol berbahaya baru.

Prinsip utama:
- F4 tetap gateway/safety/HMI, bukan tempat menjalankan algoritma perception/Nav2;
- mini PC/ROS menormalisasi data kompleks menjadi telemetry ringkas;
- F4 parser tetap bounded, line-oriented, no JSON, no heap allocation;
- data yang tidak tersedia harus `N/A/STALE`, tidak diisi hardcode;
- setiap group memiliki validity/freshness sendiri bila cadence-nya berbeda.

### 6.2 Refactor `src/Telemetry.h`

Setelah Tahap 1 stabil, ubah flat struct menjadi nested POD structs tanpa dynamic allocation:
- `CommonTelemetry`
- `EscTelemetry`
- `PerceptionTelemetry`
- `NavigationTelemetry`
- `ServiceTelemetry` bila diperlukan untuk mirror non-local diagnostics.

`VehicleTelemetry` tetap menjadi snapshot tunggal yang mengandung nested structs tersebut agar pola immutable copy pada `drawUiNow()` tidak berubah.
Setiap group telemetry sebaiknya mempunyai:
- `uint32_t lastUpdateMs`;
- `uint32_t sourceAgeMs` bila source ROS mempunyai timestamp sendiri;
- `uint32_t validMask` untuk membedakan zero yang valid dari field belum tersedia;
- health enum `OK/WARN/FAULT/STALE/NA` yang dapat diturunkan saat render atau parser.

Jangan menggunakan `bool valid` per float bila jumlah field besar; validity bitmask lebih hemat RAM dan mudah di-snapshot.
String fixed-size tetap dibatasi seperti implementasi sekarang.

Common mempertahankan:
mode, state, systemStatus, ROS connected, E-stop, speed, active source, config transaction.

ESC mempertahankan field lama dan menambah:
Vbus, motor current, input current, Iq, Id, duty, MOS temp, motor temp, fault code/text, current-limit state, command watchdog state.
Tambahkan steering encoder raw count, logical angle, homed/calibrated/synced, A/B edge count, invalid transition, direction/inversion, observer/electrical angle bila source menyediakannya.
Tambahkan left/right motor summary hanya bila bridge benar-benar mempunyai dua stream distinct.

PER mempertahankan camera/inference/object/lane fields dan menambah:
object count, ROI-valid count, nearest lateral offset, lane center offset/confidence, road width/lookahead, drivable fraction/clearance, inference latency, dropped frame/recovery, lane/obstacle age.

NAV mempertahankan GNSS/IMU/EKF/Nav2/mission dan menambah:
odom X/Y/yaw, map pose summary, covariance/accuracy summary, yaw confidence/disagreement, costmap readiness/age/count, planner/controller/smoother state, path valid, cmd age, replan/failure count.
### 6.3 Parser/protocol split dari `src/main.cpp`

Parser telemetry sekarang berupa chain panjang di `handleSerialCommand()` sekitar area command `ROS:` sampai waypoint.
Jangan ikut memindahkan handler USB/DFU/TFT/VESC maintenance yang berada di fungsi yang sama.

Pisahkan hanya telemetry parsing menjadi modul baru, misalnya:
- `src/TelemetryProtocol.h`
- `src/TelemetryProtocol.cpp`

API minimal:
`TelemetryParseResult parseTelemetryLine(const char* line, VehicleTelemetry& t, uint32_t now);`
Result membawa `recognized`, `domain`, dan dirty bits yang dibutuhkan.

`main.cpp` tetap bertanggung jawab atas:
- command routing prioritas VESC/NEO/USB/DFU;
- config ACK transaction;
- action commands;
- safety state transition;
- UI dirty scheduling.

Parser baru hanya mengubah POD telemetry dan tidak boleh melakukan drawing, delay, UART write, atau motion action.
Setelah parser mengembalikan perubahan safety-critical, `main.cpp` menjalankan gate revalidation/STOP bila perlu.

Pertahankan parser legacy prefixes selama masa integrasi, kemudian hapus hanya setelah bridge baru terbukti mengirim contract final.
### 6.4 Contract telemetry baru

Gunakan prefix domain yang tidak bertabrakan dengan command lama, misalnya `ESCX:`, `PERX:`, `NAVX:`.
Format harus sederhana, satu key per line atau snapshot ringkas dengan field count tetap.

Direkomendasikan satu explicit snapshot marker per group agar freshness tidak dianggap fresh hanya karena satu field sporadis:
- `ESCX:SNAP:<seq>:<source_age_ms>`
- `PERX:SNAP:<seq>:<source_age_ms>`
- `NAVX:SNAP:<seq>:<source_age_ms>`

Field lines setelah/before marker membawa value; marker commit memperbarui group freshness.
Alternatif yang lebih aman adalah double-buffer small staging struct lalu commit ketika SNAP END diterima, bila bridge mengirim beberapa field sebagai satu epoch.

Jangan gunakan sembarang `ESCX:*` untuk me-reset seluruh domain age.
Subgroup cadence yang berbeda seperti power vs encoder dapat memiliki `lastPowerMs`, `lastEncoderMs`, dst.

Parser harus:
- reject NaN/Inf;
- clamp field hanya pada physical/display bounds yang jelas;
- menyimpan invalid sebagai invalid, bukan silently zero;
- count malformed/unknown extended telemetry;
- tidak membuat system READY dari field parsial.

### 6.5 Sumber VESC/FOC yang direkomendasikan

`src/VescGateway.cpp` sekarang memvalidasi framing/CRC dan meneruskan packet mentah sebagai `VESC:RX:<hex>`.
Pertahankan jalur ini sebagai primary transport; jangan masukkan full semantic VESC decoder ke hot path F4.
Primary flow:
`F103/VESC -> USART1 F4 -> VESC:RX raw -> mini PC decoder -> normalized ESCX telemetry -> USB CDC F4 HMI parser`.

Keuntungan:
- safety-stop latency tidak bertambah;
- VESC UART owner/recovery behavior tidak berubah;
- decode berat/versi-dependent berada di mini PC;
- F4 hanya menerima angka yang diperlukan layar.

Data `COMM_GET_VALUES`/selective yang layak di-mirror:
Vbus, motor current, input current, duty, RPM/eRPM, MOS temperature, motor temperature bila valid, fault code.
Iq/Id hanya dikirim bila firmware F103 memang mengekspos nilai tersebut secara authoritative; jangan menebak dari current biasa.

Jika suatu saat local decode F4 diperlukan untuk HOME power saat ROS mati, implementasikan sebagai observer kecil terhadap frame yang sudah valid, tanpa mengambil ownership UART dan tanpa mengubah forwarding.
Itu enhancement opsional setelah sistem utama stabil, bukan syarat Tahap 2.

### 6.6 ESC — mapping 12 cards

ESC Page 1/4:
1. ESC Overview
2. Operator Mode
3. Steering

ESC Page 2/4:
4. Drive
5. FOC / Power
6. Motor Telemetry
ESC Page 3/4:
7. Encoder
8. Calibration
9. Communication

ESC Page 4/4:
10. Fault & Safety
11. Performance
12. Manual Test

`ESC Overview` view 1: speed, drive target/actual, steering target/actual.
View 2: motor RPM/eRPM, VESC link/freshness, current VESC fault.
View 3: ESC ready, encoder ready, active command source.

`Operator Mode`: current AUTO/MANUAL, active source, motion gate, E-stop.
Edit AUTO/MANUAL tetap transaction-based `CMD:CFG` + ACK/readback.
Tambahkan zero-motion gate sebelum request mode change; jangan hanya mengandalkan ROS reject.

`Steering` view 1: target, actual, tracking error.
View 2: calibrated/homed/synced, raw count, logical angle.
View 3: physical/safe left-right range, operational limit, center/deadband.
Safe test menggunakan preset 5..30 deg dan dedicated hold control; center test harus berupa explicit target 0 dengan dead-man semantics yang sama.
`Drive` view 1: target m/s, actual m/s, eRPM/RPM.
View 2: driveScale, eRPM-per-m/s, odometry scale.
View 3: speed limit, command watchdog, autonomy/perception gate.
Manual drive tetap memakai existing 10..50% preset; FORWARD/REVERSE hanya hold-to-run.

`FOC / Power` view 1: Vbus, motor/input current, duty.
View 2: Iq, Id, current-limit state; field unavailable tampil N/A.
View 3: MOS/motor temperature, VESC fault.
Tidak ada current/duty override dari screen ini.

`Motor Telemetry` view 1: steering/left actuator summary bila distinct stream tersedia.
View 2: drive/right actuator summary.
View 3: comparison freshness/fault/telemetry age.
Jika sistem hanya mempunyai satu semantic VESC stream pada saat integrasi, tampilkan `SOURCE N/A` pada motor yang tidak tersedia; jangan menduplikasi angka yang sama seolah dua motor.

`Encoder` view 1: raw count, logical deg, synchronized/configured.
View 2: A/B edge counters, invalid transition/errors, inversion/direction.
View 3: mechanical angle, observer/electrical angle, tracking/observer error bila tersedia.

`Calibration` view 1: homed/calibrated/span/center summary.
View 2: physical left/center/right reference.
View 3: wheel/eRPM/odometry scaling summary.
Calibration action hanya muncul jika corresponding command/ACK path sudah benar-benar ada; sampai itu read-only.
`Communication` view 1: ROS<->F4 connected, heartbeat age, deferred queue/drop.
View 2: F4<->F103 UART valid-frame age, UART errors/overflow/drop/recovery.
View 3: VESC owner RUNTIME/MAINTENANCE, active baud, maintenance lease/route state.
Sebagian besar data view 2/3 sudah tersedia di `VescGateway`/`HmiDiagnostics`; expose read-only accessor tambahan bila perlu, jangan duplicate counters.

`Fault & Safety` view 1: VESC fault, E-stop, physical NEO3 safety state.
View 2: undervoltage/overvoltage/current/thermal limit summary hanya dari source valid.
View 3: command watchdog, source freshness, autonomy gate.
Tidak ada bypass/override safety.

`Performance` view 1: command/feedback age dan update rate dari bridge.
View 2: F103 ISR/GET_VALUES/watchdog summary bila benar-benar dikirim F103/bridge.
View 3: F4 service p95/p99/max, UI draw last/max, stack headroom.
Jangan memberi label `F103 ISR` pada `Board_ServiceGap` F4; kedua metric harus dipisahkan jelas.

`Manual Test` mempertahankan dedicated 5-button layout existing.
Tambahkan display gate reason saat locked: ROS OFFLINE / ESC STALE / AUTO MODE / ESTOP / ENCODER / MOVING.
STOP tetap fire-on-press; direction switch wajib melewati STOP.

### 6.7 ESC acceptance

- tidak ada angka power/fault/encoder yang hardcoded sebagai runtime telemetry;
- HOME Vbus dan FOC Vbus berasal dari field/source yang sama atau menunjukkan source label jelas;
- freshness power, encoder dan motor dapat menjadi stale independen;
- semua motion controls tetap dead-man;
- VESC raw forwarding byte-for-byte behavior tidak berubah;
- maintenance ownership masih mencegah runtime traffic bercampur maintenance traffic.
### 6.8 PERCEPTION — mapping 9 cards

PER Page 1/3:
1. Per Overview
2. Camera
3. Detection

PER Page 2/3:
4. Lane
5. Drivable Area
6. Obstacle

PER Page 3/3:
7. Performance
8. Calibration
9. Per Test

F4 tidak menampilkan live video. Yang ditampilkan hanya health, geometry summary, actionable safety state dan freshness.

`Per Overview` view 1: camera ready, perception ready, inference ON/OFF.
View 2: lane state, drivable clear/blocked, obstacle detected/nearest distance.
View 3: FPS, perception age, advisory/control state.

`Camera` view 1: camera connected/healthy/FPS.
View 2: resolution/requested FPS/backend summary dari ROS bila tersedia.
View 3: frame age, dropped frame/recovery/hotplug state.
Restart camera bukan one-tap operator action; bila disediakan harus masuk service-confirmed action.
`Detection` view 1: nearest class, distance, confidence.
View 2: total object count, ROI-valid count, nearest obstacle class/distance.
View 3: active tracks, missed/stale indicator, detection age.
Object string tetap fixed-size dan harus disanitize/truncate seperti implementasi sekarang.

`Lane` view 1: lane state, center offset, confidence.
View 2: near/far lookahead, estimated road width, lane correction.
View 3: lane-control state, requested steering correction, data age.
F4 hanya menampilkan hasil; line/ROI overlay editing tetap di ROS Web.

`Drivable Area` view 1: valid, clear/blocked, drivable fraction.
View 2: nearest clearance, left/right usable corridor, far lookahead.
View 3: lane constraint active, ROI/config validity, source age.

`Obstacle` view 1: nearest class, longitudinal distance, lateral offset.
View 2: blocked/clear, object/track count, ROI-valid state.
View 3: confirm/release counters, obstacle age, advisory output.

`Performance` view 1: FPS, inference latency, frame age.
View 2: backend CPU/GPU label, inference enable, dropped/recovery count.
View 3: publish/update rates, lane age, obstacle age.
`Calibration` view 1: obstacle distance model/scale/bias summary.
View 2: camera height/pitch, corridor offsets, safety margin summary.
View 3: calibration version/source, saved state, ROI validity.
Semua geometry editing rinci tetap di ROS Web; F4 maksimal menjalankan safe apply/reload bila contract ACK tersedia.

`Per Test` menjalankan read-only health wizard:
- camera heartbeat;
- inference state/readback;
- detection freshness;
- lane freshness;
- obstacle freshness;
- bridge ACK/status.
Tidak ada motion command dari PER TEST.

### 6.9 PERCEPTION acceptance

- camera disconnect mengubah Camera/Per Overview menjadi stale/fault tanpa menunggu seluruh ROS link putus;
- inference OFF dibedakan dari perception fault;
- object distance tanpa valid detection tidak ditampilkan sebagai `0.00 m` yang menyesatkan;
- lane/drivable/obstacle mempunyai validity dan age yang independen;
- no video/framebuffer transfer ditambahkan ke USB HMI protocol;
- PER test tidak memiliki jalur aktuator.

### 6.10 NAVIGATION — mapping 12 cards

NAV Page 1/4:
1. Nav Overview
2. Localization
3. GNSS
NAV Page 2/4:
4. IMU / MAG
5. EKF
6. Odometry

NAV Page 3/4:
7. Mission
8. Nav2
9. Safety

NAV Page 4/4:
10. Costmap
11. Path / Control
12. Nav Test

`Nav Overview` view 1: localization state, Nav2 ready, navigation status.
View 2: active target, goal/progress summary bila tersedia, vehicle speed.
View 3: GNSS fix/age, IMU health, autonomy motion gate.

`Localization` view 1: localization state, map pose age, heading.
View 2: EKF local/global state, active pose source.
View 3: position covariance/accuracy, yaw confidence, degrade reason.
Jangan mengubah `motionReady` hanya berdasarkan string status; authoritative readiness tetap berasal dari bridge/state machine.

`GNSS` view 1: fix, satellite count, hAcc.
View 2: latitude, longitude, heading.
View 3: HDOP, GNSS age, receiver/status.
Existing fields sudah cukup untuk hampir seluruh view ini.
`IMU / MAG` view 1: IMU ready/status, gyro Z, fused heading.
View 2: magnetometer ready, magnetic heading/field summary bila tersedia, sensor age.
View 3: GNSS heading vs IMU/MAG heading, disagreement angle, warning state.

`EKF` view 1: EKF local status, odom freshness, local yaw/pose summary.
View 2: EKF global status, map freshness, global position summary.
View 3: GNSS/IMU contribution validity, innovation/covariance warning summary bila source tersedia.

`Odometry` view 1: linear speed, yaw rate, steering actual.
View 2: odom X/Y/yaw dan age.
View 3: raw ESC speed vs filtered odometry speed, tracking error, driveScale.
Semua read-only; calibration edit tetap di ESC Calibration/ROS Web.

`Mission` view 1: selected waypoint, saved flag, name.
View 2: active target, navigation status, goal freshness.
View 3: current map pose + GNSS position summary untuk save-current context.

Pertahankan empat waypoint existing dan command `CMD:WP:SELECT`, `CMD:WP:GO`, `CMD:WP:SAVE`, `CMD:NAV:STOP` selama bridge masih menggunakannya.
GO tetap butuh AUTO + SYS_READY + navigationFresh + waypoint saved.
SAVE ditingkatkan dari sekadar GPS+STOP menjadi localization/navigation freshness yang valid sesuai source pose yang benar-benar disimpan bridge.
STOP NAV tetap immediate dan tidak memerlukan readiness gate.
`Nav2` view 1: stack/lifecycle readiness, active navigation state.
View 2: planner/controller/smoother readiness.
View 3: cmd freshness, perception advisory freshness, active ESC command source.
Tidak ada Nav2 tuning kompleks dari F4 HMI.

`Safety` view 1: E-stop, NEO3 safety input, autonomy motion allowed.
View 2: lane safety, obstacle blocked, perception advisory.
View 3: ESC watchdog, cmd freshness, localization freshness.
Safety page read-only dan tidak mempunyai bypass.

`Costmap` view 1: local costmap ready, obstacle-layer age, footprint/config state.
View 2: obstacle/clearing count + age summary, inflation/blocked state.
View 3: stale/fault reason, perception source state, Nav2 consumption state.

`Path / Control` view 1: SMAC/planner state, path valid, replan/failure count.
View 2: MPPI/controller state, linear/angular command, controller age.
View 3: smoother state, target/actual speed, steering/yaw tracking error.

`Nav Test` hanya health wizard read-only:
GNSS freshness -> IMU/MAG -> EKF local/global -> Nav2 lifecycle -> costmap input -> planner/controller.
Tidak ada autonomous motion test dari menu ini; motion commissioning tetap terpusat di ESC Manual Test.

### 6.11 NAVIGATION acceptance

- GNSS loss tidak otomatis menghapus odom/local state bila local estimator masih valid, tetapi global readiness berubah sesuai bridge;
- stale localization mengunci GO waypoint sebelum command dikirim;
- STOP NAV selalu dapat dikirim;
- costmap/path/controller field yang belum ada tampil N/A;
- tidak ada Nav2 parameter hardcoded ditampilkan seolah runtime.
### 6.12 SERVICE MODE — 10 cards final

Service Page 1/4:
1. System Health
2. IO Pin Monitor
3. Display Pins

Service Page 2/4:
4. Link Diagnostics
5. Error Counters
6. SPI Bus

Service Page 3/4:
7. UART Status
8. Power Status
9. Touch Panel

Service Page 4/4:
10. TFT Test
11-12. slot kosong, non-clickable.

Lima card pertama berasal dari SYSTEM renderer yang sekarang sudah ada.
Card 6-9 memecah counters existing agar engineering screen lebih mudah dibaca; tidak perlu membuat sensor/counter duplikat.

`SPI Bus`: transaction/bytes, timeout/HAL/bus conflict/recovery, current write clock + fast/ultra validation, frame bytes last/max.
`UART Status`: VESC + GNSS error/overflow/drop, VESC valid frame age/recovery, active baud/owner bila accessor tersedia.
`Power Status`: Vbus/validity bila telemetry tersedia; board supply yang tidak diukur hardware tidak boleh dibuat-buat.
`Touch Panel`: read count, fast reject count, touch state/calibration status; optional raw XY hanya saat service diagnostic mode.
`TFT Test`: controller ID/mode/MADCTL/pixfmt, verify current clock, color test. Semua heavy action tetap memakai existing motion-safe gate.
Service session behavior:
- hidden entry long-hold MENU 3 s;
- auto return ke HOME setelah 60 s idle;
- leaving Service invalidates touch generation;
- read-only diagnostics boleh dilihat saat E-stop aktif;
- maintenance/test action tetap membutuhkan stationary/motion-safe gate;
- tidak ada command untuk bypass E-stop/safety.

### 6.13 Asset/logo pipeline

Jangan memperbanyak pola `src/VisualAssets.h` saat ini untuk setiap icon.
Tiga RGB565 asset existing saja sudah memakai sekitar 38.95 kB.

Prioritas pertama: gunakan procedural vector-like icons dari `src/Icons.h` dan perluas fungsi primitive karena hampir tidak memerlukan bitmap flash.
Untuk icon yang harus mengikuti artwork/logo spesifik, gunakan mask palette compact:
- 1 bpp 56x56 sekitar 392 B/icon;
- 2 bpp 56x56 sekitar 784 B/icon;
- 43 icon sekitar 16.9 kB pada 1 bpp atau 33.7 kB pada 2 bpp, sebelum overhead kecil.

Buat registry icon terpisah, misalnya `src/IconAssets.h`, bukan satu file RGB565 raksasa.
Renderer menggambar background/card sendiri, lalu mask menggunakan navy/cyan palette dan label native font.

Setelah semua page tidak lagi memakai `VEHICLE_ASSET`, `CAMERA_ASSET`, `GPS_ASSET`, hapus asset RGB565 lama dan ukur kembali flash.
Jangan menghapusnya sebelum grep/reference audit memastikan nol consumer.
### 6.14 Descriptor architecture dan file split

`src/UiShell.h` sudah 1310 baris; jangan menambahkan seluruh 33 card detail ke switch yang sama sampai menjadi ribuan baris lagi.
Stage 2 pecah renderer secara compile-time/header atau cpp statis tanpa heap:
- `src/UiCatalog.h`: descriptor card/domain/page/view count/capability.
- `src/UiHome.h`: HOME + MAIN MENU.
- `src/UiEsc.h`: ESC detail renderers.
- `src/UiPerception.h`: PER detail renderers.
- `src/UiNavigation.h`: NAV detail renderers.
- `src/UiService.h`: service detail renderers.
- `src/UiShell.h`: chrome, generic cards/footer/dispatch saja.

Descriptor minimal:
`id, domain, label, iconId, viewCount, capabilityFlags`.
Capability flags contoh: READ_ONLY, EDIT_SAFE, ACTION_GUARDED, MOTION_DEADMAN, SERVICE_ONLY.

Status provider sebaiknya function/static switch yang membaca immutable telemetry snapshot, bukan lambda capturing/dynamic allocation.
Card page dihitung dari descriptor array order; tidak perlu tree array terpisah untuk setiap nested detail view.

`menuTitle`, `menuWireName`, `menuDomain`, `viewCount` diturunkan dari catalog yang sama bila memungkinkan agar tidak ada empat switch yang bisa tidak sinkron.

Setelah descriptor stabil, hapus nested legacy menu IDs `ESC_STEERING_LIVE`, `ESC_DRIVE_LIVE`, `NAV_PLANNER`, dst yang sudah digantikan oleh detail view, kecuali ada external wire dependency yang ditemukan saat integration audit.
### 6.15 Dirty rendering final

Perluas dirty bits dari global TOPBAR/CONTENT/FOOTER menjadi bounded regions bila measurement menunjukkan redraw content masih terlalu mahal:
- HOME metric 0/1/2;
- card slot 0/1/2;
- detail metric rows;
- status/fault strip;
- footer/page indicator.

Tetapi jangan menambah kompleksitas sebelum benchmark Tahap 1 menunjukkan kebutuhan.
Existing value cache `UiMetricCacheEntry` sudah mencegah repaint value identik dan harus dipertahankan.

Aturan render:
- telemetry identik -> 0 repaint pada value region;
- health color berubah -> redraw hanya status/value terkait;
- page/view transition -> bounded full layout redraw;
- E-stop transition boleh memaksa top/fault region segera dirty;
- tidak render saat touch masih down kecuali visual STOP/safety yang memang dibutuhkan.

### 6.16 Performance budget

Gunakan baseline aktual, bukan angka target yang tidak diukur.
Sebelum Tahap 2, simpan benchmark Tahap 1 untuk:
- `uiDrawLastMs/uiDrawMaxMs`;
- `displayBytesLastFrame/displayBytesMaxFrame`;
- `serviceGapP95/P99/max`;
- stack headroom/min;
- SPI timeout/HAL/recovery/conflict;
- VESC UART/frame errors dan recovery.

Acceptance utama: redesign tidak boleh memperburuk p95/p99 service atau error/drop transport secara material dibanding baseline yang sama pada hardware yang sama.
Target awal yang masuk akal:
- dynamic detail update tetap <= existing 50 ms warning threshold bila hardware memungkinkan;
- page transition tidak boleh menyebabkan watchdog/service starvation;
- zero new SPI timeout/bus conflict pada stress navigation;
- no increase VESC frame/UART errors akibat repaint;
- stack minimum tidak boleh mendekati guard area.

Existing fast/ultra SPI write validation harus tetap berdasarkan pixel readback; jangan hardcode 24 MHz sebagai selalu aman pada setiap unit/display.

### 6.17 Serial line dan RAM budget

Source saat ini mempunyai `serialRx[640]`, tetapi deferred command slot hanya `256 B` dan 64 slot.
Karena telemetry dapat masuk saat realtime display service aktif dan perlu masuk deferred queue, setiap extended telemetry line wajib <256 B; targetkan <=220 B termasuk prefix/delimiter.

Jangan membuat satu mega snapshot line berisi seluruh ESC/PER/NAV.
Gunakan beberapa line kecil + snapshot/commit marker agar tidak ditolak `enqueueDeferredCommand()`.

Existing deferred queue sendiri memakai sekitar 16 kB static RAM.
Jangan menambah framebuffer 320x240 atau queue telemetry kedua yang besar.
Gunakan POD structs puluhan/ratusan byte, fixed strings pendek, validity bitmask, dan small staging buffer saja.

Setelah penambahan nested telemetry, ukur kembali `.bss`, stack headroom dan min stack headroom.
Jika memory naik tidak proporsional, prioritaskan penghapusan asset/duplicate strings/cache sebelum mengurangi robustness queue tanpa bukti.
### 6.18 Freshness model final

Model sekarang menandai satu domain fresh saat salah satu prefix domain diterima.
Untuk 33 card, itu terlalu kasar.

Pertahankan domain freshness untuk top-level E/P/N, tetapi tambahkan group freshness:
ESC: motion, power, encoder, link.
PER: camera, detection, lane, obstacle.
NAV: gnss, imuMag, ekf, odom, mission, nav2, costmap, control.

Top-level domain `fresh` hanya berarti heartbeat/snapshot domain masih hidup.
Masing-masing card memakai freshness group sendiri untuk menentukan READY/STALE/N/A.

Contoh:
- power stale tidak harus membuat steering live stale;
- camera fresh tetapi detection stale harus membuat Detection warn/stale;
- GNSS stale tidak otomatis membuat Odom stale;
- costmap stale harus terlihat walau Nav2 heartbeat masih hidup.

Gunakan timeout sesuai cadence source, jangan memaksa semua group tepat 3000 ms bila publisher memang berbeda.
Tetap sediakan hard upper bound agar data lama tidak terlihat live tanpa batas.

### 6.19 Command/action contract

Semua edit existing tetap request/ACK/readback dan tidak mengubah local authoritative value sebelum mirror kembali.
Perluas pattern yang sama jika nanti ada action baru.

Action classes:
- READ_ONLY: langsung view only;
- EDIT_SAFE: draft -> OK -> CMD:CFG -> ACK/readback;
- ACTION_GUARDED: validate local gate -> command -> status/ACK;
- MOTION_DEADMAN: HOLD -> repeated/active command policy -> RELEASE/invalid gate -> STOP;
- SERVICE_ONLY: hidden session + motion-safe gate.
Jangan menambahkan one-tap command untuk:
current/duty override, raw FOC tuning, encoder detect, safety bypass, forced Nav2 motion, firmware reset.
Fungsi engineering tersebut tetap di VESC Tool/ROS Web/service workflow yang lebih tepat.

### 6.20 Integration dengan mini PC / ROS

Sebelum coding contract baru, audit node bridge aktual yang sekarang mengirim prefix `SPD:`, `DRIVE_ACT:`, `CAM:`, `NAV2:`, waypoint, dst.
Jangan membuat publisher kedua yang berlomba mengirim state sama.

Bridge final bertugas:
- decode/normalize raw VESC telemetry yang sudah diterima mini PC;
- publish only HMI-relevant summary;
- memberi source validity dan age;
- map ROS enums/status ke string/enum stabil;
- ACK config/action setelah perubahan benar-benar applied/readback;
- tidak mengirim angka default sebagai valid saat source hilang.

Rate guidance:
- motion/steering: cukup cepat untuk operator display, tetapi tidak perlu mengikuti ISR rate;
- power/FOC: 5-20 Hz cukup untuk HMI;
- perception status: mengikuti pipeline 5-20 Hz atau change-driven;
- navigation/GNSS: sesuai source 5-20 Hz;
- diagnostics: 1-5 Hz/change-driven.

F4 display refresh tetap 100 ms saat dirty; publisher yang jauh lebih cepat hanya menambah queue pressure tanpa meningkatkan visual usability.

### 6.21 Test protocol/parser

Tambahkan host-side parser self-check untuk setiap prefix baru:
valid value, min/max, NaN, Inf, malformed delimiter, overlong field, missing field, unknown key, stale sequence, out-of-order sequence.
Test juga sequence commit:
field partial tidak boleh mengubah committed snapshot menjadi READY sebelum snapshot marker valid.
Duplicate/out-of-order snapshot harus aman dan tidak merusak freshness.

Pastikan line test <=220 B dan buat explicit test bahwa line >=256 B tidak pernah menjadi format normal yang dihasilkan bridge.

### 6.22 UI state-machine test

Automated model test minimal:
- boot -> splash -> HOME;
- HOME MENU -> MAIN_MENU;
- MAIN -> ESC/PER/NAV;
- domain LEFT/RIGHT moves page, not one card;
- page wrap first/last;
- card slot opens correct id;
- detail view cycle 0..N-1;
- BACK detail -> originating domain page;
- BACK domain -> MAIN_MENU;
- HOME from MAIN -> HOME;
- Service hold invalid while moving/nav active;
- Service hold valid when stationary;
- Service idle timeout -> HOME.

Reachability assertion:
all 33 operator card IDs reachable exactly once from their intended domain catalog.
All 10 service cards reachable only through service catalog.
Tidak boleh ada orphan enum/descriptor.

### 6.23 Geometry/visual test

Generate preview untuk setiap unique layout, bukan hanya enam screenshot lama.
Minimum preview set: HOME, MAIN, 4 ESC roots, 3 PER roots, 4 NAV roots, 4 Service roots, representative read-only detail, editor, mission action, manual test, E-stop overlay, stale/N/A states.
Semua box harus dites terhadap 320x240 dan tidak overlap top/content/footer.
Text test wajib memakai string terpanjang yang mungkin: `FAULT & SAFETY`, `MOTOR TELEMETRY`, waypoint name 19 char, error reason 31 char.
Jika label tidak muat, gunakan deterministic 2-line layout; jangan mengecilkan font per card secara acak.

### 6.24 Robustness/fault matrix

Jalankan test dengan UI terus berpindah page sambil menyuntik kondisi:
- ROS heartbeat lost/recovered;
- ESC domain stale/recovered;
- VESC valid-frame loss/recovery;
- E-stop press/release;
- GNSS loss/reacquire;
- perception camera loss;
- stale lane/obstacle subgroup;
- TFT transfer fail + runtime recovery pada fault-test build;
- USB reconnect/session recovery;
- maintenance VESC ownership enter/exit.

Untuk setiap fault, verifikasi:
1. status visual benar;
2. stale value tidak terlihat READY;
3. motion action terkunci/STOP bila relevan;
4. transport recovery tetap berjalan saat repaint;
5. tidak ada reset tak terduga;
6. setelah recover, UI kembali live tanpa reboot.

Manual test tambahan: tahan FORWARD/STEER lalu cabut freshness/ROS secara simulasi; STOP harus dikirim tanpa menunggu jari dilepas.

### 6.25 Stress navigation

Minimal 500 page/view transition menggunakan event model atau hardware automation bila tersedia.
Pantau touch generation, page index bounds, renderer cache invalidation, SPI errors, service p99, VESC UART/frame errors, stack minimum.
### 6.26 Cleanup akhir Tahap 2

Setelah bridge + UI baru stabil, lakukan reference audit sebelum delete.
Hapus:
- `drawOverview()` lama bila HOME sudah final;
- sliding `menuWindowFirst()`/`selectedChild` bila tidak ada consumer;
- nested legacy detail menu IDs yang sudah digantikan view index;
- duplicate icon/assets;
- renderer branch unreachable;
- preview/test lama yang mengunci kontrak 4-domain OVERVIEW.

Compatibility wire yang masih dipakai external system boleh dipertahankan sebagai alias tipis:
`GOTO:OVERVIEW` -> HOME, `GOTO:SYSTEM` -> Service root bila safe/read-only.
Alias tidak boleh mempertahankan renderer mati.

Run compiler warnings dengan flag existing `-Wall -Wextra -Wformat=2 -Wshadow -Wundef` tanpa menambahkan blanket suppression baru.

### 6.27 Final build/resource gate

Catat hasil sebelum/akhir:
- flash text/data;
- `.bss`;
- firmware.bin size;
- stack headroom/min;
- full/dynamic draw timing;
- display frame bytes;
- transport error counters.

Gunakan `board_upload.maximum_size=360448` sebagai hard flash limit tetapi tetapkan engineering margin; jangan memenuhi flash hanya karena masih muat.
Bootloader/app offset dan linker script tidak disentuh oleh redesign HMI.
## 7. Urutan eksekusi nyata — hanya dua tahap

### TAHAP 1 order

1. Snapshot current git diff + test baseline; jangan reset perubahan USB/udev user.
2. Tambah HOME/MAIN state dan page-of-three state tanpa menghapus compatibility lama dulu.
3. Ubah domain classification dari numeric enum-range ke explicit domain mapping.
4. Implement HOME + MAIN MENU.
5. Ubah root ESC/PER/NAV menjadi catalog final 12/9/12 dengan detail placeholder dari telemetry existing.
6. Ubah carousel menjadi page-of-three dan pertahankan footer touch target besar.
7. Implement detailViewIndex dan flatten navigation untuk card yang sudah dapat dirender.
8. Hide SYSTEM dari operator path + implement service long-hold subset.
9. Tambah centralized manual-motion gate + steering running tracking + stop-on-stale/gate-loss.
10. Update geometry/reachability/safety previews dan self-check.
11. Build/check resource hanya sebagai validation; jangan mengubah transport untuk mengejar UI.
12. Review diff khusus memastikan file USB/udev existing tidak tertimpa.

Output Tahap 1:
struktur UX final, seluruh card reachable, data existing tampil benar, advanced field N/A, safety/realtime behavior tidak regresi.

### TAHAP 2 order

1. Freeze Tahap 1 baseline resource/timing/error counters.
2. Audit exact mini-PC HMI bridge publisher sebelum menetapkan field extended final.
3. Tambah nested telemetry + validity masks + subgroup freshness.
4. Extract telemetry-only parser dari `main.cpp` ke `TelemetryProtocol.*`.
5. Implement contract `ESCX/PERX/NAVX` dengan line <=220 B dan explicit snapshot/freshness marker.
6. Integrasikan normalized VESC power/FOC telemetry dari mini PC tanpa mengubah transparent gateway hot path.
7. Implement ESC advanced renderer + validation.
8. Implement PER advanced renderer + validation.
9. Implement NAV advanced renderer + validation.
10. Expand Service Mode menjadi 10 card.
11. Implement compact/procedural icon registry dan migrasikan asset.
12. Pecah `UiShell.h` menjadi catalog + domain renderer modules bila semua test masih stabil.
13. Jalankan parser/fault/stress/hardware regression matrix.
14. Hapus compatibility renderer/menu/dead assets setelah reference audit.
15. Final build/resource/performance comparison dan documentation update.

Output Tahap 2:
HMI final 3-domain + hidden Service, 33 operator cards + 10 service cards, telemetry authoritative, compact assets, no dead renderer, no transport/safety regression.

## 8. File impact matrix

`src/Config.h` — Stage 1: HOME/MAIN IDs, page/service constants, softkeys; Stage 2: cleanup enums.
`src/UiMenu.h` — Stage 1: page-of-three + domain mapping; Stage 2: replaced/thinned by catalog.
`src/UiShell.h` — Stage 1: HOME/MAIN/generic domain/detail shell; Stage 2: split domain-specific logic.
`src/TouchButtons.h` — Stage 1: new screen hit map + service hold; Stage 2: only minor action integration.
`src/Telemetry.h` — Stage 1: minimal Vbus-valid placeholder optional; Stage 2: nested structs/validity/freshness.
`src/main.cpp` — Stage 1: state/action/safety changes surgical; Stage 2: telemetry parser extraction only.
`src/VescGateway.cpp/.h` — no Stage 1 change; Stage 2 preferably no semantic decode, accessor only if diagnostics need it.
`src/Diagnostics.h` — Stage 1: reuse existing; Stage 2: add only counters genuinely needed for parser/group freshness.
`src/Icons.h` — Stage 1: reuse procedural icons; Stage 2: distinct icons + compact asset dispatcher.
`src/VisualAssets.h` — Stage 1: leave intact; Stage 2: remove/trim only after no references.
`src/HmiDisplay.cpp` + `include/HmiDisplay.h` — no redesign; preserve SPI transaction/recovery/text tile implementation.
`src/BoardSupport.cpp` — no HMI architecture changes; preserve IRQ priorities/realtime service instrumentation.
`src/usb/*`, `include/UsbCdcPort.h`, udev scripts — outside HMI redesign; do not overwrite current worktree changes.

New Stage 2 files yang direkomendasikan:
- `src/UiCatalog.h`
- `src/UiHome.h`
- `src/UiEsc.h`
- `src/UiPerception.h`
- `src/UiNavigation.h`
- `src/UiService.h`
- `src/TelemetryProtocol.h/.cpp`
- `src/IconAssets.h` bila bitmap mask diperlukan.

Test files:
- update `test/hmi_current_geometry_self_check.py`;
- update `test/hmi_10repo_audit_self_check.py`;
- keep `test/spi_interrupt_display_robustness_self_check.py` strict;
- update `test/render_hmi_current.py`;
- tambah page/state/parser/manual-failsafe self-check terpisah agar test tidak bergantung grep satu file monolitik.

## 9. Area yang sengaja tidak disentuh

- bootloader/recovery layout;
- linker app base/manifest scheme;
- USB CDC recovery work yang sedang modified;
- physical pin assignment;
- VESC runtime/maintenance owner arbitration;
- USART1 VESC priority/safety-stop behavior;
- GNSS/MAG algorithm internals;
- ROS perception/Nav2 algorithms;
- VESC firmware control laws/ISR.

Perubahan di area tersebut hanya boleh dilakukan bila ditemukan bug independen dan harus menjadi pekerjaan terpisah, bukan efek samping redesain UI.

## 10. Gate sebelum lanjut dari Tahap 1 ke Tahap 2

Jangan mulai telemetry expansion bila salah satu berikut gagal:
- boot/display recovery regression;
- geometry overlap;
- touch release/slide cancel;
- manual STOP on gate loss;
- SPI robustness self-check;
- VESC frame/UART stability;
- USB reconnect behavior;
- HOME/MAIN/domain reachability.

Stage 2 hanya dimulai setelah Tahap 1 mempunyai commit/checkpoint yang dapat direvert secara mandiri.
Karena worktree saat audit sudah mempunyai perubahan lain, checkpoint HMI harus dibuat tanpa memasukkan diff USB/udev yang tidak terkait.

## 11. Acceptance final seluruh redesign

1. Boot selalu berakhir di HOME setelah splash selesai.
2. HOME hanya memiliki SPEED, STATE, POWER + indikator kecil + satu MENU.
3. POWER invalid selalu `N/A/STALE`, tidak pernah angka default palsu.
4. MAIN MENU tepat ESC, PERSEPSI, NAVIGASI.
5. SYSTEM tidak terlihat pada operator path.
6. ESC = 12 card / 4 page; PER = 9 / 3; NAV = 12 / 4.
7. Maksimal tiga card terlihat dan LEFT/RIGHT berpindah per page.
8. Detail LEFT/RIGHT berpindah view dalam card yang sama.
