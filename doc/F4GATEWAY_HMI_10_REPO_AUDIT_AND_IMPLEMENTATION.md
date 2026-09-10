# F4Gateway HMI — Audit 10 Repository dan Rencana Implementasi

Tanggal audit: 2026-09-10  
Target: `/home/sirobo/agv/F4gateway`  
Referensi clone: `/home/sirobo/agv/hmi`  
Hardware target: STM32F411CEU6 96 MHz, ILI9341 320×240, XPT2046, SPI1, USB CDC, 2×UART, I2C1.

## 1. Prinsip audit

Audit ini tidak memindahkan framework atau aset secara buta. Setiap pola dinilai terhadap RAM/flash, latency motor-link, ukuran 320×240, dan safety AGV.

Baseline sebelum perubahan berhasil build dengan RAM 29,196 / 131,072 byte (22.3%) dan flash 124,364 / 360,448 byte (34.5%). Renderer sekarang adalah native/raw TFT, sehingga LVGL dipakai sebagai referensi UX saja, bukan dependency baru.

Kriteria adopsi:
- mudah dibaca pada 320×240;
- tidak menambah blocking loop pada gateway;
- tidak melakukan TFT access dari ISR;
- kontrol berbahaya harus lebih sulit terpicu tidak sengaja;
- diagnostics tetap dapat dibuka ketika ROS/subsystem offline;
- pin fisik yang dipakai gateway harus dapat dilihat dari HMI;
- data stale tidak boleh terlihat seolah-olah masih realtime.

## 2. Sepuluh repository yang dikloning
| # | Repository | Commit audit | Nilai utama untuk F4Gateway |
|---|---|---|---|
| 1 | `lvgl/lvgl` | `9e40f4b` | Grid/flex, style state, widget hierarchy, status semantics. Dipakai sebagai pola UX; framework penuh tidak diadopsi. |
| 2 | `Bodmer/TFT_eSPI` | `16e3759` | Referensi native ILI9341/XPT2046, meters, graphs, partial/direct rendering, STM32. |
| 3 | `jaklys/Lvgl-mcp-esp32` | `31b8ede` | System-info table, graph page, visual validation/screenshot concepts. |
| 4 | `Brokenagain-motorsport/Speeduino-Dash` | `012c167` | Automotive hierarchy, warning emphasis, live gauges, touch calibration, UI guard/stability. |
| 5 | `Tech-Tweakers/nebula-monitor` | `815ba26` | Raw TFT architecture, left status rails, pagination, deferred refresh, fault/detail diagnostics. |
| 6 | `davweb/waveshare-home-dashboard` | `b78d382` | Icon taxonomy, consistent icon sizing, explicit no-data/offline visual vocabulary. |
| 7 | `VahaC/ESP32-4848S040-EspHome-LVGL` | `285dabc` | Gesture cancel/lock model, idle navigation, synchronized visual state, high-contrast grouping. |
| 8 | `DaradiciLevente/ESP32-8048S070c-ESPHOME-HOME-ASSISTANT-DASHBOARD` | `aa3568f` | Strong numeric hierarchy, label/value/unit separation, technical dashboard visual language. |
| 9 | `Boisti13/papers3-dashboard` | `7dc4080` | Explicit navigation stack, home/back semantics, non-blocking boot philosophy. |
| 10 | `VaAndCob/ESP32-Serial-OBD2-Gauge-Catalyst` | `41adfda` | Closest physical UI analogue: 2.8-inch ILI9341 + XPT2046, automotive gauges, diagnostics/config pages. |

Seluruh repository di atas telah dikloning langsung ke `/home/sirobo/agv/hmi/<repo>` sehingga source, screenshot, logo, dan konsepnya dapat dibandingkan lokal tanpa bergantung pada koneksi berikutnya.

Catatan lisensi: beberapa repository tidak memiliki file LICENSE pada snapshot clone. Karena itu implementasi F4Gateway hanya mengambil pola/interaksi/komposisi; source dan aset visual pihak ketiga tidak disalin. Ikon baru dibuat secara procedural di firmware.
## 3. Audit visual yang diterapkan

### V01 — Hilangkan overlap dan buat hierarchy deterministik
Temuan: layout overview lama menempatkan kartu pada `y=126, h=108`, tetapi footer carousel mulai `y=188`; area `188..233` saling menimpa. Ini menyebabkan double/overlay dan target sentuh ambigu.

Implementasi: summary dipadatkan menjadi rail atas, tiga kartu domain terlihat pada satu window carousel, dan footer tetap memiliki area sendiri. Domain keempat (`SYSTEM`) diakses melalui carousel yang sama.

### V02 — Status bukan hanya merah/hijau
Mengikuti pola monitor produksi, status dibedakan menjadi READY, WAIT/FAULT, dan STALE. Data yang sudah lama tidak diperbarui tidak boleh terus ditampilkan sebagai realtime sehat.

Implementasi: green=fresh+ready, red=fresh+not-ready, grey=stale/never. Setiap domain mempunyai age/freshness sendiri.

### V03 — Selected item harus terbaca tanpa bergantung warna saja
Mengikuti pola left status rail/pagination, kartu terpilih mendapat border/rail geometris selain perubahan warna. Ini membantu keterbacaan pada TFT kecil dan kondisi pencahayaan buruk.
### V04 — Buat halaman diagnostics lokal, bukan hanya halaman operasi
Inspirasi OBD2, ebrew system-info, dan Nebula menunjukkan bahwa commissioning lebih cepat bila health, links, counters, dan I/O terlihat dari display yang sama.

Implementasi: domain `SYSTEM` berisi `SYSTEM HEALTH`, `IO PIN MONITOR`, `DISPLAY PINS`, `LINK DIAGNOSTICS`, dan `ERROR COUNTERS`.

### V05 — Pertahankan renderer native
LVGL sangat kaya, tetapi membawa allocator/widget/style/rendering layer yang tidak diperlukan pada gateway realtime ini. Baseline native hanya memakai 22.3% RAM dan 34.5% app flash.

Keputusan: tidak menambahkan LVGL. Pola grid, hierarchy, cards, state colors, dan navigation diimplementasikan dengan primitive TFT yang sudah ada.

### V06 — Ikon konsisten dan ringan
Mengikuti taxonomy Waveshare, fungsi ikon dibuat per domain/aksi dengan ukuran konsisten. Untuk menghindari masalah lisensi dan flash bitmap, ikon SYSTEM/pin dibuat procedural (`iconChip`, `iconPins`) dan aset repo tidak dicopy.

### V07 — Nilai utama lebih dominan daripada dekorasi
Pola OBD2/energy dashboard menunjukkan nilai harus lebih besar daripada label/unit, sedangkan diagnostics cocok dengan tabel label/value yang ringkas. Leaf page tetap memakai `drawMetricRow`, overview memakai ringkasan singkat, dan dekorasi background besar tidak diadopsi karena menambah flash tanpa meningkatkan commissioning.
## 4. Audit akses, robustness, dan safety

### R01 — Touch commit-on-release + cancel-on-slide
Kontrol biasa tidak lagi dieksekusi saat jari baru menyentuh layar. Aksi dikomit setelah release yang valid; gesture yang bergeser keluar dari target dibatalkan. Ini mengadopsi konsep gesture lock/cancel tanpa membawa gesture engine LVGL.

### R02 — Manual motion memakai dead-man hold
FORWARD/REVERSE/LEFT/RIGHT harus ditahan sekitar 0.65 detik sebelum aktif. Melepas atau menggeser jari keluar harus mengirim STOP. Tombol STOP tetap bereaksi langsung saat press. Timeout drive 3 detik yang sudah ada tetap menjadi lapisan keselamatan tambahan.

### R03 — Action gate harus memakai data fresh
Motion/config/navigation yang bergantung pada host tidak boleh diizinkan hanya karena nilai boolean lama masih tersimpan. Gate kritis harus memasukkan freshness domain terkait.

### R04 — Pin yang benar-benar terhubung harus terlihat
SYSTEM menampilkan level raw serta status periferal untuk PB6/PB7 VESC, PA2/PA3 GNSS, PB8/PB9 I2C, PB12/PB13 safety, PA5/6/7 SPI, PB0/1/2 TFT, PA4 touch CS, PA11/12 USB, dan PA8 buzzer.

### R05 — Error counters dan recovery visibility
UART error/overflow/TX drop, VESC frame error/recovery, parser unknown/overlong, TFT ID, dan stream age harus dapat dilihat tanpa laptop debugger.
### R06 — Rendering tidak boleh merebut jalur realtime
TFT tetap hanya digambar dari main loop, bukan ISR/UART callback. Refresh tetap dirty/throttled dan tidak dilakukan selama touch aktif. Tidak ada full framebuffer baru.

### R07 — Local diagnostics harus tetap tersedia saat ROS mati
Kehilangan ROS tetap memaksa state operasi ke safe/not-ready, tetapi halaman SYSTEM harus tetap dapat dinavigasi untuk mendiagnosis TFT, pin, UART, I2C, USB, dan error counters.

### R08 — External page control diperluas dengan aman
Perintah `GOTO:SYSTEM` boleh membuka diagnostics, tetapi command berbahaya tetap memakai gating/ACK yang sudah ada. Tidak menambah remote bypass untuk manual motion.

## 5. Acceptance checklist implementasi

- [ ] V01 overview tanpa overlap dan 4 domain reachable.
- [ ] V02 freshness/age terlihat dan stale tidak tampak READY.
- [ ] V03 selected card mempunyai cue geometris.
- [ ] V04 lima halaman SYSTEM lengkap.
- [ ] V05 tidak ada dependency LVGL baru; RAM/flash tetap aman.
- [ ] V06 ikon SYSTEM/pin original procedural.
- [ ] V07 hierarchy label/value tetap terbaca 320×240.
- [ ] R01 commit-on-release dan slide cancel.
- [ ] R02 hold-to-run + release-to-stop + STOP immediate.
- [ ] R03 critical gates menggunakan freshness.
- [ ] R04 seluruh pin fixed wiring dapat dimonitor dari SYSTEM.
- [ ] R05 counters/recovery/TFT-ID/stream age terlihat.
- [ ] R06 tidak ada TFT access dari ISR dan refresh tetap throttled.
- [ ] R07 SYSTEM tetap local/offline-capable.
- [ ] R08 `GOTO:SYSTEM` tersedia tanpa remote safety bypass.
- [ ] Build PlatformIO `blackpill_f411ce` sukses.
- [ ] `git diff --check` bersih.
- [ ] Existing recovery self-check tetap lulus.
- [ ] Self-check HMI audit baru lulus.

## 6. Batas implementasi yang disengaja

Grafik historis dan circular gauge penuh tidak dipasang pada tahap ini karena F4Gateway lebih penting sebagai commissioning/safety gateway daripada instrument cluster. Data history buffer dan anti-aliased meter akan menambah RAM/CPU dan tidak meningkatkan diagnosis link/pin secara langsung.

Background PNG besar juga tidak dipasang. Aset konsep tetap tersimpan di `/home/sirobo/agv/hmi`, sedangkan firmware memakai primitive RGB565 agar boot, flash, dan redraw tetap deterministik.

Dokumen ini menjadi source-of-truth. Bagian checklist dan hasil verifikasi di akhir dokumen hanya ditandai selesai setelah code build dan self-check benar-benar lulus.
## 7. Audit rinci per repository

### 7.1 `lvgl/lvgl`
**Yang relevan:** grid/flex layout, state-driven style, theme separation, event model, chart/meter examples. Repo juga menunjukkan bahwa widget framework lengkap membawa resource cost yang signifikan dibanding renderer sederhana.

**Diterapkan:** konsep layout deterministik, visual state terpusat, status color semantics, pemisahan renderer dari model state.

**Tidak diterapkan:** LVGL runtime, heap widget tree, full framebuffer, animations, charts historis. Alasan: gateway harus memprioritaskan USB/UART/VESC realtime; raw renderer sekarang sudah memenuhi kebutuhan dengan overhead sangat kecil.

### 7.2 `Bodmer/TFT_eSPI`
**Yang relevan:** ILI9341/XPT2046, STM32 support, direct drawing, smooth graphics/meter primitives, touch calibration, text padding dan partial redraw.

**Diterapkan:** tetap memakai primitive/direct rendering, calibrated touch path, padded dynamic text, no full framebuffer. Ini paling dekat dengan driver/display stack F4Gateway sekarang.

**Tidak diterapkan:** Sprite besar atau anti-aliased gauge penuh karena membutuhkan RAM ekstra dan tidak dibutuhkan untuk commissioning.

### 7.3 `jaklys/Lvgl-mcp-esp32`
Screenshot referensi lokal: `examples/ebrew-08-sysinfo.png` dan `examples/ebrew-07-graphs.png`.

**Yang relevan:** halaman system-info berupa label/value table yang mudah dipindai dan halaman graph yang memisahkan trend dari status sesaat.

**Diterapkan:** lima halaman SYSTEM memakai pola label/value table ringkas.

**Tidak diterapkan:** graph/history buffer; saat ini yang lebih penting adalah current state, age, error, dan pin visibility.
### 7.4 `Brokenagain-motorsport/Speeduino-Dash`
**Yang relevan:** dashboard otomotif dengan warning hierarchy kuat, touch calibration, meter RPM, colored threshold arcs, guard agar UI tidak disentuh sebelum siap, serta mode statis saat subsystem lain membutuhkan CPU.

**Diterapkan:** safety state tetap lebih dominan daripada dekorasi; STOP/E-STOP mempunyai prioritas visual dan input. Rendering dijaga dari re-entry (`gUiDrawActive`) dan refresh tetap dirty/throttled.

**Tidak diterapkan:** shift-light/gauge RPM besar karena F4Gateway bukan instrument cluster pengemudi.

### 7.5 `Tech-Tweakers/nebula-monitor`
**Yang relevan:** raw TFT untuk operasi 24/7, `pending_refresh`, pagination/group pages, left status strip, detail status/failure counters. Pendekatan ini paling sesuai dengan resource dan fungsi gateway.

**Diterapkan:** selected/health rail geometris, carousel window, diagnostics counters, status stale/ready/fault, native renderer tanpa framework tambahan.

### 7.6 `davweb/waveshare-home-dashboard`
Referensi lokal utama: `images/icons.yaml`, dengan taxonomy ikon dan ukuran konsisten; tersedia pula SVG Bootstrap/Material seperti home, memory, no-data, no-wifi, reboot.

**Diterapkan:** taxonomy ikon per domain/aksi dan dua ikon original procedural untuk chip/system serta pins.

**Tidak diterapkan:** SVG/PNG repo ke firmware agar lisensi/aset dan flash footprint tidak menjadi ketergantungan.

### 7.7 `VahaC/ESP32-4848S040-EspHome-LVGL`
**Yang relevan:** swipe state mempunyai start/last position, `swipe_locked`, threshold, cancel saat release, dan lock saat control lain sedang di-drag. Ada pula idle timer/settings return dan theme synchronization.

**Diterapkan:** inti robustness gesture — press tidak langsung melakukan aksi biasa, slide-out membatalkan tap, repeat mempunyai delay/rate yang eksplisit, manual motion memakai hold state terpisah.

**Tidak diterapkan:** swipe page navigation karena pada layar kendaraan kecil swipe mudah berkonflik dengan tombol commissioning; carousel button lebih deterministik.
