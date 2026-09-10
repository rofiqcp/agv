# F4gateway SPI / Interrupt / Display Robustness Audit

Tanggal audit: 2026-09-10
Target: `/home/sirobo/agv/F4gateway`
Referensi: 10 repo di `/home/sirobo/agv/hmi`

## 1. Tujuan

Audit ini khusus membandingkan arsitektur SPI, interrupt, touch, refresh, buffering, dan display ownership dari 10 repo referensi terhadap STM32F411CEU6 + ILI9341 + XPT2046 pada F4gateway.

Target utama bukan menambah efek grafis, tetapi:
- mempertahankan USART1/VESC sebagai jalur realtime tertinggi;
- mencegah TFT/touch membuat latency spike;
- mencegah deadlock SPI dan bus contention;
- mengurangi jumlah byte dan transaksi SPI per frame;
- membuat tampilan lebih responsif tanpa menambah framework berat;
- mempertahankan seluruh pin hardware yang sudah terhubung.

## 2. Hardware dan priority saat ini

- MCU: STM32F411CEU6, HCLK/APB2 96 MHz.
- SPI1: PA5 SCK, PA6 MISO, PA7 MOSI.
- TFT ILI9341: PB0 CS, PB1 DC, PB2 RESET.
- XPT2046: PA4 CS, shared SPI1; tidak ada pin PENIRQ/T_IRQ pada wiring aktif.
- USART1 VESC/F103: PB6/PB7, NVIC priority 0.
- USART2 GNSS: PA2/PA3, NVIC priority 1.
- USB OTG FS: PA11/PA12, NVIC priority 1.
- TIM11 watchdog aplikasi: priority 2.
## 3. Temuan paling penting pada F4gateway saat ini

### P0-A — redraw `full=false` masih menghapus hampir seluruh content

`drawUiContent()` selalu menjalankan `fillRect(0, CONTENT_Y-1, W, H-CONTENT_Y+1, C_BG)` sebelum menggambar isi. Artinya perubahan satu nilai telemetry tetap mengirim ulang area sekitar 320 x 208 pixel.

Pada SPI write saat ini, prescaler `/16` dari 96 MHz berarti sekitar 6 MHz. Hanya clear area tersebut membutuhkan sekitar 133,120 byte atau sekitar 177 ms waktu wire minimum, belum termasuk command, kartu, teks, ikon, dan HAL overhead.

Pada full redraw, `fillScreen()` dilakukan dahulu lalu content di-clear lagi. Secara minimum ada sekitar 286 kB transfer hanya untuk dua background pass sebelum widget aktual digambar.

**Kesimpulan:** bottleneck utama saat ini bukan kekurangan DMA, tetapi terlalu banyak area yang digambar ulang.

### P0-B — primitive teks/grafik menghasilkan transaksi SPI sangat banyak

`drawPixel()` melakukan `setWindow()` + write pixel. Font GFX menggambar glyph per pixel, dan Font2 menggunakan banyak `fillRect()` kecil. Satu label dapat memicu ratusan sampai ribuan command/data/CS toggle.

Ini meningkatkan latency, flicker, dan jumlah waktu main-context berada di renderer meskipun interrupt tetap aktif.

### P0-C — SPI prescaler switch belum memiliki timeout/recovery

`setSpiPrescaler()` menunggu `SPI_SR_BSY` tanpa batas. Bila peripheral masuk kondisi abnormal, loop hanya akan berakhir melalui watchdog reset beberapa detik kemudian.

Selain itu return value beberapa `HAL_SPI_Transmit/TransmitReceive` masih diabaikan sehingga display/touch fault belum menjadi fault yang terukur dan recoverable.
### P0-D — shared SPI belum memiliki explicit bus-owner state

TFT dan XPT2046 sudah aman secara dasar karena CS lawan selalu dinaikkan, tetapi belum ada abstraction `IDLE/TFT_WRITE/TFT_READ/TOUCH` yang mendeteksi nested transaction, illegal CS state, atau re-entry.

Hal ini penting karena `Board_RealtimeService()` memang dipanggil di tengah long draw. Saat ini callback realtime tidak memakai SPI dan `gUiDrawActive` mencegah recursive UI draw, sehingga aman. Namun invariant ini belum dipaksa oleh driver SPI.

### P1-A — touch polling cukup mahal

Implementasi XPT2046 mengikuti pola validasi TFT_eSPI: pressure debounce, dua raw sample, lalu `getTouch()` mengulang validasi sampai 5 kali. Tanpa PENIRQ/T_IRQ, polling 50 Hz dapat memakan beberapa milidetik bahkan saat tidak ada sentuhan.

**Rekomendasi:** fast pressure pre-check sekali; hanya bila pressure valid lakukan 2–3 sample koordinat dan filter. Jangan menambah EXTI touch software karena pin T_IRQ memang belum terhubung.

### P1-B — interrupt hierarchy bisa dibuat lebih eksplisit

USART1/VESC priority 0 sudah benar. USART2/GNSS dan USB OTG FS saat ini sama-sama priority 1. Untuk determinisme lebih kuat direkomendasikan hierarchy:

1. USART1 VESC = 0
2. USART2 GNSS = 1
3. USB OTG FS = 2
4. TIM11 watchdog = 3
5. Jika SPI DMA ditambahkan: completion IRQ = 4 atau 5

TFT biasa tidak perlu SPI IRQ. Renderer tetap main-context, sehingga motor UART dapat preempt kapan saja.
## 4. Audit ulang 10 repo

### 1. `TFT_eSPI` — PRIORITAS TERTINGGI

Temuan relevan:
- memakai lifecycle `begin_tft_write()` / `end_tft_write()`;
- write, read, dan touch memiliki SPI frequency profile terpisah;
- touch selalu menaikkan TFT CS sebelum memilih XPT2046;
- sebelum touch/read, DMA yang aktif harus selesai;
- STM32F4 SPI1 DMA dipetakan ke DMA2 Stream3 Channel3;
- API DMA memperingatkan ownership buffer dan kewajiban menunggu transfer selesai.

Yang diterapkan ke F4:
- transaction guard eksplisit;
- speed profile TFT_WRITE / TFT_READ / TOUCH;
- BUSY/RXNE/OVR cleanup sebelum reconfigure;
- satu CS transaction untuk set-window + pixel burst;
- DMA hanya sebagai tahap opsional sesudah dirty rendering stabil.

Yang tidak boleh dicopy mentah:
- priority DMA default repo tidak cukup eksplisit untuk sistem motor; pada F4gateway harus lebih rendah dari UART/USB.

### 2. `lvgl` — PRIORITAS TINGGI sebagai konsep renderer

Temuan relevan: LVGL memiliki invalid-area tracking, PARTIAL render mode, flush callback per area, dan optional double buffer.

Yang diterapkan: lightweight dirty-rectangle/custom invalidation ke renderer native. Jangan memindahkan LVGL penuh karena native renderer saat ini jauh lebih ringan dan kebutuhan UI tidak memerlukan object tree LVGL.
### 3. `nebula-monitor` — PRIORITAS TINGGI

Temuan relevan:
- raw TFT_eSPI dipilih untuk mengurangi overhead;
- update dari worker tidak menggambar langsung, hanya menandai `pending_refresh`;
- semua display/touch diproses di satu display task;
- XPT2046 memanfaatkan T_IRQ dan melakukan filtered sampling.

Yang diterapkan:
- pertahankan single display owner;
- telemetry/parser hanya mengubah model + dirty flags;
- renderer yang memutuskan area mana yang perlu dicat;
- pertahankan `gUiDrawActive` dan tambah SPI bus-owner assertion.

Yang tidak diterapkan: dual-core FreeRTOS task model karena STM32F411 single-core bare-metal.

### 4. `Speeduino-Dash` — PRIORITAS MENENGAH-TINGGI

Temuan relevan:
- `lvReady` guard mencegah akses UI sebelum siap;
- portal mode menggambar static screen sekali lalu menghentikan update berat;
- SD menggunakan SPI instance terpisah agar tidak bertarung dengan display;
- mode transisi secara eksplisit mereset state serial/UI.

Yang diterapkan:
- `displayReady/displayFaulted` state eksplisit;
- jika SPI/TFT recovery gagal, masuk static/degraded display state dan jangan terus melakukan redraw berat;
- periferal SPI baru di masa depan tidak boleh langsung memakai SPI1 shared tanpa bus arbiter.

### 5. `waveshare-home-dashboard` — PRIORITAS MENENGAH-TINGGI

Temuan relevan: recursive LVGL mutex, dirty-area tracking/copy, flush lifecycle, dan buffer ownership yang tegas.

Yang diterapkan: bukan FreeRTOS mutex, tetapi lightweight non-recursive SPI owner + UI owner guard; dirty area disimpan sebagai fixed-size structure tanpa heap.
### 6. `papers3-dashboard` — PRIORITAS MENENGAH

Temuan relevan: UI update hanya menandai dirty; physical display commit dipisahkan dan rate-limited. Driver juga membedakan fast refresh dan full-quality refresh.

Yang diterapkan:
- pisahkan `model changed` dari `paint now`;
- coalesce banyak update telemetry menjadi satu paint window;
- full paint hanya pada menu/page/layout change;
- dynamic values memakai bounded refresh rate dan dirty region.

### 7. `ESP32-4848S040-EspHome-LVGL` — PRIORITAS MENENGAH

Temuan relevan: 25% draw buffer, gesture lock saat slider/interactive control aktif, dan idle page handling.

Yang diterapkan: interaction lock concept — jangan interpretasi gesture/tap lain selama manual hold/edit transaction aktif. Buffer 25% tidak ditiru karena tidak dibutuhkan.

### 8. `ESP32-8048S070c-ESPHOME-HOME-ASSISTANT-DASHBOARD` — PRIORITAS MENENGAH-RENDAH

Temuan relevan: display update 100 ms dan touch 50 ms dipisahkan; static background/image memberi visual hierarchy yang konsisten.

Yang diterapkan: cadence terpisah display/touch, tetapi F4 harus lebih hemat redraw. Background bitmap besar/PSRAM pattern tidak cocok.

### 9. `ESP32-Serial-OBD2-Gauge-Catalyst` — REFERENSI VISUAL/HARDWARE

Repo yang ter-clone berisi README/assets, bukan source runtime lengkap. Nilai utamanya adalah bukti konsep automotive dashboard pada ILI9341 + XPT2046 dengan hierarchy gauge/status yang mudah dibaca.

Yang diterapkan: prioritaskan angka kendaraan utama dan warning state; jangan memperberat foreground dengan banyak elemen yang selalu berubah.

### 10. `Lvgl-mcp-esp32` — PRIORITAS TESTING

Temuan relevan: simulator headless menghasilkan screenshot dan JSON widget tree.

Yang diterapkan: ide visual regression test di host — render setiap halaman dengan telemetry fixture, simpan PNG/reference geometry, lalu assert tidak ada overlap/out-of-bounds. Tidak masuk ke firmware runtime.
## 5. Bagian F4gateway yang sudah baik dan harus dipertahankan

1. USART1/VESC priority 0 dan RX/TX interrupt ring buffer sudah benar.
2. Renderer TFT berjalan di main-context, bukan di ISR.
3. Interrupt tetap enabled selama blocking HAL SPI transfer sehingga UART dapat preempt display.
4. Bulk fill/pushImage sudah dipotong menjadi blok 256 byte dan memanggil `Board_RealtimeService()` antar blok.
5. `Board_RealtimeService()` memiliki recursion guard dan throttle 1 ms.
6. `gUiDrawActive` mencegah recursive draw ketika host command diproses saat renderer sedang aktif.
7. TFT CS dan touch CS sudah saling dinonaktifkan sebelum transaksi.
8. Touch menggunakan clock lebih rendah daripada TFT.
9. Watchdog aplikasi akan reset jika main loop benar-benar macet.

Ini berarti desain dasar tidak perlu diganti. Yang diperlukan adalah mengurangi SPI workload dan memperkuat transaction/error handling.

## 6. Arsitektur SPI yang direkomendasikan

Tambahkan satu state machine ringan pada `HmiDisplay`:

```text
IDLE
  -> TFT_WRITE
  -> TFT_READ
  -> TOUCH
```

Setiap `begin*Transaction()` wajib:
- memastikan state IDLE;
- menaikkan kedua CS terlebih dahulu;
- bounded-wait hingga BSY clear;
- drain RXNE / clear OVR;
- set prescaler sekali;
- memilih device CS;
- mencatat waktu mulai dan transaction counter.

Setiap `end*Transaction()` wajib menunggu final BSY secara bounded, deassert CS, drain RX, dan kembali ke IDLE.
### Critical section

Interrupt **jangan dimatikan selama pixel transfer**. Jika perlu atomicity saat `SPE=0 -> BR update -> SPE=1`, critical section harus hanya beberapa instruction/mikrodetik.

USART1 priority 0 harus tetap dapat mem-preempt hampir seluruh proses rendering.

### Clock profile

Current native code menggunakan:
- TFT write/read: SPI1 prescaler `/16` = sekitar 6 MHz;
- touch: `/64` = sekitar 1.5 MHz.

File legacy TFT_eSPI menyebut target 10 MHz write, 6 MHz read, 2.5 MHz touch. Dengan APB2 96 MHz tidak semua angka tersebut dapat dicapai persis.

Rekomendasi robust:
- default TFT write tetap `/16` sampai benchmark/signal test lulus;
- optional validated profile `/8` = 12 MHz untuk write saja;
- TFT register read tetap `/16`;
- XPT2046 tetap `/64` karena `/32` = 3 MHz sudah melebihi profile 2.5 MHz yang dipakai referensi;
- jangan mengubah speed di setiap byte/function kecil; lakukan sekali per transaction.

## 7. Timeout dan recovery SPI — wajib sebelum DMA

Semua wait terhadap `BSY/TXE/RXNE` harus memiliki deadline berbasis DWT cycle counter, bukan infinite loop.

Jika timeout/error terjadi:
1. TFT CS HIGH dan TOUCH CS HIGH;
2. disable SPI1;
3. drain DR/SR dan clear OVR;
4. `HAL_SPI_DeInit/Init` atau recovery register yang setara;
5. restore safe clock `/16`;
6. increment `spiTimeoutCount` dan `spiRecoveryCount`;
7. retry satu kali;
8. bila retry gagal, mark `displayFaulted`, jangan terus melakukan full redraw loop.

Motor/safety tidak boleh ikut di-reset hanya karena display gagal.
## 8. Tampilan: perubahan dengan impact terbesar

### 8.1 Hapus full-content clear dari incremental frame

`drawUiNow(false)` tidak boleh lagi otomatis menghapus seluruh content region. Pisahkan:

- **layout/static layer**: background, card, border, label, divider, icon;
- **dynamic layer**: angka, status dot, age, RPM, steering, GNSS, fault state.

Static layer digambar saat:
- boot;
- pindah page/menu;
- page layout berubah;
- display recovery selesai.

Dynamic layer hanya menghapus bounding box nilai lama dan menggambar nilai baru.

### 8.2 Dirty mask fixed-size, tanpa heap

Tidak perlu generic compositor. Gunakan bitmask sederhana, contoh:

```text
DIRTY_TOPBAR
DIRTY_SUMMARY
DIRTY_ROW0
DIRTY_ROW1
DIRTY_ROW2
DIRTY_ROW3
DIRTY_FOOTER
DIRTY_FULL_PAGE
```

Parser telemetry hanya set bit yang relevan. Main loop menggabungkan/coalesce bit hingga paint window berikutnya.

### 8.3 Cache nilai terakhir

Simpan snapshot kecil per page atau global telemetry-display cache. Bila string/nilai yang akan ditampilkan sama dengan frame sebelumnya, jangan kirim SPI sama sekali.
### 8.4 Batch text/glyph rendering

Renderer font sekarang banyak memakai `drawPixel()`/`fillRect()` untuk tiap bit glyph. Ini sangat mahal karena setiap pixel/spans kecil mengulang address-window command.

Tahap aman:
- scan setiap glyph per row dan gabungkan pixel berurutan menjadi horizontal run;
- satu `fillRect()` per run, bukan satu pixel;
- lebih baik lagi, gunakan static scanline/tile RGB565 kecil lalu `pushImage()` satu kali untuk satu label/value bounding box.

Buffer yang realistis untuk F411:
- scanline 320 pixel RGB565 = 640 byte;
- tile 160 x 16 RGB565 = 5,120 byte;
- tidak perlu framebuffer 320 x 240 = 153,600 byte karena melebihi SRAM praktis.

### 8.5 Transaction batching

`setWindow()` saat ini memanggil command/data dengan CS toggle berkali-kali. Terapkan pola TFT_eSPI `startWrite/endWrite`:

```text
beginTftWrite()
  CASET + 4 byte
  PASET + 4 byte
  RAMWR
  pixel burst
endTftWrite()
```

Satu rectangle menjadi satu bus transaction. Ini mengurangi prescaler switching, CS toggle, HAL call count, dan kemungkinan state tidak konsisten.

### 8.6 Refresh cadence

Touch tetap sekitar 20 ms, tetapi display dynamic tidak harus 100 ms untuk seluruh layar.

Rekomendasi:
- motor/steering numerical region: max 10 Hz bila benar-benar berubah;
- GNSS/perception text: 5–10 Hz;
- diagnostics/system page: 2 Hz;
- static regions: event-driven only.

Data tetap diproses realtime; hanya paint yang di-rate-limit.
## 9. Touch XPT2046

Karena pin T_IRQ/PENIRQ tidak termasuk wiring aktif, solusi robust harus tetap polling.

Optimasi yang direkomendasikan:
1. satu `readTouchZ()` sebagai fast reject;
2. jika Z di bawah threshold, return dalam satu transaksi tanpa 5 kali validasi;
3. bila valid, ambil 3 raw point;
4. gunakan median/mean setelah membuang sample yang beda terlalu jauh;
5. pertahankan dead-man state machine pada `TouchButtons.h`;
6. saat TFT transaction aktif, jangan mencoba menyelipkan touch read ke tengah RAMWR burst.

Bila hardware suatu hari menambahkan kabel T_IRQ ke pin EXTI yang benar-benar kosong, barulah touch IRQ dapat dipakai hanya sebagai wake/event flag. ISR tidak boleh membaca SPI; ISR cukup set flag, pembacaan XPT2046 tetap di main context.

## 10. Apakah perlu SPI DMA?

**Ya, tetapi bukan prioritas pertama.**

TFT_eSPI membuktikan STM32F4 SPI1 TX dapat memakai DMA2 Stream3 Channel3. Namun dirty rendering + transaction batching diperkirakan memberi benefit lebih besar dan lebih sederhana.

Jika DMA diterapkan setelah tahap tersebut:
- DMA hanya untuk pixel burst besar (`fillRect`/`pushImage`), bukan command atau touch;
- buffer DMA harus static dan hidup sampai transfer selesai;
- transfer dipotong, misalnya 1–4 kB per chunk;
- antar chunk jalankan realtime service dan cek safety;
- DMA completion IRQ diberi priority 4/5, lebih rendah dari USART1, USART2, USB, watchdog;
- sebelum touch/read/reconfigure SPI, DMA harus benar-benar selesai;
- timeout DMA wajib ada;
- CS TFT tetap LOW selama satu logical rectangle, tetapi jangan menahan bus untuk frame besar tanpa chunk/service point.

Jangan memakai DMA full-screen kontinu sebagai langkah pertama karena touch ikut kehilangan akses shared SPI selama transfer dan failure mode menjadi lebih kompleks.
## 11. Interrupt policy yang direkomendasikan

| IRQ | Priority | Alasan |
|---|---:|---|
| USART1 / VESC-F103 | 0 | motor link, tidak boleh ditunda UI |
| USART2 / GNSS | 1 | sensor navigation continuous RX |
| USB OTG FS | 2 | host/HMI high-throughput, boleh kalah dari UART sensor |
| TIM11 watchdog | 3 | watchdog tetap preempt main display |
| SPI1 TX DMA optional | 4 atau 5 | display acceleration, tidak realtime |

Aturan:
- tidak ada drawing di ISR;
- tidak ada `HAL_SPI_Transmit*` dari ISR;
- DMA ISR hanya clear/complete state;
- ISR tidak memanggil parser UI;
- safety/motor stop tetap melalui jalur yang sudah ada dan priority motor tidak diturunkan.

## 12. Diagnostics baru yang sebaiknya ditambahkan

Tambahkan counter/readback pada SYSTEM diagnostics:
- `spiTransactions`;
- `spiBytesTx`;
- `spiTimeoutCount`;
- `spiHalErrorCount`;
- `spiRecoveryCount`;
- `spiBusConflictCount`;
- `touchReadCount` dan `touchRejectFastCount`;
- `displayDirtyFrames` vs `displayFullFrames`;
- `displayBytesLastFrame` / `displayBytesMaxFrame`;
- `uiDrawLastMs` dan `uiDrawMaxMs` yang sebenarnya sudah dihitung tetapi belum ditampilkan;
- `Board_MaxServiceGapMs()` pada SYSTEM page.

Dengan ini optimasi tidak dinilai dari perasaan visual saja, tetapi dari bukti latency dan byte count.
## 13. Urutan implementasi yang disarankan

### Tahap A — instrumentation terlebih dahulu
- tampilkan draw time dan max service gap;
- hitung SPI bytes/transactions/errors;
- ukur baseline pada overview, ESC live, SYSTEM, dan manual test.

### Tahap B — SPI transaction + bounded recovery
- bus owner state;
- begin/end TFT/touch transaction;
- bounded BSY wait;
- OVR/RX drain;
- error/recovery counter;
- jangan ubah pin atau behavior motor.

### Tahap C — dirty renderer
- hilangkan content-wide clear pada incremental update;
- static/dynamic layer;
- value cache;
- dirty mask per region;
- batch setWindow + pixel transfer.

### Tahap D — font/glyph batching
- horizontal run atau scanline tile;
- ukur penurunan transaction count dan draw time.

### Tahap E — touch fast path
- pressure precheck;
- 3-sample filter hanya saat pressed;
- test tap, hold, slide-out STOP, dan noise.

### Tahap F — optional SPI DMA
Hanya dilakukan bila setelah A–E latency/CPU masih belum memenuhi target.
## 14. Acceptance test wajib

### Static/code test
- seluruh SPI wait memiliki timeout;
- hanya satu owner SPI1 pada satu waktu;
- TFT CS dan TOUCH CS tidak pernah LOW bersamaan;
- tidak ada SPI/display call dari ISR;
- USART1 tetap priority 0;
- optional DMA priority lebih rendah dari communication IRQ.

### Runtime hardware test
- TFT ID/read register tetap valid setelah 1000+ page transition;
- tap XPT2046 tetap stabil setelah full draw dan burst telemetry;
- manual hold/release selalu STOP saat jari lepas/keluar area;
- VESC UART error/overflow/drop tidak meningkat saat display stress;
- GNSS frame tidak drop signifikan saat display stress;
- USB telemetry 1 Mbaud tetap responsif;
- E-stop latency tidak memburuk saat full display paint.

### Display stress
- loop page OVERVIEW/ESC/PERCEPTION/NAV/SYSTEM;
- update RPM/ERPM/steering 10 Hz;
- update perception/GNSS bersamaan;
- lakukan touch terus-menerus;
- paksa redraw recovery dan TFT register read;
- bila DMA dipakai, campur touch tepat setelah DMA completion.

### Target kuantitatif awal
- incremental frame normal: < 25 ms ideal, < 50 ms hard target;
- page/full redraw: < 150 ms target;
- `Board_MaxServiceGapMs`: tidak naik tajam akibat renderer;
- SPI timeout/recovery: 0 pada operasi normal;
- UART1 overflow/drop: 0 pada display stress;
- tidak ada full-screen clear pada telemetry-only update.

## 15. Kesimpulan

Perubahan paling bernilai **bukan langsung menambahkan DMA**. Urutan terbaik adalah: transaction ownership -> timeout/recovery -> dirty rendering -> batched text -> touch fast path -> baru evaluasi DMA.

Dari 10 repo, kombinasi terbaik untuk F4gateway adalah pola transaction/DMA dari TFT_eSPI, invalidation dari LVGL, single display owner dari nebula-monitor, dirty/rate-limited commit dari papers3-dashboard, dan readiness/degraded-mode guard dari Speeduino-Dash.

Seluruh rekomendasi dapat diterapkan tanpa mengubah pin yang saat ini terhubung dan tanpa menurunkan priority jalur motor USART1.
