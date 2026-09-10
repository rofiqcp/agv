# F4gateway — Deep Re-Audit 10 HMI Repositories

Tanggal snapshot audit: 2026-09-10 08:27 WIB  
Target: `/home/sirobo/agv/F4gateway`  
Branch: `v1`  
HEAD baseline: `ea3958e82d530fe6e401075b9cc9552722db5e53`  
Working tree saat audit: 20 file dirty/untracked; perubahan belum seluruhnya di-commit.

## 1. Tujuan dan ruang lingkup

Audit ini mengulang pemeriksaan dari source aktual F4gateway setelah implementasi audit sebelumnya.
Audit tidak hanya menilai tampilan, tetapi seluruh jalur yang dapat memengaruhi robustness HMI:
- SPI1 shared bus ILI9341 + XPT2046;
- interrupt priority dan realtime service;
- display ownership, timeout, fault, recovery;
- touch sampling dan dead-man control;
- dirty rendering, text pipeline, geometry primitives;
- page transition dan full-frame latency;
- diagnostics, provenance, dan observability;
- boot/recovery interaction;
- SRAM/flash budget;
- test coverage dan visual regression.

Audit membandingkan source F4 aktual dengan 10 repository lokal di `/home/sirobo/agv/hmi`.
## 2. Repository snapshot yang benar-benar diaudit

| No | Repository | HEAD | Nilai utama untuk F4 |
|---:|---|---|---|
| 1 | `TFT_eSPI` | `16e3759` | SPI transaction, STM32 DMA, ILI9341 driver |
| 2 | `lvgl` | `9e40f4b` | invalidation, partial render, flush lifecycle |
| 3 | `nebula-monitor` | `815ba26` | single display owner, pending refresh, XPT2046 |
| 4 | `Speeduino-Dash` | `012c167` | automotive realtime UI, PrevData, stale/warning |
| 5 | `waveshare-home-dashboard` | `b78d382` | UI locking, unchanged-data suppression, OTA identity |
| 6 | `papers3-dashboard` | `7dc4080` | dirty-vs-physical refresh separation |
| 7 | `ESP32-4848S040-EspHome-LVGL` | `285dabc` | touch lock/cancel, page lifecycle, idle behavior |
| 8 | `ESP32-8048S070c-ESPHOME-HOME-ASSISTANT-DASHBOARD` | `aa3568f` | hierarchy, cadence, typography |
| 9 | `ESP32-Serial-OBD2-Gauge-Catalyst` | `41adfda` | automotive visual concept only |
| 10 | `Lvgl-mcp-esp32` | `31b8ede` | screenshot + JSON geometry regression |

Catatan penting: repo #9 pada clone lokal **tidak mengandung source `.cpp/.h/.ino`**.
Repo tersebut tidak boleh dijadikan bukti implementasi SPI/runtime; nilainya hanya visual dan hardware concept.
Dengan kriteria “master program”, repo #9 sebaiknya diganti pada iterasi riset berikutnya.
## 3. Executive finding

### P0-1 — runtime SPI recovery masih dapat mematikan semua interrupt

`HmiDisplay::recoverSpi()` memanggil `Board_ReinitSpi1()`.
`Board_ReinitSpi1()` kemudian memanggil `Spi1_Init()`.
`Spi1_Init()` masih memanggil `FatalError()` bila `HAL_SPI_Init()` gagal.
`FatalError()` melakukan `__disable_irq()` lalu infinite loop.

Ini bertentangan langsung dengan desain degraded display:
- fault TFT seharusnya tidak mematikan VESC USART1;
- fault TFT seharusnya tidak mematikan GNSS USART2;
- fault TFT seharusnya tidak menonaktifkan watchdog;
- fault TFT tidak boleh menghilangkan jalur safety.

Kesimpulan: jalur init saat boot dan jalur reinit saat runtime wajib dipisahkan.
Runtime reinit harus mengembalikan `false`, bukan memasuki `FatalError()`.

### P0-2 — display recovery mengandung blocking wait panjang tanpa realtime service

ILI9341 init memakai `HAL_Delay(5 + 20 + 150 + 120 + 20 ms)`.
`initDisplay()` dapat mencoba dua kali dan menambah jeda 40 ms.
Selama delay tersebut interrupt UART tetap hidup, tetapi main-context safety/VESC parser tidak dijalankan.
PB12 safety saat ini adalah GPIO polling, bukan EXTI.
Rekomendasi P0:
- ubah recovery menjadi staged/non-blocking state machine;
- atau minimal ganti setiap delay recovery dengan realtime-aware delay kecil;
- panggil `pollSafetyIo()` dan `gVesc.poll()` di sela wait;
- jangan pernah menjalankan full hardware reset TFT sebagai operasi monolitik saat kendaraan aktif.

### P0-3 — `TFT:TEST` belum memiliki motion-safe gate

Command `TFT:TEST` hanya memeriksa UI busy, splash, dan display health.
Command lalu menjalankan:
- full red + `HAL_Delay(250)`;
- full green + `HAL_Delay(250)`;
- full blue + `HAL_Delay(250)`;
- full UI redraw.

Tidak ada syarat `STATE_STOPPED`, speed mendekati nol, nav inactive, atau maintenance mode.
Ini berbeda dari `BOOT:DFU:CONFIRM` yang sudah mempunyai motion-safe gate.

Tindakan wajib:
- tolak `TFT:TEST` kecuali vehicle STOPPED/STANDBY dan speed <= threshold;
- tolak saat navigation active;
- lebih aman hanya izinkan dalam maintenance/diagnostic mode;
- tetap service safety/VESC selama test pattern hold.

### P1-1 — UI state masih dapat berubah di tengah active draw
`writeColorTx()` dan `pushImageTx()` memanggil `Board_RealtimeService()` ketika SPI transaction masih aktif.
Realtime callback kemudian menjalankan `pollSerialGui(512)`.
Parser dapat menerima `GOTO:*`, ACK/ERR config, dan telemetry ketika renderer masih memakai global `gUi`/`gTelemetry`.

`gUiDrawActive` hanya mencegah recursive `drawUiNow()`; ia tidak mencegah `setMenu()` mengubah `gUi.menu`.
Akibat yang mungkin terjadi:
- frame campuran halaman lama dan baru;
- cache metric tidak lagi cocok dengan page yang sedang dicat;
- static/dynamic layer menjadi tidak sinkron;
- latency burst karena command UI masuk di tengah long draw.

Pola dari `nebula-monitor` dan `waveshare-home-dashboard` lebih tegas: satu owner display/UI.
Rekomendasi:
- realtime callback hanya drain/queue bytes + safety-critical transport;
- defer command UI/config/page ke outer main loop;
- atau snapshot `UiState` dan telemetry render sebelum frame dimulai;
- jalur emergency STOP tetap boleh diprioritaskan secara terpisah.

### P1-2 — dirty renderer masih terlalu coarse

Saat ini hanya ada `TOPBAR`, `CONTENT`, `FOOTER`, `ALL`.
`markUiForCommand()` menandai CONTENT untuk hampir setiap telemetry command yang dikenali.
Ia belum membandingkan nilai lama dan baru sebelum invalidation.
`Speeduino-Dash` memakai `PrevData`: widget hanya disentuh ketika nilai benar-benar berubah.
LVGL memakai invalid area yang lebih granular daripada mask F4 saat ini.

Rekomendasi dirty map F4 berikutnya:
```text
TOPBAR_TITLE / TOPBAR_ESC / TOPBAR_PER / TOPBAR_NAV / TOPBAR_SYS / ESTOP
SUMMARY_LINE0 / SUMMARY_LINE1 / HEALTH_RAIL
ROW0 / ROW1 / ROW2 / ROW3
OVERVIEW_CARD0 / CARD1 / CARD2
FOOTER_LEFT / FOOTER_CENTER / FOOTER_RIGHT
FULL_STATIC
```
Parser perlu mengubah bit hanya jika nilai visual yang relevan berubah.
Telemetry yang tidak dipakai halaman aktif tidak perlu memicu paint.

### P1-3 — full page redraw masih jauh dari target

Pengukuran hardware 2026-09-10, hanya memakai `GOTO:*` dan `TFT:DIAG`:

| Page | Bytes frame | UI time |
|---|---:|---:|
| OVERVIEW | 357,665 B | 534 ms |
| ESC root | 318,697 B | 466 ms |
| PERCEPTION root | 317,764 B | 464 ms |
| NAVIGATION root | 316,858 B | 461 ms |
| SYSTEM root | 318,341 B | 463 ms |

SPI fault counter pada test: timeout=0, HAL error=0, recovery=0, conflict=0.
Raw framebuffer RGB565 hanya 153,600 B.
Artinya full frame F4 mengirim sekitar 2.06–2.33 kali isi layar aktual.

Penyebab utama:
1. `drawUiFrame(full)` melakukan `fillScreen(C_BG)` 153.6 KB;
2. card/panel kemudian menimpa kembali area besar;
3. text tile juga mengirim background pada bounding box teks;
4. rounded geometry/icon masih menghasilkan banyak rectangle kecil;
5. full page menghasilkan ratusan transaksi SPI.

Pada clock write 6 MHz, 153.6 KB saja membutuhkan sekitar 205 ms wire-time ideal.
Maka target full-page <150 ms **secara fisik tidak mungkin** tercapai di 6 MHz walaupun CPU overhead nol.

Urutan yang benar:
- hilangkan redundant full-screen background pass;
- kurangi primitive transaction count;
- validasi write clock `/8` = 12 MHz;
- baru pertimbangkan `/4` = 24 MHz setelah stress/readback;
- DMA hanya mengurangi CPU occupancy, bukan wire-time.

### P1-4 — service-gap metric saat ini tercemar startup

`Board_MaxServiceGapMs()` adalah lifetime maximum sejak boot dan tidak mempunyai reset/window API.
Nilai hardware saat audit adalah 212 ms.
Nilai ini sangat mungkin berasal dari init/reset/sleep-out TFT, bukan steady-state runtime.
Rekomendasi observability:
- reset steady-state max setelah display/USB/sensors ready;
- simpan `serviceGapLast`, rolling max 1 s/10 s, dan lifetime max;
- idealnya histogram/p95/p99 untuk stress test;
- bedakan startup gap, display gap, dan motor service gap.

### P1-5 — visual regression suite sudah tidak mewakili firmware sekarang

`test/render_hmi_revision.py` menggambar mock PIL lama: HOME/CAMERA/GPS/ACTUATOR.
Preview terakhir bertanggal 2026-09-08.
`README_HMI_REVISION.md` bahkan masih menjelaskan **one-tap latched drive**.
Firmware aktual sudah memakai hold-to-run + release-to-STOP.

Risikonya bukan kosmetik: dokumentasi/test lama dapat membuat operator/developer memahami safety semantics secara salah.

Rekomendasi dari pola `Lvgl-mcp-esp32`:
- buat host renderer dari layout/spec yang sama dengan firmware;
- generate screenshot setiap page 320x240;
- export geometry tree/manifest;
- assert tidak overlap/out-of-bounds;
- simpan golden screenshot untuk perubahan terkontrol;
- masukkan dead-man visual state dan SYSTEM pages dalam regression.

### P1-6 — firmware yang berjalan belum punya provenance yang cukup
Working tree memiliki 1,445 insertions / 378 deletions terhadap baseline dan belum seluruhnya committed.
Hardware dapat menjalankan binary berbeda bila ada sesi lain yang rebuild/flash.
`TFT:STATUS` hanya memberi controller register, bukan identitas firmware.

Pola OTA/versioning pada `waveshare-home-dashboard` menunjukkan kebutuhan field-service ini.
Tambahkan read-only `FW:INFO`, minimal berisi:
- git SHA/build ID;
- dirty-build flag;
- build UTC/local timestamp;
- firmware image CRC32/SHA prefix;
- manifest generation;
- config/protocol schema version;
- app base dan image size.

Saat audit, artifact lokal SHA256:
`5ca3a0cd721e2ecab9f74f8d96c6065e848aff70456b3d74686d4fb23ac9fe2d`.
Nilai ini adalah artifact snapshot, bukan klaim bahwa binary tersebut pasti identik dengan binary hardware setelah sesi lain berjalan.

## 4. Audit repository #1 — TFT_eSPI

Relevance: **sangat tinggi**.
Ini reference paling berguna untuk low-level ILI9341/SPI pada F4gateway.

Yang terbukti dari source lokal:
- `begin_tft_write()` menahan CS selama logical transaction;
- `end_tft_write()` menunggu hardware busy lalu release bus;
- `setWindow` dirancang untuk digunakan di dalam transaction yang sama;
- read/write/touch mempunyai lifecycle/frequency berbeda;
- STM32 SPI1 TX DMA dipetakan ke DMA2 Stream3 Channel3 / Channel 3;
- tersedia `initDMA`, `pushPixelsDMA`, `dmaBusy`;
- setup ILI9341 STM32 menggunakan clock jauh di atas 6 MHz pada banyak board.

Yang sudah berhasil diadopsi F4:
- explicit `SpiOwner`;
- CS exclusion TFT/touch;
- satu transaction untuk set-window + pixel payload;
- profile clock TFT vs touch;
- bounded HAL timeout;
- RX/OVR drain;
- counters transaction/error/recovery.

Yang masih kurang:
- runtime reinit harus nonfatal;
- clock profile belum adaptive/validated;
- belum ada DMA bulk path;
- belum ada pixel readback validation untuk fast-clock self-test;
- geometry kecil masih membuka transaction berulang.

Yang **tidak boleh dicopy mentah**: beberapa busy wait library bersifat blocking tanpa deadline.
F4 harus mempertahankan bounded wait karena motor link lebih penting daripada display throughput.
## 5. Audit repository #2 — LVGL

Relevance: **tinggi sebagai arsitektur, rendah sebagai dependency runtime penuh**.

Source LVGL menunjukkan model:
- object invalidation;
- invalid-area collection;
- PARTIAL render mode;
- flush callback per area;
- explicit `flush_ready` lifecycle;
- buffer ownership yang jelas.

F4 sudah mengadopsi ide dasar partial render tetapi belum granular.
Mask tiga region belum setara invalid-area engine.

Yang sebaiknya diambil berikutnya:
- region invalidation berbasis nilai yang berubah;
- coalescing region yang berdekatan;
- satu paint cycle untuk sekumpulan update;
- static layout tidak disentuh jika hanya numerical value berubah;
- page generation/epoch agar dirty bit page lama tidak diaplikasikan ke page baru.

Yang tidak disarankan:
- membawa LVGL runtime penuh ke F411 sekarang;
- full-screen/double framebuffer;
- object tree dinamis yang memperbesar RAM dan complexity.

Native renderer F4 masih pilihan tepat bila invalidation-nya diperkuat.
## 6. Audit repository #3 — nebula-monitor

Relevance: **tinggi untuk ownership/deferred refresh**.

Temuan source:
- display dan touch dimiliki satu `DisplayManager`;
- producer hanya set `pending_refresh`;
- display manager melakukan refresh dari satu context;
- flush path memakai `startWrite -> setAddrWindow -> pushColors -> endWrite`;
- XPT2046 menggunakan pola filtering dan bus discipline.

F4 sudah lebih baik daripada nebula pada incremental leaf metrics karena tidak selalu full redraw.
Namun F4 kalah pada discipline state mutation karena realtime callback masih dapat menjalankan UI parser di tengah draw.

Yang harus diambil:
- producer tidak menggambar dan tidak mengganti page langsung;
- queue/defer UI commands;
- satu owner untuk commit display;
- explicit mode ketika display unavailable.

Yang tidak perlu diambil:
- full `drawMainScreen()` setiap `pending_refresh`;
- ESP32 task model/heap-heavy String patterns.

Kesimpulan: gunakan nebula sebagai reference **ownership**, bukan reference pixel efficiency.
## 7. Audit repository #4 — Speeduino-Dash

Relevance: **sangat tinggi untuk automotive realtime HMI**.

Source `esp_dash_v1.cpp` menggunakan `PrevData` dengan sentinel initial values.
`update_dash_values()` hanya mengubah widget ketika ECU field benar-benar berubah.
Ada pemisahan cadence:
- ECU polling 100 ms;
- UI update 60 ms;
- status update 250 ms;
- stale link 700 ms.

Ada `lvReady` guard, portal/load-shedding mode, stale link visualization, warning threshold, dan separate VSPI untuk SD.

Yang sudah ada di F4:
- domain freshness;
- display ready/fault state;
- bounded display refresh cadence;
- warning/fault colors;
- maintenance mode yang menurunkan workload best-effort.

Gap terpenting:
- F4 belum memakai per-field previous-value suppression di parser;
- status visual belum mempunyai configurable hysteresis untuk warning yang berfluktuasi;
- UI workload belum memiliki explicit “degraded paint rate” saat transport pressure tinggi.

Rekomendasi: adopsi `PrevData` semantics di layer model-to-view, bukan meng-copy LVGL.
## 8. Audit repository #5 — waveshare-home-dashboard

Relevance: **tinggi untuk concurrency discipline dan field service**.

Repo ini memakai LVGL task terpisah dan mewajibkan `lvgl_port_lock()` sebelum UI mutation.
Server juga tidak publish payload yang identik dengan nilai sebelumnya dan memakai Last Will untuk stale-source semantics.
Firmware/OTA mempunyai identitas versi yang eksplisit.

Yang dapat diterapkan ke F4 tanpa FreeRTOS:
- `UI_MUTATION_ALLOWED` hanya pada outer main-loop owner;
- serial ingress masuk queue/model, bukan langsung page mutation;
- unchanged telemetry tidak menghasilkan dirty bit;
- build/version identity tersedia dari HMI diagnostics;
- source disconnect dibedakan dari data valid terakhir.

Yang tidak dapat ditiru:
- dua full-height display buffer di PSRAM;
- dual-core separation;
- blocking mutex `portMAX_DELAY`.

F4 single-core bare-metal justru perlu invariant yang lebih sederhana:
`ISR -> ring -> parser/model -> dirty flags -> one renderer -> SPI owner`.
Tidak boleh ada jalur balik parser -> renderer ketika renderer sedang aktif.
## 9. Audit repository #6 — papers3-dashboard

Relevance: **menengah-tinggi untuk refresh lifecycle**.

`epd_driver.cpp` memisahkan dua hal:
- LVGL/render menghasilkan dirty framebuffer;
- physical display commit terjadi kemudian dan rate-limited.

Ini adalah konsep penting walaupun panelnya e-paper.
F4 sudah melakukan coalescing melalui `DISPLAY_REFRESH_MS`, tetapi masih mencampur model update dan beberapa immediate `drawUiNow()` dari action/config path.

Yang dapat diterapkan:
- semua non-emergency model mutation hanya menandai dirty;
- renderer memilih kapan commit;
- page transition adalah explicit immediate/full commit;
- telemetry burst digabung menjadi satu paint;
- diagnostic page boleh memakai cadence lebih lambat daripada drive values.

Yang tidak diterapkan:
- full-screen framebuffer PSRAM;
- 1 second physical refresh interval;
- e-paper quality/ghosting policy.

Target F4: event-driven static layer + bounded dynamic commit tanpa kehilangan respons touch/safety.
## 10. Audit repository #7 — ESP32-4848S040-EspHome-LVGL

Relevance: **menengah untuk touch/page interaction**.

Repo mempunyai:
- explicit swipe lock;
- gesture start/update/end lifecycle;
- cancel ketika koordinat hilang atau state berubah;
- settings idle timer dan auto-return;
- per-widget update actions;
- 25% LVGL buffer.

F4 sudah mempunyai implementasi safety yang lebih sesuai AGV:
- clean release commit untuk control normal;
- slide-out cancel;
- hold 650 ms untuk motion;
- release motion = STOP;
- STOP langsung aktif pada initial press.

Yang masih bisa dipakai:
- interaction epoch: event touch lama tidak boleh berlaku setelah page berganti;
- explicit cancel jika display recovery/page change terjadi ketika finger masih down;
- optional auto-return dari SYSTEM/diagnostic page ke OVERVIEW setelah idle.

25% frame buffer tidak realistis untuk F411 dan tidak disarankan.
## 11. Audit repository #8 — ESP32-8048S070c Home Assistant Dashboard

Relevance: **menengah-rendah untuk low-level, menengah untuk visual hierarchy**.

Repo terutama YAML/LVGL untuk layar besar, dengan:
- display cadence 100 ms;
- touch cadence 50 ms;
- angka utama menggunakan font besar;
- unit/label dipisahkan dari value;
- static background dan consistent theme.

Yang sudah sejalan dengan F4:
- cadence touch/display dipisahkan;
- value kanan pada metric rows jelas;
- status READY/WARNING/FAULT punya hierarchy;
- static label tidak perlu berubah bersama telemetry.

Yang tidak cocok:
- background image besar;
- font TTF/runtime asset pipeline;
- layout 7-inch yang terlalu kaya untuk 320x240;
- PSRAM assumptions.

Nilai repo ini terutama sebagai reference readability, bukan SPI/interrupt master.
## 12. Audit repository #9 — ESP32-Serial-OBD2-Gauge-Catalyst

Relevance: **visual concept saja pada clone yang tersedia**.

README menyebut:
- automotive real-time gauge;
- ILI9341 + XPT2046;
- LVGL + LovyanGFX;
- multi-page gauge/config/diagnostic;
- warning dan CPU temperature monitoring.

Namun clone lokal saat audit tidak mempunyai source runtime C/C++.
Karena itu tidak mungkin memverifikasi:
- SPI transaction handling;
- touch sampling;
- interrupt priority;
- redraw strategy;
- fault recovery;
- memory usage.

Yang boleh diambil hanyalah hierarchy automotive: critical number, warning, status, dan page grouping.
Jangan mengutip repo ini sebagai bukti bahwa teknik low-level tertentu sudah production-proven.

Rekomendasi riset lanjutan: ganti dengan repo open-source ILI9341/XPT2046 automotive HMI yang source driver/runtime-nya lengkap.
## 13. Audit repository #10 — Lvgl-mcp-esp32

Relevance: **tinggi untuk verification tooling**.

Repo menyediakan headless simulator yang menghasilkan:
- screenshot PNG;
- JSON widget tree;
- position/size/style setiap object;
- configurable resolution termasuk 320x240;
- repeatable render dalam CI.

F4 tidak perlu memakai LVGL simulator secara runtime.
Tetapi prinsip test-nya sangat layak dipindahkan.

Gap F4 saat ini:
- PIL preview tidak menggunakan renderer firmware aktual;
- tidak ada geometry manifest current menu tree;
- tidak ada automated screenshot diff untuk SYSTEM/new menu;
- tidak ada assert pixel/layout terhadap actual current safety semantics.

Target test baru:
`fixture telemetry -> host-native HMI renderer -> RGB565/PNG -> geometry manifest -> asserts`.
Golden image hanya di-update jika perubahan visual memang disetujui.
## 14. F4 SPI1 — apa yang sekarang sudah benar

Current architecture sudah jauh lebih robust daripada baseline:
- hanya `HmiDisplay` yang melakukan transfer SPI runtime;
- TFT dan touch mempunyai exclusive `SpiOwner`;
- kedua CS dinaikkan sebelum owner baru dipilih;
- transaction conflict dihitung;
- BSY wait mempunyai DWT timeout;
- HAL SPI transfer mempunyai timeout 8 ms;
- RXNE/OVR dibersihkan saat transition/recovery;
- CASET/PASET/RAMWR dan payload berada dalam logical transaction yang sama;
- long color/image transfer dipecah per 256 B;
- `Board_RealtimeService()` dipanggil antarchunk;
- touch tidak dapat menyela RAMWR transaction;
- TFT read memakai owner berbeda dari write;
- register ID/status dapat dibaca untuk health check.

Normal hardware test tidak menunjukkan SPI timeout/error/conflict.
Ini membuktikan driver transaction baru stabil pada clock konservatif saat ini.

Yang belum terbukti adalah fault-injection recovery, bukan normal-path transfer.
Normal-path PASS tidak boleh dianggap sebagai bukti bahwa recovery failure aman.
## 15. Interrupt dan realtime hierarchy

Current priority:

| IRQ | Priority | Assessment |
|---|---:|---|
| USART1 VESC/F103 | 0 | benar, highest realtime |
| USART2 GNSS | 1 | benar |
| USB OTG FS | 2 | benar setelah audit sebelumnya |
| TIM11 watchdog | 3 | benar sebagai lower critical IRQ |
| SPI TFT | polling/main | benar; tidak perlu SPI IRQ normal |

Renderer berjalan main-context sehingga USART1 dapat preempt transfer TFT.
Ini adalah desain yang tepat.

Tetapi “IRQ priority benar” tidak otomatis berarti system latency benar.
Main-context safety PB12, parser, VESC processing, dan deferred UART recovery masih bergantung pada service cadence.
Karena itu target penting bukan hanya IRQ priority, tetapi **maximum time tanpa `Board_RealtimeService()`**.

Jika DMA ditambahkan nanti:
- DMA2 Stream3 Channel3 untuk SPI1 TX adalah kandidat dari TFT_eSPI STM32 mapping;
- IRQ DMA harus priority 4/5, di bawah UART/USB/watchdog;
- ISR DMA hanya clear flag/update completion state;
- jangan parse UI atau touch dari DMA ISR;
- DMA wait wajib timeout, tidak `while(dmaBusy())` tanpa batas.
## 16. Touch XPT2046

Perubahan sebelumnya sudah tepat:
- fast pressure reject sekali saat tidak disentuh;
- hanya pressed state mengambil 3 sample koordinat;
- median-of-three;
- reject jika spread X/Y terlalu besar;
- second pressure validation;
- satu TOUCH owner transaction;
- touch read ditolak jika SPI bukan IDLE.

Hasil runtime no-touch menunjukkan fast reject bekerja hampir 1:1 terhadap polling read.
Ini jauh lebih murah daripada validasi lima-pass lama.

Current safety interaction juga kuat:
- normal tap commit pada release;
- sliding keluar membatalkan action;
- motion memerlukan hold 650 ms;
- release/slide-out setelah hold menghasilkan STOP;
- STOP sendiri immediate pada PRESS.

Gap yang masih perlu ditest secara hardware:
- noisy press di tepi tombol;
- finger down saat page change;
- finger down saat display recovery;
- repeated touch sesaat setelah full page draw;
- pressure threshold pada temperatur/supply berbeda.

Tanpa kabel T_IRQ/PENIRQ, polling tetap solusi benar. Jangan membuat synthetic EXTI.
## 17. Text renderer dan geometry pipeline

Text adalah area yang sudah membaik signifikan.
`drawString()` memakai fixed tile `320 x 8 x RGB565 = 5,120 B`.
Satu strip dirender di RAM lalu `pushImage()` ke TFT.
Ini menghilangkan pola lama satu SPI transaction per glyph pixel.

Namun geometry non-text masih mahal:
- `drawLine()` dapat memanggil `fillRect(1x1)` per pixel;
- `drawCircle()` melakukan banyak 1x1 transaction;
- `drawRoundRect()` corner juga banyak 1x1 operation;
- icons memanggil kombinasi line/circle/fill kecil.

Pada page transition, ratusan logical SPI transaction masih terjadi.

Optimasi berikutnya yang aman:
1. gabungkan contiguous pixel menjadi horizontal spans;
2. buat `drawCircle/roundRect` span-based;
3. rasterize icon kecil ke compact 1-bit/RGB565 mask bila lebih hemat;
4. pertahankan procedural color selection tetapi commit sebagai tile;
5. jangan membuat full-screen bitmap.

Tujuannya menurunkan transaction count tanpa menambah heap/dynamic allocation.
## 18. SRAM dan flash budget

Build snapshot yang dianalisis memiliki kira-kira:
- RAM reported: 38,984 / 131,072 B = 29.7%;
- Flash app: sekitar 138 KB / 360,448 B = 38.3%;
- `.text` 89,764 B;
- `.rodata` 40,440 B;
- `.data` 8,264 B;
- `.bss` 30,748 B.

Largest RAM objects:
- `gUsb` ~12.4 KB;
- `gVescUart` ~6.2 KB;
- `gGnssUart` ~6.2 KB;
- `tft` ~5.2 KB;
- `gVesc` ~1.66 KB;
- VESC line buffer ~1.55 KB.

Implikasi:
- 153.6 KB full framebuffer tidak mungkin;
- 5.1 KB text tile masih rasional;
- static DMA buffer 1–4 KB masih realistis;
- tetapi stack high-water belum diukur.

Sebelum menambah DMA/buffer lain, tambahkan stack watermark atau runtime stack margin measurement.
## 19. Clock strategy ILI9341

Current F4 native write clock `/16` dari APB2 96 MHz ≈ 6 MHz.
Legacy project `lib/TFT_eSPI_Setup/User_Setup.h` pernah mendefinisikan write 10 MHz, read 6 MHz, touch 2.5 MHz.
TFT_eSPI reference STM32 ILI9341 memakai profile yang lebih cepat pada banyak board.

Jangan langsung mengubah ke 24/48 MHz tanpa pembuktian hardware.
Strategi robust:
1. boot/recovery selalu mulai safe 6 MHz;
2. tulis deterministic test patch pada 12 MHz (`/8`);
3. baca kembali patch pada safe read clock;
4. compare pixel/CRC;
5. jika lulus beberapa iterasi, aktifkan 12 MHz untuk TFT write;
6. bila gagal, fallback permanen untuk boot session ke 6 MHz;
7. 24 MHz (`/4`) hanya setelah extended stress.

Register-ID read saja tidak cukup untuk membuktikan write clock karena corruption dapat terjadi pada pixel payload.
Perlu implementasi `readPixel/readRect` ILI9341 yang mengikuti dummy-read + RGB conversion rule.

Touch tetap konservatif; jangan ikut dinaikkan hanya karena TFT write berhasil pada clock lebih tinggi.
## 20. DMA decision

DMA **belum menjadi P0/P1 pertama**.
Alasannya:
- full-page bottleneck utama masih redundant bytes + 6 MHz wire-time;
- incremental update sudah <50 ms pada pengukuran sebelumnya;
- DMA tidak mempercepat jumlah bit yang keluar dari SPI;
- DMA menambah ownership/failure-state complexity pada bus yang juga dipakai XPT2046.

DMA layak setelah P0 selesai dan byte volume full frame turun.
Desain yang disarankan:
- SPI1 TX: DMA2 Stream3 Channel3 sesuai reference TFT_eSPI STM32;
- static buffer 1–4 KB;
- DMA hanya untuk pixel payload besar;
- command/address tetap CPU polling;
- IRQ priority 4/5;
- timeout DWT/timer wajib;
- sebelum touch/read/prescaler change, DMA harus idle;
- timeout -> CS HIGH -> abort DMA -> recover SPI secara nonfatal;
- jangan menahan bus untuk satu full frame tanpa service/cancel boundary.

Keuntungan DMA yang dicari: CPU tersedia untuk parser/safety, bukan sekadar benchmark FPS.
## 21. Diagnostics yang sudah baik dan gap-nya

Sudah tersedia:
- SPI transaction/byte count;
- timeout/HAL error/recovery/bus conflict;
- touch read/fast reject;
- frame last/max bytes;
- UI last/max draw time;
- max service gap;
- VESC/GNSS UART error/overflow/drop;
- VESC frame/recovery;
- host parser errors;
- TFT controller ID/register;
- stream freshness.

Gap:
- service gap tidak windowed/resettable;
- tidak ada SPI clock profile aktif;
- tidak ada last SPI fault reason/state;
- tidak ada display recovery stage/attempt counter yang membedakan soft/hard failure;
- tidak ada stack high-water;
- tidak ada firmware build identity;
- tidak ada per-page frame bytes/time histogram;
- tidak ada dirty-region count/coalesced update count.

SYSTEM page sebaiknya fokus pada “actionable diagnostics”, bukan hanya counter mentah.
## 22. Test coverage audit

Test sekarang membuktikan banyak invariant melalui static/string checks:
- explicit SPI owner;
- DWT bounded wait;
- IRQ ordering;
- no SPI drawing in IRQ;
- dirty path tidak content-wide clear;
- metric cache;
- tile text renderer;
- touch fast reject/median;
- diagnostics wiring;
- degraded display path tersedia.

Tetapi test tersebut belum membuktikan failure behavior yang paling penting:
- `HAL_SPI_Init` gagal pada runtime recovery;
- `HAL_SPI_Transmit` timeout di tengah rectangle;
- recovery gagal dua kali;
- display fault terjadi saat kendaraan bergerak;
- PB12 ditekan saat display recovery delay;
- `GOTO` diterima ketika active draw;
- touch tetap down ketika page epoch berubah;
- identical telemetry tidak menimbulkan physical paint;
- visual output current menu tree sesuai golden image.

Karena itu PASS self-check saat ini adalah **necessary but not sufficient** untuk klaim robustness penuh.
## 23. Prioritized remediation backlog

### P0 — harus ditutup sebelum display disebut fail-safe

**P0.1 Runtime SPI reinit nonfatal**  
Pisahkan `Spi1_InitBootOrFatal()` dan `Spi1_ReinitRuntime()`.
Runtime function tidak boleh disable IRQ/infinite loop.

**P0.2 Nonblocking/realtime-aware display recovery**  
Reset/wake sequence menjadi state machine; safety/VESC terus diservis selama wait.

**P0.3 Lock heavy display diagnostic**  
`TFT:TEST` hanya boleh saat STOPPED/STANDBY, speed aman, nav inactive/maintenance.
Ganti 250 ms raw delays dengan realtime-aware wait.

### P1 — performance dan deterministic UI

**P1.1 Defer UI mutation saat renderer aktif.**  
Parser realtime hanya queue; page/config action dijalankan outer loop.

**P1.2 Per-field dirty invalidation.**  
Adopsi `PrevData` + page-region mask; identical data = zero paint.

**P1.3 Full-frame byte reduction.**  
Hilangkan unconditional `fillScreen()` jika layout akan menutupi area; compose static regions sekali.
**P1.4 Validated fast TFT write clock.**  
Mulai 12 MHz dengan pixel readback/fallback; 24 MHz hanya sesudah stress.

**P1.5 Geometry batching.**  
Circle/roundrect/icon menggunakan horizontal spans/tile, bukan 1x1 transaction.

**P1.6 Service latency telemetry yang valid.**  
Pisahkan startup/lifetime/rolling maximum dan p95/p99 bila memungkinkan.

**P1.7 Firmware provenance.**  
Expose build SHA, dirty flag, CRC/manifest generation, schema version.

**P1.8 Current visual regression.**  
Hapus/update mock one-tap lama; test current dead-man pages + SYSTEM.

**P1.9 Page/touch generation.**  
Reset/cancel armed touch ketika page atau display generation berubah.

### P2 — setelah P0/P1 stabil

**P2.1 Optional SPI TX DMA** dengan priority 4/5 dan bounded abort/recovery.  
**P2.2 Stack high-water monitoring** sebelum memperbesar DMA/tile buffers.  
**P2.3 Optional diagnostic auto-return** ke OVERVIEW setelah idle.  
**P2.4 Optional 24 MHz profile** hanya jika long stress + pixel verification lulus.
## 24. Acceptance criteria revisi

### Safety/recovery
- display/SPI failure tidak pernah memanggil path yang disable global IRQ pada runtime;
- PB12 safety tetap diservis selama display recovery/self-test;
- `TFT:TEST` ditolak ketika motion/nav tidak safe;
- VESC USART1 priority tetap 0;
- UART1 overflow/drop tetap 0 pada display stress;
- display fault tidak mereset atau menghentikan motor communication stack.

### SPI
- CS TFT dan TOUCH tidak pernah LOW bersamaan;
- transaction conflict = 0 normal operation;
- timeout/HAL error = 0 normal operation;
- injected error pulih atau masuk degraded mode tanpa system hang;
- touch/read tidak dimulai saat TFT/DMA owner masih aktif.

### UI performance
- identical telemetry menghasilkan **0 physical paint bytes**;
- incremental frame: <25 ms ideal, <50 ms hard target;
- page transition: <150 ms target;
- full page bytes: <220 KB tahap pertama, <180 KB stretch target;
- no content-wide clear pada telemetry-only update;
- static labels/icons tidak dicat ulang untuk perubahan value saja.

### Interaction
- hold-to-run tetap 650 ms policy atau config yang eksplisit;
- release/slide-out selalu STOP untuk motion;
- page change membatalkan armed touch generation lama;
- display recovery tidak meninggalkan latched touch/motion UI state.
### Observability/test
- SYSTEM/FW command menampilkan build identity yang bisa dicocokkan dengan source;
- service-gap steady-state dipisahkan dari startup;
- fault injection test mencakup SPI init/transfer/recovery failure;
- screenshot/geometry regression menggunakan current menu tree;
- dokumentasi tidak lagi menyebut one-tap latched motion.

## 25. Decision matrix: apa yang paling bernilai diterapkan

| Teknik | Sumber utama | Status F4 | Priority |
|---|---|---|---|
| SPI owner + batched transaction | TFT_eSPI | sudah diterapkan | KEEP |
| bounded SPI recovery | custom + TFT_eSPI concept | sebagian; fatal gap | P0 |
| single UI/display owner | nebula/waveshare | belum penuh | P1 |
| per-field change detection | Speeduino | belum | P1 |
| granular invalidation | LVGL | sebagian | P1 |
| physical commit rate policy | papers3 | sebagian | P1 |
| touch cancel/interaction epoch | ESPHome LVGL | sebagian | P1 |
| current screenshot regression | Lvgl-mcp | belum | P1 |
| validated faster SPI write | TFT_eSPI evidence | belum | P1 |
| SPI TX DMA | TFT_eSPI STM32 | belum | P2 |
| full framebuffer/double buffer | LVGL/Waveshare | tidak cocok | REJECT |
| visual hierarchy big-value/unit | HA/OBD concept | sebagian | KEEP |

## 26. Kesimpulan akhir re-audit

F4gateway sekarang **jauh lebih baik** daripada baseline dalam transaction ownership, touch safety, dirty leaf rendering, diagnostics, dan IRQ hierarchy.
Namun audit mendalam ini menemukan bahwa klaim “display fault isolated dari motion system” **belum sepenuhnya benar** karena runtime SPI reinit masih dapat masuk `FatalError()` dan mematikan global IRQ.
Display recovery dan `TFT:TEST` juga masih memiliki blocking wait panjang yang tidak menjamin main-context safety polling berjalan.
Dua hal ini lebih penting untuk diperbaiki daripada menambahkan DMA atau efek visual baru.

Dari sisi performance, normal incremental update sudah berada pada kelas yang jauh lebih baik, tetapi full page masih 461–534 ms karena mengirim 316–358 KB pada SPI 6 MHz.
DMA sendiri tidak akan menyelesaikan wire-time tersebut.
Kombinasi yang tepat adalah:
1. P0 recovery isolation;
2. true single UI owner;
3. previous-value + granular invalidation;
4. mengurangi redundant full-page bytes;
5. validated 12 MHz write profile;
6. geometry batching;
7. baru evaluasi DMA.

Dari 10 repo, lima reference dengan kontribusi teknis terkuat untuk F4 adalah:
1. TFT_eSPI — SPI lifecycle, ILI9341, DMA mapping;
2. LVGL — invalidation/partial rendering;
3. Speeduino-Dash — per-value realtime automotive update;
4. nebula-monitor — single display owner/deferred refresh;
5. Lvgl-mcp-esp32 — repeatable visual/geometry verification.

`waveshare-home-dashboard` dan `papers3-dashboard` sangat berguna untuk concurrency/commit lifecycle.
Dua ESPHome dashboard terutama berguna untuk interaction/visual hierarchy.
OBD2 Gauge Catalyst pada clone sekarang harus dianggap reference visual saja karena source runtime tidak tersedia.

Status audit: **COMPLETE — remediation belum dianggap selesai sampai seluruh P0 dan acceptance test terkait ditutup dengan fault injection + hardware test.**
