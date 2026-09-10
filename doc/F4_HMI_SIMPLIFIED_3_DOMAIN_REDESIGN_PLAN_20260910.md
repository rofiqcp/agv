# F4 HMI Simplified 3-Domain Redesign Plan

Tanggal: 2026-09-10
Target: STM32F411 + ILI9341 320x240 + XPT2046
Sumber data: `/home/sirobo/agv/src`, `/home/sirobo/agv/F4gateway`, `/home/sirobo/agv/hoverboard-vesc`

## 1. Tujuan

Redesain HMI menjadi sederhana untuk operator, tetapi tetap kuat untuk commissioning dan diagnostics.
Menu utama hanya berisi HOME, ESC, PERSEPSI, dan NAVIGASI.
SYSTEM tidak tampil sebagai menu utama; health penting tetap terlihat sebagai indikator global.
Diagnostics teknis dipertahankan melalui Service Mode agar tidak ada fungsi yang hilang.

Prinsip utama:
- satu layar = satu tujuan;
- maksimal tiga kartu besar per halaman;
- BACK / LEFT / RIGHT selalu pada posisi yang sama;
- icon + label wajib pada setiap kartu;
- data live hanya yang membantu keputusan operator;
- parameter berbahaya tidak diedit langsung dari layar umum;
- test motion menggunakan hold-to-run + release-to-stop;
- stale/offline/fault harus terlihat jelas.

## 2. Hierarki layar

```text
SPLASH
  -> HOME
      -> MAIN MENU
          -> ESC
          -> PERCEPTION
          -> NAVIGATION
```
## 3. HOME

HOME tidak menjadi menu; HOME adalah ringkasan kendaraan.
Hanya tiga data utama ditampilkan besar:
1. SPEED — `speedKmh` / `driveActualMps`.
2. VEHICLE STATE — READY / STOPPED / RUNNING / FAULT, dengan E-STOP override merah.
3. POWER / VBUS — tegangan supply/VESC bus dan warning undervoltage/overvoltage.

Tambahan kecil yang tidak dianggap kartu data:
- chip kecil AUTO/MANUAL + active source;
- indikator ESC, PER, NAV berupa tiga dot kecil READY/WARN/FAULT;
- indikator link F4↔ROS dan F4↔F103 kecil;
- jam/data age tidak perlu memenuhi layar.

Interaksi HOME:
- hanya satu tombol besar `MENU` di bawah;
- tidak ada carousel;
- tidak ada tombol setting;
- E-STOP selalu menjadi banner prioritas tertinggi.

Acceptance HOME:
- dapat dibaca <1 detik;
- tidak lebih dari tiga angka/status besar;
- operator langsung tahu: bergerak seberapa cepat, kendaraan dalam state apa, dan power supply sehat atau tidak.

## 4. MAIN MENU

Header: tombol `HOME` di kiri atas.
Isi: tiga kartu besar penuh lebar efektif:
- ESC — steering wheel + bolt;
- PERSEPSI — camera/eye;
- NAVIGASI — route/map pin.

Tidak ada SYSTEM card pada menu utama.
## 5. Layout standar domain

Setiap root ESC / PERSEPSI / NAVIGASI memakai layout identik.

Top navigation bar sekitar 34 px:
- kiri: `BACK`;
- tengah: `LEFT`;
- kanan: `RIGHT`;
- judul domain kecil berada di antara/di bawah kontrol tanpa menambah tombol.

Area kartu dari sekitar y=48 sampai y=218:
- tiga kartu submenu berdampingan;
- setiap kartu sekitar 98x158 px efektif;
- icon besar di bagian atas;
- label 1–2 baris di bawah;
- status dot kecil pada pojok kartu: green/yellow/red/gray;
- page indicator kecil `1/3`, `2/3`, dst.

LEFT/RIGHT menggeser satu kelompok berisi tiga submenu, bukan satu card.
Wrap-around boleh diaktifkan: page terakhir RIGHT kembali ke page pertama.
BACK kembali satu level dan tidak mengubah state kendaraan.

## 6. Detail page standard

Ketika kartu dipilih, masuk ke detail page.
Detail page tetap memakai kontrol atas `BACK / LEFT / RIGHT`.
LEFT/RIGHT di detail berganti view data dalam fungsi yang sama.
Maksimal tiga metric primer besar per view.
Bar status tipis menampilkan freshness, source, dan fault.

Action/test hanya muncul bila memang tersedia dan aman.
Read-only menjadi default; edit/test harus eksplisit.
Parameter yang dapat menggerakkan aktuator menggunakan confirmation/gate dan dead-man.
## 7. ESC — struktur submenu

ESC dibuat 4 halaman, masing-masing tepat tiga kartu.

### ESC page 1/4 — operasi dasar
1. `ESC OVERVIEW`
2. `OPERATOR MODE`
3. `STEERING`

### ESC page 2/4 — aktuator dan FOC
4. `DRIVE`
5. `FOC / POWER`
6. `MOTOR TELEMETRY`

### ESC page 3/4 — sensor dan commissioning
7. `ENCODER`
8. `CALIBRATION`
9. `COMMUNICATION`

### ESC page 4/4 — robustness
10. `FAULT & SAFETY`
11. `PERFORMANCE`
12. `MANUAL TEST`

Semua kartu mempunyai icon berbeda dan status dot.
Tidak ada parameter tuning berbahaya langsung pada root card.
### 7.1 ESC OVERVIEW
View A — motion:
- vehicle speed;
- drive target vs actual;
- steering target vs actual.

View B — motor summary:
- right motor eRPM / mechanical RPM;
- VESC connected/fresh;
- current fault code.

View C — control readiness:
- ESC feedback valid;
- encoder ready;
- active command source: TELEOP / HMI / NAV2 / IDLE.

Tidak ada test pada overview.

### 7.2 OPERATOR MODE
Tampilan:
- current mode AUTO/MANUAL;
- active source;
- motion-ready gate;
- E-STOP / safety gate.

Action:
- ganti AUTO <-> MANUAL hanya ketika speed ~0 dan tidak E-STOP;
- perubahan membutuhkan ACK dari ROS bridge;
- timeout/failure harus mengembalikan nilai lama.
### 7.3 STEERING
View A — live:
- target angle deg;
- actual angle deg;
- tracking error deg.

View B — steering state:
- calibrated / homed / encoder synced;
- raw encoder / logical position;
- sensor/FOC mode.

View C — limits:
- physical left/right limit;
- operational limit;
- center/deadband state.

Safe test:
- selectable test angle 5–30 deg;
- HOLD LEFT / HOLD CENTER / HOLD RIGHT;
- release selalu STOP/HOLD;
- hanya MANUAL + zero speed + fresh encoder + no E-STOP.

### 7.4 DRIVE
View A — live:
- drive target m/s;
- drive actual m/s;
- eRPM / mechanical RPM.

View B — calibration:
- drive scale;
- eRPM per m/s;
- odometry calibration scale.
View C — drive safety:
- max commanded speed;
- watchdog state;
- perception/autonomy gate.

Safe test:
- manual speed selector 10–50%;
- HOLD FORWARD / HOLD REVERSE;
- release/slide-out -> current/RPM command zero;
- hard timeout tetap aktif di F4 dan F103.

### 7.5 FOC / POWER
View A — power:
- Vbus;
- motor current / input current;
- duty cycle.

View B — FOC current:
- Iq;
- Id;
- current limit.

View C — thermal/fault:
- MOS temperature;
- motor temperature bila tersedia;
- active VESC fault.

Data diambil dari VESC `COMM_GET_VALUES(_SELECTIVE)`.
F4 bridge perlu diperluas karena telemetry F4 saat ini belum membawa semua field ini.
Tidak ada direct current/duty test pada operator page.
### 7.6 MOTOR TELEMETRY
View A — LEFT steering motor:
- RPM/eRPM;
- position;
- duty/current.

View B — RIGHT drive motor:
- RPM/eRPM;
- Vbus/current;
- duty/fault.

View C — comparison:
- left/right fault state;
- link freshness;
- telemetry age.

### 7.7 ENCODER
View A — steering encoder:
- raw count;
- logical steering deg;
- synced/configured state.

View B — diagnostics:
- edge A / edge B count;
- invalid transition / error count;
- direction/inversion.

View C — rotor relationship:
- encoder mechanical angle;
- observer electrical angle;
- encoder-observer / PID error.

Read-only by default. Encoder detect/align hanya melalui commissioning gate.
### 7.8 CALIBRATION
View A — steering calibration summary:
- calibrated/homed;
- measured span / safe span;
- current center position.

View B — physical mapping:
- left physical reference;
- center reference;
- right physical reference.

View C — drive calibration:
- wheel radius;
- eRPM per m/s;
- odometry scale.

Action dibatasi:
- `HOME STEERING` dan `SET CENTER` hanya saat kendaraan idle;
- calibration wizard menggunakan langkah eksplisit dan confirmation;
- parameter raw FOC/ADC tidak diedit dari operator HMI.

### 7.9 COMMUNICATION
View A — F4↔F103:
- connected/fresh;
- UART RX/TX age;
- error/drop/recovery counters.

View B — ROS↔F4:
- HMI connected;
- command/telemetry age;
- deferred queue/drop state.

View C — maintenance ownership:
- RUNTIME / VESC TOOL / PYTHON owner;
- maintenance active;
- route transition/safe-stop state.
### 7.10 FAULT & SAFETY
View A — active faults:
- VESC fault code/text;
- E-STOP state;
- safety switch state.

View B — electrical limits:
- under/over-voltage status;
- over-current status;
- temperature limit status.

View C — command safety:
- command watchdog;
- active source freshness;
- autonomy/perception gate.

Action yang diizinkan hanya `CLEAR/ACK` bila fault memang recoverable.
Tidak ada bypass safety dari layar.

### 7.11 PERFORMANCE
View A — control timing:
- command loop rate;
- feedback age;
- max service latency.

View B — F103 realtime:
- ISR max/budget;
- GET_VALUES latency;
- watchdog/reset counters.

View C — F4 gateway:
- service p95/p99/max;
- stack headroom;
- SPI/UART error counters.

### 7.12 MANUAL TEST
Dedicated commissioning screen, bukan metric page biasa.
Controls: LEFT / FORWARD / STOP / RIGHT / REVERSE.
Speed dan steering-angle preset ditampilkan di layar.
Semua motion action harus hold-to-run; STOP immediate; release/timeout/freshness-loss menghasilkan STOP.
## 8. PERSEPSI — struktur submenu

PERSEPSI dibuat 3 halaman, masing-masing tiga kartu.

### PERSEPSI page 1/3 — sumber dan hasil utama
1. `PER OVERVIEW`
2. `CAMERA`
3. `DETECTION`

### PERSEPSI page 2/3 — pemahaman area kendaraan
4. `LANE`
5. `DRIVABLE AREA`
6. `OBSTACLE`

### PERSEPSI page 3/3 — kualitas dan commissioning
7. `PERFORMANCE`
8. `CALIBRATION`
9. `PER TEST`

F4 tidak perlu menampilkan video kamera penuh; bandwidth serial, RAM dan fungsi operator tidak membutuhkannya.
HMI menampilkan health/hasil perception yang actionable.

### 8.1 PER OVERVIEW
View A:
- camera connected/healthy;
- perception ready;
- inference ON/OFF.

View B:
- lane safety state;
- drivable clear/valid;
- obstacle detected + nearest distance.

View C:
- pipeline FPS;
- data freshness;
- control/advisory state.
### 8.2 CAMERA
View A — camera health:
- connected;
- healthy;
- current FPS.

View B — capture config summary:
- resolution;
- requested camera FPS;
- backend CPU/GPU / inference state.

View C — stream diagnostics:
- frame age;
- dropped/failed frame count bila tersedia;
- hotplug/recovery state.

Action aman:
- inference ON/OFF boleh melalui ACK;
- restart camera tidak ditempatkan sebagai tombol satu-tap; service confirmation diperlukan.

### 8.3 DETECTION
View A — nearest object:
- class;
- distance m;
- confidence.

View B — detection summary:
- number of objects;
- ROI-valid objects;
- nearest obstacle class/distance.

View C — tracking:
- active track count;
- missed/stale state;
- detection data age.

F4 bridge perlu ditambah field confidence/count bila belum tersedia.
### 8.4 LANE
View A — lane state:
- lane safety state;
- center offset;
- confidence.

View B — geometry:
- near/far lookahead;
- estimated road width;
- lane center correction.

View C — control:
- lane-control state;
- requested steering correction;
- command/data freshness.

### 8.5 DRIVABLE AREA
View A:
- valid / invalid;
- drivable clear / blocked;
- drivable fraction.

View B:
- nearest boundary/clearance;
- left/right usable corridor;
- far lookahead.

View C:
- lane constraint active;
- ROI/config status;
- source age.

Read-only untuk operator.

### 8.6 OBSTACLE
View A:
- nearest obstacle class;
- forward distance;
- lateral offset.

View B:
- blocked/clear state;
- number of obstacle points/tracks;
- ROI-valid state.
View C:
- confirm/release frame counters;
- obstacle data age;
- perception advisory output state.

### 8.7 PERFORMANCE
View A:
- pipeline FPS;
- inference latency ms;
- frame age.

View B:
- backend CPU/GPU;
- inference enabled;
- dropped/recovery count bila tersedia.

View C:
- publish/update rates;
- lane metric age;
- obstacle metric age.

### 8.8 CALIBRATION
View A — obstacle distance calibration:
- selected class/model;
- scale;
- bias.

View B — lane corridor calibration:
- camera height/pitch;
- left/right corridor offsets;
- safety margin.

View C — status:
- calibration source/version;
- saved/unsaved state;
- current ROI valid.

HMI operator hanya melihat summary. Parameter geometrik detail tetap di ROS Web karena lebih cocok untuk layar besar dan visual overlay.

### 8.9 PER TEST
Test yang aman:
- camera heartbeat test;
- inference ON/OFF test;
- lane freshness test;
- obstacle freshness test;
- publish/bridge ACK test.

Tidak boleh menggerakkan kendaraan dari PER TEST.
## 9. NAVIGASI — struktur submenu

NAVIGASI dibuat 4 halaman, masing-masing tiga kartu.

### NAVIGASI page 1/4 — state utama
1. `NAV OVERVIEW`
2. `LOCALIZATION`
3. `GNSS`

### NAVIGASI page 2/4 — estimator/sensor
4. `IMU / MAG`
5. `EKF`
6. `ODOMETRY`

### NAVIGASI page 3/4 — operasi autonomous
7. `MISSION`
8. `NAV2`
9. `SAFETY`

### NAVIGASI page 4/4 — planning dan diagnostic
10. `COSTMAP`
11. `PATH / CONTROL`
12. `NAV TEST`

### 9.1 NAV OVERVIEW
View A:
- localization READY/DEGRADED/FAULT;
- Nav2 READY/NOT READY;
- navigation status IDLE/NAVIGATING/ARRIVED/FAILED.

View B:
- active target;
- distance/pose progress bila tersedia;
- current vehicle speed.

View C:
- GNSS age/fix;
- IMU health;
- autonomy motion gate.
### 9.2 LOCALIZATION
View A:
- localization state;
- map pose freshness;
- heading deg.

View B:
- local EKF status;
- global EKF status;
- pose source priority.

View C:
- position covariance/accuracy summary bila di-bridge;
- yaw confidence summary;
- stale/fault reason.

### 9.3 GNSS
View A — fix:
- FIX type;
- satellite count;
- HACC m.

View B — position:
- latitude;
- longitude;
- heading.

View C — quality:
- HDOP;
- GNSS age;
- connected/state.

Tidak ada edit receiver port/baud pada operator HMI.

### 9.4 IMU / MAG
View A — IMU:
- connected;
- gyro Z;
- yaw/heading.

View B — magnetometer:
- connected;
- field/heading summary bila tersedia;
- sensor freshness.

View C — consistency:
- GNSS heading vs IMU heading;
- yaw-rate state;
- disagreement warning.
### 9.5 EKF
View A — EKF local:
- status;
- odom freshness;
- local pose/yaw summary.

View B — EKF global:
- status;
- map pose freshness;
- global position summary.

View C — fusion health:
- GNSS contribution valid;
- IMU contribution valid;
- innovation/covariance warning summary bila tersedia.

### 9.6 ODOMETRY
View A:
- linear speed;
- yaw rate;
- steering actual.

View B:
- odom X/Y;
- yaw;
- odom age.

View C:
- raw ESC speed vs filtered odometry speed;
- tracking error;
- drive calibration scale.

Read-only; calibration perubahan dilakukan dari ESC CALIBRATION atau ROS Web.

### 9.7 MISSION
View A — waypoint browser:
- selected waypoint;
- saved/not saved;
- name.

View B — active mission:
- active target;
- NAV status;
- goal freshness.

View C — current pose capture:
- current X/Y/yaw;
- GNSS lat/lon if valid;
- waypoint storage state.
Mission actions:
- `SAVE CURRENT` hanya saat localization fresh;
- `GO` membutuhkan confirmation dan readiness gate;
- `STOP NAV` selalu tersedia dan immediate;
- nama waypoint tidak diedit lewat keyboard HMI kecil; gunakan default/nama dari ROS Web.

### 9.8 NAV2
View A — stack readiness:
- Nav2 ready;
- lifecycle/stack state summary;
- active navigation state.

View B — planner/controller:
- planner active;
- controller active;
- smoother active.

View C — command flow:
- Nav2 cmd freshness;
- perception advisory freshness;
- ESC mux source.

Tidak ada parameter Nav2 kompleks yang diedit langsung dari F4 HMI.

### 9.9 SAFETY
View A:
- E-STOP;
- NEO3 safety switch;
- autonomy motion allowed.

View B:
- lane safety state;
- obstacle blocked state;
- perception advisory state.

View C:
- ESC watchdog;
- cmd_vel freshness;
- localization freshness.

Safety page tidak menyediakan bypass/override.
### 9.10 COSTMAP
View A:
- local costmap ready;
- obstacle layer freshness;
- footprint/robot radius status.

View B:
- obstacle point age/count summary;
- clearing point age/count summary;
- inflation/blocked summary.

View C:
- costmap stale/fault reason;
- source perception status;
- Nav2 consumption state.

### 9.11 PATH / CONTROL
View A — planner:
- SMAC planner state;
- current path valid;
- replan/failure count bila tersedia.

View B — controller:
- MPPI state;
- current linear/angular command;
- controller age.

View C — smoother/tracking:
- smoother state;
- target vs actual speed;
- steering/yaw tracking error.

### 9.12 NAV TEST
Read-only test sequence tanpa motion sebagai default:
- GNSS freshness;
- IMU freshness;
- EKF local/global ready;
- Nav2 lifecycle ready;
- costmap input fresh;
- planner/controller availability.

Optional motion test tidak digabung ke NAV TEST; motion tetap hanya melalui controlled MANUAL TEST ESC.
## 10. Service Mode — tidak tampil di MAIN MENU

Fungsi SYSTEM lama tidak dihapus.
Akses Service Mode hanya jika kendaraan STOPPED, misalnya long-press MENU/HOME 3 detik atau command service dari USB.

Service Mode memakai 10 kartu yang sudah disiapkan:
1. SYSTEM HEALTH
2. IO PIN MONITOR
3. DISPLAY PINS
4. LINK DIAGNOSTICS
5. ERROR COUNTERS
6. SPI BUS
7. UART STATUS
8. POWER STATUS
9. TOUCH PANEL
10. TFT TEST

TFT TEST, SPI verify/recovery, dan maintenance berat tetap memakai motion-safe gate.
Operator normal tidak melihat halaman ini sehingga main menu tetap hanya tiga domain.

## 11. Strategy asset/logo

Setiap kartu wajib berisi icon + label.
Jangan simpan tile 100x150 full RGB565 untuk setiap menu karena flash akan boros.
Gunakan icon-only 48–56 px dengan transparent/palette mask; label dirender native oleh HMI.

Target asset:
- 4 top assets: HOME/MENU, ESC, PERSEPSI, NAVIGASI;
- 12 ESC icons;
- 9 PERSEPSI icons;
- 12 NAVIGASI icons;
- 10 Service Mode icons.

Semua icon memakai palette navy/cyan/cream yang sama.
Selected card memakai cyan border; warning amber; fault red; disabled gray.
## 12. Data model baru di F4gateway

Jangan menambah ratusan global variable terpisah.
Pisahkan telemetry menjadi typed domain structs:
- `HomeTelemetry`;
- `EscTelemetry`;
- `PerceptionTelemetry`;
- `NavigationTelemetry`;
- `ServiceTelemetry`.

Setiap struct mempunyai:
- nilai aktual;
- validity bit;
- source timestamp/age;
- status READY/WARN/FAULT/STALE.

ROS bridge mengirim data yang sudah dinormalisasi; F4 tidak parsing JSON kompleks.
Serial update tetap incremental dan change-driven.

Tambahan ESC bridge yang dibutuhkan:
- Vbus, duty, motor/input current, Iq/Id;
- MOS/motor temperature;
- VESC fault code;
- left/right motor values;
- encoder raw/sync/homing/calibration state;
- ISR/GET_VALUES timing summary.

Tambahan perception bridge:
- object confidence/count;
- lane center offset/confidence;
- drivable fraction/clearance;
- nearest obstacle lateral offset;
- inference/pipeline latency and ages.

Tambahan navigation bridge:
- map/odom pose summary;
- covariance/accuracy summary;
- costmap/planner/controller state;
- path/controller freshness.
## 13. UI state machine

Gunakan state eksplisit, bukan menu-parent logic yang semakin bercabang:
- `HOME`;
- `MAIN_MENU`;
- `DOMAIN_MENU`;
- `DETAIL_VIEW`;
- `SERVICE_MENU`;
- `SERVICE_DETAIL`.

State menyimpan:
- active domain;
- domain page index;
- selected card;
- detail view index;
- edit/test state;
- touch generation id.

Descriptor data-driven per card:
- ID;
- label;
- icon asset;
- status provider;
- number of detail views;
- action capability flags.

Dengan descriptor ini LEFT/RIGHT dan pagination tidak memakai switch besar berulang.
Card yang belum mendapat data tidak dihapus; tampil `N/A` atau `STALE`.

## 14. Touch behavior

HOME: hanya MENU.
MAIN MENU: HOME + tiga domain cards.
DOMAIN MENU: BACK + LEFT + RIGHT + tiga cards.
DETAIL: BACK + LEFT + RIGHT untuk cycle data view.

Semua tap commit-on-release dan slide-cancel.
Motion test menggunakan long/hold action, tidak tap-latched.
Page change menaikkan touch generation agar release lama tidak mengeksekusi action baru.
## 15. Rendering dan refresh

Gunakan renderer native F4 yang sekarang sudah optimized; jangan migrasi ke LVGL penuh.
Dirty rendering dibagi per region:
- top controls;
- card 0;
- card 1;
- card 2;
- detail metric 0/1/2;
- fault/status strip.

Perubahan telemetry identik = 0 physical paint bytes.
Icon hanya redraw bila card/page/status visual berubah.
Metric hanya redraw bila text/color berubah.
Target full page transition tetap <150 ms pada validated 24 MHz SPI.
Target telemetry incremental <25 ms ideal dan <50 ms maksimum.

## 16. Visual language

Background: navy gelap.
Cards: cream/off-white untuk kontras tinggi.
Primary accent: cyan.
READY: green; WARNING/STALE: amber; FAULT/E-STOP: red; disabled: gray.

Card status tidak hanya berdasarkan warna:
- `OK` / `WARN` / `FAULT` / `STALE` micro-label;
- dot/icon status;
- sehingga tetap terbaca tanpa mengandalkan warna saja.

Label card maksimal dua baris.
Angka primer menggunakan font terbesar yang masih aman untuk 320x240.
Tidak ada dekorasi yang mengurangi area data.

## 17. Klasifikasi fungsi

READ ONLY:
- semua live telemetry, diagnostics, status sensor, planning state.

EDIT SAFE:
- AUTO/MANUAL;
- manual speed preset;
- steering test angle;
- perception inference ON/OFF.

ACTION GUARDED:
- save/go waypoint;
- steering home/set center;
- manual motion test;
- clear recoverable fault.

SERVICE ONLY:
- raw FOC tuning;
- detect motor/encoder/hall;
- TFT/SPI maintenance;
- raw pin tests;
- firmware/DFU.
## 18. Tahap implementasi

### Tahap 0 — baseline
- simpan benchmark build/RAM/flash;
- capture current screenshots/menu tree;
- pastikan branch/status bersih atau diff terdokumentasi;
- jalankan semua self-check existing.

### Tahap 1 — navigation skeleton
- buat state machine HOME / MAIN / DOMAIN / DETAIL / SERVICE;
- implement HOME baru;
- implement MAIN MENU 3-domain;
- implement top BACK/LEFT/RIGHT;
- implement three-card pagination.

### Tahap 2 — asset pipeline
- normalisasi icon menjadi ukuran seragam;
- convert ke compact palette/mask asset;
- buat asset registry;
- uji flash footprint dan render time.

### Tahap 3 — ESC
- implement 12 cards + detail views;
- tambah normalized ESC telemetry bridge;
- implement safe edit/test gates;
- hardware test F4↔F103/VESC tanpa mengubah pin.

### Tahap 4 — PERSEPSI
- implement 9 cards + detail views;
- tambah metric bridge yang belum tersedia;
- uji stale/camera disconnect/inference OFF.

### Tahap 5 — NAVIGASI
- implement 12 cards + detail views;
- tambah EKF/odom/Nav2 summary bridge;
- uji GNSS loss, EKF degraded, Nav2 unavailable, cancel navigation.
### Tahap 6 — Service Mode
- pindahkan SYSTEM lama ke Service Mode;
- tambah 10 service cards;
- akses hanya saat STOPPED;
- pertahankan motion-safe gate untuk TFT/SPI/maintenance.

### Tahap 7 — robustness
- stale/freshness di semua domain;
- disconnect/reconnect ROS/F103/camera/GNSS;
- E-STOP selama rendering/test;
- touch slide-cancel/generation invalidation;
- 500+ page transition stress.

### Tahap 8 — cleanup
- hapus menu enum/renderer/card lama yang tidak dipakai;
- hapus duplicate icon/asset;
- hapus compatibility path yang tidak lagi diperlukan;
- rapikan naming dan descriptor;
- update README/test/mockup agar sesuai UI baru.

### Tahap 9 — final validation
- build release;
- flash hardware;
- test HOME/MAIN/semua carousel/detail;
- test action guard dan dead-man;
- test UART/SPI error/drop;
- ukur full-page dan incremental render;
- verifikasi RAM/flash/stack margin;
- final audit no dead code dan no unreachable menu.

## 19. Acceptance criteria

- HOME hanya punya 3 data besar + satu MENU button.
- MAIN MENU hanya HOME + ESC + PERSEPSI + NAVIGASI.
- setiap domain root menampilkan tepat tiga card pada satu waktu.
- BACK/LEFT/RIGHT selalu di atas dan konsisten.
- semua card punya logo representatif + label.
- semua submenu reachable dan dapat kembali tanpa loop/menu buntu.
- stale data tidak pernah ditampilkan seperti live/READY.
- action berbahaya tidak mungkin berjalan saat motion gate gagal.
- release touch selalu menghentikan manual motion.
- no SPI/UART regression dan no unexpected reset.
- zero telemetry change menghasilkan zero unnecessary repaint.
- tidak ada overlap/out-of-bounds pada 320x240.
- tidak ada dead code renderer/menu lama setelah migrasi selesai.
