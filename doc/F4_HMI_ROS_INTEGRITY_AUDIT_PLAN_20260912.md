# F4 + HMI + ROS Integrity Audit & Implementation Plan — 2026-09-12

## Baseline yang diaudit
- F4gateway branch `v1`, transport NEO3Pro/MCP2515 sudah fast-ISR + raw FIFO 128.
- ROS `stmf4_hmi_bridge` menjadi bridge F4/HMI/sensor; ESC dimiliki langsung package `esc`.
- CAN live baseline: TEC/REC/EFLG=0, RX overflow=0, protocol error=0, peer ACTIVE, DNA 2/2/2.
- Build F4 dan `stmf4` PASS; cppcheck F4+ROS menghasilkan 0 warning.

## Temuan P0 — wajib diperbaiki lebih dulu
1. **Safety STOP dari HMI masih best-effort.** `stopDriveTest()` dan `stopSteeringTest()` memakai `printBoth()` -> low-priority USB ring; kegagalan enqueue diabaikan.
2. **ROS manual drive tidak punya command lease/deadman.** Setelah `drive_=FWD/REV`, `commandTick()` terus publish selama gate umum masih valid. STOP yang hilang saat USB congestion dapat memperpanjang motion.
3. **Safety gate ROS lebih longgar daripada F4.** `/hmi/request` dari Web dapat MODE/DRIVE/STEER langsung; `manualMotionAllowed()` tidak menguji state awal/stationary seperti F4 HMI.
4. **Waypoint GO tidak fail-closed lengkap.** `goWaypoint()` hanya saved+AUTO; belum wajib E-stop clear, motion_ready, nav2_ready, fresh map localization, dan finite persisted waypoint.
5. **Nav cancel memblok single-thread executor sampai 250 ms.** `wait_for_service(250ms)` dapat menahan serial F4/heartbeat saat Nav2 tidak siap.

## Temuan P1 — state/contract integrity
6. ROS selalu mengirim `NEO:LED:AUTO` saat reconnect, tetapi NEO3PRO firmware tidak mendukung AUTO; menghasilkan `ERR:NEO:LED:ARGS`.
7. `/neo3/command` menawarkan command legacy (AUTO/ON/BEEP/BUZZER) yang tidak sama dengan command set NEO3PRO.
8. `closeSerial()` tidak meng-invalidasi semua cached NEO3PRO health/wire/time state; transient-local diagnostic string dapat menyisakan health lama setelah disconnect.
9. MCU timestamp epoch dan per-stream wire sequence tidak di-reset eksplisit pada host-session baru; rapid reboot dapat membuat packet pertama dianggap duplicate/out-of-order dan timestamp sementara ter-clamp.
10. Absolute GNSS time bridge menerima formula TAI/GPS custom; ArduPilot reference hanya menggunakan absolute timestamp saat standard=UTC.
11. `publishHardware()` masih melakukan read TEC/REC/EFLG SPI2 lagi; health service sudah bisa menjadi satu-satunya sampler/cache.
12. `writeLineCritical()` memakai `HAL_Delay(1)` dan `flush()` busy-poll; keduanya belum cooperative-yield ke realtime service.

## Temuan P1/P2 — HMI, diagnostics, CI
13. HMI diagnostic fields masih overload nama legacy VESC (`pb6VescTx/pb7VescRx`) untuk MCP pins; raw source benar tetapi contract membingungkan.
14. `SYSTEM_ERRORS` belum menampilkan raw FIFO HWM/overflow, USB high-priority drops, peer state, DNA fault, dan host session freshness.
15. F4 production `F4_ESC_GATEWAY=0`, tetapi ROS bridge masih menyimpan dead legacy VESC transport code/members; test lama masih menganggap F4↔F103 aktif.
16. `hmi_10repo_audit_self_check.py` masih menuntut visible `vescFrameErrors` walaupun ESC bukan domain F4 lagi.
17. `recovery_state_machine_self_check.py` masih mengharapkan manifest v1; bootloader/generator production sudah manifest v2 + board ID.
18. `stmf4_gateway_self_check.py` masih mengharapkan `vesc_uart_baud_expected` dan layout flash lama; test contract tidak lagi mewakili architecture production.
19. Waypoint persistence belum memvalidasi finite/range saat load dan belum durability-grade fsync+rename; file korup dapat menghasilkan goal invalid bila tidak ditutup.
20. `stmf4_hmi_bridge.cpp` sangat monolitik (~166 kB source), sehingga perubahan sensor/HMI/control mudah saling memengaruhi dan regression coverage sulit dipetakan.
21. CAN TX masih hanya TXB0 + status polling SPI. Normal runtime sekarang zero-loss, jadi ini optimasi P2, bukan blocker.
22. HMI full redraw pernah mencapai ratusan ms; realtime CAN aman karena yield, tetapi touch/UI latency perlu budget dan qualification tersendiri.

# TAHAP 1 — Safety + contract + state integrity
### 1. Safety transport F4→ROS
- Buat `sendControlHighPriority()` terpisah dari telemetry `printBoth()`.
- Semua STOP (`DRIVE`, `STEER`, `NAV`) wajib high-priority dan return value tidak boleh diabaikan.
- Tambah latched `pending_stop` yang di-retry tiap loop sampai berhasil enqueue/session hilang.
- Pisahkan counter `control_tx_drop`, `stop_retry`, `stop_ack_timeout` dari low-priority telemetry drops.
- `writeLineCritical()` tidak boleh HAL_Delay; gunakan cooperative realtime service/yield bounded.

### 2. Manual command lease/deadman di ROS
- FWD/REV/steering hanya aktif jika lease HMI fresh (target 250–350 ms).
- F4 saat HOLD mengirim heartbeat/lease control periodik, bukan hanya satu state transition.
- Timeout lease, CDC disconnect, mode change, E-stop, ESC stale, atau source ownership berubah => publish zero + source STOP pada tick berikutnya.
- Start manual harus stationary/fresh; setelah lease aktif controller boleh bergerak sesuai bounded command.

### 3. Satu safety gate untuk TFT + Web
- Buat satu fungsi `manualRequestAllowed(origin, action)` yang dipakai TFT dan `/hmi/request`.
- Jangan biarkan Web bypass stationary/start interlock F4.
- MODE:MANUAL dari Web/TFT tidak otomatis memberi motion authority.
- Validasi active-source/mux ownership sebelum nonzero HMI cmd_vel diterbitkan.

### 4. Navigation mission gate
- `goWaypoint()` wajib: connected, AUTO, E-stop clear, waypoint finite+saved, `motion_ready`, `nav2_ready`, fresh `/odometry/filtered_map`, dan localization acceptable.
- Reject dengan reason eksplisit; jangan publish goal lalu berharap downstream menolak.
- `stopNavigation()` dibuat nonblocking: tidak ada `wait_for_service()` di executor callback; gunakan cached service readiness/async retry.
- Load waypoint: reject NaN/Inf/out-of-range; invalid record menjadi unsaved.
- Persistence: temp file -> flush/fsync -> atomic rename -> optional directory fsync.

### 5. Session/state invalidation ROS
- Saat `closeSerial()`: reset NEO3PRO active/health/meta/covariance/wire-sequence/MCU-clock epoch dan publish connected=false.
- Raw diagnostic transient-local topics harus mendapat sentinel `DISCONNECTED,session=...` atau dijadikan volatile; subscriber baru tidak boleh melihat stale ACTIVE tanpa connected context.
- Saat host-session ACK baru: mulai epoch baru, clear per-stream wire state dan timestamp mapper secara deterministik.

### 6. NEO command contract
- Hapus side effect `NEO:LED:AUTO` dari initial session.
- NEO3PRO startup hanya `STATUS/USB STATUS`; jangan mengubah LED/parameter saat reconnect.
- Buat allowlist profile-specific command: STATUS, CAN STATUS/RECOVER, BARO, LED BRIGHTNESS/RGB/OFF.
- Legacy BEEP/BUZZER/AUTO hanya tersedia pada profile yang benar-benar mendukungnya.
- Tambah contract self-check ROS↔F4 agar command tidak bisa drift lagi.

### 7. CAN/USB low-level cleanup tanpa mengubah RX architecture
- Cache TEC/REC/EFLG di `serviceCanHealth()`; `publishHardware()` hanya membaca cache, sehingga diagnostics tidak menambah SPI2 contention.
- Pertahankan PB10 fast ISR + raw FIFO 128 + main-context libcanard.
- Rapikan komentar TXEP/bus-off agar sesuai behavior aktual.
- Ubah `flush()` menjadi bounded cooperative flush yang tetap menjalankan realtime service.

# TAHAP 2 — HMI diagnostics, refactor, qualification
### 8. HMI observability dan latency
- Ganti alias pin legacy dengan field MCP khusus: INT/CS/SCK/MISO/MOSI.
- SYSTEM_ERRORS tampilkan MCP overflow, SW FIFO HWM/overflow, ISR max/budget, USB high-priority drop, peer state, DNA fault, host session/generation.
- Bedakan historical counter vs delta-since-session agar operator tidak salah membaca fault lama.
- Ukur full-frame dan dirty-frame p50/p95/p99; optimasi hanya renderer yang nyata mahal.
- Target touch action-to-visible response terukur dan tidak mengurangi CAN service density.

### 9. Bersihkan architecture ROS
- Hapus dead VESC-over-F4 methods/members/subscriptions dari `stmf4`; ESC hanya direct package `esc`.
- Pecah bridge bertahap menjadi SerialTransport, Neo3Protocol, HmiControlSafety, NavigationGateway, ExtendedTelemetry.
- Refactor harus behavior-preserving dan dilakukan setelah P0 tests tersedia.
- Parameter safety/protocol (`neo3_require_protocol_crc`, `neo3_safety_button_as_estop`, baro frame id) ditulis eksplisit di YAML.

### 10. Timestamp parity ArduPilot
- Untuk Fix2 absolute timestamp, gunakan epoch GNSS hanya bila `gnss_time_standard == UTC`, sama seperti AP_GPS_DroneCAN.
- NONE/TAI/GPS yang belum tervalidasi DSDL-test harus fallback ke MCU/reception time, bukan conversion custom tanpa proof.
- Tambah test UTC, invalid leap, clock disagreement, F4 reboot <5 s, wrap uint32 ms.

### 11. CI/test contract repair
- Recovery self-check wajib manifest v2 + board ID `0xF411CE01` + sector6 preservation.
- HMI audit tidak lagi mengharuskan VESC framing field pada F4_ESC_GATEWAY=0.
- `stmf4_gateway_self_check` ditulis ulang untuk direct ESC architecture, current 224 KiB app region, NEO3PRO command allowlist, session reset, safety lease.
- Pertahankan cppcheck zero-warning + PlatformIO `-Wall -Wextra -Wformat=2 -Wshadow -Wundef`.

### 12. P2 CAN TX optimization (setelah functional green)
- Pertimbangkan TXB0/TXB1/TXB2 descriptor + TXnIF event completion untuk mengurangi SPI status polling.
- RX tetap absolute priority; jangan pindahkan libcanard/DSDL ke ISR.
- Hanya diterapkan bila A/B test membuktikan latency/contention membaik tanpa regression.

## Acceptance qualification
- Inject USB low-ring saturation saat manual HOLD; STOP harus tetap diterima/lease timeout menghentikan ROS <=350 ms.
- Putus CDC saat drive command aktif; `/hmi/cmd_vel` menjadi zero dan source STOP dalam satu lease timeout.
- Web mencoba DRIVE saat AUTO/E-stop/stale ESC/moving: semua reject.
- Waypoint GO dengan nav2 off, localization stale, E-stop, NaN persisted waypoint: tidak ada `/navigation/goal_request`.
- Nav cancel service unavailable: serial F4 rates/CAN diagnostics tetap berjalan tanpa 250 ms executor stall.
- 20x F4 reset + 20x ROS respawn: tidak ada stale ACTIVE transient-local state dan timestamp tidak mundur/freeze.
- NEO reconnect tidak mengirim LED command otomatis.
- 30 min HMI+USB+ROS stress: CAN overflow/protocol error delta=0, high-priority USB drop=0, peer loss=0, DNA verification failure=0.
- Final 2 h combined soak setelah semua P0/P1 patch sebelum production commit.
