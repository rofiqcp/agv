# AGV End-to-End Robustness Audit & Recovery Plan

Tanggal audit: 2026-09-10
Target: ROSWeb localhost:5000 -> ROS/Ackermann -> STM32F411 -> STM32F103RCT6 -> dual FOC motor, dengan command dan feedback realtime 50 Hz yang deterministik, fail-safe, dan recoverable.

## 1. Scope

Tree yang diaudit:
- `/home/sirobo/agv`
- `/home/sirobo/agv/hoverboard-vesc`
- `/home/sirobo/agv/f411_pio_arduino`
- `/home/sirobo/agv/F4gateway`
- ROS packages `src/esc`, `src/stmf4`, `src/navigation/web`

Audit membandingkan 10-15 commit terakhir AGV dan hoverboard-vesc, histori F411 lama, F4gateway baru, working tree saat ini, serta hasil pengukuran hardware live sebelumnya.

## 2. Executive conclusion

Masalah utama saat ini bukan kapasitas 115200 baud dan bukan F103 parser/FOC.
Regresi terbesar muncul ketika gateway berpindah dari implementasi STM32 Arduino yang matang ke implementasi native HAL di `f411_pio`/`F4gateway`.

Arduino core lama memiliki TX ring yang self-chaining langsung dari TX-complete IRQ. Native `HalUartPort` awal hanya mengosongkan `tx_busy_` pada callback dan bergantung pada pemanggilan `service()` berikutnya. `Board_Service()` juga tidak dipanggil dari main loop awal. Ini dapat membuat backlog dan recovery storm di bawah burst ROS 50 Hz.
Native HMI juga memakai banyak `HAL_SPI_Transmit()` blocking pada sekitar 6 MHz dan glyph pixel-by-pixel. Saat ROS terus mengubah telemetry, `uiDirty` membuat redraw 100 ms berulang. Akibatnya `gVesc.poll()` dapat tertunda walaupun USART IRQ tetap menerima byte.

F103 live menunjukkan link/parser pada dasarnya sehat: RX queue drop 0, puluhan ribu frame valid, sedikit CRC error lifetime, TX queue tidak stuck, dan FOC ISR deadline miss 0. Jadi perbaikan harus memprioritaskan F411 scheduling/transport sebelum mengubah baud atau algoritma FOC.

## 3. Git timeline penting

### AGV
- `1213952` 2026-09-10 00:21 — F411 gateway dipindah menjadi pinned submodule `F4gateway`.
- `4a3ecf9` 2026-09-09 22:02 — perubahan besar `f411_pio`, `f411_pio_arduino`, ESC, STM-F4 bridge.
- `a7991fd` 2026-09-09 02:04 — update STM-F4 bridge/config.
- `b447b43` 2026-09-08 02:02 — DFU/upload + ACKermann/runtime update.
- `7073c23` 2026-09-07 22:43 — VESC Tool bridge/runtime maintenance update.
- `a8063c1` 2026-09-07 20:55 — native `f411_pio` besar + ESC integration.
- `e752c51` 2026-09-07 16:54 — `f411_pio_arduino` masuk sebagai implementasi F411 penuh.

### hoverboard-vesc
- `f3b5e31` 2026-09-09 22:02 — main/FOC/ISR/bootloader/uploader refactor.
- `4291d25` 2026-09-09 09:46 — ISR profiling, ADC timing, more slow-path extraction.
- `61c389f` 2026-09-08 23:41 — strong ISR trampoline, legacy telemetry removal.
- `a85a494` 2026-09-08 02:02 — FOC housekeeping/timeouts optimization.
- `0e4ac40`..`165da4e` — VESC protocol/FOC feature growth.
Commit sedikit sebelum window, `9888ed5` (2026-09-06), sangat penting: USART3 IDLE IRQ berhenti memanggil `usart3_rx_check()` sehingga DMA RX hanya didrain dari main context. `1382305` kemudian menaikkan prioritas USART3 RX/DMA. Kedua keputusan ini terbukti benar dan harus dipertahankan.

## 4. Provenance F411: kenapa perbandingan harus benar

`F4gateway` baru hanya memiliki dua commit awal (`952f223`, `973ed23`). Hash menunjukkan:
- `F4gateway` initial `main.cpp` identik dengan `4a3ecf9:f411_pio/src/main.cpp`.
- `HmiDisplay.cpp`, `BoardSupport.cpp`, `BoardSupport.h`, dan `UsbCdcPort.cpp` juga identik dengan native `f411_pio`.
- Jadi `F4gateway` bukan kelanjutan langsung `f411_pio_arduino`.

Versi lama yang pernah dipakai user, `f411_pio_arduino`, memakai:
- STM32 Arduino `Uart` pada PB7/PB6 USART1.
- Arduino USB `Serial` 1 Mbps.
- TFT_eSPI dengan SPI display 10 MHz.
- TX ring bawaan core yang langsung self-chain dari callback.
- main loop yang menaruh safety + VESC poll di depan dan mengulang poll VESC.

Versi native mengganti semua primitive itu dengan custom HAL implementation. Fungsinya sama di level API, tetapi sifat scheduling/reentrancy/queue-nya tidak sama.

## 5. Root cause matrix

### RC-1 — Native F411 TX ring tidak self-chain [CONFIDENCE: VERY HIGH]
Initial native `irqTxComplete()` hanya advance tail dan set `tx_busy_=false`. Segmen berikutnya baru mulai jika `service()` dipanggil lagi.
Arduino `_tx_complete_irq()` langsung memasang transfer berikut dari IRQ yang sama.
Dampak: burst/batched ROS dapat meninggalkan queued data dan menaikkan latency/backlog.
### RC-2 — `Board_Service()` awal tidak dipanggil main loop [CONFIDENCE: VERY HIGH]
Fungsi ada, tetapi initial native main tidak memanggilnya. Ini memperparah RC-1 dan membuat keberhasilan TX bergantung pada write/poll berikutnya.

### RC-3 — Native UART recovery asynchronous race [CONFIDENCE: HIGH]
Initial `end()` memakai `HAL_UART_Abort_IT()` lalu langsung `HAL_UART_DeInit()`/`begin()`. Abort callback dapat datang setelah peripheral sudah di-init ulang dan mengubah HAL state generasi baru.

### RC-4 — Display native memblok realtime service [CONFIDENCE: VERY HIGH]
Driver native berjalan sekitar 6 MHz dan memakai blocking SPI. Banyak font/glyph digambar pixel-by-pixel; setiap pixel menjalankan set-window + transfer. Dengan redraw 100 ms terus menerus, satu render dapat memakan bagian besar atau lebih dari budget 20 ms transport.

### RC-5 — Recovery memakai waktu parser, bukan waktu byte-hardware [CONFIDENCE: HIGH]
UART IRQ dapat terus menaruh byte ke ring, tetapi `gVesc.poll()` terlambat memprosesnya karena display. `last_valid_frame_ms_` lalu terlihat stale dan recovery menganggap UART mati, padahal data valid menunggu di ring. Recovery tersebut sendiri merusak frame parsial dan memicu loop recovery berikutnya.

### RC-6 — Runtime queue policy awal dapat menjadi sticky [CONFIDENCE: HIGH]
Saat queued bytes > threshold, batch baru ditolak tetapi stale queue lama tetap ada. Sistem dapat menetap pada backlog lama. Runtime seharusnya latest-value-wins tanpa memotong frame yang sedang on-wire.

### RC-7 — F103 UART error callback punya multi-owner DMA [CONFIDENCE: HIGH]
Sepanjang 10 commit terakhir, error callback IRQ masih mengubah `usart3RxOldPos` dan restart DMA, sementara main juga drain/recover DMA. Ini race nyata. Working tree sekarang sudah memindahkan restart ke main context; solusi itu harus dipertahankan setelah test fault injection.

### RC-8 — HMI/diagnostic traffic berbagi USB CDC dengan motor telemetry [CONFIDENCE: MEDIUM-HIGH]
USB 1 Mbps cukup secara bandwidth, tetapi banyak line/status/GNSS/UI update dapat menambah burst dan queue pressure. Motor RX envelope harus memiliki prioritas dan bounded-drop semantics yang eksplisit.
## 6. Yang BUKAN akar masalah utama

### 115200 baud runtime bukan oversubscription
Runtime 50 Hz sekarang mengirim per tick:
- LEFT `COMM_SET_POS`
- RIGHT `COMM_SET_RPM`
- LEFT `COMM_GET_VALUES_SELECTIVE`
- RIGHT `COMM_GET_VALUES_SELECTIVE`
- steering state 10 Hz

Runtime selective response sekitar 42 byte per motor, sehingga arah F103->F411 sekitar 4.2 kB/s untuk dua motor pada 50 Hz. Arah F411->F103 juga jauh di bawah kapasitas serial 115200 8N1 (~11.52 kB/s). Jadi baud tidak perlu dinaikkan sebagai patch pertama.

### F103 FOC ISR bukan bottleneck saat ini
Pengukuran live sebelumnya:
- TIM1/TIM8 ARR=2000, PSC=0, 16 kHz center-aligned.
- `FOC_ISR_MAX=2768` cycles dari budget 4000.
- deadline miss = 0.
- USART3 register dan DMA sesuai source.
- F103 RX queue drop = 0 dan TX queue tidak stuck.

### ROS command timer bukan bottleneck awal
`/stmf4/vesc/runtime_tx` pernah terukur tepat ~50.00 Hz. Problem muncul setelah message masuk F411 / saat reply kembali, bukan pada timer Ackermann awal.

## 7. Fitur baru yang harus dipertahankan

- Runtime vs Maintenance ownership yang eksplisit.
- TCP Python 65101 dan VESC Tool 65102 dengan arbitration.
- Physical E-stop lokal F411->F103, independen ROS.
- VESC timeout di domain ADC/ISR F103.
- Strong ISR trampoline untuk build LTO.
- ADC clock DIV6 dan ISR profiler.
- Dual-motor staggered FOC + two-phase MOE gate.
- F103 request/reply VESC native, tanpa unsolicited legacy 72-byte stream.
- GET_VALUES selective integer-scaled pada F103.
- Config update gate dan bootloader staged update/CRC.
- ROS batch 50-Hz satu message per tick; batching mengurangi callback/USB line tanpa menambah wire payload.
- ROSWeb SSE 20 ms sebagai observer realtime, bukan direct UART owner.
- Parser APP payload 700 byte, dengan bootloader tetap 512 byte.
- Maintenance diagnostic decimation untuk menjaga headroom.

## 8. Working-tree fixes yang harus divalidasi, bukan dibuang

F4gateway working tree saat audit sudah memiliki candidate fixes:
- synchronous `HAL_UART_Abort()` untuk full recovery.
- clear pending UART IRQ sebelum re-enable.
- TX-complete self-chain.
- `Board_Service()` dipanggil di main loop.
- latest-value queue replacement yang mempertahankan frame aktif.
- cooperative realtime callback di long TFT transfers.
- temporary wiring command TXHOLDLOW sudah dibuang.

hoverboard-vesc working tree sudah memiliki candidate fixes:
- signed-shift UB cleanup.
- EEPROM/config write fail-closed.
- UART error IRQ event-only + main-context DMA restart.
- NMI emergency PWM hard-off.
- ST-Link true connect-under-reset upload sequence.
- stale regression tests disesuaikan.

Semua candidate fix harus diperlakukan sebagai patch set yang belum released sampai hardware acceptance test hijau.

## 9. Target architecture

Pisahkan empat kelas kerja secara eksplisit:
1. **Hard realtime F103** — ADC/PWM/FOC/fault/watchdog.
2. **Transport realtime F411** — USART1 RX/TX, frame forwarding, E-stop, USB motor channel.
3. **Best-effort F411** — TFT, touch, GNSS, magnetometer, status display.
4. **Host/UI** — ROS, ROSWeb, TCP maintenance, logging, plotting.
Transport motor tidak boleh menunggu TFT, I2C, GNSS, Web, log, atau config IO. Best-effort layer boleh kehilangan refresh; command/feedback motor tidak boleh kehilangan deadline karena best-effort layer.

### F411 transport design rules
- TX ring self-chain dari completion callback.
- RX IRQ hanya copy byte/event ke ring; parser di bounded service context.
- `service()` harus dipanggil dengan period maksimum terukur, bukan berharap main loop cepat.
- full UART recovery synchronous dan generation-safe.
- recovery trigger harus membedakan `wire silent`, `bytes queued but parser delayed`, dan `CRC invalid`.
- runtime queue = latest complete batch wins; jangan replay stale setpoint.
- jangan abort frame aktif kecuali E-stop/fatal fault.
- E-stop boleh preempt queue dan mengirim emergency frame sesegera mungkin.
- USB motor telemetry diberi priority class di atas HMI/GNSS debug lines.

### F411 display design rules
- gunakan bulk/window transfers, bukan per-pixel HAL calls untuk glyph normal.
- target display write >= 10 MHz bila hardware valid, atau gunakan SPI DMA.
- redraw hanya area yang berubah.
- limit refresh telemetry HMI 5-10 Hz; display tidak perlu 50 Hz untuk manusia.
- selama transfer panjang, transport service tetap berjalan atau transfer benar-benar asynchronous DMA.

### ROS design rules
- Ackermann setpoint publisher 50 Hz, KeepLast(1) secara semantik latest-value.
- satu complete runtime batch per 20 ms.
- feedback setiap motor target 50 Hz, tetapi diagnostic mahal dipisah 10/25 Hz.
- runtime dan maintenance tidak boleh aktif bersamaan.
- Web/SSE hanya observer; event client lambat tidak boleh backpressure control executor.
- TCP 65101/65102 hanya mendapat UART ownership setelah vehicle idle + runtime safe-stop.
## 10. Remediation phases

### Phase 0 — Freeze forensic baseline
Tujuan: tidak kehilangan versi yang pernah bekerja dan patch terbaru.

1. Tag/branch snapshot AGV current HEAD dan semua dirty diffs.
2. Simpan patch terpisah untuk AGV, hoverboard-vesc, F4gateway.
3. Catat submodule SHA, binary F103/F411 yang sedang terpasang, config YAML, EEPROM signature.
4. Simpan snapshot `f411_pio_arduino` pada commit `4a3ecf9` sebagai golden behavioral reference.
5. Simpan snapshot native `f411_pio` `4a3ecf9` dan initial F4gateway `952f223` untuk regression comparison.
6. Jangan merge perubahan perception/navigation yang tidak terkait transport selama recovery.

Exit criteria:
- semua state dapat direproduksi dengan commit + patch.
- tidak ada binary yang tidak diketahui provenance-nya.

### Phase 1 — Build an offline F411 transport regression harness
Buat test host/model untuk `HalUartPort` yang mensimulasikan:
- write A saat idle.
- write B/C saat A masih busy.
- callback TX complete.
- ring wrap.
- HAL_BUSY pada start.
- abort/re-init.
- error callback saat RX/TX aktif.
- E-stop queue preemption.

Invariant utama: setiap accepted complete frame dikirim tepat sekali, urut, tanpa truncation, kecuali runtime stale batch yang sengaja superseded atau E-stop preemption.
### Phase 2 — Make F411 UART transport deterministic

1. Pertahankan TX-complete self-chain seperti Arduino core.
2. `Board_Service()` tetap sebagai recovery/kick fallback, bukan mekanisme utama chaining.
3. Full `end()/begin()` memakai synchronous abort, IRQ disable, pending-clear, HAL state reset, lalu init generasi baru.
4. Jangan memanggil asynchronous abort lalu langsung re-init.
5. Runtime backlog policy: pertahankan segmen aktif, drop hanya queued stale batches, enqueue latest complete batch.
6. Tambahkan counters: tx_segments_started/completed, tx_superseded_bytes, tx_start_fail, rx_irq_bytes, max_queue_bytes.
7. Tambahkan timestamp `last_rx_irq_ms` terpisah dari `last_valid_frame_ms`.

Recovery hanya diizinkan jika wire benar-benar gagal, misalnya:
- runtime command baru memang sedang dikirim;
- tidak ada valid frame dalam deadline;
- tidak ada progress RX IRQ / atau parser menunjukkan corruption nyata;
- bukan hanya karena main parser terlambat sementara ring berisi data.

Exit criteria:
- direct Python 50-Hz request/reply 10 menit tanpa queue growth.
- forced one-time UART recovery kembali sehat tanpa reset F411.
- `uart_err=0` normal, `frame_err` delta=0 normal, runtime queue tidak monoton naik.

### Phase 3 — Decouple display from motor transport

1. Ukur waktu `drawUiFrame()` min/avg/p95/max dengan DWT/TIM.
2. Pertahankan cooperative service hook sementara sebagai safety net.
3. Refactor text/glyph ke buffered row/span rendering; hilangkan drawPixel-per-glyph pada dynamic UI.
4. Gunakan bulk SPI transaction dan naikkan write clock menuju 10 MHz setelah display integrity test.
5. Kandidat akhir lebih baik: SPI DMA + completion event sehingga CPU tidak blocking.
6. Dynamic HMI telemetry cukup 5-10 Hz; transport motor tetap 50 Hz.
7. Page switch/full redraw tidak boleh membuat motor telemetry stale.

Exit criteria:
- full-screen redraw berulang tidak mengubah runtime feedback rate lebih dari 5%.
- max transport-service gap < 5 ms; target ideal < 2 ms.
- tidak ada false UART recovery selama display stress 10 menit.

### Phase 4 — Prioritize USB CDC traffic

Buat output classes:
- P0: E-stop/control acknowledgements yang safety-critical.
- P1: VESC RX realtime frame envelope.
- P2: required HMI state/ROS heartbeat.
- P3: GNSS/status/debug/log.

P1 tidak boleh ditolak karena P3 memenuhi queue. Implementasi dapat berupa reserved capacity, separate software rings, atau drop-oldest pada low-priority class.

Tambahkan counters per class dan high-watermark USB TX ring.
Jangan block main selama ratusan ms untuk maintenance response; gunakan chunked bounded service yang tetap melayani safety.

Exit criteria:
- FOC/VESC 50 Hz tetap fresh sambil GNSS, MAG, HMI status, dan page redraw aktif.
- low-priority drop diperbolehkan dan terlihat di counter; motor telemetry drop normal = 0.

### Phase 5 — Complete F103 UART single ownership

1. IRQ IDLE hanya clear/record event.
2. UART error callback hanya menaikkan counter + pending flag.
3. Main context satu-satunya owner `usart3RxOldPos`, DMA stop/start, parser reset, peripheral recovery.
4. Error recovery selalu release kedua motor sebelum membuang partial frame.
5. Inject FE/NE/ORE, truncated packet, DMA wrap, random bytes, burst packet.
Exit criteria:
- no duplicate/skip bytes pada host fault-injection.
- RX queue drop 0 normal.
- CRC error delta 0 pada clean wire test.
- recovery tidak menyebabkan motor stale command replay.

### Phase 6 — Finish F103 safety/FOC correctness work

1. Selesaikan seluruh signed-shift UB sweep dengan widening/multiplication terdefinisi.
2. EEPROM/MC/APPCONF/steering transaction: invalidate -> abort-on-any-failure -> payload -> readback verify -> valid signature.
3. Atomic command publication ke ISR: payload/targets dahulu, memory barrier, commit mode/valid terakhir atau bounded critical section.
4. Unify source state menjadi satu authority enum: NONE, LEGACY, VESC_LIVE, VESC_TIMEOUT_BRAKE, VESC_OWNED_STOP, ESTOP, FAULT.
5. NMI/HardFault/MemManage/BusFault/UsageFault semuanya register-only MOE hard-off.
6. Tambah catastrophic raw-current guard pada bridge settle blanking window.
7. Definisikan semua `_SAMPLES` dengan domain ADC_FRAME/CURRENT_SLOT/OUTER_PID/HOUSEKEEPING.
8. Sinkronkan outer PID scheduler ke PWM master clock tanpa memindahkan float/divide ke ISR.
9. Tambah IWDG setelah semantics boot/update/commissioning terbukti aman.

Exit criteria:
- ASan/UBSan host clean.
- ISR forbidden-call graph clean.
- deadline miss 0.
- EEPROM brownout injection seluruh posisi write PASS.
- preemption test command publication PASS.

### Phase 7 — Protect F103 flash/update path

1. Pertahankan true ST-Link connect-under-reset.
2. Guard target STM32F103RCT6 DEV_ID 0x414 saat halted.
3. Program/verify APP pada 0x08002800 sebelum release reset.
4. Jangan lakukan preflight reset-run.
5. Release check memakai actual APP linker region 120 KiB, bukan total 256 KiB board size.
6. Tetapkan minimum flash margin release; jangan menerima APP yang praktis menyentuh 120 KiB limit tanpa cleanup.

Exit criteria:
- 10x consecutive ST-Link reflash setelah APP running, tanpa power-cycle.
- verify image setiap siklus.
- bootloader staging recovery power-loss PASS.

### Phase 8 — Formalize protocol contracts

Dokumentasikan tiga ukuran berbeda:
- F103 application payload max = 700 byte.
- F103 bootloader payload max = 512 byte.
- F411 framing/ring harus mampu membawa frame APP terbesar + envelope.

Tambahkan compile/static assertions dan shared test vectors untuk short/medium/long VESC packet.

Runtime contract:
- 50 Hz command batch.
- LEFT SET_POS standard VESC 0..360 maps physical -30..+30 deg.
- RIGHT SET_RPM standard VESC signed ERPM.
- LEFT + RIGHT selective values 50 Hz.
- steering calibration/status 10 Hz.

Maintenance contract:
- GET_VALUES fast 50 Hz bila bandwidth mengizinkan.
- rotor snapshot diagnostic <=25 Hz.
- slow config/status 1-10 Hz.
- adapt rate downward ketika queue high-watermark/drop/latency meningkat.

Exit criteria:
- parser interoperability tests ROS/F411/F103 all PASS.
- no 512-vs-700 false rejection.

### Phase 9 — ROS transport cleanup

1. Semantik runtime publisher menjadi KeepLast(1) latest-value; jangan biarkan DDS queue delapan command batch lama direplay ke actuator.
2. Pertahankan one-message-per-tick batching.
3. Pisahkan command freshness dari feedback freshness.
4. Tambahkan sequence/timestamp host pada diagnostics agar latency tiap layer dapat dihitung.
5. `stmf4_hmi_bridge` harus memprioritaskan `VESC:RX` parsing terhadap HMI text/status processing.
6. Connected state harus freshness-based dari valid VESC frame, tetapi tidak memicu recovery hanya karena UI thread terlambat.
7. Expose metrics: tx_rate, rx_rate, feedback_age, parser_crc, format_error, F411 queue high-water, USB drop.

Exit criteria:
- `/stmf4/vesc/runtime_tx` = 50 Hz ±1%.
- `/stmf4/vesc/rx` valid motor responses efektif 100 response/s total (50 per motor) tanpa long gaps.
- `/esc/foc/telemetry` publish 50 Hz class; p99 gap target <30 ms, hard max <50 ms normal.
- no DDS stale replay setelah node pause/restart.

### Phase 10 — Make localhost:5000 observer/control path robust

1. Pertahankan SSE event timer 20 ms untuk UI observation.
2. SSE client lambat harus drop/coalesce deltas, tidak memblok ROS callbacks.
3. `/api/experiment/trial/motion` hanya publish desired command; Ackermann tetap satu-satunya actuator authority.
4. Web render tidak menjadi sumber command keepalive. Command watchdog berada di ROS/firmware, bukan browser.
5. Tambahkan UI telemetry health dari timestamps asli source, bukan timestamp saat browser menerima SSE.
6. Tampilkan rate, age, jitter, missing packets untuk runtime_tx, VESC RX, LEFT values, RIGHT values, FOC telemetry.
7. Disable actuator button jika feedback stale, E-stop aktif, maintenance aktif, atau owner bukan expected authority.
8. STOP harus idempotent dan dapat dikirim walau state UI stale.

Exit criteria:
- membuka/menutup browser, tab throttle, SSE reconnect, refresh halaman tidak mengubah motor transport rate.
- localhost:5000 menunjukkan data source 50 Hz tanpa menggandakan sample UI.
- browser CPU/render load tidak mempengaruhi ROS control timer.

### Phase 11 — TCP 65101/65102 isolation

1. Runtime harus safe-stop sebelum ownership pindah.
2. 65101 Python priority > 65102 VESC Tool sesuai desain saat keduanya ada.
3. Maintenance entry harus vehicle-idle gated.
4. Maintenance exit flush parser/session state lalu restore runtime tanpa replay.
5. Disconnect/half-open TCP punya lease timeout dan otomatis kembali runtime safe.
6. Maintenance telemetry rate harus adaptive dan tidak membuat F103 TX queue drop.

Exit criteria:
- connect/disconnect 65101 sebanyak 50 siklus tanpa stuck owner.
- connect/disconnect 65102 sebanyak 50 siklus tanpa stuck owner.
- priority takeover Python/VESC Tool sesuai policy.
- runtime recovery setelah TCP close <1 s dan command tetap zero sampai fresh owner command datang.

### Phase 12 — Non-actuating end-to-end hardware certification

Urutan test wajib:
1. F103 ST-Link attach/reflash/re-attach.
2. F411 CDC upload/boot/PONG.
3. F411<->F103 read-only FW_VERSION 50 Hz.
4. selective values LEFT 50 Hz.
5. selective values RIGHT 50 Hz.
6. dual selective values + zero SET_POS/SET_RPM batch 50 Hz.
7. ROS minimal stack 30 menit.
8. ROS + full HMI redraw stress.
9. ROS + GNSS/MAG/HMI traffic.
10. ROSWeb localhost:5000 open + SSE 50 Hz.
11. TCP maintenance enter/exit while stationary.
12. forced F411 UART recovery and forced F103 UART error recovery.

Tidak boleh masuk actuator test bila salah satu gate berikut gagal:
- fault != 0.
- feedback stale.
- frame/CRC errors bertambah pada clean test.
- queue/drop counters bertambah monoton.
- unexpected recovery terjadi.
- Vbus/current/encoder state tidak masuk akal.

### Phase 13 — Staged actuator certification

Semua test dengan roda/steering area aman, physical E-stop terjangkau, current limit konservatif, dan watchdog aktif.

LEFT steering sequence:
- 0 deg center verify.
- -5, +5.
- -10, +10.
- -20, +20.
- -30, 0, +30, 0.
Di setiap titik catat command, physical feedback, raw encoder, current, duty, fault, Vbus, age, dan settling time. Jangan lanjut bila arah terbalik, overshoot tidak aman, encoder tidak sinkron, atau feedback stale.

RIGHT drive sequence:
- 0 ERPM verify.
- 500 ERPM.
- 1000 ERPM.
- 2000 ERPM.
- 4000 ERPM.
- 6000 ERPM.
- 8000 ERPM hanya bila seluruh tahap sebelumnya stabil.
- kembali 0 di antara kelompok test.

Di setiap RPM catat target/actual RPM, current motor/input, Iq/Id, duty, Vbus, fault, feedback rate/jitter, Hall/encoder validity, timeout/recovery counters.

Combined Ackermann:
- steering +/-5 pada 1000/2000 ERPM.
- +/-10 pada 2000/4000 ERPM.
- larger steering/speed only after thermal/current margin known.

E-stop test pada low-energy condition dahulu: verify MOE/output stop, stale queue discarded, dan release tidak menyebabkan command replay.

### Phase 14 — localhost:5000 motion verification

Setelah Phase 13 PASS, jalankan command melalui path pengguna sebenarnya:
`Browser -> POST localhost:5000 -> /cmd_vel/teleop -> mux/Ackermann -> runtime batch -> F411 -> F103 -> motor`.

Test matrix:
- web STOP idle.
- web steering low angle.
- web 2000 ERPM equivalent command.
- combined steering + drive.
- browser refresh saat command active: watchdog harus fail closed jika producer berhenti.
- SSE disconnect: control tidak boleh terpengaruh.
- HMI page switch saat motion: telemetry tidak boleh stale.
- TCP maintenance request saat motion harus ditolak.

Verifikasi data UI terhadap ROS CLI capture dan raw VESC feedback; angka browser tidak boleh sekadar echo command.
### Phase 15 — Soak and recovery certification

Run minimum:
- 30 min zero-command full stack.
- 30 min 50-Hz low-energy motion cycling.
- 30 min HMI page/redraw stress + GNSS/MAG.
- 30 min localhost:5000 SSE + graphs open.
- repeated TCP maintenance sessions.
- controlled F411 USB reconnect.
- controlled F103 UART corruption/recovery.

Track monotonic deltas, not lifetime counters. Release requires zero unexpected recoveries and zero safety-critical drops during clean soak.

## 11. Acceptance metrics

### F103
- FOC deadline miss: 0.
- ISR overrun: 0.
- UART RX queue drop: 0 normal.
- UART TX queue drop: 0 runtime normal.
- CRC error delta: 0 clean link.
- fault code: 0 normal.
- stale command replay: 0.

### F411
- runtime TX queue high-water bounded and returns to zero/near-zero every tick.
- runtime stale supersede allowed but target steady-state = 0 occurrences.
- UART start error: 0 normal.
- RX overflow: 0.
- USB P1 motor drop: 0.
- unexpected recovery: 0.
- transport service max gap <5 ms.

### ROS
- runtime batch: 50 Hz ±1% steady.
- LEFT feedback: 50 Hz class.
- RIGHT feedback: 50 Hz class.
- FOC telemetry: 50 Hz class.
- p99 end-to-end feedback gap <30 ms target; hard normal max <50 ms.
- command watchdog stop verified.

### Web
- SSE timer 20 ms healthy, sequence gaps 0 normal.
- displayed data age <100 ms typical, <250 ms required for LIVE.
- browser refresh/disconnect does not alter actuator transport.
## 12. Revert strategy: jangan rollback seluruh sistem

Tidak disarankan kembali penuh ke `f411_pio_arduino`, karena fitur terbaru yang bernilai akan hilang: resident native bootloader, richer diagnostics, maintenance lease/arbitration, latest-value protection, dan integration terbaru.

Strategi yang benar adalah **behavioral backport** dari Arduino ke native:
- tiru TX self-chaining yang proven.
- tiru prinsip UART lifecycle yang selesai sebelum re-init.
- tiru efisiensi bulk TFT_eSPI / naikkan native renderer ke level setara.
- pertahankan native buffers yang lebih besar, resident bootloader, explicit diagnostics, dan safety additions.

Dengan kata lain: gunakan implementasi lama sebagai oracle perilaku, bukan sebagai source tree yang direvert mentah.

## 13. Commit strategy untuk perbaikan

Buat commit kecil dan bisectable:
1. `f4: add transport counters and host regression`
2. `f4: self-chain uart tx and synchronous recovery`
3. `f4: latest-value runtime queue without frame truncation`
4. `f4: isolate motor usb priority class`
5. `f4: make display transport-cooperative`
6. `f4: optimize native display bulk rendering`
7. `f103: single-owner uart dma recovery`
8. `f103: eliminate signed-shift ub`
9. `f103: fail-closed persistent config transactions`
10. `f103: atomic motor command publication and unified owner state`
11. `ros: formalize 700-byte app contract and latest runtime qos`
12. `ros: adaptive maintenance telemetry budget`
13. `web: expose source-rate/latency transport health`
14. `test: add full end-to-end 50hz hardware certification`

Setiap commit harus build + targeted test sebelum lanjut. Jangan gabungkan UI redesign, perception, navigation tuning, dan motor transport dalam commit yang sama.

## 14. Immediate execution order

Urutan paling efektif dari kondisi sekarang:
1. Freeze patches dan baseline.
2. Validasi candidate F4 fixes yang sudah ada dengan zero-command 50-Hz hardware test.
3. Jika rate belum 50 Hz, instrument service gap dan display render duration sebelum patch tambahan.
4. Selesaikan F411 transport sampai read-only dual feedback 50 Hz stabil saat display stress.
5. Baru finalisasi F103 working-tree safety fixes dan reflash.
6. Jalankan non-actuating certification lengkap.
7. Jalankan staged LEFT/RIGHT actuator tests.
8. Terakhir verifikasi gerakan melalui localhost:5000 dan soak full stack.

Tidak ada alasan teknis untuk menaikkan F103/F411 UART dari 115200 sebelum langkah 1-4 membuktikan kebutuhan bandwidth. Target robust harus dicapai dahulu pada konfigurasi wire yang sudah terbukti kompatibel.
## 15. Additional old-vs-new maintenance finding

Pada `f411_pio_arduino` lama, saat `gVesc.maintenanceMode()` aktif, firmware menjalankan empat extra VESC/USB/safety polls lalu melakukan early return dari loop. Ini secara efektif memberi maintenance UART prioritas penuh dan menunda Neo3/display work.

Native F4gateway menjalankan extra maintenance polls tetapi kemudian tetap melanjutkan Neo3/HMI path. Untuk 65101/65102, restore behavioral invariant lama:
- maintenance mode harus menjalankan high-priority safety + USB + VESC service loop.
- GNSS/MAG dapat tetap diterima IRQ/ring tetapi parsing/display update boleh didefer.
- TFT refresh dibatasi atau dihentikan selama active TCP bulk transaction.
- watchdog heartbeat tetap diperbarui.
- setelah maintenance exit, best-effort sensors kembali normal tanpa burst replay yang mengganggu motor transport.

Acceptance: MCCONF/APPCONF/rotor/value polling melalui 65101 dan 65102 tidak menunjukkan packet timeout walau HMI dan sensor hardware tetap terhubung.
