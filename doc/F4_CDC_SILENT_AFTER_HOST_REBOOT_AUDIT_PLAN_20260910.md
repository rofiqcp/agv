# F4 CDC Silent-after-Host-Reboot Audit & Recovery Plan

Tanggal: 2026-09-10
Target: `/home/sirobo/agv/F4gateway` + `/home/sirobo/agv/src/stmf4`
Status audit: forensik selesai, implementasi belum dilakukan.

## Ringkasan kejadian
- NUC boot sebelumnya tercatat berakhir tidak bersih (`last -x`: `crash`), lalu boot baru mulai 15:26 WIB.
- Kernel boot baru langsung mengenumerasi F411 sebagai `0483:5740 BLACKPILL_F411CE CDC in FS Mode` pada `/dev/ttyACM0`.
- Device node ada dan class CDC terbentuk, tetapi `PING`, `VESC:STATUS`, `VESC:MODE:RUNTIME`, dan `BOOT:PING` tidak menghasilkan satu byte balasan.
- F411 bukan resident BOOT CDC (`5741`) dan bukan ROM DFU; descriptor menunjukkan aplikasi runtime `5740`.
- Host tidak memiliki hak non-root untuk USBDEVFS reset karena installed udev rule tertinggal dari rule repository.

## Root cause utama — USB wrapper TX state tidak mengikuti USB session lifecycle
`UsbCdcPort::poll()` berhenti segera jika `tx_busy_ == true`.
`tx_busy_` hanya dibersihkan oleh `begin()`, `end()`, kegagalan `USBD_CDC_TransmitPacket()`, atau `onTransmitComplete()`.
Jika host/xHCI reset ketika IN transfer sedang aktif, callback transmit-complete untuk transfer lama dapat hilang.
STM32 USB CDC middleware menginisialisasi ulang internal `hcdc->TxState=0` saat class init, tetapi wrapper `gUsb.tx_busy_` tidak ikut direset.
`CDC_Init_FS()` hanya memasang Tx/Rx buffer dan `CDC_DeInit_FS()` kosong; tidak ada callback ke `UsbCdcPort` untuk menandai pergantian session.
Akibatnya device dapat ter-enumerasi normal dan menerima OUT, sementara seluruh TX wrapper terkunci permanen.
## Root cause kedua — watchdog hanya memantau main-loop, bukan forward progress USB
`gMainLoopHeartbeatMs` diperbarui setiap iterasi main loop dan juga di maintenance loop.
Saat `tx_busy_` macet, `gUsb.poll()` hanya return; main loop tidak block.
TIM11 watchdog 3.5 s karena itu tetap melihat heartbeat sehat dan tidak pernah me-reset MCU.
Kesimpulan: application watchdog saat ini mendeteksi CPU/main-loop stall, tetapi tidak mendeteksi subsystem deadlock.

## Root cause ketiga — host bridge tidak punya timeout untuk koneksi yang silent sejak awal
`stmf4_hmi_bridge::openSerial()` membuka tty, mengirim `ROS:1`, `PING`, `GET:STATE`, lalu menyetel `last_rx_={}`.
Heartbeat hanya menutup serial jika `last_rx_` pernah non-zero dan kemudian melewati timeout.
Jika F411 tidak pernah mengirim byte pertama, kondisi timeout tidak pernah true.
`reconnectTick()` juga tidak mencoba reopen karena `fd_ >= 0`.
Akibatnya host dapat mempertahankan file descriptor silent tanpa batas.

## Faktor pemicu
- NUC sebelumnya mengalami unclean reboot/hard crash, bukan clean shutdown biasa.
- F411 kemungkinan tetap memperoleh daya/VBUS sehingga MCU tidak cold-reset saat host USB controller restart.
- Runtime normal membawa traffic TX kontinu; peluang ada IN packet aktif tepat saat host hilang tinggi.
- Bug bukan baru dari commit HMI terakhir: pola `tx_busy_` yang sama sudah ada sejak commit awal gateway. Priority/message-atomic meningkatkan traffic, tetapi bukan akar lifecycle bug.

## Gap recovery host
Rule repository mencakup USB `0483:5740`/`5741`, tetapi `/etc/udev/rules.d/99-blackpill-stm32.rules` hanya memasang tty rule `5740` dan tidak memasang USB-device rule runtime/boot.
Akibatnya `/dev/bus/usb/...` tetap `root:root`; user `sirobo` tidak dapat menjalankan `usbreset` sebagai fallback.
## Tahap 1 — mandatory root-cause fix
1. Tambahkan lifecycle hook pada `UsbCdcPort`: `onUsbClassInit()` dan `onUsbClassDeInit()`.
2. Panggil hook dari `CDC_Init_FS()` dan `CDC_DeInit_FS()`.
3. Pada pergantian USB session, atomically clear `tx_busy_`, `tx_pending_`, active-priority/message flags, serta drop RX/TX queue lama agar tidak ada replay command/telemetry lintas host session.
4. Tambahkan `usb_session_generation`, `usb_class_init_count`, `usb_class_deinit_count`, `tx_abort_on_session_reset`, dan `tx_complete_count` untuk observability.
5. Track `tx_started_ms`; bila `tx_busy_` tidak mendapat completion dalam bounded threshold (rekomendasi awal 250 ms), tandai `usb_tx_stall_pending`.
6. Recovery stall harus dieksekusi dari main context, bukan USB ISR. Prioritas recovery: wrapper-state repair bila ST `TxState==0`; bila ST class juga stuck, lakukan controlled USB soft re-enumeration.
7. USB recovery wajib drop stale host queues dan mempertahankan motor link fail-closed; command runtime lama tidak boleh diputar ulang setelah host kembali.
8. Tambahkan command idempotent `USB:RECOVER`/set recovery-pending sehingga host bisa meminta USB soft restart tanpa bergantung pada TX ACK.
9. Perbaiki `stmf4_hmi_bridge`: simpan `serial_opened_at_`; jika tidak ada first valid RX dalam `hmi_transport_timeout_sec`, close dan retry walau `last_rx_` masih epoch.
10. Setelah beberapa silent-open berturut-turut, publish diagnostic escalation dan tetap fail-closed.
11. Sinkronkan/install udev rule runtime+boot yang exact terhadap VID:PID dan serial BlackPill agar identity-scoped `usbreset` tersedia sebagai recovery lapis terakhir.
12. Jangan gunakan generic ttyUSB/ttyACM reset; hanya serial `338133833134` F411 yang boleh ditarget.

### Acceptance Tahap 1
- Fresh boot: `PING -> ACK:PONG` konsisten.
- Runtime 50 Hz: P1 drop, runtime queue drop, CRC/format error delta = 0.
- Putus/replug USB data saat traffic aktif: recovery otomatis tanpa NRST, stale command tidak replay.
- Reboot NUC saat telemetry aktif: F411 harus merespons PING <=2 s setelah host membuka CDC.
- 20 siklus host reboot/re-enumeration tanpa satu pun kondisi CDC silent permanen.
## Tahap 2 — hardening dan certification
1. Tambahkan USB forward-progress health ke `VESC:STATUS`/system diagnostics: busy age, last TX complete age, session generation, recoveries, RX/TX queue depth, and reset cause.
2. Pisahkan watchdog menjadi CPU-liveness dan subsystem-progress. CPU watchdog tetap TIM11; USB progress watchdog tidak boleh bergantung pada main-loop heartbeat saja.
3. Tambahkan fault-injection self-check untuk kondisi `tx_busy_=true` lalu CDC DeInit/Init; state wrapper wajib kembali idle dan queue lama wajib dibuang.
4. Tambahkan host test yang membuka tty tetapi menerima nol byte; bridge wajib close/retry dalam bounded time.
5. Uji urutan power: F411 dulu lalu NUC, NUC dulu lalu F411, F411 reset saat host aktif, NUC reboot saat F411 powered, suspend/resume host, cable disconnect/replug.
6. Uji maintenance: Python 65101 dan VESC Tool 65102 aktif saat USB host hilang; setelah reconnect ownership harus kembali fail-closed, tanpa replay command.
7. Uji bootloader: runtime->BOOT CDC->runtime 20 siklus dan pastikan session generation/reset tidak meninggalkan tx_busy stale.
8. Uji HMI redraw/GNSS/MAG bersamaan dengan runtime 50 Hz setelah setiap reconnect.
9. Tambahkan systemd/user startup untuk ROS ESC bridge bila target produk memang harus pulih otomatis setelah NUC reboot; gunakan by-id F411, Restart=always, dan readiness berdasarkan first valid F411 response.
10. Audit terpisah penyebab NUC unclean reboot jika pola `last -x: crash` terus berulang; F411 error adalah konsekuensi setelah reboot, belum ada bukti F411 menyebabkan NUC crash.

### Acceptance Tahap 2
- 50 USB reconnect/reset cycles + 20 host reboot cycles tanpa physical reset F411.
- First valid F411 response p99 <2 s setelah CDC tersedia.
- Motor command selalu zero selama host/session transition sampai feedback dan safety gates kembali valid.
- Runtime TX 50 Hz ±1%; LEFT/RIGHT feedback 50 Hz class; P1 drop=0; qdrop=0; uart error=0; unexpected recoveries=0 pada steady state.
- Tidak ada stale ROS/HMI/VESC command yang muncul setelah session generation berubah.
- Udev fallback reset hanya dapat menarget exact BlackPill serial, bukan sensor/IMU/perangkat STM lain.

## Kesimpulan
Akar paling kuat bukan kerusakan hardware F411. Gejala sesuai deadlock state software antara ST USB CDC class yang sudah re-init dengan wrapper `UsbCdcPort` yang masih menganggap transfer lama `tx_busy`. Host bridge kemudian memperparah keadaan karena tidak memiliki first-response timeout. Perbaikan harus dilakukan di kedua sisi; menambah reset fisik saja hanya menyembunyikan lifecycle bug.
