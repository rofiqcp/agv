# LEFT steering detect — UART/current follow-up

Tanggal: 2026-09-06

## Bukti hardware yang sudah PASS
- F103 APP_STLINK build, flash, dan verify berhasil; VESC FW_VERSION membalas `6.00 / motor_left` melalui F411 pada 1 Mbaud.
- Frame DMA F103 terbukti berisi tepat request FW_VERSION 6 byte dan request `COMM_DETECT_ENCODER=27` 10 byte dengan CRC valid.
- Race RX ditemukan: `usart3_rx_check()` dipanggil dari main loop dan USART3 IRQ dengan `oldPos` statik yang sama, sehingga frame valid dapat diproses dua kali dan tercatat sebagai CRC error.
- RX DMA kini hanya didrain dari main loop; USART3 IRQ hanya clear IDLE. Setelah fix, detect tidak lagi hilang selama 50 s dan reply menjadi deterministik.
- Sebelum koreksi current baseline, detect 0.70 A gagal `encoder_stage=0xA1` dengan satu trip phase-only; DC-link tidak trip.
- Sampel trip nyata: phase A=218, B=217, C=-435 count, DC=-20 count, duty=-361 permille; skala 50 count/A menunjukkan common-mode phase shift sementara DC hanya sekitar 0.4 A.

## Perbaikan current baseline
- Firmware sekarang mempelajari baseline ADC saat bridge ON selama window zero-vector `MCCONF_BRIDGE_SETTLE_SAMPLES=80` sebelum sampel boleh masuk FOC/proteksi.
- Batas safety tetap utuh: open-loop phase dan DC-link masing-masing tetap 8 A; proteksi tidak dinonaktifkan.
- Setelah fix, detect 0.70 A berjalan sekitar 5.35 s tanpa current trip; detect 1.00 A juga tanpa current trip.
- Tahap kegagalan maju menjadi `encoder_stage=0xA9`: ramp/hold dan semua gerakan +60/0/-60/0 selesai, tetapi validasi arah/amplitudo ABI gagal.

## Bukti encoder saat ini
- TIM4 real hardware aktif: encoder mode TI12, `CR1=1`, `SMCR=3`, `ARR=4095`, counter 0.
- AFIO `TIM4_REMAP=0`, sehingga input timer benar berada di PB6/PB7.
- GPIO aktual saat idle: PB6=HIGH, PB7=LOW; pin tidak floating.
- Pada detect 0.70 A dan 1.00 A: origin count=0, plus count=0, minus count=0; encoder belum menghasilkan count terukur.
- Firmware `9888ed5` menambahkan penghitung edge GPIO A/B dan snapshot Id/Iq pada probe agar pengujian fisik berikutnya dapat membedakan rotor tidak bergerak vs sinyal encoder tidak berubah.

## Status
- `tools/run_all_checks.py`: `ALL_FINAL_HOST_CHECKS_PASS`.
- Firmware commit/push: `9888ed5 fix: stabilize steering detect uart and current baseline`.
- Kalibrasi hard-stop, center, dan pembuktian fisik -30/0/+30 belum dinyatakan PASS karena belum ada edge ABI terukur.
- Safeguard remote menolak aktuasi detect berikutnya setelah instrumentasi edge terbaru; tidak dilakukan bypass.

## Manual encoder pin diagnostic follow-up
- Firmware checkpoint `a0d3bda` menambahkan edge counter PB5/PB6/PB7 dan menginisialisasi level awal dari GPIO aktual agar tidak ada false first-edge setelah boot.
- Pole motor dipastikan kembali ke identitas hardware: LEFT 4 pole-pair (8 poles), RIGHT 15 pole-pair (30 poles), dengan migrasi speed EEPROM yang mempertahankan kecepatan mekanik.
- `tools/run_all_checks.py`: `ALL_FINAL_HOST_CHECKS_PASS`.
- APP_STLINK flash + verify: PASS.
- FW probe setelah boot: VESC FW 6.00 / `motor_left` PASS.
- Baseline hardware sesudah reboot: PB5 edge=0, PB6 edge=0, PB7 edge=0, TIM4 CNT=0. Baseline ini siap untuk uji putar manual berikutnya.

## Sweep manual hard-stop kiri ke kanan
- Baseline hard-stop kiri: TIM4=4089, PB6 edge=8, PB7 edge=7, PB5 edge=7.
- Setelah pengguna memutar manual sampai hard-stop kanan: TIM4=4089, PB6 edge=8, PB7 edge=7, PB5 edge=7.
- Delta kiri->kanan: TIM4=0 count, PB6=0 edge, PB7=0 edge, PB5=0 edge.
- Karena target hardware adalah ABI 1024 PPR / 4096 count/rev, hasil span nol membuktikan sinyal steering aktual belum mencapai PB6/PB7 selama sweep mekanik penuh.
- PB5/PB6/PB7 adalah jalur Hall LEFT asli; firmware memang meminjam PB6/PB7 untuk mode ABI, sehingga wiring fisik harus benar-benar membawa A/B encoder ke PB6/PB7.
- Closed-loop steering, hard-stop calibration, dan mapping -30/0/+30 tetap fail-closed sampai edge ABI aktual terbukti mengikuti gerak steering.

## Konfirmasi wiring Hall LEFT masih terhubung — 2026-09-06
- Runtime F103 sudah benar: sensor_port_mode=ABI, foc_sensor_mode=ENCODER, encoder_counts=4096, LEFT poles=8, encoder_configured=1, TIM4 quadrature aktif; encoder_synced=0 karena belum ada ABI motion valid.
- Tes pasif GPIO PB6/PB7: saat internal pull-up diubah menjadi pull-down, PB6 dan PB7 tetap HIGH. Artinya net dipertahankan oleh rangkaian eksternal, bukan hanya pull-up internal STM32.
- Tes pasif PB5: pin diubah sementara dari floating input menjadi input-pullup, tetapi PB5 tetap LOW; saat pull-down juga tetap LOW. Ini membuktikan perangkat eksternal masih meng-drive PB5.
- State fisik saat tes adalah PB5/PB6/PB7 = 0/1/1, konsisten dengan state Hall 3-fasa valid. Dengan mapping board LEFT Hall U/V/W = PB5/PB6/PB7, Hall LEFT asli masih tersambung.
- Karena mode proyek LEFT adalah ABI, konektor Hall LEFT tidak boleh membawa Hall motor secara paralel. Encoder harus menggantikan fungsi konektor: pin1=GND, pin2=B->PB7/TIM4_CH2, pin3=A->PB6/TIM4_CH1, pin4=NC, pin5=VCC +5V sesuai skema proyek.
- Jangan gunakan tegangan baterai/DC bus sebagai VCC encoder. Untuk output encoder push-pull 5V, gunakan level shifter; direct input hanya untuk open-collector/open-drain atau level logika 3.3V sesuai skema proyek.
- Status tetap fail-closed: PWM LEFT high-Z dan closed-loop steering tidak diaktifkan sampai Hall LEFT dilepas dan ABI menghasilkan count nyata.
