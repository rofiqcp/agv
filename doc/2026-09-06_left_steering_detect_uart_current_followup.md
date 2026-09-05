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
