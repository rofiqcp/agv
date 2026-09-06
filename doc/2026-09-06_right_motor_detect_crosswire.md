# RIGHT Motor Detect / Cross-Wiring Verification — 2026-09-06

## Kondisi power dan komunikasi
- DC-link F103 terukur kembali normal: ADC baterai 1619 count, sekitar 43.08 V.
- VESC virtual ID2 (`motor_right`) merespons FW 6.00, fault=0 setelah supply motor aktif.
- F411↔F103 berjalan 1,000,000 baud melalui jalur maintenance VESC.

## Hasil Detect Hall RIGHT
- `COMM_DETECT_HALL_FOC` pada 1.0 A selesai tanpa fault/overcurrent tetapi gagal validasi.
- Reply: hanya Hall state 0 terobservasi; tabel detect menjadi `[0, FF, FF, FF, FF, FF, FF, FF]` dengan status fail.
- Sweep open-loop RIGHT 2.0 A dan 3.5 A beberapa putaran elektrik juga mempertahankan Hall RIGHT=0 sepanjang pengujian.

## Electrical Hall pin test
- PC10/PC11/PC12 adalah RIGHT Hall U/V/W sesuai firmware lama dan firmware VESC saat ini.
- Diagnostic high-Z menguji kondisi asli, internal pull-down, dan pull-up.
- Hasil raw: ORIGINAL=111, PULLDOWN=111, PULLUP=111.
- Artinya ketiga line tidak floating; ada rangkaian eksternal/pull-up kuat yang menahan HIGH, tetapi Hall tidak menghasilkan urutan 6-state valid saat RIGHT bridge digerakkan.

## Cross-bridge proof
- Saat RIGHT power bridge digerakkan, Hall RIGHT tidak berubah.
- Saat LEFT power bridge digerakkan 2.0 A dengan phase sweep, Hall RIGHT berubah nyata: state yang terlihat `0,1,2,3`.
- Ini membuktikan motor yang memiliki Hall RIGHT saat ini secara fisik digerakkan oleh LEFT power bridge.
- Firmware lama (`a39abbc`) maupun firmware VESC saat ini sama-sama memetakan LEFT→TIM8 dan RIGHT→TIM1; jadi cross-link berasal dari wiring phase motor antar bridge, bukan perubahan logical mapping VESC.

## Safety/commissioning improvement
- Firmware ditambah bounded open-loop commissioning command yang dapat dipakai pada kedua motor, maksimum 2 A, maksimum 20 ERPM, maksimum 1 s, dengan auto-release dan fault guard.
- Firmware ditambah passive RIGHT Hall electrical test; test hanya diizinkan ketika motor OFF dan mengembalikan konfigurasi GPIO setelah sampling.
- Production closed-loop tetap fail-closed; diagnostic tidak mengubah requirement Hall/encoder valid.

## Wiring yang harus kembali sebelum Detect All RIGHT
- Tiga phase motor drive RIGHT harus kembali ke output bridge RIGHT/TIM1 sebagai satu grup.
- Hall motor drive RIGHT tetap ke konektor RIGHT Hall PC10/PC11/PC12.
- Phase steering LEFT harus ke bridge LEFT/TIM8; feedback steering tetap encoder PB6/PB7.
- Urutan U/V/W di dalam satu motor boleh salah karena Hall/FOC detect dapat menentukan phase relation; yang tidak boleh adalah power bridge motor A dipasangkan dengan feedback sensor motor B.
