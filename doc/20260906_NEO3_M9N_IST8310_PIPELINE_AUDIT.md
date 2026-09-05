# Audit NEO-3 M9N + IST8310 ke ROS2 dan ROS Web — 2026-09-06

## Tujuan
Memastikan aliran data CUAV NEO-3 benar dari hardware F411 sampai localization/EKF dan ROS Web, termasuk perilaku indoor saat GNSS terhubung tetapi belum mendapatkan fix.

## Kontrak hardware
- CUAV NEO-3 memakai GNSS u-blox NEO-M9N dan compass IST8310.
- Modul menyediakan UART + I2C + IO.
- Jalur proyek: M9N UART -> F411 USART2 PA3(RX)/PA2(TX).
- Jalur proyek: IST8310 I2C -> F411 I2C1 PB8(SCL)/PB9(SDA), 400 kHz.
- USB CDC F411 menjadi satu-satunya transport ke ROS host.

## Firmware F411
- M9N dibaca UBX NAV-PVT; UART 38400 baud dan target navigation epoch 10 Hz.
- `gnssAlive` berarti receiver benar-benar streaming, tidak sama dengan `gnssReady`/fix.
- IST8310 address 0x0E, WHO_AM_I 0x10, resolusi 0.30 uT/LSB.
- IST8310 di-soft-reset sebelum WHO_AM_I, lalu konfigurasi averaging 0x24 dan pulse 0xC0 diverifikasi read-back.
- Raw range guard diterapkan (XY sekitar ±1600 uT, Z sekitar ±2500 uT) dan Z dibalik agar right-handed.
- Firmware F411 di-upload sukses melalui USB CDC -> `BOOT:DFU` -> STM32 ROM DFU; ST-Link tidak digunakan untuk F411.

## Data real indoor setelah firmware terbaru
Frame F411 selama pengujian menunjukkan:
- `SENS:HW`: `gnss_alive=1`, `gnss_ready=0`, `ist8310=1`.
- M9N NAV-PVT: 10.0 Hz, `fix_type=0`, `satellites=0`, sehingga CONNECTED tetapi NO FIX.
- Tidak ada `/gnss/vel` yang diterbitkan ketika fix belum qualified.
- `/gnss/quality` tetap diterbitkan: source=1, sat=0, DOP=99.99, hAcc clamped 1000 m, rate=10 Hz, `gnss_fix_ok=false`.
- IST8310 raw ROS sample: X=-3.6 uT, Y=42.3 uT, Z=58.2 uT, norm=72.04 uT.
- `/neo3/mag` rate terukur sekitar 16.8 Hz pada runtime USB/ROS saat audit.
- `neo3_mag_heading_valid=true`; contoh yaw map=-0.0621 rad (~-3.56 deg), variance=0.06 rad².
- Magnetometer IMU saat indoor sekitar 912 uT dan ditolak otomatis oleh `field_norm_gate`, sehingga tidak mencemari heading.

## Aliran algoritma yang tervalidasi
1. F411 menerbitkan `/gnss/quality`, `/gnss/state`, `/gnss/fix_raw` hanya jika LLH tersedia, dan `/neo3/mag`.
2. `mag_heading_fusion` memakai `/neo3/mag` + roll/pitch `/imu/data` + `/localization/map_yaw_from_enu`.
3. Gate field norm NEO-3 lolos; gate magnetometer IMU dapat ditolak secara independen.
4. Output NEO-3 adalah `/neo3/mag_heading_fusion` dan `/neo3/mag_heading_valid`.
5. `ekf_filter_node_map` terbukti subscribe langsung ke `/neo3/mag_heading_fusion`.
6. `/odometry/filtered_map` aktif ~11-13 Hz; yaw mengikuti heading valid, sedangkan covariance X/Y tetap sangat besar ketika GNSS belum fix.

## ROS Web
Tab Sensor sekarang memisahkan dua perangkat NEO-3:
- **NEO-3 GNSS M9N**: status LINK/CONNECTED terpisah dari FIX VALID/NO FIX, satelit, hAcc, DOP, NAV-PVT rate, speed, COG dan fix type.
- Saat no-fix, lat/lon ditampilkan `--`, bukan koordinat 0.0 palsu.
- **NEO-3 Magnetometer IST8310**: I2C status, X/Y/Z uT, norm uT, map heading dan heading gate.
- Diagnostics tetap menampilkan status magnetometer IMU secara terpisah.

Web API runtime setelah rebuild menerima:
- `connected.gnss=true`
- `gnss_quality.gnss_fix_ok=false`, `fix_type=0`, `sat=0`, `pvt_rate_hz=10`
- `neo3_status.gnss_alive=true`, `gnss_ready=false`, `ist8310=true`, `parse_errors=0`
- `connected.neo3_mag=true`
- `neo3_mag_heading_valid=true`
- `magnetic_heading_status.neo3_valid=true`

## Validasi build/test
- `stmf4_gateway_self_check.py`: PASS.
- `f411_mag_pipeline_self_check.py`: PASS.
- `web_gui_self_check.py`: PASS.
- `node --check web/static/app.js`: PASS.
- `python3 -m py_compile autonomous.launch.py`: PASS.
- `colcon build --packages-select navigation`: PASS dengan maksimal 2 job dari 4 core.
- `colcon build --packages-select stmf4`: PASS dengan maksimal 2 job dari 4 core.

**Aturan proyek terbaru:** semua `colcon build` wajib dibatasi maksimal 50% CPU/RAM.
