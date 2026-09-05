# LEFT Steering Encoder AB Calibration Audit

Tanggal: 2026-09-05

## Target
- Motor LEFT memakai encoder ABI 1024 PPR / 4096 count per revolution.
- VESC Tool `Set Position`: `0 -> -30 deg`, `180 -> 0 deg`, `360 -> +30 deg`.
- ROS/Web memakai koordinat signed langsung `-30 .. 0 .. +30 deg`.
- Detect Encoder mengukur phase relation, encoder offset/ratio/inversion, hard-stop kiri/kanan, span mekanik, lalu menyimpan hasil ke EEPROM.
- Nilai raw count/span tetap diteruskan ke ROS untuk diagnosis, sedangkan sudut kendali dinormalisasi menjadi rentang fisik 60 deg.

## Perilaku boot
- Jika kalibrasi steering belum valid: LEFT tetap high-Z dan perintah posisi ditolak.
- Jika kalibrasi valid: firmware melakukan electrical alignment ABI, homing perlahan ke hard-stop kiri, rebase kiri sebagai -30 deg, lalu bergerak ke center 0 deg.
- Metadata OTA RECOVERY yang tertinggal dari update gagal harus dibersihkan agar bootloader kembali masuk aplikasi sehat; EEPROM konfigurasi/kalibrasi tidak ikut dihapus.

## Implementasi dan verifikasi
- `COMM_DETECT_ENCODER` tetap memakai bentuk reply VESC standar: offset, ratio, inverted.
- LEFT dipaksa ke mode ABI sebelum bounded phase probe agar konfigurasi legacy Hall tidak menghasilkan sentinel `offset=1001, ratio=0`.
- Hard-stop span disimpan terpisah dari MC configuration dengan magic + complement untuk fail-closed terhadap torn write.
- ROS `/esc/foc/telemetry` menyediakan `steering_cal`: calibrated, homed, encoder_synced, raw_left, raw_right, raw_center, raw_now, raw_target, dan physical -30/0/+30 deg.
- Build APP_STLINK: PASS, verify flash: PASS.
- `tools/run_all_checks.py`: ALL_FINAL_HOST_CHECKS_PASS.
- `src/esc/test/esc_runtime_self_check.py`: PASS.
- `src/navigation/test/steering_physical_part1_self_check.py`: PASS.

## Catatan hardware aktif
Eksekusi remote yang benar-benar menggerakkan steering ke hard-stop untuk menyelesaikan pengukuran raw span diblokir oleh safeguard alat pada sesi ini. Karena itu nilai hard-stop aktual belum boleh diklaim sebagai terukur dari sesi ini. Firmware dan jalur telemetry sudah siap untuk pengujian tersebut saat aktuasi hardware diizinkan.