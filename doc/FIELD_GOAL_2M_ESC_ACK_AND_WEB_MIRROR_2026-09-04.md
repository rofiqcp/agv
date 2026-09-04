# Field Goal 2 m, ESC ACK, dan ROS Web Mirror — 2026-09-04

## Tujuan
- Validasi GNSS + IMU outdoor sebelum goal pose 2 m.
- Menjalankan Nav2 hanya bila ESC ACK/READY benar-benar valid.
- Membuat preview kamera ROS Web benar-benar mirror horizontal.

## Validasi localization
- GNSS menggunakan UBX NAV-PVT, 16 satelit pada pengujian lapangan.
- Heading sebelumnya sudah dikalibrasi terhadap utara dengan IMU yaw offset -0.145822259 rad.
- Nav2 dan motion-localization berhasil mencapai READY pada pengujian ini.
- Goal planner 2 m sebelumnya terbukti dapat direncanakan oleh SmacPlannerHybrid.

## Temuan ESC
- Sebelum restart ROS, feedback firmware masih live (`protocol_fb=-9.39 deg`).
- Sesudah shutdown, PL2303 tetap terhubung tetapi firmware ACK menjadi stale (`seq=0`).
- `/esc/feedback_valid=false`, `/esc/ready=false`, `/esc/armed=false`.
- Karena itu `/system/autonomy_motion_allowed=false`; goal fisik TIDAK dipaksa.
## Perbaikan
- ROS Web mirror diterapkan pada preview utama, preview eksperimen, dan overlay kalibrasi.
- Koordinat klik homography dibalik kembali ke koordinat pixel asli agar kalibrasi tidak rusak.
- Safe shutdown ESC diubah: restart ROS mengirim drive=0 tanpa melatch firmware E-STOP.
- Runtime E-STOP tetap fail-closed; perubahan hanya mencegah E-STOP persisten akibat shutdown normal.
- Paket `esc` dibuild dengan `-j2` pada host 4 core (<=70% CPU).

## Recovery yang diuji
- Disable/re-enable worker serial: ACK tetap stale.
- Pulse DTR/RTS PL2303: adaptor mengembalikan I/O error.
- Respawn `esc_ackermann` ke binary baru tanpa menjalankan shutdown handler lama: ACK tetap stale.
- USB reset host memerlukan hak root; tidak dipaksa dari proses RDC biasa.

## Kondisi akhir aman
- Stack dikembalikan ke konfigurasi produksi, bukan commissioning sementara.
- Nav2/localization tetap berjalan tetapi motion actuator tertutup selama ESC ACK belum pulih.
- Untuk melanjutkan goal fisik diperlukan reset daya/MCU ESC sampai ACK kembali fresh.