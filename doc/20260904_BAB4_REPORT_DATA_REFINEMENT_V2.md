# Penyempurnaan Akuisisi Data Laporan BAB IV V2

Tanggal: 2026-09-04
Target: `$AGV_ROOT/src/navigation/web`

## Tujuan
Bagian ungu **LAPORAN BAB IV** dipisahkan secara konseptual dari workspace tuning Navigasi. Workspace tuning lama tetap dipertahankan dan tidak diubah. Bagian laporan hanya menampilkan tabel, grafik, serta metrik yang diperlukan untuk menjawab rumusan masalah laporan.

Prinsip utama yang diterapkan adalah **raw data boleh tetap tersedia untuk audit, tetapi grafik utama laporan tidak boleh sekadar menduplikasi semua grafik tuning**.

## Struktur Laporan
Tetap digunakan 12 subjudul dengan tiga subjudul pada setiap pembahasan utama:
- 4.1.1 GNSS, 4.1.2 IMU, 4.1.3 Odometri kendaraan.
- 4.2.1 temporal EKF lokal, 4.2.2 uncertainty/rejection, 4.2.3 evaluasi final.
- 4.3.1 temporal/GNSS availability, 4.3.2 covariance/measurement validation, 4.3.3 evaluasi final.
- 4.4.1 kinematika Smac, 4.4.2 search/costmap, 4.4.3 evaluasi final.

## Perubahan Utama
`navigationReportBlueprints()` sekarang memiliki schema tabel, grafik, dan `liveSeries` khusus laporan. Source tuning hanya dipakai untuk mewarisi parameter yang relevan, bukan untuk menyalin seluruh grafik sumber.

Pada 4.3, DOP, hAcc, satelit, dan scatter GNSS tidak lagi semuanya diulang sebagai grafik utama. 4.3 difokuskan pada availability, continuity, residual, covariance, NIS proxy, quality gate, trajectory GNSS-vs-EKF, dan position jump.
## Metrik Tambahan
Frontend laporan menambahkan metrik turunan untuk kebutuhan rekap: GNSS quality-gate pass, GNSS availability, freshness/continuity EKF lokal-global, residual posisi GNSS terhadap EKF global, inter-sample position jump, endpoint error global path, serta metrik costmap untuk Smac.

Grafik 4.3.3 kini mendukung overlay trajectory GNSS map dan EKF global. Grafik 4.4 memakai bentuk global path aktual dan bukan tracking trajectory controller.

Untuk Smac, tersedia estimasi minimum clearance dan lethal-cell validity terhadap raster global costmap yang diterima Web HMI. Perhitungan ini merupakan **proxy berbasis raster costmap/downsampling**, bukan pemeriksaan footprint polygon pada OccupancyGrid mentah.

Planning time yang tersedia di Web HMI saat ini dihitung sebagai **client-observed goal-to-plan latency**, yaitu selang sejak request goal dikirim dari web hingga `/plan` baru diterima. Nilai ini tidak boleh disamakan dengan waktu komputasi internal algoritma planner apabila diperlukan benchmark internal murni.

## Akuisisi dan Ekspor
Mekanisme START/STOP, sampling satu row per detik, CSV template, RAW backup, Save Excel, snapshot parameter YAML, dan Download PNG tetap memakai mesin akuisisi yang telah tersedia. Parameter tuning pada Excel tetap dibedakan dari data pengukuran oleh exporter.

Subjudul evaluasi final 4.2.3, 4.3.3, dan 4.4.3 tetap bersifat freeze/read-only terhadap parameter tuning agar validasi memakai konfigurasi akhir.

## Verifikasi
- `node --check src/navigation/web/static/app.js` : PASS.
- `gui_bab4_acquisition_self_check.py` : PASS 16 checks.
- `web_gui_self_check.py` : PASS.
- `git diff --check` untuk source web : PASS.
- `colcon build --packages-select navigation --executor sequential` : PASS.
- `agv_web_gui` berhasil aktif pada `127.0.0.1:5000` setelah restart.
- HTTP root menyajikan cache key `bab4-report-v2` dan `/api/state` merespons ROS server aktif.

Tidak ada parameter YAML kendaraan/Nav2 yang diubah pada pekerjaan ini.