# Panduan Pengambilan Data BAB IV dan Kalibrasi GUI Persepsi

Acuan implementasi aktual: `/home/sirobo/agv/src/navigation/web/static/perception_bab4.js`, `perception_calibration.js`, `tuning_evidence.js`, serta konfigurasi `/home/sirobo/agv/src/perception/config/astra_yolop_gpu.yaml`.
Acuan laporan: Subbab 3.12, BAB IV 4.1–4.4, dan Lampiran 2 pada laporan Tugas Akhir Davi.

> Prinsip utama: data FINAL laporan harus berasal dari pengujian aktual. Jangan mempertahankan angka DATA ESTIMASI sebagai hasil akhir.

## 1. Urutan kerja yang disarankan

1. Jalankan sistem dan pastikan kamera Astra serta YOLOPv2 CPU aktif.
2. Selesaikan kalibrasi persepsi yang diperlukan terlebih dahulu.
3. Simpan snapshot konfigurasi efektif sebelum pengambilan data final.
4. Buka **PERSEPSI → Tune** untuk masuk ke mode **FINAL BAB IV 4.1–4.4**.
5. Ikuti urutan **Pilih Tahap → Cek Source → START/CAPTURE → STOP + SAVE → XLSX/PNG**.
6. Jangan pindah subbab/experiment ketika recorder masih aktif; lakukan **STOP + SAVE** terlebih dahulu.
7. Simpan CSV/XLSX/PNG/manifest untuk setiap run sebagai evidence sidang.

## 2. Preflight sebelum kalibrasi atau pengambilan data

Pastikan pada GUI:
- Camera = ONLINE dan camera health = HEALTHY.
- YOLOPv2 CPU inference = ON.
- Backend yang dipakai untuk data laporan adalah CPU sesuai rancangan laporan final.
- `/perception/raw_detections` fresh untuk 4.2 dan 4.4.
- `/perception/lane_safety_state` fresh untuk 4.3.
- `/perception/performance` fresh untuk pengujian performa.
- Tombol START menampilkan kondisi READY, bukan BLOCKED.

### Verifikasi terminal yang berguna

```bash
source /opt/ros/humble/setup.bash
source /home/sirobo/agv/install/setup.bash
ros2 topic hz /perception/raw_detections
ros2 topic hz /perception/lane_safety_state
ros2 topic hz /perception/performance
ros2 param dump /perception
```

Simpan hasil `ros2 param dump /perception` bersama run final. Catat juga model, backend, resolusi kamera, FPS request, `cpu_inference_fps`, `cpu_threads`, dan parameter threshold yang tampil di GUI.

# BAGIAN A — KALIBRASI GUI PERSEPSI

## 3. Kalibrasi Lane Safety / Object ROI

Buka **PERSEPSI → Calibration → Lane Safety / ROI**.

GUI membaca nilai awal dari YAML authoritative melalui `/api/config`; jika muncul **WAIT CONFIG**, jangan melakukan kalibrasi sampai konfigurasi berhasil dibaca. Nilai fallback tidak dipakai sebagai candidate.

Untuk Lane Safety tersedia empat handle:
- **LT** = kiri atas
- **LB** = kiri bawah
- **RT** = kanan atas
- **RB** = kanan bawah

Handle dapat di-drag pada citra. Keyboard `1–4` memilih handle; tombol panah melakukan fine adjustment, sedangkan `Shift + panah` melakukan langkah lebih besar.

Aturan geometri yang diperiksa GUI:
- Y atas harus berada 0–0,90.
- Y bawah maksimal 0,995 dan minimal 0,05 lebih rendah dari Y atas.
- Garis kiri dan kanan tidak boleh saling silang.
- `lane_corridor_correction_gain_m_per_px` harus berada 0–0,05.
- ROI terdiri dari 4 titik normalized 0–1 dan tidak boleh degenerate/terlalu kecil.

Workflow penyimpanan:
1. Atur titik sampai overlay sesuai geometri kendaraan dan lane aktual.
2. Pastikan status bukan `INVALID` atau `YAML CHANGED`.
3. Klik **Stage + Review Calibration**.
4. Buka/review diff konfigurasi yang di-stage.
5. Lakukan **Apply** hanya setelah diff benar dan validasi PASS.
6. Reload konfigurasi lalu pastikan overlay tetap sama dengan nilai YAML aktif.

Jangan menganggap tombol Stage sebagai Apply. Implementasi saat ini sengaja memisahkan proposal, validation, dan commit ke YAML.

## 4. Kalibrasi jarak obstacle — 40 kombinasi

Buka **PERSEPSI → Calibration → Obstacle Calibration**. Wizard menggunakan `/perception/semantic_detections` untuk kalibrasi metrik; data ini berbeda dari `raw_detections` yang dipakai untuk statistik utama BAB IV 4.2.

Total kombinasi wajib wizard:
- 2 objek: **Human** dan **Motorcycle**.
- 4 orientasi: **Depan, Belakang, Samping Kiri, Samping Kanan**.
- 5 jarak ground-truth: **1, 2, 3, 4, 5 m**.
- Total = **40 capture**.

Syarat agar tombol capture aktif:
- kamera terhubung dan sehat;
- stream `semantic_detections` fresh;
- kelas yang dipilih terdeteksi dengan confidence memenuhi gate;
- objek cukup berada di tengah citra;
- titik bawah bbox tersedia;
- untuk jarak 3–5 m, bbox tidak boleh terpotong di bawah dan `raw_forward_m` harus valid.

Prosedur setiap kombinasi:
1. Ukur jarak fisik dari referensi depan kamera ke titik kontak objek pada tanah.
2. Pilih Object, Orientation, dan Distance pada wizard.
3. Posisikan satu objek utama dan tunggu bbox stabil serta status **CAPTURE READY**.
4. Klik **OK / Capture** sekali.
5. Baris yang berhasil akan menjadi **LOCK**; lanjutkan ke kombinasi berikutnya.
6. Jika capture salah, gunakan **RESET** pada baris itu saja lalu ulangi.

Setelah 40/40 lengkap, GUI membentuk fit terpisah Human dan Motorcycle. Fit metrik memakai sampel 3–5 m yang tidak clipped dan mempunyai `raw_forward_m` valid.

Urutan akhir kalibrasi obstacle:
1. Periksa grafik raw vs calibrated serta MAE yang tampil.
2. Klik **SAVE CALIBRATION**; ini membuat proposal dan menjalankan validation.
3. Setelah status **SAVED + VALIDATED**, klik **APPLY TO YAML**.
4. Ekspor **CSV Calibration** dan **PNG Graph** sebagai evidence.
5. Restart/reload runtime bila diperlukan oleh workflow configuration, lalu verifikasi koefisien aktif.

Catatan: kalibrasi obstacle metrik tidak boleh digunakan untuk “memperbaiki” jumlah berhasil/gagal pada pengujian raw YOLOPv2 4.2. Keduanya adalah jalur evaluasi berbeda.

# BAGIAN B — PENGAMBILAN DATA FINAL BAB IV

## 5. BAB IV 4.1 — Implementasi Sistem Persepsi Visual

Buka **PERSEPSI → Tune → 4.1 Implementasi**. Tahap ini merupakan audit implementasi, bukan pengujian massal.

Evidence yang harus disimpan:
- kamera Astra tampil aktual;
- YOLOPv2 inference aktif;
- backend tercatat CPU;
- raw detection, Lane Safety, dan performance telemetry tersedia;
- parameter/configuration efektif tersimpan;
- screenshot GUI yang menunjukkan kondisi runtime aktual, bukan screenshot saat YOLO OFF.

Ambil snapshot `ros2 param dump /perception` pada sesi final. Jika memungkinkan, simpan juga identitas commit/source dan model yang dipakai agar data 4.2–4.4 dapat direproduksi.

## 6. BAB IV 4.2 — Object Detection YOLOPv2

Target utama: **200 percobaan** = 2 target × 5 jarak × 20 ulangan.

Konfigurasi protokol:
- Target fisik: Human dan Motorcycle.
- Jarak fisik: 1, 2, 3, 4, 5 m.
- Ulangan: 20 untuk setiap kombinasi target-jarak.
- Beri jeda sekurang-kurangnya ±2 s antarpengamatan agar frame tidak hampir identik.
- Gunakan satu target utama pada frame evaluasi.

Definisi capture:
- **BERHASIL** bila bbox kandidat setelah NMS melingkupi sasaran fisik utama secara spasial.
- Bila ada beberapa bbox pada sasaran, pilih kandidat dengan confidence tertinggi yang memenuhi kriteria spasial.
- Bbox pada latar/objek lain tidak boleh menggantikan kegagalan sasaran.
- Kondisi meragukan dicatat sebagai **GAGAL** dan beri alasan pada Catatan Visual.
- Confidence hanya dicatat untuk kandidat BERHASIL; trial gagal jangan diisi confidence = 0.

Langkah GUI:
1. Pilih card **4.2 Object Detection**.
2. Pastikan preflight menunjukkan YOLOPv2 CPU inference dan raw detection stream siap.
3. Klik **START** sekali untuk membuka sesi evidence.
4. Pilih Target Fisik, Jarak GT, dan Ulangan.
5. Pada frame yang sudah dijadwalkan, pilih kandidat bbox yang sesuai sasaran.
6. Klik **Capture BERHASIL** atau **Capture GAGAL**.
7. GUI akan maju otomatis ke ulangan/jarak/target berikutnya; pantau progress hingga 200/200.
8. Gunakan **Undo** bila salah klik, bukan mengedit hasil diam-diam setelah sesi selesai.

Jarak ground-truth harus diukur fisik dari referensi depan kamera ke titik kontak target pada tanah. Jangan memakai estimasi homografi sebagai GT pengujian 4.2.

### Uji negatif 4.2.5

Setelah/di dalam sesi 4.2, ambil **30 frame negatif**:
- 10 frame indoor tanpa manusia/motor;
- 10 frame jalur relatif kosong;
- 10 frame latar statis dengan objek non-target.

Setiap kandidat yang muncul pada frame negatif dicatat sebagai kandidat palsu terhadap protokol target manusia/motor. Simpan catatan dan, bila perlu, screenshot/frame bukti untuk kasus false candidate.

Setelah 200 trial + uji negatif selesai, klik **STOP + SAVE**. Pastikan tabel rekap menghasilkan Detection Rate, Error, dan confidence rata-rata per target/jarak. Gunakan grafik otomatis **Detection Rate vs jarak** dan **Confidence vs jarak**, lalu ekspor PNG.

Penting: statistik 4.2 memakai `/perception/raw_detections`. Kandidat 5 m tetap boleh dihitung sebagai keberhasilan raw detection walaupun jalur metric/obstacle safety dapat menolaknya karena batas jarak, ROI, atau drivable-contact.

## 7. BAB IV 4.3 — Lane Safety

Target utama: **80 percobaan** = 4 kondisi × 20 ulangan.

Empat kondisi acuan:
- **Abu-abu**: tidak ada bukti mask yang memadai → UNKNOWN / NO_MASK_EVIDENCE.
- **Hijau**: jalur clear → kiri dan kanan GREEN, recommendation CLEAR.
- **Kuning**: lane mendekati boundary → sisi uji YELLOW, sisi lain GREEN, recommendation WARNING.
- **Merah**: lane menyentuh/masuk koridor → sisi uji RED; intrusi kiri mengharapkan RECENTER_RIGHT dan intrusi kanan RECENTER_LEFT.

Untuk kondisi Kuning dan Merah, implementasi GUI membagi sisi otomatis: ulangan **1–10 kiri**, ulangan **11–20 kanan**.

Langkah GUI:
1. Pilih card **4.3 Lane Safety** dan pastikan stream lane fresh.
2. Klik **START**.
3. Pilih kondisi acuan; untuk Kuning/Merah sisi mengikuti protokol otomatis.
4. Bentuk kondisi fisik/mask, lalu tunggu keadaan stabil.
5. Pastikan ground truth ditentukan dari penempatan aktual dan inspeksi mask, bukan sekadar menyalin warna output GUI.
6. Klik **Capture + Auto Compare** satu kali untuk frame evaluasi.
7. GUI mencatat gap kiri/kanan, warna/status kedua sisi, latch, recommendation, expected, dan hasil BENAR/SALAH.
8. Lanjutkan sampai progress 80/80 lalu **STOP + SAVE**.

### Verifikasi histeresis 4.3.5

Gunakan panel **Verifikasi Histeresis**. Urutan acuan GUI adalah:

`60 px → 25 px → 0 px → 20 px → 40 px → 50 px`

Dengan urutan status yang diharapkan:

`GREEN → YELLOW → RED → YELLOW (latched) → YELLOW (latched) → GREEN/release`.

Jalankan satu siklus penuh tanpa mengubah threshold di tengah siklus. Untuk evidence yang simetris, disarankan mengambil setidaknya satu siklus lengkap pada sisi kiri dan satu pada sisi kanan. Simpan CSV Histeresis setelah selesai.

## 8. BAB IV 4.4 — Performa Real-Time CPU

Lakukan **3 run terpisah**, masing-masing 60 detik setelah kondisi visual stabil:
1. Jalur relatif kosong.
2. Satu motor pada jalur.
3. Obstacle dan lane lebih padat.

Langkah tiap run:
1. Pilih card **4.4 Performa Real-Time**.
2. Pilih Kondisi Visual yang sesuai.
3. Pastikan raw detection dan performance stream fresh.
4. Klik **START** dan jangan mengubah kondisi secara sengaja selama run.
5. GUI mencatat update aktual dan akan **AUTO STOP** sekitar 60 s.
6. Tunggu proses STOP + SAVE selesai sebelum memulai kondisi berikutnya.
7. Pastikan `SSE gap = 0` dan Status Data = VALID sebelum memakai FPS hitung sebagai hasil laporan.

Data yang direkap GUI 4.4 mencakup:
- jumlah frame event yang benar-benar diterima;
- FPS hitung dari frame event/durasi;
- mean pipeline FPS;
- mean inference time;
- mean process total;
- mean frame age;
- mean subscriber latency;
- tabel mean per detik 1–60;
- statistik mean, std, min, max, dan jumlah sampel.

Jika terdapat SSE gap atau tidak ada frame, jangan mengisi hasil dengan angka perkiraan. Ulangi run setelah sumber data stabil.

## 9. Evidence dan file yang wajib disimpan

Setelah **STOP + SAVE**, recorder saat ini menyiapkan raw/primary CSV, tabel CSV, YAML trial/rekap konfigurasi, XLSX, PNG grafik, dan manifest. Gunakan tombol artifact/download yang muncul setelah STOP.

Untuk setiap tahap final simpan:
- F4.1: screenshot/evidence runtime + parameter dump.
- F4.2: raw 200 trial, rekap DR/confidence, raw+rekap 30 frame negatif, PNG DR, PNG confidence.
- F4.3: raw 80 trial, rekap Success Rate, CSV histeresis, PNG Success Rate.
- F4.4: satu paket evidence per kondisi 60 s, tabel per detik, statistik, dan PNG time-series.

Jangan hanya menyimpan tabel rekap. Raw trial diperlukan untuk audit ulang bila dosen menanyakan asal angka.

## 10. Checklist sebelum memindahkan hasil ke laporan

Sebelum laporan dinyatakan final, pastikan:
- seluruh label **DATA ESTIMASI** pada angka hasil sudah diganti/ditandai dengan data aktual;
- tabel, grafik, dan narasi memakai dataset aktual yang sama;
- jumlah raw row cocok dengan protokol: 200 detection, 30 negative frame, 80 lane trial, dan 3 × 60 s performa;
- confidence trial gagal tetap kosong/N/A, bukan nol;
- statistik object detection tidak tercampur dengan `semantic_detections` atau `obstacle_metrics`;
- screenshot yang dipakai menunjukkan inference benar-benar ON;
- backend dan parameter efektif pada saat run tercatat;
- file export, manifest, dan parameter dump disimpan bersama sebagai evidence.

## 11. Troubleshooting cepat

**START disabled / BLOCKED**  
Baca panel Preflight. Untuk 4.2/4.4 biasanya raw detection stream belum fresh; untuk 4.3 lane stream belum fresh; seluruh tahap final juga membutuhkan YOLOPv2 CPU inference aktif.

**Obstacle Calibration tidak bisa Capture**  
Cek CAMERA HEALTHY, `semantic_detections` fresh, kelas objek benar, confidence cukup, objek berada dekat tengah citra, dan bbox tidak bottom-clipped untuk 3–5 m.

**Lane calibration menunjukkan INVALID**  
Periksa Y atas/bawah, persilangan kiri-kanan, nilai Kp, dan luas ROI. Jangan Apply sampai geometry valid.

**YAML berubah saat sedang edit**  
GUI akan menunjukkan `YAML CHANGED`. Pilih Reload untuk membuang draft atau Keep Draft jika memang ingin mempertahankan candidate, kemudian review ulang diff sebelum Stage/Apply.

**Hasil 4.4 CHECK / SSE GAP OR NO FRAME**  
Run tidak layak dipakai untuk FPS count. Stabilkan koneksi/stream lalu ulangi 60 detik; jangan mengganti nilai dengan estimasi.

---

Dokumen ini mengikuti implementasi repository NUC yang diperiksa pada 2026-09-16. Jika `perception_bab4.js`, `perception_calibration.js`, atau protokol laporan berubah, panduan harus diaudit ulang sebelum pengambilan data final.
