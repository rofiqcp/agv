# Steering Physical Calibration — Part 1

## Prinsip

Nilai steering pada protocol STM/ESC (`-90 .. 0 .. +90`) adalah **koordinat command/feedback protocol**, bukan sudut roda fisik. Setelah Part 1 aktif, sistem menyimpan tiga domain secara terpisah:

1. **Protocol command** — nilai yang dikirim ke STM.
2. **ACK/raw feedback** — koordinat steering yang dibaca kembali dari STM.
3. **Physical wheel angle** — sudut roda nyata terhadap garis lurus chassis.

`/esc/steering_actual` dan yaw-rate odometri memakai domain (3) setelah `steering_physical_calibration_enabled=true`.

## Yang harus diukur

Model Ackermann project memakai **sudut roda dalam**:

- Belok kiri maksimum: ukur sudut **roda depan kiri** terhadap garis lurus chassis.
- Belok kanan maksimum: ukur sudut **roda depan kanan** terhadap garis lurus chassis.
- Posisi lurus didefinisikan sebagai `0°` fisik.

Gunakan angle gauge, digital inclinometer, protractor, atau metode geometri yang dapat diulang. Jangan menyalin angka ESC/encoder sebagai sudut fisik.

## Urutan GUI

1. Pastikan drive berhenti (`/esc/drive_actual_mps` mendekati nol).
2. Buka **Kalibrasi Steering Fisik**.
3. Tekan **MULAI MODE KALIBRASI**.
4. Luruskan roda secara mekanik, tahan sekitar 1 detik, tekan **CAPTURE LURUS**.
5. Gerakkan ke endpoint kiri yang aman, tahan, tekan **CAPTURE KIRI MAX**.
6. Ukur sudut roda dalam kiri dan masukkan magnitudonya pada GUI.
7. Ulangi untuk endpoint kanan dan masukkan sudut roda dalam kanan.
8. Tentukan margin operational (default 95%).
9. Periksa ringkasan parameter turunan.
10. Tekan **TERAPKAN**.

GUI menyimpan endpoint protocol/feedback dan sudut fisik sebagai parameter yang berbeda.

## Mapping Part 1

Part 1 menggunakan interpolation linear per sisi.

Untuk target fisik kanan:

```
ratio = physical_target / physical_right_limit
protocol = protocol_center + ratio * (protocol_right - protocol_center)
```

Untuk target fisik kiri:

```
ratio = physical_target / physical_left_limit
protocol = protocol_center + ratio * (protocol_left - protocol_center)
```

Feedback melakukan mapping kebalikannya menggunakan raw ACK references.

Mapping kiri dan kanan **tidak dipaksa simetris**.

## Batas operasional

Karena planner/Nav2 menggunakan limit steering simetris, GUI mengambil:

```
common_mechanical = min(abs(left_physical), abs(right_physical))
operational = common_mechanical * margin
```

Dengan contoh:

- Left physical = `-31.5°`
- Right physical = `+28.0°`
- Margin = `95%`

maka:

```
common mechanical = 28.0°
operational = 26.6°
```

Nav2/Teleop tidak boleh meminta lebih dari ±26.6°, walaupun endpoint mekanik kiri lebih besar.

## Turning radius sementara

Part 1 menghitung radius pusat teoritis konservatif:

```
Rcenter = track_width/2 + wheelbase/tan(operational_angle)
Rtemporary = 1.10 * Rcenter
```

Angka ini **sementara**. Pada Part 3 harus diganti dengan circle-test nyata kiri/kanan.

## Parameter baru

`esc/config/ackermann.yaml`:

- `steering_physical_calibration_enabled`
- `steering_physical_left_limit_deg`
- `steering_physical_right_limit_deg`
- `steering_physical_operational_limit_deg`
- `steering_physical_calibration_saved_at`

`navigation/config/vehicle.yaml`:

- `measured_left_steering_limit_rad`
- `measured_right_steering_limit_rad`
- `max_steering_angle_rad`
- `operational_steering_angle_rad`
- `steering_calibration_source`
- `steering_calibration_saved_at`

## Catatan untuk Part 2

Part 1 masih mengasumsikan hubungan linear pada setiap sisi. Part 2 harus menambah **multi-point LUT** untuk menangkap non-linear linkage, misalnya setiap 5° fisik atau beberapa titik command tetap, dan mengukur hysteresis saat datang dari kiri vs kanan.
