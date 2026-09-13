# Rumus dan Potongan Program Penting Sistem Navigasi ADV

Dokumen ini disusun sebagai bahan pembahasan teknis untuk laporan **Azka Bintang Nur Taqwa — Perancangan Sistem Navigasi Autonomous Delivery Vehicle Berbasis Sensor GNSS, IMU, dan Odometri Menggunakan Extended Kalman Filter serta SMAC Hybrid A* pada Navigation2**.

Basis utama dokumen ini adalah source aktual pada `/home/sirobo/agv/src` dan struktur pembahasan pada `Azka Bintang Nur Taqwa.docx`. Fokusnya bukan menyalin seluruh source, tetapi memilih bagian yang paling representatif untuk menjelaskan alur matematis dan implementasi navigasi.

## Snapshot source yang dibaca

- Repository: `/home/sirobo/agv`
- Branch: `v1`
- HEAD saat audit: `0554ee00df8fb86d00ce87b9c9ce210572389ee7`
- Source navigasi utama: `src/navigation`
- Source odometri Ackermann: `src/esc/src/ackermann_controller_server.cpp`
- Konfigurasi kendaraan: `src/navigation/config/vehicle.yaml`
- Konfigurasi EKF: `src/navigation/config/ekf.yaml`
- Konfigurasi lokalisasi: `src/navigation/config/localization_cpp.yaml`
- Konfigurasi Nav2: `src/navigation/config/nav2_ackermann.yaml`

## Aturan pemakaian potongan program

Semua blok program di bawah sengaja ditulis **tanpa komentar source** agar dapat langsung dipindahkan ke laporan sebagai potongan program yang bersih. Nama fungsi, nama variabel, topic, frame, dan nilai parameter dipertahankan sesuai implementasi aktual sejauh bagian tersebut ditampilkan.

Untuk EKF dan SMAC Hybrid A*, perhitungan inti tidak ditulis ulang di repository karena dijalankan oleh package `robot_localization` dan plugin `nav2_smac_planner`. Oleh sebab itu, representasi implementasi yang benar untuk laporan adalah persamaan metode ditambah konfigurasi YAML yang menentukan state, input sensor, frame, covariance, dan constraint planner.
## 1. Peta hubungan subbab laporan terhadap source

| Subbab laporan | Implementasi utama | Bagian yang layak dibahas |
|---|---|---|
| 3.6.1 Validasi GNSS | `gnss_node.cpp`, `gnss.yaml` | validasi fix, satelit, DOP, hAcc, covariance, velocity |
| 3.6.2 Pra-pengolahan IMU | `imu_node.cpp`, `imu.yaml` | parsing frame, konversi SI, bias, scale, quaternion, freshness |
| 3.6.3 Odometri kendaraan | `ackermann_controller_server.cpp`, `vehicle.yaml` | eRPM-kecepatan, geometri Ackermann, yaw-rate, integrasi pose, covariance |
| 3.7 EKF lokal dan global | `ekf.yaml`, `autonomous.launch.py` | state aktif, source measurement, frame, frequency, timeout, Q/R |
| 3.8 Transformasi koordinat | `navigation_math.hpp`, `localization_core.cpp` | WGS84→ENU, ENU→map, lever arm antena, map→odom, koreksi anchor |
| 3.9 Global costmap | `nav2_ackermann.yaml` | frame, resolution, footprint, StaticLayer, InflationLayer |
| 3.10 SMAC Hybrid A* | `nav2_ackermann.yaml` | model DUBIN, heading bin, minimum turning radius, penalty, analytic expansion |
| Evaluasi Bab IV | `navigation_metrics.py` | RMSE posisi/yaw, P95, endpoint, rate, jitter, RPE |

---

# BAGIAN A — AKUISISI DAN VALIDASI GNSS

## 2. Quality gate GNSS

Source utama: `src/navigation/src/gnss_node.cpp`, fungsi `qualityGateReason()` dan `qualityGatePasses()`.

Satu measurement GNSS diperlakukan layak untuk jalur utama apabila status receiver valid dan threshold kualitas yang dikonfigurasi terpenuhi. Pada konfigurasi aktual, parameter pentingnya adalah `min_satellites = 8`, `max_dop = 2.0`, dan `max_hacc_m = 2.5`.
### 2.1 Rumus dan kondisi validasi

Secara logika, gate kualitas dapat dituliskan sebagai:

\[
G_{GNSS}=G_{serial}\land G_{position}\land G_{fix}\land G_{sat}\land G_{DOP}\land G_{hAcc}
\]

Untuk source UBX NAV-PVT aktual:

\[
G_{sat}: N_{sat}\ge N_{sat,min}
\]

\[
G_{DOP}: DOP\le DOP_{max}
\]

\[
G_{hAcc}: hAcc\le hAcc_{max}
\]

Dengan konfigurasi saat ini:

\[
N_{sat,min}=8,\qquad DOP_{max}=2.0,\qquad hAcc_{max}=2.5\;m
\]

Variabel: `N_sat` adalah jumlah satelit yang digunakan receiver; `DOP` adalah pDOP/HDOP sesuai sumber data; `hAcc` adalah estimasi akurasi horizontal receiver dalam meter; `gnssFixOK` adalah flag validitas solusi dari UBX; dan `fixType` menentukan jenis solusi posisi yang tersedia.
### 2.2 Potongan program quality gate GNSS

```cpp
std::string GnssNode::qualityGateReason() const
{
  if (!ser_ || !ser_->isOpen()) return "serial-disconnected";
  const double now = nowSec();
  if (!hasValidPosition()) return "position-invalid";
  if (!sats_.has_value()) return "satellites-missing";
  const bool dop_valid =
    quality_dop_.has_value() && std::isfinite(*quality_dop_) && *quality_dop_ > 0.0;

  if (last_position_from_ubx_) {
    if (last_ubx_pvt_time_ < 0.0 || now - last_ubx_pvt_time_ > data_timeout_sec_) {
      return "ubx-epoch-stale";
    }
    if (!last_ubx_gnss_fix_ok_) return "ubx-gnssFixOK-false";
    if (last_ubx_invalid_llh_) return "ubx-invalid-llh";
    const bool fix_type_ok =
      last_ubx_fix_type_ == 3u || (accept_ubx_gnss_dr_fix_ && last_ubx_fix_type_ == 4u);
    if (!fix_type_ok) return "ubx-fix-type-not-position-fix";
    if (*sats_ < min_satellites_) return "ubx-satellites-low";
    if (require_dop_for_ubx_quality_) {
      if (!dop_valid) return "ubx-dop-missing-invalid";
      if (*quality_dop_ > max_dop_) return "ubx-dop-high";
    }
    if (ubx_hacc_mm_ == 0) return "ubx-hacc-missing";
    const double hacc_m = static_cast<double>(ubx_hacc_mm_) / 1000.0;
    if (!std::isfinite(hacc_m) || hacc_m > max_hacc_m_) return "ubx-hacc-high";
    const double sacc_mps = static_cast<double>(ubx_sacc_mmps_) / 1000.0;
    if (require_speed_accuracy_for_quality_ &&
        (!std::isfinite(sacc_mps) || sacc_mps > max_sacc_mps_)) {
      return "ubx-sacc-high";
    }
    return "ok-ubx-nav-pvt";
  }

  if (!dop_valid) return "nmea-hdop-missing-invalid";
  if (last_nmea_gga_time_ < 0.0 || now - last_nmea_gga_time_ > 1.5) {
    return "nmea-gga-stale";
  }
  if (require_ubx_nav_pvt_for_fix_) {
    if (!nmeaFallbackEligible(now)) {
      if (!allow_validated_nmea_fallback_) return "waiting-ubx-nav-pvt";
      if (serial_connected_since_ >= 0.0 && now - serial_connected_since_ < nmea_fallback_after_sec_) {
        return "waiting-ubx-nav-pvt-fallback-delay";
      }
      if (!last_nmea_gga_checksum_present_) return "nmea-fallback-checksum-missing";
      if (last_nmea_fix_quality_ <= 0) return "nmea-no-fix";
      if (*sats_ < nmea_fallback_min_satellites_) return "nmea-fallback-satellites-low";
      if (*quality_dop_ > nmea_fallback_max_hdop_) return "nmea-fallback-hdop-high";
      return "nmea-fallback-not-eligible";
    }
    return "ok-validated-nmea-fallback";
  }

  if (*sats_ < min_satellites_) return "nmea-satellites-low";
  if (*quality_dop_ > max_dop_) return "nmea-hdop-high";
  return "ok-nmea";
}

bool GnssNode::qualityGatePasses() const
{
  const std::string reason = qualityGateReason();
  return reason.rfind("ok-", 0) == 0;
}

```

## 3. Covariance posisi GNSS

Jika UBX NAV-COV tersedia pada epoch yang sama, source mengubah covariance NED receiver menjadi ENU ROS. Untuk covariance posisi horizontal:

\[
\Sigma_{ENU}=T\Sigma_{NED}T^T
\]

Dengan transformasi sumbu:

\[
E=E,\qquad N=N,\qquad U=-D
\]

Jika NAV-COV tidak tersedia tetapi `hAcc` tersedia, digunakan pendekatan diagonal:

\[
\sigma_h=\frac{hAcc_{mm}}{1000}
\]

\[
R_x=R_y=\sigma_h^2
\]

\[
\sigma_v=\max(1.0,2\sigma_h),\qquad R_z=\sigma_v^2
\]

`R_x`, `R_y`, dan `R_z` adalah varians posisi yang dimasukkan pada `position_covariance`. Semakin besar covariance, semakin rendah kepercayaan estimator terhadap measurement tersebut.
### 3.1 Potongan program covariance posisi

```cpp
const bool cov_epoch_match = last_position_from_ubx_ && nav_cov_.valid &&
  sameEpoch(nav_cov_.itow_ms, last_itow_ms_);
if (cov_epoch_match && nav_cov_.pos_valid) {
  fix.position_covariance[0] = nav_cov_.pos_ee;
  fix.position_covariance[1] = nav_cov_.pos_ne;
  fix.position_covariance[2] = -nav_cov_.pos_ed;
  fix.position_covariance[3] = nav_cov_.pos_ne;
  fix.position_covariance[4] = nav_cov_.pos_nn;
  fix.position_covariance[5] = -nav_cov_.pos_nd;
  fix.position_covariance[6] = -nav_cov_.pos_ed;
  fix.position_covariance[7] = -nav_cov_.pos_nd;
  fix.position_covariance[8] = nav_cov_.pos_dd;
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_KNOWN;
} else if (last_position_from_ubx_ && ubx_hacc_mm_ > 0) {
  const double sigma_h = static_cast<double>(ubx_hacc_mm_) / 1000.0;
  const double sigma_v = ubx_vacc_mm_ > 0 ? static_cast<double>(ubx_vacc_mm_) / 1000.0 :
    std::max(1.0, 2.0 * sigma_h);
  fix.position_covariance[0] = sigma_h * sigma_h;
  fix.position_covariance[4] = sigma_h * sigma_h;
  fix.position_covariance[8] = sigma_v * sigma_v;
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
}
```
## 4. Kecepatan GNSS pada kerangka ENU

Untuk fallback berbasis ground speed dan course yang diukur searah jarum jam dari utara, komponen kecepatan ENU dibentuk sebagai:

\[
v_E=v\sin\chi
\]

\[
v_N=v\cos\chi
\]

Untuk UBX NAV-PVT, source menggunakan komponen `velE`, `velN`, dan `velD` secara langsung. Karena ROS menggunakan ENU, komponen vertikal dikonversi dari Down menjadi Up:

\[
v_U=-v_D
\]

Variabel `v` adalah ground speed dalam m/s, `χ` adalah course dari utara dalam radian, `v_E` adalah kecepatan arah timur, `v_N` arah utara, dan `v_U` arah atas.

### 4.1 Potongan program publikasi velocity

```cpp
if (last_position_from_ubx_ && vel_e_mps_.has_value() &&
    vel_n_mps_.has_value() && vel_d_mps_.has_value()) {
  twist.twist.twist.linear.x = *vel_e_mps_;
  twist.twist.twist.linear.y = *vel_n_mps_;
  twist.twist.twist.linear.z = -*vel_d_mps_;
} else if (speed_mps_.has_value() && heading_rad_.has_value()) {
  const double course_from_north_clockwise = *heading_rad_;
  twist.twist.twist.linear.x = *speed_mps_ * std::sin(course_from_north_clockwise);
  twist.twist.twist.linear.y = *speed_mps_ * std::cos(course_from_north_clockwise);
}
```
---

# BAGIAN B — PRA-PENGOLAHAN IMU

## 5. Konversi data mentah akselerometer dan giroskop

Source utama: `src/navigation/src/imu_node.cpp`, fungsi `parsePacket()`.

Untuk frame akselerometer `0x51`, nilai signed 16-bit dikonversi menjadi percepatan SI:

\[
a_x=\frac{raw_x}{32768}\times16\times g
\]

\[
a_y=\frac{raw_y}{32768}\times16\times g
\]

\[
a_z=\frac{raw_z}{32768}\times16\times g
\]

dengan `g = 9.80665 m/s²`.

Untuk frame giroskop `0x52`:

\[
\omega_{deg/s}=\frac{raw}{32768}\times2000
\]

Kemudian ketika dipublikasikan ke ROS:

\[
\omega_{rad/s}=s\,\omega_{deg/s}\frac{\pi}{180}-b_g
\]

`raw` adalah data ADC/protokol 16-bit, `s` adalah tanda orientasi sumbu mounting, dan `b_g` adalah bias giroskop per sumbu.
### 5.1 Potongan program parsing akselerometer dan giroskop

```cpp
case 0x51:
  std::memcpy(&v0, &data[2], 2);
  std::memcpy(&v1, &data[4], 2);
  std::memcpy(&v2, &data[6], 2);
  ax_ = (v0 / 32768.0) * 16.0 * 9.80665;
  ay_ = (v1 / 32768.0) * 16.0 * 9.80665;
  az_ = (v2 / 32768.0) * 16.0 * 9.80665;
  has_acc_ = true;
  last_accel_packet_time_ = packet_steady_sec;
  last_accel_measurement_stamp_ = packet_stamp;
  ++packets_acc_;
  return true;
case 0x52:
  std::memcpy(&v0, &data[2], 2);
  std::memcpy(&v1, &data[4], 2);
  std::memcpy(&v2, &data[6], 2);
  gx_ = (v0 / 32768.0) * 2000.0;
  gy_ = (v1 / 32768.0) * 2000.0;
  gz_ = (v2 / 32768.0) * 2000.0;
  has_gyro_ = true;
  last_gyro_packet_time_ = packet_steady_sec;
  last_gyro_measurement_stamp_ = packet_stamp;
  ++packets_gyro_;
  return true;
```
## 6. Koreksi bias dan scale factor IMU

Pada akselerometer, source menerapkan koreksi per sumbu setelah transformasi tanda mounting:

\[
a_{corr,j}=(s_j a_{raw,j}-b_{a,j})k_{a,j}
\]

Pada giroskop:

\[
\omega_{corr,j}=s_j\omega_{raw,j}-b_{g,j}
\]

untuk `j ∈ {x,y,z}`. Variabel `b_a` adalah bias akselerometer, `k_a` adalah scale factor akselerometer, `b_g` adalah bias giroskop, dan `s_j` adalah tanda orientasi sumbu sensor terhadap body frame.

### 6.1 Potongan program koreksi IMU

```cpp
msg.angular_velocity.x = vector_x_sign_ * gx_ * M_PI / 180.0 - gyro_bias_[0];
msg.angular_velocity.y = vector_y_sign_ * gy_ * M_PI / 180.0 - gyro_bias_[1];
msg.angular_velocity.z = vector_z_sign_ * gz_ * M_PI / 180.0 - gyro_bias_[2];

msg.linear_acceleration.x =
  (vector_x_sign_ * ax_ - accel_bias_[0]) * accel_scale_[0];
msg.linear_acceleration.y =
  (vector_y_sign_ * ay_ - accel_bias_[1]) * accel_scale_[1];
msg.linear_acceleration.z =
  (vector_z_sign_ * az_ - accel_bias_[2]) * accel_scale_[2];
```

## 7. Konversi Euler ke quaternion

Source menggunakan hubungan Euler–quaternion sebelum orientasi dimasukkan ke `sensor_msgs/Imu`.

Dengan:

\[
c_r=\cos(\phi/2),\ s_r=\sin(\phi/2),\quad
c_p=\cos(\theta/2),\ s_p=\sin(\theta/2),\quad
c_y=\cos(\psi/2),\ s_y=\sin(\psi/2)
\]

maka:

\[
q_w=c_rc_pc_y+s_rs_ps_y
\]
\[
q_x=s_rc_pc_y-c_rs_ps_y
\]
\[
q_y=c_rs_pc_y+s_rc_ps_y
\]
\[
q_z=c_rc_ps_y-s_rs_pc_y
\]

`φ` adalah roll, `θ` pitch, `ψ` yaw, sedangkan `q_x,q_y,q_z,q_w` adalah komponen quaternion.

```cpp
void ImuNode::quatFromEuler(double roll, double pitch, double yaw,
                            double & qx, double & qy, double & qz, double & qw)
{
  double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  qw = cr * cp * cy + sr * sp * sy;
  qx = sr * cp * cy - cr * sp * sy;
  qy = cr * sp * cy + sr * cp * sy;
  qz = cr * cp * sy - sr * sp * cy;
}
```

## 8. Sinkronisasi timestamp dan freshness IMU

Pesan `/imu/data` memakai timestamp paket gyro sebagai epoch utama. Akselerometer dan orientasi hanya dianggap sinkron bila selisih timestamp terhadap gyro tidak melebihi `component_sync_max_gap_sec`.

\[
|t_g-t_a|\le \Delta t_{sync,max}
\]

\[
|t_g-t_o|\le \Delta t_{sync,max}
\]

Konfigurasi aktual menetapkan `component_sync_max_gap_sec = 0.05 s`, `gyro_packet_timeout_sec = 0.35 s`, `accel_packet_timeout_sec = 0.5 s`, dan `orientation_publish_timeout_sec = 0.5 s`.

```cpp
const auto measurement_stamp = last_gyro_measurement_stamp_;
const bool accel_fresh = accel_fresh_by_age &&
  std::abs((measurement_stamp - last_accel_measurement_stamp_).seconds()) <= component_sync_max_gap_sec_;
const bool orientation_fresh = orientation_fresh_by_age &&
  std::abs((measurement_stamp - last_orientation_measurement_stamp_).seconds()) <= component_sync_max_gap_sec_;

if (require_fresh_gyro_for_imu_publish_ && !gyro_fresh) {
  return false;
}
```

---

# BAGIAN C — ODOMETRI KENDARAAN ACKERMANN

## 9. Konversi feedback eRPM menjadi kecepatan longitudinal

Pada mode transport STM32, model dasar source menetapkan konstanta `drive_erpm_per_mps`. Hubungan model mentah adalah:

\[
v_{raw}=\frac{ERPM}{K_{erpm}}
\]

Kemudian hasil kalibrasi jarak diterapkan sebagai:

\[
v=v_{raw}\,k_{odom}
\]

Dengan konfigurasi saat ini `K_erpm = 8000 eRPM/(m/s)` dan `k_odom = drive_odometry_calibration_scale = 1.0` selama kalibrasi fisik belum disahkan.

Variabel `ERPM` adalah feedback electrical RPM penggerak, `K_erpm` konstanta konversi electrical speed terhadap ground speed, dan `k_odom` faktor skala hasil kalibrasi odometri lurus.

```cpp
double rawDriveMpsFromErpm(double erpm) const
{
  if (transport_mode_ != "stm32") return erpm / (right_max_rpm_ / speed_max_mps_);
  return erpm / nativeDriveErpmPerMps();
}

const double drive_raw_mps = rawDriveMpsFromErpm(rpm) *
  (invert_drive_ ? -1.0 : 1.0);
const double drive_mps = drive_raw_mps * drive_odometry_calibration_scale_;
```

## 10. Model geometri Ackermann yang benar-benar digunakan source

Source memperlakukan `steering_rad` sebagai sudut roda dalam. Radius pusat kendaraan dihitung:

\[
R_c=\frac{W}{2}+\frac{L}{\tan(|\delta|)}
\]

Kurvatur ROS mengikuti konvensi yaw positif ke kiri, sedangkan steering positif pada aktuator adalah ke kanan:

\[
\kappa=-\operatorname{sgn}(\delta)\frac{1}{R_c}
\]

Sehingga yaw-rate:

\[
\omega=v\kappa=-\operatorname{sgn}(\delta)\frac{v}{\frac{W}{2}+\frac{L}{\tan(|\delta|)}}
\]

`L` adalah wheelbase, `W` track width, `δ` sudut steering roda dalam, `R_c` radius lintasan pusat kendaraan, `κ` kurvatur, `v` kecepatan longitudinal, dan `ω` yaw-rate.

### 10.1 Potongan program model Ackermann

```cpp
const double steering_rad = steering_calibrated_deg * kPi / 180.0;
double yaw_rate = 0.0;
if (std::abs(steering_rad) > 1.0e-6) {
  const double center_radius =
    0.5 * track_width_m_ +
    wheelbase_m_ / std::tan(std::abs(steering_rad));
  if (std::isfinite(center_radius) && center_radius > 1.0e-6) {
    const double ros_curvature =
      -std::copysign(1.0 / center_radius, steering_rad);
    yaw_rate = drive_mps * ros_curvature;
  }
}
```

## 11. Integrasi pose odometri diskrit

Setelah `v` dan `ω` diperoleh, orientasi diperbarui dengan:

\[
\psi_k=\psi_{k-1}+\omega_k\Delta t
\]

Posisi kemudian diperbarui menggunakan orientasi terbaru:

\[
x_k=x_{k-1}+v_k\cos(\psi_k)\Delta t
\]

\[
y_k=y_{k-1}+v_k\sin(\psi_k)\Delta t
\]

`x_k,y_k` adalah posisi odometri pada sampel ke-k, `ψ_k` yaw pada sampel ke-k, `Δt` interval waktu antar feedback, `v_k` kecepatan longitudinal, dan `ω_k` yaw-rate.

```cpp
const auto stamp = now();
double dt = 0.0;
if (odom_time_valid_)
  dt = std::clamp((stamp - last_odom_time_).seconds(), 0.0, 0.20);
last_odom_time_ = stamp;
odom_time_valid_ = true;

if (dt > 0.0) {
  odom_yaw_ += yaw_rate * dt;
  odom_x_ += drive_mps * std::cos(odom_yaw_) * dt;
  odom_y_ += drive_mps * std::sin(odom_yaw_) * dt;
}
```

## 12. Covariance odometri adaptif

Source tidak memakai satu covariance tetap untuk seluruh kondisi. Varians kecepatan dinaikkan ketika selisih target dan feedback RPM membesar:

\[
e_{rpm}=\operatorname{clamp}\left(\frac{|RPM_{target}-RPM_{actual}|}{RPM_{limit}},0,2\right)
\]

\[
\sigma_v^2=\sigma_{v,0}^2+k_{rpm}e_{rpm}^2
\]

Jika wheel-slip terdeteksi, varians tersebut dikalikan faktor tambahan. Varians yaw juga dinaikkan mengikuti besar steering ternormalisasi:

\[
e_\delta=\operatorname{clamp}\left(\frac{|\delta|}{\delta_{op,max}},0,1\right)
\]

\[
\sigma_\psi^2=\sigma_{\psi,0}^2+k_\delta e_\delta^2
\]

```cpp
const double rpm_error_norm = std::clamp(
  std::abs(diagnostic_drive_target_rpm_ - rpm) /
  std::max(1.0, rightCommandLimit()), 0.0, 2.0);
const double steer_norm = std::clamp(
  std::abs(steering_rad) /
  std::max(1.0e-6, operationalPhysicalLimitDeg() * kPi / 180.0), 0.0, 1.0);

double v_var = odom_v_variance_base_ +
  odom_v_variance_rpm_error_gain_ * rpm_error_norm * rpm_error_norm;
if (slip_fresh) v_var *= wheel_slip_covariance_multiplier_;
const double yaw_var = odom_yaw_variance_base_ +
  odom_yaw_variance_steer_gain_ * steer_norm * steer_norm;

odom.pose.covariance[0] = std::max(0.05, v_var);
odom.pose.covariance[7] = std::max(0.05, v_var + 0.5 * yaw_var);
odom.pose.covariance[35] = yaw_var;
odom.twist.covariance[0] = v_var;
odom.twist.covariance[35] = odom_yaw_rate_variance_base_ +
  odom_yaw_variance_steer_gain_ * steer_norm * steer_norm;
```

---

# BAGIAN D — EXTENDED KALMAN FILTER

## 13. Vektor keadaan EKF

Laporan menggunakan struktur 15-state yang sama dengan urutan konfigurasi `robot_localization`:

\[
\mathbf{x}=
[x,\ y,\ z,\ \phi,\ \theta,\ \psi,\ v_x,\ v_y,\ v_z,\ \omega_x,\ \omega_y,\ \omega_z,\ a_x,\ a_y,\ a_z]^T
\]

`x,y,z` adalah posisi; `φ,θ,ψ` adalah roll, pitch, yaw; `v_x,v_y,v_z` adalah kecepatan linear; `ω_x,ω_y,ω_z` adalah kecepatan sudut; dan `a_x,a_y,a_z` adalah percepatan linear.

Pada `two_d_mode: true`, gerak utama dibatasi pada bidang planar sehingga state vertikal, roll, dan pitch tidak menjadi measurement utama navigasi.

## 14. Persamaan prediksi EKF

Prediksi state nonlinear:

\[
\hat{\mathbf{x}}^-_k=f(\hat{\mathbf{x}}_{k-1},\Delta t)
\]

Prediksi covariance:

\[
\mathbf{P}^-_k=\mathbf{F}_k\mathbf{P}_{k-1}\mathbf{F}_k^T+\mathbf{Q}_k
\]

`f(·)` adalah model proses nonlinear, `F_k` Jacobian model proses, `P` covariance state, dan `Q` process-noise covariance.

## 15. Persamaan koreksi EKF

Innovation atau residual measurement:

\[
\mathbf{y}_k=\mathbf{z}_k-h(\hat{\mathbf{x}}^-_k)
\]

Innovation covariance:

\[
\mathbf{S}_k=\mathbf{H}_k\mathbf{P}^-_k\mathbf{H}_k^T+\mathbf{R}_k
\]

Kalman gain:

\[
\mathbf{K}_k=\mathbf{P}^-_k\mathbf{H}_k^T\mathbf{S}_k^{-1}
\]

Koreksi state:

\[
\hat{\mathbf{x}}_k=\hat{\mathbf{x}}^-_k+\mathbf{K}_k\mathbf{y}_k
\]

Pembaruan covariance:

\[
\mathbf{P}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\mathbf{P}^-_k
\]

`z_k` adalah measurement, `h(·)` model measurement, `H_k` Jacobian measurement, `R_k` measurement-noise covariance, `S_k` innovation covariance, dan `K_k` Kalman gain.

Perhitungan matriks di atas dijalankan di dalam `robot_localization/ekf_node`. Repository AGV menentukan input, komponen state aktif, timing, frame, dan covariance melalui YAML.

## 16. EKF lokal — state dan measurement aktif

EKF lokal menghasilkan estimasi kontinu terhadap `world_frame: odom`. Input aktual adalah `vx` dari `/esc/odom`, `ωz` dari `/imu/data`, dan `vx` dari `/gnss/base_velocity_fusion`.

Urutan boolean selalu mengikuti 15-state:

`[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`.

```yaml
ekf_filter_node_odom:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.25
    two_d_mode: true
    predict_to_current_time: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom
    odom0: /esc/odom
    odom0_config:
      [false, false, false,
       false, false, false,
       true, false, false,
       false, false, false,
       false, false, false]
    imu0: /imu/data
    imu0_config:
      [false, false, false,
       false, false, false,
       false, false, false,
       false, false, true,
       false, false, false]
    imu0_relative: true
    twist0: /gnss/base_velocity_fusion
    twist0_config:
      [false, false, false,
       false, false, false,
       true, false, false,
       false, false, false,
       false, false, false]
    use_control: false
```

Diagonal `Q` aktual untuk urutan 15 state adalah:

\[
diag(Q)=[0.05,0.05,0.06,0.03,0.03,0.06,0.025,0.025,0.04,0.01,0.01,0.02,0.01,0.01,0.015]
\]

Nilai tersebut adalah parameter implementasi, bukan hasil optimalisasi otomatis. Pembahasan Bab IV harus menghubungkan perubahan `Q` dengan perubahan RMSE, smoothness, dan respons filter pada data pengujian nyata.

## 17. EKF global — state dan measurement aktif

EKF global bekerja pada `world_frame: map` dengan frequency 10 Hz. Posisi `x-y` berasal dari `/odometry/gnss_map`; `vx` dari `/gnss/base_velocity_fusion`; yaw absolut dapat berasal dari `/gnss/cog_heading_fusion` dan `/heading/validated_fusion`; sedangkan `ωz` tetap berasal dari IMU.

```yaml
ekf_filter_node_map:
  ros__parameters:
    frequency: 10.0
    sensor_timeout: 2.0
    two_d_mode: true
    predict_to_current_time: false
    publish_tf: false
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: map
    odom0: /odometry/gnss_map
    odom0_config:
      [true, true, false,
       false, false, false,
       false, false, false,
       false, false, false,
       false, false, false]
    twist0: /gnss/base_velocity_fusion
    twist0_config:
      [false, false, false,
       false, false, false,
       true, false, false,
       false, false, false,
       false, false, false]
    pose0: /gnss/cog_heading_fusion
    pose0_config:
      [false, false, false,
       false, false, true,
       false, false, false,
       false, false, false,
       false, false, false]
    pose1: /heading/validated_fusion
    pose1_config:
      [false, false, false,
       false, false, true,
       false, false, false,
       false, false, false,
       false, false, false]
    imu0: /imu/data
    imu0_config:
      [false, false, false,
       false, false, false,
       false, false, false,
       false, false, true,
       false, false, false]
    imu0_relative: true
```

`publish_tf: false` pada EKF global penting dibahas karena source `LocalizationCore` menjadi pemilik transformasi `map → odom`; ini menghindari dua publisher TF yang bersaing untuk transformasi global yang sama.

---

# BAGIAN E — TRANSFORMASI KOORDINAT DAN FRAME

## 18. Transformasi WGS84 ke koordinat East-North

Source menggunakan pendekatan tangent-plane/equirectangular lokal. Dengan latitude dan longitude dalam radian:

\[
\bar{\varphi}=\frac{\varphi+\varphi_0}{2}
\]

\[
E=R_E(\lambda-\lambda_0)\cos(\bar{\varphi})
\]

\[
N=R_E(\varphi-\varphi_0)
\]

Dengan `R_E = 6378137 m`, `φ,λ` adalah latitude dan longitude measurement, `φ0,λ0` titik referensi, `E` posisi relatif ke timur dan `N` posisi relatif ke utara.

```cpp
inline EnuPoint wgs84ToEnu(
  double latitude_deg, double longitude_deg,
  double reference_latitude_deg, double reference_longitude_deg)
{
  const double lat = deg2rad(latitude_deg);
  const double lon = deg2rad(longitude_deg);
  const double ref_lat = deg2rad(reference_latitude_deg);
  const double ref_lon = deg2rad(reference_longitude_deg);
  const double mean_lat = 0.5 * (lat + ref_lat);

  EnuPoint out;
  out.east_m = (lon - ref_lon) * std::cos(mean_lat) * kEarthRadiusM;
  out.north_m = (lat - ref_lat) * kEarthRadiusM;
  return out;
}
```

Parameter referensi source aktual adalah `reference_latitude = -7.050964918716`, `reference_longitude = 110.435059847106`, `reference_map_x_m = 167.493142`, dan `reference_map_y_m = 94.933756`.

## 19. Transformasi East-North ke frame map

Untuk sudut rotasi `θ_m` dari ENU ke map:

\[
x_m=x_0+E\cos\theta_m-N\sin\theta_m
\]

\[
y_m=y_0+E\sin\theta_m+N\cos\theta_m
\]

\[
\psi_m=wrap(\psi_{ENU}+\theta_m)
\]

```cpp
inline Pose2D enuToMap(
  const EnuPoint & enu, double reference_map_x_m, double reference_map_y_m,
  double map_yaw_from_enu_rad, double yaw_enu_rad)
{
  const double c = std::cos(map_yaw_from_enu_rad);
  const double s = std::sin(map_yaw_from_enu_rad);
  Pose2D out;
  out.x = reference_map_x_m + c * enu.east_m - s * enu.north_m;
  out.y = reference_map_y_m + s * enu.east_m + c * enu.north_m;
  out.yaw = normalizeAngle(yaw_enu_rad + map_yaw_from_enu_rad);
  return out;
}
```

## 20. Koreksi lever arm antena GNSS ke base_footprint

Karena antena tidak tepat berada di origin kendaraan, posisi antena harus dikoreksi menggunakan offset body `(r_x,r_y)`:

\[
x_b=x_a-(\cos\psi\,r_x-\sin\psi\,r_y)
\]

\[
y_b=y_a-(\sin\psi\,r_x+\cos\psi\,r_y)
\]

Konfigurasi aktual: `gnss_antenna_x_m = 0.165 m` dan `gnss_antenna_y_m = 0.0 m`.

```cpp
inline Pose2D antennaToBase(
  const Pose2D & antenna_pose_map, double antenna_x_m, double antenna_y_m)
{
  const double c = std::cos(antenna_pose_map.yaw);
  const double s = std::sin(antenna_pose_map.yaw);
  Pose2D out = antenna_pose_map;
  out.x -= c * antenna_x_m - s * antenna_y_m;
  out.y -= s * antenna_x_m + c * antenna_y_m;
  return out;
}
```

## 21. Pembentukan transformasi map → odom

Hubungan transformasi yang digunakan adalah:

\[
T_{map}^{odom}=T_{map}^{base}(T_{odom}^{base})^{-1}
\]

Untuk yaw planar:

\[
\psi_{mo}=wrap(\psi_{mb}-\psi_{ob})
\]

Kemudian translasi:

\[
x_{mo}=x_{mb}-(\cos\psi_{mo}\,x_{ob}-\sin\psi_{mo}\,y_{ob})
\]

\[
y_{mo}=y_{mb}-(\sin\psi_{mo}\,x_{ob}+\cos\psi_{mo}\,y_{ob})
\]

```cpp
inline Pose2D mapOdomFromBase(const Pose2D & map_base, const Pose2D & odom_base)
{
  const double yaw = normalizeAngle(map_base.yaw - odom_base.yaw);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Pose2D out;
  out.yaw = yaw;
  out.x = map_base.x - (c * odom_base.x - s * odom_base.y);
  out.y = map_base.y - (s * odom_base.x + c * odom_base.y);
  return out;
}
```

Kebalikannya untuk memperoleh pose kendaraan pada map dari anchor dan odometri lokal:

\[
T_{map}^{base}=T_{map}^{odom}T_{odom}^{base}
\]

```cpp
inline Pose2D mapBaseFromOdom(const Pose2D & map_odom, const Pose2D & odom_base)
{
  const double c = std::cos(map_odom.yaw);
  const double s = std::sin(map_odom.yaw);
  Pose2D out;
  out.x = map_odom.x + c * odom_base.x - s * odom_base.y;
  out.y = map_odom.y + s * odom_base.x + c * odom_base.y;
  out.yaw = normalizeAngle(map_odom.yaw + odom_base.yaw);
  return out;
}
```

## 22. Koreksi map → odom secara bertahap

Source tidak langsung mengganti anchor dengan satu measurement GNSS. Kandidat anchor dibaurkan dengan anchor lama:

\[
\mathbf{p}_{blend}=\mathbf{p}_{old}+\alpha(\mathbf{p}_{candidate}-\mathbf{p}_{old})
\]

Untuk yaw digunakan selisih sudut terpendek:

\[
\psi_{blend}=wrap\left(\psi_{old}+\alpha\,wrap(\psi_{candidate}-\psi_{old})\right)
\]

```cpp
inline Pose2D blendPose(const Pose2D & current, const Pose2D & target, double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  Pose2D out;
  out.x = current.x + alpha * (target.x - current.x);
  out.y = current.y + alpha * (target.y - current.y);
  out.yaw = normalizeAngle(
    current.yaw + alpha * normalizeAngle(target.yaw - current.yaw));
  return out;
}
```
Pada `LocalizationCore`, pembaruan transformasi global dibatasi agar perubahan posisi tidak meloncat. Nilai source aktual yang relevan adalah `strict_correction_alpha = 0.08`, `strict_moving_correction_alpha = 0.01`, `strict_slip_correction_alpha = 0.1`, dan `strict_max_correction_m = 0.08 m`. Bagian ini penting untuk menjelaskan bahwa `odom → base_footprint` tetap kontinu, sedangkan referensi absolut pada frame `map` diperbarui secara bertahap.
```cpp
      double alpha = slip_pose_correction ? strict_slip_correction_alpha_ :
        (stationary ? strict_correction_alpha_ : strict_moving_correction_alpha_);
      if (stationary && freeze_stationary_map_translation_ && !slip_pose_correction) {
        alpha = 0.0;
      }

      if (!strictQualityPassesUnlocked()) {
        const double hacc_scale = std::clamp(
          strict_max_hacc_m_ / std::max(strict_max_hacc_m_, quality_.hacc_m),
          0.35, 0.75);
        alpha *= hacc_scale;
      }

      auto blended =
        navigation_math::blendPose(anchor_map_odom_, candidate, alpha);
      double dx = blended.x - anchor_map_odom_.x;
      double dy = blended.y - anchor_map_odom_.y;
      const double dxy = std::hypot(dx, dy);
      if (dxy > strict_max_correction_m_ && dxy > 1.0e-9) {
        const double scale = strict_max_correction_m_ / dxy;
        dx *= scale;
        dy *= scale;
      }

      double dyaw = 0.0;
      const double current_map_yaw = navigation_math::normalizeAngle(
        anchor_map_odom_.yaw + correction_odom.yaw);
      bool yaw_from_global_ekf = false;
      if (use_global_ekf_yaw_for_map_correction_ && globalYawFusionFreshUnlocked()) {
        const double innovation = navigation_math::normalizeAngle(
          global_ekf_pose_.yaw - current_map_yaw);
        if (std::abs(innovation) <= global_ekf_yaw_max_innovation_rad_) {
          dyaw = std::clamp(global_ekf_yaw_correction_alpha_ * innovation,
            -global_ekf_yaw_max_step_rad_, global_ekf_yaw_max_step_rad_);
          yaw_from_global_ekf = true;
        }
      } else if (!enable_global_gnss_cog_fusion_ && gnssCourseYawUsableUnlocked()) {
        const double desired_map_yaw = navigation_math::normalizeAngle(
          quality_.course_enu_rad + map_yaw_from_enu_rad_ + map_calibration_.yaw);
        const double innovation = navigation_math::normalizeAngle(desired_map_yaw - current_map_yaw);
        if (std::abs(innovation) <= cog_max_innovation_rad_) {
          dyaw = std::clamp(cog_yaw_alpha_ * innovation,
            -cog_max_yaw_step_rad_, cog_max_yaw_step_rad_);
        }
      }

      anchor_map_odom_.x += dx;
      anchor_map_odom_.y += dy;
      anchor_map_odom_.yaw = navigation_math::normalizeAngle(anchor_map_odom_.yaw + dyaw);
```

## 23. Publikasi TF map → odom

Transformasi yang telah dihitung dipublikasikan sebagai `map → odom`. Pose kendaraan untuk kebutuhan estimator/map kemudian diperoleh kembali dari komposisi `map → odom → base_footprint`. Source memberi `tf_future_offset_sec` kecil untuk menahan mismatch timestamp antara loop TF dan controller.

Rantai frame utama yang perlu ditulis dalam laporan adalah:

\[
map\rightarrow odom\rightarrow base\_footprint
\]

Dengan transformasi sensor statis terhadap body didefinisikan melalui URDF/TF terpisah. `LocalizationCore` hanya menerbitkan `map → odom`, sedangkan EKF lokal memiliki `publish_tf: true` untuk jalur lokal.
```cpp
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now() + rclcpp::Duration::from_seconds(tf_future_offset_sec_);
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = anchor.x;
    tf.transform.translation.y = anchor.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = quaternionFromYaw(anchor.yaw);
    tf_broadcaster_->sendTransform(tf);
```

---

# BAGIAN F — GLOBAL COSTMAP

## 24. Representasi footprint kendaraan

Footprint aktif pada source adalah poligon:

\[
[(0.52,0.30),(0.52,-0.30),(-0.52,-0.30),(-0.52,0.30)]
\]

Dengan `footprint_padding = 0.03 m`. Artinya pemeriksaan collision tidak hanya menggunakan titik pusat kendaraan, tetapi area badan kendaraan pada frame `base_footprint`.

## 25. Static layer dan inflation layer

Global costmap bekerja pada `global_frame: map`, `robot_base_frame: base_footprint`, dan `resolution: 0.1 m`. Peta statis dimasukkan melalui `StaticLayer`, kemudian biaya di sekitar obstacle diperluas oleh `InflationLayer` dengan `inflation_radius = 0.8 m` dan `cost_scaling_factor = 2.5`.

Secara konseptual biaya inflation dapat ditulis sebagai fungsi menurun terhadap jarak:

\[
C(d)=f(d;r_{infl},k_c)
\]

`d` adalah jarak terhadap obstacle, `r_infl` radius inflation, dan `k_c` cost scaling factor. Bentuk fungsi numeriknya merupakan implementasi internal plugin Navigation2; repository ini mengatur parameternya, bukan menulis ulang algoritma inflation.
### 25.1 Potongan konfigurasi global costmap

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      update_frequency: 0.5
      publish_frequency: 0.2
      global_frame: map
      robot_base_frame: base_footprint
      resolution: 0.1
      footprint: '[[0.52,0.30],[0.52,-0.30],[-0.52,-0.30],[-0.52,0.30]]'
      footprint_padding: 0.03
      track_unknown_space: false
      plugins:
      - static_layer
      - inflation_layer
      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        enabled: true
        map_subscribe_transient_local: true
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        enabled: true
        inflation_radius: 0.8
        cost_scaling_factor: 2.5
      lethal_cost_threshold: 100
      always_send_full_costmap: false
      transform_tolerance: 0.3
```

---

# BAGIAN G — SMAC HYBRID A* GLOBAL PLANNER

## 26. Fungsi biaya pencarian

Dasar pencarian A*/Hybrid-A* dapat dinyatakan:

\[
f(n)=g(n)+h(n)
\]

`f(n)` adalah total estimasi biaya node, `g(n)` biaya aktual dari start ke node, dan `h(n)` heuristic dari node menuju goal. Pada Hybrid-A*, state tidak hanya posisi grid, tetapi juga heading kendaraan:

\[
\mathbf{s}=[x,y,\theta]^T
\]

Pada source, `angle_quantization_bins = 48`, sehingga resolusi heading diskrit:

\[
\Delta\theta=\frac{2\pi}{48}=0.1308997\;rad=7.5^\circ
\]

## 27. Constraint kurvatur dan minimum turning radius

Batas kurvatur kendaraan:

\[
|\kappa|\le\kappa_{max}=\frac{1}{R_{min}}
\]

Dengan nilai aktif `R_min = 1.597679121829 m`, maka batas kurvatur nominal sekitar `0.62590791 1/m`. Nilai tersebut juga konsisten dengan `max_yaw_rate_rps = 0.625907910003` pada kecepatan 1 m/s.

Nilai `R_min` saat ini masih merupakan fallback teoritis. Berdasarkan geometri `L = 0.70 m`, `W = 0.48 m`, steering fallback `|δ|max = 30°`, dan margin 10%:

\[
R_{min}=\left(\frac{W}{2}+\frac{L}{\tan|\delta_{max}|}\right)(1+0.10)
\]

yang menghasilkan sekitar `1.59768 m`. Karena `steering_circle_calibration_valid: false`, nilai ini jangan ditulis sebagai hasil pengukuran circle-test.
## 28. Potongan konfigurasi SMAC Hybrid A*

```yaml
planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 0.5
    planner_plugins:
    - GridBased
    GridBased:
      plugin: nav2_smac_planner/SmacPlannerHybrid
      tolerance: 0.05
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: false
      max_iterations: 250000
      max_on_approach_iterations: 1000
      max_planning_time: 2.5
      motion_model_for_search: DUBIN
      angle_quantization_bins: 48
      minimum_turning_radius: 1.597679121829
      reverse_penalty: 3.0
      change_penalty: 0.0
      non_straight_penalty: 1.2
      cost_penalty: 2.2
      retrospective_penalty: 0.015
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 8.0
      lookup_table_size: 20.2
      cache_obstacle_heuristic: true
      smooth_path: true
      smoother:
        max_iterations: 200
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-06
        do_refinement: false
```

---

# BAGIAN H — VALIDASI VELOCITY GNSS DAN HEADING COG

## 29. Normalized Innovation Squared untuk konsistensi wheel–GNSS

Sebelum `/gnss/base_velocity_fusion` diterbitkan, `LocalizationCore` membandingkan kecepatan longitudinal odometri lokal dengan kecepatan GNSS pada body frame.

Residual:

\[
r_v=v_{wheel}-v_{GNSS}
\]

Innovation variance:

\[
S_v=\max(\epsilon,\sigma_{wheel}^2+\sigma_{GNSS}^2)
\]

NIS satu dimensi:

\[
NIS_v=\frac{r_v^2}{S_v}
\]

Measurement lolos gate jika `NIS_v ≤ wheel_gnss_nis_gate`. Source aktual menggunakan threshold `6.634896601`.

Untuk COG digunakan bentuk serupa:

\[
r_\chi=wrap(\chi_{vel}-\chi_{COG}),\qquad
NIS_\chi=\frac{r_\chi^2}{\sigma_\chi^2}
\]

Bagian ini layak dibahas karena menunjukkan bahwa data GNSS tidak langsung difusikan hanya karena tersedia; konsistensi measurement diperiksa lebih dahulu.
### 29.1 Potongan program NIS dan qualification

```cpp
    const double wheel_residual = local_forward_at_gnss_mps_ - gnss_base_vx_mps_;
    const double wheel_innovation_variance = std::max(
      innovation_min_variance_, local_forward_variance_at_gnss_ + gnss_base_vx_variance_);
    const double wheel_nis = have_local_motion_at_gnss_ ?
      (wheel_residual * wheel_residual / wheel_innovation_variance) :
      std::numeric_limits<double>::quiet_NaN();
    const double cog_variance = std::max(
      innovation_min_variance_,
      std::isfinite(quality_.course_accuracy_rad) && quality_.course_accuracy_rad > 0.0 ?
        quality_.course_accuracy_rad * quality_.course_accuracy_rad : 1.0e6);
    const double cog_nis = std::isfinite(cog_vel_residual) ?
      (cog_vel_residual * cog_vel_residual / cog_variance) :
      std::numeric_limits<double>::quiet_NaN();
    const bool wheel_nis_gate_pass = !std::isfinite(wheel_nis) || wheel_nis <= wheel_gnss_nis_gate_;
    const bool cog_nis_gate_pass = !std::isfinite(cog_nis) || cog_nis <= cog_nis_gate_;
    wheel_gnss_residual_mps_ = wheel_residual;
    wheel_slip_motion_detected_ = have_local_motion_at_gnss_ && vel_fresh && quality_ok &&
      std::abs(wheel_residual) >= wheel_gnss_slip_residual_mps_ &&
      std::max(std::abs(local_forward_at_gnss_mps_), std::abs(gnss_base_vx_mps_)) >=
        slip_min_wheel_speed_mps_;

    gnss_velocity_qualified_ = vel_fresh && quality_ok && gnss_last_velocity_covariance_valid_ &&
      vector_speed_ok && cog_vel_ok && cog_nis_gate_pass && wheel_nis_gate_pass && lateral_ok;
    if (fit_fresh) gnss_velocity_qualified_ = gnss_velocity_qualified_ && fit_speed_ok;
```

## 30. Pembentukan heading pendukung `/heading/validated_fusion`

Source `mag_heading_fusion_node.cpp` membentuk heading inersial dari integrasi gyro-Z dan membandingkannya dengan heading magnetometer yang telah dikalibrasi. Bagian ini relevan karena EKF global memiliki input yaw dari `/heading/validated_fusion`.

Integrasi heading inersial:

\[
\psi_k=wrap\left(\psi_{k-1}+(\omega_z-b_{gz})\Delta t\right)
\]

Saat kendaraan stationer, estimasi bias gyro-Z diperbarui perlahan:

\[
b_{gz,k}=0.98\,b_{gz,k-1}+0.02\,\omega_{z,k}
\]

Untuk magnetometer dengan kompensasi tilt:

\[
m_x^h=m_x\cos\theta+m_z\sin\theta
\]

\[
m_y^h=m_x\sin\phi\sin\theta+m_y\cos\phi-m_z\sin\phi\cos\theta
\]

Heading magnetik kemudian:

\[
\psi_{mag}=wrap(s\,atan2(m_y^h,m_x^h)+b_\psi-D)
\]

`φ` dan `θ` adalah roll-pitch, `s` tanda orientasi sensor, `bψ` offset heading, dan `D` declination.
Jika kedua heading fresh dan beda sudut berada di bawah `consensus_max_error_rad`, source memakai rata-rata sirkular berbobot:

\[
w_N=\frac{1}{\sigma_N^2},\qquad w_I=\frac{1}{\sigma_I^2}
\]

\[
\psi_{validated}=atan2(w_N\sin\psi_N+w_I\sin\psi_I,\;w_N\cos\psi_N+w_I\cos\psi_I)
\]

Dengan demikian averaging tidak dilakukan langsung pada angka sudut linear, sehingga transisi di sekitar `-π/π` tetap benar.
---

# BAGIAN I — HUBUNGAN OUTPUT NAV2 DENGAN KINEMATIKA ACKERMANN

## 31. Konversi Twist menjadi sudut steering

Bagian ini tidak harus menjadi fokus utama jika laporan hanya membahas global planner, tetapi sangat berguna untuk menunjukkan bahwa command `Twist` dari Navigation2 dibatasi sesuai geometri kendaraan.

Dari command linear dan angular:

\[
\kappa=\frac{\omega}{v}
\]

\[
R_c=\frac{1}{|\kappa|}
\]

Radius roda dalam:

\[
R_{inner}=R_c-\frac{W}{2}
\]

Sudut roda dalam:

\[
\delta_{inner}=atan\left(\frac{L}{R_{inner}}\right)
\]

Tanda steering dibalik terhadap konvensi yaw ROS karena aktuator mendefinisikan steering positif ke kanan.
```cpp
double steeringFromTwistRad(const geometry_msgs::msg::Twist & cmd) const
{
  if (!std::isfinite(cmd.linear.x) || !std::isfinite(cmd.angular.z) ||
      std::abs(cmd.linear.x) < min_speed_for_yaw_mps_) {
    return 0.0;
  }
  const double kappa = cmd.angular.z / cmd.linear.x;
  if (!std::isfinite(kappa) || std::abs(kappa) < 1.0e-9) return 0.0;

  const double center_radius = 1.0 / std::abs(kappa);
  const double inner_radius =
    std::max(1.0e-6, center_radius - 0.5 * track_width_m_);
  const double inner_angle = std::atan(wheelbase_m_ / inner_radius);
  const double steer = -std::copysign(inner_angle, kappa);
  return std::clamp(
    steer, -max_steering_angle_rad_, max_steering_angle_rad_);
}
```
## 32. Pembatas yaw-rate berdasarkan radius putar

Source membatasi yaw-rate sehingga command tidak meminta kurvatur yang lebih tajam daripada kemampuan geometri kendaraan:

\[
\omega_{curv,max}=\frac{|v|}{R_{min}}
\]

\[
\omega_{limit}=\min(\omega_{vehicle,max},\omega_{curv,max})
\]

```cpp
const double curvature_yaw_cap =
  std::abs(out.linear.x) / std::max(0.10, minimum_turning_radius_m_);
const double yaw_cap = std::min(max_yaw_rate_rps_, curvature_yaw_cap);
out.angular.z = std::clamp(in.angular.z, -yaw_cap, yaw_cap);
if (std::abs(out.linear.x) < linear_deadband_mps_) out.linear.x = 0.0;
if (std::abs(out.angular.z) < angular_deadband_rps_) out.angular.z = 0.0;
if (std::abs(out.linear.x) < min_speed_for_yaw_mps_) out.angular.z = 0.0;
```
---

# BAGIAN J — TAMBAHAN PENTING PADA SOURCE NAVIGASI AKTUAL

## 33. Jarak berhenti dinamis berbasis latency

Bagian ini berada pada `trajectory_safety_supervisor.cpp` dan tidak wajib dimasukkan ke struktur utama laporan Azka apabila ruang lingkup tetap berhenti pada EKF, global costmap, dan SMAC Hybrid A*. Namun, ini penting bila laporan ingin menjelaskan interlock keselamatan command navigasi aktual.

Jarak reaksi:

\[
d_r=v(t_{lat}+t_{margin})
\]

Jarak pengereman:

\[
d_b=\frac{v^2}{2a_b}
\]

Jarak berhenti yang dipakai supervisor:

\[
d_{stop}=clamp(d_r+d_b+d_{margin},0,d_{max})
\]

```cpp
const double reaction_distance =
  forward_speed * (perception_latency_sec + latency_safety_margin_sec_);
const double braking_distance =
  forward_speed * forward_speed / (2.0 * braking_deceleration_mps2_);
const double required_stop_distance = std::clamp(
  reaction_distance + braking_distance + latency_distance_margin_m_,
  0.0, max_dynamic_stop_distance_m_);
```
---

# BAGIAN K — RUMUS EVALUASI BAB IV

## 34. Error posisi dan RMSE posisi

Error Euclidean setiap sampel:

\[
e_{p,i}=\sqrt{(x_{est,i}-x_{gt,i})^2+(y_{est,i}-y_{gt,i})^2}
\]

RMSE posisi:

\[
RMSE_p=\sqrt{\frac{1}{N}\sum_{i=1}^{N}e_{p,i}^2}
\]

`x_est,y_est` adalah hasil estimator, `x_gt,y_gt` ground truth, dan `N` jumlah sampel.

## 35. Error yaw dan RMSE yaw

\[
e_{\psi,i}=atan2(\sin(\psi_{est,i}-\psi_{gt,i}),\cos(\psi_{est,i}-\psi_{gt,i}))
\]

\[
RMSE_\psi=\sqrt{\frac{1}{N}\sum_{i=1}^{N}e_{\psi,i}^2}
\]

Penggunaan fungsi wrap penting agar selisih dekat `+π` dan `-π` tidak dibaca sebagai error hampir `2π`.
## 36. Endpoint error, P95, sampling rate, dan jitter

Endpoint error:

\[
e_{end}=\sqrt{(x_{end}-x_g)^2+(y_{end}-y_g)^2}
\]

Persentil ke-95 error posisi:

\[
e_{p,95}=P_{95}(\{e_{p,i}\})
\]

Estimasi rate data berdasarkan median period:

\[
f_{med}=\frac{1}{median(\Delta t_i)}
\]

Jitter period P95:

\[
j_{95}=P_{95}(|\Delta t_i-median(\Delta t)|)
\]

Source `navigation_metrics.py` juga menghitung relative position error sekitar horizon satu detik dengan membandingkan perpindahan estimator dan ground truth:

\[
e_{RPE}=\| (p_{est,i+h}-p_{est,i})-(p_{gt,i+h}-p_{gt,i}) \|_2
\]

Nilai akhir yang disimpan adalah P95 dari seluruh `e_RPE` yang valid.
### 36.1 Potongan program evaluasi numerik

```python
def wrap(x):
    return np.arctan2(np.sin(x),np.cos(x))

def p95(x):
    return float(np.percentile(np.abs(x[np.isfinite(x)]),95))

pe=np.hypot(ex-gx,ey-gy)
ye=wrap(eyaw-gyaw)
dt=np.diff(t)
good=dt[(dt>0)&np.isfinite(dt)]

m={
    'position_rmse_m':float(np.sqrt(np.mean(pe*pe))),
    'position_p95_m':float(np.percentile(pe,95)),
    'final_position_error_m':float(pe[-1]),
    'yaw_rmse_rad':float(np.sqrt(np.mean(ye*ye))),
    'yaw_p95_rad':p95(ye),
    'rate_hz_median':float(1/np.median(good)),
    'period_jitter_p95_sec':float(np.percentile(np.abs(good-np.median(good)),95))
}
```
## 37. Metrik evaluasi global planner yang sejalan dengan laporan

Panjang jalur global:

\[
L_{path}=\sum_{i=1}^{N-1}\sqrt{(x_{i+1}-x_i)^2+(y_{i+1}-y_i)^2}
\]

Waktu perencanaan:

\[
t_{plan}=t_{finish}-t_{start}
\]

Endpoint error:

\[
e_{goal}=\sqrt{(x_{end}-x_{goal})^2+(y_{end}-y_{goal})^2}
\]

Success rate:

\[
SR=\frac{N_{success}}{N_{total}}\times100\%
\]

Minimum clearance terhadap obstacle:

\[
d_{min}=\min_i d(p_i,O)
\]

`p_i` adalah titik jalur ke-i dan `O` adalah himpunan obstacle. Untuk eksperimen planner, metrik tersebut sebaiknya dianalisis bersama tracking RMSE dan kejadian steering saturation agar jalur yang pendek tidak otomatis dianggap paling baik bila tidak layak diikuti kendaraan.