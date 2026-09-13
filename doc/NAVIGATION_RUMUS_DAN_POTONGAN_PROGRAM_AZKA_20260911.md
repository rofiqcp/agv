# Rumus, Parameter YAML, dan Potongan Program Sistem Navigasi ADV

Dokumen ini disusun untuk kebutuhan laporan **Azka Bintang Nur Taqwa** dengan struktur setiap pokok bahasan: **(1) rumus dan variabel, (2) YAML yang berpengaruh, (3) program yang merepresentasikan rumus**.

## Basis source yang digunakan

- Implementasi kendaraan: `/home/sirobo/agv/src`
- Referensi EKF: `/home/sirobo/agv/referensi/navigasi/reference_repos/02_robot_localization`
- Referensi planning: `/home/sirobo/agv/referensi/navigasi/reference_repos/01_navigation2`
- EKF pada sistem aktual dijalankan oleh package `robot_localization`; karena itu rumus dan potongan program EKF di bawah mengikuti implementasi `robot_localization/src/ekf.cpp` dan urutan state pada `filter_common.hpp`.
- Global planner aktual menggunakan `nav2_smac_planner/SmacPlannerHybrid`; karena itu rumus dan potongan program planning mengikuti source Navigation2, terutama `nav2_smac_planner` dan `nav2_costmap_2d`.

> Catatan laporan: nilai YAML yang ditampilkan adalah snapshot source aktual. Nilai yang masih berstatus kalibrasi/fallback tidak boleh dinyatakan sebagai hasil pengukuran fisik sebelum pengujian lapangan tervalidasi.

---

# A. Pra-pengolahan Sensor dan Odometri

## 1. Transformasi WGS84 menjadi koordinat lokal East-North

### 1.1 Rumus lengkap dan penjelasan variabel

Untuk area operasi lokal, proyeksi yang dipakai adalah pendekatan tangent-plane/equirectangular:

\[
\bar{\varphi}=\frac{\varphi+\varphi_0}{2}
\]
\[
E=(\lambda-\lambda_0)\cos(\bar{\varphi})R_E
\]

\[
N=(\varphi-\varphi_0)R_E
\]

Keterangan: \(E\) = perpindahan arah timur (m), \(N\) = perpindahan arah utara (m), \(\varphi,\lambda\) = latitude dan longitude GNSS dalam radian, \(\varphi_0,\lambda_0\) = titik referensi geografis, \(R_E=6378137\) m = radius bumi yang digunakan source, dan \(\bar{\varphi}\) = latitude rata-rata.

### 1.2 Potongan YAML yang berpengaruh

```yaml
localization_core:
  ros__parameters:
    reference_latitude: -7.050964918716
    reference_longitude: 110.435059847106
    reference_map_x_m: 167.493142
    reference_map_y_m: 94.933756
    map_yaw_from_enu_rad: 0.0
```

Parameter yang diedit saat titik datum berubah adalah `reference_latitude`, `reference_longitude`, `reference_map_x_m`, `reference_map_y_m`, dan `map_yaw_from_enu_rad`.

### 1.3 Potongan program yang merepresentasikan rumus

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

Source: `src/navigation/include/navigation/navigation_math.hpp`.

---

## 2. Transformasi East-North ke frame map

### 2.1 Rumus lengkap dan penjelasan variabel

\[
x_m=x_0+E\cos\theta-N\sin\theta
\]

\[
y_m=y_0+E\sin\theta+N\cos\theta
\]

\[
\psi_m=\operatorname{wrap}(\psi_{ENU}+\theta)
\]

Keterangan: \(x_m,y_m\) = posisi pada frame map, \(x_0,y_0\) = koordinat titik referensi pada map, \(E,N\) = koordinat East-North, \(\theta\) = rotasi map terhadap ENU, dan \(\psi_m\) = yaw pada frame map.
### 2.2 Potongan YAML yang berpengaruh

```yaml
localization_core:
  ros__parameters:
    reference_map_x_m: 167.493142
    reference_map_y_m: 94.933756
    map_yaw_from_enu_rad: 0.0
    map_calibration_x_m: 0.0
    map_calibration_y_m: 0.0
    map_calibration_yaw_rad: 0.0
```

### 2.3 Potongan program yang merepresentasikan rumus

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

Source: `src/navigation/include/navigation/navigation_math.hpp`.

---
## 3. Koreksi posisi antena GNSS ke base_footprint

### 3.1 Rumus lengkap dan penjelasan variabel

Offset antena pada body kendaraan harus diputar ke frame map sebelum dikurangkan dari posisi antena:

\[
x_b=x_a-(\cos\psi\,r_x-\sin\psi\,r_y)
\]

\[
y_b=y_a-(\sin\psi\,r_x+\cos\psi\,r_y)
\]

Keterangan: \(x_a,y_a\) = posisi antena pada map, \(x_b,y_b\) = posisi `base_footprint`, \(r_x,r_y\) = offset antena terhadap pusat kendaraan pada body frame, dan \(\psi\) = yaw kendaraan.

### 3.2 Potongan YAML yang berpengaruh

```yaml
localization_core:
  ros__parameters:
    gnss_antenna_x_m: 0.165
    gnss_antenna_y_m: 0.0
```

Nilai ini harus diedit mengikuti hasil pengukuran mekanik pemasangan antena.

### 3.3 Potongan program yang merepresentasikan rumus

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

Source: `src/navigation/include/navigation/navigation_math.hpp`.

---

## 4. Pra-pengolahan IMU: konversi unit, bias, dan scale

### 4.1 Rumus lengkap dan penjelasan variabel

Konversi akselerometer mentah 16-bit ke SI:

\[
a=\frac{n_a}{32768}\times 16g
\]

Konversi gyroscope mentah ke derajat per detik:

\[
\omega_{deg/s}=\frac{n_g}{32768}\times 2000
\]

Konversi ke rad/s dan koreksi bias:

\[
\omega_{corr}=s_\omega\omega_{deg/s}\frac{\pi}{180}-b_\omega
\]

Koreksi akselerasi setiap sumbu:
\[
a_{corr}=(s_a a-b_a)k_a
\]

Keterangan: \(n_a,n_g\) = data mentah sensor, \(g=9.80665\text{ m/s}^2\), \(s_a,s_\omega\in\{-1,+1\}\) = koreksi arah sumbu, \(b_a,b_\omega\) = bias hasil kalibrasi, dan \(k_a\) = scale factor akselerometer.

### 4.2 Potongan YAML yang berpengaruh

```yaml
data_imu_node:
  ros__parameters:
    vector_x_sign: -1.0
    vector_y_sign: -1.0
    vector_z_sign: 1.0
    accel_bias: [0.0010990763825818317, -0.0007872455352585517, -0.00575776350095758]
    accel_scale: [1.0, 1.0, 1.0]
    gyro_bias: [0.0, 0.0, 0.0]
    angular_velocity_covariance: [0.0009, 0.0009, 0.0004]
    linear_acceleration_covariance: [8.706300385203908e-05, 7.532525585671975e-05, 0.0001076978576337632]
```

Parameter yang terutama diedit setelah kalibrasi adalah `accel_bias`, `accel_scale`, `gyro_bias`, serta covariance pengukuran.

### 4.3 Potongan program yang merepresentasikan rumus

```cpp
ax_ = (v0 / 32768.0) * 16.0 * 9.80665;
ay_ = (v1 / 32768.0) * 16.0 * 9.80665;
az_ = (v2 / 32768.0) * 16.0 * 9.80665;

gx_ = (v0 / 32768.0) * 2000.0;
gy_ = (v1 / 32768.0) * 2000.0;
gz_ = (v2 / 32768.0) * 2000.0;
msg.angular_velocity.x = vector_x_sign_ * gx_ * M_PI / 180.0 - gyro_bias_[0];
msg.angular_velocity.y = vector_y_sign_ * gy_ * M_PI / 180.0 - gyro_bias_[1];
msg.angular_velocity.z = vector_z_sign_ * gz_ * M_PI / 180.0 - gyro_bias_[2];

msg.linear_acceleration.x = (vector_x_sign_ * ax_ - accel_bias_[0]) * accel_scale_[0];
msg.linear_acceleration.y = (vector_y_sign_ * ay_ - accel_bias_[1]) * accel_scale_[1];
msg.linear_acceleration.z = (vector_z_sign_ * az_ - accel_bias_[2]) * accel_scale_[2];
```

Source: `src/navigation/src/imu_node.cpp`.

---

## 5. Konversi Euler ke quaternion IMU

### 5.1 Rumus lengkap dan penjelasan variabel

Dengan \(c_r=\cos(\phi/2)\), \(s_r=\sin(\phi/2)\), \(c_p=\cos(\theta/2)\), \(s_p=\sin(\theta/2)\), \(c_y=\cos(\psi/2)\), dan \(s_y=\sin(\psi/2)\):

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

Keterangan: \(\phi\) = roll, \(\theta\) = pitch, \(\psi\) = yaw, dan \(q_x,q_y,q_z,q_w\) = komponen quaternion.
### 5.2 Potongan YAML yang berpengaruh

```yaml
data_imu_node:
  ros__parameters:
    publish_orientation: true
    invert_roll: true
    invert_pitch: true
    yaw_sign: 1.0
    roll_offset_rad: 0.0
    pitch_offset_rad: 0.0
    yaw_offset_rad: 0.0
    magnetic_declination_radians: 0.0
    orientation_covariance: [0.0025, 0.0025, 0.0076154355]
```

### 5.3 Potongan program yang merepresentasikan rumus

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

Source: `src/navigation/src/imu_node.cpp`.

---
## 6. Odometri kinematik kendaraan Ackermann

### 6.1 Rumus lengkap dan penjelasan variabel

Source aktual memperlakukan sudut steering feedback sebagai sudut roda dalam. Radius pusat kendaraan dihitung:

\[
R_c=\frac{T}{2}+\frac{L}{\tan |\delta|}
\]

Kurvatur dengan konvensi ROS dan steering kanan-positif:

\[
\kappa=-\operatorname{sgn}(\delta)\frac{1}{R_c}
\]

Yaw-rate:

\[
\omega=v\kappa
\]

Integrasi pose diskrit:

\[
\psi_k=\psi_{k-1}+\omega_k\Delta t
\]
\[
x_k=x_{k-1}+v_k\cos(\psi_k)\Delta t
\]
\[
y_k=y_{k-1}+v_k\sin(\psi_k)\Delta t
\]

Keterangan: \(L\) = wheelbase, \(T\) = track width, \(\delta\) = steering angle, \(R_c\) = radius lintasan pusat kendaraan, \(v\) = kecepatan longitudinal, \(\omega\) = yaw-rate, dan \(\Delta t\) = selang waktu.
### 6.2 Potongan YAML yang berpengaruh

```yaml
vehicle:
  ros__parameters:
    wheelbase_m: 0.7
    track_width_m: 0.48
    drive_erpm_per_mps: 8000.0
    drive_odometry_calibration_scale: 1.0
    minimum_turning_radius_m: 1.597679121829
    max_yaw_rate_rps: 0.625907910003
```

Parameter `drive_odometry_calibration_scale` harus diperbarui dari pengujian jarak aktual, sedangkan `minimum_turning_radius_m` saat ini masih berstatus fallback teoretis pada `vehicle.yaml`.

### 6.3 Potongan program yang merepresentasikan rumus

```cpp
const double drive_raw_mps = rawDriveMpsFromErpm(rpm) * (invert_drive_ ? -1.0 : 1.0);
const double drive_mps = drive_raw_mps * drive_odometry_calibration_scale_;
const double steering_rad = steering_calibrated_deg * kPi / 180.0;
double yaw_rate = 0.0;
if (std::abs(steering_rad) > 1.0e-6) {
  const double center_radius = 0.5 * track_width_m_ +
    wheelbase_m_ / std::tan(std::abs(steering_rad));
  if (std::isfinite(center_radius) && center_radius > 1.0e-6) {
    const double ros_curvature = -std::copysign(1.0 / center_radius, steering_rad);
    yaw_rate = drive_mps * ros_curvature;
  }
}
```

Source: `src/esc/src/ackermann_controller_server.cpp`.
```cpp
const auto stamp = now();
double dt = 0.0;
if (odom_time_valid_) {
  dt = std::clamp((stamp - last_odom_time_).seconds(), 0.0, 0.20);
}
last_odom_time_ = stamp;
odom_time_valid_ = true;

if (dt > 0.0) {
  odom_yaw_ += yaw_rate * dt;
  odom_x_ += drive_mps * std::cos(odom_yaw_) * dt;
  odom_y_ += drive_mps * std::sin(odom_yaw_) * dt;
}

odom.pose.pose.position.x = odom_x_;
odom.pose.pose.position.y = odom_y_;
odom.pose.pose.orientation.z = std::sin(odom_yaw_ * 0.5);
odom.pose.pose.orientation.w = std::cos(odom_yaw_ * 0.5);
odom.twist.twist.linear.x = drive_mps;
odom.twist.twist.angular.z = yaw_rate;
```

---

# B. Extended Kalman Filter sesuai `robot_localization`

Bagian B harus dijadikan pengganti utama penjelasan EKF pada laporan. Implementasi perhitungan matriks tidak berada pada custom `navigation_core`, tetapi pada package referensi `robot_localization` yang juga digunakan runtime ROS 2.

## 7. Vektor keadaan EKF 15 dimensi
### 7.1 Rumus lengkap dan penjelasan variabel

Urutan state yang benar-benar digunakan `robot_localization` adalah:

\[
\mathbf{x}=
[x,y,z,\phi,\theta,\psi,v_x,v_y,v_z,\omega_x,\omega_y,\omega_z,a_x,a_y,a_z]^T
\]

Keterangan: \(x,y,z\) = posisi; \(\phi,\theta,\psi\) = roll, pitch, yaw; \(v_x,v_y,v_z\) = kecepatan linear; \(\omega_x,\omega_y,\omega_z\) = kecepatan sudut roll, pitch, yaw; \(a_x,a_y,a_z\) = percepatan linear. Pada mode planar `two_d_mode: true`, komponen 3D dibatasi sehingga estimasi difokuskan pada bidang X-Y.

### 7.2 Potongan YAML yang berpengaruh

```yaml
ekf_filter_node_odom:
  ros__parameters:
    two_d_mode: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_footprint
    world_frame: odom
```

Urutan semua `*_config` mengikuti tepat 15 elemen state di atas.

### 7.3 Potongan program yang merepresentasikan rumus

```cpp
enum StateMembers
{
  StateMemberX = 0,
  StateMemberY,
  StateMemberZ,
  StateMemberRoll,
  StateMemberPitch,
  StateMemberYaw,
  StateMemberVx,
  StateMemberVy,
  StateMemberVz,
  StateMemberVroll,
  StateMemberVpitch,
  StateMemberVyaw,
  StateMemberAx,
  StateMemberAy,
  StateMemberAz
};

const int STATE_SIZE = 15;
```

Source referensi: `reference_repos/02_robot_localization/include/robot_localization/filter_common.hpp`.

---

## 8. Tahap prediksi state EKF

### 8.1 Rumus lengkap dan penjelasan variabel

Bentuk umum EKF:

\[
\hat{\mathbf{x}}^-_k=f(\hat{\mathbf{x}}_{k-1},\mathbf{u}_k,\Delta t)
\]

Pada source `robot_localization`, fungsi nonlinear direpresentasikan melalui `transfer_function_` yang dibentuk dari roll, pitch, yaw, kecepatan, percepatan, dan \(\Delta t\), lalu diterapkan:

\[
\hat{\mathbf{x}}^-_k=\mathbf{A}(\hat{\mathbf{x}}_{k-1},\Delta t)\hat{\mathbf{x}}_{k-1}
\]

Untuk gerak planar, bentuk yang relevan dapat dibaca sebagai \(x_k\approx x_{k-1}+v_x\cos\psi\Delta t\), \(y_k\approx y_{k-1}+v_x\sin\psi\Delta t\), dan \(\psi_k\approx\psi_{k-1}+\omega_z\Delta t\), tetapi program tetap memakai model state penuh 15-D.
Keterangan: \(\hat{\mathbf{x}}^-_k\) = state prediksi sebelum koreksi, \(f(\cdot)\) = model gerak nonlinear, \(\mathbf{u}_k\) = input kontrol bila `use_control=true`, dan \(\Delta t\) = interval prediksi.

### 8.2 Potongan YAML yang berpengaruh

```yaml
ekf_filter_node_odom:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.25
    predict_to_current_time: true
    use_control: false

ekf_filter_node_map:
  ros__parameters:
    frequency: 10.0
    sensor_timeout: 2.0
    predict_to_current_time: false
    use_control: false
```

`frequency` menentukan frekuensi keluaran filter. `sensor_timeout` menentukan kapan filter menjalankan prediksi tanpa koreksi baru. `predict_to_current_time` menentukan apakah state diproyeksikan lagi hingga waktu sekarang.

### 8.3 Potongan program yang merepresentasikan rumus

```cpp
const double delta_sec = filter_utilities::toSec(delta);

double roll = state_(StateMemberRoll);
double pitch = state_(StateMemberPitch);
double yaw = state_(StateMemberYaw);
double x_vel = state_(StateMemberVx);
double y_vel = state_(StateMemberVy);
double z_vel = state_(StateMemberVz);
double pitch_vel = state_(StateMemberVpitch);
double yaw_vel = state_(StateMemberVyaw);
```
```cpp
transfer_function_(StateMemberX, StateMemberVx) = cy * cp * delta_sec;
transfer_function_(StateMemberY, StateMemberVx) = sy * cp * delta_sec;
transfer_function_(StateMemberRoll, StateMemberVroll) = delta_sec;
transfer_function_(StateMemberPitch, StateMemberVpitch) = cr * delta_sec;
transfer_function_(StateMemberPitch, StateMemberVyaw) = -sr * delta_sec;
transfer_function_(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta_sec;
transfer_function_(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta_sec;
transfer_function_(StateMemberVx, StateMemberAx) = delta_sec;
transfer_function_(StateMemberVy, StateMemberAy) = delta_sec;
transfer_function_(StateMemberVz, StateMemberAz) = delta_sec;

state_ = transfer_function_ * state_;
wrapStateAngles();
```

Source referensi: `reference_repos/02_robot_localization/src/ekf.cpp`.

---

## 9. Prediksi covariance EKF dan process noise

### 9.1 Rumus lengkap dan penjelasan variabel

Persamaan yang sesuai source adalah:

\[
\mathbf{P}^-_k=\mathbf{J}_k\mathbf{P}_{k-1}\mathbf{J}_k^T+\Delta t\,\mathbf{Q}_k
\]

Keterangan: \(\mathbf{P}^-_k\) = covariance prediksi, \(\mathbf{J}_k\) = Jacobian dari fungsi transisi, \(\mathbf{P}_{k-1}\) = covariance sebelumnya, \(\mathbf{Q}_k\) = process noise covariance, dan \(\Delta t\) = interval prediksi. Source `robot_localization` secara eksplisit mengalikan process noise dengan `delta_sec`.
### 9.2 Potongan YAML yang berpengaruh

```yaml
ekf_filter_node_odom:
  ros__parameters:
    dynamic_process_noise_covariance: false
    process_noise_covariance:
      [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

Potongan di atas hanya menunjukkan dua baris pertama matriks aktual. Untuk tuning, elemen diagonal mengikuti urutan state: `x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az`. Diagonal aktual kedua EKF adalah `[0.05, 0.05, 0.06, 0.03, 0.03, 0.06, 0.025, 0.025, 0.04, 0.01, 0.01, 0.02, 0.01, 0.01, 0.015]`.

### 9.3 Potongan program yang merepresentasikan rumus

```cpp
transfer_function_jacobian_ = transfer_function_;
transfer_function_jacobian_(StateMemberX, StateMemberRoll) = dFx_dR;
transfer_function_jacobian_(StateMemberX, StateMemberPitch) = dFx_dP;
transfer_function_jacobian_(StateMemberX, StateMemberYaw) = dFx_dY;
transfer_function_jacobian_(StateMemberY, StateMemberRoll) = dFy_dR;
transfer_function_jacobian_(StateMemberY, StateMemberPitch) = dFy_dP;
transfer_function_jacobian_(StateMemberY, StateMemberYaw) = dFy_dY;
```

```cpp
estimate_error_covariance_ =
  transfer_function_jacobian_ * estimate_error_covariance_ *
  transfer_function_jacobian_.transpose();
estimate_error_covariance_.noalias() += delta_sec * (*process_noise_covariance);
```

Source referensi: `reference_repos/02_robot_localization/src/ekf.cpp`.
---

## 10. Inovasi, Kalman gain, dan koreksi state

### 10.1 Rumus lengkap dan penjelasan variabel

Untuk measurement linear pada subset state, innovation adalah:

\[
\mathbf{y}_k=\mathbf{z}_k-\mathbf{H}_k\hat{\mathbf{x}}^-_k
\]

Covariance innovation:

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

Keterangan: \(\mathbf{z}_k\) = measurement sensor, \(\mathbf{H}_k\) = matriks pemetaan state ke measurement, \(\mathbf{R}_k\) = measurement covariance, \(\mathbf{S}_k\) = covariance innovation, dan \(\mathbf{K}_k\) = Kalman gain.
### 10.2 Potongan YAML yang berpengaruh

```yaml
ekf_filter_node_odom:
  ros__parameters:
    odom0: /esc/odom
    odom0_config: [false, false, false, false, false, false, true, false, false, false, false, false, false, false, false]
    odom0_twist_rejection_threshold: 5.0
    imu0: /imu/data
    imu0_config: [false, false, false, false, false, false, false, false, false, false, false, true, false, false, false]
    twist0: /gnss/base_velocity_fusion
    twist0_config: [false, false, false, false, false, false, true, false, false, false, false, false, false, false, false]
    twist0_rejection_threshold: 10.0
```

`*_config` menentukan komponen measurement yang membentuk \(\mathbf{H}_k\). Nilai covariance pada pesan sensor menjadi \(\mathbf{R}_k\), sedangkan `*_rejection_threshold` memengaruhi penerimaan innovation.

### 10.3 Potongan program yang merepresentasikan rumus

```cpp
Eigen::MatrixXd pht =
  estimate_error_covariance_ * state_to_measurement_subset.transpose();
Eigen::MatrixXd hphr_inverse =
  (state_to_measurement_subset * pht + measurement_covariance_subset).inverse();
kalman_gain_subset.noalias() = pht * hphr_inverse;
innovation_subset = measurement_subset - state_subset;

state_.noalias() += kalman_gain_subset * innovation_subset;
wrapStateAngles();
```

Source referensi: `reference_repos/02_robot_localization/src/ekf.cpp`.
---

## 11. Mahalanobis gate untuk menolak outlier measurement

### 11.1 Rumus lengkap dan penjelasan variabel

Source `robot_localization` menguji squared Mahalanobis distance:

\[
d_M^2=\mathbf{y}_k^T\mathbf{S}_k^{-1}\mathbf{y}_k
\]

Measurement diterima apabila:

\[
d_M^2<n_\sigma^2
\]

Keterangan: \(\mathbf{y}_k\) = innovation, \(\mathbf{S}_k^{-1}\) = invers covariance innovation, dan \(n_\sigma\) = threshold rejection yang dikonfigurasi untuk sumber measurement.

### 11.2 Potongan YAML yang berpengaruh

```yaml
ekf_filter_node_map:
  ros__parameters:
    odom0_pose_rejection_threshold: 12.0
    twist0_rejection_threshold: 10.0
    pose0_rejection_threshold: 4.0
    pose1_rejection_threshold: 4.0
```

Parameter ini tidak boleh diperlakukan sebagai error meter/radian langsung; pada `robot_localization` parameter tersebut digunakan sebagai batas Mahalanobis berbasis covariance.

### 11.3 Potongan program yang merepresentasikan rumus

```cpp
double squared_mahalanobis =
  innovation.dot(innovation_covariance * innovation);
double threshold = n_sigmas * n_sigmas;
return squared_mahalanobis < threshold;
```

Source referensi: `reference_repos/02_robot_localization/src/filter_base.cpp`.
---

## 12. Pembaruan covariance menggunakan Joseph form

### 12.1 Rumus lengkap dan penjelasan variabel

Setelah measurement diterima, source `robot_localization` tidak memakai bentuk sederhana \((I-KH)P\), tetapi Joseph form:

\[
\mathbf{P}_k=(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\mathbf{P}^-_k(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)^T+\mathbf{K}_k\mathbf{R}_k\mathbf{K}_k^T
\]

Keterangan: \(\mathbf{I}\) = matriks identitas, \(\mathbf{K}_k\) = Kalman gain, \(\mathbf{H}_k\) = state-to-measurement matrix, \(\mathbf{P}^-_k\) = covariance sebelum koreksi, dan \(\mathbf{R}_k\) = covariance measurement.

Joseph form penting untuk laporan karena ini adalah bentuk yang benar-benar diterapkan oleh source referensi.

### 12.2 Potongan YAML yang berpengaruh

```yaml
data_imu_node:
  ros__parameters:
    angular_velocity_covariance: [0.0009, 0.0009, 0.0004]

data_cuav_node:
  ros__parameters:
    poll_nav_cov: true
    nav_cov_poll_rate_hz: 5.0
```

Selain YAML tersebut, covariance `/esc/odom`, `/gnss/base_velocity_fusion`, `/odometry/gnss_map`, dan heading fusion yang dipublikasikan masing-masing node menjadi bagian \(\mathbf{R}_k\).

### 12.3 Potongan program yang merepresentasikan rumus

```cpp
Eigen::MatrixXd gain_residual = identity_;
gain_residual.noalias() -= kalman_gain_subset * state_to_measurement_subset;
estimate_error_covariance_ =
  gain_residual * estimate_error_covariance_ * gain_residual.transpose();
estimate_error_covariance_.noalias() += kalman_gain_subset *
  measurement_covariance_subset * kalman_gain_subset.transpose();
```

Source referensi: `reference_repos/02_robot_localization/src/ekf.cpp`.
---

## 13. EKF lokal pada frame odom

### 13.1 Rumus lengkap dan penjelasan variabel

Berdasarkan boolean `*_config` aktual, measurement utama EKF lokal dapat diringkas:

\[
\mathbf{z}_{local}=
\begin{bmatrix}
v_{x,esc}\\
\omega_{z,imu}\\
v_{x,gnss}
\end{bmatrix}
\]

Setiap measurement dipetakan ke state 15-D melalui baris matriks \(\mathbf{H}\) yang mengaktifkan indeks `Vx` atau `Vyaw`. Secara konseptual:

\[
z_i=H_i\mathbf{x}+v_i,\qquad v_i\sim\mathcal{N}(0,R_i)
\]

Keterangan: \(v_{x,esc}\) = kecepatan longitudinal dari odometri ESC, \(\omega_{z,imu}\) = yaw-rate IMU, dan \(v_{x,gnss}\) = velocity GNSS yang sudah melalui gate pada LocalizationCore.

### 13.2 Potongan YAML yang berpengaruh

```yaml
ekf_filter_node_odom:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.25
    two_d_mode: true
    predict_to_current_time: true
    publish_tf: true
    world_frame: odom
    odom0: /esc/odom
    odom0_config: [false, false, false, false, false, false, true, false, false, false, false, false, false, false, false]
    imu0: /imu/data
    imu0_config: [false, false, false, false, false, false, false, false, false, false, false, true, false, false, false]
    twist0: /gnss/base_velocity_fusion
    twist0_config: [false, false, false, false, false, false, true, false, false, false, false, false, false, false, false]
```
### 13.3 Potongan program yang merepresentasikan implementasi

```python
local_ekf = Node(
    package='robot_localization', executable='ekf_node', name='ekf_filter_node_odom',
    output='screen', respawn=True, respawn_delay=2.0,
    parameters=[ekf_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    remappings=[('odometry/filtered', '/odometry/filtered'), ('set_pose', '/ekf_local/set_pose')],
)
```

Source runtime: `src/navigation/launch/autonomous.launch.py`.

Pada `robot_localization`, pemilihan komponen measurement direalisasikan dengan matriks \(H\) subset:

```cpp
for (size_t i = 0; i < update_size; ++i) {
  state_to_measurement_subset(i, update_indices[i]) = 1;
}
```

Source referensi: `reference_repos/02_robot_localization/src/ekf.cpp`.

---

## 14. EKF global pada frame map

### 14.1 Rumus lengkap dan penjelasan variabel

Measurement aktif pada konfigurasi global dapat diringkas:

\[
\mathbf{z}_{global}=
[x_{GNSS},y_{GNSS},v_{x,GNSS},\psi_{COG},\psi_{heading},\omega_{z,IMU}]^T
\]

Masing-masing measurement dikoreksi ke state dengan persamaan EKF yang sama:

\[
\hat{\mathbf{x}}_k=\hat{\mathbf{x}}^-_k+\mathbf{K}_k(\mathbf{z}_k-\mathbf{H}_k\hat{\mathbf{x}}^-_k)
\]
Keterangan: \(x_{GNSS},y_{GNSS}\) berasal dari `/odometry/gnss_map`; \(v_{x,GNSS}\) dari `/gnss/base_velocity_fusion`; \(\psi_{COG}\) dari `/gnss/cog_heading_fusion`; \(\psi_{heading}\) dari `/heading/validated_fusion`; dan \(\omega_{z,IMU}\) dari `/imu/data`.

### 14.2 Potongan YAML yang berpengaruh

```yaml
ekf_filter_node_map:
  ros__parameters:
    frequency: 10.0
    sensor_timeout: 2.0
    two_d_mode: true
    predict_to_current_time: false
    publish_tf: false
    world_frame: map
    odom0: /odometry/gnss_map
    odom0_config: [true, true, false, false, false, false, false, false, false, false, false, false, false, false, false]
    twist0: /gnss/base_velocity_fusion
    twist0_config: [false, false, false, false, false, false, true, false, false, false, false, false, false, false, false]
    pose0: /gnss/cog_heading_fusion
    pose0_config: [false, false, false, false, false, true, false, false, false, false, false, false, false, false, false]
    pose1: /heading/validated_fusion
    pose1_config: [false, false, false, false, false, true, false, false, false, false, false, false, false, false, false]
    imu0: /imu/data
    imu0_config: [false, false, false, false, false, false, false, false, false, false, false, true, false, false, false]
```

### 14.3 Potongan program yang merepresentasikan implementasi

```python
global_ekf = Node(
    package='robot_localization', executable='ekf_node', name='ekf_filter_node_map',
    output='screen', respawn=True, respawn_delay=2.0,
    parameters=[ekf_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    remappings=[('odometry/filtered', '/odometry/filtered_map'), ('set_pose', '/ekf_global/set_pose')],
)
```

Source runtime: `src/navigation/launch/autonomous.launch.py`.
---

# C. Transformasi Global dan Koreksi Lokalisasi

## 15. Pembentukan transformasi `map -> odom`

### 15.1 Rumus lengkap dan penjelasan variabel

Hubungan frame memenuhi:

\[
\mathbf{T}_{map}^{base}=\mathbf{T}_{map}^{odom}\mathbf{T}_{odom}^{base}
\]

Sehingga:

\[
\mathbf{T}_{map}^{odom}=\mathbf{T}_{map}^{base}\left(\mathbf{T}_{odom}^{base}\right)^{-1}
\]

Untuk SE(2), yaw anchor adalah:

\[
\psi_{mo}=\operatorname{wrap}(\psi_{mb}-\psi_{ob})
\]

Dengan \(c=\cos\psi_{mo}\) dan \(s=\sin\psi_{mo}\):

\[
x_{mo}=x_{mb}-(c x_{ob}-s y_{ob})
\]
\[
y_{mo}=y_{mb}-(s x_{ob}+c y_{ob})
\]

Keterangan: subskrip `mb` = map-to-base, `ob` = odom-to-base, dan `mo` = map-to-odom.
### 15.2 Potongan YAML yang berpengaruh

```yaml
localization_core:
  ros__parameters:
    map_frame: map
    odom_frame: odom
    base_frame: base_footprint
    local_odom_topic: /odometry/filtered
    global_odom_topic: /odometry/filtered_map
    tf_publish_rate_hz: 30.0
    tf_future_offset_sec: 0.05
```

### 15.3 Potongan program yang merepresentasikan rumus

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

Source: `src/navigation/include/navigation/navigation_math.hpp`.

---

## 16. Koreksi bertahap transformasi map-odom
### 16.1 Rumus lengkap dan penjelasan variabel

Kandidat transformasi absolut dari GNSS/global pose terhadap odometri lokal dinyatakan sebagai \(\mathbf{T}^{cand}_{mo}\). Koreksi translasi diterapkan secara bertahap:

\[
\mathbf{p}^{blend}_{mo}=\mathbf{p}_{mo}+\alpha\left(\mathbf{p}^{cand}_{mo}-\mathbf{p}_{mo}\right)
\]

\[
\Delta\mathbf{p}=\mathbf{p}^{blend}_{mo}-\mathbf{p}_{mo}
\]

Jika \(\|\Delta\mathbf{p}\|>d_{max}\), koreksi dibatasi:

\[
\Delta\mathbf{p}\leftarrow \Delta\mathbf{p}\frac{d_{max}}{\|\Delta\mathbf{p}\|}
\]

Untuk koreksi yaw dari EKF global:

\[
e_\psi=\operatorname{wrap}(\psi_{global}-\psi_{map,current})
\]

\[
\Delta\psi=\operatorname{clip}(\alpha_\psi e_\psi,-\Delta\psi_{max},\Delta\psi_{max})
\]

Keterangan: \(\alpha\) = gain koreksi translasi; \(d_{max}\) = koreksi translasi maksimum per update; \(\alpha_\psi\) = gain koreksi yaw; \(e_\psi\) = innovation yaw; \(\Delta\psi_{max}\) = langkah koreksi yaw maksimum.### 16.2 Potongan YAML yang berpengaruh

```yaml
localization_core:
  ros__parameters:
    strict_correction_alpha: 0.08
    freeze_stationary_map_translation: true
    strict_moving_correction_alpha: 0.01
    enable_wheel_slip_pose_correction: false
    strict_slip_correction_alpha: 0.1
    strict_max_correction_m: 0.08
    use_global_ekf_yaw_for_map_correction: true
    global_ekf_yaw_correction_alpha: 0.05
    global_ekf_yaw_max_step_rad: 0.00872664626
    global_ekf_yaw_max_innovation_rad: 2.3561944902
```

Parameter yang paling langsung memengaruhi besar koreksi adalah `strict_correction_alpha`, `strict_moving_correction_alpha`, `strict_max_correction_m`, `global_ekf_yaw_correction_alpha`, `global_ekf_yaw_max_step_rad`, dan `global_ekf_yaw_max_innovation_rad`.

### 16.3 Potongan program yang merepresentasikan rumus

```cpp
double alpha = slip_pose_correction ? strict_slip_correction_alpha_ :
  (stationary ? strict_correction_alpha_ : strict_moving_correction_alpha_);
if (stationary && freeze_stationary_map_translation_ && !slip_pose_correction) {
  alpha = 0.0;
}
auto blended = navigation_math::blendPose(anchor_map_odom_, candidate, alpha);
double dx = blended.x - anchor_map_odom_.x;
double dy = blended.y - anchor_map_odom_.y;
const double dxy = std::hypot(dx, dy);
``````cpp
if (dxy > strict_max_correction_m_ && dxy > 1.0e-9) {
  const double scale = strict_max_correction_m_ / dxy;
  dx *= scale;
  dy *= scale;
}
const double current_map_yaw = navigation_math::normalizeAngle(
  anchor_map_odom_.yaw + correction_odom.yaw);
double dyaw = 0.0;
if (use_global_ekf_yaw_for_map_correction_ && globalYawFusionFreshUnlocked()) {
  const double innovation = navigation_math::normalizeAngle(
    global_ekf_pose_.yaw - current_map_yaw);
  if (std::abs(innovation) <= global_ekf_yaw_max_innovation_rad_) {
    dyaw = std::clamp(
      global_ekf_yaw_correction_alpha_ * innovation,
      -global_ekf_yaw_max_step_rad_,
      global_ekf_yaw_max_step_rad_);
  }
}
anchor_map_odom_.x += dx;
anchor_map_odom_.y += dy;
anchor_map_odom_.yaw = navigation_math::normalizeAngle(anchor_map_odom_.yaw + dyaw);
```

Source runtime: `src/navigation/src/localization_core.cpp`.

---

# D. Global Costmap dan SMAC Hybrid A* Navigation2
## 17. Diskretisasi koordinat global costmap

### 17.1 Rumus lengkap dan penjelasan variabel

Konversi koordinat dunia ke indeks sel costmap mengikuti implementasi `Costmap2D::worldToMap()`:

\[
m_x=\left\lfloor\frac{x_w-x_0}{r}\right\rfloor
\]

\[
m_y=\left\lfloor\frac{y_w-y_0}{r}\right\rfloor
\]

Keterangan: \(x_w,y_w\) = koordinat dunia pada frame map; \(x_0,y_0\) = origin costmap; \(r\) = resolusi peta dalam meter/sel; \(m_x,m_y\) = indeks sel. Resolusi yang lebih kecil meningkatkan ketelitian spasial namun memperbesar jumlah sel dan biaya pencarian.

### 17.2 Potongan YAML yang berpengaruh

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      robot_base_frame: base_footprint
      resolution: 0.1
      track_unknown_space: false
```

Parameter utama yang dapat dituning pada hubungan ini adalah `resolution` dan `track_unknown_space`.### 17.3 Potongan program yang merepresentasikan rumus

```cpp
bool Costmap2D::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (wx < origin_x_ || wy < origin_y_) {
    return false;
  }
  mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
  my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
  if (mx < size_x_ && my < size_y_) {
    return true;
  }
  return false;
}
```

Source referensi: `reference_repos/01_navigation2/nav2_costmap_2d/src/costmap_2d.cpp`.

---

## 18. Inflation cost pada global costmap

### 18.1 Rumus lengkap dan penjelasan variabel

Untuk sel di luar radius inscribed kendaraan, Navigation2 menggunakan penurunan biaya eksponensial:

\[
C(d)=(C_{ins}-1)\exp[-k(d-r_{ins})]
\]
Dengan bentuk lengkap:

\[
C(d)=
\begin{cases}
C_{lethal}, & d=0\\
C_{ins}, & 0<d\le r_{ins}\\
(C_{ins}-1)e^{-k(d-r_{ins})}, & d>r_{ins}
\end{cases}
\]

Keterangan: \(d\) = jarak dunia ke obstacle; \(r_{ins}\) = inscribed radius footprint; \(k\) = `cost_scaling_factor`; \(C_{lethal}\) = lethal obstacle cost; \(C_{ins}\) = inscribed inflated obstacle cost. Pada source, jarak awal diberikan dalam satuan sel lalu dikalikan `resolution_` sebelum dibandingkan dengan radius fisik.

### 18.2 Potongan YAML yang berpengaruh

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      footprint: '[[0.52,0.30],[0.52,-0.30],[-0.52,-0.30],[-0.52,0.30]]'
      footprint_padding: 0.03
      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        enabled: true
        inflation_radius: 0.8
        cost_scaling_factor: 2.5
```

Parameter yang paling memengaruhi profil biaya adalah `footprint`, `footprint_padding`, `inflation_radius`, dan `cost_scaling_factor`.### 18.3 Potongan program yang merepresentasikan rumus

```cpp
inline unsigned char computeCost(double distance) const override
{
  unsigned char cost = 0;
  if (distance == 0) {
    cost = LETHAL_OBSTACLE;
  } else if (distance * resolution_ <= inscribed_radius_) {
    cost = INSCRIBED_INFLATED_OBSTACLE;
  } else {
    double factor = exp(
      -1.0 * cost_scaling_factor_ *
      (distance * resolution_ - inscribed_radius_));
    cost = static_cast<unsigned char>(
      (INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
  }
  return cost;
}
```

Source referensi: `reference_repos/01_navigation2/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp`.

---

## 19. Fungsi evaluasi A* pada SMAC Hybrid A*

### 19.1 Rumus lengkap dan penjelasan variabel

Prinsip pemilihan node pada A* adalah:

\[
f(n)=g(n)+h(n)
\]
Keterangan: \(g(n)\) = biaya akumulatif dari start menuju node \(n\); \(h(n)\) = estimasi biaya dari node \(n\) menuju goal; \(f(n)\) = prioritas node dalam open set. Pada implementasi Navigation2, nilai baru tetangga dihitung sebagai:

\[
g(n')=g(n)+c(n,n')
\]

kemudian dimasukkan ke priority queue dengan:

\[
f(n')=g(n')+h(n')
\]

### 19.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 0.5
    GridBased:
      plugin: nav2_smac_planner/SmacPlannerHybrid
      tolerance: 0.05
      max_iterations: 250000
      max_on_approach_iterations: 1000
      max_planning_time: 2.5
```

`max_iterations`, `max_on_approach_iterations`, `max_planning_time`, dan `tolerance` tidak mengubah bentuk \(f=g+h\), tetapi menentukan batas dan kondisi penghentian proses pencarian.

### 19.3 Potongan program yang merepresentasikan rumus

```cpp
g_cost = current_node->getAccumulatedCost() +
  current_node->getTraversalCost(neighbor);
if (g_cost < neighbor->getAccumulatedCost()) {
  neighbor->setAccumulatedCost(g_cost);
  neighbor->parent = current_node;
  addNode(g_cost + getHeuristicCost(neighbor), neighbor);
}
```
Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/include/nav2_smac_planner/a_star_impl.hpp`.

---

## 20. State Hybrid A* dan diskretisasi orientasi

### 20.1 Rumus lengkap dan penjelasan variabel

State pencarian Hybrid A* membawa posisi planar dan orientasi:

\[
\mathbf{s}=[x,y,\theta]^T
\]

Orientasi dibagi menjadi \(N_\theta\) bin dengan ukuran:

\[
\Delta\theta_{bin}=\frac{2\pi}{N_\theta}
\]

Untuk konfigurasi aktual \(N_\theta=48\), sehingga:

\[
\Delta\theta_{bin}=\frac{2\pi}{48}=0.1308997\;\text{rad}=7.5^\circ
\]

Keterangan: \(x,y\) = posisi node pada grid kontinu; \(\theta\) = orientasi kendaraan; \(N_\theta\) = jumlah `angle_quantization_bins`; \(\Delta\theta_{bin}\) = resolusi heading pencarian.### 20.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      motion_model_for_search: DUBIN
      angle_quantization_bins: 48
      minimum_turning_radius: 1.597679121829
```

### 20.3 Potongan program yang merepresentasikan rumus

```cpp
angle_quantizations = node->declare_or_get_parameter(
  name + ".angle_quantization_bins", 72);
_angle_bin_size = 2.0 * M_PI / angle_quantizations;
_angle_quantizations = static_cast<unsigned int>(angle_quantizations);
_motion_model_for_search = node->declare_or_get_parameter(
  name + ".motion_model_for_search", std::string("DUBIN"));
_motion_model = fromString(_motion_model_for_search);
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid_impl.hpp`.

---

## 21. Minimum turning radius dan pembentukan primitive Dubins
### 21.1 Rumus lengkap dan penjelasan variabel

Hubungan kinematik dasar kendaraan Ackermann terhadap batas steering dapat dituliskan:

\[
R_{min,theory}=\frac{L_{eff}}{\tan |\delta_{max}|}
\]

Namun `SmacPlannerHybrid` tidak menghitung radius dari steering; planner menerima `minimum_turning_radius` sebagai parameter. Radius dalam satuan grid yang dipakai pencarian adalah:

\[
R_g=\frac{R_{min}}{r\,D}
\]

Keterangan: \(L_{eff}\) = effective wheelbase; \(\delta_{max}\) = batas steering; \(R_{min}\) = radius minimum dunia (m); \(r\) = resolusi costmap (m/sel); \(D\) = `downsampling_factor`; \(R_g\) = radius pada koordinat grid.

Untuk primitive Dubins, source Navigation2 membentuk sudut awal berdasarkan chord minimum antarsel:

\[
\theta_c=2\sin^{-1}\left(\frac{\sqrt{2}}{2R_g}\right)
\]

\[
N_c=\max\left(1,\left\lceil\frac{\theta_c}{\Delta\theta_{bin}}\right\rceil\right),\qquad
\theta_p=N_c\Delta\theta_{bin}
\]
Geometri primitive belok kemudian dihitung sebagai:

\[
\Delta x=R_g\sin\theta_p
\]

\[
\Delta y=R_g-R_g\cos\theta_p=R_g(1-\cos\theta_p)
\]

\[
\Delta s=\sqrt{(\Delta x)^2+(\Delta y)^2}
\]

Untuk primitive melengkung, biaya jarak dasarnya berupa panjang busur:

\[
s_{arc}=R_{arc}\theta_{arc}
\]

Keterangan: \(\theta_c\) = sudut chord minimum; \(N_c\) = jumlah increment bin; \(\theta_p\) = perubahan orientasi primitive; \(\Delta x,\Delta y\) = perpindahan primitive dalam frame lokal; \(\Delta s\) = panjang chord; \(R_{arc}\) = radius primitive; \(s_{arc}\) = panjang busur.

### 21.2 Potongan YAML yang berpengaruh

```yaml
vehicle:
  ros__parameters:
    effective_wheelbase_m: 0.7
    max_steering_angle_rad: 0.523598775598
    minimum_turning_radius_m: 1.597679121829
``````yaml
planner_server:
  ros__parameters:
    GridBased:
      downsample_costmap: false
      downsampling_factor: 1
      angle_quantization_bins: 48
      minimum_turning_radius: 1.597679121829
      motion_model_for_search: DUBIN
```

Nilai `minimum_turning_radius` pada konfigurasi saat ini harus dibahas sebagai parameter planner aktif. Karena `steering_circle_calibration_valid` masih `false`, nilainya belum boleh disebut sebagai radius hasil pengukuran fisik.

### 21.3 Potongan program yang merepresentasikan rumus

```cpp
_minimum_turning_radius_global_coords =
  node->declare_or_get_parameter(name + ".minimum_turning_radius", 0.4);
_search_info.minimum_turning_radius =
  _minimum_turning_radius_global_coords /
  (_costmap->getResolution() * _downsampling_factor);
float angle = 2.0 * asin(
  sqrt(2.0) / (2 * min_turning_radius));
bin_size = 2.0f * static_cast<float>(M_PI) /
  static_cast<float>(num_angle_quantization);
float increments = angle < bin_size ? 1.0f : ceil(angle / bin_size);
angle = increments * bin_size;
```
```cpp
const float delta_x = min_turning_radius * sin(angle);
const float delta_y = min_turning_radius -
  (min_turning_radius * cos(angle));
const float delta_dist = hypotf(delta_x, delta_y);
projections.clear();
projections.emplace_back(
  delta_dist, 0.0, 0.0, TurnDirection::FORWARD);
projections.emplace_back(
  delta_x, delta_y, increments, TurnDirection::LEFT);
projections.emplace_back(
  delta_x, -delta_y, -increments, TurnDirection::RIGHT);
const float arc_angle = projections[i]._theta * bin_size;
const float turning_rad = delta_dist /
  (2.0f * sin(arc_angle / 2.0f));
travel_costs[i] = turning_rad * arc_angle;
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/src/node_hybrid.cpp` dan `smac_planner_hybrid_impl.hpp`.

---

## 22. Transformasi motion primitive terhadap heading node

### 22.1 Rumus lengkap dan penjelasan variabel

Primitive lokal diputar ke heading node menggunakan rotasi 2D:

\[
\Delta x_g=\Delta x\cos\theta-\Delta y\sin\theta
\]
\[
\Delta y_g=\Delta x\sin\theta+\Delta y\cos\theta
\]
Keterangan: \(\Delta x,\Delta y\) = primitive pada orientasi nol; \(\theta\) = heading node saat ini; \(\Delta x_g,\Delta y_g\) = perpindahan primitive setelah diputar ke orientasi node.

### 22.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      motion_model_for_search: DUBIN
      angle_quantization_bins: 48
      minimum_turning_radius: 1.597679121829
```

### 22.3 Potongan program yang merepresentasikan rumus

```cpp
for (unsigned int j = 0; j != num_angle_quantization; j++) {
  double cos_theta = cos(bin_size * j);
  double sin_theta = sin(bin_size * j);
  trig_values[j] = {cos_theta, sin_theta};
  delta_xs[i][j] =
    projections[i]._x * cos_theta - projections[i]._y * sin_theta;
  delta_ys[i][j] =
    projections[i]._x * sin_theta + projections[i]._y * cos_theta;
}
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/src/node_hybrid.cpp`.

---
## 23. Traversal cost SMAC Hybrid A*

### 23.1 Rumus lengkap dan penjelasan variabel

Source Navigation2 menormalisasi cost costmap:

\[
\bar C=\frac{C_{cell}}{252}
\]

Dengan `use_quadratic_cost_penalty=false`, biaya dasar primitive adalah:

\[
c_{raw}=s_p\left[(1-p_r)+p_c\bar C\right]
\]

Jika primitive belok tanpa perubahan arah belok:

\[
c=c_{raw}p_{ns}
\]

Jika arah belok berubah:

\[
c=c_{raw}(p_{ns}+p_{chg})
\]

Untuk primitive reverse pada model yang mengizinkannya:

\[
c\leftarrow c\,p_{rev}
\]
Keterangan: \(C_{cell}\) = nilai cost pada child node; \(\bar C\) = normalized cost; \(s_p\) = travel cost primitive; \(p_r\) = `retrospective_penalty`; \(p_c\) = `cost_penalty`; \(p_{ns}\) = `non_straight_penalty`; \(p_{chg}\) = `change_penalty`; \(p_{rev}\) = `reverse_penalty`.

Pada konfigurasi `DUBIN`, primitive reverse tidak dibentuk sehingga `reverse_penalty` tidak berpengaruh pada pencarian forward-only tersebut.

### 23.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      motion_model_for_search: DUBIN
      reverse_penalty: 3.0
      change_penalty: 0.0
      non_straight_penalty: 1.2
      cost_penalty: 2.2
      retrospective_penalty: 0.015
```

### 23.3 Potongan program yang merepresentasikan rumus

```cpp
const float normalized_cost = child->getCost() / 252.0f;
float travel_cost_raw =
  _ctx->motion_table.travel_costs[child->getMotionPrimitiveIndex()];
travel_cost_raw *=
  (_ctx->motion_table.travel_distance_reward +
  _ctx->motion_table.cost_penalty * normalized_cost);
```
```cpp
if (child_turn_dir == TurnDirection::FORWARD ||
    child_turn_dir == TurnDirection::REVERSE ||
    getMotionPrimitiveIndex() == std::numeric_limits<unsigned int>::max()) {
  travel_cost = travel_cost_raw;
} else if (getTurnDirection() == child_turn_dir) {
  travel_cost = travel_cost_raw *
    _ctx->motion_table.non_straight_penalty;
} else {
  travel_cost = travel_cost_raw *
    (_ctx->motion_table.non_straight_penalty +
    _ctx->motion_table.change_penalty);
}
if (child_turn_dir == TurnDirection::REV_RIGHT ||
    child_turn_dir == TurnDirection::REV_LEFT ||
    child_turn_dir == TurnDirection::REVERSE) {
  travel_cost *= _ctx->motion_table.reverse_penalty;
}
return travel_cost;
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/src/node_hybrid.cpp`.

---

## 24. Heuristic SMAC Hybrid A*

### 24.1 Rumus lengkap dan penjelasan variabel

Navigation2 menggabungkan obstacle heuristic dan distance/kinematic heuristic dengan:

\[
h(n)=\max\left(h_{obs}(n),h_{dist}(n)\right)
\]
Keterangan: \(h_{obs}\) = heuristic berbasis obstacle/costmap; \(h_{dist}\) = heuristic jarak yang mempertimbangkan model gerak; operator maksimum dipakai agar heuristic akhir tidak lebih kecil daripada salah satu estimasi biaya tersebut.

### 24.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      cost_penalty: 2.2
      cache_obstacle_heuristic: true
      lookup_table_size: 20.2
      downsample_costmap: false
      downsampling_factor: 1
```

### 24.3 Potongan program yang merepresentasikan rumus

```cpp
const float obstacle_heuristic =
  _ctx->obstacle_heuristic->getObstacleHeuristic(
    node_coords,
    _ctx->motion_table.cost_penalty,
    _ctx->motion_table.use_quadratic_cost_penalty,
    _ctx->motion_table.downsample_obstacle_heuristic);
float distance_heuristic = std::numeric_limits<float>::max();
for (unsigned int i = 0; i < goals_coords.size(); i++) {
  distance_heuristic = std::min(
    distance_heuristic,
    _ctx->distance_heuristic->getDistanceHeuristic(
      node_coords, goals_coords[i], obstacle_heuristic,
      _ctx->motion_table));
}
return std::max(obstacle_heuristic, distance_heuristic);
```
Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/src/node_hybrid.cpp`.

---

## 25. Analytic expansion pada SMAC Hybrid A*

### 25.1 Rumus lengkap dan penjelasan variabel

Frekuensi percobaan analytic expansion disesuaikan terhadap kedekatan node ke goal. Implementasi Navigation2 membentuk jumlah iterasi target:

\[
N_{analytic}=\max\left(
\left\lfloor\frac{d_{close}}{r_a}\right\rfloor,
\left\lceil r_a\right\rceil
\right)
\]

Keterangan: \(d_{close}\) = heuristic distance terdekat yang telah dicapai; \(r_a\) = `analytic_expansion_ratio`; \(N_{analytic}\) = jumlah iterasi sebelum percobaan analytic expansion berikutnya. Ketika kendaraan semakin dekat ke goal, analytic connection dicoba lebih sering tetapi tetap dibatasi oleh ratio minimum.

Panjang analytic expansion juga dibatasi oleh:

\[
L_{analytic}\le L_{max}
\]

Keterangan: \(L_{max}\) = `analytic_expansion_max_length` dalam meter.

### 25.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      analytic_expansion_ratio: 3.5
      analytic_expansion_max_length: 8.0
```
### 25.3 Potongan program yang merepresentasikan rumus

```cpp
closest_distance = std::min(
  closest_distance,
  static_cast<int>(current_node->getHeuristicCost(
    node_coords, goals_coords)));
int desired_iterations = std::max(
  static_cast<int>(
    closest_distance / _search_info.analytic_expansion_ratio),
  static_cast<int>(
    std::ceil(_search_info.analytic_expansion_ratio)));
analytic_iterations =
  std::min(analytic_iterations, desired_iterations);
if (analytic_iterations <= 0) {
  analytic_iterations = desired_iterations;
}
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/include/nav2_smac_planner/analytic_expansion_impl.hpp`.

---

## 26. Downsampling costmap dan skala parameter planner

### 26.1 Rumus lengkap dan penjelasan variabel

Jika downsampling aktif, resolusi pencarian efektif menjadi:

\[
r_{search}=r_{costmap}D
\]

Radius minimum dalam koordinat grid kemudian menjadi:

\[
R_g=\frac{R_{min}}{r_{search}}
\]
Keterangan: \(r_{costmap}\) = resolusi costmap asli; \(D\) = `downsampling_factor`; \(r_{search}\) = resolusi grid pencarian; \(R_{min}\) = minimum turning radius dalam meter; \(R_g\) = radius minimum dalam sel pencarian.

Pada konfigurasi aktual `downsample_costmap: false`, source memaksa `downsampling_factor` efektif menjadi 1 sehingga planner tetap menggunakan resolusi costmap 0,1 m.

### 26.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      downsample_costmap: false
      downsampling_factor: 1
      minimum_turning_radius: 1.597679121829

global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 0.1
```

### 26.3 Potongan program yang merepresentasikan rumus

```cpp
if (!_downsample_costmap) {
  _downsampling_factor = 1;
}
_search_info.minimum_turning_radius =
  _minimum_turning_radius_global_coords /
  (_costmap->getResolution() * _downsampling_factor);
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid_impl.hpp`.
---

## 27. Smoothing global path pada SMAC Planner

### 27.1 Rumus lengkap dan penjelasan variabel

Smoother Navigation2 memperbarui setiap titik internal path menggunakan kombinasi tarikan ke data awal dan kelengkungan lokal:

\[
y_i^{new}=y_i+w_d(x_i-y_i)+w_s(y_{i+1}+y_{i-1}-2y_i)
\]

Perubahan total setiap iterasi adalah:

\[
\Delta=\sum_i\left|y_i^{new}-y_i^{old}\right|
\]

Iterasi berhenti ketika:

\[
\Delta<\varepsilon
\]

atau jumlah iterasi mencapai batas maksimum.

Keterangan: \(x_i\) = koordinat titik path asli; \(y_i\) = koordinat titik hasil smoothing; \(w_d\) = `w_data`; \(w_s\) = `w_smooth`; \(\varepsilon\) = `tolerance`; indeks \(i-1,i,i+1\) = tiga titik bertetangga pada path.### 27.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      smooth_path: true
      smoother:
        max_iterations: 200
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-06
        do_refinement: false
```

### 27.3 Potongan program yang merepresentasikan rumus

```cpp
y_i = getFieldByDim(new_path.poses[i], j);
y_m1 = getFieldByDim(new_path.poses[i - 1], j);
y_ip1 = getFieldByDim(new_path.poses[i + 1], j);
y_i_org = y_i;
y_i += data_w_ * (x_i - y_i) +
  smooth_w_ * (y_ip1 + y_m1 - (2.0 * y_i));
setFieldByDim(new_path.poses[i], j, y_i);
change += abs(y_i - y_i_org);
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/src/smoother.cpp`.

---

## 28. Validasi collision pada node Hybrid A*

### 28.1 Rumus lengkap dan penjelasan variabel

Kelayakan node dapat dinyatakan sebagai fungsi indikator:

\[
V(x,y,\theta)=
\begin{cases}
1, & \text{footprint tidak berkolisi}\\
0, & \text{footprint berkolisi}
\end{cases}
\]

Node hanya dimasukkan sebagai neighbor jika:

\[
V(x,y,\theta)=1
\]

Keterangan: \(x,y,\theta\) = pose kandidat node; \(V\) = status validitas collision; pemeriksaan dilakukan menggunakan `GridCollisionChecker` terhadap costmap dan footprint kendaraan.

### 28.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      allow_unknown: false

global_costmap:
  global_costmap:
    ros__parameters:
      footprint: '[[0.52,0.30],[0.52,-0.30],[-0.52,-0.30],[-0.52,0.30]]'
      footprint_padding: 0.03
```
### 28.3 Potongan program yang merepresentasikan rumus

```cpp
_is_node_valid = !collision_checker->inCollision(
  this->pose.x,
  this->pose.y,
  this->pose.theta,
  traverse_unknown);
_cell_cost = collision_checker->getCost();
return _is_node_valid;
```

```cpp
if (neighbor->isNodeValid(traverse_unknown, collision_checker)) {
  neighbor->setMotionPrimitiveIndex(
    i, motion_projections[i]._turn_dir);
  neighbors.push_back(neighbor);
}
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/src/node_hybrid.cpp`.

---

## 29. Batas waktu, iterasi, dan toleransi pencarian

### 29.1 Rumus lengkap dan penjelasan variabel

Waktu proses planner dihitung sebagai:

\[
t_{plan}=t_{now}-t_{start}
\]

Pencarian dihentikan atau dialihkan ke solusi terdekat ketika:

\[
t_{plan}\ge t_{max}
\]
Selain batas waktu, pencarian dibatasi oleh jumlah ekspansi:

\[
N_{iter}\le N_{max}
\]

dan pada fase pendekatan goal:

\[
N_{approach}\le N_{approach,max}
\]

Keterangan: \(t_{plan}\) = durasi planning; \(t_{max}\) = `max_planning_time`; \(N_{iter}\) = jumlah iterasi pencarian; \(N_{max}\) = `max_iterations`; \(N_{approach}\) = iterasi ketika sudah berada dalam toleransi goal; \(N_{approach,max}\) = `max_on_approach_iterations`.

### 29.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      tolerance: 0.05
      max_iterations: 250000
      max_on_approach_iterations: 1000
      max_planning_time: 2.5
```

### 29.3 Potongan program yang merepresentasikan rumus

```cpp
const auto planning_duration =
  std::chrono::duration_cast<std::chrono::duration<double>>(
    steady_clock::now() - start_time);
if (static_cast<double>(planning_duration.count()) >=
    _max_planning_time) {
  return getClosestPathWithinTolerance(path);
}
``````cpp
iterations++;
if (_goal_manager.isGoal(current_node)) {
  return current_node->backtracePath(path);
} else if (_best_heuristic_node.first < getToleranceHeuristic()) {
  approach_iterations++;
  if (approach_iterations >= getOnApproachMaxIterations()) {
    return _graph.at(_best_heuristic_node.second).backtracePath(path);
  }
}
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/include/nav2_smac_planner/a_star_impl.hpp`.

---

## 30. Lookup table heuristic pada SMAC Hybrid A*

### 30.1 Rumus lengkap dan penjelasan variabel

Ukuran lookup table dalam koordinat grid dibentuk dari ukuran dunia:

\[
L_g=\frac{L_{lookup}}{rD}
\]

Nilai kemudian dibulatkan ke integer dan dipaksa menjadi bilangan ganjil:

\[
L_{g,odd}=\begin{cases}
L_g+1,&L_g\text{ genap}\\
L_g,&L_g\text{ ganjil}
\end{cases}
\]
Keterangan: \(L_{lookup}\) = `lookup_table_size` dalam meter; \(r\) = resolusi costmap; \(D\) = downsampling factor; \(L_g\) = dimensi lookup table pada grid. Pemaksaan ukuran ganjil membuat lookup table memiliki pusat yang simetris.

### 30.2 Potongan YAML yang berpengaruh

```yaml
planner_server:
  ros__parameters:
    GridBased:
      lookup_table_size: 20.2
      downsample_costmap: false
      downsampling_factor: 1

global_costmap:
  global_costmap:
    ros__parameters:
      resolution: 0.1
```

### 30.3 Potongan program yang merepresentasikan rumus

```cpp
_lookup_table_dim =
  static_cast<float>(_lookup_table_size) /
  static_cast<float>(
    _costmap->getResolution() * _downsampling_factor);
_lookup_table_dim =
  static_cast<float>(static_cast<int>(_lookup_table_dim));
if (static_cast<int>(_lookup_table_dim) % 2 == 0) {
  _lookup_table_dim += 1.0;
}
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/include/nav2_smac_planner/smac_planner_hybrid_impl.hpp`.

---

## 31. Parameter default Navigation2 yang tetap memengaruhi Hybrid A*

### 31.1 Rumus lengkap dan penjelasan variabel

Walaupun beberapa parameter tidak ditulis eksplisit pada YAML aktual, default source tetap aktif. Untuk `use_quadratic_cost_penalty=false`, traversal cost memakai bentuk linear yang telah ditulis pada Bagian 23. Jika diubah menjadi `true`, komponen cost menjadi:

\[
c_{raw}=s_p\left[(1-p_r)+p_c\bar C^2\right]
\]

`allow_primitive_interpolation=true` memungkinkan primitive antara sudut nol dan primitive maksimum. Untuk increment ke-\(i\):

\[
\theta_i=i\Delta\theta_{bin}
\]

\[
R_i=\frac{\Delta s}{2\sin(\theta_i/2)}
\]

\[
\Delta x_i=R_i\sin\theta_i,\qquad
\Delta y_i=R_i(1-\cos\theta_i)
\]

Keterangan: \(p_c\) = cost penalty; \(\bar C\) = normalized cost; \(i\) = indeks interpolasi primitive; \(R_i\) = radius primitive interpolasi.### 31.2 Potongan YAML yang berpengaruh

Parameter berikut didukung source Navigation2 dan saat ini tidak ditulis eksplisit pada YAML sistem, sehingga perilakunya mengikuti default source:

```yaml
planner_server:
  ros__parameters:
    GridBased:
      allow_primitive_interpolation: true
      use_quadratic_cost_penalty: false
      downsample_obstacle_heuristic: true
      analytic_expansion_max_cost: 200.0
      analytic_expansion_max_cost_override: false
      goal_heading_mode: DEFAULT
      coarse_search_resolution: 1
```

Untuk kebutuhan laporan, parameter tersebut boleh dituliskan sebagai **default runtime dari package Navigation2**, bukan sebagai nilai yang sudah dituning pada file YAML aktual.

### 31.3 Potongan program yang merepresentasikan rumus

```cpp
_search_info.allow_primitive_interpolation =
  node->declare_or_get_parameter(
    name + ".allow_primitive_interpolation", true);
_search_info.use_quadratic_cost_penalty =
  node->declare_or_get_parameter(
    name + ".use_quadratic_cost_penalty", false);
_search_info.downsample_obstacle_heuristic =
  node->declare_or_get_parameter(
    name + ".downsample_obstacle_heuristic", true);
```
```cpp
for (unsigned int i = 1;
     i < static_cast<unsigned int>(increments); i++) {
  const float angle_n = static_cast<float>(i) * bin_size;
  const float turning_rad_n =
    delta_dist / (2.0f * sin(angle_n / 2.0f));
  const float delta_x_n = turning_rad_n * sin(angle_n);
  const float delta_y_n =
    turning_rad_n - (turning_rad_n * cos(angle_n));
  projections.emplace_back(
    delta_x_n, delta_y_n,
    static_cast<float>(i), TurnDirection::LEFT);
  projections.emplace_back(
    delta_x_n, -delta_y_n,
    -static_cast<float>(i), TurnDirection::RIGHT);
}
```

Source referensi: `reference_repos/01_navigation2/nav2_smac_planner/src/node_hybrid.cpp` dan `smac_planner_hybrid_impl.hpp`.

---

# Catatan sinkronisasi untuk penulisan laporan

Untuk pembahasan EKF, gunakan persamaan dan urutan state pada Bagian 7–14 karena bagian tersebut sudah diselaraskan dengan source `robot_localization`, termasuk Jacobian pada prediction, faktor `delta_sec` pada process noise, Mahalanobis gate, dan Joseph-form covariance update.

Untuk pembahasan planning, gunakan Bagian 17–31 karena bagian tersebut mengikuti implementasi `nav2_costmap_2d` dan `nav2_smac_planner`, bukan hanya deskripsi Hybrid A* generik. Nilai aktual yang perlu konsisten dengan source saat ini antara lain `resolution: 0.1`, `motion_model_for_search: DUBIN`, `angle_quantization_bins: 48`, `minimum_turning_radius: 1.597679121829`, `cost_penalty: 2.2`, `downsample_costmap: false`, `analytic_expansion_ratio: 3.5`, dan `smooth_path: true`.
