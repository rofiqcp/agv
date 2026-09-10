#include "imu/imu_node.hpp"

#include <cstdint>
#include <array>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <glob.h>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>
#include <atomic>
#include <stdexcept>
#include <sstream>
#include <iomanip>

using Clock = std::chrono::steady_clock;

// Fungsi: Mengambil waktu monotonic untuk timeout/reconnect tanpa terpengaruh perubahan jam sistem.
static double nowSec()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    Clock::now().time_since_epoch()).count() / 1000.0;
}

// Fungsi: Menormalkan sudut ke rentang [-pi, pi] agar heading kontinu dan aman untuk EKF.
static double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Fungsi: Mengumpulkan kandidat device serial yang cocok dengan pola filesystem.
static std::vector<std::string> globPattern(const std::string & pattern)
{
  std::vector<std::string> out;
  glob_t g{};
  if (::glob(pattern.c_str(), 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; ++i)
      out.emplace_back(g.gl_pathv[i]);
  }
  globfree(&g);
  return out;
}

// Fungsi: Mengubah roll-pitch-yaw ROS menjadi quaternion untuk sensor_msgs/Imu.
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

// Fungsi: Menginisialisasi parameter kalibrasi, publisher, serial, dan timer pembacaan IMU.
ImuNode::ImuNode(const rclcpp::NodeOptions & options)
: Node("data_imu_node", options)
{
  port_ = this->declare_parameter<std::string>("port", "auto");
  auto_port_id_contains_ = this->declare_parameter<std::string>(
    "auto_port_id_contains", "Silicon_Labs_CP2102");
  auto_port_path_contains_ = this->declare_parameter<std::string>(
    "auto_port_path_contains", "");
  baudrate_ = this->declare_parameter<int>("baudrate", 9600);
  auto_baud_ = this->declare_parameter<bool>("auto_baud", true);
  baud_probe_sec_ = std::clamp(
    this->declare_parameter<double>("baud_probe_sec", 0.6), 0.2, 2.0);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
  debug_ = this->declare_parameter<bool>("debug", false);
  publish_raw_ = this->declare_parameter<bool>("publish_raw", false);
  poll_interval_ms_ = static_cast<int>(std::clamp<int64_t>(
    this->declare_parameter<int64_t>("poll_interval_ms", 10), 2, 1000));
  publish_rate_hz_ = static_cast<int>(std::clamp<int64_t>(
    this->declare_parameter<int64_t>("publish_rate_hz", 10), 1, 200));
  data_timeout_sec_ = std::max(0.5,
    this->declare_parameter<double>("data_timeout_sec", 6.0));
  configure_output_on_connect_ = this->declare_parameter<bool>("configure_output_on_connect", true);
  persist_output_config_ = this->declare_parameter<bool>("persist_output_config", false);
  configure_algorithm_on_connect_ = this->declare_parameter<bool>("configure_algorithm_on_connect", true);
  algorithm_mode_ = static_cast<int>(std::clamp<int64_t>(
    this->declare_parameter<int64_t>("algorithm_mode", 1), 0, 1));
  output_content_mask_ = static_cast<int>(std::clamp<int64_t>(
    this->declare_parameter<int64_t>("output_content_mask", 0x000E), 0, 0x07FF));
  // ACC/GYRO/ANGLE cukup untuk EKF; bandwidth 921600 memberi margin besar untuk rate tinggi.
  // Minimal mask untuk lokalisasi: ACC(0x02)+GYRO(0x04)+ANGLE(0x08) = 0x000E.
  output_content_mask_ |= 0x000E;
  output_rate_code_ = static_cast<int>(std::clamp<int64_t>(
    this->declare_parameter<int64_t>("output_rate_code", 0x06), 1, 0x0D));
  orientation_packet_timeout_sec_ = std::max(0.5,
    this->declare_parameter<double>("orientation_packet_timeout_sec", 8.0));
  orientation_publish_timeout_sec_ = std::clamp(
    this->declare_parameter<double>("orientation_publish_timeout_sec", 0.50), 0.10,
    orientation_packet_timeout_sec_);
  gyro_packet_timeout_sec_ = std::max(0.05,
    this->declare_parameter<double>("gyro_packet_timeout_sec", 0.35));
  accel_packet_timeout_sec_ = std::max(0.05,
    this->declare_parameter<double>("accel_packet_timeout_sec", 0.50));
  require_fresh_gyro_for_imu_publish_ =
    this->declare_parameter<bool>("require_fresh_gyro_for_imu_publish", true);
  component_sync_max_gap_sec_ = std::clamp(
    this->declare_parameter<double>("component_sync_max_gap_sec", 0.05), 0.005, 0.50);
  timestamp_max_future_sec_ = std::clamp(
    this->declare_parameter<double>("timestamp_max_future_sec", 0.02), 0.0, 0.25);
  timestamp_max_regression_sec_ = std::clamp(
    this->declare_parameter<double>("timestamp_max_regression_sec", 0.002), 0.0, 0.10);
  timing_status_rate_hz_ = std::clamp(
    this->declare_parameter<double>("timing_status_rate_hz", 2.0), 0.2, 20.0);
  sensor_config_retry_sec_ = std::max(2.0,
    this->declare_parameter<double>("sensor_config_retry_sec", 30.0));
  orientation_reopen_sec_ = std::max(orientation_packet_timeout_sec_ + 2.0,
    this->declare_parameter<double>("orientation_reopen_sec", 20.0));
  publish_orientation_ = this->declare_parameter<bool>("publish_orientation", true);
  invert_roll_ = this->declare_parameter<bool>("invert_roll", false);
  invert_pitch_ = this->declare_parameter<bool>("invert_pitch", false);
  invert_yaw_ = this->declare_parameter<bool>("invert_yaw", false);
  const double configured_yaw_sign = this->declare_parameter<double>("yaw_sign", 1.0);
  yaw_sign_ = configured_yaw_sign < 0.0 ? -1.0 : 1.0;
  if (invert_yaw_) {
    // Legacy compatibility. New configurations should set yaw_sign directly.
    yaw_sign_ *= -1.0;
  }
  roll_offset_rad_ = this->declare_parameter<double>("roll_offset_rad", 0.0);
  pitch_offset_rad_ = this->declare_parameter<double>("pitch_offset_rad", 0.0);
  yaw_offset_rad_ = this->declare_parameter<double>("yaw_offset_rad", 0.0);
  magnetic_declination_rad_ = this->declare_parameter<double>("magnetic_declination_radians", 0.0);
  vector_x_sign_ = this->declare_parameter<double>("vector_x_sign", -1.0) < 0.0 ? -1.0 : 1.0;
  vector_y_sign_ = this->declare_parameter<double>("vector_y_sign", -1.0) < 0.0 ? -1.0 : 1.0;
  vector_z_sign_ = this->declare_parameter<double>("vector_z_sign", 1.0) < 0.0 ? -1.0 : 1.0;
  publish_mag_tesla_ = this->declare_parameter<bool>("publish_mag_tesla", false);
  mag_scale_tesla_per_lsb_ = this->declare_parameter<double>("mag_scale_tesla_per_lsb", 0.0);
  use_magnetic_yaw_ = this->declare_parameter<bool>("use_magnetic_yaw", false);
  const double configured_mag_yaw_sign = this->declare_parameter<double>("mag_yaw_sign", -1.0);
  mag_yaw_sign_ = configured_mag_yaw_sign < 0.0 ? -1.0 : 1.0;
  mag_yaw_offset_rad_ = this->declare_parameter<double>("mag_yaw_offset_rad", 0.0);
  mag_yaw_filter_alpha_ = std::clamp(
    this->declare_parameter<double>("mag_yaw_filter_alpha", 0.20), 0.01, 1.0);
  mag_yaw_max_step_rad_ = std::clamp(
    this->declare_parameter<double>("mag_yaw_max_step_rad", 0.0523598776), 0.001, 0.35);
  mag_yaw_packet_timeout_sec_ = std::clamp(
    this->declare_parameter<double>("mag_yaw_packet_timeout_sec", 0.50), 0.10, 2.0);
  mag_yaw_min_norm_ut_ = std::max(0.0,
    this->declare_parameter<double>("mag_yaw_min_norm_ut", 100.0));
  mag_yaw_max_norm_ut_ = std::max(mag_yaw_min_norm_ut_ + 1.0,
    this->declare_parameter<double>("mag_yaw_max_norm_ut", 1000.0));
  accel_bias_ = this->declare_parameter<std::vector<double>>("accel_bias", {0.0, 0.0, 0.0});
  gyro_bias_ = this->declare_parameter<std::vector<double>>("gyro_bias", {0.0, 0.0, 0.0});
  // Stage-2 commissioning metadata. The driver does not alter these values;
  // NavigationCore/GUI use them as persistent evidence that bias/covariance were
  // measured on the real vehicle rather than left at defaults.
  (void)this->declare_parameter<bool>("stationary_calibration_valid", false);
  (void)this->declare_parameter<std::string>("stationary_calibration_saved_at", "");
  (void)this->declare_parameter<int64_t>("stationary_calibration_sample_count", 0);
  (void)this->declare_parameter<double>("stationary_calibration_duration_sec", 0.0);
  (void)this->declare_parameter<double>("stationary_calibration_gyro_z_std_rps", 0.0);
  (void)this->declare_parameter<double>("stationary_calibration_accel_norm_error_mps2", 0.0);
  (void)this->declare_parameter<int64_t>("stationary_calibration_min_samples", 80);
  (void)this->declare_parameter<double>("stationary_calibration_min_duration_sec", 8.0);
  (void)this->declare_parameter<double>("stationary_calibration_max_gyro_z_std_rps", 0.03);
  (void)this->declare_parameter<double>("stationary_calibration_max_accel_norm_error_mps2", 0.75);
  orientation_covariance_ = this->declare_parameter<std::vector<double>>(
    "orientation_covariance", {0.02, 0.02, 0.05});
  angular_velocity_covariance_ = this->declare_parameter<std::vector<double>>(
    "angular_velocity_covariance", {0.01, 0.01, 0.02});
  linear_acceleration_covariance_ = this->declare_parameter<std::vector<double>>(
    "linear_acceleration_covariance", {0.25, 0.25, 0.36});
  const auto valid3 = [](const std::vector<double> & v) {
      return v.size() == 3 && std::all_of(v.begin(), v.end(), [](double value) {
        return std::isfinite(value);
      });
    };
  if (!valid3(accel_bias_) || !valid3(gyro_bias_) || !valid3(orientation_covariance_) ||
      !valid3(angular_velocity_covariance_) || !valid3(linear_acceleration_covariance_)) {
    throw std::invalid_argument("IMU calibration parameters must contain exactly 3 finite values");
  }
  const auto valid_variance = [](const std::vector<double> & covariance) {
      return std::all_of(covariance.begin(), covariance.end(), [](double value) {
        return value > 0.0;
      });
    };
  if (!valid_variance(orientation_covariance_) ||
      !valid_variance(angular_velocity_covariance_) ||
      !valid_variance(linear_acceleration_covariance_)) {
    throw std::invalid_argument("IMU covariance diagonal must be finite and strictly positive");
  }
  if (!std::isfinite(mag_scale_tesla_per_lsb_) || mag_scale_tesla_per_lsb_ < 0.0) {
    throw std::invalid_argument("mag_scale_tesla_per_lsb must be finite and non-negative");
  }
  if (publish_mag_tesla_ && mag_scale_tesla_per_lsb_ <= 0.0) {
    throw std::invalid_argument("publish_mag_tesla=true requires a calibrated positive mag_scale_tesla_per_lsb");
  }
  RCLCPP_INFO(
    this->get_logger(),
    "IMU yaw conversion: source=%s angle=normalize(%+.0f*yaw_raw + %.6f) "
    "mag=normalize(%+.0f*atan2(My,Mx) + %.6f) declination=%.6f; gyro_z_ros=%+.0f*gyro_z_raw; vector_sign=[%+.0f,%+.0f,%+.0f]",
    use_magnetic_yaw_ ? "MAG_FILTERED" : "ANGLE", yaw_sign_, yaw_offset_rad_,
    mag_yaw_sign_, mag_yaw_offset_rad_, magnetic_declination_rad_, vector_z_sign_,
    vector_x_sign_, vector_y_sign_, vector_z_sign_);
  auto_detect_ = (port_ == "auto");

  // USB-UART pada sensor 10 Hz tidak memerlukan timeout 2 ms. Margin 20 ms
  // menghindari false SerialException akibat scheduler/USB latency tanpa menambah
  // latency publish karena available() tetap dipoll setiap beberapa ms.
  ser_timeout_ = serial::Timeout::simpleTimeout(20);

  auto qos = rclcpp::SensorDataQoS().keep_last(20);

  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
  pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);
  pub_mag_raw_lsb_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("imu/mag_raw_lsb", qos);
  pub_raw_sensor_vectors_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("imu/raw_sensor_vectors", qos);
  pub_profile_status_ = this->create_publisher<std_msgs::msg::String>(
    "/imu/profile_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  pub_timing_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/imu/timing", rclcpp::SensorDataQoS().keep_last(20));
  pub_timing_status_ = this->create_publisher<std_msgs::msg::String>(
    "/imu/timing_status", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  srv_configure_optimal_profile_ = this->create_service<std_srvs::srv::Trigger>(
    "/imu/configure_optimal_profile",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::string detail;
      response->success = configureOptimalProfile(detail);
      response->message = detail;
      publishProfileStatus(response->success ? "PASS" : "FAIL", detail);
    });
  pub_connected_ = this->create_publisher<std_msgs::msg::Bool>(
    "/imu/connected", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  if (publish_raw_) {
    pub_raw_ = this->create_publisher<std_msgs::msg::ByteMultiArray>("imu/raw", 10);
  }

  // Runtime hanya hardware nyata. Tidak ada jalur mock/synthetic IMU.
  ser_ = std::make_unique<serial::Serial>();
  openSerial(true);
  last_reconnect_try_ = nowSec();
  publishProfileStatus(ser_ && ser_->isOpen() ? "READY" : "WAIT", "runtime profile initialized");

  // Polling serial tetap ringan; publish rate dibatasi terpisah pada publishImu().
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms_), std::bind(&ImuNode::pollSerial, this));
}

// Fungsi: Menutup serial secara deterministik saat node dihancurkan.
ImuNode::~ImuNode()
{
  closeSerial();
}

// Fungsi: Mendeteksi port IMU dengan prioritas symlink persisten lalu fallback USB/ACM.
std::vector<std::string> ImuNode::detectPort()
{
  // 1) Stable USB identity is primary. CP2102 exposes a unique by-id on this
  // vehicle, so moving the cable or kernel ttyUSB renumbering cannot swap it.
  if (!auto_port_id_contains_.empty()) {
    std::vector<std::string> matches;
    for (const auto & p : globPattern("/dev/serial/by-id/*")) {
      if (p.find(auto_port_id_contains_) != std::string::npos) matches.push_back(p);
    }
    if (matches.size() == 1U) return matches;
  }

  // 2) Keep physical topology only as a deterministic fallback for systems
  // where by-id is absent/duplicated. A selected port must still pass the WIT
  // 0x55 checksum probe before it is accepted as the IMU.
  if (!auto_port_path_contains_.empty()) {
    std::vector<std::string> matches;
    for (const auto & p : globPattern("/dev/serial/by-path/*")) {
      if (p.find(auto_port_path_contains_) != std::string::npos) matches.push_back(p);
    }
    if (matches.size() == 1U) return matches;
  }

  // Configured selectors that are unresolved/ambiguous fail closed. Hot-plug
  // retry will re-run detection; never guess another ttyUSB device.
  if (!auto_port_id_contains_.empty() || !auto_port_path_contains_.empty()) return {};

  // Generic fallback is opt-in only by clearing both selectors.
  std::vector<std::string> candidates = globPattern("/dev/serial/by-id/*");
  if (candidates.empty()) {
    auto usb = globPattern("/dev/ttyUSB*");
    auto acm = globPattern("/dev/ttyACM*");
    candidates = usb;
    candidates.insert(candidates.end(), acm.begin(), acm.end());
  }
  return candidates;
}


// Fungsi: Daftar baud WIT/Yahboom umum, dengan nilai YAML selalu dicoba pertama.
std::vector<int> ImuNode::candidateBaudrates() const
{
  std::vector<int> out;
  const std::array<int, 6> common{{baudrate_, 921600, 460800, 230400, 115200, 9600}};
  for (const int baud : common) {
    if (baud <= 0) continue;
    if (std::find(out.begin(), out.end(), baud) == out.end()) out.push_back(baud);
  }
  return out;
}

// Fungsi: Probe pasif stream WIT Standard Protocol. Port baru dianggap IMU
// valid jika minimal satu frame 11-byte 0x55/0x51..0x59 dengan checksum benar
// terlihat. Ini mencegah status CONNECTED palsu pada baud yang salah.
bool ImuNode::looksLikeWitStream(double probe_sec)
{
  if (!ser_ || !ser_->isOpen()) return false;
  const auto deadline = Clock::now() + std::chrono::duration<double>(probe_sec);
  std::vector<uint8_t> sample;
  sample.reserve(4096);

  while (Clock::now() < deadline && sample.size() < 4096) {
    try {
      const size_t n = ser_->available();
      std::string data;
      if (n > 0) data = ser_->read(std::min<size_t>(n, 512));
      else data = ser_->read(64);
      if (!data.empty()) sample.insert(sample.end(), data.begin(), data.end());
    } catch (...) {
      break;
    }

    while (sample.size() >= 11) {
      if (sample[0] != 0x55 || sample[1] < 0x51 || sample[1] > 0x59) {
        sample.erase(sample.begin());
        continue;
      }
      uint16_t sum = 0;
      for (size_t i = 0; i < 10; ++i) sum += sample[i];
      if ((sum & 0xFFU) == sample[10]) return true;
      sample.erase(sample.begin());
    }
  }
  return false;
}

// Fungsi: Menulis satu register konfigurasi protokol Yahboom/WitMotion (FF AA ADDR L H).
bool ImuNode::writeSensorRegister(uint8_t address, uint16_t value)
{
  if (!ser_ || !ser_->isOpen()) return false;
  const std::array<uint8_t, 5> cmd{{
    0xFF, 0xAA, address,
    static_cast<uint8_t>(value & 0xFFU),
    static_cast<uint8_t>((value >> 8U) & 0xFFU)}};
  try {
    const std::string bytes(reinterpret_cast<const char *>(cmd.data()), cmd.size());
    return ser_->write(bytes) == cmd.size();
  } catch (const std::exception &e) {
    logRateLimited("IMU config write gagal: " + std::string(e.what()), "warn");
    return false;
  }
}

// Fungsi: Menerbitkan profil serial/algoritma sebagai state machine untuk ROS Web.
void ImuNode::publishProfileStatus(const std::string & state, const std::string & detail)
{
  if (!pub_profile_status_) return;
  std_msgs::msg::String msg;
  msg.data = "state=" + state + ";baud=" + std::to_string(baudrate_) +
    ";rrate=" + std::to_string(output_rate_code_) +
    ";rsw=" + std::to_string(output_content_mask_) +
    ";axis6=" + std::to_string(algorithm_mode_) +
    ";detail=" + detail;
  pub_profile_status_->publish(msg);
}

// Fungsi: Terapkan profil AGV Yahboom resmi dan persisten. BAUD ditulis terakhir,
// host ikut pindah ke 921600, kemudian SAVE dan checksum stream diverifikasi.
bool ImuNode::configureOptimalProfile(std::string & detail)
{
  if (!ser_ || !ser_->isOpen()) {
    detail = "serial Yahboom belum terbuka";
    return false;
  }
  publishProfileStatus("APPLYING", "unlock dan tulis profil optimal");
  const auto pause = []() { std::this_thread::sleep_for(std::chrono::milliseconds(90)); };
  if (!writeSensorRegister(0x69, 0xB588)) { detail = "unlock gagal"; return false; }
  pause();
  if (!writeSensorRegister(0x02, 0x001E)) { detail = "RSW gagal"; return false; }
  pause();
  if (!writeSensorRegister(0x03, 0x0008)) { detail = "RRATE 50Hz gagal"; return false; }
  pause();
  if (!writeSensorRegister(0x24, 0x0001)) { detail = "AXIS6 gagal"; return false; }
  pause();
  if (!writeSensorRegister(0x25, 0x001E)) { detail = "FILTK gagal"; return false; }
  pause();
  if (!writeSensorRegister(0x23, 0x0000)) { detail = "ORIENT horizontal gagal"; return false; }
  pause();
  if (!writeSensorRegister(0x04, 0x0009)) { detail = "BAUD 921600 gagal"; return false; }
  pause();
  try {
    ser_->setBaudrate(921600U);
    baudrate_ = 921600;
    try { ser_->flushInput(); } catch (...) {}
  } catch (const std::exception & e) {
    detail = std::string("host switch 921600 gagal: ") + e.what();
    closeSerial();
    last_reconnect_try_ = 0.0;
    return false;
  }
  if (!writeSensorRegister(0x69, 0xB588)) { detail = "re-unlock @921600 gagal"; return false; }
  pause();
  if (!writeSensorRegister(0x00, 0x0000)) { detail = "SAVE profil gagal"; return false; }
  std::this_thread::sleep_for(std::chrono::milliseconds(180));
  try { ser_->flushInput(); } catch (...) {}
  const bool verified = looksLikeWitStream(std::max(0.8, baud_probe_sec_));
  if (!verified) {
    detail = "profil ditulis tetapi stream 921600 belum checksum-valid; reconnect fail-closed";
    closeSerial();
    last_reconnect_try_ = 0.0;
    return false;
  }
  output_content_mask_ = 0x001E;
  output_rate_code_ = 0x08;
  algorithm_mode_ = 1;
  buf_.clear();
  last_data_time_ = nowSec();
  last_valid_packet_time_ = last_data_time_;
  detail = "921600 baud / 50Hz / RSW 0x001E / AXIS6 1 / FILTK 30 tersimpan dan stream terverifikasi";
  RCLCPP_INFO(this->get_logger(), "IMU optimal profile PASS: %s", detail.c_str());
  return true;
}

// Fungsi: Memastikan sensor mengeluarkan ACC+GYRO+ANGLE pada rate yang dibutuhkan.
// Konfigurasi diterapkan ulang saat reconnect. SAVE opsional dan default OFF agar
// reconnect/hot-plug tidak menulis flash sensor berulang kali.
bool ImuNode::configureSensorOutput(bool persistent)
{
  if (!configure_output_on_connect_ || !ser_ || !ser_->isOpen()) return true;
  last_sensor_config_try_ = nowSec();
  // KEY unlock = 0xB588, RSW = output mask, RRATE = output rate.
  // Manual resmi meminta jeda sekitar 50-100 ms antar write register; 80 ms
  // dipakai supaya command konfigurasi tidak saling menimpa di sisi sensor.
  if (!writeSensorRegister(0x69, 0xB588)) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  if (!writeSensorRegister(0x02, static_cast<uint16_t>(output_content_mask_))) return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  if (!writeSensorRegister(0x03, static_cast<uint16_t>(output_rate_code_))) return false;
  if (configure_algorithm_on_connect_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    if (!writeSensorRegister(0x24, static_cast<uint16_t>(algorithm_mode_))) return false;
  }
  if (persistent) {
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    if (!writeSensorRegister(0x00, 0x0000)) return false;  // SAVE
  }
  RCLCPP_INFO(
    this->get_logger(),
    "IMU stream config applied: RSW=0x%04X RRATE=0x%02X AXIS6=%d persistent=%s",
    output_content_mask_, output_rate_code_, algorithm_mode_, persistent ? "yes" : "no");
  return true;
}

// Fungsi: Membuka port serial IMU dan menyiapkan reconnect bila perangkat belum tersedia.
// Pada mode auto, port baru diterima setelah frame WIT checksum-valid ditemukan;
// Prioritaskan baud YAML, lalu probe 921600/460800/230400/115200/9600 untuk recovery.
void ImuNode::openSerial(bool initial)
{
  std::vector<std::string> ports;
  if (auto_detect_) {
    ports = detectPort();
  } else if (!port_.empty()) {
    ports = {port_};
  }

  if (ports.empty()) {
    if (initial) {
      RCLCPP_INFO(this->get_logger(),
        "IMU port belum tersedia; hot-plug retry aktif (filter='%s')",
        auto_port_id_contains_.c_str());
    }
    if (pub_connected_) { std_msgs::msg::Bool b; b.data = false; pub_connected_->publish(b); }
    last_reconnect_try_ = nowSec();
    return;
  }

  std::vector<int> baudrates = auto_baud_ ? candidateBaudrates() : std::vector<int>{baudrate_};
  std::string last_err;

  for (const auto & port : ports) {
    for (const int baud : baudrates) {
      try {
        if (ser_ && ser_->isOpen()) {
          try { ser_->close(); } catch (...) {}
        }
        if (!ser_) ser_ = std::make_unique<serial::Serial>();

        ser_->setPort(port);
        ser_->setBaudrate(static_cast<uint32_t>(baud));
        ser_->setTimeout(ser_timeout_);
        ser_->open();
        try { ser_->flushInput(); } catch (...) {}

        bool stream_ok = true;
        if (auto_baud_ || auto_detect_) {
          stream_ok = looksLikeWitStream(baud_probe_sec_);
        }

        // Self-heal a WIT/Yahboom sensor whose periodic output was disabled.
        // The old logic only called configureSensorOutput() after a valid 0x55
        // frame had already been seen, so a silent-but-responsive IMU could
        // never recover. Limit the active write to the configured primary baud
        // (normally 9600) to avoid sending register writes at every probe rate.
        if (!stream_ok && configure_output_on_connect_ && baud == baudrate_) {
          try { ser_->flushInput(); } catch (...) {}
          if (configureSensorOutput(false)) {
            stream_ok = looksLikeWitStream(std::max(0.8, baud_probe_sec_));
            if (stream_ok) {
              RCLCPP_INFO(this->get_logger(),
                "IMU stream recovered on %s @ %d after output reconfigure",
                port.c_str(), baud);
            }
          }
        }

        if (!stream_ok) {
          last_err = port + "@" + std::to_string(baud) + ": no valid WIT 0x55 frame";
          try { ser_->close(); } catch (...) {}
          continue;
        }

        baudrate_ = baud;
        buf_.clear();
        const double opened_at = nowSec();
        last_data_time_ = opened_at;
        last_valid_packet_time_ = opened_at;
        last_orientation_packet_time_ = opened_at;
        last_accel_packet_time_ = 0.0;
        last_gyro_packet_time_ = 0.0;
        last_mag_packet_time_ = 0.0;
        last_accel_measurement_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        last_gyro_measurement_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        last_orientation_measurement_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        last_mag_measurement_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        last_packet_stamp_ns_ = 0;
        has_acc_ = false;
        has_gyro_ = false;
        has_angle_ = false;
        has_mag_ = false;
        stream_announced_ = false;
        error_suppress_until_ = 0.0;
        active_port_ = port;
        bytes_received_ = 0;
        packets_parsed_ = 0;
        packets_acc_ = packets_gyro_ = packets_angle_ = packets_quat_ = packets_mag_ = 0;
        RCLCPP_INFO(this->get_logger(),
          "IMU stream detected on %s @ %d (WIT frame checksum valid)",
          port.c_str(), baudrate_);
        if (pub_connected_) { std_msgs::msg::Bool b; b.data = true; pub_connected_->publish(b); }
        (void)configureSensorOutput(persist_output_config_);
        last_reconnect_try_ = nowSec();
        return;
      } catch (const std::exception & e) {
        last_err = port + "@" + std::to_string(baud) + ": " + e.what();
        try { if (ser_) ser_->close(); } catch (...) {}
      }
    }
  }

  if (pub_connected_) { std_msgs::msg::Bool b; b.data = false; pub_connected_->publish(b); }
  active_port_.clear();
  logRateLimited(
    std::string("IMU port ditemukan tetapi stream WIT belum valid. Last: ") +
      (last_err.empty() ? "unknown" : last_err) + "; akan scan ulang", "info");
  last_reconnect_try_ = nowSec();
}

// Fungsi: Menutup port serial IMU tanpa melempar exception saat shutdown/reconnect.
void ImuNode::closeSerial()
{
  if (ser_) {
    try { if (ser_->isOpen()) ser_->close(); } catch (...) {}
    ser_.reset();
  }
  active_port_.clear();
  // Never carry samples across a USB reconnect. Otherwise a newly-opened link
  // could combine fresh angle packets with gyro/accel values from the old fd.
  has_acc_ = false;
  has_gyro_ = false;
  has_angle_ = false;
  has_mag_ = false;
  mag_yaw_filter_initialized_ = false;
  filtered_mag_yaw_rad_ = 0.0;
  last_accel_packet_time_ = 0.0;
  last_gyro_packet_time_ = 0.0;
  last_mag_packet_time_ = 0.0;
  last_accel_measurement_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_gyro_measurement_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_orientation_measurement_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_mag_measurement_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_packet_stamp_ns_ = 0;
  packet_clock_initialized_ = false;
  if (pub_connected_) { std_msgs::msg::Bool b; b.data = false; pub_connected_->publish(b); }
}

// Fungsi: Membatasi frekuensi log berulang agar kegagalan serial tidak membanjiri terminal.
bool ImuNode::logRateLimited(const std::string & msg, const std::string & level)
{
  double sec = nowSec();
  if (sec < error_suppress_until_) return false;
  error_suppress_until_ = sec + error_suppress_window_;

  if (level == "error") RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str());
  else if (level == "warn") RCLCPP_WARN(this->get_logger(), "%s", msg.c_str());
  else RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
  return true;
}

// Build a measurement timestamp from steady_clock rather than publish-time now().
// WIT packets have no device timestamp, so the host reception/parse epoch is the
// strongest truthful timestamp available. The steady->ROS offset is anchored on
// the first packet and monotonicity is enforced fail-closed.
bool ImuNode::packetStampNow(rclcpp::Time & stamp)
{
  const auto steady_now = Clock::now();
  const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    steady_now.time_since_epoch()).count();
  const auto ros_now = this->now();
  const auto ros_ns = ros_now.nanoseconds();
  if (!packet_clock_initialized_) {
    packet_clock_offset_ns_ = ros_ns - steady_ns;
    packet_clock_initialized_ = true;
  }

  std::int64_t candidate_ns = packet_clock_offset_ns_ + steady_ns;
  const std::int64_t max_future_ns = static_cast<std::int64_t>(timestamp_max_future_sec_ * 1.0e9);
  if (candidate_ns > ros_ns + max_future_ns) {
    ++timestamp_future_rejects_;
    return false;
  }
  // Never future-date a hardware sample even inside the tolerance window.
  candidate_ns = std::min(candidate_ns, ros_ns);
  if (last_packet_stamp_ns_ > 0) {
    const std::int64_t max_regression_ns =
      static_cast<std::int64_t>(timestamp_max_regression_sec_ * 1.0e9);
    if (candidate_ns + max_regression_ns < last_packet_stamp_ns_) {
      ++timestamp_regression_rejects_;
      return false;
    }
    // Equal/sub-tolerance arrival stamps are made strictly monotonic by 1 ns.
    candidate_ns = std::max(candidate_ns, last_packet_stamp_ns_ + 1);
  }
  last_packet_stamp_ns_ = candidate_ns;
  stamp = rclcpp::Time(candidate_ns, get_clock()->get_clock_type());
  return true;
}

void ImuNode::publishTimingDiagnostics(
  const rclcpp::Time & measurement_stamp, const rclcpp::Time & publish_stamp)
{
  if (!pub_timing_) return;
  const auto age_ms = [&measurement_stamp](const rclcpp::Time & t) -> double {
      if (t.nanoseconds() <= 0) return -1.0;
      return (measurement_stamp - t).seconds() * 1000.0;
    };
  const double gyro_std_ms = gyro_period_count_ > 1 ?
    std::sqrt(gyro_period_m2_sec2_ / static_cast<double>(gyro_period_count_ - 1)) * 1000.0 : 0.0;
  std_msgs::msg::Float64MultiArray timing;
  // Contract (append-only): seq, sensor stamp, publish stamp, publish age ms,
  // accel/gyro/orientation/mag relative age ms, gyro period mean/std ms,
  // future rejects, regression rejects, then histogram bins <=5,10,15,25,40,80,>80 ms.
  timing.data = {
    static_cast<double>(imu_publish_sequence_), measurement_stamp.seconds(), publish_stamp.seconds(),
    (publish_stamp - measurement_stamp).seconds() * 1000.0,
    age_ms(last_accel_measurement_stamp_), age_ms(last_gyro_measurement_stamp_),
    age_ms(last_orientation_measurement_stamp_), age_ms(last_mag_measurement_stamp_),
    gyro_period_mean_sec_ * 1000.0, gyro_std_ms,
    static_cast<double>(timestamp_future_rejects_),
    static_cast<double>(timestamp_regression_rejects_)};
  for (const auto count : gyro_period_hist_) timing.data.push_back(static_cast<double>(count));
  pub_timing_->publish(timing);

  const double min_status_period = 1.0 / timing_status_rate_hz_;
  if (!pub_timing_status_ ||
      (last_timing_status_publish_.nanoseconds() > 0 &&
       (publish_stamp - last_timing_status_publish_).seconds() < min_status_period)) return;
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "{\"seq\":" << imu_publish_sequence_
      << ",\"measurement_age_ms\":" << (publish_stamp - measurement_stamp).seconds() * 1000.0
      << ",\"gyro_period_mean_ms\":" << gyro_period_mean_sec_ * 1000.0
      << ",\"gyro_period_std_ms\":" << gyro_std_ms
      << ",\"future_rejects\":" << timestamp_future_rejects_
      << ",\"regression_rejects\":" << timestamp_regression_rejects_
      << ",\"hist_ms\":[";
  for (size_t i = 0; i < gyro_period_hist_.size(); ++i) {
    if (i) out << ',';
    out << gyro_period_hist_[i];
  }
  out << "]}";
  std_msgs::msg::String status;
  status.data = out.str();
  pub_timing_status_->publish(status);
  last_timing_status_publish_ = publish_stamp;
}

// Fungsi: Menyusun Imu ROS dari state terakhir dengan bias, orientasi, dan covariance terkalibrasi.
bool ImuNode::publishImu()
{
  if (!(has_acc_ || has_gyro_ || has_angle_)) return false;

  const double stamp_sec = nowSec();
  const auto publish_stamp = this->now();
  const bool gyro_fresh = has_gyro_ && last_gyro_packet_time_ > 0.0 &&
    last_gyro_measurement_stamp_.nanoseconds() > 0 &&
    stamp_sec - last_gyro_packet_time_ >= 0.0 &&
    stamp_sec - last_gyro_packet_time_ <= gyro_packet_timeout_sec_;
  const bool accel_fresh_by_age = has_acc_ && last_accel_packet_time_ > 0.0 &&
    last_accel_measurement_stamp_.nanoseconds() > 0 &&
    stamp_sec - last_accel_packet_time_ >= 0.0 &&
    stamp_sec - last_accel_packet_time_ <= accel_packet_timeout_sec_;
  const bool orientation_fresh_by_age = has_angle_ && last_orientation_packet_time_ > 0.0 &&
    last_orientation_measurement_stamp_.nanoseconds() > 0 &&
    stamp_sec - last_orientation_packet_time_ >= 0.0 &&
    stamp_sec - last_orientation_packet_time_ <= orientation_publish_timeout_sec_;
  const auto measurement_stamp = last_gyro_measurement_stamp_;
  const bool accel_fresh = accel_fresh_by_age &&
    std::abs((measurement_stamp - last_accel_measurement_stamp_).seconds()) <= component_sync_max_gap_sec_;
  const bool orientation_fresh = orientation_fresh_by_age &&
    std::abs((measurement_stamp - last_orientation_measurement_stamp_).seconds()) <= component_sync_max_gap_sec_;
  const bool mag_fresh = has_mag_ && last_mag_packet_time_ > 0.0 &&
    stamp_sec - last_mag_packet_time_ >= 0.0 &&
    stamp_sec - last_mag_packet_time_ <= mag_yaw_packet_timeout_sec_;
  const double mag_norm_ut = std::sqrt(mx_ * mx_ + my_ * my_ + mz_ * mz_) *
    mag_scale_tesla_per_lsb_ * 1.0e6;
  const bool mag_norm_ok = std::isfinite(mag_norm_ut) &&
    mag_norm_ut >= mag_yaw_min_norm_ut_ && mag_norm_ut <= mag_yaw_max_norm_ut_;
  const bool magnetic_yaw_valid = mag_fresh && mag_norm_ok &&
    std::hypot(mx_, my_) > 1.0;

  // /imu/data uses the gyro packet as its measurement epoch. Both local and global
  // robot_localization instances fuse gyro-Z from this message; orientation remains
  // available to the startup/heading supervisory path when its packet is time-aligned.
  // Therefore a stale gyro must fail closed instead of receiving a fresh host stamp.
  if (require_fresh_gyro_for_imu_publish_ && !gyro_fresh) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "IMU gyro stale (timeout %.3fs): /imu/data ditahan agar EKF tidak menerima gyro lama",
      gyro_packet_timeout_sec_);
    return false;
  }

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = measurement_stamp;
  msg.header.frame_id = frame_id_;

  const bool yaw_source_valid = !use_magnetic_yaw_ || magnetic_yaw_valid;
  if (orientation_fresh && publish_orientation_ && yaw_source_valid) {
    const double roll_rad = (invert_roll_ ? -1.0 : 1.0) * roll_ * M_PI / 180.0 + roll_offset_rad_;
    const double pitch_rad = (invert_pitch_ ? -1.0 : 1.0) * pitch_ * M_PI / 180.0 + pitch_offset_rad_;
    double yaw_rad = normalizeAngle(
      yaw_sign_ * yaw_ * M_PI / 180.0 + yaw_offset_rad_ - magnetic_declination_rad_);
    if (use_magnetic_yaw_) {
      const double raw_mag_yaw = normalizeAngle(
        mag_yaw_sign_ * std::atan2(my_, mx_) + mag_yaw_offset_rad_ - magnetic_declination_rad_);
      if (!mag_yaw_filter_initialized_) {
        filtered_mag_yaw_rad_ = raw_mag_yaw;
        mag_yaw_filter_initialized_ = true;
      } else {
        const double innovation = normalizeAngle(raw_mag_yaw - filtered_mag_yaw_rad_);
        const double step = std::clamp(
          mag_yaw_filter_alpha_ * innovation, -mag_yaw_max_step_rad_, mag_yaw_max_step_rad_);
        filtered_mag_yaw_rad_ = normalizeAngle(filtered_mag_yaw_rad_ + step);
      }
      yaw_rad = filtered_mag_yaw_rad_;
    }
    double qx, qy, qz, qw;
    quatFromEuler(roll_rad, pitch_rad, yaw_rad, qx, qy, qz, qw);
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.orientation_covariance[0] = orientation_covariance_[0];
    msg.orientation_covariance[4] = orientation_covariance_[1];
    msg.orientation_covariance[8] = orientation_covariance_[2];
  } else {
    msg.orientation_covariance[0] = -1.0;
    if (use_magnetic_yaw_ && !magnetic_yaw_valid) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "MAG yaw invalid/stale: fresh=%s norm=%.1fuT valid_range=%.1f..%.1fuT; orientation fail-closed",
        mag_fresh ? "yes" : "no", mag_norm_ut, mag_yaw_min_norm_ut_, mag_yaw_max_norm_ut_);
    }
  }

  // Tanda angular velocity harus konsisten dengan orientasi yang dipublikasikan.
  // Bila paket gyro/accel belum pernah diterima, tandai field tersebut sebagai
  // tidak tersedia sesuai kontrak sensor_msgs/Imu, bukan memublikasikan nol palsu.
  if (gyro_fresh) {
    msg.angular_velocity.x = vector_x_sign_ * gx_ * M_PI / 180.0 - gyro_bias_[0];
    msg.angular_velocity.y = vector_y_sign_ * gy_ * M_PI / 180.0 - gyro_bias_[1];
    msg.angular_velocity.z = vector_z_sign_ * gz_ * M_PI / 180.0 - gyro_bias_[2];
    msg.angular_velocity_covariance[0] = angular_velocity_covariance_[0];
    msg.angular_velocity_covariance[4] = angular_velocity_covariance_[1];
    msg.angular_velocity_covariance[8] = angular_velocity_covariance_[2];
  } else {
    msg.angular_velocity_covariance[0] = -1.0;
  }

  if (accel_fresh) {
    // Koreksi mounting planar 180 derajat harus konsisten untuk seluruh vektor.
    // Jika roll/pitch dibalik, sumbu body X/Y sensor juga berlawanan terhadap
    // base_footprint. Z tidak berubah untuk rotasi murni 180 derajat terhadap Z.
    msg.linear_acceleration.x = vector_x_sign_ * ax_ - accel_bias_[0];
    msg.linear_acceleration.y = vector_y_sign_ * ay_ - accel_bias_[1];
    msg.linear_acceleration.z = vector_z_sign_ * az_ - accel_bias_[2];
    msg.linear_acceleration_covariance[0] = linear_acceleration_covariance_[0];
    msg.linear_acceleration_covariance[4] = linear_acceleration_covariance_[1];
    msg.linear_acceleration_covariance[8] = linear_acceleration_covariance_[2];
  } else {
    msg.linear_acceleration_covariance[0] = -1.0;
  }

  pub_imu_->publish(msg);
  ++imu_publish_sequence_;
  publishTimingDiagnostics(measurement_stamp, publish_stamp);
  publishRawSensorVectors();
  return true;
}

// Fungsi: Diagnostik sensor-frame sebelum mounting transform. Nilai: accel SI,
// gyro rad/s, lalu magnetometer raw LSB. Wizard memakai ini untuk mounting sanity-check.
void ImuNode::publishRawSensorVectors()
{
  if (!pub_raw_sensor_vectors_) return;
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {ax_, ay_, az_, gx_ * M_PI / 180.0, gy_ * M_PI / 180.0,
              gz_ * M_PI / 180.0, mx_, my_, mz_};
  pub_raw_sensor_vectors_->publish(msg);
}

// Fungsi: Menerbitkan raw magnetometer Yahboom dalam LSB resmi protokol, tetapi
// sudah diputar ke frame body REP-103 yang sama dengan accel/gyro. Ini adalah
// jalur kalibrasi yang benar karena dokumen Yahboom tidak menetapkan LSB->Tesla.
void ImuNode::publishMagRawLsb()
{
  if (!pub_mag_raw_lsb_) return;
  std_msgs::msg::Float64MultiArray msg;
  msg.data = {vector_x_sign_ * mx_, vector_y_sign_ * my_, vector_z_sign_ * mz_};
  pub_mag_raw_lsb_->publish(msg);
}

// Fungsi: Menerbitkan sensor_msgs/MagneticField hanya jika skala Tesla/LSB sudah
// benar-benar dikalibrasi. Jangan pernah mengarang unit fisik dari raw LSB.
void ImuNode::publishMag()
{
  publishMagRawLsb();
  if (!publish_mag_tesla_) return;
  sensor_msgs::msg::MagneticField msg;
  msg.header.stamp = last_mag_measurement_stamp_.nanoseconds() > 0 ? last_mag_measurement_stamp_ : this->now();
  msg.header.frame_id = frame_id_;
  msg.magnetic_field.x = vector_x_sign_ * mx_ * mag_scale_tesla_per_lsb_;
  msg.magnetic_field.y = vector_y_sign_ * my_ * mag_scale_tesla_per_lsb_;
  msg.magnetic_field.z = vector_z_sign_ * mz_ * mag_scale_tesla_per_lsb_;
  pub_mag_->publish(msg);
}

// Fungsi: Menerbitkan paket mentah IMU hanya ketika mode diagnostik raw diaktifkan.
void ImuNode::publishRaw(const std::vector<uint8_t> & packet)
{
  if (!pub_raw_) return;
  std_msgs::msg::ByteMultiArray msg;
  msg.data.assign(packet.begin(), packet.end());
  pub_raw_->publish(msg);
}

// Fungsi: Memvalidasi dan mendekode frame protokol IMU ke state akselerasi, gyro, dan orientasi.
bool ImuNode::parsePacket(const std::vector<uint8_t> & data)
{
  if (data.size() != 11 || data[0] != 0x55) return false;
  uint16_t sum = 0;
  for (size_t i = 0; i < 10; ++i) sum += data[i];
  if ((sum & 0xFF) != data[10]) return false;

  uint8_t ptype = data[1];
  int16_t v0, v1, v2;
  rclcpp::Time packet_stamp(0, 0, get_clock()->get_clock_type());
  if (!packetStampNow(packet_stamp)) return false;
  const double packet_steady_sec = nowSec();

  switch (ptype) {
  case 0x51: // accel
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
  case 0x52: // gyro
    std::memcpy(&v0, &data[2], 2);
    std::memcpy(&v1, &data[4], 2);
    std::memcpy(&v2, &data[6], 2);
    gx_ = (v0 / 32768.0) * 2000.0;
    gy_ = (v1 / 32768.0) * 2000.0;
    gz_ = (v2 / 32768.0) * 2000.0;
    has_gyro_ = true;
    if (last_gyro_packet_time_ > 0.0) {
      const double period = packet_steady_sec - last_gyro_packet_time_;
      if (period > 0.0 && period < 1.0) {
        ++gyro_period_count_;
        const double delta = period - gyro_period_mean_sec_;
        gyro_period_mean_sec_ += delta / static_cast<double>(gyro_period_count_);
        gyro_period_m2_sec2_ += delta * (period - gyro_period_mean_sec_);
        const double ms = period * 1000.0;
        const size_t bin = ms <= 5.0 ? 0U : ms <= 10.0 ? 1U : ms <= 15.0 ? 2U :
          ms <= 25.0 ? 3U : ms <= 40.0 ? 4U : ms <= 80.0 ? 5U : 6U;
        ++gyro_period_hist_[bin];
      }
    }
    last_gyro_packet_time_ = packet_steady_sec;
    last_gyro_measurement_stamp_ = packet_stamp;
    ++packets_gyro_;
    return true;
  case 0x53: // angle
    std::memcpy(&v0, &data[2], 2);
    std::memcpy(&v1, &data[4], 2);
    std::memcpy(&v2, &data[6], 2);
    roll_  = (v0 / 32768.0) * 180.0;
    pitch_ = (v1 / 32768.0) * 180.0;
    yaw_   = (v2 / 32768.0) * 180.0;
    has_angle_ = true;
    ++packets_angle_;
    last_orientation_packet_time_ = packet_steady_sec;
    last_orientation_measurement_stamp_ = packet_stamp;
    orientation_recovery_attempted_ = false;
    orientation_recovery_started_time_ = 0.0;
    return true;
  case 0x54: // mag
    std::memcpy(&v0, &data[2], 2);
    std::memcpy(&v1, &data[4], 2);
    std::memcpy(&v2, &data[6], 2);
    mx_ = static_cast<double>(v0);
    my_ = static_cast<double>(v1);
    mz_ = static_cast<double>(v2);
    has_mag_ = true;
    last_mag_packet_time_ = packet_steady_sec;
    last_mag_measurement_stamp_ = packet_stamp;
    ++packets_mag_;
    return true;
  case 0x59: { // quaternion fallback: q0=w, q1=x, q2=y, q3=z
    int16_t q0i, q1i, q2i, q3i;
    std::memcpy(&q0i, &data[2], 2);
    std::memcpy(&q1i, &data[4], 2);
    std::memcpy(&q2i, &data[6], 2);
    std::memcpy(&q3i, &data[8], 2);
    double qw = q0i / 32768.0;
    double qx = q1i / 32768.0;
    double qy = q2i / 32768.0;
    double qz = q3i / 32768.0;
    const double norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if (norm < 1e-6) return false;
    qw /= norm; qx /= norm; qy /= norm; qz /= norm;
    const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    const double sinp = std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0);
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    roll_ = std::atan2(sinr_cosp, cosr_cosp) * 180.0 / M_PI;
    pitch_ = std::asin(sinp) * 180.0 / M_PI;
    yaw_ = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    has_angle_ = true;
    ++packets_quat_;
    last_orientation_packet_time_ = packet_steady_sec;
    last_orientation_measurement_stamp_ = packet_stamp;
    orientation_recovery_attempted_ = false;
    orientation_recovery_started_time_ = 0.0;
    return true;
  }
  default:
    return false;
  }
}

// Fungsi: Membaca serial non-blocking, memproses frame baru, serta menangani timeout/reconnect.
void ImuNode::pollSerial()
{
  if (!ser_ || !ser_->isOpen()) {
    if (nowSec() - last_reconnect_try_ >= reconnect_interval_sec_) {
      openSerial(false);
    }
    return;
  }

  try {
    size_t n = ser_->available();
    if (n == 0) {
      if (last_data_time_ > 0.0 && nowSec() - last_data_time_ > data_timeout_sec_) {
        logRateLimited(
          "IMU tidak mengirim data selama " + std::to_string(data_timeout_sec_) +
          " s; serial dibuka ulang", "info");
        closeSerial();
        last_reconnect_try_ = nowSec();
      }
      return;
    }
    std::string data = ser_->read(std::min<size_t>(n, 1024));
    if (data.empty()) return;

    last_data_time_ = nowSec();
    bytes_received_ += data.size();
    consecutive_serial_errors_ = 0;

    // Append bytes to buffer
    buf_.insert(buf_.end(), data.begin(), data.end());
    if (buf_.size() > 10000)
      buf_.erase(buf_.begin(), buf_.end() - 1000);

    if (debug_) {
      double sec = nowSec();
      if (sec - last_status_time_ > 10.0) {
        RCLCPP_INFO(this->get_logger(),
          "IMU STATUS: Bytes=%zu, Packets=%zu, Buffer=%zu",
          bytes_received_, packets_parsed_, buf_.size());
        last_status_time_ = sec;
      }
    }

    bool published = false;
    bool published_mag = false;

    // Parse complete 11-byte packets
    while (buf_.size() >= 11) {
      if (buf_[0] != 0x55) { buf_.erase(buf_.begin()); continue; }
      uint8_t ptype = buf_[1];
      if (ptype < 0x51 || ptype > 0x59) { buf_.erase(buf_.begin()); continue; }

      uint16_t sum = 0;
      for (size_t i = 0; i < 10; ++i) sum += buf_[i];
      if ((sum & 0xFF) != buf_[10]) { buf_.erase(buf_.begin()); continue; }

      std::vector<uint8_t> packet(buf_.begin(), buf_.begin() + 11);
      buf_.erase(buf_.begin(), buf_.begin() + 11);

      if (publish_raw_) publishRaw(packet);

      uint8_t parsed_type = packet[1];
      if (parsePacket(packet)) {
        packets_parsed_++;
        last_valid_packet_time_ = nowSec();
        if (parsed_type == 0x54) {
          published_mag = true;
        } else if (parsed_type == 0x52) {
          // Gyro is the high-rate EKF measurement. Publish on its fresh packet
          // epoch; accel/orientation are attached only when time-coherent.
          published = true;
        }
      }
    }

    const double stream_now = nowSec();
    if (last_valid_packet_time_ > 0.0 &&
        stream_now - last_valid_packet_time_ > data_timeout_sec_) {
      logRateLimited("IMU bytes masuk tetapi tidak ada frame WIT checksum-valid; serial dibuka ulang", "info");
      closeSerial();
      buf_.clear();
      last_reconnect_try_ = stream_now;
      return;
    }
    const double orientation_age = stream_now - last_orientation_packet_time_;
    if (bytes_received_ > 0 && orientation_age > orientation_packet_timeout_sec_) {
      if (!orientation_recovery_attempted_ &&
          stream_now - last_sensor_config_try_ >= sensor_config_retry_sec_) {
        // Transient stream loss is a recovery event, not a warning storm.
        RCLCPP_INFO(
          this->get_logger(),
          "IMU orientation stale %.1fs; re-apply stream config sekali "
          "(acc=%zu gyro=%zu angle=%zu quat=%zu mag=%zu)",
          orientation_age, packets_acc_, packets_gyro_, packets_angle_, packets_quat_, packets_mag_);
        orientation_recovery_attempted_ = true;
        orientation_recovery_started_time_ = stream_now;
        (void)configureSensorOutput(false);
      } else if (orientation_recovery_attempted_ &&
                 orientation_recovery_started_time_ > 0.0 &&
                 stream_now - orientation_recovery_started_time_ > orientation_reopen_sec_) {
        const bool valid_stream_stale = last_valid_packet_time_ <= 0.0 ||
          stream_now - last_valid_packet_time_ > data_timeout_sec_;
        if (valid_stream_stale) {
          // Only tear down USB serial when the whole checksum-valid WIT stream
          // is stale. Missing ANGLE alone is a field-health issue and must not
          // flap /imu/connected while ACC/GYRO/MAG packets are still healthy.
          logRateLimited(
            "IMU valid WIT stream hilang setelah recovery; serial dibuka ulang", "info");
          closeSerial();
          buf_.clear();
          last_reconnect_try_ = stream_now;
          last_orientation_packet_time_ = stream_now;
          orientation_recovery_attempted_ = false;
          orientation_recovery_started_time_ = 0.0;
          return;
        }
        logRateLimited(
          "IMU ANGLE masih stale tetapi WIT stream valid tetap aktif; port dipertahankan", "info");
        orientation_recovery_attempted_ = false;
        orientation_recovery_started_time_ = 0.0;
        last_sensor_config_try_ = stream_now;
      }
    }

    const auto now = Clock::now();
    // Gyro packet is the fusion boundary. Jangan memakai gate persis 1/f karena
    // jitter USB/timer sub-ms dapat membuat sampel 19.x ms
    // ditolak pada target 50 Hz dan menghasilkan aliasing sekitar 25-33 Hz.
    // Toleransi 20% tetap membatasi sumber yang lebih cepat, tetapi menerima
    // boundary sensor nominal pada rate yang dikonfigurasi.
    const auto nominal_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    const auto min_period = std::chrono::duration_cast<Clock::duration>(nominal_period * 0.80);
    if (published && (last_publish_time_.time_since_epoch().count() == 0 ||
        now - last_publish_time_ >= min_period)) {
      if (publishImu()) {
        last_publish_time_ = now;
        if (!stream_announced_) {
          RCLCPP_INFO(
            this->get_logger(),
            "IMU stream valid: /imu/data aktif (acc=%zu gyro=%zu angle=%zu quat=%zu mag=%zu)",
            packets_acc_, packets_gyro_, packets_angle_, packets_quat_, packets_mag_);
          stream_announced_ = true;
        }
      }
    }
    if (published_mag) publishMag();

  } catch (const serial::IOException & e) {
    // EIO pada USB serial berarti descriptor sudah tidak sehat; retry 11 kali
    // pada fd yang sama hanya membuat warning/noise. Tutup dan biarkan hot-plug
    // reconnect membuka descriptor baru.
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "IMU serial I/O lost; reconnect dijadwalkan: %s", e.what());
    closeSerial();
    buf_.clear();
    consecutive_serial_errors_ = 0;
    last_reconnect_try_ = nowSec();
  } catch (const serial::SerialException & e) {
    // Beberapa USB-UART dapat melaporkan readiness sesaat sebelum byte tersedia.
    // Jangan reset port pada satu kejadian tunggal; reconnect hanya setelah tiga
    // exception berturut-turut tanpa satu read sukses di antaranya.
    ++consecutive_serial_errors_;
    if (consecutive_serial_errors_ < 3) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "IMU serial transient (%d/3), link dipertahankan: %s",
        consecutive_serial_errors_, e.what());
      return;
    }
    RCLCPP_INFO(
      this->get_logger(), "IMU serial tidak sehat setelah %d exception; reconnect: %s",
      consecutive_serial_errors_, e.what());
    closeSerial();
    buf_.clear();
    consecutive_serial_errors_ = 0;
    last_reconnect_try_ = nowSec();
  }
}

// Fungsi: Entry point proses IMU; menginisialisasi ROS, spin node, lalu shutdown bersih.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
