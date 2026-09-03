#include "gnss/gnss_node.hpp"

#include <cstdint>
#include <array>
#include <cstring>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <climits>
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <sstream>
#include <iomanip>
#include <limits>
#include <ctime>
#include <numeric>
#include <glob.h>

using Clock = std::chrono::steady_clock;

// Fungsi: Mengambil waktu monotonic untuk timeout data dan logika reconnect GNSS.
static double nowSec()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    Clock::now().time_since_epoch()).count() / 1000.0;
}

// Fungsi: Menyelesaikan symlink serial agar identitas device GNSS dapat dibandingkan dengan benar.
static std::string resolvePath(const std::string & p)
{
  char r[PATH_MAX] = {0};
  if (p.empty()) return p;
  if (realpath(p.c_str(), r)) return std::string(r);
  return p;
}


// Memvalidasi checksum NMEA: XOR semua karakter di antara '$' dan '*'.
// Kalimat tanpa checksum tetap diterima karena beberapa konfigurasi receiver
// dapat menonaktifkan checksum, tetapi checksum yang ada wajib benar.
// Fungsi: Memvalidasi checksum kalimat NMEA; kalimat tanpa checksum tetap diizinkan sesuai receiver.
static bool isValidNmeaChecksum(const std::string & sentence)
{
  const auto star = sentence.find('*');
  if (star == std::string::npos) return true;
  if (sentence.empty() || sentence.front() != '$' || star + 2 >= sentence.size()) return false;

  unsigned int expected = 0;
  try {
    expected = static_cast<unsigned int>(std::stoul(sentence.substr(star + 1, 2), nullptr, 16));
  } catch (...) {
    return false;
  }

  uint8_t calculated = 0;
  for (size_t i = 1; i < star; ++i) calculated ^= static_cast<uint8_t>(sentence[i]);
  return calculated == expected;
}


// Mengubah koordinat NMEA ddmm.mmmm/dddmm.mmmm menjadi derajat desimal.
// Nilai baru hanya diterima bila field angka, menit, derajat, dan hemisfer valid.
static bool parseNmeaCoordinate(
  const std::string & raw_text, const std::string & hemisphere,
  bool latitude, double & degrees_out)
{
  if (raw_text.empty() || hemisphere.size() != 1) return false;

  double raw = 0.0;
  try {
    raw = std::stod(raw_text);
  } catch (...) {
    return false;
  }
  if (!std::isfinite(raw) || raw < 0.0) return false;

  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - static_cast<double>(degrees) * 100.0;
  const int max_degrees = latitude ? 90 : 180;
  if (degrees < 0 || degrees > max_degrees || minutes < 0.0 || minutes >= 60.0) {
    return false;
  }
  // Pada 90/180 derajat, bagian menit harus nol.
  if (degrees == max_degrees && minutes > 1e-9) return false;

  const char hemi = hemisphere[0];
  if (latitude && hemi != 'N' && hemi != 'S') return false;
  if (!latitude && hemi != 'E' && hemi != 'W') return false;

  degrees_out = static_cast<double>(degrees) + minutes / 60.0;
  if (hemi == 'S' || hemi == 'W') degrees_out = -degrees_out;
  return std::isfinite(degrees_out);
}

// Fungsi: Mengumpulkan kandidat port serial dari pola device Linux.
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

// Fungsi: Menginisialisasi konfigurasi GNSS, publisher, serial parser, dan timer polling.
GnssNode::GnssNode(const rclcpp::NodeOptions & options)
: Node("data_cuav_node", options)
{
  port_ = this->declare_parameter<std::string>("port", "auto");
  auto_port_id_contains_ = this->declare_parameter<std::string>(
    "auto_port_id_contains", "1a86_USB_Serial");
  // GNSS dan ESC lapangan sama-sama memakai CH340 1a86:7523 dengan ID_SERIAL
  // identik. by-path menjadi selector utama agar frame UBX tidak pernah dikirim
  // ke ESC hanya karena nomor ttyUSB berubah.
  auto_port_path_contains_ = this->declare_parameter<std::string>(
    "auto_port_path_contains", "usb-0:3.4:1.0");
  // ROS 2 menyimpan parameter INTEGER sebagai int64_t. Gunakan tipe itu secara
  // eksplisit lalu konversi ke int setelah divalidasi agar portable di ARM64/x86_64.
  const int64_t baudrate_param =
    this->declare_parameter<int64_t>("baudrate", static_cast<int64_t>(38400));
  if (baudrate_param <= 0 || baudrate_param > static_cast<int64_t>(INT_MAX)) {
    throw std::invalid_argument("Parameter baudrate harus > 0 dan <= INT_MAX");
  }
  baudrate_ = static_cast<int>(baudrate_param);
  frame_id_ = this->declare_parameter<std::string>("frame_id", "gnss_link");
  velocity_frame_id_ = this->declare_parameter<std::string>("velocity_frame_id", "enu");
  publish_raw_ = this->declare_parameter<bool>("publish_raw", true);
  auto_baud_enabled_ = this->declare_parameter<bool>("auto_baud", false);
  velocity_stale_timeout_ = std::max(0.1,
    this->declare_parameter<double>("velocity_stale_timeout", 1.0));
  data_timeout_sec_ = std::max(1.0,
    this->declare_parameter<double>("data_timeout_sec", 3.0));

  // Autonomous GNSS gate. u-blox merekomendasikan hanya memakai fix yang
  // ditandai valid (gnssFixOK); batas sat/DOP/hAcc mengikuti praktik PX4
  // untuk GNSS standar dan dapat diubah lewat YAML tanpa recompilation.
  configure_ubx_nav_pvt_on_connect_ =
    this->declare_parameter<bool>("configure_ubx_nav_pvt_on_connect", true);
  configure_navigation_rate_ =
    this->declare_parameter<bool>("configure_navigation_rate", true);
  navigation_rate_hz_ = std::clamp(
    this->declare_parameter<double>("navigation_rate_hz", 10.0), 1.0, 25.0);
  configure_dynamic_model_ =
    this->declare_parameter<bool>("configure_dynamic_model", true);
  const std::string dynamic_model_name =
    this->declare_parameter<std::string>("dynamic_model", "automotive");
  if (dynamic_model_name == "stationary") dynamic_model_ = 2u;
  else if (dynamic_model_name == "pedestrian") dynamic_model_ = 3u;
  else if (dynamic_model_name == "automotive") dynamic_model_ = 4u;
  else if (dynamic_model_name == "sea") dynamic_model_ = 5u;
  else if (dynamic_model_name == "airborne1g") dynamic_model_ = 6u;
  else if (dynamic_model_name == "airborne2g") dynamic_model_ = 7u;
  else if (dynamic_model_name == "airborne4g") dynamic_model_ = 8u;
  else if (dynamic_model_name == "bike") dynamic_model_ = 10u;
  else dynamic_model_ = 0u;  // portable fallback
  poll_nav_cov_ = this->declare_parameter<bool>("poll_nav_cov", true);
  nav_cov_poll_rate_hz_ = std::clamp(
    this->declare_parameter<double>("nav_cov_poll_rate_hz", 5.0), 0.2, 25.0);
  poll_nav_dop_ = this->declare_parameter<bool>("poll_nav_dop", true);
  nav_dop_poll_rate_hz_ = std::clamp(
    this->declare_parameter<double>("nav_dop_poll_rate_hz", 1.0), 0.1, 10.0);
  cov_wait_timeout_sec_ = std::clamp(
    this->declare_parameter<double>("cov_wait_timeout_sec", 0.08), 0.0, 0.5);
  timestamp_mode_ = this->declare_parameter<std::string>("timestamp_mode", "auto");
  utc_stamp_max_offset_sec_ = std::max(1.0,
    this->declare_parameter<double>("utc_stamp_max_offset_sec", 10.0));
  position_fit_window_sec_ = std::clamp(
    this->declare_parameter<double>("position_fit_window_sec", 3.0), 1.0, 30.0);
  const int64_t fit_min_samples = this->declare_parameter<int64_t>(
    "position_fit_min_samples", static_cast<int64_t>(5));
  position_fit_min_samples_ = static_cast<int>(
    std::clamp<int64_t>(fit_min_samples, static_cast<int64_t>(3), static_cast<int64_t>(200)));
  position_fit_min_baseline_m_ = std::max(0.1,
    this->declare_parameter<double>("position_fit_min_baseline_m", 1.0));
  position_fit_hacc_multiplier_ = std::max(0.0,
    this->declare_parameter<double>("position_fit_hacc_multiplier", 2.5));
  prefer_ubx_nav_pvt_ = this->declare_parameter<bool>("prefer_ubx_nav_pvt", true);
  require_ubx_nav_pvt_for_fix_ =
    this->declare_parameter<bool>("require_ubx_nav_pvt_for_fix", true);
  ubx_preference_hold_sec_ = std::max(
    0.0, this->declare_parameter<double>("ubx_preference_hold_sec", 2.0));
  accept_rmc_position_fallback_ =
    this->declare_parameter<bool>("accept_rmc_position_fallback", false);
  allow_validated_nmea_fallback_ =
    this->declare_parameter<bool>("allow_validated_nmea_fallback", true);
  nmea_fallback_after_sec_ = std::max(2.0,
    this->declare_parameter<double>("nmea_fallback_after_sec", 8.0));
  const int64_t fallback_min_satellites_param =
    this->declare_parameter<int64_t>("nmea_fallback_min_satellites", static_cast<int64_t>(8));
  nmea_fallback_min_satellites_ = static_cast<int>(
    std::clamp<int64_t>(fallback_min_satellites_param, static_cast<int64_t>(4),
                        static_cast<int64_t>(255)));
  nmea_fallback_max_hdop_ = std::max(0.5,
    this->declare_parameter<double>("nmea_fallback_max_hdop", 1.4));
  ubx_config_retry_sec_ = std::max(0.5,
    this->declare_parameter<double>("ubx_config_retry_sec", 2.0));
  const int64_t ubx_config_max_attempts_param =
    this->declare_parameter<int64_t>("ubx_config_max_attempts", static_cast<int64_t>(3));
  ubx_config_max_attempts_ = static_cast<int>(
    std::clamp<int64_t>(ubx_config_max_attempts_param, static_cast<int64_t>(1),
                        static_cast<int64_t>(10)));
  state_publish_period_sec_ = std::max(0.5,
    this->declare_parameter<double>("state_publish_period_sec", 1.0));
  // Parameter INTEGER rclcpp bernilai int64_t; std::max(int, int64_t) gagal
  // dikompilasi pada GCC/ARM64. Clamp dalam domain int64_t, lalu cast eksplisit.
  const int64_t min_satellites_param =
    this->declare_parameter<int64_t>("min_satellites", static_cast<int64_t>(8));
  min_satellites_ = static_cast<int>(
    std::clamp<int64_t>(min_satellites_param, static_cast<int64_t>(0),
                        static_cast<int64_t>(255)));
  max_dop_ = std::max(0.1, this->declare_parameter<double>("max_dop", 2.0));
  // UBX NAV-PVT memberi hAcc/pDOP langsung; keduanya wajib lolos untuk
  // autonomous-grade. Fix yang tidak lolos tetap tersedia untuk diagnosis.
  require_dop_for_ubx_quality_ =
    this->declare_parameter<bool>("require_dop_for_ubx_quality", true);
  max_hacc_m_ = std::max(0.1, this->declare_parameter<double>("max_hacc_m", 2.5));
  max_sacc_mps_ = std::max(0.05, this->declare_parameter<double>("max_sacc_mps", 0.8));
  require_speed_accuracy_for_quality_ =
    this->declare_parameter<bool>("require_speed_accuracy_for_quality", false);
  accept_ubx_gnss_dr_fix_ =
    this->declare_parameter<bool>("accept_ubx_gnss_dr_fix", false);

  ser_timeout_ = serial::Timeout::simpleTimeout(10); // 10ms timeout for read

  // SensorDataQoS cocok untuk aliran sensor real-time: data lama boleh dilewati
  // daripada menahan antrean dan menambah latency.
  auto qos = rclcpp::SensorDataQoS().keep_last(10);

  pub_fix_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix", qos);
  pub_fix_raw_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gnss/fix_raw", qos);
  pub_vel_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("gnss/vel", qos);
  pub_vel_fit_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "gnss/velocity_position_fit", qos);
  pub_quality_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("gnss/quality", qos);
  pub_motion_diag_ = this->create_publisher<std_msgs::msg::String>("gnss/motion_diagnostics", 10);
  pub_state_ = this->create_publisher<std_msgs::msg::String>("gnss/state", 10);
  pub_connected_ = this->create_publisher<std_msgs::msg::Bool>(
    "/gnss/connected", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
  if (publish_raw_) {
    pub_raw_ = this->create_publisher<std_msgs::msg::String>("gnss/raw", 10);
  }

  ser_ = std::make_unique<serial::Serial>();
  openSerial(true);
  last_reconnect_try_ = nowSec();

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&GnssNode::pollSerial, this));
  state_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(state_publish_period_sec_)),
    std::bind(&GnssNode::publishState, this));
  // NavSatFix dipublish per epoch; /gnss/state periodik agar log runtime selalu
  // memiliki alasan gate, protocol state, counter parser, dan status CFG-VALSET.
}

// Fungsi: Menutup port serial GNSS secara aman ketika node dihancurkan.
GnssNode::~GnssNode()
{
  closeSerial();
}

// Fungsi: Menentukan apakah node harus melakukan auto-detection port GNSS.
bool GnssNode::isAutoPort() const
{
  std::string p = port_;
  std::transform(p.begin(), p.end(), p.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return p.empty() || p == "auto";
}

// Fungsi: Menyusun kandidat port GNSS dengan memprioritaskan symlink persisten yang benar.
std::vector<std::string> GnssNode::candidatePorts()
{
  if (!isAutoPort()) return {port_};

  std::vector<std::string> out;

  // Physical USB topology is authoritative when configured. This is required
  // on the current mini-PC because GNSS and ESC expose the SAME CH340
  // ID_VENDOR/ID_MODEL/ID_SERIAL. /dev/serial/by-path remains stable when the
  // kernel renumbers ttyUSB0/1/2. If the physical socket changes we fail
  // closed; the operator may override `port:=...` deliberately.
  if (!auto_port_path_contains_.empty()) {
    for (const auto & p : globPattern("/dev/serial/by-path/*")) {
      if (p.find(auto_port_path_contains_) != std::string::npos) out.push_back(p);
    }
    // Do NOT fall back to CH340 by-id when this configured physical socket is
    // absent: the remaining CH340 may be the ESC. Explicit `port:=...` is the
    // only deliberate override for moved hardware.
    if (out.size() == 1U) return out;
    return {};
  }

  auto by_id = globPattern("/dev/serial/by-id/*");
  if (!auto_port_id_contains_.empty()) {
    for (const auto & p : by_id) {
      if (p.find(auto_port_id_contains_) != std::string::npos) out.push_back(p);
    }
    if (out.size() == 1U) return out;
    return {};
  }

  out = std::move(by_id);
  auto acm = globPattern("/dev/ttyACM*");
  auto usb = globPattern("/dev/ttyUSB*");
  out.insert(out.end(), acm.begin(), acm.end());
  out.insert(out.end(), usb.begin(), usb.end());

  // Deduplicate by realpath
  std::vector<std::string> deduped;
  std::vector<std::string> seen;
  for (const auto & p : out) {
    std::string r = resolvePath(p);
    if (std::find(seen.begin(), seen.end(), r) == seen.end()) {
      seen.push_back(r);
      deduped.push_back(p);
    }
  }
  return deduped;
}

// Fungsi: Mem-probe stream serial dan memastikan data terlihat seperti GNSS sebelum port diterima.
bool GnssNode::looksLikeGnssStream(double probe_sec)
{
  if (!ser_ || !ser_->isOpen()) return false;
  auto deadline = Clock::now() + std::chrono::duration<double>(probe_sec);
  std::string sample;
  while (Clock::now() < deadline && sample.size() < 4096) {
    try {
      size_t n = ser_->available();
      std::string chunk;
      if (n > 0) chunk = ser_->read(n);
      else chunk = ser_->read(1);
      sample += chunk;
    } catch (...) { break; }
  }
  if (sample.empty()) return false;
  return sample.find("$GP") != std::string::npos ||
         sample.find("$GN") != std::string::npos ||
         sample.find("$GA") != std::string::npos ||
         sample.find("$BD") != std::string::npos ||
         sample.find("\xB5\x62") != std::string::npos;
}

// Fungsi: Membuka kandidat serial GNSS/baud yang valid dan mengaktifkan retry bila gagal.
void GnssNode::openSerial(bool initial)
{
  auto cands = candidatePorts();
  if (cands.empty()) {
    // Recoverable at runtime: USB GNSS may be hot-plugged later. Keep the
    // localization safety gate closed, but do not classify a retryable startup
    // condition as a fatal node error.
    RCLCPP_WARN(this->get_logger(), "No GNSS serial candidates found; hot-plug retry remains active");
    return;
  }

  bool auto_baud = isAutoPort() && auto_baud_enabled_;
  std::vector<int> baudrates;
  if (auto_baud) {
    // Nilai YAML dicoba pertama, lalu baud umum u-blox/USB-UART. Beberapa unit
    // lapangan pernah tersimpan pada rate tinggi, jadi jangan mengasumsikan
    // factory 38400 saja.
    const std::array<int, 9> common{{
      baudrate_, 38400, 9600, 57600, 115200, 230400, 460800, 921600, 19200}};
    for (const int baud : common) {
      if (baud > 0 && std::find(baudrates.begin(), baudrates.end(), baud) == baudrates.end())
        baudrates.push_back(baud);
    }
  } else {
    // Port eksplisit/direct: jangan probe baud lain. CUAV dikunci 38400 dari launch.
    // Ini menghindari open/close USB-UART berulang saat satu open gagal.
    baudrates = {baudrate_};
  }

  RCLCPP_INFO(this->get_logger(), "Opening GNSS serial from %zu candidate(s) (auto_baud=%s)",
    cands.size(), auto_baud ? "true" : "false");

  std::string last_err;
  for (int baud : baudrates) {
    for (const auto & cand : cands) {
      try {
        // Close existing port before reassigning
        if (ser_ && ser_->isOpen()) ser_->close();
        if (!ser_) ser_ = std::make_unique<serial::Serial>();
        
        ser_->setPort(cand);
        ser_->setBaudrate(static_cast<uint32_t>(baud));
        ser_->setTimeout(ser_timeout_);
        ser_->open();
        try { ser_->flushInput(); } catch (...) {}

        // Probe pasif dahulu. Jika periodic output receiver sedang disabled,
        // kirim UBX MON-VER poll (read-only, tidak mengubah konfigurasi) lalu
        // tunggu respons. Ini membuat NEO-M9N tetap dapat ditemukan tanpa
        // bergantung pada NMEA yang sudah aktif.
        // Give periodic NMEA (commonly 1 Hz) enough time to appear. The old
        // 0.25 s passive window could miss an otherwise healthy receiver when
        // UBX input was disabled, causing a false "no GNSS stream" result.
        bool stream_ok = looksLikeGnssStream(1.10);
        if (!stream_ok) {
          (void)sendUbxMessage(0x0A, 0x04, {});  // UBX-MON-VER poll
          stream_ok = looksLikeGnssStream(0.40);
        }
        if (stream_ok) {
          active_port_ = cand;
          baudrate_ = baud;
          buf_.clear();
          const double connected_now = nowSec();
          last_data_time_ = connected_now;
          serial_connected_since_ = connected_now;
          last_ubx_config_send_time_ = -1.0;
          ubx_config_attempts_ = 0;
          ubx_cfg_ack_count_ = 0;
          ubx_cfg_nak_count_ = 0;
          ubx_nav_pvt_seen_ = false;
          last_ubx_pvt_time_ = -1.0;
          last_position_from_ubx_ = false;
          last_ubx_gnss_fix_ok_ = false;
          last_ubx_fix_type_ = 0;
          last_ubx_flags_ = 0;
          last_ubx_flags2_ = 0;
          last_ubx_flags3_ = 0;
          last_ubx_invalid_llh_ = false;
          ubx_height_mm_ = 0;
          head_vehicle_rad_.reset();
          mag_declination_rad_.reset();
          mag_declination_accuracy_rad_.reset();
          last_itow_ms_ = 0;
          ubx_vacc_mm_ = 0;
          vel_n_mps_.reset(); vel_e_mps_.reset(); vel_d_mps_.reset();
          nav_cov_ = NavCovEpoch{}; nav_dop_ = NavDopEpoch{};
          pending_ubx_publish_ = false;
          fit_samples_.clear();
          pvt_rate_hz_ema_ = 0.0; have_rate_itow_ = false; rate_last_itow_ms_ = 0;
          ubx_duplicate_itow_count_ = 0; ubx_out_of_order_itow_count_ = 0;
          last_cov_poll_time_ = -1.0; last_dop_poll_time_ = -1.0;
          itow_anchor_valid_ = false; last_timestamp_source_ = "arrival"; last_measurement_age_sec_ = 0.0;
          last_nmea_gga_time_ = -1.0;
          last_nmea_fix_quality_ = 0;
          last_nmea_gga_checksum_present_ = false;
          nmea_fallback_active_ = false;
          fix_status_ = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
          RCLCPP_INFO(this->get_logger(), "GNSS connected on %s @ %d", cand.c_str(), baud);
          if (configure_ubx_nav_pvt_on_connect_) {
            if (configureUbxNavPvtOutput()) {
              ubx_config_attempts_ = 1;
              last_ubx_config_send_time_ = connected_now;
              RCLCPP_INFO(this->get_logger(),
                "Permintaan u-blox RAM config #1 dikirim: enable UBX/NAV-PVT pada UART1+UART2+USB; menunggu ACK/NAV-PVT");
            } else {
              RCLCPP_WARN(this->get_logger(),
                "Gagal mengirim u-blox RAM config NAV-PVT; akan retry dan hanya memakai NMEA fallback bila quality ketat lolos");
            }
          }
          return;
        }
        ser_->close();
        last_err = cand + "@" + std::to_string(baud) + ": no GNSS stream";
      } catch (const std::exception & e) {
        last_err = cand + "@" + std::to_string(baud) + ": " + e.what();
        try { ser_->close(); } catch (...) {}
      }
    }
  }

  // This path is intentionally WARN: the node remains alive, publishes
  // disconnected state, and retries. LocalizationCore keeps motion/localization
  // gates closed until a valid GNSS stream is actually available.
  RCLCPP_WARN(this->get_logger(),
    "GNSS serial not ready. Last: %s — will retry every 2s", last_err.c_str());
  last_reconnect_try_ = nowSec();
}

// Fungsi: Menutup serial GNSS tanpa mengganggu mekanisme reconnect.
void GnssNode::closeSerial()
{
  if (ser_) {
    try { if (ser_->isOpen()) ser_->close(); } catch (...) {}
    ser_.reset();
  }
  active_port_.clear();
  serial_connected_since_ = -1.0;
  nmea_fallback_active_ = false;
  ubx_nav_pvt_seen_ = false;
  last_position_from_ubx_ = false;
  last_ubx_gnss_fix_ok_ = false;
  last_ubx_pvt_time_ = -1.0;
  last_nmea_gga_time_ = -1.0;
  last_nmea_fix_quality_ = 0;
  last_nmea_gga_checksum_present_ = false;
  lat_.reset();
  lon_.reset();
  alt_.reset();
  sats_.reset();
  quality_dop_.reset();
  speed_mps_.reset();
  heading_rad_.reset();
  heading_accuracy_rad_.reset();
  vel_n_mps_.reset(); vel_e_mps_.reset(); vel_d_mps_.reset();
  ubx_hacc_mm_ = 0; ubx_vacc_mm_ = 0; ubx_sacc_mmps_ = 0;
  last_itow_ms_ = 0; last_ubx_flags_ = 0; last_ubx_flags2_ = 0; last_ubx_flags3_ = 0;
  last_ubx_invalid_llh_ = false; ubx_height_mm_ = 0;
  head_vehicle_rad_.reset(); mag_declination_rad_.reset(); mag_declination_accuracy_rad_.reset();
  nav_cov_ = NavCovEpoch{}; nav_dop_ = NavDopEpoch{};
  pending_ubx_publish_ = false; fit_samples_.clear();
  pvt_rate_hz_ema_ = 0.0; have_rate_itow_ = false; rate_last_itow_ms_ = 0;
  last_cov_poll_time_ = -1.0; last_dop_poll_time_ = -1.0;
  itow_anchor_valid_ = false; last_timestamp_source_ = "arrival"; last_measurement_age_sec_ = 0.0;
  markNoFix();
}

// Fungsi: Menerbitkan payload GNSS mentah untuk diagnostik ketika fitur raw diaktifkan.
void GnssNode::emitRaw(const std::string & text)
{
  if (!publish_raw_ || !pub_raw_) return;
  std_msgs::msg::String msg;
  msg.data = text;
  pub_raw_->publish(msg);
}


// Fungsi: Memeriksa apakah latitude/longitude terakhir finite, berada dalam batas bumi, dan memiliki fix.
bool GnssNode::hasValidPosition() const
{
  // Jangan membatasi koordinat hanya ke Indonesia. Validasi geodetik global
  // juga membuat paket dapat diuji di lokasi lain atau dengan rosbag.
  return lat_.has_value() && lon_.has_value() &&
         std::isfinite(*lat_) && std::isfinite(*lon_) &&
         *lat_ >= -90.0 && *lat_ <= 90.0 &&
         *lon_ >= -180.0 && *lon_ <= 180.0 &&
         !(std::abs(*lat_) < 1e-12 && std::abs(*lon_) < 1e-12);
}

// Fungsi: Mengirim frame UBX lengkap dengan checksum ke receiver u-blox.
// Digunakan hanya untuk konfigurasi RAM yang aman diulang saat reconnect; tidak
// ada write ke BBR/Flash sehingga setting pabrikan tidak diubah permanen.
bool GnssNode::sendUbxMessage(
  uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t> & payload)
{
  if (!ser_ || !ser_->isOpen()) return false;
  if (payload.size() > 0xFFFFu) return false;

  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 8u);
  frame.push_back(0xB5);
  frame.push_back(0x62);
  frame.push_back(msg_class);
  frame.push_back(msg_id);
  const uint16_t length = static_cast<uint16_t>(payload.size());
  frame.push_back(static_cast<uint8_t>(length & 0xFFu));
  frame.push_back(static_cast<uint8_t>((length >> 8) & 0xFFu));
  frame.insert(frame.end(), payload.begin(), payload.end());

  uint8_t ck_a = 0, ck_b = 0;
  ubxChecksum(&frame[2], payload.size() + 4u, ck_a, ck_b);
  frame.push_back(ck_a);
  frame.push_back(ck_b);

  try {
    const size_t written = ser_->write(frame.data(), frame.size());
    return written == frame.size();
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "UBX config write gagal: %s", e.what());
    return false;
  }
}

// Fungsi: Mengaktifkan UBX NAV-PVT pada semua port keluaran M9 yang umum.
// CUAV dapat merutekan konektor eksternal ke UART1 atau UART2, sedangkan beberapa
// bench setup memakai USB. Karena driver tidak dapat mengetahui port internal
// u-blox yang dipakai hanya dari nama /dev/tty*, ketiga output diaktifkan pada
// layer RAM saja. Ini tidak mengubah Flash/BBR dan aman diulang saat reconnect.
bool GnssNode::configureUbxNavPvtOutput()
{
  auto append_u32_le = [](std::vector<uint8_t> & out, uint32_t value) {
      out.push_back(static_cast<uint8_t>(value & 0xFFu));
      out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
      out.push_back(static_cast<uint8_t>((value >> 16) & 0xFFu));
      out.push_back(static_cast<uint8_t>((value >> 24) & 0xFFu));
    };
  auto append_logical = [&append_u32_le](
    std::vector<uint8_t> & out, uint32_t key, bool enabled) {
      append_u32_le(out, key);
      out.push_back(enabled ? 0x01u : 0x00u);
    };
  auto append_u1 = [&append_u32_le](
    std::vector<uint8_t> & out, uint32_t key, uint8_t value) {
      append_u32_le(out, key);
      out.push_back(value);
    };
  auto append_u2 = [&append_u32_le](
    std::vector<uint8_t> & out, uint32_t key, uint16_t value) {
      append_u32_le(out, key);
      out.push_back(static_cast<uint8_t>(value & 0xFFu));
      out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFu));
    };

  // UBX-CFG-VALSET v0: version=0, layers bit0=RAM, reserved[2]=0.
  // Semua konfigurasi sengaja RAM-only agar reconnect aman dan tidak menulis
  // BBR/Flash receiver tanpa persetujuan operator.
  std::vector<uint8_t> payload{0x00, 0x01, 0x00, 0x00};

  append_logical(payload, 0x10730001u, true);  // CFG-UART1INPROT-UBX
  append_logical(payload, 0x10750001u, true);  // CFG-UART2INPROT-UBX
  append_logical(payload, 0x10740001u, true);  // CFG-UART1OUTPROT-UBX
  append_logical(payload, 0x10760001u, true);  // CFG-UART2OUTPROT-UBX
  append_logical(payload, 0x10780001u, true);  // CFG-USBOUTPROT-UBX

  // NAV-PVT satu kali setiap navigation epoch pada semua interface umum.
  append_u1(payload, 0x20910007u, 1u);  // CFG-MSGOUT-UBX_NAV_PVT_UART1
  append_u1(payload, 0x20910008u, 1u);  // CFG-MSGOUT-UBX_NAV_PVT_UART2
  append_u1(payload, 0x20910009u, 1u);  // CFG-MSGOUT-UBX_NAV_PVT_USB

  if (configure_dynamic_model_) {
    // u-blox M9 CFG-NAVSPG-DYNMODEL (0x20110021), E1.
    // 4 = AUTOMOT. RAM-only, so power-cycle restores receiver defaults.
    append_u1(payload, 0x20110021u, dynamic_model_);
  }

  if (configure_navigation_rate_) {
    const double clamped_hz = std::clamp(navigation_rate_hz_, 1.0, 25.0);
    const uint16_t meas_ms = static_cast<uint16_t>(
      std::clamp<long>(std::lround(1000.0 / clamped_hz), 40L, 1000L));
    append_u2(payload, 0x30210001u, meas_ms);  // CFG-RATE-MEAS [ms]
    append_u2(payload, 0x30210002u, 1u);       // CFG-RATE-NAV, one solution/measurement
  }

  return sendUbxMessage(0x06, 0x8A, payload);
}

// Fungsi: Poll NAV-COV/NAV-DOP secara terkontrol. Keduanya periodic/polled
// menurut protokol u-blox M9. Polling menghindari ketergantungan pada MSGOUT key
// interface tertentu dan hasil selalu dicocokkan berdasarkan iTOW.
void GnssNode::maybePollAuxNav()
{
  if (!ser_ || !ser_->isOpen() || !ubx_nav_pvt_seen_) return;
  const double now = nowSec();
  if (poll_nav_cov_) {
    const double period = 1.0 / std::max(0.2, nav_cov_poll_rate_hz_);
    if (last_cov_poll_time_ < 0.0 || now - last_cov_poll_time_ >= period) {
      if (sendUbxMessage(0x01, 0x36, {})) last_cov_poll_time_ = now;
    }
  }
  if (poll_nav_dop_) {
    const double period = 1.0 / std::max(0.1, nav_dop_poll_rate_hz_);
    if (last_dop_poll_time_ < 0.0 || now - last_dop_poll_time_ >= period) {
      if (sendUbxMessage(0x01, 0x04, {})) last_dop_poll_time_ = now;
    }
  }
}

// Fungsi: Menentukan kapan fallback NMEA GGA boleh digunakan untuk autonomous.
// Fallback hanya aktif bila NAV-PVT tidak muncul setelah waktu/attempt konfigurasi
// yang cukup DAN GGA yang sama membawa fix-quality, satelit, HDOP, dan koordinat
// yang ketat. Gate map/spread/drift di initializer tetap menjadi lapisan berikutnya.
bool GnssNode::nmeaFallbackEligible(double now) const
{
  if (!allow_validated_nmea_fallback_ || last_position_from_ubx_ || ubx_nav_pvt_seen_) {
    return false;
  }
  if (serial_connected_since_ < 0.0 || now - serial_connected_since_ < nmea_fallback_after_sec_) {
    return false;
  }
  if (last_nmea_gga_time_ < 0.0 || now - last_nmea_gga_time_ > 1.5) return false;
  // Autonomous fallback hanya menerima sentence yang benar-benar mempunyai
  // checksum '*XX'. Parser diagnostik masih boleh membaca kalimat tanpa checksum,
  // tetapi data seperti itu tidak boleh menentukan pose global kendaraan.
  if (!last_nmea_gga_checksum_present_) return false;
  if (last_nmea_fix_quality_ <= 0) return false;
  if (!sats_.has_value() || *sats_ < nmea_fallback_min_satellites_) return false;
  if (!quality_dop_.has_value() || !std::isfinite(*quality_dop_) ||
      *quality_dop_ <= 0.0 || *quality_dop_ > nmea_fallback_max_hdop_) {
    return false;
  }
  return true;
}

// Fungsi: Memberi alasan tekstual deterministik untuk setiap kegagalan quality gate.
// Alasan ini diterbitkan lewat /gnss/state sehingga diagnosis tidak lagi hanya
// melihat status DEGRADED tanpa mengetahui apakah sumbernya protocol, fix type, satelit,
// DOP, hAcc, atau sAcc.
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

// Fungsi: Mengecek apakah kualitas epoch posisi terakhir layak dipakai localization.
bool GnssNode::qualityGatePasses() const
{
  const std::string reason = qualityGateReason();
  return reason.rfind("ok-", 0) == 0;
}

// Fungsi: Mengutamakan NAV-PVT atomik bila receiver sedang mengeluarkan UBX.
bool GnssNode::recentUbxPvt(double now) const
{
  return prefer_ubx_nav_pvt_ && last_ubx_pvt_time_ >= 0.0 &&
         now - last_ubx_pvt_time_ <= ubx_preference_hold_sec_;
}

// Fungsi: Mengulang konfigurasi UBX secara terbatas bila NAV-PVT belum muncul.
void GnssNode::maybeRetryUbxConfig()
{
  if (!configure_ubx_nav_pvt_on_connect_ || ubx_nav_pvt_seen_ ||
      !ser_ || !ser_->isOpen()) {
    return;
  }
  if (ubx_config_attempts_ >= ubx_config_max_attempts_) return;
  const double now = nowSec();
  if (last_ubx_config_send_time_ >= 0.0 &&
      now - last_ubx_config_send_time_ < ubx_config_retry_sec_) {
    return;
  }
  if (configureUbxNavPvtOutput()) {
    ++ubx_config_attempts_;
    last_ubx_config_send_time_ = now;
    RCLCPP_INFO(this->get_logger(),
      "UBX NAV-PVT belum terlihat; retry CFG-VALSET #%d/%d dikirim",
      ubx_config_attempts_, ubx_config_max_attempts_);
  }
}

// Fungsi: Menerbitkan snapshot protocol/quality GNSS sebagai JSON dan log periodik.
void GnssNode::publishState()
{
  const double now = nowSec();
  const bool connected = ser_ && ser_->isOpen();
  const bool fallback_eligible = nmeaFallbackEligible(now);
  const std::string reason = qualityGateReason();
  const bool quality_ok = connected && reason.rfind("ok-", 0) == 0;
  const int source_id = last_position_from_ubx_ ? 1 : (fallback_eligible ? 3 : 2);
  const char * source_name = source_id == 1 ? "UBX_NAV_PVT" :
    (source_id == 3 ? "NMEA_GGA_VALIDATED_FALLBACK" : "NMEA_GGA");
  const double hacc_m = last_position_from_ubx_ ?
    static_cast<double>(ubx_hacc_mm_) / 1000.0 :
    (quality_dop_.has_value() && *quality_dop_ > 0.0 ? (*quality_dop_ * 2.5 / 1.1774) : 0.0);
  const double sacc_mps = last_position_from_ubx_ ?
    static_cast<double>(ubx_sacc_mmps_) / 1000.0 : -1.0;
  const bool receiver_fix_valid =
    fix_status_ != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  const bool usable = connected && receiver_fix_valid && sats_.value_or(0) >= 4 &&
    std::isfinite(hacc_m) && hacc_m > 0.0 && hacc_m <= 15.0;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(4)
     << "{\"connected\":" << (connected ? "true" : "false")
     << ",\"port\":\"" << active_port_ << "\""
     << ",\"baud\":" << baudrate_
     << ",\"source\":\"" << source_name << "\""
     << ",\"source_id\":" << source_id
     << ",\"receiver_fix_valid\":"
     << (receiver_fix_valid ? "true" : "false")
     << ",\"usable\":" << (usable ? "true" : "false")
     << ",\"autonomous_grade\":" << (quality_ok ? "true" : "false")
     << ",\"quality_ok\":" << (quality_ok ? "true" : "false")
     << ",\"quality_reason\":\"" << reason << "\""
     << ",\"ubx_nav_pvt_seen\":" << (ubx_nav_pvt_seen_ ? "true" : "false")
     << ",\"ubx_config_attempts\":" << ubx_config_attempts_
     << ",\"ubx_config_max_attempts\":" << ubx_config_max_attempts_
     << ",\"ubx_cfg_ack\":" << ubx_cfg_ack_count_
     << ",\"ubx_cfg_nak\":" << ubx_cfg_nak_count_
     << ",\"ubx_checksum_errors\":" << ubx_checksum_errors_
     << ",\"nmea_checksum_errors\":" << nmea_checksum_errors_
     << ",\"nmea_gga_count\":" << nmea_gga_count_
     << ",\"ubx_nav_pvt_count\":" << ubx_nav_pvt_count_
     << ",\"fallback_eligible\":" << (fallback_eligible ? "true" : "false")
     << ",\"gga_quality\":" << last_nmea_fix_quality_
     << ",\"gga_checksum_present\":" << (last_nmea_gga_checksum_present_ ? "true" : "false")
     << ",\"gnss_fix_ok\":" << (last_ubx_gnss_fix_ok_ ? "true" : "false")
     << ",\"data_age_sec\":" << ((last_data_time_ > 0.0) ? (now - last_data_time_) : -1.0)
     << ",\"source_age_sec\":" << (last_position_from_ubx_ ?
       ((last_ubx_pvt_time_ >= 0.0) ? (now - last_ubx_pvt_time_) : -1.0) :
       ((last_nmea_gga_time_ >= 0.0) ? (now - last_nmea_gga_time_) : -1.0))
     << ",\"satellites\":" << sats_.value_or(0)
     << ",\"dop\":" << quality_dop_.value_or(99.9)
     << ",\"hacc_m\":" << hacc_m
     << ",\"sacc_mps\":" << sacc_mps
     << ",\"ubx_fix_type\":" << static_cast<unsigned int>(last_ubx_fix_type_)
     << ",\"itow_ms\":" << last_itow_ms_
     << ",\"pvt_rate_hz\":" << pvt_rate_hz_ema_
     << ",\"timestamp_source\":\"" << last_timestamp_source_ << "\""
     << ",\"measurement_age_sec\":" << last_measurement_age_sec_
     << ",\"vAcc_m\":" << (static_cast<double>(ubx_vacc_mm_) / 1000.0)
     << ",\"utc_valid_flags\":" << static_cast<unsigned int>(utc_valid_flags_)
     << ",\"tAcc_ns\":" << utc_tacc_ns_
     << ",\"velE_mps\":" << vel_e_mps_.value_or(0.0)
     << ",\"velN_mps\":" << vel_n_mps_.value_or(0.0)
     << ",\"velD_mps\":" << vel_d_mps_.value_or(0.0)
     << ",\"nav_cov_pos_valid\":" << ((nav_cov_.valid && nav_cov_.pos_valid && sameEpoch(nav_cov_.itow_ms,last_itow_ms_)) ? "true" : "false")
     << ",\"nav_cov_vel_valid\":" << ((nav_cov_.valid && nav_cov_.vel_valid && sameEpoch(nav_cov_.itow_ms,last_itow_ms_)) ? "true" : "false")
     << ",\"nav_cov_count\":" << ubx_nav_cov_count_
     << ",\"nav_dop_count\":" << ubx_nav_dop_count_
     << ",\"nav_dop_itow_ms\":" << (nav_dop_.valid ? static_cast<double>(nav_dop_.itow_ms) : -1.0)
     << ",\"gdop\":" << (nav_dop_.valid ? nav_dop_.gdop : -1.0)
     << ",\"hdop\":" << (nav_dop_.valid ? nav_dop_.hdop : -1.0)
     << ",\"vdop\":" << (nav_dop_.valid ? nav_dop_.vdop : -1.0)
     << ",\"duplicate_itow\":" << ubx_duplicate_itow_count_
     << ",\"out_of_order_itow\":" << ubx_out_of_order_itow_count_
     << ",\"flags\":" << static_cast<unsigned int>(last_ubx_flags_)
     << ",\"flags2\":" << static_cast<unsigned int>(last_ubx_flags2_)
     << ",\"flags3\":" << static_cast<unsigned int>(last_ubx_flags3_)
     << ",\"invalid_llh\":" << (last_ubx_invalid_llh_ ? "true" : "false")
     << ",\"height_ellipsoid_m\":" << (static_cast<double>(ubx_height_mm_) / 1000.0)
     << ",\"head_vehicle_valid\":" << (head_vehicle_rad_.has_value() ? "true" : "false")
     << ",\"head_vehicle_rad\":" << head_vehicle_rad_.value_or(0.0)
     << ",\"mag_valid\":" << (mag_declination_rad_.has_value() ? "true" : "false")
     << ",\"mag_declination_rad\":" << mag_declination_rad_.value_or(0.0)
     << ",\"mag_accuracy_rad\":" << mag_declination_accuracy_rad_.value_or(0.0)
     << ",\"diff_solution\":" << (((last_ubx_flags_ & 0x02u) != 0u) ? "true" : "false")
     << ",\"carrier_solution\":" << static_cast<unsigned int>((last_ubx_flags_ >> 6) & 0x03u)
     << ",\"last_correction_age_code\":" << static_cast<unsigned int>((last_ubx_flags3_ >> 1) & 0x0Fu)
     << ",\"auth_time\":" << (((last_ubx_flags3_ & 0x2000u) != 0u) ? "true" : "false")
     << std::setprecision(7)
     << ",\"lat\":" << lat_.value_or(0.0)
     << ",\"lon\":" << lon_.value_or(0.0)
     << std::setprecision(4)
     << ",\"bytes_received\":" << bytes_received_
     << ",\"packets_parsed\":" << packets_parsed_
     << "}";

  std_msgs::msg::String msg;
  msg.data = ss.str();
  pub_state_->publish(msg);
  std_msgs::msg::Bool connected_msg;
  connected_msg.data = connected;
  pub_connected_->publish(connected_msg);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 10000,
    "GNSS_STATE connected=%s source=%s quality=%s reason=%s sat=%d DOP=%.2f hAcc=%.2f sAcc=%.2f cfg=%d/%d ACK=%zu NAK=%zu UBX_PVT=%zu GGA=%zu",
    connected ? "yes" : "no", source_name, quality_ok ? "PASS" : "DEGRADED",
    reason.c_str(), sats_.value_or(0), quality_dop_.value_or(99.9), hacc_m, sacc_mps,
    ubx_config_attempts_, ubx_config_max_attempts_, ubx_cfg_ack_count_, ubx_cfg_nak_count_,
    ubx_nav_pvt_count_, nmea_gga_count_);
}

// Fungsi: Menandai waktu fix valid terakhir untuk watchdog kualitas GNSS.
void GnssNode::markFix()
{
  fix_status_ = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

// Fungsi: Menandai state GNSS tanpa fix agar output tidak dianggap valid oleh localization.
void GnssNode::markNoFix()
{
  // Status dari receiver lebih terpercaya daripada koordinat terakhir yang
  // masih tersimpan. Jangan mengubah NO_FIX menjadi FIX hanya karena lat/lon ada.
  fix_status_ = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  last_velocity_time_ = 0.0;
}

// Fungsi: Membandingkan epoch UBX berdasarkan iTOW. Untuk Part 1 setiap
// NAV-COV/NAV-DOP harus cocok dengan NAV-PVT yang sedang dipublikasikan.
bool GnssNode::sameEpoch(uint32_t a, uint32_t b) const
{
  return a == b;
}

// Fungsi: Mengestimasi rate solusi dari delta iTOW, sekaligus mendeteksi
// duplicate/out-of-order epoch. Week rollover ditangani eksplisit.
void GnssNode::updatePvtRate(uint32_t itow_ms)
{
  constexpr int64_t week_ms = 604800000LL;
  if (!have_rate_itow_) {
    have_rate_itow_ = true;
    rate_last_itow_ms_ = itow_ms;
    return;
  }
  if (itow_ms == rate_last_itow_ms_) {
    ++ubx_duplicate_itow_count_;
    return;
  }
  int64_t delta = static_cast<int64_t>(itow_ms) - static_cast<int64_t>(rate_last_itow_ms_);
  if (delta < -week_ms / 2) delta += week_ms;  // GPS week rollover
  if (delta <= 0 || delta > 10000) {
    ++ubx_out_of_order_itow_count_;
    // Jangan menggeser reference rate ke epoch lama/outlier; epoch valid berikutnya
    // harus tetap dibandingkan terhadap last accepted iTOW.
    return;
  }
  const double hz = 1000.0 / static_cast<double>(delta);
  if (std::isfinite(hz) && hz > 0.0 && hz <= 30.0) {
    pvt_rate_hz_ema_ = pvt_rate_hz_ema_ <= 0.0 ? hz : (0.90 * pvt_rate_hz_ema_ + 0.10 * hz);
  }
  rate_last_itow_ms_ = itow_ms;
}

// Fungsi: Membuat timestamp pengukuran, bukan sekadar timestamp publish.
// AUTO memakai UTC NAV-PVT bila fully-resolved dan clock ROS numeriknya konsisten
// dengan Unix UTC. Jika use_sim_time aktif / host tidak sinkron, fallback ke
// timeline iTOW yang di-anchor pada arrival pertama. Fallback iTOW tidak pernah
// dibiarkan berada di masa depan terhadap arrival ROS agar tidak memicu future-TF.
rclcpp::Time GnssNode::makeMeasurementStamp()
{
  const rclcpp::Time arrival = this->now();
  const std::string mode = timestamp_mode_;
  const bool try_utc = mode == "auto" || mode == "utc";
  const bool valid_date = (utc_valid_flags_ & 0x01u) != 0u;
  const bool valid_time = (utc_valid_flags_ & 0x02u) != 0u;
  const bool fully_resolved = (utc_valid_flags_ & 0x04u) != 0u;

  if (try_utc && valid_date && valid_time && fully_resolved &&
      utc_year_ >= 2000 && utc_month_ >= 1 && utc_month_ <= 12 &&
      utc_day_ >= 1 && utc_day_ <= 31 && utc_hour_ <= 23 && utc_minute_ <= 59 &&
      utc_second_ <= 60) {
    std::tm tm{};
    tm.tm_year = static_cast<int>(utc_year_) - 1900;
    tm.tm_mon = static_cast<int>(utc_month_) - 1;
    tm.tm_mday = static_cast<int>(utc_day_);
    tm.tm_hour = static_cast<int>(utc_hour_);
    tm.tm_min = static_cast<int>(utc_minute_);
    // timegm tidak menerima leap second secara portable; gunakan 59 + 1 s.
    tm.tm_sec = std::min<int>(static_cast<int>(utc_second_), 59);
    const time_t sec = ::timegm(&tm);
    if (sec > 0) {
      int64_t total_ns = static_cast<int64_t>(sec) * 1000000000LL +
        static_cast<int64_t>(utc_nano_);
      if (utc_second_ == 60) total_ns += 1000000000LL;
      // Gunakan domain clock ROS yang sama dengan arrival. Header ROS hanya
      // membawa sec/nsec; perbandingan numerik memastikan use_sim_time tidak
      // salah dicampur dengan UTC receiver.
      const rclcpp::Time utc_stamp(total_ns, arrival.get_clock_type());
      const double signed_age = (arrival.nanoseconds() - utc_stamp.nanoseconds()) * 1e-9;
      if (std::isfinite(signed_age) && signed_age >= -0.02 &&
          std::abs(signed_age) <= utc_stamp_max_offset_sec_) {
        last_timestamp_source_ = "ubx_utc";
        last_measurement_age_sec_ = std::max(0.0, signed_age);
        return utc_stamp;
      }
    }
  }

  if (mode != "arrival") {
    if (!itow_anchor_valid_) {
      itow_anchor_valid_ = true;
      itow_anchor_ms_ = last_itow_ms_;
      itow_anchor_stamp_ = arrival;
    }
    constexpr int64_t week_ms = 604800000LL;
    int64_t delta_ms = static_cast<int64_t>(last_itow_ms_) - static_cast<int64_t>(itow_anchor_ms_);
    if (delta_ms < -week_ms / 2) delta_ms += week_ms;
    if (delta_ms > week_ms / 2) delta_ms -= week_ms;
    int64_t stamp_ns = itow_anchor_stamp_.nanoseconds() + delta_ms * 1000000LL;
    // USB/UART scheduling jitter dapat membuat predicted iTOW stamp sedikit
    // lebih baru dari arrival saat ini. Shift anchor ke belakang sehingga
    // timestamp sensor tidak pernah future-dated.
    if (stamp_ns > arrival.nanoseconds()) {
      const int64_t shift_ns = stamp_ns - arrival.nanoseconds();
      itow_anchor_stamp_ = rclcpp::Time(
        itow_anchor_stamp_.nanoseconds() - shift_ns, itow_anchor_stamp_.get_clock_type());
      stamp_ns = arrival.nanoseconds();
    }
    const rclcpp::Time stamp(stamp_ns, arrival.get_clock_type());
    last_timestamp_source_ = "itow_aligned";
    last_measurement_age_sec_ = std::max(
      0.0, (arrival.nanoseconds() - stamp.nanoseconds()) * 1e-9);
    return stamp;
  }

  last_timestamp_source_ = "arrival";
  last_measurement_age_sec_ = 0.0;
  return arrival;
}

// Fungsi: Menambahkan epoch posisi yang lolos gate ke window regresi.
void GnssNode::addPositionFitSample(const rclcpp::Time & stamp)
{
  if (!lat_.has_value() || !lon_.has_value() || !last_position_from_ubx_) return;
  FitSample s;
  s.t_sec = stamp.seconds();
  s.lat_rad = *lat_ * M_PI / 180.0;
  s.lon_rad = *lon_ * M_PI / 180.0;
  s.hacc_m = std::max(0.05, static_cast<double>(ubx_hacc_mm_) / 1000.0);
  if (!fit_samples_.empty() && s.t_sec <= fit_samples_.back().t_sec) {
    fit_samples_.clear();
  }
  fit_samples_.push_back(s);
  while (!fit_samples_.empty() && s.t_sec - fit_samples_.front().t_sec > position_fit_window_sec_) {
    fit_samples_.pop_front();
  }
}

// Fungsi: Weighted linear regression E(t),N(t) untuk estimator kecepatan
// berbasis perubahan posisi. Estimator ini diagnostic/validator, bukan input EKF
// pada Part 1 karena noise posisi M9 dapat jauh lebih besar dari perpindahan 5 Hz.
void GnssNode::publishPositionFit(const rclcpp::Time & stamp)
{
  if (static_cast<int>(fit_samples_.size()) < position_fit_min_samples_) return;
  constexpr double earth_r = 6378137.0;
  const double lat0 = fit_samples_.front().lat_rad;
  const double lon0 = fit_samples_.front().lon_rad;
  const double cos_lat0 = std::cos(lat0);

  double sw = 0.0, st = 0.0, se = 0.0, sn = 0.0;
  std::vector<std::array<double, 4>> pts;
  pts.reserve(fit_samples_.size());
  for (const auto & p : fit_samples_) {
    const double t = p.t_sec - fit_samples_.front().t_sec;
    const double e = earth_r * (p.lon_rad - lon0) * cos_lat0;
    const double n = earth_r * (p.lat_rad - lat0);
    const double sigma = std::max(0.05, p.hacc_m);
    const double w = 1.0 / (sigma * sigma);
    pts.push_back({t, e, n, w});
    sw += w; st += w * t; se += w * e; sn += w * n;
  }
  if (sw <= 0.0) return;
  const double mt = st / sw, me = se / sw, mn = sn / sw;
  double denom = 0.0, num_e = 0.0, num_n = 0.0;
  for (const auto & p : pts) {
    const double dt = p[0] - mt;
    denom += p[3] * dt * dt;
    num_e += p[3] * dt * (p[1] - me);
    num_n += p[3] * dt * (p[2] - mn);
  }
  if (denom <= 1e-9) return;
  const double ve = num_e / denom;
  const double vn = num_n / denom;
  const double first_e = pts.front()[1], first_n = pts.front()[2];
  const double last_e = pts.back()[1], last_n = pts.back()[2];
  const double baseline = std::hypot(last_e - first_e, last_n - first_n);
  double mean_hacc = 0.0;
  for (const auto & p : fit_samples_) mean_hacc += p.hacc_m;
  mean_hacc /= static_cast<double>(fit_samples_.size());
  const double required_baseline = std::max(
    position_fit_min_baseline_m_, position_fit_hacc_multiplier_ * mean_hacc);

  double sse = 0.0;
  for (const auto & p : pts) {
    const double pred_e = me + ve * (p[0] - mt);
    const double pred_n = mn + vn * (p[0] - mt);
    const double err2 = (p[1] - pred_e) * (p[1] - pred_e) + (p[2] - pred_n) * (p[2] - pred_n);
    sse += p[3] * err2;
  }
  const double weighted_rmse = std::sqrt(std::max(0.0, sse / sw));
  const bool fit_valid = baseline >= required_baseline && std::isfinite(ve) && std::isfinite(vn);

  if (fit_valid) {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = velocity_frame_id_;
    msg.twist.twist.linear.x = ve;
    msg.twist.twist.linear.y = vn;
    msg.twist.twist.linear.z = 0.0;
    const double duration = std::max(0.1, fit_samples_.back().t_sec - fit_samples_.front().t_sec);
    const double vel_var = std::max(0.01, (weighted_rmse / duration) * (weighted_rmse / duration));
    msg.twist.covariance[0] = vel_var;
    msg.twist.covariance[7] = vel_var;
    msg.twist.covariance[14] = 1e9;
    msg.twist.covariance[21] = 1e9;
    msg.twist.covariance[28] = 1e9;
    msg.twist.covariance[35] = 1e9;
    pub_vel_fit_->publish(msg);
  }

  const double fit_speed = std::hypot(ve, vn);
  const double fit_course = std::atan2(vn, ve);  // ROS ENU yaw, 0=east CCW
  const double cog_enu = heading_rad_.has_value() ?
    std::atan2(std::sin((0.5 * M_PI) - *heading_rad_), std::cos((0.5 * M_PI) - *heading_rad_)) :
    std::numeric_limits<double>::quiet_NaN();
  const double speed_residual = speed_mps_.has_value() ? fit_speed - *speed_mps_ :
    std::numeric_limits<double>::quiet_NaN();
  const double course_residual = std::isfinite(cog_enu) ?
    std::atan2(std::sin(fit_course - cog_enu), std::cos(fit_course - cog_enu)) :
    std::numeric_limits<double>::quiet_NaN();

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(5)
     << "{\"fit_valid\":" << (fit_valid ? "true" : "false")
     << ",\"samples\":" << fit_samples_.size()
     << ",\"window_sec\":" << (fit_samples_.back().t_sec - fit_samples_.front().t_sec)
     << ",\"baseline_m\":" << baseline
     << ",\"required_baseline_m\":" << required_baseline
     << ",\"fit_rmse_m\":" << weighted_rmse
     << ",\"vel_e_fit_mps\":" << ve
     << ",\"vel_n_fit_mps\":" << vn
     << ",\"speed_fit_mps\":" << fit_speed
     << ",\"course_fit_enu_rad\":" << fit_course
     << ",\"speed_doppler_mps\":" << speed_mps_.value_or(-1.0)
     << ",\"speed_fit_minus_doppler_mps\":" << speed_residual
     << ",\"course_fit_minus_cog_rad\":" << course_residual
     << "}";
  std_msgs::msg::String diag;
  diag.data = ss.str();
  pub_motion_diag_->publish(diag);
}

// Fungsi: Menerbitkan NavSatFix/velocity dari SATU epoch posisi yang koheren.
// Posisi buruk tetap boleh dipublikasikan sebagai STATUS_NO_FIX untuk diagnosis,
// tetapi tidak boleh masuk ke georeference/EKF sebagai pengukuran valid.
void GnssNode::publish()
{
  if (!hasValidPosition()) return;

  const bool quality_good = qualityGatePasses();
  const bool receiver_fix_good = fix_status_ != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  const bool fix_good = receiver_fix_good && quality_good;
  const rclcpp::Time stamp = last_position_from_ubx_ ? makeMeasurementStamp() : this->now();
  if (!last_position_from_ubx_) {
    last_timestamp_source_ = "arrival_nmea";
    last_measurement_age_sec_ = 0.0;
  }

  sensor_msgs::msg::NavSatFix fix;
  fix.header.stamp = stamp;
  fix.header.frame_id = frame_id_;
  fix.status.status = fix_good ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                               : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
    sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
    sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO |
    sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
  fix.latitude = lat_.value_or(0.0);
  fix.longitude = lon_.value_or(0.0);
  fix.altitude = alt_.value_or(0.0);

  const bool cov_epoch_match = last_position_from_ubx_ && nav_cov_.valid &&
    sameEpoch(nav_cov_.itow_ms, last_itow_ms_);
  if (cov_epoch_match && nav_cov_.pos_valid) {
    // UBX NAV-COV = NED. sensor_msgs/NavSatFix covariance = ENU.
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
  } else if (!last_position_from_ubx_ && quality_dop_.has_value() &&
             *quality_dop_ > 0.0 && *quality_dop_ < 50.0) {
    const double cep_m = *quality_dop_ * 2.5;
    const double sigma_h = cep_m / 1.1774;
    const double h_var = sigma_h * sigma_h;
    fix.position_covariance[0] = h_var;
    fix.position_covariance[4] = h_var;
    fix.position_covariance[8] = std::max(4.0 * h_var, 1.0);
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
  } else {
    fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }

  sensor_msgs::msg::NavSatFix raw_fix = fix;
  const bool raw_horizontal_fix_good = last_position_from_ubx_ ?
    (last_ubx_fix_type_ >= 2u && !last_ubx_invalid_llh_) : receiver_fix_good;
  // /gnss/fix_raw is diagnostic/display authority, not motion authority. A valid
  // UBX 2D horizontal solution may therefore carry STATUS_FIX so map x/y can be
  // displayed while strict fusion still checks gnssFixOK + 3D + accuracy gates.
  raw_fix.status.status = raw_horizontal_fix_good ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                                   : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  pub_fix_raw_->publish(raw_fix);
  pub_fix_->publish(fix);

  const bool speed_quality_good = !last_position_from_ubx_ ||
    (ubx_sacc_mmps_ > 0 && static_cast<double>(ubx_sacc_mmps_) / 1000.0 <= max_sacc_mps_);
  (void)speed_quality_good;
  // Publish raw Doppler velocity whenever the receiver epoch is fresh. This keeps
  // HUD/diagnostics alive during convergence; LocalizationCore owns the separate
  // quality gate before /gnss/base_velocity_fusion reaches robot_localization.
  if (last_velocity_time_ > 0.0 &&
      nowSec() - last_velocity_time_ <= velocity_stale_timeout_) {
    geometry_msgs::msg::TwistWithCovarianceStamped twist;
    twist.header.stamp = stamp;
    twist.header.frame_id = velocity_frame_id_;
    if (last_position_from_ubx_ && vel_e_mps_.has_value() && vel_n_mps_.has_value() && vel_d_mps_.has_value()) {
      // NAV-PVT already provides N/E/D Doppler velocity. Use it directly rather
      // than reconstructing components from gSpeed/headMot.
      twist.twist.twist.linear.x = *vel_e_mps_;
      twist.twist.twist.linear.y = *vel_n_mps_;
      twist.twist.twist.linear.z = -*vel_d_mps_;  // NED Down -> ENU Up
    } else if (speed_mps_.has_value() && heading_rad_.has_value()) {
      const double course_from_north_clockwise = *heading_rad_;
      twist.twist.twist.linear.x = *speed_mps_ * std::sin(course_from_north_clockwise);
      twist.twist.twist.linear.y = *speed_mps_ * std::cos(course_from_north_clockwise);
    }

    if (cov_epoch_match && nav_cov_.vel_valid) {
      twist.twist.covariance[0] = nav_cov_.vel_ee;
      twist.twist.covariance[1] = nav_cov_.vel_ne;
      twist.twist.covariance[2] = -nav_cov_.vel_ed;
      twist.twist.covariance[6] = nav_cov_.vel_ne;
      twist.twist.covariance[7] = nav_cov_.vel_nn;
      twist.twist.covariance[8] = -nav_cov_.vel_nd;
      twist.twist.covariance[12] = -nav_cov_.vel_ed;
      twist.twist.covariance[13] = -nav_cov_.vel_nd;
      twist.twist.covariance[14] = nav_cov_.vel_dd;
    } else {
      double vel_var = 1.0;
      if (last_position_from_ubx_ && ubx_sacc_mmps_ > 0) {
        const double sigma_v = static_cast<double>(ubx_sacc_mmps_) / 1000.0;
        vel_var = std::max(0.01, sigma_v * sigma_v);
      }
      twist.twist.covariance[0] = vel_var;
      twist.twist.covariance[7] = vel_var;
      twist.twist.covariance[14] = vel_var;
    }
    twist.twist.covariance[21] = 1e9;
    twist.twist.covariance[28] = 1e9;
    twist.twist.covariance[35] = 1e9;
    pub_vel_->publish(twist);
  }

  if (fix_good && last_position_from_ubx_) {
    addPositionFitSample(stamp);
    publishPositionFit(stamp);
  }

  const bool fallback_eligible = nmeaFallbackEligible(nowSec());
  const double nmea_sigma_h = (!last_position_from_ubx_ && quality_dop_.has_value() &&
      *quality_dop_ > 0.0) ? (*quality_dop_ * 2.5 / 1.1774) : 0.0;
  const int source_id = last_position_from_ubx_ ? 1 : (fallback_eligible ? 3 : 2);
  const bool dop_epoch_match = nav_dop_.valid && last_position_from_ubx_ &&
    sameEpoch(nav_dop_.itow_ms, last_itow_ms_);
  auto itow_delta_abs_ms = [](uint32_t a, uint32_t b) {
    constexpr int64_t week_ms = 604800000LL;
    int64_t d = static_cast<int64_t>(a) - static_cast<int64_t>(b);
    if (d < -week_ms / 2) d += week_ms;
    if (d > week_ms / 2) d -= week_ms;
    return std::llabs(d);
  };
  const bool dop_recent = nav_dop_.valid && last_position_from_ubx_ &&
    itow_delta_abs_ms(nav_dop_.itow_ms, last_itow_ms_) <= 1500;

  std_msgs::msg::Float64MultiArray quality;
  // Append-only layout. Indeks 0..8 dipertahankan untuk compatibility Part 2/3.
  //  0 sat, 1 pDOP/HDOP fallback, 2 hAcc, 3 fixType, 4 source, 5 sAcc,
  //  6 gSpeed, 7 COG ENU, 8 headAcc,
  //  9 iTOW ms, 10 vAcc, 11 velE, 12 velN, 13 velD(Down),
  // 14 gDOP, 15 hDOP, 16 vDOP, 17 nDOP, 18 eDOP, 19 tDOP,
  // 20 NAV-COV pos valid, 21 NAV-COV vel valid, 22 actual PVT rate Hz,
  // 23 measurement age sec, 24 timestamp source code (0 arrival,1 iTOW,2 UTC),
  // 25 flags2, 26 flags3, 27 NAV-DOP iTOW, 28 DOP exact-epoch flag,
  // 29 NAV-COV iTOW, 30 COV exact-epoch flag, 31 UTC valid flags, 32 tAcc ns,
  // 33 ellipsoid height, 34 headVeh ENU (valid only), 35 magDec, 36 magAcc,
  // 37 diffSoln, 38 carrSoln, 39 invalidLlh, 40 lastCorrectionAge code,
  // 41 authTime, 42 headVehValid, 43 validMag, 44 gnssFixOK.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const int stamp_source_code = last_timestamp_source_ == "ubx_utc" ? 2 :
    (last_timestamp_source_ == "itow_aligned" ? 1 : 0);
  quality.data = {
    sats_.has_value() ? static_cast<double>(*sats_) : 0.0,
    quality_dop_.has_value() ? *quality_dop_ : 99.9,
    last_position_from_ubx_ ? static_cast<double>(ubx_hacc_mm_) / 1000.0 : nmea_sigma_h,
    last_position_from_ubx_ ? static_cast<double>(last_ubx_fix_type_) : static_cast<double>(last_nmea_fix_quality_),
    static_cast<double>(source_id),
    last_position_from_ubx_ ? static_cast<double>(ubx_sacc_mmps_) / 1000.0 : -1.0,
    (last_position_from_ubx_ && speed_mps_.has_value()) ? *speed_mps_ : -1.0,
    (last_position_from_ubx_ && heading_rad_.has_value()) ?
      std::atan2(std::sin((0.5*M_PI)-*heading_rad_), std::cos((0.5*M_PI)-*heading_rad_)) : nan,
    (last_position_from_ubx_ && heading_accuracy_rad_.has_value()) ? *heading_accuracy_rad_ : nan,
    last_position_from_ubx_ ? static_cast<double>(last_itow_ms_) : nan,
    last_position_from_ubx_ ? static_cast<double>(ubx_vacc_mm_) / 1000.0 : nan,
    (last_position_from_ubx_ && vel_e_mps_.has_value()) ? *vel_e_mps_ : nan,
    (last_position_from_ubx_ && vel_n_mps_.has_value()) ? *vel_n_mps_ : nan,
    (last_position_from_ubx_ && vel_d_mps_.has_value()) ? *vel_d_mps_ : nan,
    dop_recent ? nav_dop_.gdop : nan,
    dop_recent ? nav_dop_.hdop : nan,
    dop_recent ? nav_dop_.vdop : nan,
    dop_recent ? nav_dop_.ndop : nan,
    dop_recent ? nav_dop_.edop : nan,
    dop_recent ? nav_dop_.tdop : nan,
    (cov_epoch_match && nav_cov_.pos_valid) ? 1.0 : 0.0,
    (cov_epoch_match && nav_cov_.vel_valid) ? 1.0 : 0.0,
    pvt_rate_hz_ema_,
    last_measurement_age_sec_,
    static_cast<double>(stamp_source_code),
    static_cast<double>(last_ubx_flags2_),
    static_cast<double>(last_ubx_flags3_),
    nav_dop_.valid ? static_cast<double>(nav_dop_.itow_ms) : nan,
    dop_epoch_match ? 1.0 : 0.0,
    nav_cov_.valid ? static_cast<double>(nav_cov_.itow_ms) : nan,
    cov_epoch_match ? 1.0 : 0.0,
    static_cast<double>(utc_valid_flags_),
    static_cast<double>(utc_tacc_ns_),
    last_position_from_ubx_ ? static_cast<double>(ubx_height_mm_) / 1000.0 : nan,
    (last_position_from_ubx_ && head_vehicle_rad_.has_value()) ?
      std::atan2(std::sin((0.5*M_PI)-*head_vehicle_rad_), std::cos((0.5*M_PI)-*head_vehicle_rad_)) : nan,
    (last_position_from_ubx_ && mag_declination_rad_.has_value()) ? *mag_declination_rad_ : nan,
    (last_position_from_ubx_ && mag_declination_accuracy_rad_.has_value()) ? *mag_declination_accuracy_rad_ : nan,
    last_position_from_ubx_ ? (((last_ubx_flags_ & 0x02u) != 0u) ? 1.0 : 0.0) : nan,
    last_position_from_ubx_ ? static_cast<double>((last_ubx_flags_ >> 6) & 0x03u) : nan,
    last_position_from_ubx_ ? (last_ubx_invalid_llh_ ? 1.0 : 0.0) : nan,
    last_position_from_ubx_ ? static_cast<double>((last_ubx_flags3_ >> 1) & 0x0Fu) : nan,
    last_position_from_ubx_ ? (((last_ubx_flags3_ & 0x2000u) != 0u) ? 1.0 : 0.0) : nan,
    last_position_from_ubx_ ? (head_vehicle_rad_.has_value() ? 1.0 : 0.0) : nan,
    last_position_from_ubx_ ? (mag_declination_rad_.has_value() ? 1.0 : 0.0) : nan,
    last_position_from_ubx_ ? (last_ubx_gnss_fix_ok_ ? 1.0 : 0.0) : nan
  };
  pub_quality_->publish(quality);

  const std::string quality_reason = qualityGateReason();
  const double quality_hacc_m = last_position_from_ubx_ ?
    static_cast<double>(ubx_hacc_mm_) / 1000.0 : nmea_sigma_h;
  if (!fix_good) {
    if (quality_state_initialized_ && last_quality_good_) {
      RCLCPP_WARN(this->get_logger(),
        "GNSS localization QUALITY_LOST: reason=%s receiver_fix=%s sat=%d DOP=%.2f hAcc=%.2f m",
        quality_reason.c_str(), receiver_fix_good ? "yes" : "no", sats_.value_or(0),
        quality_dop_.value_or(99.9), quality_hacc_m);
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 15000,
        "GNSS localization QUALITY_DEGRADED: reason=%s source=%d fix=%u sat=%d DOP=%.2f hAcc=%.2f m sAcc=%.2f m/s",
        quality_reason.c_str(), source_id,
        static_cast<unsigned int>(last_position_from_ubx_ ? last_ubx_fix_type_ :
          static_cast<uint8_t>(std::max(0, last_nmea_fix_quality_))),
        sats_.value_or(0), quality_dop_.value_or(99.9), quality_hacc_m,
        last_position_from_ubx_ ? static_cast<double>(ubx_sacc_mmps_) / 1000.0 : -1.0);
    }
  } else if (!quality_state_initialized_ || !last_quality_good_) {
    RCLCPP_INFO(this->get_logger(),
      "GNSS localization QUALITY_READY: source=%s sat=%d DOP=%.2f hAcc=%.2f m rate=%.2fHz stamp=%s",
      source_id == 1 ? "UBX-NAV-PVT" : "NMEA-VALIDATED-FALLBACK", sats_.value_or(0),
      quality_dop_.value_or(99.9), quality_hacc_m, pvt_rate_hz_ema_, last_timestamp_source_.c_str());
  }
  quality_state_initialized_ = true;
  last_quality_good_ = fix_good;
}

// Fungsi: Mendekode NMEA tanpa mencampur epoch/protokol kualitas.
// GGA menjadi sumber posisi NMEA karena coordinate, fix quality, satellite count,
// dan HDOP berada pada kalimat yang sama. RMC dipakai untuk speed/course saja
// kecuali fallback posisi diaktifkan eksplisit.
bool GnssNode::handleNmea(const std::string & sentence)
{
  if (sentence.size() < 7) return false;
  if (!isValidNmeaChecksum(sentence)) {
    ++nmea_checksum_errors_;
    return false;
  }
  const bool checksum_present = sentence.find('*') != std::string::npos;
  const std::string type = sentence.substr(3, 3);
  const auto fields = splitNmea(sentence);
  const double now = nowSec();

  // Bila NAV-PVT aktif, jangan biarkan NMEA yang datang beberapa milidetik
  // kemudian menimpa koordinat UBX tetapi tetap membawa hAcc UBX lama.
  if (recentUbxPvt(now)) return false;

  if (type == "GGA") {
    if (fields.size() < 11) return false;
    ++nmea_gga_count_;
    last_nmea_gga_checksum_present_ = checksum_present;

    int qual = -1;
    int sats = -1;
    double hdop = -1.0;
    try { qual = std::stoi(fields[6]); } catch (...) {}
    try { sats = std::stoi(fields[7]); } catch (...) {}
    try { hdop = std::stod(fields[8]); } catch (...) {}

    last_nmea_fix_quality_ = std::max(0, qual);
    if (sats >= 0) sats_ = sats;
    if (std::isfinite(hdop) && hdop > 0.0) quality_dop_ = hdop;
    else quality_dop_.reset();

    double latitude = 0.0, longitude = 0.0;
    const bool coordinates_ok =
      parseNmeaCoordinate(fields[2], fields[3], true, latitude) &&
      parseNmeaCoordinate(fields[4], fields[5], false, longitude);

    if (qual > 0 && coordinates_ok) {
      lat_ = latitude;
      lon_ = longitude;
      double alt = -1.0;
      try { alt = std::stod(fields[9]); } catch (...) {}
      if (std::isfinite(alt) && alt > -1000.0 && alt < 100000.0) alt_ = alt;

      last_position_from_ubx_ = false;
      last_ubx_gnss_fix_ok_ = false;
      last_ubx_fix_type_ = 0;
      ubx_hacc_mm_ = 0;
      ubx_sacc_mmps_ = 0;
      last_nmea_gga_time_ = now;
      nmea_fallback_active_ = nmeaFallbackEligible(now);
      markFix();
    } else {
      nmea_fallback_active_ = false;
      markNoFix();
    }
    return true;
  }

  if (type == "RMC") {
    if (fields.size() < 10) return false;
    const bool valid_flag = fields[2] == "A";

    double speed_knots = -1.0;
    double course_degrees = -1.0;
    try { speed_knots = std::stod(fields[7]); } catch (...) {}
    try { course_degrees = std::stod(fields[8]); } catch (...) {}
    if (valid_flag && std::isfinite(speed_knots) && speed_knots >= 0.0 &&
        std::isfinite(course_degrees) && course_degrees >= 0.0 &&
        course_degrees < 360.0) {
      speed_mps_ = speed_knots * 0.514444;
      heading_rad_ = course_degrees * M_PI / 180.0;
      last_velocity_time_ = now;
    }

    if (!accept_rmc_position_fallback_) return false;

    // Fallback hanya boleh memakai quality GGA yang masih fresh. RMC sendiri
    // tidak membawa satellite count/HDOP sehingga tidak cukup untuk autonomous gate.
    const bool recent_gga = last_nmea_gga_time_ >= 0.0 && now - last_nmea_gga_time_ <= 1.5;
    double latitude = 0.0, longitude = 0.0;
    const bool coordinates_ok =
      parseNmeaCoordinate(fields[3], fields[4], true, latitude) &&
      parseNmeaCoordinate(fields[5], fields[6], false, longitude);
    if (valid_flag && recent_gga && coordinates_ok && qualityGatePasses()) {
      lat_ = latitude;
      lon_ = longitude;
      last_position_from_ubx_ = false;
      last_ubx_gnss_fix_ok_ = false;
      ubx_hacc_mm_ = 0;
      ubx_sacc_mmps_ = 0;
      markFix();
      return true;
    }
    return false;
  }

  return false;
}

// Fungsi: Mendekode UBX NAV-PVT sebagai satu epoch atomik posisi+kualitas.
bool GnssNode::handleUbx(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t> & payload)
{
  // UBX-ACK-ACK / UBX-ACK-NAK untuk CFG-VALSET (06 8A).
  if (msg_class == 0x05 && (msg_id == 0x01 || msg_id == 0x00) && payload.size() >= 2) {
    if (payload[0] == 0x06 && payload[1] == 0x8A) {
      if (msg_id == 0x01) {
        ++ubx_cfg_ack_count_;
        RCLCPP_INFO(this->get_logger(),
          "u-blox ACK-ACK diterima untuk CFG-VALSET NAV-PVT/rate (attempt=%d)",
          ubx_config_attempts_);
      } else {
        ++ubx_cfg_nak_count_;
        RCLCPP_WARN(this->get_logger(),
          "u-blox ACK-NAK untuk CFG-VALSET; receiver tetap dibaca dengan gate fail-safe");
      }
    }
    return false;
  }

  // UBX-NAV-COV (01 36): covariance NED, dicocokkan berdasarkan iTOW.
  if (msg_class == 0x01 && msg_id == 0x36) {
    if (payload.size() < 64) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "UBX NAV-COV too short (%zu, need 64)", payload.size());
      return false;
    }
    ++ubx_nav_cov_count_;
    NavCovEpoch cov;
    std::memcpy(&cov.itow_ms, &payload[0], 4);
    cov.pos_valid = payload[5] != 0;
    cov.vel_valid = payload[6] != 0;
    float f[12]{};
    for (size_t i = 0; i < 12; ++i) std::memcpy(&f[i], &payload[16 + 4 * i], 4);
    cov.pos_nn=f[0]; cov.pos_ne=f[1]; cov.pos_nd=f[2]; cov.pos_ee=f[3]; cov.pos_ed=f[4]; cov.pos_dd=f[5];
    cov.vel_nn=f[6]; cov.vel_ne=f[7]; cov.vel_nd=f[8]; cov.vel_ee=f[9]; cov.vel_ed=f[10]; cov.vel_dd=f[11];
    const auto finite6 = [](double a,double b,double c,double d,double e,double f0) {
      return std::isfinite(a)&&std::isfinite(b)&&std::isfinite(c)&&std::isfinite(d)&&std::isfinite(e)&&std::isfinite(f0);
    };
    cov.pos_valid = cov.pos_valid && finite6(cov.pos_nn,cov.pos_ne,cov.pos_nd,cov.pos_ee,cov.pos_ed,cov.pos_dd) &&
      cov.pos_nn >= 0.0 && cov.pos_ee >= 0.0 && cov.pos_dd >= 0.0;
    cov.vel_valid = cov.vel_valid && finite6(cov.vel_nn,cov.vel_ne,cov.vel_nd,cov.vel_ee,cov.vel_ed,cov.vel_dd) &&
      cov.vel_nn >= 0.0 && cov.vel_ee >= 0.0 && cov.vel_dd >= 0.0;
    cov.valid = true;
    nav_cov_ = cov;
    // Jika PVT ditahan menunggu covariance dan epoch cocok, publish sekarang.
    if (pending_ubx_publish_ && sameEpoch(nav_cov_.itow_ms, last_itow_ms_)) {
      pending_ubx_publish_ = false;
      return true;
    }
    return false;
  }

  // UBX-NAV-DOP (01 04): DOP lengkap untuk diagnostic/reporting.
  if (msg_class == 0x01 && msg_id == 0x04) {
    if (payload.size() < 18) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "UBX NAV-DOP too short (%zu, need 18)", payload.size());
      return false;
    }
    ++ubx_nav_dop_count_;
    NavDopEpoch dop;
    uint16_t raw[7]{};
    std::memcpy(&dop.itow_ms, &payload[0], 4);
    for (size_t i=0;i<7;++i) std::memcpy(&raw[i], &payload[4+2*i], 2);
    dop.gdop=raw[0]*0.01; dop.pdop=raw[1]*0.01; dop.tdop=raw[2]*0.01;
    dop.vdop=raw[3]*0.01; dop.hdop=raw[4]*0.01; dop.ndop=raw[5]*0.01; dop.edop=raw[6]*0.01;
    dop.valid = true;
    nav_dop_ = dop;
    return false;
  }

  if (msg_class != 0x01 || msg_id != 0x07) return false;
  ++ubx_nav_pvt_count_;
  nmea_fallback_active_ = false;
  if (!ubx_nav_pvt_seen_) {
    ubx_nav_pvt_seen_ = true;
    RCLCPP_INFO(this->get_logger(),
      "UBX NAV-PVT terdeteksi; driver V2 memakai iTOW, velN/E/D dan quality epoch atomik");
  }
  if (payload.size() < 92) {
    RCLCPP_WARN(this->get_logger(), "UBX NAV-PVT too short (%zu bytes, need 92)", payload.size());
    return false;
  }

  uint32_t itow_ms = 0, tacc_ns = 0;
  uint16_t year = 0, p_dop_raw = 0, flags3 = 0;
  int32_t nano = 0;
  std::memcpy(&itow_ms, &payload[0], 4);
  std::memcpy(&year, &payload[4], 2);
  std::memcpy(&tacc_ns, &payload[12], 4);
  std::memcpy(&nano, &payload[16], 4);
  std::memcpy(&p_dop_raw, &payload[76], 2);
  std::memcpy(&flags3, &payload[78], 2);

  const uint8_t month=payload[6], day=payload[7], hour=payload[8], minute=payload[9], second=payload[10];
  const uint8_t valid_flags=payload[11];
  const uint8_t fix_type=payload[20], flags=payload[21], flags2=payload[22], num_sv=payload[23];
  const bool gnss_fix_ok = (flags & 0x01u) != 0u;

  int32_t lon_raw=0, lat_raw=0, height_raw=0, h_msl_raw=0;
  int32_t vel_n_raw=0, vel_e_raw=0, vel_d_raw=0, g_speed_raw=0, head_mot_raw=0, head_veh_raw=0;
  int16_t mag_dec_raw=0;
  uint16_t mag_acc_raw=0;
  uint32_t h_acc_mm=0, v_acc_mm=0, s_acc_mmps=0, head_acc_raw=0;
  std::memcpy(&lon_raw,&payload[24],4); std::memcpy(&lat_raw,&payload[28],4);
  std::memcpy(&height_raw,&payload[32],4); std::memcpy(&h_msl_raw,&payload[36],4);
  std::memcpy(&h_acc_mm,&payload[40],4); std::memcpy(&v_acc_mm,&payload[44],4);
  std::memcpy(&vel_n_raw,&payload[48],4); std::memcpy(&vel_e_raw,&payload[52],4);
  std::memcpy(&vel_d_raw,&payload[56],4); std::memcpy(&g_speed_raw,&payload[60],4);
  std::memcpy(&head_mot_raw,&payload[64],4); std::memcpy(&s_acc_mmps,&payload[68],4);
  std::memcpy(&head_acc_raw,&payload[72],4); std::memcpy(&head_veh_raw,&payload[84],4);
  std::memcpy(&mag_dec_raw,&payload[88],2); std::memcpy(&mag_acc_raw,&payload[90],2);
  const bool invalid_llh = (flags3 & 0x0001u) != 0u;
  const bool head_veh_valid = (flags & 0x20u) != 0u;
  const bool valid_mag = (valid_flags & 0x08u) != 0u;
  ubx_height_mm_ = height_raw;
  head_vehicle_rad_ = head_veh_valid ? std::optional<double>(
    (static_cast<double>(head_veh_raw) * 1e-5) * M_PI / 180.0) : std::nullopt;
  mag_declination_rad_ = valid_mag ? std::optional<double>(
    (static_cast<double>(mag_dec_raw) * 1e-2) * M_PI / 180.0) : std::nullopt;
  mag_declination_accuracy_rad_ = valid_mag ? std::optional<double>(
    (static_cast<double>(mag_acc_raw) * 1e-2) * M_PI / 180.0) : std::nullopt;

  // Duplicate iTOW tidak diterbitkan ulang. Out-of-order tetap tercatat dan
  // diparse agar diagnostic receiver tidak hilang, tetapi estimator rate akan menandainya.
  const bool duplicate_itow = have_rate_itow_ && itow_ms == rate_last_itow_ms_;
  bool out_of_order_itow = false;
  if (have_rate_itow_ && !duplicate_itow) {
    constexpr int64_t week_ms = 604800000LL;
    int64_t d = static_cast<int64_t>(itow_ms) - static_cast<int64_t>(rate_last_itow_ms_);
    if (d < -week_ms / 2) d += week_ms;
    out_of_order_itow = d <= 0;
  }
  updatePvtRate(itow_ms);
  if (duplicate_itow || out_of_order_itow) return false;

  const double longitude=lon_raw*1e-7, latitude=lat_raw*1e-7;
  const bool coordinates_ok=std::isfinite(latitude)&&std::isfinite(longitude)&&
    std::abs(latitude)<=90.0&&std::abs(longitude)<=180.0&&
    !(std::abs(latitude)<1e-12&&std::abs(longitude)<1e-12);

  last_ubx_pvt_time_=nowSec();
  last_position_from_ubx_=true;
  last_ubx_gnss_fix_ok_=gnss_fix_ok;
  last_ubx_fix_type_=fix_type;
  last_ubx_flags_=flags;
  last_ubx_flags2_=flags2;
  last_ubx_flags3_=flags3;
  last_ubx_invalid_llh_=invalid_llh;
  last_itow_ms_=itow_ms;
  utc_year_=year; utc_month_=month; utc_day_=day; utc_hour_=hour; utc_minute_=minute; utc_second_=second;
  utc_valid_flags_=valid_flags; utc_tacc_ns_=tacc_ns; utc_nano_=nano;
  sats_=static_cast<int>(num_sv);
  quality_dop_=static_cast<double>(p_dop_raw)*0.01;
  ubx_hacc_mm_=h_acc_mm; ubx_vacc_mm_=v_acc_mm; ubx_sacc_mmps_=s_acc_mmps;

  if (!coordinates_ok) {
    markNoFix();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "UBX NAV-PVT koordinat invalid lat=%.7f lon=%.7f", latitude, longitude);
    pending_ubx_publish_ = false;
    return true;
  }

  lat_=latitude; lon_=longitude; alt_=static_cast<double>(h_msl_raw)/1000.0;
  vel_n_mps_=static_cast<double>(vel_n_raw)/1000.0;
  vel_e_mps_=static_cast<double>(vel_e_raw)/1000.0;
  vel_d_mps_=static_cast<double>(vel_d_raw)/1000.0;
  speed_mps_=static_cast<double>(g_speed_raw)/1000.0;
  heading_rad_=(static_cast<double>(head_mot_raw)*1e-5)*M_PI/180.0;
  // courseAccuracyRad compatibility semantic: headAcc NAV-PVT converted to radians.
  heading_accuracy_rad_=(static_cast<double>(head_acc_raw)*1e-5)*M_PI/180.0;
  last_velocity_time_=last_ubx_pvt_time_;

  const bool position_fix_type_ok=fix_type==3u||(accept_ubx_gnss_dr_fix_&&fix_type==4u);
  const bool receiver_valid=gnss_fix_ok&&position_fix_type_ok&&!invalid_llh;
  if (receiver_valid) markFix(); else markNoFix();

  // Jika NAV-COV aktif, tahan PVT sangat singkat agar covariance dengan iTOW
  // yang sama dapat ikut pada publish pertama. Timeout di pollSerial mencegah
  // kehilangan data jika receiver tidak merespons poll NAV-COV.
  pending_ubx_publish_ = poll_nav_cov_ && cov_wait_timeout_sec_ > 0.0 &&
    !(nav_cov_.valid && sameEpoch(nav_cov_.itow_ms, last_itow_ms_));
  pending_ubx_since_ = nowSec();
  return !pending_ubx_publish_;
}

// Fungsi: Membaca byte serial GNSS secara periodik serta menangani timeout dan hot-plug reconnect.
void GnssNode::pollSerial()
{
  if (!ser_ || !ser_->isOpen()) {
    double now = nowSec();
    if (now - last_reconnect_try_ > reconnect_interval_sec_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "GNSS serial closed — attempting reconnect");
      openSerial(false);
      last_reconnect_try_ = now;
    }
    return;
  }

  maybeRetryUbxConfig();
  maybePollAuxNav();
  if (pending_ubx_publish_ && nowSec() - pending_ubx_since_ >= cov_wait_timeout_sec_) {
    pending_ubx_publish_ = false;
    publish();
  }

  try {
    // CH340 fix: select() may report ready but read() returns empty.
    // Try reading regardless of available() — handle empty read as "not ready yet" not disconnect.
    size_t n = ser_->available();
    std::string data;
    if (n > 0) {
      data = ser_->read(std::min<size_t>(n, 2048));
      consecutive_read_errors_ = 0;
    } else {
      // CH340 false-ready: drain a bounded chunk anyway; empty = not ready, not disconnect.
      data = ser_->read(2048);
      if (data.empty()) {
        consecutive_read_errors_ = 0;
        if (last_data_time_ > 0.0 && nowSec() - last_data_time_ > data_timeout_sec_) {
          RCLCPP_INFO(this->get_logger(),
            "GNSS tidak mengirim data selama %.1f s; serial dibuka ulang", data_timeout_sec_);
          closeSerial();
          last_reconnect_try_ = nowSec();
        }
        return;  // belum ada data; port hanya ditutup setelah timeout nyata
      }
      consecutive_read_errors_ = 0;
    }
    last_data_time_ = nowSec();
    bytes_received_ += data.size();
    buf_.insert(buf_.end(), data.begin(), data.end());
    if (buf_.size() > 20000)
      buf_.erase(buf_.begin(), buf_.end() - 5000);
    processBuffer();
    if (pending_ubx_publish_ && nowSec() - pending_ubx_since_ >= cov_wait_timeout_sec_) {
      pending_ubx_publish_ = false;
      publish();
    }
  } catch (const serial::IOException & e) {
    consecutive_read_errors_++;
    if (consecutive_read_errors_ > 10) {
      RCLCPP_WARN(this->get_logger(), "GNSS serial I/O error after %d retries: %s",
        consecutive_read_errors_, e.what());
      closeSerial();
      consecutive_read_errors_ = 0;
    }
  } catch (const serial::SerialException & e) {
    consecutive_read_errors_++;
    if (consecutive_read_errors_ > 10) {
      RCLCPP_WARN(this->get_logger(), "GNSS serial exception after %d retries: %s",
        consecutive_read_errors_, e.what());
      closeSerial();
      consecutive_read_errors_ = 0;
    }
  }
}

// Fungsi: Memisahkan buffer serial menjadi frame NMEA/UBX lengkap lalu meneruskannya ke parser.
void GnssNode::processBuffer()
{
  while (true) {
    // Find NMEA start ('$') and UBX start (0xB5 0x62)
    auto it_dollar = std::find(buf_.begin(), buf_.end(), '$');
    auto it_ubx = buf_.end();
    for (auto it = buf_.begin(); it != buf_.end(); ++it) {
      if (static_cast<uint8_t>(*it) == 0xB5) {
        auto nxt = std::next(it);
        if (nxt != buf_.end() && static_cast<uint8_t>(*nxt) == 0x62) {
          it_ubx = it;
          break;
        }
      }
    }

    if (it_dollar == buf_.end() && it_ubx == buf_.end()) break;

    bool use_nmea;
    if (it_dollar != buf_.end() && it_ubx != buf_.end())
      use_nmea = std::distance(buf_.begin(), it_dollar) < std::distance(buf_.begin(), it_ubx);
    else
      use_nmea = (it_dollar != buf_.end());

    if (use_nmea) {
      // Erase everything before '$'
      if (it_dollar != buf_.begin())
        buf_.erase(buf_.begin(), it_dollar);

      // Find end-of-line (\r or \n)
      auto it_end = std::find_if(buf_.begin(), buf_.end(),
        [](uint8_t c) { return c == '\r' || c == '\n'; });
      if (it_end == buf_.end()) {
        if (buf_.size() > 200) buf_.erase(buf_.begin());
        break; // incomplete sentence, wait for more data
      }

      auto sentence_end = it_end;
      // Include \r\n if present
      if (*it_end == '\r' && std::next(it_end) != buf_.end() && *std::next(it_end) == '\n')
        sentence_end = std::next(it_end);

      std::string sentence(buf_.begin(), std::next(sentence_end));
      buf_.erase(buf_.begin(), std::next(sentence_end));

      // Trim whitespace
      while (!sentence.empty() && (sentence.back() == '\r' || sentence.back() == '\n'))
        sentence.pop_back();

      if (!sentence.empty() && sentence[0] == '$') {
        emitRaw(sentence);
        if (handleNmea(sentence)) {
          packets_parsed_++;
          publish();
        }
      }
      continue;
    }

    // UBX path
    if (it_ubx != buf_.begin())
      buf_.erase(buf_.begin(), it_ubx);
    if (buf_.size() < 8) break;

    uint8_t msg_class = buf_[2];
    uint8_t msg_id = buf_[3];
    uint16_t length = static_cast<uint16_t>(buf_[4]) | (static_cast<uint16_t>(buf_[5]) << 8);
    if (length > 4096) { buf_.erase(buf_.begin()); continue; }

    size_t total_len = 6 + length + 2;
    if (buf_.size() < total_len) break;

    uint8_t ck_a = buf_[6 + length];
    uint8_t ck_b = buf_[6 + length + 1];
    uint8_t ca = 0, cb = 0;
    ubxChecksum(&buf_[2], static_cast<size_t>(length) + 4u, ca, cb);
    if (ck_a != ca || ck_b != cb) {
      ++ubx_checksum_errors_;
      buf_.erase(buf_.begin());
      continue;
    }

    std::vector<uint8_t> payload(buf_.begin() + 6, buf_.begin() + 6 + length);
    buf_.erase(buf_.begin(), buf_.begin() + total_len);

    if (handleUbx(msg_class, msg_id, payload)) {
      packets_parsed_++;
      publish();
    }
  }
}

// Fungsi: Entry point proses GNSS; menjalankan ROS executor dan shutdown node secara bersih.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GnssNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
