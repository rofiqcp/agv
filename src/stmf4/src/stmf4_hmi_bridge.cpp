#include <algorithm>
#include <atomic>
#include <array>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <unistd.h>
#include <termios.h>

#include "action_msgs/srv/cancel_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {
constexpr double kPi = 3.14159265358979323846;

uint32_t wireCrc32(const std::string &data) {
  uint32_t crc = 0xFFFFFFFFU;
  for (const unsigned char c : data) {
    crc ^= c;
    for (uint8_t b = 0; b < 8U; ++b)
      crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320U) : (crc >> 1U);
  }
  return crc ^ 0xFFFFFFFFU;
}

fs::path agvRootPath() {
  if (const char * root = std::getenv("AGV_ROOT"); root && *root) return fs::path(root).lexically_normal();
  if (const char * home = std::getenv("HOME"); home && *home) return (fs::path(home) / "agv").lexically_normal();
  return (fs::current_path() / "agv").lexically_normal();
}

std::string resolveAgvPath(const std::string & value) {
  if (value.empty()) return value;
  const fs::path p(value);
  return (p.is_absolute() ? p : agvRootPath() / p).lexically_normal().string();
}

std::string upper(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

std::string trim(std::string s) {
  const auto first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return {};
  const auto last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

std::optional<double> jsonNumber(const std::string &s, const std::string &key) {
  const std::string needle = "\"" + key + "\":";
  const auto pos = s.find(needle);
  if (pos == std::string::npos) return std::nullopt;
  const char *start = s.c_str() + pos + needle.size();
  char *end = nullptr;
  errno = 0;
  const double value = std::strtod(start, &end);
  if (errno != 0 || end == start || !std::isfinite(value)) return std::nullopt;
  return value;
}

std::optional<bool> jsonBool(const std::string &s, const std::string &key) {
  const std::string needle = "\"" + key + "\":";
  const auto pos = s.find(needle);
  if (pos == std::string::npos) return std::nullopt;
  const auto start = pos + needle.size();
  if (s.compare(start, 4, "true") == 0) return true;
  if (s.compare(start, 5, "false") == 0) return false;
  return std::nullopt;
}

std::optional<std::string> jsonString(const std::string &s, const std::string &key) {
  const std::string needle = "\"" + key + "\":\"";
  const auto pos = s.find(needle);
  if (pos == std::string::npos) return std::nullopt;
  const auto start = pos + needle.size();
  const auto end = s.find('"', start);
  if (end == std::string::npos) return std::nullopt;
  return s.substr(start, end - start);
}

std::optional<std::string> kvString(const std::string &s, const std::string &key) {
  const std::string needle = key + "=";
  const auto pos = s.find(needle);
  if (pos == std::string::npos) return std::nullopt;
  const auto start = pos + needle.size();
  const auto end = s.find(';', start);
  return trim(s.substr(start, end == std::string::npos ? std::string::npos : end - start));
}

std::optional<std::string> jsonObject(const std::string &s, const std::string &key) {
  const std::string needle = "\"" + key + "\":{";
  const auto pos = s.find(needle);
  if (pos == std::string::npos) return std::nullopt;
  const size_t start = pos + needle.size() - 1U;
  int depth = 0;
  bool in_string = false;
  bool escape = false;
  for (size_t i = start; i < s.size(); ++i) {
    const char c = s[i];
    if (in_string) {
      if (escape) escape = false;
      else if (c == '\\') escape = true;
      else if (c == '"') in_string = false;
      continue;
    }
    if (c == '"') { in_string = true; continue; }
    if (c == '{') ++depth;
    else if (c == '}' && --depth == 0) return s.substr(start, i - start + 1U);
  }
  return std::nullopt;
}

std::optional<double> kvNumber(const std::string &s, const std::string &key) {
  const auto raw = kvString(s, key);
  if (!raw || raw->empty() || *raw == "N/A") return std::nullopt;
  errno = 0;
  char *end = nullptr;
  const double value = std::strtod(raw->c_str(), &end);
  if (errno != 0 || end == raw->c_str() || *end != '\0' || !std::isfinite(value)) return std::nullopt;
  return value;
}

std::optional<bool> kvBool(const std::string &s, const std::string &key) {
  const auto raw = kvString(s, key);
  if (!raw) return std::nullopt;
  const std::string u = upper(*raw);
  if (u == "TRUE" || u == "1" || u == "READY" || u == "YES") return true;
  if (u == "FALSE" || u == "0" || u == "WAIT" || u == "NO") return false;
  return std::nullopt;
}

std::string wireToken(std::string value, size_t max_len = 18U) {
  value = upper(trim(value));
  std::string out;
  out.reserve(std::min(max_len, value.size()));
  for (unsigned char c : value) {
    if (out.size() >= max_len) break;
    if (std::isalnum(c) || c == '_' || c == '-' || c == '.') out.push_back(static_cast<char>(c));
    else if (c == ' ' || c == '/') out.push_back('_');
  }
  return out.empty() ? "UNKNOWN" : out;
}

uint32_t steadyAgeMs(const std::chrono::steady_clock::time_point &stamp) {
  if (stamp.time_since_epoch().count() == 0) return 0xFFFFFFFFU;
  const auto now = std::chrono::steady_clock::now();
  if (now <= stamp) return 0U;
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - stamp).count();
  return static_cast<uint32_t>(std::min<int64_t>(ms, 0xFFFFFFFELL));
}

std::string className(int id) {
  if (id == 0) return "PERSON";
  if (id == 2) return "CAR";
  if (id == 3) return "MOTOR";
  return id >= 0 ? "OBJECT" : "NONE";
}

double yawFromQuat(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::string fixed(double value, int precision) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(precision) << value;
  return out.str();
}

std::string sanitizeWaypointName(std::string name) {
  name = trim(name);
  std::string out;
  out.reserve(std::min<size_t>(18, name.size()));
  for (unsigned char c : name) {
    if (out.size() >= 18) break;
    if (std::isalnum(c) || c == ' ' || c == '-' || c == '_') out.push_back(static_cast<char>(c));
  }
  while (!out.empty() && out.back() == ' ') out.pop_back();
  return out;
}
}  // namespace

class StmF4HmiBridge final : public rclcpp::Node {
public:
  StmF4HmiBridge() : Node("stmf4_hmi_bridge") {
    acquireInstanceLock();
    declareParameters();
    readParameters();
    initializeWaypoints();
    loadWaypoints();
    createRosInterfaces();
    perception_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "perception");
    esc_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "esc_ackermann");
    reconnect_timer_ = create_wall_timer(250ms, std::bind(&StmF4HmiBridge::reconnectTick, this));
    serial_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / serial_poll_hz_)), std::bind(&StmF4HmiBridge::serialTick, this));
    telemetry_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / telemetry_rate_hz_), std::bind(&StmF4HmiBridge::telemetryTick, this));
    heartbeat_timer_ = create_wall_timer(500ms, [this]() {
      if (fd_ >= 0) {
        const auto now = std::chrono::steady_clock::now();
        if (awaiting_host_session_) {
          if (last_host_hello_tx_.time_since_epoch().count() == 0 ||
              now - last_host_hello_tx_ > std::chrono::milliseconds(500)) {
            (void)sendLine("HOST:HELLO:" + std::to_string(host_session_token_), 0);
            last_host_hello_tx_ = now;
          }
        } else {
          (void)sendLine("ROS:1");
        }
        if (last_rx_.time_since_epoch().count() == 0) {
          if (serial_opened_at_.time_since_epoch().count() != 0 &&
              now - serial_opened_at_ > std::chrono::duration<double>(hmi_transport_timeout_sec_)) {
            ++silent_open_failures_;
            if (silent_open_failures_ >= 3U) {
              RCLCPP_ERROR(get_logger(),
                "F411 CDC silent after open (%lu consecutive first-response timeouts); escalating USB session recovery fail-closed",
                static_cast<unsigned long>(silent_open_failures_));
              if (last_usb_recovery_request_.time_since_epoch().count() == 0 ||
                  now - last_usb_recovery_request_ > std::chrono::seconds(8)) {
                // One-way recovery request: it is intentionally useful when CDC OUT
                // still works but the old IN transfer can no longer ACK.
                if (sendLine("USB:RECOVER", 0)) {
                  ++usb_recovery_requests_;
                  last_usb_recovery_request_ = now;
                }
              }
            } else {
              RCLCPP_WARN(get_logger(), "F411 CDC first-response timeout; closing and retrying");
            }
            publishState();
            closeSerial("first response timeout");
          }
        } else if (now - last_rx_ > std::chrono::duration<double>(hmi_transport_timeout_sec_)) {
          closeSerial("transport heartbeat timeout");
        } else if (!awaiting_host_session_) {
          if (last_usb_status_request_.time_since_epoch().count() == 0 ||
              now - last_usb_status_request_ > std::chrono::seconds(2)) {
            if (sendLine("USB:STATUS", 0)) last_usb_status_request_ = now;
          }
          if (last_compass_params_request_.time_since_epoch().count() == 0 ||
              now - last_compass_params_request_ > std::chrono::seconds(15)) {
            if (sendLine("NEO:COMPASS:PARAMS", 0)) last_compass_params_request_ = now;
          }
          // Production GNSS-rate contract: target NEO3 Pro is 10 Hz. If the
          // measured PVT stream is not near the configured target, retry the
          // F411->AP_Periph rate request at a slow bounded cadence. Firmware is
          // idempotent and will not restart the GNSS node when already at target.
          const bool have_rate = std::isfinite(last_neo3_pvt_rate_hz_) && last_neo3_pvt_rate_hz_ > 0.0;
          const bool target_mismatch = !have_rate ||
            std::abs(last_neo3_pvt_rate_hz_ - static_cast<double>(neo3pro_gnss_rate_hz_)) > 0.75;
          const bool gnss_recent = last_neo3_gnss_time_.time_since_epoch().count() != 0 &&
            now - last_neo3_gnss_time_ <= std::chrono::duration<double>(neo3_sensor_timeout_sec_);
          if (gnss_recent && target_mismatch &&
              (last_neo3_rate_command_.time_since_epoch().count() == 0 ||
               now - last_neo3_rate_command_ > std::chrono::duration<double>(neo3pro_rate_reapply_sec_))) {
            if (sendLine("NEO:GPS:RATE:" + std::to_string(neo3pro_gnss_rate_hz_), 0))
              last_neo3_rate_command_ = now;
          }
        }
      }
      sensorWatchdogTick();
    });
    command_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / command_rate_hz_), std::bind(&StmF4HmiBridge::commandTick, this));
    config_sync_timer_ = create_wall_timer(1s, std::bind(&StmF4HmiBridge::syncRuntimeConfig, this));
    publishConnected(false);
    publishState();
    RCLCPP_INFO(get_logger(), "STM32F411 HMI SCADA bridge ready | serial=%s @ %d | mode=%s",
                serial_device_.c_str(), serial_baud_, mode_.c_str());
  }

  ~StmF4HmiBridge() override {
    closeSerial("shutdown");
    if (lock_fd_ >= 0) { ::flock(lock_fd_, LOCK_UN); ::close(lock_fd_); lock_fd_ = -1; }
  }

private:
  struct Waypoint {
    bool saved{false};
    std::string name;
    double x{0.0}, y{0.0}, yaw{0.0}, lat{0.0}, lon{0.0};
  };

  struct Neo3ProWireState {
    bool initialized{false};
    std::uint32_t last_sequence{0U};
    std::uint64_t gaps{0U};
    std::uint64_t duplicates{0U};
    std::uint64_t crc_errors{0U};
  };

  struct Neo3ProGnssMeta {
    bool valid{false};
    std::uint32_t mcu_ms{0U};
    int node_id{0};
    std::uint64_t network_timestamp_usec{0U};
    std::uint64_t gnss_timestamp_usec{0U};
    int time_standard{0}, leap_seconds{0};
    int status{0}, mode{0}, sub_mode{0};
    int sats_used{0}, sats_visible{0};
    double height_msl_m{0.0}, height_ellipsoid_m{0.0};
    double gdop{-1.0}, pdop{-1.0}, hdop{-1.0}, vdop{-1.0};
    double tdop{-1.0}, ndop{-1.0}, edop{-1.0}, rate_hz{0.0};
    bool pos_cov_valid{false}, vel_cov_valid{false};
    std::chrono::steady_clock::time_point received{};
  };

  struct Neo3ProCovariance {
    bool valid{false};
    std::uint32_t mcu_ms{0U};
    int node_id{0};
    std::vector<double> covariance;
    std::chrono::steady_clock::time_point received{};
  };

  struct Neo3ProMagMeta {
    bool valid{false};
    std::uint32_t mcu_ms{0U};
    int node_id{0}, message_type{0}, sensor_id{0};
    double x_ut{0.0}, y_ut{0.0}, z_ut{0.0};
    std::vector<double> covariance_t2;
    std::chrono::steady_clock::time_point received{};
  };

  struct Neo3ProHeading {
    bool seen{false}, valid{false}, accuracy_valid{false};
    std::uint32_t mcu_ms{0U};
    int node_id{0};
    double heading_rad{0.0}, accuracy_rad{0.0};
    std::chrono::steady_clock::time_point received{};
  };

  struct Neo3ProNodeHealth {
    bool seen{false};
    int node_id{0}, health{0}, mode{0}, sub_mode{0};
    std::uint32_t mcu_ms{0U}, uptime_sec{0U};
    std::uint16_t vendor_status{0U};
    std::chrono::steady_clock::time_point received{};
  };

  struct Neo3ProGnssHealth {
    bool seen{false}, healthy{true};
    int node_id{0};
    std::uint32_t mcu_ms{0U};
    std::uint64_t error_codes{0U}, status_bits{0U};
    std::chrono::steady_clock::time_point received{};
  };

  static constexpr size_t kWaypointCount = 4;

  void acquireInstanceLock() {
    lock_fd_ = ::open("/tmp/adv_stmf4_hmi_bridge.lock", O_CREAT | O_RDWR | O_CLOEXEC, 0660);
    if (lock_fd_ < 0 || ::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      if (lock_fd_ >= 0) ::close(lock_fd_);
      lock_fd_ = -1;
      throw std::runtime_error("another stmf4_hmi_bridge instance already owns the HMI bridge lock");
    }
  }

  void declareParameters() {
    declare_parameter<std::string>("serial_device", "auto");
    declare_parameter<int>("serial_baud", 1000000);
    declare_parameter<double>("reconnect_sec", 0.5);
    declare_parameter<double>("telemetry_rate_hz", 20.0);
    declare_parameter<double>("serial_poll_hz", 1000.0);
    declare_parameter<double>("command_rate_hz", 30.0);
    declare_parameter<double>("heartbeat_sec", 5.0);
    declare_parameter<double>("hmi_transport_timeout_sec", 2.0);
    declare_parameter<double>("manual_speed_max_mps", 1.0);
    declare_parameter<double>("manual_command_lease_sec", 0.30);
    declare_parameter<int>("manual_speed_min_pct", 10);
    declare_parameter<int>("manual_speed_max_pct", 50);
    declare_parameter<int>("manual_speed_default_pct", 20);
    declare_parameter<double>("steering_test_angle_deg", 20.0);
    declare_parameter<double>("hmi_steer_full_scale_deg", 90.0);
    declare_parameter<double>("teleop_yaw_max_deg_s", 80.0);
    declare_parameter<bool>("invert_hmi_steering", true);
    declare_parameter<std::string>("default_mode", "AUTO");
    declare_parameter<std::string>("waypoint_file", "data/hmi_waypoints.tsv");
    declare_parameter<double>("waypoint_pose_timeout_sec", 2.5);
    declare_parameter<double>("neo3_sensor_timeout_sec", 2.0);
    declare_parameter<std::string>("neo3_gnss_frame_id", "gnss_link");
    declare_parameter<std::string>("rm3100_mag_frame_id", "gnss_link");
    declare_parameter<std::string>("neo3pro_baro_frame_id", "gnss_link");
    declare_parameter<double>("rm3100_mag_sigma_ut", 3.0);
    declare_parameter<int>("neo3pro_navsat_service_mask", sensor_msgs::msg::NavSatStatus::SERVICE_GPS);
    declare_parameter<int>("neo3pro_gnss_rate_hz", 10);
    declare_parameter<double>("neo3pro_min_usable_rate_hz", 7.0);
    declare_parameter<double>("neo3pro_rate_reapply_sec", 60.0);
    declare_parameter<bool>("neo3_safety_button_as_estop", false);
    declare_parameter<bool>("publish_stm32_gnss", true);
    declare_parameter<bool>("neo3_require_protocol_crc", true);
  }

  void readParameters() {
    serial_device_ = get_parameter("serial_device").as_string();
    serial_baud_ = static_cast<int>(get_parameter("serial_baud").as_int());
    reconnect_sec_ = std::clamp(get_parameter("reconnect_sec").as_double(), 0.1, 5.0);
    telemetry_rate_hz_ = std::clamp(get_parameter("telemetry_rate_hz").as_double(), 2.0, 30.0);
    serial_poll_hz_ = std::clamp(get_parameter("serial_poll_hz").as_double(), 100.0, 2000.0);
    command_rate_hz_ = std::clamp(get_parameter("command_rate_hz").as_double(), 10.0, 50.0);
    heartbeat_sec_ = std::clamp(get_parameter("heartbeat_sec").as_double(), 0.25, 5.0);
    hmi_transport_timeout_sec_ = std::clamp(get_parameter("hmi_transport_timeout_sec").as_double(), 0.5, 10.0);
    manual_speed_max_mps_ = std::clamp(get_parameter("manual_speed_max_mps").as_double(), 0.05, 3.0);
    manual_command_lease_sec_ = std::clamp(get_parameter("manual_command_lease_sec").as_double(), 0.15, 1.0);
    speed_min_pct_ = std::clamp(static_cast<int>(get_parameter("manual_speed_min_pct").as_int()), 1, 100);
    speed_max_pct_ = std::clamp(static_cast<int>(get_parameter("manual_speed_max_pct").as_int()), speed_min_pct_, 100);
    manual_speed_pct_ = std::clamp(static_cast<int>(get_parameter("manual_speed_default_pct").as_int()), speed_min_pct_, speed_max_pct_);
    steering_test_angle_deg_ = std::clamp(get_parameter("steering_test_angle_deg").as_double(), 5.0, 30.0);
    hmi_steer_full_scale_deg_ = std::clamp(get_parameter("hmi_steer_full_scale_deg").as_double(), 1.0, 90.0);
    teleop_yaw_max_rps_ = std::clamp(get_parameter("teleop_yaw_max_deg_s").as_double(), 1.0, 180.0) * kPi / 180.0;
    invert_hmi_steering_ = get_parameter("invert_hmi_steering").as_bool();
    mode_ = upper(trim(get_parameter("default_mode").as_string())) == "MANUAL" ? "MANUAL" : "AUTO";
    waypoint_file_ = resolveAgvPath(get_parameter("waypoint_file").as_string());
    waypoint_pose_timeout_sec_ = std::clamp(get_parameter("waypoint_pose_timeout_sec").as_double(), 0.25, 10.0);
    neo3_sensor_timeout_sec_ = std::clamp(get_parameter("neo3_sensor_timeout_sec").as_double(), 0.5, 10.0);
    neo3_gnss_frame_id_ = get_parameter("neo3_gnss_frame_id").as_string();
    rm3100_mag_frame_id_ = get_parameter("rm3100_mag_frame_id").as_string();
    neo3pro_baro_frame_id_ = get_parameter("neo3pro_baro_frame_id").as_string();
    rm3100_mag_sigma_ut_ = std::clamp(get_parameter("rm3100_mag_sigma_ut").as_double(), 0.1, 100.0);
    neo3pro_navsat_service_mask_ = static_cast<std::uint16_t>(std::clamp<std::int64_t>(get_parameter("neo3pro_navsat_service_mask").as_int(), 0, 15));
    neo3pro_gnss_rate_hz_ = static_cast<int>(std::clamp<std::int64_t>(get_parameter("neo3pro_gnss_rate_hz").as_int(), 5, 20));
    neo3pro_min_usable_rate_hz_ = std::clamp(get_parameter("neo3pro_min_usable_rate_hz").as_double(), 1.0, 20.0);
    neo3pro_rate_reapply_sec_ = std::clamp(get_parameter("neo3pro_rate_reapply_sec").as_double(), 2.0, 60.0);
    neo3_safety_button_as_estop_ = get_parameter("neo3_safety_button_as_estop").as_bool();
    publish_stm32_gnss_ = get_parameter("publish_stm32_gnss").as_bool();
    neo3_require_protocol_crc_ = get_parameter("neo3_require_protocol_crc").as_bool();
    if (serial_baud_ != 1000000) throw std::runtime_error("stmf4 requires serial_baud=1000000");
  }

  void initializeWaypoints() {
    const char *defaults[kWaypointCount] = {"Titik A", "Titik B", "Titik C", "Titik D"};
    for (size_t i = 0; i < kWaypointCount; ++i) waypoints_[i].name = defaults[i];
  }

  static bool waypointPoseSane(const Waypoint &wp) {
    constexpr double kMaxMapCoordinateM = 1.0e6;
    return std::isfinite(wp.x) && std::isfinite(wp.y) && std::isfinite(wp.yaw) &&
           std::isfinite(wp.lat) && std::isfinite(wp.lon) &&
           std::abs(wp.x) <= kMaxMapCoordinateM && std::abs(wp.y) <= kMaxMapCoordinateM &&
           std::abs(wp.yaw) <= (2.0 * kPi + 1.0e-6) &&
           wp.lat >= -90.0 && wp.lat <= 90.0 && wp.lon >= -180.0 && wp.lon <= 180.0;
  }

  void loadWaypoints() {
    if (waypoint_file_.empty()) return;
    std::ifstream in(waypoint_file_);
    if (!in.good()) return;
    std::string line;
    while (std::getline(in, line)) {
      std::vector<std::string> f;
      std::stringstream ss(line);
      std::string item;
      while (std::getline(ss, item, '\t')) f.push_back(item);
      if (f.size() != 8) continue;
      const int idx = std::atoi(f[0].c_str());
      if (idx < 0 || idx >= static_cast<int>(kWaypointCount)) continue;
      Waypoint &wp = waypoints_[static_cast<size_t>(idx)];
      const std::string name = sanitizeWaypointName(f[2]);
      try {
        wp.saved = std::atoi(f[1].c_str()) != 0;
        if (!name.empty()) wp.name = name;
        wp.x = std::stod(f[3]); wp.y = std::stod(f[4]); wp.yaw = std::stod(f[5]);
        wp.lat = std::stod(f[6]); wp.lon = std::stod(f[7]);
        if (!waypointPoseSane(wp)) wp.saved = false;
      } catch (...) {
        wp.saved = false;
      }
    }
  }

  bool persistWaypoints() {
    if (waypoint_file_.empty()) return false;
    try {
      const fs::path target(waypoint_file_);
      const fs::path parent = target.has_parent_path() ? target.parent_path() : fs::path(".");
      fs::create_directories(parent);
      const fs::path temp = target.string() + ".tmp";
      {
        std::ofstream out(temp, std::ios::trunc);
        if (!out.good()) return false;
        out << std::setprecision(15);
        for (size_t i = 0; i < kWaypointCount; ++i) {
          const Waypoint &wp = waypoints_[i];
          out << i << '\t' << (wp.saved ? 1 : 0) << '\t' << wp.name << '\t'
              << wp.x << '\t' << wp.y << '\t' << wp.yaw << '\t' << wp.lat << '\t' << wp.lon << '\n';
        }
        out.flush();
        if (!out.good()) return false;
      }
      const int tmp_fd = ::open(temp.c_str(), O_RDONLY | O_CLOEXEC);
      if (tmp_fd < 0) return false;
      const bool file_synced = ::fsync(tmp_fd) == 0;
      ::close(tmp_fd);
      if (!file_synced) return false;

      std::error_code ec;
      fs::rename(temp, target, ec);
      if (ec) { fs::remove(temp, ec); return false; }

      const int dir_fd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
      if (dir_fd < 0) return false;
      const bool dir_synced = ::fsync(dir_fd) == 0;
      ::close(dir_fd);
      return dir_synced;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Waypoint persistence failed: %s", e.what());
      return false;
    }
  }

  bool mapPoseFresh() const {
    if (!have_map_pose_) return false;
    return std::chrono::steady_clock::now() - last_map_pose_ <=
           std::chrono::duration<double>(waypoint_pose_timeout_sec_);
  }

  rclcpp::QoS stateQos() const { return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(); }

  void createRosInterfaces() {
    connected_pub_ = create_publisher<std_msgs::msg::Bool>("/hmi/connected", stateQos());
    page_pub_ = create_publisher<std_msgs::msg::String>("/hmi/page", stateQos());
    mode_pub_ = create_publisher<std_msgs::msg::String>("/hmi/operator_mode", stateQos());
    waypoints_pub_ = create_publisher<std_msgs::msg::String>("/hmi/waypoints", stateQos());
    navigation_state_pub_ = create_publisher<std_msgs::msg::String>("/hmi/navigation_state", stateQos());
    manual_state_pub_ = create_publisher<std_msgs::msg::String>("/hmi/manual_state", stateQos());
    status_pub_ = create_publisher<std_msgs::msg::String>("/hmi/status", stateQos());
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/hmi/cmd_vel", 10);
    source_pub_ = create_publisher<std_msgs::msg::String>("/hmi/active_source", stateQos());
    commissioning_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/esc/commissioning/raw_actuator", 10);
    neo3_fix_raw_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gnss/fix_raw", rclcpp::SensorDataQoS().keep_last(5));
    neo3_fix_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/gnss/fix", rclcpp::SensorDataQoS().keep_last(5));
    neo3_vel_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/gnss/vel", rclcpp::SensorDataQoS().keep_last(5));
    neo3_quality_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/gnss/quality", rclcpp::SensorDataQoS().keep_last(5));
    neo3_gnss_state_pub_ = create_publisher<std_msgs::msg::String>("/gnss/state", stateQos());
    neo3_gnss_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/gnss/connected", stateQos());
    neo3pro_mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/neo3pro/mag", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_rm3100_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3pro/rm3100_connected", stateQos());
    neo3pro_node_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3pro/node_connected", stateQos());
    neo3pro_gnss_status_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3pro/gnss/status_connected", stateQos());
    neo3pro_baro_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3pro/baro/connected", stateQos());
    neo3pro_pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>("/neo3pro/baro/pressure", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/neo3pro/baro/temperature", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_gnss_meta_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/neo3pro/gnss/meta", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_gnss_cov_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/neo3pro/gnss/covariance", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_ecef_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/neo3pro/gnss/ecef", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_mag_meta_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/neo3pro/mag/meta", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_node_status_pub_ = create_publisher<std_msgs::msg::String>("/neo3pro/node_status", stateQos());
    neo3pro_node_info_pub_ = create_publisher<std_msgs::msg::String>("/neo3pro/node_info", stateQos());
    neo3pro_param_pub_ = create_publisher<std_msgs::msg::String>("/neo3pro/param", stateQos());
    neo3pro_gnss_status_pub_ = create_publisher<std_msgs::msg::String>("/neo3pro/gnss/status", stateQos());
    neo3pro_can_status_pub_ = create_publisher<std_msgs::msg::String>("/neo3pro/can/status", stateQos());
    neo3pro_can_raw_pub_ = create_publisher<std_msgs::msg::String>("/neo3pro/can/raw", stateQos());
    neo3pro_health_pub_ = create_publisher<std_msgs::msg::String>("/neo3pro/health", stateQos());
    neo3pro_dna_status_pub_ = create_publisher<std_msgs::msg::String>("/neo3pro/dna/status", stateQos());
    neo3pro_heading_pub_ = create_publisher<std_msgs::msg::Float64>("/neo3pro/gnss/heading_rad", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_heading_accuracy_pub_ = create_publisher<std_msgs::msg::Float64>("/neo3pro/gnss/heading_accuracy_rad", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_time_reference_pub_ = create_publisher<sensor_msgs::msg::TimeReference>("/gnss/time_reference", rclcpp::SensorDataQoS().keep_last(10));
    neo3pro_safety_button_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3pro/safety_button", stateQos());
    neo3pro_safety_button_raw_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/neo3pro/safety_button_raw", rclcpp::SensorDataQoS().keep_last(10));
    neo3_safety_switch_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3/safety_switch", stateQos());
    // Satu publisher E-stop sistem: safety switch NEO3 dipetakan langsung ke
    // gate ROS, sementara F411 juga menghentikan F103 secara hardware-link lokal.
    neo3_estop_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/estop", stateQos());
    neo3_status_pub_ = create_publisher<std_msgs::msg::String>("/neo3/status", stateQos());
    usb_status_pub_ = create_publisher<std_msgs::msg::String>("/stmf4/usb/status", stateQos());
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/navigation/goal_request", 10);
    cancel_nav_client_ = create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");

    request_sub_ = create_subscription<std_msgs::msg::String>("/hmi/request", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { handleRequest(msg->data); });
    neo3_command_sub_ = create_subscription<std_msgs::msg::String>("/neo3/command", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const std::string command = upper(trim(msg->data));
        if (neo3CommandAllowed(command)) {
          (void)sendLine("NEO:" + command);
        } else {
          RCLCPP_WARN(get_logger(), "Rejected unsupported NEO3PRO command: %s", msg->data.c_str());
        }
      });
    // ESC transport intentionally absent: F411/stmf4 is HMI + sensor bridge only.
    boolSub("/gnss/connected", gnss_ready_);
    boolSub("/imu/connected", imu_ready_);
    camera_connected_sub_ = create_subscription<std_msgs::msg::Bool>("/perception/camera_connected", stateQos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { camera_ready_ = msg->data; last_camera_state_time_ = std::chrono::steady_clock::now(); });
    camera_healthy_sub_ = create_subscription<std_msgs::msg::Bool>("/perception/camera_healthy", stateQos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { perception_ready_ = msg->data; last_camera_state_time_ = std::chrono::steady_clock::now(); });
    boolSub("/perception/emergency_stop", perception_emergency_);
    boolSub("/esc/ready", esc_ready_);
    boolSub("/esc/feedback_valid", esc_feedback_);
    boolSub("/esc/steer/connected", steer_connected_);
    boolSub("/system/motion_ready", motion_ready_);
    boolSub("/system/nav2_ready", nav2_ready_);
    boolSub("/safety/estop", estop_);
    stringSub("/system/localization_state", localization_state_text_, stateQos());
    stringSub("/system/gnss_status", gnss_status_text_, stateQos());
    stringSub("/system/imu_status", imu_status_text_, stateQos());
    stringSub("/system/ekf_local_status", ekf_local_status_text_, stateQos());
    stringSub("/system/ekf_global_status", ekf_global_status_text_, stateQos());
    stringSub("/perception/lane_safety_state", lane_state_text_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());

    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/gnss/fix_raw", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) { latitude_ = msg->latitude; longitude_ = msg->longitude; });
    quality_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/gnss/quality", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (msg->data.size() > 0 && std::isfinite(msg->data[0])) satellites_ = std::clamp(static_cast<int>(std::lround(msg->data[0])), 0, 99);
        if (msg->data.size() > 3 && std::isfinite(msg->data[3])) fix_type_ = std::clamp(static_cast<int>(std::lround(msg->data[3])), 0, 4);
        if (msg->data.size() > 15 && std::isfinite(msg->data[15])) hdop_ = msg->data[15];
        else if (msg->data.size() > 1 && std::isfinite(msg->data[1])) hdop_ = msg->data[1];
        if (msg->data.size() > 2 && std::isfinite(msg->data[2])) hacc_m_ = std::max(0.0, msg->data[2]);
        if (msg->data.size() > 23 && std::isfinite(msg->data[23])) gnss_age_sec_ = std::max(0.0, msg->data[23]);
        if (msg->data.size() > 7 && std::isfinite(msg->data[7])) {
          heading_deg_ = msg->data[7] * 180.0 / kPi;
          if (heading_deg_ < 0.0) heading_deg_ += 360.0;
        }
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        if (std::isfinite(msg->angular_velocity.z)) gyro_z_rps_ = msg->angular_velocity.z;
        imu_yaw_deg_ = yawFromQuat(msg->orientation) * 180.0 / kPi;
        if (imu_yaw_deg_ < 0.0) imu_yaw_deg_ += 360.0;
        if (!gnss_ready_) heading_deg_ = imu_yaw_deg_;
        last_imu_data_time_ = std::chrono::steady_clock::now();
      });
    floatSub("/esc/drive_target_mps", drive_target_mps_);
    floatSub("/esc/drive_actual_mps", drive_actual_mps_);
    floatSub("/esc/steering_target_rad", steering_target_rad_);
    floatSub("/esc/steering_actual_rad", steering_actual_rad_);
    esc_status_sub_ = create_subscription<std_msgs::msg::String>("/esc/status", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const auto p = msg->data.find(" right=");
        if (p != std::string::npos) {
          char *end = nullptr;
          const double v = std::strtod(msg->data.c_str() + p + 7, &end);
          if (end != msg->data.c_str() + p + 7 && std::isfinite(v)) {
            motor_erpm_ = v;
          }
        }
      });
    foc_telemetry_sub_ = create_subscription<std_msgs::msg::String>("/esc/foc/telemetry", stateQos(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const auto left = jsonObject(msg->data, "left");
        const auto right = jsonObject(msg->data, "right");
        const auto cal = jsonObject(msg->data, "steering_cal");
        if (left && right) {
          left_vbus_v_ = jsonNumber(*left, "vbus_v").value_or(0.0);
          left_current_motor_a_ = jsonNumber(*left, "current_motor_a").value_or(0.0);
          left_current_in_a_ = jsonNumber(*left, "current_in_a").value_or(0.0);
          left_id_a_ = jsonNumber(*left, "id_a").value_or(0.0);
          left_iq_a_ = jsonNumber(*left, "iq_a").value_or(0.0);
          left_duty_ = jsonNumber(*left, "duty").value_or(0.0);
          left_temp_mos_c_ = jsonNumber(*left, "temp_mos_c").value_or(0.0);
          left_erpm_ = jsonNumber(*left, "erpm").value_or(0.0);
          left_fault_ = static_cast<unsigned>(std::clamp(jsonNumber(*left, "fault").value_or(255.0), 0.0, 255.0));
          left_position_deg_ = jsonNumber(*left, "position_deg").value_or(0.0);
          right_vbus_v_ = jsonNumber(*right, "vbus_v").value_or(0.0);
          right_current_motor_a_ = jsonNumber(*right, "current_motor_a").value_or(0.0);
          right_current_in_a_ = jsonNumber(*right, "current_in_a").value_or(0.0);
          right_id_a_ = jsonNumber(*right, "id_a").value_or(0.0);
          right_iq_a_ = jsonNumber(*right, "iq_a").value_or(0.0);
          right_duty_ = jsonNumber(*right, "duty").value_or(0.0);
          right_temp_mos_c_ = jsonNumber(*right, "temp_mos_c").value_or(0.0);
          right_erpm_ = jsonNumber(*right, "erpm").value_or(0.0);
          right_fault_ = static_cast<unsigned>(std::clamp(jsonNumber(*right, "fault").value_or(255.0), 0.0, 255.0));
          foc_values_valid_ = std::isfinite(left_vbus_v_) && std::isfinite(right_vbus_v_);
        }
        if (cal) {
          steering_calibrated_ext_ = jsonBool(*cal, "calibrated").value_or(false);
          steering_homed_ext_ = jsonBool(*cal, "homed").value_or(false);
          steering_synced_ext_ = jsonBool(*cal, "encoder_synced").value_or(false);
          steering_raw_count_ext_ = static_cast<int32_t>(jsonNumber(*cal, "raw_now").value_or(0.0));
          steering_span_ext_ = static_cast<int32_t>(jsonNumber(*cal, "raw_right").value_or(0.0));
          steering_raw_target_ext_ = static_cast<int32_t>(jsonNumber(*cal, "raw_target").value_or(0.0));
          steering_cal_ext_valid_ = true;
        }
        last_foc_telemetry_time_ = std::chrono::steady_clock::now();
      });
    obstacle_sub_ = create_subscription<std_msgs::msg::String>("/perception/obstacle_metrics", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { parseObstacle(msg->data); });
    drivable_sub_ = create_subscription<std_msgs::msg::String>("/perception/drivable_space", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        if (auto v = jsonBool(msg->data, "valid")) drivable_valid_ = *v;
        drivable_valid_rows_ = static_cast<unsigned>(std::max(0.0, jsonNumber(msg->data, "valid_rows").value_or(0.0)));
        drivable_sample_rows_ = static_cast<unsigned>(std::max(0.0, jsonNumber(msg->data, "sample_rows").value_or(0.0)));
        last_drivable_time_ = std::chrono::steady_clock::now();
      });
    performance_sub_ = create_subscription<std_msgs::msg::String>("/perception/performance", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        for (const char *key : {"fps", "pipeline_fps", "pipeline_fps_ema"}) {
          if (auto v = jsonNumber(msg->data, key)) { camera_fps_ = std::max(0.0, *v); break; }
        }
        perception_latency_ms_ = jsonNumber(msg->data, "pipeline_ms_per_frame").value_or(0.0);
        perception_dropped_frames_ = static_cast<unsigned>(std::max(0.0, jsonNumber(msg->data, "capture_dropped_total").value_or(0.0)));
        perception_raw_detection_count_ = static_cast<unsigned>(std::max(0.0, jsonNumber(msg->data, "raw_detection_count").value_or(0.0)));
        perception_confirmed_count_ = static_cast<unsigned>(std::max(0.0, jsonNumber(msg->data, "confirmed_obstacle_count").value_or(0.0)));
        perception_backend_ = wireToken(jsonString(msg->data, "backend").value_or("UNKNOWN"), 10U);
        last_performance_time_ = std::chrono::steady_clock::now();
      });
    lane_metrics_sub_ = create_subscription<std_msgs::msg::String>("/yolop/lane_metrics", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        lane_metric_valid_ = jsonBool(msg->data, "valid").value_or(false);
        lane_center_offset_m_ = jsonNumber(msg->data, "center_error_m").value_or(0.0);
        lane_confidence_pct_ = 100.0 * jsonNumber(msg->data, "confidence").value_or(0.0);
        lane_road_width_m_ = jsonNumber(msg->data, "road_width_m").value_or(0.0);
        lane_left_clearance_m_ = jsonNumber(msg->data, "left_clearance_m").value_or(0.0);
        lane_right_clearance_m_ = jsonNumber(msg->data, "right_clearance_m").value_or(0.0);
        lane_heading_error_deg_ = jsonNumber(msg->data, "heading_error_rad").value_or(0.0) * 180.0 / kPi;
        last_lane_metrics_time_ = std::chrono::steady_clock::now();
      });
    map_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered_map", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto &p = msg->pose.pose.position;
        const double yaw = yawFromQuat(msg->pose.pose.orientation);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(yaw)) return;
        map_x_ = p.x; map_y_ = p.y; map_yaw_ = yaw;
        map_cov_x_ = std::max(0.0, msg->pose.covariance[0]);
        map_cov_y_ = std::max(0.0, msg->pose.covariance[7]);
        map_yaw_var_ = std::max(0.0, msg->pose.covariance[35]);
        have_map_pose_ = true; last_map_pose_ = std::chrono::steady_clock::now();
      });
    local_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto &p = msg->pose.pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return;
        odom_x_ = p.x; odom_y_ = p.y; odom_yaw_ = yawFromQuat(msg->pose.pose.orientation);
        odom_linear_mps_ = msg->twist.twist.linear.x; odom_yaw_rate_rps_ = msg->twist.twist.angular.z;
        last_local_odom_time_ = std::chrono::steady_clock::now();
      });
    mppi_status_sub_ = create_subscription<std_msgs::msg::String>("/navigation/mppi_closed_loop/status", stateQos(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        mppi_status_text_ = msg->data;
        last_mppi_status_time_ = std::chrono::steady_clock::now();
      });
    trajectory_state_sub_ = create_subscription<std_msgs::msg::String>("/navigation/trajectory_safety_state", stateQos(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        trajectory_state_text_ = msg->data;
        last_trajectory_state_time_ = std::chrono::steady_clock::now();
      });
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/local_costmap/costmap", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        unsigned occupied = 0U;
        bool blocked = false;
        for (const auto cell : msg->data) {
          if (cell >= 50) ++occupied;
          if (cell >= 100) blocked = true;
        }
        costmap_obstacle_cells_ = occupied;
        costmap_blocked_ = blocked;
        last_costmap_time_ = std::chrono::steady_clock::now();
      });
    nav_goal_state_sub_ = create_subscription<std_msgs::msg::String>("/navigation/goal_state", stateQos(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { handleNavigationGoalState(msg->data); });
  }

  void boolSub(const std::string &topic, bool &field) {
    bool *const target = &field;
    bool_subs_.push_back(create_subscription<std_msgs::msg::Bool>(topic, stateQos(),
      [target](std_msgs::msg::Bool::ConstSharedPtr msg) { *target = msg->data; }));
  }

  void floatSub(const std::string &topic, double &field) {
    double *const target = &field;
    float_subs_.push_back(create_subscription<std_msgs::msg::Float64>(topic, 10,
      [target](std_msgs::msg::Float64::ConstSharedPtr msg) { if (std::isfinite(msg->data)) *target = msg->data; }));
  }

  void stringSub(const std::string &topic, std::string &field, const rclcpp::QoS &qos) {
    std::string *const target = &field;
    string_subs_.push_back(create_subscription<std_msgs::msg::String>(topic, qos,
      [target](std_msgs::msg::String::ConstSharedPtr msg) { *target = trim(msg->data); }));
  }

  bool validWaypointIndex(int index) const {
    return index >= 0 && index < static_cast<int>(kWaypointCount);
  }

  void publishWaypointState() {
    if (!waypoints_pub_ || !navigation_state_pub_) return;
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{\"selected\":" << selected_waypoint_
        << ",\"active_target\":\"" << active_target_ << "\""
        << ",\"nav_state\":\"" << navigation_state_ << "\",\"items\":[";
    for (size_t i = 0; i < kWaypointCount; ++i) {
      if (i) out << ',';
      const Waypoint &wp = waypoints_[i];
      out << "{\"index\":" << i << ",\"saved\":" << (wp.saved ? "true" : "false")
          << ",\"name\":\"" << wp.name << "\",\"x\":" << fixed(wp.x, 4)
          << ",\"y\":" << fixed(wp.y, 4) << ",\"yaw\":" << fixed(wp.yaw, 5)
          << ",\"lat\":" << fixed(wp.lat, 8) << ",\"lon\":" << fixed(wp.lon, 8) << '}';
    }
    out << "]}";
    msg.data = out.str();
    waypoints_pub_->publish(msg);

    std::ostringstream nav;
    nav << "{\"state\":\"" << navigation_state_ << "\",\"target\":\"" << active_target_
        << "\",\"selected\":" << selected_waypoint_ << ",\"goal_state\":\"" << last_goal_state_
        << "\",\"origin\":\"" << navigation_origin_ << "\"}";
    msg.data = nav.str();
    navigation_state_pub_->publish(msg);
  }

  void mirrorWaypointState(bool force, bool force_nav = false) {
    for (size_t i = 0; i < kWaypointCount; ++i) {
      const Waypoint &wp = waypoints_[i];
      sendState("WP" + std::to_string(i), std::string(wp.saved ? "1:" : "0:") + wp.name, force);
    }
    sendState("WPSEL", std::to_string(selected_waypoint_), force);
    sendState("TARGET", active_target_.empty() ? "NONE" : active_target_, force);
    sendState("NAV", navigation_state_, force || force_nav);
  }

  void selectWaypoint(int index, const std::string &origin, bool mirror) {
    if (!validWaypointIndex(index)) { reject("waypoint index must be 0..3"); return; }
    selected_waypoint_ = index;
    navigation_origin_ = origin;
    if (navigation_state_ != "NAVIGATING" && navigation_state_ != "QUEUED") {
      if (waypoints_[static_cast<size_t>(index)].saved) {
        navigation_state_ = "SELECTED";
        active_target_ = waypoints_[static_cast<size_t>(index)].name;
      } else {
        navigation_state_ = "IDLE";
        active_target_ = "NONE";
      }
    }
    if (mirror) sendState("WPSEL", std::to_string(selected_waypoint_), true);
    publishState();
  }

  void saveWaypoint(int index, std::string requested_name, const std::string &origin) {
    if (!validWaypointIndex(index)) { reject("waypoint index must be 0..3"); return; }
    if (!mapPoseFresh()) {
      reject("save waypoint requires fresh /odometry/filtered_map pose");
      sendLine("ERR:WP_SAVE_POSE_NOT_READY");
      return;
    }
    Waypoint &wp = waypoints_[static_cast<size_t>(index)];
    requested_name = sanitizeWaypointName(requested_name);
    if (!requested_name.empty()) wp.name = requested_name;
    wp.saved = true;
    wp.x = map_x_; wp.y = map_y_; wp.yaw = map_yaw_;
    wp.lat = std::isfinite(latitude_) ? latitude_ : 0.0;
    wp.lon = std::isfinite(longitude_) ? longitude_ : 0.0;
    selected_waypoint_ = index;
    active_target_ = wp.name;
    navigation_state_ = "SELECTED";
    navigation_origin_ = origin;
    last_rejection_.clear();
    if (!persistWaypoints()) RCLCPP_WARN(get_logger(), "Waypoint saved in memory but persistence failed");
    tx_cache_.erase("WP" + std::to_string(index));
    publishState();
    mirrorWaypointState(true);
    RCLCPP_INFO(get_logger(), "Waypoint %d '%s' saved map=(%.3f, %.3f, %.3f)",
                index, wp.name.c_str(), wp.x, wp.y, wp.yaw);
  }

  bool navigationMotionGateReady() const {
    if (estop_ || !motion_ready_ || !nav2_ready_ || !esc_ready_ || !esc_feedback_ || !mapPoseFresh()) return false;
    const bool planning_ready = kvBool(localization_state_text_, "planning_ready").value_or(false);
    const bool localization_motion_ready = kvBool(localization_state_text_, "motion_localization_ready").value_or(false);
    return planning_ready && localization_motion_ready;
  }

  void goWaypoint(int index, const std::string &origin) {
    if (!validWaypointIndex(index)) { reject("waypoint index must be 0..3"); return; }
    const Waypoint &wp = waypoints_[static_cast<size_t>(index)];
    if (!wp.saved) { reject("selected waypoint has not been saved"); sendLine("ERR:WP_NOT_SAVED"); return; }
    if (!waypointPoseSane(wp)) {
      reject("saved waypoint contains invalid pose/range"); sendLine("ERR:WP_INVALID_POSE"); return;
    }
    if (mode_ != "AUTO") { reject("waypoint navigation requires AUTO mode"); sendLine("ERR:WP_GO_REQUIRES_AUTO"); return; }
    if (!navigationMotionGateReady()) {
      reject("navigation requires E-stop clear + ESC + Nav2 + fresh motion localization");
      sendLine("ERR:WP_GO_NOT_READY");
      return;
    }
    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = "map";
    goal.pose.position.x = wp.x;
    goal.pose.position.y = wp.y;
    goal.pose.orientation.z = std::sin(wp.yaw * 0.5);
    goal.pose.orientation.w = std::cos(wp.yaw * 0.5);
    goal_pub_->publish(goal);
    selected_waypoint_ = index;
    active_target_ = wp.name;
    navigation_state_ = "QUEUED";
    navigation_origin_ = origin;
    last_goal_state_ = "QUEUED";
    drive_ = "STOP";
    steer_ = "NONE";
    last_rejection_.clear();
    publishState();
    mirrorWaypointState(true);
    RCLCPP_INFO(get_logger(), "Navigate waypoint %d '%s' -> map=(%.3f, %.3f)", index, wp.name.c_str(), wp.x, wp.y);
  }

  void stopNavigation(const std::string &origin) {
    navigation_origin_ = origin;
    // Never block the single-threaded executor waiting for Nav2. Serial CDC and
    // manual-safety timers must keep running even when the cancel service is down.
    if (!cancel_nav_client_->service_is_ready()) {
      reject("Nav2 cancel service unavailable");
      sendLine("ERR:NAV_CANCEL_UNAVAILABLE");
      return;
    }
    auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
    cancel_nav_client_->async_send_request(request,
      [this](rclcpp::Client<action_msgs::srv::CancelGoal>::SharedFuture future) {
        try {
          const auto result = future.get();
          const bool ok = result->return_code == 0 || !result->goals_canceling.empty();
          navigation_state_ = ok ? "STOPPED" : "FAILED";
          last_goal_state_ = ok ? "CANCELED" : "CANCEL_REJECTED";
        } catch (const std::exception &e) {
          navigation_state_ = "FAILED";
          last_goal_state_ = "CANCEL_ERROR";
          RCLCPP_ERROR(get_logger(), "Nav2 cancel failed: %s", e.what());
        }
        publishState();
        mirrorWaypointState(true);
      });
  }

  void handleNavigationGoalState(const std::string &raw) {
    const std::string state = upper(trim(raw));
    last_goal_state_ = state;
    if (state == "ACTIVE") navigation_state_ = "NAVIGATING";
    else if (state == "QUEUED" || state == "QUEUED_MAP" || state == "QUEUED_LOCALIZATION" ||
             state == "QUEUED_NAV2" || state == "SENDING") navigation_state_ = "QUEUED";
    else if (state == "SUCCEEDED") navigation_state_ = "ARRIVED";
    else if (state == "CANCELED") navigation_state_ = "STOPPED";
    else if (state == "ABORTED" || state == "REJECTED" || state == "UNKNOWN") navigation_state_ = "FAILED";
    publishState();
  }

  void parseObstacle(const std::string &raw) {
    obstacle_count_ = std::max(0, static_cast<int>(std::lround(jsonNumber(raw, "count").value_or(0.0))));
    nearest_object_ = "NONE";
    nearest_distance_m_ = 0.0;
    nearest_lateral_m_ = 0.0;
    nearest_conf_pct_ = 0.0;
    nearest_track_id_ = -1;
    nearest_missed_frames_ = 0U;
    size_t cursor = 0;
    while (true) {
      const auto p = raw.find("\"forward_m\":", cursor);
      if (p == std::string::npos) break;
      const auto fragment_start = raw.rfind('{', p);
      const auto fragment_end = raw.find('}', p);
      if (fragment_start == std::string::npos || fragment_end == std::string::npos) break;
      const std::string fragment = raw.substr(fragment_start, fragment_end - fragment_start + 1);
      const double d = jsonNumber(fragment, "forward_m").value_or(0.0);
      if (d > 0.0 && (nearest_distance_m_ <= 0.0 || d < nearest_distance_m_)) {
        nearest_distance_m_ = d;
        nearest_lateral_m_ = jsonNumber(fragment, "left_m").value_or(0.0);
        nearest_track_id_ = static_cast<int>(std::lround(jsonNumber(fragment, "track_id").value_or(-1.0)));
        nearest_missed_frames_ = static_cast<unsigned>(std::max(0.0, jsonNumber(fragment, "missed_frames").value_or(0.0)));
        if (auto name = jsonString(fragment, "class_name")) nearest_object_ = wireToken(*name, 20U);
        else {
          const int cls = static_cast<int>(std::lround(jsonNumber(fragment, "class_id").value_or(-1.0)));
          nearest_object_ = className(cls);
        }
        nearest_conf_pct_ = 100.0 * jsonNumber(fragment, "score").value_or(0.0);
      }
      cursor = fragment_end + 1;
    }
    last_obstacle_time_ = std::chrono::steady_clock::now();
  }

  std::optional<std::string> resolveSerialDevice() {
    const std::string configured = trim(serial_device_);
    if (!configured.empty() && upper(configured) != "AUTO") {
      return fs::exists(configured) ? std::optional<std::string>(configured) : std::nullopt;
    }

    std::vector<std::string> candidates;
    std::error_code ec;
    const fs::path by_id("/dev/serial/by-id");
    if (fs::exists(by_id, ec) && fs::is_directory(by_id, ec)) {
      for (const auto &entry : fs::directory_iterator(by_id, ec)) {
        if (ec) break;
        const std::string name = upper(entry.path().filename().string());
        if (name.find("STMICROELECTRONICS") == std::string::npos ||
            name.find("F411") == std::string::npos ||
            name.find("CDC") == std::string::npos ||
            name.find("BOOT_CDC") != std::string::npos) continue;
        candidates.push_back(entry.path().string());
      }
    }
    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
    if (candidates.size() == 1U) return candidates.front();
    if (candidates.size() > 1U) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "F411 auto-discovery ambiguous: %zu matching CDC devices; set serial_device explicitly", candidates.size());
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "F411 auto-discovery: no STMicroelectronics F411 CDC endpoint under /dev/serial/by-id");
    }
    return std::nullopt;
  }

  static bool parseU32DecimalStrict(const std::string &text, std::uint32_t &out) {
    if (text.empty()) return false;
    for (const char c : text) if (c < '0' || c > '9') return false;
    errno = 0;
    char *end = nullptr;
    const unsigned long value = std::strtoul(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' || value > 0xFFFFFFFFUL) return false;
    out = static_cast<std::uint32_t>(value);
    return true;
  }

  bool neo3CommandAllowed(const std::string &command) const {
    if (command == "STATUS" || command == "CAN:STATUS" || command == "CAN:RECOVER" ||
        command == "BARO:ON" || command == "BARO:OFF" || command == "LED:OFF") return true;
    if (command.rfind("GPS:RATE:", 0) == 0) {
      const std::string raw = command.substr(sizeof("GPS:RATE:") - 1U);
      char *end = nullptr; errno = 0;
      const unsigned long value = std::strtoul(raw.c_str(), &end, 10);
      return errno == 0 && end != raw.c_str() && *end == '\0' && value >= 5UL && value <= 20UL && (1000UL % value) == 0UL;
    }
    if (command.rfind("LED:BRIGHTNESS:", 0) == 0) {
      const std::string raw = command.substr(sizeof("LED:BRIGHTNESS:") - 1U);
      char *end = nullptr; errno = 0;
      const unsigned long value = std::strtoul(raw.c_str(), &end, 10);
      return errno == 0 && end != raw.c_str() && *end == '\0' && value <= 100UL;
    }
    if (command.rfind("LED:", 0) == 0) {
      unsigned r = 0U, g = 0U, b = 0U; char extra = '\0';
      return std::sscanf(command.c_str() + 4, "%u,%u,%u%c", &r, &g, &b, &extra) == 3 &&
             r <= 255U && g <= 255U && b <= 255U;
    }
    return false;
  }

  void resetTransportEpochState(const char *reason, bool publish_invalid) {
    neo3_sequence_initialized_ = false;
    neo3_last_sequence_ = 0U;
    neo3_protocol_version_ = 0;
    for (auto &entry : neo3pro_wire_state_) {
      entry.second.initialized = false;
      entry.second.last_sequence = 0U;
    }
    neo3pro_active_ = false;
    neo3pro_gnss_meta_ = {}; neo3pro_gnss_cov_ = {}; neo3pro_mag_meta_ = {};
    neo3pro_heading_ = {}; neo3pro_node_health_ = {}; neo3pro_gnss_health_ = {};
    neo3pro_timestamp_source_code_ = 3;
    last_neo3_gnss_time_ = {}; last_neo3_mag_time_ = {}; last_neo3pro_baro_time_ = {};
    neo3pro_started_time_ = {};
    mcu_clock_initialized_ = false; mcu_last_raw_ms_ = 0U; mcu_unwrapped_ms_ = 0U;
    mcu_clock_offset_ns_ = 0; last_mcu_stamp_ns_ = 0;
    neo3pro_node_connected_state_ = false;
    neo3pro_gnss_status_connected_state_ = false;
    neo3pro_baro_connected_state_ = false;
    if (!publish_invalid) return;

    std_msgs::msg::Bool b; b.data = false;
    neo3_gnss_connected_pub_->publish(b);
    neo3pro_rm3100_connected_pub_->publish(b);
    neo3pro_node_connected_pub_->publish(b);
    neo3pro_gnss_status_connected_pub_->publish(b);
    neo3pro_baro_connected_pub_->publish(b);
    neo3pro_safety_button_pub_->publish(b);

    std_msgs::msg::String text;
    const std::string why = reason != nullptr ? wireToken(reason, 24U) : "DISCONNECTED";
    text.data = "{\"connected\":false,\"reason\":\"" + why + "\"}";
    neo3_gnss_state_pub_->publish(text);
    neo3pro_node_status_pub_->publish(text);
    neo3pro_gnss_status_pub_->publish(text);
    neo3pro_can_status_pub_->publish(text);
    neo3pro_health_pub_->publish(text);
    neo3pro_dna_status_pub_->publish(text);
  }

  void sendInitialSessionCommands() {
    sendLine("ROS:1");
    sendLine("PING");
    sendLine("GET:STATE");
    sendLine("MODE:" + mode_);
    // Do not mutate peripheral indication state on reconnect. LED commands are
    // explicit operator actions only; this also avoids reconnect-triggered load.
    sendLine("NEO:STATUS");
    if (sendLine("NEO:COMPASS:PARAMS"))
      last_compass_params_request_ = std::chrono::steady_clock::now();
    if (sendLine("NEO:GPS:RATE:" + std::to_string(neo3pro_gnss_rate_hz_)))
      last_neo3_rate_command_ = std::chrono::steady_clock::now();
    sendLine("USB:STATUS");
    last_usb_status_request_ = std::chrono::steady_clock::now();
  }

  bool openSerial() {
    const auto resolved = resolveSerialDevice();
    if (!resolved) return false;
    const std::string device = *resolved;
    const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return false;
    // Kernel-level exclusive ownership prevents another serial monitor/uploader
    // from opening the same CDC endpoint and interleaving bytes with ROS frames.
    if (::ioctl(fd, TIOCEXCL) != 0) {
      RCLCPP_WARN(get_logger(), "Cannot claim exclusive HMI tty ownership: %s", std::strerror(errno));
      ::close(fd);
      return false;
    }
    termios tty{};
    if (::tcgetattr(fd, &tty) != 0) { ::close(fd); return false; }
    ::cfmakeraw(&tty);
    ::cfsetispeed(&tty, B1000000);
    ::cfsetospeed(&tty, B1000000);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    if (::tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return false; }
    ::tcflush(fd, TCIOFLUSH);
    active_serial_device_ = device;
    fd_ = fd;
    rx_.clear();
    tx_cache_.clear();
    // Resynchronize the MCU line parser. If a previous host disappeared after
    // writing only part of a command, this newline closes that stale frame.
    const char resync = '\n';
    const ssize_t resync_written = ::write(fd_, &resync, 1);
    if (resync_written != 1 && errno != EAGAIN && errno != EWOULDBLOCK) {
      ::close(fd_);
      fd_ = -1;
      active_serial_device_.clear();
      return false;
    }
    (void)::tcdrain(fd_);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ::tcflush(fd_, TCIFLUSH);
    last_rx_ = {};
    serial_opened_at_ = std::chrono::steady_clock::now();
    publishConnected(false);
    const auto host_now = std::chrono::steady_clock::now();
    host_session_token_ = static_cast<std::uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(host_now.time_since_epoch()).count());
    if (host_session_token_ == 0U) host_session_token_ = 1U;
    awaiting_host_session_ = true;
    last_host_hello_tx_ = host_now;
    RCLCPP_INFO(get_logger(), "HMI USB opened: %s (selector=%s) session=%u",
                active_serial_device_.c_str(), serial_device_.c_str(), host_session_token_);
    if (!sendLine("HOST:HELLO:" + std::to_string(host_session_token_), 0)) {
      closeSerial("host session hello failed");
      return false;
    }
    return true;
  }

  void closeSerial(const char *reason) {
    if (fd_ >= 0) ::close(fd_);
    const bool was = connected_;
    fd_ = -1;
    active_serial_device_.clear();
    rx_.clear();
    best_effort_lines_.clear();
    awaiting_host_session_ = false;
    host_session_token_ = 0U;
    host_transport_generation_ = 0U;
    resetExtendedTelemetrySession(0U);
    last_host_hello_tx_ = {};
    if (was) RCLCPP_WARN(get_logger(), "HMI USB disconnected: %s", reason);
    publishConnected(false);
    resetTransportEpochState(reason, true);
    drive_ = "STOP";
    steer_ = "NONE";
    last_drive_command_time_ = {};
    last_steer_command_time_ = {};
    control_origin_ = "NONE";
    publishState();
  }

  void reconnectTick() {
    if (fd_ >= 0) return;
    const auto now_steady = std::chrono::steady_clock::now();
    if (now_steady - last_reconnect_try_ < std::chrono::duration<double>(reconnect_sec_)) return;
    last_reconnect_try_ = now_steady;
    (void)openSerial();
  }

  bool sendLine(const std::string &line, int eagain_retry_budget = 3) {
    std::lock_guard<std::mutex> tx_lock(tx_mutex_);
    if (fd_ < 0) return false;
    const std::string packet = line + "\n";
    size_t offset = 0;
    int would_block_retries = 0;
    while (offset < packet.size()) {
      const ssize_t n = ::write(fd_, packet.data() + offset, packet.size() - offset);
      if (n > 0) {
        offset += static_cast<size_t>(n);
        would_block_retries = 0;
        continue;
      }
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        if (++would_block_retries <= eagain_retry_budget) {
          if (eagain_retry_budget > 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        // If no byte was emitted it is safe to drop this refresh and retry on
        // the next timer tick. A partial frame must be closed/re-synchronized.
        if (offset == 0) return false;
        closeSerial("partial HMI write timed out");
        return false;
      }
      closeSerial(n == 0 ? "zero-byte HMI write" : std::strerror(errno));
      return false;
    }
    return true;
  }

  void sendState(const std::string &key, const std::string &value, bool force) {
    const std::string line = key + ":" + value;
    auto it = tx_cache_.find(key);
    if (!force && it != tx_cache_.end() && it->second == value) return;
    if (sendLine(line)) tx_cache_[key] = value;
  }

  void serialTick() {
    if (fd_ < 0) return;
    char buf[256];
    // Bound one executor callback. At 1 kHz, 2 KiB/tick is far above the
    // normal CDC stream but prevents a sensor backlog from monopolizing the
    // single-threaded ROS executor and delaying the 50-Hz actuator callback.
    size_t read_budget = 2048U;
    while (read_budget > 0U) {
      const size_t want = std::min(read_budget, sizeof(buf));
      const ssize_t n = ::read(fd_, buf, want);
      if (n > 0) {
        read_budget -= static_cast<size_t>(n);
        rx_.append(buf, static_cast<size_t>(n));
        if (rx_.size() > 8192U) {
          ++serial_rx_backlog_drops_;
          rx_.erase(0, rx_.size() - 4096U);
        }
      } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        closeSerial(std::strerror(errno));
        return;
      } else break;
    }

    size_t pos = 0U;
    while ((pos = rx_.find('\n')) != std::string::npos) {
      std::string line = trim(rx_.substr(0, pos));
      rx_.erase(0, pos + 1U);
      if (line.empty()) continue;
      if (awaiting_host_session_) {
        const std::string expected = "ACK:HOST:SESSION:" + std::to_string(host_session_token_) + ":";
        if (line.rfind(expected, 0) == 0) {
          std::uint32_t transport_generation = 0U;
          const std::string generation_text = line.substr(expected.size());
          if (!parseU32DecimalStrict(generation_text, transport_generation) || transport_generation == 0U) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "Ignoring malformed F411 host-session ACK for token=%u", host_session_token_);
            continue;
          }
          awaiting_host_session_ = false;
          host_transport_generation_ = transport_generation;
          resetTransportEpochState("NEW_HOST_SESSION", false);
          resetExtendedTelemetrySession(host_session_token_);
          last_rx_ = std::chrono::steady_clock::now();
          silent_open_failures_ = 0U;
          best_effort_lines_.clear();
          publishConnected(true);
          RCLCPP_INFO(get_logger(), "F411 host session synchronized token=%u transport_generation=%u",
                      host_session_token_, host_transport_generation_);
          sendInitialSessionCommands();
        }
        // Ignore every pre-session byte, including stale CDC backlog.
        continue;
      }
      // Safety and direct operator commands bypass best-effort telemetry queues.
      const bool urgent = line.rfind("SENS:SW:", 0) == 0 ||
                          line.rfind("CMD:DRIVE:", 0) == 0 ||
                          line.rfind("CMD:STEER:", 0) == 0 ||
                          line == "CMD:NAV:STOP";
      if (urgent) {
        handleHmiLine(line);
      } else {
        if (best_effort_lines_.size() >= 128U) {
          best_effort_lines_.pop_front();
          ++serial_best_effort_drops_;
        }
        best_effort_lines_.push_back(std::move(line));
      }
    }

    // P3 budget: enough for >8k lines/s at the 1-kHz poll rate, but bounded so
    // navigation/HMI parsing can never create a long control scheduling gap.
    size_t line_budget = 2U;
    while (line_budget-- > 0U && !best_effort_lines_.empty()) {
      std::string line = std::move(best_effort_lines_.front());
      best_effort_lines_.pop_front();
      handleHmiLine(line);
    }
  }

  bool parseCsvNumbers(const std::string &payload, size_t expected_min, std::vector<double> &values) {
    values.clear();
    std::stringstream ss(payload);
    std::string item;
    while (std::getline(ss, item, ',')) {
      item = trim(item);
      if (item.empty()) return false;
      char *end = nullptr;
      errno = 0;
      const double value = std::strtod(item.c_str(), &end);
      if (errno != 0 || end == item.c_str() || *end != '\0' || !std::isfinite(value)) return false;
      values.push_back(value);
    }
    return values.size() >= expected_min;
  }

  static double normalizeAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a <= -kPi) a += 2.0 * kPi;
    return a;
  }

  static std::uint16_t sensorCrc16Ccitt(const std::string &text) {
    std::uint16_t crc = 0xFFFFU;
    for (const unsigned char byte : text) {
      crc ^= static_cast<std::uint16_t>(byte) << 8U;
      for (std::uint8_t bit = 0; bit < 8U; ++bit) {
        crc = (crc & 0x8000U) != 0U ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                                    : static_cast<std::uint16_t>(crc << 1U);
      }
    }
    return crc;
  }

  bool validateNeo3Protocol(
    const std::string &payload, std::string &legacy_payload, int &version, std::uint32_t &sequence)
  {
    legacy_payload.clear(); version = 0; sequence = 0U;
    const auto crc_comma = payload.rfind(',');
    const auto version_comma = crc_comma == std::string::npos ? std::string::npos :
      payload.rfind(',', crc_comma - 1U);
    if (crc_comma == std::string::npos || version_comma == std::string::npos) {
      if (!neo3_require_protocol_crc_) { legacy_payload = payload; return true; }
      ++neo3_crc_errors_;
      return false;
    }
    char *end = nullptr;
    errno = 0;
    const long parsed_version = std::strtol(payload.c_str() + version_comma + 1U, &end, 10);
    if (errno != 0 || end != payload.c_str() + crc_comma || parsed_version != 2L) {
      ++neo3_crc_errors_;
      return false;
    }
    errno = 0;
    char *crc_end = nullptr;
    const unsigned long supplied_crc = std::strtoul(payload.c_str() + crc_comma + 1U, &crc_end, 10);
    if (errno != 0 || crc_end == payload.c_str() + crc_comma + 1U || *crc_end != '\0' || supplied_crc > 0xFFFFUL) {
      ++neo3_crc_errors_;
      return false;
    }
    legacy_payload = payload.substr(0, version_comma);
    const auto calculated = sensorCrc16Ccitt(legacy_payload);
    if (calculated != static_cast<std::uint16_t>(supplied_crc)) {
      ++neo3_crc_errors_;
      return false;
    }
    const auto first_comma = legacy_payload.find(',');
    if (first_comma == std::string::npos) { ++neo3_crc_errors_; return false; }
    errno = 0;
    char *seq_end = nullptr;
    const unsigned long parsed_seq = std::strtoul(legacy_payload.c_str(), &seq_end, 10);
    if (errno != 0 || seq_end != legacy_payload.c_str() + first_comma || parsed_seq > UINT32_MAX) {
      ++neo3_crc_errors_;
      return false;
    }
    sequence = static_cast<std::uint32_t>(parsed_seq);
    version = static_cast<int>(parsed_version);
    if (neo3_sequence_initialized_) {
      const std::uint32_t delta = sequence - neo3_last_sequence_;
      if (delta == 0U) { ++neo3_duplicate_sequences_; return false; }
      if (delta < 0x80000000U && delta > 1U) neo3_sequence_gaps_ += static_cast<std::uint64_t>(delta - 1U);
      // A large unsigned delta is treated as an MCU reboot/sequence reset.
    }
    neo3_last_sequence_ = sequence;
    neo3_sequence_initialized_ = true;
    neo3_protocol_version_ = version;
    return true;
  }

  bool validateNeo3ProProtocol(const std::string &stream, const std::string &payload,
                               std::string &body, int &version, std::uint32_t &sequence) {
    body.clear(); version = 0; sequence = 0U;
    auto &state = neo3pro_wire_state_[stream];
    const auto crc_comma = payload.rfind(',');
    const auto version_comma = crc_comma == std::string::npos ? std::string::npos :
      payload.rfind(',', crc_comma - 1U);
    if (crc_comma == std::string::npos || version_comma == std::string::npos) {
      if (!neo3_require_protocol_crc_) body = payload;
      else { ++state.crc_errors; ++neo3_parse_errors_; return false; }
    } else {
      char *end = nullptr;
      errno = 0;
      const long parsed_version = std::strtol(payload.c_str() + version_comma + 1U, &end, 10);
      if (errno != 0 || end != payload.c_str() + crc_comma || parsed_version != 2L) {
        ++state.crc_errors; ++neo3_parse_errors_; return false;
      }
      errno = 0;
      char *crc_end = nullptr;
      const unsigned long supplied = std::strtoul(payload.c_str() + crc_comma + 1U, &crc_end, 10);
      if (errno != 0 || crc_end == payload.c_str() + crc_comma + 1U || *crc_end != '\0' || supplied > 0xFFFFUL) {
        ++state.crc_errors; ++neo3_parse_errors_; return false;
      }
      body = payload.substr(0, version_comma);
      if (sensorCrc16Ccitt(body) != static_cast<std::uint16_t>(supplied)) {
        ++state.crc_errors; ++neo3_parse_errors_; return false;
      }
      version = static_cast<int>(parsed_version);
    }
    const auto comma = body.find(',');
    if (comma == std::string::npos) { ++neo3_parse_errors_; return false; }
    char *seq_end = nullptr;
    errno = 0;
    const unsigned long raw = std::strtoul(body.c_str(), &seq_end, 10);
    if (errno != 0 || seq_end != body.c_str() + comma || raw > UINT32_MAX) {
      ++neo3_parse_errors_; return false;
    }
    sequence = static_cast<std::uint32_t>(raw);
    if (state.initialized) {
      const std::uint32_t delta = sequence - state.last_sequence;
      if (delta == 0U) { ++state.duplicates; return false; }
      if (delta < 0x80000000U && delta > 1U) state.gaps += static_cast<std::uint64_t>(delta - 1U);
    }
    state.last_sequence = sequence;
    state.initialized = true;
    return true;
  }

  rclcpp::Time stampFromNeo3ProGnss(const Neo3ProGnssMeta &meta, const rclcpp::Time &fallback) {
    neo3pro_timestamp_source_code_ = 3;
    // ArduPilot AP_GPS_DroneCAN only treats Fix2 absolute epoch as authoritative
    // when gnss_time_standard is UTC. Other time standards remain reception-time
    // telemetry unless a separately tested DSDL conversion contract is added.
    if (meta.gnss_timestamp_usec == 0U || meta.time_standard != 2) return fallback;
    if (meta.gnss_timestamp_usec > static_cast<std::uint64_t>(INT64_MAX / 1000LL)) return fallback;
    const std::int64_t stamp_ns = static_cast<std::int64_t>(meta.gnss_timestamp_usec) * 1000LL;
    const std::int64_t host_ns = now().nanoseconds();
    constexpr std::int64_t kMaxClockDisagreementNs = 7LL * 24LL * 3600LL * 1000000000LL;
    const std::int64_t delta = stamp_ns >= host_ns ? stamp_ns - host_ns : host_ns - stamp_ns;
    if (delta > kMaxClockDisagreementNs) return fallback;
    neo3pro_timestamp_source_code_ = 4;
    return rclcpp::Time(stamp_ns, get_clock()->get_clock_type());
  }

  bool neo3ProHealthAllowsFusion() const {
    const auto now_steady = std::chrono::steady_clock::now();
    constexpr auto kHealthTimeout = 3s;
    // DroneCAN NodeStatus: health=0 is OK and mode=0 is OPERATIONAL.  A never-
    // observed/stale status is not evidence of health, so source=4 remains
    // telemetry-only until the peripheral proves it is operational.
    if (!neo3pro_node_health_.seen || neo3pro_node_health_.node_id <= 0 ||
        now_steady - neo3pro_node_health_.received > kHealthTimeout ||
        neo3pro_node_health_.health != 0 || neo3pro_node_health_.mode != 0) return false;
    // ardupilot.gnss.Status is an independent receiver-health contract.
    if (!neo3pro_gnss_health_.seen || neo3pro_gnss_health_.node_id <= 0 ||
        now_steady - neo3pro_gnss_health_.received > kHealthTimeout ||
        !neo3pro_gnss_health_.healthy ||
        (neo3pro_gnss_health_.status_bits & 2U) == 0U ||  // STATUS_ARMABLE
        neo3pro_gnss_health_.node_id != neo3pro_node_health_.node_id) return false;
    return true;
  }

  void publishRm3100Connected(bool connected) {
    std_msgs::msg::Bool b; b.data = connected;
    neo3pro_rm3100_connected_pub_->publish(b);
  }

  void publishNeo3Connected(bool connected) {
    if (neo3_gnss_connected_state_ == connected && neo3_gnss_connected_initialized_) return;
    neo3_gnss_connected_state_ = connected;
    neo3_gnss_connected_initialized_ = true;
    std_msgs::msg::Bool b; b.data = connected;
    neo3_gnss_connected_pub_->publish(b);
  }

  void publishNeo3SafetyState(bool active, bool force = false) {
    if (!force && neo3_switch_initialized_ && neo3_switch_state_ == active) return;
    neo3_switch_state_ = active;
    neo3_switch_initialized_ = true;
    std_msgs::msg::Bool b;
    b.data = active;
    neo3_safety_switch_pub_->publish(b);
    // Legacy NEO3 physical safety input retains its historical E-stop mapping.
    // NEO3 Pro's DroneCAN Button is an arming/safety-button event, NOT an
    // emergency-stop definition. It only drives /safety/estop when explicitly
    // enabled by configuration.
    if (!neo3pro_active_ || neo3_safety_button_as_estop_) {
      estop_ = active;
      neo3_estop_pub_->publish(b);
    }
  }

  rclcpp::Time stampFromMcuMillis(double raw_ms_value) {
    const auto host_now = now();
    if (!std::isfinite(raw_ms_value) || raw_ms_value < 0.0 ||
        raw_ms_value > static_cast<double>(UINT32_MAX)) return host_now;
    const auto raw_ms = static_cast<std::uint32_t>(std::llround(raw_ms_value));
    const std::int64_t host_ns = host_now.nanoseconds();
    std::uint64_t mapped_ms = raw_ms;

    if (!mcu_clock_initialized_) {
      mcu_clock_initialized_ = true;
      mcu_last_raw_ms_ = raw_ms;
      mcu_unwrapped_ms_ = raw_ms;
      mcu_clock_offset_ns_ = host_ns - static_cast<std::int64_t>(mcu_unwrapped_ms_) * 1000000LL;
    } else {
      const auto delta_ms = static_cast<std::int32_t>(raw_ms - mcu_last_raw_ms_);
      if (delta_ms >= 0 && delta_ms <= 3600000) {
        mcu_unwrapped_ms_ += static_cast<std::uint32_t>(delta_ms);
        mcu_last_raw_ms_ = raw_ms;
        mapped_ms = mcu_unwrapped_ms_;
        const std::int64_t candidate = host_ns - static_cast<std::int64_t>(mapped_ms) * 1000000LL;
        if (candidate < mcu_clock_offset_ns_) mcu_clock_offset_ns_ = candidate;
      } else if (delta_ms < 0 && delta_ms >= -5000) {
        mapped_ms = mcu_unwrapped_ms_ - static_cast<std::uint32_t>(-delta_ms);
      } else {
        // F411 reboot atau gap sangat panjang: bentuk epoch baru dari paket ini.
        mcu_last_raw_ms_ = raw_ms;
        mcu_unwrapped_ms_ = raw_ms;
        mapped_ms = raw_ms;
        mcu_clock_offset_ns_ = host_ns - static_cast<std::int64_t>(mapped_ms) * 1000000LL;
      }
    }

    std::int64_t stamp_ns = mcu_clock_offset_ns_ + static_cast<std::int64_t>(mapped_ms) * 1000000LL;
    stamp_ns = std::min(stamp_ns, host_ns);
    if (last_mcu_stamp_ns_ > 0) stamp_ns = std::max(stamp_ns, last_mcu_stamp_ns_);
    last_mcu_stamp_ns_ = stamp_ns;
    return rclcpp::Time(stamp_ns, get_clock()->get_clock_type());
  }

  void publishGnssState(const char *source, bool receiver_valid, int fix_type, int satellites,
                        double hacc_m, double pdop, double pvt_rate_hz, bool rate_usable) {
    std_msgs::msg::String state;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "{\"source\":\"" << source << "\",\"transport\":\"stm32f411_usb_cdc\""
        << ",\"receiver_valid\":" << (receiver_valid ? "true" : "false")
        << ",\"fix_type\":" << fix_type << ",\"satellites\":" << satellites
        << ",\"hacc_m\":" << hacc_m << ",\"dop\":" << pdop
        << ",\"pvt_rate_hz\":" << pvt_rate_hz
        << ",\"min_usable_rate_hz\":" << neo3pro_min_usable_rate_hz_
        << ",\"rate_usable\":" << (rate_usable ? "true" : "false")
        << ",\"protocol_version\":" << neo3_protocol_version_
        << ",\"sequence\":" << neo3_last_sequence_
        << ",\"sequence_gaps\":" << neo3_sequence_gaps_
        << ",\"crc_errors\":" << neo3_crc_errors_
        << ",\"duplicate_sequences\":" << neo3_duplicate_sequences_ << "}";
    state.data = out.str();
    neo3_gnss_state_pub_->publish(state);
  }

  void publishGnssMeasurement(const rclcpp::Time &stamp, int source_id, int fix_type,
                              bool receiver_valid, int satellites, double lat, double lon,
                              double alt, double hacc, double vacc, double vel_n,
                              double vel_e, double vel_d, double ground_speed,
                              double course_ned_deg, double sacc, double head_acc_deg,
                              double pdop, double itow_ms, double pvt_rate_hz,
                              double flags2, double flags3, bool velocity_valid, bool pro_enriched = false) {
    // A receiver can be fully connected and streaming NAV-PVT indoors while it
    // has no usable LLH yet. Keep quality/state telemetry alive in that case;
    // only position/velocity fusion measurements are suppressed.
    const bool coordinates_valid = std::isfinite(lat) && std::isfinite(lon) &&
      std::abs(lat) <= 90.0 && std::abs(lon) <= 180.0 &&
      !(std::abs(lat) < 1.0e-12 && std::abs(lon) < 1.0e-12);
    const bool rate_usable = std::isfinite(pvt_rate_hz) && pvt_rate_hz > neo3pro_min_usable_rate_hz_;
    const bool qualified_fix = receiver_valid && coordinates_valid && rate_usable;
    const bool pro_source = source_id == 4;
    last_neo3_pvt_rate_hz_ = pvt_rate_hz;
    const bool pro_sample = pro_source && pro_enriched && neo3pro_gnss_meta_.valid;
    const bool pro_cov_match = pro_sample && neo3pro_gnss_cov_.valid &&
      neo3pro_gnss_cov_.mcu_ms == neo3pro_gnss_meta_.mcu_ms &&
      neo3pro_gnss_cov_.covariance.size() >= 6U;
    const double h_sigma = std::clamp(std::isfinite(hacc) && hacc > 0.0 ? hacc : 100.0, 0.02, 1000.0);
    const double v_sigma = std::clamp(std::isfinite(vacc) && vacc > 0.0 ? vacc : h_sigma * 1.5, 0.03, 1500.0);
    const auto valid_dop = [](double value) { return std::isfinite(value) && value > 0.0; };
    // Horizontal navigation quality should use HDOP when AP_Periph provides it.
    // Its Auxiliary message may leave PDOP at zero, which means unavailable,
    // not perfect geometry.  Keep /gnss/quality[1] transport-invariant as the
    // effective DOP consumed by localization quality gates.
    double effective_dop = valid_dop(pdop) ? pdop : std::numeric_limits<double>::quiet_NaN();
    if (pro_sample) {
      if (valid_dop(neo3pro_gnss_meta_.hdop)) effective_dop = neo3pro_gnss_meta_.hdop;
      else if (valid_dop(neo3pro_gnss_meta_.pdop)) effective_dop = neo3pro_gnss_meta_.pdop;
    }

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = neo3_gnss_frame_id_;
    fix.status.status = qualified_fix ? sensor_msgs::msg::NavSatStatus::STATUS_FIX :
                                       sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix.status.service = pro_source ? neo3pro_navsat_service_mask_ : sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    if (coordinates_valid) {
      fix.latitude = lat;
      fix.longitude = lon;
      const double ros_alt = (pro_sample && std::isfinite(neo3pro_gnss_meta_.height_ellipsoid_m)) ? neo3pro_gnss_meta_.height_ellipsoid_m : alt;
      fix.altitude = std::isfinite(ros_alt) ? ros_alt : 0.0;
      fix.position_covariance.fill(0.0);
      if (pro_cov_match && neo3pro_gnss_cov_.covariance[0] >= 0.0 &&
          neo3pro_gnss_cov_.covariance[1] >= 0.0 && neo3pro_gnss_cov_.covariance[2] >= 0.0) {
        fix.position_covariance[0] = neo3pro_gnss_cov_.covariance[1]; // East
        fix.position_covariance[4] = neo3pro_gnss_cov_.covariance[0]; // North
        fix.position_covariance[8] = neo3pro_gnss_cov_.covariance[2]; // Up/Down variance
      } else {
        fix.position_covariance[0] = h_sigma * h_sigma;
        fix.position_covariance[4] = h_sigma * h_sigma;
        fix.position_covariance[8] = v_sigma * v_sigma;
      }
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      // Raw fix remains visible for diagnostics even below the rate gate. Only
      // the qualified /gnss/fix stream is allowed downstream when rate >= gate.
      neo3_fix_raw_pub_->publish(fix);
      if (qualified_fix) neo3_fix_pub_->publish(fix);
    }

    const double course_ned_rad = course_ned_deg * kPi / 180.0;
    const double course_enu_rad = normalizeAngle(0.5 * kPi - course_ned_rad);
    if (qualified_fix && velocity_valid && std::isfinite(vel_n) && std::isfinite(vel_e) && std::isfinite(vel_d)) {
      geometry_msgs::msg::TwistWithCovarianceStamped vel;
      vel.header = fix.header;
      vel.twist.twist.linear.x = vel_e;
      vel.twist.twist.linear.y = vel_n;
      vel.twist.twist.linear.z = -vel_d;  // UBX NED Down -> ROS ENU Up
      const double sigma = std::clamp(std::isfinite(sacc) && sacc >= 0.0 ? sacc : 5.0, 0.01, 10.0);
      vel.twist.covariance.fill(0.0);
      if (pro_cov_match && neo3pro_gnss_cov_.covariance[3] >= 0.0 &&
          neo3pro_gnss_cov_.covariance[4] >= 0.0 && neo3pro_gnss_cov_.covariance[5] >= 0.0) {
        vel.twist.covariance[0] = neo3pro_gnss_cov_.covariance[4]; // East
        vel.twist.covariance[7] = neo3pro_gnss_cov_.covariance[3]; // North
        vel.twist.covariance[14] = neo3pro_gnss_cov_.covariance[5];
      } else {
        vel.twist.covariance[0] = sigma * sigma;
        vel.twist.covariance[7] = sigma * sigma;
        vel.twist.covariance[14] = sigma * sigma * 2.0;
      }
      vel.twist.covariance[21] = 1.0e6;
      vel.twist.covariance[28] = 1.0e6;
      vel.twist.covariance[35] = 1.0e6;
      neo3_vel_pub_->publish(vel);
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    std_msgs::msg::Float64MultiArray quality;
    quality.data.assign(50, nan);
    quality.data[0] = static_cast<double>(satellites);
    quality.data[1] = effective_dop;
    quality.data[2] = h_sigma;
    quality.data[3] = static_cast<double>(fix_type);
    quality.data[4] = static_cast<double>(source_id);
    quality.data[5] = sacc;
    quality.data[6] = ground_speed;
    quality.data[7] = course_enu_rad;
    quality.data[8] = head_acc_deg * kPi / 180.0;
    quality.data[9] = itow_ms;
    quality.data[10] = v_sigma;
    quality.data[11] = vel_e;
    quality.data[12] = vel_n;
    quality.data[13] = vel_d;
    // Canonical /gnss/quality indices 0..44 are transport invariant. The compact
    // F411 frame does not carry UBX NAV-COV, so both NAV-COV validity bits must be
    // false here. The Twist covariance derived from NAV-PVT sAcc remains valid and
    // is advertised separately in the F411-only append extension at index 49.
    quality.data[20] = pro_cov_match && neo3pro_gnss_meta_.pos_cov_valid ? 1.0 : 0.0;
    quality.data[21] = pro_cov_match && neo3pro_gnss_meta_.vel_cov_valid ? 1.0 : 0.0;
    quality.data[22] = pvt_rate_hz;
    quality.data[23] = std::max(0.0, (now() - stamp).seconds());
    quality.data[24] = pro_sample ? static_cast<double>(neo3pro_timestamp_source_code_) : 3.0;
    quality.data[25] = pro_source ? nan : flags2;
    quality.data[26] = pro_source ? nan : flags3;
    if (pro_sample) {
      const auto &m = neo3pro_gnss_meta_;
      quality.data[14] = valid_dop(m.gdop) ? m.gdop : nan;
      quality.data[15] = valid_dop(m.hdop) ? m.hdop : nan;
      quality.data[16] = valid_dop(m.vdop) ? m.vdop : nan;
      quality.data[17] = valid_dop(m.ndop) ? m.ndop : nan;
      quality.data[18] = valid_dop(m.edop) ? m.edop : nan;
      quality.data[19] = valid_dop(m.tdop) ? m.tdop : nan;
      quality.data[29] = pro_cov_match ? itow_ms : nan;
      quality.data[30] = pro_cov_match ? 1.0 : 0.0;
      quality.data[33] = m.height_ellipsoid_m;
      if (neo3pro_heading_.seen && neo3pro_heading_.valid &&
          neo3pro_heading_.node_id == m.node_id &&
          std::chrono::steady_clock::now() - neo3pro_heading_.received <= 2s) {
        quality.data[34] = normalizeAngle(0.5 * kPi - neo3pro_heading_.heading_rad);
        quality.data[42] = 1.0;
      } else {
        quality.data[34] = nan;
        quality.data[42] = 0.0;
      }
      quality.data[37] = (m.mode == 1 || m.mode == 2) ? 1.0 : 0.0;
      quality.data[38] = m.mode == 2 ? (m.sub_mode == 1 ? 2.0 : 1.0) : 0.0;
    }
    quality.data[44] = qualified_fix ? 1.0 : 0.0;
    // 45..48 protocol diagnostics, append-only to preserve existing consumers.
    quality.data[45] = static_cast<double>(neo3_protocol_version_);
    quality.data[46] = static_cast<double>(neo3_last_sequence_);
    quality.data[47] = static_cast<double>(neo3_sequence_gaps_);
    quality.data[48] = static_cast<double>(neo3_crc_errors_ + neo3_duplicate_sequences_);
    quality.data[49] = pro_sample ? (pro_cov_match && neo3pro_gnss_meta_.vel_cov_valid ? 1.0 : 0.0) :
                                      (velocity_valid ? 1.0 : 0.0);
    neo3_quality_pub_->publish(quality);

    last_neo3_gnss_time_ = std::chrono::steady_clock::now();
    publishNeo3Connected(true);
    const char *source_name = source_id == 4 ? "STM32_DRONECAN_NEO3PRO" :
      (source_id == 1 ? "STM32_UBX_NAV_PVT" : "STM32_NMEA_FALLBACK");
    publishGnssState(source_name, qualified_fix, fix_type, satellites, h_sigma, effective_dop,
                     pvt_rate_hz, rate_usable);
  }

  void handleNeo3Gnss(const std::string &payload) {
    if (!publish_stm32_gnss_) return;
    std::string legacy_payload; int protocol_version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3Protocol(payload, legacy_payload, protocol_version, sequence)) {
      ++neo3_parse_errors_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejected SENS:GNSS protocol/CRC/sequence frame (crc=%llu dup=%llu gaps=%llu)",
        static_cast<unsigned long long>(neo3_crc_errors_),
        static_cast<unsigned long long>(neo3_duplicate_sequences_),
        static_cast<unsigned long long>(neo3_sequence_gaps_));
      return;
    }
    std::vector<double> v;
    if (!parseCsvNumbers(legacy_payload, 23, v)) {
      ++neo3_parse_errors_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Malformed SENS:GNSS frame");
      return;
    }
    const int fix_type = static_cast<int>(std::lround(v[3]));
    const bool fix_ok = v[4] > 0.5;
    const bool invalid_llh = v[5] > 0.5;
    const std::uint32_t sample_mcu_ms = static_cast<std::uint32_t>(std::llround(v[1]));
    const bool pro_profile = neo3pro_active_;
    const bool pro_meta_match = pro_profile && neo3pro_gnss_meta_.valid &&
      neo3pro_gnss_meta_.mcu_ms == sample_mcu_ms;
    bool receiver_valid = fix_ok && !invalid_llh &&
      (pro_profile ? fix_type == 3 : (fix_type == 3 || fix_type == 4));
    if (pro_profile) receiver_valid = receiver_valid && neo3ProHealthAllowsFusion();
    const auto mcu_stamp = stampFromMcuMillis(v[1]);
    const auto measurement_stamp = pro_meta_match ? stampFromNeo3ProGnss(neo3pro_gnss_meta_, mcu_stamp) : mcu_stamp;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    publishGnssMeasurement(measurement_stamp, pro_profile ? 4 : 1, fix_type, receiver_valid,
      static_cast<int>(std::lround(v[6])), v[7], v[8], v[9], v[10], v[11],
      v[12], v[13], v[14], v[15], v[16], v[17], v[18], v[19], v[2], v[20],
      pro_profile ? nan : v[21], pro_profile ? nan : v[22], receiver_valid, pro_meta_match);
  }

  void handleNeo3GnssFallback(const std::string &payload) {
    if (!publish_stm32_gnss_) return;
    std::string legacy_payload; int protocol_version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3Protocol(payload, legacy_payload, protocol_version, sequence)) {
      ++neo3_parse_errors_;
      return;
    }
    std::vector<double> v;
    if (!parseCsvNumbers(legacy_payload, 10, v)) {
      ++neo3_parse_errors_;
      return;
    }
    const int nmea_fix = static_cast<int>(std::lround(v[2]));
    const int sats = static_cast<int>(std::lround(v[3]));
    const double hdop = v[7];
    const double speed = v[8];
    const double course_deg = v[9];
    const bool receiver_valid = nmea_fix > 0 && sats >= 3 && std::isfinite(hdop) && hdop > 0.0;
    const bool velocity_valid = receiver_valid && speed >= 0.0 && course_deg >= 0.0;
    const double theta = course_deg * kPi / 180.0;
    const double vel_n = velocity_valid ? speed * std::cos(theta) : 0.0;
    const double vel_e = velocity_valid ? speed * std::sin(theta) : 0.0;
    const double hacc = std::clamp(hdop * 2.5 / 1.1774, 0.5, 100.0);
    // source=3 deliberately prevents fallback NMEA velocity/COG from entering the
    // strict Doppler fusion gate; it remains useful for position and HMI display.
    const auto measurement_stamp = stampFromMcuMillis(v[1]);
    publishGnssMeasurement(measurement_stamp, 3, nmea_fix, receiver_valid, sats, v[4], v[5], v[6],
      hacc, hacc * 1.5, vel_n, vel_e, 0.0, speed, course_deg, 5.0, 90.0,
      hdop, 0.0, 1.0, 0.0, 0.0, velocity_valid);
  }

  void markNeo3ProActive() {
    if (!neo3pro_active_) {
      neo3pro_active_ = true;
      neo3pro_started_time_ = std::chrono::steady_clock::now();
    }
  }

  void handleNeo3ProGnssMeta(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("GNSSPRO", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 24U, v) || v.size() != 24U) { ++neo3_parse_errors_; return; }
    Neo3ProGnssMeta m{};
    m.valid = true;
    m.mcu_ms = static_cast<std::uint32_t>(std::llround(v[1]));
    m.node_id = std::clamp(static_cast<int>(std::lround(v[2])), 0, 127);
    m.network_timestamp_usec = static_cast<std::uint64_t>(std::max(0.0, v[3]));
    m.gnss_timestamp_usec = static_cast<std::uint64_t>(std::max(0.0, v[4]));
    m.time_standard = std::clamp(static_cast<int>(std::lround(v[5])), 0, 3);
    m.leap_seconds = std::clamp(static_cast<int>(std::lround(v[6])), 0, 255);
    m.status = std::clamp(static_cast<int>(std::lround(v[7])), 0, 3);
    m.mode = std::clamp(static_cast<int>(std::lround(v[8])), 0, 15);
    m.sub_mode = std::clamp(static_cast<int>(std::lround(v[9])), 0, 63);
    m.sats_used = std::clamp(static_cast<int>(std::lround(v[10])), 0, 63);
    m.sats_visible = std::clamp(static_cast<int>(std::lround(v[11])), 0, 127);
    m.height_msl_m = v[12] * 1.0e-3;
    m.height_ellipsoid_m = v[13] * 1.0e-3;
    m.gdop = v[14]; m.pdop = v[15]; m.hdop = v[16]; m.vdop = v[17];
    m.tdop = v[18]; m.ndop = v[19]; m.edop = v[20]; m.rate_hz = std::max(0.0, v[21]);
    m.pos_cov_valid = v[22] > 0.5; m.vel_cov_valid = v[23] > 0.5;
    m.received = std::chrono::steady_clock::now();
    neo3pro_gnss_meta_ = m;
    markNeo3ProActive();
    std_msgs::msg::Float64MultiArray out;
    // [seq,mcu_ms,node,network_us,gnss_us,time_std,leap,status,mode,submode,
    //  sats_used,sats_visible,h_msl_m,h_ellipsoid_m,gdop,pdop,hdop,vdop,tdop,ndop,edop,rate,posCov,velCov]
    out.data = {static_cast<double>(sequence), v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9],
                v[10], v[11], m.height_msl_m, m.height_ellipsoid_m, v[14], v[15], v[16], v[17],
                v[18], v[19], v[20], v[21], v[22], v[23]};
    neo3pro_gnss_meta_pub_->publish(out);
    const auto reception_stamp = stampFromMcuMillis(v[1]);
    const auto absolute_stamp = stampFromNeo3ProGnss(neo3pro_gnss_meta_, reception_stamp);
    if (neo3pro_timestamp_source_code_ == 4) {
      sensor_msgs::msg::TimeReference tref;
      tref.header.stamp = reception_stamp;
      tref.header.frame_id = neo3_gnss_frame_id_;
      tref.time_ref = absolute_stamp;
      tref.source = "NEO3PRO_DRONECAN";
      neo3pro_time_reference_pub_->publish(tref);
    }
  }

  void handleNeo3ProGnssCov(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("GNSSCOV", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 4U, v)) { ++neo3_parse_errors_; return; }
    const int len = static_cast<int>(std::lround(v[3]));
    if (len < 0 || len > 36 || v.size() != static_cast<size_t>(4 + len)) { ++neo3_parse_errors_; return; }
    Neo3ProCovariance c{};
    c.valid = true;
    c.mcu_ms = static_cast<std::uint32_t>(std::llround(v[1]));
    c.node_id = std::clamp(static_cast<int>(std::lround(v[2])), 0, 127);
    c.covariance.assign(v.begin() + 4, v.end());
    c.received = std::chrono::steady_clock::now();
    neo3pro_gnss_cov_ = c;
    std_msgs::msg::Float64MultiArray out;
    out.data.reserve(static_cast<size_t>(4 + len));
    out.data = {static_cast<double>(sequence), v[1], v[2], static_cast<double>(len)};
    out.data.insert(out.data.end(), c.covariance.begin(), c.covariance.end());
    neo3pro_gnss_cov_pub_->publish(out);
  }

  void handleNeo3ProEcef(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("ECEF", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 10U, v)) { ++neo3_parse_errors_; return; }
    const int len = static_cast<int>(std::lround(v[9]));
    if (len < 0 || len > 36 || v.size() != static_cast<size_t>(10 + len)) { ++neo3_parse_errors_; return; }
    std_msgs::msg::Float64MultiArray out;
    // SI layout: seq,mcu_ms,node,pos_xyz_m,vel_xyz_mps,cov_len,cov...
    out.data = {static_cast<double>(sequence), v[1], v[2], v[6] * 1.0e-3, v[7] * 1.0e-3,
                v[8] * 1.0e-3, v[3], v[4], v[5], static_cast<double>(len)};
    out.data.insert(out.data.end(), v.begin() + 10, v.end());
    neo3pro_ecef_pub_->publish(out);
  }

  void handleNeo3ProMag(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("MAGPRO", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 9U, v)) { ++neo3_parse_errors_; return; }
    const int len = static_cast<int>(std::lround(v[8]));
    if (len < 0 || len > 9 || v.size() != static_cast<size_t>(9 + len)) { ++neo3_parse_errors_; return; }
    Neo3ProMagMeta m{};
    m.valid = true; m.mcu_ms = static_cast<std::uint32_t>(std::llround(v[1]));
    m.node_id = std::clamp(static_cast<int>(std::lround(v[2])), 0, 127);
    m.message_type = static_cast<int>(std::lround(v[3])); m.sensor_id = static_cast<int>(std::lround(v[4]));
    m.x_ut = v[5]; m.y_ut = v[6]; m.z_ut = v[7];
    m.covariance_t2.assign(v.begin() + 9, v.end());
    m.received = std::chrono::steady_clock::now();
    neo3pro_mag_meta_ = m; neo3pro_active_ = true;

    sensor_msgs::msg::MagneticField mag;
    mag.header.stamp = stampFromMcuMillis(v[1]); mag.header.frame_id = rm3100_mag_frame_id_;
    mag.magnetic_field.x = m.x_ut * 1.0e-6; mag.magnetic_field.y = m.y_ut * 1.0e-6; mag.magnetic_field.z = m.z_ut * 1.0e-6;
    mag.magnetic_field_covariance.fill(0.0);
    bool actual_cov = len == 9;
    if (actual_cov) {
      // A covariance matrix may legitimately contain negative off-diagonal
      // correlations. Validate finiteness, positive variances, symmetry and a
      // conservative PSD condition instead of rejecting every negative value.
      const auto &c = m.covariance_t2;
      for (double value : c) actual_cov = actual_cov && std::isfinite(value);
      if (actual_cov) {
        actual_cov = c[0] > 0.0 && c[4] > 0.0 && c[8] > 0.0;
        const double scale = std::max({c[0], c[4], c[8], 1.0e-24});
        const double sym_tol = std::max(1.0e-18, scale * 1.0e-3);
        actual_cov = actual_cov &&
          std::abs(c[1] - c[3]) <= sym_tol &&
          std::abs(c[2] - c[6]) <= sym_tol &&
          std::abs(c[5] - c[7]) <= sym_tol;
        const double psd_tol = std::max(1.0e-36, scale * scale * 1.0e-6);
        actual_cov = actual_cov &&
          c[1] * c[1] <= c[0] * c[4] + psd_tol &&
          c[2] * c[2] <= c[0] * c[8] + psd_tol &&
          c[5] * c[5] <= c[4] * c[8] + psd_tol;
        const double det = c[0] * (c[4] * c[8] - c[5] * c[7]) -
                           c[1] * (c[3] * c[8] - c[5] * c[6]) +
                           c[2] * (c[3] * c[7] - c[4] * c[6]);
        const double det_tol = std::max(1.0e-54, scale * scale * scale * 1.0e-6);
        actual_cov = actual_cov && det >= -det_tol;
      }
    }
    if (actual_cov) {
      for (int i = 0; i < 9; ++i) mag.magnetic_field_covariance[static_cast<size_t>(i)] = m.covariance_t2[static_cast<size_t>(i)];
    } else {
      const double sigma_t = rm3100_mag_sigma_ut_ * 1.0e-6;
      mag.magnetic_field_covariance[0] = sigma_t * sigma_t;
      mag.magnetic_field_covariance[4] = sigma_t * sigma_t;
      mag.magnetic_field_covariance[8] = sigma_t * sigma_t;
    }
    neo3pro_mag_pub_->publish(mag);
    last_neo3_mag_time_ = std::chrono::steady_clock::now(); publishRm3100Connected(true);
    std_msgs::msg::Float64MultiArray meta; meta.data = v; neo3pro_mag_meta_pub_->publish(meta);
  }

  void handleNeo3ProPressure(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("BARO", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 5U, v) || v.size() != 5U || v[3] <= 0.0) { ++neo3_parse_errors_; return; }
    markNeo3ProActive();
    sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = stampFromMcuMillis(v[1]); msg.header.frame_id = neo3pro_baro_frame_id_;
    msg.fluid_pressure = v[3]; msg.variance = v[4] >= 0.0 ? v[4] : 0.0;
    neo3pro_pressure_pub_->publish(msg);
    last_neo3pro_baro_time_ = std::chrono::steady_clock::now();
    if (!neo3pro_baro_connected_state_) { neo3pro_baro_connected_state_=true; std_msgs::msg::Bool b; b.data=true; neo3pro_baro_connected_pub_->publish(b); }
  }

  void handleNeo3ProTemperature(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("TEMP", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 5U, v) || v.size() != 5U || v[3] <= 0.0) { ++neo3_parse_errors_; return; }
    markNeo3ProActive();
    sensor_msgs::msg::Temperature msg;
    msg.header.stamp = stampFromMcuMillis(v[1]); msg.header.frame_id = neo3pro_baro_frame_id_;
    msg.temperature = v[3] - 273.15; // DroneCAN Kelvin -> ROS Temperature Celsius
    msg.variance = v[4] >= 0.0 ? v[4] : 0.0; // K^2 and C^2 have identical scale
    neo3pro_temperature_pub_->publish(msg);
    last_neo3pro_baro_time_ = std::chrono::steady_clock::now();
    if (!neo3pro_baro_connected_state_) { neo3pro_baro_connected_state_=true; std_msgs::msg::Bool b; b.data=true; neo3pro_baro_connected_pub_->publish(b); }
  }

  void handleNeo3ProTextDiagnostic(const std::string &stream, const std::string &payload,
                                  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr &pub) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol(stream, payload, body, version, sequence)) return;
    std_msgs::msg::String msg; msg.data = body; pub->publish(msg);
    markNeo3ProActive();
  }

  void handleNeo3ProNode(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("NODE", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 8U, v) || v.size() != 8U) { ++neo3_parse_errors_; return; }
    markNeo3ProActive();
    neo3pro_node_health_.seen = true; neo3pro_node_health_.mcu_ms = static_cast<std::uint32_t>(std::llround(v[1]));
    neo3pro_node_health_.node_id = static_cast<int>(std::lround(v[2]));
    neo3pro_node_health_.health = static_cast<int>(std::lround(v[4]));
    neo3pro_node_health_.mode = static_cast<int>(std::lround(v[5]));
    neo3pro_node_health_.sub_mode = static_cast<int>(std::lround(v[6]));
    neo3pro_node_health_.received = std::chrono::steady_clock::now();
    if (!neo3pro_node_connected_state_) { neo3pro_node_connected_state_=true; std_msgs::msg::Bool b; b.data=true; neo3pro_node_connected_pub_->publish(b); }
    std_msgs::msg::String msg; std::ostringstream o;
    o << "{\"node_id\":" << neo3pro_node_health_.node_id << ",\"uptime_sec\":" << static_cast<std::uint64_t>(v[3])
      << ",\"health\":" << neo3pro_node_health_.health << ",\"mode\":" << neo3pro_node_health_.mode
      << ",\"sub_mode\":" << neo3pro_node_health_.sub_mode << ",\"vendor_status\":" << static_cast<int>(std::lround(v[7])) << "}";
    msg.data = o.str(); neo3pro_node_status_pub_->publish(msg);
  }

  void handleNeo3ProGnssStatus(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("GNSSSTAT", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 6U, v) || v.size() != 6U) { ++neo3_parse_errors_; return; }
    markNeo3ProActive();
    neo3pro_gnss_health_.seen = true; neo3pro_gnss_health_.mcu_ms = static_cast<std::uint32_t>(std::llround(v[1]));
    neo3pro_gnss_health_.node_id = static_cast<int>(std::lround(v[2])); neo3pro_gnss_health_.healthy = v[3] > 0.5;
    neo3pro_gnss_health_.error_codes = static_cast<std::uint64_t>(std::max(0.0, v[4]));
    neo3pro_gnss_health_.status_bits = static_cast<std::uint64_t>(std::max(0.0, v[5]));
    neo3pro_gnss_health_.received = std::chrono::steady_clock::now();
    if (!neo3pro_gnss_status_connected_state_) { neo3pro_gnss_status_connected_state_=true; std_msgs::msg::Bool b; b.data=true; neo3pro_gnss_status_connected_pub_->publish(b); }
    std_msgs::msg::String msg; std::ostringstream o;
    o << "{\"node_id\":" << neo3pro_gnss_health_.node_id << ",\"healthy\":" << (neo3pro_gnss_health_.healthy ? "true" : "false")
      << ",\"error_codes\":" << neo3pro_gnss_health_.error_codes << ",\"status_bits\":" << neo3pro_gnss_health_.status_bits
      << ",\"logging\":" << ((neo3pro_gnss_health_.status_bits & 1U) ? "true" : "false")
      << ",\"armable\":" << ((neo3pro_gnss_health_.status_bits & 2U) ? "true" : "false") << "}";
    msg.data = o.str(); neo3pro_gnss_status_pub_->publish(msg);
  }

  void handleNeo3ProHeading(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("GNSSHEAD", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 7U, v) || v.size() != 7U) { ++neo3_parse_errors_; return; }
    markNeo3ProActive();
    neo3pro_heading_.seen = true; neo3pro_heading_.mcu_ms = static_cast<std::uint32_t>(std::llround(v[1]));
    neo3pro_heading_.node_id = static_cast<int>(std::lround(v[2])); neo3pro_heading_.valid = v[3] > 0.5;
    neo3pro_heading_.accuracy_valid = v[4] > 0.5; neo3pro_heading_.heading_rad = v[5]; neo3pro_heading_.accuracy_rad = v[6];
    neo3pro_heading_.received = std::chrono::steady_clock::now();
    std_msgs::msg::Float64 h; h.data = neo3pro_heading_.valid ? v[5] : std::numeric_limits<double>::quiet_NaN(); neo3pro_heading_pub_->publish(h);
    h.data = neo3pro_heading_.accuracy_valid ? v[6] : std::numeric_limits<double>::quiet_NaN(); neo3pro_heading_accuracy_pub_->publish(h);
  }

  void handleNeo3ProButton(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("BUTTON", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 6U, v) || v.size() != 6U) { ++neo3_parse_errors_; return; }
    markNeo3ProActive();
    const bool pressed = v[5] > 0.5;
    std_msgs::msg::Bool b; b.data = pressed; neo3pro_safety_button_pub_->publish(b);
    std_msgs::msg::UInt8MultiArray raw;
    raw.data = {static_cast<std::uint8_t>(std::clamp(static_cast<int>(std::lround(v[2])), 0, 127)),
                static_cast<std::uint8_t>(std::clamp(static_cast<int>(std::lround(v[3])), 0, 255)),
                static_cast<std::uint8_t>(std::clamp(static_cast<int>(std::lround(v[4])), 0, 255)),
                static_cast<std::uint8_t>(pressed ? 1U : 0U)};
    neo3pro_safety_button_raw_pub_->publish(raw);
    publishNeo3SafetyState(pressed);
  }

  void handleNeo3ProHardware(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("HWPRO", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 17U, v)) { ++neo3_parse_errors_; return; }
    markNeo3ProActive();
    std_msgs::msg::String msg; std::ostringstream o;
    o << "{\"can_ok\":" << (v[2] > 0.5 ? "true" : "false") << ",\"oscillator_mhz\":" << static_cast<int>(std::lround(v[3]))
      << ",\"node_id\":" << static_cast<int>(std::lround(v[4])) << ",\"raw_frames\":" << static_cast<std::uint64_t>(v[5])
      << ",\"dronecan_transfers\":" << static_cast<std::uint64_t>(v[6]) << ",\"accepted_transfers\":" << static_cast<std::uint64_t>(v[7])
      << ",\"foreign_node_drops\":" << static_cast<std::uint64_t>(v[8]) << ",\"spi_errors\":" << static_cast<std::uint64_t>(v[9])
      << ",\"decode_errors\":" << static_cast<std::uint64_t>(v[10]) << ",\"can_overflows\":" << static_cast<std::uint64_t>(v[11])
      << ",\"recoveries\":" << static_cast<std::uint64_t>(v[12]) << ",\"spi_contentions\":" << static_cast<std::uint64_t>(v[13])
      << ",\"tec\":" << static_cast<int>(std::lround(v[14])) << ",\"rec\":" << static_cast<int>(std::lround(v[15]))
      << ",\"eflg\":" << static_cast<int>(std::lround(v[16]));
    if (v.size() >= 19U) o << ",\"sensor_usb_drops\":" << static_cast<std::uint64_t>(v[17]) << ",\"usb_tx_drops\":" << static_cast<std::uint64_t>(v[18]);
    const auto &w = neo3pro_wire_state_["HWPRO"];
    o << ",\"wire_gaps\":" << w.gaps << ",\"wire_duplicates\":" << w.duplicates << ",\"wire_crc_errors\":" << w.crc_errors << "}";
    msg.data = o.str(); neo3pro_can_status_pub_->publish(msg);
  }

  void handleNeo3ProCanRaw(const std::string &payload) {
    std::string body; int version = 0; std::uint32_t sequence = 0U;
    if (!validateNeo3ProProtocol("CANRAW", payload, body, version, sequence)) return;
    std::vector<double> v;
    if (!parseCsvNumbers(body, 12U, v)) { ++neo3_parse_errors_; return; }
    markNeo3ProActive();
    std_msgs::msg::String msg; std::ostringstream o;
    o << "{\"raw_frames\":" << static_cast<std::uint64_t>(v[0])
      << ",\"raw_allocation_frames\":" << static_cast<std::uint64_t>(v[1])
      << ",\"raw_fix2_frames\":" << static_cast<std::uint64_t>(v[2])
      << ",\"raw_node_frames\":" << static_cast<std::uint64_t>(v[3])
      << ",\"raw_mag_frames\":" << static_cast<std::uint64_t>(v[4])
      << ",\"raw_other_frames\":" << static_cast<std::uint64_t>(v[5])
      << ",\"oscillator_mhz\":" << static_cast<int>(std::lround(v[6]))
      << ",\"oscillator_locked\":" << (v[7] > 0.5 ? "true" : "false")
      << ",\"host_node_id\":" << static_cast<int>(std::lround(v[8]))
      << ",\"allocated_node_id\":" << static_cast<int>(std::lround(v[9]))
      << ",\"can_tx_frames\":" << static_cast<std::uint64_t>(v[10])
      << ",\"can_tx_errors\":" << static_cast<std::uint64_t>(v[11]) << "}";
    msg.data = o.str();
    neo3pro_can_raw_pub_->publish(msg);
  }

  void handleNeo3Hardware(const std::string &payload) {
    if (neo3pro_active_) return;  // HWPRO is authoritative for NEO3 Pro.
    std::vector<double> v;
    if (!parseCsvNumbers(payload, 8, v)) {
      ++neo3_parse_errors_;
      return;
    }
    const bool gnss_alive = v[2] > 0.5;
    const bool gnss_ready = v[3] > 0.5;
    const bool sw = v[5] > 0.5;
    const bool led = v[6] > 0.5;
    // Host freshness of actual GNSS measurement frames is authoritative.
    // A 1-Hz hardware snapshot may briefly report not-alive between UART bursts;
    // never let that low-rate diagnostic flap /gnss/connected false.
    if (publish_stm32_gnss_ && gnss_alive) publishNeo3Connected(true);
    publishNeo3SafetyState(sw);
    std_msgs::msg::String status;
    std::ostringstream out;
    out << "{\"gnss_alive\":" << (gnss_alive ? "true" : "false")
        << ",\"gnss_ready\":" << (gnss_ready ? "true" : "false")
        << ",\"safety_switch\":" << (sw ? "true" : "false")
        << ",\"safety_led\":" << (led ? "true" : "false")
        << ",\"gnss_config_attempts\":" << static_cast<int>(std::lround(v[7]));
    out << ",\"parse_errors\":" << neo3_parse_errors_ << "}";
    status.data = out.str();
    neo3_status_pub_->publish(status);
  }

  void handleNeo3Switch(const std::string &payload) {
    std::vector<double> v;
    if (!parseCsvNumbers(payload, 3, v)) return;
    publishNeo3SafetyState(v[2] > 0.5);
  }

  void sensorWatchdogTick() {
    const auto t = std::chrono::steady_clock::now();
    if (publish_stm32_gnss_ && last_neo3_gnss_time_.time_since_epoch().count() != 0 &&
        t - last_neo3_gnss_time_ > std::chrono::duration<double>(neo3_sensor_timeout_sec_)) {
      publishNeo3Connected(false);
    }
    if (neo3pro_active_ && last_neo3_mag_time_.time_since_epoch().count() != 0 &&
        t - last_neo3_mag_time_ > std::chrono::duration<double>(neo3_sensor_timeout_sec_)) {
      publishRm3100Connected(false);
    }
    if (neo3pro_baro_connected_state_ && last_neo3pro_baro_time_.time_since_epoch().count() != 0 &&
        t - last_neo3pro_baro_time_ > std::chrono::duration<double>(neo3_sensor_timeout_sec_)) {
      neo3pro_baro_connected_state_ = false; std_msgs::msg::Bool b; b.data=false; neo3pro_baro_connected_pub_->publish(b);
    }
    if (neo3pro_node_connected_state_ && neo3pro_node_health_.seen &&
        t - neo3pro_node_health_.received > 3s) {
      neo3pro_node_connected_state_ = false; std_msgs::msg::Bool b; b.data=false; neo3pro_node_connected_pub_->publish(b);
    }
    if (neo3pro_gnss_status_connected_state_ && neo3pro_gnss_health_.seen &&
        t - neo3pro_gnss_health_.received > 3s) {
      neo3pro_gnss_status_connected_state_ = false; std_msgs::msg::Bool b; b.data=false; neo3pro_gnss_status_connected_pub_->publish(b);
    }
  }

  void sendConfigReply(bool ok, std::uint16_t txn, const std::string &key, const std::string &value) {
    std::string safe = value;
    std::replace(safe.begin(), safe.end(), ':', '_');
    (void)sendLine(std::string(ok ? "ACK:CFG:" : "ERR:CFG:") + std::to_string(txn) + ":" + key + ":" + safe);
  }

  static bool parseFiniteDouble(const std::string &text, double *value) {
    if (value == nullptr || text.empty()) return false;
    char *end = nullptr;
    errno = 0;
    const double parsed = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() || *end != '\0' || !std::isfinite(parsed)) return false;
    *value = parsed;
    return true;
  }

  static bool parseBool01(const std::string &text, bool *value) {
    if (value == nullptr) return false;
    const std::string normalized = upper(trim(text));
    if (normalized == "1" || normalized == "TRUE" || normalized == "ON") { *value = true; return true; }
    if (normalized == "0" || normalized == "FALSE" || normalized == "OFF") { *value = false; return true; }
    return false;
  }

  void requestDoubleConfig(const std::shared_ptr<rclcpp::AsyncParametersClient> &client,
                           const std::string &parameter, std::uint16_t txn,
                           const std::string &key, double requested,
                           double minimum, double maximum) {
    if (!std::isfinite(requested) || requested < minimum || requested > maximum) {
      sendConfigReply(false, txn, key, "OUT_OF_RANGE");
      return;
    }
    if (!client || !client->service_is_ready()) {
      sendConfigReply(false, txn, key, "PARAM_SERVICE_OFFLINE");
      return;
    }
    bool expected_false = false;
    if (!config_request_in_flight_.compare_exchange_strong(expected_false, true)) {
      sendConfigReply(false, txn, key, "BUSY");
      return;
    }
    const std::uint32_t epoch = config_epoch_.fetch_add(1U) + 1U;
    client->set_parameters_atomically({rclcpp::Parameter(parameter, requested)},
      [this, client, parameter, txn, key, requested, epoch](auto future) {
        try {
          const auto result = future.get();
          if (!result.successful) {
            config_request_in_flight_.store(false);
            sendConfigReply(false, txn, key, result.reason.empty() ? "SET_REJECTED" : result.reason);
            return;
          }
          client->get_parameters({parameter},
            [this, txn, key, requested, epoch](auto read_future) {
              try {
                const auto values = read_future.get();
                if (epoch != config_epoch_.load() || values.size() != 1U ||
                    values.front().get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                  config_request_in_flight_.store(false);
                  sendConfigReply(false, txn, key, "READBACK_INVALID");
                  return;
                }
                const double actual = values.front().as_double();
                const double tolerance = 1.0e-7 * std::max(1.0, std::abs(requested));
                if (!std::isfinite(actual) || std::abs(actual - requested) > tolerance) {
                  config_request_in_flight_.store(false);
                  sendConfigReply(false, txn, key, "READBACK_MISMATCH");
                  return;
                }
                if (key == "ERPMMPS") drive_erpm_per_mps_runtime_ = actual;
                config_request_in_flight_.store(false);
                sendConfigReply(true, txn, key, fixed(actual, 4));
              } catch (const std::exception &e) {
                config_request_in_flight_.store(false);
                RCLCPP_ERROR(get_logger(), "HMI config readback %s failed: %s", key.c_str(), e.what());
                sendConfigReply(false, txn, key, "READBACK_EXCEPTION");
              }
            });
        } catch (const std::exception &e) {
          config_request_in_flight_.store(false);
          RCLCPP_ERROR(get_logger(), "HMI config set %s failed: %s", key.c_str(), e.what());
          sendConfigReply(false, txn, key, "SET_EXCEPTION");
        }
      });
  }

  void requestBoolConfig(const std::shared_ptr<rclcpp::AsyncParametersClient> &client,
                         const std::string &parameter, std::uint16_t txn,
                         const std::string &key, bool requested) {
    if (!client || !client->service_is_ready()) {
      sendConfigReply(false, txn, key, "PARAM_SERVICE_OFFLINE");
      return;
    }
    bool expected_false = false;
    if (!config_request_in_flight_.compare_exchange_strong(expected_false, true)) {
      sendConfigReply(false, txn, key, "BUSY");
      return;
    }
    const std::uint32_t epoch = config_epoch_.fetch_add(1U) + 1U;
    client->set_parameters_atomically({rclcpp::Parameter(parameter, requested)},
      [this, client, parameter, txn, key, requested, epoch](auto future) {
        try {
          const auto result = future.get();
          if (!result.successful) {
            config_request_in_flight_.store(false);
            sendConfigReply(false, txn, key, result.reason.empty() ? "SET_REJECTED" : result.reason);
            return;
          }
          client->get_parameters({parameter},
            [this, txn, key, requested, epoch](auto read_future) {
              try {
                const auto values = read_future.get();
                if (epoch != config_epoch_.load() || values.size() != 1U ||
                    values.front().get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                  config_request_in_flight_.store(false);
                  sendConfigReply(false, txn, key, "READBACK_INVALID");
                  return;
                }
                const bool actual = values.front().as_bool();
                if (actual != requested) {
                  config_request_in_flight_.store(false);
                  sendConfigReply(false, txn, key, "READBACK_MISMATCH");
                  return;
                }
                if (key == "PERINF") perception_inference_runtime_ = actual;
                config_request_in_flight_.store(false);
                sendConfigReply(true, txn, key, actual ? "1" : "0");
              } catch (const std::exception &e) {
                config_request_in_flight_.store(false);
                RCLCPP_ERROR(get_logger(), "HMI bool config readback %s failed: %s", key.c_str(), e.what());
                sendConfigReply(false, txn, key, "READBACK_EXCEPTION");
              }
            });
        } catch (const std::exception &e) {
          config_request_in_flight_.store(false);
          RCLCPP_ERROR(get_logger(), "HMI bool config set %s failed: %s", key.c_str(), e.what());
          sendConfigReply(false, txn, key, "SET_EXCEPTION");
        }
      });
  }

  void handleConfigCommand(const std::string &line) {
    constexpr char prefix[] = "CMD:CFG:";
    const std::string body = line.substr(sizeof(prefix) - 1U);
    const auto first = body.find(':');
    const auto second = first == std::string::npos ? std::string::npos : body.find(':', first + 1U);
    if (first == std::string::npos || second == std::string::npos) return;
    const std::string txn_text = body.substr(0, first);
    std::uint32_t txn_raw = 0U;
    if (!parseU32DecimalStrict(txn_text, txn_raw) || txn_raw == 0U || txn_raw > 65535U) return;
    const auto txn = static_cast<std::uint16_t>(txn_raw);
    const std::string key = upper(trim(body.substr(first + 1U, second - first - 1U)));
    const std::string raw = trim(body.substr(second + 1U));

    if (key == "MODE") {
      bool manual = false;
      if (!parseBool01(raw, &manual)) { sendConfigReply(false, txn, key, "INVALID_BOOL"); return; }
      config_epoch_.fetch_add(1U);
      if (!setMode(manual ? "MANUAL" : "AUTO", "TFT_CFG", false)) {
        sendConfigReply(false, txn, key, "MODE_INTERLOCK");
        return;
      }
      sendConfigReply(true, txn, key, mode_ == "MANUAL" ? "1" : "0");
      return;
    }
    if (key == "MANSPD") {
      double value = 0.0;
      if (!parseFiniteDouble(raw, &value)) { sendConfigReply(false, txn, key, "INVALID_NUMBER"); return; }
      const int rounded = static_cast<int>(std::lround(value));
      if (std::abs(value - rounded) > 1.0e-6 || rounded < speed_min_pct_ || rounded > speed_max_pct_) {
        sendConfigReply(false, txn, key, "OUT_OF_RANGE");
        return;
      }
      config_epoch_.fetch_add(1U);
      manual_speed_pct_ = rounded;
      control_origin_ = "TFT_CFG";
      publishState();
      sendConfigReply(true, txn, key, std::to_string(manual_speed_pct_));
      return;
    }
    if (key == "STEERTEST") {
      double value = 0.0;
      if (!parseFiniteDouble(raw, &value) || value < 5.0 || value > 30.0) {
        sendConfigReply(false, txn, key, "OUT_OF_RANGE");
        return;
      }
      const auto result = set_parameter(rclcpp::Parameter("steering_test_angle_deg", value));
      if (!result.successful) {
        sendConfigReply(false, txn, key, result.reason.empty() ? "SET_REJECTED" : result.reason);
        return;
      }
      steering_test_angle_deg_ = std::clamp(get_parameter("steering_test_angle_deg").as_double(), 5.0, 30.0);
      config_epoch_.fetch_add(1U);
      sendState("CFGSTEERTEST", fixed(steering_test_angle_deg_, 1), true);
      sendConfigReply(true, txn, key, fixed(steering_test_angle_deg_, 1));
      return;
    }
    if (key == "ERPMMPS") {
      double value = 0.0;
      if (!parseFiniteDouble(raw, &value)) { sendConfigReply(false, txn, key, "INVALID_NUMBER"); return; }
      requestDoubleConfig(esc_params_, "drive_erpm_per_mps", txn, key, value, 100.0, 50000.0);
      return;
    }
    if (key == "PERINF") {
      bool enabled = false;
      if (!parseBool01(raw, &enabled)) { sendConfigReply(false, txn, key, "INVALID_BOOL"); return; }
      requestBoolConfig(perception_params_, "inference_enabled", txn, key, enabled);
      return;
    }
    sendConfigReply(false, txn, key.empty() ? "UNKNOWN" : key, "UNKNOWN_KEY");
  }

  void syncRuntimeConfig() {
    if (fd_ < 0 || config_request_in_flight_.load()) return;
    const std::uint32_t generation = config_sync_generation_.fetch_add(1U) + 1U;
    const std::uint32_t epoch = config_epoch_.load();
    if (esc_params_ && esc_params_->service_is_ready()) {
      esc_params_->get_parameters({"drive_erpm_per_mps"},
        [this, generation, epoch](auto future) {
          try {
            const auto values = future.get();
            if (generation != config_sync_generation_.load() || epoch != config_epoch_.load() ||
                config_request_in_flight_.load() || values.size() != 1U) return;
            if (values[0].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
              const double value = values[0].as_double();
              if (std::isfinite(value)) drive_erpm_per_mps_runtime_ = value;
            }
            sendState("CFGERPMMPS", fixed(drive_erpm_per_mps_runtime_, 3), false);
          } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ESC config sync failed: %s", e.what());
          }
        });
    }
    if (perception_params_ && perception_params_->service_is_ready()) {
      perception_params_->get_parameters({"inference_enabled"},
        [this, generation, epoch](auto future) {
          try {
            const auto values = future.get();
            if (generation != config_sync_generation_.load() || epoch != config_epoch_.load() ||
                config_request_in_flight_.load() || values.size() != 1U) return;
            if (values[0].get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
              perception_inference_runtime_ = values[0].as_bool();
              sendState("CFGPERINF", perception_inference_runtime_ ? "1" : "0", false);
            }
          } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Perception config sync failed: %s", e.what());
          }
        });
    }
  }

  void disableSteeringTest() {
    steering_test_active_ = false;
    steering_test_target_deg_ = 0.0;
    sendState("STEERTEST", "IDLE", true);
    if (!esc_params_ || !esc_params_->service_is_ready() || steering_test_disable_in_flight_) return;
    steering_test_disable_in_flight_ = true;
    esc_params_->set_parameters_atomically({rclcpp::Parameter("raw_commissioning_enabled", false)},
      [this](auto future) {
        try {
          const auto result = future.get();
          if (!result.successful) {
            RCLCPP_WARN(get_logger(), "Failed disabling HMI steering commissioning: %s", result.reason.c_str());
          }
        } catch (const std::exception &e) {
          RCLCPP_WARN(get_logger(), "Exception disabling HMI steering commissioning: %s", e.what());
        }
        steering_test_disable_in_flight_ = false;
      });
  }

  void startSteeringTest(double target_deg) {
    if (!std::isfinite(target_deg) || std::abs(target_deg) > 30.0 ||
        !manualMotionAllowed(true) || std::abs(drive_actual_mps_) > 0.02 ||
        !esc_params_ || !esc_params_->service_is_ready() ||
        steering_test_enable_in_flight_ || steering_test_disable_in_flight_) {
      sendState("STEERTEST", "LOCKED", true);
      return;
    }
    drive_ = "STOP";
    steer_ = "NONE";
    steering_hmi_target_deg_ = 0.0;
    steering_test_enable_in_flight_ = true;
    esc_params_->set_parameters_atomically({rclcpp::Parameter("raw_commissioning_enabled", true)},
      [this, target_deg](auto future) {
        try {
          const auto result = future.get();
          if (!result.successful) {
            steering_test_enable_in_flight_ = false;
            sendState("STEERTEST", "LOCKED", true);
            RCLCPP_WARN(get_logger(), "HMI steering test enable rejected: %s", result.reason.c_str());
            return;
          }
          esc_params_->get_parameters({"raw_commissioning_enabled"},
            [this, target_deg](auto read_future) {
              try {
                const auto values = read_future.get();
                const bool enabled = values.size() == 1U &&
                  values.front().get_type() == rclcpp::ParameterType::PARAMETER_BOOL &&
                  values.front().as_bool();
                steering_test_enable_in_flight_ = false;
                if (!enabled || !manualMotionAllowed(true) || std::abs(drive_actual_mps_) > 0.02) {
                  sendState("STEERTEST", "LOCKED", true);
                  disableSteeringTest();
                  return;
                }
                steering_test_target_deg_ = target_deg;
                steering_test_deadline_ = std::chrono::steady_clock::now() + 2s;
                steering_test_active_ = true;
                sendState("STEERTEST", "ACTIVE", true);
              } catch (const std::exception &e) {
                steering_test_enable_in_flight_ = false;
                sendState("STEERTEST", "LOCKED", true);
                RCLCPP_WARN(get_logger(), "HMI steering test readback failed: %s", e.what());
                disableSteeringTest();
              }
            });
        } catch (const std::exception &e) {
          steering_test_enable_in_flight_ = false;
          sendState("STEERTEST", "LOCKED", true);
          RCLCPP_WARN(get_logger(), "HMI steering test enable exception: %s", e.what());
        }
      });
    publishState();
  }

  void publishSteeringTestTick() {
    if (!steering_test_active_) return;
    if (!manualMotionAllowed(true) || estop_ || std::abs(drive_actual_mps_) > 0.02 ||
        std::chrono::steady_clock::now() >= steering_test_deadline_) {
      disableSteeringTest();
      return;
    }
    std_msgs::msg::Float64MultiArray command;
    command.data = {0.0, steering_test_target_deg_};
    commissioning_pub_->publish(command);
  }

  void handleHmiLine(const std::string &line) {
    last_rx_ = std::chrono::steady_clock::now();
    silent_open_failures_ = 0U;
    if (!connected_) publishConnected(true);
    if (line.rfind("USB:STAT:", 0) == 0) {
      last_usb_status_ = line;
      std_msgs::msg::String msg; msg.data = line; usb_status_pub_->publish(msg);
      return;
    }
    if (line.rfind("SENS:GNSSPRO:", 0) == 0) { handleNeo3ProGnssMeta(line.substr(sizeof("SENS:GNSSPRO:") - 1U)); return; }
    if (line.rfind("SENS:GNSSCOV:", 0) == 0) { handleNeo3ProGnssCov(line.substr(sizeof("SENS:GNSSCOV:") - 1U)); return; }
    if (line.rfind("SENS:ECEF:", 0) == 0) { handleNeo3ProEcef(line.substr(sizeof("SENS:ECEF:") - 1U)); return; }
    if (line.rfind("SENS:MAGPRO:", 0) == 0) { handleNeo3ProMag(line.substr(sizeof("SENS:MAGPRO:") - 1U)); return; }
    if (line.rfind("SENS:BARO:", 0) == 0) { handleNeo3ProPressure(line.substr(sizeof("SENS:BARO:") - 1U)); return; }
    if (line.rfind("SENS:TEMP:", 0) == 0) { handleNeo3ProTemperature(line.substr(sizeof("SENS:TEMP:") - 1U)); return; }
    if (line.rfind("SENS:NODEINFO:", 0) == 0) { handleNeo3ProTextDiagnostic("NODEINFO", line.substr(sizeof("SENS:NODEINFO:") - 1U), neo3pro_node_info_pub_); return; }
    if (line.rfind("SENS:PARAM:", 0) == 0) { handleNeo3ProTextDiagnostic("PARAM", line.substr(sizeof("SENS:PARAM:") - 1U), neo3pro_param_pub_); return; }
    if (line.rfind("SENS:NODE:", 0) == 0) { handleNeo3ProNode(line.substr(sizeof("SENS:NODE:") - 1U)); return; }
    if (line.rfind("SENS:GNSSSTAT:", 0) == 0) { handleNeo3ProGnssStatus(line.substr(sizeof("SENS:GNSSSTAT:") - 1U)); return; }
    if (line.rfind("SENS:GNSSHEAD:", 0) == 0) { handleNeo3ProHeading(line.substr(sizeof("SENS:GNSSHEAD:") - 1U)); return; }
    if (line.rfind("SENS:BUTTON:", 0) == 0) { handleNeo3ProButton(line.substr(sizeof("SENS:BUTTON:") - 1U)); return; }
    if (line.rfind("SENS:HWPRO:", 0) == 0) { handleNeo3ProHardware(line.substr(sizeof("SENS:HWPRO:") - 1U)); return; }
    if (line.rfind("SENS:CANRAW:", 0) == 0) { handleNeo3ProCanRaw(line.substr(sizeof("SENS:CANRAW:") - 1U)); return; }
    if (line.rfind("SENS:NEOHEALTH:", 0) == 0) { handleNeo3ProTextDiagnostic("NEOHEALTH", line.substr(sizeof("SENS:NEOHEALTH:") - 1U), neo3pro_health_pub_); return; }
    if (line.rfind("SENS:CANRX:", 0) == 0) { handleNeo3ProTextDiagnostic("CANRX", line.substr(sizeof("SENS:CANRX:") - 1U), neo3pro_can_status_pub_); return; }
    if (line.rfind("SENS:DNASRV:", 0) == 0) { handleNeo3ProTextDiagnostic("DNASRV", line.substr(sizeof("SENS:DNASRV:") - 1U), neo3pro_dna_status_pub_); return; }
    if (line.rfind("SENS:GNSS:", 0) == 0) { handleNeo3Gnss(line.substr(10)); return; }
    if (line.rfind("SENS:GNSSF:", 0) == 0) { handleNeo3GnssFallback(line.substr(11)); return; }
    if (line.rfind("SENS:HW:", 0) == 0) { handleNeo3Hardware(line.substr(8)); return; }
    if (line.rfind("SENS:SW:", 0) == 0) { handleNeo3Switch(line.substr(8)); return; }
    if (line.rfind("[TOUCH]", 0) == 0) {
      RCLCPP_INFO(get_logger(), "%s", line.c_str());
      return;
    }
    if (line.rfind("LINK:ROS:", 0) == 0) {
      RCLCPP_INFO(get_logger(), "HMI %s", line.c_str());
      return;
    }
    if (line.rfind("CMD:CFG:", 0) == 0) { handleConfigCommand(line); return; }
    if (line.rfind("PAGE:", 0) == 0) {
      const std::string p = upper(trim(line.substr(5)));
      const bool token_ok = !p.empty() && p.size() <= 27U &&
        std::all_of(p.begin(), p.end(), [](unsigned char c) {
          return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
        });
      if (token_ok) page_ = p;
      publishState();
      return;
    }
    if (line.rfind("CMD:WP:SELECT:", 0) == 0) {
      selectWaypoint(std::atoi(line.c_str() + 14), "TFT", false);
      return;
    }
    if (line.rfind("CMD:WP:SAVE:", 0) == 0) {
      saveWaypoint(std::atoi(line.c_str() + 12), "", "TFT");
      return;
    }
    if (line.rfind("CMD:WP:GO:", 0) == 0) {
      goWaypoint(std::atoi(line.c_str() + 10), "TFT");
      return;
    }
    if (line == "CMD:NAV:STOP") { stopNavigation("TFT"); return; }
    if (line.rfind("CMD:DRIVE:", 0) == 0) {
      const std::string rest = line.substr(10);
      const auto colon = rest.find(':');
      const std::string action = upper(trim(rest.substr(0, colon)));
      if (colon != std::string::npos) manual_speed_pct_ = std::clamp(std::atoi(rest.c_str() + colon + 1), speed_min_pct_, speed_max_pct_);
      applyDrive(action, "TFT", false);
      return;
    }
    if (line.rfind("CMD:STEER:", 0) == 0) {
      const std::string request = upper(trim(line.substr(10)));
      if (request == "STOP") { disableSteeringTest(); return; }
      double target = 0.0;
      if (parseFiniteDouble(request, &target)) startSteeringTest(target);
      else sendState("STEERTEST", "LOCKED", true);
      return;
    }
  }

  void handleRequest(const std::string &raw) {
    const std::string cleaned = trim(raw);
    const std::string req = upper(cleaned);
    if (req.rfind("PAGE:", 0) == 0) {
      const std::string p = req.substr(5);
      if (p == "OVERVIEW" || p == "ESC" || p == "PERCEPTION" || p == "NAVIGATION") {
        sendLine("GOTO:" + p);
      }
      return;
    }
    if (req.rfind("WAYPOINT:SELECT:", 0) == 0) {
      selectWaypoint(std::atoi(req.c_str() + 16), "WEB", true);
      return;
    }
    if (req.rfind("WAYPOINT:SAVE:", 0) == 0) {
      const std::string rest = cleaned.substr(14);
      const auto colon = rest.find(':');
      const int index = std::atoi(rest.substr(0, colon).c_str());
      const std::string name = colon == std::string::npos ? "" : rest.substr(colon + 1);
      saveWaypoint(index, name, "WEB");
      return;
    }
    if (req.rfind("WAYPOINT:GO:", 0) == 0) { goWaypoint(std::atoi(req.c_str() + 12), "WEB"); return; }
    if (req == "NAV:STOP") { stopNavigation("WEB"); return; }
    if (req.rfind("MODE:", 0) == 0) { setMode(req.substr(5), "WEB", true); return; }
    if (req.rfind("DRIVE:", 0) == 0) { applyDrive(req.substr(6), "WEB", true); return; }
    if (req.rfind("STEER:", 0) == 0) { applySteer(req.substr(6), "WEB", true); return; }
    if (req.rfind("SPEED:", 0) == 0) {
      const int pct = std::clamp(std::atoi(req.c_str() + 6), speed_min_pct_, speed_max_pct_);
      manual_speed_pct_ = pct;
      control_origin_ = "WEB";
      publishState();
      return;
    }
    if (req == "SYNC") { tx_cache_.clear(); sendLine("GET:STATE"); telemetryTick(); return; }
  }

  bool navigationStateActive() const {
    return navigation_state_ == "QUEUED" || navigation_state_ == "NAVIGATING";
  }

  bool setMode(const std::string &requested, const std::string &origin, bool mirror) {
    const std::string next = requested == "MANUAL" ? "MANUAL" : "AUTO";
    if (next == "MANUAL" &&
        (!connected_ || estop_ || !esc_ready_ || !esc_feedback_ ||
         std::abs(drive_actual_mps_) > 0.02 || navigationStateActive())) {
      reject("MANUAL mode requires connected/stationary ESC, E-stop clear, navigation idle");
      if (mirror) sendLine("MODE:" + mode_);
      return false;
    }
    mode_ = next;
    control_origin_ = origin;
    if (mode_ == "AUTO") {
      drive_ = "STOP"; steer_ = "NONE"; steering_hmi_target_deg_ = 0.0;
      last_drive_command_time_ = {}; last_steer_command_time_ = {};
      if (steering_test_active_ || steering_test_enable_in_flight_) disableSteeringTest();
    }
    if (mirror) sendLine("MODE:" + mode_);
    publishState();
    return true;
  }

  bool manualMotionAllowed(bool steering) const {
    if (!connected_ || awaiting_host_session_ || mode_ != "MANUAL" || estop_ ||
        !esc_ready_ || !esc_feedback_ || navigationStateActive()) return false;
    if (steering && (!steer_connected_ || std::abs(drive_actual_mps_) > 0.02)) return false;
    return true;
  }

  bool manualInitialMotionAllowed(bool steering) const {
    // Match the physical TFT initial-motion interlock: a new manual motion may
    // only start from a stopped vehicle. Once an identical DRIVE lease is active,
    // refresh packets are permitted while moving so the deadman can remain alive.
    return manualMotionAllowed(steering) && std::abs(drive_actual_mps_) <= 0.02;
  }

  bool manualLeaseFresh(const std::chrono::steady_clock::time_point &stamp) const {
    if (stamp.time_since_epoch().count() == 0) return false;
    return std::chrono::steady_clock::now() - stamp <= std::chrono::duration<double>(manual_command_lease_sec_);
  }

  void reject(const std::string &reason) {
    last_rejection_ = reason;
    RCLCPP_WARN(get_logger(), "HMI manual request rejected: %s", reason.c_str());
    publishState();
  }

  void applyDrive(std::string action, const std::string &origin, bool mirror) {
    action = upper(trim(action));
    if (action == "STOP") {
      drive_ = "STOP"; last_drive_command_time_ = {}; control_origin_ = origin;
      (void)mirror;
      publishState(); return;
    }
    if (action != "FWD" && action != "REV") return;
    const bool same_direction_refresh = drive_ == action && manualLeaseFresh(last_drive_command_time_);
    if ((drive_ == "FWD" || drive_ == "REV") && drive_ != action) {
      reject("manual direction reversal requires explicit STOP first");
      return;
    }
    if (same_direction_refresh) {
      if (!manualMotionAllowed(false)) { reject("manual drive lease lost safety authority"); return; }
    } else if (!manualInitialMotionAllowed(false)) {
      reject("drive start requires MANUAL + stopped vehicle + ESC ACK + E-STOP clear + nav idle");
      return;
    }
    drive_ = action; last_drive_command_time_ = std::chrono::steady_clock::now();
    control_origin_ = origin; last_rejection_.clear();
    (void)mirror;
    publishState();
  }

  void applySteer(std::string action, const std::string &origin, bool mirror) {
    action = upper(trim(action));
    if (action == "STOP" || action == "NONE") {
      steer_ = "NONE"; last_steer_command_time_ = {}; steering_hmi_target_deg_ = 0.0;
      control_origin_ = origin; publishState(); return;
    }
    if (action != "LEFT" && action != "RIGHT" && action != "CENTER") return;
    if (!manualMotionAllowed(true)) { reject("steering requires MANUAL + HMI + ESC/encoder ready + vehicle stopped"); return; }
    steer_ = action; last_steer_command_time_ = std::chrono::steady_clock::now();
    control_origin_ = origin; last_rejection_.clear();
    steering_hmi_target_deg_ = action == "LEFT" ? -hmi_steer_full_scale_deg_ : (action == "RIGHT" ? hmi_steer_full_scale_deg_ : 0.0);
    (void)mirror;
    publishState();
  }

  void commandTick() {
    publishSteeringTestTick();
    bool lease_expired = false;
    if ((drive_ == "FWD" || drive_ == "REV") && !manualLeaseFresh(last_drive_command_time_)) {
      drive_ = "STOP"; last_drive_command_time_ = {}; ++manual_lease_expirations_; lease_expired = true;
    }
    if ((steer_ == "LEFT" || steer_ == "RIGHT" || steer_ == "CENTER") && !manualLeaseFresh(last_steer_command_time_)) {
      steer_ = "NONE"; steering_hmi_target_deg_ = 0.0; last_steer_command_time_ = {};
      ++manual_lease_expirations_; lease_expired = true;
    }
    if (lease_expired) {
      control_origin_ = "LEASE_TIMEOUT";
      last_rejection_ = "manual command lease expired; output forced to zero";
      publishState();
    }
    geometry_msgs::msg::Twist cmd;
    std_msgs::msg::String source;
    const bool allowed = manualMotionAllowed(false);
    const bool drive_active = allowed && manualLeaseFresh(last_drive_command_time_) && (drive_ == "FWD" || drive_ == "REV");
    const bool steer_active = manualMotionAllowed(true) && manualLeaseFresh(last_steer_command_time_) &&
                              (steer_ == "LEFT" || steer_ == "RIGHT" || steer_ == "CENTER");
    if (drive_active) {
      const double speed = manual_speed_max_mps_ * static_cast<double>(manual_speed_pct_) / 100.0;
      cmd.linear.x = drive_ == "FWD" ? speed : -speed;
    }
    if (steer_active) {
      double hmi_deg = steering_hmi_target_deg_;
      if (invert_hmi_steering_) hmi_deg = -hmi_deg;
      const double fraction = std::clamp(hmi_deg / hmi_steer_full_scale_deg_, -1.0, 1.0);
      cmd.angular.z = fraction * teleop_yaw_max_rps_;
    }
    source.data = (drive_active || steer_active) ? ("HMI_" + control_origin_) : "STOP";
    cmd_pub_->publish(cmd);
    source_pub_->publish(source);
  }

  void resetExtendedTelemetrySession(uint32_t session) {
    extended_session_id_ = session;
    escx_power_seq_ = escx_motor_seq_ = escx_encoder_seq_ = escx_link_seq_ = escx_perf_seq_ = 0U;
    perx_cam_seq_ = perx_det_seq_ = perx_lane_seq_ = perx_drv_seq_ = perx_obs_seq_ = perx_perf_seq_ = 0U;
    navx_pose_seq_ = navx_odom_seq_ = navx_imu_seq_ = navx_nav2_seq_ = navx_cost_seq_ = navx_ctrl_seq_ = 0U;
    last_extended_tx_ = {};
    last_critical_state_tx_ = {};
  }

  void extendedTelemetryTick() {
    if (awaiting_host_session_ || extended_session_id_ == 0U) return;
    const auto now_steady = std::chrono::steady_clock::now();
    if (last_extended_tx_.time_since_epoch().count() != 0 &&
        now_steady - last_extended_tx_ < 200ms) return;
    last_extended_tx_ = now_steady;

    const auto flag = [](bool v) { return v ? "1" : "0"; };
    const auto finite = [](double v) { return std::isfinite(v) ? v : 0.0; };
    const auto send_ext = [this](const char *domain, const char *group, uint32_t seq,
                                 uint32_t age, const std::string &payload) {
      const char *d = std::strcmp(domain, "ESCX") == 0 ? "ESC" :
                      (std::strcmp(domain, "PERX") == 0 ? "PER" : "NAV");
      std::ostringstream signed_part;
      signed_part << "F4X3:" << d << ':' << group << ":3:" << extended_session_id_
                  << ':' << seq << ':' << age << ':' << payload.size() << ':' << payload;
      const std::string body = signed_part.str();
      std::ostringstream line;
      line << body << ':' << std::uppercase << std::hex << std::setw(8) << std::setfill('0')
           << wireCrc32(body);
      const std::string encoded = line.str();
      if (encoded.size() <= 360U) (void)sendLine(encoded, 0);
      else RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "F4X3 telemetry line exceeded 360 bytes: %s/%s size=%zu", domain, group, encoded.size());
    };

    const uint32_t foc_age = steadyAgeMs(last_foc_telemetry_time_);
    static constexpr uint32_t kEscTelemetryFreshLimitMs = 2000U;
    const uint32_t esc_fresh_limit_ms = kEscTelemetryFreshLimitMs;
    // Transient-local FOC telemetry may retain the last physical sample after a
    // USB-UART hot-unplug. Never present that latched sample as valid once its
    // age exceeds the same transport freshness contract used by the ESC link.
    const bool foc_valid = foc_values_valid_ && foc_age != 0xFFFFFFFFU &&
      foc_age <= esc_fresh_limit_ms && esc_ready_ && esc_feedback_;
    const double vbus = (left_vbus_v_ > 0.1 && right_vbus_v_ > 0.1) ?
      0.5 * (left_vbus_v_ + right_vbus_v_) : std::max(left_vbus_v_, right_vbus_v_);
    {
      std::ostringstream p; p << flag(foc_valid) << ',' << fixed(finite(vbus),2) << ','
        << fixed(finite(right_current_motor_a_),2) << ',' << fixed(finite(right_current_in_a_),2) << ','
        << fixed(finite(right_iq_a_),2) << ',' << fixed(finite(right_id_a_),2) << ','
        << fixed(finite(right_duty_),4) << ',' << fixed(finite(right_temp_mos_c_),1) << ',' << right_fault_;
      send_ext("ESCX","PWR",++escx_power_seq_,foc_age,p.str());
    }
    {
      std::ostringstream p; p << flag(foc_valid) << ',' << fixed(finite(left_vbus_v_),2) << ','
        << fixed(finite(left_current_motor_a_),2) << ',' << fixed(finite(left_duty_),4) << ',' << fixed(finite(left_erpm_),1) << ',' << left_fault_ << ','
        << fixed(finite(right_vbus_v_),2) << ',' << fixed(finite(right_current_motor_a_),2) << ',' << fixed(finite(right_duty_),4) << ',' << fixed(finite(right_erpm_),1) << ',' << right_fault_;
      send_ext("ESCX","MTR",++escx_motor_seq_,foc_age,p.str());
    }
    {
      const bool enc_valid = steering_cal_ext_valid_ && foc_age != 0xFFFFFFFFU &&
        foc_age <= esc_fresh_limit_ms && esc_ready_ && esc_feedback_;
      std::ostringstream p; p << flag(enc_valid) << ',' << steering_raw_count_ext_ << ',' << steering_span_ext_ << ',' << steering_raw_target_ext_ << ','
        << flag(steering_calibrated_ext_) << ',' << flag(steering_homed_ext_) << ',' << flag(steering_synced_ext_) << ",0," << fixed(finite(left_position_deg_),2);
      send_ext("ESCX","ENC",++escx_encoder_seq_,foc_age,p.str());
    }
    {
      const uint32_t age = steadyAgeMs(last_foc_telemetry_time_);
      std::ostringstream p; p << flag(esc_ready_ && esc_feedback_) << ",115200,DIRECT";
      send_ext("ESCX","LINK",++escx_link_seq_,age,p.str());
    }
    send_ext("ESCX","PERF",++escx_perf_seq_,0xFFFFFFFFU,"0,0,0");

    const uint32_t cam_age = std::min(steadyAgeMs(last_camera_state_time_), steadyAgeMs(last_performance_time_));
    const bool cam_seen = cam_age != 0xFFFFFFFFU;
    {
      std::ostringstream p; p << flag(cam_seen) << ',' << flag(perception_ready_) << ',' << fixed(finite(camera_fps_),2) << ','
        << fixed(finite(perception_latency_ms_),2) << ',' << perception_dropped_frames_ << ",0," << wireToken(perception_backend_,10U) << ',' << flag(perception_inference_runtime_);
      send_ext("PERX","CAM",++perx_cam_seq_,cam_age,p.str());
    }
    const uint32_t obs_age = steadyAgeMs(last_obstacle_time_);
    const bool obs_seen = obs_age != 0xFFFFFFFFU;
    {
      std::ostringstream p; p << flag(obs_seen) << ',' << obstacle_count_ << ',' << obstacle_count_ << ',' << nearest_missed_frames_ << ',' << nearest_track_id_ << ','
        << fixed(finite(nearest_distance_m_),3) << ',' << fixed(finite(nearest_lateral_m_),3) << ',' << fixed(finite(nearest_conf_pct_),1) << ',' << wireToken(nearest_object_,20U);
      send_ext("PERX","DET",++perx_det_seq_,obs_age,p.str());
    }
    const uint32_t lane_age = steadyAgeMs(last_lane_metrics_time_);
    const bool lane_seen = lane_age != 0xFFFFFFFFU;
    {
      std::ostringstream p; p << flag(lane_seen) << ',' << flag(lane_metric_valid_) << ',' << fixed(finite(lane_center_offset_m_),3) << ','
        << fixed(finite(lane_confidence_pct_),1) << ',' << fixed(finite(lane_road_width_m_),3) << ',' << fixed(finite(lane_left_clearance_m_),3) << ','
        << fixed(finite(lane_right_clearance_m_),3) << ',' << fixed(finite(lane_heading_error_deg_),2);
      send_ext("PERX","LANE",++perx_lane_seq_,lane_age,p.str());
    }
    const uint32_t drv_age = steadyAgeMs(last_drivable_time_);
    const bool drv_seen = drv_age != 0xFFFFFFFFU;
    const double valid_row_pct = drivable_sample_rows_ > 0U ?
      100.0 * static_cast<double>(drivable_valid_rows_) / static_cast<double>(drivable_sample_rows_) : 0.0;
    {
      std::ostringstream p; p << flag(drv_seen) << ',' << flag(drivable_valid_) << ',' << fixed(finite(valid_row_pct),1) << ",0,0,0";
      send_ext("PERX","DRV",++perx_drv_seq_,drv_age,p.str());
    }
    {
      const bool blocked = perception_emergency_;
      std::ostringstream p; p << flag(obs_seen) << ',' << obstacle_count_ << ',' << flag(blocked) << ',' << fixed(finite(nearest_distance_m_),3) << ','
        << fixed(finite(nearest_lateral_m_),3) << ',' << nearest_track_id_ << ',' << nearest_missed_frames_ << ",0";
      send_ext("PERX","OBS",++perx_obs_seq_,obs_age,p.str());
    }
    const uint32_t perf_age = steadyAgeMs(last_performance_time_);
    const bool perf_seen = perf_age != 0xFFFFFFFFU;
    {
      std::ostringstream p; p << flag(perf_seen) << ',' << fixed(finite(camera_fps_),2) << ',' << fixed(finite(perception_latency_ms_),2) << ','
        << perception_dropped_frames_ << ',' << perception_raw_detection_count_ << ',' << perception_confirmed_count_ << ',' << wireToken(perception_backend_,10U);
      send_ext("PERX","PERF",++perx_perf_seq_,perf_age,p.str());
    }

    const uint32_t pose_age = steadyAgeMs(last_map_pose_);
    const bool pose_seen = have_map_pose_ && pose_age != 0xFFFFFFFFU;
    {
      std::ostringstream p; p << flag(pose_seen) << ',' << fixed(finite(map_x_),3) << ',' << fixed(finite(map_y_),3) << ',' << fixed(finite(map_yaw_ * 180.0 / kPi),2) << ','
        << fixed(finite(map_cov_x_),4) << ',' << fixed(finite(map_cov_y_),4) << ',' << fixed(finite(map_yaw_var_),5);
      send_ext("NAVX","POSE",++navx_pose_seq_,pose_age,p.str());
    }
    const uint32_t odom_age = steadyAgeMs(last_local_odom_time_);
    const bool odom_seen = odom_age != 0xFFFFFFFFU;
    {
      std::ostringstream p; p << flag(odom_seen) << ',' << fixed(finite(odom_x_),3) << ',' << fixed(finite(odom_y_),3) << ',' << fixed(finite(odom_yaw_ * 180.0 / kPi),2) << ','
        << fixed(finite(odom_linear_mps_),3) << ',' << fixed(finite(odom_yaw_rate_rps_),3);
      send_ext("NAVX","ODOM",++navx_odom_seq_,odom_age,p.str());
    }
    const uint32_t imu_age = steadyAgeMs(last_imu_data_time_);
    const bool imu_seen = imu_age != 0xFFFFFFFFU;
    double heading_diff = std::fabs(heading_deg_ - imu_yaw_deg_);
    if (heading_diff > 180.0) heading_diff = 360.0 - heading_diff;
    {
      std::ostringstream p; p << flag(imu_seen) << ',' << fixed(finite(imu_yaw_deg_),2) << ',' << fixed(finite(heading_deg_),2) << ',' << fixed(finite(heading_diff),2);
      send_ext("NAVX","IMU",++navx_imu_seq_,imu_age,p.str());
    }
    const uint32_t mppi_age_local = steadyAgeMs(last_mppi_status_time_);
    const uint32_t traj_age = steadyAgeMs(last_trajectory_state_time_);
    const uint32_t nav_age = std::min(mppi_age_local, traj_age);
    const bool nav_seen = nav_age != 0xFFFFFFFFU;
    const bool path_valid = kvBool(trajectory_state_text_, "path_valid").value_or(false);
    const uint32_t cmd_age_ms = static_cast<uint32_t>(std::clamp(kvNumber(mppi_status_text_, "mppi_age").value_or(999.0) * 1000.0, 0.0, 4294967294.0));
    const double cmd_v = kvNumber(mppi_status_text_, "mppi_v").value_or(0.0);
    const double cmd_w = kvNumber(mppi_status_text_, "mppi_w").value_or(0.0);
    const bool controller_ready = kvBool(mppi_status_text_, "ready").value_or(nav2_ready_);
    const bool smoother_ready = kvBool(mppi_status_text_, "smoother_closed_loop_eligible").value_or(false);
    {
      std::ostringstream p; p << flag(nav_seen) << ',' << flag(nav2_ready_) << ',' << flag(path_valid) << ',' << cmd_age_ms << ','
        << fixed(finite(cmd_v),3) << ',' << fixed(finite(cmd_w),3) << ',' << (nav2_ready_ ? "READY" : "WAIT") << ','
        << (controller_ready ? "READY" : "WAIT") << ',' << (smoother_ready ? "READY" : "WAIT");
      send_ext("NAVX","NAV2",++navx_nav2_seq_,nav_age,p.str());
    }
    const uint32_t cost_age = steadyAgeMs(last_costmap_time_);
    const bool cost_seen = cost_age != 0xFFFFFFFFU;
    const unsigned path_relevant = static_cast<unsigned>(std::max(0.0, kvNumber(trajectory_state_text_, "path_relevant_points").value_or(0.0)));
    {
      std::ostringstream p; p << flag(cost_seen) << ',' << flag(cost_seen) << ',' << costmap_obstacle_cells_ << ',' << path_relevant << ',' << flag(costmap_blocked_);
      send_ext("NAVX","COST",++navx_cost_seq_,cost_age,p.str());
    }
    {
      std::ostringstream p; p << flag(nav_seen) << ',' << cmd_age_ms << ',' << fixed(finite(cmd_v),3) << ',' << fixed(finite(cmd_w),3);
      send_ext("NAVX","CTRL",++navx_ctrl_seq_,mppi_age_local,p.str());
    }
  }

  void telemetryTick() {
    if (fd_ < 0 || awaiting_host_session_) return;
    // Best-effort display telemetry remains changed-only. Safety/readiness state
    // gets a tiny periodic refresh so a line accepted by Linux but dropped inside
    // the F411 cannot remain stale forever. Nine short lines/s is negligible next
    // to the session-bound F4X3 stream and avoids the old ~40-field burst.
    const bool force = false;
    const auto now_steady = std::chrono::steady_clock::now();
    const bool force_critical = last_critical_state_tx_.time_since_epoch().count() == 0 ||
        now_steady - last_critical_state_tx_ >= std::chrono::seconds(1);
    if (force_critical) last_critical_state_tx_ = now_steady;

    const bool manual_ready = esc_ready_ && esc_feedback_ && !estop_;
    const bool auto_ready = motion_ready_ && nav2_ready_ && esc_ready_ && !estop_;
    const bool ready = mode_ == "MANUAL" ? manual_ready : auto_ready;
    const double sign = invert_hmi_steering_ ? -1.0 : 1.0;
    const std::string lane_compact = jsonString(lane_state_text_, "state").value_or(
      lane_state_text_.empty() ? "UNKNOWN" : lane_state_text_.substr(0, 19));
    const std::string localization_compact = kvString(localization_state_text_, "mode").value_or(
      localization_state_text_.empty() ? "UNKNOWN" : localization_state_text_.substr(0, 23));
    const bool ekf_local_fresh = kvString(ekf_local_status_text_, "fresh").value_or("false") == "true";
    const bool ekf_global_fresh = kvString(ekf_global_status_text_, "fresh").value_or("false") == "true";
    const std::string gnss_source = kvString(gnss_status_text_, "src").value_or(gnss_ready_ ? "GNSS" : "OFFLINE");

    sendState("SYS", estop_ ? "FAULT" : (ready ? "READY" : "NOT READY"), force_critical);
    sendState("MODE", mode_, force_critical);
    sendState("STATE", estop_ ? "FAULT" : (std::abs(drive_actual_mps_) > 0.02 ? "RUNNING" : "STOPPED"), force_critical);
    sendState("ESTOP", estop_ ? "1" : "0", force_critical);
    sendState("VESC_LINK", (esc_ready_ && esc_feedback_) ? "1" : "0", force_critical);
    sendState("ESC", (esc_ready_ && esc_feedback_) ? "1" : "0", force_critical);
    sendState("ENC", (steer_connected_ && esc_feedback_) ? "1" : "0", force_critical);
    sendState("MOTION", motion_ready_ ? "1" : "0", force_critical);
    sendState("NAV2", nav2_ready_ ? "1" : "0", force_critical);

    sendState("SPD", fixed(std::abs(drive_actual_mps_) * 3.6, 2), force);
    sendState("DRIVE_TGT", fixed(drive_target_mps_, 3), force);
    sendState("DRIVE_ACT", fixed(drive_actual_mps_, 3), force);
    sendState("ERPM", fixed(motor_erpm_, 1), force);
    sendState("STEER_TARGET", fixed(sign * steering_target_rad_ * 180.0 / kPi, 2), force);
    sendState("STEER_ACTUAL", fixed(sign * steering_actual_rad_ * 180.0 / kPi, 2), force);
    sendState("STEER_ERR", fixed(sign * (steering_target_rad_ - steering_actual_rad_) * 180.0 / kPi, 2), force);

    sendState("GPS", gnss_ready_ ? "1" : "0", force);
    sendState("FIX", std::to_string(fix_type_), force);
    sendState("LAT", fixed(latitude_, 7), force);
    sendState("LON", fixed(longitude_, 7), force);
    sendState("SAT", std::to_string(satellites_), force);
    sendState("HDOP", fixed(hdop_, 2), force);
    sendState("HACC", fixed(hacc_m_, 2), force);
    sendState("GAGE", fixed(gnss_age_sec_, 3), force);
    sendState("HEAD", fixed(heading_deg_, 1), force);
    sendState("IMU", imu_ready_ ? "1" : "0", force);
    sendState("GYROZ", fixed(gyro_z_rps_, 3), force);
    const bool rm3100_fresh = neo3pro_active_ && last_neo3_mag_time_.time_since_epoch().count() != 0 &&
      std::chrono::steady_clock::now() - last_neo3_mag_time_ <= std::chrono::duration<double>(neo3_sensor_timeout_sec_);
    sendState("MAG", rm3100_fresh ? "1" : "0", force);
    sendState("GNSSSTATUS", gnss_source.substr(0, 19), force);
    sendState("IMUSTATUS", imu_ready_ ? "READY" : "OFFLINE", force);
    sendState("EKFLOCAL", ekf_local_fresh ? "READY" : "STALE", force);
    sendState("EKFGLOBAL", ekf_global_fresh ? "READY" : "STALE", force);
    sendState("LOCSTATE", localization_compact.substr(0, 23), force);

    sendState("CAM", camera_ready_ ? "1" : "0", force);
    sendState("PER", perception_ready_ ? "1" : "0", force);
    sendState("FPS", fixed(camera_fps_, 1), force);
    sendState("OBJ", nearest_object_.substr(0, 23), force);
    sendState("DIST", fixed(nearest_distance_m_, 2), force);
    sendState("CONF", fixed(nearest_conf_pct_, 0), force);
    sendState("DRV", (drivable_valid_ && !perception_emergency_) ? "1" : "0", force);
    sendState("OBS", (perception_emergency_ || obstacle_count_ > 0) ? "1" : "0", force);
    sendState("LANE", lane_compact.substr(0, 19), force);

    sendState("MANUAL_SPEED", std::to_string(manual_speed_pct_), force);
    sendState("CFGSTEERTEST", fixed(steering_test_angle_deg_, 1), force);
    sendState("CFGERPMMPS", fixed(drive_erpm_per_mps_runtime_, 3), force);
    sendState("CFGPERINF", perception_inference_runtime_ ? "1" : "0", force);
    extendedTelemetryTick();
    mirrorWaypointState(force, force_critical);
  }

  void publishConnected(bool value) {
    connected_ = value;
    std_msgs::msg::Bool msg; msg.data = value; connected_pub_->publish(msg);
  }

  void publishState() {
    std_msgs::msg::String s;
    s.data = page_; page_pub_->publish(s);
    s.data = mode_; mode_pub_->publish(s);
    std::ostringstream json;
    json << "{\"connected\":" << (connected_ ? "true" : "false")
         << ",\"page\":\"" << page_ << "\",\"mode\":\"" << mode_
         << "\",\"drive\":\"" << drive_ << "\",\"steer\":\"" << steer_
         << "\",\"speed_pct\":" << manual_speed_pct_
         << ",\"steering_test_angle_deg\":" << steering_test_angle_deg_
         << ",\"origin\":\"" << control_origin_
         << "\",\"nav_state\":\"" << navigation_state_ << "\",\"target\":\"" << active_target_
         << "\",\"rejection\":\"" << last_rejection_ << "\"}";
    s.data = json.str(); manual_state_pub_->publish(s);
    std::ostringstream status;
    status << "{\"serial\":\"" << (connected_ ? "connected" : "offline")
           << "\",\"device\":\"" << (active_serial_device_.empty() ? serial_device_ : active_serial_device_)
           << "\",\"selector\":\"" << serial_device_ << "\",\"baud\":" << serial_baud_
           << ",\"mode\":\"" << mode_ << "\",\"page\":\"" << page_
           << "\",\"navigation\":\"" << navigation_state_
           << "\",\"target\":\"" << active_target_
           << "\",\"silent_open_failures\":" << silent_open_failures_
           << ",\"usb_recovery_requests\":" << usb_recovery_requests_ << "}";
    s.data = status.str(); status_pub_->publish(s);
    publishWaypointState();
  }

  std::string serial_device_, active_serial_device_;
  std::string waypoint_file_;
  int serial_baud_{1000000};
  double reconnect_sec_{0.5}, telemetry_rate_hz_{20.0}, serial_poll_hz_{1000.0}, command_rate_hz_{30.0}, heartbeat_sec_{5.0};
  double hmi_transport_timeout_sec_{2.0};
  double waypoint_pose_timeout_sec_{2.5};
  double neo3_sensor_timeout_sec_{2.0}, rm3100_mag_sigma_ut_{3.0};
  std::uint16_t neo3pro_navsat_service_mask_{sensor_msgs::msg::NavSatStatus::SERVICE_GPS};
  int neo3pro_gnss_rate_hz_{10};
  double neo3pro_min_usable_rate_hz_{7.0}, neo3pro_rate_reapply_sec_{60.0};
  double last_neo3_pvt_rate_hz_{std::numeric_limits<double>::quiet_NaN()};
  bool publish_stm32_gnss_{true};
  bool neo3_require_protocol_crc_{true};
  bool neo3_sequence_initialized_{false};
  int neo3_protocol_version_{0};
  std::uint32_t neo3_last_sequence_{0U};
  std::uint64_t neo3_sequence_gaps_{0U};
  std::uint64_t neo3_crc_errors_{0U};
  std::uint64_t neo3_duplicate_sequences_{0U};
  std::string neo3_gnss_frame_id_{"gnss_link"}, rm3100_mag_frame_id_{"gnss_link"}, neo3pro_baro_frame_id_{"gnss_link"};
  bool neo3_safety_button_as_estop_{false};
  double manual_speed_max_mps_{1.0}, manual_command_lease_sec_{0.30}, steering_test_angle_deg_{20.0},
         hmi_steer_full_scale_deg_{90.0}, teleop_yaw_max_rps_{80.0 * kPi / 180.0};
  int speed_min_pct_{10}, speed_max_pct_{50}, manual_speed_pct_{20};
  bool invert_hmi_steering_{true};
  int fd_{-1};
  int lock_fd_{-1};
  std::mutex tx_mutex_;
  bool connected_{false};
  std::string rx_, page_{"SPLASH"}, mode_{"AUTO"}, drive_{"STOP"}, steer_{"NONE"}, control_origin_{"NONE"}, last_rejection_;
  std::deque<std::string> best_effort_lines_;
  std::uint64_t serial_rx_backlog_drops_{0}, serial_best_effort_drops_{0};
  std::string navigation_state_{"IDLE"}, active_target_{"NONE"};
  std::string navigation_origin_{"NONE"}, last_goal_state_{"IDLE"};
  double steering_hmi_target_deg_{0.0};
  std::unordered_map<std::string, std::string> tx_cache_;
  std::chrono::steady_clock::time_point last_reconnect_try_{}, last_forced_tx_{}, last_rx_{}, serial_opened_at_{},
      last_usb_status_request_{}, last_compass_params_request_{}, last_usb_recovery_request_{}, last_map_pose_{},
      last_drive_command_time_{}, last_steer_command_time_{}, last_neo3_rate_command_{};
  std::uint32_t silent_open_failures_{0U};
  std::uint32_t usb_recovery_requests_{0U};
  std::uint64_t manual_lease_expirations_{0U};
  std::uint32_t host_session_token_{0U};
  std::uint32_t host_transport_generation_{0U};
  bool awaiting_host_session_{false};
  std::chrono::steady_clock::time_point last_host_hello_tx_{};
  std::string last_usb_status_{"UNKNOWN"};
  std::array<Waypoint, kWaypointCount> waypoints_{};
  int selected_waypoint_{0};
  bool have_map_pose_{false};
  double map_x_{0.0}, map_y_{0.0}, map_yaw_{0.0};

  bool gnss_ready_{false}, imu_ready_{false}, camera_ready_{false}, perception_ready_{false}, perception_emergency_{false};
  bool esc_ready_{false}, esc_feedback_{false}, steer_connected_{false}, motion_ready_{false}, nav2_ready_{false}, estop_{false};
  double latitude_{0.0}, longitude_{0.0}, hdop_{0.0}, hacc_m_{999.0}, gnss_age_sec_{99.0};
  double heading_deg_{0.0}, gyro_z_rps_{0.0}, drive_target_mps_{0.0}, drive_actual_mps_{0.0};
  double steering_target_rad_{0.0}, steering_actual_rad_{0.0}, motor_erpm_{0.0};
  double drive_erpm_per_mps_runtime_{1.0};
  bool perception_inference_runtime_{false};
  std::string localization_state_text_{"UNKNOWN"}, gnss_status_text_{"UNKNOWN"}, imu_status_text_{"UNKNOWN"};
  std::string ekf_local_status_text_{"UNKNOWN"}, ekf_global_status_text_{"UNKNOWN"}, lane_state_text_{"UNKNOWN"};
  int satellites_{0}, fix_type_{0}, obstacle_count_{0};
  bool drivable_valid_{false};
  bool neo3_gnss_connected_state_{false}, neo3_gnss_connected_initialized_{false};
  bool neo3_switch_state_{false}, neo3_switch_initialized_{false};
  uint64_t neo3_parse_errors_{0};
  bool neo3pro_active_{false};
  Neo3ProGnssMeta neo3pro_gnss_meta_{};
  Neo3ProCovariance neo3pro_gnss_cov_{};
  Neo3ProMagMeta neo3pro_mag_meta_{};
  Neo3ProHeading neo3pro_heading_{};
  Neo3ProNodeHealth neo3pro_node_health_{};
  Neo3ProGnssHealth neo3pro_gnss_health_{};
  std::unordered_map<std::string, Neo3ProWireState> neo3pro_wire_state_;
  std::chrono::steady_clock::time_point last_neo3pro_baro_time_{}, neo3pro_started_time_{};
  bool neo3pro_baro_connected_state_{false}, neo3pro_baro_connected_initialized_{false};
  bool neo3pro_node_connected_state_{false}, neo3pro_gnss_status_connected_state_{false};
  int neo3pro_timestamp_source_code_{3};
  std::chrono::steady_clock::time_point last_neo3_gnss_time_{}, last_neo3_mag_time_{};
  // Konversi epoch millis F411 ke ROS time memakai offset minimum yang diamati;
  // ini mempertahankan waktu pengukuran alih-alih waktu paket selesai diparse.
  bool mcu_clock_initialized_{false};
  std::uint32_t mcu_last_raw_ms_{0};
  std::uint64_t mcu_unwrapped_ms_{0};
  std::int64_t mcu_clock_offset_ns_{0};
  std::int64_t last_mcu_stamp_ns_{0};
  std::string nearest_object_{"NONE"};
  double nearest_distance_m_{0.0}, nearest_conf_pct_{0.0}, camera_fps_{0.0};
  // Extended HMI telemetry mirrors authoritative ROS sources; no duplicate control ownership.
  bool foc_values_valid_{false}, steering_cal_ext_valid_{false};
  double left_vbus_v_{0.0}, left_current_motor_a_{0.0}, left_current_in_a_{0.0}, left_id_a_{0.0}, left_iq_a_{0.0};
  double left_duty_{0.0}, left_temp_mos_c_{0.0}, left_erpm_{0.0}, left_position_deg_{0.0};
  double right_vbus_v_{0.0}, right_current_motor_a_{0.0}, right_current_in_a_{0.0}, right_id_a_{0.0}, right_iq_a_{0.0};
  double right_duty_{0.0}, right_temp_mos_c_{0.0}, right_erpm_{0.0};
  unsigned left_fault_{255U}, right_fault_{255U};
  bool steering_calibrated_ext_{false}, steering_homed_ext_{false}, steering_synced_ext_{false};
  int32_t steering_raw_count_ext_{0}, steering_span_ext_{0}, steering_raw_target_ext_{0};
  double perception_latency_ms_{0.0}, lane_center_offset_m_{0.0}, lane_confidence_pct_{0.0};
  double lane_road_width_m_{0.0}, lane_left_clearance_m_{0.0}, lane_right_clearance_m_{0.0}, lane_heading_error_deg_{0.0};
  bool lane_metric_valid_{false};
  unsigned perception_dropped_frames_{0U}, perception_raw_detection_count_{0U}, perception_confirmed_count_{0U};
  unsigned drivable_valid_rows_{0U}, drivable_sample_rows_{0U};
  std::string perception_backend_{"UNKNOWN"};
  double nearest_lateral_m_{0.0};
  int nearest_track_id_{-1};
  unsigned nearest_missed_frames_{0U};
  double map_cov_x_{0.0}, map_cov_y_{0.0}, map_yaw_var_{0.0};
  double odom_x_{0.0}, odom_y_{0.0}, odom_yaw_{0.0}, odom_linear_mps_{0.0}, odom_yaw_rate_rps_{0.0};
  double imu_yaw_deg_{0.0};
  std::string mppi_status_text_{"UNKNOWN"}, trajectory_state_text_{"UNKNOWN"};
  unsigned costmap_obstacle_cells_{0U};
  bool costmap_blocked_{false};
  std::chrono::steady_clock::time_point last_foc_telemetry_time_{}, last_camera_state_time_{}, last_performance_time_{},
      last_obstacle_time_{}, last_lane_metrics_time_{}, last_drivable_time_{}, last_imu_data_time_{},
      last_local_odom_time_{}, last_mppi_status_time_{}, last_trajectory_state_time_{}, last_costmap_time_{}, last_extended_tx_{},
      last_critical_state_tx_{};
  uint32_t extended_session_id_{0U};
  uint32_t escx_power_seq_{0U}, escx_motor_seq_{0U}, escx_encoder_seq_{0U}, escx_link_seq_{0U}, escx_perf_seq_{0U};
  uint32_t perx_cam_seq_{0U}, perx_det_seq_{0U}, perx_lane_seq_{0U}, perx_drv_seq_{0U}, perx_obs_seq_{0U}, perx_perf_seq_{0U};
  uint32_t navx_pose_seq_{0U}, navx_odom_seq_{0U}, navx_imu_seq_{0U}, navx_nav2_seq_{0U}, navx_cost_seq_{0U}, navx_ctrl_seq_{0U};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr page_pub_, mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr waypoints_pub_, navigation_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manual_state_pub_, status_pub_, source_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr commissioning_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr neo3_fix_raw_pub_, neo3_fix_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr neo3_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr neo3_quality_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr neo3_gnss_state_pub_, neo3_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr usb_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr neo3_gnss_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr neo3pro_rm3100_connected_pub_, neo3pro_baro_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr neo3pro_node_connected_pub_, neo3pro_gnss_status_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr neo3_safety_switch_pub_, neo3_estop_pub_, neo3pro_safety_button_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr neo3pro_mag_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr neo3pro_pressure_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr neo3pro_temperature_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr neo3pro_gnss_meta_pub_, neo3pro_gnss_cov_pub_, neo3pro_ecef_pub_, neo3pro_mag_meta_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr neo3pro_node_status_pub_, neo3pro_node_info_pub_, neo3pro_param_pub_, neo3pro_gnss_status_pub_, neo3pro_can_status_pub_, neo3pro_can_raw_pub_, neo3pro_health_pub_, neo3pro_dna_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr neo3pro_heading_pub_, neo3pro_heading_accuracy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr neo3pro_time_reference_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr neo3pro_safety_button_raw_pub_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_nav_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_, neo3_command_sub_, esc_status_sub_, obstacle_sub_, drivable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr foc_telemetry_sub_, lane_metrics_sub_, mppi_status_sub_, trajectory_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_connected_sub_, camera_healthy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr performance_sub_, nav_goal_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr map_pose_sub_, local_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr quality_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> bool_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> float_subs_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> string_subs_;
  std::shared_ptr<rclcpp::AsyncParametersClient> perception_params_, esc_params_;
  std::atomic_bool config_request_in_flight_{false};
  bool steering_test_active_{false}, steering_test_enable_in_flight_{false}, steering_test_disable_in_flight_{false};
  double steering_test_target_deg_{0.0};
  std::chrono::steady_clock::time_point steering_test_deadline_{};
  std::atomic<std::uint32_t> config_epoch_{0U}, config_sync_generation_{0U};
  rclcpp::TimerBase::SharedPtr reconnect_timer_, serial_timer_, telemetry_timer_, heartbeat_timer_, command_timer_, config_sync_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StmF4HmiBridge>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("stmf4_hmi_bridge"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
