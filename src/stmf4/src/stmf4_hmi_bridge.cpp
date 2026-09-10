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
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {
constexpr double kPi = 3.14159265358979323846;

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
      if (fd_ >= 0 && !vesc_maintenance_mode_) (void)sendLine("ROS:1");
      if (fd_ >= 0 && last_rx_.time_since_epoch().count() != 0 &&
          std::chrono::steady_clock::now() - last_rx_ > std::chrono::duration<double>(hmi_transport_timeout_sec_)) {
        closeSerial("transport heartbeat timeout");
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
    declare_parameter<std::string>("neo3_mag_frame_id", "gnss_link");
    declare_parameter<double>("neo3_mag_sigma_ut", 3.0);
    declare_parameter<bool>("publish_stm32_gnss", true);
    declare_parameter<bool>("neo3_require_protocol_crc", true);
    declare_parameter<double>("vesc_transport_timeout_sec", 2.0);
    declare_parameter<int>("vesc_uart_baud_expected", 115200);
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
    neo3_mag_frame_id_ = get_parameter("neo3_mag_frame_id").as_string();
    neo3_mag_sigma_ut_ = std::clamp(get_parameter("neo3_mag_sigma_ut").as_double(), 0.1, 100.0);
    publish_stm32_gnss_ = get_parameter("publish_stm32_gnss").as_bool();
    neo3_require_protocol_crc_ = get_parameter("neo3_require_protocol_crc").as_bool();
    vesc_transport_timeout_sec_ = std::clamp(get_parameter("vesc_transport_timeout_sec").as_double(), 0.25, 10.0);
    vesc_uart_baud_expected_ = static_cast<std::uint32_t>(std::clamp<std::int64_t>(get_parameter("vesc_uart_baud_expected").as_int(), 9600, 2000000));
    vesc_uart_baud_active_.store(vesc_uart_baud_expected_);
    if (serial_baud_ != 1000000) throw std::runtime_error("stmf4 requires serial_baud=1000000");
  }

  void initializeWaypoints() {
    const char *defaults[kWaypointCount] = {"Titik A", "Titik B", "Titik C", "Titik D"};
    for (size_t i = 0; i < kWaypointCount; ++i) waypoints_[i].name = defaults[i];
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
      } catch (...) {
        wp.saved = false;
      }
    }
  }

  bool persistWaypoints() {
    if (waypoint_file_.empty()) return false;
    try {
      const fs::path target(waypoint_file_);
      if (target.has_parent_path()) fs::create_directories(target.parent_path());
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
      }
      std::error_code ec;
      fs::rename(temp, target, ec);
      if (ec) {
        fs::copy_file(temp, target, fs::copy_options::overwrite_existing, ec);
        if (!ec) fs::remove(temp, ec);
      }
      return !ec;
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
    neo3_mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/neo3/mag", rclcpp::SensorDataQoS().keep_last(10));
    neo3_ist_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3/ist8310_connected", stateQos());
    neo3_safety_switch_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3/safety_switch", stateQos());
    // Satu publisher E-stop sistem: safety switch NEO3 dipetakan langsung ke
    // gate ROS, sementara F411 juga menghentikan F103 secara hardware-link lokal.
    neo3_estop_pub_ = create_publisher<std_msgs::msg::Bool>("/safety/estop", stateQos());
    neo3_status_pub_ = create_publisher<std_msgs::msg::String>("/neo3/status", stateQos());
    vesc_rx_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/stmf4/vesc/rx", rclcpp::QoS(rclcpp::KeepLast(16)).reliable());
    vesc_status_pub_ = create_publisher<std_msgs::msg::String>("/stmf4/vesc/status", stateQos());
    vesc_error_pub_ = create_publisher<std_msgs::msg::String>("/stmf4/vesc/error", stateQos());
    vesc_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/stmf4/vesc/connected", stateQos());
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/navigation/goal_request", 10);
    cancel_nav_client_ = create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");

    request_sub_ = create_subscription<std_msgs::msg::String>("/hmi/request", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { handleRequest(msg->data); });
    neo3_command_sub_ = create_subscription<std_msgs::msg::String>("/neo3/command", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const std::string command = upper(trim(msg->data));
        if (command == "LED:AUTO" || command == "LED:ON" || command == "LED:OFF" ||
            command == "BUZZER:OFF" || command == "STATUS" || command.rfind("BEEP:", 0) == 0) {
          (void)sendLine("NEO:" + command);
        } else {
          RCLCPP_WARN(get_logger(), "Rejected /neo3/command: %s", msg->data.c_str());
        }
      });
    vesc_runtime_tx_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>("/stmf4/vesc/runtime_tx", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg) { (void)sendVescBytes(msg->data, 'R'); });
    vesc_maintenance_tx_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>("/stmf4/vesc/maintenance_tx", rclcpp::QoS(100).reliable(),
      [this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg) { (void)sendVescBytes(msg->data, 'M'); });
    vesc_diagnostic_command_sub_ = create_subscription<std_msgs::msg::String>("/stmf4/vesc/diagnostic_command", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const std::string command = upper(trim(msg->data));
        if (command == "STATUS" || command == "LINECHECK" ||
            command == "BAUD:115200" || command == "BAUD:1000000" || command == "BAUD:2000000") {
          (void)sendLine("VESC:" + command);
        } else {
          RCLCPP_WARN(get_logger(), "Rejected /stmf4/vesc/diagnostic_command: %s", msg->data.c_str());
        }
      });
    vesc_mode_sub_ = create_subscription<std_msgs::msg::String>("/stmf4/vesc/mode", stateQos(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const std::string mode = upper(trim(msg->data));
        if (mode != "RUNTIME" && mode != "NORMAL" && mode != "MAINTENANCE") {
          RCLCPP_WARN(get_logger(), "Rejected /stmf4/vesc/mode: %s", msg->data.c_str());
          return;
        }
        const std::string route = mode == "NORMAL" ? "RUNTIME" : mode;
        // Cache desired ownership even while USB is temporarily disconnected.
        // reconnectSerial() will restore this exact route instead of forcing RUNTIME.
        vesc_desired_mode_ = route;
        vesc_maintenance_mode_ = route == "MAINTENANCE";
        (void)sendLine(std::string("VESC:MODE:") + route);
      });
    boolSub("/gnss/connected", gnss_ready_);
    boolSub("/imu/connected", imu_ready_);
    boolSub("/perception/camera_connected", camera_ready_);
    boolSub("/perception/camera_healthy", perception_ready_);
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
        if (!gnss_ready_) {
          heading_deg_ = yawFromQuat(msg->orientation) * 180.0 / kPi;
          if (heading_deg_ < 0.0) heading_deg_ += 360.0;
        }
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
            motor_mech_rpm_ = drive_motor_pole_pairs_ > 0 ? v / static_cast<double>(drive_motor_pole_pairs_) : 0.0;
          }
        }
      });
    obstacle_sub_ = create_subscription<std_msgs::msg::String>("/perception/obstacle_metrics", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { parseObstacle(msg->data); });
    drivable_sub_ = create_subscription<std_msgs::msg::String>("/perception/drivable_space", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) { if (auto v = jsonBool(msg->data, "valid")) drivable_valid_ = *v; });
    performance_sub_ = create_subscription<std_msgs::msg::String>("/perception/performance", rclcpp::SensorDataQoS(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        for (const char *key : {"fps", "pipeline_fps", "pipeline_fps_ema"}) {
          if (auto v = jsonNumber(msg->data, key)) { camera_fps_ = std::max(0.0, *v); break; }
        }
      });
    map_pose_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered_map", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto &p = msg->pose.pose.position;
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return;
        map_x_ = p.x; map_y_ = p.y; map_yaw_ = yawFromQuat(msg->pose.pose.orientation);
        have_map_pose_ = true; last_map_pose_ = std::chrono::steady_clock::now();
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

  void mirrorWaypointState(bool force) {
    for (size_t i = 0; i < kWaypointCount; ++i) {
      const Waypoint &wp = waypoints_[i];
      sendState("WP" + std::to_string(i), std::string(wp.saved ? "1:" : "0:") + wp.name, force);
    }
    sendState("WPSEL", std::to_string(selected_waypoint_), force);
    sendState("TARGET", active_target_.empty() ? "NONE" : active_target_, force);
    sendState("NAV", navigation_state_, force);
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

  void goWaypoint(int index, const std::string &origin) {
    if (!validWaypointIndex(index)) { reject("waypoint index must be 0..3"); return; }
    const Waypoint &wp = waypoints_[static_cast<size_t>(index)];
    if (!wp.saved) { reject("selected waypoint has not been saved"); sendLine("ERR:WP_NOT_SAVED"); return; }
    if (mode_ != "AUTO") { reject("waypoint navigation requires AUTO mode"); sendLine("ERR:WP_GO_REQUIRES_AUTO"); return; }
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
    if (!cancel_nav_client_->service_is_ready()) {
      if (!cancel_nav_client_->wait_for_service(250ms)) {
        reject("Nav2 cancel service unavailable");
        sendLine("ERR:NAV_CANCEL_UNAVAILABLE");
        return;
      }
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
    nearest_conf_pct_ = 0.0;
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
        const int cls = static_cast<int>(std::lround(jsonNumber(fragment, "class_id").value_or(-1.0)));
        nearest_object_ = className(cls);
        nearest_conf_pct_ = 100.0 * jsonNumber(fragment, "score").value_or(0.0);
      }
      cursor = fragment_end + 1;
    }
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
    publishConnected(false);
    RCLCPP_INFO(get_logger(), "HMI USB opened: %s (selector=%s)", active_serial_device_.c_str(), serial_device_.c_str());
    sendLine("ROS:1");
    sendLine("PING");
    sendLine("GET:STATE");
    sendLine("MODE:" + mode_);
    sendLine("NEO:LED:AUTO");
    sendLine("NEO:STATUS");
    sendLine("VESC:MODE:" + vesc_desired_mode_);
    sendLine("VESC:STATUS");
    return true;
  }

  void closeSerial(const char *reason) {
    if (fd_ >= 0) ::close(fd_);
    const bool was = connected_;
    fd_ = -1;
    active_serial_device_.clear();
    rx_.clear();
    if (was) RCLCPP_WARN(get_logger(), "HMI USB disconnected: %s", reason);
    publishConnected(false);
    publishNeo3Connected(false);
    if (neo3_ist_connected_state_) {
      neo3_ist_connected_state_ = false;
      std_msgs::msg::Bool b; b.data = false; neo3_ist_connected_pub_->publish(b);
    }
    publishVescConnected(false);
    last_neo3_gnss_time_ = {}; last_neo3_mag_time_ = {};
    last_vesc_line_time_ = {}; last_vesc_rx_time_ = {};
    drive_ = "STOP";
    steer_ = "NONE";
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
      // P1/P0 lines bypass the best-effort queue. In particular every VESC:RX
      // packet is published before any queued GNSS/MAG/HMI text processing.
      const bool urgent = line.rfind("VESC:", 0) == 0 ||
                          line.rfind("SENS:SW:", 0) == 0 ||
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
    estop_ = active;
    std_msgs::msg::Bool b;
    b.data = active;
    neo3_safety_switch_pub_->publish(b);
    neo3_estop_pub_->publish(b);
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
                        double hacc_m, double pdop) {
    std_msgs::msg::String state;
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "{\"source\":\"" << source << "\",\"transport\":\"stm32f411_usb_cdc\""
        << ",\"receiver_valid\":" << (receiver_valid ? "true" : "false")
        << ",\"fix_type\":" << fix_type << ",\"satellites\":" << satellites
        << ",\"hacc_m\":" << hacc_m << ",\"dop\":" << pdop
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
                              double flags2, double flags3, bool velocity_valid) {
    // A receiver can be fully connected and streaming NAV-PVT indoors while it
    // has no usable LLH yet. Keep quality/state telemetry alive in that case;
    // only position/velocity fusion measurements are suppressed.
    const bool coordinates_valid = std::isfinite(lat) && std::isfinite(lon) &&
      std::abs(lat) <= 90.0 && std::abs(lon) <= 180.0 &&
      !(std::abs(lat) < 1.0e-12 && std::abs(lon) < 1.0e-12);
    const bool qualified_fix = receiver_valid && coordinates_valid;
    const double h_sigma = std::clamp(std::isfinite(hacc) && hacc > 0.0 ? hacc : 100.0, 0.02, 1000.0);
    const double v_sigma = std::clamp(std::isfinite(vacc) && vacc > 0.0 ? vacc : h_sigma * 1.5, 0.03, 1500.0);

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = neo3_gnss_frame_id_;
    fix.status.status = qualified_fix ? sensor_msgs::msg::NavSatStatus::STATUS_FIX :
                                       sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    if (coordinates_valid) {
      fix.latitude = lat;
      fix.longitude = lon;
      fix.altitude = std::isfinite(alt) ? alt : 0.0;
      fix.position_covariance.fill(0.0);
      fix.position_covariance[0] = h_sigma * h_sigma;
      fix.position_covariance[4] = h_sigma * h_sigma;
      fix.position_covariance[8] = v_sigma * v_sigma;
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      neo3_fix_raw_pub_->publish(fix);
      neo3_fix_pub_->publish(fix);
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
      vel.twist.covariance[0] = sigma * sigma;
      vel.twist.covariance[7] = sigma * sigma;
      vel.twist.covariance[14] = sigma * sigma * 2.0;
      vel.twist.covariance[21] = 1.0e6;
      vel.twist.covariance[28] = 1.0e6;
      vel.twist.covariance[35] = 1.0e6;
      neo3_vel_pub_->publish(vel);
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    std_msgs::msg::Float64MultiArray quality;
    quality.data.assign(50, nan);
    quality.data[0] = static_cast<double>(satellites);
    quality.data[1] = pdop;
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
    quality.data[20] = 0.0;  // NAV-COV position valid
    quality.data[21] = 0.0;  // NAV-COV velocity valid
    quality.data[22] = pvt_rate_hz;
    quality.data[23] = std::max(0.0, (now() - stamp).seconds());
    quality.data[24] = 3.0;  // canonical timestamp source: 3 = F411 MCU millis mapped to ROS epoch
    quality.data[25] = flags2;
    quality.data[26] = flags3;
    quality.data[44] = qualified_fix ? 1.0 : 0.0;
    // 45..48 protocol diagnostics, append-only to preserve existing consumers.
    quality.data[45] = static_cast<double>(neo3_protocol_version_);
    quality.data[46] = static_cast<double>(neo3_last_sequence_);
    quality.data[47] = static_cast<double>(neo3_sequence_gaps_);
    quality.data[48] = static_cast<double>(neo3_crc_errors_ + neo3_duplicate_sequences_);
    quality.data[49] = velocity_valid ? 1.0 : 0.0;  // F411 extension: Twist covariance from NAV-PVT sAcc valid
    neo3_quality_pub_->publish(quality);

    last_neo3_gnss_time_ = std::chrono::steady_clock::now();
    publishNeo3Connected(true);
    publishGnssState(source_id == 1 ? "STM32_UBX_NAV_PVT" : "STM32_NMEA_FALLBACK",
                     qualified_fix, fix_type, satellites, h_sigma, pdop);
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
    const bool receiver_valid = fix_ok && !invalid_llh && (fix_type == 3 || fix_type == 4);
    const auto measurement_stamp = stampFromMcuMillis(v[1]);
    publishGnssMeasurement(measurement_stamp, 1, fix_type, receiver_valid,
      static_cast<int>(std::lround(v[6])), v[7], v[8], v[9], v[10], v[11],
      v[12], v[13], v[14], v[15], v[16], v[17], v[18], v[19], v[2], v[20],
      v[21], v[22], receiver_valid);
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

  void handleNeo3Mag(const std::string &payload) {
    std::vector<double> v;
    if (!parseCsvNumbers(payload, 7, v)) {
      ++neo3_parse_errors_;
      return;
    }
    if (v[6] <= 0.5) return;
    sensor_msgs::msg::MagneticField mag;
    mag.header.stamp = stampFromMcuMillis(v[1]);
    mag.header.frame_id = neo3_mag_frame_id_;
    mag.magnetic_field.x = v[2] * 1.0e-6;
    mag.magnetic_field.y = v[3] * 1.0e-6;
    mag.magnetic_field.z = v[4] * 1.0e-6;
    const double sigma_t = neo3_mag_sigma_ut_ * 1.0e-6;
    mag.magnetic_field_covariance.fill(0.0);
    mag.magnetic_field_covariance[0] = sigma_t * sigma_t;
    mag.magnetic_field_covariance[4] = sigma_t * sigma_t;
    mag.magnetic_field_covariance[8] = sigma_t * sigma_t;
    neo3_mag_pub_->publish(mag);
    last_neo3_mag_time_ = std::chrono::steady_clock::now();
    if (!neo3_ist_connected_state_) {
      neo3_ist_connected_state_ = true;
      std_msgs::msg::Bool b; b.data = true; neo3_ist_connected_pub_->publish(b);
    }
  }

  void handleNeo3Hardware(const std::string &payload) {
    std::vector<double> v;
    if (!parseCsvNumbers(payload, 8, v)) {
      ++neo3_parse_errors_;
      return;
    }
    const bool gnss_alive = v[2] > 0.5;
    const bool gnss_ready = v[3] > 0.5;
    const bool ist_ok = v[4] > 0.5;
    const bool sw = v[5] > 0.5;
    const bool led = v[6] > 0.5;
    // Host freshness of actual GNSS measurement frames is authoritative.
    // A 1-Hz hardware snapshot may briefly report not-alive between UART bursts;
    // never let that low-rate diagnostic flap /gnss/connected false.
    if (publish_stm32_gnss_ && gnss_alive) publishNeo3Connected(true);
    if (neo3_ist_connected_state_ != ist_ok) {
      neo3_ist_connected_state_ = ist_ok;
      std_msgs::msg::Bool b; b.data = ist_ok; neo3_ist_connected_pub_->publish(b);
    }
    publishNeo3SafetyState(sw);
    std_msgs::msg::String status;
    std::ostringstream out;
    out << "{\"gnss_alive\":" << (gnss_alive ? "true" : "false")
        << ",\"gnss_ready\":" << (gnss_ready ? "true" : "false")
        << ",\"ist8310\":" << (ist_ok ? "true" : "false")
        << ",\"safety_switch\":" << (sw ? "true" : "false")
        << ",\"safety_led\":" << (led ? "true" : "false")
        << ",\"gnss_config_attempts\":" << static_cast<int>(std::lround(v[7]));
    if (v.size() >= 11U) {
      out << ",\"ist_i2c_addr\":" << static_cast<int>(std::lround(v[8]))
          << ",\"ist_whoami\":" << static_cast<int>(std::lround(v[9]))
          << ",\"ist_init_error\":" << static_cast<int>(std::lround(v[10]));
    }
    if (v.size() >= 13U) {
      out << ",\"ist_sda_level\":" << static_cast<int>(std::lround(v[11]))
          << ",\"ist_scl_level\":" << static_cast<int>(std::lround(v[12]));
    }
    out << ",\"parse_errors\":" << neo3_parse_errors_ << "}";
    status.data = out.str();
    neo3_status_pub_->publish(status);
  }

  void handleNeo3Switch(const std::string &payload) {
    std::vector<double> v;
    if (!parseCsvNumbers(payload, 3, v)) return;
    publishNeo3SafetyState(v[2] > 0.5);
  }

  static int vescHexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  static std::string bytesToHex(const std::uint8_t *data, size_t size) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string out;
    out.resize(size * 2U);
    for (size_t i = 0; i < size; ++i) {
      out[i * 2U] = kHex[data[i] >> 4U];
      out[i * 2U + 1U] = kHex[data[i] & 0x0FU];
    }
    return out;
  }

  static bool hexToBytes(const std::string &hex, std::vector<std::uint8_t> *out) {
    if (out == nullptr || hex.empty() || (hex.size() & 1U) != 0U || hex.size() > 8192U) return false;
    out->clear();
    out->reserve(hex.size() / 2U);
    for (size_t i = 0; i < hex.size(); i += 2U) {
      const int hi = vescHexNibble(hex[i]);
      const int lo = vescHexNibble(hex[i + 1U]);
      if (hi < 0 || lo < 0) { out->clear(); return false; }
      out->push_back(static_cast<std::uint8_t>((hi << 4) | lo));
    }
    return true;
  }

  bool sendVescBytes(const std::vector<std::uint8_t> &bytes, char source) {
    if (bytes.empty() || bytes.size() > 4096U || (source != 'R' && source != 'M')) return false;
    // Startup/hot-plug gap is expected and fail-closed. Drop stale refreshes.
    if (fd_ < 0) {
      if (source == 'R') ++vesc_runtime_tx_rejected_;
      else ++vesc_maintenance_tx_rejected_;
      return false;
    }

    std::lock_guard<std::mutex> wire_lock(vesc_wire_tx_mutex_);
    /* /stmf4/vesc/mode is the ONLY ownership authority. Runtime batches are
     * latest-value traffic and are dropped on route mismatch; maintenance is
     * never interleaved with runtime bytes. */
    const bool maintenance = source == 'M';
    if (maintenance != vesc_maintenance_mode_) {
      if (maintenance) ++vesc_maintenance_tx_rejected_;
      else ++vesc_runtime_tx_rejected_;
      return false;
    }

    // USB CDC is much faster than USART1 and the F411 now owns a non-blocking
    // interrupt-driven UART TX ring. Do NOT sleep here to emulate USART timing:
    // blocking this ROS callback also delays serialTick(), which is the receive
    // path for F103 replies, GNSS and HMI. Backpressure belongs at the F411 ring.
    constexpr size_t kChunk = 240U;  // fits the F411 640-byte line parser as hex
    for (size_t offset = 0; offset < bytes.size(); offset += kChunk) {
      const size_t count = std::min(kChunk, bytes.size() - offset);
      const std::string line = std::string("VESC:TX:") + source + ":" +
        bytesToHex(bytes.data() + offset, count);
      const int max_attempts = source == 'R' ? 1 : 8;
      bool sent = false;
      for (int attempt = 0; attempt < max_attempts && fd_ >= 0; ++attempt) {
        if (sendLine(line, source == 'R' ? 0 : 1)) { sent = true; break; }
        if (attempt + 1 < max_attempts) std::this_thread::yield();
      }
      if (!sent) {
        if (source == 'R') ++vesc_runtime_tx_rejected_;
        else ++vesc_maintenance_tx_rejected_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "VESC USB batch dropped while F411 CDC is unavailable/backpressured");
        return false;
      }
    }
    return true;
  }

  void publishVescConnected(bool connected) {
    if (vesc_connected_initialized_ && connected == vesc_connected_state_) return;
    vesc_connected_initialized_ = true;
    vesc_connected_state_ = connected;
    std_msgs::msg::Bool msg;
    msg.data = connected;
    vesc_connected_pub_->publish(msg);
  }

  void handleVescLine(const std::string &line) {
    last_vesc_line_time_ = std::chrono::steady_clock::now();
    if (line.rfind("VESC:RX:", 0) == 0) {
      std::vector<std::uint8_t> bytes;
      if (!hexToBytes(line.substr(8), &bytes)) {
        ++vesc_parse_errors_;
        return;
      }
      std_msgs::msg::UInt8MultiArray msg;
      msg.data = std::move(bytes);
      vesc_rx_pub_->publish(msg);
      last_vesc_rx_time_ = last_vesc_line_time_;
      publishVescConnected(true);
      return;
    }
    if (line.rfind("VESC:STAT:", 0) == 0) {
      std_msgs::msg::String msg;
      msg.data = line.substr(10);
      vesc_status_pub_->publish(msg);
      std_msgs::msg::String clear_error; clear_error.data.clear(); vesc_error_pub_->publish(clear_error);
      const std::string payload = line.substr(10);
      const auto baud_pos = payload.find("baud=");
      if (baud_pos != std::string::npos) {
        char *baud_end = nullptr;
        const unsigned long reported = std::strtoul(payload.c_str() + baud_pos + 5, &baud_end, 10);
        if (baud_end != payload.c_str() + baud_pos + 5 && reported >= 9600UL && reported <= 2000000UL) {
          const auto previous = vesc_uart_baud_active_.exchange(static_cast<std::uint32_t>(reported));
          if (previous != reported) {
            RCLCPP_INFO(get_logger(), "F411<->F103 VESC UART baud synchronized from gateway: %lu", reported);
          }
          if (reported != vesc_uart_baud_expected_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
              "F411<->F103 baud=%lu differs from expected=%u; adaptive pacing active; Mini-PC<->F411 remains 1000000",
              reported, vesc_uart_baud_expected_);
          }
        }
      }
      const auto age_pos = payload.find("age_ms=");
      const auto rx_pos = payload.find("rx=");
      if (age_pos != std::string::npos && rx_pos != std::string::npos) {
        char *age_end = nullptr;
        char *rx_end = nullptr;
        const unsigned long age_ms = std::strtoul(payload.c_str() + age_pos + 7, &age_end, 10);
        const unsigned long rx_count = std::strtoul(payload.c_str() + rx_pos + 3, &rx_end, 10);
        if (age_end != payload.c_str() + age_pos + 7 && rx_end != payload.c_str() + rx_pos + 3 &&
            rx_count > 0UL && age_ms <= static_cast<unsigned long>(vesc_transport_timeout_sec_ * 1000.0)) {
          publishVescConnected(true);
        }
      }
      return;
    }
    if (line.rfind("VESC:LINE:", 0) == 0 || line.rfind("VESC:BAUD:", 0) == 0) {
      std_msgs::msg::String msg; msg.data = line.substr(5); vesc_status_pub_->publish(msg); return;
    }
    if (line.rfind("VESC:MODE:", 0) == 0) {
      std_msgs::msg::String msg; msg.data = line.substr(5); vesc_status_pub_->publish(msg); return;
    }
    if (line.rfind("VESC:ERR:", 0) == 0) {
      std_msgs::msg::String msg; msg.data = line.substr(9); vesc_error_pub_->publish(msg); return;
    }
  }

  void sensorWatchdogTick() {
    const auto t = std::chrono::steady_clock::now();
    if (publish_stm32_gnss_ && last_neo3_gnss_time_.time_since_epoch().count() != 0 &&
        t - last_neo3_gnss_time_ > std::chrono::duration<double>(neo3_sensor_timeout_sec_)) {
      publishNeo3Connected(false);
    }
    if (neo3_ist_connected_state_ && last_neo3_mag_time_.time_since_epoch().count() != 0 &&
        t - last_neo3_mag_time_ > std::chrono::duration<double>(neo3_sensor_timeout_sec_)) {
      neo3_ist_connected_state_ = false;
      std_msgs::msg::Bool b; b.data = false; neo3_ist_connected_pub_->publish(b);
    }
    if (vesc_connected_state_ && last_vesc_rx_time_.time_since_epoch().count() != 0 &&
        t - last_vesc_rx_time_ > std::chrono::duration<double>(vesc_transport_timeout_sec_)) {
      publishVescConnected(false);
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
                if (key == "DRVSCALE") drive_scale_runtime_ = actual;
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
    char *txn_end = nullptr;
    const unsigned long txn_raw = std::strtoul(txn_text.c_str(), &txn_end, 10);
    if (txn_end == txn_text.c_str() || *txn_end != '\0' || txn_raw == 0UL || txn_raw > 65535UL) return;
    const auto txn = static_cast<std::uint16_t>(txn_raw);
    const std::string key = upper(trim(body.substr(first + 1U, second - first - 1U)));
    const std::string raw = trim(body.substr(second + 1U));

    if (key == "MODE") {
      bool manual = false;
      if (!parseBool01(raw, &manual)) { sendConfigReply(false, txn, key, "INVALID_BOOL"); return; }
      config_epoch_.fetch_add(1U);
      setMode(manual ? "MANUAL" : "AUTO", "TFT_CFG", false);
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
    if (key == "DRVSCALE") {
      double value = 0.0;
      if (!parseFiniteDouble(raw, &value)) { sendConfigReply(false, txn, key, "INVALID_NUMBER"); return; }
      requestDoubleConfig(esc_params_, "drive_odometry_calibration_scale", txn, key, value, 0.20, 5.00);
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
    if (fd_ < 0 || vesc_maintenance_mode_ || config_request_in_flight_.load()) return;
    const std::uint32_t generation = config_sync_generation_.fetch_add(1U) + 1U;
    const std::uint32_t epoch = config_epoch_.load();
    if (esc_params_ && esc_params_->service_is_ready()) {
      esc_params_->get_parameters({"drive_odometry_calibration_scale", "drive_motor_pole_pairs"},
        [this, generation, epoch](auto future) {
          try {
            const auto values = future.get();
            if (generation != config_sync_generation_.load() || epoch != config_epoch_.load() ||
                config_request_in_flight_.load() || values.size() != 2U) return;
            if (values[0].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
              const double value = values[0].as_double();
              if (std::isfinite(value)) drive_scale_runtime_ = value;
            }
            if (values[1].get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
              drive_motor_pole_pairs_ = std::clamp(static_cast<int>(values[1].as_int()), 1, 100);
              motor_mech_rpm_ = motor_erpm_ / static_cast<double>(drive_motor_pole_pairs_);
            }
            sendState("CFGDRVSCALE", fixed(drive_scale_runtime_, 4), false);
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
    if (!connected_) publishConnected(true);
    if (line.rfind("VESC:", 0) == 0) { handleVescLine(line); return; }
    if (line.rfind("SENS:GNSS:", 0) == 0) { handleNeo3Gnss(line.substr(10)); return; }
    if (line.rfind("SENS:GNSSF:", 0) == 0) { handleNeo3GnssFallback(line.substr(11)); return; }
    if (line.rfind("SENS:MAG:", 0) == 0) { handleNeo3Mag(line.substr(9)); return; }
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

  void setMode(const std::string &requested, const std::string &origin, bool mirror) {
    const std::string next = requested == "MANUAL" ? "MANUAL" : "AUTO";
    mode_ = next;
    control_origin_ = origin;
    if (mode_ == "AUTO") {
      drive_ = "STOP"; steer_ = "NONE"; steering_hmi_target_deg_ = 0.0;
      if (steering_test_active_ || steering_test_enable_in_flight_) disableSteeringTest();
    }
    if (mirror) sendLine("MODE:" + mode_);
    publishState();
  }

  bool manualMotionAllowed(bool steering) {
    if (!connected_ || mode_ != "MANUAL" || estop_ || !esc_ready_ || !esc_feedback_) return false;
    return !steering || steer_connected_;
  }

  void reject(const std::string &reason) {
    last_rejection_ = reason;
    RCLCPP_WARN(get_logger(), "HMI manual request rejected: %s", reason.c_str());
    publishState();
  }

  void applyDrive(std::string action, const std::string &origin, bool mirror) {
    action = upper(trim(action));
    if (action == "STOP") {
      drive_ = "STOP"; control_origin_ = origin;
      (void)mirror;
      publishState(); return;
    }
    if (action != "FWD" && action != "REV") return;
    if (!manualMotionAllowed(false)) { reject("drive requires MANUAL + HMI + ESC ACK + E-STOP clear"); return; }
    drive_ = action; control_origin_ = origin; last_rejection_.clear();
    (void)mirror;
    publishState();
  }

  void applySteer(std::string action, const std::string &origin, bool mirror) {
    action = upper(trim(action));
    if (action != "LEFT" && action != "RIGHT" && action != "CENTER") return;
    if (!manualMotionAllowed(true)) { reject("steering requires MANUAL + HMI + ESC/encoder ready"); return; }
    steer_ = action; control_origin_ = origin; last_rejection_.clear();
    steering_hmi_target_deg_ = action == "LEFT" ? -hmi_steer_full_scale_deg_ : (action == "RIGHT" ? hmi_steer_full_scale_deg_ : 0.0);
    (void)mirror;
    publishState();
  }

  void commandTick() {
    publishSteeringTestTick();
    geometry_msgs::msg::Twist cmd;
    std_msgs::msg::String source;
    const bool allowed = manualMotionAllowed(false);
    const bool drive_active = allowed && (drive_ == "FWD" || drive_ == "REV");
    const bool steer_active = allowed && steer_connected_ && (steer_ == "LEFT" || steer_ == "RIGHT" || steer_ == "CENTER");
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

  void telemetryTick() {
    if (fd_ < 0 || vesc_maintenance_mode_) return;
    const auto now_steady = std::chrono::steady_clock::now();
    (void)now_steady;
    // Changed-only state transmission. openSerial() clears tx_cache_, so a CDC
    // reconnect already triggers a complete state resync. Periodically forcing
    // all ~40 fields at once created a deterministic burst that could delay
    // runtime_tx in the single-threaded executor.
    const bool force = false;

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

    sendState("SYS", estop_ ? "FAULT" : (ready ? "READY" : "NOT READY"), force);
    sendState("MODE", mode_, force);
    sendState("STATE", estop_ ? "FAULT" : (std::abs(drive_actual_mps_) > 0.02 ? "RUNNING" : "STOPPED"), force);
    sendState("ESTOP", estop_ ? "1" : "0", force);
    sendState("VESC_LINK", vesc_connected_state_ ? "1" : "0", force);
    sendState("ESC", (esc_ready_ && esc_feedback_) ? "1" : "0", force);
    sendState("ENC", (steer_connected_ && esc_feedback_) ? "1" : "0", force);
    sendState("MOTION", motion_ready_ ? "1" : "0", force);
    sendState("NAV2", nav2_ready_ ? "1" : "0", force);

    sendState("SPD", fixed(std::abs(drive_actual_mps_) * 3.6, 2), force);
    sendState("DRIVE_TGT", fixed(drive_target_mps_, 3), force);
    sendState("DRIVE_ACT", fixed(drive_actual_mps_, 3), force);
    sendState("ERPM", fixed(motor_erpm_, 1), force);
    sendState("RPM", fixed(motor_mech_rpm_, 1), force);
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
    sendState("MAG", neo3_ist_connected_state_ ? "1" : "0", force);
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
    sendState("CFGDRVSCALE", fixed(drive_scale_runtime_, 4), force);
    sendState("CFGPERINF", perception_inference_runtime_ ? "1" : "0", force);
    mirrorWaypointState(force);
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
           << "\",\"target\":\"" << active_target_ << "\"}";
    s.data = status.str(); status_pub_->publish(s);
    publishWaypointState();
  }

  std::string serial_device_, active_serial_device_;
  std::string waypoint_file_;
  int serial_baud_{1000000};
  double reconnect_sec_{0.5}, telemetry_rate_hz_{20.0}, serial_poll_hz_{1000.0}, command_rate_hz_{30.0}, heartbeat_sec_{5.0};
  double hmi_transport_timeout_sec_{2.0};
  double waypoint_pose_timeout_sec_{2.5};
  double neo3_sensor_timeout_sec_{2.0}, neo3_mag_sigma_ut_{3.0};
  double vesc_transport_timeout_sec_{2.0};
  std::uint32_t vesc_uart_baud_expected_{115200U};
  std::atomic<std::uint32_t> vesc_uart_baud_active_{115200U};
  bool publish_stm32_gnss_{true};
  bool neo3_require_protocol_crc_{true};
  bool neo3_sequence_initialized_{false};
  int neo3_protocol_version_{0};
  std::uint32_t neo3_last_sequence_{0U};
  std::uint64_t neo3_sequence_gaps_{0U};
  std::uint64_t neo3_crc_errors_{0U};
  std::uint64_t neo3_duplicate_sequences_{0U};
  std::string neo3_gnss_frame_id_{"gnss_link"}, neo3_mag_frame_id_{"gnss_link"};
  double manual_speed_max_mps_{1.0}, steering_test_angle_deg_{20.0},
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
  std::string vesc_desired_mode_{"RUNTIME"};
  std::string navigation_state_{"IDLE"}, active_target_{"NONE"};
  std::string navigation_origin_{"NONE"}, last_goal_state_{"IDLE"};
  double steering_hmi_target_deg_{0.0};
  std::unordered_map<std::string, std::string> tx_cache_;
  std::chrono::steady_clock::time_point last_reconnect_try_{}, last_forced_tx_{}, last_rx_{}, last_map_pose_{};
  std::array<Waypoint, kWaypointCount> waypoints_{};
  int selected_waypoint_{0};
  bool have_map_pose_{false};
  double map_x_{0.0}, map_y_{0.0}, map_yaw_{0.0};

  bool gnss_ready_{false}, imu_ready_{false}, camera_ready_{false}, perception_ready_{false}, perception_emergency_{false};
  bool esc_ready_{false}, esc_feedback_{false}, steer_connected_{false}, motion_ready_{false}, nav2_ready_{false}, estop_{false};
  double latitude_{0.0}, longitude_{0.0}, hdop_{0.0}, hacc_m_{999.0}, gnss_age_sec_{99.0};
  double heading_deg_{0.0}, gyro_z_rps_{0.0}, drive_target_mps_{0.0}, drive_actual_mps_{0.0};
  double steering_target_rad_{0.0}, steering_actual_rad_{0.0}, motor_erpm_{0.0}, motor_mech_rpm_{0.0};
  int drive_motor_pole_pairs_{15};
  double drive_scale_runtime_{1.0};
  bool perception_inference_runtime_{false};
  std::string localization_state_text_{"UNKNOWN"}, gnss_status_text_{"UNKNOWN"}, imu_status_text_{"UNKNOWN"};
  std::string ekf_local_status_text_{"UNKNOWN"}, ekf_global_status_text_{"UNKNOWN"}, lane_state_text_{"UNKNOWN"};
  int satellites_{0}, fix_type_{0}, obstacle_count_{0};
  bool drivable_valid_{false};
  bool neo3_gnss_connected_state_{false}, neo3_gnss_connected_initialized_{false};
  bool neo3_ist_connected_state_{false};
  bool neo3_switch_state_{false}, neo3_switch_initialized_{false};
  uint64_t neo3_parse_errors_{0};
  uint64_t vesc_parse_errors_{0};
  bool vesc_connected_state_{false}, vesc_connected_initialized_{false};
  std::chrono::steady_clock::time_point last_neo3_gnss_time_{}, last_neo3_mag_time_{};
  // Konversi epoch millis F411 ke ROS time memakai offset minimum yang diamati;
  // ini mempertahankan waktu pengukuran alih-alih waktu paket selesai diparse.
  bool mcu_clock_initialized_{false};
  std::uint32_t mcu_last_raw_ms_{0};
  std::uint64_t mcu_unwrapped_ms_{0};
  std::int64_t mcu_clock_offset_ns_{0};
  std::int64_t last_mcu_stamp_ns_{0};
  std::chrono::steady_clock::time_point last_vesc_line_time_{}, last_vesc_rx_time_{};
  std::string nearest_object_{"NONE"};
  double nearest_distance_m_{0.0}, nearest_conf_pct_{0.0}, camera_fps_{0.0};

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
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr vesc_rx_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr vesc_status_pub_, vesc_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vesc_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr neo3_gnss_connected_pub_, neo3_ist_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr neo3_safety_switch_pub_, neo3_estop_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr neo3_mag_pub_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_nav_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_, neo3_command_sub_, esc_status_sub_, obstacle_sub_, drivable_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr vesc_runtime_tx_sub_, vesc_maintenance_tx_sub_;
  bool vesc_maintenance_mode_{false};
  std::uint64_t vesc_runtime_tx_rejected_{0}, vesc_maintenance_tx_rejected_{0};
  std::mutex vesc_wire_tx_mutex_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vesc_mode_sub_, vesc_diagnostic_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr performance_sub_, nav_goal_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr map_pose_sub_;
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
