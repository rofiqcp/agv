#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr std::uint8_t kSof0 = 0xA5;
constexpr std::uint8_t kSof1 = 0x5A;
constexpr std::uint8_t kVersion = 0x01;
constexpr std::uint8_t kTypeCommand = 0x01;
constexpr std::uint8_t kTypeAck = 0x81;
constexpr std::uint8_t kFlagEstop = 0x01;
constexpr std::size_t kFrameSize = 14;
constexpr std::uint8_t kVescGetValues = 4;
constexpr std::uint8_t kVescSetRpm = 8;
constexpr std::uint8_t kVescSetPos = 9;
constexpr std::uint8_t kVescCustomAppData = 36;
constexpr std::uint8_t kHbMagic0 = 0x48;
constexpr std::uint8_t kHbMagic1 = 0x42;
constexpr std::uint8_t kHbVersion = 1;
constexpr std::uint8_t kHbSetSteeringDeg = 9;
constexpr std::uint8_t kHbGetSteeringCal = 10;
constexpr std::uint8_t kVescForwardCan = 34;
constexpr std::uint8_t kVescRightMotorId = 2;

rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

bool finiteTwist(const geometry_msgs::msg::Twist & msg)
{
  return std::isfinite(msg.linear.x) && std::isfinite(msg.angular.z);
}

std::uint16_t crc16Ccitt(const std::uint8_t * data, std::size_t len)
{
  std::uint16_t crc = 0xFFFFU;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U
        ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
        : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

void writeU16Le(std::uint8_t * p, std::uint16_t value)
{
  p[0] = static_cast<std::uint8_t>(value & 0xFFU);
  p[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

std::uint16_t vescCrc16(const std::uint8_t *data, std::size_t len)
{
  std::uint16_t crc = 0U;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8U;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U
        ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
        : static_cast<std::uint16_t>(crc << 1U);
    }
  }
  return crc;
}

void appendI32Be(std::vector<std::uint8_t> &out, std::int32_t value)
{
  const auto u = static_cast<std::uint32_t>(value);
  out.push_back(static_cast<std::uint8_t>(u >> 24U));
  out.push_back(static_cast<std::uint8_t>(u >> 16U));
  out.push_back(static_cast<std::uint8_t>(u >> 8U));
  out.push_back(static_cast<std::uint8_t>(u));
}

std::int16_t readI16Be(const std::uint8_t *p)
{
  return static_cast<std::int16_t>(
    static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[0]) << 8U) |
    static_cast<std::uint16_t>(p[1]));
}

std::int32_t readI32Be(const std::uint8_t *p)
{
  const std::uint32_t u =
    (static_cast<std::uint32_t>(p[0]) << 24U) |
    (static_cast<std::uint32_t>(p[1]) << 16U) |
    (static_cast<std::uint32_t>(p[2]) << 8U) |
    static_cast<std::uint32_t>(p[3]);
  return static_cast<std::int32_t>(u);
}

std::vector<std::uint8_t> makeVescFrame(const std::vector<std::uint8_t> &payload)
{
  std::vector<std::uint8_t> frame;
  if (payload.empty() || payload.size() > 65535U) return frame;
  if (payload.size() <= 255U) {
    frame.reserve(payload.size() + 5U);
    frame.push_back(2U);
    frame.push_back(static_cast<std::uint8_t>(payload.size()));
  } else {
    frame.reserve(payload.size() + 6U);
    frame.push_back(3U);
    frame.push_back(static_cast<std::uint8_t>(payload.size() >> 8U));
    frame.push_back(static_cast<std::uint8_t>(payload.size()));
  }
  frame.insert(frame.end(), payload.begin(), payload.end());
  const std::uint16_t crc = vescCrc16(payload.data(), payload.size());
  frame.push_back(static_cast<std::uint8_t>(crc >> 8U));
  frame.push_back(static_cast<std::uint8_t>(crc));
  frame.push_back(3U);
  return frame;
}

std::vector<std::uint8_t> wrapRightMotor(const std::vector<std::uint8_t> &inner)
{
  std::vector<std::uint8_t> payload;
  payload.reserve(inner.size() + 2U);
  payload.push_back(kVescForwardCan);
  payload.push_back(kVescRightMotorId);
  payload.insert(payload.end(), inner.begin(), inner.end());
  return payload;
}

std::uint16_t readU16Le(const std::uint8_t * p)
{
  return static_cast<std::uint16_t>(p[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(p[1]) << 8U);
}

std::int16_t readI16Le(const std::uint8_t * p)
{
  return static_cast<std::int16_t>(readU16Le(p));
}

}  // namespace

class EscAckermann final : public rclcpp::Node
{
public:
  EscAckermann()
  : Node("esc_ackermann")
  {
    declareParameters();
    readParameters();
    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&EscAckermann::onRuntimeParameters, this, std::placeholders::_1));
    createInterfaces();

    if (serial_enabled_ && transport_mode_ == "serial") startSerialThread();

    RCLCPP_INFO(get_logger(), "[ESC] Ackermann controller ready | calibrated CMD/FB topics + direct calibration jog enabled");
    RCLCPP_INFO(
      get_logger(),
      "ESC Ackermann integrated | priority TELEOP > NAV2 > IDLE | teleop=%s nav2=%s | transport=%s endpoint=%s",
      teleop_topic_.c_str(), nav2_topic_.c_str(), transport_mode_.c_str(),
      transport_mode_ == "stm32" ? "/stmf4/vesc/runtime_tx" : (serial_device_.empty() ? "AUTO" : serial_device_.c_str()));
    RCLCPP_INFO(
      get_logger(),
      "Drive scale: speed +/-%.3f m/s -> native VESC +/-%.1f ERPM (r=%.3fm, pp=%d, gear=%.3f); "
      "legacy serial limit +/-%.1f units | steering +/-%.1f deg | teleop yaw +/-%.1f deg/s",
      speed_max_mps_, speed_max_mps_ * nativeDriveErpmPerMps(), drive_wheel_radius_m_,
      drive_motor_pole_pairs_, drive_gear_ratio_, right_max_rpm_, steering_max_deg_, yaw_max_deg_s_);
    RCLCPP_INFO(
      get_logger(),
      "Steering feedback calibration: %s | CMD L/C/R=%+.3f/%+.3f/%+.3f deg | FB L/C/R=%+.3f/%+.3f/%+.3f deg | "
      "direct_mode=%s limit=%.1fdeg | saved=%s",
      (steering_feedback_calibration_enabled_ && steering_feedback_calibration_valid_) ? "ACTIVE" : "OFF",
      steering_feedback_left_stop_deg_, steering_feedback_center_deg_, steering_feedback_right_stop_deg_,
      steering_feedback_left_reference_deg_, steering_feedback_center_reference_deg_, steering_feedback_right_reference_deg_,
      steering_calibration_mode_enabled_ ? "ON" : "OFF", steering_calibration_direct_limit_deg_,
      steering_feedback_calibration_saved_at_.empty() ? "-" : steering_feedback_calibration_saved_at_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Steering PHYSICAL Part1: %s | wheel L/R=%+.2f/%+.2f deg | operational +/-%.2f deg | saved=%s",
      (steering_physical_calibration_enabled_ && steering_physical_calibration_valid_) ? "ACTIVE" : "NOT-CALIBRATED",
      steering_physical_left_limit_deg_, steering_physical_right_limit_deg_,
      operationalPhysicalLimitDeg(),
      steering_physical_calibration_saved_at_.empty() ? "-" : steering_physical_calibration_saved_at_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Steering center lock: %s | request<=%.2fdeg capture<=%.1fdeg fb_deadband=%.2fdeg "
      "P=%.2f adapt=%.2f/s max_trim=%.1fdeg trim_rate=%.1fdeg/s latch=%s",
      steering_center_hold_enabled_ ? "ON" : "OFF", steering_center_hold_request_deadband_deg_,
      steering_center_hold_capture_deg_, steering_center_hold_feedback_deadband_deg_,
      steering_center_hold_kp_, steering_center_hold_adapt_gain_per_sec_, steering_center_hold_max_trim_deg_,
      steering_center_hold_trim_rate_deg_s_,
      steering_center_hold_latch_inside_deadband_ ? "ON" : "OFF");
  }

  ~EscAckermann() override
  {
    requestSafeShutdown();
    if (transport_mode_ == "stm32") sendStm32SafeStop();
    stopSerialThread();
  }

  // Called from rclcpp::on_shutdown as soon as SIGINT/SIGTERM is observed.
  // Keep this callback lock-free/non-blocking: the serial worker performs the
  // bounded final safe-stop burst and owns the file descriptor close.
  void requestSafeShutdown() noexcept
  {
    safe_stop_requested_.store(true);
    serial_thread_running_.store(false);
  }

private:
  struct Sample
  {
    geometry_msgs::msg::Twist cmd{};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  struct Selected
  {
    geometry_msgs::msg::Twist twist{};
    std::string source{"IDLE"};
    bool estop{false};
    bool teleop{false};
    bool nav2{false};
  };

  struct SerialCommand
  {
    std::int16_t left_cdeg{0};
    std::int16_t right_rpm_x10{0};
    std::uint8_t flags{0};
  };

  void declareParameters()
  {
    declare_parameter<std::string>("teleop_topic", "/cmd_vel/teleop");
    declare_parameter<std::string>("teleop_source_topic", "/teleop/active_source");
    declare_parameter<std::string>("teleop_estop_topic", "/teleop/emergency_stop_latched");
    declare_parameter<std::string>("hmi_topic", "/hmi/cmd_vel");
    declare_parameter<std::string>("hmi_source_topic", "/hmi/active_source");
    declare_parameter<double>("hmi_timeout_sec", 0.30);
    declare_parameter<std::string>("nav2_topic", "/cmd_vel");
    declare_parameter<std::string>("output_topic", "/cmd_vel/actuator");
    declare_parameter<std::string>("active_source_topic", "/esc/mux/active_source");
    declare_parameter<std::string>("autonomy_gate_topic", "/system/autonomy_motion_allowed");
    declare_parameter<std::string>("global_estop_topic", "/safety/estop");
    declare_parameter<std::string>("maintenance_owner_topic", "/esc/vesc/maintenance_owner");
    declare_parameter<std::string>("perception_state_topic", "/navigation/trajectory_safety_state");

    declare_parameter<double>("command_rate_hz", 50.0);
    declare_parameter<double>("teleop_timeout_sec", 0.30);
    declare_parameter<double>("nav2_timeout_sec", 0.60);
    declare_parameter<double>("perception_state_timeout_sec", 0.75);
    declare_parameter<double>("manual_release_hold_sec", 0.50);
    declare_parameter<bool>("require_autonomy_gate", true);

    // Injected by esc.launch.py from the single esc/config/teleop.yaml source of truth.
    declare_parameter<double>("speed_max", 0.5);
    declare_parameter<double>("yaw_max_deg_s", 80.0);
    declare_parameter<double>("serial_left_max_deg", 90.0);
    // Legacy direct-serial command scale. Native VESC transport derives ERPM
    // physically from wheel radius, gear ratio and motor pole-pairs below.
    declare_parameter<double>("serial_right_max_rpm", 300.0);
    declare_parameter<double>("drive_wheel_radius_m", 0.145);
    declare_parameter<int>("drive_motor_pole_pairs", 15);
    declare_parameter<double>("drive_gear_ratio", 1.0);
    // Multiplicative calibration from raw wheel-model speed to measured ground speed.
    // scale = V_GNSS / V_raw_ESC. This is runtime-adjustable after N2.1 trials.
    declare_parameter<double>("drive_odometry_calibration_scale", 1.0);
    declare_parameter<double>("wheelbase_m", 0.70);
    declare_parameter<double>("track_width_m", 0.48);
    declare_parameter<double>("min_speed_for_nav_steering_mps", 0.05);
    declare_parameter<bool>("invert_steering", false);
    declare_parameter<bool>("invert_drive", false);

    // Persistent 3-point feedback calibration written by the GUI. The stored
    // readings use the legacy/un-calibrated steering convention after
    // invert_steering is applied, so the values match the old GUI display.
    declare_parameter<bool>("steering_feedback_calibration_enabled", false);
    declare_parameter<double>("steering_feedback_center_deg", 0.0);
    declare_parameter<double>("steering_feedback_right_stop_deg", 90.0);
    declare_parameter<double>("steering_feedback_left_stop_deg", -90.0);
    // Backward-compatible parameter only. Runtime uses independently captured RIGHT
    // and LEFT points directly; do not force equal raw encoder spans because the
    // steering linkage can have different transfer ratios on each side.
    declare_parameter<bool>("steering_feedback_force_symmetric_span", false);
    // Suppress tiny feedback/encoder jitter around the calibrated straight-ahead
    // position so odometry does not integrate a false yaw while driving straight.
    declare_parameter<double>("steering_straight_deadband_deg", 1.0);
    // Direction-dependent return error (backlash/static friction) can leave the
    // feedback on a different side of CENTER after a left vs right turn. When
    // the requested vehicle steering is straight, apply a small outer feedback
    // trim to the raw STM position command once the mechanism is near center.
    declare_parameter<bool>("steering_center_hold_enabled", true);
    declare_parameter<double>("steering_center_hold_request_deadband_deg", 0.75);
    declare_parameter<double>("steering_center_hold_capture_deg", 20.0);
    declare_parameter<double>("steering_center_hold_feedback_deadband_deg", 0.35);
    declare_parameter<double>("steering_center_hold_kp", 0.25);
    declare_parameter<double>("steering_center_hold_adapt_gain_per_sec", 1.50);
    // Deprecated compatibility parameters. Center hold intentionally has no integral term.
    declare_parameter<double>("steering_center_hold_ki", 0.0);
    declare_parameter<double>("steering_center_hold_integral_limit_deg_s", 0.0);
    declare_parameter<double>("steering_center_hold_max_trim_deg", 12.0);
    declare_parameter<double>("steering_center_hold_trim_rate_deg_s", 60.0);
    declare_parameter<bool>("steering_center_hold_latch_inside_deadband", true);
    declare_parameter<double>("steering_center_hold_feedback_timeout_sec", 0.30);
    declare_parameter<double>("diagnostic_trace_hz", 2.0);
    declare_parameter<std::string>("steering_feedback_calibration_saved_at", "");
    // The GUI writes a one-shot token with each atomic apply. Older runtimes that do
    // not declare this parameter cannot falsely report
    // that a live calibration was applied.
    declare_parameter<std::string>("steering_calibration_apply_token", "");

    // ROS-only measured calibration. The existing STM firmware remains protocol V1
    // (-90..+90 degrees). GUI capture stores BOTH the protocol command that placed the
    // wheel and the protocol feedback measured at that physical reference point.
    declare_parameter<bool>("steering_calibration_mode_enabled", false);
    declare_parameter<double>("steering_calibration_direct_limit_deg", 90.0);
    declare_parameter<double>("steering_feedback_left_reference_deg", -80.0);
    declare_parameter<double>("steering_feedback_center_reference_deg", 0.0);
    declare_parameter<double>("steering_feedback_right_reference_deg", 80.0);

    // Physical calibration: protocol -90/0/+90 is NOT a physical wheel angle. These
    // parameters are measured wheel angles relative to a physically straight
    // wheel. LEFT is negative and RIGHT is positive in the vehicle convention.
    // The operational limit is deliberately symmetric and must not exceed the
    // smaller measured mechanical side. It is what Teleop/Nav2 are allowed to request.
    declare_parameter<bool>("steering_physical_calibration_enabled", false);
    declare_parameter<double>("steering_physical_left_limit_deg", -30.0);
    declare_parameter<double>("steering_physical_right_limit_deg", 30.0);
    declare_parameter<double>("steering_physical_operational_limit_deg", 28.0);
    declare_parameter<std::string>("steering_physical_calibration_saved_at", "");

    // Multi-point physical steering LUT. Physical angles are strictly
    // increasing LEFT(negative) -> RIGHT(positive). Separate increasing and
    // decreasing sweeps preserve linkage hysteresis/backlash. The three-point
    // Part-1 calibration remains the fail-safe fallback when this LUT is disabled.
    declare_parameter<bool>("steering_physical_lut_enabled", false);
    declare_parameter<std::vector<double>>("steering_lut_physical_deg", {-30.0, 0.0, 30.0});
    declare_parameter<std::vector<double>>("steering_lut_command_increasing_deg", {-90.0, 0.0, 90.0});
    declare_parameter<std::vector<double>>("steering_lut_command_decreasing_deg", {-90.0, 0.0, 90.0});
    declare_parameter<std::vector<double>>("steering_lut_feedback_increasing_deg", {-90.0, 0.0, 90.0});
    declare_parameter<std::vector<double>>("steering_lut_feedback_decreasing_deg", {-90.0, 0.0, 90.0});
    declare_parameter<double>("steering_lut_direction_deadband_deg", 0.15);
    declare_parameter<std::string>("steering_lut_calibration_saved_at", "");

    declare_parameter<double>("steering_center_bias_from_left_deg", 0.0);
    declare_parameter<double>("steering_center_bias_from_right_deg", 0.0);

    // Transport is normally the STM32F411 gateway. Direct PL2303 is retained only
    // as a recovery/commissioning fallback so one process never competes for F103 UART.
    declare_parameter<std::string>("transport_mode", "stm32");
    declare_parameter<std::string>("stm32_tx_topic", "/stmf4/vesc/runtime_tx");
    declare_parameter<std::string>("stm32_rx_topic", "/stmf4/vesc/rx");
    declare_parameter<std::string>("stm32_connected_topic", "/stmf4/vesc/connected");
    // Keep the ROS control/localization stack alive when the actuator transport is
    // intentionally disabled. Disabled never means READY; it only suppresses TX/open/retry.
    declare_parameter<bool>("serial_enabled", true);
    declare_parameter<std::string>("serial_device", "auto");
    declare_parameter<std::string>(
      "serial_auto_id_contains", "Prolific_Technology_Inc._USB-Serial_Controller");
    // Current ESC UART is a Prolific PL2303 (067b:2303). Unique by-id is the
    // primary selector; physical topology is only a deterministic fallback.
    declare_parameter<std::string>("serial_auto_path_contains", "");
    declare_parameter<int>("serial_baud", 1000000);
    declare_parameter<double>("serial_tx_rate_hz", 50.0);
    declare_parameter<double>("serial_reconnect_sec", 0.25);
    declare_parameter<double>("serial_ack_timeout_sec", 0.60);
    declare_parameter<double>("command_watchdog_sec", 0.25);

    declare_parameter<std::string>("odom_topic", "/esc/odom");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_footprint");
    // Covariance is a confidence signal to robot_localization, not decoration.
    // Raise uncertainty when RPM tracking error is high and when the Ackermann
    // steering model is operating near its mechanical extremes.
    declare_parameter<double>("odom_v_variance_base", 0.03);
    declare_parameter<double>("odom_v_variance_rpm_error_gain", 0.20);
    declare_parameter<double>("odom_yaw_variance_base", 0.08);
    declare_parameter<double>("odom_yaw_variance_steer_gain", 0.30);
    declare_parameter<double>("odom_yaw_rate_variance_base", 0.05);
  }

  void readParameters()
  {
    teleop_topic_ = get_parameter("teleop_topic").as_string();
    teleop_source_topic_ = get_parameter("teleop_source_topic").as_string();
    teleop_estop_topic_ = get_parameter("teleop_estop_topic").as_string();
    hmi_topic_ = get_parameter("hmi_topic").as_string();
    hmi_source_topic_ = get_parameter("hmi_source_topic").as_string();
    hmi_timeout_sec_ = std::clamp(get_parameter("hmi_timeout_sec").as_double(), 0.05, 2.0);
    nav2_topic_ = get_parameter("nav2_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();
    active_source_topic_ = get_parameter("active_source_topic").as_string();
    autonomy_gate_topic_ = get_parameter("autonomy_gate_topic").as_string();
    global_estop_topic_ = get_parameter("global_estop_topic").as_string();
    maintenance_owner_topic_ = get_parameter("maintenance_owner_topic").as_string();
    perception_state_topic_ = get_parameter("perception_state_topic").as_string();

    command_rate_hz_ = std::clamp(get_parameter("command_rate_hz").as_double(), 10.0, 100.0);
    teleop_timeout_sec_ = std::clamp(get_parameter("teleop_timeout_sec").as_double(), 0.05, 2.0);
    nav2_timeout_sec_ = std::clamp(get_parameter("nav2_timeout_sec").as_double(), 0.05, 2.0);
    perception_state_timeout_sec_ = std::clamp(get_parameter("perception_state_timeout_sec").as_double(), 0.05, 2.0);
    manual_release_hold_sec_ = std::clamp(get_parameter("manual_release_hold_sec").as_double(), 0.0, 2.0);
    require_autonomy_gate_ = get_parameter("require_autonomy_gate").as_bool();

    speed_max_mps_ = std::max(0.01, get_parameter("speed_max").as_double());
    yaw_max_deg_s_ = std::clamp(get_parameter("yaw_max_deg_s").as_double(), 1.0, 180.0);
    steering_max_deg_ = std::clamp(get_parameter("serial_left_max_deg").as_double(), 1.0, 90.0);
    right_max_rpm_ = std::max(1.0, get_parameter("serial_right_max_rpm").as_double());
    drive_wheel_radius_m_ = std::clamp(get_parameter("drive_wheel_radius_m").as_double(), 0.01, 1.0);
    drive_motor_pole_pairs_ = std::clamp(static_cast<int>(get_parameter("drive_motor_pole_pairs").as_int()), 1, 100);
    drive_gear_ratio_ = std::clamp(get_parameter("drive_gear_ratio").as_double(), 0.01, 100.0);
    drive_odometry_calibration_scale_ = std::clamp(
      get_parameter("drive_odometry_calibration_scale").as_double(), 0.20, 5.0);
    wheelbase_m_ = std::max(0.05, get_parameter("wheelbase_m").as_double());
    track_width_m_ = std::max(0.0, get_parameter("track_width_m").as_double());
    min_speed_for_nav_steering_mps_ = std::max(
      0.001, get_parameter("min_speed_for_nav_steering_mps").as_double());
    invert_steering_ = get_parameter("invert_steering").as_bool();
    invert_drive_ = get_parameter("invert_drive").as_bool();

    steering_feedback_calibration_enabled_ =
      get_parameter("steering_feedback_calibration_enabled").as_bool();
    steering_feedback_center_deg_ = get_parameter("steering_feedback_center_deg").as_double();
    steering_feedback_right_stop_deg_ =
      get_parameter("steering_feedback_right_stop_deg").as_double();
    steering_feedback_left_stop_deg_ =
      get_parameter("steering_feedback_left_stop_deg").as_double();
    steering_feedback_force_symmetric_span_ =
      get_parameter("steering_feedback_force_symmetric_span").as_bool();
    if (steering_feedback_force_symmetric_span_) {
      RCLCPP_WARN(
        get_logger(),
        "steering_feedback_force_symmetric_span=true is deprecated; independent RIGHT/LEFT endpoints are used.");
      steering_feedback_force_symmetric_span_ = false;
    }
    steering_straight_deadband_deg_ = std::clamp(
      get_parameter("steering_straight_deadband_deg").as_double(), 0.0, 10.0);
    steering_center_hold_enabled_ = get_parameter("steering_center_hold_enabled").as_bool();
    steering_center_hold_request_deadband_deg_ = std::clamp(
      get_parameter("steering_center_hold_request_deadband_deg").as_double(), 0.0, 5.0);
    steering_center_hold_capture_deg_ = std::clamp(
      get_parameter("steering_center_hold_capture_deg").as_double(), 1.0, 45.0);
    steering_center_hold_feedback_deadband_deg_ = std::clamp(
      get_parameter("steering_center_hold_feedback_deadband_deg").as_double(), 0.05, 5.0);
    steering_center_hold_kp_ = std::clamp(
      get_parameter("steering_center_hold_kp").as_double(), 0.0, 1.0);
    steering_center_hold_adapt_gain_per_sec_ = std::clamp(
      get_parameter("steering_center_hold_adapt_gain_per_sec").as_double(), 0.0, 10.0);
    steering_center_hold_max_trim_deg_ = std::clamp(
      get_parameter("steering_center_hold_max_trim_deg").as_double(), 0.0, 25.0);
    steering_center_hold_trim_rate_deg_s_ = std::clamp(
      get_parameter("steering_center_hold_trim_rate_deg_s").as_double(), 1.0, 360.0);
    steering_center_hold_latch_inside_deadband_ =
      get_parameter("steering_center_hold_latch_inside_deadband").as_bool();
    steering_center_hold_feedback_timeout_sec_ = std::clamp(
      get_parameter("steering_center_hold_feedback_timeout_sec").as_double(), 0.05, 2.0);
    diagnostic_trace_hz_ = std::clamp(
      get_parameter("diagnostic_trace_hz").as_double(), 0.2, 20.0);
    steering_feedback_calibration_saved_at_ =
      get_parameter("steering_feedback_calibration_saved_at").as_string();
    steering_calibration_apply_token_ =
      get_parameter("steering_calibration_apply_token").as_string();
    steering_calibration_mode_enabled_ =
      get_parameter("steering_calibration_mode_enabled").as_bool();
    steering_calibration_direct_limit_deg_ = std::clamp(
      get_parameter("steering_calibration_direct_limit_deg").as_double(), 10.0, 90.0);
    steering_feedback_left_reference_deg_ =
      get_parameter("steering_feedback_left_reference_deg").as_double();
    steering_feedback_center_reference_deg_ =
      get_parameter("steering_feedback_center_reference_deg").as_double();
    steering_feedback_right_reference_deg_ =
      get_parameter("steering_feedback_right_reference_deg").as_double();
    steering_physical_calibration_enabled_ =
      get_parameter("steering_physical_calibration_enabled").as_bool();
    steering_physical_left_limit_deg_ =
      get_parameter("steering_physical_left_limit_deg").as_double();
    steering_physical_right_limit_deg_ =
      get_parameter("steering_physical_right_limit_deg").as_double();
    steering_physical_operational_limit_deg_ =
      get_parameter("steering_physical_operational_limit_deg").as_double();
    steering_physical_calibration_saved_at_ =
      get_parameter("steering_physical_calibration_saved_at").as_string();
    steering_physical_lut_enabled_ = get_parameter("steering_physical_lut_enabled").as_bool();
    steering_lut_physical_deg_ = get_parameter("steering_lut_physical_deg").as_double_array();
    steering_lut_command_increasing_deg_ = get_parameter("steering_lut_command_increasing_deg").as_double_array();
    steering_lut_command_decreasing_deg_ = get_parameter("steering_lut_command_decreasing_deg").as_double_array();
    steering_lut_feedback_increasing_deg_ = get_parameter("steering_lut_feedback_increasing_deg").as_double_array();
    steering_lut_feedback_decreasing_deg_ = get_parameter("steering_lut_feedback_decreasing_deg").as_double_array();
    steering_lut_direction_deadband_deg_ = std::clamp(
      get_parameter("steering_lut_direction_deadband_deg").as_double(), 0.01, 2.0);
    steering_lut_calibration_saved_at_ = get_parameter("steering_lut_calibration_saved_at").as_string();
    steering_center_bias_from_left_deg_ = std::clamp(
      get_parameter("steering_center_bias_from_left_deg").as_double(), -20.0, 20.0);
    steering_center_bias_from_right_deg_ = std::clamp(
      get_parameter("steering_center_bias_from_right_deg").as_double(), -20.0, 20.0);

    const double right_span = steering_feedback_right_stop_deg_ - steering_feedback_center_deg_;
    const double left_span = steering_feedback_left_stop_deg_ - steering_feedback_center_deg_;
    const double right_fb_span =
      steering_feedback_right_reference_deg_ - steering_feedback_center_reference_deg_;
    const double left_fb_span =
      steering_feedback_left_reference_deg_ - steering_feedback_center_reference_deg_;
    const auto command_in_range = [](double v) {
      return std::isfinite(v) && v >= -90.0 && v <= 90.0;
    };
    const auto feedback_in_range = [](double v) {
      return std::isfinite(v) && std::abs(v) <= 360.0;
    };
    steering_feedback_calibration_valid_ =
      command_in_range(steering_feedback_center_deg_) &&
      command_in_range(steering_feedback_right_stop_deg_) &&
      command_in_range(steering_feedback_left_stop_deg_) &&
      feedback_in_range(steering_feedback_center_reference_deg_) &&
      feedback_in_range(steering_feedback_right_reference_deg_) &&
      feedback_in_range(steering_feedback_left_reference_deg_) &&
      std::abs(right_span) >= 0.5 && std::abs(left_span) >= 0.5 &&
      (right_span * left_span) < 0.0 &&
      std::abs(right_fb_span) >= 0.5 && std::abs(left_fb_span) >= 0.5 &&
      (right_fb_span * left_fb_span) < 0.0;
    if (steering_feedback_calibration_enabled_ && !steering_feedback_calibration_valid_) {
      RCLCPP_WARN(
        get_logger(),
        "Steering feedback calibration ENABLED but invalid; fallback to legacy feedback. "
        "center=%.3f right=%.3f left=%.3f",
        steering_feedback_center_deg_, steering_feedback_right_stop_deg_,
        steering_feedback_left_stop_deg_);
    }

    const double physical_common_max = std::min(
      std::abs(steering_physical_left_limit_deg_),
      std::abs(steering_physical_right_limit_deg_));
    steering_physical_calibration_valid_ =
      std::isfinite(steering_physical_left_limit_deg_) &&
      std::isfinite(steering_physical_right_limit_deg_) &&
      std::isfinite(steering_physical_operational_limit_deg_) &&
      steering_physical_left_limit_deg_ < -1.0 &&
      steering_physical_right_limit_deg_ > 1.0 &&
      physical_common_max > 1.0 &&
      steering_physical_operational_limit_deg_ >= 1.0 &&
      steering_physical_operational_limit_deg_ <= physical_common_max + 1.0e-9;
    steering_physical_lut_valid_ = validateSteeringLut(
      steering_lut_physical_deg_, steering_lut_command_increasing_deg_,
      steering_lut_command_decreasing_deg_, steering_lut_feedback_increasing_deg_,
      steering_lut_feedback_decreasing_deg_);
    if (steering_physical_lut_enabled_ && !steering_physical_lut_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "Physical steering LUT invalid; disabling LUT and falling back to 3-point physical mapping.");
      steering_physical_lut_enabled_ = false;
    }

    if (steering_physical_calibration_enabled_ && !steering_physical_calibration_valid_) {
      RCLCPP_WARN(
        get_logger(),
        "Physical steering calibration ENABLED but invalid; falling back to legacy +/-%.1f deg. "
        "physical L/R=%+.3f/%+.3f operational=%.3f",
        steering_max_deg_, steering_physical_left_limit_deg_,
        steering_physical_right_limit_deg_, steering_physical_operational_limit_deg_);
    }

    // Use the independently captured raw point for each physical side.
    // Equal raw encoder spans are NOT required and must not be synthesized.
    steering_feedback_effective_right_stop_deg_ = steering_feedback_right_stop_deg_;
    steering_feedback_effective_left_stop_deg_ = steering_feedback_left_stop_deg_;

    transport_mode_ = get_parameter("transport_mode").as_string();
    std::transform(transport_mode_.begin(), transport_mode_.end(), transport_mode_.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (transport_mode_ != "stm32" && transport_mode_ != "serial") {
      throw std::runtime_error("transport_mode must be stm32 or serial");
    }
    stm32_tx_topic_ = get_parameter("stm32_tx_topic").as_string();
    stm32_rx_topic_ = get_parameter("stm32_rx_topic").as_string();
    stm32_connected_topic_ = get_parameter("stm32_connected_topic").as_string();
    serial_enabled_ = get_parameter("serial_enabled").as_bool();
    serial_device_ = get_parameter("serial_device").as_string();
    serial_auto_id_contains_ = get_parameter("serial_auto_id_contains").as_string();
    serial_auto_path_contains_ = get_parameter("serial_auto_path_contains").as_string();
    serial_baud_ = static_cast<int>(get_parameter("serial_baud").as_int());
    serial_tx_rate_hz_ = std::clamp(get_parameter("serial_tx_rate_hz").as_double(), 10.0, 100.0);
    serial_reconnect_sec_ = std::clamp(get_parameter("serial_reconnect_sec").as_double(), 0.05, 5.0);
    serial_ack_timeout_sec_ = std::clamp(get_parameter("serial_ack_timeout_sec").as_double(), 0.10, 5.0);
    command_watchdog_sec_ = std::clamp(get_parameter("command_watchdog_sec").as_double(), 0.05, 2.0);

    odom_topic_ = get_parameter("odom_topic").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    odom_v_variance_base_ = std::max(1.0e-6, get_parameter("odom_v_variance_base").as_double());
    odom_v_variance_rpm_error_gain_ = std::max(0.0, get_parameter("odom_v_variance_rpm_error_gain").as_double());
    odom_yaw_variance_base_ = std::max(1.0e-6, get_parameter("odom_yaw_variance_base").as_double());
    odom_yaw_variance_steer_gain_ = std::max(0.0, get_parameter("odom_yaw_variance_steer_gain").as_double());
    odom_yaw_rate_variance_base_ = std::max(1.0e-6, get_parameter("odom_yaw_rate_variance_base").as_double());
  }

  rcl_interfaces::msg::SetParametersResult onRuntimeParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;

    bool enabled = steering_feedback_calibration_enabled_;
    bool cal_mode = steering_calibration_mode_enabled_;
    double center_cmd = steering_feedback_center_deg_;
    double right_cmd = steering_feedback_right_stop_deg_;
    double left_cmd = steering_feedback_left_stop_deg_;
    double center_fb = steering_feedback_center_reference_deg_;
    double right_fb = steering_feedback_right_reference_deg_;
    double left_fb = steering_feedback_left_reference_deg_;
    bool physical_enabled = steering_physical_calibration_enabled_;
    double physical_left_deg = steering_physical_left_limit_deg_;
    double physical_right_deg = steering_physical_right_limit_deg_;
    double physical_operational_deg = steering_physical_operational_limit_deg_;
    std::string physical_saved_at = steering_physical_calibration_saved_at_;
    bool lut_enabled = steering_physical_lut_enabled_;
    std::vector<double> lut_physical = steering_lut_physical_deg_;
    std::vector<double> lut_cmd_inc = steering_lut_command_increasing_deg_;
    std::vector<double> lut_cmd_dec = steering_lut_command_decreasing_deg_;
    std::vector<double> lut_fb_inc = steering_lut_feedback_increasing_deg_;
    std::vector<double> lut_fb_dec = steering_lut_feedback_decreasing_deg_;
    double lut_direction_deadband = steering_lut_direction_deadband_deg_;
    std::string lut_saved_at = steering_lut_calibration_saved_at_;
    double bias_left = steering_center_bias_from_left_deg_;
    double bias_right = steering_center_bias_from_right_deg_;
    double direct_limit = steering_calibration_direct_limit_deg_;
    double max_deg = steering_max_deg_;
    bool force_symmetric = false;
    std::string saved_at = steering_feedback_calibration_saved_at_;
    std::string apply_token = steering_calibration_apply_token_;
    bool touched = false;
    bool serial_enabled_requested = serial_enabled_;
    bool serial_enabled_touched = false;
    double drive_scale = drive_odometry_calibration_scale_;
    bool drive_scale_touched = false;

    try {
      for (const auto & parameter : parameters) {
        const std::string & name = parameter.get_name();
        if (name == "serial_enabled") {
          serial_enabled_requested = parameter.as_bool(); serial_enabled_touched = true;
        } else if (name == "drive_odometry_calibration_scale") {
          drive_scale = parameter.as_double(); drive_scale_touched = true;
        } else if (name == "steering_feedback_calibration_enabled") {
          enabled = parameter.as_bool(); touched = true;
        } else if (name == "steering_calibration_mode_enabled") {
          cal_mode = parameter.as_bool(); touched = true;
        } else if (name == "steering_calibration_direct_limit_deg") {
          direct_limit = parameter.as_double(); touched = true;
        } else if (name == "steering_feedback_center_deg") {
          center_cmd = parameter.as_double(); touched = true;
        } else if (name == "steering_feedback_right_stop_deg") {
          right_cmd = parameter.as_double(); touched = true;
        } else if (name == "steering_feedback_left_stop_deg") {
          left_cmd = parameter.as_double(); touched = true;
        } else if (name == "steering_feedback_center_reference_deg") {
          center_fb = parameter.as_double(); touched = true;
        } else if (name == "steering_feedback_right_reference_deg") {
          right_fb = parameter.as_double(); touched = true;
        } else if (name == "steering_feedback_left_reference_deg") {
          left_fb = parameter.as_double(); touched = true;
        } else if (name == "steering_physical_calibration_enabled") {
          physical_enabled = parameter.as_bool(); touched = true;
        } else if (name == "steering_physical_left_limit_deg") {
          physical_left_deg = parameter.as_double(); touched = true;
        } else if (name == "steering_physical_right_limit_deg") {
          physical_right_deg = parameter.as_double(); touched = true;
        } else if (name == "steering_physical_operational_limit_deg") {
          physical_operational_deg = parameter.as_double(); touched = true;
        } else if (name == "steering_physical_calibration_saved_at") {
          physical_saved_at = parameter.as_string(); touched = true;
        } else if (name == "steering_physical_lut_enabled") {
          lut_enabled = parameter.as_bool(); touched = true;
        } else if (name == "steering_lut_physical_deg") {
          lut_physical = parameter.as_double_array(); touched = true;
        } else if (name == "steering_lut_command_increasing_deg") {
          lut_cmd_inc = parameter.as_double_array(); touched = true;
        } else if (name == "steering_lut_command_decreasing_deg") {
          lut_cmd_dec = parameter.as_double_array(); touched = true;
        } else if (name == "steering_lut_feedback_increasing_deg") {
          lut_fb_inc = parameter.as_double_array(); touched = true;
        } else if (name == "steering_lut_feedback_decreasing_deg") {
          lut_fb_dec = parameter.as_double_array(); touched = true;
        } else if (name == "steering_lut_direction_deadband_deg") {
          lut_direction_deadband = parameter.as_double(); touched = true;
        } else if (name == "steering_lut_calibration_saved_at") {
          lut_saved_at = parameter.as_string(); touched = true;
        } else if (name == "steering_center_bias_from_left_deg") {
          bias_left = parameter.as_double(); touched = true;
        } else if (name == "steering_center_bias_from_right_deg") {
          bias_right = parameter.as_double(); touched = true;
        } else if (name == "serial_left_max_deg") {
          max_deg = parameter.as_double(); touched = true;
        } else if (name == "steering_feedback_force_symmetric_span") {
          force_symmetric = parameter.as_bool(); touched = true;
        } else if (name == "steering_feedback_calibration_saved_at") {
          saved_at = parameter.as_string(); touched = true;
        } else if (name == "steering_calibration_apply_token") {
          apply_token = parameter.as_string(); touched = true;
        }
      }
    } catch (const std::exception & exc) {
      result.reason = std::string("parameter type invalid: ") + exc.what();
      return result;
    }

    if (drive_scale_touched) {
      if (!std::isfinite(drive_scale) || drive_scale < 0.20 || drive_scale > 5.0) {
        result.reason = "drive_odometry_calibration_scale harus finite dan 0.20..5.00";
        return result;
      }
      drive_odometry_calibration_scale_ = drive_scale;
      RCLCPP_INFO(get_logger(), "Drive velocity scale runtime updated: %.8f", drive_odometry_calibration_scale_);
    }

    if (!touched) {
      if (serial_enabled_touched) {
        setSerialEnabled(serial_enabled_requested);
        result.successful = true;
        result.reason = serial_enabled_requested ?
          "ESC serial enabled; reconnect worker started" :
          "ESC serial disabled; ROS node remains running and reports READY=false";
        return result;
      }
      result.successful = true;
      result.reason = drive_scale_touched ?
        "drive odometry scale diterapkan live" :
        "parameter tidak terkait steering calibration/serial";
      return result;
    }
    if (force_symmetric) {
      result.reason = "Kalibrasi ROS-only memakai titik LEFT/CENTER/RIGHT independen; force_symmetric_span harus false";
      return result;
    }
    const auto command_ok = [](double v) {
      return std::isfinite(v) && v >= -90.0 && v <= 90.0;
    };
    const auto feedback_ok = [](double v) {
      return std::isfinite(v) && std::abs(v) <= 360.0;
    };
    if (!command_ok(left_cmd) || !command_ok(center_cmd) || !command_ok(right_cmd)) {
      result.reason = "command steering ke STM harus finite dan berada pada -90..+90 deg";
      return result;
    }
    if (!feedback_ok(left_fb) || !feedback_ok(center_fb) || !feedback_ok(right_fb)) {
      result.reason = "feedback ACK steering harus finite dan berada dalam envelope +/-360 deg";
      return result;
    }
    if (!std::isfinite(max_deg) || max_deg < 1.0 || max_deg > 90.0) {
      result.reason = "serial_left_max_deg legacy fallback harus 1..90 deg";
      return result;
    }
    if (!std::isfinite(direct_limit) || direct_limit < 10.0 || direct_limit > 90.0) {
      result.reason = "steering_calibration_direct_limit_deg harus 10..90 deg";
      return result;
    }
    const double physical_common_max = std::min(
      std::abs(physical_left_deg), std::abs(physical_right_deg));
    const bool physical_geometry_valid =
      std::isfinite(physical_left_deg) && std::isfinite(physical_right_deg) &&
      std::isfinite(physical_operational_deg) &&
      physical_left_deg < -1.0 && physical_right_deg > 1.0 &&
      physical_common_max > 1.0 && physical_operational_deg >= 1.0 &&
      physical_operational_deg <= physical_common_max + 1.0e-9;
    if (physical_enabled && !physical_geometry_valid) {
      result.reason =
        "kalibrasi fisik invalid: LEFT harus negatif, RIGHT positif, dan operational <= sisi mekanik terkecil";
      return result;
    }
    const bool lut_geometry_valid = validateSteeringLut(
      lut_physical, lut_cmd_inc, lut_cmd_dec, lut_fb_inc, lut_fb_dec);
    if (lut_enabled && !lut_geometry_valid) {
      result.reason =
        "steering LUT invalid: butuh >=5 titik, physical/CMD/FB monoton naik, range valid, dan titik center";
      return result;
    }
    if (!std::isfinite(lut_direction_deadband) ||
        lut_direction_deadband < 0.01 || lut_direction_deadband > 2.0) {
      result.reason = "steering_lut_direction_deadband_deg harus 0.01..2.0 deg";
      return result;
    }

    if (!std::isfinite(bias_left) || !std::isfinite(bias_right) ||
        std::abs(bias_left) > 20.0 || std::abs(bias_right) > 20.0) {
      result.reason = "center bias harus finite dan dalam +/-20 deg";
      return result;
    }

    const double cmd_rspan = right_cmd - center_cmd;
    const double cmd_lspan = left_cmd - center_cmd;
    const double fb_rspan = right_fb - center_fb;
    const double fb_lspan = left_fb - center_fb;
    const bool geometry_valid =
      std::abs(cmd_rspan) >= 0.5 && std::abs(cmd_lspan) >= 0.5 && cmd_rspan * cmd_lspan < 0.0 &&
      std::abs(fb_rspan) >= 0.5 && std::abs(fb_lspan) >= 0.5 && fb_rspan * fb_lspan < 0.0;
    if (enabled && !geometry_valid) {
      result.reason = "kalibrasi invalid: CENTER harus berada di antara LEFT dan RIGHT pada command DAN feedback protocol";
      return result;
    }

    // Entering direct calibration mode must not jump the steering. Latch the
    // currently measured protocol feedback (display convention) as the first hold command.
    if (cal_mode && !steering_calibration_mode_enabled_) {
      // Hold the command that was already being sent, not the measured feedback,
      // so entering calibration mode does not intentionally create a position step.
      calibration_latched_command_deg_ = std::clamp(
        diagnostic_steering_raw_target_deg_, -direct_limit, direct_limit);
    }

    steering_feedback_calibration_enabled_ = enabled;
    steering_calibration_mode_enabled_ = cal_mode;
    steering_calibration_direct_limit_deg_ = direct_limit;
    steering_feedback_center_deg_ = center_cmd;
    steering_feedback_right_stop_deg_ = right_cmd;
    steering_feedback_left_stop_deg_ = left_cmd;
    steering_feedback_center_reference_deg_ = center_fb;
    steering_feedback_right_reference_deg_ = right_fb;
    steering_feedback_left_reference_deg_ = left_fb;
    steering_physical_calibration_enabled_ = physical_enabled;
    steering_physical_left_limit_deg_ = physical_left_deg;
    steering_physical_right_limit_deg_ = physical_right_deg;
    steering_physical_operational_limit_deg_ = physical_operational_deg;
    steering_physical_calibration_saved_at_ = physical_saved_at;
    steering_physical_calibration_valid_ = physical_geometry_valid;
    steering_physical_lut_enabled_ = lut_enabled;
    steering_physical_lut_valid_ = lut_geometry_valid;
    steering_lut_physical_deg_ = lut_physical;
    steering_lut_command_increasing_deg_ = lut_cmd_inc;
    steering_lut_command_decreasing_deg_ = lut_cmd_dec;
    steering_lut_feedback_increasing_deg_ = lut_fb_inc;
    steering_lut_feedback_decreasing_deg_ = lut_fb_dec;
    steering_lut_direction_deadband_deg_ = lut_direction_deadband;
    steering_lut_calibration_saved_at_ = lut_saved_at;
    steering_lut_motion_direction_ = 0;
    steering_lut_last_target_deg_ = 0.0;
    steering_center_bias_from_left_deg_ = bias_left;
    steering_center_bias_from_right_deg_ = bias_right;
    steering_feedback_force_symmetric_span_ = false;
    steering_feedback_effective_right_stop_deg_ = right_cmd;
    steering_feedback_effective_left_stop_deg_ = left_cmd;
    steering_feedback_calibration_valid_ = geometry_valid;
    steering_max_deg_ = std::clamp(max_deg, 1.0, 90.0);
    steering_feedback_calibration_saved_at_ = saved_at;
    steering_calibration_apply_token_ = apply_token;
    if (serial_enabled_touched) setSerialEnabled(serial_enabled_requested);

    resetCenterHold();
    last_steering_direction_ = 0;

    result.successful = true;
    std::ostringstream message;
    message << "LIVE steering calibration applied: CMD L/C/R="
            << left_cmd << "/" << center_cmd << "/" << right_cmd
            << " FB L/C/R=" << left_fb << "/" << center_fb << "/" << right_fb
            << " physical L/R=" << physical_left_deg << "/" << physical_right_deg
            << " operational +/-" << (physical_enabled ? physical_operational_deg : steering_max_deg_)
            << " cal_mode=" << (cal_mode ? "ON" : "OFF");
    result.reason = message.str();
    RCLCPP_WARN(
      get_logger(),
      "[CAL-APPLY] %s | CMD L/C/R=%+.3f/%+.3f/%+.3f | FB L/C/R=%+.3f/%+.3f/%+.3f | logical +/-%.1f",
      enabled ? "ACTIVE" : "OFF", left_cmd, center_cmd, right_cmd,
      left_fb, center_fb, right_fb, steering_max_deg_);
    return result;
  }

  void createInterfaces()
  {
    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      teleop_topic_, cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!finiteTwist(*msg)) {
          teleop_.valid = false;
          return;
        }
        teleop_.cmd = *msg;
        teleop_.received = now();
        teleop_.valid = true;
      });

    teleop_source_sub_ = create_subscription<std_msgs::msg::String>(
      teleop_source_topic_, cmd_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const auto t = now();
        teleop_source_ = msg->data;
        teleop_source_received_ = t;
        const bool active = msg->data != "STOP" && msg->data != "IDLE" && msg->data != "";
        if (active && msg->data != "E_STOP") {
          teleop_takeover_until_ = t + rclcpp::Duration::from_seconds(manual_release_hold_sec_);
        }
      });

    hmi_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      hmi_topic_, cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!finiteTwist(*msg)) { hmi_.valid = false; return; }
        hmi_.cmd = *msg;
        hmi_.received = now();
        hmi_.valid = true;
      });

    hmi_source_sub_ = create_subscription<std_msgs::msg::String>(
      hmi_source_topic_, cmd_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        hmi_source_ = msg->data;
        hmi_source_received_ = now();
      });

    teleop_estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      teleop_estop_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        teleop_estop_ = msg->data;
      });

    nav2_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav2_topic_, cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!finiteTwist(*msg)) {
          nav2_.valid = false;
          return;
        }
        nav2_.cmd = *msg;
        nav2_.received = now();
        nav2_.valid = true;
      });

    autonomy_gate_sub_ = create_subscription<std_msgs::msg::Bool>(
      autonomy_gate_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        autonomy_gate_ = msg->data;
        autonomy_gate_seen_ = true;
      });

    global_estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      global_estop_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        global_estop_ = msg->data;
      });

    maintenance_owner_sub_ = create_subscription<std_msgs::msg::String>(
      maintenance_owner_topic_, stateQos(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        maintenance_owner_ = msg->data.empty() ? "MAINTENANCE" : msg->data;
      });

    perception_state_sub_ = create_subscription<std_msgs::msg::String>(
      perception_state_topic_, rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const std::string key = "decision=";
        const auto begin = msg->data.find(key);
        if (begin == std::string::npos) return;
        const auto value_begin = begin + key.size();
        const auto end = msg->data.find(';', value_begin);
        const std::string decision = msg->data.substr(value_begin, end - value_begin);
        std::lock_guard<std::mutex> lock(state_mutex_);
        perception_decision_ = decision;
        perception_state_received_ = now();
        perception_intervening_ = !decision.empty() &&
          decision != "NAV2_PASS" && decision != "GUARD_ONLY_PASS";
      });

    actuator_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, cmd_qos);
    active_source_pub_ = create_publisher<std_msgs::msg::String>(active_source_topic_, stateQos());
    status_pub_ = create_publisher<std_msgs::msg::String>("/esc/status", stateQos());
    foc_telemetry_pub_ = create_publisher<std_msgs::msg::String>("/esc/foc/telemetry", stateQos());
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/ready", stateQos());
    armed_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/armed", stateQos());
    feedback_valid_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/feedback_valid", stateQos());
    drive_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/drive/connected", stateQos());
    steer_connected_pub_ = create_publisher<std_msgs::msg::Bool>("/esc/steer/connected", stateQos());
    calibration_controller_ready_pub_ =
      create_publisher<std_msgs::msg::Bool>("/esc/steering_calibration/controller_ready", stateQos());
    {
      std_msgs::msg::Bool v;
      v.data = true;
      calibration_controller_ready_pub_->publish(v);
    }
    speed_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/speed", 10);
    drive_target_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/drive_target_mps", 10);
    drive_actual_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/drive_actual_mps", 10);
    drive_raw_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/drive_raw_mps", 10);
    steering_target_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/steering_target_rad", 10);
    // Command target in the same uncalibrated convention used by saved CENTER/RIGHT/LEFT.
    steering_uncal_target_pub_ =
      create_publisher<std_msgs::msg::Float64>("/esc/steering_command_uncalibrated_rad", 10);
    // Uncalibrated keeps the exact legacy feedback visible for debugging and GUI capture.
    steering_uncalibrated_pub_ =
      create_publisher<std_msgs::msg::Float64>("/esc/steering_uncalibrated_rad", 10);
    steering_protocol_command_pub_ =
      create_publisher<std_msgs::msg::Float64>("/esc/steering_protocol_command_rad", 10);
    steering_feedback_raw_pub_ =
      create_publisher<std_msgs::msg::Float64>("/esc/steering_feedback_raw_rad", 10);
    // Actual is the calibrated steering used by odometry/RViz/EKF.
    steering_actual_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/steering_actual_rad", 10);
    // Kinematic yaw rate derived from measured drive RPM + calibrated steering.
    // Keep the legacy topic for compatibility, but expose an explicit name so
    // localization/GUI never mistake this model-derived value for a gyro.
    yaw_rate_actual_pub_ = create_publisher<std_msgs::msg::Float64>("/esc/yaw_rate_actual_rps", 10);
    yaw_rate_kinematic_pub_ =
      create_publisher<std_msgs::msg::Float64>("/esc/kinematic_yaw_rate_rps", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    // F411 owns the physical USB CDC. Ackermann exchanges the exact existing
    // 14-byte actuator frames through ROS byte arrays; direct serial remains fallback.
    stm32_tx_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(stm32_tx_topic_, rclcpp::QoS(100).reliable());
    stm32_rx_sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      stm32_rx_topic_, rclcpp::QoS(100).reliable(),
      [this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr msg) {
        if (transport_mode_ != "stm32") return;
        consumeVescRxBytes(msg->data);
      });
    stm32_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      stm32_connected_topic_, stateQos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        if (transport_mode_ != "stm32") return;
        serial_connected_.store(serial_enabled_ && msg->data);
        std::lock_guard<std::mutex> lock(serial_state_mutex_);
        serial_active_path_ = msg->data ? "stm32f411:PB6/PB7->f103:PB11/PB10" : "";
      });
    maintenance_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/vesc/maintenance_active", stateQos(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        bool entering = false;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          entering = msg->data && !maintenance_mode_active_;
          maintenance_mode_active_ = msg->data;
          if (!msg->data) maintenance_owner_ = "RUNTIME";
        }
        if (entering) sendStm32SafeStop();
        if (msg->data) ack_timeout_.store(true);
      });
    const auto transport_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / serial_tx_rate_hz_));
    stm32_transport_timer_ = create_wall_timer(transport_period, std::bind(&EscAckermann::stm32TransportTick, this));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / command_rate_hz_));
    control_timer_ = create_wall_timer(period, std::bind(&EscAckermann::controlTick, this));
  }

  Selected selectCommand(const rclcpp::Time & t)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    Selected selected;

    const auto fresh_stamp = [&t](const rclcpp::Time & stamp, double timeout_sec) {
      if (stamp.nanoseconds() <= 0) return false;
      const double age = (t - stamp).seconds();
      return age >= 0.0 && age <= timeout_sec;
    };
    const bool source_fresh =
      fresh_stamp(teleop_source_received_, teleop_timeout_sec_);
    const bool teleop_fresh = teleop_.valid && fresh_stamp(teleop_.received, teleop_timeout_sec_);
    const bool hmi_source_fresh = fresh_stamp(hmi_source_received_, hmi_timeout_sec_);
    const bool hmi_fresh = hmi_.valid && fresh_stamp(hmi_.received, hmi_timeout_sec_);
    const bool hmi_active = hmi_source_fresh && hmi_source_ != "STOP" && hmi_source_ != "IDLE" && hmi_source_ != "";
    const bool nav2_fresh = nav2_.valid && fresh_stamp(nav2_.received, nav2_timeout_sec_);
    const bool perception_state_fresh = perception_intervening_ &&
      fresh_stamp(perception_state_received_, perception_state_timeout_sec_);
    const bool teleop_active = source_fresh &&
      teleop_source_ != "STOP" && teleop_source_ != "IDLE" && teleop_source_ != "E_STOP" &&
      teleop_source_ != "";
    const bool teleop_hold = teleop_takeover_until_.nanoseconds() > 0 &&
      t.nanoseconds() >= 0 && t <= teleop_takeover_until_;
    const bool estop = global_estop_ || teleop_estop_ || (source_fresh && teleop_source_ == "E_STOP");
    const bool gate_ok = !require_autonomy_gate_ || (autonomy_gate_seen_ && autonomy_gate_);

    if (estop) {
      selected.estop = true;
      selected.source = "E_STOP";
      return selected;
    }

    // TELEOP, physical HMI and ROS Web all belong to the same MANUAL tier.
    // When more than one manual source is active, the freshest command wins;
    // this avoids an arbitrary permanent priority inside the operator tier.
    const bool hmi_manual = hmi_fresh && hmi_active;
    const bool teleop_manual = teleop_fresh && (teleop_active || teleop_hold);
    if (hmi_manual || teleop_manual) {
      const bool choose_hmi = hmi_manual &&
        (!teleop_manual || hmi_.received.nanoseconds() >= teleop_.received.nanoseconds());
      if (choose_hmi) {
        selected.twist = clampTwist(hmi_.cmd);
        selected.source = hmi_source_;
      } else {
        selected.twist = clampTwist(teleop_.cmd);
        selected.source = teleop_active ? "TELEOP" : "TELEOP_RELEASE_HOLD";
      }
      selected.teleop = true;
      return selected;
    }

    if (nav2_fresh && gate_ok) {
      selected.twist = clampTwist(nav2_.cmd);
      selected.nav2 = true;
      selected.source = perception_state_fresh ?
        (std::string("PERCEPTION:") + perception_decision_) : "NAV2";
      return selected;
    }

    selected.source = gate_ok ? "IDLE" : "NAV2_GATE_CLOSED";
    return selected;
  }

  geometry_msgs::msg::Twist clampTwist(const geometry_msgs::msg::Twist & in) const
  {
    geometry_msgs::msg::Twist out{};
    out.linear.x = std::clamp(in.linear.x, -speed_max_mps_, speed_max_mps_);
    const double max_yaw_rps = yaw_max_deg_s_ * kPi / 180.0;
    out.angular.z = std::clamp(in.angular.z, -max_yaw_rps, max_yaw_rps);
    return out;
  }

  static bool strictlyIncreasing(const std::vector<double> & values)
  {
    if (values.size() < 2U) return false;
    for (std::size_t i = 1; i < values.size(); ++i) {
      if (!std::isfinite(values[i]) || !(values[i] > values[i - 1] + 1.0e-6)) return false;
    }
    return std::isfinite(values.front());
  }

  static bool validateSteeringLut(
    const std::vector<double> & physical,
    const std::vector<double> & cmd_inc,
    const std::vector<double> & cmd_dec,
    const std::vector<double> & fb_inc,
    const std::vector<double> & fb_dec)
  {
    const std::size_t n = physical.size();
    if (n < 5U || n > 41U || cmd_inc.size() != n || cmd_dec.size() != n ||
        fb_inc.size() != n || fb_dec.size() != n) return false;
    if (!strictlyIncreasing(physical) || !strictlyIncreasing(cmd_inc) ||
        !strictlyIncreasing(cmd_dec) || !strictlyIncreasing(fb_inc) ||
        !strictlyIncreasing(fb_dec)) return false;
    if (!(physical.front() < -1.0 && physical.back() > 1.0)) return false;
    bool has_center = false;
    for (std::size_t i = 0; i < n; ++i) {
      if (std::abs(physical[i]) <= 1.0) has_center = true;
      if (std::abs(cmd_inc[i]) > 90.0 || std::abs(cmd_dec[i]) > 90.0 ||
          std::abs(fb_inc[i]) > 360.0 || std::abs(fb_dec[i]) > 360.0) return false;
    }
    return has_center;
  }

  static double interpolateMonotonic(
    double x, const std::vector<double> & xs, const std::vector<double> & ys)
  {
    if (xs.empty() || xs.size() != ys.size()) return 0.0;
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    const auto upper = std::upper_bound(xs.begin(), xs.end(), x);
    const std::size_t hi = static_cast<std::size_t>(std::distance(xs.begin(), upper));
    const std::size_t lo = hi - 1U;
    const double span = xs[hi] - xs[lo];
    if (span <= 1.0e-12) return ys[lo];
    const double ratio = std::clamp((x - xs[lo]) / span, 0.0, 1.0);
    return ys[lo] + ratio * (ys[hi] - ys[lo]);
  }

  static std::vector<double> averageLut(
    const std::vector<double> & a, const std::vector<double> & b)
  {
    if (a.size() != b.size()) return a;
    std::vector<double> out(a.size(), 0.0);
    for (std::size_t i = 0; i < a.size(); ++i) out[i] = 0.5 * (a[i] + b[i]);
    return out;
  }

  int updateSteeringLutDirection(double physical_deg)
  {
    const double delta = physical_deg - steering_lut_last_target_deg_;
    if (delta > steering_lut_direction_deadband_deg_) steering_lut_motion_direction_ = +1;
    else if (delta < -steering_lut_direction_deadband_deg_) steering_lut_motion_direction_ = -1;
    steering_lut_last_target_deg_ = physical_deg;
    return steering_lut_motion_direction_;
  }

  double leftPhysicalLimitDeg() const
  {
    if (steering_physical_calibration_enabled_ && steering_physical_calibration_valid_) {
      return steering_physical_left_limit_deg_;
    }
    // Before physical calibration, never reinterpret the legacy protocol span
    // (typically +/-80) as a real wheel angle. The configured physical values
    // are conservative commissioning fallbacks only.
    return -std::min(steering_max_deg_, std::abs(steering_physical_left_limit_deg_));
  }

  double rightPhysicalLimitDeg() const
  {
    if (steering_physical_calibration_enabled_ && steering_physical_calibration_valid_) {
      return steering_physical_right_limit_deg_;
    }
    return std::min(steering_max_deg_, std::abs(steering_physical_right_limit_deg_));
  }

  double operationalPhysicalLimitDeg() const
  {
    if (steering_physical_calibration_enabled_ && steering_physical_calibration_valid_) {
      return steering_physical_operational_limit_deg_;
    }
    const double fallback_mechanical = std::min(
      std::abs(leftPhysicalLimitDeg()), std::abs(rightPhysicalLimitDeg()));
    return std::min(
      fallback_mechanical,
      std::min(steering_max_deg_, std::abs(steering_physical_operational_limit_deg_)));
  }

  double clampPhysicalSteeringDeg(double physical_deg) const
  {
    const double op = operationalPhysicalLimitDeg();
    return std::clamp(physical_deg, -op, op);
  }

  double steeringDegFor(const Selected & selected) const
  {
    if (selected.estop) return 0.0;

    double steering_deg = 0.0;
    if (selected.teleop) {
      // Teleop angular.z is a normalized steering request encoded in Twist units.
      // Full stick reaches the currently allowed PHYSICAL operating limit, never
      // the legacy STM protocol endpoint directly.
      const double max_yaw_rps = yaw_max_deg_s_ * kPi / 180.0;
      const double fraction = max_yaw_rps > 1.0e-9
        ? std::clamp(selected.twist.angular.z / max_yaw_rps, -1.0, 1.0)
        : 0.0;
      steering_deg = fraction * operationalPhysicalLimitDeg();
    } else if (selected.nav2) {
      // ROS/Nav2 uses REP-103: +angular.z = CCW/left turn. The deployed steering
      // hardware convention is +physical degree = RIGHT, therefore Nav2 yaw-rate
      // must be sign-inverted exactly once here. Teleop/HMI conventions are separate.
      const double v = selected.twist.linear.x;
      const double omega = selected.twist.angular.z;
      if (std::abs(v) >= min_speed_for_nav_steering_mps_) {
        const double kappa = omega / v;
        if (std::abs(kappa) > 1.0e-9) {
          const double center_radius = 1.0 / std::abs(kappa);
          const double inner_radius =
            std::max(1.0e-6, center_radius - 0.5 * track_width_m_);
          const double inner_angle = std::atan(wheelbase_m_ / inner_radius);
          steering_deg =
            -std::copysign(inner_angle, kappa) * 180.0 / kPi;
        }
      } else {
        steering_deg = 0.0;  // Ackermann cannot execute pure rotation.
      }
    }

    // Return the CALIBRATED vehicle steering convention. Hardware polarity and
    // persistent raw-endpoint mapping are applied only when building the STM command.
    steering_deg = clampPhysicalSteeringDeg(steering_deg);
    return steering_deg;
  }

  double uncalibratedSteeringTargetDeg(double physical_deg)
  {
    physical_deg = clampPhysicalSteeringDeg(physical_deg);

    if (steering_physical_lut_enabled_ && steering_physical_lut_valid_) {
      const int direction = updateSteeringLutDirection(physical_deg);
      if (std::abs(physical_deg) <= steering_center_hold_request_deadband_deg_) {
        const auto avg_cmd = averageLut(
          steering_lut_command_increasing_deg_, steering_lut_command_decreasing_deg_);
        return interpolateMonotonic(physical_deg, steering_lut_physical_deg_, avg_cmd);
      }
      const auto & lut = direction < 0 ?
        steering_lut_command_decreasing_deg_ : steering_lut_command_increasing_deg_;
      return interpolateMonotonic(physical_deg, steering_lut_physical_deg_, lut);
    }
    if (!steering_feedback_calibration_enabled_ || !steering_feedback_calibration_valid_) {
      return physical_deg;
    }

    if (std::abs(physical_deg) <= 1.0e-9) {
      return steering_feedback_center_deg_;
    }

    // Convert REAL wheel angle to STM protocol command. Protocol
    // endpoints and physical wheel endpoints are deliberately separate domains.
    if (steering_physical_calibration_enabled_ && steering_physical_calibration_valid_) {
      if (physical_deg > 0.0) {
        const double ratio = std::clamp(
          physical_deg / steering_physical_right_limit_deg_, 0.0, 1.0);
        return steering_feedback_center_deg_ +
               ratio * (steering_feedback_effective_right_stop_deg_ - steering_feedback_center_deg_);
      }
      const double ratio = std::clamp(
        physical_deg / steering_physical_left_limit_deg_, 0.0, 1.0);
      return steering_feedback_center_deg_ +
             ratio * (steering_feedback_effective_left_stop_deg_ - steering_feedback_center_deg_);
    }

    // Uncalibrated fallback keeps the Ackermann wheel-angle domain separate from
    // the STM/FOC protocol domain. The full SAFE operational wheel span maps to
    // the configured STM endpoints (normally -90..+90 deg), so a 28 deg wheel
    // safety limit no longer truncates the motor protocol command to about 28 deg.
    const double operational_deg = std::max(1.0e-9, operationalPhysicalLimitDeg());
    if (physical_deg > 0.0) {
      const double ratio = std::clamp(physical_deg / operational_deg, 0.0, 1.0);
      return steering_feedback_center_deg_ +
             ratio * (steering_feedback_effective_right_stop_deg_ - steering_feedback_center_deg_);
    }
    const double ratio = std::clamp((-physical_deg) / operational_deg, 0.0, 1.0);
    return steering_feedback_center_deg_ +
           ratio * (steering_feedback_effective_left_stop_deg_ - steering_feedback_center_deg_);
  }

  double stmFromUncalibratedDeg(double uncalibrated_deg) const
  {
    // Saved GUI points use the legacy DISPLAY convention (feedback after
    // invert_steering). Convert that convention back to STM hardware polarity.
    return uncalibrated_deg * (invert_steering_ ? -1.0 : 1.0);
  }

  double stmSteeringCommandDeg(double calibrated_deg)
  {
    return stmFromUncalibratedDeg(uncalibratedSteeringTargetDeg(calibrated_deg));
  }

  void resetCenterHold()
  {
    steering_center_hold_active_ = false;
    steering_center_hold_last_update_ = std::chrono::steady_clock::now();
    steering_center_hold_trim_state_deg_ = 0.0;
    last_center_hold_trim_deg_ = 0.0;
    last_center_hold_p_deg_ = 0.0;
    last_center_hold_error_deg_ = 0.0;
    last_center_hold_measured_uncal_deg_ = steering_feedback_center_reference_deg_;
  }

  void pauseCenterHoldPreserveTrim()
  {
    // A turn must pause center adaptation, but must NOT erase the bounded trim
    // that was learned while the wheel was physically centered. Erasing it made
    // the return-to-center command jump back to the raw 3-point center and could
    // leave the wheel near its operational limit after every Nav2 turn.
    steering_center_hold_active_ = false;
    steering_center_hold_last_update_ = std::chrono::steady_clock::now();
    last_center_hold_p_deg_ = 0.0;
    last_center_hold_trim_deg_ = steering_center_hold_trim_state_deg_;
  }

  double centerHeldUncalibratedTargetDeg(double calibrated_target_deg, double base_protocol_cmd_deg)
  {
    const auto now_steady = std::chrono::steady_clock::now();
    last_center_hold_p_deg_ = 0.0;

    // Remember the direction from which the mechanism will later return to center.
    if (calibrated_target_deg < -steering_center_hold_request_deadband_deg_) {
      last_steering_direction_ = -1;
      pauseCenterHoldPreserveTrim();
      return base_protocol_cmd_deg;
    }
    if (calibrated_target_deg > steering_center_hold_request_deadband_deg_) {
      last_steering_direction_ = +1;
      pauseCenterHoldPreserveTrim();
      return base_protocol_cmd_deg;
    }

    if (!steering_center_hold_enabled_ || !steering_feedback_calibration_enabled_ ||
        !steering_feedback_calibration_valid_ || steering_calibration_mode_enabled_) {
      resetCenterHold();
      return base_protocol_cmd_deg;
    }

    bool feedback_ok = false;
    double measured_hw_deg = 0.0;
    std::chrono::steady_clock::time_point ack_time;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      feedback_ok = ack_seen_;
      measured_hw_deg = measured_steering_deg_;
      ack_time = last_ack_time_;
    }
    if (!feedback_ok) {
      pauseCenterHoldPreserveTrim();
      return base_protocol_cmd_deg;
    }

    const double age = std::chrono::duration<double>(now_steady - ack_time).count();
    if (age > steering_center_hold_feedback_timeout_sec_) {
      pauseCenterHoldPreserveTrim();
      return base_protocol_cmd_deg;
    }

    const double measured_protocol_deg = measured_hw_deg * (invert_steering_ ? -1.0 : 1.0);
    const double error_deg = steering_feedback_center_reference_deg_ - measured_protocol_deg;
    last_center_hold_error_deg_ = error_deg;
    last_center_hold_measured_uncal_deg_ = measured_protocol_deg;

    // Direction-specific feed-forward is optional and defaults to zero. It can be
    // populated later without changing the 3-point calibration geometry.
    double direction_bias = 0.0;
    if (last_steering_direction_ < 0) direction_bias = steering_center_bias_from_left_deg_;
    if (last_steering_direction_ > 0) direction_bias = steering_center_bias_from_right_deg_;
    const double biased_base = base_protocol_cmd_deg + direction_bias;

    if (std::abs(error_deg) > steering_center_hold_capture_deg_) {
      // Outside the adaptation capture window, retain the previously learned
      // bounded center trim instead of discarding it. This gives the actuator a
      // deterministic path back toward center after a full steering excursion.
      pauseCenterHoldPreserveTrim();
      last_center_hold_error_deg_ = error_deg;
      last_center_hold_measured_uncal_deg_ = measured_protocol_deg;
      return std::clamp(
        biased_base + steering_center_hold_trim_state_deg_, -90.0, 90.0);
    }

    double dt = 0.0;
    if (!steering_center_hold_active_) {
      steering_center_hold_active_ = true;
      steering_center_hold_last_update_ = now_steady;
      // Keep the retained trim from the previous centered episode. Adaptation
      // below may refine it, but a normal steering turn never resets it to zero.
    } else {
      dt = std::clamp(
        std::chrono::duration<double>(now_steady - steering_center_hold_last_update_).count(),
        0.0, 0.10);
      steering_center_hold_last_update_ = now_steady;
    }

    const bool inside_deadband =
      std::abs(error_deg) <= steering_center_hold_feedback_deadband_deg_;

    // Bounded center take-up. This is deliberately NOT an unbounded PI
    // integrator: trim_state is clamped and survives normal steering turns so
    // return-to-center remains repeatable. It is cleared only when center-hold
    // itself is disabled/invalidated, not on each commanded turn.
    if (!inside_deadband && dt > 0.0) {
      const double requested_delta = steering_center_hold_adapt_gain_per_sec_ * error_deg * dt;
      const double max_delta = steering_center_hold_trim_rate_deg_s_ * dt;
      steering_center_hold_trim_state_deg_ += std::clamp(requested_delta, -max_delta, max_delta);
    } else if (inside_deadband && !steering_center_hold_latch_inside_deadband_) {
      steering_center_hold_trim_state_deg_ = 0.0;
    }
    steering_center_hold_trim_state_deg_ = std::clamp(
      steering_center_hold_trim_state_deg_,
      -steering_center_hold_max_trim_deg_,
      steering_center_hold_max_trim_deg_);

    const double p_term = inside_deadband ? 0.0 : std::clamp(
      steering_center_hold_kp_ * error_deg, -3.0, 3.0);
    last_center_hold_p_deg_ = p_term;
    last_center_hold_trim_deg_ = steering_center_hold_trim_state_deg_;

    return std::clamp(
      biased_base + steering_center_hold_trim_state_deg_ + p_term, -90.0, 90.0);
  }

  double nativeDriveErpmPerMps() const
  {
    // COMM_SET_RPM and COMM_GET_VALUES use electrical RPM. Convert vehicle
    // linear speed -> wheel mechanical RPM -> motor mechanical RPM -> ERPM.
    return (60.0 * drive_gear_ratio_ * static_cast<double>(drive_motor_pole_pairs_)) /
      (2.0 * kPi * drive_wheel_radius_m_);
  }

  double rightCommandUnitsPerMps() const
  {
    // If measured ground speed is raw_model_speed * scale, command ERPM for a
    // requested ground speed must be divided by the same scale.
    return transport_mode_ == "stm32" ?
      (nativeDriveErpmPerMps() / drive_odometry_calibration_scale_) :
      (right_max_rpm_ / speed_max_mps_);
  }

  double rawDriveMpsFromErpm(double erpm) const
  {
    if (transport_mode_ != "stm32") return erpm / (right_max_rpm_ / speed_max_mps_);
    return erpm / nativeDriveErpmPerMps();
  }

  double rightCommandLimit() const
  {
    return transport_mode_ == "stm32" ? speed_max_mps_ * nativeDriveErpmPerMps() : right_max_rpm_;
  }

  double rightRpmFor(const Selected & selected) const
  {
    if (selected.estop) return 0.0;
    const double limit = rightCommandLimit();
    double rpm = selected.twist.linear.x * rightCommandUnitsPerMps();
    rpm = std::clamp(rpm, -limit, limit);
    if (invert_drive_) rpm = -rpm;
    return rpm;
  }

  void controlTick()
  {
    const auto t = now();
    const Selected selected = selectCommand(t);
    bool maintenance_active = false;
    std::string maintenance_owner;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      maintenance_active = maintenance_mode_active_;
      maintenance_owner = maintenance_owner_;
    }
    if (maintenance_owner.empty() || maintenance_owner == "RUNTIME") maintenance_owner = "MAINTENANCE";
    const double steering_deg = maintenance_active ? 0.0 : steeringDegFor(selected);

    double protocol_cmd_deg = 0.0;  // display convention; converted to STM polarity below
    double right_rpm = maintenance_active ? 0.0 : rightRpmFor(selected);
    std::string effective_source = maintenance_active ? maintenance_owner : selected.source;

    if (maintenance_active) {
      resetCenterHold();
    } else if (steering_calibration_mode_enabled_ && !selected.estop) {
      // Direct calibration uses the freshest teleop Twist directly, not mux labels.
      double teleop_yaw = 0.0;
      bool teleop_fresh_direct = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const double age = (t - teleop_.received).seconds();
        teleop_fresh_direct = teleop_.valid && teleop_.received.nanoseconds() > 0 &&
          age >= 0.0 && age <= teleop_timeout_sec_;
        if (teleop_fresh_direct) teleop_yaw = teleop_.cmd.angular.z;
      }
      const double yaw_scale = yaw_max_deg_s_ * kPi / 180.0;
      const double direct_fraction = (teleop_fresh_direct && yaw_scale > 1.0e-9)
        ? std::clamp(teleop_yaw / yaw_scale, -1.0, 1.0) : 0.0;
      if (std::abs(direct_fraction) > 0.02) {
        calibration_latched_command_deg_ = std::clamp(
          direct_fraction * steering_calibration_direct_limit_deg_,
          -steering_calibration_direct_limit_deg_, steering_calibration_direct_limit_deg_);
      }
      protocol_cmd_deg = calibration_latched_command_deg_;
      right_rpm = 0.0;
      effective_source = "STEERING_CALIBRATION_DIRECT";
      resetCenterHold();
    } else if (selected.source.rfind("HMI_", 0) == 0) {
      // HMI/Web steering is intentionally a full-scale STM protocol request.
      // The operator-visible command domain is -90..+90 deg, while
      // steering_deg above remains the separate calibrated PHYSICAL wheel target.
      // This prevents the historical +/-30 physical fallback from silently
      // shrinking a full LEFT/RIGHT HMI command before it reaches the STM.
      const double yaw_scale = yaw_max_deg_s_ * kPi / 180.0;
      const double fraction = yaw_scale > 1.0e-9
        ? std::clamp(selected.twist.angular.z / yaw_scale, -1.0, 1.0)
        : 0.0;
      if (fraction > 0.0) {
        protocol_cmd_deg = steering_feedback_center_deg_ +
          fraction * (steering_feedback_effective_right_stop_deg_ - steering_feedback_center_deg_);
      } else if (fraction < 0.0) {
        protocol_cmd_deg = steering_feedback_center_deg_ +
          (-fraction) * (steering_feedback_effective_left_stop_deg_ - steering_feedback_center_deg_);
      } else {
        protocol_cmd_deg = steering_feedback_center_deg_;
      }
      protocol_cmd_deg = std::clamp(protocol_cmd_deg, -steering_max_deg_, steering_max_deg_);
      resetCenterHold();
    } else {
      const double base_protocol_cmd_deg = uncalibratedSteeringTargetDeg(steering_deg);
      protocol_cmd_deg = centerHeldUncalibratedTargetDeg(steering_deg, base_protocol_cmd_deg);
    }

    const double steering_stm_deg = stmFromUncalibratedDeg(protocol_cmd_deg);

    geometry_msgs::msg::Twist actuator = selected.twist;
    if (maintenance_active || selected.estop || steering_calibration_mode_enabled_) actuator = geometry_msgs::msg::Twist{};
    actuator_pub_->publish(actuator);

    std_msgs::msg::String source_msg;
    source_msg.data = effective_source;
    active_source_pub_->publish(source_msg);

    std_msgs::msg::Float64 drive_target;
    drive_target.data = (maintenance_active || selected.estop || steering_calibration_mode_enabled_) ? 0.0 : selected.twist.linear.x;
    drive_target_pub_->publish(drive_target);
    std_msgs::msg::Float64 steering_target;
    steering_target.data = steering_deg * kPi / 180.0;
    steering_target_pub_->publish(steering_target);
    std_msgs::msg::Float64 steering_uncal_target;
    steering_uncal_target.data = protocol_cmd_deg * kPi / 180.0;
    steering_uncal_target_pub_->publish(steering_uncal_target);
    steering_protocol_command_pub_->publish(steering_uncal_target);

    SerialCommand serial_cmd;
    const double transport_steering_deg = transport_mode_ == "stm32" ? steering_deg : steering_stm_deg;
    serial_cmd.left_cdeg = static_cast<std::int16_t>(std::lround(transport_steering_deg * 100.0));
    serial_cmd.right_rpm_x10 = static_cast<std::int16_t>(std::lround(right_rpm * 10.0));
    serial_cmd.flags = selected.estop ? kFlagEstop : 0U;
    {
      std::lock_guard<std::mutex> lock(serial_command_mutex_);
      serial_command_ = serial_cmd;
      serial_command_stamp_ = std::chrono::steady_clock::now();
    }

    const std::string source = effective_source;
    if (source != last_logged_source_) {
      last_logged_source_ = source;
      RCLCPP_INFO(
        get_logger(),
        "MUX -> %s | v=%.3f m/s yaw=%.3f rad/s | steer_target=%.2f deg uncal_target=%.2f deg "
        "center_fb=%+.2f err=%+.2f P=%+.2f trim_state=%+.2f deg STM=%.2f deg right_target=%.1f rpm",
        effective_source.c_str(), actuator.linear.x, actuator.angular.z, steering_deg,
        protocol_cmd_deg, last_center_hold_measured_uncal_deg_, last_center_hold_error_deg_,
        last_center_hold_p_deg_, last_center_hold_trim_deg_, steering_stm_deg, right_rpm);
    }

    diagnostic_source_ = effective_source;
    diagnostic_drive_target_mps_ = (maintenance_active || selected.estop || steering_calibration_mode_enabled_) ? 0.0 : selected.twist.linear.x;
    diagnostic_drive_target_rpm_ = right_rpm;
    diagnostic_steering_target_deg_ = steering_deg;
    diagnostic_steering_raw_target_deg_ = protocol_cmd_deg;

    publishLinkState();
  }

  std::vector<std::string> serialCandidates() const
  {
    if (!serial_device_.empty() && serial_device_ != "auto" && serial_device_ != "AUTO") {
      return {serial_device_};
    }

    std::error_code ec;

    // 1) Stable USB identity is authoritative when unique. The deployed ESC
    // adapter is Prolific PL2303 (067b:2303), distinct from GNSS CH340 and IMU
    // CP2102, so this remains correct across ttyUSB renumbering and port moves.
    if (!serial_auto_id_contains_.empty() && fs::exists("/dev/serial/by-id", ec)) {
      std::vector<std::string> matches;
      for (const auto & entry : fs::directory_iterator("/dev/serial/by-id", ec)) {
        const std::string path = entry.path().string();
        if (path.find(serial_auto_id_contains_) != std::string::npos) matches.push_back(path);
      }
      std::sort(matches.begin(), matches.end());
      if (matches.size() == 1U) return matches;
    }

    // 2) Physical USB topology is only a fallback if by-id is unavailable or
    // ambiguous. This preserves deterministic operation on clone adapters while
    // never overriding a unique USB identity.
    if (!serial_auto_path_contains_.empty() && fs::exists("/dev/serial/by-path", ec)) {
      std::vector<std::string> matches;
      for (const auto & entry : fs::directory_iterator("/dev/serial/by-path", ec)) {
        const std::string path = entry.path().string();
        if (path.find(serial_auto_path_contains_) != std::string::npos) matches.push_back(path);
      }
      std::sort(matches.begin(), matches.end());
      if (matches.size() == 1U) return matches;
    }

    // Actuator routing must fail closed if configured selectors cannot resolve
    // exactly one device. Never send STM command frames to arbitrary ttyUSBs.
    if (!serial_auto_id_contains_.empty() || !serial_auto_path_contains_.empty()) return {};

    // Generic discovery is explicit opt-in only (both selectors cleared).
    std::vector<std::string> result;
    for (const char * prefix : {"/dev/ttyUSB", "/dev/ttyACM"}) {
      for (int i = 0; i < 16; ++i) {
        const std::string path = std::string(prefix) + std::to_string(i);
        if (fs::exists(path, ec)) result.push_back(path);
      }
    }
    return result;
  }


  int openSerial(std::string & active_path)
  {
    const auto candidates = serialCandidates();
    if (candidates.empty()) {
      // Kondisi USB yang memang belum terpasang tidak perlu memenuhi log setiap
      // lima detik. Peringatkan sekali, lalu worker tetap retry normal; status
      // /esc/ready dan ROS Web tetap OFFLINE sampai perangkat benar-benar muncul.
      RCLCPP_WARN_ONCE(
        get_logger(),
        "[ESC] no unique serial candidate; expected by-id '*%s*' or fallback by-path '*%s*'",
        serial_auto_id_contains_.c_str(), serial_auto_path_contains_.c_str());
      return -1;
    }

    int last_error = 0;
    for (const auto & path : candidates) {
      const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (fd < 0) {
        last_error = errno;
        continue;
      }

      // Prevent a stale/second autonomous launch from becoming another owner
      // of the same USB-UART. TIOCEXCL is released automatically on last close.
      // This is especially important for PL2303 adapters after interrupted runs.
      if (::ioctl(fd, TIOCEXCL) != 0) {
        last_error = errno;
        ::close(fd);
        continue;
      }

      termios tty{};
      if (::tcgetattr(fd, &tty) != 0) {
        last_error = errno;
        ::close(fd);
        continue;
      }
      ::cfmakeraw(&tty);
      speed_t baud = B1000000;
      if (serial_baud_ != 1000000) {
        RCLCPP_WARN_ONCE(get_logger(), "F103 protocol is fixed at 1000000 baud; forcing 1000000.");
      }
      ::cfsetispeed(&tty, baud);
      ::cfsetospeed(&tty, baud);
      tty.c_cflag |= CLOCAL | CREAD;
      tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
      if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        last_error = errno;
        ::close(fd);
        continue;
      }
      ::tcflush(fd, TCIOFLUSH);
      active_path = path;
      return fd;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[ESC] serial candidate exists but open/configure failed: errno=%d (%s). Check dialout permission and USB-UART state.",
      last_error, last_error != 0 ? std::strerror(last_error) : "unknown");
    return -1;
  }

  std::array<std::uint8_t, kFrameSize> buildFrame(std::uint16_t seq, const SerialCommand & cmd) const
  {
    std::array<std::uint8_t, kFrameSize> frame{};
    frame[0] = kSof0;
    frame[1] = kSof1;
    frame[2] = kVersion;
    frame[3] = kTypeCommand;
    writeU16Le(&frame[4], seq);
    writeU16Le(&frame[6], static_cast<std::uint16_t>(cmd.left_cdeg));
    writeU16Le(&frame[8], static_cast<std::uint16_t>(cmd.right_rpm_x10));
    frame[10] = cmd.flags;
    frame[11] = 0U;
    writeU16Le(&frame[12], crc16Ccitt(&frame[2], 10));
    return frame;
  }

  bool writeFrame(int fd, const std::array<std::uint8_t, kFrameSize> & frame)
  {
    const std::uint8_t * p = frame.data();
    std::size_t remaining = frame.size();
    int would_block_retries = 0;
    while (remaining > 0U) {
      const ssize_t n = ::write(fd, p, remaining);
      if (n > 0) {
        p += n;
        remaining -= static_cast<std::size_t>(n);
        would_block_retries = 0;
        continue;
      }
      if (n < 0 && errno == EINTR) continue;
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && would_block_retries++ < 20) {
        // O_NONBLOCK avoids a stuck actuator thread. A USB-UART adapter may
        // transiently report EAGAIN under USB scheduling pressure; retry for <=2 ms before
        // reconnecting instead of dropping a valid link immediately.
        std::this_thread::sleep_for(100us);
        continue;
      }
      // Never carry a partial frame to a later control cycle. Reconnect and resync instead.
      return false;
    }
    return true;
  }

  void consumeRxByte(std::uint8_t byte)
  {
    if (rx_pos_ == 0U) {
      if (byte == kSof0) rx_frame_[rx_pos_++] = byte;
      return;
    }
    if (rx_pos_ == 1U) {
      if (byte == kSof1) {
        rx_frame_[rx_pos_++] = byte;
      } else if (byte == kSof0) {
        rx_frame_[0] = kSof0;
      } else {
        rx_pos_ = 0U;
      }
      return;
    }

    rx_frame_[rx_pos_++] = byte;
    if (rx_pos_ >= kFrameSize) {
      handleAck(rx_frame_);
      rx_pos_ = 0U;
    }
  }

  bool drainRx(int fd)
  {
    std::uint8_t buffer[256];
    for (;;) {
      const ssize_t n = ::read(fd, buffer, sizeof(buffer));
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) consumeRxByte(buffer[i]);
        continue;
      }
      if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) return true;
      if (errno == EINTR) continue;
      return false;
    }
  }

  void handleAck(const std::array<std::uint8_t, kFrameSize> & frame)
  {
    if (frame[0] != kSof0 || frame[1] != kSof1 || frame[2] != kVersion || frame[3] != kTypeAck) return;
    if (readU16Le(&frame[12]) != crc16Ccitt(&frame[2], 10)) return;

    const std::uint16_t seq = readU16Le(&frame[4]);
    const double steering_deg = static_cast<double>(readI16Le(&frame[6])) * 0.01;
    const double rpm = static_cast<double>(readI16Le(&frame[8])) * 0.1;
    const std::uint8_t status = frame[10];

    std::lock_guard<std::mutex> lock(feedback_mutex_);
    ack_seen_ = true;
    last_ack_time_ = std::chrono::steady_clock::now();
    last_ack_seq_ = seq;
    measured_steering_deg_ = steering_deg;
    measured_rpm_ = rpm;
    feedback_status_ = status;
    feedback_updated_ = true;
  }

  void publishStm32Bytes(const std::vector<std::uint8_t> &bytes)
  {
    if (!serial_enabled_ || transport_mode_ != "stm32" || !stm32_tx_pub_ || bytes.empty()) return;
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = bytes;
    stm32_tx_pub_->publish(msg);
  }

  void sendVescPayload(const std::vector<std::uint8_t> &payload)
  {
    publishStm32Bytes(makeVescFrame(payload));
  }

  SerialCommand currentSafeCommand(const std::chrono::steady_clock::time_point &now_steady)
  {
    SerialCommand cmd;
    std::chrono::steady_clock::time_point command_stamp;
    {
      std::lock_guard<std::mutex> lock(serial_command_mutex_);
      cmd = serial_command_;
      command_stamp = serial_command_stamp_;
    }
    const double age = command_stamp.time_since_epoch().count() == 0
      ? std::numeric_limits<double>::infinity()
      : std::chrono::duration<double>(now_steady - command_stamp).count();
    if (!std::isfinite(age) || age > command_watchdog_sec_) {
      cmd = SerialCommand{};
      cmd.left_cdeg = static_cast<std::int16_t>(
        std::lround((transport_mode_ == "stm32" ? 0.0 : stmSteeringCommandDeg(0.0)) * 100.0));
    }
    return cmd;
  }

  void sendVescSetPos(double position_deg)
  {
    /* ROS/Web path is signed physical steering. Standard COMM_SET_POS is kept
     * exclusively for VESC Tool's 0..360 widget and is remapped in F103. */
    position_deg = std::clamp(position_deg, -30.0, 30.0);
    std::vector<std::uint8_t> payload{
      kVescCustomAppData, kHbMagic0, kHbMagic1, kHbVersion, kHbSetSteeringDeg};
    appendI32Be(payload, static_cast<std::int32_t>(std::lround(position_deg * 1000.0)));
    sendVescPayload(payload);
  }

  void sendVescSetRpm(double rpm)
  {
    std::vector<std::uint8_t> inner{kVescSetRpm};
    appendI32Be(inner, static_cast<std::int32_t>(std::lround(rpm)));
    sendVescPayload(wrapRightMotor(inner));
  }

  void requestVescValues(bool second)
  {
    std::vector<std::uint8_t> payload{kVescGetValues};
    if (second) payload = wrapRightMotor(payload);
    sendVescPayload(payload);
  }

  void requestSteeringCalibration()
  {
    std::vector<std::uint8_t> payload{
      kVescCustomAppData, kHbMagic0, kHbMagic1, kHbVersion, kHbGetSteeringCal};
    sendVescPayload(payload);
  }

  static double signedPositionDeg(double deg)
  {
    while (deg >= 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
  }

  void handleVescValues(const std::vector<std::uint8_t> &p)
  {
    // Firmware's full COMM_GET_VALUES response follows VESC 6.00 field order.
    if (p.size() < 59U || p[0] != kVescGetValues) return;
    const double temp_mos_c = static_cast<double>(readI16Be(&p[1])) / 10.0;
    const double current_motor_a = static_cast<double>(readI32Be(&p[5])) / 100.0;
    const double current_in_a = static_cast<double>(readI32Be(&p[9])) / 100.0;
    const double id_a = static_cast<double>(readI32Be(&p[13])) / 100.0;
    const double iq_a = static_cast<double>(readI32Be(&p[17])) / 100.0;
    const double duty = static_cast<double>(readI16Be(&p[21])) / 1000.0;
    const double rpm = static_cast<double>(readI32Be(&p[23]));
    const double vbus_v = static_cast<double>(readI16Be(&p[27])) / 10.0;
    const std::uint8_t fault = p[53];
    const double position_deg = static_cast<double>(readI32Be(&p[54])) / 1000000.0;
    const std::uint8_t vesc_id = p[58];
    const auto t = std::chrono::steady_clock::now();

    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (vesc_id == 1U) {
        measured_steering_deg_ = signedPositionDeg(position_deg);
        left_fault_code_ = fault;
        left_values_seen_ = true;
        left_values_time_ = t;
        left_temp_mos_c_ = temp_mos_c;
        left_current_motor_a_ = current_motor_a;
        left_current_in_a_ = current_in_a;
        left_id_a_ = id_a;
        left_iq_a_ = iq_a;
        left_duty_ = duty;
        left_rpm_ = rpm;
        left_vbus_v_ = vbus_v;
      } else if (vesc_id == kVescRightMotorId) {
        measured_rpm_ = rpm;
        right_fault_code_ = fault;
        right_values_seen_ = true;
        right_values_time_ = t;
        right_temp_mos_c_ = temp_mos_c;
        right_current_motor_a_ = current_motor_a;
        right_current_in_a_ = current_in_a;
        right_id_a_ = id_a;
        right_iq_a_ = iq_a;
        right_duty_ = duty;
        right_vbus_v_ = vbus_v;
      } else {
        return;
      }
      ack_seen_ = left_values_seen_ && right_values_seen_;
      if (ack_seen_) last_ack_time_ = std::min(left_values_time_, right_values_time_);
      feedback_updated_ = true;
      feedback_status_ = 0U;
      if (left_fault_code_ == 0U) feedback_status_ |= 0x01U | 0x04U;
      if (right_fault_code_ == 0U) feedback_status_ |= 0x02U | 0x08U;
    }
    publishFocTelemetry();
  }

  void handleVescPayload(const std::vector<std::uint8_t> &payload)
  {
    if (payload.empty()) return;
    if (payload[0] == kVescGetValues) { handleVescValues(payload); return; }
    if (payload.size() >= 23U && payload[0] == kVescCustomAppData &&
        payload[1] == kHbMagic0 && payload[2] == kHbMagic1 &&
        payload[3] == kHbVersion && payload[4] == kHbGetSteeringCal && payload[5] == 0U) {
      const std::uint8_t flags=payload[6];
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      steering_calibrated_=(flags&0x01U)!=0U;
      steering_homed_=(flags&0x02U)!=0U;
      steering_encoder_synced_=(flags&0x04U)!=0U;
      steering_span_counts_=readI32Be(&payload[7]);
      steering_raw_count_=readI32Be(&payload[11]);
      steering_raw_target_=readI32Be(&payload[15]);
      measured_steering_deg_=static_cast<double>(readI32Be(&payload[19]))/1000.0;
    }
  }

  void consumeVescRxBytes(const std::vector<std::uint8_t> &bytes)
  {
    if (bytes.empty()) return;
    vesc_rx_stream_.insert(vesc_rx_stream_.end(), bytes.begin(), bytes.end());
    if (vesc_rx_stream_.size() > 8192U) {
      vesc_rx_stream_.erase(vesc_rx_stream_.begin(), vesc_rx_stream_.end() - 4096);
      ++vesc_rx_overflow_count_;
    }
    for (;;) {
      while (!vesc_rx_stream_.empty() && vesc_rx_stream_.front() != 2U &&
             vesc_rx_stream_.front() != 3U && vesc_rx_stream_.front() != 4U) {
        vesc_rx_stream_.erase(vesc_rx_stream_.begin());
      }
      if (vesc_rx_stream_.size() < 2U) return;
      const std::uint8_t start = vesc_rx_stream_[0];
      std::size_t header = 0U;
      std::size_t payload_len = 0U;
      if (start == 2U) {
        header = 2U;
        payload_len = vesc_rx_stream_[1];
      } else if (start == 3U) {
        if (vesc_rx_stream_.size() < 3U) return;
        header = 3U;
        payload_len = (static_cast<std::size_t>(vesc_rx_stream_[1]) << 8U) | vesc_rx_stream_[2];
      } else {
        if (vesc_rx_stream_.size() < 4U) return;
        header = 4U;
        payload_len = (static_cast<std::size_t>(vesc_rx_stream_[1]) << 16U) |
                      (static_cast<std::size_t>(vesc_rx_stream_[2]) << 8U) | vesc_rx_stream_[3];
      }
      if (payload_len == 0U || payload_len > 4096U) {
        vesc_rx_stream_.erase(vesc_rx_stream_.begin());
        ++vesc_rx_format_error_count_;
        continue;
      }
      const std::size_t total = header + payload_len + 3U;
      if (vesc_rx_stream_.size() < total) return;
      if (vesc_rx_stream_[total - 1U] != 3U) {
        vesc_rx_stream_.erase(vesc_rx_stream_.begin());
        ++vesc_rx_format_error_count_;
        continue;
      }
      const auto *payload_ptr = vesc_rx_stream_.data() + header;
      const std::uint16_t expected_crc =
        static_cast<std::uint16_t>((static_cast<std::uint16_t>(vesc_rx_stream_[header + payload_len]) << 8U) |
                                   vesc_rx_stream_[header + payload_len + 1U]);
      const std::uint16_t actual_crc = vescCrc16(payload_ptr, payload_len);
      if (expected_crc == actual_crc) {
        std::vector<std::uint8_t> payload(payload_ptr, payload_ptr + payload_len);
        handleVescPayload(payload);
      } else {
        ++vesc_rx_crc_error_count_;
      }
      vesc_rx_stream_.erase(vesc_rx_stream_.begin(), vesc_rx_stream_.begin() + static_cast<std::ptrdiff_t>(total));
    }
  }

  void publishFocTelemetry()
  {
    if (!foc_telemetry_pub_) return;
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    std_msgs::msg::String msg;
    std::ostringstream o;
    o << "{\"transport\":\"" << transport_mode_ << "\","
      << "\"left\":{\"id\":1,\"fault\":" << static_cast<unsigned>(left_fault_code_)
      << ",\"temp_mos_c\":" << left_temp_mos_c_
      << ",\"current_motor_a\":" << left_current_motor_a_
      << ",\"current_in_a\":" << left_current_in_a_
      << ",\"id_a\":" << left_id_a_ << ",\"iq_a\":" << left_iq_a_
      << ",\"duty\":" << left_duty_ << ",\"rpm\":" << left_rpm_
      << ",\"vbus_v\":" << left_vbus_v_ << ",\"position_deg\":" << measured_steering_deg_ << "},"
      << "\"right\":{\"id\":2,\"fault\":" << static_cast<unsigned>(right_fault_code_)
      << ",\"temp_mos_c\":" << right_temp_mos_c_
      << ",\"current_motor_a\":" << right_current_motor_a_
      << ",\"current_in_a\":" << right_current_in_a_
      << ",\"id_a\":" << right_id_a_ << ",\"iq_a\":" << right_iq_a_
      << ",\"duty\":" << right_duty_ << ",\"rpm\":" << measured_rpm_
      << ",\"vbus_v\":" << right_vbus_v_ << "},"
      << "\"steering_cal\":{\"calibrated\":" << (steering_calibrated_?"true":"false")
      << ",\"homed\":" << (steering_homed_?"true":"false")
      << ",\"encoder_synced\":" << (steering_encoder_synced_?"true":"false")
      << ",\"raw_left\":0,\"raw_right\":" << steering_span_counts_
      << ",\"raw_center\":" << (steering_span_counts_/2)
      << ",\"raw_now\":" << steering_raw_count_ << ",\"raw_target\":" << steering_raw_target_
      << ",\"physical_min_deg\":-30,\"physical_center_deg\":0,\"physical_max_deg\":30},"
      << "\"rx_crc_errors\":" << vesc_rx_crc_error_count_
      << ",\"rx_format_errors\":" << vesc_rx_format_error_count_
      << ",\"rx_overflows\":" << vesc_rx_overflow_count_ << "}";
    msg.data = o.str();
    foc_telemetry_pub_->publish(msg);
  }

  void stm32TransportTick()
  {
    if (transport_mode_ != "stm32" || !serial_enabled_ || maintenance_mode_active_) return;
    const auto now_steady = std::chrono::steady_clock::now();
    const SerialCommand cmd = currentSafeCommand(now_steady);
    const double steering_deg = static_cast<double>(cmd.left_cdeg) * 0.01;
    const double right_rpm = static_cast<double>(cmd.right_rpm_x10) * 0.1;
    sendVescSetPos(steering_deg);
    sendVescSetRpm(right_rpm);

    if (++vesc_runtime_tick_ >= 3U) {
      vesc_runtime_tick_ = 0U;
      requestVescValues(false);
      requestVescValues(true);
      if (++steering_cal_tick_ >= 5U) { steering_cal_tick_ = 0U; requestSteeringCalibration(); }
    }

    bool timed_out = true;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (left_values_seen_ && right_values_seen_) {
        const bool left_fresh = std::chrono::duration<double>(now_steady - left_values_time_).count() <= serial_ack_timeout_sec_;
        const bool right_fresh = std::chrono::duration<double>(now_steady - right_values_time_).count() <= serial_ack_timeout_sec_;
        timed_out = !(left_fresh && right_fresh);
      }
    }
    ack_timeout_.store(timed_out);
  }

  void sendStm32SafeStop()
  {
    if (transport_mode_ != "stm32" || !stm32_tx_pub_) return;
    SerialCommand safe_cmd{};
    bool command_seen = false;
    {
      std::lock_guard<std::mutex> lock(serial_command_mutex_);
      safe_cmd.left_cdeg = serial_command_.left_cdeg;
      command_seen = serial_command_stamp_.time_since_epoch().count() != 0;
    }
    if (!command_seen) {
      safe_cmd.left_cdeg = static_cast<std::int16_t>(
        std::lround(stmSteeringCommandDeg(0.0) * 100.0));
    }
    const double steering_deg = static_cast<double>(safe_cmd.left_cdeg) * 0.01;
    for (int i = 0; i < 3; ++i) {
      sendVescSetRpm(0.0);
      sendVescSetPos(steering_deg);
    }
  }

  void startSerialThread()
  {
    safe_stop_requested_.store(false);
    if (serial_thread_running_.exchange(true)) return;
    if (serial_thread_.joinable()) serial_thread_.join();
    serial_thread_ = std::thread(&EscAckermann::serialLoop, this);
    RCLCPP_INFO(get_logger(), "[ESC] serial worker ENABLED");
  }

  void stopSerialThread()
  {
    // Runtime disable and process shutdown both fail closed. The worker owns fd,
    // so never close it from this thread while read/write may still be active.
    safe_stop_requested_.store(true);
    serial_thread_running_.store(false);
    if (serial_thread_.joinable()) serial_thread_.join();
    serial_connected_.store(false);
    {
      std::lock_guard<std::mutex> lock(serial_state_mutex_);
      serial_active_path_.clear();
    }
    ack_timeout_.store(false);
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      ack_seen_ = false;
      feedback_updated_ = false;
      feedback_status_ = 0U;
    }
  }

  void setSerialEnabled(bool enabled)
  {
    if (enabled == serial_enabled_) return;
    if (!enabled && transport_mode_ == "stm32") sendStm32SafeStop();
    serial_enabled_ = enabled;
    if (transport_mode_ == "serial") {
      if (serial_enabled_) startSerialThread();
      else stopSerialThread();
    } else if (!serial_enabled_) {
      serial_connected_.store(false);
      ack_timeout_.store(true);
    }
    if (!serial_enabled_) {
      RCLCPP_WARN(get_logger(),
        "[ESC] actuator transport DISABLED by parameter; ROS interfaces stay alive, READY remains false");
    }
  }

  void serialLoop()
  {
    int fd = -1;
    std::string active_path;
    std::uint16_t sequence = 0U;
    auto next_connect = std::chrono::steady_clock::now();
    auto next_tx = next_connect;
    const auto tx_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / serial_tx_rate_hz_));

    while (serial_thread_running_.load()) {
      const auto t = std::chrono::steady_clock::now();
      if (fd < 0) {
        if (t < next_connect) {
          std::this_thread::sleep_for(5ms);
          continue;
        }
        fd = openSerial(active_path);
        if (fd < 0) {
          serial_connected_.store(false);
          next_connect = t + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(serial_reconnect_sec_));
          std::this_thread::sleep_for(10ms);
          continue;
        }
        sequence = 0U;
        rx_pos_ = 0U;
        serial_connected_.store(true);
        {
          std::lock_guard<std::mutex> lock(serial_state_mutex_);
          serial_active_path_ = active_path;
        }
        {
          std::lock_guard<std::mutex> lock(feedback_mutex_);
          ack_seen_ = false;
          feedback_updated_ = false;
          last_ack_time_ = t;
          feedback_status_ = 0U;
        }
        next_tx = t;
        RCLCPP_INFO(get_logger(), "SERIAL CONNECT %s @ 1000000 8N1", active_path.c_str());
      }

      if (!drainRx(fd)) {
        ::close(fd);
        fd = -1;
        serial_connected_.store(false);
        {
          std::lock_guard<std::mutex> lock(serial_state_mutex_);
          serial_active_path_.clear();
        }
        next_connect = std::chrono::steady_clock::now();
        continue;
      }

      const auto now_steady = std::chrono::steady_clock::now();
      if (now_steady >= next_tx) {
        SerialCommand cmd;
        std::chrono::steady_clock::time_point command_stamp;
        {
          std::lock_guard<std::mutex> lock(serial_command_mutex_);
          cmd = serial_command_;
          command_stamp = serial_command_stamp_;
        }
        const double age = std::chrono::duration<double>(now_steady - command_stamp).count();
        if (command_stamp.time_since_epoch().count() == 0 || age > command_watchdog_sec_) {
          cmd = SerialCommand{};
          // Zero vehicle steering must mean the SAVED physical center, not raw STM 0°.
          cmd.left_cdeg = static_cast<std::int16_t>(
            std::lround(stmSteeringCommandDeg(0.0) * 100.0));
        }

        ++sequence;
        if (!writeFrame(fd, buildFrame(sequence, cmd))) {
          ::close(fd);
          fd = -1;
          serial_connected_.store(false);
          {
            std::lock_guard<std::mutex> lock(serial_state_mutex_);
            serial_active_path_.clear();
          }
          next_connect = std::chrono::steady_clock::now();
          continue;
        }

        next_tx += tx_period;
        if (next_tx + tx_period < now_steady) next_tx = now_steady + tx_period;  // latest-command-wins
      }

      bool timed_out = false;
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        if (ack_seen_) {
          timed_out = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_ack_time_).count() > serial_ack_timeout_sec_;
        }
      }
      ack_timeout_.store(timed_out);
      std::this_thread::sleep_for(1ms);
    }

    if (fd >= 0) {
      if (safe_stop_requested_.load()) {
        // Final bounded shutdown transaction: command propulsion=0 while
        // preserving the latest steering command. Do NOT latch the firmware
        // E-STOP on an ordinary ROS restart/shutdown: field validation showed
        // that the actuator firmware can stop ACKing after that latch and then
        // requires a hardware reset. Runtime E-STOP remains fail-closed through
        // selected.estop above; loss of host frames is also covered by the MCU
        // command watchdog.
        SerialCommand safe_cmd{};
        bool command_seen = false;
        {
          std::lock_guard<std::mutex> lock(serial_command_mutex_);
          safe_cmd.left_cdeg = serial_command_.left_cdeg;
          command_seen = serial_command_stamp_.time_since_epoch().count() != 0;
        }
        if (!command_seen) {
          safe_cmd.left_cdeg = static_cast<std::int16_t>(
            std::lround(stmSteeringCommandDeg(0.0) * 100.0));
        }
        safe_cmd.right_rpm_x10 = 0;
        safe_cmd.flags = 0U;

        bool stop_sent = false;
        for (int attempt = 0; attempt < 3; ++attempt) {
          ++sequence;
          stop_sent = writeFrame(fd, buildFrame(sequence, safe_cmd));
          if (!stop_sent) break;
          std::this_thread::sleep_for(2ms);
        }
        RCLCPP_INFO(
          get_logger(), "[ESC] SAFE SHUTDOWN: drive=0 (no persistent E-STOP latch) %s; closing %s",
          stop_sent ? "sent" : "best-effort failed", active_path.c_str());
      }
      // Never tcdrain() here: a disconnected USB-UART can block in the kernel.
      // fd is O_NONBLOCK and all writes above have a bounded retry budget.
      ::close(fd);
    }
    serial_connected_.store(false);
    {
      std::lock_guard<std::mutex> lock(serial_state_mutex_);
      serial_active_path_.clear();
    }
  }

  double calibratedSteeringDeg(double measured_raw_deg) const
  {
    // Keep ACK feedback in its protocol/encoder domain, then map it to the REAL
    // wheel angle when physical calibration exists. Before that calibration is
    // certified, use the SAFE operational wheel span as a bounded estimate; do
    // not scale protocol feedback by the STM +/-90 degree command domain.
    const double protocol_fb_deg = measured_raw_deg * (invert_steering_ ? -1.0 : 1.0);
    if (steering_physical_lut_enabled_ && steering_physical_lut_valid_) {
      const std::vector<double> * fb_lut = nullptr;
      if (steering_lut_motion_direction_ > 0) fb_lut = &steering_lut_feedback_increasing_deg_;
      else if (steering_lut_motion_direction_ < 0) fb_lut = &steering_lut_feedback_decreasing_deg_;
      const auto average_fb = averageLut(
        steering_lut_feedback_increasing_deg_, steering_lut_feedback_decreasing_deg_);
      const auto & selected_fb = fb_lut ? *fb_lut : average_fb;
      double physical_deg = interpolateMonotonic(
        protocol_fb_deg, selected_fb, steering_lut_physical_deg_);
      physical_deg = std::clamp(physical_deg, leftPhysicalLimitDeg(), rightPhysicalLimitDeg());
      if (std::abs(physical_deg) <= steering_straight_deadband_deg_) return 0.0;
      return physical_deg;
    }
    if (!steering_feedback_calibration_enabled_ || !steering_feedback_calibration_valid_) {
      return protocol_fb_deg;
    }

    const double d = protocol_fb_deg - steering_feedback_center_reference_deg_;
    if (std::abs(d) <= 1.0e-9) return 0.0;

    const double right_span =
      steering_feedback_right_reference_deg_ - steering_feedback_center_reference_deg_;
    const double left_span =
      steering_feedback_left_reference_deg_ - steering_feedback_center_reference_deg_;
    double physical_deg = 0.0;

    if (d * right_span > 0.0 && std::abs(right_span) > 1.0e-9) {
      const double ratio = std::clamp(d / right_span, 0.0, 1.0);
      physical_deg = ratio * (
        (steering_physical_calibration_enabled_ && steering_physical_calibration_valid_)
          ? steering_physical_right_limit_deg_ : operationalPhysicalLimitDeg());
    } else if (d * left_span > 0.0 && std::abs(left_span) > 1.0e-9) {
      const double ratio = std::clamp(d / left_span, 0.0, 1.0);
      physical_deg = ratio * (
        (steering_physical_calibration_enabled_ && steering_physical_calibration_valid_)
          ? steering_physical_left_limit_deg_ : -operationalPhysicalLimitDeg());
    }

    physical_deg = std::clamp(physical_deg, leftPhysicalLimitDeg(), rightPhysicalLimitDeg());
    if (std::abs(physical_deg) <= steering_straight_deadband_deg_) return 0.0;
    return physical_deg;
  }

  void publishLinkState()
  {
    bool ack_seen = false;
    bool feedback_updated = false;
    double steering_deg = 0.0;
    double rpm = 0.0;
    std::uint8_t status = 0U;
    std::uint16_t ack_seq = 0U;
    std::chrono::steady_clock::time_point ack_time;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      ack_seen = ack_seen_;
      feedback_updated = feedback_updated_;
      feedback_updated_ = false;
      steering_deg = measured_steering_deg_;
      rpm = measured_rpm_;
      status = feedback_status_;
      ack_seq = last_ack_seq_;
      ack_time = last_ack_time_;
    }

    const bool connected = serial_connected_.load() && !maintenance_mode_active_;
    std::string active_path;
    {
      std::lock_guard<std::mutex> lock(serial_state_mutex_);
      active_path = serial_active_path_;
    }
    const bool ack_fresh = connected && ack_seen && !ack_timeout_.load();
    const bool left_ready = (status & 0x01U) != 0U;
    const bool right_ready = (status & 0x02U) != 0U;
    const bool left_enabled = (status & 0x04U) != 0U;
    const bool right_enabled = (status & 0x08U) != 0U;
    const bool firmware_failsafe = (status & 0x20U) != 0U;
    const bool ready = ack_fresh && left_ready && right_ready && !firmware_failsafe;
    const bool armed = ready && left_enabled && right_enabled;

    std_msgs::msg::Bool boolean;
    boolean.data = ready;
    ready_pub_->publish(boolean);
    boolean.data = armed;
    armed_pub_->publish(boolean);
    boolean.data = ack_fresh;
    feedback_valid_pub_->publish(boolean);
    boolean.data = ack_fresh && right_ready;
    drive_connected_pub_->publish(boolean);
    boolean.data = ack_fresh && left_ready;
    steer_connected_pub_->publish(boolean);

    const double steering_uncalibrated_deg =
      steering_deg * (invert_steering_ ? -1.0 : 1.0);
    const double steering_calibrated_deg = calibratedSteeringDeg(steering_deg);

    const auto diag_now = std::chrono::steady_clock::now();
    const double diag_period = 1.0 / diagnostic_trace_hz_;
    if (std::chrono::duration<double>(diag_now - diagnostic_last_log_).count() >= diag_period) {
      diagnostic_last_log_ = diag_now;
      RCLCPP_INFO(
        get_logger(),
        "[ESC-TRACE] src=%s drive_cmd=%.3f m/s target=%.1f rpm actual=%.1f rpm err=%+.1f rpm | "
        "steer_target=%+.2f deg protocol_cmd=%+.2f deg protocol_fb=%+.2f deg cal_fb=%+.2f deg",
        diagnostic_source_.c_str(), diagnostic_drive_target_mps_, diagnostic_drive_target_rpm_,
        rpm, diagnostic_drive_target_rpm_ - rpm, diagnostic_steering_target_deg_,
        diagnostic_steering_raw_target_deg_, steering_uncalibrated_deg, steering_calibrated_deg);
    }

    std_msgs::msg::String status_msg;
    std::ostringstream oss;
    oss << "controller=ackermann transport=" << transport_mode_
        << " mode=" << (maintenance_mode_active_ ? "maintenance" : "runtime")
        << " link=" << (!serial_enabled_ ? "disabled" : (connected ? "connected" : (maintenance_mode_active_ ? "maintenance" : "offline")))
        << " path=" << (active_path.empty() ? "-" : active_path)
        << " ack=" << (ack_fresh ? "fresh" : "stale")
        << " seq=" << ack_seq
        << " protocol_fb=" << steering_uncalibrated_deg << "deg"
        << " steer=" << steering_calibrated_deg << "deg"
        << " cal=" << ((steering_feedback_calibration_enabled_ &&
                         steering_feedback_calibration_valid_) ? "on" : "off")
        << " physical_cal=" << ((steering_physical_calibration_enabled_ &&
                                  steering_physical_calibration_valid_) ? "on" : "off")
        << " physical_LR=" << steering_physical_left_limit_deg_ << "/"
        << steering_physical_right_limit_deg_ << "deg"
        << " wheel_op=+/-" << operationalPhysicalLimitDeg() << "deg"
        << " stm_range=+/-" << steering_max_deg_ << "deg"
        << " lut=" << ((steering_physical_lut_enabled_ && steering_physical_lut_valid_) ? "on" : "off")
        << " lut_n=" << steering_lut_physical_deg_.size()
        << " lut_dir=" << steering_lut_motion_direction_
        << " right=" << rpm << (transport_mode_ == "stm32" ? "erpm" : "rpm")
        << " fw_status=0x" << std::hex << static_cast<unsigned>(status);
    status_msg.data = oss.str();
    status_pub_->publish(status_msg);

    if (!ack_fresh || !feedback_updated) return;

    // The command mapping intentionally defines speed_max_mps <-> right_max_rpm.
    const double drive_raw_mps = rawDriveMpsFromErpm(rpm) * (invert_drive_ ? -1.0 : 1.0);
    const double drive_mps = drive_raw_mps * drive_odometry_calibration_scale_;
    const double steering_rad = steering_calibrated_deg * kPi / 180.0;
    // steering_rad adalah sudut roda DALAM Ackermann (RIGHT-positive).
    // R_center = track/2 + L/tan(|delta_inner|).
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

    std_msgs::msg::Float64 scalar;
    scalar.data = drive_mps;
    speed_pub_->publish(scalar);
    drive_actual_pub_->publish(scalar);
    scalar.data = drive_raw_mps;
    drive_raw_pub_->publish(scalar);
    scalar.data = steering_uncalibrated_deg * kPi / 180.0;
    steering_uncalibrated_pub_->publish(scalar);
    steering_feedback_raw_pub_->publish(scalar);
    scalar.data = steering_rad;
    steering_actual_pub_->publish(scalar);
    scalar.data = yaw_rate;
    yaw_rate_actual_pub_->publish(scalar);
    yaw_rate_kinematic_pub_->publish(scalar);

    const auto stamp = now();
    double dt = 0.0;
    if (odom_time_valid_) dt = std::clamp((stamp - last_odom_time_).seconds(), 0.0, 0.20);
    last_odom_time_ = stamp;
    odom_time_valid_ = true;

    if (dt > 0.0) {
      odom_yaw_ += yaw_rate * dt;
      odom_x_ += drive_mps * std::cos(odom_yaw_) * dt;
      odom_y_ += drive_mps * std::sin(odom_yaw_) * dt;
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = odom_x_;
    odom.pose.pose.position.y = odom_y_;
    odom.pose.pose.orientation.z = std::sin(odom_yaw_ * 0.5);
    odom.pose.pose.orientation.w = std::cos(odom_yaw_ * 0.5);
    odom.twist.twist.linear.x = drive_mps;
    odom.twist.twist.angular.z = yaw_rate;
    const double rpm_error_norm = std::clamp(
      std::abs(diagnostic_drive_target_rpm_ - rpm) / std::max(1.0, rightCommandLimit()), 0.0, 2.0);
    const double steer_norm = std::clamp(
      std::abs(steering_rad) /
      std::max(1.0e-6, operationalPhysicalLimitDeg() * kPi / 180.0), 0.0, 1.0);
    const double v_var = odom_v_variance_base_ +
      odom_v_variance_rpm_error_gain_ * rpm_error_norm * rpm_error_norm;
    const double yaw_var = odom_yaw_variance_base_ +
      odom_yaw_variance_steer_gain_ * steer_norm * steer_norm;
    odom.pose.covariance[0] = std::max(0.05, v_var);
    odom.pose.covariance[7] = std::max(0.05, v_var + 0.5 * yaw_var);
    odom.pose.covariance[35] = yaw_var;
    odom.twist.covariance[0] = v_var;
    odom.twist.covariance[35] = odom_yaw_rate_variance_base_ +
      odom_yaw_variance_steer_gain_ * steer_norm * steer_norm;
    odom_pub_->publish(odom);

    (void)ack_time;
  }

  // ROS parameters
  std::string teleop_topic_;
  std::string teleop_source_topic_;
  std::string teleop_estop_topic_;
  std::string hmi_topic_{"/hmi/cmd_vel"};
  std::string hmi_source_topic_{"/hmi/active_source"};
  std::string nav2_topic_;
  std::string output_topic_;
  std::string active_source_topic_{"/esc/mux/active_source"};
  std::string autonomy_gate_topic_;
  std::string global_estop_topic_;
  std::string maintenance_owner_topic_{"/esc/vesc/maintenance_owner"};
  std::string perception_state_topic_{"/navigation/trajectory_safety_state"};
  double command_rate_hz_{50.0};
  double teleop_timeout_sec_{0.30};
  double hmi_timeout_sec_{0.30};
  double nav2_timeout_sec_{0.60};
  double perception_state_timeout_sec_{0.75};
  double manual_release_hold_sec_{0.50};
  bool require_autonomy_gate_{true};
  double speed_max_mps_{0.5};
  double yaw_max_deg_s_{80.0};
  double steering_max_deg_{90.0};
  double right_max_rpm_{300.0};  // direct-serial fallback scale only
  double drive_wheel_radius_m_{0.145};
  int drive_motor_pole_pairs_{15};
  double drive_gear_ratio_{1.0};
  double drive_odometry_calibration_scale_{1.0};
  double wheelbase_m_{0.70};
  double track_width_m_{0.48};
  double min_speed_for_nav_steering_mps_{0.05};
  bool invert_steering_{false};
  bool invert_drive_{false};
  bool steering_feedback_calibration_enabled_{false};
  bool steering_feedback_calibration_valid_{false};
  double steering_feedback_center_deg_{0.0};
  double steering_feedback_right_stop_deg_{90.0};
  double steering_feedback_left_stop_deg_{-90.0};
  bool steering_feedback_force_symmetric_span_{false};
  double steering_feedback_effective_right_stop_deg_{90.0};
  double steering_feedback_effective_left_stop_deg_{-90.0};
  double steering_straight_deadband_deg_{1.0};
  bool steering_center_hold_enabled_{true};
  double steering_center_hold_request_deadband_deg_{0.75};
  double steering_center_hold_capture_deg_{20.0};
  double steering_center_hold_feedback_deadband_deg_{0.35};
  double steering_center_hold_kp_{0.25};
  double steering_center_hold_adapt_gain_per_sec_{1.50};
  double steering_center_hold_max_trim_deg_{12.0};
  double steering_center_hold_trim_rate_deg_s_{60.0};
  bool steering_center_hold_latch_inside_deadband_{true};
  double steering_center_hold_feedback_timeout_sec_{0.30};
  bool steering_center_hold_active_{false};
  std::chrono::steady_clock::time_point steering_center_hold_last_update_{std::chrono::steady_clock::now()};
  double steering_center_hold_trim_state_deg_{0.0};
  double last_center_hold_trim_deg_{0.0};
  double last_center_hold_p_deg_{0.0};
  double last_center_hold_error_deg_{0.0};
  double last_center_hold_measured_uncal_deg_{0.0};
  std::string steering_feedback_calibration_saved_at_;
  std::string steering_calibration_apply_token_;
  bool steering_calibration_mode_enabled_{false};
  double steering_calibration_direct_limit_deg_{90.0};
  double calibration_latched_command_deg_{0.0};
  double steering_feedback_left_reference_deg_{-80.0};
  double steering_feedback_center_reference_deg_{0.0};
  double steering_feedback_right_reference_deg_{80.0};
  bool steering_physical_calibration_enabled_{false};
  bool steering_physical_calibration_valid_{false};
  double steering_physical_left_limit_deg_{-30.0};
  double steering_physical_right_limit_deg_{30.0};
  double steering_physical_operational_limit_deg_{28.0};
  std::string steering_physical_calibration_saved_at_;
  bool steering_physical_lut_enabled_{false};
  bool steering_physical_lut_valid_{false};
  std::vector<double> steering_lut_physical_deg_{-30.0, 0.0, 30.0};
  std::vector<double> steering_lut_command_increasing_deg_{-90.0, 0.0, 90.0};
  std::vector<double> steering_lut_command_decreasing_deg_{-90.0, 0.0, 90.0};
  std::vector<double> steering_lut_feedback_increasing_deg_{-90.0, 0.0, 90.0};
  std::vector<double> steering_lut_feedback_decreasing_deg_{-90.0, 0.0, 90.0};
  double steering_lut_direction_deadband_deg_{0.15};
  std::string steering_lut_calibration_saved_at_;
  int steering_lut_motion_direction_{0};
  double steering_lut_last_target_deg_{0.0};
  double steering_center_bias_from_left_deg_{0.0};
  double steering_center_bias_from_right_deg_{0.0};
  int last_steering_direction_{0};
  std::string transport_mode_{"stm32"};
  std::string stm32_tx_topic_{"/stmf4/vesc/runtime_tx"};
  std::string stm32_rx_topic_{"/stmf4/vesc/rx"};
  std::string stm32_connected_topic_{"/stmf4/vesc/connected"};
  std::uint16_t stm32_sequence_{0U};
  std::atomic<bool> maintenance_mode_active_{false};
  std::uint32_t vesc_runtime_tick_{0U};
  std::uint32_t steering_cal_tick_{0U};
  bool steering_calibrated_{false};
  bool steering_homed_{false};
  bool steering_encoder_synced_{false};
  std::int32_t steering_span_counts_{0};
  std::int32_t steering_raw_count_{0};
  std::int32_t steering_raw_target_{0};
  std::vector<std::uint8_t> vesc_rx_stream_;
  std::uint64_t vesc_rx_crc_error_count_{0U};
  std::uint64_t vesc_rx_format_error_count_{0U};
  std::uint64_t vesc_rx_overflow_count_{0U};
  std::mutex serial_state_mutex_;
  std::string serial_active_path_;
  bool serial_enabled_{true};
  std::string serial_device_{"auto"};
  std::string serial_auto_id_contains_{"Prolific_Technology_Inc._USB-Serial_Controller"};
  std::string serial_auto_path_contains_{};
  int serial_baud_{1000000};
  double serial_tx_rate_hz_{50.0};
  double serial_reconnect_sec_{0.25};
  double serial_ack_timeout_sec_{0.60};
  double command_watchdog_sec_{0.25};
  std::string odom_topic_{"/esc/odom"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"};
  double odom_v_variance_base_{0.03};
  double odom_v_variance_rpm_error_gain_{0.20};
  double odom_yaw_variance_base_{0.08};
  double odom_yaw_variance_steer_gain_{0.30};
  double odom_yaw_rate_variance_base_{0.05};
  double diagnostic_trace_hz_{2.0};
  std::chrono::steady_clock::time_point diagnostic_last_log_{std::chrono::steady_clock::now()};
  std::string diagnostic_source_{"IDLE"};
  double diagnostic_drive_target_mps_{0.0};
  double diagnostic_drive_target_rpm_{0.0};
  double diagnostic_steering_target_deg_{0.0};
  double diagnostic_steering_raw_target_deg_{0.0};

  // Arbitration state
  std::mutex state_mutex_;
  Sample teleop_{};
  Sample hmi_{};
  Sample nav2_{};
  std::string teleop_source_{"STOP"};
  rclcpp::Time teleop_source_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time teleop_takeover_until_{0, 0, RCL_ROS_TIME};
  std::string hmi_source_{"STOP"};
  rclcpp::Time hmi_source_received_{0, 0, RCL_ROS_TIME};
  std::string maintenance_owner_{"RUNTIME"};
  std::string perception_decision_{""};
  rclcpp::Time perception_state_received_{0, 0, RCL_ROS_TIME};
  bool perception_intervening_{false};
  bool teleop_estop_{false};
  bool global_estop_{false};
  bool autonomy_gate_{false};
  bool autonomy_gate_seen_{false};
  std::string last_logged_source_;

  // Serial command + feedback
  std::atomic<bool> serial_thread_running_{false};
  std::atomic<bool> safe_stop_requested_{false};
  std::thread serial_thread_;
  std::atomic<bool> serial_connected_{false};
  std::atomic<bool> ack_timeout_{false};
  std::mutex serial_command_mutex_;
  SerialCommand serial_command_{};
  std::chrono::steady_clock::time_point serial_command_stamp_{};
  std::array<std::uint8_t, kFrameSize> rx_frame_{};
  std::size_t rx_pos_{0U};

  std::mutex feedback_mutex_;
  bool ack_seen_{false};
  bool feedback_updated_{false};
  std::chrono::steady_clock::time_point last_ack_time_{};
  std::uint16_t last_ack_seq_{0U};
  double measured_steering_deg_{0.0};
  double measured_rpm_{0.0};
  std::uint8_t feedback_status_{0U};
  bool left_values_seen_{false}, right_values_seen_{false};
  std::chrono::steady_clock::time_point left_values_time_{}, right_values_time_{};
  std::uint8_t left_fault_code_{0U}, right_fault_code_{0U};
  double left_temp_mos_c_{0.0}, right_temp_mos_c_{0.0};
  double left_current_motor_a_{0.0}, right_current_motor_a_{0.0};
  double left_current_in_a_{0.0}, right_current_in_a_{0.0};
  double left_id_a_{0.0}, right_id_a_{0.0};
  double left_iq_a_{0.0}, right_iq_a_{0.0};
  double left_duty_{0.0}, right_duty_{0.0};
  double left_rpm_{0.0};
  double left_vbus_v_{0.0}, right_vbus_v_{0.0};

  // Odom integration
  bool odom_time_valid_{false};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};

  // ROS interfaces
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_source_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teleop_estop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr hmi_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hmi_source_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_gate_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_estop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr maintenance_owner_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_state_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr stm32_rx_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stm32_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr maintenance_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr stm32_tx_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr actuator_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_source_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr foc_telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr feedback_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drive_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr steer_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_controller_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drive_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drive_actual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drive_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_uncal_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_uncalibrated_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_protocol_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_feedback_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_actual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_actual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_kinematic_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_, stm32_transport_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EscAckermann>();
  std::weak_ptr<EscAckermann> weak_node = node;
  rclcpp::on_shutdown([weak_node]() {
    if (auto locked = weak_node.lock()) locked->requestSafeShutdown();
  });
  rclcpp::spin(node);
  node->requestSafeShutdown();
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
