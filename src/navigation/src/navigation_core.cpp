#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace
{
rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

bool finiteTwist(const geometry_msgs::msg::Twist & msg)
{
  return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
         std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
         std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string kvValue(const std::string & text, const std::string & key, const std::string & fallback = "-")
{
  const std::string token = key + "=";
  size_t pos = text.find(token);
  if (pos == std::string::npos) return fallback;
  pos += token.size();
  const size_t end = text.find(';', pos);
  return text.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
}

double kvDouble(const std::string & text, const std::string & key)
{
  const auto value = kvValue(text, key, "");
  if (value.empty() || value == "-" || value == "N/A") {
    return std::numeric_limits<double>::quiet_NaN();
  }
  try { return std::stod(value); } catch (...) { return std::numeric_limits<double>::quiet_NaN(); }
}

std::string fmtNumber(double value, int precision = 1)
{
  if (!std::isfinite(value)) return "--";
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(precision) << value;
  return ss.str();
}

double radToDeg(double value)
{
  return value * 180.0 / 3.14159265358979323846;
}

std_msgs::msg::String makeHudOverlay(
  int, int, int, int, uint8_t, uint8_t, float, float, float, float)
{
  // Keep HUD diagnostics on standard ROS messages so the native/web GUI and
  // rosbag tooling work on a stock ROS 2 Humble install without third-party
  // rviz_2d_overlay plugins. Embedded RViz uses its built-in displays only.
  return std_msgs::msg::String{};
}
}  // namespace

class NavigationCore final : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using ManageLifecycleNodes = nav2_msgs::srv::ManageLifecycleNodes;
  using ChangeState = lifecycle_msgs::srv::ChangeState;
  using GetState = lifecycle_msgs::srv::GetState;

  NavigationCore()
  : Node("navigation_core")
  {
    declareParameters();
    readParameters();
    createInterfaces();
    RCLCPP_INFO(
      get_logger(),
      "NavigationCore aktif: %s -> %s%s -> %s -> cmd_vel_router -> velocity_smoother -> /cmd_vel -> ESC",
      autonomy_topic_.c_str(), pre_collision_topic_.c_str(),
      collision_monitor_enabled_ ? " -> collision_monitor" : "", final_topic_.c_str());
  }

private:
  struct CommandSample
  {
    geometry_msgs::msg::Twist cmd{};
    rclcpp::Time received{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  void declareParameters()
  {
    declare_parameter<std::string>("autonomy_topic", "/cmd_vel_nav_raw");
    declare_parameter<std::string>("pre_collision_topic", "/cmd_vel/nav2_pre_collision");
    declare_parameter<std::string>("final_topic", "/cmd_vel/autonomy_pre_smoother");
    declare_parameter<std::string>("goal_topic", "/navigation/goal_request");
    declare_parameter<std::string>("legacy_goal_topic", "/move_base_simple/goal");
    declare_parameter<std::string>("action_name", "navigate_to_pose");
    declare_parameter<std::string>("lifecycle_manager_service", "/lifecycle_manager_navigation/manage_nodes");
    declare_parameter<std::string>("map_topic", "/map");
    declare_parameter<bool>("defer_nav2_startup_until_localization", true);
    declare_parameter<double>("map_nav2_startup_settle_sec", 2.5);
    declare_parameter<bool>("collision_monitor_enabled", false);
    declare_parameter<double>("command_rate_hz", 20.0);
    declare_parameter<double>("status_rate_hz", 10.0);
    declare_parameter<double>("autonomy_timeout_sec", 0.60);
    declare_parameter<double>("max_forward_speed_mps", 0.50);
    declare_parameter<double>("max_reverse_speed_mps", 0.30);
    declare_parameter<double>("max_yaw_rate_rps", 0.292028888392);
    declare_parameter<double>("linear_deadband_mps", 0.08);
    declare_parameter<double>("angular_deadband_rps", 0.02);
    declare_parameter<double>("min_speed_for_yaw_mps", 0.08);
    declare_parameter<bool>("require_perception_for_autonomy_motion", false);
    declare_parameter<double>("perception_timeout_sec", 1.5);
    declare_parameter<bool>("require_sensor_publisher_contract", true);
    declare_parameter<std::string>("sensor_publisher_contract_topic", "/system/sensor_publishers_ok");
    declare_parameter<double>("sensor_publisher_contract_timeout_sec", 0.75);
    declare_parameter<bool>("require_camera_metric_calibration", false);
    declare_parameter<bool>("camera_metric_calibration_validated", false);
    // Stage-1 commissioning interlocks. Autonomous motion must stay fail-closed
    // until the physical steering map, circle geometry, and linear odometry scale
    // have been measured on the actual vehicle. Manual/steering commissioning is
    // intentionally unaffected because the ESC mux applies this gate only to Nav2.
    declare_parameter<bool>("require_steering_calibration_for_autonomy", true);
    declare_parameter<bool>("steering_calibration_validated", false);
    declare_parameter<bool>("require_steering_circle_calibration_for_autonomy", true);
    declare_parameter<bool>("steering_circle_calibration_validated", false);
    declare_parameter<bool>("require_drive_odometry_calibration_for_autonomy", true);
    declare_parameter<bool>("drive_odometry_calibration_validated", false);
    // Stage-2 localization interlock: local yaw-rate is IMU-driven, therefore
    // physical autonomy cannot open before stationary bias/covariance calibration.
    declare_parameter<bool>("require_imu_calibration_for_autonomy", true);
    declare_parameter<bool>("imu_calibration_validated", false);
    // Stage-3 production gate. Commissioning mode is an explicit launch-time
    // opt-in and applies a low speed cap; production remains fail-closed until
    // MPPI/safety/fault-injection evidence has been signed off.
    declare_parameter<bool>("require_stage3_production_certification", true);
    declare_parameter<bool>("precision_mode", false);
    declare_parameter<bool>("require_collision_monitor_for_production", true);
    declare_parameter<bool>("require_perception_for_production", true);
    declare_parameter<bool>("stage3_production_certified", false);
    declare_parameter<bool>("stage3_commissioning_mode", false);
    declare_parameter<double>("stage3_commissioning_speed_cap_mps", 1.0);
    declare_parameter<double>("overlay_text_size_px", 9.0);
    declare_parameter<int>("hud_margin_px", 12);
    declare_parameter<bool>("keyboard_available", false);
    declare_parameter<double>("wheelbase_m", 0.70);
    declare_parameter<double>("track_width_m", 0.48);
    declare_parameter<double>("max_steering_angle_rad", 0.488692190558);
    declare_parameter<double>("minimum_turning_radius_m", 1.712159378317);
    declare_parameter<double>("log_heartbeat_sec", 10.0);
  }

  void readParameters()
  {
    autonomy_topic_ = get_parameter("autonomy_topic").as_string();
    pre_collision_topic_ = get_parameter("pre_collision_topic").as_string();
    final_topic_ = get_parameter("final_topic").as_string();
    goal_topic_ = get_parameter("goal_topic").as_string();
    legacy_goal_topic_ = get_parameter("legacy_goal_topic").as_string();
    action_name_ = get_parameter("action_name").as_string();
    lifecycle_manager_service_ = get_parameter("lifecycle_manager_service").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    defer_nav2_startup_until_localization_ =
      get_parameter("defer_nav2_startup_until_localization").as_bool();
    map_nav2_startup_settle_sec_ = std::clamp(
      get_parameter("map_nav2_startup_settle_sec").as_double(), 0.0, 10.0);
    collision_monitor_enabled_ = get_parameter("collision_monitor_enabled").as_bool();
    command_rate_hz_ = std::max(5.0, get_parameter("command_rate_hz").as_double());
    status_rate_hz_ = std::max(0.5, get_parameter("status_rate_hz").as_double());
    autonomy_timeout_sec_ = std::clamp(get_parameter("autonomy_timeout_sec").as_double(), 0.10, 3.0);
    max_forward_speed_mps_ = std::max(0.0, get_parameter("max_forward_speed_mps").as_double());
    max_reverse_speed_mps_ = std::max(0.0, get_parameter("max_reverse_speed_mps").as_double());
    max_yaw_rate_rps_ = std::max(0.01, get_parameter("max_yaw_rate_rps").as_double());
    linear_deadband_mps_ = std::max(0.0, get_parameter("linear_deadband_mps").as_double());
    angular_deadband_rps_ = std::max(0.0, get_parameter("angular_deadband_rps").as_double());
    min_speed_for_yaw_mps_ = std::max(0.0, get_parameter("min_speed_for_yaw_mps").as_double());
    require_perception_for_motion_ = get_parameter("require_perception_for_autonomy_motion").as_bool();
    perception_timeout_sec_ = std::clamp(get_parameter("perception_timeout_sec").as_double(), 0.2, 10.0);
    require_sensor_publisher_contract_ = get_parameter("require_sensor_publisher_contract").as_bool();
    sensor_publisher_contract_topic_ = get_parameter("sensor_publisher_contract_topic").as_string();
    sensor_publisher_contract_timeout_sec_ = std::clamp(
      get_parameter("sensor_publisher_contract_timeout_sec").as_double(), 0.1, 5.0);
    require_camera_calibration_ = get_parameter("require_camera_metric_calibration").as_bool();
    camera_calibration_validated_ = get_parameter("camera_metric_calibration_validated").as_bool();
    require_steering_calibration_ =
      get_parameter("require_steering_calibration_for_autonomy").as_bool();
    steering_calibration_validated_ =
      get_parameter("steering_calibration_validated").as_bool();
    require_steering_circle_calibration_ =
      get_parameter("require_steering_circle_calibration_for_autonomy").as_bool();
    steering_circle_calibration_validated_ =
      get_parameter("steering_circle_calibration_validated").as_bool();
    require_drive_odometry_calibration_ =
      get_parameter("require_drive_odometry_calibration_for_autonomy").as_bool();
    drive_odometry_calibration_validated_ =
      get_parameter("drive_odometry_calibration_validated").as_bool();
    require_imu_calibration_ = get_parameter("require_imu_calibration_for_autonomy").as_bool();
    imu_calibration_validated_ = get_parameter("imu_calibration_validated").as_bool();
    require_stage3_production_certification_ =
      get_parameter("require_stage3_production_certification").as_bool();
    precision_mode_ = get_parameter("precision_mode").as_bool();
    require_collision_monitor_for_production_ =
      get_parameter("require_collision_monitor_for_production").as_bool();
    require_perception_for_production_ =
      get_parameter("require_perception_for_production").as_bool();
    stage3_production_certified_ = get_parameter("stage3_production_certified").as_bool();
    stage3_commissioning_mode_ = get_parameter("stage3_commissioning_mode").as_bool();
    stage3_commissioning_speed_cap_mps_ = std::max(
      0.05, get_parameter("stage3_commissioning_speed_cap_mps").as_double());
    overlay_text_size_px_ = std::max(8.0, get_parameter("overlay_text_size_px").as_double());
    hud_margin_px_ = static_cast<int>(std::max<int64_t>(0, get_parameter("hud_margin_px").as_int()));
    keyboard_available_ = get_parameter("keyboard_available").as_bool();
    wheelbase_m_ = std::max(0.1, get_parameter("wheelbase_m").as_double());
    track_width_m_ = std::max(0.0, get_parameter("track_width_m").as_double());
    max_steering_angle_rad_ = std::clamp(
      get_parameter("max_steering_angle_rad").as_double(), 0.01, 1.553343034);
    minimum_turning_radius_m_ = std::max(0.10, get_parameter("minimum_turning_radius_m").as_double());
    log_heartbeat_sec_ = std::max(1.0, get_parameter("log_heartbeat_sec").as_double());
  }

  void createInterfaces()
  {
    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    autonomy_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      autonomy_topic_, cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto t = now();
        if (!finiteTwist(*msg)) {
          autonomy_cmd_ = {geometry_msgs::msg::Twist{}, t, false};
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000, "Autonomy Twist NaN/Inf -> sample dibuang");
          return;
        }
        autonomy_cmd_.cmd = clampCommand(*msg);
        autonomy_cmd_.received = t;
        autonomy_cmd_.valid = true;
      });

    mppi_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav_raw", cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { storeDiagnostic(mppi_raw_, *msg); });
    smoothed_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav_smoothed", cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { storeDiagnostic(smoothed_, *msg); });
    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel/teleop", cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) { storeDiagnostic(teleop_cmd_, *msg); });

    planning_loc_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/planning_localization_ready", stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        planning_localization_ready_ = msg->data;
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, stateQos(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (msg->info.width == 0U || msg->info.height == 0U ||
            !(msg->info.resolution > 0.0F) || msg->data.empty()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Map sample invalid/empty pada %s; Nav2 tetap ditahan", map_topic_.c_str());
          return;
        }
        bool first = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          first = !map_ready_;
          map_ready_ = true;
          if (first) map_ready_since_ = now();
          map_width_ = msg->info.width;
          map_height_ = msg->info.height;
          map_resolution_ = msg->info.resolution;
        }
        if (first) {
          RCLCPP_INFO(
            get_logger(), "MAP READY: %s %ux%u @ %.3f m/cell; Nav2 boleh STARTUP setelah localization siap",
            map_topic_.c_str(), msg->info.width, msg->info.height, msg->info.resolution);
        }
      });
    motion_loc_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/motion_localization_ready", stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        motion_localization_ready_ = msg->data;
      });
    sensor_contract_sub_ = create_subscription<std_msgs::msg::Bool>(
      sensor_publisher_contract_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        sensor_publisher_contract_ok_ = msg->data;
        last_sensor_publisher_contract_time_ = now();
      });
    precision_localization_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/system/precision_localization_ready", stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        precision_localization_ready_ = msg->data;
      });
    localization_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/localization_state", stateQos(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        localization_state_ = msg->data;
      });
    gnss_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/gnss_status", stateQos(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        gnss_status_ = msg->data;
      });
    imu_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/imu_status", stateQos(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_status_ = msg->data;
      });
    ekf_local_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/ekf_local_status", stateQos(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        ekf_local_status_ = msg->data;
      });
    ekf_global_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/system/ekf_global_status", stateQos(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        ekf_global_status_ = msg->data;
      });
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    gnss_map_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/gnss_map", sensor_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        gnss_map_odom_ = *msg;
        gnss_map_received_ = now();
        have_gnss_map_ = true;
      });
    esc_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/esc/odom", sensor_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        esc_odom_ = *msg;
        esc_odom_received_ = now();
        have_esc_odom_ = true;
      });
    esc_steering_actual_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/steering_actual_rad", sensor_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        esc_steering_actual_rad_ = msg->data;
        esc_steering_received_ = now();
      });
    esc_drive_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/drive_target_mps", sensor_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        esc_drive_target_mps_ = msg->data;
        esc_drive_target_received_ = now();
      });
    esc_drive_actual_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/drive_actual_mps", sensor_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        esc_drive_actual_mps_ = msg->data;
        esc_drive_actual_received_ = now();
      });
    esc_steering_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/steering_target_rad", sensor_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        esc_steering_target_rad_ = msg->data;
        esc_steering_target_received_ = now();
      });
    esc_feedback_valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/feedback_valid", stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        esc_feedback_valid_ = msg->data;
      });
    gnss_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/gnss/connected", stateQos(), [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_); gnss_connected_ = msg->data;
      });
    imu_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/imu/connected", stateQos(), [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_); imu_connected_ = msg->data;
      });
    camera_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/perception/camera_connected", stateQos(), [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_); camera_connected_ = msg->data;
      });
    esc_drive_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/drive/connected", stateQos(), [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_); esc_drive_connected_ = msg->data;
      });
    esc_steer_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/steer/connected", stateQos(), [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_); esc_steer_connected_ = msg->data;
      });
    esc_armed_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/armed", stateQos(), [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_); esc_armed_ = msg->data;
      });
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", sensor_qos, [this](sensor_msgs::msg::Joy::SharedPtr) {
        std::lock_guard<std::mutex> lock(mutex_); last_joy_time_ = now();
      });

    esc_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/esc/status", stateQos(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        esc_status_ = msg->data;
      });
    esc_mux_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/esc/mux/active_source", stateQos(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        esc_mux_status_ = msg->data;
      });
    esc_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/ready", stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        esc_ready_ = msg->data;
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/estop", stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        estop_ = msg->data;
      });
    perception_state_sub_ = create_subscription<std_msgs::msg::String>(
      "/perception/lane_safety_state", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        perception_state_ = msg->data;
        last_perception_time_ = now();
      });
    raw_detection_sub_ = create_subscription<std_msgs::msg::String>(
      "/perception/raw_detections", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        raw_detection_status_ = msg->data;
        last_raw_detection_time_ = now();
      });

    pre_collision_pub_ = create_publisher<geometry_msgs::msg::Twist>(pre_collision_topic_, cmd_qos);
    if (collision_monitor_enabled_) {
      final_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        final_topic_, cmd_qos,
        [this](geometry_msgs::msg::Twist::SharedPtr msg) { storeDiagnostic(final_cmd_, *msg); });
    } else {
      final_pub_ = create_publisher<geometry_msgs::msg::Twist>(final_topic_, cmd_qos);
    }

    autonomy_motion_allowed_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/system/autonomy_motion_allowed", stateQos());
    planning_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/nav2_ready", stateQos());
    motion_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/motion_ready", stateQos());
    autonomy_ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/autonomy_ready", stateQos());
    sensor_status_pub_ = create_publisher<std_msgs::msg::String>("/system/sensor_status", stateQos());
    goal_state_pub_ = create_publisher<std_msgs::msg::String>("/navigation/goal_state", stateQos());
    // Tiga panel kecil dengan posisi eksplisit; tidak lagi satu kotak 980x330
    // yang menutupi map dan saling bertumpuk dengan panel RViz.
    hud_nav_pub_ = create_publisher<std_msgs::msg::String>("/system/hud/navigation", stateQos());
    hud_sensor_pub_ = create_publisher<std_msgs::msg::String>("/system/hud/sensors", stateQos());
    hud_drive_pub_ = create_publisher<std_msgs::msg::String>("/system/hud/drive", stateQos());

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, cmd_qos, std::bind(&NavigationCore::onGoal, this, std::placeholders::_1));
    if (!legacy_goal_topic_.empty() && legacy_goal_topic_ != goal_topic_) {
      legacy_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        legacy_goal_topic_, cmd_qos, std::bind(&NavigationCore::onGoal, this, std::placeholders::_1));
    }

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    nav2_lifecycle_client_ = create_client<ManageLifecycleNodes>(lifecycle_manager_service_);
    smoother_get_state_client_ = create_client<GetState>("/velocity_smoother/get_state");
    smoother_change_state_client_ = create_client<ChangeState>("/velocity_smoother/change_state");

    smoother_guard_timer_ = create_wall_timer(500ms, std::bind(&NavigationCore::onSmootherGuardTimer, this));
    nav2_startup_timer_ = create_wall_timer(500ms, std::bind(&NavigationCore::maybeStartNav2Lifecycle, this));
    goal_timer_ = create_wall_timer(500ms, std::bind(&NavigationCore::trySendQueuedGoal, this));
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / command_rate_hz_)),
      std::bind(&NavigationCore::onCommandTimer, this));
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / status_rate_hz_)),
      std::bind(&NavigationCore::onStatusTimer, this));
  }

  geometry_msgs::msg::Twist clampCommand(const geometry_msgs::msg::Twist & in) const
  {
    geometry_msgs::msg::Twist out{};
    out.linear.x = std::clamp(in.linear.x, -max_reverse_speed_mps_, max_forward_speed_mps_);
    if (stage3_commissioning_mode_ && !stage3_production_certified_) {
      const double cap = std::min(stage3_commissioning_speed_cap_mps_, max_forward_speed_mps_);
      out.linear.x = std::clamp(out.linear.x, -std::min(cap, max_reverse_speed_mps_), cap);
    }
    const double curvature_yaw_cap =
      std::abs(out.linear.x) / std::max(0.10, minimum_turning_radius_m_);
    const double yaw_cap = std::min(max_yaw_rate_rps_, curvature_yaw_cap);
    out.angular.z = std::clamp(in.angular.z, -yaw_cap, yaw_cap);
    if (std::abs(out.linear.x) < linear_deadband_mps_) out.linear.x = 0.0;
    if (std::abs(out.angular.z) < angular_deadband_rps_) out.angular.z = 0.0;
    if (std::abs(out.linear.x) < min_speed_for_yaw_mps_) out.angular.z = 0.0;
    return out;
  }

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
    // ROS +yaw=LEFT, STM steering + = RIGHT.
    const double steer = -std::copysign(inner_angle, kappa);
    return std::clamp(
      steer, -max_steering_angle_rad_, max_steering_angle_rad_);
  }

  void storeDiagnostic(CommandSample & slot, const geometry_msgs::msg::Twist & msg)
  {
    if (!finiteTwist(msg)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    slot.cmd = msg;
    slot.received = now();
    slot.valid = true;
  }

  bool fresh(const CommandSample & sample, double timeout_sec, const rclcpp::Time & t) const
  {
    if (!sample.valid || sample.received.nanoseconds() == 0) return false;
    const double age = (t - sample.received).seconds();
    return age >= 0.0 && age <= timeout_sec;
  }

  bool autonomousMotionReadyUnlocked(const rclcpp::Time & t) const
  {
    // The autonomous gate is the last software interlock before the ESC mux.
    // Do not open it merely because localization is ready: the STM link must
    // also have a fresh ACK with both actuator-ready status bits.
    // Commissioning mode is intentionally a bounded pre-certification mode: it
    // keeps hard E-stop, ESC-ready ACK, map, smoother, and planning-localization
    // gates active, but permits indoor motion before GNSS strict/certification is
    // complete. clampCommand() enforces the commissioning speed cap.
    const bool commissioning = stage3_commissioning_mode_ && !stage3_production_certified_;
    if (estop_ || !esc_ready_ || !map_ready_ || !velocity_smoother_active_ ||
        !planning_localization_ready_) return false;
    if (require_sensor_publisher_contract_) {
      if (!sensor_publisher_contract_ok_ || last_sensor_publisher_contract_time_.nanoseconds() == 0) return false;
      const double publisher_contract_age = (t - last_sensor_publisher_contract_time_).seconds();
      if (publisher_contract_age < 0.0 || publisher_contract_age > sensor_publisher_contract_timeout_sec_) return false;
    }
    if (precision_mode_ && !precision_localization_ready_) return false;
    if (!commissioning && !motion_localization_ready_) return false;
    if (!commissioning) {
      if (require_camera_calibration_ && !camera_calibration_validated_) return false;
      if (require_steering_calibration_ && !steering_calibration_validated_) return false;
      if (require_steering_circle_calibration_ && !steering_circle_calibration_validated_) return false;
      if (require_drive_odometry_calibration_ && !drive_odometry_calibration_validated_) return false;
      if (require_imu_calibration_ && !imu_calibration_validated_) return false;
      if (require_stage3_production_certification_ && !stage3_production_certified_) return false;
      // Defense-in-depth: production flag saja tidak pernah cukup. Bahkan bila
      // YAML stage3 salah diedit manual, actuator tetap tertutup tanpa collision
      // monitor dan perception stream yang fresh.
      if (stage3_production_certified_ && require_collision_monitor_for_production_ &&
          !collision_monitor_enabled_) return false;
      if (stage3_production_certified_ && require_perception_for_production_) {
        if (!camera_connected_ || last_perception_time_.nanoseconds() == 0) return false;
        const double production_perception_age = (t - last_perception_time_).seconds();
        if (production_perception_age < 0.0 || production_perception_age > perception_timeout_sec_) return false;
      }
    }
    if (require_perception_for_motion_) {
      // Fresh state dari publisher lama/stale tidak boleh membuka actuator bila
      // instance perception aktif belum benar-benar memiliki kamera USB.
      if (!camera_connected_) return false;
      if (last_perception_time_.nanoseconds() == 0) return false;
      const double age = (t - last_perception_time_).seconds();
      if (age < 0.0 || age > perception_timeout_sec_) return false;
    }
    return true;
  }

  void requestSmootherTransition(uint8_t transition_id, const char * transition_name)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (smoother_transition_request_in_flight_) return;
      smoother_transition_request_in_flight_ = true;
    }
    if (!smoother_change_state_client_ || !smoother_change_state_client_->service_is_ready()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        smoother_transition_request_in_flight_ = false;
        velocity_smoother_active_ = false;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "velocity_smoother change_state service belum ready; autonomy tetap fail-closed");
      return;
    }

    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition_id;
    smoother_change_state_client_->async_send_request(
      request,
      [this, transition_id, name = std::string(transition_name)](
        rclcpp::Client<ChangeState>::SharedFuture future) {
        bool success = false;
        try {
          const auto response = future.get();
          success = response && response->success;
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "velocity_smoother %s exception: %s", name.c_str(), e.what());
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          smoother_transition_request_in_flight_ = false;
          if (transition_id == lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE) {
            velocity_smoother_active_ = success;
          }
        }
        if (success) {
          RCLCPP_INFO(get_logger(), "velocity_smoother lifecycle %s berhasil", name.c_str());
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "velocity_smoother lifecycle %s gagal; guard akan retry", name.c_str());
        }
      });
  }

  void onSmootherGuardTimer()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (smoother_state_request_in_flight_) return;
      smoother_state_request_in_flight_ = true;
    }
    if (!smoother_get_state_client_ || !smoother_get_state_client_->service_is_ready()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        smoother_state_request_in_flight_ = false;
        velocity_smoother_active_ = false;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "velocity_smoother get_state service belum ready; autonomy tetap fail-closed");
      return;
    }

    auto request = std::make_shared<GetState::Request>();
    smoother_get_state_client_->async_send_request(
      request,
      [this](rclcpp::Client<GetState>::SharedFuture future) {
        uint8_t state_id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        bool valid = false;
        try {
          const auto response = future.get();
          if (response) {
            state_id = response->current_state.id;
            valid = true;
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "velocity_smoother get_state exception: %s", e.what());
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          smoother_state_request_in_flight_ = false;
          velocity_smoother_active_ =
            valid && state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
        }
        if (!valid || state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
        if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
          requestSmootherTransition(
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "CONFIGURE");
        } else if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
          requestSmootherTransition(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "ACTIVATE");
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "velocity_smoother state=%u bukan ACTIVE/INACTIVE/UNCONFIGURED; autonomy fail-closed",
            static_cast<unsigned>(state_id));
        }
      });
  }

  bool mapStartupSettledUnlocked(const rclcpp::Time & t) const
  {
    if (!map_ready_ || map_ready_since_.nanoseconds() == 0) return false;
    const double age = (t - map_ready_since_).seconds();
    return age >= map_nav2_startup_settle_sec_;
  }

  void maybeStartNav2Lifecycle()
  {
    // Map harus benar-benar sudah diterima. Jika deferred mode aktif, anchor
    // localization juga wajib ready. Cek prerequisite SEBELUM mempercayai
    // action server di DDS agar proses stale dari launch lama tidak menipu state.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool prerequisites = mapStartupSettledUnlocked(now()) &&
        (!defer_nav2_startup_until_localization_ || planning_localization_ready_);
      if (!prerequisites) return;
    }

    if (nav_client_ && nav_client_->wait_for_action_server(0s)) {
      std::lock_guard<std::mutex> lock(mutex_);
      nav2_lifecycle_started_ = true;
      nav2_startup_requested_ = false;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool prerequisites = mapStartupSettledUnlocked(now()) &&
        (!defer_nav2_startup_until_localization_ || planning_localization_ready_);
      if (!prerequisites || nav2_lifecycle_started_ || nav2_startup_requested_) return;
    }
    if (!nav2_lifecycle_client_ || !nav2_lifecycle_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Map/localization prerequisite siap tetapi lifecycle service %s belum ready",
        lifecycle_manager_service_.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool prerequisites = mapStartupSettledUnlocked(now()) &&
        (!defer_nav2_startup_until_localization_ || planning_localization_ready_);
      if (!prerequisites || nav2_lifecycle_started_ || nav2_startup_requested_) return;
      nav2_startup_requested_ = true;
    }

    auto request = std::make_shared<ManageLifecycleNodes::Request>();
    request->command = 0U;  // STARTUP
    if (defer_nav2_startup_until_localization_) {
      RCLCPP_INFO(
        get_logger(),
        "map + map->odom ready dan map settle %.1fs -> STARTUP Nav2 lifecycle",
        map_nav2_startup_settle_sec_);
    } else {
      RCLCPP_INFO(
        get_logger(), "map ready + settle %.1fs -> STARTUP Nav2 lifecycle (localization defer disabled)",
        map_nav2_startup_settle_sec_);
    }
    nav2_lifecycle_client_->async_send_request(
      request,
      [this](rclcpp::Client<ManageLifecycleNodes>::SharedFuture future) {
        bool success = false;
        try {
          const auto response = future.get();
          success = response && response->success;
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "Nav2 STARTUP service exception: %s", e.what());
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          nav2_lifecycle_started_ = success;
          nav2_startup_requested_ = false;
        }
        if (success) RCLCPP_INFO(get_logger(), "Nav2 lifecycle STARTUP berhasil");
        else RCLCPP_WARN(get_logger(), "Nav2 lifecycle STARTUP gagal; retry otomatis 0.5 s");
      });
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped goal = *msg;
    if (goal.header.frame_id.empty()) goal.header.frame_id = "map";
    const auto & p = goal.pose.position;
    const auto & q = goal.pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
      RCLCPP_ERROR(get_logger(), "GOAL ditolak: PoseStamped mengandung NaN/Inf");
      return;
    }
    if (goal.header.frame_id != "map") {
      RCLCPP_WARN(get_logger(), "GOAL frame '%s' bukan map; Nav2 akan melakukan transform", goal.header.frame_id.c_str());
    }

    uint64_t count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queued_goal_ = goal;
      last_goal_pose_ = goal;
      goal_state_ = "QUEUED";
      count = ++goal_rx_count_;
      publishGoalStateUnlocked();
    }
    RCLCPP_INFO(
      get_logger(), "GOAL RX #%lu frame=%s x=%.3f y=%.3f yaw=%.3f",
      static_cast<unsigned long>(count), goal.header.frame_id.c_str(),
      p.x, p.y, yawFromQuaternion(q));
    trySendQueuedGoal();
  }

  void trySendQueuedGoal()
  {
    geometry_msgs::msg::PoseStamped pose;
    uint64_t request_id = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      nav2_action_ready_ = nav_client_->action_server_is_ready();
      if (!queued_goal_.has_value() || goal_send_in_flight_) return;
      if (!map_ready_) {
        goal_state_ = "QUEUED_MAP";
        publishGoalStateUnlocked();
        return;
      }
      if (!planning_localization_ready_) {
        goal_state_ = "QUEUED_LOCALIZATION";
        publishGoalStateUnlocked();
        return;
      }
      if (!nav2_action_ready_) {
        goal_state_ = "QUEUED_NAV2";
        publishGoalStateUnlocked();
        return;
      }
      pose = *queued_goal_;
      queued_goal_.reset();
      goal_send_in_flight_ = true;
      request_id = ++goal_send_sequence_;
      current_goal_request_id_ = request_id;
      goal_state_ = "SENDING";
      publishGoalStateUnlocked();
    }

    NavigateToPose::Goal goal;
    goal.pose = pose;
    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.goal_response_callback = [this, request_id](GoalHandleNavigate::SharedPtr handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request_id != current_goal_request_id_) return;
      goal_send_in_flight_ = false;
      active_goal_handle_ = handle;
      goal_state_ = handle ? "ACTIVE" : "REJECTED";
      publishGoalStateUnlocked();
    };
    options.result_callback = [this, request_id](const GoalHandleNavigate::WrappedResult & result) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (request_id != current_goal_request_id_) return;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: goal_state_ = "SUCCEEDED"; break;
        case rclcpp_action::ResultCode::ABORTED: goal_state_ = "ABORTED"; break;
        case rclcpp_action::ResultCode::CANCELED: goal_state_ = "CANCELED"; break;
        default: goal_state_ = "UNKNOWN"; break;
      }
      active_goal_handle_.reset();
      goal_send_in_flight_ = false;
      publishGoalStateUnlocked();
    };
    nav_client_->async_send_goal(goal, options);
  }

  void publishGoalStateUnlocked()
  {
    std_msgs::msg::String msg;
    msg.data = goal_state_;
    goal_state_pub_->publish(msg);
  }

  void onCommandTimer()
  {
    geometry_msgs::msg::Twist command{};
    bool motion_allowed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto t = now();
      if (map_ready_ && planning_localization_ready_ &&
          fresh(autonomy_cmd_, autonomy_timeout_sec_, t)) {
        command = autonomy_cmd_.cmd;
      }

      // Goal success/stop sengaja TIDAK dihitung dari raw /odometry/gnss_map.
      // Nav2 SimpleGoalChecker menggunakan TF map->base_footprint yang sama dengan
      // URDF di RViz. Dengan tolerance commissioning yang ketat, kendaraan baru dianggap selesai
      // ketika pose yang terlihat oleh planner/URDF memang sudah masuk radius itu.

      motion_allowed = autonomousMotionReadyUnlocked(t);
      routed_cmd_.cmd = command;
      routed_cmd_.received = t;
      routed_cmd_.valid = true;
    }

    // Publish the physical-autonomy gate at command rate, not only HUD rate.
    // A localization/perception interlock therefore closes in <= 1 command period.
    std_msgs::msg::Bool gate_msg;
    gate_msg.data = motion_allowed;
    autonomy_motion_allowed_pub_->publish(gate_msg);

    // Diagnostic Nav2 command continues even while motion gate is closed. ESC mux
    // applies the gate only to the physical Nav2 source; manual commissioning is separate.
    pre_collision_pub_->publish(command);
    if (!collision_monitor_enabled_ && final_pub_) final_pub_->publish(command);
  }

  void onStatusTimer()
  {
    bool map_ready = false;
    bool planning_loc = false;
    bool motion_loc = false;
    bool nav2_action = false;
    bool motion_allowed = false;
    bool smoother_active = false;
    bool perception_fresh = false;
    bool estop = false;
    bool esc_ready = false;
    bool gnss_connected = false;
    bool imu_connected = false;
    bool camera_connected = false;
    bool esc_drive_connected = false;
    bool esc_steer_connected = false;
    bool esc_armed = false;
    bool esc_feedback_valid = false;
    bool precision_localization_ready = false;
    std::string localization;
    std::string gnss;
    std::string imu;
    std::string ekf_local;
    std::string ekf_global;
    std::string esc_status;
    std::string esc_mux_status;
    std::string raw_detections;
    std::string goal_state;
    CommandSample teleop;
    CommandSample yolop;
    CommandSample raw;
    CommandSample smoothed;
    CommandSample final;
    nav_msgs::msg::Odometry gnss_map;
    nav_msgs::msg::Odometry esc_odom;
    bool have_gnss_map = false;
    bool have_esc_odom = false;
    double esc_steering = std::numeric_limits<double>::quiet_NaN();
    double esc_drive_target = std::numeric_limits<double>::quiet_NaN();
    double esc_drive_actual = std::numeric_limits<double>::quiet_NaN();
    double esc_steering_target = std::numeric_limits<double>::quiet_NaN();
    rclcpp::Time joy_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time gnss_map_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time esc_odom_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time esc_steer_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time esc_drive_target_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time esc_drive_actual_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time esc_steering_target_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time raw_detection_time{0, 0, RCL_ROS_TIME};
    const auto snapshot_time = now();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      nav2_action_ready_ = nav_client_->action_server_is_ready();
      nav2_action = nav2_action_ready_;
      map_ready = map_ready_;
      planning_loc = planning_localization_ready_;
      motion_loc = motion_localization_ready_;
      motion_allowed = autonomousMotionReadyUnlocked(snapshot_time);
      smoother_active = velocity_smoother_active_;
      perception_fresh = last_perception_time_.nanoseconds() > 0 &&
        (snapshot_time - last_perception_time_).seconds() >= 0.0 &&
        (snapshot_time - last_perception_time_).seconds() <= perception_timeout_sec_;
      estop = estop_;
      esc_ready = esc_ready_;
      gnss_connected = gnss_connected_;
      imu_connected = imu_connected_;
      camera_connected = camera_connected_;
      esc_drive_connected = esc_drive_connected_;
      esc_steer_connected = esc_steer_connected_;
      esc_armed = esc_armed_;
      esc_feedback_valid = esc_feedback_valid_;
      precision_localization_ready = precision_localization_ready_;
      localization = localization_state_;
      gnss = gnss_status_;
      imu = imu_status_;
      ekf_local = ekf_local_status_;
      ekf_global = ekf_global_status_;
      esc_status = esc_status_;
      esc_mux_status = esc_mux_status_;
      raw_detections = raw_detection_status_;
      goal_state = goal_state_;
      teleop = teleop_cmd_;
      yolop = autonomy_cmd_;
      raw = mppi_raw_;
      smoothed = smoothed_;
      final = final_cmd_;
      gnss_map = gnss_map_odom_;
      esc_odom = esc_odom_;
      have_gnss_map = have_gnss_map_;
      have_esc_odom = have_esc_odom_;
      esc_steering = esc_steering_actual_rad_;
      esc_drive_target = esc_drive_target_mps_;
      esc_drive_actual = esc_drive_actual_mps_;
      esc_steering_target = esc_steering_target_rad_;
      joy_time = last_joy_time_;
      gnss_map_time = gnss_map_received_;
      esc_odom_time = esc_odom_received_;
      esc_steer_time = esc_steering_received_;
      esc_drive_target_time = esc_drive_target_received_;
      esc_drive_actual_time = esc_drive_actual_received_;
      esc_steering_target_time = esc_steering_target_received_;
      raw_detection_time = last_raw_detection_time_;
    }

    const auto isTimeFresh = [&snapshot_time](const rclcpp::Time & stamp, double timeout) {
      if (stamp.nanoseconds() == 0) return false;
      const double age = (snapshot_time - stamp).seconds();
      return age >= 0.0 && age <= timeout;
    };
    const auto zeroIfStale = [this, &snapshot_time](CommandSample & sample) {
      if (!fresh(sample, 1.0, snapshot_time)) sample.cmd = geometry_msgs::msg::Twist{};
    };
    zeroIfStale(teleop);
    zeroIfStale(yolop);
    zeroIfStale(raw);
    zeroIfStale(smoothed);
    zeroIfStale(final);

    const bool joy_online = isTimeFresh(joy_time, 2.0);
    const bool gnss_xy_fresh = have_gnss_map && isTimeFresh(gnss_map_time, 3.0);
    const bool esc_odom_fresh = have_esc_odom && isTimeFresh(esc_odom_time, 1.0);
    const bool esc_steer_fresh = isTimeFresh(esc_steer_time, 0.5);
    const bool esc_drive_target_fresh = isTimeFresh(esc_drive_target_time, 0.5);
    const bool esc_drive_actual_fresh = isTimeFresh(esc_drive_actual_time, 0.5);
    const bool esc_steering_target_fresh = isTimeFresh(esc_steering_target_time, 0.5);
    const bool raw_detection_fresh = isTimeFresh(raw_detection_time, 1.0);
    const bool nav2_ready = map_ready && planning_loc && nav2_action;

    std_msgs::msg::Bool b;
    b.data = nav2_ready;
    planning_ready_pub_->publish(b);
    b.data = motion_allowed;
    motion_ready_pub_->publish(b);
    autonomy_ready_pub_->publish(b);
    autonomy_motion_allowed_pub_->publish(b);

    std::ostringstream ss;
    ss << std::boolalpha
       << "map=" << map_ready
       << ";planning_localization=" << planning_loc
       << ";motion_localization=" << motion_loc
       << ";nav2_action=" << nav2_action
       << ";velocity_smoother_active=" << smoother_active
       << ";sensor_publishers=" << sensor_publisher_contract_ok_
       << ";autonomy_motion_allowed=" << motion_allowed
       << ";perception=" << perception_fresh
       << ";camera_usb=" << camera_connected
       << ";gnss_usb=" << gnss_connected
       << ";imu_usb=" << imu_connected
       << ";esc_usb=" << esc_drive_connected
       << ";camera_cal=" << camera_calibration_validated_
       << ";steering_cal=" << steering_calibration_validated_
       << ";circle_cal=" << steering_circle_calibration_validated_
       << ";drive_odom_cal=" << drive_odometry_calibration_validated_
       << ";imu_cal=" << imu_calibration_validated_
       << ";precision_mode=" << precision_mode_
       << ";precision_localization=" << precision_localization_ready
       << ";stage3_cert=" << stage3_production_certified_
       << ";stage3_commissioning=" << stage3_commissioning_mode_
       << ";estop=" << estop
       << ";esc_ready=" << esc_ready
       << ";goal=" << goal_state;
    std_msgs::msg::String status;
    status.data = ss.str();
    sensor_status_pub_->publish(status);

    // LEFT=0, RIGHT=1, TOP=3, BOTTOM=4.
    const float hud_text = static_cast<float>(overlay_text_size_px_);
    const std::string gnss_quality = kvValue(gnss, "quality", "INVALID");
    const std::string loc_mode = kvValue(localization, "mode", "STARTUP");

    // KIRI ATAS: hanya status koneksi/readiness, tidak bercampur angka telemetry.
    auto status_hud = makeHudOverlay(420, 110, hud_margin_px_, hud_margin_px_, 0, 3, hud_text,
      nav2_ready ? 0.70F : 1.0F, nav2_ready ? 1.0F : 0.84F, 0.50F);
    std::ostringstream status_text;
    status_text << "SYSTEM STATUS\n"
                << "ESC LINK " << (esc_feedback_valid ? "ACK LIVE" : "NO ACK")
                << " | DRIVE " << (esc_drive_connected ? "READY" : "OFFLINE")
                << " | STEER " << (esc_steer_connected ? "READY" : "OFFLINE") << "\n"
                << "ESC READY " << (esc_ready ? "YES" : "NO")
                << " | ARM " << (esc_armed ? "ARMED" : "DISARMED")
                << " | SRC " << (esc_mux_status.empty() ? "IDLE" : esc_mux_status) << "\n"
                << "GNSS USB " << (gnss_connected ? "CONNECTED" : "OFFLINE")
                << " | FIX " << gnss_quality << "\n"
                << "IMU USB " << (imu_connected ? "CONNECTED" : "OFFLINE")
                << " | CAMERA " << (camera_connected ? "CONNECTED" : "WAIT/BUSY")
                << " | YOLOP " << (perception_fresh ? "ONLINE" : "WAIT") << "\n"
                << "MAP " << (map_ready ? "READY" : "WAIT")
                << " | NAV2 " << (nav2_action ? "ACTIVE" : "WAIT")
                << " | LOC " << loc_mode << "\n"
                << "PLANNING " << (planning_loc ? "READY" : "WAIT")
                << " | MOTION " << (motion_allowed ? "OPEN" : "BLOCKED")
                << " | GOAL " << goal_state << "\n"
                << "CAL STEER " << (steering_calibration_validated_ ? "PASS" : "WAIT")
                << " | CIRCLE " << (steering_circle_calibration_validated_ ? "PASS" : "WAIT")
                << " | ODOM " << (drive_odometry_calibration_validated_ ? "PASS" : "WAIT")
                << " | IMU " << (imu_calibration_validated_ ? "PASS" : "WAIT") << "\n"
                << "MUX " << (esc_mux_status.empty() ? "IDLE" : esc_mux_status)
                << " | KEYBOARD " << (keyboard_available_ ? "READY" : "OFF")
                << " | JOYSTICK " << (joy_online ? "CONNECTED" : "OFFLINE")
                << " | ESTOP " << (estop ? "ACTIVE" : "CLEAR");
    status_hud.data = status_text.str();
    hud_nav_pub_->publish(status_hud);

    // KIRI BAWAH: angka sensor/estimasi. Panel dibuat panjang ke kanan agar
// semua telemetry terbaca satu blok tanpa menutupi status/CMD. Semua angka 1 digit desimal kecuali
    // latitude/longitude yang wajib 7 digit agar koordinat geografis tetap berguna.
    double gnss_x = std::numeric_limits<double>::quiet_NaN();
    double gnss_y = std::numeric_limits<double>::quiet_NaN();
    if (gnss_xy_fresh) {
      gnss_x = gnss_map.pose.pose.position.x;
      gnss_y = gnss_map.pose.pose.position.y;
    }
    const double lon = kvDouble(gnss, "lon");
    const double lat = kvDouble(gnss, "lat");
    const double dop = kvDouble(gnss, "dop");
    const double hacc = kvDouble(gnss, "hacc");
    const double sacc = kvDouble(gnss, "sacc");
    const double sat = kvDouble(gnss, "sat");
    const double gnss_yaw = kvDouble(gnss, "gnss_yaw");
    const double gnss_vx = kvDouble(gnss, "gnss_base_vx");
    const double gnss_vyaw = kvDouble(gnss, "gnss_vyaw");
    const bool gnss_vyaw_valid = kvValue(gnss, "gnss_vyaw_valid", "false") == "true";

    const double nan = std::numeric_limits<double>::quiet_NaN();
    const bool ekf_global_fresh = kvValue(ekf_global, "fresh", "false") == "true";
    const bool ekf_local_fresh = kvValue(ekf_local, "fresh", "false") == "true";
    const double eg_x = ekf_global_fresh ? kvDouble(ekf_global, "x") : nan;
    const double eg_y = ekf_global_fresh ? kvDouble(ekf_global, "y") : nan;
    const double eg_yaw = ekf_global_fresh ? kvDouble(ekf_global, "yaw") : nan;
    const double eg_v = ekf_global_fresh ? kvDouble(ekf_global, "v") : nan;
    const double eg_w = ekf_global_fresh ? kvDouble(ekf_global, "w") : nan;
    const double el_x = ekf_local_fresh ? kvDouble(ekf_local, "x") : nan;
    const double el_y = ekf_local_fresh ? kvDouble(ekf_local, "y") : nan;
    const double el_yaw = ekf_local_fresh ? kvDouble(ekf_local, "yaw") : nan;
    const double el_v = ekf_local_fresh ? kvDouble(ekf_local, "v") : nan;
    const double el_w = ekf_local_fresh ? kvDouble(ekf_local, "w") : nan;

    double esc_x = std::numeric_limits<double>::quiet_NaN();
    double esc_y = std::numeric_limits<double>::quiet_NaN();
    double esc_yaw = std::numeric_limits<double>::quiet_NaN();
    double esc_v = std::numeric_limits<double>::quiet_NaN();
    if (esc_odom_fresh) {
      esc_x = esc_odom.pose.pose.position.x;
      esc_y = esc_odom.pose.pose.position.y;
      esc_yaw = yawFromQuaternion(esc_odom.pose.pose.orientation);
      esc_v = esc_odom.twist.twist.linear.x;
    }
    if (!esc_steer_fresh) esc_steering = std::numeric_limits<double>::quiet_NaN();

    const double imu_roll = kvDouble(imu, "roll");
    const double imu_pitch = kvDouble(imu, "pitch");
    const double imu_yaw = kvDouble(imu, "yaw");
    const double imu_ax = kvDouble(imu, "ax");
    const double imu_ay = kvDouble(imu, "ay");
    const double imu_az = kvDouble(imu, "az");
    const double imu_gx = kvDouble(imu, "gx");
    const double imu_gy = kvDouble(imu, "gy");
    const double imu_gz = kvDouble(imu, "gz");
    const double imu_mx = kvDouble(imu, "mx_ut");
    const double imu_my = kvDouble(imu, "my_ut");
    const double imu_mz = kvDouble(imu, "mz_ut");

    // Raw detection tetap ditampilkan seluruhnya untuk kalibrasi, tetapi dibungkus
    // dua objek per baris agar HUD tidak melebar berlebihan pada 1920x1080.
    std::string raw_hud = raw_detection_fresh ? raw_detections : "--";
    int raw_lines = 1;
    if (raw_detection_fresh) {
      std::size_t search_pos = 0;
      int item_index = 0;
      while ((search_pos = raw_hud.find(" | ", search_pos)) != std::string::npos) {
        ++item_index;
        if ((item_index % 2) == 0) {
          raw_hud.replace(search_pos, 3, "\n          ");
          search_pos += 11;
          ++raw_lines;
        } else {
          search_pos += 3;
        }
      }
    }
    const int sensor_height = 184 + std::max(0, raw_lines - 1) * 12;
    auto sensor_hud = makeHudOverlay(1400, sensor_height, hud_margin_px_, hud_margin_px_, 0, 4, hud_text,
      gnss_quality == "STRICT" ? 0.68F : 1.0F, gnss_quality == "STRICT" ? 1.0F : 0.84F, 0.58F);
    std::ostringstream sensor_text;
    sensor_text << "SENSOR TELEMETRY\n"
                << "GNSS x " << fmtNumber(gnss_x) << " y " << fmtNumber(gnss_y)
                << "m | lon " << fmtNumber(lon, 7) << " lat " << fmtNumber(lat, 7) << "\n"
                << "     " << gnss_quality << " | sat " << fmtNumber(sat, 0)
                << " DOP " << fmtNumber(dop) << " hAcc " << fmtNumber(hacc)
                << "m sAcc " << fmtNumber(sacc) << "m/s\n"
                << "GNSS yaw " << fmtNumber(radToDeg(gnss_yaw)) << "deg | vx "
                << fmtNumber(gnss_vx) << "m/s | vyaw "
                << (gnss_vyaw_valid ? fmtNumber(radToDeg(gnss_vyaw)) : std::string("--"))
                << " deg/s | " << (gnss_vyaw_valid ? "VALID" : "LOW-SPEED/INVALID") << "\n"
                << "GNSS yaw " << fmtNumber(radToDeg(gnss_yaw)) << "deg | vx "
                << fmtNumber(gnss_vx) << "m/s | vyaw "
                << (gnss_vyaw_valid ? fmtNumber(radToDeg(gnss_vyaw)) : std::string("--"))
                << " deg/s " << (gnss_vyaw_valid ? "VALID" : "LOW-SPEED/INVALID") << "\n"
                << "EKF-G x " << fmtNumber(eg_x) << " y " << fmtNumber(eg_y)
                << " yaw " << fmtNumber(radToDeg(eg_yaw)) << "deg | speed " << fmtNumber(eg_v)
                << "m/s speedyaw " << fmtNumber(radToDeg(eg_w)) << "deg/s\n"
                << "EKF-L x " << fmtNumber(el_x) << " y " << fmtNumber(el_y)
                << " yaw " << fmtNumber(radToDeg(el_yaw)) << "deg | speed " << fmtNumber(el_v)
                << "m/s speedyaw " << fmtNumber(radToDeg(el_w)) << "deg/s\n"
                << "ESC   x " << fmtNumber(esc_x) << " y " << fmtNumber(esc_y)
                << " yaw " << fmtNumber(radToDeg(esc_yaw)) << "deg | odom speed " << fmtNumber(esc_v)
                << "m/s | steering " << fmtNumber(radToDeg(esc_steering)) << "deg\n"
                << "IMU   pitch " << fmtNumber(radToDeg(imu_pitch)) << " roll " << fmtNumber(radToDeg(imu_roll))
                << " yaw " << fmtNumber(radToDeg(imu_yaw)) << "deg | acc " << fmtNumber(imu_ax)
                << " " << fmtNumber(imu_ay) << " " << fmtNumber(imu_az) << " m/s2\n"
                << "      gyro " << fmtNumber(radToDeg(imu_gx)) << " " << fmtNumber(radToDeg(imu_gy))
                << " " << fmtNumber(radToDeg(imu_gz)) << " deg/s | mag " << fmtNumber(imu_mx)
                << " " << fmtNumber(imu_my) << " " << fmtNumber(imu_mz) << " uT\n"
                << "YOLOP RAW " << raw_hud;
    sensor_hud.data = sensor_text.str();
    hud_sensor_pub_->publish(sensor_hud);

    if (!esc_drive_target_fresh) esc_drive_target = std::numeric_limits<double>::quiet_NaN();
    if (!esc_drive_actual_fresh) esc_drive_actual = std::numeric_limits<double>::quiet_NaN();
    if (!esc_steering_target_fresh) esc_steering_target = std::numeric_limits<double>::quiet_NaN();

    // KANAN ATAS V19: urutan kausal command autonomous yang sebenarnya.
    // MPPI=/cmd_vel_nav_raw -> SMOOTHER=/cmd_vel_nav_smoothed ->
    // TRAJECTORY SAFETY=/cmd_vel/autonomy_integrated -> FINAL=/cmd_vel.
    // Perception hanya constraint/advisory, bukan owner command autonomous.
    auto cmd_hud = makeHudOverlay(560, 154, hud_margin_px_, hud_margin_px_, 1, 3, hud_text,
      0.78F, 1.0F, 0.72F);
    const auto commandLine = [this](const char * label, const CommandSample & sample) {
      std::ostringstream line;
      line << std::left << std::setw(11) << label
           << " speed " << std::right << std::setw(5) << fmtNumber(sample.cmd.linear.x)
           << " m/s | steering " << std::setw(5)
           << fmtNumber(radToDeg(steeringFromTwistRad(sample.cmd))) << " deg";
      return line.str();
    };
    std::ostringstream cmd_text;
    cmd_text << "COMMAND PIPELINE  [RIGHT steering = +]\n"
             << commandLine("TELEOP", teleop) << "\n"
             << commandLine("MPPI RAW", raw) << "\n"
             << commandLine("SMOOTHER", smoothed) << "\n"
             << commandLine("NAV2 FINAL", final) << "\n"
             << "ESC " << (esc_feedback_valid ? "ACK" : "NO-ACK")
             << " | SRC " << (esc_mux_status.empty() ? "IDLE" : esc_mux_status)
             << " | " << (esc_ready ? "READY" : "NOT READY") << " | " << (esc_armed ? "ARMED" : "DISARMED") << "\n"
             << "ESC TARGET speed " << fmtNumber(esc_drive_target)
             << " m/s | steering " << fmtNumber(radToDeg(esc_steering_target)) << " deg\n"
             << "ESC ACTUAL speed " << fmtNumber(esc_drive_actual)
             << " m/s | steering " << fmtNumber(radToDeg(esc_steering)) << " deg";
    cmd_hud.data = cmd_text.str();
    hud_drive_pub_->publish(cmd_hud);

    // Bukti runtime numerik bahwa command benar-benar keluar dari MPPI dan
    // berubah di setiap post-processing stage. Log hanya 2 detik sekali saat goal aktif.
    if (goal_state == "ACTIVE" &&
        (last_cmd_pipeline_log_time_.nanoseconds() == 0 ||
         (snapshot_time - last_cmd_pipeline_log_time_).seconds() >= 2.0)) {
      RCLCPP_INFO(
        get_logger(),
        "CMD PIPELINE MPPI[v=%.3f w=%.3f steer=%.2fdeg] -> SMOOTHER[v=%.3f w=%.3f steer=%.2fdeg] -> YOLOPv2[v=%.3f w=%.3f steer=%.2fdeg] -> FINAL[v=%.3f w=%.3f steer=%.2fdeg]",
        raw.cmd.linear.x, raw.cmd.angular.z, radToDeg(steeringFromTwistRad(raw.cmd)),
        smoothed.cmd.linear.x, smoothed.cmd.angular.z, radToDeg(steeringFromTwistRad(smoothed.cmd)),
        yolop.cmd.linear.x, yolop.cmd.angular.z, radToDeg(steeringFromTwistRad(yolop.cmd)),
        final.cmd.linear.x, final.cmd.angular.z, radToDeg(steeringFromTwistRad(final.cmd)));
      last_cmd_pipeline_log_time_ = snapshot_time;
    }

    const bool changed = status.data != last_status_text_;
    const bool heartbeat = last_status_log_time_.nanoseconds() == 0 ||
      (snapshot_time - last_status_log_time_).seconds() >= log_heartbeat_sec_;
    if (changed || heartbeat) {
      RCLCPP_INFO(get_logger(), "SYSTEM %s", status.data.c_str());
      last_status_text_ = status.data;
      last_status_log_time_ = snapshot_time;
    }
  }

  std::mutex mutex_;
  std::string autonomy_topic_, pre_collision_topic_, final_topic_;
  std::string goal_topic_, legacy_goal_topic_, action_name_, lifecycle_manager_service_, map_topic_;
  bool defer_nav2_startup_until_localization_{true};
  double map_nav2_startup_settle_sec_{2.5};
  bool collision_monitor_enabled_{false};
  double command_rate_hz_{20.0}, status_rate_hz_{10.0}, autonomy_timeout_sec_{0.60};
  double max_forward_speed_mps_{0.5}, max_reverse_speed_mps_{0.3}, max_yaw_rate_rps_{0.292028888392};
  double linear_deadband_mps_{0.08}, angular_deadband_rps_{0.02}, min_speed_for_yaw_mps_{0.08};
  bool require_perception_for_motion_{true};
  double perception_timeout_sec_{1.5};
  bool require_sensor_publisher_contract_{true};
  std::string sensor_publisher_contract_topic_{"/system/sensor_publishers_ok"};
  double sensor_publisher_contract_timeout_sec_{0.75};
  bool require_camera_calibration_{true}, camera_calibration_validated_{false};
  bool require_steering_calibration_{true};
  bool steering_calibration_validated_{false};
  bool require_steering_circle_calibration_{true};
  bool steering_circle_calibration_validated_{false};
  bool require_drive_odometry_calibration_{true};
  bool drive_odometry_calibration_validated_{false};
  bool require_imu_calibration_{true};
  bool imu_calibration_validated_{false};
  bool require_stage3_production_certification_{true};
  bool require_collision_monitor_for_production_{true};
  bool require_perception_for_production_{true};
  bool stage3_production_certified_{false};
  bool precision_mode_{false};
  bool precision_localization_ready_{false};
  bool stage3_commissioning_mode_{false};
  double stage3_commissioning_speed_cap_mps_{1.0};
  double overlay_text_size_px_{9.0}, log_heartbeat_sec_{10.0};
  int hud_margin_px_{12};
  bool keyboard_available_{false};
  double wheelbase_m_{0.70}, track_width_m_{0.48};
  double max_steering_angle_rad_{0.488692190558};
  double minimum_turning_radius_m_{1.712159378317};

  bool map_ready_{false};
  rclcpp::Time map_ready_since_{0, 0, RCL_ROS_TIME};
  uint32_t map_width_{0}, map_height_{0};
  float map_resolution_{0.0F};
  bool planning_localization_ready_{false}, motion_localization_ready_{false};
  bool sensor_publisher_contract_ok_{false};
  rclcpp::Time last_sensor_publisher_contract_time_{0, 0, RCL_ROS_TIME};
  bool nav2_action_ready_{false}, nav2_lifecycle_started_{false}, nav2_startup_requested_{false};
  bool velocity_smoother_active_{false};
  bool smoother_state_request_in_flight_{false}, smoother_transition_request_in_flight_{false};
  bool estop_{false}, esc_ready_{false};
  bool gnss_connected_{false}, imu_connected_{false}, camera_connected_{false};
  bool esc_drive_connected_{false}, esc_steer_connected_{false}, esc_armed_{false}, esc_feedback_valid_{false};
  rclcpp::Time last_perception_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_raw_detection_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time gnss_map_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time esc_odom_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time esc_steering_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time esc_drive_target_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time esc_drive_actual_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time esc_steering_target_received_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry gnss_map_odom_{};
  nav_msgs::msg::Odometry esc_odom_{};
  bool have_gnss_map_{false}, have_esc_odom_{false};
  double esc_steering_actual_rad_{0.0}, esc_drive_target_mps_{0.0}, esc_drive_actual_mps_{0.0}, esc_steering_target_rad_{0.0};
  std::string perception_state_, raw_detection_status_, localization_state_, gnss_status_, imu_status_, ekf_local_status_, ekf_global_status_;
  std::string esc_status_, esc_mux_status_, goal_state_{"IDLE"};
  std::string last_status_text_;
  rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cmd_pipeline_log_time_{0, 0, RCL_ROS_TIME};
  CommandSample autonomy_cmd_, teleop_cmd_, mppi_raw_, smoothed_, routed_cmd_, final_cmd_;

  std::optional<geometry_msgs::msg::PoseStamped> queued_goal_;
  geometry_msgs::msg::PoseStamped last_goal_pose_;
  uint64_t goal_rx_count_{0}, goal_send_sequence_{0}, current_goal_request_id_{0};
  bool goal_send_in_flight_{false};
  GoalHandleNavigate::SharedPtr active_goal_handle_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr autonomy_sub_, teleop_sub_, mppi_raw_sub_, smoothed_sub_, final_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_map_sub_, esc_odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_steering_actual_sub_, esc_drive_target_sub_, esc_drive_actual_sub_, esc_steering_target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr planning_loc_sub_, motion_loc_sub_, sensor_contract_sub_, precision_localization_sub_, esc_ready_sub_, estop_sub_, esc_feedback_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gnss_connected_sub_, imu_connected_sub_, camera_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_drive_connected_sub_, esc_steer_connected_sub_, esc_armed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_state_sub_, gnss_status_sub_, imu_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ekf_local_status_sub_, ekf_global_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esc_status_sub_, esc_mux_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perception_state_sub_, raw_detection_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_, legacy_goal_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pre_collision_pub_, final_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr autonomy_motion_allowed_pub_, planning_ready_pub_, motion_ready_pub_, autonomy_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sensor_status_pub_, goal_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr hud_nav_pub_, hud_sensor_pub_, hud_drive_pub_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Client<ManageLifecycleNodes>::SharedPtr nav2_lifecycle_client_;
  rclcpp::Client<GetState>::SharedPtr smoother_get_state_client_;
  rclcpp::Client<ChangeState>::SharedPtr smoother_change_state_client_;
  rclcpp::TimerBase::SharedPtr smoother_guard_timer_, nav2_startup_timer_, goal_timer_, command_timer_, status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationCore>());
  rclcpp::shutdown();
  return 0;
}
