#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace navigation {
namespace {


rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool valid{false};
  rclcpp::Time received{0, 0, RCL_ROS_TIME};
};

struct TimedTwist
{
  geometry_msgs::msg::Twist msg{};
  bool valid{false};
  rclcpp::Time received{0, 0, RCL_ROS_TIME};
};

struct TimedPath
{
  nav_msgs::msg::Path msg{};
  bool valid{false};
  rclcpp::Time received{0, 0, RCL_ROS_TIME};
};

struct BasePoint
{
  double x{0.0};
  double y{0.0};
};

struct PathSnapshot
{
  std::vector<BasePoint> points;
  bool valid{false};
  std::string source;
};

struct DrivableEnvelope
{
  std::vector<BasePoint> left;
  std::vector<BasePoint> right;
  bool valid{false};
  rclcpp::Time received{0, 0, RCL_ROS_TIME};
};

struct FreeCorridor
{
  bool valid{false};
  bool left_free{false};
  bool right_free{false};
  double left_available_m{0.0};
  double right_available_m{0.0};
  double required_width_m{0.0};
  std::string preferred{"NONE"};
};

struct CorridorHit
{
  bool relevant{false};
  double lateral_distance_m{std::numeric_limits<double>::infinity()};
  double along_path_m{std::numeric_limits<double>::infinity()};
};

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

std::string normalizeFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
  return frame;
}

bool finiteTwist(const geometry_msgs::msg::Twist & cmd)
{
  return std::isfinite(cmd.linear.x) && std::isfinite(cmd.angular.z);
}

bool extractBoolField(const std::string & text, const std::string & key, bool fallback)
{
  const std::string token = "\"" + key + "\":";
  const auto pos = text.find(token);
  if (pos == std::string::npos) return fallback;
  const auto value = pos + token.size();
  if (text.compare(value, 4, "true") == 0) return true;
  if (text.compare(value, 5, "false") == 0) return false;
  return fallback;
}

double extractDoubleField(const std::string & text, const std::string & key, double fallback)
{
  const std::string token = "\"" + key + "\":";
  const auto pos = text.find(token);
  if (pos == std::string::npos) return fallback;
  std::istringstream stream(text.substr(pos + token.size()));
  double value = fallback;
  if (!(stream >> value) || !std::isfinite(value)) return fallback;
  return value;
}

std::string extractStringField(
  const std::string & text, const std::string & key, const std::string & fallback)
{
  const std::string token = "\"" + key + "\":\"";
  const auto pos = text.find(token);
  if (pos == std::string::npos) return fallback;
  const auto begin = pos + token.size();
  const auto end = text.find('"', begin);
  if (end == std::string::npos) return fallback;
  return text.substr(begin, end - begin);
}

bool fieldOffset(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name, uint32_t & offset)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      offset = field.offset;
      return true;
    }
  }
  return false;
}

float readFloat(const uint8_t * ptr)
{
  float value = 0.0F;
  std::memcpy(&value, ptr, sizeof(float));
  return value;
}

bool nearestBoundaryY(
  const std::vector<BasePoint> &points, double forward_m, double tolerance_m, double &y)
{
  double best = std::numeric_limits<double>::infinity();
  bool found = false;
  for (const auto &p : points) {
    const double dx = std::abs(p.x - forward_m);
    if (dx <= tolerance_m && dx < best) {
      best = dx;
      y = p.y;
      found = true;
    }
  }
  return found;
}

FreeCorridor evaluateFreeCorridor(
  const DrivableEnvelope &envelope,
  double obstacle_forward_m,
  double obstacle_right_edge_m,
  double obstacle_left_edge_m,
  double vehicle_width_m,
  double side_margin_m,
  double boundary_lookup_tolerance_m)
{
  FreeCorridor result;
  result.required_width_m = vehicle_width_m + 2.0 * side_margin_m;
  if (!envelope.valid || !std::isfinite(obstacle_forward_m) ||
      !std::isfinite(obstacle_right_edge_m) || !std::isfinite(obstacle_left_edge_m)) return result;

  double left_boundary = 0.0;
  double right_boundary = 0.0;
  const bool have_left = nearestBoundaryY(
    envelope.left, obstacle_forward_m, boundary_lookup_tolerance_m, left_boundary);
  const bool have_right = nearestBoundaryY(
    envelope.right, obstacle_forward_m, boundary_lookup_tolerance_m, right_boundary);
  if (have_left) {
    result.left_available_m = std::max(0.0, left_boundary - obstacle_left_edge_m);
    result.left_free = result.left_available_m >= result.required_width_m;
  }
  if (have_right) {
    result.right_available_m = std::max(0.0, obstacle_right_edge_m - right_boundary);
    result.right_free = result.right_available_m >= result.required_width_m;
  }
  result.valid = have_left || have_right;
  if (result.left_free && result.right_free) {
    result.preferred = result.left_available_m >= result.right_available_m ? "LEFT" : "RIGHT";
  } else if (result.left_free) {
    result.preferred = "LEFT";
  } else if (result.right_free) {
    result.preferred = "RIGHT";
  }
  return result;
}

CorridorHit pointToCorridor(
  const BasePoint & obstacle,
  const std::vector<BasePoint> & path,
  double corridor_half_width_m,
  double horizon_m)
{
  CorridorHit best;
  if (path.empty()) return best;
  if (path.size() == 1U) {
    const double distance = std::hypot(obstacle.x - path.front().x, obstacle.y - path.front().y);
    best.lateral_distance_m = distance;
    best.along_path_m = 0.0;
    best.relevant = distance <= corridor_half_width_m;
    return best;
  }

  double cumulative = 0.0;
  for (size_t i = 0; i + 1U < path.size(); ++i) {
    const BasePoint a = path[i];
    const BasePoint b = path[i + 1U];
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double segment_length = std::hypot(vx, vy);
    if (segment_length <= 1.0e-6) continue;
    if (cumulative > horizon_m) break;

    const double wx = obstacle.x - a.x;
    const double wy = obstacle.y - a.y;
    const double denom = vx * vx + vy * vy;
    const double t = std::clamp((wx * vx + wy * vy) / denom, 0.0, 1.0);
    const double px = a.x + t * vx;
    const double py = a.y + t * vy;
    const double distance = std::hypot(obstacle.x - px, obstacle.y - py);
    const double along = cumulative + t * segment_length;

    if (along <= horizon_m && distance < best.lateral_distance_m) {
      best.lateral_distance_m = distance;
      best.along_path_m = along;
    }
    cumulative += segment_length;
  }

  best.relevant = std::isfinite(best.along_path_m) &&
    best.along_path_m >= 0.0 && best.along_path_m <= horizon_m &&
    best.lateral_distance_m <= corridor_half_width_m;
  return best;
}

std::vector<BasePoint> buildCommandSweptPath(
  const geometry_msgs::msg::Twist & cmd,
  double horizon_m,
  double sample_step_m)
{
  std::vector<BasePoint> path;
  path.push_back({0.0, 0.0});
  if (!finiteTwist(cmd) || cmd.linear.x <= 1.0e-4 || horizon_m <= 0.0) return path;

  const double step = std::clamp(sample_step_m, 0.02, std::max(0.02, horizon_m));
  const double curvature = std::abs(cmd.linear.x) > 1.0e-4 ? cmd.angular.z / cmd.linear.x : 0.0;
  for (double along = step; along <= horizon_m + 1.0e-9; along += step) {
    if (std::abs(curvature) < 1.0e-5) {
      path.push_back({along, 0.0});
    } else {
      const double angle = curvature * along;
      path.push_back({std::sin(angle) / curvature, (1.0 - std::cos(angle)) / curvature});
    }
  }
  if (path.size() == 1U) path.push_back({horizon_m, 0.0});
  return path;
}

}  // namespace

class TrajectorySafetySupervisor final : public rclcpp::Node
{
public:
  TrajectorySafetySupervisor()
  : Node("trajectory_safety_supervisor")
  {
    declareParameters();
    readParameters();
    createInterfaces();

    RCLCPP_INFO(
      get_logger(),
      "TrajectorySafetySupervisor aktif: mode=%s NAV2=%s + PATH(global=%s/local=%s) + perception constraints -> %s; corridor=+/-%.2fm horizon=%.2fm",
      metric_obstacle_safety_enabled_ ? "METRIC_FULL" : "GUARD_ONLY",
      nav_cmd_topic_.c_str(), global_plan_topic_.c_str(), local_plan_topic_.c_str(),
      output_cmd_topic_.c_str(), path_corridor_half_width_m_, path_horizon_m_);
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("nav_cmd_topic", "/cmd_vel_nav_raw");
    declare_parameter<std::string>("perception_advisory_topic", "/cmd_vel/perception_advisory");
    declare_parameter<std::string>("output_cmd_topic", "/cmd_vel/autonomy_integrated");
    declare_parameter<std::string>("global_plan_topic", "/plan");
    declare_parameter<std::string>("local_plan_topic", "/controller_server/transformed_global_plan");
    declare_parameter<std::string>("map_pose_topic", "/localization/pose_estimator_map");
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("candidate_obstacle_topic", "/perception/object_points");
    declare_parameter<std::string>("path_relevant_obstacle_topic", "/perception/path_relevant_points");
    declare_parameter<std::string>("planning_relevant_obstacle_topic", "/perception/planning_relevant_points");
    declare_parameter<std::string>("drivable_boundary_topic", "/perception/drivable_boundary_points");
    declare_parameter<std::string>("avoidance_hint_topic", "/navigation/avoidance_hint");
    declare_parameter<std::string>("lane_state_topic", "/perception/lane_safety_state");
    declare_parameter<std::string>("lane_control_state_topic", "/perception/lane_control_state");
    declare_parameter<std::string>("camera_connected_topic", "/perception/camera_connected");
    declare_parameter<std::string>("camera_health_topic", "/perception/camera_healthy");
    declare_parameter<std::string>("emergency_stop_topic", "/perception/emergency_stop");
    declare_parameter<std::string>("perception_performance_topic", "/perception/performance");
    declare_parameter<std::string>("state_topic", "/navigation/trajectory_safety_state");

    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("status_log_period_sec", 1.0);
    declare_parameter<double>("nav_cmd_timeout_sec", 0.60);
    declare_parameter<double>("advisory_timeout_sec", 0.50);
    declare_parameter<double>("plan_timeout_sec", 3.0);
    declare_parameter<double>("pose_timeout_sec", 0.60);
    declare_parameter<double>("obstacle_timeout_sec", 0.90);
    declare_parameter<double>("lane_timeout_sec", 0.75);
    declare_parameter<double>("drivable_timeout_sec", 0.75);
    declare_parameter<double>("perception_performance_timeout_sec", 2.0);

    // Metric obstacle/path decisions require validated homography. Guard-only mode
    // still enforces camera health + image-space near-field emergency stop.
    declare_parameter<bool>("metric_obstacle_safety_enabled", false);
    declare_parameter<bool>("latency_compensation_enabled", true);
    declare_parameter<double>("perception_latency_fallback_sec", 1.0);
    declare_parameter<double>("latency_safety_margin_sec", 0.15);
    declare_parameter<double>("latency_distance_margin_m", 0.25);
    declare_parameter<double>("braking_deceleration_mps2", 1.0);
    declare_parameter<double>("max_dynamic_stop_distance_m", 2.50);

    declare_parameter<double>("vehicle_width_m", 0.60);
    declare_parameter<double>("trajectory_lateral_margin_m", 0.25);
    // Planning envelope sengaja lebih lebar dari collision corridor agar obstacle
    // tetap terlihat oleh MPPI ketika path sedang menghindar dan tidak berosilasi
    // hilang-muncul hanya karena path bergeser beberapa puluh sentimeter.
    declare_parameter<double>("planning_lateral_margin_m", 0.65);
    // Planning cloud tidak dibatasi hanya pada current path. MPPI harus melihat
    // obstacle di corridor alternatif kiri/kanan sebelum memilih avoidance.
    declare_parameter<double>("planning_min_forward_m", 0.15);
    declare_parameter<double>("planning_max_abs_lateral_m", 2.6);
    declare_parameter<double>("path_horizon_m", 4.0);
    // Swept-command hanya lapisan reaksi sangat dekat. Keputusan obstacle normal
    // tetap memakai future Nav2 path, sehingga parked car di seberang tikungan
    // tidak menghentikan kendaraan hanya karena tampak lurus di depan kamera.
    declare_parameter<double>("command_collision_horizon_m", 0.90);
    declare_parameter<double>("command_collision_sample_step_m", 0.05);
    declare_parameter<double>("command_collision_lateral_margin_m", 0.20);
    declare_parameter<double>("immediate_command_hard_stop_m", 0.65);
    declare_parameter<double>("hard_stop_path_distance_m", 0.90);
    declare_parameter<double>("slow_path_distance_m", 2.20);
    declare_parameter<double>("minimum_slow_speed_scale", 0.35);
    declare_parameter<double>("avoidance_side_margin_m", 0.25);
    declare_parameter<double>("avoidance_boundary_lookup_tolerance_m", 0.28);
    declare_parameter<double>("avoidance_speed_mps", 0.18);
    declare_parameter<bool>("require_drivable_space_for_avoidance", true);
    declare_parameter<double>("critical_lane_speed_mps", 0.10);
    declare_parameter<double>("warning_lane_speed_mps", 0.20);
    declare_parameter<double>("maximum_yaw_rate_rps", 0.825470725);
    declare_parameter<double>("minimum_turning_radius_m", 1.712159378317);
    declare_parameter<double>("lane_advisory_blend", 1.0);
    // Lane safety is independently switchable from obstacle/path safety. When
    // false, no lane state, lane-control veto, speed cap, or yaw advisory may
    // alter Nav2 commands.
    declare_parameter<bool>("lane_safety_enabled", false);

    declare_parameter<bool>("require_plan_when_moving", true);
    declare_parameter<bool>("require_camera_connected", true);
    declare_parameter<bool>("require_camera_health", true);
    declare_parameter<bool>("require_obstacle_stream_when_moving", true);
    declare_parameter<bool>("stop_on_lane_lost", false);
    declare_parameter<bool>("use_local_plan_when_available", true);
  }

  void readParameters()
  {
    nav_cmd_topic_ = get_parameter("nav_cmd_topic").as_string();
    perception_advisory_topic_ = get_parameter("perception_advisory_topic").as_string();
    output_cmd_topic_ = get_parameter("output_cmd_topic").as_string();
    global_plan_topic_ = get_parameter("global_plan_topic").as_string();
    local_plan_topic_ = get_parameter("local_plan_topic").as_string();
    map_pose_topic_ = get_parameter("map_pose_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    candidate_obstacle_topic_ = get_parameter("candidate_obstacle_topic").as_string();
    path_relevant_obstacle_topic_ = get_parameter("path_relevant_obstacle_topic").as_string();
    planning_relevant_obstacle_topic_ = get_parameter("planning_relevant_obstacle_topic").as_string();
    drivable_boundary_topic_ = get_parameter("drivable_boundary_topic").as_string();
    avoidance_hint_topic_ = get_parameter("avoidance_hint_topic").as_string();
    lane_state_topic_ = get_parameter("lane_state_topic").as_string();
    lane_control_state_topic_ = get_parameter("lane_control_state_topic").as_string();
    camera_connected_topic_ = get_parameter("camera_connected_topic").as_string();
    camera_health_topic_ = get_parameter("camera_health_topic").as_string();
    emergency_stop_topic_ = get_parameter("emergency_stop_topic").as_string();
    perception_performance_topic_ = get_parameter("perception_performance_topic").as_string();
    state_topic_ = get_parameter("state_topic").as_string();

    control_rate_hz_ = std::clamp(get_parameter("control_rate_hz").as_double(), 2.0, 100.0);
    status_log_period_sec_ = std::clamp(get_parameter("status_log_period_sec").as_double(), 0.2, 10.0);
    nav_cmd_timeout_sec_ = std::max(0.05, get_parameter("nav_cmd_timeout_sec").as_double());
    advisory_timeout_sec_ = std::max(0.05, get_parameter("advisory_timeout_sec").as_double());
    plan_timeout_sec_ = std::max(0.20, get_parameter("plan_timeout_sec").as_double());
    pose_timeout_sec_ = std::max(0.10, get_parameter("pose_timeout_sec").as_double());
    obstacle_timeout_sec_ = std::max(0.10, get_parameter("obstacle_timeout_sec").as_double());
    lane_timeout_sec_ = std::max(0.10, get_parameter("lane_timeout_sec").as_double());
    drivable_timeout_sec_ = std::max(0.10, get_parameter("drivable_timeout_sec").as_double());
    perception_performance_timeout_sec_ = std::max(0.20, get_parameter("perception_performance_timeout_sec").as_double());
    metric_obstacle_safety_enabled_ = get_parameter("metric_obstacle_safety_enabled").as_bool();
    latency_compensation_enabled_ = get_parameter("latency_compensation_enabled").as_bool();
    perception_latency_fallback_sec_ = std::clamp(get_parameter("perception_latency_fallback_sec").as_double(), 0.05, 3.0);
    latency_safety_margin_sec_ = std::clamp(get_parameter("latency_safety_margin_sec").as_double(), 0.0, 1.0);
    latency_distance_margin_m_ = std::clamp(get_parameter("latency_distance_margin_m").as_double(), 0.0, 1.5);
    braking_deceleration_mps2_ = std::clamp(get_parameter("braking_deceleration_mps2").as_double(), 0.2, 5.0);
    max_dynamic_stop_distance_m_ = std::clamp(get_parameter("max_dynamic_stop_distance_m").as_double(), 0.5, 4.0);

    vehicle_width_m_ = std::max(0.10, get_parameter("vehicle_width_m").as_double());
    trajectory_lateral_margin_m_ = std::max(0.0, get_parameter("trajectory_lateral_margin_m").as_double());
    path_corridor_half_width_m_ = 0.5 * vehicle_width_m_ + trajectory_lateral_margin_m_;
    planning_lateral_margin_m_ = std::max(trajectory_lateral_margin_m_, get_parameter("planning_lateral_margin_m").as_double());
    planning_corridor_half_width_m_ = 0.5 * vehicle_width_m_ + planning_lateral_margin_m_;
    planning_min_forward_m_ = std::max(0.0, get_parameter("planning_min_forward_m").as_double());
    planning_max_abs_lateral_m_ = std::max(0.5, get_parameter("planning_max_abs_lateral_m").as_double());
    path_horizon_m_ = std::max(0.5, get_parameter("path_horizon_m").as_double());
    command_collision_horizon_m_ =
      std::max(0.5, get_parameter("command_collision_horizon_m").as_double());
    command_collision_sample_step_m_ =
      std::clamp(get_parameter("command_collision_sample_step_m").as_double(), 0.02, 0.50);
    command_collision_lateral_margin_m_ =
      std::max(0.0, get_parameter("command_collision_lateral_margin_m").as_double());
    command_collision_half_width_m_ =
      0.5 * vehicle_width_m_ + command_collision_lateral_margin_m_;
    immediate_command_hard_stop_m_ = std::min(
      command_collision_horizon_m_,
      std::max(0.10, get_parameter("immediate_command_hard_stop_m").as_double()));
    hard_stop_path_distance_m_ = std::max(0.05, get_parameter("hard_stop_path_distance_m").as_double());
    slow_path_distance_m_ = std::max(
      hard_stop_path_distance_m_ + 0.05, get_parameter("slow_path_distance_m").as_double());
    minimum_slow_speed_scale_ = std::clamp(
      get_parameter("minimum_slow_speed_scale").as_double(), 0.0, 1.0);
    avoidance_side_margin_m_ = std::max(0.0, get_parameter("avoidance_side_margin_m").as_double());
    avoidance_boundary_lookup_tolerance_m_ = std::clamp(
      get_parameter("avoidance_boundary_lookup_tolerance_m").as_double(), 0.05, 1.0);
    avoidance_speed_mps_ = std::max(0.0, get_parameter("avoidance_speed_mps").as_double());
    require_drivable_space_for_avoidance_ = get_parameter("require_drivable_space_for_avoidance").as_bool();
    critical_lane_speed_mps_ = std::max(0.0, get_parameter("critical_lane_speed_mps").as_double());
    warning_lane_speed_mps_ = std::max(critical_lane_speed_mps_, get_parameter("warning_lane_speed_mps").as_double());
    maximum_yaw_rate_rps_ = std::max(0.05, get_parameter("maximum_yaw_rate_rps").as_double());
    minimum_turning_radius_m_ = std::max(0.10, get_parameter("minimum_turning_radius_m").as_double());
    lane_advisory_blend_ = std::clamp(get_parameter("lane_advisory_blend").as_double(), 0.0, 1.0);
    lane_safety_enabled_ = get_parameter("lane_safety_enabled").as_bool();

    require_plan_when_moving_ = get_parameter("require_plan_when_moving").as_bool();
    require_camera_connected_ = get_parameter("require_camera_connected").as_bool();
    require_camera_health_ = get_parameter("require_camera_health").as_bool();
    require_obstacle_stream_when_moving_ = get_parameter("require_obstacle_stream_when_moving").as_bool();
    stop_on_lane_lost_ = get_parameter("stop_on_lane_lost").as_bool();
    use_local_plan_when_available_ = get_parameter("use_local_plan_when_available").as_bool();
  }

  void createInterfaces()
  {
    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    const auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    nav_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic_, cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        nav_cmd_.valid = finiteTwist(*msg);
        nav_cmd_.msg = nav_cmd_.valid ? *msg : geometry_msgs::msg::Twist{};
        nav_cmd_.received = now();
      });

    advisory_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      perception_advisory_topic_, cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        advisory_cmd_.valid = finiteTwist(*msg);
        advisory_cmd_.msg = advisory_cmd_.valid ? *msg : geometry_msgs::msg::Twist{};
        advisory_cmd_.received = now();
      });

    global_plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_plan_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        global_plan_.msg = *msg;
        global_plan_.valid = !msg->poses.empty();
        global_plan_.received = now();
      });

    local_plan_sub_ = create_subscription<nav_msgs::msg::Path>(
      local_plan_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](nav_msgs::msg::Path::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        local_plan_.msg = *msg;
        local_plan_.valid = !msg->poses.empty();
        local_plan_.received = now();
      });

    map_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      map_pose_topic_, stateQos(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        map_pose_.x = msg->pose.pose.position.x;
        map_pose_.y = msg->pose.pose.position.y;
        map_pose_.yaw = yawFromQuaternion(msg->pose.pose.orientation);
        map_pose_.valid = std::isfinite(map_pose_.x) && std::isfinite(map_pose_.y) && std::isfinite(map_pose_.yaw);
        map_pose_.received = now();
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_pose_.x = msg->pose.pose.position.x;
        odom_pose_.y = msg->pose.pose.position.y;
        odom_pose_.yaw = yawFromQuaternion(msg->pose.pose.orientation);
        odom_pose_.valid = std::isfinite(odom_pose_.x) && std::isfinite(odom_pose_.y) && std::isfinite(odom_pose_.yaw);
        odom_pose_.received = now();
      });

    drivable_boundary_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      drivable_boundary_topic_, cloud_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        DrivableEnvelope envelope;
        envelope.received = now();
        uint32_t x_offset = 0U;
        uint32_t y_offset = 0U;
        const bool layout_ok = msg->point_step > 0U &&
          fieldOffset(*msg, "x", x_offset) && fieldOffset(*msg, "y", y_offset) &&
          x_offset + sizeof(float) <= msg->point_step && y_offset + sizeof(float) <= msg->point_step;
        if (layout_ok) {
          const size_t count = msg->data.size() / msg->point_step;
          envelope.left.reserve(count / 2U + 1U);
          envelope.right.reserve(count / 2U + 1U);
          for (size_t i = 0; i < count; ++i) {
            const auto *record = msg->data.data() + i * msg->point_step;
            const double x = static_cast<double>(readFloat(record + x_offset));
            const double y = static_cast<double>(readFloat(record + y_offset));
            if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0) continue;
            if (y >= 0.0) envelope.left.push_back({x, y});
            else envelope.right.push_back({x, y});
          }
          envelope.valid = !envelope.left.empty() || !envelope.right.empty();
        }
        std::lock_guard<std::mutex> lock(mutex_);
        drivable_envelope_ = std::move(envelope);
      });

    lane_state_sub_ = create_subscription<std_msgs::msg::String>(
      lane_state_topic_, sensor_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        lane_state_ = extractStringField(msg->data, "state", "UNKNOWN");
        lane_valid_ = extractBoolField(msg->data, "valid", false);
        lane_critical_ = extractBoolField(msg->data, "critical", false);
        lane_received_ = now();
        lane_seen_ = true;
      });

    lane_control_state_sub_ = create_subscription<std_msgs::msg::String>(
      lane_control_state_topic_, sensor_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        lane_control_decision_ = extractStringField(msg->data, "decision", "UNKNOWN");
        lane_recenter_blocked_ = extractBoolField(msg->data, "recenter_blocked", false);
        lane_control_critical_ = extractBoolField(msg->data, "critical", false);
        lane_control_received_ = now();
        lane_control_seen_ = true;
      });

    camera_connected_sub_ = create_subscription<std_msgs::msg::Bool>(
      camera_connected_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        camera_connected_ = msg->data;
        camera_connected_seen_ = true;
        camera_connected_received_ = now();
      });

    camera_health_sub_ = create_subscription<std_msgs::msg::Bool>(
      camera_health_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        camera_healthy_ = msg->data;
        camera_health_seen_ = true;
        camera_health_received_ = now();
      });

    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        perception_emergency_stop_ = msg->data;
        emergency_seen_ = true;
        emergency_received_ = now();
      });

    performance_sub_ = create_subscription<std_msgs::msg::String>(
      perception_performance_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](std_msgs::msg::String::SharedPtr msg) {
        const double p95_ms = extractDoubleField(msg->data, "pipeline_p95_ms", -1.0);
        const double current_ms = extractDoubleField(msg->data, "pipeline_ms", -1.0);
        const double chosen_ms = p95_ms > 0.0 ? p95_ms : current_ms;
        if (chosen_ms <= 0.0 || !std::isfinite(chosen_ms)) return;
        std::lock_guard<std::mutex> lock(mutex_);
        perception_latency_sec_ = std::clamp(chosen_ms / 1000.0, 0.0, 5.0);
        perception_latency_received_ = now();
        perception_latency_seen_ = true;
      });

    candidate_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      candidate_obstacle_topic_, cloud_qos,
      std::bind(&TrajectorySafetySupervisor::onCandidateCloud, this, std::placeholders::_1));

    integrated_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_topic_, cmd_qos);
    relevant_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      path_relevant_obstacle_topic_, cloud_qos);
    planning_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      planning_relevant_obstacle_topic_, cloud_qos);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, stateQos());
    avoidance_hint_pub_ = create_publisher<std_msgs::msg::String>(avoidance_hint_topic_, stateQos());

    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TrajectorySafetySupervisor::controlTick, this));
  }

  bool fresh(const rclcpp::Time & received, double timeout, const rclcpp::Time & t) const
  {
    if (received.nanoseconds() <= 0) return false;
    const double age = (t - received).seconds();
    return age >= 0.0 && age <= timeout;
  }

  PathSnapshot buildPathSnapshotUnlocked(const rclcpp::Time & t) const
  {
    const TimedPath * selected = nullptr;
    std::string source;
    if (use_local_plan_when_available_ && local_plan_.valid && fresh(local_plan_.received, plan_timeout_sec_, t)) {
      selected = &local_plan_;
      source = "local";
    } else if (global_plan_.valid && fresh(global_plan_.received, plan_timeout_sec_, t)) {
      selected = &global_plan_;
      source = "global";
    }
    if (!selected) return {};

    const std::string frame = normalizeFrame(selected->msg.header.frame_id);
    Pose2D robot_pose;
    bool identity = false;
    if (frame == "base_footprint" || frame == "base_link") {
      identity = true;
    } else if (frame == "map") {
      robot_pose = map_pose_;
      if (!robot_pose.valid || !fresh(robot_pose.received, pose_timeout_sec_, t)) return {};
    } else if (frame == "odom") {
      robot_pose = odom_pose_;
      if (!robot_pose.valid || !fresh(robot_pose.received, pose_timeout_sec_, t)) return {};
    } else {
      return {};
    }

    std::vector<BasePoint> transformed;
    transformed.reserve(selected->msg.poses.size());
    const double c = std::cos(robot_pose.yaw);
    const double s = std::sin(robot_pose.yaw);
    for (const auto & pose : selected->msg.poses) {
      const double px = pose.pose.position.x;
      const double py = pose.pose.position.y;
      if (!std::isfinite(px) || !std::isfinite(py)) continue;
      if (identity) {
        transformed.push_back({px, py});
      } else {
        const double dx = px - robot_pose.x;
        const double dy = py - robot_pose.y;
        transformed.push_back({c * dx + s * dy, -s * dx + c * dy});
      }
    }
    if (transformed.size() < 2U) return {};

    size_t nearest = 0U;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < transformed.size(); ++i) {
      const double distance = std::hypot(transformed[i].x, transformed[i].y);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = i;
      }
    }

    PathSnapshot snapshot;
    snapshot.source = source + ":" + frame;
    snapshot.points.reserve(transformed.size() - nearest);
    double accumulated = 0.0;
    BasePoint previous = transformed[nearest];
    snapshot.points.push_back(previous);
    for (size_t i = nearest + 1U; i < transformed.size(); ++i) {
      const double segment = std::hypot(transformed[i].x - previous.x, transformed[i].y - previous.y);
      accumulated += segment;
      snapshot.points.push_back(transformed[i]);
      previous = transformed[i];
      if (accumulated >= path_horizon_m_ + 1.0) break;
    }
    snapshot.valid = snapshot.points.size() >= 2U;
    return snapshot;
  }

  void onCandidateCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto t = now();
    PathSnapshot path;
    geometry_msgs::msg::Twist nav_cmd;
    bool nav_cmd_fresh = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_obstacle_stream_ = t;
      obstacle_stream_seen_ = true;
      path = buildPathSnapshotUnlocked(t);
      nav_cmd_fresh = nav_cmd_.valid && fresh(nav_cmd_.received, nav_cmd_timeout_sec_, t);
      if (nav_cmd_fresh) nav_cmd = nav_cmd_.msg;
    }
    const std::vector<BasePoint> command_path = nav_cmd_fresh ?
      buildCommandSweptPath(
        nav_cmd, command_collision_horizon_m_, command_collision_sample_step_m_) :
      std::vector<BasePoint>{};

    sensor_msgs::msg::PointCloud2 filtered = *msg;
    filtered.height = 1U;
    filtered.width = 0U;
    filtered.row_step = 0U;
    filtered.data.clear();
    filtered.is_dense = true;

    sensor_msgs::msg::PointCloud2 planning_filtered = *msg;
    planning_filtered.height = 1U;
    planning_filtered.width = 0U;
    planning_filtered.row_step = 0U;
    planning_filtered.data.clear();
    planning_filtered.is_dense = true;

    if (!metric_obstacle_safety_enabled_) {
      relevant_cloud_pub_->publish(filtered);
      planning_cloud_pub_->publish(planning_filtered);
      std::lock_guard<std::mutex> lock(mutex_);
      last_obstacle_stream_ = t;
      obstacle_stream_seen_ = true;
      latest_path_ = {};
      candidate_point_count_ = 0U;
      relevant_point_count_ = 0U;
      planning_relevant_point_count_ = 0U;
      min_obstacle_along_path_m_ = std::numeric_limits<double>::infinity();
      min_obstacle_along_command_m_ = std::numeric_limits<double>::infinity();
      closest_path_obstacle_forward_m_ = std::numeric_limits<double>::infinity();
      closest_path_obstacle_right_edge_m_ = std::numeric_limits<double>::infinity();
      closest_path_obstacle_left_edge_m_ = -std::numeric_limits<double>::infinity();
      last_relevant_cloud_ = t;
      return;
    }

    uint32_t x_offset = 0U;
    uint32_t y_offset = 0U;
    uint32_t track_offset = 0U;
    const bool cloud_layout_ok = msg->point_step >= sizeof(float) &&
      fieldOffset(*msg, "x", x_offset) && fieldOffset(*msg, "y", y_offset) &&
      x_offset + sizeof(float) <= msg->point_step && y_offset + sizeof(float) <= msg->point_step;
    const bool have_track_id = cloud_layout_ok && fieldOffset(*msg, "track_id", track_offset) &&
      track_offset + sizeof(float) <= msg->point_step;

    size_t candidate_count = 0U;
    size_t relevant_count = 0U;
    size_t planning_relevant_count = 0U;
    double minimum_along = std::numeric_limits<double>::infinity();
    double minimum_command_along = std::numeric_limits<double>::infinity();
    double closest_obstacle_forward = std::numeric_limits<double>::infinity();
    double closest_obstacle_right_edge = std::numeric_limits<double>::infinity();
    double closest_obstacle_left_edge = -std::numeric_limits<double>::infinity();
    int closest_obstacle_track_id = -1;
    if (cloud_layout_ok && path.valid && msg->point_step > 0U) {
      const size_t count = msg->data.size() / msg->point_step;
      filtered.data.reserve(msg->data.size());
      planning_filtered.data.reserve(msg->data.size());
      for (size_t i = 0; i < count; ++i) {
        const auto * record = msg->data.data() + i * msg->point_step;
        const double x = static_cast<double>(readFloat(record + x_offset));
        const double y = static_cast<double>(readFloat(record + y_offset));
        if (!std::isfinite(x) || !std::isfinite(y)) continue;
        ++candidate_count;
        const BasePoint obstacle{x, y};
        const CorridorHit hit = pointToCorridor(
          obstacle, path.points, path_corridor_half_width_m_, path_horizon_m_);
        // Planner harus melihat obstacle di seluruh ruang manuver lokal, bukan hanya
        // yang dekat current/global path. Dengan begitu alternate corridor tidak
        // dipilih ke arah obstacle yang sebelumnya disembunyikan oleh path filter.
        if (x >= planning_min_forward_m_ && x <= path_horizon_m_ &&
            std::abs(y) <= planning_max_abs_lateral_m_) {
          planning_filtered.data.insert(
            planning_filtered.data.end(), record, record + msg->point_step);
          ++planning_relevant_count;
        }
        if (!command_path.empty()) {
          const CorridorHit command_hit = pointToCorridor(
            obstacle, command_path, command_collision_half_width_m_, command_collision_horizon_m_);
          if (command_hit.relevant) {
            minimum_command_along = std::min(minimum_command_along, command_hit.along_path_m);
          }
        }
        if (!hit.relevant) continue;
        filtered.data.insert(filtered.data.end(), record, record + msg->point_step);
        ++relevant_count;
        const int track_id = have_track_id ?
          static_cast<int>(std::lround(static_cast<double>(readFloat(record + track_offset)))) : -1;
        if (hit.along_path_m < minimum_along) {
          minimum_along = hit.along_path_m;
          closest_obstacle_forward = x;
          closest_obstacle_right_edge = y;
          closest_obstacle_left_edge = y;
          closest_obstacle_track_id = track_id;
        } else if (!have_track_id && std::abs(x - closest_obstacle_forward) <= 0.12) {
          // Fallback untuk cloud legacy tanpa track_id.
          closest_obstacle_right_edge = std::min(closest_obstacle_right_edge, y);
          closest_obstacle_left_edge = std::max(closest_obstacle_left_edge, y);
        }
      }
      // Setelah track obstacle terdekat diketahui dari intersection terhadap path,
      // ambil SEMUA point milik track yang sama. Free-space harus memakai lebar
      // penuh obstacle, bukan hanya edge yang kebetulan masuk collision corridor.
      if (have_track_id && closest_obstacle_track_id >= 0) {
        closest_obstacle_right_edge = std::numeric_limits<double>::infinity();
        closest_obstacle_left_edge = -std::numeric_limits<double>::infinity();
        double forward_sum = 0.0;
        size_t track_points = 0U;
        for (size_t i = 0; i < count; ++i) {
          const auto *record = msg->data.data() + i * msg->point_step;
          const int track_id = static_cast<int>(std::lround(
            static_cast<double>(readFloat(record + track_offset))));
          if (track_id != closest_obstacle_track_id) continue;
          const double x = static_cast<double>(readFloat(record + x_offset));
          const double y = static_cast<double>(readFloat(record + y_offset));
          if (!std::isfinite(x) || !std::isfinite(y)) continue;
          forward_sum += x;
          ++track_points;
          closest_obstacle_right_edge = std::min(closest_obstacle_right_edge, y);
          closest_obstacle_left_edge = std::max(closest_obstacle_left_edge, y);
        }
        if (track_points > 0U) closest_obstacle_forward = forward_sum / static_cast<double>(track_points);
      }
    } else if (cloud_layout_ok) {
      candidate_count = msg->data.size() / std::max<uint32_t>(1U, msg->point_step);
    }

    filtered.width = static_cast<uint32_t>(relevant_count);
    filtered.row_step = filtered.point_step * filtered.width;
    relevant_cloud_pub_->publish(filtered);

    planning_filtered.width = static_cast<uint32_t>(planning_relevant_count);
    planning_filtered.row_step = planning_filtered.point_step * planning_filtered.width;
    planning_cloud_pub_->publish(planning_filtered);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_path_ = path;
      candidate_point_count_ = candidate_count;
      relevant_point_count_ = relevant_count;
      planning_relevant_point_count_ = planning_relevant_count;
      min_obstacle_along_path_m_ = minimum_along;
      min_obstacle_along_command_m_ = minimum_command_along;
      closest_path_obstacle_forward_m_ = closest_obstacle_forward;
      closest_path_obstacle_right_edge_m_ = closest_obstacle_right_edge;
      closest_path_obstacle_left_edge_m_ = closest_obstacle_left_edge;
      last_relevant_cloud_ = t;
    }
  }

  void controlTick()
  {
    const auto t = now();
    geometry_msgs::msg::Twist nav{};
    geometry_msgs::msg::Twist advisory{};
    bool nav_fresh = false;
    bool advisory_fresh = false;
    bool camera_ok = false;
    bool camera_health_ok = true;
    bool emergency = false;
    bool obstacle_stream_fresh = false;
    bool lane_fresh = false;
    bool lane_valid = false;
    bool lane_critical = false;
    bool lane_control_fresh = false;
    bool lane_recenter_blocked = false;
    double perception_latency_sec = perception_latency_fallback_sec_;
    bool perception_latency_fresh = false;
    std::string lane_state = "UNKNOWN";
    std::string lane_control_decision = "UNKNOWN";
    PathSnapshot path;
    double min_obstacle_along = std::numeric_limits<double>::infinity();
    double min_obstacle_command_along = std::numeric_limits<double>::infinity();
    size_t candidates = 0U;
    size_t relevant = 0U;
    size_t planning_relevant = 0U;
    DrivableEnvelope drivable;
    double closest_obstacle_forward = std::numeric_limits<double>::infinity();
    double closest_obstacle_right_edge = std::numeric_limits<double>::infinity();
    double closest_obstacle_left_edge = -std::numeric_limits<double>::infinity();

    {
      std::lock_guard<std::mutex> lock(mutex_);
      nav_fresh = nav_cmd_.valid && fresh(nav_cmd_.received, nav_cmd_timeout_sec_, t);
      if (nav_fresh) nav = nav_cmd_.msg;
      advisory_fresh = advisory_cmd_.valid && fresh(advisory_cmd_.received, advisory_timeout_sec_, t);
      if (advisory_fresh) advisory = advisory_cmd_.msg;
      // camera_connected adalah state transient-local, bukan heartbeat. Jangan
      // menutup gate hanya karena nilai TRUE tidak dipublish ulang tiap detik.
      camera_ok = camera_connected_seen_ && camera_connected_;
      camera_health_ok = !require_camera_health_ ||
        (camera_health_seen_ && camera_healthy_);
      emergency = emergency_seen_ && perception_emergency_stop_;
      obstacle_stream_fresh = obstacle_stream_seen_ && fresh(last_obstacle_stream_, obstacle_timeout_sec_, t);
      perception_latency_fresh = perception_latency_seen_ &&
        fresh(perception_latency_received_, perception_performance_timeout_sec_, t);
      if (perception_latency_fresh) perception_latency_sec = perception_latency_sec_;
      lane_fresh = lane_seen_ && fresh(lane_received_, lane_timeout_sec_, t);
      lane_valid = lane_valid_;
      lane_critical = lane_critical_;
      lane_state = lane_state_;
      lane_control_fresh = lane_control_seen_ && fresh(lane_control_received_, lane_timeout_sec_, t);
      if (lane_control_fresh) {
        lane_recenter_blocked = lane_recenter_blocked_;
        lane_control_decision = lane_control_decision_;
      }
      path = buildPathSnapshotUnlocked(t);
      min_obstacle_along = min_obstacle_along_path_m_;
      min_obstacle_command_along = min_obstacle_along_command_m_;
      candidates = candidate_point_count_;
      relevant = relevant_point_count_;
      planning_relevant = planning_relevant_point_count_;
      drivable = drivable_envelope_;
      closest_obstacle_forward = closest_path_obstacle_forward_m_;
      closest_obstacle_right_edge = closest_path_obstacle_right_edge_m_;
      closest_obstacle_left_edge = closest_path_obstacle_left_edge_m_;
    }

    const bool drivable_fresh = drivable.valid && fresh(drivable.received, drivable_timeout_sec_, t);
    const FreeCorridor free_corridor = drivable_fresh ? evaluateFreeCorridor(
      drivable, closest_obstacle_forward, closest_obstacle_right_edge, closest_obstacle_left_edge,
      vehicle_width_m_, avoidance_side_margin_m_, avoidance_boundary_lookup_tolerance_m_) : FreeCorridor{};

    geometry_msgs::msg::Twist output = nav;
    std::string decision = "NAV2_PASS";
    double speed_scale = 1.0;
    const bool moving_request = nav_fresh && std::abs(nav.linear.x) > 1.0e-4;
    const double forward_speed = nav_fresh ? std::max(0.0, nav.linear.x) : 0.0;
    const double reaction_distance = forward_speed * (perception_latency_sec + latency_safety_margin_sec_);
    const double braking_distance = forward_speed * forward_speed / (2.0 * braking_deceleration_mps2_);
    const double required_stop_distance = std::clamp(
      reaction_distance + braking_distance + latency_distance_margin_m_, 0.0, max_dynamic_stop_distance_m_);
    const double effective_hard_stop_m = latency_compensation_enabled_ ?
      std::max(hard_stop_path_distance_m_, required_stop_distance) : hard_stop_path_distance_m_;
    const double effective_immediate_stop_m = latency_compensation_enabled_ ?
      std::max(immediate_command_hard_stop_m_, required_stop_distance) : immediate_command_hard_stop_m_;
    const double effective_slow_distance_m = std::max(slow_path_distance_m_, effective_hard_stop_m + 0.60);

    if (!nav_fresh) {
      decision = "NAV_CMD_STALE_STOP";
      output = geometry_msgs::msg::Twist{};
    } else if (emergency) {
      decision = "PERCEPTION_EMERGENCY_STOP";
      output = geometry_msgs::msg::Twist{};
    } else if (require_camera_connected_ && !camera_ok) {
      decision = "CAMERA_DISCONNECTED_STOP";
      output = geometry_msgs::msg::Twist{};
    } else if (!camera_health_ok) {
      decision = "CAMERA_UNHEALTHY_STOP";
      output = geometry_msgs::msg::Twist{};
    } else if (metric_obstacle_safety_enabled_ && moving_request && require_plan_when_moving_ && !path.valid) {
      decision = "PLAN_UNAVAILABLE_STOP";
      output = geometry_msgs::msg::Twist{};
    } else if (metric_obstacle_safety_enabled_ && moving_request && require_obstacle_stream_when_moving_ && !obstacle_stream_fresh) {
      decision = "OBSTACLE_STREAM_STALE_STOP";
      output = geometry_msgs::msg::Twist{};
    } else if (lane_safety_enabled_ && lane_control_fresh && lane_recenter_blocked) {
      decision = "LANE_RECENTER_BLOCKED_STOP";
      output = geometry_msgs::msg::Twist{};
    } else if (metric_obstacle_safety_enabled_ && std::isfinite(min_obstacle_command_along) &&
               min_obstacle_command_along <= effective_immediate_stop_m) {
      // Emergency geometric veto: current MPPI command masih membawa footprint
      // langsung menuju obstacle yang sudah sangat dekat.
      decision = "IMMEDIATE_COMMAND_HARD_STOP";
      output = geometry_msgs::msg::Twist{};
    } else {
      const bool path_obstacle_near = metric_obstacle_safety_enabled_ && std::isfinite(min_obstacle_along) &&
        min_obstacle_along < effective_slow_distance_m;
      const bool path_obstacle_close = metric_obstacle_safety_enabled_ && std::isfinite(min_obstacle_along) &&
        min_obstacle_along <= effective_hard_stop_m;
      const bool command_still_hits_close = metric_obstacle_safety_enabled_ && std::isfinite(min_obstacle_command_along) &&
        min_obstacle_command_along <= effective_hard_stop_m;
      const bool any_free_side = free_corridor.left_free || free_corridor.right_free;

      if (!metric_obstacle_safety_enabled_) {
        decision = "GUARD_ONLY_PASS";
      } else if (path_obstacle_near) {
        const double span = std::max(0.05, effective_slow_distance_m - effective_hard_stop_m);
        const double ratio = std::clamp(
          (min_obstacle_along - effective_hard_stop_m) / span, 0.0, 1.0);
        speed_scale = minimum_slow_speed_scale_ + (1.0 - minimum_slow_speed_scale_) * ratio;
        output.linear.x *= speed_scale;
        if (output.linear.x > 0.0 && avoidance_speed_mps_ > 0.0) {
          output.linear.x = std::min(output.linear.x, avoidance_speed_mps_);
        }
        if (!drivable_fresh && require_drivable_space_for_avoidance_ && path_obstacle_close) {
          decision = "DRIVABLE_SPACE_UNKNOWN_STOP";
          output = geometry_msgs::msg::Twist{};
        } else if (path_obstacle_close && !any_free_side) {
          decision = "NO_SAFE_CORRIDOR_STOP";
          output = geometry_msgs::msg::Twist{};
        } else if (path_obstacle_close && command_still_hits_close) {
          // Ruang mungkin tersedia, tetapi MPPI belum menghasilkan command yang
          // benar-benar menjauh dari obstacle. Jangan menerobos sambil menunggu.
          decision = "AVOIDANCE_NOT_ENGAGED_STOP";
          output = geometry_msgs::msg::Twist{};
        } else if (any_free_side) {
          const bool turning_left = output.angular.z > 0.03;
          const bool turning_right = output.angular.z < -0.03;
          if (turning_left && free_corridor.left_free) decision = "AVOID_LEFT";
          else if (turning_right && free_corridor.right_free) decision = "AVOID_RIGHT";
          else if (free_corridor.left_free && free_corridor.right_free) decision = "SEARCH_AVOIDANCE_BOTH";
          else decision = free_corridor.left_free ? "SEARCH_AVOIDANCE_LEFT" : "SEARCH_AVOIDANCE_RIGHT";
        } else {
          decision = "PATH_OBSTACLE_SLOW_SEARCH";
        }
      }

      const bool recenter = lane_safety_enabled_ && !path_obstacle_near && lane_fresh && lane_valid &&
        (lane_state == "RECENTER_LEFT" || lane_state == "RECENTER_RIGHT");
      if (recenter && advisory_fresh) {
        const double speed_cap = lane_critical ? critical_lane_speed_mps_ : warning_lane_speed_mps_;
        if (output.linear.x > 0.0) output.linear.x = std::min(output.linear.x, speed_cap);
        output.angular.z = (1.0 - lane_advisory_blend_) * output.angular.z +
          lane_advisory_blend_ * advisory.angular.z;
        decision = lane_critical ? "LANE_CRITICAL_RECENTER" : "LANE_RECENTER";
      } else if (lane_safety_enabled_ && stop_on_lane_lost_ && lane_fresh &&
                 (!lane_valid || lane_state == "LANE_LOST")) {
        decision = "LANE_LOST_STOP";
        output = geometry_msgs::msg::Twist{};
      }
    }

    if (!finiteTwist(output)) {
      decision = "NONFINITE_COMMAND_STOP";
      output = geometry_msgs::msg::Twist{};
    }
    // Any speed reduction or lane-advisory blend must preserve Ackermann
    // feasibility. Otherwise reducing v while leaving w unchanged tightens the
    // commanded radius and can silently violate the physical steering limit.
    const double curvature_yaw_cap =
      std::abs(output.linear.x) / std::max(0.10, minimum_turning_radius_m_);
    const double yaw_cap = std::min(maximum_yaw_rate_rps_, curvature_yaw_cap);
    output.angular.z = std::clamp(output.angular.z, -yaw_cap, yaw_cap);
    if (std::abs(output.linear.x) <= 1.0e-4) output.angular.z = 0.0;
    integrated_cmd_pub_->publish(output);

    std_msgs::msg::String state;
    std::ostringstream ss;
    ss << std::boolalpha << std::fixed << std::setprecision(3)
       << "decision=" << decision
       << ";nav_fresh=" << nav_fresh
       << ";path_valid=" << path.valid
       << ";path_source=" << (path.source.empty() ? "none" : path.source)
       << ";camera_connected=" << camera_ok
       << ";camera_health_ok=" << camera_health_ok
       << ";emergency=" << emergency
       << ";metric_obstacle_safety=" << metric_obstacle_safety_enabled_
       << ";perception_latency_fresh=" << perception_latency_fresh
       << ";perception_latency_sec=" << perception_latency_sec
       << ";required_stop_m=" << required_stop_distance
       << ";effective_hard_stop_m=" << effective_hard_stop_m
       << ";effective_slow_m=" << effective_slow_distance_m
       << ";obstacle_stream_fresh=" << obstacle_stream_fresh
       << ";candidate_points=" << candidates
       << ";path_relevant_points=" << relevant
       << ";planning_relevant_points=" << planning_relevant
       << ";drivable_fresh=" << drivable_fresh
       << ";free_left=" << free_corridor.left_free
       << ";free_right=" << free_corridor.right_free
       << ";free_left_m=" << free_corridor.left_available_m
       << ";free_right_m=" << free_corridor.right_available_m
       << ";required_corridor_m=" << free_corridor.required_width_m
       << ";avoidance_preferred=" << free_corridor.preferred
       << ";min_obstacle_path_m=";
    if (std::isfinite(min_obstacle_along)) ss << min_obstacle_along;
    else ss << "NA";
    ss << ";min_obstacle_command_m=";
    if (std::isfinite(min_obstacle_command_along)) ss << min_obstacle_command_along;
    else ss << "NA";
    ss << ";lane_state=" << lane_state
       << ";lane_safety_enabled=" << lane_safety_enabled_
       << ";lane_control_decision=" << lane_control_decision
       << ";lane_recenter_blocked=" << lane_recenter_blocked
       << ";lane_fresh=" << lane_fresh
       << ";lane_valid=" << lane_valid
       << ";lane_critical=" << lane_critical
       << ";speed_scale=" << speed_scale
       << ";nav_v=" << nav.linear.x
       << ";nav_w=" << nav.angular.z
       << ";out_v=" << output.linear.x
       << ";out_w=" << output.angular.z;
    state.data = ss.str();
    state_pub_->publish(state);

    const bool decision_changed = decision != last_logged_decision_;
    const bool log_due = last_status_log_time_.nanoseconds() == 0 ||
      (t - last_status_log_time_).seconds() >= status_log_period_sec_;
    if (decision_changed || log_due) {
      const std::string path_obstacle_text = std::isfinite(min_obstacle_along) ?
        std::to_string(min_obstacle_along) : "NA";
      const std::string command_obstacle_text = std::isfinite(min_obstacle_command_along) ?
        std::to_string(min_obstacle_command_along) : "NA";
      RCLCPP_INFO(
        get_logger(),
        "SAFETY decision=%s path=%s obs(path/cmd)=%s/%s free(L/R)=%.2f/%.2f pref=%s nav=(%.2f,%.2f) out=(%.2f,%.2f)",
        decision.c_str(), path.source.empty() ? "none" : path.source.c_str(),
        path_obstacle_text.c_str(), command_obstacle_text.c_str(),
        free_corridor.left_available_m, free_corridor.right_available_m,
        free_corridor.preferred.c_str(), nav.linear.x, nav.angular.z, output.linear.x, output.angular.z);
      last_logged_decision_ = decision;
      last_status_log_time_ = t;
    }

    if (avoidance_hint_pub_) {
      std_msgs::msg::String hint;
      std::ostringstream hs;
      hs << std::boolalpha << std::fixed << std::setprecision(3)
         << "preferred=" << free_corridor.preferred
         << ";left_free=" << free_corridor.left_free
         << ";right_free=" << free_corridor.right_free
         << ";left_available_m=" << free_corridor.left_available_m
         << ";right_available_m=" << free_corridor.right_available_m
         << ";required_m=" << free_corridor.required_width_m
         << ";decision=" << decision;
      hint.data = hs.str();
      avoidance_hint_pub_->publish(hint);
    }
  }

  std::mutex mutex_;

  std::string nav_cmd_topic_;
  std::string perception_advisory_topic_;
  std::string output_cmd_topic_;
  std::string global_plan_topic_;
  std::string local_plan_topic_;
  std::string map_pose_topic_;
  std::string odom_topic_;
  std::string candidate_obstacle_topic_;
  std::string path_relevant_obstacle_topic_;
  std::string planning_relevant_obstacle_topic_;
  std::string drivable_boundary_topic_;
  std::string avoidance_hint_topic_;
  std::string lane_state_topic_;
  std::string lane_control_state_topic_;
  std::string camera_connected_topic_;
  std::string camera_health_topic_;
  std::string emergency_stop_topic_;
  std::string perception_performance_topic_;
  std::string state_topic_;

  double control_rate_hz_{20.0};
  double status_log_period_sec_{1.0};
  double nav_cmd_timeout_sec_{0.60};
  double advisory_timeout_sec_{0.50};
  double plan_timeout_sec_{3.0};
  double pose_timeout_sec_{0.60};
  double obstacle_timeout_sec_{0.90};
  double lane_timeout_sec_{0.75};
  double drivable_timeout_sec_{0.75};
  double perception_performance_timeout_sec_{2.0};
  bool metric_obstacle_safety_enabled_{false};
  bool latency_compensation_enabled_{true};
  double perception_latency_fallback_sec_{1.0};
  double latency_safety_margin_sec_{0.15};
  double latency_distance_margin_m_{0.25};
  double braking_deceleration_mps2_{1.0};
  double max_dynamic_stop_distance_m_{2.50};
  double vehicle_width_m_{0.60};
  double trajectory_lateral_margin_m_{0.25};
  double path_corridor_half_width_m_{0.55};
  double planning_lateral_margin_m_{0.65};
  double planning_corridor_half_width_m_{0.95};
  double planning_min_forward_m_{0.15};
  double planning_max_abs_lateral_m_{2.6};
  double path_horizon_m_{4.0};
  double command_collision_horizon_m_{0.90};
  double command_collision_sample_step_m_{0.05};
  double command_collision_lateral_margin_m_{0.20};
  double command_collision_half_width_m_{0.50};
  double immediate_command_hard_stop_m_{0.65};
  double hard_stop_path_distance_m_{0.90};
  double slow_path_distance_m_{2.20};
  double minimum_slow_speed_scale_{0.35};
  double avoidance_side_margin_m_{0.25};
  double avoidance_boundary_lookup_tolerance_m_{0.28};
  double avoidance_speed_mps_{0.18};
  bool require_drivable_space_for_avoidance_{true};
  double critical_lane_speed_mps_{0.10};
  double warning_lane_speed_mps_{0.20};
  double maximum_yaw_rate_rps_{0.825470725};
  double minimum_turning_radius_m_{1.712159378317};
  double lane_advisory_blend_{1.0};
  bool lane_safety_enabled_{false};

  bool require_plan_when_moving_{true};
  bool require_camera_connected_{true};
  bool require_camera_health_{true};
  bool require_obstacle_stream_when_moving_{true};
  bool stop_on_lane_lost_{false};
  bool use_local_plan_when_available_{true};

  TimedTwist nav_cmd_;
  TimedTwist advisory_cmd_;
  TimedPath global_plan_;
  TimedPath local_plan_;
  Pose2D map_pose_;
  Pose2D odom_pose_;
  PathSnapshot latest_path_;
  DrivableEnvelope drivable_envelope_;

  bool lane_seen_{false};
  bool lane_valid_{false};
  bool lane_critical_{false};
  std::string lane_state_{"UNKNOWN"};
  rclcpp::Time lane_received_{0, 0, RCL_ROS_TIME};
  bool lane_control_seen_{false};
  bool lane_recenter_blocked_{false};
  bool lane_control_critical_{false};
  std::string lane_control_decision_{"UNKNOWN"};
  rclcpp::Time lane_control_received_{0, 0, RCL_ROS_TIME};

  bool camera_connected_seen_{false};
  bool camera_connected_{false};
  rclcpp::Time camera_connected_received_{0, 0, RCL_ROS_TIME};
  bool camera_health_seen_{false};
  bool camera_healthy_{false};
  rclcpp::Time camera_health_received_{0, 0, RCL_ROS_TIME};
  bool emergency_seen_{false};
  bool perception_emergency_stop_{false};
  rclcpp::Time emergency_received_{0, 0, RCL_ROS_TIME};
  bool perception_latency_seen_{false};
  double perception_latency_sec_{1.0};
  rclcpp::Time perception_latency_received_{0, 0, RCL_ROS_TIME};

  bool obstacle_stream_seen_{false};
  rclcpp::Time last_obstacle_stream_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_relevant_cloud_{0, 0, RCL_ROS_TIME};
  size_t candidate_point_count_{0U};
  size_t relevant_point_count_{0U};
  size_t planning_relevant_point_count_{0U};
  double min_obstacle_along_path_m_{std::numeric_limits<double>::infinity()};
  double min_obstacle_along_command_m_{std::numeric_limits<double>::infinity()};
  double closest_path_obstacle_forward_m_{std::numeric_limits<double>::infinity()};
  double closest_path_obstacle_right_edge_m_{std::numeric_limits<double>::infinity()};
  double closest_path_obstacle_left_edge_m_{-std::numeric_limits<double>::infinity()};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr advisory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_plan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr map_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lane_control_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_connected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_health_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr performance_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr candidate_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr drivable_boundary_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr integrated_cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr relevant_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr planning_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr avoidance_hint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string last_logged_decision_;
  rclcpp::Time last_status_log_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation::TrajectorySafetySupervisor>());
  rclcpp::shutdown();
  return 0;
}
