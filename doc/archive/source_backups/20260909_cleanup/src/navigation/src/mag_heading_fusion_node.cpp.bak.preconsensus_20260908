#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

bool finiteQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
         (q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w) > 1.0e-8;
}

void quaternionToRollPitch(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch) {
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);
  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  pitch = std::abs(sinp) >= 1.0 ? std::copysign(0.5 * kPi, sinp) : std::asin(sinp);
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}
}

class MagHeadingFusionNode final : public rclcpp::Node {
public:
  MagHeadingFusionNode() : Node("mag_heading_fusion") {
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("imu_mag_topic", "/imu/mag");
    declare_parameter<std::string>("neo3_mag_topic", "/neo3/mag");
    declare_parameter<std::string>("map_yaw_topic", "/localization/map_yaw_from_enu");
    declare_parameter<std::string>("imu_heading_topic", "/imu/mag_heading_fusion");
    declare_parameter<std::string>("neo3_heading_topic", "/neo3/mag_heading_fusion");
    declare_parameter<std::string>("output_frame", "map");
    declare_parameter<double>("tilt_timeout_sec", 0.75);
    declare_parameter<double>("max_tilt_rad", 0.7853981634);
    declare_parameter<double>("magnetic_declination_rad", 0.0);
    declare_parameter<double>("min_field_norm_ut", 15.0);
    declare_parameter<double>("max_field_norm_ut", 100.0);
    declare_parameter<double>("imu_mag_yaw_sign", -1.0);
    declare_parameter<double>("imu_mag_yaw_offset_rad", 1.5451826276);
    declare_parameter<double>("neo3_mag_yaw_sign", -1.0);
    declare_parameter<double>("neo3_mag_yaw_offset_rad", 1.5707963268);
    declare_parameter<double>("imu_heading_variance_rad2", 0.08);
    declare_parameter<double>("neo3_heading_variance_rad2", 0.06);
    declare_parameter<double>("max_heading_rate_rps", 3.0);
    declare_parameter<double>("rate_gate_margin_rad", 0.35);
    declare_parameter<bool>("enable_imu_mag_heading", true);
    declare_parameter<bool>("enable_neo3_mag_heading", true);

    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_mag_topic_ = get_parameter("imu_mag_topic").as_string();
    neo3_mag_topic_ = get_parameter("neo3_mag_topic").as_string();
    map_yaw_topic_ = get_parameter("map_yaw_topic").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    tilt_timeout_sec_ = std::clamp(get_parameter("tilt_timeout_sec").as_double(), 0.1, 5.0);
    max_tilt_rad_ = std::clamp(get_parameter("max_tilt_rad").as_double(), 0.1, 1.4);
    declination_rad_ = get_parameter("magnetic_declination_rad").as_double();
    min_norm_ut_ = std::max(1.0, get_parameter("min_field_norm_ut").as_double());
    max_norm_ut_ = std::max(min_norm_ut_ + 1.0, get_parameter("max_field_norm_ut").as_double());
    imu_sign_ = get_parameter("imu_mag_yaw_sign").as_double() >= 0.0 ? 1.0 : -1.0;
    imu_offset_ = get_parameter("imu_mag_yaw_offset_rad").as_double();
    neo_sign_ = get_parameter("neo3_mag_yaw_sign").as_double() >= 0.0 ? 1.0 : -1.0;
    neo_offset_ = get_parameter("neo3_mag_yaw_offset_rad").as_double();
    imu_variance_ = std::clamp(get_parameter("imu_heading_variance_rad2").as_double(), 1.0e-4, 10.0);
    neo_variance_ = std::clamp(get_parameter("neo3_heading_variance_rad2").as_double(), 1.0e-4, 10.0);
    max_heading_rate_rps_ = std::clamp(get_parameter("max_heading_rate_rps").as_double(), 0.2, 10.0);
    rate_gate_margin_rad_ = std::clamp(get_parameter("rate_gate_margin_rad").as_double(), 0.05, kPi);
    enable_imu_ = get_parameter("enable_imu_mag_heading").as_bool();
    enable_neo_ = get_parameter("enable_neo3_mag_heading").as_bool();

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos, std::bind(&MagHeadingFusionNode::onImu, this, std::placeholders::_1));
    imu_mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      imu_mag_topic_, sensor_qos,
      [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) { onMag(*msg, Source::IMU); });
    neo_mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      neo3_mag_topic_, sensor_qos,
      [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) { onMag(*msg, Source::NEO3); });
    map_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
      map_yaw_topic_, state_qos, [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        if (std::isfinite(msg->data)) {
          map_yaw_from_enu_ = normalizeAngle(msg->data);
          have_map_yaw_ = true;
          last_map_yaw_time_ = now();
        }
      });

    imu_heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("imu_heading_topic").as_string(), sensor_qos);
    neo_heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("neo3_heading_topic").as_string(), sensor_qos);
    imu_valid_pub_ = create_publisher<std_msgs::msg::Bool>("/imu/mag_heading_valid", state_qos);
    neo_valid_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3/mag_heading_valid", state_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("/system/magnetic_heading_status", state_qos);

    status_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&MagHeadingFusionNode::publishStatus, this));
    RCLCPP_INFO(get_logger(), "Magnetic heading fusion: IMU=%s NEO3=%s norm=[%.1f, %.1f]uT",
                enable_imu_ ? "ON" : "OFF", enable_neo_ ? "ON" : "OFF", min_norm_ut_, max_norm_ut_);
  }

private:
  enum class Source { IMU, NEO3 };
  struct HeadingState {
    bool valid{false};
    double heading_rad{0.0};
    double norm_ut{0.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint64_t accepted{0};
    uint64_t rejected{0};
    std::string reject_reason{"no_sample"};
  };

  void onImu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    if (!finiteQuaternion(msg->orientation)) return;
    double roll = 0.0, pitch = 0.0;
    quaternionToRollPitch(msg->orientation, roll, pitch);
    if (!std::isfinite(roll) || !std::isfinite(pitch)) return;
    roll_rad_ = roll;
    pitch_rad_ = pitch;
    last_tilt_time_ = now();
    have_tilt_ = true;
  }

  void setValid(Source source, bool valid) {
    auto &state = source == Source::IMU ? imu_state_ : neo_state_;
    if (state.valid == valid) return;
    state.valid = valid;
    std_msgs::msg::Bool b; b.data = valid;
    (source == Source::IMU ? imu_valid_pub_ : neo_valid_pub_)->publish(b);
  }

  bool temporalGate(HeadingState &state, double heading, const rclcpp::Time &stamp) {
    if (state.stamp.nanoseconds() == 0) return true;
    const double dt = (stamp - state.stamp).seconds();
    if (dt <= 0.0 || dt > 2.0) return true;
    const double delta = std::abs(normalizeAngle(heading - state.heading_rad));
    return delta <= max_heading_rate_rps_ * dt + rate_gate_margin_rad_;
  }

  void onMag(const sensor_msgs::msg::MagneticField &msg, Source source) {
    if ((source == Source::IMU && !enable_imu_) || (source == Source::NEO3 && !enable_neo_)) return;
    HeadingState &state = source == Source::IMU ? imu_state_ : neo_state_;
    const auto reject = [&](const char *reason) {
      ++state.rejected;
      state.reject_reason = reason;
      setValid(source, false);
    };

    const auto t = now();
    if (!have_map_yaw_) { reject("map_yaw_unavailable"); return; }
    if (!have_tilt_ || (t - last_tilt_time_).seconds() < 0.0 ||
        (t - last_tilt_time_).seconds() > tilt_timeout_sec_) { reject("tilt_stale"); return; }
    if (std::abs(roll_rad_) > max_tilt_rad_ || std::abs(pitch_rad_) > max_tilt_rad_) {
      reject("tilt_out_of_range"); return;
    }

    const double mx = msg.magnetic_field.x * 1.0e6;
    const double my = msg.magnetic_field.y * 1.0e6;
    const double mz = msg.magnetic_field.z * 1.0e6;
    if (!std::isfinite(mx) || !std::isfinite(my) || !std::isfinite(mz)) { reject("non_finite_field"); return; }
    const double norm = std::sqrt(mx*mx + my*my + mz*mz);
    state.norm_ut = norm;
    if (!std::isfinite(norm) || norm < min_norm_ut_ || norm > max_norm_ut_) { reject("field_norm_gate"); return; }

    const double cr = std::cos(roll_rad_), sr = std::sin(roll_rad_);
    const double cp = std::cos(pitch_rad_), sp = std::sin(pitch_rad_);
    const double xh = mx * cp + mz * sp;
    const double yh = mx * sr * sp + my * cr - mz * sr * cp;
    if (std::hypot(xh, yh) < 1.0) { reject("horizontal_field_too_small"); return; }

    const double raw_north_in_body = std::atan2(yh, xh);
    const double sign = source == Source::IMU ? imu_sign_ : neo_sign_;
    const double offset = source == Source::IMU ? imu_offset_ : neo_offset_;
    const double yaw_enu = normalizeAngle(sign * raw_north_in_body + offset - declination_rad_);
    const double yaw_map = normalizeAngle(yaw_enu + map_yaw_from_enu_);
    const rclcpp::Time stamp = msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0 ? t : rclcpp::Time(msg.header.stamp);
    if (!temporalGate(state, yaw_map, stamp)) { reject("heading_rate_gate"); return; }

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = output_frame_;
    pose.pose.pose.orientation = yawQuaternion(yaw_map);
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0] = 1.0e6;
    pose.pose.covariance[7] = 1.0e6;
    pose.pose.covariance[14] = 1.0e6;
    pose.pose.covariance[21] = 1.0e6;
    pose.pose.covariance[28] = 1.0e6;
    pose.pose.covariance[35] = source == Source::IMU ? imu_variance_ : neo_variance_;
    (source == Source::IMU ? imu_heading_pub_ : neo_heading_pub_)->publish(pose);

    state.heading_rad = yaw_map;
    state.stamp = stamp;
    state.reject_reason.clear();
    ++state.accepted;
    setValid(source, true);
  }

  void publishStatus() {
    std_msgs::msg::String msg;
    const auto t = now();
    const double tilt_age = have_tilt_ ? (t - last_tilt_time_).seconds() : 999.0;
    const double map_age = have_map_yaw_ ? (t - last_map_yaw_time_).seconds() : 999.0;
    msg.data = std::string("imu_valid=") + (imu_state_.valid ? "true" : "false") +
      ";neo3_valid=" + (neo_state_.valid ? "true" : "false") +
      ";imu_norm_ut=" + std::to_string(imu_state_.norm_ut) +
      ";neo3_norm_ut=" + std::to_string(neo_state_.norm_ut) +
      ";imu_reject=" + imu_state_.reject_reason +
      ";neo3_reject=" + neo_state_.reject_reason +
      ";tilt_age_sec=" + std::to_string(tilt_age) +
      ";map_yaw_age_sec=" + std::to_string(map_age);
    status_pub_->publish(msg);
  }

  std::string imu_topic_, imu_mag_topic_, neo3_mag_topic_, map_yaw_topic_, output_frame_;
  double tilt_timeout_sec_{0.75}, max_tilt_rad_{0.7853981634}, declination_rad_{0.0};
  double min_norm_ut_{15.0}, max_norm_ut_{100.0};
  double imu_sign_{-1.0}, imu_offset_{1.5451826276}, neo_sign_{-1.0}, neo_offset_{1.5707963268};
  double imu_variance_{0.08}, neo_variance_{0.06};
  double max_heading_rate_rps_{3.0}, rate_gate_margin_rad_{0.35};
  bool enable_imu_{true}, enable_neo_{true};
  bool have_tilt_{false}, have_map_yaw_{false};
  double roll_rad_{0.0}, pitch_rad_{0.0}, map_yaw_from_enu_{0.0};
  rclcpp::Time last_tilt_time_{0, 0, RCL_ROS_TIME}, last_map_yaw_time_{0, 0, RCL_ROS_TIME};
  HeadingState imu_state_, neo_state_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_sub_, neo_mag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr map_yaw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr imu_heading_pub_, neo_heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_valid_pub_, neo_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagHeadingFusionNode>());
  rclcpp::shutdown();
  return 0;
}
