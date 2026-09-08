#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
    declare_parameter<std::string>("imu_raw_mag_topic", "/imu/mag_raw_lsb");
    declare_parameter<std::string>("neo3_mag_topic", "/neo3/mag");
    declare_parameter<std::string>("map_yaw_topic", "/localization/map_yaw_from_enu");
    declare_parameter<std::string>("imu_heading_topic", "/imu/mag_heading_fusion");
    declare_parameter<std::string>("neo3_heading_topic", "/neo3/mag_heading_fusion");
    declare_parameter<std::string>("imu_inertial_heading_topic", "/imu/inertial_heading");
    declare_parameter<std::string>("validated_heading_topic", "/heading/validated_fusion");
    declare_parameter<std::string>("output_frame", "map");
    declare_parameter<double>("tilt_timeout_sec", 0.75);
    declare_parameter<double>("max_tilt_rad", 0.7853981634);
    declare_parameter<double>("magnetic_declination_rad", 0.0);
    declare_parameter<double>("min_field_norm_ut", 15.0);
    declare_parameter<double>("max_field_norm_ut", 100.0);
    declare_parameter<double>("imu_mag_yaw_sign", -1.0);
    declare_parameter<double>("imu_mag_yaw_offset_rad", 1.5451826276);
    declare_parameter<bool>("imu_planar_calibration_enabled", false);
    declare_parameter<std::vector<double>>("imu_mag_bias_xy_lsb", std::vector<double>{0.0, 0.0});
    declare_parameter<std::vector<double>>("imu_mag_matrix_xy_per_lsb", std::vector<double>{1.0, 0.0, 0.0, 1.0});
    declare_parameter<bool>("imu_heading_lut_enabled", false);
    declare_parameter<std::vector<double>>("imu_heading_lut_input_rad", std::vector<double>{});
    declare_parameter<std::vector<double>>("imu_heading_lut_correction_rad", std::vector<double>{});
    declare_parameter<double>("imu_corrected_norm_min", 0.70);
    declare_parameter<double>("imu_corrected_norm_max", 1.30);
    declare_parameter<double>("imu_planar_max_tilt_rad", 0.2617993878);
    declare_parameter<double>("neo3_mag_yaw_sign", -1.0);
    declare_parameter<double>("neo3_mag_yaw_offset_rad", 1.5707963268);
    declare_parameter<bool>("neo3_planar_calibration_enabled", false);
    declare_parameter<std::vector<double>>("neo3_mag_bias_xy_ut", std::vector<double>{0.0, 0.0});
    declare_parameter<std::vector<double>>("neo3_mag_matrix_xy", std::vector<double>{1.0, 0.0, 0.0, 1.0});
    declare_parameter<bool>("neo3_heading_lut_enabled", false);
    declare_parameter<std::vector<double>>("neo3_heading_lut_input_rad", std::vector<double>{});
    declare_parameter<std::vector<double>>("neo3_heading_lut_correction_rad", std::vector<double>{});
    declare_parameter<double>("imu_heading_variance_rad2", 0.08);
    declare_parameter<double>("neo3_heading_variance_rad2", 0.06);
    declare_parameter<double>("max_heading_rate_rps", 3.0);
    declare_parameter<double>("rate_gate_margin_rad", 0.35);
    declare_parameter<double>("consensus_max_error_rad", 0.0872664626);
    declare_parameter<double>("consensus_hold_sec", 2.0);
    declare_parameter<double>("consensus_timeout_sec", 0.5);
    declare_parameter<double>("validated_heading_variance_rad2", 0.01);
    declare_parameter<bool>("enable_imu_mag_heading", true);
    declare_parameter<bool>("enable_neo3_mag_heading", true);

    imu_topic_ = get_parameter("imu_topic").as_string();
    imu_mag_topic_ = get_parameter("imu_mag_topic").as_string();
    imu_raw_mag_topic_ = get_parameter("imu_raw_mag_topic").as_string();
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
    imu_planar_calibration_enabled_ = get_parameter("imu_planar_calibration_enabled").as_bool();
    const auto imu_bias = get_parameter("imu_mag_bias_xy_lsb").as_double_array();
    const auto imu_matrix = get_parameter("imu_mag_matrix_xy_per_lsb").as_double_array();
    if (imu_bias.size() == 2) { imu_bias_x_lsb_ = imu_bias[0]; imu_bias_y_lsb_ = imu_bias[1]; }
    else { RCLCPP_WARN(get_logger(), "Yahboom bias XY invalid size=%zu; calibration disabled", imu_bias.size()); imu_planar_calibration_enabled_ = false; }
    if (imu_matrix.size() == 4) {
      imu_m00_ = imu_matrix[0]; imu_m01_ = imu_matrix[1]; imu_m10_ = imu_matrix[2]; imu_m11_ = imu_matrix[3];
      const double det = imu_m00_ * imu_m11_ - imu_m01_ * imu_m10_;
      if (!std::isfinite(det) || std::abs(det) < 1.0e-12) { RCLCPP_WARN(get_logger(), "Yahboom XY matrix singular; calibration disabled"); imu_planar_calibration_enabled_ = false; }
    } else { RCLCPP_WARN(get_logger(), "Yahboom matrix XY invalid size=%zu; calibration disabled", imu_matrix.size()); imu_planar_calibration_enabled_ = false; }
    imu_heading_lut_enabled_ = get_parameter("imu_heading_lut_enabled").as_bool();
    const auto imu_lut_in = get_parameter("imu_heading_lut_input_rad").as_double_array();
    const auto imu_lut_corr = get_parameter("imu_heading_lut_correction_rad").as_double_array();
    if (imu_heading_lut_enabled_) {
      if (imu_lut_in.size() != imu_lut_corr.size() || imu_lut_in.size() < 4) { imu_heading_lut_enabled_ = false; }
      else {
        for (size_t i = 0; i < imu_lut_in.size(); ++i) if (std::isfinite(imu_lut_in[i]) && std::isfinite(imu_lut_corr[i])) imu_heading_lut_.emplace_back(normalizeAngle(imu_lut_in[i]), normalizeAngle(imu_lut_corr[i]));
        std::sort(imu_heading_lut_.begin(), imu_heading_lut_.end(), [](const auto &a, const auto &b){ return a.first < b.first; });
        if (imu_heading_lut_.size() < 4) imu_heading_lut_enabled_ = false;
      }
    }
    imu_corrected_norm_min_ = std::max(0.05, get_parameter("imu_corrected_norm_min").as_double());
    imu_corrected_norm_max_ = std::max(imu_corrected_norm_min_ + 0.05, get_parameter("imu_corrected_norm_max").as_double());
    imu_planar_max_tilt_rad_ = std::clamp(get_parameter("imu_planar_max_tilt_rad").as_double(), 0.05, 0.7);
    neo_sign_ = get_parameter("neo3_mag_yaw_sign").as_double() >= 0.0 ? 1.0 : -1.0;
    neo_offset_ = get_parameter("neo3_mag_yaw_offset_rad").as_double();
    neo_planar_calibration_enabled_ = get_parameter("neo3_planar_calibration_enabled").as_bool();
    const auto neo_bias = get_parameter("neo3_mag_bias_xy_ut").as_double_array();
    const auto neo_matrix = get_parameter("neo3_mag_matrix_xy").as_double_array();
    if (neo_bias.size() == 2) { neo_bias_x_ut_ = neo_bias[0]; neo_bias_y_ut_ = neo_bias[1]; }
    else { RCLCPP_WARN(get_logger(), "NEO3 bias XY invalid size=%zu; calibration disabled", neo_bias.size()); neo_planar_calibration_enabled_ = false; }
    if (neo_matrix.size() == 4) {
      neo_m00_ = neo_matrix[0]; neo_m01_ = neo_matrix[1]; neo_m10_ = neo_matrix[2]; neo_m11_ = neo_matrix[3];
      const double det = neo_m00_ * neo_m11_ - neo_m01_ * neo_m10_;
      if (!std::isfinite(det) || std::abs(det) < 1.0e-6) {
        RCLCPP_WARN(get_logger(), "NEO3 XY matrix singular; calibration disabled");
        neo_planar_calibration_enabled_ = false;
      }
    } else { RCLCPP_WARN(get_logger(), "NEO3 matrix XY invalid size=%zu; calibration disabled", neo_matrix.size()); neo_planar_calibration_enabled_ = false; }
    neo_heading_lut_enabled_ = get_parameter("neo3_heading_lut_enabled").as_bool();
    const auto lut_in = get_parameter("neo3_heading_lut_input_rad").as_double_array();
    const auto lut_corr = get_parameter("neo3_heading_lut_correction_rad").as_double_array();
    if (neo_heading_lut_enabled_) {
      if (lut_in.size() != lut_corr.size() || lut_in.size() < 4) {
        RCLCPP_WARN(get_logger(), "NEO3 heading LUT invalid sizes %zu/%zu; LUT disabled", lut_in.size(), lut_corr.size());
        neo_heading_lut_enabled_ = false;
      } else {
        for (size_t i = 0; i < lut_in.size(); ++i) {
          if (std::isfinite(lut_in[i]) && std::isfinite(lut_corr[i]))
            neo_heading_lut_.emplace_back(normalizeAngle(lut_in[i]), normalizeAngle(lut_corr[i]));
        }
        std::sort(neo_heading_lut_.begin(), neo_heading_lut_.end(),
          [](const auto &a, const auto &b) { return a.first < b.first; });
        if (neo_heading_lut_.size() < 4) neo_heading_lut_enabled_ = false;
      }
    }
    imu_variance_ = std::clamp(get_parameter("imu_heading_variance_rad2").as_double(), 1.0e-4, 10.0);
    neo_variance_ = std::clamp(get_parameter("neo3_heading_variance_rad2").as_double(), 1.0e-4, 10.0);
    max_heading_rate_rps_ = std::clamp(get_parameter("max_heading_rate_rps").as_double(), 0.2, 10.0);
    rate_gate_margin_rad_ = std::clamp(get_parameter("rate_gate_margin_rad").as_double(), 0.05, kPi);
    consensus_max_error_rad_ = std::clamp(get_parameter("consensus_max_error_rad").as_double(), 0.0087266463, 0.5235987756);
    consensus_hold_sec_ = std::clamp(get_parameter("consensus_hold_sec").as_double(), 0.2, 30.0);
    consensus_timeout_sec_ = std::clamp(get_parameter("consensus_timeout_sec").as_double(), 0.1, 2.0);
    validated_variance_ = std::clamp(get_parameter("validated_heading_variance_rad2").as_double(), 1.0e-4, 1.0);
    enable_imu_ = get_parameter("enable_imu_mag_heading").as_bool();
    enable_neo_ = get_parameter("enable_neo3_mag_heading").as_bool();

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos, std::bind(&MagHeadingFusionNode::onImu, this, std::placeholders::_1));
    imu_mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      imu_mag_topic_, sensor_qos,
      [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) { onMag(*msg, Source::IMU); });
    imu_raw_mag_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      imu_raw_mag_topic_, sensor_qos,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) { onImuRawMag(*msg); });
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
    inertial_heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("imu_inertial_heading_topic").as_string(), sensor_qos);
    validated_heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("validated_heading_topic").as_string(), sensor_qos);
    consensus_valid_pub_ = create_publisher<std_msgs::msg::Bool>("/heading/validated", state_qos);
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
    const auto t = now();
    const double wz = msg->angular_velocity.z;
    const double an = std::sqrt(msg->linear_acceleration.x*msg->linear_acceleration.x +
                                msg->linear_acceleration.y*msg->linear_acceleration.y +
                                msg->linear_acceleration.z*msg->linear_acceleration.z);
    const bool stationary = std::isfinite(wz) && std::isfinite(an) &&
      std::abs(wz) <= 0.03 && std::abs(an - 9.80665) <= 0.15;
    if (stationary) {
      if (stationary_since_.nanoseconds() == 0) stationary_since_ = t;
      if ((t - stationary_since_).seconds() >= 2.0)
        gyro_bias_z_rps_ = 0.98 * gyro_bias_z_rps_ + 0.02 * wz;
    } else {
      stationary_since_ = rclcpp::Time(0,0,RCL_ROS_TIME);
    }
    if (have_inertial_heading_ && last_inertial_time_.nanoseconds() != 0) {
      const double dt = (t - last_inertial_time_).seconds();
      if (dt > 0.0 && dt < 0.25 && std::isfinite(wz))
        inertial_heading_rad_ = normalizeAngle(inertial_heading_rad_ + (wz - gyro_bias_z_rps_) * dt);
    }
    last_inertial_time_ = t;
    if (have_inertial_heading_) {
      geometry_msgs::msg::PoseWithCovarianceStamped pose;
      pose.header.stamp = t; pose.header.frame_id = output_frame_;
      pose.pose.pose.orientation = yawQuaternion(inertial_heading_rad_);
      pose.pose.covariance.fill(0.0);
      pose.pose.covariance[0]=pose.pose.covariance[7]=pose.pose.covariance[14]=pose.pose.covariance[21]=pose.pose.covariance[28]=1.0e6;
      pose.pose.covariance[35] = 0.02;
      inertial_heading_pub_->publish(pose);
    }
    last_tilt_time_ = t;
    have_tilt_ = true;
    publishConsensusIfValid();
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

  double applyImuHeadingLut(double yaw_enu) const {
    if (!imu_heading_lut_enabled_ || imu_heading_lut_.size() < 2) return normalizeAngle(yaw_enu);
    const double y = normalizeAngle(yaw_enu);
    size_t hi = 0;
    while (hi < imu_heading_lut_.size() && imu_heading_lut_[hi].first < y) ++hi;
    double x0, x1, c0, c1, yy = y;
    if (hi == 0) { x0=imu_heading_lut_.back().first; c0=imu_heading_lut_.back().second; x1=imu_heading_lut_.front().first+2.0*kPi; c1=imu_heading_lut_.front().second; yy+=2.0*kPi; }
    else if (hi == imu_heading_lut_.size()) { x0=imu_heading_lut_.back().first; c0=imu_heading_lut_.back().second; x1=imu_heading_lut_.front().first+2.0*kPi; c1=imu_heading_lut_.front().second; }
    else { x0=imu_heading_lut_[hi-1].first; c0=imu_heading_lut_[hi-1].second; x1=imu_heading_lut_[hi].first; c1=imu_heading_lut_[hi].second; }
    const double f=std::clamp((yy-x0)/std::max(1.0e-9,x1-x0),0.0,1.0);
    return normalizeAngle(y + c0 + f*normalizeAngle(c1-c0));
  }

  void onImuRawMag(const std_msgs::msg::Float64MultiArray &msg) {
    if (!enable_imu_ || !imu_planar_calibration_enabled_) return;
    HeadingState &state = imu_state_;
    const auto reject = [&](const char *reason) { ++state.rejected; state.reject_reason=reason; setValid(Source::IMU,false); };
    const auto t=now();
    if (msg.data.size() < 3) { reject("raw_lsb_size"); return; }
    if (!have_map_yaw_) { reject("map_yaw_unavailable"); return; }
    if (!have_tilt_ || (t-last_tilt_time_).seconds() < 0.0 || (t-last_tilt_time_).seconds() > tilt_timeout_sec_) { reject("tilt_stale"); return; }
    if (std::abs(roll_rad_) > imu_planar_max_tilt_rad_ || std::abs(pitch_rad_) > imu_planar_max_tilt_rad_) { reject("planar_tilt_out_of_range"); return; }
    const double x=msg.data[0], y=msg.data[1];
    if (!std::isfinite(x) || !std::isfinite(y)) { reject("non_finite_raw_lsb"); return; }
    const double bx=x-imu_bias_x_lsb_, by=y-imu_bias_y_lsb_;
    const double qx=imu_m00_*bx + imu_m01_*by;
    const double qy=imu_m10_*bx + imu_m11_*by;
    const double cn=std::hypot(qx,qy); imu_corrected_norm_=cn; state.norm_ut=std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(cn) || cn < imu_corrected_norm_min_ || cn > imu_corrected_norm_max_) { reject("corrected_norm_gate"); return; }
    double yaw_enu=normalizeAngle(imu_sign_*std::atan2(qy,qx)+imu_offset_-declination_rad_);
    yaw_enu=applyImuHeadingLut(yaw_enu);
    const double yaw_map=normalizeAngle(yaw_enu+map_yaw_from_enu_);
    if (!temporalGate(state,yaw_map,t)) { reject("heading_rate_gate"); return; }
    geometry_msgs::msg::PoseWithCovarianceStamped pose; pose.header.stamp=t; pose.header.frame_id=output_frame_; pose.pose.pose.orientation=yawQuaternion(yaw_map); pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0]=pose.pose.covariance[7]=pose.pose.covariance[14]=pose.pose.covariance[21]=pose.pose.covariance[28]=1.0e6; pose.pose.covariance[35]=imu_variance_; imu_heading_pub_->publish(pose);
    state.heading_rad=yaw_map; state.stamp=t; state.reject_reason.clear(); ++state.accepted; setValid(Source::IMU,true);
  }

  double applyNeoHeadingLut(double yaw_enu) const {
    if (!neo_heading_lut_enabled_ || neo_heading_lut_.size() < 2) return normalizeAngle(yaw_enu);
    const double y = normalizeAngle(yaw_enu);
    size_t hi = 0;
    while (hi < neo_heading_lut_.size() && neo_heading_lut_[hi].first < y) ++hi;
    double x0, x1, c0, c1, yy = y;
    if (hi == 0) {
      x0 = neo_heading_lut_.back().first; c0 = neo_heading_lut_.back().second;
      x1 = neo_heading_lut_.front().first + 2.0 * kPi; c1 = neo_heading_lut_.front().second;
      yy += 2.0 * kPi;
    } else if (hi == neo_heading_lut_.size()) {
      x0 = neo_heading_lut_.back().first; c0 = neo_heading_lut_.back().second;
      x1 = neo_heading_lut_.front().first + 2.0 * kPi; c1 = neo_heading_lut_.front().second;
    } else {
      x0 = neo_heading_lut_[hi - 1].first; c0 = neo_heading_lut_[hi - 1].second;
      x1 = neo_heading_lut_[hi].first; c1 = neo_heading_lut_[hi].second;
    }
    const double span = std::max(1.0e-9, x1 - x0);
    const double f = std::clamp((yy - x0) / span, 0.0, 1.0);
    const double dc = normalizeAngle(c1 - c0);
    return normalizeAngle(y + c0 + f * dc);
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

    double mx_heading = mx;
    double my_heading = my;
    if (source == Source::NEO3 && neo_planar_calibration_enabled_) {
      const double bx = mx - neo_bias_x_ut_;
      const double by = my - neo_bias_y_ut_;
      mx_heading = neo_m00_ * bx + neo_m01_ * by;
      my_heading = neo_m10_ * bx + neo_m11_ * by;
    }
    const double cr = std::cos(roll_rad_), sr = std::sin(roll_rad_);
    const double cp = std::cos(pitch_rad_), sp = std::sin(pitch_rad_);
    const double xh = mx_heading * cp + mz * sp;
    const double yh = mx_heading * sr * sp + my_heading * cr - mz * sr * cp;
    if (std::hypot(xh, yh) < 1.0) { reject("horizontal_field_too_small"); return; }

    const double raw_north_in_body = std::atan2(yh, xh);
    const double sign = source == Source::IMU ? imu_sign_ : neo_sign_;
    const double offset = source == Source::IMU ? imu_offset_ : neo_offset_;
    double yaw_enu = normalizeAngle(sign * raw_north_in_body + offset - declination_rad_);
    if (source == Source::NEO3) yaw_enu = applyNeoHeadingLut(yaw_enu);
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
    if (source == Source::NEO3 && !have_inertial_heading_) {
      inertial_heading_rad_ = yaw_map;
      have_inertial_heading_ = true;
      last_inertial_time_ = t;
      last_correction_time_ = t;
    }
    publishConsensusIfValid();
  }

  void publishConsensusIfValid() {
    const auto t = now();
    const bool inertial_fresh = have_inertial_heading_ && (t - last_inertial_time_).seconds() >= 0.0 && (t - last_inertial_time_).seconds() <= consensus_timeout_sec_;
    const bool neo_fresh = neo_state_.valid && neo_state_.stamp.nanoseconds() != 0 && (t - neo_state_.stamp).seconds() >= 0.0 && (t - neo_state_.stamp).seconds() <= consensus_timeout_sec_;
    const double err = (inertial_fresh && neo_fresh) ? std::abs(normalizeAngle(inertial_heading_rad_ - neo_state_.heading_rad)) : kPi;
    const bool pair_ok = inertial_fresh && neo_fresh && err <= consensus_max_error_rad_;
    if (!pair_ok) {
      consensus_since_ = rclcpp::Time(0,0,RCL_ROS_TIME);
      if (consensus_valid_) { consensus_valid_ = false; std_msgs::msg::Bool b; b.data=false; consensus_valid_pub_->publish(b); }
      consensus_error_rad_ = err;
      return;
    }
    if (consensus_since_.nanoseconds() == 0) consensus_since_ = t;
    consensus_error_rad_ = err;
    if ((t - consensus_since_).seconds() < consensus_hold_sec_) return;
    if (last_correction_time_.nanoseconds() == 0) last_correction_time_ = t;
    const double corr_dt = std::clamp((t - last_correction_time_).seconds(), 0.0, 0.25);
    const double correction = normalizeAngle(neo_state_.heading_rad - inertial_heading_rad_);
    const double max_step = 0.01745329252 * corr_dt; // maksimum 1 deg/s, tanpa heading jump
    inertial_heading_rad_ = normalizeAngle(inertial_heading_rad_ + std::clamp(correction, -max_step, max_step));
    last_correction_time_ = t;
    const double w_neo = 1.0 / std::max(neo_variance_, 1.0e-4);
    const double w_imu = 1.0 / 0.05;
    const double sx = w_neo*std::cos(neo_state_.heading_rad) + w_imu*std::cos(inertial_heading_rad_);
    const double sy = w_neo*std::sin(neo_state_.heading_rad) + w_imu*std::sin(inertial_heading_rad_);
    const double yaw = std::atan2(sy, sx);
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = t; pose.header.frame_id = output_frame_;
    pose.pose.pose.orientation = yawQuaternion(yaw);
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0]=pose.pose.covariance[7]=pose.pose.covariance[14]=pose.pose.covariance[21]=pose.pose.covariance[28]=1.0e6;
    pose.pose.covariance[35] = std::max(validated_variance_, err*err);
    validated_heading_pub_->publish(pose);
    validated_heading_rad_ = yaw;
    if (!consensus_valid_) { consensus_valid_=true; std_msgs::msg::Bool b; b.data=true; consensus_valid_pub_->publish(b); }
  }

  void publishStatus() {
    std_msgs::msg::String msg;
    const auto t = now();
    const double tilt_age = have_tilt_ ? (t - last_tilt_time_).seconds() : 999.0;
    const double map_age = have_map_yaw_ ? (t - last_map_yaw_time_).seconds() : 999.0;
    msg.data = std::string("imu_valid=") + (imu_state_.valid ? "true" : "false") +
      ";neo3_valid=" + (neo_state_.valid ? "true" : "false") +
      ";imu_norm_ut=" + std::to_string(imu_state_.norm_ut) +
      ";imu_corrected_norm=" + std::to_string(imu_corrected_norm_) +
      ";neo3_norm_ut=" + std::to_string(neo_state_.norm_ut) +
      ";imu_reject=" + imu_state_.reject_reason +
      ";neo3_reject=" + neo_state_.reject_reason +
      ";tilt_age_sec=" + std::to_string(tilt_age) +
      ";map_yaw_age_sec=" + std::to_string(map_age) +
      ";inertial_heading_deg=" + std::to_string(inertial_heading_rad_ * 180.0 / kPi) +
      ";consensus_error_deg=" + std::to_string(consensus_error_rad_ * 180.0 / kPi) +
      ";consensus_valid=" + (consensus_valid_ ? "true" : "false") +
      ";validated_heading_deg=" + std::to_string(validated_heading_rad_ * 180.0 / kPi) +
      ";gyro_bias_z_rps=" + std::to_string(gyro_bias_z_rps_);
    status_pub_->publish(msg);
  }

  std::string imu_topic_, imu_mag_topic_, imu_raw_mag_topic_, neo3_mag_topic_, map_yaw_topic_, output_frame_;
  double tilt_timeout_sec_{0.75}, max_tilt_rad_{0.7853981634}, declination_rad_{0.0};
  double min_norm_ut_{15.0}, max_norm_ut_{100.0};
  double imu_sign_{-1.0}, imu_offset_{1.5451826276}, neo_sign_{-1.0}, neo_offset_{1.5707963268};
  bool imu_planar_calibration_enabled_{false}, imu_heading_lut_enabled_{false};
  double imu_bias_x_lsb_{0.0}, imu_bias_y_lsb_{0.0};
  double imu_m00_{1.0}, imu_m01_{0.0}, imu_m10_{0.0}, imu_m11_{1.0};
  double imu_corrected_norm_min_{0.70}, imu_corrected_norm_max_{1.30}, imu_planar_max_tilt_rad_{0.2617993878}, imu_corrected_norm_{0.0};
  std::vector<std::pair<double,double>> imu_heading_lut_;
  bool neo_planar_calibration_enabled_{false}, neo_heading_lut_enabled_{false};
  double neo_bias_x_ut_{0.0}, neo_bias_y_ut_{0.0};
  double neo_m00_{1.0}, neo_m01_{0.0}, neo_m10_{0.0}, neo_m11_{1.0};
  std::vector<std::pair<double, double>> neo_heading_lut_;
  double imu_variance_{0.08}, neo_variance_{0.06};
  double max_heading_rate_rps_{3.0}, rate_gate_margin_rad_{0.35};
  double consensus_max_error_rad_{0.0872664626}, consensus_hold_sec_{2.0}, consensus_timeout_sec_{0.5};
  double validated_variance_{0.01}, inertial_heading_rad_{0.0}, validated_heading_rad_{0.0}, consensus_error_rad_{kPi};
  bool enable_imu_{true}, enable_neo_{true};
  bool have_tilt_{false}, have_map_yaw_{false}, have_inertial_heading_{false}, consensus_valid_{false};
  double roll_rad_{0.0}, pitch_rad_{0.0}, map_yaw_from_enu_{0.0};
  rclcpp::Time last_tilt_time_{0, 0, RCL_ROS_TIME}, last_map_yaw_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_inertial_time_{0,0,RCL_ROS_TIME}, consensus_since_{0,0,RCL_ROS_TIME};
  rclcpp::Time stationary_since_{0,0,RCL_ROS_TIME}, last_correction_time_{0,0,RCL_ROS_TIME};
  double gyro_bias_z_rps_{0.0};
  HeadingState imu_state_, neo_state_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_sub_, neo_mag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr imu_raw_mag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr map_yaw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr imu_heading_pub_, neo_heading_pub_, inertial_heading_pub_, validated_heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_valid_pub_, neo_valid_pub_, consensus_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagHeadingFusionNode>());
  rclcpp::shutdown();
  return 0;
}
