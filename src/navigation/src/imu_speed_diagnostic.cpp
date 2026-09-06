#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <optional>

using namespace std::chrono_literals;

namespace {
constexpr double kGravity = 9.80665;
double pitchFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  return std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
}
}

class ImuSpeedDiagnostic final : public rclcpp::Node {
public:
  ImuSpeedDiagnostic() : Node("imu_speed_diagnostic") {
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<double>("accel_lpf_alpha", 0.18);
    declare_parameter<double>("process_velocity_noise", 0.20);
    declare_parameter<double>("process_bias_noise", 0.002);
    declare_parameter<double>("zupt_velocity_variance", 0.0025);
    declare_parameter<double>("stationary_accel_norm_tolerance_mps2", 0.35);
    declare_parameter<double>("stationary_gyro_norm_max_rps", 0.04);
    declare_parameter<double>("stationary_hold_sec", 0.6);
    declare_parameter<bool>("gyro_assisted_stationary_gate", true);
    declare_parameter<double>("forward_accel_sign", 1.0);
    declare_parameter<double>("gravity_compensation_sign", 1.0);
    declare_parameter<double>("max_abs_speed_mps", 3.0);
    configure();
    const auto qos = rclcpp::SensorDataQoS().keep_last(20);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos, std::bind(&ImuSpeedDiagnostic::onImu, this, std::placeholders::_1));
    speed_pub_ = create_publisher<std_msgs::msg::Float64>("/imu/speed_kalman", qos);
    accel_pub_ = create_publisher<std_msgs::msg::Float64>("/imu/forward_accel_filtered", qos);
    bias_pub_ = create_publisher<std_msgs::msg::Float64>("/imu/forward_accel_bias", qos);
    stationary_pub_ = create_publisher<std_msgs::msg::Bool>("/imu/speed_stationary", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/imu/speed_diagnostic_status", 10);
    RCLCPP_INFO(get_logger(), "IMU diagnostic speed active; NOT connected to local/global EKF");
  }

private:
  void configure() {
    imu_topic_ = get_parameter("imu_topic").as_string();
    alpha_ = std::clamp(get_parameter("accel_lpf_alpha").as_double(), 0.01, 1.0);
    q_v_ = std::max(1e-8, get_parameter("process_velocity_noise").as_double());
    q_b_ = std::max(1e-10, get_parameter("process_bias_noise").as_double());
    r_zupt_ = std::max(1e-8, get_parameter("zupt_velocity_variance").as_double());
    accel_tol_ = std::max(0.02, get_parameter("stationary_accel_norm_tolerance_mps2").as_double());
    gyro_max_ = std::max(0.001, get_parameter("stationary_gyro_norm_max_rps").as_double());
    stationary_hold_sec_ = std::max(0.1, get_parameter("stationary_hold_sec").as_double());
    gyro_assisted_ = get_parameter("gyro_assisted_stationary_gate").as_bool();
    accel_sign_ = get_parameter("forward_accel_sign").as_double() < 0.0 ? -1.0 : 1.0;
    gravity_sign_ = get_parameter("gravity_compensation_sign").as_double() < 0.0 ? -1.0 : 1.0;
    max_speed_ = std::max(0.2, get_parameter("max_abs_speed_mps").as_double());
  }
  void predict(double accel, double dt) {
    velocity_ += (accel - bias_) * dt;
    const double p00 = p00_ - dt * (p10_ + p01_) + dt * dt * p11_ + q_v_ * dt * dt;
    const double p01 = p01_ - dt * p11_;
    const double p10 = p10_ - dt * p11_;
    const double p11 = p11_ + q_b_ * dt;
    p00_ = p00; p01_ = p01; p10_ = p10; p11_ = p11;
  }

  void zuptUpdate() {
    const double innovation = -velocity_;
    const double s = p00_ + r_zupt_;
    if (!(s > 1e-12) || !std::isfinite(s)) return;
    const double k0 = p00_ / s;
    const double k1 = p10_ / s;
    velocity_ += k0 * innovation;
    bias_ += k1 * innovation;
    const double p00 = (1.0 - k0) * p00_;
    const double p01 = (1.0 - k0) * p01_;
    const double p10 = p10_ - k1 * p00_;
    const double p11 = p11_ - k1 * p01_;
    p00_ = p00; p01_ = p01; p10_ = p10; p11_ = p11;
    if (std::abs(velocity_) < 0.005) velocity_ = 0.0;
  }

  void publish(double accel, bool stationary, double accel_norm, double gyro_norm) {
    std_msgs::msg::Float64 f;
    f.data = velocity_; speed_pub_->publish(f);
    f.data = accel; accel_pub_->publish(f);
    f.data = bias_; bias_pub_->publish(f);
    std_msgs::msg::Bool b; b.data = stationary; stationary_pub_->publish(b);
    if (++status_divider_ % 10 != 0) return;
    std_msgs::msg::String s; std::ostringstream o;
    o << "speed_mps=" << velocity_ << ";accel_forward=" << accel
      << ";bias=" << bias_ << ";stationary=" << (stationary ? "true" : "false")
      << ";gate=" << (gyro_assisted_ ? "ACCEL_GYRO_ZUPT" : "ACCEL_ONLY_ZUPT")
      << ";accel_norm=" << accel_norm << ";gyro_norm=" << gyro_norm
      << ";ekf_fusion=false";
    s.data = o.str(); status_pub_->publish(s);
  }
  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    const rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() <= 0) return;
    if (last_stamp_.nanoseconds() <= 0) { last_stamp_ = stamp; return; }
    const double dt = (stamp - last_stamp_).seconds();
    last_stamp_ = stamp;
    if (!(dt > 0.0) || dt > 0.25) { stationary_since_.reset(); return; }

    const auto &a = msg->linear_acceleration;
    const auto &g = msg->angular_velocity;
    const double accel_norm = std::sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
    const double gyro_norm = std::sqrt(g.x*g.x + g.y*g.y + g.z*g.z);
    const double pitch = pitchFromQuaternion(msg->orientation);
    const double forward = accel_sign_ * (a.x + gravity_sign_ * kGravity * std::sin(pitch));
    if (!std::isfinite(forward) || !std::isfinite(accel_norm) || !std::isfinite(gyro_norm)) return;
    if (!lpf_initialized_) { accel_filtered_ = forward; lpf_initialized_ = true; }
    else accel_filtered_ += alpha_ * (forward - accel_filtered_);

    predict(accel_filtered_, dt);
    velocity_ = std::clamp(velocity_, -max_speed_, max_speed_);
    const bool accel_stationary = std::abs(accel_norm - kGravity) <= accel_tol_;
    const bool gate = accel_stationary && (!gyro_assisted_ || gyro_norm <= gyro_max_);
    if (gate) {
      if (!stationary_since_) stationary_since_ = stamp;
    } else {
      stationary_since_.reset();
    }
    const bool stationary = stationary_since_.has_value() &&
      (stamp - *stationary_since_).seconds() >= stationary_hold_sec_;
    if (stationary) zuptUpdate();
    publish(accel_filtered_, stationary, accel_norm, gyro_norm);
  }

  std::string imu_topic_;
  double alpha_{0.18}, q_v_{0.20}, q_b_{0.002}, r_zupt_{0.0025};
  double accel_tol_{0.35}, gyro_max_{0.04}, stationary_hold_sec_{0.6};
  double accel_sign_{1.0}, gravity_sign_{1.0}, max_speed_{3.0};
  bool gyro_assisted_{true}, lpf_initialized_{false};
  double velocity_{0.0}, bias_{0.0}, accel_filtered_{0.0};
  double p00_{0.25}, p01_{0.0}, p10_{0.0}, p11_{0.04};
  int status_divider_{0};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> stationary_since_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_, accel_pub_, bias_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stationary_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSpeedDiagnostic>());
  rclcpp::shutdown();
  return 0;
}
