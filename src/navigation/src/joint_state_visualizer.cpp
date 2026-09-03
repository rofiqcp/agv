// Lightweight URDF joint-state bridge for the operator/Nav2 visualization.
//
// robot_state_publisher can publish all fixed joints immediately, but the four
// wheel joints and two steering joints are movable. Without /joint_states those
// links never receive TF and RViz RobotModel looks incomplete/absent. This node
// converts the already available ESC feedback into sensor_msgs/JointState and
// publishes a safe zero pose even before ESC feedback becomes valid.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

namespace navigation
{
class JointStateVisualizer final : public rclcpp::Node
{
public:
  JointStateVisualizer()
  : Node("joint_state_visualizer")
  {
    declare_parameter<double>("publish_rate_hz", 20.0);
    declare_parameter<double>("wheelbase_m", 0.70);
    declare_parameter<double>("track_width_m", 0.48);
    declare_parameter<double>("wheel_radius_m", 0.145);
    declare_parameter<double>("max_visual_steering_rad", 1.3962634016);
    declare_parameter<double>("feedback_timeout_sec", 0.75);

    publish_rate_hz_ = std::clamp(get_parameter("publish_rate_hz").as_double(), 2.0, 50.0);
    wheelbase_m_ = std::max(0.05, get_parameter("wheelbase_m").as_double());
    track_width_m_ = std::max(0.01, get_parameter("track_width_m").as_double());
    wheel_radius_m_ = std::max(0.01, get_parameter("wheel_radius_m").as_double());
    max_visual_steering_rad_ = std::clamp(
      get_parameter("max_visual_steering_rad").as_double(), 0.05, 1.55);
    feedback_timeout_sec_ = std::clamp(
      get_parameter("feedback_timeout_sec").as_double(), 0.1, 5.0);

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));

    const auto qos = rclcpp::SensorDataQoS();
    steering_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/steering_actual_rad", qos,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) return;
        std::lock_guard<std::mutex> lock(mutex_);
        steering_rad_ = std::clamp(msg->data, -max_visual_steering_rad_, max_visual_steering_rad_);
      });
    speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/drive_actual_mps", qos,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) return;
        std::lock_guard<std::mutex> lock(mutex_);
        speed_mps_ = msg->data;
        speed_stamp_ = now();
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&JointStateVisualizer::publish, this));

    last_publish_steady_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "URDF joint bridge aktif: /esc/steering_actual_rad + /esc/drive_actual_mps -> /joint_states @ %.1f Hz",
      publish_rate_hz_);
  }

private:
  void ackermannAngles(double center, double & left, double & right) const
  {
    if (std::abs(center) < 1.0e-5) {
      left = 0.0;
      right = 0.0;
      return;
    }

    const double mag = std::min(std::abs(center), max_visual_steering_rad_);
    const double tan_mag = std::tan(mag);
    if (!std::isfinite(tan_mag) || std::abs(tan_mag) < 1.0e-6) {
      left = right = 0.0;
      return;
    }

    const double radius = wheelbase_m_ / tan_mag;
    // Keep denominator positive even at extreme visualization angles.
    const double inner_den = std::max(0.02, radius - 0.5 * track_width_m_);
    const double outer_den = std::max(0.02, radius + 0.5 * track_width_m_);
    const double inner = std::atan(wheelbase_m_ / inner_den);
    const double outer = std::atan(wheelbase_m_ / outer_den);

    if (center > 0.0) {  // left turn: FL inner, FR outer
      left = inner;
      right = outer;
    } else {             // right turn: FR inner, FL outer
      left = -outer;
      right = -inner;
    }
    left = std::clamp(left, -max_visual_steering_rad_, max_visual_steering_rad_);
    right = std::clamp(right, -max_visual_steering_rad_, max_visual_steering_rad_);
  }

  bool fresh(const rclcpp::Time & stamp, const rclcpp::Time & t) const
  {
    if (stamp.nanoseconds() == 0) return false;
    const double age = (t - stamp).seconds();
    return age >= 0.0 && age <= feedback_timeout_sec_;
  }

  void publish()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(steady_now - last_publish_steady_).count();
    last_publish_steady_ = steady_now;
    dt = std::clamp(dt, 0.0, 0.25);

    const auto t = now();
    double steering = 0.0;
    double speed = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Steering stays at last measured value for a natural static model.
      steering = steering_rad_;
      // Stale drive feedback must not keep visually spinning wheels forever.
      speed = fresh(speed_stamp_, t) ? speed_mps_ : 0.0;
    }

    wheel_phase_rad_ = std::remainder(
      wheel_phase_rad_ + (speed / wheel_radius_m_) * dt, 2.0 * 3.14159265358979323846);

    double left_steer = 0.0;
    double right_steer = 0.0;
    ackermannAngles(steering, left_steer, right_steer);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = t;
    msg.name = {
      "front_left_steering_joint", "front_right_steering_joint",
      "front_left_wheel_joint", "front_right_wheel_joint",
      "rear_left_wheel_joint", "rear_right_wheel_joint"};
    msg.position = {
      left_steer, right_steer,
      wheel_phase_rad_, wheel_phase_rad_, wheel_phase_rad_, wheel_phase_rad_};
    msg.velocity = {
      0.0, 0.0,
      speed / wheel_radius_m_, speed / wheel_radius_m_,
      speed / wheel_radius_m_, speed / wheel_radius_m_};
    joint_pub_->publish(msg);
  }

  double publish_rate_hz_{20.0};
  double wheelbase_m_{0.70};
  double track_width_m_{0.48};
  double wheel_radius_m_{0.145};
  double max_visual_steering_rad_{1.3962634016};
  double feedback_timeout_sec_{0.75};

  std::mutex mutex_;
  double steering_rad_{0.0};
  double speed_mps_{0.0};
  double wheel_phase_rad_{0.0};
  rclcpp::Time speed_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_publish_steady_{};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation::JointStateVisualizer>());
  rclcpp::shutdown();
  return 0;
}
