#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace {
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

double angleDiff(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}
}  // namespace

class PrecisionLocalizationMonitor final : public rclcpp::Node
{
public:
  PrecisionLocalizationMonitor() : Node("precision_localization_monitor")
  {
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/localization/absolute_pose");
    required_frame_ = declare_parameter<std::string>("required_frame", "map");
    timeout_sec_ = std::clamp(declare_parameter<double>("timeout_sec", 0.5), 0.05, 5.0);
    max_stamp_age_sec_ = std::clamp(declare_parameter<double>("max_stamp_age_sec", 0.5), 0.05, 5.0);
    max_future_stamp_sec_ = std::clamp(declare_parameter<double>("max_future_stamp_sec", 0.05), 0.0, 1.0);
    max_position_std_m_ = std::clamp(declare_parameter<double>("max_position_std_m", 0.05), 0.001, 5.0);
    max_yaw_std_rad_ = std::clamp(declare_parameter<double>("max_yaw_std_rad", 0.03490658504), 0.001, 3.14);
    max_sample_jump_m_ = std::clamp(declare_parameter<double>("max_sample_jump_m", 0.30), 0.01, 10.0);
    max_sample_yaw_jump_rad_ = std::clamp(
      declare_parameter<double>("max_sample_yaw_jump_rad", 0.1745329252), 0.01, 3.14);
    min_consecutive_samples_ = std::max<int64_t>(1, declare_parameter<int64_t>("min_consecutive_samples", 10));

    ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/precision_localization_ready", stateQos());
    status_pub_ = create_publisher<std_msgs::msg::String>("/system/precision_localization_status", stateQos());
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&PrecisionLocalizationMonitor::onPose, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PrecisionLocalizationMonitor::publishState, this));
  }

private:
  void reject(const std::string & reason)
  {
    last_reason_ = reason;
    consecutive_good_ = 0;
    sample_valid_ = false;
  }

  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    const auto arrival = now();
    if (msg->header.frame_id != required_frame_) {
      reject("wrong_frame"); return;
    }
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      reject("missing_measurement_stamp"); return;
    }
    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    const double age = (arrival - stamp).seconds();
    if (!std::isfinite(age) || age > max_stamp_age_sec_ || age < -max_future_stamp_sec_) {
      reject("timestamp_age"); return;
    }

    const double vx = msg->pose.covariance[0];
    const double vy = msg->pose.covariance[7];
    const double vyaw = msg->pose.covariance[35];
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vyaw) ||
        vx <= 0.0 || vy <= 0.0 || vyaw <= 0.0) {
      reject("invalid_covariance"); return;
    }
    const double position_std = std::sqrt(std::max(vx, vy));
    const double yaw_std = std::sqrt(vyaw);
    if (position_std > max_position_std_m_ || yaw_std > max_yaw_std_rad_) {
      last_position_std_m_ = position_std;
      last_yaw_std_rad_ = yaw_std;
      reject("covariance_too_large"); return;
    }

    const auto & q = msg->pose.pose.orientation;
    const double qnorm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(qnorm) || qnorm < 0.5 || qnorm > 1.5 ||
        !std::isfinite(msg->pose.pose.position.x) || !std::isfinite(msg->pose.pose.position.y)) {
      reject("invalid_pose"); return;
    }
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double yaw = yawFromQuaternion(q);
    if (have_previous_) {
      const double jump = std::hypot(x - previous_x_, y - previous_y_);
      const double yaw_jump = std::abs(angleDiff(yaw, previous_yaw_));
      if (jump > max_sample_jump_m_ || yaw_jump > max_sample_yaw_jump_rad_) {
        reject(jump > max_sample_jump_m_ ? "pose_jump" : "yaw_jump");
        previous_x_ = x; previous_y_ = y; previous_yaw_ = yaw;
        last_measurement_stamp_ = stamp; last_arrival_ = arrival; have_previous_ = true;
        return;
      }
    }

    previous_x_ = x; previous_y_ = y; previous_yaw_ = yaw; have_previous_ = true;
    last_position_std_m_ = position_std;
    last_yaw_std_rad_ = yaw_std;
    last_measurement_stamp_ = stamp;
    last_arrival_ = arrival;
    sample_valid_ = true;
    last_reason_ = "ok";
    consecutive_good_ = std::min<int64_t>(1000000, consecutive_good_ + 1);
  }

  void publishState()
  {
    const auto t = now();
    const double arrival_age = last_arrival_.nanoseconds() > 0 ? (t - last_arrival_).seconds() : 999.0;
    const bool fresh = arrival_age >= 0.0 && arrival_age <= timeout_sec_;
    const bool ready = sample_valid_ && fresh && consecutive_good_ >= min_consecutive_samples_;
    if (!fresh && last_arrival_.nanoseconds() > 0) {
      sample_valid_ = false;
      consecutive_good_ = 0;
      last_reason_ = "stale";
    }
    std_msgs::msg::Bool b; b.data = ready; ready_pub_->publish(b);
    std::ostringstream ss;
    ss << std::boolalpha << std::fixed << std::setprecision(6)
       << "ready=" << ready
       << ";source=" << pose_topic_
       << ";reason=" << last_reason_
       << ";consecutive=" << consecutive_good_
       << ";age_sec=" << arrival_age
       << ";position_std_m=" << last_position_std_m_
       << ";yaw_std_rad=" << last_yaw_std_rad_
       << ";max_position_std_m=" << max_position_std_m_
       << ";max_yaw_std_rad=" << max_yaw_std_rad_;
    std_msgs::msg::String status; status.data = ss.str(); status_pub_->publish(status);
  }

  std::string pose_topic_, required_frame_, last_reason_{"no_sample"};
  double timeout_sec_{0.5}, max_stamp_age_sec_{0.5}, max_future_stamp_sec_{0.05};
  double max_position_std_m_{0.05}, max_yaw_std_rad_{0.03490658504};
  double max_sample_jump_m_{0.30}, max_sample_yaw_jump_rad_{0.1745329252};
  int64_t min_consecutive_samples_{10}, consecutive_good_{0};
  bool sample_valid_{false}, have_previous_{false};
  double previous_x_{0.0}, previous_y_{0.0}, previous_yaw_{0.0};
  double last_position_std_m_{999.0}, last_yaw_std_rad_{999.0};
  rclcpp::Time last_measurement_stamp_{0, 0, RCL_ROS_TIME}, last_arrival_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrecisionLocalizationMonitor>());
  rclcpp::shutdown();
  return 0;
}
