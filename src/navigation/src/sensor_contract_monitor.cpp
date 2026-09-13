#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class SensorContractMonitor final : public rclcpp::Node
{
public:
  SensorContractMonitor() : Node("sensor_contract_monitor")
  {
    topics_ = declare_parameter<std::vector<std::string>>(
      "required_single_publisher_topics",
      {"/imu/data", "/gnss/fix_raw", "/gnss/vel", "/esc/odom"});
    check_rate_hz_ = std::clamp(declare_parameter<double>("check_rate_hz", 5.0), 1.0, 20.0);
    imu_timeout_sec_ = std::clamp(declare_parameter<double>("imu_timeout_sec", 0.75), 0.05, 5.0);
    gnss_fix_timeout_sec_ = std::clamp(declare_parameter<double>("gnss_fix_timeout_sec", 2.5), 0.1, 10.0);
    gnss_velocity_timeout_sec_ = std::clamp(declare_parameter<double>("gnss_velocity_timeout_sec", 0.8), 0.05, 5.0);
    gnss_quality_timeout_sec_ = std::clamp(declare_parameter<double>("gnss_quality_timeout_sec", 0.8), 0.05, 5.0);
    esc_odom_timeout_sec_ = std::clamp(declare_parameter<double>("esc_odom_timeout_sec", 0.75), 0.05, 5.0);
    max_future_stamp_sec_ = std::clamp(declare_parameter<double>("max_future_stamp_sec", 0.20), 0.0, 2.0);
    max_message_stamp_age_sec_ = std::clamp(declare_parameter<double>("max_message_stamp_age_sec", 3.0), 0.1, 30.0);

    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    publisher_ok_pub_ = create_publisher<std_msgs::msg::Bool>("/system/sensor_publishers_ok", state_qos);
    publisher_status_pub_ = create_publisher<std_msgs::msg::String>("/system/sensor_publishers_status", state_qos);
    transport_ok_pub_ = create_publisher<std_msgs::msg::Bool>("/system/sensors_transport_ok", state_qos);
    transport_status_pub_ = create_publisher<std_msgs::msg::String>("/system/sensors_transport_status", state_qos);
    localization_inputs_ok_pub_ = create_publisher<std_msgs::msg::Bool>("/system/localization_inputs_ok", state_qos);

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
        imu_.seen = true; imu_.received = now();
        imu_.valid = stampSane(m->header.stamp) && std::isfinite(m->angular_velocity.z) &&
          std::isfinite(m->angular_velocity_covariance[8]) && m->angular_velocity_covariance[8] >= 0.0;
      });
    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/gnss/fix_raw", sensor_qos,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr m) {
        fix_.seen = true; fix_.received = now();
        const bool ll = std::isfinite(m->latitude) && std::isfinite(m->longitude) &&
          std::abs(m->latitude) <= 90.0 && std::abs(m->longitude) <= 180.0;
        const bool cov = std::isfinite(m->position_covariance[0]) &&
          std::isfinite(m->position_covariance[4]) && std::isfinite(m->position_covariance[8]) &&
          m->position_covariance[0] >= 0.0 && m->position_covariance[4] >= 0.0 && m->position_covariance[8] >= 0.0;
        fix_.valid = stampSane(m->header.stamp) && ll && cov;
      });
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/gnss/vel", sensor_qos,
      [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr m) {
        vel_.seen = true; vel_.received = now();
        const auto &v = m->twist.twist.linear;
        const auto &c = m->twist.covariance;
        const bool cov = std::isfinite(c[0]) && std::isfinite(c[7]) && std::isfinite(c[14]) &&
          c[0] >= 0.0 && c[7] >= 0.0 && c[14] >= 0.0;
        vel_.valid = stampSane(m->header.stamp) && std::isfinite(v.x) &&
          std::isfinite(v.y) && std::isfinite(v.z) && cov;
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/esc/odom", sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        odom_.seen = true; odom_.received = now();
        const auto &t = m->twist.twist;
        odom_.valid = stampSane(m->header.stamp) && !m->header.frame_id.empty() && !m->child_frame_id.empty() &&
          std::isfinite(t.linear.x) && std::isfinite(t.angular.z) &&
          std::isfinite(m->twist.covariance[0]) && std::isfinite(m->twist.covariance[35]) &&
          m->twist.covariance[0] >= 0.0 && m->twist.covariance[35] >= 0.0;
      });
    quality_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/gnss/quality", sensor_qos,
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr m) { onQuality(*m); });

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / check_rate_hz_)),
      std::bind(&SensorContractMonitor::tick, this));
  }

private:
  struct InputState { bool seen{false}; bool valid{false}; rclcpp::Time received{0,0,RCL_ROS_TIME}; };

  bool stampSane(const builtin_interfaces::msg::Time &raw) const
  {
    if (raw.sec == 0 && raw.nanosec == 0) return false;
    const rclcpp::Time stamp(raw, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    return std::isfinite(age) && age >= -max_future_stamp_sec_ && age <= max_message_stamp_age_sec_;
  }

  bool fresh(const InputState &s, double timeout) const
  {
    if (!s.seen || !s.valid || s.received.nanoseconds() <= 0) return false;
    const double age = (now() - s.received).seconds();
    return std::isfinite(age) && age >= 0.0 && age <= timeout;
  }

  double age(const InputState &s) const
  {
    if (!s.seen || s.received.nanoseconds() <= 0) return 999.0;
    return (now() - s.received).seconds();
  }

  void onQuality(const std_msgs::msg::Float64MultiArray &m)
  {
    quality_.seen = true; quality_.received = now(); quality_.valid = false;
    quality_source_ = 0; quality_high_integrity_ = false;
    if (m.data.size() < 24U) return;
    for (size_t i : {size_t(0),size_t(1),size_t(2),size_t(3),size_t(4),size_t(5),size_t(23)})
      if (!std::isfinite(m.data[i])) return;
    quality_source_ = static_cast<int>(std::lround(m.data[4]));
    const bool source_hi = quality_source_ == 1 || quality_source_ == 4;
    const bool fix_ok = m.data.size() >= 45U && std::isfinite(m.data[44]) && m.data[44] > 0.5;
    const bool age_ok = m.data[23] >= 0.0 && m.data[23] <= 1.0;
    quality_.valid = quality_source_ >= 1 && quality_source_ <= 4 && age_ok;
    if (!source_hi || !fix_ok) return;
    if (quality_source_ == 4) {
      if (m.data.size() < 50U || !std::isfinite(m.data[21]) || !std::isfinite(m.data[24]) ||
          !std::isfinite(m.data[49])) return;
      const int stamp_source = static_cast<int>(std::lround(m.data[24]));
      if ((stamp_source != 3 && stamp_source != 4) || m.data[21] <= 0.5 || m.data[49] <= 0.5) return;
    }
    quality_high_integrity_ = true;
  }

  void tick()
  {
    bool publisher_ok = !topics_.empty();
    std::ostringstream counts;
    counts << '[';
    for (size_t i = 0; i < topics_.size(); ++i) {
      const auto infos = get_publishers_info_by_topic(topics_[i]);
      publisher_ok = publisher_ok && infos.size() == 1U;
      if (i) counts << ',';
      counts << "{\"topic\":\"" << topics_[i] << "\",\"count\":" << infos.size() << '}';
    }
    counts << ']';

    const bool imu_ok = fresh(imu_, imu_timeout_sec_);
    const bool fix_ok = fresh(fix_, gnss_fix_timeout_sec_);
    const bool vel_ok = fresh(vel_, gnss_velocity_timeout_sec_);
    const bool odom_ok = fresh(odom_, esc_odom_timeout_sec_);
    const bool quality_ok = fresh(quality_, gnss_quality_timeout_sec_);
    const bool transport_ok = publisher_ok && imu_ok && fix_ok && vel_ok && odom_ok && quality_ok;
    const bool localization_inputs_ok = transport_ok && quality_high_integrity_;

    std_msgs::msg::Bool b; b.data = publisher_ok; publisher_ok_pub_->publish(b);
    b.data = transport_ok; transport_ok_pub_->publish(b);
    b.data = localization_inputs_ok; localization_inputs_ok_pub_->publish(b);

    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "{\"publisher_ok\":" << (publisher_ok?"true":"false")
        << ",\"transport_ok\":" << (transport_ok?"true":"false")
        << ",\"localization_inputs_ok\":" << (localization_inputs_ok?"true":"false")
        << ",\"imu\":{\"ok\":" << (imu_ok?"true":"false") << ",\"age\":" << age(imu_) << "}"
        << ",\"fix\":{\"ok\":" << (fix_ok?"true":"false") << ",\"age\":" << age(fix_) << "}"
        << ",\"vel\":{\"ok\":" << (vel_ok?"true":"false") << ",\"age\":" << age(vel_) << "}"
        << ",\"quality\":{\"ok\":" << (quality_ok?"true":"false") << ",\"age\":" << age(quality_)
        << ",\"source\":" << quality_source_ << ",\"high_integrity\":" << (quality_high_integrity_?"true":"false") << "}"
        << ",\"odom\":{\"ok\":" << (odom_ok?"true":"false") << ",\"age\":" << age(odom_) << "}"
        << ",\"publishers\":" << counts.str() << '}';
    const std::string status = out.str();

    if (!initialized_ || publisher_ok != last_publisher_ok_ || status != last_status_) {
      std_msgs::msg::String s; s.data = status; publisher_status_pub_->publish(s); transport_status_pub_->publish(s);
      if (initialized_ && last_transport_ok_ && !transport_ok)
        RCLCPP_ERROR(get_logger(), "Sensor transport integrity LOST: %s", status.c_str());
      else if (!initialized_ || (!last_transport_ok_ && transport_ok))
        RCLCPP_INFO(get_logger(), "Sensor transport integrity %s: %s", transport_ok?"READY":"WAIT", status.c_str());
      last_status_ = status;
    }
    initialized_ = true; last_publisher_ok_ = publisher_ok; last_transport_ok_ = transport_ok;
  }

  std::vector<std::string> topics_;
  double check_rate_hz_{5.0}, imu_timeout_sec_{0.75}, gnss_fix_timeout_sec_{2.5};
  double gnss_velocity_timeout_sec_{0.8}, gnss_quality_timeout_sec_{0.8}, esc_odom_timeout_sec_{0.75};
  double max_future_stamp_sec_{0.20}, max_message_stamp_age_sec_{3.0};
  InputState imu_{}, fix_{}, vel_{}, odom_{}, quality_{};
  int quality_source_{0}; bool quality_high_integrity_{false};
  bool initialized_{false}, last_publisher_ok_{false}, last_transport_ok_{false};
  std::string last_status_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr quality_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_ok_pub_, transport_ok_pub_, localization_inputs_ok_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_status_pub_, transport_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorContractMonitor>());
  rclcpp::shutdown();
  return 0;
}
