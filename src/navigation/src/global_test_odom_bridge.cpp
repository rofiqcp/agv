// global_test_odom_bridge.cpp
// Single owner of dynamic TF for global field test:
//   map -> odom = identity
//   odom -> base_footprint = global filtered pose
// Also republishes /odometry/filtered for Nav2 compatibility.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

using namespace std::chrono_literals;

class GlobalTestOdomBridge final : public rclcpp::Node
{
public:
  explicit GlobalTestOdomBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("global_test_odom_bridge", options),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    filtered_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/global_test/filtered_map", rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&GlobalTestOdomBridge::onFiltered, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);

    // Re-publish at stable rate so TF and /odometry/filtered never go stale.
    publish_timer_ = create_wall_timer(
      50ms, std::bind(&GlobalTestOdomBridge::onTimer, this));
    // Refuse to own the test TF tree beside known production/actuation owners.
    safety_timer_ = create_wall_timer(
      500ms, std::bind(&GlobalTestOdomBridge::checkUnsafeRuntime, this));

    RCLCPP_INFO(get_logger(), "GlobalTestOdomBridge active: map→odom=identity, odom→base_footprint=global filtered");
  }

private:
  void checkUnsafeRuntime()
  {
    const auto names = get_node_names();
    static const std::vector<std::string> forbidden{
      "localization_core", "ekf_filter_node_odom", "esc_ackermann",
      "motor_teleop", "navigation_core", "mppi_closed_loop_supervisor",
      "trajectory_safety_supervisor"};
    for (const auto & name : forbidden) {
      if (std::find(names.begin(), names.end(), name) != names.end()) {
        RCLCPP_FATAL(
          get_logger(),
          "UNSAFE GLOBAL_FIELD_TEST: production/actuation node '%s' detected; shutting down test bridge",
          name.c_str());
        rclcpp::shutdown();
        return;
      }
    }
    for (const std::string topic : {"/cmd_vel", "/cmd_vel/actuator"}) {
      if (!get_publishers_info_by_topic(topic).empty()) {
        RCLCPP_FATAL(
          get_logger(),
          "UNSAFE GLOBAL_FIELD_TEST: hardware command publisher detected on %s; shutting down test bridge",
          topic.c_str());
        rclcpp::shutdown();
        return;
      }
    }
  }

  void onFiltered(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_filtered_ = *msg;
    have_filtered_ = true;
    last_filtered_time_ = now();
  }

  void onTimer()
  {
    if (!have_filtered_) return;

    std::lock_guard<std::mutex> lock(mutex_);
    const auto stamp = now();

    // map → odom = identity
    geometry_msgs::msg::TransformStamped tf_map_odom;
    tf_map_odom.header.stamp = stamp;
    tf_map_odom.header.frame_id = "map";
    tf_map_odom.child_frame_id = "odom";
    tf_map_odom.transform.translation.x = 0.0;
    tf_map_odom.transform.translation.y = 0.0;
    tf_map_odom.transform.translation.z = 0.0;
    tf_map_odom.transform.rotation.x = 0.0;
    tf_map_odom.transform.rotation.y = 0.0;
    tf_map_odom.transform.rotation.z = 0.0;
    tf_map_odom.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_map_odom);

    // odom → base_footprint = global filtered pose (since map=odom on test)
    geometry_msgs::msg::TransformStamped tf_odom_base;
    tf_odom_base.header.stamp = stamp;
    tf_odom_base.header.frame_id = "odom";
    tf_odom_base.child_frame_id = "base_footprint";
    tf_odom_base.transform.translation.x = last_filtered_.pose.pose.position.x;
    tf_odom_base.transform.translation.y = last_filtered_.pose.pose.position.y;
    tf_odom_base.transform.translation.z = last_filtered_.pose.pose.position.z;
    tf_odom_base.transform.rotation = last_filtered_.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_odom_base);

    // /odometry/filtered compatibility topic
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom.pose = last_filtered_.pose;
    odom.twist = last_filtered_.twist;  // meaningful if EKF produces twist
    if (!std::isfinite(odom.twist.twist.linear.x)) {
      odom.twist.twist.linear.x = 0.0;
    }
    if (!std::isfinite(odom.twist.twist.angular.z)) {
      odom.twist.twist.angular.z = 0.0;
    }
    odom_pub_->publish(odom);
  }

  std::mutex mutex_;
  nav_msgs::msg::Odometry last_filtered_;
  bool have_filtered_{false};
  rclcpp::Time last_filtered_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalTestOdomBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
