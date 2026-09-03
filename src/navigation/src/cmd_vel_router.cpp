#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

namespace {
rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

bool finiteTwist(const geometry_msgs::msg::Twist & msg)
{
  return std::isfinite(msg.linear.x) && std::isfinite(msg.angular.z);
}
}  // namespace

class CmdVelRouter final : public rclcpp::Node
{
public:
  CmdVelRouter() : Node("cmd_vel_router")
  {
    declare_parameter<std::string>("teleop_topic", "/cmd_vel/teleop");
    declare_parameter<std::string>("teleop_source_topic", "/teleop/active_source");
    declare_parameter<std::string>("teleop_estop_topic", "/teleop/emergency_stop_latched");
    declare_parameter<std::string>("autonomy_topic", "/cmd_vel/autonomy_pre_smoother");
    declare_parameter<std::string>("autonomy_gate_topic", "/system/autonomy_motion_allowed");
    declare_parameter<std::string>("global_estop_topic", "/safety/estop");
    declare_parameter<std::string>("output_topic", "/cmd_vel/pre_smoother");
    declare_parameter<std::string>("source_topic", "/navigation/cmd_mux/source");
    declare_parameter<double>("rate_hz", 50.0);
    declare_parameter<double>("teleop_timeout_sec", 0.30);
    declare_parameter<double>("autonomy_timeout_sec", 0.60);
    declare_parameter<double>("manual_release_hold_sec", 0.50);

    const auto teleop_topic = get_parameter("teleop_topic").as_string();
    const auto teleop_source_topic = get_parameter("teleop_source_topic").as_string();
    const auto teleop_estop_topic = get_parameter("teleop_estop_topic").as_string();
    const auto autonomy_topic = get_parameter("autonomy_topic").as_string();
    const auto autonomy_gate_topic = get_parameter("autonomy_gate_topic").as_string();
    const auto global_estop_topic = get_parameter("global_estop_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    const auto source_topic = get_parameter("source_topic").as_string();
    teleop_timeout_sec_ = std::clamp(get_parameter("teleop_timeout_sec").as_double(), 0.05, 2.0);
    autonomy_timeout_sec_ = std::clamp(get_parameter("autonomy_timeout_sec").as_double(), 0.05, 2.0);
    manual_release_hold_sec_ = std::clamp(get_parameter("manual_release_hold_sec").as_double(), 0.0, 2.0);
    const double rate_hz = std::clamp(get_parameter("rate_hz").as_double(), 10.0, 100.0);

    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      teleop_topic, cmd_qos, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        teleop_valid_ = finiteTwist(*msg);
        if (teleop_valid_) {
          teleop_cmd_ = *msg;
          teleop_stamp_ = now();
        }
      });
    autonomy_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      autonomy_topic, cmd_qos, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        autonomy_valid_ = finiteTwist(*msg);
        if (autonomy_valid_) {
          autonomy_cmd_ = *msg;
          autonomy_stamp_ = now();
        }
      });
    teleop_source_sub_ = create_subscription<std_msgs::msg::String>(
      teleop_source_topic, stateQos(), [this](std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        teleop_source_ = msg->data;
        teleop_source_stamp_ = now();
        const bool active = !msg->data.empty() && msg->data != "STOP" &&
          msg->data != "IDLE" && msg->data != "E_STOP";
        if (active) {
          teleop_hold_until_ = now() + rclcpp::Duration::from_seconds(manual_release_hold_sec_);
        }
      });
    teleop_estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      teleop_estop_topic, stateQos(), [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        teleop_estop_ = msg->data;
      });
    autonomy_gate_sub_ = create_subscription<std_msgs::msg::Bool>(
      autonomy_gate_topic, stateQos(), [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        autonomy_gate_ = msg->data;
        autonomy_gate_seen_ = true;
      });
    global_estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      global_estop_topic, stateQos(), [this](std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        global_estop_ = msg->data;
      });

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, cmd_qos);
    source_pub_ = create_publisher<std_msgs::msg::String>(source_topic, stateQos());
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, std::bind(&CmdVelRouter::tick, this));
    RCLCPP_INFO(
      get_logger(),
      "cmd_vel_router ready: TELEOP/AUTONOMY -> %s -> velocity_smoother -> /cmd_vel -> ESC",
      output_topic.c_str());
  }

private:
  bool fresh(const rclcpp::Time & stamp, const rclcpp::Time & t, double timeout) const
  {
    if (stamp.nanoseconds() <= 0) return false;
    const double age = (t - stamp).seconds();
    return age >= 0.0 && age <= timeout;
  }

  void tick()
  {
    geometry_msgs::msg::Twist out;
    std::string source = "IDLE";
    const auto t = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool estop = global_estop_ || teleop_estop_ ||
        (fresh(teleop_source_stamp_, t, teleop_timeout_sec_) && teleop_source_ == "E_STOP");
      const bool source_fresh = fresh(teleop_source_stamp_, t, teleop_timeout_sec_);
      const bool teleop_active = source_fresh && !teleop_source_.empty() &&
        teleop_source_ != "STOP" && teleop_source_ != "IDLE" && teleop_source_ != "E_STOP";
      const bool teleop_hold = teleop_hold_until_.nanoseconds() > 0 && t <= teleop_hold_until_;
      const bool teleop_fresh = teleop_valid_ && fresh(teleop_stamp_, t, teleop_timeout_sec_);
      const bool auto_fresh = autonomy_valid_ && fresh(autonomy_stamp_, t, autonomy_timeout_sec_);
      if (estop) {
        source = "E_STOP";
      } else if (teleop_fresh && (teleop_active || teleop_hold)) {
        out = teleop_cmd_;
        source = teleop_active ? "TELEOP" : "TELEOP_RELEASE_HOLD";
      } else if (auto_fresh && autonomy_gate_seen_ && autonomy_gate_) {
        out = autonomy_cmd_;
        source = "AUTONOMY";
      } else if (!autonomy_gate_seen_ || !autonomy_gate_) {
        source = "AUTONOMY_GATE_CLOSED";
      }
    }
    output_pub_->publish(out);
    std_msgs::msg::String msg;
    msg.data = source;
    source_pub_->publish(msg);
  }

  std::mutex mutex_;
  geometry_msgs::msg::Twist teleop_cmd_{};
  geometry_msgs::msg::Twist autonomy_cmd_{};
  rclcpp::Time teleop_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time autonomy_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time teleop_source_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time teleop_hold_until_{0, 0, RCL_ROS_TIME};
  bool teleop_valid_{false};
  bool autonomy_valid_{false};
  bool teleop_estop_{false};
  bool global_estop_{false};
  bool autonomy_gate_{false};
  bool autonomy_gate_seen_{false};
  std::string teleop_source_;
  double teleop_timeout_sec_{0.30};
  double autonomy_timeout_sec_{0.60};
  double manual_release_hold_sec_{0.50};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr autonomy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teleop_source_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teleop_estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr autonomy_gate_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr global_estop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelRouter>());
  rclcpp::shutdown();
  return 0;
}
