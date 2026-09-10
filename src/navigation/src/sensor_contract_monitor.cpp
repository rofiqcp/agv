#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
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
    ok_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/system/sensor_publishers_ok", rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/sensor_publishers_status", rclcpp::QoS(1).reliable().transient_local());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / check_rate_hz_)),
      std::bind(&SensorContractMonitor::tick, this));
  }

private:
  void tick()
  {
    bool all_ok = !topics_.empty();
    std::ostringstream out;
    out << "{\"ok\":";
    std::vector<std::string> details;
    details.reserve(topics_.size());
    for (const auto & topic : topics_) {
      const auto infos = get_publishers_info_by_topic(topic);
      const bool topic_ok = infos.size() == 1U;
      all_ok = all_ok && topic_ok;
      std::ostringstream item;
      item << "{\"topic\":\"" << topic << "\",\"count\":" << infos.size() << ",\"nodes\":[";
      for (size_t i = 0; i < infos.size(); ++i) {
        if (i) item << ',';
        item << "\"" << infos[i].node_namespace() << infos[i].node_name() << "\"";
      }
      item << "]}";
      details.push_back(item.str());
    }
    out << (all_ok ? "true" : "false") << ",\"topics\":[";
    for (size_t i = 0; i < details.size(); ++i) {
      if (i) out << ',';
      out << details[i];
    }
    out << "]}";

    std_msgs::msg::Bool ok;
    ok.data = all_ok;
    ok_pub_->publish(ok);
    if (!initialized_ || all_ok != last_ok_ || out.str() != last_status_) {
      std_msgs::msg::String status;
      status.data = out.str();
      status_pub_->publish(status);
      if (initialized_ && last_ok_ && !all_ok) {
        RCLCPP_ERROR(get_logger(), "Critical sensor publisher contract LOST: %s", status.data.c_str());
      } else if (!initialized_ || (!last_ok_ && all_ok)) {
        RCLCPP_INFO(get_logger(), "Critical sensor publisher contract %s: %s",
          all_ok ? "READY" : "WAIT", status.data.c_str());
      }
      last_status_ = status.data;
    }
    last_ok_ = all_ok;
    initialized_ = true;
  }

  std::vector<std::string> topics_;
  double check_rate_hz_{5.0};
  bool initialized_{false};
  bool last_ok_{false};
  std::string last_status_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ok_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorContractMonitor>());
  rclcpp::shutdown();
  return 0;
}
