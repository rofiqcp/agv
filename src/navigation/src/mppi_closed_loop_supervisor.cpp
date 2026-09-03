#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>

using namespace std::chrono_literals;

namespace {

rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

bool finiteTwist(const geometry_msgs::msg::Twist & msg)
{
  return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
         std::isfinite(msg.angular.z);
}

struct TimedScalar
{
  rclcpp::Time stamp;
  double value{0.0};
};

}  // namespace

/*
 * MPPI / velocity-smoother feedback qualification supervisor.
 *
 * Nav2 MPPI already consumes /odometry/filtered in controller_server. This node
 * does not create a second controller. It continuously verifies that the real
 * actuator -> ESC -> local EKF feedback loop is fresh and sufficiently stable,
 * and publishes an evidence-based recommendation for whether Nav2
 * velocity_smoother CLOSED_LOOP is eligible for a field trial.
 *
 * CLOSED_LOOP eligibility is deliberately diagnostic only. Changing
 * velocity_smoother.feedback still requires YAML update + lifecycle restart on
 * the Humble deployment, avoiding an unsafe runtime flip while the vehicle moves.
 */
class MppiClosedLoopSupervisor : public rclcpp::Node
{
public:
  MppiClosedLoopSupervisor()
  : Node("mppi_closed_loop_supervisor")
  {
    feedback_timeout_sec_ = std::max(0.05, declare_parameter<double>("feedback_timeout_sec", 0.30));
    command_timeout_sec_ = std::max(0.05, declare_parameter<double>("command_timeout_sec", 0.60));
    enable_hold_sec_ = std::max(0.0, declare_parameter<double>("enable_hold_sec", 0.75));
    disable_hold_sec_ = std::max(0.0, declare_parameter<double>("disable_hold_sec", 0.20));
    status_rate_hz_ = std::clamp(declare_parameter<double>("status_rate_hz", 5.0), 1.0, 20.0);

    qualification_window_sec_ = std::clamp(
      declare_parameter<double>("qualification_window_sec", 12.0), 3.0, 120.0);
    const std::int64_t min_qualification_samples_param =
      declare_parameter<std::int64_t>("min_qualification_samples", 60);
    min_qualification_samples_ = static_cast<int>(std::clamp<std::int64_t>(
      min_qualification_samples_param, std::int64_t{10}, std::int64_t{5000}));
    min_odom_rate_hz_ = std::clamp(
      declare_parameter<double>("min_odom_rate_hz", 20.0), 1.0, 200.0);
    max_odom_jitter_sec_ = std::clamp(
      declare_parameter<double>("max_odom_jitter_sec", 0.020), 0.001, 1.0);
    max_velocity_rmse_mps_ = std::clamp(
      declare_parameter<double>("max_velocity_rmse_mps", 0.08), 0.001, 5.0);
    max_yaw_rate_rmse_rps_ = std::clamp(
      declare_parameter<double>("max_yaw_rate_rmse_rps", 0.15), 0.001, 10.0);
    max_steering_rmse_rad_ = std::clamp(
      declare_parameter<double>("max_steering_rmse_rad", 0.12), 0.001, 3.2);

    const auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    const auto feedback_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    esc_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/esc/ready", stateQos(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        esc_ready_ = msg->data;
        esc_ready_rx_ = now();
      });

    esc_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/esc/odom", feedback_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!finiteTwist(msg->twist.twist)) {
          return;
        }
        esc_odom_rx_ = now();
      });

    filtered_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", feedback_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!finiteTwist(msg->twist.twist)) {
          return;
        }
        const auto t = now();
        if (filtered_odom_rx_.nanoseconds() != 0) {
          const double dt = (t - filtered_odom_rx_).seconds();
          if (dt > 0.0 && dt < 2.0) {
            odom_intervals_.push_back({t, dt});
          }
        }
        filtered_odom_ = *msg;
        filtered_odom_rx_ = t;
      });

    mppi_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_nav_raw", cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!finiteTwist(*msg)) {
          return;
        }
        mppi_cmd_ = *msg;
        mppi_cmd_rx_ = now();
      });

    actuator_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel/actuator", cmd_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        if (!finiteTwist(*msg)) {
          return;
        }
        actuator_cmd_ = *msg;
        actuator_cmd_rx_ = now();
      });

    drive_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/drive_target_mps", feedback_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) {
          return;
        }
        drive_target_mps_ = msg->data;
        drive_target_rx_ = now();
      });

    drive_actual_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/drive_actual_mps", feedback_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) {
          return;
        }
        drive_actual_mps_ = msg->data;
        drive_actual_rx_ = now();
      });

    steering_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/steering_target_rad", feedback_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) {
          return;
        }
        steering_target_rad_ = msg->data;
        steering_target_rx_ = now();
      });

    steering_actual_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/steering_actual_rad", feedback_qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        if (!std::isfinite(msg->data)) {
          return;
        }
        steering_actual_rad_ = msg->data;
        steering_actual_rx_ = now();
      });

    closed_loop_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/navigation/mppi_closed_loop/ready", stateQos());
    smoother_eligible_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/navigation/velocity_smoother/closed_loop_eligible", stateQos());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/navigation/mppi_closed_loop/status", stateQos());
    qualification_pub_ = create_publisher<std_msgs::msg::String>(
      "/navigation/velocity_smoother/qualification", stateQos());
    velocity_error_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/navigation/mppi_closed_loop/velocity_error_mps", feedback_qos);
    steering_error_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/navigation/mppi_closed_loop/steering_error_rad", feedback_qos);
    yaw_rate_error_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/navigation/mppi_closed_loop/yaw_rate_error_rps", feedback_qos);

    candidate_state_since_ = now();
    last_status_pub_ = now();
    timer_ = create_wall_timer(100ms, std::bind(&MppiClosedLoopSupervisor::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "MPPI feedback supervisor aktif; CLOSED_LOOP smoother hanya direkomendasikan setelah qualification window lulus");
  }

private:
  bool fresh(const rclcpp::Time & stamp, double timeout_sec) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    const double a = (now() - stamp).seconds();
    return a >= 0.0 && a <= timeout_sec;
  }

  double age(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, (now() - stamp).seconds());
  }

  bool rawFeedbackReady() const
  {
    return esc_ready_ && fresh(esc_ready_rx_, feedback_timeout_sec_ * 2.0) &&
           fresh(esc_odom_rx_, feedback_timeout_sec_) &&
           fresh(filtered_odom_rx_, feedback_timeout_sec_) &&
           fresh(drive_actual_rx_, feedback_timeout_sec_) &&
           fresh(steering_actual_rx_, feedback_timeout_sec_);
  }

  bool stableFeedbackReady(bool raw_ready)
  {
    const auto t = now();
    if (raw_ready != candidate_feedback_ready_) {
      candidate_feedback_ready_ = raw_ready;
      candidate_state_since_ = t;
    }
    const double held = std::max(0.0, (t - candidate_state_since_).seconds());
    const double required = raw_ready ? enable_hold_sec_ : disable_hold_sec_;
    if (held >= required) {
      stable_feedback_ready_ = raw_ready;
    }
    return stable_feedback_ready_;
  }

  void prune(std::deque<TimedScalar> & values, const rclcpp::Time & t)
  {
    while (!values.empty() && (t - values.front().stamp).seconds() > qualification_window_sec_) {
      values.pop_front();
    }
  }

  static double mean(const std::deque<TimedScalar> & values)
  {
    if (values.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    double sum = 0.0;
    for (const auto & v : values) {
      sum += v.value;
    }
    return sum / static_cast<double>(values.size());
  }

  static double rms(const std::deque<TimedScalar> & values)
  {
    if (values.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    double sum = 0.0;
    for (const auto & v : values) {
      sum += v.value * v.value;
    }
    return std::sqrt(sum / static_cast<double>(values.size()));
  }

  static double stddev(const std::deque<TimedScalar> & values)
  {
    if (values.size() < 2) {
      return std::numeric_limits<double>::infinity();
    }
    const double m = mean(values);
    double sum = 0.0;
    for (const auto & v : values) {
      const double d = v.value - m;
      sum += d * d;
    }
    return std::sqrt(sum / static_cast<double>(values.size() - 1));
  }

  void sampleErrors(const rclcpp::Time & t)
  {
    const bool target_fresh = fresh(drive_target_rx_, command_timeout_sec_) &&
                              fresh(steering_target_rx_, command_timeout_sec_);
    const bool actual_fresh = fresh(drive_actual_rx_, feedback_timeout_sec_) &&
                              fresh(steering_actual_rx_, feedback_timeout_sec_);

    if (target_fresh && actual_fresh) {
      velocity_errors_.push_back({t, drive_target_mps_ - drive_actual_mps_});
      steering_errors_.push_back({t, steering_target_rad_ - steering_actual_rad_});
    }
    if (fresh(actuator_cmd_rx_, command_timeout_sec_) && fresh(filtered_odom_rx_, feedback_timeout_sec_)) {
      yaw_errors_.push_back({t, actuator_cmd_.angular.z - filtered_odom_.twist.twist.angular.z});
    }

    prune(velocity_errors_, t);
    prune(steering_errors_, t);
    prune(yaw_errors_, t);
    prune(odom_intervals_, t);
  }

  bool smootherEligible(bool feedback_ready, double & odom_rate, double & odom_jitter,
                        double & v_rmse, double & steer_rmse, double & yaw_rmse) const
  {
    const double dt_mean = mean(odom_intervals_);
    odom_rate = std::isfinite(dt_mean) && dt_mean > 1.0e-6 ? 1.0 / dt_mean : 0.0;
    odom_jitter = stddev(odom_intervals_);
    v_rmse = rms(velocity_errors_);
    steer_rmse = rms(steering_errors_);
    yaw_rmse = rms(yaw_errors_);

    const bool enough = static_cast<int>(odom_intervals_.size()) >= min_qualification_samples_ &&
                        static_cast<int>(velocity_errors_.size()) >= min_qualification_samples_ &&
                        static_cast<int>(yaw_errors_.size()) >= min_qualification_samples_;
    return feedback_ready && enough &&
           odom_rate >= min_odom_rate_hz_ &&
           std::isfinite(odom_jitter) && odom_jitter <= max_odom_jitter_sec_ &&
           std::isfinite(v_rmse) && v_rmse <= max_velocity_rmse_mps_ &&
           std::isfinite(steer_rmse) && steer_rmse <= max_steering_rmse_rad_ &&
           std::isfinite(yaw_rmse) && yaw_rmse <= max_yaw_rate_rmse_rps_;
  }

  void publishDiagnostics(bool feedback_ready)
  {
    const auto t = now();
    sampleErrors(t);

    std_msgs::msg::Bool ready;
    ready.data = feedback_ready;
    closed_loop_ready_pub_->publish(ready);

    const bool target_fresh = fresh(drive_target_rx_, command_timeout_sec_) &&
                              fresh(steering_target_rx_, command_timeout_sec_);
    const bool actual_fresh = fresh(drive_actual_rx_, feedback_timeout_sec_) &&
                              fresh(steering_actual_rx_, feedback_timeout_sec_);

    if (target_fresh && actual_fresh) {
      std_msgs::msg::Float64 velocity_error;
      velocity_error.data = drive_target_mps_ - drive_actual_mps_;
      velocity_error_pub_->publish(velocity_error);

      std_msgs::msg::Float64 steering_error;
      steering_error.data = steering_target_rad_ - steering_actual_rad_;
      steering_error_pub_->publish(steering_error);
    }

    if (fresh(actuator_cmd_rx_, command_timeout_sec_) && fresh(filtered_odom_rx_, feedback_timeout_sec_)) {
      std_msgs::msg::Float64 yaw_error;
      yaw_error.data = actuator_cmd_.angular.z - filtered_odom_.twist.twist.angular.z;
      yaw_rate_error_pub_->publish(yaw_error);
    }

    double odom_rate = 0.0;
    double odom_jitter = std::numeric_limits<double>::infinity();
    double v_rmse = std::numeric_limits<double>::quiet_NaN();
    double steer_rmse = std::numeric_limits<double>::quiet_NaN();
    double yaw_rmse = std::numeric_limits<double>::quiet_NaN();
    const bool eligible = smootherEligible(
      feedback_ready, odom_rate, odom_jitter, v_rmse, steer_rmse, yaw_rmse);

    std_msgs::msg::Bool eligible_msg;
    eligible_msg.data = eligible;
    smoother_eligible_pub_->publish(eligible_msg);

    const double status_period = 1.0 / status_rate_hz_;
    if ((t - last_status_pub_).seconds() < status_period) {
      return;
    }
    last_status_pub_ = t;

    std::ostringstream qualification;
    qualification << std::boolalpha << std::fixed << std::setprecision(4)
                  << "eligible=" << eligible
                  << ";window_sec=" << qualification_window_sec_
                  << ";samples=" << odom_intervals_.size()
                  << ";odom_rate_hz=" << odom_rate
                  << ";odom_jitter_sec=" << odom_jitter
                  << ";velocity_rmse_mps=" << v_rmse
                  << ";steering_rmse_rad=" << steer_rmse
                  << ";yaw_rate_rmse_rps=" << yaw_rmse
                  << ";min_rate_hz=" << min_odom_rate_hz_
                  << ";max_jitter_sec=" << max_odom_jitter_sec_
                  << ";max_velocity_rmse_mps=" << max_velocity_rmse_mps_
                  << ";max_steering_rmse_rad=" << max_steering_rmse_rad_
                  << ";max_yaw_rate_rmse_rps=" << max_yaw_rate_rmse_rps_;
    std_msgs::msg::String qualification_msg;
    qualification_msg.data = qualification.str();
    qualification_pub_->publish(qualification_msg);

    std_msgs::msg::String status;
    std::ostringstream stream;
    stream << std::boolalpha << std::fixed << std::setprecision(3)
           << "mode=" << (feedback_ready ? "ACTUAL_FEEDBACK" : "COMMISSIONING")
           << ";ready=" << feedback_ready
           << ";smoother_closed_loop_eligible=" << eligible
           << ";esc_ready=" << esc_ready_
           << ";esc_odom_age=" << age(esc_odom_rx_)
           << ";ekf_odom_age=" << age(filtered_odom_rx_)
           << ";mppi_age=" << age(mppi_cmd_rx_)
           << ";actuator_age=" << age(actuator_cmd_rx_)
           << ";odom_rate_hz=" << odom_rate
           << ";odom_jitter_sec=" << odom_jitter
           << ";velocity_rmse_mps=" << v_rmse
           << ";steering_rmse_rad=" << steer_rmse
           << ";yaw_rate_rmse_rps=" << yaw_rmse
           << ";mppi_v=" << (fresh(mppi_cmd_rx_, command_timeout_sec_) ? std::to_string(mppi_cmd_.linear.x) : "N/A")
           << ";mppi_w=" << (fresh(mppi_cmd_rx_, command_timeout_sec_) ? std::to_string(mppi_cmd_.angular.z) : "N/A")
           << ";act_v=" << (fresh(actuator_cmd_rx_, command_timeout_sec_) ? std::to_string(actuator_cmd_.linear.x) : "N/A")
           << ";act_w=" << (fresh(actuator_cmd_rx_, command_timeout_sec_) ? std::to_string(actuator_cmd_.angular.z) : "N/A")
           << ";v_target=" << (target_fresh ? std::to_string(drive_target_mps_) : "N/A")
           << ";v_actual=" << (actual_fresh ? std::to_string(drive_actual_mps_) : "N/A")
           << ";steer_target=" << (target_fresh ? std::to_string(steering_target_rad_) : "N/A")
           << ";steer_actual=" << (actual_fresh ? std::to_string(steering_actual_rad_) : "N/A")
           << ";yaw_actual=" << (fresh(filtered_odom_rx_, feedback_timeout_sec_) ? std::to_string(filtered_odom_.twist.twist.angular.z) : "N/A");
    status.data = stream.str();
    status_pub_->publish(status);
  }

  void tick()
  {
    const bool feedback_ready = stableFeedbackReady(rawFeedbackReady());
    publishDiagnostics(feedback_ready);
  }

  double feedback_timeout_sec_{0.30};
  double command_timeout_sec_{0.60};
  double enable_hold_sec_{0.75};
  double disable_hold_sec_{0.20};
  double status_rate_hz_{5.0};
  double qualification_window_sec_{12.0};
  int min_qualification_samples_{60};
  double min_odom_rate_hz_{20.0};
  double max_odom_jitter_sec_{0.020};
  double max_velocity_rmse_mps_{0.08};
  double max_yaw_rate_rmse_rps_{0.15};
  double max_steering_rmse_rad_{0.12};

  bool esc_ready_{false};
  bool candidate_feedback_ready_{false};
  bool stable_feedback_ready_{false};

  nav_msgs::msg::Odometry filtered_odom_{};
  geometry_msgs::msg::Twist mppi_cmd_{};
  geometry_msgs::msg::Twist actuator_cmd_{};
  double drive_target_mps_{0.0};
  double drive_actual_mps_{0.0};
  double steering_target_rad_{0.0};
  double steering_actual_rad_{0.0};

  std::deque<TimedScalar> odom_intervals_;
  std::deque<TimedScalar> velocity_errors_;
  std::deque<TimedScalar> steering_errors_;
  std::deque<TimedScalar> yaw_errors_;

  rclcpp::Time esc_ready_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time esc_odom_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time filtered_odom_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time mppi_cmd_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time actuator_cmd_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time drive_target_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time drive_actual_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time steering_target_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time steering_actual_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time candidate_state_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_status_pub_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr esc_ready_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr esc_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mppi_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr actuator_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drive_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr drive_actual_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_actual_sub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr closed_loop_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr smoother_eligible_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qualification_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_rate_error_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MppiClosedLoopSupervisor>());
  rclcpp::shutdown();
  return 0;
}
