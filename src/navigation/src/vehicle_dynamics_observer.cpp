#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double stampSec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double finiteVariance(double value, double fallback)
{
  return std::isfinite(value) && value >= 0.0 ? value : fallback;
}
}  // namespace

class VehicleDynamicsObserver final : public rclcpp::Node
{
public:
  VehicleDynamicsObserver() : Node("vehicle_dynamics_observer")
  {
    wheelbase_m_ = std::max(0.05, declare_parameter<double>("wheelbase_m", 0.70));
    update_rate_hz_ = std::clamp(declare_parameter<double>("update_rate_hz", 50.0), 10.0, 100.0);
    message_timeout_sec_ = std::clamp(declare_parameter<double>("message_timeout_sec", 0.30), 0.05, 2.0);
    sync_max_gap_sec_ = std::clamp(declare_parameter<double>("sync_max_gap_sec", 0.08), 0.005, 0.50);
    predictor_tau_sec_ = std::clamp(declare_parameter<double>("steering_predictor_tau_sec", 0.12), 0.01, 2.0);
    predictor_delay_sec_ = std::clamp(declare_parameter<double>("steering_predictor_delay_sec", 0.04), 0.0, 0.50);
    offset_estimator_enabled_ = declare_parameter<bool>("offset_estimator_enabled", true);
    offset_process_noise_ = std::max(1.0e-12, declare_parameter<double>("offset_process_noise_rad2", 1.0e-7));
    offset_measurement_noise_ = std::max(1.0e-9, declare_parameter<double>("offset_measurement_noise_rps2", 2.5e-3));
    offset_covariance_ = std::max(1.0e-9, declare_parameter<double>("offset_initial_covariance_rad2", 0.01));
    offset_limit_rad_ = std::clamp(declare_parameter<double>("offset_limit_deg", 5.0), 0.1, 20.0) * kPi / 180.0;
    min_velocity_mps_ = std::max(0.01, declare_parameter<double>("offset_min_velocity_mps", 0.12));
    max_yaw_rate_rps_ = std::max(0.01, declare_parameter<double>("offset_max_yaw_rate_rps", 0.35));
    max_steer_rad_ = std::clamp(declare_parameter<double>("offset_max_steer_deg", 12.0), 1.0, 45.0) * kPi / 180.0;
    max_steer_rate_rps_ = std::clamp(declare_parameter<double>("offset_max_steer_rate_deg_s", 20.0), 1.0, 180.0) * kPi / 180.0;
    fused_v_variance_fallback_ = std::max(1.0e-6, declare_parameter<double>("fused_v_variance_fallback", 0.03));
    fused_w_variance_fallback_ = std::max(1.0e-6, declare_parameter<double>("fused_w_variance_fallback", 0.02));

    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/esc/odom", sensor_qos, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdom(*msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", sensor_qos, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { onImu(*msg); });
    steer_target_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/steering_target_rad", sensor_qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) { onSteeringTarget(msg->data); });
    steer_actual_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/esc/steering_actual_rad", sensor_qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) { onSteeringActual(msg->data); });

    fused_twist_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/vehicle/twist_fused", 20);
    steering_predicted_pub_ = create_publisher<std_msgs::msg::Float64>("/precision/steering_predicted_rad", 20);
    steering_offset_pub_ = create_publisher<std_msgs::msg::Float64>("/precision/steering_offset_rad", 20);
    yaw_innovation_pub_ = create_publisher<std_msgs::msg::Float64>("/precision/yaw_rate_innovation_rps", 20);
    sync_gap_pub_ = create_publisher<std_msgs::msg::Float64>("/precision/twist_sync_gap_sec", 20);
    status_pub_ = create_publisher<std_msgs::msg::String>("/precision/dynamics_status", rclcpp::QoS(1).reliable());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / update_rate_hz_));
    timer_ = create_wall_timer(period, std::bind(&VehicleDynamicsObserver::tick, this));
    last_tick_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Vehicle dynamics observer active: ESC vx + IMU wz fusion, steering predictor, offset/innovation observer | %.1f Hz",
      update_rate_hz_);
  }

private:
  struct OdomSample { nav_msgs::msg::Odometry msg; rclcpp::Time received{0, 0, RCL_ROS_TIME}; bool valid{false}; };
  struct ImuSample { sensor_msgs::msg::Imu msg; rclcpp::Time received{0, 0, RCL_ROS_TIME}; bool valid{false}; };
  struct ScalarSample { double value{0.0}; rclcpp::Time received{0, 0, RCL_ROS_TIME}; bool valid{false}; };
  struct CommandSample { rclcpp::Time stamp{0, 0, RCL_ROS_TIME}; double value{0.0}; };

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    if (!std::isfinite(msg.twist.twist.linear.x)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    odom_.msg = msg; odom_.received = now(); odom_.valid = true;
  }

  void onImu(const sensor_msgs::msg::Imu & msg)
  {
    if (!std::isfinite(msg.angular_velocity.z)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    imu_.msg = msg; imu_.received = now(); imu_.valid = true;
  }

  void onSteeringTarget(double value)
  {
    if (!std::isfinite(value)) return;
    const auto t = now();
    std::lock_guard<std::mutex> lock(mutex_);
    steer_target_.value = value; steer_target_.received = t; steer_target_.valid = true;
    command_history_.push_back({t, value});
    while (command_history_.size() > 256U ||
           (!command_history_.empty() && (t - command_history_.front().stamp).seconds() > 2.0)) {
      command_history_.pop_front();
    }
  }

  void onSteeringActual(double value)
  {
    if (!std::isfinite(value)) return;
    const auto t = now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (steer_actual_.valid) {
      const double dt = (t - steer_actual_.received).seconds();
      if (dt > 1.0e-4) steering_rate_rps_ = (value - steer_actual_.value) / dt;
    }
    steer_actual_.value = value; steer_actual_.received = t; steer_actual_.valid = true;
  }

  static bool fresh(const rclcpp::Time & now_time, const rclcpp::Time & received, double timeout)
  {
    if (received.nanoseconds() <= 0) return false;
    const double age = (now_time - received).seconds();
    return age >= 0.0 && age <= timeout;
  }

  double delayedSteeringCommand(const rclcpp::Time & t) const
  {
    if (command_history_.empty()) return steer_target_.valid ? steer_target_.value : 0.0;
    const rclcpp::Time delayed = t - rclcpp::Duration::from_seconds(predictor_delay_sec_);
    double value = command_history_.front().value;
    for (const auto & sample : command_history_) {
      if (sample.stamp > delayed) break;
      value = sample.value;
    }
    return value;
  }

  void publishFusedTwist(
    const OdomSample & odom, const ImuSample & imu, double sync_gap, const rclcpp::Time & now_time)
  {
    geometry_msgs::msg::TwistWithCovarianceStamped out;
    const double odom_stamp = stampSec(odom.msg.header.stamp);
    const double imu_stamp = stampSec(imu.msg.header.stamp);
    out.header.stamp = odom_stamp >= imu_stamp ? odom.msg.header.stamp : imu.msg.header.stamp;
    if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) out.header.stamp = now_time;
    out.header.frame_id = "base_footprint";
    out.twist.twist.linear.x = odom.msg.twist.twist.linear.x;
    out.twist.twist.angular.z = imu.msg.angular_velocity.z;
    out.twist.covariance.fill(0.0);
    out.twist.covariance[0] = finiteVariance(odom.msg.twist.covariance[0], fused_v_variance_fallback_);
    out.twist.covariance[7] = 1.0e5;
    out.twist.covariance[14] = 1.0e5;
    out.twist.covariance[21] = 1.0e5;
    out.twist.covariance[28] = 1.0e5;
    out.twist.covariance[35] = finiteVariance(imu.msg.angular_velocity_covariance[8], fused_w_variance_fallback_);
    fused_twist_pub_->publish(out);

    std_msgs::msg::Float64 gap;
    gap.data = sync_gap;
    sync_gap_pub_->publish(gap);
  }

  void updateOffsetEstimator(double v, double wz, double steering)
  {
    if (!offset_estimator_enabled_) return;
    if (std::abs(v) < min_velocity_mps_ || std::abs(wz) > max_yaw_rate_rps_ ||
        std::abs(steering) > max_steer_rad_ || std::abs(steering_rate_rps_) > max_steer_rate_rps_) return;

    // Vehicle convention is +steer RIGHT while REP-103 +wz is LEFT, hence phi<0.
    const double phi = -v / wheelbase_m_;
    const double y = wz - phi * steering;
    const double p_prior = offset_covariance_ + offset_process_noise_;
    const double denom = std::max(offset_measurement_noise_ + phi * phi * p_prior, 1.0e-12);
    const double gain = p_prior * phi / denom;
    const double residual = y - phi * steering_offset_rad_;
    steering_offset_rad_ = std::clamp(steering_offset_rad_ + gain * residual, -offset_limit_rad_, offset_limit_rad_);
    offset_covariance_ = std::max(p_prior - (p_prior * phi * phi * p_prior) / denom, 1.0e-12);
    offset_gain_ = gain;
    offset_residual_ = residual;
    offset_updated_ = true;
  }

  void tick()
  {
    const auto t = now();
    OdomSample odom; ImuSample imu; ScalarSample target; ScalarSample actual;
    double steer_rate = 0.0; double delayed_cmd = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      odom = odom_; imu = imu_; target = steer_target_; actual = steer_actual_;
      steer_rate = steering_rate_rps_;
      delayed_cmd = delayedSteeringCommand(t);
    }

    const double dt = std::clamp((t - last_tick_).seconds(), 1.0e-4, 0.2);
    last_tick_ = t;
    const double alpha = 1.0 - std::exp(-dt / predictor_tau_sec_);
    steering_predicted_rad_ += alpha * (delayed_cmd - steering_predicted_rad_);

    const bool odom_fresh = odom.valid && fresh(t, odom.received, message_timeout_sec_);
    const bool imu_fresh = imu.valid && fresh(t, imu.received, message_timeout_sec_);
    const bool actual_fresh = actual.valid && fresh(t, actual.received, message_timeout_sec_);
    const double os = odom.valid ? stampSec(odom.msg.header.stamp) : 0.0;
    const double is = imu.valid ? stampSec(imu.msg.header.stamp) : 0.0;
    const double sync_gap = (os > 0.0 && is > 0.0) ? std::abs(os - is) : 1.0e9;
    const bool sync_ok = odom_fresh && imu_fresh && sync_gap <= sync_max_gap_sec_;

    double yaw_model = 0.0;
    double innovation = 0.0;
    double innovation_sigma = 0.0;
    offset_updated_ = false;
    if (sync_ok && actual_fresh) {
      const double v = odom.msg.twist.twist.linear.x;
      const double wz = imu.msg.angular_velocity.z;
      updateOffsetEstimator(v, wz, actual.value);
      const double effective_steer = actual.value + steering_offset_rad_;
      yaw_model = -v * std::tan(effective_steer) / wheelbase_m_;
      innovation = wz - yaw_model;
      const double var_w = finiteVariance(imu.msg.angular_velocity_covariance[8], fused_w_variance_fallback_);
      const double var_model = finiteVariance(odom.msg.twist.covariance[0], fused_v_variance_fallback_) + 1.0e-4;
      innovation_sigma = std::abs(innovation) / std::sqrt(std::max(var_w + var_model, 1.0e-9));
      publishFusedTwist(odom, imu, sync_gap, t);
    }

    std_msgs::msg::Float64 scalar;
    scalar.data = steering_predicted_rad_; steering_predicted_pub_->publish(scalar);
    scalar.data = steering_offset_rad_; steering_offset_pub_->publish(scalar);
    scalar.data = innovation; yaw_innovation_pub_->publish(scalar);
    if (!sync_ok) { scalar.data = sync_gap; sync_gap_pub_->publish(scalar); }

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6)
       << "state=" << (sync_ok ? "READY" : "WAIT")
       << ";odom_fresh=" << (odom_fresh ? 1 : 0)
       << ";imu_fresh=" << (imu_fresh ? 1 : 0)
       << ";steering_fresh=" << (actual_fresh ? 1 : 0)
       << ";sync_gap_sec=" << (std::isfinite(sync_gap) ? sync_gap : -1.0)
       << ";vx_mps=" << (odom.valid ? odom.msg.twist.twist.linear.x : 0.0)
       << ";wz_rps=" << (imu.valid ? imu.msg.angular_velocity.z : 0.0)
       << ";steering_actual_rad=" << (actual.valid ? actual.value : 0.0)
       << ";steering_target_rad=" << (target.valid ? target.value : 0.0)
       << ";steering_predicted_rad=" << steering_predicted_rad_
       << ";steering_rate_rps=" << steer_rate
       << ";steering_offset_rad=" << steering_offset_rad_
       << ";offset_covariance=" << offset_covariance_
       << ";offset_gain=" << offset_gain_
       << ";offset_residual=" << offset_residual_
       << ";offset_updated=" << (offset_updated_ ? 1 : 0)
       << ";yaw_rate_model_rps=" << yaw_model
       << ";yaw_rate_innovation_rps=" << innovation
       << ";innovation_sigma=" << innovation_sigma;
    std_msgs::msg::String status;
    status.data = ss.str();
    status_pub_->publish(status);
  }

  double wheelbase_m_{0.70};
  double update_rate_hz_{50.0};
  double message_timeout_sec_{0.30};
  double sync_max_gap_sec_{0.08};
  double predictor_tau_sec_{0.12};
  double predictor_delay_sec_{0.04};
  bool offset_estimator_enabled_{true};
  double offset_process_noise_{1.0e-7};
  double offset_measurement_noise_{2.5e-3};
  double offset_covariance_{0.01};
  double offset_limit_rad_{5.0 * kPi / 180.0};
  double min_velocity_mps_{0.12};
  double max_yaw_rate_rps_{0.35};
  double max_steer_rad_{12.0 * kPi / 180.0};
  double max_steer_rate_rps_{20.0 * kPi / 180.0};
  double fused_v_variance_fallback_{0.03};
  double fused_w_variance_fallback_{0.02};

  std::mutex mutex_;
  OdomSample odom_;
  ImuSample imu_;
  ScalarSample steer_target_;
  ScalarSample steer_actual_;
  std::deque<CommandSample> command_history_;
  double steering_rate_rps_{0.0};
  double steering_predicted_rad_{0.0};
  double steering_offset_rad_{0.0};
  double offset_gain_{0.0};
  double offset_residual_{0.0};
  bool offset_updated_{false};
  rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steer_actual_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr fused_twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_predicted_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steering_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_innovation_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr sync_gap_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleDynamicsObserver>());
  rclcpp::shutdown();
  return 0;
}
