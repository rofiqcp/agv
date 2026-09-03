// global_field_test_localizer.cpp
// Global field test GNSS+IMU localization WITHOUT dependency on /odometry/filtered.
// Reuses production math/convention but independent gate logic.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <navigation/navigation_math.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

namespace
{

// Helper: yaw from quaternion
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return navigation_math::normalizeAngle(std::atan2(siny_cosp, cosy_cosp));
}

// Helper: quaternion from yaw
geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

}  // namespace

class GlobalFieldTestLocalizer final : public rclcpp::Node
{
public:
  explicit GlobalFieldTestLocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("global_field_test_localizer", options)
  {
    declareParameters();
    readParameters();
    createInterfaces();

    RCLCPP_INFO(
      get_logger(),
      "GlobalFieldTestLocalizer active: WGS84→ENU→map, independent COG gate, no local odom dependency");
  }

private:
  struct Quality
  {
    int satellites{0};
    double dop{99.9};
    double hacc_m{999.0};
    double sacc_mps{-1.0};
    double ground_speed_mps{-1.0};
    double course_enu_rad{std::numeric_limits<double>::quiet_NaN()};
    double course_accuracy_rad{std::numeric_limits<double>::quiet_NaN()};
    bool gnss_fix_ok{false};
  };

  void declareParameters()
  {
    // Reuse production reference
    declare_parameter<double>("reference_latitude", -7.050964918716);
    declare_parameter<double>("reference_longitude", 110.435059847106);
    declare_parameter<double>("reference_map_x_m", 167.493142);
    declare_parameter<double>("reference_map_y_m", 94.933756);
    declare_parameter<double>("map_yaw_from_enu_rad", 0.0);
    declare_parameter<double>("map_calibration_x_m", 0.0);
    declare_parameter<double>("map_calibration_y_m", 0.0);
    declare_parameter<double>("map_calibration_yaw_rad", 0.0);
    declare_parameter<double>("gnss_antenna_x_m", 0.165);
    declare_parameter<double>("gnss_antenna_y_m", 0.0);

    // COG test gate (independent of local odom)
    declare_parameter<bool>("enable_cog_fusion", true);
    declare_parameter<double>("cog_min_ground_speed_mps", 0.25);
    declare_parameter<double>("cog_max_sacc_mps", 0.50);
    declare_parameter<double>("cog_max_heading_accuracy_rad", 0.3490658504); // 20 deg
    declare_parameter<double>("cog_max_imu_gyro_z_rps", 0.12);
    declare_parameter<double>("cog_velocity_course_max_rad", 0.1745329252); // 10 deg
    declare_parameter<double>("quality_timeout_sec", 0.8);
    declare_parameter<double>("velocity_timeout_sec", 1.0);
    declare_parameter<double>("imu_timeout_sec", 0.75);

    // GNSS quality gate
    declare_parameter<int>("min_satellites", 8);
    declare_parameter<double>("max_dop", 2.0);
    declare_parameter<double>("max_hacc_m", 2.5);

    // IMU
    declare_parameter<bool>("enable_imu", true);
    declare_parameter<bool>("use_imu_orientation_seed", false);
  }

  void readParameters()
  {
    reference_latitude_ = get_parameter("reference_latitude").as_double();
    reference_longitude_ = get_parameter("reference_longitude").as_double();
    reference_map_x_m_ = get_parameter("reference_map_x_m").as_double();
    reference_map_y_m_ = get_parameter("reference_map_y_m").as_double();
    map_yaw_from_enu_rad_ = get_parameter("map_yaw_from_enu_rad").as_double();
    map_calibration_x_m_ = get_parameter("map_calibration_x_m").as_double();
    map_calibration_y_m_ = get_parameter("map_calibration_y_m").as_double();
    map_calibration_yaw_rad_ = get_parameter("map_calibration_yaw_rad").as_double();
    antenna_x_m_ = get_parameter("gnss_antenna_x_m").as_double();
    antenna_y_m_ = get_parameter("gnss_antenna_y_m").as_double();

    enable_cog_fusion_ = get_parameter("enable_cog_fusion").as_bool();
    cog_min_ground_speed_mps_ = get_parameter("cog_min_ground_speed_mps").as_double();
    cog_max_sacc_mps_ = get_parameter("cog_max_sacc_mps").as_double();
    cog_max_heading_accuracy_rad_ = get_parameter("cog_max_heading_accuracy_rad").as_double();
    cog_max_imu_gyro_z_rps_ = get_parameter("cog_max_imu_gyro_z_rps").as_double();
    cog_velocity_course_max_rad_ = get_parameter("cog_velocity_course_max_rad").as_double();
    quality_timeout_sec_ = get_parameter("quality_timeout_sec").as_double();
    velocity_timeout_sec_ = get_parameter("velocity_timeout_sec").as_double();
    imu_timeout_sec_ = get_parameter("imu_timeout_sec").as_double();

    min_satellites_ = get_parameter("min_satellites").as_int();
    max_dop_ = get_parameter("max_dop").as_double();
    max_hacc_m_ = get_parameter("max_hacc_m").as_double();

    enable_imu_ = get_parameter("enable_imu").as_bool();
    use_imu_orientation_seed_ = get_parameter("use_imu_orientation_seed").as_bool();

    map_calibration_.x = map_calibration_x_m_;
    map_calibration_.y = map_calibration_y_m_;
    map_calibration_.yaw = map_calibration_yaw_rad_;
  }

  void createInterfaces()
  {
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);

    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss/fix_raw", sensor_qos,
      std::bind(&GlobalFieldTestLocalizer::onGnss, this, std::placeholders::_1));
    gnss_vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/gnss/vel", sensor_qos,
      std::bind(&GlobalFieldTestLocalizer::onGnssVelocity, this, std::placeholders::_1));
    quality_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/gnss/quality", sensor_qos,
      std::bind(&GlobalFieldTestLocalizer::onQuality, this, std::placeholders::_1));
    if (enable_imu_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", sensor_qos,
        std::bind(&GlobalFieldTestLocalizer::onImu, this, std::placeholders::_1));
    }

    gnss_map_pub_ = create_publisher<nav_msgs::msg::Odometry>("/global_test/gnss_map", 10);
    base_velocity_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/global_test/base_velocity", 10);
    cog_heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/global_test/cog_heading", 10);
    diagnostics_pub_ = create_publisher<std_msgs::msg::String>("/global_test/diagnostics", 10);
  }

  void onQuality(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 10) return;
    std::lock_guard<std::mutex> lock(mutex_);
    // Kontrak /gnss/quality aktual (gnss_node.cpp):
    //   0 sat, 1 pDOP/HDOP, 2 hAcc, 3 fixType, 4 source, 5 sAcc,
    //   6 gSpeed, 7 COG ENU, 8 headAcc, ..., 44 gnssFixOK
    quality_.satellites = static_cast<int>(msg->data[0]);
    quality_.dop = msg->data[1];
    quality_.hacc_m = msg->data[2];
    quality_.sacc_mps = msg->data.size() > 5 ? msg->data[5] : -1.0;
    quality_.ground_speed_mps = msg->data.size() > 6 ? msg->data[6] : -1.0;
    quality_.course_enu_rad = msg->data.size() > 7 ? msg->data[7] : std::numeric_limits<double>::quiet_NaN();
    quality_.course_accuracy_rad = msg->data.size() > 8 ? msg->data[8] : std::numeric_limits<double>::quiet_NaN();
    quality_.gnss_fix_ok = (msg->data.size() > 44 && msg->data[44] > 0.5);
    last_quality_time_ = now();
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_gyro_z_rps_ = msg->angular_velocity.z;
    imu_yaw_enu_rad_ = yawFromQuaternion(msg->orientation);
    last_imu_time_ = now();
  }

  void onGnssVelocity(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gnss_velocity_ = *msg;
    have_gnss_velocity_ = true;
    last_gnss_velocity_time_ = now();
  }

  void onGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) return;
    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) return;

    std::lock_guard<std::mutex> lock(mutex_);

    // /gnss/fix_raw is display-authority data. The EKF test receives position
    // only after fresh quality passes the configured sat/DOP/hAcc/fixOK gate.
    if (!positionQualityValidUnlocked()) {
      publishDiagnosticsUnlocked();
      return;
    }

    // Latch only valid moving COG. At low speed/standstill this is intentionally
    // not updated, so low-speed headMot noise cannot rotate the vehicle.
    if (cogValidUnlocked()) {
      held_map_heading_rad_ = navigation_math::normalizeAngle(
        quality_.course_enu_rad + map_yaw_from_enu_rad_ + map_calibration_.yaw);
      have_held_map_heading_ = true;
    }

    // WGS84 → ENU (reuse production pipeline)
    const auto enu = navigation_math::wgs84ToEnu(
      msg->latitude, msg->longitude, reference_latitude_, reference_longitude_);

    // ENU → map antenna (same as production)
    const double heading_map = getMapHeadingUnlocked();
    auto map_antenna = navigation_math::enuToMap(
      enu, reference_map_x_m_, reference_map_y_m_, map_yaw_from_enu_rad_,
      heading_map - map_yaw_from_enu_rad_ - map_calibration_.yaw);

    // Antenna → base (same as production)
    const auto raw_map_base = navigation_math::antennaToBase(map_antenna, antenna_x_m_, antenna_y_m_);

    // Apply map calibration (same as production)
    const auto map_base = navigation_math::mapBaseFromOdom(map_calibration_, raw_map_base);

    // Publish /global_test/gnss_map
    nav_msgs::msg::Odometry odom;
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
      odom.header.stamp = msg->header.stamp;
    } else {
      odom.header.stamp = static_cast<builtin_interfaces::msg::Time>(now());
    }
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = map_base.x;
    odom.pose.pose.position.y = map_base.y;
    odom.pose.pose.orientation = quaternionFromYaw(map_base.yaw);
    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      odom.pose.covariance[0] = msg->position_covariance[0];
      odom.pose.covariance[7] = msg->position_covariance[4];
    } else {
      odom.pose.covariance[0] = 400.0;
      odom.pose.covariance[7] = 400.0;
    }
    odom.pose.covariance[35] = 0.10;
    gnss_map_pub_->publish(odom);

    // Convert ENU velocity into the actual base_footprint frame. Merely changing
    // frame_id would be incorrect. The held COG heading supplies the body rotation;
    // at standstill the last valid heading is retained and COG is not republished.
    if (velocityFreshUnlocked() && have_held_map_heading_) {
      auto vel = gnss_velocity_;
      vel.header.stamp = odom.header.stamp;
      vel.header.frame_id = "base_footprint";
      const double theta = map_yaw_from_enu_rad_ + map_calibration_.yaw;
      const double ct = std::cos(theta), st = std::sin(theta);
      const double ve = gnss_velocity_.twist.twist.linear.x;
      const double vn = gnss_velocity_.twist.twist.linear.y;
      const double vmap_x = ct * ve - st * vn;
      const double vmap_y = st * ve + ct * vn;
      const double cy = std::cos(held_map_heading_rad_);
      const double sy = std::sin(held_map_heading_rad_);
      vel.twist.twist.linear.x = cy * vmap_x + sy * vmap_y;
      vel.twist.twist.linear.y = -sy * vmap_x + cy * vmap_y;
      vel.twist.twist.angular.z = 0.0;  // not fused; IMU owns vyaw

      // Rotate horizontal covariance ENU -> map -> body.
      const double a = gnss_velocity_.twist.covariance[0];
      const double b = gnss_velocity_.twist.covariance[1];
      const double d = gnss_velocity_.twist.covariance[7];
      auto rotateCov = [](double c, double s, double xx, double xy, double yy,
                          double & ox, double & oxy, double & oy) {
        ox = c * c * xx - 2.0 * c * s * xy + s * s * yy;
        oxy = c * s * (xx - yy) + (c * c - s * s) * xy;
        oy = s * s * xx + 2.0 * c * s * xy + c * c * yy;
      };
      double map_xx = 0.0, map_xy = 0.0, map_yy = 0.0;
      rotateCov(ct, st, a, b, d, map_xx, map_xy, map_yy);
      double body_xx = 0.0, body_xy = 0.0, body_yy = 0.0;
      rotateCov(cy, -sy, map_xx, map_xy, map_yy, body_xx, body_xy, body_yy);
      vel.twist.covariance[0] = body_xx;
      vel.twist.covariance[1] = body_xy;
      vel.twist.covariance[6] = body_xy;
      vel.twist.covariance[7] = body_yy;
      vel.twist.covariance[35] = 1.0e9;
      base_velocity_pub_->publish(vel);
    }

    // Publish COG heading if valid
    publishCogHeadingUnlocked(odom.header.stamp);

    // Diagnostics
    publishDiagnosticsUnlocked();
  }

  double getMapHeadingUnlocked()
  {
    if (have_held_map_heading_) return held_map_heading_rad_;
    // For bootstrap: use IMU orientation seed if enabled, else use ENU default
    if (use_imu_orientation_seed_ && enable_imu_ && std::isfinite(imu_yaw_enu_rad_)) {
      return navigation_math::normalizeAngle(
        imu_yaw_enu_rad_ + map_yaw_from_enu_rad_ + map_calibration_.yaw);
    }
    return navigation_math::normalizeAngle(map_yaw_from_enu_rad_ + map_calibration_.yaw);
  }

  bool cogValidUnlocked() const
  {
    if (!enable_cog_fusion_) return false;
    if (!quality_.gnss_fix_ok) return false;
    if (quality_.satellites < min_satellites_) return false;
    if (quality_.dop > max_dop_) return false;
    if (quality_.hacc_m > max_hacc_m_) return false;
    if (!std::isfinite(quality_.course_enu_rad)) return false;

    // Freshness: all supporting measurements must be recent. IMU freshness is
    // skipped when IMU is disabled (pure-GNSS test mode).
    const bool imu_fresh_ok = !enable_imu_ || imuFreshUnlocked();
    if (!qualityFreshUnlocked() || !velocityFreshUnlocked() || !imu_fresh_ok) return false;

    // Speed gate
    if (quality_.ground_speed_mps < cog_min_ground_speed_mps_) return false;

    // sAcc gate
    if (quality_.sacc_mps > 0.0 && quality_.sacc_mps > cog_max_sacc_mps_) return false;

    // Heading accuracy gate
    if (std::isfinite(quality_.course_accuracy_rad) &&
        quality_.course_accuracy_rad > cog_max_heading_accuracy_rad_) {
      return false;
    }

    // Course must agree with the actual ENU Doppler velocity vector.
    const double ve = gnss_velocity_.twist.twist.linear.x;
    const double vn = gnss_velocity_.twist.twist.linear.y;
    if (!std::isfinite(ve) || !std::isfinite(vn)) return false;
    const double velocity_speed = std::hypot(ve, vn);
    if (velocity_speed < cog_min_ground_speed_mps_) return false;
    const double velocity_course_enu = std::atan2(vn, ve);
    const double course_residual = navigation_math::normalizeAngle(
      quality_.course_enu_rad - velocity_course_enu);
    if (std::abs(course_residual) > cog_velocity_course_max_rad_) return false;

    // IMU gyro-Z gate: COG hanya valid saat tidak berputar tajam.
    // Skipped when IMU is disabled (pure-GNSS test mode).
    if (enable_imu_ && std::abs(imu_gyro_z_rps_) > cog_max_imu_gyro_z_rps_) return false;

    return true;
  }

  bool qualityFreshUnlocked() const
  {
    const double age = (now() - last_quality_time_).seconds();
    return last_quality_time_.nanoseconds() > 0 && age >= 0.0 && age <= quality_timeout_sec_;
  }

  bool velocityFreshUnlocked() const
  {
    const double age = (now() - last_gnss_velocity_time_).seconds();
    return have_gnss_velocity_ && last_gnss_velocity_time_.nanoseconds() > 0 &&
      age >= 0.0 && age <= velocity_timeout_sec_;
  }

  bool imuFreshUnlocked() const
  {
    const double age = (now() - last_imu_time_).seconds();
    return std::isfinite(imu_gyro_z_rps_) && last_imu_time_.nanoseconds() > 0 &&
      age >= 0.0 && age <= imu_timeout_sec_;
  }

  bool positionQualityValidUnlocked() const
  {
    return qualityFreshUnlocked() && quality_.gnss_fix_ok &&
      quality_.satellites >= min_satellites_ && std::isfinite(quality_.dop) &&
      quality_.dop <= max_dop_ && std::isfinite(quality_.hacc_m) &&
      quality_.hacc_m <= max_hacc_m_;
  }

  void publishCogHeadingUnlocked(const rclcpp::Time & stamp)
  {
    if (!cogValidUnlocked()) return;

    geometry_msgs::msg::PoseWithCovarianceStamped cog;
    cog.header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);
    cog.header.frame_id = "map";
    const double cog_map_yaw = navigation_math::normalizeAngle(
      quality_.course_enu_rad + map_yaw_from_enu_rad_ + map_calibration_.yaw);
    cog.pose.pose.orientation = quaternionFromYaw(cog_map_yaw);

    // Mark all pose states except yaw as uncertain
    for (auto & v : cog.pose.covariance) v = 1.0e6;
    const double head_var = std::isfinite(quality_.course_accuracy_rad) ?
      quality_.course_accuracy_rad * quality_.course_accuracy_rad : 0.0025;
    cog.pose.covariance[35] = std::clamp(head_var, 0.00121846968, 0.2741556778);

    cog_heading_pub_->publish(cog);
  }

  void publishDiagnosticsUnlocked()
  {
    std::ostringstream ss;
    ss << "sat=" << quality_.satellites
       << " dop=" << quality_.dop
       << " hAcc=" << quality_.hacc_m
       << " sAcc=" << quality_.sacc_mps
       << " speed=" << quality_.ground_speed_mps
       << " cog_valid=" << (cogValidUnlocked() ? "YES" : "NO")
       << " imu_gz=" << imu_gyro_z_rps_;
    std_msgs::msg::String msg;
    msg.data = ss.str();
    diagnostics_pub_->publish(msg);
  }

  // Parameters
  double reference_latitude_{0.0};
  double reference_longitude_{0.0};
  double reference_map_x_m_{0.0};
  double reference_map_y_m_{0.0};
  double map_yaw_from_enu_rad_{0.0};
  double map_calibration_x_m_{0.0};
  double map_calibration_y_m_{0.0};
  double map_calibration_yaw_rad_{0.0};
  double antenna_x_m_{0.0};
  double antenna_y_m_{0.0};
  bool enable_cog_fusion_{true};
  double cog_min_ground_speed_mps_{0.25};
  double cog_max_sacc_mps_{0.50};
  double cog_max_heading_accuracy_rad_{0.349};
  double cog_max_imu_gyro_z_rps_{0.12};
  double cog_velocity_course_max_rad_{0.1745};
  double quality_timeout_sec_{0.8};
  double velocity_timeout_sec_{1.0};
  double imu_timeout_sec_{0.75};
  int min_satellites_{8};
  double max_dop_{2.0};
  double max_hacc_m_{2.5};
  bool enable_imu_{true};
  bool use_imu_orientation_seed_{false};

  navigation_math::Pose2D map_calibration_;

  // State
  std::mutex mutex_;
  Quality quality_;
  double imu_gyro_z_rps_{0.0};
  double imu_yaw_enu_rad_{0.0};
  geometry_msgs::msg::TwistWithCovarianceStamped gnss_velocity_;
  bool have_gnss_velocity_{false};
  double held_map_heading_rad_{0.0};
  bool have_held_map_heading_{false};
  rclcpp::Time last_quality_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gnss_velocity_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};

  // Interfaces
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr quality_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gnss_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr base_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr cog_heading_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalFieldTestLocalizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
