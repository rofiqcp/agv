#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <navigation/navigation_math.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace
{

// Fungsi: Mengambil yaw dari quaternion ROS tanpa membuat dependensi konversi tf2 tambahan.
double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return navigation_math::normalizeAngle(std::atan2(siny_cosp, cosy_cosp));
}

// Fungsi: Mengubah quaternion menjadi roll-pitch-yaw untuk diagnostic IMU.
// Perhitungan ini hanya beberapa operasi skalar dan dijalankan saat pesan IMU
// datang; tidak membuat node/filter tambahan.
void rpyFromQuaternion(
  const geometry_msgs::msg::Quaternion & q, double & roll, double & pitch, double & yaw)
{
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  pitch = std::abs(sinp) >= 1.0 ? std::copysign(1.5707963267948966, sinp) : std::asin(sinp);
  yaw = yawFromQuaternion(q);
}

// sensor_msgs/Imu marks unavailable orientation with covariance[0] == -1.
// Reject that contract explicitly and also reject zero/non-finite quaternions;
// otherwise atan2(0, 1) silently turns a missing heading into a valid 0 rad seed.
bool usableImuOrientation(const sensor_msgs::msg::Imu & msg)
{
  if (msg.orientation_covariance[0] < 0.0) return false;
  const auto & q = msg.orientation;
  if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w)) return false;
  const double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(norm_sq) && norm_sq > 0.50 && norm_sq < 1.50;
}

// Fungsi: Membentuk quaternion planar dari yaw untuk TF map->odom dan diagnostic odometry.
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

// Fungsi: QoS state kecil dengan durability transient-local agar subscriber yang
// baru hidup langsung mengetahui state terakhir tanpa polling atau service tambahan.
rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

}  // namespace

class LocalizationCore final : public rclcpp::Node
{
public:
  // Fungsi: Membangun satu-satunya custom localization runtime. Node ini
  // menggantikan georeference initializer + stationary filter + global TF gate.
  // EKF global publish_tf=false dipakai sebagai pose absolut terfilter untuk
  // koreksi map->odom; gate kontrol tetap ditentukan kualitas/freshness sensor.
  explicit LocalizationCore(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("localization_core", options), tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    declareParameters();
    readParameters();
    loadPersistentMapCalibration();
    loadPersistentCalibrationSamples();
    createInterfaces();

    RCLCPP_INFO(
      get_logger(),
      "LocalizationCore C++ aktif: local=wheel/GNSS vx + relative IMU yaw/gyro-z; magnetic yaw=startup seed; GNSS COG=moving heading correction; fusion vel=%s(cert=%s) COG=%s(cert=%s); direct COG fallback=%s; degraded planning=%s; strict hAcc<=%.2fm DOP<=%.2f sat>=%d",
      enable_global_gnss_velocity_fusion_ ? "on" : "off",
      gnss_velocity_calibration_valid_ ? "PASS" : "WAIT",
      enable_global_gnss_cog_fusion_ ? "on" : "off",
      gnss_cog_calibration_valid_ ? "PASS" : "WAIT",
      enable_gnss_course_yaw_correction_ ? "on" : "off",
      allow_degraded_planning_ ? "on" : "off", strict_max_hacc_m_, strict_max_dop_, strict_min_satellites_);
  }

private:
  struct GnssSample
  {
    navigation_math::Pose2D map_base;
    double hacc_m{999.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct RawMapSample
  {
    navigation_math::Pose2D raw_map_base;
    double hacc_m{999.0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  // Snapshot local odometry with the original measurement stamp. GNSS Part 2
  // interpolates this history so lever-arm position/velocity correction uses
  // the vehicle pose at GNSS measurement time rather than callback-arrival time.
  struct LocalMotionSample
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    navigation_math::Pose2D pose;
    double vx_mps{0.0};
    double vy_mps{0.0};
    double yaw_rate_rps{0.0};
  };

  struct MapCalibrationPair
  {
    double raw_x{0.0};
    double raw_y{0.0};
    double true_x{0.0};
    double true_y{0.0};
    double true_yaw{0.0};
    double hacc_m{999.0};
  };

  struct Quality
  {
    bool received{false};
    int satellites{0};
    double dop{99.9};
    double hacc_m{999.0};
    double fix_metric{0.0};
    int source{0};
    double sacc_mps{-1.0};
    double ground_speed_mps{-1.0};
    double course_enu_rad{std::numeric_limits<double>::quiet_NaN()};
    double course_accuracy_rad{std::numeric_limits<double>::quiet_NaN()};
    double itow_ms{-1.0};
    double pvt_rate_hz{0.0};
    double measurement_age_sec{999.0};
    bool nav_cov_vel_valid{false};
    bool gnss_fix_ok{false};
  };

  // Fungsi: Mendeklarasikan semua parameter di satu tempat agar konfigurasi
  // lapangan dapat diaudit tanpa parameter tersembunyi di beberapa Python node.
  void declareParameters()
  {
    declare_parameter<double>("reference_latitude", -7.050964918716);
    declare_parameter<double>("reference_longitude", 110.435059847106);
    declare_parameter<double>("reference_map_x_m", 167.493142);
    declare_parameter<double>("reference_map_y_m", 94.933756);
    declare_parameter<double>("map_yaw_from_enu_rad", 0.0);
    // Koreksi SE(2) tambahan untuk menyelaraskan raster map dengan GNSS nyata.
    // Nilai bisa dikalibrasi sekali lewat RViz /initialpose dan disimpan otomatis.
    declare_parameter<double>("map_calibration_x_m", 0.0);
    declare_parameter<double>("map_calibration_y_m", 0.0);
    declare_parameter<double>("map_calibration_yaw_rad", 0.0);
    declare_parameter<bool>("persist_map_calibration", true);
    declare_parameter<bool>("use_imu_initial_heading", true);
    declare_parameter<bool>("allow_map_calibration_with_degraded_fix", true);
    declare_parameter<std::string>("map_calibration_file", "");
    declare_parameter<std::string>("map_calibration_samples_file", "");
    declare_parameter<bool>("multi_point_map_calibration", true);
    declare_parameter<int>("map_calibration_min_unique_points", 3);
    declare_parameter<double>("map_calibration_unique_distance_m", 1.0);
    declare_parameter<double>("map_calibration_min_baseline_m", 2.0);
    declare_parameter<double>("map_calibration_min_geometry_score", 0.03);
    declare_parameter<double>("map_calibration_max_rmse_m", 3.0);
    declare_parameter<double>("calibration_capture_window_sec", 3.0);
    declare_parameter<int>("calibration_capture_min_samples", 3);
    declare_parameter<double>("gnss_antenna_x_m", 0.165);
    declare_parameter<double>("gnss_antenna_y_m", 0.0);

    // GNSS Motion Validation Part 2. These topics remain diagnostic/qualification
    // inputs only; they are deliberately NOT wired into robot_localization yet.
    declare_parameter<std::string>("gnss_velocity_topic", "/gnss/vel");
    declare_parameter<std::string>("gnss_velocity_fit_topic", "/gnss/velocity_position_fit");
    declare_parameter<double>("gnss_motion_history_sec", 6.0);
    declare_parameter<double>("gnss_sync_max_gap_sec", 0.30);
    declare_parameter<double>("gnss_velocity_timeout_sec", 0.80);
    declare_parameter<double>("gnss_fit_timeout_sec", 4.0);
    declare_parameter<double>("gnss_velocity_min_validation_speed_mps", 0.15);
    declare_parameter<double>("gnss_speed_consistency_max_mps", 0.20);
    declare_parameter<double>("gnss_fit_speed_residual_max_mps", 0.30);
    declare_parameter<double>("gnss_cog_velocity_course_max_rad", 0.1745329252);
    declare_parameter<double>("gnss_cog_fit_course_max_rad", 0.3490658504);
    declare_parameter<double>("gnss_lateral_velocity_warn_mps", 0.15);
    declare_parameter<double>("wheel_gnss_slip_residual_mps", 0.25);
    declare_parameter<double>("cog_valid_hold_sec", 1.5);
    declare_parameter<double>("cog_invalid_hold_sec", 0.5);

    // User-requested state ownership: GNSS supplies longitudinal velocity and
    // yaw-rate. vyaw is derived from the time derivative of Doppler COG and is
    // deliberately de-weighted at very low speed where COG is unobservable.
    declare_parameter<double>("gnss_yaw_rate_min_speed_mps", 0.20);
    declare_parameter<double>("gnss_yaw_rate_max_abs_rps", 1.50);
    declare_parameter<double>("gnss_yaw_rate_filter_alpha", 0.35);
    declare_parameter<double>("gnss_yaw_rate_min_variance", 0.0025);
    declare_parameter<double>("gnss_yaw_rate_max_variance", 4.0);

    // Stage-2 field certification and temporal/covariance integrity. Fusion is
    // fail-closed: a momentary qualification flag cannot enable a persistent
    // EKF input until a complete field run has been certified in the GUI/tool.
    declare_parameter<bool>("gnss_require_measurement_timestamp", true);
    declare_parameter<double>("gnss_max_future_stamp_sec", 0.10);
    declare_parameter<double>("gnss_max_measurement_age_sec", 1.00);
    declare_parameter<double>("gnss_max_stamp_regression_sec", 0.02);
    declare_parameter<double>("gnss_quality_timeout_sec", 0.60);
    declare_parameter<double>("gnss_velocity_covariance_min_variance", 1.0e-6);
    declare_parameter<double>("gnss_velocity_covariance_max_variance", 1.0);
    declare_parameter<bool>("require_gnss_velocity_certification_for_fusion", true);
    declare_parameter<bool>("require_gnss_cog_certification_for_fusion", true);
    declare_parameter<bool>("gnss_velocity_calibration_valid", false);
    declare_parameter<bool>("gnss_cog_calibration_valid", false);
    // Audit metadata / GUI thresholds. Declared here so localization_cpp.yaml has
    // no hidden/undeclared parameters even though only the GUI/tool modifies them.
    declare_parameter<std::string>("gnss_velocity_calibration_saved_at", "");
    declare_parameter<std::string>("gnss_cog_calibration_saved_at", "");
    declare_parameter<int>("gnss_velocity_calibration_epoch_count", 0);
    declare_parameter<double>("gnss_velocity_calibration_qualified_ratio", 0.0);
    declare_parameter<double>("gnss_velocity_calibration_sync_p95_sec", 0.0);
    declare_parameter<double>("gnss_velocity_calibration_wheel_residual_p95_mps", 0.0);
    declare_parameter<double>("gnss_velocity_calibration_lateral_p95_mps", 0.0);
    declare_parameter<int>("gnss_cog_calibration_epoch_count", 0);
    declare_parameter<double>("gnss_cog_calibration_qualified_ratio", 0.0);
    declare_parameter<double>("gnss_cog_calibration_residual_p95_rad", 0.0);
    declare_parameter<int>("stage2_min_velocity_epochs", 50);
    declare_parameter<int>("stage2_min_cog_epochs", 30);
    declare_parameter<double>("stage2_min_velocity_qualified_ratio", 0.85);
    declare_parameter<double>("stage2_min_cog_qualified_ratio", 0.70);
    declare_parameter<double>("stage2_max_sync_gap_p95_sec", 0.20);
    declare_parameter<double>("stage2_max_wheel_gnss_residual_p95_mps", 0.25);
    declare_parameter<double>("stage2_max_lateral_velocity_p95_mps", 0.15);
    declare_parameter<double>("stage2_max_cog_doppler_residual_p95_rad", 0.1745329252);

    // GNSS Fusion Part 3. Robot_localization subscribes only to these gated
    // topics, never to the raw qualification topics. Both are OFF by default
    // until Part-2 field qualification has been completed on the real AGV.
    declare_parameter<bool>("enable_global_gnss_velocity_fusion", false);
    declare_parameter<bool>("enable_global_gnss_cog_fusion", false);
    declare_parameter<std::string>("gnss_velocity_fusion_topic", "/gnss/base_velocity_fusion");
    declare_parameter<std::string>("gnss_cog_fusion_topic", "/gnss/cog_heading_fusion");
    declare_parameter<double>("gnss_velocity_fusion_min_variance", 0.0025);
    declare_parameter<double>("gnss_velocity_fusion_max_variance", 0.25);
    declare_parameter<double>("gnss_cog_fusion_min_variance_rad2", 0.00121846968);
    declare_parameter<double>("gnss_cog_fusion_max_variance_rad2", 0.2741556778);
    declare_parameter<double>("gnss_fusion_measurement_timeout_sec", 1.0);
    declare_parameter<bool>("use_global_ekf_yaw_for_map_correction", true);
    declare_parameter<double>("global_ekf_yaw_correction_alpha", 0.05);
    declare_parameter<double>("global_ekf_yaw_max_step_rad", 0.00872664626);
    declare_parameter<double>("global_ekf_yaw_max_innovation_rad", 0.7853981634);

    // Legacy direct COG correction is retained only as a fallback mode. When
    // enable_global_gnss_cog_fusion=true, yaw correction comes from global EKF
    // so the same COG measurement is never applied twice.
    // Single-antenna GNSS course-over-ground can correct long-term yaw drift,
    // but only while moving forward sufficiently straight and with a good NAV-PVT
    // course accuracy. Disabled until a field test proves the receiver is reliable.
    declare_parameter<bool>("enable_gnss_course_yaw_correction", false);
    declare_parameter<double>("cog_min_forward_speed_mps", 0.25);
    declare_parameter<double>("cog_max_sacc_mps", 0.50);
    declare_parameter<double>("cog_max_heading_accuracy_rad", 0.3490658504);
    declare_parameter<double>("cog_max_local_yaw_rate_rps", 0.12);
    declare_parameter<double>("cog_max_innovation_rad", 0.7853981634);
    declare_parameter<double>("cog_yaw_alpha", 0.02);
    declare_parameter<double>("cog_max_yaw_step_rad", 0.00872664626);

    declare_parameter<double>("map_min_x_m", 0.0);
    declare_parameter<double>("map_max_x_m", 620.0);
    declare_parameter<double>("map_min_y_m", 0.0);
    declare_parameter<double>("map_max_y_m", 400.0);
    declare_parameter<double>("map_bounds_margin_m", 0.6);

    declare_parameter<bool>("allow_degraded_planning", true);
    // Provisional display gate is intentionally separate from motion safety.
    declare_parameter<bool>("allow_provisional_map_display", true);
    declare_parameter<int>("provisional_min_satellites", 3);
    declare_parameter<double>("provisional_max_dop", 25.0);
    declare_parameter<double>("provisional_max_hacc_m", 150.0);
    declare_parameter<int>("degraded_min_satellites", 8);
    declare_parameter<double>("degraded_max_dop", 6.0);
    declare_parameter<double>("degraded_max_hacc_m", 20.0);
    // Anchor bootstrap is intentionally separate from the autonomous motion gate.
    // false lets a sane degraded fix put the robot on the map; motor autonomy still
    // requires strictQualityHeldUnlocked() later in the readiness gate.
    declare_parameter<bool>("anchor_init_requires_strict", false);
    declare_parameter<int>("startup_gnss_samples", 5);
    declare_parameter<int>("startup_imu_samples", 3);
    declare_parameter<double>("startup_max_spread_m", 5.0);
    // Global EKF may briefly publish an initialized (0,0) before its absolute
    // GNSS state converges. Never let that transient overwrite a valid calibrated
    // GNSS map pose during map->odom bootstrap.
    declare_parameter<double>("global_odom_map_reference_max_error_m", 15.0);

    declare_parameter<int>("strict_min_satellites", 8);
    declare_parameter<double>("strict_max_dop", 2.0);
    declare_parameter<double>("strict_max_hacc_m", 2.5);
    declare_parameter<double>("strict_quality_hold_sec", 1.5);
    // Hysteresis motion: acquire pada strict <=2.5 m, lalu boleh HOLD dengan
    // fix sehat sampai 4.5 m. Di atas HOLD diberi grace singkat, sedangkan
    // no-fix / hAcc sangat buruk menutup gate segera.
    declare_parameter<double>("motion_hold_max_hacc_m", 4.5);
    declare_parameter<double>("motion_hold_max_dop", 2.5);
    declare_parameter<int>("motion_hold_min_satellites", 8);
    declare_parameter<double>("motion_degrade_grace_sec", 3.0);
    declare_parameter<double>("motion_critical_max_hacc_m", 6.0);
    declare_parameter<double>("motion_critical_max_dop", 4.0);
    declare_parameter<int>("motion_critical_min_satellites", 6);

    // Continuous map->odom correction. Gain dibuat kecil saat kendaraan
    // benar-benar bergerak agar URDF tidak meloncat terhadap planner.
    declare_parameter<double>("strict_correction_alpha", 0.20);
    declare_parameter<bool>("freeze_stationary_map_translation", true);
    declare_parameter<double>("strict_moving_correction_alpha", 0.03);
    declare_parameter<bool>("enable_wheel_slip_pose_correction", false);
    declare_parameter<double>("strict_slip_correction_alpha", 0.60);
    declare_parameter<double>("strict_max_correction_m", 0.25);
    declare_parameter<double>("strict_max_yaw_correction_rad", 0.087266463);
    declare_parameter<double>("strict_stationary_speed_mps", 0.15);
    declare_parameter<double>("strict_stationary_yaw_rate_rps", 0.10);
    declare_parameter<double>("slip_min_wheel_speed_mps", 0.25);
    declare_parameter<double>("slip_speed_difference_mps", 0.20);
    declare_parameter<double>("slip_sacc_multiplier", 0.50);
    declare_parameter<double>("slip_yaw_rate_difference_rps", 0.20);
    declare_parameter<std::string>(
      "esc_kinematic_yaw_rate_topic", "/esc/kinematic_yaw_rate_rps");
    declare_parameter<double>("esc_kinematic_yaw_timeout_sec", 0.50);

    declare_parameter<double>("gnss_timeout_sec", 2.5);
    declare_parameter<double>("imu_timeout_sec", 0.75);
    declare_parameter<double>("odom_timeout_sec", 0.75);
    declare_parameter<double>("tf_publish_rate_hz", 10.0);
    declare_parameter<double>("tf_future_offset_sec", 0.05);
    declare_parameter<double>("status_publish_rate_hz", 2.0);
    declare_parameter<double>("log_heartbeat_sec", 10.0);
    declare_parameter<bool>("allow_manual_pose_for_motion", false);

    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<std::string>("raw_gnss_topic", "/gnss/fix_raw");
    declare_parameter<std::string>("quality_topic", "/gnss/quality");
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("local_odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("global_odom_topic", "/odometry/filtered_map");
  }

  // Fungsi: Membaca parameter sekali saat startup. Tidak ada lookup parameter
  // per callback sehingga beban realtime tetap konstan.
  void readParameters()
  {
    reference_latitude_ = get_parameter("reference_latitude").as_double();
    reference_longitude_ = get_parameter("reference_longitude").as_double();
    reference_map_x_m_ = get_parameter("reference_map_x_m").as_double();
    reference_map_y_m_ = get_parameter("reference_map_y_m").as_double();
    map_yaw_from_enu_rad_ = get_parameter("map_yaw_from_enu_rad").as_double();
    map_calibration_.x = get_parameter("map_calibration_x_m").as_double();
    map_calibration_.y = get_parameter("map_calibration_y_m").as_double();
    map_calibration_.yaw = navigation_math::normalizeAngle(
      get_parameter("map_calibration_yaw_rad").as_double());
    persist_map_calibration_ = get_parameter("persist_map_calibration").as_bool();
    use_imu_initial_heading_ = get_parameter("use_imu_initial_heading").as_bool();
    allow_map_calibration_with_degraded_fix_ =
      get_parameter("allow_map_calibration_with_degraded_fix").as_bool();
    map_calibration_file_ = get_parameter("map_calibration_file").as_string();
    if (map_calibration_file_.empty()) {
      const char * home = std::getenv("HOME");
      if (home != nullptr && std::string(home).size() > 0) {
        map_calibration_file_ = std::string(home) + "/.ros/agv_map_calibration.txt";
      }
    }

    map_calibration_samples_file_ =
      get_parameter("map_calibration_samples_file").as_string();
    if (map_calibration_samples_file_.empty()) {
      const char * home = std::getenv("HOME");
      if (home != nullptr && std::string(home).size() > 0) {
        map_calibration_samples_file_ =
          std::string(home) + "/.ros/agv_map_calibration_samples.csv";
      }
    }
    multi_point_map_calibration_ =
      get_parameter("multi_point_map_calibration").as_bool();
    map_calibration_min_unique_points_ = std::max(
      2, static_cast<int>(get_parameter("map_calibration_min_unique_points").as_int()));
    map_calibration_unique_distance_m_ = std::max(
      0.20, get_parameter("map_calibration_unique_distance_m").as_double());
    map_calibration_min_baseline_m_ = std::max(
      0.10, get_parameter("map_calibration_min_baseline_m").as_double());
    map_calibration_min_geometry_score_ = std::clamp(
      get_parameter("map_calibration_min_geometry_score").as_double(), 0.0, 1.0);
    map_calibration_max_rmse_m_ = std::max(
      0.05, get_parameter("map_calibration_max_rmse_m").as_double());
    calibration_capture_window_sec_ = std::max(
      0.5, get_parameter("calibration_capture_window_sec").as_double());
    calibration_capture_min_samples_ = std::max(
      1, static_cast<int>(get_parameter("calibration_capture_min_samples").as_int()));

    antenna_x_m_ = get_parameter("gnss_antenna_x_m").as_double();
    antenna_y_m_ = get_parameter("gnss_antenna_y_m").as_double();
    gnss_velocity_topic_ = get_parameter("gnss_velocity_topic").as_string();
    gnss_velocity_fit_topic_ = get_parameter("gnss_velocity_fit_topic").as_string();
    gnss_motion_history_sec_ = std::max(1.0, get_parameter("gnss_motion_history_sec").as_double());
    gnss_sync_max_gap_sec_ = std::clamp(get_parameter("gnss_sync_max_gap_sec").as_double(), 0.02, 2.0);
    gnss_velocity_timeout_sec_ = std::clamp(get_parameter("gnss_velocity_timeout_sec").as_double(), 0.05, 5.0);
    gnss_fit_timeout_sec_ = std::clamp(get_parameter("gnss_fit_timeout_sec").as_double(), 0.2, 30.0);
    gnss_velocity_min_validation_speed_mps_ = std::max(0.01, get_parameter("gnss_velocity_min_validation_speed_mps").as_double());
    gnss_speed_consistency_max_mps_ = std::max(0.01, get_parameter("gnss_speed_consistency_max_mps").as_double());
    gnss_fit_speed_residual_max_mps_ = std::max(0.01, get_parameter("gnss_fit_speed_residual_max_mps").as_double());
    gnss_cog_velocity_course_max_rad_ = std::clamp(get_parameter("gnss_cog_velocity_course_max_rad").as_double(), 0.01, M_PI);
    gnss_cog_fit_course_max_rad_ = std::clamp(get_parameter("gnss_cog_fit_course_max_rad").as_double(), 0.01, M_PI);
    gnss_lateral_velocity_warn_mps_ = std::max(0.01, get_parameter("gnss_lateral_velocity_warn_mps").as_double());
    wheel_gnss_slip_residual_mps_ = std::max(0.01, get_parameter("wheel_gnss_slip_residual_mps").as_double());
    cog_valid_hold_sec_ = std::max(0.0, get_parameter("cog_valid_hold_sec").as_double());
    cog_invalid_hold_sec_ = std::max(0.0, get_parameter("cog_invalid_hold_sec").as_double());
    gnss_yaw_rate_min_speed_mps_ = std::max(0.05,
      get_parameter("gnss_yaw_rate_min_speed_mps").as_double());
    gnss_yaw_rate_max_abs_rps_ = std::max(0.10,
      get_parameter("gnss_yaw_rate_max_abs_rps").as_double());
    gnss_yaw_rate_filter_alpha_ = std::clamp(
      get_parameter("gnss_yaw_rate_filter_alpha").as_double(), 0.01, 1.0);
    gnss_yaw_rate_min_variance_ = std::max(1.0e-8,
      get_parameter("gnss_yaw_rate_min_variance").as_double());
    gnss_yaw_rate_max_variance_ = std::max(gnss_yaw_rate_min_variance_,
      get_parameter("gnss_yaw_rate_max_variance").as_double());
    gnss_require_measurement_timestamp_ =
      get_parameter("gnss_require_measurement_timestamp").as_bool();
    gnss_max_future_stamp_sec_ = std::clamp(
      get_parameter("gnss_max_future_stamp_sec").as_double(), 0.0, 2.0);
    gnss_max_measurement_age_sec_ = std::clamp(
      get_parameter("gnss_max_measurement_age_sec").as_double(), 0.05, 10.0);
    gnss_max_stamp_regression_sec_ = std::clamp(
      get_parameter("gnss_max_stamp_regression_sec").as_double(), 0.0, 1.0);
    gnss_quality_timeout_sec_ = std::clamp(
      get_parameter("gnss_quality_timeout_sec").as_double(), 0.05, 5.0);
    gnss_velocity_covariance_min_variance_ = std::max(
      1.0e-12, get_parameter("gnss_velocity_covariance_min_variance").as_double());
    gnss_velocity_covariance_max_variance_ = std::max(
      gnss_velocity_covariance_min_variance_,
      get_parameter("gnss_velocity_covariance_max_variance").as_double());
    require_gnss_velocity_certification_for_fusion_ =
      get_parameter("require_gnss_velocity_certification_for_fusion").as_bool();
    require_gnss_cog_certification_for_fusion_ =
      get_parameter("require_gnss_cog_certification_for_fusion").as_bool();
    gnss_velocity_calibration_valid_ =
      get_parameter("gnss_velocity_calibration_valid").as_bool();
    gnss_cog_calibration_valid_ = get_parameter("gnss_cog_calibration_valid").as_bool();
    enable_global_gnss_velocity_fusion_ =
      get_parameter("enable_global_gnss_velocity_fusion").as_bool();
    enable_global_gnss_cog_fusion_ = get_parameter("enable_global_gnss_cog_fusion").as_bool();
    gnss_velocity_fusion_topic_ = get_parameter("gnss_velocity_fusion_topic").as_string();
    gnss_cog_fusion_topic_ = get_parameter("gnss_cog_fusion_topic").as_string();
    gnss_velocity_fusion_min_variance_ = std::max(1.0e-8,
      get_parameter("gnss_velocity_fusion_min_variance").as_double());
    gnss_velocity_fusion_max_variance_ = std::max(gnss_velocity_fusion_min_variance_,
      get_parameter("gnss_velocity_fusion_max_variance").as_double());
    gnss_cog_fusion_min_variance_rad2_ = std::max(1.0e-8,
      get_parameter("gnss_cog_fusion_min_variance_rad2").as_double());
    gnss_cog_fusion_max_variance_rad2_ = std::max(gnss_cog_fusion_min_variance_rad2_,
      get_parameter("gnss_cog_fusion_max_variance_rad2").as_double());
    gnss_fusion_measurement_timeout_sec_ = std::clamp(
      get_parameter("gnss_fusion_measurement_timeout_sec").as_double(), 0.05, 10.0);
    use_global_ekf_yaw_for_map_correction_ =
      get_parameter("use_global_ekf_yaw_for_map_correction").as_bool();
    global_ekf_yaw_correction_alpha_ = std::clamp(
      get_parameter("global_ekf_yaw_correction_alpha").as_double(), 0.0, 1.0);
    global_ekf_yaw_max_step_rad_ = std::clamp(
      get_parameter("global_ekf_yaw_max_step_rad").as_double(), 1.0e-5, 0.25);
    global_ekf_yaw_max_innovation_rad_ = std::clamp(
      get_parameter("global_ekf_yaw_max_innovation_rad").as_double(), 0.05, M_PI);
    enable_gnss_course_yaw_correction_ =
      get_parameter("enable_gnss_course_yaw_correction").as_bool();
    cog_min_forward_speed_mps_ =
      std::max(0.05, get_parameter("cog_min_forward_speed_mps").as_double());
    cog_max_sacc_mps_ = std::max(0.01, get_parameter("cog_max_sacc_mps").as_double());
    cog_max_heading_accuracy_rad_ = std::clamp(
      get_parameter("cog_max_heading_accuracy_rad").as_double(), 0.01, M_PI);
    cog_max_local_yaw_rate_rps_ =
      std::max(0.01, get_parameter("cog_max_local_yaw_rate_rps").as_double());
    cog_max_innovation_rad_ = std::clamp(
      get_parameter("cog_max_innovation_rad").as_double(), 0.05, M_PI);
    cog_yaw_alpha_ = std::clamp(get_parameter("cog_yaw_alpha").as_double(), 0.0, 1.0);
    cog_max_yaw_step_rad_ = std::clamp(
      get_parameter("cog_max_yaw_step_rad").as_double(), 1.0e-5, 0.25);

    map_min_x_m_ = get_parameter("map_min_x_m").as_double();
    map_max_x_m_ = get_parameter("map_max_x_m").as_double();
    map_min_y_m_ = get_parameter("map_min_y_m").as_double();
    map_max_y_m_ = get_parameter("map_max_y_m").as_double();
    map_bounds_margin_m_ = get_parameter("map_bounds_margin_m").as_double();

    allow_degraded_planning_ = get_parameter("allow_degraded_planning").as_bool();
    allow_provisional_map_display_ = get_parameter("allow_provisional_map_display").as_bool();
    provisional_min_satellites_ = std::max(1, static_cast<int>(get_parameter("provisional_min_satellites").as_int()));
    provisional_max_dop_ = std::max(1.0, get_parameter("provisional_max_dop").as_double());
    provisional_max_hacc_m_ = std::max(5.0, get_parameter("provisional_max_hacc_m").as_double());
    degraded_min_satellites_ = static_cast<int>(get_parameter("degraded_min_satellites").as_int());
    degraded_max_dop_ = get_parameter("degraded_max_dop").as_double();
    degraded_max_hacc_m_ = get_parameter("degraded_max_hacc_m").as_double();
    anchor_init_requires_strict_ = get_parameter("anchor_init_requires_strict").as_bool();
    startup_gnss_samples_ = std::max(1, static_cast<int>(get_parameter("startup_gnss_samples").as_int()));
    startup_imu_samples_ = std::max(1, static_cast<int>(get_parameter("startup_imu_samples").as_int()));
    startup_max_spread_m_ = get_parameter("startup_max_spread_m").as_double();
    global_odom_map_reference_max_error_m_ = std::max(1.0,
      get_parameter("global_odom_map_reference_max_error_m").as_double());

    strict_min_satellites_ = static_cast<int>(get_parameter("strict_min_satellites").as_int());
    strict_max_dop_ = get_parameter("strict_max_dop").as_double();
    strict_max_hacc_m_ = get_parameter("strict_max_hacc_m").as_double();
    strict_quality_hold_sec_ = get_parameter("strict_quality_hold_sec").as_double();
    motion_hold_max_hacc_m_ = std::max(
      strict_max_hacc_m_, get_parameter("motion_hold_max_hacc_m").as_double());
    motion_hold_max_dop_ = std::max(
      strict_max_dop_, get_parameter("motion_hold_max_dop").as_double());
    motion_hold_min_satellites_ = std::max(
      1, static_cast<int>(get_parameter("motion_hold_min_satellites").as_int()));
    motion_degrade_grace_sec_ = std::max(
      0.0, get_parameter("motion_degrade_grace_sec").as_double());
    motion_critical_max_hacc_m_ = std::max(
      motion_hold_max_hacc_m_, get_parameter("motion_critical_max_hacc_m").as_double());
    motion_critical_max_dop_ = std::max(
      motion_hold_max_dop_, get_parameter("motion_critical_max_dop").as_double());
    motion_critical_min_satellites_ = std::max(
      1, static_cast<int>(get_parameter("motion_critical_min_satellites").as_int()));
    strict_correction_alpha_ = std::clamp(
      get_parameter("strict_correction_alpha").as_double(), 0.0, 1.0);
    freeze_stationary_map_translation_ =
      get_parameter("freeze_stationary_map_translation").as_bool();
    strict_moving_correction_alpha_ = std::clamp(
      get_parameter("strict_moving_correction_alpha").as_double(), 0.0, 1.0);
    enable_wheel_slip_pose_correction_ = get_parameter("enable_wheel_slip_pose_correction").as_bool();
    strict_slip_correction_alpha_ = std::clamp(
      get_parameter("strict_slip_correction_alpha").as_double(), 0.0, 1.0);
    strict_max_correction_m_ = std::max(
      0.05, get_parameter("strict_max_correction_m").as_double());
    strict_max_yaw_correction_rad_ = std::max(
      0.01, get_parameter("strict_max_yaw_correction_rad").as_double());
    strict_stationary_speed_mps_ = std::max(
      0.0, get_parameter("strict_stationary_speed_mps").as_double());
    strict_stationary_yaw_rate_rps_ = std::max(
      0.0, get_parameter("strict_stationary_yaw_rate_rps").as_double());
    slip_min_wheel_speed_mps_ = std::max(
      0.0, get_parameter("slip_min_wheel_speed_mps").as_double());
    slip_speed_difference_mps_ = std::max(
      0.0, get_parameter("slip_speed_difference_mps").as_double());
    slip_sacc_multiplier_ = std::max(
      0.0, get_parameter("slip_sacc_multiplier").as_double());
    slip_yaw_rate_difference_rps_ = std::max(
      0.0, get_parameter("slip_yaw_rate_difference_rps").as_double());
    esc_kinematic_yaw_rate_topic_ =
      get_parameter("esc_kinematic_yaw_rate_topic").as_string();
    esc_kinematic_yaw_timeout_sec_ = std::max(
      0.05, get_parameter("esc_kinematic_yaw_timeout_sec").as_double());

    gnss_timeout_sec_ = get_parameter("gnss_timeout_sec").as_double();
    imu_timeout_sec_ = get_parameter("imu_timeout_sec").as_double();
    odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();
    tf_publish_rate_hz_ = std::max(1.0, get_parameter("tf_publish_rate_hz").as_double());
    tf_future_offset_sec_ = std::clamp(
      get_parameter("tf_future_offset_sec").as_double(), 0.0, 0.25);
    status_publish_rate_hz_ = std::max(0.5, get_parameter("status_publish_rate_hz").as_double());
    log_heartbeat_sec_ = std::max(1.0, get_parameter("log_heartbeat_sec").as_double());
    allow_manual_pose_for_motion_ = get_parameter("allow_manual_pose_for_motion").as_bool();

    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    raw_gnss_topic_ = get_parameter("raw_gnss_topic").as_string();
    quality_topic_ = get_parameter("quality_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    local_odom_topic_ = get_parameter("local_odom_topic").as_string();
    global_odom_topic_ = get_parameter("global_odom_topic").as_string();
  }

  // Fungsi: Terapkan transform koreksi SE(2) dari frame georeference kasar
  // menuju frame raster map yang benar. map_calibration_ adalah T_map<-raw_map.
  navigation_math::Pose2D applyMapCalibrationUnlocked(
    const navigation_math::Pose2D & raw_pose) const
  {
    return navigation_math::mapBaseFromOdom(map_calibration_, raw_pose);
  }

  // Fungsi: Load kalibrasi map persisten. Format sengaja sangat sederhana agar
  // bisa diaudit/diedit tanpa parser YAML tambahan: "x <m>", "y <m>", "yaw <rad>".
  void loadPersistentMapCalibration()
  {
    if (!persist_map_calibration_ || map_calibration_file_.empty()) return;
    std::ifstream in(map_calibration_file_);
    if (!in.is_open()) {
      RCLCPP_INFO(
        get_logger(),
        "Map calibration: belum ada file persisten (%s); memakai parameter config.",
        map_calibration_file_.c_str());
      return;
    }

    navigation_math::Pose2D loaded = map_calibration_;
    std::string key;
    double value = 0.0;
    while (in >> key >> value) {
      if (key == "x") loaded.x = value;
      else if (key == "y") loaded.y = value;
      else if (key == "yaw") loaded.yaw = navigation_math::normalizeAngle(value);
    }
    if (navigation_math::finite3(loaded.x, loaded.y, loaded.yaw)) {
      map_calibration_ = loaded;
      RCLCPP_INFO(
        get_logger(),
        "Map calibration loaded: dx=%.3f dy=%.3f dyaw=%.2fdeg (%s)",
        map_calibration_.x, map_calibration_.y,
        map_calibration_.yaw * 180.0 / 3.14159265358979323846,
        map_calibration_file_.c_str());
    }
  }

  // Fungsi: Simpan kalibrasi hasil 2D Pose Estimate agar stop/run berikutnya
  // menggunakan transform map yang sama, bukan kembali ke offset lama.
  void savePersistentMapCalibrationUnlocked()
  {
    if (!persist_map_calibration_ || map_calibration_file_.empty()) return;
    std::ofstream out(map_calibration_file_, std::ios::trunc);
    if (!out.is_open()) {
      RCLCPP_ERROR(
        get_logger(), "Gagal menyimpan map calibration ke %s",
        map_calibration_file_.c_str());
      return;
    }
    out << std::setprecision(17);
    out << "x " << map_calibration_.x << "\n";
    out << "y " << map_calibration_.y << "\n";
    out << "yaw " << map_calibration_.yaw << "\n";
    out.close();
    RCLCPP_INFO(
      get_logger(),
      "Map calibration saved: dx=%.3f dy=%.3f dyaw=%.2fdeg -> %s",
      map_calibration_.x, map_calibration_.y,
      map_calibration_.yaw * 180.0 / 3.14159265358979323846,
      map_calibration_file_.c_str());
  }

  // Persist pasangan RAW GNSS-map -> ground-truth raster map.
  void savePersistentCalibrationSamplesUnlocked()
  {
    if (map_calibration_samples_file_.empty()) return;
    std::ofstream out(map_calibration_samples_file_, std::ios::trunc);
    if (!out.is_open()) {
      RCLCPP_ERROR(
        get_logger(), "Gagal menyimpan calibration samples ke %s",
        map_calibration_samples_file_.c_str());
      return;
    }
    out << std::setprecision(17);
    out << "# raw_x,raw_y,true_x,true_y,true_yaw,hacc_m\n";
    for (const auto & p : map_calibration_pairs_) {
      out << p.raw_x << "," << p.raw_y << ","
          << p.true_x << "," << p.true_y << ","
          << p.true_yaw << "," << p.hacc_m << "\n";
    }
  }

  void loadPersistentCalibrationSamples()
  {
    if (map_calibration_samples_file_.empty()) return;
    std::ifstream in(map_calibration_samples_file_);
    if (!in.is_open()) {
      RCLCPP_INFO(
        get_logger(),
        "Map calibration samples: belum ada (%s).",
        map_calibration_samples_file_.c_str());
      return;
    }

    std::vector<MapCalibrationPair> loaded;
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream ss(line);
      MapCalibrationPair p;
      if (ss >> p.raw_x >> p.raw_y >> p.true_x >> p.true_y >>
          p.true_yaw >> p.hacc_m)
      {
        if (std::isfinite(p.raw_x) && std::isfinite(p.raw_y) &&
            std::isfinite(p.true_x) && std::isfinite(p.true_y)) {
          loaded.push_back(p);
        }
      }
    }
    map_calibration_pairs_ = std::move(loaded);
    RCLCPP_INFO(
      get_logger(), "Map calibration samples loaded: %zu observasi dari %s",
      map_calibration_pairs_.size(), map_calibration_samples_file_.c_str());
  }

  int uniqueCalibrationTargetsUnlocked() const
  {
    std::vector<std::pair<double, double>> unique;
    for (const auto & p : map_calibration_pairs_) {
      bool found = false;
      for (const auto & u : unique) {
        if (std::hypot(p.true_x - u.first, p.true_y - u.second) <=
            map_calibration_unique_distance_m_) {
          found = true;
          break;
        }
      }
      if (!found) unique.emplace_back(p.true_x, p.true_y);
    }
    return static_cast<int>(unique.size());
  }

  bool robustRawCaptureUnlocked(
    navigation_math::Pose2D & raw_out,
    double & hacc_out,
    int & used_samples) const
  {
    const auto t = now();
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> hs;
    for (const auto & sample : raw_map_capture_window_) {
      const double age = (t - sample.stamp).seconds();
      if (age < 0.0 || age > calibration_capture_window_sec_) continue;
      xs.push_back(sample.raw_map_base.x);
      ys.push_back(sample.raw_map_base.y);
      if (std::isfinite(sample.hacc_m)) hs.push_back(sample.hacc_m);
    }
    used_samples = static_cast<int>(xs.size());
    if (used_samples < calibration_capture_min_samples_) return false;

    raw_out.x = navigation_math::median(xs);
    raw_out.y = navigation_math::median(ys);
    raw_out.yaw = have_last_raw_map_base_ ? last_raw_map_base_.yaw : 0.0;
    hacc_out = hs.empty() ? quality_.hacc_m : navigation_math::median(hs);
    return std::isfinite(raw_out.x) && std::isfinite(raw_out.y);
  }

  bool calibrationGeometryUnlocked(double & baseline_m, double & score) const
  {
    baseline_m = 0.0;
    score = 0.0;
    if (map_calibration_pairs_.size() < 3U) return false;

    double mx = 0.0, my = 0.0;
    for (const auto & p : map_calibration_pairs_) { mx += p.true_x; my += p.true_y; }
    const double n = static_cast<double>(map_calibration_pairs_.size());
    mx /= n; my /= n;

    double cxx = 0.0, cyy = 0.0, cxy = 0.0;
    for (const auto & p : map_calibration_pairs_) {
      const double x = p.true_x - mx;
      const double y = p.true_y - my;
      cxx += x * x; cyy += y * y; cxy += x * y;
    }
    cxx /= n; cyy /= n; cxy /= n;
    const double trace = cxx + cyy;
    const double disc = std::sqrt(std::max(0.0, (cxx - cyy) * (cxx - cyy) + 4.0 * cxy * cxy));
    const double lmax = std::max(0.0, 0.5 * (trace + disc));
    const double lmin = std::max(0.0, 0.5 * (trace - disc));
    score = lmax > 1.0e-9 ? std::sqrt(lmin / lmax) : 0.0;

    for (size_t i = 0; i < map_calibration_pairs_.size(); ++i) {
      for (size_t j = i + 1; j < map_calibration_pairs_.size(); ++j) {
        baseline_m = std::max(baseline_m, std::hypot(
          map_calibration_pairs_[i].true_x - map_calibration_pairs_[j].true_x,
          map_calibration_pairs_[i].true_y - map_calibration_pairs_[j].true_y));
      }
    }
    return std::isfinite(baseline_m) && std::isfinite(score);
  }

  bool solveMultiPointMapCalibrationUnlocked(
    navigation_math::Pose2D & solved,
    double & rmse_m,
    double & diagnostic_scale) const
  {
    if (map_calibration_pairs_.size() < 2U) return false;
    if (uniqueCalibrationTargetsUnlocked() < map_calibration_min_unique_points_) {
      return false;
    }
    double geometry_baseline = 0.0;
    double geometry_score = 0.0;
    if (!calibrationGeometryUnlocked(geometry_baseline, geometry_score) ||
        geometry_baseline < map_calibration_min_baseline_m_ ||
        geometry_score < map_calibration_min_geometry_score_) {
      return false;
    }

    double sx = 0.0, sy = 0.0, tx = 0.0, ty = 0.0;
    const double n = static_cast<double>(map_calibration_pairs_.size());
    for (const auto & p : map_calibration_pairs_) {
      sx += p.raw_x; sy += p.raw_y;
      tx += p.true_x; ty += p.true_y;
    }
    sx /= n; sy /= n; tx /= n; ty /= n;

    double a = 0.0;
    double b = 0.0;
    double source_energy = 0.0;
    for (const auto & p : map_calibration_pairs_) {
      const double x = p.raw_x - sx;
      const double y = p.raw_y - sy;
      const double X = p.true_x - tx;
      const double Y = p.true_y - ty;
      a += x * X + y * Y;
      b += x * Y - y * X;
      source_energy += x * x + y * y;
    }
    if (source_energy < 1.0e-6) return false;

    solved.yaw = navigation_math::normalizeAngle(std::atan2(b, a));
    const double c = std::cos(solved.yaw);
    const double sn = std::sin(solved.yaw);
    solved.x = tx - (c * sx - sn * sy);
    solved.y = ty - (sn * sx + c * sy);

    double residual_sq = 0.0;
    double scale_num = 0.0;
    for (const auto & p : map_calibration_pairs_) {
      const double px = solved.x + c * p.raw_x - sn * p.raw_y;
      const double py = solved.y + sn * p.raw_x + c * p.raw_y;
      const double ex = px - p.true_x;
      const double ey = py - p.true_y;
      residual_sq += ex * ex + ey * ey;

      const double x = p.raw_x - sx;
      const double y = p.raw_y - sy;
      const double X = p.true_x - tx;
      const double Y = p.true_y - ty;
      const double rx = c * x - sn * y;
      const double ry = sn * x + c * y;
      scale_num += rx * X + ry * Y;
    }

    rmse_m = std::sqrt(residual_sq / n);
    diagnostic_scale = source_energy > 1.0e-9 ? scale_num / source_energy : 1.0;
    return navigation_math::finite3(solved.x, solved.y, solved.yaw) &&
      std::isfinite(rmse_m) && rmse_m <= map_calibration_max_rmse_m_ &&
      std::isfinite(diagnostic_scale);
  }

  std::string estimatorSummaryUnlocked() const
  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);

    navigation_math::Pose2D map_pose;
    const bool have_map_pose = anchor_valid_ && have_odom_;
    if (have_map_pose) {
      map_pose = navigation_math::mapBaseFromOdom(anchor_map_odom_, odom_base_);
    }

    ss << "POSE_ESTIMATOR ";
    if (have_map_pose) {
      ss << "map=(" << map_pose.x << "," << map_pose.y << ","
         << map_pose.yaw * 180.0 / 3.14159265358979323846 << "deg)";
    } else {
      ss << "map=(NA)";
    }

    if (have_global_odom_) {
      ss << " global=(" << global_ekf_pose_.x << "," << global_ekf_pose_.y << ","
         << global_ekf_pose_.yaw * 180.0 / 3.14159265358979323846 << "deg)";
    } else {
      ss << " global=(NA)";
    }

    if (have_odom_) {
      ss << " local_odom=(" << odom_base_.x << "," << odom_base_.y << ","
         << odom_base_.yaw * 180.0 / 3.14159265358979323846 << "deg)";
    } else {
      ss << " local_odom=(NA)";
    }

    if (have_last_raw_map_base_) {
      const auto corrected = applyMapCalibrationUnlocked(last_raw_map_base_);
      ss << " raw_gnss_map=(" << last_raw_map_base_.x << ","
         << last_raw_map_base_.y << ")"
         << " corrected_gnss_map=(" << corrected.x << "," << corrected.y << ")";
    } else {
      ss << " raw_gnss_map=(NA)";
    }

    double cal_baseline = 0.0, cal_geometry = 0.0;
    calibrationGeometryUnlocked(cal_baseline, cal_geometry);
    ss << " hAcc=" << quality_.hacc_m
       << " cal_samples=" << map_calibration_pairs_.size()
       << " unique=" << uniqueCalibrationTargetsUnlocked()
       << " cal_baseline_m=" << cal_baseline
       << " cal_geometry=" << cal_geometry;
    return ss.str();
  }

  void onCapturePoseEstimator(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response->success = anchor_valid_ && have_odom_;
    response->message = estimatorSummaryUnlocked();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void onResetCalibrationSamples(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_calibration_pairs_.clear();
    savePersistentCalibrationSamplesUnlocked();
    response->success = true;
    response->message =
      "Semua calibration samples dihapus; map calibration aktif tidak diubah.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }


  // Fungsi: Membuat subscriber depth-1 dan publisher state transient-local.
  // Tidak ada subscription /map 25 juta cell; validasi custom cukup memakai
  // metadata bounds karena collision/static occupancy sudah menjadi tugas Nav2.
  rclcpp::Time stampOrNow(const builtin_interfaces::msg::Time & stamp) const
  {
    if (stamp.sec == 0 && stamp.nanosec == 0) return now();
    return rclcpp::Time(stamp, get_clock()->get_clock_type());
  }

  // Interpolate local EKF state at a sensor measurement timestamp. Yaw uses the
  // shortest angular path. Extrapolation is intentionally limited; a GNSS sample
  // too far from odom history is rejected instead of silently using current pose.
  bool localStateAtUnlocked(const rclcpp::Time & stamp, LocalMotionSample & out) const
  {
    if (local_motion_history_.empty()) return false;
    if (local_motion_history_.size() == 1U) {
      const double gap = std::abs((stamp - local_motion_history_.front().stamp).seconds());
      if (gap > gnss_sync_max_gap_sec_) return false;
      out = local_motion_history_.front();
      return true;
    }

    if (stamp <= local_motion_history_.front().stamp) {
      const double gap = (local_motion_history_.front().stamp - stamp).seconds();
      if (gap > gnss_sync_max_gap_sec_) return false;
      out = local_motion_history_.front();
      return true;
    }
    if (stamp >= local_motion_history_.back().stamp) {
      const double gap = (stamp - local_motion_history_.back().stamp).seconds();
      if (gap > gnss_sync_max_gap_sec_) return false;
      out = local_motion_history_.back();
      return true;
    }

    for (size_t i = 1; i < local_motion_history_.size(); ++i) {
      const auto & b = local_motion_history_[i];
      if (stamp > b.stamp) continue;
      const auto & a = local_motion_history_[i - 1];
      const double dt = (b.stamp - a.stamp).seconds();
      if (dt <= 1.0e-9) { out = b; return true; }
      const double ta = (stamp - a.stamp).seconds();
      if (ta < -gnss_sync_max_gap_sec_ || (b.stamp - stamp).seconds() < -gnss_sync_max_gap_sec_) return false;
      const double u = std::clamp(ta / dt, 0.0, 1.0);
      out.stamp = stamp;
      out.pose.x = a.pose.x + u * (b.pose.x - a.pose.x);
      out.pose.y = a.pose.y + u * (b.pose.y - a.pose.y);
      out.pose.yaw = navigation_math::normalizeAngle(
        a.pose.yaw + u * navigation_math::normalizeAngle(b.pose.yaw - a.pose.yaw));
      out.vx_mps = a.vx_mps + u * (b.vx_mps - a.vx_mps);
      out.vy_mps = a.vy_mps + u * (b.vy_mps - a.vy_mps);
      out.yaw_rate_rps = a.yaw_rate_rps + u * (b.yaw_rate_rps - a.yaw_rate_rps);
      return true;
    }
    return false;
  }

  double mapHeadingForLocalStateUnlocked(const LocalMotionSample & local) const
  {
    if (anchor_valid_) {
      return navigation_math::normalizeAngle(anchor_map_odom_.yaw + local.pose.yaw);
    }
    if (initial_heading_latched_) {
      const double delta = navigation_math::normalizeAngle(local.pose.yaw - initial_odom_yaw_rad_);
      return navigation_math::normalizeAngle(initial_map_heading_rad_ + delta);
    }
    return navigation_math::normalizeAngle(map_yaw_from_enu_rad_ + map_calibration_.yaw);
  }

  bool gnssVelocityFreshUnlocked() const
  {
    if (last_gnss_velocity_time_.nanoseconds() <= 0) return false;
    const double age = (now() - last_gnss_velocity_time_).seconds();
    return age >= 0.0 && age <= gnss_velocity_timeout_sec_;
  }

  bool gnssFitFreshUnlocked() const
  {
    if (last_gnss_fit_time_.nanoseconds() <= 0) return false;
    const double age = (now() - last_gnss_fit_time_).seconds();
    return age >= 0.0 && age <= gnss_fit_timeout_sec_;
  }

  bool qualityFreshUnlocked() const
  {
    if (!quality_.received || last_quality_time_.nanoseconds() <= 0) return false;
    const double age = (now() - last_quality_time_).seconds();
    return age >= 0.0 && age <= gnss_quality_timeout_sec_;
  }

  bool velocityCertificationAllowsFusionUnlocked() const
  {
    return !require_gnss_velocity_certification_for_fusion_ ||
      gnss_velocity_calibration_valid_;
  }

  bool cogCertificationAllowsFusionUnlocked() const
  {
    return (!require_gnss_cog_certification_for_fusion_ || gnss_cog_calibration_valid_) &&
      velocityCertificationAllowsFusionUnlocked();
  }

  bool validVelocityCovarianceUnlocked(
    const geometry_msgs::msg::TwistWithCovarianceStamped & msg) const
  {
    const double xx = msg.twist.covariance[0];
    const double xy = 0.5 * (msg.twist.covariance[1] + msg.twist.covariance[6]);
    const double yy = msg.twist.covariance[7];
    if (!std::isfinite(xx) || !std::isfinite(xy) || !std::isfinite(yy)) return false;
    if (xx < gnss_velocity_covariance_min_variance_ ||
        yy < gnss_velocity_covariance_min_variance_ ||
        xx > gnss_velocity_covariance_max_variance_ ||
        yy > gnss_velocity_covariance_max_variance_) return false;
    const double det = xx * yy - xy * xy;
    return det >= -1.0e-10;
  }

  bool gnssMeasurementStampUsableUnlocked(
    const builtin_interfaces::msg::Time & raw_stamp,
    rclcpp::Time & stamp, std::string & reason)
  {
    const bool zero = raw_stamp.sec == 0 && raw_stamp.nanosec == 0;
    if (zero && gnss_require_measurement_timestamp_) {
      reason = "missing_measurement_stamp";
      return false;
    }
    stamp = zero ? now() : rclcpp::Time(raw_stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (age < -gnss_max_future_stamp_sec_) {
      reason = "measurement_stamp_in_future";
      return false;
    }
    if (age > gnss_max_measurement_age_sec_) {
      reason = "measurement_stamp_stale";
      return false;
    }
    if (last_accepted_gnss_velocity_stamp_.nanoseconds() > 0) {
      const double regression = (last_accepted_gnss_velocity_stamp_ - stamp).seconds();
      if (regression > gnss_max_stamp_regression_sec_) {
        reason = "measurement_stamp_regression";
        return false;
      }
      if (stamp == last_accepted_gnss_velocity_stamp_) {
        reason = "duplicate_measurement_stamp";
        return false;
      }
    }
    return true;
  }

  bool fusionStampFreshUnlocked(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() <= 0) return false;
    const double age = (now() - stamp).seconds();
    return age >= 0.0 && age <= gnss_fusion_measurement_timeout_sec_;
  }

  bool globalYawFusionFreshUnlocked() const
  {
    return enable_global_gnss_cog_fusion_ && have_global_odom_ &&
      last_cog_fusion_pub_time_.nanoseconds() > 0 &&
      last_global_odom_time_.nanoseconds() > 0 &&
      last_global_odom_time_ >= last_cog_fusion_pub_time_ &&
      (now() - last_cog_fusion_pub_time_).seconds() >= 0.0 &&
      (now() - last_cog_fusion_pub_time_).seconds() <= gnss_fusion_measurement_timeout_sec_;
  }

  void publishGatedFusionMeasurementsUnlocked(const rclcpp::Time & stamp)
  {
    gnss_velocity_fusion_active_ = false;
    gnss_cog_fusion_active_ = false;
    if (!fusionStampFreshUnlocked(stamp)) return;

    // Feed qualified GNSS-derived base-frame vx to both EKFs. vy and GNSS-derived
    // yaw-rate remain diagnostic. Absolute GNSS heading is published separately as
    // motion-qualified COG and is fused only by the global EKF.
    if (enable_global_gnss_velocity_fusion_ && velocityCertificationAllowsFusionUnlocked() &&
        gnss_velocity_qualified_ && have_last_base_velocity_msg_) {
      auto fused = last_base_velocity_msg_;
      fused.header.stamp = stamp;
      fused.header.frame_id = base_frame_;
      const double raw_var = std::isfinite(fused.twist.covariance[0]) ?
        fused.twist.covariance[0] : gnss_velocity_fusion_max_variance_;
      const double vx_var = std::clamp(
        raw_var, gnss_velocity_fusion_min_variance_, gnss_velocity_fusion_max_variance_);
      fused.twist.covariance[0] = vx_var;
      // Keep lateral velocity available for inspection but deliberately uncertain;
      // robot_localization is configured not to fuse vy for an Ackermann vehicle.
      fused.twist.covariance[7] = std::max(
        std::isfinite(fused.twist.covariance[7]) ? fused.twist.covariance[7] : vx_var,
        gnss_velocity_fusion_min_variance_);
      fused.twist.covariance[35] = gnss_yaw_rate_valid_ ?
        std::clamp(gnss_yaw_rate_variance_, gnss_yaw_rate_min_variance_, gnss_yaw_rate_max_variance_) :
        1.0e6;
      if (stamp.nanoseconds() != last_velocity_fusion_stamp_ns_) {
        gnss_velocity_fusion_pub_->publish(fused);
        last_velocity_fusion_stamp_ns_ = stamp.nanoseconds();
        last_velocity_fusion_pub_time_ = now();
      }
      gnss_velocity_fusion_active_ = true;
    }

    // COG is an absolute direction-of-motion measurement. Publish it only after
    // Part-2 qualification and only while moving forward/straight. It is encoded
    // as PoseWithCovarianceStamped in map so global EKF can fuse yaw only.
    if (enable_global_gnss_cog_fusion_ && cogCertificationAllowsFusionUnlocked() &&
        cog_motion_qualified_ && std::isfinite(quality_.course_enu_rad)) {
      geometry_msgs::msg::PoseWithCovarianceStamped cog;
      cog.header.stamp = stamp;
      cog.header.frame_id = map_frame_;
      cog.pose.pose.orientation = quaternionFromYaw(navigation_math::normalizeAngle(
        quality_.course_enu_rad + map_yaw_from_enu_rad_ + map_calibration_.yaw));
      for (auto & v : cog.pose.covariance) v = 1.0e6;
      const double head_var_raw = std::isfinite(quality_.course_accuracy_rad) ?
        quality_.course_accuracy_rad * quality_.course_accuracy_rad :
        gnss_cog_fusion_max_variance_rad2_;
      cog.pose.covariance[35] = std::clamp(
        head_var_raw, gnss_cog_fusion_min_variance_rad2_,
        gnss_cog_fusion_max_variance_rad2_);
      if (stamp.nanoseconds() != last_cog_fusion_stamp_ns_) {
        gnss_cog_fusion_pub_->publish(cog);
        last_cog_fusion_stamp_ns_ = stamp.nanoseconds();
        last_cog_fusion_pub_time_ = now();
      }
      gnss_cog_fusion_active_ = true;
    }

    std_msgs::msg::Bool vb; vb.data = gnss_velocity_fusion_active_;
    gnss_velocity_fusion_active_pub_->publish(vb);
    std_msgs::msg::Bool cb; cb.data = gnss_cog_fusion_active_;
    gnss_cog_fusion_active_pub_->publish(cb);

    std::ostringstream fs;
    fs << std::fixed << std::setprecision(6)
       << "{\"velocity_enabled\":" << (enable_global_gnss_velocity_fusion_ ? "true" : "false")
       << ",\"cog_enabled\":" << (enable_global_gnss_cog_fusion_ ? "true" : "false")
       << ",\"velocity_certified\":" << (gnss_velocity_calibration_valid_ ? "true" : "false")
       << ",\"cog_certified\":" << (gnss_cog_calibration_valid_ ? "true" : "false")
       << ",\"velocity_active\":" << (gnss_velocity_fusion_active_ ? "true" : "false")
       << ",\"cog_active\":" << (gnss_cog_fusion_active_ ? "true" : "false")
       << ",\"global_yaw_fresh\":" << (globalYawFusionFreshUnlocked() ? "true" : "false")
       << ",\"stamp_sec\":" << stamp.seconds() << "}";
    std_msgs::msg::String sm; sm.data = fs.str(); gnss_fusion_status_pub_->publish(sm);
  }

  // Publish a compact diagnostic snapshot and qualification flags. Part 3 also
  // emits dedicated gated topics for global EKF, while raw Part-2 topics remain
  // diagnostic-only and can never bypass qualification.
  void publishMotionValidationUnlocked(const rclcpp::Time & stamp)
  {
    const bool vel_fresh = gnssVelocityFreshUnlocked();
    const bool fit_fresh = gnssFitFreshUnlocked();
    const double antenna_speed = std::hypot(gnss_enu_ve_mps_, gnss_enu_vn_mps_);
    const double base_speed = std::hypot(gnss_base_vx_mps_, gnss_base_vy_mps_);
    const bool speed_active = antenna_speed >= gnss_velocity_min_validation_speed_mps_;
    const bool quality_ok = quality_.source == 1 && qualityFreshUnlocked() &&
      (correctionQualityPassesUnlocked() || degradedQualityPassesUnlocked()) &&
      (quality_.nav_cov_vel_valid || gnss_last_velocity_covariance_valid_) &&
      std::isfinite(quality_.measurement_age_sec) && quality_.measurement_age_sec >= 0.0 &&
      quality_.measurement_age_sec <= gnss_max_measurement_age_sec_ &&
      std::isfinite(quality_.sacc_mps) && quality_.sacc_mps >= 0.0 &&
      quality_.sacc_mps <= cog_max_sacc_mps_;
    const bool vector_speed_ok = !std::isfinite(quality_.ground_speed_mps) ||
      std::abs(antenna_speed - quality_.ground_speed_mps) <= gnss_speed_consistency_max_mps_;

    const double vel_course_enu = std::atan2(gnss_enu_vn_mps_, gnss_enu_ve_mps_);
    const double cog_vel_residual = (speed_active && std::isfinite(quality_.course_enu_rad)) ?
      navigation_math::normalizeAngle(vel_course_enu - quality_.course_enu_rad) :
      std::numeric_limits<double>::quiet_NaN();
    const bool cog_vel_ok = !speed_active || !std::isfinite(cog_vel_residual) ||
      std::abs(cog_vel_residual) <= gnss_cog_velocity_course_max_rad_;

    const double fit_speed_residual = fit_fresh ? (gnss_fit_speed_mps_ - antenna_speed) :
      std::numeric_limits<double>::quiet_NaN();
    const double cog_fit_residual = (fit_fresh && std::isfinite(quality_.course_enu_rad)) ?
      navigation_math::normalizeAngle(gnss_fit_course_enu_rad_ - quality_.course_enu_rad) :
      std::numeric_limits<double>::quiet_NaN();
    const bool fit_speed_ok = !fit_fresh ||
      std::abs(fit_speed_residual) <= gnss_fit_speed_residual_max_mps_;
    const bool fit_course_ok = !fit_fresh || !speed_active || !std::isfinite(cog_fit_residual) ||
      std::abs(cog_fit_residual) <= gnss_cog_fit_course_max_rad_;

    const bool lateral_ok = std::abs(gnss_base_vy_mps_) <= gnss_lateral_velocity_warn_mps_;
    const double wheel_residual = local_forward_at_gnss_mps_ - gnss_base_vx_mps_;
    wheel_gnss_residual_mps_ = wheel_residual;
    wheel_slip_motion_detected_ = have_local_motion_at_gnss_ && vel_fresh && quality_ok &&
      std::abs(wheel_residual) >= wheel_gnss_slip_residual_mps_ &&
      std::max(std::abs(local_forward_at_gnss_mps_), std::abs(gnss_base_vx_mps_)) >=
        slip_min_wheel_speed_mps_;

    gnss_velocity_qualified_ = vel_fresh && quality_ok && gnss_last_velocity_covariance_valid_ &&
      vector_speed_ok && cog_vel_ok && lateral_ok;
    if (fit_fresh) gnss_velocity_qualified_ = gnss_velocity_qualified_ && fit_speed_ok;

    // COG heading qualification must NOT depend on the current map/body yaw projection.
    // If the startup magnetic seed is wrong by ~90 deg, gnss_base_vy becomes large by
    // construction; requiring lateral_ok/gnss_velocity_qualified here would create a
    // circular gate where the correct GNSS COG can never repair the wrong heading.
    // For absolute heading bootstrap we instead require receiver quality, positive
    // forward motion, straight local gyro, and mutually consistent GNSS Doppler/course.
    // Base-frame velocity fusion remains stricter below and will only qualify after
    // the heading has converged enough for lateral velocity/residual checks to pass.
    const bool raw_cog_candidate = have_local_motion_at_gnss_ && vel_fresh && quality_ok &&
      speed_active && vector_speed_ok &&
      local_forward_at_gnss_mps_ >= cog_min_forward_speed_mps_ &&
      std::isfinite(quality_.course_enu_rad) &&
      std::isfinite(quality_.course_accuracy_rad) &&
      quality_.course_accuracy_rad <= cog_max_heading_accuracy_rad_ &&
      std::abs(local_yaw_rate_at_gnss_rps_) <= cog_max_local_yaw_rate_rps_ &&
      cog_vel_ok && fit_course_ok;

    const auto t = now();
    if (raw_cog_candidate) {
      cog_bad_since_.reset();
      if (!cog_candidate_since_.has_value()) cog_candidate_since_ = t;
      if ((t - *cog_candidate_since_).seconds() >= cog_valid_hold_sec_) cog_motion_qualified_ = true;
    } else {
      cog_candidate_since_.reset();
      if (!cog_bad_since_.has_value()) cog_bad_since_ = t;
      if ((t - *cog_bad_since_).seconds() >= cog_invalid_hold_sec_) cog_motion_qualified_ = false;
    }

    std_msgs::msg::Bool vb; vb.data = gnss_velocity_qualified_; gnss_velocity_qualified_pub_->publish(vb);
    std_msgs::msg::Bool cb; cb.data = cog_motion_qualified_; gnss_cog_qualified_pub_->publish(cb);
    publishGatedFusionMeasurementsUnlocked(stamp);
    std_msgs::msg::Float64 sr; sr.data = wheel_residual; gnss_speed_residual_pub_->publish(sr);
    std_msgs::msg::Float64 cr; cr.data = std::isfinite(cog_vel_residual) ? cog_vel_residual : 0.0;
    gnss_course_residual_pub_->publish(cr);

    const char * confidence = gnss_velocity_qualified_ ? (cog_motion_qualified_ ? "HIGH" : "MEDIUM") : "LOW";
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6)
       << "{\"confidence\":\"" << confidence << "\""
       << ",\"velocity_qualified\":" << (gnss_velocity_qualified_ ? "true" : "false")
       << ",\"cog_qualified\":" << (cog_motion_qualified_ ? "true" : "false")
       << ",\"velocity_fusion_enabled\":" << (enable_global_gnss_velocity_fusion_ ? "true" : "false")
       << ",\"cog_fusion_enabled\":" << (enable_global_gnss_cog_fusion_ ? "true" : "false")
       << ",\"velocity_fusion_active\":" << (gnss_velocity_fusion_active_ ? "true" : "false")
       << ",\"cog_fusion_active\":" << (gnss_cog_fusion_active_ ? "true" : "false")
       << ",\"wheel_slip\":" << (wheel_slip_motion_detected_ ? "true" : "false")
       << ",\"sync_gap_sec\":" << gnss_last_sync_gap_sec_
       << ",\"yaw_at_measurement_rad\":" << gnss_map_yaw_at_measurement_rad_
       << ",\"yaw_rate_at_measurement_rps\":" << local_yaw_rate_at_gnss_rps_
       << ",\"gnss_vyaw_rps\":" << gnss_yaw_rate_rps_
       << ",\"gnss_vyaw_valid\":" << (gnss_yaw_rate_valid_ ? "true" : "false")
       << ",\"local_motion_available\":" << (have_local_motion_at_gnss_ ? "true" : "false")
       << ",\"wheel_vx_at_measurement_mps\":" << local_forward_at_gnss_mps_
       << ",\"vel_e_mps\":" << gnss_enu_ve_mps_
       << ",\"vel_n_mps\":" << gnss_enu_vn_mps_
       << ",\"vel_map_x_mps\":" << gnss_map_vx_mps_
       << ",\"vel_map_y_mps\":" << gnss_map_vy_mps_
       << ",\"vel_base_x_mps\":" << gnss_base_vx_mps_
       << ",\"vel_base_y_mps\":" << gnss_base_vy_mps_
       << ",\"antenna_speed_mps\":" << antenna_speed
       << ",\"base_speed_mps\":" << base_speed
       << ",\"wheel_minus_gnss_mps\":" << wheel_residual
       << ",\"cog_minus_vel_course_rad\":" << (std::isfinite(cog_vel_residual) ? -cog_vel_residual : 999.0)
       << ",\"fit_minus_gnss_speed_mps\":" << (std::isfinite(fit_speed_residual) ? fit_speed_residual : 999.0)
       << ",\"fit_minus_cog_rad\":" << (std::isfinite(cog_fit_residual) ? cog_fit_residual : 999.0)
       << ",\"fit_fresh\":" << (fit_fresh ? "true" : "false")
       << ",\"quality_fresh\":" << (qualityFreshUnlocked() ? "true" : "false")
       << ",\"nav_cov_vel_valid\":" << (quality_.nav_cov_vel_valid ? "true" : "false")
       << ",\"velocity_covariance_valid\":" << (gnss_last_velocity_covariance_valid_ ? "true" : "false")
       << ",\"velocity_certified\":" << (gnss_velocity_calibration_valid_ ? "true" : "false")
       << ",\"cog_certified\":" << (gnss_cog_calibration_valid_ ? "true" : "false")
       << ",\"reject_reason\":\"" << gnss_last_motion_reject_reason_ << "\""
       << ",\"timestamp_reject_count\":" << gnss_timestamp_reject_count_
       << ",\"covariance_reject_count\":" << gnss_covariance_reject_count_
       << ",\"sync_reject_count\":" << gnss_sync_reject_count_
       << ",\"stamp_sec\":" << stamp.seconds() << "}";
    std_msgs::msg::String msg; msg.data = ss.str(); gnss_motion_validation_pub_->publish(msg);
  }

  void onGnssVelocity(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    const double ve = msg->twist.twist.linear.x;
    const double vn = msg->twist.twist.linear.y;
    if (!std::isfinite(ve) || !std::isfinite(vn)) return;

    std::lock_guard<std::mutex> lock(mutex_);
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::string stamp_reason;
    if (!gnssMeasurementStampUsableUnlocked(msg->header.stamp, stamp, stamp_reason)) {
      ++gnss_timestamp_reject_count_;
      gnss_last_motion_reject_reason_ = stamp_reason;
      gnss_velocity_qualified_ = false;
      return;
    }
    gnss_last_velocity_covariance_valid_ = validVelocityCovarianceUnlocked(*msg);
    if (!gnss_last_velocity_covariance_valid_) {
      ++gnss_covariance_reject_count_;
      gnss_last_motion_reject_reason_ = "invalid_velocity_covariance";
    } else {
      gnss_last_motion_reject_reason_.clear();
    }

    // GNSS velocity must remain usable even when the ESC serial link is absent.
    // Before map anchoring, magnetic yaw is only a startup seed. After anchoring, the
    // synchronized local EKF gyro heading rotates ENU velocity into the vehicle frame.
    // COG qualification below is deliberately independent of this projection so a bad
    // startup seed cannot block its own correction.
    const bool imu_heading_fresh = imu_orientation_valid_ &&
      last_imu_orientation_time_.nanoseconds() > 0 &&
      (now() - last_imu_orientation_time_).seconds() >= 0.0 &&
      (now() - last_imu_orientation_time_).seconds() <= imu_timeout_sec_;
    if (!imu_heading_fresh) {
      gnss_last_motion_reject_reason_ = "imu_yaw_unavailable";
      gnss_velocity_qualified_ = false;
      return;
    }

    LocalMotionSample local;
    have_local_motion_at_gnss_ = localStateAtUnlocked(stamp, local);
    if (have_local_motion_at_gnss_) {
      gnss_last_sync_gap_sec_ = std::abs((stamp - local.stamp).seconds());
      local_forward_at_gnss_mps_ = local.vx_mps;
      local_yaw_rate_at_gnss_rps_ = local.yaw_rate_rps;
    } else {
      // No rejection: this is the expected state when ESC is disabled/offline
      // before the local EKF has started publishing.
      gnss_last_sync_gap_sec_ = 999.0;
      local_forward_at_gnss_mps_ = 0.0;
      local_yaw_rate_at_gnss_rps_ = 0.0;
    }

    gnss_enu_ve_mps_ = ve;
    gnss_enu_vn_mps_ = vn;
    const double antenna_speed = std::hypot(ve, vn);
    const double course_enu = std::atan2(vn, ve);

    // Derive GNSS vyaw from consecutive Doppler velocity directions. This is not
    // an absolute heading source; absolute yaw remains exclusively IMU in ekf.yaml.
    gnss_yaw_rate_valid_ = false;
    if (antenna_speed >= gnss_yaw_rate_min_speed_mps_) {
      if (last_gnss_velocity_course_enu_rad_.has_value() &&
          last_gnss_velocity_course_stamp_.nanoseconds() > 0) {
        const double dt = (stamp - last_gnss_velocity_course_stamp_).seconds();
        if (dt > 1.0e-3 && dt <= std::max(2.0, 3.0 * gnss_velocity_timeout_sec_)) {
          const double raw_rate = navigation_math::normalizeAngle(
            course_enu - *last_gnss_velocity_course_enu_rad_) / dt;
          if (std::isfinite(raw_rate) && std::abs(raw_rate) <= gnss_yaw_rate_max_abs_rps_) {
            if (!have_gnss_yaw_rate_filter_) {
              gnss_yaw_rate_rps_ = raw_rate;
              have_gnss_yaw_rate_filter_ = true;
            } else {
              gnss_yaw_rate_rps_ += gnss_yaw_rate_filter_alpha_ *
                (raw_rate - gnss_yaw_rate_rps_);
            }
            double course_sigma = std::numeric_limits<double>::quiet_NaN();
            if (std::isfinite(quality_.course_accuracy_rad) && quality_.course_accuracy_rad > 0.0) {
              course_sigma = quality_.course_accuracy_rad;
            } else if (gnss_last_velocity_covariance_valid_) {
              const double sigma_v = std::sqrt(std::max(
                msg->twist.covariance[0], msg->twist.covariance[7]));
              course_sigma = sigma_v / std::max(antenna_speed, gnss_yaw_rate_min_speed_mps_);
            }
            const double raw_var = std::isfinite(course_sigma) ?
              (2.0 * course_sigma * course_sigma / (dt * dt)) :
              gnss_yaw_rate_max_variance_;
            gnss_yaw_rate_variance_ = std::clamp(
              raw_var, gnss_yaw_rate_min_variance_, gnss_yaw_rate_max_variance_);
            gnss_yaw_rate_valid_ = true;
          }
        }
      }
      last_gnss_velocity_course_enu_rad_ = course_enu;
      last_gnss_velocity_course_stamp_ = stamp;
    } else {
      // COG direction becomes noisy near standstill. Reset the derivative history
      // and advertise a huge covariance so robot_localization effectively ignores vyaw.
      last_gnss_velocity_course_enu_rad_.reset();
      last_gnss_velocity_course_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      have_gnss_yaw_rate_filter_ = false;
      gnss_yaw_rate_rps_ = 0.0;
      gnss_yaw_rate_variance_ = 1.0e6;
    }

    // Magnetometer is reliable as a stationary absolute seed but can be disturbed
    // by steering/traction motor current. Once map->odom is anchored, rotate GNSS
    // velocity with the synchronized local EKF heading (gyro-integrated yaw) instead
    // of the live magnetic yaw. This prevents motor magnetic fields from creating a
    // false heading correction during motion.
    if (anchor_valid_ && have_local_motion_at_gnss_) {
      gnss_map_yaw_at_measurement_rad_ = navigation_math::normalizeAngle(
        anchor_map_odom_.yaw + local.pose.yaw);
    } else {
      gnss_map_yaw_at_measurement_rad_ = navigation_math::normalizeAngle(
        imu_yaw_enu_rad_ + map_yaw_from_enu_rad_ + map_calibration_.yaw);
    }

    const double theta = map_yaw_from_enu_rad_ + map_calibration_.yaw;
    const double c = std::cos(theta), sn = std::sin(theta);
    const double ant_map_x = c * ve - sn * vn;
    const double ant_map_y = sn * ve + c * vn;

    const double cy = std::cos(gnss_map_yaw_at_measurement_rad_);
    const double sy = std::sin(gnss_map_yaw_at_measurement_rad_);
    const double rx = cy * antenna_x_m_ - sy * antenna_y_m_;
    const double ry = sy * antenna_x_m_ + cy * antenna_y_m_;
    // v_ant = v_base + omega x r. GNSS-derived vyaw is used for the lever arm;
    // if COG rate is not observable yet, omit this small correction safely.
    const double omega = gnss_yaw_rate_valid_ ? gnss_yaw_rate_rps_ : 0.0;
    gnss_map_vx_mps_ = ant_map_x + omega * ry;
    gnss_map_vy_mps_ = ant_map_y - omega * rx;
    gnss_base_vx_mps_ = cy * gnss_map_vx_mps_ + sy * gnss_map_vy_mps_;
    gnss_base_vy_mps_ = -sy * gnss_map_vx_mps_ + cy * gnss_map_vy_mps_;

    const double a = msg->twist.covariance[0];
    const double b = msg->twist.covariance[1];
    const double d = msg->twist.covariance[7];
    auto rotCov = [](double ca, double sa, double xx, double xy, double yy,
                     double & ox, double & oxy, double & oy) {
      ox = ca*ca*xx - 2.0*ca*sa*xy + sa*sa*yy;
      oxy = ca*sa*(xx-yy) + (ca*ca-sa*sa)*xy;
      oy = sa*sa*xx + 2.0*ca*sa*xy + ca*ca*yy;
    };
    double map_xx=0.0,map_xy=0.0,map_yy=0.0;
    rotCov(c, sn, a, b, d, map_xx, map_xy, map_yy);
    double body_xx=0.0,body_xy=0.0,body_yy=0.0;
    rotCov(cy, -sy, map_xx, map_xy, map_yy, body_xx, body_xy, body_yy);

    geometry_msgs::msg::TwistWithCovarianceStamped map_msg = *msg;
    map_msg.header.frame_id = map_frame_;
    map_msg.twist.twist.linear.x = gnss_map_vx_mps_;
    map_msg.twist.twist.linear.y = gnss_map_vy_mps_;
    map_msg.twist.twist.angular.z = gnss_yaw_rate_valid_ ? gnss_yaw_rate_rps_ : 0.0;
    map_msg.twist.covariance[0]=map_xx; map_msg.twist.covariance[1]=map_xy;
    map_msg.twist.covariance[6]=map_xy; map_msg.twist.covariance[7]=map_yy;
    map_msg.twist.covariance[35] = gnss_yaw_rate_valid_ ? gnss_yaw_rate_variance_ : 1.0e6;
    gnss_vel_map_pub_->publish(map_msg);

    geometry_msgs::msg::TwistWithCovarianceStamped base_msg = *msg;
    base_msg.header.frame_id = base_frame_;
    base_msg.twist.twist.linear.x = gnss_base_vx_mps_;
    base_msg.twist.twist.linear.y = gnss_base_vy_mps_;
    base_msg.twist.twist.angular.z = gnss_yaw_rate_valid_ ? gnss_yaw_rate_rps_ : 0.0;
    base_msg.twist.covariance[0]=body_xx; base_msg.twist.covariance[1]=body_xy;
    base_msg.twist.covariance[6]=body_xy; base_msg.twist.covariance[7]=body_yy;
    base_msg.twist.covariance[35] = gnss_yaw_rate_valid_ ? gnss_yaw_rate_variance_ : 1.0e6;
    gnss_base_vel_pub_->publish(base_msg);
    last_base_velocity_msg_ = base_msg;
    have_last_base_velocity_msg_ = true;

    last_accepted_gnss_velocity_stamp_ = stamp;
    last_gnss_velocity_stamp_ = stamp;
    last_gnss_velocity_time_ = now();
    publishMotionValidationUnlocked(stamp);
  }

  void onGnssVelocityFit(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    const double ve = msg->twist.twist.linear.x;
    const double vn = msg->twist.twist.linear.y;
    if (!std::isfinite(ve) || !std::isfinite(vn)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    gnss_fit_speed_mps_ = std::hypot(ve, vn);
    gnss_fit_course_enu_rad_ = std::atan2(vn, ve);
    last_gnss_fit_stamp_ = stampOrNow(msg->header.stamp);
    last_gnss_fit_time_ = now();
    if (gnssVelocityFreshUnlocked()) {
      publishMotionValidationUnlocked(last_gnss_velocity_stamp_);
    }
  }

  void createInterfaces()
  {
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    gnss_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      raw_gnss_topic_, sensor_qos,
      std::bind(&LocalizationCore::onGnss, this, std::placeholders::_1));
    quality_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      quality_topic_, sensor_qos,
      std::bind(&LocalizationCore::onQuality, this, std::placeholders::_1));
    gnss_velocity_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      gnss_velocity_topic_, sensor_qos,
      std::bind(&LocalizationCore::onGnssVelocity, this, std::placeholders::_1));
    gnss_velocity_fit_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      gnss_velocity_fit_topic_, sensor_qos,
      std::bind(&LocalizationCore::onGnssVelocityFit, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      std::bind(&LocalizationCore::onImu, this, std::placeholders::_1));
    esc_kinematic_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
      esc_kinematic_yaw_rate_topic_, sensor_qos,
      std::bind(&LocalizationCore::onEscKinematicYawRate, this, std::placeholders::_1));
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      "/imu/mag", sensor_qos,
      std::bind(&LocalizationCore::onMag, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_, sensor_qos,
      std::bind(&LocalizationCore::onOdom, this, std::placeholders::_1));
    global_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      global_odom_topic_, sensor_qos,
      std::bind(&LocalizationCore::onGlobalOdom, this, std::placeholders::_1));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", command_qos,
      std::bind(&LocalizationCore::onInitialPose, this, std::placeholders::_1));

    planning_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/system/planning_localization_ready", stateQos());
    motion_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/system/motion_localization_ready", stateQos());
    mode_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/localization_state", stateQos());
    map_yaw_offset_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/localization/map_yaw_from_enu", stateQos());
    gnss_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/gnss_status", stateQos());
    imu_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/imu_status", stateQos());
    ekf_local_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/ekf_local_status", stateQos());
    ekf_global_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/ekf_global_status", stateQos());
    gnss_map_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odometry/gnss_map", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    gnss_vel_map_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/gnss/vel_map", rclcpp::SensorDataQoS().keep_last(5));
    gnss_base_vel_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/gnss/base_velocity", rclcpp::SensorDataQoS().keep_last(5));
    gnss_velocity_fusion_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      gnss_velocity_fusion_topic_, rclcpp::SensorDataQoS().keep_last(5));
    gnss_cog_fusion_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      gnss_cog_fusion_topic_, rclcpp::SensorDataQoS().keep_last(5));
    gnss_velocity_fusion_active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/gnss/velocity_fusion_active", stateQos());
    gnss_cog_fusion_active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/gnss/cog_fusion_active", stateQos());
    gnss_fusion_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/gnss/fusion_status", stateQos());
    gnss_motion_validation_pub_ = create_publisher<std_msgs::msg::String>(
      "/gnss/motion_validation", stateQos());
    gnss_velocity_qualified_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/gnss/velocity_qualified", stateQos());
    gnss_cog_qualified_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/gnss/cog_qualified", stateQos());
    gnss_speed_residual_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/gnss/speed_residual", stateQos());
    gnss_course_residual_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/gnss/course_residual", stateQos());
    pose_estimator_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/pose_estimator_map", stateQos());
    pose_estimator_status_pub_ = create_publisher<std_msgs::msg::String>(
      "/system/pose_estimator", stateQos());

    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/reset_anchor",
      std::bind(
        &LocalizationCore::onResetAnchor, this,
        std::placeholders::_1, std::placeholders::_2));
    capture_pose_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/capture_pose_estimator",
      std::bind(
        &LocalizationCore::onCapturePoseEstimator, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_calibration_samples_service_ = create_service<std_srvs::srv::Trigger>(
      "/localization/reset_calibration_samples",
      std::bind(
        &LocalizationCore::onResetCalibrationSamples, this,
        std::placeholders::_1, std::placeholders::_2));

    tf_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / tf_publish_rate_hz_)),
      std::bind(&LocalizationCore::onTfTimer, this));
    status_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / status_publish_rate_hz_)),
      std::bind(&LocalizationCore::publishState, this));
  }

  // Store the append-only /gnss/quality contract. Stage 2 consumes the original
  // 0..8 quality prefix plus iTOW/NAV-COV/PVT-rate/measurement-age metadata while
  // keeping all later Driver-V2 fields backward compatible for GUI/reporting.
  void onQuality(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6U) return;
    std::lock_guard<std::mutex> lock(mutex_);
    quality_.received = true;
    quality_.satellites = static_cast<int>(std::lround(msg->data[0]));
    quality_.dop = msg->data[1];
    quality_.hacc_m = msg->data[2];
    quality_.fix_metric = msg->data[3];
    quality_.source = static_cast<int>(std::lround(msg->data[4]));
    quality_.sacc_mps = msg->data[5];
    quality_.ground_speed_mps = msg->data.size() >= 7U ? msg->data[6] : -1.0;
    quality_.course_enu_rad = msg->data.size() >= 8U ? msg->data[7] :
      std::numeric_limits<double>::quiet_NaN();
    quality_.course_accuracy_rad = msg->data.size() >= 9U ? msg->data[8] :
      std::numeric_limits<double>::quiet_NaN();
    quality_.itow_ms = msg->data.size() >= 10U ? msg->data[9] : -1.0;
    quality_.nav_cov_vel_valid = msg->data.size() >= 22U && msg->data[21] > 0.5;
    quality_.pvt_rate_hz = msg->data.size() >= 23U ? msg->data[22] : 0.0;
    quality_.measurement_age_sec = msg->data.size() >= 24U ? msg->data[23] : 999.0;
    quality_.gnss_fix_ok = msg->data.size() >= 45U ? msg->data[44] > 0.5 : false;
    last_quality_time_ = now();

    const auto t = now();
    const bool strict = strictQualityPassesUnlocked();
    if (strict) {
      if (!strict_since_.has_value()) strict_since_ = t;
      last_motion_quality_acceptable_time_ = t;
      if ((t - *strict_since_).seconds() >= strict_quality_hold_sec_) {
        motion_strict_armed_ = true;
      }
    } else {
      strict_since_.reset();
      if (motionHoldQualityPassesUnlocked()) {
        last_motion_quality_acceptable_time_ = t;
      }
    }
    // /gnss/quality is published after /gnss/vel by the driver. Re-evaluate the
    // last synchronized velocity here so qualification always uses the same/latest
    // NAV-PVT quality epoch rather than being stuck one epoch behind.
    if (gnssVelocityFreshUnlocked()) {
      publishMotionValidationUnlocked(last_gnss_velocity_stamp_);
    }
  }

  // Fungsi: Menyimpan orientation IMU terbaru dan jumlah sampel startup. IMU
  // tidak di-copy ke node lain atau dipublikasikan ulang dalam frame map.
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    const bool orientation_available = usableImuOrientation(*msg);
    if (orientation_available) {
      rpyFromQuaternion(msg->orientation, roll, pitch, yaw);
      if (!navigation_math::finite3(roll, pitch, yaw)) return;
    }

    const bool gyro_available = msg->angular_velocity_covariance[0] >= 0.0 &&
      std::isfinite(msg->angular_velocity.x) &&
      std::isfinite(msg->angular_velocity.y) &&
      std::isfinite(msg->angular_velocity.z);
    // Absolute yaw is now a first-class EKF input. Do not discard a valid
    // orientation just because the gyro part of this IMU message is unavailable.
    if (!orientation_available && !gyro_available) return;

    std::lock_guard<std::mutex> lock(mutex_);
    const auto received = now();
    if (orientation_available) {
      imu_roll_rad_ = roll;
      imu_pitch_rad_ = pitch;
      imu_yaw_enu_rad_ = yaw;
      imu_orientation_valid_ = true;
      last_imu_orientation_time_ = received;
      imu_orientation_samples_seen_ = std::min(1000000, imu_orientation_samples_seen_ + 1);
    }
    if (gyro_available) {
      imu_gyro_x_rps_ = msg->angular_velocity.x;
      imu_gyro_y_rps_ = msg->angular_velocity.y;
      imu_yaw_rate_rps_ = msg->angular_velocity.z;
    }
    if (msg->linear_acceleration_covariance[0] >= 0.0 &&
        std::isfinite(msg->linear_acceleration.x) &&
        std::isfinite(msg->linear_acceleration.y) &&
        std::isfinite(msg->linear_acceleration.z)) {
      imu_accel_x_mps2_ = msg->linear_acceleration.x;
      imu_accel_y_mps2_ = msg->linear_acceleration.y;
      imu_accel_z_mps2_ = msg->linear_acceleration.z;
    }
    last_imu_time_ = received;
    imu_samples_seen_ = std::min(1000000, imu_samples_seen_ + 1);
  }

  // Model yaw Ackermann dari ESC dipertahankan sebagai sensor pembanding,
  // BUKAN sebagai yaw utama EKF. Residual model-vs-gyro berguna untuk mendeteksi
  // steering mismatch / wheel slip setelah Part 1 memindahkan yaw lokal ke IMU.
  void onEscKinematicYawRate(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    esc_kinematic_yaw_rate_rps_ = msg->data;
    last_esc_kinematic_yaw_time_ = now();
  }

  // Fungsi: Menyimpan magnetometer mentah untuk HUD. MagneticField memakai
  // Tesla; state internal disimpan sebagai microtesla agar angka mudah dibaca.
  void onMag(const sensor_msgs::msg::MagneticField::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_mag_x_ut_ = std::isfinite(msg->magnetic_field.x) ? msg->magnetic_field.x * 1.0e6 : 0.0;
    imu_mag_y_ut_ = std::isfinite(msg->magnetic_field.y) ? msg->magnetic_field.y * 1.0e6 : 0.0;
    imu_mag_z_ut_ = std::isfinite(msg->magnetic_field.z) ? msg->magnetic_field.z * 1.0e6 : 0.0;
    last_mag_time_ = now();
  }

  // Local EKF follows GNSS longitudinal velocity/yaw-rate plus absolute IMU yaw.
  // Therefore odom->base yaw is short-term inertial heading, while ESC Ackermann
  // yaw remains a diagnostic/model signal rather than the primary yaw sensor.
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    navigation_math::Pose2D pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    if (!navigation_math::finite3(pose.x, pose.y, pose.yaw)) return;

    std::lock_guard<std::mutex> lock(mutex_);
    odom_base_ = pose;
    local_forward_speed_mps_ = msg->twist.twist.linear.x;
    local_speed_mps_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    local_yaw_rate_rps_ = msg->twist.twist.angular.z;
    local_x_var_ = msg->pose.covariance[0];
    local_y_var_ = msg->pose.covariance[7];
    local_yaw_var_ = msg->pose.covariance[35];
    last_odom_time_ = now();
    have_odom_ = true;

    const rclcpp::Time measurement_stamp = stampOrNow(msg->header.stamp);
    local_motion_history_.push_back(LocalMotionSample{
      measurement_stamp, pose, msg->twist.twist.linear.x, msg->twist.twist.linear.y,
      msg->twist.twist.angular.z});
    while (!local_motion_history_.empty() &&
      (measurement_stamp - local_motion_history_.front().stamp).seconds() > gnss_motion_history_sec_) {
      local_motion_history_.pop_front();
    }
  }

  // Fungsi: Menyimpan output EKF global map-pose. EKF global tidak menerbitkan
  // TF; LocalizationCore tetap satu-satunya owner map->odom.
  void onGlobalOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    navigation_math::Pose2D pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    if (!navigation_math::finite3(pose.x, pose.y, pose.yaw)) return;

    std::lock_guard<std::mutex> lock(mutex_);
    global_ekf_pose_ = pose;
    global_ekf_speed_mps_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    global_ekf_yaw_rate_rps_ = msg->twist.twist.angular.z;
    global_x_var_ = msg->pose.covariance[0];
    global_y_var_ = msg->pose.covariance[7];
    global_yaw_var_ = msg->pose.covariance[35];
    last_global_odom_time_ = now();
    have_global_odom_ = true;
  }

  // Heading visual/map yang cepat: before anchor, IMU orientation can seed map yaw.
  // After anchor, live heading follows local EKF odom (wheel/GNSS vx + relative IMU yaw/gyro-z). Optional
  // gated GNSS COG below only adjusts map->odom slowly for long-term yaw drift.
  double fastMapHeadingUnlocked()
  {
    if (anchor_valid_ && have_odom_) {
      return navigation_math::normalizeAngle(anchor_map_odom_.yaw + odom_base_.yaw);
    }

    if (!initial_heading_latched_ && have_odom_) {
      initial_odom_yaw_rad_ = odom_base_.yaw;
      const double imu_seed = use_imu_initial_heading_ ? imu_yaw_enu_rad_ : 0.0;
      initial_map_heading_rad_ = navigation_math::normalizeAngle(
        imu_seed + map_yaw_from_enu_rad_ + map_calibration_.yaw);
      initial_heading_latched_ = true;
      RCLCPP_INFO(
        get_logger(),
        "HEADING SEED: %s map_yaw=%.2fdeg odom_yaw=%.2fdeg; "
        "live heading selanjutnya dari local EKF gyro + qualified GNSS COG correction",
        use_imu_initial_heading_ ? "IMU_ONCE" : "CONFIG_ONLY",
        initial_map_heading_rad_ * 180.0 / 3.14159265358979323846,
        initial_odom_yaw_rad_ * 180.0 / 3.14159265358979323846);
    }

    if (initial_heading_latched_ && have_odom_) {
      const double delta_local =
        navigation_math::normalizeAngle(odom_base_.yaw - initial_odom_yaw_rad_);
      return navigation_math::normalizeAngle(initial_map_heading_rad_ + delta_local);
    }

    return navigation_math::normalizeAngle(map_yaw_from_enu_rad_ + map_calibration_.yaw);
  }

  // Fungsi: Mengubah raw 3D fix ke map, mengoreksi offset antena, lalu hanya
  // mengumpulkan beberapa sampel untuk membuat anchor awal. Setelah anchor ada,
  // GNSS degraded tidak pernah menggeser TF setiap frame.
  void onGnss(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) return;
    if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) return;

    std::lock_guard<std::mutex> lock(mutex_);
    last_gnss_time_ = now();
    have_raw_fix_ = true;
    gnss_latitude_ = msg->latitude;
    gnss_longitude_ = msg->longitude;
    gnss_altitude_m_ = std::isfinite(msg->altitude) ? msg->altitude : 0.0;

    if (!have_odom_ || imu_samples_seen_ < startup_imu_samples_) return;
    if (use_imu_initial_heading_) {
      const bool orientation_fresh = imu_orientation_valid_ &&
        last_imu_orientation_time_.nanoseconds() > 0 &&
        (now() - last_imu_orientation_time_).seconds() >= 0.0 &&
        (now() - last_imu_orientation_time_).seconds() <= imu_timeout_sec_;
      if (!orientation_fresh || imu_orientation_samples_seen_ < startup_imu_samples_) {
        last_reject_reason_ = "menunggu orientation IMU fresh untuk heading awal";
        return;
      }
    }
    const bool provisional_display_ok = provisionalQualityPassesUnlocked();
    if (!provisional_display_ok && !degradedQualityPassesUnlocked() && !strictQualityPassesUnlocked()) return;

    const auto enu = navigation_math::wgs84ToEnu(
      msg->latitude, msg->longitude, reference_latitude_, reference_longitude_);
    const rclcpp::Time gnss_stamp = stampOrNow(msg->header.stamp);
    LocalMotionSample local_at_gnss;
    const bool have_time_aligned_local = localStateAtUnlocked(gnss_stamp, local_at_gnss);
    const double fast_map_heading = have_time_aligned_local ?
      mapHeadingForLocalStateUnlocked(local_at_gnss) : fastMapHeadingUnlocked();
    auto map_antenna = navigation_math::enuToMap(
      enu, reference_map_x_m_, reference_map_y_m_, map_yaw_from_enu_rad_,
      fast_map_heading - map_yaw_from_enu_rad_ - map_calibration_.yaw);
    const auto raw_map_base =
      navigation_math::antennaToBase(map_antenna, antenna_x_m_, antenna_y_m_);
    last_raw_map_base_ = raw_map_base;
    have_last_raw_map_base_ = true;

    raw_map_capture_window_.push_back(
      RawMapSample{raw_map_base, quality_.hacc_m, gnss_stamp});
    while (!raw_map_capture_window_.empty()) {
      const double age =
        (now() - raw_map_capture_window_.front().stamp).seconds();
      if (age <= std::max(10.0, 2.0 * calibration_capture_window_sec_)) break;
      raw_map_capture_window_.pop_front();
    }

    const auto map_base = applyMapCalibrationUnlocked(raw_map_base);
    if (!mapPoseInBounds(map_base)) {
      last_reject_reason_ = "GNSS di luar batas map";
      return;
    }

    publishGnssMapUnlocked(map_base, *msg);

    // Bootstrap map visualization independently from the motor-autonomy gate.
    // A sane degraded fix may establish map->odom so RViz/Nav2 can display a pose;
    // autonomous motion remains fail-closed until strictQualityHeldUnlocked().
    if (!anchor_valid_) {
      const bool strict_anchor_ok = strictQualityHeldUnlocked();
      const bool degraded_anchor_ok = !anchor_init_requires_strict_ && degradedQualityPassesUnlocked();
      if (!strict_anchor_ok && !degraded_anchor_ok) {
        if (!provisional_display_ok || !allow_provisional_map_display_) {
          startup_samples_.clear();
          return;
        }
        // One-shot provisional anchor gives the operator x/y + TF immediately.
        // It never arms autonomous motion; that still requires strict quality.
        navigation_math::Pose2D seed_pose = map_base;
        seed_pose.yaw = fastMapHeadingUnlocked();
        anchor_map_odom_ = navigation_math::mapOdomFromBase(seed_pose, odom_base_);
        anchor_valid_ = true;
        anchor_mode_ = "PROVISIONAL_DISPLAY";
        last_reject_reason_ = "provisional GNSS display only; motion gate remains closed";
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
          "PROVISIONAL map->odom display: sat=%d fix=%.0f DOP=%.2f hAcc=%.1fm gnssFixOK=%s; autonomous motion remains CLOSED",
          quality_.satellites, quality_.fix_metric, quality_.dop, quality_.hacc_m,
          quality_.gnss_fix_ok ? "true" : "false");
        return;
      }
      startup_anchor_quality_mode_ = strict_anchor_ok ? "STRICT" : "DEGRADED_BOOTSTRAP";
      // Bootstrap MUST use the calibrated raw GNSS map pose. A freshly-created
      // global EKF can legitimately be fresh while still sitting at (0,0).
      // Substituting it here races startup and can place Nav2 in lethal space.
      navigation_math::Pose2D seed_pose = map_base;
      seed_pose.yaw = fastMapHeadingUnlocked();
      startup_samples_.push_back({seed_pose, quality_.hacc_m, gnss_stamp});
      while (startup_samples_.size() > static_cast<size_t>(startup_gnss_samples_)) {
        startup_samples_.pop_front();
      }
      trySeedAnchorUnlocked();
      return;
    }

    // Upgrade a provisional display anchor as soon as a degraded/strict solution
    // becomes available. This is still independent of the strict motion gate.
    if (anchor_mode_ == "PROVISIONAL_DISPLAY" &&
        (degradedQualityPassesUnlocked() || strictQualityPassesUnlocked())) {
      navigation_math::Pose2D better = map_base;
      better.yaw = fastMapHeadingUnlocked();
      anchor_map_odom_ = navigation_math::mapOdomFromBase(better, odom_base_);
      anchor_mode_ = strictQualityPassesUnlocked() ? "STRICT_RESEED" : "DEGRADED_RESEED";
      last_reject_reason_.clear();
    }

    // Continuous correction memakai global EKF sebagai referensi utama bila fresh.
    // Global EKF hanya memfilter GNSS map-position dan tidak memakai wheel twist,
    // sehingga wheel-slip pada local EKF dapat dikoreksi kembali ke posisi absolut tanpa TF snap besar.
    if (correctionQualityPassesUnlocked()) {
      navigation_math::Pose2D map_reference = map_base;
      if (globalOdomUsableAsMapReferenceUnlocked(map_base)) {
        map_reference.x = global_ekf_pose_.x;
        map_reference.y = global_ekf_pose_.y;
      }
      map_reference.yaw = fastMapHeadingUnlocked();

      // Pair the absolute GNSS pose with local odometry from the SAME measurement
      // time. Using current odom here would turn serial/receiver latency into a
      // false map->odom translation while the vehicle is moving.
      const navigation_math::Pose2D correction_odom =
        have_time_aligned_local ? local_at_gnss.pose : odom_base_;
      const auto candidate = navigation_math::mapOdomFromBase(map_reference, correction_odom);
      const bool slip_detected = wheelSlipDetectedUnlocked();
      // Ground-speed UBX pada log lapangan beberapa kali ~0 saat ESC benar-benar
      // bergerak ~0.5 m/s. Karena itu slip tetap boleh didiagnostikkan, tetapi
      // TIDAK diberi gain pose tinggi kecuali commissioning mengaktifkannya.
      const bool slip_pose_correction =
        enable_wheel_slip_pose_correction_ && slip_detected;
      const bool stationary = vehicleStationaryUnlocked();

      double alpha = slip_pose_correction ? strict_slip_correction_alpha_ :
        (stationary ? strict_correction_alpha_ : strict_moving_correction_alpha_);
      // Once the startup anchor is valid, wheel/IMU odometry is the short-term
      // motion authority. Repeatedly pulling map->odom toward single-antenna GNSS
      // jitter while the wheel speed is zero makes a parked robot appear to slide
      // sideways and can provoke maximum-steering MPPI commands at launch.
      if (stationary && freeze_stationary_map_translation_ && !slip_pose_correction) {
        alpha = 0.0;
      }

      // Saat hanya kualitas HOLD (bukan strict), tetap koreksi tetapi lebih lembut.
      if (!strictQualityPassesUnlocked()) {
        const double hacc_scale = std::clamp(
          strict_max_hacc_m_ / std::max(strict_max_hacc_m_, quality_.hacc_m),
          0.35, 0.75);
        alpha *= hacc_scale;
      }

      auto blended =
        navigation_math::blendPose(anchor_map_odom_, candidate, alpha);
      double dx = blended.x - anchor_map_odom_.x;
      double dy = blended.y - anchor_map_odom_.y;
      const double dxy = std::hypot(dx, dy);
      if (dxy > strict_max_correction_m_ && dxy > 1.0e-9) {
        const double scale = strict_max_correction_m_ / dxy;
        dx *= scale;
        dy *= scale;
      }

      // Absolute yaw correction has exactly one owner. In Part 3 the preferred
      // path is COG -> gated pose -> global EKF (+ IMU gyro) -> map->odom. The
      // legacy direct COG path is used only when global COG fusion is disabled.
      double dyaw = 0.0;
      const double current_map_yaw = navigation_math::normalizeAngle(
        anchor_map_odom_.yaw + correction_odom.yaw);
      bool yaw_from_global_ekf = false;
      if (use_global_ekf_yaw_for_map_correction_ && globalYawFusionFreshUnlocked()) {
        const double innovation = navigation_math::normalizeAngle(
          global_ekf_pose_.yaw - current_map_yaw);
        if (std::abs(innovation) <= global_ekf_yaw_max_innovation_rad_) {
          dyaw = std::clamp(global_ekf_yaw_correction_alpha_ * innovation,
            -global_ekf_yaw_max_step_rad_, global_ekf_yaw_max_step_rad_);
          yaw_from_global_ekf = true;
        }
      } else if (!enable_global_gnss_cog_fusion_ && gnssCourseYawUsableUnlocked()) {
        const double desired_map_yaw = navigation_math::normalizeAngle(
          quality_.course_enu_rad + map_yaw_from_enu_rad_ + map_calibration_.yaw);
        const double innovation = navigation_math::normalizeAngle(desired_map_yaw - current_map_yaw);
        if (std::abs(innovation) <= cog_max_innovation_rad_) {
          dyaw = std::clamp(cog_yaw_alpha_ * innovation,
            -cog_max_yaw_step_rad_, cog_max_yaw_step_rad_);
        }
      }

      anchor_map_odom_.x += dx;
      anchor_map_odom_.y += dy;
      anchor_map_odom_.yaw = navigation_math::normalizeAngle(anchor_map_odom_.yaw + dyaw);
      anchor_mode_ = slip_pose_correction ? "STRICT_SLIP_CORRECT" :
        ((stationary && freeze_stationary_map_translation_) ? "STATIONARY_HOLD" :
         (strictQualityPassesUnlocked() ? "STRICT" : "GNSS_HOLD"));
      if (std::abs(dyaw) > 0.0) anchor_mode_ += yaw_from_global_ekf ? "+EKF_YAW" : "+COG_YAW";

      if (slip_detected) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "WHEEL/MODEL SLIP DIAG: wheel_v=%.3f GNSS_v=%.3f sAcc=%.3f "
          "ackermann_w=%.3f IMU_w=%.3f residual=%.3f pose_high_gain=%s alpha=%.3f correction=(%.2f,%.2f,%.1fdeg)",
          local_speed_mps_, quality_.ground_speed_mps, quality_.sacc_mps,
          esc_kinematic_yaw_rate_rps_, imu_yaw_rate_rps_,
          esc_kinematic_yaw_rate_rps_ - imu_yaw_rate_rps_,
          slip_pose_correction ? "ON" : "OFF", alpha,
          dx, dy, dyaw * 180.0 / 3.14159265358979323846);
      }
    }
  }

  // /initialpose dipakai sebagai GROUND-TRUTH SAMPLE map.
  // Setiap klik menyimpan pasangan RAW GNSS-map -> posisi raster yang benar.
  // Dengan >=3 lokasi fisik unik, transform rigid SE(2) diselesaikan dari
  // SELURUH observasi. STOP1==START2 tetap disimpan sebagai dua observasi.
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!have_odom_) {
      last_reject_reason_ = "initialpose ditolak: local odom belum tersedia";
      return;
    }
    // Explicit indoor commissioning path. When enabled by the launch-level
    // stage3_commissioning_mode, /initialpose is a temporary manual map anchor.
    // It is NEVER appended to persistent GNSS calibration samples.
    if (allow_manual_pose_for_motion_) {
      navigation_math::Pose2D manual_map_base;
      manual_map_base.x = msg->pose.pose.position.x;
      manual_map_base.y = msg->pose.pose.position.y;
      manual_map_base.yaw = yawFromQuaternion(msg->pose.pose.orientation);
      if (!navigation_math::finite3(manual_map_base.x, manual_map_base.y, manual_map_base.yaw) ||
          !mapPoseInBounds(manual_map_base)) {
        last_reject_reason_ = "manual initialpose tidak finite/di luar map";
        return;
      }
      if (!vehicleStationaryUnlocked()) {
        last_reject_reason_ = "manual initialpose ditolak: robot harus diam";
        return;
      }
      anchor_map_odom_ = navigation_math::mapOdomFromBase(manual_map_base, odom_base_);
      anchor_valid_ = true;
      anchor_mode_ = "MANUAL";
      initial_heading_latched_ = true;
      initial_map_heading_rad_ = manual_map_base.yaw;
      initial_odom_yaw_rad_ = odom_base_.yaw;
      last_reject_reason_.clear();
      RCLCPP_WARN(get_logger(),
        "MANUAL COMMISSIONING ANCHOR: map=(%.3f,%.3f,%.1fdeg), GNSS samples unchanged",
        manual_map_base.x, manual_map_base.y,
        manual_map_base.yaw * 180.0 / 3.14159265358979323846);
      return;
    }
    if (!have_last_raw_map_base_) {
      last_reject_reason_ = "initialpose ditolak: raw GNSS map pose belum tersedia";
      return;
    }

    const bool calibration_quality_ok =
      strictQualityHeldUnlocked() ||
      (allow_map_calibration_with_degraded_fix_ && degradedQualityPassesUnlocked());
    if (!calibration_quality_ok) {
      last_reject_reason_ =
        "initialpose ditolak: tunggu GNSS fix sehat (STRICT atau degraded-valid)";
      return;
    }
    if (!vehicleStationaryUnlocked()) {
      last_reject_reason_ = "initialpose ditolak: robot harus benar-benar diam";
      return;
    }

    navigation_math::Pose2D true_map_base;
    true_map_base.x = msg->pose.pose.position.x;
    true_map_base.y = msg->pose.pose.position.y;
    true_map_base.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    if (!navigation_math::finite3(
        true_map_base.x, true_map_base.y, true_map_base.yaw) ||
        !mapPoseInBounds(true_map_base))
    {
      last_reject_reason_ = "initialpose tidak finite/di luar map";
      return;
    }

    navigation_math::Pose2D raw_capture;
    double capture_hacc = 999.0;
    int used = 0;
    if (!robustRawCaptureUnlocked(raw_capture, capture_hacc, used)) {
      last_reject_reason_ =
        "initialpose ditolak: tunggu cukup sampel GNSS stationary untuk median capture";
      return;
    }

    navigation_math::Pose2D estimator_before;
    if (anchor_valid_ && have_odom_) {
      estimator_before =
        navigation_math::mapBaseFromOdom(anchor_map_odom_, odom_base_);
    } else {
      estimator_before = applyMapCalibrationUnlocked(raw_capture);
    }

    MapCalibrationPair pair;
    pair.raw_x = raw_capture.x;
    pair.raw_y = raw_capture.y;
    pair.true_x = true_map_base.x;
    pair.true_y = true_map_base.y;
    pair.true_yaw = true_map_base.yaw;
    pair.hacc_m = capture_hacc;
    map_calibration_pairs_.push_back(pair);
    savePersistentCalibrationSamplesUnlocked();

    const int unique = uniqueCalibrationTargetsUnlocked();
    double geometry_baseline_m = 0.0;
    double geometry_score = 0.0;
    calibrationGeometryUnlocked(geometry_baseline_m, geometry_score);

    RCLCPP_WARN(
      get_logger(),
      "CAL SAMPLE #%zu unique=%d/%d baseline=%.2fm geometry=%.3f used_gnss=%d hAcc=%.2f | "
      "GROUND_TRUTH=(%.3f,%.3f,%.1fdeg) | "
      "POSE_ESTIMATOR_BEFORE=(%.3f,%.3f,%.1fdeg) | "
      "RAW_GNSS_MAP=(%.3f,%.3f)",
      map_calibration_pairs_.size(),
      unique, map_calibration_min_unique_points_, geometry_baseline_m, geometry_score,
      used, capture_hacc,
      true_map_base.x, true_map_base.y,
      true_map_base.yaw * 180.0 / 3.14159265358979323846,
      estimator_before.x, estimator_before.y,
      estimator_before.yaw * 180.0 / 3.14159265358979323846,
      raw_capture.x, raw_capture.y);

    navigation_math::Pose2D solved;
    double rmse = 0.0;
    double diagnostic_scale = 1.0;
    const bool solved_ok =
      multi_point_map_calibration_ &&
      solveMultiPointMapCalibrationUnlocked(solved, rmse, diagnostic_scale);

    if (!solved_ok) {
      // Agar RViz langsung berada di titik ground truth saat pengambilan sampel,
      // anchor sesi ini boleh diselaraskan. Ini TIDAK mengubah raw sample yang
      // dipakai solver multi-point.
      anchor_map_odom_ =
        navigation_math::mapOdomFromBase(true_map_base, odom_base_);
      initial_heading_latched_ = true;
      initial_map_heading_rad_ = true_map_base.yaw;
      initial_odom_yaw_rad_ = odom_base_.yaw;
      anchor_valid_ = true;
      anchor_mode_ = "CAL_SAMPLE_ONLY";
      startup_samples_.clear();
      last_reject_reason_.clear();

      RCLCPP_WARN(
        get_logger(),
        "CAL SAMPLE tersimpan. Belum apply multi-point: observations=%zu unique=%d, "
        "baseline=%.2fm (min %.2f), geometry=%.3f (min %.3f), max_RMSE=%.2fm. "
        "STOP1==START2 boleh dicapture dua kali.",
        map_calibration_pairs_.size(), unique, geometry_baseline_m, map_calibration_min_baseline_m_,
        geometry_score, map_calibration_min_geometry_score_, map_calibration_max_rmse_m_);
      return;
    }

    // Scale raster dikunci 1.0. diagnostic_scale hanya untuk audit apakah
    // ada indikasi map resolution / pengukuran jarak yang tidak konsisten.
    map_calibration_ = solved;
    savePersistentMapCalibrationUnlocked();

    anchor_map_odom_ =
      navigation_math::mapOdomFromBase(true_map_base, odom_base_);
    initial_heading_latched_ = true;
    initial_map_heading_rad_ = true_map_base.yaw;
    initial_odom_yaw_rad_ = odom_base_.yaw;
    anchor_valid_ = true;
    anchor_mode_ = "MULTIPOINT_CALIBRATED";
    startup_samples_.clear();
    last_reject_reason_.clear();

    have_global_odom_ = false;
    last_global_odom_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    global_odom_ignore_until_ = now() + rclcpp::Duration::from_seconds(5.0);

    RCLCPP_WARN(
      get_logger(),
      "MULTIPOINT MAP CALIBRATED: observations=%zu unique=%d baseline=%.2fm geometry=%.3f "
      "dx=%.3f dy=%.3f dyaw=%.3fdeg RMSE=%.3fm "
      "diagnostic_scale=%.5f (APPLIED_SCALE=1.00000) "
      "current_ground_truth=(%.3f,%.3f)",
      map_calibration_pairs_.size(), unique, geometry_baseline_m, geometry_score,
      map_calibration_.x, map_calibration_.y,
      map_calibration_.yaw * 180.0 / 3.14159265358979323846,
      rmse, diagnostic_scale,
      true_map_base.x, true_map_base.y);
  }

  // Fungsi: Menghapus anchor dan memaksa inisialisasi ulang tanpa restart stack.
  void onResetAnchor(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    anchor_valid_ = false;
    anchor_mode_ = "STARTUP";
    initial_heading_latched_ = false;
    startup_samples_.clear();
    strict_since_.reset();
    motion_strict_armed_ = false;
    last_motion_quality_acceptable_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    response->success = true;
    response->message = "Anchor localization dihapus; menunggu sampel GNSS/IMU/local odom baru.";
  }

  // Fungsi: Mencoba membuat anchor dari median beberapa sampel. Spread besar
  // ditolak agar satu burst GNSS buruk tidak dijadikan origin permanen.
  void trySeedAnchorUnlocked()
  {
    if (startup_samples_.size() < static_cast<size_t>(startup_gnss_samples_)) return;

    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(startup_samples_.size());
    ys.reserve(startup_samples_.size());
    for (const auto & sample : startup_samples_) {
      xs.push_back(sample.map_base.x);
      ys.push_back(sample.map_base.y);
    }
    navigation_math::Pose2D map_base;
    map_base.x = navigation_math::median(xs);
    map_base.y = navigation_math::median(ys);
    map_base.yaw = fastMapHeadingUnlocked();

    double max_spread = 0.0;
    for (const auto & sample : startup_samples_) {
      max_spread = std::max(max_spread, navigation_math::distance2D(map_base, sample.map_base));
    }
    if (max_spread > startup_max_spread_m_) {
      last_reject_reason_ = "spread GNSS startup terlalu besar";
      startup_samples_.pop_front();
      return;
    }

    anchor_map_odom_ =
      navigation_math::mapOdomFromBase(map_base, odom_base_);
    anchor_valid_ = true;
    anchor_mode_ = startup_anchor_quality_mode_;
    startup_samples_.clear();
    last_reject_reason_.clear();
    RCLCPP_INFO(
      get_logger(),
      "Anchor map->odom terkunci dari %d sampel %s: x=%.3f y=%.3f yaw=%.3f spread<=%.2fm",
      startup_gnss_samples_, anchor_mode_.c_str(), anchor_map_odom_.x, anchor_map_odom_.y,
      anchor_map_odom_.yaw, startup_max_spread_m_);
  }

  // Very loose horizontal-position gate used only to make map x/y and TF visible
  // while the receiver is converging. NEVER used by the autonomous motion gate.
  bool provisionalQualityPassesUnlocked() const
  {
    if (!allow_provisional_map_display_ || !quality_.received) return false;
    if (quality_.satellites < provisional_min_satellites_) return false;
    if (!std::isfinite(quality_.dop) || quality_.dop <= 0.0 || quality_.dop > provisional_max_dop_) return false;
    if (!std::isfinite(quality_.hacc_m) || quality_.hacc_m <= 0.0 || quality_.hacc_m > provisional_max_hacc_m_) return false;
    if (quality_.source == 1 && quality_.fix_metric < 2.0) return false;
    if (quality_.source != 1 && quality_.fix_metric <= 0.0) return false;
    return true;
  }

  // Fungsi: Quality gate longgar hanya untuk membuat pose planning yang stabil.
  // Tidak pernah dipakai untuk membuka motor autonomous.
  bool degradedQualityPassesUnlocked() const
  {
    if (!allow_degraded_planning_ || !quality_.received) return false;
    if (quality_.satellites < degraded_min_satellites_) return false;
    if (!std::isfinite(quality_.dop) || quality_.dop > degraded_max_dop_) return false;
    if (!std::isfinite(quality_.hacc_m) || quality_.hacc_m <= 0.0 || quality_.hacc_m > degraded_max_hacc_m_) return false;
    // UBX: fix type 3/4 = 3D/GNSS+DR. NMEA fallback: fix quality >0.
    if (quality_.source == 1 && (!quality_.gnss_fix_ok || quality_.fix_metric < 3.0)) return false;
    if (quality_.source != 1 && quality_.fix_metric <= 0.0) return false;
    return true;
  }

  // Fungsi: Quality gate ketat untuk autonomous fisik. Threshold sengaja sama
  // dengan target proyek lama agar refactor tidak diam-diam menurunkan safety.
  bool strictQualityPassesUnlocked() const
  {
    if (!quality_.received) return false;
    if (quality_.satellites < strict_min_satellites_) return false;
    if (!std::isfinite(quality_.dop) || quality_.dop > strict_max_dop_) return false;
    if (!std::isfinite(quality_.hacc_m) || quality_.hacc_m <= 0.0 || quality_.hacc_m > strict_max_hacc_m_) return false;
    if (quality_.source == 1 && (!quality_.gnss_fix_ok || quality_.fix_metric < 3.0)) return false;
    if (quality_.source != 1 && quality_.fix_metric <= 0.0) return false;
    return true;
  }

  // Hysteresis setelah autonomous pernah memperoleh strict fix.
  bool motionHoldQualityPassesUnlocked() const
  {
    if (!quality_.received) return false;
    if (quality_.satellites < motion_hold_min_satellites_) return false;
    if (!std::isfinite(quality_.dop) || quality_.dop > motion_hold_max_dop_) return false;
    if (!std::isfinite(quality_.hacc_m) || quality_.hacc_m <= 0.0 ||
        quality_.hacc_m > motion_hold_max_hacc_m_) return false;
    if (quality_.source == 1 && (!quality_.gnss_fix_ok || quality_.fix_metric < 3.0)) return false;
    if (quality_.source != 1 && quality_.fix_metric <= 0.0) return false;
    return true;
  }

  bool motionCriticalQualityFailsUnlocked() const
  {
    if (!quality_.received) return true;
    if (quality_.satellites < motion_critical_min_satellites_) return true;
    if (!std::isfinite(quality_.dop) || quality_.dop > motion_critical_max_dop_) return true;
    if (!std::isfinite(quality_.hacc_m) || quality_.hacc_m <= 0.0 ||
        quality_.hacc_m > motion_critical_max_hacc_m_) return true;
    if (quality_.source == 1 && (!quality_.gnss_fix_ok || quality_.fix_metric < 3.0)) return true;
    if (quality_.source != 1 && quality_.fix_metric <= 0.0) return true;
    return false;
  }

  bool motionQualityGraceUnlocked() const
  {
    if (!motion_strict_armed_ || motionCriticalQualityFailsUnlocked()) return false;
    if (last_motion_quality_acceptable_time_.nanoseconds() <= 0) return false;
    const double age = (now() - last_motion_quality_acceptable_time_).seconds();
    return age >= 0.0 && age <= motion_degrade_grace_sec_;
  }

  bool motionQualityReadyUnlocked() const
  {
    if (strictQualityHeldUnlocked()) return true;
    if (!motion_strict_armed_) return false;
    if (motionHoldQualityPassesUnlocked()) return true;
    return motionQualityGraceUnlocked();
  }

  bool gnssCourseYawUsableUnlocked() const
  {
    if (!enable_gnss_course_yaw_correction_ || quality_.source != 1) return false;
    if (!cog_motion_qualified_ || !gnssVelocityFreshUnlocked()) return false;
    if (!correctionQualityPassesUnlocked()) return false;
    if (!std::isfinite(quality_.course_enu_rad) ||
        !std::isfinite(quality_.course_accuracy_rad)) return false;
    if (!std::isfinite(quality_.ground_speed_mps) ||
        quality_.ground_speed_mps < cog_min_forward_speed_mps_) return false;
    if (!std::isfinite(local_forward_at_gnss_mps_) ||
        local_forward_at_gnss_mps_ < cog_min_forward_speed_mps_) return false;
    if (!std::isfinite(quality_.sacc_mps) || quality_.sacc_mps < 0.0 ||
        quality_.sacc_mps > cog_max_sacc_mps_) return false;
    if (quality_.course_accuracy_rad > cog_max_heading_accuracy_rad_) return false;
    if (!std::isfinite(local_yaw_rate_at_gnss_rps_) ||
        std::abs(local_yaw_rate_at_gnss_rps_) > cog_max_local_yaw_rate_rps_) return false;
    return true;
  }

  bool correctionQualityPassesUnlocked() const
  {
    return strictQualityPassesUnlocked() ||
      (motion_strict_armed_ && motionHoldQualityPassesUnlocked());
  }

  bool globalOdomFreshUnlocked() const
  {
    if (!have_global_odom_ || last_global_odom_time_.nanoseconds() <= 0) return false;
    if (global_odom_ignore_until_.nanoseconds() > 0 && now() < global_odom_ignore_until_) {
      return false;
    }
    const double age = (now() - last_global_odom_time_).seconds();
    return age >= 0.0 && age <= std::max(1.5, 2.0 * odom_timeout_sec_);
  }

  bool globalOdomUsableAsMapReferenceUnlocked(const navigation_math::Pose2D &raw_map_base)
  {
    if (!globalOdomFreshUnlocked()) return false;
    if (!std::isfinite(global_ekf_pose_.x) || !std::isfinite(global_ekf_pose_.y) ||
        !std::isfinite(raw_map_base.x) || !std::isfinite(raw_map_base.y)) return false;

    const double raw_norm = std::hypot(raw_map_base.x, raw_map_base.y);
    const double global_norm = std::hypot(global_ekf_pose_.x, global_ekf_pose_.y);
    if (raw_norm > 20.0 && global_norm < 1.0) return false;

    const double disagreement = std::hypot(
      global_ekf_pose_.x - raw_map_base.x, global_ekf_pose_.y - raw_map_base.y);
    const double hacc_tolerance = std::isfinite(quality_.hacc_m) && quality_.hacc_m > 0.0 ?
      2.0 * quality_.hacc_m : global_odom_map_reference_max_error_m_;
    const double tolerance = std::min(30.0, std::max(
      global_odom_map_reference_max_error_m_, hacc_tolerance));
    if (disagreement > tolerance) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Global EKF map reference ditolak: disagreement=%.2fm > %.2fm; raw=(%.2f,%.2f) global=(%.2f,%.2f)",
        disagreement, tolerance, raw_map_base.x, raw_map_base.y,
        global_ekf_pose_.x, global_ekf_pose_.y);
      return false;
    }
    return true;
  }

  // Fungsi: Memastikan kualitas strict stabil beberapa detik sebelum motion gate.
  bool strictQualityHeldUnlocked() const
  {
    if (!strictQualityPassesUnlocked() || !strict_since_.has_value()) return false;
    return (now() - *strict_since_).seconds() >= strict_quality_hold_sec_;
  }

  // Fungsi: Freshness input lokal agar TF tidak terus dianggap valid saat sensor mati.
  bool inputsFreshUnlocked() const
  {
    const auto t = now();
    return have_odom_ && have_raw_fix_ &&
      last_quality_time_.nanoseconds() > 0 &&
      last_imu_time_.nanoseconds() > 0 && last_odom_time_.nanoseconds() > 0 &&
      (t - last_gnss_time_).seconds() >= 0.0 &&
      (t - last_gnss_time_).seconds() <= gnss_timeout_sec_ &&
      (t - last_quality_time_).seconds() >= 0.0 &&
      (t - last_quality_time_).seconds() <= gnss_timeout_sec_ &&
      (t - last_imu_time_).seconds() >= 0.0 &&
      (t - last_imu_time_).seconds() <= imu_timeout_sec_ &&
      (t - last_odom_time_).seconds() >= 0.0 &&
      (t - last_odom_time_).seconds() <= odom_timeout_sec_;
  }

  // Stationary memakai LOCAL EKF motion. IMU tidak ikut menentukan gain
  // koreksi pose sehingga noise/drift gyro tidak mengubah perilaku TF.
  bool vehicleStationaryUnlocked() const
  {
    return std::isfinite(local_speed_mps_) && std::isfinite(local_yaw_rate_rps_) &&
      std::abs(local_speed_mps_) <= strict_stationary_speed_mps_ &&
      std::abs(local_yaw_rate_rps_) <= strict_stationary_yaw_rate_rps_;
  }

  // Slip high-gain hanya untuk kondisi seperti bench test roda diangkat:
  // GNSS ground speed ~0 tetapi wheel odom mengatakan kendaraan bergerak/berputar.
  // Saat kendaraan benar-benar bergerak, mismatch sesaat wheel-vs-IMU tidak boleh
  // lagi memicu alpha besar dan menggeser URDF terhadap planner.
  bool wheelSlipDetectedUnlocked() const
  {
    if (!correctionQualityPassesUnlocked()) return false;
    if (quality_.source != 1 || !std::isfinite(quality_.ground_speed_mps) ||
        quality_.ground_speed_mps < 0.0) return false;

    const double ground_speed = gnssVelocityFreshUnlocked() ?
      std::abs(gnss_base_vx_mps_) : quality_.ground_speed_mps;
    if (ground_speed > strict_stationary_speed_mps_) return false;

    const double wheel_speed = std::abs(local_speed_mps_);
    const double sacc =
      (std::isfinite(quality_.sacc_mps) && quality_.sacc_mps > 0.0)
      ? quality_.sacc_mps : 0.0;
    const double required_gap = std::max(
      slip_speed_difference_mps_, slip_sacc_multiplier_ * sacc);

    const bool translational_slip =
      std::isfinite(wheel_speed) &&
      wheel_speed >= slip_min_wheel_speed_mps_ &&
      (wheel_speed - ground_speed) >= required_gap;

    const bool esc_yaw_fresh = last_esc_kinematic_yaw_time_.nanoseconds() > 0 &&
      (now() - last_esc_kinematic_yaw_time_).seconds() >= 0.0 &&
      (now() - last_esc_kinematic_yaw_time_).seconds() <= esc_kinematic_yaw_timeout_sec_;
    const bool rotational_slip = esc_yaw_fresh &&
      std::isfinite(esc_kinematic_yaw_rate_rps_) && std::isfinite(imu_yaw_rate_rps_) &&
      std::max(std::abs(esc_kinematic_yaw_rate_rps_), std::abs(imu_yaw_rate_rps_)) >=
        slip_yaw_rate_difference_rps_ &&
      std::abs(esc_kinematic_yaw_rate_rps_ - imu_yaw_rate_rps_) >=
        slip_yaw_rate_difference_rps_;

    return translational_slip || rotational_slip;
  }

  // Fungsi: Validasi bounding-box map dari metadata konfigurasi; tidak menyimpan
  // OccupancyGrid besar kedua di RAM custom node.
  bool mapPoseInBounds(const navigation_math::Pose2D & pose) const
  {
    return pose.x >= map_min_x_m_ + map_bounds_margin_m_ &&
      pose.x <= map_max_x_m_ - map_bounds_margin_m_ &&
      pose.y >= map_min_y_m_ + map_bounds_margin_m_ &&
      pose.y <= map_max_y_m_ - map_bounds_margin_m_;
  }

  // Fungsi: Menerbitkan diagnostic GNSS dalam frame map dengan covariance asli.
  void publishGnssMapUnlocked(
    const navigation_math::Pose2D & map_base, const sensor_msgs::msg::NavSatFix & fix)
  {
    nav_msgs::msg::Odometry odom;
    if (fix.header.stamp.sec != 0 || fix.header.stamp.nanosec != 0) {
      odom.header.stamp = fix.header.stamp;
    } else {
      odom.header.stamp = static_cast<builtin_interfaces::msg::Time>(now());
    }
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = map_base.x;
    odom.pose.pose.position.y = map_base.y;
    odom.pose.pose.orientation = quaternionFromYaw(map_base.yaw);
    if (fix.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      odom.pose.covariance[0] = fix.position_covariance[0];
      odom.pose.covariance[7] = fix.position_covariance[4];
      odom.pose.covariance[35] = 0.10;
    } else {
      odom.pose.covariance[0] = 400.0;
      odom.pose.covariance[7] = 400.0;
      odom.pose.covariance[35] = 0.25;
    }
    gnss_map_odom_pub_->publish(odom);
  }

  // Fungsi: Broadcast TF map->odom stabil. Saat degraded, anchor membeku dan
  // hanya timestamp diperbarui; ini menghilangkan TF visualization yang menari
  // mengikuti noise GNSS seperti pada log sebelumnya.
  void onTfTimer()
  {
    navigation_math::Pose2D anchor;
    bool valid = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      valid = anchor_valid_ && have_odom_;
      anchor = anchor_map_odom_;
    }
    if (!valid) return;

    geometry_msgs::msg::TransformStamped tf;
    // Future-date the continuously stable map->odom transform slightly. At 30 Hz
    // a controller pose can be 0..33 ms newer than the latest TF sample; without
    // this bounded offset tf2 intermittently rejects an otherwise valid path.
    tf.header.stamp = now() + rclcpp::Duration::from_seconds(tf_future_offset_sec_);
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.translation.x = anchor.x;
    tf.transform.translation.y = anchor.y;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = quaternionFromYaw(anchor.yaw);
    tf_broadcaster_->sendTransform(tf);

    navigation_math::Pose2D map_pose;
    double x_var = 0.0;
    double y_var = 0.0;
    double yaw_var = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!anchor_valid_ || !have_odom_) return;
      map_pose =
        navigation_math::mapBaseFromOdom(anchor_map_odom_, odom_base_);
      x_var = global_x_var_;
      y_var = global_y_var_;
      yaw_var = local_yaw_var_;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = tf.header.stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.pose.position.x = map_pose.x;
    pose_msg.pose.pose.position.y = map_pose.y;
    pose_msg.pose.pose.position.z = 0.0;
    pose_msg.pose.pose.orientation = quaternionFromYaw(map_pose.yaw);
    pose_msg.pose.covariance[0] = std::max(0.0, x_var);
    pose_msg.pose.covariance[7] = std::max(0.0, y_var);
    pose_msg.pose.covariance[35] = std::max(0.0, yaw_var);
    pose_estimator_pub_->publish(pose_msg);
  }

  // Fungsi: Menerbitkan dua readiness yang maknanya tidak dicampur: planning
  // boleh degraded/frozen; motion localization harus strict. Status human-readable
  // tidak menumpuk kata WAIT merah untuk sensor yang sebenarnya hidup.
  void publishState()
  {
    bool planning_ready = false;
    bool motion_ready = false;
    bool global_fresh = false;
    std::string mode;
    std::string detail;
    int sats = 0;
    int fix_metric = 0;
    int gnss_source = 0;
    std::string gnss_quality = "INVALID";
    double dop = 99.9;
    double hacc = 999.0;
    double sacc = -1.0;
    double ground_speed = -1.0;
    double cog_enu = std::numeric_limits<double>::quiet_NaN();
    double cog_acc = std::numeric_limits<double>::quiet_NaN();
    bool cog_yaw_gate = false;
    double esc_kinematic_w = std::numeric_limits<double>::quiet_NaN();
    double yaw_model_imu_residual = std::numeric_limits<double>::quiet_NaN();
    bool esc_kinematic_fresh = false;
    bool wheel_slip = false;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double imu_roll = 0.0;
    double imu_pitch = 0.0;
    double imu_yaw = 0.0;
    double imu_gx = 0.0;
    double imu_gy = 0.0;
    double imu_wz = 0.0;
    double imu_ax = 0.0;
    double imu_ay = 0.0;
    double imu_az = 0.0;
    double imu_mx_ut = 0.0;
    double imu_my_ut = 0.0;
    double imu_mz_ut = 0.0;
    navigation_math::Pose2D map_base;
    navigation_math::Pose2D local_pose;
    navigation_math::Pose2D global_pose;
    double local_v = 0.0;
    double local_w = 0.0;
    double local_x_var = 0.0;
    double local_y_var = 0.0;
    double local_yaw_var = 0.0;
    double global_v = 0.0;
    double global_w = 0.0;
    double global_x_var = 0.0;
    double global_y_var = 0.0;
    double global_yaw_var = 0.0;
    bool have_odom = false;
    bool have_raw_fix = false;
    int imu_samples = 0;
    bool degraded_gate = false;
    bool imu_data_fresh = false;
    bool imu_orientation_fresh = false;
    bool mag_data_fresh = false;
    bool local_odom_fresh_status = false;
    int calibration_samples = 0;
    int calibration_unique = 0;
    double calibration_baseline_m = 0.0;
    double calibration_geometry_score = 0.0;
    bool velocity_fusion_active_status = false;
    bool cog_fusion_active_status = false;
    bool global_yaw_fusion_fresh_status = false;
    double map_yaw_from_enu_status = 0.0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto t = now();
      imu_data_fresh = last_imu_time_.nanoseconds() > 0 &&
        (t - last_imu_time_).seconds() >= 0.0 &&
        (t - last_imu_time_).seconds() <= imu_timeout_sec_;
      imu_orientation_fresh = imu_orientation_valid_ &&
        last_imu_orientation_time_.nanoseconds() > 0 &&
        (t - last_imu_orientation_time_).seconds() >= 0.0 &&
        (t - last_imu_orientation_time_).seconds() <= imu_timeout_sec_;
      mag_data_fresh = last_mag_time_.nanoseconds() > 0 &&
        (t - last_mag_time_).seconds() >= 0.0 &&
        (t - last_mag_time_).seconds() <= std::max(2.0, 2.0 * imu_timeout_sec_);
      // Setelah anchor map->odom terkunci, planning tidak boleh flap hanya karena
      // GNSS/IMU sesaat stale. Anchor DEGRADED_HOLD memang sengaja dibekukan.
      // Cukup pertahankan local EKF odom fresh untuk planning; motion fisik tetap
      // memerlukan SEMUA sensor fresh + strict quality (atau manual commissioning).
      const bool local_odom_fresh = have_odom_ && last_odom_time_.nanoseconds() > 0 &&
        (t - last_odom_time_).seconds() >= 0.0 &&
        (t - last_odom_time_).seconds() <= odom_timeout_sec_;
      local_odom_fresh_status = local_odom_fresh;
      const bool inputs_fresh = inputsFreshUnlocked();
      planning_ready = anchor_valid_ && local_odom_fresh;
      const bool manual_motion =
        anchor_mode_ == "MANUAL" && allow_manual_pose_for_motion_;
      motion_ready = planning_ready && inputs_fresh &&
        (motionQualityReadyUnlocked() || manual_motion);

      if (!anchor_valid_) {
        mode = "STARTUP";
      } else if (manual_motion) {
        mode = "MANUAL";
      } else if (strictQualityHeldUnlocked()) {
        mode = anchor_mode_;
      } else if (motion_strict_armed_ && motionHoldQualityPassesUnlocked()) {
        mode = "GNSS_HOLD";
      } else if (motionQualityGraceUnlocked()) {
        mode = "GNSS_GRACE";
      } else {
        mode = "DEGRADED_HOLD";
      }
      sats = quality_.satellites;
      fix_metric = static_cast<int>(std::lround(quality_.fix_metric));
      gnss_source = quality_.source;
      if (strictQualityPassesUnlocked()) {
        gnss_quality = "STRICT";
      } else if (motion_strict_armed_ && motionHoldQualityPassesUnlocked()) {
        gnss_quality = "HOLD";
      } else if (motionQualityGraceUnlocked()) {
        gnss_quality = "GRACE";
      } else {
        gnss_quality = degradedQualityPassesUnlocked() ? "DEGRADED" : "INVALID";
      }
      dop = quality_.dop;
      hacc = quality_.hacc_m;
      sacc = quality_.sacc_mps;
      ground_speed = quality_.ground_speed_mps;
      cog_enu = quality_.course_enu_rad;
      cog_acc = quality_.course_accuracy_rad;
      cog_yaw_gate = gnssCourseYawUsableUnlocked();
      esc_kinematic_fresh = last_esc_kinematic_yaw_time_.nanoseconds() > 0 &&
        (t - last_esc_kinematic_yaw_time_).seconds() >= 0.0 &&
        (t - last_esc_kinematic_yaw_time_).seconds() <= esc_kinematic_yaw_timeout_sec_;
      if (esc_kinematic_fresh) {
        esc_kinematic_w = esc_kinematic_yaw_rate_rps_;
        yaw_model_imu_residual = esc_kinematic_yaw_rate_rps_ - imu_yaw_rate_rps_;
      }
      wheel_slip = wheelSlipDetectedUnlocked();
      latitude = gnss_latitude_;
      longitude = gnss_longitude_;
      altitude = gnss_altitude_m_;
      imu_roll = imu_roll_rad_;
      imu_pitch = imu_pitch_rad_;
      imu_yaw = imu_yaw_enu_rad_;
      imu_gx = imu_gyro_x_rps_;
      imu_gy = imu_gyro_y_rps_;
      imu_wz = imu_yaw_rate_rps_;
      imu_ax = imu_accel_x_mps2_;
      imu_ay = imu_accel_y_mps2_;
      imu_az = imu_accel_z_mps2_;
      imu_mx_ut = imu_mag_x_ut_;
      imu_my_ut = imu_mag_y_ut_;
      imu_mz_ut = imu_mag_z_ut_;
      local_pose = odom_base_;
      local_v = local_speed_mps_;
      local_w = local_yaw_rate_rps_;
      local_x_var = local_x_var_;
      local_y_var = local_y_var_;
      local_yaw_var = local_yaw_var_;
      global_pose = global_ekf_pose_;
      global_v = global_ekf_speed_mps_;
      global_w = global_ekf_yaw_rate_rps_;
      global_x_var = global_x_var_;
      global_y_var = global_y_var_;
      global_yaw_var = global_yaw_var_;
      have_odom = have_odom_;
      have_raw_fix = have_raw_fix_;
      imu_samples = imu_samples_seen_;
      degraded_gate = degradedQualityPassesUnlocked();
      calibration_samples = static_cast<int>(map_calibration_pairs_.size());
      calibration_unique = uniqueCalibrationTargetsUnlocked();
      calibrationGeometryUnlocked(calibration_baseline_m, calibration_geometry_score);
      global_fresh = have_global_odom_ && last_global_odom_time_.nanoseconds() > 0 &&
        (t - last_global_odom_time_).seconds() >= 0.0 &&
        (t - last_global_odom_time_).seconds() <= std::max(2.0, 2.0 * odom_timeout_sec_);
      const bool vel_fusion_fresh = last_velocity_fusion_pub_time_.nanoseconds() > 0 &&
        (t - last_velocity_fusion_pub_time_).seconds() >= 0.0 &&
        (t - last_velocity_fusion_pub_time_).seconds() <= gnss_fusion_measurement_timeout_sec_;
      const bool cog_fusion_fresh = last_cog_fusion_pub_time_.nanoseconds() > 0 &&
        (t - last_cog_fusion_pub_time_).seconds() >= 0.0 &&
        (t - last_cog_fusion_pub_time_).seconds() <= gnss_fusion_measurement_timeout_sec_;
      gnss_velocity_fusion_active_ = enable_global_gnss_velocity_fusion_ &&
        velocityCertificationAllowsFusionUnlocked() && vel_fusion_fresh;
      gnss_cog_fusion_active_ = enable_global_gnss_cog_fusion_ &&
        cogCertificationAllowsFusionUnlocked() && cog_fusion_fresh;
      velocity_fusion_active_status = gnss_velocity_fusion_active_;
      cog_fusion_active_status = gnss_cog_fusion_active_;
      global_yaw_fusion_fresh_status = globalYawFusionFreshUnlocked();
      map_yaw_from_enu_status = navigation_math::normalizeAngle(
        map_yaw_from_enu_rad_ + map_calibration_.yaw);
      if (anchor_valid_ && have_odom_) map_base = navigation_math::mapBaseFromOdom(anchor_map_odom_, odom_base_);
      detail = last_reject_reason_;
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (!imu_orientation_fresh) {
      imu_roll = nan; imu_pitch = nan; imu_yaw = nan;
    }
    if (!imu_data_fresh) {
      imu_gx = nan; imu_gy = nan; imu_wz = nan;
      imu_ax = nan; imu_ay = nan; imu_az = nan;
    }
    if (!mag_data_fresh) { imu_mx_ut = nan; imu_my_ut = nan; imu_mz_ut = nan; }

    std_msgs::msg::Float64 map_yaw_msg;
    map_yaw_msg.data = map_yaw_from_enu_status;
    map_yaw_offset_pub_->publish(map_yaw_msg);

    std_msgs::msg::Bool pmsg;
    pmsg.data = planning_ready;
    planning_ready_pub_->publish(pmsg);
    std_msgs::msg::Bool mmsg;
    mmsg.data = motion_ready;
    motion_ready_pub_->publish(mmsg);
    std_msgs::msg::Bool gvmsg; gvmsg.data = velocity_fusion_active_status;
    gnss_velocity_fusion_active_pub_->publish(gvmsg);
    std_msgs::msg::Bool gcmsg; gcmsg.data = cog_fusion_active_status;
    gnss_cog_fusion_active_pub_->publish(gcmsg);
    std::ostringstream fusion_ss;
    fusion_ss << std::boolalpha
              << "velocity_enabled=" << enable_global_gnss_velocity_fusion_
              << ";cog_enabled=" << enable_global_gnss_cog_fusion_
              << ";velocity_certified=" << gnss_velocity_calibration_valid_
              << ";cog_certified=" << gnss_cog_calibration_valid_
              << ";velocity_active=" << velocity_fusion_active_status
              << ";cog_active=" << cog_fusion_active_status
              << ";global_yaw_fresh=" << global_yaw_fusion_fresh_status;
    std_msgs::msg::String fusion_state; fusion_state.data = fusion_ss.str();
    gnss_fusion_status_pub_->publish(fusion_state);

    std::ostringstream ss;
    ss << std::boolalpha << std::fixed << std::setprecision(3)
       << "mode=" << mode
       << ";planning_ready=" << planning_ready
       << ";motion_localization_ready=" << motion_ready
       << ";imu_orientation_fresh=" << imu_orientation_fresh
       << ";sat=" << sats
       << ";dop=" << dop
       << ";hacc_m=" << hacc
       << ";calibration_samples=" << calibration_samples
       << ";calibration_unique=" << calibration_unique
       << ";calibration_baseline_m=" << calibration_baseline_m
       << ";calibration_geometry=" << calibration_geometry_score;
    if (!planning_ready) {
      ss << ";bootstrap_odom=" << have_odom
         << ";bootstrap_fix=" << have_raw_fix
         << ";imu_samples=" << imu_samples
         << ";degraded_gate=" << degraded_gate;
    }
    if (planning_ready) {
      ss << ";map_x=" << map_base.x << ";map_y=" << map_base.y << ";yaw=" << map_base.yaw;
    }
    if (!detail.empty()) ss << ";note=" << detail;

    std_msgs::msg::String state;
    state.data = ss.str();
    mode_pub_->publish(state);

    std_msgs::msg::String estimator_status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      estimator_status.data = estimatorSummaryUnlocked();
    }
    pose_estimator_status_pub_->publish(estimator_status);

    // Empat topic diagnostic kecil berikut berisi string depth-1 pada 0.5 Hz.
    // Raw sensor tidak diduplikasi ke NavigationCore, sehingga HUD informatif
    // tanpa menambah bandwidth besar atau callback berfrekuensi tinggi.
    std_msgs::msg::String gnss_msg;
    std::ostringstream gnss_ss;
    gnss_ss << std::fixed << std::setprecision(7)
            << "lat=" << latitude << ";lon=" << longitude
            << std::setprecision(2) << ";alt=" << altitude
            << ";fix=" << fix_metric << ";sat=" << sats
            << ";quality=" << gnss_quality << ";src=" << gnss_source
            << ";dop=" << dop << ";hacc=" << hacc << ";sacc=" << sacc
            << ";ground_v=" << ground_speed
            << ";cog_enu=" << cog_enu
            << ";cog_acc=" << cog_acc
            << ";cog_yaw_gate=" << (cog_yaw_gate ? "true" : "false")
            << ";gnss_fix_ok=" << (quality_.gnss_fix_ok ? "true" : "false")
            << ";gnss_yaw=" << cog_enu
            << ";gnss_vyaw=" << gnss_yaw_rate_rps_
            << ";gnss_vyaw_valid=" << (gnss_yaw_rate_valid_ ? "true" : "false")
            << ";gnss_velocity_qualified=" << (gnss_velocity_qualified_ ? "true" : "false")
            << ";gnss_cog_qualified=" << (cog_motion_qualified_ ? "true" : "false")
            << ";gnss_base_vx=" << gnss_base_vx_mps_
            << ";gnss_base_vy=" << gnss_base_vy_mps_
            << ";wheel_gnss_residual=" << wheel_gnss_residual_mps_
            << ";wheel_slip=" << (wheel_slip || wheel_slip_motion_detected_ ? "true" : "false");
    gnss_msg.data = gnss_ss.str();
    gnss_status_pub_->publish(gnss_msg);

    std_msgs::msg::String imu_msg;
    std::ostringstream imu_ss;
    imu_ss << std::fixed << std::setprecision(3)
           << "roll=" << imu_roll << ";pitch=" << imu_pitch << ";yaw=" << imu_yaw
           << ";gx=" << imu_gx << ";gy=" << imu_gy << ";gz=" << imu_wz
           << ";ax=" << imu_ax << ";ay=" << imu_ay << ";az=" << imu_az
           << ";mx_ut=" << imu_mx_ut << ";my_ut=" << imu_my_ut << ";mz_ut=" << imu_mz_ut
           << ";imu_fresh=" << (imu_data_fresh ? "true" : "false")
           << ";mag_fresh=" << (mag_data_fresh ? "true" : "false")
           << ";ackermann_w=" << esc_kinematic_w
           << ";yaw_residual=" << yaw_model_imu_residual
           << ";ackermann_fresh=" << (esc_kinematic_fresh ? "true" : "false");
    imu_msg.data = imu_ss.str();
    imu_status_pub_->publish(imu_msg);

    std_msgs::msg::String local_msg;
    std::ostringstream local_ss;
    local_ss << std::boolalpha << std::fixed << std::setprecision(3)
             << "fresh=" << local_odom_fresh_status
             << ";x=" << local_pose.x << ";y=" << local_pose.y << ";yaw=" << local_pose.yaw
             << ";v=" << local_v << ";w=" << local_w
             << ";var_x=" << local_x_var << ";var_y=" << local_y_var << ";var_yaw=" << local_yaw_var;
    local_msg.data = local_ss.str();
    ekf_local_status_pub_->publish(local_msg);

    std_msgs::msg::String global_msg;
    std::ostringstream global_ss;
    global_ss << std::boolalpha << std::fixed << std::setprecision(3)
              << "fresh=" << global_fresh
              << ";x=" << global_pose.x << ";y=" << global_pose.y << ";yaw=" << global_pose.yaw
              << ";v=" << global_v << ";w=" << global_w
              << ";var_x=" << global_x_var << ";var_y=" << global_y_var << ";var_yaw=" << global_yaw_var;
    global_msg.data = global_ss.str();
    ekf_global_status_pub_->publish(global_msg);

    // Log terminal hanya saat mode/readiness berubah atau heartbeat 10 detik.
    // Angka GNSS/IMU/EKF tetap diperbarui lewat topic diagnostic 0.5 Hz, sehingga
    // perubahan hAcc kecil tidak menulis log terus-menerus dan menghemat CPU/I/O.
    const auto t = now();
    const std::string log_key = mode + ":" + (planning_ready ? "P1" : "P0") +
      ":" + (motion_ready ? "M1" : "M0");
    const bool state_changed = log_key != last_log_key_;
    const bool heartbeat = last_log_time_.nanoseconds() == 0 ||
      (t - last_log_time_).seconds() >= log_heartbeat_sec_;
    if (state_changed || heartbeat) {
      if (!anchor_valid_) {
        RCLCPP_INFO(get_logger(), "LOCALIZATION STARTUP: %s", state.data.c_str());
      } else if (motion_ready) {
        RCLCPP_INFO(get_logger(), "LOCALIZATION STRICT: %s", state.data.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "LOCALIZATION DEGRADED-HOLD (planning tetap tersedia): %s", state.data.c_str());
      }
      last_log_key_ = log_key;
      last_log_time_ = t;
    }
  }

  std::mutex mutex_;

  double reference_latitude_{0.0};
  double reference_longitude_{0.0};
  double reference_map_x_m_{0.0};
  double reference_map_y_m_{0.0};
  double map_yaw_from_enu_rad_{0.0};
  navigation_math::Pose2D map_calibration_;
  bool persist_map_calibration_{true};
  bool use_imu_initial_heading_{true};
  bool allow_map_calibration_with_degraded_fix_{true};
  bool initial_heading_latched_{false};
  double initial_map_heading_rad_{0.0};
  double initial_odom_yaw_rad_{0.0};
  std::string map_calibration_file_;
  std::string map_calibration_samples_file_;
  bool multi_point_map_calibration_{true};
  int map_calibration_min_unique_points_{3};
  double map_calibration_unique_distance_m_{1.0};
  double map_calibration_min_baseline_m_{2.0};
  double map_calibration_min_geometry_score_{0.03};
  double map_calibration_max_rmse_m_{3.0};
  double calibration_capture_window_sec_{3.0};
  int calibration_capture_min_samples_{3};
  double antenna_x_m_{0.0};
  double antenna_y_m_{0.0};
  std::string gnss_velocity_topic_{"/gnss/vel"};
  std::string gnss_velocity_fit_topic_{"/gnss/velocity_position_fit"};
  double gnss_motion_history_sec_{6.0};
  double gnss_sync_max_gap_sec_{0.30};
  double gnss_velocity_timeout_sec_{0.80};
  double gnss_fit_timeout_sec_{4.0};
  double gnss_velocity_min_validation_speed_mps_{0.15};
  double gnss_speed_consistency_max_mps_{0.20};
  double gnss_fit_speed_residual_max_mps_{0.30};
  double gnss_cog_velocity_course_max_rad_{0.1745329252};
  double gnss_cog_fit_course_max_rad_{0.3490658504};
  double gnss_lateral_velocity_warn_mps_{0.15};
  double wheel_gnss_slip_residual_mps_{0.25};
  double cog_valid_hold_sec_{1.5};
  double cog_invalid_hold_sec_{0.5};
  double gnss_yaw_rate_min_speed_mps_{0.20};
  double gnss_yaw_rate_max_abs_rps_{1.50};
  double gnss_yaw_rate_filter_alpha_{0.35};
  double gnss_yaw_rate_min_variance_{0.0025};
  double gnss_yaw_rate_max_variance_{4.0};
  bool gnss_require_measurement_timestamp_{true};
  double gnss_max_future_stamp_sec_{0.10};
  double gnss_max_measurement_age_sec_{1.0};
  double gnss_max_stamp_regression_sec_{0.02};
  double gnss_quality_timeout_sec_{0.60};
  double gnss_velocity_covariance_min_variance_{1.0e-6};
  double gnss_velocity_covariance_max_variance_{1.0};
  bool require_gnss_velocity_certification_for_fusion_{true};
  bool require_gnss_cog_certification_for_fusion_{true};
  bool gnss_velocity_calibration_valid_{false};
  bool gnss_cog_calibration_valid_{false};
  bool enable_global_gnss_velocity_fusion_{false};
  bool enable_global_gnss_cog_fusion_{false};
  std::string gnss_velocity_fusion_topic_{"/gnss/base_velocity_fusion"};
  std::string gnss_cog_fusion_topic_{"/gnss/cog_heading_fusion"};
  double gnss_velocity_fusion_min_variance_{0.0025};
  double gnss_velocity_fusion_max_variance_{0.25};
  double gnss_cog_fusion_min_variance_rad2_{0.00121846968};
  double gnss_cog_fusion_max_variance_rad2_{0.2741556778};
  double gnss_fusion_measurement_timeout_sec_{1.0};
  bool use_global_ekf_yaw_for_map_correction_{true};
  double global_ekf_yaw_correction_alpha_{0.05};
  double global_ekf_yaw_max_step_rad_{0.00872664626};
  double global_ekf_yaw_max_innovation_rad_{0.7853981634};
  bool enable_gnss_course_yaw_correction_{false};
  double cog_min_forward_speed_mps_{0.25};
  double cog_max_sacc_mps_{0.50};
  double cog_max_heading_accuracy_rad_{0.3490658504};
  double cog_max_local_yaw_rate_rps_{0.12};
  double cog_max_innovation_rad_{0.7853981634};
  double cog_yaw_alpha_{0.02};
  double cog_max_yaw_step_rad_{0.00872664626};
  double map_min_x_m_{0.0};
  double map_max_x_m_{0.0};
  double map_min_y_m_{0.0};
  double map_max_y_m_{0.0};
  double map_bounds_margin_m_{0.0};

  bool allow_degraded_planning_{true};
  bool allow_provisional_map_display_{true};
  int provisional_min_satellites_{3};
  double provisional_max_dop_{25.0};
  double provisional_max_hacc_m_{150.0};
  int degraded_min_satellites_{8};
  double degraded_max_dop_{6.0};
  double degraded_max_hacc_m_{20.0};
  bool anchor_init_requires_strict_{false};
  int startup_gnss_samples_{5};
  int startup_imu_samples_{3};
  double startup_max_spread_m_{5.0};
  double global_odom_map_reference_max_error_m_{15.0};
  int strict_min_satellites_{8};
  double strict_max_dop_{2.0};
  double strict_max_hacc_m_{2.5};
  double strict_quality_hold_sec_{1.5};
  double motion_hold_max_hacc_m_{4.5};
  double motion_hold_max_dop_{2.5};
  int motion_hold_min_satellites_{8};
  double motion_degrade_grace_sec_{3.0};
  double motion_critical_max_hacc_m_{6.0};
  double motion_critical_max_dop_{4.0};
  int motion_critical_min_satellites_{6};
  double strict_correction_alpha_{0.20};
  bool freeze_stationary_map_translation_{true};
  double strict_moving_correction_alpha_{0.03};
  bool enable_wheel_slip_pose_correction_{false};
  double strict_slip_correction_alpha_{0.60};
  double strict_max_correction_m_{0.25};
  double strict_max_yaw_correction_rad_{0.087266463};
  double strict_stationary_speed_mps_{0.15};
  double strict_stationary_yaw_rate_rps_{0.10};
  double slip_min_wheel_speed_mps_{0.25};
  double slip_speed_difference_mps_{0.20};
  double slip_sacc_multiplier_{0.50};
  double slip_yaw_rate_difference_rps_{0.20};
  std::string esc_kinematic_yaw_rate_topic_{"/esc/kinematic_yaw_rate_rps"};
  double esc_kinematic_yaw_timeout_sec_{0.50};
  double gnss_timeout_sec_{2.5};
  double imu_timeout_sec_{0.75};
  double odom_timeout_sec_{0.75};
  double tf_publish_rate_hz_{10.0};
  double tf_future_offset_sec_{0.05};
  double status_publish_rate_hz_{2.0};
  double log_heartbeat_sec_{10.0};
  bool allow_manual_pose_for_motion_{false};

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string raw_gnss_topic_;
  std::string quality_topic_;
  std::string imu_topic_;
  std::string local_odom_topic_;
  std::string global_odom_topic_;

  Quality quality_;
  navigation_math::Pose2D odom_base_;
  navigation_math::Pose2D anchor_map_odom_;
  std::deque<LocalMotionSample> local_motion_history_;
  bool have_odom_{false};
  bool have_raw_fix_{false};
  bool anchor_valid_{false};
  int imu_samples_seen_{0};
  int imu_orientation_samples_seen_{0};
  bool imu_orientation_valid_{false};
  double gnss_latitude_{0.0};
  double gnss_longitude_{0.0};
  double gnss_altitude_m_{0.0};
  double imu_roll_rad_{0.0};
  double imu_pitch_rad_{0.0};
  double imu_yaw_enu_rad_{0.0};
  double imu_gyro_x_rps_{0.0};
  double imu_gyro_y_rps_{0.0};
  double imu_yaw_rate_rps_{0.0};
  double esc_kinematic_yaw_rate_rps_{0.0};
  double imu_accel_x_mps2_{0.0};
  double imu_accel_y_mps2_{0.0};
  double imu_accel_z_mps2_{0.0};
  double imu_mag_x_ut_{0.0};
  double imu_mag_y_ut_{0.0};
  double imu_mag_z_ut_{0.0};
  double local_forward_speed_mps_{0.0};
  double local_speed_mps_{0.0};
  double local_yaw_rate_rps_{0.0};
  double local_x_var_{0.0};
  double local_y_var_{0.0};
  double local_yaw_var_{0.0};
  double gnss_enu_ve_mps_{0.0};
  double gnss_enu_vn_mps_{0.0};
  double gnss_map_vx_mps_{0.0};
  double gnss_map_vy_mps_{0.0};
  double gnss_base_vx_mps_{0.0};
  double gnss_base_vy_mps_{0.0};
  double gnss_fit_speed_mps_{0.0};
  double gnss_fit_course_enu_rad_{0.0};
  double local_forward_at_gnss_mps_{0.0};
  double local_yaw_rate_at_gnss_rps_{0.0};
  double gnss_map_yaw_at_measurement_rad_{0.0};
  double gnss_yaw_rate_rps_{0.0};
  double gnss_yaw_rate_variance_{1.0e6};
  bool gnss_yaw_rate_valid_{false};
  bool have_gnss_yaw_rate_filter_{false};
  bool have_local_motion_at_gnss_{false};
  std::optional<double> last_gnss_velocity_course_enu_rad_;
  rclcpp::Time last_gnss_velocity_course_stamp_{0, 0, RCL_ROS_TIME};
  double gnss_last_sync_gap_sec_{999.0};
  double wheel_gnss_residual_mps_{0.0};
  bool gnss_velocity_qualified_{false};
  bool cog_motion_qualified_{false};
  bool gnss_velocity_fusion_active_{false};
  bool gnss_cog_fusion_active_{false};
  bool wheel_slip_motion_detected_{false};
  bool have_last_base_velocity_msg_{false};
  geometry_msgs::msg::TwistWithCovarianceStamped last_base_velocity_msg_;
  int64_t last_velocity_fusion_stamp_ns_{-1};
  int64_t last_cog_fusion_stamp_ns_{-1};
  size_t gnss_sync_reject_count_{0};
  size_t gnss_timestamp_reject_count_{0};
  size_t gnss_covariance_reject_count_{0};
  bool gnss_last_velocity_covariance_valid_{false};
  std::string gnss_last_motion_reject_reason_;
  rclcpp::Time last_accepted_gnss_velocity_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> cog_candidate_since_;
  std::optional<rclcpp::Time> cog_bad_since_;
  rclcpp::Time last_gnss_velocity_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gnss_velocity_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gnss_fit_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gnss_fit_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_velocity_fusion_pub_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cog_fusion_pub_time_{0, 0, RCL_ROS_TIME};
  navigation_math::Pose2D last_raw_map_base_;
  bool have_last_raw_map_base_{false};
  navigation_math::Pose2D global_ekf_pose_;
  double global_ekf_speed_mps_{0.0};
  double global_ekf_yaw_rate_rps_{0.0};
  double global_x_var_{0.0};
  double global_y_var_{0.0};
  double global_yaw_var_{0.0};
  bool have_global_odom_{false};
  std::string anchor_mode_{"STARTUP"};
  std::string startup_anchor_quality_mode_{"DEGRADED_BOOTSTRAP"};
  std::string last_reject_reason_;
  std::deque<GnssSample> startup_samples_;
  std::deque<RawMapSample> raw_map_capture_window_;
  std::vector<MapCalibrationPair> map_calibration_pairs_;
  std::optional<rclcpp::Time> strict_since_;
  bool motion_strict_armed_{false};
  rclcpp::Time last_motion_quality_acceptable_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gnss_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_quality_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_imu_orientation_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_esc_kinematic_yaw_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_mag_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_global_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time global_odom_ignore_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_log_time_{0, 0, RCL_ROS_TIME};
  std::string last_log_key_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr quality_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_velocity_fit_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_kinematic_yaw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planning_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr map_yaw_offset_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gnss_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr imu_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ekf_local_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ekf_global_status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gnss_map_odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_vel_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_base_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gnss_velocity_fusion_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_cog_fusion_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gnss_velocity_fusion_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gnss_cog_fusion_active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gnss_fusion_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gnss_motion_validation_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gnss_velocity_qualified_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gnss_cog_qualified_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gnss_speed_residual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gnss_course_residual_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_estimator_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_estimator_status_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
    reset_calibration_samples_service_;
  rclcpp::TimerBase::SharedPtr tf_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

// Fungsi: Entry point single-threaded. Callback tidak blocking sehingga executor
// satu thread lebih hemat scheduling/RAM dibanding MultiThreadedExecutor.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationCore>());
  rclcpp::shutdown();
  return 0;
}
