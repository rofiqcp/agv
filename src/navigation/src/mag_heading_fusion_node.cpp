#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace {
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

geometry_msgs::msg::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

void rollPitchFromQuat(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch) {
  const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr, cosr);
  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  pitch = std::abs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);
}

rclcpp::Time stampOrNow(const std_msgs::msg::Header &header, const rclcpp::Time &now) {
  if (header.stamp.sec == 0 && header.stamp.nanosec == 0) return now;
  return rclcpp::Time(header.stamp);
}
}  // namespace

class Rm3100HeadingNode final : public rclcpp::Node {
 public:
  Rm3100HeadingNode() : Node("mag_heading_fusion") {
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("rm3100_mag_topic", "/neo3pro/mag");
    declare_parameter<std::string>("map_yaw_topic", "/localization/map_yaw_from_enu");
    declare_parameter<std::string>("heading_topic", "/neo3pro/mag_heading_fusion");
    declare_parameter<std::string>("calibrated_mag_topic", "/neo3pro/mag_calibrated");
    declare_parameter<std::string>("inertial_heading_topic", "/imu/inertial_heading");
    declare_parameter<std::string>("validated_heading_topic", "/heading/validated_fusion");
    declare_parameter<std::string>("local_heading_topic", "/heading/validated_local");
    declare_parameter<std::string>("local_heading_frame", "odom");
    declare_parameter<std::string>("output_frame", "map");
    declare_parameter<double>("magnetic_declination_rad", 0.0);
    declare_parameter<double>("rm3100_yaw_sign", 1.0);
    declare_parameter<double>("rm3100_yaw_offset_rad", 0.0);
    declare_parameter<double>("rm3100_heading_alignment_rad", 0.0);
    declare_parameter<double>("rm3100_heading_variance_rad2", 0.01);
    declare_parameter<double>("validated_heading_variance_rad2", 0.01);
    declare_parameter<std::string>("rm3100_calibration_owner", "ros_host");
    declare_parameter<bool>("rm3100_full_calibration_enabled", false);
    declare_parameter<std::vector<double>>("rm3100_mag_bias_xyz_ut", {0.0, 0.0, 0.0});
    declare_parameter<std::vector<double>>("rm3100_mag_matrix_3x3", {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0});
    declare_parameter<bool>("rm3100_planar_calibration_enabled", false);
    declare_parameter<std::vector<double>>("rm3100_mag_bias_xy_ut", {0.0, 0.0});
    declare_parameter<std::vector<double>>("rm3100_mag_matrix_xy", {1.0,0.0,0.0,1.0});
    declare_parameter<bool>("rm3100_heading_lut_enabled", false);
    declare_parameter<std::vector<double>>("rm3100_heading_lut_input_rad", std::vector<double>{});
    declare_parameter<std::vector<double>>("rm3100_heading_lut_correction_rad", std::vector<double>{});
    declare_parameter<bool>("rm3100_inertial_stationary_correction", true);
    declare_parameter<double>("rm3100_inertial_correction_max_rate_dps", 0.1);
    declare_parameter<double>("rm3100_inertial_correction_max_error_rad", 0.2617993878);

    imu_topic_ = get_parameter("imu_topic").as_string();
    mag_topic_ = get_parameter("rm3100_mag_topic").as_string();
    map_yaw_topic_ = get_parameter("map_yaw_topic").as_string();
    output_frame_ = get_parameter("output_frame").as_string();
    local_heading_frame_ = get_parameter("local_heading_frame").as_string();
    declination_rad_ = get_parameter("magnetic_declination_rad").as_double();
    yaw_sign_ = get_parameter("rm3100_yaw_sign").as_double() < 0.0 ? -1.0 : 1.0;
    yaw_offset_rad_ = get_parameter("rm3100_yaw_offset_rad").as_double();
    alignment_rad_ = get_parameter("rm3100_heading_alignment_rad").as_double();
    heading_variance_ = std::max(1.0e-6, get_parameter("rm3100_heading_variance_rad2").as_double());
    validated_variance_ = std::max(1.0e-6, get_parameter("validated_heading_variance_rad2").as_double());
    calibration_owner_ = get_parameter("rm3100_calibration_owner").as_string();
    if (calibration_owner_ != "ros_host" && calibration_owner_ != "ap_periph") {
      throw std::invalid_argument("rm3100_calibration_owner must be ros_host or ap_periph");
    }
    full_enabled_ = get_parameter("rm3100_full_calibration_enabled").as_bool();
    planar_enabled_ = get_parameter("rm3100_planar_calibration_enabled").as_bool();
    calibration_applied_ = (calibration_owner_ == "ap_periph") ||
      (calibration_owner_ == "ros_host" && full_enabled_);
    loadVector("rm3100_mag_bias_xyz_ut", bias_xyz_, 3);
    loadVector("rm3100_mag_matrix_3x3", matrix_3x3_, 9);
    loadVector("rm3100_mag_bias_xy_ut", bias_xy_, 2);
    loadVector("rm3100_mag_matrix_xy", matrix_xy_, 4);
    loadLut();
    inertial_stationary_correction_ = get_parameter("rm3100_inertial_stationary_correction").as_bool();
    inertial_correction_rate_rps_ = std::max(0.0, get_parameter("rm3100_inertial_correction_max_rate_dps").as_double()) * kPi / 180.0;
    inertial_correction_max_error_ = std::max(0.0, get_parameter("rm3100_inertial_correction_max_error_rad").as_double());

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(20);
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(imu_topic_, sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { onImu(*msg); });
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(mag_topic_, sensor_qos,
      [this](sensor_msgs::msg::MagneticField::ConstSharedPtr msg) { onRm3100(*msg); });
    map_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(map_yaw_topic_, state_qos,
      [this](std_msgs::msg::Float64::ConstSharedPtr msg) {
        if (std::isfinite(msg->data)) { map_yaw_rad_ = normalizeAngle(msg->data); have_map_yaw_ = true; }
      });

    heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(get_parameter("heading_topic").as_string(), sensor_qos);
    calibrated_mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(get_parameter("calibrated_mag_topic").as_string(), sensor_qos);
    inertial_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(get_parameter("inertial_heading_topic").as_string(), sensor_qos);
    validated_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(get_parameter("validated_heading_topic").as_string(), sensor_qos);
    local_heading_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(get_parameter("local_heading_topic").as_string(), sensor_qos);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>("/heading/validated", state_qos);
    rm_valid_pub_ = create_publisher<std_msgs::msg::Bool>("/neo3pro/mag_heading_valid", state_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("/system/magnetic_heading_status", state_qos);
    status_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publishStatus(); });

    RCLCPP_INFO(get_logger(), "RM3100-only heading: input=%s owner=%s full3d=%s planar=%s applied=%s",
      mag_topic_.c_str(), calibration_owner_.c_str(), full_enabled_ ? "on" : "off",
      planar_enabled_ ? "on" : "off", calibration_applied_ ? "yes" : "no");
  }

 private:
  void loadVector(const std::string &name, std::vector<double> &dst, std::size_t expected) {
    dst = get_parameter(name).as_double_array();
    if (dst.size() != expected || !std::all_of(dst.begin(), dst.end(), [](double v){ return std::isfinite(v); })) {
      throw std::invalid_argument(name + " has invalid size/value");
    }
  }

  void loadLut() {
    lut_enabled_ = get_parameter("rm3100_heading_lut_enabled").as_bool();
    const auto x = get_parameter("rm3100_heading_lut_input_rad").as_double_array();
    const auto y = get_parameter("rm3100_heading_lut_correction_rad").as_double_array();
    if (!lut_enabled_) return;
    if (x.size() != y.size() || x.size() < 4) { lut_enabled_ = false; return; }
    for (std::size_t i=0; i<x.size(); ++i) if (std::isfinite(x[i]) && std::isfinite(y[i])) lut_.emplace_back(normalizeAngle(x[i]), y[i]);
    std::sort(lut_.begin(), lut_.end(), [](const auto &a, const auto &b){ return a.first < b.first; });
    if (lut_.size() < 4) lut_enabled_ = false;
  }

  double applyLut(double yaw) const {
    if (!lut_enabled_ || lut_.size() < 2) return normalizeAngle(yaw);
    const double y = normalizeAngle(yaw);
    std::size_t hi = 0; while (hi < lut_.size() && lut_[hi].first < y) ++hi;
    double x0,c0,x1,c1,yy=y;
    if (hi == 0) { x0=lut_.back().first; c0=lut_.back().second; x1=lut_.front().first+2*kPi; c1=lut_.front().second; yy+=2*kPi; }
    else if (hi == lut_.size()) { x0=lut_.back().first; c0=lut_.back().second; x1=lut_.front().first+2*kPi; c1=lut_.front().second; }
    else { x0=lut_[hi-1].first; c0=lut_[hi-1].second; x1=lut_[hi].first; c1=lut_[hi].second; }
    const double t = std::abs(x1-x0) < 1e-12 ? 0.0 : (yy-x0)/(x1-x0);
    return normalizeAngle(y + c0 + t*(c1-c0));
  }

  geometry_msgs::msg::PoseWithCovarianceStamped poseMsg(double yaw, double variance, const rclcpp::Time &stamp,
      const std::string &frame = "") const {
    geometry_msgs::msg::PoseWithCovarianceStamped p;
    p.header.stamp = stamp; p.header.frame_id = frame.empty() ? output_frame_ : frame;
    p.pose.pose.orientation = yawQuaternion(yaw); p.pose.covariance.fill(0.0);
    p.pose.covariance[0]=p.pose.covariance[7]=p.pose.covariance[14]=p.pose.covariance[21]=p.pose.covariance[28]=1.0e6;
    p.pose.covariance[35]=variance; return p;
  }

  void onImu(const sensor_msgs::msg::Imu &msg) {
    const auto now_t = now();
    const auto stamp = stampOrNow(msg.header, now_t);
    if (msg.orientation_covariance[0] >= 0.0) {
      rollPitchFromQuat(msg.orientation, roll_rad_, pitch_rad_); have_tilt_ = true;
    }
    const double wz = msg.angular_velocity.z;
    const double an = std::sqrt(msg.linear_acceleration.x*msg.linear_acceleration.x + msg.linear_acceleration.y*msg.linear_acceleration.y + msg.linear_acceleration.z*msg.linear_acceleration.z);
    stationary_ = std::isfinite(wz) && std::abs(wz) < 0.03 && std::isfinite(an) && std::abs(an-9.80665) < 1.2;
    if (have_inertial_ && std::isfinite(wz) && last_imu_stamp_.nanoseconds()!=0) {
      const double dt=(stamp-last_imu_stamp_).seconds();
      if (dt>0.0 && dt<0.5) inertial_yaw_=normalizeAngle(inertial_yaw_ + wz*dt);
    }
    last_imu_stamp_=stamp;
    if (have_inertial_) {
      inertial_pub_->publish(poseMsg(inertial_yaw_, validated_variance_, stamp));
      if (have_local_heading_reference_) {
        local_heading_pub_->publish(poseMsg(
          normalizeAngle(inertial_yaw_ - local_heading_reference_yaw_),
          validated_variance_, stamp, local_heading_frame_));
      }
    }
  }

  void onRm3100(const sensor_msgs::msg::MagneticField &msg) {
    const auto stamp = stampOrNow(msg.header, now());
    const std::array<double,3> raw{{msg.magnetic_field.x*1e6,msg.magnetic_field.y*1e6,msg.magnetic_field.z*1e6}};
    if (!std::all_of(raw.begin(), raw.end(), [](double v){return std::isfinite(v);})) return;
    wire_xyz_ = raw; wire_norm_ = std::hypot(std::hypot(raw[0],raw[1]),raw[2]);

    std::array<double,3> corrected=raw;
    double xh=raw[0], yh=raw[1], zh=raw[2];
    bool calibrated_vector=false;
    if (calibration_owner_ == "ros_host" && full_enabled_) {
      const std::array<double,3> b{{raw[0]-bias_xyz_[0],raw[1]-bias_xyz_[1],raw[2]-bias_xyz_[2]}};
      for (int r=0;r<3;++r) corrected[r]=matrix_3x3_[r*3]*b[0]+matrix_3x3_[r*3+1]*b[1]+matrix_3x3_[r*3+2]*b[2];
      xh=corrected[0]; yh=corrected[1]; zh=corrected[2]; calibrated_vector=true;
    } else if (calibration_owner_ == "ros_host" && planar_enabled_) {
      const double bx=raw[0]-bias_xy_[0], by=raw[1]-bias_xy_[1];
      xh=matrix_xy_[0]*bx+matrix_xy_[1]*by; yh=matrix_xy_[2]*bx+matrix_xy_[3]*by;
    } else if (calibration_owner_ == "ap_periph") {
      calibrated_vector=true;
    }

    if (calibrated_vector) {
      sensor_msgs::msg::MagneticField out=msg;
      out.magnetic_field.x=corrected[0]*1e-6; out.magnetic_field.y=corrected[1]*1e-6; out.magnetic_field.z=corrected[2]*1e-6;
      calibrated_mag_pub_->publish(out);
      calibrated_xyz_=corrected; calibrated_norm_=std::hypot(std::hypot(corrected[0],corrected[1]),corrected[2]);
    }

    if (!(calibration_owner_ == "ros_host" && planar_enabled_) && have_tilt_) {
      const double cr=std::cos(roll_rad_), sr=std::sin(roll_rad_), cp=std::cos(pitch_rad_), sp=std::sin(pitch_rad_);
      const double tx=xh*cp+zh*sp;
      const double ty=xh*sr*sp+yh*cr-zh*sr*cp;
      xh=tx; yh=ty;
    }
    if (!std::isfinite(xh) || !std::isfinite(yh) || std::hypot(xh,yh)<1e-12) return;
    double yaw_enu=normalizeAngle(yaw_sign_*std::atan2(yh,xh)+yaw_offset_rad_-declination_rad_);
    yaw_enu=normalizeAngle(applyLut(yaw_enu)+alignment_rad_);
    const double yaw_map=normalizeAngle(yaw_enu+(have_map_yaw_?map_yaw_rad_:0.0));
    last_heading_=yaw_map; last_mag_stamp_=stamp; have_heading_=true;

    // Diagnostic heading may always be observed, but the EKF-facing validated
    // stream is fail-closed: it exists only after the selected calibration owner
    // has an applied production correction.  This prevents the global EKF from
    // silently consuming legacy planar/LUT or identity/raw magnetic yaw.
    heading_pub_->publish(poseMsg(yaw_map, heading_variance_, stamp));
    std_msgs::msg::Bool ok; ok.data=calibration_applied_;
    valid_pub_->publish(ok); rm_valid_pub_->publish(ok);
    if (!calibration_applied_) return;

    validated_pub_->publish(poseMsg(yaw_map, validated_variance_, stamp));
    if (!have_inertial_) {
      inertial_yaw_=yaw_map; have_inertial_=true;
      local_heading_reference_yaw_=yaw_map; have_local_heading_reference_=true;
    }
    else if (inertial_stationary_correction_ && stationary_) {
      const double e=normalizeAngle(yaw_map-inertial_yaw_);
      if (std::abs(e)<=inertial_correction_max_error_) {
        const double dt=last_correction_stamp_.nanoseconds()==0?0.0:std::clamp((stamp-last_correction_stamp_).seconds(),0.0,1.0);
        const double step=inertial_correction_rate_rps_*dt;
        inertial_yaw_=normalizeAngle(inertial_yaw_+std::clamp(e,-step,step));
      }
    }
    last_correction_stamp_=stamp;
    inertial_pub_->publish(poseMsg(inertial_yaw_, validated_variance_, stamp));
    if (have_local_heading_reference_) {
      local_heading_pub_->publish(poseMsg(
        normalizeAngle(inertial_yaw_ - local_heading_reference_yaw_),
        validated_variance_, stamp, local_heading_frame_));
    }
  }

  void publishStatus() {
    const auto t=now();
    const double age=have_heading_?(t-last_mag_stamp_).seconds():999.0;
    const bool live=have_heading_ && age>=0.0 && age<1.0;
    const bool validated_live=live && calibration_applied_;
    std_msgs::msg::Bool b; b.data=validated_live; valid_pub_->publish(b); rm_valid_pub_->publish(b);
    std_msgs::msg::String s;
    s.data="mag_source=RM3100;input_topic="+mag_topic_+
      ";rm3100_live="+(live?std::string("true"):std::string("false"))+
      ";rm3100_age_sec="+std::to_string(age)+
      ";rm3100_norm_ut="+std::to_string(wire_norm_)+
      ";rm3100_heading_deg="+std::to_string(last_heading_*180.0/kPi)+
      ";validated_heading_deg="+std::to_string(last_heading_*180.0/kPi)+
      ";inertial_heading_deg="+std::to_string(inertial_yaw_*180.0/kPi)+
      ";map_yaw_known="+(have_map_yaw_?std::string("true"):std::string("false"))+
      ";cal_owner="+calibration_owner_+
      ";rm3100_calibration_applied="+(calibration_applied_?std::string("true"):std::string("false"))+
      ";validated_live="+(validated_live?std::string("true"):std::string("false"))+
      ";rm3100_full3d="+(full_enabled_?std::string("true"):std::string("false"))+
      ";rm3100_planar="+(planar_enabled_?std::string("true"):std::string("false"))+
      ";rm3100_lut="+(lut_enabled_?std::string("true"):std::string("false"));
    status_pub_->publish(s);
  }

  std::string imu_topic_, mag_topic_, map_yaw_topic_, output_frame_, local_heading_frame_, calibration_owner_;
  double declination_rad_{0.0}, yaw_sign_{1.0}, yaw_offset_rad_{0.0}, alignment_rad_{0.0};
  double heading_variance_{0.01}, validated_variance_{0.01};
  bool full_enabled_{false}, planar_enabled_{false}, lut_enabled_{false}, calibration_applied_{false};
  std::vector<double> bias_xyz_, matrix_3x3_, bias_xy_, matrix_xy_;
  std::vector<std::pair<double,double>> lut_;
  bool inertial_stationary_correction_{true};
  double inertial_correction_rate_rps_{0.00174532925}, inertial_correction_max_error_{0.2617993878};
  bool have_tilt_{false}, have_map_yaw_{false}, have_inertial_{false}, have_heading_{false}, stationary_{false};
  bool have_local_heading_reference_{false};
  double roll_rad_{0.0}, pitch_rad_{0.0}, map_yaw_rad_{0.0}, inertial_yaw_{0.0}, last_heading_{0.0};
  double local_heading_reference_yaw_{0.0};
  std::array<double,3> wire_xyz_{{0,0,0}}, calibrated_xyz_{{0,0,0}};
  double wire_norm_{0.0}, calibrated_norm_{0.0};
  rclcpp::Time last_imu_stamp_{0,0,RCL_ROS_TIME}, last_mag_stamp_{0,0,RCL_ROS_TIME}, last_correction_stamp_{0,0,RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr map_yaw_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr heading_pub_, inertial_pub_, validated_pub_, local_heading_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr calibrated_mag_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_, rm_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rm3100HeadingNode>());
  rclcpp::shutdown();
  return 0;
}
