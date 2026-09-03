#ifndef GNSS_NODE_HPP
#define GNSS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <serial/serial.h>

#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class GnssNode : public rclcpp::Node
{
public:
  explicit GnssNode(const rclcpp::NodeOptions & options);
  ~GnssNode() override;

private:
  struct NavCovEpoch
  {
    bool valid = false;
    uint32_t itow_ms = 0;
    bool pos_valid = false;
    bool vel_valid = false;
    double pos_nn = 0.0, pos_ne = 0.0, pos_nd = 0.0;
    double pos_ee = 0.0, pos_ed = 0.0, pos_dd = 0.0;
    double vel_nn = 0.0, vel_ne = 0.0, vel_nd = 0.0;
    double vel_ee = 0.0, vel_ed = 0.0, vel_dd = 0.0;
  };

  struct NavDopEpoch
  {
    bool valid = false;
    uint32_t itow_ms = 0;
    double gdop = std::numeric_limits<double>::quiet_NaN();
    double pdop = std::numeric_limits<double>::quiet_NaN();
    double tdop = std::numeric_limits<double>::quiet_NaN();
    double vdop = std::numeric_limits<double>::quiet_NaN();
    double hdop = std::numeric_limits<double>::quiet_NaN();
    double ndop = std::numeric_limits<double>::quiet_NaN();
    double edop = std::numeric_limits<double>::quiet_NaN();
  };

  struct FitSample
  {
    double t_sec = 0.0;
    double lat_rad = 0.0;
    double lon_rad = 0.0;
    double hacc_m = 1.0;
  };

  void openSerial(bool initial = false);
  void closeSerial();
  void pollSerial();
  void processBuffer();
  void publish();
  bool handleNmea(const std::string & sentence);
  bool handleUbx(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t> & payload);
  void emitRaw(const std::string & text);
  bool looksLikeGnssStream(double probe_sec = 0.7);
  std::vector<std::string> candidatePorts();
  bool isAutoPort() const;
  bool hasValidPosition() const;
  bool qualityGatePasses() const;
  std::string qualityGateReason() const;
  bool recentUbxPvt(double now) const;
  bool nmeaFallbackEligible(double now) const;
  bool configureUbxNavPvtOutput();
  void maybeRetryUbxConfig();
  void maybePollAuxNav();
  void publishState();
  bool sendUbxMessage(uint8_t msg_class, uint8_t msg_id, const std::vector<uint8_t> & payload);
  void markFix();
  void markNoFix();
  rclcpp::Time makeMeasurementStamp();
  void updatePvtRate(uint32_t itow_ms);
  void addPositionFitSample(const rclcpp::Time & stamp);
  void publishPositionFit(const rclcpp::Time & stamp);
  bool sameEpoch(uint32_t a, uint32_t b) const;

  // Parameters
  std::string port_;
  std::string auto_port_id_contains_;
  std::string auto_port_path_contains_;
  int baudrate_ = 115200;
  std::string frame_id_ = "gnss_link";
  std::string velocity_frame_id_ = "enu";
  bool publish_raw_ = false;
  bool auto_baud_enabled_ = false;

  bool configure_ubx_nav_pvt_on_connect_ = true;
  bool configure_navigation_rate_ = true;
  double navigation_rate_hz_ = 5.0;
  bool configure_dynamic_model_ = true;
  uint8_t dynamic_model_ = 4;  // u-blox AUTOMOT
  bool poll_nav_cov_ = true;
  double nav_cov_poll_rate_hz_ = 5.0;
  bool poll_nav_dop_ = true;
  double nav_dop_poll_rate_hz_ = 1.0;
  double cov_wait_timeout_sec_ = 0.08;
  std::string timestamp_mode_ = "auto";
  double utc_stamp_max_offset_sec_ = 10.0;
  double position_fit_window_sec_ = 3.0;
  int position_fit_min_samples_ = 5;
  double position_fit_min_baseline_m_ = 1.0;
  double position_fit_hacc_multiplier_ = 2.5;

  bool prefer_ubx_nav_pvt_ = true;
  bool require_ubx_nav_pvt_for_fix_ = true;
  double ubx_preference_hold_sec_ = 2.0;
  bool accept_rmc_position_fallback_ = false;
  bool allow_validated_nmea_fallback_ = true;
  double nmea_fallback_after_sec_ = 8.0;
  int nmea_fallback_min_satellites_ = 8;
  double nmea_fallback_max_hdop_ = 1.4;
  double ubx_config_retry_sec_ = 2.0;
  int ubx_config_max_attempts_ = 3;
  double state_publish_period_sec_ = 1.0;
  int min_satellites_ = 8;
  double max_dop_ = 2.0;
  bool require_dop_for_ubx_quality_ = true;
  double max_hacc_m_ = 2.5;
  double max_sacc_mps_ = 0.8;
  bool require_speed_accuracy_for_quality_ = false;
  bool accept_ubx_gnss_dr_fix_ = false;
  bool ubx_nav_pvt_seen_ = false;
  bool nmea_fallback_active_ = false;
  bool quality_state_initialized_ = false;
  bool last_quality_good_ = false;
  int last_nmea_fix_quality_ = 0;
  bool last_nmea_gga_checksum_present_ = false;

  // Serial
  std::unique_ptr<serial::Serial> ser_;
  std::string active_port_;
  serial::Timeout ser_timeout_;
  std::vector<uint8_t> buf_;

  // GNSS state
  std::optional<double> lat_, lon_, alt_;
  std::optional<int> sats_;
  std::optional<double> quality_dop_;
  int fix_status_ = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  std::optional<double> speed_mps_;
  std::optional<double> heading_rad_;
  std::optional<double> heading_accuracy_rad_;
  std::optional<double> vel_n_mps_, vel_e_mps_, vel_d_mps_;
  double last_velocity_time_ = 0.0;
  double velocity_stale_timeout_ = 1.0;
  double data_timeout_sec_ = 3.0;
  double last_data_time_ = 0.0;
  double last_ubx_pvt_time_ = -1.0;
  double last_nmea_gga_time_ = -1.0;
  double serial_connected_since_ = -1.0;
  double last_ubx_config_send_time_ = -1.0;
  double last_cov_poll_time_ = -1.0;
  double last_dop_poll_time_ = -1.0;
  bool last_position_from_ubx_ = false;
  uint8_t last_ubx_fix_type_ = 0;
  bool last_ubx_gnss_fix_ok_ = false;
  uint8_t last_ubx_flags_ = 0;
  uint8_t last_ubx_flags2_ = 0;
  uint16_t last_ubx_flags3_ = 0;
  bool last_ubx_invalid_llh_ = false;
  int32_t ubx_height_mm_ = 0;
  std::optional<double> head_vehicle_rad_;
  std::optional<double> mag_declination_rad_;
  std::optional<double> mag_declination_accuracy_rad_;
  uint32_t ubx_hacc_mm_ = 0;
  uint32_t ubx_vacc_mm_ = 0;
  uint32_t ubx_sacc_mmps_ = 0;
  uint32_t last_itow_ms_ = 0;
  uint16_t utc_year_ = 0;
  uint8_t utc_month_ = 0, utc_day_ = 0, utc_hour_ = 0, utc_minute_ = 0, utc_second_ = 0;
  uint8_t utc_valid_flags_ = 0;
  uint32_t utc_tacc_ns_ = 0;
  int32_t utc_nano_ = 0;
  bool pending_ubx_publish_ = false;
  double pending_ubx_since_ = 0.0;
  NavCovEpoch nav_cov_;
  NavDopEpoch nav_dop_;
  std::deque<FitSample> fit_samples_;
  double pvt_rate_hz_ema_ = 0.0;
  bool have_rate_itow_ = false;
  uint32_t rate_last_itow_ms_ = 0;
  std::string last_timestamp_source_ = "arrival";
  double last_measurement_age_sec_ = 0.0;
  bool itow_anchor_valid_ = false;
  uint32_t itow_anchor_ms_ = 0;
  rclcpp::Time itow_anchor_stamp_{0, 0, RCL_SYSTEM_TIME};

  // Stats
  size_t bytes_received_ = 0;
  size_t packets_parsed_ = 0;
  size_t nmea_gga_count_ = 0;
  size_t ubx_nav_pvt_count_ = 0;
  size_t ubx_nav_cov_count_ = 0;
  size_t ubx_nav_dop_count_ = 0;
  size_t ubx_duplicate_itow_count_ = 0;
  size_t ubx_out_of_order_itow_count_ = 0;
  size_t ubx_checksum_errors_ = 0;
  size_t nmea_checksum_errors_ = 0;
  size_t ubx_cfg_ack_count_ = 0;
  size_t ubx_cfg_nak_count_ = 0;
  int ubx_config_attempts_ = 0;
  double last_status_time_ = 0.0;
  double last_reconnect_try_ = 0.0;
  const double reconnect_interval_sec_ = 2.0;
  int consecutive_read_errors_ = 0;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_fix_raw_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_vel_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_vel_fit_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_quality_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_motion_diag_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_raw_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_connected_;

  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;
};

static inline void ubxChecksum(const uint8_t * data, size_t len, uint8_t & a, uint8_t & b)
{
  a = b = 0;
  for (size_t i = 0; i < len; ++i) {
    a += data[i];
    b += a;
  }
}

static inline std::vector<std::string> splitNmea(const std::string & sentence)
{
  std::vector<std::string> fields;
  size_t start = 0;
  for (size_t i = 0; i <= sentence.size(); ++i) {
    if (i == sentence.size() || sentence[i] == ',' || sentence[i] == '*') {
      fields.push_back(sentence.substr(start, i - start));
      start = i + 1;
      if (i < sentence.size() && sentence[i] == '*') break;
    }
  }
  return fields;
}

#endif // GNSS_NODE_HPP
