/*
 * Portable YOLOPv2 CPU backend for ROS 2 Humble.
 *
 * Direct TorchScript YOLOPv2 CPU backend for the Mini-PC runtime.
 * The node reads the official yolopv2.pt directly through LibTorch; there is
 * no PT->TorchScript conversion, TorchScript cache, or onnx Python dependency. Post-processing,
 * metric projection, and ROS safety streams remain compatible with the
 * navigation stack. If processing becomes too slow, downstream freshness
 * watchdogs stop autonomous motion instead of accepting stale perception.
 */

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "perception/perception_safety_core.hpp"

#include <opencv2/core.hpp>
#include <torch/script.h>
#include <torch/torch.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <unistd.h>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace perception
{

namespace
{

constexpr int MODEL_WIDTH = 640;
constexpr int MODEL_HEIGHT = 384;

rclcpp::QoS stateQos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

float sigmoid(float value)
{
  if (value >= 0.0F) {
    const float z = std::exp(-value);
    return 1.0F / (1.0F + z);
  }
  const float z = std::exp(value);
  return z / (1.0F + z);
}

struct Detection
{
  float x1{0.0F};
  float y1{0.0F};
  float x2{0.0F};
  float y2{0.0F};
  float score{0.0F};
  int class_id{-1};
};

struct MetricObstacle
{
  int track_id{-1};
  int class_id{-1};
  float score{0.0F};
  float forward_m{0.0F};
  float left_m{0.0F};
  float width_m{0.0F};
  int hits{0};
  int missed_frames{0};
  bool confirmed{false};
};

struct ObstacleTrack
{
  int track_id{-1};
  int class_id{-1};
  float score{0.0F};
  float forward_m{0.0F};
  float left_m{0.0F};
  float width_m{0.0F};
  int hits{0};
  int missed{0};
  bool confirmed{false};
};

struct NearFieldDecision
{
  bool raw_candidate{false};
  bool emergency{false};
  bool drivable_contact{false};
  int class_id{-1};
  double confidence{0.0};
  double bbox_height_fraction{0.0};
  double center_x_fraction{0.5};
  double drivable_fraction{0.0};
  std::string reason{"CLEAR"};
};

// Koridor safety diproyeksikan dari envelope fisik kendaraan ke bidang citra. Lane mask harus berada
// di luar kedua sisi koridor. Gap positif berarti mask masih di luar garis,
// gap nol/negatif berarti menyentuh atau menembus koridor.
struct LaneCorridorDecision
{
  bool enabled{false};
  bool drivable_detected{false};
  bool left_valid{false};
  bool right_valid{false};
  int drivable_rows{0};
  int left_samples{0};
  int right_samples{0};
  double drivable_fraction{0.0};
  double left_gap_px{0.0};
  double right_gap_px{0.0};
  double left_penetration_px{0.0};
  double right_penetration_px{0.0};
  double left_correction_m{0.0};
  double right_correction_m{0.0};
  double correction_error_px{0.0};
  bool left_recenter_latched{false};
  bool right_recenter_latched{false};
  std::string left_status{"UNKNOWN"};
  std::string right_status{"UNKNOWN"};
  std::string recommendation{"NONE"};
};

float intersectionOverUnion(const Detection & a, const Detection & b)
{
  const float left = std::max(a.x1, b.x1);
  const float top = std::max(a.y1, b.y1);
  const float right = std::min(a.x2, b.x2);
  const float bottom = std::min(a.y2, b.y2);
  const float intersection = std::max(0.0F, right - left) * std::max(0.0F, bottom - top);
  const float area_a = std::max(0.0F, a.x2 - a.x1) * std::max(0.0F, a.y2 - a.y1);
  const float area_b = std::max(0.0F, b.x2 - b.x1) * std::max(0.0F, b.y2 - b.y1);
  return intersection / (area_a + area_b - intersection + 1.0e-6F);
}

bool regularNonEmptyFile(const std::string & path)
{
  std::error_code error;
  return fs::is_regular_file(path, error) && !error && fs::file_size(path, error) > 0U && !error;
}

std::string resolveCpuModelPath(const std::string & requested)
{
  if (!requested.empty() && requested != "auto") {
    return fs::path(requested).lexically_normal().string();
  }

  std::vector<fs::path> candidates;
  if (const char * env = std::getenv("YOLOPV2_PT_PATH"); env && *env) {
    candidates.emplace_back(env);
  }
  if (const char * home = std::getenv("HOME"); home && *home) {
    candidates.emplace_back(fs::path(home) / "ros/models/yolopv2.pt");
  }
  std::error_code error;
  const fs::path cwd = fs::current_path(error);
  if (!error) {
    candidates.emplace_back(cwd / "models/yolopv2.pt");
  }
  // Lokasi deployment Mini-PC yang disepakati.
  candidates.emplace_back("/home/otomasi/ros/models/yolopv2.pt");

  for (const auto & candidate : candidates) {
    if (regularNonEmptyFile(candidate.string())) return candidate.lexically_normal().string();
  }
  return "auto";
}

int resolveCpuThreadCount(int requested)
{
  if (requested > 0) return std::clamp(requested, 1, 64);
  const unsigned int hardware = std::max(1U, std::thread::hardware_concurrency());
  // Sisakan core untuk ROS 2, GUI, sensor serial, EKF, dan Nav2.
  if (hardware <= 2U) return 1;
  return std::clamp(static_cast<int>(hardware) - 2, 1, 4);
}

}  // namespace

class AstraYolopCpuNode final : public rclcpp::Node
{
public:
  AstraYolopCpuNode()
  : Node("perception")
  {
    declareParameters();
    readParameters();
    validateParameters();
    inference_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    createInterfaces();
    parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() != "inference_enabled") continue;
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.successful = false;
            result.reason = "inference_enabled wajib bool";
            return result;
          }
          const bool enabled = parameter.as_bool();
          inference_enabled_.store(enabled);
          if (!enabled) {
            publishEmergency(false);
            publishHealth(true, "CAMERA_ONLY");
            publishCameraOnlyPerformance();
            RCLCPP_INFO(get_logger(), "YOLOPv2 inference OFF -> camera-only mode");
          } else {
            RCLCPP_INFO(get_logger(), "YOLOPv2 inference requested ON; model lazy-load on next frame");
          }
        }
        return result;
      });
    publishConnected(false);
    publishHealth(false, "STARTUP");
    publishEmergency(inference_enabled_.load());

    capture_thread_ = std::thread([this]() { captureLoop(); });
    const auto inference_period = std::chrono::duration<double>(1.0 / inference_fps_);
    inference_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(inference_period),
      std::bind(&AstraYolopCpuNode::inferenceTick, this), inference_group_);
    const auto control_period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      std::bind(&AstraYolopCpuNode::controlTick, this), control_group_);

    RCLCPP_INFO(
      get_logger(),
      "YOLOPv2 CPU backend siap: model=%s target_fps=%.1f threads=%d inference_default=%s. "
      "Model di-lazy-load hanya saat tab Persepsi mengaktifkan inference.",
      pt_model_path_.c_str(), inference_fps_, cpu_threads_, inference_enabled_.load() ? "ON" : "OFF");
  }

  ~AstraYolopCpuNode() override
  {
    capture_stop_.store(true);
    if (capture_thread_.joinable()) capture_thread_.join();
    if (capture_.isOpened()) capture_.release();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("pt_model_path", "/home/otomasi/ros/models/yolopv2.pt");
    declare_parameter<bool>("inference_enabled", false);
    declare_parameter<double>("cpu_inference_fps", 2.0);
    // Benchmark full-stack i5-7500 menunjukkan 2 thread paling stabil; 3-4 thread
    // mengganggu capture/ROS dan menghasilkan stall multi-detik.
    declare_parameter<int>("cpu_threads", 2);
    declare_parameter<int>("opencv_threads", 1);
    declare_parameter<bool>("torch_optimize_for_inference", true);
    declare_parameter<std::string>("rgb_device", "auto");
    declare_parameter<int>("rgb_width", 1280);
    declare_parameter<int>("rgb_height", 720);
    declare_parameter<int>("fps", 30);
    declare_parameter<std::string>("v4l2_pixel_format", "MJPEG");
    declare_parameter<bool>("strict_camera_mode", false);
    declare_parameter<bool>("flip_horizontal", true);
    declare_parameter<bool>("camera_hotplug_retry", true);
    declare_parameter<double>("camera_retry_interval_sec", 2.0);
    declare_parameter<int>("camera_read_fail_threshold", 5);
    declare_parameter<int>("camera_online_good_frames", 3);
    declare_parameter<double>("confidence_threshold", 0.10);
    declare_parameter<double>("iou_threshold", 0.45);
    declare_parameter<double>("lane_threshold", 0.50);
    declare_parameter<int>("max_candidates", 16384);
    declare_parameter<int>("max_detections", 300);
    declare_parameter<bool>("publish_annotated", true);
    declare_parameter<bool>("publish_raw_rgb", true);
    declare_parameter<double>("rviz_max_publish_rate_hz", 10.0);
    declare_parameter<bool>("web_preview_enabled", true);
    declare_parameter<double>("web_preview_fps", 5.0);
    declare_parameter<int>("web_preview_width", 640);
    declare_parameter<int>("web_preview_height", 360);
    declare_parameter<int>("web_preview_jpeg_quality", 75);
    declare_parameter<std::string>("web_preview_topic", "/camera/astra/image_preview/compressed");
    declare_parameter<bool>("publish_drivable_mask", false);
    declare_parameter<bool>("publish_lane_mask", false);
    declare_parameter<bool>("publish_detections", false);
    declare_parameter<double>("overlay_alpha", 0.45);
    declare_parameter<int>("box_thickness", 2);
    declare_parameter<std::string>("frame_id", "camera_color_optical_frame");
    declare_parameter<std::string>("metric_frame_id", "base_footprint");
    declare_parameter<std::string>("annotated_topic", "/camera/yolop/image_annotated");
    declare_parameter<std::string>("raw_topic", "/camera/astra/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera/color/camera_info");
    declare_parameter<std::string>("detections_topic", "/yolop/detections");
    declare_parameter<std::string>("drivable_mask_topic", "/yolop/drivable_mask");
    declare_parameter<std::string>("lane_mask_topic", "/yolop/lane_mask");
    declare_parameter<std::string>("performance_topic", "/perception/performance");
    declare_parameter<std::string>("lane_metrics_topic", "/yolop/lane_metrics");
    declare_parameter<std::string>("drivable_space_topic", "/perception/drivable_space");
    declare_parameter<std::string>("perception_obstacle_metrics_topic", "/perception/obstacle_metrics");
    declare_parameter<std::string>("near_field_state_topic", "/perception/near_field_state");
    declare_parameter<std::string>("camera_connected_topic", "/perception/camera_connected");
    declare_parameter<std::string>("camera_health_topic", "/perception/camera_healthy");
    declare_parameter<std::string>("camera_health_state_topic", "/perception/camera_health_state");
    declare_parameter<std::string>("emergency_stop_topic", "/perception/emergency_stop");
    declare_parameter<std::string>("object_points_topic", "/perception/object_points");
    declare_parameter<std::string>("object_clearing_points_topic", "/perception/object_clearing_points");
    declare_parameter<std::string>("drivable_boundary_points_topic", "/perception/drivable_boundary_points");
    declare_parameter<std::string>("lane_state_topic", "/perception/lane_safety_state");
    declare_parameter<std::string>("lane_control_state_topic", "/perception/lane_control_state");
    declare_parameter<std::string>("raw_detection_summary_topic", "/perception/raw_detections");
    declare_parameter<std::string>("nav_cmd_topic", "/cmd_vel_nav_smoothed");
    declare_parameter<std::string>("safe_cmd_topic", "/cmd_vel/perception_advisory");
    declare_parameter<double>("control_rate_hz", 20.0);
    declare_parameter<double>("cmd_timeout_sec", 0.50);
    declare_parameter<double>("lane_state_timeout_sec", 0.50);
    declare_parameter<bool>("camera_metric_calibration_validated", false);
    declare_parameter<bool>("lane_safety_enabled", false);
    // Dua garis safety diproyeksikan dari envelope kendaraan + geometri kamera.
    // Default sinkron dengan URDF ADV: lebar 0.55 m, tinggi kamera 0.736 m, pitch 0 deg.
    // Offset pixel disediakan hanya untuk fine-calibration pemasangan kamera nyata.
    declare_parameter<bool>("lane_corridor_overlay_enabled", true);
    declare_parameter<bool>("lane_corridor_control_enabled", true);
    declare_parameter<double>("lane_corridor_top_y_ratio", 0.02);
    declare_parameter<double>("lane_corridor_bottom_y_ratio", 0.98);
    declare_parameter<double>("lane_corridor_camera_height_m", 0.736);
    declare_parameter<double>("lane_corridor_camera_pitch_deg", 0.0);
    declare_parameter<double>("lane_corridor_safety_margin_m", 0.30);
    declare_parameter<double>("lane_corridor_far_lookahead_m", 3.8);
    declare_parameter<double>("lane_corridor_center_offset_px", 0.0);
    declare_parameter<double>("lane_corridor_left_offset_px", 0.0);
    declare_parameter<double>("lane_corridor_right_offset_px", 0.0);
    declare_parameter<double>("lane_corridor_warning_gap_px", 36.0);
    declare_parameter<double>("lane_corridor_touch_margin_px", 2.0);
    declare_parameter<double>("lane_corridor_release_gap_px", 48.0);
    declare_parameter<double>("lane_corridor_critical_penetration_px", 24.0);
    declare_parameter<int>("lane_corridor_sample_stride_px", 6);
    declare_parameter<int>("lane_corridor_minimum_valid_rows", 6);
    declare_parameter<double>("lane_corridor_minimum_drivable_fraction", 0.20);
    declare_parameter<double>("lane_corridor_correction_gain_m_per_px", 0.004);
    declare_parameter<double>("lane_corridor_max_correction_m", 0.35);
    declare_parameter<std::string>("control_mode", "active");
    declare_parameter<double>("wheelbase_m", 0.70);
    declare_parameter<double>("maximum_steering_angle_rad", 0.34);
    declare_parameter<double>("recenter_speed_mps", 0.20);
    declare_parameter<double>("critical_recenter_speed_mps", 0.10);
    declare_parameter<double>("center_gain", 0.55);
    declare_parameter<double>("heading_gain", 0.70);
    declare_parameter<double>("lane_blend_gain", 1.0);
    declare_parameter<double>("minimum_speed_for_yaw_limit_mps", 0.10);

    declare_parameter<int>("ground_calibration_width", 1280);
    declare_parameter<int>("ground_calibration_height", 720);
    declare_parameter<std::vector<double>>(
      "ground_src_points", {40.0, 680.0, 1240.0, 680.0, 760.0, 350.0, 520.0, 350.0});
    declare_parameter<std::vector<double>>(
      "ground_dst_points", {0.0, 513.5714, 1279.0, 503.30, 1279.0, 10.2714, 0.0, 0.0});
    declare_parameter<int>("ground_canvas_width", 1280);
    declare_parameter<int>("ground_canvas_height", 720);
    declare_parameter<double>("ground_origin_x_px", 596.8667);
    declare_parameter<double>("ground_origin_y_px", 719.0);
    declare_parameter<double>("ground_meters_per_pixel_x", 0.0039093041);
    declare_parameter<double>("ground_meters_per_pixel_y", 0.0097357441);
    declare_parameter<double>("metric_forward_offset_m", 0.0);
    declare_parameter<double>("metric_lateral_offset_m", 0.0);
    declare_parameter<double>("metric_minimum_forward_m", 0.20);
    declare_parameter<double>("metric_maximum_forward_m", 4.0);
    declare_parameter<double>("metric_maximum_abs_left_m", 2.5);
    declare_parameter<double>("minimum_obstacle_confidence", 0.30);
    // Per-class affine correction: [human_scale, human_bias_m, motorcycle_scale, motorcycle_bias_m].
    // Identity is safe until the 1..5 m physical calibration wizard is completed.
    rcl_interfaces::msg::ParameterDescriptor obstacle_calibration_descriptor;
    obstacle_calibration_descriptor.dynamic_typing = true;
    declare_parameter(
      "obstacle_distance_calibration_coefficients",
      rclcpp::ParameterValue(std::vector<double>{1.0, 0.0, 1.0, 0.0}),
      obstacle_calibration_descriptor);
    declare_parameter<bool>("accept_all_detected_classes_as_obstacles", false);
    declare_parameter<std::vector<int64_t>>("safety_obstacle_class_ids", {0, 2, 3});
    declare_parameter<bool>("require_drivable_contact", true);
    declare_parameter<int>("drivable_contact_radius_px", 12);
    declare_parameter<int>("drivable_contact_vertical_tolerance_px", 8);
    declare_parameter<int>("drivable_contact_min_samples", 3);
    declare_parameter<double>("drivable_contact_min_fraction", 0.20);
    declare_parameter<double>("track_match_distance_m", 0.90);
    declare_parameter<double>("track_ema_alpha", 0.45);
    declare_parameter<int>("track_confirm_hits", 3);
    declare_parameter<int>("track_max_missed_frames", 8);
    declare_parameter<double>("obstacle_forward_min_m", 0.20);
    declare_parameter<double>("obstacle_forward_max_m", 3.0);
    declare_parameter<double>("minimum_obstacle_width_m", 0.20);
    declare_parameter<int>("points_per_box", 5);
    declare_parameter<int>("clearing_ray_count", 41);
    declare_parameter<double>("clearing_fov_deg", 100.0);
    declare_parameter<double>("clearing_range_m", 4.5);
    declare_parameter<double>("clearing_obstacle_margin_m", 0.20);
    declare_parameter<double>("cpu_emergency_stop_distance_m", 0.65);
    declare_parameter<double>("cpu_emergency_half_width_m", 0.55);
    // Near-field image-space guard tetap valid sebelum homography metriks tersertifikasi.
    declare_parameter<bool>("near_field_emergency_enabled", true);
    declare_parameter<double>("near_field_min_confidence", 0.25);
    declare_parameter<double>("near_field_center_corridor_fraction", 0.65);
    declare_parameter<double>("near_field_min_bbox_height_fraction", 0.20);
    declare_parameter<bool>("near_field_drivable_guard_enabled", true);
    declare_parameter<double>("near_field_bottom_roi_fraction", 0.22);
    declare_parameter<double>("near_field_min_drivable_fraction", 0.12);
    declare_parameter<int>("near_field_confirm_frames", 2);
    declare_parameter<int>("near_field_release_frames", 5);
    declare_parameter<double>("lane_vehicle_width_m", 0.55);
    declare_parameter<double>("edge_warning_clearance_m", 1.0);
    declare_parameter<double>("edge_critical_clearance_m", 0.40);
    declare_parameter<double>("edge_release_clearance_m", 1.20);
    declare_parameter<double>("center_deadband_m", 0.25);
    declare_parameter<int>("state_confirm_frames", 3);
    declare_parameter<int>("release_confirm_frames", 5);
    declare_parameter<int>("lost_confirm_frames", 2);
    declare_parameter<int>("camera_health_dark_luma", 18);
    declare_parameter<int>("camera_health_bright_luma", 245);
    declare_parameter<double>("camera_health_extreme_fraction", 0.97);
    declare_parameter<double>("camera_health_min_stddev", 5.0);
    declare_parameter<double>("camera_health_min_gradient", 2.0);
    declare_parameter<double>("camera_fx", 910.0);
    declare_parameter<double>("camera_fy", 910.0);
    declare_parameter<double>("camera_cx", 640.0);
    declare_parameter<double>("camera_cy", 360.0);
    declare_parameter<std::vector<double>>("camera_distortion", {0.0, 0.0, 0.0, 0.0, 0.0});
  }

  void readParameters()
  {
    pt_model_path_ = resolveCpuModelPath(get_parameter("pt_model_path").as_string());
    inference_enabled_.store(get_parameter("inference_enabled").as_bool());
    inference_fps_ = get_parameter("cpu_inference_fps").as_double();
    cpu_threads_ = resolveCpuThreadCount(get_parameter("cpu_threads").as_int());
    opencv_threads_ = get_parameter("opencv_threads").as_int();
    torch_optimize_for_inference_ = get_parameter("torch_optimize_for_inference").as_bool();
    rgb_device_ = get_parameter("rgb_device").as_string();
    requested_width_ = get_parameter("rgb_width").as_int();
    requested_height_ = get_parameter("rgb_height").as_int();
    camera_fps_ = get_parameter("fps").as_int();
    pixel_format_ = get_parameter("v4l2_pixel_format").as_string();
    strict_camera_mode_ = get_parameter("strict_camera_mode").as_bool();
    flip_horizontal_ = get_parameter("flip_horizontal").as_bool();
    hotplug_retry_ = get_parameter("camera_hotplug_retry").as_bool();
    retry_sec_ = get_parameter("camera_retry_interval_sec").as_double();
    camera_read_fail_threshold_ = static_cast<int>(
      std::clamp<std::int64_t>(get_parameter("camera_read_fail_threshold").as_int(), 2, 30));
    camera_online_good_frames_ = static_cast<int>(
      std::clamp<std::int64_t>(get_parameter("camera_online_good_frames").as_int(), 1, 30));
    confidence_threshold_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
    iou_threshold_ = static_cast<float>(get_parameter("iou_threshold").as_double());
    lane_threshold_ = static_cast<float>(get_parameter("lane_threshold").as_double());
    max_candidates_ = get_parameter("max_candidates").as_int();
    max_detections_ = get_parameter("max_detections").as_int();
    publish_annotated_ = get_parameter("publish_annotated").as_bool();
    publish_raw_ = get_parameter("publish_raw_rgb").as_bool();
    visual_publish_rate_hz_ = get_parameter("rviz_max_publish_rate_hz").as_double();
    web_preview_enabled_ = get_parameter("web_preview_enabled").as_bool();
    web_preview_fps_ = get_parameter("web_preview_fps").as_double();
    web_preview_width_ = get_parameter("web_preview_width").as_int();
    web_preview_height_ = get_parameter("web_preview_height").as_int();
    web_preview_jpeg_quality_ = get_parameter("web_preview_jpeg_quality").as_int();
    web_preview_topic_ = get_parameter("web_preview_topic").as_string();
    publish_drivable_ = get_parameter("publish_drivable_mask").as_bool();
    publish_lane_ = get_parameter("publish_lane_mask").as_bool();
    publish_detections_ = get_parameter("publish_detections").as_bool();
    overlay_alpha_ = get_parameter("overlay_alpha").as_double();
    box_thickness_ = get_parameter("box_thickness").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    metric_frame_id_ = get_parameter("metric_frame_id").as_string();
    annotated_topic_ = get_parameter("annotated_topic").as_string();
    raw_topic_ = get_parameter("raw_topic").as_string();
    camera_info_topic_ = get_parameter("camera_info_topic").as_string();
    detections_topic_ = get_parameter("detections_topic").as_string();
    drivable_topic_ = get_parameter("drivable_mask_topic").as_string();
    lane_topic_ = get_parameter("lane_mask_topic").as_string();
    performance_topic_ = get_parameter("performance_topic").as_string();
    lane_metrics_topic_ = get_parameter("lane_metrics_topic").as_string();
    drivable_space_topic_ = get_parameter("drivable_space_topic").as_string();
    obstacle_metrics_topic_ = get_parameter("perception_obstacle_metrics_topic").as_string();
    near_field_state_topic_ = get_parameter("near_field_state_topic").as_string();
    camera_connected_topic_ = get_parameter("camera_connected_topic").as_string();
    camera_health_topic_ = get_parameter("camera_health_topic").as_string();
    camera_health_state_topic_ = get_parameter("camera_health_state_topic").as_string();
    emergency_topic_ = get_parameter("emergency_stop_topic").as_string();
    object_points_topic_ = get_parameter("object_points_topic").as_string();
    clearing_points_topic_ = get_parameter("object_clearing_points_topic").as_string();
    drivable_boundary_topic_ = get_parameter("drivable_boundary_points_topic").as_string();
    lane_state_topic_ = get_parameter("lane_state_topic").as_string();
    lane_control_state_topic_ = get_parameter("lane_control_state_topic").as_string();
    raw_detection_topic_ = get_parameter("raw_detection_summary_topic").as_string();
    nav_cmd_topic_ = get_parameter("nav_cmd_topic").as_string();
    safe_cmd_topic_ = get_parameter("safe_cmd_topic").as_string();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    cmd_timeout_sec_ = get_parameter("cmd_timeout_sec").as_double();
    lane_state_timeout_sec_ = get_parameter("lane_state_timeout_sec").as_double();
    camera_metric_calibration_validated_ =
      get_parameter("camera_metric_calibration_validated").as_bool();
    lane_safety_enabled_ = get_parameter("lane_safety_enabled").as_bool();
    lane_corridor_overlay_enabled_ = get_parameter("lane_corridor_overlay_enabled").as_bool();
    lane_corridor_control_enabled_ = get_parameter("lane_corridor_control_enabled").as_bool();
    lane_corridor_top_y_ratio_ = get_parameter("lane_corridor_top_y_ratio").as_double();
    lane_corridor_bottom_y_ratio_ = get_parameter("lane_corridor_bottom_y_ratio").as_double();
    lane_corridor_camera_height_m_ = get_parameter("lane_corridor_camera_height_m").as_double();
    lane_corridor_camera_pitch_deg_ = get_parameter("lane_corridor_camera_pitch_deg").as_double();
    lane_corridor_safety_margin_m_ = get_parameter("lane_corridor_safety_margin_m").as_double();
    lane_corridor_far_lookahead_m_ = get_parameter("lane_corridor_far_lookahead_m").as_double();
    lane_corridor_center_offset_px_ = get_parameter("lane_corridor_center_offset_px").as_double();
    lane_corridor_left_offset_px_ = get_parameter("lane_corridor_left_offset_px").as_double();
    lane_corridor_right_offset_px_ = get_parameter("lane_corridor_right_offset_px").as_double();
    lane_corridor_warning_gap_px_ = get_parameter("lane_corridor_warning_gap_px").as_double();
    lane_corridor_touch_margin_px_ = get_parameter("lane_corridor_touch_margin_px").as_double();
    lane_corridor_release_gap_px_ = get_parameter("lane_corridor_release_gap_px").as_double();
    lane_corridor_critical_penetration_px_ = get_parameter("lane_corridor_critical_penetration_px").as_double();
    lane_corridor_sample_stride_px_ = get_parameter("lane_corridor_sample_stride_px").as_int();
    lane_corridor_minimum_valid_rows_ = get_parameter("lane_corridor_minimum_valid_rows").as_int();
    lane_corridor_minimum_drivable_fraction_ =
      get_parameter("lane_corridor_minimum_drivable_fraction").as_double();
    lane_corridor_correction_gain_m_per_px_ = get_parameter("lane_corridor_correction_gain_m_per_px").as_double();
    lane_corridor_max_correction_m_ = get_parameter("lane_corridor_max_correction_m").as_double();
    control_mode_ = get_parameter("control_mode").as_string();
    mixer_config_.wheelbase_m = get_parameter("wheelbase_m").as_double();
    mixer_config_.maximum_steering_angle_rad =
      get_parameter("maximum_steering_angle_rad").as_double();
    mixer_config_.recenter_speed_mps = get_parameter("recenter_speed_mps").as_double();
    mixer_config_.critical_recenter_speed_mps =
      get_parameter("critical_recenter_speed_mps").as_double();
    mixer_config_.center_gain = get_parameter("center_gain").as_double();
    mixer_config_.heading_gain = get_parameter("heading_gain").as_double();
    mixer_config_.lane_blend_gain = get_parameter("lane_blend_gain").as_double();
    mixer_config_.minimum_speed_for_yaw_limit_mps =
      get_parameter("minimum_speed_for_yaw_limit_mps").as_double();

    ground_calibration_width_ = get_parameter("ground_calibration_width").as_int();
    ground_calibration_height_ = get_parameter("ground_calibration_height").as_int();
    ground_src_ = get_parameter("ground_src_points").as_double_array();
    ground_dst_ = get_parameter("ground_dst_points").as_double_array();
    ground_canvas_width_ = get_parameter("ground_canvas_width").as_int();
    ground_canvas_height_ = get_parameter("ground_canvas_height").as_int();
    ground_origin_x_ = get_parameter("ground_origin_x_px").as_double();
    ground_origin_y_ = get_parameter("ground_origin_y_px").as_double();
    ground_scale_x_ = get_parameter("ground_meters_per_pixel_x").as_double();
    ground_scale_y_ = get_parameter("ground_meters_per_pixel_y").as_double();
    forward_offset_ = get_parameter("metric_forward_offset_m").as_double();
    lateral_offset_ = get_parameter("metric_lateral_offset_m").as_double();
    min_forward_ = get_parameter("metric_minimum_forward_m").as_double();
    max_forward_ = get_parameter("metric_maximum_forward_m").as_double();
    max_abs_left_ = get_parameter("metric_maximum_abs_left_m").as_double();
    minimum_obstacle_confidence_ = get_parameter("minimum_obstacle_confidence").as_double();
    {
      const auto calibration_parameter = get_parameter("obstacle_distance_calibration_coefficients");
      if (calibration_parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        obstacle_distance_calibration_coefficients_ = calibration_parameter.as_double_array();
      } else if (calibration_parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        obstacle_distance_calibration_coefficients_.clear();
        for (const auto value : calibration_parameter.as_integer_array())
          obstacle_distance_calibration_coefficients_.push_back(static_cast<double>(value));
        RCLCPP_WARN(get_logger(),
          "obstacle_distance_calibration_coefficients arrived as integer_array; converted safely to double_array");
      } else {
        obstacle_distance_calibration_coefficients_.clear();
      }
      if (obstacle_distance_calibration_coefficients_.size() != 4U)
        obstacle_distance_calibration_coefficients_ = {1.0, 0.0, 1.0, 0.0};
    }
    obstacle_distance_calibration_coefficients_[0] =
      std::clamp(obstacle_distance_calibration_coefficients_[0], 0.50, 1.50);
    obstacle_distance_calibration_coefficients_[1] =
      std::clamp(obstacle_distance_calibration_coefficients_[1], -2.0, 2.0);
    obstacle_distance_calibration_coefficients_[2] =
      std::clamp(obstacle_distance_calibration_coefficients_[2], 0.50, 1.50);
    obstacle_distance_calibration_coefficients_[3] =
      std::clamp(obstacle_distance_calibration_coefficients_[3], -2.0, 2.0);
    accept_all_obstacles_ = get_parameter("accept_all_detected_classes_as_obstacles").as_bool();
    safety_classes_ = get_parameter("safety_obstacle_class_ids").as_integer_array();
    require_drivable_contact_ = get_parameter("require_drivable_contact").as_bool();
    drivable_contact_radius_px_ = get_parameter("drivable_contact_radius_px").as_int();
    drivable_contact_vertical_tolerance_px_ = get_parameter("drivable_contact_vertical_tolerance_px").as_int();
    drivable_contact_min_samples_ = get_parameter("drivable_contact_min_samples").as_int();
    drivable_contact_min_fraction_ = get_parameter("drivable_contact_min_fraction").as_double();
    track_match_distance_m_ = get_parameter("track_match_distance_m").as_double();
    track_ema_alpha_ = get_parameter("track_ema_alpha").as_double();
    track_confirm_hits_ = get_parameter("track_confirm_hits").as_int();
    track_max_missed_frames_ = get_parameter("track_max_missed_frames").as_int();
    obstacle_forward_min_m_ = get_parameter("obstacle_forward_min_m").as_double();
    obstacle_forward_max_m_ = get_parameter("obstacle_forward_max_m").as_double();
    minimum_obstacle_width_m_ = get_parameter("minimum_obstacle_width_m").as_double();
    points_per_box_ = get_parameter("points_per_box").as_int();
    clearing_ray_count_ = get_parameter("clearing_ray_count").as_int();
    clearing_fov_rad_ = get_parameter("clearing_fov_deg").as_double() *
      3.14159265358979323846 / 180.0;
    clearing_range_m_ = get_parameter("clearing_range_m").as_double();
    clearing_obstacle_margin_m_ = get_parameter("clearing_obstacle_margin_m").as_double();
    emergency_distance_m_ = get_parameter("cpu_emergency_stop_distance_m").as_double();
    emergency_half_width_m_ = get_parameter("cpu_emergency_half_width_m").as_double();
    near_field_emergency_enabled_ = get_parameter("near_field_emergency_enabled").as_bool();
    near_field_min_confidence_ = get_parameter("near_field_min_confidence").as_double();
    near_field_center_corridor_fraction_ = get_parameter("near_field_center_corridor_fraction").as_double();
    near_field_min_bbox_height_fraction_ = get_parameter("near_field_min_bbox_height_fraction").as_double();
    near_field_drivable_guard_enabled_ = get_parameter("near_field_drivable_guard_enabled").as_bool();
    near_field_bottom_roi_fraction_ = get_parameter("near_field_bottom_roi_fraction").as_double();
    near_field_min_drivable_fraction_ = get_parameter("near_field_min_drivable_fraction").as_double();
    near_field_confirm_frames_ = get_parameter("near_field_confirm_frames").as_int();
    near_field_release_frames_ = get_parameter("near_field_release_frames").as_int();
    lane_vehicle_width_m_ = get_parameter("lane_vehicle_width_m").as_double();
    lane_thresholds_.warning_clearance_m = get_parameter("edge_warning_clearance_m").as_double();
    lane_thresholds_.critical_clearance_m = get_parameter("edge_critical_clearance_m").as_double();
    lane_thresholds_.release_clearance_m = get_parameter("edge_release_clearance_m").as_double();
    lane_thresholds_.center_deadband_m = get_parameter("center_deadband_m").as_double();
    lane_state_filter_ = std::make_unique<safety::ConfirmedState>(
      safety::LANE_LOST,
      get_parameter("state_confirm_frames").as_int(),
      get_parameter("release_confirm_frames").as_int(),
      get_parameter("lost_confirm_frames").as_int());
    health_dark_ = get_parameter("camera_health_dark_luma").as_int();
    health_bright_ = get_parameter("camera_health_bright_luma").as_int();
    health_extreme_fraction_ = get_parameter("camera_health_extreme_fraction").as_double();
    health_min_stddev_ = get_parameter("camera_health_min_stddev").as_double();
    health_min_gradient_ = get_parameter("camera_health_min_gradient").as_double();
    camera_fx_ = get_parameter("camera_fx").as_double();
    camera_fy_ = get_parameter("camera_fy").as_double();
    camera_cx_ = get_parameter("camera_cx").as_double();
    camera_cy_ = get_parameter("camera_cy").as_double();
    camera_distortion_ = get_parameter("camera_distortion").as_double_array();
  }

  void validateParameters()
  {
    if (!regularNonEmptyFile(pt_model_path_)) {
      throw std::runtime_error("YOLOPv2 .pt tidak ditemukan/kosong: " + pt_model_path_);
    }
    const fs::path pt_path(pt_model_path_);
    if (pt_path.extension() != ".pt" && pt_path.extension() != ".torchscript") {
      throw std::runtime_error("CPU backend membutuhkan model TorchScript .pt/.torchscript: " + pt_model_path_);
    }
    inference_fps_ = std::clamp(inference_fps_, 0.5, 30.0);
    cpu_threads_ = std::clamp(cpu_threads_, 1, 64);
    opencv_threads_ = std::clamp(opencv_threads_, 1, 8);
    requested_width_ = std::max(320, requested_width_);
    requested_height_ = std::max(240, requested_height_);
    camera_fps_ = std::clamp(camera_fps_, 1, 60);
    std::transform(pixel_format_.begin(), pixel_format_.end(), pixel_format_.begin(),
      [](unsigned char value) {return static_cast<char>(std::toupper(value));});
    if (pixel_format_ != "MJPEG" && pixel_format_ != "YUYV") {
      throw std::runtime_error("v4l2_pixel_format CPU harus MJPEG atau YUYV");
    }
    retry_sec_ = std::clamp(retry_sec_, 0.5, 30.0);
    confidence_threshold_ = std::clamp(confidence_threshold_, 0.01F, 0.99F);
    iou_threshold_ = std::clamp(iou_threshold_, 0.01F, 0.99F);
    lane_threshold_ = std::clamp(lane_threshold_, 0.01F, 0.99F);
    max_candidates_ = std::clamp(max_candidates_, 100, 100000);
    max_detections_ = std::clamp(max_detections_, 1, 1000);
    overlay_alpha_ = std::clamp(overlay_alpha_, 0.0, 1.0);
    box_thickness_ = std::clamp(box_thickness_, 1, 10);
    control_rate_hz_ = std::clamp(control_rate_hz_, 5.0, 50.0);
    cmd_timeout_sec_ = std::clamp(cmd_timeout_sec_, 0.10, 2.0);
    lane_state_timeout_sec_ = std::clamp(lane_state_timeout_sec_, 0.10, 2.0);
    if (control_mode_ != "active" && control_mode_ != "monitor") {
      throw std::runtime_error("control_mode CPU harus 'active' atau 'monitor'");
    }
    mixer_config_.validate();
    if (ground_src_.size() != 8U || ground_dst_.size() != 8U ||
      ground_calibration_width_ <= 0 || ground_calibration_height_ <= 0 ||
      ground_canvas_width_ <= 0 || ground_canvas_height_ <= 0 ||
      ground_scale_x_ <= 0.0 || ground_scale_y_ <= 0.0 ||
      max_forward_ <= min_forward_ || max_abs_left_ <= 0.0)
    {
      throw std::runtime_error("Parameter homography/ground metric CPU tidak valid");
    }
    lane_thresholds_.validate();
    lane_corridor_top_y_ratio_ = std::clamp(lane_corridor_top_y_ratio_, 0.0, 0.95);
    lane_corridor_bottom_y_ratio_ = std::clamp(
      lane_corridor_bottom_y_ratio_, lane_corridor_top_y_ratio_ + 0.05, 0.99);
    lane_corridor_camera_height_m_ = std::clamp(lane_corridor_camera_height_m_, 0.20, 2.50);
    lane_corridor_camera_pitch_deg_ = std::clamp(lane_corridor_camera_pitch_deg_, -25.0, 45.0);
    lane_corridor_safety_margin_m_ = std::clamp(lane_corridor_safety_margin_m_, 0.0, 1.50);
    lane_corridor_far_lookahead_m_ = std::clamp(lane_corridor_far_lookahead_m_, 1.0, 20.0);
    lane_corridor_center_offset_px_ = std::clamp(lane_corridor_center_offset_px_, -500.0, 500.0);
    lane_corridor_left_offset_px_ = std::clamp(lane_corridor_left_offset_px_, -500.0, 500.0);
    lane_corridor_right_offset_px_ = std::clamp(lane_corridor_right_offset_px_, -500.0, 500.0);
    lane_corridor_warning_gap_px_ = std::clamp(lane_corridor_warning_gap_px_, 2.0, 300.0);
    lane_corridor_touch_margin_px_ = std::clamp(
      lane_corridor_touch_margin_px_, 0.0, lane_corridor_warning_gap_px_ - 1.0);
    lane_corridor_release_gap_px_ = std::clamp(
      lane_corridor_release_gap_px_, lane_corridor_warning_gap_px_ + 1.0, 500.0);
    lane_corridor_critical_penetration_px_ = std::clamp(
      lane_corridor_critical_penetration_px_, 1.0, 300.0);
    lane_corridor_sample_stride_px_ = std::clamp(lane_corridor_sample_stride_px_, 2, 40);
    lane_corridor_minimum_valid_rows_ = std::clamp(lane_corridor_minimum_valid_rows_, 2, 100);
    lane_corridor_minimum_drivable_fraction_ =
      std::clamp(lane_corridor_minimum_drivable_fraction_, 0.01, 0.95);
    lane_corridor_correction_gain_m_per_px_ = std::clamp(
      lane_corridor_correction_gain_m_per_px_, 0.0, 0.05);
    lane_corridor_max_correction_m_ = std::clamp(lane_corridor_max_correction_m_, 0.0, 1.0);
    visual_publish_rate_hz_ = std::clamp(visual_publish_rate_hz_, 1.0, 30.0);
    web_preview_fps_ = std::clamp(web_preview_fps_, 1.0, 12.0);
    web_preview_width_ = std::clamp(web_preview_width_, 160, requested_width_);
    web_preview_height_ = std::clamp(web_preview_height_, 90, requested_height_);
    web_preview_jpeg_quality_ = std::clamp(web_preview_jpeg_quality_, 40, 95);
    if (web_preview_topic_.empty()) web_preview_topic_ = "/camera/astra/image_preview/compressed";
    drivable_contact_radius_px_ = std::clamp(drivable_contact_radius_px_, 0, 128);
    drivable_contact_vertical_tolerance_px_ = std::clamp(drivable_contact_vertical_tolerance_px_, 0, 128);
    drivable_contact_min_samples_ = std::clamp(drivable_contact_min_samples_, 1, 50);
    drivable_contact_min_fraction_ = std::clamp(drivable_contact_min_fraction_, 0.0, 1.0);
    track_match_distance_m_ = std::max(0.05, track_match_distance_m_);
    track_ema_alpha_ = std::clamp(track_ema_alpha_, 0.01, 1.0);
    track_confirm_hits_ = std::clamp(track_confirm_hits_, 1, 30);
    track_max_missed_frames_ = std::clamp(track_max_missed_frames_, 0, 100);
    obstacle_forward_min_m_ = std::max(0.0, obstacle_forward_min_m_);
    obstacle_forward_max_m_ = std::min(max_forward_, std::max(obstacle_forward_min_m_ + 0.05, obstacle_forward_max_m_));
    minimum_obstacle_width_m_ = std::clamp(minimum_obstacle_width_m_, 0.05, 3.0);
    near_field_min_confidence_ = std::clamp(near_field_min_confidence_, 0.01, 1.0);
    near_field_center_corridor_fraction_ = std::clamp(near_field_center_corridor_fraction_, 0.10, 1.0);
    near_field_min_bbox_height_fraction_ = std::clamp(near_field_min_bbox_height_fraction_, 0.05, 0.95);
    near_field_bottom_roi_fraction_ = std::clamp(near_field_bottom_roi_fraction_, 0.05, 0.80);
    near_field_min_drivable_fraction_ = std::clamp(near_field_min_drivable_fraction_, 0.0, 1.0);
    near_field_confirm_frames_ = std::clamp(near_field_confirm_frames_, 1, 30);
    near_field_release_frames_ = std::clamp(near_field_release_frames_, 1, 60);
    points_per_box_ = std::clamp(points_per_box_, 1, 21);
    clearing_ray_count_ = std::clamp(clearing_ray_count_, 3, 181);
    cv::setNumThreads(opencv_threads_);
  }

  static cv::Mat tensorToCv(const torch::Tensor & source)
  {
    auto tensor = source.detach().to(torch::kCPU).to(torch::kFloat32).contiguous();
    std::vector<int> sizes;
    sizes.reserve(static_cast<size_t>(tensor.dim()));
    for (const auto value : tensor.sizes()) {
      sizes.push_back(static_cast<int>(value));
    }
    cv::Mat view(static_cast<int>(sizes.size()), sizes.data(), CV_32F, tensor.data_ptr<float>());
    return view.clone();
  }

  std::vector<cv::Mat> forwardTorch(const cv::Mat & input)
  {
    cv::Mat rgb;
    cv::cvtColor(input, rgb, cv::COLOR_BGR2RGB);
    auto tensor = torch::from_blob(
      rgb.data, {1, rgb.rows, rgb.cols, 3},
      torch::TensorOptions().dtype(torch::kUInt8));
    tensor = tensor.permute({0, 3, 1, 2}).to(torch::kFloat32).div_(255.0).contiguous();

    torch::InferenceMode inference_guard;
    const auto output = module_.forward({tensor});
    if (!output.isTuple()) {
      throw std::runtime_error("Output root YOLOPv2 .pt harus tuple 3-item");
    }
    const auto root = output.toTuple()->elements();
    if (root.size() != 3U) {
      throw std::runtime_error("Output root YOLOPv2 .pt bukan 3-item ([pred,anchor],seg,lane)");
    }

    std::vector<torch::Tensor> prediction_heads;
    std::vector<torch::Tensor> anchor_heads;
    const auto extract_tensor_list = [](const torch::IValue & value, const char * label) {
      std::vector<torch::Tensor> tensors;
      if (value.isTensorList()) {
        for (const auto & tensor : value.toTensorVector()) tensors.push_back(tensor);
      } else if (value.isList()) {
        // GenericList iteration returns c10::ListElementReference on newer
        // LibTorch releases.  That proxy deliberately has no isTensor()/
        // toTensor() members.  Access through List::get() materializes the
        // underlying IValue and is compatible with ROS Humble-era and current
        // PyTorch C++ APIs.
        const auto list = value.toList();
        tensors.reserve(static_cast<size_t>(list.size()));
        for (size_t i = 0; i < static_cast<size_t>(list.size()); ++i) {
          const torch::IValue element = list.get(i);
          if (!element.isTensor()) {
            throw std::runtime_error(std::string(label) + " berisi non-tensor");
          }
          tensors.push_back(element.toTensor());
        }
      } else if (value.isTuple()) {
        for (const auto & element : value.toTuple()->elements()) {
          if (!element.isTensor()) throw std::runtime_error(std::string(label) + " berisi non-tensor");
          tensors.push_back(element.toTensor());
        }
      } else {
        throw std::runtime_error(std::string(label) + " bukan list/tuple tensor");
      }
      return tensors;
    };

    if (!root[0].isTuple() && !root[0].isList()) {
      throw std::runtime_error("Detection pack YOLOPv2 .pt bukan [pred, anchor_grid]");
    }
    std::vector<torch::IValue> det_items;
    if (root[0].isTuple()) det_items = root[0].toTuple()->elements();
    else {
      const auto list = root[0].toList();
      det_items.reserve(static_cast<size_t>(list.size()));
      for (size_t i = 0; i < static_cast<size_t>(list.size()); ++i) {
        det_items.push_back(list.get(i));
      }
    }
    if (det_items.size() != 2U) {
      throw std::runtime_error("Detection pack YOLOPv2 .pt harus tepat 2-item");
    }
    prediction_heads = extract_tensor_list(det_items[0], "pred");
    anchor_heads = extract_tensor_list(det_items[1], "anchor_grid");
    if (prediction_heads.size() != 3U || anchor_heads.size() != 3U) {
      throw std::runtime_error("YOLOPv2 .pt harus memiliki 3 prediction head dan 3 anchor head");
    }
    if (!root[1].isTensor() || !root[2].isTensor()) {
      throw std::runtime_error("Output drivable/lane YOLOPv2 .pt harus tensor");
    }

    std::vector<cv::Mat> outputs;
    outputs.reserve(8U);
    for (const auto & tensor_head : prediction_heads) outputs.push_back(tensorToCv(tensor_head));
    for (const auto & tensor_head : anchor_heads) outputs.push_back(tensorToCv(tensor_head));
    outputs.push_back(tensorToCv(root[1].toTensor()));
    outputs.push_back(tensorToCv(root[2].toTensor()));
    return outputs;
  }

  void loadModel()
  {
    if (model_loaded_.load()) return;
    std::lock_guard<std::mutex> lock(model_mutex_);
    if (model_loaded_.load()) return;
    torch::set_num_threads(cpu_threads_);
    // Inter-op parallelism >1 mudah meng-oversubscribe Mini-PC ketika OpenCV,
    // ROS executor, GUI, dan LibTorch aktif bersamaan.
    torch::set_num_interop_threads(1);

    const auto verify_contract = [this]() {
      cv::Mat zero_image = cv::Mat::zeros(MODEL_HEIGHT, MODEL_WIDTH, CV_8UC3);
      auto outputs = forwardTorch(zero_image);
      validateOutputs(outputs);
    };

    try {
      auto loaded = torch::jit::load(pt_model_path_, torch::kCPU);
      loaded.eval();
      if (torch_optimize_for_inference_) {
        try {
          auto frozen = torch::jit::freeze(loaded);
          module_ = torch::jit::optimize_for_inference(frozen);
          module_.eval();
          verify_contract();
          torch_optimized_active_ = true;
          RCLCPP_INFO(get_logger(), "TorchScript freeze + optimize_for_inference: PASS");
        } catch (const std::exception & optimize_error) {
          RCLCPP_WARN(
            get_logger(),
            "Torch optimize_for_inference tidak kompatibel dengan checkpoint ini; fallback model asli: %s",
            optimize_error.what());
          module_ = std::move(loaded);
          module_.eval();
          verify_contract();
          torch_optimized_active_ = false;
        }
      } else {
        module_ = std::move(loaded);
        module_.eval();
        verify_contract();
        torch_optimized_active_ = false;
      }
    } catch (const c10::Error & error) {
      throw std::runtime_error(
        std::string("LibTorch gagal membaca YOLOPv2 .pt sebagai TorchScript: ") + error.what());
    }

    model_loaded_.store(true);
    RCLCPP_INFO(
      get_logger(),
      "YOLOPv2 CPU TorchScript lazy-load + warm-up + 8-output contract: PASS (%s)",
      pt_model_path_.c_str());
  }

  void createInterfaces()
  {
    // Citra besar bersifat latest-frame/best-effort agar preview tidak menahan inference.
    const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    // Metrik, state diagnostik, dan command tetap reliable; jangan ikut QoS citra.
    const auto data_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    const auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, image_qos);
    raw_pub_ = create_publisher<sensor_msgs::msg::Image>(raw_topic_, image_qos);
    drivable_pub_ = create_publisher<sensor_msgs::msg::Image>(drivable_topic_, image_qos);
    lane_pub_ = create_publisher<sensor_msgs::msg::Image>(lane_topic_, image_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, image_qos);
    compressed_preview_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(web_preview_topic_, image_qos);
    detections_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(detections_topic_, data_qos);
    performance_pub_ = create_publisher<std_msgs::msg::String>(performance_topic_, data_qos);
    lane_metrics_pub_ = create_publisher<std_msgs::msg::String>(lane_metrics_topic_, data_qos);
    drivable_space_pub_ = create_publisher<std_msgs::msg::String>(drivable_space_topic_, data_qos);
    obstacle_metrics_pub_ = create_publisher<std_msgs::msg::String>(obstacle_metrics_topic_, data_qos);
    near_field_state_pub_ = create_publisher<std_msgs::msg::String>(near_field_state_topic_, data_qos);
    connected_pub_ = create_publisher<std_msgs::msg::Bool>(camera_connected_topic_, stateQos());
    health_pub_ = create_publisher<std_msgs::msg::Bool>(camera_health_topic_, stateQos());
    health_state_pub_ = create_publisher<std_msgs::msg::String>(camera_health_state_topic_, stateQos());
    emergency_pub_ = create_publisher<std_msgs::msg::Bool>(emergency_topic_, stateQos());
    object_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(object_points_topic_, cloud_qos);
    clearing_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(clearing_points_topic_, cloud_qos);
    boundary_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(drivable_boundary_topic_, cloud_qos);
    lane_state_pub_ = create_publisher<std_msgs::msg::String>(lane_state_topic_, stateQos());
    lane_control_state_pub_ = create_publisher<std_msgs::msg::String>(lane_control_state_topic_, stateQos());
    raw_detection_pub_ = create_publisher<std_msgs::msg::String>(raw_detection_topic_, data_qos);
    advisory_pub_ = create_publisher<geometry_msgs::msg::Twist>(safe_cmd_topic_, data_qos);
    rclcpp::SubscriptionOptions nav_options;
    nav_options.callback_group = control_group_;
    nav_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_cmd_topic_, data_qos,
      [this](geometry_msgs::msg::Twist::SharedPtr message) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_nav_cmd_ = *message;
        last_nav_cmd_time_ = now();
        have_nav_cmd_ = true;
      }, nav_options);
  }

  std::vector<std::string> candidateCameraDevices() const
  {
    if (rgb_device_ != "auto") return {rgb_device_};

    std::vector<std::string> candidates;
    std::error_code error;
    const fs::path by_id("/dev/v4l/by-id");
    if (fs::is_directory(by_id, error)) {
      for (const auto & entry : fs::directory_iterator(by_id, error)) {
        if (error) break;
        const std::string name = entry.path().filename().string();
        if (name.find("index0") != std::string::npos || name.find("video") != std::string::npos) {
          candidates.push_back(entry.path().string());
        }
      }
    }
    std::sort(candidates.begin(), candidates.end());

    // USB/V4L2 numbering is not stable across power cycles. Probe all present
    // numeric nodes after stable by-id symlinks instead of hard-coding video0.
    for (int index = 0; index < 64; ++index) {
      const std::string device = "/dev/video" + std::to_string(index);
      if (::access(device.c_str(), R_OK) == 0) candidates.push_back(device);
    }
    std::vector<std::string> unique;
    for (const auto & candidate : candidates) {
      if (std::find(unique.begin(), unique.end(), candidate) == unique.end()) unique.push_back(candidate);
    }
    return unique;
  }

  bool configureCameraCandidate(const std::string & device, cv::VideoCapture & candidate)
  {
    bool opened = false;
    if (!device.empty() && std::all_of(device.begin(), device.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
      })) {
      opened = candidate.open(std::stoi(device), cv::CAP_V4L2);
    } else {
      opened = candidate.open(device, cv::CAP_V4L2);
    }
    if (!opened) return false;

    const int fourcc = pixel_format_ == "YUYV" ? cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V') :
      cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    candidate.set(cv::CAP_PROP_FOURCC, fourcc);
    candidate.set(cv::CAP_PROP_FRAME_WIDTH, requested_width_);
    candidate.set(cv::CAP_PROP_FRAME_HEIGHT, requested_height_);
    candidate.set(cv::CAP_PROP_FPS, camera_fps_);
    candidate.set(cv::CAP_PROP_BUFFERSIZE, 1.0);

    const double actual_width = candidate.get(cv::CAP_PROP_FRAME_WIDTH);
    const double actual_height = candidate.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double actual_fps = candidate.get(cv::CAP_PROP_FPS);
    const double fps_tolerance = std::max(1.0, 0.05 * static_cast<double>(camera_fps_));
    const bool resolution_matches =
      std::abs(actual_width - requested_width_) <= 0.5 &&
      std::abs(actual_height - requested_height_) <= 0.5;
    const bool fps_matches = actual_fps > 0.0 &&
      std::abs(actual_fps - static_cast<double>(camera_fps_)) <= fps_tolerance;
    if (strict_camera_mode_ && (!resolution_matches || !fps_matches)) {
      // CAMERA_MODE_MISMATCH: reject this V4L2 node and continue probing other
      // candidates; do not kill the localization/navigation stack.
      candidate.release();
      return false;
    }

    // A V4L2 node may open successfully even when it is metadata/depth-only.
    // Accept the node only after a real BGR frame can be dequeued.
    cv::Mat probe;
    bool frame_ok = false;
    for (int attempt = 0; attempt < 4; ++attempt) {
      if (candidate.read(probe) && !probe.empty() && probe.cols > 0 && probe.rows > 0 &&
        probe.type() == CV_8UC3)
      {
        frame_ok = true;
        break;
      }
    }
    if (!frame_ok) {
      candidate.release();
      return false;
    }

    if (!resolution_matches || !fps_matches) {
      RCLCPP_WARN(
        get_logger(),
        "Mode kamera CPU fallback: device=%s request=%dx%d@%d actual=%.0fx%.0f@%.2f (strict=false)",
        device.c_str(), requested_width_, requested_height_, camera_fps_,
        actual_width, actual_height, actual_fps);
    }
    return true;
  }

  bool openCamera()
  {
    if (capture_.isOpened()) return true;
    if (!hotplug_retry_ && last_open_attempt_.time_since_epoch().count() != 0) return false;
    const auto t = std::chrono::steady_clock::now();
    if (last_open_attempt_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(t - last_open_attempt_).count() < retry_sec_) return false;
    last_open_attempt_ = t;

    const auto candidates = candidateCameraDevices();
    for (const auto & device : candidates) {
      cv::VideoCapture candidate;
      if (!configureCameraCandidate(device, candidate)) continue;
      capture_ = std::move(candidate);
      camera_read_fail_streak_ = 0;
      camera_good_frame_streak_ = 1;
      // configureCameraCandidate already dequeued one real BGR frame. Delay the
      // ONLINE edge until a few consecutive capture-loop frames also succeed;
      // this prevents USB/V4L2 status flapping on transient open/read glitches.
      const double actual_width = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
      const double actual_height = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
      const double actual_fps = capture_.get(cv::CAP_PROP_FPS);
      RCLCPP_INFO(
        get_logger(), "Kamera CPU terbuka: %s %.0fx%.0f @ %.1f FPS",
        device.c_str(), actual_width, actual_height, actual_fps);
      return true;
    }

    publishConnected(false);
    publishHealth(false, "CAMERA_OPEN_FAILED");
    publishEmergency(inference_enabled_.load());
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Kamera CPU belum tersedia/usable; %zu kandidat V4L2 sudah diprobe, menunggu hot-plug",
      candidates.size());
    return false;
  }

  cv::Mat letterbox(const cv::Mat & frame)
  {
    gain_ = std::min(
      static_cast<float>(MODEL_WIDTH) / static_cast<float>(frame.cols),
      static_cast<float>(MODEL_HEIGHT) / static_cast<float>(frame.rows));
    resized_width_ = std::max(1, static_cast<int>(std::round(frame.cols * gain_)));
    resized_height_ = std::max(1, static_cast<int>(std::round(frame.rows * gain_)));
    pad_x_ = (MODEL_WIDTH - resized_width_) / 2;
    pad_y_ = (MODEL_HEIGHT - resized_height_) / 2;
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(resized_width_, resized_height_), 0.0, 0.0, cv::INTER_LINEAR);
    cv::Mat canvas(MODEL_HEIGHT, MODEL_WIDTH, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(canvas(cv::Rect(pad_x_, pad_y_, resized_width_, resized_height_)));
    return canvas;
  }

  void validateOutputs(const std::vector<cv::Mat> & outputs) const
  {
    if (outputs.size() != 8U) throw std::runtime_error("TorchScript wajib memiliki tepat 8 output YOLOPv2");
    for (const auto & output : outputs) {
      if (output.empty() || output.type() != CV_32F || !output.isContinuous() ||
        !cv::checkRange(output, true, nullptr, -std::numeric_limits<double>::max(),
          std::numeric_limits<double>::max()))
      {
        throw std::runtime_error("Output TorchScript wajib float32 kontinu dan tanpa NaN/Inf");
      }
    }
    for (int head = 0; head < 3; ++head) {
      const cv::Mat & value = outputs[static_cast<size_t>(head)];
      const int stride = 1 << (head + 3);
      if (value.dims != 4 || value.size[0] != 1 || value.size[1] != 255 ||
        value.size[2] != MODEL_HEIGHT / stride || value.size[3] != MODEL_WIDTH / stride ||
        value.total() != static_cast<size_t>(255 * (MODEL_HEIGHT / stride) * (MODEL_WIDTH / stride)))
      {
        throw std::runtime_error("Shape detection head TorchScript tidak sesuai kontrak YOLOPv2");
      }
      const cv::Mat & anchor = outputs[static_cast<size_t>(head + 3)];
      if (anchor.total() != 6U) throw std::runtime_error("Anchor-grid TorchScript harus berisi 6 float/head");
    }
    if (outputs[6].dims != 4 || outputs[6].size[0] != 1 || outputs[6].size[1] != 2 ||
      outputs[6].size[2] != MODEL_HEIGHT || outputs[6].size[3] != MODEL_WIDTH ||
      outputs[7].dims != 4 || outputs[7].size[0] != 1 || outputs[7].size[1] != 1 ||
      outputs[7].size[2] != MODEL_HEIGHT || outputs[7].size[3] != MODEL_WIDTH)
    {
      throw std::runtime_error("Shape segmentation TorchScript tidak sesuai kontrak YOLOPv2");
    }
  }

  std::vector<Detection> decodeDetections(const std::vector<cv::Mat> & outputs, int width, int height)
  {
    std::vector<Detection> candidates;
    candidates.reserve(static_cast<size_t>(std::min(max_candidates_, 20000)));
    for (int head_index = 0; head_index < 3; ++head_index) {
      const cv::Mat & head = outputs[static_cast<size_t>(head_index)];
      const float * values = head.ptr<float>();
      const float * anchors = outputs[static_cast<size_t>(head_index + 3)].ptr<float>();
      const int grid_h = head.size[2];
      const int grid_w = head.size[3];
      const int area = grid_h * grid_w;
      const int attrs = head.size[1] / 3;
      const int classes = attrs - 5;
      const int stride = MODEL_WIDTH / grid_w;
      for (int anchor = 0; anchor < 3 && static_cast<int>(candidates.size()) < max_candidates_; ++anchor) {
        const int base = anchor * attrs * area;
        for (int cell = 0; cell < area && static_cast<int>(candidates.size()) < max_candidates_; ++cell) {
          const float objectness = sigmoid(values[base + 4 * area + cell]);
          if (objectness < confidence_threshold_) continue;
          float best_score = 0.0F;
          int best_class = -1;
          for (int cls = 0; cls < classes; ++cls) {
            const float score = objectness * sigmoid(values[base + (5 + cls) * area + cell]);
            if (score > best_score) {best_score = score; best_class = cls;}
          }
          if (best_score < confidence_threshold_) continue;
          const int gy = cell / grid_w;
          const int gx = cell % grid_w;
          const float tx = sigmoid(values[base + cell]);
          const float ty = sigmoid(values[base + area + cell]);
          const float tw = sigmoid(values[base + 2 * area + cell]);
          const float th = sigmoid(values[base + 3 * area + cell]);
          const float cx = (tx * 2.0F - 0.5F + gx) * stride;
          const float cy = (ty * 2.0F - 0.5F + gy) * stride;
          const float box_w = std::pow(tw * 2.0F, 2.0F) * anchors[anchor * 2];
          const float box_h = std::pow(th * 2.0F, 2.0F) * anchors[anchor * 2 + 1];
          candidates.push_back({
            cx - box_w * 0.5F, cy - box_h * 0.5F,
            cx + box_w * 0.5F, cy + box_h * 0.5F, best_score, best_class});
        }
      }
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
      return a.score > b.score;
    });
    std::vector<Detection> result;
    std::vector<bool> suppressed(candidates.size(), false);
    for (size_t i = 0; i < candidates.size() && static_cast<int>(result.size()) < max_detections_; ++i) {
      if (suppressed[i]) continue;
      Detection d = candidates[i];
      for (size_t j = i + 1; j < candidates.size(); ++j) {
        if (!suppressed[j] && candidates[j].class_id == d.class_id &&
          intersectionOverUnion(d, candidates[j]) > iou_threshold_) suppressed[j] = true;
      }
      d.x1 = std::clamp((d.x1 - pad_x_) / gain_, 0.0F, static_cast<float>(width - 1));
      d.y1 = std::clamp((d.y1 - pad_y_) / gain_, 0.0F, static_cast<float>(height - 1));
      d.x2 = std::clamp((d.x2 - pad_x_) / gain_, 0.0F, static_cast<float>(width - 1));
      d.y2 = std::clamp((d.y2 - pad_y_) / gain_, 0.0F, static_cast<float>(height - 1));
      if (d.x2 > d.x1 && d.y2 > d.y1) result.push_back(d);
    }
    return result;
  }

  void decodeMasks(
    const std::vector<cv::Mat> & outputs, int width, int height,
    cv::Mat & drivable, cv::Mat & lane)
  {
    const size_t plane = static_cast<size_t>(MODEL_WIDTH * MODEL_HEIGHT);
    const float * da = outputs[6].ptr<float>();
    const float * ll = outputs[7].ptr<float>();
    cv::Mat da_small(MODEL_HEIGHT, MODEL_WIDTH, CV_8UC1);
    cv::Mat ll_small(MODEL_HEIGHT, MODEL_WIDTH, CV_8UC1);
    for (size_t i = 0; i < plane; ++i) {
      da_small.data[i] = da[plane + i] > da[i] ? 255U : 0U;
      ll_small.data[i] = ll[i] > lane_threshold_ ? 255U : 0U;
    }
    const cv::Rect valid_roi(pad_x_, pad_y_, resized_width_, resized_height_);
    cv::resize(da_small(valid_roi), drivable, cv::Size(width, height), 0.0, 0.0, cv::INTER_NEAREST);
    cv::resize(ll_small(valid_roi), lane, cv::Size(width, height), 0.0, 0.0, cv::INTER_NEAREST);
  }

  void prepareHomography(int width, int height)
  {
    if (!homography_.empty() && homography_width_ == width && homography_height_ == height) return;
    std::vector<cv::Point2f> source(4), destination(4);
    const float sx = static_cast<float>(width) / ground_calibration_width_;
    const float sy = static_cast<float>(height) / ground_calibration_height_;
    for (size_t i = 0; i < 4U; ++i) {
      source[i] = cv::Point2f(
        static_cast<float>(ground_src_[i * 2] * sx),
        static_cast<float>(ground_src_[i * 2 + 1] * sy));
      destination[i] = cv::Point2f(
        static_cast<float>(ground_dst_[i * 2]), static_cast<float>(ground_dst_[i * 2 + 1]));
    }
    homography_ = cv::getPerspectiveTransform(source, destination);
    if (homography_.empty()) throw std::runtime_error("Homography CPU tidak dapat dihitung");
    homography_width_ = width;
    homography_height_ = height;
  }

  bool projectPixel(float x, float y, float & forward, float & left) const
  {
    if (homography_.empty()) return false;
    std::vector<cv::Point2f> input{{x, y}};
    std::vector<cv::Point2f> output;
    cv::perspectiveTransform(input, output, homography_);
    if (output.size() != 1U || !std::isfinite(output[0].x) || !std::isfinite(output[0].y) ||
      output[0].x < 0.0F || output[0].x >= ground_canvas_width_ ||
      output[0].y < 0.0F || output[0].y >= ground_canvas_height_) return false;
    forward = static_cast<float>((ground_origin_y_ - output[0].y) * ground_scale_y_ + forward_offset_);
    left = static_cast<float>((ground_origin_x_ - output[0].x) * ground_scale_x_ + lateral_offset_);
    return std::isfinite(forward) && std::isfinite(left);
  }

  float calibratedForwardDistance(int class_id, float raw_forward_m) const
  {
    if (!std::isfinite(raw_forward_m) || obstacle_distance_calibration_coefficients_.size() != 4U) {
      return raw_forward_m;
    }
    double scale = 1.0;
    double bias = 0.0;
    if (class_id == 0) {
      scale = obstacle_distance_calibration_coefficients_[0];
      bias = obstacle_distance_calibration_coefficients_[1];
    } else if (class_id == 3) {
      scale = obstacle_distance_calibration_coefficients_[2];
      bias = obstacle_distance_calibration_coefficients_[3];
    }
    const double corrected = scale * static_cast<double>(raw_forward_m) + bias;
    return std::isfinite(corrected) ? static_cast<float>(corrected) : raw_forward_m;
  }

  bool drivableContact(const Detection & d, const cv::Mat & drivable) const
  {
    if (!require_drivable_contact_) return true;
    if (drivable.empty()) return false;
    const int bottom = std::clamp(static_cast<int>(std::lround(d.y2)), 0, drivable.rows - 1);
    const int tol = std::max(0, drivable_contact_vertical_tolerance_px_);
    const float box_width = std::max(1.0F, d.x2 - d.x1);
    const float inset = std::min(static_cast<float>(drivable_contact_radius_px_), 0.45F * box_width);
    const float left = d.x1 + inset;
    const float right = d.x2 - inset;
    int road = 0;
    int total = 0;
    const std::array<int, 5> ys{bottom - tol, bottom - tol / 2, bottom, bottom + tol / 2, bottom + tol};
    for (const int raw_y : ys) {
      const int y = std::clamp(raw_y, 0, drivable.rows - 1);
      for (int sample = 0; sample < 5; ++sample) {
        const float alpha = sample * 0.25F;
        const int x = std::clamp(static_cast<int>(std::lround(left + alpha * (right - left))), 0, drivable.cols - 1);
        ++total;
        if (drivable.at<uint8_t>(y, x) > 0U) ++road;
      }
    }
    return road >= drivable_contact_min_samples_ &&
      static_cast<double>(road) / std::max(1, total) >= drivable_contact_min_fraction_;
  }

  bool isSafetyClass(int class_id) const
  {
    return accept_all_obstacles_ || std::find(safety_classes_.begin(), safety_classes_.end(), class_id) != safety_classes_.end();
  }

  sensor_msgs::msg::PointCloud2 makeCloud(
    const rclcpp::Time & stamp, const std::vector<std::array<float, 4>> & points) const
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = metric_frame_id_;
    message.height = 1U;
    message.width = static_cast<uint32_t>(points.size());
    message.is_bigendian = false;
    message.is_dense = true;
    message.point_step = 16U;
    message.row_step = message.point_step * message.width;
    message.fields.resize(4U);
    const char * names[] = {"x", "y", "z", "intensity"};
    for (size_t i = 0; i < 4U; ++i) {
      message.fields[i].name = names[i];
      message.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
      message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[i].count = 1U;
    }
    message.data.resize(points.size() * 4U * sizeof(float));
    if (!points.empty()) std::memcpy(message.data.data(), points.data(), message.data.size());
    return message;
  }

  std::vector<std::array<float, 4>> projectObstacles(
    const std::vector<Detection> & detections, const cv::Mat & drivable,
    bool & emergency, std::vector<std::pair<float, float>> & centers,
    std::vector<MetricObstacle> & metric_obstacles)
  {
    std::vector<std::array<float, 4>> points;
    emergency = false;
    centers.clear();
    metric_obstacles.clear();
    for (const auto & d : detections) {
      if (!isSafetyClass(d.class_id) || d.score < minimum_obstacle_confidence_ ||
        !drivableContact(d, drivable))
      {
        continue;
      }
      const float center_x = 0.5F * (d.x1 + d.x2);
      float forward = 0.0F;
      float left = 0.0F;
      if (!projectPixel(center_x, d.y2, forward, left))
      {
        continue;
      }
      forward = calibratedForwardDistance(d.class_id, forward);
      if (forward < obstacle_forward_min_m_ || forward > obstacle_forward_max_m_ || std::abs(left) > max_abs_left_)
      {
        continue;
      }
      float left_edge_forward = 0.0F;
      float left_edge = 0.0F;
      float right_edge_forward = 0.0F;
      float right_edge = 0.0F;
      float width = 0.35F;
      if (projectPixel(d.x1, d.y2, left_edge_forward, left_edge) &&
        projectPixel(d.x2, d.y2, right_edge_forward, right_edge))
      {
        width = std::clamp(std::abs(left_edge - right_edge), 0.20F, 2.5F);
      }
      if (width < minimum_obstacle_width_m_) continue;
      centers.emplace_back(forward, left);
      MetricObstacle obstacle;
      obstacle.class_id = d.class_id;
      obstacle.score = d.score;
      obstacle.forward_m = forward;
      obstacle.left_m = left;
      obstacle.width_m = width;
      metric_obstacles.push_back(obstacle);
      if (forward <= emergency_distance_m_ &&
        std::abs(left) <= emergency_half_width_m_ + 0.5F * width)
      {
        emergency = true;
      }
      const float half = 0.5F * width;
      for (int i = 0; i < points_per_box_; ++i) {
        const float ratio = points_per_box_ <= 1 ? 0.5F :
          static_cast<float>(i) / static_cast<float>(points_per_box_ - 1);
        points.push_back({forward, left - half + ratio * 2.0F * half, 0.20F, d.score});
      }
    }
    return points;
  }

  std::vector<MetricObstacle> updateObstacleTracks(const std::vector<MetricObstacle> & detections)
  {
    std::vector<bool> used(obstacle_tracks_.size(), false);
    std::vector<MetricObstacle> confirmed;
    confirmed.reserve(detections.size());
    for (const auto & detection : detections) {
      int best = -1;
      double best_distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < obstacle_tracks_.size(); ++i) {
        if (used[i] || obstacle_tracks_[i].class_id != detection.class_id) continue;
        const double distance = std::hypot(
          obstacle_tracks_[i].forward_m - detection.forward_m,
          obstacle_tracks_[i].left_m - detection.left_m);
        if (distance < track_match_distance_m_ && distance < best_distance) {best = static_cast<int>(i); best_distance = distance;}
      }
      if (best < 0) {
        obstacle_tracks_.push_back(ObstacleTrack{
          next_track_id_++, detection.class_id, detection.score, detection.forward_m, detection.left_m,
          detection.width_m, 1, 0, track_confirm_hits_ <= 1});
        used.push_back(true);
        best = static_cast<int>(obstacle_tracks_.size() - 1U);
      } else {
        auto & track = obstacle_tracks_[static_cast<size_t>(best)];
        const float a = static_cast<float>(track_ema_alpha_);
        track.forward_m = a * detection.forward_m + (1.0F - a) * track.forward_m;
        track.left_m = a * detection.left_m + (1.0F - a) * track.left_m;
        track.width_m = a * detection.width_m + (1.0F - a) * track.width_m;
        track.score = a * detection.score + (1.0F - a) * track.score;
        track.hits = std::min(track.hits + 1, 1000000);
        track.missed = 0;
        track.confirmed = track.confirmed || track.hits >= track_confirm_hits_;
        used[static_cast<size_t>(best)] = true;
      }
      const auto & track = obstacle_tracks_[static_cast<size_t>(best)];
      if (track.confirmed) {
        MetricObstacle out = detection;
        out.track_id = track.track_id; out.score = track.score; out.forward_m = track.forward_m;
        out.left_m = track.left_m; out.width_m = track.width_m; out.hits = track.hits; out.confirmed = true;
        confirmed.push_back(out);
      }
    }
    for (size_t i = 0; i < obstacle_tracks_.size(); ++i) if (i >= used.size() || !used[i]) ++obstacle_tracks_[i].missed;
    obstacle_tracks_.erase(std::remove_if(obstacle_tracks_.begin(), obstacle_tracks_.end(), [this](const auto & t) {return t.missed > track_max_missed_frames_;}), obstacle_tracks_.end());
    // Missed tracks stay only in the association cache. They are never republished as active safety obstacles on slow CPU.
    return confirmed;
  }

  sensor_msgs::msg::PointCloud2 makeObstacleCloud(
    const rclcpp::Time & stamp, const std::vector<MetricObstacle> & obstacles) const
  {
    sensor_msgs::msg::PointCloud2 message;
    message.header.stamp = stamp;
    message.header.frame_id = metric_frame_id_;
    message.height = 1U;
    message.is_bigendian = false;
    message.is_dense = true;
    message.point_step = 20U;
    message.fields.resize(5U);
    const char *names[] = {"x", "y", "z", "intensity", "track_id"};
    for (size_t i = 0; i < 5U; ++i) {
      message.fields[i].name = names[i];
      message.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
      message.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[i].count = 1U;
    }
    const size_t count = obstacles.size() * static_cast<size_t>(points_per_box_);
    message.width = static_cast<uint32_t>(count);
    message.row_step = message.point_step * message.width;
    message.data.resize(count * message.point_step);
    size_t out_index = 0U;
    for (const auto & obstacle : obstacles) {
      const float half = 0.5F * obstacle.width_m;
      for (int i = 0; i < points_per_box_; ++i) {
        const float ratio = points_per_box_ <= 1 ? 0.5F :
          static_cast<float>(i) / static_cast<float>(points_per_box_ - 1);
        const std::array<float, 5> point{
          obstacle.forward_m, obstacle.left_m - half + ratio * 2.0F * half,
          0.20F, obstacle.score, static_cast<float>(obstacle.track_id)};
        std::memcpy(message.data.data() + out_index * message.point_step, point.data(), message.point_step);
        ++out_index;
      }
    }
    return message;
  }

  std::vector<std::array<float, 4>> obstaclePoints(const std::vector<MetricObstacle> & obstacles) const
  {
    std::vector<std::array<float, 4>> points;
    for (const auto & obstacle : obstacles) {
      const float half = 0.5F * obstacle.width_m;
      for (int i = 0; i < points_per_box_; ++i) {
        const float ratio = points_per_box_ <= 1 ? 0.5F : static_cast<float>(i) / static_cast<float>(points_per_box_ - 1);
        points.push_back({obstacle.forward_m, obstacle.left_m - half + ratio * 2.0F * half, 0.20F, obstacle.score});
      }
    }
    return points;
  }

  std::vector<std::array<float, 4>> buildDrivableBoundary(
    const cv::Mat & drivable, bool & valid, double & left_clearance,
    double & right_clearance, double & center_error, double & heading_error,
    double & road_width, int & valid_rows)
  {
    std::vector<std::array<float, 4>> points;
    std::vector<double> left_values;
    std::vector<double> right_values;
    std::vector<double> width_values;
    std::vector<std::pair<double, double>> center_samples;
    valid_rows = 0;
    const int start_y = std::max(0, static_cast<int>(drivable.rows * 0.42));
    const int step = std::max(4, drivable.rows / 48);
    for (int y = drivable.rows - 2; y >= start_y; y -= step) {
      int left_pixel = -1;
      int right_pixel = -1;
      const auto * row = drivable.ptr<uint8_t>(y);
      for (int x = 0; x < drivable.cols; ++x) {
        if (row[x] > 0U) {left_pixel = x; break;}
      }
      for (int x = drivable.cols - 1; x >= 0; --x) {
        if (row[x] > 0U) {right_pixel = x; break;}
      }
      if (left_pixel < 0 || right_pixel <= left_pixel) continue;
      float lf = 0.0F, ll = 0.0F, rf = 0.0F, rl = 0.0F;
      if (projectPixel(static_cast<float>(left_pixel), static_cast<float>(y), lf, ll) &&
        projectPixel(static_cast<float>(right_pixel), static_cast<float>(y), rf, rl))
      {
        if (lf >= min_forward_ && lf <= max_forward_ && std::abs(ll) <= max_abs_left_) {
          points.push_back({lf, ll, 0.20F, 1.0F});
        }
        if (rf >= min_forward_ && rf <= max_forward_ && std::abs(rl) <= max_abs_left_) {
          points.push_back({rf, rl, 0.20F, 1.0F});
        }
        const double average_forward = 0.5 * (static_cast<double>(lf) + static_cast<double>(rf));
        if (average_forward >= 0.8 && average_forward <= 2.5 && ll > rl) {
          left_values.push_back(ll);
          right_values.push_back(rl);
          width_values.push_back(static_cast<double>(ll - rl));
          center_samples.emplace_back(average_forward, 0.5 * static_cast<double>(ll + rl));
          ++valid_rows;
        }
      }
    }
    valid = left_values.size() >= 4U && right_values.size() >= 4U;
    left_clearance = right_clearance = center_error = heading_error = road_width = 0.0;
    if (valid) {
      const auto mean = [](const std::vector<double> & values) {
        return std::accumulate(values.begin(), values.end(), 0.0) /
          static_cast<double>(values.size());
      };
      const double left = mean(left_values);
      const double right = mean(right_values);
      road_width = mean(width_values);
      left_clearance = left - 0.5 * lane_vehicle_width_m_;
      right_clearance = -right - 0.5 * lane_vehicle_width_m_;
      center_error = 0.5 * (left + right);
      if (center_samples.size() >= 3U) {
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (const auto & sample : center_samples) {
          mean_x += sample.first;
          mean_y += sample.second;
        }
        mean_x /= static_cast<double>(center_samples.size());
        mean_y /= static_cast<double>(center_samples.size());
        double covariance = 0.0;
        double variance = 0.0;
        for (const auto & sample : center_samples) {
          const double dx = sample.first - mean_x;
          covariance += dx * (sample.second - mean_y);
          variance += dx * dx;
        }
        if (variance > 1.0e-9) heading_error = std::atan(covariance / variance);
      }
      valid = left_clearance >= 0.0 && right_clearance >= 0.0 &&
        std::isfinite(heading_error) && road_width > lane_vehicle_width_m_;
    }
    return points;
  }

  void publishClearingFan(
    const rclcpp::Time & stamp, const std::vector<std::pair<float, float>> & obstacles)
  {
    std::vector<std::array<float, 4>> points;
    for (int i = 0; i < clearing_ray_count_; ++i) {
      const double ratio = static_cast<double>(i) / std::max(1, clearing_ray_count_ - 1);
      const double angle = -0.5 * clearing_fov_rad_ + ratio * clearing_fov_rad_;
      bool blocked = false;
      for (const auto & obstacle : obstacles) {
        const double distance = std::hypot(obstacle.first, obstacle.second);
        const double obstacle_angle = std::atan2(obstacle.second, obstacle.first);
        const double half_angle = std::atan2(clearing_obstacle_margin_m_ + 0.25, std::max(0.05, distance));
        const double error = std::atan2(std::sin(angle - obstacle_angle), std::cos(angle - obstacle_angle));
        if (std::abs(error) <= half_angle) {blocked = true; break;}
      }
      if (!blocked) {
        points.push_back({
          static_cast<float>(clearing_range_m_ * std::cos(angle)),
          static_cast<float>(clearing_range_m_ * std::sin(angle)), 0.20F, 0.0F});
      }
    }
    clearing_pub_->publish(makeCloud(stamp, points));
  }

  bool cameraHealthy(
    const cv::Mat & frame, double & mean_luma, double & stddev,
    double & extreme_fraction, double & mean_gradient) const
  {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::Scalar mean_value;
    cv::Scalar std_value;
    cv::meanStdDev(gray, mean_value, std_value);
    mean_luma = mean_value[0];
    stddev = std_value[0];
    int extreme = 0;
    int total = 0;
    double gradient_sum = 0.0;
    int gradient_count = 0;
    const int stride = std::max(4, std::min(frame.cols, frame.rows) / 80);
    for (int y = 0; y < gray.rows; y += stride) {
      const auto * row = gray.ptr<uint8_t>(y);
      for (int x = 0; x < gray.cols; x += stride) {
        ++total;
        if (row[x] <= health_dark_ || row[x] >= health_bright_) ++extreme;
        if (x + stride < gray.cols) {
          gradient_sum += std::abs(static_cast<int>(row[x + stride]) - static_cast<int>(row[x]));
          ++gradient_count;
        }
        if (y + stride < gray.rows) {
          const auto * next = gray.ptr<uint8_t>(y + stride);
          gradient_sum += std::abs(static_cast<int>(next[x]) - static_cast<int>(row[x]));
          ++gradient_count;
        }
      }
    }
    extreme_fraction = total > 0 ? static_cast<double>(extreme) / total : 1.0;
    mean_gradient = gradient_count > 0 ? gradient_sum / gradient_count : 0.0;
    return stddev >= health_min_stddev_ && mean_gradient >= health_min_gradient_ &&
      extreme_fraction < health_extreme_fraction_;
  }

  void publishImage(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & publisher,
    const rclcpp::Time & stamp, const cv::Mat & image, const std::string & encoding)
  {
    if (!publisher || publisher->get_subscription_count() == 0U) return;
    cv::Mat source = image.isContinuous() ? image : image.clone();
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    message->header.stamp = stamp;
    message->header.frame_id = frame_id_;
    message->height = static_cast<uint32_t>(source.rows);
    message->width = static_cast<uint32_t>(source.cols);
    message->encoding = encoding;
    message->is_bigendian = false;
    message->step = static_cast<uint32_t>(source.cols * source.elemSize());
    message->data.assign(source.data, source.data + source.total() * source.elemSize());
    publisher->publish(std::move(message));
  }

  void publishCameraInfo(const rclcpp::Time & stamp, int width, int height)
  {
    sensor_msgs::msg::CameraInfo message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.width = static_cast<uint32_t>(width);
    message.height = static_cast<uint32_t>(height);
    message.distortion_model = "plumb_bob";
    message.d = camera_distortion_;
    message.k = {camera_fx_, 0.0, camera_cx_, 0.0, camera_fy_, camera_cy_, 0.0, 0.0, 1.0};
    message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    message.p = {camera_fx_, 0.0, camera_cx_, 0.0, 0.0, camera_fy_, camera_cy_, 0.0, 0.0, 0.0, 1.0, 0.0};
    camera_info_pub_->publish(std::move(message));
  }

  void publishDetectionMessages(
    const rclcpp::Time & stamp, const std::vector<Detection> & detections, int image_width, int image_height)
  {
    if (publish_detections_ && detections_pub_->get_subscription_count() > 0U) {
      vision_msgs::msg::Detection2DArray array;
      array.header.stamp = stamp;
      array.header.frame_id = frame_id_;
      for (const auto & source : detections) {
        vision_msgs::msg::Detection2D detection;
        detection.header = array.header;
        detection.bbox.center.position.x = 0.5 * (source.x1 + source.x2);
        detection.bbox.center.position.y = 0.5 * (source.y1 + source.y2);
        detection.bbox.center.theta = 0.0;
        detection.bbox.size_x = source.x2 - source.x1;
        detection.bbox.size_y = source.y2 - source.y1;
        vision_msgs::msg::ObjectHypothesisWithPose result;
        result.hypothesis.class_id = std::to_string(source.class_id);
        result.hypothesis.score = source.score;
        detection.results.push_back(std::move(result));
        array.detections.push_back(std::move(detection));
      }
      detections_pub_->publish(std::move(array));
    }
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(5)
            << "{\"backend\":\"cpu\",\"count\":" << detections.size()
            << ",\"image_width\":" << image_width
            << ",\"image_height\":" << image_height
            << ",\"detections\":[";
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto &d = detections[i];
      const float u = 0.5F * (d.x1 + d.x2);
      const float v = d.y2;
      float raw_forward = 0.0F;
      float raw_left = 0.0F;
      const bool metric_ok = projectPixel(u, v, raw_forward, raw_left);
      const double hfrac = image_height > 0 ?
        std::max(0.0, static_cast<double>(d.y2 - d.y1)) / static_cast<double>(image_height) : 0.0;
      const double cfrac = image_width > 0 ?
        0.5 * static_cast<double>(d.x1 + d.x2) / static_cast<double>(image_width) : 0.5;
      const bool clipped = image_height > 0 && d.y2 >= static_cast<float>(image_height - 2);
      if (i) summary << ',';
      summary << "{\"class_id\":" << d.class_id
              << ",\"score\":" << d.score
              << ",\"x1\":" << d.x1 << ",\"y1\":" << d.y1
              << ",\"x2\":" << d.x2 << ",\"y2\":" << d.y2
              << ",\"bottom_u_px\":" << u << ",\"bottom_v_px\":" << v
              << ",\"bbox_height_fraction\":" << hfrac
              << ",\"center_x_fraction\":" << cfrac
              << ",\"bottom_clipped\":" << (clipped ? "true" : "false");
      if (metric_ok) {
        summary << ",\"raw_forward_m\":" << raw_forward
                << ",\"raw_left_m\":" << raw_left
                << ",\"calibrated_forward_m\":"
                << calibratedForwardDistance(d.class_id, raw_forward);
      } else {
        summary << ",\"raw_forward_m\":null,\"raw_left_m\":null,\"calibrated_forward_m\":null";
      }
      summary << '}';
    }
    summary << "]}";
    std_msgs::msg::String message;
    message.data = summary.str();
    raw_detection_pub_->publish(std::move(message));
  }

  double bottomDrivableFraction(const cv::Mat & drivable) const
  {
    if (drivable.empty()) return 0.0;
    const int y0 = std::clamp(
      static_cast<int>(std::lround(drivable.rows * (1.0 - near_field_bottom_roi_fraction_))),
      0, drivable.rows - 1);
    const cv::Mat roi = drivable.rowRange(y0, drivable.rows);
    return static_cast<double>(cv::countNonZero(roi)) /
      static_cast<double>(std::max<size_t>(1U, roi.total()));
  }

  NearFieldDecision evaluateNearField(
    const std::vector<Detection> & detections, const cv::Mat & drivable, int width, int height)
  {
    NearFieldDecision decision;
    decision.drivable_fraction = bottomDrivableFraction(drivable);
    if (!near_field_emergency_enabled_ || width <= 0 || height <= 0) {
      near_field_candidate_hits_ = 0;
      near_field_release_hits_ = 0;
      near_field_latched_ = false;
      decision.reason = near_field_emergency_enabled_ ? "INVALID_IMAGE" : "DISABLED";
      return decision;
    }

    const double half_corridor = 0.5 * near_field_center_corridor_fraction_;
    const double min_center = 0.5 - half_corridor;
    const double max_center = 0.5 + half_corridor;
    const double bottom_roi_start = 1.0 - near_field_bottom_roi_fraction_;
    const Detection *best = nullptr;
    double best_height = 0.0;
    bool best_contact = false;
    for (const auto &d : detections) {
      if (!isSafetyClass(d.class_id) || d.score < near_field_min_confidence_) continue;
      const double center = 0.5 * static_cast<double>(d.x1 + d.x2) / static_cast<double>(width);
      const double height_fraction = std::max(0.0, static_cast<double>(d.y2 - d.y1)) / static_cast<double>(height);
      const double bottom_fraction = std::clamp(static_cast<double>(d.y2) / static_cast<double>(height), 0.0, 1.0);
      if (center < min_center || center > max_center ||
          height_fraction < near_field_min_bbox_height_fraction_ ||
          bottom_fraction < bottom_roi_start) continue;
      const bool contact = drivableContact(d, drivable);
      if (near_field_drivable_guard_enabled_ &&
          (!contact || decision.drivable_fraction < near_field_min_drivable_fraction_)) continue;
      if (!best || height_fraction > best_height ||
          (std::abs(height_fraction - best_height) < 1.0e-6 && d.score > best->score)) {
        best = &d;
        best_height = height_fraction;
        best_contact = contact;
      }
    }

    decision.raw_candidate = best != nullptr;
    if (best) {
      decision.class_id = best->class_id;
      decision.confidence = best->score;
      decision.bbox_height_fraction = best_height;
      decision.center_x_fraction = 0.5 * static_cast<double>(best->x1 + best->x2) / static_cast<double>(width);
      decision.drivable_contact = best_contact;
      last_near_field_class_id_ = decision.class_id;
      last_near_field_confidence_ = decision.confidence;
      last_near_field_bbox_height_fraction_ = decision.bbox_height_fraction;
      last_near_field_center_x_fraction_ = decision.center_x_fraction;
      last_near_field_drivable_contact_ = decision.drivable_contact;
      ++near_field_candidate_hits_;
      near_field_release_hits_ = 0;
      if (near_field_candidate_hits_ >= near_field_confirm_frames_) near_field_latched_ = true;
    } else {
      near_field_candidate_hits_ = 0;
      if (near_field_latched_) {
        ++near_field_release_hits_;
        if (near_field_release_hits_ >= near_field_release_frames_) {
          near_field_latched_ = false;
          near_field_release_hits_ = 0;
        }
      } else {
        near_field_release_hits_ = 0;
      }
    }

    decision.emergency = near_field_latched_;
    if (decision.emergency && !best) {
      decision.class_id = last_near_field_class_id_;
      decision.confidence = last_near_field_confidence_;
      decision.bbox_height_fraction = last_near_field_bbox_height_fraction_;
      decision.center_x_fraction = last_near_field_center_x_fraction_;
      decision.drivable_contact = last_near_field_drivable_contact_;
    }
    decision.reason = decision.emergency ? "IMAGE_NEAR_FIELD_CONFIRMED" :
      (decision.raw_candidate ? "IMAGE_NEAR_FIELD_CONFIRMING" : "CLEAR");
    return decision;
  }

  void publishObstacleMetrics(
    const rclcpp::Time & stamp, const std::vector<MetricObstacle> & obstacles)
  {
    if (!obstacle_metrics_pub_) return;
    std::ostringstream json;
    json << std::fixed << std::setprecision(5)
         << "{\"backend\":\"cpu\",\"stamp_ns\":" << stamp.nanoseconds()
         << ",\"frame_id\":\"" << metric_frame_id_ << "\",\"mode\":\"cpu_homography_ready\""
         << ",\"count\":" << obstacles.size() << ",\"detections\":[";
    for (size_t i = 0; i < obstacles.size(); ++i) {
      if (i) json << ',';
      const auto & obstacle = obstacles[i];
      json << "{\"track_id\":" << obstacle.track_id
           << ",\"class_id\":" << obstacle.class_id
           << ",\"score\":" << obstacle.score
           << ",\"hits\":" << obstacle.hits
           << ",\"confirmed\":" << (obstacle.confirmed ? "true" : "false")
           << ",\"forward_homography_m\":" << obstacle.forward_m
           << ",\"forward_area_m\":null"
           << ",\"forward_m\":" << obstacle.forward_m
           << ",\"left_m\":" << obstacle.left_m
           << ",\"metric_width_m\":" << obstacle.width_m << '}';
    }
    json << "]}";
    std_msgs::msg::String message;
    message.data = json.str();
    obstacle_metrics_pub_->publish(std::move(message));
  }

  void publishNearFieldState(const NearFieldDecision & near, bool metric_emergency)
  {
    const bool emergency = near.emergency || metric_emergency;
    const std::string reason = near.emergency ? near.reason :
      (metric_emergency ? "METRIC_NEAR_FIELD" : near.reason);
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(4)
         << "{\"backend\":\"cpu\",\"emergency\":" << emergency
         << ",\"image_emergency\":" << near.emergency
         << ",\"metric_emergency\":" << metric_emergency
         << ",\"raw_candidate\":" << near.raw_candidate
         << ",\"reason\":\"" << reason << "\""
         << ",\"raw_class_id\":" << near.class_id
         << ",\"confidence\":" << near.confidence
         << ",\"bbox_height_fraction\":" << near.bbox_height_fraction
         << ",\"center_x_fraction\":" << near.center_x_fraction
         << ",\"drivable_contact\":" << near.drivable_contact
         << ",\"near_field_drivable_fraction\":" << near.drivable_fraction
         << ",\"confirm_hits\":" << near_field_candidate_hits_
         << ",\"release_hits\":" << near_field_release_hits_ << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    near_field_state_pub_->publish(std::move(message));
  }

  void publishDrivableSpace(
    const rclcpp::Time & stamp, bool valid, int valid_rows, size_t boundary_points)
  {
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(4)
         << "{\"backend\":\"cpu\",\"stamp_ns\":" << stamp.nanoseconds()
         << ",\"frame_id\":\"" << metric_frame_id_ << "\""
         << ",\"valid_rows\":" << valid_rows
         << ",\"sample_rows\":48"
         << ",\"boundary_points\":" << boundary_points
         << ",\"valid\":" << valid << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    drivable_space_pub_->publish(std::move(message));
  }

  static double robustClosestGap(std::vector<double> gaps)
  {
    if (gaps.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::sort(gaps.begin(), gaps.end());
    // Rata-rata kuartil terdekat: sensitif terhadap intrusi nyata tetapi tidak
    // bereaksi pada satu piksel noise terisolasi.
    const size_t count = std::max<size_t>(1U, gaps.size() / 4U);
    return std::accumulate(gaps.begin(), gaps.begin() + static_cast<std::ptrdiff_t>(count), 0.0) /
      static_cast<double>(count);
  }

  std::string laneCorridorStatus(bool valid, double gap_px) const
  {
    if (!valid || !std::isfinite(gap_px)) return "UNKNOWN";
    if (gap_px <= lane_corridor_touch_margin_px_) return "RED";
    if (gap_px <= lane_corridor_warning_gap_px_) return "YELLOW";
    return "GREEN";
  }

  struct LaneSafetyLineAtRow
  {
    double center_x{0.0};
    double left_x{0.0};
    double right_x{0.0};
  };

  LaneSafetyLineAtRow laneSafetyLineAtRow(int y, int image_width, int image_height) const
  {
    const double sx = static_cast<double>(image_width) / static_cast<double>(std::max(1, requested_width_));
    const double sy = static_cast<double>(image_height) / static_cast<double>(std::max(1, requested_height_));
    const double fx = camera_fx_ * sx;
    const double fy = camera_fy_ * sy;
    const double cx = camera_cx_ * sx + lane_corridor_center_offset_px_ * sx;
    const double cy = camera_cy_ * sy;
    const double pitch = lane_corridor_camera_pitch_deg_ * 3.14159265358979323846 / 180.0;
    const double sin_pitch = std::sin(pitch);
    const double cos_pitch = std::cos(pitch);
    const double q = fy > 1.0e-9 ? (static_cast<double>(y) - cy) / fy : 0.0;
    const double denominator = q * cos_pitch + sin_pitch;

    // Pada horizon, proyeksi ground menuju tak hingga. Untuk safety overlay operator,
    // jangan biarkan half-width menjadi nol (yang membentuk segitiga). Gunakan
    // far-lookahead fisik sebagai batas maksimum depth; titik bawah tetap mengikuti
    // tinggi/pitch kamera dan intrinsic/FOV bila ground projection valid.
    double optical_depth_m = lane_corridor_far_lookahead_m_;
    if (denominator > 1.0e-6) {
      const double forward_m = lane_corridor_camera_height_m_ *
        (cos_pitch - q * sin_pitch) / denominator;
      const double projected_depth_m = cos_pitch * forward_m +
        lane_corridor_camera_height_m_ * sin_pitch;
      if (forward_m >= 0.0 && projected_depth_m > 1.0e-6) {
        optical_depth_m = std::min(projected_depth_m, lane_corridor_far_lookahead_m_);
      }
    }
    optical_depth_m = std::max(0.25, optical_depth_m);
    const double half_width_m = 0.5 * lane_vehicle_width_m_ + lane_corridor_safety_margin_m_;
    double half_width_px = fx * half_width_m / optical_depth_m;
    half_width_px = std::clamp(half_width_px, 1.0, 0.49 * static_cast<double>(image_width));

    LaneSafetyLineAtRow line;
    line.center_x = cx;
    line.left_x = cx - half_width_px + lane_corridor_left_offset_px_ * sx;
    line.right_x = cx + half_width_px + lane_corridor_right_offset_px_ * sx;
    line.left_x = std::clamp(line.left_x, 0.0, static_cast<double>(image_width - 1));
    line.right_x = std::clamp(line.right_x, 0.0, static_cast<double>(image_width - 1));
    if (line.left_x > line.right_x) std::swap(line.left_x, line.right_x);
    return line;
  }

  LaneCorridorDecision evaluateLaneCorridor(const cv::Mat & lane, const cv::Mat & drivable) const
  {
    LaneCorridorDecision result;
    result.enabled = lane_corridor_overlay_enabled_ || lane_corridor_control_enabled_;
    if (!result.enabled || lane.empty() || drivable.empty() ||
      lane.type() != CV_8UC1 || drivable.type() != CV_8UC1 ||
      lane.size() != drivable.size() || lane.cols < 20 || lane.rows < 20)
    {
      return result;
    }
    const int top_y = std::clamp(
      static_cast<int>(std::lround(lane.rows * lane_corridor_top_y_ratio_)), 0, lane.rows - 2);
    const int bottom_y = std::clamp(
      static_cast<int>(std::lround(lane.rows * lane_corridor_bottom_y_ratio_)), top_y + 1, lane.rows - 1);
    std::vector<double> left_gaps;
    std::vector<double> right_gaps;
    left_gaps.reserve(static_cast<size_t>((bottom_y - top_y) / lane_corridor_sample_stride_px_ + 2));
    right_gaps.reserve(left_gaps.capacity());
    uint64_t drivable_pixels = 0U;
    uint64_t corridor_pixels = 0U;

    const auto top_line = laneSafetyLineAtRow(top_y, lane.cols, lane.rows);
    const auto bottom_line = laneSafetyLineAtRow(bottom_y, lane.cols, lane.rows);
    for (int y = top_y; y <= bottom_y; y += lane_corridor_sample_stride_px_) {
      const double t = static_cast<double>(y - top_y) /
        static_cast<double>(std::max(1, bottom_y - top_y));
      const double center_x = top_line.center_x + t * (bottom_line.center_x - top_line.center_x);
      const double left_line = top_line.left_x + t * (bottom_line.left_x - top_line.left_x);
      const double right_line = top_line.right_x + t * (bottom_line.right_x - top_line.right_x);
      const auto * lane_row = lane.ptr<uint8_t>(y);
      const auto * drivable_row = drivable.ptr<uint8_t>(y);

      const int corridor_left = std::clamp(static_cast<int>(std::ceil(left_line)), 0, lane.cols - 1);
      const int corridor_right = std::clamp(static_cast<int>(std::floor(right_line)), 0, lane.cols - 1);
      int row_total = 0;
      int row_drivable = 0;
      for (int x = corridor_left; x <= corridor_right; x += 4) {
        ++row_total;
        if (drivable_row[x] > 0U) ++row_drivable;
      }
      corridor_pixels += static_cast<uint64_t>(row_total);
      drivable_pixels += static_cast<uint64_t>(row_drivable);
      const double row_fraction = row_total > 0 ?
        static_cast<double>(row_drivable) / static_cast<double>(row_total) : 0.0;
      if (row_fraction >= lane_corridor_minimum_drivable_fraction_) ++result.drivable_rows;

      int left_lane = -1;
      int right_lane = -1;
      // Ambil lane pixel paling dalam pada masing-masing sisi kendaraan.
      for (int x = 0; x <= static_cast<int>(center_x); ++x) {
        if (lane_row[x] > 0U) left_lane = x;
      }
      for (int x = static_cast<int>(center_x) + 1; x < lane.cols; ++x) {
        if (lane_row[x] > 0U) {right_lane = x; break;}
      }
      if (left_lane >= 0) left_gaps.push_back(left_line - static_cast<double>(left_lane));
      if (right_lane >= 0) right_gaps.push_back(static_cast<double>(right_lane) - right_line);
    }

    result.drivable_fraction = corridor_pixels > 0U ?
      static_cast<double>(drivable_pixels) / static_cast<double>(corridor_pixels) : 0.0;
    result.drivable_detected =
      result.drivable_rows >= lane_corridor_minimum_valid_rows_ &&
      result.drivable_fraction >= lane_corridor_minimum_drivable_fraction_;
    result.left_samples = static_cast<int>(left_gaps.size());
    result.right_samples = static_cast<int>(right_gaps.size());
    result.left_valid = result.left_samples >= lane_corridor_minimum_valid_rows_;
    result.right_valid = result.right_samples >= lane_corridor_minimum_valid_rows_;
    if (result.left_valid) result.left_gap_px = robustClosestGap(left_gaps);
    if (result.right_valid) result.right_gap_px = robustClosestGap(right_gaps);

    // Abu-abu HANYA bila kedua evidence tidak ada: drivable area tidak terdeteksi
    // dan lane mask juga tidak terdeteksi. Bila salah satu evidence tersedia dan
    // lane tidak mendekati batas, kondisi visual harus GREEN (aman).
    const bool lane_mask_detected = result.left_samples > 0 || result.right_samples > 0;
    if (!result.drivable_detected && !lane_mask_detected) {
      result.left_status = "UNKNOWN";
      result.right_status = "UNKNOWN";
      result.recommendation = "NO_MASK_EVIDENCE";
      return result;
    }
    result.left_status = result.left_valid ? laneCorridorStatus(true, result.left_gap_px) : "GREEN";
    result.right_status = result.right_valid ? laneCorridorStatus(true, result.right_gap_px) : "GREEN";
    result.left_penetration_px = result.left_valid ? std::max(0.0, -result.left_gap_px) : 0.0;
    result.right_penetration_px = result.right_valid ? std::max(0.0, -result.right_gap_px) : 0.0;
    // Positif = sisi kanan masuk -> koreksi ke kiri. Negatif = sisi kiri masuk -> koreksi kanan.
    result.correction_error_px = result.right_penetration_px - result.left_penetration_px;
    const bool left_red = result.left_status == "RED";
    const bool right_red = result.right_status == "RED";
    if (left_red && right_red) result.recommendation = "BOTH_INTRUSION_STOP";
    else if (left_red) result.recommendation = "RECENTER_RIGHT";
    else if (right_red) result.recommendation = "RECENTER_LEFT";
    else if (result.left_status == "YELLOW" || result.right_status == "YELLOW") result.recommendation = "WARNING";
    else result.recommendation = "CLEAR";
    return result;
  }

  cv::Scalar laneCorridorColor(const std::string & status) const
  {
    if (status == "GREEN") return cv::Scalar(0, 255, 0);
    if (status == "YELLOW") return cv::Scalar(0, 220, 255);
    if (status == "RED") return cv::Scalar(0, 0, 255);
    return cv::Scalar(150, 150, 150);
  }

  void drawLaneCorridorOverlay(cv::Mat & image, const LaneCorridorDecision & corridor) const
  {
    if (!lane_corridor_overlay_enabled_ || image.empty()) return;
    const int top_y = std::clamp(
      static_cast<int>(std::lround(image.rows * lane_corridor_top_y_ratio_)), 0, image.rows - 2);
    const int bottom_y = std::clamp(
      static_cast<int>(std::lround(image.rows * lane_corridor_bottom_y_ratio_)), top_y + 1, image.rows - 1);
    const auto top = laneSafetyLineAtRow(top_y, image.cols, image.rows);
    const auto bottom = laneSafetyLineAtRow(bottom_y, image.cols, image.rows);
    const cv::Point top_left(static_cast<int>(std::lround(top.left_x)), top_y);
    const cv::Point top_right(static_cast<int>(std::lround(top.right_x)), top_y);
    const cv::Point bottom_left(static_cast<int>(std::lround(bottom.left_x)), bottom_y);
    const cv::Point bottom_right(static_cast<int>(std::lround(bottom.right_x)), bottom_y);

    // Preview hanya menampilkan dua garis safety miring. Tidak ada garis horizontal
    // atau label diagnostik, sehingga kamera tetap bersih dan mudah dibaca operator.
    cv::line(image, top_left, bottom_left, laneCorridorColor(corridor.left_status), 5, cv::LINE_AA);
    cv::line(image, top_right, bottom_right, laneCorridorColor(corridor.right_status), 5, cv::LINE_AA);
  }

  void publishLaneState(
    const rclcpp::Time & stamp, bool valid, double left_clearance,
    double right_clearance, double center_error, double heading_error,
    double road_width, int valid_rows, LaneCorridorDecision & corridor)
  {
    const bool left_evidence = corridor.left_valid && std::isfinite(corridor.left_gap_px);
    const bool right_evidence = corridor.right_valid && std::isfinite(corridor.right_gap_px);
    const bool corridor_evidence = left_evidence || right_evidence;

    const safety::PixelCorridorConfig pixel_config{
      lane_corridor_touch_margin_px_, lane_corridor_warning_gap_px_,
      lane_corridor_release_gap_px_, lane_corridor_correction_gain_m_per_px_,
      lane_corridor_max_correction_m_};
    const auto left_pixel = safety::updatePixelCorridorSide(
      left_evidence, corridor.left_gap_px, left_lane_recenter_latched_, pixel_config);
    const auto right_pixel = safety::updatePixelCorridorSide(
      right_evidence, corridor.right_gap_px, right_lane_recenter_latched_, pixel_config);

    left_lane_recenter_latched_ = left_pixel.latched;
    right_lane_recenter_latched_ = right_pixel.latched;
    corridor.left_recenter_latched = left_pixel.latched;
    corridor.right_recenter_latched = right_pixel.latched;
    corridor.left_correction_m = left_pixel.correction_m;
    corridor.right_correction_m = right_pixel.correction_m;
    if (left_evidence) corridor.left_status = left_pixel.status;
    if (right_evidence) corridor.right_status = right_pixel.status;

    bool effective_valid = valid || corridor_evidence;
    double effective_center_error = valid ? center_error : 0.0;
    auto metric_decision = safety::classifyLaneState(
      valid, left_clearance, right_clearance, center_error,
      lane_state_filter_->state(), lane_thresholds_);
    safety::LaneDecision decision = metric_decision;
    bool corridor_override = false;

    // Pixel safety mempunyai authority arah hanya setelah lane mask benar-benar
    // menyentuh/masuk garis. Zona kuning sebelum touch hanya warning. Setelah
    // touch, latch mempertahankan koreksi sampai gap >= release_gap_px.
    if (lane_corridor_control_enabled_) {
      if (left_pixel.latched && right_pixel.latched) {
        effective_valid = false;
        decision = {safety::LANE_LOST, true, "corridor_both_intrusion"};
        corridor.recommendation = "BOTH_INTRUSION_STOP";
        corridor_override = true;
      } else if (left_pixel.latched) {
        effective_valid = true;
        effective_center_error = (valid ? center_error : 0.0) - left_pixel.correction_m;
        decision = {safety::RECENTER_RIGHT,
          corridor.left_penetration_px >= lane_corridor_critical_penetration_px_,
          "corridor_left_touch_recover"};
        corridor.recommendation = "RECENTER_RIGHT";
        corridor_override = true;
      } else if (right_pixel.latched) {
        effective_valid = true;
        effective_center_error = (valid ? center_error : 0.0) + right_pixel.correction_m;
        decision = {safety::RECENTER_LEFT,
          corridor.right_penetration_px >= lane_corridor_critical_penetration_px_,
          "corridor_right_touch_recover"};
        corridor.recommendation = "RECENTER_LEFT";
        corridor_override = true;
      }
    }

    std::string state;
    if (corridor_override) {
      // Touch sudah melalui robust multi-row gap; recenter pixel tidak menunggu
      // filter state metric beberapa frame agar respons safety tidak terlambat.
      state = decision.state;
    } else if (!valid && corridor_evidence) {
      decision = {safety::NORMAL, false, "lane_mask_safety_clear"};
      state = safety::NORMAL;
    } else {
      state = lane_state_filter_->update(metric_decision.state);
      decision = metric_decision;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      latest_lane_state_ = state;
      latest_lane_valid_ = effective_valid;
      latest_lane_critical_ = decision.critical;
      latest_center_error_ = effective_center_error;
      latest_lane_time_ = now();
      have_lane_ = true;
    }
    const double confidence = std::clamp(static_cast<double>(valid_rows) / 12.0, 0.0, 1.0);
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(4)
         << "{\"backend\":\"cpu\",\"valid\":" << effective_valid
         << ",\"drivable_valid\":" << valid
         << ",\"state\":\"" << state << "\",\"critical\":" << decision.critical
         << ",\"reason\":\"" << decision.reason << "\""
         << ",\"left_clearance_m\":" << left_clearance
         << ",\"right_clearance_m\":" << right_clearance
         << ",\"center_error_m\":" << effective_center_error
         << ",\"drivable_center_error_m\":" << center_error
         << ",\"heading_error_rad\":" << heading_error
         << ",\"road_width_m\":" << road_width
         << ",\"confidence\":" << confidence
         << ",\"valid_rows\":" << valid_rows
         << ",\"corridor\":{\"enabled\":" << corridor.enabled
         << ",\"control_enabled\":" << lane_corridor_control_enabled_
         << ",\"drivable_detected\":" << corridor.drivable_detected
         << ",\"drivable_rows\":" << corridor.drivable_rows
         << ",\"drivable_fraction\":" << corridor.drivable_fraction
         << ",\"left_status\":\"" << corridor.left_status
         << "\",\"right_status\":\"" << corridor.right_status
         << "\",\"left_gap_px\":" << corridor.left_gap_px
         << ",\"right_gap_px\":" << corridor.right_gap_px
         << ",\"left_penetration_px\":" << corridor.left_penetration_px
         << ",\"right_penetration_px\":" << corridor.right_penetration_px
         << ",\"left_correction_m\":" << corridor.left_correction_m
         << ",\"right_correction_m\":" << corridor.right_correction_m
         << ",\"left_recenter_latched\":" << corridor.left_recenter_latched
         << ",\"right_recenter_latched\":" << corridor.right_recenter_latched
         << ",\"warning_gap_px\":" << lane_corridor_warning_gap_px_
         << ",\"touch_gap_px\":" << lane_corridor_touch_margin_px_
         << ",\"release_gap_px\":" << lane_corridor_release_gap_px_
         << ",\"correction_error_px\":" << corridor.correction_error_px
         << ",\"left_samples\":" << corridor.left_samples
         << ",\"right_samples\":" << corridor.right_samples
         << ",\"recommendation\":\"" << corridor.recommendation << "\"}"
         << ",\"stamp_ns\":" << stamp.nanoseconds() << '}';
    std_msgs::msg::String message;
    message.data = json.str();
    lane_state_pub_->publish(message);

    std_msgs::msg::String metrics;
    metrics.data = json.str();
    lane_metrics_pub_->publish(std::move(metrics));
  }

  void publishConnected(bool connected)
  {
    std_msgs::msg::Bool message;
    message.data = connected;
    connected_pub_->publish(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    camera_connected_ = connected;
  }

  void publishHealth(
    bool healthy, const std::string & reason,
    double mean_luma = std::numeric_limits<double>::quiet_NaN(),
    double stddev_luma = std::numeric_limits<double>::quiet_NaN(),
    double extreme_fraction = std::numeric_limits<double>::quiet_NaN(),
    double mean_gradient = std::numeric_limits<double>::quiet_NaN())
  {
    std_msgs::msg::Bool state;
    state.data = healthy;
    health_pub_->publish(state);
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(4)
         << "{\"backend\":\"cpu\",\"healthy\":" << healthy
         << ",\"reason\":\"" << reason << "\"";
    const auto field = [&json](const char * name, double value) {
      json << ",\"" << name << "\":";
      if (std::isfinite(value)) json << value; else json << "null";
    };
    field("mean_luma", mean_luma);
    field("stddev_luma", stddev_luma);
    field("extreme_fraction", extreme_fraction);
    field("mean_gradient", mean_gradient);
    json << '}';
    std_msgs::msg::String detail;
    detail.data = json.str();
    health_state_pub_->publish(detail);
    std::lock_guard<std::mutex> lock(state_mutex_);
    camera_healthy_ = healthy;
  }

  void publishEmergency(bool emergency)
  {
    std_msgs::msg::Bool message;
    message.data = emergency;
    emergency_pub_->publish(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    emergency_stop_ = emergency;
  }

  void publishCameraOnlyPerformance()
  {
    if (!performance_pub_) return;
    std_msgs::msg::String performance;
    std::ostringstream json;
    json << std::fixed << std::setprecision(3)
         << "{\"backend\":\"camera_only\",\"inference_enabled\":false"
         << ",\"capture_fps\":" << capture_fps_.load()
         << ",\"capture_frames_total\":" << capture_frames_total_.load()
         << ",\"capture_dropped_total\":" << capture_dropped_total_.load()
         << ",\"capture_overwrite_total\":" << capture_overwrite_total_.load()
         << ",\"web_preview_published_total\":" << web_preview_published_total_.load()
         << ",\"web_preview_fps_target\":" << web_preview_fps_
         << ",\"cpu_threads\":" << cpu_threads_
         << ",\"opencv_threads\":" << opencv_threads_
         << '}';
    performance.data = json.str();
    performance_pub_->publish(performance);
  }

  void captureLoop()
  {
    while (rclcpp::ok() && !capture_stop_.load()) {
      if (!openCamera()) {std::this_thread::sleep_for(50ms); continue;}
      cv::Mat frame;
      if (!capture_.read(frame) || frame.empty() || frame.type() != CV_8UC3) {
        ++capture_dropped_total_;
        ++camera_read_fail_streak_;
        camera_good_frame_streak_ = 0;
        if (camera_read_fail_streak_ < camera_read_fail_threshold_) {
          std::this_thread::sleep_for(10ms);
          continue;
        }
        capture_.release();
        camera_read_fail_streak_ = 0;
        publishConnected(false);
        publishHealth(false, "FRAME_READ_FAILED_STREAK");
        publishEmergency(inference_enabled_.load());
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Camera read gagal %d frame berturut-turut; reopen V4L2", camera_read_fail_threshold_);
        continue;
      }
      camera_read_fail_streak_ = 0;
      camera_good_frame_streak_ = std::min(camera_good_frame_streak_ + 1, camera_online_good_frames_);
      if (camera_good_frame_streak_ == camera_online_good_frames_) publishConnected(true);
      if (flip_horizontal_) cv::flip(frame, frame, 1);
      const auto steady_now = std::chrono::steady_clock::now();
      const auto ros_stamp = now();
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (latest_frame_sequence_ > consumed_frame_sequence_) ++capture_overwrite_total_;
        latest_frame_ = frame;
        latest_frame_stamp_ = ros_stamp;
        latest_frame_steady_ = steady_now;
        ++latest_frame_sequence_;
      }
      ++capture_frames_total_;
      if (!inference_enabled_.load()) {
        // Camera-only is a valid lightweight state: camera capture/preview stays
        // alive while model inference, postprocess and obstacle/lane geometry are idle.
        const auto frames = capture_frames_total_.load();
        if (frames == 1U || frames % 30U == 0U) {
          publishConnected(true);
          publishHealth(true, "CAMERA_ONLY");
          publishEmergency(false);
          publishCameraOnlyPerformance();
        }
      }
      if (last_capture_time_.time_since_epoch().count() != 0) {
        const double dt = std::chrono::duration<double>(steady_now - last_capture_time_).count();
        if (dt > 1.0e-6) {const double f = 1.0 / dt; const double old_fps = capture_fps_.load(); capture_fps_.store(old_fps <= 0.0 ? f : 0.12 * f + 0.88 * old_fps);}
      }
      last_capture_time_ = steady_now;
      const double since_visual = last_raw_publish_time_.time_since_epoch().count() == 0 ? 1e9 :
        std::chrono::duration<double>(steady_now - last_raw_publish_time_).count();
      if (publish_raw_ && since_visual >= 1.0 / visual_publish_rate_hz_) {
        publishImage(raw_pub_, ros_stamp, frame, "bgr8");
        publishCameraInfo(ros_stamp, frame.cols, frame.rows);
        last_raw_publish_time_ = steady_now;
      }
      const double since_preview = last_web_preview_publish_time_.time_since_epoch().count() == 0 ? 1e9 :
        std::chrono::duration<double>(steady_now - last_web_preview_publish_time_).count();
      if (!inference_enabled_.load() && web_preview_enabled_ && compressed_preview_pub_ &&
        compressed_preview_pub_->get_subscription_count() > 0U && since_preview >= 1.0 / web_preview_fps_)
      {
        cv::Mat preview;
        cv::resize(frame, preview, cv::Size(web_preview_width_, web_preview_height_), 0.0, 0.0, cv::INTER_AREA);
        std::vector<uchar> jpeg;
        if (cv::imencode(".jpg", preview, jpeg, {cv::IMWRITE_JPEG_QUALITY, web_preview_jpeg_quality_}) && !jpeg.empty()) {
          sensor_msgs::msg::CompressedImage message;
          message.header.stamp = ros_stamp;
          message.header.frame_id = frame_id_;
          message.format = "jpeg; source=camera_raw";
          message.data.assign(jpeg.begin(), jpeg.end());
          compressed_preview_pub_->publish(std::move(message));
          ++web_preview_published_total_;
        }
        last_web_preview_publish_time_ = steady_now;
      }
    }
  }

  void inferenceTick()
  {
    if (!inference_enabled_.load()) return;
    try {
      loadModel();
    } catch (const std::exception & error) {
      publishHealth(false, "MODEL_LOAD_ERROR");
      publishEmergency(true);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "YOLOPv2 lazy-load gagal: %s", error.what());
      return;
    }
    cv::Mat frame;
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::chrono::steady_clock::time_point frame_steady{};
    uint64_t sequence = 0U;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (latest_frame_.empty() || latest_frame_sequence_ == consumed_frame_sequence_) return;
      frame = latest_frame_.clone();
      stamp = latest_frame_stamp_;
      frame_steady = latest_frame_steady_;
      sequence = latest_frame_sequence_;
      consumed_frame_sequence_ = sequence;
    }
    const auto started = std::chrono::steady_clock::now();
    const double frame_age_ms = frame_steady.time_since_epoch().count() == 0 ? 0.0 : std::chrono::duration<double, std::milli>(started - frame_steady).count();
    try {
      prepareHomography(frame.cols, frame.rows);
      double mean_luma = 0.0;
      double image_stddev = 0.0;
      double extreme_fraction = 1.0;
      double mean_gradient = 0.0;
      const bool image_healthy = cameraHealthy(
        frame, mean_luma, image_stddev, extreme_fraction, mean_gradient);
      const auto preprocess_started = std::chrono::steady_clock::now();
      const cv::Mat input = letterbox(frame);
      const auto inference_started = std::chrono::steady_clock::now();
      const double preprocess_ms = std::chrono::duration<double, std::milli>(inference_started - preprocess_started).count();
      std::vector<cv::Mat> outputs = forwardTorch(input);
      const auto decode_started = std::chrono::steady_clock::now();
      const double inference_ms = std::chrono::duration<double, std::milli>(decode_started - inference_started).count();
      validateOutputs(outputs);
      auto detections = decodeDetections(outputs, frame.cols, frame.rows);
      cv::Mat drivable;
      cv::Mat lane;
      decodeMasks(outputs, frame.cols, frame.rows, drivable, lane);
      const auto geometry_started = std::chrono::steady_clock::now();
      const double decode_ms = std::chrono::duration<double, std::milli>(geometry_started - decode_started).count();

      bool metric_emergency_raw = false;
      std::vector<std::pair<float, float>> obstacle_centers;
      std::vector<MetricObstacle> metric_obstacles;
      projectObstacles(detections, drivable, metric_emergency_raw, obstacle_centers, metric_obstacles);
      const NearFieldDecision near_field = evaluateNearField(detections, drivable, frame.cols, frame.rows);
      // Metric emergency hanya memiliki authority setelah homography divalidasi secara fisik.
      const bool metric_emergency = camera_metric_calibration_validated_ && metric_emergency_raw;
      const size_t metric_candidate_count = metric_obstacles.size();
      const auto confirmed_obstacles = updateObstacleTracks(metric_obstacles);
      obstacle_centers.clear();
      for (const auto & obstacle : confirmed_obstacles) obstacle_centers.emplace_back(obstacle.forward_m, obstacle.left_m);
      const auto object_points = obstaclePoints(confirmed_obstacles);
      bool lane_valid = false;
      double left_clearance = 0.0;
      double right_clearance = 0.0;
      double center_error = 0.0;
      double heading_error = 0.0;
      double road_width = 0.0;
      int valid_rows = 0;
      const auto boundary_points = buildDrivableBoundary(
        drivable, lane_valid, left_clearance, right_clearance, center_error,
        heading_error, road_width, valid_rows);
      LaneCorridorDecision lane_corridor = evaluateLaneCorridor(lane, drivable);

      object_pub_->publish(makeObstacleCloud(stamp, confirmed_obstacles));
      boundary_pub_->publish(makeCloud(stamp, boundary_points));
      publishClearingFan(stamp, obstacle_centers);
      publishObstacleMetrics(stamp, confirmed_obstacles);
      publishDrivableSpace(stamp, lane_valid, valid_rows, boundary_points.size());
      publishNearFieldState(near_field, metric_emergency);
      publishLaneState(
        stamp, lane_valid, left_clearance, right_clearance, center_error,
        heading_error, road_width, valid_rows, lane_corridor);
      publishEmergency(near_field.emergency || metric_emergency || !image_healthy);
      publishHealth(
        image_healthy, image_healthy ? "OK" : "IMAGE_DEGRADED",
        mean_luma, image_stddev, extreme_fraction, mean_gradient);
      publishConnected(true);
      publishDetectionMessages(stamp, detections, frame.cols, frame.rows);
      const auto publish_started = std::chrono::steady_clock::now();
      const double geometry_ms = std::chrono::duration<double, std::milli>(publish_started - geometry_started).count();

      if (false && publish_raw_) publishImage(raw_pub_, stamp, frame, "bgr8");
      if (publish_drivable_) publishImage(drivable_pub_, stamp, drivable, "mono8");
      if (publish_lane_) publishImage(lane_pub_, stamp, lane, "mono8");
      const bool web_wants_annotated = web_preview_enabled_ && compressed_preview_pub_ &&
        compressed_preview_pub_->get_subscription_count() > 0U;
      const bool rviz_wants_annotated = annotated_pub_->get_subscription_count() > 0U;
      if (publish_annotated_ && (rviz_wants_annotated || web_wants_annotated)) {
        cv::Mat annotated = frame.clone();
        cv::Mat green(frame.size(), CV_8UC3, cv::Scalar(0, 255, 0));
        cv::Mat red(frame.size(), CV_8UC3, cv::Scalar(0, 0, 255));
        green.copyTo(annotated, drivable);
        cv::addWeighted(frame, 1.0 - overlay_alpha_, annotated, overlay_alpha_, 0.0, annotated);
        red.copyTo(annotated, lane);
        // Dua garis safety digambar setelah mask agar warna status kiri/kanan selalu terlihat.
        drawLaneCorridorOverlay(annotated, lane_corridor);
        // The official YOLOPv2 checkpoint used here does not expose reliable COCO
        // semantic class identities.  Keep this layer class-agnostic; the separate
        // COCO semantic detector owns person/car/motorcycle labels.
        const auto class_name = [](int) -> const char * { return "obstacle"; };
        for (const auto & d : detections) {
          const cv::Point p1(static_cast<int>(d.x1), static_cast<int>(d.y1));
          const cv::Point p2(static_cast<int>(d.x2), static_cast<int>(d.y2));
          cv::rectangle(annotated, p1, p2, cv::Scalar(0, 255, 255), box_thickness_);
          std::ostringstream label;
          label << class_name(d.class_id) << " " << std::fixed << std::setprecision(2) << d.score;
          cv::putText(annotated, label.str(), cv::Point(p1.x, std::max(16, p1.y - 6)),
            cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }
        if (rviz_wants_annotated) {
          publishImage(annotated_pub_, stamp, annotated, "bgr8");
          ++rviz_published_total_;
        }
        const auto web_now = std::chrono::steady_clock::now();
        const double since_web = last_web_preview_publish_time_.time_since_epoch().count() == 0 ? 1e9 :
          std::chrono::duration<double>(web_now - last_web_preview_publish_time_).count();
        if (web_wants_annotated && since_web >= 1.0 / web_preview_fps_) {
          cv::Mat preview;
          cv::resize(annotated, preview, cv::Size(web_preview_width_, web_preview_height_), 0.0, 0.0, cv::INTER_AREA);
          std::vector<uchar> jpeg;
          if (cv::imencode(".jpg", preview, jpeg, {cv::IMWRITE_JPEG_QUALITY, web_preview_jpeg_quality_}) && !jpeg.empty()) {
            sensor_msgs::msg::CompressedImage message;
            message.header.stamp = stamp;
            message.header.frame_id = frame_id_;
            message.format = "jpeg; source=yolop_annotated";
            message.data.assign(jpeg.begin(), jpeg.end());
            compressed_preview_pub_->publish(std::move(message));
            ++web_preview_published_total_;
          }
          last_web_preview_publish_time_ = web_now;
        }
      }

      const auto finished = std::chrono::steady_clock::now();
      const double publish_ms = std::chrono::duration<double, std::milli>(finished - publish_started).count();
      const double postprocess_ms = decode_ms + geometry_ms + publish_ms;
      const double elapsed_ms = std::chrono::duration<double, std::milli>(finished - started).count();
      double instantaneous_fps = 1000.0 / std::max(1.0, elapsed_ms);
      if (last_inference_finished_.time_since_epoch().count() != 0) {
        const double completion_period =
          std::chrono::duration<double>(finished - last_inference_finished_).count();
        if (completion_period > 1.0e-6) instantaneous_fps = 1.0 / completion_period;
      }
      last_inference_finished_ = finished;
      performance_fps_ = performance_fps_ <= 0.0 ? instantaneous_fps :
        0.15 * instantaneous_fps + 0.85 * performance_fps_;
      timing_window_ms_.push_back(elapsed_ms);
      while (timing_window_ms_.size() > 120U) timing_window_ms_.pop_front();
      std::vector<double> timing(timing_window_ms_.begin(), timing_window_ms_.end());
      std::sort(timing.begin(), timing.end());
      const double mean_ms = std::accumulate(timing.begin(), timing.end(), 0.0) /
        static_cast<double>(std::max<size_t>(1U, timing.size()));
      const size_t p95_index = timing.empty() ? 0U :
        std::min(timing.size() - 1U, static_cast<size_t>(std::ceil(0.95 * timing.size()) - 1.0));
      const double p95_ms = timing.empty() ? elapsed_ms : timing[p95_index];
      const double min_ms = timing.empty() ? elapsed_ms : timing.front();
      const double max_ms = timing.empty() ? elapsed_ms : timing.back();
      double variance_ms = 0.0;
      for (const double value : timing) {
        const double delta = value - mean_ms;
        variance_ms += delta * delta;
      }
      const double std_ms = timing.empty() ? 0.0 :
        std::sqrt(variance_ms / static_cast<double>(timing.size()));

      std_msgs::msg::String performance;
      std::ostringstream json;
      json << std::fixed << std::setprecision(3)
           << "{\"backend\":\"cpu\",\"inference_enabled\":true,\"window_frames\":" << timing.size()
           << ",\"pipeline_fps\":" << performance_fps_
           << ",\"pipeline_fps_ema\":" << performance_fps_
           << ",\"pipeline_ms_per_frame\":" << mean_ms
           << ",\"pipeline_ms\":" << elapsed_ms
           << ",\"pipeline_p95_ms\":" << p95_ms
           << ",\"pipeline_min_ms\":" << min_ms
           << ",\"pipeline_max_ms\":" << max_ms
           << ",\"pipeline_std_ms\":" << std_ms
           << ",\"capture_fps\":" << capture_fps_.load()
           << ",\"frame_age_ms\":" << frame_age_ms
           << ",\"preprocess_ms\":" << preprocess_ms
           << ",\"inference_ms\":" << inference_ms
           << ",\"decode_ms\":" << decode_ms
           << ",\"geometry_ms\":" << geometry_ms
           << ",\"publish_ms\":" << publish_ms
           << ",\"postprocess_ms\":" << postprocess_ms
           << ",\"target_fps\":" << inference_fps_
           << ",\"cpu_threads\":" << cpu_threads_
           << ",\"opencv_threads\":" << opencv_threads_
           << ",\"torch_optimized\":" << (torch_optimized_active_ ? "true" : "false")
           << ",\"capture_dropped_total\":" << capture_dropped_total_.load()
           << ",\"capture_overwrite_total\":" << capture_overwrite_total_.load()
           << ",\"capture_frames_total\":" << capture_frames_total_.load()
           << ",\"rviz_published_total\":" << rviz_published_total_
           << ",\"rviz_dropped_total\":" << rviz_dropped_total_
           << ",\"web_preview_published_total\":" << web_preview_published_total_.load()
           << ",\"web_preview_fps_target\":" << web_preview_fps_
           << ",\"raw_detection_count\":" << detections.size()
           << ",\"metric_candidate_count\":" << metric_candidate_count
           << ",\"confirmed_obstacle_count\":" << confirmed_obstacles.size()
           << ",\"object_points\":" << object_points.size()
           << ",\"camera_mean_luma\":" << mean_luma
           << ",\"camera_stddev\":" << image_stddev
           << ",\"camera_mean_gradient\":" << mean_gradient
           << ",\"extreme_fraction\":" << extreme_fraction
           << ",\"timing_scope\":\"inferenceTick_total\"}";
      performance.data = json.str();
      performance_pub_->publish(performance);
      if (elapsed_ms > 900.0) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 30000,
          "CPU inference %.1f ms > 0.9 s safety freshness target; perception tetap berjalan tetapi autonomous safety fail-closed. Jetson GPU direkomendasikan untuk motion authority.",
          elapsed_ms);
      }
    } catch (const cv::Exception & error) {
      publishHealth(false, "OPENCV_INFERENCE_ERROR");
      publishEmergency(true);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "OpenCV CPU perception error: %s", error.what());
    } catch (const std::exception & error) {
      publishHealth(false, "CPU_PIPELINE_ERROR");
      publishEmergency(true);
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "CPU perception error: %s", error.what());
    }
  }

  void controlTick()
  {
    ++control_sequence_;
    geometry_msgs::msg::Twist nav;
    geometry_msgs::msg::Twist desired;
    geometry_msgs::msg::Twist output;
    std::string lane_state = safety::LANE_LOST;
    std::string decision = "WAIT_NAV_CMD";
    bool nav_fresh = false;
    bool lane_fresh = false;
    bool lane_valid = false;
    bool lane_critical = false;
    bool camera_connected = false;
    bool camera_healthy = false;
    bool emergency = true;
    double center_error = 0.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      nav_fresh = have_nav_cmd_ && last_nav_cmd_time_.nanoseconds() > 0 &&
        (now() - last_nav_cmd_time_).seconds() >= 0.0 &&
        (now() - last_nav_cmd_time_).seconds() <= cmd_timeout_sec_;
      if (nav_fresh) nav = last_nav_cmd_;
      lane_fresh = have_lane_ && latest_lane_time_.nanoseconds() > 0 &&
        (now() - latest_lane_time_).seconds() >= 0.0 &&
        (now() - latest_lane_time_).seconds() <= lane_state_timeout_sec_;
      if (lane_fresh) {
        lane_state = latest_lane_state_;
        lane_valid = latest_lane_valid_;
        lane_critical = latest_lane_critical_;
        center_error = latest_center_error_;
      }
      camera_connected = camera_connected_;
      camera_healthy = camera_healthy_;
      emergency = emergency_stop_;
    }

    desired = nav;
    if (!nav_fresh) {
      decision = "NAV_CMD_STALE_STOP";
      desired = geometry_msgs::msg::Twist{};
    } else if (emergency) {
      decision = "EMERGENCY_STOP";
      desired = geometry_msgs::msg::Twist{};
    } else if (!camera_connected) {
      decision = "CAMERA_DISCONNECTED_STOP";
      desired = geometry_msgs::msg::Twist{};
    } else if (!camera_healthy) {
      decision = "CAMERA_UNHEALTHY_STOP";
      desired = geometry_msgs::msg::Twist{};
    } else if (!lane_safety_enabled_) {
      decision = "DISABLED_PASSTHROUGH";
    } else if (!camera_metric_calibration_validated_) {
      decision = "CALIBRATION_REQUIRED_PASSTHROUGH";
    } else if (!lane_fresh || !lane_valid || lane_state == safety::LANE_LOST) {
      decision = safety::LANE_LOST;
      desired = geometry_msgs::msg::Twist{};
    } else if (lane_state == safety::RECENTER_LEFT || lane_state == safety::RECENTER_RIGHT) {
      const auto mixed = safety::mixRecenterCommand(
        nav.linear.x, nav.angular.z, center_error, 0.0, lane_critical, mixer_config_);
      desired = geometry_msgs::msg::Twist{};
      desired.linear.x = mixed.linear_x;
      desired.angular.z = mixed.angular_z;
      decision = lane_state;
    } else {
      decision = safety::NORMAL;
    }

    const bool applied = lane_safety_enabled_ && camera_metric_calibration_validated_ &&
      control_mode_ == "active";
    output = applied ? desired : nav;
    advisory_pub_->publish(output);
    std_msgs::msg::String detail;
    std::ostringstream json;
    json << std::boolalpha << std::fixed << std::setprecision(5)
         << "{\"sequence\":" << control_sequence_
         << ",\"backend\":\"cpu\""
         << ",\"mode\":\"" << control_mode_
         << "\",\"enabled\":" << lane_safety_enabled_
         << ",\"metric_calibrated\":" << camera_metric_calibration_validated_
         << ",\"applied\":" << applied
         << ",\"decision\":\"" << decision
         << "\",\"lane_state\":\"" << lane_state
         << "\",\"lane_valid\":" << lane_valid
         << ",\"lane_fresh\":" << lane_fresh
         << ",\"nav_cmd_fresh\":" << nav_fresh
         << ",\"critical\":" << lane_critical
         << ",\"center_error_m\":" << center_error
         << ",\"raw_recenter_blocked\":false"
         << ",\"recenter_blocked\":false"
         << ",\"nav_cmd\":{\"linear_x\":" << nav.linear.x
         << ",\"angular_z\":" << nav.angular.z
         << "},\"calculated_cmd\":{\"linear_x\":" << desired.linear.x
         << ",\"angular_z\":" << desired.angular.z
         << "},\"output_cmd\":{\"linear_x\":" << output.linear.x
         << ",\"angular_z\":" << output.angular.z << "}}";
    detail.data = json.str();
    lane_control_state_pub_->publish(detail);
  }

  torch::jit::script::Module module_;
  cv::VideoCapture capture_;
  cv::Mat homography_;
  int homography_width_{0};
  int homography_height_{0};
  float gain_{1.0F};
  int pad_x_{0};
  int pad_y_{0};
  int resized_width_{MODEL_WIDTH};
  int resized_height_{MODEL_HEIGHT};

  std::string pt_model_path_;
  std::atomic_bool inference_enabled_{false};
  std::atomic_bool model_loaded_{false};
  std::mutex model_mutex_;
  double inference_fps_{5.0};
  int cpu_threads_{2};
  int opencv_threads_{1};
  bool torch_optimize_for_inference_{true};
  bool torch_optimized_active_{false};
  std::string rgb_device_{"auto"};
  int requested_width_{1280};
  int requested_height_{720};
  int camera_fps_{30};
  std::string pixel_format_{"MJPEG"};
  bool strict_camera_mode_{false};
  bool flip_horizontal_{true};
  bool hotplug_retry_{true};
  double retry_sec_{2.0};
  int camera_read_fail_threshold_{5};
  int camera_online_good_frames_{3};
  int camera_read_fail_streak_{0};
  int camera_good_frame_streak_{0};
  float confidence_threshold_{0.10F};
  float iou_threshold_{0.45F};
  float lane_threshold_{0.50F};
  int max_candidates_{16384};
  int max_detections_{300};
  bool publish_annotated_{true};
  bool publish_raw_{true};
  double visual_publish_rate_hz_{10.0};
  bool web_preview_enabled_{true};
  double web_preview_fps_{5.0};
  int web_preview_width_{640};
  int web_preview_height_{360};
  int web_preview_jpeg_quality_{75};
  std::string web_preview_topic_{"/camera/astra/image_preview/compressed"};
  bool publish_drivable_{false};
  bool publish_lane_{false};
  bool publish_detections_{false};
  double overlay_alpha_{0.45};
  int box_thickness_{2};
  std::string frame_id_;
  std::string metric_frame_id_;
  std::string annotated_topic_, raw_topic_, camera_info_topic_, detections_topic_;
  std::string drivable_topic_, lane_topic_, performance_topic_;
  std::string camera_connected_topic_, camera_health_topic_, camera_health_state_topic_;
  std::string emergency_topic_, object_points_topic_, clearing_points_topic_;
  std::string drivable_boundary_topic_, lane_state_topic_, lane_control_state_topic_;
  std::string lane_metrics_topic_, drivable_space_topic_, obstacle_metrics_topic_, near_field_state_topic_;
  std::string raw_detection_topic_, nav_cmd_topic_, safe_cmd_topic_;
  double control_rate_hz_{20.0};
  double cmd_timeout_sec_{0.50};
  double lane_state_timeout_sec_{0.50};
  bool camera_metric_calibration_validated_{false};
  bool lane_safety_enabled_{false};
  bool lane_corridor_overlay_enabled_{true};
  bool lane_corridor_control_enabled_{true};
  double lane_corridor_top_y_ratio_{0.02};
  double lane_corridor_bottom_y_ratio_{0.98};
  double lane_corridor_camera_height_m_{0.736};
  double lane_corridor_camera_pitch_deg_{0.0};
  double lane_corridor_safety_margin_m_{0.30};
  double lane_corridor_far_lookahead_m_{3.8};
  double lane_corridor_center_offset_px_{0.0};
  double lane_corridor_left_offset_px_{0.0};
  double lane_corridor_right_offset_px_{0.0};
  double lane_corridor_warning_gap_px_{36.0};
  double lane_corridor_touch_margin_px_{2.0};
  double lane_corridor_release_gap_px_{48.0};
  double lane_corridor_critical_penetration_px_{24.0};
  int lane_corridor_sample_stride_px_{6};
  int lane_corridor_minimum_valid_rows_{6};
  double lane_corridor_minimum_drivable_fraction_{0.20};
  double lane_corridor_correction_gain_m_per_px_{0.004};
  double lane_corridor_max_correction_m_{0.35};
  bool left_lane_recenter_latched_{false};
  bool right_lane_recenter_latched_{false};
  std::string control_mode_{"active"};
  safety::MixerConfig mixer_config_{};

  int ground_calibration_width_{1280};
  int ground_calibration_height_{720};
  std::vector<double> ground_src_, ground_dst_;
  int ground_canvas_width_{1280};
  int ground_canvas_height_{720};
  double ground_origin_x_{596.8667};
  double ground_origin_y_{719.0};
  double ground_scale_x_{0.0039093041};
  double ground_scale_y_{0.0097357441};
  double forward_offset_{0.0};
  double lateral_offset_{0.0};
  double min_forward_{0.20};
  double max_forward_{4.0};
  double max_abs_left_{2.5};
  double minimum_obstacle_confidence_{0.30};
  std::vector<double> obstacle_distance_calibration_coefficients_{1.0, 0.0, 1.0, 0.0};
  bool accept_all_obstacles_{false};
  std::vector<int64_t> safety_classes_;
  bool require_drivable_contact_{true};
  int drivable_contact_radius_px_{12};
  int drivable_contact_vertical_tolerance_px_{8};
  int drivable_contact_min_samples_{3};
  double drivable_contact_min_fraction_{0.20};
  double track_match_distance_m_{0.90};
  double track_ema_alpha_{0.45};
  int track_confirm_hits_{3};
  int track_max_missed_frames_{8};
  int next_track_id_{1};
  std::vector<ObstacleTrack> obstacle_tracks_;
  double obstacle_forward_min_m_{0.20};
  double obstacle_forward_max_m_{3.0};
  double minimum_obstacle_width_m_{0.20};
  int points_per_box_{5};
  int clearing_ray_count_{41};
  double clearing_fov_rad_{1.7453};
  double clearing_range_m_{4.5};
  double clearing_obstacle_margin_m_{0.20};
  double emergency_distance_m_{0.65};
  double emergency_half_width_m_{0.55};
  bool near_field_emergency_enabled_{true};
  double near_field_min_confidence_{0.25};
  double near_field_center_corridor_fraction_{0.65};
  double near_field_min_bbox_height_fraction_{0.20};
  bool near_field_drivable_guard_enabled_{true};
  double near_field_bottom_roi_fraction_{0.22};
  double near_field_min_drivable_fraction_{0.12};
  int near_field_confirm_frames_{2};
  int near_field_release_frames_{5};
  int near_field_candidate_hits_{0};
  int near_field_release_hits_{0};
  bool near_field_latched_{false};
  int last_near_field_class_id_{-1};
  double last_near_field_confidence_{0.0};
  double last_near_field_bbox_height_fraction_{0.0};
  double last_near_field_center_x_fraction_{0.5};
  bool last_near_field_drivable_contact_{false};
  double lane_vehicle_width_m_{0.55};
  safety::LaneThresholds lane_thresholds_;
  std::unique_ptr<safety::ConfirmedState> lane_state_filter_;
  int health_dark_{18};
  int health_bright_{245};
  double health_extreme_fraction_{0.97};
  double health_min_stddev_{5.0};
  double health_min_gradient_{2.0};
  double camera_fx_{910.0}, camera_fy_{910.0}, camera_cx_{640.0}, camera_cy_{360.0};
  std::vector<double> camera_distortion_;

  std::mutex frame_mutex_;
  cv::Mat latest_frame_;
  rclcpp::Time latest_frame_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point latest_frame_steady_{};
  uint64_t latest_frame_sequence_{0U};
  uint64_t consumed_frame_sequence_{0U};
  std::atomic_bool capture_stop_{false};
  std::thread capture_thread_;
  std::atomic<double> capture_fps_{0.0};
  std::atomic<uint64_t> capture_frames_total_{0U};
  std::atomic<uint64_t> capture_overwrite_total_{0U};
  std::atomic<uint64_t> web_preview_published_total_{0U};
  std::chrono::steady_clock::time_point last_capture_time_{};
  std::chrono::steady_clock::time_point last_raw_publish_time_{};
  std::chrono::steady_clock::time_point last_web_preview_publish_time_{};

  std::mutex state_mutex_;
  geometry_msgs::msg::Twist last_nav_cmd_;
  rclcpp::Time last_nav_cmd_time_{0, 0, RCL_ROS_TIME};
  bool have_nav_cmd_{false};
  bool camera_connected_{false};
  bool camera_healthy_{false};
  bool emergency_stop_{true};
  bool latest_lane_valid_{false};
  bool latest_lane_critical_{false};
  std::string latest_lane_state_{safety::LANE_LOST};
  double latest_center_error_{0.0};
  rclcpp::Time latest_lane_time_{0, 0, RCL_ROS_TIME};
  bool have_lane_{false};
  uint64_t control_sequence_{0U};
  double performance_fps_{0.0};
  std::atomic<uint64_t> capture_dropped_total_{0U};
  uint64_t rviz_published_total_{0U};
  uint64_t rviz_dropped_total_{0U};
  std::deque<double> timing_window_ms_;
  std::chrono::steady_clock::time_point last_inference_finished_{};
  std::chrono::steady_clock::time_point last_open_attempt_{};

  rclcpp::TimerBase::SharedPtr inference_timer_, control_timer_;
  rclcpp::CallbackGroup::SharedPtr inference_group_, control_group_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_, raw_pub_, drivable_pub_, lane_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_preview_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr performance_pub_, health_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_state_pub_, lane_control_state_pub_, raw_detection_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_metrics_pub_, drivable_space_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr obstacle_metrics_pub_, near_field_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_, health_pub_, emergency_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_pub_, clearing_pub_, boundary_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr advisory_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_cmd_sub_;
};

}  // namespace perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<perception::AstraYolopCpuNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "perception_cpu_node fatal: %s\n", error.what());
    exit_code = 2;
  }
  rclcpp::shutdown();
  return exit_code;
}
