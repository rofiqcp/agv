#!/usr/bin/python3
import json
import math
import os
import struct
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from nav2_msgs.srv import ClearEntireCostmap

# TorchVision COCO label -> project internal safety label.
# Internal mapping is deliberately kept compatible with the thesis/runtime:
# 0=person, 2=car, 3=motorcycle.
COCO_TO_INTERNAL = {
    1: (0, "person"),
    3: (2, "car"),
    4: (3, "motorcycle"),
}

class SemanticObstacleNode(Node):
    def __init__(self):
        super().__init__("semantic_obstacle")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.declare_parameter("input_topic", "/camera/astra/image_raw")
        self.declare_parameter("output_topic", "/perception/semantic_detections")
        self.declare_parameter("status_topic", "/perception/semantic_status")
        self.declare_parameter("object_points_topic", "/perception/object_points")
        self.declare_parameter("metric_frame_id", "base_footprint")
        self.declare_parameter("points_per_box", 3)
        self.declare_parameter("obstacle_forward_min_m", 0.20)
        self.declare_parameter("obstacle_forward_max_m", 5.50)
        self.declare_parameter("obstacle_max_abs_left_m", 2.50)
        # Direct homography is less reliable near image edges. Raw-detector
        # supported matches may use the full ROI; fallback-only candidates get
        # a tighter lateral gate before they can become Nav2 obstacles.
        self.declare_parameter("fallback_metric_max_abs_left_m", 2.00)
        self.declare_parameter("minimum_obstacle_width_m", 0.30)
        # HUMAN distance calibration fitted from the latest 1-5 m multi-orientation
        # BAB IV dataset: distance_m = intercept + slope * bbox_height_fraction.
        self.declare_parameter("person_bbox_distance_intercept_m", 7.1447892353)
        self.declare_parameter("person_bbox_distance_slope_m", -6.01911242)
        self.declare_parameter("person_bbox_distance_min_m", 0.50)
        self.declare_parameter("person_bbox_distance_max_m", 5.50)
        # Dedicated semantic clearing is intentionally separate from the broad
        # YOLOPv2 clearing fan. It only clears the last HUMAN corridor after
        # consecutive semantic misses, preventing stale human ghosts in Nav2.
        self.declare_parameter("semantic_clearing_points_topic", "/perception/semantic_clearing_points")
        self.declare_parameter("human_clear_confirm_misses", 3)
        # 4.5A world-fixed HUMAN memory. During an armed obstacle trial the
        # first valid HUMAN position is latched in odom so a temporary camera
        # FOV loss while steering cannot make Nav2 forget the obstacle.
        self.declare_parameter("obstacle_memory_enabled", True)
        self.declare_parameter("obstacle_memory_guard_topic", "/perception/final_trial_guard")
        self.declare_parameter("obstacle_memory_width_m", 0.55)
        self.declare_parameter("obstacle_memory_release_behind_m", 0.80)
        self.declare_parameter("obstacle_memory_release_confirm_updates", 3)
        self.declare_parameter("human_detection_hold_sec", 2.0)
        self.declare_parameter("human_track_continue_min_score", 0.08)
        self.declare_parameter("human_track_iou_min", 0.10)
        self.declare_parameter("human_track_center_distance_max", 0.12)
        self.declare_parameter("human_track_height_ratio_min", 0.55)
        self.declare_parameter("human_track_max_without_strong_sec", 4.0)
        self.declare_parameter("semantic_watchdog_enabled", True)
        self.declare_parameter("semantic_watchdog_inference_timeout_sec", 2.5)
        self.declare_parameter("semantic_watchdog_output_timeout_sec", 3.0)
        self.declare_parameter("semantic_watchdog_upstream_fresh_sec", 2.0)
        self.declare_parameter("semantic_watchdog_startup_grace_sec", 8.0)
        self.declare_parameter("human_clear_burst_frames", 3)
        self.declare_parameter("human_clear_half_width_m", 0.75)
        self.declare_parameter("human_clear_extra_forward_m", 0.80)
        self.declare_parameter("human_clear_ray_count", 11)
        self.declare_parameter("human_clear_history_size", 16)
        self.declare_parameter("human_clear_motion_threshold_m", 0.25)
        self.declare_parameter("human_remark_delay_sec", 0.08)
        self.declare_parameter("human_remark_timer_hz", 20.0)
        self.declare_parameter("boundary_points_topic", "/perception/drivable_boundary_points")
        self.declare_parameter("fallback_full_costmap_clear_enabled", True)
        self.declare_parameter("fallback_full_costmap_clear_cooldown_sec", 2.0)
        self.declare_parameter("fallback_full_clear_confirm_frames", 3)
        self.declare_parameter("boundary_fresh_timeout_sec", 1.0)
        self.declare_parameter("local_costmap_clear_service", "/local_costmap/clear_entirely_local_costmap")
        # If semantic inference stops completely there are no "miss" callbacks.
        # This independent watchdog clears the last HUMAN footprint once the
        # semantic stream has been stale long enough; motion remains fail-closed
        # through the existing SOURCE STALE gate.
        self.declare_parameter("human_clear_stale_timeout_sec", 1.20)
        self.declare_parameter("human_clear_watchdog_hz", 10.0)
        self.declare_parameter("score_threshold", 0.20)
        # Adaptive confidence: close/large objects at 1-2 m may keep the lower
        # model floor, while small/far detections require stronger confidence.
        self.declare_parameter("strict_score_threshold", 0.30)
        self.declare_parameter("close_range_score_threshold", 0.20)
        self.declare_parameter("close_range_min_bbox_height_fraction", 0.20)
        # Person <-> motorcycle are mutually exclusive only when the two boxes
        # are essentially the same physical object. A rider + motorcycle remain
        # separate when geometry is materially different.
        self.declare_parameter("person_motor_conflict_iou", 0.60)
        self.declare_parameter("person_motor_conflict_area_ratio_min", 0.55)
        self.declare_parameter("person_motor_min_score_margin", 0.08)
        # Same-class duplicate suppression for one physical object. IoM catches
        # nested boxes where standard IoU is small because one box is oversized.
        self.declare_parameter("same_class_nms_iou", 0.35)
        self.declare_parameter("same_class_nms_iom", 0.55)
        self.declare_parameter("inference_hz", 2.0)
        self.declare_parameter("semantic_letterbox_input", False)
        self.declare_parameter("semantic_input_size", 320)
        self.declare_parameter("semantic_center_crop_fraction", 0.80)
        self.declare_parameter("person_acquire_score_threshold", 0.18)
        self.declare_parameter("person_acquire_min_bbox_height_fraction", 0.30)
        self.declare_parameter("person_safety_min_bbox_height_fraction", 0.28)
        self.declare_parameter("human_distance_history_size", 5)
        self.declare_parameter("human_distance_bbox_jump_ratio", 0.22)
        self.declare_parameter("human_distance_jump_confirm_frames", 2)
        self.declare_parameter("human_distance_alpha_approach", 0.65)
        self.declare_parameter("human_distance_alpha_recede", 0.25)
        self.declare_parameter("cpu_threads", 1)
        self.declare_parameter("enabled", True)
        self.declare_parameter("torch_hub_dir", "models/torch")

        self.score_threshold = float(self.get_parameter("score_threshold").value)
        self.strict_score_threshold = float(self.get_parameter("strict_score_threshold").value)
        self.close_range_score_threshold = float(
            self.get_parameter("close_range_score_threshold").value)
        self.close_range_min_bbox_height_fraction = float(
            self.get_parameter("close_range_min_bbox_height_fraction").value)
        self.person_motor_conflict_iou = float(
            self.get_parameter("person_motor_conflict_iou").value)
        self.person_motor_conflict_area_ratio_min = float(
            self.get_parameter("person_motor_conflict_area_ratio_min").value)
        self.person_motor_min_score_margin = float(
            self.get_parameter("person_motor_min_score_margin").value)
        self.same_class_nms_iou = float(
            self.get_parameter("same_class_nms_iou").value)
        self.same_class_nms_iom = float(
            self.get_parameter("same_class_nms_iom").value)

        self.score_threshold = min(0.95, max(0.05, self.score_threshold))
        self.close_range_score_threshold = min(
            0.95, max(self.score_threshold, self.close_range_score_threshold))
        self.strict_score_threshold = min(
            0.95, max(self.close_range_score_threshold, self.strict_score_threshold))
        self.close_range_min_bbox_height_fraction = min(
            0.80, max(0.05, self.close_range_min_bbox_height_fraction))
        self.person_motor_conflict_iou = min(
            0.95, max(0.20, self.person_motor_conflict_iou))
        self.person_motor_conflict_area_ratio_min = min(
            1.0, max(0.20, self.person_motor_conflict_area_ratio_min))
        self.person_motor_min_score_margin = min(
            0.50, max(0.0, self.person_motor_min_score_margin))
        self.same_class_nms_iou = min(0.95, max(0.05, self.same_class_nms_iou))
        self.same_class_nms_iom = min(1.0, max(0.20, self.same_class_nms_iom))
        self.period = 1.0 / max(0.1, float(self.get_parameter("inference_hz").value))
        self.semantic_letterbox_input = bool(
            self.get_parameter("semantic_letterbox_input").value)
        self.semantic_input_size = max(
            256, min(640, int(self.get_parameter("semantic_input_size").value)))
        self.semantic_center_crop_fraction = max(
            0.60, min(1.0, float(self.get_parameter("semantic_center_crop_fraction").value)))
        self.person_acquire_score_threshold = max(
            self.score_threshold, min(self.strict_score_threshold,
            float(self.get_parameter("person_acquire_score_threshold").value)))
        self.person_acquire_min_bbox_height_fraction = max(
            0.05, min(0.40,
            float(self.get_parameter("person_acquire_min_bbox_height_fraction").value)))
        self.person_safety_min_bbox_height_fraction = max(
            0.10, min(0.50,
            float(self.get_parameter("person_safety_min_bbox_height_fraction").value)))
        self.human_distance_history_size = max(
            3, min(11, int(self.get_parameter("human_distance_history_size").value)))
        self.human_distance_bbox_jump_ratio = max(
            0.05, min(0.60, float(self.get_parameter("human_distance_bbox_jump_ratio").value)))
        self.human_distance_jump_confirm_frames = max(
            1, min(4, int(self.get_parameter("human_distance_jump_confirm_frames").value)))
        self.human_distance_alpha_approach = max(
            0.20, min(1.0, float(self.get_parameter("human_distance_alpha_approach").value)))
        self.human_distance_alpha_recede = max(
            0.05, min(self.human_distance_alpha_approach,
            float(self.get_parameter("human_distance_alpha_recede").value)))
        self.enabled = bool(self.get_parameter("enabled").value)
        self.torch = None
        self.model = None
        self.backend_error = ""
        self.raw_detections = []
        self.raw_image_width = 1280
        self.raw_image_height = 720
        self.raw_stamp = 0.0
        self.raw_subscription = self.create_subscription(
            String, "/perception/raw_detections", self.on_raw_detections, qos_profile_sensor_data
        )
        self.final_trial_guard_subscription = self.create_subscription(
            String, str(self.get_parameter("obstacle_memory_guard_topic").value), self.on_final_trial_guard, 10
        )
        self.publisher = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), 10)
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)
        self.object_points_publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("object_points_topic").value), 10)
        self.semantic_clearing_publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter("semantic_clearing_points_topic").value), 10)
        self.boundary_subscription = self.create_subscription(
            PointCloud2,
            str(self.get_parameter("boundary_points_topic").value),
            self.on_boundary_points,
            qos_profile_sensor_data,
        )
        self.metric_frame_id = str(self.get_parameter("metric_frame_id").value)
        self.points_per_box = max(3, min(21, int(self.get_parameter("points_per_box").value)))
        self.obstacle_forward_min_m = float(self.get_parameter("obstacle_forward_min_m").value)
        self.obstacle_forward_max_m = float(self.get_parameter("obstacle_forward_max_m").value)
        self.obstacle_max_abs_left_m = float(self.get_parameter("obstacle_max_abs_left_m").value)
        self.fallback_metric_max_abs_left_m = float(
            self.get_parameter("fallback_metric_max_abs_left_m").value)
        self.minimum_obstacle_width_m = float(self.get_parameter("minimum_obstacle_width_m").value)
        self.person_bbox_distance_intercept_m = float(
            self.get_parameter("person_bbox_distance_intercept_m").value)
        self.person_bbox_distance_slope_m = float(
            self.get_parameter("person_bbox_distance_slope_m").value)
        self.person_bbox_distance_min_m = float(
            self.get_parameter("person_bbox_distance_min_m").value)
        self.person_bbox_distance_max_m = float(
            self.get_parameter("person_bbox_distance_max_m").value)
        self.human_clear_confirm_misses = max(
            1, int(self.get_parameter("human_clear_confirm_misses").value))
        self.obstacle_memory_enabled = bool(
            self.get_parameter("obstacle_memory_enabled").value)
        self.obstacle_memory_guard_topic = str(
            self.get_parameter("obstacle_memory_guard_topic").value)
        self.obstacle_memory_width_m = max(
            self.minimum_obstacle_width_m,
            min(1.20, float(self.get_parameter("obstacle_memory_width_m").value)))
        self.obstacle_memory_release_behind_m = max(
            0.40, min(2.0, float(self.get_parameter("obstacle_memory_release_behind_m").value)))
        self.obstacle_memory_release_confirm_updates = max(
            2, min(10, int(self.get_parameter("obstacle_memory_release_confirm_updates").value)))
        self.human_detection_hold_sec = max(
            0.0, min(2.5, float(self.get_parameter("human_detection_hold_sec").value)))
        self.human_track_continue_min_score = max(
            0.05, min(0.30, float(self.get_parameter("human_track_continue_min_score").value)))
        self.human_track_iou_min = max(
            0.0, min(0.80, float(self.get_parameter("human_track_iou_min").value)))
        self.human_track_center_distance_max = max(
            0.02, min(0.50, float(self.get_parameter("human_track_center_distance_max").value)))
        self.human_track_height_ratio_min = max(
            0.20, min(1.0, float(self.get_parameter("human_track_height_ratio_min").value)))
        self.human_track_max_without_strong_sec = max(
            0.5, min(5.0, float(self.get_parameter("human_track_max_without_strong_sec").value)))
        self.semantic_watchdog_enabled = bool(
            self.get_parameter("semantic_watchdog_enabled").value)
        self.semantic_watchdog_inference_timeout_sec = max(
            1.0, float(self.get_parameter("semantic_watchdog_inference_timeout_sec").value))
        self.semantic_watchdog_output_timeout_sec = max(
            1.5, float(self.get_parameter("semantic_watchdog_output_timeout_sec").value))
        self.semantic_watchdog_upstream_fresh_sec = max(
            0.5, float(self.get_parameter("semantic_watchdog_upstream_fresh_sec").value))
        self.semantic_watchdog_startup_grace_sec = max(
            3.0, float(self.get_parameter("semantic_watchdog_startup_grace_sec").value))
        self.human_clear_burst_frames = max(
            1, int(self.get_parameter("human_clear_burst_frames").value))
        self.human_clear_half_width_m = max(
            0.20, float(self.get_parameter("human_clear_half_width_m").value))
        self.human_clear_extra_forward_m = max(
            0.20, float(self.get_parameter("human_clear_extra_forward_m").value))
        self.human_clear_ray_count = max(
            3, min(31, int(self.get_parameter("human_clear_ray_count").value)))
        self.human_clear_history_size = max(
            4, min(64, int(self.get_parameter("human_clear_history_size").value)))
        self.human_clear_motion_threshold_m = max(
            0.05, min(1.0, float(self.get_parameter("human_clear_motion_threshold_m").value)))
        self.human_remark_delay_sec = max(
            0.02, min(0.30, float(self.get_parameter("human_remark_delay_sec").value)))
        self.human_remark_timer_hz = max(
            5.0, min(50.0, float(self.get_parameter("human_remark_timer_hz").value)))
        self.fallback_full_costmap_clear_enabled = bool(
            self.get_parameter("fallback_full_costmap_clear_enabled").value)
        self.fallback_full_costmap_clear_cooldown_sec = max(
            0.5, float(self.get_parameter("fallback_full_costmap_clear_cooldown_sec").value))
        self.fallback_full_clear_confirm_frames = max(
            2, min(10, int(self.get_parameter("fallback_full_clear_confirm_frames").value)))
        self.boundary_fresh_timeout_sec = max(
            0.2, float(self.get_parameter("boundary_fresh_timeout_sec").value))
        self.human_clear_stale_timeout_sec = max(
            0.50, float(self.get_parameter("human_clear_stale_timeout_sec").value))
        self.human_clear_watchdog_hz = max(
            1.0, min(20.0, float(self.get_parameter("human_clear_watchdog_hz").value)))
        self.last_human_metric = []
        self.last_valid_human_detection = None
        self.last_valid_human_seen_monotonic = 0.0
        self.last_strong_human_seen_monotonic = 0.0
        self.human_distance_history = []
        self.human_bbox_height_history = []
        self.human_stable_forward_m = None
        self.human_stable_bbox_height = None
        self.human_bbox_jump_pending = None
        self.human_bbox_jump_pending_count = 0
        self.human_track_promoted_count = 0
        self.human_temporal_hold_count = 0
        self.human_clear_history = []
        self.human_miss_count = 0
        self.human_clear_burst_remaining = 0
        self.motion_clear_this_cycle = False
        self.pending_remark_cloud = None
        self.pending_remark_due = 0.0
        self.last_input_frame_monotonic = 0.0
        self.last_semantic_publish_monotonic = 0.0
        self.semantic_publish_count = 0
        self.semantic_watchdog_started_monotonic = time.monotonic()
        self.inference_in_progress = False
        self.inference_started_monotonic = 0.0
        self.human_stale_clear_count = 0
        self.final_trial_mode = "IDLE"
        self.obstacle_memory_trial_active = False
        self.obstacle_memory_odom = None
        self.obstacle_memory_score = 0.0
        self.obstacle_memory_latched_monotonic = 0.0
        self.obstacle_memory_release_count = 0
        self.obstacle_memory_released_for_trial = False
        self.last_boundary_count = 0
        self.last_boundary_update_monotonic = 0.0
        self.last_nav2_obstacle_count = 0
        self.no_obstacle_frame_count = 0
        self.no_obstacle_clear_armed = True
        self.pending_full_clear_reason = ""
        self.last_full_costmap_clear_request = 0.0
        self.full_costmap_clear_request_count = 0
        self.subscription = self.create_subscription(
            Image,
            str(self.get_parameter("input_topic").value),
            self.on_frame,
            qos_profile_sensor_data,
        )
        self.metric_parameter_names = [
            "ground_calibration_width", "ground_calibration_height",
            "ground_src_points", "ground_dst_points",
            "ground_canvas_width", "ground_canvas_height",
            "ground_origin_x_px", "ground_origin_y_px",
            "ground_meters_per_pixel_x", "ground_meters_per_pixel_y",
            "metric_forward_offset_m", "metric_lateral_offset_m",
            "obstacle_distance_calibration_coefficients",
        ]
        self.metric_config = None
        self.metric_request_inflight = False
        self.metric_client = self.create_client(GetParameters, "/perception/get_parameters")
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, str(self.get_parameter("local_costmap_clear_service").value))
        self.metric_timer = self.create_timer(2.0, self.refresh_metric_parameters)
        self.human_clear_watchdog_timer = self.create_timer(
            1.0 / self.human_clear_watchdog_hz, self.human_clear_watchdog)
        self.human_remark_timer = self.create_timer(
            1.0 / self.human_remark_timer_hz, self.publish_pending_remark)

        self.last_inference = 0.0
        self.last_status = 0.0
        if self.enabled:
            try:
                import torch
                from torchvision.models.detection import (
                    SSDLite320_MobileNet_V3_Large_Weights,
                    ssdlite320_mobilenet_v3_large,
                )
                threads = max(1, int(self.get_parameter("cpu_threads").value))
                torch.set_num_threads(threads)
                torch.set_num_interop_threads(1)
                root = Path(os.environ.get("AGV_ROOT", str(Path.home() / "agv"))).expanduser().resolve()
                configured_hub = Path(str(self.get_parameter("torch_hub_dir").value)).expanduser()
                hub_dir = configured_hub if configured_hub.is_absolute() else root / configured_hub
                torch.hub.set_dir(str(hub_dir))
                checkpoint = os.path.join(
                    str(hub_dir), "checkpoints",
                    "ssdlite320_mobilenet_v3_large_coco-a79551df.pth")
                if not os.path.isfile(checkpoint):
                    raise RuntimeError("checkpoint missing: " + checkpoint)
                weights = SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
                self.model = ssdlite320_mobilenet_v3_large(weights=weights).eval()
                self.torch = torch
            except Exception as exc:
                self.backend_error = str(exc)
                self.enabled = False
                self.get_logger().warning(
                    "Semantic detector backend unavailable; node remains alive and fail-closed: " + self.backend_error)
                self.publish_status("BACKEND_UNAVAILABLE", error=self.backend_error)
        if self.enabled:
            self.get_logger().info(
                "COCO semantic detector ready: internal classes 0=person, 2=car, 3=motorcycle; "
                f"model_floor={self.score_threshold:.2f} strict={self.strict_score_threshold:.2f} "
                f"close={self.close_range_score_threshold:.2f} "
                f"close_bbox_h>={self.close_range_min_bbox_height_fraction:.2f} "
                f"rate={1.0/self.period:.2f} Hz")
        else:
            self.get_logger().info("Semantic detector disabled/unavailable; no inference will run")
        self.refresh_metric_parameters()
        self.semantic_watchdog_thread = threading.Thread(
            target=self.semantic_watchdog_loop,
            name="semantic_watchdog",
            daemon=True,
        )
        self.semantic_watchdog_thread.start()

    def semantic_watchdog_loop(self):
        while True:
            time.sleep(0.25)
            if not self.semantic_watchdog_enabled or not self.enabled:
                continue
            now = time.monotonic()
            if now - self.semantic_watchdog_started_monotonic < self.semantic_watchdog_startup_grace_sec:
                continue
            if self.inference_in_progress:
                age = now - self.inference_started_monotonic
                if age >= self.semantic_watchdog_inference_timeout_sec:
                    self.get_logger().fatal(
                        f"Semantic inference watchdog timeout {age:.2f}s; respawn node")
                    os._exit(87)
            raw_age = now - self.raw_stamp if self.raw_stamp > 0.0 else float("inf")
            input_age = (now - self.last_input_frame_monotonic
                         if self.last_input_frame_monotonic > 0.0 else float("inf"))
            semantic_age = (now - self.last_semantic_publish_monotonic
                            if self.last_semantic_publish_monotonic > 0.0 else float("inf"))
            input_fresh = input_age <= self.semantic_watchdog_upstream_fresh_sec

            # Raw YOLO detections are produced inside the C++ node and may stay
            # fresh while the Python Image subscription is starved. Restarting
            # semantic in that situation only creates a respawn loop. Use the
            # actual Image callback heartbeat as the watchdog authority.
            if self.semantic_publish_count > 0:
                if input_fresh and semantic_age >= self.semantic_watchdog_output_timeout_sec:
                    self.get_logger().error(
                        f"Semantic output stale {semantic_age:.2f}s while image input fresh "
                        f"{input_age:.2f}s (raw {raw_age:.2f}s); respawn node")
                    os._exit(88)
            else:
                first_output_deadline = (
                    self.semantic_watchdog_startup_grace_sec +
                    self.semantic_watchdog_output_timeout_sec)
                if (input_fresh and
                        now - self.semantic_watchdog_started_monotonic >= first_output_deadline):
                    self.get_logger().error(
                        f"Semantic first output missing with image input fresh "
                        f"{input_age:.2f}s after {first_output_deadline:.2f}s; respawn node")
                    os._exit(88)

    def refresh_metric_parameters(self):
        if self.metric_request_inflight or not self.metric_client.service_is_ready():
            return
        request = GetParameters.Request()
        request.names = self.metric_parameter_names
        self.metric_request_inflight = True
        future = self.metric_client.call_async(request)
        future.add_done_callback(self.on_metric_parameters)

    @staticmethod
    def _parameter_value(value):
        if value.type == ParameterType.PARAMETER_INTEGER:
            return int(value.integer_value)
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return float(value.double_value)
        if value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return [float(v) for v in value.double_array_value]
        return None

    def on_metric_parameters(self, future):
        self.metric_request_inflight = False
        try:
            response = future.result()
            if response is None or len(response.values) != len(self.metric_parameter_names):
                return
            cfg = {name: self._parameter_value(value)
                   for name, value in zip(self.metric_parameter_names, response.values)}
            required = ("ground_calibration_width", "ground_calibration_height",
                        "ground_src_points", "ground_dst_points",
                        "ground_canvas_width", "ground_canvas_height",
                        "ground_origin_x_px", "ground_origin_y_px",
                        "ground_meters_per_pixel_x", "ground_meters_per_pixel_y",
                        "metric_forward_offset_m", "metric_lateral_offset_m",
                        "obstacle_distance_calibration_coefficients")
            if any(cfg.get(name) is None for name in required):
                return
            if len(cfg["ground_src_points"]) != 8 or len(cfg["ground_dst_points"]) != 8:
                return
            self.metric_config = cfg
        except Exception as exc:
            self.get_logger().warning(f"Metric parameter refresh failed: {exc}")

    def project_semantic_bottom(self, u, v, width, height):
        cfg = self.metric_config
        if not cfg or width <= 0 or height <= 0:
            return None
        cal_w = max(1.0, float(cfg["ground_calibration_width"]))
        cal_h = max(1.0, float(cfg["ground_calibration_height"]))
        src = np.asarray(cfg["ground_src_points"], dtype=np.float32).reshape(4, 2).copy()
        src[:, 0] *= float(width) / cal_w
        src[:, 1] *= float(height) / cal_h
        dst = np.asarray(cfg["ground_dst_points"], dtype=np.float32).reshape(4, 2)
        try:
            matrix = cv2.getPerspectiveTransform(src, dst)
            point = np.asarray([[[float(u), float(v)]]], dtype=np.float32)
            projected = cv2.perspectiveTransform(point, matrix)[0, 0]
        except Exception:
            return None
        x, y = float(projected[0]), float(projected[1])
        if not np.isfinite(x) or not np.isfinite(y):
            return None
        if x < 0.0 or x >= float(cfg["ground_canvas_width"]) or y < 0.0 or y >= float(cfg["ground_canvas_height"]):
            return None
        forward = ((float(cfg["ground_origin_y_px"]) - y) *
                   float(cfg["ground_meters_per_pixel_y"]) + float(cfg["metric_forward_offset_m"]))
        left = ((float(cfg["ground_origin_x_px"]) - x) *
                float(cfg["ground_meters_per_pixel_x"]) + float(cfg["metric_lateral_offset_m"]))
        if not np.isfinite(forward) or not np.isfinite(left):
            return None
        return {"raw_forward_m": float(forward), "raw_left_m": float(left),
                "metric_match_mode": "semantic_bottom_homography", "metric_match_count": 1}

    def legacy_affine_semantic_forward(self, class_id, raw_forward_m):
        """Preserve the old homography-affine result for audit/report comparison."""
        if not isinstance(raw_forward_m, (int, float)) or not np.isfinite(float(raw_forward_m)):
            return None
        cfg = self.metric_config or {}
        coeff = cfg.get("obstacle_distance_calibration_coefficients")
        if not isinstance(coeff, list) or len(coeff) != 4:
            return float(raw_forward_m)
        scale, bias = 1.0, 0.0
        if int(class_id) == 0:
            scale, bias = float(coeff[0]), float(coeff[1])
        elif int(class_id) == 3:
            scale, bias = float(coeff[2]), float(coeff[3])
        corrected = scale * float(raw_forward_m) + bias
        return float(corrected) if np.isfinite(corrected) else float(raw_forward_m)

    def person_bbox_forward(self, bbox_height_fraction):
        """1-5 m HUMAN calibration from the latest multi-orientation BAB IV dataset."""
        if not isinstance(bbox_height_fraction, (int, float)):
            return None
        h = float(bbox_height_fraction)
        if not np.isfinite(h) or h <= 0.0:
            return None
        distance = (self.person_bbox_distance_intercept_m +
                    self.person_bbox_distance_slope_m * h)
        if not np.isfinite(distance):
            return None
        return float(np.clip(
            distance, self.person_bbox_distance_min_m, self.person_bbox_distance_max_m))

    def calibrated_semantic_forward(self, class_id, raw_forward_m, bbox_height_fraction=None):
        # HUMAN uses bbox-height calibration because the ground homography saturates
        # at roughly 2 m in the 1-3 m near field. Other classes keep the legacy
        # metric calibration until a dedicated class dataset exists.
        if int(class_id) == 0:
            bbox_distance = self.person_bbox_forward(bbox_height_fraction)
            if bbox_distance is not None:
                return bbox_distance
        return self.legacy_affine_semantic_forward(class_id, raw_forward_m)

    def on_final_trial_guard(self, message):
        """Track the 4.5 final-trial mode without coupling normal navigation to the GUI."""
        try:
            payload = json.loads(message.data)
        except Exception:
            payload = {}
        mode = str(payload.get("mode", "STOP") or "STOP").upper()
        previous = self.final_trial_mode
        self.final_trial_mode = mode
        # OBSTACLE_GATE is the pre-motion / replan HOLD state. Treat it as the
        # same obstacle-trial family for world-memory purposes so switching
        # GATE <-> OBSTACLE never clears or relatches the protected HUMAN.
        obstacle_modes = {"OBSTACLE", "OBSTACLE_GATE"}
        entering_obstacle = mode in obstacle_modes and previous not in obstacle_modes
        leaving_obstacle = mode not in obstacle_modes and previous in obstacle_modes
        if entering_obstacle:
            self.obstacle_memory_trial_active = bool(self.obstacle_memory_enabled)
            self.obstacle_memory_odom = None
            self.obstacle_memory_score = 0.0
            self.obstacle_memory_latched_monotonic = 0.0
            self.obstacle_memory_release_count = 0
            self.obstacle_memory_released_for_trial = False
            self.pending_full_clear_reason = ""
            self.get_logger().info(
                "4.5A obstacle memory ARMED: waiting for first valid HUMAN world position")
        elif leaving_obstacle:
            if self.obstacle_memory_odom is not None:
                self.get_logger().info("4.5A obstacle memory cleared because final trial ended")
            self.obstacle_memory_trial_active = False
            self.obstacle_memory_odom = None
            self.obstacle_memory_score = 0.0
            self.obstacle_memory_latched_monotonic = 0.0
            self.obstacle_memory_release_count = 0
            self.obstacle_memory_released_for_trial = False

    def obstacle_memory_protected(self):
        return bool(
            self.obstacle_memory_enabled and
            self.obstacle_memory_trial_active and
            not self.obstacle_memory_released_for_trial and
            self.obstacle_memory_odom is not None)

    def _obstacle_memory_base_position(self):
        if self.obstacle_memory_odom is None:
            return None
        return self._odom_xy_to_base(
            self.obstacle_memory_odom[0], self.obstacle_memory_odom[1])

    def build_obstacle_memory_release_clear_cloud(self, stamp, base_xy):
        """Ray-clear the remembered world cell after the vehicle fully passes it."""
        if base_xy is None:
            return self.make_metric_cloud(stamp, [])
        forward, left = base_xy
        if not np.isfinite(forward) or not np.isfinite(left):
            return self.make_metric_cloud(stamp, [])
        radius = float(np.hypot(forward, left))
        if radius < 0.05:
            return self.make_metric_cloud(stamp, [])
        ux, uy = forward / radius, left / radius
        # Extend the clearing ray beyond the remembered HUMAN cell. Offsets are
        # perpendicular to the ray so the full remembered obstacle width clears.
        end_forward = forward + 0.80 * ux
        end_left = left + 0.80 * uy
        px, py = -uy, ux
        half = 0.5 * self.obstacle_memory_width_m + 0.15
        points = []
        for offset in np.linspace(-half, half, max(7, self.human_clear_ray_count // 2)):
            points.append((
                float(end_forward + px * offset),
                float(end_left + py * offset),
                0.20, 0.0))
        return self.make_metric_cloud(stamp, points)

    def update_obstacle_memory(self, detections, stamp):
        """Latch a static 4.5A HUMAN in odom and release only after it is behind the ADV."""
        if not self.obstacle_memory_trial_active or self.obstacle_memory_released_for_trial:
            return
        if self.obstacle_memory_odom is None:
            candidates = []
            for detection in detections:
                if int(detection.get("class_id", -1)) != 0:
                    continue
                if not detection.get("nav2_eligible", False):
                    continue
                if detection.get("temporal_hold", False):
                    continue
                try:
                    forward = float(detection.get("nav2_forward_m"))
                    left = float(detection.get("raw_left_m"))
                    score = float(detection.get("score", 0.0))
                    bbox_h = float(detection.get("bbox_height_fraction", 0.0) or 0.0)
                except (TypeError, ValueError):
                    continue
                if (not np.isfinite(forward) or not np.isfinite(left) or
                        forward < self.obstacle_forward_min_m or
                        forward > self.obstacle_forward_max_m or
                        abs(left) > self.obstacle_max_abs_left_m):
                    continue
                candidates.append((bbox_h, score, forward, left))
            if candidates:
                # The intended 4.5A obstacle is the dominant/closest HUMAN in
                # the trial ROI. Largest bbox is more robust than raw confidence.
                _, score, forward, left = max(candidates, key=lambda item: (item[0], item[1]))
                odom_xy = self._base_xy_to_odom(forward, left)
                if odom_xy is not None:
                    self.obstacle_memory_odom = (float(odom_xy[0]), float(odom_xy[1]))
                    self.obstacle_memory_score = float(score)
                    self.obstacle_memory_latched_monotonic = time.monotonic()
                    self.obstacle_memory_release_count = 0
                    self.pending_full_clear_reason = ""
                    self.get_logger().info(
                        f"4.5A obstacle memory LATCH odom=({odom_xy[0]:.2f},"
                        f"{odom_xy[1]:.2f}) initial_base=({forward:.2f},{left:.2f})")

        if self.obstacle_memory_odom is None:
            return
        base_xy = self._obstacle_memory_base_position()
        if base_xy is None:
            return
        forward, _ = base_xy
        if forward <= -self.obstacle_memory_release_behind_m:
            self.obstacle_memory_release_count += 1
        else:
            self.obstacle_memory_release_count = 0
        if self.obstacle_memory_release_count >= self.obstacle_memory_release_confirm_updates:
            clear_cloud = self.build_obstacle_memory_release_clear_cloud(stamp, base_xy)
            if clear_cloud.width > 0 and rclpy.ok():
                self.semantic_clearing_publisher.publish(clear_cloud)
            old = self.obstacle_memory_odom
            self.obstacle_memory_odom = None
            self.obstacle_memory_released_for_trial = True
            self.obstacle_memory_release_count = 0
            self.get_logger().info(
                f"4.5A obstacle memory RELEASE after pass: odom=({old[0]:.2f},"
                f"{old[1]:.2f}), obstacle {forward:.2f} m behind base")

    def obstacle_memory_points(self):
        """Return remembered HUMAN points in the *current* base frame.
        None means TF is unavailable and the candidate cloud must not refresh,
        causing trajectory safety to fail closed instead of driving blind.
        """
        if not self.obstacle_memory_protected():
            return []
        base_xy = self._obstacle_memory_base_position()
        if base_xy is None:
            return None
        forward, center_left = base_xy
        if not np.isfinite(forward) or not np.isfinite(center_left):
            return None
        half = 0.5 * self.obstacle_memory_width_m
        points = []
        for lateral in np.linspace(
                center_left - half, center_left + half, self.points_per_box):
            points.append((
                float(forward), float(lateral), 0.20,
                max(0.20, float(self.obstacle_memory_score))))
        return points

    def build_object_cloud(self, detections, stamp, width, height):
        points = []
        memory_points = self.obstacle_memory_points()
        if memory_points is None:
            # TF failure while a world-fixed obstacle is protected: do not
            # refresh the obstacle stream. The safety supervisor will STOP on
            # source timeout rather than interpreting an empty cloud as clear.
            return None
        memory_replaces_human = len(memory_points) > 0
        for detection in detections:
            class_id = int(detection.get("class_id", -1))
            # Formal 4.5A tests isolate HUMAN avoidance. Car/motorcycle may
            # remain visible in semantic detections/GUI for audit, but while an
            # obstacle trial is active they must not contaminate the planning
            # cloud or create a false lethal cell around the ADV.
            if self.obstacle_memory_trial_active and class_id != 0:
                continue
            if memory_replaces_human and class_id == 0:
                continue
            if detection.get("nav2_eligible") is False:
                continue
            try:
                forward = float(detection.get("nav2_forward_m"))
                center_left = float(detection.get("raw_left_m"))
                score = float(detection.get("score", 0.0))
            except (TypeError, ValueError):
                continue
            if not np.isfinite(forward) or not np.isfinite(center_left):
                continue
            if forward < self.obstacle_forward_min_m or forward > self.obstacle_forward_max_m:
                continue
            if abs(center_left) > self.obstacle_max_abs_left_m:
                continue
            left_edge = self.project_semantic_bottom(
                float(detection.get("x1", 0.0)), float(detection.get("y2", 0.0)), width, height)
            right_edge = self.project_semantic_bottom(
                float(detection.get("x2", 0.0)), float(detection.get("y2", 0.0)), width, height)
            edge_values = []
            for edge in (left_edge, right_edge):
                if isinstance(edge, dict):
                    value = edge.get("raw_left_m")
                    if isinstance(value, (int, float)) and np.isfinite(float(value)):
                        edge_values.append(float(value))
            measured_width = abs(edge_values[1] - edge_values[0]) if len(edge_values) == 2 else 0.0
            class_name = str(detection.get("class_name", "")).lower()
            max_width = 0.55 if class_name == "person" else (1.20 if class_name == "motorcycle" else 2.20)
            nominal_width = 0.45 if class_name == "person" else (0.85 if class_name == "motorcycle" else 1.80)
            physical_width = measured_width if np.isfinite(measured_width) and measured_width > 0.0 else nominal_width
            physical_width = min(max(physical_width, self.minimum_obstacle_width_m), max_width)
            half = 0.5 * physical_width
            y_min, y_max = center_left - half, center_left + half
            y_min = max(-self.obstacle_max_abs_left_m, y_min)
            y_max = min(self.obstacle_max_abs_left_m, y_max)
            if y_max < y_min:
                continue
            for lateral in np.linspace(y_min, y_max, self.points_per_box):
                points.append((forward, float(lateral), 0.20, score))

        points.extend(memory_points)
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.metric_frame_id
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = b"".join(struct.pack("<ffff", *point) for point in points)
        return cloud

    def make_metric_cloud(self, stamp, points):
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.metric_frame_id
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = b"".join(struct.pack("<ffff", *point) for point in points)
        return cloud

    def build_targeted_human_clear_cloud(self, stamp, positions=None):
        points = []
        targets = self.last_human_metric if positions is None else positions
        for forward, center_left in targets:
            if not np.isfinite(forward) or not np.isfinite(center_left) or forward <= 0.05:
                continue
            end_forward = min(6.0, max(forward + self.human_clear_extra_forward_m, forward + 0.25))
            # Choose ray endpoints so that the rays cross the former HUMAN region
            # at the last measured forward distance, not only at the endpoint.
            scale = end_forward / max(0.05, forward)
            for target_left in np.linspace(
                    center_left - self.human_clear_half_width_m,
                    center_left + self.human_clear_half_width_m,
                    self.human_clear_ray_count):
                endpoint_left = float(target_left) * scale
                endpoint_left = float(np.clip(endpoint_left, -3.0, 3.0))
                points.append((float(end_forward), endpoint_left, 0.20, 0.0))
        return self.make_metric_cloud(stamp, points)

    def _base_pose_in_odom(self):
        try:
            tf = self.tf_buffer.lookup_transform("odom", self.metric_frame_id, Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return float(t.x), float(t.y), float(yaw)

    def _base_xy_to_odom(self, forward, left):
        pose = self._base_pose_in_odom()
        if pose is None:
            return None
        tx, ty, yaw = pose
        c = math.cos(yaw)
        s = math.sin(yaw)
        return (
            tx + c * float(forward) - s * float(left),
            ty + s * float(forward) + c * float(left),
        )

    def _odom_xy_to_base(self, x_odom, y_odom):
        pose = self._base_pose_in_odom()
        if pose is None:
            return None
        tx, ty, yaw = pose
        dx = float(x_odom) - tx
        dy = float(y_odom) - ty
        c = math.cos(yaw)
        s = math.sin(yaw)
        return (
            c * dx + s * dy,
            -s * dx + c * dy,
        )

    def remember_human_clear_positions(self, positions):
        # Persist old HUMAN anchors in odom, not base_footprint. Replaying old
        # base coordinates after heading/pose changes points to the wrong global
        # cells and is exactly what caused the stale costmap islands in BAB 4.5.
        for forward, left in positions:
            if not np.isfinite(forward) or not np.isfinite(left):
                continue
            odom_xy = self._base_xy_to_odom(forward, left)
            if odom_xy is None:
                continue
            ox, oy = odom_xy
            duplicate = any(
                np.hypot(ox - old_x, oy - old_y) < self.human_clear_motion_threshold_m
                for old_x, old_y in self.human_clear_history)
            if not duplicate:
                self.human_clear_history.append((float(ox), float(oy)))
        if len(self.human_clear_history) > self.human_clear_history_size:
            self.human_clear_history = self.human_clear_history[-self.human_clear_history_size:]

    def all_human_clear_positions(self):
        # Convert stored odom anchors back into *current* base_footprint rays so
        # raytracing crosses the exact old global cells even when robot yaw drifts.
        positions = []
        for ox, oy in self.human_clear_history:
            base_xy = self._odom_xy_to_base(ox, oy)
            if base_xy is None:
                continue
            forward, left = base_xy
            if forward > 0.05 and np.isfinite(forward) and np.isfinite(left):
                positions.append((float(forward), float(left)))
        for forward, left in self.last_human_metric:
            if not any(
                    np.hypot(forward - old_f, left - old_l) <
                    0.5 * self.human_clear_motion_threshold_m
                    for old_f, old_l in positions):
                positions.append((float(forward), float(left)))
        return positions

    def update_human_clear_state(self, detections, stamp):
        self.motion_clear_this_cycle = False
        if self.obstacle_memory_protected():
            # 4.5A owns this HUMAN mark until geometric pass confirmation.
            self.human_miss_count = 0
            self.human_clear_burst_remaining = 0
            return
        humans = []
        for detection in detections:
            if int(detection.get("class_id", -1)) != 0 or not detection.get("nav2_eligible", False):
                continue
            try:
                forward = float(detection.get("nav2_forward_m"))
                left = float(detection.get("raw_left_m"))
            except (TypeError, ValueError):
                continue
            if np.isfinite(forward) and np.isfinite(left):
                humans.append((forward, left))

        if humans:
            # Anchor-based motion clearing: do not ray-clear every inference because
            # clearing and marking arrive on different topics and could race. Keep
            # one anchor while jitter stays small; once cumulative movement exceeds
            # the threshold, clear the old anchor and move it to the new position.
            if not self.last_human_metric:
                self.last_human_metric = humans
            else:
                moved = len(humans) != len(self.last_human_metric)
                if not moved:
                    for old_f, old_l in self.last_human_metric:
                        nearest = min(
                            np.hypot(old_f - new_f, old_l - new_l)
                            for new_f, new_l in humans)
                        if nearest >= self.human_clear_motion_threshold_m:
                            moved = True
                            break
                if moved:
                    self.remember_human_clear_positions(self.last_human_metric)
                    clear_cloud = self.build_targeted_human_clear_cloud(
                        stamp, self.last_human_metric)
                    if clear_cloud.width > 0:
                        self.semantic_clearing_publisher.publish(clear_cloud)
                        self.motion_clear_this_cycle = True
                    self.last_human_metric = humans
            self.human_miss_count = 0
            self.human_clear_burst_remaining = 0
            return

        if not self.last_human_metric and not self.human_clear_history:
            return
        if self.last_human_metric:
            self.remember_human_clear_positions(self.last_human_metric)
        self.human_miss_count += 1
        if self.human_miss_count == self.human_clear_confirm_misses:
            self.human_clear_burst_remaining = self.human_clear_burst_frames
        if self.human_clear_burst_remaining > 0:
            targets = self.all_human_clear_positions()
            clear_cloud = self.build_targeted_human_clear_cloud(stamp, targets)
            if clear_cloud.width > 0:
                self.semantic_clearing_publisher.publish(clear_cloud)
            self.human_clear_burst_remaining -= 1
            if self.human_clear_burst_remaining == 0:
                # Do not keep stale metric coordinates/history after the clearing burst.
                self.last_human_metric = []
                self.human_clear_history = []
                self.human_miss_count = 0

    def publish_pending_remark(self):
        if self.pending_remark_cloud is None:
            return
        if time.monotonic() < self.pending_remark_due:
            return
        cloud = self.pending_remark_cloud
        self.pending_remark_cloud = None
        self.pending_remark_due = 0.0
        cloud.header.stamp = self.get_clock().now().to_msg()
        if rclpy.ok():
            self.object_points_publisher.publish(cloud)

    def queue_full_costmap_clear(self, reason):
        if self.obstacle_memory_protected():
            self.pending_full_clear_reason = ""
            return
        if not self.fallback_full_costmap_clear_enabled:
            return
        self.pending_full_clear_reason = str(reason)

    def process_pending_full_costmap_clear(self):
        if self.obstacle_memory_protected():
            self.pending_full_clear_reason = ""
            return
        if not self.pending_full_clear_reason:
            return
        now = time.monotonic()
        if now - self.last_full_costmap_clear_request < self.fallback_full_costmap_clear_cooldown_sec:
            return
        # Never wipe the full local map while another semantic obstacle is active.
        if self.last_nav2_obstacle_count > 0:
            self.pending_full_clear_reason = ""
            return
        # Full clear is only permitted when the separate drivable-boundary source
        # is fresh and explicitly empty. This avoids transiently deleting a live
        # lane virtual wall.
        if self.last_boundary_update_monotonic <= 0.0:
            return
        if now - self.last_boundary_update_monotonic > self.boundary_fresh_timeout_sec:
            return
        if self.last_boundary_count > 0:
            self.pending_full_clear_reason = ""
            return
        if not self.clear_local_costmap_client.service_is_ready():
            return
        reason = self.pending_full_clear_reason
        self.pending_full_clear_reason = ""
        self.last_full_costmap_clear_request = now
        self.full_costmap_clear_request_count += 1
        future = self.clear_local_costmap_client.call_async(ClearEntireCostmap.Request())
        def _done(fut):
            try:
                fut.result()
                self.get_logger().warning(
                    f"Full local costmap clear completed: {reason}")
            except Exception as exc:
                self.get_logger().error(
                    f"Full local costmap clear failed ({reason}): {exc}")
        future.add_done_callback(_done)

    def human_clear_watchdog(self):
        """Clear stale HUMAN history even if semantic callbacks stop entirely."""
        if self.obstacle_memory_protected():
            # Do not erase the world-fixed 4.5A obstacle merely because the
            # steering maneuver moved it outside the camera FOV.
            self.pending_full_clear_reason = ""
            return
        self.process_pending_full_costmap_clear()
        if not self.last_human_metric and not self.human_clear_history:
            return
        now = time.monotonic()
        stale_for = now - self.last_semantic_publish_monotonic
        if stale_for < self.human_clear_stale_timeout_sec:
            return
        if self.last_human_metric:
            self.remember_human_clear_positions(self.last_human_metric)
        if self.human_clear_burst_remaining <= 0:
            self.human_clear_burst_remaining = self.human_clear_burst_frames
        stamp = self.get_clock().now().to_msg()
        targets = self.all_human_clear_positions()
        clear_cloud = self.build_targeted_human_clear_cloud(stamp, targets)
        if clear_cloud.width > 0:
            self.semantic_clearing_publisher.publish(clear_cloud)
        self.human_clear_burst_remaining -= 1
        if self.human_clear_burst_remaining <= 0:
            self.last_human_metric = []
            self.human_clear_history = []
            self.human_miss_count = 0
            self.human_clear_burst_remaining = 0
            self.human_stale_clear_count += 1
            self.get_logger().warning(
                f"Stale semantic HUMAN history cleared after {stale_for:.2f}s without semantic updates")

    def publish_status(self, state, **extra):
        now = time.monotonic()
        if state == "OK" and now - self.last_status < 1.0:
            return
        self.last_status = now
        payload = {
            "state": state,
            "source": "torchvision_ssdlite320_coco",
            "mapping": {"person": 0, "car": 2, "motorcycle": 3},
            "metric_config_ready": self.metric_config is not None,
            **extra,
        }
        if not rclpy.ok():
            return
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.status_publisher.publish(msg)

    def on_boundary_points(self, message):
        self.last_boundary_count = int(message.width) * int(message.height)
        self.last_boundary_update_monotonic = time.monotonic()

    def on_raw_detections(self, message):
        try:
            data = json.loads(message.data)
            self.raw_detections = data.get("detections", []) if isinstance(data, dict) else []
            self.raw_image_width = int(data.get("image_width", 1280)) if isinstance(data, dict) else 1280
            self.raw_image_height = int(data.get("image_height", 720)) if isinstance(data, dict) else 720
            self.raw_stamp = time.monotonic()
        except Exception:
            self.raw_detections = []

    @staticmethod
    def iou(a, b):
        x1=max(a[0],b[0]); y1=max(a[1],b[1]); x2=min(a[2],b[2]); y2=min(a[3],b[3])
        inter=max(0.0,x2-x1)*max(0.0,y2-y1)
        if inter<=0.0: return 0.0
        aa=max(0.0,a[2]-a[0])*max(0.0,a[3]-a[1])
        bb=max(0.0,b[2]-b[0])*max(0.0,b[3]-b[1])
        return inter/max(1e-9,aa+bb-inter)

    @staticmethod
    def _box_area(detection):
        try:
            return max(0.0, float(detection["x2"]) - float(detection["x1"])) * \
                   max(0.0, float(detection["y2"]) - float(detection["y1"]))
        except Exception:
            return 0.0

    @staticmethod
    def intersection_over_min_area(a, b):
        x1=max(float(a["x1"]),float(b["x1"])); y1=max(float(a["y1"]),float(b["y1"]))
        x2=min(float(a["x2"]),float(b["x2"])); y2=min(float(a["y2"]),float(b["y2"]))
        inter=max(0.0,x2-x1)*max(0.0,y2-y1)
        if inter<=0.0:
            return 0.0
        aa=max(1e-9,(float(a["x2"])-float(a["x1"]))*(float(a["y2"])-float(a["y1"])))
        bb=max(1e-9,(float(b["x2"])-float(b["x1"]))*(float(b["y2"])-float(b["y1"])))
        return inter/min(aa,bb)

    def person_continuity_rank(self, detection):
        score = float(detection.get("score", 0.0) or 0.0)
        if int(detection.get("class_id", -1)) != 0 or self.last_valid_human_detection is None:
            return (0, 0.0, 0.0, 0.0, score)
        try:
            last = self.last_valid_human_detection
            box = (float(detection["x1"]), float(detection["y1"]),
                   float(detection["x2"]), float(detection["y2"]))
            last_box = (float(last["x1"]), float(last["y1"]),
                        float(last["x2"]), float(last["y2"]))
            overlap = self.iou(box, last_box)
            h = max(1.0, box[3] - box[1])
            last_h = max(1.0, last_box[3] - last_box[1])
            height_ratio = min(h, last_h) / max(h, last_h)
            cx = 0.5 * (box[0] + box[2]); cy = 0.5 * (box[1] + box[3])
            lcx = 0.5 * (last_box[0] + last_box[2]); lcy = 0.5 * (last_box[1] + last_box[3])
            center_distance = float(np.hypot(cx - lcx, cy - lcy)) / max(1.0, last_h)
            associated = bool(
                overlap >= self.human_track_iou_min or
                (center_distance <= 1.25 and
                 height_ratio >= self.human_track_height_ratio_min))
            return (
                1 if associated else 0,
                overlap,
                height_ratio,
                -center_distance,
                score,
            )
        except Exception:
            return (0, 0.0, 0.0, 0.0, score)

    def reset_human_distance_state(self):
        self.human_distance_history = []
        self.human_bbox_height_history = []
        self.human_stable_forward_m = None
        self.human_stable_bbox_height = None
        self.human_bbox_jump_pending = None
        self.human_bbox_jump_pending_count = 0

    def stabilize_human_distance(self, detection):
        current = detection.get("bbox_calibrated_forward_m")
        bbox_h = detection.get("bbox_height_fraction")
        if not isinstance(current, (int, float)) or not np.isfinite(float(current)):
            current = detection.get("nav2_forward_m")
        if (not isinstance(current, (int, float)) or not np.isfinite(float(current)) or
                not isinstance(bbox_h, (int, float)) or not np.isfinite(float(bbox_h))):
            if self.human_stable_forward_m is not None:
                detection["stable_forward_m"] = float(self.human_stable_forward_m)
                detection["stable_bbox_height_fraction"] = self.human_stable_bbox_height
                detection["distance_filter_state"] = "HOLD_NO_MEASUREMENT"
            return detection

        current = float(current)
        bbox_h = float(bbox_h)

        if detection.get("temporal_hold", False) and self.human_stable_forward_m is not None:
            detection["stable_forward_m"] = float(self.human_stable_forward_m)
            detection["stable_bbox_height_fraction"] = self.human_stable_bbox_height
            detection["distance_filter_state"] = "TEMPORAL_HOLD"
            detection["distance_raw_for_filter_m"] = current
            return detection

        reject_jump = False
        jump_ratio = 0.0
        if self.human_stable_bbox_height is not None:
            jump_ratio = abs(bbox_h - self.human_stable_bbox_height) / max(
                0.05, abs(self.human_stable_bbox_height))
            if jump_ratio > self.human_distance_bbox_jump_ratio:
                pending = self.human_bbox_jump_pending
                if pending is not None:
                    similar = abs(bbox_h - pending) / max(0.05, abs(pending)) <= 0.12
                else:
                    similar = False
                if similar:
                    self.human_bbox_jump_pending_count += 1
                else:
                    self.human_bbox_jump_pending = bbox_h
                    self.human_bbox_jump_pending_count = 1
                reject_jump = self.human_bbox_jump_pending_count < self.human_distance_jump_confirm_frames
            else:
                self.human_bbox_jump_pending = None
                self.human_bbox_jump_pending_count = 0

        if reject_jump and self.human_stable_forward_m is not None:
            detection["stable_forward_m"] = float(self.human_stable_forward_m)
            detection["stable_bbox_height_fraction"] = self.human_stable_bbox_height
            detection["distance_filter_state"] = "BBOX_JUMP_REJECTED"
            detection["distance_bbox_jump_ratio"] = round(jump_ratio, 5)
            detection["distance_raw_for_filter_m"] = current
            return detection

        self.human_bbox_jump_pending = None
        self.human_bbox_jump_pending_count = 0
        self.human_distance_history.append(current)
        self.human_bbox_height_history.append(bbox_h)
        self.human_distance_history = self.human_distance_history[-self.human_distance_history_size:]
        self.human_bbox_height_history = self.human_bbox_height_history[-self.human_distance_history_size:]

        robust_distance = float(np.median(np.asarray(self.human_distance_history, dtype=np.float64)))
        robust_bbox_h = float(np.median(np.asarray(self.human_bbox_height_history, dtype=np.float64)))
        if self.human_stable_forward_m is None:
            stable = robust_distance
        else:
            alpha = (self.human_distance_alpha_approach
                     if robust_distance < self.human_stable_forward_m
                     else self.human_distance_alpha_recede)
            stable = alpha * robust_distance + (1.0 - alpha) * self.human_stable_forward_m

        self.human_stable_forward_m = float(np.clip(
            stable, self.person_bbox_distance_min_m, self.person_bbox_distance_max_m))
        self.human_stable_bbox_height = robust_bbox_h

        detection["stable_forward_m"] = self.human_stable_forward_m
        detection["stable_bbox_height_fraction"] = self.human_stable_bbox_height
        detection["distance_filter_state"] = "TRACK_FILTERED"
        detection["distance_bbox_jump_ratio"] = round(jump_ratio, 5)
        detection["distance_raw_for_filter_m"] = current
        detection["distance_history_count"] = len(self.human_distance_history)
        return detection

    def suppress_same_class_duplicates(self, detections):
        """Keep one runtime obstacle for overlapping boxes of the same class.

        Standard IoU alone misses a common SSDLite failure mode where a very
        large low-score box contains a correct smaller box. IoM catches that
        containment while separate side-by-side people remain independent.
        Evaluation candidates are preserved upstream; only the safety/runtime
        stream is deduplicated here.
        """
        ordered=sorted(detections,key=self.person_continuity_rank,reverse=True)
        kept=[]
        suppressed=0
        for detection in ordered:
            box=(float(detection["x1"]),float(detection["y1"]),
                 float(detection["x2"]),float(detection["y2"]))
            duplicate=False
            for primary in kept:
                if int(primary.get("class_id",-1))!=int(detection.get("class_id",-1)):
                    continue
                pbox=(float(primary["x1"]),float(primary["y1"]),
                      float(primary["x2"]),float(primary["y2"]))
                overlap=self.iou(box,pbox)
                iom=self.intersection_over_min_area(detection,primary)
                if overlap>=self.same_class_nms_iou or iom>=self.same_class_nms_iom:
                    duplicate=True
                    suppressed+=1
                    primary["same_class_duplicates_suppressed"]=int(
                        primary.get("same_class_duplicates_suppressed",0))+1
                    primary["class_resolution"]="same_class_primary"
                    break
            if not duplicate:
                kept.append(detection)
        return kept, suppressed

    def resolve_person_motor_conflicts(self, detections):
        """Reject cross-class duplicates for the same physical object.

        TorchVision can emit both COCO person and motorcycle for a single large
        close object. We only resolve pairs whose boxes have both high IoU and
        similar area. This avoids collapsing a real rider + motorcycle pair.
        If competing scores are too close, both are rejected as ambiguous so
        calibration cannot silently save the wrong semantic class.
        """
        if len(detections) < 2:
            for d in detections:
                d.setdefault("class_resolution", "single")
                d.setdefault("class_margin", None)
            return detections, {"suppressed": 0, "ambiguous": 0}

        removed = set()
        suppressed = 0
        ambiguous = 0
        for i in range(len(detections)):
            if i in removed:
                continue
            a = detections[i]
            if int(a.get("class_id", -1)) not in (0, 3):
                continue
            box_a = (float(a["x1"]), float(a["y1"]), float(a["x2"]), float(a["y2"]))
            area_a = self._box_area(a)
            for j in range(i + 1, len(detections)):
                if j in removed:
                    continue
                b = detections[j]
                class_a = int(a.get("class_id", -1))
                class_b = int(b.get("class_id", -1))
                if {class_a, class_b} != {0, 3}:
                    continue
                box_b = (float(b["x1"]), float(b["y1"]), float(b["x2"]), float(b["y2"]))
                overlap = self.iou(box_a, box_b)
                if overlap < self.person_motor_conflict_iou:
                    continue
                area_b = self._box_area(b)
                area_ratio = min(area_a, area_b) / max(1e-9, max(area_a, area_b))
                if area_ratio < self.person_motor_conflict_area_ratio_min:
                    continue

                score_a = float(a.get("score", 0.0))
                score_b = float(b.get("score", 0.0))
                margin = abs(score_a - score_b)
                if margin < self.person_motor_min_score_margin:
                    removed.add(i)
                    removed.add(j)
                    ambiguous += 1
                    a["class_resolution"] = "ambiguous_rejected"
                    b["class_resolution"] = "ambiguous_rejected"
                    break

                winner_i, loser_i = (i, j) if score_a > score_b else (j, i)
                winner = detections[winner_i]
                winner["class_resolution"] = "cross_class_winner"
                winner["class_margin"] = round(margin, 5)
                winner["competing_class_id"] = int(
                    detections[loser_i].get("class_id", -1))
                winner["competing_score"] = round(
                    float(detections[loser_i].get("score", 0.0)), 5)
                removed.add(loser_i)
                suppressed += 1
                if loser_i == i:
                    break

        filtered = []
        for index, detection in enumerate(detections):
            if index in removed:
                continue
            detection.setdefault("class_resolution", "single")
            detection.setdefault("class_margin", None)
            filtered.append(detection)
        return filtered, {"suppressed": suppressed, "ambiguous": ambiguous}

    def metric_match(self, box, width, height):
        if time.monotonic() - self.raw_stamp > 2.0 or not self.raw_detections:
            return None
        sx = self.raw_image_width / max(1.0, float(width))
        sy = self.raw_image_height / max(1.0, float(height))
        semantic = (box[0] * sx, box[1] * sy, box[2] * sx, box[3] * sy)
        matches = []
        for d in self.raw_detections:
            try:
                raw = (float(d["x1"]), float(d["y1"]), float(d["x2"]), float(d["y2"]))
            except Exception:
                continue
            x1 = max(semantic[0], raw[0]); y1 = max(semantic[1], raw[1])
            x2 = min(semantic[2], raw[2]); y2 = min(semantic[3], raw[3])
            inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
            raw_area = max(1e-9, raw[2] - raw[0]) * max(1e-9, raw[3] - raw[1])
            contained_fraction = inter / raw_area
            iou = self.iou(semantic, raw)
            # YOLOPv2 raw heads are geometry proposals and can be much smaller than
            # the full COCO semantic object.  A proposal mostly enclosed by the
            # semantic box is still the same physical obstacle even when IoU is low.
            if iou < 0.08 and contained_fraction < 0.35:
                continue
            forward = d.get("raw_forward_m")
            metric_ok = isinstance(forward, (int, float)) and np.isfinite(float(forward))
            score = float(d.get("score", 0.0) or 0.0)
            matches.append((metric_ok, contained_fraction, iou, score, d))
        if not matches:
            return None
        matches.sort(key=lambda item: (item[0], item[1], item[2], item[3]), reverse=True)
        best = dict(matches[0][4])
        finite = [item[4] for item in matches if item[0]]
        if finite:
            forwards = [float(d["raw_forward_m"]) for d in finite]
            lefts = [float(d["raw_left_m"]) for d in finite
                     if isinstance(d.get("raw_left_m"), (int, float)) and np.isfinite(float(d["raw_left_m"]))]
            best["raw_forward_m"] = float(np.median(forwards))
            best["raw_left_m"] = float(np.median(lefts)) if lefts else None
            best["metric_match_mode"] = "semantic_containment_median"
            best["metric_match_count"] = len(finite)
        return best

    def on_frame(self, message):
        if not self.enabled:
            return
        now = time.monotonic()
        self.last_input_frame_monotonic = now
        if now - self.last_inference < self.period:
            return
        self.last_inference = now

        self.inference_in_progress = True
        self.inference_started_monotonic = time.monotonic()
        try:
            height = int(message.height)
            width = int(message.width)
            step = int(message.step)
            if height <= 0 or width <= 0 or step <= 0:
                self.publish_status("DECODE_FAILED", error="invalid image dimensions")
                return
            buf = np.frombuffer(message.data, dtype=np.uint8)
            if buf.size < height * step:
                self.publish_status("DECODE_FAILED", error="short image buffer")
                return
            rows = buf[:height * step].reshape(height, step)
            encoding = str(message.encoding).lower()
            if encoding in ("bgr8", "8uc3"):
                bgr = rows[:, :width * 3].reshape(height, width, 3).copy()
            elif encoding == "rgb8":
                rgb_in = rows[:, :width * 3].reshape(height, width, 3)
                bgr = cv2.cvtColor(rgb_in, cv2.COLOR_RGB2BGR)
            elif encoding in ("mono8", "8uc1"):
                mono = rows[:, :width].reshape(height, width)
                bgr = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
            else:
                self.publish_status("UNSUPPORTED_ENCODING", encoding=encoding)
                return
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            # SSDLite320 uses a fixed 320x320 detector input. For the forward
            # safety corridor, crop the central 80% horizontally before inference:
            # the HUMAN occupies more detector pixels without adding a second pass.
            # Coordinates are remapped back to the original camera below.
            model_rgb = rgb
            preprocess_mode = "MODEL_INTERNAL"
            crop_offset_x = 0
            crop_offset_y = 0
            letterbox_scale = 1.0
            letterbox_pad_x = 0
            letterbox_pad_y = 0

            if self.semantic_center_crop_fraction < 0.999:
                crop_w = max(1, min(
                    width, int(round(width * self.semantic_center_crop_fraction))))
                crop_offset_x = max(0, (width - crop_w) // 2)
                model_rgb = rgb[:, crop_offset_x:crop_offset_x + crop_w]
                preprocess_mode = "CENTER_CROP"
            elif self.semantic_letterbox_input:
                target = int(self.semantic_input_size)
                letterbox_scale = min(
                    float(target) / max(1.0, float(width)),
                    float(target) / max(1.0, float(height)))
                resized_w = max(1, min(target, int(round(width * letterbox_scale))))
                resized_h = max(1, min(target, int(round(height * letterbox_scale))))
                resized = cv2.resize(
                    rgb, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
                model_rgb = np.full((target, target, 3), 114, dtype=np.uint8)
                letterbox_pad_x = (target - resized_w) // 2
                letterbox_pad_y = (target - resized_h) // 2
                model_rgb[
                    letterbox_pad_y:letterbox_pad_y + resized_h,
                    letterbox_pad_x:letterbox_pad_x + resized_w] = resized
                preprocess_mode = "LETTERBOX"

            tensor = self.torch.from_numpy(model_rgb).permute(2, 0, 1).float().div_(255.0)
            started = time.perf_counter()
            with self.torch.inference_mode():
                output = self.model([tensor])[0]
            inference_ms = (time.perf_counter() - started) * 1000.0
        except Exception as exc:
            self.get_logger().error(f"Semantic inference failed: {exc}")
            self.publish_status("INFERENCE_FAILED", error=str(exc))
            return
        finally:
            self.inference_in_progress = False

        detections = []
        height, width = bgr.shape[:2]
        for score_t, label_t, box_t in zip(
            output["scores"], output["labels"], output["boxes"]
        ):
            score = float(score_t)
            if score < self.score_threshold:
                break
            coco_id = int(label_t)
            if coco_id not in COCO_TO_INTERNAL:
                continue

            internal_id, class_name = COCO_TO_INTERNAL[coco_id]
            x1, y1, x2, y2 = [float(v) for v in box_t.tolist()]
            if preprocess_mode == "CENTER_CROP":
                x1 += float(crop_offset_x)
                x2 += float(crop_offset_x)
                y1 += float(crop_offset_y)
                y2 += float(crop_offset_y)
            elif preprocess_mode == "LETTERBOX":
                inv_scale = 1.0 / max(1.0e-9, float(letterbox_scale))
                x1 = (x1 - float(letterbox_pad_x)) * inv_scale
                x2 = (x2 - float(letterbox_pad_x)) * inv_scale
                y1 = (y1 - float(letterbox_pad_y)) * inv_scale
                y2 = (y2 - float(letterbox_pad_y)) * inv_scale
            x1 = max(0.0, min(x1, width - 1.0))
            x2 = max(0.0, min(x2, width - 1.0))
            y1 = max(0.0, min(y1, height - 1.0))
            y2 = max(0.0, min(y2, height - 1.0))
            if x2 <= x1 or y2 <= y1:
                continue
            metric = self.metric_match((x1, y1, x2, y2), width, height)
            raw_forward = metric.get("raw_forward_m") if isinstance(metric, dict) else None
            if not isinstance(raw_forward, (int, float)) or not np.isfinite(float(raw_forward)):
                metric = self.project_semantic_bottom((x1 + x2) * 0.5, y2, width, height)
                raw_forward = metric.get("raw_forward_m") if isinstance(metric, dict) else None
            raw_left = metric.get("raw_left_m") if isinstance(metric, dict) else None
            detections.append({
                "class_id": internal_id,
                "class_name": class_name,
                "coco_class_id": coco_id,
                "score": round(score, 5),
                "x1": round(x1, 3), "y1": round(y1, 3),
                "x2": round(x2, 3), "y2": round(y2, 3),
                "bottom_u_px": round((x1 + x2) * 0.5, 3),
                "bottom_v_px": round(y2, 3),
                "bbox_height_fraction": round((y2 - y1) / max(1.0, height), 5),
                "center_x_fraction": round(((x1 + x2) * 0.5) / max(1.0, width), 5),
                "bottom_clipped": y2 >= height - 2.0,
                "raw_forward_m": raw_forward if isinstance(raw_forward, (int, float)) else None,
                "raw_left_m": raw_left if isinstance(raw_left, (int, float)) else None,
                "metric_matched": bool(metric is not None and isinstance(raw_forward, (int, float)) and np.isfinite(float(raw_forward))),
                "metric_match_mode": metric.get("metric_match_mode") if isinstance(metric, dict) else None,
            })

        raw_candidate_count = len(detections)
        # Keep a low-floor candidate stream for controlled BAB IV evaluation.
        # Safety/runtime detections below remain protected by the existing
        # close/strict adaptive thresholds.
        evaluation_candidates = [dict(detection) for detection in detections]
        for detection in evaluation_candidates:
            class_id = detection.get("class_id", -1)
            raw_forward = detection.get("raw_forward_m")
            bbox_h = detection.get("bbox_height_fraction")
            legacy = self.legacy_affine_semantic_forward(class_id, raw_forward)
            bbox_distance = self.person_bbox_forward(bbox_h) if int(class_id) == 0 else None
            detection["legacy_affine_forward_m"] = legacy
            detection["bbox_calibrated_forward_m"] = bbox_distance
            detection["calibrated_forward_m"] = self.calibrated_semantic_forward(
                class_id, raw_forward, bbox_h)
        adaptive = []
        adaptive_rejected = 0
        for detection in detections:
            bbox_h = float(detection.get("bbox_height_fraction", 0.0) or 0.0)
            close_range = bool(
                detection.get("bottom_clipped", False) or
                bbox_h >= self.close_range_min_bbox_height_fraction)
            effective_threshold = (
                self.close_range_score_threshold if close_range
                else self.strict_score_threshold)
            person_acquire = bool(
                int(detection.get("class_id", -1)) == 0 and
                bbox_h >= self.person_acquire_min_bbox_height_fraction)
            if person_acquire:
                effective_threshold = min(
                    effective_threshold, self.person_acquire_score_threshold)
            detection["close_range_candidate"] = close_range
            detection["person_acquire_candidate"] = person_acquire
            detection["effective_score_threshold"] = round(effective_threshold, 5)
            if float(detection.get("score", 0.0)) < effective_threshold:
                adaptive_rejected += 1
                continue
            adaptive.append(detection)

        adaptive, same_class_suppressed = self.suppress_same_class_duplicates(adaptive)
        detections, conflict_stats = self.resolve_person_motor_conflicts(adaptive)
        for detection in detections:
            class_id = detection.get("class_id", -1)
            raw = detection.get("raw_forward_m")
            bbox_h = detection.get("bbox_height_fraction")
            legacy = self.legacy_affine_semantic_forward(class_id, raw)
            bbox_distance = self.person_bbox_forward(bbox_h) if int(class_id) == 0 else None
            calibrated = self.calibrated_semantic_forward(class_id, raw, bbox_h)
            detection["legacy_affine_forward_m"] = legacy
            detection["bbox_calibrated_forward_m"] = bbox_distance
            detection["calibrated_forward_m"] = calibrated
            nav2_forward = None
            nav2_source = "invalid"
            if isinstance(calibrated, (int, float)) and np.isfinite(float(calibrated)) and                     self.obstacle_forward_min_m <= float(calibrated) <= self.obstacle_forward_max_m:
                nav2_forward = float(calibrated)
                nav2_source = "bbox_calibrated" if int(class_id) == 0 and bbox_distance is not None else "calibrated"
            elif isinstance(raw, (int, float)) and np.isfinite(float(raw)) and                     self.obstacle_forward_min_m <= float(raw) <= self.obstacle_forward_max_m:
                nav2_forward = float(raw)
                nav2_source = "raw_metric_fallback"
            detection["nav2_forward_m"] = nav2_forward
            detection["nav2_distance_source"] = nav2_source
            left = detection.get("raw_left_m")
            left_valid = isinstance(left, (int, float)) and np.isfinite(float(left))
            within_full_roi = bool(
                left_valid and abs(float(left)) <= self.obstacle_max_abs_left_m)
            match_mode = str(detection.get("metric_match_mode") or "")
            fallback_edge_rejected = bool(
                match_mode == "semantic_bottom_homography" and left_valid and
                abs(float(left)) > self.fallback_metric_max_abs_left_m)
            person_size_valid = bool(
                int(class_id) != 0 or
                float(bbox_h or 0.0) >= self.person_safety_min_bbox_height_fraction)
            detection["person_safety_size_valid"] = person_size_valid
            detection["nav2_eligible"] = bool(
                nav2_forward is not None and within_full_roi and
                not fallback_edge_rejected and person_size_valid)
            if nav2_forward is None:
                detection["nav2_rejection_reason"] = "metric_invalid"
            elif not person_size_valid:
                detection["nav2_rejection_reason"] = "person_bbox_too_small"
            elif not within_full_roi:
                detection["nav2_rejection_reason"] = "outside_metric_roi"
            elif fallback_edge_rejected:
                detection["nav2_rejection_reason"] = "fallback_edge_projection"
            else:
                detection["nav2_rejection_reason"] = ""

        # Timestamp for temporal HUMAN tracking. The semantic-output
        # watchdog heartbeat is updated only after the ROS publish succeeds below.
        semantic_now = time.monotonic()

        # HUMAN stabilization has three fail-closed states:
        #   STRONG  : normal adaptive detector accepted the person.
        #   TRACKED : a low-floor person proposal is allowed only when it is
        #             geometrically associated with a recently STRONG person.
        #   HELD    : no proposal for <= human_detection_hold_sec; retain the
        #             last accepted metric position briefly.
        # A low-score proposal can never start a track by itself.
        real_humans = [
            detection for detection in detections
            if int(detection.get("class_id", -1)) == 0
            and detection.get("nav2_eligible", False)
        ]
        tracking_state = "NONE"
        strong_age = float("inf")

        if real_humans:
            best_human = max(real_humans, key=self.person_continuity_rank)
            best_human["safety_human_valid"] = True
            best_human["tracking_state"] = "STRONG"
            best_human["tracking_promoted"] = False
            best_human["temporal_hold"] = False
            best_human["temporal_hold_age_ms"] = 0
            best_human["strong_age_ms"] = 0
            self.stabilize_human_distance(best_human)
            self.last_valid_human_detection = dict(best_human)
            self.last_valid_human_seen_monotonic = semantic_now
            self.last_strong_human_seen_monotonic = semantic_now
            tracking_state = "STRONG"
            strong_age = 0.0
        elif self.last_valid_human_detection is not None:
            strong_age = (
                semantic_now - self.last_strong_human_seen_monotonic
                if self.last_strong_human_seen_monotonic > 0.0 else float("inf"))
            promoted = None
            promoted_rank = None
            last = self.last_valid_human_detection
            try:
                last_box = (
                    float(last["x1"]), float(last["y1"]),
                    float(last["x2"]), float(last["y2"]))
                last_cx = 0.5 * (last_box[0] + last_box[2])
                last_cy = 0.5 * (last_box[1] + last_box[3])
                last_h = max(1.0, last_box[3] - last_box[1])
            except Exception:
                last_box = None

            if last_box is not None and strong_age <= self.human_track_max_without_strong_sec:
                for candidate in evaluation_candidates:
                    if int(candidate.get("class_id", -1)) != 0:
                        continue
                    candidate_score = float(candidate.get("score", 0.0) or 0.0)
                    if candidate_score < self.human_track_continue_min_score:
                        continue
                    try:
                        box = (
                            float(candidate["x1"]), float(candidate["y1"]),
                            float(candidate["x2"]), float(candidate["y2"]))
                    except Exception:
                        continue
                    overlap = self.iou(last_box, box)
                    cx = 0.5 * (box[0] + box[2])
                    cy = 0.5 * (box[1] + box[3])
                    center_distance = float(np.hypot(
                        (cx - last_cx) / max(1.0, float(width)),
                        (cy - last_cy) / max(1.0, float(height))))
                    h = max(1.0, box[3] - box[1])
                    height_ratio = min(h, last_h) / max(h, last_h)
                    associated = bool(
                        overlap >= self.human_track_iou_min or
                        (center_distance <= self.human_track_center_distance_max and
                         height_ratio >= self.human_track_height_ratio_min))
                    if not associated:
                        continue

                    tracked = dict(candidate)
                    raw = tracked.get("raw_forward_m")
                    bbox_h = tracked.get("bbox_height_fraction")
                    bbox_distance = self.person_bbox_forward(bbox_h)
                    calibrated = self.calibrated_semantic_forward(0, raw, bbox_h)
                    tracked["legacy_affine_forward_m"] = self.legacy_affine_semantic_forward(0, raw)
                    tracked["bbox_calibrated_forward_m"] = bbox_distance
                    tracked["calibrated_forward_m"] = calibrated
                    nav2_forward = None
                    nav2_source = "invalid"
                    if (isinstance(calibrated, (int, float)) and
                            np.isfinite(float(calibrated)) and
                            self.obstacle_forward_min_m <= float(calibrated) <=
                            self.obstacle_forward_max_m):
                        nav2_forward = float(calibrated)
                        nav2_source = "bbox_calibrated_track"
                    elif (isinstance(raw, (int, float)) and np.isfinite(float(raw)) and
                            self.obstacle_forward_min_m <= float(raw) <=
                            self.obstacle_forward_max_m):
                        nav2_forward = float(raw)
                        nav2_source = "raw_metric_track"
                    left = tracked.get("raw_left_m")
                    left_valid = isinstance(left, (int, float)) and np.isfinite(float(left))
                    within_roi = bool(
                        left_valid and abs(float(left)) <= self.obstacle_max_abs_left_m)
                    tracked["nav2_forward_m"] = nav2_forward
                    tracked["nav2_distance_source"] = nav2_source
                    tracked_size_valid = bool(
                        float(bbox_h or 0.0) >= self.person_safety_min_bbox_height_fraction)
                    tracked["person_safety_size_valid"] = tracked_size_valid
                    tracked["nav2_eligible"] = bool(
                        nav2_forward is not None and within_roi and tracked_size_valid)
                    if not tracked["nav2_eligible"]:
                        continue

                    rank = (candidate_score, overlap, -center_distance)
                    if promoted is None or rank > promoted_rank:
                        tracked["safety_human_valid"] = True
                        tracked["tracking_state"] = "TRACKED"
                        tracked["tracking_promoted"] = True
                        tracked["tracking_iou"] = round(overlap, 5)
                        tracked["tracking_center_distance"] = round(center_distance, 5)
                        tracked["tracking_height_ratio"] = round(height_ratio, 5)
                        tracked["strong_age_ms"] = int(round(strong_age * 1000.0))
                        tracked["temporal_hold"] = False
                        tracked["temporal_hold_age_ms"] = 0
                        tracked["class_resolution"] = "temporal_track_low_score"
                        promoted = tracked
                        promoted_rank = rank

            if promoted is not None:
                self.stabilize_human_distance(promoted)
                detections.append(promoted)
                self.last_valid_human_detection = dict(promoted)
                self.last_valid_human_seen_monotonic = semantic_now
                self.human_track_promoted_count += 1
                tracking_state = "TRACKED"
            else:
                held_age = semantic_now - self.last_valid_human_seen_monotonic
                if (0.0 <= held_age <= self.human_detection_hold_sec and
                        strong_age <= self.human_track_max_without_strong_sec):
                    held = dict(self.last_valid_human_detection)
                    held["safety_human_valid"] = True
                    held["tracking_state"] = "HELD"
                    held["tracking_promoted"] = bool(
                        held.get("tracking_promoted", False))
                    held["temporal_hold"] = True
                    held["temporal_hold_age_ms"] = int(round(held_age * 1000.0))
                    held["strong_age_ms"] = int(round(strong_age * 1000.0))
                    held["class_resolution"] = "temporal_hold_last_valid"
                    self.stabilize_human_distance(held)
                    detections.append(held)
                    self.human_temporal_hold_count += 1
                    tracking_state = "HELD"
                else:
                    self.last_valid_human_detection = None
                    self.last_valid_human_seen_monotonic = 0.0
                    self.last_strong_human_seen_monotonic = 0.0
                    self.reset_human_distance_state()

        # 4.5A must remember a static obstacle in world coordinates before the
        # camera FOV rotates away during avoidance.
        self.update_obstacle_memory(detections, message.header.stamp)

        # Clearing/marking sees the stabilized safety detections. During a
        # protected 4.5A world-memory phase clearing is intentionally inhibited.
        self.update_human_clear_state(detections, message.header.stamp)

        self.last_nav2_obstacle_count = sum(
            1 for detection in detections if detection.get("nav2_eligible", False))
        if self.obstacle_memory_protected():
            self.last_nav2_obstacle_count = max(1, self.last_nav2_obstacle_count)
        if self.last_nav2_obstacle_count == 0:
            self.no_obstacle_frame_count += 1
            if (self.no_obstacle_clear_armed and
                    self.no_obstacle_frame_count >= self.fallback_full_clear_confirm_frames):
                self.queue_full_costmap_clear("semantic_no_obstacle")
                self.no_obstacle_clear_armed = False
        else:
            self.no_obstacle_frame_count = 0
            self.no_obstacle_clear_armed = True
            self.pending_full_clear_reason = ""

        payload = {
            "source": "torchvision_ssdlite320_coco",
            "image_width": width,
            "image_height": height,
            "score_threshold": self.score_threshold,
            "strict_score_threshold": self.strict_score_threshold,
            "close_range_score_threshold": self.close_range_score_threshold,
            "close_range_min_bbox_height_fraction":
                self.close_range_min_bbox_height_fraction,
            "person_motor_conflict_iou": self.person_motor_conflict_iou,
            "person_motor_min_score_margin": self.person_motor_min_score_margin,
            "same_class_nms_iou": self.same_class_nms_iou,
            "same_class_nms_iom": self.same_class_nms_iom,
            "inference_ms": round(inference_ms, 3),
            "semantic_preprocess": preprocess_mode,
            "semantic_input_size": int(self.semantic_input_size),
            "semantic_center_crop_fraction": self.semantic_center_crop_fraction,
            "semantic_crop_offset_x": int(crop_offset_x),
            "person_acquire_score_threshold": self.person_acquire_score_threshold,
            "person_acquire_min_bbox_height_fraction":
                self.person_acquire_min_bbox_height_fraction,
            "person_safety_min_bbox_height_fraction":
                self.person_safety_min_bbox_height_fraction,
            "semantic_watchdog_input_fresh_sec":
                self.semantic_watchdog_upstream_fresh_sec,
            "semantic_publish_count": int(self.semantic_publish_count),
            "letterbox_scale": round(float(letterbox_scale), 6),
            "letterbox_pad_x": int(letterbox_pad_x),
            "letterbox_pad_y": int(letterbox_pad_y),
            "metric_config_ready": self.metric_config is not None,
            "person_distance_model": {
                "type": "bbox_height_linear",
                "intercept_m": self.person_bbox_distance_intercept_m,
                "slope_m": self.person_bbox_distance_slope_m,
                "min_m": self.person_bbox_distance_min_m,
                "max_m": self.person_bbox_distance_max_m,
            },
            "human_clear_miss_count": int(self.human_miss_count),
            "obstacle_memory_trial_active": bool(self.obstacle_memory_trial_active),
            "trial_person_only_planning_filter": bool(self.obstacle_memory_trial_active),
            "final_trial_guard_mode": str(self.final_trial_mode),
            "obstacle_memory_latched": bool(self.obstacle_memory_odom is not None),
            "obstacle_memory_released": bool(self.obstacle_memory_released_for_trial),
            "obstacle_memory_release_count": int(self.obstacle_memory_release_count),
            "obstacle_memory_odom_x": (
                round(float(self.obstacle_memory_odom[0]), 4)
                if self.obstacle_memory_odom is not None else None),
            "obstacle_memory_odom_y": (
                round(float(self.obstacle_memory_odom[1]), 4)
                if self.obstacle_memory_odom is not None else None),
            "obstacle_memory_base_forward_m": (
                round(float(self._obstacle_memory_base_position()[0]), 4)
                if self.obstacle_memory_odom is not None and
                   self._obstacle_memory_base_position() is not None else None),
            "human_detection_hold_sec": self.human_detection_hold_sec,
            "human_tracking_state": tracking_state,
            "human_track_continue_min_score": self.human_track_continue_min_score,
            "human_track_max_without_strong_sec": self.human_track_max_without_strong_sec,
            "human_track_promoted_count": int(self.human_track_promoted_count),
            "human_temporal_hold_count": int(self.human_temporal_hold_count),
            "human_temporal_hold_active": any(
                bool(d.get("temporal_hold", False)) for d in detections),
            "human_temporal_hold_age_ms": max(
                [int(d.get("temporal_hold_age_ms", 0) or 0)
                 for d in detections if d.get("temporal_hold", False)] or [0]),
            "human_clear_active": bool(self.human_clear_burst_remaining > 0),
            "human_clear_last_count": len(self.last_human_metric),
            "human_clear_history_count": len(self.human_clear_history),
            "human_clear_stale_timeout_sec": self.human_clear_stale_timeout_sec,
            "human_stale_clear_count": int(self.human_stale_clear_count),
            "nav2_obstacle_count": int(self.last_nav2_obstacle_count),
            "no_obstacle_frame_count": int(self.no_obstacle_frame_count),
            "full_costmap_clear_request_count": int(self.full_costmap_clear_request_count),
            "boundary_point_count": int(self.last_boundary_count),
            "raw_candidate_count": raw_candidate_count,
            "evaluation_candidate_count": len(evaluation_candidates),
            "evaluation_candidates": evaluation_candidates,
            "adaptive_rejected_count": adaptive_rejected,
            "suppressed_same_class": int(same_class_suppressed),
            "suppressed_cross_class": int(conflict_stats["suppressed"]),
            "ambiguous_conflicts": int(conflict_stats["ambiguous"]),
            "count": len(detections),
            "detections": detections,
        }

        if not rclpy.ok():
            return
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(msg)
        self.last_semantic_publish_monotonic = time.monotonic()
        self.semantic_publish_count += 1
        object_cloud = self.build_object_cloud(detections, message.header.stamp, width, height)
        if object_cloud is not None:
            self.object_points_publisher.publish(object_cloud)
            if self.motion_clear_this_cycle and object_cloud.width > 0:
                self.pending_remark_cloud = object_cloud
                self.pending_remark_due = time.monotonic() + self.human_remark_delay_sec
        self.publish_status(
            "OK",
            count=len(detections),
            raw_candidate_count=raw_candidate_count,
            adaptive_rejected_count=adaptive_rejected,
            suppressed_same_class=int(same_class_suppressed),
            suppressed_cross_class=int(conflict_stats["suppressed"]),
            ambiguous_conflicts=int(conflict_stats["ambiguous"]),
            inference_ms=round(inference_ms, 3),
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
