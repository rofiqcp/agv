#!/usr/bin/python3
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters

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
        self.declare_parameter("input_topic", "/camera/astra/image_raw")
        self.declare_parameter("output_topic", "/perception/semantic_detections")
        self.declare_parameter("status_topic", "/perception/semantic_status")
        self.declare_parameter("score_threshold", 0.20)
        self.declare_parameter("inference_hz", 2.0)
        self.declare_parameter("cpu_threads", 1)
        self.declare_parameter("enabled", True)
        self.declare_parameter("torch_hub_dir", "models/torch")

        self.score_threshold = float(self.get_parameter("score_threshold").value)
        self.period = 1.0 / max(0.1, float(self.get_parameter("inference_hz").value))
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
        self.publisher = self.create_publisher(
            String, str(self.get_parameter("output_topic").value), 10)
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)
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
        ]
        self.metric_config = None
        self.metric_request_inflight = False
        self.metric_client = self.create_client(GetParameters, "/perception/get_parameters")
        self.metric_timer = self.create_timer(2.0, self.refresh_metric_parameters)

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
                f"threshold={self.score_threshold:.2f} rate={1.0/self.period:.2f} Hz")
        else:
            self.get_logger().info("Semantic detector disabled/unavailable; no inference will run")
        self.refresh_metric_parameters()

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
                        "metric_forward_offset_m", "metric_lateral_offset_m")
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
        if now - self.last_inference < self.period:
            return
        self.last_inference = now

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
            tensor = self.torch.from_numpy(rgb).permute(2, 0, 1).float().div_(255.0)
            started = time.perf_counter()
            with self.torch.inference_mode():
                output = self.model([tensor])[0]
            inference_ms = (time.perf_counter() - started) * 1000.0
        except Exception as exc:
            self.get_logger().error(f"Semantic inference failed: {exc}")
            self.publish_status("INFERENCE_FAILED", error=str(exc))
            return

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

        payload = {
            "source": "torchvision_ssdlite320_coco",
            "image_width": width,
            "image_height": height,
            "score_threshold": self.score_threshold,
            "inference_ms": round(inference_ms, 3),
            "metric_config_ready": self.metric_config is not None,
            "count": len(detections),
            "detections": detections,
        }

        if not rclpy.ok():
            return
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(msg)
        self.publish_status(
            "OK", count=len(detections), inference_ms=round(inference_ms, 3)
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
