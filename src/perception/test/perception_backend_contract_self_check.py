#!/usr/bin/env python3
"""Static contract for web-lazy CPU/OpenVINO + direct PT fallback + optional GPU(TensorRT)."""
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
NAV = ROOT.parent / "navigation"

def require(condition: bool, message: str) -> None:
    if not condition:
        print(f"FAIL: {message}")
        raise SystemExit(1)

cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
cpu = (ROOT / "src/astra_yolop_cpu_pt_node.cpp").read_text(encoding="utf-8")
camera = (ROOT / "src/camera_only_node.cpp").read_text(encoding="utf-8")
gpu = (ROOT / "src/astra_yolop_gpu_node.cpp").read_text(encoding="utf-8")
direct_launch = (ROOT / "launch/astra_yolop.launch.py").read_text(encoding="utf-8")
autonomous = (NAV / "launch/autonomous.launch.py").read_text(encoding="utf-8")
gui = (NAV / "launch/gui.launch.py").read_text(encoding="utf-8")
params = yaml.safe_load((ROOT / "config/astra_yolop_gpu.yaml").read_text())["perception"]["ros__parameters"]

require("project(perception LANGUAGES CXX)" in cmake, "perception project must stay C++")
require("find_package(Torch QUIET)" in cmake and "PERCEPTION_CPU_AVAILABLE" in cmake,
        "CPU direct PT must use optional LibTorch capability detection")
require("torch.utils.cmake_prefix_path" in cmake, "CMake must discover LibTorch from system Python torch")
require("add_executable(perception_cpu_node src/astra_yolop_cpu_pt_node.cpp)" in cmake,
        "direct PT CPU target missing")
require("camera-only tetap dibangun" in cmake, "Torch-missing camera-only fallback missing")
require("add_executable(camera_only_node src/camera_only_node.cpp)" in cmake,
        "camera-only target missing")
require("RENAME perception_cpu_node" not in cmake, "legacy Python converter must not own CPU executable")
require("check_language(CUDA)" in cmake and "PERCEPTION_GPU_AVAILABLE" in cmake,
        "GPU target must remain optional/capability-gated")
require('_PERCEPTION_GPU_DEFAULT ON' in cmake and '_PERCEPTION_GPU_DEFAULT OFF' in cmake and
        'option(PERCEPTION_BUILD_GPU "Build the CUDA/TensorRT backend when dependencies exist" ${_PERCEPTION_GPU_DEFAULT})' in cmake,
        "GPU default must be platform-aware: Jetson ON, x86 Mini-PC OFF")
require('set(CMAKE_MESSAGE_LOG_LEVEL ERROR)' in cmake,
        "TorchConfig optional-warning isolation missing")
require('COMPONENTS core imgproc imgcodecs highgui videoio)' in cmake and 'videoio dnn)' not in cmake,
        "unused OpenCV DNN build dependency must stay removed")

for token in (
    "pt_model_path", "torch::jit::load", "torch::InferenceMode", "forwardTorch",
    "prediction_heads", "anchor_heads", "drivable", "lane",
    "/perception/object_points", "/perception/drivable_boundary_points",
    "/perception/camera_healthy", "/perception/emergency_stop",
    "mixRecenterCommand", "MultiThreadedExecutor", "inference_group_", "control_group_",
):
    require(token in cpu, f"CPU direct PT backend token missing: {token}")
require("readNetFromONNX" not in cpu and "onnx_model_path" not in cpu,
        "CPU backend still contains ONNX inference path")
require('declare_parameter<std::string>("pt_model_path", "auto")' in cpu, "CPU portable model default missing")
require("resolveCpuModelPath" in cpu and "YOLOPV2_PT_PATH" in cpu and "AGV_ROOT" in cpu, "CPU portable model discovery missing")
require("resolveCpuThreadCount" in cpu and "torch::set_num_interop_threads(1)" in cpu,
        "CPU anti-oversubscription policy missing")
require('source=yolop_annotated' in cpu and 'source=camera_raw' in cpu,
        "CPU web preview must switch raw/annotated with inference state")

for token in ("camera_only", "model=off", "/camera/yolop/image_annotated",
              "/camera/astra/image_raw", "/perception/camera_connected"):
    require(token in camera, f"camera-only backend missing: {token}")
require("torch" not in camera.lower() and "onnx" not in camera.lower() and "tensorrt" not in camera.lower(),
        "camera-only backend must not depend on model runtimes")

for source, label in ((direct_launch, "direct"), (autonomous, "autonomous"), (gui, "gui")):
    require("perception_mode" in source and "pt_model_path" in source,
            f"{label} launch missing mode/PT path")
require("camera_only_node" in autonomous and "camera_only_enabled" in autonomous,
        "autonomous must retain camera-only fallback outside lazy web mode")
require("importlib.util.find_spec" not in autonomous, "autonomous CPU validation must not probe Python ONNX modules")
require("model CPU tidak ditemukan/kosong" in autonomous, "CPU model fail-fast validation missing")

require(params.get("perception_mode") == "off", "Mini-PC YAML must keep web-lazy CPU ownership under perception_mode=off")
require(params.get("inference_enabled") is True, "Web-lazy Mini-PC runtime must default inference ON for Bab IV perception")
require("web_lazy_cpu_enabled" in autonomous and "perception_inference_enabled" in autonomous,
        "web mode must keep one lazy CPU camera owner with explicit inference toggle")
require("default_value='true'" in autonomous and "perception_inference_enabled" in autonomous,
        "autonomous web-lazy inference launch default must remain ON")
require("PERCEPTION_HAVE_OPENVINO" in cpu and "OpenVINO backend loaded" in cpu,
        "CPU backend must retain OpenVINO acceleration with direct PT fallback")
require("find_library(TURBOJPEG_LIBRARY NAMES turbojpeg)" in cmake and "${TURBOJPEG_LIBRARY}" in cmake,
        "CPU MJPEG capture must link libturbojpeg")
require("tjDecompress2" in cpu and "TJFLAG_FASTDCT" in cpu and
        "TJFLAG_FASTDCT | TJFLAG_STOPONWARNING" not in cpu,
        "MJPEG decoder must accept recoverable warnings and reject fatal decode errors")
require("direct_capture_ever_worked_" in cpu and "DIRECT_V4L2_RECONNECT_WAIT" in cpu and
        "OpenCV fallback dilewati" in cpu and "direct_v4l2_restart_timeout_sec_" in cpu and "rclcpp::shutdown()" in cpu,
        "After direct V4L2 has worked, transient faults must retry direct capture then cleanly respawn instead of sticking to OpenCV MJPEG fallback")
require(params.get("opencv_capture_fallback_enabled") is False,
        "Mini-PC research profile must reject OpenCV MJPEG fallback")
require(float(params.get("direct_v4l2_restart_timeout_sec", 0.0)) >= 5.0,
        "Direct V4L2 restart watchdog timeout missing")
require(params.get("rgb_width") == 640 and params.get("rgb_height") == 480 and
        params.get("v4l2_pixel_format") == "YUYV" and
        params.get("capture_normalize_to_output") is True and
        params.get("capture_output_width") == 1280 and params.get("capture_output_height") == 720,
        "Mini-PC camera profile must capture 640x480 and normalize to calibrated 1280x720 output")
for token in ("normalizeCaptureFrame", "capture_source_width", "capture_output_width",
              "capture_normalize_to_output_"):
    require(token in cpu, f"Capture normalization contract missing: {token}")
require(params.get("raw_publish_width") == 640 and params.get("raw_publish_height") == 360 and
        "raw_publish_width_" in cpu and "cv::INTER_AREA" in cpu,
        "Python semantic image transport must use the lightweight 640x360 raw publish path")
require("if (!raw_resized) publishCameraInfo" in cpu and
        "Do not publish mismatched intrinsics" in cpu,
        "Resized semantic transport must not publish mismatched CameraInfo intrinsics")
require(float(params.get("camera_reopen_stale_sec", 0.0)) >= 1.0 and
        "stale_sec < camera_reopen_stale_sec_" in cpu and
        "FRAME_READ_FAILED_STALE" in cpu,
        "Camera recovery must require both miss streak and last-good-frame staleness before reopen")
require("VIDIOC_REQBUFS" in cpu and "release.count = 0" in cpu and
        "for (int attempt = 0; attempt < 12; ++attempt)" in cpu,
        "Direct V4L2 recovery must release MMAP driver buffers and allow a warm-up window after reopen")
require(params.get("pt_model_path") == "auto", "YAML PT path must use portable auto discovery")
require(float(params.get("cpu_inference_fps", 0.0)) >= 0.5, "CPU inference FPS missing")
require(1 <= int(params.get("cpu_threads", 0)) <= 4, "CPU thread policy must use a bounded mini-PC thread count")
for token in ("lane_metrics_topic", "drivable_space_topic", "perception_obstacle_metrics_topic", "near_field_state_topic"):
    require(token in cpu, f"CPU GUI/BAB IV topic contract missing: {token}")
require(params.get("lane_classical_fallback_enabled") is True,
        "Indoor/tape lane fallback must be explicitly enabled in the Mini-PC research profile")
for token in ("applyClassicalLaneFallback", "lane_classical_fallback_active",
              "lane_classical_left_samples", "lane_classical_right_samples",
              "CLASSICAL_FALLBACK", "lane_classical_min_samples_per_side_"):
    require(token in cpu, f"Classical lane fallback contract missing: {token}")
require("lane_pixels >=" in cpu and "drivable_pixels >=" in cpu and
        "left_points.size()) < lane_classical_min_samples_per_side_" in cpu and
        "right_points.size()) < lane_classical_min_samples_per_side_" in cpu,
        "Classical lane fallback must be fail-closed and require both boundaries")
for token in ("pipeline_p95_ms", "capture_dropped_total", "mean_gradient", "road_width_m"):
    require(token in cpu, f"CPU acquisition metric missing: {token}")

for source, label in ((cpu, "CPU"), (gpu, "GPU")):
    require('get_parameter("camera_metric_calibration_validated")' in source,
            f"{label} backend does not enforce metric calibration")

print("PASS perception backend contract")
print("off=web-lazy CPU owner | OpenVINO preferred | TorchScript .pt fallback | gpu=optional CUDA/TensorRT")

require('TORCH_CXX11_ABI' in cmake and 'compiled_with_cxx11_abi' in cmake, 'Torch CXX11 ABI guard missing')
