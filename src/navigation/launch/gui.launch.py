#!/usr/bin/env python3
"""Full autonomous runtime with RViz inside the native C++/Qt AGV operator GUI."""
from __future__ import annotations

import os
from pathlib import Path
import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node




def _active_config_dir(nav_share: str) -> str:
    """Use AGV_CONFIG_DIR when set; otherwise prefer workspace source config.

    This makes GUI edits and launch-time parameters address the same bytes even
    when colcon was built without --symlink-install. Installed-only deployments
    transparently fall back to package share/config.
    """
    env = os.environ.get("AGV_CONFIG_DIR", "").strip()
    if env and os.path.isdir(os.path.expanduser(env)):
        return os.path.abspath(os.path.expanduser(env))
    share = Path(nav_share).resolve()
    parts = list(share.parts)
    if "install" in parts:
        idx = parts.index("install")
        workspace = Path(*parts[:idx]) if idx > 0 else Path("/")
        candidate = workspace / "src" / "navigation" / "config"
        if candidate.is_dir():
            return str(candidate.resolve())
    return os.path.join(nav_share, "config")


def _yaml_ros_param(path: str, node_name: str, key: str, default):
    """Read a launch default from the same ROS YAML edited by the GUI."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        return data.get(node_name, {}).get("ros__parameters", {}).get(key, default)
    except (OSError, TypeError, yaml.YAMLError):
        return default



def _discover_cpu_model(configured: str, perception_share: str = "") -> str:
    env = os.environ.get("YOLOPV2_PT_PATH", "").strip()
    if env:
        return str(Path(os.path.expandvars(os.path.expanduser(env))).resolve())
    configured = (configured or "auto").strip()
    if configured and configured.lower() != "auto":
        return str(Path(os.path.expandvars(os.path.expanduser(configured))).resolve())
    candidates = []
    if perception_share:
        share = Path(perception_share).resolve(); parts = list(share.parts)
        if "install" in parts:
            idx = parts.index("install")
            workspace = Path(*parts[:idx]) if idx > 0 else Path("/")
            # Model runtime dikelola di root workspace: <workspace>/models/yolopv2.pt.
            candidates.append(workspace / "models" / "yolopv2.pt")
    candidates.extend([Path.home()/"ros"/"models"/"yolopv2.pt", Path("/home/otomasi/ros/models/yolopv2.pt")])
    for candidate in candidates:
        candidate = candidate.expanduser().resolve()
        if candidate.is_file() and candidate.stat().st_size > 0:
            return str(candidate)
    return str((candidates[0] if candidates else Path.home()/"ros"/"models"/"yolopv2.pt").expanduser().resolve())


def generate_launch_description() -> LaunchDescription:
    nav_share = get_package_share_directory("navigation")
    nav_config_dir = _active_config_dir(nav_share)
    esc_share = get_package_share_directory("esc")
    try:
        perception_share = get_package_share_directory("perception")
    except PackageNotFoundError:
        perception_share = ""

    def package_source_config(share_path: str, package_name: str) -> str:
        share = Path(share_path).resolve(); parts = list(share.parts)
        if "install" in parts:
            idx = parts.index("install"); workspace = Path(*parts[:idx]) if idx > 0 else Path("/")
            candidate = workspace / "src" / package_name / "config"
            if candidate.is_dir(): return str(candidate.resolve())
        return str(share / "config")

    esc_config_dir = os.environ.get("AGV_ESC_CONFIG_DIR", "").strip() or package_source_config(esc_share, "esc")
    perception_config_dir = os.environ.get("AGV_PERCEPTION_CONFIG_DIR", "").strip() or (
        package_source_config(perception_share, "perception") if perception_share else "")
    autonomous_launch = os.path.join(nav_share, "launch", "autonomous.launch.py")
    navigation_core_params = os.path.join(nav_config_dir, "navigation_core.yaml")
    perception_params = (
        os.path.join(perception_config_dir, "astra_yolop_gpu.yaml")
        if perception_config_dir else "")

    collision_default = bool(_yaml_ros_param(
        navigation_core_params, "navigation_core", "collision_monitor_enabled", False))
    camera_metric_default = bool(_yaml_ros_param(
        navigation_core_params, "navigation_core", "camera_metric_calibration_validated", False))
    configured_mode = str(_yaml_ros_param(
        perception_params, "perception", "perception_mode", "cpu") or "cpu").strip().lower()
    configured_engine = str(_yaml_ros_param(
        perception_params, "perception", "engine_path", "") or "")
    configured_pt = str(_yaml_ros_param(
        perception_params, "perception", "pt_model_path", "auto") or "auto")
    cpu_fps_default = float(_yaml_ros_param(
        perception_params, "perception", "cpu_inference_fps", 2.0))
    cpu_threads_default = int(_yaml_ros_param(
        perception_params, "perception", "cpu_threads", 0))

    # Keep the runtime arguments intentionally aligned with autonomous.launch.py.
    # A second rviz2 process stays disabled: agv_gui embeds rviz_common directly
    # in its NAV2 MAP page, so the operator gets one window and one renderer.
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map", default_value=os.path.join(nav_share, "maps", "undip", "undip_nav2.yaml")),
        DeclareLaunchArgument("nav2_params", default_value=os.path.join(nav_config_dir, "nav2_ackermann.yaml")),
        # One selector shared with autonomous.launch.py: off | cpu | gpu.
        DeclareLaunchArgument("perception_mode", default_value=configured_mode),
        DeclareLaunchArgument("engine_path", default_value=os.environ.get(
            "YOLOP_ENGINE_PATH", configured_engine)),
        DeclareLaunchArgument("pt_model_path", default_value=_discover_cpu_model(configured_pt, perception_share)),
        DeclareLaunchArgument("cpu_inference_fps", default_value=str(cpu_fps_default)),
        DeclareLaunchArgument("cpu_threads", default_value=str(cpu_threads_default)),
        DeclareLaunchArgument("enable_trajectory_safety", default_value="true"),
        DeclareLaunchArgument("enable_lane_safety", default_value="false"),
        DeclareLaunchArgument("lane_safety_mode", default_value="active"),
        DeclareLaunchArgument(
            "enable_collision_monitor", default_value=str(collision_default).lower()),
        DeclareLaunchArgument("enable_joystick", default_value="true"),
        DeclareLaunchArgument("enable_keyboard", default_value="true"),
        DeclareLaunchArgument("start_esc_ackermann", default_value="true"),
        DeclareLaunchArgument("esc_port", default_value="auto"),
        DeclareLaunchArgument("esc_serial_enabled", default_value="true"),
        DeclareLaunchArgument("start_gnss", default_value="true"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("gnss_port", default_value="auto"),
        DeclareLaunchArgument("imu_port", default_value="auto"),
        DeclareLaunchArgument("rgb_device", default_value="auto"),
        DeclareLaunchArgument("rgb_width", default_value="1280"),
        DeclareLaunchArgument("rgb_height", default_value="720"),
        DeclareLaunchArgument("camera_fps", default_value="30"),
        DeclareLaunchArgument("gpu_device", default_value="0"),
        DeclareLaunchArgument("strict_camera_mode", default_value="false"),
        DeclareLaunchArgument("v4l2_pixel_format", default_value="MJPEG"),
        DeclareLaunchArgument("allow_mjpeg_cpu_fallback", default_value="true"),
        DeclareLaunchArgument("use_v4l2_userptr_zero_copy", default_value="false"),
        DeclareLaunchArgument(
            "camera_metric_calibration_validated",
            default_value=str(camera_metric_default).lower()),
        DeclareLaunchArgument("perception_respawn", default_value="true"),
        DeclareLaunchArgument("stage3_commissioning_mode", default_value="false"),
        DeclareLaunchArgument("start_web_gui", default_value="true"),
        DeclareLaunchArgument("web_bind_address", default_value="127.0.0.1", choices=["127.0.0.1"]),
        DeclareLaunchArgument("web_port", default_value="5000", choices=["5000"]),
        DeclareLaunchArgument("web_read_only", default_value="false"),
    ]

    autonomous = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(autonomous_launch),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "nav2_params": LaunchConfiguration("nav2_params"),
            "enable_rviz": "false",
            "perception_mode": LaunchConfiguration("perception_mode"),
            "engine_path": LaunchConfiguration("engine_path"),
            "pt_model_path": LaunchConfiguration("pt_model_path"),
            "cpu_inference_fps": LaunchConfiguration("cpu_inference_fps"),
            "cpu_threads": LaunchConfiguration("cpu_threads"),
            "enable_trajectory_safety": LaunchConfiguration("enable_trajectory_safety"),
            "enable_lane_safety": LaunchConfiguration("enable_lane_safety"),
            "lane_safety_mode": LaunchConfiguration("lane_safety_mode"),
            "enable_collision_monitor": LaunchConfiguration("enable_collision_monitor"),
            "enable_joystick": LaunchConfiguration("enable_joystick"),
            "enable_keyboard": LaunchConfiguration("enable_keyboard"),
            "start_esc_ackermann": LaunchConfiguration("start_esc_ackermann"),
            "esc_port": LaunchConfiguration("esc_port"),
            "esc_serial_enabled": LaunchConfiguration("esc_serial_enabled"),
            "start_gnss": LaunchConfiguration("start_gnss"),
            "start_imu": LaunchConfiguration("start_imu"),
            "gnss_port": LaunchConfiguration("gnss_port"),
            "imu_port": LaunchConfiguration("imu_port"),
            "rgb_device": LaunchConfiguration("rgb_device"),
            "rgb_width": LaunchConfiguration("rgb_width"),
            "rgb_height": LaunchConfiguration("rgb_height"),
            "camera_fps": LaunchConfiguration("camera_fps"),
            "gpu_device": LaunchConfiguration("gpu_device"),
            "strict_camera_mode": LaunchConfiguration("strict_camera_mode"),
            "v4l2_pixel_format": LaunchConfiguration("v4l2_pixel_format"),
            "allow_mjpeg_cpu_fallback": LaunchConfiguration("allow_mjpeg_cpu_fallback"),
            "use_v4l2_userptr_zero_copy": LaunchConfiguration("use_v4l2_userptr_zero_copy"),
            "camera_metric_calibration_validated": LaunchConfiguration("camera_metric_calibration_validated"),
            "perception_respawn": LaunchConfiguration("perception_respawn"),
            "stage3_commissioning_mode": LaunchConfiguration("stage3_commissioning_mode"),
            "start_web_gui": LaunchConfiguration("start_web_gui"),
            "web_bind_address": LaunchConfiguration("web_bind_address"),
            "web_port": LaunchConfiguration("web_port"),
            "web_read_only": LaunchConfiguration("web_read_only"),
        }.items(),
    )

    gui = Node(
        package="navigation",
        executable="agv_gui",
        # Jangan set name= di launch. launch_ros akan menerjemahkannya menjadi
        # __node remap tingkat proses, sehingga semua rclcpp::Node yang dibuat
        # di dalam executable (agv_gui + embedded RViz) mendapat nama yang sama.
        # Biarkan masing-masing constructor memakai /agv_gui dan
        # /agv_gui_embedded_rviz.
        output="screen",
        emulate_tty=True,
        additional_env={"QT_QPA_PLATFORM": "xcb"},
    )

    # If the operator closes the GUI, stop the enclosing launch as well. This
    # prevents a hidden autonomous stack from continuing after the HMI disappears.
    stop_when_gui_closes = RegisterEventHandler(
        OnProcessExit(
            target_action=gui,
            on_exit=[EmitEvent(event=Shutdown(reason="AGV GUI closed"))],
        )
    )

    environment = [
        SetEnvironmentVariable("AGV_CONFIG_DIR", nav_config_dir),
        SetEnvironmentVariable("AGV_ESC_CONFIG_DIR", esc_config_dir),
        SetEnvironmentVariable("AGV_PERCEPTION_CONFIG_DIR", perception_config_dir),
        SetEnvironmentVariable("QT_QPA_PLATFORM", os.environ.get("QT_QPA_PLATFORM", "xcb")),
        SetEnvironmentVariable("XDG_RUNTIME_DIR", os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}")),
        SetEnvironmentVariable("QT_ACCESSIBILITY", "0"),
        SetEnvironmentVariable("QT_AUTO_SCREEN_SCALE_FACTOR", "1"),
        SetEnvironmentVariable("QT_ENABLE_HIGHDPI_SCALING", "1"),
        SetEnvironmentVariable("QT_SCALE_FACTOR_ROUNDING_POLICY", "PassThrough"),
    ]

    return LaunchDescription(args + environment + [autonomous, gui, stop_when_gui_closes])
