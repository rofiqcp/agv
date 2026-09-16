#!/usr/bin/env python3
"""Standalone browser HMI for an already-running ADV ROS2 stack."""
from launch import LaunchDescription
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _active_config_dir() -> str:
    env = os.environ.get("AGV_CONFIG_DIR", "").strip()
    if env and os.path.isdir(os.path.expanduser(env)):
        return os.path.abspath(os.path.expanduser(env))
    share = Path(get_package_share_directory("navigation")).resolve()
    parts = list(share.parts)
    if "install" in parts:
        idx = parts.index("install")
        workspace = Path(*parts[:idx]) if idx > 0 else Path("/")
        candidate = workspace / "src" / "navigation" / "config"
        if candidate.is_dir():
            return str(candidate.resolve())
    return str(share / "config")


def generate_launch_description():
    navigation_core = os.path.join(_active_config_dir(), "navigation_core.yaml")
    args = [DeclareLaunchArgument("use_sim_time", default_value="false")]
    web = Node(
        package="navigation",
        executable="agv_web_gui",
        name="agv_web_gui",
        output="screen",
        parameters=[navigation_core, {
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )
    return LaunchDescription(args + [web])
