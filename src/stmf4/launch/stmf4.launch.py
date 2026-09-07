#!/usr/bin/env python3
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = str(Path(get_package_share_directory("stmf4")) / "config" / "hmi.yaml")
    args = [
        DeclareLaunchArgument("serial_device", default_value="auto"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    node = Node(
        package="stmf4", executable="stmf4_hmi_bridge", name="stmf4_hmi_bridge", output="screen",
        respawn=True, respawn_delay=5.0,
        parameters=[cfg, {
            "serial_device": LaunchConfiguration("serial_device"),
            "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        }],
    )
    return LaunchDescription(args + [node])
