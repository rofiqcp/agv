#!/usr/bin/env python3
"""Menjalankan IMU Yahboom/WitMotion dengan parameter kalibrasi ROS ENU."""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue




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


# Fungsi: Menyusun LaunchDescription beserta node, parameter, kondisi, dan remapping yang diperlukan.
def generate_launch_description():
    nav_share = get_package_share_directory('navigation')
    params_file = os.path.join(_active_config_dir(nav_share), 'imu.yaml')
    args = [
        DeclareLaunchArgument('port', default_value='auto'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('publish_raw', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
    ]
    # Parameter kalibrasi orientation (yaw_sign/yaw_offset/declination dan
    # invert axis) sengaja TIDAK dioverride launch. Sumber tunggalnya adalah
    # navigation/config/imu.yaml agar hasil kalibrasi GUI/file benar-benar dipakai runtime.
    node = Node(
        package='navigation', executable='imu_node', name='data_imu_node',
        output='screen', emulate_tty=True,
        respawn=True, respawn_delay=2.0,
        parameters=[params_file, {
            'port': LaunchConfiguration('port'),
            'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_raw': ParameterValue(LaunchConfiguration('publish_raw'), value_type=bool),
            'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        }],
    )
    return LaunchDescription(args + [node])
