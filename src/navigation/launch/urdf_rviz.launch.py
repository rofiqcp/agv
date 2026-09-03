#!/usr/bin/env python3

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Fungsi: Menyusun LaunchDescription beserta node, parameter, kondisi, dan remapping yang diperlukan.
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    pkg_share = get_package_share_directory('navigation')
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'agv.urdf.xacro')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'urdf.rviz')

    robot_description = xacro.process_file(urdf_xacro, mappings={
        'agv_description': pkg_share,
    }).toxml()

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            condition=UnlessCondition(gui),
            parameters=[{
                'use_sim_time': use_sim_time,
                'rate': 30,
            }],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_color_optical_frame',
            ],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(rviz),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
