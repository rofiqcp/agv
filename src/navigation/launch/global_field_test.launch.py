#!/usr/bin/env python3
"""Global field test: GNSS + IMU + COG localization WITHOUT ESC and WITHOUT local EKF.

This launch is fully independent from autonomous.launch.py. It does NOT start:
- LocalizationCore production
- Local EKF production
- ESC / actuation
- NavigationCore
- MPPI / trajectory safety supervisors
- Nav2 controller / planner / bt_navigator / behavior / smoother

Architecture:
  GNSS + IMU -> global_field_test_localizer -> /global_test/* -> EKF global test
  -> /global_test/filtered_map -> global_test_odom_bridge
  -> TF map->odom (identity) + odom->base_footprint + /odometry/filtered
  -> robot_state_publisher + RViz
"""

import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('navigation')

    default_params = os.path.join(nav_share, 'config')
    map_file = os.path.join(nav_share, 'maps', 'undip', 'undip_nav2.yaml')
    ekf_params = os.path.join(nav_share, 'config', 'ekf_global_field_test.yaml')
    localizer_params = os.path.join(nav_share, 'config', 'global_field_test.yaml')
    rviz_file = os.path.join(nav_share, 'rviz', 'global_field_test.rviz')
    xacro_file = os.path.join(nav_share, 'urdf', 'agv.urdf.xacro')

    robot_description = xacro.process_file(
        xacro_file, mappings={'agv_description': nav_share}).toxml()

    args = [
        DeclareLaunchArgument('gnss_port', default_value='auto'),
        DeclareLaunchArgument('imu_port', default_value='auto'),
        DeclareLaunchArgument('enable_imu', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_file),
    ]

    robot_state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    gnss = Node(
        package='navigation', executable='gnss_node', name='data_cuav_node',
        output='screen', emulate_tty=True,
        parameters=[os.path.join(default_params, 'gnss.yaml'),
                    {'port': LaunchConfiguration('gnss_port'),
                     'frame_id': 'gnss_link',
                     'velocity_frame_id': 'enu',
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    imu = Node(
        package='navigation', executable='imu_node', name='data_imu_node',
        output='screen', emulate_tty=True,
        parameters=[os.path.join(default_params, 'imu.yaml'),
                    {'port': LaunchConfiguration('imu_port'),
                     'frame_id': 'imu_link',
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('enable_imu')),
    )

    localizer = Node(
        package='navigation', executable='global_field_test_localizer',
        name='global_field_test_localizer', output='screen',
        parameters=[localizer_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'enable_imu': LaunchConfiguration('enable_imu')}],
    )

    global_ekf_with_imu = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node_global_test', output='screen',
        parameters=[ekf_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('odometry/filtered', '/global_test/filtered_map')],
        condition=IfCondition(LaunchConfiguration('enable_imu')),
    )

    global_ekf_gnss_only = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node_global_test', output='screen',
        parameters=[ekf_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'imu0': '',
                     'imu0_config': [False] * 15}],
        remappings=[('odometry/filtered', '/global_test/filtered_map')],
        condition=UnlessCondition(LaunchConfiguration('enable_imu')),
    )

    bridge = Node(
        package='navigation', executable='global_test_odom_bridge',
        name='global_test_odom_bridge', output='screen',
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': False}])

    planner = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen',
        parameters=[os.path.join(default_params, 'nav2_ackermann.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}])

    lifecycle_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 12.0,
                     'node_names': ['map_server', 'planner_server']}])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_global_field_test',
        output='screen', condition=IfCondition(LaunchConfiguration('enable_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    return LaunchDescription(args + [
        LogInfo(msg=['[GLOBAL_FIELD_TEST] GNSS+COG localization; enable_imu=',
                     LaunchConfiguration('enable_imu'), ', NO ESC, NO local EKF']),
        robot_state, gnss, imu, localizer, global_ekf_with_imu,
        global_ekf_gnss_only, bridge,
        map_server, planner, lifecycle_map, rviz,
    ])
