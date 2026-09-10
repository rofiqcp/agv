"""ESC runtime: exactly two nodes, motor_teleop + esc_ackermann."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _active_config_dir() -> str:
    env = os.environ.get("AGV_ESC_CONFIG_DIR", "").strip()
    if env and os.path.isdir(os.path.expanduser(env)):
        return os.path.abspath(os.path.expanduser(env))
    share = Path(get_package_share_directory("esc")).resolve()
    parts = list(share.parts)
    if "install" in parts:
        idx = parts.index("install")
        workspace = Path(*parts[:idx]) if idx > 0 else Path("/")
        candidate = workspace / "src" / "esc" / "config"
        if candidate.is_dir():
            return str(candidate.resolve())
    return str(share / "config")


def generate_launch_description():
    config_dir = _active_config_dir()
    teleop_params = os.path.join(config_dir, "teleop.yaml")
    ackermann_params = os.path.join(config_dir, "ackermann.yaml")
    vesc_tool_params = os.path.join(config_dir, "vesc_tool.yaml")

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_gateway", default_value="true"),
        DeclareLaunchArgument("hmi_port", default_value="auto"),
        DeclareLaunchArgument("publish_stm32_gnss", default_value="true"),
        DeclareLaunchArgument("start_teleop", default_value="true"),
        DeclareLaunchArgument("start_ackermann", default_value="true"),
        DeclareLaunchArgument("start_vesc_tool_bridge", default_value="true"),
        DeclareLaunchArgument("transport_mode", default_value="stm32"),
        DeclareLaunchArgument("serial_device", default_value="auto"),
        DeclareLaunchArgument("serial_enabled", default_value="true"),
        DeclareLaunchArgument("nav2_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("teleop_topic", default_value="/cmd_vel/teleop"),
        DeclareLaunchArgument("teleop_source_topic", default_value="/teleop/active_source"),
        DeclareLaunchArgument("active_source_topic", default_value="/esc/mux/active_source"),
        DeclareLaunchArgument("require_autonomy_gate", default_value="true"),
        # Runtime vehicle SSOT values. Autonomous launch always supplies these from vehicle.yaml.
        DeclareLaunchArgument("vehicle_speed_max_mps", default_value="1.0"),
        DeclareLaunchArgument("vehicle_wheelbase_m", default_value="0.70"),
        DeclareLaunchArgument("vehicle_track_width_m", default_value="0.48"),
        DeclareLaunchArgument("vehicle_wheel_radius_m", default_value="0.145"),
        DeclareLaunchArgument("vehicle_drive_erpm_per_mps", default_value="8000.0"),
        DeclareLaunchArgument("vehicle_drive_odometry_scale", default_value="1.0"),
        DeclareLaunchArgument("vehicle_drive_motor_pole_pairs", default_value="15"),
        DeclareLaunchArgument("vehicle_drive_gear_ratio", default_value="1.0"),
    ]


    stmf4_share = get_package_share_directory("stmf4")
    gateway = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(stmf4_share, "launch", "stmf4.launch.py")),
        condition=IfCondition(LaunchConfiguration("start_gateway")),
        launch_arguments={
            "serial_device": LaunchConfiguration("hmi_port"),
            "publish_stm32_gnss": LaunchConfiguration("publish_stm32_gnss"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    teleop = Node(
        package="esc",
        executable="motor_teleop",
        name="motor_teleop",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_teleop")),
        parameters=[
            teleop_params,
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
        ],
    )

    ackermann = Node(
        package="esc",
        executable="ackermann_controller_server",
        name="esc_ackermann",
        output="screen",
        respawn=True, respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("start_ackermann")),
        # Runtime-routing keys intentionally live only in this launch override
        # block (not in node-scoped ackermann.yaml), so deployment/commissioning
        # values cannot be shadowed by a more-specific YAML node scope.
        parameters=[
            teleop_params,  # shared manual encoding defaults
            ackermann_params,
            {
                # LAST parameter source wins: deployment routing and certified vehicle
                # values cannot be shadowed by node-scoped YAML defaults.
                "transport_mode": LaunchConfiguration("transport_mode"),
                "serial_device": LaunchConfiguration("serial_device"),
                "serial_enabled": ParameterValue(LaunchConfiguration("serial_enabled"), value_type=bool),
                "nav2_topic": LaunchConfiguration("nav2_topic"),
                "teleop_topic": LaunchConfiguration("teleop_topic"),
                "teleop_source_topic": LaunchConfiguration("teleop_source_topic"),
                "active_source_topic": LaunchConfiguration("active_source_topic"),
                "require_autonomy_gate": ParameterValue(
                    LaunchConfiguration("require_autonomy_gate"), value_type=bool),
                "speed_max": ParameterValue(LaunchConfiguration("vehicle_speed_max_mps"), value_type=float),
                "wheelbase_m": ParameterValue(LaunchConfiguration("vehicle_wheelbase_m"), value_type=float),
                "track_width_m": ParameterValue(LaunchConfiguration("vehicle_track_width_m"), value_type=float),
                "drive_wheel_radius_m": ParameterValue(LaunchConfiguration("vehicle_wheel_radius_m"), value_type=float),
                "drive_erpm_per_mps": ParameterValue(LaunchConfiguration("vehicle_drive_erpm_per_mps"), value_type=float),
                "drive_odometry_calibration_scale": ParameterValue(
                    LaunchConfiguration("vehicle_drive_odometry_scale"), value_type=float),
                "drive_motor_pole_pairs": ParameterValue(
                    LaunchConfiguration("vehicle_drive_motor_pole_pairs"), value_type=int),
                "drive_gear_ratio": ParameterValue(LaunchConfiguration("vehicle_drive_gear_ratio"), value_type=float),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
    )

    vesc_tool = Node(
        package="esc",
        executable="vesc_tool_bridge",
        name="vesc_tool_bridge",
        output="screen",
        respawn=True, respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration("start_vesc_tool_bridge")),
        parameters=[vesc_tool_params, {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
    )

    return LaunchDescription(args + [gateway, teleop, ackermann, vesc_tool])
