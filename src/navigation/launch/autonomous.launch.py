#!/usr/bin/env python3
"""Autonomous runtime with deterministic TF startup and ESC-owned actuation."""
from __future__ import annotations

import os
import signal
import time
import math
import subprocess
from pathlib import Path
import xacro
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def _validate_ekf_params(path: str) -> None:
    """Fail fast on robot_localization parameter shapes that crash Humble EKF.

    ROS 2 Humble's robot_localization expects covariance parameters as full
    STATE_SIZE x STATE_SIZE arrays (15 x 15 = 225 values). A 15-element
    diagonal-only form is accepted by newer releases, but not by Humble.
    """
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}

    for node_name in ('ekf_filter_node_odom', 'ekf_filter_node_map'):
        try:
            params = data[node_name]['ros__parameters']
        except (KeyError, TypeError) as exc:
            raise RuntimeError(f'EKF config missing {node_name}.ros__parameters') from exc

        # robot_localization only requires a <sensor>N_config entry for sensors
        # that are actually configured on this EKF instance.  The local EKF in
        # this project intentionally has no odom0 input (ESC odometry is optional),
        # while the global EKF does have odom0=/odometry/gnss_map.  Validate every
        # configured sensor generically instead of incorrectly requiring odom0 on
        # both filters.
        sensor_prefixes = ('odom', 'pose', 'twist', 'imu')
        for key, value in params.items():
            if not any(key.startswith(prefix) for prefix in sensor_prefixes):
                continue
            if key.endswith('_config'):
                if not isinstance(value, list) or len(value) != 15 or not all(type(v) is bool for v in value):
                    raise RuntimeError(f'{node_name}.{key} must contain exactly 15 booleans')

        for prefix in sensor_prefixes:
            index = 0
            while f'{prefix}{index}' in params:
                config_key = f'{prefix}{index}_config'
                if config_key not in params:
                    raise RuntimeError(
                        f'{node_name}.{config_key} is required because {prefix}{index} is configured')
                index += 1

        covariance = params.get('process_noise_covariance')
        if not isinstance(covariance, list) or len(covariance) != 225:
            size = len(covariance) if isinstance(covariance, list) else 0
            raise RuntimeError(
                f'{node_name}.process_noise_covariance has {size} values; '
                'ROS 2 Humble robot_localization requires 225 (15x15) values')
        if not all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in covariance):
            raise RuntimeError(f'{node_name}.process_noise_covariance contains non-finite/non-numeric values')

        # Q must be symmetric and every state needs positive diagonal process noise.
        for row in range(15):
            diag = float(covariance[row * 15 + row])
            if diag <= 0.0:
                raise RuntimeError(f'{node_name}.process_noise_covariance diagonal Q[{row},{row}] must be > 0')
            for col in range(row + 1, 15):
                a = float(covariance[row * 15 + col])
                b = float(covariance[col * 15 + row])
                if abs(a - b) > 1.0e-12:
                    raise RuntimeError(
                        f'{node_name}.process_noise_covariance must be symmetric; '
                        f'Q[{row},{col}]={a} != Q[{col},{row}]={b}')


def _keyboard_evdev_readable() -> bool:
    # Hanya node keyboard nyata. Generic /dev/input/event* dapat menunjuk joystick
    # yang readable dan sebelumnya membuat launch menyalakan keyboard node walau
    # event keyboard sendiri tidak punya permission.
    candidates = []
    for pattern in ('/dev/input/by-id/*-event-kbd', '/dev/input/by-path/*-event-kbd'):
        import glob
        candidates.extend(glob.glob(pattern))
    return any(os.path.exists(dev) and os.access(dev, os.R_OK) for dev in candidates)


def _cleanup_stale_workspace_runtime(nav_share: str, esc_share: str, astra_share: str) -> None:
    """Stop older autonomous processes from this workspace before spawning new nodes.

    The currently running launch process and all of its ancestors are excluded, so a
    direct ``ros2 launch`` cannot accidentally terminate itself. We match either the
    installed package executable prefix or the ROS node names owned by this autonomous
    launch; unrelated camera/ROS applications are left untouched.
    """
    uid = os.getuid()
    current_pid = os.getpid()

    ancestors = {current_pid}
    pid = current_pid
    for _ in range(16):
        try:
            stat_fields = Path(f'/proc/{pid}/stat').read_text().split()
            ppid = int(stat_fields[3])
        except (OSError, ValueError, IndexError):
            break
        if ppid <= 1 or ppid in ancestors:
            break
        ancestors.add(ppid)
        pid = ppid

    executable_prefixes = tuple(prefix for prefix in (
        str(Path(nav_share).parents[1] / 'lib' / 'navigation'),
        str(Path(esc_share).parents[1] / 'lib' / 'esc'),
        str(Path(astra_share).parents[1] / 'lib' / 'perception') if astra_share else '',
    ) if prefix)
    autonomous_node_names = (
        'map_server', 'controller_server', 'planner_server', 'behavior_server',
        'velocity_smoother', 'collision_monitor', 'bt_navigator',
        'lifecycle_manager_map', 'lifecycle_manager_smoother', 'lifecycle_manager_navigation',
        'ekf_filter_node_odom', 'ekf_filter_node_map', 'localization_core',
        'navigation_core', 'mppi_closed_loop_supervisor', 'perception',
        'esc_ackermann', 'motor_teleop',
        'data_imu_node', 'data_cuav_node', 'robot_state_publisher', 'rviz2_autonomous',
    )

    victims = []
    proc_root = Path('/proc')
    try:
        processes = list(proc_root.iterdir())
    except OSError:
        return
    for proc in processes:
        if not proc.name.isdigit():
            continue
        proc_pid = int(proc.name)
        if proc_pid in ancestors:
            continue
        try:
            if proc.stat().st_uid != uid:
                continue
            raw = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='ignore')
        except (OSError, PermissionError):
            continue
        if not raw:
            continue
        workspace_node = any(prefix in raw for prefix in executable_prefixes)
        named_autonomous_node = '--ros-args' in raw and any(
            f'__node:={name}' in raw or f'__node:={name} ' in raw for name in autonomous_node_names)
        old_launch = 'ros2 launch navigation autonomous.launch.py' in raw
        if workspace_node or named_autonomous_node or old_launch:
            victims.append(proc_pid)

    if not victims:
        return
    for proc_pid in victims:
        try:
            os.kill(proc_pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 1.2
    while time.monotonic() < deadline:
        if not any(Path(f'/proc/{proc_pid}').exists() for proc_pid in victims):
            return
        time.sleep(0.05)
    for proc_pid in victims:
        if Path(f'/proc/{proc_pid}').exists():
            try:
                os.kill(proc_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass







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

def _active_package_config_dir(package_share: str, package_name: str, env_var: str) -> str:
    """Resolve a writable runtime config directory for non-navigation packages."""
    env = os.environ.get(env_var, "").strip()
    if env and os.path.isdir(os.path.expanduser(env)):
        return os.path.abspath(os.path.expanduser(env))
    share = Path(package_share).resolve()
    parts = list(share.parts)
    if "install" in parts:
        idx = parts.index("install")
        workspace = Path(*parts[:idx]) if idx > 0 else Path("/")
        candidate = workspace / "src" / package_name / "config"
        if candidate.is_dir():
            return str(candidate.resolve())
    return str(share / "config")


def _optional_package_share(package_name: str) -> str:
    """Resolve an optional package without making the navigation-only profile fail."""
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        return ''

def _yaml_ros_param(path: str, node_name: str, key: str, default):
    """Read one ROS parameter from the same writable YAML used by runtime."""
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        return data.get(node_name, {}).get('ros__parameters', {}).get(key, default)
    except (OSError, TypeError, yaml.YAMLError):
        return default


def _runtime_model_path(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(value))).resolve()


def _discover_cpu_model(configured: str, perception_share: str = "") -> str:
    env = os.environ.get("YOLOPV2_PT_PATH", "").strip()
    if env:
        return str(_runtime_model_path(env))
    configured = (configured or "auto").strip()
    if configured and configured.lower() != "auto":
        return str(_runtime_model_path(configured))
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


def _validate_dynamic_links(executable: Path, label: str) -> None:
    """Fail before graph startup when an installed ELF has unresolved shared libraries."""
    try:
        result = subprocess.run(
            ['ldd', str(executable)], text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, timeout=8.0, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f'{label}: ldd gagal dijalankan: {exc}') from exc
    output = result.stdout or ''
    missing = [line.strip() for line in output.splitlines() if 'not found' in line]
    if result.returncode != 0 or missing:
        detail = '; '.join(missing) if missing else output.strip()
        raise RuntimeError(f'{label}: shared-library unresolved: {detail}')


def _validate_perception_request(
    context, package_available: bool, camera_available: bool,
    cpu_available: bool, gpu_available: bool):
    """Validate the exact requested backend. OFF means camera-only, not camera-off."""
    mode = LaunchConfiguration('perception_mode').perform(context).strip()
    if mode not in {'off', 'cpu', 'gpu'}:
        raise RuntimeError('perception_mode wajib tepat: off, cpu, atau gpu')
    if not package_available:
        raise RuntimeError('package perception tidak terpasang; kamera tidak dapat dijalankan')
    if mode == 'off':
        if not camera_available:
            raise RuntimeError('perception_mode=off tetapi camera_only_node tidak terpasang')
        return []
    if mode == 'cpu':
        if not cpu_available:
            raise RuntimeError(
                'perception_mode=cpu tetapi perception_cpu_node (LibTorch .pt langsung) tidak terpasang')
        model = _runtime_model_path(LaunchConfiguration('pt_model_path').perform(context))
        if model.suffix.lower() not in {'.pt', '.torchscript'}:
            raise RuntimeError(f'model CPU harus TorchScript .pt/.torchscript: {model}')
        if not model.is_file() or model.stat().st_size <= 0:
            raise RuntimeError(f'model CPU tidak ditemukan/kosong: {model}')
        perception_exe = Path(get_package_share_directory('perception')).resolve().parents[1] / 'lib/perception/perception_cpu_node'
        _validate_dynamic_links(perception_exe, 'perception_mode=cpu')
        return []
    if not gpu_available:
        raise RuntimeError('perception_mode=gpu tetapi executable CUDA/TensorRT tidak terpasang')
    engine = _runtime_model_path(LaunchConfiguration('engine_path').perform(context))
    if engine.suffix.lower() != '.engine':
        raise RuntimeError(f'model GPU harus .engine: {engine}')
    if not engine.is_file() or engine.stat().st_size <= 0:
        raise RuntimeError(f'TensorRT engine tidak ditemukan/kosong: {engine}')
    perception_exe = Path(get_package_share_directory('perception')).resolve().parents[1] / 'lib/perception/perception_node'
    _validate_dynamic_links(perception_exe, 'perception_mode=gpu')
    return []


def _validate_operator_mode(context):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    if mode not in {'rviz', 'gui', 'web'}:
        raise RuntimeError("mode wajib tepat salah satu: rviz, gui, web")
    return []


def generate_launch_description() -> LaunchDescription:
    nav_share = get_package_share_directory('navigation')
    nav_config_dir = _active_config_dir(nav_share)
    esc_share = get_package_share_directory('esc')
    # Perception/TensorRT is optional for the mini-PC navigation-only profile.
    # Do not resolve it as a hard launch dependency when the package was skipped.
    astra_share = _optional_package_share('perception')
    perception_package_available = bool(astra_share)
    perception_lib_dir = (
        Path(astra_share).resolve().parents[1] / 'lib' / 'perception'
        if astra_share else Path('/nonexistent'))
    perception_camera_path = perception_lib_dir / 'camera_only_node'
    perception_cpu_path = perception_lib_dir / 'perception_cpu_node'
    perception_gpu_path = perception_lib_dir / 'perception_node'
    perception_camera_executable_available = (
        perception_camera_path.is_file() and os.access(perception_camera_path, os.X_OK))
    perception_cpu_executable_available = (
        perception_cpu_path.is_file() and os.access(perception_cpu_path, os.X_OK))
    perception_gpu_executable_available = (
        perception_gpu_path.is_file() and os.access(perception_gpu_path, os.X_OK))
    # Destructive stale-process cleanup is opt-in in production. Default launch
    # never SIGKILLs unrelated diagnostics/rosbag sessions from the same workspace.
    if os.environ.get('AGV_CLEAN_STALE_RUNTIME', '0').strip().lower() in {'1', 'true', 'yes'}:
        _cleanup_stale_workspace_runtime(nav_share, esc_share, astra_share)
    keyboard_hw_available = _keyboard_evdev_readable()

    map_file = os.path.join(nav_share, 'maps', 'undip', 'undip_nav2.yaml')
    nav2_params = os.path.join(nav_config_dir, 'nav2_ackermann.yaml')
    ekf_params = os.path.join(nav_config_dir, 'ekf.yaml')
    _validate_ekf_params(ekf_params)
    localization_params = os.path.join(nav_config_dir, 'localization_cpp.yaml')
    navigation_core_params = os.path.join(nav_config_dir, 'navigation_core.yaml')
    vehicle_params = os.path.join(nav_config_dir, 'vehicle.yaml')
    imu_params = os.path.join(nav_config_dir, 'imu.yaml')
    mppi_closed_loop_params = os.path.join(nav_config_dir, 'mppi_closed_loop.yaml')
    trajectory_safety_params = os.path.join(nav_config_dir, 'trajectory_safety.yaml')
    stage3_params = os.path.join(nav_config_dir, 'stage3_navigation.yaml')
    collision_params = os.path.join(nav_config_dir, 'collision_monitor_production.yaml')
    bt_xml = os.path.join(nav_share, 'behavior_trees', 'ackermann_navigate_to_pose.xml')
    rviz_file = os.path.join(nav_share, 'rviz', 'autonomous.rviz')
    xacro_file = os.path.join(nav_share, 'urdf', 'agv.urdf.xacro')
    perception_config_dir = _active_package_config_dir(
        astra_share, "perception", "AGV_PERCEPTION_CONFIG_DIR") if astra_share else ''
    camera_params = os.path.join(perception_config_dir, 'astra_yolop_gpu.yaml') if astra_share else ''

    collision_default = bool(_yaml_ros_param(
        navigation_core_params, 'navigation_core', 'collision_monitor_enabled', False))
    camera_metric_default = bool(_yaml_ros_param(
        navigation_core_params, 'navigation_core', 'camera_metric_calibration_validated', False))
    steering_calibration_default = bool(_yaml_ros_param(
        vehicle_params, 'vehicle', 'steering_calibration_valid', False))
    steering_circle_calibration_default = bool(_yaml_ros_param(
        vehicle_params, 'vehicle', 'steering_circle_calibration_valid', False))
    drive_odometry_calibration_default = bool(_yaml_ros_param(
        vehicle_params, 'vehicle', 'drive_odometry_calibration_valid', False))
    imu_calibration_default = bool(_yaml_ros_param(
        imu_params, 'data_imu_node', 'stationary_calibration_valid', False))
    stage3_production_default = bool(_yaml_ros_param(
        stage3_params, 'stage3_navigation', 'production_autonomy_certified', False))
    stage3_commissioning_speed = float(_yaml_ros_param(
        stage3_params, 'stage3_navigation', 'commissioning_speed_cap_mps', 0.18))
    configured_mode = str(_yaml_ros_param(camera_params, 'perception', 'perception_mode', 'cpu') or 'cpu').strip().lower()
    configured_engine = str(_yaml_ros_param(camera_params, 'perception', 'engine_path', '') or '')
    engine_path = os.environ.get('YOLOP_ENGINE_PATH', configured_engine)
    configured_pt = str(_yaml_ros_param(camera_params, 'perception', 'pt_model_path', 'auto') or 'auto')
    pt_model_path = _discover_cpu_model(configured_pt, astra_share)
    cpu_fps_default = float(_yaml_ros_param(
        camera_params, 'perception', 'cpu_inference_fps', 2.0))
    cpu_threads_default = int(_yaml_ros_param(
        camera_params, 'perception', 'cpu_threads', 0))

    robot_description = xacro.process_file(
        xacro_file,
        mappings={'publish_functional_sensor_frames': 'true', 'publish_camera_optical_frame': 'true'},
    ).toxml()

    args = [
        DeclareLaunchArgument('mode', default_value='web', description='Operator UI: rviz | gui | web'),
        # Full AGV stack is intentionally isolated from ROS domain 0. Domain 0 on
        # lab/office networks is frequently polluted by stale or foreign DDS
        # participants and can stall endpoint creation before serial/web startup.
        # A dedicated domain + localhost-only transport keeps all on-board C++
        # nodes deterministic while localhost:5000 remains reachable via SSH.
        DeclareLaunchArgument('ros_domain_id', default_value=os.environ.get('AGV_ROS_DOMAIN_ID', '42')),
        DeclareLaunchArgument('ros_localhost_only', default_value=os.environ.get('AGV_ROS_LOCALHOST_ONLY', '1')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_file),
        DeclareLaunchArgument('nav2_params', default_value=nav2_params),
        DeclareLaunchArgument('rviz_config', default_value=rviz_file),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        # Exactly one operator-facing switch: off, cpu, or gpu. OFF means
        # camera-only: camera/GUI preview stay alive while no ML model is loaded.
        DeclareLaunchArgument('perception_mode', default_value=configured_mode),
        DeclareLaunchArgument('engine_path', default_value=engine_path),
        DeclareLaunchArgument('pt_model_path', default_value=pt_model_path),
        DeclareLaunchArgument('cpu_inference_fps', default_value=str(cpu_fps_default)),
        DeclareLaunchArgument('cpu_threads', default_value=str(cpu_threads_default)),
        DeclareLaunchArgument('enable_trajectory_safety', default_value='true'),
        DeclareLaunchArgument('enable_lane_safety', default_value='false'),
        DeclareLaunchArgument('lane_safety_mode', default_value='active'),
        DeclareLaunchArgument('enable_collision_monitor', default_value=str(collision_default).lower()),
        DeclareLaunchArgument('enable_joystick', default_value='true'),
        DeclareLaunchArgument('enable_keyboard', default_value='true'),
        DeclareLaunchArgument('start_esc_ackermann', default_value='true'),
        DeclareLaunchArgument('esc_port', default_value='auto'),
        DeclareLaunchArgument('esc_serial_enabled', default_value='true'),
        DeclareLaunchArgument('start_gnss', default_value='true'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('gnss_port', default_value='auto'),
        DeclareLaunchArgument('imu_port', default_value='auto'),
        DeclareLaunchArgument('rgb_device', default_value='auto'),
        DeclareLaunchArgument('rgb_width', default_value='1280'),
        DeclareLaunchArgument('rgb_height', default_value='720'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('gpu_device', default_value='0'),
        DeclareLaunchArgument('strict_camera_mode', default_value='false'),
        DeclareLaunchArgument('v4l2_pixel_format', default_value='MJPEG'),
        DeclareLaunchArgument('allow_mjpeg_cpu_fallback', default_value='true'),
        DeclareLaunchArgument('use_v4l2_userptr_zero_copy', default_value='false'),
        DeclareLaunchArgument('camera_metric_calibration_validated', default_value=str(camera_metric_default).lower()),
        DeclareLaunchArgument('perception_respawn', default_value='true'),
        DeclareLaunchArgument('stage3_commissioning_mode', default_value='false'),
        DeclareLaunchArgument('start_web_gui', default_value='true'),
        DeclareLaunchArgument('web_bind_address', default_value='127.0.0.1'),
        DeclareLaunchArgument('web_port', default_value='5000'),
        DeclareLaunchArgument('web_read_only', default_value='false'),
    ]

    environment = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', LaunchConfiguration('ros_localhost_only')),
        SetEnvironmentVariable('CUDA_MODULE_LOADING', 'LAZY'),
        SetEnvironmentVariable('NO_AT_BRIDGE', '1'),
        SetEnvironmentVariable('QT_ACCESSIBILITY', '0'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', os.environ.get('QT_QPA_PLATFORM', 'xcb')),
        SetEnvironmentVariable('XDG_RUNTIME_DIR', os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')),
        SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '1'),
        SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '1'),
        SetEnvironmentVariable('QT_SCALE_FACTOR_ROUNDING_POLICY', 'PassThrough'),
        SetEnvironmentVariable('__GL_SYNC_TO_VBLANK', '0'),
        SetEnvironmentVariable('__GL_MaxFramesAllowed', '1'),
    ]

    robot_state = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    joint_state_visualizer = Node(
        package='navigation', executable='joint_state_visualizer',
        name='joint_state_visualizer', output='screen',
        respawn=True, respawn_delay=2.0,
        parameters=[{
            'wheelbase_m': float(_yaml_ros_param(vehicle_params, 'vehicle', 'wheelbase_m', 0.70)),
            'track_width_m': float(_yaml_ros_param(vehicle_params, 'vehicle', 'track_width_m', 0.48)),
            'wheel_radius_m': float(_yaml_ros_param(vehicle_params, 'vehicle', 'wheel_radius_m', 0.145)),
            'max_visual_steering_rad': float(_yaml_ros_param(vehicle_params, 'vehicle', 'max_steering_angle_rad', 0.523598775598)),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'gnss.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_gnss')),
        launch_arguments={'port': LaunchConfiguration('gnss_port'), 'baudrate': '38400',
                          'frame_id': 'gnss_link', 'velocity_frame_id': 'enu',
                          'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'imu.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_imu')),
        launch_arguments={'port': LaunchConfiguration('imu_port'), 'baudrate': '9600',
                          'frame_id': 'imu_link',
                          'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
    )

    # ESC package now has exactly two runtime nodes: motor_teleop + esc_ackermann.
    # esc_ackermann owns arbitration, Ackermann conversion and the STM UART.
    esc_runtime = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(esc_share, 'launch', 'esc.launch.py')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'start_teleop': PythonExpression([
                "'", LaunchConfiguration('enable_keyboard'), "' == 'true' or '",
                LaunchConfiguration('enable_joystick'), "' == 'true'"]),
            'start_ackermann': LaunchConfiguration('start_esc_ackermann'),
            'serial_device': LaunchConfiguration('esc_port'),
            'serial_enabled': LaunchConfiguration('esc_serial_enabled'),
            # Direct teleop consumption is disabled here: every motion command must
            # pass cmd_vel_router -> velocity_smoother before ESC/USART.
            'require_autonomy_gate': 'false',
            'teleop_topic': '/cmd_vel/teleop_disabled_after_router',
            'active_source_topic': '/esc/mux/active_source_internal',
            'nav2_topic': '/cmd_vel',
        }.items(),
    )

    local_ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node_odom',
        output='screen', parameters=[ekf_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('odometry/filtered', '/odometry/filtered'), ('set_pose', '/ekf_local/set_pose')],
    )
    global_ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node_map',
        output='screen', parameters=[ekf_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('odometry/filtered', '/odometry/filtered_map'), ('set_pose', '/ekf_global/set_pose')],
    )
    localization_core = Node(
        package='navigation', executable='localization_core', name='localization_core',
        output='screen', respawn=True, respawn_delay=2.0,
        parameters=[localization_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    perception_mode_valid = PythonExpression([
        "'", LaunchConfiguration('perception_mode'), "' in ['off', 'cpu', 'gpu']",
    ])
    camera_only_enabled = PythonExpression([
        "'", LaunchConfiguration('perception_mode'), "' == 'off' and ",
        str(perception_package_available), " and ",
        str(perception_camera_executable_available),
    ])
    perception_cpu_enabled = PythonExpression([
        "'", LaunchConfiguration('perception_mode'), "' == 'cpu' and ",
        str(perception_package_available), " and ",
        str(perception_cpu_executable_available),
    ])
    perception_gpu_enabled = PythonExpression([
        "'", LaunchConfiguration('perception_mode'), "' == 'gpu' and ",
        str(perception_package_available), " and ",
        str(perception_gpu_executable_available),
    ])
    # ``perception_enabled`` means model inference/safety streams, not camera.
    perception_enabled = PythonExpression([
        "(", perception_cpu_enabled, ") or (", perception_gpu_enabled, ")",
    ])
    perception_requested_but_unavailable = PythonExpression([
        "not ", str(perception_package_available),
    ])
    perception_mode_invalid = PythonExpression([
        "not (", perception_mode_valid, ")",
    ])
    perception_backend_unavailable = PythonExpression([
        "('", LaunchConfiguration('perception_mode'), "' == 'off' and not ",
        str(perception_camera_executable_available), ") or ('",
        LaunchConfiguration('perception_mode'), "' == 'cpu' and not ",
        str(perception_cpu_executable_available), ") or ('",
        LaunchConfiguration('perception_mode'), "' == 'gpu' and not ",
        str(perception_gpu_executable_available), ")",
    ])
    # Metric projection is a physical calibration. Camera images/inference may be
    # tested before it passes, but perception must not own autonomous commands.
    perception_safety_enabled = PythonExpression([
        "(", perception_enabled, ") and '",
        LaunchConfiguration('camera_metric_calibration_validated'), "' == 'true'",
    ])
    lane_enabled = PythonExpression([
        "'", LaunchConfiguration('enable_lane_safety'), "' == 'true' and (",
        perception_safety_enabled, ")",
    ])
    common_perception_parameters = {
        'rgb_device': LaunchConfiguration('rgb_device'),
        'rgb_width': ParameterValue(LaunchConfiguration('rgb_width'), value_type=int),
        'rgb_height': ParameterValue(LaunchConfiguration('rgb_height'), value_type=int),
        'fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=int),
        'strict_camera_mode': ParameterValue(LaunchConfiguration('strict_camera_mode'), value_type=bool),
        'v4l2_pixel_format': LaunchConfiguration('v4l2_pixel_format'),
        'allow_mjpeg_cpu_fallback': ParameterValue(LaunchConfiguration('allow_mjpeg_cpu_fallback'), value_type=bool),
        'use_v4l2_userptr_zero_copy': ParameterValue(LaunchConfiguration('use_v4l2_userptr_zero_copy'), value_type=bool),
        'camera_hotplug_retry': True,
        'camera_retry_interval_sec': 2.0,
        'camera_retry_log_interval_sec': 10.0,
        'allow_resolution_fallback_for_fps': False,
        'enable_integrated_perception_runtime': True,
        'camera_metric_calibration_validated': ParameterValue(LaunchConfiguration('camera_metric_calibration_validated'), value_type=bool),
        'lane_safety_enabled': ParameterValue(lane_enabled, value_type=bool),
        'control_mode': ParameterValue(LaunchConfiguration('lane_safety_mode'), value_type=str),
        'publish_annotated': True,
        'publish_masks': False, 'publish_drivable_mask': False, 'publish_lane_mask': False,
        'publish_lane_metrics': False, 'publish_obstacle_metrics': False,
        'publish_detections': False, 'publish_raw_rgb': True,
        'publish_only_when_subscribed': True, 'async_rviz_publish': True,
        'rviz_publish_buffer_count': 2, 'rviz_max_publish_rate_hz': 8.0,
        'show_opencv_cuda_window': False, 'draw_text_labels_cpu': False,
        'annotated_topic': '/camera/yolop/image_annotated',
        'metric_frame_id': 'base_footprint',
        'nav_cmd_topic': '/cmd_vel_nav_raw',
        'safe_cmd_topic': '/cmd_vel/perception_advisory',
    }
    camera_only = Node(
        package='perception', executable='camera_only_node', name='perception_camera',
        output='screen', emulate_tty=True, condition=IfCondition(camera_only_enabled),
        respawn=PythonExpression(["'", LaunchConfiguration('perception_respawn'), "' == 'true'"]),
        respawn_delay=5.0,
        parameters=[camera_params, {
            **common_perception_parameters,
        }],
    )
    perception_gpu = Node(
        package='perception', executable='perception_node', name='perception',
        output='screen', emulate_tty=True, condition=IfCondition(perception_gpu_enabled),
        respawn=PythonExpression(["'", LaunchConfiguration('perception_respawn'), "' == 'true'"]),
        respawn_delay=5.0,
        parameters=[camera_params, {
            'engine_path': LaunchConfiguration('engine_path'),
            'gpu_device': ParameterValue(LaunchConfiguration('gpu_device'), value_type=int),
            **common_perception_parameters,
        }],
    )
    perception_cpu = Node(
        package='perception', executable='perception_cpu_node', name='perception',
        output='screen', emulate_tty=True, condition=IfCondition(perception_cpu_enabled),
        respawn=PythonExpression(["'", LaunchConfiguration('perception_respawn'), "' == 'true'"]),
        respawn_delay=5.0,
        parameters=[camera_params, {
            'pt_model_path': LaunchConfiguration('pt_model_path'),
            'cpu_inference_fps': ParameterValue(LaunchConfiguration('cpu_inference_fps'), value_type=float),
            'cpu_threads': ParameterValue(LaunchConfiguration('cpu_threads'), value_type=int),
            **common_perception_parameters,
        }],
    )
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        respawn=True, respawn_delay=2.0,
        parameters=[LaunchConfiguration('nav2_params'), {'yaml_filename': LaunchConfiguration('map')}])
    planner = Node(
        package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
        respawn=True, respawn_delay=2.0, parameters=[LaunchConfiguration('nav2_params')])
    controller = Node(
        package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
        respawn=True, respawn_delay=2.0, parameters=[LaunchConfiguration('nav2_params')],
        remappings=[('cmd_vel', '/cmd_vel_nav_raw')])
    behavior = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
        respawn=True, respawn_delay=2.0, parameters=[LaunchConfiguration('nav2_params')])
    navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
        respawn=True, respawn_delay=2.0,
        parameters=[LaunchConfiguration('nav2_params'), {'default_nav_to_pose_bt_xml': bt_xml}])
    cmd_vel_router = Node(
        package='navigation', executable='cmd_vel_router', name='cmd_vel_router',
        output='screen', respawn=True, respawn_delay=2.0,
        parameters=[{
            'teleop_topic': '/cmd_vel/teleop',
            'autonomy_topic': '/cmd_vel/autonomy_pre_smoother',
            'autonomy_gate_topic': '/system/autonomy_motion_allowed',
            'global_estop_topic': '/safety/estop',
            'output_topic': '/cmd_vel/pre_smoother',
            'source_topic': '/esc/mux/active_source',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }])

    smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
        output='screen', respawn=True, respawn_delay=2.0,
        parameters=[LaunchConfiguration('nav2_params')],
        remappings=[('cmd_vel', '/cmd_vel/pre_smoother'), ('cmd_vel_smoothed', '/cmd_vel')])

    collision_enabled = PythonExpression([
        "'", LaunchConfiguration('enable_collision_monitor'), "' == 'true' and '",
        LaunchConfiguration('camera_metric_calibration_validated'), "' == 'true' and (",
        perception_safety_enabled, ")",
    ])
    collision = Node(
        package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
        output='screen', condition=IfCondition(collision_enabled), respawn=True, respawn_delay=2.0,
        parameters=[collision_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}])

    # Map statis adalah infrastruktur visual/planning dan tidak boleh ikut terkunci
    # oleh kualitas GNSS/IMU. Lifecycle map terpisah dan langsung aktif sehingga
    # /map tersedia di RViz bahkan saat localization masih STARTUP/DEGRADED.
    lifecycle_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 12.0,
                     'attempt_respawn_reconnection': True, 'bond_respawn_max_duration': 30.0,
                     'node_names': ['map_server']}])

    # Final velocity smoother is an independent command-conditioning stage and must be
    # ACTIVE from launch. Every TELEOP/AUTONOMY command is routed into it before ESC;
    # it must never depend on localization lifecycle startup.
    # localization allowed the entire Nav2 stack to STARTUP. Keep it managed by
    # its own autostart lifecycle manager instead.
    lifecycle_smoother = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_smoother', output='screen',
        parameters=[{'autostart': True, 'bond_timeout': 12.0,
                     'attempt_respawn_reconnection': True, 'bond_respawn_max_duration': 30.0,
                     'node_names': ['velocity_smoother']}])

    # Planner/controller/BT tetap fail-closed: NavigationCore memanggil STARTUP
    # hanya setelah LocalizationCore mempunyai anchor map->odom yang valid.
    lifecycle_with_collision = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', condition=IfCondition(collision_enabled),
        parameters=[{'autostart': False, 'bond_timeout': 12.0,
                     'attempt_respawn_reconnection': True, 'bond_respawn_max_duration': 30.0,
                     'node_names': ['controller_server', 'planner_server',
                                    'behavior_server', 'collision_monitor', 'bt_navigator']}])
    lifecycle_without_collision = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', condition=UnlessCondition(collision_enabled),
        parameters=[{'autostart': False, 'bond_timeout': 12.0,
                     'attempt_respawn_reconnection': True, 'bond_respawn_max_duration': 30.0,
                     'node_names': ['controller_server', 'planner_server',
                                    'behavior_server', 'bt_navigator']}])

    # Perception tidak lagi menjadi owner command autonomous. Ia hanya menerbitkan
    # obstacle candidate + lane-state + advisory yaw. TrajectorySafetySupervisor
    # menggabungkan future path Nav2, localization pose, obstacle path-relevance,
    # camera/emergency state, dan lane constraint sebelum NavigationCore.
    trajectory_safety = Node(
        package='navigation', executable='trajectory_safety_supervisor',
        name='trajectory_safety_supervisor', output='screen',
        condition=IfCondition(perception_safety_enabled),
        respawn=True, respawn_delay=2.0,
        parameters=[trajectory_safety_params, {
            'lane_safety_enabled': ParameterValue(lane_enabled, value_type=bool),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Bypass eksplisit tetap tersedia untuk commissioning: bila perception atau
    # trajectory-safety command dimatikan, NavigationCore kembali memakai smoother Nav2.
    # Supervisor tetap hidup selama perception aktif agar path-relevant cloud untuk
    # local costmap/diagnostic tetap tersedia pada mode commissioning.
    autonomy_source = PythonExpression([
        "'/cmd_vel/autonomy_integrated' if ('", LaunchConfiguration('enable_trajectory_safety'),
        "' == 'true' and (", perception_safety_enabled,
        ")) else '/cmd_vel_nav_raw'",
    ])
    navigation_core = Node(
        package='navigation', executable='navigation_core', name='navigation_core', output='screen',
        respawn=True, respawn_delay=2.0,
        parameters=[navigation_core_params, {
            'autonomy_topic': autonomy_source,
            # Launch default is read from navigation_core.yaml. If the operator
            # explicitly overrides enable_collision_monitor on the CLI, route ownership
            # must change atomically too; otherwise NavigationCore and Collision Monitor
            # could both publish /cmd_vel.
            'collision_monitor_enabled': ParameterValue(collision_enabled, value_type=bool),
            'camera_metric_calibration_validated': ParameterValue(LaunchConfiguration('camera_metric_calibration_validated'), value_type=bool),
            # Stage-1 calibration state is sourced from vehicle.yaml on every launch.
            # This keeps autonomous actuation fail-closed after a reboot until the
            # corresponding physical commissioning data has actually been saved.
            'steering_calibration_validated': steering_calibration_default,
            'steering_circle_calibration_validated': steering_circle_calibration_default,
            'drive_odometry_calibration_validated': drive_odometry_calibration_default,
            # Stage-2 IMU calibration state is persistent in imu.yaml. Local EKF
            # uses gyro-Z, so autonomous motion remains fail-closed until this PASS.
            'imu_calibration_validated': imu_calibration_default,
            'stage3_production_certified': stage3_production_default,
            'stage3_commissioning_mode': ParameterValue(LaunchConfiguration('stage3_commissioning_mode'), value_type=bool),
            'stage3_commissioning_speed_cap_mps': stage3_commissioning_speed,
            'keyboard_available': keyboard_hw_available,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )
    mppi_closed_loop = Node(
        package='navigation', executable='mppi_closed_loop_supervisor',
        name='mppi_closed_loop_supervisor', output='screen', respawn=True, respawn_delay=2.0,
        parameters=[mppi_closed_loop_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}])

    mode_rviz = PythonExpression(["'", LaunchConfiguration('mode'), "'.lower() == 'rviz'"])
    mode_gui = PythonExpression(["'", LaunchConfiguration('mode'), "'.lower() == 'gui'"])
    mode_web = PythonExpression(["'", LaunchConfiguration('mode'), "'.lower() == 'web'"])

    # Native Qt operator GUI. It embeds RViz itself, therefore mode=gui never
    # starts a second standalone rviz2 process. Do not set name= here because
    # the executable creates both /agv_gui and /agv_gui_embedded_rviz nodes.
    native_gui = Node(
        package='navigation', executable='agv_gui', output='screen', emulate_tty=True,
        condition=IfCondition(mode_gui),
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            'XDG_RUNTIME_DIR': os.environ.get('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}'),
            'QT_ACCESSIBILITY': '0',
        })

    stop_when_gui_closes = RegisterEventHandler(
        OnProcessExit(
            target_action=native_gui,
            on_exit=[EmitEvent(event=Shutdown(reason='AGV GUI closed'))],
        ),
        condition=IfCondition(mode_gui),
    )

    # Native C++ browser HMI. Loopback-only by default; ubah bind address ke
    # 0.0.0.0 hanya bila operator memang membutuhkan akses dari LAN tepercaya.
    web_gui = Node(
        package='navigation', executable='agv_web_gui', name='agv_web_gui', output='screen',
        condition=IfCondition(mode_web),
        respawn=True, respawn_delay=2.0,
        parameters=[{
            'bind_address': LaunchConfiguration('web_bind_address'),
            'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int),
            'read_only': ParameterValue(LaunchConfiguration('web_read_only'), value_type=bool),
            'camera_jpeg_fps': 5.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }])

    # Jalankan RViz resmi secara langsung. Jangan bergantung pada wrapper
    # navigation/rviz_clean_launcher karena wrapper itu bisa tidak ikut
    # ter-install dan membuat seluruh launch berhenti sebelum bringup selesai.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_autonomous', output='screen',
        condition=IfCondition(mode_rviz),
        arguments=['-d', LaunchConfiguration('rviz_config'), '--ros-args', '--log-level', 'warn'],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    # Direct test mode: sensor serial dimulai langsung seperti bringup lama.
    # Tidak ada delay buatan sehingga EKF/localization segera menerima sensor.
    return LaunchDescription(args + environment + [
        OpaqueFunction(function=_validate_operator_mode),
        OpaqueFunction(
            function=_validate_perception_request,
            args=[perception_package_available, perception_camera_executable_available,
                  perception_cpu_executable_available, perception_gpu_executable_available]),
        LogInfo(msg=['[AGV] autonomous stack | mode=', LaunchConfiguration('mode'),
                     ' | velocity smoother + measured steering calibration + safety gates enabled']),
        LogInfo(
            condition=IfCondition(perception_requested_but_unavailable),
            msg='[AGV] ERROR: package perception/camera backend tidak tersedia.'),
        LogInfo(
            condition=IfCondition(perception_mode_invalid),
            msg='[AGV] ERROR: perception_mode wajib off, cpu, atau gpu; perception tidak dijalankan.'),
        LogInfo(
            condition=IfCondition(perception_backend_unavailable),
            msg='[AGV] ERROR: executable backend yang dipilih tidak tersedia. OFF membutuhkan camera_only_node; '
                'CPU membutuhkan LibTorch; GPU membutuhkan CUDA+TensorRT.'),
        LogInfo(
            condition=IfCondition(perception_cpu_enabled),
            msg='[AGV] PERCEPTION CPU: LibTorch/TorchScript aktif di CPU; path model dipilih portable dari launch; ONNX tidak digunakan.'),
        LogInfo(
            condition=IfCondition(perception_gpu_enabled),
            msg='[AGV] PERCEPTION GPU: CUDA + TensorRT engine aktif.'),
        LogInfo(
            condition=IfCondition(PythonExpression([
                "(", perception_enabled, ") and '",
                LaunchConfiguration('camera_metric_calibration_validated'), "' != 'true'",
            ])),
            msg='[AGV] CAMERA TEST ONLY: kalibrasi metrik kamera belum valid; trajectory/lane/collision safety '
                'tidak diberi authority command autonomous.'),
        LogInfo(
            condition=IfCondition(camera_only_enabled),
            msg='[AGV] PERCEPTION OFF: camera-only aktif; raw/preview kamera jalan, model/inference OFF.'),
        robot_state, joint_state_visualizer, gnss, imu, esc_runtime,
        local_ekf, global_ekf, localization_core,
        camera_only, perception_cpu, perception_gpu,
        map_server, lifecycle_map, controller, planner, behavior, cmd_vel_router, smoother, collision, navigator,
        lifecycle_smoother, lifecycle_with_collision, lifecycle_without_collision,
        trajectory_safety, navigation_core, mppi_closed_loop, native_gui, stop_when_gui_closes, web_gui, rviz,
    ])
