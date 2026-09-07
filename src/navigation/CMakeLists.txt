cmake_minimum_required(VERSION 3.18)
project(navigation)

# ROS 2 Humble must use the system Python, not an unrelated activated venv
# (e.g. ~/.hermes/venv). ament_cmake executes package_xml_2_cmake.py through
# Python3::Interpreter, so a stale virtualenv here breaks even pure C++ packages.
if(EXISTS "/usr/bin/python3")
  set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS system Python" FORCE)
  set(PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "ROS system Python (legacy)" FORCE)
endif()


# ROS 2 Humble dependencies may emit legacy CMake-policy notices unrelated to
# this package; suppress only dependency/developer-policy chatter, never compiler warnings.
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "Suppress dependency deprecation notices" FORCE)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON CACHE BOOL "Suppress dependency developer notices" FORCE)

# Navigation runtime ditulis C++17. Launch tetap Python karena itu format launch
# standar ROS 2, tetapi tidak ada rclpy node pada autonomous runtime.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTORCC OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Network)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)


# Native C++/Qt operator GUI. No rclpy/PyQt runtime is required.
add_executable(agv_gui gui/agv_gui.cpp gui/modules/experiment_components.hpp)
target_include_directories(agv_gui PRIVATE gui include)
ament_target_dependencies(
  agv_gui
  rclcpp rcl_interfaces action_msgs sensor_msgs std_msgs builtin_interfaces
  geometry_msgs nav_msgs std_srvs ament_index_cpp
  rviz_common rviz_default_plugins rviz_rendering)
target_link_libraries(agv_gui Qt5::Widgets Qt5::Network yaml-cpp)

# Browser operator console. Backend tetap native C++/rclcpp dan menyajikan
# HTML/CSS/JS lokal pada localhost:5000; tidak membutuhkan npm/CDN runtime.
add_executable(agv_web_gui web/web_server.cpp)
target_include_directories(agv_web_gui PRIVATE gui include)
ament_target_dependencies(
  agv_web_gui
  rclcpp rcl_interfaces action_msgs sensor_msgs std_msgs geometry_msgs nav_msgs visualization_msgs
  std_srvs ament_index_cpp tf2_ros)
target_link_libraries(agv_web_gui Qt5::Core Qt5::Gui Qt5::Network yaml-cpp)

# GNSS / IMU driver C++.
add_executable(gnss_node src/gnss_node.cpp)
target_include_directories(gnss_node PRIVATE include)
ament_target_dependencies(gnss_node rclcpp sensor_msgs std_msgs geometry_msgs)

add_executable(imu_node src/imu_node.cpp)
target_include_directories(imu_node PRIVATE include)
ament_target_dependencies(imu_node rclcpp sensor_msgs std_msgs)

# Diagnostic-only longitudinal speed estimator from IMU acceleration + optional gyro-assisted ZUPT.
# It deliberately publishes no Odometry and is not wired into robot_localization.
add_executable(imu_speed_diagnostic src/imu_speed_diagnostic.cpp)
ament_target_dependencies(imu_speed_diagnostic rclcpp sensor_msgs std_msgs geometry_msgs)

# Tilt-compensated independent heading observations from Yahboom MAG + NEO3 IST8310.
add_executable(mag_heading_fusion src/mag_heading_fusion_node.cpp)
ament_target_dependencies(mag_heading_fusion rclcpp sensor_msgs std_msgs geometry_msgs)

# Lightweight /joint_states bridge so robot_state_publisher can publish TF for
# steering/wheel links and the embedded RViz always shows the complete URDF.
add_executable(joint_state_visualizer src/joint_state_visualizer.cpp)
ament_target_dependencies(joint_state_visualizer rclcpp sensor_msgs std_msgs)

# Satu localization runtime menggantikan georeference/filter/TF-gate Python; EKF global hanya telemetry.
add_executable(localization_core src/localization_core.cpp)
target_include_directories(localization_core PRIVATE include)
ament_target_dependencies(
  localization_core
  rclcpp sensor_msgs std_msgs builtin_interfaces geometry_msgs nav_msgs std_srvs tf2_ros)

# Final command router: TELEOP/AUTONOMY arbitration happens before the single final velocity_smoother.
add_executable(cmd_vel_router src/cmd_vel_router.cpp)
ament_target_dependencies(cmd_vel_router rclcpp std_msgs geometry_msgs)

# NavigationCore hanya mengurus lifecycle/goal/autonomous routing.
add_executable(navigation_core src/navigation_core.cpp)
target_include_directories(navigation_core PRIVATE include)
ament_target_dependencies(
  navigation_core
  rclcpp rclcpp_action std_msgs geometry_msgs nav_msgs sensor_msgs nav2_msgs lifecycle_msgs)


# Closed-loop observer/supervisor: tidak membuat controller baru; hanya mengawasi
# feedback aktual. velocity_smoother tetap OPEN_LOOP statis untuk kompatibilitas Humble.
add_executable(mppi_closed_loop_supervisor src/mppi_closed_loop_supervisor.cpp)
ament_target_dependencies(
  mppi_closed_loop_supervisor
  rclcpp std_msgs geometry_msgs nav_msgs)

# Precision observer: co-register ESC longitudinal speed + IMU yaw-rate, estimate
# steering response/offset and publish innovations. Observer-only: never commands actuators.
add_executable(vehicle_dynamics_observer src/vehicle_dynamics_observer.cpp)
ament_target_dependencies(
  vehicle_dynamics_observer
  rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs)

# Global field test nodes: independent localization without /odometry/filtered dependency
add_executable(global_field_test_localizer src/global_field_test_localizer.cpp)
target_include_directories(global_field_test_localizer PRIVATE include)
ament_target_dependencies(
  global_field_test_localizer
  rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs)

add_executable(global_test_odom_bridge src/global_test_odom_bridge.cpp)
ament_target_dependencies(
  global_test_odom_bridge
  rclcpp nav_msgs geometry_msgs tf2_ros)

# Supervisor integrasi trajectory + perception constraints. Nav2 tetap pemilik
# trajectory/velocity; node ini hanya melakukan path-aware safety arbitration.
add_executable(trajectory_safety_supervisor src/trajectory_safety_supervisor.cpp)
ament_target_dependencies(
  trajectory_safety_supervisor
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

# Regression/self-check suite. These checks are intentionally dependency-light so
# `colcon test --packages-select navigation` catches geometry/config regressions.
if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  set(NAVIGATION_PY_SELF_CHECKS
    avoidance_contract_self_check
    ekf_humble_config_self_check
    ekf_launch_validation_self_check
    gnss_driver_v2_part1_self_check
    gnss_global_fusion_part3_self_check
    gnss_motion_math_self_check
    gnss_motion_validation_part2_self_check
    gnss_provisional_tf_self_check
    gnss_ubx_fixture_self_check
    full_stack_reaudit_self_check
    gui_cpp_migration_self_check
    gui_floating_menu_self_check
    gui_qt_signal_slot_self_check
    gui_nav2_embed_self_check
    web_gui_self_check
    launch_dependency_self_check
    humble_build_compat_self_check
    minipc_navigation_only_self_check
    minipc_launch_integration_self_check
    minipc_build_environment_self_check
    nav2_velocity_chain_self_check
    gui_bab4_acquisition_self_check
    gui_navigation_tuning_workspace_self_check
    precision_part1_self_check
    precision_part2_self_check
    precision_part3_self_check
    precision_upgrade_self_check
    project_consistency_self_check
    stage1_foundation_self_check
    stage2_localization_foundation_self_check
    stage3_nav2_safety_self_check
    steering_physical_part1_self_check
    steering_physical_part2_self_check
    steering_physical_part3_self_check
    trajectory_safety_contract_self_check)

  foreach(check_name IN LISTS NAVIGATION_PY_SELF_CHECKS)
    add_test(
      NAME ${check_name}
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/${check_name}.py)
  endforeach()

  set(NAVIGATION_CPP_SELF_CHECKS
    gnss_global_fusion_math_self_check
    gnss_vyaw_math_self_check
    stage2_gnss_integrity_math_self_check
    steering_physical_lut_math_self_check
    steering_physical_math_self_check
    steering_physical_part3_math_self_check)

  foreach(check_name IN LISTS NAVIGATION_CPP_SELF_CHECKS)
    add_executable(${check_name} test/${check_name}.cpp)
    target_compile_features(${check_name} PRIVATE cxx_std_17)
    add_test(NAME ${check_name} COMMAND ${check_name})
  endforeach()
  add_executable(gui_navigation_tuning_catalog_self_check test/gui_navigation_tuning_catalog_self_check.cpp)
  target_compile_features(gui_navigation_tuning_catalog_self_check PRIVATE cxx_std_17)
  target_link_libraries(gui_navigation_tuning_catalog_self_check Qt5::Core)
  add_test(NAME gui_navigation_tuning_catalog_self_check COMMAND gui_navigation_tuning_catalog_self_check)

  add_test(
    NAME humble_launch_condition_contract_self_check
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/humble_launch_condition_contract_self_check.py)
endif()


install(TARGETS
    gnss_node
    imu_node
    imu_speed_diagnostic
    mag_heading_fusion
    joint_state_visualizer
    localization_core
    cmd_vel_router
    navigation_core
    mppi_closed_loop_supervisor
    vehicle_dynamics_observer
    trajectory_safety_supervisor
    global_field_test_localizer
    global_test_odom_bridge
    agv_gui
    agv_web_gui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch config rviz urdf maps behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY docs tools DESTINATION share/${PROJECT_NAME})

# Static browser assets untuk agv_web_gui.
install(DIRECTORY web/static DESTINATION share/${PROJECT_NAME}/web)

# GUI assets are shared resources for the native C++/Qt HMI.
install(
  PROGRAMS
    tools/experiment_rank.py
    tools/minipc_nav2_preflight.py
    tools/nav2_gui_flow_check.sh
    tools/rosbag_regression.py
    tools/runtime_profile.py
    tools/serial_usb_preflight.py
    tools/stage1_commissioning_check.py
    tools/stage2_commissioning_check.py
    tools/stage2_motion_csv_analyzer.py
    tools/stage3_commissioning_check.py
    tools/stage3_nav2_csv_analyzer.py
    tools/stage3_certify.py
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY gui/assets
  DESTINATION share/${PROJECT_NAME}/gui)

ament_package()
