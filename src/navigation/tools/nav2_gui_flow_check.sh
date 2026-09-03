#!/usr/bin/env bash
set -u

echo '=== NAV2 GUI DATA FLOW CHECK ==='
for n in map_server planner_server controller_server bt_navigator; do
  echo "--- lifecycle /$n"
  timeout 2 ros2 lifecycle get "/$n" 2>/dev/null || true
done

echo '--- nodes penting'
ros2 node list 2>/dev/null | grep -E 'agv_gui|embedded_rviz|robot_state_publisher|joint_state_visualizer|map_server|planner_server|controller_server|localization_core' || true

echo '--- topics visual/Nav2'
ros2 topic list 2>/dev/null | grep -E '^/map$|^/robot_description$|^/joint_states$|^/tf$|^/tf_static$|costmap|^/plan$|transformed_global_plan|^/trajectories$|published_footprint|planning_relevant_points|drivable_boundary_points' || true

echo '--- one-shot frequencies (2 s each)'
for t in /joint_states /local_costmap/costmap /local_costmap/costmap_updates /plan /controller_server/transformed_global_plan /trajectories; do
  echo "[$t]"
  timeout 3 ros2 topic hz "$t" 2>/dev/null | tail -n 2 || true
done

echo '--- TF chain'
timeout 3 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | head -n 12 || true
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | head -n 12 || true
