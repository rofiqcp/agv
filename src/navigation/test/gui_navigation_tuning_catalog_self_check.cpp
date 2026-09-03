#include "../gui/agv_experiment_catalog.hpp"
#include <iostream>
#include <map>
#include <set>
#include <string>

int main() {
  const auto catalog = buildExperimentCatalog(QStringLiteral("navigation"));
  if (catalog.size() != 50) {
    std::cerr << "navigation leaf count=" << catalog.size() << " expected=50\n";
    return 1;
  }
  const std::map<std::string, std::pair<int,int>> expected = {
    {"4.1.1",{2,2}}, {"4.1.2",{2,2}}, {"4.1.3",{2,2}},
    {"4.2.1",{1,1}}, {"4.2.2",{1,1}}, {"4.2.3",{1,1}},
    {"4.2.4",{2,1}}, {"4.2.5",{1,1}}, {"4.2.6",{1,1}},
    {"4.3.1",{1,1}}, {"4.3.2",{1,1}}, {"4.3.3",{1,1}},
    {"4.3.4",{1,1}}, {"4.3.5",{1,1}}, {"4.3.6",{1,1}},
    {"4.4.1",{1,1}}, {"4.4.2",{1,1}}, {"4.4.3",{1,1}},
    {"4.4.4",{1,1}}, {"4.4.5",{1,1}}, {"4.4.6",{1,1}},
    {"4.5.1",{1,1}}, {"4.5.2",{1,1}}, {"4.5.3",{1,1}},
    {"4.5.4",{1,1}}, {"4.6.1",{1,1}}, {"4.6.2",{1,1}},
    {"4.6.3",{1,1}}, {"4.6.4",{1,1}}, {"4.6.5",{1,1}},
    {"4.6.6",{1,1}}, {"4.6.7",{2,1}}, {"4.7.1",{1,1}},
    {"4.7.2",{1,1}}, {"4.7.3",{1,1}}, {"4.7.4",{1,1}},
    {"4.7.5",{1,1}}, {"4.7.6",{1,1}}, {"4.7.7",{1,1}},
    {"4.7.8",{1,1}}, {"4.8.1",{1,1}}, {"4.8.2",{1,1}},
    {"4.8.3",{1,1}}, {"4.8.4",{1,1}}, {"4.8.5",{1,1}},
    {"4.9.1",{1,1}}, {"4.9.2",{1,1}}, {"4.9.3",{1,1}},
    {"4.9.4",{1,1}}, {"4.10",{1,0}}
  };
  std::set<std::string> seen;
  for (const auto &spec : catalog) {
    const std::string id = spec.id.toStdString();
    const auto it = expected.find(id);
    if (it == expected.end()) continue;
    seen.insert(id);
    if (spec.tableColumns.size() != it->second.first ||
        spec.graphCaptions.size() != it->second.second) {
      std::cerr << id << " tables=" << spec.tableColumns.size()
                << " graphs=" << spec.graphCaptions.size() << " expected="
                << it->second.first << "/" << it->second.second << "\n";
      return 2;
    }
    if (id != "4.10" && spec.parameterFields.isEmpty()) {
      std::cerr << id << " has no tuning/acquisition parameter fields\n";
      return 3;
    }
  }
  if (seen.size() != expected.size()) {
    std::cerr << "covered requested leaves=" << seen.size()
              << " expected=" << expected.size() << "\n";
    return 4;
  }
  const std::set<std::string> requiredPaths = {
    "ekf_filter_node_odom.ros__parameters.frequency",
    "ekf_filter_node_map.ros__parameters.odom0_pose_rejection_threshold",
    "localization_core.ros__parameters.strict_correction_alpha",
    "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius",
    "planner_server.ros__parameters.GridBased.minimum_turning_radius",
    "controller_server.ros__parameters.FollowPath.PathAlignCritic.cost_weight",
    "velocity_smoother.ros__parameters.smoothing_frequency",
    "controller_server.ros__parameters.goal_checker.xy_goal_tolerance"
  };
  std::set<std::string> foundPaths;
  for (const auto &spec : catalog)
    for (const auto &field : spec.parameterFields)
      foundPaths.insert(field.yamlPath.toStdString());
  for (const auto &path : requiredPaths) {
    if (!foundPaths.count(path)) {
      std::cerr << "missing YAML tuning path: " << path << "\n";
      return 5;
    }
  }
  std::cout << "PASS: navigation BAB-IV 50 leaves, requested table/graph counts, tuning YAML fields\n";
  return 0;
}
