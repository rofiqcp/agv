#include "../gui/agv_experiment_catalog.hpp"
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

int main() {
  const auto catalog = buildExperimentCatalog(QStringLiteral("navigation"));
  const std::vector<std::string> orderedIds = {
    "N0.1","N1.1","N2.1","N3.1","N3.2","N4.1","N4.2","N5.1","N5.2","N6.1",
    "N7.1","N7.2","N7.3","N7.4","N8.1","N8.2","N8.3","N8.4","N9.1","N9.2",
    "N10.1","N10.2","N11.1","N11.2","N12.1","N12.2","N12.3","N13.1","N13.2",
    "N14.1","N14.2","N15.1","N16.1","N17.1"
  };
  if (catalog.size() != static_cast<int>(orderedIds.size())) {
    std::cerr << "navigation leaf count=" << catalog.size()
              << " expected=" << orderedIds.size() << "\n";
    return 1;
  }
  const std::map<std::string, std::pair<int,int>> expectedCounts = {
    {"N0.1",{1,3}},{"N1.1",{1,0}},{"N2.1",{2,4}},{"N3.1",{1,4}},
    {"N3.2",{2,4}},{"N4.1",{2,4}},{"N4.2",{1,2}},{"N5.1",{2,4}},
    {"N5.2",{2,3}},{"N6.1",{2,2}},{"N7.1",{2,3}},{"N7.2",{2,4}},
    {"N7.3",{2,6}},{"N7.4",{1,3}},{"N8.1",{1,3}},{"N8.2",{2,4}},
    {"N8.3",{2,6}},{"N8.4",{1,3}},{"N9.1",{1,2}},{"N9.2",{1,3}},
    {"N10.1",{1,0}},{"N10.2",{1,0}},{"N11.1",{1,2}},{"N11.2",{1,2}},
    {"N12.1",{1,3}},{"N12.2",{1,2}},{"N12.3",{1,3}},{"N13.1",{1,2}},
    {"N13.2",{1,2}},{"N14.1",{1,2}},{"N14.2",{1,1}},{"N15.1",{1,2}},
    {"N16.1",{1,4}},{"N17.1",{1,0}}
  };

  std::set<std::string> groups;
  for (int i = 0; i < catalog.size(); ++i) {
    const auto &spec = catalog.at(i);
    const std::string id = spec.id.toStdString();
    if (id != orderedIds.at(static_cast<size_t>(i))) {
      std::cerr << "order mismatch at " << i << ": got=" << id
                << " expected=" << orderedIds.at(static_cast<size_t>(i)) << "\n";
      return 2;
    }
    const auto it = expectedCounts.find(id);
    if (it == expectedCounts.end()) return 3;
    if (spec.tableColumns.size() != it->second.first || spec.graphCaptions.size() != it->second.second) {
      std::cerr << id << " tables/graphs=" << spec.tableColumns.size() << "/"
                << spec.graphCaptions.size() << " expected=" << it->second.first << "/"
                << it->second.second << "\n";
      return 4;
    }
    if (spec.parameterFields.isEmpty()) {
      std::cerr << id << " has no staged parameter fields\n";
      return 5;
    }
    if (!spec.graphs.isEmpty() && spec.graphs.size() != spec.graphCaptions.size()) {
      std::cerr << id << " graph spec count mismatch\n";
      return 6;
    }
    for (const auto &graph : spec.graphs) {
      if (graph.xLabel.isEmpty() || graph.yLabel.isEmpty()) {
        std::cerr << id << " graph axis label is empty\n";
        return 7;
      }
      if (graph.type == QStringLiteral("time_series")) {
        if (!graph.xLabel.contains(QStringLiteral("Time [s]"), Qt::CaseInsensitive)) {
          std::cerr << id << " time-series x-axis is not Time [s]\n";
          return 10;
        }
        for (const QString &label : graph.series) {
          if (!spec.liveSeries.contains(label)) {
            std::cerr << id << " graph series has no live binding: " << label.toStdString() << "\n";
            return 11;
          }
        }
      } else if (graph.type == QStringLiteral("scatter")) {
        if (graph.xSeries.isEmpty() || graph.ySeries.isEmpty()) {
          std::cerr << id << " scatter source path missing\n";
          return 12;
        }
      }
    }
    groups.insert(spec.groupId.toStdString());
  }
  for (int n = 0; n <= 17; ++n) {
    if (!groups.count("N" + std::to_string(n))) {
      std::cerr << "missing staged group N" << n << "\n";
      return 8;
    }
  }

  const std::set<std::string> requiredPaths = {
    "ekf_filter_node_odom.ros__parameters.frequency",
    "ekf_filter_node_odom.ros__parameters.process_noise_covariance.96",
    "ekf_filter_node_map.ros__parameters.process_noise_covariance.0",
    "localization_core.ros__parameters.strict_correction_alpha",
    "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius",
    "local_costmap.local_costmap.ros__parameters.inflation_layer.inflation_radius",
    "planner_server.ros__parameters.GridBased.minimum_turning_radius",
    "controller_server.ros__parameters.FollowPath.PathAlignCritic.cost_weight",
    "velocity_smoother.ros__parameters.smoothing_frequency",
    "controller_server.ros__parameters.goal_checker.xy_goal_tolerance"
  };
  std::set<std::string> foundPaths;
  for (const auto &spec : catalog)
    for (const auto &field : spec.parameterFields)
      if (!field.yamlPath.isEmpty()) foundPaths.insert(field.yamlPath.toStdString());
  for (const auto &path : requiredPaths) {
    if (!foundPaths.count(path)) {
      std::cerr << "missing YAML tuning path: " << path << "\n";
      return 9;
    }
  }
  std::cout << "PASS: navigation staged workbench N0..N17, 34 leaves, table/graph counts, axes, groups, YAML paths\n";
  return 0;
}
