#!/usr/bin/env python3
"""Static contract for the lazy, in-process Nav2/RViz map page."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PANEL = (ROOT / "gui/modules/navigation_rviz_panel.cpp").read_text(encoding="utf-8")
MAIN = (ROOT / "gui/modules/main_window.cpp").read_text(encoding="utf-8")
GUI = (ROOT / "gui/agv_gui.cpp").read_text(encoding="utf-8")
CMAKE = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
PACKAGE = (ROOT / "package.xml").read_text(encoding="utf-8")
GUI_LAUNCH = (ROOT / "launch/gui.launch.py").read_text(encoding="utf-8")
AUTONOMOUS = (ROOT / "launch/autonomous.launch.py").read_text(encoding="utf-8")
NAV2 = (ROOT / "config/nav2_ackermann.yaml").read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


checks = [
    ("navigation_rviz_panel.cpp" in GUI, "embedded RViz module is not included"),
    ("class NavigationMapPage" in PANEL and "class EmbeddedNav2Panel" in PANEL,
     "Nav2 map wrapper/panel classes are missing"),
    ("NAV2 MAP" in MAIN and "mapTabIndex()" in MAIN,
     "direct NAV2 MAP navigation button is missing"),
    ("new NavigationMapPage" in MAIN and "mapPage_->setActive(mapSelected)" in MAIN,
     "MainWindow does not activate/deactivate the lazy renderer"),
    ("RosNodeAbstraction" in PANEL and "VisualizationManager::onUpdate()->spin_some()" in PANEL
     and "rvizExecutor_->add_node" not in PANEL,
     "embedded RViz must rely on VisualizationManager executor without double add_node"),
    ("manager_->startUpdate()" in PANEL and "manager_->stopUpdate()" in PANEL,
     "render updates are not paused outside the map page"),
    ("globalCostmapCheck_->setChecked(false)" in PANEL
     and "localCostmapCheck_->setChecked(true)" in PANEL,
     "global costmap must remain opt-in while lightweight local inflation is default ON"),
    (all(topic in PANEL for topic in (
        'QStringLiteral("/map")',
        'QStringLiteral("/robot_description")',
        'QStringLiteral("/navigation/goal_request")',
        'QStringLiteral("/plan")',
        'QStringLiteral("/controller_server/transformed_global_plan")',
        'QStringLiteral("/trajectories")')),
     "one or more required Nav2/URDF/MPPI topics are not displayed"),
    ("rviz_default_plugins/SetGoal" in PANEL and "/initialpose" in PANEL,
     "Goal Pose / Initial Pose tools are incomplete"),
    ("rviz_common" in CMAKE and "rviz_default_plugins" in CMAKE and "rviz_rendering" in CMAKE,
     "CMake RViz dependencies are incomplete"),
    (all(f"<depend>{dep}</depend>" in PACKAGE for dep in (
        "rviz_common", "rviz_default_plugins", "rviz_rendering")),
     "package.xml RViz dependencies are incomplete"),
    ('"enable_rviz": "false"' in GUI_LAUNCH,
     "gui.launch.py must not start a second rviz2 process"),
    ("IncludeLaunchDescription" in GUI_LAUNCH and "autonomous.launch.py" in GUI_LAUNCH,
     "GUI launch no longer includes the canonical autonomous launch"),
    ("robot_state_publisher" in AUTONOMOUS and "robot_description" in AUTONOMOUS
     and "joint_state_visualizer" in AUTONOMOUS,
     "canonical autonomous launch does not publish complete URDF/joint TF"),
    ("Transient Local" in PANEL and "Description Topic" in PANEL,
     "embedded RobotModel must receive late /robot_description via transient-local QoS"),
    ('name="agv_gui_process"' not in GUI_LAUNCH,
     "gui.launch.py must not apply a process-wide __node remap to embedded RViz"),
    ("nav2_smac_planner/SmacPlannerHybrid" in NAV2,
     "Smac Hybrid-A* planner is not configured"),
    ("nav2_mppi_controller::MPPIController" in NAV2 and "visualize: true" in NAV2,
     "MPPI controller/candidate visualization is not configured"),
    ("trajectory_step: 20" in NAV2,
     "sampled MPPI visualization contract changed unexpectedly"),
]

for name, message in checks:
    require(name, message)

print(f"PASS: {len(checks)} embedded Nav2/RViz GUI checks")
