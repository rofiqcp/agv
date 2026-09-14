#!/usr/bin/env python3
"""Shared Stage-6 ROS Web source manifest for dependency-light self-checks."""

APP_MODULES = (
    "core.js", "navigation.js", "perception.js", "sensors.js", "esc.js",
    "system_views.js", "perception_calibration.js", "imu_calibration.js",
    "tuning_catalog.js", "tuning_ui.js", "tuning_charts.js",
    "tuning_trials.js", "tuning_evidence.js", "config.js", "boot.js",
)

FOUNDATION_MODULES = (
    "tokens.css", "base.css", "shell.css", "components.css",
    "responsive.css", "accessibility.css", "workspaces.css",
    # Workbench-specific responsive contracts are part of the browser bundle too.
    "experiments.css", "evidence.css",
)

def read_app_bundle(static_root, encoding="utf-8"):
    return "\n".join((static_root / name).read_text(encoding=encoding) for name in APP_MODULES)

def read_css_bundle(static_root, encoding="utf-8"):
    names = ("styles.css",) + FOUNDATION_MODULES
    return "\n".join((static_root / name).read_text(encoding=encoding) for name in names)
