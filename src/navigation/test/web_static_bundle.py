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
    """Return the effective local CSS bundle, including linked files and @imports.

    Static checks must model what the browser actually loads.  Reading only the
    literal text of styles.css caused false regressions whenever a rule lived in
    an imported stylesheet.  Query strings are cache-busters, not filenames.
    """
    import re

    index = (static_root / "index.html").read_text(encoding=encoding)
    linked = re.findall(r'<link[^>]+rel=["\']stylesheet["\'][^>]+href=["\']([^"\']+)', index)
    imported = re.compile(r'@import\s+url\(["\']?([^"\')]+)')
    seen = set()
    chunks = []

    def add(ref):
        name = ref.split("?", 1)[0].lstrip("/")
        if not name or "://" in name or name in seen:
            return
        path = static_root / name
        if not path.is_file():
            return
        seen.add(name)
        text = path.read_text(encoding=encoding)
        for child in imported.findall(text):
            add(child)
        chunks.append(text)

    for ref in linked:
        add(ref)
    return "\n".join(chunks)
