#!/usr/bin/env python3
"""Exercise autonomous.launch.py EKF validator without importing ROS launch modules."""
from __future__ import annotations

import ast
import copy
import math
from pathlib import Path
import tempfile
import yaml

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "autonomous.launch.py"
EKF = ROOT / "config" / "ekf.yaml"

source = LAUNCH.read_text(encoding="utf-8")
tree = ast.parse(source)
fn = next(
    node for node in tree.body
    if isinstance(node, ast.FunctionDef) and node.name == "_validate_ekf_params"
)
module = ast.Module(body=[fn], type_ignores=[])
ast.fix_missing_locations(module)
namespace = {"yaml": yaml, "math": math}
exec(compile(module, str(LAUNCH), "exec"), namespace)
validate = namespace["_validate_ekf_params"]

# Final project contract: local EKF may fuse optional ESC odometry; validator must accept it.
data = yaml.safe_load(EKF.read_text(encoding="utf-8"))
local = data["ekf_filter_node_odom"]["ros__parameters"]
assert local.get("odom0") == "/esc/odom" and len(local.get("odom0_config", [])) == 15
validate(str(EKF))

# But any sensor that IS configured must still have exactly 15 booleans.
broken = copy.deepcopy(data)
broken["ekf_filter_node_odom"]["ros__parameters"]["twist0_config"] = [False] * 14
with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as handle:
    yaml.safe_dump(broken, handle, sort_keys=False)
    bad_path = handle.name
try:
    try:
        validate(bad_path)
    except RuntimeError as exc:
        assert "twist0_config must contain exactly 15 booleans" in str(exc)
    else:
        raise AssertionError("validator accepted malformed twist0_config")
finally:
    Path(bad_path).unlink(missing_ok=True)

print("EKF LAUNCH VALIDATION SELF-CHECK: PASS")
