#!/usr/bin/env python3
"""Static ROS 2 Humble launch-condition/runtime-backend contract.

Prevents the exact runtime failure:
  'PythonExpression' object has no attribute 'evaluate'
Actions require a launch.conditions.Condition, not a bare Substitution.
"""
from __future__ import annotations
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = ROOT.parent
launch_files = sorted(SRC_ROOT.rglob("*.launch.py"))


def fail(msg: str) -> None:
    raise SystemExit("FAIL: " + msg)


def is_name_or_attr(node: ast.AST, name: str) -> bool:
    if isinstance(node, ast.Name):
        return node.id == name
    if isinstance(node, ast.Attribute):
        return node.attr == name
    return False

bad_direct_conditions: list[str] = []
for path in launch_files:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        for kw in call.keywords:
            if kw.arg != "condition":
                continue
            value = kw.value
            # Bare PythonExpression is a Substitution. Action.execute() expects
            # Condition.evaluate(context), which causes the Humble runtime crash.
            if isinstance(value, ast.Call) and is_name_or_attr(value.func, "PythonExpression"):
                bad_direct_conditions.append(f"{path}:{call.lineno}")

if bad_direct_conditions:
    fail("bare condition=PythonExpression found: " + ", ".join(bad_direct_conditions))

autonomous = (ROOT / "launch/autonomous.launch.py").read_text(encoding="utf-8")
gui = (ROOT / "launch/gui.launch.py").read_text(encoding="utf-8")

# CPU runtime is direct TorchScript only. Converter is an explicit ros2 run tool,
# never a launch action/dependency of autonomous/gui.
for label, text in (("autonomous", autonomous), ("gui", gui)):
    low = text.lower()
    if "converter_pt_to_onnx_to_engine" in low:
        fail(f"{label} launch must never invoke/reference converter")
    if "onnx" in low:
        # Human log saying 'ONNX tidak digunakan' is allowed; no executable/path/probe.
        forbidden = (".onnx", "onnxruntime", "find_spec('onnx", 'find_spec("onnx')
        if any(token in low for token in forbidden):
            fail(f"{label} launch contains ONNX runtime dependency")

required = [
    "LaunchConfiguration('pt_model_path')",
    "perception_cpu_node",
    "IfCondition(perception_cpu_enabled)",
]
for token in required:
    if token not in autonomous:
        fail(f"autonomous direct-PT contract missing: {token}")

print(f"PASS ROS 2 Humble launch condition contract ({len(launch_files)} launch files)")
