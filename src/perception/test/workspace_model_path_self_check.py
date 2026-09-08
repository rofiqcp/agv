#!/usr/bin/env python3
from pathlib import Path
import re
ROOT = Path(__file__).resolve().parents[2]
expected = "models/yolopv2.pt"
yaml = (ROOT/"perception/config/astra_yolop_gpu.yaml").read_text()
assert "pt_model_path: auto" in yaml, "YAML PT model harus memakai auto discovery"
for rel in ["perception/launch/astra_yolop.launch.py", "navigation/launch/gui.launch.py", "navigation/launch/autonomous.launch.py", "perception/src/astra_yolop_cpu_pt_node.cpp", "perception/tools/perception_backend_preflight.py"]:
    text=(ROOT/rel).read_text()
    assert "src/perception/models/yolopv2.pt" not in text, f"source-model fallback masih ada: {rel}"
cpu=(ROOT/"perception/src/astra_yolop_cpu_pt_node.cpp").read_text(); assert "AGV_ROOT" in cpu and "models/yolopv2.pt" in cpu, "C++ CPU AGV_ROOT model discovery hilang"
model_script=(ROOT.parent/'models/model.sh').read_text()
assert 'MODEL_PATH="${SCRIPT_DIR}/yolopv2.pt"' in model_script, 'models/model.sh target tidak sesuai workspace/models'
print("PASS workspace model path contract: <workspace>/models/yolopv2.pt")
