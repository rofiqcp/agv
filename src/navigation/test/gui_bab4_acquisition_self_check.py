#!/usr/bin/env python3
from gui_source_helper import read_gui_source
"""Static acceptance checks for the BAB IV GUI-only acquisition extension."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
GUI = read_gui_source(ROOT)
SPECS = (ROOT / "gui" / "agv_gui_specs.hpp").read_text(encoding="utf-8")
CATALOG = (ROOT / "gui" / "agv_experiment_catalog.hpp").read_text(encoding="utf-8")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def count_catalog(subsystem: str) -> int:
    return len(re.findall(rf'add\("{re.escape(subsystem)}"', CATALOG))


checks = [
    (count_catalog("navigation") == 50, "navigation catalog must contain 50 report leaves"),
    (count_catalog("perception") == 35, "perception catalog must contain 35 report leaves"),
    (count_catalog("steering") == 15, "steering catalog must contain 15 report leaves"),
    ('"4.9.1"' in CATALOG and '"4.9.4"' in CATALOG, "navigation section 4.9 must be covered"),
    (all(token in CATALOG for token in ('"|Target|"', '"e ss kiri"', '"e ss kanan"', '"Selisih rise time"', '"Kesimpulan"')),
     "steering symmetry table columns must preserve report notation"),
    (all(token in CATALOG for token in ('"Target"', '"Mean |error|"', '"Std"', '"Maksimum"', '"Jumlah run"')),
     "steering repeatability table columns must preserve report notation"),
    ("Gambar 4.24 Perbandingan error koreksi global baseline dan hasil tuning" in CATALOG,
     "navigation figure-only section 4.4.6 must be exposed"),
    ("Gambar 4.1 Diagram alir pipeline sistem persepsi visual" in CATALOG,
     "perception pipeline figure must be exposed"),
    ('"experiment_navigation"' in SPECS and '"experiment_perception"' in SPECS
     and '"experiment_steering"' in SPECS, "three BAB IV acquisition tabs must be registered"),
    ("class ExperimentWorkspacePage" in GUI, "shared acquisition workspace class is missing"),
    ("saveCsvOrdered" in GUI and "savePng" in GUI and '"graph_options"' in GUI,
     "ordered CSV, PNG, and manifest graph choices must be saved"),
    ('"/plan"' in GUI and '"/controller_server/transformed_global_plan"' in GUI,
     "navigation global and controller plan topics must be sampled"),
    ('"/perception/camera_health_state"' in GUI and '"/perception/near_field_state"' in GUI,
     "perception health and near-field topics must be sampled"),
    ('cloud("/perception/drivable_boundary_points","drivable_boundary_points")' in GUI,
     "drivable-boundary point count must be sampled"),
    ('"/esc/foc/telemetry"' in GUI and "Source ESC tidak mempublish" in GUI,
     "FOC telemetry availability must be explicit"),
    ("no estimated report values were injected" in GUI,
     "saved manifest must state that report estimates were not injected"),
]

for condition, message in checks:
    require(condition, message)

print(f"PASS: {len(checks)} BAB IV GUI acquisition checks")
sys.exit(0)
