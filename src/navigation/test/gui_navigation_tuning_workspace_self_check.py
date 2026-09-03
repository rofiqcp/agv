#!/usr/bin/env python3
"""Static contract for BAB-IV navigation tuning workspace behavior."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
COMP = (ROOT / "gui/modules/experiment_components.cpp").read_text(encoding="utf-8")
WORK = (ROOT / "gui/modules/system_and_gnss_pages.cpp").read_text(encoding="utf-8")
MAIN = (ROOT / "gui/modules/main_window.cpp").read_text(encoding="utf-8")
CAT = (ROOT / "gui/agv_experiment_catalog.hpp").read_text(encoding="utf-8")

checks = {
    "YAML editor loads actual value": "if (yamlValue.isValid()) edit->setText(editorText(yamlValue));" in COMP,
    "YAML commits on editingFinished": "&QLineEdit::editingFinished" in COMP,
    "typed bool YAML support": 'fkind == QStringLiteral("bool")' in COMP,
    "typed integer YAML support": 'fkind == QStringLiteral("int")' in COMP,
    "typed float YAML support": 'fkind == QStringLiteral("float")' in COMP,
    "YAML commit event exists": "yamlParameterCommitted" in COMP,
    "graph selector exists": "graphSelector_ = new NoWheelComboBox()" in WORK,
    "Start CSV label exists": 'QStringLiteral("● Start CSV")' in WORK,
    "Stop CSV autosave label exists": 'QStringLiteral("■ Stop CSV + Auto Save")' in WORK,
    "Stop triggers evidence autosave": "saveEvidence(false);" in WORK,
    "autosave exports every table": '"table_csv_files"' in WORK and "tableCsvPaths" in WORK,
    "autosave exports every graph": "for (int i = 0; i < graphCards_.size(); ++i)" in WORK,
    "autosave status exists": "CSV AUTO-SAVED" in WORK,
    "leaf sample-rate override": 'parameterValue(QStringLiteral("sample_rate"))' in WORK,
    "menu exposes counts": 'spec.tableColumns.size()).arg(spec.graphCaptions.size())' in MAIN,
    "4.10 has zero graph UX": "0 grafik" in WORK,
    "planner tuning metadata": "GridBased.minimum_turning_radius" in CAT,
    "MPPI tuning metadata": "PathAlignCritic.cost_weight" in CAT,
    "velocity smoother tuning metadata": "velocity_smoother.ros__parameters.smoothing_frequency" in CAT,
    "goal checker tuning metadata": "goal_checker.xy_goal_tolerance" in CAT,
}
failed = [name for name, ok in checks.items() if not ok]
if failed:
    for name in failed:
        print(f"FAIL: {name}")
    raise SystemExit(1)
print(f"PASS: {len(checks)} navigation tuning workspace checks")
