#!/usr/bin/env python3
from pathlib import Path
from gui_source_helper import read_gui_source

ROOT = Path(__file__).resolve().parents[1]
GUI = read_gui_source(ROOT)
MAIN = (ROOT / 'gui/modules/main_window.cpp').read_text(encoding='utf-8')
CAT = (ROOT / 'gui/agv_experiment_catalog.hpp').read_text(encoding='utf-8')

checks = {
    'single popup menu exists': 'menuPopup_=new QFrame(this,Qt::Popup)' in MAIN,
    'three-tab widget exists': 'menuTabs_=new QTabWidget()' in MAIN,
    'navigation tab': 'addSubsystemMenu(QStringLiteral("navigation"),QStringLiteral("Navigasi"))' in MAIN,
    'perception tab': 'addSubsystemMenu(QStringLiteral("perception"),QStringLiteral("Persepsi"))' in MAIN,
    'esc tab': 'addSubsystemMenu(QStringLiteral("steering"),QStringLiteral("ESC"))' in MAIN,
    'menu built from experiment catalog': 'buildExperimentCatalog(subsystem)' in MAIN,
    'leaf routes to existing experiment workspace': 'selectExperimentLeaf(parts[1],parts[2],true)' in MAIN,
    'last leaf persists in yaml': 'navigation_menu.active_subsystem' in MAIN and 'navigation_menu.active_leaf' in MAIN,
    'no legacy tools button': 'Tools lama' not in GUI,
    'no legacy popup': 'legacyPopup_' not in GUI,
    'no permanent report tree': 'reportTree' not in MAIN and 'buildReportTree' not in MAIN,
    'catalog still has navigation 4.1': '4.1 Pengujian Sensor' in CAT,
    'catalog still has perception 4.1': '4.1 Pengujian Pipeline Kamera dan YOLOPv2' in CAT,
    'catalog has ESC Ackermann 4.1': '4.1 Alur Pengujian Ackermann dan Aturan Penguncian Parameter' in CAT,
}
for name, ok in checks.items():
    print(('PASS' if ok else 'FAIL'), name)
if not all(checks.values()):
    raise SystemExit(1)
print(f'PASS floating BAB IV menu: {sum(checks.values())}/{len(checks)} checks')
