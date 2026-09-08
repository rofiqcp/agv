#!/usr/bin/env python3
from pathlib import Path
import re, sys
root=Path(__file__).resolve().parents[1]
html=(root/'web/static/index.html').read_text()
js=(root/'web/static/app.js').read_text()
vesc=(root/'web/static/vesc_workbench.js').read_text()
css=(root/'web/static/vesc_mp_theme.css').read_text()
def req(ok,msg):
    if not ok: print('FAIL:',msg); sys.exit(1)
for token in ['workspaceTabs','testPhaseTabs','testFamilyTabs','vescUtilitySection','vescTerminalCard']:
    req(f'id="{token}"' in html, f'missing DOM {token}')
for token in ['WORKSPACE_TABS','activateWorkspaceTab','experimentPhase','phaseLabel','testPhaseMode']:
    req(token in js, f'missing JS {token}')
for label in ['Connection / FW','Terminal','FOC','PID Controllers','MC / App Config','Map / Mission','EKF Local','EKF Global','Planner','MPPI','Safety','Command Chain','Metric Calibration','Performance','Detection']:
    req(label in js, f'missing engineering tab {label}')
for token in ["TERMINAL:${m}:${text}","FW:1","FW:2","ALIVE:1"]:
    req(token in vesc, f'missing VESC utility contract {token}')
ids=re.findall(r'id="([^"]+)"',html)
req(len(ids)==len(set(ids)),'duplicate DOM id')
req('.workspace-tabs' in css and '.test-phase-tabs' in css and '.test-family-tabs' in css and '.test-role' in css,'tab styling missing')
for token in ['navInspectorEkfLocal','navInspectorEkfGlobal','navInspectorMppi','navInspectorSafety','navInspectorCommand']:
    req(f'id="{token}"' in html, f'missing navigation inspector {token}')
req("foundation:'Foundation / Sensors'" in js and "certification:'End-to-End / Certification'" in js,'navigation phase taxonomy missing')
req("setup:'Setup / Calibration'" in js and "foc:'FOC Current Loop'" in js,'ESC phase taxonomy missing')
print('PASS web_workspace_tabs_self_check')
print('layers: domain -> engineering workspace -> test phase -> test family -> test leaf')
