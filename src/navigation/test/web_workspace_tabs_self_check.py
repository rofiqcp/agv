#!/usr/bin/env python3
from pathlib import Path
import re, sys

root=Path(__file__).resolve().parents[1]
html=(root/'web/static/index.html').read_text()
js=(root/'web/static/app.js').read_text()
css=(root/'web/static/styles.css').read_text()
vesc=(root/'web/static/vesc_workbench.js').read_text()

def req(ok,msg):
    if not ok: print('FAIL:',msg); sys.exit(1)

for token in ['workspaceTabs','tuningModebar','helpDrawer','vescUtilitySection','vescTerminalCard']:
    req(f'id="{token}"' in html, f'missing DOM {token}')
for token in ['WORKSPACE_TABS','WORKSPACE_PANES','activateWorkspaceTab','setWorkspacePaneVisibility','setPaneAccessible']:
    req(token in js, f'missing workspace contract {token}')
for token in ['setTuningMode','setTuningLevel','validateTuningDrafts','applyTuningDrafts','revertTuningDrafts']:
    req(token in js, f'missing tuning workflow {token}')
for label in ['Monitor','Health','Authority','Live','Control','Tune','Diagnostics','Map / Mission','Sensors','Localization','Planner','Controller','Safety','Calibration','Detection','Lane','Planning','Performance','Evidence']:
    req(label in js, f'missing workspace label {label}')
for token in ["TERMINAL:${m}:${text}","FW:1","FW:2","ALIVE:1"]:
    req(token in vesc, f'missing VESC utility contract {token}')
ids=re.findall(r'id="([^"]+)"',html)
req(len(ids)==len(set(ids)),'duplicate DOM id')
req('.workspace-tabs' in css and '.workspace-pane[hidden]' in css and '.tuning-modebar' in css and '.help-drawer' in css,'workspace styling missing')
req('vesc_mp_theme.css' not in html,'legacy theme still linked')
for token in ['navInspectorEkfLocal','navInspectorEkfGlobal','navInspectorMppi','navInspectorSafety','navInspectorCommand']:
    req(f'id="{token}"' in html, f'missing navigation inspector {token}')
req("foundation:'Foundation / Sensors'" in js and "certification:'End-to-End / Certification'" in js,'navigation phase taxonomy missing')
req("setup:'Setup / Calibration'" in js and "foc:'FOC Current Loop'" in js,'ESC phase taxonomy missing')
print('PASS web_workspace_tabs_self_check')
print('layers: domain -> true workspace pane/page -> task/tune/analyze -> basic/advanced/expert')
