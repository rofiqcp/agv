#!/usr/bin/env python3
from pathlib import Path
import re, yaml
from web_static_bundle import read_app_bundle, read_css_bundle
ROOT=Path(__file__).resolve().parents[1]
WEB=ROOT/'web'; STATIC=WEB/'static'
app=read_app_bundle(STATIC); html=(STATIC/'index.html').read_text(); css=read_css_bundle(STATIC)
replay=(STATIC/'replay.js').read_text(); geom=(STATIC/'camera_geometry.js').read_text(); analysis=(STATIC/'analysis_session.js').read_text(); diag=(STATIC/'diagnostics.js').read_text(); cpp=(WEB/'web_server.cpp').read_text()
meta=yaml.safe_load((WEB/'config/ui_parameter_metadata.yaml').read_text()) or {}; params=meta.get('parameters') or {}

def need(text,needle): assert needle in text, needle
# Load order: pure helpers must exist before app orchestration; replay remains after app.
order=[html.index('/camera_geometry.js'),html.index('/analysis_session.js'),html.index('/diagnostics.js'),html.index('/core.js'),html.index('/navigation.js'),html.index('/perception_calibration.js'),html.index('/tuning_catalog.js'),html.index('/config.js'),html.index('/boot.js'),html.index('/replay.js')]
assert order==sorted(order), order
# One camera acquisition owner, shared geometry, no double-mirrored overlay.
need(geom,'class CameraGeometry'); need(geom,'class CameraFrameStore'); need(app,'new AGVCameraFrameStore')
assert 'cameraFrameStore?.tick' not in app
assert not re.search(r'setInterval\([^\n]*(updateCamera|updateLabCamera|updateObstacleCalCamera)',app)
need(app,'screenToImage(e.clientX,e.clientY)'); assert 'naturalWidth-displayX' not in app
need(app,'function drawObstacleCalibrationOverlay()'); need(app,'normalizedToScreen')
need(css,'#obcalOverlay{transform:none}'); assert '#obcalImage,#obcalOverlay{transform:scaleX(-1)' not in css
# Direct perception calibration panel must be real, not dead markup.
for cid in ['perceptionCalLaneMode','perceptionCalSave','perceptionCalReload','perceptionCalReset','perceptionCalOverlay','perceptionObstacleCalTab','perceptionLaneCalTab','perceptionObstacleCalHost','laneSafetyCalibrationCard']:
    need(html,f'id="{cid}"'); need(app,cid)
for fn in ['bindPerceptionDirectCalibration','directCalPointerDown','directCalPointerMove','stagePerceptionDirectCalibration','captureHomographyPoint']:
    need(app,f'function {fn}')
# Generated calibration is server-owned proposal + Stage-1 transaction.
for source in ['perception:homography','perception:obstacle-distance','perception:lane-roi']:
    need(app,source); need(cpp,source)
for needle in ['/api/config/proposal','registerConfigProposal','generatedProposalItemValid','GENERATED_PROPOSAL_STALE_CONFIG','proposal_id','allowedPaths']:
    need(cpp,needle)
for ident in ['perception:perception.ros__parameters.nav2_obstacle_roi_points','perception:perception.ros__parameters.ground_src_points','perception:perception.ros__parameters.ground_dst_points','perception:perception.ros__parameters.obstacle_distance_calibration_coefficients','vehicle:vehicle.ros__parameters.drive_erpm_per_mps']:
    assert params[ident]['write_authority']=='calibration_generated', ident
# N2.1 scale remains proposal-only at Web boundary.
need(app,"writeRequest('/api/experiment/trial/optimal-scale',{apply:false})"); assert "writeRequest('/api/experiment/trial/optimal-scale',{apply:true})" not in app
need(cpp,'USE_CONFIG_TRANSACTION'); need(cpp,'navigation:N2.1')
for legacy in ['/api/config/set','/api/config/reset','/api/config/reset-batch']:
    assert legacy not in app and legacy not in (STATIC/'vesc_workbench.js').read_text(), legacy
# Evidence existence must not unlock phase; only effective qualification can.
need(app,"qualificationPass('steering','4.9')"); need(app,"qualificationPass('navigation','N16.1')"); need(app,"qualificationPass('perception','4.9')")
need(cpp,'taskQualified'); need(cpp,'effectiveQualification'); need(cpp,'config_fingerprint'); need(cpp,'evidence_trial_ids')
need(cpp,'membutuhkan minimal satu evidence trial yang valid')
assert "trialList(QStringLiteral(\"steering\"), QStringLiteral(\"4.9\")).isEmpty()" not in cpp
# Shared analysis cursor + replay isolation.
for needle in ['agv-analysis-cursor','setCursor(t,\'graph\')','CURRENT_RECORDING','selectTask(currentExp,id)']:
    need(app,needle)
for needle in ['registerReplayAdapter','syncReplay','setSource(\'REPLAY\')','setSource(\'LIVE\')']:
    need(replay,needle)
need(analysis,"source:'LIVE'"); need(analysis,'nearestEvidenceAt')
# Diagnostics/events are normalized and layered; raw inspector still remains available.
need(diag,'buildFindings'); need(diag,'normalizeEvent'); need(app,'AGVDiagnostics?.normalizeEvent'); need(app,'AGVDiagnostics?.buildFindings')
for layer in ['transport','actuator','localization','control','sensor','safety','authority']:
    need(diag,f"'{layer}'")
need(html,'id="rawState"')
# UI wording must reflect proposal/stage behavior.
for stale in ['Save + Apply YAML','Hitung + Apply Scale Optimal','READY TO SAVE & APPLY','Apply PASS Calibration']:
    assert stale not in html+app, stale
# Catch duplicate DOM ids introduced by panel patching.
ids=re.findall(r'\bid="([^"]+)"',html); dup=sorted({x for x in ids if ids.count(x)>1}); assert not dup, dup
# Light source-balance sanity without compiling/building.
assert cpp.count('{')==cpp.count('}'), (cpp.count('{'),cpp.count('}'))
assert cpp.count('(')==cpp.count(')'), (cpp.count('('),cpp.count(')'))
assert cpp.count('[')==cpp.count(']'), (cpp.count('['),cpp.count(']'))
print(f'PASS ROS Web Stage-2 source contract: {len(params)} metadata, {len(ids)} DOM ids, camera/proposal/qualification/analysis/diagnostics wired')
