#!/usr/bin/env python3
from pathlib import Path
import re, yaml
ROOT=Path(__file__).resolve().parents[1]
WEB=ROOT/'web'; STATIC=WEB/'static'
app=(STATIC/'app.js').read_text(); html=(STATIC/'index.html').read_text(); css=(STATIC/'styles.css').read_text()
replay=(STATIC/'replay.js').read_text(); geom=(STATIC/'camera_geometry.js').read_text(); analysis=(STATIC/'analysis_session.js').read_text(); diag=(STATIC/'diagnostics.js').read_text(); cpp=(WEB/'web_server.cpp').read_text(); imu=(ROOT/'tools/yahboom_apply_calibration.py').read_text()
meta=yaml.safe_load((WEB/'config/ui_parameter_metadata.yaml').read_text()) or {}; params=meta.get('parameters') or {}

def need(text,needle): assert needle in text, needle
# Load order: pure helpers must exist before app orchestration; replay remains after app.
order=[html.index('/camera_geometry.js'),html.index('/analysis_session.js'),html.index('/diagnostics.js'),html.index('/app.js'),html.index('/replay.js')]
assert order==sorted(order), order
# One camera acquisition owner, shared geometry, no double-mirrored overlay.
need(geom,'class CameraGeometry'); need(geom,'class CameraFrameStore'); need(app,'new AGVCameraFrameStore')
assert 'cameraFrameStore?.tick' not in app
assert not re.search(r'setInterval\([^\n]*(updateCamera|updateLabCamera|updateObstacleCalCamera)',app)
need(app,'screenToImage(e.clientX,e.clientY)'); assert 'naturalWidth-displayX' not in app
need(app,'function drawObstacleCalibrationOverlay()'); need(app,'normalizedToScreen')
need(css,'#obcalOverlay{transform:none}'); assert '#obcalImage,#obcalOverlay{transform:scaleX(-1)' not in css
# Direct perception calibration panel must be real, not dead markup.
for cid in ['perceptionCalLaneMode','perceptionCalObjectMode','perceptionCalSave','perceptionCalReload','perceptionCalReset','perceptionCalOverlay']:
    need(html,f'id="{cid}"'); need(app,cid)
for fn in ['bindPerceptionDirectCalibration','directCalPointerDown','directCalPointerMove','stagePerceptionDirectCalibration','captureHomographyPoint']:
    need(app,f'function {fn}')
# Generated calibration is server-owned proposal + Stage-1 transaction.
for source in ['perception:homography','perception:obstacle-distance','perception:lane-roi']:
    need(app,source); need(cpp,source)
for needle in ['/api/config/proposal','registerConfigProposal','generatedProposalItemValid','GENERATED_PROPOSAL_STALE_CONFIG','proposal_id','allowedPaths']:
    need(cpp,needle)
for ident in ['perception:perception.ros__parameters.nav2_obstacle_roi_points','perception:perception.ros__parameters.ground_src_points','perception:perception.ros__parameters.ground_dst_points','perception:perception.ros__parameters.obstacle_distance_calibration_coefficients','vehicle:vehicle.ros__parameters.drive_odometry_calibration_scale','mag_heading:mag_heading_fusion.ros__parameters.imu_mag_yaw_offset_rad']:
    assert params[ident]['write_authority']=='calibration_generated', ident
# N2.1 and Yahboom are proposal-only at Web boundary.
need(app,"body:JSON.stringify({apply:false})"); assert "body:JSON.stringify({apply:true})" not in app
need(cpp,'USE_CONFIG_TRANSACTION'); need(cpp,'/api/imu/calibration/proposal'); need(cpp,'navigation:N2.1'); need(cpp,'imu:yahboom')
need(imu,"add_argument('--propose'"); need(imu,"if args.propose")
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
