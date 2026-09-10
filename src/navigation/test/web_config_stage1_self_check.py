#!/usr/bin/env python3
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
web=ROOT/'web'; app=(web/'static/app.js').read_text(); vesc=(web/'static/vesc_workbench.js').read_text(); html=(web/'static/index.html').read_text(); cpp=(web/'web_server.cpp').read_text()
meta=yaml.safe_load((web/'config/ui_parameter_metadata.yaml').read_text())
assert meta.get('version')==2
params=meta.get('parameters') or {}
assert len(params)>=240, len(params)
for identity,m in params.items():
    assert identity==m.get('identity')
    assert identity==f"{m.get('file_key')}:{m.get('yaml_path')}"
    assert m.get('metadata_complete') is True
    assert m.get('level') in {'basic','advanced','expert'}
    assert m.get('apply_mode') in {'node_restart','startup_only','read_only'}
    assert m.get('write_authority') in {'ros_yaml','calibration_generated','read_only'}
for src in (app,vesc):
    for legacy in ['/api/config/set','/api/config/reset','/api/config/reset-batch']:
        assert legacy not in src, legacy
for needle in ['configDiffDrawer','configDiffApply','configDiffRevalidate']: assert needle in html
for needle in ['validation_id','config_revision','CONFIG_CONFLICT','EXTERNAL_RUNTIME_CHANGE','configDraftSignature','configRuntimeState']: assert needle in cpp
assert 'USE_CONFIG_TRANSACTION' in cpp and 'proposal_items' in cpp
assert 'Legacy direct config write dinonaktifkan' in cpp
assert 'request.path == "/api/config/set" || request.path == "/api/config/reset" || request.path == "/api/config/reset-batch"' in cpp
assert 'calculateOptimalScale(bool apply' not in cpp
assert 'calculateOptimalScaleProposal' in cpp
assert 'CONFIG_REVISION_CHANGED' in cpp and 'dependency_results' in cpp
assert 'list_length' in cpp and 'array_bounds' in cpp
assert "body:JSON.stringify({apply:false})" in app
assert "body:JSON.stringify({apply:true})" not in app
assert "if(/topic|frame|plugin|source|debug|raw/.test(text))" not in app
assert 'metadata_complete' in app and "write_authority!=='ros_yaml'" in app

# Authoritative metadata must resolve to a real source YAML leaf.
FILES={
 'vehicle':ROOT/'config/vehicle.yaml','navigation_core':ROOT/'config/navigation_core.yaml','nav2':ROOT/'config/nav2_ackermann.yaml',
 'ekf':ROOT/'config/ekf.yaml','localization':ROOT/'config/localization_cpp.yaml','gnss':ROOT/'config/gnss.yaml',
 'imu':ROOT/'config/imu.yaml','mag_heading':ROOT/'config/mag_heading.yaml','imu_speed':ROOT/'config/imu_speed.yaml',
 'stage3':ROOT/'config/stage3_navigation.yaml','trajectory_safety':ROOT/'config/trajectory_safety.yaml',
 'collision':ROOT/'config/collision_monitor_production.yaml','mppi_closed_loop':ROOT/'config/mppi_closed_loop.yaml',
 'gui':ROOT/'config/gui_calibration.yaml','esc':ROOT.parent/'esc/config/ackermann.yaml','teleop':ROOT.parent/'esc/config/teleop.yaml',
 'foc_thesis':ROOT.parent/'esc/config/foc_thesis.yaml','vesc_tool':ROOT.parent/'esc/config/vesc_tool.yaml',
 'hmi':ROOT.parent/'stmf4/config/hmi.yaml','perception':ROOT.parent/'perception/config/astra_yolop_gpu.yaml',
 'bbox_calibration':ROOT.parent/'perception/config/bbox_obstacle_calibration.yaml'}
def leaf(data,path):
    cur=data
    for part in path.split('.'):
        if isinstance(cur,list): cur=cur[int(part)]
        else: cur=cur[part]
    return cur
loaded={}
for identity,m in params.items():
    f=FILES[m['file_key']]; assert f.is_file(), (identity,f)
    loaded.setdefault(m['file_key'],yaml.safe_load(f.read_text()))
    leaf(loaded[m['file_key']],m['yaml_path'])
    if 'hard_min' in m and 'hard_max' in m: assert m['hard_min'] <= m['hard_max'], identity
    for dep in m.get('dependencies',[]):
        assert dep['identity'] in params, (identity,dep['identity'])
        assert dep['operator'] in {'eq','neq','gt_candidate','gte_candidate','lt_candidate','lte_candidate','abs_gte_candidate','lte_abs_candidate','gte_candidate_plus','lte_candidate_minus'}
    if 'list_length' in m:
        assert int(m['list_length']) > 0
        for b in m.get('array_bounds',[]): assert 0 <= int(b['index']) < int(m['list_length'])

# Safety-critical Stage-1 authority/constraints.
assert params['esc:esc_ackermann.ros__parameters.drive_odometry_calibration_scale']['hard_min']==0.20
assert params['esc:esc_ackermann.ros__parameters.drive_odometry_calibration_scale']['hard_max']==5.00
for ident in ['perception:perception.ros__parameters.ground_src_points','perception:perception.ros__parameters.ground_dst_points',
              'perception:perception.ros__parameters.obstacle_distance_calibration_coefficients',
              'perception:perception.ros__parameters.camera_metric_calibration_validated']:
    assert params[ident]['write_authority']=='calibration_generated', ident
assert params['perception:perception.ros__parameters.ground_src_points']['list_length']==8
assert params['perception:perception.ros__parameters.ground_dst_points']['list_length']==8
assert params['perception:perception.ros__parameters.obstacle_distance_calibration_coefficients']['list_length']==4
assert params['perception:perception.ros__parameters.lane_safety_enabled'].get('dependencies')
assert "generated:true" in app

print(f'PASS web config stage1 contract: {len(params)} authoritative metadata entries')
