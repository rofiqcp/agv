#!/usr/bin/env python3
from gui_source_helper import read_gui_source
from pathlib import Path
import math, sys, yaml
ROOT=Path(__file__).resolve().parents[1]
def load(name,node): return (yaml.safe_load((ROOT/'config'/name).read_text()) or {})[node]['ros__parameters']
def fail(m): print('FAIL:',m); sys.exit(1)
veh=load('vehicle.yaml','vehicle')
nav=yaml.safe_load((ROOT/'config/nav2_ackermann.yaml').read_text())
nc=load('navigation_core.yaml','navigation_core')
traj=load('trajectory_safety.yaml','trajectory_safety_supervisor')
st3=load('stage3_navigation.yaml','stage3_navigation')
col=load('collision_monitor_production.yaml','collision_monitor')
gui=yaml.safe_load((ROOT/'config/gui_calibration.yaml').read_text()) or {}
ctrl=nav['controller_server']['ros__parameters']; m=ctrl['FollowPath']; sm=nav['velocity_smoother']['ros__parameters']; planner=nav['planner_server']['ros__parameters']['GridBased']
r=float(veh['minimum_turning_radius_m'])
for label,x in [('Smac',planner['minimum_turning_radius']),('MPPI',m['AckermannConstraints']['min_turning_r']),('NavigationCore',nc['minimum_turning_radius_m']),('TrajectorySafety',traj['minimum_turning_radius_m'])]:
    if not math.isclose(float(x),r,rel_tol=0,abs_tol=1e-7): fail(f'{label} Rmin != vehicle')
if str(planner['motion_model_for_search']).upper()!='DUBIN': fail('production baseline must be forward-only DUBIN')
if float(m['vx_min']) < -1e-9: fail('MPPI reverse enabled while Smac baseline is DUBIN')
if abs(float(m['model_dt'])-1.0/float(ctrl['controller_frequency']))>1e-9: fail('MPPI model_dt != controller period')
if float(m['wz_max']) > float(m['vx_max'])/r+1e-7: fail('MPPI yaw cap violates v/R')
if abs(float(sm['max_velocity'][2])) > float(sm['max_velocity'][0])/r+1e-7: fail('smoother yaw cap violates v/R')
if col['cmd_vel_in_topic']!='/cmd_vel/nav2_pre_collision' or col['cmd_vel_out_topic']!='/cmd_vel/autonomy_pre_smoother': fail('collision monitor command ownership invalid')
if traj['output_cmd_topic']!='/cmd_vel/autonomy_integrated': fail('trajectory safety command ownership invalid')
for src,token in [
    ((ROOT/'src/navigation_core.cpp').read_text(),'curvature_yaw_cap'),
    ((ROOT/'src/trajectory_safety_supervisor.cpp').read_text(),'curvature_yaw_cap')]:
    if token not in src: fail('dynamic Ackermann curvature clamp missing')
if nc.get('require_stage3_production_certification') is not True: fail('Stage3 production gate must be enabled')
if 'stage3_navigation.yaml' not in read_gui_source(ROOT): fail('Stage3 YAML missing from GUI config snapshot map')
if st3.get('production_autonomy_certified',False):
    req=['mppi_profile_calibration_valid','trajectory_safety_calibration_valid','collision_monitor_calibration_valid','fault_injection_calibration_valid']
    if not all(bool(st3.get(k,False)) for k in req): fail('production certified without all Stage3 evidence')
    for prefix in ['mppi_profile','trajectory_safety','collision_monitor','fault_injection']:
        if not st3.get(prefix+'_evidence_file') or len(str(st3.get(prefix+'_evidence_sha256',''))) != 64: fail(prefix+' evidence audit trail missing')
    if not bool(nc.get('require_camera_metric_calibration')) or not bool(nc.get('camera_metric_calibration_validated')): fail('production certified without camera metric runtime gate')
    if not bool(nc.get('collision_monitor_enabled')): fail('production certified without Collision Monitor runtime gate')
if str(sm.get('feedback','OPEN_LOOP')).upper()=='CLOSED_LOOP' and not bool(st3.get('velocity_smoother_closed_loop_certified',False)):
    fail('CLOSED_LOOP smoother enabled without persistent Stage3 certification')
print('PASS stage3_nav2_safety_self_check')
print(f'Rmin={r:.6f}m vx={float(m["vx_max"]):.3f}m/s wz={float(m["wz_max"]):.6f}rad/s feedback={sm.get("feedback")}')
