#!/usr/bin/env python3
"""Explicit Stage-3 evidence writer. Never certifies a component without evidence."""
from __future__ import annotations
import argparse,datetime,hashlib,math
from pathlib import Path
import yaml
from stage3_nav2_csv_analyzer import analyze

def load(p): return yaml.safe_load(p.read_text()) or {}
def save(p,d): p.write_text(yaml.safe_dump(d,sort_keys=False,width=120))
def digest(p:Path):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--workspace',required=True); sub=ap.add_subparsers(dest='cmd',required=True)
    m=sub.add_parser('mppi');m.add_argument('--csv',required=True);m.add_argument('--profile',choices=['A','B','C'],required=True)
    mark=sub.add_parser('mark');mark.add_argument('component',choices=['trajectory_safety','collision_monitor','fault_injection','velocity_smoother_closed_loop']);mark.add_argument('--evidence',required=True)
    sub.add_parser('finalize'); sub.add_parser('revoke')
    a=ap.parse_args(); ws=Path(a.workspace).expanduser().resolve(); cfg=ws/'src/navigation/config'; p=cfg/'stage3_navigation.yaml'; d=load(p); q=d['stage3_navigation']['ros__parameters']; now=datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    if a.cmd=='mppi':
        r=analyze(Path(a.csv).expanduser(),p)
        if not r['pass']: print(r); return 2
        e=Path(a.csv).expanduser().resolve();q['mppi_profile_calibration_valid']=True;q['mppi_profile_calibration_profile']=a.profile;q['mppi_profile_calibration_saved_at']=now;q['mppi_profile_calibration_sample_count']=r['usable_samples'];q['mppi_profile_velocity_rmse_mps']=float(r['velocity_rmse_mps']);q['mppi_profile_steering_rmse_rad']=float(r['steering_rmse_rad']);q['mppi_profile_yaw_rate_rmse_rps']=float(r['yaw_rate_rmse_rps']);q['mppi_profile_evidence_file']=str(e);q['mppi_profile_evidence_sha256']=digest(e);q['production_autonomy_certified']=False;q['production_autonomy_certified_at']='';save(p,d);print('MPPI evidence PASS dan tersimpan; final production certification dicabut sampai semua gate PASS.');return 0
    if a.cmd=='mark':
        e=Path(a.evidence).expanduser().resolve()
        if not e.is_file() or e.stat().st_size==0: print('Evidence file tidak ada/kosong'); return 2
        if a.component=='velocity_smoother_closed_loop':
            q['velocity_smoother_closed_loop_certified']=True
            q['velocity_smoother_closed_loop_evidence_file']=str(e)
            q['velocity_smoother_closed_loop_evidence_sha256']=digest(e)
        else:
            key=a.component+'_calibration_valid'; t=a.component+'_calibration_saved_at'; q[key]=True; q[t]=now
            q[a.component+'_evidence_file']=str(e); q[a.component+'_evidence_sha256']=digest(e)
        q['production_autonomy_certified']=False;q['production_autonomy_certified_at']='';save(p,d);print(f'{a.component} evidence ditandai PASS: {e}');return 0
    if a.cmd=='revoke':
        for k in ['mppi_profile_calibration_valid','velocity_smoother_closed_loop_certified','trajectory_safety_calibration_valid','collision_monitor_calibration_valid','fault_injection_calibration_valid','production_autonomy_certified']:q[k]=False
        q['production_autonomy_certified_at']='';save(p,d);print('Stage3 certifications revoked');return 0
    # finalize: use preflight-equivalent persistent prerequisites
    v=(load(cfg/'vehicle.yaml')['vehicle']['ros__parameters']); im=(load(cfg/'imu.yaml')['data_imu_node']['ros__parameters']); loc=(load(cfg/'localization_cpp.yaml')['localization_core']['ros__parameters']); gui=load(cfg/'gui_calibration.yaml'); nc=load(cfg/'navigation_core.yaml')['navigation_core']['ros__parameters']
    need=[bool(v.get('steering_calibration_valid')),bool(v.get('steering_circle_calibration_valid')),bool(v.get('drive_odometry_calibration_valid')),bool(im.get('stationary_calibration_valid')),bool(loc.get('gnss_velocity_calibration_valid')),bool((gui.get('camera_metric_validation') or {}).get('certified',False)),bool(nc.get('require_camera_metric_calibration')),bool(nc.get('camera_metric_calibration_validated')),bool(nc.get('collision_monitor_enabled')),bool(q.get('mppi_profile_calibration_valid')),bool(q.get('trajectory_safety_calibration_valid')),bool(q.get('collision_monitor_calibration_valid')),bool(q.get('fault_injection_calibration_valid'))]
    if loc.get('enable_global_gnss_cog_fusion'): need.append(bool(loc.get('gnss_cog_calibration_valid')))
    if not all(need): print('FINALIZE ditolak: prerequisite/evidence belum lengkap'); return 2
    for prefix in ['mppi_profile','trajectory_safety','collision_monitor','fault_injection']:
        ep=Path(str(q.get(prefix+'_evidence_file',''))).expanduser()
        expected=str(q.get(prefix+'_evidence_sha256',''))
        if not ep.is_file() or len(expected)!=64 or digest(ep)!=expected:
            print(f'FINALIZE ditolak: evidence {prefix} hilang atau SHA-256 berubah'); return 2
    q['production_autonomy_certified']=True;q['production_autonomy_certified_at']=now;save(p,d);print('Stage3 production autonomy CERTIFIED. Restart autonomous.launch.py.');return 0
if __name__=='__main__': raise SystemExit(main())
