#!/usr/bin/env python3
"""Read-only Stage-3 production preflight. WAIT is not a configuration error."""
from __future__ import annotations
import argparse,hashlib,math
from pathlib import Path
import yaml

def wsfind(x):
    if x:
        p=Path(x).expanduser().resolve()
        if (p/'src/navigation/config').is_dir(): return p
        raise FileNotFoundError(p)
    for p in (Path.cwd(),*Path.cwd().parents):
        if (p/'src/navigation/config').is_dir(): return p
    h=Path(__file__).resolve()
    for p in (h,*h.parents):
        if (p/'src/navigation/config').is_dir(): return p
    raise FileNotFoundError('gunakan --workspace')
def pars(p,n): return (yaml.safe_load(p.read_text()) or {})[n]['ros__parameters']
def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workspace');ap.add_argument('--require-ready',action='store_true');a=ap.parse_args();ws=wsfind(a.workspace);c=ws/'src/navigation/config'
    v=pars(c/'vehicle.yaml','vehicle'); imu=pars(c/'imu.yaml','data_imu_node'); loc=pars(c/'localization_cpp.yaml','localization_core'); nc=pars(c/'navigation_core.yaml','navigation_core'); st=pars(c/'stage3_navigation.yaml','stage3_navigation'); nav=yaml.safe_load((c/'nav2_ackermann.yaml').read_text()); gui=yaml.safe_load((c/'gui_calibration.yaml').read_text()) or {}
    ctrl=nav['controller_server']['ros__parameters']['FollowPath']; sm=nav['velocity_smoother']['ros__parameters']; errs=[]; waits=[]
    def req(ok,msg): errs.append(msg) if not ok else None
    def ready(ok,msg): waits.append(msg) if not ok else None
    r=float(v['minimum_turning_radius_m']); req(float(ctrl['wz_max'])<=float(ctrl['vx_max'])/r+1e-7,'MPPI curvature cap invalid'); req(float(sm['max_velocity'][2])<=float(sm['max_velocity'][0])/r+1e-7,'smoother curvature cap invalid'); req(nc.get('require_stage3_production_certification') is True,'Stage3 production gate disabled')
    # Stage1/2 prerequisites; COG only required if requested.
    for ok,msg in [(v.get('steering_calibration_valid'), 'steering physical'),(v.get('steering_circle_calibration_valid'),'circle geometry'),(v.get('drive_odometry_calibration_valid'),'drive odometry'),(imu.get('stationary_calibration_valid'),'IMU stationary'),(loc.get('gnss_velocity_calibration_valid'),'GNSS velocity')]: ready(bool(ok),msg+' belum certified')
    if loc.get('enable_global_gnss_cog_fusion'): ready(bool(loc.get('gnss_cog_calibration_valid')),'GNSS COG fusion aktif tetapi COG belum certified')
    camera=bool((gui.get('camera_metric_validation') or {}).get('certified',False));
    if st.get('require_camera_metric_for_production',True):
        ready(camera,'camera metric belum certified')
        ready(bool(nc.get('require_camera_metric_calibration')),'NavigationCore camera metric gate belum diaktifkan')
        ready(bool(nc.get('camera_metric_calibration_validated')),'NavigationCore camera metric belum validated')
    if st.get('require_collision_monitor_for_production',True): ready(bool(nc.get('collision_monitor_enabled')),'Collision Monitor production belum diaktifkan pada NavigationCore')
    for k,label in [('mppi_profile_calibration_valid','MPPI profile'),('trajectory_safety_calibration_valid','trajectory safety'),('collision_monitor_calibration_valid','collision monitor'),('fault_injection_calibration_valid','fault injection')]: ready(bool(st.get(k,False)),label+' belum certified')
    all_component=not waits
    req(not st.get('production_autonomy_certified',False) or all_component,'production_autonomy_certified=true tetapi evidence belum lengkap')
    for prefix,valid_key in [('mppi_profile','mppi_profile_calibration_valid'),('trajectory_safety','trajectory_safety_calibration_valid'),('collision_monitor','collision_monitor_calibration_valid'),('fault_injection','fault_injection_calibration_valid')]:
        if st.get(valid_key,False):
            ep=Path(str(st.get(prefix+'_evidence_file',''))).expanduser(); expected=str(st.get(prefix+'_evidence_sha256',''))
            req(bool(ep.is_file()) and len(expected)==64 and (digest(ep)==expected if ep.is_file() and len(expected)==64 else False),prefix+' certified tetapi evidence hilang/berubah')
    if str(sm.get('feedback','OPEN_LOOP')).upper()=='CLOSED_LOOP':
        req(bool(st.get('velocity_smoother_closed_loop_certified',False)),'CLOSED_LOOP tanpa certification')
        if st.get('velocity_smoother_closed_loop_certified',False):
            ep=Path(str(st.get('velocity_smoother_closed_loop_evidence_file',''))).expanduser(); expected=str(st.get('velocity_smoother_closed_loop_evidence_sha256',''))
            req(bool(ep.is_file()) and len(expected)==64 and (digest(ep)==expected if ep.is_file() and len(expected)==64 else False),'CLOSED_LOOP certified tetapi evidence hilang/berubah')
    print('STAGE-3 NAV2/SAFETY PREFLIGHT'); print('workspace:',ws); print('production_certified:',bool(st.get('production_autonomy_certified',False))); print('commissioning default:',bool(st.get('commissioning_mode_default',False))); print('smoother:',sm.get('feedback'))
    if errs:
        print('\nCONFIG ERROR:');[print(' -',x) for x in errs];return 2
    if waits:
        print('\nCONFIG SAFE, FIELD COMMISSIONING BELUM LENGKAP:');[print(' -',x) for x in waits];return 3 if a.require_ready else 0
    if not st.get('production_autonomy_certified',False): print('\nEVIDENCE PASS, tetapi final production sign-off belum disimpan.'); return 3 if a.require_ready else 0
    print('\nSTAGE-3 PRODUCTION READY'); return 0
if __name__=='__main__': raise SystemExit(main())
