#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,os,shutil,tempfile,time
from pathlib import Path
import yaml
FIELDS={'imu_mag_yaw_sign':'yaw_sign','imu_mag_yaw_offset_rad':'yaw_offset_rad','imu_mag_bias_xy_lsb':'bias_xy_lsb','imu_mag_matrix_xy_per_lsb':'matrix_xy_per_lsb','imu_heading_lut_input_rad':'heading_lut_input_rad','imu_heading_lut_correction_rad':'heading_lut_correction_rad'}
def atomic_yaml(path,data):
    fd,tmp=tempfile.mkstemp(prefix=path.name+'.tmp.',dir=path.parent);os.close(fd)
    try:
        Path(tmp).write_text(yaml.safe_dump(data,sort_keys=False,width=120));yaml.safe_load(Path(tmp).read_text());os.replace(tmp,path)
    finally:
        if Path(tmp).exists():Path(tmp).unlink()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--workspace',default='/home/otomasi/ros');ap.add_argument('--calibration',default='');ap.add_argument('--state',default='');a=ap.parse_args();ws=Path(a.workspace).resolve();cal=Path(a.calibration) if a.calibration else ws/'calibration/yahboom_mag_planar_latest.yaml';state=Path(a.state) if a.state else ws/'calibration/yahboom_calibration_state.json'
    if not cal.is_file():print(json.dumps({'ok':False,'message':'calibration YAML tidak ditemukan'}));return 2
    d=yaml.safe_load(cal.read_text()) or {};q=d.get('yahboom_mag_planar_calibration',{});v=q.get('validation',{})
    if not(q.get('valid') is True and v.get('pass') is True and q.get('transition_check_pass') is True and float(v.get('rms_error_deg',999))<3 and float(v.get('max_abs_error_deg',999))<5):print(json.dumps({'ok':False,'message':'fit belum PASS fail-closed'}));return 2
    if state.is_file():
        st=json.loads(state.read_text());
        if st.get('mounting_ok') is False or st.get('mounting_state') in ('UPSIDE_DOWN','SIDE_MOUNTED','TILT_MISMATCH'):print(json.dumps({'ok':False,'message':'mounting mismatch; apply ditolak'}));return 2
    mag=ws/'src/navigation/config/mag_heading.yaml';evidence=ws/'src/navigation/config/yahboom_mag_calibration.yaml';backupdir=ws/'calibration/backups';backupdir.mkdir(parents=True,exist_ok=True);stamp=time.strftime('%Y%m%d_%H%M%S')
    for p in (mag,evidence):
        if p.exists():shutil.copy2(p,backupdir/f'{p.name}.{stamp}.bak')
    m=yaml.safe_load(mag.read_text());r=m['mag_heading_fusion']['ros__parameters']
    for dst,src in FIELDS.items():r[dst]=q[src]
    r['imu_planar_calibration_enabled']=True;r['imu_heading_lut_enabled']=True;r['enable_imu_mag_heading']=True
    atomic_yaml(mag,m);atomic_yaml(evidence,d)
    out={'ok':True,'message':'Yahboom calibration applied atomically; mag_heading_fusion restart required','mag_heading_yaml':str(mag),'evidence_yaml':str(evidence),'direction':q.get('direction'),'rms_error_deg':v.get('rms_error_deg'),'max_abs_error_deg':v.get('max_abs_error_deg'),'backup_dir':str(backupdir)};print(json.dumps(out));return 0
if __name__=='__main__':raise SystemExit(main())
