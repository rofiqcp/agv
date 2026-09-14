#!/usr/bin/env python3
from __future__ import annotations
import argparse, datetime, hashlib, json, os, tempfile
from pathlib import Path
import yaml

YAH_FIELDS={
 'imu_mag_yaw_sign':'yaw_sign','imu_mag_yaw_offset_rad':'yaw_offset_rad',
 'imu_mag_bias_xy_lsb':'bias','imu_mag_matrix_xy_per_lsb':'matrix',
 'imu_heading_lut_input_rad':'heading_lut_input_rad','imu_heading_lut_correction_rad':'heading_lut_correction_rad'}
NEO_RESIDUAL_FIELDS={
 'neo3_mag_yaw_sign':'yaw_sign','neo3_mag_yaw_offset_rad':'yaw_offset_rad',
 'neo3_heading_lut_input_rad':'heading_lut_input_rad','neo3_heading_lut_correction_rad':'heading_lut_correction_rad'}

def sha256_file(path:Path)->str:
 h=hashlib.sha256()
 with path.open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''):h.update(b)
 return h.hexdigest()

def atomic_yaml(path:Path,data)->None:
 path.parent.mkdir(parents=True,exist_ok=True)
 fd,tmp=tempfile.mkstemp(prefix=path.name+'.tmp.',dir=path.parent);os.close(fd)
 try:
  Path(tmp).write_text(yaml.safe_dump(data,sort_keys=False,width=120))
  yaml.safe_load(Path(tmp).read_text());os.replace(tmp,path)
 finally:
  if Path(tmp).exists():Path(tmp).unlink()

def state_ok(state:Path)->None:
 if not state.is_file():return
 st=json.loads(state.read_text())
 if st.get('mounting_ok') is False or st.get('mounting_state') in ('UPSIDE_DOWN','SIDE_MOUNTED','TILT_MISMATCH'):
  raise ValueError('mounting mismatch; Stage-2 apply ditolak')

def validate_stage1(data:dict,state:Path)->dict:
 h=data.get('heading_360_calibration') or {}
 residual=h.get('neo3_ap_periph_residual') or data.get('neo3_ap_periph_residual_candidate') or {}
 ready=(h.get('stage1_only') is True and h.get('ready_for_stage2') is True and h.get('valid') is True and
        (h.get('yahboom') or {}).get('valid') is True and residual.get('valid') is True and
        (h.get('gyro') or {}).get('pass') is True and (h.get('cross_sensor') or {}).get('pass') is True)
 if not ready:raise ValueError('Stage-1 9-stop/360 belum PASS seluruh gate')
 state_ok(state)
 return h

def verify_owner_contract(ws:Path,h:dict)->dict:
 f4=ws/'F4gateway/src/Neo3ProSensors.cpp'; fusion=ws/'src/navigation/src/mag_heading_fusion_node.cpp'
 if not f4.is_file() or not fusion.is_file():raise ValueError('source contract ownership NEO3 tidak ditemukan')
 f4s=f4.read_text(errors='ignore');fs=fusion.read_text(errors='ignore')
 forward=('SENS:MAGPRO:' in f4s and 'mag_.message_type' in f4s and 'mag_.x_ga * 100.0F' in f4s)
 guard=('neo_calibration_owner_ == "ap_periph"' in fs and 'host correction disabled to prevent double calibration' in fs)
 residual=h.get('neo3_ap_periph_residual') or {}
 norm_ok=residual.get('field_norm_gate_pass') is True
 if not (forward and guard and norm_ok):raise ValueError('NEO3 owner contract belum terverifikasi fail-closed')
 return {'verified':True,'owner':'ap_periph','f411_dronecan_forward_contract':forward,
         'ros_double_calibration_guard':guard,'field_norm_gate_pass':norm_ok,
         'source_semantics':'AP_Periph publishes corrected compass.get_field over DroneCAN; ROS hard/soft-iron must stay disabled'}

def stage2_values(h:dict)->dict:
 yah=h['yahboom'];res=h['neo3_ap_periph_residual']
 v={dst:yah[src] for dst,src in YAH_FIELDS.items()};v.update({dst:res[src] for dst,src in NEO_RESIDUAL_FIELDS.items()})
 v.update({
  'imu_planar_calibration_enabled':True,'imu_heading_lut_enabled':True,'enable_imu_mag_heading':True,
  'neo3_calibration_owner':'ap_periph','neo3_calibration_ownership_verified':True,
  'neo3_planar_calibration_enabled':False,'neo3_full_calibration_enabled':False,
  'neo3_mag_bias_xy_ut':[0.0,0.0],'neo3_mag_matrix_xy':[1.0,0.0,0.0,1.0],
  'neo3_mag_bias_xyz_ut':[0.0,0.0,0.0],
  'neo3_mag_matrix_3x3':[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],
  'neo3_heading_lut_enabled':True,'enable_neo3_mag_heading':True,
  'field_qualification_valid':False,'field_qualification_saved_at':'','field_qualification_evidence_sha256':'',
 })
 return v

def proposal_items(values:dict)->list[dict]:
 return [{'file_key':'mag_heading','path':f'mag_heading_fusion.ros__parameters.{k}','value':v,'generated':True,'stage2':True}
         for k,v in values.items()]

def evidence(h:dict,cal:Path,owner:dict)->dict:
 return {'calibration_yaml':str(cal),'calibration_sha256':sha256_file(cal),'stage2':True,
         'direction':h.get('direction'),'segment_count':h.get('segment_count'),'gyro':h.get('gyro'),
         'cross_sensor':h.get('cross_sensor'),'yahboom_validation':(h.get('yahboom') or {}).get('validation'),
         'neo3_residual_validation':(h.get('neo3_ap_periph_residual') or {}).get('validation'),'ownership':owner}

def apply_stage2(ws:Path,data:dict,cal:Path,state:Path)->dict:
 h=validate_stage1(data,state);owner=verify_owner_contract(ws,h);values=stage2_values(h)
 mag=ws/'src/navigation/config/mag_heading.yaml';model=yaml.safe_load(mag.read_text()) or {}
 ros=(model.setdefault('mag_heading_fusion',{})).setdefault('ros__parameters',{})
 for k,v in values.items():ros[k]=v
 atomic_yaml(mag,model)
 ev={'heading_360_calibration':h,'stage2_apply':{'applied_at':datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
     'calibration_sha256':sha256_file(cal),'ownership':owner,'field_qualification_valid':False}}
 evpath=ws/'src/navigation/config/heading_360_calibration.yaml';atomic_yaml(evpath,ev)
 return {'ok':True,'message':'Stage-2 calibration ditulis atomik; field qualification tetap fail-closed',
         'mag_heading_yaml':str(mag),'evidence_yaml':str(evpath),'values':values,'proposal_items':proposal_items(values),
         'evidence':evidence(h,cal,owner),'field_qualification_valid':False,'localization_heading_enabled':False}

def stage1_preview(h:dict,cal:Path)->dict:
 yah=h['yahboom'];neo=h['neo3'];values={dst:yah[src] for dst,src in YAH_FIELDS.items()}
 # Diagnostic full fit only: Stage-1 preview, never authority/apply.
 values.update({'neo3_mag_yaw_sign':neo['yaw_sign'],'neo3_mag_yaw_offset_rad':neo['yaw_offset_rad'],
  'neo3_mag_bias_xy_ut':neo['bias'],'neo3_mag_matrix_xy':neo['matrix'],
  'neo3_heading_lut_input_rad':neo['heading_lut_input_rad'],'neo3_heading_lut_correction_rad':neo['heading_lut_correction_rad'],
  'imu_planar_calibration_enabled':True,'imu_heading_lut_enabled':True,'enable_imu_mag_heading':True,
  'neo3_planar_calibration_enabled':True,'neo3_heading_lut_enabled':True,'enable_neo3_mag_heading':True})
 items=[{'file_key':'mag_heading','path':f'mag_heading_fusion.ros__parameters.{k}','value':v,'generated':True,'stage1_only':True} for k,v in values.items()]
 return {'ok':True,'message':'Stage-1 heading 360 PASS; preview only. Tahap 2 diperlukan untuk ownership + apply.',
  'stage1_only':True,'apply_locked':True,'proposal_only':True,'runtime_write':False,'yaml_write':False,
  'proposal_items':items,'evidence':{'calibration_yaml':str(cal),'direction':h.get('direction'),'segment_count':h.get('segment_count')}}

def legacy_preview(data:dict)->dict:
 fit=data.get('yahboom_mag_planar_calibration') or {};val=fit.get('validation') or {}
 passed=fit.get('valid') is True and val.get('pass') is True and fit.get('transition_check_pass') is True
 if not passed:raise ValueError('fit belum PASS fail-closed')
 values={'imu_mag_yaw_sign':fit['yaw_sign'],'imu_mag_yaw_offset_rad':fit['yaw_offset_rad'],
  'imu_mag_bias_xy_lsb':fit['bias_xy_lsb'],'imu_mag_matrix_xy_per_lsb':fit['matrix_xy_per_lsb'],
  'imu_heading_lut_input_rad':fit['heading_lut_input_rad'],'imu_heading_lut_correction_rad':fit['heading_lut_correction_rad'],
  'imu_planar_calibration_enabled':True,'imu_heading_lut_enabled':True,'enable_imu_mag_heading':True}
 return {'ok':True,'proposal_only':True,'legacy':True,'proposal_items':proposal_items(values),
         'message':'Legacy Yahboom proposal generated; no YAML/runtime write'}

def main()->int:
 ap=argparse.ArgumentParser();ap.add_argument('--workspace',default=os.environ.get('AGV_ROOT',str(Path.home()/'agv')))
 ap.add_argument('--calibration',default='');ap.add_argument('--state',default='');ap.add_argument('--propose',action='store_true');ap.add_argument('--stage2-propose',action='store_true');ap.add_argument('--apply-stage2',action='store_true')
 a=ap.parse_args();ws=Path(a.workspace).expanduser().resolve();cal=Path(a.calibration) if a.calibration else ws/'calibration/heading_360_latest.yaml'
 if not cal.is_file():
  legacy=ws/'calibration/yahboom_mag_planar_latest.yaml';cal=legacy if legacy.is_file() else cal
 state=Path(a.state) if a.state else ws/'calibration/yahboom_calibration_state.json'
 try:
  data=yaml.safe_load(cal.read_text()) or {}
  if data.get('heading_360_calibration'):
   h=validate_stage1(data,state)
   if not (a.apply_stage2 or a.stage2_propose):
    if a.propose:
     print(json.dumps(stage1_preview(h,cal)));return 0
    raise ValueError('Stage-1 direct apply dikunci; gunakan --stage2-propose lalu --apply-stage2')
   owner=verify_owner_contract(ws,h);values=stage2_values(h);ev=evidence(h,cal,owner)
   if a.apply_stage2:
    print(json.dumps(apply_stage2(ws,data,cal,state)));return 0
   print(json.dumps({'ok':True,'message':'Stage-2 calibration proposal PASS; field qualification tetap false sampai EMI campaign PASS',
    'stage2':True,'apply_locked':False,'proposal_only':True,'runtime_write':False,'yaml_write':False,
    'proposal_items':proposal_items(values),'evidence':ev,'field_qualification_required':True}));return 0
  if a.apply_stage2:raise ValueError('Stage-2 apply membutuhkan heading_360_calibration, legacy fit ditolak')
  print(json.dumps(legacy_preview(data)));return 0
 except (ValueError,OSError,json.JSONDecodeError,KeyError,TypeError) as exc:
  print(json.dumps({'ok':False,'message':str(exc),'stage2':bool(a.apply_stage2 or a.propose)}));return 2
if __name__=='__main__':raise SystemExit(main())
