#!/usr/bin/env python3
"""Fail-closed physical field campaign for AGV navigation.

Creates required CSV templates and evaluates measured evidence. It never writes
runtime calibration/certification flags. Missing evidence is PENDING.
"""
from __future__ import annotations
import argparse,csv,datetime,hashlib,json,math
from pathlib import Path
import numpy as np
import yaml
import navigation_calibration as cal

DATASETS={
 'imu_stationary.csv':['t','ax','ay','az','gx','gy','gz'],
 'imu_six_position.csv':['pose','ax','ay','az'],
 'imu_motor_off.csv':['t','ax','ay','az','gx','gy','gz'],
 'imu_motor_on.csv':['t','ax','ay','az','gx','gy','gz'],
 'drive_multispeed.csv':['erpm','ground_speed_mps','direction','command_speed_mps','distance_ref_m'],
 'steering_hysteresis.csv':['sweep','command_deg','physical_deg'],
 'steering_circles.csv':['x_m','y_m','direction'],
 'lever_arm.csv':['x_m','y_m','z_m'],
 'map_alignment.csv':['raw_x','raw_y','true_x','true_y'],
 'mag_interference.csv':['heading_ref_rad','heading_mag_rad','current_a','condition'],
 'control_slalom.csv':['t','steer_cmd_rad','steer_actual_rad','speed_mps','yaw_rate_rps'],
 'ground_truth.csv':['t','est_x','est_y','est_yaw','gt_x','gt_y','gt_yaw'],
}
SIX={'x+','x-','y+','y-','z+','z-'}

def sha(path:Path):
 h=hashlib.sha256()
 with path.open('rb') as f:
  for block in iter(lambda:f.read(1<<20),b''): h.update(block)
 return h.hexdigest()
def read_rows(path:Path):
 with path.open(newline='',encoding='utf-8') as f:return list(csv.DictReader(f))
def col(rows,key):return np.asarray([float(r[key]) for r in rows],float)
def p95(values):
 x=np.abs(np.asarray(values,float));x=x[np.isfinite(x)]
 return float(np.percentile(x,95)) if x.size else math.nan
def result(status,summary,metrics=None,files=None):
 return {'status':status,'summary':summary,'metrics':metrics or {},'files':files or []}
def init_campaign(root:Path):
 root.mkdir(parents=True,exist_ok=True)
 for name,columns in DATASETS.items():
  path=root/name
  if not path.exists():
   with path.open('w',newline='') as f:csv.writer(f).writerow(columns)
 meta={'created_at':datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
  'evidence_type':'PHYSICAL_FIELD_MEASUREMENT_REQUIRED','imu_stationary_min_sec':1200,
  'drive_distances_m':[10,25,50],'drive_min_speeds_each_direction':4,
  'ground_truth':'RTK-fixed / total-station / motion-capture / surveyed fixture'}
 (root/'campaign.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
 (root/'README.md').write_text('# AGV Navigation Field Campaign\n\nFill every CSV with physical measured data. Synthetic data cannot certify production.\n')
 print(root)

def six_position(rows):
 groups={}
 for r in rows:
  groups.setdefault(r['pose'].strip().lower(),[]).append([float(r['ax']),float(r['ay']),float(r['az'])])
 missing=sorted(SIX-set(groups))
 if missing:raise ValueError('missing six-position labels: '+','.join(missing))
 means={k:np.mean(np.asarray(v,float),axis=0) for k,v in groups.items()};g=9.80665
 bias=[];scale=[];residual=[]
 for pos,neg,axis in [('x+','x-',0),('y+','y-',1),('z+','z-',2)]:
  vp=float(means[pos][axis]);vn=float(means[neg][axis]);b=.5*(vp+vn);span=.5*(vp-vn)
  if abs(span)<1e-6:raise ValueError(f'six-position zero span axis {axis}')
  s=g/span;bias.append(b);scale.append(s)
  residual += [abs((vp-b)*s-g),abs((vn-b)*s+g)]
 return {'bias_mps2':bias,'scale':scale,'endpoint_residual_p95_mps2':p95(residual),
         'poses':{k:[float(x) for x in means[k]] for k in sorted(means)}}

def steering_lut(rows):
 cmd=col(rows,'command_deg');physical=col(rows,'physical_deg')
 sweeps=np.asarray([r['sweep'].strip().lower() for r in rows]);out={};centers=[]
 for name in sorted(set(sweeps)):
  mask=sweeps==name
  if np.count_nonzero(mask)<5:continue
  order=np.argsort(physical[mask]);out[name]={'physical_deg':physical[mask][order].tolist(),'command_deg':cmd[mask][order].tolist()}
  idx=np.where(mask & (np.abs(physical)<=1.0))[0];centers += cmd[idx].tolist()
 if len(out)<2:raise ValueError('need increasing and decreasing steering sweeps')
 names=list(out)[:2];lo=max(min(out[n]['physical_deg']) for n in names);hi=min(max(out[n]['physical_deg']) for n in names)
 if hi<=lo:raise ValueError('steering sweeps do not overlap')
 grid=np.linspace(lo,hi,61);a=np.interp(grid,out[names[0]]['physical_deg'],out[names[0]]['command_deg']);b=np.interp(grid,out[names[1]]['physical_deg'],out[names[1]]['command_deg'])
 return {'sweeps':out,'hysteresis_command_p95_deg':p95(a-b),'center_command_std_deg':float(np.std(centers,ddof=1)) if len(centers)>1 else math.nan,
         'physical_left_limit_deg':float(np.min(physical)),'physical_right_limit_deg':float(np.max(physical))}

def lever_arm(rows,vehicle):
 xyz=np.column_stack([col(rows,k) for k in ('x_m','y_m','z_m')]);mean=xyz.mean(0)
 std=xyz.std(0,ddof=1) if len(xyz)>1 else np.zeros(3)
 nominal=np.asarray([float(vehicle.get('gnss_antenna_x_m',0)),float(vehicle.get('gnss_antenna_y_m',0)),float(vehicle.get('gnss_antenna_z_m',0))])
 return {'samples':len(rows),'mean_m':mean.tolist(),'std_m':std.tolist(),'nominal_m':nominal.tolist(),
         'delta_to_nominal_m':float(np.linalg.norm(mean-nominal))}

def gt_metrics(rows):
 t=col(rows,'t');ex,ey,eyaw,gx,gy,gyaw=(col(rows,k) for k in ('est_x','est_y','est_yaw','gt_x','gt_y','gt_yaw'))
 pe=np.hypot(ex-gx,ey-gy);ye=np.arctan2(np.sin(eyaw-gyaw),np.cos(eyaw-gyaw));dt=np.diff(t);dt=dt[(dt>0)&np.isfinite(dt)]
 return {'samples':len(rows),'position_rmse_m':float(np.sqrt(np.mean(pe*pe))),
  'position_p95_m':float(np.percentile(pe,95)),'final_position_error_m':float(pe[-1]),
  'yaw_p95_deg':math.degrees(p95(ye)),'rate_hz_median':float(1/np.median(dt)) if dt.size else 0,
  'period_jitter_p95_sec':float(np.percentile(np.abs(dt-np.median(dt)),95)) if dt.size else math.inf}

def analyze(campaign:Path,workspace:Path):
 vehicle=(yaml.safe_load((workspace/'src/navigation/config/vehicle.yaml').read_text()) or {})['vehicle']['ros__parameters']
 results={};evidence=[]
 def get(name):
  path=campaign/name
  if not path.is_file():raise FileNotFoundError(name)
  data=read_rows(path)
  if not data:raise ValueError('no data rows')
  missing=set(DATASETS[name])-set(data[0])
  if missing:raise ValueError('missing columns: '+','.join(sorted(missing)))
  evidence.append(path);return data
 def evaluate(key,name,analyzer,gate):
  try:
   data=get(name);metrics=analyzer(data);ok,why=gate(data,metrics)
   results[key]=result('PASS' if ok else 'FAIL',why,metrics,[name])
  except (FileNotFoundError,ValueError,KeyError,ZeroDivisionError) as exc:
   pending=isinstance(exc,FileNotFoundError) or 'no data rows' in str(exc)
   results[key]=result('PENDING' if pending else 'FAIL',str(exc),files=[name])

 evaluate('imu_stationary','imu_stationary.csv',cal.imu,
  lambda r,m:(m['duration_sec']>=1200 and bool(m['gyro_z_allan']),f"duration={m['duration_sec']:.1f}s; need >=1200s + Allan"))
 evaluate('imu_six_position','imu_six_position.csv',six_position,
  lambda r,m:(m['endpoint_residual_p95_mps2']<=0.15,f"endpoint residual p95={m['endpoint_residual_p95_mps2']:.3f}m/s2"))
 try:
  off=cal.imu(get('imu_motor_off.csv'));on=cal.imu(get('imu_motor_on.csv'))
  ratio=float(on['gyro_std_rps'][2])/max(float(off['gyro_std_rps'][2]),1e-9);ok=ratio<=3.0
  results['imu_motor_interference']=result('PASS' if ok else 'FAIL',f'gyro-z noise ON/OFF={ratio:.2f}',{'off':off,'on':on,'ratio':ratio},['imu_motor_off.csv','imu_motor_on.csv'])
 except (FileNotFoundError,ValueError,KeyError) as exc:
  results['imu_motor_interference']=result('PENDING' if isinstance(exc,FileNotFoundError) or 'no data rows' in str(exc) else 'FAIL',str(exc),files=['imu_motor_off.csv','imu_motor_on.csv'])
 def drive_gate(rows,m):
  directions={r['direction'].strip().lower() for r in rows};distances={round(float(r['distance_ref_m'])) for r in rows}
  speeds={d:{round(abs(float(r['command_speed_mps'])),3) for r in rows if r['direction'].strip().lower()==d} for d in directions}
  design={'forward','reverse'}<=directions and {10,25,50}<=distances and all(len(speeds.get(d,set()))>=4 for d in ('forward','reverse'))
  fit=m['residual_p95_mps']<=0.08 and m['direction_slope_asymmetry_fraction']<=0.05
  return design and fit,f"design={design} residual_p95={m['residual_p95_mps']:.3f}m/s asym={m['direction_slope_asymmetry_fraction']:.3f}"
 evaluate('drive_multispeed','drive_multispeed.csv',lambda r:cal.drive(r,float(vehicle.get('drive_erpm_per_mps',8000))),drive_gate)
 evaluate('steering_hysteresis','steering_hysteresis.csv',steering_lut,
  lambda r,m:(m['hysteresis_command_p95_deg']<=8.0 and (not math.isfinite(m['center_command_std_deg']) or m['center_command_std_deg']<=2.0),f"hysteresis p95={m['hysteresis_command_p95_deg']:.2f}deg center_std={m['center_command_std_deg']:.2f}deg"))
 evaluate('steering_circles','steering_circles.csv',cal.circles,
  lambda r,m:(set(m['directions'])>={'left','right'} and min(v['samples'] for v in m['directions'].values())>=20 and max(v['fit_rmse_m'] for v in m['directions'].values())<=0.10,f"Rmin={m['minimum_measured_radius_m']:.3f}m directions={list(m['directions'])}"))
 evaluate('gnss_lever_arm','lever_arm.csv',lambda r:lever_arm(r,vehicle),
  lambda r,m:(m['samples']>=3 and max(m['std_m'])<=0.01,f"samples={m['samples']} max_std={max(m['std_m']):.4f}m delta_nominal={m['delta_to_nominal_m']:.3f}m"))
 evaluate('map_alignment','map_alignment.csv',cal.map_align,
  lambda r,m:(m['samples']>=3 and m['baseline_m']>=10 and m['geometry_score']>=0.15 and m['rmse_m']<=0.15,f"points={m['samples']} baseline={m['baseline_m']:.1f}m geometry={m['geometry_score']:.3f} rmse={m['rmse_m']:.3f}m"))
 def mag_gate(rows,m):
  conditions={r['condition'].strip().lower() for r in rows};design=len(conditions)>=3
  return design and m['heading_residual_p95_deg']<=5.0,f"conditions={sorted(conditions)} p95={m['heading_residual_p95_deg']:.2f}deg corr={m.get('abs_residual_current_correlation',math.nan):.3f}"
 evaluate('mag_interference','mag_interference.csv',cal.mag,mag_gate)
 evaluate('control_slalom','control_slalom.csv',lambda r:cal.control(r,float(vehicle.get('effective_wheelbase_m',vehicle.get('wheelbase_m',.7)))),
  lambda r,m:(m['estimated_steering_delay_sec']<=0.30 and m['kinematic_yaw_rate_p95_rps']<=0.08,f"delay={m['estimated_steering_delay_sec']:.3f}s yaw residual p95={m['kinematic_yaw_rate_p95_rps']:.3f}rad/s"))
 evaluate('ground_truth','ground_truth.csv',gt_metrics,
  lambda r,m:(m['position_p95_m']<=0.10 and m['yaw_p95_deg']<=2.0,f"position p95={m['position_p95_m']:.3f}m yaw p95={m['yaw_p95_deg']:.2f}deg"))
 def passed(*keys):return all(results.get(k,{}).get('status')=='PASS' for k in keys)
 foundation=('imu_stationary','imu_six_position','imu_motor_interference','drive_multispeed','steering_hysteresis','steering_circles','gnss_lever_arm','map_alignment','mag_interference','control_slalom')
 tier1=passed(*foundation);gt=results['ground_truth'].get('metrics',{})
 tier2=tier1 and results['ground_truth']['status']=='PASS' and gt.get('position_p95_m',999)<=0.15
 tier3=tier2 and gt.get('position_rmse_m',999)<=0.05 and gt.get('position_p95_m',999)<=0.10 and gt.get('final_position_error_m',999)<=0.10 and gt.get('yaw_p95_deg',999)<=2.0
 tiers={'tier1_robust_meter_class':'PASS' if tier1 else 'PENDING','tier2_decimeter_local':'PASS' if tier2 else 'PENDING','tier3_precision_global':'PASS' if tier3 else 'PENDING'}
 report={'generated_at':datetime.datetime.now().astimezone().isoformat(timespec='seconds'),'campaign':str(campaign.resolve()),'evidence_type':'PHYSICAL_FIELD_MEASUREMENT_REQUIRED','items':results,'tiers':tiers,'production_certification_changed':False,'evidence_sha256':{str(p.relative_to(campaign)):sha(p) for p in evidence if p.is_file()}}
 out=campaign/'field_acceptance.json';out.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
 lines=['# AGV Navigation Field Acceptance','',f"Generated: `{report['generated_at']}`",'',
  '> This report never changes runtime certification flags. Missing physical evidence remains PENDING.','', '## Evidence items','']
 for key,value in results.items():lines.append(f"- **{key} — {value['status']}**: {value['summary']}")
 lines += ['', '## Acceptance tiers','']+[f"- **{k}**: `{v}`" for k,v in tiers.items()]
 (campaign/'field_acceptance.md').write_text('\n'.join(lines)+'\n')
 counts={s:sum(v['status']==s for v in results.values()) for s in ('PASS','FAIL','PENDING')}
 print(json.dumps({'out':str(out),'tiers':tiers,'status_counts':counts},indent=2))
 return 0 if counts['FAIL']==0 and counts['PENDING']==0 else 2

def main():
 ap=argparse.ArgumentParser();sub=ap.add_subparsers(dest='cmd',required=True)
 init=sub.add_parser('init');init.add_argument('--campaign',type=Path,required=True)
 ana=sub.add_parser('analyze');ana.add_argument('--campaign',type=Path,required=True);ana.add_argument('--workspace',type=Path,required=True)
 args=ap.parse_args()
 if args.cmd=='init':init_campaign(args.campaign);return 0
 return analyze(args.campaign,args.workspace.resolve())

if __name__=='__main__':raise SystemExit(main())
