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
 'imu_traction_on.csv':['t','ax','ay','az','gx','gy','gz'],
 'imu_steering_on.csv':['t','ax','ay','az','gx','gy','gz'],
 'imu_cold_start.csv':['t','ax','ay','az','gx','gy','gz'],
 'imu_warm_start.csv':['t','ax','ay','az','gx','gy','gz'],
 'drive_multispeed.csv':['trial_id','erpm','ground_speed_mps','direction','command_speed_mps','distance_ref_m','distance_odom_m'],
 'steering_hysteresis.csv':['sweep','command_deg','feedback_deg','physical_deg','settling_time_sec'],
 'steering_circles.csv':['x_m','y_m','direction','physical_steering_deg','reference_radius_m'],
 'lever_arm.csv':['x_m','y_m','z_m'],
 'map_alignment.csv':['raw_x','raw_y','true_x','true_y'],
 'mag_interference.csv':['direction_deg','heading_ref_rad','heading_mag_rad','current_a','condition'],
 'gnss_motion_validation.csv':['validation_velocity_qualified','validation_cog_qualified','validation_velocity_covariance_valid','validation_quality_fresh','validation_sync_gap_sec','validation_wheel_minus_gnss_mps','base_vy','validation_cog_minus_vel_course_rad'],
 'localization_innovations.csv':['wheel_nis','cog_nis','wheel_gate_pass','cog_gate_pass'],
 'localization_reacquisition.csv':['trial_id','t','event','map_odom_jump_m','map_odom_yaw_jump_rad','autonomy_gate_open'],
 'nav2_ab.csv':['profile','t','cpu_pct','controller_period_sec','cross_track_error_m','steering_command_rad'],
 'control_slalom.csv':['t','steer_cmd_rad','steer_actual_rad','speed_mps','yaw_rate_rps','yaw_target_rps','yaw_correction_deg','direction'],
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
  'evidence_type':'PHYSICAL_FIELD_MEASUREMENT_REQUIRED','no_eskf':True,'no_rtk_calibration':True,'imu_stationary_min_sec':1200,
  'drive_distances_m':[10,25,50],'drive_min_speeds_each_direction':4,
  'ground_truth':'surveyed tape/fixture / total-station / motion-capture (no RTK calibration required)'}
 (root/'campaign.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
 (root/'README.md').write_text(
  '# AGV Navigation Field Campaign\n\n'
  'Fill every CSV with physical measured data. Synthetic data cannot certify production.\n\n'
  '## IMU evidence source\n'
  'For imu_stationary/six_position/motor/cold/warm datasets, record body-frame SI values from `/imu/calibration_vectors` '
  '(layout ax,ay,az,gx,gy,gz) before runtime bias/scale correction. Do not fit field calibration from already-corrected `/imu/data`.\n')
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
 cmd=col(rows,'command_deg');fb=col(rows,'feedback_deg');physical=col(rows,'physical_deg');settle=col(rows,'settling_time_sec')
 sweeps=np.asarray([r['sweep'].strip().lower() for r in rows]);out={};centers=[]
 for name in sorted(set(sweeps)):
  mask=sweeps==name
  if np.count_nonzero(mask)<5:continue
  order=np.argsort(physical[mask]);out[name]={'physical_deg':physical[mask][order].tolist(),'command_deg':cmd[mask][order].tolist(),'feedback_deg':fb[mask][order].tolist()}
  idx=np.where(mask & (np.abs(physical)<=1.0))[0];centers += cmd[idx].tolist()
 if len(out)<2:raise ValueError('need increasing and decreasing steering sweeps')
 names=list(out)[:2];lo=max(min(out[n]['physical_deg']) for n in names);hi=min(max(out[n]['physical_deg']) for n in names)
 if hi<=lo:raise ValueError('steering sweeps do not overlap')
 grid=np.linspace(lo,hi,61)
 cmd_a=np.interp(grid,out[names[0]]['physical_deg'],out[names[0]]['command_deg']);cmd_b=np.interp(grid,out[names[1]]['physical_deg'],out[names[1]]['command_deg'])
 fb_a=np.interp(grid,out[names[0]]['physical_deg'],out[names[0]]['feedback_deg']);fb_b=np.interp(grid,out[names[1]]['physical_deg'],out[names[1]]['feedback_deg'])
 return {'sweeps':out,'hysteresis_command_p95_deg':p95(cmd_a-cmd_b),'hysteresis_feedback_p95_deg':p95(fb_a-fb_b),
         'center_command_std_deg':float(np.std(centers,ddof=1)) if len(centers)>1 else math.nan,'settling_time_p95_sec':p95(settle),
         'physical_left_limit_deg':float(np.min(physical)),'physical_right_limit_deg':float(np.max(physical))}

def lever_arm(rows,loc):
 xyz=np.column_stack([col(rows,k) for k in ('x_m','y_m','z_m')]);mean=xyz.mean(0)
 std=xyz.std(0,ddof=1) if len(xyz)>1 else np.zeros(3)
 nominal=np.asarray([float(loc.get('gnss_antenna_x_m',0)),float(loc.get('gnss_antenna_y_m',0)),float(loc.get('gnss_antenna_z_m',0))])
 return {'samples':len(rows),'mean_m':mean.tolist(),'std_m':std.tolist(),'nominal_m':nominal.tolist(),
         'delta_to_nominal_m':float(np.linalg.norm(mean-nominal))}

def imu_repeatability(cold,warm):
 c=cal.imu(cold);w=cal.imu(warm)
 gd=np.abs(np.asarray(c['gyro_mean_rps'])-np.asarray(w['gyro_mean_rps']))
 ad=np.abs(np.asarray(c['accel_mean_mps2'])-np.asarray(w['accel_mean_mps2']))
 return {'cold':c,'warm':w,'gyro_bias_delta_max_rps':float(np.max(gd)),
  'accel_mean_delta_norm_mps2':float(np.linalg.norm(ad))}

def circle_geometry(rows,vehicle):
 m=cal.circles(rows);track=float(vehicle.get('track_width_m',0.48));details={};errs=[];wheelbases=[]
 dirs=np.asarray([r['direction'].strip().lower() for r in rows])
 for d,fit in m['directions'].items():
  rr=[float(r['reference_radius_m']) for r in rows if r['direction'].strip().lower()==d and float(r['reference_radius_m'])>0]
  sd=[abs(float(r['physical_steering_deg'])) for r in rows if r['direction'].strip().lower()==d and abs(float(r['physical_steering_deg']))>0.1]
  ref=float(np.median(rr)) if rr else math.nan;delta=float(np.median(sd)) if sd else math.nan
  err=abs(fit['radius_m']-ref)/ref if math.isfinite(ref) and ref>0 else math.nan
  wb=(fit['radius_m']-0.5*track)*math.tan(math.radians(delta)) if math.isfinite(delta) else math.nan
  if math.isfinite(err):errs.append(err)
  if math.isfinite(wb) and wb>0:wheelbases.append(wb)
  details[d]={'reference_radius_m':ref,'radius_error_fraction':err,'physical_steering_deg':delta,'effective_wheelbase_m':wb}
 m['field_details']=details;m['radius_error_p95_fraction']=float(np.percentile(errs,95)) if errs else math.nan
 m['effective_wheelbase_m']=float(np.mean(wheelbases)) if wheelbases else math.nan
 return m

def truth(v):
 return str(v).strip().lower() in {'1','true','yes','pass'}

def gnss_motion_metrics(rows,loc):
 n=len(rows);q=lambda k:sum(truth(r[k]) for r in rows)/max(n,1)
 sync=p95(col(rows,'validation_sync_gap_sec'));wheel=p95(col(rows,'validation_wheel_minus_gnss_mps'))
 lateral=p95(col(rows,'base_vy'));cog=p95(col(rows,'validation_cog_minus_vel_course_rad'))
 return {'samples':n,'velocity_qualified_ratio':q('validation_velocity_qualified'),'cog_qualified_ratio':q('validation_cog_qualified'),
  'velocity_covariance_valid_ratio':q('validation_velocity_covariance_valid'),'quality_fresh_ratio':q('validation_quality_fresh'),
  'sync_gap_p95_sec':sync,'wheel_gnss_residual_p95_mps':wheel,'base_lateral_velocity_p95_mps':lateral,'cog_vs_doppler_p95_rad':cog,
  'thresholds':{k:loc[k] for k in ('stage2_min_velocity_epochs','stage2_min_cog_epochs','stage2_min_velocity_qualified_ratio','stage2_min_cog_qualified_ratio','stage2_max_sync_gap_p95_sec','stage2_max_wheel_gnss_residual_p95_mps','stage2_max_lateral_velocity_p95_mps','stage2_max_cog_doppler_residual_p95_rad')}}

def innovation_metrics(rows):
 w=col(rows,'wheel_nis');c=col(rows,'cog_nis');n=len(rows)
 wp=np.asarray([truth(r['wheel_gate_pass']) for r in rows]);cp=np.asarray([truth(r['cog_gate_pass']) for r in rows])
 return {'samples':n,'wheel_nis_p95':float(np.percentile(w[np.isfinite(w)],95)),'cog_nis_p95':float(np.percentile(c[np.isfinite(c)],95)),
  'wheel_gate_pass_ratio':float(np.mean(wp)),'cog_gate_pass_ratio':float(np.mean(cp))}

def reacquisition_metrics(rows):
 reacq=[r for r in rows if r['event'].strip().lower() in {'reacquired','reacquire'}]
 jumps=np.asarray([abs(float(r['map_odom_jump_m'])) for r in reacq],float);yaw=np.asarray([abs(float(r['map_odom_yaw_jump_rad'])) for r in reacq],float)
 return {'reacquisition_events':len(reacq),'position_jump_p95_m':float(np.percentile(jumps,95)) if jumps.size else math.inf,
  'yaw_jump_p95_deg':math.degrees(float(np.percentile(yaw,95))) if yaw.size else math.inf,
  'gate_open_at_reacquire_ratio':float(np.mean([truth(r['autonomy_gate_open']) for r in reacq])) if reacq else 1.0}

def nav2_ab_metrics(rows):
 out={}
 for profile in sorted({r['profile'].strip().lower() for r in rows}):
  rr=[r for r in rows if r['profile'].strip().lower()==profile]
  t=np.asarray([float(r['t']) for r in rr]);period=np.asarray([float(r['controller_period_sec']) for r in rr]);cpu=np.asarray([float(r['cpu_pct']) for r in rr]);cte=np.asarray([abs(float(r['cross_track_error_m'])) for r in rr]);steer=np.asarray([float(r['steering_command_rad']) for r in rr])
  ds=np.abs(np.diff(steer)) if len(steer)>1 else np.asarray([],float)
  out[profile]={'samples':len(rr),'rate_hz_median':float(1/np.median(period)) if len(period) else 0.0,'period_jitter_p95_sec':float(np.percentile(np.abs(period-np.median(period)),95)) if len(period) else math.inf,'cpu_p95_pct':float(np.percentile(cpu,95)) if len(cpu) else math.inf,'cross_track_p95_m':float(np.percentile(cte,95)) if len(cte) else math.inf,'steering_delta_p95_rad':float(np.percentile(ds,95)) if ds.size else 0.0,'duration_sec':float(t[-1]-t[0]) if len(t)>1 else 0.0}
 return {'profiles':out}

def gt_metrics(rows):
 t=col(rows,'t');ex,ey,eyaw,gx,gy,gyaw=(col(rows,k) for k in ('est_x','est_y','est_yaw','gt_x','gt_y','gt_yaw'))
 pe=np.hypot(ex-gx,ey-gy);ye=np.arctan2(np.sin(eyaw-gyaw),np.cos(eyaw-gyaw));dt=np.diff(t);dt=dt[(dt>0)&np.isfinite(dt)]
 return {'samples':len(rows),'position_rmse_m':float(np.sqrt(np.mean(pe*pe))),
  'position_p95_m':float(np.percentile(pe,95)),'final_position_error_m':float(pe[-1]),
  'yaw_p95_deg':math.degrees(p95(ye)),'rate_hz_median':float(1/np.median(dt)) if dt.size else 0,
  'period_jitter_p95_sec':float(np.percentile(np.abs(dt-np.median(dt)),95)) if dt.size else math.inf}

def analyze(campaign:Path,workspace:Path):
 vehicle=(yaml.safe_load((workspace/'src/navigation/config/vehicle.yaml').read_text()) or {})['vehicle']['ros__parameters']
 loc=(yaml.safe_load((workspace/'src/navigation/config/localization_cpp.yaml').read_text()) or {})['localization_core']['ros__parameters']
 esc=(yaml.safe_load((workspace/'src/esc/config/ackermann.yaml').read_text()) or {})['esc_ackermann']['ros__parameters']
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
 def six_gate(rows,m):
  scale=np.asarray(m['scale'],float);bias=np.asarray(m['bias_mps2'],float)
  sane=bool(np.all(np.isfinite(scale))) and bool(np.all((scale>=0.80)&(scale<=1.20))) and bool(np.all(np.abs(bias)<=1.0))
  ok=sane and m['endpoint_residual_p95_mps2']<=0.15
  return ok,f"sane={sane} scale={scale.tolist()} bias={bias.tolist()} endpoint_residual_p95={m['endpoint_residual_p95_mps2']:.3f}m/s2"
 evaluate('imu_six_position','imu_six_position.csv',six_position,six_gate)
 try:
  off=cal.imu(get('imu_motor_off.csv'));traction=cal.imu(get('imu_traction_on.csv'));steering=cal.imu(get('imu_steering_on.csv'))
  base=max(float(off['gyro_std_rps'][2]),1e-9)
  traction_ratio=float(traction['gyro_std_rps'][2])/base;steering_ratio=float(steering['gyro_std_rps'][2])/base
  worst=max(traction_ratio,steering_ratio);ok=worst<=3.0
  results['imu_motor_interference']=result('PASS' if ok else 'FAIL',f'gyro-z noise ratio traction={traction_ratio:.2f} steering={steering_ratio:.2f}',
   {'off':off,'traction':traction,'steering':steering,'traction_ratio':traction_ratio,'steering_ratio':steering_ratio,'worst_ratio':worst},
   ['imu_motor_off.csv','imu_traction_on.csv','imu_steering_on.csv'])
 except (FileNotFoundError,ValueError,KeyError) as exc:
  results['imu_motor_interference']=result('PENDING' if isinstance(exc,FileNotFoundError) or 'no data rows' in str(exc) else 'FAIL',str(exc),files=['imu_motor_off.csv','imu_traction_on.csv','imu_steering_on.csv'])
 try:
  repeat=imu_repeatability(get('imu_cold_start.csv'),get('imu_warm_start.csv'))
  ok=repeat['gyro_bias_delta_max_rps']<=0.02 and repeat['accel_mean_delta_norm_mps2']<=0.20
  results['imu_repeatability']=result('PASS' if ok else 'FAIL',f"gyro bias delta={repeat['gyro_bias_delta_max_rps']:.4f}rad/s accel delta={repeat['accel_mean_delta_norm_mps2']:.3f}m/s2",repeat,['imu_cold_start.csv','imu_warm_start.csv'])
 except (FileNotFoundError,ValueError,KeyError) as exc:
  results['imu_repeatability']=result('PENDING' if isinstance(exc,FileNotFoundError) or 'no data rows' in str(exc) else 'FAIL',str(exc),files=['imu_cold_start.csv','imu_warm_start.csv'])
 def drive_gate(rows,m):
  directions={r['direction'].strip().lower() for r in rows};distances={round(float(r['distance_ref_m'])) for r in rows}
  speeds={d:{round(abs(float(r['command_speed_mps'])),3) for r in rows if r['direction'].strip().lower()==d} for d in directions}
  design={'forward','reverse'}<=directions and {10,25,50}<=distances and all(len(speeds.get(d,set()))>=4 for d in ('forward','reverse'))
  derr=[abs(float(r['distance_odom_m'])-float(r['distance_ref_m']))/max(abs(float(r['distance_ref_m'])),1e-9) for r in rows]
  distance_p95=float(np.percentile(derr,95)) if derr else math.inf;m['distance_error_p95_fraction']=distance_p95
  fit=m['residual_p95_mps']<=0.08 and m['direction_slope_asymmetry_fraction']<=0.05 and distance_p95<=0.02
  return design and fit,f"design={design} speed_residual_p95={m['residual_p95_mps']:.3f}m/s asym={m['direction_slope_asymmetry_fraction']:.3f} distance_p95={100*distance_p95:.2f}%"
 evaluate('drive_multispeed','drive_multispeed.csv',lambda r:cal.drive(r,float(vehicle.get('drive_erpm_per_mps',8000))),drive_gate)
 evaluate('steering_hysteresis','steering_hysteresis.csv',steering_lut,
  lambda r,m:(m['hysteresis_command_p95_deg']<=8.0 and m['hysteresis_feedback_p95_deg']<=8.0 and m['settling_time_p95_sec']<=0.50 and (not math.isfinite(m['center_command_std_deg']) or m['center_command_std_deg']<=2.0),f"cmd_hyst={m['hysteresis_command_p95_deg']:.2f}deg fb_hyst={m['hysteresis_feedback_p95_deg']:.2f}deg settle_p95={m['settling_time_p95_sec']:.3f}s center_std={m['center_command_std_deg']:.2f}deg"))
 evaluate('steering_circles','steering_circles.csv',lambda r:circle_geometry(r,vehicle),
  lambda r,m:(set(m['directions'])>={'left','right'} and min(v['samples'] for v in m['directions'].values())>=20 and max(v['fit_rmse_m'] for v in m['directions'].values())<=0.10 and math.isfinite(m['radius_error_p95_fraction']) and m['radius_error_p95_fraction']<=0.05,f"Rmin={m['minimum_measured_radius_m']:.3f}m radius_error_p95={100*m['radius_error_p95_fraction']:.2f}% effective_wheelbase={m['effective_wheelbase_m']:.3f}m"))
 evaluate('gnss_lever_arm','lever_arm.csv',lambda r:lever_arm(r,loc),
  lambda r,m:(m['samples']>=3 and max(m['std_m'])<=0.01,f"samples={m['samples']} max_std={max(m['std_m']):.4f}m delta_nominal={m['delta_to_nominal_m']:.3f}m"))
 evaluate('map_alignment','map_alignment.csv',cal.map_align,
  lambda r,m:(m['samples']>=3 and m['baseline_m']>=10 and m['geometry_score']>=0.15 and m['rmse_m']<=0.15,f"points={m['samples']} baseline={m['baseline_m']:.1f}m geometry={m['geometry_score']:.3f} rmse={m['rmse_m']:.3f}m"))
 def mag_gate(rows,m):
  conditions={r['condition'].strip().lower() for r in rows};required={'motor_off','steering_on','traction_on'}
  dirs={c:{round(float(r['direction_deg'])/45.0)%8 for r in rows if r['condition'].strip().lower()==c} for c in required}
  design=required<=conditions and all(len(dirs[c])>=8 for c in required)
  corr=abs(float(m.get('abs_residual_current_correlation',0.0))) if math.isfinite(float(m.get('abs_residual_current_correlation',0.0))) else 1.0
  m['emi_current_correlation_abs']=corr;m['direction_bins_by_condition']={k:len(v) for k,v in dirs.items()}
  return design and m['heading_residual_p95_deg']<=5.0,f"8dir_design={design} bins={m['direction_bins_by_condition']} p95={m['heading_residual_p95_deg']:.2f}deg |corr|={corr:.3f}"
 evaluate('mag_interference','mag_interference.csv',cal.mag,mag_gate)

 def gnss_gate(rows,m):
  th=m['thresholds'];ok=(m['samples']>=int(th['stage2_min_velocity_epochs']) and m['velocity_qualified_ratio']>=float(th['stage2_min_velocity_qualified_ratio']) and m['velocity_covariance_valid_ratio']>=float(th['stage2_min_velocity_qualified_ratio']) and m['quality_fresh_ratio']>=float(th['stage2_min_velocity_qualified_ratio']) and m['cog_qualified_ratio']>=float(th['stage2_min_cog_qualified_ratio']) and m['sync_gap_p95_sec']<=float(th['stage2_max_sync_gap_p95_sec']) and m['wheel_gnss_residual_p95_mps']<=float(th['stage2_max_wheel_gnss_residual_p95_mps']) and m['base_lateral_velocity_p95_mps']<=float(th['stage2_max_lateral_velocity_p95_mps']) and m['cog_vs_doppler_p95_rad']<=float(th['stage2_max_cog_doppler_residual_p95_rad']))
  return ok,f"n={m['samples']} vel_q={m['velocity_qualified_ratio']:.3f} cog_q={m['cog_qualified_ratio']:.3f} sync95={m['sync_gap_p95_sec']:.3f}s wheel95={m['wheel_gnss_residual_p95_mps']:.3f}m/s"
 evaluate('gnss_motion_validation','gnss_motion_validation.csv',lambda r:gnss_motion_metrics(r,loc),gnss_gate)

 evaluate('localization_innovations','localization_innovations.csv',innovation_metrics,
  lambda r,m:(m['samples']>=100 and m['wheel_gate_pass_ratio']>=0.85 and m['cog_gate_pass_ratio']>=0.85,f"n={m['samples']} wheel_pass={m['wheel_gate_pass_ratio']:.3f} cog_pass={m['cog_gate_pass_ratio']:.3f} wheel_nis95={m['wheel_nis_p95']:.2f} cog_nis95={m['cog_nis_p95']:.2f}"))
 evaluate('localization_reacquisition','localization_reacquisition.csv',reacquisition_metrics,
  lambda r,m:(m['reacquisition_events']>=3 and m['position_jump_p95_m']<=0.30 and m['yaw_jump_p95_deg']<=5.0 and m['gate_open_at_reacquire_ratio']<=0.10,f"events={m['reacquisition_events']} jump95={m['position_jump_p95_m']:.3f}m yaw95={m['yaw_jump_p95_deg']:.2f}deg immediate_gate_open={m['gate_open_at_reacquire_ratio']:.2f}"))

 def nav2_gate(rows,m):
  ps=m['profiles'];std=ps.get('standard_8hz');pre=ps.get('precision_15hz')
  ok=bool(std and pre) and std['samples']>=80 and pre['samples']>=120 and std['rate_hz_median']>=7.2 and pre['rate_hz_median']>=13.5 and pre['period_jitter_p95_sec']<=0.020 and pre['cpu_p95_pct']<=85.0 and pre['cross_track_p95_m']<=0.15 and pre['cross_track_p95_m']<=max(0.15,1.20*std['cross_track_p95_m'])
  return ok,f"profiles={sorted(ps)} standard={std} precision={pre}"
 evaluate('nav2_ab','nav2_ab.csv',nav2_ab_metrics,nav2_gate)

 def control_field(rows):
  m=cal.control(rows,float(vehicle.get('effective_wheelbase_m',vehicle.get('wheelbase_m',.7))));dirs=sorted({r['direction'].strip().lower() for r in rows});m['directions']=dirs;m['reverse_yaw_feedback_qualified']='reverse' in dirs;return m
 evaluate('control_slalom','control_slalom.csv',control_field,
  lambda r,m:(m['estimated_steering_delay_sec']<=0.30 and math.isfinite(m.get('estimated_steering_tau_sec',math.nan)) and m['estimated_steering_tau_sec']<=0.50 and m['kinematic_yaw_rate_p95_rps']<=0.08,f"delay={m['estimated_steering_delay_sec']:.3f}s tau={m.get('estimated_steering_tau_sec',math.nan):.3f}s yaw95={m['kinematic_yaw_rate_p95_rps']:.3f}rad/s directions={m['directions']} reverse_qualified={m['reverse_yaw_feedback_qualified']}"))
 evaluate('ground_truth','ground_truth.csv',gt_metrics,
  lambda r,m:(m['position_p95_m']<=0.15 and m['yaw_p95_deg']<=2.0,f"position p95={m['position_p95_m']:.3f}m yaw p95={m['yaw_p95_deg']:.2f}deg"))
 def passed(*keys):return all(results.get(k,{}).get('status')=='PASS' for k in keys)
 foundation=('imu_stationary','imu_six_position','imu_motor_interference','imu_repeatability','drive_multispeed','steering_hysteresis','steering_circles','gnss_lever_arm','map_alignment','mag_interference','gnss_motion_validation','localization_innovations','localization_reacquisition','nav2_ab','control_slalom')
 tier1=passed(*foundation);gt=results['ground_truth'].get('metrics',{})
 tier2=tier1 and results['ground_truth']['status']=='PASS' and gt.get('position_p95_m',999)<=0.15
 tiers={'tier1_robust_calibrated_navigation':'PASS' if tier1 else 'PENDING','tier2_decimeter_local':'PASS' if tier2 else 'PENDING','precision_global':'DEFERRED_NO_RTK'}
 report={'generated_at':datetime.datetime.now().astimezone().isoformat(timespec='seconds'),'campaign':str(campaign.resolve()),'evidence_type':'PHYSICAL_FIELD_MEASUREMENT_REQUIRED','vehicle_max_forward_speed_mps':float(vehicle.get('max_forward_speed_mps',1.0)),'items':results,'tiers':tiers,'production_certification_changed':False,'evidence_sha256':{str(p.relative_to(campaign)):sha(p) for p in evidence if p.is_file()}}
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
