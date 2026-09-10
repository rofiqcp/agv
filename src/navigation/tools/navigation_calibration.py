#!/usr/bin/env python3
"""Evidence-first calibration analyzers for AGV navigation.

No command writes runtime calibration YAML automatically. Each subcommand only
analyzes measured data and writes JSON/Markdown evidence; certification remains
an explicit separate action after reviewing the raw trial.
"""
from __future__ import annotations
import argparse, csv, json, math
from pathlib import Path
import numpy as np


def load_csv(path: Path):
    with path.open(newline='', encoding='utf-8') as f:
        rows=list(csv.DictReader(f))
    if not rows: raise ValueError('CSV has no data rows')
    return rows

def arr(rows,key):
    try: return np.asarray([float(r[key]) for r in rows],dtype=float)
    except KeyError as e: raise ValueError(f'missing column: {key}') from e

def p95_abs(x):
    x=np.asarray(x,dtype=float); x=x[np.isfinite(x)]
    return float(np.percentile(np.abs(x),95)) if x.size else math.nan

def angle_wrap(x): return np.arctan2(np.sin(x),np.cos(x))

def output(result, out_json: Path|None, out_md: Path|None):
    text=json.dumps(result,indent=2,sort_keys=True)
    print(text)
    if out_json:
        out_json.parent.mkdir(parents=True,exist_ok=True); out_json.write_text(text+'\n')
    if out_md:
        out_md.parent.mkdir(parents=True,exist_ok=True)
        lines=[f"# {result.get('analysis','Navigation calibration')} evidence",'']
        def walk(prefix,obj):
            for k,v in obj.items():
                if isinstance(v,dict):
                    lines.append(f"## {prefix}{k}"); walk('',v)
                elif isinstance(v,list): lines.append(f"- **{prefix}{k}**: `{json.dumps(v)}`")
                else: lines.append(f"- **{prefix}{k}**: `{v}`")
        walk('',result); out_md.write_text('\n'.join(lines)+'\n')

def allan_deviation(rate, dt):
    rate=np.asarray(rate,dtype=float); rate=rate[np.isfinite(rate)]
    n=rate.size
    if n<20: return []
    max_m=max(1,n//10); ms=np.unique(np.geomspace(1,max_m,num=min(18,max_m),dtype=int))
    out=[]
    for m in ms:
        k=n//m
        if k<3: continue
        y=rate[:k*m].reshape(k,m).mean(axis=1)
        adev=math.sqrt(0.5*float(np.mean(np.diff(y)**2)))
        out.append({'tau_sec':float(m*dt),'adev':adev})
    return out

def imu(rows):
    t=arr(rows,'t'); ax,ay,az=(arr(rows,k) for k in ('ax','ay','az')); gx,gy,gz=(arr(rows,k) for k in ('gx','gy','gz'))
    dt=np.diff(t); dt=dt[np.isfinite(dt)&(dt>0)]
    med_dt=float(np.median(dt)) if dt.size else math.nan
    norm=np.sqrt(ax*ax+ay*ay+az*az)
    return {'analysis':'IMU stationary/Allan','samples':len(rows),'duration_sec':float(t[-1]-t[0]),'median_dt_sec':med_dt,
      'accel_mean_mps2':[float(np.mean(x)) for x in (ax,ay,az)],'accel_std_mps2':[float(np.std(x,ddof=1)) for x in (ax,ay,az)],
      'gyro_mean_rps':[float(np.mean(x)) for x in (gx,gy,gz)],'gyro_std_rps':[float(np.std(x,ddof=1)) for x in (gx,gy,gz)],
      'accel_norm_mean_mps2':float(np.mean(norm)),'accel_norm_error_mps2':float(abs(np.mean(norm)-9.80665)),
      'gyro_z_allan': allan_deviation(gz,med_dt) if math.isfinite(med_dt) else []}

def drive(rows, baseline_erpm_per_mps):
    x=arr(rows,'erpm'); y=arr(rows,'ground_speed_mps'); good=np.isfinite(x)&np.isfinite(y)
    x=x[good];y=y[good]
    if x.size<8: raise ValueError('drive fit requires >=8 samples')
    A=np.column_stack([x,np.ones_like(x)]); a,b=np.linalg.lstsq(A,y,rcond=None)[0]
    pred=a*x+b; res=y-pred
    def signed(mask):
        if np.count_nonzero(mask)<3:return {'samples':int(np.count_nonzero(mask))}
        aa,bb=np.linalg.lstsq(np.column_stack([x[mask],np.ones(np.count_nonzero(mask))]),y[mask],rcond=None)[0]
        rr=y[mask]-(aa*x[mask]+bb)
        return {'samples':int(np.count_nonzero(mask)),'slope_mps_per_erpm':float(aa),'intercept_mps':float(bb),'residual_p95_mps':p95_abs(rr)}
    f=signed(x>0); r=signed(x<0)
    asym=abs(abs(f.get('slope_mps_per_erpm',a))-abs(r.get('slope_mps_per_erpm',a))) / max(abs(a),1e-12)
    return {'analysis':'Drive eRPM -> ground speed fit','samples':int(x.size),'slope_mps_per_erpm':float(a),'intercept_mps':float(b),
      'equivalent_erpm_per_mps':float(1.0/a) if abs(a)>1e-12 else math.inf,
      'multiplicative_scale_vs_baseline':float(a*baseline_erpm_per_mps),'residual_rmse_mps':float(np.sqrt(np.mean(res*res))),
      'residual_p95_mps':p95_abs(res),'forward':f,'reverse':r,'direction_slope_asymmetry_fraction':float(asym)}

def circle_fit(x,y):
    A=np.column_stack([2*x,2*y,np.ones_like(x)]); rhs=x*x+y*y
    cx,cy,c=np.linalg.lstsq(A,rhs,rcond=None)[0]; r=math.sqrt(max(0.0,c+cx*cx+cy*cy))
    radial=np.sqrt((x-cx)**2+(y-cy)**2); return float(cx),float(cy),r,float(np.sqrt(np.mean((radial-r)**2)))

def circles(rows):
    x=arr(rows,'x_m');y=arr(rows,'y_m'); dirs=np.asarray([r['direction'].strip().lower() for r in rows])
    result={'analysis':'Steering circle geometry','samples':len(rows),'directions':{}}
    radii=[]
    for d in sorted(set(dirs)):
        m=dirs==d
        if np.count_nonzero(m)<6: continue
        cx,cy,rad,rmse=circle_fit(x[m],y[m]);radii.append(rad)
        result['directions'][d]={'samples':int(np.count_nonzero(m)),'center_m':[cx,cy],'radius_m':rad,'fit_rmse_m':rmse}
    if not radii: raise ValueError('need >=6 points per at least one direction')
    result['minimum_measured_radius_m']=float(min(radii)); result['left_right_radius_difference_m']=float(max(radii)-min(radii)) if len(radii)>1 else math.nan
    return result

def map_align(rows):
    src=np.column_stack([arr(rows,'raw_x'),arr(rows,'raw_y')]); dst=np.column_stack([arr(rows,'true_x'),arr(rows,'true_y')])
    if len(src)<3: raise ValueError('map alignment requires >=3 points')
    sm=src.mean(0);dm=dst.mean(0); X=src-sm;Y=dst-dm; H=X.T@Y; U,S,Vt=np.linalg.svd(H);R=Vt.T@U.T
    if np.linalg.det(R)<0: Vt[-1]*=-1;R=Vt.T@U.T
    trans=dm-R@sm; pred=(R@src.T).T+trans; e=np.linalg.norm(pred-dst,axis=1); yaw=math.atan2(R[1,0],R[0,0])
    cov=np.cov(dst.T); eig=np.linalg.eigvalsh(cov); geom=math.sqrt(max(eig[0],0)/max(eig[-1],1e-12))
    baseline=max(float(np.linalg.norm(dst[i]-dst[j])) for i in range(len(dst)) for j in range(i))
    return {'analysis':'Map SE(2) alignment','samples':len(src),'translation_m':[float(trans[0]),float(trans[1])],'yaw_rad':yaw,
      'rmse_m':float(np.sqrt(np.mean(e*e))),'residual_p95_m':float(np.percentile(e,95)),'geometry_score':geom,'baseline_m':baseline}

def mag(rows):
    ref=arr(rows,'heading_ref_rad'); obs=arr(rows,'heading_mag_rad'); e=angle_wrap(obs-ref)
    result={'analysis':'Magnetic heading interference','samples':len(rows),'heading_residual_mean_rad':float(np.mean(e)),
            'heading_residual_p95_rad':p95_abs(e),'heading_residual_p95_deg':math.degrees(p95_abs(e))}
    if 'current_a' in rows[0]:
        cur=arr(rows,'current_a'); result['abs_residual_current_correlation']=float(np.corrcoef(np.abs(e),cur)[0,1]) if np.std(cur)>1e-9 else 0.0
    return result

def control(rows,wheelbase):
    t=arr(rows,'t');cmd=arr(rows,'steer_cmd_rad');act=arr(rows,'steer_actual_rad');v=arr(rows,'speed_mps');w=arr(rows,'yaw_rate_rps')
    dt=np.median(np.diff(t)); maxlag=max(1,int(1.0/max(dt,1e-4))); cor=[]
    c0=cmd-np.mean(cmd);a0=act-np.mean(act)
    for lag in range(maxlag+1):
        if lag==0: aa,cc=a0,c0
        else: aa,cc=a0[lag:],c0[:-lag]
        cor.append(float(np.dot(aa,cc)/(np.linalg.norm(aa)*np.linalg.norm(cc)+1e-12)))
    lag=int(np.argmax(cor)); pred=np.where(np.abs(act)>1e-6,v*np.tan(act)/wheelbase,0.0); err=w-pred
    return {'analysis':'Steering/control identification','samples':len(rows),'median_dt_sec':float(dt),'estimated_steering_delay_sec':float(lag*dt),
      'steering_tracking_rmse_rad':float(np.sqrt(np.mean((act-cmd)**2))),'kinematic_yaw_rate_rmse_rps':float(np.sqrt(np.mean(err*err))),
      'kinematic_yaw_rate_p95_rps':p95_abs(err),'peak_delay_correlation':cor[lag]}

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--json',type=Path);ap.add_argument('--markdown',type=Path)
    sub=ap.add_subparsers(dest='cmd',required=True)
    for name in ('imu','circle','map','mag'): q=sub.add_parser(name);q.add_argument('--csv',type=Path,required=True)
    q=sub.add_parser('drive');q.add_argument('--csv',type=Path,required=True);q.add_argument('--baseline-erpm-per-mps',type=float,default=8000.0)
    q=sub.add_parser('control');q.add_argument('--csv',type=Path,required=True);q.add_argument('--wheelbase',type=float,default=0.7)
    a=ap.parse_args();rows=load_csv(a.csv)
    fn={'imu':lambda:imu(rows),'drive':lambda:drive(rows,a.baseline_erpm_per_mps),'circle':lambda:circles(rows),'map':lambda:map_align(rows),'mag':lambda:mag(rows),'control':lambda:control(rows,a.wheelbase)}[a.cmd]
    output(fn(),a.json,a.markdown)
if __name__=='__main__': raise SystemExit(main())
