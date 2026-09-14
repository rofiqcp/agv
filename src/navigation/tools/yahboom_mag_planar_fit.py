#!/usr/bin/env python3
from __future__ import annotations
import argparse, csv, json, math, os
from datetime import datetime
from pathlib import Path
import numpy as np
import yaml

AGV_ROOT=Path(os.environ.get('AGV_ROOT', str(Path.home()/'agv'))).expanduser().resolve()
FIT_SEGMENTS=tuple(range(1,9)); CLOSURE_SEGMENT=9; STOP_COUNT=9
FIT_RMS_MAX_DEG=3.0; FIT_MAX_ERROR_DEG=5.0; CLOSURE_MAX_DEG=2.0
CROSS_RMS_MAX_DEG=3.0; CROSS_MAX_DEG=5.0; GYRO_CLOSURE_MAX_DEG=20.0
NORM_P05_MIN=0.75; NORM_P95_MAX=1.25; MIN_SAMPLES_PER_STOP=15

def portable_path(path):
    resolved=Path(path).expanduser().resolve()
    try:return str(resolved.relative_to(AGV_ROOT))
    except ValueError:return str(resolved)

def wrap(a): return math.atan2(math.sin(a),math.cos(a))
def cmean(v):
    return math.atan2(sum(math.sin(x) for x in v),sum(math.cos(x) for x in v)) if v else float('nan')
def rms_deg(errors):
    return math.degrees(math.sqrt(sum(e*e for e in errors)/len(errors))) if errors else float('inf')
def finite(v): return isinstance(v,(int,float)) and math.isfinite(v)
def compass_target_deg(segment,direction):
    step=45.0*(segment-1); return (step if direction=='CW' else -step)%360.0
def enu_target_rad(segment,direction):
    return wrap(math.pi/2.0-math.radians(compass_target_deg(segment,direction)))
def lut_apply(y,knots,corr):
    y=wrap(y); pts=sorted(zip(knots,corr)); xs=[p[0] for p in pts]
    if len(pts)<2:return y
    j=0
    while j<len(xs) and xs[j]<y:j+=1
    if j==0:x0,c0=pts[-1][0],pts[-1][1];x1,c1=pts[0][0]+2*math.pi,pts[0][1];yy=y+2*math.pi
    elif j==len(pts):x0,c0=pts[-1];x1,c1=pts[0][0]+2*math.pi,pts[0][1];yy=y
    else:x0,c0=pts[j-1];x1,c1=pts[j];yy=y
    f=max(0.0,min(1.0,(yy-x0)/max(1e-12,x1-x0)))
    return wrap(y+c0+f*wrap(c1-c0))

def model_heading(model,x,y):
    b=model['bias'];m=model['matrix'];bx=x-b[0];by=y-b[1]
    qx=m[0]*bx+m[1]*by;qy=m[2]*bx+m[3]*by
    base=wrap(model['yaw_sign']*math.atan2(qy,qx)+model['yaw_offset_rad'])
    return lut_apply(base,model['heading_lut_input_rad'],model['heading_lut_correction_rad']),math.hypot(qx,qy)

def fit_planar(rows,xkey,ykey,label,unit):
    per={s:[r for r in rows if r['segment']==s and finite(r.get(xkey)) and finite(r.get(ykey))] for s in range(1,STOP_COUNT+1)}
    counts={s:len(per[s]) for s in per}
    if any(counts[s]<MIN_SAMPLES_PER_STOP for s in range(1,STOP_COUNT+1)):
        raise ValueError(f'{label}: sample per stop kurang: {counts}')
    means=[]
    for s in FIT_SEGMENTS:
        q=per[s];means.append((s,float(np.mean([r[xkey] for r in q])),float(np.mean([r[ykey] for r in q])),enu_target_rad(s,rows[0]['direction'])))
    P=np.array([[m[1],m[2]] for m in means],dtype=float);bias=(P.max(axis=0)+P.min(axis=0))/2.0
    C=np.cov((P-bias).T,bias=False);ev,V=np.linalg.eigh(C)
    if np.min(ev)<=1e-9 or not np.all(np.isfinite(ev)):raise ValueError(f'{label}: geometri XY degenerat')
    W=V@np.diag(1.0/np.sqrt(ev))@V.T;Q=(P-bias)@W.T
    radius=np.linalg.norm(Q,axis=1);med=float(np.median(radius))
    if not math.isfinite(med) or med<=1e-9:raise ValueError(f'{label}: radius fit invalid')
    W=W/med;Q=(P-bias)@W.T;alpha=np.arctan2(Q[:,1],Q[:,0]);target=np.array([m[3] for m in means])
    best=None
    for sign in (-1.0,1.0):
        off=cmean([wrap(float(t-sign*a)) for t,a in zip(target,alpha)])
        base=np.array([wrap(sign*float(a)+off) for a in alpha]);err=np.array([wrap(float(t-b)) for t,b in zip(target,base)])
        score=rms_deg(err.tolist())
        if best is None or score<best[0]:best=(score,sign,off,base,err)
    _,sign,off,base,err=best;knots=[float(v) for v in base];corr=[float(v) for v in err]
    model={'bias':[float(bias[0]),float(bias[1])],'matrix':[float(W[0,0]),float(W[0,1]),float(W[1,0]),float(W[1,1])],
           'yaw_sign':float(sign),'yaw_offset_rad':float(off),'heading_lut_input_rad':knots,'heading_lut_correction_rad':corr}
    fit_err=[];closure_err=[];norms=[];per_stop=[]
    for s in range(1,STOP_COUNT+1):
        hs=[];ns=[];tgt=enu_target_rad(s,rows[0]['direction'])
        for r in per[s]:
            h,n=model_heading(model,r[xkey],r[ykey]);hs.append(h);ns.append(n);norms.append(n)
            (fit_err if s in FIT_SEGMENTS else closure_err).append(wrap(h-tgt))
        mean_h=cmean(hs);per_stop.append({'segment':s,'target_compass_deg':compass_target_deg(s,rows[0]['direction']),
          'target_enu_deg':math.degrees(tgt),'predicted_enu_deg':math.degrees(mean_h),'mean_error_deg':math.degrees(wrap(mean_h-tgt)),
          'samples':len(hs),'corrected_norm_mean':float(np.mean(ns))})
    arr=np.asarray(norms,dtype=float);p05=float(np.percentile(arr,5));p95=float(np.percentile(arr,95))
    fr=rms_deg(fit_err);fm=math.degrees(max(abs(e) for e in fit_err));cr=rms_deg(closure_err);cm=math.degrees(max(abs(e) for e in closure_err));cmean_abs=abs(math.degrees(cmean(closure_err)))
    norm_ok=p05>=NORM_P05_MIN and p95<=NORM_P95_MAX
    valid=fr<FIT_RMS_MAX_DEG and fm<FIT_MAX_ERROR_DEG and cmean_abs<=CLOSURE_MAX_DEG and cm<FIT_MAX_ERROR_DEG and norm_ok
    return {'valid':bool(valid),'sensor':label,'unit':unit,'sample_count':sum(counts.values()),'fit_sample_count':sum(counts[s] for s in FIT_SEGMENTS),
      'closure_sample_count':counts[CLOSURE_SEGMENT],'bias':model['bias'],'matrix':model['matrix'],'yaw_sign':model['yaw_sign'],'yaw_offset_rad':model['yaw_offset_rad'],
      'heading_lut_input_rad':knots,'heading_lut_correction_rad':corr,'corrected_norm_mean':float(np.mean(arr)),'corrected_norm_std':float(np.std(arr)),
      'corrected_norm_p05':p05,'corrected_norm_p95':p95,'norm_gate_pass':bool(norm_ok),'per_stop':per_stop,
      'validation':{'fit_rms_error_deg':fr,'fit_max_abs_error_deg':fm,'closure_rms_error_deg':cr,'closure_max_abs_error_deg':cm,
        'closure_mean_error_deg':math.degrees(cmean(closure_err)),'rms_error_deg':fr,'max_abs_error_deg':fm,'pass':bool(valid)}}

def load_rows(path,direction):
    rows=[]
    with open(path,newline='') as f:
        for r in csv.DictReader(f):
            try:seg=int(r.get('segment','0'));state=r.get('state','')
            except Exception:continue
            if seg not in range(1,STOP_COUNT+1) or state!=f'STATIC_{seg}':continue
            one={'segment':seg,'direction':direction}
            for key in ('yah_mag_x_lsb','yah_mag_y_lsb','yah_mag_z_lsb','neo_mag_x_ut','neo_mag_y_ut','neo_mag_z_ut'):
                try:one[key]=float(r.get(key,'nan'))
                except Exception:one[key]=float('nan')
            rows.append(one)
    return rows

def transition_evidence(meta,direction):
    checks=meta.get('transition_checks',[]) if isinstance(meta,dict) else []
    by={s:[] for s in range(1,9)}
    for item in checks:
        try:
            fs=int(item.get('from_segment'));d=float(item.get('gyro_delta_deg'))
            if fs in by and math.isfinite(d):by[fs].append(d)
        except Exception:pass
    expected_sign=-1.0 if direction=='CW' else 1.0;step_ok=True;selected=[]
    for s in range(1,9):
        cand=[d for d in by[s] if expected_sign*d>0]
        if not cand:step_ok=False;continue
        d=max(cand,key=abs);selected.append(d)
        if abs(d)<25.0 or abs(d)>65.0:step_ok=False
    total=float(meta.get('gyro_total_turn_deg',sum(selected))) if meta else sum(selected);expected=expected_sign*360.0
    closure_error=abs(total-expected);return {'pass':bool(step_ok and closure_error<=GYRO_CLOSURE_MAX_DEG),'step_pass':bool(step_ok),
      'steps_deg':[float(v) for v in selected],'total_turn_deg':total,'expected_turn_deg':expected,'closure_error_deg':closure_error}

def cross_sensor(rows,yah,neo):
    diffs=[];per=[]
    for s in range(1,STOP_COUNT+1):
        ds=[]
        for r in rows:
            if r['segment']!=s or not all(finite(r.get(k)) for k in ('yah_mag_x_lsb','yah_mag_y_lsb','neo_mag_x_ut','neo_mag_y_ut')):continue
            yh,_=model_heading({'bias':yah['bias'],'matrix':yah['matrix'],'yaw_sign':yah['yaw_sign'],'yaw_offset_rad':yah['yaw_offset_rad'],'heading_lut_input_rad':yah['heading_lut_input_rad'],'heading_lut_correction_rad':yah['heading_lut_correction_rad']},r['yah_mag_x_lsb'],r['yah_mag_y_lsb'])
            nh,_=model_heading({'bias':neo['bias'],'matrix':neo['matrix'],'yaw_sign':neo['yaw_sign'],'yaw_offset_rad':neo['yaw_offset_rad'],'heading_lut_input_rad':neo['heading_lut_input_rad'],'heading_lut_correction_rad':neo['heading_lut_correction_rad']},r['neo_mag_x_ut'],r['neo_mag_y_ut'])
            d=wrap(yh-nh);ds.append(d);diffs.append(d)
        if ds:per.append({'segment':s,'mean_delta_deg':math.degrees(cmean(ds)),'rms_delta_deg':rms_deg(ds)})
    rr=rms_deg(diffs);mx=math.degrees(max(abs(v) for v in diffs)) if diffs else float('inf')
    return {'pass':bool(rr<=CROSS_RMS_MAX_DEG and mx<=CROSS_MAX_DEG),'rms_delta_deg':rr,'max_abs_delta_deg':mx,'per_stop':per}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('raw_csv');ap.add_argument('--meta-json',default='');ap.add_argument('--direction',choices=['CW','CCW'],default='');a=ap.parse_args()
    meta={}
    if a.meta_json and Path(a.meta_json).exists():meta=json.loads(Path(a.meta_json).read_text())
    direction=(a.direction or str(meta.get('expected_rotation','CW'))).upper();rows=load_rows(a.raw_csv,direction)
    present=sorted(set(r['segment'] for r in rows))
    if present!=list(range(1,STOP_COUNT+1)):raise SystemExit(f'need static segments 1..9 including North closure, got {present}')
    try:
        yah=fit_planar(rows,'yah_mag_x_lsb','yah_mag_y_lsb','Yahboom MAG','LSB')
        neo=fit_planar(rows,'neo_mag_x_ut','neo_mag_y_ut','NEO3 IST8310','uT')
    except ValueError as exc:raise SystemExit(str(exc))
    gyro=transition_evidence(meta,direction);cross=cross_sensor(rows,yah,neo)
    ready=bool(yah['valid'] and neo['valid'] and gyro['pass'] and cross['pass'])
    created=datetime.now().astimezone().isoformat(timespec='milliseconds')
    unified={'stage1_only':True,'runtime_yaml_written':False,'ready_for_stage2':ready,'valid':ready,'source':'operator-referenced physical North 0deg, 45deg static stops, full 360deg closure',
      'direction':direction,'created_at':created,'raw_csv':portable_path(a.raw_csv),'segment_count':STOP_COUNT,'fit_segment_count':8,'closure_segment':9,
      'ground_truth':{'convention':'compass_deg_0_North_90_East_CW','ros_conversion':'yaw_enu = pi/2 - compass_heading','north_reference':'operator_physical_alignment'},
      'transition_check_pass':gyro['pass'],'gyro':gyro,'yahboom':yah,'neo3':neo,'cross_sensor':cross,
      'gates':{'fit_rms_max_deg':FIT_RMS_MAX_DEG,'fit_max_error_deg':FIT_MAX_ERROR_DEG,'north_closure_max_deg':CLOSURE_MAX_DEG,
        'cross_rms_max_deg':CROSS_RMS_MAX_DEG,'cross_max_deg':CROSS_MAX_DEG,'gyro_closure_max_deg':GYRO_CLOSURE_MAX_DEG,
        'corrected_norm_p05_min':NORM_P05_MIN,'corrected_norm_p95_max':NORM_P95_MAX,'min_samples_per_stop':MIN_SAMPLES_PER_STOP}}
    legacy={'valid':yah['valid'],'stage1_only':True,'source':'9-stop North-referenced Stage-1 fit; YAML/runtime apply intentionally locked until Stage 2',
      'direction':direction,'created_at':created,'raw_csv':portable_path(a.raw_csv),'sample_count':yah['fit_sample_count'],'segment_count':STOP_COUNT,
      'transition_check_pass':gyro['pass'],'bias_xy_lsb':yah['bias'],'matrix_xy_per_lsb':yah['matrix'],'yaw_sign':yah['yaw_sign'],'yaw_offset_rad':yah['yaw_offset_rad'],
      'heading_lut_input_rad':yah['heading_lut_input_rad'],'heading_lut_correction_rad':yah['heading_lut_correction_rad'],'corrected_norm_mean':yah['corrected_norm_mean'],
      'corrected_norm_std':yah['corrected_norm_std'],'validation':yah['validation'],'scope':'planar XY; stop 9 is validation-only North closure'}
    out={'heading_360_calibration':unified,'yahboom_mag_planar_calibration':legacy,
      'neo3_planar_candidate':{'valid':neo['valid'],'stage1_only':True,'bias_xy_ut':neo['bias'],'matrix_xy':neo['matrix'],'yaw_sign':neo['yaw_sign'],
        'yaw_offset_rad':neo['yaw_offset_rad'],'heading_lut_input_rad':neo['heading_lut_input_rad'],'heading_lut_correction_rad':neo['heading_lut_correction_rad'],
        'validation':neo['validation'],'requires_calibration_owner_verification':True}}
    root=Path(a.raw_csv).parent;stem=Path(a.raw_csv).stem;stamp=stem.split('_')[-2]+'_'+stem.split('_')[-1] if len(stem.split('_'))>=2 else datetime.now().strftime('%Y%m%d_%H%M%S')
    dest=root/f'heading_360_{stamp}.yaml';payload=yaml.safe_dump(out,sort_keys=False,width=120)
    dest.write_text(payload);(root/'heading_360_latest.yaml').write_text(payload);(root/'yahboom_mag_planar_latest.yaml').write_text(payload)
    print(f'HEADING360_STAGE1 valid={ready} direction={direction} yah_rms={yah["validation"]["fit_rms_error_deg"]:.3f}deg neo_rms={neo["validation"]["fit_rms_error_deg"]:.3f}deg yah_close={abs(yah["validation"]["closure_mean_error_deg"]):.3f}deg neo_close={abs(neo["validation"]["closure_mean_error_deg"]):.3f}deg cross={cross["rms_delta_deg"]:.3f}deg gyro_close={gyro["closure_error_deg"]:.3f}deg yaml={dest}',flush=True)
    return 0 if ready else 2
if __name__=='__main__':raise SystemExit(main())
