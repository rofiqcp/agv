#!/usr/bin/env python3
import argparse, csv, json, math
from datetime import datetime
from pathlib import Path
import numpy as np
import yaml

def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def cmean(v): return math.atan2(sum(math.sin(x) for x in v), sum(math.cos(x) for x in v))
def lut_apply(y, knots, corr):
    y=wrap(y); pts=sorted(zip(knots,corr)); xs=[p[0] for p in pts]
    j=0
    while j<len(xs) and xs[j] < y: j+=1
    if j==0: x0,c0=pts[-1][0],pts[-1][1]; x1,c1=pts[0][0]+2*math.pi,pts[0][1]; yy=y+2*math.pi
    elif j==len(pts): x0,c0=pts[-1]; x1,c1=pts[0][0]+2*math.pi,pts[0][1]; yy=y
    else: x0,c0=pts[j-1]; x1,c1=pts[j]; yy=y
    f=max(0,min(1,(yy-x0)/max(1e-12,x1-x0)))
    return wrap(y+c0+f*wrap(c1-c0))

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('raw_csv'); ap.add_argument('--meta-json',default=''); a=ap.parse_args()
    raw_rows=[]; map_samples=[]
    with open(a.raw_csv,newline='') as f:
        for r in csv.DictReader(f):
            try:
                seg=int(r['segment']); st=r['state']; x=float(r['yah_mag_x_lsb']); y=float(r['yah_mag_y_lsb']);
                neo=math.radians(float(r['neo_yaw_deg'])); my=math.radians(float(r['map_yaw_from_enu_deg']))
            except Exception: continue
            if math.isfinite(my): map_samples.append(my)
            if seg in range(1,9) and st == f'STATIC_{seg}' and all(math.isfinite(v) for v in (x,y,neo)):
                raw_rows.append((seg,x,y,neo,my))
    if not map_samples: raise SystemExit('map_yaw_from_enu unavailable for calibration run')
    map_ref=cmean(map_samples)
    map_spread=max(abs(wrap(v-map_ref)) for v in map_samples)
    if map_spread > math.radians(0.25):
        raise SystemExit(f'map_yaw_from_enu changed during run: spread={math.degrees(map_spread):.3f} deg')
    rows=[]
    for seg,x,y,neo,my in raw_rows:
        # map_yaw_from_enu is a map-calibration transform, not a high-rate sensor.
        # When its low-rate publisher is stale in one IMU row, reuse the verified
        # run-constant value instead of discarding a valid static MAG sample.
        map_yaw = my if math.isfinite(my) else map_ref
        rows.append((seg,x,y,wrap(neo-map_yaw)))
    segs=sorted(set(r[0] for r in rows))
    if segs != list(range(1,9)): raise SystemExit(f'need segments 1..8, got {segs}')
    means=[]
    for s in segs:
        q=[r for r in rows if r[0]==s]; means.append((s,np.mean([r[1] for r in q]),np.mean([r[2] for r in q]),cmean([r[3] for r in q])))
    P=np.array([[m[1],m[2]] for m in means],dtype=float)
    bias=(P.max(axis=0)+P.min(axis=0))/2.0
    C=np.cov((P-bias).T,bias=False); ev,V=np.linalg.eigh(C)
    if np.min(ev)<=1e-9: raise SystemExit('degenerate XY geometry')
    W=V @ np.diag(1.0/np.sqrt(ev)) @ V.T
    Q=(P-bias) @ W.T
    radius=np.linalg.norm(Q,axis=1); W=W/np.median(radius); Q=(P-bias)@W.T
    alpha=np.arctan2(Q[:,1],Q[:,0]); target=np.array([m[3] for m in means])
    best=None
    for sign in (-1.0,1.0):
        offs=[wrap(float(t-sign*a0)) for t,a0 in zip(target,alpha)]; off=cmean(offs)
        base=np.array([wrap(sign*float(a0)+off) for a0 in alpha]); err=np.array([wrap(float(t-b)) for t,b in zip(target,base)])
        rms=math.degrees(math.sqrt(float(np.mean(err*err))))
        if best is None or rms<best[0]: best=(rms,sign,off,base,err)
    _,sign,off,base,err=best
    knots=[float(x) for x in base]; corr=[float(e) for e in err]
    sample_err=[]; corrected_norm=[]
    for _,x,y,t in rows:
        q=W@np.array([x-bias[0],y-bias[1]]); corrected_norm.append(float(np.linalg.norm(q)))
        b=wrap(sign*math.atan2(float(q[1]),float(q[0]))+off); h=lut_apply(b,knots,corr); sample_err.append(wrap(h-t))
    rms=math.degrees(math.sqrt(sum(e*e for e in sample_err)/len(sample_err))); mx=math.degrees(max(abs(e) for e in sample_err))
    meta={}
    if a.meta_json and Path(a.meta_json).exists(): meta=json.loads(Path(a.meta_json).read_text())
    cw=meta.get('transition_checks',[])
    grouped={s:[] for s in range(1,8)}
    for item in cw:
        try:
            fs=int(item.get('from_segment')); d=float(item.get('gyro_delta_deg'))
            if fs in grouped and math.isfinite(d) and abs(d)>=5.0: grouped[fs].append(d)
        except Exception:
            pass
    # Debounce duplicates from stationary-state chatter: each transition must have
    # at least one meaningful CW rotation and must not contain a meaningful CCW reversal.
    cw_ok=all(any(d < -5.0 for d in grouped[s]) and not any(d > 5.0 for d in grouped[s]) for s in range(1,8))
    valid=cw_ok and rms<3.0 and mx<5.0 and len(rows)>=200
    out={'yahboom_mag_planar_calibration':{'valid':bool(valid),'source':'8-direction CW static fit against calibrated IST8310 ENU heading','created_at':datetime.now().isoformat(),'raw_csv':str(Path(a.raw_csv).resolve()),'sample_count':len(rows),'segment_count':8,'cw_transition_check_pass':bool(cw_ok),'map_yaw_from_enu_rad':float(map_ref),'map_yaw_spread_deg':float(math.degrees(map_spread)),'bias_xy_lsb':[float(bias[0]),float(bias[1])],'matrix_xy_per_lsb':[float(W[0,0]),float(W[0,1]),float(W[1,0]),float(W[1,1])],'yaw_sign':float(sign),'yaw_offset_rad':float(off),'heading_lut_input_rad':knots,'heading_lut_correction_rad':corr,'corrected_norm_mean':float(np.mean(corrected_norm)),'corrected_norm_std':float(np.std(corrected_norm)),'validation':{'rms_error_deg':float(rms),'max_abs_error_deg':float(mx),'pass':bool(valid)},'scope':'planar XY only; Z/3D spherical calibration requires multi-axis rotation'}}
    root=Path(a.raw_csv).parent; stamp=Path(a.raw_csv).stem.replace('heading_8dir_raw_','')
    dest=root/f'yahboom_mag_planar_{stamp}.yaml'; dest.write_text(yaml.safe_dump(out,sort_keys=False)); (root/'yahboom_mag_planar_latest.yaml').write_text(yaml.safe_dump(out,sort_keys=False))
    print(f'YAHBOOM_PLANAR_FIT valid={valid} samples={len(rows)} cw_ok={cw_ok} rms={rms:.3f}deg max={mx:.3f}deg yaml={dest}',flush=True)
    return 0 if valid else 2
if __name__=='__main__': raise SystemExit(main())
