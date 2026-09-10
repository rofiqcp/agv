#!/usr/bin/env python3
"""Generate deterministic JSON/CSV/Markdown navigation accuracy metrics."""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
import numpy as np

def load(p):
    with p.open(newline='',encoding='utf-8') as f:return list(csv.DictReader(f))
def col(r,k): return np.asarray([float(x[k]) for x in r],float)
def wrap(x):return np.arctan2(np.sin(x),np.cos(x))
def p95(x):return float(np.percentile(np.abs(x[np.isfinite(x)]),95))
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--csv',type=Path,required=True);ap.add_argument('--out-dir',type=Path,required=True);a=ap.parse_args();r=load(a.csv)
    if len(r)<5: raise SystemExit('need >=5 samples')
    t=col(r,'t');ex=col(r,'est_x');ey=col(r,'est_y');eyaw=col(r,'est_yaw');gx=col(r,'gt_x');gy=col(r,'gt_y');gyaw=col(r,'gt_yaw')
    pe=np.hypot(ex-gx,ey-gy);ye=wrap(eyaw-gyaw);dt=np.diff(t);good=dt[(dt>0)&np.isfinite(dt)]
    m={'samples':len(r),'duration_sec':float(t[-1]-t[0]),'position_rmse_m':float(np.sqrt(np.mean(pe*pe))),'position_p95_m':float(np.percentile(pe,95)),
       'final_position_error_m':float(pe[-1]),'yaw_rmse_rad':float(np.sqrt(np.mean(ye*ye))),'yaw_p95_rad':p95(ye),'yaw_p95_deg':math.degrees(p95(ye)),
       'rate_hz_median':float(1/np.median(good)),'period_jitter_p95_sec':float(np.percentile(np.abs(good-np.median(good)),95))}
    if {'cmd_v','actual_v','cmd_w','actual_w'} <= set(r[0]):
        cv,av,cw,aw=(col(r,k) for k in ('cmd_v','actual_v','cmd_w','actual_w'));m.update(velocity_rmse_mps=float(np.sqrt(np.mean((cv-av)**2))),yaw_rate_rmse_rps=float(np.sqrt(np.mean((cw-aw)**2))))
    # 1-second-ish relative position error using nearest index horizon.
    med=float(np.median(good));h=max(1,int(round(1.0/med)));re=[]
    for i in range(len(r)-h):
        de=np.array([ex[i+h]-ex[i],ey[i+h]-ey[i]]);dg=np.array([gx[i+h]-gx[i],gy[i+h]-gy[i]]);re.append(np.linalg.norm(de-dg))
    m['rpe_1s_p95_m']=float(np.percentile(re,95)) if re else math.nan
    a.out_dir.mkdir(parents=True,exist_ok=True);(a.out_dir/'metrics.json').write_text(json.dumps(m,indent=2)+'\n')
    with (a.out_dir/'metrics.csv').open('w',newline='') as f:w=csv.writer(f);w.writerow(['metric','value']);w.writerows(m.items())
    (a.out_dir/'metrics.md').write_text('# Navigation metrics\n\n'+'\n'.join(f'- **{k}**: `{v}`' for k,v in m.items())+'\n')
    print(json.dumps(m,indent=2))
if __name__=='__main__':raise SystemExit(main())
