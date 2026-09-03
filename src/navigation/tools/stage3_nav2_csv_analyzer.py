#!/usr/bin/env python3
"""Read-only Stage-3 MPPI/actuator tuning CSV analyzer."""
from __future__ import annotations
import argparse,csv,json,math
from pathlib import Path
import yaml

def num(v):
    try:x=float(v)
    except (TypeError,ValueError): return None
    return x if math.isfinite(x) else None

def pick(r,names):
    for n in names:
        if n in r and str(r[n]).strip()!='': return r[n]
    return ''
def rms(xs): return math.sqrt(sum(x*x for x in xs)/len(xs)) if xs else math.nan

def analyze(csv_path:Path,cfg_path:Path):
    cfg=(yaml.safe_load(cfg_path.read_text()) or {})['stage3_navigation']['ros__parameters']
    with csv_path.open(newline='',encoding='utf-8-sig') as f: rows=list(csv.DictReader(f))
    vals={'velocity':[],'steering':[],'yaw':[]}
    aliases={
      'velocity':('v err','velocity_error_mps','mppi_velocity_error'),
      'steering':('steer err','steering_error_rad','mppi_steering_error'),
      'yaw':('yaw err','yaw_rate_error_rps','mppi_yaw_error')}
    for r in rows:
        for k,a in aliases.items():
            x=num(pick(r,a))
            if x is not None: vals[k].append(x)
    n=min((len(v) for v in vals.values()),default=0)
    vr, sr, yr = rms(vals['velocity']),rms(vals['steering']),rms(vals['yaw'])
    ok=(n>=int(cfg['stage3_min_tuning_samples']) and math.isfinite(vr) and vr<=float(cfg['stage3_max_velocity_rmse_mps']) and math.isfinite(sr) and sr<=float(cfg['stage3_max_steering_rmse_rad']) and math.isfinite(yr) and yr<=float(cfg['stage3_max_yaw_rate_rmse_rps']))
    return {'csv':str(csv_path),'usable_samples':n,'velocity_rmse_mps':vr,'steering_rmse_rad':sr,'yaw_rate_rmse_rps':yr,'pass':ok}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('csv',type=Path);ap.add_argument('--stage3-yaml',type=Path,default=Path(__file__).resolve().parents[1]/'config/stage3_navigation.yaml');ap.add_argument('--json',action='store_true');a=ap.parse_args()
    r=analyze(a.csv.expanduser(),a.stage3_yaml.expanduser())
    print(json.dumps(r,indent=2,allow_nan=True) if a.json else '\n'.join(f'{k:28s}: {v}' for k,v in r.items()))
    return 0 if r['pass'] else 2
if __name__=='__main__': raise SystemExit(main())
