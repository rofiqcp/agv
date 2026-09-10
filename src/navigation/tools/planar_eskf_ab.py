#!/usr/bin/env python3
"""Offline planar bias-aware EKF A/B with delayed-measurement rewind/replay.

CSV required columns: t, wheel_v, imu_wz.
Optional aiding columns on their arrival row:
  gnss_x, gnss_y, gnss_var, gnss_measurement_t
  cog_yaw, cog_var, cog_measurement_t
Optional comparison columns: gt_x, gt_y, gt_yaw, baseline_x, baseline_y, baseline_yaw.

This tool never replaces the runtime estimator automatically. It produces evidence
for the P2 decision gate: naive current-time fusion vs rewind/replay + gyro bias.
"""
from __future__ import annotations
import argparse,csv,json,math
from dataclasses import dataclass
from pathlib import Path
import numpy as np

def wrap(a): return math.atan2(math.sin(a),math.cos(a))
def finite(v):
    try:return math.isfinite(float(v))
    except (TypeError,ValueError):return False

def fval(row,key,default=math.nan):
    v=row.get(key,'')
    return float(v) if finite(v) else default

class Filter:
    def __init__(self):
        self.x=np.zeros(4,float) # x,y,yaw,gyro_bias
        self.P=np.diag([4.0,4.0,1.0,0.05])
        self.nis_pos=[];self.nis_yaw=[];self.reject_pos=0;self.reject_yaw=0
    def copy(self):
        q=Filter();q.x=self.x.copy();q.P=self.P.copy();q.nis_pos=list(self.nis_pos);q.nis_yaw=list(self.nis_yaw);q.reject_pos=self.reject_pos;q.reject_yaw=self.reject_yaw;return q
    def predict(self,dt,v,wz):
        if not (dt>0 and math.isfinite(dt)):return
        yaw=float(self.x[2]); b=float(self.x[3]); omega=wz-b
        self.x[0]+=v*math.cos(yaw)*dt; self.x[1]+=v*math.sin(yaw)*dt; self.x[2]=wrap(self.x[2]+omega*dt)
        F=np.eye(4);F[0,2]=-v*math.sin(yaw)*dt;F[1,2]=v*math.cos(yaw)*dt;F[2,3]=-dt
        # Conservative planar process model; tune only from bag evidence.
        qpos=(0.03+0.04*abs(v))**2*max(dt,1e-3); qyaw=(0.02+0.03*abs(omega))**2*max(dt,1e-3); qbias=(0.0015**2)*max(dt,1e-3)
        self.P=F@self.P@F.T+np.diag([qpos,qpos,qyaw,qbias])
    def pos_update(self,z,var,gate=9.210340372):
        H=np.array([[1.,0,0,0],[0,1.,0,0]]);R=np.eye(2)*max(var,1e-6);innov=np.asarray(z)-H@self.x;S=H@self.P@H.T+R
        nis=float(innov.T@np.linalg.solve(S,innov));self.nis_pos.append(nis)
        if nis>gate:self.reject_pos+=1;return False
        K=self.P@H.T@np.linalg.inv(S);self.x=self.x+K@innov;I=np.eye(4);self.P=(I-K@H)@self.P@(I-K@H).T+K@R@K.T;return True
    def yaw_update(self,z,var,gate=6.634896601):
        H=np.array([[0.,0,1.,0.]]);R=np.array([[max(var,1e-6)]]);innov=np.array([wrap(z-self.x[2])]);S=H@self.P@H.T+R;nis=float(innov.T@np.linalg.solve(S,innov));self.nis_yaw.append(nis)
        if nis>gate:self.reject_yaw+=1;return False
        K=self.P@H.T@np.linalg.inv(S);self.x=self.x+(K@innov);self.x[2]=wrap(self.x[2]);I=np.eye(4);self.P=(I-K@H)@self.P@(I-K@H).T+K@R@K.T;return True

@dataclass
class Hist:
    t:float; state:np.ndarray; P:np.ndarray; dt:float; v:float; wz:float

class RewindFilter:
    def __init__(self,lag_sec=3.0): self.f=Filter();self.h=[];self.lag=lag_sec
    def predict(self,t,dt,v,wz):
        self.f.predict(dt,v,wz);self.h.append(Hist(t,self.f.x.copy(),self.f.P.copy(),dt,v,wz))
        cutoff=t-self.lag
        while len(self.h)>2 and self.h[1].t<cutoff:self.h.pop(0)
    def update(self,mt,kind,z,var):
        if not self.h:return False
        idx=max((i for i,e in enumerate(self.h) if e.t<=mt),default=0)
        tmp=Filter();tmp.x=self.h[idx].state.copy();tmp.P=self.h[idx].P.copy();tmp.nis_pos=self.f.nis_pos;tmp.nis_yaw=self.f.nis_yaw;tmp.reject_pos=self.f.reject_pos;tmp.reject_yaw=self.f.reject_yaw
        ok=tmp.pos_update(z,var) if kind=='pos' else tmp.yaw_update(z,var)
        self.h[idx].state=tmp.x.copy();self.h[idx].P=tmp.P.copy()
        for k in range(idx+1,len(self.h)):
            e=self.h[k];tmp.predict(e.dt,e.v,e.wz);e.state=tmp.x.copy();e.P=tmp.P.copy()
        self.f=tmp;return ok

def errors(xs,rows,prefix='gt'):
    if not rows or not finite(rows[0].get(prefix+'_x','')): return None
    pe=[];ye=[]
    for x,r in zip(xs,rows):
        if not finite(r.get(prefix+'_x','')):continue
        pe.append(math.hypot(x[0]-float(r[prefix+'_x']),x[1]-float(r[prefix+'_y'])))
        if finite(r.get(prefix+'_yaw','')):ye.append(abs(wrap(x[2]-float(r[prefix+'_yaw']))))
    if not pe:return None
    return {'position_rmse_m':float(math.sqrt(np.mean(np.square(pe)))),'position_p95_m':float(np.percentile(pe,95)),
            'yaw_rmse_rad':float(math.sqrt(np.mean(np.square(ye)))) if ye else math.nan,'yaw_p95_rad':float(np.percentile(ye,95)) if ye else math.nan}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--csv',type=Path,required=True);ap.add_argument('--out',type=Path);ap.add_argument('--lag-sec',type=float,default=3.0);a=ap.parse_args()
    with a.csv.open(newline='',encoding='utf-8') as f:rows=list(csv.DictReader(f))
    if len(rows)<10:raise SystemExit('need >=10 samples')
    naive=Filter();rw=RewindFilter(a.lag_sec);prev=fval(rows[0],'t');nxs=[];rxs=[]
    for i,r in enumerate(rows):
        t=fval(r,'t');dt=max(0.0,t-prev) if i else 0.0;prev=t;v=fval(r,'wheel_v',0.0);w=fval(r,'imu_wz',0.0)
        naive.predict(dt,v,w);rw.predict(t,dt,v,w)
        gx,gy=fval(r,'gnss_x'),fval(r,'gnss_y')
        if math.isfinite(gx) and math.isfinite(gy):
            var=fval(r,'gnss_var',1.0);mt=fval(r,'gnss_measurement_t',t);naive.pos_update([gx,gy],var);rw.update(mt,'pos',[gx,gy],var)
        cy=fval(r,'cog_yaw')
        if math.isfinite(cy):
            var=fval(r,'cog_var',0.1);mt=fval(r,'cog_measurement_t',t);naive.yaw_update(cy,var);rw.update(mt,'yaw',cy,var)
        nxs.append(naive.x.copy());rxs.append(rw.f.x.copy())
    result={'samples':len(rows),'lag_sec':a.lag_sec,'naive':errors(nxs,rows),'rewind_bias_ekf':errors(rxs,rows),
            'naive_final_gyro_bias_rps':float(naive.x[3]),'rewind_final_gyro_bias_rps':float(rw.f.x[3]),
            'naive_pos_rejects':naive.reject_pos,'rewind_pos_rejects':rw.f.reject_pos,
            'naive_yaw_rejects':naive.reject_yaw,'rewind_yaw_rejects':rw.f.reject_yaw,
            'rewind_pos_nis_p95':float(np.percentile(rw.f.nis_pos,95)) if rw.f.nis_pos else math.nan,
            'rewind_yaw_nis_p95':float(np.percentile(rw.f.nis_yaw,95)) if rw.f.nis_yaw else math.nan}
    text=json.dumps(result,indent=2);print(text)
    if a.out:a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(text+'\n')
if __name__=='__main__':raise SystemExit(main())
