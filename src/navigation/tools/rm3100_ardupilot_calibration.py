#!/usr/bin/env python3
import argparse, csv, json, math, signal, sys, time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField

FACES = ('LEVEL','LEFT_SIDE','RIGHT_SIDE','NOSE_UP','NOSE_DOWN','INVERTED')

def quat_rp(q):
    sinr=2*(q.w*q.x+q.y*q.z); cosr=1-2*(q.x*q.x+q.y*q.y)
    roll=math.atan2(sinr,cosr)
    sinp=2*(q.w*q.y-q.z*q.x)
    pitch=math.copysign(math.pi/2,sinp) if abs(sinp)>=1 else math.asin(sinp)
    return math.degrees(roll),math.degrees(pitch)

def classify_face(roll,pitch):
    if not math.isfinite(roll) or not math.isfinite(pitch): return 'UNKNOWN'
    if abs(roll)>145: return 'INVERTED'
    if 55<roll<125 and abs(pitch)<50: return 'LEFT_SIDE'
    if -125<roll<-55 and abs(pitch)<50: return 'RIGHT_SIDE'
    if 55<pitch<125 and abs(roll)<55: return 'NOSE_UP'
    if -125<pitch<-55 and abs(roll)<55: return 'NOSE_DOWN'
    if abs(roll)<35 and abs(pitch)<35: return 'LEVEL'
    return 'TRANSITION'

def atomic_json(path,obj):
    tmp=path.with_suffix(path.suffix+'.tmp'); tmp.write_text(json.dumps(obj,indent=2)); tmp.replace(path)

def sphere_coverage(points):
    if len(points)<10: return 0.0,0
    centered=points-np.mean(points,axis=0); n=np.linalg.norm(centered,axis=1); ok=n>1e-9
    u=centered[ok]/n[ok,None]
    az=np.mod(np.arctan2(u[:,1],u[:,0]),2*np.pi); z=np.clip(u[:,2],-1,1)
    ai=np.minimum(11,(az/(2*np.pi)*12).astype(int)); zi=np.minimum(5,((z+1)*3).astype(int))
    bins=set(zip(ai.tolist(),zi.tolist())); return 100.0*len(bins)/72.0,len(bins)

def fit_ellipsoid(points):
    x,y,z=points.T
    D=np.column_stack((x*x,y*y,z*z,2*x*y,2*x*z,2*y*z,x,y,z))
    p,*_=np.linalg.lstsq(D,np.ones(len(points)),rcond=None)
    A=np.array([[p[0],p[3],p[4]],[p[3],p[1],p[5]],[p[4],p[5],p[2]]],float)
    b=np.array(p[6:9],float); center=-0.5*np.linalg.solve(A,b)
    k=1.0+center@A@center; shape=A/k
    vals,vecs=np.linalg.eigh(shape)
    if not np.all(np.isfinite(vals)) or np.min(vals)<=1e-12: raise ValueError('ellipsoid bukan positive-definite')
    root=vecs@np.diag(np.sqrt(vals))@vecs.T
    radius=np.median(np.linalg.norm(points-center,axis=1)); M=radius*root
    corrected=(M@(points-center).T).T; norms=np.linalg.norm(corrected,axis=1)
    return center,M,corrected,norms

def result_yaml(path,result):
    import yaml
    tmp=path.with_suffix(path.suffix+'.tmp'); tmp.write_text(yaml.safe_dump(result,sort_keys=False)); tmp.replace(path)

class Recorder(Node):
    def __init__(self,args):
        super().__init__('rm3100_ardupilot_calibration')
        self.args=args; self.rows=[]; self.roll=float('nan'); self.pitch=float('nan')
        self.face_counts={k:0 for k in FACES}; self.last=None; self.started=time.time(); self.last_state=0
        self.target_face='LEVEL'; self.observed_face='UNKNOWN'; self.control_mtime=0.0
        self.create_subscription(Imu,args.imu_topic,self.on_imu,qos_profile_sensor_data)
        self.create_subscription(MagneticField,args.topic,self.on_mag,qos_profile_sensor_data)
    def on_imu(self,msg):
        self.roll,self.pitch=quat_rp(msg.orientation)
    def refresh_control(self):
        try:
            mt=self.args.control_path.stat().st_mtime
            if mt==self.control_mtime:return
            self.control_mtime=mt; obj=json.loads(self.args.control_path.read_text())
            face=str(obj.get('target_face','')).upper()
            if face in FACES:self.target_face=face
        except Exception:
            pass
    def on_mag(self,msg):
        self.refresh_control()
        xyz=np.array([msg.magnetic_field.x,msg.magnetic_field.y,msg.magnetic_field.z],float)*1e6
        finite=bool(np.all(np.isfinite(xyz)))
        norm=float(np.linalg.norm(xyz)) if finite else float('nan')
        accepted=finite and 5.0<=norm<=150.0
        reason='OK' if accepted else ('NONFINITE' if not finite else 'FIELD_RANGE')
        observed=classify_face(self.roll,self.pitch); face=self.target_face; now=time.time()
        self.observed_face=observed
        self.rows.append((now,*xyz,norm,self.roll,self.pitch,face,observed,accepted,reason))
        if accepted and face in self.face_counts:self.face_counts[face]+=1
        self.last=(xyz,norm,face,observed,accepted,reason); self.write_state(False)
    def write_state(self,force=True,status='RUNNING',instruction='Putar perlahan sesuai orientasi wizard.'):
        now=time.time()
        if not force and now-self.last_state<0.20:return
        self.last_state=now; pts=np.array([r[1:4] for r in self.rows if r[9]],float) if any(r[9] for r in self.rows) else np.empty((0,3))
        cov,bins=sphere_coverage(pts)
        raw={}
        if self.last is not None:
            v,n,f,obs,accepted,reason=self.last; raw={'x_ut':float(v[0]) if np.isfinite(v[0]) else None,'y_ut':float(v[1]) if np.isfinite(v[1]) else None,'z_ut':float(v[2]) if np.isfinite(v[2]) else None,'norm_ut':float(n) if np.isfinite(n) else None,'face':f,'observed_face':obs,'accepted_for_fit':bool(accepted),'fit_reason':reason}
        atomic_json(self.args.state_path,{
          'status':status,'running':status=='RUNNING','sample_count':len(self.rows),'fit_sample_count':len(pts),'rejected_fit_count':len(self.rows)-len(pts),'elapsed_sec':round(now-self.started,2),
          'coverage_pct':round(cov,1),'coverage_bins':bins,'face_counts':self.face_counts,'target_face':self.target_face,'observed_face':self.observed_face,'roll_deg':self.roll,'pitch_deg':self.pitch,
          'raw_wire':raw,'instruction':instruction,'source_topic':self.args.topic,
          'raw_semantics':'DroneCAN/AP_Periph field before ROS-host correction; not guaranteed native RM3100 register raw',
          'csv_path':str(self.args.csv_path),'fit_path':str(self.args.fit_path)})

    def save_csv(self):
        with self.args.csv_path.open('w',newline='') as f:
            w=csv.writer(f); w.writerow(['wall_time_s','x_ut','y_ut','z_ut','norm_ut','roll_deg','pitch_deg','target_face','observed_face','accepted_for_fit','fit_reason'])
            w.writerows(self.rows)

    def finalize(self):
        self.save_csv(); pts=np.array([r[1:4] for r in self.rows if r[9]],float) if any(r[9] for r in self.rows) else np.empty((0,3))
        coverage,bins=sphere_coverage(pts)
        face_ok=sum(1 for k in FACES if self.face_counts[k]>=30)
        if len(pts)<360 or coverage<45 or face_ok<6:
            why=f'kriteria fit belum cukup: accepted={len(pts)}/360 dari raw={len(self.rows)}, sphere={coverage:.1f}%/45%, faces={face_ok}/6 (min 30 sample/fase)'
            self.write_state(True,'FAILED',why); return False,why
        norms=np.linalg.norm(pts,axis=1); lo,hi=np.percentile(norms,[0.5,99.5]); fitpts=pts[(norms>=lo)&(norms<=hi)]
        try: center,M,corr,cn=fit_ellipsoid(fitpts)
        except Exception as e:
            why='fit gagal: '+str(e); self.write_state(True,'FAILED',why); return False,why
        cv=float(np.std(cn)/max(np.mean(cn),1e-9)); cond=float(np.linalg.cond(M)); passed=cv<0.06 and cond<6.0
        ofs_mg=(-center*10.0).tolist()
        result={'rm3100_ardupilot_calibration':{
          'valid':bool(passed),'created_at':datetime.now().astimezone().isoformat(timespec='seconds'),
          'source_topic':self.args.topic,'source_semantics':'DroneCAN/AP_Periph pre-ROS correction field',
          'sample_count':len(pts),'fit_sample_count':len(fitpts),'coverage_pct':float(coverage),'coverage_bins':bins,
          'face_counts':self.face_counts,'host_ros':{'bias_xyz_ut':center.tolist(),'matrix_3x3':M.reshape(-1).tolist()},
          'ardupilot_equivalent':{'COMPASS_OFS_X_mG':ofs_mg[0],'COMPASS_OFS_Y_mG':ofs_mg[1],'COMPASS_OFS_Z_mG':ofs_mg[2],
            'COMPASS_DIA_X':float(M[0,0]),'COMPASS_DIA_Y':float(M[1,1]),'COMPASS_DIA_Z':float(M[2,2]),
            'COMPASS_ODI_X':float(M[0,1]),'COMPASS_ODI_Y':float(M[0,2]),'COMPASS_ODI_Z':float(M[1,2])},
          'validation':{'corrected_norm_mean_ut':float(np.mean(cn)),'corrected_norm_std_ut':float(np.std(cn)),
            'corrected_norm_cv':cv,'corrected_norm_p05_ut':float(np.percentile(cn,5)),'corrected_norm_p95_ut':float(np.percentile(cn,95)),
            'matrix_condition':cond,'pass':bool(passed)},
          'warning':'AP_Periph parameter readback must be checked before applying host calibration; never enable AP_Periph and ROS hard/soft-iron correction together.'}}
        result_yaml(self.args.fit_path,result)
        why='PASS full 3D fit; stage ke ROS hanya setelah memastikan AP_Periph tidak sudah mengoreksi compass.' if passed else f'fit belum memenuhi kriteria: CV={cv:.4f}, cond={cond:.2f}'
        self.write_state(True,'PASS' if passed else 'FAILED',why); return passed,why

def main():
    p=argparse.ArgumentParser(description='RM3100 Mission-Planner-style 3D host calibration recorder')
    p.add_argument('--workspace',default=str(Path(__file__).resolve().parents[3])); p.add_argument('--topic',default='/neo3pro/mag')
    p.add_argument('--imu-topic',default='/imu/data'); a=p.parse_args(); root=Path(a.workspace).resolve()
    cal=root/'calibration'; cal.mkdir(parents=True,exist_ok=True); stamp=datetime.now().strftime('%Y%m%d_%H%M%S')
    a.state_path=cal/'rm3100_calibration_state.json'; a.control_path=cal/'rm3100_calibration_control.json'; a.csv_path=cal/f'rm3100_3d_raw_{stamp}.csv'; a.fit_path=cal/'rm3100_ardupilot_latest.yaml'
    atomic_json(a.control_path,{'target_face':'LEVEL','updated_at':datetime.now().astimezone().isoformat(timespec='seconds')})
    rclpy.init(); node=Recorder(a); stopping={'v':False}
    def stop_handler(*_): stopping['v']=True
    signal.signal(signal.SIGTERM,stop_handler); signal.signal(signal.SIGINT,stop_handler)
    node.write_state(True,'RUNNING','Mulai LEVEL. Putar kendaraan/sensor perlahan 360°, lalu ikuti enam orientasi wizard.')
    try:
        while rclpy.ok() and not stopping['v']:
            rclpy.spin_once(node,timeout_sec=0.1)
    finally:
        ok,msg=node.finalize(); print(json.dumps({'ok':ok,'message':msg,'csv_path':str(a.csv_path),'fit_path':str(a.fit_path)}),flush=True)
        node.destroy_node(); rclpy.shutdown()
    return 0 if ok else 2

if __name__=='__main__': sys.exit(main())
