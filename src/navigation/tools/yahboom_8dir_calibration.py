#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,fcntl,json,math,os,statistics,subprocess,time
from datetime import datetime
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Bool,Float64,Float64MultiArray
G=9.80665

def wrap(a): return math.atan2(math.sin(a),math.cos(a))
def qdeg(a): return math.degrees(wrap(a))
def yaw_q(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def cmean(v): return math.atan2(sum(math.sin(x) for x in v),sum(math.cos(x) for x in v)) if v else float('nan')
def cstd(v):
    if len(v)<2:return float('nan')
    r=max(1e-12,min(1.0,math.hypot(sum(math.cos(x) for x in v)/len(v),sum(math.sin(x) for x in v)/len(v))))
    return math.sqrt(max(0,-2*math.log(r)))

def mounting_state(raw):
    if not raw or len(raw)<9:return ('WAIT',False,'raw sensor belum fresh')
    ax,ay,az=raw[:3]; an=math.sqrt(ax*ax+ay*ay+az*az)
    if not math.isfinite(an) or abs(an-G)>1.5:return ('MOVING_OR_TILTED',False,f'|a|={an:.2f} m/s2')
    if az>7.2 and abs(ax)<4.5 and abs(ay)<4.5:return ('TOP_UP',True,'horizontal/top-up; yaw mounting akan dipelajari dari 8 arah')
    if az<-7.2:return ('UPSIDE_DOWN',False,'sensor terbalik terhadap Z; pasang top-up atau ubah mounting config lalu ulang')
    if abs(ax)>7.2 or abs(ay)>7.2:return ('SIDE_MOUNTED',False,'sensor terpasang pada sisi; kalibrasi planar ditolak')
    return ('TILT_MISMATCH',False,f'raw accel=({ax:.2f},{ay:.2f},{az:.2f})')

class Cal(Node):
    def __init__(self,direction,root,fit_script):
        super().__init__('yahboom_8dir_calibration')
        self.direction=direction; self.sign=-1 if direction=='CW' else 1
        self.root=root; root.mkdir(parents=True,exist_ok=True); ts=datetime.now().strftime('%Y%m%d_%H%M%S')
        self.raw=root/f'heading_8dir_raw_{ts}.csv';self.seg=root/f'heading_8dir_segments_{ts}.csv';self.meta=root/f'heading_8dir_meta_{ts}.json';self.statefile=root/'yahboom_calibration_state.json';self.fit_script=fit_script
        self.f=self.raw.open('w',newline='',buffering=1);self.w=csv.writer(self.f)
        self.w.writerow(['wall_time','ros_ns','state','segment','imu_yaw_deg','inertial_yaw_deg','validated_yaw_deg','validated_ok','neo_yaw_deg','map_yaw_from_enu_deg','gnss_cog_yaw_deg','gyro_x_rps','gyro_y_rps','gyro_z_rps','gyro_norm_rps','acc_x','acc_y','acc_z','acc_norm','yah_mag_x_lsb','yah_mag_y_lsb','yah_mag_z_lsb','yah_mag_norm_lsb','neo_mag_x_ut','neo_mag_y_ut','neo_mag_z_ut','neo_mag_norm_ut'])
        self.latest={};self.validated=False;self.segment=0;self.mode='WAIT_STATIC';self.since=time.monotonic();self.samples=[];self.segments=[];self.checks=[];self.delta=0.;self.last_t=None;self.start_heading=float('nan');self.finished=False
        self.static_hold=1.2;self.capture=0.85;self.gin=.025;self.gout=.045;self.ain=.30;self.aout=.50
        self.create_subscription(Imu,'/imu/data',self.on_imu,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/imu/inertial_heading',lambda m:self.setv('inertial',yaw_q(m.pose.pose.orientation)),qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/heading/validated_fusion',lambda m:self.setv('validated',yaw_q(m.pose.pose.orientation)),qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/neo3/mag_heading_fusion',lambda m:self.setv('neo',yaw_q(m.pose.pose.orientation)),qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/gnss/cog_heading_fusion',lambda m:self.setv('cog',yaw_q(m.pose.pose.orientation)),qos_profile_sensor_data)
        self.create_subscription(Float64,'/localization/map_yaw_from_enu',lambda m:self.setv('mapyaw',float(m.data)),10)
        self.create_subscription(Bool,'/heading/validated',lambda m:setattr(self,'validated',bool(m.data)),10)
        self.create_subscription(Float64MultiArray,'/imu/mag_raw_lsb',self.mag,qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray,'/imu/raw_sensor_vectors',lambda m:self.setv('rawsensor',list(m.data)),qos_profile_sensor_data)
        self.create_subscription(MagneticField,'/neo3/mag',self.neo_mag,qos_profile_sensor_data)
        self.create_timer(.1,self.tick);self.write_state('RUNNING','Tahan robot diam untuk STATIC_1')
    def setv(self,k,v):self.latest[k]=(v,time.monotonic())
    def fresh(self,k,age=.5,default=float('nan')):
        x=self.latest.get(k);return x[0] if x and time.monotonic()-x[1]<=age else default
    def mag(self,m):
        if len(m.data)>=3:
            x,y,z=map(float,m.data[:3]);self.setv('ym',(x,y,z,math.sqrt(x*x+y*y+z*z)))
    def neo_mag(self,m):
        x=m.magnetic_field.x*1e6;y=m.magnetic_field.y*1e6;z=m.magnetic_field.z*1e6;self.setv('nm',(x,y,z,math.sqrt(x*x+y*y+z*z)))
    def stationary(self,gn,an):
        limg=self.gout if self.mode.startswith('STATIC') else self.gin;lima=self.aout if self.mode.startswith('STATIC') else self.ain
        return gn<=limg and abs(an-G)<=lima
    def on_imu(self,m):
        now=time.monotonic();gx=m.angular_velocity.x;gy=m.angular_velocity.y;gz=m.angular_velocity.z;gn=math.sqrt(gx*gx+gy*gy+gz*gz);ax=m.linear_acceleration.x;ay=m.linear_acceleration.y;az=m.linear_acceleration.z;an=math.sqrt(ax*ax+ay*ay+az*az);st=self.stationary(gn,an)
        imu=yaw_q(m.orientation);self.setv('imu',imu);self.latest['kin']=((gx,gy,gz,gn,ax,ay,az,an,st),now)
        inert=self.fresh('inertial');val=self.fresh('validated');neo=self.fresh('neo');my=self.fresh('mapyaw');cog=self.fresh('cog');ym=self.fresh('ym',default=(float('nan'),)*4);nm=self.fresh('nm',default=(float('nan'),)*4)
        seg=self.segment if self.mode==f'STATIC_{self.segment}' else 0;rosns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        self.w.writerow([time.time(),rosns,self.mode,seg,qdeg(imu),qdeg(inert),qdeg(val),int(self.validated),qdeg(neo),qdeg(my),qdeg(cog),gx,gy,gz,gn,ax,ay,az,an,*ym,*nm])
        if self.mode=='TRANSITION':
            if self.last_t is not None:self.delta+=gz*max(0,min(.1,now-self.last_t))
            self.last_t=now
        if self.mode==f'STATIC_{self.segment}' and seg>0 and st:self.samples.append({'t':now,'imu':imu,'inertial':inert,'validated':val,'neo':neo,'mapyaw':my,'gz':gz,'gn':gn,'an':an,'ym':ym,'nm':nm})
    def target_deg(self,next_seg=None):
        s=next_seg or max(1,self.segment)
        if not math.isfinite(self.start_heading):return float('nan')
        return qdeg(self.start_heading+self.sign*math.radians(45)*(s-1))
    def write_state(self,status=None,instruction=''):
        raw_entry=self.latest.get('rawsensor');raw=raw_entry[0] if raw_entry else None;ms,mok,mdetail=mounting_state(raw)
        kin=self.fresh('kin',default=None);neo=self.fresh('neo');actual_rel=float('nan')
        if math.isfinite(self.start_heading) and math.isfinite(neo):actual_rel=qdeg(neo-self.start_heading)
        nextseg=min(8,self.segment+1 if self.mode in ('WAIT_MOVE','TRANSITION','STATIC_CANDIDATE') else max(1,self.segment))
        d={'status':status or ('COMPLETE' if self.finished else 'RUNNING'),'direction':self.direction,'state':self.mode,'segment':self.segment,'next_segment':nextseg,'progress':self.segment/8.0,'instruction':instruction,'start_heading_deg':qdeg(self.start_heading) if math.isfinite(self.start_heading) else None,'target_heading_deg':self.target_deg(nextseg) if math.isfinite(self.start_heading) else None,'target_relative_deg':self.sign*45*(nextseg-1),'actual_relative_deg':actual_rel if math.isfinite(actual_rel) else None,'neo_heading_deg':qdeg(neo) if math.isfinite(neo) else None,'gyro_z_rps':kin[2] if kin else None,'acc_norm_mps2':kin[7] if kin else None,'static':bool(kin[-1]) if kin else False,'mounting_state':ms,'mounting_ok':mok,'mounting_detail':mdetail,'raw_sensor_vectors':raw,'transition_checks':self.checks,'raw_csv':str(self.raw),'segments_csv':str(self.seg),'meta_json':str(self.meta),'updated_at':datetime.now().astimezone().isoformat(timespec='milliseconds')}
        tmp=self.statefile.with_suffix('.tmp');tmp.write_text(json.dumps(d,indent=2));os.replace(tmp,self.statefile)
    def candidate(self):
        if self.mode=='TRANSITION' and self.segment>0:
            deg=math.degrees(self.delta)
            if abs(deg)<5.0:
                self.mode='WAIT_MOVE';self.since=time.monotonic();self.write_state('RUNNING','Gerakan <5 derajat diabaikan; putar sekitar 45 derajat '+self.direction);return
            ok=self.sign*deg>5;self.checks.append({'from_segment':self.segment,'gyro_delta_deg':deg,'direction_ok':ok})
            if not ok:
                self.mode='DIRECTION_MISMATCH';self.write_state('DIRECTION_MISMATCH',f'Putaran {deg:+.1f} deg berlawanan dengan pilihan {self.direction}; Stop lalu ulang');return
        self.mode='STATIC_CANDIDATE';self.since=time.monotonic()
    def begin(self):
        raw=self.fresh('rawsensor',default=None);_,mok,_=mounting_state(raw)
        if not mok:self.mode='WAIT_STATIC';self.since=time.monotonic();self.write_state('MOUNTING_MISMATCH','Perbaiki mounting IMU: Z harus top-up dan robot diam');return
        self.segment+=1;self.mode=f'STATIC_{self.segment}';self.since=time.monotonic();self.samples=[]
        if self.segment==1:
            neo=self.fresh('neo');self.start_heading=neo if math.isfinite(neo) else self.fresh('inertial')
        self.write_state('RUNNING',f'Tahan diam posisi {self.segment}/8')
    def finish_segment(self):
        if not self.samples:return
        def arr(k):return [x[k] for x in self.samples if isinstance(x[k],(int,float)) and math.isfinite(x[k])]
        vals={k:arr(k) for k in ('imu','inertial','validated','neo','mapyaw','gz','gn','an')};ym=[x['ym'] for x in self.samples if all(math.isfinite(v) for v in x['ym'])]
        d={'segment':self.segment,'samples':len(self.samples),'duration_sec':self.samples[-1]['t']-self.samples[0]['t'],'imu_mean_deg':qdeg(cmean(vals['imu'])),'inertial_mean_deg':qdeg(cmean(vals['inertial'])),'neo_mean_deg':qdeg(cmean(vals['neo'])),'gyro_z_mean_rps':statistics.fmean(vals['gz']) if vals['gz'] else float('nan'),'acc_norm_mean':statistics.fmean(vals['an']) if vals['an'] else float('nan'),'yah_mag_x_mean_lsb':statistics.fmean(v[0] for v in ym),'yah_mag_y_mean_lsb':statistics.fmean(v[1] for v in ym),'yah_mag_z_mean_lsb':statistics.fmean(v[2] for v in ym)};self.segments.append(d);self.summary()
    def summary(self):
        with self.seg.open('w',newline='') as f:
            cols=list(self.segments[0].keys()) if self.segments else ['segment'];w=csv.DictWriter(f,fieldnames=cols);w.writeheader();w.writerows(self.segments)
        meta={'status':'COMPLETE' if self.finished else 'RUNNING','expected_rotation':self.direction,'transition_checks':self.checks,'segments':self.segments,'raw_csv':str(self.raw),'segments_csv':str(self.seg),'updated_at':datetime.now().astimezone().isoformat()};tmp=self.meta.with_suffix('.tmp');tmp.write_text(json.dumps(meta,indent=2));os.replace(tmp,self.meta)
    def run_fit(self):
        r=subprocess.run(['/usr/bin/python3',str(self.fit_script),str(self.raw),'--meta-json',str(self.meta),'--direction',self.direction],capture_output=True,text=True,timeout=25)
        fit=self.root/'yahboom_mag_planar_latest.yaml';detail=(r.stdout or r.stderr).strip()[-600:];self.write_state('PASS' if r.returncode==0 else 'FAIL',detail);return r.returncode==0
    def tick(self):
        if self.finished:return
        kin=self.fresh('kin',age=.4,default=None)
        if not kin:self.write_state('WAIT_SENSOR','Menunggu /imu/data');return
        st=kin[-1];now=time.monotonic()
        if self.mode in ('WAIT_STATIC','TRANSITION'):
            if st:self.candidate()
        elif self.mode=='WAIT_MOVE':
            if not st:self.mode='TRANSITION';self.since=now;self.delta=0;self.last_t=now
        elif self.mode=='STATIC_CANDIDATE':
            if not st:self.mode='TRANSITION';self.since=now
            elif now-self.since>=self.static_hold:self.begin()
        elif self.mode==f'STATIC_{self.segment}':
            if now-self.since>=self.capture:
                self.finish_segment()
                if self.segment>=8:
                    self.finished=True;self.mode='COMPLETE';self.summary();self.run_fit();rclpy.shutdown();return
                self.mode='WAIT_MOVE';self.since=now
        instr=(f'Putar {self.direction} menuju posisi {min(8,self.segment+1)} lalu tahan diam' if self.mode in ('WAIT_MOVE','TRANSITION') else f'Tahan diam posisi {max(1,self.segment)}')
        self.write_state('RUNNING',instr)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--direction',choices=['CW','CCW'],required=True);ap.add_argument('--root',default='/home/otomasi/ros/calibration');ap.add_argument('--fit-script',default='');a=ap.parse_args()
    lock=open('/tmp/agv_yahboom_8dir_calibration.lock','w');
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('another Yahboom calibration is already running')
    fit=Path(a.fit_script) if a.fit_script else Path(__file__).resolve().with_name('yahboom_mag_planar_fit.py')
    rclpy.init();n=Cal(a.direction,Path(a.root),fit)
    try:rclpy.spin(n)
    except KeyboardInterrupt:n.write_state('STOPPED','Calibration dihentikan operator')
    finally:
        try:n.summary();n.f.close();n.destroy_node()
        except Exception:pass
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
