#!/usr/bin/env python3
import csv, json, math, os, signal, statistics, subprocess, sys, time
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float64, Float64MultiArray

G=9.80665
AGV_ROOT=Path(os.environ.get('AGV_ROOT', str(Path.home()/'agv'))).expanduser().resolve()
def portable_path(path):
    resolved=Path(path).expanduser().resolve()
    try:return str(resolved.relative_to(AGV_ROOT))
    except ValueError:return str(resolved)

def yaw_from_q(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def circ_mean(vals):
    if not vals: return float('nan')
    return math.atan2(sum(math.sin(v) for v in vals), sum(math.cos(v) for v in vals))

def circ_std(vals):
    if len(vals)<2: return float('nan')
    c=sum(math.cos(v) for v in vals)/len(vals); s=sum(math.sin(v) for v in vals)/len(vals)
    R=max(1e-12, min(1.0, math.hypot(c,s)))
    return math.sqrt(max(0.0,-2.0*math.log(R)))

def qdeg(rad): return math.degrees(wrap(rad))

class Logger(Node):
    def __init__(self):
        super().__init__('heading_8dir_calibration_logger')
        ts=datetime.now().strftime('%Y%m%d_%H%M%S')
        root=Path(os.environ.get('AGV_ROOT', str(Path.home()/'agv'))) / 'calibration'
        root.mkdir(parents=True, exist_ok=True)
        self.raw_path=root/f'heading_8dir_raw_{ts}.csv'
        self.seg_path=root/f'heading_8dir_segments_{ts}.csv'
        self.meta_path=root/f'heading_8dir_meta_{ts}.json'
        self.done_path=root/'heading_8dir_latest.json'
        self.f=self.raw_path.open('w',newline='',buffering=1)
        self.w=csv.writer(self.f)
        self.w.writerow(['wall_time','ros_ns','state','segment','imu_yaw_deg','inertial_yaw_deg','validated_yaw_deg','validated_ok','neo_yaw_deg','map_yaw_from_enu_deg','gnss_cog_yaw_deg','gyro_x_rps','gyro_y_rps','gyro_z_rps','gyro_norm_rps','acc_x','acc_y','acc_z','acc_norm','yah_mag_x_lsb','yah_mag_y_lsb','yah_mag_z_lsb','yah_mag_norm_lsb','neo_mag_x_ut','neo_mag_y_ut','neo_mag_z_ut','neo_mag_norm_ut'])
        self.latest={}; self.validated=False; self.segment=0; self.state='WAIT_STATIC'; self.state_since=time.monotonic(); self.samples=[]; self.segments=[]
        self.expected_rotation='CW'; self.transition_gyro_delta=0.0; self.transition_last_t=None; self.transition_checks=[]
        self.static_hold=1.20; self.min_segment_collect=0.80; self.max_segments=8
        self.capture_target_sec=0.85
        self.gyro_enter=0.025; self.gyro_exit=0.045; self.acc_tol_enter=0.30; self.acc_tol_exit=0.50
        self.last_row_t=0.0; self.last_print=0.0; self.finished=False
        self.create_subscription(Imu,'/imu/data',self.on_imu,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/imu/inertial_heading',lambda m:self.setv('inertial',yaw_from_q(m.pose.pose.orientation)),qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/heading/validated_fusion',lambda m:self.setv('validated_yaw',yaw_from_q(m.pose.pose.orientation)),qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/neo3/mag_heading_fusion',lambda m:self.setv('neo_yaw',yaw_from_q(m.pose.pose.orientation)),qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/gnss/cog_heading_fusion',lambda m:self.setv('gnss_cog',yaw_from_q(m.pose.pose.orientation)),qos_profile_sensor_data)
        self.create_subscription(Float64,'/localization/map_yaw_from_enu',lambda m:self.setv('map_yaw',float(m.data)),10)
        self.create_subscription(Bool,'/heading/validated',lambda m:setattr(self,'validated',bool(m.data)),10)
        self.create_subscription(Float64MultiArray,'/imu/mag_raw_lsb',self.on_yah_raw_mag,qos_profile_sensor_data)
        self.create_subscription(MagneticField,'/neo3/mag',lambda m:self.setmag('neo',m),qos_profile_sensor_data)
        self.create_timer(0.1,self.tick)
        print(f'8DIR_READY raw={self.raw_path} segments={self.seg_path}',flush=True)
        print('Urutan otomatis CW: STATIC_1 -> putar CW -> STATIC_2 ... sampai STATIC_8. REP-103 mengharapkan gyro_z NEGATIF saat CW.',flush=True)
    def setv(self,k,v): self.latest[k]=(v,time.monotonic())
    def on_yah_raw_mag(self,m):
        if len(m.data) < 3: return
        x=float(m.data[0]); y=float(m.data[1]); z=float(m.data[2])
        self.latest['yahmag']=((x,y,z,math.sqrt(x*x+y*y+z*z)),time.monotonic())
    def setmag(self,p,m):
        x=m.magnetic_field.x*1e6; y=m.magnetic_field.y*1e6; z=m.magnetic_field.z*1e6
        self.latest[p+'mag']=((x,y,z,math.sqrt(x*x+y*y+z*z)),time.monotonic())
    def fresh(self,k,age=.35,default=float('nan')):
        v=self.latest.get(k)
        return v[0] if v and time.monotonic()-v[1]<=age else default
    def stationary(self,gn,an):
        if self.state.startswith('STATIC') or self.state=='STATIC_CANDIDATE': return gn<=self.gyro_exit and abs(an-G)<=self.acc_tol_exit
        return gn<=self.gyro_enter and abs(an-G)<=self.acc_tol_enter
    def on_imu(self,m):
        now=time.monotonic(); gx=m.angular_velocity.x; gy=m.angular_velocity.y; gz=m.angular_velocity.z; gn=math.sqrt(gx*gx+gy*gy+gz*gz)
        ax=m.linear_acceleration.x; ay=m.linear_acceleration.y; az=m.linear_acceleration.z; an=math.sqrt(ax*ax+ay*ay+az*az)
        imu=yaw_from_q(m.orientation); st=self.stationary(gn,an)
        self.latest['imu']=(imu,now); self.latest['kin']=((gx,gy,gz,gn,ax,ay,az,an,st),now)
        inert=self.fresh('inertial'); val=self.fresh('validated_yaw'); neo=self.fresh('neo_yaw'); map_yaw=self.fresh('map_yaw'); cog=self.fresh('gnss_cog'); ym=self.fresh('yahmag',default=(float('nan'),)*4); nm=self.fresh('neomag',default=(float('nan'),)*4)
        seg=self.segment if self.state.startswith('STATIC') else 0
        ros_ns=int(m.header.stamp.sec)*1000000000+int(m.header.stamp.nanosec)
        row=[time.time(),ros_ns,self.state,seg,qdeg(imu),qdeg(inert),qdeg(val),int(self.validated),qdeg(neo),qdeg(map_yaw),qdeg(cog),gx,gy,gz,gn,ax,ay,az,an,*ym,*nm]
        self.w.writerow(row)
        if self.state=='TRANSITION':
            if self.transition_last_t is not None:
                dt=max(0.0,min(0.1,now-self.transition_last_t)); self.transition_gyro_delta += gz*dt
            self.transition_last_t=now
        if self.state.startswith('STATIC') and seg>0 and st:
            self.samples.append({'t':now,'imu':imu,'inertial':inert,'validated':val,'neo':neo,'map_yaw':map_yaw,'gz':gz,'gn':gn,'an':an,'ym':ym,'nm':nm})
    def start_candidate(self):
        if self.state=='TRANSITION' and self.segment>0:
            d=math.degrees(self.transition_gyro_delta); ok=d < -5.0
            self.transition_checks.append({'from_segment':self.segment,'gyro_delta_deg':d,'cw_ok':ok})
            print(f'CW CHECK transition {self.segment}->{self.segment+1}: gyro integral={d:+.2f} deg => {"OK" if ok else "SALAH ARAH/TERLALU KECIL"}',flush=True)
        self.state='STATIC_CANDIDATE'; self.state_since=time.monotonic(); print('STATIC candidate...',flush=True)
    def begin_segment(self):
        self.segment+=1; self.state=f'STATIC_{self.segment}'; self.state_since=time.monotonic(); self.samples=[]
        print(f'>>> STATIC_{self.segment} CAPTURE mulai — tahan diam 2–3 detik',flush=True)
    def finish_segment(self):
        if not self.samples: return
        s=self.samples
        def arr(k): return [x[k] for x in s if isinstance(x[k],(int,float)) and math.isfinite(x[k])]
        vals={k:arr(k) for k in ('imu','inertial','validated','neo','map_yaw','gz','gn','an')}
        ym=[x['ym'] for x in s if all(math.isfinite(v) for v in x['ym'])]; nm=[x['nm'] for x in s if all(math.isfinite(v) for v in x['nm'])]
        d={'segment':self.segment,'samples':len(s),'duration_sec':s[-1]['t']-s[0]['t'],'imu_mean_deg':qdeg(circ_mean(vals['imu'])),'imu_std_deg':math.degrees(circ_std(vals['imu'])),'inertial_mean_deg':qdeg(circ_mean(vals['inertial'])),'inertial_std_deg':math.degrees(circ_std(vals['inertial'])),'validated_mean_deg':qdeg(circ_mean(vals['validated'])),'validated_std_deg':math.degrees(circ_std(vals['validated'])),'neo_mean_deg':qdeg(circ_mean(vals['neo'])),'neo_std_deg':math.degrees(circ_std(vals['neo'])),'inertial_neo_residual_deg':qdeg(wrap(circ_mean(vals['inertial'])-circ_mean(vals['neo']))),'gyro_z_mean_rps':statistics.fmean(vals['gz']) if vals['gz'] else float('nan'),'gyro_norm_mean_rps':statistics.fmean(vals['gn']) if vals['gn'] else float('nan'),'acc_norm_mean':statistics.fmean(vals['an']) if vals['an'] else float('nan'),'yah_mag_x_mean_lsb':statistics.fmean([v[0] for v in ym]) if ym else float('nan'),'yah_mag_y_mean_lsb':statistics.fmean([v[1] for v in ym]) if ym else float('nan'),'yah_mag_z_mean_lsb':statistics.fmean([v[2] for v in ym]) if ym else float('nan'),'yah_mag_norm_mean_lsb':statistics.fmean([v[3] for v in ym]) if ym else float('nan'),'map_yaw_from_enu_mean_deg':qdeg(circ_mean(vals['map_yaw'])),'neo_mag_norm_mean_ut':statistics.fmean([v[3] for v in nm]) if nm else float('nan')}
        self.segments.append(d); print('<<< STATIC_%d selesai: n=%d dur=%.2fs inertial=%.2f° neo=%.2f° residual=%.2f°'%(self.segment,d['samples'],d['duration_sec'],d['inertial_mean_deg'],d['neo_mean_deg'],d['inertial_neo_residual_deg']),flush=True)
        self.write_summary()
    def write_summary(self):
        cols=['segment','samples','duration_sec','imu_mean_deg','imu_std_deg','inertial_mean_deg','inertial_std_deg','validated_mean_deg','validated_std_deg','neo_mean_deg','neo_std_deg','inertial_neo_residual_deg','gyro_z_mean_rps','gyro_norm_mean_rps','acc_norm_mean','yah_mag_x_mean_lsb','yah_mag_y_mean_lsb','yah_mag_z_mean_lsb','yah_mag_norm_mean_lsb','map_yaw_from_enu_mean_deg','neo_mag_norm_mean_ut']
        with self.seg_path.open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=cols); w.writeheader(); w.writerows(self.segments)
        meta={'status':'COMPLETE' if len(self.segments)>=8 else 'RUNNING','expected_rotation':self.expected_rotation,'transition_checks':self.transition_checks,'segments':self.segments,'raw_csv':portable_path(self.raw_path),'segments_csv':portable_path(self.seg_path),'updated_at':datetime.now().isoformat(),'thresholds':{'static_hold_sec':self.static_hold,'gyro_enter_rps':self.gyro_enter,'gyro_exit_rps':self.gyro_exit,'acc_enter_tol_mps2':self.acc_tol_enter,'acc_exit_tol_mps2':self.acc_tol_exit,'capture_target_sec':self.capture_target_sec}}
        self.meta_path.write_text(json.dumps(meta,indent=2)); self.done_path.write_text(json.dumps(meta,indent=2))
    def run_yahboom_fit(self):
        cmd=[os.environ.get('AGV_PYTHON','/usr/bin/python3'), str(Path(os.environ.get('AGV_ROOT', str(Path.home()/'agv'))) / 'tools/yahboom_mag_planar_fit.py'), str(self.raw_path), '--meta-json', str(self.meta_path)]
        try:
            r=subprocess.run(cmd,text=True,capture_output=True,timeout=20)
            if r.stdout.strip(): print(r.stdout.strip(),flush=True)
            if r.returncode != 0:
                print(f'YAHBOOM_PLANAR_FIT belum lulus (rc={r.returncode}); runtime tidak diaktifkan otomatis.',flush=True)
                if r.stderr.strip(): print(r.stderr.strip(),flush=True)
        except Exception as e:
            print(f'YAHBOOM_PLANAR_FIT ERROR: {e}',flush=True)

    def tick(self):
        if self.finished: return
        kin=self.fresh('kin',age=.40,default=None); now=time.monotonic()
        if kin is None: return
        st=kin[-1]
        if self.state in ('WAIT_STATIC','TRANSITION'):
            if st: self.start_candidate()
        elif self.state=='WAIT_MOVE':
            if not st:
                self.state='TRANSITION'; self.state_since=now; self.transition_gyro_delta=0.0; self.transition_last_t=now
                print(f'--- TRANSITION CW menuju arah {self.segment+1} ---',flush=True)
        elif self.state=='STATIC_CANDIDATE':
            if not st:
                self.state='TRANSITION'; self.state_since=now
            elif now-self.state_since>=self.static_hold:
                self.begin_segment()
        elif self.state.startswith('STATIC'):
            dur=now-self.state_since
            if not st and dur<self.min_segment_collect:
                print(f'!!! STATIC_{self.segment} terlalu singkat ({dur:.2f}s), diulang',flush=True)
                self.segment-=1; self.samples=[]; self.state='TRANSITION'; self.state_since=now
            elif dur>=self.capture_target_sec or (not st and dur>=self.min_segment_collect):
                self.finish_segment()
                if self.segment>=self.max_segments:
                    self.finished=True; self.state='COMPLETE'; self.write_summary(); print('=== 8 ARAH COMPLETE ===',flush=True); self.run_yahboom_fit(); rclpy.shutdown(); return
                self.state='WAIT_MOVE'; self.state_since=now
                print(f'*** STATIC_{self.segment} terkunci. Silakan putar menuju arah {self.segment+1}. ***',flush=True)
        if now-self.last_print>2.0:
            self.last_print=now; inert=self.fresh('inertial'); neo=self.fresh('neo_yaw')
            print(f'STATUS state={self.state} seg={self.segment}/8 static={st} inertial={qdeg(inert):.2f}° neo={qdeg(neo):.2f}° valid={self.validated}',flush=True)

def main():
    rclpy.init(); n=Logger()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        try: n.write_summary(); n.f.close(); n.destroy_node()
        except Exception: pass
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
