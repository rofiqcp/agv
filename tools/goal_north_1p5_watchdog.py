#!/usr/bin/env python3
import math, time, json, urllib.request
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, String

def yaw_deg(q):
    return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

def post(path,obj=None):
    data=json.dumps(obj or {}).encode()
    req=urllib.request.Request('http://127.0.0.1:5000'+path,data=data,headers={'Content-Type':'application/json'},method='POST')
    with urllib.request.urlopen(req,timeout=2) as r: return r.read().decode()
class Watch(Node):
    def __init__(self):
        super().__init__('goal_north_1p5_watchdog')
        state=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose=None; self.local=None; self.gnss=None
        self.auto=False; self.ready=False; self.feedback=False; self.estop=True
        self.actual=0.0; self.target=0.0; self.steer=0.0; self.goal=''; self.source=''
        self.create_subscription(PoseWithCovarianceStamped,'/localization/pose_estimator_map',self.on_pose,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odometry/filtered',lambda m:setattr(self,'local',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odometry/gnss_map',lambda m:setattr(self,'gnss',m),qos_profile_sensor_data)
        self.create_subscription(Bool,'/system/autonomy_motion_allowed',lambda m:setattr(self,'auto',m.data),state)
        self.create_subscription(Bool,'/esc/ready',lambda m:setattr(self,'ready',m.data),state)
        self.create_subscription(Bool,'/esc/feedback_valid',lambda m:setattr(self,'feedback',m.data),state)
        self.create_subscription(Bool,'/safety/estop',lambda m:setattr(self,'estop',m.data),state)
        self.create_subscription(Float64,'/esc/drive_actual_mps',lambda m:setattr(self,'actual',m.data),qos_profile_sensor_data)
        self.create_subscription(Float64,'/esc/drive_target_mps',lambda m:setattr(self,'target',m.data),qos_profile_sensor_data)
        self.create_subscription(Float64,'/esc/steering_actual_rad',lambda m:setattr(self,'steer',m.data),qos_profile_sensor_data)
        self.create_subscription(String,'/navigation/goal_state',lambda m:setattr(self,'goal',m.data),state)
        self.create_subscription(String,'/esc/mux/active_source',lambda m:setattr(self,'source',m.data),state)
    def on_pose(self,m): self.pose=m
rclpy.init(); n=Watch()
start_wait=time.monotonic(); stable_since=None
while rclpy.ok() and time.monotonic()-start_wait<12:
    rclpy.spin_once(n,timeout_sec=0.1)
    ok=n.pose is not None and n.auto and n.ready and n.feedback and not n.estop
    if ok:
        y=yaw_deg(n.pose.pose.pose.orientation)
        ok=abs(((y-90+180)%360)-180) <= 8.0
    if ok:
        stable_since=stable_since or time.monotonic()
        if time.monotonic()-stable_since>=2.0: break
    else: stable_since=None
if stable_since is None or time.monotonic()-stable_since<2.0:
    print('PRECHECK_FAIL',n.auto,n.ready,n.feedback,n.estop,'pose',n.pose is not None,'goal',n.goal,'source',n.source,flush=True)
    rclpy.shutdown(); raise SystemExit(2)
p=n.pose.pose.pose.position; y0=yaw_deg(n.pose.pose.pose.orientation)
x0=float(p.x); yy0=float(p.y); gx=x0; gy=yy0+1.5
print(f'PRECHECK_OK x0={x0:.4f} y0={yy0:.4f} yaw={y0:.2f} target=({gx:.4f},{gy:.4f},90deg)',flush=True)
print('GOAL_REPLY',post('/api/navigation/goal',{'x':gx,'y':gy,'yaw_deg':90.0}),flush=True)
t0=time.monotonic(); last=0; canceled=False
while rclpy.ok() and time.monotonic()-t0<25:
    rclpy.spin_once(n,timeout_sec=0.05)
    if n.pose is None: continue
    p=n.pose.pose.pose.position; yd=yaw_deg(n.pose.pose.pose.orientation)
    dy=p.y-yy0; dx=p.x-x0; herr=abs(((yd-90+180)%360)-180)
    now=time.monotonic()
    if now-last>=0.5:
        print(f'T {now-t0:5.2f}s state={n.goal or "-":9s} src={n.source or "-":7s} dx={dx:+.3f} dy={dy:+.3f} yaw={yd:6.2f} vtar={n.target:+.3f} vact={n.actual:+.3f} steer={math.degrees(n.steer):+.2f} auto={n.auto}',flush=True); last=now
    bad=None
    if not n.auto: bad='autonomy_false'
    elif n.estop: bad='estop'
    elif not n.ready or not n.feedback: bad='esc_not_ready'
    elif abs(dx)>0.50: bad='lateral_deviation'
    elif herr>15.0: bad='heading_deviation'
    elif abs(n.actual)>0.35: bad='speed_over_limit'
    elif dy>1.80 or dy < -0.20: bad='distance_direction_guard'
    if bad:
        print('WATCHDOG_CANCEL',bad,flush=True); print('CANCEL_REPLY',post('/api/navigation/cancel'),flush=True); canceled=True; break
    if n.goal in ('SUCCEEDED','ABORTED','CANCELED') and now-t0>1.0: break
if time.monotonic()-t0>=25 and n.goal not in ('SUCCEEDED','ABORTED','CANCELED'):
    print('WATCHDOG_CANCEL timeout',flush=True); print('CANCEL_REPLY',post('/api/navigation/cancel'),flush=True); canceled=True
for _ in range(40): rclpy.spin_once(n,timeout_sec=0.05)
p=n.pose.pose.pose.position if n.pose else None; yd=yaw_deg(n.pose.pose.pose.orientation) if n.pose else float('nan')
print('FINAL',n.goal,'pose',None if p is None else (p.x,p.y,yd),'delta',None if p is None else (p.x-x0,p.y-yy0),'v',n.actual,'target',n.target,'steer_deg',math.degrees(n.steer),'auto',n.auto,'canceled',canceled,flush=True)
rclpy.shutdown()
