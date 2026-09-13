#!/usr/bin/env python3
"""Runtime P0 proof: dropped STOP/F4 silence must zero /hmi/cmd_vel within 350 ms."""
import os, pty, fcntl, subprocess, time, signal, sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Twist

WS=Path(__file__).resolve().parents[3]
DOMAIN=str(180 + (os.getpid() % 40))
os.environ['ROS_DOMAIN_ID']=DOMAIN
master, slave = pty.openpty()
slave_path=os.ttyname(slave)
os.close(slave)
fcntl.fcntl(master, fcntl.F_SETFL, os.O_NONBLOCK)
env=os.environ.copy(); env['ROS_DOMAIN_ID']=DOMAIN
binary=WS/'build/stmf4/stmf4_hmi_bridge'
if not binary.is_file(): binary=WS/'install/stmf4/lib/stmf4/stmf4_hmi_bridge'
if not binary.is_file(): raise RuntimeError('stmf4_hmi_bridge binary not built')
bridge=[str(binary),'--ros-args',
        '-p',f'serial_device:={slave_path}','-p','serial_baud:=1000000',
        '-p','hmi_transport_timeout_sec:=2.0','-p','default_mode:=AUTO']
log_path=Path('/tmp')/f'stmf4_manual_lease_runtime_{os.getpid()}.log'
log=open(log_path,'w')
proc=subprocess.Popen(bridge,env=env,stdout=log,stderr=subprocess.STDOUT,text=True)

rclpy.init()
node=Node('stmf4_lease_fault_test')
state_qos=QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
req_pub=node.create_publisher(String,'/hmi/request',10)
esc_ready_pub=node.create_publisher(Bool,'/esc/ready',state_qos)
esc_fb_pub=node.create_publisher(Bool,'/esc/feedback_valid',state_qos)
steer_pub=node.create_publisher(Bool,'/esc/steer/connected',state_qos)
estop_pub=node.create_publisher(Bool,'/safety/estop',state_qos)
motion_pub=node.create_publisher(Bool,'/system/motion_ready',state_qos)
drive_actual_pub=node.create_publisher(Float64,'/esc/drive_actual_mps',10)
obs={'connected':False,'mode':'','nonzero_seen':False,'zero_after':None,'samples':[],'source':''}
node.create_subscription(Bool,'/hmi/connected',lambda m: obs.__setitem__('connected',m.data),state_qos)
node.create_subscription(String,'/hmi/operator_mode',lambda m: obs.__setitem__('mode',m.data),state_qos)
node.create_subscription(String,'/hmi/active_source',lambda m: obs.__setitem__('source',m.data),state_qos)
def cmd_cb(m):
    t=time.monotonic(); v=float(m.linear.x); obs['samples'].append((t,v))
    if abs(v)>1e-6: obs['nonzero_seen']=True
    if obs['nonzero_seen'] and last_tx[0] is not None and t>=last_tx[0] and abs(v)<=1e-6 and obs['zero_after'] is None:
        obs['zero_after']=t
node.create_subscription(Twist,'/hmi/cmd_vel',cmd_cb,10)

buf=b''; token=None; last_tx=[None]
def spin(dt=0.01):
    global buf, token
    end=time.monotonic()+dt
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=0.002)
        try:
            data=os.read(master,4096)
        except BlockingIOError:
            data=b''
        except OSError as e:
            if e.errno == 5: data=b''  # PTY slave not open yet
            else: raise
        if data:
            buf+=data
            while b'\n' in buf:
                raw,buf=buf.split(b'\n',1)
                line=raw.decode(errors='replace').strip()
                if line.startswith('HOST:HELLO:'):
                    token=line.split(':',2)[2]
                    os.write(master,f'ACK:HOST:SESSION:{token}:1\r\n'.encode())
        time.sleep(0.001)

def wait(pred,sec,label):
    deadline=time.monotonic()+sec
    while time.monotonic()<deadline:
        spin(0.02)
        if pred(): return
    raise RuntimeError('timeout '+label)

try:
    wait(lambda: token is not None,3,'HOST:HELLO')
    wait(lambda: obs['connected'],3,'connected')
    for pub,val in [(esc_ready_pub,True),(esc_fb_pub,True),(steer_pub,True),(estop_pub,False),(motion_pub,True)]:
        m=Bool(); m.data=val; pub.publish(m)
    f=Float64(); f.data=0.0; drive_actual_pub.publish(f)
    # Give transient state callbacks time, then switch to MANUAL.
    for _ in range(10):
        for pub,val in [(esc_ready_pub,True),(esc_fb_pub,True),(steer_pub,True),(estop_pub,False),(motion_pub,True)]:
            m=Bool(); m.data=val; pub.publish(m)
        drive_actual_pub.publish(f); spin(0.03)
    req=String(); req.data='MODE:MANUAL'; req_pub.publish(req)
    wait(lambda: obs['mode']=='MANUAL',2,'manual mode')
    # Simulate physical-HMI HOLD: refresh FWD at 100 ms. No STOP is ever sent.
    for i in range(8):
        os.write(master,b'CMD:DRIVE:FWD:20\r\n')
        last_tx[0]=time.monotonic()
        spin(0.10)
    wait(lambda: obs['nonzero_seen'],1,'nonzero /hmi/cmd_vel')
    # USB/F4 silence: no FWD refresh and intentionally no STOP packet.
    t_last=last_tx[0]
    wait(lambda: obs['zero_after'] is not None,1.0,'lease zero')
    latency=obs['zero_after']-t_last
    peak=max(abs(v) for _,v in obs['samples']) if obs['samples'] else 0.0
    print(f'P0_LEASE_RESULT latency_ms={latency*1000:.3f} peak_mps={peak:.3f} source={obs["source"]} connected={obs["connected"]}')
    if latency > 0.350:
        raise RuntimeError(f'lease fail-safe exceeded 350 ms: {latency*1000:.3f} ms')
    print('P0_MANUAL_LEASE_RUNTIME_PASS')
finally:
    try: proc.send_signal(signal.SIGINT); proc.wait(timeout=3)
    except Exception:
        proc.kill()
    log.close(); os.close(master)
    node.destroy_node(); rclpy.shutdown()
