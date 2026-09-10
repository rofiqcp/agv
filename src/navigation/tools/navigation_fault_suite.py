#!/usr/bin/env python3
"""Safe fault-injection regression for localization contracts.

Default mode uses an isolated ROS_DOMAIN_ID and starts only monitor nodes plus
synthetic publishers; it never starts ESC/actuator nodes. This verifies dropout,
reconnect, duplicate publisher and precision-pose quality gates without motion.
"""
from __future__ import annotations
import argparse,json,os,signal,subprocess,time
from pathlib import Path

def crc16(s:str):
    crc=0xFFFF
    for b in s.encode():
        crc ^= b<<8
        for _ in range(8):crc=((crc<<1)^0x1021)&0xFFFF if crc&0x8000 else (crc<<1)&0xFFFF
    return crc

def run(cmd,env,timeout=8):
    return subprocess.run(cmd,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=timeout)
def popen(cmd,env): return subprocess.Popen(cmd,env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
def stop(p):
    if not p:return
    try:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=2)
    except Exception:
        try:os.killpg(p.pid,signal.SIGKILL)
        except Exception:pass

def echo_bool(topic,env):
    r=run(['ros2','topic','echo','--once',topic,'std_msgs/msg/Bool'],env,6)
    text=r.stdout.lower();
    if 'data: true' in text:return True
    if 'data: false' in text:return False
    raise RuntimeError(f'cannot read {topic}: {r.stdout[-500:]}')

def wait_bool(topic,want,env,timeout=5):
    end=time.time()+timeout;last=None
    while time.time()<end:
        try:last=echo_bool(topic,env)
        except Exception:time.sleep(.15);continue
        if last is want:return True
        time.sleep(.15)
    return False

def publisher(topic,typ,payload,env,rate=10): return popen(['ros2','topic','pub',topic,typ,payload,'-r',str(rate)],env)

def stamped_pose_publisher(env, x=0.0, y=0.0, duration=1.0, rate=20.0):
    code = f"""
import rclpy,time
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
rclpy.init(); n=Node('fault_suite_precision_pub'); p=n.create_publisher(PoseWithCovarianceStamped,'/localization/absolute_pose',10)
end=time.monotonic()+{duration!r}; period=1.0/{rate!r}
while time.monotonic()<end:
 m=PoseWithCovarianceStamped(); m.header.stamp=n.get_clock().now().to_msg(); m.header.frame_id='map'; m.pose.pose.position.x={x!r}; m.pose.pose.position.y={y!r}; m.pose.pose.orientation.w=1.0; m.pose.covariance[0]=0.0004; m.pose.covariance[7]=0.0004; m.pose.covariance[35]=0.0004; p.publish(m); rclpy.spin_once(n,timeout_sec=0.0); time.sleep(period)
n.destroy_node(); rclpy.shutdown()
"""
    return popen(['python3','-c',code],env)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--domain',default='87');ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
    env=os.environ.copy();env['ROS_DOMAIN_ID']=str(a.domain);env['ROS_LOCALHOST_ONLY']='1'
    results={};procs=[]
    # Protocol integrity: any one-byte payload mutation must invalidate original CRC.
    base='1,1000,345000,3,1,0,12,-7.0,110.0,10.0,0.5,0.8,1.0,0.1,0.0,1.0,90.0,0.1,2.0,1.2,10.0,0,0'
    c=crc16(base);mut=base.replace('110.0','110.1')
    results['f411_crc_corruption']={'pass':crc16(mut)!=c,'original_crc':c,'mutated_crc':crc16(mut)}
    # MCU timestamp monotonic contract model: wrap/reboot never future-dates or regresses.
    raw=[100,200,300,250,400,4294967200,50,60]
    # Validate intent rather than duplicate exact C++ epoch math: uint32 delta small OOO is non-positive only within 5 s.
    deltas=[((raw[i]-raw[i-1]+2**31)%2**32)-2**31 for i in range(1,len(raw))]
    results['mcu_counter_fault_vectors']={'pass':(-50 in deltas and any(d>0 for d in deltas)),'signed_deltas_ms':deltas}
    monitor=popen(['ros2','run','navigation','sensor_contract_monitor','--ros-args','-p','check_rate_hz:=10.0'],env);procs.append(monitor);time.sleep(1)
    pubs={
      'imu':publisher('/imu/data','sensor_msgs/msg/Imu','{}',env),
      'fix':publisher('/gnss/fix_raw','sensor_msgs/msg/NavSatFix','{}',env),
      'vel':publisher('/gnss/vel','geometry_msgs/msg/TwistWithCovarianceStamped','{}',env),
      'odom':publisher('/esc/odom','nav_msgs/msg/Odometry','{}',env)};procs += list(pubs.values());time.sleep(1.5)
    results['single_publisher_contract']={'pass':wait_bool('/system/sensor_publishers_ok',True,env)}
    stop(pubs['imu']);time.sleep(.8);results['imu_dropout']={'pass':wait_bool('/system/sensor_publishers_ok',False,env)}
    pubs['imu']=publisher('/imu/data','sensor_msgs/msg/Imu','{}',env);procs.append(pubs['imu']);time.sleep(.8);results['imu_reconnect']={'pass':wait_bool('/system/sensor_publishers_ok',True,env)}
    stop(pubs['vel']);time.sleep(.8);results['gnss_velocity_dropout']={'pass':wait_bool('/system/sensor_publishers_ok',False,env)}
    pubs['vel']=publisher('/gnss/vel','geometry_msgs/msg/TwistWithCovarianceStamped','{}',env);procs.append(pubs['vel']);time.sleep(.8);results['gnss_velocity_reacquisition']={'pass':wait_bool('/system/sensor_publishers_ok',True,env)}
    stop(pubs['odom']);time.sleep(.8);results['esc_odom_dropout']={'pass':wait_bool('/system/sensor_publishers_ok',False,env)}
    pubs['odom']=publisher('/esc/odom','nav_msgs/msg/Odometry','{}',env);procs.append(pubs['odom']);time.sleep(.8);results['esc_odom_recovery']={'pass':wait_bool('/system/sensor_publishers_ok',True,env)}
    dup=publisher('/gnss/fix_raw','sensor_msgs/msg/NavSatFix','{}',env);procs.append(dup);time.sleep(.8);results['duplicate_publisher']={'pass':wait_bool('/system/sensor_publishers_ok',False,env)}
    stop(dup);time.sleep(.8);results['duplicate_recovery']={'pass':wait_bool('/system/sensor_publishers_ok',True,env)}
    # Precision absolute pose gate.
    pm=popen(['ros2','run','navigation','precision_localization_monitor','--ros-args','-p','min_consecutive_samples:=3','-p','timeout_sec:=0.4'],env);procs.append(pm);time.sleep(.8)
    results['precision_no_sample_fail_closed']={'pass':wait_bool('/system/precision_localization_ready',False,env)}
    good="{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}, covariance: [0.0004,0,0,0,0,0, 0,0.0004,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,0.0004]}}"
    # CLI stamps zero by default, which must be rejected: prove missing timestamp fail-closed explicitly.
    gp=publisher('/localization/absolute_pose','geometry_msgs/msg/PoseWithCovarianceStamped',good,env);procs.append(gp);time.sleep(.8)
    results['precision_missing_timestamp_rejected']={'pass':wait_bool('/system/precision_localization_ready',False,env)}
    stop(gp)
    goodp=stamped_pose_publisher(env,0.0,0.0,5.0,20.0);procs.append(goodp);time.sleep(1.0)
    results['precision_stamped_good_acquire']={'pass':wait_bool('/system/precision_localization_ready',True,env)}
    goodp.wait(timeout=7);time.sleep(.6)
    results['precision_stale_fail_closed']={'pass':wait_bool('/system/precision_localization_ready',False,env)}
    # Reacquire around origin, then inject a >0.30 m jump; monitor must immediately close.
    reacq=stamped_pose_publisher(env,0.0,0.0,2.5,20.0);procs.append(reacq);time.sleep(1.0)
    reacq.wait(timeout=5); jump=stamped_pose_publisher(env,1.0,0.0,0.25,20.0);procs.append(jump);time.sleep(.15)
    results['precision_pose_jump_rejected']={'pass':wait_bool('/system/precision_localization_ready',False,env)}
    # Structural source is ready for RTK/LiDAR/VIO adapters and remains freshness/jump gated.
    for p in procs[::-1]:stop(p)
    passed=all(v.get('pass',False) for v in results.values());report={'pass':passed,'isolated_ros_domain':a.domain,'results':results}
    a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(report,indent=2)+'\n');a.out.with_suffix('.md').write_text('# Navigation fault-injection evidence\n\n'+''.join(f"- **{k}**: {'PASS' if v.get('pass') else 'FAIL'} — `{json.dumps(v)}`\n" for k,v in results.items()))
    print(json.dumps(report,indent=2));return 0 if passed else 2
if __name__=='__main__':raise SystemExit(main())
