#!/usr/bin/env python3
import json, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

class Probe(Node):
    def __init__(self):
        super().__init__('raw_motor_probe')
        self.total=0; self.motor=[]
        self.create_subscription(String,'/perception/raw_detections',self.cb,qos_profile_sensor_data)
    def cb(self,msg):
        self.total+=1
        try: d=json.loads(msg.data)
        except Exception: return
        for x in d.get('detections',[]):
            if int(x.get('class_id',-1))==3:
                self.motor.append((time.monotonic(),x.get('score'),x.get('center_x_fraction'),x.get('bbox_height_fraction'),x.get('raw_forward_m'),x.get('bottom_clipped')))
rclpy.init(); n=Probe(); start=time.monotonic()
while rclpy.ok() and time.monotonic()-start<20:
    rclpy.spin_once(n,timeout_sec=.25)
print(json.dumps({'messages':n.total,'motor_hits':len(n.motor),'motor':n.motor[-30:]},indent=2))
rclpy.shutdown()
