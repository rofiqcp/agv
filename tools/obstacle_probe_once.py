#!/usr/bin/env python3
import json, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Probe(Node):
    def __init__(self):
        super().__init__('obstacle_probe_once')
        self.data = {}
        self.create_subscription(String, '/perception/raw_detections', lambda m:self.cb('raw', m), 10)
        self.create_subscription(String, '/perception/semantic_detections', lambda m:self.cb('semantic', m), 10)
    def cb(self, key, msg):
        try:
            d=json.loads(msg.data)
        except Exception as e:
            d={'error':str(e)}
        self.data[key]=d
        print('\n###', key)
        for x in d.get('detections', []):
            keep={k:x.get(k) for k in ('class_id','class_name','score','x1','y1','x2','y2','bbox_height_fraction','center_x_fraction','raw_forward_m','class_resolution','competing_class_id','competing_score')}
            print(json.dumps(keep, sort_keys=True))
        if 'raw' in self.data and 'semantic' in self.data:
            rclpy.shutdown()

rclpy.init()
n=Probe()
deadline=time.time()+12
while rclpy.ok() and time.time()<deadline:
    rclpy.spin_once(n, timeout_sec=0.5)
if rclpy.ok():
    rclpy.shutdown()
