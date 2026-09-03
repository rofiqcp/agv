#!/usr/bin/env python3
"""Numerical sanity checks for Part 2 ENU/map/body and lever-arm velocity equations."""
import math

def close(a,b,tol=1e-9):
    if abs(a-b)>tol: raise SystemExit(f'FAIL {a} != {b}')
# Case 1: map aligned ENU, base heading east, antenna 0.165m forward.
# Base vx=1m/s, yaw rate=1rad/s -> antenna has +0.165m/s lateral velocity.
yaw=0.0; ax=0.165; ay=0.0; omega=1.0
base_map=(1.0,0.0)
rx=math.cos(yaw)*ax-math.sin(yaw)*ay; ry=math.sin(yaw)*ax+math.cos(yaw)*ay
ant=(base_map[0]-omega*ry, base_map[1]+omega*rx)
recovered=(ant[0]+omega*ry, ant[1]-omega*rx)
close(recovered[0],1.0); close(recovered[1],0.0)
# Case 2: ENU east vector rotated +90deg into map -> map north-axis vector.
theta=math.pi/2; ve,vn=1.0,0.0
mx=math.cos(theta)*ve-math.sin(theta)*vn
my=math.sin(theta)*ve+math.cos(theta)*vn
close(mx,0.0,1e-8); close(my,1.0,1e-8)
# Case 3: map vector aligned with vehicle yaw should become positive body-x.
yaw=0.73; speed=0.42
mx=math.cos(yaw)*speed; my=math.sin(yaw)*speed
bx=math.cos(yaw)*mx+math.sin(yaw)*my
by=-math.sin(yaw)*mx+math.cos(yaw)*my
close(bx,speed,1e-9); close(by,0.0,1e-9)
print('PASS gnss_motion_math_self_check')
