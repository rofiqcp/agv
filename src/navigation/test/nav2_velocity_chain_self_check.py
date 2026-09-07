#!/usr/bin/env python3
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
launch = (ROOT / 'launch/autonomous.launch.py').read_text()
core = (ROOT / 'src/navigation_core.cpp').read_text()
router = (ROOT / 'src/cmd_vel_router.cpp').read_text()
esc = (ROOT.parent / 'esc/src/ackermann_controller_server.cpp').read_text()
teleop = (ROOT.parent / 'esc/src/motor_teleop.cpp').read_text()
prod = (ROOT / 'config/collision_monitor_production.yaml').read_text()
nav2 = yaml.safe_load((ROOT / 'config/nav2_ackermann.yaml').read_text())

def need(cond, message):
    if not cond:
        raise SystemExit('FAIL ' + message)

need("remappings=[('cmd_vel', '/cmd_vel_nav_raw')]" in launch, 'MPPI raw topic missing')
need("('cmd_vel', '/cmd_vel/pre_smoother'), ('cmd_vel_smoothed', '/cmd_vel')" in launch, 'velocity_smoother must own final /cmd_vel')
need("executable='cmd_vel_router'" in launch, 'cmd_vel_router missing')
need("'teleop_topic': '/cmd_vel/ackermann_direct_teleop_disabled'" in launch, 'main stack must disable direct Ackermann teleop bypass')
need("'teleop_source_topic': '/navigation/cmd_mux/source'" in launch, 'Ackermann must receive router source metadata')
need("'nav2_topic': '/cmd_vel'" in launch, 'Ackermann must consume smoothed unified /cmd_vel')
need('/cmd_vel/autonomy_pre_smoother' in core and '/cmd_vel/autonomy_pre_smoother' in prod, 'autonomy pre-smoother chain missing')
need('/cmd_vel/teleop' in router and '/cmd_vel/pre_smoother' in router, 'teleop router input/output contract missing')
need('global_estop_topic' in router and 'autonomy_gate_topic' in router, 'router safety gates missing')
need('routed_teleop' in esc and 'selected.teleop = true' in esc, 'Ackermann must preserve TELEOP steering semantics after smoothing')
need('/teleop/joystick_connected' in teleop, 'joystick connection telemetry missing')
vs = nav2['velocity_smoother']['ros__parameters']
need(float(vs['min_velocity'][0]) < 0.0 and float(vs['max_velocity'][0]) > 0.0, 'velocity smoother must preserve forward and reverse joystick commands')
need('global_estop_topic' in esc, 'ESC immediate E-stop path must remain after smoother')
print('PASS velocity chain: JOYSTICK -> motor_teleop -> cmd_vel_router -> velocity_smoother -> /cmd_vel -> Ackermann -> ESC')
