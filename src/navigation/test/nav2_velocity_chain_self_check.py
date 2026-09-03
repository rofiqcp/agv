#!/usr/bin/env python3
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
launch=(ROOT/'launch/autonomous.launch.py').read_text()
core=(ROOT/'src/navigation_core.cpp').read_text()
router=(ROOT/'src/cmd_vel_router.cpp').read_text()
esc=(ROOT.parent/'esc/src/ackermann_controller_server.cpp').read_text()
prod=(ROOT/'config/collision_monitor_production.yaml').read_text()

def need(c,m):
    if not c: raise SystemExit('FAIL '+m)
need("remappings=[('cmd_vel', '/cmd_vel_nav_raw')]" in launch,'MPPI raw topic missing')
need("('cmd_vel', '/cmd_vel/pre_smoother'), ('cmd_vel_smoothed', '/cmd_vel')" in launch,'final velocity_smoother must own /cmd_vel')
need("executable='cmd_vel_router'" in launch,'cmd_vel_router missing')
need("'teleop_topic': '/cmd_vel/teleop'" in launch,'ESC direct teleop path missing for responsive manual control')
need("'require_autonomy_gate': 'false'" in launch,'ESC gate must move before smoother in autonomous runtime')
need('/cmd_vel/autonomy_pre_smoother' in core,'NavigationCore pre-smoother output missing')
need('/cmd_vel/autonomy_pre_smoother' in prod,'collision output must feed mux before smoother')
need('/cmd_vel/teleop' in router and '/cmd_vel/pre_smoother' in router,'router input/output contract missing')
need('global_estop_topic' in router and 'autonomy_gate_topic' in router,'router safety gates missing')
need('global_estop_topic' in esc,'ESC immediate E-stop path must remain after smoother')
print('PASS velocity chain: TELEOP -> ESC mux direct; AUTONOMY -> safety/router -> velocity_smoother -> /cmd_vel -> ESC/USART')
