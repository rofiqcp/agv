#!/usr/bin/env python3
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
launch = (ROOT / 'launch/autonomous.launch.py').read_text()
core = (ROOT / 'src/navigation_core.cpp').read_text()
router = (ROOT / 'src/cmd_vel_router.cpp').read_text()
esc = (ROOT.parent / 'esc/src/ackermann_controller_server.cpp').read_text()
teleop = (ROOT.parent / 'esc/src/motor_teleop.cpp').read_text()
web = (ROOT / 'web/web_server.cpp').read_text()
prod = (ROOT / 'config/collision_monitor_production.yaml').read_text()
nav2 = yaml.safe_load((ROOT / 'config/nav2_ackermann.yaml').read_text())

def need(cond, message):
    if not cond:
        raise SystemExit('FAIL ' + message)

need("remappings=[('cmd_vel', '/cmd_vel_nav_raw')]" in launch, 'MPPI raw topic missing')
need(launch.count("remappings=[('cmd_vel', '/cmd_vel_nav_raw')]") >= 2, 'controller and behavior server must both route through raw Nav2 chain')
need("('cmd_vel', '/cmd_vel/pre_smoother'), ('cmd_vel_smoothed', '/cmd_vel')" in launch, 'velocity_smoother must own final /cmd_vel')
need("executable='cmd_vel_router'" in launch, 'cmd_vel_router missing')
need("'teleop_topic': '/cmd_vel/teleop'" in launch, 'main stack must preserve direct manual steering semantics at Ackermann')
need("'teleop_source_topic': '/teleop/active_source'" in launch, 'Ackermann must receive direct manual authority metadata')
need("'router_source_topic': '/navigation/cmd_mux/source'" in launch, 'Ackermann must receive router source metadata separately')
need("'nav2_topic': '/cmd_vel'" in launch, 'Ackermann must consume smoothed unified /cmd_vel')
need('/cmd_vel/autonomy_pre_smoother' in core and '/cmd_vel/autonomy_pre_smoother' in prod, 'autonomy pre-smoother chain missing')
need('/cmd_vel/teleop' in router and '/cmd_vel/pre_smoother' in router, 'teleop router input/output contract missing')
need('/cmd_vel/web_trial' in router and '/web_trial/active_source' in router, 'Web Trial must have an isolated manual input')
need('create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/web_trial"' in web, 'Web GUI must not publish directly on physical joystick topic')
need('create_publisher<geometry_msgs::msg::Twist>("/cmd_vel/teleop"' not in web, 'Web GUI must not be a second /cmd_vel/teleop writer')
need('global_estop_topic' in router and 'autonomy_gate_topic' in router, 'router safety gates missing')
need('routed_teleop' in esc and 'selected.teleop = true' in esc, 'Ackermann must preserve TELEOP steering semantics on the routed mirror path')
need('router_source_ == "WEB_TRIAL"' not in esc.split('const bool routed_teleop',1)[1].split('const bool routed_web_trial',1)[0],
     'WEB_TRIAL must never be classified as TELEOP takeover authority')
need('(routed_web_trial && !gate_ok)' in esc, 'WEB_TRIAL must be blocked whenever an admitted autonomous mission owns the gate')
need('fraction * operationalPhysicalLimitDeg()' in esc, 'full TELEOP stick must map directly to full physical steering limit')
need('physicalToVescPositionDeg(steering_deg)' in esc, 'physical steering must map to calibrated VESC POS exactly once')
need('steering_vesc_pos_left_deg: 0.0' in (ROOT.parent / 'esc/config/ackermann.yaml').read_text(), 'LEFT full-steer endpoint must remain VESC POS 0')
need('steering_vesc_pos_center_deg: 180.0' in (ROOT.parent / 'esc/config/ackermann.yaml').read_text(), 'CENTER endpoint must remain VESC POS 180')
need('steering_vesc_pos_right_deg: 360.0' in (ROOT.parent / 'esc/config/ackermann.yaml').read_text(), 'RIGHT full-steer endpoint must remain VESC POS 360')
need('full LEFT = VESC POS 0' in esc and 'stick CENTER/release = 180' in esc and 'full RIGHT = 360' in esc, 'physical TELEOP raw POS 0/180/360 contract missing')
need('left_position_active{false}' in esc and 'appendVescSetCurrent(batch, 0.0)' in esc, 'IDLE/watchdog steering must torque-off instead of buzzing at POS180')
need('/teleop/joystick_connected' in teleop, 'joystick connection telemetry missing')
vs = nav2['velocity_smoother']['ros__parameters']
need(float(vs['min_velocity'][0]) < 0.0 and float(vs['max_velocity'][0]) > 0.0, 'velocity smoother must preserve forward and reverse joystick commands')
need('global_estop_topic' in esc, 'ESC immediate E-stop path must remain after smoother')
print('PASS velocity chains: TELEOP actuator=/cmd_vel/teleop -> Ackermann direct priority; routed mirror=/cmd_vel/teleop -> router -> smoother -> /cmd_vel; AUTONOMY -> router -> smoother -> /cmd_vel -> Ackermann')
