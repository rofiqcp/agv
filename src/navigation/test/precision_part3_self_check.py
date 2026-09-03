#!/usr/bin/env python3
from gui_source_helper import read_gui_source
"""Static contract checks for Precision Part 3 production/tuning changes."""
from pathlib import Path
import sys
try:
    import yaml
except Exception as exc:
    print(f"FAIL: python3-yaml required: {exc}", file=sys.stderr); raise SystemExit(2)
ROOT=Path(__file__).resolve().parents[1]; WS=ROOT.parent

def load(p): return yaml.safe_load(Path(p).read_text(encoding='utf-8')) or {}
def fail(msg): print('FAIL:',msg,file=sys.stderr); raise SystemExit(1)

mppi=load(ROOT/'config/mppi_closed_loop.yaml')['mppi_closed_loop_supervisor']['ros__parameters']
cpp=(ROOT/'src/mppi_closed_loop_supervisor.cpp').read_text()
for key in ['qualification_window_sec','min_qualification_samples','min_odom_rate_hz','max_odom_jitter_sec','max_velocity_rmse_mps','max_steering_rmse_rad','max_yaw_rate_rmse_rps']:
    if key not in mppi or key not in cpp: fail(f'smoother qualification missing {key}')
for token in ['/navigation/velocity_smoother/closed_loop_eligible','/navigation/velocity_smoother/qualification','smootherEligible']:
    if token not in cpp: fail(f'qualification runtime token missing {token}')

preview=load(ROOT/'config/collision_monitor.yaml')['collision_monitor']['ros__parameters']
prod=load(ROOT/'config/collision_monitor_production.yaml')['collision_monitor']['ros__parameters']
if preview.get('cmd_vel_out_topic') == '/cmd_vel': fail('preview config must never own final cmd_vel')
if prod.get('cmd_vel_out_topic') != '/cmd_vel/autonomy_pre_smoother': fail('production collision monitor must feed autonomy pre-smoother')
auto=(ROOT/'launch/autonomous.launch.py').read_text()
for token in ["collision_monitor_production.yaml", "_yaml_ros_param", "collision_monitor_enabled': ParameterValue", "AGV_CLEAN_STALE_RUNTIME"]:
    if token not in auto: fail(f'autonomous production routing missing {token}')
if '_cleanup_stale_workspace_runtime(nav_share, esc_share, astra_share)\n' in auto and "AGV_CLEAN_STALE_RUNTIME" not in auto:
    fail('destructive stale cleanup must be opt-in')

gui = read_gui_source(ROOT) + (ROOT / 'gui/agv_gui_specs.hpp').read_text(encoding='utf-8')
for token in ['class NavigationTuningPage','Buat Paket Sweep','CLOSED_LOOP ELIGIBLE','numeric_summary','experiment_category','void certify()','camera_metric_validation','Mulai Rosbag','Peringkat Eksperimen']:
    if token not in gui: fail(f'GUI Part3 feature missing {token}')
per=load(WS/'perception/config/astra_yolop_gpu.yaml')['perception']['ros__parameters']
if 'camera_metric_calibration_validated' not in per: fail('perception calibration certification key missing')
per_cpp=(WS/'perception/src/astra_yolop_gpu_node.cpp').read_text()
if 'declare_parameter<bool>("camera_metric_calibration_validated"' not in per_cpp: fail('perception certification key not declared')

cmake=(ROOT/'CMakeLists.txt').read_text()
for tool in ['experiment_rank.py','rosbag_regression.py','runtime_profile.py']:
    if not (ROOT/'tools'/tool).exists() or tool not in cmake: fail(f'Part3 tool not installed: {tool}')

print('PASS precision_part3_self_check')
