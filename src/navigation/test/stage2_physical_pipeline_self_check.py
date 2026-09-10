#!/usr/bin/env python3
from pathlib import Path
import math
import yaml

WS=Path(__file__).resolve().parents[3]
NAV=WS/'src/navigation'
ESC=WS/'src/esc'

def text(rel): return (WS/rel).read_text(encoding='utf-8')
def params(path,node):
    return (yaml.safe_load(path.read_text(encoding='utf-8')) or {})[node]['ros__parameters']
def need(ok,msg):
    if not ok: raise AssertionError(msg)

field=text('src/navigation/tools/navigation_field_campaign.py')
proposal=text('src/navigation/tools/stage2_calibration_proposal.py')
cal=text('src/navigation/tools/navigation_calibration.py')
magcpp=text('src/navigation/src/mag_heading_fusion_node.cpp')
esccpp=text('src/esc/src/ackermann_controller_server.cpp')
cmake=text('src/navigation/CMakeLists.txt')
vehicle=params(NAV/'config/vehicle.yaml','vehicle')
imu=params(NAV/'config/imu.yaml','data_imu_node')
mag=params(NAV/'config/mag_heading.yaml','mag_heading_fusion')
loc=params(NAV/'config/localization_cpp.yaml','localization_core')
mppi=params(NAV/'config/mppi_closed_loop.yaml','mppi_closed_loop_supervisor')
need(imu.get('calibration_level')=='commissioning','short IMU calibration must stay commissioning')
need(imu.get('field_calibration_valid') is False,'IMU field flag must default false')
need(float(imu.get('field_calibration_stationary_duration_sec',0.0))==0.0,'field duration must not be fabricated')
need(imu.get('accel_scale') == [1.0,1.0,1.0],'uncertified accel scale must remain neutral')
need('accel_scale_' in text('src/navigation/include/imu/imu_node.hpp'),'IMU runtime accel scale state missing')
need('accel_scale_[0]' in text('src/navigation/src/imu_node.cpp'),'IMU publish path does not apply accel scale')
need("'accel_scale': six['scale']" in proposal,'six-position scale missing from review proposal')
need('imu/calibration_vectors' in text('src/navigation/src/imu_node.cpp'),'body-frame pre-calibration IMU topic missing')
need('scale>=0.80' in field and 'scale<=1.20' in field,'six-position scale sanity gate missing')
need("'vehicle_changes': {'control_field_calibration_valid'" in proposal,'control certification proposal points to wrong authority')
need(vehicle.get('drive_odometry_calibration_valid') is False,'drive flag must remain fail-closed')
need(vehicle.get('steering_calibration_valid') is False,'steering flag must remain fail-closed')
need(vehicle.get('steering_circle_calibration_valid') is False,'circle flag must remain fail-closed')
need(vehicle.get('control_field_calibration_valid') is False,'control field flag must remain fail-closed')
need(mag.get('field_qualification_valid') is False,'mag field flag must remain fail-closed')
need(mag.get('enable_current_emi_gate') is False,'EMI gate cannot auto-enable without threshold evidence')
need(mag.get('motor_current_topic')=='/esc/motor_current_abs_a','mag current source mismatch')
need(float(mppi.get('min_odom_rate_hz',0.0))>=25.0,'closed-loop qualification rate must match 30 Hz local EKF')
need(loc.get('require_gnss_velocity_certification_for_fusion') is True,'GNSS velocity certification gate missing')
need(loc.get('require_gnss_cog_certification_for_fusion') is True,'GNSS COG certification gate missing')
need(loc.get('gnss_velocity_calibration_valid') is False,'GNSS velocity must not auto-certify')
need(loc.get('gnss_cog_calibration_valid') is False,'GNSS COG must not auto-certify')

for name in ('navigation_field_campaign.py','stage2_calibration_proposal.py'):
    need(name in cmake,f'{name} not installed by CMake')
need('production_certification_changed' in field and "'precision_global':'DEFERRED_NO_RTK'" in field,
     'field campaign must be non-mutating and no-RTK scoped')
need('automatic_runtime_write' in proposal and 'automatic_certification_promotion' in proposal,
     'proposal tool must be review-only')
for dataset in (
    'imu_stationary.csv','imu_six_position.csv','imu_motor_off.csv','imu_traction_on.csv',
    'imu_steering_on.csv','imu_cold_start.csv','imu_warm_start.csv','drive_multispeed.csv',
    'steering_hysteresis.csv','steering_circles.csv','lever_arm.csv','map_alignment.csv',
    'mag_interference.csv','gnss_motion_validation.csv','localization_innovations.csv',
    'localization_reacquisition.csv','nav2_ab.csv','control_slalom.csv','ground_truth.csv'):
    need(dataset in field,f'missing Stage-2 dataset contract: {dataset}')
for marker in ('distance_error_p95_fraction','radius_error_p95_fraction','estimated_steering_tau_sec',
               'gnss_motion_validation','localization_innovations','localization_reacquisition','nav2_ab'):
    need(marker in field,f'missing acceptance metric/gate: {marker}')
need('steering_tau_step_count' in cal,'control analyzer missing tau evidence')
need("report['vehicle_max_forward_speed_mps']" in proposal,
     'yaw-rate proposal must derive from measured Rmin and vehicle max speed')

need('/esc/motor_current_abs_a' in esccpp,'ESC motor-current evidence topic missing')
need('motor_current_abs_pub_' in esccpp,'ESC motor-current publisher missing')
need('currentEmiGate' in magcpp and 'motor_current_stale' in magcpp and 'current_emi_gate' in magcpp,
     'magnetic current-aware EMI rejection missing')
need('motor_current_age_sec' in magcpp,'magnetic EMI diagnostic age missing')
gui_cal=text('src/navigation/gui/modules/system_and_gnss_pages.cpp')
gui_core=text('src/navigation/gui/modules/gui_core.cpp')
need('QString("imu_calibration")' not in gui_cal,'commissioning wizard still writes legacy IMU authority')
need('put("imu_calibration"' not in gui_core,'legacy imu_calibration.yaml still loaded as active GUI authority')
legacy_imu=text('src/navigation/config/imu_calibration.yaml')
legacy_data=yaml.safe_load(legacy_imu) or {}
need(legacy_imu.startswith('# LEGACY EVIDENCE ONLY'),'legacy IMU file missing deprecation guard')
need(legacy_data['data_imu_node']['ros__parameters'].get('stationary_calibration_valid') is False,'legacy IMU evidence can still certify runtime')
need('data_imu_node.ros__parameters.accel_scale' in text('src/navigation/gui/agv_gui_specs.hpp'),'accel scale missing from tuning catalog')
need('GNSS field evidence dicabut' not in gui_cal,'IMU commissioning still mutates unrelated GNSS evidence semantics')
need('IMU STAGE-2 CERTIFIED' not in gui_cal,'short IMU commissioning is mislabeled as field certification')
need('Estimator selalu memakai GNSS vx+vyaw' not in gui_cal,'GNSS GUI still documents obsolete estimator topology')
need('Certify COG Diagnostic' not in gui_cal,'GNSS COG is still mislabeled diagnostic-only')
need('enable_global_gnss_cog_fusion",false' not in gui_cal,'GNSS Stage-2 GUI can disable canonical COG fusion policy')

print('PASS stage2_physical_pipeline_self_check')
print('evidence pipeline: IMU/drive/steering/circle/mag/GNSS/NIS/reacquisition/Nav2/control/GT')
print('certification policy: fail-closed, review-only, no ESKF, no RTK calibration')
