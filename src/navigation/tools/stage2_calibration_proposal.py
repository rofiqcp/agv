#!/usr/bin/env python3
"""Generate review-only Stage-2 calibration proposals from accepted field evidence.

This tool never edits runtime YAML and never promotes certification flags.  It
only converts PASS evidence into candidate values for an operator review.
"""
from __future__ import annotations
import argparse, datetime, hashlib, json, math
from pathlib import Path
import numpy as np
import yaml


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def pass_item(report: dict, key: str) -> dict | None:
    item = (report.get('items') or {}).get(key) or {}
    return item if item.get('status') == 'PASS' else None


def common_steering_lut(metrics: dict, points: int = 9) -> dict:
    sweeps = metrics.get('sweeps') or {}
    names = sorted(sweeps)
    if len(names) < 2:
        raise ValueError('steering PASS evidence has fewer than two sweeps')
    inc = next((n for n in names if 'inc' in n or 'up' in n), names[0])
    dec = next((n for n in names if 'dec' in n or 'down' in n), names[1])
    a, b = sweeps[inc], sweeps[dec]
    lo = max(min(a['physical_deg']), min(b['physical_deg']))
    hi = min(max(a['physical_deg']), max(b['physical_deg']))
    if hi <= lo:
        raise ValueError('steering sweep overlap invalid')
    grid = np.linspace(lo, hi, max(5, int(points)))
    interp = lambda d, k: np.interp(grid, d['physical_deg'], d[k]).tolist()
    return {'physical_deg': grid.tolist(),
            'command_increasing_deg': interp(a, 'command_deg'),
            'command_decreasing_deg': interp(b, 'command_deg'),
            'feedback_increasing_deg': interp(a, 'feedback_deg'),
            'feedback_decreasing_deg': interp(b, 'feedback_deg')}


def build_proposal(report: dict, acceptance: Path) -> dict:
    if report.get('production_certification_changed') is not False:
        raise ValueError('acceptance report must be non-mutating')
    proposals, blocked = {}, []
    required_imu = ['imu_stationary', 'imu_six_position',
                    'imu_motor_interference', 'imu_repeatability']
    if all(pass_item(report, k) for k in required_imu):
        stat = pass_item(report, 'imu_stationary')['metrics']
        six = pass_item(report, 'imu_six_position')['metrics']
        proposals['imu'] = {
            'config': 'src/navigation/config/imu.yaml',
            'changes': {
                'accel_bias': six['bias_mps2'],
                'accel_scale': six['scale'],
                'gyro_bias': stat['gyro_mean_rps'],
                'field_calibration_valid': 'true_CANDIDATE_REVIEW_ONLY',
                'field_calibration_stationary_duration_sec': float(stat['duration_sec']),
                'field_calibration_six_position_valid': 'true_CANDIDATE_REVIEW_ONLY',
                'field_calibration_motor_interference_valid': 'true_CANDIDATE_REVIEW_ONLY',
                'angular_velocity_covariance': [max(float(x)**2, 1e-8) for x in stat['gyro_std_rps']],
                'linear_acceleration_covariance': [max(float(x)**2, 1e-8) for x in stat['accel_std_mps2']],
                'calibration_level': 'field_certified_CANDIDATE_REVIEW_ONLY'}}
    else:
        blocked.append('imu: requires stationary + six-position + EMI + repeatability PASS')

    drive = pass_item(report, 'drive_multispeed')
    if drive:
        m = drive['metrics']
        proposals['drive'] = {
            'configs': ['src/navigation/config/vehicle.yaml', 'src/esc/config/ackermann.yaml'],
            'changes': {'drive_odometry_calibration_scale': float(m['multiplicative_scale_vs_baseline']),
                        'drive_odometry_calibration_valid': 'true_CANDIDATE_REVIEW_ONLY',
                        'odom_v_variance_base_CANDIDATE': max(float(m['residual_rmse_mps'])**2, 1e-4)},
            'evidence': {'equivalent_erpm_per_mps': float(m['equivalent_erpm_per_mps']),
                         'distance_error_p95_fraction': float(m['distance_error_p95_fraction']),
                         'speed_residual_p95_mps': float(m['residual_p95_mps'])}}
    else:
        blocked.append('drive: requires multispeed 10/25/50m forward+reverse PASS')

    steer = pass_item(report, 'steering_hysteresis')
    if steer:
        m = steer['metrics']; lut = common_steering_lut(m)
        proposals['steering'] = {
            'configs': ['src/navigation/config/vehicle.yaml', 'src/esc/config/ackermann.yaml'],
            'vehicle_changes': {'measured_left_steering_limit_rad': math.radians(float(m['physical_left_limit_deg'])),
                                'measured_right_steering_limit_rad': math.radians(float(m['physical_right_limit_deg'])),
                                'steering_lut_point_count': len(lut['physical_deg']),
                                'steering_calibration_valid': 'true_CANDIDATE_REVIEW_ONLY',
                                'steering_calibration_source': 'field_sweep_CANDIDATE_REVIEW_ONLY'},
            'esc_lut_changes': lut}
    else:
        blocked.append('steering: requires increasing/decreasing command+feedback sweep PASS')

    circle = pass_item(report, 'steering_circles')
    if circle:
        m = circle['metrics']; radii = m.get('directions') or {}
        safety = 1.10
        rmin = float(m['minimum_measured_radius_m']) * safety
        proposals['ackermann_geometry'] = {
            'config': 'src/navigation/config/vehicle.yaml',
            'changes': {'effective_wheelbase_m': float(m['effective_wheelbase_m']),
                        'turning_radius_left_m': float((radii.get('left') or {})['radius_m']),
                        'turning_radius_right_m': float((radii.get('right') or {})['radius_m']),
                        'minimum_turning_radius_m': rmin,
                        'steering_circle_calibration_valid': 'true_CANDIDATE_REVIEW_ONLY',
                        'minimum_turning_radius_source': 'field_circle_fit_CANDIDATE_REVIEW_ONLY',
                        'max_yaw_rate_rps': float(report['vehicle_max_forward_speed_mps']) / rmin},
            'note': 'Rmin candidate includes 10% conservatism; launch remains SSOT materializer.'}
    else:
        blocked.append('ackermann geometry: requires left+right circle PASS')

    lever = pass_item(report, 'gnss_lever_arm')
    if lever:
        xyz = lever['metrics']['mean_m']
        proposals['gnss_lever_arm'] = {
            'config': 'src/navigation/config/localization_cpp.yaml',
            'changes': {'gnss_antenna_x_m': float(xyz[0]), 'gnss_antenna_y_m': float(xyz[1])},
            'evidence_z_m': float(xyz[2])}
    else:
        blocked.append('GNSS lever arm: requires repeated physical survey PASS')

    amap = pass_item(report, 'map_alignment')
    if amap:
        m = amap['metrics']
        proposals['map_alignment'] = {
            'config': 'src/navigation/config/localization_cpp.yaml',
            'changes': {'map_calibration_x_m': float(m['translation_m'][0]),
                        'map_calibration_y_m': float(m['translation_m'][1]),
                        'map_calibration_yaw_rad': float(m['yaw_rad'])},
            'evidence': {'rmse_m': float(m['rmse_m']), 'geometry_score': float(m['geometry_score'])}}
    else:
        blocked.append('map alignment: requires >=3 surveyed points PASS')

    gnss = pass_item(report, 'gnss_motion_validation')
    if gnss:
        m = gnss['metrics']
        proposals['gnss_motion_validation'] = {
            'config': 'src/navigation/config/localization_cpp.yaml',
            'changes': {
                'gnss_velocity_calibration_valid': 'true_CANDIDATE_REVIEW_ONLY',
                'gnss_cog_calibration_valid': 'true_CANDIDATE_REVIEW_ONLY',
                'gnss_velocity_calibration_epoch_count': int(m['samples']),
                'gnss_velocity_calibration_qualified_ratio': float(m['velocity_qualified_ratio']),
                'gnss_velocity_calibration_sync_p95_sec': float(m['sync_gap_p95_sec']),
                'gnss_velocity_calibration_wheel_residual_p95_mps': float(m['wheel_gnss_residual_p95_mps']),
                'gnss_velocity_calibration_lateral_p95_mps': float(m['base_lateral_velocity_p95_mps']),
                'gnss_cog_calibration_epoch_count': int(m['samples']),
                'gnss_cog_calibration_qualified_ratio': float(m['cog_qualified_ratio']),
                'gnss_cog_calibration_residual_p95_rad': float(m['cog_vs_doppler_p95_rad'])}}
    else:
        blocked.append('GNSS motion: requires Doppler/COG field validation PASS')

    innovations = pass_item(report, 'localization_innovations')
    if innovations:
        m = innovations['metrics']
        proposals['filter_health'] = {
            'config': 'src/navigation/config/localization_cpp.yaml', 'changes': {},
            'evidence': {'wheel_gate_pass_ratio': float(m['wheel_gate_pass_ratio']),
                         'cog_gate_pass_ratio': float(m['cog_gate_pass_ratio']),
                         'wheel_nis_p95': float(m['wheel_nis_p95']), 'cog_nis_p95': float(m['cog_nis_p95'])},
            'note': 'NIS thresholds are not auto-tuned; review reject ratios before changing chi-square gates.'}
    else:
        blocked.append('filter health: requires NIS field evidence PASS')

    reacq = pass_item(report, 'localization_reacquisition')
    if reacq:
        proposals['localization_reacquisition'] = {'config': 'src/navigation/config/localization_cpp.yaml',
            'changes': {}, 'evidence': reacq['metrics'],
            'note': 'Map correction gains remain review-only; no automatic gain increase from reacquisition data.'}
    else:
        blocked.append('localization reacquisition: requires >=3 dropout/reacquire trials PASS')

    nav2 = pass_item(report, 'nav2_ab')
    if nav2:
        proposals['nav2_ab'] = {'configs': ['src/navigation/config/vehicle.yaml','src/navigation/config/nav2_ackermann.yaml','src/navigation/config/mppi_closed_loop.yaml'],
            'changes': {}, 'evidence': nav2['metrics'],
            'note': '15 Hz precision-local profile is accepted only as local tracking; no absolute 8 cm claim.'}
    else:
        blocked.append('Nav2 A/B: requires standard_8hz vs precision_15hz field evidence PASS')

    mag = pass_item(report, 'mag_interference')
    if mag:
        m = mag['metrics']; corr = float(m.get('emi_current_correlation_abs', 0.0))
        proposals['magnetic_heading'] = {
            'config': 'src/navigation/config/mag_heading.yaml',
            'changes': {'field_qualification_valid': 'true_CANDIDATE_REVIEW_ONLY'},
            'evidence': {'heading_residual_p95_deg': float(m['heading_residual_p95_deg']),
                         'current_correlation_abs': corr},
            'note': 'Current EMI gate stays disabled unless a measured rejection threshold is separately derived.'}
    else:
        blocked.append('magnetic heading: requires motor_off + steering_on + traction_on PASS')

    control = pass_item(report, 'control_slalom')
    if control:
        m = control['metrics']
        proposals['control_identification'] = {
            'configs': ['src/navigation/config/vehicle.yaml', 'src/esc/config/ackermann.yaml'],
            'vehicle_changes': {'control_field_calibration_valid': 'true_CANDIDATE_REVIEW_ONLY'},
            'esc_changes': {},
            'evidence': {'steering_delay_sec': float(m['estimated_steering_delay_sec']),
                         'steering_tau_sec': float(m.get('estimated_steering_tau_sec', math.nan)),
                         'yaw_rate_residual_p95_rps': float(m['kinematic_yaw_rate_p95_rps']),
                         'reverse_yaw_feedback_qualified': bool(m.get('reverse_yaw_feedback_qualified', False))},
            'note': 'No automatic Kp/Ki proposal; tune feedforward then PI from reviewed step/slalom/circle traces.'}
    else:
        blocked.append('control: requires step/slalom/circle evidence PASS')

    return {
        'generated_at': datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'source_acceptance': str(acceptance.resolve()),
        'source_acceptance_sha256': sha256(acceptance),
        'review_required': True,
        'automatic_runtime_write': False,
        'automatic_certification_promotion': False,
        'proposals': proposals,
        'blocked': blocked,
        'precision_global': 'DEFERRED_NO_RTK'}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--acceptance', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    report = json.loads(args.acceptance.read_text(encoding='utf-8'))
    proposal = build_proposal(report, args.acceptance)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(yaml.safe_dump(proposal, sort_keys=False), encoding='utf-8')
    md = args.out.with_suffix('.md')
    lines = ['# Stage-2 Calibration Proposal', '',
             '> REVIEW ONLY. This file never edits runtime YAML or certification flags.', '',
             f"Source: `{proposal['source_acceptance']}`", '', '## Proposed groups', '']
    for name in proposal['proposals']:
        lines.append(f'- **{name}**')
    lines += ['', '## Blocked / still requires physical evidence', '']
    lines += [f'- {x}' for x in proposal['blocked']]
    lines += ['', '- Precision-global: `DEFERRED_NO_RTK`', '']
    md.write_text('\n'.join(lines), encoding='utf-8')
    print(json.dumps({'proposal': str(args.out), 'groups': sorted(proposal['proposals']),
                      'blocked': len(proposal['blocked']), 'runtime_modified': False}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
