#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, os, tempfile
from pathlib import Path
import yaml

FIELDS = {
    'imu_mag_yaw_sign': 'yaw_sign',
    'imu_mag_yaw_offset_rad': 'yaw_offset_rad',
    'imu_mag_bias_xy_lsb': 'bias_xy_lsb',
    'imu_mag_matrix_xy_per_lsb': 'matrix_xy_per_lsb',
    'imu_heading_lut_input_rad': 'heading_lut_input_rad',
    'imu_heading_lut_correction_rad': 'heading_lut_correction_rad',
}
ENABLE_FIELDS = {
    'imu_planar_calibration_enabled': True,
    'imu_heading_lut_enabled': True,
    'enable_imu_mag_heading': True,
}

def atomic_yaml(path: Path, data) -> None:
    fd, tmp = tempfile.mkstemp(prefix=path.name + '.tmp.', dir=path.parent)
    os.close(fd)
    try:
        Path(tmp).write_text(yaml.safe_dump(data, sort_keys=False, width=120))
        yaml.safe_load(Path(tmp).read_text())
        os.replace(tmp, path)
    finally:
        if Path(tmp).exists():
            Path(tmp).unlink()

def load_validated_candidate(workspace: Path, calibration: Path, state: Path):
    if not calibration.is_file():
        raise ValueError('calibration YAML tidak ditemukan')
    data = yaml.safe_load(calibration.read_text()) or {}
    fit = data.get('yahboom_mag_planar_calibration', {})
    validation = fit.get('validation', {})
    passed = (
        fit.get('valid') is True
        and validation.get('pass') is True
        and fit.get('transition_check_pass') is True
        and float(validation.get('rms_error_deg', 999)) < 3
        and float(validation.get('max_abs_error_deg', 999)) < 5
    )
    if not passed:
        raise ValueError('fit belum PASS fail-closed')
    if state.is_file():
        st = json.loads(state.read_text())
        if st.get('mounting_ok') is False or st.get('mounting_state') in ('UPSIDE_DOWN', 'SIDE_MOUNTED', 'TILT_MISMATCH'):
            raise ValueError('mounting mismatch; apply ditolak')
    values = {dst: fit[src] for dst, src in FIELDS.items()}
    values.update(ENABLE_FIELDS)
    evidence = {
        'calibration_yaml': str(calibration),
        'direction': fit.get('direction'),
        'sample_count': fit.get('sample_count'),
        'segment_count': fit.get('segment_count'),
        'transition_check_pass': fit.get('transition_check_pass'),
        'rms_error_deg': validation.get('rms_error_deg'),
        'max_abs_error_deg': validation.get('max_abs_error_deg'),
    }
    return data, values, evidence

def proposal_items(values):
    return [
        {
            'file_key': 'mag_heading',
            'path': f'mag_heading_fusion.ros__parameters.{name}',
            'value': value,
            'generated': True,
        }
        for name, value in values.items()
    ]

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--workspace', default=os.environ.get('AGV_ROOT', str(Path.home() / 'agv')))
    ap.add_argument('--calibration', default='')
    ap.add_argument('--state', default='')
    ap.add_argument('--propose', action='store_true', help='Validate and emit proposal only; never edit config/runtime.')
    args = ap.parse_args()
    ws = Path(args.workspace).expanduser().resolve()
    calibration = Path(args.calibration) if args.calibration else ws / 'calibration/yahboom_mag_planar_latest.yaml'
    state = Path(args.state) if args.state else ws / 'calibration/yahboom_calibration_state.json'
    try:
        evidence_yaml, values, evidence = load_validated_candidate(ws, calibration, state)
    except (ValueError, OSError, json.JSONDecodeError, KeyError, TypeError) as exc:
        print(json.dumps({'ok': False, 'message': str(exc), 'proposal_only': bool(args.propose)}))
        return 2

    if args.propose:
        print(json.dumps({
            'ok': True,
            'message': 'Yahboom calibration PASS; proposal generated, no YAML/runtime write',
            'proposal_only': True,
            'proposal_items': proposal_items(values),
            'evidence': evidence,
        }))
        return 0

    mag = ws / 'src/navigation/config/mag_heading.yaml'
    evidence_path = ws / 'src/navigation/config/yahboom_mag_calibration.yaml'
    model = yaml.safe_load(mag.read_text())
    ros = model['mag_heading_fusion']['ros__parameters']
    for name, value in values.items():
        ros[name] = value
    atomic_yaml(mag, model)
    atomic_yaml(evidence_path, evidence_yaml)
    print(json.dumps({
        'ok': True,
        'message': 'Yahboom calibration applied atomically; mag_heading_fusion restart required',
        'mag_heading_yaml': str(mag),
        'evidence_yaml': str(evidence_path),
        **evidence,
    }))
    return 0

if __name__ == '__main__':
    raise SystemExit(main())
