#!/usr/bin/env python3
"""Field logger/calibrator: raw RIGHT eRPM versus raw u-blox GNSS ground speed."""
from __future__ import annotations
import os, sys, shlex
from pathlib import Path

ROOT = Path('/home/sirobo/agv')

def _ensure_ros_python() -> None:
    try:
        import rclpy  # noqa: F401
        return
    except ModuleNotFoundError:
        pass
    if os.environ.get('_SPEEDTEST_ROS_SOURCED') == '1':
        raise
    argv = shlex.join([sys.executable, str(Path(__file__).resolve()), *sys.argv[1:]])
    cmd = (
        'source /opt/ros/humble/setup.bash && '
        f'source {shlex.quote(str(ROOT / "install/setup.bash"))} && '
        'export _SPEEDTEST_ROS_SOURCED=1; '
        'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}; '
        'export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}; '
        f'exec {argv}'
    )
    os.execv('/bin/bash', ['bash', '-lc', cmd])

_ensure_ros_python()

import argparse, csv, json, math, signal, statistics, subprocess, time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float64, Float64MultiArray, String

@dataclass
class Latest:
    value: object = None
    rx_mono: float = -1.0

class SpeedCalibrationNode(Node):
    def __init__(self, csv_writer, csv_file, args):
        super().__init__('speedtest_calibrator')
        self.w, self.f, self.args = csv_writer, csv_file, args
        self.telemetry, self.vel, self.steer = Latest(), Latest(), Latest()
        self.rows, self.valid_rows = 0, []
        self.last_print = 0.0
        self.create_subscription(String, '/esc/foc/telemetry', self.on_telemetry, qos_profile_sensor_data)
        self.create_subscription(TwistWithCovarianceStamped, '/gnss/vel', self.on_vel, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/gnss/quality', self.on_quality, qos_profile_sensor_data)
        self.create_subscription(Float64, '/esc/steering_actual_rad', self.on_steer, qos_profile_sensor_data)

    def on_telemetry(self, msg):
        try:
            d = json.loads(msg.data)
            r = d.get('right', {})
            self.telemetry = Latest({
                'erpm': float(r.get('erpm', math.nan)),
                'fault': int(r.get('fault', -1)),
                'right_age_ms': float(d.get('right_age_ms', math.nan)),
            }, time.monotonic())
        except Exception:
            return

    def on_vel(self, msg):
        x, y = float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y)
        cov = msg.twist.covariance
        self.vel = Latest({
            'speed': math.hypot(x, y), 've': x, 'vn': y,
            'sigma': math.sqrt(max(0.0, max(float(cov[0]), float(cov[7])))),
        }, time.monotonic())

    def on_steer(self, msg):
        self.steer = Latest(float(msg.data), time.monotonic())

    def on_quality(self, msg):
        q = list(msg.data)
        now_mono, now_wall = time.monotonic(), time.time()
        def qv(i, default=math.nan):
            return float(q[i]) if len(q) > i else default
        sat, dop, hacc, fix_type = qv(0, 0), qv(1, 99.9), qv(2, 99.9), qv(3, 0)
        sacc, gspeed, meas_age, fix_ok = qv(5, -1), qv(6, -1), qv(23, 99), qv(44, 0)
        erpm = math.nan; fault = -1; esc_age = math.inf; right_age_ms = math.nan
        if self.telemetry.value is not None:
            erpm = self.telemetry.value['erpm']; fault = self.telemetry.value['fault']
            right_age_ms = self.telemetry.value['right_age_ms']
            esc_age = now_mono - self.telemetry.rx_mono
        # Same-epoch UBX NAV-PVT velocity components are carried in quality[11:13].
        # Use these for calibration cross-check so DDS scheduling cannot pair
        # gSpeed with the previous /gnss/vel epoch. The topic copy is diagnostic only.
        ve_q, vn_q = qv(11), qv(12)
        vel_speed = math.hypot(ve_q, vn_q) if math.isfinite(ve_q) and math.isfinite(vn_q) else math.nan
        vel_sigma = self.vel.value['sigma'] if self.vel.value is not None else math.nan
        steer_deg = math.nan; steer_age = math.inf
        if self.steer.value is not None:
            steer_deg = math.degrees(float(self.steer.value)); steer_age = now_mono - self.steer.rx_mono
        reasons = []
        if not math.isfinite(erpm): reasons.append('no_erpm')
        if fault != 0: reasons.append(f'esc_fault_{fault}')
        if esc_age > self.args.max_pair_age: reasons.append('erpm_stale')
        if math.isfinite(right_age_ms) and (right_age_ms < 0 or right_age_ms > self.args.max_esc_feedback_age_ms): reasons.append('esc_feedback_stale')
        if fix_ok < 0.5: reasons.append('gnss_fix_not_ok')
        if int(round(fix_type)) != 3: reasons.append(f'fix_type_{int(round(fix_type))}')
        if sat < self.args.min_sat: reasons.append('sat_low')
        if dop > self.args.max_dop: reasons.append('dop_high')
        if hacc <= 0 or hacc > self.args.max_hacc: reasons.append('hacc_high')
        if sacc <= 0 or sacc > self.args.max_sacc: reasons.append('sacc_high')
        if meas_age < 0 or meas_age > self.args.max_gnss_age: reasons.append('gnss_measurement_stale')
        if not math.isfinite(gspeed) or gspeed < self.args.min_speed: reasons.append('speed_low')
        if not math.isfinite(vel_speed): reasons.append('gnss_velocity_vector_missing')
        elif abs(vel_speed - gspeed) > self.args.max_speed_disagreement: reasons.append('gnss_speed_mismatch')
        if steer_age > 0.5 or not math.isfinite(steer_deg): reasons.append('steer_stale')
        elif abs(steer_deg) > self.args.max_steer_deg: reasons.append('not_straight')
        valid = not reasons
        ratio = abs(erpm) / gspeed if valid and gspeed > 1e-6 else math.nan
        row = [
            datetime.fromtimestamp(now_wall).isoformat(timespec='milliseconds'), f'{now_wall:.6f}',
            f'{erpm:.3f}', f'{gspeed:.6f}', f'{vel_speed:.6f}', f'{ratio:.6f}',
            f'{sat:.0f}', f'{dop:.3f}', f'{hacc:.3f}', f'{sacc:.3f}', f'{meas_age:.3f}',
            f'{esc_age:.3f}', f'{right_age_ms:.1f}', f'{vel_sigma:.4f}', f'{steer_deg:.3f}',
            str(fault), '1' if valid else '0', ';'.join(reasons) if reasons else 'OK'
        ]
        self.w.writerow(row); self.f.flush(); self.rows += 1
        if valid:
            self.valid_rows.append((now_wall, gspeed, erpm, ratio, steer_deg, sacc))
        if now_mono - self.last_print >= self.args.print_period:
            self.last_print = now_mono
            state = 'VALID' if valid else ('WAIT:' + ','.join(reasons[:2]))
            print(f'ERPM {erpm:8.0f} | GNSS {gspeed:6.3f} m/s | K {ratio:8.1f} ERPM/(m/s) | sat {sat:2.0f} sAcc {sacc:5.3f} | {state}', flush=True)


def robust_calibration(rows, args):
    if len(rows) < args.min_valid_samples:
        return None, f'valid samples only {len(rows)} < {args.min_valid_samples}'
    stable = []
    prev = None
    for r in rows:
        t, speed, erpm, ratio, steer, sacc = r
        if prev is not None:
            dt = max(1e-3, t - prev[0])
            accel = abs(speed - prev[1]) / dt
            erpm_rate = abs(abs(erpm) - abs(prev[2])) / dt
            if accel <= args.max_accel and erpm_rate <= args.max_erpm_rate:
                stable.append(r)
        prev = r
    pool = stable if len(stable) >= args.min_valid_samples else rows
    ratios = [r[3] for r in pool if math.isfinite(r[3])]
    med = statistics.median(ratios)
    absdev = [abs(x - med) for x in ratios]
    mad = statistics.median(absdev) if absdev else 0.0
    # Give the robust scale a small physical floor (0.5% of K). Without this,
    # nearly quantized eRPM can make MAD unrealistically tiny and keep only one
    # side of otherwise symmetric measurement noise.
    robust_sigma = max(1.4826 * mad, 0.005 * abs(med))
    if robust_sigma > 1e-9:
        filt = [r for r in pool if abs(r[3] - med) <= args.mad_sigma * robust_sigma]
    else:
        filt = pool[:]
    if len(filt) < max(5, args.min_valid_samples // 2):
        filt = pool[:]
    sx2 = sum(r[1] * r[1] for r in filt)
    k = sum(r[1] * abs(r[2]) for r in filt) / sx2 if sx2 > 1e-12 else math.nan
    residuals = [abs(r[2]) - k * r[1] for r in filt]
    rmse = math.sqrt(sum(e*e for e in residuals) / len(residuals))
    mean_y = sum(abs(r[2]) for r in filt) / len(filt)
    sst = sum((abs(r[2]) - mean_y)**2 for r in filt)
    sse = sum(e*e for e in residuals)
    r2 = 1.0 - sse/sst if sst > 1e-12 else math.nan
    def dir_fit(sign):
        d = [r for r in filt if (r[2] > 0 if sign > 0 else r[2] < 0)]
        if len(d) < 5: return {'n': len(d), 'k': None}
        den = sum(r[1]**2 for r in d)
        return {'n': len(d), 'k': sum(r[1]*abs(r[2]) for r in d)/den if den else None}
    return {
        'erpm_per_mps': k, 'mps_per_erpm': (1.0/k if k > 0 else None),
        'n_all_valid': len(rows), 'n_stable_pool': len(pool), 'n_used': len(filt),
        'ratio_median': statistics.median([r[3] for r in filt]),
        'ratio_mean': statistics.fmean([r[3] for r in filt]),
        'ratio_stdev': statistics.stdev([r[3] for r in filt]) if len(filt) > 1 else 0.0,
        'fit_rmse_erpm': rmse, 'fit_r2': r2,
        'speed_min_mps': min(r[1] for r in filt), 'speed_max_mps': max(r[1] for r in filt),
        'forward': dir_fit(+1), 'reverse': dir_fit(-1),
    }, None


def parse_args():
    p = argparse.ArgumentParser(description='Log RIGHT eRPM vs raw GNSS ground speed and calibrate ERPM per m/s on Ctrl+C.')
    p.add_argument('--no-launch', action='store_true', help='Do not start autonomous.launch.py (debug only).')
    p.add_argument('--duration', type=float, default=0.0, help='Optional auto-stop seconds; 0 means Ctrl+C.')
    p.add_argument('--min-speed', type=float, default=0.30)
    p.add_argument('--min-sat', type=int, default=8)
    p.add_argument('--max-dop', type=float, default=2.0)
    p.add_argument('--max-hacc', type=float, default=2.5)
    p.add_argument('--max-sacc', type=float, default=0.25)
    p.add_argument('--max-gnss-age', type=float, default=0.35)
    p.add_argument('--max-pair-age', type=float, default=0.20)
    p.add_argument('--max-esc-feedback-age-ms', type=float, default=250.0)
    p.add_argument('--max-speed-disagreement', type=float, default=0.10)
    p.add_argument('--max-steer-deg', type=float, default=3.0)
    p.add_argument('--max-accel', type=float, default=0.45)
    p.add_argument('--max-erpm-rate', type=float, default=2500.0)
    p.add_argument('--mad-sigma', type=float, default=3.5)
    p.add_argument('--min-valid-samples', type=int, default=30)
    p.add_argument('--print-period', type=float, default=0.20)
    return p.parse_args()


def main():
    args = parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = ROOT / 'tools' / 'record' / stamp
    out_dir.mkdir(parents=True, exist_ok=True)
    (ROOT / 'log').mkdir(parents=True, exist_ok=True)
    csv_path, log_path = out_dir / 'speedtest.csv', ROOT / 'log' / f'{stamp}.txt'
    launch_proc: Optional[subprocess.Popen] = None
    if not args.no_launch:
        launch_cmd = (
            f'cd {shlex.quote(str(ROOT))} && source /opt/ros/humble/setup.bash && '
            f'source {shlex.quote(str(ROOT / "install/setup.bash"))} && '
            'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}; '
            f'ros2 launch navigation autonomous.launch.py 2>&1 | tee {shlex.quote(str(log_path))} >/dev/null'
        )
        # Keep the full ROS launch output in the tee log, but never mix it into
        # the operator calibration display. Popen redirection is a second guard
        # for shell/setup diagnostics outside the ros2|tee pipeline.
        launch_proc = subprocess.Popen(
            ['/bin/bash', '-lc', launch_cmd], preexec_fn=os.setsid,
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    print(f'[speedtest] CSV: {csv_path}', flush=True)
    print('[speedtest] Ctrl+C = stop, save, and calculate calibration.', flush=True)
    headers = ['time_iso','unix_s','right_erpm','gnss_gspeed_mps','gnss_vel_norm_mps','ratio_erpm_per_mps',
               'sat','dop','hacc_m','sacc_mps','gnss_measurement_age_s','erpm_pair_age_s','esc_right_age_ms',
               'gnss_vel_sigma_mps','steering_deg','esc_fault','valid','reject_reason']
    rclpy.init()
    started = time.monotonic()
    stop_requested = {'value': False, 'signal': None}
    def _request_stop(signum, _frame):
        stop_requested['value'] = True
        stop_requested['signal'] = signum
        print(f'\n[speedtest] signal {signum} received. Finalizing...', flush=True)
    signal.signal(signal.SIGTERM, _request_stop)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, _request_stop)
    with csv_path.open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f); w.writerow(headers); f.flush()
        node = SpeedCalibrationNode(w, f, args)
        try:
            while rclpy.ok() and not stop_requested['value']:
                rclpy.spin_once(node, timeout_sec=0.10)
                if launch_proc is not None and launch_proc.poll() is not None:
                    print(f'\n[speedtest] autonomous launch exited code={launch_proc.returncode}', flush=True)
                    break
                if args.duration > 0 and time.monotonic() - started >= args.duration:
                    break
        except KeyboardInterrupt:
            print('\n[speedtest] Ctrl+C received. Finalizing...', flush=True)
        finally:
            result, err = robust_calibration(node.valid_rows, args)
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
    if launch_proc is not None and launch_proc.poll() is None:
        try:
            os.killpg(launch_proc.pid, signal.SIGINT)
            launch_proc.wait(timeout=8.0)
        except Exception:
            try: os.killpg(launch_proc.pid, signal.SIGTERM)
            except Exception: pass
    summary_path = out_dir / 'calibration_result.yaml'
    payload = {
        'created_at': datetime.now().isoformat(timespec='seconds'),
        'csv': str(csv_path), 'launch_log': str(log_path),
        'total_rows': node.rows, 'valid_rows': len(node.valid_rows),
        'filters': vars(args), 'calibration': result, 'error': err,
    }
    summary_path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
    print('\n' + '='*72)
    print('SPEED CALIBRATION RESULT')
    print('='*72)
    print(f'Total epochs : {node.rows}')
    print(f'Valid epochs : {len(node.valid_rows)}')
    if result:
        print(f"ERPM per m/s : {result['erpm_per_mps']:.6f}")
        print(f"m/s per ERPM : {result['mps_per_erpm']:.10f}")
        print(f"Used / stable: {result['n_used']} / {result['n_stable_pool']}")
        print(f"Speed range  : {result['speed_min_mps']:.3f} .. {result['speed_max_mps']:.3f} m/s")
        print(f"Fit RMSE     : {result['fit_rmse_erpm']:.2f} ERPM")
        print(f"Fit R^2      : {result['fit_r2']:.5f}")
        print(f"Forward K    : {result['forward']['k']} (n={result['forward']['n']})")
        print(f"Reverse K    : {result['reverse']['k']} (n={result['reverse']['n']})")
        print(f"\nYAML target  : vehicle.ros__parameters.drive_erpm_per_mps = {result['erpm_per_mps']:.6f}")
    else:
        print(f'Calibration NOT VALID: {err}')
    print(f'CSV          : {csv_path}')
    print(f'Result YAML  : {summary_path}')
    print(f'Launch log   : {log_path}')
    print('='*72, flush=True)
    return 0 if result else 2

if __name__ == '__main__':
    raise SystemExit(main())
