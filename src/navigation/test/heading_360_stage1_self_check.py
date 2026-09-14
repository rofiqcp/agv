#!/usr/bin/env python3
from __future__ import annotations
import csv, json, math, subprocess, sys, tempfile
from pathlib import Path
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
SOLVER = ROOT / 'tools/yahboom_mag_planar_fit.py'
APPLY = ROOT / 'tools/yahboom_apply_calibration.py'
WEB_JS = ROOT / 'web/static/imu_calibration.js'
INDEX = ROOT / 'web/static/index.html'
SERVER = ROOT / 'web/web_server.cpp'
FIELDS = ['state','segment','yah_mag_x_lsb','yah_mag_y_lsb','yah_mag_z_lsb',
          'neo_mag_x_ut','neo_mag_y_ut','neo_mag_z_ut']

def require(cond, message):
    if not cond:
        raise SystemExit('HEADING_360_STAGE1_SELF_CHECK_FAIL: ' + message)

def build_case(root: Path, direction='CW', closure_offset_deg=0.0, gyro_error_deg=0.0):
    raw=root/'synthetic.csv'; meta=root/'meta.json'; rng=np.random.default_rng(42)
    by=np.array([3100.0,-700.0]); ay=np.array([[1600.0,260.0],[-180.0,1450.0]])
    bn=np.array([8.0,-2.0]); an=np.array([[28.0,4.0],[-3.0,31.0]])
    sign=-1.0 if direction=='CW' else 1.0
    with raw.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=FIELDS);w.writeheader()
        for seg in range(1,10):
            compass=(sign*-45.0*(seg-1)) % 360.0
            if seg==9: compass=(compass+closure_offset_deg)%360.0
            enu=math.pi/2.0-math.radians(compass); u=np.array([math.cos(enu),math.sin(enu)])
            for _ in range(30):
                y=by+ay@u+rng.normal(0.0,2.0,2); n=bn+an@u+rng.normal(0.0,0.03,2)
                w.writerow({'state':f'STATIC_{seg}','segment':seg,
                            'yah_mag_x_lsb':y[0],'yah_mag_y_lsb':y[1],'yah_mag_z_lsb':2800.0,
                            'neo_mag_x_ut':n[0],'neo_mag_y_ut':n[1],'neo_mag_z_ut':-32.0})
    step=sign*45.0
    checks=[{'from_segment':s,'gyro_delta_deg':step} for s in range(1,9)]
    meta.write_text(json.dumps({'expected_rotation':direction,
                                'gyro_total_turn_deg':sign*360.0+gyro_error_deg,
                                'transition_checks':checks}))
    return raw,meta

def run_solver(raw,meta,direction):
    return subprocess.run([sys.executable,str(SOLVER),str(raw),'--meta-json',str(meta),
                           '--direction',direction],text=True,capture_output=True)

def main():
    with tempfile.TemporaryDirectory(prefix='heading360_good_') as td:
        root=Path(td);raw,meta=build_case(root,'CW')
        good=run_solver(raw,meta,'CW')
        require(good.returncode==0,'healthy synthetic dataset did not PASS: '+good.stdout+good.stderr)
        data=yaml.safe_load((root/'heading_360_latest.yaml').read_text())
        h=data['heading_360_calibration']
        require(h['stage1_only'] is True and h['runtime_yaml_written'] is False,'Stage-1 write lock missing')
        require(h['ready_for_stage2'] is True and h['valid'] is True,'healthy data not ready_for_stage2')
        require(h['yahboom']['valid'] is True and h['neo3']['valid'] is True,'dual MAG fit not valid')
        require(h['gyro']['pass'] is True and h['cross_sensor']['pass'] is True,'gyro/cross gate not valid')
        preview=subprocess.run([sys.executable,str(APPLY),'--workspace',str(root),
                                '--calibration',str(root/'heading_360_latest.yaml'),'--propose'],
                               text=True,capture_output=True)
        require(preview.returncode==0,'Stage-1 preview failed: '+preview.stdout+preview.stderr)
        pj=json.loads(preview.stdout)
        require(pj.get('stage1_only') is True and pj.get('apply_locked') is True,'preview apply lock missing')
        require(pj.get('runtime_write') is False and pj.get('yaml_write') is False,'preview claims a write')
        paths={x['path'] for x in pj.get('proposal_items',[])}
        require('mag_heading_fusion.ros__parameters.imu_mag_bias_xy_lsb' in paths,'Yahboom candidate missing')
        require('mag_heading_fusion.ros__parameters.neo3_mag_bias_xy_ut' in paths,'NEO3 candidate missing')
        require(not any('ownership_verified' in x or 'field_qualification_valid' in x for x in paths),
                'Stage-1 must not open ownership/field qualification gates')
        unsafe=subprocess.run([sys.executable,str(APPLY),'--workspace',str(root),
                               '--calibration',str(root/'heading_360_latest.yaml')],
                              text=True,capture_output=True)
        require(unsafe.returncode!=0,'direct Stage-1 apply was not rejected')

    with tempfile.TemporaryDirectory(prefix='heading360_bad_') as td:
        root=Path(td);raw,meta=build_case(root,'CW',closure_offset_deg=12.0,gyro_error_deg=12.0)
        bad=run_solver(raw,meta,'CW')
        require(bad.returncode!=0,'bad 12deg closure unexpectedly PASS')
        h=yaml.safe_load((root/'heading_360_latest.yaml').read_text())['heading_360_calibration']
        require(h['ready_for_stage2'] is False,'bad closure marked ready_for_stage2')
    js=WEB_JS.read_text(); index=INDEX.read_text(); server=SERVER.read_text()
    for token in ('HEADING360_STOPS=9','NORTH REF','APPLY/YAML LOCKED','Generate Stage-1 Preview'):
        require(token in js,f'web wizard contract missing: {token}')
    for token in ('Absolute North 9-Stop Heading Calibration','Start 360° CW','0 / 9'):
        require(token in index,f'index Stage-1 contract missing: {token}')
    for token in ('proposal_registered','apply_locked','Stage-1 heading 360 PASS'):
        require(token in server,f'web server Stage-1 lock missing: {token}')
    print('HEADING_360_STAGE1_SELF_CHECK_PASS')

if __name__=='__main__':
    main()
