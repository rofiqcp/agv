#!/usr/bin/env python3
from pathlib import Path
import sys, yaml
ROOT=Path(__file__).resolve().parents[1]
gnss=(ROOT/'src/gnss_node.cpp').read_text()
loc=(ROOT/'src/localization_core.cpp').read_text()
nav=(ROOT/'src/navigation_core.cpp').read_text()
gui=(ROOT/'gui/agv_gui.cpp').read_text()+(ROOT/'gui/modules/system_and_gnss_pages.cpp').read_text()+(ROOT/'gui/agv_gui_specs.hpp').read_text()
gcfg=yaml.safe_load((ROOT/'config/gnss.yaml').read_text())['data_cuav_node']['ros__parameters']
lcfg=yaml.safe_load((ROOT/'config/localization_cpp.yaml').read_text())['localization_core']['ros__parameters']
ekf=yaml.safe_load((ROOT/'config/ekf.yaml').read_text())
checks={
 'raw horizontal 2D display': 'last_ubx_fix_type_ >= 2u' in gnss and 'raw_horizontal_fix_good' in gnss,
 'strict gnssFixOK retained': 'quality_.gnss_fix_ok || quality_.fix_metric < 3.0' in loc,
 'gnssFixOK telemetry': 'gnss_fix_ok' in loc and 'gnss_fix_ok' in gui,
 'raw Doppler independent of strict fix': 'Publish raw Doppler velocity whenever the receiver epoch is fresh' in gnss,
 'provisional TF gate': 'provisionalQualityPassesUnlocked' in loc and 'PROVISIONAL_DISPLAY' in loc,
 'provisional cannot be motion authority': 'autonomous motion remains CLOSED' in loc and 'strictQualityHeldUnlocked()' in loc,
 'automotive 10Hz': gcfg.get('navigation_rate_hz')==10.0 and gcfg.get('dynamic_model')=='automotive' and '0x20110021u' in gnss,
 'ekf load reduced': ekf['ekf_filter_node_odom']['ros__parameters']['frequency']==20.0 and ekf['ekf_filter_node_map']['ros__parameters']['frequency']==10.0,
 'HUD gnss yaw vx vyaw': all(x in nav for x in ['GNSS yaw','gnss_base_vx','gnss_vyaw']),
 'GUI live gnss': all(x in gui for x in ['gnssFixOK','GNSS heading: COG yaw / derived vyaw','Receiver rate','Dynamic model']),
 'yaml provisional thresholds': lcfg.get('allow_provisional_map_display') is True and lcfg.get('provisional_min_satellites')==3,
}
failed=[k for k,v in checks.items() if not v]
if failed:
 print('FAIL:', ', '.join(failed), file=sys.stderr); raise SystemExit(1)
print(f'PASS gnss provisional TF/HUD safety contract {len(checks)}/{len(checks)}')
