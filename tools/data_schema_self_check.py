#!/usr/bin/env python3
from pathlib import Path
import ast
root=Path('/home/sirobo/agv')
for p in (root/'tools/data.py', root/'F4gateway/tools/data.py', root/'tools/data_common.py'):
    ast.parse(p.read_text(), filename=str(p))
common=(root/'tools/data_common.py').read_text()
ros=(root/'tools/data.py').read_text(); direct=(root/'F4gateway/tools/data.py').read_text()
for token in ('UNIFIED_FIELDS','dronecan_gnss_us','yaw_neo3_deg','yaw_yahboom_deg','yaw_inertial_deg','imu_acc_raw_x','raw_message_count'):
    assert token in common, token
for corpus in (ros,direct):
    assert 'UNIFIED_FIELDS' in corpus
    assert 'summary.csv' in corpus and 'raw.jsonl' in corpus and 'metadata.json' in corpus
assert 'DIRECT_SERIAL' in direct and '/dev/ttyACM0' in direct and '/dev/ttyUSB0' in direct
assert 'ROS_TOPICS' in ros and 'get_topic_names_and_types' in ros
cm=(root/'src/navigation/CMakeLists.txt').read_text()
assert '../../tools/data.py' in cm and 'RENAME data' in cm
assert not (root/'tools/imu_mag_yaw_logger.py').exists()
print('AGV_UNIFIED_DATA_SCHEMA_SELF_CHECK_PASS')
