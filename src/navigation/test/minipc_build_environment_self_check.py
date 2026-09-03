#!/usr/bin/env python3
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
WS=ROOT.parent.parent
cm=ROOT/'CMakeLists.txt'; pkg=ROOT/'package.xml'; shim=ROOT/'include/serial/serial.h'
s=cm.read_text(); p=pkg.read_text(); h=shim.read_text()
setup=(WS/'src/setup_minipc_cpu_yolopv2.sh').read_text()
preflight=(ROOT/'tools/minipc_nav2_preflight.py').read_text()
assert 'Python3_EXECUTABLE "/usr/bin/python3"' in s
assert 'find_package(serial REQUIRED)' not in s
assert '<depend>serial</depend>' not in p
for token in ['termios.h','FIONREAD','cfmakeraw','B115200','class Serial']:
    assert token in h, token
for token in ['unset AMENT_PREFIX_PATH', 'unset CMAKE_PREFIX_PATH', 'unset COLCON_PREFIX_PATH', 'Timestamp future dinormalisasi', 'models/yolopv2.pt', 'colcon test --packages-select esc perception navigation', 'minipc_nav2_preflight.py --workspace .']:
    assert token in setup, token
for token in ['os.access(target, os.R_OK | os.W_OK)', '--require-hardware', '--require-esc']:
    assert token in preflight, token
print('PASS MiniPC build environment: clean env + timestamp guard + model + tests + serial RW preflight')
