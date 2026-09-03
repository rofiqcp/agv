#!/usr/bin/env python3
from gui_source_helper import read_gui_source
from pathlib import Path
import re

pkg = Path(__file__).resolve().parents[1]
cpp = read_gui_source(pkg)
spec = (pkg / 'gui/agv_gui_specs.hpp').read_text(encoding='utf-8')
cmake = (pkg / 'CMakeLists.txt').read_text(encoding='utf-8')
package = (pkg / 'package.xml').read_text(encoding='utf-8')
launch = (pkg / 'launch/gui.launch.py').read_text(encoding='utf-8')

checks = []
def check(name, cond):
    checks.append((name, bool(cond)))
    print(('PASS' if cond else 'FAIL'), name)

check('native C++ GUI source exists', (pkg / 'gui/agv_gui.cpp').is_file())
check('legacy agv_gui.py removed', not (pkg / 'gui/agv_gui.py').exists())
check('CMake builds agv_gui', 'add_executable(agv_gui gui/agv_gui.cpp' in cmake)
check('CMake AUTOMOC enabled', 'set(CMAKE_AUTOMOC ON)' in cmake)
check('Qt5 Widgets+Network linked', 'Qt5::Widgets Qt5::Network yaml-cpp' in cmake)
check('rclpy runtime dependency removed', '<exec_depend>rclpy</exec_depend>' not in package)
check('PyQt5 runtime dependency removed', 'python3-pyqt5' not in package)
check('rclcpp bridge exists', 'class RosBridge:public QObject' in cpp and 'MultiThreadedExecutor' in cpp)
check('ROS subscriptions retain lifetime', 'subscriptions_.push_back(subscription)' in cpp)
check('atomic YAML QSaveFile', 'QSaveFile' in cpp and 'class YamlStore' in cpp)
check('runtime parameter verification', 'verifyRuntime()' in cpp and 'getParameters' in cpp)
check('report numeric summary', 'numericSummary' in cpp and '"rmse"' in cpp)
check('rosbag support', 'ros2' in cpp and 'bagTopics()' in cpp)
check('map + OSM support', 'QNetworkAccessManager' in cpp and 'drawOsm' in cpp)
check('steering calibration', 'class SteeringCalibrationPage' in cpp and 'CAPTURE KIRI' in cpp and 'CAPTURE LURUS' in cpp and 'CAPTURE KANAN' in cpp)
check('IMU calibration', 'class ImuCalibrationPage' in cpp)
check('GNSS calibration/fusion', 'class GnssCalibrationPage' in cpp and 'gnss_cog_fusion_active' in cpp)
check('camera calibration', 'class CameraCalibrationCanvas' in cpp and 'class CameraPage' in cpp)
check('MPPI tuning', 'class NavigationTuningPage' in cpp and 'mppi_sweep_' in cpp)
check('21 tabs retained (current GUI catalog)', spec.count('tabs.push_back(t);') == 21)
check('378+ settings retained', spec.count('SettingSpec{') >= 378)
check('Qt5-compatible QVariant API', '.metaType().id()' not in cpp and '.userType()' in cpp)
check('launch describes native GUI', 'native C++/Qt AGV operator GUI' in launch)
check('no PyQt/rclpy in C++ GUI', 'PyQt5' not in cpp and 'rclpy' not in cpp)

# Basic lexical bracket check ignoring comments/strings is deliberately conservative.
def balanced(text, a, b):
    depth = 0
    in_s = in_d = False
    esc = False
    i = 0
    while i < len(text):
        c = text[i]
        if not in_s and not in_d and c == '/' and i+1 < len(text) and text[i+1] == '/':
            i = text.find('\n', i)
            if i < 0: break
            continue
        if not in_s and not in_d and c == '/' and i+1 < len(text) and text[i+1] == '*':
            e = text.find('*/', i+2)
            if e < 0: return False
            i = e + 2
            continue
        if esc:
            esc = False
        elif c == '\\' and (in_s or in_d):
            esc = True
        elif c == '"' and not in_s:
            in_d = not in_d
        elif c == "'" and not in_d:
            in_s = not in_s
        elif not in_s and not in_d:
            if c == a: depth += 1
            elif c == b:
                depth -= 1
                if depth < 0: return False
        i += 1
    return depth == 0 and not in_s and not in_d

check('C++ braces balanced', balanced(cpp, '{', '}'))
check('C++ parentheses balanced', balanced(cpp, '(', ')'))

if not all(ok for _, ok in checks):
    raise SystemExit(1)
print(f'PASS GUI C++ migration: {sum(ok for _, ok in checks)}/{len(checks)} checks')
