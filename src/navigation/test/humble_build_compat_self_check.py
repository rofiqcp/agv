#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
mppi = (ROOT / 'src' / 'mppi_closed_loop_supervisor.cpp').read_text()
loc = (ROOT / 'src' / 'localization_core.cpp').read_text()
all_cpp = '\n'.join(p.read_text(errors='ignore') for p in list(ROOT.rglob('*.cpp')) + list(ROOT.rglob('*.hpp')))

assert '.to_msg()' not in all_cpp, 'ROS 2 Humble incompatible rclcpp::Time::to_msg() remains'
assert 'declare_parameter<int>("min_qualification_samples"' not in mppi
assert 'declare_parameter<std::int64_t>("min_qualification_samples", 60)' in mppi
assert 'std::clamp<std::int64_t>' in mppi
assert 'static_cast<builtin_interfaces::msg::Time>(now())' in loc
print('PASS humble_build_compat_self_check')
