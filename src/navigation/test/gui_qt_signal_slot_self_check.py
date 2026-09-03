#!/usr/bin/env python3
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
page = (root / 'gui/modules/esc_map_overview_pages.cpp').read_text(encoding='utf-8')
errors = []

# Regression for Qt5 compile-time signal/slot mismatch: QPushButton::clicked(bool)
# must not be connected directly to RosBridge::cancelNavigation when that method
# has a defaulted/non-bool argument in some revisions. A lambda is stable across
# the zero-argument and default-argument RosBridge implementations.
if 'connect(cancel,&QPushButton::clicked,ros_,&RosBridge::cancelNavigation)' in page:
    errors.append('direct clicked(bool) -> RosBridge::cancelNavigation pointer connect reintroduced')
if 'connect(cancel,&QPushButton::clicked,this,[this](bool)' not in page:
    errors.append('cancel button must use an explicit bool-consuming lambda')
if 'if(ros_)ros_->cancelNavigation();' not in page:
    errors.append('cancel lambda no longer null-checks RosBridge before cancelNavigation')

if errors:
    print('GUI QT SIGNAL/SLOT SELF-CHECK: FAIL')
    for e in errors:
        print(' -', e)
    raise SystemExit(1)
print('GUI QT SIGNAL/SLOT SELF-CHECK: PASS')
