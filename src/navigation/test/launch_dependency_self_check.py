#!/usr/bin/env python3
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
launch_text = '\n'.join(p.read_text(encoding='utf-8') for p in (root / 'launch').glob('*.py'))
used = set(re.findall(r"package\s*=\s*['\"]([^'\"]+)['\"]", launch_text))
xml_root = ET.parse(root / 'package.xml').getroot()
declared = {e.text.strip() for e in xml_root if e.tag.endswith('depend') and e.text}

# Local/optional packages are intentionally not forced here. Everything else
# referenced directly by launch files must be declared so rosdep can provision
# a clean Mini-PC installation deterministically.
allowed_local_optional = {'navigation', 'perception'}
missing = sorted(p for p in used if p not in declared and p not in allowed_local_optional)
if missing:
    print('LAUNCH DEPENDENCY SELF-CHECK: FAIL')
    print('missing package.xml dependencies:', ', '.join(missing))
    raise SystemExit(1)
print('LAUNCH DEPENDENCY SELF-CHECK: PASS')
