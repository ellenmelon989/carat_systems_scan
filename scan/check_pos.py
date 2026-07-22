"""
check_pos.py — quick, no-side-effects position readout. Run from the
repo root: `python scan/check_pos.py`.
"""
# --- repo-root import bootstrap -------------------------------------------
# See scan/scan_manager.py's own copy of this comment for the full
# rationale. Also used here to locate config.yaml by absolute path, so
# this still finds it even if invoked from inside scan/ rather than the
# repo root.
import os as _os
import sys as _sys

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)
# ---------------------------------------------------------------------------

import yaml
from motion.motion_controller import get_motion_controller

with open(_os.path.join(_REPO_ROOT, "config.yaml")) as f:
    config = yaml.safe_load(f)

motion = get_motion_controller(config)
print("Position (no home()/resume() called):", motion.get_position())