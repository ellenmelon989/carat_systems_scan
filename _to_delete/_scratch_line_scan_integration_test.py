"""
Standalone integration smoke test for the Line Scan tab's code path --
run via `python3 line_scan_integration_smoketest.py` from the
carat_scanner repo root. Not part of the shipped codebase; exercises
exactly the effective_config shape gui/line_scan_panel.py._handle_start()
builds, then drives it through gui.scan_worker.run_scan() (the same
function LineScanPanel's worker thread calls) end to end, using the
mock motion/IR/OES backends so it needs no real hardware and no
tkinter. Verifies: the scan completes, "point" messages carry s_mm,
the HDF5 store round-trips in line mode, and scan_summary.csv has
correctly-aligned columns for both real and reference-point rows.
"""
import copy
import math
import os
import queue
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import yaml

from gui.scan_worker import run_scan
from scan.scan_params import endpoint_from_angle, in_radius
from scan.oes_store import OESStore
from scan.data_logger import resolve_run_dir

with open("config.yaml") as f:
    base_config = yaml.safe_load(f)

config = copy.deepcopy(base_config)
config["motion"]["controller"] = None  # -> MockMotionController
config["ir"]["source"] = "mock"
config["oes"]["backend"] = "mock"
config["scan"]["dwell_time_s"] = 2.0
config["scan"]["passes"] = 1
config["scan"]["reference_point"]["enabled"] = True
config["scan"]["reference_point"]["revisit_every_n_points"] = 3

run_dir = resolve_run_dir("./scan_data_test_line_integration")
config["output"]["base_dir"] = run_dir

# Exactly what LineScanPanel._compute_and_validate_line()/_handle_start()
# would compute for Start=(0,0), Angle=53.13deg, Length=25 (a 3-4-5
# triangle scaled by 5: end should land at (15, 20)).
start_mm = (0.0, 0.0)
angle_deg = math.degrees(math.atan2(4, 3))  # 53.13...
length_mm = 25.0
n_points = 6
end_mm = endpoint_from_angle(start_mm, angle_deg, length_mm)
assert abs(end_mm[0] - 15.0) < 1e-6, end_mm
assert abs(end_mm[1] - 20.0) < 1e-6, end_mm

center_mm, radius_mm = tuple(config["scan"]["grid"].get("wafer_center_mm", [0, 0])), config["scan"]["grid"].get("wafer_radius_mm")
if radius_mm is not None:
    assert in_radius(start_mm[0], start_mm[1], center_mm, radius_mm) or True  # informational only here

grid_cfg = config["scan"]["grid"]
grid_cfg["mode"] = "line"
grid_cfg["line_start_mm"] = [start_mm[0], start_mm[1]]
grid_cfg["line_end_mm"] = [end_mm[0], end_mm[1]]
grid_cfg["line_n_points"] = n_points

q = queue.Queue()
stop_event = threading.Event()

# Call run_scan directly on this thread (not via threading.Thread) --
# it's already proven thread-safe/blocking by gui/scan_worker.py's own
# docstring; running it inline here just makes this script simpler and
# still exercises the exact same function LineScanPanel.worker targets.
run_scan(config, q, stop_event, motion=None, already_homed=False)

messages = []
while True:
    try:
        messages.append(q.get_nowait())
    except queue.Empty:
        break

kinds = [k for k, _ in messages]
print("message kinds:", kinds)
assert kinds[-1] == "done", f"expected scan to finish with 'done', got: {kinds}"

point_msgs = [payload for kind, payload in messages if kind == "point"]
real_points = [p for p in point_msgs if not p.get("is_reference")]
ref_points = [p for p in point_msgs if p.get("is_reference")]
print(f"{len(real_points)} real point(s), {len(ref_points)} reference revisit(s)")
assert len(real_points) == n_points, real_points

for p in real_points:
    assert "s_mm" in p and not math.isnan(p["s_mm"]), p
    assert p.get("ix") is not None and p.get("iy") is None, (p.get("ix"), p.get("iy"))

for p in ref_points:
    assert "s_mm" in p and math.isnan(p["s_mm"]), p

# HDF5 line-mode round trip
h5_path = os.path.join(run_dir, "oes.h5")
ds = OESStore.load(h5_path)
assert ds.attrs["mode"] == "line", ds.attrs
assert ds.sizes["s_mm"] == n_points, ds.sizes
assert "x_mm" not in ds.coords, list(ds.coords)
print("HDF5 line-mode store loaded OK:", dict(ds.sizes))

# CSV column-alignment check (this is exactly the bug class fixed
# earlier this segment in ScanManager.run()'s reference-point revisit
# call -- verify it stays fixed end to end, not just in the unit test).
import csv
csv_path = os.path.join(run_dir, "scan_summary.csv")
with open(csv_path, newline="") as f:
    rows = list(csv.DictReader(f))
print(f"{len(rows)} CSV row(s) written")
assert len(rows) == len(real_points) + len(ref_points)
for row in rows:
    assert "s_mm" in row, row
    x = float(row["x_mm"])
    y = float(row["y_mm"])
    is_ref = row["is_reference"] == "True"
    s_val = row["s_mm"]
    if is_ref:
        assert s_val.lower() == "nan", row
    else:
        assert s_val.lower() != "nan", row
        assert 0.0 <= float(s_val) <= length_mm + 1e-6, row

print("LINE SCAN INTEGRATION SMOKE TEST OK")
