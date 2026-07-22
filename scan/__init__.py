"""
scan/ — the precision, absolute-position scan path: fixed grid
(scan_manager.py + scan_params.py), calibration (calibrate_scan_area.py),
data output (data_logger.py, oes_store.py), and post-scan plotting
(map_plotter.py). Trusts commanded position (steps_per_mm-derived) as
physical position, unlike adaptive_scan/ — see that package's docstring
and docs/adaptive_scan_spec.md for why the two are kept separate.
"""
