"""
gui/control_panel.py

Operator-facing scan parameters plus Start/Abort. Exposes every knob
scan_params.py validates for a run (dwell time, step size, passes),
plus scan order and output directory. Grid range (x_range_mm/y_range_mm)
is deliberately NOT an editable field here -- it's calibration-only, set
by the Calibrate tab (gui/calibration_panel.py) or the standalone
calibrate_scan_area.py script, matching the existing "operator sets
step size, range comes from calibration" split already documented in
scan_params.py.
"""

import copy
import tkinter as tk
from tkinter import ttk, messagebox

from scan_params import (
    DWELL_TIME_DEFAULT_S, DWELL_TIME_MIN_S, DWELL_TIME_MAX_S,
    STEP_SIZE_DEFAULT_MM, STEP_SIZE_MIN_MM, STEP_SIZE_MAX_MM,
    PASSES_DEFAULT, PASSES_MIN, PASSES_MAX,
    validate_dwell_time_s, validate_step_size_mm, validate_passes,
)


class ControlPanel(ttk.Frame):
    def __init__(self, parent, config, on_start, on_abort):
        super().__init__(parent, padding=8)
        self.base_config = config
        self.on_start = on_start
        self.on_abort = on_abort

        scan_cfg = config["scan"]
        self.dwell_var = tk.StringVar(value=str(scan_cfg.get("dwell_time_s", DWELL_TIME_DEFAULT_S)))
        self.step_var = tk.StringVar(value=str(scan_cfg["grid"].get("step_size_mm", STEP_SIZE_DEFAULT_MM)))
        self.passes_var = tk.StringVar(value=str(scan_cfg.get("passes", PASSES_DEFAULT)))
        self.order_var = tk.StringVar(value=scan_cfg.get("scan_order", "raster"))
        self.outdir_var = tk.StringVar(value=config["output"].get("base_dir", "./scan_data"))

        row = 0
        ttk.Label(self, text=f"Dwell time (s)  [{DWELL_TIME_MIN_S}-{DWELL_TIME_MAX_S}]").grid(
            row=row, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.dwell_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(self, text=f"Step size (mm)  [{STEP_SIZE_MIN_MM}-{STEP_SIZE_MAX_MM}]").grid(
            row=row, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.step_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(self, text=f"Passes  [{PASSES_MIN}-{PASSES_MAX}]").grid(row=row, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.passes_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(self, text="Scan order").grid(row=row, column=0, sticky="w")
        ttk.Combobox(
            self, textvariable=self.order_var, values=["raster", "serpentine"],
            state="readonly", width=10,
        ).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(self, text="Output dir").grid(row=row, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.outdir_var, width=30).grid(
            row=row, column=1, columnspan=2, sticky="we")
        row += 1

        self.start_btn = ttk.Button(self, text="Start Scan", command=self._handle_start)
        self.start_btn.grid(row=row, column=0, pady=(10, 0), sticky="we")
        self.abort_btn = ttk.Button(self, text="Abort", command=self._handle_abort, state="disabled")
        self.abort_btn.grid(row=row, column=1, pady=(10, 0), sticky="we")

    def _handle_start(self):
        # Validate through the SAME functions scan_manager.py itself uses
        # (ScanManager.__init__ calls validate_dwell_time_s/validate_passes
        # again too) -- this just gives the operator an immediate error
        # dialog instead of a scan that starts and then blows up on
        # ScanManager construction inside the worker thread a moment later.
        try:
            dwell = validate_dwell_time_s(self.dwell_var.get())
            step = validate_step_size_mm(self.step_var.get())
            passes = validate_passes(self.passes_var.get())
        except (ValueError, TypeError) as exc:
            messagebox.showerror("Invalid scan parameters", str(exc))
            return

        effective_config = copy.deepcopy(self.base_config)
        effective_config["scan"]["dwell_time_s"] = dwell
        effective_config["scan"]["grid"]["step_size_mm"] = step
        effective_config["scan"]["passes"] = passes
        effective_config["scan"]["scan_order"] = self.order_var.get()
        effective_config["output"]["base_dir"] = self.outdir_var.get()

        self.on_start(effective_config)

    def load_config(self, config):
        """
        Refresh every field from a freshly-loaded config dict -- called
        after the Calibrate tab writes new values (range, step size,
        dwell time, passes) to config.yaml and hands off to the Scan
        tab, so the operator sees exactly what was just calibrated
        rather than stale values from GUI startup. Does NOT touch
        base_config's identity (still the same dict object app.py
        holds) -- only replaces its contents and the displayed
        StringVars, mirroring how __init__ reads these same keys.
        """
        self.base_config.clear()
        self.base_config.update(copy.deepcopy(config))
        scan_cfg = self.base_config["scan"]
        self.dwell_var.set(str(scan_cfg.get("dwell_time_s", DWELL_TIME_DEFAULT_S)))
        self.step_var.set(str(scan_cfg["grid"].get("step_size_mm", STEP_SIZE_DEFAULT_MM)))
        self.passes_var.set(str(scan_cfg.get("passes", PASSES_DEFAULT)))
        self.order_var.set(scan_cfg.get("scan_order", "raster"))
        self.outdir_var.set(self.base_config["output"].get("base_dir", "./scan_data"))

    def _handle_abort(self):
        self.abort_btn.config(state="disabled")
        self.on_abort()

    def set_running(self, running: bool):
        self.start_btn.config(state="disabled" if running else "normal")
        self.abort_btn.config(state="normal" if running else "disabled")
