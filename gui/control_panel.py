"""
gui/control_panel.py

Operator-facing scan parameters plus Start/Abort. Exposes every knob
scan_params.py validates for a run (dwell time, step size, passes),
plus scan order and output directory.

PR2a (2026-09-04): grid range (x_range_mm/y_range_mm/wafer_radius_mm) was
"deliberately NOT an editable field here" before this -- calibration-only,
set by the Calibrate tab or the standalone calibrate_scan_area.py script.
That's being changed on purpose: the "Scan diameter" field below lets the
operator shrink the masked scan area for THIS run only, capped at the
full calibrated wafer diameter -- e.g. a quick smaller-area check scan
without re-jogging edges. Like every other field here, it's folded into
the per-run effective_config override in _handle_start() and never
written back to config.yaml; the calibration itself (config.yaml's
scan.grid.wafer_radius_mm/x_range_mm/y_range_mm) is untouched. "Target
point count" is a convenience calculator on top of that: solves for the
step size that gets AT LEAST that many points within the (possibly
shrunk) diameter and fills in the Step size field above -- step_size_mm
is still the one value effective_config actually carries forward, same
as before this PR.
"""

import copy
import tkinter as tk
from tkinter import ttk, messagebox

from scan.scan_params import (
    DWELL_TIME_DEFAULT_S, DWELL_TIME_MIN_S, DWELL_TIME_MAX_S,
    STEP_SIZE_DEFAULT_MM, STEP_SIZE_MIN_MM, STEP_SIZE_MAX_MM,
    PASSES_DEFAULT, PASSES_MIN, PASSES_MAX,
    SCAN_TIME_WARNING_THRESHOLD_S,
    validate_dwell_time_s, validate_step_size_mm, validate_passes,
    estimate_scan_time_s,
)
from scan.scan_manager import solve_step_size_for_target_points


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

        # PR2a: diameter_var is blank by default -- blank means "use the
        # full calibrated wafer, exactly as before this field existed" (see
        # _handle_start()). Only a value STRICTLY LESS than the full
        # calibrated diameter actually overrides the grid; typing the full
        # diameter back in is equivalent to leaving it blank, not a
        # separately-recomputed (and possibly subtly different, see
        # _handle_start()'s comment) bounding box.
        self.diameter_var = tk.StringVar(value="")
        self.target_points_var = tk.StringVar(value="")
        self.solve_result_var = tk.StringVar(value="")

        row = 0
        ttk.Label(self, text=f"Dwell time (s)  [{DWELL_TIME_MIN_S}-{DWELL_TIME_MAX_S}]").grid(
            row=row, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.dwell_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(self, text=f"Step size (mm)  [{STEP_SIZE_MIN_MM}-{STEP_SIZE_MAX_MM}]").grid(
            row=row, column=0, sticky="w")
        ttk.Entry(self, textvariable=self.step_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        # PR2a: point-count-driven geometry, as an alternative to typing
        # step size directly above. Diameter is capped at the calibrated
        # wafer (see _handle_start()) -- blank means "don't shrink,
        # nothing new is happening." Disabled entirely if nothing is
        # calibrated yet (no wafer_center_mm/wafer_radius_mm to cap
        # against or solve within).
        center_mm, radius_mm = self._calibrated_center_and_radius()
        calibrated = radius_mm is not None
        diameter_state = "normal" if calibrated else "disabled"
        full_diameter_label = f"≤ {2 * radius_mm:.4g} mm calibrated" if calibrated else "not calibrated yet"

        ttk.Label(self, text=f"Scan diameter (mm)  [{full_diameter_label}]").grid(
            row=row, column=0, sticky="w")
        self.diameter_entry = ttk.Entry(
            self, textvariable=self.diameter_var, width=10, state=diameter_state)
        self.diameter_entry.grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(self, text="Target point count").grid(row=row, column=0, sticky="w")
        self.target_points_entry = ttk.Entry(
            self, textvariable=self.target_points_var, width=10, state=diameter_state)
        self.target_points_entry.grid(row=row, column=1, sticky="w")
        self.solve_btn = ttk.Button(
            self, text="Solve step size", command=self._handle_solve_step_size,
            state=diameter_state)
        self.solve_btn.grid(row=row, column=2, sticky="w", padx=(6, 0))
        row += 1

        ttk.Label(self, textvariable=self.solve_result_var, foreground="#555555").grid(
            row=row, column=0, columnspan=3, sticky="w")
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

    def _calibrated_center_and_radius(self):
        """
        (wafer_center_mm, wafer_radius_mm) from the base config, or
        (None, None) if nothing's been calibrated yet -- same "absent
        means no mask" convention generate_grid() already uses. Reads
        self.base_config fresh every call (not cached) so it stays
        correct across load_config() hand-offs from the Calibrate tab.
        """
        grid_cfg = self.base_config["scan"]["grid"]
        radius_mm = grid_cfg.get("wafer_radius_mm")
        if radius_mm is None:
            return None, None
        center_mm = grid_cfg.get("wafer_center_mm", [0.0, 0.0])
        return tuple(center_mm), float(radius_mm)

    def _handle_solve_step_size(self):
        center_mm, full_radius_mm = self._calibrated_center_and_radius()
        if full_radius_mm is None:
            messagebox.showerror("Not calibrated", "No calibrated wafer center/radius yet.")
            return

        diameter_text = self.diameter_var.get().strip()
        radius_mm = full_radius_mm
        if diameter_text:
            try:
                diameter_mm = float(diameter_text)
            except ValueError:
                messagebox.showerror("Invalid diameter", f"'{diameter_text}' is not a number.")
                return
            if not (0 < diameter_mm <= 2 * full_radius_mm + 1e-6):
                messagebox.showerror(
                    "Invalid diameter",
                    f"Scan diameter must be > 0 and <= the calibrated "
                    f"{2 * full_radius_mm:.4g} mm wafer diameter.",
                )
                return
            radius_mm = diameter_mm / 2.0

        try:
            target_n = int(self.target_points_var.get().strip())
            if target_n < 1:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid point count", "Target point count must be a positive integer.")
            return

        step_size_mm, n_points, achieved = solve_step_size_for_target_points(
            center_mm, radius_mm, target_n)
        self.step_var.set(f"{step_size_mm:.4f}")

        settle_time_s = self.base_config["scan"].get("settle_time_s", 0.0)
        dwell_time_s = self.base_config["scan"].get("dwell_time_s", DWELL_TIME_DEFAULT_S)
        passes = self.base_config["scan"].get("passes", PASSES_DEFAULT)
        est_s = estimate_scan_time_s(n_points, 1, dwell_time_s, settle_time_s, passes)

        if not achieved:
            self.solve_result_var.set(
                f"Only {n_points} points reachable at this diameter (target was {target_n}) "
                f"-- step size floored at {step_size_mm:.4f} mm.")
            messagebox.showwarning(
                "Target point count not reachable",
                f"Even the finest allowed step size ({step_size_mm:.4f} mm) only reaches "
                f"{n_points} points within this diameter, short of the {target_n} requested. "
                "Increase the diameter or lower the target count.",
            )
        else:
            self.solve_result_var.set(
                f"Step size {step_size_mm:.4f} mm -> {n_points} points, "
                f"~{est_s / 60:.1f} min estimated (passes={passes}).")

        if est_s > SCAN_TIME_WARNING_THRESHOLD_S:
            messagebox.showwarning(
                "Scan time estimate",
                f"Estimated scan time is {est_s / 60:.1f} min, over the "
                f"{SCAN_TIME_WARNING_THRESHOLD_S / 60:.0f}-min target.\n\n"
                "Consider a larger step size (fewer target points), a "
                "smaller diameter, shorter dwell time, or fewer passes.",
            )

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

        # PR2a: only override the grid range/mask when a diameter was
        # actually typed in AND it's strictly smaller than the full
        # calibrated wafer -- see the diameter_var comment in __init__.
        # Blank, or exactly the full diameter, leaves x_range_mm/
        # y_range_mm/wafer_radius_mm exactly as calibrated: those come
        # from the real 4-edge jog (compute_area()), which isn't
        # necessarily a perfectly symmetric box around wafer_center_mm at
        # radius wafer_radius_mm (wafer_radius_mm can be the larger of
        # two DIFFERENT measurements -- see compute_radius_mm()'s
        # docstring), so recomputing a symmetric box even when no real
        # shrink was requested could silently change what a "no changes
        # made" scan actually measures. Only build the new tight box when
        # a real shrink is intended.
        diameter_text = self.diameter_var.get().strip()
        if diameter_text:
            center_mm, full_radius_mm = self._calibrated_center_and_radius()
            if full_radius_mm is None:
                messagebox.showerror("Not calibrated", "No calibrated wafer center/radius yet.")
                return
            try:
                diameter_mm = float(diameter_text)
            except ValueError:
                messagebox.showerror("Invalid diameter", f"'{diameter_text}' is not a number.")
                return
            full_diameter_mm = 2 * full_radius_mm
            if not (0 < diameter_mm <= full_diameter_mm + 1e-6):
                messagebox.showerror(
                    "Invalid diameter",
                    f"Scan diameter must be > 0 and <= the calibrated "
                    f"{full_diameter_mm:.4g} mm wafer diameter.",
                )
                return
            if diameter_mm < full_diameter_mm - 1e-6:
                radius_mm = diameter_mm / 2.0
                cx, cy = center_mm
                grid_cfg = effective_config["scan"]["grid"]
                grid_cfg["wafer_radius_mm"] = radius_mm
                grid_cfg["wafer_center_mm"] = [cx, cy]
                grid_cfg["x_range_mm"] = [cx - radius_mm, cx + radius_mm]
                grid_cfg["y_range_mm"] = [cy - radius_mm, cy + radius_mm]

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

        # PR2a: a fresh calibration hand-off changes what the full wafer
        # diameter even IS -- clear any diameter/point-count typed against
        # the OLD calibration rather than silently capping/solving against
        # a now-stale wafer_radius_mm, and re-enable the fields if this is
        # the first calibration of the session.
        self.diameter_var.set("")
        self.target_points_var.set("")
        self.solve_result_var.set("")
        _, radius_mm = self._calibrated_center_and_radius()
        new_state = "normal" if radius_mm is not None else "disabled"
        self.diameter_entry.config(state=new_state)
        self.target_points_entry.config(state=new_state)
        self.solve_btn.config(state=new_state)

    def _handle_abort(self):
        self.abort_btn.config(state="disabled")
        self.on_abort()

    def set_running(self, running: bool):
        self.start_btn.config(state="disabled" if running else "normal")
        self.abort_btn.config(state="normal" if running else "disabled")
