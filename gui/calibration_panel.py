"""
gui/calibration_panel.py

GUI wizard for scanner-area calibration -- the on-screen equivalent of
calibrate_scan_area.py's CLI workflow. See that module's docstring for
the full rationale behind each step (fiducial homing instead of a
hard-stop home(), the clearance check, per-jog checkpoints, the
circular wafer mask, etc.) -- this module re-implements only the
JOGGING and CONFIRMATION steps as Tk widgets/dialogs in place of
input()/msvcrt. It reuses calibrate_scan_area.py's pure geometry and
config-writing functions (compute_area, rescale_edges,
compute_radius_mm, recommend_home_steps, write_results) so the math and
the config.yaml patching stay byte-for-byte identical between the CLI
tool and this tab -- the same "preview and execution must derive from
identical logic" principle CONFIG_RANGE_DECIMALS and generate_grid's
mask already enforce elsewhere in this codebase (see scan_params.py
and calibrate_scan_area.py's module docstrings).

Threading: unlike scan_worker.py's ScanManager.run() (a multi-minute
scan run on a background thread), every motion call here is a single
fast relative jog or position read -- cheap enough to call directly
from Tk button/key callbacks on the mainloop thread, same as the CLI
script's blocking loop. Nothing in this module runs on a second
thread, and it never shares the motion object with a running scan --
ownership is handed to the app (see on_calibrated) only once, when the
operator clicks "Proceed to Scan".
"""

import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

import yaml

from motion.motion_controller import get_motion_controller
import scan_params
from calibrate_scan_area import (
    CLEARANCE_CHECK_STEP_MM,
    CONFIG_RANGE_DECIMALS,
    EDGE_ORDER,
    EDGE_PROMPTS,
    JOG_CHECKPOINT_INTERVAL_DEFAULT_MM,
    JOG_CHECKPOINT_INTERVAL_MAX_MM,
    JOG_CHECKPOINT_INTERVAL_MIN_MM,
    JOG_STEP_DEFAULT_MM,
    JOG_STEP_MAX_MM,
    JOG_STEP_MIN_MM,
    compute_area,
    compute_radius_mm,
    recommend_home_steps,
    rescale_edges,
    validate_jog_checkpoint_interval_mm,
    write_results,
)
from scan_manager import generate_grid

# Arrow keysym -> (dx_sign, dy_sign). Matches the msvcrt jog loop's
# convention in calibrate_scan_area.py (Up/Down = Y, Left/Right = X).
ARROW_KEYSYMS = {
    "Up": (0.0, 1.0),
    "Down": (0.0, -1.0),
    "Left": (-1.0, 0.0),
    "Right": (1.0, 0.0),
}


class CalibrationPanel(ttk.Frame):
    """
    config: the App's live config dict -- read (not mutated) for
        motion connection settings and the current (possibly
        placeholder) steps_per_mm. Calibration results only ever reach
        the rest of the app through on_calibrated(), never by silently
        editing this dict in place.
    config_path: path to the config.yaml this will patch (same file
        run_gui.py loaded config from).
    on_calibrated(config, motion, already_homed): called once the
        operator clicks "Proceed to Scan". `config` is config.yaml
        RE-READ from disk after write_results() patched it (so it's
        exactly what a fresh `python scan_manager.py` run would see);
        `motion` is this panel's already-connected, already-homed
        MotionController, handed off so the app doesn't open a second
        hardware connection and re-home it; `already_homed` is always
        True when this fires.
    """

    def __init__(self, parent, config, config_path, on_calibrated):
        super().__init__(parent, padding=8)
        self.config = config
        self.config_path = Path(config_path)
        self.on_calibrated = on_calibrated

        self.motion = None
        self.jog_enabled = False
        self.jog_step_mm = JOG_STEP_DEFAULT_MM
        self.checkpoint_interval_mm = JOG_CHECKPOINT_INTERVAL_DEFAULT_MM
        self._moved_since_checkpoint = 0.0
        self._arrows_bound = False
        self._position_poll_job = None

        self.edges = {}
        self._edge_idx = 0
        self.spmm_result = None
        self.area = None
        self.radius_mm = None
        self.recommended_home_steps = None

        self.step = "idle"

        # ---- header: position readout, jog step size, abort/reset -----
        header = ttk.Frame(self)
        header.grid(row=0, column=0, sticky="we")

        self.pos_var = tk.StringVar(value="Position: --")
        ttk.Label(header, textvariable=self.pos_var).grid(row=0, column=0, sticky="w", padx=(0, 16))

        ttk.Label(header, text="Jog step (mm)").grid(row=0, column=1, sticky="w")
        self.jog_step_var = tk.StringVar(value=f"{JOG_STEP_DEFAULT_MM:.2f}")
        jog_entry = ttk.Entry(header, textvariable=self.jog_step_var, width=6)
        jog_entry.grid(row=0, column=2, sticky="w")
        jog_entry.bind("<Return>", self._apply_jog_step_entry)
        jog_entry.bind("<FocusOut>", self._apply_jog_step_entry)
        ttk.Button(header, text="-", width=2, command=self._halve_jog_step).grid(row=0, column=3)
        ttk.Button(header, text="+", width=2, command=self._double_jog_step).grid(row=0, column=4)

        self.abort_btn = ttk.Button(header, text="Abort / Reset", command=self._handle_abort_click)
        self.abort_btn.grid(row=0, column=5, padx=(16, 0))

        # ---- jog pad (built once, enabled/disabled per step) -----------
        pad = ttk.Frame(self)
        pad.grid(row=1, column=0, pady=(8, 0))
        ttk.Button(pad, text="↑", width=3, command=lambda: self._do_jog(0.0, 1.0)).grid(row=0, column=1)
        ttk.Button(pad, text="←", width=3, command=lambda: self._do_jog(-1.0, 0.0)).grid(row=1, column=0)
        ttk.Button(pad, text="→", width=3, command=lambda: self._do_jog(1.0, 0.0)).grid(row=1, column=2)
        ttk.Button(pad, text="↓", width=3, command=lambda: self._do_jog(0.0, -1.0)).grid(row=2, column=1)
        self.jog_buttons = list(pad.winfo_children())

        # ---- dynamic step area ------------------------------------------
        self.step_frame = ttk.Frame(self, padding=(0, 12, 0, 0))
        self.step_frame.grid(row=2, column=0, sticky="we")

        # ---- log ---------------------------------------------------------
        self.log = ScrolledText(self, height=14, width=64, state="disabled")
        self.log.grid(row=3, column=0, pady=(8, 0), sticky="nsew")

        self.bind("<Destroy>", self._on_destroy)

        self._set_jog_enabled(False)
        self._render_idle()

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log(self, text):
        self.log.config(state="normal")
        self.log.insert(tk.END, text + "\n")
        self.log.see(tk.END)
        self.log.config(state="disabled")

    # ------------------------------------------------------------------
    # Jog step size
    # ------------------------------------------------------------------

    def _halve_jog_step(self):
        self.jog_step_mm = max(JOG_STEP_MIN_MM, self.jog_step_mm / 2)
        self.jog_step_var.set(f"{self.jog_step_mm:.2f}")

    def _double_jog_step(self):
        self.jog_step_mm = min(JOG_STEP_MAX_MM, self.jog_step_mm * 2)
        self.jog_step_var.set(f"{self.jog_step_mm:.2f}")

    def _apply_jog_step_entry(self, event=None):
        try:
            value = float(self.jog_step_var.get())
        except ValueError:
            self.jog_step_var.set(f"{self.jog_step_mm:.2f}")
            return
        value = max(JOG_STEP_MIN_MM, min(JOG_STEP_MAX_MM, value))
        self.jog_step_mm = value
        self.jog_step_var.set(f"{value:.2f}")

    # ------------------------------------------------------------------
    # Jogging (buttons + arrow keys), position polling, checkpoints
    # ------------------------------------------------------------------

    def _set_jog_enabled(self, enabled):
        self.jog_enabled = enabled
        state = "normal" if enabled else "disabled"
        for btn in self.jog_buttons:
            btn.config(state=state)
        if enabled:
            self._bind_arrows()
            self._start_position_poll()
        else:
            self._unbind_arrows()
            self._stop_position_poll()

    def _bind_arrows(self):
        if self._arrows_bound:
            return
        for keysym in ARROW_KEYSYMS:
            self.bind_all(f"<{keysym}>", self._on_arrow_key)
        self._arrows_bound = True

    def _unbind_arrows(self):
        if not self._arrows_bound:
            return
        for keysym in ARROW_KEYSYMS:
            self.unbind_all(f"<{keysym}>")
        self._arrows_bound = False

    def _on_arrow_key(self, event):
        if not self.jog_enabled:
            return
        focus = self.focus_get()
        if isinstance(focus, (tk.Entry, ttk.Entry)):
            # Let the focused entry handle cursor movement instead of
            # hijacking arrow keys into a jog command.
            return
        dx_sign, dy_sign = ARROW_KEYSYMS[event.keysym]
        self._do_jog(dx_sign, dy_sign)

    def _do_jog(self, dx_sign, dy_sign):
        if not self.jog_enabled or self.motion is None:
            return
        dx = dx_sign * self.jog_step_mm
        dy = dy_sign * self.jog_step_mm
        self.motion.jog(dx_mm=dx, dy_mm=dy)
        self._moved_since_checkpoint += abs(dx) + abs(dy)
        self._refresh_position()

        if self._moved_since_checkpoint >= self.checkpoint_interval_mm:
            moved = self._moved_since_checkpoint
            self._moved_since_checkpoint = 0.0
            if not messagebox.askyesno(
                "Jog checkpoint",
                f"Moved ~{moved:.1f}mm since the last check — still tracking "
                "real motion on the mirror?",
            ):
                self._abort(
                    "Jog checkpoint not confirmed — the stage may have run "
                    "into a hard limit partway through this move and could "
                    "now be silently miscounting position. Stopping rather "
                    "than continuing to jog on an axis that may be stalled; "
                    "check the hardware before re-running."
                )

    def _refresh_position(self):
        if self.motion is None:
            self.pos_var.set("Position: --")
            return
        x, y = self.motion.get_position()
        self.pos_var.set(f"Position: {x:.3f}, {y:.3f} mm")

    def _start_position_poll(self):
        if self._position_poll_job is not None:
            return

        def _tick():
            self._refresh_position()
            self._position_poll_job = self.after(300, _tick)

        self._position_poll_job = self.after(300, _tick)

    def _stop_position_poll(self):
        if self._position_poll_job is not None:
            self.after_cancel(self._position_poll_job)
            self._position_poll_job = None

    # ------------------------------------------------------------------
    # Step: idle
    # ------------------------------------------------------------------

    def _render_idle(self):
        self.step = "idle"
        self._clear_step_frame()
        ttk.Label(self.step_frame, text=(
            "Jog checkpoint interval (mm) — how far to jog before "
            f"re-confirming real motion [{JOG_CHECKPOINT_INTERVAL_MIN_MM}-"
            f"{JOG_CHECKPOINT_INTERVAL_MAX_MM}]"
        ), wraplength=460, justify="left").grid(row=0, column=0, columnspan=2, sticky="w")
        self.checkpoint_var = tk.StringVar(value=str(JOG_CHECKPOINT_INTERVAL_DEFAULT_MM))
        ttk.Entry(self.step_frame, textvariable=self.checkpoint_var, width=10).grid(
            row=1, column=0, sticky="w")
        ttk.Button(self.step_frame, text="Start Calibration", command=self._start_calibration).grid(
            row=2, column=0, pady=(8, 0), sticky="w")

    def _start_calibration(self):
        if not messagebox.askokcancel(
            "Before you begin",
            "Confirm:\n"
            "  - Wafer or calibration target installed\n"
            "  - Scanner mounted\n"
            "  - Pyrometer aim light ON",
        ):
            return

        try:
            checkpoint_interval = validate_jog_checkpoint_interval_mm(self.checkpoint_var.get())
        except (ValueError, TypeError) as exc:
            messagebox.showerror("Invalid checkpoint interval", str(exc))
            return
        self.checkpoint_interval_mm = checkpoint_interval

        try:
            self.motion = get_motion_controller(self.config)
        except Exception as exc:
            messagebox.showerror("Motion connection failed", str(exc))
            return

        # Provisional zero -- move_to()/jog() refuse to move at all until
        # _homed is True, so this just permits the clearance check and
        # reference-mark jog below. Re-zeroed for real once the operator
        # confirms the actual reference mark (see _confirm_reference).
        self.motion.zero_here()
        self._log("Connected. Provisional zero set (allows jogging).")

        if not self._run_clearance_check():
            return

        self.edges = {}
        self._moved_since_checkpoint = 0.0
        self._render_reference()

    def _run_clearance_check(self):
        directions = [
            ("RIGHT (+X)", CLEARANCE_CHECK_STEP_MM, 0.0),
            ("LEFT (-X)", -CLEARANCE_CHECK_STEP_MM, 0.0),
            ("UP (+Y)", 0.0, CLEARANCE_CHECK_STEP_MM),
            ("DOWN (-Y)", 0.0, -CLEARANCE_CHECK_STEP_MM),
        ]
        self._log("Running clearance check (small test jog in each direction)...")
        for label, dx, dy in directions:
            self.motion.jog(dx_mm=dx, dy_mm=dy)
            self._refresh_position()
            if not messagebox.askyesno("Clearance check", f"Jogged {label}. Did the spot visibly move?"):
                self._abort(
                    f"No visible motion jogging {label} — the stage may already be "
                    "at a hard limit in this direction. Stopping here rather than "
                    "issuing further moves onto an axis in this state; check the "
                    "hardware before re-running."
                )
                return False
        self._log("Clearance confirmed in all 4 directions.")
        return True

    # ------------------------------------------------------------------
    # Step: reference mark
    # ------------------------------------------------------------------

    def _render_reference(self):
        self.step = "reference"
        self._clear_step_frame()
        ttk.Label(self.step_frame, text=EDGE_PROMPTS["reference"], wraplength=460, justify="left").grid(
            row=0, column=0, sticky="w")
        ttk.Button(self.step_frame, text="Confirm reference mark", command=self._confirm_reference).grid(
            row=1, column=0, pady=(8, 0), sticky="w")
        self._set_jog_enabled(True)

    def _confirm_reference(self):
        x, y = self.motion.get_position()
        self.motion.zero_here()
        self._log(f"Origin zeroed at reference mark (was at {x:.3f}, {y:.3f} mm "
                  "in the provisional frame).")
        self._moved_since_checkpoint = 0.0
        self._edge_idx = 0
        self._render_edge()

    # ------------------------------------------------------------------
    # Step: 4 wafer edges
    # ------------------------------------------------------------------

    def _render_edge(self):
        self.step = "edge"
        edge_name = EDGE_ORDER[self._edge_idx]
        self._clear_step_frame()
        ttk.Label(
            self.step_frame,
            text=f"Edge {self._edge_idx + 1}/{len(EDGE_ORDER)}: {EDGE_PROMPTS[edge_name]}",
            wraplength=460, justify="left",
        ).grid(row=0, column=0, sticky="w")
        ttk.Button(
            self.step_frame, text=f"Confirm {edge_name} edge", command=self._confirm_edge,
        ).grid(row=1, column=0, pady=(8, 0), sticky="w")
        self._moved_since_checkpoint = 0.0
        self._set_jog_enabled(True)

    def _confirm_edge(self):
        edge_name = EDGE_ORDER[self._edge_idx]
        x, y = self.motion.get_position()
        self.edges[edge_name] = (x, y)
        self._log(f"Recorded {edge_name} edge at ({x:.3f}, {y:.3f}) mm")
        self._edge_idx += 1
        if self._edge_idx < len(EDGE_ORDER):
            self._render_edge()
        else:
            self._set_jog_enabled(False)
            self._render_steps_per_mm()

    # ------------------------------------------------------------------
    # Step: optional steps_per_mm recalibration
    # ------------------------------------------------------------------

    def _render_steps_per_mm(self):
        self.step = "steps_per_mm"
        self._clear_step_frame()

        old_spmm_x = float(self.config["motion"]["steps_per_mm_x"])
        old_spmm_y = float(self.config["motion"]["steps_per_mm_y"])
        x_left, _ = self.edges["left"]
        x_right, _ = self.edges["right"]
        _, y_top = self.edges["top"]
        _, y_bottom = self.edges["bottom"]
        self._steps_lr = abs(x_right - x_left) * old_spmm_x
        self._steps_bt = abs(y_top - y_bottom) * old_spmm_y

        ttk.Label(self.step_frame, text=(
            "Optional: re-derive steps_per_mm from these same 4 edges "
            "(one sample from your jog, not an averaged measurement — "
            "good for a quick pass, not final precision).\n"
            f"Left-right jog used {self._steps_lr:.0f} motor steps (X).\n"
            f"Bottom-top jog used {self._steps_bt:.0f} motor steps (Y).\n"
            "Leave both blank and click Skip if steps_per_mm is already calibrated."
        ), wraplength=460, justify="left").grid(row=0, column=0, columnspan=2, sticky="w")

        ttk.Label(self.step_frame, text="True left-right distance (mm)").grid(row=1, column=0, sticky="w")
        self.true_x_var = tk.StringVar()
        ttk.Entry(self.step_frame, textvariable=self.true_x_var, width=10).grid(row=1, column=1, sticky="w")

        ttk.Label(self.step_frame, text="True bottom-top distance (mm)").grid(row=2, column=0, sticky="w")
        self.true_y_var = tk.StringVar()
        ttk.Entry(self.step_frame, textvariable=self.true_y_var, width=10).grid(row=2, column=1, sticky="w")

        ttk.Button(self.step_frame, text="Apply", command=self._apply_steps_per_mm).grid(
            row=3, column=0, pady=(8, 0), sticky="w")
        ttk.Button(self.step_frame, text="Skip", command=self._skip_steps_per_mm).grid(
            row=3, column=1, pady=(8, 0), sticky="w")

    def _apply_steps_per_mm(self):
        x_raw = self.true_x_var.get().strip()
        y_raw = self.true_y_var.get().strip()
        if not x_raw or not y_raw:
            messagebox.showerror("Missing values", "Enter both true distances, or click Skip.")
            return
        try:
            true_x_mm = float(x_raw)
            true_y_mm = float(y_raw)
        except ValueError:
            messagebox.showerror("Invalid values", "Enter numeric distances in mm.")
            return

        old_spmm_x = float(self.config["motion"]["steps_per_mm_x"])
        old_spmm_y = float(self.config["motion"]["steps_per_mm_y"])
        new_spmm_x = self._steps_lr / true_x_mm
        new_spmm_y = self._steps_bt / true_y_mm

        self.spmm_result = {
            "steps_per_mm_x": new_spmm_x,
            "steps_per_mm_y": new_spmm_y,
            "old_steps_per_mm_x": old_spmm_x,
            "old_steps_per_mm_y": old_spmm_y,
            "true_x_mm": true_x_mm,
            "true_y_mm": true_y_mm,
        }
        self._log(f"steps_per_mm_x: {old_spmm_x:.2f} -> {new_spmm_x:.2f}")
        self._log(f"steps_per_mm_y: {old_spmm_y:.2f} -> {new_spmm_y:.2f}")

        self.edges = rescale_edges(self.edges, self.spmm_result)
        self._render_params()

    def _skip_steps_per_mm(self):
        self.spmm_result = None
        self._render_params()

    # ------------------------------------------------------------------
    # Step: scan parameters (step size / dwell time / passes)
    # ------------------------------------------------------------------

    def _render_params(self):
        self.step = "params"
        self._clear_step_frame()

        self.step_size_var = tk.StringVar(value=str(scan_params.STEP_SIZE_DEFAULT_MM))
        self.dwell_var = tk.StringVar(value=str(scan_params.DWELL_TIME_DEFAULT_S))
        self.passes_var = tk.StringVar(value=str(scan_params.PASSES_DEFAULT))

        ttk.Label(self.step_frame, text=(
            f"Step size (mm) [{scan_params.STEP_SIZE_MIN_MM}-{scan_params.STEP_SIZE_MAX_MM}]"
        )).grid(row=0, column=0, sticky="w")
        ttk.Entry(self.step_frame, textvariable=self.step_size_var, width=10).grid(row=0, column=1, sticky="w")

        ttk.Label(self.step_frame, text=(
            f"Dwell time (s) [{scan_params.DWELL_TIME_MIN_S}-{scan_params.DWELL_TIME_MAX_S}]"
        )).grid(row=1, column=0, sticky="w")
        ttk.Entry(self.step_frame, textvariable=self.dwell_var, width=10).grid(row=1, column=1, sticky="w")

        ttk.Label(self.step_frame, text=(
            f"Passes [{scan_params.PASSES_MIN}-{scan_params.PASSES_MAX}]"
        )).grid(row=2, column=0, sticky="w")
        ttk.Entry(self.step_frame, textvariable=self.passes_var, width=10).grid(row=2, column=1, sticky="w")

        ttk.Button(self.step_frame, text="Preview", command=self._handle_preview).grid(
            row=3, column=0, pady=(8, 0), sticky="w")

    def _handle_preview(self):
        try:
            step_size_mm = scan_params.validate_step_size_mm(self.step_size_var.get())
            dwell_time_s = scan_params.validate_dwell_time_s(self.dwell_var.get())
            passes = scan_params.validate_passes(self.passes_var.get())
        except (ValueError, TypeError) as exc:
            messagebox.showerror("Invalid scan parameters", str(exc))
            return

        area = compute_area(self.edges)
        # Round once, here, to the exact precision write_results() persists --
        # see calibrate_scan_area.CONFIG_RANGE_DECIMALS for why this must
        # happen before the preview, not just at write time.
        area = {
            "x_range_mm": [round(v, CONFIG_RANGE_DECIMALS) for v in area["x_range_mm"]],
            "y_range_mm": [round(v, CONFIG_RANGE_DECIMALS) for v in area["y_range_mm"]],
            "wafer_center_mm": [round(v, CONFIG_RANGE_DECIMALS) for v in area["wafer_center_mm"]],
        }
        radius_mm = compute_radius_mm(area, self.spmm_result)

        nx, ny = scan_params.grid_dims_from_range(area["x_range_mm"], area["y_range_mm"], step_size_mm)

        # Build the SAME grid scan_manager.py will actually run (bounding
        # box + circular mask), not just nx*ny -- matches
        # calibrate_scan_area.py's own preview for the same reason.
        preview_scan_cfg = {
            "grid": {
                "x_range_mm": area["x_range_mm"],
                "y_range_mm": area["y_range_mm"],
                "wafer_center_mm": area["wafer_center_mm"],
                "wafer_radius_mm": radius_mm,
                "step_size_mm": step_size_mm,
            },
            "scan_order": self.config.get("scan", {}).get("scan_order", "raster"),
        }
        masked_points, _, _ = generate_grid(preview_scan_cfg)
        n_points = len(masked_points)
        est_s = n_points * passes * dwell_time_s

        self.area = area
        self.radius_mm = radius_mm
        self.step_size_mm = step_size_mm
        self.dwell_time_s = dwell_time_s
        self.passes = passes
        self.recommended_home_steps = (
            recommend_home_steps(self.edges, self.spmm_result) if self.spmm_result is not None else None
        )

        self._log(f"X range: {area['x_range_mm']} mm")
        self._log(f"Y range: {area['y_range_mm']} mm")
        self._log(f"Wafer center: {area['wafer_center_mm']} mm")
        self._log(f"Wafer radius (scan mask): {radius_mm} mm")
        self._log(f"Bounding box: {nx} x {ny} = {nx * ny} grid positions")
        self._log(f"Within wafer radius: {n_points} points/pass "
                  f"({nx * ny - n_points} corner/off-wafer positions excluded)")
        self._log(f"Passes: {passes}")
        self._log(f"Estimated scan time: {est_s / 60:.1f} min "
                  f"({n_points * passes} total point measurements)")

        self._render_preview(nx, ny, n_points, est_s)

    # ------------------------------------------------------------------
    # Step: preview + write
    # ------------------------------------------------------------------

    def _render_preview(self, nx, ny, n_points, est_s):
        self.step = "preview"
        self._clear_step_frame()

        summary = (
            f"X range: {self.area['x_range_mm']} mm\n"
            f"Y range: {self.area['y_range_mm']} mm\n"
            f"Wafer center: {self.area['wafer_center_mm']} mm\n"
            f"Wafer radius: {self.radius_mm} mm\n"
            f"Grid: {nx} x {ny} bounding box, {n_points} points/pass within radius\n"
            f"Passes: {self.passes}  |  Estimated time: {est_s / 60:.1f} min"
        )
        ttk.Label(self.step_frame, text=summary, justify="left").grid(
            row=0, column=0, columnspan=2, sticky="w")

        self.write_home_steps_var = tk.BooleanVar(value=self.recommended_home_steps is not None)
        if self.recommended_home_steps is not None:
            ttk.Checkbutton(
                self.step_frame,
                text=f"Also write home_steps={self.recommended_home_steps} (2x margin lower bound)",
                variable=self.write_home_steps_var,
            ).grid(row=1, column=0, columnspan=2, sticky="w", pady=(8, 0))

        ttk.Button(self.step_frame, text="Write to config.yaml", command=self._write_config).grid(
            row=2, column=0, pady=(8, 0), sticky="w")
        ttk.Button(self.step_frame, text="Back", command=self._render_params).grid(
            row=2, column=1, pady=(8, 0), sticky="w")

    def _write_config(self):
        results = dict(self.area)
        results["wafer_radius_mm"] = self.radius_mm
        results["step_size_mm"] = self.step_size_mm
        results["dwell_time_s"] = self.dwell_time_s
        results["passes"] = self.passes
        if self.spmm_result is not None:
            results["steps_per_mm_x"] = self.spmm_result["steps_per_mm_x"]
            results["steps_per_mm_y"] = self.spmm_result["steps_per_mm_y"]
        if self.recommended_home_steps is not None and self.write_home_steps_var.get():
            results["home_steps"] = self.recommended_home_steps

        try:
            failed = write_results(self.config_path, results)
        except Exception as exc:
            messagebox.showerror("Write failed", str(exc))
            return

        if failed:
            detail = "\n".join(f"- {label}: {msg}" for label, msg in failed)
            messagebox.showwarning(
                "Partial write",
                f"Wrote PARTIAL calibration results — {len(failed)} field(s) "
                f"could not be written:\n{detail}\n\nEverything else was "
                "saved; add the missing line(s) to config.yaml by hand.",
            )
            self._log(f"Wrote PARTIAL results to {self.config_path} — {len(failed)} field(s) failed.")
        else:
            self._log(f"Wrote calibration results to {self.config_path}")

        self._render_done()

    # ------------------------------------------------------------------
    # Step: done
    # ------------------------------------------------------------------

    def _render_done(self):
        self.step = "done"
        self._clear_step_frame()
        ttk.Label(self.step_frame, text="Calibration written to config.yaml.").grid(
            row=0, column=0, columnspan=2, sticky="w")
        ttk.Button(self.step_frame, text="Proceed to Scan", command=self._proceed_to_scan).grid(
            row=1, column=0, pady=(8, 0), sticky="w")
        ttk.Button(self.step_frame, text="Recalibrate", command=self._recalibrate).grid(
            row=1, column=1, pady=(8, 0), sticky="w")

    def _proceed_to_scan(self):
        config = yaml.safe_load(self.config_path.read_text(encoding="utf-8-sig"))
        motion = self.motion
        # Ownership transferred to the app -- this panel must not
        # poll/jog it after handoff (a scan may start using it any moment).
        self.motion = None
        self._set_jog_enabled(False)
        self.on_calibrated(config, motion, True)

    def _recalibrate(self):
        self._close_motion()
        self._reset_state()
        self._render_idle()

    # ------------------------------------------------------------------
    # Abort / reset / cleanup
    # ------------------------------------------------------------------

    def _handle_abort_click(self):
        if self.step == "idle":
            return
        if self.step == "done":
            self._recalibrate()
            return
        if messagebox.askyesno(
            "Abort calibration",
            "Abort this calibration and reset? Progress on the current run will be lost.",
        ):
            self._abort("Aborted by operator.")

    def _abort(self, message):
        self._set_jog_enabled(False)
        self._close_motion()
        self._reset_state()
        self._log(f"ABORTED: {message}")
        messagebox.showerror("Calibration aborted", message)
        self._render_idle()

    def _close_motion(self):
        if self.motion is not None:
            close = getattr(self.motion, "close", None)
            if callable(close):
                close()
        self.motion = None

    def _reset_state(self):
        self.edges = {}
        self._edge_idx = 0
        self.spmm_result = None
        self.area = None
        self.radius_mm = None
        self.recommended_home_steps = None
        self._moved_since_checkpoint = 0.0

    def _clear_step_frame(self):
        for child in self.step_frame.winfo_children():
            child.destroy()

    def shutdown(self):
        """Called by app.py's on_close -- release the hardware connection
        if calibration is mid-flow and was never handed off to a scan."""
        self._stop_position_poll()
        self._unbind_arrows()
        self._close_motion()

    def _on_destroy(self, event):
        if event.widget is not self:
            return
        self._stop_position_poll()
        self._unbind_arrows()
