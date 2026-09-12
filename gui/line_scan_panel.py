"""
gui/line_scan_panel.py

PR2b (2026-09-08): GUI tab for an arbitrary-angle LINE scan -- one
endpoint plus an angle and a length, rather than the Scan tab's 2D
raster grid. Built to the same "investigate -> present -> approve ->
build" pattern as PR1/PR2a; see project doc
claude/scan_requirements_gap_analysis.md (PR2b section) for the
approved design this implements.

Operator-facing geometry input is "endpoint + angle + length", not two
typed-in endpoints: Start X/Y plus Angle (deg) plus Length (mm). The
second endpoint is computed via scan_params.endpoint_from_angle().
That function's docstring explains the angle convention in detail --
worth repeating the headline here since it's the one thing on this tab
most likely to be confused with something else in this codebase:
angle_deg=0 points along +x_mm, increasing COUNTERCLOCKWISE toward
+y_mm, in the SCAN-PLANE (x_mm, y_mm) frame. This is a completely
different "degrees" from motion_controller.py/real_conexagap_motion.py/
real_mr1530_motion.py's hardware-frame mirror/mount-tilt degrees --
the two never appear in the same function, and this docstring plus
endpoint_from_angle()'s are where that's called out explicitly.

No "scan order" field here (unlike ControlPanel) -- raster/serpentine
ordering is meaningless for a 1D line; scan.scan_manager.
generate_line_points() doesn't read scan_cfg["scan_order"] at all.

Bounds checking only validates the two ENDPOINTS against the
calibrated wafer (scan_params.in_radius()), not every intermediate
point -- a circle is convex, so if both endpoints of a chord lie
inside a circular mask, every point on that chord does too. (Once the
scan actually starts, scan_manager.preflight_check() still validates
every individual commanded point against motion.soft_limits --
motion.soft_limits is a rectangle, also convex, so the same argument
holds there too, but preflight_check() doesn't special-case line mode
at all: it already checks whatever compute_commanded_points() hands
it, point by point, regardless of mode.) A calibrated-wafer violation
here is a hard block (error dialog, refuse to start) -- never a
silent clip -- matching PR2a's diameter-exceeds-wafer precedent.
"Not calibrated yet" is permissive, same "absent means no mask"
convention generate_grid()/ControlPanel already use: nothing here
stops an operator from running a line scan before ever calibrating,
same as today's grid Scan tab.

Threading/display: fully self-contained, following
gui/adaptive_scan_panel.py's precedent rather than reusing
StatusPanel/LiveMapPanel/ControlPanel -- its own queue.Queue(),
threading.Event(), worker thread, poll loop, status labels, log, and
live plot. Unlike Adaptive Scan, though, this tab does NOT open its
own independent motion connection: it shares the Calibrate tab's
hand-off with the regular Scan tab (see take_shared_motion, passed in
by app.py -- mirrors what App.start_scan() already does with
self.motion/self._pending_already_homed) because Line Scan runs the
same precision ScanManager/get_motion_controller() acquisition path as
the Scan tab, not Adaptive Scan's genuinely different open-loop mode.
Opening a second, competing connection here risks re-triggering the
exact class of bug already fixed once in this codebase (see memory:
carat_scanner_8742_connection_leak_fix -- once the hardware accepts
one live connection, every later connection attempt fails until the
first is released). Consequence: this tab never owns and never closes
the shared motion object -- App.on_close() is solely responsible for
that (see this class's shutdown(), which only stops a running scan).

gui/scan_worker.py's run_scan() is reused completely unchanged --
ScanManager itself already handles the grid-vs-line dispatch
internally (see scan_manager.ScanManager.__init__/run()), so the
worker function needs no line-mode awareness at all.
"""

import copy
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from scan.scan_params import (
    DWELL_TIME_DEFAULT_S, DWELL_TIME_MIN_S, DWELL_TIME_MAX_S,
    PASSES_DEFAULT, PASSES_MIN, PASSES_MAX,
    SCAN_TIME_WARNING_THRESHOLD_S,
    validate_dwell_time_s, validate_passes, estimate_scan_time_s,
    in_radius, endpoint_from_angle,
)
from scan.live_map_accumulator import LineAccumulator
from gui.scan_worker import run_scan

POLL_INTERVAL_MS = 150


class LineScanPanel(ttk.Frame):
    def __init__(self, parent, config, take_shared_motion):
        super().__init__(parent, padding=0)
        self.base_config = config
        self.take_shared_motion = take_shared_motion

        self.q = None
        self.stop_event = None
        self.worker = None
        self.running = False
        self.scan_start_time = None
        self.points_done = 0
        self.total_points = 0

        self.accumulator = None

        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=1)

        self.params_frame = ttk.Frame(self, padding=8)
        self.params_frame.grid(row=0, column=0, sticky="n")
        self.status_frame = ttk.Frame(self, padding=8)
        self.status_frame.grid(row=1, column=0, sticky="n")
        self.live_frame = ttk.Frame(self, padding=8)
        self.live_frame.grid(row=0, column=1, rowspan=2, sticky="nsew")

        self._build_params()
        self._build_status()
        self._build_live_view()

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _build_params(self):
        scan_cfg = self.base_config["scan"]
        self.dwell_var = tk.StringVar(value=str(scan_cfg.get("dwell_time_s", DWELL_TIME_DEFAULT_S)))
        self.passes_var = tk.StringVar(value=str(scan_cfg.get("passes", PASSES_DEFAULT)))
        self.outdir_var = tk.StringVar(value=self.base_config["output"].get("base_dir", "./scan_data"))

        # Start X/Y default to the calibrated wafer center once one
        # exists (see load_config()); blank until then, same "nothing to
        # default to yet" convention as ControlPanel's diameter field.
        center_mm, _ = self._calibrated_center_and_radius()
        start_x_default = f"{center_mm[0]:.4f}" if center_mm is not None else ""
        start_y_default = f"{center_mm[1]:.4f}" if center_mm is not None else ""
        self.start_x_var = tk.StringVar(value=start_x_default)
        self.start_y_var = tk.StringVar(value=start_y_default)
        self.angle_var = tk.StringVar(value="0.0")
        self.length_var = tk.StringVar(value="")
        self.target_points_var = tk.StringVar(value="")
        self.solve_result_var = tk.StringVar(value="")

        # PR-Stage2 (2026-09-09): mirrors ControlPanel's ir_enabled_var/
        # oes_enabled_var -- see config.yaml's ir.enabled/oes.enabled
        # comments. Readers are still constructed either way; this only
        # skips the actual dwell-window read.
        self.ir_enabled_var = tk.BooleanVar(
            value=bool(self.base_config.get("ir", {}).get("enabled", True)))
        self.oes_enabled_var = tk.BooleanVar(
            value=bool(self.base_config.get("oes", {}).get("enabled", True)))

        frame = ttk.LabelFrame(self.params_frame, text="Line scan parameters", padding=8)
        frame.grid(row=0, column=0, sticky="n")

        row = 0
        ttk.Label(frame, text=f"Dwell time (s)  [{DWELL_TIME_MIN_S}-{DWELL_TIME_MAX_S}]").grid(
            row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.dwell_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(frame, text="Start X (mm)").grid(row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.start_x_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(frame, text="Start Y (mm)").grid(row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.start_y_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(frame, text="Angle (deg, CCW from +x)").grid(row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.angle_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(frame, text="Length (mm)").grid(row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.length_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(frame, text="Target point count").grid(row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.target_points_var, width=10).grid(row=row, column=1, sticky="w")
        self.solve_btn = ttk.Button(frame, text="Solve", command=self._handle_solve)
        self.solve_btn.grid(row=row, column=2, sticky="w", padx=(6, 0))
        row += 1

        ttk.Label(frame, textvariable=self.solve_result_var, foreground="#555555",
                  wraplength=280, justify="left").grid(row=row, column=0, columnspan=3, sticky="w")
        row += 1

        ttk.Label(frame, text=f"Passes  [{PASSES_MIN}-{PASSES_MAX}]").grid(row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.passes_var, width=10).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Checkbutton(
            frame, text="Enable IR (pyrometer)", variable=self.ir_enabled_var,
        ).grid(row=row, column=0, sticky="w")
        ttk.Checkbutton(
            frame, text="Enable OES (spectrometer)", variable=self.oes_enabled_var,
        ).grid(row=row, column=1, sticky="w")
        row += 1

        ttk.Label(frame, text="Output dir").grid(row=row, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.outdir_var, width=30).grid(
            row=row, column=1, columnspan=2, sticky="we")
        row += 1

        self.start_btn = ttk.Button(frame, text="Start Scan", command=self._handle_start)
        self.start_btn.grid(row=row, column=0, pady=(10, 0), sticky="we")
        self.abort_btn = ttk.Button(frame, text="Abort", command=self._handle_abort, state="disabled")
        self.abort_btn.grid(row=row, column=1, pady=(10, 0), sticky="we")

    def _build_status(self):
        self.point_var = tk.StringVar(value="Point: -- / --")
        self.pos_var = tk.StringVar(value="Position: --")
        self.ir_var = tk.StringVar(value="Last IR: --")
        self.elapsed_var = tk.StringVar(value="Elapsed: --")

        ttk.Label(self.status_frame, textvariable=self.point_var).grid(row=0, column=0, sticky="w")
        ttk.Label(self.status_frame, textvariable=self.pos_var).grid(row=1, column=0, sticky="w")
        ttk.Label(self.status_frame, textvariable=self.ir_var).grid(row=2, column=0, sticky="w")
        ttk.Label(self.status_frame, textvariable=self.elapsed_var).grid(row=3, column=0, sticky="w")

        self.log = ScrolledText(self.status_frame, height=16, width=58, state="disabled")
        self.log.grid(row=4, column=0, pady=(8, 0), sticky="nsew")

    def _build_live_view(self):
        self.live_frame.columnconfigure(0, weight=1)
        self.live_frame.rowconfigure(0, weight=1)

        self.figure = Figure(figsize=(4.5, 4.5), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Position along line, s (mm)")
        self.ax.set_ylabel("IR temp (°C)")
        self.ax.set_title("IR temperature vs. position — live")
        self.ax.grid(True, alpha=0.3)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.live_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # Mirrors gui/live_map.py's show_counts_var toggle, same UX and
        # same 2026-07-27 surprise-monitoring rationale, just plotted
        # against s_mm instead of imshow'd over (x_mm, y_mm).
        self.show_counts_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            self.live_frame, text="Show read counts per point (instead of temperature)",
            variable=self.show_counts_var, command=self._redraw_live,
        ).grid(row=1, column=0, sticky="w", pady=(6, 0))

    # ------------------------------------------------------------------
    # Config / calibration helpers
    # ------------------------------------------------------------------

    def _calibrated_center_and_radius(self):
        """
        Same convention as ControlPanel._calibrated_center_and_radius():
        (wafer_center_mm, wafer_radius_mm) from base_config, or
        (None, None) if nothing's calibrated yet.
        """
        grid_cfg = self.base_config["scan"]["grid"]
        radius_mm = grid_cfg.get("wafer_radius_mm")
        if radius_mm is None:
            return None, None
        center_mm = grid_cfg.get("wafer_center_mm", [0.0, 0.0])
        return tuple(center_mm), float(radius_mm)

    def load_config(self, config):
        """
        Called by app.py's _handle_calibrated(), alongside
        ControlPanel.load_config() -- refreshes dwell/passes/output-dir
        from the freshly-written config.yaml and re-defaults Start X/Y
        to the newly calibrated wafer center. A fresh calibration
        changes what "the wafer" even is, so any angle/length typed
        against the OLD calibration is left as-is (they're relative
        offsets from Start X/Y, not absolute positions) but the solved
        result label is cleared rather than left showing a stale
        end-point/estimate.
        """
        self.base_config.clear()
        self.base_config.update(copy.deepcopy(config))
        scan_cfg = self.base_config["scan"]
        self.dwell_var.set(str(scan_cfg.get("dwell_time_s", DWELL_TIME_DEFAULT_S)))
        self.passes_var.set(str(scan_cfg.get("passes", PASSES_DEFAULT)))
        self.outdir_var.set(self.base_config["output"].get("base_dir", "./scan_data"))
        self.ir_enabled_var.set(bool(self.base_config.get("ir", {}).get("enabled", True)))
        self.oes_enabled_var.set(bool(self.base_config.get("oes", {}).get("enabled", True)))

        center_mm, _ = self._calibrated_center_and_radius()
        if center_mm is not None:
            self.start_x_var.set(f"{center_mm[0]:.4f}")
            self.start_y_var.set(f"{center_mm[1]:.4f}")
        self.solve_result_var.set("")

    # ------------------------------------------------------------------
    # Geometry / validation
    # ------------------------------------------------------------------

    def _compute_and_validate_line(self):
        """
        Parse Start X/Y, Angle, Length, and Target point count; compute
        the end point via endpoint_from_angle(); and, if a wafer is
        calibrated, hard-block on either endpoint falling outside it
        (see this module's docstring for the convexity argument that
        makes checking just the two endpoints sufficient).

        Returns (start_mm, end_mm, n_points, None) on success, or
        (None, None, None, error_message) on any validation failure.
        Shared by both _handle_solve() and _handle_start() so the two
        can never drift apart on what counts as valid -- mirrors
        ControlPanel's own re-validate-on-Start pattern.
        """
        try:
            start_x = float(self.start_x_var.get())
            start_y = float(self.start_y_var.get())
        except ValueError:
            return None, None, None, "Start X and Start Y must be numbers."

        try:
            angle_deg = float(self.angle_var.get())
        except ValueError:
            return None, None, None, "Angle must be a number."

        try:
            length_mm = float(self.length_var.get())
        except ValueError:
            return None, None, None, "Length must be a number."
        if length_mm <= 0:
            return None, None, None, "Length must be greater than 0."

        try:
            n_points = int(self.target_points_var.get())
        except ValueError:
            return None, None, None, "Target point count must be a whole number."
        if n_points < 1:
            return None, None, None, "Target point count must be at least 1."

        start_mm = (start_x, start_y)
        end_mm = endpoint_from_angle(start_mm, angle_deg, length_mm)

        center_mm, radius_mm = self._calibrated_center_and_radius()
        if radius_mm is not None:
            if not in_radius(start_mm[0], start_mm[1], center_mm, radius_mm):
                return None, None, None, (
                    f"Start point ({start_mm[0]:.3f}, {start_mm[1]:.3f}) mm falls outside "
                    f"the calibrated wafer (center {center_mm}, radius {radius_mm:.4g} mm)."
                )
            if not in_radius(end_mm[0], end_mm[1], center_mm, radius_mm):
                return None, None, None, (
                    f"End point ({end_mm[0]:.3f}, {end_mm[1]:.3f}) mm falls outside "
                    f"the calibrated wafer (center {center_mm}, radius {radius_mm:.4g} mm). "
                    "Reduce the length or change the angle/start point."
                )

        return start_mm, end_mm, n_points, None

    def _handle_solve(self):
        start_mm, end_mm, n_points, err = self._compute_and_validate_line()
        if err:
            messagebox.showerror("Invalid line parameters", err)
            return

        try:
            dwell = validate_dwell_time_s(self.dwell_var.get())
            passes = validate_passes(self.passes_var.get())
        except (ValueError, TypeError) as exc:
            messagebox.showerror("Invalid scan parameters", str(exc))
            return

        length_mm = math.hypot(end_mm[0] - start_mm[0], end_mm[1] - start_mm[1])
        step_mm = length_mm / (n_points - 1) if n_points > 1 else 0.0
        settle_time_s = self.base_config["scan"].get("settle_time_s", 0.0)
        est_s = estimate_scan_time_s(n_points, 1, dwell, settle_time_s, passes)

        self.solve_result_var.set(
            f"End point: ({end_mm[0]:.3f}, {end_mm[1]:.3f}) mm  |  "
            f"step spacing {step_mm:.4f} mm  |  "
            f"~{est_s / 60:.1f} min estimated (passes={passes})."
        )

        if est_s > SCAN_TIME_WARNING_THRESHOLD_S:
            messagebox.showwarning(
                "Scan time estimate",
                f"Estimated scan time is {est_s / 60:.1f} min, over the "
                f"{SCAN_TIME_WARNING_THRESHOLD_S / 60:.0f}-min target.\n\n"
                "Consider fewer target points, a shorter length, shorter "
                "dwell time, or fewer passes.",
            )

    # ------------------------------------------------------------------
    # Start / Abort
    # ------------------------------------------------------------------

    def _handle_start(self):
        if self.running:
            return  # Start button is disabled while running, but guard anyway

        start_mm, end_mm, n_points, err = self._compute_and_validate_line()
        if err:
            messagebox.showerror("Invalid line parameters", err)
            return

        try:
            dwell = validate_dwell_time_s(self.dwell_var.get())
            passes = validate_passes(self.passes_var.get())
        except (ValueError, TypeError) as exc:
            messagebox.showerror("Invalid scan parameters", str(exc))
            return

        effective_config = copy.deepcopy(self.base_config)
        effective_config["scan"]["dwell_time_s"] = dwell
        effective_config["scan"]["passes"] = passes
        effective_config["output"]["base_dir"] = self.outdir_var.get()
        effective_config["ir"]["enabled"] = self.ir_enabled_var.get()
        effective_config["oes"]["enabled"] = self.oes_enabled_var.get()

        # Additive line-mode keys only -- the pre-existing grid-mode keys
        # (x_range_mm/wafer_center_mm/wafer_radius_mm/step_size_mm) are
        # deliberately left untouched, same non-destructive convention
        # ScanManager's mode dispatch already relies on elsewhere. Never
        # written back to config.yaml on disk (matches PR2a's
        # effective_config convention) -- this dict only ever lives for
        # the duration of this one run.
        grid_cfg = effective_config["scan"]["grid"]
        grid_cfg["mode"] = "line"
        grid_cfg["line_start_mm"] = [start_mm[0], start_mm[1]]
        grid_cfg["line_end_mm"] = [end_mm[0], end_mm[1]]
        grid_cfg["line_n_points"] = n_points

        # Reuse the Calibrate tab's motion connection the same way the
        # Scan tab does -- see this module's docstring for why Line Scan
        # does NOT open its own independent connection the way Adaptive
        # Scan does. already_homed is one-shot, consumed by app.py's
        # take_shared_motion() regardless of whether this scan actually
        # starts successfully.
        motion, already_homed = self.take_shared_motion()

        self.stop_event = threading.Event()
        self.q = queue.Queue()

        self.accumulator = LineAccumulator(start_mm, end_mm, n_points)
        self._redraw_live()

        self.total_points = n_points * passes
        self.points_done = 0
        self.point_var.set(f"Point: 0 / {self.total_points}")
        self.pos_var.set("Position: --")
        self.ir_var.set("Last IR: --")
        self.elapsed_var.set("Elapsed: 0s")
        self.log.config(state="normal")
        self.log.delete("1.0", tk.END)
        self.log.config(state="disabled")
        self._log(f"Line scan starting — {self.total_points} points queued "
                   f"(start={start_mm}, end=({end_mm[0]:.3f}, {end_mm[1]:.3f}), n={n_points}).")

        self.scan_start_time = time.time()
        self.running = True
        self._set_running(True)

        self.worker = threading.Thread(
            target=run_scan,
            args=(effective_config, self.q, self.stop_event),
            kwargs={"motion": motion, "already_homed": already_homed},
            daemon=True,
        )
        self.worker.start()
        self._poll_queue()

    def _handle_abort(self):
        if not self.running or self.stop_event is None:
            return
        self.stop_event.set()
        self._log("Abort requested — stopping after the current point.")

    def _set_running(self, running: bool):
        self.start_btn.config(state="disabled" if running else "normal")
        self.abort_btn.config(state="normal" if running else "disabled")

    # ------------------------------------------------------------------
    # Queue draining (Tk mainloop thread only)
    # ------------------------------------------------------------------

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                self._handle_message(kind, payload)
        except queue.Empty:
            pass

        if self.running and self.scan_start_time is not None:
            self.elapsed_var.set(f"Elapsed: {int(time.time() - self.scan_start_time)}s")

        if self.running:
            self.after(POLL_INTERVAL_MS, self._poll_queue)

    def _handle_message(self, kind, payload):
        if kind == "point":
            self._update_status_point(payload)
            self._update_live(payload)
        elif kind == "done":
            self.running = False
            self._set_running(False)
            self._log("Scan complete.")
        elif kind == "aborted":
            self.running = False
            self._set_running(False)
            self._log("Scan aborted.")
        elif kind == "error":
            self.running = False
            self._set_running(False)
            self._log("ERROR — see dialog for full traceback.")
            messagebox.showerror("Line scan error", payload)

    # ------------------------------------------------------------------
    # Status / log
    # ------------------------------------------------------------------

    def _update_status_point(self, record):
        is_reference = bool(record.get("is_reference"))
        tag = "REF" if is_reference else "PT"

        if not is_reference:
            self.points_done += 1
            self.point_var.set(f"Point: {self.points_done} / {self.total_points}")

        x_mm = record.get("x_mm")
        y_mm = record.get("y_mm")
        s_mm = record.get("s_mm")
        if s_mm is None or (isinstance(s_mm, float) and math.isnan(s_mm)):
            s_str = "--"
        else:
            s_str = f"{s_mm:.3f}"
        self.pos_var.set(f"Position: x={x_mm:.2f}, y={y_mm:.2f} mm  (s={s_str} mm)")

        ir_val = record.get("ir_temp_c")
        if record.get("ir_skipped"):
            ir_str = "disabled"
        elif ir_val is None or (isinstance(ir_val, float) and math.isnan(ir_val)):
            ir_str = "NaN (read error)"
        else:
            ir_str = f"{ir_val:.1f} C"
        self.ir_var.set(f"Last IR: {ir_str}")

        fault_tag = " *** MOTION FAULT ***" if record.get("motion_error") else ""
        self._log(
            f"[{tag}] pt {record.get('point_id')} "
            f"(x={x_mm:.2f}, y={y_mm:.2f}, s={s_str}) IR={ir_str} "
            f"ir_err={record.get('ir_error')} "
            f"oes_err={record.get('oes_error')} sat={record.get('oes_saturated')}"
            f"{fault_tag}"
        )

    def _log(self, text):
        self.log.config(state="normal")
        self.log.insert(tk.END, text + "\n")
        self.log.see(tk.END)
        self.log.config(state="disabled")

    # ------------------------------------------------------------------
    # Live view
    # ------------------------------------------------------------------

    def _update_live(self, record):
        # Reference-point revisits carry s_mm=NaN (see scan_manager.run
        # and data_logger.py's fixed-schema fix) -- LineAccumulator.
        # nearest_index() already treats a NaN/None s_mm as "no matching
        # point" and returns None, same convention MapAccumulator uses
        # for out-of-grid coordinates, so no separate is_reference check
        # is needed here (unlike gui/live_map.py's update_point(), which
        # checks is_reference explicitly because MapAccumulator has no
        # such NaN-aware guard of its own).
        if self.accumulator is None:
            return
        self.accumulator.add_reading(record.get("s_mm"), record.get("ir_temp_c"))
        self._redraw_live()

    def _redraw_live(self):
        if self.accumulator is None:
            return

        if self.show_counts_var.get():
            y = self.accumulator.read_count.astype(float)
            y = np.where(y == 0, np.nan, y)
            ylabel = "reads"
            title = "IR read count per point — live"
        else:
            y = self.accumulator.value_grid
            ylabel = "IR temp (°C)"
            title = "IR temperature vs. position — live"

        self.ax.clear()
        self.ax.plot(self.accumulator.s_values, y, marker="o", markersize=3, linewidth=1)
        self.ax.set_xlabel("Position along line, s (mm)")
        self.ax.set_ylabel(ylabel)
        self.ax.set_title(title)
        self.ax.grid(True, alpha=0.3)
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self):
        """
        Called by app.py's on_close(). Best-effort stop of any running
        scan, same as ControlPanel/AdaptiveScanPanel's shutdown paths --
        but deliberately does NOT touch the motion connection: unlike
        AdaptiveScanPanel (which opens and therefore owns/closes its
        own), this tab only ever borrows the shared connection via
        take_shared_motion(), and App.on_close() is solely responsible
        for closing self.motion exactly once. Closing it again here
        would double-close an already-closed connection if the Scan tab
        (or App itself) gets to it first.
        """
        if self.running and self.stop_event is not None:
            self.stop_event.set()
            if self.worker is not None:
                self.worker.join(timeout=5)
