"""
gui/adaptive_scan_panel.py

GUI tab for the adaptive (edge-following) raster scan -- see
docs/adaptive_scan_spec.md. The on-screen equivalent of running
adaptive_scan/adaptive_scan.py directly, for an operator who just wants to
position the instrument, fill in the 7 parameters, and watch it run.

Workflow, mirroring spec §3 step 1 and gui/calibration_panel.py's own
connect-then-jog pattern:
  1. Connect -- opens its OWN motion/IR/OES connection (independent of the
     Calibrate/Scan tabs' -- this mode doesn't need or use a calibrated
     range, so there's nothing to hand off to or from). Zeros the origin
     at whatever position the stage is currently sitting at (fiducial-
     style, same as CalibrationPanel/calibrate_scan_area.py) -- this
     mode's TravelGuard measures travel FROM this point, not from any
     absolute origin, so "zeroed here" is all it needs (see
     adaptive_scan.py's TravelGuard docstring).
  2. Jog to a position that reads a valid wafer signal (watch the live
     temp/emissivity/dilution readout below the jog pad).
  3. Fill in the 7 operator parameters (spec §4) -- deliberately NOT
     pre-filled with recommended values (per Roy's defaults policy, spec
     §4): every one of these is direct operator input for a real trial,
     except coarse grid cell count, which does have a real default (100).
  4. Start. Runs on a background thread (gui/adaptive_scan_worker.py) so
     the Tk mainloop stays responsive; only this file ever touches a
     tkinter object, and only from callbacks Tk itself invokes on the
     mainloop thread (poll loop, button commands) -- same rule
     gui/app.py's own docstring states for the whole GUI.
  5. On completion (or abort), the coarse grid is rendered as a heatmap
     and saved to <output dir>/coarse_grid.csv alongside the raw
     per-reading CSV adaptive_scan.py itself already writes incrementally.

Independent hardware connection, deliberately: unlike Calibrate -> Scan
(which hand off ONE motion object because they're sequential steps of the
same session), this tab and the precision Scan tab are two different,
mutually exclusive uses of the same physical mount. Nothing in this
codebase currently stops an operator from clicking Connect here AND
Start Scan on the other tab at the same time -- same trust model the
existing Calibrate/Scan split already relies on (the operator doesn't run
two hardware-touching tabs at once), not a new gap this tab introduces.
"""

import os
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from motion.motion_controller import get_motion_controller
from readers.ir_reader_base import get_ir_reader
from readers.spectrometer_reader_base import get_spectrometer_reader

from adaptive_scan.adaptive_scan import AdaptiveScanParams
from adaptive_scan.adaptive_scan_logger import save_coarse_grid_csv
from adaptive_scan.adaptive_scan_signal import read_raw_signals
from adaptive_scan.adaptive_scan_params import (
    available_signal_names, READING_INTERVAL_MODES, COARSE_GRID_CELLS_DEFAULT,
)
from gui.adaptive_scan_worker import run_adaptive_scan

# Reuse the SAME jog-step constants the Calibrate tab uses (see
# scan/calibrate_scan_area.py) rather than redefining a second set of
# magic numbers -- both tabs jog the same physical mount the same way.
from scan.calibrate_scan_area import JOG_STEP_DEFAULT_MM, JOG_STEP_MIN_MM, JOG_STEP_MAX_MM

ARROW_KEYSYMS = {
    "Up": (0.0, 1.0),
    "Down": (0.0, -1.0),
    "Left": (-1.0, 0.0),
    "Right": (1.0, 0.0),
}

POLL_INTERVAL_MS = 150          # queue drain cadence while a scan is running
POSITION_POLL_INTERVAL_MS = 300  # live position/signal readout cadence while connected


class AdaptiveScanPanel(ttk.Frame):
    """
    This tab's content (jog pad, the 7-parameter form, log, and heatmap)
    adds up to more vertical space than the other tabs -- taller than a
    lot of screens can show at once, especially at 100% Tk scaling on a
    laptop display. Rather than trust every user's screen to be big
    enough (see the module docstring's "cut off" fix, 2026-07): this
    whole panel is built inside a scrollable canvas (self._canvas /
    self.content below), so nothing is ever truly inaccessible -- if the
    window is too short to show everything at once, the operator scrolls
    (mouse wheel or the scrollbar) instead of losing the bottom of the
    tab. Every _build_* method below parents its widgets to self.content
    (the scrollable inner frame), not to `self` directly.
    """

    def __init__(self, parent, config):
        super().__init__(parent, padding=0)
        self.config = config
        self.oes_features_cfg = config.get("oes", {}).get("features", {})
        self.feature_window_nm = config.get("oes", {}).get("feature_window_nm", 1.0)

        self.motion = None
        self.ir_reader = None
        self.spectrometer = None
        self.jog_enabled = False
        self.jog_step_mm = JOG_STEP_DEFAULT_MM
        self._arrows_bound = False
        self._position_poll_job = None

        self.running = False
        self.q = None
        self.stop_event = None
        self.worker = None
        self._current_outdir = None
        self._cbar = None

        self._build_scroll_container()

        self.content.columnconfigure(0, weight=1)
        self.content.columnconfigure(1, weight=1)
        self.content.rowconfigure(3, weight=1)

        self._build_connection_row()
        self._build_jog_pad()
        self._build_param_form()
        self._build_log()
        self._build_results()

        self._set_jog_enabled(False)

    # ------------------------------------------------------------------
    # Scrollable container
    # ------------------------------------------------------------------

    def _build_scroll_container(self):
        """
        Standard Tk scrollable-frame pattern: a Canvas + Scrollbar owned
        by this Frame, with self.content (a plain ttk.Frame) embedded
        inside the canvas as the actual parent for every widget this tab
        builds. self.content's width is kept in sync with the canvas's
        visible width (so sticky="we"/"nsew" widgets inside it lay out
        correctly), and its scrollregion is kept in sync with its own
        requested height (so the scrollbar's range always matches the
        real content, including whenever the coarse-grid heatmap or log
        text grows the content taller after the panel is first built).
        """
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self._canvas = tk.Canvas(self, highlightthickness=0)
        vscroll = ttk.Scrollbar(self, orient="vertical", command=self._canvas.yview)
        self._canvas.configure(yscrollcommand=vscroll.set)
        self._canvas.grid(row=0, column=0, sticky="nsew")
        vscroll.grid(row=0, column=1, sticky="ns")

        self.content = ttk.Frame(self._canvas, padding=8)
        self._content_window = self._canvas.create_window((0, 0), window=self.content, anchor="nw")

        def _on_content_configure(event):
            self._canvas.configure(scrollregion=self._canvas.bbox("all"))

        def _on_canvas_configure(event):
            # Keep the embedded frame exactly as wide as the visible
            # canvas, so sticky="we"/"nsew" widgets inside it can actually
            # expand -- without this, self.content only ever gets its own
            # *requested* width, ignoring how much room the tab actually has.
            self._canvas.itemconfig(self._content_window, width=event.width)

        self.content.bind("<Configure>", _on_content_configure)
        self._canvas.bind("<Configure>", _on_canvas_configure)

        # Mouse-wheel scrolling, bound only while the pointer is actually
        # over this tab's canvas (not bind_all) -- so it doesn't hijack
        # scrolling on the Calibrate/Scan tabs or elsewhere in the window.
        def _on_mousewheel(event):
            self._canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        self._canvas.bind("<Enter>", lambda e: self._canvas.bind_all("<MouseWheel>", _on_mousewheel))
        self._canvas.bind("<Leave>", lambda e: self._canvas.unbind_all("<MouseWheel>"))

    # ------------------------------------------------------------------
    # Layout
    # ------------------------------------------------------------------

    def _build_connection_row(self):
        row = ttk.Frame(self.content)
        row.grid(row=0, column=0, columnspan=2, sticky="we")

        self.connect_btn = ttk.Button(row, text="Connect", command=self._handle_connect)
        self.connect_btn.grid(row=0, column=0)

        self.pos_var = tk.StringVar(value="Position: --")
        ttk.Label(row, textvariable=self.pos_var).grid(row=0, column=1, padx=(12, 0), sticky="w")

        self.live_signal_var = tk.StringVar(value="")
        ttk.Label(row, textvariable=self.live_signal_var).grid(row=1, column=1, padx=(12, 0), sticky="w")

    def _build_jog_pad(self):
        frame = ttk.LabelFrame(self.content, text="Jog to a valid wafer signal", padding=8)
        frame.grid(row=1, column=0, sticky="nw", pady=(8, 0))

        ttk.Label(frame, text="Jog step (mm)").grid(row=0, column=0, columnspan=2, sticky="w")
        self.jog_step_var = tk.StringVar(value=f"{JOG_STEP_DEFAULT_MM:.2f}")
        step_row = ttk.Frame(frame)
        step_row.grid(row=1, column=0, columnspan=2, sticky="w")
        step_entry = ttk.Entry(step_row, textvariable=self.jog_step_var, width=6)
        step_entry.grid(row=0, column=0)
        step_entry.bind("<Return>", self._apply_jog_step_entry)
        step_entry.bind("<FocusOut>", self._apply_jog_step_entry)
        ttk.Button(step_row, text="-", width=2, command=self._halve_jog_step).grid(row=0, column=1)
        ttk.Button(step_row, text="+", width=2, command=self._double_jog_step).grid(row=0, column=2)

        pad = ttk.Frame(frame)
        pad.grid(row=2, column=0, pady=(8, 0))
        ttk.Button(pad, text="↑", width=3, command=lambda: self._do_jog(0.0, 1.0)).grid(row=0, column=1)
        ttk.Button(pad, text="←", width=3, command=lambda: self._do_jog(-1.0, 0.0)).grid(row=1, column=0)
        ttk.Button(pad, text="→", width=3, command=lambda: self._do_jog(1.0, 0.0)).grid(row=1, column=2)
        ttk.Button(pad, text="↓", width=3, command=lambda: self._do_jog(0.0, -1.0)).grid(row=2, column=1)
        self.jog_buttons = list(pad.winfo_children())

    def _build_param_form(self):
        """
        The 7 operator parameters (spec §4) plus output directory.
        Deliberately blank StringVars (no placeholder value) for every
        field except coarse_grid_cells -- see this module's docstring and
        spec §4's defaults policy. A blank Entry left unfilled fails
        validation with a clear message rather than silently running with
        a guessed value.
        """
        frame = ttk.LabelFrame(self.content, text="Adaptive scan parameters", padding=8)
        frame.grid(row=1, column=1, sticky="new", pady=(8, 0))

        r = 0

        ttk.Label(frame, text="Wafer detection signal").grid(row=r, column=0, sticky="w")
        self.signal_var = tk.StringVar(value="")
        signal_names = available_signal_names(self.config)
        ttk.Combobox(
            frame, textvariable=self.signal_var, values=signal_names, state="readonly", width=18,
        ).grid(row=r, column=1, sticky="w")
        r += 1

        ttk.Label(frame, text="On-wafer threshold").grid(row=r, column=0, sticky="w")
        self.on_threshold_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.on_threshold_var, width=12).grid(row=r, column=1, sticky="w")
        r += 1

        ttk.Label(frame, text="Off-wafer threshold").grid(row=r, column=0, sticky="w")
        self.off_threshold_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.off_threshold_var, width=12).grid(row=r, column=1, sticky="w")
        r += 1

        ttk.Label(frame, text="Confirm count (N readings)").grid(row=r, column=0, sticky="w")
        self.confirm_count_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.confirm_count_var, width=12).grid(row=r, column=1, sticky="w")
        r += 1

        ttk.Label(frame, text="Reading interval mode").grid(row=r, column=0, sticky="w")
        self.interval_mode_var = tk.StringVar(value="")
        interval_combo = ttk.Combobox(
            frame, textvariable=self.interval_mode_var, values=list(READING_INTERVAL_MODES),
            state="readonly", width=14,
        )
        interval_combo.grid(row=r, column=1, sticky="w")
        interval_combo.bind("<<ComboboxSelected>>", self._on_interval_mode_change)
        r += 1

        ttk.Label(frame, text="Reading interval value").grid(row=r, column=0, sticky="w")
        self.interval_value_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.interval_value_var, width=12).grid(row=r, column=1, sticky="w")
        self.interval_hint_var = tk.StringVar(value="(select a mode above)")
        ttk.Label(frame, textvariable=self.interval_hint_var, foreground="#555").grid(
            row=r, column=2, sticky="w", padx=(6, 0))
        r += 1

        ttk.Label(frame, text="Y raster spacing (mm)").grid(row=r, column=0, sticky="w")
        self.y_spacing_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.y_spacing_var, width=12).grid(row=r, column=1, sticky="w")
        r += 1

        ttk.Label(frame, text="Max X travel (mm)").grid(row=r, column=0, sticky="w")
        self.max_x_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.max_x_var, width=12).grid(row=r, column=1, sticky="w")
        r += 1

        ttk.Label(frame, text="Max Y travel (mm)").grid(row=r, column=0, sticky="w")
        self.max_y_var = tk.StringVar(value="")
        ttk.Entry(frame, textvariable=self.max_y_var, width=12).grid(row=r, column=1, sticky="w")
        r += 1

        ttk.Label(frame, text="Coarse grid cell count").grid(row=r, column=0, sticky="w")
        self.coarse_cells_var = tk.StringVar(value=str(COARSE_GRID_CELLS_DEFAULT))
        ttk.Entry(frame, textvariable=self.coarse_cells_var, width=12).grid(row=r, column=1, sticky="w")
        r += 1

        ttk.Label(frame, text="Output directory").grid(row=r, column=0, sticky="w")
        self.outdir_var = tk.StringVar(value="./adaptive_scan_data")
        ttk.Entry(frame, textvariable=self.outdir_var, width=24).grid(
            row=r, column=1, columnspan=2, sticky="we")
        r += 1

        btn_row = ttk.Frame(frame)
        btn_row.grid(row=r, column=0, columnspan=3, pady=(10, 0), sticky="we")
        self.start_btn = ttk.Button(btn_row, text="Start Adaptive Scan", command=self._handle_start)
        self.start_btn.grid(row=0, column=0)
        self.abort_btn = ttk.Button(btn_row, text="Abort", command=self._handle_abort, state="disabled")
        self.abort_btn.grid(row=0, column=1, padx=(8, 0))

        self.progress_var = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.progress_var).grid(
            row=r + 1, column=0, columnspan=3, sticky="w", pady=(4, 0))

    def _build_log(self):
        ttk.Label(self.content, text="Log").grid(row=2, column=0, sticky="w", pady=(8, 0))
        self.log = ScrolledText(self.content, height=12, width=52, state="disabled")
        self.log.grid(row=3, column=0, sticky="nsew")

    def _build_results(self):
        ttk.Label(self.content, text="Coarse map (after scan completes)").grid(
            row=2, column=1, sticky="w", pady=(8, 0))
        self.figure = Figure(figsize=(3.8, 3.8), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title("No data yet")
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.content)
        self.canvas.get_tk_widget().grid(row=3, column=1, sticky="nsew")
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _log(self, text):
        self.log.config(state="normal")
        self.log.insert(tk.END, text + "\n")
        self.log.see(tk.END)
        self.log.config(state="disabled")

    # ------------------------------------------------------------------
    # Connect + live readout
    # ------------------------------------------------------------------

    def _handle_connect(self):
        try:
            self.motion = get_motion_controller(self.config)
            self.ir_reader = get_ir_reader(self.config)
            self.spectrometer = get_spectrometer_reader(self.config)
        except Exception as exc:
            messagebox.showerror("Connection failed", str(exc))
            return

        # Fiducial zero at wherever the stage currently is -- this mode
        # never trusts an absolute origin the way the precision path's
        # calibration does; it only needs get_position() to return
        # numbers relative to SOME fixed point so TravelGuard has a
        # reference to measure travel from (see adaptive_scan.py).
        self.motion.zero_here()
        self._log("Connected. Origin zeroed at current position.")
        self.connect_btn.config(state="disabled")
        self._set_jog_enabled(True)
        self._start_position_poll()

    def _start_position_poll(self):
        if self._position_poll_job is not None:
            return

        def _tick():
            self._refresh_readout()
            self._position_poll_job = self.after(POSITION_POLL_INTERVAL_MS, _tick)

        self._position_poll_job = self.after(POSITION_POLL_INTERVAL_MS, _tick)

    def _stop_position_poll(self):
        if self._position_poll_job is not None:
            self.after_cancel(self._position_poll_job)
            self._position_poll_job = None

    def _refresh_readout(self):
        if self.motion is None:
            return
        x, y = self.motion.get_position()
        self.pos_var.set(f"Position: {x:.3f}, {y:.3f} mm")

        # Best-effort live signal readout -- purely informational (lets
        # the operator confirm "this looks like a valid wafer signal"
        # before clicking Start, per spec §3 step 1). A transient read
        # failure here shouldn't interrupt jogging, so any exception is
        # swallowed and just leaves the previous reading on screen.
        try:
            raw = read_raw_signals(self.ir_reader, self.spectrometer,
                                    self.oes_features_cfg, self.feature_window_nm)
            dilution_str = "--" if raw.ir_dilution is None else f"{raw.ir_dilution:.3f}"
            self.live_signal_var.set(
                f"temp={raw.ir_temp_c:.1f}C  emissivity={raw.ir_emissivity:.3f}  dilution={dilution_str}"
            )
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Jogging
    # ------------------------------------------------------------------

    def _set_jog_enabled(self, enabled):
        self.jog_enabled = enabled
        state = "normal" if enabled else "disabled"
        for btn in self.jog_buttons:
            btn.config(state=state)
        if enabled:
            self._bind_arrows()
        else:
            self._unbind_arrows()

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
            return  # let the focused entry handle cursor movement
        dx_sign, dy_sign = ARROW_KEYSYMS[event.keysym]
        self._do_jog(dx_sign, dy_sign)

    def _do_jog(self, dx_sign, dy_sign):
        if not self.jog_enabled or self.motion is None:
            return
        self.motion.jog(dx_mm=dx_sign * self.jog_step_mm, dy_mm=dy_sign * self.jog_step_mm)
        self._refresh_readout()

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
    # Parameter form helpers
    # ------------------------------------------------------------------

    def _on_interval_mode_change(self, event=None):
        mode = self.interval_mode_var.get()
        if mode == "time_s":
            self.interval_hint_var.set("(seconds between readings)")
        elif mode == "motor_pulses":
            self.interval_hint_var.set("(motor pulses -- direction-sensitive, see docs §7)")
        else:
            self.interval_hint_var.set("(select a mode above)")

    # ------------------------------------------------------------------
    # Start / Abort
    # ------------------------------------------------------------------

    def _handle_start(self):
        if self.running:
            return
        if self.motion is None:
            messagebox.showerror("Not connected", "Click Connect first, then jog to a valid wafer signal.")
            return

        try:
            params = AdaptiveScanParams.from_operator_input(
                self.config,
                signal_name=self.signal_var.get(),
                on_threshold=self.on_threshold_var.get(),
                off_threshold=self.off_threshold_var.get(),
                confirm_count=self.confirm_count_var.get(),
                reading_interval_mode=self.interval_mode_var.get(),
                reading_interval_value=self.interval_value_var.get(),
                y_raster_spacing_mm=self.y_spacing_var.get(),
                max_x_travel_mm=self.max_x_var.get(),
                max_y_travel_mm=self.max_y_var.get(),
                coarse_grid_cells=self.coarse_cells_var.get(),
            )
        except (ValueError, TypeError) as exc:
            messagebox.showerror("Invalid parameters", str(exc))
            return

        outdir = self.outdir_var.get().strip() or "./adaptive_scan_data"
        self._current_outdir = outdir
        output_path = os.path.join(outdir, "raw_readings.csv")

        # Scanning and jogging both drive the SAME motion object -- disable
        # the jog pad for the duration so the operator can't issue a manual
        # jog while AdaptiveRasterScanner is mid-row (same reasoning
        # ControlPanel's Start button disabling has for the precision path).
        self._set_jog_enabled(False)
        self._stop_position_poll()

        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self._reset_results()
        self._log(f"Starting adaptive scan -> {output_path}")
        self._set_running(True)

        self.worker = threading.Thread(
            target=run_adaptive_scan,
            args=(self.config, params, self.motion, self.ir_reader, self.spectrometer,
                  output_path, self.q, self.stop_event),
            daemon=True,
        )
        self.worker.start()
        self._poll_queue()

    def _handle_abort(self):
        if not self.running or self.stop_event is None:
            return
        self.stop_event.set()
        self._log("Abort requested — stopping after the current row.")

    def _set_running(self, running):
        self.running = running
        self.start_btn.config(state="disabled" if running else "normal")
        self.abort_btn.config(state="normal" if running else "disabled")

    def _reset_results(self):
        self._rows_done = 0
        self._readings_done = 0
        self.progress_var.set("Rows: 0   Readings: 0")
        self.ax.clear()
        self.ax.set_title("Scan in progress...")
        if self._cbar is not None:
            self._cbar.remove()
            self._cbar = None
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Queue draining (Tk mainloop thread only, per module docstring)
    # ------------------------------------------------------------------

    def _poll_queue(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                self._handle_message(kind, payload)
        except queue.Empty:
            pass

        if self.running:
            self.after(POLL_INTERVAL_MS, self._poll_queue)

    def _handle_message(self, kind, payload):
        if kind == "row":
            row_summary = payload["row_summary"]
            self._rows_done += 1
            self._readings_done += payload["n_readings"]
            self.progress_var.set(f"Rows: {self._rows_done}   Readings: {self._readings_done}")
            self._log(
                f"Row {row_summary.row_number} ({row_summary.scan_direction}): "
                f"{payload['n_readings']} readings"
            )
        elif kind == "done":
            self._on_finished(payload, aborted=False)
        elif kind == "aborted":
            self._on_finished(payload, aborted=True)
        elif kind == "error":
            self._set_running(False)
            self._set_jog_enabled(True)
            self._start_position_poll()
            self._log("ERROR — see dialog for full traceback.")
            messagebox.showerror("Adaptive scan error", payload)

    def _on_finished(self, result, aborted):
        self._set_running(False)
        self._set_jog_enabled(True)
        self._start_position_poll()

        status_word = "ABORTED" if aborted else "COMPLETE"
        self._log(f"Scan {status_word} — {len(result.rows)} row(s), "
                  f"{len(result.readings)} reading(s) total.")

        flagged = {rn: fl for rn, fl in result.row_flags.items() if fl}
        if flagged:
            self._log(f"{len(flagged)} row(s) flagged:")
            for rn, fl in sorted(flagged.items()):
                self._log(f"  row {rn}: {', '.join(fl)}")
        else:
            self._log("No rows flagged.")

        if result.coarse_grid.get("mean") is not None:
            self._render_heatmap(result.coarse_grid)
            try:
                grid_path = os.path.join(self._current_outdir, "coarse_grid.csv")
                save_coarse_grid_csv(result.coarse_grid, grid_path)
                self._log(f"Coarse grid saved to {grid_path}")
            except Exception as exc:
                self._log(f"Could not save coarse grid CSV: {exc}")
        else:
            self._log("No readings collected — nothing to grid.")

    def _render_heatmap(self, coarse_grid):
        """
        mean_grid is indexed [cell_ix, cell_iy] (see build_coarse_grid) --
        transposed here so imshow's (row, col) convention lines up with
        (y, x), matching gui/live_map.py's own orientation for the
        precision path's live map.
        """
        mean_grid = coarse_grid["mean"]
        self.ax.clear()
        im = self.ax.imshow(mean_grid.T, origin="lower", cmap="inferno", aspect="equal")
        self.ax.set_xlabel("normalized X (row-relative)")
        self.ax.set_ylabel("normalized Y (row rank)")
        self.ax.set_title(f"Coarse map ({coarse_grid['n_side']}x{coarse_grid['n_side']} cells)")
        if self._cbar is not None:
            self._cbar.remove()
        self._cbar = self.figure.colorbar(im, ax=self.ax)
        self.canvas.draw_idle()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self):
        """
        Called by app.py's on_close(). Best-effort: request a stop if a
        scan is running and give it a moment, then release the hardware
        connection this tab opened -- mirrors CalibrationPanel.shutdown().
        """
        if self.running and self.stop_event is not None:
            self.stop_event.set()
            if self.worker is not None:
                self.worker.join(timeout=5)

        self._stop_position_poll()
        self._unbind_arrows()

        if self.motion is not None:
            close = getattr(self.motion, "close", None)
            if callable(close):
                close()
            self.motion = None
