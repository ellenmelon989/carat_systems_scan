"""
gui/app.py

Top-level Tk application. Owns the queue.Queue / threading.Event pair
that bridge the worker thread (gui/scan_worker.py, running
ScanManager) and the GUI thread, plus the root.after() poll loop that
drains the queue. See memory: carat_scanner_gui_scan_connection for
the full threading contract this relies on.

Rule enforced throughout this module: only this file and the panel
widgets it owns ever touch a tkinter object, and only ever from a
method Tk itself calls on the mainloop thread (__init__, the
button-command callbacks routed through here, and poll_queue). The
worker thread never receives a reference to any widget -- it only
ever gets the config dict, the queue, the stop_event, and (see below)
an optional already-connected motion controller.

Four tabs, one window: "Calibrate" (gui/calibration_panel.py), "Scan"
(the ControlPanel/StatusPanel/LiveMapPanel trio that already existed
here), "Line Scan" (gui/line_scan_panel.py, PR2b -- an arbitrary-angle
1D line scan, added 2026-09-08), and "Adaptive Scan"
(gui/adaptive_scan_panel.py). Calibration, Scan, and Line Scan all
share this one App instance's motion hand-off so a motion controller
connected and homed on the Calibrate tab can be handed straight to a
scan -- either kind -- without a second hardware connection or a
redundant re-home. See _handle_calibrated()/_take_shared_motion()
below and ScanManager's motion=/already_homed= params; Line Scan
consumes this exactly the way start_scan() does, via
_take_shared_motion() (see gui/line_scan_panel.py's docstring for why
it shares this hand-off rather than opening its own connection, unlike
Adaptive Scan). Adaptive Scan does NOT participate in that hand-off --
it opens its own, independent motion/reader connection (see
gui/adaptive_scan_panel.py's docstring for why) -- so it only needs its
own shutdown() called alongside the Calibrate tab's, not a place in the
Calibrate -> Scan/Line Scan hand-off chain.
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

from gui.calibration_panel import CalibrationPanel
from gui.control_panel import ControlPanel
from gui.status_panel import StatusPanel
from gui.live_map import LiveMapPanel
from gui.scan_worker import run_scan
from gui.adaptive_scan_panel import AdaptiveScanPanel
from gui.line_scan_panel import LineScanPanel
from gui.background_panel import BackgroundPanel
from scan.scan_manager import generate_grid
from scan.scan_params import PASSES_DEFAULT, validate_passes

POLL_INTERVAL_MS = 150


class App(tk.Tk):
    def __init__(self, config, config_path):
        super().__init__()
        self.title("carat_scanner")

        self.base_config = config  # named to avoid shadowing tk.Tk's own .config()
        self.config_path = config_path
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.worker = None
        self.running = False
        self.scan_start_time = None

        # Motion controller handed off from the Calibrate tab once
        # calibration finishes -- None until then, in which case
        # scan_worker/ScanManager fall back to building their own (exactly
        # today's behavior, unaffected by any of this). See
        # _handle_calibrated() and ScanManager's `motion=` param.
        self.motion = None
        # One-shot flag: True only for the very next start_scan() (or
        # LineScanPanel start, via _take_shared_motion()) call right
        # after a hand-off, then consumed (reset False) regardless of
        # outcome -- a second scan later in the same session, on either
        # tab, goes through ScanManager's normal rehome-at-start logic
        # instead of skipping it forever just because a motion object is
        # being reused.
        self._pending_already_homed = False

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=0, column=0, sticky="nsew")

        self.calibration = CalibrationPanel(
            self.notebook, self.base_config, self.config_path,
            on_calibrated=self._handle_calibrated,
        )
        self.notebook.add(self.calibration, text="Calibrate")

        scan_tab = ttk.Frame(self.notebook)
        self.notebook.add(scan_tab, text="Scan")
        scan_tab.columnconfigure(1, weight=1)
        scan_tab.rowconfigure(1, weight=1)

        self.control = ControlPanel(scan_tab, config, on_start=self.start_scan, on_abort=self.abort_scan)
        self.status = StatusPanel(scan_tab)
        self.live_map = LiveMapPanel(
            scan_tab,
            config["scan"]["grid"]["x_range_mm"],
            config["scan"]["grid"]["y_range_mm"],
            config["scan"]["grid"]["step_size_mm"],
        )

        self.control.grid(row=0, column=0, sticky="n")
        self.status.grid(row=1, column=0, sticky="n")
        self.live_map.grid(row=0, column=1, rowspan=2, sticky="nsew")

        # PR2b (2026-09-08): shares the Calibrate/Scan motion hand-off
        # via _take_shared_motion() -- see this class's own docstring and
        # gui/line_scan_panel.py's for why.
        self.line_scan = LineScanPanel(self.notebook, config, take_shared_motion=self._take_shared_motion)
        self.notebook.add(self.line_scan, text="Line Scan")

        # Batch 2 (2026-10-19): capture/save OES backgrounds. Added AFTER
        # Line Scan so existing tab indices (notebook.select(1) == Scan)
        # don't move.
        self.background = BackgroundPanel(
            self.notebook, config,
            can_capture=self._spectrometer_free_for_capture,
            get_scan_settings=self.control.oes_settings.values,
            on_saved=self._refresh_background_pickers,
        )
        self.notebook.add(self.background, text="OES Background")

        # Independent of the Calibrate/Scan hand-off -- see this class's
        # own docstring and gui/adaptive_scan_panel.py's for why.
        self.adaptive_scan = AdaptiveScanPanel(self.notebook, config)
        self.notebook.add(self.adaptive_scan, text="Adaptive Scan")

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(POLL_INTERVAL_MS, self.poll_queue)

    def _handle_calibrated(self, config, motion, already_homed):
        """
        CalibrationPanel's on_calibrated callback, fired when the operator
        clicks "Proceed to Scan". `config` is config.yaml re-read from
        disk after the Calibrate tab patched it (so it reflects exactly
        what write_results() wrote -- range, wafer center/radius, step
        size, dwell time, passes, and deg_per_mm_x/y if recalibrated).
        `motion` is that tab's already-connected, already-homed
        controller; adopting it here (rather than discarding it) is what
        lets the very next scan -- Scan tab or Line Scan tab -- skip a
        redundant second home().
        """
        self.motion = motion
        self._pending_already_homed = already_homed

        self.control.load_config(config)
        self.line_scan.load_config(config)
        self.background.load_config(config)
        grid_cfg = config["scan"]["grid"]
        self.live_map.reset(grid_cfg["x_range_mm"], grid_cfg["y_range_mm"], grid_cfg["step_size_mm"])

        self.notebook.select(1)  # jump to the Scan tab
        self.status.log_message("Calibration complete — scan parameters refreshed from config.yaml.")

    def _refresh_background_pickers(self):
        self.control.bg_selector.refresh()
        self.line_scan.bg_selector.refresh()

    def _spectrometer_free_for_capture(self) -> bool:
        """Background capture needs the spectrometer to itself."""
        if self.running or getattr(self.line_scan, "running", False):
            messagebox.showerror("Scan running",
                                 "A scan is using the spectrometer. Wait for it to "
                                 "finish (or abort it) before capturing a background.")
            return False
        if self.adaptive_scan.holds_connection():
            if getattr(self.adaptive_scan, "running", False):
                messagebox.showerror("Adaptive scan running",
                                     "Stop the adaptive scan before capturing a background.")
                return False
            if not messagebox.askyesno(
                    "Adaptive Scan connected",
                    "The Adaptive Scan tab holds the spectrometer.\n\n"
                    "Disconnect it so the background can be captured?"):
                return False
            self.adaptive_scan.release_connections()
        return True

    def _take_shared_motion(self):
        """
        Called by start_scan() and LineScanPanel right before a scan
        starts. Returns (ok, motion, already_homed); ok=False means "don't
        start" (the operator was told why).

        Batch 1 (2026-10-14) -- COM7 "access denied" fixes:
          1. If the Calibrate tab still holds the motion controller
             (operator never clicked "Proceed to Scan"), it is released
             here instead of the scan opening the port a second time:
             a finished calibration is handed off (its results loaded
             into the scan tabs; operator reviews and presses Start
             again), an unfinished one is cancelled after confirmation.
          2. If the Adaptive Scan tab is connected, it is disconnected
             after confirmation (it holds motion AND the spectrometer).
          3. OWNERSHIP of the shared motion connection now passes to the
             scan: self.motion is cleared here, and the scan closes it,
             together with the spectrometer and IR reader, when it
             finishes (gui/scan_worker.py). The next scan opens a fresh
             connection and homes normally.
        already_homed is still one-shot.
        """
        if getattr(self.background, "running", False):
            messagebox.showerror("Background capture running",
                                 "A background capture is using the spectrometer. "
                                 "Wait for it to finish before starting a scan.")
            return False, None, False
        if self.calibration.holds_connection():
            if self.calibration.finished_but_not_handed_off():
                current_tab = self.notebook.select()
                self.calibration.release_for_scan()   # -> _handle_calibrated()
                self.notebook.select(current_tab)     # stay on the tab the operator used
                messagebox.showinfo(
                    "Calibration results loaded",
                    "The calibration you just finished has been loaded into the "
                    "scan tabs and its motion connection handed over.\n\n"
                    "Check the scan settings, then press Start again.")
                return False, None, False
            if not messagebox.askyesno(
                "Calibration in progress",
                "The Calibrate tab is still connected to the motion controller.\n\n"
                "Cancel the calibration and release the connection so the scan "
                "can use it?"):
                return False, None, False
            self.calibration.release_for_scan()
        if self.adaptive_scan.holds_connection():
            if getattr(self.adaptive_scan, "running", False):
                messagebox.showerror(
                    "Adaptive scan running",
                    "An adaptive scan is running. Stop it before starting this scan.")
                return False, None, False
            if not messagebox.askyesno(
                "Adaptive Scan connected",
                "The Adaptive Scan tab is connected to the motion controller and "
                "spectrometer.\n\nDisconnect it so this scan can use them?"):
                return False, None, False
            self.adaptive_scan.release_connections()

        motion = self.motion
        already_homed = self._pending_already_homed
        self.motion = None                 # ownership passes to the scan
        self._pending_already_homed = False
        return True, motion, already_homed

    def start_scan(self, effective_config):
        if self.running:
            return  # Start button is disabled while running, but guard anyway

        # Resolve who holds the hardware BEFORE touching any UI state, so
        # a declined/handed-off start leaves the tab exactly as it was.
        ok, motion, already_homed = self._take_shared_motion()
        if not ok:
            return

        self.stop_event = threading.Event()  # fresh Event per run -- never reuse a fired one

        grid_cfg = effective_config["scan"]["grid"]
        self.live_map.reset(grid_cfg["x_range_mm"], grid_cfg["y_range_mm"], grid_cfg["step_size_mm"])

        # total_points must come from the ACTUAL point list, not nx*ny:
        # a circular wafer_radius_mm mask drops off-wafer corner points
        # (generate_grid skips them entirely), and scan.passes repeats
        # the whole masked list that many times. Either one alone makes
        # nx*ny wrong; both together compound. Reference-point revisits
        # are intentionally excluded here too, matching how
        # status_panel's own counter already treats them (extra, not
        # part of the planned total).
        points, _, _ = generate_grid(effective_config["scan"])
        passes = validate_passes(effective_config["scan"].get("passes", PASSES_DEFAULT))
        self.status.reset(total_points=len(points) * passes)

        self.scan_start_time = time.time()
        self.running = True
        self.control.set_running(True)

        # Reuse the Calibrate tab's motion connection if one was handed
        # off (see _handle_calibrated) -- None falls back to
        # scan_worker/ScanManager building a fresh one, unchanged from
        # before this tab existed. already_homed is consumed here
        # (one-shot) regardless of whether the scan actually starts
        # successfully, so it can never apply to a later scan.
        self.worker = threading.Thread(
            target=run_scan,
            args=(effective_config, self.q, self.stop_event),
            kwargs={"motion": motion, "already_homed": already_homed},
            daemon=True,
        )
        self.worker.start()

    def abort_scan(self):
        if not self.running:
            return
        self.stop_event.set()
        self.status.log_message("Abort requested — stopping after the current point.")

    def poll_queue(self):
        try:
            while True:
                kind, payload = self.q.get_nowait()
                self._handle_message(kind, payload)
        except queue.Empty:
            pass

        if self.running and self.scan_start_time is not None:
            self.status.set_elapsed(time.time() - self.scan_start_time)

        self.after(POLL_INTERVAL_MS, self.poll_queue)

    def _handle_message(self, kind, payload):
        if kind == "point":
            self.status.update_point(payload)
            self.live_map.update_point(payload)
        elif kind == "done":
            self.running = False
            self.control.set_running(False)
            self.status.log_message("Scan complete.")
        elif kind == "aborted":
            self.running = False
            self.control.set_running(False)
            self.status.log_message("Scan aborted.")
        elif kind == "warning":
            self.status.log_message(f"WARNING: {payload}")
            messagebox.showwarning("Scan warning", payload)
        elif kind == "error":
            self.running = False
            self.control.set_running(False)
            self.status.log_message("ERROR — see dialog for full traceback.")
            messagebox.showerror("Scan error", payload)

    def on_close(self):
        # Best-effort clean stop rather than killing the process mid-move:
        # set the flag and give the worker a few seconds to notice and
        # return from run() before we tear the window down.
        if self.running:
            self.stop_event.set()
            if self.worker is not None:
                self.worker.join(timeout=5)

        # Release any hardware connection left open -- either a
        # calibration that never got handed off to a scan, or (if no scan
        # ever consumed it) the one this App itself is holding, or the
        # Adaptive Scan tab's own independent connection (see that
        # panel's docstring for why it's not part of this same hand-off).
        # Line Scan is stopped here too (its worker may still be using
        # the shared self.motion below), but -- unlike Adaptive Scan --
        # it never owns that connection, so its shutdown() only stops a
        # running scan and never closes anything itself; see
        # gui/line_scan_panel.py's shutdown() docstring for why.
        self.calibration.shutdown()
        self.line_scan.shutdown()
        self.adaptive_scan.shutdown()
        self.background.shutdown()
        if self.motion is not None:
            close = getattr(self.motion, "close", None)
            if callable(close):
                close()
            self.motion = None

        self.destroy()
