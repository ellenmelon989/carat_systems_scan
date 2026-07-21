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
ever gets the config dict, the queue, and the stop_event.
"""

import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox

from gui.control_panel import ControlPanel
from gui.status_panel import StatusPanel
from gui.live_map import LiveMapPanel
from gui.scan_worker import run_scan
from scan_manager import generate_grid
from scan_params import PASSES_DEFAULT, validate_passes

POLL_INTERVAL_MS = 150


class App(tk.Tk):
    def __init__(self, config):
        super().__init__()
        self.title("carat_scanner")

        self.base_config = config  # named to avoid shadowing tk.Tk's own .config()
        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.worker = None
        self.running = False
        self.scan_start_time = None

        self.control = ControlPanel(self, config, on_start=self.start_scan, on_abort=self.abort_scan)
        self.status = StatusPanel(self)
        self.live_map = LiveMapPanel(
            self,
            config["scan"]["grid"]["x_range_mm"],
            config["scan"]["grid"]["y_range_mm"],
            config["scan"]["grid"]["step_size_mm"],
        )

        self.control.grid(row=0, column=0, sticky="n")
        self.status.grid(row=1, column=0, sticky="n")
        self.live_map.grid(row=0, column=1, rowspan=2, sticky="nsew")
        self.columnconfigure(1, weight=1)
        self.rowconfigure(1, weight=1)

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(POLL_INTERVAL_MS, self.poll_queue)

    def start_scan(self, effective_config):
        if self.running:
            return  # Start button is disabled while running, but guard anyway

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

        self.worker = threading.Thread(
            target=run_scan,
            args=(effective_config, self.q, self.stop_event),
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
        self.destroy()
