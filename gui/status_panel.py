"""
gui/status_panel.py

Live scan status: point counter, last position/reading, elapsed time,
and a scrolling event log. Purely a display -- every value shown here
comes from a record handed to update_point() by app.py's poll_queue,
which only ever runs on the Tk mainloop thread (see the threading
contract in gui/scan_worker.py). This module never touches the queue,
the worker thread, or ScanManager directly.
"""

import math
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText


class StatusPanel(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent, padding=8)
        self.total_points = 0
        self.points_done = 0

        self.point_var = tk.StringVar(value="Point: -- / --")
        self.pos_var = tk.StringVar(value="Position: --")
        self.ir_var = tk.StringVar(value="Last IR: --")
        self.elapsed_var = tk.StringVar(value="Elapsed: --")

        ttk.Label(self, textvariable=self.point_var).grid(row=0, column=0, sticky="w")
        ttk.Label(self, textvariable=self.pos_var).grid(row=1, column=0, sticky="w")
        ttk.Label(self, textvariable=self.ir_var).grid(row=2, column=0, sticky="w")
        ttk.Label(self, textvariable=self.elapsed_var).grid(row=3, column=0, sticky="w")

        self.log = ScrolledText(self, height=16, width=58, state="disabled")
        self.log.grid(row=4, column=0, pady=(8, 0), sticky="nsew")

    def reset(self, total_points):
        """Called once per scan start (app.start_scan), before the worker
        thread is spun up -- so the counter/log are clean before the
        first "point" message can possibly arrive."""
        self.total_points = total_points
        self.points_done = 0
        self.point_var.set(f"Point: 0 / {total_points}")
        self.pos_var.set("Position: --")
        self.ir_var.set("Last IR: --")
        self.elapsed_var.set("Elapsed: 0s")

        self.log.config(state="normal")
        self.log.delete("1.0", tk.END)
        self.log.config(state="disabled")
        self.log_message(f"Scan starting — {total_points} grid points queued.")

    def update_point(self, record):
        is_reference = bool(record.get("is_reference"))
        tag = "REF" if is_reference else "PT"

        # Reference-point revisits are extra, outside the planned grid
        # total (see scan_manager.run's ref_every logic) -- only count
        # real grid points toward the "N / total" display.
        if not is_reference:
            self.points_done += 1
            self.point_var.set(f"Point: {self.points_done} / {self.total_points}")

        x_mm = record.get("x_mm")
        y_mm = record.get("y_mm")
        self.pos_var.set(f"Position: {x_mm:.2f}, {y_mm:.2f} mm")

        ir_val = record.get("ir_temp_c")
        if ir_val is None or (isinstance(ir_val, float) and math.isnan(ir_val)):
            ir_str = "NaN (read error)"
        else:
            ir_str = f"{ir_val:.1f} C"
        self.ir_var.set(f"Last IR: {ir_str}")

        # motion_error means the move itself failed (axis timeout/stall/
        # comm fault) -- ir_temp_c will already be NaN in that case (see
        # scan_manager._measure_point), but that alone reads like an
        # ordinary IR read failure. Flag it explicitly so the operator
        # can tell "sensor hiccup" apart from "stage didn't get there,"
        # which needs a different reaction.
        fault_tag = " *** MOTION FAULT ***" if record.get("motion_error") else ""

        self.log_message(
            f"[{tag}] pt {record.get('point_id')} "
            f"(x={x_mm:.2f}, y={y_mm:.2f}) IR={ir_str} "
            f"ir_err={record.get('ir_error')} "
            f"oes_err={record.get('oes_error')} sat={record.get('oes_saturated')}"
            f"{fault_tag}"
        )

    def set_elapsed(self, seconds):
        self.elapsed_var.set(f"Elapsed: {int(seconds)}s")

    def log_message(self, text):
        self.log.config(state="normal")
        self.log.insert(tk.END, text + "\n")
        self.log.see(tk.END)
        self.log.config(state="disabled")
