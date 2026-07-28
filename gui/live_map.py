"""
gui/live_map.py

Embedded matplotlib canvas showing the IR temperature map filling in
live as the scan proceeds. This is a rough in-progress preview only --
the polished maps (OES feature maps, ratio maps, the full IR map)
still come from map_plotter.py run against the finished
scan_summary.csv once the scan is done.

Binning/averaging math lives in scan/live_map_accumulator.py (see that
module's docstring), NOT here -- this file only owns the tkinter/
matplotlib display. That split is deliberate: MapAccumulator.add_reading()
takes a plain (x_mm, y_mm, value) triple and doesn't know or care
whether the coordinates came from the commanded motor position (today)
or an encoder readback (once the CONEX-AG-M100D mount arrives -- see
memory: carat_scanner_hardware_status). Swapping the coordinate source
later means changing what update_point() is handed, not this class.

Uses matplotlib's Figure/FigureCanvasTkAgg directly rather than
pyplot, so embedding here never touches global pyplot state that
map_plotter.py (or anything else in the process) might rely on.
"""

import numpy as np
import tkinter as tk
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from scan.live_map_accumulator import MapAccumulator


class LiveMapPanel(ttk.Frame):
    def __init__(self, parent, x_range_mm, y_range_mm, step_size_mm):
        super().__init__(parent, padding=8)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self.figure = Figure(figsize=(4.5, 4.5), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

        # NOT self.grid (attribute) -- ttk.Frame (every Tk widget) already
        # has an inherited .grid(...) METHOD, which is the layout call
        # app.py makes on this widget (self.live_map.grid(row=..., ...)).
        # An instance attribute of that name would silently shadow the
        # method instead of erroring. self.accumulator (a MapAccumulator,
        # see scan/live_map_accumulator.py) holds the actual grid arrays
        # instead, sidestepping the collision entirely.
        self.accumulator = None
        self.im = None
        self._cbar = None

        # Lets the operator flip between "average temperature per cell"
        # (the main product) and "how many reads landed in each cell" --
        # Roy's 2026-07-27 ask to monitor for surprises (e.g. a cell hit
        # far more often than the configured pass count would suggest a
        # binning/positioning problem worth a second look).
        self.show_counts_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            self, text="Show read counts per cell (instead of temperature)",
            variable=self.show_counts_var, command=self._redraw,
        ).grid(row=1, column=0, sticky="w", pady=(6, 0))

        self.reset(x_range_mm, y_range_mm, step_size_mm)

    def reset(self, x_range_mm, y_range_mm, step_size_mm):
        """
        Rebuild the backing accumulator and redraw a blank map. Called
        every time a scan starts (app.start_scan) rather than once at
        __init__ -- step_size_mm is operator-editable per run, so the
        grid shape can change between runs even though x_range_mm/
        y_range_mm stay fixed by calibration.
        """
        self.accumulator = MapAccumulator(x_range_mm, y_range_mm, step_size_mm)
        x0, x1 = x_range_mm
        y0, y1 = y_range_mm

        # Remove any existing colorbar BEFORE clearing self.ax, not after.
        # self.ax.clear() on newer matplotlib implicitly detaches/removes
        # any colorbar already attached to that axes as a side effect of
        # clearing it. The previous order (clear() first, then
        # self._cbar.remove()) tried to explicitly remove an axes
        # matplotlib had already torn down, which surfaced as either
        # AttributeError: 'NoneType' object has no attribute
        # 'set_subplotspec' or KeyError: <Axes: label='<colorbar>'>
        # depending on exactly how far the stale internal state got before
        # failing -- both are the same root cause.
        if self._cbar is not None:
            self._cbar.remove()
            self._cbar = None

        self.ax.clear()
        self.im = self.ax.imshow(
            self.accumulator.value_grid, origin="lower", extent=[x0, x1, y0, y1],
            aspect="equal", cmap="inferno",
        )
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")
        self.ax.set_title("IR temperature — live")

        self._cbar = self.figure.colorbar(self.im, ax=self.ax)
        self._cbar.set_label("°C")

        self.canvas.draw_idle()

    def update_point(self, record):
        """
        Feed one scan-point record into the shared accumulator and
        redraw. Only reads x_mm/y_mm/ir_temp_c off `record` -- see this
        module's docstring for why that's the whole point (coordinate
        source can change later without touching this method).

        Reference-point revisits (record["is_reference"]) are skipped --
        not part of the spatial grid, same convention DataLogger/
        OESStore/StatusPanel already use for them.
        """
        if record.get("is_reference"):
            return
        self.accumulator.add_reading(
            record.get("x_mm"), record.get("y_mm"), record.get("ir_temp_c"),
        )
        self._redraw()

    def _redraw(self):
        if self.accumulator is None:
            return

        if self.show_counts_var.get():
            grid = self.accumulator.read_count.astype(float)
            grid = np.where(grid == 0, np.nan, grid)  # unvisited cells stay blank
            self.ax.set_title("IR read count per cell — live")
            self._cbar.set_label("reads")
        else:
            grid = self.accumulator.value_grid
            self.ax.set_title("IR temperature — live")
            self._cbar.set_label("°C")

        valid = grid[~np.isnan(grid)]
        if valid.size:
            self.im.set_clim(vmin=float(valid.min()), vmax=float(valid.max()))
        self.im.set_data(grid)
        self.canvas.draw_idle()
