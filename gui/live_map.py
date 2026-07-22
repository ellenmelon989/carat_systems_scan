"""
gui/live_map.py

Embedded matplotlib canvas showing the IR temperature map filling in
live, one cell per completed grid point. This is a rough in-progress
preview only -- the polished maps (OES feature maps, ratio maps, the
full IR map) still come from map_plotter.py run against the finished
scan_summary.csv once the scan is done.

Uses matplotlib's Figure/FigureCanvasTkAgg directly rather than
pyplot, so embedding here never touches global pyplot state that
map_plotter.py (or anything else in the process) might rely on.
"""

import numpy as np
from tkinter import ttk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from scan.scan_params import grid_dims_from_range


class LiveMapPanel(ttk.Frame):
    def __init__(self, parent, x_range_mm, y_range_mm, step_size_mm):
        super().__init__(parent, padding=8)

        self.figure = Figure(figsize=(4.5, 4.5), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self.im = None
        # NOT named self.grid -- ttk.Frame (every Tk widget) already has
        # an inherited .grid(...) method, which is the layout call
        # app.py makes on this widget (self.live_map.grid(row=..., ...)).
        # An instance attribute of that name silently shadows the
        # method instead of erroring, so app.py's layout call ends up
        # trying to call a numpy array -- TypeError: 'numpy.ndarray'
        # object is not callable. value_grid sidesteps the collision.
        self.value_grid = None
        self.nx = self.ny = 0
        self.reset(x_range_mm, y_range_mm, step_size_mm)

    def reset(self, x_range_mm, y_range_mm, step_size_mm):
        """
        Rebuild the backing array and redraw a blank map. Called every
        time a scan starts (app.start_scan) rather than once at __init__
        -- step_size_mm is operator-editable per run, so nx/ny (and
        therefore the array shape) can change between runs even though
        x_range_mm/y_range_mm stay fixed by calibration.
        """
        x0, x1 = x_range_mm
        y0, y1 = y_range_mm
        self.nx, self.ny = grid_dims_from_range((x0, x1), (y0, y1), step_size_mm)
        self.value_grid = np.full((self.ny, self.nx), np.nan)

        self.ax.clear()
        self.im = self.ax.imshow(
            self.value_grid, origin="lower", extent=[x0, x1, y0, y1],
            aspect="equal", cmap="inferno",
        )
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")
        self.ax.set_title("IR temperature — live")
        self.canvas.draw_idle()

    def update_point(self, record):
        ix, iy = record.get("ix"), record.get("iy")
        if ix is None or iy is None:
            return  # reference-point revisit -- not part of the spatial grid

        self.value_grid[iy, ix] = record.get("ir_temp_c", np.nan)

        valid = self.value_grid[~np.isnan(self.value_grid)]
        if valid.size:
            self.im.set_clim(vmin=float(valid.min()), vmax=float(valid.max()))
        self.im.set_data(self.value_grid)
        self.canvas.draw_idle()
