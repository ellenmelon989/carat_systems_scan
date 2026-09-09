"""
scan/live_map_accumulator.py

Coordinate-source-agnostic accumulator for a live 2D map: bins
(x_mm, y_mm, value) readings into a fixed rectangular grid and keeps a
running mean plus a raw read count per cell, updated incrementally as
readings arrive.

Deliberately doesn't know or care where x_mm/y_mm came from. Today
that's the COMMANDED motor position ScanManager._measure_point moved
to (threaded through build_point_record's x_mm/y_mm and out to
gui/scan_worker.py's on_point records). When the CONEX-AG-M100D
encoder-based mount arrives (see memory:
carat_scanner_hardware_status), the plan is to feed this same
add_reading(x_mm, y_mm, value) call with the ENCODER's reported
position instead of the commanded target -- no change needed here, in
gui/live_map.py, or in gui/app.py; only whatever feeds add_reading's
x_mm/y_mm needs to change. That's the whole point of keeping this
class ignorant of hardware: "the scan supplies an X, a Y, and a
measured value to a common mapping function" per Roy's 2026-07-27
request, regardless of which positioning system produced the X/Y.

Binning: nearest-cell snap against the SAME xs/ys linspace
scan.scan_manager.generate_grid() and scan.oes_store.OESStore build
their grids from (see scan_params.grid_dims_from_range) -- so a
reading commanded at exactly a grid point lands in exactly that cell
today, matching the old ix/iy-indexed behavior this replaces. Once
real encoder coordinates replace commanded ones, a reading's actual
physical position may not land exactly on a grid line any more
(motor slippage/backlash is the whole reason this rework matters) --
nearest-cell snap handles that gracefully, and MULTIPLE readings
snapping into the same cell is exactly the "several readings fall
into the same map cell" case the running average and read_count below
exist for.
"""
from __future__ import annotations

# --- repo-root import bootstrap -------------------------------------------
# Same pattern as scan_manager.py / adaptive_scan.py -- lets this module be
# imported whether the process was started as `python run_gui.py`, a direct
# script invocation of this file, or `python -m scan.live_map_accumulator`.
import os as _os
import sys as _sys

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)
# ---------------------------------------------------------------------------

import numpy as np

from scan.scan_params import grid_dims_from_range


def _is_valid(value) -> bool:
    """True if value is a real, usable number -- not None and not NaN."""
    if value is None:
        return False
    try:
        return not np.isnan(value)
    except TypeError:
        return False


class MapAccumulator:
    """
    Fixed-grid accumulator: construct once per scan (or step-size
    change) with the same (x_range_mm, y_range_mm, step_size_mm) the
    scan itself is using, then call add_reading() once per point.

    value_grid holds the running mean per cell (NaN = never visited, or
    every reading that landed there was itself invalid). read_count
    holds the total number of readings that landed in each cell,
    valid or not -- separate from the mean's own denominator so an
    unexpectedly high count (e.g. a cell hit far more often than the
    configured pass count) is visible even when every one of those
    reads happened to be a valid temperature. That's the "track number
    of read points per xy position, to monitor for surprises" figure
    Roy asked for.
    """

    def __init__(self, x_range_mm, y_range_mm, step_size_mm):
        self.reset(x_range_mm, y_range_mm, step_size_mm)

    def reset(self, x_range_mm, y_range_mm, step_size_mm):
        """
        Rebuild the backing arrays from scratch -- call at the start of
        every scan (and whenever step_size_mm changes, since that
        changes nx/ny), exactly like the panel this backs already did
        for its own array before this class existed.
        """
        x0, x1 = x_range_mm
        y0, y1 = y_range_mm
        self.x_range_mm = (float(x0), float(x1))
        self.y_range_mm = (float(y0), float(y1))
        self.step_size_mm = step_size_mm

        self.nx, self.ny = grid_dims_from_range((x0, x1), (y0, y1), step_size_mm)
        self.xs = np.linspace(x0, x1, self.nx)
        self.ys = np.linspace(y0, y1, self.ny)

        self.value_grid = np.full((self.ny, self.nx), np.nan)
        # Denominator behind value_grid's running mean -- only readings
        # that were actually finite increment this (see _is_valid).
        self._valid_count = np.zeros((self.ny, self.nx), dtype=int)
        # EVERY reading that landed in a cell, valid or not.
        self.read_count = np.zeros((self.ny, self.nx), dtype=int)

    def nearest_cell(self, x_mm, y_mm):
        """
        Snap (x_mm, y_mm) to the nearest (ix, iy) grid cell. Returns
        None if either coordinate is missing (e.g. a caller forgot to
        filter out a reference-point revisit before calling
        add_reading -- see gui/live_map.py's is_reference check).
        """
        if x_mm is None or y_mm is None:
            return None
        ix = int(np.argmin(np.abs(self.xs - x_mm)))
        iy = int(np.argmin(np.abs(self.ys - y_mm)))
        return ix, iy

    def add_reading(self, x_mm, y_mm, value):
        """
        Bin one (x_mm, y_mm, value) reading into its nearest cell,
        updating that cell's running mean and read count in place.

        value may be None or NaN (a failed IR/motion read, e.g.
        ir_result["value"] on a retry-exhausted read -- see
        ScanManager._read_ir_with_retry): read_count still increments
        (a read WAS attempted at this position), but the running mean
        skips it, the same way a NaN reading already doesn't corrupt
        any other statistic in this codebase.

        Returns the (ix, iy) cell the reading landed in, or None if
        x_mm/y_mm was missing.
        """
        cell = self.nearest_cell(x_mm, y_mm)
        if cell is None:
            return None
        ix, iy = cell
        self.read_count[iy, ix] += 1

        if _is_valid(value):
            n = self._valid_count[iy, ix] + 1
            self._valid_count[iy, ix] = n
            prev = self.value_grid[iy, ix]
            prev = 0.0 if np.isnan(prev) else prev
            # Incremental mean (Welford, first-moment only -- no need
            # for variance here): new_mean = prev + (x - prev) / n.
            self.value_grid[iy, ix] = prev + (value - prev) / n

        return ix, iy


class LineAccumulator:
    """
    PR2b (2026-09-08): line-mode counterpart to MapAccumulator above.

    MapAccumulator needs a rectangular (x_range_mm, y_range_mm,
    step_size_mm) to build its 2D binning grid from -- none of those
    exist for an arbitrary-angle line (see scan_manager.generate_line_points()
    and oes_store.py's line-mode schema for why: a line's only real
    spatial axis is s_mm, arc length from its start point, not two
    independent x/y ranges). This exists instead of trying to force a
    line scan through MapAccumulator, which corrects an earlier looser
    claim (in the project gap-analysis doc) that gui/live_map.py
    "likely needs no change" for line mode -- it does, via this class.

    Same running-mean-plus-read-count shape as MapAccumulator, just
    over a 1D array indexed by nearest s_mm position instead of a 2D
    nearest-cell snap -- gui/line_scan_panel.py's live view reads
    value_grid/read_count the same way gui/live_map.py already reads
    MapAccumulator's, just plotted as a line/scatter (x=s_mm) instead
    of an imshow() heatmap.
    """

    def __init__(self, start_mm, end_mm, n_points):
        self.reset(start_mm, end_mm, n_points)

    def reset(self, start_mm, end_mm, n_points):
        """
        Rebuild the backing arrays from scratch -- call at the start of
        every line scan, mirroring MapAccumulator.reset().
        """
        x0, y0 = start_mm
        x1, y1 = end_mm
        self.start_mm = (float(x0), float(y0))
        self.end_mm = (float(x1), float(y1))
        self.n_points = int(n_points)

        length_mm = float(np.hypot(x1 - x0, y1 - y0))
        self.s_values = np.linspace(0.0, length_mm, self.n_points) if self.n_points > 1 \
            else np.array([0.0])

        self.value_grid = np.full(self.n_points, np.nan)
        self._valid_count = np.zeros(self.n_points, dtype=int)
        self.read_count = np.zeros(self.n_points, dtype=int)

    def nearest_index(self, s_mm):
        """
        Snap s_mm to the nearest index along the line. Returns None if
        s_mm is missing (e.g. a reference-point revisit, which carries
        NaN s_mm in line mode -- see scan_manager.py's reference-point
        handling -- or, for a non-line record fed here by mistake, a
        genuinely missing key).
        """
        if s_mm is None:
            return None
        try:
            if np.isnan(s_mm):
                return None
        except TypeError:
            return None
        return int(np.argmin(np.abs(self.s_values - s_mm)))

    def add_reading(self, s_mm, value):
        """
        Bin one (s_mm, value) reading into its nearest index, updating
        that index's running mean and read count in place. Same NaN/
        None-tolerant semantics as MapAccumulator.add_reading() -- see
        that method's docstring.

        Returns the index the reading landed in, or None if s_mm was
        missing/NaN.
        """
        i = self.nearest_index(s_mm)
        if i is None:
            return None
        self.read_count[i] += 1

        if _is_valid(value):
            n = self._valid_count[i] + 1
            self._valid_count[i] = n
            prev = self.value_grid[i]
            prev = 0.0 if np.isnan(prev) else prev
            self.value_grid[i] = prev + (value - prev) / n

        return i


if __name__ == "__main__":
    # Smoke test -- no tkinter/hardware involved, just the accumulation math.
    acc = MapAccumulator(x_range_mm=[0, 4], y_range_mm=[0, 4], step_size_mm=2.0)
    assert acc.value_grid.shape == (3, 3)
    assert acc.read_count.shape == (3, 3)
    assert np.all(np.isnan(acc.value_grid))
    assert np.all(acc.read_count == 0)

    # Single reading exactly on a grid point.
    ix, iy = acc.add_reading(0.0, 0.0, 900.0)
    assert (ix, iy) == (0, 0)
    assert acc.value_grid[0, 0] == 900.0
    assert acc.read_count[0, 0] == 1

    # A second reading in the SAME cell -- running average, count increments.
    acc.add_reading(0.0, 0.0, 902.0)
    assert acc.value_grid[0, 0] == 901.0, acc.value_grid[0, 0]
    assert acc.read_count[0, 0] == 2

    # A NaN reading -- read_count still increments, mean unaffected.
    acc.add_reading(0.0, 0.0, float("nan"))
    assert acc.value_grid[0, 0] == 901.0
    assert acc.read_count[0, 0] == 3

    # A reading that lands off-grid-line (simulated encoder position, not
    # exactly on a commanded grid point) snaps to the nearest cell.
    ix2, iy2 = acc.add_reading(0.3, 1.9, 850.0)
    assert (ix2, iy2) == (0, 1), (ix2, iy2)  # nearest to x=0, y=2
    assert acc.value_grid[1, 0] == 850.0

    # A reference-point revisit (no x/y) is a no-op, not an error.
    assert acc.add_reading(None, None, 999.0) is None
    assert acc.read_count[0, 0] == 3  # unchanged

    # An unvisited cell stays blank (NaN), per spec.
    assert np.isnan(acc.value_grid[2, 2])
    assert acc.read_count[2, 2] == 0

    print("live_map_accumulator smoke test OK")

    # ------------------------------------------------------------------
    # PR2b (2026-09-08): LineAccumulator
    # ------------------------------------------------------------------
    # A 3-4-5 triangle again (see scan_manager.py's own generate_line_points()
    # regression, scan_params.py __main__) so s_values land on clean numbers.
    line_acc = LineAccumulator(start_mm=(0.0, 0.0), end_mm=(3.0, 4.0), n_points=6)
    assert line_acc.value_grid.shape == (6,)
    assert line_acc.read_count.shape == (6,)
    assert np.all(np.isnan(line_acc.value_grid))
    assert abs(float(line_acc.s_values[-1]) - 5.0) < 1e-9  # length of a 3-4-5 triangle

    # A reading exactly at s_mm=0 (line start).
    i0 = line_acc.add_reading(0.0, 900.0)
    assert i0 == 0
    assert line_acc.value_grid[0] == 900.0
    assert line_acc.read_count[0] == 1

    # A second reading at the SAME s_mm -- running average, count increments.
    line_acc.add_reading(0.0, 902.0)
    assert line_acc.value_grid[0] == 901.0
    assert line_acc.read_count[0] == 2

    # A NaN reading -- read_count still increments, mean unaffected.
    line_acc.add_reading(0.0, float("nan"))
    assert line_acc.value_grid[0] == 901.0
    assert line_acc.read_count[0] == 3

    # A reading that lands off-grid-point snaps to the nearest s_mm index.
    i_near = line_acc.add_reading(0.9, 850.0)
    assert i_near == 1, i_near  # nearest to s_mm=1.0 (step is 1.0mm for n=6 over length 5)
    assert line_acc.value_grid[1] == 850.0

    # A reference-point revisit (NaN s_mm, per scan_manager.py's line-mode
    # handling -- see that module's comment) is a no-op, not an error.
    assert line_acc.add_reading(float("nan"), 999.0) is None
    assert line_acc.add_reading(None, 999.0) is None
    assert line_acc.read_count[0] == 3  # unchanged

    # An unvisited index stays blank (NaN), per spec.
    assert np.isnan(line_acc.value_grid[5])
    assert line_acc.read_count[5] == 0

    print("live_map_accumulator LineAccumulator smoke test OK")
