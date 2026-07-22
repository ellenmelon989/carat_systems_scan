"""
adaptive_scan_logger.py

Raw per-reading logging, row-level QC flags, and post-scan coarse-grid
aggregation for the adaptive raster scan — see docs/adaptive_scan_spec.md
§8, §10, §11, §12.
"""

from __future__ import annotations

# --- repo-root import bootstrap -------------------------------------------
# Needed because this file imports its own sibling (adaptive_scan_signal)
# via the absolute `adaptive_scan.X` form below, which requires the repo
# ROOT (parent of adaptive_scan/) on sys.path -- not just this file's own
# directory, which is all a direct `python adaptive_scan/adaptive_scan_logger.py`
# invocation puts there by default. See scan/scan_manager.py for the full
# rationale (same pattern, applied throughout both scan/ and adaptive_scan/).
import os as _os
import sys as _sys

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)
# ---------------------------------------------------------------------------

import csv
import os
from dataclasses import dataclass
from typing import Optional

import numpy as np

from adaptive_scan.adaptive_scan_signal import RawSignals


@dataclass
class RowSummary:
    """
    One row's bookkeeping, used only for cross-row QC (§11) — not written
    to the raw CSV itself (that's per-reading; see AdaptiveScanRawLogger).
    """
    row_number: int
    scan_direction: str
    n_readings: int
    had_internal_loss: bool
    total_commanded_dx_mm: float  # sum of |dx| jogged while working this row
                                    # (ignore-phase + collection phase +
                                    # exit-confirm phase) — the "how much
                                    # motion did it take to get this many
                                    # confirmed readings" figure flag #4 uses.
    start_time: float
    end_time: float


class AdaptiveScanRawLogger:
    """
    Crash-safe-per-ROW CSV writer for raw readings.

    Buffers one row's accepted (between-confirmed-boundaries) readings in
    memory while that row is being scanned, because normalized_x
    (j / (N-1), spec §8) can't be computed until the row's total valid
    count N is known — which only happens once the row ends. The whole row
    is then appended to disk in one write. Worst case on a mid-row crash:
    lose that one row's readings (at most ~100 readings, seconds to low
    minutes of work) — every previously completed row is already on disk,
    same crash-safety spirit as data_logger.py's per-point writes, just at
    row granularity instead of point granularity, because this mode's unit
    of "safely committed" data is a row, not a single reading (a lone
    reading has no normalized_x until its row finishes).

    normalized_y is deliberately NOT a column here — it depends on the
    TOTAL row count, which isn't known until the whole scan ends (§3 step
    13), so it structurally can't be written incrementally. row_number
    (which is known and immutable at write time) is written instead;
    build_coarse_grid() derives normalized_y from it after the scan
    completes. This also matches Roy's original field list (§10), which
    asks for "calculated normalized position" — i.e. normalized_x, the one
    formula actually specified — plus row_number, not a second normalized
    axis.
    """

    _FIELDNAMES = [
        "reading_id", "row_number", "reading_index_in_row", "scan_direction",
        "timestamp", "selected_signal_name", "selected_value",
        "ir_temp_c", "ir_emissivity", "ir_dilution",
        "motor_dx_mm", "motor_dy_mm",
        "normalized_x",
    ]

    def __init__(self, path: str, oes_feature_names: Optional[list] = None):
        self.path = path
        self.oes_feature_names = list(oes_feature_names or [])
        self._fieldnames = self._FIELDNAMES + [f"oes_{n}" for n in self.oes_feature_names]
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._header_written = os.path.exists(path)
        self._next_reading_id = 0
        self._row_buffer: list[dict] = []

    def start_row(self):
        """Call once at the beginning of each new row (spec §3 step 6)."""
        self._row_buffer = []

    def add_reading(self, row_number: int, scan_direction: str, raw: RawSignals,
                     selected_signal_name: str, selected_value: float,
                     motor_dx_mm: float, motor_dy_mm: float) -> int:
        """
        Buffer one accepted (between-confirmed-boundaries) reading —
        readings during the "ignore until entry confirmed" phase (§3 step
        7) or after a confirmed exit are NOT passed here at all, per
        Roy's "retain only the readings between the confirmed first and
        second wafer boundaries" (§3 preamble to step-by-step retention).

        Returns the assigned reading_id (monotonic across the whole scan,
        not reset per row — mirrors scan_manager's point_id convention).
        """
        reading_id = self._next_reading_id
        self._next_reading_id += 1
        record = {
            "reading_id": reading_id,
            "row_number": row_number,
            "reading_index_in_row": len(self._row_buffer),
            "scan_direction": scan_direction,
            "timestamp": raw.timestamp,
            "selected_signal_name": selected_signal_name,
            "selected_value": selected_value,
            "ir_temp_c": raw.ir_temp_c,
            "ir_emissivity": raw.ir_emissivity,
            "ir_dilution": raw.ir_dilution if raw.ir_dilution is not None else float("nan"),
            "motor_dx_mm": motor_dx_mm,
            "motor_dy_mm": motor_dy_mm,
        }
        for name in self.oes_feature_names:
            record[f"oes_{name}"] = raw.oes_feature_values.get(name, float("nan"))
        self._row_buffer.append(record)
        return reading_id

    def finish_row(self) -> list[dict]:
        """
        Assign normalized_x to every buffered reading (spec §8:
        j / (N-1); 0.0 for a degenerate single-reading row rather than a
        divide-by-zero — not expected in practice, but must not crash),
        write the whole row to disk in one append, and return the
        finished records (each a plain dict, ready for RowSummary
        construction and coarse-grid binning by the caller).
        """
        n = len(self._row_buffer)
        for j, record in enumerate(self._row_buffer):
            record["normalized_x"] = (j / (n - 1)) if n > 1 else 0.0

        write_header = not self._header_written
        with open(self.path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self._fieldnames)
            if write_header:
                writer.writeheader()
            for record in self._row_buffer:
                writer.writerow(record)
        self._header_written = True

        finished = self._row_buffer
        self._row_buffer = []
        return finished


def compute_row_qc_flags(rows: list[RowSummary]) -> dict:
    """
    Per-row anomaly flags (spec §11), computed over ALL rows of one scan
    at once — flags 2-4 are inherently relative ("unusually low," "differs
    from neighbors") and meaningless for a single row in isolation.

    Returns {row_number: [flag_name, ...]} — a row with no anomalies maps
    to an empty list, not an absent key, so callers can always index by
    row_number without a membership check.

    These thresholds (0.5x median, 2x median ratio, etc.) are heuristics,
    not hardware-validated constants — expect to retune them against a
    handful of real on-site scans (spec §14 step 7) once what "normal"
    looks like on the real mount is actually known. Flagging is meant to
    surface rows for a human to judge, not to silently discard data.
    """
    if not rows:
        return {}

    flags: dict = {r.row_number: [] for r in rows}
    counts = np.array([r.n_readings for r in rows], dtype=float)
    median_count = float(np.median(counts))

    for r in rows:
        # 1. Internal loss of wafer signal mid-row.
        if r.had_internal_loss:
            flags[r.row_number].append("internal_signal_loss")

        # 2. Unusually low reading count vs. the scan as a whole.
        if median_count > 0 and r.n_readings < 0.5 * median_count:
            flags[r.row_number].append("low_reading_count")

    # 3. Differs substantially from its immediate NEIGHBORS specifically
    # (not just the global median) — a row can match the scan's overall
    # median and still be a real outlier if both neighbors are far higher
    # or lower.
    by_row_number = {r.row_number: r for r in rows}
    sorted_numbers = sorted(by_row_number)
    for i, row_number in enumerate(sorted_numbers):
        neighbor_numbers = [
            n for n in (
                sorted_numbers[i - 1] if i > 0 else None,
                sorted_numbers[i + 1] if i < len(sorted_numbers) - 1 else None,
            ) if n is not None
        ]
        if not neighbor_numbers:
            continue
        neighbor_avg = sum(by_row_number[n].n_readings for n in neighbor_numbers) / len(neighbor_numbers)
        if neighbor_avg > 0 and abs(by_row_number[row_number].n_readings - neighbor_avg) > 0.5 * neighbor_avg:
            flags[row_number].append("reading_count_outlier_vs_neighbors")

    # 4. Stall / large motor slip: commanded distance per confirmed
    # reading far above the scan's typical ratio means this row needed
    # much more jogging to produce the same number of confirmed readings —
    # consistent with slipping/stalling and re-covering real distance.
    ratios = np.array([
        (r.total_commanded_dx_mm / r.n_readings) if r.n_readings > 0 else np.nan
        for r in rows
    ])
    valid_ratios = ratios[~np.isnan(ratios)]
    if len(valid_ratios) >= 2:
        median_ratio = float(np.median(valid_ratios))
        for r, ratio in zip(rows, ratios):
            if not np.isnan(ratio) and median_ratio > 0 and ratio > 2.0 * median_ratio:
                flags[r.row_number].append("possible_stall_or_slip")

    return flags


def build_coarse_grid(readings: list, n_cells: int) -> dict:
    """
    Bin every raw reading into an n_side x n_side coarse grid
    (n_side = round(sqrt(n_cells)), so a 100-cell request gives an exact
    10x10 grid — spec §4 parameter 7, §10, §12).

    Both axes are NORMALIZED INDEX positions, not mm:
      - x: reading["normalized_x"], already computed per row by
        AdaptiveScanRawLogger.finish_row() — j / (N_row - 1), exactly the
        formula Roy specified (spec §8).
      - y: row_index / (R - 1), where row_index is this row's rank among
        all rows actually scanned and R is the total row count. This
        applies the SAME treatment to the row axis that Roy specified for
        readings within a row, for the same underlying reason: total row
        count isn't known until the scan naturally terminates (§3 step
        13), just as a row's own N isn't known until that row ends. This
        extension beyond the original spec (which only defines the
        within-row x formula) is this implementation's call — flagged for
        Roy to confirm. The alternative (row_number * y_raster_spacing_mm,
        trusting the commanded raster increment as real mm) is still
        fully recoverable from the raw CSV's row_number + the configured
        y_raster_spacing_mm without re-scanning, if that's preferred
        instead for the displayed map.

    Returns a dict with n_side and three (n_side, n_side) numpy arrays:
    mean, count, stddev (NaN where no reading landed in a cell — an empty
    cell, not zero). stddev needs >=2 readings in a cell to be defined
    (NaN otherwise, same "not enough information" convention as mean).
    """
    if not readings:
        raise ValueError("No readings to grid.")

    n_side = max(1, round(n_cells ** 0.5))

    row_numbers = sorted({r["row_number"] for r in readings})
    row_rank = {rn: i for i, rn in enumerate(row_numbers)}
    n_rows = len(row_numbers)

    xs = np.array([r["normalized_x"] for r in readings], dtype=float)
    ys = np.array(
        [(row_rank[r["row_number"]] / (n_rows - 1)) if n_rows > 1 else 0.0 for r in readings],
        dtype=float,
    )
    values = np.array([r["selected_value"] for r in readings], dtype=float)

    # Clip the top edge (x or y == 1.0) into the last bin rather than
    # letting it fall into an (n_side)-th index out of range.
    ix = np.clip((xs * n_side).astype(int), 0, n_side - 1)
    iy = np.clip((ys * n_side).astype(int), 0, n_side - 1)

    mean_grid = np.full((n_side, n_side), np.nan)
    count_grid = np.zeros((n_side, n_side), dtype=int)
    stddev_grid = np.full((n_side, n_side), np.nan)

    for cx in range(n_side):
        col_x = (ix == cx)
        if not np.any(col_x):
            continue
        for cy in range(n_side):
            mask = col_x & (iy == cy)
            n = int(np.sum(mask))
            count_grid[cx, cy] = n
            if n == 0:
                continue
            cell_values = values[mask]
            valid = cell_values[~np.isnan(cell_values)]
            if len(valid) > 0:
                mean_grid[cx, cy] = float(np.mean(valid))
            if len(valid) > 1:
                stddev_grid[cx, cy] = float(np.std(valid, ddof=1))

    return {"n_side": n_side, "mean": mean_grid, "count": count_grid, "stddev": stddev_grid}


if __name__ == "__main__":
    import tempfile

    raw_template = dict(ir_emissivity=0.85, ir_dilution=1.0, ir_error=False, oes_feature_values={})

    with tempfile.TemporaryDirectory() as tmpdir:
        path = os.path.join(tmpdir, "raw_readings.csv")
        logger = AdaptiveScanRawLogger(path, oes_feature_names=["C2_Swan"])

        all_rows = []
        all_readings = []

        # Row 0: 10 clean readings, ramping temperature (simulating a real
        # wafer's gradual profile), direction +x.
        logger.start_row()
        for j in range(10):
            raw = RawSignals(timestamp=1000.0 + j, ir_temp_c=900.0 + j, **raw_template)
            logger.add_reading(0, "+x", raw, "ir_temp_c", raw.ir_temp_c, motor_dx_mm=1.0, motor_dy_mm=0.0)
        finished = logger.finish_row()
        assert len(finished) == 10
        assert finished[0]["normalized_x"] == 0.0 and finished[-1]["normalized_x"] == 1.0
        all_readings.extend(finished)
        all_rows.append(RowSummary(0, "+x", 10, had_internal_loss=False,
                                    total_commanded_dx_mm=10.0, start_time=1000.0, end_time=1010.0))

        # Row 1: 9 readings, direction -x, mirrors row 0 roughly.
        logger.start_row()
        for j in range(9):
            raw = RawSignals(timestamp=2000.0 + j, ir_temp_c=905.0 + j, **raw_template)
            logger.add_reading(1, "-x", raw, "ir_temp_c", raw.ir_temp_c, motor_dx_mm=1.0, motor_dy_mm=0.0)
        finished = logger.finish_row()
        all_readings.extend(finished)
        all_rows.append(RowSummary(1, "-x", 9, had_internal_loss=False,
                                    total_commanded_dx_mm=9.0, start_time=2000.0, end_time=2009.0))

        # Row 2: a deliberately anomalous row — only 2 readings (vs.
        # neighbors' ~10) and a huge commanded distance for those 2
        # readings (simulated stall/slip).
        logger.start_row()
        for j in range(2):
            raw = RawSignals(timestamp=3000.0 + j, ir_temp_c=910.0 + j, **raw_template)
            logger.add_reading(2, "+x", raw, "ir_temp_c", raw.ir_temp_c, motor_dx_mm=1.0, motor_dy_mm=0.0)
        finished = logger.finish_row()
        all_readings.extend(finished)
        all_rows.append(RowSummary(2, "+x", 2, had_internal_loss=True,
                                    total_commanded_dx_mm=80.0, start_time=3000.0, end_time=3002.0))

        # CSV round-trip check.
        with open(path) as f:
            lines = f.readlines()
        assert len(lines) == 1 + 10 + 9 + 2, f"expected header + 21 rows, got {len(lines)}"
        print(f"CSV written OK ({len(lines) - 1} reading rows + header)")

        flags = compute_row_qc_flags(all_rows)
        print("QC flags:", flags)
        assert flags[2] and "internal_signal_loss" in flags[2]
        assert "low_reading_count" in flags[2]
        assert "possible_stall_or_slip" in flags[2]
        assert flags[0] == [] or "internal_signal_loss" not in flags[0]
        print("Row QC flagging OK")

        grid = build_coarse_grid(all_readings, n_cells=9)  # 3x3 for an easy check
        assert grid["n_side"] == 3
        assert grid["mean"].shape == (3, 3)
        assert grid["count"].sum() == len(all_readings)
        print(f"Coarse grid OK: n_side={grid['n_side']}, "
              f"total binned={int(grid['count'].sum())}, "
              f"mean range={np.nanmin(grid['mean']):.1f}-{np.nanmax(grid['mean']):.1f}")

    print("adaptive_scan_logger smoke test OK")
