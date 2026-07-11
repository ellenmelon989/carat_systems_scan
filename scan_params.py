"""
scan_params.py

Shared scan-parameter definitions and validation, used by both
scan_manager.py (runtime scan execution) and calibrate_scan_area.py
(interactive edge calibration + parameter entry).

Operator-facing scan parameters are dwell time (per-point IR averaging
duration) and step size (grid spacing in mm). Grid point counts (nx, ny)
are DERIVED from step size and the edge-calibrated scan range — they are
not set directly in config.yaml. See grid_dims_from_range() below and
scan.grid in config.yaml.
"""

from __future__ import annotations

DWELL_TIME_DEFAULT_S = 8.0
DWELL_TIME_MIN_S = 2.0
DWELL_TIME_MAX_S = 22.0

STEP_SIZE_DEFAULT_MM = 2.0
STEP_SIZE_MIN_MM = 1.0
STEP_SIZE_MAX_MM = 11.0


def validate_dwell_time_s(dwell_time_s: float) -> float:
    """Raise ValueError if dwell_time_s is outside the operator-valid range."""
    dwell_time_s = float(dwell_time_s)
    if not (DWELL_TIME_MIN_S <= dwell_time_s <= DWELL_TIME_MAX_S):
        raise ValueError(
            f"dwell_time_s={dwell_time_s} outside valid range "
            f"[{DWELL_TIME_MIN_S}, {DWELL_TIME_MAX_S}] seconds"
        )
    return dwell_time_s


def validate_step_size_mm(step_size_mm: float) -> float:
    """Raise ValueError if step_size_mm is outside the operator-valid range."""
    step_size_mm = float(step_size_mm)
    if not (STEP_SIZE_MIN_MM <= step_size_mm <= STEP_SIZE_MAX_MM):
        raise ValueError(
            f"step_size_mm={step_size_mm} outside valid range "
            f"[{STEP_SIZE_MIN_MM}, {STEP_SIZE_MAX_MM}] mm"
        )
    return step_size_mm


def grid_dims_from_range(x_range_mm, y_range_mm, step_size_mm: float):
    """
    Derive (nx, ny) grid point counts from an edge-calibrated scan range
    and a step size in mm.

    A zero-width range on one axis (e.g. x0 == x1) collapses that axis to
    a single point — this is how a 1D line scan falls naturally out of the
    2D grid machinery; "if X changes and Y doesn't, the map ends up a
    line" per spec, with no special-casing needed elsewhere in the code.
    """
    step_size_mm = validate_step_size_mm(step_size_mm)

    def _dim(lo, hi):
        span = abs(hi - lo)
        if span == 0:
            return 1
        return int(round(span / step_size_mm)) + 1

    x0, x1 = x_range_mm
    y0, y1 = y_range_mm
    nx = _dim(x0, x1)
    ny = _dim(y0, y1)
    return nx, ny


def estimate_scan_time_s(nx: int, ny: int, dwell_time_s: float, settle_time_s: float = 0.0) -> float:
    """Rough total scan time estimate: point count * (dwell + settle)."""
    return nx * ny * (dwell_time_s + settle_time_s)


if __name__ == "__main__":
    # Smoke test
    assert grid_dims_from_range([0, 50], [0, 50], 2.0) == (26, 26)
    assert grid_dims_from_range([0, 0], [0, 40], 2.0) == (1, 21)  # degenerate X -> line scan
    assert grid_dims_from_range([10, 10], [5, 5], 2.0) == (1, 1)  # degenerate both -> point scan

    try:
        validate_step_size_mm(15)
        raise AssertionError("expected ValueError for step_size_mm out of range")
    except ValueError:
        pass

    try:
        validate_dwell_time_s(1)
        raise AssertionError("expected ValueError for dwell_time_s out of range")
    except ValueError:
        pass

    est = estimate_scan_time_s(*grid_dims_from_range([0, 50], [0, 50], 2.0), dwell_time_s=8.0)
    print(f"26x26 grid @ 8s dwell -> {est/60:.1f} min estimated scan time")
    print("scan_params smoke test OK")
