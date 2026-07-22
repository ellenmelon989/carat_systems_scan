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

# Number of full grid passes per scan. 1 = old behavior (single pass; only
# the fixed reference_point gets revisited). >1 repeats the ENTIRE grid
# that many times so every point gets rechecked over the course of the
# scan — needed for per-XY-point drift/oscillation tracking (T/e over
# time); a single fixed reference point can't give you that. Upper bound
# is a sanity cap, not a hardware limit — raise it if a real use case
# needs more.
PASSES_DEFAULT = 1
PASSES_MIN = 1
PASSES_MAX = 50


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

    Rounds span/step_size to 6 decimal places before the integer round().
    Without this, ordinary floating-point noise carried through
    calibrate_scan_area.py's edge/steps-per-mm math (e.g. a span landing
    at 74.99999999999999 instead of an intended 75.0) can fall on the
    opposite side of round()'s tie-break boundary than the same nominal
    value does after being serialized to config.yaml at 4 decimal places
    and reloaded by scan_manager.py in a fresh process — silently
    disagreeing on nx/ny between the calibration preview and the grid the
    scan actually runs (seen 2026-07-17: preview said 8x8=64, scan ran
    9x9=81). 6 decimal places (1e-6 mm) is far finer than any real
    steps_per_mm calibration can resolve (~1e-4 mm at best), so this only
    absorbs float noise — it never masks a real, intended distinction.
    """
    step_size_mm = validate_step_size_mm(step_size_mm)

    def _dim(lo, hi):
        span = abs(hi - lo)
        if span == 0:
            return 1
        ratio = round(span / step_size_mm, 6)
        return int(round(ratio)) + 1

    x0, x1 = x_range_mm
    y0, y1 = y_range_mm
    nx = _dim(x0, x1)
    ny = _dim(y0, y1)
    return nx, ny


def in_radius(x_mm: float, y_mm: float, center_mm, radius_mm: float) -> bool:
    """
    True if (x_mm, y_mm) is within radius_mm of center_mm.

    Shared by scan_manager.generate_grid() (which grid points actually
    get measured) and calibrate_scan_area.py's preview (how many points
    that will be) so the two can't independently disagree the way the
    rectangular-grid preview/execution counts once did (see
    grid_dims_from_range()'s docstring and the 2026-07-17 diagnosis).

    Exists because a rectangular scan range (x_range_mm x y_range_mm) is
    a bounding box around a circular wafer, not the wafer itself — its
    corners are, by construction, off-sample. Masking the grid down to a
    circle means those corner points are never measured at all: no
    wasted/meaningless off-wafer data, and no motion command is ever
    issued to the largest-excursion points in the grid, which is also
    where the mount's own separate mechanical travel limit (independent
    of wafer shape) is most likely to be exceeded.
    """
    cx, cy = center_mm
    return (x_mm - cx) ** 2 + (y_mm - cy) ** 2 <= radius_mm ** 2


def find_out_of_limits(points_xy, limits):
    """
    Return a list of (label, x_mm, y_mm, reason) for every (label, x_mm,
    y_mm) triple in points_xy that falls outside the soft limits dict
    (x_min_mm/x_max_mm/y_min_mm/y_max_mm — same shape as
    motion.soft_limits in config.yaml and the same keys
    MotionController.check_limits() checks).

    Does NOT raise and does NOT stop at the first violation — checks
    every point against both axes independently and returns the full
    list. This is what makes a pre-flight scan check useful for
    debugging: MotionController.check_limits() (used per-point, during
    the scan) raises on the FIRST bad axis of the FIRST bad point, which
    is enough to stop a scan safely but not enough to see "is this one
    bad corner, or is the whole grid offset" at a glance before
    anything moves.
    """
    x_min, x_max = limits["x_min_mm"], limits["x_max_mm"]
    y_min, y_max = limits["y_min_mm"], limits["y_max_mm"]
    violations = []
    for label, x, y in points_xy:
        reasons = []
        if not (x_min <= x <= x_max):
            reasons.append(f"x={x:.4f} outside [{x_min}, {x_max}]")
        if not (y_min <= y <= y_max):
            reasons.append(f"y={y:.4f} outside [{y_min}, {y_max}]")
        if reasons:
            violations.append((label, x, y, "; ".join(reasons)))
    return violations


def validate_points_within_limits(points_xy, limits, max_shown: int = 20):
    """
    Raise ValueError if any (label, x_mm, y_mm) in points_xy falls
    outside limits, listing every violation (up to max_shown) with its
    label, position, and which bound it broke.

    Intended to run BEFORE any motion controller is connected or homed —
    see scan_manager.preflight_check(). This checks the same
    x_min_mm/x_max_mm/y_min_mm/y_max_mm bounds as
    MotionController.check_limits(), just over the WHOLE commanded
    position list at once and before anything moves, rather than one
    point at a time as each move is issued mid-scan. It does not replace
    the per-point check_limits() call in scan_manager._measure_point —
    that stays in place as defense in depth (e.g. if soft_limits itself
    were ever edited mid-process, or this function's caller forgot to
    include some position this one doesn't know about).
    """
    violations = find_out_of_limits(points_xy, limits)
    if not violations:
        return
    lines = [f"  {label}: ({x:.4f}, {y:.4f}) mm — {reason}"
             for label, x, y, reason in violations[:max_shown]]
    if len(violations) > max_shown:
        lines.append(f"  ...and {len(violations) - max_shown} more")
    raise ValueError(
        f"{len(violations)} of {len(points_xy)} commanded position(s) fall outside "
        f"motion.soft_limits (x: [{limits['x_min_mm']}, {limits['x_max_mm']}], "
        f"y: [{limits['y_min_mm']}, {limits['y_max_mm']}]) — refusing to start. "
        "No motion has been commanded.\n" + "\n".join(lines)
    )


def validate_passes(passes: int) -> int:
    """Raise ValueError if passes is outside the operator-valid range."""
    passes = int(passes)
    if not (PASSES_MIN <= passes <= PASSES_MAX):
        raise ValueError(
            f"passes={passes} outside valid range [{PASSES_MIN}, {PASSES_MAX}]"
        )
    return passes


def estimate_scan_time_s(nx: int, ny: int, dwell_time_s: float, settle_time_s: float = 0.0,
                          passes: int = 1) -> float:
    """
    Rough total scan time estimate: point count * (dwell + settle) * passes.

    Does NOT include reference-point revisits or periodic rehomes (both
    optional and config-dependent) — this is the grid-only floor.
    """
    return nx * ny * passes * (dwell_time_s + settle_time_s)


if __name__ == "__main__":
    # Smoke test
    assert grid_dims_from_range([0, 50], [0, 50], 2.0) == (26, 26)
    assert grid_dims_from_range([0, 0], [0, 40], 2.0) == (1, 21)  # degenerate X -> line scan
    assert grid_dims_from_range([10, 10], [5, 5], 2.0) == (1, 1)  # degenerate both -> point scan

    # Regression for 2026-07-17: span/step landing exactly on round()'s
    # .5 tie-break boundary (75mm / 10mm = 7.5) must resolve the SAME way
    # whether fed the noisy live float (74.99999999999999, as computed
    # in-process by calibrate_scan_area.py before writing to disk) or the
    # clean value it round-trips to after being written at 4 decimal
    # places and reloaded by scan_manager.py — otherwise the calibration
    # preview and the executed scan silently disagree on grid size.
    assert grid_dims_from_range([-50.84745762711864, 24.152542372881353],
                                 [-57.77166437414029, 17.228335625859696], 10.0) == (9, 9)
    assert grid_dims_from_range([-50.8475, 24.1525], [-57.7717, 17.2283], 10.0) == (9, 9)

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

    try:
        validate_passes(0)
        raise AssertionError("expected ValueError for passes out of range")
    except ValueError:
        pass
    assert validate_passes(3) == 3

    # in_radius(): center included, a bounding-box corner excluded, a
    # cardinal edge point included (on the circle, not past it).
    assert in_radius(0, 0, (0, 0), 25.0) is True
    assert in_radius(25, 25, (0, 0), 25.0) is False   # bounding-box corner, off-wafer
    assert in_radius(25, 0, (0, 0), 25.0) is True     # exactly on the circle
    assert in_radius(0, -25, (5, -5), 25.0) is True   # off-origin center

    # find_out_of_limits() / validate_points_within_limits(): all-in,
    # one-out-of-two-axes, and the "don't stop at the first violation"
    # behavior that per-point check_limits() can't give you.
    limits = {"x_min_mm": -10, "x_max_mm": 10, "y_min_mm": -5, "y_max_mm": 5}
    all_ok = [("a", 0, 0), ("b", 10, 5), ("c", -10, -5)]
    assert find_out_of_limits(all_ok, limits) == []
    validate_points_within_limits(all_ok, limits)  # must not raise

    mixed = [("a", 0, 0), ("b", 11, 0), ("c", 0, 6), ("d", 11, 6)]
    violations = find_out_of_limits(mixed, limits)
    assert [v[0] for v in violations] == ["b", "c", "d"]  # "a" excluded, order preserved
    assert "x=11.0000" in violations[0][3] and "y" not in violations[0][3]
    assert "y=6.0000" in violations[1][3] and "x" not in violations[1][3]
    assert "x=11.0000" in violations[2][3] and "y=6.0000" in violations[2][3]  # both axes bad

    try:
        validate_points_within_limits(mixed, limits)
        raise AssertionError("expected ValueError for out-of-limits points")
    except ValueError as e:
        msg = str(e)
        assert "3 of 4" in msg
        assert "b" in msg and "c" in msg and "d" in msg  # every violation listed, not just first

    est = estimate_scan_time_s(*grid_dims_from_range([0, 50], [0, 50], 2.0), dwell_time_s=8.0)
    print(f"26x26 grid @ 8s dwell, 1 pass -> {est/60:.1f} min estimated scan time")
    est3 = estimate_scan_time_s(*grid_dims_from_range([0, 50], [0, 50], 2.0), dwell_time_s=8.0, passes=3)
    assert est3 == est * 3
    print(f"26x26 grid @ 8s dwell, 3 passes -> {est3/60:.1f} min estimated scan time")
    print("scan_params smoke test OK")
