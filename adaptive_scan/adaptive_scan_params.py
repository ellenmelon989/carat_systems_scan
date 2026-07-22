"""
adaptive_scan_params.py

Operator-parameter validation for the adaptive (edge-following) raster
scan — see docs/adaptive_scan_spec.md. Mirrors scan_params.py's pattern
(one validate_* function per parameter, sanity bounds as module-level
constants) but with one deliberate difference:

Per Roy (2026-07-22): every parameter here is direct operator input at
run time for a real scan — there is NO real-use default to fall back on,
unlike scan_params.py's DWELL_TIME_DEFAULT_S / STEP_SIZE_DEFAULT_MM (which
ARE meant as real, usable starting points for the precision scan path).
So this module intentionally does NOT export *_DEFAULT constants for
those parameters — only MIN/MAX sanity bounds, used purely to validate
whatever the operator enters. Any default-looking values that appear in
this file's own __main__ smoke test are test scaffolding only (mirrors
scan_manager.py's --smoke-test convention) and must not be reused as a
real scan's fallback.

ONE exception: coarse grid cell count. Roy confirmed this one DOES have a
real, usable default (100) even for actual trials, while still being
operator-adjustable — so COARSE_GRID_CELLS_DEFAULT is exported and safe
to use as a real fallback, unlike every other constant in this file.
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# Signal selection (parameter 1)
# ---------------------------------------------------------------------------

# The three pyrometer-derived scalar fields always available per reading
# (see readers/ir_reader_base.py's IRReading). ir_dilution stays unusable
# until ir.pac.dilution_tag_name is confirmed on the real PAC (see
# tools/list_pac_strategy_vars.py) — validate_signal_name() does not know
# that at config-parse time, only that the tag is configured; a chosen but
# unconfigured dilution signal will read back as NaN at scan time, same as
# the existing precision scan path already handles it.
IR_SIGNAL_FIELDS = ("ir_temp_c", "ir_emissivity", "ir_dilution")

OES_SIGNAL_PREFIX = "oes_"


def oes_signal_name(feature_name: str) -> str:
    """Canonical signal name for a configured OES feature, e.g. 'oes_C2_Swan'."""
    return f"{OES_SIGNAL_PREFIX}{feature_name}"


def available_signal_names(config: dict) -> list[str]:
    """
    Every signal name the operator may choose from parameter 1: the 3 IR
    fields, plus one per OES feature already defined under oes.features in
    config.yaml (CH, C2_Swan, H_alpha, H_beta by default — see
    scan_manager.extract_features for how those get integrated from raw
    spectra). No new wavelengths are introduced here; this mode only picks
    among signals the rest of the codebase already knows how to read.
    """
    names = list(IR_SIGNAL_FIELDS)
    oes_features = config.get("oes", {}).get("features", {})
    names.extend(oes_signal_name(name) for name in oes_features)
    return names


def validate_signal_name(name: str, config: dict) -> str:
    """Raise ValueError if `name` isn't one of available_signal_names(config)."""
    valid = available_signal_names(config)
    if name not in valid:
        raise ValueError(
            f"signal_name={name!r} is not available. Valid options for this "
            f"config: {valid}"
        )
    return name


# ---------------------------------------------------------------------------
# On/off-wafer thresholds (parameter 2) — sanity check only; the two
# threshold VALUES are entirely operator/signal-dependent (a dilution
# threshold and a temperature threshold live on completely different
# scales), so there's no numeric range to validate beyond "not equal to
# NaN / both finite." Polarity (which threshold is larger) is inferred by
# EdgeDetector itself, not validated here.
# ---------------------------------------------------------------------------

def validate_thresholds(on_threshold: float, off_threshold: float) -> tuple[float, float]:
    on_threshold = float(on_threshold)
    off_threshold = float(off_threshold)
    if on_threshold != on_threshold or off_threshold != off_threshold:  # NaN check
        raise ValueError("on_threshold/off_threshold must not be NaN")
    return on_threshold, off_threshold


# ---------------------------------------------------------------------------
# Confirm count (parameter 3)
# ---------------------------------------------------------------------------

CONFIRM_COUNT_MIN = 1
CONFIRM_COUNT_MAX = 50  # sanity cap, not a hardware limit — see spec §6:
                          # the real ceiling on a sane N is the field-of-view
                          # blur width at the chosen reading interval, which
                          # this validator has no way to know; it only
                          # catches typos/nonsense, not a bad-for-this-FOV N.


def validate_confirm_count(n: int) -> int:
    n = int(n)
    if not (CONFIRM_COUNT_MIN <= n <= CONFIRM_COUNT_MAX):
        raise ValueError(
            f"confirm_count={n} outside valid range [{CONFIRM_COUNT_MIN}, {CONFIRM_COUNT_MAX}]"
        )
    return n


# ---------------------------------------------------------------------------
# Reading interval (parameter 4) — two modes, operator picks one per run.
# See spec §7: acquisition-time is the direction-safe choice (a fixed pulse
# count maps to a different real mm spacing depending on travel direction,
# per real_newport_motion.py's documented backlash/hysteresis behavior);
# motor-pulse mode is still offered as a real option, just direction-
# sensitive, not defaulted to.
# ---------------------------------------------------------------------------

READING_INTERVAL_MODES = ("time_s", "motor_pulses")

READING_INTERVAL_TIME_S_MIN = 0.02
READING_INTERVAL_TIME_S_MAX = 30.0

READING_INTERVAL_PULSES_MIN = 1
READING_INTERVAL_PULSES_MAX = 200_000


def validate_reading_interval(mode: str, value: float | int) -> tuple[str, float | int]:
    if mode not in READING_INTERVAL_MODES:
        raise ValueError(
            f"reading_interval mode={mode!r} not recognized. Expected one of {READING_INTERVAL_MODES}"
        )
    if mode == "time_s":
        value = float(value)
        if not (READING_INTERVAL_TIME_S_MIN <= value <= READING_INTERVAL_TIME_S_MAX):
            raise ValueError(
                f"reading_interval time_s={value} outside valid range "
                f"[{READING_INTERVAL_TIME_S_MIN}, {READING_INTERVAL_TIME_S_MAX}] seconds"
            )
    else:
        value = int(value)
        if not (READING_INTERVAL_PULSES_MIN <= value <= READING_INTERVAL_PULSES_MAX):
            raise ValueError(
                f"reading_interval motor_pulses={value} outside valid range "
                f"[{READING_INTERVAL_PULSES_MIN}, {READING_INTERVAL_PULSES_MAX}]"
            )
    return mode, value


# ---------------------------------------------------------------------------
# Y raster spacing (parameter 5)
# ---------------------------------------------------------------------------

Y_RASTER_SPACING_MM_MIN = 0.1
Y_RASTER_SPACING_MM_MAX = 50.0


def validate_y_raster_spacing_mm(spacing_mm: float) -> float:
    spacing_mm = float(spacing_mm)
    if not (Y_RASTER_SPACING_MM_MIN <= spacing_mm <= Y_RASTER_SPACING_MM_MAX):
        raise ValueError(
            f"y_raster_spacing_mm={spacing_mm} outside valid range "
            f"[{Y_RASTER_SPACING_MM_MIN}, {Y_RASTER_SPACING_MM_MAX}] mm"
        )
    return spacing_mm


# ---------------------------------------------------------------------------
# Max X/Y travel safety limit (parameter 6) — same bound shape used for
# either axis; see adaptive_scan.py's TravelGuard for how this gets
# enforced live (§9 of the spec — a runtime cumulative-displacement check,
# NOT the existing static preflight_check(), since this mode's points
# aren't known in advance).
# ---------------------------------------------------------------------------

MAX_TRAVEL_MM_MIN = 1.0
MAX_TRAVEL_MM_MAX = 1000.0  # matches the largest sane single-axis travel on
                              # this mount's existing soft_limits (+/-250mm
                              # each way => 500mm span); doubled for headroom
                              # since this mode may explore outward from an
                              # arbitrary, not-necessarily-centered start.


def validate_max_travel_mm(travel_mm: float) -> float:
    travel_mm = float(travel_mm)
    if not (MAX_TRAVEL_MM_MIN <= travel_mm <= MAX_TRAVEL_MM_MAX):
        raise ValueError(
            f"max_travel_mm={travel_mm} outside valid range "
            f"[{MAX_TRAVEL_MM_MIN}, {MAX_TRAVEL_MM_MAX}] mm"
        )
    return travel_mm


# ---------------------------------------------------------------------------
# Coarse grid cell count — the one parameter WITH a real-use default (100),
# per Roy. Still operator-adjustable.
# ---------------------------------------------------------------------------

COARSE_GRID_CELLS_MIN = 4
COARSE_GRID_CELLS_MAX = 10_000
COARSE_GRID_CELLS_DEFAULT = 100  # real default, safe to use as a fallback —
                                    # unlike every other constant in this file.


def validate_coarse_grid_cells(n: int) -> int:
    n = int(n)
    if not (COARSE_GRID_CELLS_MIN <= n <= COARSE_GRID_CELLS_MAX):
        raise ValueError(
            f"coarse_grid_cells={n} outside valid range "
            f"[{COARSE_GRID_CELLS_MIN}, {COARSE_GRID_CELLS_MAX}]"
        )
    return n


if __name__ == "__main__":
    # Smoke test
    test_config = {"oes": {"features": {"CH": 431.0, "C2_Swan": 516.0}}}

    names = available_signal_names(test_config)
    assert names == ["ir_temp_c", "ir_emissivity", "ir_dilution", "oes_CH", "oes_C2_Swan"], names
    assert validate_signal_name("ir_dilution", test_config) == "ir_dilution"
    assert validate_signal_name("oes_C2_Swan", test_config) == "oes_C2_Swan"
    try:
        validate_signal_name("oes_H_alpha", test_config)  # not configured in test_config
        raise AssertionError("expected ValueError for unavailable signal")
    except ValueError:
        pass
    print("signal name validation OK")

    assert validate_thresholds(0.9, 0.7) == (0.9, 0.7)
    try:
        validate_thresholds(float("nan"), 0.7)
        raise AssertionError("expected ValueError for NaN threshold")
    except ValueError:
        pass
    print("threshold validation OK")

    assert validate_confirm_count(3) == 3
    try:
        validate_confirm_count(0)
        raise AssertionError("expected ValueError for confirm_count=0")
    except ValueError:
        pass
    print("confirm count validation OK")

    assert validate_reading_interval("time_s", 0.5) == ("time_s", 0.5)
    assert validate_reading_interval("motor_pulses", 200) == ("motor_pulses", 200)
    try:
        validate_reading_interval("bogus_mode", 1)
        raise AssertionError("expected ValueError for unknown mode")
    except ValueError:
        pass
    try:
        validate_reading_interval("time_s", 100.0)  # over max
        raise AssertionError("expected ValueError for out-of-range time_s")
    except ValueError:
        pass
    print("reading interval validation OK")

    assert validate_y_raster_spacing_mm(2.0) == 2.0
    try:
        validate_y_raster_spacing_mm(0.0)
        raise AssertionError("expected ValueError for spacing below min")
    except ValueError:
        pass
    print("y raster spacing validation OK")

    assert validate_max_travel_mm(300.0) == 300.0
    try:
        validate_max_travel_mm(5000.0)
        raise AssertionError("expected ValueError for travel above max")
    except ValueError:
        pass
    print("max travel validation OK")

    assert validate_coarse_grid_cells(100) == 100
    assert COARSE_GRID_CELLS_DEFAULT == 100
    try:
        validate_coarse_grid_cells(1)
        raise AssertionError("expected ValueError for cell count below min")
    except ValueError:
        pass
    print("coarse grid cell count validation OK")

    print("adaptive_scan_params smoke test OK")
