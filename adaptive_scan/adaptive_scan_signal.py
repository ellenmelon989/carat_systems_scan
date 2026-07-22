"""
adaptive_scan_signal.py

Signal dispatch for the adaptive (edge-following) raster scan — see
docs/adaptive_scan_spec.md §2, §5.

Every reading in this mode is a single raw, unaveraged poll (NOT
read_averaged() / scan.dwell_time_s — that's the precision scan path's
per-point averaging, and reusing it here would cost 1.5-5+ hours for a
full wafer at ~1mm spacing; see spec §5). The same poll is used both to
feed EdgeDetector's threshold comparison and, if it falls between
confirmed boundaries, as the raw value saved to the log — there is no
separate "detection" vs. "recorded" acquisition.

This module's job is just: take one poll of whichever readers are
available, package every field that's cheap to have already read (not
just the one the operator selected for detection) into one flat record,
and pick out the one scalar value the operator chose as parameter 1
(spec §4) to feed to EdgeDetector / the coarse map.
"""

from __future__ import annotations

# --- repo-root import bootstrap -------------------------------------------
# Needed for this file's own __main__ smoke test, which imports
# readers.mock_ir_reader / readers.mock_spectrometer_reader (top-level
# sibling packages) -- see scan/scan_manager.py for the full rationale.
import os as _os
import sys as _sys

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)
# ---------------------------------------------------------------------------

import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from adaptive_scan.adaptive_scan_params import IR_SIGNAL_FIELDS, OES_SIGNAL_PREFIX


def extract_features(wavelengths, intensities, features_cfg, window_nm):
    """
    Extract intensity values for each named spectral feature by
    integrating (summing) intensities within +/- window_nm of the
    feature's center wavelength.

    Deliberately a standalone copy of scan_manager.extract_features, not
    an import from it — importing scan_manager here would pull in its
    full module-level import chain (oes_store -> h5py, motion_controller
    -> pylablib) purely to reach one small, pure numpy function, coupling
    this mode's lightweight signal dispatch to the precision scan path's
    heavier hardware-adjacent dependencies for no reason (per spec §14:
    this is meant to be a separate module, not layered onto
    scan_manager.py). Same logic, so a feature name (e.g. "C2_Swan") means
    the same thing in both scan modes.
    """
    feature_values = {}
    for name, center_nm in features_cfg.items():
        mask = np.abs(wavelengths - center_nm) <= window_nm
        feature_values[name] = float(np.sum(intensities[mask])) if np.any(mask) else float("nan")
    return feature_values


@dataclass
class RawSignals:
    """
    Everything cheap to have on hand from one poll of the IR reader and
    (optionally) the spectrometer. Kept flat and complete — not just the
    operator-selected detection signal — because spec §10 asks to retain
    "raw signal value(s)" per reading, and whichever fields aren't the
    selected detection signal are still worth having in the log for free.
    """
    timestamp: float
    ir_temp_c: float
    ir_emissivity: float
    ir_dilution: Optional[float]
    ir_error: bool
    oes_feature_values: dict = field(default_factory=dict)
    oes_saturated: bool = False
    oes_error: bool = False


def read_raw_signals(ir_reader, spectrometer=None, oes_features_cfg: Optional[dict] = None,
                      feature_window_nm: float = 1.0) -> RawSignals:
    """
    One raw poll of the IR reader, and (if a spectrometer + feature config
    are given) one raw poll of the spectrometer with named features
    extracted the same way the precision scan path does
    (scan_manager.extract_features — same window-integration logic, so a
    feature named e.g. "C2_Swan" means the same thing in both scan modes).

    ir_reader.read() (not read_averaged()) — a single instantaneous poll,
    per this mode's "no averaging at read time" design (see module
    docstring). Errors are captured as flags (ir_error/oes_error) rather
    than raised — a single failed poll during edge-following should be
    treated as an ambiguous/missing reading by the caller (e.g. skip
    feeding it to EdgeDetector, or feed NaN and let EdgeDetector's
    _classify treat it as neither on nor off), not abort the whole scan
    the way a hard failure would.
    """
    reading = ir_reader.read()
    oes_features_cfg = oes_features_cfg or {}

    oes_feature_values: dict = {}
    oes_saturated = False
    oes_error = False

    if spectrometer is not None and oes_features_cfg:
        try:
            spec_reading = spectrometer.read()
            if spec_reading.error:
                oes_error = True
            elif spec_reading.wavelengths is not None:
                oes_feature_values = extract_features(
                    spec_reading.wavelengths, spec_reading.intensities,
                    oes_features_cfg, feature_window_nm,
                )
            oes_saturated = bool(getattr(spec_reading, "saturated", False))
        except Exception:
            oes_error = True

    return RawSignals(
        timestamp=time.time(),
        ir_temp_c=reading.value_c,
        ir_emissivity=reading.emissivity,
        ir_dilution=reading.dilution,
        ir_error=bool(reading.error) if reading.error is not None else reading.stale,
        oes_feature_values=oes_feature_values,
        oes_saturated=oes_saturated,
        oes_error=oes_error,
    )


def select_value(raw: RawSignals, signal_name: str) -> float:
    """
    Pick out the one scalar this scan run is using for edge detection and
    the coarse map — signal_name is whatever the operator chose for
    parameter 1 (spec §4), already validated against
    adaptive_scan_params.available_signal_names() before the scan starts.

    Returns NaN (never raises) for a missing/unread value — e.g. an OES
    feature that failed to extract this poll, or an IR field the reader
    didn't populate. A NaN reaching EdgeDetector.update() will compare
    false against both thresholds and land in "ambiguous" by construction
    (NaN comparisons are always False in Python), which is the right
    behavior: a missing reading is neither confirmed-on nor confirmed-off
    evidence, exactly like a genuine FOV-blur reading — see edge_detector.py.
    """
    if signal_name in IR_SIGNAL_FIELDS:
        value = getattr(raw, signal_name)
        return float(value) if value is not None else float("nan")

    if signal_name.startswith(OES_SIGNAL_PREFIX):
        feature_name = signal_name[len(OES_SIGNAL_PREFIX):]
        value = raw.oes_feature_values.get(feature_name)
        return float(value) if value is not None else float("nan")

    raise ValueError(f"Unrecognized signal_name: {signal_name!r}")


if __name__ == "__main__":
    from readers.mock_ir_reader import MockIRReader
    from readers.mock_spectrometer_reader import MockSpectrometerReader

    ir = MockIRReader(base_temp_c=850.0, dilution=1.0)
    oes = MockSpectrometerReader()
    features_cfg = {"CH": 431.0, "C2_Swan": 516.0, "H_alpha": 656.3, "H_beta": 486.1}

    raw = read_raw_signals(ir, oes, features_cfg, feature_window_nm=1.0)
    print(raw)

    assert not raw.ir_error
    assert 840.0 <= raw.ir_temp_c <= 860.0
    assert raw.ir_dilution is not None
    assert set(raw.oes_feature_values) == set(features_cfg)

    v_temp = select_value(raw, "ir_temp_c")
    v_dilution = select_value(raw, "ir_dilution")
    v_oes = select_value(raw, "oes_C2_Swan")
    print(f"selected ir_temp_c={v_temp}, ir_dilution={v_dilution}, oes_C2_Swan={v_oes}")
    assert v_temp == raw.ir_temp_c
    assert v_dilution == raw.ir_dilution
    assert v_oes == raw.oes_feature_values["C2_Swan"]

    # Missing OES feature -> NaN, not a crash.
    v_missing = select_value(raw, "oes_not_a_real_feature")
    assert v_missing != v_missing  # NaN != NaN
    print("Missing-feature NaN fallback OK")

    # IR-only mode (no spectrometer/features configured) still works.
    raw_ir_only = read_raw_signals(ir)
    assert raw_ir_only.oes_feature_values == {}
    print("IR-only mode OK")

    print("adaptive_scan_signal smoke test OK")
