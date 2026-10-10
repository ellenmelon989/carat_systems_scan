"""
wavelength_calibration.py -- build and sanity-check the OES wavelength axis.

Why this exists (2026-10-10)
----------------------------
seabreeze's pyseabreeze backend builds every Ocean Optics wavelength axis
the same way:

    wavelength(i) = c0 + c1*i + c2*i**2 + c3*i**3     (i = pixel 0..N-1)

taking c0..c3 from EEPROM slots 1..4. That's right for most Ocean
spectrometers, but NOT for our ADC1000-USB: its channel-0 coefficients
sit in slots 2..5 (slot 1 holds the literal value 1). Using slots 1..4
shifts every coefficient up one power of i, which is exactly what the
2026-10 scan data showed -- an axis running 1 nm -> ~2,000,000 nm,
fit exactly by (1, 201.55, 0.3321, 2.16e-5).

This module is deliberately pure (no seabreeze import, no hardware) so
the slot choice and the validity checks can be unit-tested: callers pass
in the raw EEPROM slot strings, get back the axis plus a provenance
record that is written into metadata.yaml and oes.h5 with every scan.

Sources (config.yaml -> oes.wavelength_calibration.source)
  auto             ADC1000-USB -> eeprom_slots_2_5, anything else ->
                   eeprom_slots_1_4. Default.
  eeprom_slots_2_5 Force the ADC1000 channel-0 layout.
  eeprom_slots_1_4 Force seabreeze's standard layout.
  manual           Use oes.wavelength_calibration.coeffs ([c0, c1, c2, c3])
                   -- e.g. coefficients from an Ocean calibration sheet or
                   a fit to known plasma lines. Overrides the EEPROM.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence

import numpy as np

SOURCES = ("auto", "eeprom_slots_2_5", "eeprom_slots_1_4", "manual")

# What we'll accept as a physically plausible axis for a UV-VIS-NIR
# Ocean bench. Deliberately generous: this is a "the calibration is
# garbage" detector, not an accuracy check. The real accuracy check is
# known plasma lines (H-alpha 656.28, H-beta 486.13, H-gamma 434.05).
PLAUSIBLE_MIN_NM = 150.0
PLAUSIBLE_MAX_NM = 1150.0
MIN_SPAN_NM = 100.0


@dataclass
class WavelengthCalibration:
    wavelengths: np.ndarray              # nm, one per pixel
    coeffs: list                         # [c0, c1, c2, c3] actually used
    source: str                          # resolved source, never "auto"
    requested_source: str                # what config asked for
    eeprom_slots: dict = field(default_factory=dict)  # slot -> raw string read
    valid: bool = True
    problems: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    @property
    def range_nm(self):
        if self.wavelengths is None or len(self.wavelengths) == 0:
            return (float("nan"), float("nan"))
        return (float(self.wavelengths[0]), float(self.wavelengths[-1]))

    def summary(self) -> str:
        lo, hi = self.range_nm
        state = "VALID" if self.valid else "INVALID"
        s = (f"wavelength calibration {state}: source={self.source} "
             f"range={lo:.1f}-{hi:.1f} nm coeffs={self.coeffs}")
        if self.problems:
            s += " problems: " + "; ".join(self.problems)
        return s

    def to_record(self) -> dict:
        """Plain-python dict for metadata.yaml / HDF5 attrs (no numpy)."""
        lo, hi = self.range_nm
        return {
            "valid": bool(self.valid),
            "source": self.source,
            "requested_source": self.requested_source,
            "coeffs": [float(c) for c in self.coeffs],
            "range_nm": [lo, hi],
            "n_pixels": int(len(self.wavelengths)),
            "eeprom_slots": {int(k): str(v) for k, v in self.eeprom_slots.items()},
            "problems": list(self.problems),
            "warnings": list(self.warnings),
        }


def resolve_source(requested: str, model: Optional[str]) -> str:
    if requested not in SOURCES:
        raise ValueError(
            f"oes.wavelength_calibration.source must be one of {SOURCES}, got {requested!r}")
    if requested != "auto":
        return requested
    if model and "ADC1000" in str(model).upper():
        return "eeprom_slots_2_5"
    return "eeprom_slots_1_4"


def _parse_float(raw) -> Optional[float]:
    if raw is None:
        return None
    try:
        return float(str(raw).replace("\x00", "").strip())
    except ValueError:
        return None


def evaluate(coeffs: Sequence[float], n_pixels: int) -> np.ndarray:
    i = np.arange(n_pixels, dtype=np.float64)
    return sum(float(c) * i ** k for k, c in enumerate(coeffs))


def validate(wavelengths: np.ndarray) -> list:
    """Return a list of human-readable problems ([] == plausible)."""
    problems = []
    w = np.asarray(wavelengths, dtype=np.float64)
    if w.size < 2:
        return ["wavelength axis has fewer than 2 points"]
    if not np.all(np.isfinite(w)):
        problems.append("axis contains NaN/inf")
        return problems
    if not np.all(np.diff(w) > 0):
        problems.append("axis is not strictly increasing")
    lo, hi = float(w.min()), float(w.max())
    if lo < PLAUSIBLE_MIN_NM or hi > PLAUSIBLE_MAX_NM:
        problems.append(
            f"axis spans {lo:.1f}-{hi:.1f} nm, outside the plausible "
            f"{PLAUSIBLE_MIN_NM:.0f}-{PLAUSIBLE_MAX_NM:.0f} nm window "
            "(EEPROM slots probably misread or uncalibrated)")
    if hi - lo < MIN_SPAN_NM:
        problems.append(f"axis only spans {hi - lo:.1f} nm")
    return problems


def build_calibration(
    n_pixels: int,
    requested_source: str = "auto",
    model: Optional[str] = None,
    read_slot: Optional[Callable[[int], str]] = None,
    manual_coeffs: Optional[Sequence[float]] = None,
) -> WavelengthCalibration:
    """
    read_slot(k) -> raw EEPROM string for slot k. Not called for
    source='manual'. Slots 0..6 are recorded (best effort) for provenance
    regardless of which ones feed the axis, so a scan's metadata always
    shows what the EEPROM actually held.
    """
    source = resolve_source(requested_source, model)
    slots = {}
    warnings = []

    if read_slot is not None and source != "manual":
        for k in range(0, 7):
            try:
                slots[k] = read_slot(k)
            except Exception as exc:  # recorded, judged below only if needed
                slots[k] = f"<read failed: {exc}>"

    if source == "manual":
        if not manual_coeffs or len(manual_coeffs) < 2:
            raise ValueError(
                "oes.wavelength_calibration.source is 'manual' but "
                "oes.wavelength_calibration.coeffs is missing (need [c0, c1, c2, c3])")
        coeffs = [float(c) for c in manual_coeffs]
    else:
        if read_slot is None:
            raise ValueError(f"source={source!r} needs an EEPROM reader")
        first = 2 if source == "eeprom_slots_2_5" else 1
        coeffs = []
        problems = []
        for k in range(first, first + 4):
            v = _parse_float(slots.get(k))
            if v is None:
                problems.append(f"EEPROM slot {k} is not a number: {slots.get(k)!r}")
                v = float("nan")
            coeffs.append(v)
        # The cubic term is tiny (~1e-9..1e-8) and least likely to be
        # stored correctly; a missing c3 still gives a usable axis, so
        # warn instead of failing. Missing c0..c2 is a hard failure.
        if np.isnan(coeffs[3]) and not any(np.isnan(coeffs[:3])):
            warnings.append(f"slot {first + 3} (cubic term) unreadable; using c3=0")
            coeffs[3] = 0.0
            problems = [p for p in problems if f"slot {first + 3} " not in p]
        if problems:
            w = np.full(n_pixels, np.nan)
            return WavelengthCalibration(
                w, coeffs, source, requested_source, slots, False, problems, warnings)

    w = evaluate(coeffs, n_pixels)
    problems = validate(w)
    return WavelengthCalibration(
        w, coeffs, source, requested_source, slots, not problems, problems, warnings)
