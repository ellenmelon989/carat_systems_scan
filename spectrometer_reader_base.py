"""
spectrometer_reader_base.py — shared interface for all OES reader backends.

Mirrors the ir_reader_base.py pattern: every backend implements read() and
returns the same SpectrumReading shape, so scan_manager.py doesn't care
which backend is plugged in.
"""

from __future__ import annotations
import time
from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class SpectrumReading:
    wavelengths: Optional[np.ndarray]   # nm, None if read failed
    intensities: Optional[np.ndarray]   # a.u., None if read failed
    integration_time_us: int
    num_averages: int
    boxcar_width: int
    saturated: bool
    timestamp: float
    error: Optional[str] = None


class SpectrometerReader:
    """Base interface. All backends implement read() and set_integration_time()."""

    def read(self) -> SpectrumReading:
        raise NotImplementedError

    def set_integration_time(self, integration_time_us: int):
        raise NotImplementedError

    def close(self):
        pass
