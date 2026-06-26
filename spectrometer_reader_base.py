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


def get_spectrometer_reader(config) -> "SpectrometerReader":
    """
    Factory. Returns the appropriate SpectrometerReader based on
    config["oes"]["backend"]:
      "pyseabreeze" → PySeabreezeSpectrometerReader (real hardware)
      "mock"        → MockSpectrometerReader
      None          → MockSpectrometerReader
    """
    oes_cfg = config.get("oes", {})
    backend = oes_cfg.get("backend")
    integration_time_us = oes_cfg.get("integration_time_us", 20000)
    num_averages = oes_cfg.get("num_averages", 3)
    boxcar_width = oes_cfg.get("boxcar_width", 2)

    if backend == "pyseabreeze":
        from pyseabreeze_spectrometer_reader import PySeabreezeSpectrometerReader
        return PySeabreezeSpectrometerReader(
            integration_time_us=integration_time_us,
            num_averages=num_averages,
            boxcar_width=boxcar_width,
        )

    if backend not in (None, "mock"):
        raise ValueError(f"Unknown oes.backend: {backend!r}. Expected 'pyseabreeze' or 'mock'.")

    from mock_spectrometer_reader import MockSpectrometerReader
    return MockSpectrometerReader(
        integration_time_us=integration_time_us,
        num_averages=num_averages,
        boxcar_width=boxcar_width,
    )
