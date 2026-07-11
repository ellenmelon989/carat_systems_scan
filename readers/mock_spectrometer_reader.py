"""
mock_spectrometer_reader.py — synthetic OES reader for dev/testing without
the spectrometer connected. Generates a fake spectrum with a few Gaussian
emission-line-like peaks so scan_manager.py's data pipeline (averaging,
boxcar, saturation logic, CSV/spectrum file writing) can be built and tested
before the real hardware path is finalized.
"""

import time
from typing import Optional

import numpy as np

try:
    from .spectrometer_reader_base import SpectrometerReader, SpectrumReading, boxcar_smooth
except ImportError:
    # Fallback for running this file directly (e.g. python mock_spectrometer_reader.py),
    # where relative imports don't work because there's no parent package.
    from spectrometer_reader_base import SpectrometerReader, SpectrumReading, boxcar_smooth


class MockSpectrometerReader(SpectrometerReader):
    def __init__(self, integration_time_us: int = 100_000, num_averages: int = 1,
                 boxcar_width: int = 0, num_pixels: int = 2048,
                 wavelength_range_nm: tuple = (200.0, 900.0), noise_level: float = 50.0):
        self.integration_time_us = integration_time_us
        self.num_averages = num_averages
        self.boxcar_width = boxcar_width
        self._wavelengths = np.linspace(*wavelength_range_nm, num_pixels)
        self._noise_level = noise_level
        self._max_intensity = 65535.0  # typical 16-bit ADC ceiling, mock only

        # rough stand-ins for CH (~430nm), C2 Swan (~516nm), H-alpha (~656nm)
        self._peaks = [(430.0, 8000.0, 3.0), (516.0, 12000.0, 4.0), (656.0, 5000.0, 1.5)]

    def set_integration_time(self, integration_time_us: int):
        self.integration_time_us = integration_time_us

    def read(self) -> SpectrumReading:
        timestamp = time.time()
        baseline = np.random.normal(200, self._noise_level, size=self._wavelengths.shape)
        spectrum = baseline.copy()
        for center, height, width in self._peaks:
            spectrum += height * np.exp(-0.5 * ((self._wavelengths - center) / width) ** 2)
        spectrum = np.clip(spectrum, 0, self._max_intensity)

        if self.boxcar_width > 0:
            spectrum = boxcar_smooth(spectrum, self.boxcar_width)

        saturated = bool(np.max(spectrum) >= self._max_intensity)

        return SpectrumReading(
            wavelengths=self._wavelengths,
            intensities=spectrum,
            integration_time_us=self.integration_time_us,
            num_averages=self.num_averages,
            boxcar_width=self.boxcar_width,
            saturated=saturated,
            timestamp=timestamp,
        )


if __name__ == "__main__":
    reader = MockSpectrometerReader()
    reading = reader.read()
    print(f"Got {len(reading.wavelengths)} points, "
          f"peak intensity={reading.intensities.max():.1f}, "
          f"saturated={reading.saturated}")
