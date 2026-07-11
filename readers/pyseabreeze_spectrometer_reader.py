"""
pyseabreeze_spectrometer_reader.py — real OES backend via python-seabreeze's
pyseabreeze backend. This is the only documented path for the ADC1000-USB
(added in seabreeze v2.1.0, tagged EXPERIMENTAL by python-seabreeze).

Prerequisites (see tools/check_seabreeze_device.py -- run that first):
  - pip install "seabreeze>=2.1.0" pyusb   (in the carat_scans env)
  - libusb-1.0.dll (64-bit) placed in C:\\Windows\\System32

seabreeze.use('pyseabreeze') and the seabreeze.spectrometers import are
intentionally deferred to __init__ (not module level). This ensures the
backend selection only runs when this reader is actually instantiated,
so importing this module does not globally lock the seabreeze backend.
That keeps oceandirect_spectrometer_reader.py (or any other backend) safe
to import in the same process without interference.

API surface below (Spectrometer.intensities/wavelengths/integration_time_micros,
list_devices, from_first_available/from_serial_number, max_intensity property)
was checked directly against the installed seabreeze package, not assumed
from memory.
"""

import time
from typing import Optional

import numpy as np

try:
    from .spectrometer_reader_base import SpectrometerReader, SpectrumReading, boxcar_smooth
except ImportError:
    # Fallback for running this file directly (e.g. python pyseabreeze_spectrometer_reader.py),
    # where relative imports don't work because there's no parent package.
    from spectrometer_reader_base import SpectrometerReader, SpectrumReading, boxcar_smooth


class PySeabreezeSpectrometerReader(SpectrometerReader):
    def __init__(self, integration_time_us: int = 100_000, num_averages: int = 1,
                 boxcar_width: int = 0, serial: Optional[str] = None):
        self.integration_time_us = integration_time_us
        self.num_averages = num_averages
        self.boxcar_width = boxcar_width
        self.spec = None
        self._wavelengths = None
        self._max_intensity = None
        self._init_error: Optional[str] = None

        try:
            import seabreeze
            seabreeze.use('pyseabreeze')
            from seabreeze.spectrometers import Spectrometer
            self.spec = (Spectrometer.from_serial_number(serial) if serial
                         else Spectrometer.from_first_available())
            self.spec.integration_time_micros(self.integration_time_us)
            self._wavelengths = self.spec.wavelengths()
            self._max_intensity = self.spec.max_intensity  # device-reported saturation
            # NOTE: seabreeze's own docs caveat that real saturation can occur
            # below this value -- treat it as an upper bound, not a guarantee.
        except Exception as e:
            self._init_error = str(e)

    def set_integration_time(self, integration_time_us: int):
        self.integration_time_us = integration_time_us
        if self.spec:
            self.spec.integration_time_micros(integration_time_us)

    def read(self) -> SpectrumReading:
        timestamp = time.time()
        if self.spec is None:
            return SpectrumReading(
                wavelengths=None, intensities=None,
                integration_time_us=self.integration_time_us,
                num_averages=self.num_averages, boxcar_width=self.boxcar_width,
                saturated=False, timestamp=timestamp, error=self._init_error,
            )
        try:
            accum = None
            for _ in range(self.num_averages):
                intensities = self.spec.intensities()
                accum = intensities.copy() if accum is None else accum + intensities
            intensities = accum / self.num_averages

            if self.boxcar_width > 0:
                intensities = boxcar_smooth(intensities, self.boxcar_width)

            saturated = bool(np.max(intensities) >= self._max_intensity)

            return SpectrumReading(
                wavelengths=self._wavelengths,
                intensities=intensities,
                integration_time_us=self.integration_time_us,
                num_averages=self.num_averages,
                boxcar_width=self.boxcar_width,
                saturated=saturated,
                timestamp=timestamp,
            )
        except Exception as e:
            return SpectrumReading(
                wavelengths=None, intensities=None,
                integration_time_us=self.integration_time_us,
                num_averages=self.num_averages, boxcar_width=self.boxcar_width,
                saturated=False, timestamp=timestamp, error=str(e),
            )

    def close(self):
        if self.spec:
            self.spec.close()


if __name__ == "__main__":
    # Run tools/check_seabreeze_device.py first -- this will hang/fail confusingly
    # if no device is enumerated yet.
    reader = PySeabreezeSpectrometerReader(integration_time_us=100_000, num_averages=3)
    reading = reader.read()
    if reading.error:
        print(f"Error: {reading.error}")
    else:
        print(f"Got {len(reading.wavelengths)} points, "
              f"saturated={reading.saturated}, "
              f"peak intensity={reading.intensities.max():.1f}")
    reader.close()
