"""
oceandirect_spectrometer_reader.py — OES backend via Ocean Insight's
OceanDirect SDK (oceandirect Python package).

Prerequisites:
  - pip install oceandirect   (Ocean Insight's official Python SDK)
  - OceanDirect runtime installed: https://www.oceaninsight.com/products/software/
  - USB driver for the spectrometer (e.g. Ocean USB2000+, HR2000+, etc.)

Unlike pyseabreeze, OceanDirect does NOT share the python-seabreeze package,
so there is no backend-lock conflict. Both readers can be imported in the same
process safely.

OceanDirect API sketch (fill in from installed SDK docs / oceandirect.py):
  from oceandirect.OceanDirectAPI import OceanDirectAPI, Feature
  api = OceanDirectAPI()
  ids = api.find_usb_devices()
  device = api.open_device(ids[0])
  device.set_integration_time(integration_time_us)
  wavelengths = device.get_wavelengths()
  intensities = device.get_corrected_spectrum()  # or get_raw_spectrum()
  device.close_device()
"""

import time
from typing import Optional

import numpy as np

try:
    from .spectrometer_reader_base import SpectrometerReader, SpectrumReading
except ImportError:
    # Fallback for running this file directly, where relative imports
    # don't work because there's no parent package.
    from spectrometer_reader_base import SpectrometerReader, SpectrumReading


class OceanDirectSpectrometerReader(SpectrometerReader):
    """
    SpectrometerReader backend using the OceanDirect SDK.

    TODO: Fill in real OceanDirect API calls once the SDK is available.
    The interface contract (read() -> SpectrumReading, set_integration_time(),
    close()) is fixed — scan_manager.py will not need to change.
    """

    def __init__(self, integration_time_us: int = 20_000, num_averages: int = 1,
                 boxcar_width: int = 0, serial: Optional[str] = None):
        self.integration_time_us = integration_time_us
        self.num_averages = num_averages
        self.boxcar_width = boxcar_width
        self._serial = serial
        self._device = None
        self._wavelengths = None
        self._max_intensity = None
        self._init_error: Optional[str] = None

        try:
            # TODO: replace with real OceanDirect initialization, e.g.:
            #   from oceandirect.OceanDirectAPI import OceanDirectAPI
            #   self._api = OceanDirectAPI()
            #   ids = self._api.find_usb_devices()
            #   device_id = next((i for i in ids if ...), ids[0])
            #   self._device = self._api.open_device(device_id)
            #   self._device.set_integration_time(self.integration_time_us)
            #   self._wavelengths = np.array(self._device.get_wavelengths())
            #   self._max_intensity = self._device.get_maximum_intensity()
            raise NotImplementedError(
                "OceanDirect backend is scaffolded but not yet implemented. "
                "Fill in the SDK calls in oceandirect_spectrometer_reader.py."
            )
        except NotImplementedError as e:
            self._init_error = str(e)
        except Exception as e:
            self._init_error = str(e)

    def set_integration_time(self, integration_time_us: int):
        self.integration_time_us = integration_time_us
        if self._device is not None:
            # TODO: self._device.set_integration_time(integration_time_us)
            pass

    def read(self) -> SpectrumReading:
        timestamp = time.time()
        if self._device is None:
            return SpectrumReading(
                wavelengths=None, intensities=None,
                integration_time_us=self.integration_time_us,
                num_averages=self.num_averages, boxcar_width=self.boxcar_width,
                saturated=False, timestamp=timestamp, error=self._init_error,
            )
        try:
            # TODO: replace with real OceanDirect acquisition, e.g.:
            #   accum = None
            #   for _ in range(self.num_averages):
            #       frame = np.array(self._device.get_corrected_spectrum())
            #       accum = frame.copy() if accum is None else accum + frame
            #   intensities = accum / self.num_averages
            #   if self.boxcar_width > 0:
            #       from .spectrometer_reader_base import boxcar_smooth
            #       intensities = boxcar_smooth(intensities, self.boxcar_width)
            raise NotImplementedError("OceanDirect read() not yet implemented.")
        except Exception as e:
            return SpectrumReading(
                wavelengths=None, intensities=None,
                integration_time_us=self.integration_time_us,
                num_averages=self.num_averages, boxcar_width=self.boxcar_width,
                saturated=False, timestamp=timestamp, error=str(e),
            )

    def close(self):
        if self._device is not None:
            # TODO: self._api.close_device(self._device)
            self._device = None
