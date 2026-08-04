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

import os
import time
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from .spectrometer_reader_base import SpectrometerReader, SpectrumReading, boxcar_smooth
except ImportError:
    # Fallback for running this file directly (e.g. python pyseabreeze_spectrometer_reader.py),
    # where relative imports don't work because there's no parent package.
    from spectrometer_reader_base import SpectrometerReader, SpectrumReading, boxcar_smooth


_LIBUSB_DLL_DIRECTORY_HANDLE = None


def _prepare_windows_libusb_runtime():
    """Make a venv-installed libusb-package DLL discoverable on Windows.

    This keeps the USB runtime inside the virtual environment instead of
    requiring a copy in C:\\Windows\\System32.  If libusb-package is absent,
    SeaBreeze retains its normal system-library discovery behavior.
    """
    global _LIBUSB_DLL_DIRECTORY_HANDLE

    if os.name != "nt":
        return

    try:
        import libusb_package
    except ImportError:
        return

    library_path = libusb_package.get_library_path()
    if not library_path:
        return

    dll_directory = str(Path(library_path).resolve().parent)
    path_entries = os.environ.get("PATH", "").split(os.pathsep)
    if dll_directory not in path_entries:
        os.environ["PATH"] = dll_directory + os.pathsep + os.environ.get("PATH", "")

    # Python 3.8+ uses an explicit DLL directory API on Windows.  Retain the
    # returned handle for the life of this module; discarding it removes the
    # directory from the process search path.
    if hasattr(os, "add_dll_directory") and _LIBUSB_DLL_DIRECTORY_HANDLE is None:
        try:
            _LIBUSB_DLL_DIRECTORY_HANDLE = os.add_dll_directory(dll_directory)
        except OSError:
            # Some unpatched Windows 7 installations do not expose the API.
            # The PATH entry above remains the compatible fallback.
            pass


def _adc1000_get_spectrum_raw_with_retry(feature):
    """Retry an ADC1000 acquisition after a transient short USB response.

    The ADC1000 spectrum is one 4097-byte transfer.  Some libusb routes first
    return only one 64/65-byte response (or an overflow/timeout) while the
    device and endpoint synchronize.  That short response is not a fragment
    of a spectrum and must be discarded.  Request a fresh spectrum, as the
    successful workaround reported for this device family does.
    """
    from usb.core import USBError

    expected = int(feature._spectrum_raw_length)
    transport = feature.protocol.transport
    timeout_ms = int(feature._integration_time * 1e-3 + transport.default_timeout_ms)
    mode = transport._default_read_spectrum_endpoint
    attempts = []

    for attempt in range(1, 5):
        feature.protocol.send(0x09)
        try:
            response = feature.protocol.receive(
                size=expected,
                timeout_ms=timeout_ms,
                mode=mode,
            )
        except USBError as exc:
            attempts.append(f"attempt {attempt}: {exc}")
            if attempt == 4:
                raise RuntimeError(
                    "ADC1000 did not return a complete spectrum after four "
                    f"requests ({'; '.join(attempts)})."
                ) from exc
            continue

        received = len(response)
        if received == expected:
            return np.frombuffer(response, dtype=np.uint8).copy()
        if received > expected:
            raise RuntimeError(
                f"ADC1000 returned too much spectrum data: "
                f"{received} bytes, expected {expected}."
            )
        attempts.append(f"attempt {attempt}: {received}/{expected} bytes")

    raise RuntimeError(
        "ADC1000 returned only short USB responses after four requests "
        f"({'; '.join(attempts)})."
    )


def _install_adc1000_read_retry_workaround():
    """Patch only SeaBreeze's ADC1000 feature class for this process."""
    from seabreeze.pyseabreeze.features.spectrometer import (
        SeaBreezeSpectrometerFeatureADC,
    )

    if getattr(SeaBreezeSpectrometerFeatureADC, "_carat_read_retry", False):
        return

    SeaBreezeSpectrometerFeatureADC._get_spectrum_raw = (
        _adc1000_get_spectrum_raw_with_retry
    )
    SeaBreezeSpectrometerFeatureADC._carat_read_retry = True


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
            _prepare_windows_libusb_runtime()
            import seabreeze
            seabreeze.use('pyseabreeze')
            _install_adc1000_read_retry_workaround()
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
        print(f"Connected to {reader.spec.model}, serial={reader.spec.serial_number}")
        print(f"Got {len(reading.wavelengths)} points, "
              f"saturated={reading.saturated}, "
              f"peak intensity={reading.intensities.max():.1f}")
    reader.close()
