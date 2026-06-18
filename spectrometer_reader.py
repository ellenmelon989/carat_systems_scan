"""
spectrometer_reader.py

Responsible for acquiring spectra from the Ocean Optics ADC1000-USB
spectrometer via python-seabreeze.

IMPORTANT: ADC1000-USB support lives in the pyseabreeze backend
(EXPERIMENTAL, added v2.1.0), NOT cseabreeze. The backend must be
selected with seabreeze.use('pyseabreeze') BEFORE seabreeze.spectrometers
is imported anywhere in the process. This module enforces that ordering
internally so callers don't need to worry about import order elsewhere
in the codebase.

Requires:
    pip install "seabreeze>=2.1.0" pyusb
    libusb-1.0.dll placed in C:\\Windows\\System32 (Windows only)
"""

from __future__ import annotations
import seabreeze
seabreeze.use("pyseabreeze")  # MUST run before any seabreeze.spectrometers import,
                                # anywhere in the process — see module docstring.
import time
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class SpectrumReading:
    """Container for a single acquired spectrum and its metadata."""
    wavelengths: np.ndarray
    intensities: np.ndarray
    integration_time_us: int
    timestamp: float = field(default_factory=time.time)
    saturated: bool = False
    error: Optional[str] = None


class SpectrometerInterface(ABC):
    """Abstract base class for spectrometer hardware access."""

    @abstractmethod
    def connect(self) -> bool:
        """Establish connection to the spectrometer. Returns True on success."""
        raise NotImplementedError

    @abstractmethod
    def disconnect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def set_integration_time(self, micros: int) -> None:
        raise NotImplementedError

    @abstractmethod
    def acquire_spectrum(self) -> SpectrumReading:
        raise NotImplementedError

    @abstractmethod
    def is_connected(self) -> bool:
        raise NotImplementedError


class MockSpectrometer(SpectrometerInterface):
    """
    Mock spectrometer for development without hardware present.
    Generates a synthetic spectrum with a few fake emission peaks so
    downstream code (feature extraction, plotting) has something
    realistic to chew on.
    """

    def __init__(self, num_pixels: int = 2048):
        self._connected = False
        self._integration_time_us = 20000
        self._num_pixels = num_pixels
        self._wavelengths = np.linspace(200, 900, num_pixels)

    def connect(self) -> bool:
        logger.info("[MOCK] Connecting to simulated spectrometer.")
        self._connected = True
        return True

    def disconnect(self) -> None:
        logger.info("[MOCK] Disconnecting simulated spectrometer.")
        self._connected = False

    def set_integration_time(self, micros: int) -> None:
        self._integration_time_us = micros

    def is_connected(self) -> bool:
        return self._connected

    def acquire_spectrum(self) -> SpectrumReading:
        if not self._connected:
            return SpectrumReading(
                wavelengths=np.array([]),
                intensities=np.array([]),
                integration_time_us=self._integration_time_us,
                error="Not connected",
            )

        # Fake baseline + noise + a couple of synthetic peaks (e.g. CH ~430nm, H-alpha ~656nm)
        baseline = np.random.normal(50, 5, self._num_pixels)
        peak_ch = 2000 * np.exp(-0.5 * ((self._wavelengths - 430) / 3) ** 2)
        peak_halpha = 5000 * np.exp(-0.5 * ((self._wavelengths - 656) / 2) ** 2)
        intensities = baseline + peak_ch + peak_halpha

        return SpectrumReading(
            wavelengths=self._wavelengths.copy(),
            intensities=intensities,
            integration_time_us=self._integration_time_us,
            saturated=False,
        )


class ADC1000Spectrometer(SpectrometerInterface):
    """
    Real hardware interface to the Ocean Optics ADC1000-USB via
    python-seabreeze, pyseabreeze backend.

    NOTE: pyseabreeze backend selection MUST happen before
    `seabreeze.spectrometers` is imported anywhere in the running
    process. This class handles that internally and guards against
    being instantiated after some other module has already imported
    seabreeze.spectrometers with a different backend active.
    """

    def __init__(self, serial_number: Optional[str] = None):
        self._serial_number = serial_number
        self._device = None
        self._connected = False
        self._integration_time_us = 20000

    def _verify_pyseabreeze_active() -> None:
        """Defensive check: confirm pyseabreeze backend actually took effect."""
        import seabreeze.spectrometers as sb
        backend_module = sb.SeaBreezeAPI.__module__
        if "pyseabreeze" not in backend_module:
            raise RuntimeError(
                f"Expected pyseabreeze backend, but seabreeze.spectrometers is "
                f"using '{backend_module}'. Some other import likely loaded "
                f"seabreeze.spectrometers before backend selection occurred. "
                f"Check import order — spectrometer_reader.py must be imported "
                f"first, or at least before any other seabreeze usage."
        )

    def connect(self) -> bool:
        self._verify_pyseabreeze_active()
        devices = seabreeze.spectrometers.list_devices()
        logger.info("Discovered seabreeze devices: %s", devices)
        spec = seabreeze.spectrometers.Spectrometer
        if not devices:
            logger.error(
                "No seabreeze devices found. Confirm ADC1000-USB is "
                "plugged in, libusb-1.0.dll is in System32, and the "
                "device shows up in Windows Device Manager."
            )
            self._connected = False
            return False

        try:
            if self._serial_number:
                self._device = spec.from_serial_number(self._serial_number)
            else:
                self._device = spec.from_first_available()
            self._device.integration_time_micros(self._integration_time_us)
            self._connected = True
            logger.info("Connected to spectrometer: %s", self._device.model)
            return True
        except Exception as exc:
            logger.error("Failed to connect to spectrometer: %s", exc)
            self._connected = False
            return False

    def disconnect(self) -> None:
        if self._device is not None:
            try:
                self._device.close()
            except Exception as exc:
                logger.warning("Error closing spectrometer device: %s", exc)
        self._connected = False
        self._device = None

    def is_connected(self) -> bool:
        return self._connected

    def set_integration_time(self, micros: int) -> None:
        self._integration_time_us = micros
        if self._connected and self._device is not None:
            self._device.integration_time_micros(micros)

    def acquire_spectrum(self) -> SpectrumReading:
        if not self._connected or self._device is None:
            return SpectrumReading(
                wavelengths=np.array([]),
                intensities=np.array([]),
                integration_time_us=self._integration_time_us,
                error="Not connected",
            )
        try:
            wavelengths = self._device.wavelengths()
            intensities = self._device.intensities()
            saturated = bool(np.max(intensities) >= 0.98 * np.iinfo(np.uint16).max) \
                if intensities.size else False
            return SpectrumReading(
                wavelengths=wavelengths,
                intensities=intensities,
                integration_time_us=self._integration_time_us,
                saturated=saturated,
            )
        except Exception as exc:
            logger.error("Spectrum acquisition failed: %s", exc)
            return SpectrumReading(
                wavelengths=np.array([]),
                intensities=np.array([]),
                integration_time_us=self._integration_time_us,
                error=str(exc),
            )


def get_spectrometer(use_mock: bool = False, serial_number: Optional[str] = None) -> SpectrometerInterface:
    """
    Factory function. Attempts real hardware first unless use_mock is
    forced True; falls back to MockSpectrometer if connection fails,
    so callers always get a usable object.
    """
    if use_mock:
        logger.info("Using MockSpectrometer (forced).")
        spec = MockSpectrometer()
        spec.connect()
        return spec

    real = ADC1000Spectrometer(serial_number=serial_number)
    if real.connect():
        return real

    logger.warning("Falling back to MockSpectrometer — real hardware not available.")
    mock = MockSpectrometer()
    mock.connect()
    return mock


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    # Quick standalone test — run this directly on-site to validate
    # ADC1000-USB enumeration before building anything else on top of it.
    spec = get_spectrometer(use_mock=False)
    print(f"Connected: {spec.is_connected()}, type: {type(spec).__name__}")

    reading = spec.acquire_spectrum()
    if reading.error:
        print(f"Acquisition error: {reading.error}")
    else:
        print(f"Got {len(reading.intensities)} pixels, "
              f"max intensity: {reading.intensities.max():.1f}, "
              f"saturated: {reading.saturated}")

    spec.disconnect()