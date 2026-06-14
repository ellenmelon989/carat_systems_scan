"""
spectrometer_reader.py

Responsible for acquiring spectra from the Ocean Optics ADC1000-USB
(via python-seabreeze, cseabreeze backend) or a mock source for
development without hardware.

Provides: SpectrometerReader (abstract), MockSpectrometerReader,
SeabreezeSpectrometerReader.
"""

from abc import ABC, abstractmethod
import numpy as np


class SpectrometerReader(ABC):
    """Abstract interface for OES spectrum acquisition."""

    @abstractmethod
    def connect(self):
        """Establish connection to the spectrometer. Should be safe
        to call multiple times (idempotent)."""
        raise NotImplementedError

    @abstractmethod
    def is_connected(self):
        """Return True if the spectrometer is connected and ready."""
        raise NotImplementedError

    @abstractmethod
    def set_integration_time(self, integration_time_us):
        raise NotImplementedError

    @abstractmethod
    def acquire_spectrum(self):
        """Return (wavelengths_nm, intensities) as numpy arrays."""
        raise NotImplementedError

    def acquire_averaged(self, num_averages, boxcar_width=0):
        """
        Acquire num_averages spectra, average them, and optionally
        apply boxcar smoothing across pixels.

        Returns (wavelengths_nm, intensities, saturated)
        """
        spectra = []
        wavelengths = None

        for _ in range(num_averages):
            wl, intens = self.acquire_spectrum()
            wavelengths = wl
            spectra.append(intens)

        avg_intensities = np.mean(spectra, axis=0)

        if boxcar_width > 0:
            avg_intensities = self._boxcar_smooth(avg_intensities, boxcar_width)

        saturated = self._check_saturation(avg_intensities)

        return wavelengths, avg_intensities, saturated

    @staticmethod
    def _boxcar_smooth(intensities, width):
        """Simple moving-average boxcar smoothing across pixels."""
        if width <= 0:
            return intensities
        kernel_size = 2 * width + 1
        kernel = np.ones(kernel_size) / kernel_size
        return np.convolve(intensities, kernel, mode="same")

    def _check_saturation(self, intensities):
        """Override in subclasses with the device's actual max value."""
        return False


class MockSpectrometerReader(SpectrometerReader):
    """
    Simulated spectrometer for development without hardware.
    Produces a synthetic spectrum with peaks near the tracked
    plasma emission features.
    """

    FEATURES_NM = {
        "CH": 431.0,
        "C2_Swan": 516.0,
        "H_beta": 486.1,
        "H_alpha": 656.3,
    }

    def __init__(self, num_pixels=2048, wl_range_nm=(350.0, 750.0)):
        self.num_pixels = num_pixels
        self.wl_range_nm = wl_range_nm
        self._connected = False
        self._integration_time_us = 20000

    def connect(self):
        self._connected = True
        print("[MockSpectrometer] Connected (simulated)")

    def is_connected(self):
        return self._connected

    def set_integration_time(self, integration_time_us):
        self._integration_time_us = integration_time_us

    def acquire_spectrum(self):
        if not self._connected:
            raise RuntimeError("Spectrometer not connected")

        wavelengths = np.linspace(self.wl_range_nm[0], self.wl_range_nm[1], self.num_pixels)
        intensities = np.random.normal(loc=100, scale=5, size=self.num_pixels)

        for center_nm in self.FEATURES_NM.values():
            peak = 2000 * (self._integration_time_us / 20000.0)
            intensities += peak * np.exp(-((wavelengths - center_nm) ** 2) / (2 * 0.5 ** 2))

        intensities = np.clip(intensities, 0, 16383)  # 14-bit ADC max
        return wavelengths, intensities

    def _check_saturation(self, intensities):
        return bool(np.max(intensities) >= 16383)


class SeabreezeSpectrometerReader(SpectrometerReader):
    """
    Reads spectra from the Ocean Optics ADC1000-USB via
    python-seabreeze (cseabreeze backend).

    Setup (one-time, on the diagnostics PC):
        pip install seabreeze
        seabreeze_os_setup
        (unplug/replug the ADC1000-USB)

    TODO (Phase 3):
    - Confirm seabreeze.list_devices() detects the ADC1000-USB
    - Confirm wavelength calibration looks correct vs OceanView baseline
    - Confirm spectrum_max_value for saturation checks (device-specific)
    """

    def __init__(self):
        self._spec = None

    def connect(self):
        import seabreeze.spectrometers as sb

        devices = sb.list_devices()
        if not devices:
            raise RuntimeError(
                "No spectrometer devices found. Check USB connection "
                "and that seabreeze_os_setup has been run."
            )

        self._spec = sb.Spectrometer(devices[0])
        print(f"[SeabreezeSpectrometer] Connected: {self._spec.model} "
              f"(serial {self._spec.serial_number})")

    def is_connected(self):
        return self._spec is not None

    def set_integration_time(self, integration_time_us):
        if self._spec is None:
            raise RuntimeError("Spectrometer not connected")
        self._spec.integration_time_micros(integration_time_us)

    def acquire_spectrum(self):
        if self._spec is None:
            raise RuntimeError("Spectrometer not connected")
        wavelengths = self._spec.wavelengths()
        intensities = self._spec.intensities()
        return wavelengths, intensities

    def _check_saturation(self, intensities):
        # TODO: replace 16383 with the confirmed max ADC value for
        # the attached detector head (check device spec / EEPROM).
        return bool(np.max(intensities) >= 16383)


def get_spectrometer_reader(config):
    """
    Factory function. Returns a SeabreezeSpectrometerReader if
    hardware is available and connects successfully, otherwise
    falls back to MockSpectrometerReader.
    """
    try:
        reader = SeabreezeSpectrometerReader()
        reader.connect()
        return reader
    except Exception as e:
        print(f"[spectrometer_reader] Falling back to mock: {e}")
        reader = MockSpectrometerReader()
        reader.connect()
        return reader


if __name__ == "__main__":
    oes_cfg = {"integration_time_us": 20000, "num_averages": 3, "boxcar_width": 2}

    reader = MockSpectrometerReader()
    reader.connect()
    reader.set_integration_time(oes_cfg["integration_time_us"])

    wl, intens, saturated = reader.acquire_averaged(
        num_averages=oes_cfg["num_averages"],
        boxcar_width=oes_cfg["boxcar_width"],
    )
    print(f"Spectrum shape: {wl.shape}, max intensity: {intens.max():.1f}, saturated: {saturated}")
