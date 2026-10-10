"""
dump_spectrometer_eeprom.py -- print the spectrometer's EEPROM slots and
the wavelength axis each slot layout would produce.

Run on the instrument PC, spectrometer plugged in, scanner GUI CLOSED
(only one process can hold the USB device):

    python tools/dump_spectrometer_eeprom.py

What to look for (ADC1000-USB, 2026-10-10): slot 1 = 1, slots 2-5 =
wavelength coefficients c0..c3 (c0 ~ 200, c1 ~ 0.33, c2 ~ 1e-5,
c3 ~ -1e-8). The "slots 2-5" axis should run ~200-850 nm and be marked
VALID; the "slots 1-4" axis (what seabreeze uses by default) is the
broken 1 -> ~2,000,000 nm one.

Add --plasma-check with the plasma ON to also take one spectrum and
print where the brightest line near 656 nm lands (H-alpha = 656.28 nm).
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from readers.pyseabreeze_spectrometer_reader import (
    _install_adc1000_read_retry_workaround,
    _prepare_windows_libusb_runtime,
)
from readers.wavelength_calibration import build_calibration


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--plasma-check", action="store_true",
                    help="also read one spectrum and locate H-alpha (plasma must be on)")
    ap.add_argument("--integration-time-us", type=int, default=500_000)
    args = ap.parse_args()

    _prepare_windows_libusb_runtime()
    import seabreeze
    seabreeze.use("pyseabreeze")
    _install_adc1000_read_retry_workaround()
    from seabreeze.spectrometers import Spectrometer

    spec = Spectrometer.from_first_available()
    eeprom = spec._dev.f.eeprom
    print(f"Device: {spec.model}  serial={spec.serial_number}  pixels={spec.pixels}")
    print("\nEEPROM slots:")
    for k in range(0, 18):
        try:
            print(f"  slot {k:2d}: {eeprom.eeprom_read_slot(k)!r}")
        except Exception as exc:
            print(f"  slot {k:2d}: <read failed: {exc}>")

    print()
    cals = {}
    for source in ("eeprom_slots_1_4", "eeprom_slots_2_5", "auto"):
        cal = build_calibration(spec.pixels, source, spec.model,
                                lambda k: eeprom.eeprom_read_slot(k))
        cals[source] = cal
        print(f"[{source:>16}] {cal.summary()}")
        for w in cal.warnings:
            print(f"                   warning: {w}")

    if args.plasma_check:
        cal = cals["auto"]
        spec.integration_time_micros(args.integration_time_us)
        spec.intensities()                      # discard stale first frame
        counts = spec.intensities()
        w = cal.wavelengths
        win = (w > 640) & (w < 675)
        if not np.any(win):
            print("\nplasma check: axis doesn't cover 640-675 nm -- calibration unusable")
        else:
            j = np.flatnonzero(win)[np.argmax(counts[win])]
            base = float(np.median(counts))
            print(f"\nplasma check @ {args.integration_time_us} us: brightest pixel in "
                  f"640-675 nm is pixel {j} = {w[j]:.2f} nm, "
                  f"{counts[j]:.0f} counts (median {base:.0f}). "
                  f"H-alpha is 656.28 nm -> offset {w[j] - 656.28:+.2f} nm")
    spec.close()


if __name__ == "__main__":
    main()
