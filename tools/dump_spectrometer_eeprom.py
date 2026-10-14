"""
dump_spectrometer_eeprom.py -- print the spectrometer's EEPROM slots and
the wavelength axis each slot layout would produce, then (optionally)
check that real light reaches the detector and lands on known lines.

Run on the instrument PC, spectrometer plugged in, scanner GUI CLOSED
(only one process can hold the USB device):

    python tools/dump_spectrometer_eeprom.py
    python tools/dump_spectrometer_eeprom.py --line-check plasma
    python tools/dump_spectrometer_eeprom.py --line-check fluorescent

EEPROM (ADC1000-USB serial ADUD7590, confirmed 2026-10-14): slot 1 = 1,
slots 2-5 = c0..c3 = 201.5547, 0.3320928, 2.159012e-05, -1.387201e-08,
axis 201.6-852.8 nm. Slot 5 fills all 15 payload bytes; seabreeze's own
read drops its last character, so this tool shows both reads.

--line-check takes one spectrum per integration time (default 20 ms,
100 ms, 500 ms, 2 s) and reports overall signal level plus where the
strongest pixel near each reference line lands:
  plasma       H-alpha 656.28, H-beta 486.13, CH 431.4, C2 516.5
  fluorescent  Hg 404.66, 435.83, 546.07 nm; Eu 611.6 nm (room lights --
               a wavelength check that doesn't need the plasma on)
A line is only reported as FOUND if it stands well above the background;
otherwise the offset printed would be meaningless.
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
from readers.wavelength_calibration import build_calibration, make_eeprom_slot_reader

LINES = {
    "plasma": [("H-alpha", 656.28), ("H-beta", 486.13), ("CH A-X", 431.4),
               ("C2 Swan (0,0)", 516.5)],
    "fluorescent": [("Hg", 404.66), ("Hg", 435.83), ("Hg", 546.07),
                    ("Eu (phosphor)", 611.6)],
}
SEARCH_HALF_WIDTH_NM = 5.0
FOUND_SIGMA = 8.0   # peak must exceed background by this many noise sigmas


def line_report(w, counts, lines):
    counts = np.asarray(counts, dtype=float)
    base = float(np.median(counts))
    # robust noise estimate: MAD of pixel-to-pixel differences
    noise = float(1.4826 * np.median(np.abs(np.diff(counts))) / np.sqrt(2)) or 1.0
    print(f"    detector: min {counts.min():.0f}  median {base:.0f}  "
          f"max {counts.max():.0f}  noise ~{noise:.1f} counts")
    top = np.argsort(counts)[-5:][::-1]
    print("    5 brightest pixels: " + ", ".join(
        f"{w[j]:.1f} nm ({counts[j]:.0f})" for j in top))
    for name, nm in lines:
        win = (w > nm - SEARCH_HALF_WIDTH_NM) & (w < nm + SEARCH_HALF_WIDTH_NM)
        if not np.any(win):
            print(f"    {name:>14} {nm:7.2f} nm: outside axis")
            continue
        j = np.flatnonzero(win)[np.argmax(counts[win])]
        snr = (counts[j] - base) / noise
        if snr >= FOUND_SIGMA:
            print(f"    {name:>14} {nm:7.2f} nm: FOUND at {w[j]:.2f} nm "
                  f"(offset {w[j] - nm:+.2f} nm, {counts[j] - base:.0f} counts "
                  f"above background, {snr:.0f} sigma)")
        else:
            print(f"    {name:>14} {nm:7.2f} nm: not above background "
                  f"({snr:.1f} sigma) -- no offset measured")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--line-check", choices=sorted(LINES),
                    help="take spectra and look for reference lines")
    ap.add_argument("--plasma-check", action="store_true",
                    help="same as --line-check plasma (kept for compatibility)")
    ap.add_argument("--integration-times-us", type=int, nargs="+",
                    default=[20_000, 100_000, 500_000, 2_000_000])
    ap.add_argument("--save-csv", metavar="DIR",
                    help="also save each check spectrum as CSV into DIR")
    args = ap.parse_args()
    if args.plasma_check and not args.line_check:
        args.line_check = "plasma"

    _prepare_windows_libusb_runtime()
    import seabreeze
    seabreeze.use("pyseabreeze")
    _install_adc1000_read_retry_workaround()
    from seabreeze.spectrometers import Spectrometer

    spec = Spectrometer.from_first_available()
    eeprom = spec._dev.f.eeprom
    read_slot = make_eeprom_slot_reader(eeprom)
    print(f"Device: {spec.model}  serial={spec.serial_number}  pixels={spec.pixels}  "
          f"max_intensity={spec.max_intensity}")
    print(f"Complete-payload EEPROM read available: {read_slot.complete_payload}")
    print("\nEEPROM slots (complete read | seabreeze's normal read):")
    for k in range(0, 18):
        try:
            full = read_slot(k)
        except Exception as exc:
            full = f"<read failed: {exc}>"
        try:
            normal = eeprom.eeprom_read_slot(k)
        except Exception as exc:
            normal = f"<read failed: {exc}>"
        flag = "" if full == normal else "   <-- differs (truncated by seabreeze)"
        print(f"  slot {k:2d}: {full!r:24} | {normal!r}{flag}")

    print()
    cals = {}
    for source in ("eeprom_slots_1_4", "eeprom_slots_2_5", "auto"):
        cal = build_calibration(spec.pixels, source, spec.model, read_slot)
        cals[source] = cal
        print(f"[{source:>16}] {cal.summary()}")
        for w in cal.warnings:
            print(f"                   warning: {w}")

    if args.line_check:
        cal = cals["auto"]
        if not cal.valid:
            print("\nline check skipped: 'auto' calibration is INVALID")
        else:
            w = cal.wavelengths
            print(f"\nLine check ({args.line_check}):")
            for t_us in args.integration_times_us:
                spec.integration_time_micros(t_us)
                spec.intensities()               # discard stale first frame
                counts = spec.intensities()
                sat = " SATURATED" if np.max(counts) >= spec.max_intensity else ""
                print(f"\n  integration {t_us / 1000:g} ms{sat}")
                line_report(w, counts, LINES[args.line_check])
                if args.save_csv:
                    os.makedirs(args.save_csv, exist_ok=True)
                    path = os.path.join(args.save_csv,
                                        f"{args.line_check}_{t_us}us.csv")
                    np.savetxt(path, np.column_stack([w, counts]), delimiter=",",
                               header="wavelength_nm,counts", comments="")
                    print(f"    saved {path}")
    spec.close()


if __name__ == "__main__":
    main()
