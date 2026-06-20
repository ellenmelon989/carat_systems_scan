"""
check_seabreeze_device.py — RUN THIS FIRST. Before touching
spectrometer_reader.py, confirm the ADC1000-USB actually enumerates under
pyseabreeze. If this finds zero devices, the problem is almost certainly
one of:

  1. libusb-1.0.dll isn't in C:\\Windows\\System32 (must be the 64-bit
     version, from libusb.info -> MinGW64/dll folder). This is the most
     common silent failure point -- pip can't install it for you, and the
     symptom is exactly "zero devices found", no error message.
  2. The ADC1000 doesn't show up in Windows Device Manager at all (check
     there before blaming the Python side).
  3. pyusb isn't installed (`pip show pyusb` in the carat_scans env).

seabreeze.use('pyseabreeze') MUST be called before any import of
seabreeze.spectrometers -- anywhere in the process. If some other module
imports seabreeze.spectrometers first, the backend silently locks to the
default (cseabreeze) and this won't behave as expected. Run this script
standalone, not imported alongside other seabreeze-touching code, until
you've confirmed it works.
"""

import seabreeze
seabreeze.use('pyseabreeze')

from seabreeze.spectrometers import list_devices

devices = list_devices()
print(f"Found {len(devices)} device(s).")

for d in devices:
    print(f"  model={d.model}  serial={d.serial_number}")

if not devices:
    print()
    print("No devices found. Before debugging further, check in this order:")
    print("  1. libusb-1.0.dll (64-bit) is in C:\\Windows\\System32")
    print("  2. ADC1000 appears in Windows Device Manager")
    print("  3. pip show pyusb   (confirm it's installed in this env)")
