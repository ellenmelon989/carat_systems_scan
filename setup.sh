#!/usr/bin/env bash
# One-time environment setup for the diagnostics PC (Linux/Mac).
# Run this once: bash setup.sh
set -e

echo "=== Creating virtual environment ==="
python3 -m venv venv
source venv/bin/activate

echo "=== Installing Python dependencies ==="
pip install --upgrade pip
pip install -r requirements.txt

echo "=== Running seabreeze OS setup (udev rules / drivers) ==="
seabreeze_os_setup

echo ""
echo "=== Setup complete ==="
echo "If this is the first time setting up the spectrometer, unplug and"
echo "replug the ADC1000-USB now, then verify with:"
echo "  source venv/bin/activate"
echo "  python -c \"import seabreeze.spectrometers as sb; print(sb.list_devices())\""
