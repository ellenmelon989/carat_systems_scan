@echo off
REM One-time environment setup for the diagnostics PC (Windows).
REM Run this once: setup.bat

echo === Creating virtual environment ===
python -m venv venv
call venv\Scripts\activate.bat

echo === Installing Python dependencies ===
pip install --upgrade pip
pip install -r requirements.txt

echo === Running seabreeze OS setup (drivers) ===
seabreeze_os_setup

echo.
echo === Setup complete ===
echo If this is the first time setting up the spectrometer, unplug and
echo replug the ADC1000-USB now, then verify with:
echo   venv\Scripts\activate.bat
echo   python -c "import seabreeze.spectrometers as sb; print(sb.list_devices())"
