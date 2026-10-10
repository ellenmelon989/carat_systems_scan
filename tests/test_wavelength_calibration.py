"""
2026-10-10: OES wavelength axis built from the wrong EEPROM slots.

The ADC1000-USB keeps its channel-0 wavelength coefficients in EEPROM
slots 2-5; seabreeze reads 1-4, which on this unit gave the October
scans an axis of 1 -> ~2,000,000 nm (an exact cubic with coefficients
(1, 201.55, 0.3321, 2.16e-5) -- the real ones shifted one power up).
These tests pin the slot choice, the validity flagging, the scan-level
recording, and the post-hoc correction tool.
"""

import csv
import json
import os as _os
import shutil
import subprocess
import sys as _sys
import tempfile
import unittest
from unittest.mock import patch

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)

import numpy as np
import yaml

from readers.wavelength_calibration import build_calibration, evaluate, resolve_source

# Recovered from the 2026-10 scan data (slots 2-4); slot 5 (c3) is the
# value that puts the one visible line on H-alpha -- representative only.
TRUE = [201.5547, 0.3320928, 2.159012e-05, -1.35e-08]
ADC_EEPROM = {0: "ADC1A1234", 1: "1", 2: "201.5547", 3: "0.3320928",
              4: "2.159012E-05", 5: "-1.35E-08", 6: "0"}

try:
    import h5py  # noqa: F401
    import xarray  # noqa: F401
    _HAVE_H5 = True
except ImportError:
    _HAVE_H5 = False


def _reader(eeprom):
    def read(k):
        if k not in eeprom:
            raise IOError("no such slot")
        return eeprom[k]
    return read


class SlotSelectionTests(unittest.TestCase):
    def test_auto_on_adc1000_uses_slots_2_to_5(self):
        cal = build_calibration(2048, "auto", "ADC1000-USB", _reader(ADC_EEPROM))
        self.assertEqual(cal.source, "eeprom_slots_2_5")
        self.assertTrue(cal.valid, cal.problems)
        np.testing.assert_allclose(cal.coeffs, TRUE)
        lo, hi = cal.range_nm
        self.assertAlmostEqual(lo, 201.5547, places=3)
        self.assertTrue(840 < hi < 870, hi)

    def test_seabreeze_default_slots_reproduce_the_broken_axis_and_are_flagged(self):
        cal = build_calibration(2048, "eeprom_slots_1_4", "ADC1000-USB", _reader(ADC_EEPROM))
        self.assertFalse(cal.valid)
        self.assertAlmostEqual(cal.wavelengths[0], 1.0)
        self.assertGreater(cal.wavelengths[-1], 1e6)
        self.assertTrue(any("plausible" in p for p in cal.problems), cal.problems)

    def test_auto_on_other_models_keeps_seabreeze_layout(self):
        self.assertEqual(resolve_source("auto", "USB2000PLUS"), "eeprom_slots_1_4")
        self.assertEqual(resolve_source("auto", None), "eeprom_slots_1_4")

    def test_unknown_source_rejected(self):
        with self.assertRaises(ValueError):
            resolve_source("slots_3_6", "ADC1000-USB")

    def test_unreadable_cubic_term_warns_but_stays_valid(self):
        eeprom = {**ADC_EEPROM, 5: "\x00\x00"}
        cal = build_calibration(2048, "auto", "ADC1000-USB", _reader(eeprom))
        self.assertTrue(cal.valid, cal.problems)
        self.assertEqual(cal.coeffs[3], 0.0)
        self.assertTrue(cal.warnings)

    def test_garbage_slot_is_flagged_not_raised(self):
        eeprom = {**ADC_EEPROM, 3: "not a number"}
        cal = build_calibration(2048, "auto", "ADC1000-USB", _reader(eeprom))
        self.assertFalse(cal.valid)
        self.assertTrue(any("slot 3" in p for p in cal.problems), cal.problems)
        rec = cal.to_record()
        json.dumps(rec)  # must be serializable for HDF5 attrs
        self.assertEqual(rec["eeprom_slots"][3], "not a number")

    def test_manual_overrides_eeprom(self):
        cal = build_calibration(2048, "manual", "ADC1000-USB", _reader(ADC_EEPROM),
                                manual_coeffs=[200.0, 0.3, 0.0, 0.0])
        self.assertTrue(cal.valid)
        self.assertEqual(cal.eeprom_slots, {})
        self.assertAlmostEqual(cal.wavelengths[-1], 200 + 0.3 * 2047)

    def test_manual_without_coeffs_is_an_error(self):
        with self.assertRaises(ValueError):
            build_calibration(2048, "manual", "ADC1000-USB", None)

    def test_non_monotonic_axis_flagged(self):
        cal = build_calibration(2048, "manual", None, None,
                                manual_coeffs=[800.0, -0.3, 0.0, 0.0])
        self.assertFalse(cal.valid)
        self.assertTrue(any("increasing" in p for p in cal.problems))


class _FakeSpectrometer:
    """Connected reader whose calibration came out invalid."""
    def __init__(self, cal):
        self.wavelength_calibration = cal
        self._wavelengths = cal.wavelengths

    wavelengths = property(lambda self: self._wavelengths)
    calibration_record = property(lambda self: self.wavelength_calibration.to_record())

    def set_integration_time(self, *_):
        pass

    def close(self):
        pass


@unittest.skipUnless(_HAVE_H5, "h5py/xarray not installed")
class ScanRecordsCalibrationTests(unittest.TestCase):
    def _config(self):
        with open(_os.path.join(_REPO_ROOT, "config.example.yaml"), encoding="utf-8-sig") as f:
            cfg = yaml.safe_load(f)
        cfg["motion"]["controller"] = None
        cfg["motion"]["motion_enabled"] = True
        cfg["ir"]["source"] = "mock"
        cfg["oes"]["enabled"] = True
        cfg["output"]["base_dir"] = tempfile.mkdtemp()
        cfg["output"].pop("oes_hdf5", None)
        return cfg

    def _manager(self, source):
        import scan.scan_manager as sm
        cal = build_calibration(2048, source, "ADC1000-USB", _reader(ADC_EEPROM))
        with patch.object(sm, "get_spectrometer_reader", return_value=_FakeSpectrometer(cal)):
            return sm.ScanManager(self._config())

    def test_invalid_calibration_warns_and_is_recorded(self):
        m = self._manager("eeprom_slots_1_4")
        self.assertIsNotNone(m.calibration_warning)
        self.assertIn("INVALID", m.calibration_warning)
        import h5py
        with h5py.File(m.store.path, "r") as f:
            rec = json.loads(f.attrs["wavelength_calibration"])
        self.assertFalse(rec["valid"])
        with open(m.logger.log_path) as f:
            self.assertIn("WARNING: OES wavelength calibration is INVALID", f.read())

    def test_valid_calibration_no_warning_and_recorded(self):
        m = self._manager("auto")
        self.assertIsNone(m.calibration_warning)
        import h5py
        with h5py.File(m.store.path, "r") as f:
            rec = json.loads(f.attrs["wavelength_calibration"])
            self.assertAlmostEqual(float(f["wavelength_nm"][0]), 201.5547, places=3)
        self.assertTrue(rec["valid"])
        self.assertEqual(rec["source"], "eeprom_slots_2_5")
        m.logger.write_metadata({"oes_wavelength_calibration": m.oes_calibration})
        with open(m.logger.metadata_path) as f:
            meta = yaml.safe_load(f)
        self.assertTrue(meta["oes_wavelength_calibration"]["valid"])


@unittest.skipUnless(_HAVE_H5, "h5py/xarray not installed")
class FixToolTests(unittest.TestCase):
    """tools/fix_wavelength_axis.py on a synthetic scan saved with the broken axis."""

    def setUp(self):
        import h5py
        self.d = tempfile.mkdtemp()
        n = 2048
        self.broken = evaluate([1.0] + TRUE[:3], n)
        self.true = evaluate(TRUE, n)
        rng = np.random.default_rng(0)
        _os.makedirs(_os.path.join(self.d, "spectra"))
        self.spectra = {}
        for pid in range(3):
            inten = 150 + rng.normal(0, 1, n)
            inten += 500 * np.exp(-0.5 * ((self.true - 656.28) / 1.0) ** 2)  # H-alpha
            self.spectra[pid] = inten
            with open(_os.path.join(self.d, "spectra", f"point_{pid:05d}.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["wavelength_nm", "intensity"])
                w.writerows(zip(self.broken, inten))
        with open(_os.path.join(self.d, "scan_summary.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["point_id", "x_mm", "feature_H_alpha", "feature_CH"])
            for pid in range(3):
                w.writerow([pid, pid, "nan", "nan"])
        with open(_os.path.join(self.d, "metadata.yaml"), "w") as f:
            yaml.safe_dump({"config": {"oes": {"features": {"H_alpha": 656.3, "CH": 431.0},
                                               "feature_window_nm": 1.0}}}, f)
        with h5py.File(_os.path.join(self.d, "oes.h5"), "w") as f:
            f.create_dataset("wavelength_nm", data=self.broken.astype("float32"))

    def tearDown(self):
        shutil.rmtree(self.d, ignore_errors=True)

    def _run(self, *args):
        return subprocess.run(
            [_sys.executable, _os.path.join(_REPO_ROOT, "tools", "fix_wavelength_axis.py"),
             self.d, *args], capture_output=True, text=True)

    def test_without_coeffs_reports_inferred_values_and_writes_nothing(self):
        r = self._run()
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("201.5547", r.stdout)
        self.assertFalse(_os.path.exists(_os.path.join(self.d, "spectra_original_axis")))

    def test_fix_rewrites_axis_recomputes_features_and_keeps_originals(self):
        import h5py
        r = self._run("--coeffs", *map(str, TRUE))
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        w = np.loadtxt(_os.path.join(self.d, "spectra", "point_00001.csv"),
                       delimiter=",", skiprows=1)
        np.testing.assert_allclose(w[:, 0], self.true, rtol=1e-9)
        np.testing.assert_allclose(w[:, 1], self.spectra[1], rtol=1e-9)
        orig = np.loadtxt(_os.path.join(self.d, "spectra_original_axis", "point_00001.csv"),
                          delimiter=",", skiprows=1)
        np.testing.assert_allclose(orig[:, 0], self.broken, rtol=1e-9)
        with h5py.File(_os.path.join(self.d, "oes.h5"), "r") as f:
            np.testing.assert_allclose(f["wavelength_nm"][:], self.true, rtol=1e-6)
            np.testing.assert_allclose(f["wavelength_nm_original_axis"][:], self.broken, rtol=1e-6)
            self.assertTrue(json.loads(f.attrs["wavelength_calibration"])["valid"])
        with open(_os.path.join(self.d, "scan_summary.csv")) as f:
            rows = list(csv.DictReader(f))
        ha, ch = float(rows[0]["feature_H_alpha"]), float(rows[0]["feature_CH"])
        self.assertGreater(ha, ch + 300, (ha, ch))  # H-alpha peak now falls in its window
        self.assertTrue(_os.path.exists(_os.path.join(self.d, "scan_summary_original_axis.csv")))
        with open(_os.path.join(self.d, "metadata.yaml")) as f:
            self.assertIn("wavelength_axis_corrected", yaml.safe_load(f))
        # second run is refused -- axis already valid
        r2 = self._run("--coeffs", *map(str, TRUE))
        self.assertNotEqual(r2.returncode, 0)
        self.assertIn("already", r2.stdout + r2.stderr)


if __name__ == "__main__":
    unittest.main()
