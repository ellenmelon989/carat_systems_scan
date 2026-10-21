"""
Batch 2 (2026-10-19): OES background capture / selection / subtraction.

RG's requirements: capture + save true-dark and plasma-off backgrounds
(room lights on/off), averaging N readings with settings and timestamps
recorded; choose the subtraction at scan start; keep both unsubtracted and
subtracted spectra; flag (warn + record) a background whose acquisition
settings differ from the scan's.
"""
import csv
import json
import os as _os
import sys as _sys
import tempfile
import time
import unittest
from unittest.mock import patch

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)

import numpy as np
import yaml

try:
    import h5py
    import xarray  # noqa: F401
    _HAVE_H5 = True
except ImportError:
    _HAVE_H5 = False

from readers.mock_spectrometer_reader import MockSpectrometerReader
from readers.spectrometer_reader_base import SpectrumReading, boxcar_smooth
from scan import background as bgmod


class ConstReader(MockSpectrometerReader):
    """Mock whose raw reading is a fixed level + index, for exact checks."""
    def __init__(self, level=150.0, n=64, **kw):
        super().__init__(num_pixels=n, **kw)
        self.level, self.k = level, 0

    def read(self):
        self.k += 1
        raw = np.full(len(self._wavelengths), self.level) + (self.k % 2)  # 150/151
        t = time.time()
        proc = boxcar_smooth(raw, self.boxcar_width) if self.boxcar_width else raw
        return SpectrumReading(self._wavelengths, proc, self.integration_time_us,
                               self.num_averages, self.boxcar_width, False, t,
                               raw_intensities=raw, acq_start=t, acq_end=t + 1e-3)


@unittest.skipUnless(_HAVE_H5, "h5py not installed")
class CaptureSaveLoadTests(unittest.TestCase):
    def test_capture_averages_raw_and_records_everything(self):
        r = ConstReader(integration_time_us=20000, num_averages=3, boxcar_width=2)
        bg = bgmod.capture(r, 4, "plasma_off", "off", "test")
        np.testing.assert_allclose(bg.mean, 150.5)       # mean of 151,150,151,150
        self.assertEqual(bg.readings.shape, (4, 64))
        rec = bg.record
        self.assertEqual(rec["kind"], "plasma_off")
        self.assertEqual(rec["room_lights"], "off")
        self.assertEqual(rec["n_readings"], 4)
        self.assertEqual(rec["acquisition"]["integration_time_us"], 20000)
        self.assertEqual(rec["acquisition"]["num_averages"], 3)
        self.assertEqual(len(bg.reading_t_start), 4)
        self.assertLessEqual(rec["t_start"], rec["t_end"])

        d = tempfile.mkdtemp()
        p1 = bgmod.save(bg, d)
        p2 = bgmod.save(bg, d)                   # same second -> never overwrite
        self.assertNotEqual(p1, p2)
        self.assertTrue(_os.path.exists(p1[:-3] + ".csv"))
        back = bgmod.load(p1, with_readings=True)
        np.testing.assert_allclose(back.mean, bg.mean)
        self.assertEqual(back.record["notes"], "test")
        self.assertEqual(back.readings.shape, (4, 64))
        rows = bgmod.list_saved(d)
        self.assertEqual(len(rows), 2)
        self.assertIn("plasma-off", rows[0]["label"])
        self.assertIn("lights off", rows[0]["label"])

    def test_failed_reading_aborts_capture(self):
        r = ConstReader()
        r.read = lambda: SpectrumReading(None, None, 1, 1, 0, False, 0.0, error="usb gone")
        with self.assertRaises(IOError):
            bgmod.capture(r, 3, "true_dark")

    def test_bad_inputs(self):
        r = ConstReader()
        for kw in ({"kind": "dark"}, {"kind": "true_dark", "room_lights": "dim"}):
            with self.assertRaises(ValueError):
                bgmod.capture(r, 2, **kw)
        with self.assertRaises(ValueError):
            bgmod.validate_n_readings(0)


class CompareTests(unittest.TestCase):
    def _bg(self, **acq):
        a = {"integration_time_us": 20000, "num_averages": 3, "boxcar_width": 2,
             "serial": "ADUD7590"}
        a.update(acq)
        n = 64
        return bgmod.Background(np.linspace(200, 850, n), np.zeros(n), np.zeros(n),
                                {"acquisition": a, "t_start": time.time()})

    def test_matching(self):
        c = bgmod.compare(self._bg(), {"integration_time_us": 20000, "num_averages": 3,
                                       "boxcar_width": 2}, np.linspace(200, 850, 64),
                          {"serial": "ADUD7590"})
        self.assertEqual(c["mismatches"], [])
        self.assertEqual(c["errors"], [])

    def test_every_setting_difference_is_flagged(self):
        c = bgmod.compare(self._bg(), {"integration_time_us": 500000, "num_averages": 1,
                                       "boxcar_width": 0}, np.linspace(200, 850, 64),
                          {"serial": "OTHER"})
        joined = " ".join(c["mismatches"])
        for key in ("integration_time_us", "num_averages", "boxcar_width", "serial"):
            self.assertIn(key, joined)

    def test_pixel_count_is_an_error(self):
        c = bgmod.compare(self._bg(), {}, np.linspace(200, 850, 100))
        self.assertTrue(c["errors"])


def _config(bg_file, ir_enabled=False):
    with open(_os.path.join(_REPO_ROOT, "config.example.yaml"), encoding="utf-8-sig") as f:
        cfg = yaml.safe_load(f)
    cfg["motion"]["controller"] = None
    cfg["motion"]["motion_enabled"] = True
    cfg["motion"]["soft_limits"] = {"x_min_mm": -100, "x_max_mm": 100,
                                    "y_min_mm": -100, "y_max_mm": 100}
    cfg["ir"]["source"] = "mock"
    cfg["ir"]["enabled"] = ir_enabled
    cfg["oes"]["enabled"] = True
    cfg["oes"]["integration_time_us"] = 20000
    cfg["oes"]["num_averages"] = 3
    cfg["oes"]["boxcar_width"] = 1
    cfg["oes"]["background_file"] = bg_file
    cfg["scan"]["dwell_time_s"] = 2.0
    cfg["scan"]["settle_time_s"] = 0.0
    cfg["scan"]["reference_point"]["enabled"] = False
    cfg["scan"]["grid"].update({"mode": "grid", "x_range_mm": [0, 2], "y_range_mm": [0, 0],
                                "step_size_mm": 2.0, "wafer_radius_mm": None})
    cfg["output"]["base_dir"] = tempfile.mkdtemp()
    cfg["output"].pop("oes_hdf5", None)
    return cfg


@unittest.skipUnless(_HAVE_H5, "h5py/xarray not installed")
class ScanSubtractionTests(unittest.TestCase):
    def _save_bg(self, **settings):
        s = {"integration_time_us": 20000, "num_averages": 3, "boxcar_width": 1}
        s.update(settings)
        r = ConstReader(level=100.0, n=2048)
        r.apply_settings(**s)
        return bgmod.save(bgmod.capture(r, 4, "true_dark", "n/a"), tempfile.mkdtemp())

    def _run(self, cfg):
        import scan.scan_manager as sm
        spec = MockSpectrometerReader()
        with patch.object(sm, "get_spectrometer_reader", return_value=spec):
            m = sm.ScanManager(cfg)
            with patch.object(sm.time, "sleep"):
                self.assertEqual(m.run(), "completed")
            m.close()
        return m

    def test_both_spectra_saved_and_subtraction_exact(self):
        path = self._save_bg()
        cfg = _config(path)
        m = self._run(cfg)
        self.assertEqual(m.startup_warnings, [])
        bg = bgmod.load(path)
        with h5py.File(m.store.path, "r") as f:
            raw = f["intensity_raw"][0, 0, 0].astype(float)
            raw_sub = f["intensity_raw_bgsub"][0, 0, 0].astype(float)
            proc_sub = f["intensity_bgsub"][0, 0, 0].astype(float)
            np.testing.assert_allclose(raw_sub, raw - bg.mean, atol=1e-2)
            np.testing.assert_allclose(proc_sub, boxcar_smooth(raw - bg.mean, 1), atol=1e-2)
            np.testing.assert_allclose(f["background/mean"][:], bg.mean)
            rec = json.loads(f.attrs["oes_background"])
            self.assertTrue(rec["applied"])
            self.assertFalse(rec["settings_mismatch"])
        meta = yaml.safe_load(open(_os.path.join(cfg["output"]["base_dir"], "metadata.yaml")))
        self.assertTrue(meta["oes_background"]["applied"])
        self.assertEqual(meta["oes_background"]["kind"], "true_dark")
        with open(_os.path.join(cfg["output"]["base_dir"], "spectra", "point_00000.csv")) as f:
            self.assertEqual(next(csv.reader(f)), ["wavelength_nm", "intensity", "intensity_raw",
                                                  "intensity_bgsub", "intensity_raw_bgsub"])
        with open(_os.path.join(cfg["output"]["base_dir"], "scan_summary.csv")) as f:
            self.assertEqual(next(csv.DictReader(f))["oes_bg_subtracted"], "True")

    def test_mismatch_warns_and_is_recorded(self):
        path = self._save_bg(integration_time_us=500000)
        cfg = _config(path)
        m = self._run(cfg)
        self.assertEqual(len(m.startup_warnings), 1)
        self.assertIn("integration_time_us", m.startup_warnings[0])
        meta = yaml.safe_load(open(_os.path.join(cfg["output"]["base_dir"], "metadata.yaml")))
        self.assertTrue(meta["oes_background"]["settings_mismatch"])
        self.assertTrue(any("integration_time_us" in x
                            for x in meta["oes_background"]["mismatches"]))

    def test_no_background_records_not_applied(self):
        cfg = _config(None)
        m = self._run(cfg)
        meta = yaml.safe_load(open(_os.path.join(cfg["output"]["base_dir"], "metadata.yaml")))
        self.assertEqual(meta["oes_background"], {"applied": False})
        with h5py.File(m.store.path, "r") as f:
            self.assertNotIn("intensity_bgsub", f)

    def test_missing_or_wrong_size_background_fails_before_motion(self):
        import scan.scan_manager as sm
        with patch.object(sm, "get_spectrometer_reader", return_value=MockSpectrometerReader()):
            with self.assertRaises(RuntimeError):
                sm.ScanManager(_config("/nonexistent/bg.h5"))
        r = ConstReader(n=100)
        small = bgmod.save(bgmod.capture(r, 2, "true_dark"), tempfile.mkdtemp())
        with patch.object(sm, "get_spectrometer_reader", return_value=MockSpectrometerReader()):
            with self.assertRaises(RuntimeError) as ctx:
                sm.ScanManager(_config(small))
        self.assertIn("pixels", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
