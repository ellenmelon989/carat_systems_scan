"""
2026-10-06 on-site: the PAC reports the pyrometer's LOW / no-signal state as
a 300.0 C placeholder (the Williamson's real minimum is 600 C), and the
precision scan was recording it as a valid temperature (ir_error=False).
While fixing that, found IRReader.read_averaged() had no deadline: a dwell
window with zero valid reads (PyroTempInvalid stuck on, PAC unreachable)
looped forever.

Covers:
  - RestApiIRReader tags < min_valid_c as below_range, value left as reported
    (the adaptive edge detector uses "reads low" as "off-wafer").
  - read_averaged excludes LOW reads and always returns (bounded).
  - ScanManager._read_ir_with_retry turns a no-valid-reads point into
    ir_error=True with an ir_error_detail saying why.
  - A full precision scan against a cold (all-LOW) pyrometer completes,
    marks the point LOW, and still writes maps without crashing.
"""
import csv
import glob
import os as _os
import sys as _sys
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock, patch

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)

import yaml

from readers.ir_reader_base import IR_LOW_PREFIX, IRReader, IRReading
from readers.rest_ir_reader import RestApiIRReader


def _run_bounded(fn, timeout_s):
    """Run fn in a thread; fail (instead of hanging the suite) if it doesn't
    return in time -- the pre-fix read_averaged() never returned."""
    out = {}

    def target():
        out["result"] = fn()
    t = threading.Thread(target=target, daemon=True)
    t0 = time.time()
    t.start()
    t.join(timeout_s)
    if t.is_alive():
        raise AssertionError(f"did not return within {timeout_s}s (unbounded loop)")
    return out["result"], time.time() - t0


class _ScriptedReader(IRReader):
    def __init__(self, readings):
        self.readings = list(readings)
        self.i = 0

    def read(self):
        r = self.readings[min(self.i, len(self.readings) - 1)]
        self.i += 1
        return r


def _r(value, below=False, error=None, stale=False):
    now = time.time()
    return IRReading(value_c=value, emissivity=0.5, pac_timestamp=now, read_time=now,
                     stale=stale, error=error, below_range=below)


class RestReaderLowTaggingTests(unittest.TestCase):
    def _reader(self, temp, **kw):
        rd = RestApiIRReader("1.2.3.4", "k", "v", temp_tag_name="iai_PYRO_TEMP", **kw)
        rd._get_int32 = MagicMock(return_value=0)        # PyroTempInvalid clear
        rd._get_float = MagicMock(side_effect=lambda name: temp if name == "iai_PYRO_TEMP" else 0.5)
        return rd

    def test_placeholder_300_is_tagged_low_and_value_kept(self):
        r = self._reader(300.0).read()
        self.assertTrue(r.below_range)
        self.assertEqual(r.value_c, 300.0)
        self.assertIsNone(r.error)

    def test_real_temperature_not_tagged(self):
        self.assertFalse(self._reader(850.0).read().below_range)

    def test_threshold_can_be_disabled(self):
        self.assertFalse(self._reader(300.0, min_valid_c=None).read().below_range)

    def test_factory_defaults_to_600_without_config_key(self):
        from readers.ir_reader_base import get_ir_reader
        cfg = {"ir": {"source": "pac", "pac": {"ip": "1.2.3.4", "api_key_id": "k",
                                                "api_key_value": "v",
                                                "temp_tag_name": "iai_PYRO_TEMP"}}}
        self.assertEqual(get_ir_reader(cfg).min_valid_c, 600.0)


class ReadAveragedBoundedTests(unittest.TestCase):
    def test_all_low_returns_nan_at_end_of_dwell(self):
        rd = _ScriptedReader([_r(300.0, below=True)])
        (mean, std, last), dt = _run_bounded(
            lambda: rd.read_averaged(0.2, poll_interval_s=0.01), 3.0)
        self.assertNotEqual(mean, mean)  # NaN
        self.assertTrue(last.below_range)
        self.assertLess(dt, 0.6)

    def test_all_invalid_flag_returns_instead_of_hanging(self):
        rd = _ScriptedReader([_r(float("nan"), stale=True, error="PyroTempInvalid flag set")])
        (mean, _, last), _dt = _run_bounded(
            lambda: rd.read_averaged(0.2, poll_interval_s=0.01), 3.0)
        self.assertNotEqual(mean, mean)
        self.assertEqual(last.error, "PyroTempInvalid flag set")

    def test_low_reads_excluded_from_mean(self):
        rd = _ScriptedReader([_r(800.0), _r(300.0, below=True), _r(820.0),
                              _r(300.0, below=True), _r(810.0)])
        (mean, _, _), _dt = _run_bounded(
            lambda: rd.read_averaged(0.05, poll_interval_s=0.02), 3.0)
        self.assertGreaterEqual(mean, 800.0)

    def test_too_few_valid_reads_stops_at_deadline(self):
        # one valid read, then nothing valid: min_reads=3 can never be met
        rd = _ScriptedReader([_r(800.0)] + [_r(300.0, below=True)] * 10_000)
        (mean, _, _), dt = _run_bounded(
            lambda: rd.read_averaged(0.2, poll_interval_s=0.01), 5.0)
        self.assertEqual(mean, 800.0)
        self.assertLess(dt, 2.6)  # dwell 0.2 + grace max(2.0, 0.2)


class _ColdPyrometer(IRReader):
    """PAC + pyrometer on a cold reactor: every poll is the 300 C LOW placeholder."""
    def read(self):
        return _r(300.0, below=True)


class PrecisionScanColdReactorTests(unittest.TestCase):
    def _config(self):
        with open(_os.path.join(_REPO_ROOT, "config.example.yaml"), encoding="utf-8-sig") as f:
            cfg = yaml.safe_load(f)
        cfg["motion"]["controller"] = None
        cfg["motion"]["motion_enabled"] = True
        cfg["motion"]["soft_limits"] = {"x_min_mm": -100, "x_max_mm": 100,
                                        "y_min_mm": -100, "y_max_mm": 100}
        cfg["ir"]["source"] = "mock"
        cfg["oes"]["enabled"] = False
        cfg["scan"]["dwell_time_s"] = 2.0
        cfg["scan"]["reference_point"]["enabled"] = False
        cfg["scan"]["grid"].update(mode="grid", x_range_mm=[0, 0], y_range_mm=[0, 0],
                                   step_size_mm=5, wafer_center_mm=[0, 0],
                                   wafer_radius_mm=None)
        cfg["output"]["base_dir"] = tempfile.mkdtemp()
        cfg["output"].pop("oes_hdf5", None)
        return cfg

    def test_cold_scan_completes_and_marks_point_low(self):
        try:
            import h5py  # noqa: F401
        except ImportError:
            self.skipTest("h5py not installed")
        import scan.scan_manager as sm
        cfg = self._config()
        with patch.object(sm, "get_ir_reader", return_value=_ColdPyrometer()):
            m = sm.ScanManager(cfg)
            _run_bounded(m.run, 30.0)
        csv_path = glob.glob(_os.path.join(cfg["output"]["base_dir"], "**", "*.csv"),
                             recursive=True)[0]
        with open(csv_path, newline="") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["ir_error"], "True")
        self.assertTrue(rows[0]["ir_error_detail"].startswith(IR_LOW_PREFIX))
        self.assertEqual(rows[0]["ir_temp_c"], "nan")


if __name__ == "__main__":
    unittest.main()
