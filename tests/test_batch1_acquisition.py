"""
Batch 1 (2026-10-14) regression tests -- on-site requests from RG:

  * OES dwell is enforced per point whether or not IR is enabled (an
    OES-only scan with dwell 2 s used to move on after settle_time only).
  * Every scan closes the motion, spectrometer and IR connections when it
    ends (completed, aborted, crashed, or failed init) -- the COM7
    "access denied" path. A handed-off motion is closed only when the
    caller passes ownership.
  * Acquisition settings (integration, averages, boxcar) from config are
    applied to the device and recorded in metadata.yaml and oes.h5.
  * Unsmoothed detector values are saved (intensity_raw) alongside the
    processed, boxcar-smoothed copy.
  * Saturation is judged per frame, not on the average.
  * The first frame after a settings change is discarded.
  * Each point records its actual IR dwell and OES acquisition windows.
"""
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
from readers.spectrometer_reader_base import boxcar_smooth
from scan.scan_params import (
    validate_boxcar_width, validate_integration_time_us, validate_num_averages,
)


def _config(ir_enabled=True, oes_enabled=True, dwell=2.0):
    with open(_os.path.join(_REPO_ROOT, "config.example.yaml"), encoding="utf-8-sig") as f:
        cfg = yaml.safe_load(f)
    cfg["motion"]["controller"] = None
    cfg["motion"]["motion_enabled"] = True
    cfg["motion"]["soft_limits"] = {"x_min_mm": -100, "x_max_mm": 100,
                                    "y_min_mm": -100, "y_max_mm": 100}
    cfg["ir"]["source"] = "mock"
    cfg["ir"]["enabled"] = ir_enabled
    cfg["oes"]["enabled"] = oes_enabled
    cfg["oes"]["source"] = "mock"
    cfg["oes"]["integration_time_us"] = 50_000
    cfg["oes"]["num_averages"] = 2
    cfg["oes"]["boxcar_width"] = 1
    cfg["scan"]["dwell_time_s"] = dwell
    cfg["scan"]["settle_time_s"] = 0.0
    cfg["scan"]["passes"] = 1
    cfg["scan"]["reference_point"]["enabled"] = False
    cfg["scan"]["grid"].update({
        "mode": "grid", "x_range_mm": [0, 2], "y_range_mm": [0, 0],
        "step_size_mm": 2.0, "wafer_radius_mm": None,
    })
    cfg["output"]["base_dir"] = tempfile.mkdtemp()
    cfg["output"].pop("oes_hdf5", None)
    return cfg


class _Closable:
    def __init__(self):
        self.closed = 0

    def close(self):
        self.closed += 1


class ValidatorTests(unittest.TestCase):
    def test_ranges(self):
        self.assertEqual(validate_integration_time_us("20000"), 20000)
        self.assertEqual(validate_num_averages(3.0), 3)
        self.assertEqual(validate_boxcar_width(0), 0)
        for bad in (100, 61_000_000, "abc", 2.5):
            with self.assertRaises(ValueError):
                validate_integration_time_us(bad)
        with self.assertRaises(ValueError):
            validate_num_averages(0)
        with self.assertRaises(ValueError):
            validate_boxcar_width(-1)


class ReaderTests(unittest.TestCase):
    def test_raw_is_unsmoothed_and_processed_is_smoothed(self):
        r = MockSpectrometerReader(num_averages=2, boxcar_width=2)
        reading = r.read()
        np.testing.assert_allclose(reading.intensities,
                                   boxcar_smooth(reading.raw_intensities, 2))
        self.assertFalse(np.allclose(reading.intensities, reading.raw_intensities))
        self.assertLessEqual(reading.acq_start, reading.acq_end)

    def test_boxcar_zero_means_identical(self):
        reading = MockSpectrometerReader(boxcar_width=0).read()
        np.testing.assert_array_equal(reading.intensities, reading.raw_intensities)

    def test_pyseabreeze_discards_stale_frame_and_flags_single_saturated_frame(self):
        from readers.pyseabreeze_spectrometer_reader import PySeabreezeSpectrometerReader

        class FakeSpec:
            def __init__(self):
                self.frames = []
                self.calls = 0

            def intensities(self):
                self.calls += 1
                return self.frames.pop(0)

            def integration_time_micros(self, _):
                pass

        r = PySeabreezeSpectrometerReader.__new__(PySeabreezeSpectrometerReader)
        r.spec = FakeSpec()
        r._wavelengths = np.arange(4.0)
        r._max_intensity = 4095.0
        r.integration_time_us, r.num_averages, r.boxcar_width = 20000, 2, 0
        r._init_error = None
        r.apply_settings(30000, 2, 0)            # -> next read discards one frame
        stale = np.full(4, 9999.0)
        r.spec.frames = [stale, np.array([100, 4095, 100, 100.0]),
                         np.array([100, 100, 100, 100.0])]
        reading = r.read()
        self.assertEqual(r.spec.calls, 3)        # 1 discarded + 2 averaged
        self.assertTrue(reading.saturated, "one saturated frame must flag the point")
        self.assertLess(reading.intensities.max(), 4095)   # average hides it
        self.assertEqual(reading.integration_time_us, 30000)
        # no settings change -> no discard on the next read
        r.spec.frames = [np.zeros(4), np.zeros(4)]
        r.read()
        self.assertEqual(r.spec.calls, 5)


@unittest.skipUnless(_HAVE_H5, "h5py/xarray not installed")
class ScanTests(unittest.TestCase):
    def _run(self, cfg, motion=None, owns_motion=False, ir=None):
        import scan.scan_manager as sm
        spec = MockSpectrometerReader(integration_time_us=1, num_averages=1, boxcar_width=0)
        spec.close = lambda: setattr(spec, "closed", True)
        spec.closed = False
        patches = [patch.object(sm, "get_spectrometer_reader", return_value=spec)]
        if ir is not None:
            patches.append(patch.object(sm, "get_ir_reader", return_value=ir))
        for p in patches:
            p.start()
        try:
            m = sm.ScanManager(cfg, motion=motion, owns_motion=owns_motion)
            sleeps = []
            real_sleep = time.sleep
            with patch.object(sm.time, "sleep", side_effect=lambda s: sleeps.append(s)):
                status = m.run()
            m.close()
        finally:
            for p in patches:
                p.stop()
        return m, spec, status, sleeps

    def test_oes_only_scan_waits_for_dwell(self):
        m, _, status, sleeps = self._run(_config(ir_enabled=False, dwell=3.0))
        self.assertEqual(status, "completed")
        dwell_sleeps = [s for s in sleeps if s > 1.0]
        self.assertEqual(len(dwell_sleeps), 2, sleeps)       # one per point
        for s in dwell_sleeps:
            self.assertAlmostEqual(s, 3.0, delta=0.3)

    def test_settings_applied_and_recorded(self):
        cfg = _config(ir_enabled=False)
        m, spec, _, _ = self._run(cfg)
        self.assertEqual((spec.integration_time_us, spec.num_averages, spec.boxcar_width),
                         (50_000, 2, 1))
        with open(_os.path.join(cfg["output"]["base_dir"], "metadata.yaml")) as f:
            meta = yaml.safe_load(f)
        acq = meta["oes_acquisition"]
        self.assertEqual(acq["integration_time_us"], 50_000)
        self.assertEqual(acq["num_averages"], 2)
        self.assertEqual(acq["boxcar_width"], 1)
        with h5py.File(m.store.path, "r") as f:
            self.assertEqual(json.loads(f.attrs["oes_acquisition"])["num_averages"], 2)

    def test_raw_and_timing_saved(self):
        cfg = _config(ir_enabled=False)
        m, _, _, _ = self._run(cfg)
        with h5py.File(m.store.path, "r") as f:
            raw = f["intensity_raw"][0, 0, 0]
            proc = f["intensity"][0, 0, 0]
            np.testing.assert_allclose(proc, boxcar_smooth(raw.astype(float), 1), rtol=1e-4)
            self.assertTrue(np.isfinite(f["t_oes_start"][0, 0, 0]))
            self.assertTrue(np.isnan(f["t_ir_start"][0, 0, 0]))       # IR off
        import csv
        spectra = _os.path.join(cfg["output"]["base_dir"], "spectra", "point_00000.csv")
        with open(spectra) as f:
            self.assertEqual(next(csv.reader(f)), ["wavelength_nm", "intensity", "intensity_raw"])
        with open(_os.path.join(cfg["output"]["base_dir"], "scan_summary.csv")) as f:
            rows = list(csv.DictReader(f))
        for k in ("t_ir_start", "t_ir_end", "t_oes_start", "t_oes_end"):
            self.assertIn(k, rows[0])
        self.assertLessEqual(float(rows[0]["t_oes_start"]), float(rows[0]["t_oes_end"]))
        from scan.oes_store import OESStore
        ds = OESStore.load(m.store.path)
        self.assertIn("intensity_raw", ds)
        self.assertIn("t_oes_start", ds)

    def test_ir_timing_recorded_when_ir_on(self):
        cfg = _config(ir_enabled=True)
        m, _, _, _ = self._run(cfg)
        with h5py.File(m.store.path, "r") as f:
            a, b = f["t_ir_start"][0, 0, 0], f["t_ir_end"][0, 0, 0]
            self.assertTrue(np.isfinite(a) and b >= a)
            self.assertGreaterEqual(f["t_oes_start"][0, 0, 0], b - 1e-6)

    def test_scan_closes_all_connections_when_owned(self):
        from motion.motion_controller import get_motion_controller
        cfg = _config(ir_enabled=False)
        motion = get_motion_controller(cfg)
        closed = []
        motion.close = lambda: closed.append("motion")
        ir = _Closable()
        ir.read_averaged = lambda *_a, **_k: (float("nan"), float("nan"), None)
        m, spec, _, _ = self._run(cfg, motion=motion, owns_motion=True, ir=ir)
        self.assertEqual(closed, ["motion"])
        self.assertTrue(spec.closed)
        self.assertEqual(ir.closed, 1)
        m.close()                                    # idempotent
        self.assertEqual(ir.closed, 1)

    def test_handed_off_motion_not_closed_without_ownership(self):
        from motion.motion_controller import get_motion_controller
        cfg = _config(ir_enabled=False)
        motion = get_motion_controller(cfg)
        closed = []
        motion.close = lambda: closed.append("motion")
        _, spec, _, _ = self._run(cfg, motion=motion, owns_motion=False)
        self.assertEqual(closed, [])
        self.assertTrue(spec.closed)

    def test_failed_storage_init_releases_readers(self):
        import scan.scan_manager as sm
        cfg = _config(ir_enabled=False)
        spec = MockSpectrometerReader()
        spec.closed = False
        spec.close = lambda: setattr(spec, "closed", True)
        with patch.object(sm, "get_spectrometer_reader", return_value=spec), \
             patch.object(sm, "OESStore", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                sm.ScanManager(cfg)
        self.assertTrue(spec.closed)


class WorkerTests(unittest.TestCase):
    def test_worker_closes_manager_even_if_run_raises(self):
        import queue
        import threading
        import gui.scan_worker as sw

        class FakeManager:
            calibration_warning = None
            closed = False

            def __init__(self, *a, **k):
                FakeManager.kwargs = k

            def run(self, **_):
                raise RuntimeError("boom")

            def close(self):
                FakeManager.closed = True

        q = queue.Queue()
        with patch.object(sw, "ScanManager", FakeManager):
            sw.run_scan({}, q, threading.Event(), motion=object())
        self.assertTrue(FakeManager.closed)
        self.assertTrue(FakeManager.kwargs["owns_motion"])
        self.assertEqual(q.get_nowait()[0], "error")


if __name__ == "__main__":
    unittest.main()
