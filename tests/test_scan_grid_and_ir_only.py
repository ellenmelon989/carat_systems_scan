"""
Regression tests for two issues found on-site 2026-10-02 (first MR-15-30
trial scan):

1. Circular grid dropped a boundary point to float noise: diameter 10 /
   step 5 at saved center (-6.219, 1.9891) queued 4 points, not 5 -- the
   -X edge point came out at d^2 - r^2 = +1.07e-14 and in_radius() masked
   it off. Fixed with a 1e-6 mm tolerance in scan_params.in_radius().

2. IR-only scan (OES unchecked) with no spectrometer connected died on the
   first point with OESStore's "wavelengths must be provided on the first
   write_point() call": the store was never initialized because
   spectrometer.wavelengths was None and OES-skipped points pass
   wavelengths=None. Fixed in ScanManager.__init__: OES disabled -> a
   zero-length spectral axis; OES enabled with no spectrometer -> fail at
   init with the real cause, before any motion.
"""
import copy
import os as _os
import sys as _sys
import tempfile
import unittest

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)

import numpy as np
import yaml
from unittest.mock import patch

from scan.scan_params import in_radius

try:
    import h5py  # noqa: F401
    import xarray  # noqa: F401
    _HAVE_H5 = True
except ImportError:
    _HAVE_H5 = False

ONSITE_CENTER = (-6.219, 1.9891)


def _grid_cfg(center, radius, step):
    cx, cy = center
    return {"grid": {
        "x_range_mm": [cx - radius, cx + radius],
        "y_range_mm": [cy - radius, cy + radius],
        "step_size_mm": step,
        "wafer_center_mm": list(center),
        "wafer_radius_mm": radius,
    }}


class CircularGridBoundaryTests(unittest.TestCase):
    def test_onsite_case_gives_five_points(self):
        from scan.scan_manager import generate_grid
        pts, _, _ = generate_grid(_grid_cfg(ONSITE_CENTER, 5.0, 5))
        xy = sorted((round(x, 4), round(y, 4)) for _, _, x, y in pts)
        self.assertEqual(xy, sorted([
            (-11.219, 1.9891), (-6.219, 1.9891), (-1.219, 1.9891),
            (-6.219, -3.0109), (-6.219, 6.9891),
        ]))

    def test_boundary_symmetric_for_many_centers(self):
        # Center + 4 edge points must always survive, whatever float noise
        # the center's digits produce.
        from scan.scan_manager import generate_grid
        rng = np.random.default_rng(0)
        for _ in range(200):
            c = tuple(np.round(rng.uniform(-20, 20, 2), 4))
            pts, _, _ = generate_grid(_grid_cfg(c, 5.0, 5))
            self.assertEqual(len(pts), 5, f"center {c}")

    def test_point_clearly_outside_still_excluded(self):
        self.assertFalse(in_radius(5.001, 0.0, (0.0, 0.0), 5.0))
        self.assertTrue(in_radius(5.0, 0.0, (0.0, 0.0), 5.0))


class _DeadSpectrometer:
    """A pyseabreeze reader whose connect failed: wavelengths None."""
    _wavelengths = None
    _init_error = "No spectrometer found"
    wavelengths = None

    def read(self):
        raise IOError("not connected")

    def set_integration_time(self, *_):
        pass

    def close(self):
        pass


@unittest.skipUnless(_HAVE_H5, "h5py/xarray not installed")
class IrOnlyWithoutSpectrometerTests(unittest.TestCase):
    def _config(self, oes_enabled):
        with open(_os.path.join(_REPO_ROOT, "config.example.yaml"), encoding="utf-8-sig") as f:
            cfg = yaml.safe_load(f)
        cfg["motion"]["controller"] = None          # mock motion
        cfg["motion"]["motion_enabled"] = True
        cfg["motion"]["soft_limits"] = {"x_min_mm": -100, "x_max_mm": 100,
                                        "y_min_mm": -100, "y_max_mm": 100}
        cfg["ir"]["source"] = "mock"
        cfg["oes"]["enabled"] = oes_enabled
        cfg["scan"]["grid"].update(_grid_cfg(ONSITE_CENTER, 5.0, 5)["grid"], mode="grid")
        cfg["output"]["base_dir"] = tempfile.mkdtemp()
        cfg["output"].pop("oes_hdf5", None)
        return cfg

    def test_oes_disabled_store_initializes_with_empty_spectral_axis(self):
        import scan.scan_manager as sm
        with patch.object(sm, "get_spectrometer_reader", return_value=_DeadSpectrometer()):
            m = sm.ScanManager(self._config(oes_enabled=False))
        self.assertTrue(m.store._initialized,
                        "store must be ready before the first point, or the first "
                        "IR-only write_point() raises")
        # First point exactly as an OES-skipped point writes it.
        m.store.write_point(0, 1, wavelengths=None, intensities=None, ir_temp_c=300.0)
        from scan.oes_store import OESStore
        ds = OESStore.load(m.store.path)
        self.assertEqual(ds.intensity.shape[-1], 0)
        self.assertEqual(float(ds.ir_temp_c.isel(x_mm=0, y_mm=1, pass_id=0)), 300.0)

    def test_oes_enabled_without_spectrometer_fails_at_init(self):
        import scan.scan_manager as sm
        with patch.object(sm, "get_spectrometer_reader", return_value=_DeadSpectrometer()):
            with self.assertRaises(RuntimeError) as ctx:
                sm.ScanManager(self._config(oes_enabled=True))
        self.assertIn("spectrometer did not connect", str(ctx.exception))
        self.assertIn("uncheck OES", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
