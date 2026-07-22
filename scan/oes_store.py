"""
oes_store.py — HDF5-backed store for OES scan data.

Replaces the per-point spectrum CSVs (spectra/point_XXXXX.csv) with a
single structured file that preserves the full (x, y, wavelength) grid.

HDF5 schema
-----------
  /x_mm           (nx,)              mm, spatial grid x coords
  /y_mm           (ny,)              mm, spatial grid y coords
  /wavelength_nm  (nλ,)              nm, initialized on first write
  /intensity      (nx, ny, npass, nλ) float32, NaN until written
  /ir_temp_c      (nx, ny, npass)    float32, NaN until written
  /ir_emissivity  (nx, ny, npass)    float32, NaN until written (last-poll
                                       value, not dwell-averaged — see
                                       ScanManager._read_ir_with_retry)
  /ir_dilution    (nx, ny, npass)    float32, NaN until written; stays all-NaN
                                       until ir.pac.dilution_tag_name is set
                                       (see tools/list_pac_strategy_vars.py)
  /timestamp      (nx, ny, npass)    float64, Unix epoch seconds
  /saturated      (nx, ny, npass)    bool
  /ir_error       (nx, ny, npass)    bool
  /oes_error      (nx, ny, npass)    bool

The `npass` axis holds one full-grid-pass revisit per index (see
scan.passes in config.yaml / ScanManager). npass=1 is the old shape in
everything but name — single-pass scans just have a size-1 pass axis,
so existing single-pass analysis code only needs `.isel(pass=0)` (or
`.sel(...).squeeze("pass")`) added to keep working. This axis exists
specifically so a repeated-pass scan preserves a per-point time series
instead of each later pass silently overwriting the previous one —
that history is what per-XY-point drift/oscillation tracking (T/e over
time) needs.

Crash safety: each write_point() opens the file, writes, and closes
immediately — a crashed scan leaves all completed points intact.

Typical usage
-------------
    # 1. At scan start (n_passes from scan.passes in config.yaml)
    store = OESStore("scan_data/oes.h5", x_coords_mm=xs, y_coords_mm=ys,
                      n_passes=3)

    # 2. Per point (ix/iy are 0-based grid indices, not mm values;
    #    pass_id is the 0-based index of which full-grid pass this is)
    store.write_point(ix=3, iy=7, pass_id=0,
                      wavelengths=reading.wavelengths,
                      intensities=reading.intensities,
                      ir_temp_c=950.2,
                      timestamp=time.time(),
                      saturated=reading.saturated)

    # 3. Analysis / post-processing
    ds = OESStore.load("scan_data/oes.h5")

    # Full spectrum at the center point, first pass
    ds.intensity.sel(x_mm=25.0, y_mm=25.0, method="nearest").isel(pass_id=0)

    # C2 Swan (516 nm) spatial map for the latest pass
    ds.intensity.sel(wavelength_nm=516.0, method="nearest").isel(pass_id=-1)

    # IR temperature time series at one point, across passes (oscillation input)
    ds.ir_temp_c.sel(x_mm=25.0, y_mm=25.0, method="nearest")

    # All spectra where IR > 900 °C
    hot = ds.where(ds.ir_temp_c > 900)
"""

from __future__ import annotations

import time
from typing import Optional

import h5py
import numpy as np
import xarray as xr


class OESStore:
    """
    Crash-safe HDF5 writer for 2D spatial OES scans.

    Wavelengths are not required at construction — the file is created
    (and the wavelength/intensity datasets are pre-allocated) on the
    first call to write_point() that supplies wavelengths.
    """

    def __init__(self, path: str, x_coords_mm, y_coords_mm, n_passes: int = 1):
        """
        Parameters
        ----------
        path : str
            Destination .h5 file path. Created on first write.
        x_coords_mm : array-like
            1-D array of x grid positions in mm (length nx).
        y_coords_mm : array-like
            1-D array of y grid positions in mm (length ny).
        n_passes : int
            Number of full-grid passes this scan will make (scan.passes
            in config.yaml). Must be known upfront so the pass axis can
            be pre-allocated like every other dimension here — defaults
            to 1 (single pass) for callers that don't care about repeats.
        """
        self.path = path
        self.x_coords = np.asarray(x_coords_mm, dtype="float32")
        self.y_coords = np.asarray(y_coords_mm, dtype="float32")
        self.n_passes = int(n_passes)
        self._initialized = False

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _initialize(self, wavelengths: np.ndarray) -> None:
        """Create the HDF5 file and pre-allocate all datasets."""
        nx = len(self.x_coords)
        ny = len(self.y_coords)
        np_ = self.n_passes
        nl = len(wavelengths)

        with h5py.File(self.path, "w") as f:
            f.create_dataset("x_mm", data=self.x_coords)
            f.create_dataset("y_mm", data=self.y_coords)
            f.create_dataset("wavelength_nm", data=wavelengths.astype("float32"))
            f.create_dataset("pass_id", data=np.arange(np_, dtype="int32"))

            f.create_dataset("intensity", shape=(nx, ny, np_, nl),
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("ir_temp_c", shape=(nx, ny, np_),
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("ir_emissivity", shape=(nx, ny, np_),
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("ir_dilution", shape=(nx, ny, np_),
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("timestamp", shape=(nx, ny, np_),
                             dtype="float64", fillvalue=np.nan)
            f.create_dataset("saturated", shape=(nx, ny, np_),
                             dtype=bool, fillvalue=False)
            f.create_dataset("ir_error", shape=(nx, ny, np_),
                             dtype=bool, fillvalue=False)
            f.create_dataset("oes_error", shape=(nx, ny, np_),
                             dtype=bool, fillvalue=False)

        self._initialized = True

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def write_point(
        self,
        ix: int,
        iy: int,
        pass_id: int = 0,
        wavelengths: Optional[np.ndarray] = None,
        intensities: Optional[np.ndarray] = None,
        ir_temp_c: Optional[float] = None,
        ir_emissivity: Optional[float] = None,
        ir_dilution: Optional[float] = None,
        timestamp: Optional[float] = None,
        saturated: bool = False,
        ir_error: bool = False,
        oes_error: bool = False,
    ) -> None:
        """
        Write one scan point to the HDF5 file.

        Parameters
        ----------
        ix, iy : int
            0-based grid indices (not mm values).
        pass_id : int
            0-based index of which full-grid pass this point belongs to.
            Must be < n_passes given at construction — out-of-range
            raises IndexError from h5py rather than silently truncating,
            since that would quietly discard a real measurement.
        wavelengths : np.ndarray, optional
            Required on the first call to initialize the file.
        intensities : np.ndarray, optional
            Spectrum counts/a.u., same length as wavelengths.
        ir_temp_c : float, optional
            Dwell-time-averaged ("filtered") pyrometer temperature.
        ir_emissivity : float, optional
            Last-poll pyrometer emissivity/strength (not dwell-averaged).
        ir_dilution : float, optional
            Last-poll pyro signal dilution. Stays NaN for every point until
            ir.pac.dilution_tag_name is confirmed and set in config.yaml.
        timestamp : float, optional
            Unix epoch seconds. Defaults to now.
        saturated, ir_error, oes_error : bool
        """
        if not self._initialized:
            if wavelengths is None:
                raise ValueError(
                    "wavelengths must be provided on the first write_point() "
                    "call so the HDF5 datasets can be sized correctly."
                )
            self._initialize(wavelengths)

        with h5py.File(self.path, "a") as f:
            if intensities is not None:
                f["intensity"][ix, iy, pass_id, :] = intensities.astype("float32")
            if ir_temp_c is not None:
                f["ir_temp_c"][ix, iy, pass_id] = float(ir_temp_c)
            if ir_emissivity is not None:
                f["ir_emissivity"][ix, iy, pass_id] = float(ir_emissivity)
            if ir_dilution is not None:
                f["ir_dilution"][ix, iy, pass_id] = float(ir_dilution)
            f["timestamp"][ix, iy, pass_id] = timestamp if timestamp is not None else time.time()
            f["saturated"][ix, iy, pass_id] = saturated
            f["ir_error"][ix, iy, pass_id] = ir_error
            f["oes_error"][ix, iy, pass_id] = oes_error

    @staticmethod
    def load(path: str) -> xr.Dataset:
        """
        Load a completed (or partial) scan as a labeled xarray Dataset.

        NaN entries in intensity/ir_temp_c mark points not yet written
        (useful for inspecting a scan that died partway through).

        Returns
        -------
        xr.Dataset with data variables:
            intensity     (x_mm, y_mm, pass_id, wavelength_nm)
            ir_temp_c     (x_mm, y_mm, pass_id)
            ir_emissivity (x_mm, y_mm, pass_id)
            ir_dilution   (x_mm, y_mm, pass_id)
            timestamp     (x_mm, y_mm, pass_id)
            saturated     (x_mm, y_mm, pass_id)
            ir_error      (x_mm, y_mm, pass_id)
            oes_error     (x_mm, y_mm, pass_id)

        pass_id is size 1 for an ordinary single-pass scan — index/select
        it the same way regardless (e.g. `.isel(pass_id=-1)` for "latest
        pass"), rather than special-casing single- vs. multi-pass scans.
        `.sel(x_mm=..., y_mm=..., method="nearest")` on ir_temp_c gives
        the full time series across passes at one point, which is the
        oscillation-detection input.
        """
        with h5py.File(path, "r") as f:
            shape = f["ir_temp_c"].shape

            def _optional(name):
                # ir_emissivity/ir_dilution were added 2026-07-21 — older
                # .h5 files written before that won't have these datasets.
                # Fall back to all-NaN (same convention as "not yet
                # written") instead of a KeyError so old scans still load.
                if name in f:
                    return f[name][:]
                return np.full(shape, np.nan, dtype="float32")

            return xr.Dataset(
                {
                    "intensity": (
                        ["x_mm", "y_mm", "pass_id", "wavelength_nm"],
                        f["intensity"][:],
                    ),
                    "ir_temp_c": (["x_mm", "y_mm", "pass_id"], f["ir_temp_c"][:]),
                    "ir_emissivity": (["x_mm", "y_mm", "pass_id"], _optional("ir_emissivity")),
                    "ir_dilution": (["x_mm", "y_mm", "pass_id"], _optional("ir_dilution")),
                    "timestamp": (["x_mm", "y_mm", "pass_id"], f["timestamp"][:]),
                    "saturated": (["x_mm", "y_mm", "pass_id"], f["saturated"][:]),
                    "ir_error": (["x_mm", "y_mm", "pass_id"], f["ir_error"][:]),
                    "oes_error": (["x_mm", "y_mm", "pass_id"], f["oes_error"][:]),
                },
                coords={
                    "x_mm": f["x_mm"][:],
                    "y_mm": f["y_mm"][:],
                    "pass_id": f["pass_id"][:],
                    "wavelength_nm": f["wavelength_nm"][:],
                },
                attrs={"source": str(path)},
            )


# ---------------------------------------------------------------------------
# Smoke test — run this file directly to verify h5py + xarray install
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import os
    import tempfile

    print("Running OESStore smoke test...")

    xs = np.linspace(0, 50, 5)
    ys = np.linspace(0, 50, 5)
    wl = np.linspace(200, 900, 2048)

    with tempfile.NamedTemporaryFile(suffix=".h5", delete=False) as tmp:
        path = tmp.name

    n_passes = 2
    try:
        store = OESStore(path, x_coords_mm=xs, y_coords_mm=ys, n_passes=n_passes)

        for pass_id in range(n_passes):
            for ix, x in enumerate(xs):
                for iy, y in enumerate(ys):
                    # fake spectrum: baseline + C2 Swan peak at 516 nm,
                    # drifting slightly between passes so the pass axis
                    # is actually distinguishable in the assertions below
                    spec = np.random.normal(200, 50, len(wl))
                    spec += 12000 * np.exp(-0.5 * ((wl - 516) / 4) ** 2)
                    spec = np.clip(spec, 0, 65535)

                    store.write_point(
                        ix=ix, iy=iy, pass_id=pass_id,
                        wavelengths=wl,
                        intensities=spec,
                        ir_temp_c=900.0 + ix * 5 + iy + pass_id * 10,
                        ir_emissivity=0.85 + 0.01 * ix,
                        ir_dilution=1.0 + 0.02 * iy,
                        saturated=False,
                    )

        ds = OESStore.load(path)
        print(f"Dataset shape: {dict(ds.sizes)}")
        assert ds.sizes["pass_id"] == n_passes
        assert not np.isnan(ds.ir_temp_c.values).any(), "every (ix, iy, pass) should be written"
        assert not np.isnan(ds.ir_emissivity.values).any(), "every (ix, iy, pass) should be written"
        assert not np.isnan(ds.ir_dilution.values).any(), "every (ix, iy, pass) should be written"
        # Same XY point, later pass should be +10 (per the fake drift above)
        delta = float(ds.ir_temp_c.isel(pass_id=1).values[2, 2] - ds.ir_temp_c.isel(pass_id=0).values[2, 2])
        assert abs(delta - 10.0) < 1e-3, f"expected pass-to-pass delta of 10.0, got {delta}"
        print(f"IR range: {float(ds.ir_temp_c.min()):.1f} – {float(ds.ir_temp_c.max()):.1f} °C")
        print(f"Emissivity range: {float(ds.ir_emissivity.min()):.3f} – {float(ds.ir_emissivity.max()):.3f}")
        print(f"Dilution range: {float(ds.ir_dilution.min()):.3f} – {float(ds.ir_dilution.max()):.3f}")
        c2_map = ds.intensity.sel(wavelength_nm=516.0, method="nearest").isel(pass_id=-1)
        print(f"C2 Swan (516 nm) map mean (latest pass): {float(c2_map.mean()):.1f}")
        print("OK")
    finally:
        os.unlink(path)
