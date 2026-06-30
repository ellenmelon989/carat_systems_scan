"""
oes_store.py — HDF5-backed store for OES scan data.

Replaces the per-point spectrum CSVs (spectra/point_XXXXX.csv) with a
single structured file that preserves the full (x, y, wavelength) grid.

HDF5 schema
-----------
  /x_mm           (nx,)          mm, spatial grid x coords
  /y_mm           (ny,)          mm, spatial grid y coords
  /wavelength_nm  (nλ,)          nm, initialized on first write
  /intensity      (nx, ny, nλ)   float32, NaN until written
  /ir_temp_c      (nx, ny)       float32, NaN until written
  /timestamp      (nx, ny)       float64, Unix epoch seconds
  /saturated      (nx, ny)       bool
  /ir_error       (nx, ny)       bool
  /oes_error      (nx, ny)       bool

Crash safety: each write_point() opens the file, writes, and closes
immediately — a crashed scan leaves all completed points intact.

Typical usage
-------------
    # 1. At scan start
    store = OESStore("scan_data/oes.h5", x_coords_mm=xs, y_coords_mm=ys)

    # 2. Per point (ix/iy are 0-based grid indices, not mm values)
    store.write_point(ix=3, iy=7,
                      wavelengths=reading.wavelengths,
                      intensities=reading.intensities,
                      ir_temp_c=950.2,
                      timestamp=time.time(),
                      saturated=reading.saturated)

    # 3. Analysis / post-processing
    ds = OESStore.load("scan_data/oes.h5")

    # Full spectrum at the center point
    ds.intensity.sel(x_mm=25.0, y_mm=25.0, method="nearest")

    # C2 Swan (516 nm) spatial map — plug straight into matplotlib
    ds.intensity.sel(wavelength_nm=516.0, method="nearest").plot()

    # IR temperature map
    ds.ir_temp_c.plot()

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

    def __init__(self, path: str, x_coords_mm, y_coords_mm):
        """
        Parameters
        ----------
        path : str
            Destination .h5 file path. Created on first write.
        x_coords_mm : array-like
            1-D array of x grid positions in mm (length nx).
        y_coords_mm : array-like
            1-D array of y grid positions in mm (length ny).
        """
        self.path = path
        self.x_coords = np.asarray(x_coords_mm, dtype="float32")
        self.y_coords = np.asarray(y_coords_mm, dtype="float32")
        self._initialized = False

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _initialize(self, wavelengths: np.ndarray) -> None:
        """Create the HDF5 file and pre-allocate all datasets."""
        nx = len(self.x_coords)
        ny = len(self.y_coords)
        nl = len(wavelengths)

        with h5py.File(self.path, "w") as f:
            f.create_dataset("x_mm", data=self.x_coords)
            f.create_dataset("y_mm", data=self.y_coords)
            f.create_dataset("wavelength_nm", data=wavelengths.astype("float32"))

            f.create_dataset("intensity", shape=(nx, ny, nl),
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("ir_temp_c", shape=(nx, ny),
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("timestamp", shape=(nx, ny),
                             dtype="float64", fillvalue=np.nan)
            f.create_dataset("saturated", shape=(nx, ny),
                             dtype=bool, fillvalue=False)
            f.create_dataset("ir_error", shape=(nx, ny),
                             dtype=bool, fillvalue=False)
            f.create_dataset("oes_error", shape=(nx, ny),
                             dtype=bool, fillvalue=False)

        self._initialized = True

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def write_point(
        self,
        ix: int,
        iy: int,
        wavelengths: Optional[np.ndarray] = None,
        intensities: Optional[np.ndarray] = None,
        ir_temp_c: Optional[float] = None,
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
        wavelengths : np.ndarray, optional
            Required on the first call to initialize the file.
        intensities : np.ndarray, optional
            Spectrum counts/a.u., same length as wavelengths.
        ir_temp_c : float, optional
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
                f["intensity"][ix, iy, :] = intensities.astype("float32")
            if ir_temp_c is not None:
                f["ir_temp_c"][ix, iy] = float(ir_temp_c)
            f["timestamp"][ix, iy] = timestamp if timestamp is not None else time.time()
            f["saturated"][ix, iy] = saturated
            f["ir_error"][ix, iy] = ir_error
            f["oes_error"][ix, iy] = oes_error

    @staticmethod
    def load(path: str) -> xr.Dataset:
        """
        Load a completed (or partial) scan as a labeled xarray Dataset.

        NaN entries in intensity/ir_temp_c mark points not yet written
        (useful for inspecting a scan that died partway through).

        Returns
        -------
        xr.Dataset with data variables:
            intensity   (x_mm, y_mm, wavelength_nm)
            ir_temp_c   (x_mm, y_mm)
            timestamp   (x_mm, y_mm)
            saturated   (x_mm, y_mm)
            ir_error    (x_mm, y_mm)
            oes_error   (x_mm, y_mm)
        """
        with h5py.File(path, "r") as f:
            return xr.Dataset(
                {
                    "intensity": (
                        ["x_mm", "y_mm", "wavelength_nm"],
                        f["intensity"][:],
                    ),
                    "ir_temp_c": (["x_mm", "y_mm"], f["ir_temp_c"][:]),
                    "timestamp": (["x_mm", "y_mm"], f["timestamp"][:]),
                    "saturated": (["x_mm", "y_mm"], f["saturated"][:]),
                    "ir_error": (["x_mm", "y_mm"], f["ir_error"][:]),
                    "oes_error": (["x_mm", "y_mm"], f["oes_error"][:]),
                },
                coords={
                    "x_mm": f["x_mm"][:],
                    "y_mm": f["y_mm"][:],
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

    try:
        store = OESStore(path, x_coords_mm=xs, y_coords_mm=ys)

        for ix, x in enumerate(xs):
            for iy, y in enumerate(ys):
                # fake spectrum: baseline + C2 Swan peak at 516 nm
                spec = np.random.normal(200, 50, len(wl))
                spec += 12000 * np.exp(-0.5 * ((wl - 516) / 4) ** 2)
                spec = np.clip(spec, 0, 65535)

                store.write_point(
                    ix=ix, iy=iy,
                    wavelengths=wl,
                    intensities=spec,
                    ir_temp_c=900.0 + ix * 5 + iy,
                    saturated=False,
                )

        ds = OESStore.load(path)
        print(f"Dataset shape: {dict(ds.sizes)}")
        print(f"IR range: {float(ds.ir_temp_c.min()):.1f} – {float(ds.ir_temp_c.max()):.1f} °C")
        c2_map = ds.intensity.sel(wavelength_nm=516.0, method="nearest")
        print(f"C2 Swan (516 nm) map mean: {float(c2_map.mean()):.1f}")
        print("OK")
    finally:
        os.unlink(path)
