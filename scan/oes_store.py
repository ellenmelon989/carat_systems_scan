"""
oes_store.py — HDF5-backed store for OES scan data.

Replaces the per-point spectrum CSVs (spectra/point_XXXXX.csv) with a
single structured file that preserves the full (x, y, wavelength) grid.

HDF5 schema — GRID MODE (mode="grid", the default; unchanged since this
module was written)
-----------------------------------------------------------------------
  /x_mm           (nx,)              mm, spatial grid x coords
  /y_mm           (ny,)              mm, spatial grid y coords
  /wavelength_nm  (nλ,)              nm, sized from the wavelengths given
                                       at construction (or on first write
                                       if omitted there — see class docstring)
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

HDF5 schema — LINE MODE (mode="line", added PR2b 2026-09-08)
-----------------------------------------------------------------------
An arbitrary-angle line scan's x_mm and y_mm are NOT independent axes
the way a rectangular grid's are — each point's (x_mm, y_mm) is jointly
determined by one line-position parameter, not separately addressable.
The grid-mode (nx,)/(ny,) tensor-product layout above physically cannot
represent that, so line mode uses a different, smaller layout with ONE
spatial axis:

  /s_mm           (n,)               mm, arc length along the line from
                                       start_mm (0.0) to end_mm (line
                                       length) — see
                                       scan_manager.generate_line_points().
                                       This is the ONLY queryable spatial
                                       coordinate axis in line mode.
  /line_x_mm      (n,)               mm, REAL physical x — side-car
                                       (non-axis) dataset, NOT queryable
                                       via .sel(), for reference/plotting.
  /line_y_mm      (n,)               mm, REAL physical y — same as above.
  /wavelength_nm, /intensity, /ir_temp_c, /ir_emissivity, /ir_dilution,
  /timestamp, /saturated, /ir_error, /oes_error — same meaning as grid
    mode, but shaped (n, npass, ...) instead of (nx, ny, npass, ...).

  File attrs (h5py root-group attrs, not datasets): start_mm, end_mm —
  the two endpoints generate_line_points() was called with, recorded
  for provenance (so a saved line-mode .h5 is self-describing about
  where its line actually was, without having to cross-reference
  metadata.yaml).

Deliberately does NOT reuse the name x_mm/y_mm for the s_mm axis, even
though a line scan only has one true spatial degree of freedom that
COULD have been squeezed into an existing name. Checked before this
decision: map_plotter.py, gui/status_panel.py, gui/live_map.py, and
this module's own .load()/docstring all read x_mm/y_mm as real
physical millimetres — silently redefining that name to mean "distance
along the line" for line-mode files would misread as a real coordinate
at every one of those call sites. s_mm is new, additive, and only
present for line-mode scans; x_mm/y_mm (as line_x_mm/line_y_mm here,
and unconditionally under their own real names everywhere else in the
codebase — scan_summary.csv, DataLogger's live records) always mean
what they've always meant.

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

Typical usage — grid mode
--------------------------
    # 1. At scan start (n_passes from scan.passes in config.yaml; pass
    #    wavelengths up front whenever known, e.g. reader.wavelengths,
    #    so a failed first point can't crash the whole scan)
    store = OESStore("scan_data/oes.h5", x_coords_mm=xs, y_coords_mm=ys,
                      n_passes=3, wavelengths=reader.wavelengths)

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

Typical usage — line mode
---------------------------
    store = OESStore("scan_data/oes.h5", mode="line",
                      s_coords_mm=s_arr, line_x_mm=x_arr, line_y_mm=y_arr,
                      start_mm=(0.0, 0.0), end_mm=(20.0, 10.0),
                      n_passes=1, wavelengths=reader.wavelengths)
    store.write_point(ix=3, pass_id=0, wavelengths=..., intensities=...,
                      ir_temp_c=950.2, timestamp=time.time())

    ds = OESStore.load("scan_data/oes.h5")
    # Temperature vs. position along the line, first pass
    ds.ir_temp_c.sel(s_mm=10.0, method="nearest").isel(pass_id=0)
"""

from __future__ import annotations

import os
import time
from typing import Optional

import h5py
import numpy as np
import xarray as xr


class OESStore:
    """
    Crash-safe HDF5 writer for a 2D spatial (grid mode) or 1D
    arbitrary-angle-line (line mode) OES scan — see module docstring
    for the two schemas and when each applies.

    Wavelengths should be supplied at construction whenever they're known
    upfront (see `wavelengths` param below) so the file is fully
    pre-allocated before the scan's first point is ever measured. If
    omitted, the file is instead created lazily on the first call to
    write_point() that supplies wavelengths -- but that means a failed
    *first* point (motion fault or OES read error, both of which pass
    wavelengths=None) raises ValueError before the store has ever been
    initialized, aborting the whole scan over a single-point hiccup. See
    scan_manager.py's ScanManager.__init__, which passes
    spectrometer.wavelengths (the reader's calibration array, set at
    reader construction/connection time independent of any read()
    succeeding) to avoid exactly this.
    """

    def __init__(self, path: str, x_coords_mm=None, y_coords_mm=None, n_passes: int = 1,
                 wavelengths=None, mode: str = "grid",
                 s_coords_mm=None, line_x_mm=None, line_y_mm=None,
                 start_mm=None, end_mm=None):
        """
        Parameters
        ----------
        path : str
            Destination .h5 file path. Created on first write.
        mode : str
            "grid" (default, unchanged behavior) or "line" (PR2b,
            2026-09-08). Determines which schema this store uses — see
            module docstring. Every other param below is grouped by
            which mode it applies to; passing the wrong group for the
            given mode raises ValueError rather than silently ignoring
            it, since a mismatched param set here means a caller bug
            (e.g. scan_manager.py building the wrong kind of store for
            scan_cfg["grid"]["mode"]), not a legitimate "don't care".
        x_coords_mm, y_coords_mm : array-like, GRID MODE only
            1-D arrays of grid positions in mm (length nx, ny).
        s_coords_mm, line_x_mm, line_y_mm : array-like, LINE MODE only
            1-D arrays, all length n: arc-length position (the one
            queryable spatial axis), and the real physical x/y at each
            of those positions (side-car, non-axis datasets — see
            module docstring for why these aren't named x_mm/y_mm).
        start_mm, end_mm : (float, float), LINE MODE only
            The line's two endpoints, recorded as file attrs for
            provenance.
        n_passes : int
            Number of full-grid (or full-line) passes this scan will
            make (scan.passes in config.yaml). Must be known upfront so
            the pass axis can be pre-allocated like every other
            dimension here — defaults to 1 (single pass) for callers
            that don't care about repeats.
        wavelengths : array-like, optional
            The spectrometer's wavelength calibration (nm), if already
            known (e.g. reader.wavelengths right after it connects).
            When given, the HDF5 file is created and every dataset
            pre-allocated immediately, so a failed first scan point no
            longer crashes the whole scan for lack of a sizing reference
            (see class docstring). When omitted, falls back to the old
            lazy-init-on-first-successful-write_point() behavior.
        """
        if mode not in ("grid", "line"):
            raise ValueError(f"mode must be 'grid' or 'line', got {mode!r}")
        self.mode = mode
        self.path = path
        self.n_passes = int(n_passes)
        self._initialized = False

        if mode == "grid":
            if x_coords_mm is None or y_coords_mm is None:
                raise ValueError("mode='grid' requires x_coords_mm and y_coords_mm")
            self.x_coords = np.asarray(x_coords_mm, dtype="float32")
            self.y_coords = np.asarray(y_coords_mm, dtype="float32")
        else:
            if s_coords_mm is None or line_x_mm is None or line_y_mm is None:
                raise ValueError(
                    "mode='line' requires s_coords_mm, line_x_mm, and line_y_mm")
            if start_mm is None or end_mm is None:
                raise ValueError("mode='line' requires start_mm and end_mm")
            self.s_coords = np.asarray(s_coords_mm, dtype="float32")
            self.line_x = np.asarray(line_x_mm, dtype="float32")
            self.line_y = np.asarray(line_y_mm, dtype="float32")
            self.start_mm = (float(start_mm[0]), float(start_mm[1]))
            self.end_mm = (float(end_mm[0]), float(end_mm[1]))

        # Loud, not silent: _initialize() below opens this path with
        # h5py.File(path, "w") -- "w" mode TRUNCATES an existing file.
        # Reusing output.base_dir from an earlier scan (e.g. the default
        # ./scan_data left unchanged between runs) means the PREVIOUS
        # scan's entire oes.h5 is silently destroyed the moment this
        # store initializes, with no prompt and no backup. Opposite
        # failure mode from DataLogger's summary CSV (which appends
        # instead -- see that class's own warning), but the same root
        # cause: nothing in this codebase checks for or timestamps a
        # reused output directory.
        if os.path.exists(self.path):
            print(
                f"WARNING: {self.path} already exists and will be OVERWRITTEN "
                "(replaced, not appended) as soon as this store initializes. If "
                "this is a different scan than whatever wrote the existing file, "
                "point a different output.base_dir / output.oes_hdf5 at it "
                "first, or move/rename the existing file now."
            )

        if wavelengths is not None:
            self._initialize(np.asarray(wavelengths))

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _initialize(self, wavelengths: np.ndarray) -> None:
        """Create the HDF5 file and pre-allocate all datasets."""
        np_ = self.n_passes
        nl = len(wavelengths)

        # Defensive: normally scan_manager.py has already created
        # output.base_dir before this ever runs, but output.oes_hdf5 can
        # be pointed at an arbitrary path outside base_dir (see config.yaml),
        # whose parent directory nothing else guarantees exists. Without
        # this, h5py.File(path, "w") below fails with "Unable to
        # synchronously create file ... No such file or directory" -- a
        # confusing error that names the wrong problem (looks like a
        # hardware/file-corruption issue, not a missing folder).
        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)

        with h5py.File(self.path, "w") as f:
            if self.mode == "grid":
                nx = len(self.x_coords)
                ny = len(self.y_coords)
                shape = (nx, ny, np_)
                f.create_dataset("x_mm", data=self.x_coords)
                f.create_dataset("y_mm", data=self.y_coords)
            else:
                n = len(self.s_coords)
                shape = (n, np_)
                f.create_dataset("s_mm", data=self.s_coords)
                f.create_dataset("line_x_mm", data=self.line_x)
                f.create_dataset("line_y_mm", data=self.line_y)
                f.attrs["start_mm"] = self.start_mm
                f.attrs["end_mm"] = self.end_mm

            f.create_dataset("wavelength_nm", data=wavelengths.astype("float32"))
            f.create_dataset("pass_id", data=np.arange(np_, dtype="int32"))

            f.create_dataset("intensity", shape=shape + (nl,),
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("ir_temp_c", shape=shape,
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("ir_emissivity", shape=shape,
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("ir_dilution", shape=shape,
                             dtype="float32", fillvalue=np.nan)
            f.create_dataset("timestamp", shape=shape,
                             dtype="float64", fillvalue=np.nan)
            f.create_dataset("saturated", shape=shape,
                             dtype=bool, fillvalue=False)
            f.create_dataset("ir_error", shape=shape,
                             dtype=bool, fillvalue=False)
            f.create_dataset("oes_error", shape=shape,
                             dtype=bool, fillvalue=False)

        self._initialized = True

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def write_point(
        self,
        ix: int,
        iy: Optional[int] = None,
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
        ix : int
            0-based index. GRID MODE: the x grid index (paired with
            iy). LINE MODE: the single line-position index i (0-based
            along the line, from generate_line_points()) -- the "ix"
            name is reused rather than adding a separate parameter, to
            keep this call shape close to DataLogger.write_point()'s,
            which forwards ix/iy straight through from
            scan_manager.py's per-mode-normalized (ix, iy, x, y, s_mm)
            tuples (see ScanManager.run()).
        iy : int, optional
            GRID MODE: required, the y grid index. LINE MODE: always
            None -- there is only one spatial index. self.mode decides
            which of these two shapes to expect; passing iy in line
            mode (or omitting it in grid mode) raises ValueError rather
            than silently indexing the wrong axis.
        pass_id : int
            0-based index of which full pass this point belongs to.
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
        if self.mode == "grid":
            if iy is None:
                raise ValueError("mode='grid' requires iy")
            index = (ix, iy, pass_id)
        else:
            if iy is not None:
                raise ValueError("mode='line' does not use iy (got a non-None value)")
            index = (ix, pass_id)

        if not self._initialized:
            if wavelengths is None:
                raise ValueError(
                    "wavelengths must be provided on the first write_point() "
                    "call so the HDF5 datasets can be sized correctly."
                )
            self._initialize(wavelengths)

        with h5py.File(self.path, "a") as f:
            if intensities is not None:
                f["intensity"][index + (slice(None),)] = intensities.astype("float32")
            if ir_temp_c is not None:
                f["ir_temp_c"][index] = float(ir_temp_c)
            if ir_emissivity is not None:
                f["ir_emissivity"][index] = float(ir_emissivity)
            if ir_dilution is not None:
                f["ir_dilution"][index] = float(ir_dilution)
            f["timestamp"][index] = timestamp if timestamp is not None else time.time()
            f["saturated"][index] = saturated
            f["ir_error"][index] = ir_error
            f["oes_error"][index] = oes_error

    @staticmethod
    def load(path: str) -> xr.Dataset:
        """
        Load a completed (or partial) scan as a labeled xarray Dataset.
        Detects grid vs. line mode from whether the file has an /s_mm
        dataset (line mode) or /x_mm + /y_mm (grid mode) -- mode isn't
        itself persisted as a separate flag since the dataset shapes
        already say unambiguously which schema is in use.

        NaN entries in intensity/ir_temp_c mark points not yet written
        (useful for inspecting a scan that died partway through).

        Returns
        -------
        xr.Dataset with data variables (dims (x_mm, y_mm, pass_id, ...)
        for a grid-mode file, (s_mm, pass_id, ...) for a line-mode one):
            intensity, ir_temp_c, ir_emissivity, ir_dilution, timestamp,
            saturated, ir_error, oes_error

        Line-mode files also carry line_x_mm/line_y_mm (real physical
        coordinates, indexed by s_mm — NOT usable as .sel() axes
        themselves, look them up via .sel(s_mm=...) instead) and the
        start_mm/end_mm attrs.

        pass_id is size 1 for an ordinary single-pass scan — index/select
        it the same way regardless (e.g. `.isel(pass_id=-1)` for "latest
        pass"), rather than special-casing single- vs. multi-pass scans.
        Grid mode: `.sel(x_mm=..., y_mm=..., method="nearest")` on
        ir_temp_c gives the full time series across passes at one point.
        Line mode: `.sel(s_mm=..., method="nearest")` does the same.
        """
        with h5py.File(path, "r") as f:
            is_line = "s_mm" in f
            shape = f["ir_temp_c"].shape

            def _optional(name):
                # ir_emissivity/ir_dilution were added 2026-07-21 — older
                # .h5 files written before that won't have these datasets.
                # Fall back to all-NaN (same convention as "not yet
                # written") instead of a KeyError so old scans still load.
                if name in f:
                    return f[name][:]
                return np.full(shape, np.nan, dtype="float32")

            if is_line:
                dims = ["s_mm", "pass_id"]
                data_vars = {
                    "intensity": (dims + ["wavelength_nm"], f["intensity"][:]),
                    "ir_temp_c": (dims, f["ir_temp_c"][:]),
                    "ir_emissivity": (dims, _optional("ir_emissivity")),
                    "ir_dilution": (dims, _optional("ir_dilution")),
                    "timestamp": (dims, f["timestamp"][:]),
                    "saturated": (dims, f["saturated"][:]),
                    "ir_error": (dims, f["ir_error"][:]),
                    "oes_error": (dims, f["oes_error"][:]),
                    "line_x_mm": (["s_mm"], f["line_x_mm"][:]),
                    "line_y_mm": (["s_mm"], f["line_y_mm"][:]),
                }
                coords = {
                    "s_mm": f["s_mm"][:],
                    "pass_id": f["pass_id"][:],
                    "wavelength_nm": f["wavelength_nm"][:],
                }
                attrs = {"source": str(path), "mode": "line"}
                if "start_mm" in f.attrs:
                    attrs["start_mm"] = tuple(f.attrs["start_mm"])
                if "end_mm" in f.attrs:
                    attrs["end_mm"] = tuple(f.attrs["end_mm"])
            else:
                dims = ["x_mm", "y_mm", "pass_id"]
                data_vars = {
                    "intensity": (dims + ["wavelength_nm"], f["intensity"][:]),
                    "ir_temp_c": (dims, f["ir_temp_c"][:]),
                    "ir_emissivity": (dims, _optional("ir_emissivity")),
                    "ir_dilution": (dims, _optional("ir_dilution")),
                    "timestamp": (dims, f["timestamp"][:]),
                    "saturated": (dims, f["saturated"][:]),
                    "ir_error": (dims, f["ir_error"][:]),
                    "oes_error": (dims, f["oes_error"][:]),
                }
                coords = {
                    "x_mm": f["x_mm"][:],
                    "y_mm": f["y_mm"][:],
                    "pass_id": f["pass_id"][:],
                    "wavelength_nm": f["wavelength_nm"][:],
                }
                attrs = {"source": str(path), "mode": "grid"}

            return xr.Dataset(data_vars, coords=coords, attrs=attrs)


# ---------------------------------------------------------------------------
# Smoke test — run this file directly to verify h5py + xarray install
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import os
    import tempfile

    print("Running OESStore smoke test (grid mode)...")

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
        assert ds.attrs["mode"] == "grid"
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
        print("OK (grid mode)")
    finally:
        os.unlink(path)

    # ------------------------------------------------------------------
    # PR2b (2026-09-08): line mode
    # ------------------------------------------------------------------
    print("Running OESStore smoke test (line mode)...")

    start_mm = (0.0, 0.0)
    end_mm = (30.0, 40.0)  # length 50mm, a 3-4-5 triangle for a clean check
    n = 6
    s_coords = np.linspace(0.0, 50.0, n)
    t = np.linspace(0.0, 1.0, n)
    line_x = start_mm[0] + t * (end_mm[0] - start_mm[0])
    line_y = start_mm[1] + t * (end_mm[1] - start_mm[1])

    with tempfile.NamedTemporaryFile(suffix=".h5", delete=False) as tmp:
        path = tmp.name

    try:
        store = OESStore(
            path, mode="line",
            s_coords_mm=s_coords, line_x_mm=line_x, line_y_mm=line_y,
            start_mm=start_mm, end_mm=end_mm, n_passes=1,
        )

        for i in range(n):
            spec = np.random.normal(200, 50, len(wl))
            store.write_point(
                ix=i, pass_id=0,
                wavelengths=wl, intensities=spec,
                ir_temp_c=900.0 + i,
                ir_emissivity=0.8,
                saturated=False,
            )

        ds = OESStore.load(path)
        print(f"Dataset shape: {dict(ds.sizes)}")
        assert ds.attrs["mode"] == "line"
        assert ds.sizes["s_mm"] == n
        assert "x_mm" not in ds.coords and "y_mm" not in ds.coords, (
            "line-mode dataset must not carry x_mm/y_mm as coordinate axes")
        assert not np.isnan(ds.ir_temp_c.values).any()
        assert float(ds.s_mm.max()) == 50.0
        assert abs(float(ds.line_x_mm.isel(s_mm=-1)) - 30.0) < 1e-3
        assert abs(float(ds.line_y_mm.isel(s_mm=-1)) - 40.0) < 1e-3
        assert tuple(ds.attrs["start_mm"]) == start_mm
        assert tuple(ds.attrs["end_mm"]) == end_mm
        # Temperature vs. position at a specific s_mm, nearest-match.
        mid_temp = float(ds.ir_temp_c.sel(s_mm=20.0, method="nearest").isel(pass_id=0))
        assert 900.0 <= mid_temp <= 900.0 + n - 1

        # Passing iy in line mode must fail loud, not silently misindex.
        try:
            store.write_point(ix=0, iy=0, pass_id=0)
            raise AssertionError("expected ValueError for iy in line mode")
        except ValueError:
            pass

        print(f"IR range: {float(ds.ir_temp_c.min()):.1f} – {float(ds.ir_temp_c.max()):.1f} °C")
        print("OK (line mode)")
    finally:
        os.unlink(path)

    # Passing iy=None in grid mode must also fail loud.
    with tempfile.NamedTemporaryFile(suffix=".h5", delete=False) as tmp:
        path2 = tmp.name
    try:
        store2 = OESStore(path2, x_coords_mm=xs, y_coords_mm=ys, n_passes=1,
                           wavelengths=wl)
        try:
            store2.write_point(ix=0, iy=None, pass_id=0)
            raise AssertionError("expected ValueError for missing iy in grid mode")
        except ValueError:
            pass
        print("OK (grid mode requires iy)")
    finally:
        os.unlink(path2)
