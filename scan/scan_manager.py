"""
scan_manager.py

Coordinates the scan sequence: motion, IR acquisition, OES
acquisition, and data logging. Sequential, single-threaded by
design (plasma drift timescale ~minutes makes async unnecessary).

Implements:
- Raster/serpentine 2D grid generation
- Per-point error policy (retry -> NaN + flag -> continue)
- Periodic revisit of a fixed reference point for drift tracking
- Optional multi-pass scanning (scan.passes in config.yaml): repeats the
  ENTIRE grid N times so every point gets rechecked over the course of
  the scan, not just the one fixed reference point. Defaults to 1 pass
  (old behavior). See OESStore's pass_id axis for how repeats are
  preserved rather than overwriting each other on disk.
Lives in scan/ (see scan/__init__.py) alongside scan_params.py,
data_logger.py, oes_store.py, calibrate_scan_area.py, and map_plotter.py —
the precision, absolute-position scan path, kept separate from
adaptive_scan/'s open-loop-safe edge-following mode.
"""
# --- repo-root import bootstrap -------------------------------------------
# Lets this file be run directly (`python scan/scan_manager.py`), as a
# module (`python -m scan.scan_manager`), or imported from elsewhere in the
# repo (e.g. gui/, via `python run_gui.py`) -- all three need the repo root
# on sys.path so sibling top-level packages (motion/, readers/) and this
# file's own package (scan/) resolve the same way regardless of how it was
# invoked. See adaptive_scan/adaptive_scan.py for the same pattern.
import os as _os
import sys as _sys

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)
# ---------------------------------------------------------------------------

import logging

import numpy as np

from motion.motion_controller import get_motion_controller, AxisStateUnknown
from readers.ir_reader_base import get_ir_reader
from readers.spectrometer_reader_base import get_spectrometer_reader
from scan.data_logger import DataLogger, build_point_record, resolve_run_dir
from scan.oes_store import OESStore
from scan.scan_params import (
    PASSES_DEFAULT,
    STEP_SIZE_MIN_MM,
    STEP_SIZE_MAX_MM,
    grid_dims_from_range,
    in_radius,
    validate_dwell_time_s,
    validate_passes,
    validate_points_within_limits,
)

logger = logging.getLogger(__name__)


def generate_grid(scan_cfg):
    """
    Generate a list of (ix, iy, x_mm, y_mm) points based on config,
    in raster or serpentine order.

    Grid point counts (nx, ny) are DERIVED from the edge-calibrated scan
    range (scan.grid.x_range_mm / y_range_mm — set by calibrate_scan_area.py)
    and the operator-set step_size_mm; they are not stored directly in
    config. See scan_params.grid_dims_from_range().

    ix, iy are 0-based grid indices used to address the HDF5 dataset;
    x_mm, y_mm are the physical positions in millimetres.

    x_range_mm/y_range_mm describe a rectangular BOUNDING BOX around the
    wafer (set from 4 edge jogs), not the wafer itself — a circular
    wafer's own shape means that box's corners are off-sample by
    construction. If scan.grid.wafer_radius_mm is set (calibrate_scan_area.py
    computes and writes it), points outside that radius of
    wafer_center_mm are dropped from the returned list entirely — never
    measured, and never commanded as a move. xs/ys are still returned as
    the FULL rectangular linspace (unmasked) — OESStore's HDF5 array is
    shaped from these, and masked-out cells simply stay unwritten/NaN
    rather than shrinking the array to a non-rectangular shape.
    wafer_radius_mm absent/None (old configs, or not yet calibrated)
    means no mask — every point in the rectangle is measured, exactly
    the old behavior.
    """
    x0, x1 = scan_cfg["grid"]["x_range_mm"]
    y0, y1 = scan_cfg["grid"]["y_range_mm"]
    step_size_mm = scan_cfg["grid"]["step_size_mm"]
    center_mm = scan_cfg["grid"].get("wafer_center_mm", [0.0, 0.0])
    radius_mm = scan_cfg["grid"].get("wafer_radius_mm")

    nx, ny = grid_dims_from_range((x0, x1), (y0, y1), step_size_mm)

    xs = np.linspace(x0, x1, nx)
    ys = np.linspace(y0, y1, ny)

    order = scan_cfg.get("scan_order", "raster")
    points = []

    for iy, y in enumerate(ys):
        row = list(enumerate(xs))           # [(ix, x), ...]
        if order == "serpentine" and iy % 2 == 1:
            row = row[::-1]
        for ix, x in row:
            if radius_mm is not None and not in_radius(x, y, center_mm, radius_mm):
                continue
            points.append((ix, iy, float(x), float(y)))

    return points, xs, ys


def solve_step_size_for_target_points(center_mm, radius_mm, target_n_points,
                                       step_min_mm=STEP_SIZE_MIN_MM,
                                       step_max_mm=STEP_SIZE_MAX_MM,
                                       scan_order="raster"):
    """
    PR2a (2026-09-04): find the LARGEST step_size_mm -- coarsest, so the
    fastest scan -- whose masked circular grid (tightly boxed around
    center_mm/radius_mm, same in_radius() masking generate_grid() applies
    everywhere else) contains AT LEAST target_n_points points.

    "At least, never fewer" per the confirmed requirement: resolution is
    never sacrificed below what was asked for, even if that means running
    a bit longer than a naive reading of target_n_points would suggest.
    Bisects on step_size_mm rather than solving in closed form, because
    the masked point count is a step function of step size (grid
    quantization + the circular mask), not a smooth one.

    Lives here, not in scan_params.py, because it has to call
    generate_grid() to count points -- scan_manager.py already imports
    scan_params.py, so the reverse import would be circular. Both
    existing call sites that need generate_grid() already reach into
    this module locally for exactly that reason (see PR1's comments in
    gui/calibration_panel.py and calibrate_scan_area.py) -- this is the
    same pattern, one more name in that same local import.

    Returns (step_size_mm, n_points, achieved):
      achieved=True  -- n_points >= target_n_points, step_size_mm is the
                         coarsest step that gets there.
      achieved=False -- even step_min_mm (the 1mm floor) can't reach
                         target_n_points at this radius; step_size_mm is
                         step_min_mm and n_points is the MAXIMUM
                         achievable -- surface this to the operator
                         rather than silently returning an undersized
                         scan.
    """
    cx, cy = center_mm
    x_range_mm = [cx - radius_mm, cx + radius_mm]
    y_range_mm = [cy - radius_mm, cy + radius_mm]

    def n_points_at(step_size_mm):
        cfg = {
            "grid": {
                "x_range_mm": x_range_mm,
                "y_range_mm": y_range_mm,
                "wafer_center_mm": [cx, cy],
                "wafer_radius_mm": radius_mm,
                "step_size_mm": step_size_mm,
            },
            "scan_order": scan_order,
        }
        points, _, _ = generate_grid(cfg)
        return len(points)

    n_at_max = n_points_at(step_max_mm)
    if n_at_max >= target_n_points:
        return step_max_mm, n_at_max, True

    n_at_min = n_points_at(step_min_mm)
    if n_at_min < target_n_points:
        return step_min_mm, n_at_min, False

    # Invariant now holds: n_points_at(lo) >= target > n_points_at(hi).
    lo, hi = step_min_mm, step_max_mm
    for _ in range(40):  # far more than enough for sub-1e-4mm precision
        if hi - lo < 1e-7:
            break
        mid = (lo + hi) / 2.0
        if n_points_at(mid) >= target_n_points:
            lo = mid
        else:
            hi = mid

    # Round to the codebase's persisted config precision (4 decimal
    # places -- matches CONFIG_RANGE_DECIMALS in calibrate_scan_area.py;
    # kept as a local literal here rather than importing it, to avoid
    # scan_manager.py reaching up into a calibration-workflow module for
    # a constant). Round DOWN (finer step, never coarser) and re-verify
    # -- rounding can tip the count back under target, exactly the class
    # of bug the 2026-07-17 grid-mismatch diagnosis already hit once from
    # an unguarded round().
    quantum = 1e-4
    step_rounded = (int(lo / quantum)) * quantum  # truncate toward zero == round down for positive mm
    step_rounded = round(step_rounded, 4)
    step_rounded = max(step_rounded, step_min_mm)
    n_rounded = n_points_at(step_rounded)
    if n_rounded < target_n_points:
        # Extremely rare (rounding landed exactly on a mask boundary) --
        # fall back to the un-rounded bisection value rather than
        # under-deliver.
        step_rounded = lo
        n_rounded = n_points_at(step_rounded)

    return step_rounded, n_rounded, True


def generate_line_points(start_mm, end_mm, n_points: int):
    """
    PR2b (2026-09-04, endpoint math added 2026-09-08): generate
    n_points evenly-spaced points along the straight segment from
    start_mm to end_mm (both (x_mm, y_mm) pairs), inclusive of both
    endpoints. Returns a list of (i, x_mm, y_mm, s_mm) tuples -- i is
    the 0-based index along the line (the line-mode counterpart to
    generate_grid()'s ix/iy), s_mm is arc length from start_mm (0.0 at
    start_mm, the full line length at end_mm).

    Closed-form -- step_mm = length_mm / (n_points - 1) -- unlike
    solve_step_size_for_target_points()'s bisection: a line's point
    count and spacing are exactly and uniquely determined by its two
    endpoints and the target count, no masking or grid quantization
    involved (see the PR2 scope writeup in the project gap-analysis doc
    for why the circular case needed bisection and this one doesn't).

    s_mm is the ONE queryable spatial coordinate axis oes_store.py
    stores for a line-mode scan -- NEVER x_mm/y_mm, which stay real
    physical millimetres everywhere in this codebase and would be
    actively misleading if repurposed to mean "distance along the
    line" (see oes_store.py's module docstring and the PR2b design
    writeup for the four call sites that assume x_mm/y_mm are real).
    An arbitrary-angle line's x_mm and y_mm are NOT independent axes
    the way a rectangular grid's are -- each point's x_mm/y_mm is
    jointly determined by s_mm, not separately addressable -- which is
    exactly why generate_grid()'s (nx,)/(ny,) tensor-product HDF5
    layout can't represent this and s_mm exists.

    n_points=1 returns a single point at start_mm (s_mm=0.0, i=0) --
    same "a degenerate range collapses to one point" convention
    scan_params.grid_dims_from_range() already uses for a zero-width
    grid axis.

    Does NOT validate against a calibrated wafer or motion.soft_limits
    -- both are checked elsewhere (in_radius() on start_mm/end_mm in
    gui/line_scan_panel.py before this is ever called; every point this
    returns is later re-checked against motion.soft_limits by
    preflight_check() below, same as every grid point already is).
    """
    if n_points < 1:
        raise ValueError(f"n_points must be >= 1, got {n_points}")

    x0, y0 = start_mm
    x1, y1 = end_mm

    if n_points == 1:
        return [(0, float(x0), float(y0), 0.0)]

    length_mm = float(np.hypot(x1 - x0, y1 - y0))
    points = []
    for i in range(n_points):
        t = i / (n_points - 1)
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        s = t * length_mm
        points.append((i, float(x), float(y), float(s)))
    return points


def compute_commanded_points(scan_cfg):
    """
    Build the COMPLETE list of (label, x_mm, y_mm) positions this scan
    config will ever command the motion controller to move to: every
    (already wafer-radius-masked) grid point from generate_grid(), PLUS
    the periodic reference-point revisit position if scan.reference_point
    is enabled (it's a real move issued during the scan, just not part
    of the spatial grid/HDF5 array — see generate_grid()'s docstring).

    Pure function over config — does not touch hardware, does not call
    generate_grid()'s np.linspace/masking more than once, and runs in
    well under a second even for a large grid. That's what makes it safe
    to call as a pre-flight check (see preflight_check() below) before
    connecting to a motion controller at all, and also what makes it a
    useful fast-iteration tool on its own: verifying a config's commanded
    positions doesn't require running an actual scan (minutes to hours,
    real hardware) to find out a soft limit or calibration is wrong.

    PR2b (2026-09-08): branches on scan_cfg["grid"].get("mode", "grid").
    Line mode's points come from generate_line_points() instead of
    generate_grid() -- every point it returns still gets checked against
    motion.soft_limits below exactly the same way, which is what makes
    this the right (and only) place a line scan's wafer-shape validation
    would need supplementing if that were needed here too. It isn't: a
    rectangle (motion.soft_limits) is convex just like a circle
    (wafer_radius_mm) is, but this function isn't the one relying on
    that -- gui/line_scan_panel.py's own two-endpoint in_radius() check
    is (see that panel's docstring) -- this function already checks
    every generated point regardless of mode, same as it always has.
    """
    mode = scan_cfg["grid"].get("mode", "grid")
    if mode == "line":
        line_points = generate_line_points(
            scan_cfg["grid"]["line_start_mm"],
            scan_cfg["grid"]["line_end_mm"],
            scan_cfg["grid"]["line_n_points"],
        )
        commanded = [(f"line point (i={i})", x, y) for i, x, y, _s in line_points]
    else:
        points, _, _ = generate_grid(scan_cfg)
        commanded = [(f"grid point (ix={ix}, iy={iy})", x, y) for ix, iy, x, y in points]

    ref_cfg = scan_cfg.get("reference_point", {})
    if ref_cfg.get("enabled", False):
        rx, ry = ref_cfg.get("position", (0.0, 0.0))
        commanded.append(("reference point", float(rx), float(ry)))

    return commanded


def preflight_check(config):
    """
    Validate EVERY position this scan config will ever command the
    motion controller to move to — the full raster grid plus the
    reference-point revisit — against motion.soft_limits, all before any
    hardware is touched. Raises ValueError (via
    scan_params.validate_points_within_limits) listing every violating
    point, not just the first, if anything is out of bounds.

    Deliberately does NOT build a MotionController, IR reader, or
    spectrometer connection: a bad config.yaml (wrong soft_limits after
    a re-calibration, a raster that drifted past the mount's real
    travel, a reference_point.position typo) should fail this check in
    well under a second, not after connecting to hardware, homing
    (which can take minutes at conservative home_velocity settings —
    see config.yaml's home_timeout_s), or running partway into a scan.

    Called from ScanManager.__init__ before anything else happens (see
    there), and also runnable standalone via
    `python scan/scan_manager.py --check-only` for a zero-hardware, sub-
    second sanity check of a config file.

    NOTE on what this does and doesn't prove: this checks that the
    *coordinates* generate_grid() produces fall inside the configured mm
    box. It does NOT re-derive whether motion.soft_limits itself
    correctly reflects the mount's real mechanical travel, and it does
    NOT verify homing repeatability — both of those are calibration/
    hardware properties, not something a pure-Python check over
    config.yaml can confirm. See motion/real_newport_motion.py's HOMING
    NOTE and resume() docstring for what home() actually guarantees.
    """
    scan_cfg = config["scan"]
    commanded = compute_commanded_points(scan_cfg)
    limits = config["motion"]["soft_limits"]
    validate_points_within_limits(commanded, limits)
    return commanded


def extract_features(wavelengths, intensities, features_cfg, window_nm):
    """
    Extract intensity values for each named spectral feature by
    integrating (summing) intensities within +/- window_nm of the
    feature's center wavelength.
    """
    feature_values = {}
    for name, center_nm in features_cfg.items():
        mask = np.abs(wavelengths - center_nm) <= window_nm
        feature_values[name] = float(np.sum(intensities[mask])) if np.any(mask) else float("nan")
    return feature_values


class ScanManager:
    def __init__(self, config, motion=None):
        """
        motion: optional, pre-constructed MotionController. Defaults to
        None, which builds a fresh one via get_motion_controller(config)
        exactly as before — standalone `python scan/scan_manager.py` is
        unaffected.

        Pass an existing (already-connected, already-homed) controller
        instance instead when combining calibration and scanning into
        one continuous session — see calibrate_scan_area.py's post-
        calibration "run the scan now" prompt. Reusing the SAME object
        (rather than letting ScanManager open a second connection) is
        what makes it safe to also pass already_homed=True to run():
        there's no re-derived cross-process trust involved, it's
        literally the same in-memory controller that was just homed.
        """
        self.config = config
        self.scan_cfg = config["scan"]
        self.oes_cfg = config["oes"]
        self.error_cfg = config["error_policy"]

        # dwell_time_s is the single operator-facing per-point IR averaging
        # duration (replaces the old ir.averaging_time_s config key — see
        # scan_params.py for the valid range).
        self.dwell_time_s = validate_dwell_time_s(self.scan_cfg["dwell_time_s"])

        # Number of full-grid passes (scan.passes in config.yaml). Defaults
        # to 1 (old single-pass behavior) so existing configs are
        # unaffected. >1 means the ENTIRE grid repeats that many times —
        # this is the "revisit every point" path, distinct from (and in
        # addition to) the single fixed reference_point revisit below.
        self.passes = validate_passes(self.scan_cfg.get("passes", PASSES_DEFAULT))

        # Pre-flight: validate the COMPLETE list of commanded positions
        # (every grid point + the reference-point revisit) against
        # motion.soft_limits BEFORE connecting to any hardware below —
        # see preflight_check()'s docstring. Deliberately placed ahead of
        # get_motion_controller()/get_ir_reader()/get_spectrometer_reader()
        # so a bad config fails immediately, not after opening a hardware
        # connection or (if a caller passed motion= already homed) after
        # burning real homing time.
        preflight_check(config)

        # _owns_motion tracks whether THIS __init__ opened the connection
        # (motion=None -> get_motion_controller() below) vs. received an
        # already-connected one from a caller (e.g. the Calibrate tab
        # hand-off in gui/app.py). Only a connection we opened ourselves
        # gets closed on the error path right below -- closing a
        # caller-owned, handed-off motion object here would pull the rug
        # out from under that caller's own cleanup expectations.
        self._owns_motion = motion is None
        self.motion = motion if motion is not None else get_motion_controller(config)
        try:
            self.ir_reader = get_ir_reader(config)
            self.spectrometer = get_spectrometer_reader(config)
        except Exception:
            # Without this, a config error here (e.g. a missing
            # ir.pac.temp_tag_name key) leaves the just-opened 8742/motion
            # connection dangling: ScanManager.__init__ raises, run_scan()
            # reports "Failed to initialize scan hardware" and returns, but
            # nothing ever calls self.motion.close(). The controller object
            # is only released whenever Python happens to garbage-collect
            # it, and if the hardware only allows one live connection (true
            # of the 8742 over USB/Ethernet), every later connection
            # attempt -- including an independent one from the Adaptive
            # Scan tab -- fails until then, usually surfacing as a
            # confusing secondary "Error closing 8742 connection: ...
            # object has no attribute '_stage'" instead of pointing at the
            # real cause. Close what we opened, then let the original
            # exception propagate unchanged.
            if self._owns_motion:
                try:
                    self.motion.close()
                except Exception as close_exc:
                    logger.warning(
                        "Also failed to close the motion connection while "
                        "cleaning up after a failed ScanManager init: %s",
                        close_exc,
                    )
            raise

        # Stamp output.base_dir with this run's date/time BEFORE it's used
        # for anything below (the default oes.h5 path and DataLogger's
        # CSV/metadata/log/spectra all derive from it) -- see
        # resolve_run_dir()'s docstring in data_logger.py. The operator's
        # configured/typed base_dir becomes a parent directory that each
        # scan gets its own dated subfolder under, so every scan's saved
        # data is labeled with when it ran without operator effort, and
        # two runs can never collide on the same output.base_dir.
        config["output"]["base_dir"] = resolve_run_dir(config["output"]["base_dir"])

        # Create the dated run dir now, before anything tries to write into
        # it. OESStore below can eagerly write oes.h5 (see its wavelengths=
        # comment) as soon as it's constructed, which happens BEFORE
        # DataLogger.__init__ -- previously the only place that called
        # os.makedirs() on base_dir. That ordering meant a successful
        # spectrometer connection made h5py.File(path, "w") the first thing
        # to touch this not-yet-existent directory, failing with h5py's
        # "Unable to synchronously create file (unable to open file: ...
        # errno = 2, error message = 'No such file or directory')" instead
        # of a clear "failed to initialize scan hardware" cause. Making the
        # dir here, right after base_dir is finalized, removes the ordering
        # dependency on DataLogger entirely.
        _os.makedirs(config["output"]["base_dir"], exist_ok=True)

        # Build OESStore from grid coords so it's ready before the scan starts.
        # Pass the spectrometer's wavelength calibration (known from
        # connection, not from a successful read) so the HDF5 file is
        # fully pre-allocated now rather than lazily on first
        # write_point() -- otherwise a motion or OES failure on the
        # scan's very first point (both pass wavelengths=None) raises
        # ValueError out of OESStore before it's ever initialized and
        # aborts the entire scan over one bad point. self.spectrometer.
        # wavelengths is None only if the reader itself never connected
        # (e.g. pyseabreeze couldn't find the device), in which case
        # OESStore falls back to the old lazy-init behavior.
        # n_passes must be given upfront so the pass axis can be
        # pre-allocated (see oes_store.py) — a later pass overwriting a
        # smaller array would silently discard earlier passes' data.
        #
        # PR2b (2026-09-08): self.scan_mode ("grid" or "line") is read
        # once here and reused by run()/_measure_point below, rather
        # than re-reading scan_cfg["grid"]["mode"] in three places --
        # same reason self.passes is resolved once in __init__ instead
        # of at every use site.
        self.scan_mode = self.scan_cfg["grid"].get("mode", "grid")
        hdf5_path = config["output"].get(
            "oes_hdf5",
            config["output"]["base_dir"] + "/oes.h5",
        )
        if self.scan_mode == "line":
            line_start_mm = tuple(self.scan_cfg["grid"]["line_start_mm"])
            line_end_mm = tuple(self.scan_cfg["grid"]["line_end_mm"])
            line_n_points = self.scan_cfg["grid"]["line_n_points"]
            line_points = generate_line_points(line_start_mm, line_end_mm, line_n_points)
            s_coords = np.array([s for _i, _x, _y, s in line_points], dtype="float32")
            line_x = np.array([x for _i, x, _y, _s in line_points], dtype="float32")
            line_y = np.array([y for _i, _x, y, _s in line_points], dtype="float32")
            self.store = OESStore(
                hdf5_path, mode="line",
                s_coords_mm=s_coords, line_x_mm=line_x, line_y_mm=line_y,
                start_mm=line_start_mm, end_mm=line_end_mm,
                n_passes=self.passes,
                wavelengths=self.spectrometer.wavelengths,
            )
        else:
            _, xs, ys = generate_grid(self.scan_cfg)
            self.store = OESStore(hdf5_path, mode="grid", x_coords_mm=xs, y_coords_mm=ys,
                                   n_passes=self.passes,
                                   wavelengths=self.spectrometer.wavelengths)

        self.logger = DataLogger(config, store=self.store)

    def _safe_rehome(self, context: str):
        """
        Call motion.home() only when it's actually safe to; otherwise
        resume() without re-zeroing.

        hard_home=True is idempotent: it always drives back to the same
        physical mechanical stop, so calling it again mid-session (scan
        start, periodic drift-reset) just resets accumulated open-loop
        error to zero at a known-good origin. Safe anytime — resume()
        just calls home() in this case, so behavior is unchanged.

        hard_home=False is NOT idempotent: home() just labels wherever
        the stage physically is *right now* as zero. calibrate_scan_area.py
        already called it once, before jogging to the edges, and every
        x_range_mm/y_range_mm/etc. value it wrote is relative to THAT
        origin. Calling home() again here -- at scan start, or worse,
        mid-scan during a periodic rehome -- would re-zero at wherever
        the stage happens to be at that moment (the last calibration
        edge jogged to, or some arbitrary grid point mid-scan) instead
        of restoring the calibration's origin, silently invalidating
        the whole scan rather than "resetting drift". So in this case
        we call motion.resume() instead: it marks the controller ready
        to move (needed — move_to() refuses to move until _homed is
        True in THIS process, even though the physical origin is still
        valid from the previous process's home() call) without
        re-zeroing anything. See NewportPicomotorController.resume()
        for exactly what it assumes and how to verify that assumption.
        """
        if self.config.get("motion", {}).get("hard_home", True):
            self.motion.home()
        else:
            self.logger.log_event(
                f"Soft home ({context}): resuming without re-zeroing — "
                "calling home() again here would re-zero at the current "
                "position instead of restoring the calibration's origin."
            )
            self.motion.resume()

    def run(self, on_point=None, stop_event=None, already_homed=False):
        """
        on_point: optional callback(record: dict, ix: int | None, iy: int | None)
        -> None, invoked after each point (including reference-point
        revisits) has been written to disk. ix/iy are the grid indices
        (None for reference-point revisits, which aren't part of the
        spatial grid — same convention DataLogger/OESStore already use).
        Lets a caller (e.g. a GUI) observe progress without scan_manager
        knowing anything about what's consuming it. Called from whatever
        thread run() executes on — the caller is responsible for any
        thread-safe handoff (e.g. pushing onto a queue.Queue rather than
        touching UI widgets directly from here).

        stop_event: optional threading.Event, checked once per grid point
        (between points, not mid-point). Setting it stops the scan after
        the in-flight point finishes and is written — logged as an abort,
        not as "Scan complete".

        already_homed: set True ONLY when the caller has already called
        home() on THIS SAME self.motion instance, earlier in this same
        process (e.g. calibrate_scan_area.py's combined calibrate-then-
        scan flow, right after it homes and jogs to the wafer edges).
        Skips the redundant "scan start" rehome below.

        Returns one of "completed" / "aborted" / "axis_fault" -- callers
        that only care about success/failure for a log line can ignore
        it (the CLI __main__ block below does), but a caller driving a
        UI needs to tell these apart: "aborted" was requested by the
        operator (stop_event), "axis_fault" means the scan stopped
        itself after an AxisStateUnknown fault and needs a manual
        hardware check before scanning again -- very much NOT the same
        as a normal "Scan complete".

        Why this is safe here but wasn't safe as soft-home's cross-
        process resume(): the picomotor is open-loop, so home() always
        drives the full home_steps/home_velocity distance regardless of
        whether it's already at the stop — calling it twice back-to-back
        (once in the caller's own home(), once again here) burns real
        time for zero benefit when it's the same physical session. But
        that's only true when it's genuinely the same in-memory
        controller object that was just homed, not a fresh connection in
        a new process trusting the hardware's register persisted (that
        unverified assumption is what caused the 2026-07-17 incident —
        see MEMORY carat_scanner_2026-07-17_scan_diagnosis). Do NOT set
        already_homed=True across a process boundary; only when self.motion
        was passed in already-homed via __init__'s `motion=` param.

        Periodic mid-scan rehomes (scan.rehome, if enabled) are
        unaffected by this flag — those still run normally regardless,
        since they exist to correct drift accumulated mid-scan.
        """
        self.logger.write_metadata()
        self.logger.log_event(f"Scan started ({self.passes} pass"
                               f"{'es' if self.passes != 1 else ''}), "
                               f"output dir: {self.config['output']['base_dir']}")

        if already_homed:
            self.logger.log_event(
                "Skipping scan-start rehome: motion controller was already "
                "homed earlier in this same session (combined calibrate-"
                "then-scan flow) — re-homing again would just re-drive the "
                "same physical hard stop a second time for no benefit."
            )
        else:
            self._safe_rehome("scan start")
        self.spectrometer.set_integration_time(self.oes_cfg["integration_time_us"])

        # PR2b (2026-09-08): normalize both modes to the same
        # (ix, iy, x, y, s_mm) shape so the rest of run() below (rehome/
        # reference-point cadence, stop_event check, point_id bookkeeping)
        # doesn't need to know or care which mode is active. Grid mode:
        # iy is the real grid index, s_mm is always None. Line mode: ix
        # is the line index i, iy is always None (the sentinel
        # DataLogger.write_point()/OESStore use to tell "line mode, one
        # spatial axis" apart from a grid point), s_mm is the arc-length
        # position generate_line_points() computed.
        if self.scan_mode == "line":
            grid_cfg = self.scan_cfg["grid"]
            line_points = generate_line_points(
                tuple(grid_cfg["line_start_mm"]), tuple(grid_cfg["line_end_mm"]),
                grid_cfg["line_n_points"],
            )
            points = [(i, None, x, y, s) for i, x, y, s in line_points]
        else:
            grid_points, _, _ = generate_grid(self.scan_cfg)
            points = [(ix, iy, x, y, None) for ix, iy, x, y in grid_points]
        ref_cfg = self.scan_cfg.get("reference_point", {})
        ref_enabled = ref_cfg.get("enabled", False)
        ref_every = ref_cfg.get("revisit_every_n_points", 0)
        ref_position = tuple(ref_cfg.get("position", (0.0, 0.0)))

        # Periodic re-home: the Newport picomotors are open-loop (no encoder),
        # so absolute position error accumulates with step count over a long
        # scan. Re-homing every N points resets that error to zero at known
        # intervals instead of letting it drift for the whole scan. Disabled
        # by default — opt in via scan.rehome in config.yaml.
        rehome_cfg = self.scan_cfg.get("rehome", {})
        rehome_enabled = rehome_cfg.get("enabled", False)
        rehome_every = rehome_cfg.get("every_n_points", 0)

        # point_id is a single counter spanning every pass (not reset per
        # pass) — keeps CSV point_id unique per row and keeps the
        # rehome/reference-revisit "every N points" cadence continuous
        # across pass boundaries instead of restarting each pass.
        point_id = 0
        try:
            for pass_id in range(self.passes):
                if self.passes > 1:
                    self.logger.log_event(f"Starting pass {pass_id + 1}/{self.passes}")

                for ix, iy, x, y, s_mm in points:
                    if stop_event is not None and stop_event.is_set():
                        self.logger.log_event(f"Scan aborted by operator after point {point_id} "
                                               f"(pass {pass_id + 1}/{self.passes})")
                        return "aborted"

                    self._measure_point(point_id, ix, iy, x, y, pass_id=pass_id,
                                         s_mm=s_mm, on_point=on_point)
                    point_id += 1

                    if rehome_enabled and rehome_every > 0 and point_id % rehome_every == 0:
                        self.logger.log_event(
                            f"Re-homing after point {point_id} (open-loop drift reset)"
                        )
                        self._safe_rehome(f"periodic rehome after point {point_id}")

                    if ref_enabled and ref_every > 0 and point_id % ref_every == 0:
                        self.logger.log_event(f"Revisiting reference point {ref_position} "
                                               f"after point {point_id}")
                        # Reference points don't belong to the spatial grid — skip HDF5 write.
                        #
                        # PR2b (2026-09-08): in LINE mode, still pass s_mm
                        # (NaN, "not applicable") rather than leaving it
                        # None/omitted. build_point_record() only adds the
                        # "s_mm" dict key when a value is given -- if this
                        # reference row omitted it while every REAL line
                        # point's row includes it, the two rows would have
                        # different key sets, and DataLogger._append_summary_row()
                        # recomputes the CSV fieldnames from EACH row's own
                        # keys (not fixed from the first row), so a later
                        # row with a different key set silently shifts
                        # every column after the missing one -- exactly the
                        # "misalign every column after it" bug class
                        # motion_error_detail's own comment above already
                        # guards against for a different field. NaN (not
                        # omission) keeps every line-mode row's schema
                        # identical, same fix, same reasoning.
                        ref_s_mm = float("nan") if self.scan_mode == "line" else None
                        self._measure_point(point_id, None, None,
                                            ref_position[0], ref_position[1],
                                            pass_id=pass_id, is_reference=True,
                                            s_mm=ref_s_mm, on_point=on_point)
                        point_id += 1
        except AxisStateUnknown:
            # Unlike an ordinary motion fault (flagged and continued inside
            # _measure_point), this means the last stop() couldn't even be
            # confirmed — the axis may still be physically moving. Do NOT
            # continue the loop: the next iteration's move_to() would be
            # issued onto an axis in an unverified state, which is the
            # exact "leftover distance" queuing failure this is guarding
            # against. Stop here; all points completed so far are already
            # flushed to disk (DataLogger writes per-point, not buffered).
            self.logger.log_event(
                f"Scan STOPPED after point {point_id}: axis state unknown "
                "after a motion fault. Check the hardware (mechanical "
                "binding, cabling, motion controller connection) before "
                "running again."
            )
            return "axis_fault"

        self.logger.log_event(f"Scan complete ({self.passes} pass"
                               f"{'es' if self.passes != 1 else ''}, "
                               f"{point_id} total points written)")
        self._generate_maps()
        return "completed"

    def _generate_maps(self):
        """
        Auto-generate and save the PNG summary maps (temperature,
        emissivity, dilution, OES feature/ratio maps -- see
        scan.map_plotter.generate_all_maps) into <output.base_dir>/maps/
        right after a completed scan, instead of requiring a separate
        manual `python scan/map_plotter.py` run afterward. Lives here
        (called from run(), not either caller) so both the GUI
        (gui/scan_worker.py's run_scan -> ScanManager.run()) and the
        standalone CLI (`python scan/scan_manager.py`) get this for free.

        Only called from the "completed" path, deliberately -- an
        aborted or axis-fault-stopped scan's partial data is still real
        (see DataLogger's per-point crash safety), but "regenerate maps
        automatically" was asked for completed scans specifically; a
        partial scan's maps can still be produced by hand afterward via
        `python scan/map_plotter.py` (or by calling this same
        scan.map_plotter.generate_all_maps against config.yaml), same as
        before this method existed.

        Best-effort: a plotting failure (matplotlib/backend issue, an
        empty or unexpectedly-shaped summary CSV, etc.) is logged and
        swallowed rather than re-raised. A scan that finished
        successfully must not be reported as failed just because the
        downstream PNG rendering had a problem -- scan_summary.csv and
        the HDF5 store are already safely on disk either way, and
        scan/map_plotter.py can always be re-run by hand afterward.

        Lazy import (not a module-level import in this file): matplotlib
        + pandas are real dependencies of map_plotter.py, but pulling
        them in at import time for every ScanManager use (including
        preflight-only `--check-only` runs that touch no hardware and
        produce no data) is unnecessary weight this defers until a scan
        has actually completed.
        """
        try:
            from scan.map_plotter import generate_all_maps
            generate_all_maps(self.config)
        except Exception as exc:
            self.logger.log_event(
                f"WARNING: automatic map generation failed (scan data itself is "
                f"unaffected -- re-run `python scan/map_plotter.py` by hand once "
                f"fixed): {exc}"
            )
            return

        maps_dir = _os.path.join(self.config["output"]["base_dir"], "maps")
        self.logger.log_event(f"Maps generated and saved to {maps_dir}")

    def _measure_point(self, point_id, ix, iy, x, y, pass_id=0, is_reference=False,
                      s_mm=None, on_point=None):
        limits = self.config["motion"]["soft_limits"]
        self.motion.check_limits(x, y, limits)

        motion_ok, motion_error_detail = self._move_with_retry(x, y)

        if motion_ok:
            ir_result = self._read_ir_with_retry()
            oes_result, wavelengths, intensities = self._read_oes_with_retry()
        else:
            # Position is unknown/unreliable after a failed move (axis
            # timeout, stall, comm fault) — don't trust an IR/OES reading
            # taken from wherever the mirror actually ended up. Flag the
            # point and move on rather than measuring blind or aborting
            # the whole scan (see _move_with_retry).
            ir_result = {"value": float("nan"), "emissivity": float("nan"),
                         "dilution": None, "error": True}
            oes_result = {"saturated": False, "error": True}
            wavelengths, intensities = None, None

        if intensities is not None:
            feature_values = extract_features(
                wavelengths, intensities,
                self.oes_cfg["features"], self.oes_cfg["feature_window_nm"],
            )
        else:
            feature_values = {name: float("nan") for name in self.oes_cfg["features"]}

        record = build_point_record(point_id, x, y, ir_result, oes_result, feature_values,
                                     pass_id=pass_id, s_mm=s_mm)
        record["is_reference"] = is_reference
        record["motion_error"] = not motion_ok
        # Always present (not conditional) — _append_summary_row derives the
        # CSV header from the first row's keys, so a key that only shows up
        # on later (faulted) rows would silently misalign every column after
        # it. Empty string on success keeps every row's schema identical.
        record["motion_error_detail"] = motion_error_detail or ""

        # Pass ix/iy so DataLogger can forward them to OESStore. Reference
        # points have ix=iy=None regardless of mode — DataLogger skips the
        # HDF5 write for those. Real LINE-mode points also have iy=None
        # (there's only one spatial index, i, carried in ix -- see
        # generate_line_points()/run()'s normalization above), which is
        # why DataLogger.write_point() below can't use "ix is not None and
        # iy is not None" alone to tell "real line point" apart from
        # "reference point, skip" -- it asks self.store.mode instead. See
        # that method's own comment.
        self.logger.write_point(
            record,
            wavelengths=wavelengths,
            intensities=intensities,
            ix=ix,
            iy=iy,
        )

        if on_point is not None:
            on_point(record, ix, iy)

        tag = "REF" if is_reference else "PT"
        pass_tag = f" pass={pass_id + 1}/{self.passes}" if self.passes > 1 else ""
        self.logger.log_event(
            f"[{tag}] point {point_id}{pass_tag} (x={x}, y={y}) "
            f"IR={ir_result.get('value')} err={ir_result.get('error')} "
            f"OES_err={oes_result.get('error')} sat={oes_result.get('saturated')}"
        )

    def _move_with_retry(self, x, y):
        """
        Move to (x, y) and wait for settle, retrying on a motion fault
        (axis timeout, stall, or comm error surfaced as RuntimeError by
        the real controller) instead of letting it crash the whole scan
        the way an uncaught wait_for_settle() timeout does.

        Returns (ok, error_detail) for an ordinary, recoverable motion
        fault. On failure after exhausting retries, _measure_point flags
        the point (motion_error=True, NaN readings) and the scan
        continues to the next point — safe, because the axis was
        confirmed stopped before the exception was raised (see
        MotionFault in motion_controller.py).

        Raises AxisStateUnknown instead of returning, and does NOT
        retry, if the underlying controller couldn't even confirm the
        axis actually stopped. Issuing another move_to() here would be
        exactly the "queue a new move on top of leftover motion"
        failure mode this method exists to prevent. This is deliberately
        NOT caught here — it propagates up through _measure_point to
        run(), which stops the whole scan rather than commanding this
        axis (or the other one, which shares wait_for_settle) again.
        Callers other than run() must let it propagate for the same
        reason.

        Every fault is logged with get_position() at the moment of
        failure — where the controller actually is vs. the (x, y) it was
        asked to reach. That's the data needed to tell apart the two
        likely mechanisms if this fires again:
          - reported position lands near the target -> likely a status/
            comms desync (is_moving() polling issue), not a real stall.
          - reported position is far short of the target -> axis is
            genuinely still mid-travel / lost steps, most plausible right
            after a much larger move (e.g. a reference-point revisit)
            left more real distance to cover than the next nominal grid
            step assumes. Don't discard these log lines.
        """
        max_retries = self.error_cfg["max_retries"]
        last_error = None

        for attempt in range(max_retries + 1):
            try:
                self.motion.move_to(x, y)
                self.motion.wait_for_settle(self.scan_cfg["settle_time_s"])
                return True, None
            except AxisStateUnknown as e:
                try:
                    actual = self.motion.get_position()
                except Exception:
                    actual = "unavailable"
                self.logger.log_event(
                    f"Axis state UNKNOWN moving to ({x}, {y}) after attempt "
                    f"{attempt + 1}: {e} | last reported position: {actual} | "
                    "NOT retrying, NOT continuing scan — refusing to issue "
                    "another move onto an unconfirmed axis. Manual check "
                    "required before this axis moves again."
                )
                raise
            except RuntimeError as e:
                last_error = str(e)
                try:
                    actual = self.motion.get_position()
                except Exception:
                    actual = "unavailable"
                self.logger.log_event(
                    f"Motion fault moving to ({x}, {y}) "
                    f"(attempt {attempt + 1}/{max_retries + 1}): {e} | "
                    f"position at fault: {actual}"
                )

        self.logger.log_event(
            f"Motion fault persisted after {max_retries + 1} attempts moving to "
            f"({x}, {y}) — flagging point instead of aborting scan. "
            f"Last error: {last_error}"
        )
        return False, last_error

    def _read_ir_with_retry(self):
        """
        value_c is dwell-time-averaged (read_averaged() polls for the full
        dwell window and means the valid reads — this IS the "filtered"
        temperature). emissivity and dilution are NOT averaged the same
        way: read_averaged() only means value_c and hands back the LAST
        poll's IRReading for everything else, so these two are last-value,
        not dwell-averaged. That matches how the PAC side reports them
        (an instantaneous strength/dilution reading, not an integrated one).

        Previously this discarded the returned IRReading entirely
        (`value, _ = ...`), which meant emissivity was read from the PAC
        every poll and then thrown away — never reached build_point_record,
        the CSV, or the HDF5 store. Fixed 2026-07-21.
        """
        max_retries = self.error_cfg["max_retries"]
        for attempt in range(max_retries + 1):
            try:
                value, reading = self.ir_reader.read_averaged(self.dwell_time_s)
                return {
                    "value": value,
                    "emissivity": reading.emissivity if reading is not None else float("nan"),
                    "dilution": reading.dilution if reading is not None else None,
                    "error": False,
                }
            except Exception as e:
                self.logger.log_event(f"IR read failed (attempt {attempt + 1}): {e}")

        return {"value": float("nan"), "emissivity": float("nan"), "dilution": None, "error": True}

    def _read_oes_with_retry(self):
        max_retries = self.error_cfg["max_retries"]
        for attempt in range(max_retries + 1):
            try:
                reading = self.spectrometer.read()
                if reading.error:
                    raise IOError(reading.error)
                return ({"saturated": reading.saturated, "error": False},
                        reading.wavelengths, reading.intensities)
            except Exception as e:
                self.logger.log_event(f"OES read failed (attempt {attempt + 1}): {e}")

        return {"saturated": False, "error": True}, None, None


if __name__ == "__main__":
    import argparse
    import sys

    import yaml

    parser = argparse.ArgumentParser(
        description="Run a carat_scanner scan using the settings in config.yaml."
    )
    parser.add_argument(
        "--config", default="config.yaml",
        help="Path to the config YAML to run (default: config.yaml).",
    )
    parser.add_argument(
        "--smoke-test", action="store_true",
        help=(
            "Run a fast, small-grid dev sanity check INSTEAD of the "
            "configured scan: shrinks the range to a 3x3 grid at the "
            "default step size, uses the minimum valid dwell time (2s), "
            "and writes to ./scan_data_smoketest instead of the configured "
            "output dir. Does not modify config.yaml on disk. Without this "
            "flag, `python scan/scan_manager.py` runs exactly what's in "
            "config.yaml — no silent overrides."
        ),
    )
    parser.add_argument(
        "--check-only", action="store_true",
        help=(
            "Validate every commanded grid + reference-point position "
            "against motion.soft_limits and exit — 0 if all are in "
            "bounds, 1 (with every violation listed) if not. Touches NO "
            "hardware: no motion controller, IR reader, or spectrometer "
            "connection is opened, and nothing is homed or moved. Fast "
            "way to sanity-check a config after editing soft_limits, "
            "step_size_mm, or re-running calibrate_scan_area.py, before "
            "trusting it on real hardware. Combine with --smoke-test to "
            "check the shrunk smoke-test grid instead of the real one."
        ),
    )
    args = parser.parse_args()

    # utf-8-sig: see run_gui.py's copy of this comment -- tolerates/strips
    # a UTF-8 BOM (e.g. from editing config.yaml in Notepad on Windows),
    # no-op if absent. Matches gui/calibration_panel.py and
    # scan/calibrate_scan_area.py's own config.yaml reads.
    with open(args.config, encoding="utf-8-sig") as f:
        config = yaml.safe_load(f)

    if args.smoke_test:
        # Dev-only override, opt-in via --smoke-test. dwell_time_s is kept
        # at its validated minimum (2.0s) rather than bypassing validation
        # entirely — smoke test runs a bit slower (~2s/point) but exercises
        # the real, enforced bounds instead of a fake fast value.
        config["scan"]["grid"]["x_range_mm"] = [0, 4]
        config["scan"]["grid"]["y_range_mm"] = [0, 4]
        config["scan"]["grid"]["step_size_mm"] = 2.0
        # Force no circular mask for the smoke test regardless of what's
        # calibrated in the real config — wafer_center_mm there could be
        # far outside this shrunk 4x4 test box, which would mask out
        # every point and silently "succeed" at measuring nothing.
        config["scan"]["grid"]["wafer_radius_mm"] = None
        config["scan"]["dwell_time_s"] = 2.0
        config["scan"]["settle_time_s"] = 0.0
        config["scan"]["reference_point"]["enabled"] = True
        config["scan"]["reference_point"]["revisit_every_n_points"] = 4
        config["output"]["base_dir"] = "./scan_data_smoketest"

    if args.check_only:
        try:
            commanded = preflight_check(config)
        except ValueError as e:
            print(f"FAIL: {e}")
            sys.exit(1)
        print(f"OK: all {len(commanded)} commanded position(s) "
              "(grid + reference point) are within motion.soft_limits. "
              "No hardware was touched.")
        sys.exit(0)

    manager = ScanManager(config)
    manager.run()
