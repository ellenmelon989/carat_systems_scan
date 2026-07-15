"""
scan_manager.py

Coordinates the scan sequence: motion, IR acquisition, OES
acquisition, and data logging. Sequential, single-threaded by
design (plasma drift timescale ~minutes makes async unnecessary).

Implements:
- Raster/serpentine 2D grid generation
- Per-point error policy (retry -> NaN + flag -> continue)
- Periodic revisit of a fixed reference point for drift tracking
"""
import numpy as np

from motion.motion_controller import get_motion_controller
from readers.ir_reader_base import get_ir_reader
from readers.spectrometer_reader_base import get_spectrometer_reader
from data_logger import DataLogger, build_point_record
from oes_store import OESStore
from scan_params import grid_dims_from_range, validate_dwell_time_s


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
    """
    x0, x1 = scan_cfg["grid"]["x_range_mm"]
    y0, y1 = scan_cfg["grid"]["y_range_mm"]
    step_size_mm = scan_cfg["grid"]["step_size_mm"]

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
            points.append((ix, iy, float(x), float(y)))

    return points, xs, ys


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
    def __init__(self, config):
        self.config = config
        self.motion = get_motion_controller(config)
        self.ir_reader = get_ir_reader(config)
        self.spectrometer = get_spectrometer_reader(config)

        self.scan_cfg = config["scan"]
        self.oes_cfg = config["oes"]
        self.error_cfg = config["error_policy"]

        # dwell_time_s is the single operator-facing per-point IR averaging
        # duration (replaces the old ir.averaging_time_s config key — see
        # scan_params.py for the valid range).
        self.dwell_time_s = validate_dwell_time_s(self.scan_cfg["dwell_time_s"])

        # Build OESStore from grid coords so it's ready before the scan starts.
        # Wavelength dimension is initialized lazily on first write_point().
        _, xs, ys = generate_grid(self.scan_cfg)
        hdf5_path = config["output"].get(
            "oes_hdf5",
            config["output"]["base_dir"] + "/oes.h5",
        )
        self.store = OESStore(hdf5_path, x_coords_mm=xs, y_coords_mm=ys)

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

    def run(self, on_point=None, stop_event=None):
        """
        on_point: optional callback(record: dict) -> None, invoked after each
        point (including reference-point revisits) has been written to disk.
        Lets a caller (e.g. a GUI) observe progress without scan_manager
        knowing anything about what's consuming it. Called from whatever
        thread run() executes on — the caller is responsible for any
        thread-safe handoff (e.g. pushing onto a queue.Queue rather than
        touching UI widgets directly from here).

        stop_event: optional threading.Event, checked once per grid point
        (between points, not mid-point). Setting it stops the scan after
        the in-flight point finishes and is written — logged as an abort,
        not as "Scan complete".
        """
        self.logger.write_metadata()
        self.logger.log_event("Scan started")

        self._safe_rehome("scan start")
        self.spectrometer.set_integration_time(self.oes_cfg["integration_time_us"])

        points, _, _ = generate_grid(self.scan_cfg)
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

        point_id = 0
        for ix, iy, x, y in points:
            if stop_event is not None and stop_event.is_set():
                self.logger.log_event(f"Scan aborted by operator after point {point_id}")
                return

            self._measure_point(point_id, ix, iy, x, y, on_point=on_point)
            point_id += 1

            if rehome_enabled and rehome_every > 0 and point_id % rehome_every == 0:
                self.logger.log_event(
                    f"Re-homing after point {point_id} (open-loop drift reset)"
                )
                self._safe_rehome(f"periodic rehome after point {point_id}")

            if ref_enabled and ref_every > 0 and point_id % ref_every == 0:
                self.logger.log_event(f"Revisiting reference point {ref_position} "
                                       f"after point {point_id}")
                # Reference points don't belong to the spatial grid — skip HDF5 write
                self._measure_point(point_id, None, None,
                                    ref_position[0], ref_position[1],
                                    is_reference=True, on_point=on_point)
                point_id += 1

        self.logger.log_event("Scan complete")

    def _measure_point(self, point_id, ix, iy, x, y, is_reference=False, on_point=None):
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
            ir_result = {"value": float("nan"), "error": True}
            oes_result = {"saturated": False, "error": True}
            wavelengths, intensities = None, None

        if intensities is not None:
            feature_values = extract_features(
                wavelengths, intensities,
                self.oes_cfg["features"], self.oes_cfg["feature_window_nm"],
            )
        else:
            feature_values = {name: float("nan") for name in self.oes_cfg["features"]}

        record = build_point_record(point_id, x, y, ir_result, oes_result, feature_values)
        record["is_reference"] = is_reference
        record["motion_error"] = not motion_ok
        # Always present (not conditional) — _append_summary_row derives the
        # CSV header from the first row's keys, so a key that only shows up
        # on later (faulted) rows would silently misalign every column after
        # it. Empty string on success keeps every row's schema identical.
        record["motion_error_detail"] = motion_error_detail or ""

        # Pass ix/iy so DataLogger can forward them to OESStore.
        # Reference points have ix=iy=None — DataLogger skips the HDF5 write.
        self.logger.write_point(
            record,
            wavelengths=wavelengths,
            intensities=intensities,
            ix=ix,
            iy=iy,
        )

        if on_point is not None:
            on_point(record)

        tag = "REF" if is_reference else "PT"
        self.logger.log_event(
            f"[{tag}] point {point_id} (x={x}, y={y}) "
            f"IR={ir_result.get('value')} err={ir_result.get('error')} "
            f"OES_err={oes_result.get('error')} sat={oes_result.get('saturated')}"
        )

    def _move_with_retry(self, x, y):
        """
        Move to (x, y) and wait for settle, retrying on a motion fault
        (axis timeout, stall, or comm error surfaced as RuntimeError by
        the real controller) instead of letting it crash the whole scan
        the way an uncaught wait_for_settle() timeout does.

        Returns (ok, error_detail). On failure after exhausting retries,
        _measure_point flags the point (motion_error=True, NaN readings)
        and the scan continues to the next point.

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
        max_retries = self.error_cfg["max_retries"]
        for attempt in range(max_retries + 1):
            try:
                value, _ = self.ir_reader.read_averaged(self.dwell_time_s)
                return {"value": value, "error": False}
            except Exception as e:
                self.logger.log_event(f"IR read failed (attempt {attempt + 1}): {e}")

        return {"value": float("nan"), "error": True}

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
            "flag, `python scan_manager.py` runs exactly what's in "
            "config.yaml — no silent overrides."
        ),
    )
    args = parser.parse_args()

    with open(args.config) as f:
        config = yaml.safe_load(f)

    if args.smoke_test:
        # Dev-only override, opt-in via --smoke-test. dwell_time_s is kept
        # at its validated minimum (2.0s) rather than bypassing validation
        # entirely — smoke test runs a bit slower (~2s/point) but exercises
        # the real, enforced bounds instead of a fake fast value.
        config["scan"]["grid"]["x_range_mm"] = [0, 4]
        config["scan"]["grid"]["y_range_mm"] = [0, 4]
        config["scan"]["grid"]["step_size_mm"] = 2.0
        config["scan"]["dwell_time_s"] = 2.0
        config["scan"]["settle_time_s"] = 0.0
        config["scan"]["reference_point"]["enabled"] = True
        config["scan"]["reference_point"]["revisit_every_n_points"] = 4
        config["output"]["base_dir"] = "./scan_data_smoketest"

    manager = ScanManager(config)
    manager.run()
