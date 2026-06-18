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
from spectrometer_reader import get_spectrometer_reader
import numpy as np

from motion_controller import get_motion_controller
from ir_reader import get_ir_reader
from data_logger import DataLogger, build_point_record


def generate_grid(scan_cfg):
    """
    Generate a list of (x_mm, y_mm) points based on config,
    in raster or serpentine order.
    """
    nx = scan_cfg["grid"]["nx"]
    ny = scan_cfg["grid"]["ny"]
    x0, x1 = scan_cfg["grid"]["x_range_mm"]
    y0, y1 = scan_cfg["grid"]["y_range_mm"]

    xs = np.linspace(x0, x1, nx)
    ys = np.linspace(y0, y1, ny)

    order = scan_cfg.get("scan_order", "raster")
    points = []

    for j, y in enumerate(ys):
        row_xs = xs
        if order == "serpentine" and j % 2 == 1:
            row_xs = xs[::-1]
        for x in row_xs:
            points.append((float(x), float(y)))

    return points


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
        self.logger = DataLogger(config)

        self.scan_cfg = config["scan"]
        self.ir_cfg = config["ir"]
        self.oes_cfg = config["oes"]
        self.error_cfg = config["error_policy"]

    def run(self):
        self.logger.write_metadata()
        self.logger.log_event("Scan started")

        self.motion.home()
        self.spectrometer.set_integration_time(self.oes_cfg["integration_time_us"])

        points = generate_grid(self.scan_cfg)
        ref_cfg = self.scan_cfg.get("reference_point", {})
        ref_enabled = ref_cfg.get("enabled", False)
        ref_every = ref_cfg.get("revisit_every_n_points", 0)
        ref_position = tuple(ref_cfg.get("position", (0.0, 0.0)))

        point_id = 0
        for x, y in points:
            self._measure_point(point_id, x, y)
            point_id += 1

            if ref_enabled and ref_every > 0 and point_id % ref_every == 0:
                self.logger.log_event(f"Revisiting reference point {ref_position} "
                                       f"after point {point_id}")
                self._measure_point(point_id, ref_position[0], ref_position[1], is_reference=True)
                point_id += 1

        self.logger.log_event("Scan complete")

    def _measure_point(self, point_id, x, y, is_reference=False):
        limits = self.config["motion"]["soft_limits"]
        self.motion.check_limits(x, y, limits)

        self.motion.move_to(x, y)
        self.motion.wait_for_settle(self.scan_cfg["settle_time_s"])

        ir_result = self._read_ir_with_retry()
        oes_result, wavelengths, intensities = self._read_oes_with_retry()

        if intensities is not None:
            feature_values = extract_features(
                wavelengths, intensities,
                self.oes_cfg["features"], self.oes_cfg["feature_window_nm"],
            )
        else:
            feature_values = {name: float("nan") for name in self.oes_cfg["features"]}

        record = build_point_record(point_id, x, y, ir_result, oes_result, feature_values)
        record["is_reference"] = is_reference

        self.logger.write_point(record, wavelengths=wavelengths, intensities=intensities)

        tag = "REF" if is_reference else "PT"
        self.logger.log_event(
            f"[{tag}] point {point_id} (x={x}, y={y}) "
            f"IR={ir_result.get('value')} err={ir_result.get('error')} "
            f"OES_err={oes_result.get('error')} sat={oes_result.get('saturated')}"
        )

    def _read_ir_with_retry(self):
        max_retries = self.error_cfg["max_retries"]
        for attempt in range(max_retries + 1):
            try:
                value, _ = self.ir_reader.read_averaged(self.ir_cfg["averaging_time_s"])
                return {"value": value, "error": False}
            except Exception as e:
                self.logger.log_event(f"IR read failed (attempt {attempt + 1}): {e}")

        return {"value": float("nan"), "error": True}

    def _read_oes_with_retry(self):
        max_retries = self.error_cfg["max_retries"]
        for attempt in range(max_retries + 1):
            try:
                wl, intens, saturated = self.spectrometer.acquire_averaged(
                    num_averages=self.oes_cfg["num_averages"],
                    boxcar_width=self.oes_cfg["boxcar_width"],
                )
                return {"saturated": saturated, "error": False}, wl, intens
            except Exception as e:
                self.logger.log_event(f"OES read failed (attempt {attempt + 1}): {e}")

        return {"saturated": False, "error": True}, None, None


if __name__ == "__main__":
    import yaml

    with open("config.yaml") as f:
        config = yaml.safe_load(f)

    # Use a tiny grid for a quick smoke test
    config["scan"]["grid"]["nx"] = 3
    config["scan"]["grid"]["ny"] = 3
    config["scan"]["reference_point"]["revisit_every_n_points"] = 4
    config["output"]["base_dir"] = "./scan_data_smoketest"

    manager = ScanManager(config)
    manager.run()
