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


def generate_grid(scan_cfg):
    """
    Generate a list of (ix, iy, x_mm, y_mm) points based on config,
    in raster or serpentine order.

    ix, iy are 0-based grid indices used to address the HDF5 dataset;
    x_mm, y_mm are the physical positions in millimetres.
    """
    nx = scan_cfg["grid"]["nx"]
    ny = scan_cfg["grid"]["ny"]
    x0, x1 = scan_cfg["grid"]["x_range_mm"]
    y0, y1 = scan_cfg["grid"]["y_range_mm"]

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
        self.ir_cfg = config["ir"]
        self.oes_cfg = config["oes"]
        self.error_cfg = config["error_policy"]

        # Build OESStore from grid coords so it's ready before the scan starts.
        # Wavelength dimension is initialized lazily on first write_point().
        _, xs, ys = generate_grid(self.scan_cfg)
        hdf5_path = config["output"].get(
            "oes_hdf5",
            config["output"]["base_dir"] + "/oes.h5",
        )
        self.store = OESStore(hdf5_path, x_coords_mm=xs, y_coords_mm=ys)

        self.logger = DataLogger(config, store=self.store)

    def run(self):
        self.logger.write_metadata()
        self.logger.log_event("Scan started")

        self.motion.home()
        self.spectrometer.set_integration_time(self.oes_cfg["integration_time_us"])

        points, _, _ = generate_grid(self.scan_cfg)
        ref_cfg = self.scan_cfg.get("reference_point", {})
        ref_enabled = ref_cfg.get("enabled", False)
        ref_every = ref_cfg.get("revisit_every_n_points", 0)
        ref_position = tuple(ref_cfg.get("position", (0.0, 0.0)))

        point_id = 0
        for ix, iy, x, y in points:
            self._measure_point(point_id, ix, iy, x, y)
            point_id += 1

            if ref_enabled and ref_every > 0 and point_id % ref_every == 0:
                self.logger.log_event(f"Revisiting reference point {ref_position} "
                                       f"after point {point_id}")
                # Reference points don't belong to the spatial grid — skip HDF5 write
                self._measure_point(point_id, None, None,
                                    ref_position[0], ref_position[1], is_reference=True)
                point_id += 1

        self.logger.log_event("Scan complete")

    def _measure_point(self, point_id, ix, iy, x, y, is_reference=False):
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

        # Pass ix/iy so DataLogger can forward them to OESStore.
        # Reference points have ix=iy=None — DataLogger skips the HDF5 write.
        self.logger.write_point(
            record,
            wavelengths=wavelengths,
            intensities=intensities,
            ix=ix,
            iy=iy,
        )

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
                reading = self.spectrometer.read()
                if reading.error:
                    raise IOError(reading.error)
                return ({"saturated": reading.saturated, "error": False},
                        reading.wavelengths, reading.intensities)
            except Exception as e:
                self.logger.log_event(f"OES read failed (attempt {attempt + 1}): {e}")

        return {"saturated": False, "error": True}, None, None


if __name__ == "__main__":
    import yaml

    with open("config.yaml") as f:
        config = yaml.safe_load(f)

    # Fast params for smoke test — override slow real-scan values
    config["scan"]["grid"]["nx"] = 3
    config["scan"]["grid"]["ny"] = 3
    config["scan"]["settle_time_s"] = 0.0
    config["scan"]["reference_point"]["enabled"] = True
    config["scan"]["reference_point"]["revisit_every_n_points"] = 4
    config["ir"]["averaging_time_s"] = 0.2   # 5s → 0.2s for smoke test
    config["output"]["base_dir"] = "./scan_data_smoketest"

    manager = ScanManager(config)
    manager.run()
