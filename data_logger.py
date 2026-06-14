"""
data_logger.py

Writes CSV summaries, per-point spectra files, metadata files,
and error logs. Designed to be crash-safe: each point's data is
written to disk immediately (append-only), so a scan that dies
partway through preserves all completed points.
"""

import csv
import os
import time
from datetime import datetime

import numpy as np
import yaml


class DataLogger:
    def __init__(self, config):
        self.config = config
        out_cfg = config["output"]

        self.base_dir = out_cfg["base_dir"]
        self.spectra_dir = os.path.join(self.base_dir, out_cfg["spectra_subdir"])
        self.summary_path = os.path.join(self.base_dir, out_cfg["summary_csv"])
        self.metadata_path = os.path.join(self.base_dir, out_cfg["metadata_file"])
        self.log_path = os.path.join(self.base_dir, out_cfg["log_file"])

        self._summary_initialized = False

        os.makedirs(self.base_dir, exist_ok=True)
        os.makedirs(self.spectra_dir, exist_ok=True)

    def log_event(self, message):
        """Append a timestamped message to the runtime log."""
        timestamp = datetime.now().isoformat()
        line = f"{timestamp}\t{message}"
        print(line)
        with open(self.log_path, "a") as f:
            f.write(line + "\n")

    def write_metadata(self, extra_metadata=None):
        """Write the scan metadata file (config + extra info)."""
        metadata = dict(self.config.get("metadata", {}))
        metadata["scan_started"] = datetime.now().isoformat()
        metadata["config"] = self.config
        if extra_metadata:
            metadata.update(extra_metadata)

        with open(self.metadata_path, "w") as f:
            yaml.safe_dump(metadata, f, default_flow_style=False)

    def write_point(self, point_record, wavelengths=None, intensities=None):
        """
        Append one point's summary row to the CSV, and (if provided)
        write its full spectrum to a separate CSV file.

        point_record: dict with keys such as
            point_id, x_mm, y_mm, ir_temp_c, ir_error,
            oes_error, oes_saturated, feature_<name> values, timestamp
        """
        self._append_summary_row(point_record)

        if wavelengths is not None and intensities is not None:
            self._write_spectrum(point_record["point_id"], wavelengths, intensities)

    def _append_summary_row(self, point_record):
        fieldnames = list(point_record.keys())

        write_header = not self._summary_initialized and not os.path.exists(self.summary_path)

        with open(self.summary_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerow(point_record)

        self._summary_initialized = True

    def _write_spectrum(self, point_id, wavelengths, intensities):
        filename = os.path.join(self.spectra_dir, f"point_{point_id:05d}.csv")
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["wavelength_nm", "intensity"])
            for wl, intens in zip(wavelengths, intensities):
                writer.writerow([wl, intens])


def build_point_record(point_id, x_mm, y_mm, ir_result, oes_result, feature_values):
    """
    Helper to assemble a flat dict for one scan point, suitable for
    DataLogger.write_point().

    ir_result: dict with keys 'value', 'error' (bool)
    oes_result: dict with keys 'saturated', 'error' (bool)
    feature_values: dict of feature_name -> intensity
    """
    record = {
        "point_id": point_id,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "timestamp": datetime.now().isoformat(),
        "ir_temp_c": ir_result.get("value", float("nan")),
        "ir_error": ir_result.get("error", False),
        "oes_saturated": oes_result.get("saturated", False),
        "oes_error": oes_result.get("error", False),
    }

    for name, value in feature_values.items():
        record[f"feature_{name}"] = value

    return record


if __name__ == "__main__":
    # Smoke test with mock config
    test_config = {
        "output": {
            "base_dir": "./scan_data_test",
            "spectra_subdir": "spectra",
            "summary_csv": "scan_summary.csv",
            "metadata_file": "metadata.yaml",
            "log_file": "scan_log.txt",
        },
        "metadata": {"operator": "test"},
    }

    logger = DataLogger(test_config)
    logger.write_metadata()
    logger.log_event("Test scan started")

    wl = np.linspace(350, 750, 100)
    intens = np.random.rand(100) * 1000

    record = build_point_record(
        point_id=1,
        x_mm=0.0,
        y_mm=0.0,
        ir_result={"value": 950.2, "error": False},
        oes_result={"saturated": False, "error": False},
        feature_values={"CH": 1500.0, "C2_Swan": 800.0, "H_alpha": 2200.0, "H_beta": 600.0},
    )
    logger.write_point(record, wavelengths=wl, intensities=intens)
    logger.log_event("Test scan complete")
