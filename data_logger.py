"""
data_logger.py

Writes CSV summaries, per-point spectra files, metadata files,
and error logs. Designed to be crash-safe: each point's data is
written to disk immediately (append-only), so a scan that dies
partway through preserves all completed points.

KNOWN REDUNDANCY (intentional for now, flagged 2026-07-11 code review):
ScanManager always builds an OESStore, so in practice every spectrum
is written BOTH to a per-point CSV (spectra/point_XXXXX.csv, via
_write_spectrum) AND into the HDF5 store (write_point below) — not
"instead of," always "in addition to," despite what the paragraph
below might imply. That's 2x disk I/O and 2x storage for the same
data on every point of every scan. Deliberately left as-is for now;
HDF5 is almost certainly the one to keep long-term (it preserves the
full x/y/wavelength grid and loads straight into xarray — see
oes_store.py), so when this gets revisited, drop the per-point CSVs
rather than the HDF5 store, once nothing downstream still reads them.
"""

import copy
import csv
import os
import time
from datetime import datetime
from typing import Optional

import numpy as np
import yaml

# Substrings (case-insensitive) that mark a config key as sensitive.
# Any dict key containing one of these has its value redacted before
# the config is echoed into metadata.yaml.
_SECRET_KEY_MARKERS = ("key", "token", "secret", "password")


def _redact_secrets(obj):
    """
    Recursively deep-copy a config dict, replacing the value of any
    key that looks like a credential (matches _SECRET_KEY_MARKERS)
    with "***REDACTED***".

    write_metadata() embeds the full run config into metadata.yaml for
    provenance. Without this, credentials like ir.pac.api_key_value get
    written in plaintext into every scan's output directory.
    """
    if isinstance(obj, dict):
        redacted = {}
        for key, value in obj.items():
            if isinstance(key, str) and any(marker in key.lower() for marker in _SECRET_KEY_MARKERS):
                redacted[key] = "***REDACTED***" if value else value
            else:
                redacted[key] = _redact_secrets(value)
        return redacted
    if isinstance(obj, list):
        return [_redact_secrets(item) for item in obj]
    return copy.deepcopy(obj)


class DataLogger:
    def __init__(self, config, store=None):
        """
        Parameters
        ----------
        config : dict
            Scan configuration (standard shape — see config.yaml).
        store : OESStore, optional
            If provided, full spectra are written to HDF5 via
            store.write_point() in addition to the summary CSV.
            Pass ix and iy to write_point() when using this.
        """
        self.config = config
        self.store = store  # OESStore instance, or None
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
        metadata["config"] = _redact_secrets(self.config)
        if extra_metadata:
            metadata.update(extra_metadata)

        with open(self.metadata_path, "w") as f:
            yaml.safe_dump(metadata, f, default_flow_style=False)

    def write_point(self, point_record, wavelengths=None, intensities=None,
                    ix: Optional[int] = None, iy: Optional[int] = None):
        """
        Append one point's summary row to the CSV, write its spectrum
        to a per-point CSV file, and (if an OESStore is attached) write
        the full spectrum into the HDF5 file.

        point_record: dict with keys such as
            point_id, pass_id, x_mm, y_mm, ir_temp_c, ir_error,
            oes_error, oes_saturated, feature_<name> values, timestamp
        ix, iy : int, optional
            Grid indices required when an OESStore is attached.
            Ignored (and not needed) for pure-CSV operation.
        """
        self._append_summary_row(point_record)

        if wavelengths is not None and intensities is not None:
            self._write_spectrum(point_record["point_id"], wavelengths, intensities)

        # HDF5 write — only if a store is wired up and grid indices are known
        if self.store is not None and ix is not None and iy is not None:
            self.store.write_point(
                ix=ix,
                iy=iy,
                pass_id=point_record.get("pass_id", 0),
                wavelengths=wavelengths,
                intensities=intensities,
                ir_temp_c=point_record.get("ir_temp_c"),
                timestamp=time.time(),
                saturated=bool(point_record.get("oes_saturated", False)),
                ir_error=bool(point_record.get("ir_error", False)),
                oes_error=bool(point_record.get("oes_error", False)),
            )

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


def build_point_record(point_id, x_mm, y_mm, ir_result, oes_result, feature_values, pass_id=0):
    """
    Helper to assemble a flat dict for one scan point, suitable for
    DataLogger.write_point().

    ir_result: dict with keys 'value', 'error' (bool)
    oes_result: dict with keys 'saturated', 'error' (bool)
    feature_values: dict of feature_name -> intensity
    pass_id: 0-based index of which full-grid pass this point belongs to
        (see scan.passes in config.yaml). 0 for single-pass scans, so
        existing single-pass CSVs/callers are unaffected.
    """
    record = {
        "point_id": point_id,
        "pass_id": pass_id,
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
