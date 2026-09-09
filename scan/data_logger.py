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

# --- repo-root import bootstrap -------------------------------------------
# Added PR2b (2026-09-08): this module's __main__ self-check now exercises
# a local `from scan.oes_store import OESStore` (to build a line-mode store
# for the new smoke-test block), so `python scan/data_logger.py` needs the
# repo root on sys.path the same way scan_params.py/scan_manager.py already
# do for themselves -- see scan_params.py's own copy of this comment for
# the full rationale. Harmless as an import-time no-op for every OTHER
# caller of this module (scan_manager.py, gui/), since they already put the
# repo root on sys.path before importing data_logger.
import os as _os
import sys as _sys

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)
# ---------------------------------------------------------------------------

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


def resolve_run_dir(base_dir, when=None):
    """
    Append a scan_<YYYYMMDD_HHMMSS> subfolder to base_dir so every scan
    run gets its own uniquely dated/timestamped output directory
    automatically -- the operator's output.base_dir field (config.yaml,
    or the GUI's "Output dir" box) is treated as a PARENT location to
    file runs under, not the run's own directory.

    Call this once, before base_dir is used to derive anything else
    (see ScanManager.__init__, which calls this before computing the
    default oes.h5 path) -- so the HDF5 store and DataLogger's
    CSV/metadata/log/spectra all resolve against the identical,
    already-timestamped path rather than drifting apart if resolved
    twice a few milliseconds apart.

    This also closes the "silent output-dir-reuse corruption" gap the
    WARNING in DataLogger.__init__ below guards against: reusing the
    same static base_dir across two scans used to append a second run's
    rows under the first run's header, with point_id restarting from 0.
    Every run now gets a fresh directory by construction, so that
    collision can no longer happen from an unchanged output.base_dir
    alone. (The warning stays as a backstop for the case where an
    operator manually points two different runs at the exact same
    already-timestamped path, or calls DataLogger directly without
    going through this function.)

    when : datetime, optional
        Defaults to datetime.now(). Exposed for tests that need a
        deterministic directory name.
    """
    timestamp = (when or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return os.path.join(base_dir, f"scan_{timestamp}")


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

        # Loud, not silent: reusing an output.base_dir from an earlier scan
        # means _append_summary_row below opens summary_path in "a"
        # (append) mode -- write_header only fires when the file doesn't
        # already exist, so a pre-existing summary CSV silently gets a
        # SECOND scan's rows appended under the FIRST scan's header, with
        # point_id restarting from 0 and no column-count check between the
        # two runs. That's silent data corruption for anything downstream
        # (map_plotter.py, manual analysis) that assumes one CSV == one
        # scan. This can't be fixed by refusing to run (a genuine
        # multi-session append might be intentional), so at minimum make
        # it visible: warn once, here, before anything is written.
        #
        # ScanManager.__init__ runs every base_dir through resolve_run_dir()
        # above before constructing this class, which appends a fresh
        # scan_<timestamp> subfolder per run -- so in the normal app/CLI
        # flow this warning should be rare (only fires if two runs
        # started in the same second, or if DataLogger is constructed
        # directly without going through resolve_run_dir first).
        if os.path.exists(self.summary_path):
            # self.log_event (not print): writes to BOTH stdout and
            # scan_log.txt in this same base_dir, so the warning survives
            # in the record even if nobody was watching the console when
            # the scan started.
            self.log_event(
                f"WARNING: {self.summary_path} already exists — new rows will be "
                "APPENDED to it (point_id restarts from 0), not written to a fresh "
                "file. If this is a different scan than whatever wrote the existing "
                "rows, point a different output.base_dir at it first or move/rename "
                "the existing scan_data directory."
            )

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
            point_id, pass_id, x_mm, y_mm, ir_temp_c, ir_emissivity,
            ir_dilution, ir_error, oes_error, oes_saturated,
            feature_<name> values, timestamp
        ix, iy : int, optional
            Grid indices required when an OESStore is attached.
            Ignored (and not needed) for pure-CSV operation.
        """
        self._append_summary_row(point_record)

        if wavelengths is not None and intensities is not None:
            self._write_spectrum(point_record["point_id"], wavelengths, intensities)

        # HDF5 write — only if a store is wired up and this is a real
        # spatial point, not a reference-point revisit.
        #
        # PR2b (2026-09-08): a real LINE-mode point has ix set and iy
        # ALWAYS None (there's only one spatial index in line mode — see
        # scan_manager.generate_line_points()/OESStore's line-mode
        # schema), which is indistinguishable from a reference-point
        # revisit (also ix=iy=None... except ix is None there too) by
        # "ix is not None and iy is not None" alone once iy is allowed to
        # be None on purpose. Ask self.store.mode instead: line mode only
        # needs ix; grid mode still needs both, exactly as before.
        if self.store is not None and ix is not None and (
            iy is not None or self.store.mode == "line"
        ):
            self.store.write_point(
                ix=ix,
                iy=iy,
                pass_id=point_record.get("pass_id", 0),
                wavelengths=wavelengths,
                intensities=intensities,
                ir_temp_c=point_record.get("ir_temp_c"),
                ir_emissivity=point_record.get("ir_emissivity"),
                ir_dilution=point_record.get("ir_dilution"),
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


def build_point_record(point_id, x_mm, y_mm, ir_result, oes_result, feature_values,
                        pass_id=0, s_mm=None):
    """
    Helper to assemble a flat dict for one scan point, suitable for
    DataLogger.write_point().

    ir_result: dict with keys 'value', 'error' (bool)
    oes_result: dict with keys 'saturated', 'error' (bool)
    feature_values: dict of feature_name -> intensity
    pass_id: 0-based index of which full-grid pass this point belongs to
        (see scan.passes in config.yaml). 0 for single-pass scans, so
        existing single-pass CSVs/callers are unaffected.
    s_mm: PR2b (2026-09-08), LINE MODE only. Arc length along the line
        from its start point (see scan_manager.generate_line_points()).
        None (the default) for grid-mode scans and reference-point
        revisits — the "s_mm" key is only added to the record at all
        when a real value is given, so grid-mode CSVs/HDF5 records are
        byte-for-byte unaffected by this parameter's existence. x_mm/
        y_mm ALWAYS mean real physical millimetres regardless of mode —
        s_mm is additive, never a replacement for them (see
        oes_store.py's module docstring for why line mode doesn't just
        repurpose x_mm/y_mm for this instead).
    """
    record = {
        "point_id": point_id,
        "pass_id": pass_id,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "timestamp": datetime.now().isoformat(),
        "ir_temp_c": ir_result.get("value", float("nan")),
        # Last-poll value, not dwell-averaged like ir_temp_c — see
        # ScanManager._read_ir_with_retry()'s docstring.
        "ir_emissivity": ir_result.get("emissivity", float("nan")),
        # None until ir.pac.dilution_tag_name is confirmed and set in
        # config.yaml (see tools/list_pac_strategy_vars.py) — written as
        # NaN in that case so the CSV column stays numeric/consistent
        # rather than mixing None and floats.
        "ir_dilution": ir_result.get("dilution") if ir_result.get("dilution") is not None else float("nan"),
        "ir_error": ir_result.get("error", False),
        "oes_saturated": oes_result.get("saturated", False),
        "oes_error": oes_result.get("error", False),
    }

    for name, value in feature_values.items():
        record[f"feature_{name}"] = value

    if s_mm is not None:
        record["s_mm"] = s_mm

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
        ir_result={"value": 950.2, "emissivity": 0.85, "dilution": 1.0, "error": False},
        oes_result={"saturated": False, "error": False},
        feature_values={"CH": 1500.0, "C2_Swan": 800.0, "H_alpha": 2200.0, "H_beta": 600.0},
    )
    logger.write_point(record, wavelengths=wl, intensities=intens)
    logger.log_event("Test scan complete")

    # ------------------------------------------------------------------
    # PR2b (2026-09-08): line mode -- a real point (has s_mm) written
    # first, then a reference-point-style revisit (ix=iy=None, no s_mm)
    # written second, confirming _append_summary_row's "header comes
    # from the first row's keys" convention doesn't break when a LATER
    # row is missing a key the header has (csv.DictWriter fills blank
    # for that column rather than raising) -- and that this is safe
    # specifically because scan_manager.run() always measures point_id=0
    # as a real point before any reference-point revisit can fire (see
    # that method's ref_every gating), so a line-mode CSV's header
    # always includes s_mm to begin with.
    from scan.oes_store import OESStore

    # Unique per run (not a fixed "./scan_data_test_line") -- this repo's
    # device-bridge sandbox can't delete files (no rm permission on the
    # mounted folder), so a fixed dir would accumulate stale APPENDED
    # rows across repeated `python scan/data_logger.py` runs and break
    # the exact-row-count assertions below on the second run onward.
    # resolve_run_dir() (already used by ScanManager for the exact same
    # "never reuse an output dir" reason) gives each run its own
    # timestamped folder for free.
    line_test_dir = resolve_run_dir("./scan_data_test_line")
    line_test_config = {
        "output": {
            "base_dir": line_test_dir,
            "spectra_subdir": "spectra",
            "summary_csv": "scan_summary.csv",
            "metadata_file": "metadata.yaml",
            "log_file": "scan_log.txt",
        },
        "metadata": {"operator": "test"},
    }

    line_store = OESStore(
        os.path.join(line_test_dir, "oes.h5"), mode="line",
        s_coords_mm=[0.0, 10.0, 20.0], line_x_mm=[0.0, 6.0, 12.0],
        line_y_mm=[0.0, 8.0, 16.0], start_mm=(0.0, 0.0), end_mm=(12.0, 16.0),
        n_passes=1,
    )
    line_logger = DataLogger(line_test_config, store=line_store)
    line_logger.write_metadata()

    # Mirrors EXACTLY what ScanManager._measure_point() does to every
    # record regardless of mode -- build_point_record() then
    # record["is_reference"] = ... unconditionally -- so this test
    # exercises the real key-set shape, not a simplified one. The point
    # of this test is the PR2b fix in scan_manager.py's reference-point
    # revisit call: it now always passes s_mm (NaN for a reference
    # point in line mode, never omitted) specifically so every row in a
    # line-mode scan has an IDENTICAL key set -- otherwise
    # DataLogger._append_summary_row() (which recomputes CSV fieldnames
    # from EACH row's own keys, not fixed from the first row) would
    # silently shift every column after a row with a different key set,
    # exactly the bug class this same file's motion_error_detail
    # comment already warns about for a different field.
    real_record = build_point_record(
        point_id=0, x_mm=6.0, y_mm=8.0,
        ir_result={"value": 900.0, "emissivity": 0.8, "dilution": None, "error": False},
        oes_result={"saturated": False, "error": False},
        feature_values={"CH": 100.0},
        s_mm=10.0,
    )
    real_record["is_reference"] = False
    assert "s_mm" in real_record
    line_logger.write_point(real_record, wavelengths=wl, intensities=intens, ix=1, iy=None)

    ref_record = build_point_record(
        point_id=1, x_mm=0.0, y_mm=0.0,
        ir_result={"value": 899.0, "emissivity": 0.8, "dilution": None, "error": False},
        oes_result={"saturated": False, "error": False},
        feature_values={"CH": 99.0},
        s_mm=float("nan"),  # NOT omitted -- see comment above
    )
    ref_record["is_reference"] = True
    assert "s_mm" in ref_record
    line_logger.write_point(ref_record, wavelengths=wl, intensities=intens, ix=None, iy=None)

    import csv as _csv
    with open(os.path.join(line_test_dir, "scan_summary.csv"), newline="") as f:
        rows = list(_csv.DictReader(f))
    assert len(rows) == 2
    # Both rows have the SAME key set -> both keyed correctly by column,
    # no shift -- x_mm/y_mm land in the right columns on BOTH rows, not
    # just the first.
    assert rows[0]["s_mm"] == "10.0" and rows[0]["x_mm"] == "6.0" and rows[0]["is_reference"] == "False"
    assert rows[1]["s_mm"] == "nan" and rows[1]["x_mm"] == "0.0" and rows[1]["is_reference"] == "True"

    ds_line = OESStore.load(os.path.join(line_test_dir, "oes.h5"))
    assert not np.isnan(ds_line.ir_temp_c.isel(s_mm=1).values), (
        "the real line point (ix=1) should have been written to the HDF5 store")
    assert np.isnan(ds_line.ir_temp_c.isel(s_mm=0).values), (
        "the reference-point revisit must NOT have been written into the line store "
        "(ix=None -- write_point's mode-aware skip condition should have caught it)")

    print("data_logger line-mode smoke test OK")
