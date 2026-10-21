"""
scan/background.py -- OES background (dark / plasma-off) capture, storage,
selection and subtraction. Batch 2, 2026-10-19.

Workflow (RG, Oct 2026):
  1. Before an OES scan, capture one or more backgrounds on the "OES
     Background" tab (or tools/capture_background.py):
       true_dark   light input blocked -- detector dark current + offset
       plasma_off  normal optical path, plasma off -- dark + stray/room
                   light; capture with room lights on AND off to see
                   their effect
     Each capture averages N readings (each reading is itself the usual
     integration x averages acquisition) and is saved with its settings,
     per-reading timestamps, device and wavelength axis.
  2. At scan start, pick one background (or None) on the Scan / Line Scan
     tab. The scan stores BOTH the unsubtracted and the subtracted spectra,
     plus a copy of the background itself, so nothing is lost and the
     choice can be redone afterwards.
  3. A background whose acquisition settings differ from the scan's is
     flagged: warning at start, and recorded in metadata.yaml / oes.h5.

Subtraction is done on the UNSMOOTHED frame average (intensity_raw), per
pixel, with no clipping (negative values are kept: clipping would bias
weak lines upward). The processed copy is then smoothed exactly like the
unsubtracted one. Same-device, same-pixel-count is required; a different
pixel count is a hard error, everything else is a recorded warning.

File format: backgrounds/bg_<YYYYmmdd_HHMMSS>_<kind>.h5
  /wavelength_nm   (n,)       axis at capture time (provenance only)
  /mean            (n,)       mean of the N raw readings -- what's subtracted
  /std             (n,)       per-pixel std across the N readings
  /readings        (N, n)     every raw reading
  /reading_t_start (N,)       Unix epoch s
  /reading_t_end   (N,)
  attrs: json "record" (kind, room_lights, notes, captured_at, n_readings,
         acquisition settings, device, wavelength calibration)
plus a human-readable bg_<...>.csv (wavelength_nm, mean, std) next to it.
"""

from __future__ import annotations

import csv
import json
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Callable, List, Optional

import numpy as np

KINDS = {
    "true_dark": "True dark (input blocked)",
    "plasma_off": "Plasma off (normal optical path)",
    "other": "Other",
}
ROOM_LIGHTS = ("on", "off", "n/a")
N_READINGS_MIN, N_READINGS_MAX, N_READINGS_DEFAULT = 1, 500, 10
DEFAULT_DIR = "./backgrounds"

# Settings that must match for a background to be strictly comparable.
# Integration time changes both dark current and signal scale; averages and
# boxcar change only noise/smoothing (subtraction uses unsmoothed data) but
# RG asked that ANY settings difference be flagged.
COMPARED_SETTINGS = ("integration_time_us", "num_averages", "boxcar_width")
WAVELENGTH_TOL_NM = 0.01


def background_dir(config) -> str:
    return (config.get("oes", {}) or {}).get("background_dir") or DEFAULT_DIR


def validate_n_readings(value) -> int:
    try:
        f = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"Number of readings must be a whole number, got {value!r}")
    if f != int(f) or not (N_READINGS_MIN <= int(f) <= N_READINGS_MAX):
        raise ValueError(
            f"Number of readings must be {N_READINGS_MIN}-{N_READINGS_MAX}, got {value!r}")
    return int(f)


@dataclass
class Background:
    wavelengths: np.ndarray
    mean: np.ndarray
    std: np.ndarray
    record: dict
    readings: Optional[np.ndarray] = None
    reading_t_start: Optional[np.ndarray] = None
    reading_t_end: Optional[np.ndarray] = None
    path: Optional[str] = None

    @property
    def settings(self) -> dict:
        return self.record.get("acquisition", {})

    def label(self) -> str:
        return describe(self.record, self.path)


def describe(record: dict, path: Optional[str] = None) -> str:
    """One-line operator label, e.g. for the scan-start dropdown."""
    acq = record.get("acquisition", {})
    when = record.get("captured_at", "?").replace("T", " ")[:16]
    kind = record.get("kind", "?").replace("_", "-")
    lights = record.get("room_lights", "n/a")
    it = acq.get("integration_time_us")
    it_s = f"{it / 1000:g} ms" if isinstance(it, (int, float)) else "? ms"
    s = (f"{when} | {kind} | lights {lights} | {it_s} x{acq.get('num_averages', '?')} "
         f"| boxcar {acq.get('boxcar_width', '?')} | N={record.get('n_readings', '?')}")
    if record.get("notes"):
        s += f" | {record['notes'][:40]}"
    return s


# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------

def capture(reader, n_readings: int, kind: str, room_lights: str = "n/a",
            notes: str = "", progress: Optional[Callable[[int, int], None]] = None,
            stop: Optional[Callable[[], bool]] = None) -> Background:
    """
    Take n_readings with an already-connected, already-configured reader
    (settings applied via reader.apply_settings first). Averages the RAW
    (unsmoothed) readings. Raises on any failed read -- a background with
    a silently missing reading isn't one you can trust.
    """
    if kind not in KINDS:
        raise ValueError(f"kind must be one of {sorted(KINDS)}, got {kind!r}")
    if room_lights not in ROOM_LIGHTS:
        raise ValueError(f"room_lights must be one of {ROOM_LIGHTS}, got {room_lights!r}")
    n_readings = validate_n_readings(n_readings)
    wl = reader.wavelengths
    if wl is None:
        raise RuntimeError("Spectrometer is not connected (no wavelength axis)")

    rows, t0s, t1s, saturated = [], [], [], 0
    for k in range(n_readings):
        if stop is not None and stop():
            raise RuntimeError(f"Background capture cancelled after {k} readings")
        r = reader.read()
        if r.error:
            raise IOError(f"Background reading {k + 1}/{n_readings} failed: {r.error}")
        raw = r.raw_intensities if r.raw_intensities is not None else r.intensities
        rows.append(np.asarray(raw, dtype=np.float64))
        t0s.append(r.acq_start if r.acq_start else r.timestamp)
        t1s.append(r.acq_end if r.acq_end else time.time())
        saturated += bool(r.saturated)
        if progress is not None:
            progress(k + 1, n_readings)

    readings = np.vstack(rows)
    acq = reader.acquisition_record() if hasattr(reader, "acquisition_record") else {}
    record = {
        "format": "carat_scanner background v1",
        "kind": kind,
        "kind_description": KINDS[kind],
        "room_lights": room_lights,
        "notes": notes or "",
        "captured_at": datetime.fromtimestamp(t0s[0]).isoformat(timespec="seconds"),
        "t_start": float(t0s[0]),
        "t_end": float(t1s[-1]),
        "n_readings": int(n_readings),
        "n_pixels": int(readings.shape[1]),
        "saturated_readings": int(saturated),
        "acquisition": acq,
        "wavelength_calibration": getattr(reader, "calibration_record", None),
        "summary_counts": {
            "mean_of_mean": float(readings.mean()),
            "min": float(readings.min()),
            "max": float(readings.max()),
            "median_pixel_std": float(np.median(readings.std(axis=0))),
        },
    }
    return Background(
        wavelengths=np.asarray(wl, dtype=np.float64),
        mean=readings.mean(axis=0),
        std=readings.std(axis=0, ddof=1) if n_readings > 1 else np.zeros(readings.shape[1]),
        record=record,
        readings=readings,
        reading_t_start=np.asarray(t0s),
        reading_t_end=np.asarray(t1s),
    )


# ---------------------------------------------------------------------------
# Save / load / list
# ---------------------------------------------------------------------------

def save(bg: Background, directory: str) -> str:
    import h5py
    os.makedirs(directory, exist_ok=True)
    stamp = datetime.fromtimestamp(bg.record["t_start"]).strftime("%Y%m%d_%H%M%S")
    base = os.path.join(directory, f"bg_{stamp}_{bg.record['kind']}")
    path, n = base + ".h5", 1
    while os.path.exists(path):            # never overwrite a saved capture
        n += 1
        path = f"{base}_{n}.h5"
    with h5py.File(path, "w") as f:
        f.create_dataset("wavelength_nm", data=bg.wavelengths)
        f.create_dataset("mean", data=bg.mean)
        f.create_dataset("std", data=bg.std)
        if bg.readings is not None:
            f.create_dataset("readings", data=bg.readings.astype("float32"))
            f.create_dataset("reading_t_start", data=bg.reading_t_start)
            f.create_dataset("reading_t_end", data=bg.reading_t_end)
        f.attrs["record"] = json.dumps(bg.record)
    with open(path[:-3] + ".csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["# " + describe(bg.record)])
        w.writerow(["wavelength_nm", "mean", "std"])
        for row in zip(bg.wavelengths, bg.mean, bg.std):
            w.writerow(row)
    bg.path = path
    return path


def load(path: str, with_readings: bool = False) -> Background:
    import h5py
    with h5py.File(path, "r") as f:
        record = json.loads(f.attrs["record"])
        return Background(
            wavelengths=f["wavelength_nm"][:],
            mean=f["mean"][:],
            std=f["std"][:],
            record=record,
            readings=f["readings"][:] if with_readings and "readings" in f else None,
            reading_t_start=f["reading_t_start"][:] if "reading_t_start" in f else None,
            reading_t_end=f["reading_t_end"][:] if "reading_t_end" in f else None,
            path=path,
        )


def list_saved(directory: str) -> List[dict]:
    """[{path, record, label}] newest first; unreadable files are skipped."""
    import h5py
    out = []
    if not os.path.isdir(directory):
        return out
    for name in os.listdir(directory):
        if not (name.startswith("bg_") and name.endswith(".h5")):
            continue
        path = os.path.join(directory, name)
        try:
            with h5py.File(path, "r") as f:
                record = json.loads(f.attrs["record"])
        except Exception:
            continue
        out.append({"path": path, "record": record, "label": describe(record, path)})
    out.sort(key=lambda d: d["record"].get("t_start", 0), reverse=True)
    return out


# ---------------------------------------------------------------------------
# Compatibility checks + subtraction
# ---------------------------------------------------------------------------

def compare(bg: Background, scan_settings: dict, wavelengths, device: dict = None,
            now: float = None) -> dict:
    """
    -> {"mismatches": [str], "errors": [str], "age_h": float}
    errors: subtraction impossible (pixel count). mismatches: flagged, recorded.
    """
    errors, mismatches = [], []
    n_scan = len(wavelengths) if wavelengths is not None else None
    if n_scan is not None and n_scan != len(bg.mean):
        errors.append(f"background has {len(bg.mean)} pixels, spectrometer has {n_scan}")
    acq = bg.settings
    for key in COMPARED_SETTINGS:
        b, s = acq.get(key), scan_settings.get(key)
        if b is not None and s is not None and int(b) != int(s):
            mismatches.append(f"{key}: background {b}, scan {s}")
    if device:
        for key in ("serial", "model"):
            b, s = acq.get(key), device.get(key)
            if b and s and str(b) != str(s):
                mismatches.append(f"spectrometer {key}: background {b}, scan {s}")
    if not errors and wavelengths is not None:
        d = float(np.max(np.abs(np.asarray(wavelengths, float) - bg.wavelengths)))
        if d > WAVELENGTH_TOL_NM:
            mismatches.append(
                f"wavelength axis differs by up to {d:.2f} nm (calibration changed since "
                "capture; subtraction is per pixel, so it is still valid for the same device)")
    if bg.record.get("saturated_readings"):
        mismatches.append(f"{bg.record['saturated_readings']} background reading(s) saturated")
    now = time.time() if now is None else now
    age_h = (now - float(bg.record.get("t_start", now))) / 3600.0
    return {"mismatches": mismatches, "errors": errors, "age_h": age_h}


def subtract(raw: np.ndarray, bg: Background) -> np.ndarray:
    return np.asarray(raw, dtype=np.float64) - bg.mean


def scan_record(bg: Optional[Background], check: Optional[dict]) -> dict:
    """What metadata.yaml / oes.h5 record about the background for a scan."""
    if bg is None:
        return {"applied": False}
    return {
        "applied": True,
        "file": os.path.abspath(bg.path) if bg.path else None,
        "label": bg.label(),
        "kind": bg.record.get("kind"),
        "room_lights": bg.record.get("room_lights"),
        "notes": bg.record.get("notes"),
        "captured_at": bg.record.get("captured_at"),
        "n_readings": bg.record.get("n_readings"),
        "acquisition": bg.settings,
        "age_hours_at_scan_start": round(check["age_h"], 3) if check else None,
        "settings_mismatch": bool(check and check["mismatches"]),
        "mismatches": list(check["mismatches"]) if check else [],
        "method": "per-pixel subtraction of background mean from unsmoothed frame "
                  "average; no clipping; processed copy smoothed afterwards",
    }
