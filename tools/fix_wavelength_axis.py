"""
fix_wavelength_axis.py -- correct the OES wavelength axis of scans that
were already collected with the broken EEPROM-slot calibration.

Background (2026-10-10): scans before this fix built the axis from
EEPROM slots 1-4, but the ADC1000-USB's coefficients are in slots 2-5, so
every saved wavelength_nm runs 1 -> ~2,000,000 nm. The INTENSITIES are
fine -- only the axis (and anything derived from it: the feature_*
columns in scan_summary.csv and the maps) is wrong. This tool rewrites
the axis in place and recomputes the feature columns, after backing up
the originals. Nothing is deleted.

Usage (preferred: coefficients from tools/dump_spectrometer_eeprom.py,
i.e. EEPROM slots 2, 3, 4, 5 -- or from an Ocean calibration sheet):

    python tools/fix_wavelength_axis.py scan_data/scan_20261009_143000 \\
        --coeffs 201.5547 0.3320928 2.159012e-05 -1.35e-08

Without --coeffs the tool shows the three coefficients it can recover
from the broken axis itself (c0..c2 -- the broken axis is the true one
shifted by one power) and stops; the cubic term c3 lives in a slot the
old code never read, so it can't be recovered from saved data. If you
accept that, pass --c3 explicitly (e.g. --c3 0 for a quadratic-only
axis, typically within a few nm at the red end -- not good enough for
narrow integration intervals there).

Per scan folder it touches:
  spectra/point_*.csv     wavelength_nm column rewritten
                          (originals copied to spectra_original_axis/)
  oes.h5                  /wavelength_nm rewritten; original kept as
                          /wavelength_nm_original_axis; attrs record the fix
  scan_summary.csv        feature_* columns recomputed from the spectra
                          (original copied to scan_summary_original_axis.csv)
  metadata.yaml           'wavelength_axis_corrected' entry added
                          (original copied to metadata_original_axis.yaml)
Add --maps to regenerate maps/ afterwards.

Re-running on an already-corrected folder is refused (the axis already
passes validation) unless --force is given.
"""

import argparse
import csv
import glob
import json
import os
import shutil
import sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import yaml

from readers.wavelength_calibration import evaluate, validate

CORRECTED_SOURCE = "tools/fix_wavelength_axis.py"


def _read_spectrum(path):
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return data[:, 0], data[:, 1]


def infer_from_broken_axis(w):
    """Broken axis = 1 + c0*i + c1*i^2 + c2*i^3 (true coeffs shifted one
    power up). Returns (true c0, c1, c2, fit residual nm)."""
    i = np.arange(len(w), dtype=np.float64)
    fit = np.polynomial.polynomial.polyfit(i, w, 3)
    resid = float(np.max(np.abs(np.polynomial.polynomial.polyval(i, fit) - w)))
    return float(fit[1]), float(fit[2]), float(fit[3]), resid, float(fit[0])


def extract_features(wavelengths, intensities, features_cfg, window_nm):
    # Same definition as scan.scan_manager.extract_features() (not imported
    # to keep this tool free of the motion/hardware import chain).
    out = {}
    for name, center_nm in features_cfg.items():
        mask = np.abs(wavelengths - center_nm) <= window_nm
        out[name] = float(np.sum(intensities[mask])) if np.any(mask) else float("nan")
    return out


def _backup(src, dst):
    if os.path.exists(dst):
        print(f"  backup exists, leaving it alone: {dst}")
        return
    if os.path.isdir(src):
        shutil.copytree(src, dst)
    else:
        shutil.copy2(src, dst)
    print(f"  backed up {os.path.basename(src)} -> {os.path.basename(dst)}")


def fix_scan_dir(scan_dir, coeffs, force=False, maps=False, dry_run=False):
    spectra = sorted(glob.glob(os.path.join(scan_dir, "spectra", "point_*.csv")))
    h5_path = os.path.join(scan_dir, "oes.h5")
    summary_path = os.path.join(scan_dir, "scan_summary.csv")
    meta_path = os.path.join(scan_dir, "metadata.yaml")
    if not spectra and not os.path.exists(h5_path):
        raise SystemExit(f"{scan_dir}: no spectra/ or oes.h5 -- not a scan folder?")

    # Current axis: from the first CSV, else the HDF5.
    if spectra:
        old_w, _ = _read_spectrum(spectra[0])
    else:
        import h5py
        with h5py.File(h5_path, "r") as f:
            old_w = f["wavelength_nm"][:].astype(np.float64)
    n = len(old_w)

    if not validate(old_w) and not force:
        raise SystemExit(
            f"{scan_dir}: current axis {old_w[0]:.1f}-{old_w[-1]:.1f} nm already "
            "passes validation -- already corrected? (use --force to redo)")

    new_w = evaluate(coeffs, n)
    problems = validate(new_w)
    if problems:
        raise SystemExit("new axis fails validation: " + "; ".join(problems))
    print(f"{scan_dir}\n  {n} pixels: {old_w[0]:.1f}-{old_w[-1]:.1f} nm  ->  "
          f"{new_w[0]:.2f}-{new_w[-1]:.2f} nm   ({len(spectra)} spectrum CSVs)")
    if dry_run:
        print("  dry run -- nothing written")
        return

    record = {
        "corrected_at": datetime.now().isoformat(timespec="seconds"),
        "by": CORRECTED_SOURCE,
        "coeffs": [float(c) for c in coeffs],
        "range_nm": [float(new_w[0]), float(new_w[-1])],
        "original_range_nm": [float(old_w[0]), float(old_w[-1])],
    }

    # 1. per-point CSVs
    if spectra:
        _backup(os.path.join(scan_dir, "spectra"),
                os.path.join(scan_dir, "spectra_original_axis"))
        for p in spectra:
            w, inten = _read_spectrum(p)
            if len(w) != n:
                print(f"  SKIP {os.path.basename(p)}: {len(w)} rows, expected {n}")
                continue
            # Keep every column after wavelength_nm (intensity, and
            # intensity_raw in files written after 2026-10-14) untouched.
            with open(p, newline="") as f:
                rows = list(csv.reader(f))
            header, body = rows[0], rows[1:]
            with open(p, "w", newline="") as f:
                wr = csv.writer(f)
                wr.writerow(header)
                for wl, row in zip(new_w, body):
                    wr.writerow([wl] + row[1:])
        print(f"  rewrote {len(spectra)} spectrum CSVs")

    # 2. HDF5
    if os.path.exists(h5_path):
        import h5py
        with h5py.File(h5_path, "a") as f:
            if "wavelength_nm_original_axis" not in f:
                f.create_dataset("wavelength_nm_original_axis", data=f["wavelength_nm"][:])
            if f["wavelength_nm"].shape[0] != n:
                raise SystemExit(f"oes.h5 has {f['wavelength_nm'].shape[0]} wavelengths, CSVs have {n}")
            f["wavelength_nm"][:] = new_w.astype("float32")
            f.attrs["wavelength_axis_corrected"] = json.dumps(record)
            cal = {"valid": True, "source": "manual (post-hoc correction)",
                   "coeffs": record["coeffs"], "range_nm": record["range_nm"],
                   "n_pixels": n, "problems": [], "warnings": []}
            f.attrs["wavelength_calibration"] = json.dumps(cal)
        print("  rewrote oes.h5 /wavelength_nm (original kept as /wavelength_nm_original_axis)")

    # 3. feature columns in scan_summary.csv
    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path, encoding="utf-8-sig") as f:
            meta = yaml.safe_load(f) or {}
    oes_cfg = (meta.get("config") or {}).get("oes") or {}
    features = oes_cfg.get("features")
    window = oes_cfg.get("feature_window_nm")
    if os.path.exists(summary_path) and features and window is not None:
        with open(summary_path, newline="") as f:
            rows = list(csv.DictReader(f))
            fieldnames = list(rows[0].keys()) if rows else []
        updated = 0
        for row in rows:
            p = os.path.join(scan_dir, "spectra", f"point_{int(float(row['point_id'])):05d}.csv")
            if not os.path.exists(p):
                continue
            _, inten = _read_spectrum(p)
            for name, val in extract_features(new_w, inten, features, float(window)).items():
                col = f"feature_{name}"
                if col in row:
                    row[col] = val
            updated += 1
        _backup(summary_path, os.path.join(scan_dir, "scan_summary_original_axis.csv"))
        with open(summary_path, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=fieldnames)
            wr.writeheader()
            wr.writerows(rows)
        print(f"  recomputed feature columns for {updated}/{len(rows)} rows "
              f"({', '.join(features)}; +/-{window} nm)")
    else:
        print("  feature columns NOT recomputed (no scan_summary.csv or no oes.features "
              "in metadata.yaml)")

    # 4. metadata
    if os.path.exists(meta_path):
        _backup(meta_path, os.path.join(scan_dir, "metadata_original_axis.yaml"))
        meta["wavelength_axis_corrected"] = record
        with open(meta_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(meta, f, default_flow_style=False)
        print("  metadata.yaml: added wavelength_axis_corrected")

    if maps:
        cfg = dict(meta.get("config") or {})
        cfg["output"] = dict(cfg.get("output") or {})
        cfg["output"]["base_dir"] = scan_dir
        cfg["output"].setdefault("summary_csv", "scan_summary.csv")
        from scan.map_plotter import generate_all_maps
        generate_all_maps(cfg)
        print(f"  regenerated {os.path.join(scan_dir, 'maps')}")


def _parse_coeffs(text):
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(f"need exactly 4 coefficients, got {len(parts)}")
    return [float(p) for p in parts]


def _join_coeffs(argv):
    """argparse treats '-1.35e-08' (a typical c3) as an unknown option, so
    fold the 4 tokens after --coeffs into one '--coeffs=a,b,c,d' first."""
    out, i = [], 0
    while i < len(argv):
        if argv[i] == "--coeffs" and i + 4 < len(argv) + 1:
            out.append("--coeffs=" + ",".join(argv[i + 1:i + 5]))
            i += 5
        else:
            out.append(argv[i])
            i += 1
    return out


def main():
    ap = argparse.ArgumentParser(
        description="Correct the OES wavelength axis of already-collected scans.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("scan_dirs", nargs="+", help="scan folder(s) (contain spectra/, oes.h5, ...)")
    ap.add_argument("--coeffs", type=_parse_coeffs, metavar="C0 C1 C2 C3",
                    help="true coefficients (EEPROM slots 2,3,4,5 on the ADC1000); "
                         "space- or comma-separated, negative values fine")
    ap.add_argument("--c3", type=float, default=None,
                    help="without --coeffs: use c0..c2 inferred from the broken axis plus this c3")
    ap.add_argument("--maps", action="store_true", help="regenerate maps/ afterwards")
    ap.add_argument("--force", action="store_true", help="re-correct an axis that already looks valid")
    ap.add_argument("--dry-run", action="store_true", help="show what would change, write nothing")
    args = ap.parse_args(_join_coeffs(sys.argv[1:]))

    # Show what the broken axis implies -- cross-check for --coeffs, or
    # the source of c0..c2 when using --c3.
    first = args.scan_dirs[0]
    sp = sorted(glob.glob(os.path.join(first, "spectra", "point_*.csv")))
    inferred = None
    if sp:
        w, _ = _read_spectrum(sp[0])
        if validate(w):
            c0, c1, c2, resid, lead = infer_from_broken_axis(w)
            inferred = (c0, c1, c2)
            print(f"Broken axis in {first}: leading term {lead:.4g} (EEPROM slot 1), "
                  f"fit residual {resid:.2g} nm")
            print(f"  -> true c0, c1, c2 = {c0:.7g}, {c1:.7g}, {c2:.7g}   (c3 not recoverable)")

    if args.coeffs:
        coeffs = list(args.coeffs)
        if inferred:
            for k, (a, b) in enumerate(zip(coeffs[:3], inferred)):
                if abs(a - b) > 1e-3 * max(abs(b), 1e-12):
                    print(f"  WARNING: --coeffs c{k}={a:.7g} differs from the value in the "
                          f"saved data ({b:.7g}). Different spectrometer, or a typo?")
    elif args.c3 is not None:
        if not inferred:
            raise SystemExit("--c3 needs a broken axis to infer c0..c2 from; pass --coeffs instead")
        coeffs = [*inferred, args.c3]
        print(f"  using c3 = {args.c3:g} (supplied, not measured)")
    else:
        raise SystemExit("\nPass --coeffs C0 C1 C2 C3 (from tools/dump_spectrometer_eeprom.py, "
                         "slots 2-5), or --c3 VALUE to accept the inferred c0..c2.")

    for d in args.scan_dirs:
        fix_scan_dir(d, coeffs, force=args.force, maps=args.maps, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
