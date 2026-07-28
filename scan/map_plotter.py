"""
map_plotter.py

Creates 2D maps from a completed scan's summary CSV:
- IR temperature map (dwell-averaged/"filtered" — see ir_temp_c)
- IR emissivity (signal strength) map
- IR signal dilution map (blank/all-NaN until ir.pac.dilution_tag_name
  is confirmed and set in config.yaml — see tools/list_pac_strategy_vars.py)
- Total OES intensity map
- Selected emission line maps (CH, C2 Swan, H-alpha, H-beta)
- Spectral ratio maps

Reads scan_summary.csv (written by data_logger.py) and produces
PNG maps via matplotlib.

Uses matplotlib's Figure/FigureCanvasAgg directly rather than pyplot --
same reasoning as gui/live_map.py's own module docstring: pyplot carries
global figure-manager state and, depending on what's already imported in
the process, may pick an interactive GUI backend. That was harmless while
this module only ever ran standalone (`python scan/map_plotter.py`, its
own process) -- it stopped being harmless once scan_manager.py started
calling generate_all_maps() automatically from ScanManager._generate_maps()
at the end of a GUI-driven scan, which runs on gui/scan_worker.py's
background worker THREAD, not the Tk mainloop thread. Tkinter (and any
interactive matplotlib backend built on it) is not thread-safe to touch
from a non-main thread, which is exactly the rule gui/scan_worker.py's own
module docstring is careful about elsewhere ("This module never touches a
tkinter object"). Figure + FigureCanvasAgg is the pure offscreen-rendering
path -- no GUI toolkit involved at all -- so it's safe to call from any
thread regardless of what the rest of the process is doing with Tk.
"""

import os

import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg


def load_scan_summary(summary_csv_path):
    df = pd.read_csv(summary_csv_path)
    # Reference-point revisits are for drift tracking, not the spatial map
    if "is_reference" in df.columns:
        df = df[df["is_reference"] == False]  # noqa: E712
    return df


def grid_from_points(df, value_col):
    """
    Reshape a column of point values into a 2D grid based on
    unique x/y coordinates. Assumes a regular grid (nx by ny).
    """
    xs = np.sort(df["x_mm"].unique())
    ys = np.sort(df["y_mm"].unique())

    grid = np.full((len(ys), len(xs)), np.nan)

    x_index = {x: i for i, x in enumerate(xs)}
    y_index = {y: i for i, y in enumerate(ys)}

    for _, row in df.iterrows():
        xi = x_index[row["x_mm"]]
        yi = y_index[row["y_mm"]]
        grid[yi, xi] = row[value_col]

    return xs, ys, grid


def plot_map(xs, ys, grid, title, output_path, cmap="viridis", label=None):
    # Figure(...) + FigureCanvasAgg(fig) (not plt.subplots()) -- see this
    # module's docstring for why: pure offscreen rendering, no pyplot
    # global state, safe to call from any thread.
    fig = Figure(figsize=(6, 5))
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    im = ax.imshow(
        grid,
        extent=[xs.min(), xs.max(), ys.min(), ys.max()],
        origin="lower",
        cmap=cmap,
        aspect="equal",
    )
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title(title)
    cbar = fig.colorbar(im, ax=ax)
    if label:
        cbar.set_label(label)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    print(f"Saved {output_path}")


def generate_all_maps(config):
    out_cfg = config["output"]
    base_dir = out_cfg["base_dir"]
    summary_path = os.path.join(base_dir, out_cfg["summary_csv"])
    maps_dir = os.path.join(base_dir, "maps")
    os.makedirs(maps_dir, exist_ok=True)

    df = load_scan_summary(summary_path)

    # grid_from_points() below keys each cell by (x_mm, y_mm) alone and
    # just overwrites on collision -- for a scan.passes > 1 run (see
    # scan_manager.py/oes_store.py's pass_id axis), the CSV has one row
    # per (x, y, pass), so every map this function produces silently ends
    # up showing only the LAST pass's snapshot, with no per-pass output
    # and no indication that earlier passes were dropped. Made loud here
    # rather than fixed (which pass to show, or averaged/per-pass PNGs, is
    # a product decision) -- matches this file's existing "make a skip
    # loud" convention (see the ratio-map / dilution-map skip notes below).
    if "pass_id" in df.columns and df["pass_id"].nunique() > 1:
        print(
            f"NOTE: this scan has {df['pass_id'].nunique()} passes (scan.passes > 1), "
            "but every map below only shows the LAST pass at each point — "
            "grid_from_points() has no per-pass handling. For full multi-pass "
            "data (per-point time series across passes, e.g. oscillation "
            "tracking), load the HDF5 store instead: "
            "scan.oes_store.OESStore.load(<output.oes_hdf5 path>)."
        )

    # IR temperature map (dwell-averaged/"filtered")
    xs, ys, grid = grid_from_points(df, "ir_temp_c")
    plot_map(xs, ys, grid, "Substrate Temperature", os.path.join(maps_dir, "temperature_map.png"),
             cmap="inferno", label="Temperature (C)")

    # IR emissivity (signal strength) map
    if "ir_emissivity" in df.columns:
        xs, ys, grid = grid_from_points(df, "ir_emissivity")
        plot_map(xs, ys, grid, "Pyrometer Emissivity", os.path.join(maps_dir, "emissivity_map.png"),
                 cmap="viridis", label="Emissivity")
    else:
        print("NOTE: skipping emissivity map — 'ir_emissivity' column not found "
              "in scan_summary.csv (older scan, run before 2026-07-21).")

    # IR signal dilution map — all-NaN (skipped) until ir.pac.dilution_tag_name
    # is confirmed and filled into config.yaml; see tools/list_pac_strategy_vars.py.
    if "ir_dilution" in df.columns and df["ir_dilution"].notna().any():
        xs, ys, grid = grid_from_points(df, "ir_dilution")
        plot_map(xs, ys, grid, "Pyrometer Signal Dilution", os.path.join(maps_dir, "dilution_map.png"),
                 cmap="viridis", label="Dilution")
    else:
        print("NOTE: skipping dilution map — 'ir_dilution' is missing or all-NaN. "
              "Set ir.pac.dilution_tag_name in config.yaml once the real REST tag "
              "name is confirmed (tools/list_pac_strategy_vars.py can help find it).")

    # Feature maps
    feature_cols = [c for c in df.columns if c.startswith("feature_")]
    for col in feature_cols:
        feature_name = col.replace("feature_", "")
        xs, ys, grid = grid_from_points(df, col)
        plot_map(xs, ys, grid, f"{feature_name} Intensity",
                 os.path.join(maps_dir, f"{feature_name}_map.png"),
                 cmap="viridis", label="Intensity (a.u.)")

    # Total OES intensity (sum of all features as a proxy)
    if feature_cols:
        df["total_intensity"] = df[feature_cols].sum(axis=1)
        xs, ys, grid = grid_from_points(df, "total_intensity")
        plot_map(xs, ys, grid, "Total OES Intensity",
                 os.path.join(maps_dir, "total_intensity_map.png"),
                 cmap="viridis", label="Summed Intensity (a.u.)")

    # Example spectral ratio map: C2 Swan / H-alpha (adjust as needed).
    # Hardcoded to these two exact feature names — if oes.features in
    # config.yaml doesn't define both, this is skipped. Made that skip
    # loud on purpose (2026-07-11): it used to fail silently with no
    # indication the ratio map was ever expected.
    if "feature_C2_Swan" in df.columns and "feature_H_alpha" in df.columns:
        with np.errstate(divide="ignore", invalid="ignore"):
            df["ratio_C2_Halpha"] = df["feature_C2_Swan"] / df["feature_H_alpha"]
        xs, ys, grid = grid_from_points(df, "ratio_C2_Halpha")
        plot_map(xs, ys, grid, "C2 Swan / H-alpha Ratio",
                 os.path.join(maps_dir, "ratio_C2_Halpha_map.png"),
                 cmap="coolwarm", label="Ratio")
    else:
        print(
            "NOTE: skipping C2 Swan / H-alpha ratio map — expected columns "
            "'feature_C2_Swan' and 'feature_H_alpha' not found in "
            "scan_summary.csv. This ratio is hardcoded in "
            "generate_all_maps() to those two feature names; if "
            "oes.features in config.yaml uses different names (or omits "
            "one of these), this map is intentionally skipped, not broken."
        )


if __name__ == "__main__":
    import yaml

    # utf-8-sig: see run_gui.py's copy of this comment -- tolerates/strips a
    # UTF-8 BOM (e.g. from editing config.yaml in Notepad on Windows).
    with open("config.yaml", encoding="utf-8-sig") as f:
        config = yaml.safe_load(f)

    generate_all_maps(config)
