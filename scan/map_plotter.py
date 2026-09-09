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


def line_values_from_points(df, value_col):
    """
    PR2b (2026-09-08): line-mode counterpart to grid_from_points()
    above. A line scan's points don't form a 2D grid -- there's one
    real spatial axis, s_mm (arc length along the line; see
    scan_manager.generate_line_points() and oes_store.py's line-mode
    schema) -- so this returns (s_mm, values) sorted by position,
    instead of a 2D array. Requires an "s_mm" column, present in
    scan_summary.csv only for line-mode scans (see
    data_logger.build_point_record()'s s_mm parameter).
    """
    sorted_df = df.sort_values("s_mm")
    return sorted_df["s_mm"].to_numpy(), sorted_df[value_col].to_numpy()


def plot_line(s_mm, values, title, output_path, ylabel=None):
    """
    PR2b (2026-09-08): line-mode counterpart to plot_map() above --
    same Figure/FigureCanvasAgg pure-offscreen pattern (see this
    module's docstring for why that matters here specifically), same
    savefig/print convention, but a position-vs-value line plot instead
    of imshow() -- this is the "Temperature versus position" /
    "Selected OES feature versus position" plot the original spec (2D
    Project.docx, Phase 6) asks for, which plot_map() alone could never
    produce (it unconditionally calls imshow(), even for today's
    axis-aligned line scans -- see the PR2b design writeup in the
    project gap-analysis doc for this gap).
    """
    fig = Figure(figsize=(6, 5))
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    ax.plot(s_mm, values, marker="o", markersize=3, linewidth=1)
    ax.set_xlabel("Position along line, s (mm)")
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    print(f"Saved {output_path}")


def _generate_line_maps(df, maps_dir):
    """
    PR2b (2026-09-08): line-mode sibling of generate_all_maps()'s body
    below -- same set of quantities (temperature, emissivity, dilution,
    per-feature, total intensity, the C2 Swan/H-alpha ratio), same
    "skip loud" convention for anything missing, just plotted with
    plot_line()/line_values_from_points() instead of
    plot_map()/grid_from_points(). Kept as a separate function (rather
    than threading an if/else through every block below) so neither
    mode's logic has to read through the other's branches to follow
    what actually happens for its own case.
    """
    s_mm, values = line_values_from_points(df, "ir_temp_c")
    plot_line(s_mm, values, "Substrate Temperature vs. Position",
              os.path.join(maps_dir, "temperature_line.png"), ylabel="Temperature (C)")

    if "ir_emissivity" in df.columns:
        s_mm, values = line_values_from_points(df, "ir_emissivity")
        plot_line(s_mm, values, "Pyrometer Emissivity vs. Position",
                  os.path.join(maps_dir, "emissivity_line.png"), ylabel="Emissivity")
    else:
        print("NOTE: skipping emissivity line plot — 'ir_emissivity' column not found "
              "in scan_summary.csv (older scan, run before 2026-07-21).")

    if "ir_dilution" in df.columns and df["ir_dilution"].notna().any():
        s_mm, values = line_values_from_points(df, "ir_dilution")
        plot_line(s_mm, values, "Pyrometer Signal Dilution vs. Position",
                  os.path.join(maps_dir, "dilution_line.png"), ylabel="Dilution")
    else:
        print("NOTE: skipping dilution line plot — 'ir_dilution' is missing or all-NaN. "
              "Set ir.pac.dilution_tag_name in config.yaml once the real REST tag "
              "name is confirmed (tools/list_pac_strategy_vars.py can help find it).")

    feature_cols = [c for c in df.columns if c.startswith("feature_")]
    for col in feature_cols:
        feature_name = col.replace("feature_", "")
        s_mm, values = line_values_from_points(df, col)
        plot_line(s_mm, values, f"{feature_name} Intensity vs. Position",
                  os.path.join(maps_dir, f"{feature_name}_line.png"), ylabel="Intensity (a.u.)")

    if feature_cols:
        df = df.copy()
        df["total_intensity"] = df[feature_cols].sum(axis=1)
        s_mm, values = line_values_from_points(df, "total_intensity")
        plot_line(s_mm, values, "Total OES Intensity vs. Position",
                  os.path.join(maps_dir, "total_intensity_line.png"),
                  ylabel="Summed Intensity (a.u.)")

    if "feature_C2_Swan" in df.columns and "feature_H_alpha" in df.columns:
        df = df.copy()
        with np.errstate(divide="ignore", invalid="ignore"):
            df["ratio_C2_Halpha"] = df["feature_C2_Swan"] / df["feature_H_alpha"]
        s_mm, values = line_values_from_points(df, "ratio_C2_Halpha")
        plot_line(s_mm, values, "C2 Swan / H-alpha Ratio vs. Position",
                  os.path.join(maps_dir, "ratio_C2_Halpha_line.png"), ylabel="Ratio")
    else:
        print(
            "NOTE: skipping C2 Swan / H-alpha ratio line plot — expected columns "
            "'feature_C2_Swan' and 'feature_H_alpha' not found in "
            "scan_summary.csv. This ratio is hardcoded to those two feature "
            "names; if oes.features in config.yaml uses different names (or "
            "omits one of these), this plot is intentionally skipped, not broken."
        )


def generate_all_maps(config):
    out_cfg = config["output"]
    base_dir = out_cfg["base_dir"]
    summary_path = os.path.join(base_dir, out_cfg["summary_csv"])
    maps_dir = os.path.join(base_dir, "maps")
    os.makedirs(maps_dir, exist_ok=True)

    df = load_scan_summary(summary_path)

    # PR2b (2026-09-08): line-mode scans branch off entirely here, into
    # _generate_line_maps() -- everything below this point (grid_from_points()/
    # plot_map(), the multi-pass warning, every "skip loud" note) assumes
    # a 2D (x_mm, y_mm) grid, which a line scan's points never form (see
    # oes_store.py's module docstring for why an arbitrary-angle line
    # can't be reshaped into one). scan_cfg["grid"]["mode"] is the
    # authoritative signal -- not "does an s_mm column exist" -- because
    # it's set at scan time regardless of whether map generation ever
    # runs, matching how ScanManager itself decides which geometry
    # generator to call.
    scan_mode = config.get("scan", {}).get("grid", {}).get("mode", "grid")
    if scan_mode == "line":
        _generate_line_maps(df, maps_dir)
        return

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


def _self_test():
    """
    PR2b (2026-09-08): synthetic-data smoke test for the new line-mode
    path (plot_line()/line_values_from_points()/_generate_line_maps()/
    generate_all_maps()'s mode branch) -- this file had no self-check
    convention before this (unlike scan_params.py/data_logger.py/
    oes_store.py/live_map_accumulator.py, all of which do; see the
    2026-08-05 test-coverage-gap finding this codebase already tracks),
    so `python scan/map_plotter.py --self-test` is new, opt-in, and
    does NOT change the file's default behavior (no flags still runs
    generate_all_maps() against config.yaml, exactly as before this
    function existed).
    """
    import shutil
    import tempfile

    import pandas as pd

    print("Running map_plotter self-test (line mode)...")
    tmp_dir = tempfile.mkdtemp(prefix="map_plotter_selftest_")
    try:
        n = 6
        s_mm = np.linspace(0.0, 10.0, n)
        df = pd.DataFrame({
            "point_id": range(n),
            "pass_id": [0] * n,
            "x_mm": np.linspace(0.0, 6.0, n),
            "y_mm": np.linspace(0.0, 8.0, n),
            "s_mm": s_mm,
            "is_reference": [False] * n,
            "ir_temp_c": 900.0 + s_mm,
            "ir_emissivity": [0.85] * n,
            "ir_dilution": [float("nan")] * n,  # unset tag name -- must skip loud, not crash
            "feature_C2_Swan": np.linspace(100.0, 200.0, n),
            "feature_H_alpha": np.linspace(50.0, 60.0, n),
        })
        summary_path = os.path.join(tmp_dir, "scan_summary.csv")
        df.to_csv(summary_path, index=False)

        config = {
            "scan": {"grid": {"mode": "line"}},
            "output": {"base_dir": tmp_dir, "summary_csv": "scan_summary.csv"},
        }
        generate_all_maps(config)

        maps_dir = os.path.join(tmp_dir, "maps")
        expected = [
            "temperature_line.png", "emissivity_line.png", "C2_Swan_line.png",
            "H_alpha_line.png", "total_intensity_line.png", "ratio_C2_Halpha_line.png",
        ]
        for filename in expected:
            path = os.path.join(maps_dir, filename)
            assert os.path.exists(path) and os.path.getsize(path) > 0, f"missing or empty: {path}"

        # dilution_line.png must NOT exist -- ir_dilution is all-NaN in
        # this synthetic data, same "skip loud, don't crash" behavior
        # grid mode already has for the same column.
        assert not os.path.exists(os.path.join(maps_dir, "dilution_line.png"))

        # line_values_from_points() itself: sorted by s_mm, values follow.
        s_sorted, values = line_values_from_points(df, "ir_temp_c")
        assert list(s_sorted) == sorted(s_mm)
        assert abs(values[0] - 900.0) < 1e-9 and abs(values[-1] - 910.0) < 1e-9

        print(f"map_plotter self-test OK -- {len(expected)} line plots generated in {maps_dir}")
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


if __name__ == "__main__":
    import sys

    if "--self-test" in sys.argv:
        _self_test()
    else:
        import yaml

        # utf-8-sig: see run_gui.py's copy of this comment -- tolerates/strips a
        # UTF-8 BOM (e.g. from editing config.yaml in Notepad on Windows).
        with open("config.yaml", encoding="utf-8-sig") as f:
            config = yaml.safe_load(f)

        generate_all_maps(config)
