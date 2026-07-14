"""
calibrate_scan_area.py

Interactive scanner-area calibration: the operator jogs the aim spot to
the four wafer edges, and this tool derives the scan center and range
from those positions, then collects dwell time and step size and writes
all of it into config.yaml.

Run this after installation, or whenever the scanner is physically moved.

Usage
-----
    python calibrate_scan_area.py [config.yaml]

Workflow
--------
1. Manual checklist: wafer or calibration target installed, scanner
   mounted, pyrometer aim light ON. There is no software control of the
   aim light in this codebase yet, so this is a human checklist step,
   not an automated one.
2. Home the motion axes.
3. Jog to each of 4 edges (left, right, top, bottom) and confirm:
     - On Windows: real arrow keys via msvcrt (Left/Right = X, Up/Down = Y).
     - Elsewhere (this dev sandbox, Mac/Linux terminals): typed w/a/s/d +
       Enter. Raw arrow-key capture is OS-specific and this is a command-
       line lab-instrument script rather than a GUI, so the fallback keeps
       it dependency-free; both paths call the same motion.jog().
4. Optionally derive steps_per_mm_x/y from those same 4 edges (see
   calibrate_steps_per_mm()) — no new hardware call needed. The mm
   values just recorded were computed from raw motor steps using
   whatever steps_per_mm is currently in config (often still a
   placeholder), so steps_taken = mm * current_steps_per_mm recovers
   the exact step count between edges. Divide that by a TRUE,
   independently-known distance (a wafer of known diameter, a caliper
   measurement) and you get the real ratio — folding what used to be a
   separate --calibrate-x/-y procedure into this same edge jog. Skip by
   leaving the prompts blank if steps_per_mm is already calibrated.
   Caveat: this is one sample from an organic back-and-forth jog, not
   the averaged, single-direction measurement the dedicated CLI
   procedure is designed to produce — good for a quick or first pass,
   not final precision.
5. Compute wafer center + scan range from the 4 recorded positions,
   rescaled to the corrected steps_per_mm if step 4 ran.
6. If steps_per_mm was recalibrated, optionally suggest a tighter
   home_steps bound for future homes (see recommend_home_steps()),
   in place of the 100,000-step blind default — this is a lower bound
   from how far this session's edges got, not a real measurement of
   the hard-stop distance, so it's padded with a safety margin and
   gated behind its own explicit prompt.
7. Prompt for step size (mm) and dwell time (s), validated against
   scan_params' operator-valid ranges.
8. Preview the resulting grid size and a rough total scan time.
9. Write everything collected back into config.yaml. This is a
   targeted line-level patch, not a full YAML re-dump — it preserves
   the file's hand-written comments, which a plain yaml.safe_dump
   round-trip would strip.

Note: motion.soft_limits is intentionally NOT enforced while jogging —
this process is what defines the safe scan area, so it can't already be
constrained by it. The physical hard stops are the only real limit at
this stage; watch the mirror.
"""

from __future__ import annotations

import re
import sys
from datetime import date
from pathlib import Path

import yaml

from motion.motion_controller import get_motion_controller
import scan_params

JOG_STEP_DEFAULT_MM = 1.0
JOG_STEP_MIN_MM = 0.05
JOG_STEP_MAX_MM = 10.0

EDGE_ORDER = ["left", "right", "top", "bottom"]
EDGE_PROMPTS = {
    "left": "Jog the spot to the LEFT edge of the wafer.",
    "right": "Jog the spot to the RIGHT edge of the wafer.",
    "top": "Jog the spot to the TOP edge of the wafer.",
    "bottom": "Jog the spot to the BOTTOM edge of the wafer.",
}

try:
    import msvcrt  # Windows only
    _HAS_MSVCRT = True
except ImportError:
    _HAS_MSVCRT = False


# ---------------------------------------------------------------------------
# Jog loops — same motion.jog() calls underneath, different input methods
# ---------------------------------------------------------------------------

def _jog_loop_msvcrt(motion, jog_step_mm):
    """Windows: real arrow keys via msvcrt. Returns confirmed (x_mm, y_mm)."""
    print("  Arrow keys to jog, +/- to change step size, ENTER to confirm, q to abort.")
    while True:
        x, y = motion.get_position()
        print(f"  step={jog_step_mm:.2f}mm  pos=({x:.3f}, {y:.3f}) mm   ", end="\r")
        ch = msvcrt.getch()
        if ch in (b"\r", b"\n"):
            print()
            return x, y
        if ch in (b"q", b"Q"):
            print()
            raise KeyboardInterrupt("Calibration aborted by operator")
        if ch == b"+":
            jog_step_mm = min(JOG_STEP_MAX_MM, jog_step_mm * 2)
            continue
        if ch == b"-":
            jog_step_mm = max(JOG_STEP_MIN_MM, jog_step_mm / 2)
            continue
        if ch == b"\xe0":  # arrow-key prefix on Windows
            arrow = msvcrt.getch()
            if arrow == b"H":      # up
                motion.jog(dy_mm=jog_step_mm)
            elif arrow == b"P":    # down
                motion.jog(dy_mm=-jog_step_mm)
            elif arrow == b"K":    # left
                motion.jog(dx_mm=-jog_step_mm)
            elif arrow == b"M":    # right
                motion.jog(dx_mm=jog_step_mm)


def _jog_loop_typed(motion, jog_step_mm):
    """Fallback jog loop for non-Windows terminals: typed commands."""
    print("  Commands: w/s = Y +/-, a/d = X +/-, +/- = change step size, "
          "c = confirm, q = abort. Enter after each command.")
    while True:
        x, y = motion.get_position()
        print(f"  step={jog_step_mm:.2f}mm  pos=({x:.3f}, {y:.3f}) mm")
        cmd = input("  > ").strip().lower()
        if cmd == "c":
            return x, y
        if cmd == "q":
            raise KeyboardInterrupt("Calibration aborted by operator")
        if cmd == "+":
            jog_step_mm = min(JOG_STEP_MAX_MM, jog_step_mm * 2)
        elif cmd == "-":
            jog_step_mm = max(JOG_STEP_MIN_MM, jog_step_mm / 2)
        elif cmd == "w":
            motion.jog(dy_mm=jog_step_mm)
        elif cmd == "s":
            motion.jog(dy_mm=-jog_step_mm)
        elif cmd == "a":
            motion.jog(dx_mm=-jog_step_mm)
        elif cmd == "d":
            motion.jog(dx_mm=jog_step_mm)
        else:
            print(f"  (unrecognized command {cmd!r})")


def jog_to_edge(motion, edge_name, jog_step_mm=JOG_STEP_DEFAULT_MM):
    print(f"\n--- {edge_name.upper()} EDGE ---")
    print(f"  {EDGE_PROMPTS[edge_name]}")
    loop = _jog_loop_msvcrt if _HAS_MSVCRT else _jog_loop_typed
    x, y = loop(motion, jog_step_mm)
    print(f"  Recorded {edge_name} edge at ({x:.3f}, {y:.3f}) mm")
    return x, y


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def compute_area(edges: dict) -> dict:
    """
    Derive x_range, y_range, and wafer center from the 4 recorded edge
    positions. Uses sorted() rather than assuming which physical edge has
    the larger coordinate — robust to axis-direction conventions.
    """
    x_left, _ = edges["left"]
    x_right, _ = edges["right"]
    _, y_top = edges["top"]
    _, y_bottom = edges["bottom"]

    x_min, x_max = sorted((x_left, x_right))
    y_min, y_max = sorted((y_bottom, y_top))

    center_x = (x_min + x_max) / 2.0
    center_y = (y_min + y_max) / 2.0

    return {
        "x_range_mm": [x_min, x_max],
        "y_range_mm": [y_min, y_max],
        "wafer_center_mm": [center_x, center_y],
    }


def calibrate_steps_per_mm(edges: dict, config: dict):
    """
    Derive steps_per_mm_x/y from the same 4 edge positions used for the
    wafer-area calibration, instead of the separate --calibrate-x/-y CLI
    procedure. No new hardware call needed: the mm values in `edges`
    were computed from raw motor steps using the CURRENT (possibly
    placeholder) steps_per_mm, so steps_taken = mm * current_steps_per_mm
    recovers the exact step count. Dividing that by a TRUE, independently
    known distance (a wafer of known diameter, a caliper measurement)
    gives the real ratio.

    Optional — returns None if the operator leaves either prompt blank.

    Caveat: this reuses the operator's organic back-and-forth edge jog,
    not a clean single-direction sweep. The original --calibrate-x/-y
    procedure recommends repeating a single-direction move 3-5x and
    averaging specifically to cancel out backlash/hysteresis. This is
    one convenient sample, not an averaged, hysteresis-controlled one —
    fine for a quick or first-pass calibration, but prefer the dedicated
    procedure for final precision.
    """
    old_spmm_x = float(config["motion"]["steps_per_mm_x"])
    old_spmm_y = float(config["motion"]["steps_per_mm_y"])

    x_left, _ = edges["left"]
    x_right, _ = edges["right"]
    _, y_top = edges["top"]
    _, y_bottom = edges["bottom"]

    steps_left_right = abs(x_right - x_left) * old_spmm_x
    steps_bottom_top = abs(y_top - y_bottom) * old_spmm_y

    print("\n--- Steps/mm calibration (optional) ---")
    print("Reuses the left/right and top/bottom edges you just jogged to.")
    print("Leave either prompt blank to skip if steps_per_mm is already")
    print("calibrated and you're just re-checking the wafer area.")
    print(f"  Left-to-right jog used {steps_left_right:.0f} motor steps (X axis).")
    print(f"  Bottom-to-top jog used {steps_bottom_top:.0f} motor steps (Y axis).")
    print("  NOTE: one sample from your organic jog, not an averaged, single-")
    print("  direction measurement — good for a quick pass, not final precision.")

    true_x_raw = input("  True left-to-right distance, mm (e.g. wafer diameter) [skip]: ").strip()
    true_y_raw = input("  True bottom-to-top distance, mm [skip]: ").strip()

    if not true_x_raw or not true_y_raw:
        print("  Skipped.")
        return None

    true_x_mm = float(true_x_raw)
    true_y_mm = float(true_y_raw)

    new_spmm_x = steps_left_right / true_x_mm
    new_spmm_y = steps_bottom_top / true_y_mm

    print(f"  steps_per_mm_x: {old_spmm_x:.2f} -> {new_spmm_x:.2f}")
    print(f"  steps_per_mm_y: {old_spmm_y:.2f} -> {new_spmm_y:.2f}")

    return {
        "steps_per_mm_x": new_spmm_x,
        "steps_per_mm_y": new_spmm_y,
        "old_steps_per_mm_x": old_spmm_x,
        "old_steps_per_mm_y": old_spmm_y,
    }


def rescale_edges(edges: dict, spmm_result: dict) -> dict:
    """
    Re-express recorded edge positions in TRUE mm using a freshly
    calibrated steps_per_mm, instead of the old (possibly placeholder)
    ratio they were originally recorded with. Same raw step counts,
    different mm conversion — steps_taken = old_mm * old_ratio is fixed,
    so new_mm = steps_taken / new_ratio = old_mm * (old_ratio / new_ratio).
    """
    scale_x = spmm_result["old_steps_per_mm_x"] / spmm_result["steps_per_mm_x"]
    scale_y = spmm_result["old_steps_per_mm_y"] / spmm_result["steps_per_mm_y"]
    return {name: (x * scale_x, y * scale_y) for name, (x, y) in edges.items()}


def recommend_home_steps(edges: dict, spmm_result: dict, margin: float = 2.0) -> int:
    """
    Suggest a tighter home_steps bound than the 100,000-step blind
    default, based on how far this session's edges actually got from
    the current origin (in corrected steps, via spmm_result).

    This is a LOWER BOUND, not a measurement of the true hard-stop
    distance — the mechanical stop could be anywhere beyond the
    furthest edge reached. `margin` (default 2x) pads for that
    uncertainty. If a future home() still times out with this value,
    the true stop is farther than this heuristic assumed — raise it
    further rather than assuming something is wrong.
    """
    max_x_mm = max(abs(edges["left"][0]), abs(edges["right"][0]))
    max_y_mm = max(abs(edges["top"][1]), abs(edges["bottom"][1]))
    steps_x = max_x_mm * spmm_result["steps_per_mm_x"]
    steps_y = max_y_mm * spmm_result["steps_per_mm_y"]
    return int(max(steps_x, steps_y) * margin)


def prompt_float(label, default, validator):
    while True:
        raw = input(f"  {label} [{default}]: ").strip()
        value = float(raw) if raw else default
        try:
            return validator(value)
        except ValueError as e:
            print(f"  {e} — try again.")


# ---------------------------------------------------------------------------
# Config file patching — line-level, preserves comments
# ---------------------------------------------------------------------------

def _patch_scalar(text: str, key: str, new_value: str) -> str:
    """
    Replace the value on a single `<key>: <value>` line, preserving any
    trailing inline `# comment` on that same line and every other line
    untouched. A full yaml.safe_dump round-trip would strip config.yaml's
    hand-written comments (including these values' own inline docs), so
    this patches in place instead.
    """
    pattern = re.compile(
        rf"^([ \t]*{re.escape(key)}:)[ \t]*([^#\r\n]*?)[ \t]*(#.*)?$",
        re.MULTILINE,
    )
    if not pattern.search(text):
        raise KeyError(f"Could not find a '{key}:' line to patch in config file")

    def _replace(m):
        prefix, comment = m.group(1), m.group(3)
        line = f"{prefix} {new_value}"
        if comment:
            line += f"  {comment}"
        return line

    return pattern.sub(_replace, text, count=1)


def write_results(config_path: Path, results: dict):
    """
    Patch config.yaml with the calibration results.

    `results` always has x_range_mm, y_range_mm, wafer_center_mm (each a
    [lo, hi] pair), step_size_mm, and dwell_time_s. It may also carry
    steps_per_mm_x/steps_per_mm_y (if calibrate_steps_per_mm ran) and
    home_steps (if the operator chose to write the suggested homing
    bound) — both optional scalars, patched the same way as everything
    else, one dict in rather than separate parameters.
    """
    text = config_path.read_text(encoding="utf-8-sig")

    for key in ("x_range_mm", "y_range_mm", "wafer_center_mm"):
        lo, hi = results[key]
        text = _patch_scalar(text, key, f"[{lo:.4f}, {hi:.4f}]")

    text = _patch_scalar(text, "step_size_mm", f"{results['step_size_mm']}")
    text = _patch_scalar(text, "dwell_time_s", f"{results['dwell_time_s']}")

    if "steps_per_mm_x" in results:
        text = _patch_scalar(text, "steps_per_mm_x", f"{results['steps_per_mm_x']:.4f}")
        text = _patch_scalar(text, "steps_per_mm_y", f"{results['steps_per_mm_y']:.4f}")
        text = _patch_scalar(text, "calibration_date", f'"{date.today().isoformat()}"')

    if "home_steps" in results:
        text = _patch_scalar(text, "home_steps", f"{results['home_steps']}")

    config_path.write_text(text, encoding="utf-8")
    print(f"\nWrote calibration results to {config_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    config_path = Path(sys.argv[1] if len(sys.argv) > 1 else "config.yaml")
    config = yaml.safe_load(config_path.read_text(encoding="utf-8-sig"))

    print("=" * 60)
    print("SCANNER AREA CALIBRATION")
    print("Run after installation, or whenever the scanner is moved.")
    print("=" * 60)
    input(
        "\nBefore continuing, confirm:\n"
        "  [ ] Wafer or calibration target installed\n"
        "  [ ] Scanner mounted\n"
        "  [ ] Pyrometer aim light ON\n"
        "Press ENTER when ready..."
    )

    motion = get_motion_controller(config)
    print("\nHoming...")
    motion.home()

    edges = {}
    for edge_name in EDGE_ORDER:
        edges[edge_name] = jog_to_edge(motion, edge_name)

    spmm_result = calibrate_steps_per_mm(edges, config)
    if spmm_result is not None:
        edges = rescale_edges(edges, spmm_result)

    area = compute_area(edges)
    print("\n--- Computed scan area ---")
    print(f"  X range: {area['x_range_mm']} mm")
    print(f"  Y range: {area['y_range_mm']} mm")
    print(f"  Wafer center: {area['wafer_center_mm']} mm")

    recommended_home_steps = None
    if spmm_result is not None:
        recommended_home_steps = recommend_home_steps(edges, spmm_result)
        print("\n--- Homing sweep bound ---")
        print(f"  Furthest edge reached this session implies the hard stop is at least")
        print(f"  ~{recommended_home_steps // 2} steps away; suggesting home_steps="
              f"{recommended_home_steps} (2x margin) instead of the current blind default.")
        print("  This is a lower-bound guess, not a measurement — if a future home()")
        print("  times out with this value, the real stop is farther; raise it and retry.")
        if input("  Write this home_steps to config.yaml too? [y/N] ").strip().lower() != "y":
            recommended_home_steps = None

    print("\n--- Scan parameters ---")
    step_size_mm = prompt_float(
        f"Step size mm (range {scan_params.STEP_SIZE_MIN_MM}-{scan_params.STEP_SIZE_MAX_MM})",
        scan_params.STEP_SIZE_DEFAULT_MM, scan_params.validate_step_size_mm,
    )
    dwell_time_s = prompt_float(
        f"Dwell time s (range {scan_params.DWELL_TIME_MIN_S}-{scan_params.DWELL_TIME_MAX_S})",
        scan_params.DWELL_TIME_DEFAULT_S, scan_params.validate_dwell_time_s,
    )

    nx, ny = scan_params.grid_dims_from_range(area["x_range_mm"], area["y_range_mm"], step_size_mm)
    est_s = scan_params.estimate_scan_time_s(nx, ny, dwell_time_s)
    print("\n--- Preview ---")
    print(f"  Grid: {nx} x {ny} = {nx * ny} points")
    print(f"  Estimated scan time: {est_s / 60:.1f} min")

    if input("\nWrite these values to config.yaml? [y/N] ").strip().lower() != "y":
        print("Not written. Re-run to try again.")
        return

    results = dict(area)
    results["step_size_mm"] = step_size_mm
    results["dwell_time_s"] = dwell_time_s
    if spmm_result is not None:
        results["steps_per_mm_x"] = spmm_result["steps_per_mm_x"]
        results["steps_per_mm_y"] = spmm_result["steps_per_mm_y"]
    if recommended_home_steps is not None:
        results["home_steps"] = recommended_home_steps
    write_results(config_path, results)


if __name__ == "__main__":
    main()
