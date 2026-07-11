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
4. Compute wafer center + scan range from the 4 recorded positions.
5. Prompt for step size (mm) and dwell time (s), validated against
   scan_params' operator-valid ranges.
6. Preview the resulting grid size and a rough total scan time.
7. Write x_range_mm, y_range_mm, wafer_center_mm, step_size_mm, and
   dwell_time_s back into config.yaml. This is a targeted line-level
   patch, not a full YAML re-dump — it preserves the file's hand-written
   comments, which a plain yaml.safe_dump round-trip would strip.

Note: motion.soft_limits is intentionally NOT enforced while jogging —
this process is what defines the safe scan area, so it can't already be
constrained by it. The physical hard stops are the only real limit at
this stage; watch the mirror.
"""

from __future__ import annotations

import re
import sys
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


def write_results(config_path: Path, area: dict, step_size_mm: float, dwell_time_s: float):
    text = config_path.read_text()

    text = _patch_scalar(text, "x_range_mm",
                          f"[{area['x_range_mm'][0]:.4f}, {area['x_range_mm'][1]:.4f}]")
    text = _patch_scalar(text, "y_range_mm",
                          f"[{area['y_range_mm'][0]:.4f}, {area['y_range_mm'][1]:.4f}]")
    text = _patch_scalar(text, "wafer_center_mm",
                          f"[{area['wafer_center_mm'][0]:.4f}, {area['wafer_center_mm'][1]:.4f}]")
    text = _patch_scalar(text, "step_size_mm", f"{step_size_mm}")
    text = _patch_scalar(text, "dwell_time_s", f"{dwell_time_s}")

    config_path.write_text(text)
    print(f"\nWrote calibration results to {config_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    config_path = Path(sys.argv[1] if len(sys.argv) > 1 else "config.yaml")
    config = yaml.safe_load(config_path.read_text())

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

    area = compute_area(edges)
    print("\n--- Computed scan area ---")
    print(f"  X range: {area['x_range_mm']} mm")
    print(f"  Y range: {area['y_range_mm']} mm")
    print(f"  Wafer center: {area['wafer_center_mm']} mm")

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

    write_results(config_path, area, step_size_mm, dwell_time_s)


if __name__ == "__main__":
    main()
