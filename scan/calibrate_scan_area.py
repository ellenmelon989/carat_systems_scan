"""
calibrate_scan_area.py

Interactive scanner-area calibration: the operator jogs the aim spot to
the four wafer edges, and this tool derives the scan center and range
from those positions, then collects dwell time and step size and writes
all of it into config.yaml.

Run this after installation, or whenever the scanner is physically moved.

Usage
-----
    python scan/calibrate_scan_area.py [config.yaml]

Workflow
--------
1. Manual checklist: wafer or calibration target installed, scanner
   mounted, pyrometer aim light ON. There is no software control of the
   aim light in this codebase yet, so this is a human checklist step,
   not an automated one.
2. Clearance check: small test jogs in all 4 directions from wherever the
   stage is currently sitting, confirmed by eye — catches an already-
   at-a-limit starting position before anything else moves (open-loop,
   no stall feedback, so this can only be verified visually).
3. Jog to a fixed reference mark and zero the origin there (fiducial
   homing) — NOT a drive into a mechanical hard stop. Avoids ever
   needing to characterize home_steps/home_velocity or risk stall-
   contact wear; trustworthy only insofar as the operator confirms
   they've actually reached the mark, same as any wafer edge below.
4. Jog to each of 4 wafer edges (left, right, top, bottom) and confirm:
     - On Windows: real arrow keys via msvcrt (Left/Right = X, Up/Down = Y).
     - Elsewhere (this dev sandbox, Mac/Linux terminals): typed w/a/s/d +
       Enter. Raw arrow-key capture is OS-specific and this is a command-
       line lab-instrument script rather than a GUI, so the fallback keeps
       it dependency-free; both paths call the same motion.jog().
5. Optionally derive steps_per_mm_x/y from those same 4 edges (see
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
6. Compute wafer center + scan range from the 4 recorded positions,
   rescaled to the corrected steps_per_mm if step 5 ran. Also compute a
   wafer radius (see compute_radius_mm()) so the scan can mask its
   rectangular bounding box down to the wafer's actual circular
   footprint — the box's corners are off-sample by construction for a
   round wafer, and are also the largest-excursion grid points, the
   likeliest place to exceed the mount's own mechanical travel
   regardless of what shape the sample is.
7. If steps_per_mm was recalibrated, optionally suggest a tighter
   home_steps bound — kept as a fallback/reference figure only; the
   fiducial approach above means home_steps/home_velocity no longer
   need to be right for correctness, only if motion.hard_home is ever
   deliberately re-enabled.
8. Prompt for step size (mm) and dwell time (s), validated against
   scan_params' operator-valid ranges.
9. Preview the resulting grid size and a rough total scan time.
10. Write everything collected back into config.yaml. This is a
    targeted line-level patch, not a full YAML re-dump — it preserves
    the file's hand-written comments, which a plain yaml.safe_dump
    round-trip would strip.
11. Optionally continue straight into a scan using this calibration,
    reusing the same already-homed motion connection (see the "Run the
    scan now?" prompt at the end of main()) — one origin per session,
    not one per script.

Note: motion.soft_limits is intentionally NOT enforced while jogging —
this process is what defines the safe scan area, so it can't already be
constrained by it. The physical hard stops are the only real limit at
this stage; watch the mirror.
"""

from __future__ import annotations

# --- repo-root import bootstrap -------------------------------------------
# See scan/scan_manager.py's own copy of this comment for the full
# rationale -- lets this file run directly, as a module, or be imported
# from elsewhere (e.g. gui/calibration_panel.py), all resolving motion/ and
# scan.scan_params/scan.scan_manager the same way regardless of invocation.
import os as _os
import sys as _sys

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)
# ---------------------------------------------------------------------------

import re
import sys
from datetime import date
from pathlib import Path

import yaml

from motion.motion_controller import get_motion_controller
import scan.scan_params as scan_params

JOG_STEP_DEFAULT_MM = 1.0
JOG_STEP_MIN_MM = 0.05
JOG_STEP_MAX_MM = 10.0

# Decimal precision write_results() persists x_range_mm/y_range_mm/
# wafer_center_mm to in config.yaml. compute_area()'s output is rounded
# to this SAME precision in main(), immediately and once, before it's
# used for anything else (printed, fed into the grid-size preview, or
# eventually written to disk) — so the preview and the file
# scan_manager.py later reads in a separate process are guaranteed to be
# derived from identical numbers, not just numbers that happen to be
# close.
#
# Rounding away float noise inside grid_dims_from_range (see
# scan_params.py) is NOT sufficient on its own to guarantee this: the
# config.yaml write below is a real quantization (up to 0.5 * 10^-DECIMALS
# mm per bound), not just floating-point noise, and by itself can move a
# span across a round()-tie boundary. Found 2026-07-17: with this
# rounding applied only at write time (not also at preview time), a live
# span of 74.99996 mm previewed as an 8x8 grid but round-tripped through
# a 4-decimal config.yaml write to exactly 75.0000 mm, which the executed
# scan then read back as a 9x9 grid — a silent preview/execution
# mismatch, distinct from (and not fixed by) the ordinary float-noise
# case grid_dims_from_range() guards against.
CONFIG_RANGE_DECIMALS = 4

EDGE_ORDER = ["left", "right", "top", "bottom"]
EDGE_PROMPTS = {
    "left": "Jog the spot to the LEFT edge of the wafer.",
    "right": "Jog the spot to the RIGHT edge of the wafer.",
    "top": "Jog the spot to the TOP edge of the wafer.",
    "bottom": "Jog the spot to the BOTTOM edge of the wafer.",
    "reference": "Jog the spot to the FIXED REFERENCE MARK (see on-site setup "
                 "notes) — a permanent, visible fiducial independent of any "
                 "wafer/target, used to anchor this session's origin.",
}

# How far (mm) each direction the clearance check jogs to confirm the
# stage isn't already sitting at a hard limit before anything else moves.
# Small enough to be a negligible risk even right at a true edge, large
# enough to be unambiguous to the eye — not a precision measurement, just
# "did it visibly move at all."
CLEARANCE_CHECK_STEP_MM = 3.0

# How far (mm) of CUMULATIVE jogging within a single jog_to_edge() call
# (reference mark OR any wafer edge) is allowed before the loop forces a
# "are you still watching real motion?" checkpoint. clearance_check() only
# rules out being already-at-a-limit at the very start of a session — it
# says nothing about a limit reached partway through a LATER, larger jog
# (e.g. 40mm into jogging toward a wafer edge). Since there's no stall
# feedback on this hardware, the only thing that can ever catch that is a
# human watching at the moment it happens — a single checkpoint at t=0
# doesn't extend to jog #47. This forces that same check periodically
# throughout every jog, not just once before any of them start.
# Raised from a 10mm hardcoded value to a 100mm default (2026-07-20,
# on-site feedback): 10mm was interrupting operators too often on ordinary
# edge jogs (wafer edges are often tens of mm away), turning a safety
# check into an annoyance that invites reflexively hitting "y" without
# actually looking. Now operator-adjustable per session (prompted in
# main()) rather than a fixed constant, since how naggy is "too naggy"
# depends on wafer size/setup; 100mm is just the starting default.
JOG_CHECKPOINT_INTERVAL_DEFAULT_MM = 100.0
JOG_CHECKPOINT_INTERVAL_MIN_MM = 10.0    # below this, the check fires so
                                          # often it stops being meaningful
                                          # (same "reflexive y" problem)
JOG_CHECKPOINT_INTERVAL_MAX_MM = 500.0   # above this, a real limit could be
                                          # hit and gone uncaught for too
                                          # long — matches this script's
                                          # largest sane single edge jog

try:
    import msvcrt  # Windows only
    _HAS_MSVCRT = True
except ImportError:
    _HAS_MSVCRT = False


# ---------------------------------------------------------------------------
# Jog loops — same motion.jog() calls underneath, different input methods
# ---------------------------------------------------------------------------

def _jog_checkpoint(moved_since_checkpoint_mm):
    """
    Force an explicit "are you still watching real motion?" confirmation
    once cumulative jogging since the last checkpoint reaches the
    operator-set checkpoint interval (see JOG_CHECKPOINT_INTERVAL_DEFAULT_MM).
    See that constant's docstring for why a single check at the start of
    a jog isn't enough — this is what extends the same protection to
    every jog step along the way, not just the first one.

    Raises RuntimeError (aborting the whole calibration) on "n", same
    policy as clearance_check() — a jog that isn't producing confirmed
    real motion means the step count is likely already desynced from
    true position, and continuing to jog on that basis only compounds it.
    """
    resp = input(
        f"  Moved ~{moved_since_checkpoint_mm:.1f}mm since the last check — "
        "still tracking real motion on the mirror? [Y/n] "
    ).strip().lower()
    if resp == "n":
        raise RuntimeError(
            "Jog checkpoint not confirmed — the stage may have run into a hard "
            "limit partway through this move and could now be silently "
            "miscounting position. Stopping rather than continuing to jog on "
            "an axis that may be stalled; check the hardware before re-running."
        )


def _jog_loop_msvcrt(motion, jog_step_mm, checkpoint_interval_mm=JOG_CHECKPOINT_INTERVAL_DEFAULT_MM):
    """Windows: real arrow keys via msvcrt. Returns confirmed (x_mm, y_mm)."""
    print("  Arrow keys to jog, +/- to change step size, ENTER to confirm, q to abort.")
    moved_since_checkpoint = 0.0
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
            moved = 0.0
            if arrow == b"H":      # up
                motion.jog(dy_mm=jog_step_mm)
                moved = jog_step_mm
            elif arrow == b"P":    # down
                motion.jog(dy_mm=-jog_step_mm)
                moved = jog_step_mm
            elif arrow == b"K":    # left
                motion.jog(dx_mm=-jog_step_mm)
                moved = jog_step_mm
            elif arrow == b"M":    # right
                motion.jog(dx_mm=jog_step_mm)
                moved = jog_step_mm
            moved_since_checkpoint += moved
            if moved_since_checkpoint >= checkpoint_interval_mm:
                print()
                _jog_checkpoint(moved_since_checkpoint)
                moved_since_checkpoint = 0.0


def _jog_loop_typed(motion, jog_step_mm, checkpoint_interval_mm=JOG_CHECKPOINT_INTERVAL_DEFAULT_MM):
    """Fallback jog loop for non-Windows terminals: typed commands."""
    print("  Commands: w/s = Y +/-, a/d = X +/-, +/- = change step size, "
          "c = confirm, q = abort. Enter after each command.")
    moved_since_checkpoint = 0.0
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
        elif cmd in ("w", "s", "a", "d"):
            if cmd == "w":
                motion.jog(dy_mm=jog_step_mm)
            elif cmd == "s":
                motion.jog(dy_mm=-jog_step_mm)
            elif cmd == "a":
                motion.jog(dx_mm=-jog_step_mm)
            elif cmd == "d":
                motion.jog(dx_mm=jog_step_mm)
            moved_since_checkpoint += jog_step_mm
            if moved_since_checkpoint >= checkpoint_interval_mm:
                _jog_checkpoint(moved_since_checkpoint)
                moved_since_checkpoint = 0.0
        else:
            print(f"  (unrecognized command {cmd!r})")


def jog_to_edge(motion, edge_name, jog_step_mm=JOG_STEP_DEFAULT_MM,
                 checkpoint_interval_mm=JOG_CHECKPOINT_INTERVAL_DEFAULT_MM):
    print(f"\n--- {edge_name.upper()} EDGE ---")
    print(f"  {EDGE_PROMPTS[edge_name]}")
    loop = _jog_loop_msvcrt if _HAS_MSVCRT else _jog_loop_typed
    x, y = loop(motion, jog_step_mm, checkpoint_interval_mm)
    print(f"  Recorded {edge_name} edge at ({x:.3f}, {y:.3f}) mm")
    return x, y


def clearance_check(motion, test_step_mm=CLEARANCE_CHECK_STEP_MM):
    """
    Small test jog in each of the 4 cardinal directions from wherever the
    stage is CURRENTLY sitting, requiring an explicit operator
    confirmation that the spot actually visibly moved each time.

    Why this exists: the picomotor is open-loop with no limit switches
    and no stall/force feedback (confirmed against pylablib's own docs —
    "these steps can be different depending on the direction, position,
    instantaneous load, speed... it is recommended to generally use
    relative positioning"). If the stage happens to already be sitting at
    a hard mechanical limit when a jog is issued, the controller has no
    way to know the move didn't actually happen — it just keeps counting
    commanded steps as if it did, silently desyncing the step count from
    true physical position from that point on, with no error raised
    anywhere. There is no software fix for this on this hardware: the
    only reliable check is a human confirming visible motion. This
    matters most right at the start of a session, before ANYTHING has
    moved yet and the true starting position relative to the limits is
    completely unknown — hence running this before the very first jog
    (the reference-mark jog in main()), not just before the wafer edges.

    Net displacement across all 4 jogs is zero by construction (+test,
    -test on X; +test, -test on Y), so a stage that passes this check
    ends up back where it started, confirmed-clear in every direction.

    Raises RuntimeError (aborting the whole calibration) rather than
    warning-and-continuing if any direction doesn't confirm — proceeding
    past an unconfirmed direction is exactly the "issuing moves onto an
    axis that can't actually make them" failure mode this exists to catch
    BEFORE it corrupts an entire session's worth of jogging, not after.
    """
    print("\n--- Clearance check ---")
    print(f"  Small test jog ({test_step_mm:.1f}mm) in each direction — confirm the")
    print("  spot ACTUALLY visibly moves each time. If it doesn't move in some")
    print("  direction, the stage is already at (or very near) a hard limit")
    print("  there — do not proceed; jog the other way, or check the hardware.")
    directions = [
        ("RIGHT (+X)", test_step_mm, 0.0),
        ("LEFT (-X)", -test_step_mm, 0.0),
        ("UP (+Y)", 0.0, test_step_mm),
        ("DOWN (-Y)", 0.0, -test_step_mm),
    ]
    for label, dx, dy in directions:
        motion.jog(dx_mm=dx, dy_mm=dy)
        resp = input(f"  Jogged {label}. Did the spot visibly move? [Y/n] ").strip().lower()
        if resp == "n":
            raise RuntimeError(
                f"No visible motion jogging {label} — the stage may already be "
                "at a hard limit in this direction. Stopping here rather than "
                "issuing further moves onto an axis in this state; check the "
                "hardware before re-running."
            )
    print("  Clearance confirmed in all 4 directions.")


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
        # Kept (not just consumed here) so compute_radius_mm() can use
        # them as an independent, operator-supplied wafer size — see
        # its docstring.
        "true_x_mm": true_x_mm,
        "true_y_mm": true_y_mm,
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


def compute_radius_mm(area: dict, spmm_result: dict | None) -> float:
    """
    Radius (mm) of the circular scan mask, centered on
    area["wafer_center_mm"] — see scan_params.in_radius() and
    scan_manager.generate_grid() for how this gets applied.

    Why a mask at all: x_range_mm/y_range_mm is a rectangular BOUNDING
    BOX around the 4 jogged wafer edges, not the wafer itself. A
    circular wafer inscribed in that box means the box's corners are, by
    construction, off-sample — measuring them wastes time on
    meaningless background/chuck readings, and (per the 2026-07-17
    diagnosis) they're also the largest-excursion grid points, the most
    likely place to exceed the mount's own separate mechanical travel
    limit regardless of what shape the sample is.

    Two candidate radii, take the LARGER — deliberately generous rather
    than conservative, so the mask never excludes real wafer area that
    either measurement suggests exists:
      - r_xy: half of whichever of the x_range/y_range spans (from the
        edge jogs themselves) is bigger. Always available.
      - r_diameter: half of whichever of the operator-entered true
        left-right / true bottom-top distances is bigger — only
        available if calibrate_steps_per_mm() ran (spmm_result is not
        None). This is an independently-measured wafer size (e.g. a
        caliper reading or known wafer spec), not derived from the jogs
        at all, so it can disagree with r_xy in either direction.
    """
    x0, x1 = area["x_range_mm"]
    y0, y1 = area["y_range_mm"]
    r_xy = max(abs(x1 - x0), abs(y1 - y0)) / 2.0

    if spmm_result is not None:
        r_diameter = max(spmm_result["true_x_mm"], spmm_result["true_y_mm"]) / 2.0
        return round(max(r_xy, r_diameter), CONFIG_RANGE_DECIMALS)

    return round(r_xy, CONFIG_RANGE_DECIMALS)


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


def validate_jog_checkpoint_interval_mm(value: float) -> float:
    """Raise ValueError if the jog checkpoint interval is outside the valid range."""
    value = float(value)
    if not (JOG_CHECKPOINT_INTERVAL_MIN_MM <= value <= JOG_CHECKPOINT_INTERVAL_MAX_MM):
        raise ValueError(
            f"checkpoint interval {value} outside valid range "
            f"[{JOG_CHECKPOINT_INTERVAL_MIN_MM}, {JOG_CHECKPOINT_INTERVAL_MAX_MM}] mm"
        )
    return value


def prompt_float(label, default, validator):
    while True:
        raw = input(f"  {label} [{default}]: ").strip()
        value = float(raw) if raw else default
        try:
            return validator(value)
        except ValueError as e:
            print(f"  {e} — try again.")


def prompt_int(label, default, validator):
    while True:
        raw = input(f"  {label} [{default}]: ").strip()
        try:
            value = int(raw) if raw else default
        except ValueError:
            print("  Enter a whole number — try again.")
            continue
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


def _patch_or_insert_scalar(text: str, key: str, new_value: str, after_key: str) -> str:
    """
    Like _patch_scalar, but if `key:` isn't found in the file at all
    (e.g. an on-site config.yaml predating a field this script later
    grew — wafer_radius_mm and passes were both added after this script's
    first release), INSERT a new `key: value` line immediately after the
    line for `after_key` instead of raising.

    Why this exists (2026-07-20 on-site incident): write_results() used
    to build the whole patched file in memory and only call
    config_path.write_text() once, at the very end. A single missing key
    (an operator's config.yaml still missing wafer_radius_mm) raised
    KeyError from _patch_scalar() partway through and aborted the whole
    function BEFORE that final write_text() call — so NONE of the
    already-computed x_range_mm/y_range_mm/wafer_center_mm/step_size_mm/
    steps_per_mm_x/y etc. got saved either, silently discarding an
    entire calibration session's results over one missing line, with no
    indication to the operator that everything else was also lost.
    Falling forward (insert rather than crash) for fields that are known
    to be newer/optional keeps that failure contained to just this one
    field instead of the whole write.
    """
    pattern = re.compile(
        rf"^([ \t]*{re.escape(key)}:)[ \t]*([^#\r\n]*?)[ \t]*(#.*)?$",
        re.MULTILINE,
    )
    if pattern.search(text):
        return _patch_scalar(text, key, new_value)

    anchor_pattern = re.compile(
        rf"^([ \t]*){re.escape(after_key)}:[^\r\n]*$",
        re.MULTILINE,
    )
    m = anchor_pattern.search(text)
    if not m:
        raise KeyError(
            f"Could not find a '{key}:' line to patch, and its insertion anchor "
            f"'{after_key}:' is also missing from the config file — this field "
            "was not written. Add it manually."
        )
    indent = m.group(1)
    insert_at = m.end()
    return text[:insert_at] + f"\n{indent}{key}: {new_value}" + text[insert_at:]


def write_results(config_path: Path, results: dict):
    """
    Patch config.yaml with the calibration results.

    `results` always has x_range_mm, y_range_mm, wafer_center_mm (each a
    [lo, hi] pair), wafer_radius_mm (scalar — see compute_radius_mm()),
    step_size_mm, dwell_time_s, and passes. It may also
    carry steps_per_mm_x/steps_per_mm_y (if calibrate_steps_per_mm ran)
    and home_steps (if the operator chose to write the suggested homing
    bound) — both optional scalars, patched the same way as everything
    else, one dict in rather than separate parameters.

    Each field is patched independently and failures are collected
    rather than raised immediately — see _patch_or_insert_scalar's
    docstring for why an all-or-nothing write is dangerous here. Whatever
    DID succeed is still written to disk even if some field failed, and
    the operator gets a clear report of exactly what didn't make it in
    (and why) instead of a stack trace and a config.yaml that silently
    still has last week's placeholder values.
    """
    text = config_path.read_text(encoding="utf-8-sig")
    failed = []

    def _try(fn, label, *args):
        nonlocal text
        try:
            text = fn(text, *args)
        except KeyError as e:
            failed.append((label, str(e)))

    for key in ("x_range_mm", "y_range_mm", "wafer_center_mm"):
        lo, hi = results[key]
        _try(_patch_scalar, key, key,
             f"[{lo:.{CONFIG_RANGE_DECIMALS}f}, {hi:.{CONFIG_RANGE_DECIMALS}f}]")

    # wafer_radius_mm and passes are newer fields (added 2026-07-17 and
    # 2026-07-15 respectively) that an older on-site config.yaml may not
    # have yet — insert rather than require they already exist.
    _try(_patch_or_insert_scalar, "wafer_radius_mm", "wafer_radius_mm",
         f"{results['wafer_radius_mm']:.{CONFIG_RANGE_DECIMALS}f}", "wafer_center_mm")

    _try(_patch_scalar, "step_size_mm", "step_size_mm", f"{results['step_size_mm']}")
    _try(_patch_scalar, "dwell_time_s", "dwell_time_s", f"{results['dwell_time_s']}")
    _try(_patch_or_insert_scalar, "passes", "passes", f"{results['passes']}", "dwell_time_s")

    if "steps_per_mm_x" in results:
        _try(_patch_scalar, "steps_per_mm_x", "steps_per_mm_x", f"{results['steps_per_mm_x']:.4f}")
        _try(_patch_scalar, "steps_per_mm_y", "steps_per_mm_y", f"{results['steps_per_mm_y']:.4f}")
        _try(_patch_or_insert_scalar, "calibration_date", "calibration_date",
             f'"{date.today().isoformat()}"', "steps_per_mm_y")

    if "home_steps" in results:
        _try(_patch_or_insert_scalar, "home_steps", "home_steps",
             f"{results['home_steps']}", "calibration_direction")

    config_path.write_text(text, encoding="utf-8")

    if failed:
        print(f"\nWrote PARTIAL calibration results to {config_path} — "
              f"{len(failed)} field(s) could NOT be written:")
        for label, msg in failed:
            print(f"  - {label}: {msg}")
        print("  Everything else above was saved. Add the missing line(s) to "
              "config.yaml by hand (see the values printed earlier in this run), "
              "then re-run calibration or edit config.yaml directly.")
    else:
        print(f"\nWrote calibration results to {config_path}")

    # Returned (in addition to the print()s above) so a non-console caller —
    # gui/calibration_panel.py's Calibrate tab — can show the operator the
    # same "which fields failed" detail in a dialog instead of only on
    # stdout. The __main__ flow below ignores this; existing behavior for
    # standalone `python scan/calibrate_scan_area.py` runs is unchanged.
    return failed


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

    checkpoint_interval_mm = prompt_float(
        f"Jog checkpoint interval mm — how far to jog before re-confirming "
        f"real motion (range {JOG_CHECKPOINT_INTERVAL_MIN_MM}-{JOG_CHECKPOINT_INTERVAL_MAX_MM})",
        JOG_CHECKPOINT_INTERVAL_DEFAULT_MM,
        validate_jog_checkpoint_interval_mm,
    )

    # Fiducial homing instead of driving into a mechanical hard stop:
    # avoids ever needing to characterize home_steps/home_velocity, avoids
    # any stall-contact risk/wear, and is only as trustworthy as an
    # operator visually confirming a fixed reference point — which is
    # exactly the same verification already relied on for the wafer-edge
    # jogs below, just applied to the origin too. See MEMORY
    # carat_scanner_2026-07-17_scan_diagnosis for why this replaced
    # motion.home()'s mechanical-limit-based approach here.
    #
    # zero_here() is called TWICE, deliberately:
    #  1. Right now, PROVISIONALLY — move_to()/jog() refuse to move at all
    #     until _homed is True, so relative jogging (the clearance check,
    #     then the reference-mark jog) isn't possible otherwise. This call
    #     doesn't move anything — it's pure bookkeeping ("call wherever we
    #     already are zero"), so unlike driving into a hard stop, it
    #     carries no boundary risk regardless of where the stage happens
    #     to be sitting. Not trusted for anything past letting us jog.
    #  2. Again below, for real, once the operator has visually confirmed
    #     we're actually at the fixed reference mark — THAT call is what
    #     the rest of this session's coordinates are anchored to.
    motion.zero_here()

    # clearance_check() runs before the reference-mark jog is trusted:
    # the stage's starting position this session is otherwise completely
    # unknown, and an open-loop axis can't tell software when a jog
    # silently fails against a limit it's already sitting at.
    clearance_check(motion)

    ref_x, ref_y = jog_to_edge(motion, "reference", checkpoint_interval_mm=checkpoint_interval_mm)
    motion.zero_here()
    print(f"  Origin zeroed at reference mark (was at {ref_x:.3f}, {ref_y:.3f} mm "
          "in the provisional frame).")

    edges = {}
    for edge_name in EDGE_ORDER:
        edges[edge_name] = jog_to_edge(motion, edge_name, checkpoint_interval_mm=checkpoint_interval_mm)

    spmm_result = calibrate_steps_per_mm(edges, config)
    if spmm_result is not None:
        edges = rescale_edges(edges, spmm_result)

    area = compute_area(edges)
    # Round once, here, to the exact precision write_results() persists —
    # see CONFIG_RANGE_DECIMALS above for why this must happen before the
    # preview below, not just at write time.
    area = {
        "x_range_mm": [round(v, CONFIG_RANGE_DECIMALS) for v in area["x_range_mm"]],
        "y_range_mm": [round(v, CONFIG_RANGE_DECIMALS) for v in area["y_range_mm"]],
        "wafer_center_mm": [round(v, CONFIG_RANGE_DECIMALS) for v in area["wafer_center_mm"]],
    }
    radius_mm = compute_radius_mm(area, spmm_result)

    print("\n--- Computed scan area ---")
    print(f"  X range: {area['x_range_mm']} mm")
    print(f"  Y range: {area['y_range_mm']} mm")
    print(f"  Wafer center: {area['wafer_center_mm']} mm")
    print(f"  Wafer radius (scan mask): {radius_mm} mm")

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
    passes = prompt_int(
        f"Number of full-grid passes (range {scan_params.PASSES_MIN}-{scan_params.PASSES_MAX}; "
        "1 = scan the grid once, >1 = revisit every point that many times over the scan, "
        "for drift/oscillation tracking)",
        scan_params.PASSES_DEFAULT, scan_params.validate_passes,
    )

    nx, ny = scan_params.grid_dims_from_range(area["x_range_mm"], area["y_range_mm"], step_size_mm)

    # Build the SAME grid scan_manager.py will actually run (bounding box
    # + circular mask), not just nx*ny, so this preview can't overstate
    # point count/scan time the way the pre-mask preview did — same
    # motivation as CONFIG_RANGE_DECIMALS above: preview and execution
    # must derive from identical logic, not two separately-computed
    # numbers that happen to usually agree.
    from scan.scan_manager import generate_grid
    preview_scan_cfg = {
        "grid": {
            "x_range_mm": area["x_range_mm"],
            "y_range_mm": area["y_range_mm"],
            "wafer_center_mm": area["wafer_center_mm"],
            "wafer_radius_mm": radius_mm,
            "step_size_mm": step_size_mm,
        },
        "scan_order": config.get("scan", {}).get("scan_order", "raster"),
    }
    masked_points, _, _ = generate_grid(preview_scan_cfg)
    n_points = len(masked_points)
    est_s = n_points * passes * dwell_time_s

    print("\n--- Preview ---")
    print(f"  Bounding box: {nx} x {ny} = {nx * ny} grid positions")
    print(f"  Within wafer radius ({radius_mm} mm): {n_points} points per pass "
          f"({nx * ny - n_points} corner/off-wafer positions excluded)")
    print(f"  Passes: {passes}")
    print(f"  Estimated scan time: {est_s / 60:.1f} min "
          f"({n_points * passes} total point measurements, excluding reference-point revisits)")

    if input("\nWrite these values to config.yaml? [y/N] ").strip().lower() != "y":
        print("Not written. Re-run to try again.")
        return

    results = dict(area)
    results["wafer_radius_mm"] = radius_mm
    results["step_size_mm"] = step_size_mm
    results["dwell_time_s"] = dwell_time_s
    results["passes"] = passes
    if spmm_result is not None:
        results["steps_per_mm_x"] = spmm_result["steps_per_mm_x"]
        results["steps_per_mm_y"] = spmm_result["steps_per_mm_y"]
    if recommended_home_steps is not None:
        results["home_steps"] = recommended_home_steps
    write_results(config_path, results)

    if input("\nRun the scan now using this calibration? [y/N] ").strip().lower() == "y":
        # Reload config.yaml so the scan sees the just-written range/
        # step_size/dwell_time/passes exactly as a standalone
        # `python scan/scan_manager.py` run would — but reuse THIS SAME
        # `motion` object (already connected, already homed at the top
        # of this script) instead of letting ScanManager build a second
        # connection and re-home. Since the picomotor is open-loop, a
        # second home() would cost another full home_steps/home_velocity
        # drive-to-stop with no way for the hardware to short-circuit
        # "already there" — pure wasted time for a controller that's
        # already sitting at a verified origin. See ScanManager's
        # `motion=`/`already_homed=` params for why reusing the object
        # (not just trusting a fresh one) is what makes this safe.
        from scan.scan_manager import ScanManager
        scan_config = yaml.safe_load(config_path.read_text(encoding="utf-8-sig"))
        print("\nStarting scan with this calibration...")
        manager = ScanManager(scan_config, motion=motion)
        manager.run(already_homed=True)


if __name__ == "__main__":
    main()
