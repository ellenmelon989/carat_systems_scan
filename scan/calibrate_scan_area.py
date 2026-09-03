"""
calibrate_scan_area.py

Interactive scanner-area calibration: the operator jogs the aim spot to
the four wafer edges, and this tool derives the scan center and range
from those positions, then collects dwell time and step size and writes
all of it into config.yaml.

Run this after installation, or whenever the scanner is physically moved.

DEGREE-FIRST, for the CONEX-AGAP: every jog in this script (clearance
check, reference mark, 4 wafer edges) commands the controller directly
in native degrees (calibration_jog_deg()/get_position_deg()) — it never
multiplies by deg_per_mm_x/y until AFTER the edges are recorded. That's
deliberate: the CONEX's whole travel is only about +/-0.76 degrees, so a
jog computed through an unknown or placeholder deg_per_mm (as this script
used to do, in mm, before 2026-08) can overshoot the real limit by
orders of magnitude on the very first move. Jogging in degrees can't do
that — every target is checked against the same ~1 degree limit it's
already expressed in. See calibrate_deg_per_mm() for how the mm
conversion factor gets derived (or reused) only once, at the end, from
a real measured distance instead of a guess.

Usage
-----
    python scan/calibrate_scan_area.py [config.yaml]

Workflow
--------
1. Manual checklist: wafer or calibration target installed, scanner
   mounted, pyrometer aim light ON. There is no software control of the
   aim light in this codebase yet, so this is a human checklist step,
   not an automated one.
2. Clearance check: small test jogs (in degrees) in all 4 directions from
   wherever the stage is currently sitting, confirmed by eye — the
   controller itself already rejects any out-of-limit target, so this is
   now a sanity check on the optical path (blocked beam, wrong axis
   mapping), not a defense against a silent step-count desync.
3. Jog (in degrees) to a fixed reference mark and zero the origin there
   (fiducial homing) — not a drive into a mechanical hard stop; there
   isn't one on this controller (see real_conexagap_motion.py's HOMING /
   ORIGIN section). Trustworthy only insofar as the operator confirms
   they've actually reached the mark, same as any wafer edge below.
4. Jog (in degrees) to each of 4 wafer edges (left, right, top, bottom)
   and confirm:
     - On Windows: real arrow keys via msvcrt (Left/Right = X, Up/Down = Y).
     - Elsewhere (this dev sandbox, Mac/Linux terminals): typed w/a/s/d +
       Enter. Raw arrow-key capture is OS-specific and this is a command-
       line lab-instrument script rather than a GUI, so the fallback keeps
       it dependency-free; both paths call the same motion.calibration_jog_deg().
5. Derive deg_per_mm_x/y from those same 4 edges (see
   calibrate_deg_per_mm()) — no new hardware call needed, and no assumed
   prior ratio: the degree separation between opposite edges comes
   straight from the live encoder. Divide that by a TRUE,
   independently-known distance (a wafer of known diameter, a caliper
   measurement) and that's the real ratio. Leave the prompts blank to
   instead reuse whatever deg_per_mm_x/y is already in config.yaml (only
   valid once a real calibration has been done at least once).
   Caveat: this is one sample from an organic back-and-forth jog, not an
   averaged, single-direction measurement — good for a quick or first
   pass, not final precision.
6. Convert the recorded edges from degrees to mm using deg_per_mm_x/y
   (see edges_deg_to_mm()), then compute wafer center + scan range from
   those 4 positions. Also compute a wafer radius (see
   compute_radius_mm()) so the scan can mask its rectangular bounding box
   down to the wafer's actual circular footprint — the box's corners are
   off-sample by construction for a round wafer, and are also the
   largest-excursion grid points, the likeliest place to exceed the
   mount's own travel regardless of what shape the sample is.
7. Prompt for step size (mm) and dwell time (s), validated against
   scan_params' operator-valid ranges.
8. Preview the resulting grid size and a rough total scan time.
9. Write everything collected back into config.yaml. This is a
   targeted line-level patch, not a full YAML re-dump — it preserves
   the file's hand-written comments, which a plain yaml.safe_dump
   round-trip would strip.
10. Optionally continue straight into a scan using this calibration,
    reusing the same already-homed motion connection (see the "Run the
    scan now?" prompt at the end of main()) — one origin per session,
    not one per script.

Note: motion.soft_limits (mm) is intentionally NOT enforced while
jogging — this process is what defines the safe scan area, so it can't
already be constrained by it, and jogging happens in degrees anyway
(see above). The controller's own live SL/SR limits (degrees) are the
only real limit at this stage; watch the mirror.

8742 note: this script now assumes a controller with native-degree
calibration support (calibration_jog_deg()/get_position_deg()) — i.e.
ConexAGAPController. Running this against the 8742 driver
(NewportPicomotorController, which has no rotational degrees to jog in)
will fail with a clear NotImplementedError from
MotionController.calibration_jog_deg()'s base implementation. The 8742's
own steps_per_mm calibration flow (--calibrate-x/-y) still lives in
real_newport_motion.py's __main__ block if that driver is ever needed
again.
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

JOG_STEP_DEFAULT_DEG = 0.05
JOG_STEP_MIN_DEG = 0.005
JOG_STEP_MAX_DEG = 0.3

# Minimum degree separation between a pair of opposite edges (left/right or
# top/bottom) for calibrate_deg_per_mm() to trust it as a real measurement.
# Below this, treat it as "the operator didn't actually jog between the two
# edges" rather than a genuine near-zero span -- dividing a true distance by
# ~0 produces a ~0 deg_per_mm, which then makes edges_deg_to_mm() divide BY
# that ~0 and raise an unhandled ZeroDivisionError several steps later, far
# from the actual mistake. Set well below JOG_STEP_MIN_DEG so one real jog
# step of any size still passes. See MEMORY
# carat_scanner_2026-08-06_calibration_zero_separation_crash.
MIN_EDGE_SEPARATION_DEG = 0.001

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

# How far (degrees) each direction the clearance check jogs to confirm the
# stage isn't already sitting at a hard limit before anything else moves.
# Unlike the 8742 (no stall/limit feedback at all, hence needing a human
# visual check to catch a silently-failed move), the CONEX-AGAP DOES reject
# any target outside its live SL/SR limits before sending anything -- so
# this check can no longer be silently defeated by a stale step count. It's
# kept anyway as a cheap sanity check that the beam is doing what's
# commanded (optics can still be blocked/misaligned in ways the encoder
# can't see), sized to be unambiguous to the eye without risking much of
# the ~1.5 degree total travel.
CLEARANCE_CHECK_STEP_DEG = 0.1

# How far (degrees) of CUMULATIVE jogging within a single jog_to_edge() call
# (reference mark OR any wafer edge) is allowed before the loop forces a
# "are you still watching real motion?" checkpoint. Every individual target
# is already validated against the controller's live limits (see
# ConexAGAPController._validate_axis_target) before it's ever sent, so
# unlike the 8742 this can't mask a runaway desync -- it's purely a human
# sanity check that the beam is visibly tracking commanded motion, kept
# from the original 8742 workflow. Scaled down from the 8742's mm-range
# defaults to fit the CONEX's much smaller (~1.5 degree total) travel.
JOG_CHECKPOINT_INTERVAL_DEFAULT_DEG = 0.3
JOG_CHECKPOINT_INTERVAL_MIN_DEG = 0.05   # below this, the check fires so
                                          # often it stops being meaningful
                                          # (same "reflexive y" problem)
JOG_CHECKPOINT_INTERVAL_MAX_DEG = 0.76   # above this, a real limit could be
                                          # hit and gone uncaught for too
                                          # long -- matches the controller's
                                          # own single-direction travel

try:
    import msvcrt  # Windows only
    _HAS_MSVCRT = True
except ImportError:
    _HAS_MSVCRT = False


# ---------------------------------------------------------------------------
# Jog loops — same motion.calibration_jog() calls underneath, different input methods
# ---------------------------------------------------------------------------

def _jog_checkpoint(moved_since_checkpoint_deg):
    """
    Force an explicit "are you still watching real motion?" confirmation
    once cumulative jogging since the last checkpoint reaches the
    operator-set checkpoint interval (see JOG_CHECKPOINT_INTERVAL_DEFAULT_DEG).
    See that constant's docstring for why a single check at the start of
    a jog isn't enough — this is what extends the same protection to
    every jog step along the way, not just the first one.

    Raises RuntimeError (aborting the whole calibration) on "n" — a jog
    that isn't producing confirmed real motion on the mirror means
    something upstream of the (already limit-checked) motor command is
    wrong (optics blocked/misaligned, wrong axis), and continuing to jog
    on that basis only wastes the rest of the session.
    """
    resp = input(
        f"  Moved ~{moved_since_checkpoint_deg:.2f} deg since the last check — "
        "still tracking real motion on the mirror? [Y/n] "
    ).strip().lower()
    if resp == "n":
        raise RuntimeError(
            "Jog checkpoint not confirmed — the commanded motion isn't "
            "visibly reaching the wafer. Stopping rather than continuing to "
            "jog blind; check the optical path (aim light, blocked beam, "
            "wrong axis/mirror) before re-running."
        )


def _jog_loop_msvcrt(motion, jog_step_deg, checkpoint_interval_deg=JOG_CHECKPOINT_INTERVAL_DEFAULT_DEG):
    """Windows: real arrow keys via msvcrt. Returns confirmed (x_deg, y_deg)."""
    print("  Arrow keys to jog, +/- to change step size, ENTER to confirm, q to abort.")
    moved_since_checkpoint = 0.0
    while True:
        x, y = motion.get_position_deg()
        print(f"  step={jog_step_deg:.3f}deg  pos=({x:.4f}, {y:.4f}) deg   ", end="\r")
        ch = msvcrt.getch()
        if ch in (b"\r", b"\n"):
            print()
            return x, y
        if ch in (b"q", b"Q"):
            print()
            raise KeyboardInterrupt("Calibration aborted by operator")
        if ch == b"+":
            jog_step_deg = min(JOG_STEP_MAX_DEG, jog_step_deg * 2)
            continue
        if ch == b"-":
            jog_step_deg = max(JOG_STEP_MIN_DEG, jog_step_deg / 2)
            continue
        if ch == b"\xe0":  # arrow-key prefix on Windows
            arrow = msvcrt.getch()
            moved = 0.0
            if arrow == b"H":      # up
                motion.calibration_jog_deg(dy_deg=jog_step_deg)
                moved = jog_step_deg
            elif arrow == b"P":    # down
                motion.calibration_jog_deg(dy_deg=-jog_step_deg)
                moved = jog_step_deg
            elif arrow == b"K":    # left
                motion.calibration_jog_deg(dx_deg=-jog_step_deg)
                moved = jog_step_deg
            elif arrow == b"M":    # right
                motion.calibration_jog_deg(dx_deg=jog_step_deg)
                moved = jog_step_deg
            moved_since_checkpoint += moved
            if moved_since_checkpoint >= checkpoint_interval_deg:
                print()
                _jog_checkpoint(moved_since_checkpoint)
                moved_since_checkpoint = 0.0


def _jog_loop_typed(motion, jog_step_deg, checkpoint_interval_deg=JOG_CHECKPOINT_INTERVAL_DEFAULT_DEG):
    """Fallback jog loop for non-Windows terminals: typed commands."""
    print("  Commands: w/s = Y +/-, a/d = X +/-, +/- = change step size, "
          "c = confirm, q = abort. Enter after each command.")
    moved_since_checkpoint = 0.0
    while True:
        x, y = motion.get_position_deg()
        print(f"  step={jog_step_deg:.3f}deg  pos=({x:.4f}, {y:.4f}) deg")
        cmd = input("  > ").strip().lower()
        if cmd == "c":
            return x, y
        if cmd == "q":
            raise KeyboardInterrupt("Calibration aborted by operator")
        if cmd == "+":
            jog_step_deg = min(JOG_STEP_MAX_DEG, jog_step_deg * 2)
        elif cmd == "-":
            jog_step_deg = max(JOG_STEP_MIN_DEG, jog_step_deg / 2)
        elif cmd in ("w", "s", "a", "d"):
            if cmd == "w":
                motion.calibration_jog_deg(dy_deg=jog_step_deg)
            elif cmd == "s":
                motion.calibration_jog_deg(dy_deg=-jog_step_deg)
            elif cmd == "a":
                motion.calibration_jog_deg(dx_deg=-jog_step_deg)
            elif cmd == "d":
                motion.calibration_jog_deg(dx_deg=jog_step_deg)
            moved_since_checkpoint += jog_step_deg
            if moved_since_checkpoint >= checkpoint_interval_deg:
                _jog_checkpoint(moved_since_checkpoint)
                moved_since_checkpoint = 0.0
        else:
            print(f"  (unrecognized command {cmd!r})")


def jog_to_edge(motion, edge_name, jog_step_deg=JOG_STEP_DEFAULT_DEG,
                 checkpoint_interval_deg=JOG_CHECKPOINT_INTERVAL_DEFAULT_DEG):
    print(f"\n--- {edge_name.upper()} EDGE ---")
    print(f"  {EDGE_PROMPTS[edge_name]}")
    loop = _jog_loop_msvcrt if _HAS_MSVCRT else _jog_loop_typed
    x, y = loop(motion, jog_step_deg, checkpoint_interval_deg)
    print(f"  Recorded {edge_name} edge at ({x:.4f}, {y:.4f}) deg")
    return x, y


def clearance_check(motion, test_step_deg=CLEARANCE_CHECK_STEP_DEG):
    """
    Small test jog in each of the 4 cardinal directions from wherever the
    stage is CURRENTLY sitting, requiring an explicit operator
    confirmation that the spot actually visibly moved each time.

    Unlike the 8742 (open-loop, no limit switches or stall feedback --
    a jog issued at a hard mechanical limit was silently absorbed with
    no error, desyncing the step count from true position from then on),
    the CONEX-AGAP validates every target against its live SL/SR limits
    in software before sending anything, and reports a real MotionFault
    rather than silently failing. So this check can no longer mask a
    runaway desync -- it's kept as a cheap sanity check that the optical
    path is actually working (a correctly-commanded, in-limits move can
    still fail to produce a visible spot if the beam is blocked,
    misaligned, or the wrong axis letter is mapped to X/Y), run once at
    the start of a session before the reference-mark jog in main().

    Net displacement across all 4 jogs is zero by construction (+test,
    -test on X; +test, -test on Y), so a stage that passes this check
    ends up back where it started, confirmed-clear in every direction.

    Raises RuntimeError (aborting the whole calibration) rather than
    warning-and-continuing if any direction doesn't confirm real visible
    motion -- that means something is wrong upstream of the motor
    command itself, worth stopping to investigate before jogging blind
    for the rest of the session.
    """
    print("\n--- Clearance check ---")
    print(f"  Small test jog ({test_step_deg:.2f} deg) in each direction — confirm the")
    print("  spot ACTUALLY visibly moves each time. If it doesn't, the optical path")
    print("  may be blocked/misaligned, or axis_x/axis_y may be mapped wrong --")
    print("  do not proceed; check before continuing.")
    directions = [
        ("RIGHT (+X)", test_step_deg, 0.0),
        ("LEFT (-X)", -test_step_deg, 0.0),
        ("UP (+Y)", 0.0, test_step_deg),
        ("DOWN (-Y)", 0.0, -test_step_deg),
    ]
    for label, dx, dy in directions:
        motion.calibration_jog_deg(dx_deg=dx, dy_deg=dy)
        resp = input(f"  Jogged {label}. Did the spot visibly move? [Y/n] ").strip().lower()
        if resp == "n":
            raise RuntimeError(
                f"No visible motion jogging {label} — check the optical path "
                "(aim light, blocked beam) and axis_x/axis_y mapping before "
                "re-running. Stopping here rather than continuing to jog on "
                "an axis that isn't visibly doing what's commanded."
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


def calibrate_deg_per_mm(edges_deg: dict, config: dict):
    """
    Derive deg_per_mm_x/y directly from the same 4 edge positions (in
    DEGREES, from calibration_jog_deg()/get_position_deg()) used for the
    wafer-area calibration: divide the measured degree separation between
    opposite edges by a TRUE, independently known real-world distance (a
    wafer of known diameter, a caliper measurement).

    This replaces the old calibrate_steps_per_mm()/rescale_edges() pair,
    which had to first convert raw motor steps into mm using whatever
    ratio was ALREADY in config (a chicken-and-egg problem when that
    ratio isn't known yet). There's no such intermediate here: the
    degree separation comes straight from the live encoder, independent
    of any prior deg_per_mm guess.

    Optional — if the operator leaves either prompt blank, reuses the
    CURRENT config.yaml deg_per_mm_x/y as-is to convert this session's
    edges to mm (see edges_deg_to_mm()) rather than deriving a fresh
    value. Raises ValueError if neither a true distance is given NOR a
    usable existing config value exists — a first-time calibration on a
    fresh setup has to provide at least one real measurement.

    Caveat: this reuses the operator's organic back-and-forth edge jog,
    not a clean single-direction sweep, so backlash/hysteresis aren't
    averaged out the way a dedicated repeated single-direction
    measurement would be — good for a quick or first-pass calibration,
    not final precision.
    """
    motion_cfg = config.get("motion", {})
    current_x = motion_cfg.get("deg_per_mm_x")
    current_y = motion_cfg.get("deg_per_mm_y")
    current_x = float(current_x) if current_x else None
    current_y = float(current_y) if current_y else None

    x_left, _ = edges_deg["left"]
    x_right, _ = edges_deg["right"]
    _, y_top = edges_deg["top"]
    _, y_bottom = edges_deg["bottom"]

    deg_left_right = abs(x_right - x_left)
    deg_bottom_top = abs(y_top - y_bottom)

    print("\n--- deg/mm calibration ---")
    print("Reuses the left/right and top/bottom edges you just jogged to, in degrees.")
    print(f"  Left-to-right jog spanned {deg_left_right:.4f} deg (X axis).")
    print(f"  Bottom-to-top jog spanned {deg_bottom_top:.4f} deg (Y axis).")
    print("Enter a TRUE real-world distance for each (e.g. wafer diameter, or a")
    print("caliper measurement) to compute deg_per_mm directly from this session.")
    print(f"Leave blank to reuse the current config value instead "
          f"(X={current_x}, Y={current_y}).")

    true_x_raw = input("  True left-to-right distance, mm (e.g. wafer diameter) [skip]: ").strip()
    true_y_raw = input("  True bottom-to-top distance, mm [skip]: ").strip()

    if not true_x_raw or not true_y_raw:
        if current_x is None or current_y is None:
            raise ValueError(
                "No true distance entered, and config.yaml has no existing "
                "deg_per_mm_x/y to fall back to — enter a real distance for "
                "at least the first calibration on a new/moved setup."
            )
        print("  Skipped — reusing current config deg_per_mm_x/y.")
        return {
            "deg_per_mm_x": current_x,
            "deg_per_mm_y": current_y,
            "recalibrated": False,
        }

    true_x_mm = float(true_x_raw)
    true_y_mm = float(true_y_raw)

    # Guard BEFORE dividing: a near-zero measured span means left/right (or
    # top/bottom) were confirmed at essentially the same spot -- almost
    # always a jog that was skipped, not a real near-zero wafer dimension.
    # Left unguarded, this produces a near-zero deg_per_mm that then makes
    # edges_deg_to_mm() divide BY it and crash with an unhandled
    # ZeroDivisionError -- confusing because the crash lands several steps
    # after, and after any log of, the actual mistake.
    if deg_left_right < MIN_EDGE_SEPARATION_DEG:
        raise ValueError(
            f"Left/right edges are only {deg_left_right:.4f} deg apart -- "
            "that's within jog noise of zero, so this looks like the X jog "
            "was skipped rather than a real measurement. Re-run and "
            "actually jog to each edge (watch the printed position change) "
            "before confirming."
        )
    if deg_bottom_top < MIN_EDGE_SEPARATION_DEG:
        raise ValueError(
            f"Top/bottom edges are only {deg_bottom_top:.4f} deg apart -- "
            "that's within jog noise of zero, so this looks like the Y jog "
            "was skipped rather than a real measurement. Re-run and "
            "actually jog to each edge (watch the printed position change) "
            "before confirming."
        )

    new_deg_per_mm_x = deg_left_right / true_x_mm
    new_deg_per_mm_y = deg_bottom_top / true_y_mm

    print(f"  deg_per_mm_x: {current_x} -> {new_deg_per_mm_x:.4f}")
    print(f"  deg_per_mm_y: {current_y} -> {new_deg_per_mm_y:.4f}")

    return {
        "deg_per_mm_x": new_deg_per_mm_x,
        "deg_per_mm_y": new_deg_per_mm_y,
        # Kept (not just consumed here) so compute_radius_mm() can use
        # them as an independent, operator-supplied wafer size — see
        # its docstring.
        "true_x_mm": true_x_mm,
        "true_y_mm": true_y_mm,
        "recalibrated": True,
    }


def edges_deg_to_mm(edges_deg: dict, deg_per_mm_x: float, deg_per_mm_y: float) -> dict:
    """
    Convert recorded edge positions from degrees (relative to this
    session's zeroed origin) to mm, using deg_per_mm_x/y — either just
    freshly measured by calibrate_deg_per_mm(), or reused from
    config.yaml.

    Replaces the old rescale_edges(), which corrected an edge that had
    ALREADY been recorded in mm (via some prior ratio) for a newly
    calibrated ratio. Here the edges were always in degrees — this is
    the one and only conversion to mm, not a correction of an earlier one.
    """
    return {name: (x / deg_per_mm_x, y / deg_per_mm_y) for name, (x, y) in edges_deg.items()}


def compute_radius_mm(area: dict, deg_per_mm_result: dict | None) -> float:
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
        available if calibrate_deg_per_mm() actually recalibrated this
        session (deg_per_mm_result["recalibrated"] is True; reusing an
        existing config ratio has no fresh independent measurement to
        offer here). This is an independently-measured wafer size (e.g.
        a caliper reading or known wafer spec), not derived from the
        jogs at all, so it can disagree with r_xy in either direction.
    """
    x0, x1 = area["x_range_mm"]
    y0, y1 = area["y_range_mm"]
    r_xy = max(abs(x1 - x0), abs(y1 - y0)) / 2.0

    if deg_per_mm_result is not None and "true_x_mm" in deg_per_mm_result:
        r_diameter = max(deg_per_mm_result["true_x_mm"], deg_per_mm_result["true_y_mm"]) / 2.0
        return round(max(r_xy, r_diameter), CONFIG_RANGE_DECIMALS)

    return round(r_xy, CONFIG_RANGE_DECIMALS)


def recommend_home_steps(edges: dict, spmm_result: dict, margin: float = 2.0) -> int:
    """
    8742-ONLY, unused by the CONEX degree-first calibration flow below.

    Suggest a tighter home_steps bound than the 100,000-step blind
    default, based on how far this session's edges actually got from
    the current origin (in corrected steps, via spmm_result). home_steps/
    home_velocity govern driving into a mechanical hard stop — a concept
    that doesn't exist for the CONEX-AGAP (see real_conexagap_motion.py's
    HOMING / ORIGIN section), so this has nothing to compute for that
    controller. Kept only in case the 8742 driver is ever brought back
    (see real_newport_motion.py); expects spmm_result["steps_per_mm_x/y"],
    NOT the CONEX flow's deg_per_mm_result dict.

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


def validate_jog_checkpoint_interval_deg(value: float) -> float:
    """Raise ValueError if the jog checkpoint interval is outside the valid range."""
    value = float(value)
    if not (JOG_CHECKPOINT_INTERVAL_MIN_DEG <= value <= JOG_CHECKPOINT_INTERVAL_MAX_DEG):
        raise ValueError(
            f"checkpoint interval {value} outside valid range "
            f"[{JOG_CHECKPOINT_INTERVAL_MIN_DEG}, {JOG_CHECKPOINT_INTERVAL_MAX_DEG}] deg"
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
    carry deg_per_mm_x/deg_per_mm_y (if calibrate_deg_per_mm() actually
    recalibrated this session) and home_steps (8742-only, if the
    operator chose to write the suggested homing bound) — both optional
    scalars, patched the same way as everything else, one dict in
    rather than separate parameters.

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

    if "deg_per_mm_x" in results:
        _try(_patch_scalar, "deg_per_mm_x", "deg_per_mm_x", f"{results['deg_per_mm_x']:.6f}")
        _try(_patch_scalar, "deg_per_mm_y", "deg_per_mm_y", f"{results['deg_per_mm_y']:.6f}")
        _try(_patch_or_insert_scalar, "calibration_date", "calibration_date",
             f'"{date.today().isoformat()}"', "deg_per_mm_y")
        # NOTE 2026-08-06: this block briefly also wrote calibration_confirmed:
        # true here (see MEMORY carat_scanner_2026-08-06_calibration_confirmed_not_written
        # for why that was added). Removed again the same day per explicit
        # operator decision to drop the calibration_confirmed interlock
        # entirely -- see MEMORY
        # carat_scanner_2026-08-06_calibration_confirmed_guard_removed and
        # ConexAGAPController._require_motion_permission()'s docstring.
        # This function's OWN job -- measuring and writing real deg_per_mm_x/y
        # -- is unchanged; only the now-irrelevant flag write was removed.

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

    checkpoint_interval_deg = prompt_float(
        f"Jog checkpoint interval, deg — how far to jog before re-confirming "
        f"real motion (range {JOG_CHECKPOINT_INTERVAL_MIN_DEG}-{JOG_CHECKPOINT_INTERVAL_MAX_DEG})",
        JOG_CHECKPOINT_INTERVAL_DEFAULT_DEG,
        validate_jog_checkpoint_interval_deg,
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

    # clearance_check() runs before the reference-mark jog is trusted: the
    # stage's starting position this session is otherwise unconfirmed, and
    # this is the first real motion of the session.
    clearance_check(motion)

    ref_x, ref_y = jog_to_edge(motion, "reference", checkpoint_interval_deg=checkpoint_interval_deg)
    motion.zero_here()
    print(f"  Origin zeroed at reference mark (was at {ref_x:.4f}, {ref_y:.4f} deg "
          "in the provisional frame).")

    edges_deg = {}
    for edge_name in EDGE_ORDER:
        edges_deg[edge_name] = jog_to_edge(
            motion, edge_name, checkpoint_interval_deg=checkpoint_interval_deg
        )

    # Degrees straight from the live encoder -- no assumed deg_per_mm needed
    # to get here safely. This step is what turns those degrees into a true
    # mm distance: either freshly measured from a real wafer diameter, or
    # by reusing the ratio already in config.yaml.
    deg_per_mm_result = calibrate_deg_per_mm(edges_deg, config)
    edges_mm = edges_deg_to_mm(
        edges_deg, deg_per_mm_result["deg_per_mm_x"], deg_per_mm_result["deg_per_mm_y"]
    )

    area = compute_area(edges_mm)
    # Round once, here, to the exact precision write_results() persists —
    # see CONFIG_RANGE_DECIMALS above for why this must happen before the
    # preview below, not just at write time.
    area = {
        "x_range_mm": [round(v, CONFIG_RANGE_DECIMALS) for v in area["x_range_mm"]],
        "y_range_mm": [round(v, CONFIG_RANGE_DECIMALS) for v in area["y_range_mm"]],
        "wafer_center_mm": [round(v, CONFIG_RANGE_DECIMALS) for v in area["wafer_center_mm"]],
    }
    radius_mm = compute_radius_mm(area, deg_per_mm_result)

    print("\n--- Computed scan area ---")
    print(f"  X range: {area['x_range_mm']} mm")
    print(f"  Y range: {area['y_range_mm']} mm")
    print(f"  Wafer center: {area['wafer_center_mm']} mm")
    print(f"  Wafer radius (scan mask): {radius_mm} mm")

    # No "homing sweep bound" step here: home_steps/home_velocity are
    # 8742-specific (see recommend_home_steps()'s docstring) and don't
    # apply to the CONEX-AGAP, which has no mechanical-hard-stop homing.

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
    settle_time_s = config.get("scan", {}).get("settle_time_s", 0.0)
    # n_points is the WAFER-MASKED count -- pass it as nx with ny=1, same
    # call shape as gui/calibration_panel.py's preview, so both reuse
    # estimate_scan_time_s() without overstating time the way the unmasked
    # nx*ny bounding box would. See scan_params.py.
    est_s = scan_params.estimate_scan_time_s(n_points, 1, dwell_time_s, settle_time_s, passes)

    print("\n--- Preview ---")
    print(f"  Bounding box: {nx} x {ny} = {nx * ny} grid positions")
    print(f"  Within wafer radius ({radius_mm} mm): {n_points} points per pass "
          f"({nx * ny - n_points} corner/off-wafer positions excluded)")
    print(f"  Passes: {passes}")
    print(f"  Estimated scan time: {est_s / 60:.1f} min "
          f"({n_points * passes} total point measurements, excluding reference-point revisits)")

    if est_s > scan_params.SCAN_TIME_WARNING_THRESHOLD_S:
        print(f"  *** WARNING: estimated scan time exceeds the "
              f"{scan_params.SCAN_TIME_WARNING_THRESHOLD_S / 60:.0f}-min target. "
              "Consider a larger step size, shorter dwell time, or fewer passes. ***")

    if input("\nWrite these values to config.yaml? [y/N] ").strip().lower() != "y":
        print("Not written. Re-run to try again.")
        return

    results = dict(area)
    results["wafer_radius_mm"] = radius_mm
    results["step_size_mm"] = step_size_mm
    results["dwell_time_s"] = dwell_time_s
    results["passes"] = passes
    # Only write deg_per_mm_x/y back if this session actually recalibrated
    # them from a real measurement -- reusing the existing config value has
    # nothing new to persist, and shouldn't bump calibration_date to today.
    if deg_per_mm_result["recalibrated"]:
        results["deg_per_mm_x"] = deg_per_mm_result["deg_per_mm_x"]
        results["deg_per_mm_y"] = deg_per_mm_result["deg_per_mm_y"]
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
