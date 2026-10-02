"""
mr1530_onsite.py -- on-site switchover helper for the Optotune MR-15-30.

Written for on-site staff with no code knowledge: every command is run
exactly as written, needs no file editing, prints a clear PASS / CHECK /
FAIL banner plus the next step, and saves a full transcript (including
every answer typed) under switchover_results/ so it can be reviewed later
without anyone being on a call.

Run from the repo root with the venv active:

    python tools\\mr1530_onsite.py check        # talk to the mirror, NO motion
    python tools\\mr1530_onsite.py reliability  # 200 random moves (~30 s)
    python tools\\mr1530_onsite.py laser        # guided laser-spot angle check
    python tools\\mr1530_onsite.py trial-on     # back up config.yaml, point it at the mirror
    python tools\\mr1530_onsite.py trial-off    # put the backed-up config.yaml back

Add --port COMn if the MR-E-3 is not on COM7.

`check`, `reliability` and `laser` never read or write config.yaml -- they
build their own connection settings. Only trial-on / trial-off touch it,
always keeping a timestamped backup.

Safety: follows the driver's v4 freeze policy. If a settle times out
(AxisStateUnknown), nothing here sends another move -- including the
end-of-run "park at centre".
"""

from __future__ import annotations

import argparse
import datetime as _dt
import logging
import math
import random
import re
import shutil
import statistics
import sys
import time
import traceback
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import yaml  # noqa: E402

import motion.real_mr1530_motion as mr  # noqa: E402
from motion.motion_controller import AxisStateUnknown, get_motion_controller  # noqa: E402
from motion.real_mr1530_motion import MR1530Controller  # noqa: E402

DEFAULT_PORT = "COM7"
TAN50 = math.tan(math.radians(50.0))
_FAULT_MASK = (
    (1 << mr._STATUS_BIT_CURRENT_LIMIT)
    | (1 << mr._STATUS_BIT_CURRENT_AVG_LIMIT)
    | (1 << mr._STATUS_BIT_MIRROR_TEMP_LIMIT)
)

# Laser check: positions (label, normalized x, normalized y), and pass
# thresholds on OPTICAL angle (spot angle seen from the mirror). Rated
# accuracy is 0.15 deg mechanical = 0.3 deg optical; the rest is allowance
# for tape-measure / distance-D error (~9 mm at 1 m for 0.5 deg at 20 deg).
LASER_POINTS = [
    ("X +0.1", 0.1, 0.0), ("X -0.1", -0.1, 0.0),
    ("X +0.3", 0.3, 0.0), ("X -0.3", -0.3, 0.0),
    ("Y +0.1", 0.0, 0.1), ("Y -0.1", 0.0, -0.1),
    ("Y +0.3", 0.0, 0.3), ("Y -0.3", 0.0, -0.3),
]
LASER_PASS_OPT_DEG = 0.5
LASER_CHECK_OPT_DEG = 1.0
CENTRE_PASS_MM = 3.0
CENTRE_CHECK_MM = 6.0

# trial-on writes these into config.yaml's motion: block.
#   motion_enabled true: calibration jogs and scans need it (fail-closed
#     interlock in the driver).
#   hard_home false: the Calibrate tab anchors the origin at the reference
#     mark (zero_here). hard_home true would make the scan re-zero at the
#     mirror's centre at scan start / periodic rehome, silently shifting
#     the whole grid away from the calibration. With false, resume()
#     reuses the reference-mark origin that calibration persists as
#     motion.origin_abs_deg_x/y (so a program restart is fine).
#   invert_x/y false: the old CONEX's invert_y does not apply to the mirror;
#     direction is reviewed from the test-scan map afterwards.
TRIAL_KEYS = ["controller", "port", "motion_enabled", "hard_home", "invert_x", "invert_y"]


# ---------------------------------------------------------------------------
# Transcript + prompts
# ---------------------------------------------------------------------------

class _Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, s):
        for st in self.streams:
            st.write(s)
        return len(s)

    def flush(self):
        for st in self.streams:
            st.flush()


_TRANSCRIPT = None


def _start_transcript(results_dir: Path, name: str) -> Path:
    global _TRANSCRIPT
    results_dir.mkdir(exist_ok=True)
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = results_dir / f"{stamp}_{name}.txt"
    n = 2
    while path.exists():  # two runs in the same second: never overwrite
        path = results_dir / f"{stamp}_{name}_{n}.txt"
        n += 1
    _TRANSCRIPT = open(path, "w", encoding="utf-8")
    sys.stdout = _Tee(sys.__stdout__, _TRANSCRIPT)
    sys.stderr = _Tee(sys.__stderr__, _TRANSCRIPT)
    logging.basicConfig(
        level=logging.INFO, stream=sys.stdout, force=True,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    print(f"Saving a copy of everything shown here to:\n  {path}\n")
    return path


def _ask(prompt: str) -> str:
    sys.stdout.write(prompt + " ")
    sys.stdout.flush()
    line = sys.stdin.readline()
    if line == "":
        raise SystemExit("\nInput ended -- stopping.")
    answer = line.strip()
    if _TRANSCRIPT is not None:
        _TRANSCRIPT.write(f"{answer}\n")
    return answer


def _ask_yes(prompt: str) -> bool:
    while True:
        a = _ask(prompt + " (y/n):").lower()
        if a in ("y", "yes"):
            return True
        if a in ("n", "no"):
            return False
        print("  Please type y or n.")


def _ask_float(prompt: str, lo: float, hi: float) -> float:
    while True:
        a = _ask(prompt).replace(",", ".")
        try:
            v = float(a)
        except ValueError:
            print("  Please type a number, e.g. 358")
            continue
        if lo <= v <= hi:
            return v
        print(f"  That looks wrong -- expected a number between {lo:g} and {hi:g}.")


def _ask_choice(prompt: str, choices: str) -> str:
    while True:
        a = _ask(prompt).lower()[:1]
        if a and a in choices:
            return a
        print(f"  Please type one letter: {', '.join(choices)}")


def _banner(result: str, msg: str, next_step: str = ""):
    line = "=" * 64
    print(f"\n{line}\n  {result}: {msg}\n{line}")
    if next_step:
        print(f"NEXT: {next_step}")


# ---------------------------------------------------------------------------
# Mirror helpers
# ---------------------------------------------------------------------------

def _connect(port: str, allow_motion: bool) -> MR1530Controller:
    """Standalone connection -- deliberately NOT from config.yaml. deg_per_mm
    is a placeholder; these checks command normalized XY / read degrees."""
    return MR1530Controller({"motion": {
        "controller": "mr1530", "port": port,
        "deg_per_mm_x": 1.0, "deg_per_mm_y": 1.0,
        "motion_enabled": allow_motion, "hard_home": True,
    }})


def _mech_deg(n: float) -> float:
    """Normalized unit-circle coordinate -> mechanical degrees."""
    return math.degrees(math.atan(n * TAN50)) / 2.0


def _move(mc: MR1530Controller, nx: float, ny: float, settle_s: float):
    mc._validate_normalized_target(nx, ny)
    mc._send_xy(nx, ny)
    mc.wait_for_settle(settle_s)


def _park(mc: MR1530Controller, frozen: bool):
    if frozen:
        print("\nNOT moving the mirror back to centre: the software froze it on "
              "purpose after a settle timeout. Leave it alone.")
        return
    try:
        _move(mc, 0.0, 0.0, 0.05)
        print("Mirror parked at centre.")
    except Exception as exc:  # report, never retry
        print(f"WARNING: could not park the mirror at centre: {exc}")


def _comms_ok(port: str) -> tuple[bool, str]:
    """No-motion connection check: START/GETID, Pro-mode fault + position
    reads, and a Simple-mode STATUS afterwards (proves the mode switch back)."""
    mc = _connect(port, allow_motion=False)
    try:
        status1 = mc._get_status()
        fault = mc._read_fault_register()
        pos = mc.get_absolute_position_deg()
        status2 = mc._get_status()
    finally:
        mc.close()
    print(f"  STATUS before : 0x{status1:08X}")
    print(f"  Fault register: 0x{fault:08X}")
    print(f"  Position (deg): X={pos[0]:+.4f}  Y={pos[1]:+.4f}")
    print(f"  STATUS after  : 0x{status2:08X}")
    if fault != 0:
        return False, f"controller reports a fault (register 0x1007 = 0x{fault:08X})"
    if status2 & _FAULT_MASK:
        return False, f"STATUS shows a current/temperature fault (0x{status2:08X})"
    return True, "mirror answers and reports no faults"


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def cmd_check(args) -> str:
    print("Checking the mirror controller answers. The mirror will NOT move.\n")
    ok, why = _comms_ok(args.port)
    _banner("PASS" if ok else "FAIL", why,
            "run:  python tools\\mr1530_onsite.py laser" if ok
            else "stop here and tell Ellen. The transcript file above has the details.")
    return "PASS" if ok else "FAIL"


def cmd_reliability(args) -> str:
    n, r_max, tol = 200, 0.3, 0.2
    print(f"{n} random moves. The mirror twitches around for about 30 seconds.")
    if not _ask_yes("Is the area around the mirror clear and any laser pointed safely?"):
        _banner("FAIL", "not started (area not clear)", "clear the area, then run this again.")
        return "FAIL"
    random.seed(1)
    mc = _connect(args.port, allow_motion=True)
    frozen = False
    errs, settle_ms, read_ms, fails = [], [], [], []
    try:
        mc.home()
        for i in range(n):
            r, a = r_max * math.sqrt(random.random()), random.uniform(0, 2 * math.pi)
            nx, ny = round(r * math.cos(a), 4), round(r * math.sin(a), 4)
            t = time.perf_counter()
            _move(mc, nx, ny, 0.02)
            settle_ms.append((time.perf_counter() - t) * 1000)
            t = time.perf_counter()
            rx, ry = mc.get_absolute_position_deg()
            read_ms.append((time.perf_counter() - t) * 1000)
            e = max(abs(rx - _mech_deg(nx)), abs(ry - _mech_deg(ny)))
            errs.append(e)
            if e > tol:
                fails.append((i, nx, ny, round(e, 4)))
            if (i + 1) % 50 == 0:
                print(f"  {i + 1}/{n} moves done")
        status = mc._get_status()
        fault = mc._read_fault_register()
    except AxisStateUnknown:
        frozen = True
        raise
    finally:
        _park(mc, frozen)
        mc.close()

    def q(xs, p):
        return sorted(xs)[int(p * (len(xs) - 1))]
    print(f"\nmoves: {len(errs)}/{n}   out of tolerance: {len(fails)}")
    print(f"position error (deg): max {max(errs):.4f}  mean {statistics.mean(errs):.4f}")
    print(f"settle ms: median {statistics.median(settle_ms):.1f}  p95 {q(settle_ms, .95):.1f}  max {max(settle_ms):.1f}")
    print(f"read ms  : median {statistics.median(read_ms):.1f}  p95 {q(read_ms, .95):.1f}  max {max(read_ms):.1f}")
    print(f"final STATUS 0x{status:08X}   fault register 0x{fault:08X}")
    for f in fails[:10]:
        print("  out of tolerance:", f)
    ok = not fails and fault == 0 and not (status & _FAULT_MASK)
    _banner("PASS" if ok else "FAIL", f"{len(errs)}/{n} moves, {len(fails)} out of tolerance",
            "" if ok else "stop here and tell Ellen.")
    return "PASS" if ok else "FAIL"


def cmd_laser(args) -> str:
    print("""
LASER SPOT CHECK -- checks the mirror really tilts by the angle the
software reports. Set up first (see the checklist, Step 1):
  - paper on a flat board about 1 m from the mirror, facing it
  - laser fixed next to the board, hitting the mirror almost straight on
  - area clear, beam below eye level
""")
    if not _ask_yes("Is everything set up and the area clear?"):
        _banner("FAIL", "not started (setup not ready)", "finish the setup, then run this again.")
        return "FAIL"

    mc = _connect(args.port, allow_motion=True)
    frozen = False
    rows, dirs = [], {}
    try:
        mc.home()
        print("\nThe mirror is at its CENTRE position.")
        print("1. Mark the laser spot on the paper with a cross and label it CENTRE.")
        print("2. Square up the board (checklist Step 1, 'Squaring the board').")
        _ask("Press Enter when done.")
        d_mm = _ask_float("Distance D from the mirror face to the CENTRE cross, in mm (e.g. 1000):", 200, 5000)

        for label, nx, ny in LASER_POINTS:
            _move(mc, nx, ny, 0.1)
            rx, ry = mc.get_absolute_position_deg()
            reported_mech = abs(rx) if nx else abs(ry)
            expected_opt = 2.0 * reported_mech
            expected_mm = d_mm * math.tan(math.radians(expected_opt))
            print(f"\n--- Position {label} ---")
            print(f"Mark the spot and label it '{label}'. Expected about {expected_mm:.0f} mm from CENTRE.")
            meas = _ask_float("Measured straight-line distance from CENTRE (mm):", 0, 5000)
            dirn = _ask_choice("Which way did the spot move from CENTRE? l=left r=right u=up d=down:", "lrud")
            meas_opt = math.degrees(math.atan(meas / d_mm))
            diff = meas_opt - expected_opt
            if abs(diff) <= LASER_PASS_OPT_DEG:
                verdict = "PASS"
            elif abs(diff) <= LASER_CHECK_OPT_DEG:
                verdict = "CHECK"
            else:
                verdict = "FAIL"
            rows.append((label, expected_mm, meas, diff, verdict, dirn))
            dirs[label] = dirn
            print(f"  -> {verdict}  (angle difference {diff:+.2f} deg)")

        _move(mc, 0.0, 0.0, 0.1)
        print("\n--- Back to CENTRE ---")
        back = _ask_float("How far is the spot now from the CENTRE cross, in mm (0 if on it):", 0, 5000)
    except AxisStateUnknown:
        frozen = True
        raise
    finally:
        _park(mc, frozen)
        mc.close()

    opposite = {"l": "r", "r": "l", "u": "d", "d": "u"}
    horizontal = {"l", "r"}
    problems = []
    for ax in ("X", "Y"):
        p1, p3, m1, m3 = dirs[f"{ax} +0.1"], dirs[f"{ax} +0.3"], dirs[f"{ax} -0.1"], dirs[f"{ax} -0.3"]
        if not (p1 == p3 and m1 == m3 and opposite[p1] == m1):
            problems.append(f"{ax} moves were not all along one line in opposite directions")
    if (dirs["X +0.1"] in horizontal) == (dirs["Y +0.1"] in horizontal):
        problems.append("X and Y moved the spot along the same line (should be at right angles)")

    print(f"\nD = {d_mm:.0f} mm")
    print(f"{'Position':<8} | {'expected mm':>11} | {'measured mm':>11} | {'angle diff':>10} | result | direction")
    for label, exp_mm, meas, diff, verdict, dirn in rows:
        print(f"{label:<8} | {exp_mm:11.0f} | {meas:11.0f} | {diff:+9.2f}° | {verdict:<6} | {dirn}")
    centre_verdict = "PASS" if back <= CENTRE_PASS_MM else ("CHECK" if back <= CENTRE_CHECK_MM else "FAIL")
    print(f"Return to centre: {back:.0f} mm -> {centre_verdict}")
    names = {"l": "left", "r": "right", "u": "up", "d": "down"}
    print(f"+X moves the spot {names[dirs['X +0.1']]}; +Y moves the spot {names[dirs['Y +0.1']]}.")
    for p in problems:
        print("PROBLEM:", p)

    verdicts = [r[4] for r in rows] + [centre_verdict]
    if "FAIL" in verdicts or problems:
        result = "FAIL"
    elif "CHECK" in verdicts:
        result = "CHECK"
    else:
        result = "PASS"
    msg = {"PASS": "mirror angles match the software",
           "CHECK": "close but some points are borderline -- Ellen will review the transcript",
           "FAIL": "angles or directions don't match"}[result]
    _banner(result, msg,
            "stop here and tell Ellen. You can re-run this if you think you mis-measured."
            if result == "FAIL" else "run:  python tools\\mr1530_onsite.py trial-on")
    return result


def _motion_block_span(text: str) -> tuple[int, int]:
    m = re.search(r"^motion:[^\n]*\n", text, re.MULTILINE)
    if not m:
        raise KeyError("config.yaml has no top-level 'motion:' section")
    end = re.search(r"^(?=[^\s#])", text[m.end():], re.MULTILINE)
    return m.end(), (m.end() + end.start()) if end else len(text)


def _set_motion_keys(text: str, values: dict) -> str:
    from scan.calibrate_scan_area import _patch_or_insert_scalar
    start, end = _motion_block_span(text)
    block = text[start:end]
    for key, val in values.items():
        block = _patch_or_insert_scalar(block, key, val, after_key="controller")
    return text[:start] + block + text[end:]


def cmd_trial_on(args) -> str:
    cfg_path = args.config
    text = cfg_path.read_text(encoding="utf-8-sig")
    current = (yaml.safe_load(text) or {}).get("motion", {}) or {}
    if current.get("controller") == "mr1530":
        _banner("PASS", "config.yaml already uses the mirror (trial mode is already on)",
                "continue with Step 2 (Calibrate) in the checklist.")
        return "PASS"

    print("1/4  Checking the mirror answers (no motion)...")
    ok, why = _comms_ok(args.port)
    if not ok:
        _banner("FAIL", f"{why} -- config.yaml NOT changed", "stop here and tell Ellen.")
        return "FAIL"

    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = cfg_path.with_name(f"config.backup_before_mr1530_{stamp}.yaml")
    shutil.copy2(cfg_path, backup)
    args.results_dir.mkdir(exist_ok=True)
    (args.results_dir / "trial_backup_path.txt").write_text(str(backup), encoding="utf-8")
    print(f"2/4  Backed up config.yaml to {backup.name}")

    wanted_text = {
        "controller": "mr1530", "port": f'"{args.port}"', "motion_enabled": "true",
        "hard_home": "false", "invert_x": "false", "invert_y": "false",
    }
    wanted = {"controller": "mr1530", "port": args.port, "motion_enabled": True,
              "hard_home": False, "invert_x": False, "invert_y": False}
    try:
        new_text = _set_motion_keys(text, wanted_text)
        new_motion = (yaml.safe_load(new_text) or {}).get("motion", {})
        bad = {k: new_motion.get(k) for k in TRIAL_KEYS if new_motion.get(k) != wanted[k]}
        if bad:
            raise ValueError(f"settings did not come out as intended: {bad}")
        cfg_path.write_text(new_text, encoding="utf-8")
        print("3/4  config.yaml now points at the mirror (trial mode).")
        print("4/4  Checking the scanner program can open the mirror with the new settings...")
        mc = get_motion_controller(yaml.safe_load(cfg_path.read_text(encoding="utf-8-sig")))
        if not isinstance(mc, MR1530Controller):
            mc.close()
            raise TypeError(f"settings opened {type(mc).__name__}, not the mirror")
        mc.close()
    except Exception as exc:
        shutil.copy2(backup, cfg_path)
        print(f"ERROR: {exc}")
        _banner("FAIL", "could not switch -- config.yaml has been put back exactly as it was",
                "stop here and tell Ellen.")
        return "FAIL"
    _banner("PASS", "trial mode ON -- the scanner program now uses the new mirror",
            "open the scanner program and do Step 2 (Calibrate) in the checklist.")
    return "PASS"


def cmd_trial_off(args) -> str:
    cfg_path = args.config
    marker = args.results_dir / "trial_backup_path.txt"
    backup = Path(marker.read_text(encoding="utf-8").strip()) if marker.exists() else None
    if backup is None or not backup.exists():
        found = sorted(cfg_path.parent.glob("config.backup_before_mr1530_*.yaml"))
        backup = found[-1] if found else None
    if backup is None:
        _banner("FAIL", "no backup of the old config.yaml found -- nothing changed",
                "stop here and tell Ellen.")
        return "FAIL"
    current = (yaml.safe_load(cfg_path.read_text(encoding="utf-8-sig")) or {}).get("motion", {}) or {}
    if current.get("controller") != "mr1530":
        _banner("PASS", "config.yaml is not in trial mode -- nothing to undo")
        return "PASS"
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    keep = cfg_path.with_name(f"config.trial_mr1530_{stamp}.yaml")
    shutil.copy2(cfg_path, keep)
    shutil.copy2(backup, cfg_path)
    restored = (yaml.safe_load(cfg_path.read_text(encoding="utf-8-sig")) or {}).get("motion", {}) or {}
    print(f"Saved the trial settings (including any calibration) as {keep.name}")
    print(f"Restored config.yaml from {backup.name} (controller: {restored.get('controller')})")
    _banner("PASS", "trial mode OFF -- config.yaml is back to how it was before the trial",
            "tell Ellen which step failed. The old mount needs to be physically back in place "
            "and re-calibrated before it can scan again.")
    return "PASS"


COMMANDS = {
    "check": cmd_check,
    "reliability": cmd_reliability,
    "laser": cmd_laser,
    "trial-on": cmd_trial_on,
    "trial-off": cmd_trial_off,
}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="MR-15-30 on-site switchover helper")
    ap.add_argument("command", choices=list(COMMANDS))
    ap.add_argument("--port", default=DEFAULT_PORT, help="MR-E-3 COM port (default COM7)")
    ap.add_argument("--config", type=Path, default=REPO_ROOT / "config.yaml", help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    args.results_dir = args.config.resolve().parent / "switchover_results"

    _start_transcript(args.results_dir, args.command)
    try:
        result = COMMANDS[args.command](args)
    except AxisStateUnknown as exc:
        print(f"\n{exc}")
        _banner("FAIL", "the software froze the mirror after it did not settle",
                "do NOT run anything else. Leave the mirror alone and tell Ellen.")
        result = "FAIL"
    except SystemExit:
        raise
    except Exception:
        traceback.print_exc()
        _banner("FAIL", "unexpected error (details above)", "stop here and tell Ellen.")
        result = "FAIL"
    return 0 if result in ("PASS", "CHECK") else 1


if __name__ == "__main__":
    sys.exit(main())
