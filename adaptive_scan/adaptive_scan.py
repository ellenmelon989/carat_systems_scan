"""
adaptive_scan.py

AdaptiveRasterScanner — the edge-following, open-loop-safe coarse wafer
scan described in docs/adaptive_scan_spec.md. Separate module from
scan_manager.py by design (spec §14): shares motion_controller and the IR/
OES readers, but never trusts commanded position as physical position the
way the precision scan path's generate_grid()/move_to() does.

Implements the 13-step procedure from spec §3 using motion.jog() (a
relative move — see motion_controller.MotionController.jog()) plus one
raw signal poll per step, fed through edge_detector.EdgeDetector.

Lives in adaptive_scan/ (see adaptive_scan/__init__.py) alongside
edge_detector.py, adaptive_scan_params.py, adaptive_scan_signal.py, and
adaptive_scan_logger.py.
"""

from __future__ import annotations

# --- repo-root import bootstrap -------------------------------------------
# Lets this file be run directly (`python adaptive_scan/adaptive_scan.py`),
# as a module (`python -m adaptive_scan.adaptive_scan`), or imported from
# elsewhere in the repo -- all need the repo root on sys.path so sibling
# top-level packages (motion/, readers/, used by this file's own __main__
# smoke test) and this file's own package (adaptive_scan/) resolve the same
# way regardless of invocation. See scan/scan_manager.py for the same
# pattern applied to the other scan mode.
import os as _os
import sys as _sys

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)
# ---------------------------------------------------------------------------

import time
from dataclasses import dataclass, field
from typing import Optional

from adaptive_scan.adaptive_scan_logger import (
    AdaptiveScanRawLogger, RowSummary, compute_row_qc_flags, build_coarse_grid,
)
from adaptive_scan.adaptive_scan_params import (
    validate_signal_name, validate_thresholds, validate_confirm_count,
    validate_reading_interval, validate_y_raster_spacing_mm, validate_max_travel_mm,
    validate_coarse_grid_cells, COARSE_GRID_CELLS_DEFAULT,
)
from adaptive_scan.adaptive_scan_signal import read_raw_signals, select_value
from adaptive_scan.edge_detector import EdgeDetector


class ScanAborted(Exception):
    """Raised internally when stop_event fires mid-row; caught by run()."""


class TravelLimitExceeded(RuntimeError):
    """
    Raised when the runtime max-travel guard (spec §9) trips — a genuine
    safety stop, distinct from the NORMAL step-13 termination (a bounded
    search finding no wafer signal, which is expected and returned as a
    regular result, not an exception). If this fires, something has gone
    further than the operator's configured max_x_travel_mm/max_y_travel_mm
    from the session's starting position — either the wafer/geometry
    doesn't match what the operator expected, or a stall/slip is much
    larger than this method can tolerate. See TravelGuard's docstring for
    why this is checked via get_position()/check_limits() rather than by
    summing commanded deltas.

    Caught by run() (see below) rather than left to propagate: whatever
    rows were already completed before the trip are real, already-logged
    data (each row is committed to the raw CSV as it finishes — see
    AdaptiveScanRawLogger.finish_row()), so a hard trip mid-scan should
    still return them via AdaptiveScanResult rather than discarding
    everything the way an uncaught exception would.
    """


class EdgeSearchFailed(RuntimeError):
    """
    Raised when one of the fixed boundary searches in spec §3 steps 2, 3,
    or 5 (seeking the FIRST Y exit/entry, or the first row's X exit)
    fails to confirm its expected transition within a bounded number of
    iterations. Distinct from both TravelLimitExceeded (a raw safety trip
    that can happen anywhere, including mid-row) and the per-row step-13
    termination (a normal, expected "no more wafer in this direction"
    conclusion once at least one row has already been scanned).

    Steps 2/3/5 are NOT allowed to fail normally the way step 13 is --
    they run before any row/data exists, and the 13-step procedure
    assumes the operator has already jogged onto a valid wafer signal
    (spec §3 step 1) before starting. If one of these three searches
    exhausts its bound, that almost always means the start position,
    on_threshold/off_threshold, or polarity don't match the real signal
    -- a setup problem, not a "scan finished" outcome. Previously these
    three searches had no bound at all and would loop until
    TravelLimitExceeded eventually fired somewhere far from the start
    position, with no indication of which step failed or why; this
    exception exists to surface that failure immediately, at the step
    that actually detected it, with a message that names the problem.
    """


@dataclass
class AdaptiveScanParams:
    """
    The 7 operator-adjustable parameters from spec §4, already validated
    (see from_operator_input() below) — every field here is direct
    operator input for a real scan, per Roy's defaults policy (spec §4):
    no real-use default exists for any of these except coarse_grid_cells.
    """
    signal_name: str
    on_threshold: float
    off_threshold: float
    confirm_count: int
    reading_interval_mode: str    # "time_s" | "motor_pulses"
    reading_interval_value: float
    y_raster_spacing_mm: float
    max_x_travel_mm: float
    max_y_travel_mm: float
    coarse_grid_cells: int = COARSE_GRID_CELLS_DEFAULT

    @staticmethod
    def from_operator_input(config: dict, *, signal_name: str, on_threshold: float,
                             off_threshold: float, confirm_count: int,
                             reading_interval_mode: str, reading_interval_value: float,
                             y_raster_spacing_mm: float, max_x_travel_mm: float,
                             max_y_travel_mm: float,
                             coarse_grid_cells: int = COARSE_GRID_CELLS_DEFAULT) -> "AdaptiveScanParams":
        """
        Validate every raw operator-entered value through
        adaptive_scan_params' validators before constructing the params
        bundle a scan actually runs with — mirrors ScanManager's own
        preflight-validate-before-touching-hardware discipline, just for
        this mode's parameter set instead of the grid/soft_limits one.
        """
        signal_name = validate_signal_name(signal_name, config)
        on_threshold, off_threshold = validate_thresholds(on_threshold, off_threshold)
        confirm_count = validate_confirm_count(confirm_count)
        reading_interval_mode, reading_interval_value = validate_reading_interval(
            reading_interval_mode, reading_interval_value)
        y_raster_spacing_mm = validate_y_raster_spacing_mm(y_raster_spacing_mm)
        max_x_travel_mm = validate_max_travel_mm(max_x_travel_mm)
        max_y_travel_mm = validate_max_travel_mm(max_y_travel_mm)
        coarse_grid_cells = validate_coarse_grid_cells(coarse_grid_cells)
        return AdaptiveScanParams(
            signal_name=signal_name, on_threshold=on_threshold, off_threshold=off_threshold,
            confirm_count=confirm_count, reading_interval_mode=reading_interval_mode,
            reading_interval_value=reading_interval_value, y_raster_spacing_mm=y_raster_spacing_mm,
            max_x_travel_mm=max_x_travel_mm, max_y_travel_mm=max_y_travel_mm,
            coarse_grid_cells=coarse_grid_cells,
        )


@dataclass
class AdaptiveScanResult:
    rows: list        # list[RowSummary]
    readings: list     # list[dict], from AdaptiveScanRawLogger
    row_flags: dict     # {row_number: [flag, ...]}
    coarse_grid: dict    # from build_coarse_grid()
    status: str          # "completed" | "aborted"
    # None for "completed". For "aborted", distinguishes WHY the scan
    # stopped early -- "operator_abort" (stop_event set by the GUI/CLI
    # caller) vs "travel_limit_exceeded" (TravelGuard tripped, spec §9).
    # Both preserve whatever rows/readings/row_flags/coarse_grid were
    # already accumulated; this field just tells the caller which of the
    # two happened, since "the operator clicked Abort" and "the safety
    # guard stopped this because something went further than expected"
    # call for different operator reactions to the SAME status string.
    stop_reason: Optional[str] = None


class TravelGuard:
    """
    Runtime safety bound on total travel from session start (spec §9),
    checked via motion.get_position()/check_limits() — the SAME open-loop,
    step-count-derived position estimate the existing precision scan path
    already trusts for its own soft_limits check. That's a deliberate
    reuse, not an oversight: this guard exists purely as a safety margin
    (don't drive the mount somewhere unexpected), not a map-accuracy claim,
    so it's fine to lean on the same dead-reckoning position the rest of
    the codebase already accepts for that purpose.

    Earlier drafts of this design summed |dx|/|dy| across every jog
    instead. That over-counts ordinary back-and-forth jitter during
    entry/exit confirmation (many small jogs that mostly cancel out would
    still add up to a large "cumulative" total despite barely displacing
    the mount) — net position via get_position() doesn't have that
    false-positive problem, so it's what this class uses instead.
    """

    def __init__(self, motion, start_x_mm: float, start_y_mm: float,
                 max_x_travel_mm: float, max_y_travel_mm: float):
        self.motion = motion
        self._limits = {
            "x_min_mm": start_x_mm - max_x_travel_mm,
            "x_max_mm": start_x_mm + max_x_travel_mm,
            "y_min_mm": start_y_mm - max_y_travel_mm,
            "y_max_mm": start_y_mm + max_y_travel_mm,
        }

    def check(self):
        x, y = self.motion.get_position()
        try:
            self.motion.check_limits(x, y, self._limits)
        except ValueError as e:
            raise TravelLimitExceeded(str(e)) from e


class AdaptiveRasterScanner:
    """
    Parameters
    ----------
    config : dict
        Same shape as the precision scan's config.yaml — used here for
        oes.features (signal dispatch), oes.feature_window_nm, and
        motion.steps_per_mm_x/move_velocity (to convert reading_interval
        into an equivalent per-step jog distance — see _compute_step_mm).
    params : AdaptiveScanParams
        Already-validated operator parameters (spec §4) — build via
        AdaptiveScanParams.from_operator_input().
    motion : MotionController
        Must already be homed/zeroed with the operator positioned on a
        valid wafer signal (spec §3 step 1) BEFORE calling run() — this
        class does not home or prompt for that; it assumes whatever
        get_position() reads right now is where the session starts.
    ir_reader, spectrometer : as constructed by
        readers.ir_reader_base.get_ir_reader() / spectrometer_reader_base.
        get_spectrometer_reader(), or their mocks. spectrometer may be
        None if signal_name is an IR field and no OES features are needed.
    output_path : str
        Where AdaptiveScanRawLogger writes the raw per-reading CSV.
    """

    # Fixed search directions for the boundary-seeking phases (spec §3
    # steps 2-3, 5-6) — arbitrary but must be consistent and documented,
    # since nothing in the operator's 7 parameters (spec §4) specifies
    # which physical direction is "the" search direction. +Y first, then
    # -Y is "inward"; +X first for the first row. Flip these constants if
    # a given install's geometry makes the opposite convention more
    # natural (e.g. the wafer is reliably above/left of the start point).
    _INITIAL_Y_SEARCH_SIGN = 1
    _INITIAL_X_SEARCH_SIGN = 1

    def __init__(self, config: dict, params: AdaptiveScanParams, motion, ir_reader,
                 spectrometer=None, output_path: str = "./adaptive_scan_data/raw_readings.csv"):
        self.config = config
        self.params = params
        self.motion = motion
        self.ir_reader = ir_reader
        self.spectrometer = spectrometer
        self.oes_features_cfg = config.get("oes", {}).get("features", {})
        self.feature_window_nm = config.get("oes", {}).get("feature_window_nm", 1.0)

        self._step_mm = self._compute_step_mm(config, params)
        # Step-13 bound (spec §3 step 13, §9): how many consecutive ignore-
        # phase iterations (i.e. how far in X) to search for a wafer signal
        # before concluding the opposite Y edge has been passed. Tied to
        # the same max_x_travel_mm the operator already set as a safety
        # bound, so a normal "no wafer found" conclusion is reached at
        # essentially the same point TravelGuard would otherwise raise a
        # hard fault at — but as a clean, expected termination instead.
        # Also reused, unchanged, as the bound for step 5's X-direction
        # search (run(), below) -- same axis, same safety budget.
        self._max_search_iterations = max(1, int(params.max_x_travel_mm / self._step_mm))
        # Same idea for the Y-direction searches in steps 2 and 3 (run(),
        # below), which have no analog to step 13's "normal termination"
        # -- reaching this bound there means EdgeSearchFailed, not a
        # graceful conclusion (see that exception's docstring for why).
        self._max_y_search_iterations = max(1, int(params.max_y_travel_mm / self._step_mm))

        self.logger = AdaptiveScanRawLogger(output_path, oes_feature_names=list(self.oes_features_cfg))
        self.travel_guard: Optional[TravelGuard] = None
        self._events: list = []

    @staticmethod
    def _compute_step_mm(config: dict, params: AdaptiveScanParams) -> float:
        """
        Convert the operator's reading_interval (spec §4 parameter 4) into
        an equivalent per-reading jog distance in mm.

        motor_pulses mode: direct — pulses / steps_per_mm_x. Note this is
        the one place a placeholder/uncalibrated steps_per_mm_x would
        distort real spacing, and per spec §7, a fixed pulse count maps to
        a DIFFERENT real mm distance depending on travel direction
        (backlash) — a known, accepted limitation of this mode, not
        something this conversion tries to correct.

        time_s mode: distance = configured move_velocity (steps/s) /
        steps_per_mm_x, times the requested seconds — i.e. "how far the
        stage would travel in this many seconds at the configured scan
        velocity." This still bottoms out in the same steps_per_mm_x
        conversion, so it isn't immune to calibration error either, but it
        lets the operator reason in seconds rather than raw pulses, which
        is the UI-friendliness point made in spec §7 (not a true
        continuous-motion-while-polling implementation — motion.jog() is
        blocking; see the module docstring below for that caveat spelled
        out for a future revision).
        """
        motion_cfg = config.get("motion", {})
        steps_per_mm_x = float(motion_cfg.get("steps_per_mm_x", 500))
        if params.reading_interval_mode == "motor_pulses":
            return params.reading_interval_value / steps_per_mm_x
        move_velocity = float(motion_cfg.get("move_velocity", 2000))  # steps/s
        velocity_mm_s = move_velocity / steps_per_mm_x
        return params.reading_interval_value * velocity_mm_s

    # ------------------------------------------------------------------
    # Low-level building blocks
    # ------------------------------------------------------------------

    def _guarded_jog(self, dx_mm: float = 0.0, dy_mm: float = 0.0):
        """
        Relative jog through motion.jog(), bracketed by the travel guard
        (spec §9) both before and after — before, so we never issue a jog
        while already outside the safety envelope; after, so a single jog
        that overshoots past it is caught immediately rather than only on
        the NEXT iteration's pre-check.
        """
        self.travel_guard.check()
        self.motion.jog(dx_mm=dx_mm, dy_mm=dy_mm)
        self.travel_guard.check()

    def _poll(self):
        raw = read_raw_signals(self.ir_reader, self.spectrometer,
                                self.oes_features_cfg, self.feature_window_nm)
        value = select_value(raw, self.params.signal_name)
        return raw, value

    def _log_event(self, message: str):
        line = f"{time.strftime('%Y-%m-%dT%H:%M:%S')}\t{message}"
        print(line)
        self._events.append(line)

    def _check_stop(self, stop_event):
        if stop_event is not None and stop_event.is_set():
            raise ScanAborted()

    def _seek_transition(self, dx_mm: float, dy_mm: float, detector: EdgeDetector,
                          want: str, stop_event=None, max_iterations: Optional[int] = None):
        """
        Repeatedly guarded-jog by (dx_mm, dy_mm) and poll until detector
        reports `want` ("entered_wafer" or "exited_wafer"). Used for spec
        §3 steps 2, 3, and 5 — plain boundary search, no data retained
        (readings before a row's own confirmed entry are never logged;
        see spec's "ignore readings until wafer signal detected").

        max_iterations: if given and reached without `want` occurring,
        returns False instead of looping forever. run() passes a bound
        (_max_y_search_iterations for steps 2/3, _max_search_iterations
        for step 5) and raises EdgeSearchFailed when this returns False --
        unlike the per-row ignore-phase in _scan_row (which treats its own
        bounded search hitting the limit as the normal step-13
        termination), steps 2/3/5 have no such "normal" failure mode: they
        run before any row exists, so exhausting the bound here means
        setup (start position/thresholds/polarity), not "scan complete".
        Returns True once `want` is confirmed.
        """
        iterations = 0
        while True:
            self._check_stop(stop_event)
            self._guarded_jog(dx_mm=dx_mm, dy_mm=dy_mm)
            _, value = self._poll()
            event = detector.update(value)
            iterations += 1
            if event == want:
                return True
            if max_iterations is not None and iterations >= max_iterations:
                return False

    # ------------------------------------------------------------------
    # Row scanning (spec §3 steps 6-9, plus the step-13 bounded search)
    # ------------------------------------------------------------------

    def _scan_row(self, row_number: int, direction_sign: int, stop_event=None):
        """
        Scan one raster row in the X direction given by direction_sign
        (+1 or -1). Combines:
          - the bounded "ignore until entry confirmed" search (step 7,
            §3) — also this method's step-13 check: if entry is never
            confirmed within self._max_search_iterations, this row (and
            the whole scan) is over, normally.
          - the "record while valid, end on confirmed exit" collection
            phase (steps 8-9).

        Returns (row_summary_or_None, finished_readings, scan_complete).
        scan_complete=True means step 13 fired — caller should stop the
        whole serpentine loop, not just skip this row.
        """
        scan_direction = "+x" if direction_sign > 0 else "-x"
        detector = EdgeDetector(self.params.on_threshold, self.params.off_threshold,
                                 self.params.confirm_count, initial_state="off")
        self.logger.start_row()

        total_commanded_mm = 0.0
        start_time = time.time()

        # --- Step 7 (bounded): ignore readings until entry confirmed ---
        ignore_iterations = 0
        entered = False
        while not entered:
            self._check_stop(stop_event)
            self._guarded_jog(dx_mm=self._step_mm * direction_sign)
            total_commanded_mm += self._step_mm
            ignore_iterations += 1
            _, value = self._poll()
            event = detector.update(value)
            if event == "entered_wafer":
                entered = True
                break
            if ignore_iterations >= self._max_search_iterations:
                # Step 13: a complete bounded X sweep found no wafer
                # signal at all -- normal, expected termination, not a
                # fault. Nothing was buffered (only ignore-phase reads
                # happened), so just report scan_complete.
                return None, [], True

        # --- Steps 8-9: record while valid, stop on confirmed exit ---
        dip_starts = 0
        prev_consecutive_off = 0
        while True:
            self._check_stop(stop_event)
            self._guarded_jog(dx_mm=self._step_mm * direction_sign)
            total_commanded_mm += self._step_mm
            raw, value = self._poll()
            event = detector.update(value)

            # Internal-signal-loss proxy (spec §11 flag #1): count every
            # time a FRESH off-streak starts while this row is still in
            # progress. The final off-streak that actually ends the row
            # also starts at consecutive_off==1, so dip_starts always
            # counts that one too -- more than 1 means there was at least
            # one earlier streak that recovered before confirming, i.e. a
            # transient loss the row survived.
            if detector.consecutive_off == 1 and prev_consecutive_off == 0:
                dip_starts += 1
            prev_consecutive_off = detector.consecutive_off

            if event == "exited_wafer":
                break

            self.logger.add_reading(
                row_number=row_number, scan_direction=scan_direction, raw=raw,
                selected_signal_name=self.params.signal_name, selected_value=value,
                motor_dx_mm=self._step_mm * direction_sign, motor_dy_mm=0.0,
            )

        finished = self.logger.finish_row()
        end_time = time.time()
        row_summary = RowSummary(
            row_number=row_number, scan_direction=scan_direction,
            n_readings=len(finished), had_internal_loss=(dip_starts > 1),
            total_commanded_dx_mm=total_commanded_mm, start_time=start_time, end_time=end_time,
        )
        return row_summary, finished, False

    # ------------------------------------------------------------------
    # Full serpentine procedure (spec §3, all 13 steps)
    # ------------------------------------------------------------------

    def run(self, on_row=None, stop_event=None) -> AdaptiveScanResult:
        """
        on_row: optional callback(row_summary, finished_readings) invoked
        after each completed row — mirrors ScanManager.run()'s on_point,
        for a GUI to show live progress (spec §14 step 6).

        stop_event: optional threading.Event, checked at the top of every
        jog iteration (finer-grained than ScanManager's between-points
        check, since one row here can take many jogs).

        Returns an AdaptiveScanResult with every row's summary, every
        retained raw reading, per-row QC flags (spec §11), and the
        aggregated coarse grid (spec §10, §12). Does not itself decide
        what to do with the result (write to disk, plot, etc.) — the CLI
        entry point below and, eventually, the GUI (spec §14 step 6) do
        that, the same separation ScanManager keeps from DataLogger.
        """
        start_x, start_y = self.motion.get_position()
        self.travel_guard = TravelGuard(self.motion, start_x, start_y,
                                         self.params.max_x_travel_mm, self.params.max_y_travel_mm)
        self._log_event(
            f"Adaptive scan started at ({start_x:.3f}, {start_y:.3f}) mm — "
            f"signal={self.params.signal_name}, step={self._step_mm:.3f}mm, "
            f"confirm_count={self.params.confirm_count}, "
            f"y_raster_spacing_mm={self.params.y_raster_spacing_mm}"
        )

        all_rows: list = []
        all_readings: list = []

        try:
            # --- Step 2: seek Y exit ---
            boundary_detector = EdgeDetector(self.params.on_threshold, self.params.off_threshold,
                                              self.params.confirm_count, initial_state="on")
            if not self._seek_transition(dx_mm=0.0, dy_mm=self._step_mm * self._INITIAL_Y_SEARCH_SIGN,
                                          detector=boundary_detector, want="exited_wafer", stop_event=stop_event,
                                          max_iterations=self._max_y_search_iterations):
                raise EdgeSearchFailed(
                    f"Step 2: no wafer-edge Y exit found within max_y_travel_mm "
                    f"({self.params.max_y_travel_mm} mm) of the start position "
                    f"({start_x:.3f}, {start_y:.3f}). Check that the operator start "
                    "position is actually on the wafer signal, and that "
                    "on_threshold/off_threshold match the real signal."
                )
            self._log_event("Step 2: Y exit confirmed")

            # --- Step 3: reverse Y, seek entry ---
            boundary_detector.reset(initial_state="off")
            if not self._seek_transition(dx_mm=0.0, dy_mm=-self._step_mm * self._INITIAL_Y_SEARCH_SIGN,
                                          detector=boundary_detector, want="entered_wafer", stop_event=stop_event,
                                          max_iterations=self._max_y_search_iterations):
                raise EdgeSearchFailed(
                    f"Step 3: no wafer-edge Y entry found within max_y_travel_mm "
                    f"({self.params.max_y_travel_mm} mm) after reversing from the "
                    "step-2 Y exit. The Y exit that was confirmed may have been a "
                    "noise transient, or the wafer is narrower in Y than expected."
                )
            self._log_event("Step 3: Y entry confirmed (first Y edge found)")

            # --- Step 4: move inward by one Y raster increment ---
            self._guarded_jog(dy_mm=-self.params.y_raster_spacing_mm * self._INITIAL_Y_SEARCH_SIGN)
            self._log_event("Step 4: moved inward one Y raster increment — first row established")

            # --- Step 5: seek X exit (to find the far side, then reverse) ---
            boundary_detector.reset(initial_state="on")
            if not self._seek_transition(dx_mm=self._step_mm * self._INITIAL_X_SEARCH_SIGN, dy_mm=0.0,
                                          detector=boundary_detector, want="exited_wafer", stop_event=stop_event,
                                          max_iterations=self._max_search_iterations):
                raise EdgeSearchFailed(
                    f"Step 5: no wafer-edge X exit found within max_x_travel_mm "
                    f"({self.params.max_x_travel_mm} mm) after moving inward one Y "
                    "raster increment. The Y raster increment may have moved off the "
                    "wafer already, or the wafer is narrower in X than expected."
                )
            self._log_event("Step 5: X exit confirmed")

            # --- Step 6: reverse X — begin the first row ---
            direction_sign = -self._INITIAL_X_SEARCH_SIGN
            row_number = 0

            while True:
                row_summary, finished, scan_complete = self._scan_row(
                    row_number, direction_sign, stop_event=stop_event)

                if scan_complete:
                    self._log_event(
                        f"Step 13: complete X sweep (row {row_number}) found no wafer "
                        "signal within max_x_travel_mm — scan complete (opposite Y edge passed)."
                    )
                    break

                all_rows.append(row_summary)
                all_readings.extend(finished)
                self._log_event(
                    f"Row {row_number} ({row_summary.scan_direction}): "
                    f"{row_summary.n_readings} readings, "
                    f"internal_loss={row_summary.had_internal_loss}"
                )
                if on_row is not None:
                    on_row(row_summary, finished)

                # --- Steps 10-11: Y raster increment, reverse X ---
                self._guarded_jog(dy_mm=-self.params.y_raster_spacing_mm * self._INITIAL_Y_SEARCH_SIGN)
                direction_sign *= -1
                row_number += 1

            status = "completed"
            stop_reason = None

        except ScanAborted:
            self._log_event(f"Adaptive scan ABORTED by operator after {len(all_rows)} row(s)")
            status = "aborted"
            stop_reason = "operator_abort"

        except TravelLimitExceeded as e:
            # Previously uncaught here: it propagated straight out of run()
            # to gui/adaptive_scan_worker.py's generic `except Exception`,
            # which reported it as a plain error and discarded this
            # in-memory result entirely -- even though every row in
            # all_rows/all_readings was already safely committed to the
            # raw CSV (AdaptiveScanRawLogger.finish_row() writes per row,
            # not at the end). Catching it here means a hard safety trip
            # mid-scan still returns those completed rows/flags/coarse
            # grid, same as an operator abort does, instead of losing
            # everything already collected on top of losing the rest.
            self._log_event(
                f"Adaptive scan STOPPED — travel-limit safety guard tripped after "
                f"{len(all_rows)} row(s): {e}"
            )
            status = "aborted"
            stop_reason = "travel_limit_exceeded"

        row_flags = compute_row_qc_flags(all_rows)
        coarse_grid = (build_coarse_grid(all_readings, self.params.coarse_grid_cells)
                       if all_readings else {"n_side": 0, "mean": None, "count": None, "stddev": None})

        return AdaptiveScanResult(rows=all_rows, readings=all_readings, row_flags=row_flags,
                                   coarse_grid=coarse_grid, status=status, stop_reason=stop_reason)


if __name__ == "__main__":
    # Smoke test: a simulated circular "wafer" driving a mock IR reader by
    # actual (x, y) position, run through the FULL serpentine procedure end
    # to end on MockMotionController — the strongest available check short
    # of real hardware, since it exercises multiple rows, the serpentine
    # direction reversal, internal-loss/stall QC flags under injected
    # anomalies, and the step-13 termination, all together.
    import os
    import random
    import tempfile

    from motion.motion_controller import MockMotionController
    from readers.ir_reader_base import IRReader, IRReading

    WAFER_RADIUS_MM = 20.0
    WAFER_CENTER = (0.0, 0.0)

    class SimulatedWaferIRReader(IRReader):
        """
        Test-only: dilution reads high (on-wafer) within WAFER_RADIUS_MM
        of WAFER_CENTER (as seen by the mock motion controller's own
        get_position()), low outside. A few positions are marked "flaky"
        to inject a transient internal dip (tests the internal_signal_loss
        QC flag) without faking an entire row.
        """

        def __init__(self, motion):
            self.motion = motion
            self._flaky_calls = 0

        def read(self) -> IRReading:
            x, y = self.motion.get_position()
            r = ((x - WAFER_CENTER[0]) ** 2 + (y - WAFER_CENTER[1]) ** 2) ** 0.5
            on_wafer = r <= WAFER_RADIUS_MM
            dilution = 1.0 if on_wafer else 0.3
            dilution += random.uniform(-0.02, 0.02)

            # Inject exactly ONE transient single-reading dip while
            # on-wafer, partway through the scan, to exercise internal-loss
            # flagging on exactly one row without touching the rest — a
            # lone bad reading (not confirm_count in a row), so it should
            # register as a recovered dip rather than a real exit.
            if on_wafer:
                self._flaky_calls += 1
                if self._flaky_calls == 500:
                    dilution = 0.3

            t = time.time()
            return IRReading(value_c=900.0, emissivity=0.85, dilution=dilution,
                              pac_timestamp=t, read_time=t, stale=False)

    motion = MockMotionController()
    motion.home()
    ir_reader = SimulatedWaferIRReader(motion)

    config = {
        "motion": {"steps_per_mm_x": 500, "move_velocity": 2000},
        "oes": {"features": {}, "feature_window_nm": 1.0},
    }

    params = AdaptiveScanParams.from_operator_input(
        config,
        signal_name="ir_dilution",
        on_threshold=0.9, off_threshold=0.6,
        confirm_count=3,
        reading_interval_mode="time_s", reading_interval_value=0.05,
        y_raster_spacing_mm=3.0,
        max_x_travel_mm=60.0, max_y_travel_mm=60.0,
        coarse_grid_cells=100,
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        output_path = os.path.join(tmpdir, "raw_readings.csv")
        scanner = AdaptiveRasterScanner(config, params, motion, ir_reader, spectrometer=None,
                                         output_path=output_path)
        result = scanner.run()

        assert result.status == "completed", result.status
        assert result.stop_reason is None, result.stop_reason
        assert len(result.rows) >= 3, f"expected several rows across a 40mm-diameter wafer, got {len(result.rows)}"
        print(f"\nRows scanned: {len(result.rows)}")
        for r in result.rows:
            print(f"  row {r.row_number} ({r.scan_direction}): {r.n_readings} readings, "
                  f"flags={result.row_flags.get(r.row_number)}")

        assert any(result.row_flags.get(r.row_number) for r in result.rows), \
            "expected at least one row to be flagged (internal-loss injection)"
        assert any("internal_signal_loss" in flags for flags in result.row_flags.values()), \
            result.row_flags

        assert len(result.readings) > 50, len(result.readings)
        assert os.path.exists(output_path)
        with open(output_path) as f:
            n_lines = sum(1 for _ in f)
        assert n_lines == 1 + len(result.readings), (n_lines, len(result.readings))
        print(f"Raw CSV rows: {n_lines - 1} (matches {len(result.readings)} logged readings)")

        grid = result.coarse_grid
        print(f"Coarse grid: {grid['n_side']}x{grid['n_side']}, "
              f"{int(grid['count'].sum())} readings binned, "
              f"mean dilution range {grid['mean'][~__import__('numpy').isnan(grid['mean'])].min():.2f}"
              f"-{grid['mean'][~__import__('numpy').isnan(grid['mean'])].max():.2f}")
        assert grid["n_side"] == 10  # 100 cells -> 10x10

    print("\nadaptive_scan smoke test OK (full serpentine procedure, mock hardware)")

    # --- Regression test: EdgeSearchFailed (steps 2/3/5 now bounded) ---
    # A reader that never reports an on-wafer signal at all -- the
    # simplest way to exercise "step 2 never confirms its transition"
    # without needing to reposition the simulated wafer relative to the
    # mock motion controller's start point. Previously (no bound on
    # _seek_transition for steps 2/3/5) this would have looped forever
    # jogging in Y until TravelGuard itself eventually raised a generic
    # TravelLimitExceeded somewhere well past the operator's configured
    # travel budget -- now it should fail fast, at step 2, with a message
    # naming the problem.
    motion2 = MockMotionController()
    motion2.home()

    class NeverOnWaferIRReader(IRReader):
        def read(self) -> IRReading:
            t = time.time()
            return IRReading(value_c=300.0, emissivity=0.85, dilution=0.3,
                              pac_timestamp=t, read_time=t, stale=False)

    params_tiny_budget = AdaptiveScanParams.from_operator_input(
        config,
        signal_name="ir_dilution",
        on_threshold=0.9, off_threshold=0.6,
        confirm_count=3,
        reading_interval_mode="time_s", reading_interval_value=0.05,
        y_raster_spacing_mm=3.0,
        max_x_travel_mm=60.0, max_y_travel_mm=2.0,  # tiny -- forces a fast bound
        coarse_grid_cells=100,
    )
    with tempfile.TemporaryDirectory() as tmpdir2:
        scanner2 = AdaptiveRasterScanner(
            config, params_tiny_budget, motion2, NeverOnWaferIRReader(), spectrometer=None,
            output_path=os.path.join(tmpdir2, "raw_readings.csv"),
        )
        try:
            scanner2.run()
            raise AssertionError("expected EdgeSearchFailed, scan completed instead")
        except EdgeSearchFailed as e:
            # A reader that's always "off" trivially confirms step 2's
            # exit immediately (it starts assuming "on", so the first few
            # low readings confirm an exit) but can never confirm step 3's
            # re-entry -- so this always fails at step 3, not step 2. Both
            # are the same bounded-search code path; asserting on "Step "
            # (rather than pinning the exact step number) keeps this test
            # honest about which step actually failed instead of assuming.
            assert "Step 2" in str(e) or "Step 3" in str(e), str(e)
            print(f"\nEdgeSearchFailed regression test OK — raised as expected: {e}")

    # --- Regression test: TravelLimitExceeded caught, partial results kept ---
    # TravelGuard bounds NET position relative to wherever run() finds the
    # session starting (spec §9) -- and steps 2/3 themselves must travel
    # roughly the wafer radius in Y just to find the FIRST edge. Starting
    # from dead center (like the tests above) would burn almost the whole
    # max_y_travel_mm budget on that initial search alone, leaving no
    # headroom to demonstrate a trip partway through actual row-scanning.
    # So this test pre-positions the mock stage near the wafer's edge
    # (y=15mm of a 20mm-radius wafer) before run() ever samples its start
    # position, leaving most of a deliberately small max_y_travel_mm free
    # for the row loop's own Y raster increments to exhaust.
    motion3 = MockMotionController()
    motion3.home()
    motion3.jog(dy_mm=15.0)
    ir_reader3 = SimulatedWaferIRReader(motion3)
    params_travel_trip = AdaptiveScanParams.from_operator_input(
        config,
        signal_name="ir_dilution",
        on_threshold=0.9, off_threshold=0.6,
        confirm_count=3,
        reading_interval_mode="time_s", reading_interval_value=0.05,
        y_raster_spacing_mm=3.0,
        max_x_travel_mm=60.0, max_y_travel_mm=10.0,  # trips a few rows into the raster
        coarse_grid_cells=100,
    )
    with tempfile.TemporaryDirectory() as tmpdir3:
        scanner3 = AdaptiveRasterScanner(
            config, params_travel_trip, motion3, ir_reader3, spectrometer=None,
            output_path=os.path.join(tmpdir3, "raw_readings.csv"),
        )
        result3 = scanner3.run()
        assert result3.status == "aborted", result3.status
        assert result3.stop_reason == "travel_limit_exceeded", result3.stop_reason
        assert len(result3.rows) >= 1, "expected at least one completed row before the trip"
        assert len(result3.readings) > 0
        print(f"\nTravelLimitExceeded regression test OK — status={result3.status}, "
              f"stop_reason={result3.stop_reason}, {len(result3.rows)} row(s) preserved")

    print("\nAll adaptive_scan regression tests OK (bounded steps 2/3/5 + caught TravelLimitExceeded)")
