# Adaptive (edge-following) raster scan — design spec

Status: Core logic implemented and smoke-tested against mock hardware
(2026-07-22) — see adaptive_scan/edge_detector.py, adaptive_scan/adaptive_scan_params.py,
adaptive_scan/adaptive_scan_signal.py, adaptive_scan/adaptive_scan_logger.py,
adaptive_scan/adaptive_scan.py. GUI
wiring (§14 step 6) and real on-site hardware validation (§14 step 7) are
still outstanding. Two implementation decisions below refine what's
written in §9 and §10/§12 — flagged inline where they diverge from the
original text so this stays the accurate reference.

## 1. Motivation / why this is a separate mode

The existing scan path (`scan/scan_manager.py`, `real_newport_motion.py`) commands
absolute positions computed from a step-count origin (`move_to(x_mm, y_mm)`),
and everything downstream — `generate_grid()`'s precomputed point list,
`OESStore`'s fixed `(nx, ny, npass)` HDF5 array, the pre-flight limits check —
assumes that mapping from commanded position to physical position is
trustworthy. It isn't, on the current open-loop Picomotor hardware: no
encoder, no limit switches, and per `real_newport_motion.py`'s own docs,
per-step displacement is direction/load/speed-dependent.

This spec describes a second, independent scan mode that never trusts
cumulative step count as physical position. Instead, each raster row detects
the wafer's own edges from a live sensor signal (pyrometer temp/emissivity/
dilution, or a selected OES line), and assigns each valid reading's position
by its *sequence within the row*, not by commanded mm. This trades precision
for robustness: it produces an approximate (~100-cell) map that degrades
gracefully under motor slip, rather than a precise map that's silently wrong
when slip occurs.

**Goals**
- Produce a coarse (~100-cell) wafer map using only the existing pyrometer/
  OES signals and open-loop motion, with no dependency on `steps_per_mm`
  accuracy or absolute position tracking.
- Make every edge decision from consecutive sensor readings, never from
  motor step counts.
- Keep per-row assignment independent — no row assumes the same point count
  or step count as any other row.

**Non-goals**
- This does not replace the existing precision `ScanManager` path (fixed
  grid, multi-pass drift tracking, HDF5 spatial array) — that stays as-is
  for closed-loop or already-calibrated-geometry use cases.
- Not intended to localize a single small feature precisely; it's a coarse
  map, ~100 cells across the wafer, matching the ~1-6mm field of view.
- Does not attempt to correct or compensate for backlash/hysteresis — see
  §8 for why the position-assignment method makes this unnecessary rather
  than something to fix.

## 2. Terminology

- **Reading** — one raw, single-acquisition poll of the selected signal.
  There is only one kind of read in this mode, used for two purposes at
  once: (1) compared against the on/off-wafer thresholds to feed the
  entry/exit debounce, and (2) — if it falls between confirmed boundaries —
  saved as-is to the raw data table (§10). No separate, longer-averaged
  "recording" acquisition and no dwell-time averaging at read time at all.
  Averaging happens exactly once, downstream, when raw readings are binned
  into coarse-grid cells (§10) — that's also where count and stddev per
  cell come from. Averaging before that point would throw away the
  scatter information the per-cell stddev is meant to capture.
- **Field of view (FOV) blur** — the instrument integrates over a ~1-6mm
  spot, so the wafer edge does not appear as a clean on/off step. As the
  spot straddles the physical boundary, the signal ramps through
  intermediate values over roughly one spot-width of travel. Any reading
  taken while the spot is in that zone is ambiguous by construction, not a
  sensor fault.
- **Confirm count (N)** — number of consecutive readings above/below
  threshold required before an entry or exit is accepted. Exists to reject
  single-sample noise; see §6 for its relationship to FOV blur width.
- **Row** — one raster line, scanned in one X direction, terminated by
  confirmed loss of wafer signal on both ends.

## 3. Scan procedure

1. Operator positions the instrument anywhere producing a valid wafer
   signal (manual, visual — same trust model as the existing
   `scan/calibrate_scan_area.py` edge jogs).
2. Jog Y in one direction until the wafer signal is lost (confirmed by N
   consecutive detection reads below threshold).
3. Reverse Y until the wafer signal is detected again (confirmed by N
   consecutive reads above threshold).
4. Jog inward by one Y raster increment. This establishes the first row
   near one known Y edge.
5. Jog X in one direction until the wafer signal is lost.
6. Reverse X — this begins the first row.
7. Ignore readings until the wafer signal is confirmed (first X boundary
   for this row).
8. Record readings while the signal remains valid.
9. When the signal is lost for N consecutive readings, that's the second
   X boundary — end the row.
10. While outside the wafer, jog Y by one raster increment toward the
    opposite Y edge.
11. Reverse X, scan the next row in the opposite direction.
12. Repeat (serpentine).
13. Stop when a complete X sweep, within the configured max travel, finds
    no valid wafer signal at all — this means the scan has passed the
    opposite Y edge. **This termination sweep needs its own bounded
    max-sweep distance, applied together with the runtime max-travel
    safety limit (§9) — not just "sweep until told to stop."** A hardware
    fault that looks like "no signal" (e.g. a stuck reader, a disconnected
    sensor) must not be indistinguishable from "we're past the wafer";
    bounding this sweep is what keeps that failure mode safe rather than
    an unbounded jog.

Every row is processed independently — no assumption that different rows
need the same step count or produce the same reading count.

## 4. Operator-adjustable parameters

| # | Parameter | Notes |
|---|---|---|
| 1 | Wafer detection signal | temp / emissivity / dilution (pyrometer) or a selected OES line. Dilution is the suggested starting choice per Roy, to be confirmed experimentally. Blocked until `ir.pac.dilution_tag_name` is confirmed on the real PAC (see `tools/list_pac_strategy_vars.py` — still unset in `config.yaml`). |
| 2 | On-wafer / off-wafer thresholds | Two values (or one threshold + hysteresis band) against the selected signal. |
| 3 | Confirm count (N) | Consecutive readings required to accept an entry or exit. See §6 for how this interacts with FOV blur width — cannot be chosen independently of reading interval and FOV size. |
| 4 | Reading interval | Either an acquisition-time cadence or a commanded-motor-pulse cadence — operator chooses which at run time (no baked default, per policy above). See §7 for why acquisition-time is the safer *option to recommend in the UI* given direction-dependent backlash. |
| 5 | Y raster spacing | mm between rows. |
| 6 | Max X/Y travel | Runtime safety bound — see §9; different mechanism from the existing static pre-flight check. |
| 7 | Coarse grid cell count | Number of cells in the post-scan display/aggregation grid. Operator-adjustable; **default 100** per Roy. See §10. |

**Defaults policy (per Roy, 2026-07-22): none of the above are baked-in
defaults for real use — every parameter is direct operator input at run
time, for every real trial.** Hardcoded defaults are only appropriate
inside internal software testing (unit tests, a mock-hardware smoke test
analogous to `scan/scan_manager.py --smoke-test`), never as an assumed value in
an actual on-site scan. Any default value that appears in code for this
mode should be understood as dev/test scaffolding, not a recommendation —
this differs from the existing `scan/scan_params.py` pattern (e.g.
`DWELL_TIME_DEFAULT_S`), where a validated default is treated as a real,
usable operator starting point. This mode should validate operator input
against sane bounds (same style as `scan_params.validate_*`) without
supplying a real-use default to fall back on.

## 5. Signal selection & reads

`readers/ir_reader_base.py` already exposes `value_c` (temperature),
`emissivity`, and `dilution` per `IRReading`; `readers/spectrometer_reader_base.py`
exposes per-wavelength intensity. Signal selection for this mode is a thin
dispatch: pick one scalar stream (IR field, or an OES feature per
`scan_manager.extract_features`'s window-integration pattern) and feed it to
detection.

Every reading is a single raw, unaveraged poll (`ir_reader.read()`, or a
single spectrometer `.read()`) — fast enough to run near-continuously while
jogging, and used directly both for the threshold comparison and (if
between confirmed boundaries) as the value saved to the raw table. This
mode does **not** use `read_averaged()` or `scan.dwell_time_s` (2-22s) at
all — that averaging exists for the precision scan path's fixed, trusted
points, and at ~1mm spacing across a ~100mm row it would cost 1.5-5+ hours
for a full wafer for no benefit here, since the averaging this mode needs
happens anyway, once, at the coarse-grid aggregation step (§10): each cell's
mean comes from however many raw readings land in it, and the per-cell
stddev is the noise/uncertainty measure that a per-reading average would
otherwise have thrown away. No open question remains here — resolved by
Roy 2026-07-22.

## 6. Confirm count (N) vs. FOV blur

The FOV (1-6mm) means a true wafer edge produces a ramp, not a step, across
roughly one spot-width of travel. At a 1mm reading interval and (say) a
3mm blur zone, ~3 consecutive readings will sit in that ambiguous middle
ground for every real edge crossing.

- N chosen larger than the blur-zone reading count (e.g. N=5 against a
  3-reading blur zone) systematically walks several readings *past* the
  real edge before confirming — every row gets truncated by roughly the
  same amount.
- N chosen too small (1-2) risks a single noisy off-wafer reading
  (dust, transient signal dropout) mid-wafer being misread as an edge,
  ending a row early.

N should be tuned empirically against the FOV and reading interval
actually in use, not picked as an arbitrary constant. Recommend exposing
both N and reading interval together in whatever calibration/preview tool
this mode gets (mirroring how `scan/calibrate_scan_area.py` previews grid size
before committing), so an operator can see the implied blur-zone reading
count before running a real scan.

## 7. Direction / hysteresis handling

**Decision: do not use separate +X/-X (or +Y/-Y) thresholds.** The
on/off-wafer signal is optical/thermal, tied to a fixed physical boundary —
the sensor doesn't know or care which way the mount is travelling, so
there's no physical reason the *signal* would differ by direction.

What is direction-dependent is the motor's step response (backlash/
hysteresis) — the same commanded pulse count can correspond to a different
real mm displacement depending on travel direction. This turns out not to
require special handling, for two reasons already inherent to the design:

1. The procedure never trusts a pulse count as a distance — it keeps
   issuing jog commands and polling until the *sensor* reports a
   transition. Backlash slack at the start of a reversed row (the first
   few commanded pulses taking up mechanical slack before real motion
   resumes) just costs a few "wasted" polls; it doesn't corrupt detection.
2. Final position within a row is assigned by reading *index*
   (`x_position = j / (N_valid - 1)`), never by commanded mm. If backlash
   means one direction packs slightly more or fewer readings into the same
   physical span than the other, that's absorbed automatically by the
   per-row-independent indexing — it doesn't need correcting.

Where direction *should* still be captured: logging, not detection. Record
`scan_direction` and total commanded pulses per row (both already on the
requested field list in §10) so backlash asymmetry is visible for later
analysis (e.g. "does +X consistently take more pulses to cross the same
wafer than -X"), without building direction-awareness into the threshold
logic itself.

Reading-interval definition interacts with this: if the interval is defined
by commanded motor pulses, a fixed pulse count maps to a different real mm
spacing in each direction (backlash again), meaning the *effective* map
resolution per row would vary by direction even though the index-based
position assignment still works. Defining the interval by acquisition time
instead sidesteps this. Per the defaults policy in §4, this isn't baked in
as a default — the operator picks either mode each run — but the
motor-pulse option should be labeled in the UI as direction-sensitive, so
whoever is choosing understands the tradeoff rather than discovering it
later in asymmetric row lengths.

## 8. Position assignment

For a row with N valid (in-bounds) readings, reading j (0-indexed) gets:

```
x_position = j / (N - 1)
```

j=0 maps to one edge, j=N-1 to the other, regardless of actual motor steps
commanded. Alternate rows are reversed in stored order so every row reads
left-to-right in the final map, per the existing raster/serpentine
convention already in `scan_manager.generate_grid()`.

This is a real approximation (assumes roughly uniform effective spacing
across the row) — acceptable and intended, per "approximate wafer mapping"
being the stated goal, not a defect to fix.

## 9. Safety: runtime max-travel guard

The existing pre-flight mechanism (`scan_manager.preflight_check()`,
`scan_params.validate_points_within_limits()`) validates a *complete,
known* list of commanded points against `motion.soft_limits` before any
hardware is touched. That doesn't apply here — this mode doesn't know its
points in advance.

**As implemented (`TravelGuard` in adaptive_scan/adaptive_scan.py), this differs from the
original idea above** of summing `|dx|`/`|dy|` across every jog: that
approach over-counts ordinary back-and-forth jitter during entry/exit
confirmation — many small jogs that mostly cancel out would still add up
to a large "cumulative" total despite barely displacing the mount.
Instead, the guard checks NET position via `motion.get_position()` /
`MotionController.check_limits()` against a fixed envelope
(`start ± max_travel_mm` on each axis) — the same open-loop,
step-count-derived position the existing precision scan path already
trusts for its own `soft_limits` check. That's a deliberate reuse: this
bound is a safety margin, not a map-accuracy claim, so leaning on
dead-reckoning position here is fine, the same way it already is
elsewhere in this codebase. Checked before and after every jog. This bound
also gates the step-13 termination sweep (§3) — implemented via a
separate bounded-iteration count in the row's ignore-phase, tuned to fire
at essentially the same point `TravelGuard` would, but reported as a
normal, expected scan-complete result rather than a `TravelLimitExceeded`
safety fault.

The fixed boundary searches in steps 2, 3, and 5 (the FIRST Y exit/entry,
and the first row's X exit — all of which happen before any row exists,
so step 13's "normal termination" framing doesn't apply to them) are
bounded the same way, via `_max_y_search_iterations`/`_max_search_iterations`
in `adaptive_scan.py`. Reaching that bound there raises `EdgeSearchFailed`
(a distinct, descriptive exception naming which step failed and why —
almost always a start-position or threshold/polarity mismatch, not a
"scan complete" outcome), rather than looping until `TravelGuard`
eventually trips somewhere far from the start position with no
indication of which step caused it. `TravelLimitExceeded` itself is now
caught inside `run()` (previously it was not): a genuine mid-scan safety
trip returns whatever rows/readings/QC flags/coarse grid were already
accumulated (each row is committed to the raw CSV as it finishes,
regardless), with `AdaptiveScanResult.status="aborted"` and
`stop_reason="travel_limit_exceeded"` — distinguishable from an operator-
requested stop (`stop_reason="operator_abort"`) but handled identically
by callers, since both preserve partial results rather than discarding
them. Fixed 2026-07-22 after a code-review pass surfaced both gaps.

## 10. Data logging

Per reading, retain: raw signal value(s) (temp/emissivity/dilution/OES
feature — whichever aren't the selected detection signal are still worth
recording if cheap to read), timestamp, row number, reading number within
row, scan direction, selected edge-detection signal, motor command(s)
issued (as a relative delta, not absolute position — consistent with §7),
and calculated normalized position.

This does not fit `OESStore`'s fixed `(nx, ny, npass)` HDF5 array (row
lengths vary). Recommend the existing flat-CSV pattern from
`scan/data_logger.py` (`_append_summary_row`) — crash-safe, append-per-reading,
already proven — over trying to force a ragged structure into HDF5. The
100-cell coarse grid (mean, count, stddev per cell) is then a derived
aggregation computed from the raw table after (or during) the scan, not
written incrementally the way `OESStore.write_point()` works today.

## 11. Row-level QC flags

Per row, flag:
1. Internal loss of wafer signal (a confirmed exit followed by a confirmed
   re-entry before the row's expected far edge) — worth surfacing since
   it's an interior anomaly, not just a boundary.
2. Unusually low valid-reading count.
3. A reading count that differs substantially from neighboring rows.
4. Evidence of stall or large motor slip (e.g. commanded pulse count far
   exceeding what any other row needed for a similar reading count).

These are analysis over the raw per-reading table (§10), not something
that needs to happen live during the scan.

## 12. Known limitations (confirmed with Roy)

- This method reduces the effect of small, ordinary positioning error
  (backlash, minor per-step variance) by never depending on absolute step
  count for position — but it **cannot recover the true position of
  readings if a large slip or stall occurs within a row.** If the mount
  loses steps or stalls mid-row, the index-based assignment
  (`x_position = j / (N-1)`, §8) still produces N evenly-spaced positions
  across the row — it has no way to detect that the physical spacing
  between some of those readings was not actually uniform. This is a
  fundamental limit of not tracking real position, not a bug to fix; the
  row-level QC flags (§11, especially #4 "stall or large motor slip") are
  the intended mitigation — they flag the row as suspect for a human to
  judge, rather than silently producing a wrong-but-confident map cell.
- Coarse-grid averaging (§10) is a deliberate reduction from ~1,000 raw
  readings to ~100 cells: the ~1-6mm field of view means many raw readings
  strongly overlap in what they're actually sampling, so displaying all
  1,000 as independent, precisely-located points would overstate the
  spatial resolution actually achieved. The 100-cell map is the honest
  representation of what this method can actually resolve.

## 13. Open questions

None remaining from the original design discussion. One implementation-
level call was made that's worth Roy's explicit sign-off, since it extends
beyond what was literally specified:

- **Y-axis normalization in the coarse grid.** §8 specifies
  `x = j / (N-1)` within a row. The original spec never defined an
  equivalent for the row (Y) axis. `build_coarse_grid()` (adaptive_scan/adaptive_scan_logger.py)
  applies the same principle to Y — `row_rank / (total_rows - 1)` — for the
  same underlying reason: total row count isn't known until the scan
  naturally terminates (step 13), exactly mirroring why a row's own N isn't
  known until that row ends. The raw per-reading CSV still stores the
  immutable `row_number` (not a derived Y value), so if a real,
  y_raster_spacing_mm-scaled Y axis is preferred instead for the displayed
  map, that's recoverable from the raw data without re-scanning — just a
  different aggregation step, not a different scan.

Everything else (recorded-read averaging, per-parameter defaults, coarse
grid cell count) is resolved per §2, §4, and §5 — readings are raw and
unaveraged at capture time; every parameter is direct operator input at
run time; averaging, count, and stddev all happen once, at the coarse-grid
aggregation step (§10).

## 14. Implementation plan (proposed)

Build as a new, separate module (e.g. `adaptive_scan/adaptive_scan.py` /
`AdaptiveRasterScanner`), sharing `motion_controller`, IR/OES readers, and
the `DataLogger`/CSV pattern with the existing code, rather than modifying
`scan/scan_manager.py`. The existing absolute-grid path is recent and actively
serves a different real use case (precision fixed-geometry scans,
multi-pass drift tracking per `scan.passes`) that stays valid once
closed-loop motion arrives — forking avoids destabilizing that path while
this one is being tuned on real hardware.

Build order and status:
1. **DONE** — Signal-selection dispatch + fast, unaveraged polling.
   `adaptive_scan/adaptive_scan_signal.py` (`RawSignals`, `read_raw_signals()`,
   `select_value()`). Smoke-tested against `mock_ir_reader.py`/
   `mock_spectrometer_reader.py`.
2. **DONE** — Entry/exit debounce state machine. `adaptive_scan/edge_detector.py`
   (`EdgeDetector`). Unit-tested standalone against synthetic sequences,
   including a noise blip and a synthetic FOV-blur ramp (no motion/reader
   involved at all).
3. **DONE** — Row scan loop (steps 6-9) using `motion.jog()`.
   `AdaptiveRasterScanner._scan_row()` in `adaptive_scan/adaptive_scan.py`.
4. **DONE** — Serpentine row-to-row logic (steps 2-5, 10-13), the bounded
   step-13 termination sweep, and the runtime max-travel guard (`TravelGuard`
   — see §9's updated description of how this actually works, which differs
   from this doc's original text). `AdaptiveRasterScanner.run()`.
5. **DONE** — Raw-reading CSV logger (`AdaptiveScanRawLogger`), row QC
   flagging (`compute_row_qc_flags()`), and coarse-grid aggregation
   (`build_coarse_grid()`) — all in `adaptive_scan/adaptive_scan_logger.py`.
   `adaptive_scan/adaptive_scan.py`'s own `__main__` smoke test runs the full 13-step
   procedure against a simulated circular wafer (dilution signal, radius
   20mm) on `MockMotionController`, with an injected transient signal dip
   to verify the internal-signal-loss QC flag actually fires — 13 rows
   scanned, correct serpentine reversal, clean step-13 termination, correct
   CSV round-trip (2022 readings), correct 10x10 coarse grid.
6. **NOT STARTED** — GUI wiring (new parameters, start/stop, live low-res
   preview — mirrors `gui/live_map.py`'s existing role of being the only
   real-time check available on open-loop hardware).
7. **NOT STARTED** — On-site validation against a real wafer, with the
   dilution-vs-other-signal comparison Roy flagged as needing experimental
   confirmation. Blocked in part on `ir.pac.dilution_tag_name` being
   confirmed on the real PAC (still unset in `config.yaml` — see
   `tools/list_pac_strategy_vars.py`).
