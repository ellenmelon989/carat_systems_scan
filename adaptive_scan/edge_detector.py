"""
edge_detector.py

Standalone entry/exit debounce state machine for the adaptive
(edge-following) raster scan — see docs/adaptive_scan_spec.md.

Deliberately has NO dependency on motion_controller or any reader: it only
ever sees a stream of raw scalar values fed to it one at a time via
update(), and reports confirmed transitions. That keeps it independently
unit-testable (see the smoke test below, which feeds it synthetic
sequences including a noise blip and a gradual FOV-blur ramp) without any
mock hardware at all.

Two thresholds (on_threshold, off_threshold), not one — this is a
Schmitt-trigger / hysteresis design, not a single cutoff. Which threshold
is numerically larger determines the polarity (whether higher signal
values mean "on wafer" or lower ones do) automatically, so the same class
works for a signal like dilution (higher = on wafer, say) or one where the
relationship runs the other way, without a separate "direction" flag the
operator would have to also get right.

A reading strictly between the two thresholds is AMBIGUOUS — this is the
expected, normal state for however many readings fall inside the
field-of-view blur zone at a real physical edge (see spec §2, §6).
Ambiguous readings do not reset progress toward a pending confirmation;
they simply don't advance it either. This is what keeps the confirm count
from being hypersensitive to exactly where in the blur zone a reading
happens to land, while still requiring genuinely consistent evidence
(confirm_count consecutive non-ambiguous readings, all on the same side)
before accepting a transition.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class EdgeDetector:
    """
    Parameters
    ----------
    on_threshold, off_threshold : float
        The two operator-set thresholds (spec §4, parameter 2). Whichever
        is larger determines polarity: if on_threshold > off_threshold,
        higher signal values mean "on wafer" (value >= on_threshold to
        classify ON, value <= off_threshold to classify OFF). If
        on_threshold < off_threshold, the relationship is reversed. Equal
        values collapse to a single cutoff with no ambiguous band at all
        (not an error, just no hysteresis margin).
    confirm_count : int
        Number of consecutive non-ambiguous readings, all agreeing, needed
        to confirm a transition. See spec §6 for how this must be chosen
        relative to the field-of-view blur width and reading interval —
        this class only enforces whatever value it's given; it does not
        validate that choice against FOV (see adaptive_scan_params.py for
        the plain sanity-bound validators, and the operator's own judgment
        for the FOV-aware tuning that constant checks can't capture).
    initial_state : str
        "off" (default) or "on" — what state to start in. Matters because
        this class only ever reports a *transition*, not "still off" — the
        very first update() call needs a known prior state to compare
        against.
    """

    on_threshold: float
    off_threshold: float
    confirm_count: int
    initial_state: str = "off"

    def __post_init__(self):
        if self.confirm_count < 1:
            raise ValueError(f"confirm_count must be >= 1, got {self.confirm_count}")
        if self.initial_state not in ("on", "off"):
            raise ValueError(f"initial_state must be 'on' or 'off', got {self.initial_state!r}")
        self.state = self.initial_state
        self._consecutive_on = 0
        self._consecutive_off = 0
        # Total readings classified into each bucket over this detector's
        # whole life, independent of confirm_count resets — useful for a
        # caller building QC stats (e.g. "this row spent N readings in the
        # ambiguous band," a proxy for how blurry that particular edge was).
        self.n_on_seen = 0
        self.n_off_seen = 0
        self.n_ambiguous_seen = 0

    @property
    def consecutive_on(self) -> int:
        """Read-only view of the current on-streak — used by callers
        (adaptive_scan.py's internal-signal-loss detection) that need to
        notice a streak *starting*, not just a confirmed transition."""
        return self._consecutive_on

    @property
    def consecutive_off(self) -> int:
        """Read-only view of the current off-streak — see consecutive_on."""
        return self._consecutive_off

    def _classify(self, value: float) -> str:
        hi, lo = self.on_threshold, self.off_threshold
        if hi == lo:
            return "on" if value >= hi else "off"
        if hi > lo:
            if value >= hi:
                return "on"
            if value <= lo:
                return "off"
            return "ambiguous"
        # on_threshold < off_threshold: polarity reversed
        if value <= hi:
            return "on"
        if value >= lo:
            return "off"
        return "ambiguous"

    def update(self, value: float) -> str:
        """
        Feed one raw reading. Returns one of:
          "entered_wafer" — confirm_count consecutive ON readings just
              flipped the confirmed state from off to on.
          "exited_wafer"  — confirm_count consecutive OFF readings just
              flipped the confirmed state from on to off.
          "no_change"     — anything else (still building toward a
              transition, already in the state this reading agrees with,
              or an ambiguous/blur-zone reading).

        Never raises on a value outside any expected range — an
        out-of-range or NaN reading from a failed sensor poll should be
        handled by the caller (e.g. treat a NaN as ambiguous / skip it
        before calling update()) rather than crash a live scan; this
        method itself just compares floats.
        """
        classification = self._classify(value)

        if classification == "on":
            self.n_on_seen += 1
            self._consecutive_on += 1
            self._consecutive_off = 0
        elif classification == "off":
            self.n_off_seen += 1
            self._consecutive_off += 1
            self._consecutive_on = 0
        else:
            self.n_ambiguous_seen += 1
            # Ambiguous: freeze both counters. Does not reset progress
            # (a real edge's blur zone shouldn't cost you your confirm
            # streak) and does not advance it either (an ambiguous
            # reading is not evidence for either side).

        if self.state != "on" and self._consecutive_on >= self.confirm_count:
            self.state = "on"
            self._consecutive_off = 0
            return "entered_wafer"

        if self.state != "off" and self._consecutive_off >= self.confirm_count:
            self.state = "off"
            self._consecutive_on = 0
            return "exited_wafer"

        return "no_change"

    def reset(self, initial_state: str | None = None):
        """
        Reset all counters and state — use at the start of each new row
        (§3 step 6: "begin the first row") rather than reusing one
        detector's confirm-streak across row boundaries, since a
        just-confirmed exit at the end of one row has nothing to do with
        the entry confirmation the next row needs.
        """
        self.state = self.initial_state if initial_state is None else initial_state
        self._consecutive_on = 0
        self._consecutive_off = 0
        self.n_on_seen = 0
        self.n_off_seen = 0
        self.n_ambiguous_seen = 0


if __name__ == "__main__":
    # Smoke test — synthetic sequences, no hardware/mock reader involved.

    # 1. Clean, unambiguous entry then exit (dilution-like: higher = on).
    d = EdgeDetector(on_threshold=0.9, off_threshold=0.7, confirm_count=3)
    seq = [0.5, 0.5, 0.95, 0.95, 0.95, 0.95, 0.5, 0.5, 0.5]
    results = [d.update(v) for v in seq]
    assert results == ["no_change", "no_change", "no_change", "no_change",
                        "entered_wafer", "no_change", "no_change", "no_change",
                        "exited_wafer"], results
    print("Clean entry/exit sequence OK:", results)

    # 2. A single noise blip should NOT trigger a false exit mid-wafer.
    d = EdgeDetector(on_threshold=0.9, off_threshold=0.7, confirm_count=3, initial_state="on")
    seq = [0.95, 0.95, 0.3, 0.95, 0.95]  # one bad low reading, then back on
    results = [d.update(v) for v in seq]
    assert "exited_wafer" not in results, f"single blip falsely confirmed an exit: {results}"
    print("Noise-blip rejection OK:", results)

    # 3. FOV-blur ramp: several ambiguous readings between clearly-on and
    #    clearly-off should not falsely confirm anything from the
    #    ambiguous readings themselves, and should still confirm the real
    #    exit once enough clearly-off readings arrive.
    d = EdgeDetector(on_threshold=0.9, off_threshold=0.7, confirm_count=3, initial_state="on")
    seq = [0.95, 0.95, 0.82, 0.80, 0.78, 0.5, 0.5, 0.5]  # ramp down through the blur band, then off
    results = [d.update(v) for v in seq]
    assert results.count("exited_wafer") == 1, results
    assert results[-1] == "exited_wafer", results
    # None of the ambiguous (0.82/0.80/0.78) readings should have confirmed anything.
    assert results[2] == "no_change" and results[3] == "no_change" and results[4] == "no_change"
    print("FOV-blur ramp handled OK:", results)

    # 4. Reversed polarity (on_threshold < off_threshold) — e.g. a signal
    #    where LOWER values mean on-wafer.
    d = EdgeDetector(on_threshold=0.2, off_threshold=0.4, confirm_count=2)
    seq = [0.5, 0.15, 0.15, 0.5, 0.5]
    results = [d.update(v) for v in seq]
    assert results == ["no_change", "no_change", "entered_wafer", "no_change", "exited_wafer"], results
    print("Reversed polarity OK:", results)

    # 5. reset() clears state for the next row.
    d.reset()
    assert d.state == "off" and d._consecutive_on == 0 and d._consecutive_off == 0
    print("reset() OK")

    print("edge_detector smoke test OK")
