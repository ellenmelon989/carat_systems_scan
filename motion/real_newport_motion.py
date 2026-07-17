"""
real_newport_motion.py

Newport 8742 Picomotor controller backend for the CARAT scanner.
Hardware: Newport 8742 controller + 8816-6 motorized mirror mounts.
Communication: Ethernet (TCP, port 23).

Implements MotionController using pylablib's Picomotor8742 driver.

Coordinate system
-----------------
The 8816-6 is an ANGULAR mount.  The scan grid is defined in mm of
displacement at the plasma surface.  Conversion is:

    steps = mm * steps_per_mm

steps_per_mm must be calibrated on-site (see CALIBRATION NOTE below).
Separate values for X and Y axes because the two mount axes may differ
in mechanical advantage and beam geometry.

CALIBRATION NOTE
----------------
To measure steps_per_mm for each axis:
  1. Put a target (paper, camera) at the plasma plane.
  2. Command 1000 steps on axis 1: python real_newport_motion.py <ip> --calibrate-x 1000
  3. Measure how far the beam spot moved (mm).
  4. steps_per_mm_x = 1000 / measured_mm
  5. Repeat for axis 2 (--calibrate-y).
  6. Enter both values in config.yaml under motion.

HOMING NOTE
-----------
The 8742 / picomotor system is open-loop — there are no encoders or
limit switches on the 8816-6.  "Homing" here means:
  - Drive toward the mount's mechanical hard stop at slow speed
    (limit_steps steps in the negative direction)
  - Reset the internal step counter to 0 at that position
  - This gives a repeatable origin across power cycles
The alternative (soft home = just zero the counter at current position)
is also supported; set hard_home: false in config.

Config keys (under motion:)
---------------------------
  controller: newport_8742
  interface: ethernet       # "ethernet" or "usb" (default: ethernet)
  host: "192.168.100.2"     # 8742 IP; only used when interface: ethernet
  tcp_port: 23              # 8742 Telnet port (default 23); ethernet only
  usb_index: 0              # 8742 USB enumeration index; only used when interface: usb
  axis_x: 1                 # 8742 axis number for X mirror axis
  axis_y: 2                 # 8742 axis number for Y mirror axis
  steps_per_mm_x: 500       # CALIBRATE ON-SITE (see above)
  steps_per_mm_y: 500       # CALIBRATE ON-SITE (see above)
  hard_home: true           # true = drive to hard stop; false = zero-in-place
  home_steps: 100000        # steps to drive toward hard stop during homing
  home_velocity: 200        # steps/s during homing (slow to avoid crash)
  move_velocity: 2000       # steps/s during normal scan moves
  home_timeout_s: 60        # per-axis home timeout
  move_timeout_s: 30        # per-axis move timeout

Usage
-----
    from real_newport_motion import NewportPicomotorController
    mc = NewportPicomotorController(config)
    mc.home()
    mc.move_to(10.0, 25.0)
    mc.wait_for_settle(0.5)
    print(mc.get_position())   # returns (mm, mm) based on step count
    mc.close()

As context manager:
    with NewportPicomotorController(config) as mc:
        mc.home()
        ...
"""

import time
import logging

try:
    from pylablib.devices import Newport
except ImportError as exc:
    raise ImportError(
        "pylablib is required. Install with: pip install pylablib"
    ) from exc

try:
    from .motion_controller import MotionController, MotionFault, AxisStateUnknown
except ImportError:
    # Fallback for running this file directly (e.g. python real_newport_motion.py),
    # where relative imports don't work because there's no parent package.
    from motion_controller import MotionController, MotionFault, AxisStateUnknown

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Defaults — all overridable in config.yaml under motion:
# ---------------------------------------------------------------------------
_DEFAULT_INTERFACE = "ethernet"    # "ethernet" or "usb"
_DEFAULT_HOST = "192.168.100.2"
_DEFAULT_PORT = 23
_DEFAULT_USB_INDEX = 0
_DEFAULT_AXIS_X = 1
_DEFAULT_AXIS_Y = 2
_DEFAULT_STEPS_PER_MM = 500        # placeholder — MUST calibrate on-site
_DEFAULT_HARD_HOME = True
_DEFAULT_HOME_STEPS = 100_000      # large enough to always reach the hard stop
_DEFAULT_HOME_VELOCITY = 200       # steps/s — slow for safety at hard stop
_DEFAULT_MOVE_VELOCITY = 2000      # steps/s — tune for speed vs. vibration
_DEFAULT_HOME_TIMEOUT = 60.0
_DEFAULT_MOVE_TIMEOUT = 60.0
_DEFAULT_STOP_CONFIRM_TIMEOUT = 10.0  # how long to wait for stop() to actually
                                       # take effect before giving up on it too
_STOP_CONFIRM_POLL_S = 0.05
_SETTLE_CONFIRM_DELAY_S = 0.1  # gap before the second "not moving" read that
                                # confirms a settle wasn't just a stale/transient
                                # status reply


class NewportPicomotorController(MotionController):
    """
    Real motion controller: Newport 8742 Picomotor over Ethernet.

    Position tracking is in steps (open-loop).  Public API converts
    to/from mm using steps_per_mm_x / steps_per_mm_y.

    Thread safety: NOT thread-safe (scan loop is single-threaded).
    """

    def __init__(self, config: dict):
        motion_cfg = config.get("motion", {})

        interface = str(motion_cfg.get("interface", _DEFAULT_INTERFACE)).lower()
        if interface not in ("ethernet", "usb"):
            raise ValueError(
                f"Unknown motion.interface: '{interface}'. Supported: ethernet | usb"
            )

        host = motion_cfg.get("host", _DEFAULT_HOST)
        tcp_port = int(motion_cfg.get("tcp_port", _DEFAULT_PORT))
        usb_index = int(motion_cfg.get("usb_index", _DEFAULT_USB_INDEX))

        self._axis_x = int(motion_cfg.get("axis_x", _DEFAULT_AXIS_X))
        self._axis_y = int(motion_cfg.get("axis_y", _DEFAULT_AXIS_Y))

        self._steps_per_mm_x = float(
            motion_cfg.get("steps_per_mm_x", _DEFAULT_STEPS_PER_MM)
        )
        self._steps_per_mm_y = float(
            motion_cfg.get("steps_per_mm_y", _DEFAULT_STEPS_PER_MM)
        )

        self._hard_home = bool(motion_cfg.get("hard_home", _DEFAULT_HARD_HOME))
        self._home_steps = int(motion_cfg.get("home_steps", _DEFAULT_HOME_STEPS))
        self._home_velocity = int(motion_cfg.get("home_velocity", _DEFAULT_HOME_VELOCITY))
        self._move_velocity = int(motion_cfg.get("move_velocity", _DEFAULT_MOVE_VELOCITY))
        self._home_timeout = float(motion_cfg.get("home_timeout_s", _DEFAULT_HOME_TIMEOUT))
        self._move_timeout = float(motion_cfg.get("move_timeout_s", _DEFAULT_MOVE_TIMEOUT))
        self._stop_confirm_timeout = float(
            motion_cfg.get("stop_confirm_timeout_s", _DEFAULT_STOP_CONFIRM_TIMEOUT)
        )

        self._homed = False
        # Internal step-count origin (set during home)
        self._origin_x = 0
        self._origin_y = 0

        if self._steps_per_mm_x == _DEFAULT_STEPS_PER_MM:
            logger.warning(
                "steps_per_mm_x is using the default placeholder value (%g). "
                "Calibrate on-site and update config.yaml.",
                _DEFAULT_STEPS_PER_MM,
            )
        if self._steps_per_mm_y == _DEFAULT_STEPS_PER_MM:
            logger.warning(
                "steps_per_mm_y is using the default placeholder value (%g). "
                "Calibrate on-site and update config.yaml.",
                _DEFAULT_STEPS_PER_MM,
            )

        if interface == "usb":
            conn = usb_index
            conn_desc = f"USB index {usb_index}"
        else:
            conn = f"{host}:{tcp_port}"
            conn_desc = f"{host}:{tcp_port}"

        logger.info(
            "Connecting to Newport 8742 via %s (%s) (X=axis%d, Y=axis%d)",
            interface, conn_desc, self._axis_x, self._axis_y,
        )

        try:
            self._stage = Newport.Picomotor8742(conn=conn)
        except Exception as exc:
            if interface == "usb":
                raise RuntimeError(
                    f"Failed to connect to Newport 8742 over USB (index {usb_index}): {exc}\n"
                    "Check: (1) Picomotor Application drivers installed? "
                    "(2) status LED solid green? (3) correct usb_index if multiple "
                    "controllers are connected?"
                ) from exc
            raise RuntimeError(
                f"Failed to connect to Newport 8742 at {host}:{tcp_port}: {exc}\n"
                "Check: (1) IP address correct? (2) cable plugged in? "
                "(3) controller powered on?"
            ) from exc

        logger.info("8742 connected.")
        self._set_velocity(self._axis_x, self._move_velocity)
        self._set_velocity(self._axis_y, self._move_velocity)

    # ------------------------------------------------------------------
    # MotionController interface
    # ------------------------------------------------------------------

    def home(self):
        """
        Home both axes.

        hard_home=True  → drives each axis toward its mechanical hard
                          stop for home_steps steps, then zeros the
                          internal counter. Gives a repeatable absolute
                          origin across power cycles.
        hard_home=False → zeros the step counter at the current
                          position (soft/in-place home). Faster but
                          origin shifts if the mount is moved by hand.
        """
        if self._hard_home:
            self._hard_home_axis(self._axis_x, label="X")
            self._hard_home_axis(self._axis_y, label="Y")
        else:
            logger.info("Soft-homing X (axis %d): zeroing counter in place", self._axis_x)
            self._stage.set_position_reference(axis=self._axis_x, position=0)
            logger.info("Soft-homing Y (axis %d): zeroing counter in place", self._axis_y)
            self._stage.set_position_reference(axis=self._axis_y, position=0)

        self._origin_x = self._stage.get_position(axis=self._axis_x)
        self._origin_y = self._stage.get_position(axis=self._axis_y)
        self._homed = True
        logger.info(
            "Homing complete. Step origin: X=%d, Y=%d",
            self._origin_x, self._origin_y,
        )

    def resume(self):
        """
        Mark this controller ready to move WITHOUT re-homing — for a
        fresh process (e.g. scan_manager.py) picking up right after a
        previous process (e.g. calibrate_scan_area.py) already called
        home() and jogged around, in the same physical session.

        hard_home=True: just calls home() — idempotent, always drives
        back to the same mechanical stop, so there's no real
        distinction from resuming.

        hard_home=False: does NOT call set_position_reference() again
        (that's what real home() does, and it's destructive here — it
        would zero at wherever the stage happens to be right now,
        discarding the previous process's origin rather than
        restoring it). Instead just sets _homed=True and leaves
        _origin_x/_origin_y at their __init__ default of 0.

        Why 0 is correct, not just "not obviously wrong": home() sets
        _origin_x/_origin_y by reading the position back from the 8742
        immediately after zeroing it (lines above), and a step counter
        reads exactly 0 immediately after being told "you are 0" — so
        _origin_x/_origin_y ARE 0 right after any real home() call.
        Leaving this fresh object's origin at its 0 default reproduces
        that state exactly, provided the 8742's own position register
        (not this Python object) is what's authoritative and it hasn't
        been re-zeroed or the controller power-cycled since the last
        real home().

        CAVEAT — not independently verified against this specific
        pylablib version / 8742 firmware: this assumes
        set_position_reference() writes to a register that lives on
        the controller itself and persists across a fresh serial/USB
        connection, rather than being a pylablib-side, connection-
        scoped offset that resets when this process reconnects. Most
        real motion controllers work the former way (that's the whole
        point of a hardware "zero" command), but if position readings
        look wrong after using resume() instead of home(), this
        assumption is the first thing to check — e.g. by comparing
        get_position() right after connecting fresh vs. what it was
        at the end of the previous process, before commanding any
        move.
        """
        if self._hard_home:
            self.home()
        else:
            self._homed = True
            logger.info(
                "Resuming (soft home) without re-zeroing: trusting the "
                "8742's own position register still reflects the last "
                "real home(). Origin left at (0, 0), matching what "
                "home() would set it to right after zeroing."
            )

    def zero_here(self):
        """
        Zero BOTH axes at the stage's current physical position,
        unconditionally — regardless of self._hard_home. This is the
        "soft home" primitive exposed directly (not gated behind hard_home
        the way home() is), so a caller that has already jogged to and
        visually confirmed a fixed reference mark can anchor the origin
        there without accidentally triggering a mechanical hard-stop
        drive if hard_home happens to be True in config.

        Unlike home(), this never drives anywhere — it just labels
        wherever the stage currently is as (0, 0). It's only as
        trustworthy as the caller's confirmation that "wherever the
        stage currently is" is actually the intended reference point;
        calibrate_scan_area.py's clearance check + reference-mark jog
        loop is what provides that confirmation. See MEMORY
        carat_scanner_2026-07-17_scan_diagnosis for the fiducial-homing
        rationale (avoids ever needing to characterize home_steps/
        home_velocity or drive into a hard mechanical stop at all).
        """
        logger.info("Zeroing X (axis %d) at current position", self._axis_x)
        self._stage.set_position_reference(axis=self._axis_x, position=0)
        logger.info("Zeroing Y (axis %d) at current position", self._axis_y)
        self._stage.set_position_reference(axis=self._axis_y, position=0)
        self._origin_x = self._stage.get_position(axis=self._axis_x)
        self._origin_y = self._stage.get_position(axis=self._axis_y)
        self._homed = True
        logger.info(
            "Zeroed at current position. Step origin: X=%d, Y=%d",
            self._origin_x, self._origin_y,
        )

    def move_to(self, x_mm: float, y_mm: float):
        """
        Absolute move to (x_mm, y_mm) in scan-grid coordinates.

        Converts mm → steps using steps_per_mm_x / _y, offsets by the
        homed origin, and issues both axis moves simultaneously.
        Returns immediately; call wait_for_settle() to block.
        """
        if not self._homed:
            raise RuntimeError("Must call home() before move_to().")

        target_x = self._origin_x + round(x_mm * self._steps_per_mm_x)
        target_y = self._origin_y + round(y_mm * self._steps_per_mm_y)

        logger.debug(
            "move_to(%.4f mm, %.4f mm) → steps (%d, %d)",
            x_mm, y_mm, target_x, target_y,
        )

        self._stage.move_to(axis=self._axis_x, position=target_x)
        self._stage.move_to(axis=self._axis_y, position=target_y)

    def get_position(self) -> tuple[float, float]:
        """
        Return current position as (x_mm, y_mm).

        Derived from the internal step counter (open-loop — no encoder).
        Only accurate if no steps have been lost (normal operation).
        """
        try:
            sx = self._stage.get_position(axis=self._axis_x) - self._origin_x
            sy = self._stage.get_position(axis=self._axis_y) - self._origin_y
            return (sx / self._steps_per_mm_x, sy / self._steps_per_mm_y)
        except Exception as exc:
            logger.warning("get_position() failed: %s", exc)
            return (0.0, 0.0)

    def wait_for_settle(self, settle_time_s: float):
        """
        Block until both axes have stopped, then sleep settle_time_s.
        """
        self._wait_move(self._axis_x, self._move_timeout, label="Wait X")
        self._wait_move(self._axis_y, self._move_timeout, label="Wait Y")
        if settle_time_s > 0:
            logger.debug("Settling %.3f s", settle_time_s)
            time.sleep(settle_time_s)

    # ------------------------------------------------------------------
    # Resource management
    # ------------------------------------------------------------------

    def close(self):
        try:
            self._stage.close()
            logger.info("8742 connection closed.")
        except Exception as exc:
            logger.warning("Error closing 8742 connection: %s", exc)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _hard_home_axis(self, axis: int, label: str):
        """
        Drive axis toward the mechanical hard stop, then zero counter.

        Uses a slow velocity to avoid damaging the mount at the stop.
        The picomotor stalls harmlessly at the hard stop (no encoder
        means it just stops counting).  After the timeout, we assume
        the stop has been reached and zero the position.
        """
        logger.info(
            "Hard-homing %s (axis %d): driving %d steps at %d steps/s",
            label, axis, self._home_steps, self._home_velocity,
        )
        self._set_velocity(axis, self._home_velocity)
        # Negative direction = toward hard stop (physical mount minimum)
        self._stage.move_by(axis=axis, steps=-self._home_steps)
        # Wait for motion to stop (stall at hard stop or steps exhaust)
        self._wait_move(axis, self._home_timeout, label=f"Home {label}")
        # Zero the counter here = define this as the origin
        self._stage.set_position_reference(axis=axis, position=0)
        logger.info("%s axis homed and zeroed.", label)
        # Restore normal velocity for scan moves
        self._set_velocity(axis, self._move_velocity)

    def _wait_move(self, axis: int, timeout_s: float, label: str = ""):
        """
        Block until axis is confirmed idle, or raise.

        "Confirmed" means two consecutive is_moving()==False reads,
        separated by _SETTLE_CONFIRM_DELAY_S, not just one. A single
        stale/transient "not moving" reply (comms lag, status-register
        desync on the 8742) is exactly how a caller could conclude a
        move finished and issue the next move_to() while the axis is
        still physically working through leftover distance from this
        one — which is how a routine 2mm step can silently inherit
        real travel from a much larger prior move (e.g. a reference-
        point revisit) and blow way past its own timeout. The double-
        read costs one extra ~0.1s poll per successful move; cheap
        insurance against that failure mode.

        On timeout, we don't just fire-and-forget a stop() and hand
        control back — we actively wait (up to _stop_confirm_timeout)
        for the stop to actually take effect before raising, so that
        by the time this call returns control to the caller, the axis
        is DEFINITELY not moving. That's what makes it safe for
        scan_manager to retry with a fresh move_to() afterward: the
        raised MotionFault means "confirmed stopped, safe to re-issue
        a move." If even the stop can't be confirmed within
        _stop_confirm_timeout, we raise AxisStateUnknown instead — the
        axis's real state is unverified and a caller MUST NOT respond
        by sending another absolute move on top of it.
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                if not self._stage.is_moving(axis=axis):
                    time.sleep(_SETTLE_CONFIRM_DELAY_S)
                    if not self._stage.is_moving(axis=axis):
                        return
                    logger.debug(
                        "[%s] Axis %d reported not-moving then moving again on "
                        "confirm read — treating as still in motion.",
                        label, axis,
                    )
            except Exception as exc:
                raise RuntimeError(
                    f"[{label}] Lost communication with 8742 while waiting: {exc}"
                ) from exc
            time.sleep(0.05)

        # Timed out waiting for the move itself. Command a stop, then
        # actively confirm it took effect before raising — see docstring.
        try:
            self._stage.stop(axis=axis)
        except Exception as exc:
            raise AxisStateUnknown(
                f"[{label}] Axis {axis} did not stop within {timeout_s:.1f} s, "
                f"and the follow-up stop() command itself failed ({exc}). "
                "Axis state is unknown — do not issue further moves without "
                "checking the hardware."
            ) from exc

        stop_deadline = time.monotonic() + self._stop_confirm_timeout
        while time.monotonic() < stop_deadline:
            try:
                if not self._stage.is_moving(axis=axis):
                    raise MotionFault(
                        f"[{label}] Axis {axis} did not stop within {timeout_s:.1f} s "
                        f"(stop confirmed {self._stop_confirm_timeout - (stop_deadline - time.monotonic()):.1f} s "
                        "after an explicit stop() command)"
                    )
            except MotionFault:
                raise
            except Exception:
                # Comm hiccup while confirming the stop — can't tell if it's
                # actually idle. Fall through to the unconfirmed-state raise
                # below rather than guessing.
                break
            time.sleep(_STOP_CONFIRM_POLL_S)

        raise AxisStateUnknown(
            f"[{label}] Axis {axis} did not stop within {timeout_s:.1f} s, AND "
            f"did not confirm stopped within {self._stop_confirm_timeout:.1f} s "
            "after an explicit stop() command. Axis state is unknown — do not "
            "issue further moves without checking the hardware."
        )

    def _set_velocity(self, axis: int, velocity: int):
        """Set axis velocity in steps/s."""
        try:
            self._stage.setup_velocity(axis=axis, speed=velocity)
        except Exception as exc:
            logger.warning("Could not set velocity on axis %d: %s", axis, exc)


# ---------------------------------------------------------------------------
# CLI: connection smoke test + calibration helper
#
# Note: the factory function that picks Mock vs. NewportPicomotorController
# lives in motion_controller.py (get_motion_controller) — that's the only
# copy. scan_manager.py and calibrate_scan_area.py both import from there.
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="Newport 8742 smoke test and calibration helper"
    )
    parser.add_argument("host", nargs="?", default=None,
                        help="8742 IP address (e.g. 192.168.100.2); ignored if --usb is given")
    parser.add_argument("--usb", action="store_true",
                        help="Connect over USB instead of Ethernet")
    parser.add_argument("--usb-index", type=int, default=0,
                        help="USB enumeration index if multiple controllers are connected (default: 0)")
    parser.add_argument("--axis-x", type=int, default=1)
    parser.add_argument("--axis-y", type=int, default=2)
    parser.add_argument("--move-x", type=float, default=None,
                        help="Move X by this many mm (requires steps-per-mm-x)")
    parser.add_argument("--move-y", type=float, default=None,
                        help="Move Y by this many mm (requires steps-per-mm-y)")
    parser.add_argument("--steps-per-mm-x", type=float, default=_DEFAULT_STEPS_PER_MM)
    parser.add_argument("--steps-per-mm-y", type=float, default=_DEFAULT_STEPS_PER_MM)
    parser.add_argument("--move-velocity", type=int, default=_DEFAULT_MOVE_VELOCITY,
                        help=(
                            "steps/s to run moves AND calibration at (default: %(default)d). "
                            "Open-loop step size is velocity-dependent — calibration is only "
                            "valid for the velocity it was measured at. Always pass the same "
                            "value your config.yaml uses for motion.move_velocity."
                        ))
    parser.add_argument("--calibrate-x", type=int, default=None,
                        help="Drive axis X by N steps (measure beam displacement to get steps/mm)")
    parser.add_argument("--calibrate-y", type=int, default=None,
                        help="Drive axis Y by N steps (measure beam displacement to get steps/mm)")
    args = parser.parse_args()

    if not args.usb and args.host is None:
        parser.error("host is required unless --usb is given")

    cfg = {
        "motion": {
            "controller": "newport_8742",
            "interface": "usb" if args.usb else "ethernet",
            "host": args.host,
            "usb_index": args.usb_index,
            "axis_x": args.axis_x,
            "axis_y": args.axis_y,
            "steps_per_mm_x": args.steps_per_mm_x,
            "steps_per_mm_y": args.steps_per_mm_y,
            "hard_home": False,   # soft home for smoke test — safer
            "move_velocity": args.move_velocity,
        }
    }

    with NewportPicomotorController(cfg) as mc:

        if args.calibrate_x is not None:
            print(f"Calibration: moving axis {args.axis_x} by {args.calibrate_x} steps "
                  f"at {args.move_velocity} steps/s...")
            print("NOTE: this must match motion.move_velocity in config.yaml, since open-loop "
                  "step size is velocity-dependent. Pass --move-velocity to match if it differs "
                  "from the default.")
            mc._stage.move_by(axis=args.axis_x, steps=args.calibrate_x)
            mc._wait_move(args.axis_x, 30, "calibrate X")
            print(f"Done. Measure beam displacement in mm, then: steps_per_mm_x = {args.calibrate_x} / measured_mm")
            print("Repeat this run 3-5x in the same direction and average the result — "
                  "single-shot measurements are noisy on an open-loop inertial drive. "
                  "Record the velocity, direction, and date in config.yaml's "
                  "calibration_velocity_sps / calibration_direction / calibration_date fields.")

        elif args.calibrate_y is not None:
            print(f"Calibration: moving axis {args.axis_y} by {args.calibrate_y} steps "
                  f"at {args.move_velocity} steps/s...")
            print("NOTE: this must match motion.move_velocity in config.yaml, since open-loop "
                  "step size is velocity-dependent. Pass --move-velocity to match if it differs "
                  "from the default.")
            mc._stage.move_by(axis=args.axis_y, steps=args.calibrate_y)
            mc._wait_move(args.axis_y, 30, "calibrate Y")
            print(f"Done. Measure beam displacement in mm, then: steps_per_mm_y = {args.calibrate_y} / measured_mm")
            print("Repeat this run 3-5x in the same direction and average the result — "
                  "single-shot measurements are noisy on an open-loop inertial drive. "
                  "Record the velocity, direction, and date in config.yaml's "
                  "calibration_velocity_sps / calibration_direction / calibration_date fields.")

        else:
            print("=== Homing (soft) ===")
            mc.home()
            print(f"Position: {mc.get_position()}")

            if args.move_x is not None or args.move_y is not None:
                x = args.move_x or 0.0
                y = args.move_y or 0.0
                print(f"\n=== Moving to ({x} mm, {y} mm) ===")
                mc.move_to(x, y)
                mc.wait_for_settle(0.3)
                print(f"Position: {mc.get_position()}")

                print("\n=== Returning to origin ===")
                mc.move_to(0.0, 0.0)
                mc.wait_for_settle(0.3)
                print(f"Final position: {mc.get_position()}")

    print("\nDone.")
