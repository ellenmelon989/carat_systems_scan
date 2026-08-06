"""
motion_controller.py

Responsible for motor communication, homing, position commands,
limits, and settling for the 2D scanning mirror.

Provides a hardware abstraction: MotionController (abstract),
MockMotionController (for development without hardware),
NewportPicomotorController (real Newport 8742 + 8816-6 hardware, see
real_newport_motion.py -- open-loop, being phased out), and
ConexAGAPController (real Newport CONEX-AGAP + AG-M100D hardware, see
real_conexagap_motion.py -- closed-loop, its replacement).
"""

from abc import ABC, abstractmethod
import time


class MotionFault(RuntimeError):
    """
    A move/settle failed (axis timeout, stall, comm error) but the
    controller was able to command a stop and CONFIRM the axis is now
    actually idle. It's safe for a caller to issue a fresh move_to()
    after this — the axis isn't secretly still working through
    leftover distance from the move that failed.
    """


class AxisStateUnknown(MotionFault):
    """
    A move/settle failed AND the follow-up stop could not be confirmed
    (the axis never reported "not moving" even after an explicit stop
    command). The axis's real physical state is unknown — it may still
    be traveling. Callers MUST NOT respond to this by issuing another
    move_to(): a new absolute-position command layered on top of
    unconfirmed motion is exactly how a small routine step can silently
    inherit a large leftover distance from a prior move. Treat this as
    non-retryable — flag the point/scan and require a human to check
    the hardware before moving again.
    """


class MotionController(ABC):
    """Abstract interface for 2D mirror motion control."""

    @abstractmethod
    def home(self):
        """Home the axes. Should be called before any moves."""
        raise NotImplementedError

    @abstractmethod
    def move_to(self, x_mm, y_mm):
        """Command an absolute move to (x_mm, y_mm)."""
        raise NotImplementedError

    @abstractmethod
    def get_position(self):
        """Return (x_mm, y_mm) actual or last-commanded position."""
        raise NotImplementedError

    @abstractmethod
    def wait_for_settle(self, settle_time_s):
        """Block until the mirror has settled after a move."""
        raise NotImplementedError

    @abstractmethod
    def zero_here(self):
        """
        Zero the origin at the stage's CURRENT physical position,
        unconditionally — unlike home(), this is never gated by
        motion.hard_home and never drives toward a mechanical stop.

        Exists for fiducial-based homing: a caller that has already
        jogged to, and had an operator visually confirm, a fixed
        reference mark can anchor the origin there directly. This avoids
        driving into a hard mechanical stop at all (no home_steps/
        home_velocity to characterize, no stall-contact risk, no blind
        step-count timing) — the origin is instead only as good as the
        operator's visual confirmation, which is the same verification
        already relied on for the wafer-edge jogs in
        calibrate_scan_area.py. See that script's clearance-check +
        reference-mark jog for how this gets called safely.
        """
        raise NotImplementedError

    def check_limits(self, x_mm, y_mm, limits):
        """Raise ValueError if (x_mm, y_mm) is outside soft limits."""
        if not (limits["x_min_mm"] <= x_mm <= limits["x_max_mm"]):
            raise ValueError(f"X position {x_mm} outside limits {limits['x_min_mm']}-{limits['x_max_mm']}")
        if not (limits["y_min_mm"] <= y_mm <= limits["y_max_mm"]):
            raise ValueError(f"Y position {y_mm} outside limits {limits['y_min_mm']}-{limits['y_max_mm']}")

    def resume(self):
        """
        Mark the controller ready to move without re-homing, for a
        process continuing after a previous home() rather than starting
        fresh (e.g. scan_manager.py picking up right after
        calibrate_scan_area.py already homed and jogged).

        Default: just calls home(). Safe for controllers where "resume"
        and "home" are the same thing (the mock, or real hardware in
        hard_home mode, where home() is idempotent anyway).
        NewportPicomotorController overrides this for soft-home mode,
        where re-calling home() would be destructive rather than a
        no-op — see its docstring.
        """
        self.home()

    def jog(self, dx_mm: float = 0.0, dy_mm: float = 0.0):
        """
        Relative move by (dx_mm, dy_mm) from the current position.

        Built on top of get_position()/move_to(), so every MotionController
        subclass gets it for free. Used by the interactive edge-calibration
        workflow (see calibrate_scan_area.py) where the operator jogs the
        aim spot to the wafer edges to define the scan area — deliberately
        NOT clamped by motion.soft_limits, since that calibration process is
        what defines the safe area in the first place. The physical hard
        stops are the only real limit while jogging; the operator watches
        the mirror.
        """
        x, y = self.get_position()
        self.move_to(x + dx_mm, y + dy_mm)
        self.wait_for_settle(0.0)

    def calibration_jog(self, dx_mm: float = 0.0, dy_mm: float = 0.0):
        """
        Relative move by (dx_mm, dy_mm), for use ONLY by the interactive
        calibration workflow (calibrate_scan_area.py / gui/calibration_panel.py)
        — i.e. before scan-grid calibration exists yet.

        Default implementation is identical to jog(): most controllers gate
        motion the same way regardless of calibration state, so this is a
        no-op distinction for them. ConexAGAPController overrides it: that
        driver's ordinary move_to() (and therefore jog()) additionally
        requires motion.calibration_confirmed, a flag that can only honestly
        be set AFTER this same jog-to-the-4-edges workflow has produced real
        measured deg/mm numbers. Routing the calibration workflow through
        calibration_jog() instead of jog() breaks that chicken-and-egg: it
        still requires motion.motion_enabled and still validates against
        live controller limits, it just doesn't require calibration to
        already be confirmed. See ConexAGAPController.calibration_jog()'s
        docstring for the full rationale.
        """
        self.jog(dx_mm=dx_mm, dy_mm=dy_mm)

    def calibration_jog_deg(self, dx_deg: float = 0.0, dy_deg: float = 0.0):
        """
        Relative move by (dx_deg, dy_deg) in the controller's OWN native
        rotational unit (degrees), completely bypassing whatever mm
        conversion factor the controller uses — i.e. safe to call even
        before that factor is known at all.

        No sensible default: a controller with no native "degrees of
        tilt" concept (the open-loop linear-step 8742/picomotor, or the
        mock) has nothing to bypass to, so the base implementation raises.
        Only ConexAGAPController overrides this — see its docstring for
        why a degree-first calibration pass matters specifically for that
        controller (deg_per_mm_x/y can't safely be guessed at, unlike the
        8742's steps_per_mm_x/y, because the CONEX's full travel is only
        about a degree wide in either direction).
        """
        raise NotImplementedError(
            f"{type(self).__name__} has no native rotational unit to jog "
            "in -- use calibration_jog(dx_mm=..., dy_mm=...) instead."
        )

    def get_position_deg(self) -> tuple:
        """
        Live position in the controller's own native rotational unit
        (degrees), relative to origin — the counterpart to
        calibration_jog_deg(). See that method's docstring.
        """
        raise NotImplementedError(
            f"{type(self).__name__} has no native rotational unit to "
            "report position in -- use get_position() instead."
        )

    def get_absolute_position_deg(self) -> tuple:
        """
        Live position in the controller's own native rotational unit
        (degrees), WITHOUT subtracting the origin -- i.e. the exact raw
        value the hardware's own software limits (SL/SR on the CONEX)
        are expressed in and checked against. get_position_deg() is
        relative to wherever zero_here()/home() last anchored the
        origin, which is convenient for scan-grid math but can be
        confusing when debugging an "outside stored limits" fault: a
        small-looking relative delta can still push the RAW target past
        an absolute hardware limit if the origin itself sits close to
        that limit. This method exists so the operator can see that
        raw number directly instead of inferring it. See MEMORY
        carat_scanner_2026-08-06_gui_absolute_position_display.

        No sensible default, same reasoning as get_position_deg().
        """
        raise NotImplementedError(
            f"{type(self).__name__} has no native rotational unit to "
            "report position in -- use get_position() instead."
        )


class MockMotionController(MotionController):
    """
    Simulated motion controller for development/testing without
    physical hardware. Moves are instantaneous; position is tracked
    in memory.
    """

    def __init__(self):
        self._position = (0.0, 0.0)
        self._homed = False

    def home(self):
        print("[MockMotion] Homing...")
        time.sleep(0.1)
        self._position = (0.0, 0.0)
        self._homed = True
        print("[MockMotion] Homed to (0, 0)")

    def move_to(self, x_mm, y_mm):
        if not self._homed:
            raise RuntimeError("Must home before moving")
        print(f"[MockMotion] Moving to ({x_mm}, {y_mm})")
        self._position = (x_mm, y_mm)

    def get_position(self):
        return self._position

    def wait_for_settle(self, settle_time_s):
        time.sleep(settle_time_s)

    def zero_here(self):
        print("[MockMotion] Zeroing at current position (fiducial reference)")
        self._position = (0.0, 0.0)
        self._homed = True


def get_motion_controller(config):
    """
    Factory function. Returns a MockMotionController unless
    config specifies real hardware.

    Supported controller types:
      null / omitted   → MockMotionController (development)
      newport_8742      → NewportPicomotorController (real hardware; open-loop
                           8742 + 8816-6 picomotor mount -- being phased out,
                           see real_newport_motion.py)
      conex_agap         → ConexAGAPController (real hardware; closed-loop
                           CONEX-AGAP + AG-M100D mirror mount -- see
                           real_conexagap_motion.py for why this replaces
                           newport_8742 and what config keys it needs)
    """
    motion_cfg = config.get("motion", {})
    controller_type = motion_cfg.get("controller")

    if controller_type is None:
        print("[motion_controller] No controller configured - using mock")
        return MockMotionController()

    if controller_type == "newport_8742":
        try:
            from .real_newport_motion import NewportPicomotorController
        except ImportError:
            # Fallback for running this file directly, where relative imports
            # don't work because there's no parent package.
            from real_newport_motion import NewportPicomotorController
        return NewportPicomotorController(config)

    if controller_type == "conex_agap":
        try:
            from .real_conexagap_motion import ConexAGAPController
        except ImportError:
            # Fallback for running this file directly, where relative imports
            # don't work because there's no parent package.
            from real_conexagap_motion import ConexAGAPController
        return ConexAGAPController(config)

    raise ValueError(
        f"Unknown motion controller type: '{controller_type}'. "
        "Supported: newport_8742 | conex_agap | null (mock)"
    )


if __name__ == "__main__":
    # Simple smoke test using the mock controller
    mc = MockMotionController()
    mc.home()
    mc.move_to(10, 10)
    mc.wait_for_settle(0.2)
    print("Position:", mc.get_position())
