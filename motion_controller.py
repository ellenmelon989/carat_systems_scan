"""
motion_controller.py

Responsible for motor communication, homing, position commands,
limits, and settling for the 2D scanning mirror.

Provides a hardware abstraction: MotionController (abstract),
MockMotionController (for development without hardware), and
a real implementation to be filled in once motor/controller
hardware is identified (Phase 4).
"""

from abc import ABC, abstractmethod
import time


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

    def check_limits(self, x_mm, y_mm, limits):
        """Raise ValueError if (x_mm, y_mm) is outside soft limits."""
        if not (limits["x_min_mm"] <= x_mm <= limits["x_max_mm"]):
            raise ValueError(f"X position {x_mm} outside limits {limits['x_min_mm']}-{limits['x_max_mm']}")
        if not (limits["y_min_mm"] <= y_mm <= limits["y_max_mm"]):
            raise ValueError(f"Y position {y_mm} outside limits {limits['y_min_mm']}-{limits['y_max_mm']}")


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


class RealMotionController(MotionController):
    """
    Placeholder for real motor/controller hardware.

    TODO (Phase 4):
    - Identify motor type and controller (e.g. stepper + driver,
      servo, galvo, etc.)
    - Identify communication protocol (serial, USB, ethernet)
    - Implement home(), move_to(), get_position(), wait_for_settle()
    - Define coordinate system and units (mm vs steps vs degrees)
    """

    def __init__(self, port, **kwargs):
        self.port = port
        raise NotImplementedError("Real motion controller not yet implemented - Phase 4")

    def home(self):
        raise NotImplementedError

    def move_to(self, x_mm, y_mm):
        raise NotImplementedError

    def get_position(self):
        raise NotImplementedError

    def wait_for_settle(self, settle_time_s):
        raise NotImplementedError


def get_motion_controller(config):
    """
    Factory function. Returns a MockMotionController unless
    config specifies real hardware.

    Supported controller types:
      null / omitted   → MockMotionController (development)
      newport_esp301   → NewportESP301MotionController (real hardware)
    """
    motion_cfg = config.get("motion", {})
    controller_type = motion_cfg.get("controller")

    if controller_type is None:
        print("[motion_controller] No controller configured - using mock")
        return MockMotionController()

    if controller_type == "newport_esp301":
        from real_newport_motion import NewportESP301MotionController
        return NewportESP301MotionController(config)

    raise ValueError(
        f"Unknown motion controller type: '{controller_type}'. "
        "Supported: newport_esp301 | null (mock)"
    )


if __name__ == "__main__":
    # Simple smoke test using the mock controller
    mc = MockMotionController()
    mc.home()
    mc.move_to(10, 10)
    mc.wait_for_settle(0.2)
    print("Position:", mc.get_position())
