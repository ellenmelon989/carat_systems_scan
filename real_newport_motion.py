"""
real_newport_motion.py

Newport ESP301 (or ESP300/302) backend for the CARAT scanner motion system.
Implements MotionController using pylablib's Newport driver over RS-232/USB-serial.

Hardware assumptions
--------------------
- Newport ESP301 multi-axis controller, firmware >= 3.x
- Axis 1 → X, Axis 2 → Y  (overridable via config)
- Controller units pre-configured to millimetres (see UNITS NOTE below)
- RS-232 at 19200 baud, 8N1, hardware flow control OFF (ESP301 default)

UNITS NOTE: The ESP301 stores per-axis units in non-volatile memory.
Before first use, verify with `1SN?` (axis 1 units). Value 2 = mm.
If not already mm, run once from a terminal:
    1SN2   (set axis 1 to mm)
    2SN2   (set axis 2 to mm)
    SM     (save to NVM)
This driver does NOT override the on-controller unit setting at runtime
to avoid clobbering a carefully calibrated setup.

Config keys (under motion:)
---------------------------
  controller: newport_esp301
  port: "COM3"          # Windows: "COM3"; Linux: "/dev/ttyUSB0"
  baud: 19200           # default; most ESP301s ship at 19200
  axis_x: 1            # ESP301 axis number for X
  axis_y: 2            # ESP301 axis number for Y
  home_timeout_s: 60   # per-axis home timeout
  move_timeout_s: 30   # per-axis move timeout

Usage
-----
    from real_newport_motion import NewportESP301MotionController
    mc = NewportESP301MotionController(config)
    mc.home()
    mc.move_to(10.0, 25.0)
    mc.wait_for_settle(0.5)
    print(mc.get_position())
    mc.close()

Or as a context manager:
    with NewportESP301MotionController(config) as mc:
        mc.home()
        mc.move_to(10.0, 25.0)
        mc.wait_for_settle(0.5)
"""

import time
import logging

try:
    from pylablib.devices import Newport
except ImportError as exc:
    raise ImportError(
        "pylablib is required for NewportESP301MotionController. "
        "Install with: pip install pylablib"
    ) from exc

from motion_controller import MotionController

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Defaults — overridden by config["motion"] keys
# ---------------------------------------------------------------------------
_DEFAULT_BAUD = 19200
_DEFAULT_AXIS_X = 1
_DEFAULT_AXIS_Y = 2
_DEFAULT_HOME_TIMEOUT_S = 60.0
_DEFAULT_MOVE_TIMEOUT_S = 30.0


class NewportESP301MotionController(MotionController):
    """
    Real motion controller backend: Newport ESP301 via pylablib.

    Thread safety: NOT thread-safe. The scan loop is single-threaded
    (by design in scan_manager.py), so no locking is added here.
    """

    def __init__(self, config: dict):
        motion_cfg = config.get("motion", {})

        port = motion_cfg.get("port")
        if not port:
            raise ValueError(
                "motion.port must be set for newport_esp301 "
                "(e.g. 'COM3' or '/dev/ttyUSB0')"
            )

        baud = motion_cfg.get("baud", _DEFAULT_BAUD)
        self._axis_x = int(motion_cfg.get("axis_x", _DEFAULT_AXIS_X))
        self._axis_y = int(motion_cfg.get("axis_y", _DEFAULT_AXIS_Y))
        self._home_timeout = float(motion_cfg.get("home_timeout_s", _DEFAULT_HOME_TIMEOUT_S))
        self._move_timeout = float(motion_cfg.get("move_timeout_s", _DEFAULT_MOVE_TIMEOUT_S))

        self._homed = False
        self._position = (0.0, 0.0)  # last-commanded, updated on move_to

        logger.info(
            "Connecting to Newport ESP301 on %s at %d baud "
            "(X=axis%d, Y=axis%d)",
            port, baud, self._axis_x, self._axis_y,
        )

        # pylablib connection string for serial: "portname::baud"
        conn = f"{port}::{baud}"
        try:
            self._stage = Newport.ESP301(conn)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to open Newport ESP301 on {port}: {exc}"
            ) from exc

        logger.info("ESP301 connected. Firmware: %s", self._get_firmware_version())
        self._check_axis_configured(self._axis_x)
        self._check_axis_configured(self._axis_y)

    # ------------------------------------------------------------------
    # MotionController interface
    # ------------------------------------------------------------------

    def home(self):
        """
        Home both axes sequentially (X then Y).

        The ESP301 moves each axis to the home switch, then sets that
        position as the origin.  This call blocks until both axes have
        finished homing or raises RuntimeError on timeout.
        """
        logger.info("Homing X (axis %d)...", self._axis_x)
        self._stage.home(axis=self._axis_x, wait=False)
        self._wait_move(self._axis_x, self._home_timeout, label="Home X")

        logger.info("Homing Y (axis %d)...", self._axis_y)
        self._stage.home(axis=self._axis_y, wait=False)
        self._wait_move(self._axis_y, self._home_timeout, label="Home Y")

        self._homed = True
        self._position = (0.0, 0.0)
        logger.info("Homing complete. Position zeroed.")

    def move_to(self, x_mm: float, y_mm: float):
        """
        Command an absolute move to (x_mm, y_mm).

        Issues both axis moves simultaneously (non-blocking), then
        returns immediately.  Call wait_for_settle() to block until
        motion is complete and an optional dwell has elapsed.
        """
        if not self._homed:
            raise RuntimeError(
                "Must call home() before move_to(). "
                "The ESP301 has no absolute reference until homed."
            )

        logger.debug("move_to(%.4f, %.4f)", x_mm, y_mm)
        # Issue both moves without waiting so they run in parallel.
        self._stage.move_to(axis=self._axis_x, position=x_mm)
        self._stage.move_to(axis=self._axis_y, position=y_mm)
        self._position = (x_mm, y_mm)  # commanded position; actual may lag

    def get_position(self) -> tuple[float, float]:
        """
        Return the current encoder-reported position as (x_mm, y_mm).

        Falls back to last-commanded position if the query fails
        (e.g. communication glitch), logging a warning.
        """
        try:
            x = self._stage.get_position(axis=self._axis_x)
            y = self._stage.get_position(axis=self._axis_y)
            return (float(x), float(y))
        except Exception as exc:
            logger.warning(
                "get_position() query failed (%s); returning last-commanded %s",
                exc, self._position,
            )
            return self._position

    def wait_for_settle(self, settle_time_s: float):
        """
        Block until both axes have stopped moving, then sleep an
        additional settle_time_s for mechanical damping.

        Raises RuntimeError if either axis does not stop within
        move_timeout_s.
        """
        self._wait_move(self._axis_x, self._move_timeout, label="Wait X")
        self._wait_move(self._axis_y, self._move_timeout, label="Wait Y")

        if settle_time_s > 0:
            logger.debug("Settling for %.3f s", settle_time_s)
            time.sleep(settle_time_s)

        # Sanity-check for motor fault after every move.
        self._check_errors()

    # ------------------------------------------------------------------
    # Resource management
    # ------------------------------------------------------------------

    def close(self):
        """Release the serial connection to the ESP301."""
        try:
            self._stage.close()
            logger.info("ESP301 connection closed.")
        except Exception as exc:
            logger.warning("Error closing ESP301 connection: %s", exc)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def __del__(self):
        # Best-effort cleanup — don't raise in __del__.
        try:
            self.close()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _wait_move(self, axis: int, timeout_s: float, label: str = ""):
        """
        Poll until axis stops or timeout_s elapses.

        pylablib's wait_move() can hang if the controller sends an
        unexpected error mid-motion.  We wrap it with our own deadline
        so the scan loop always makes progress.
        """
        deadline = time.monotonic() + timeout_s
        try:
            # pylablib's built-in wait; raises on controller error
            self._stage.wait_move(axis=axis, timeout=timeout_s)
        except Exception as exc:
            # Distinguish timeout from genuine fault
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"[{label}] Axis {axis} did not stop within {timeout_s:.1f} s"
                ) from exc
            raise RuntimeError(
                f"[{label}] Axis {axis} motion error: {exc}"
            ) from exc

    def _check_errors(self):
        """
        Query the ESP301 error buffer and raise RuntimeError if any
        error is present.  The ESP301 error queue is FIFO; we drain it.
        """
        try:
            # `get_all_errors()` returns a list of (code, message) tuples.
            # Not all pylablib versions expose this; guard with hasattr.
            if hasattr(self._stage, "get_all_errors"):
                errors = self._stage.get_all_errors()
                if errors:
                    descriptions = "; ".join(f"{c}: {m}" for c, m in errors)
                    raise RuntimeError(f"ESP301 reported errors: {descriptions}")
        except RuntimeError:
            raise
        except Exception as exc:
            # Non-fatal: log and continue if error query itself fails.
            logger.warning("Could not query ESP301 error buffer: %s", exc)

    def _check_axis_configured(self, axis: int):
        """
        Verify that the axis is recognised by the controller
        (i.e., a motor stage is actually plugged in).
        """
        try:
            stage_id = self._stage.get_stage(axis=axis)
            logger.info("Axis %d stage ID: %s", axis, stage_id or "<unknown>")
        except Exception as exc:
            logger.warning(
                "Could not verify axis %d configuration: %s — "
                "check that the stage is connected and powered.",
                axis, exc,
            )

    def _get_firmware_version(self) -> str:
        try:
            return str(self._stage.get_device_info())
        except Exception:
            return "<unavailable>"


# ---------------------------------------------------------------------------
# Factory integration
# ---------------------------------------------------------------------------

def get_motion_controller(config: dict) -> MotionController:
    """
    Extended factory that handles 'newport_esp301' in addition to the
    mock path.  Drop this in as a replacement for the function of the
    same name in motion_controller.py, or call it from there.

    Config example (config.yaml):
        motion:
          controller: newport_esp301
          port: "COM3"
          axis_x: 1
          axis_y: 2
          home_timeout_s: 60
          move_timeout_s: 30
    """
    from motion_controller import MockMotionController  # local import avoids circular

    motion_cfg = config.get("motion", {})
    controller_type = motion_cfg.get("controller")

    if controller_type is None:
        logger.info("No controller configured — using MockMotionController")
        return MockMotionController()

    if controller_type == "newport_esp301":
        return NewportESP301MotionController(config)

    raise ValueError(
        f"Unknown motion controller type: '{controller_type}'. "
        "Supported: newport_esp301 | null (mock)"
    )


# ---------------------------------------------------------------------------
# Smoke test — run directly to verify connection without a full scan
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(description="Newport ESP301 connection smoke test")
    parser.add_argument("port", help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--axis-x", type=int, default=1)
    parser.add_argument("--axis-y", type=int, default=2)
    parser.add_argument("--move-x", type=float, default=5.0, help="Test move X mm")
    parser.add_argument("--move-y", type=float, default=5.0, help="Test move Y mm")
    args = parser.parse_args()

    cfg = {
        "motion": {
            "controller": "newport_esp301",
            "port": args.port,
            "axis_x": args.axis_x,
            "axis_y": args.axis_y,
            "home_timeout_s": 60,
            "move_timeout_s": 30,
        }
    }

    with NewportESP301MotionController(cfg) as mc:
        print("=== Homing ===")
        mc.home()
        print(f"Position after home: {mc.get_position()}")

        print(f"\n=== Moving to ({args.move_x}, {args.move_y}) ===")
        mc.move_to(args.move_x, args.move_y)
        mc.wait_for_settle(settle_time_s=0.2)
        print(f"Position after move: {mc.get_position()}")

        print("\n=== Returning to origin ===")
        mc.move_to(0.0, 0.0)
        mc.wait_for_settle(settle_time_s=0.2)
        print(f"Final position: {mc.get_position()}")

    print("\nSmoke test passed.")
