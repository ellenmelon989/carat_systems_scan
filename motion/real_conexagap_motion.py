"""
real_conexagap_motion.py

Newport CONEX-AGAP (Agilis-D, strain-gage feedback) controller backend
for the CARAT scanner.  Hardware: CONEX-AG-M100D controller + mirror
mount, replacing the Newport 8742 + 8816-6 picomotor setup (see
real_newport_motion.py, kept alongside this file — not deleted, in
case of rollback).

Communication: USB virtual COM port (standard serial, appears as
"COMn" in Windows Device Manager once the Newport USB driver — see
"USB Driver Installation Manual.pdf" — is installed). Fixed settings
per the CONEX-AGAP Controller Documentation (section 1.6): 921600 baud,
8 data bits, no parity, 1 stop bit, Xon/Xoff flow control, CRLF line
terminator. These are NOT configurable on the controller and are hard
coded below — no baud_rate config key, unlike a generic serial device.

Talks to the controller directly over pyserial with the raw ASCII
command set documented in "CONEX-AGAP - Controller Documentation.pdf"
section 2.4. This deliberately bypasses Newport's ConexAGAPCmdLib.dll
(.NET, Windows-only, referenced in the "Command Library API Manual")
and the LabVIEW samples — same "pyserial not PyMoDAQ/vendor SDK"
choice already made for other instruments in this project (see MEMORY
carat_scanner_hardware_status). The DLL/LabVIEW layers are thin
wrappers around the exact same ASCII-over-serial protocol implemented
here, so nothing is lost by talking to the port directly, and it keeps
this driver pure-Python / cross-platform-buildable like the rest of
the repo.

WHY THIS CONTROLLER IS NOT A DROP-IN FOR THE 8742
---------------------------------------------------
The 8742 + 8816-6 picomotor stack is OPEN LOOP: no position feedback,
"position" is just an internal step counter, and "homing" means
physically driving into a mechanical hard stop to get a repeatable
zero (see real_newport_motion.py's HOMING NOTE).

The CONEX-AGAP + AG-M100D is CLOSED LOOP: the mount has strain gages
that give a real, absolute-ish angular reading in DEGREES, read back
directly with the TP[a] command any time, even before the first move.
Consequences:
  - There is no mechanical-hard-stop homing command in the CONEX-AGAP
    command set at all (no OR-equivalent). "home()" below does NOT
    drive anywhere by default — see home()'s docstring for what
    hard_home actually means on this controller.
  - Position is read straight from hardware every time (get_position()
    is a live TP[a] query, not a locally-tracked step count), so it
    can't drift out of sync the way an open-loop step count can.
  - Travel is tiny compared to the picomotor: the CONEX-AGAP's own
    software limits (SL/SR) cap each axis to roughly +/-1 degree
    (see the SL[a]/SR[a] command docs — range is exactly -1..0 and
    0..1 degrees respectively). motion.soft_limits (in mm, at the
    plasma surface) must convert to a target well inside that, via
    steps_per_mm_x/y below — this is a much smaller absolute range
    than the picomotor's open-loop step space, so recheck
    soft_limits/steps_per_mm together on-site rather than assuming
    the old mm range still fits.

COORDINATE SYSTEM / AXIS NAMING
--------------------------------
The AG-M100D is a 2-axis mount, but CONEX-AGAP addresses its two axes
by LETTER — 'U' and 'V' — not by axis number the way the 8742 used
axis_x=1/axis_y=2. There is also only ONE controller address (both
axes live behind the same USB connection and the same RS-485 address,
factory default 1 — see the SA command docs: "only used when the
controller is configured for RS-485 communication", irrelevant for a
single unit on USB). config keys axis_x/axis_y here hold 'U' or 'V'
(NOT integers) to say which letter drives the scan-grid X and Y axes —
find this the same way axis_x/axis_y were found for the 8742: jog each
letter individually and watch which physical direction moves.

REUSED CONFIG KEY NAMES ON PURPOSE: steps_per_mm_x/y
------------------------------------------------------
Despite this controller's native unit being DEGREES, not motor steps,
this driver deliberately reads/writes the SAME config keys
(motion.steps_per_mm_x / motion.steps_per_mm_y) that
real_newport_motion.py uses for its steps-per-mm ratio, rather than
introducing e.g. deg_per_mm_x/y. That's because gui/calibration_panel.py
and scan/calibrate_scan_area.py (calibrate_steps_per_mm(),
recommend_home_steps(), the config-patching in
calibrate_scan_area.py's write_results()) hardcode those exact key
names and are otherwise fully unit-agnostic — they only ever do
"native_units = mm * steps_per_mm" arithmetic, never touch a motor
directly. Renaming the key would silently break the existing
calibration workflow (Calibrate tab, calibrate_scan_area.py CLI) for
this controller with no error, just wrong numbers. So: here,
steps_per_mm_x/y are really "degrees per mm at the plasma surface",
but the name in config.yaml stays steps_per_mm_x/y so the same
calibration tools work unmodified. See _eff_deg_per_mm_x/y below.

home_steps / home_velocity / home_timeout_s are 8742-specific (govern
driving into a mechanical hard stop) and are IGNORED by this driver —
harmless if still present in config.yaml, just unused.

HOMING / ORIGIN
----------------
No mechanical hard stop exists to home against, so hard_home means
something different here than for the 8742:

  hard_home: true  -> home() commands both axes to the controller's
                       own absolute zero (PA U 0 / PA V 0 -- i.e. the
                       factory-calibrated electrical center of the
                       strain-gage range) and anchors the scan-grid
                       origin there. This IS genuinely repeatable
                       across power cycles (closed-loop absolute
                       reading, not an open-loop counter), so unlike
                       the 8742 there's no crash-into-a-stop risk and
                       no home_steps/home_velocity to characterize.

  hard_home: false -> home() anchors the scan-grid origin at whatever
                       position the mount is CURRENTLY at (identical
                       to zero_here()) -- the fiducial-based workflow
                       already used for the 8742 (see MEMORY
                       carat_scanner_2026-07-17_scan_diagnosis) carries
                       over unchanged: calibrate_scan_area.py jogs to a
                       visually-confirmed reference mark, then calls
                       zero_here() to anchor there.

Either way, resume() follows the exact same hard_home-gated contract
as NewportPicomotorController.resume() (see scan_manager.py's
_safe_rehome(), which every controller must satisfy identically) --
hard_home=True: resume() == home() (idempotent). hard_home=False:
resume() must NOT re-zero, just mark ready.

Config keys (under motion:)
---------------------------
  controller: conex_agap
  port: "COM5"               # Windows COM port (Device Manager, once the
                              # Newport USB driver is installed) -- REQUIRED,
                              # no default; find via list_ports() below or
                              # Device Manager > Ports (COM & LPT).
  controller_address: 1      # CONEX-AGAP factory default over USB. Only
                              # change if it's been reconfigured for RS-485.
  motion_enabled: false      # explicit operator interlock; fail-closed
  calibration_confirmed: false # true only after measured CONEX deg/mm
  axis_x: "U"                # which CONEX axis letter ('U' or 'V') drives
  axis_y: "V"                # scan-grid X / Y -- CONFIRM by jogging, don't
                              # assume U=X.
  steps_per_mm_x: 500         # really "deg per mm" -- see module docstring.
  steps_per_mm_y: 500         # CALIBRATE ON-SITE (same workflow as before).
  invert_x: false
  invert_y: false             # same wiring/orientation-fact flag as the 8742
                               # driver, same reasoning for keeping it
                               # separate from steps_per_mm's own sign.
  hard_home: false            # see HOMING / ORIGIN above.
  move_timeout_s: 30          # per-axis move timeout (settle poll)
  stop_confirm_timeout_s: 10  # how long to wait for an explicit ST to land
  soft_limits: {x_min_mm, x_max_mm, y_min_mm, y_max_mm}   # unchanged

Usage
-----
    from real_conexagap_motion import ConexAGAPController
    mc = ConexAGAPController(config)
    mc.home()
    mc.move_to(2.0, -1.5)
    mc.wait_for_settle(0.2)
    print(mc.get_position())
    mc.close()
"""

# Deferred (string, non-evaluated) annotations -- REQUIRED for
# `-> tuple[float, float]` below (PEP 585 bare-generic subscripting) to
# not raise `TypeError: 'type' object is not subscriptable` at class-
# definition time on Python 3.8, the last version officially supported
# on Windows 7 (this box's OS -- see module docstring). Cheap, always-
# safe to include even on newer interpreters.
from __future__ import annotations

import time
import logging

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise ImportError(
        "pyserial is required. Install with: pip install pyserial"
    ) from exc

try:
    from .motion_controller import MotionController, MotionFault, AxisStateUnknown
except ImportError:
    # Fallback for running this file directly (e.g. python real_conexagap_motion.py),
    # where relative imports don't work because there's no parent package.
    from motion_controller import MotionController, MotionFault, AxisStateUnknown

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Fixed by the CONEX-AGAP hardware (Controller Documentation section 1.6) --
# NOT configurable, so these are not read from config.yaml.
# ---------------------------------------------------------------------------
_BAUD_RATE = 921_600
_BYTESIZE = serial.EIGHTBITS
_PARITY = serial.PARITY_NONE
_STOPBITS = serial.STOPBITS_ONE
_XONXOFF = True
_TERMINATOR = "\r\n"

# ---------------------------------------------------------------------------
# Defaults -- overridable in config.yaml under motion:
# ---------------------------------------------------------------------------
_DEFAULT_CONTROLLER_ADDRESS = 1
_DEFAULT_AXIS_X = "U"
_DEFAULT_AXIS_Y = "V"
_DEFAULT_STEPS_PER_MM = 500        # placeholder -- MUST calibrate on-site (deg/mm)
_DEFAULT_HARD_HOME = False
_DEFAULT_MOVE_TIMEOUT = 30.0
_DEFAULT_STOP_CONFIRM_TIMEOUT = 10.0
_DEFAULT_SERIAL_TIMEOUT = 2.0      # per-read timeout for the serial port itself
_DEFAULT_MOTION_ENABLED = False
_DEFAULT_CALIBRATION_CONFIRMED = False
_STOP_CONFIRM_POLL_S = 0.05
_MOVE_POLL_S = 0.05
_SETTLE_CONFIRM_DELAY_S = 0.1  # mirrors real_newport_motion.py's double-read
_LIMIT_POSITION_TOLERANCE_DEG = 0.005

# TS controller-state codes (last 2 chars of the TS reply) that mean the
# axis is actively in motion -- see Controller Documentation section 2.4,
# "TS -- Get positioner error and controller state".
_MOVING_STATES = {"28", "29", "46"}   # MOVING CL, STEPPING OL, JOGGING OL
_CONFIG_STATE = "14"
_DISABLE_STATES = {"3C", "3D"}
_READY_STATES = {"32", "33", "34", "35", "36"}
_STATE_NAMES = {
    "14": "CONFIGURATION",
    "28": "MOVING CL",
    "29": "STEPPING OL",
    "32": "READY from reset",
    "33": "READY from closed-loop move",
    "34": "READY from disable",
    "35": "READY from jog",
    "36": "READY from step",
    "3C": "DISABLE from ready",
    "3D": "DISABLE from closed-loop move",
    "46": "JOGGING OL",
}


def probe_conex_connection(port: str, address: int = _DEFAULT_CONTROLLER_ADDRESS,
                           serial_timeout: float = _DEFAULT_SERIAL_TIMEOUT,
                           serial_factory=None) -> dict:
    """Open the CONEX port and perform a no-motion communications check.

    The probe deliberately does not instantiate :class:`ConexAGAPController`:
    the normal controller refuses an out-of-limit encoder and is intended for
    later motion workflows, while a connection diagnostic must remain usable
    to describe a fault without altering controller state.  Only commands
    accepted in every state and incapable of commanding motion are used here:

    ``VE`` (firmware), ``ID?`` (stage identifier), ``TPU``/``TPV``
    (live strain-gage positions), ``THU``/``THV`` (targets), ``MM?``
    (controller state), and the read-only ``SL[a]?``/``SR[a]?`` software-limit
    queries.  The question mark is essential on SL/SR and MM: without it
    those commands set a value or alter state instead of reading one.

    ``serial_factory`` is an internal test seam.  Production callers should
    leave it as ``None`` so :class:`serial.Serial` is used.
    """
    if not port or not str(port).strip():
        raise ValueError("A CONEX COM port is required (for example, COM4).")

    address = int(address)
    if not 1 <= address <= 31:
        raise ValueError(f"CONEX controller address must be 1-31, got {address}.")

    timeout = float(serial_timeout)
    if timeout <= 0:
        raise ValueError(f"serial_timeout must be greater than zero, got {timeout}.")

    factory = serial_factory or serial.Serial
    ser = None

    def query(command: str) -> tuple[str, str]:
        ser.reset_input_buffer()
        ser.write((command + _TERMINATOR).encode("ascii"))
        ser.flush()
        raw = ser.readline()
        if not raw:
            raise RuntimeError(
                f"No response to {command!r} after {timeout:g} seconds."
            )

        response = raw.decode("ascii", errors="replace").strip()
        prefix = command[:-1] if command.endswith("?") else command
        if not response.upper().startswith(prefix.upper()):
            raise RuntimeError(
                f"Unexpected response {response!r} to {command!r}; expected "
                f"it to begin with {prefix!r}."
            )
        return response[len(prefix):].strip(), response

    try:
        ser = factory(
            port=str(port).strip(),
            baudrate=_BAUD_RATE,
            bytesize=_BYTESIZE,
            parity=_PARITY,
            stopbits=_STOPBITS,
            xonxoff=_XONXOFF,
            timeout=timeout,
            write_timeout=timeout,
        )

        # Give the Windows USB-serial driver a moment to finish opening the
        # virtual COM port before the first request.  This does not reset or
        # enable the controller.
        time.sleep(0.1)

        revision, raw_revision = query(f"{address}VE")
        stage_id, raw_stage_id = query(f"{address}ID?")
        position_u_text, raw_position_u = query(f"{address}TPU")
        position_v_text, raw_position_v = query(f"{address}TPV")
        target_u_text, raw_target_u = query(f"{address}THU")
        target_v_text, raw_target_v = query(f"{address}THV")
        state_text, raw_state = query(f"{address}MM?")
        negative_u_text, raw_negative_u = query(f"{address}SLU?")
        positive_u_text, raw_positive_u = query(f"{address}SRU?")
        negative_v_text, raw_negative_v = query(f"{address}SLV?")
        positive_v_text, raw_positive_v = query(f"{address}SRV?")

        try:
            position_u = float(position_u_text)
            position_v = float(position_v_text)
            target_u = float(target_u_text)
            target_v = float(target_v_text)
            negative_u = float(negative_u_text)
            positive_u = float(positive_u_text)
            negative_v = float(negative_v_text)
            positive_v = float(positive_v_text)
        except ValueError as exc:
            raise RuntimeError(
                "CONEX replied, but a position or software-limit value was "
                "not numeric: "
                f"position U={position_u_text!r}, V={position_v_text!r}; "
                f"target U={target_u_text!r}, V={target_v_text!r}; "
                f"limits U=({negative_u_text!r}, {positive_u_text!r}), "
                f"V=({negative_v_text!r}, {positive_v_text!r})."
            ) from exc

        return {
            "port": str(port).strip(),
            "address": address,
            "revision": revision,
            "stage_id": stage_id,
            "position_u_deg": position_u,
            "position_v_deg": position_v,
            "target_u_deg": target_u,
            "target_v_deg": target_v,
            "controller_state": state_text.upper(),
            "controller_state_name": _STATE_NAMES.get(
                state_text.upper(), "unknown state"
            ),
            "negative_limit_u_deg": negative_u,
            "positive_limit_u_deg": positive_u,
            "negative_limit_v_deg": negative_v,
            "positive_limit_v_deg": positive_v,
            "raw_responses": [
                raw_revision,
                raw_stage_id,
                raw_position_u,
                raw_position_v,
                raw_target_u,
                raw_target_v,
                raw_state,
                raw_negative_u,
                raw_positive_u,
                raw_negative_v,
                raw_positive_v,
            ],
        }
    except Exception as exc:
        if isinstance(exc, (ValueError, RuntimeError)):
            raise
        raise RuntimeError(
            f"Could not communicate with the CONEX on {port}: {exc}"
        ) from exc
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


class ConexAGAPController(MotionController):
    """
    Real motion controller: Newport CONEX-AGAP (Agilis-D, strain-gage
    feedback) over a USB virtual COM port.

    Position is read live from hardware in degrees (TP[a]) and
    converted to/from mm using steps_per_mm_x / steps_per_mm_y (really
    deg/mm here -- see module docstring).

    Thread safety: NOT thread-safe (scan loop is single-threaded), same
    as NewportPicomotorController.
    """

    def __init__(self, config: dict):
        motion_cfg = config.get("motion", {})

        port = motion_cfg.get("port")
        if not port:
            raise ValueError(
                "motion.port is required for controller: conex_agap "
                "(e.g. 'COM5' -- check Windows Device Manager > Ports "
                "(COM & LPT) once the Newport USB driver is installed)."
            )

        self._address = int(motion_cfg.get("controller_address", _DEFAULT_CONTROLLER_ADDRESS))

        self._axis_x = str(motion_cfg.get("axis_x", _DEFAULT_AXIS_X)).strip().upper()
        self._axis_y = str(motion_cfg.get("axis_y", _DEFAULT_AXIS_Y)).strip().upper()
        if self._axis_x not in ("U", "V") or self._axis_y not in ("U", "V"):
            raise ValueError(
                f"motion.axis_x/axis_y must each be 'U' or 'V' (got "
                f"{self._axis_x!r}/{self._axis_y!r})."
            )
        if self._axis_x == self._axis_y:
            raise ValueError(
                f"motion.axis_x and motion.axis_y must be different CONEX "
                f"axes, both are {self._axis_x!r}."
            )

        # Really deg/mm -- kept under the steps_per_mm_x/y config key names
        # for calibration-tool compatibility. See module docstring.
        self._steps_per_mm_x = float(motion_cfg.get("steps_per_mm_x", _DEFAULT_STEPS_PER_MM))
        self._steps_per_mm_y = float(motion_cfg.get("steps_per_mm_y", _DEFAULT_STEPS_PER_MM))

        self._invert_x = bool(motion_cfg.get("invert_x", False))
        self._invert_y = bool(motion_cfg.get("invert_y", False))
        self._eff_deg_per_mm_x = self._steps_per_mm_x * (-1.0 if self._invert_x else 1.0)
        self._eff_deg_per_mm_y = self._steps_per_mm_y * (-1.0 if self._invert_y else 1.0)

        self._hard_home = bool(motion_cfg.get("hard_home", _DEFAULT_HARD_HOME))
        # Two independent fail-closed interlocks.  motion_enabled is the
        # operator's deliberate permission to command hardware.  The separate
        # calibration_confirmed flag prevents an old 8742 steps/mm value (or
        # this driver's 500 placeholder) from becoming a CONEX degrees/mm
        # command merely because motion was enabled.
        self._motion_enabled = bool(
            motion_cfg.get("motion_enabled", _DEFAULT_MOTION_ENABLED)
        )
        self._calibration_confirmed = bool(
            motion_cfg.get(
                "calibration_confirmed", _DEFAULT_CALIBRATION_CONFIRMED
            )
        )
        self._move_timeout = float(motion_cfg.get("move_timeout_s", _DEFAULT_MOVE_TIMEOUT))
        self._stop_confirm_timeout = float(
            motion_cfg.get("stop_confirm_timeout_s", _DEFAULT_STOP_CONFIRM_TIMEOUT)
        )
        serial_timeout = float(motion_cfg.get("serial_timeout_s", _DEFAULT_SERIAL_TIMEOUT))

        self._homed = False
        # Origin, in the controller's own native degrees, that scan-grid
        # (0, 0) maps to. Set by home()/zero_here() -- see their docstrings
        # and the module-level HOMING / ORIGIN section.
        self._origin_u = 0.0
        self._origin_v = 0.0

        if self._steps_per_mm_x == _DEFAULT_STEPS_PER_MM:
            logger.warning(
                "steps_per_mm_x (deg/mm) is using the default placeholder "
                "value (%g). Calibrate on-site and update config.yaml.",
                _DEFAULT_STEPS_PER_MM,
            )
        if self._steps_per_mm_y == _DEFAULT_STEPS_PER_MM:
            logger.warning(
                "steps_per_mm_y (deg/mm) is using the default placeholder "
                "value (%g). Calibrate on-site and update config.yaml.",
                _DEFAULT_STEPS_PER_MM,
            )

        logger.info(
            "Connecting to CONEX-AGAP on %s (address %d, X=axis %s, Y=axis %s)",
            port, self._address, self._axis_x, self._axis_y,
        )

        # Set BEFORE the connect attempt so close()/__del__ always have a
        # real attribute to check -- same reasoning as
        # NewportPicomotorController.__init__ (see its comment): a failed
        # open() here must not leave a half-constructed object whose
        # __del__ raises a confusing secondary AttributeError that masks
        # the real connection error.
        self._ser = None

        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=_BAUD_RATE,
                bytesize=_BYTESIZE,
                parity=_PARITY,
                stopbits=_STOPBITS,
                xonxoff=_XONXOFF,
                timeout=serial_timeout,
                write_timeout=serial_timeout,
            )
        except Exception as exc:
            raise RuntimeError(
                f"Failed to open CONEX-AGAP serial port {port}: {exc}\n"
                "Check: (1) correct COM port (Device Manager > Ports)? "
                "(2) Newport USB driver installed (see 'USB Driver "
                "Installation Manual.pdf')? (3) controller powered on and "
                "not already claimed by another program (e.g. the Newport "
                "applet or a leftover previous connection)?"
            ) from exc

        logger.info("CONEX-AGAP connected.")

        # Confirm it's actually a CONEX-AGAP / AG-M100D and not a
        # mis-identified port -- fail loudly and immediately rather than
        # timing out mysteriously on the first real move.
        try:
            stage_id = self._query(f"{self._address}ID?")
            logger.info("Stage identifier: %s", stage_id)
        except Exception as exc:
            self.close()
            raise RuntimeError(
                f"Connected to {port} but the device did not answer an ID? "
                f"query as expected: {exc}\nIs this actually the CONEX-AGAP's "
                "COM port (not e.g. a different instrument's port)?"
            ) from exc

        # Construction must never change controller state.  In particular,
        # do not send MM1 here: GUI startup and diagnostics are not permission
        # to enable motion.  Also fail before a normal controller object can
        # be used if the encoder is already outside the stored SL/SR limits.
        try:
            self._assert_current_positions_within_limits("controller startup")
        except Exception:
            self.close()
            raise

    # ------------------------------------------------------------------
    # MotionController interface
    # ------------------------------------------------------------------

    def home(self):
        """
        Establish the scan-grid origin.

        hard_home=True  -> commands BOTH axes to the controller's own
                          absolute zero (PA[a] 0) and anchors scan-grid
                          (0, 0) there. Repeatable across power cycles
                          (closed-loop absolute reading) -- see module
                          docstring's HOMING / ORIGIN section for why
                          this is safe here in a way it isn't for the
                          open-loop 8742.
        hard_home=False -> anchors scan-grid (0, 0) at the CURRENT
                          physical position (identical to zero_here()).
                          Fiducial-based workflow, unchanged from the
                          8742 driver.
        """
        if self._hard_home:
            self._require_motion_permission("hard home", require_calibration=False)
            self._assert_current_positions_within_limits("hard home")
            self._validate_axis_target(self._axis_x, 0.0)
            self._validate_axis_target(self._axis_y, 0.0)
            self._ensure_enabled()
            logger.info("Hard-homing: driving both axes to controller zero (0 deg)")
            self._send(f"{self._address}PA{self._axis_x}0")
            self._send(f"{self._address}PA{self._axis_y}0")
            self._wait_move(label="Home")
            self._origin_u = 0.0
            self._origin_v = 0.0
        else:
            logger.info("Soft-homing: anchoring origin at current position")
            self._origin_u = self._get_axis_position(self._axis_x)
            self._origin_v = self._get_axis_position(self._axis_y)

        self._homed = True
        logger.info(
            "Homing complete. Origin (native units): %s=%.5f, %s=%.5f",
            self._axis_x, self._origin_u, self._axis_y, self._origin_v,
        )

    def resume(self):
        """
        Mark this controller ready to move WITHOUT re-homing -- same
        contract as NewportPicomotorController.resume() (see its
        docstring and scan_manager.py's _safe_rehome(), which every
        controller must satisfy identically).

        hard_home=True: just calls home() -- idempotent (always the
        same controller-zero target), so no real distinction from
        resuming.

        hard_home=False: does NOT re-read/re-zero the origin -- that
        would discard whatever origin a previous process (e.g.
        calibrate_scan_area.py) already established. Just confirms the
        axes are enabled and marks _homed True, leaving
        _origin_u/_origin_v untouched (they default to 0.0 at __init__,
        matching a fresh object that hasn't called home() itself --
        same caveat as the 8742 driver: this assumes the CALLER is
        responsible for knowing this process's origin should match the
        previous one, which is true for scan_manager.py's use of
        resume() specifically because it's a live re-read of physical
        position, not a locally-cached value -- so as long as the mount
        hasn't been bumped, get_position() after resume() reports
        correctly relative to whatever _origin_u/_origin_v this object
        has, and scan_manager only ever calls resume() when it means to
        keep using the ALREADY-established origin from earlier in the
        same physical session, mirroring the 8742 case).
        """
        if self._hard_home:
            self.home()
        else:
            self._homed = True
            logger.info(
                "Resuming (soft home) without re-zeroing: origin left at "
                "(%s=%.5f, %s=%.5f).",
                self._axis_x, self._origin_u, self._axis_y, self._origin_v,
            )

    def zero_here(self):
        """
        Zero BOTH axes at the stage's current physical position,
        unconditionally -- regardless of self._hard_home. Same fiducial-
        homing primitive as NewportPicomotorController.zero_here(); see
        its docstring and MEMORY carat_scanner_2026-07-17_scan_diagnosis
        for the rationale. Never commands a move.
        """
        logger.info("Zeroing at current position (fiducial reference)")
        self._origin_u = self._get_axis_position(self._axis_x)
        self._origin_v = self._get_axis_position(self._axis_y)
        self._homed = True
        logger.info(
            "Zeroed. Origin (native units): %s=%.5f, %s=%.5f",
            self._axis_x, self._origin_u, self._axis_y, self._origin_v,
        )

    def move_to(self, x_mm: float, y_mm: float):
        """
        Absolute move to (x_mm, y_mm) in scan-grid coordinates.

        Converts mm -> degrees using steps_per_mm_x/_y (and invert_x/
        invert_y), offsets by the homed origin, and issues both axis
        PA commands. Returns immediately; call wait_for_settle() to
        block.
        """
        if not self._homed:
            raise RuntimeError("Must call home() before move_to().")

        self._require_motion_permission("move_to", require_calibration=True)
        self._assert_current_positions_within_limits("move_to")

        target_u = self._origin_u + x_mm * self._eff_deg_per_mm_x
        target_v = self._origin_v + y_mm * self._eff_deg_per_mm_y

        logger.debug(
            "move_to(%.4f mm, %.4f mm) -> degrees (%s=%.5f, %s=%.5f)",
            x_mm, y_mm, self._axis_x, target_u, self._axis_y, target_v,
        )

        # Validate BOTH targets before sending EITHER command.  Otherwise an
        # invalid second-axis target could leave the first axis moving alone.
        self._validate_axis_target(self._axis_x, target_u)
        self._validate_axis_target(self._axis_y, target_v)
        self._ensure_enabled()
        self._send(f"{self._address}PA{self._axis_x}{target_u:.6f}")
        self._send(f"{self._address}PA{self._axis_y}{target_v:.6f}")

    def jog_axis_relative(self, axis_letter: str, delta_deg: float):
        """Guarded raw-axis direction test in native degrees.

        This is intentionally separate from scan-grid calibration.  It still
        requires explicit motion permission, an in-limit starting position,
        and an in-limit final target.  It cannot be used to recover an axis
        that is already outside SL/SR; use the dedicated recovery utility.
        """
        axis = str(axis_letter).strip().upper()
        if axis not in ("U", "V"):
            raise ValueError(f"axis must be 'U' or 'V', got {axis_letter!r}")
        delta = float(delta_deg)
        if delta == 0.0:
            raise ValueError("relative jog must be non-zero")

        self._require_motion_permission(
            f"relative jog on axis {axis}", require_calibration=False
        )
        self._assert_current_positions_within_limits(
            f"relative jog on axis {axis}"
        )
        target = self._get_axis_position(axis) + delta
        self._validate_axis_target(axis, target)
        self._ensure_enabled()
        self._send(f"{self._address}PR{axis}{delta:.6f}")
        self._wait_move(label=f"jog {axis}")

    def get_position(self) -> tuple[float, float]:
        """
        Return current position as (x_mm, y_mm), read LIVE from the
        strain-gage feedback (TP[a]) -- not a locally-tracked value, so
        unlike the 8742 this can't silently drift out of sync with
        reality.
        """
        try:
            u = self._get_axis_position(self._axis_x) - self._origin_u
            v = self._get_axis_position(self._axis_y) - self._origin_v
            return (u / self._eff_deg_per_mm_x, v / self._eff_deg_per_mm_y)
        except Exception as exc:
            logger.warning("get_position() failed: %s", exc)
            return (0.0, 0.0)

    def wait_for_settle(self, settle_time_s: float):
        """Block until both axes have stopped, then sleep settle_time_s."""
        self._wait_move(label="Settle")
        if settle_time_s > 0:
            logger.debug("Settling %.3f s", settle_time_s)
            time.sleep(settle_time_s)

    # ------------------------------------------------------------------
    # Resource management
    # ------------------------------------------------------------------

    def close(self):
        if self._ser is None:
            return
        try:
            self._ser.close()
            logger.info("CONEX-AGAP connection closed.")
        except Exception as exc:
            logger.warning("Error closing CONEX-AGAP connection: %s", exc)
        finally:
            self._ser = None

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
    # Internal helpers -- raw ASCII protocol
    # ------------------------------------------------------------------

    def _send(self, command: str):
        """
        Fire-and-forget an execute command (no response expected -- e.g.
        PA<value>, PR<value>, MM<value>, ST). Per the Controller
        Documentation, execute commands don't reply; check TE/TS
        afterward to confirm they were accepted.
        """
        line = command + _TERMINATOR
        logger.debug("-> %s", command)
        self._ser.write(line.encode("ascii"))

    def _query(self, command: str) -> str:
        """
        Send a query command (ends in '?', or TS/TE/TB/VE/ID) and return
        the response with the "{address}{command}" prefix stripped off,
        e.g. querying "1TPU" returns just the numeric string.

        Prefix is derived from the exact command string just sent
        (minus a trailing '?', if any) rather than by scanning the
        response for a run of leading alphabetic characters. An earlier
        version of this method did the latter and had a real bug: for
        "1ID?" the reply is "1IDAG-M100D" (stage identifier "AG-M100D"
        starts with letters too), so alpha-scanning ate into the value
        itself ("AG-M100D" -> mangled to "-M100D"). Anchoring the strip
        length to what was actually sent has no such ambiguity, since
        the controller always echoes address+command verbatim before
        the value (see the worked examples throughout the Controller
        Documentation, e.g. "1TS000032", "1TB@ No error").
        """
        line = command + _TERMINATOR
        logger.debug("-> %s", command)
        self._ser.reset_input_buffer()
        self._ser.write(line.encode("ascii"))
        raw = self._ser.readline()
        if not raw:
            raise RuntimeError(
                f"No response from CONEX-AGAP to {command!r} "
                f"(timed out after {self._ser.timeout}s). Lost connection?"
            )
        resp = raw.decode("ascii", errors="replace").strip()
        logger.debug("<- %s", resp)
        prefix = command[:-1] if command.endswith("?") else command
        if resp.upper().startswith(prefix.upper()):
            return resp[len(prefix):]
        # Unexpected echo shape (firmware quirk, or a controller-level
        # error string like "1TS@" style reply we didn't anticipate) --
        # don't guess further, hand back the full response so the
        # caller's own parsing (e.g. float()) fails loudly with the
        # actual text visible, rather than silently returning a
        # mis-sliced value.
        logger.warning(
            "Response %r to %r didn't start with expected prefix %r -- "
            "returning it unparsed.",
            resp, command, prefix,
        )
        return resp

    def _get_axis_position(self, axis_letter: str) -> float:
        """Live TP[a] query, in native degrees.

        TP is a get-only command on CONEX-AGAP.  Newport documents it as
        ``xxTP[a]`` (for example, ``1TPU``), without the question mark used
        by set/get parameter commands such as ID?.
        """
        value = self._query(f"{self._address}TP{axis_letter}")
        return float(value)

    def _get_axis_limits(self, axis_letter: str) -> tuple[float, float]:
        """Read the controller's stored negative/positive limits."""
        axis = str(axis_letter).strip().upper()
        negative = float(self._query(f"{self._address}SL{axis}?"))
        positive = float(self._query(f"{self._address}SR{axis}?"))
        if negative >= positive:
            raise RuntimeError(
                f"Malformed controller limits for axis {axis}: "
                f"{negative:.6f} to {positive:.6f} deg"
            )
        return negative, positive

    def _assert_current_positions_within_limits(self, context: str):
        """Fail closed if either live encoder lies outside stored SL/SR."""
        problems = []
        for axis in ("U", "V"):
            position = self._get_axis_position(axis)
            negative, positive = self._get_axis_limits(axis)
            if not (
                negative - _LIMIT_POSITION_TOLERANCE_DEG
                <= position
                <= positive + _LIMIT_POSITION_TOLERANCE_DEG
            ):
                problems.append(
                    f"{axis}={position:.6f} deg outside "
                    f"[{negative:.6f}, {positive:.6f}]"
                )
        if problems:
            raise MotionFault(
                f"CONEX motion blocked during {context}: "
                + "; ".join(problems)
                + ". Do not use PA/PR, the GUI, calibration, or scanning. "
                "Recover the axis with the dedicated open-loop recovery "
                "utility, then rerun the read-only CONEX diagnostic."
            )

    def _validate_axis_target(self, axis_letter: str, target_deg: float):
        """Reject an absolute target outside the live controller limits."""
        axis = str(axis_letter).strip().upper()
        target = float(target_deg)
        negative, positive = self._get_axis_limits(axis)
        if not negative <= target <= positive:
            raise MotionFault(
                f"CONEX target blocked: axis {axis} target {target:.6f} deg "
                f"is outside stored limits [{negative:.6f}, "
                f"{positive:.6f}] deg. No motion was commanded."
            )

    def _require_motion_permission(
        self, operation: str, require_calibration: bool
    ):
        """Require explicit config interlocks before any normal motion."""
        if not self._motion_enabled:
            raise MotionFault(
                f"CONEX {operation} blocked: set motion.motion_enabled: true "
                "only after the hardware position and optical path are safe."
            )
        if require_calibration and not self._calibration_confirmed:
            raise MotionFault(
                f"CONEX {operation} blocked: motion.calibration_confirmed "
                "is false. Replace the old 8742 steps_per_mm values with "
                "measured CONEX degrees/mm values before enabling scan moves."
            )

    def _get_state(self) -> str:
        """
        TS query -> last 2 chars (controller state code). See the
        _MOVING_STATES / _READY_STATES / _DISABLE_STATES / _CONFIG_STATE
        constants above for the meaning of each code.
        """
        resp = self._query(f"{self._address}TS")
        if len(resp) < 2:
            raise RuntimeError(f"Malformed TS response: {resp!r}")
        return resp[-2:].upper()

    def _ensure_enabled(self):
        """
        Make sure the controller is in a state that accepts PA/PR (READY
        or MOVING), enabling it (MM1) if it's currently DISABLE. Per the
        Controller Documentation, the controller powers up in READY OL
        by default, so this is normally a no-op -- it exists to recover
        from an operator having left it DISABLE (e.g. mid config-panel
        experiment) rather than to replace any homing step.
        """
        state = self._get_state()
        if state in _READY_STATES or state in _MOVING_STATES:
            return
        if state in _DISABLE_STATES:
            logger.info("Controller is DISABLE -- sending MM1 to enable")
            self._send(f"{self._address}MM1")
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                if self._get_state() in _READY_STATES:
                    return
                time.sleep(0.05)
            raise RuntimeError(
                "Controller did not reach READY state within 5s after MM1."
            )
        if state == _CONFIG_STATE:
            raise RuntimeError(
                "Controller is in CONFIGURATION state (PW1 was sent by "
                "something, e.g. the Newport applet, and never left with "
                "PW0). Move commands are rejected in this state. Resolve "
                "manually before scanning (send PW0, or power-cycle the "
                "controller) -- this driver deliberately does not send PW0 "
                "itself, since CONFIGURATION also gates writes to travel "
                "limits and other persisted settings that shouldn't be "
                "silently exited by a script."
            )
        raise RuntimeError(f"Unexpected controller state: {state!r}")

    def _wait_move(self, label: str = ""):
        """
        Block until BOTH axes are confirmed idle, or raise.

        Mirrors NewportPicomotorController._wait_move()'s double-read
        confirmation and stop-then-confirm timeout escalation (see that
        docstring for the full rationale) -- same MotionFault vs.
        AxisStateUnknown contract scan_manager.py already relies on,
        just driven off the single controller-wide TS state instead of
        a per-axis is_moving() (CONEX-AGAP moves both axes under one
        controller state, so one poll loop covers both).
        """
        deadline = time.monotonic() + self._move_timeout
        while time.monotonic() < deadline:
            try:
                state = self._get_state()
            except Exception as exc:
                raise RuntimeError(
                    f"[{label}] Lost communication with CONEX-AGAP while waiting: {exc}"
                ) from exc
            if state not in _MOVING_STATES:
                time.sleep(_SETTLE_CONFIRM_DELAY_S)
                try:
                    state2 = self._get_state()
                except Exception as exc:
                    raise RuntimeError(
                        f"[{label}] Lost communication with CONEX-AGAP while waiting: {exc}"
                    ) from exc
                if state2 not in _MOVING_STATES:
                    return
                logger.debug(
                    "[%s] Reported not-moving (%s) then moving again (%s) on "
                    "confirm read -- treating as still in motion.",
                    label, state, state2,
                )
            time.sleep(_MOVE_POLL_S)

        # Timed out waiting for the move itself. Command a stop on both
        # axes, then actively confirm it took effect before raising.
        try:
            self._send(f"{self._address}ST")
        except Exception as exc:
            raise AxisStateUnknown(
                f"[{label}] Motion did not stop within {self._move_timeout:.1f} s, "
                f"and the follow-up ST command itself failed ({exc}). Axis "
                "state is unknown -- do not issue further moves without "
                "checking the hardware."
            ) from exc

        stop_deadline = time.monotonic() + self._stop_confirm_timeout
        while time.monotonic() < stop_deadline:
            try:
                state = self._get_state()
            except Exception:
                # Comm hiccup while confirming the stop -- can't tell if
                # it's actually idle. Fall through to the unconfirmed-
                # state raise below rather than guessing.
                break
            if state not in _MOVING_STATES:
                raise MotionFault(
                    f"[{label}] Motion did not stop within {self._move_timeout:.1f} s "
                    f"(stop confirmed within {self._stop_confirm_timeout:.1f} s after "
                    "an explicit ST command)"
                )
            time.sleep(_STOP_CONFIRM_POLL_S)

        raise AxisStateUnknown(
            f"[{label}] Motion did not stop within {self._move_timeout:.1f} s, AND "
            f"did not confirm stopped within {self._stop_confirm_timeout:.1f} s "
            "after an explicit ST command. Axis state is unknown -- do not "
            "issue further moves without checking the hardware."
        )


def list_ports_verbose():
    """Print all serial ports currently visible to Windows, for finding
    which COMn is the CONEX-AGAP (Device Manager > Ports works too)."""
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"  {p.device}  --  {p.description}  (hwid: {p.hwid})")


# ---------------------------------------------------------------------------
# CLI: connection smoke test
#
# Note: the factory function that picks Mock vs. a real controller lives in
# motion_controller.py (get_motion_controller) -- that's the only copy.
# scan_manager.py and calibrate_scan_area.py both import from there.
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(
        description="CONEX-AGAP smoke test"
    )
    parser.add_argument("port", nargs="?", default=None,
                        help="COM port (e.g. COM5). Omit with --list-ports to just list ports.")
    parser.add_argument("--list-ports", action="store_true",
                        help="List available serial ports and exit.")
    parser.add_argument("--address", type=int, default=_DEFAULT_CONTROLLER_ADDRESS)
    parser.add_argument("--axis-x", default=_DEFAULT_AXIS_X, choices=["U", "V"])
    parser.add_argument("--axis-y", default=_DEFAULT_AXIS_Y, choices=["U", "V"])
    parser.add_argument("--jog-u", type=float, default=None,
                        help="Move axis U by this many degrees (relative) to test direction/wiring")
    parser.add_argument("--jog-v", type=float, default=None,
                        help="Move axis V by this many degrees (relative) to test direction/wiring")
    parser.add_argument(
        "--allow-motion", action="store_true",
        help="Explicitly permit the optional --jog-u/--jog-v command",
    )
    args = parser.parse_args()

    if args.list_ports:
        list_ports_verbose()
        raise SystemExit(0)

    if not args.port:
        parser.error("port is required unless --list-ports is given")

    cfg = {
        "motion": {
            "controller": "conex_agap",
            "port": args.port,
            "controller_address": args.address,
            "axis_x": args.axis_x,
            "axis_y": args.axis_y,
            "hard_home": False,   # soft home for smoke test -- safer
            "motion_enabled": args.allow_motion,
            "calibration_confirmed": False,
        }
    }

    with ConexAGAPController(cfg) as mc:
        print("=== Soft-homing at current position ===")
        mc.home()
        print(f"Position (mm, using placeholder deg/mm): {mc.get_position()}")

        # --jog-u/--jog-v always address the raw CONEX axis letter directly
        # (not scan-grid X/Y) -- this is a wiring/direction smoke test, run
        # before axis_x/axis_y in config.yaml are even decided.
        if args.jog_u is not None:
            print(f"\n=== Jogging axis U by {args.jog_u} deg (relative, raw PR) ===")
            mc.jog_axis_relative("U", args.jog_u)
            print(f"Raw TP: U={mc._get_axis_position('U'):.5f}  V={mc._get_axis_position('V'):.5f}")

        if args.jog_v is not None:
            print(f"\n=== Jogging axis V by {args.jog_v} deg (relative, raw PR) ===")
            mc.jog_axis_relative("V", args.jog_v)
            print(f"Raw TP: U={mc._get_axis_position('U'):.5f}  V={mc._get_axis_position('V'):.5f}")

    print("\nDone.")
