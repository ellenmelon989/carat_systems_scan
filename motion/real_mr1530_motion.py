"""
real_mr1530_motion.py

Optotune MR-15-30 (2-axis fast steering mirror) + MR-E-3 controller
backend for the CARAT scanner. Hardware: MR-15-30-G-25x25D mirror head +
MR-E-3 base unit, REPLACING the Newport CONEX-AGAP + AG-M100D mount (see
real_conexagap_motion.py, kept alongside this file for rollback -- not
deleted, same posture as real_newport_motion.py was kept after the 8742
was replaced by the CONEX-AGAP).

STATUS AS OF 2026-08-18 (v2): PARTIALLY FUNCTIONAL SCAFFOLD. Commanded
moves, settle-polling, and hard-homing are filled in from Optotune's
published "MR-E-3 Development Kit Operation Manual" (Simple Serial mode,
section 7). Reading back the ACTUAL (measured, closed-loop) X/Y position
is NOT filled in -- that requires either the "Pro mode" binary
register-read protocol (GOPRO/GOPROCRC), whose frame format is
undocumented in the operation manual and lives only in the separate
"MR-E-3 Firmware documentation" (not yet obtained -- see BLOCKING ON
below), or Optotune's official Python SDK. See the ARCHITECTURE DECISION
NEEDED section below -- this is a real fork in how this driver gets
finished, not just a missing detail.

SOURCE: Optotune "MR-E-3 Development Kit Operation Manual", Rev 1.0,
updated 2025-12-01 (https://www.optotune.com/product/mr-e-3/ -> Manual).
Everything under "CONFIRMED FROM THE MANUAL" below is read directly from
that document, section numbers noted where useful.

CONFIRMED FROM THE MANUAL
--------------------------
Serial framing (section 7, USB Type-C virtual COM port):
  baud rate: 256000 bps, 8 data bits, 1 stop bit, no parity.
  Line terminator: \\r\\n on every command.
  Max message size: 64 bytes. Commands are NOT case-sensitive.

Handshake: send "START" -> expect "OK\\r\\n" (Table 3).

Simple Serial commands used by this driver (Table 3):
  X=%f          set X coordinate, range -1.0 to 1.0 (normalized, see below)
  Y=%f          set Y coordinate, same range
  XY=%f;%f      set both in one command -- "drives channels X and Y to
                calibrated coordinates" (preferred over two single-axis
                sends: matches this project's existing "validate both,
                then send" pattern from the CONEX driver, and Optotune's
                own docs frame XY= as the intended two-axis move command)
  STATUS        returns the 32-bit status register in hex (Table 5).
                Bit 4 = "Mirror not stable" -- this is the settled/
                not-settled signal wait_for_settle() polls. Bits 5/6
                (output current limit reached / average limit reached)
                and bit 2 (mirror temperature threshold reached) are
                also worth surfacing as MotionFault causes, not silently
                ignored, if seen set during a move.
  GETTEMP       mirror temperature in degC -- useful for future thermal
                monitoring, not required for the base move/settle loop.
  GETID / GETVERSION / GETSN / GETDEVICESN / DETECTDEVICE
                identification queries, used at connect time the same
                way ConexAGAPController's __init__ confirms it's talking
                to the right device via ID? before trusting the port.

Serial response codes (Table 4), returned after most commands:
  OK      command accepted and performed within limits
  NO      command not accepted, for any reason
  OL      parameter reached LOWER limit (i.e. the firmware clamped it)
  OU      parameter reached UPPER limit (same, other direction)
  ERROR   command not available
  These are NOT the same shape as the CONEX's fire-and-forget PA command
  (no reply at all) -- every Simple Serial command here gets an explicit
  ack, so _send() below can and should check it, unlike the CONEX driver
  where a separate TS poll was the only way to know a command landed.

Position representation -- NOT plain degrees, read this carefully:
  X and Y are unitless, normalized to a UNIT CIRCLE (X^2 + Y^2 <= 1), NOT
  independent +/-25 deg ranges per axis. The manual gives the exact
  conversion: "the numerical values on the axes are ... defined by the
  maximum deflection of the mirror in optical angles, i.e., 50 deg ...
  a deflection of +50 deg corresponds to a value of +1." The formula is
      x = tan(gamma) / tan(50 deg)
  where gamma is the OPTICAL deflection angle in degrees (optical = 2x
  mechanical mirror tilt, standard fold-mirror doubling -- so the +/-25
  deg MECHANICAL spec sold in the quote corresponds to +/-50 deg OPTICAL,
  i.e. exactly the +/-1 normalized range). The firmware auto-clamps any
  commanded (X, Y) outside the unit circle to the nearest point ON the
  circle rather than rejecting it -- this driver validates and rejects
  BEFORE sending instead (see _mm_to_xy() below), matching this
  project's existing philosophy (ConexAGAPController never lets the
  firmware silently reinterpret an out-of-range target either) rather
  than relying on the firmware's own clamp-and-succeed behavior.

Homing: XY=0;0 drives to the mechanical/optical center of the unit
circle, which is a well-defined, always-reachable, non-hardware-specific
target (unlike the CONEX, whose PA[a] 0 was described as "the factory-
calibrated electrical center" of a specific unit's strain-gage range) --
this is what hard_home=True below sends.

ARCHITECTURE DECISION NEEDED -- reading ACTUAL position
----------------------------------------------------------
The manual's Simple Serial command table (Table 3) has no command that
reads back measured/actual position -- only X=/Y=/XY= (set commanded
position) and STATUS/GETTEMP (device state, not position). Reading the
actual closed-loop position requires "Pro mode" (GOPRO / GOPROCRC), a
binary register-addressed protocol the operation manual explicitly defers
to a SEPARATE document: "For full description of the MR-E-3 register map,
please refer to MR-E-3 Firmware documentation." That document (not this
operation manual) is what has the register addresses (the SPI section
mentions 0x2300/0x2301 as "Optical feedback read registers X, Y" over
SPI -- presumably the same or analogous registers are reachable over
Pro-mode serial, but the exact binary frame format for serial Pro mode is
not in the operation manual) and the binary frame layout for Pro mode.

This is a real fork, not just a missing detail -- two ways to close it:
  (a) Get the "MR-E-3 Firmware documentation" zip (Optotune publishes it
      alongside the firmware download) and hand-roll the Pro-mode binary
      protocol in pure pyserial, consistent with this project's existing
      "no vendor SDK" precedent (see real_conexagap_motion.py's module
      docstring -- that precedent was specifically about avoiding a
      Windows-only .NET-wrapped SDK, though, which doesn't describe
      Optotune's Python SDK below).
  (b) Use Optotune's own "MR-E Python SDK" (published alongside the
      firmware, pure Python per its changelog -- not a .NET wrapper like
      the CONEX-AGAP's rejected ConexAGAPCmdLib.dll), which almost
      certainly already implements Pro-mode register reads correctly.
      This would be a real dependency addition and a departure from this
      project's prior no-vendor-SDK pattern, but that pattern was a
      reaction to a specific bad option (Windows-only .NET), not a
      blanket rule -- worth deciding deliberately, not by default.
This decision should be made explicitly (see docs/mr1530_migration_plan.md)
before get_position()/get_position_deg()/zero_here()/calibration_jog_deg()
below are filled in. Tracking last-COMMANDED position as a stand-in is
deliberately NOT done here as a stopgap: given accuracy is only 0.15 deg
against 40 urad repeatability (~65x gap), substituting commanded position
for measured position would silently defeat the entire reason this
project calibrates against real closed-loop feedback in the first place.

COORDINATE SYSTEM / CONFIG KEYS
--------------------------------
Following the same deg_per_mm_x/y convention established for the CONEX-
AGAP driver (see its module docstring's CONFIG KEYS section) -- this is a
property of the mirror-to-target throw distance, not of a specific motor,
so the *pattern* carries over even though the calibrated *values* do not.
Internally this driver converts mm -> MECHANICAL degrees using
deg_per_mm_x/y (same as the CONEX driver), then mechanical degrees ->
OPTICAL degrees (x2) -> normalized X/Y via the tan() formula above, in
_mm_to_xy(). deg_per_mm_x/y themselves must still be measured on-site;
the CONEX's ~0.05 value is not a usable starting guess for this device's
different mount geometry.

Config keys (under motion:) -- DRAFT, see docs/mr1530_migration_plan.md:
  controller: mr1530
  port: "COMn"                  # Windows COM port once the MR-E-3's USB
                                 # driver is installed (Device Manager >
                                 # Ports, or --list-ports below).
  motion_enabled: false          # fail-closed operator interlock, same
                                  # pattern as both prior controllers
  axis_x / axis_y: not used -- the MR-E-3 addresses X/Y directly by name
                                 # (confirmed: Table 3's X=/Y=/XY= commands),
                                 # unlike the CONEX's U/V letter mapping.
                                 # invert_x/invert_y (below) is the only
                                 # wiring-direction knob needed.
  deg_per_mm_x: <no default>    # CALIBRATE ON-SITE -- __init__ refuses to
  deg_per_mm_y: <no default>    # start without these explicitly set.
  invert_x: false
  invert_y: false               # re-verify independently on real wiring
  hard_home: false              # true -> XY=0;0 (mechanical/optical
                                 # center, confirmed always valid). false
                                 # -> fiducial soft-home at current
                                 # position, same as CONEX -- BLOCKED until
                                 # get_position() is real (see decision
                                 # above), since soft-home needs to read
                                 # where the mirror currently is.
  move_timeout_s: 30            # placeholder, same default as CONEX driver
  soft_limits: {x_min_mm, x_max_mm, y_min_mm, y_max_mm}   # needs a real
                                 # value sized to the +/-25 deg mechanical
                                 # (+/-50 deg optical, unit-circle-bounded)
                                 # range and the real throw distance -- do
                                 # NOT carry over the CONEX's already-stale
                                 # +/-1000 mm 8742-era placeholder.

Usage (once the position-readback decision above is resolved)
----------------------------------------------------------------
    from real_mr1530_motion import MR1530Controller
    mc = MR1530Controller(config)
    mc.home()
    mc.move_to(2.0, -1.5)
    mc.wait_for_settle(0.2)
    print(mc.get_position())
    mc.close()
"""

from __future__ import annotations

import math
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
    # Fallback for running this file directly, where relative imports
    # don't work because there's no parent package.
    from motion_controller import MotionController, MotionFault, AxisStateUnknown

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Fixed by the MR-E-3 hardware (Operation Manual section 7) -- NOT
# configurable, so these are not read from config.yaml. Confirmed from
# Optotune's published operation manual, 2026-08-18.
# ---------------------------------------------------------------------------
_BAUD_RATE = 256_000
_BYTESIZE = serial.EIGHTBITS
_PARITY = serial.PARITY_NONE
_STOPBITS = serial.STOPBITS_ONE
_TERMINATOR = "\r\n"
_MAX_MESSAGE_BYTES = 64  # manual: "Maximum message size is 64 bytes"

# Optical deflection at the edge of the unit circle (X or Y = +/-1).
# Manual: "a deflection of +50 deg corresponds to a value of +1." Optical
# angle = 2x mechanical mirror tilt (standard fold-mirror doubling), so
# this matches the quoted +/-25 deg MECHANICAL spec.
_MAX_OPTICAL_DEG = 50.0
_MECH_TO_OPTICAL = 2.0

# Status register bits (Table 5) this driver cares about. Bit numbering
# per the manual; "latched"/"was" bits (8-13) mirror the live bits (0-6)
# but latch until ACKNOWLEDGE clears them -- not used by this driver yet.
_STATUS_BIT_MIRROR_NOT_STABLE = 4
_STATUS_BIT_MIRROR_TEMP_LIMIT = 2
_STATUS_BIT_CURRENT_LIMIT = 5
_STATUS_BIT_CURRENT_AVG_LIMIT = 6

# ---------------------------------------------------------------------------
# Defaults -- overridable in config.yaml under motion:.
# ---------------------------------------------------------------------------
_DEFAULT_MOVE_TIMEOUT = 30.0
_DEFAULT_SERIAL_TIMEOUT = 2.0
_DEFAULT_MOTION_ENABLED = False
_MOVE_POLL_S = 0.05
_SETTLE_CONFIRM_DELAY_S = 0.1
_UNIT_CIRCLE_TOLERANCE = 1e-9


class MR1530Controller(MotionController):
    """
    Real motion controller: Optotune MR-15-30 + MR-E-3 over a USB virtual
    COM port, Simple Serial mode.

    v2 status: home()/resume() (hard_home path), move_to(), and
    wait_for_settle() are implemented against the confirmed protocol.
    get_position(), get_position_deg(), zero_here(), and
    calibration_jog_deg() -- along with the fiducial (hard_home=False)
    home() path, which depends on reading current position -- raise
    NotImplementedError pending the position-readback architecture
    decision described in the module docstring.

    Thread safety: NOT thread-safe (scan loop is single-threaded), same
    as the other real controllers in this project.
    """

    def __init__(self, config: dict):
        motion_cfg = config.get("motion", {})

        port = motion_cfg.get("port")
        if not port:
            raise ValueError(
                "motion.port is required for controller: mr1530 "
                "(check Windows Device Manager > Ports (COM & LPT) once "
                "the MR-E-3's USB driver is installed)."
            )
        self._port = port

        # Unlike the CONEX driver, there is no safe placeholder value for
        # deg_per_mm here: the CONEX's ~0.05 default was a genuine
        # order-of-magnitude estimate for that specific benchtop mount
        # geometry, and this device's mount geometry/throw distance is
        # not yet established. Fail loudly rather than silently adopting
        # a wrong-by-construction number. These are MECHANICAL degrees of
        # mirror tilt per mm, same convention as the CONEX driver -- the
        # mechanical->optical doubling and the tan() unit-circle mapping
        # happen internally in _mm_to_xy(), not in this ratio.
        deg_per_mm_x = motion_cfg.get("deg_per_mm_x")
        deg_per_mm_y = motion_cfg.get("deg_per_mm_y")
        if deg_per_mm_x is None or deg_per_mm_y is None:
            raise ValueError(
                "motion.deg_per_mm_x / motion.deg_per_mm_y must be set "
                "explicitly for controller: mr1530 -- there is no safe "
                "default yet (unlike the CONEX-AGAP driver's placeholder, "
                "this device's mount geometry hasn't been characterized). "
                "Measure a real value on-site before using this driver for "
                "anything but hard-home connectivity checks."
            )
        self._deg_per_mm_x = float(deg_per_mm_x)
        self._deg_per_mm_y = float(deg_per_mm_y)

        self._invert_x = bool(motion_cfg.get("invert_x", False))
        self._invert_y = bool(motion_cfg.get("invert_y", False))
        self._sign_x = -1.0 if self._invert_x else 1.0
        self._sign_y = -1.0 if self._invert_y else 1.0
        self._eff_deg_per_mm_x = self._deg_per_mm_x * self._sign_x
        self._eff_deg_per_mm_y = self._deg_per_mm_y * self._sign_y

        self._hard_home = bool(motion_cfg.get("hard_home", False))
        self._motion_enabled = bool(
            motion_cfg.get("motion_enabled", _DEFAULT_MOTION_ENABLED)
        )
        self._move_timeout = float(motion_cfg.get("move_timeout_s", _DEFAULT_MOVE_TIMEOUT))
        serial_timeout = float(motion_cfg.get("serial_timeout_s", _DEFAULT_SERIAL_TIMEOUT))

        self._homed = False
        # Origin, in this driver's internal MECHANICAL-degree units, that
        # scan-grid (0, 0) maps to. Only meaningfully settable right now
        # via hard_home (always 0, 0 -- the unit-circle center); the
        # fiducial soft-home path needs get_position() to be real first.
        self._origin_x = 0.0
        self._origin_y = 0.0

        # Set BEFORE the connect attempt so close()/__del__ always have a
        # real attribute to check -- same reasoning as
        # ConexAGAPController.__init__.
        self._ser = None

        try:
            self._ser = serial.Serial(
                port=port,
                baudrate=_BAUD_RATE,
                bytesize=_BYTESIZE,
                parity=_PARITY,
                stopbits=_STOPBITS,
                timeout=serial_timeout,
                write_timeout=serial_timeout,
            )
        except Exception as exc:
            raise RuntimeError(
                f"Failed to open MR-E-3 serial port {port}: {exc}\n"
                "Check: (1) correct COM port (Device Manager > Ports)? "
                "(2) MR-E-3 USB driver installed? (3) controller powered "
                "on (24 VDC external supply) and not already claimed by "
                "another program (e.g. Optotune Cockpit)?"
            ) from exc

        logger.info("MR-E-3 serial port opened on %s.", port)

        try:
            self._handshake()
            device_id = self._query("GETID")
            logger.info("MR-E-3 identified: %s", device_id)
        except Exception as exc:
            self.close()
            raise RuntimeError(
                f"Connected to {port} but the device did not respond to "
                f"START/GETID as expected: {exc}\nIs this actually the "
                "MR-E-3's COM port (not e.g. a different instrument's, or "
                "a port already open in Optotune Cockpit)?"
            ) from exc

    # ------------------------------------------------------------------
    # MotionController interface
    # ------------------------------------------------------------------

    def home(self):
        """
        hard_home=True  -> commands XY=0;0, the unit circle's mechanical/
                          optical center -- always valid regardless of
                          this specific unit's calibration, unlike the
                          CONEX's PA[a] 0 (that unit's own factory-
                          calibrated electrical center). Anchors scan-grid
                          (0, 0) there.
        hard_home=False -> NOT YET IMPLEMENTED. Fiducial soft-homing (like
                          the CONEX's default) needs to read the mirror's
                          CURRENT position to anchor the origin there --
                          see the module docstring's ARCHITECTURE DECISION
                          NEEDED section. Raises until that's resolved.
        """
        if not self._hard_home:
            raise NotImplementedError(
                "hard_home=False (fiducial soft-home) requires reading "
                "current position, which this driver doesn't implement "
                "yet -- see the module docstring's position-readback "
                "architecture decision. Use hard_home=True for now."
            )
        self._require_motion_permission("hard home")
        logger.info("Hard-homing: driving to unit-circle center (XY=0;0)")
        self._send_xy(0.0, 0.0)
        self._wait_move(label="Home")
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._homed = True
        logger.info("Homing complete. Origin (mechanical deg): 0.0, 0.0")

    def resume(self):
        """Same hard_home-gated contract as ConexAGAPController.resume()."""
        if self._hard_home:
            self.home()
        else:
            self._homed = True
            logger.info(
                "Resuming (soft home) without re-zeroing: origin left at "
                "(%.5f, %.5f) mechanical deg.", self._origin_x, self._origin_y,
            )

    def zero_here(self):
        raise NotImplementedError(
            "zero_here() requires reading current position -- see the "
            "module docstring's position-readback architecture decision."
        )

    def move_to(self, x_mm: float, y_mm: float):
        if not self._homed:
            raise RuntimeError("Must call home() before move_to().")
        self._move_to_impl(x_mm, y_mm, label="move_to")

    def calibration_jog_deg(self, dx_deg: float = 0.0, dy_deg: float = 0.0):
        """Same contract as ConexAGAPController.calibration_jog_deg() --
        see that docstring. Blocked here for the same reason zero_here()
        and get_position_deg() are: it's a RELATIVE move from the CURRENT
        position, which requires reading that position first."""
        raise NotImplementedError(
            "calibration_jog_deg() requires reading current position -- "
            "see the module docstring's position-readback architecture "
            "decision."
        )

    def get_position_deg(self) -> tuple:
        raise NotImplementedError(
            "get_position_deg() requires reading ACTUAL (measured) "
            "position, which Simple Serial mode does not expose (only "
            "X=/Y=/XY= to SET commanded position) -- see the module "
            "docstring's position-readback architecture decision."
        )

    def get_absolute_position_deg(self) -> tuple:
        raise NotImplementedError(
            "get_absolute_position_deg() has the same blocker as "
            "get_position_deg() -- see the module docstring."
        )

    def get_position(self) -> tuple:
        raise NotImplementedError(
            "get_position() has the same blocker as get_position_deg() -- "
            "see the module docstring's position-readback architecture "
            "decision. Deliberately NOT stubbed with last-commanded "
            "position as a stand-in: given this device's accuracy "
            "(0.15 deg) is ~65x worse than its repeatability (40 urad), "
            "silently substituting commanded for measured position would "
            "defeat the reason this project calibrates against real "
            "closed-loop feedback at all."
        )

    def _move_to_impl(self, x_mm: float, y_mm: float, label: str):
        self._require_motion_permission(label)

        target_mech_x = self._origin_x + x_mm * self._eff_deg_per_mm_x
        target_mech_y = self._origin_y + y_mm * self._eff_deg_per_mm_y

        norm_x, norm_y = self._mm_to_xy(target_mech_x, target_mech_y)

        logger.debug(
            "move_to(%.4f mm, %.4f mm) -> mechanical deg (%.5f, %.5f) -> "
            "normalized XY (%.6f, %.6f)",
            x_mm, y_mm, target_mech_x, target_mech_y, norm_x, norm_y,
        )

        # Validate BOTH axes (the unit-circle constraint couples them --
        # X^2+Y^2<=1 is not two independent per-axis checks) before
        # sending anything, same "don't let one axis start moving on a
        # target that's about to be rejected" principle as
        # ConexAGAPController._move_to_impl(), just expressed as one
        # circular constraint instead of two independent SL/SR checks.
        self._validate_normalized_target(norm_x, norm_y)
        self._send_xy(norm_x, norm_y)

    def _mm_to_xy(self, mech_deg_x: float, mech_deg_y: float) -> tuple:
        """MECHANICAL degrees (this driver's internal unit, same
        convention as deg_per_mm_x/y) -> normalized XY per the manual's
        formula: optical_deg = 2 * mechanical_deg (fold-mirror doubling),
        x = tan(optical_deg) / tan(50 deg). Does NOT clamp to the unit
        circle -- see _validate_normalized_target(), which rejects
        out-of-circle targets instead of relying on the firmware's own
        silent clamp-to-nearest-edge behavior.
        """
        max_optical_rad = math.radians(_MAX_OPTICAL_DEG)
        tan_max = math.tan(max_optical_rad)

        optical_x = math.radians(mech_deg_x * _MECH_TO_OPTICAL)
        optical_y = math.radians(mech_deg_y * _MECH_TO_OPTICAL)

        return (math.tan(optical_x) / tan_max, math.tan(optical_y) / tan_max)

    def _validate_normalized_target(self, norm_x: float, norm_y: float):
        """Reject (not silently clamp) any target outside the unit
        circle X^2+Y^2<=1 that the manual documents as the mirror's full
        reachable range."""
        radius_sq = norm_x * norm_x + norm_y * norm_y
        if radius_sq > 1.0 + _UNIT_CIRCLE_TOLERANCE:
            raise MotionFault(
                f"MR-15-30 target blocked: normalized (X={norm_x:.6f}, "
                f"Y={norm_y:.6f}) has radius {math.sqrt(radius_sq):.6f} > 1.0 "
                "-- outside the mirror's reachable unit circle. No motion "
                "was commanded. (The firmware itself would silently clamp "
                "this to the nearest edge point rather than reject it -- "
                "this driver deliberately fails loud instead, matching "
                "this project's existing validate-before-send philosophy.)"
            )

    def wait_for_settle(self, settle_time_s: float):
        """Block until STATUS bit 4 ('Mirror not stable') clears on both
        axes, then sleep settle_time_s. Mirrors
        ConexAGAPController._wait_move()'s double-read confirmation and
        timeout escalation -- same MotionFault vs. AxisStateUnknown
        contract scan_manager.py relies on."""
        self._wait_move(label="Settle")
        if settle_time_s > 0:
            logger.debug("Settling %.3f s", settle_time_s)
            time.sleep(settle_time_s)

    def _require_motion_permission(self, operation: str):
        """Fail-closed interlock -- same pattern as both prior controllers."""
        if not self._motion_enabled:
            raise MotionFault(
                f"MR-15-30 {operation} blocked: set motion.motion_enabled: "
                "true only after the hardware position and optical path "
                "are safe."
            )

    # ------------------------------------------------------------------
    # Resource management
    # ------------------------------------------------------------------

    def close(self):
        if self._ser is None:
            return
        try:
            self._ser.close()
            logger.info("MR-15-30 connection closed.")
        except Exception as exc:
            logger.warning("Error closing MR-15-30 connection: %s", exc)
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
    # Internal helpers -- Simple Serial protocol (Operation Manual sec. 7)
    # ------------------------------------------------------------------

    def _handshake(self):
        """START -> OK, per Table 3. Called once at connect time."""
        resp = self._query("START")
        if resp.strip().upper() != "OK":
            raise RuntimeError(f"Unexpected response to START: {resp!r}")

    def _send(self, command: str) -> str:
        """Send a Simple Serial command and return its response code/text
        (OK/NO/OL/OU/ERROR, or a data reply for query-style commands like
        GETID/STATUS). Every Simple Serial command gets an explicit
        reply per the manual (unlike the CONEX's fire-and-forget PA) --
        so this always reads one line back."""
        line = command + _TERMINATOR
        if len(line.encode("ascii")) > _MAX_MESSAGE_BYTES:
            raise ValueError(
                f"Command {command!r} exceeds the MR-E-3's {_MAX_MESSAGE_BYTES}-"
                "byte max message size."
            )
        logger.debug("-> %s", command)
        self._ser.reset_input_buffer()
        self._ser.write(line.encode("ascii"))
        raw = self._ser.readline()
        if not raw:
            raise RuntimeError(
                f"No response from MR-E-3 to {command!r} "
                f"(timed out after {self._ser.timeout}s). Lost connection?"
            )
        resp = raw.decode("ascii", errors="replace").strip()
        logger.debug("<- %s", resp)
        return resp

    def _query(self, command: str) -> str:
        """Alias for _send() -- kept as a separate name for readability
        at call sites that are semantically "asking for data" (GETID,
        STATUS, GETTEMP) vs. "commanding an action" (X=, XY=), even
        though the wire behavior (write, read one line back) is
        identical for both in Simple Serial mode."""
        return self._send(command)

    def _send_xy(self, norm_x: float, norm_y: float):
        """XY=%f;%f -- set both axes in one command. Raises on NO/ERROR;
        logs (does not raise on) OL/OU, since those mean the firmware
        clamped rather than rejected -- but _validate_normalized_target()
        should have already caught anything that would trigger OL/OU
        before this is ever called, so seeing one here is itself worth a
        loud warning (means our own pre-validation missed a case the
        firmware caught)."""
        resp = self._send(f"XY={norm_x:.6f};{norm_y:.6f}")
        code = resp.strip().upper()
        if code == "OK":
            return
        if code in ("OL", "OU"):
            logger.warning(
                "MR-E-3 clamped XY=%.6f;%.6f (%s) despite pre-validation -- "
                "this shouldn't happen; check _validate_normalized_target() "
                "against the real firmware behavior.",
                norm_x, norm_y, code,
            )
            return
        raise MotionFault(f"MR-15-30 XY={norm_x:.6f};{norm_y:.6f} rejected: {resp!r}")

    def _get_status(self) -> int:
        """STATUS -> 32-bit register, hex string per the manual."""
        resp = self._query("STATUS")
        try:
            return int(resp, 16)
        except ValueError as exc:
            raise RuntimeError(f"Malformed STATUS response: {resp!r}") from exc

    def _wait_move(self, label: str = ""):
        """Block until STATUS bit 4 ('Mirror not stable') clears, or
        raise. Structurally mirrors ConexAGAPController._wait_move()'s
        double-read confirmation and MotionFault/AxisStateUnknown
        escalation -- see that method's docstring for the full rationale.
        There is no documented explicit stop command in Simple Serial
        mode (no equivalent to the CONEX's ST) -- if a timeout is hit
        here, this raises AxisStateUnknown directly rather than
        attempting a stop-then-confirm sequence, since there's nothing
        confirmed to send. This is a real gap worth closing once Pro
        mode or the SDK is in the picture (they may expose a real stop
        command) -- flagged, not silently assumed safe.
        """
        deadline = time.monotonic() + self._move_timeout
        while time.monotonic() < deadline:
            try:
                status = self._get_status()
            except Exception as exc:
                raise RuntimeError(
                    f"[{label}] Lost communication with MR-E-3 while waiting: {exc}"
                ) from exc

            if status & (1 << _STATUS_BIT_CURRENT_LIMIT) or status & (1 << _STATUS_BIT_CURRENT_AVG_LIMIT):
                raise MotionFault(
                    f"[{label}] MR-15-30 output current limit reached "
                    f"(STATUS=0x{status:08X}) -- motion may be incomplete."
                )
            if status & (1 << _STATUS_BIT_MIRROR_TEMP_LIMIT):
                raise MotionFault(
                    f"[{label}] MR-15-30 mirror temperature threshold "
                    f"reached (STATUS=0x{status:08X})."
                )

            if not (status & (1 << _STATUS_BIT_MIRROR_NOT_STABLE)):
                time.sleep(_SETTLE_CONFIRM_DELAY_S)
                try:
                    status2 = self._get_status()
                except Exception as exc:
                    raise RuntimeError(
                        f"[{label}] Lost communication with MR-E-3 while waiting: {exc}"
                    ) from exc
                if not (status2 & (1 << _STATUS_BIT_MIRROR_NOT_STABLE)):
                    return
                logger.debug(
                    "[%s] Reported stable then unstable again on confirm "
                    "read -- treating as still settling.", label,
                )
            time.sleep(_MOVE_POLL_S)

        raise AxisStateUnknown(
            f"[{label}] Mirror did not report stable (STATUS bit "
            f"{_STATUS_BIT_MIRROR_NOT_STABLE}) within {self._move_timeout:.1f} s, "
            "and Simple Serial mode has no documented stop command to fall "
            "back on. Axis state is unknown -- do not issue further moves "
            "without checking the hardware."
        )


def list_ports_verbose():
    """Print all serial ports currently visible to Windows, for finding
    which COMn is the MR-E-3 (Device Manager > Ports works too)."""
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        print(f"  {p.device}  --  {p.description}  (hwid: {p.hwid})")


# ---------------------------------------------------------------------------
# CLI: connection smoke test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(description="MR-15-30 connection smoke test")
    parser.add_argument("port", nargs="?", default=None,
                        help="COM port (e.g. COM6). Omit with --list-ports to just list ports.")
    parser.add_argument("--list-ports", action="store_true",
                        help="List available serial ports and exit.")
    parser.add_argument("--deg-per-mm-x", type=float, default=1.0,
                        help="Placeholder deg_per_mm_x for this smoke test only.")
    parser.add_argument("--deg-per-mm-y", type=float, default=1.0,
                        help="Placeholder deg_per_mm_y for this smoke test only.")
    parser.add_argument("--allow-motion", action="store_true",
                        help="Explicitly permit the hard-home move this smoke test performs.")
    args = parser.parse_args()

    if args.list_ports:
        list_ports_verbose()
        raise SystemExit(0)

    if not args.port:
        parser.error("port is required unless --list-ports is given")

    cfg = {
        "motion": {
            "controller": "mr1530",
            "port": args.port,
            "hard_home": True,   # the only home() path implemented so far
            "motion_enabled": args.allow_motion,
            "deg_per_mm_x": args.deg_per_mm_x,
            "deg_per_mm_y": args.deg_per_mm_y,
        }
    }

    with MR1530Controller(cfg) as mc:
        print("=== Hard-homing (XY=0;0) ===")
        mc.home()
        print("Homed. (get_position() is not yet implemented -- see module docstring.)")

    print("\nDone.")
