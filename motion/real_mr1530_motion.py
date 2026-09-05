"""
real_mr1530_motion.py

Optotune MR-15-30 (2-axis fast steering mirror) + MR-E-3 controller
backend for the CARAT scanner. Hardware: MR-15-30-G-25x25D mirror head +
MR-E-3 base unit, REPLACING the Newport CONEX-AGAP + AG-M100D mount (see
real_conexagap_motion.py, kept alongside this file for rollback -- not
deleted, same posture as real_newport_motion.py was kept after the 8742
was replaced by the CONEX-AGAP).

STATUS AS OF 2026-08-18 (v3): FUNCTIONALLY COMPLETE, UNTESTED ON REAL
HARDWARE. Every MotionController method is implemented against a protocol
now fully sourced from two places: Optotune's published "MR-E-3
Development Kit Operation Manual" (Simple Serial ASCII mode -- moves,
settle-status, handshake) and the actual source of Optotune's official
"MR-E Python SDK" (optomdc + optoKummenberg packages, read directly from
their .whl files, not imported as a dependency -- see POSITION READBACK
below for why). No physical MR-15-30 has touched this code yet (see
docs/mr1530_migration_plan.md) -- treat this as "ready to verify against
a fake-serial simulator," the same pre-hardware verification step used
for real_conexagap_motion.py, not as "ready to run against real hardware
unsupervised."

v4 (2026-09-05): wait_for_settle()'s timeout path now makes a
best-effort diagnostic read of the board-fault register before raising
AxisStateUnknown -- see the V4 UPDATE section further below and
_diagnose_timeout_fault(). This is diagnosis only; the decision to
freeze (raise, send no further motion) rather than auto-recover on a
timeout is unchanged and deliberate, since this firmware has no
stop/abort command to recover TO. Still functionally complete, still
untested against real hardware.

SIMPLE SERIAL MODE (moves, settle, handshake) -- from the Operation Manual
-----------------------------------------------------------------------------
  - Serial framing: 256000 baud, 8 data bits, 1 stop bit, no parity,
    "\\r\\n" terminator, 64-byte max message, commands case-insensitive.
  - Handshake: "START" -> "OK".
  - Move commands: "X=%f", "Y=%f", "XY=%f;%f" -- NOT plain degrees.
    Values are normalized to a UNIT CIRCLE (X^2+Y^2<=1): "a deflection of
    +50 deg [optical] corresponds to a value of +1," i.e.
        x = tan(optical_deg) / tan(50 deg)
    and optical deflection is 2x mechanical mirror tilt (standard
    fold-mirror doubling), so the quoted +/-25 deg mechanical spec is
    exactly the +/-1 normalized range. The firmware auto-clamps an
    out-of-circle target to the nearest edge point rather than rejecting
    it -- this driver validates and rejects BEFORE sending instead
    (_validate_normalized_target()), matching this project's existing
    fail-loud philosophy (ConexAGAPController never lets the firmware
    silently reinterpret an out-of-range target either).
  - Response codes: OK / NO / OL (lower-limit clamp) / OU (upper-limit
    clamp) / ERROR -- every Simple Serial command gets an explicit reply,
    unlike the CONEX's fire-and-forget PA.
  - Settled status: "STATUS" returns a 32-bit hex register; bit 4 =
    "Mirror not stable." This is what wait_for_settle() polls -- NOT the
    same thing as register 0x1007 (see POSITION READBACK below).
  - Homing: "XY=0;0" drives to the unit circle's mechanical/optical
    center, always valid regardless of unit-specific calibration (unlike
    the CONEX's factory-calibrated electrical zero). hard_home=True's
    target.

POSITION READBACK (get_position(), zero_here(), calibration_jog_deg()) --
sourced from the MR-E Python SDK's ACTUAL SOURCE, not the operation manual
------------------------------------------------------------------------------
The Operation Manual's Simple Serial command table has NO command that
reads back measured position -- only X=/Y=/XY= to SET commanded position.
Reading ACTUAL (closed-loop measured) position requires "Pro mode"
(binary, register-addressed), whose frame format the Operation Manual
explicitly defers to a separate "MR-E-3 Firmware documentation" file this
session did not have. What this session DID get: Optotune's official
"MR-E Python SDK" download, which ships as two pure-Python wheels
(optomdc, optokummenberg -- no compiled extensions, no .NET wrapper,
unlike the CONEX-AGAP's rejected ConexAGAPCmdLib.dll). Rather than adding
that SDK as a runtime dependency -- which would be a real, deliberate
departure from this project's "no vendor SDK" precedent (every prior
motion backend here is hand-rolled pyserial: real_newport_motion.py
avoided pylablib, real_conexagap_motion.py avoided the CONEX .NET DLL) --
this driver's Pro-mode implementation below was written by READING the
SDK's source directly (unzipped from optomdc-*.whl / optokummenberg-*.whl)
and porting the exact protocol logic into plain pyserial. Zero new
dependency; correctness sourced from the vendor's own tested
implementation instead of guessed from a spec document.

Confirmed from optokummenberg/tools/parsing_tools.py's encode()/decode()
and optokummenberg/commands.py:
  - Frame: FRAME_BOUNDARY (0x7E, "~") + slave_addr(0x00) + command_id(1B)
    + payload_size(1B) + payload + crc(2B) + FRAME_BOUNDARY. Byte-stuffed:
    any 0x7D or 0x7E byte INSIDE the frame (not the delimiters) is
    replaced by 0x7D followed by (byte XOR 0x20).
  - CRC is NOT actually computed by the SDK's default encode() path (the
    `CommandID.CRC_ENABLED` flag it checks for doesn't exist on the class,
    so the check always fails) -- it always sends 0x00 0x00. This matches
    "GOPRO" (not "GOPROCRC") mode from the Operation Manual, which is what
    this driver uses.
  - Command IDs used here: GET_VALUE=0x11 (read a register, big-endian
    float32 payload back), SET_COMM_MODE=0x06 (switch back to Simple mode
    -- register_id field doubles as the target mode, 0=simple).
  - Mode switching is bidirectional and cheap: ASCII "GOPRO" (Simple mode
    command) enters Pro mode; a Pro-mode SET_COMM_MODE(0) frame followed
    by a fresh ASCII "START"/"OK" handshake confirms return to Simple
    mode (this exactly mirrors optoKummenberg.commands.Command.go_simple()
    / go_pro()). This driver stays in Simple mode for everything except
    the brief round-trip needed to read actual position, rather than
    living in Pro mode permanently -- keeps the already-verified Simple
    Serial move/settle logic untouched.
  - Register addresses, from optomdc/registers/mre3_registers.py's
    RadialBasisFunction class docstring (System ID 0x3B): 0x3B00 =
    "Mirror coordinate X" (read only, float, "Mirror unary circle X
    coordinate"), 0x3B01 = same for Y. These are in the SAME normalized
    unit-circle coordinate system as the X=/Y=/XY= Simple Serial commands
    (confirmed by the SDK's own example script,
    MR-E-3_ReadBackSignalFlowValues.py, which reads the equivalent
    feedback-path signal and labels it "Position Values (XY when in XY
    mode)") -- so no separate calibration/scaling is needed between the
    two; the same _mm_to_xy()/_xy_to_mm() conversion in this file applies
    to both.
  - Register 0x1007 ("System status errors," from optomdc/registers/
    mre3_registers.py's MRE3Status class) is a DIFFERENT, board-level
    fault register (channel output faults, over-heat, device-not-detected,
    over-current) -- NOT a mirror-settled bit. The email thread's mention
    of "register 0x1007" for settled-status was a mix-up; this driver
    uses the Simple Serial STATUS command's bit 4 for settling instead,
    confirmed directly against the Operation Manual. As of v4 (see
    below), 0x1007 IS read, but only as a best-effort diagnostic on a
    wait_for_settle() timeout -- see _diagnose_timeout_fault().
  - No stop/abort command exists in Simple OR Pro mode (confirmed absent
    from both the Operation Manual's Table 3 and every method in the
    SDK's Command class) -- wait_for_settle()'s timeout path has nothing
    to escalate to, unlike the CONEX's ST command. This is a genuine
    hardware/firmware limitation, not a gap in this driver.

V4 UPDATE (2026-09-05) -- settle-timeout diagnosis, decision on the
missing stop/abort command
-----------------------------------------------------------------------------
Given there is no stop/abort command anywhere in this firmware, the only
two real options on a wait_for_settle() timeout are (a) freeze -- raise,
send no further commands, require a human to check the hardware before
anything moves again -- or (b) auto-recover by commanding the mirror
somewhere "safe" (e.g. XY=0;0) on the assumption the timeout was benign.
Decision: (a), unchanged from v3. Auto-recovering with (b) means issuing
a new motion command on top of an axis whose real state is unconfirmed
-- exactly the failure mode AxisStateUnknown's contract in
motion_controller.py exists to prevent (see that class's docstring), and
"safe" is optics-dependent in a way this driver has no basis to assert.
What v4 DOES add is diagnosis, not recovery: _wait_move()'s timeout path
now makes a best-effort Pro-mode read of the board-fault register
(_REG_SYSTEM_STATUS_ERRORS, 0x1007) via _diagnose_timeout_fault() before
raising, and folds the raw value into both the log and the raised
AxisStateUnknown's message -- so an operator sees more than "it didn't
report stable." This is intentionally NOT bit-level fault decoding: the
exact field layout of 0x1007 was never confirmed against real
documentation (only the register's existence and general category came
from reading the MRE3Status class name while researching position
readback -- see the class docstring above) -- the raw hex value is
surfaced for a human to cross-reference against Optotune's own docs, not
decoded into specific flags this driver would then have to guess the
meaning of. If the diagnostic read itself fails (e.g. the same comm
fault that broke settling also breaks Pro mode), that failure is logged
and folded into the exception message too, rather than silently
swallowed -- a failed diagnostic read is itself informative (probably a
dead link, not just a stuck mirror) and must never be treated as "no
fault found." Still untested against real hardware.

COORDINATE SYSTEM / CONFIG KEYS
--------------------------------
Same deg_per_mm_x/y convention as the CONEX-AGAP driver: MECHANICAL
degrees of mirror tilt per mm of spot travel, calibrated on-site (no safe
default exists for this device -- __init__ refuses to start without it).
Internally converted mm -> mechanical degrees -> optical degrees (x2) ->
normalized XY via the tan() formula, and back, in _mm_to_xy()/_xy_to_mm().

Config keys (under motion:) -- see docs/mr1530_migration_plan.md:
  controller: mr1530
  port: "COMn"
  motion_enabled: false          # fail-closed operator interlock
  deg_per_mm_x / deg_per_mm_y: <no default, must calibrate on-site>
  invert_x / invert_y: false     # re-verify on real wiring
                                  # (axis_x/axis_y are NOT needed -- the
                                  # MR-E-3 addresses X/Y directly, unlike
                                  # the CONEX's U/V letters)
  hard_home: true|false          # true -> XY=0;0 (unit-circle center).
                                  # false -> fiducial soft-home at current
                                  # ACTUAL position (now implemented, v3).
  move_timeout_s: 30
  soft_limits: {x_min_mm, x_max_mm, y_min_mm, y_max_mm}   # needs a real
                                  # value; the device's own reachable
                                  # range is the unit circle X^2+Y^2<=1
                                  # (not two independent per-axis limits).

Usage
-----
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
import struct
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
# Fixed by the MR-E-3 hardware (Operation Manual section 7) -- confirmed
# 2026-08-18.
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

# Status register bits (Operation Manual Table 5) this driver cares about.
_STATUS_BIT_MIRROR_NOT_STABLE = 4
_STATUS_BIT_MIRROR_TEMP_LIMIT = 2
_STATUS_BIT_CURRENT_LIMIT = 5
_STATUS_BIT_CURRENT_AVG_LIMIT = 6

# ---------------------------------------------------------------------------
# Pro-mode binary protocol constants -- sourced from optoKummenberg's
# tools/definitions.py and tools/parsing_tools.py (SDK source, not a
# runtime dependency -- see module docstring's POSITION READBACK section).
# ---------------------------------------------------------------------------
_FRAME_BOUNDARY = 0x7E
_ESCAPE_BYTE = 0x7D
_ESCAPE_MASK = 0x20
_PRO_CMD_GET_VALUE = 0x11
_PRO_CMD_SET_COMM_MODE = 0x06
_PRO_SLAVE_ADDRESS = 0x00
_PRO_ERROR_FLAG = 0x80  # response command_id == (sent | 0x80) means error

# RadialBasisFunction system (sys_id 0x3B) registers -- read-only floats,
# "Mirror unary circle X/Y coordinate," SAME normalized unit-circle
# coordinate system as the Simple Serial X=/Y=/XY= commands.
_REG_MIRROR_COORD_X = 0x3B00
_REG_MIRROR_COORD_Y = 0x3B01

# Board-level fault/status register (from optomdc/registers/
# mre3_registers.py's MRE3Status class -- same source as the two
# registers above) -- distinct from the Simple Serial STATUS command's
# settle bit. Read ONLY as a best-effort diagnostic on a
# wait_for_settle() timeout (see _diagnose_timeout_fault()); this driver
# does not otherwise touch it. Its exact per-bit field layout was never
# confirmed against real documentation -- only the register's existence
# and general category (channel faults, over-heat, device-not-detected,
# over-current) came from reading the MRE3Status class name/docstring
# while researching position readback. Treat the raw value as a
# diagnostic clue for a human to cross-reference against Optotune's
# docs, not as a decoded flag this driver can act on with confidence.
_REG_SYSTEM_STATUS_ERRORS = 0x1007

_PRO_FRAME_TIMEOUT_S = 2.0

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
    COM port. Lives in Simple Serial mode for moves/settle/handshake;
    briefly enters Pro mode (and back) only to read actual position.

    v3 status: every MotionController method is implemented. UNTESTED
    against real hardware -- see module docstring's opening paragraph.

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

        # No safe placeholder value for deg_per_mm exists yet for this
        # device (unlike the CONEX driver's ~0.05 order-of-magnitude
        # guess) -- fail loudly rather than silently adopting a
        # wrong-by-construction number. MECHANICAL degrees of mirror tilt
        # per mm, same convention as the CONEX driver -- the
        # mechanical->optical doubling and unit-circle mapping happen
        # internally in _mm_to_xy()/_xy_to_mm(), not in this ratio.
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
        # Origin, in MECHANICAL degrees (this driver's internal unit,
        # same convention as deg_per_mm_x/y), that scan-grid (0, 0) maps
        # to. Set by home()/zero_here().
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
                          CONEX's factory-calibrated electrical zero.
        hard_home=False -> fiducial soft-home: anchors scan-grid (0, 0)
                          at the CURRENT actual position (v3: implemented,
                          via the Pro-mode position readback -- identical
                          to zero_here()).
        """
        if self._hard_home:
            self._require_motion_permission("hard home")
            logger.info("Hard-homing: driving to unit-circle center (XY=0;0)")
            self._send_xy(0.0, 0.0)
            self._wait_move(label="Home")
            self._origin_x = 0.0
            self._origin_y = 0.0
            self._homed = True
            logger.info("Homing complete. Origin (mechanical deg): 0.0, 0.0")
        else:
            self.zero_here()

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
        """
        Zero at the stage's current ACTUAL position (read via a brief
        Pro-mode round-trip), unconditionally -- same fiducial-homing
        primitive as ConexAGAPController.zero_here().
        """
        logger.info("Zeroing at current position (fiducial reference)")
        mech_x, mech_y = self._read_actual_mechanical_deg()
        self._origin_x = mech_x
        self._origin_y = mech_y
        self._homed = True
        logger.info(
            "Zeroed. Origin (mechanical deg): %.5f, %.5f", mech_x, mech_y,
        )

    def move_to(self, x_mm: float, y_mm: float):
        if not self._homed:
            raise RuntimeError("Must call home() before move_to().")
        self._move_to_impl(x_mm, y_mm, label="move_to")

    def calibration_jog(self, dx_mm: float = 0.0, dy_mm: float = 0.0):
        """Same contract/rationale as ConexAGAPController.calibration_jog()
        -- relative move by mm, for use once deg_per_mm_x/y are at least
        roughly known. Prefer calibration_jog_deg() for a fresh setup."""
        if not self._homed:
            raise RuntimeError("Must call home() before calibration_jog().")
        x, y = self.get_position()
        self._move_to_impl(x + dx_mm, y + dy_mm, label="calibration jog")
        self.wait_for_settle(0.0)

    def calibration_jog_deg(self, dx_deg: float = 0.0, dy_deg: float = 0.0):
        """
        Relative move by (dx_deg, dy_deg) in MECHANICAL degrees, reading
        the CURRENT actual position first (via Pro mode) so this is a
        real relative move from wherever the mirror is right now -- same
        contract as ConexAGAPController.calibration_jog_deg(), and the
        same bugfix lesson applies (see that method's docstring on
        carat_scanner_2026-08-06_calibration_jog_absolute_not_relative):
        this MUST read live position, not compute from a cached origin,
        or repeated jogs in one direction silently become no-ops.
        """
        if not self._homed:
            raise RuntimeError("Must call home() before calibration_jog_deg().")
        self._require_motion_permission("calibration jog (deg)")
        current_x, current_y = self._read_actual_mechanical_deg()
        target_x = current_x + dx_deg * self._sign_x
        target_y = current_y + dy_deg * self._sign_y
        norm_x, norm_y = self._mm_to_xy_from_mech_deg(target_x, target_y)
        self._validate_normalized_target(norm_x, norm_y)
        self._send_xy(norm_x, norm_y)
        self._wait_move(label="calibration jog (deg)")

    def get_position_deg(self) -> tuple:
        """Live ACTUAL position in scan-grid-semantic mechanical degrees,
        relative to origin -- the counterpart to calibration_jog_deg().
        Same axis convention as get_position(), but without dividing by
        deg_per_mm_x/y."""
        mech_x, mech_y = self._read_actual_mechanical_deg()
        u = mech_x - self._origin_x
        v = mech_y - self._origin_y
        return (u * self._sign_x, v * self._sign_y)

    def get_absolute_position_deg(self) -> tuple:
        """Live ACTUAL position in RAW mechanical degrees, WITHOUT
        subtracting the origin and WITHOUT the invert_x/invert_y sign
        flip -- same rationale as ConexAGAPController's version: lets an
        operator compare directly against the device's own raw range
        independent of wherever zero_here()/home() anchored the origin."""
        return self._read_actual_mechanical_deg()

    def get_position(self) -> tuple:
        """
        Return current ACTUAL position as (x_mm, y_mm), read live via a
        brief Pro-mode round-trip (RadialBasisFunction registers
        0x3B00/0x3B01) -- not a locally-tracked commanded value. See the
        module docstring's POSITION READBACK section for exactly how this
        is sourced and why last-commanded position was deliberately never
        used as a stand-in: this device's accuracy (0.15 deg) is ~65x
        worse than its repeatability (40 urad), so substituting commanded
        for measured position would defeat the reason this project
        calibrates against real closed-loop feedback at all.
        """
        try:
            mech_x, mech_y = self._read_actual_mechanical_deg()
            u = mech_x - self._origin_x
            v = mech_y - self._origin_y
            return (u / self._eff_deg_per_mm_x, v / self._eff_deg_per_mm_y)
        except Exception as exc:
            logger.warning("get_position() failed: %s", exc)
            return (0.0, 0.0)

    def _move_to_impl(self, x_mm: float, y_mm: float, label: str):
        self._require_motion_permission(label)

        target_mech_x = self._origin_x + x_mm * self._eff_deg_per_mm_x
        target_mech_y = self._origin_y + y_mm * self._eff_deg_per_mm_y

        norm_x, norm_y = self._mm_to_xy_from_mech_deg(target_mech_x, target_mech_y)

        logger.debug(
            "move_to(%.4f mm, %.4f mm) -> mechanical deg (%.5f, %.5f) -> "
            "normalized XY (%.6f, %.6f)",
            x_mm, y_mm, target_mech_x, target_mech_y, norm_x, norm_y,
        )

        # Validate BOTH axes (the unit-circle constraint couples them --
        # X^2+Y^2<=1 is not two independent per-axis checks) before
        # sending anything, same "don't let one axis start moving on a
        # target that's about to be rejected" principle as
        # ConexAGAPController._move_to_impl().
        self._validate_normalized_target(norm_x, norm_y)
        self._send_xy(norm_x, norm_y)

    def wait_for_settle(self, settle_time_s: float):
        """Block until STATUS bit 4 ('Mirror not stable') clears, then
        sleep settle_time_s. Mirrors ConexAGAPController._wait_move()'s
        double-read confirmation and MotionFault/AxisStateUnknown
        escalation contract."""
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
    # Coordinate conversion (mechanical deg <-> normalized unit-circle XY)
    # ------------------------------------------------------------------

    def _mm_to_xy_from_mech_deg(self, mech_deg_x: float, mech_deg_y: float) -> tuple:
        """MECHANICAL degrees -> normalized XY per the Operation Manual's
        formula: optical_deg = 2 * mechanical_deg (fold-mirror doubling),
        x = tan(optical_deg) / tan(50 deg). Does NOT clamp to the unit
        circle -- see _validate_normalized_target()."""
        max_optical_rad = math.radians(_MAX_OPTICAL_DEG)
        tan_max = math.tan(max_optical_rad)
        optical_x = math.radians(mech_deg_x * _MECH_TO_OPTICAL)
        optical_y = math.radians(mech_deg_y * _MECH_TO_OPTICAL)
        return (math.tan(optical_x) / tan_max, math.tan(optical_y) / tan_max)

    def _xy_to_mech_deg(self, norm_x: float, norm_y: float) -> tuple:
        """Inverse of _mm_to_xy_from_mech_deg(): normalized XY (as read
        back from the RadialBasisFunction registers) -> MECHANICAL
        degrees. optical_deg = atan(norm * tan(50 deg)), mechanical_deg =
        optical_deg / 2."""
        max_optical_rad = math.radians(_MAX_OPTICAL_DEG)
        tan_max = math.tan(max_optical_rad)
        optical_x = math.atan(norm_x * tan_max)
        optical_y = math.atan(norm_y * tan_max)
        return (math.degrees(optical_x) / _MECH_TO_OPTICAL, math.degrees(optical_y) / _MECH_TO_OPTICAL)

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

    def _read_actual_mechanical_deg(self) -> tuple:
        """Read ACTUAL (measured) position via a brief Pro-mode
        round-trip, return as MECHANICAL degrees. Enters Pro mode, reads
        both RadialBasisFunction registers, then returns to Simple mode
        -- see module docstring's POSITION READBACK section. Raises if
        the mode switch or either register read fails; does NOT silently
        fall back to Simple mode on error, since a caller relying on this
        for closed-loop feedback should see a real failure, not a
        plausible-looking wrong number."""
        self._enter_pro_mode()
        try:
            norm_x = self._pro_get_float(_REG_MIRROR_COORD_X)
            norm_y = self._pro_get_float(_REG_MIRROR_COORD_Y)
        finally:
            self._exit_pro_mode()
        return self._xy_to_mech_deg(norm_x, norm_y)

    def _read_fault_register(self) -> int:
        """Diagnostic-only Pro-mode read of _REG_SYSTEM_STATUS_ERRORS,
        used by _diagnose_timeout_fault() on a wait_for_settle() timeout.
        Same enter/read/exit-Pro-mode shape as
        _read_actual_mechanical_deg() -- always attempts to return to
        Simple mode even if the read itself fails, so a failed
        diagnostic attempt doesn't additionally strand the connection in
        Pro mode."""
        self._enter_pro_mode()
        try:
            return self._pro_get_uint32(_REG_SYSTEM_STATUS_ERRORS)
        finally:
            self._exit_pro_mode()

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
        """START -> OK, per Table 3. Called at connect time and again
        when returning from Pro mode."""
        resp = self._query("START")
        if resp.strip().upper() != "OK":
            raise RuntimeError(f"Unexpected response to START: {resp!r}")

    def _send(self, command: str) -> str:
        """Send a Simple Serial command and return its response
        (OK/NO/OL/OU/ERROR, or a data reply for GETID/STATUS/etc). Every
        Simple Serial command gets an explicit reply per the manual."""
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
        """Alias for _send() -- kept separate for readability at call
        sites that are semantically "asking for data" vs. "commanding an
        action," even though the wire behavior is identical in Simple
        Serial mode."""
        return self._send(command)

    def _send_xy(self, norm_x: float, norm_y: float):
        """XY=%f;%f -- set both axes in one command. Raises on NO/ERROR;
        logs (does not raise on) OL/OU, since _validate_normalized_target()
        should have already caught anything that would trigger those --
        seeing one here means our own pre-validation missed a case the
        firmware caught."""
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
        escalation. There is no documented stop command in this
        firmware (confirmed absent from both the Simple Serial command
        table and every method in Optotune's own SDK) -- a timeout here
        raises AxisStateUnknown directly, with nothing to try first, but
        (v4) first makes a best-effort diagnostic read of the board-fault
        register via _diagnose_timeout_fault() and folds it into the
        raised message -- diagnosis only, never a reason to auto-command
        another move. See the module docstring's V4 UPDATE section."""
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

        fault_note = self._diagnose_timeout_fault()
        raise AxisStateUnknown(
            f"[{label}] Mirror did not report stable (STATUS bit "
            f"{_STATUS_BIT_MIRROR_NOT_STABLE}) within {self._move_timeout:.1f} s, "
            "and this firmware has no documented stop command to fall back "
            "on. Axis state is unknown -- do not issue further moves "
            f"without checking the hardware. {fault_note}"
        )

    def _diagnose_timeout_fault(self) -> str:
        """Best-effort diagnostic read of the board-fault register
        (_REG_SYSTEM_STATUS_ERRORS) on a wait_for_settle() timeout,
        folded into the AxisStateUnknown _wait_move() is about to raise.
        This is diagnosis only -- it never changes the escalation
        itself. See the module docstring's V4 UPDATE section for why
        freeze-and-raise (never auto-recover with another move) is still
        the only response to a timeout, given this firmware has no
        stop/abort command at all to recover TO.

        A failure here (e.g. the same comm problem that broke settling
        also breaks this Pro-mode round-trip) is logged and folded into
        the returned note rather than swallowed -- it must never be
        mistaken for "no fault found," since a dead diagnostic read is
        itself evidence something is wrong, just not what.
        """
        try:
            fault_word = self._read_fault_register()
        except Exception as exc:
            logger.error(
                "Settle timeout: diagnostic read of the board-fault "
                "register also failed (%s) -- communication with the "
                "MR-E-3 may be down entirely, not just the mirror "
                "failing to settle.", exc,
            )
            return (
                "Diagnostic read of the board-fault register also "
                f"failed ({exc}) -- this may be a lost connection, not "
                "just a stuck mirror."
            )
        logger.error(
            "Settle timeout: board-fault register (0x%04X) = 0x%08X. "
            "Exact bit layout not confirmed by this project (see module "
            "docstring) -- cross-reference against Optotune's MRE3Status "
            "documentation before assuming a specific fault.",
            _REG_SYSTEM_STATUS_ERRORS, fault_word,
        )
        return (
            f"Board-fault register 0x{_REG_SYSTEM_STATUS_ERRORS:04X} "
            f"read back 0x{fault_word:08X} at timeout (exact bit layout "
            "unconfirmed -- log this value for cross-reference against "
            "Optotune's docs)."
        )

    # ------------------------------------------------------------------
    # Internal helpers -- Pro-mode binary protocol, ported from
    # Optotune's own SDK source (optoKummenberg.tools.parsing_tools /
    # .commands) -- see module docstring's POSITION READBACK section.
    # Used ONLY for reading actual position; every other operation stays
    # in Simple Serial mode.
    # ------------------------------------------------------------------

    def _enter_pro_mode(self):
        """ASCII "GOPRO" -> OK, per the Operation Manual's Table 3.
        CRC is not used (GOPRO, not GOPROCRC) -- matches the SDK's own
        default encode() behavior, which never actually computes a CRC
        (see module docstring)."""
        resp = self._query("GOPRO")
        if resp.strip().upper() != "OK":
            raise RuntimeError(f"Unexpected response to GOPRO: {resp!r}")

    def _exit_pro_mode(self):
        """Pro-mode SET_COMM_MODE(0) frame, then a fresh ASCII
        START/OK handshake to confirm the switch back to Simple mode --
        exactly mirrors optoKummenberg.commands.Command.go_simple().
        Always attempted even if the caller is mid-exception (see
        _read_actual_mechanical_deg()'s try/finally), so a failed
        register read doesn't strand the connection in Pro mode where
        none of the Simple Serial move/settle code above would work."""
        try:
            frame = self._pro_encode(_PRO_CMD_SET_COMM_MODE, 0, data=None)
            self._ser.reset_input_buffer()
            self._ser.write(frame)
            self._read_pro_frame()  # consume the Pro-mode ack; don't need to parse it
        except Exception as exc:
            logger.warning(
                "Error sending Pro-mode exit frame (continuing to attempt "
                "handshake anyway): %s", exc,
            )
        self._handshake()

    def _pro_get_float(self, register_id: int) -> float:
        """GET_VALUE(register_id) in Pro mode -> big-endian float32.
        Caller must already be in Pro mode (see _read_actual_mechanical_deg())."""
        frame = self._pro_encode(_PRO_CMD_GET_VALUE, register_id, data=None)
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        raw = self._read_pro_frame()
        payload = self._pro_decode_get_value(raw, _PRO_CMD_GET_VALUE, register_id)
        if len(payload) != 4:
            raise RuntimeError(
                f"Expected 4-byte float payload reading register "
                f"0x{register_id:04X}, got {len(payload)} bytes: {payload!r}"
            )
        return struct.unpack(">f", payload)[0]

    def _pro_get_uint32(self, register_id: int) -> int:
        """GET_VALUE(register_id) in Pro mode -> big-endian uint32. Same
        wire mechanics as _pro_get_float() but for a register that holds
        a bitmask/integer rather than a measurement (e.g.
        _REG_SYSTEM_STATUS_ERRORS) -- caller must already be in Pro
        mode."""
        frame = self._pro_encode(_PRO_CMD_GET_VALUE, register_id, data=None)
        self._ser.reset_input_buffer()
        self._ser.write(frame)
        raw = self._read_pro_frame()
        payload = self._pro_decode_get_value(raw, _PRO_CMD_GET_VALUE, register_id)
        if len(payload) != 4:
            raise RuntimeError(
                f"Expected 4-byte payload reading register "
                f"0x{register_id:04X}, got {len(payload)} bytes: {payload!r}"
            )
        return struct.unpack(">I", payload)[0]

    @staticmethod
    def _pro_stuff(core: bytes) -> bytes:
        """Byte-stuff 0x7D/0x7E inside `core`, wrap with FRAME_BOUNDARY
        delimiters. Ported from optoKummenberg.tools.parsing_tools.encode()'s
        stuffing step: 0x7D -> 0x7D,0x5D ; 0x7E -> 0x7D,0x5E (i.e. escape
        byte followed by original XOR 0x20)."""
        stuffed = bytearray()
        for b in core:
            if b in (_ESCAPE_BYTE, _FRAME_BOUNDARY):
                stuffed.append(_ESCAPE_BYTE)
                stuffed.append(b ^ _ESCAPE_MASK)
            else:
                stuffed.append(b)
        return bytes([_FRAME_BOUNDARY]) + bytes(stuffed) + bytes([_FRAME_BOUNDARY])

    @staticmethod
    def _pro_destuff(frame: bytes) -> bytes:
        """Inverse of _pro_stuff(), operating on a full frame (leading and
        trailing FRAME_BOUNDARY included) -- returns the de-stuffed core
        bytes (delimiters stripped). Ported from
        optoKummenberg.tools.parsing_tools.decode()."""
        if len(frame) < 2 or frame[0] != _FRAME_BOUNDARY or frame[-1] != _FRAME_BOUNDARY:
            raise RuntimeError(f"Malformed Pro-mode frame (bad delimiters): {frame!r}")
        result = bytearray()
        i = 1
        while i < len(frame) - 1:
            if frame[i] == _ESCAPE_BYTE:
                result.append(frame[i + 1] ^ _ESCAPE_MASK)
                i += 2
            else:
                result.append(frame[i])
                i += 1
        return bytes(result)

    def _pro_encode(self, command_id: int, register_id: int, data=None) -> bytes:
        """Build a Pro-mode frame for GET_VALUE (data=None) or the
        SET_COMM_MODE(0) mode-switch command (also data=None -- per the
        SDK's go_simple(), the "register_id" field doubles as the target
        mode for this specific command). This driver never needs to SET
        a float/int register value, so the data-carrying branches of the
        SDK's encode() are intentionally not ported here -- add them if a
        future need (e.g. writing PID parameters) comes up."""
        core = bytes([_PRO_SLAVE_ADDRESS, command_id, 0x02]) + struct.pack(">H", register_id)
        core += bytes([0x00, 0x00])  # CRC -- always zero, see module docstring
        return self._pro_stuff(core)

    def _read_pro_frame(self) -> bytes:
        """Read one complete "~...~" Pro-mode frame from the serial port,
        byte by byte, bounded by the serial port's configured timeout.
        Deliberately simpler than the SDK's own receive()/read_until()
        logic (which has special-cased retry behavior for the
        leading-boundary-byte edge case) -- this reads until it has seen
        a genuine start delimiter followed later by an end delimiter,
        which is correct regardless of exactly how many bytes arrive per
        read() call."""
        deadline = time.monotonic() + _PRO_FRAME_TIMEOUT_S
        b = self._ser.read(1)
        while b and b[0] != _FRAME_BOUNDARY:
            if time.monotonic() > deadline:
                raise RuntimeError("Timed out waiting for Pro-mode frame start.")
            b = self._ser.read(1)
        if not b:
            raise RuntimeError(
                "No response from MR-E-3 in Pro mode (timed out waiting for "
                "frame start). Lost connection, or device not actually in "
                "Pro mode?"
            )
        frame = bytearray(b)
        while True:
            if time.monotonic() > deadline:
                raise RuntimeError("Timed out waiting for Pro-mode frame end.")
            b = self._ser.read(1)
            if not b:
                raise RuntimeError("Timed out waiting for Pro-mode frame end.")
            frame.append(b[0])
            if b[0] == _FRAME_BOUNDARY:
                break
        return bytes(frame)

    def _pro_decode_get_value(self, raw_frame: bytes, sent_command_id: int, register_id: int) -> bytes:
        """De-stuff a Pro-mode response frame and return its data payload,
        after checking slave address and command ID (raising on an error
        response, i.e. command_id == sent | 0x80)."""
        core = self._pro_destuff(raw_frame)
        if len(core) < 5:
            raise RuntimeError(f"Pro-mode response too short: {core!r}")
        slave_id = core[0]
        resp_command_id = core[1]
        size = core[2]
        payload = core[3:3 + size]
        if slave_id != _PRO_SLAVE_ADDRESS:
            raise RuntimeError(
                f"Pro-mode response from unexpected slave address "
                f"0x{slave_id:02X} (expected 0x{_PRO_SLAVE_ADDRESS:02X})."
            )
        if resp_command_id == (sent_command_id | _PRO_ERROR_FLAG):
            error_code = struct.unpack(">I", payload[:4])[0] if len(payload) >= 4 else None
            raise MotionFault(
                f"MR-15-30 Pro-mode GET_VALUE(0x{register_id:04X}) rejected: "
                f"error code 0x{error_code:08X}" if error_code is not None
                else f"MR-15-30 Pro-mode GET_VALUE(0x{register_id:04X}) rejected: {payload!r}"
            )
        if resp_command_id != sent_command_id:
            raise RuntimeError(
                f"Pro-mode response command ID mismatch: sent "
                f"0x{sent_command_id:02X}, got 0x{resp_command_id:02X}."
            )
        return payload


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
    parser.add_argument("--hard-home", action="store_true", default=True,
                        help="Use hard-home (XY=0;0) for this smoke test (default).")
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
            "hard_home": args.hard_home,
            "motion_enabled": args.allow_motion,
            "deg_per_mm_x": args.deg_per_mm_x,
            "deg_per_mm_y": args.deg_per_mm_y,
        }
    }

    with MR1530Controller(cfg) as mc:
        print("=== Homing ===")
        mc.home()
        print(f"Position (mm, using placeholder deg/mm): {mc.get_position()}")
        print(f"Absolute position (raw mechanical deg): {mc.get_absolute_position_deg()}")

    print("\nDone.")
