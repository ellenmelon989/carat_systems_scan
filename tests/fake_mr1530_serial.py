"""
fake_mr1530_serial.py

Shared stateful fake for the MR-E-3's Simple Serial + Pro-mode wire
protocol, used by both test_mr1530_safety.py (settle-timeout/diagnostic
path) and test_mr1530_driver.py (the broader protocol surface: moves,
position readback, calibration jogging, firmware fault/clamp responses).

Deliberately reuses MR1530Controller's own _pro_stuff/_pro_destuff
staticmethods to build/parse Pro-mode frames rather than re-implementing
the byte-stuffing protocol a second time -- a bug in this fake's framing
would then be a bug in the shared (already-exercised-by-the-real-driver)
staticmethods, not a second independent implementation that could
silently drift from the real one.

This is the same pre-hardware-verification role FakeConexSerial played
for real_conexagap_motion.py before that driver ever touched a real
CONEX unit (see tests/test_conex_safety.py) -- it is NOT a substitute
for testing against real hardware once it arrives (see
docs/mr1530_migration_plan.md section 5).
"""

import struct
from unittest.mock import patch

import motion.real_mr1530_motion as mr1530_module
from motion.real_mr1530_motion import MR1530Controller


class FakeMR1530Serial:
    """Simulates the MR-E-3 well enough to exercise MR1530Controller's
    full public surface without real hardware.

    Tracks the "mirror's" true position as normalized unit-circle XY
    (norm_x, norm_y) -- the same coordinate system both the Simple
    Serial X=/Y=/XY= move commands and the Pro-mode 0x3B00/0x3B01
    position registers use on the real device -- rather than also
    duplicating the driver's mechanical-degree <-> normalized-XY
    conversion math here. That conversion lives in (and is exercised
    through) the driver itself; this fake only needs to remember where
    the mirror was last commanded and answer position reads with it.

    Configurable failure/edge-case knobs, all defaulting to "everything
    works normally":
      - xy_response: what a XY=... move command receives back. "OK"
        applies the move; "NO"/"ERROR" reject it like a real firmware
        fault; "OL"/"OU" simulate a firmware-side clamp happening
        despite the driver's own pre-validation -- see
        _validate_normalized_target()'s docstring on why that's a
        "shouldn't happen but log, don't crash" case, not an error.
      - status_bit4_sequence: an explicit per-STATUS-call sequence of
        "mirror not stable" readings (True/False), for precisely
        exercising the double-read settle-confirmation logic, including
        the "reported stable then unstable again on confirm" branch.
        Falls back to `never_settles` once the sequence is exhausted.
      - never_settles: STATUS always reports "not stable" (bit 4 set).
      - current_limit / avg_current_limit / temp_limit: force the
        corresponding STATUS fault bits, which _wait_move() escalates
        to MotionFault immediately (no timeout wait needed).
      - fault_register_value / fault_register_readable: the Pro-mode
        board-fault register (0x1007) _diagnose_timeout_fault() reads
        on a settle timeout.
    """

    def __init__(
        self,
        xy_response="OK",
        status_bit4_sequence=None,
        never_settles=False,
        current_limit=False,
        avg_current_limit=False,
        temp_limit=False,
        fault_register_value=0,
        fault_register_readable=True,
        initial_norm_x=0.0,
        initial_norm_y=0.0,
    ):
        self.timeout = 0.05
        self.write_timeout = 0.05
        self.commands = []
        self.move_commands = []

        self.xy_response = xy_response
        self.status_bit4_sequence = list(status_bit4_sequence or [])
        self.never_settles = never_settles
        self.current_limit = current_limit
        self.avg_current_limit = avg_current_limit
        self.temp_limit = temp_limit
        self.fault_register_value = fault_register_value
        self.fault_register_readable = fault_register_readable
        self._status_call_count = 0

        self.norm_x = initial_norm_x
        self.norm_y = initial_norm_y

        self._out = bytearray()
        self.closed = False

    # ------------------------------------------------------------------
    # pyserial.Serial-compatible surface
    # ------------------------------------------------------------------

    def reset_input_buffer(self):
        pass

    def write(self, payload: bytes) -> int:
        self.commands.append(payload)
        if payload[:1] == bytes([mr1530_module._FRAME_BOUNDARY]):
            self._handle_pro_frame(payload)
        else:
            self._handle_ascii(payload)
        return len(payload)

    def read(self, size: int = 1) -> bytes:
        if not self._out:
            return b""
        chunk = bytes(self._out[:size])
        del self._out[:size]
        return chunk

    def readline(self) -> bytes:
        idx = self._out.find(b"\n")
        if idx == -1:
            data = bytes(self._out)
            self._out.clear()
            return data
        data = bytes(self._out[:idx + 1])
        del self._out[:idx + 1]
        return data

    def close(self):
        self.closed = True

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _queue(self, data: bytes):
        self._out += data

    def _status_bit4_is_set(self) -> bool:
        if self._status_call_count < len(self.status_bit4_sequence):
            value = self.status_bit4_sequence[self._status_call_count]
        else:
            value = self.never_settles
        self._status_call_count += 1
        return bool(value)

    def _handle_ascii(self, payload: bytes):
        cmd = payload.decode("ascii").strip()
        if cmd == "START":
            self._queue(b"OK\r\n")
        elif cmd == "GETID":
            self._queue(b"MR-E-3-TEST\r\n")
        elif cmd == "GOPRO":
            self._queue(b"OK\r\n")
        elif cmd == "STATUS":
            bits = 0
            if self.current_limit:
                bits |= 1 << mr1530_module._STATUS_BIT_CURRENT_LIMIT
            if self.avg_current_limit:
                bits |= 1 << mr1530_module._STATUS_BIT_CURRENT_AVG_LIMIT
            if self.temp_limit:
                bits |= 1 << mr1530_module._STATUS_BIT_MIRROR_TEMP_LIMIT
            if self._status_bit4_is_set():
                bits |= 1 << mr1530_module._STATUS_BIT_MIRROR_NOT_STABLE
            self._queue(f"{bits:08X}\r\n".encode("ascii"))
        elif cmd.startswith("XY="):
            self.move_commands.append(cmd)
            body = cmd[len("XY="):]
            x_str, y_str = body.split(";")
            target_x, target_y = float(x_str), float(y_str)
            if self.xy_response == "OK":
                self.norm_x, self.norm_y = target_x, target_y
            self._queue(f"{self.xy_response}\r\n".encode("ascii"))
        else:
            self._queue(b"ERROR\r\n")

    def _handle_pro_frame(self, payload: bytes):
        core = MR1530Controller._pro_destuff(payload)
        cmd_id, size = core[1], core[2]
        body = core[3:3 + size]
        if cmd_id == mr1530_module._PRO_CMD_GET_VALUE:
            register_id = struct.unpack(">H", body[:2])[0]
            if register_id == mr1530_module._REG_MIRROR_COORD_X:
                resp_payload = struct.pack(">f", self.norm_x)
            elif register_id == mr1530_module._REG_MIRROR_COORD_Y:
                resp_payload = struct.pack(">f", self.norm_y)
            elif register_id == mr1530_module._REG_SYSTEM_STATUS_ERRORS:
                if not self.fault_register_readable:
                    # Simulate a dead link: no response queued at all --
                    # the real _read_pro_frame() sees an empty read and
                    # raises "no response," matching a genuinely broken
                    # comm link on the wire.
                    return
                resp_payload = struct.pack(">I", self.fault_register_value)
            else:
                resp_payload = struct.pack(">f", 0.0)
            resp_core = bytes([0x00, cmd_id, len(resp_payload)]) + resp_payload + bytes([0x00, 0x00])
            self._queue(MR1530Controller._pro_stuff(resp_core))
        elif cmd_id == mr1530_module._PRO_CMD_SET_COMM_MODE:
            # _exit_pro_mode() discards this response's content -- any
            # well-formed frame is enough to unblock its _read_pro_frame().
            resp_core = bytes([0x00, cmd_id, 0]) + bytes([0x00, 0x00])
            self._queue(MR1530Controller._pro_stuff(resp_core))


def make_controller(fake_serial, **motion_overrides):
    """Construct a MR1530Controller wired to `fake_serial` instead of a
    real serial.Serial(). Defaults to the minimal viable config (1.0
    deg/mm both axes, motion enabled, hard-home) -- pass motion_overrides
    to change any motion.* key (e.g. invert_y=True, hard_home=False,
    deg_per_mm_x=2.0)."""
    motion_cfg = {
        "controller": "mr1530",
        "port": "COM_TEST",
        "deg_per_mm_x": 1.0,
        "deg_per_mm_y": 1.0,
        "motion_enabled": True,
        "hard_home": True,
        "move_timeout_s": 0.2,
        "serial_timeout_s": 0.05,
    }
    motion_cfg.update(motion_overrides)
    config = {"motion": motion_cfg}
    with patch.object(mr1530_module.serial, "Serial", return_value=fake_serial):
        return MR1530Controller(config)
