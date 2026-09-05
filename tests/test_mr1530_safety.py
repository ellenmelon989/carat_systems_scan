import unittest
from unittest.mock import patch
import struct

# --- repo-root import bootstrap -------------------------------------------
# Same reasoning as tests/test_conex_safety.py -- running this file
# directly only puts tests/ on sys.path, not the repo root.
import os as _os
import sys as _sys

_REPO_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _REPO_ROOT not in _sys.path:
    _sys.path.insert(0, _REPO_ROOT)
# ---------------------------------------------------------------------------

from motion.motion_controller import AxisStateUnknown
import motion.real_mr1530_motion as mr1530_module
from motion.real_mr1530_motion import MR1530Controller

_STATUS_BIT_MIRROR_NOT_STABLE = 4


class FakeMR1530Serial:
    """Fakes the MR-E-3's Simple Serial + Pro-mode wire behavior far
    enough to exercise wait_for_settle()'s timeout/diagnostic path
    without real hardware. Reuses MR1530Controller's own _pro_stuff/
    _pro_destuff staticmethods to build/parse Pro-mode frames instead of
    re-implementing the byte-stuffing protocol a second time here --
    those two methods are exactly what the driver itself uses, so a bug
    in this fake's protocol handling would be a bug in the shared
    (already-covered-by-use) staticmethods, not a second independent
    implementation that could silently drift from the real one.
    """

    def __init__(self, fault_register_value=0, never_settles=True):
        self.timeout = 0.05
        self.write_timeout = 0.05
        self.commands = []
        self.move_commands = []
        self.fault_register_value = fault_register_value
        self.never_settles = never_settles
        self.fault_register_readable = True
        self._out = bytearray()
        self.closed = False

    def reset_input_buffer(self):
        pass

    def write(self, payload: bytes) -> int:
        self.commands.append(payload)
        if payload[:1] == bytes([mr1530_module._FRAME_BOUNDARY]):
            self._handle_pro_frame(payload)
        else:
            self._handle_ascii(payload)
        return len(payload)

    def _queue(self, data: bytes):
        self._out += data

    def _handle_ascii(self, payload: bytes):
        cmd = payload.decode("ascii").strip()
        if cmd == "START":
            self._queue(b"OK\r\n")
        elif cmd == "GETID":
            self._queue(b"MR-E-3-TEST\r\n")
        elif cmd == "GOPRO":
            self._queue(b"OK\r\n")
        elif cmd == "STATUS":
            status = (1 << _STATUS_BIT_MIRROR_NOT_STABLE) if self.never_settles else 0
            self._queue(f"{status:08X}\r\n".encode("ascii"))
        elif cmd.startswith(("X=", "Y=", "XY=")):
            # Freeze-on-timeout is the behavior under test -- these
            # should never actually be sent in the timeout scenarios,
            # but handle them so a regression that DOES send one fails
            # via the move_commands assertion, not a fake-serial crash.
            self.move_commands.append(cmd)
            self._queue(b"OK\r\n")
        else:
            self._queue(b"ERROR\r\n")

    def _handle_pro_frame(self, payload: bytes):
        core = MR1530Controller._pro_destuff(payload)
        cmd_id, size = core[1], core[2]
        body = core[3:3 + size]
        if cmd_id == mr1530_module._PRO_CMD_GET_VALUE:
            register_id = struct.unpack(">H", body[:2])[0]
            if register_id == mr1530_module._REG_SYSTEM_STATUS_ERRORS:
                if not self.fault_register_readable:
                    # Simulate a dead link: no response queued at all --
                    # the real _read_pro_frame() will see an empty read
                    # and raise "no response," matching what a genuinely
                    # broken comm link looks like on the wire.
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


def _make_controller(fake_serial):
    config = {
        "motion": {
            "controller": "mr1530",
            "port": "COM_TEST",
            "deg_per_mm_x": 1.0,
            "deg_per_mm_y": 1.0,
            "motion_enabled": True,
            "move_timeout_s": 0.15,
            "serial_timeout_s": 0.05,
        }
    }
    with patch.object(mr1530_module.serial, "Serial", return_value=fake_serial):
        return MR1530Controller(config)


class SettleTimeoutDiagnosisTests(unittest.TestCase):
    """Regression coverage for the 2026-09-05 settle-timeout policy
    decision: on a wait_for_settle() timeout, diagnose (best-effort read
    of the board-fault register) but always freeze -- never auto-command
    another move, since this firmware has no stop/abort command to
    recover TO and an unconfirmed axis state must not get a new target
    layered on top of it.
    """

    def test_timeout_includes_fault_register_value_when_readable(self):
        fake = FakeMR1530Serial(fault_register_value=0x00000005, never_settles=True)
        mc = _make_controller(fake)
        with self.assertRaises(AxisStateUnknown) as ctx:
            mc.wait_for_settle(settle_time_s=0.0)
        message = str(ctx.exception)
        self.assertIn("0x1007", message)
        self.assertIn("0x00000005", message)

    def test_timeout_never_commands_further_motion(self):
        fake = FakeMR1530Serial(fault_register_value=0x0, never_settles=True)
        mc = _make_controller(fake)
        with self.assertRaises(AxisStateUnknown):
            mc.wait_for_settle(settle_time_s=0.0)
        self.assertEqual(
            fake.move_commands, [],
            "wait_for_settle() timeout must never auto-command a move "
            "(e.g. a recovery XY=0;0) on top of an axis whose state is "
            "unconfirmed -- freeze and raise only.",
        )

    def test_timeout_survives_a_dead_diagnostic_read(self):
        fake = FakeMR1530Serial(never_settles=True)
        fake.fault_register_readable = False
        mc = _make_controller(fake)
        with self.assertRaises(AxisStateUnknown) as ctx:
            mc.wait_for_settle(settle_time_s=0.0)
        message = str(ctx.exception).lower()
        self.assertIn("diagnostic read", message)
        self.assertEqual(fake.move_commands, [])

    def test_settled_mirror_does_not_raise(self):
        fake = FakeMR1530Serial(never_settles=False)
        mc = _make_controller(fake)
        mc.wait_for_settle(settle_time_s=0.0)  # should return normally


if __name__ == "__main__":
    unittest.main()
