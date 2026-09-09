import unittest

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

from tests.fake_mr1530_serial import FakeMR1530Serial, make_controller


class SettleTimeoutDiagnosisTests(unittest.TestCase):
    """Regression coverage for the 2026-09-05 settle-timeout policy
    decision: on a wait_for_settle() timeout, diagnose (best-effort read
    of the board-fault register) but always freeze -- never auto-command
    another move, since this firmware has no stop/abort command to
    recover TO and an unconfirmed axis state must not get a new target
    layered on top of it.

    Uses the shared FakeMR1530Serial from fake_mr1530_serial.py (also
    used by test_mr1530_driver.py's broader protocol coverage) rather
    than a locally duplicated fake, so there is exactly one
    implementation of the wire protocol to keep in sync with the driver.
    """

    def test_timeout_includes_fault_register_value_when_readable(self):
        fake = FakeMR1530Serial(fault_register_value=0x00000005, never_settles=True)
        mc = make_controller(fake)
        with self.assertRaises(AxisStateUnknown) as ctx:
            mc.wait_for_settle(settle_time_s=0.0)
        message = str(ctx.exception)
        self.assertIn("0x1007", message)
        self.assertIn("0x00000005", message)

    def test_timeout_never_commands_further_motion(self):
        fake = FakeMR1530Serial(fault_register_value=0x0, never_settles=True)
        mc = make_controller(fake)
        with self.assertRaises(AxisStateUnknown):
            mc.wait_for_settle(settle_time_s=0.0)
        self.assertEqual(
            fake.move_commands, [],
            "wait_for_settle() timeout must never auto-command a move "
            "(e.g. a recovery XY=0;0) on top of an axis whose state is "
            "unconfirmed -- freeze and raise only.",
        )

    def test_timeout_survives_a_dead_diagnostic_read(self):
        fake = FakeMR1530Serial(never_settles=True, fault_register_readable=False)
        mc = make_controller(fake)
        with self.assertRaises(AxisStateUnknown) as ctx:
            mc.wait_for_settle(settle_time_s=0.0)
        message = str(ctx.exception).lower()
        self.assertIn("diagnostic read", message)
        self.assertEqual(fake.move_commands, [])

    def test_settled_mirror_does_not_raise(self):
        fake = FakeMR1530Serial(never_settles=False)
        mc = make_controller(fake)
        mc.wait_for_settle(settle_time_s=0.0)  # should return normally


if __name__ == "__main__":
    unittest.main()
