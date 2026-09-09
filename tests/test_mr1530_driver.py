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

from motion.motion_controller import MotionFault, AxisStateUnknown

from tests.fake_mr1530_serial import FakeMR1530Serial, make_controller


class ConstructorValidationTests(unittest.TestCase):
    """The driver's __init__ deliberately has no safe placeholder for
    deg_per_mm_x/y (unlike the CONEX driver's ~0.05 guess) -- these lock
    in that fail-loud behavior."""

    def test_constructor_requires_port(self):
        with self.assertRaises(ValueError):
            make_controller(FakeMR1530Serial(), port=None, deg_per_mm_x=1.0, deg_per_mm_y=1.0)

    def test_constructor_requires_deg_per_mm(self):
        with self.assertRaises(ValueError):
            make_controller(FakeMR1530Serial(), deg_per_mm_x=None, deg_per_mm_y=None)


class InterlockAndPreconditionTests(unittest.TestCase):
    def test_home_requires_motion_permission(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake, motion_enabled=False)
        with self.assertRaises(MotionFault):
            mc.home()
        self.assertEqual(
            fake.move_commands, [],
            "the fail-closed interlock must block the move before "
            "anything is sent, not just raise after the fact.",
        )

    def test_move_to_requires_homed_first(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake)
        with self.assertRaises(RuntimeError):
            mc.move_to(1.0, 1.0)


class PositionAndMoveTests(unittest.TestCase):
    def test_move_to_and_get_position_round_trip(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake)
        mc.home()
        mc.move_to(10.0, -8.0)
        mc.wait_for_settle(0.0)
        x, y = mc.get_position()
        self.assertAlmostEqual(x, 10.0, places=3)
        self.assertAlmostEqual(y, -8.0, places=3)

    def test_calibration_jog_mm_wrapper_moves_relative(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake)
        mc.home()
        mc.calibration_jog(dx_mm=1.0, dy_mm=0.0)
        x, y = mc.get_position()
        self.assertAlmostEqual(x, 1.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)

    def test_move_to_rejects_target_outside_unit_circle_before_sending(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake)
        mc.home()
        commands_before = len(fake.move_commands)
        with self.assertRaises(MotionFault):
            mc.move_to(24.0, 24.0)  # combined radius > 1 -- the unit
            # circle couples both axes, this is not "24 deg is under the
            # 25 deg mechanical spec so it must be fine" on either axis
            # alone.
        self.assertEqual(
            len(fake.move_commands), commands_before,
            "an out-of-circle target must be rejected before anything "
            "is sent to the firmware.",
        )

    def test_move_rejected_by_firmware_raises_motionfault(self):
        for code in ("NO", "ERROR"):
            with self.subTest(code=code):
                fake = FakeMR1530Serial()
                mc = make_controller(fake)
                mc.home()
                fake.xy_response = code
                with self.assertRaises(MotionFault):
                    mc.move_to(1.0, 1.0)

    def test_firmware_clamp_response_does_not_raise(self):
        for code in ("OL", "OU"):
            with self.subTest(code=code):
                fake = FakeMR1530Serial()
                mc = make_controller(fake)
                mc.home()
                fake.xy_response = code
                try:
                    mc.move_to(1.0, 1.0)
                except Exception as exc:
                    self.fail(
                        f"move_to() must not raise on firmware {code} -- "
                        f"that's a logged pre-validation mismatch, not an "
                        f"error (see _send_xy()'s docstring) -- raised {exc!r}"
                    )


class AxisMappingAndOriginTests(unittest.TestCase):
    """Mirrors test_conex_safety.py's
    test_get_position_deg_applies_axis_mapping_and_invert_not_ratio /
    test_absolute_position_ignores_origin_and_invert -- same questions,
    same reasoning, different hardware."""

    def test_get_position_deg_applies_axis_mapping_and_invert_not_ratio(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake, deg_per_mm_x=2.0, deg_per_mm_y=3.0, invert_y=True)
        mc.home()  # origin = (0, 0) mechanical deg

        mech_x, mech_y = 4.0, -6.0
        fake.norm_x, fake.norm_y = mc._mm_to_xy_from_mech_deg(mech_x, mech_y)

        abs_x, abs_y = mc.get_absolute_position_deg()
        self.assertAlmostEqual(abs_x, mech_x, places=4)
        self.assertAlmostEqual(abs_y, mech_y, places=4)

        dx, dy = mc.get_position_deg()
        self.assertAlmostEqual(dx, 4.0, places=4)   # sign_x = +1
        self.assertAlmostEqual(dy, 6.0, places=4)   # sign_y = -1 (invert_y)

        px, py = mc.get_position()
        self.assertAlmostEqual(px, 2.0, places=4)   # eff_deg_per_mm_x = 2.0
        self.assertAlmostEqual(py, 2.0, places=4)   # eff_deg_per_mm_y = 3.0 * -1

    def test_absolute_position_ignores_origin_and_invert(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake, invert_x=True, invert_y=True)
        mc.home()

        mech_x, mech_y = 5.0, -3.0
        fake.norm_x, fake.norm_y = mc._mm_to_xy_from_mech_deg(mech_x, mech_y)
        mc.zero_here()  # anchors origin at this exact position

        abs_x, abs_y = mc.get_absolute_position_deg()
        self.assertAlmostEqual(abs_x, mech_x, places=4)
        self.assertAlmostEqual(abs_y, mech_y, places=4)

        dx, dy = mc.get_position_deg()
        self.assertAlmostEqual(dx, 0.0, places=4)
        self.assertAlmostEqual(dy, 0.0, places=4)

    def test_resume_soft_home_leaves_origin_and_sends_nothing(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake, hard_home=False)
        mech_x, mech_y = 2.0, 2.0
        fake.norm_x, fake.norm_y = mc._mm_to_xy_from_mech_deg(mech_x, mech_y)
        mc.zero_here()
        commands_before = len(fake.move_commands)
        mc.resume()
        self.assertEqual(
            len(fake.move_commands), commands_before,
            "resume() in soft-home mode must not re-home or send a move "
            "-- it should just mark the controller ready, keeping the "
            "existing fiducial origin.",
        )
        self.assertAlmostEqual(mc.get_position_deg()[0], 0.0, places=4)


class CalibrationJogRegressionTests(unittest.TestCase):
    def test_calibration_jog_deg_is_relative_to_current_position_not_origin(self):
        """Regression class for the 2026-08-06 CONEX bug
        (calibration_jog_deg_absolute_not_relative): a repeated jog in
        the same direction must not be a no-op. This only catches that
        bug class if calibration_jog_deg() reads LIVE position before
        each jog rather than computing from a cached origin -- exactly
        what this test drives twice in a row to check."""
        fake = FakeMR1530Serial()
        mc = make_controller(fake)
        mc.home()

        mc.calibration_jog_deg(dx_deg=1.0, dy_deg=0.0)
        first_x, first_y = mc.get_absolute_position_deg()
        self.assertAlmostEqual(first_x, 1.0, places=4)
        self.assertAlmostEqual(first_y, 0.0, places=4)

        mc.calibration_jog_deg(dx_deg=1.0, dy_deg=0.0)
        second_x, _ = mc.get_absolute_position_deg()
        self.assertAlmostEqual(
            second_x, 2.0, places=4,
            msg="calibration_jog_deg() must read LIVE position before "
            "jogging, not jog relative to a cached origin -- a repeated "
            "jog in one direction must not be a no-op.",
        )


class StatusFaultBitTests(unittest.TestCase):
    def test_current_limit_status_bit_raises_motionfault_immediately(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake)
        mc.home()
        fake.current_limit = True
        with self.assertRaises(MotionFault) as ctx:
            mc.wait_for_settle(settle_time_s=0.0)
        self.assertNotIsInstance(
            ctx.exception, AxisStateUnknown,
            "a current-limit fault is CONFIRMED by STATUS directly, not "
            "an unknown-state timeout -- must raise the plain "
            "MotionFault branch, not fall through to AxisStateUnknown.",
        )

    def test_mirror_temperature_limit_status_bit_raises_motionfault_immediately(self):
        fake = FakeMR1530Serial()
        mc = make_controller(fake)
        mc.home()
        fake.temp_limit = True
        with self.assertRaises(MotionFault) as ctx:
            mc.wait_for_settle(settle_time_s=0.0)
        self.assertNotIsInstance(ctx.exception, AxisStateUnknown)

    def test_settle_confirm_flip_back_is_treated_as_still_settling(self):
        """Sequence: unstable, unstable, STABLE (triggers the confirm
        read) -> confirm read UNSTABLE again (must be treated as still
        settling, per _wait_move()'s "reported stable then unstable
        again" branch, not a false "done") -> more polling -> STABLE,
        confirm STABLE -> actually settled."""
        fake = FakeMR1530Serial(
            status_bit4_sequence=[True, True, False, True, False, False]
        )
        mc = make_controller(fake, move_timeout_s=1.0)
        mc.wait_for_settle(settle_time_s=0.0)  # must return, not raise
        self.assertGreaterEqual(fake._status_call_count, 6)


class ProModeProtocolTests(unittest.TestCase):
    def test_fault_register_round_trip_survives_byte_stuffing(self):
        """0x7E7D0102 deliberately contains both bytes the Pro-mode
        framing must escape (0x7E the frame boundary, 0x7D the escape
        byte itself) -- unlike every value exercised in
        test_mr1530_safety.py, which never happened to need escaping."""
        tricky_value = 0x7E7D0102
        fake = FakeMR1530Serial(fault_register_value=tricky_value)
        mc = make_controller(fake)
        result = mc._read_fault_register()
        self.assertEqual(result, tricky_value)


if __name__ == "__main__":
    unittest.main()
