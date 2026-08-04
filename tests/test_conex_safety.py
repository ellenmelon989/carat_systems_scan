import unittest
from unittest.mock import patch

from motion.motion_controller import MotionFault
from motion.real_conexagap_motion import ConexAGAPController
import motion.real_conexagap_motion as conex_module
import tools.recover_conex_axis as recovery_module


class FakeConexSerial:
    def __init__(self, position_u=0.0, position_v=0.0):
        self.position_u = position_u
        self.position_v = position_v
        self.timeout = 0.1
        self.commands = []
        self._response = b""
        self.closed = False

    def reset_input_buffer(self):
        self._response = b""

    def write(self, payload):
        command = payload.decode("ascii").strip()
        self.commands.append(command)
        responses = {
            "1ID?": "1IDAG-M100D",
            "1TPU": f"1TPU{self.position_u}",
            "1TPV": f"1TPV{self.position_v}",
            "1SLU?": "1SLU-0.76",
            "1SRU?": "1SRU0.76",
            "1SLV?": "1SLV-0.76",
            "1SRV?": "1SRV0.76",
            "1TS": "1TS000032",
        }
        response = responses.get(command)
        self._response = (response + "\r\n").encode("ascii") if response else b""
        return len(payload)

    def flush(self):
        pass

    def readline(self):
        response = self._response
        self._response = b""
        return response

    def close(self):
        self.closed = True


class FakeRecoverySerial(FakeConexSerial):
    def __init__(self):
        super().__init__(position_v=-1.477)
        self.xu_v = 35

    def write(self, payload):
        command = payload.decode("ascii").strip()
        if command == "1XUV?":
            self.commands.append(command)
            self._response = f"1XUV{self.xu_v}\r\n".encode("ascii")
            return len(payload)
        if command.startswith("1XUV"):
            self.commands.append(command)
            self.xu_v = int(command[len("1XUV"):])
            self._response = b""
            return len(payload)
        if command == "1TE":
            self.commands.append(command)
            self._response = b"1TE@\r\n"
            return len(payload)
        if command == "1XRV10":
            self.commands.append(command)
            self.position_v = -1.476
            self._response = b""
            return len(payload)
        return super().write(payload)


def config(**motion_overrides):
    motion = {
        "controller": "conex_agap",
        "port": "COM4",
        "controller_address": 1,
        "axis_x": "U",
        "axis_y": "V",
        "steps_per_mm_x": 0.01,
        "steps_per_mm_y": 0.01,
        "motion_enabled": False,
        "calibration_confirmed": False,
        "hard_home": False,
    }
    motion.update(motion_overrides)
    return {"motion": motion}


class ConexSafetyTests(unittest.TestCase):
    def make_controller(self, fake, **overrides):
        with patch.object(conex_module.serial, "Serial", return_value=fake):
            return ConexAGAPController(config(**overrides))

    def assert_no_motion_command(self, fake):
        motion_prefixes = ("1PA", "1PR", "1XR", "1JA", "1MM1")
        self.assertFalse(
            any(command.startswith(motion_prefixes) for command in fake.commands),
            fake.commands,
        )

    def test_constructor_blocks_out_of_limit_encoder_without_motion(self):
        fake = FakeConexSerial(position_v=-1.477)
        with self.assertRaisesRegex(MotionFault, "V=-1.477000"):
            self.make_controller(fake)
        self.assertTrue(fake.closed)
        self.assert_no_motion_command(fake)

    def test_move_requires_operator_motion_interlock(self):
        fake = FakeConexSerial()
        controller = self.make_controller(fake)
        controller.home()
        with self.assertRaisesRegex(MotionFault, "motion_enabled"):
            controller.move_to(1.0, 1.0)
        self.assert_no_motion_command(fake)

    def test_move_requires_confirmed_calibration(self):
        fake = FakeConexSerial()
        controller = self.make_controller(fake, motion_enabled=True)
        controller.home()
        with self.assertRaisesRegex(MotionFault, "calibration_confirmed"):
            controller.move_to(1.0, 1.0)
        self.assert_no_motion_command(fake)

    def test_both_targets_are_validated_before_either_axis_moves(self):
        fake = FakeConexSerial()
        controller = self.make_controller(
            fake,
            motion_enabled=True,
            calibration_confirmed=True,
            steps_per_mm_x=1.0,
            steps_per_mm_y=0.01,
        )
        controller.home()
        with self.assertRaisesRegex(MotionFault, "outside stored limits"):
            controller.move_to(2.0, 0.0)
        self.assert_no_motion_command(fake)

    def test_in_limit_confirmed_move_sends_both_targets(self):
        fake = FakeConexSerial()
        controller = self.make_controller(
            fake, motion_enabled=True, calibration_confirmed=True
        )
        controller.home()
        controller.move_to(10.0, 20.0)
        self.assertIn("1PAU0.100000", fake.commands)
        self.assertIn("1PAV0.200000", fake.commands)


class ConexRecoveryTests(unittest.TestCase):
    def test_one_recovery_invocation_sends_exactly_one_bounded_batch(self):
        fake = FakeRecoverySerial()
        probe = {
            "position_v_deg": -1.477,
            "negative_limit_v_deg": -0.76,
            "positive_limit_v_deg": 0.76,
        }
        with patch.object(recovery_module.serial, "Serial", return_value=fake):
            before, after = recovery_module._execute_trial(
                "COM4", 1, 0.1, "V", 5, 10, probe
            )
        self.assertEqual(before, -1.477)
        self.assertEqual(after, -1.476)
        self.assertEqual(fake.commands.count("1XRV10"), 1)
        self.assertIn("1XUV5", fake.commands)
        self.assertEqual(fake.commands[-1], "1XUV35")


if __name__ == "__main__":
    unittest.main()
