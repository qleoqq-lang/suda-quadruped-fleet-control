import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from dual_dog.devices.drivers import (
    BPX_VELOCITY_FLAG_FLUSH_SECONDS,
    BPX_WALK_FLUSH_SECONDS,
    BPX_ZERO_VELOCITY_FLUSH_SECONDS,
    _bpx_process_main,
)
from dual_dog.models import Command, RobotConfig


class FakeConnection:
    def __init__(self, operations):
        self.operations = list(operations)
        self.responses = []

    def recv(self):
        if not self.operations:
            raise EOFError
        return self.operations.pop(0)

    def send(self, response):
        self.responses.append(response)

    def close(self):
        pass


class RecordingMotion:
    def __init__(self):
        self.events = []

    def isConnected(self):
        return True

    def setVelocityControlFlag(self, enabled):
        self.events.append(("velocity_flag", bool(enabled)))

    def setVelocity(self, x, y, yaw):
        self.events.append(("velocity", float(x), float(y), float(yaw)))
        return True

    def setWalk(self):
        self.events.append(("walk",))
        return True

    def setStandUp(self):
        self.events.append(("stand",))
        return True

    def setSitDown(self):
        self.events.append(("sit",))
        return True

    def disconnect(self):
        self.events.append(("disconnect",))

    def setLeftFlip(self):
        self.events.append(("left_flip",))
        return True

    def setRightFlip(self):
        self.events.append(("right_flip",))
        return True

    def setUpJump(self):
        self.events.append(("up_jump",))
        return True

    def setFrontJump(self):
        self.events.append(("front_jump",))
        return True

    def setBackJump(self):
        self.events.append(("back_jump",))
        return True

    def setLeftJump(self):
        self.events.append(("left_jump",))
        return True

    def setRightJump(self):
        self.events.append(("right_jump",))
        return True

    def setUpright(self):
        self.events.append(("upright",))
        return True

    def getCurrentMotionState(self):
        return 6

    def getCurrentGait(self):
        return 0

    def getSubGait(self):
        return 0


class BpxDriverProtocolTests(unittest.TestCase):
    def test_hold_stand_flushes_zero_before_standing_and_disconnect(self):
        motion = RecordingMotion()
        connection = FakeConnection([("hold_stand", ()), ("disconnect", ())])
        config = RobotConfig("dog01", "dog1", "192.0.2.1", "", 9901, 1)
        with patch.dict(sys.modules, {"bpx_sdk": SimpleNamespace(MotionLevelControl=lambda: motion)}), patch("dual_dog.devices.drivers.time.sleep"):
            _bpx_process_main(connection, config)
        self.assertEqual(motion.events, [
            ("velocity_flag", True), ("velocity", 0.0, 0.0, 0.0),
            ("velocity_flag", False), ("stand",), ("disconnect",),
        ])
        self.assertTrue(all(ok for ok, _ in connection.responses))

    def test_hold_sit_flushes_zero_before_sitting_and_disconnect(self):
        motion = RecordingMotion()
        connection = FakeConnection([("hold_sit", ()), ("disconnect", ())])
        config = RobotConfig("dog01", "dog1", "192.0.2.1", "", 9901, 1)
        with patch.dict(sys.modules, {"bpx_sdk": SimpleNamespace(MotionLevelControl=lambda: motion)}), patch("dual_dog.devices.drivers.time.sleep"):
            _bpx_process_main(connection, config)
        self.assertEqual(motion.events, [
            ("velocity_flag", True), ("velocity", 0.0, 0.0, 0.0),
            ("velocity_flag", False), ("sit",), ("disconnect",),
        ])
        self.assertTrue(all(ok for ok, _ in connection.responses))

    def test_sdk_jump_and_upright_commands_reach_native_sdk(self):
        commands = (
            Command.UP_JUMP,
            Command.FRONT_JUMP,
            Command.BACK_JUMP,
            Command.LEFT_JUMP,
            Command.RIGHT_JUMP,
            Command.UPRIGHT,
        )
        motion = RecordingMotion()
        connection = FakeConnection(
            [("command", (command.value, {})) for command in commands]
        )
        config = RobotConfig("dog01", "dog1", "192.0.2.1", "", 9901, 1)
        with patch.dict(sys.modules, {"bpx_sdk": SimpleNamespace(MotionLevelControl=lambda: motion)}):
            _bpx_process_main(connection, config)
        self.assertEqual(
            motion.events,
            [("up_jump",), ("front_jump",), ("back_jump",),
             ("left_jump",), ("right_jump",), ("upright",)],
        )
        self.assertTrue(all(ok for ok, _ in connection.responses))

    def test_stop_fully_exits_velocity_mode_before_flip(self):
        motion = RecordingMotion()
        connection = FakeConnection([
            ("command", (Command.STOP.value, {})),
            ("command", (Command.LEFT_FLIP.value, {})),
        ])
        config = RobotConfig(
            "dog01", "机器狗1", "192.168.31.205", "00:00:00:00:00:01",
            20001, 1,
        )
        fake_sdk = SimpleNamespace(MotionLevelControl=lambda: motion)
        with (
            patch.dict(sys.modules, {"bpx_sdk": fake_sdk}),
            patch("dual_dog.devices.drivers.time.sleep") as sleep,
        ):
            _bpx_process_main(connection, config)

        self.assertEqual(motion.events, [
            ("velocity_flag", True),
            ("velocity", 0.0, 0.0, 0.0),
            ("walk",),
            ("velocity_flag", False),
            ("left_flip",),
        ])
        sleep.assert_called_once_with(0.5)
        self.assertEqual(connection.responses, [(True, True), (True, True)])

    def test_safe_stop_exits_gait_through_walk(self):
        motion = RecordingMotion()
        connection = FakeConnection([("safe_stop", ())])
        config = RobotConfig(
            "dog01", "机器狗1", "192.168.31.205", "00:00:00:00:00:01",
            20001, 1,
        )
        fake_sdk = SimpleNamespace(MotionLevelControl=lambda: motion)
        with (
            patch.dict(sys.modules, {"bpx_sdk": fake_sdk}),
            patch("dual_dog.devices.drivers.time.sleep") as sleep,
        ):
            _bpx_process_main(connection, config)

        self.assertEqual(motion.events, [
            ("velocity_flag", True),
            ("velocity", 0.0, 0.0, 0.0),
            ("walk",),
            ("velocity_flag", False),
        ])
        sleep.assert_called_once_with(0.5)
        self.assertEqual(connection.responses, [(True, None)])

    def test_recover_walk_flushes_bipedal_exit(self):
        motion = RecordingMotion()
        connection = FakeConnection([("recover_walk", ())])
        config = RobotConfig(
            "dog01", "机器狗1", "192.168.31.205", "00:00:00:00:00:01",
            20001, 1,
        )
        fake_sdk = SimpleNamespace(MotionLevelControl=lambda: motion)
        with (
            patch.dict(sys.modules, {"bpx_sdk": fake_sdk}),
            patch("dual_dog.devices.drivers.time.sleep") as sleep,
        ):
            _bpx_process_main(connection, config)

        self.assertEqual(motion.events, [
            ("velocity_flag", True),
            ("velocity", 0.0, 0.0, 0.0),
            ("walk",),
            ("velocity_flag", False),
        ])
        self.assertEqual(
            [call.args[0] for call in sleep.call_args_list],
            [
                BPX_ZERO_VELOCITY_FLUSH_SECONDS,
                BPX_WALK_FLUSH_SECONDS,
                BPX_VELOCITY_FLAG_FLUSH_SECONDS,
            ],
        )
        self.assertEqual(connection.responses, [(True, True)])

    def test_prepared_flip_is_atomic_and_flushes_walk_and_flag(self):
        motion = RecordingMotion()
        connection = FakeConnection([
            ("prepared_stunt", (Command.RIGHT_FLIP.value,)),
        ])
        config = RobotConfig(
            "dog01", "机器狗1", "192.168.31.205", "00:00:00:00:00:01",
            20001, 1,
        )
        fake_sdk = SimpleNamespace(MotionLevelControl=lambda: motion)
        with (
            patch.dict(sys.modules, {"bpx_sdk": fake_sdk}),
            patch("dual_dog.devices.drivers.time.sleep") as sleep,
        ):
            _bpx_process_main(connection, config)

        self.assertEqual(motion.events, [
            ("velocity_flag", True),
            ("velocity", 0.0, 0.0, 0.0),
            ("walk",),
            ("velocity_flag", False),
            ("right_flip",),
        ])
        self.assertEqual(
            [call.args[0] for call in sleep.call_args_list],
            [
                BPX_ZERO_VELOCITY_FLUSH_SECONDS,
                BPX_WALK_FLUSH_SECONDS,
                BPX_VELOCITY_FLAG_FLUSH_SECONDS,
            ],
        )
        self.assertEqual(connection.responses, [(
            True,
            {
                "accepted": True,
                "before": {"motion_state": 6, "gait": 0, "sub_gait": 0},
                "ready": {"motion_state": 6, "gait": 0, "sub_gait": 0},
                "ready_confirmed": True,
            },
        )])


if __name__ == "__main__":
    unittest.main()
