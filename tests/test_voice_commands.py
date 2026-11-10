from pathlib import Path
import threading
import time
import unittest

from dual_dog.control.controller import FleetController
from dual_dog.devices.drivers import SimDriver
from dual_dog.models import Command
from dual_dog.models import RobotConfig, SafetyConfig
from voice_control.commands import (
    STUNT_COMMANDS,
    normalize,
    parse_targets,
    parse_voice,
    parse_voice_intents,
)
from voice_control.commands import VoiceIntent
from voice_control.robot import execute_intent


ROOT = Path(__file__).resolve().parents[1]
ROBOT_IDS = [f"dog{index:02d}" for index in range(1, 11)]
DEFAULTS = tuple(ROBOT_IDS)


def wait_until(predicate, timeout=1.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class VoiceCommandMappingTests(unittest.TestCase):
    def parse(self, text, **kwargs):
        return parse_voice(text, ROBOT_IDS, DEFAULTS, **kwargs)

    def test_normalize_removes_punctuation(self):
        self.assertEqual(normalize("一号，前进！"), "一号前进")

    def test_forward_velocity(self):
        intent = self.parse("前进")
        self.assertIsNotNone(intent)
        self.assertEqual(intent.command, Command.VELOCITY)
        self.assertGreater(intent.params["x"], 0)
        self.assertEqual(intent.duration_s, 2.0)
        self.assertEqual(intent.targets, DEFAULTS)

    def test_forward_three_seconds_on_dog_one(self):
        intent = self.parse("一号前进3秒")
        self.assertEqual(intent.targets, ("dog01",))
        self.assertEqual(intent.duration_s, 3.0)

    def test_straight_distance_uses_duration_conversion(self):
        intent = self.parse("直行3米")
        self.assertEqual(intent.command, Command.VELOCITY)
        self.assertGreater(intent.params["x"], 0)
        self.assertAlmostEqual(intent.duration_s, 3.0 / 0.3, places=3)

    def test_forward_distance_uses_duration_conversion(self):
        intent = self.parse("前进2米", base_speed=0.5)
        self.assertAlmostEqual(intent.duration_s, 2.0 / 0.5, places=3)

    def test_backward(self):
        intent = self.parse("后退")
        self.assertEqual(intent.command, Command.VELOCITY)
        self.assertLess(intent.params["x"], 0)

    def test_full_formation_macro_expands_to_stand_and_forward(self):
        intents = parse_voice_intents(
            "全军列阵",
            ROBOT_IDS,
            DEFAULTS,
            base_speed=0.3,
            base_yaw_speed=0.4,
        )
        self.assertEqual(len(intents), 2)
        self.assertEqual(intents[0].command, Command.STAND)
        self.assertEqual(intents[0].targets, DEFAULTS)
        self.assertEqual(intents[1].command, Command.VELOCITY)
        self.assertEqual(intents[1].duration_s, 5.0)
        self.assertEqual(intents[1].targets, DEFAULTS)

    def test_full_formation_macro_honors_target(self):
        intents = parse_voice_intents(
            "一号全军列阵",
            ROBOT_IDS,
            DEFAULTS,
        )
        self.assertEqual(intents[0].targets, ("dog01",))
        self.assertEqual(intents[1].targets, ("dog01",))

    def test_turn_directions(self):
        left = self.parse("左转")
        right = self.parse("右转")
        self.assertGreater(left.params["yaw"], 0)
        self.assertLess(right.params["yaw"], 0)

    def test_fast_speed_modifier(self):
        normal = self.parse("前进")
        fast = self.parse("快速前进")
        self.assertGreater(fast.params["x"], normal.params["x"])

    def test_stop_and_emergency(self):
        self.assertEqual(self.parse("停止").command, Command.STOP)
        self.assertEqual(self.parse("急停").command, Command.DAMPING)
        self.assertEqual(self.parse("别动").command, Command.STOP)

    def test_stand_and_sit(self):
        self.assertEqual(self.parse("站起来").command, Command.STAND)
        self.assertEqual(self.parse("趴下").command, Command.SIT)

    def test_walk_gait(self):
        self.assertEqual(self.parse("走路").command, Command.WALK)

    def test_stunt_rules_are_marked(self):
        self.assertTrue(self.parse("左翻").stunt)
        self.assertTrue(self.parse("跑步").stunt)
        self.assertTrue(self.parse("跳跃").stunt)
        self.assertTrue(self.parse("双足倒立").stunt)

    def test_run_and_flips(self):
        self.assertEqual(self.parse("跑步").command, Command.RUN)
        self.assertEqual(self.parse("左翻").command, Command.LEFT_FLIP)
        self.assertEqual(self.parse("右翻").command, Command.RIGHT_FLIP)

    def test_pronk_pace_bound(self):
        self.assertEqual(self.parse("弹跳").command, Command.PRONK)
        self.assertEqual(self.parse("踱步").command, Command.PACE)
        self.assertEqual(self.parse("跳步").command, Command.BOUND)

    def test_bipedal_and_inverse(self):
        self.assertEqual(self.parse("双足正立").command, Command.BIPEDAL)
        self.assertEqual(self.parse("倒立").command, Command.INV_BIPEDAL)

    def test_sdk_jumps_and_upright(self):
        self.assertEqual(self.parse("原地跳跃").command, Command.UP_JUMP)
        self.assertEqual(self.parse("前跳").command, Command.FRONT_JUMP)
        self.assertEqual(self.parse("后跳").command, Command.BACK_JUMP)
        self.assertEqual(self.parse("左跳").command, Command.LEFT_JUMP)
        self.assertEqual(self.parse("右跳").command, Command.RIGHT_JUMP)
        self.assertEqual(self.parse("翻身站起").command, Command.UPRIGHT)

    def test_all_targets(self):
        intent = self.parse("全部趴下")
        self.assertEqual(intent.targets, DEFAULTS)

    def test_all_targets_limited_to_online_subset(self):
        online = ("dog01", "dog03")
        intent = parse_voice(
            "全部趴下",
            ROBOT_IDS,
            online,
            base_speed=0.3,
            base_yaw_speed=0.4,
        )
        self.assertEqual(intent.targets, online)

    def test_multiple_targets(self):
        intent = self.parse("二号和四号一起站立")
        self.assertEqual(intent.targets, ("dog02", "dog04"))

    def test_target_parser_dog_ids(self):
        self.assertEqual(parse_targets("dog1 dog10", ROBOT_IDS), ("dog01", "dog10"))

    def test_unknown_text_returns_none(self):
        self.assertIsNone(self.parse("今天天气不错"))

    def test_stunt_command_set(self):
        self.assertIn(Command.RUN, STUNT_COMMANDS)
        self.assertIn(Command.INV_BIPEDAL, STUNT_COMMANDS)
        self.assertIn(Command.UP_JUMP, STUNT_COMMANDS)
        self.assertIn(Command.UPRIGHT, STUNT_COMMANDS)


class VoiceRobotExecutionTests(unittest.TestCase):
    def setUp(self):
        self.robot = RobotConfig(
            robot_id="dog01",
            name="机器狗1",
            ip="192.0.2.1",
            mac="",
            state_port=9990,
            session_id=1,
        )
        self.fleet = FleetController([self.robot], SafetyConfig(), SimDriver)

    def tearDown(self):
        self.fleet.close()

    def connect(self):
        self.fleet.connect(["dog01"])
        self.assertTrue(wait_until(lambda: self.fleet.snapshots()[0].connected))

    def test_velocity_refreshes_then_stops(self):
        self.connect()
        intent = VoiceIntent(
            targets=("dog01",),
            command=Command.VELOCITY,
            params={"x": 0.2, "y": 0.0, "yaw": 0.0},
            duration_s=0.6,
            label="前进",
        )
        execute_intent(self.fleet, intent)
        self.assertTrue(wait_until(lambda: self.fleet.snapshots()[0].action == "stop"))

    def test_velocity_stop_event_interrupts_early(self):
        self.connect()
        intent = VoiceIntent(
            targets=("dog01",),
            command=Command.VELOCITY,
            params={"x": 0.2, "y": 0.0, "yaw": 0.0},
            duration_s=5.0,
            label="前进",
        )
        stop_event = threading.Event()
        results = []

        def run():
            try:
                execute_intent(self.fleet, intent, stop_event=stop_event)
                results.append("done")
            except Exception as exc:
                results.append(str(exc))

        thread = threading.Thread(target=run)
        thread.start()
        time.sleep(0.4)
        stop_event.set()
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())
        self.assertEqual(results, ["done"])
        self.assertTrue(wait_until(lambda: self.fleet.snapshots()[0].action == "stop"))

    def test_stunt_is_blocked_without_allow_flag(self):
        intent = VoiceIntent(
            targets=("dog01",),
            command=Command.LEFT_FLIP,
            params={},
            label="左翻",
            stunt=True,
        )
        with self.assertRaises(PermissionError):
            execute_intent(self.fleet, intent)

    def test_regular_command_executes_with_flag(self):
        self.connect()
        intent = VoiceIntent(
            targets=("dog01",),
            command=Command.STAND,
            params={},
            label="站立",
        )
        execute_intent(self.fleet, intent, allow_stunts=True)
        self.assertTrue(wait_until(lambda: self.fleet.snapshots()[0].action == "stand"))


if __name__ == "__main__":
    unittest.main()
