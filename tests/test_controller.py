from pathlib import Path
from dataclasses import replace
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from dual_dog.config import load_application_name, load_config
from dual_dog.control.controller import (
    FleetController,
    RECONNECT_WATCHDOG_INTERVAL_SECONDS,
    RobotWorker,
    one_shot_recovery_seconds,
    post_stunt_yaw_command,
)
from dual_dog.devices.drivers import SimDriver
from dual_dog.models import Command


ROOT = Path(__file__).resolve().parents[1]


class PostStuntYawCommandTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _, _, cls.safety = load_config(ROOT / "config" / "robots.json")

    def test_large_error_uses_fast_safe_yaw_limit(self):
        command, stage = post_stunt_yaw_command(
            0.5, 0.3, 0, self.safety
        )
        self.assertEqual(command, self.safety.max_yaw)
        self.assertEqual(stage, "快速拉近")

    def test_small_error_uses_sparse_minimum_effective_pulses(self):
        error = self.safety.feedback_deadband_rad + 0.01
        active, active_stage = post_stunt_yaw_command(
            error, 0.22, 0, self.safety
        )
        pause, pause_stage = post_stunt_yaw_command(
            error, 0.22, 1, self.safety
        )
        self.assertEqual(active, self.safety.min_yaw_actuation)
        self.assertEqual(active_stage, "脉冲微调")
        self.assertEqual(pause, 0.0)
        self.assertEqual(pause_stage, "微调停顿")

    def test_deadband_outputs_zero(self):
        command, stage = post_stunt_yaw_command(
            self.safety.feedback_deadband_rad / 2.0,
            self.safety.min_yaw_actuation,
            0,
            self.safety,
        )
        self.assertEqual(command, 0.0)
        self.assertEqual(stage, "稳定确认")

    def test_gait_commands_do_not_add_extra_recovery_lock(self):
        for command in (
            Command.PRONK,
            Command.PACE,
            Command.BOUND,
            Command.BIPEDAL,
            Command.INV_BIPEDAL,
            Command.RUN,
        ):
            with self.subTest(command=command):
                self.assertEqual(one_shot_recovery_seconds(command, 3000), 0.0)

    def test_flip_and_jump_commands_keep_recovery_lock(self):
        for command in (
            Command.LEFT_FLIP,
            Command.RIGHT_FLIP,
            Command.UP_JUMP,
            Command.FRONT_JUMP,
            Command.BACK_JUMP,
            Command.LEFT_JUMP,
            Command.RIGHT_JUMP,
            Command.UPRIGHT,
        ):
            with self.subTest(command=command):
                self.assertEqual(one_shot_recovery_seconds(command, 3000), 3.0)


class CountingSimDriver(SimDriver):
    def __init__(self, config):
        super().__init__(config)
        self.calls = []
        self.command_log = []

    def command(self, command, params):
        self.calls.append(command)
        self.command_log.append((command, time.monotonic()))
        super().command(command, params)


class StuckMotionStateSimDriver(CountingSimDriver):
    """Emulate telemetry that never reports Motion after a stunt."""

    def telemetry(self):
        data = super().telemetry()
        if data.connected:
            data.motion_state = 0
        return data


class FailingConnectDriver(SimDriver):
    def connect(self):
        raise TimeoutError("连接超时")


class FirstTelemetryFailureDriver(SimDriver):
    """Fail only the first status read after a successful connection."""

    def __init__(self, config):
        super().__init__(config)
        self.fail_next_connected_read = True

    def telemetry(self):
        if self._data.connected and self.fail_next_connected_read:
            self.fail_next_connected_read = False
            raise RuntimeError("首帧状态暂不可用")
        return super().telemetry()


class FinalPostureRecordingDriver(SimDriver):
    def __init__(self, config):
        super().__init__(config)
        self.disconnect_posture = None

    def disconnect(self, final_posture=None):
        self.disconnect_posture = final_posture
        super().disconnect(final_posture)


class LowBatterySimDriver(SimDriver):
    def telemetry(self):
        data = super().telemetry()
        if data.connected:
            data.battery = 5
        return data


class HorizontalYawDriftSimDriver(SimDriver):
    def __init__(self, config):
        super().__init__(config)
        self.velocity_commands = []

    def command(self, command, params):
        if command is Command.VELOCITY:
            self.velocity_commands.append(dict(params))
        return super().command(command, params)

    def telemetry(self):
        data = super().telemetry()
        if data.connected:
            data.rpy = (0.0, 0.0, 0.10)
        return data


class HeadingRecordingSimDriver(SimDriver):
    def __init__(self, config):
        super().__init__(config)
        self.velocity_commands = []

    def command(self, command, params):
        if command is Command.VELOCITY:
            self.velocity_commands.append(dict(params))
        return super().command(command, params)


class LeftDriftSimDriver(SimDriver):
    """Inject a repeatable leftward velocity bias into straight motion."""

    def __init__(self, config):
        super().__init__(config)
        self.requested_lateral = []

    def command(self, command, params):
        if command is Command.VELOCITY:
            self.requested_lateral.append(float(params["y"]))
            actual = dict(params)
            if abs(actual["x"]) > 1e-6:
                actual["y"] += 0.06
            return super().command(command, actual)
        return super().command(command, params)


class RecordingRobotWorker:
    def __init__(self):
        self.calls = []

    def submit_connect(self, delay_seconds=0.0):
        self.calls.append(("connect", delay_seconds))

    def cancel_pending_connect(self):
        self.calls.append(("cancel",))

    def submit(self, command, **params):
        self.calls.append(("submit", command, params))


class ConnectCountingSimDriver(SimDriver):
    def __init__(self, config):
        super().__init__(config)
        self.connect_count = 0

    def connect(self):
        self.connect_count += 1
        super().connect()


class ReconnectRecordingDriver:
    def __init__(self, config):
        self.config = config
        self.connection_requested = False
        self.calls = []

    def disconnect(self, final_posture=None):
        self.calls.append("disconnect")

    def connect(self):
        self.calls.append("connect")
        self.connection_requested = True


class ReconnectWatchdogTarget:
    def __init__(self, driver):
        self.driver = driver
        self._close_requested = SimpleNamespace(is_set=lambda: False)
        self._disconnected_since = None
        self._next_reconnect_at = 0.0
        self._reconnect_interval_seconds = 1.0

    def _reconnect_jitter_seconds(self):
        return 0.0


def wait_until(predicate, timeout=1.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class DualDogTests(unittest.TestCase):
    def setUp(self):
        self.mode, self.robots, self.safety = load_config(ROOT / "config" / "robots.json")
        self.fleet = FleetController(self.robots, self.safety, SimDriver)

    def tearDown(self):
        self.fleet.close()

    def test_demo_config_has_twenty_unique_robots_in_sim_mode(self):
        self.assertEqual("sim", self.mode)
        self.assertTrue(load_application_name(ROOT / "config" / "robots.json").strip())
        self.assertEqual(20, len(self.robots))
        self.assertEqual(20, len({robot.robot_id for robot in self.robots}))
        self.assertEqual(20, len({robot.ip for robot in self.robots}))
        self.assertEqual(20, len({robot.state_port for robot in self.robots}))
        self.assertEqual(20, len({robot.session_id for robot in self.robots}))
        self.assertEqual("dog01", self.robots[0].robot_id)
        self.assertEqual("dog02", self.robots[1].robot_id)
        self.assertTrue(all(robot.ip.startswith("192.0.2.") for robot in self.robots))
        self.assertTrue(all(not robot.mac for robot in self.robots))

    def test_critical_battery_only_warns_and_does_not_block_motion(self):
        fleet = FleetController(
            [self.robots[0]], self.safety, LowBatterySimDriver
        )
        try:
            robot_id = self.robots[0].robot_id
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.submit(
                [robot_id], Command.VELOCITY, x=0.3, y=0.0, yaw=0.0
            )
            self.assertTrue(
                wait_until(lambda: "x=0.30" in fleet.snapshots()[0].action)
            )
            self.assertIn("仅报警", fleet.snapshots()[0].protection)
        finally:
            fleet.close()

    def test_horizontal_move_keeps_lateral_speed_and_corrects_yaw(self):
        created = []

        def factory(config):
            driver = HorizontalYawDriftSimDriver(config)
            created.append(driver)
            return driver

        fleet = FleetController([self.robots[0]], self.safety, factory)
        try:
            robot_id = self.robots[0].robot_id
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.submit(
                [robot_id],
                Command.VELOCITY,
                x=0.0,
                y=0.25,
                yaw=0.0,
                target_yaw=0.0,
            )
            self.assertTrue(wait_until(lambda: bool(created[0].velocity_commands)))
            command = created[0].velocity_commands[-1]
            self.assertEqual(command["y"], 0.25)
            self.assertLess(command["yaw"], 0.0)
        finally:
            fleet.close()

    def test_ten_robot_template_is_valid(self):
        mode, robots, _ = load_config(ROOT / "config" / "robots.10.template.json")
        self.assertEqual("sim", mode)
        self.assertEqual(10, len(robots))
        self.assertEqual(10, len({robot.robot_id for robot in robots}))
        self.assertEqual(10, len({robot.ip for robot in robots}))
        self.assertEqual(10, len({robot.state_port for robot in robots}))
        self.assertEqual(10, len({robot.session_id for robot in robots}))

    def test_connect_and_control_both(self):
        ids = [r.robot_id for r in self.robots]
        self.fleet.connect(ids)
        self.assertTrue(wait_until(lambda: all(s.connected for s in self.fleet.snapshots())))
        self.fleet.submit(ids, Command.STAND)
        self.assertTrue(wait_until(lambda: all(s.action == "stand" for s in self.fleet.snapshots())))

    def test_fleet_connect_uses_staggered_connect_requests(self):
        robots = self.robots[:3]
        fleet = FleetController([], self.safety, SimDriver)
        workers = {
            robot.robot_id: RecordingRobotWorker() for robot in robots
        }
        fleet.robots = workers
        fleet.connect_stagger_seconds = 0.6

        fleet.connect([robot.robot_id for robot in robots])

        self.assertEqual(workers["dog01"].calls, [("connect", 0.0)])
        self.assertEqual(workers["dog02"].calls, [("connect", 0.6)])
        self.assertEqual(workers["dog03"].calls, [("connect", 1.2)])

    def test_fleet_disconnect_cancels_pending_connect_request(self):
        fleet = FleetController([], self.safety, SimDriver)
        worker = RecordingRobotWorker()
        fleet.robots = {"dog01": worker}
        fleet.connect_stagger_seconds = 0.6

        fleet.connect(["dog01", "dog01"])
        fleet.disconnect(["dog01"])

        self.assertEqual(
            worker.calls,
            [
                ("connect", 0.0),
                ("cancel",),
                ("submit", "disconnect", {}),
            ],
        )

    def test_worker_ignores_cancelled_delayed_connect(self):
        driver = ConnectCountingSimDriver(self.robots[0])
        worker = RobotWorker(driver, self.safety)
        try:
            worker.submit_connect(0.2)
            worker.cancel_pending_connect()
            time.sleep(0.3)
            self.assertEqual(0, driver.connect_count)

            worker.submit_connect(0.0)
            self.assertTrue(wait_until(lambda: driver._data.connected))
            self.assertEqual(1, driver.connect_count)
        finally:
            worker.close()

    def test_worker_reconnects_after_prolonged_disconnect(self):
        driver = ReconnectRecordingDriver(self.robots[0])
        driver.connection_requested = True
        target = ReconnectWatchdogTarget(driver)
        started = time.monotonic()

        with patch(
            "dual_dog.control.controller.RECONNECT_WATCHDOG_JITTER_SECONDS_PER_ROBOT",
            0.0,
        ):
            RobotWorker._maybe_reconnect(target, started, False)
            self.assertEqual([], driver.calls)

            RobotWorker._maybe_reconnect(target, started + 0.5, False)
            self.assertEqual([], driver.calls)

            RobotWorker._maybe_reconnect(target, started + 1.2, False)
            self.assertEqual(["disconnect", "connect"], driver.calls)
            self.assertEqual(2.0, target._reconnect_interval_seconds)
            self.assertEqual(started + 1.2, target._disconnected_since)
            self.assertEqual(
                started + 1.2 + 2.0,
                target._next_reconnect_at,
            )

            RobotWorker._maybe_reconnect(target, started + 1.3, True)
            self.assertIsNone(target._disconnected_since)
            self.assertEqual(0.0, target._next_reconnect_at)
            self.assertEqual(
                RECONNECT_WATCHDOG_INTERVAL_SECONDS,
                target._reconnect_interval_seconds,
            )

    def test_close_requests_sit_before_disconnecting(self):
        created = []

        def factory(config):
            driver = FinalPostureRecordingDriver(config)
            created.append(driver)
            return driver

        fleet = FleetController([self.robots[0]], self.safety, factory)
        try:
            robot_id = self.robots[0].robot_id
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.close(final_posture=Command.SIT)
        finally:
            fleet.close()
        self.assertIs(created[0].disconnect_posture, Command.SIT)
        self.assertEqual(created[0]._data.motion_state, 3)
        self.assertFalse(created[0]._data.connected)

    def test_controller_scales_to_ten_robots(self):
        robots = [
            replace(
                self.robots[0],
                robot_id=f"dog{index:02d}",
                name=f"机器狗 {index}",
                ip=f"192.0.2.{index}",
                state_port=9900 + index,
                session_id=index,
            )
            for index in range(1, 11)
        ]
        fleet = FleetController(robots, self.safety, SimDriver)
        try:
            ids = [robot.robot_id for robot in robots]
            fleet.connect(ids)
            self.assertTrue(
                wait_until(lambda: len(fleet.snapshots()) == 10 and all(
                    snapshot.connected for snapshot in fleet.snapshots()
                ))
            )
            fleet.submit(ids, Command.STAND)
            self.assertTrue(
                wait_until(lambda: all(
                    snapshot.action == "stand" for snapshot in fleet.snapshots()
                ))
            )
        finally:
            fleet.close()

    def test_connection_error_remains_visible(self):
        fleet = FleetController([self.robots[0]], self.safety, FailingConnectDriver)
        try:
            fleet.connect([self.robots[0].robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].error == "连接超时"))
            time.sleep(0.15)
            self.assertEqual("连接超时", fleet.snapshots()[0].error)
        finally:
            fleet.close()

    def test_first_telemetry_failure_does_not_leave_connected_robot_offline(self):
        fleet = FleetController(
            [self.robots[0]], self.safety, FirstTelemetryFailureDriver
        )
        try:
            fleet.connect([self.robots[0].robot_id])
            self.assertTrue(
                wait_until(
                    lambda: fleet.snapshots()[0].connected
                    and fleet.snapshots()[0].error is None
                )
            )
        finally:
            fleet.close()

    def test_velocity_is_clamped(self):
        ids = [self.robots[0].robot_id]
        self.fleet.connect(ids)
        self.assertTrue(wait_until(lambda: self.fleet.snapshots()[0].connected))
        self.fleet.submit(ids, Command.VELOCITY, x=99, y=-99, yaw=99)
        expected_x = f"x={self.safety.max_x:.2f}"
        expected_y = f"y={-self.safety.max_y:.2f}"
        expected_yaw = f"yaw={self.safety.max_yaw:.2f}"
        self.assertTrue(wait_until(lambda: expected_x in self.fleet.snapshots()[0].action))
        action = self.fleet.snapshots()[0].action
        self.assertIn(expected_y, action)
        self.assertIn(expected_yaw, action)

    def test_manual_turn_keeps_direction_with_speed_sync_and_ignores_old_target(self):
        ids = [self.robots[0].robot_id]
        self.fleet.connect(ids)
        self.assertTrue(wait_until(lambda: self.fleet.snapshots()[0].connected))
        self.fleet.submit(
            ids,
            Command.VELOCITY,
            x=0.0,
            y=0.0,
            yaw=0.25,
            target_yaw=-1.0,
            path_correction=True,
            speed_sync=True,
        )
        self.assertTrue(
            wait_until(
                lambda: self.fleet.snapshots()[0].commanded_velocity is not None
                and self.fleet.snapshots()[0].commanded_velocity[2] > 0.0
                and "最高优先级" in self.fleet.snapshots()[0].feedback
            )
        )

    def test_lateral_feedback_opposes_simulated_left_drift(self):
        created = []

        def factory(config):
            driver = LeftDriftSimDriver(config)
            created.append(driver)
            return driver

        robot = self.robots[0]
        fleet = FleetController([robot], self.safety, factory)
        try:
            fleet.connect([robot.robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            for _ in range(24):
                fleet.submit(
                    [robot.robot_id],
                    Command.VELOCITY,
                    x=0.3,
                    y=0.0,
                    yaw=0.0,
                    lateral_correction=True,
                )
                time.sleep(0.05)
            self.assertTrue(any(value < -0.005 for value in created[0].requested_lateral))
            self.assertIn("轨迹[补偿]", fleet.snapshots()[0].feedback)
        finally:
            fleet.close()

    def test_online_adaptive_slip_learns_per_robot_cross_axis_trim(self):
        created = []

        def factory(config):
            driver = LeftDriftSimDriver(config)
            created.append(driver)
            return driver

        robot = self.robots[0]
        safety = replace(
            self.safety,
            adaptive_slip_warmup_s=0.10,
            adaptive_slip_learning_rate=1.0,
        )
        fleet = FleetController([robot], safety, factory)
        try:
            fleet.connect([robot.robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            for _ in range(30):
                fleet.submit(
                    [robot.robot_id],
                    Command.VELOCITY,
                    x=0.3,
                    y=0.0,
                    yaw=0.0,
                    adaptive_slip=True,
                )
                time.sleep(0.05)
            self.assertTrue(any(value < -0.01 for value in created[0].requested_lateral))
            snapshot = fleet.snapshots()[0]
            self.assertEqual(snapshot.adaptive_slip_direction, "forward")
            self.assertLess(snapshot.adaptive_slip_bias, 0.0)
            self.assertIn("侧滑自适应", snapshot.feedback)
        finally:
            fleet.close()

    def test_pure_lateral_velocity_has_effective_start_and_hard_limit(self):
        self.assertGreaterEqual(self.safety.min_lateral_actuation, 0.25)
        self.assertLessEqual(self.safety.min_lateral_actuation, self.safety.max_y)
        self.assertLessEqual(self.safety.max_y, 0.50)
        ids = [self.robots[0].robot_id]
        self.fleet.connect(ids)
        self.assertTrue(wait_until(lambda: self.fleet.snapshots()[0].connected))

        self.fleet.submit(ids, Command.VELOCITY, x=0, y=0.01, yaw=0)
        effective = f"y={self.safety.min_lateral_actuation:.2f}"
        self.assertTrue(wait_until(lambda: effective in self.fleet.snapshots()[0].action))

        self.fleet.submit(ids, Command.VELOCITY, x=0, y=10, yaw=0)
        maximum = f"y={self.safety.max_y:.2f}"
        self.assertTrue(wait_until(lambda: maximum in self.fleet.snapshots()[0].action))

    def test_command_watchdog_stops_motion(self):
        ids = [self.robots[0].robot_id]
        self.fleet.connect(ids)
        self.assertTrue(wait_until(lambda: self.fleet.snapshots()[0].connected))
        self.fleet.submit(ids, Command.VELOCITY, x=0.2, y=0, yaw=0)
        self.assertTrue(wait_until(lambda: "速度" in self.fleet.snapshots()[0].action))
        self.assertTrue(
            wait_until(lambda: self.fleet.snapshots()[0].protection == "指令超时，已停止")
        )

    def test_one_shot_trick_executes_once_then_returns_to_walk(self):
        created = []

        def factory(config):
            driver = CountingSimDriver(config)
            created.append(driver)
            return driver

        safety = replace(self.safety, one_shot_return_ms=500)
        fleet = FleetController([self.robots[0]], safety, factory)
        try:
            robot_id = self.robots[0].robot_id
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            submitted_at = time.monotonic()
            fleet.submit([robot_id], Command.LEFT_FLIP)
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].action == "left_flip"))
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].action == "walk", timeout=2.0))
            time.sleep(0.5)
            self.assertEqual(created[0].calls.count(Command.LEFT_FLIP), 1)
            self.assertEqual(created[0].calls.count(Command.WALK), 1)
            flip_at = next(
                timestamp
                for command, timestamp in created[0].command_log
                if command is Command.LEFT_FLIP
            )
            walk_at = next(
                timestamp
                for command, timestamp in created[0].command_log
                if command is Command.WALK
            )
            self.assertGreaterEqual(flip_at - submitted_at, 0.20)
            self.assertGreaterEqual(walk_at - flip_at, 0.95)
        finally:
            fleet.close()

    def test_stale_motion_state_cannot_hold_stunt_lock_forever(self):
        created = []

        def factory(config):
            driver = StuckMotionStateSimDriver(config)
            created.append(driver)
            return driver

        safety = replace(self.safety, one_shot_return_ms=500)
        with patch(
            "dual_dog.control.controller.ONE_SHOT_MOTION_RECOVERY_GRACE_SECONDS",
            0.10,
        ):
            fleet = FleetController([self.robots[0]], safety, factory)
            try:
                robot_id = self.robots[0].robot_id
                fleet.connect([robot_id])
                self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
                fleet.submit(
                    [robot_id], Command.LEFT_FLIP, return_to_start=True
                )
                self.assertTrue(
                    wait_until(lambda: Command.LEFT_FLIP in created[0].calls)
                )
                self.assertTrue(
                    wait_until(
                        lambda: fleet.robots[robot_id]._one_shot_unlock_at is None,
                        timeout=2.5,
                    )
                )
                fleet.submit(
                    [robot_id], Command.RIGHT_FLIP, return_to_start=True
                )
                self.assertTrue(
                    wait_until(lambda: Command.RIGHT_FLIP in created[0].calls)
                )
                self.assertEqual(created[0].calls.count(Command.RIGHT_FLIP), 1)
            finally:
                fleet.close()

    def test_one_shot_uses_imu_to_restore_explicit_audience_heading(self):
        created = []

        def factory(config):
            driver = HeadingRecordingSimDriver(config)
            created.append(driver)
            return driver

        safety = replace(self.safety, one_shot_return_ms=50)
        fleet = FleetController([self.robots[0]], safety, factory)
        try:
            robot_id = self.robots[0].robot_id
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.submit(
                [robot_id],
                Command.PRONK,
                duration_s=0.5,
                target_yaw=0.35,
            )
            self.assertTrue(
                wait_until(
                    lambda: any(item["yaw"] > 0.0 for item in created[0].velocity_commands),
                    timeout=2.0,
                )
            )
            self.assertTrue(
                wait_until(
                    lambda: (
                        abs(fleet.snapshots()[0].rpy[2] - 0.35)
                        <= self.safety.feedback_deadband_rad
                    ),
                    timeout=7.0,
                )
            )
            self.assertAlmostEqual(
                fleet.snapshots()[0].rpy[2],
                0.35,
                delta=self.safety.feedback_deadband_rad,
            )
            self.assertTrue(
                all(
                    abs(item["x"]) < 1e-9 and abs(item["y"]) < 1e-9
                    for item in created[0].velocity_commands
                )
            )
        finally:
            fleet.close()

    def test_non_flip_action_is_held_for_requested_seconds(self):
        created = []

        def factory(config):
            driver = CountingSimDriver(config)
            created.append(driver)
            return driver

        safety = replace(self.safety, one_shot_return_ms=500)
        fleet = FleetController([self.robots[0]], safety, factory)
        try:
            robot_id = self.robots[0].robot_id
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.submit([robot_id], Command.PRONK, duration_s=0.5)
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].action == "pronk"))
            time.sleep(0.2)
            self.assertEqual(created[0].calls.count(Command.WALK), 0)
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].action == "walk"))
            self.assertEqual(created[0].calls.count(Command.PRONK), 1)
            self.assertEqual(created[0].calls.count(Command.WALK), 1)
        finally:
            fleet.close()

    def test_sdk_jump_runs_as_one_shot_and_returns_walk(self):
        created = []

        def factory(config):
            driver = CountingSimDriver(config)
            created.append(driver)
            return driver

        safety = replace(self.safety, one_shot_return_ms=500)
        fleet = FleetController([self.robots[0]], safety, factory)
        try:
            robot_id = self.robots[0].robot_id
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.submit([robot_id], Command.UP_JUMP, duration_s=0.5)
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].action == "up_jump"))
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].action == "walk"))
            self.assertEqual(created[0].calls.count(Command.UP_JUMP), 1)
            self.assertEqual(created[0].calls.count(Command.WALK), 1)
        finally:
            fleet.close()

    def test_discrete_jump_ignores_long_requested_duration(self):
        created = []

        def factory(config):
            driver = CountingSimDriver(config)
            created.append(driver)
            return driver

        safety = replace(self.safety, one_shot_return_ms=500)
        fleet = FleetController([self.robots[0]], safety, factory)
        try:
            robot_id = self.robots[0].robot_id
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.submit([robot_id], Command.UP_JUMP, duration_s=5.0)
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].action == "up_jump"))
            self.assertTrue(
                wait_until(
                    lambda: fleet.snapshots()[0].action == "walk",
                    timeout=2.5,
                )
            )
            self.assertEqual(created[0].calls.count(Command.UP_JUMP), 1)
            self.assertEqual(created[0].calls.count(Command.WALK), 1)
        finally:
            fleet.close()

    def test_failed_return_preparation_never_suppresses_flip_command(self):
        created = []

        def factory(config):
            driver = CountingSimDriver(config)
            created.append(driver)
            return driver

        fleet = FleetController([self.robots[0]], self.safety, factory)
        try:
            robot_id = self.robots[0].robot_id
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.submit(
                [robot_id],
                Command.LEFT_FLIP,
                return_to_start=True,
                return_latency_ms="invalid",
            )
            self.assertTrue(
                wait_until(lambda: Command.LEFT_FLIP in created[0].calls)
            )
            self.assertEqual(created[0].calls.count(Command.LEFT_FLIP), 1)
        finally:
            fleet.close()


if __name__ == "__main__":
    unittest.main()
