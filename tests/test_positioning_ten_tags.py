import json
import os
from dataclasses import replace
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from dual_dog.config import load_config
from dual_dog.control.controller import (
    FleetController,
    TAG_POSITION_TOLERANCE_M,
    braking_speed_limit,
    direction_lock_mask,
)
from dual_dog.devices.drivers import SimDriver
from dual_dog.models import Command
from dual_dog.ui.position_window import PositionWindow
from dual_dog.devices.positioning import (
    MIN_MULTI_TAG_STALE_SECONDS,
    SUPPORTED_TAGS,
    WAYPOINT_TAGS,
    PositionSettings,
    TagLineParser,
    TagSample,
    default_tag_bindings,
)


ROOT = Path(__file__).resolve().parents[1]


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


class RecordingSimDriver(SimDriver):
    def __init__(self, config):
        super().__init__(config)
        self.velocity_commands = []
        self.command_log = []

    def command(self, command, params):
        self.command_log.append((command, dict(params), time.monotonic()))
        if command is Command.VELOCITY:
            self.velocity_commands.append(dict(params))
        return super().command(command, params)


class TenTagParserTests(unittest.TestCase):
    def test_fragmented_and_coalesced_tag1_through_tag10(self):
        parser = TagLineParser()
        first = parser.feed(b"TAG1,1,2\r\nTAG2,3")
        second = parser.feed(
            b",4\n" + b"".join(
                f"TAG{index},{index * 10},{index * -10}\r\n".encode("ascii")
                for index in range(3, 11)
            )
        )
        samples = first + second
        self.assertEqual([sample.tag for sample in samples], list(SUPPORTED_TAGS))
        self.assertEqual(samples[-1], TagSample("TAG10", 100.0, -100.0))
        self.assertEqual(parser.invalid_lines, 0)

    def test_out_of_fleet_tags_are_rejected_not_misbound(self):
        parser = TagLineParser()
        samples = parser.feed(b"TAG0,1,2\nTAG11,3,4\nTAG6,5,6\n")
        self.assertEqual(samples, [TagSample("TAG6", 5.0, 6.0)])
        self.assertEqual(parser.invalid_lines, 2)

    def test_old_tag3_tag4_settings_migrate_to_ten_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "positioning.json"
            path.write_text(
                json.dumps({
                    "unit": "cm",
                    "bindings": {"TAG3": "dog03", "TAG4": "dog04"},
                }),
                encoding="utf-8",
            )
            settings = PositionSettings.load(path)
        self.assertEqual(settings.bindings, default_tag_bindings())
        self.assertEqual(tuple(settings.bindings), SUPPORTED_TAGS)
        self.assertEqual(settings.stale_seconds, MIN_MULTI_TAG_STALE_SECONDS)
        self.assertFalse(settings.heading_confirmed)
        self.assertEqual(tuple(settings.tag_targets), WAYPOINT_TAGS)
        self.assertTrue(all(point is None for point in settings.tag_targets.values()))
        self.assertEqual(settings.direction_locks, {f"dog{i:02d}": "xy" for i in range(1, 11)})

    def test_old_six_point_snapshot_migrates_to_ten_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "positioning.json"
            six_point = {
                f"TAG{index}": [index * 10.0, index * 20.0]
                for index in range(1, 7)
            }
            path.write_text(
                json.dumps({
                    "unit": "cm",
                    "tag_targets": dict(six_point),
                    "waypoint_snapshots": {"默认点位": dict(six_point)},
                }),
                encoding="utf-8",
            )
            settings = PositionSettings.load(path)
        snapshot = settings.waypoint_snapshots["默认点位"]
        self.assertEqual(tuple(settings.tag_targets), WAYPOINT_TAGS)
        self.assertEqual(tuple(snapshot), WAYPOINT_TAGS)
        self.assertIsNone(snapshot["TAG7"])
        self.assertIsNone(snapshot["TAG10"])


class TenTagWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.path = Path(self.temporary.name) / "positioning.json"
        configs = load_config(ROOT / "config" / "robots.json")[1]
        self.window = PositionWindow(configs, self.path)
        self.window.show()
        self.app.processEvents()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.temporary.cleanup()

    def test_ui_reserves_all_ten_tags_and_default_dog_bindings(self):
        self.assertEqual(tuple(self.window.tracks), SUPPORTED_TAGS)
        self.assertEqual(self.window.position_table.rowCount(), 10)
        self.assertEqual(self.window.target_tag_choice.count(), 10)
        self.assertEqual(
            self.window.add_all_dance_button.text(), "十个点位同时加入编舞"
        )
        for index, tag in enumerate(SUPPORTED_TAGS, start=1):
            self.assertEqual(
                self.window.binding_choices[tag].currentData(), f"dog{index:02d}"
            )

    def test_one_serial_batch_updates_dog1_through_dog6_independently(self):
        captured = []
        self.window.sample_processed.connect(captured.append)
        samples = [
            TagSample(f"TAG{index}", float(index), float(index + 100))
            for index in range(1, 7)
        ]
        self.window.receive_samples(samples)
        with patch.object(self.window.reader.port, "isOpen", return_value=True):
            self.window.refresh_display()
        self.assertEqual(len(captured), 6)
        self.assertTrue(all(self.window.tracks[f"TAG{i}"].received == 1 for i in range(1, 7)))
        self.assertEqual(self.window.tracks["TAG7"].received, 0)
        self.assertIn("TAG1", self.window.position_table.item(0, 0).text())
        self.assertIn("实时", self.window.position_table.item(5, 5).text())
        self.assertEqual(self.window.position_table.item(6, 5).text(), "等待数据")

    def test_each_coalesced_frame_is_forwarded_even_when_filter_holds_value(self):
        captured = []
        self.window.sample_processed.connect(captured.append)
        self.window.receive_samples(
            [TagSample("TAG1", 100, 200), TagSample("TAG1", 110, 210)]
        )
        self.assertEqual(len(captured), 2)
        self.assertEqual(captured[0]["raw"], (100, 200))
        self.assertEqual(captured[1]["raw"], (110, 210))

    def test_zero_payload_carries_confirmed_stage_heading(self):
        calibrated = []
        self.window.zero_calibrated.connect(calibrated.append)
        self.window.settings.unit = "cm"
        self.window.settings.xy_confirmed = True
        self.window.settings.heading_confirmed = True
        self.window.settings.zero_heading_deg = 90.0
        direction_choice = self.window.direction_lock_choices["dog01"]
        direction_choice.setCurrentIndex(direction_choice.findData("x"))
        self.window.receive_samples([TagSample("TAG1", 100.0, 200.0)])

        with patch.object(self.window.reader.port, "isOpen", return_value=True):
            self.window.calibrate_tag_zero("TAG1")

        self.assertEqual(len(calibrated), 1)
        self.assertEqual(calibrated[0]["robot_id"], "dog01")
        self.assertEqual(calibrated[0]["stage_heading_deg"], 90.0)
        self.assertEqual(calibrated[0]["return_speed_percent"], 100.0)
        self.assertTrue(calibrated[0]["continuous_return"])
        self.assertEqual(calibrated[0]["return_latency_ms"], 300.0)
        self.assertEqual(calibrated[0]["return_max_seconds"], 4.0)
        self.assertEqual(calibrated[0]["direction_lock"], "x")
        self.assertEqual(calibrated[0]["meters"], (1.0, 2.0))

    def test_map_target_payload_converts_tag1_through_tag10_to_meters(self):
        self.window.settings.unit = "cm"
        self.window.settings.xy_confirmed = True
        self.window.settings.heading_confirmed = True
        for index, tag in enumerate(WAYPOINT_TAGS, start=1):
            self.window.target_points[tag] = (index * 10.0, index * 20.0)
        payload = self.window.target_payload(WAYPOINT_TAGS)
        self.assertIsNotNone(payload)
        self.assertEqual(len(payload["entries"]), 10)
        self.assertEqual(payload["entries"][0]["robot_id"], "dog01")
        self.assertEqual(payload["entries"][0]["meters"], (0.1, 0.2))
        self.assertEqual(payload["entries"][-1]["robot_id"], "dog10")
        self.assertEqual(payload["entries"][-1]["meters"], (1.0, 2.0))

    def test_named_point_snapshots_can_be_saved_loaded_and_persisted(self):
        self.window.settings.unit = "cm"
        self.window.settings.xy_confirmed = True
        self.window.settings.heading_confirmed = True
        expected = {
            tag: (index * 10.0, index * 20.0)
            for index, tag in enumerate(WAYPOINT_TAGS, start=1)
        }
        self.window.target_points = dict(expected)
        self.window.snapshot_name.setText("进场队形")
        self.assertTrue(self.window.save_target_snapshot())
        self.window.target_points["TAG1"] = (300.0, 300.0)
        self.assertTrue(self.window.load_target_snapshot())
        self.assertEqual(self.window.target_points, expected)
        loaded = PositionSettings.load(self.path)
        self.assertEqual(loaded.waypoint_snapshots["进场队形"], expected)


class TagReturnControllerTests(unittest.TestCase):
    def test_direction_lock_mask_maps_xy_x_y(self):
        self.assertEqual(direction_lock_mask("xy"), (True, True))
        self.assertEqual(direction_lock_mask("x"), (True, False))
        self.assertEqual(direction_lock_mask("y"), (False, True))
        self.assertEqual(direction_lock_mask("invalid"), (True, True))

    def test_braking_speed_limit_tapers_smoothly_near_tolerance(self):
        far = braking_speed_limit(0.50)
        medium = braking_speed_limit(0.20)
        near = braking_speed_limit(0.05)
        self.assertGreater(far, medium)
        self.assertGreater(medium, near)
        self.assertGreater(near, 0.0)
        self.assertEqual(braking_speed_limit(0.02), 0.0)

    def test_direction_lock_x_keeps_stage_y_correction_zero(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(
                robot_id,
                1.0,
                1.0,
                12.0,
                stage_heading_deg=0.0,
                direction_lock="x",
            )
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.update_tag_position(robot_id, 0.70, 0.80, time.monotonic())
            fleet.start_tag_position_hold(
                robot_id,
                1.0,
                1.0,
                12.0,
                stage_heading_deg=0.0,
                direction_lock="x",
            )
            self.assertTrue(wait_until(lambda: bool(drivers[0].velocity_commands)))
            command = drivers[0].velocity_commands[-1]
            self.assertGreater(command["x"], 0.0)
            self.assertAlmostEqual(command["y"], 0.0, places=6)
        finally:
            fleet.close()

    def test_direction_lock_y_keeps_stage_x_correction_zero(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(
                robot_id,
                1.0,
                1.0,
                12.0,
                stage_heading_deg=0.0,
                direction_lock="y",
            )
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.update_tag_position(robot_id, 0.80, 0.70, time.monotonic())
            fleet.start_tag_position_hold(
                robot_id,
                1.0,
                1.0,
                12.0,
                stage_heading_deg=0.0,
                direction_lock="y",
            )
            self.assertTrue(wait_until(lambda: bool(drivers[0].velocity_commands)))
            command = drivers[0].velocity_commands[-1]
            self.assertAlmostEqual(command["x"], 0.0, places=6)
            self.assertGreater(command["y"], 0.0)
        finally:
            fleet.close()

    def test_continuous_return_direction_lock_keeps_locked_axis_zero(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(
                robot_id,
                1.0,
                1.0,
                12.0,
                continuous_return=True,
                direction_lock="x",
            )
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.update_tag_position(robot_id, 0.50, 0.80, time.monotonic())
            fleet.start_tag_position_hold(
                robot_id,
                1.0,
                1.0,
                12.0,
                continuous_return=True,
                direction_lock="x",
            )
            self.assertTrue(
                wait_until(lambda: len(drivers[0].velocity_commands) >= 3)
            )
            self.assertTrue(
                all(command["x"] > 0.0 for command in drivers[0].velocity_commands[:3])
            )
            self.assertTrue(
                all(
                    abs(command["y"]) < 1e-6
                    for command in drivers[0].velocity_commands[:3]
                )
            )
        finally:
            fleet.close()

    def test_one_tag_sample_produces_one_bounded_pulse_then_stops(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.update_tag_position(robot_id, 0.70, 1.0, time.monotonic())
            fleet.start_tag_position_hold(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: len(drivers[0].velocity_commands) == 1))
            self.assertTrue(wait_until(lambda: not worker._velocity_active, timeout=0.8))
            time.sleep(0.30)
            self.assertEqual(len(drivers[0].velocity_commands), 1)
            self.assertTrue(
                any(command is Command.STOP for command, _, _ in drivers[0].command_log)
            )
        finally:
            fleet.close()

    def test_return_times_out_after_configured_seconds(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        fleet = FleetController([configs[0]], safety, RecordingSimDriver)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.update_tag_position(robot_id, 0.50, 1.0, time.monotonic())
            fleet.start_tag_position_hold(
                robot_id,
                1.0,
                1.0,
                12.0,
                continuous_return=True,
                return_max_seconds=0.5,
            )
            self.assertTrue(
                wait_until(
                    lambda: "UWB 回位超时" in fleet.snapshots()[0].feedback,
                    timeout=2.0,
                )
            )
            self.assertFalse(worker._tag_position_hold)
        finally:
            fleet.close()

    def test_two_consecutive_samples_moving_away_abort_return(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        fleet = FleetController([configs[0]], safety, RecordingSimDriver)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            first = time.monotonic()
            fleet.update_tag_position(robot_id, 0.70, 1.0, first)
            fleet.start_tag_position_hold(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(
                wait_until(
                    lambda: worker._tag_last_control_generation
                    == worker._tag_position_generation
                    and worker._tag_last_error_distance is not None
                )
            )
            first_generation = worker._tag_position_generation
            second = time.monotonic()
            fleet.update_tag_position(robot_id, 0.60, 1.0, second)
            self.assertTrue(
                wait_until(
                    lambda: worker._tag_position_generation > first_generation
                    and worker._tag_last_control_generation
                    == worker._tag_position_generation
                )
            )
            third = time.monotonic()
            fleet.update_tag_position(robot_id, 0.50, 1.0, third)
            stopped = wait_until(lambda: not worker._tag_position_hold)
            self.assertTrue(
                stopped,
                (
                    worker._tag_divergence_ticks,
                    worker._tag_last_error_distance,
                    worker._tag_last_control_generation,
                    worker._tag_position,
                ),
            )
            self.assertFalse(worker._tag_auto_return_enabled)
            self.assertIn("连续远离", fleet.snapshots()[0].feedback)

            # A stale TAG divergence stop is not a hardware fault. An explicit
            # operator stunt must clear that controller state and still reach
            # the SDK exactly once.
            fleet.submit([robot_id], Command.LEFT_FLIP, return_to_start=True)
            self.assertTrue(
                wait_until(
                    lambda: any(
                        command is Command.LEFT_FLIP
                        for command, _, _ in worker.driver.command_log
                    )
                )
            )
            self.assertEqual(
                sum(
                    command is Command.LEFT_FLIP
                    for command, _, _ in worker.driver.command_log
                ),
                1,
            )
            self.assertEqual(worker._tag_divergence_ticks, 0)
        finally:
            fleet.close()

    def test_direct_tag_error_ignores_wrong_odometry_xy(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(
                wait_until(lambda: fleet.robots[robot_id]._tag_zero is not None)
            )
            # Deliberately make odometry X disagree by five metres. The return
            # direction must still follow the +0.30 m TAG error.
            fleet.set_odometry_pose(robot_id, 5.0, 0.0, 0.0)
            fleet.update_tag_position(robot_id, 0.70, 1.0, time.monotonic())
            fleet.start_tag_position_hold(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: bool(drivers[0].velocity_commands)))
            self.assertGreater(drivers[0].velocity_commands[-1]["x"], 0.0)
        finally:
            fleet.close()

    def test_stage_plus_y_heading_converts_tag_plus_y_error_to_forward(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(
                robot_id, 1.0, 1.0, 12.0, stage_heading_deg=90.0
            )
            self.assertTrue(
                wait_until(lambda: fleet.robots[robot_id]._tag_zero is not None)
            )
            # The dog is facing stage +Y. Returning from y=0.70 to y=1.00
            # must therefore command body-forward, not body-left.
            fleet.update_tag_position(robot_id, 1.0, 0.70, time.monotonic())
            fleet.start_tag_position_hold(
                robot_id, 1.0, 1.0, 12.0, stage_heading_deg=90.0
            )
            self.assertTrue(wait_until(lambda: bool(drivers[0].velocity_commands)))
            command = drivers[0].velocity_commands[-1]
            self.assertGreater(command["x"], 0.0)
            self.assertAlmostEqual(command["y"], 0.0, places=6)
        finally:
            fleet.close()

    def test_200_percent_return_is_faster_but_stays_below_global_limit(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(
                robot_id,
                1.0,
                1.0,
                12.0,
                stage_heading_deg=0.0,
                return_speed_percent=200.0,
            )
            self.assertTrue(
                wait_until(lambda: fleet.robots[robot_id]._tag_zero is not None)
            )
            fleet.update_tag_position(robot_id, 0.70, 1.0, time.monotonic())
            fleet.start_tag_position_hold(
                robot_id,
                1.0,
                1.0,
                12.0,
                stage_heading_deg=0.0,
                return_speed_percent=200.0,
            )
            self.assertTrue(wait_until(lambda: bool(drivers[0].velocity_commands)))
            command = drivers[0].velocity_commands[-1]
            self.assertGreater(command["x"], 0.40)
            self.assertLessEqual(command["x"], safety.max_x)
        finally:
            fleet.close()

    def test_continuous_return_refreshes_motion_between_tag_frames(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(
                robot_id,
                1.0,
                1.0,
                12.0,
                continuous_return=True,
            )
            self.assertTrue(
                wait_until(lambda: fleet.robots[robot_id]._tag_zero is not None)
            )
            # Only one displaced TAG frame is supplied. Continuous mode must
            # keep updating from short-term odometry instead of issuing one
            # pulse and parking until another absolute frame arrives.
            fleet.update_tag_position(robot_id, 0.50, 1.0, time.monotonic())
            fleet.start_tag_position_hold(
                robot_id,
                1.0,
                1.0,
                12.0,
                continuous_return=True,
            )
            self.assertTrue(
                wait_until(lambda: len(drivers[0].velocity_commands) >= 3)
            )
            self.assertTrue(
                all(command["x"] > 0.0 for command in drivers[0].velocity_commands[:3])
            )
            self.assertIn("连续回位", fleet.snapshots()[0].feedback)
        finally:
            fleet.close()

    def test_second_scale_uwb_latency_stops_before_waiting_for_fresh_sample(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 0.50, 1.0, time.monotonic())
            fleet.start_tag_position_hold(
                robot_id,
                1.0,
                1.0,
                12.0,
                continuous_return=True,
                return_latency_ms=1000.0,
            )
            self.assertTrue(
                wait_until(lambda: worker._tag_latency_wait_until > 0.0, timeout=2.0)
            )
            self.assertFalse(worker._velocity_active)
            command_count = len(drivers[0].velocity_commands)
            time.sleep(0.35)
            self.assertEqual(len(drivers[0].velocity_commands), command_count)
            self.assertIn("秒级 UWB", fleet.snapshots()[0].feedback)
        finally:
            fleet.close()

    def test_second_scale_map_target_uses_continuous_odometry_approach(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 0.50, 1.0, time.monotonic())
            fleet.start_tag_position_target(
                robot_id,
                1.0,
                1.0,
                12.0,
                continuous_return=True,
                return_latency_ms=1000.0,
            )
            self.assertTrue(
                wait_until(lambda: len(drivers[0].velocity_commands) >= 3)
            )
            self.assertIsNone(worker._tag_latency_burst_params)
            self.assertIn("里程计连续导航", fleet.snapshots()[0].feedback)
        finally:
            fleet.close()

    def test_delayed_frames_while_moving_do_not_false_trigger_divergence(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        fleet = FleetController([configs[0]], safety, RecordingSimDriver)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 0.0, 1.0, time.monotonic())
            fleet.start_tag_position_target(
                robot_id,
                1.0,
                1.0,
                12.0,
                continuous_return=True,
                return_latency_ms=2000.0,
            )
            self.assertTrue(wait_until(lambda: worker._velocity_active))

            # These positions deliberately look progressively worse, but they
            # arrive during motion and therefore represent historical robot
            # positions under a two-second UWB transport delay.
            for x in (-0.10, -0.20, -0.30):
                prior_generation = worker._tag_position_generation
                fleet.update_tag_position(robot_id, x, 1.0, time.monotonic())
                self.assertTrue(
                    wait_until(
                        lambda: worker._tag_position_generation > prior_generation
                        and worker._tag_last_control_generation
                        == worker._tag_position_generation
                    )
                )
            self.assertTrue(worker._tag_position_hold)
            self.assertEqual(worker._tag_divergence_ticks, 0)
        finally:
            fleet.close()

    def test_stunt_can_return_to_its_own_pre_action_uwb_position(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        safety = replace(safety, one_shot_return_ms=50)
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.submit(
                [robot_id],
                Command.PRONK,
                duration_s=0.5,
                return_to_start=True,
                stage_heading_deg=0.0,
                continuous_return=True,
                return_latency_ms=300.0,
            )
            self.assertTrue(
                wait_until(lambda: worker._stunt_tag_return_target is not None)
            )
            fleet.update_tag_position(robot_id, 0.70, 1.0, time.monotonic())
            self.assertTrue(
                wait_until(
                    lambda: worker._tag_position_hold,
                    timeout=2.5,
                )
            )
            self.assertTrue(
                wait_until(
                    lambda: drivers[0].velocity_commands
                    and drivers[0].velocity_commands[-1]["x"] > 0.0,
                    timeout=2.0,
                )
            )
            self.assertEqual(worker._tag_origin[:2], (1.0, 1.0))
        finally:
            fleet.close()

    def test_stunt_direction_locks_apply_to_post_stunt_return(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        safety = replace(safety, one_shot_return_ms=50)
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.submit(
                [robot_id],
                Command.PRONK,
                duration_s=0.5,
                return_to_start=True,
                stage_heading_deg=0.0,
                continuous_return=True,
                return_latency_ms=300.0,
                direction_locks={"dog01": "x"},
            )
            self.assertTrue(
                wait_until(lambda: worker._stunt_tag_return_target is not None)
            )
            self.assertEqual(worker._tag_direction_lock, "x")
            fleet.update_tag_position(robot_id, 0.70, 0.80, time.monotonic())
            self.assertTrue(
                wait_until(
                    lambda: worker._tag_position_hold
                    and drivers[0].velocity_commands
                    and drivers[0].velocity_commands[-1]["x"] > 0.0,
                    timeout=2.5,
                )
            )
            self.assertTrue(
                all(
                    abs(command["y"]) < 1e-6
                    for command in drivers[0].velocity_commands
                )
            )
            self.assertEqual(worker._tag_origin[:2], (1.0, 1.0))
        finally:
            fleet.close()

    def test_stunt_returns_to_calibrated_zero_immediately(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        safety = replace(safety, one_shot_return_ms=50)
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 0.50, 0.50, time.monotonic())
            fleet.set_tag_zero(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.submit(
                [robot_id],
                Command.PRONK,
                duration_s=0.5,
                return_to_start=True,
                stage_heading_deg=0.0,
                continuous_return=True,
                return_latency_ms=300.0,
                return_max_seconds=4.0,
            )
            self.assertTrue(
                wait_until(
                    lambda: worker._stunt_tag_return_target is not None
                )
            )
            self.assertEqual(worker._stunt_tag_return_target[:2], (1.0, 1.0))
            self.assertTrue(
                wait_until(
                    lambda: worker._tag_origin is not None
                    and worker._tag_origin[:2] == (1.0, 1.0),
                    timeout=2.5,
                )
            )
        finally:
            fleet.close()

    def test_fast_continuous_return_brakes_before_crossing_tag_zero(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        fleet = FleetController([configs[0]], safety, RecordingSimDriver)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(
                robot_id,
                1.0,
                1.0,
                12.0,
                return_speed_percent=200.0,
                continuous_return=True,
            )
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.update_tag_position(robot_id, 0.50, 1.0, time.monotonic())
            fleet.start_tag_position_hold(
                robot_id,
                1.0,
                1.0,
                12.0,
                return_speed_percent=200.0,
                continuous_return=True,
            )
            self.assertTrue(
                wait_until(
                    lambda: worker._tag_hold_arrived
                    or worker._tag_waiting_for_generation is not None,
                    timeout=3.0,
                )
            )
            time.sleep(0.25)
            travelled = fleet.snapshots()[0].corrected_pose[0]
            self.assertGreater(travelled, 0.25)
            self.assertLess(travelled, 0.52)
        finally:
            fleet.close()

    def test_position_inside_ten_centimetres_is_accepted_without_translation(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.update_tag_position(
                robot_id,
                1.0 - (TAG_POSITION_TOLERANCE_M - 0.01),
                1.0,
                time.monotonic(),
            )
            fleet.start_tag_position_hold(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_hold_arrived))
            self.assertEqual(drivers[0].velocity_commands, [])
            fleet.update_tag_position(robot_id, 0.80, 1.0, time.monotonic())
            self.assertTrue(wait_until(lambda: bool(drivers[0].velocity_commands)))
            self.assertGreater(drivers[0].velocity_commands[-1]["x"], 0.0)
        finally:
            fleet.close()

    def test_stunt_completion_rearms_direct_tag_return(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        safety = replace(safety, one_shot_return_ms=50)
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.start_tag_position_hold(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_auto_return_enabled))
            fleet.submit([robot_id], Command.PRONK, duration_s=0.5, target_yaw=0.0)
            self.assertTrue(wait_until(lambda: worker._one_shot_unlock_at is not None))
            fleet.update_tag_position(robot_id, 0.60, 1.0, time.monotonic())
            self.assertTrue(
                wait_until(
                    lambda: worker._tag_position_hold
                    and drivers[0].velocity_commands
                    and drivers[0].velocity_commands[-1]["x"] > 0.0,
                    timeout=2.5,
                )
            )
        finally:
            fleet.close()

    def test_tag_return_drives_and_manual_turn_cancels_it(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        drivers = []

        def factory(config):
            driver = RecordingSimDriver(config)
            drivers.append(driver)
            return driver

        fleet = FleetController([configs[0]], safety, factory)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            now = time.monotonic()
            fleet.update_tag_position(robot_id, 1.0, 1.0, now)
            fleet.set_tag_zero(robot_id, 1.0, 1.0, 3.0)
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            fleet.update_tag_position(robot_id, 0.70, 1.0, time.monotonic())
            fleet.start_tag_position_hold(robot_id, 1.0, 1.0, 3.0)
            self.assertTrue(wait_until(lambda: worker._tag_position_hold))
            self.assertTrue(wait_until(lambda: bool(drivers[0].velocity_commands)))
            self.assertGreater(drivers[0].velocity_commands[-1]["x"], 0.0)

            fleet.submit(
                [robot_id],
                Command.VELOCITY,
                x=0.0,
                y=0.0,
                yaw=0.4,
                operator_override=True,
            )
            self.assertTrue(
                wait_until(
                    lambda: drivers[0].velocity_commands
                    and abs(drivers[0].velocity_commands[-1]["yaw"] - 0.4) < 1e-9
                )
            )
            self.assertFalse(worker._tag_position_hold)
            self.assertFalse(worker._tag_auto_return_enabled)
        finally:
            fleet.close()

    def test_temporary_map_target_does_not_replace_saved_tag_zero(self):
        _, configs, safety = load_config(ROOT / "config" / "robots.json")
        fleet = FleetController([configs[0]], safety, RecordingSimDriver)
        robot_id = configs[0].robot_id
        worker = fleet.robots[robot_id]
        try:
            fleet.connect([robot_id])
            self.assertTrue(wait_until(lambda: fleet.snapshots()[0].connected))
            fleet.update_tag_position(robot_id, 1.0, 1.0, time.monotonic())
            fleet.set_tag_zero(robot_id, 1.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_zero is not None))
            saved_zero = worker._tag_zero
            fleet.start_tag_position_target(robot_id, 2.0, 1.0, 12.0)
            self.assertTrue(wait_until(lambda: worker._tag_target_mode))
            self.assertEqual(worker._tag_zero, saved_zero)
            self.assertEqual(worker._tag_origin[:2], (2.0, 1.0))
            self.assertFalse(worker._tag_auto_return_enabled)
        finally:
            fleet.close()


if __name__ == "__main__":
    unittest.main()
