import logging
import os
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import QApplication, QMessageBox

from dual_dog.control.choreography import DanceStep, PhaseKind
from dual_dog.ui.gui import MainWindow
from dual_dog.models import Command, Telemetry
from voice_control.commands import VoiceIntent


ROOT = Path(__file__).resolve().parents[1]


class RecordingFleet:
    def __init__(self, snapshots):
        self.snapshots_by_id = {item.robot_id: item for item in snapshots}
        self.submitted = []

    def snapshots(self):
        return list(self.snapshots_by_id.values())

    def submit(self, targets, command, **params):
        self.submitted.append((tuple(targets), command, params))


class MainWindowRefreshTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_connected_snapshot_reaches_status_table_without_diagnostics(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        try:
            self.assertIsNone(window.diagnostic_recorder)
            window.fleet = SimpleNamespace(
                snapshots=lambda: [
                    Telemetry(
                        "dog01",
                        connected=True,
                        action="就绪",
                        battery=88,
                        rpy=(0.0, 0.0, 0.25),
                    )
                ]
            )

            window.refresh()

            row = window.status_table.rows["dog01"]
            self.assertEqual("已连接", window.status_table.item(row, 2).text())
            self.assertEqual("88%", window.status_table.item(row, 3).text())
            self.assertEqual("就绪", window.status_table.item(row, 5).text())
        finally:
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_refresh_keeps_motion_when_offline_selected_robots_exist(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        try:
            window.fleet = SimpleNamespace(
                snapshots=lambda: [
                    Telemetry("dog01", connected=True),
                    Telemetry("dog02", connected=False),
                ],
                submit=lambda *_args, **_kwargs: None,
            )
            window.active_velocity = (0.5, 0.0, 0.0)
            window.motion_timer.start()

            window.refresh()

            self.assertTrue(window.motion_timer.isActive())
        finally:
            window.motion_timer.stop()
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_refresh_checks_robot_when_it_becomes_connected(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        try:
            self.assertFalse(window.targets["dog11"].isChecked())
            window.fleet = SimpleNamespace(
                snapshots=lambda: [Telemetry("dog11", connected=True)]
            )

            window.refresh()

            self.assertTrue(window.targets["dog11"].isChecked())
        finally:
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_refresh_does_not_recheck_after_manual_deselect(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        try:
            window.fleet = SimpleNamespace(
                snapshots=lambda: [Telemetry("dog11", connected=True)]
            )
            window.refresh()
            window.targets["dog11"].setChecked(False)

            window.refresh()

            self.assertFalse(window.targets["dog11"].isChecked())
        finally:
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_first_ten_robots_are_selected_by_default(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        try:
            expected = [config.robot_id for config in window.configs[:10]]
            self.assertEqual(window.selected(), expected)
            self.assertTrue(all(window.targets[item].isChecked() for item in expected))
            self.assertTrue(
                all(
                    not window.targets[config.robot_id].isChecked()
                    for config in window.configs[10:]
                )
            )
        finally:
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_voice_tab_exists_and_blocks_stunts_by_default(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        try:
            self.assertEqual("开始语音识别", window.voice_button.text())
            self.assertFalse(window.voice_allow_stunts.isChecked())
            intent = VoiceIntent(
                targets=("dog01",),
                command=Command.LEFT_FLIP,
                params={},
                label="左翻",
                stunt=True,
            )

            window.handle_voice_result("一号左翻", (intent,))

            self.assertIn("已拦截", window.voice_status.text())
            self.assertIn("安全确认", window.voice_result.text())
            self.assertIsNone(window.voice_executor)
        finally:
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_close_confirmation_requests_sit_before_fleet_close(self):
        class ClosingFleet:
            def __init__(self):
                self.close_kwargs = None

            def submit(self, targets, command, **params):
                pass

            def close(self, timeout=3.0, final_posture=None):
                self.close_kwargs = {
                    "timeout": timeout,
                    "final_posture": final_posture,
                }

        window = MainWindow(ROOT / "config" / "robots.json")
        fleet = ClosingFleet()
        window.fleet = fleet
        event = QCloseEvent()
        try:
            with patch(
                "dual_dog.ui.gui.QMessageBox.question",
                return_value=QMessageBox.Yes,
            ):
                window.closeEvent(event)
            self.assertIs(fleet.close_kwargs["final_posture"], Command.SIT)
            self.assertTrue(event.isAccepted())
        finally:
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_start_dance_confirms_offline_skip_then_skips_offline_robots(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        fleet = RecordingFleet(
            [
                Telemetry("dog01", connected=True),
                Telemetry("dog02", connected=False),
            ]
        )
        window.fleet = fleet
        window.dance_steps = [
            DanceStep(
                PhaseKind.POSTURE,
                duration=3.0,
                command=Command.STAND,
                start_time=0.0,
                targets=("dog01", "dog02"),
            )
        ]
        try:
            with patch(
                "dual_dog.ui.gui.QMessageBox.question",
                return_value=QMessageBox.Yes,
            ) as question:
                window.start_dance()

            question.assert_called_once()
            self.assertIn("dog02", str(question.call_args))
            self.assertIn("跳过", str(question.call_args))
            self.assertTrue(window.dance_timer.isActive())
            self.assertEqual(window.dance_targets, ["dog01"])
            self.assertEqual(
                fleet.submitted,
                [(("dog01",), Command.STAND, {})],
            )
        finally:
            window.cancel_dance(send_stop=False)
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_start_dance_does_not_start_when_offline_skip_is_not_confirmed(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        fleet = RecordingFleet(
            [
                Telemetry("dog01", connected=True),
                Telemetry("dog02", connected=False),
            ]
        )
        window.fleet = fleet
        window.dance_steps = [
            DanceStep(
                PhaseKind.POSTURE,
                duration=3.0,
                command=Command.STAND,
                start_time=0.0,
                targets=("dog01", "dog02"),
            )
        ]
        try:
            with patch(
                "dual_dog.ui.gui.QMessageBox.question",
                return_value=QMessageBox.No,
            ) as question:
                window.start_dance()

            question.assert_called_once()
            self.assertFalse(window.dance_timer.isActive())
            self.assertEqual(fleet.submitted, [])
        finally:
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_start_dance_skips_loaded_tracks_that_are_not_selected(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        fleet = RecordingFleet(
            [
                Telemetry("dog01", connected=True),
                Telemetry("dog02", connected=True),
            ]
        )
        window.fleet = fleet
        window.targets["dog02"].setChecked(False)
        window.dance_steps = [
            DanceStep(
                PhaseKind.POSTURE,
                duration=3.0,
                command=Command.STAND,
                start_time=0.0,
                targets=("dog01", "dog02"),
            )
        ]
        try:
            with patch(
                "dual_dog.ui.gui.QMessageBox.question",
                return_value=QMessageBox.Yes,
            ) as question:
                window.start_dance()

            question.assert_called_once()
            self.assertIn("dog02", str(question.call_args))
            self.assertIn("未勾选", str(question.call_args))
            self.assertTrue(window.dance_timer.isActive())
            self.assertEqual(window.dance_targets, ["dog01"])
            self.assertEqual(
                fleet.submitted,
                [(("dog01",), Command.STAND, {})],
            )
        finally:
            window.cancel_dance(send_stop=False)
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_start_dance_does_not_start_when_all_robots_are_offline(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        fleet = RecordingFleet(
            [
                Telemetry("dog01", connected=False),
                Telemetry("dog02", connected=False),
            ]
        )
        window.fleet = fleet
        window.dance_steps = [
            DanceStep(
                PhaseKind.POSTURE,
                duration=3.0,
                command=Command.STAND,
                start_time=0.0,
                targets=("dog01", "dog02"),
            )
        ]
        try:
            with patch("dual_dog.ui.gui.QMessageBox.warning") as warning:
                window.start_dance()

            self.assertTrue(warning.called)
            self.assertIn("均未勾选或未连接", str(warning.call_args))
            self.assertFalse(window.dance_timer.isActive())
            self.assertEqual(fleet.submitted, [])
        finally:
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()

    def test_start_dance_confirms_overlapping_time_instructions(self):
        window = MainWindow(ROOT / "config" / "robots.json")
        fleet = RecordingFleet([Telemetry("dog01", connected=True)])
        window.fleet = fleet
        window.dance_steps = [
            DanceStep(
                PhaseKind.POSTURE,
                duration=2.0,
                command=Command.STAND,
                start_time=0.0,
                targets=("dog01",),
            ),
            DanceStep(
                PhaseKind.POSTURE,
                duration=1.0,
                command=Command.SIT,
                start_time=1.0,
                targets=("dog01",),
            ),
        ]
        try:
            with patch(
                "dual_dog.ui.gui.QMessageBox.question",
                return_value=QMessageBox.Yes,
            ) as question:
                window.start_dance()

            self.assertTrue(question.called)
            self.assertIn("时间重叠", str(question.call_args))
            self.assertTrue(window.dance_timer.isActive())
            self.assertEqual(window.dance_targets, ["dog01"])
        finally:
            window.cancel_dance(send_stop=False)
            window.refresh_timer.stop()
            window.music_player.stop()
            logging.getLogger().removeHandler(window.log_handler)
            window.deleteLater()
            self.app.processEvents()


if __name__ == "__main__":
    unittest.main()
