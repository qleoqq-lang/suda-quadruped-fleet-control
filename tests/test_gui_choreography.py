import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from dual_dog.control.choreography import DancePhase, PhaseKind
from dual_dog.ui.gui import MainWindow
from dual_dog.models import Command, Telemetry
from dual_dog.devices.positioning import PositionSettings


class RecordingFleet:
    def __init__(self):
        self.calls = []

    def submit(self, targets, command, **params):
        self.calls.append((tuple(targets), command, params))


class SnapshotRecordingFleet(RecordingFleet):
    def __init__(self, snapshots):
        super().__init__()
        self.snapshots_by_id = {item.robot_id: item for item in snapshots}

    def snapshots(self):
        return list(self.snapshots_by_id.values())


class GuiChoreographyTransitionTests(unittest.TestCase):
    def test_restart_sdk_disconnects_then_reconnects_without_motion(self):
        class Fleet:
            def __init__(self):
                self.events = []

            def disconnect(self, targets):
                self.events.append(("disconnect", tuple(targets)))

            def connect(self, targets):
                self.events.append(("connect", tuple(targets)))

        class Button:
            def __init__(self):
                self.enabled = None

            def setEnabled(self, enabled):
                self.enabled = enabled

        fleet = Fleet()
        button = Button()
        scheduled = []
        window = SimpleNamespace(
            selected=lambda: ["dog01", "dog04"],
            ensure_fleet=lambda connection_only=False: True,
            flip_pair_timer=SimpleNamespace(isActive=lambda: False),
            flip_test_queue=[],
            dance_timer=SimpleNamespace(isActive=lambda: False),
            cancel_dance=lambda **kwargs: None,
            disable_yaw_hold_and_stop=lambda: None,
            formation_sync=SimpleNamespace(reset=lambda: None),
            connection_targets=("dog01",),
            connection_last_missing=(),
            connection_report_after=1.0,
            restart_sdk_button=button,
            statusBar=lambda: SimpleNamespace(showMessage=lambda *_args: None),
            fleet=fleet,
        )
        window._reconnect_after_sdk_restart = (
            lambda targets: MainWindow._reconnect_after_sdk_restart(window, targets)
        )

        with patch("dual_dog.ui.gui.QTimer.singleShot", side_effect=lambda _ms, callback: scheduled.append(callback)):
            MainWindow.restart_selected_sdk(window)

        self.assertEqual(fleet.events, [("disconnect", ("dog01", "dog04"))])
        self.assertFalse(button.enabled)
        self.assertEqual(len(scheduled), 1)

        scheduled[0]()

        self.assertEqual(
            fleet.events,
            [
                ("disconnect", ("dog01", "dog04")),
                ("connect", ("dog01", "dog04")),
            ],
        )
        self.assertTrue(button.enabled)

    def test_move_end_still_sends_stop(self):
        fleet = RecordingFleet()
        window = SimpleNamespace(fleet=fleet, dance_finished_cues=set())
        phase = DancePhase(
            PhaseKind.MOVE,
            duration=2.0,
            x=-0.3,
            targets=("dog01",),
        )
        MainWindow.finish_dance_cue(window, 0, phase)
        self.assertEqual(fleet.calls, [(("dog01",), Command.STOP, {})])

    def test_manual_motion_skips_offline_selected_robots(self):
        class Value:
            def __init__(self, value):
                self._value = value

            def value(self):
                return self._value

            def isChecked(self):
                return bool(self._value)

        fleet = SnapshotRecordingFleet(
            [
                Telemetry("dog01", connected=True),
                Telemetry("dog02", connected=False),
            ]
        )
        window = SimpleNamespace(
            fleet=fleet,
            active_velocity=(0.5, 0.0, 0.0),
            target_yaw=Value(0.0),
            yaw_hold=Value(False),
            lateral_hold=Value(False),
            speed_sync=Value(False),
            adaptive_slip=Value(False),
            yaw_offsets={},
            selected=lambda: ["dog01", "dog02"],
        )

        MainWindow.repeat_motion(window)

        self.assertEqual([call[0] for call in fleet.calls], [("dog01",)])
        self.assertEqual(fleet.calls[0][1], Command.VELOCITY)

    def test_stationary_correction_turns_toward_each_calibrated_target(self):
        from dual_dog.control.feedback import StraightLineTracker
        from dual_dog.models import SafetyConfig

        fleet = RecordingFleet()
        window = SimpleNamespace(
            fleet=fleet,
            target_yaw=SimpleNamespace(value=lambda: 0.0),
            speed_sync=SimpleNamespace(isChecked=lambda: False),
            yaw_offsets={"dog01": 0.3, "dog02": -0.4},
        )
        phase = DancePhase(
            PhaseKind.CORRECT_YAW, duration=2.0,
            targets=("dog01", "dog02"),
        )
        MainWindow.submit_dance_velocity(window, phase)
        self.assertEqual(len(fleet.calls), 2)
        for targets, command, params in fleet.calls:
            with self.subTest(robot=targets[0]):
                self.assertEqual(command, Command.VELOCITY)
                self.assertEqual(params["x"], 0.0)
                self.assertEqual(params["y"], 0.0)
                target = -window.yaw_offsets[targets[0]]
                self.assertAlmostEqual(params["target_yaw"], target)
                result = StraightLineTracker(SafetyConfig()).apply(
                    params["x"], params["y"], params["yaw"],
                    Telemetry(targets[0], connected=True, rpy=(0.0, 0.0, 0.0)),
                    target_yaw=params["target_yaw"],
                )
                self.assertGreater(result.yaw * target, 0.0)
                self.assertEqual((result.x, result.y), (0.0, 0.0))

    def test_stunt_return_params_include_stale_and_timeout(self):
        window = SimpleNamespace(
            position_window=SimpleNamespace(settings=PositionSettings())
        )
        params = MainWindow._stunt_position_return_params(window, True)
        self.assertTrue(params["return_to_start"])
        self.assertIn("stale_seconds", params)
        self.assertEqual(params["return_max_seconds"], 4.0)

    def test_stunt_start_return_uses_current_yaw_without_shared_calibration(self):
        class Checked:
            def isChecked(self):
                return True

        window = SimpleNamespace(
            imu_heading_return=Checked(),
            target_yaw=SimpleNamespace(value=lambda: 0.0),
            yaw_calibrated=set(),
            yaw_offsets={},
            fleet=SimpleNamespace(
                snapshots=lambda: [
                    Telemetry("dog01", connected=True, rpy=(0.0, 0.0, 0.42))
                ]
            ),
        )
        targets = MainWindow.stunt_heading_targets(
            window,
            ["dog01"],
            show_warning=False,
            allow_current_fallback=True,
        )
        self.assertEqual(targets, {"dog01": 0.42})

    def test_manual_stunt_always_allows_current_heading_fallback(self):
        class Checked:
            def __init__(self, checked):
                self.checked = checked

            def isChecked(self):
                return self.checked

        class Timer:
            def isActive(self):
                return False

            def stop(self):
                pass

        class Status:
            def showMessage(self, *_args):
                pass

        class Value:
            def value(self):
                return 2.0

        fleet = RecordingFleet()
        fallback_values = []
        window = SimpleNamespace(
            dance_timer=Timer(),
            flip_pair_timer=Timer(),
            fleet=fleet,
            confirm_stunts=Checked(True),
            position_hold=Checked(True),
            stunt_return_to_start=Checked(False),
            selected=lambda: ["dog01"],
            stunt_heading_targets=lambda targets, allow_current_fallback: (
                fallback_values.append(allow_current_fallback) or {"dog01": 0.2}
            ),
            motion_timer=Timer(),
            active_velocity=(0.0, 0.0, 0.0),
            manual_turn_active=False,
            _stunt_position_return_params=lambda enabled: {
                "return_to_start": enabled
            },
            non_flip_seconds=Value(),
            statusBar=lambda: Status(),
            safety=SimpleNamespace(
                one_shot_trigger_ms=100,
                one_shot_return_ms=3000,
            ),
        )

        MainWindow.trigger_one_shot(window, Command.LEFT_FLIP)

        self.assertEqual(fallback_values, [True])
        self.assertEqual(fleet.calls[0][0:2], (("dog01",), Command.LEFT_FLIP))

    def test_manual_stunt_survives_optional_recovery_preparation_failures(self):
        class Checked:
            def __init__(self, checked):
                self.checked = checked

            def isChecked(self):
                return self.checked

        class Timer:
            def isActive(self):
                return False

            def stop(self):
                pass

        class Status:
            def showMessage(self, *_args):
                pass

        fleet = RecordingFleet()

        def fail_heading(*_args, **_kwargs):
            raise RuntimeError("IMU preparation failed")

        def fail_position(*_args, **_kwargs):
            raise RuntimeError("position preparation failed")

        window = SimpleNamespace(
            dance_timer=Timer(),
            flip_pair_timer=Timer(),
            fleet=fleet,
            confirm_stunts=Checked(True),
            position_hold=Checked(True),
            stunt_return_to_start=Checked(True),
            selected=lambda: ["dog01"],
            stunt_heading_targets=fail_heading,
            motion_timer=Timer(),
            active_velocity=(0.0, 0.0, 0.0),
            manual_turn_active=False,
            _stunt_position_return_params=fail_position,
            non_flip_seconds=SimpleNamespace(value=lambda: 2.0),
            statusBar=lambda: Status(),
            safety=SimpleNamespace(
                one_shot_trigger_ms=100,
                one_shot_return_ms=3000,
            ),
        )

        MainWindow.trigger_one_shot(window, Command.LEFT_FLIP)

        self.assertEqual(fleet.calls[0][0:2], (("dog01",), Command.LEFT_FLIP))
        self.assertFalse(fleet.calls[0][2]["return_to_start"])

    def test_manual_stunt_takes_over_stale_dance_timer(self):
        class Checked:
            def isChecked(self):
                return True

        class ActiveTimer:
            def isActive(self):
                return True

            def stop(self):
                pass

        class InactiveTimer(ActiveTimer):
            def isActive(self):
                return False

        fleet = RecordingFleet()
        cancelled = []
        window = SimpleNamespace(
            dance_timer=ActiveTimer(),
            flip_pair_timer=InactiveTimer(),
            fleet=fleet,
            confirm_stunts=Checked(),
            position_hold=Checked(),
            stunt_return_to_start=Checked(),
            selected=lambda: ["dog01"],
            cancel_dance=lambda send_stop: cancelled.append(send_stop),
            stunt_heading_targets=lambda *_args, **_kwargs: {},
            motion_timer=InactiveTimer(),
            active_velocity=(0.0, 0.0, 0.0),
            manual_turn_active=False,
            _stunt_position_return_params=lambda enabled: {
                "return_to_start": enabled
            },
            non_flip_seconds=SimpleNamespace(value=lambda: 2.0),
            statusBar=lambda: SimpleNamespace(showMessage=lambda *_args: None),
            safety=SimpleNamespace(
                one_shot_trigger_ms=100,
                one_shot_return_ms=3000,
            ),
        )

        MainWindow.trigger_one_shot(window, Command.LEFT_FLIP)

        self.assertEqual(cancelled, [True])
        self.assertEqual(fleet.calls[0][0:2], (("dog01",), Command.LEFT_FLIP))

    def test_stunt_heading_uses_calibrated_shared_audience_target(self):
        class Value:
            def __init__(self, value):
                self._value = value

            def value(self):
                return self._value

            def isChecked(self):
                return bool(self._value)

        window = SimpleNamespace(
            imu_heading_return=Value(True),
            target_yaw=Value(15.0),
            yaw_calibrated={"dog01", "dog02"},
            yaw_offsets={"dog01": 0.10, "dog02": -0.20},
            fleet=SimpleNamespace(
                snapshots=lambda: [
                    Telemetry("dog01", connected=True, rpy=(0.0, 0.0, 0.1)),
                    Telemetry("dog02", connected=True, rpy=(0.0, 0.0, -0.2)),
                ]
            ),
        )

        targets = MainWindow.stunt_heading_targets(
            window, ["dog01", "dog02"], show_warning=False
        )

        self.assertIsNotNone(targets)
        self.assertAlmostEqual(targets["dog01"], math.radians(15.0) - 0.10)
        self.assertAlmostEqual(targets["dog02"], math.radians(15.0) + 0.20)

    def test_dance_trick_forwards_individual_imu_heading_targets(self):
        class Checked:
            def isChecked(self):
                return False

        class Status:
            def showMessage(self, *_args):
                pass

        fleet = RecordingFleet()
        window = SimpleNamespace(
            dance_started_cues=set(),
            fleet=fleet,
            dance_stunt_yaw_targets={"dog01": 0.3, "dog02": -0.4},
            statusBar=lambda: Status(),
        )
        phase = DancePhase(
            PhaseKind.TRICK,
            duration=2.0,
            command=Command.LEFT_FLIP,
            targets=("dog01", "dog02"),
        )

        MainWindow.start_dance_cue(window, 2, phase)

        self.assertEqual(len(fleet.calls), 2)
        self.assertEqual(fleet.calls[0][2]["target_yaw"], 0.3)
        self.assertEqual(fleet.calls[1][2]["target_yaw"], -0.4)

    def test_finished_trick_does_not_queue_stop(self):
        fleet = RecordingFleet()
        window = SimpleNamespace(dance_finished_cues=set(), fleet=fleet)
        phase = DancePhase(
            PhaseKind.TRICK,
            duration=2.0,
            command=Command.PRONK,
            targets=("dog01",),
        )

        MainWindow.finish_dance_cue(window, 4, phase)

        self.assertEqual(window.dance_finished_cues, {4})
        self.assertEqual(fleet.calls, [])

    def test_posture_cue_submits_stand_without_stunt_parameters_or_finish_stop(self):
        class Status:
            def showMessage(self, *_args):
                pass

        fleet = RecordingFleet()
        window = SimpleNamespace(
            dance_started_cues=set(),
            dance_finished_cues=set(),
            fleet=fleet,
            statusBar=lambda: Status(),
        )
        phase = DancePhase(
            PhaseKind.POSTURE,
            duration=3.0,
            command=Command.STAND,
            targets=("dog01",),
        )

        MainWindow.start_dance_cue(window, 5, phase)
        MainWindow.finish_dance_cue(window, 5, phase)

        self.assertEqual(fleet.calls, [(('dog01',), Command.STAND, {})])

    def test_dance_velocity_forwards_speed_sync_setting(self):
        class Value:
            def __init__(self, value):
                self._value = value

            def value(self):
                return self._value

            def isChecked(self):
                return bool(self._value)

        fleet = RecordingFleet()
        window = SimpleNamespace(
            fleet=fleet,
            target_yaw=Value(0.0),
            speed_sync=Value(True),
            lateral_hold=Value(False),
            adaptive_slip=Value(True),
            yaw_offsets={"dog01": 0.0},
        )
        phase = DancePhase(
            PhaseKind.MOVE,
            duration=1.0,
            x=0.0,
            y=0.25,
            yaw=0.0,
            targets=("dog01",),
        )

        MainWindow.submit_dance_velocity(window, phase)

        self.assertTrue(fleet.calls[0][2]["speed_sync"])
        self.assertIn("target_yaw", fleet.calls[0][2])
        self.assertFalse(fleet.calls[0][2]["path_correction"])
        self.assertTrue(fleet.calls[0][2]["adaptive_slip"])

    def test_manual_horizontal_move_forwards_yaw_target(self):
        class Value:
            def __init__(self, value):
                self._value = value

            def value(self):
                return self._value

            def isChecked(self):
                return bool(self._value)

        fleet = RecordingFleet()
        window = SimpleNamespace(
            fleet=fleet,
            active_velocity=(0.0, 0.25, 0.0),
            target_yaw=Value(0.0),
            yaw_hold=Value(True),
            lateral_hold=Value(False),
            speed_sync=Value(True),
            adaptive_slip=Value(True),
            yaw_offsets={"dog01": 0.0},
            selected=lambda: ["dog01"],
        )

        MainWindow.repeat_motion(window)

        self.assertEqual(fleet.calls[0][2]["y"], 0.25)
        self.assertIn("target_yaw", fleet.calls[0][2])
        self.assertTrue(fleet.calls[0][2]["adaptive_slip"])

    def test_manual_turn_does_not_forward_old_yaw_target(self):
        class Value:
            def __init__(self, value):
                self._value = value

            def value(self):
                return self._value

            def isChecked(self):
                return bool(self._value)

        fleet = RecordingFleet()
        window = SimpleNamespace(
            fleet=fleet,
            active_velocity=(0.0, 0.0, 0.25),
            target_yaw=Value(45.0),
            yaw_hold=Value(True),
            lateral_hold=Value(True),
            speed_sync=Value(True),
            adaptive_slip=Value(True),
            yaw_offsets={"dog01": 0.0},
            selected=lambda: ["dog01"],
        )

        MainWindow.repeat_motion(window)

        params = fleet.calls[0][2]
        self.assertEqual(params["yaw"], 0.25)
        self.assertNotIn("target_yaw", params)
        self.assertFalse(params["path_correction"])
        self.assertTrue(params["speed_sync"])
        self.assertFalse(params["adaptive_slip"])

    def test_manual_turn_result_becomes_new_common_yaw_target(self):
        class Target:
            def __init__(self):
                self.current = 0.0

            def setValue(self, value):
                self.current = value

        target = Target()
        snapshots = [
            Telemetry("dog01", connected=True, rpy=(0.0, 0.0, 0.40)),
            Telemetry("dog02", connected=True, rpy=(0.0, 0.0, -0.20)),
        ]
        window = SimpleNamespace(
            fleet=SimpleNamespace(snapshots=lambda: snapshots),
            target_yaw=target,
            yaw_offsets={"dog01": -0.10, "dog02": 0.50},
            selected=lambda: ["dog01", "dog02"],
        )

        adopted = MainWindow.adopt_current_heading_target(window)

        self.assertTrue(adopted)
        self.assertAlmostEqual(target.current, 17.1887, places=3)

    def test_last_flip_keeps_button_locked_through_safety_interval(self):
        class Timer:
            def __init__(self):
                self.started_ms = None

            def start(self, milliseconds):
                self.started_ms = milliseconds

        class Value:
            def value(self):
                return 5.0

        class Status:
            def showMessage(self, *_args):
                pass

        triggered = []
        timer = Timer()
        window = SimpleNamespace(
            selected=lambda: ["dog01"],
            flip_pair_targets=("dog01",),
            flip_test_queue=[Command.LEFT_FLIP],
            trigger_one_shot=triggered.append,
            flip_pair_timer=timer,
            flip_pair_interval=Value(),
            statusBar=lambda: Status(),
            cancel_flip_pair_test=lambda _message: None,
        )

        MainWindow.run_next_flip_test(window)

        self.assertEqual(triggered, [Command.LEFT_FLIP])
        self.assertEqual(timer.started_ms, 5000)


if __name__ == "__main__":
    unittest.main()
