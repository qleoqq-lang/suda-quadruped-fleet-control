import queue
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from dual_dog.control.controller import RobotWorker, WorkItem
from dual_dog.devices.drivers import BpxDriver, _bpx_process_main
from dual_dog.models import Command, RobotConfig
from test_bpx_driver_protocol import FakeConnection


CONFIG = RobotConfig("dog01", "dog1", "192.0.2.1", "", 9901, 1)


def telemetry_payload(connected=True):
    payload = dict.fromkeys((
        "battery", "rpy", "odom_position", "leg_velocity_body", "leg_orientation",
        "imu_acceleration", "imu_angular_velocity", "imu_quaternion", "joint_position",
        "joint_velocity", "motion_state", "gait", "sub_gait",
    ))
    payload.update(connected=connected, temperatures=[], timestamps=(1, 2, 3),
                   odometry_token=(1, 2, 3), battery=75, motion_state=6)
    return payload


class SdkLifecycleTests(unittest.TestCase):
    def test_connect_does_not_claim_online_and_keeps_polling_until_recovery(self):
        driver = BpxDriver(CONFIG)
        with patch.object(driver, "_request", side_effect=[False, {"connected": False}, telemetry_payload()]) as request:
            driver.connect()
            driver.connect()
            self.assertFalse(driver._connected)
            self.assertTrue(driver.connection_requested)
            self.assertFalse(driver.telemetry().connected)
            recovered = driver.telemetry()
            self.assertTrue(recovered.connected)
            self.assertEqual(recovered.battery, 75)
            self.assertEqual([entry.args[0] for entry in request.call_args_list], ["connect", "telemetry", "telemetry"])

    def test_lost_tcp_does_not_kill_sdk_and_can_recover(self):
        driver = BpxDriver(CONFIG)
        driver.connection_requested = True
        driver._connected = True
        with patch.object(driver, "_request", side_effect=[{"connected": False}, telemetry_payload()]), patch.object(driver, "_terminate_process") as terminate:
            self.assertFalse(driver.telemetry().connected)
            self.assertTrue(driver.connection_requested)
            self.assertTrue(driver.telemetry().connected)
            terminate.assert_not_called()

    def test_explicit_disconnect_stops_background_attempts_even_while_offline(self):
        driver = BpxDriver(CONFIG)
        driver.connection_requested = True
        with patch.object(driver, "_request") as request, patch.object(driver, "_terminate_process"):
            driver.disconnect()
            request.assert_called_once_with("disconnect")
            self.assertFalse(driver.connection_requested)

    def test_disconnect_with_sit_posture_uses_sit_before_disconnecting(self):
        driver = BpxDriver(CONFIG)
        driver.connection_requested = True
        driver._connected = True
        with (
            patch.object(driver, "hold_sit") as hold_sit,
            patch.object(driver, "hold_stand") as hold_stand,
            patch.object(driver, "_request") as request,
            patch.object(driver, "_terminate_process"),
        ):
            driver.disconnect(Command.SIT)
        hold_sit.assert_called_once()
        hold_stand.assert_not_called()
        request.assert_called_once_with("disconnect")

    def test_child_refuses_motion_while_tcp_offline(self):
        motion = Mock()
        motion.isConnected.return_value = False
        connection = FakeConnection([("command", (Command.STAND.value, {}))])
        with patch.dict("sys.modules", {"bpx_sdk": SimpleNamespace(MotionLevelControl=lambda: motion)}):
            _bpx_process_main(connection, CONFIG)
        motion.setStandUp.assert_not_called()
        self.assertFalse(connection.responses[0][0])

    def test_child_connect_is_idempotent_and_reports_actual_tcp_state(self):
        motion = Mock()
        motion.connect.return_value = True
        motion.isConnected.return_value = False
        connection = FakeConnection([("connect", ()), ("connect", ()), ("telemetry", ())])
        with patch.dict("sys.modules", {"bpx_sdk": SimpleNamespace(MotionLevelControl=lambda: motion)}):
            _bpx_process_main(connection, CONFIG)
        motion.connect.assert_called_once()
        self.assertEqual(connection.responses, [(True, False), (True, False), (True, {"connected": False})])

    def test_disconnect_clears_queued_motion_but_keeps_shutdown(self):
        worker = SimpleNamespace(
            _queue=queue.Queue(), _priority_queue=queue.Queue(),
            _clear_stunt_recovery=Mock(), _tracker=Mock(), _speed_sync=Mock(),
            _turn_speed_sync=Mock(), _pause_adaptive_slip=Mock(),
        )
        worker._queue.put(WorkItem(Command.VELOCITY, {"x": 0.1}))
        worker._priority_queue.put(WorkItem(Command.LEFT_FLIP, {}))
        worker._priority_queue.put(WorkItem("shutdown", {}))
        RobotWorker._cancel_disconnected_actions(worker)
        self.assertTrue(worker._queue.empty())
        self.assertEqual(worker._priority_queue.get_nowait().command, "shutdown")
        self.assertTrue(worker._priority_queue.empty())
        self.assertFalse(worker._velocity_active)
        worker._clear_stunt_recovery.assert_called_once()


if __name__ == "__main__":
    unittest.main()
