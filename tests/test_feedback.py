import math
import unittest

from dual_dog.control.feedback import (
    StraightLineTracker,
    YawPid,
    common_yaw_from_raw,
    raw_target_for_common_yaw,
    wrap_angle,
    yaw_calibration_offset,
)
from dual_dog.models import SafetyConfig, Telemetry


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.safety = SafetyConfig()
        self.tracker = StraightLineTracker(self.safety)

    @staticmethod
    def data(yaw=0.0):
        return Telemetry("dog01", connected=True, rpy=(0.0, 0.0, yaw))

    @staticmethod
    def path_data(
        x=0.0, y=0.0, yaw=0.0, sigma=0.02, velocity=(0.0, 0.0)
    ):
        return Telemetry(
            "dog01",
            connected=True,
            rpy=(0.0, 0.0, yaw),
            corrected_pose=(x, y, math.degrees(yaw)),
            corrected_velocity_world=velocity,
            odom_uncertainty=(sigma, sigma, 1.0),
        )

    def test_wrap_angle_crosses_pi_boundary(self):
        self.assertAlmostEqual(wrap_angle(math.radians(358)), math.radians(-2))
        self.assertAlmostEqual(wrap_angle(math.radians(-358)), math.radians(2))

    def test_two_different_imu_zeros_map_to_same_common_heading(self):
        raw_dog1 = math.radians(25)
        raw_dog2 = math.radians(-40)
        offset1 = yaw_calibration_offset(raw_dog1)
        offset2 = yaw_calibration_offset(raw_dog2)
        self.assertAlmostEqual(common_yaw_from_raw(raw_dog1, offset1), 0.0)
        self.assertAlmostEqual(common_yaw_from_raw(raw_dog2, offset2), 0.0)
        common_target = math.radians(30)
        target1 = raw_target_for_common_yaw(common_target, offset1)
        target2 = raw_target_for_common_yaw(common_target, offset2)
        self.assertNotAlmostEqual(target1, target2)
        self.assertAlmostEqual(common_yaw_from_raw(target1, offset1), common_target)
        self.assertAlmostEqual(common_yaw_from_raw(target2, offset2), common_target)

    def test_first_command_locks_current_yaw(self):
        result = self.tracker.apply(0.2, 0, 0, self.data(0.7), now=1.0)
        self.assertEqual(result.yaw, 0.0)
        self.assertAlmostEqual(self.tracker.pid.target, 0.7)

    def test_user_can_give_absolute_target_yaw(self):
        result = self.tracker.apply(
            0.0, 0.0, 0.0, self.data(0.0), now=1.0, target_yaw=math.radians(30)
        )
        self.assertGreater(result.yaw, 0.0)
        self.assertAlmostEqual(self.tracker.pid.target, math.radians(30))

    def test_user_target_can_change_while_running(self):
        self.tracker.apply(
            0.2, 0.0, 0.0, self.data(0.0), now=1.0, target_yaw=math.radians(20)
        )
        result = self.tracker.apply(
            0.2, 0.0, 0.0, self.data(0.0), now=1.1, target_yaw=math.radians(-20)
        )
        self.assertLess(result.yaw, 0.0)
        self.assertAlmostEqual(self.tracker.pid.target, math.radians(-20))

    def test_yaw_change_generates_opposite_rotation(self):
        self.tracker.apply(0.2, 0, 0, self.data(0.5), now=1.0)
        result = self.tracker.apply(0.2, 0, 0, self.data(0.6), now=1.1)
        self.assertLess(result.yaw, 0.0)
        self.assertEqual(result.y, 0.0)

    def test_positive_and_negative_drift_correct_both_directions(self):
        self.tracker.apply(0.2, 0, 0, self.data(0.0), now=1.0)
        right = self.tracker.apply(0.2, 0, 0, self.data(0.1), now=1.1)
        self.assertLess(right.yaw, 0.0)
        self.tracker.reset()
        self.tracker.apply(0.2, 0, 0, self.data(0.0), now=2.0)
        left = self.tracker.apply(0.2, 0, 0, self.data(-0.1), now=2.1)
        self.assertGreater(left.yaw, 0.0)

    def test_output_is_limited(self):
        self.tracker.apply(0.2, 0, 0, self.data(), now=1.0)
        result = self.tracker.apply(0.2, 0, 0, self.data(2.0), now=1.1)
        self.assertEqual(abs(result.yaw), self.safety.max_yaw_correction)

    def test_output_is_zero_inside_stationary_tolerance(self):
        self.tracker.apply(
            0.0, 0.0, 0.0, self.data(0.0), now=1.0, target_yaw=0.0
        )
        result = self.tracker.apply(
            0.0, 0.0, 0.0, self.data(0.009), now=1.05, target_yaw=0.0
        )
        self.assertEqual(result.yaw, 0.0)
        self.assertIn("到位", result.status)

    def test_small_stationary_error_uses_effective_pulses_with_pauses(self):
        self.tracker.apply(
            0.0, 0.0, 0.0, self.data(0.0), now=1.0, target_yaw=0.0
        )
        outputs = [self.tracker.apply(
            0.0, 0.0, 0.0, self.data(0.021), now=1.05+i*.05, target_yaw=0.0
        ).yaw for i in range(12)]
        self.assertIn(-self.safety.min_yaw_actuation, outputs)
        self.assertIn(0.0, outputs)
        self.assertTrue(all(v in (0.0, -self.safety.min_yaw_actuation) for v in outputs))

    def test_stationary_heading_brakes_before_crossing_target(self):
        data = self.data(-0.03)
        data.corrected_yaw_rate = 0.3
        result = self.tracker.apply(0., 0., 0., data, now=1., target_yaw=0.)
        self.assertEqual(result.yaw, 0.0)

    def test_stale_telemetry_does_not_drive_heading(self):
        data = self.data(-0.5)
        data.updated_at = 1.0
        result = self.tracker.apply(0., 0., 0., data, now=3., target_yaw=0.)
        self.assertEqual(result.yaw, 0.0)
        self.assertIn("停止", result.status)

    def test_actual_sub_deadband_error_is_not_reported_as_zero(self):
        result = self.tracker.apply(0.,0.,0.,self.data(0.015),now=1.,target_yaw=0.)
        self.assertAlmostEqual(result.heading_error, -0.015)

    def test_pid_integral_is_anti_windup_limited(self):
        pid = YawPid(self.safety)
        pid.lock(0.0, 0.0)
        for index in range(1, 100):
            pid.update(-2.0, index * 0.1)
        self.assertLessEqual(abs(pid.integral), self.safety.yaw_integral_limit)

    def test_manual_turn_has_priority(self):
        result = self.tracker.apply(0.2, 0, 0.2, self.data(0.4), now=1.0)
        self.assertEqual((result.y, result.yaw), (0, 0.2))
        self.assertEqual(result.status, "人工转向")

    def test_missing_imu_falls_back_without_rotation(self):
        result = self.tracker.apply(
            0.2, 0, 0, Telemetry("dog01", connected=True), now=1.0
        )
        self.assertEqual((result.x, result.y, result.yaw), (0.2, 0, 0))
        self.assertIn("缺少 IMU", result.status)

    def test_lateral_feedback_moves_right_when_robot_drifted_left(self):
        self.tracker.apply(
            0.2, 0.0, 0.0, self.path_data(), now=1.0, lateral_correction=True
        )
        result = self.tracker.apply(
            0.2,
            0.0,
            0.0,
            self.path_data(x=0.2, y=0.05),
            now=1.4,
            lateral_correction=True,
        )
        self.assertLess(result.y, 0.0)
        self.assertIn("轨迹[补偿]", result.status)

    def test_lateral_feedback_moves_left_when_robot_drifted_right(self):
        self.tracker.apply(
            -0.2, 0.0, 0.0, self.path_data(), now=1.0, lateral_correction=True
        )
        result = self.tracker.apply(
            -0.2,
            0.0,
            0.0,
            self.path_data(x=-0.2, y=-0.05),
            now=1.4,
            lateral_correction=True,
        )
        self.assertGreater(result.y, 0.0)

    def test_lateral_feedback_obeys_correction_and_slew_limits(self):
        self.tracker.configure_lateral(5.0, 0.0, 0.0, 0.04)
        self.tracker.apply(
            0.2, 0.0, 0.0, self.path_data(), now=1.0, lateral_correction=True
        )
        first = self.tracker.apply(
            0.2,
            0.0,
            0.0,
            self.path_data(x=0.1, y=0.2),
            now=1.4,
            lateral_correction=True,
        )
        self.assertLessEqual(abs(first.y), self.safety.lateral_slew_rate * 0.1 + 1e-9)
        second = self.tracker.apply(
            0.2,
            0.0,
            0.0,
            self.path_data(x=0.2, y=0.2),
            now=1.5,
            lateral_correction=True,
        )
        self.assertLessEqual(abs(second.y), 0.04)

    def test_lateral_feedback_pauses_when_odometry_is_uncertain(self):
        result = self.tracker.apply(
            0.2,
            0.0,
            0.0,
            self.path_data(sigma=0.5),
            now=1.0,
            lateral_correction=True,
        )
        self.assertEqual(result.y, 0.0)
        self.assertIn("置信度不足", result.status)

    def test_lateral_feedback_waits_for_large_heading_error(self):
        result = self.tracker.apply(
            0.2,
            0.0,
            0.0,
            self.path_data(yaw=math.radians(20)),
            now=1.0,
            target_yaw=0.0,
            lateral_correction=True,
        )
        self.assertEqual(result.y, 0.0)
        self.assertIn("等待航向误差", result.status)

    def test_lateral_feedback_faults_on_implausibly_large_error(self):
        self.tracker.apply(
            0.2, 0.0, 0.0, self.path_data(), now=1.0, lateral_correction=True
        )
        result = self.tracker.apply(
            0.2,
            0.0,
            0.0,
            self.path_data(x=0.2, y=0.6),
            now=1.4,
            lateral_correction=True,
        )
        self.assertEqual(result.y, 0.0)
        self.assertIn("偏差过大", result.status)

    def test_diagonal_command_is_preserved_when_path_tracking_locks(self):
        result = self.tracker.apply(
            0.2,
            -0.1,
            0.0,
            self.path_data(),
            now=1.0,
            lateral_correction=True,
        )
        self.assertEqual(result.y, -0.1)
        self.assertEqual(result.x, 0.2)
        self.assertIn("轨迹[直线已锁定]", result.status)

    def test_left_move_corrects_forward_drift_by_moving_backward(self):
        self.tracker.apply(
            0.0,
            0.25,
            0.0,
            self.path_data(),
            now=1.0,
            target_yaw=0.0,
            path_correction=True,
        )
        result = self.tracker.apply(
            0.0,
            0.25,
            0.0,
            self.path_data(x=0.05, y=0.20),
            now=1.4,
            target_yaw=0.0,
            path_correction=True,
        )
        self.assertLess(result.x, 0.0)
        self.assertEqual(result.y, 0.25)
        self.assertIn("轨迹[补偿]", result.status)

    def test_right_move_corrects_backward_drift_by_moving_forward(self):
        self.tracker.apply(
            0.0,
            -0.25,
            0.0,
            self.path_data(),
            now=1.0,
            target_yaw=0.0,
            path_correction=True,
        )
        result = self.tracker.apply(
            0.0,
            -0.25,
            0.0,
            self.path_data(x=-0.05, y=-0.20),
            now=1.4,
            target_yaw=0.0,
            path_correction=True,
        )
        self.assertGreater(result.x, 0.0)
        self.assertEqual(result.y, -0.25)

    def test_cross_velocity_is_corrected_before_position_error_accumulates(self):
        self.tracker.apply(
            0.2,
            0.0,
            0.0,
            self.path_data(),
            now=1.0,
            target_yaw=0.0,
            path_correction=True,
        )
        result = self.tracker.apply(
            0.2,
            0.0,
            0.0,
            self.path_data(velocity=(0.2, 0.06)),
            now=1.4,
            target_yaw=0.0,
            path_correction=True,
        )
        self.assertLess(result.y, 0.0)
        self.assertIn("横速=", result.status)

    def test_corrected_gyro_rate_damps_rotation_at_target_angle(self):
        data = self.data(yaw=0.0)
        data.corrected_yaw_rate = 0.10
        result = self.tracker.apply(
            0.2, 0.0, 0.0, data, now=1.0, target_yaw=0.0
        )
        self.assertLess(result.yaw, 0.0)
        self.assertIn("角速度阻尼", result.status)

    def test_horizontal_move_can_use_yaw_pid_without_losing_lateral_speed(self):
        for lateral_speed in (0.25, -0.25):
            with self.subTest(lateral_speed=lateral_speed):
                tracker = StraightLineTracker(self.safety)
                result = tracker.apply(
                    0.0,
                    lateral_speed,
                    0.0,
                    self.data(yaw=math.radians(8.0)),
                    now=1.0,
                    target_yaw=0.0,
                )
                self.assertEqual(result.y, lateral_speed)
                self.assertLess(result.yaw, 0.0)
                self.assertIn("Yaw PID", result.status)

    def test_horizontal_yaw_hold_preserves_motion_when_imu_is_missing(self):
        result = self.tracker.apply(
            0.0,
            0.25,
            0.0,
            Telemetry("dog01", connected=True),
            now=1.0,
            target_yaw=0.0,
        )
        self.assertEqual((result.x, result.y, result.yaw), (0.0, 0.25, 0.0))
        self.assertIn("缺少 IMU", result.status)


if __name__ == "__main__":
    unittest.main()
