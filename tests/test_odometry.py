import math
import unittest

import numpy as np

from dual_dog.control.odometry import CorrectedOdometry, FourLegKinematics, wrap_angle


class OdometryTests(unittest.TestCase):
    @staticmethod
    def sample(yaw=0.0, velocity=(0.0, 0.0, 0.0), omega=0.0):
        quaternion = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        return {
            "leg_velocity_body": velocity,
            "leg_orientation": quaternion,
            "imu_acceleration": (0.0, 0.0, 9.80665),
            "imu_angular_velocity": (0.0, 0.0, omega),
            "imu_quaternion": quaternion,
            "joint_position": None,
            "joint_velocity": None,
            "dt": 0.05,
        }

    def test_forward_velocity_advances_x(self):
        odometry = CorrectedOdometry()
        for _ in range(40):
            odometry.update(**self.sample(velocity=(0.5, 0.0, 0.0)))
        x, y, _ = odometry.pose()
        self.assertGreater(x, 0.7)
        self.assertAlmostEqual(y, 0.0, places=3)

    def test_absolute_yaw_defines_user_frame_without_being_pulled_back(self):
        odometry = CorrectedOdometry()
        odometry.update(**self.sample(yaw=0.4))
        odometry.set_pose(2.0, -1.0, 90.0)
        for _ in range(20):
            odometry.update(**self.sample(yaw=0.4))
        x, y, yaw = odometry.pose()
        self.assertAlmostEqual(x, 2.0, places=2)
        self.assertAlmostEqual(y, -1.0, places=2)
        self.assertAlmostEqual(yaw, 90.0, places=1)

    def test_delta_yaw_updates_user_frame(self):
        odometry = CorrectedOdometry()
        odometry.update(**self.sample(yaw=-0.3))
        odometry.add_correction(0.2, -0.1, 30.0)
        for _ in range(10):
            odometry.update(**self.sample(yaw=-0.3))
        x, y, yaw = odometry.pose()
        self.assertAlmostEqual(x, 0.2, places=2)
        self.assertAlmostEqual(y, -0.1, places=2)
        self.assertAlmostEqual(yaw, 30.0, places=1)

    def test_direction_scales_are_validated(self):
        odometry = CorrectedOdometry()
        odometry.set_scales({"forward": 1.1, "right": 0.9})
        self.assertEqual(odometry.scales["forward"], 1.1)
        with self.assertRaises(ValueError):
            odometry.set_scales({"forward": 1.6})

    def test_long_data_gap_is_ignored(self):
        odometry = CorrectedOdometry()
        odometry.update(**(self.sample(velocity=(1.0, 0.0, 0.0)) | {"dt": 0.3}))
        self.assertEqual(odometry.pose(), (0.0, 0.0, 0.0))

    def test_zero_joint_velocity_has_four_consistent_legs(self):
        velocity, count = FourLegKinematics().body_velocity_measurement(
            [0.0] * 12, [0.0] * 12, (0.0, 0.0, 0.0)
        )
        self.assertEqual(count, 4)
        self.assertAlmostEqual(float(velocity[0]), 0.0)
        self.assertAlmostEqual(float(velocity[1]), 0.0)

    def test_robust_leg_filter_keeps_consistent_pair(self):
        class SyntheticKinematics(FourLegKinematics):
            def foot_position(self, leg_index, joint_angles):
                del leg_index, joint_angles
                return np.zeros(3)

            def jacobian(self, leg_index, joint_angles):
                del leg_index, joint_angles
                return np.eye(3)

        kinematics = SyntheticKinematics()
        candidates = (
            (0.20, 0.00),
            (0.21, 0.01),
            (-1.00, 0.50),
            (0.70, -0.80),
        )
        joint_velocity = []
        for vx, vy in candidates:
            joint_velocity.extend((-vx, -vy, 0.0))
        velocity, count = kinematics.body_velocity_measurement(
            [0.0] * 12,
            joint_velocity,
            (0.0, 0.0, 0.0),
            reference_velocity=(0.2, 0.0),
        )
        self.assertEqual(count, 2)
        self.assertEqual(kinematics.last_inliers, (True, True, False, False))
        self.assertAlmostEqual(float(velocity[0]), 0.205)

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(math.radians(370)), math.radians(10))


if __name__ == "__main__":
    unittest.main()
