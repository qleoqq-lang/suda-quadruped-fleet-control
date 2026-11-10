import unittest

from dual_dog.models import Telemetry
from dual_dog.control.speed_sync import FormationProgressSynchronizer, VelocitySynchronizer


class VelocitySynchronizerTests(unittest.TestCase):
    def test_duplicate_sensor_sample_does_not_wind_up(self):
        sync = VelocitySynchronizer()
        first = sync.apply(.3, 0., (0.,0.), 1., sample_token=1)
        integral = sync.integral_x
        for tick in range(20):
            self.assertEqual(sync.apply(.3,0.,(0.,0.),1.01+tick*.02,sample_token=1), first)
        self.assertEqual(sync.integral_x, integral)

    def test_actuator_saturation_does_not_accumulate_integral(self):
        sync = VelocitySynchronizer()
        for tick in range(50):
            output, _, _ = sync.apply(.3,0.,(0.,0.),1.+tick*.05,limits=(.3,.3))
            self.assertLessEqual(output, .3)
        self.assertEqual(sync.integral_x, 0.)

    def test_reversal_and_zero_do_not_retain_old_integral(self):
        sync = VelocitySynchronizer()
        for tick in range(50):
            sync.apply(.3,0.,(.1,0.),1.+tick*.05)
        output, _, _ = sync.apply(-.3,0.,(-.3,0.),4.)
        self.assertLessEqual(output, -.3)
        output, _, _ = sync.apply(0.,0.,(-.2,0.),4.1)
        self.assertEqual(output, 0.)
        self.assertEqual(sync.integral_x, 0.)

    def test_pulses_preserve_average_and_never_reverse(self):
        from dual_dog.control.speed_sync import ActuationPulse
        pulse = ActuationPulse()
        for target in (.10, -.10):
            values = [pulse.apply(target, .25) for _ in range(100)]
            self.assertAlmostEqual(sum(values)/100, target, delta=.003)
            self.assertTrue(all(v*target >= 0 for v in values))
        self.assertEqual(pulse.apply(0., .25), 0.)

    def test_slow_robot_receives_more_command(self):
        sync = VelocitySynchronizer()
        command_x, _, _ = sync.apply(0.4, 0.0, (0.2, 0.0, 0.0), 1.0)
        self.assertGreater(command_x, 0.4)

    def test_first_equal_measurement_does_not_create_artificial_boost(self):
        sync = VelocitySynchronizer()
        command_x, command_y, _ = sync.apply(
            0.25, 0.0, (0.25, 0.0, 0.0), 1.0
        )
        self.assertAlmostEqual(command_x, 0.25, places=6)
        self.assertEqual(command_y, 0.0)

    def test_fast_robot_receives_less_command_after_filter_settles(self):
        sync = VelocitySynchronizer()
        command_x = 0.0
        for index in range(12):
            command_x, _, _ = sync.apply(
                0.4, 0.0, (0.6, 0.0, 0.0), 1.0 + index * 0.05
            )
        self.assertLess(command_x, 0.4)

    def test_zero_target_never_creates_motion(self):
        sync = VelocitySynchronizer()
        command_x, command_y, _ = sync.apply(
            0.0, 0.0, (0.4, -0.3, 0.0), 1.0
        )
        self.assertEqual((command_x, command_y), (0.0, 0.0))

    def test_trim_is_bounded(self):
        sync = VelocitySynchronizer()
        command_x, _, _ = sync.apply(0.4, 0.0, (-10.0, 0.0, 0.0), 1.0)
        self.assertLessEqual(command_x, 0.4 + sync.MAX_TRIM)


class FormationProgressSynchronizerTests(unittest.TestCase):
    def test_sustained_saturation_reduces_group_reference(self):
        sync = FormationProgressSynchronizer()
        targets = ('dog01','dog02')
        result = None
        for tick in range(30):
            data = {robot: self.data(robot) for robot in targets}
            data['dog01'].commanded_velocity = (0.3, 0., 0.)
            data['dog01'].corrected_velocity_world = (0.20, 0.)
            data['dog02'].commanded_velocity = (0.25, 0., 0.)
            data['dog02'].corrected_velocity_world = (0.30, 0.)
            result = sync.apply(targets,.3,0.,data,1.+tick*.1,limits=(.3,.3))
        self.assertLess(result['dog02'][0], .25)
        self.assertGreater(result['dog02'][0], .15)

    def test_stale_pose_is_not_used_as_fleet_reference(self):
        sync = FormationProgressSynchronizer()
        data = {robot:self.data(robot) for robot in ('dog01','dog02')}
        data['dog02'].updated_at = 1.
        result = sync.apply(tuple(data),.3,0.,data,2.)
        self.assertIn('等待至少两台', result['dog01'][2])

    @staticmethod
    def data(robot_id, x=0.0, y=0.0, yaw=0.0, connected=True):
        return Telemetry(
            robot_id,
            connected=connected,
            corrected_pose=(x, y, yaw),
            odom_uncertainty=(0.02, 0.02, 1.0),
        )

    def test_lagging_robot_accelerates_and_leader_slows(self):
        sync = FormationProgressSynchronizer()
        initial = {
            "dog01": self.data("dog01"),
            "dog02": self.data("dog02"),
        }
        sync.apply(("dog01", "dog02"), 0.25, 0.0, initial, 1.0)
        moved = {
            "dog01": self.data("dog01", x=0.20),
            "dog02": self.data("dog02", x=0.10),
        }
        result = sync.apply(("dog01", "dog02"), 0.25, 0.0, moved, 1.1)
        self.assertLess(result["dog01"][0], 0.25)
        self.assertGreater(result["dog02"][0], 0.25)
        self.assertIn("队形同步", result["dog02"][2])

    def test_horizontal_progress_uses_lateral_distance(self):
        sync = FormationProgressSynchronizer()
        initial = {
            "dog01": self.data("dog01"),
            "dog02": self.data("dog02"),
        }
        sync.apply(("dog01", "dog02"), 0.0, -0.25, initial, 1.0)
        moved = {
            "dog01": self.data("dog01", y=-0.20),
            "dog02": self.data("dog02", y=-0.10),
        }
        result = sync.apply(("dog01", "dog02"), 0.0, -0.25, moved, 1.1)
        self.assertGreater(result["dog01"][1], -0.25)
        self.assertLess(result["dog02"][1], -0.25)

    def test_missing_second_pose_leaves_commands_unchanged(self):
        sync = FormationProgressSynchronizer()
        snapshots = {
            "dog01": self.data("dog01"),
            "dog02": Telemetry("dog02", connected=True),
        }
        result = sync.apply(("dog01", "dog02"), 0.25, 0.0, snapshots, 1.0)
        self.assertEqual(result["dog01"][:2], (0.25, 0.0))
        self.assertEqual(result["dog02"][:2], (0.25, 0.0))

    def test_trim_is_bounded_and_never_reverses(self):
        sync = FormationProgressSynchronizer()
        initial = {
            "dog01": self.data("dog01"),
            "dog02": self.data("dog02"),
        }
        sync.apply(("dog01", "dog02"), 0.05, 0.0, initial, 1.0)
        result = None
        for index in range(1, 30):
            moved = {
                "dog01": self.data("dog01", x=2.0),
                "dog02": self.data("dog02", x=0.0),
            }
            result = sync.apply(
                ("dog01", "dog02"), 0.05, 0.0, moved, 1.0 + index * 0.05
            )
        self.assertIsNotNone(result)
        self.assertGreaterEqual(result["dog01"][0], 0.0)
        self.assertLessEqual(result["dog02"][0], 0.05 + sync.MAX_TRIM)


if __name__ == "__main__":
    unittest.main()
