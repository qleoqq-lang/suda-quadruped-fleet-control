import csv
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dual_dog.diagnostics import StraightLineCsvRecorder
from dual_dog.models import Telemetry


class DiagnosticsRecorderTests(unittest.TestCase):
    def test_records_commands_estimates_and_leg_inliers(self):
        with TemporaryDirectory() as directory:
            recorder = StraightLineCsvRecorder(Path(directory))
            path = recorder.path
            data = Telemetry(
                "dog01",
                connected=True,
                rpy=(0.0, 0.0, 0.1),
                imu_angular_velocity=(0.0, 0.0, 0.02),
                leg_velocity_body=(0.2, 0.03, 0.0),
                corrected_velocity_world=(0.19, 0.025),
                corrected_yaw_rate=0.018,
                corrected_pose=(1.0, 0.1, 5.0),
                odom_uncertainty=(0.02, 0.03, 1.0),
                odom_support_legs=2,
                leg_velocity_candidates_body=(
                    (0.2, 0.02), (0.21, 0.03), (-0.5, 0.4), (0.7, -0.3)
                ),
                leg_velocity_inliers=(True, True, False, False),
                commanded_velocity=(0.2, -0.01, 0.0),
                adaptive_slip_direction="forward",
                adaptive_slip_cross_velocity=-0.04,
                adaptive_slip_bias=0.03,
                adaptive_slip_correction=(0.0, 0.025),
                motion_state=6,
                gait=0,
                sub_gait=-1,
            )
            recorder.record([data])
            recorder.close()

            with path.open("r", encoding="utf-8-sig", newline="") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["robot_id"], "dog01")
            self.assertEqual(row["cmd_y"], "-0.01")
            self.assertEqual(row["ekf_vy_world"], "0.025")
            self.assertEqual(row["fl_used"], "1")
            self.assertEqual(row["hl_used"], "0")
            self.assertEqual(row["adaptive_direction"], "forward")
            self.assertEqual(row["adaptive_bias"], "0.03")
            self.assertEqual(row["adaptive_correction_y"], "0.025")
            self.assertEqual(row["motion_state"], "6")
            self.assertEqual(row["gait"], "0")
            self.assertEqual(row["sub_gait"], "-1")


if __name__ == "__main__":
    unittest.main()
