"""Regression checks against a delayed, heterogeneous synthetic plant."""
import unittest
from tools.benchmark_control_loops import heading, velocity
from dual_dog.control import feedback, speed_sync


class ControlPlantTests(unittest.TestCase):
    def test_six_direction_velocity_spread_with_gain_mismatch(self):
        for case in velocity(speed_sync):
            with self.subTest(axis=case['axis'], sign=case['sign']):
                self.assertLess(case['steady_spread'], .015)

    def test_heading_settles_despite_delay_and_actuation_deadzone(self):
        for case in heading(feedback):
            with self.subTest(angle=case['initial_deg']):
                self.assertIsNotNone(case['settle_1deg_s'])
                self.assertLessEqual(case['settle_1deg_s'], 3.)
                self.assertLess(case['last_2s_peak_deg'], .6)
