from dataclasses import replace
import unittest

from dual_dog.control.adaptive_slip import AdaptiveSlipCompensator
from dual_dog.models import SafetyConfig


class AdaptiveSlipCompensatorTests(unittest.TestCase):
    def setUp(self):
        self.safety = replace(
            SafetyConfig(),
            adaptive_slip_warmup_s=0.10,
            adaptive_slip_learning_rate=1.0,
            adaptive_slip_slew_rate=1.0,
        )
        self.compensator = AdaptiveSlipCompensator(self.safety)

    def test_forward_right_drift_learns_left_correction(self):
        result = None
        for index in range(20):
            result = self.compensator.apply(
                0.4,
                0.0,
                (0.4, -0.08, 0.0),
                1.0 + index * 0.05,
                sample_token=(index,),
                support_legs=2,
            )
        self.assertIsNotNone(result)
        self.assertGreater(result.correction_y, 0.0)
        self.assertEqual(result.correction_x, 0.0)
        self.assertIn("前进/学习", result.status)

    def test_each_direction_has_an_independent_learned_value(self):
        for index in range(15):
            self.compensator.apply(
                0.4, 0.0, (0.4, -0.08), 1.0 + index * 0.05,
                sample_token=("forward", index),
            )
        forward_bias = self.compensator.biases["forward"]
        self.compensator.apply(0.0, 0.25, (0.06, 0.25), 2.0, sample_token=("left", 0))
        for index in range(1, 15):
            self.compensator.apply(
                0.0, 0.25, (0.06, 0.25), 2.0 + index * 0.05,
                sample_token=("left", index),
            )
        self.assertGreater(forward_bias, 0.0)
        self.assertLess(self.compensator.biases["left"], 0.0)
        self.assertEqual(self.compensator.biases["backward"], 0.0)

    def test_large_drift_can_learn_beyond_old_point_zero_five_cap(self):
        result = None
        for index in range(100):
            result = self.compensator.apply(
                0.4, 0.0, (0.4, -0.5), 1.0 + index * 0.05,
                sample_token=(index,),
            )
        self.assertIsNotNone(result)
        self.assertGreater(result.learned_bias, 0.05)
        self.assertLessEqual(
            abs(result.learned_bias), self.safety.adaptive_slip_hard_limit
        )

    def test_repeated_telemetry_token_does_not_learn_twice(self):
        self.compensator.apply(0.4, 0.0, (0.4, -0.1), 1.0, sample_token=(1,))
        learned = self.compensator.apply(
            0.4, 0.0, (0.4, -0.1), 1.2, sample_token=(2,)
        ).learned_bias
        repeated = self.compensator.apply(
            0.4, 0.0, (0.4, -0.1), 1.25, sample_token=(2,)
        ).learned_bias
        self.assertEqual(learned, repeated)

    def test_stop_freezes_learning_and_outputs_no_translation(self):
        for index in range(10):
            self.compensator.apply(
                0.4, 0.0, (0.4, -0.1), 1.0 + index * 0.05,
                sample_token=(index,),
            )
        learned = self.compensator.biases["forward"]
        stopped = self.compensator.apply(0.0, 0.0, (0.0, -0.1), 2.0)
        self.assertEqual((stopped.correction_x, stopped.correction_y), (0.0, 0.0))
        self.assertEqual(self.compensator.biases["forward"], learned)


if __name__ == "__main__":
    unittest.main()
