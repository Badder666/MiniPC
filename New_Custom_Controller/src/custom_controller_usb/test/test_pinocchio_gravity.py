"""Partial feedback must not reset other joints' gravity calculation."""

import math
from pathlib import Path
import unittest

from custom_controller_usb.pinocchio_gravity import PinocchioGravityModel


class PartialFeedbackGravityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        model = Path(__file__).resolve().parents[1] / "config/controller_model_urdf2/controller_identified.urdf"
        cls.gravity = PinocchioGravityModel(model, (0.0, 0.0, -9.7883))

    def test_lost_distal_feedback_preserves_j2_torque(self):
        # Representative of the recorded 63 -> 3 mask transition.
        positions = [0.1, 1.3, 1.226, 0.2, -0.4, 0.1]
        mapping = ([0, 1, 2, 3, 4, 5], [-1., -1., 1., -1., 1., -1.],
                   [-0.0452, -0.0174, -0.0395, 0.0296, -0.0765, 0.0193],
                   [1., -1., 1., -1., 1., 1.])
        full, _ = self.gravity.torque_from_motor_positions(positions, *mapping)
        self.assertGreater(abs(full[2]), 0.5)
        for mask in (3, 11, 59, 0):
            with self.subTest(mask=mask):
                partial, actual_mask = self.gravity.torque_from_motor_positions(
                    positions, *mapping, valid_motor_mask=mask)
                self.assertEqual(actual_mask, mask)
                for i in range(6):
                    self.assertAlmostEqual(partial[i], full[i] if mask & (1 << i) else 0.0)

    def test_output_mask_uses_motor_order(self):
        positions = [0.2, 0.8, 0.5, 0.1, -0.3, 0.7]
        mapping = ([2, 0, 1, 5, 3, 4], [-1., 1., -1., 1., -1., 1.],
                   [0.1] * 6, [1., -1., 1., -1., 1., -1.])
        full, _ = self.gravity.torque_from_motor_positions(positions, *mapping, torque_scale=0.75)
        partial, mask = self.gravity.torque_from_motor_positions(
            positions, *mapping, torque_scale=0.75, valid_motor_mask=2)
        self.assertEqual(mask, 2)
        for i in range(6):
            self.assertAlmostEqual(partial[i], full[i] if i == 1 else 0.0)

    def test_nonfinite_pose_is_rejected_even_if_motor_is_invalid(self):
        for invalid in (math.nan, math.inf):
            with self.subTest(position=invalid):
                with self.assertRaises(ValueError):
                    self.gravity.torque_from_motor_positions(
                        [0., 0.4, invalid, 0., 0., 0.], range(6), [1.] * 6,
                        [0.] * 6, valid_motor_mask=3)


if __name__ == "__main__":
    unittest.main()
