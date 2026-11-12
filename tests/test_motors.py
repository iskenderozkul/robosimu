import unittest

import numpy as np

from robosimu.model import load_model
from robosimu.motors import drive, torque_limit

M = load_model("models/six_axis_arm.json")


class TestMotors(unittest.TestCase):
    def test_stall_torque(self):
        for link in M.links:
            self.assertAlmostEqual(torque_limit(link.motor, 0.0), link.motor.torque_limit, places=6)

    def test_limit_decreases_with_speed(self):
        motor = M.links[1].motor
        self.assertLess(torque_limit(motor, 2.0), torque_limit(motor, 0.0))

    def test_drive_power_positive(self):
        s = drive(M.links[1].motor, 20.0, 0.5, 0.01)
        self.assertGreaterEqual(s.p_electrical, 0.0)
        self.assertLessEqual(abs(s.tau_joint), s.tau_limit + 1e-9)

    def test_saturation_flag(self):
        s = drive(M.links[0].motor, 1e5, 0.0, 0.01)
        self.assertTrue(s.saturated)
        self.assertLessEqual(s.tau_joint, s.tau_limit + 1e-9)


if __name__ == "__main__":
    unittest.main()
