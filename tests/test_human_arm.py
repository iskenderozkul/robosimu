"""Tests for the human-limb-like demonstration arm."""
import unittest

import numpy as np

from robosimu.model import load_model, load_postures
from robosimu.kinematics import end_effector_position, forward_kinematics
from robosimu.dynamics import gravity_torque


class HumanArmModelTest(unittest.TestCase):
    def setUp(self):
        self.m = load_model("models/human_arm.json")
        self.postures = load_postures("models/human_arm.json")

    def test_segments_human_proportions(self):
        upper = self.m.links[1].a
        fore = self.m.links[2].a
        self.assertAlmostEqual(upper, 0.31, places=6)
        self.assertAlmostEqual(fore, 0.26, places=6)
        self.assertAlmostEqual(upper + fore, 0.57, places=6)

    def test_limits_within_human_ranges(self):
        self.assertTrue(np.all(np.degrees([l.limits[0] for l in self.m.links]) <
                               np.degrees([l.limits[1] for l in self.m.links])))

    def test_hanging_pose_is_dead_vertical(self):
        p = end_effector_position(self.m, np.zeros(6))
        self.assertAlmostEqual(float(p[0]), 0.0, places=6)
        # The kinematic base sits at the left shoulder (base_origin y = 0.19)
        self.assertAlmostEqual(float(p[1]), 0.19, places=6)
        self.assertLess(float(p[2]), 0.30)

    def test_hanging_pose_gravity_torque_is_zero(self):
        tau = gravity_torque(self.m, np.zeros(6))
        self.assertLess(float(np.max(np.abs(tau))), 0.5)

    def test_wrist_axis_hand_points_down(self):
        _, frames = forward_kinematics(self.m, np.zeros(6))
        z6 = frames[5][:3, 2]
        self.assertAlmostEqual(abs(float(z6[2])), 1.0, places=6)

    def test_postures_load_and_shape(self):
        self.assertIn("hanging_natural", self.postures)
        self.assertIn("hanging_vertical", self.postures)
        self.assertIn("ready_desk", self.postures)
        self.assertIn("reach_high", self.postures)
        for name, angles in self.postures.items():
            self.assertEqual(len(angles), 6)

    def test_postures_within_limits(self):
        for name, angles in self.postures.items():
            q = np.radians(angles)
            for i, l in enumerate(self.m.links):
                lo, hi = l.limits
                self.assertGreaterEqual(q[i], lo - 1e-6, f"{name} j{i+1} below limit")
                self.assertLessEqual(q[i], hi + 1e-6, f"{name} j{i+1} above limit")

    def test_ready_desk_pose_values(self):
        q = np.radians(self.postures["ready_desk"])
        p = end_effector_position(self.m, q)
        self.assertAlmostEqual(float(p[0]), 0.6, delta=0.05)
        self.assertGreater(float(p[2]), 0.7)

    def test_with_payload_raises_shoulder_torque(self):
        q = np.radians(self.postures["ready_desk"])
        free = gravity_torque(self.m, q)
        loaded = gravity_torque(self.m.with_payload(2.0), q)
        self.assertGreater(float(np.abs(loaded).max()),
                           float(np.abs(free).max()))
