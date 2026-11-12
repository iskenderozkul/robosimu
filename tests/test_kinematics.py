import unittest

import numpy as np

from robosimu.kinematics import (end_effector_position, jacobian,
                                 point_position, point_velocity)
from robosimu.model import load_model

M = load_model("models/six_axis_arm.json")
Q = np.array([0.2, -0.4, 0.6, -0.3, 0.5, 0.1])


class TestKinematics(unittest.TestCase):
    def test_fk_changes_with_joint(self):
        q2 = Q.copy()
        q2[0] += 0.1
        self.assertGreater(np.linalg.norm(end_effector_position(M, q2) - end_effector_position(M, Q)), 1e-4)

    def test_jacobian_matches_finite_difference(self):
        J = jacobian(M, Q, M.n - 1)
        eps = 1e-6
        for j in range(M.n):
            dq = np.zeros(M.n)
            dq[j] = eps
            p1 = end_effector_position(M, Q + dq)
            p0 = end_effector_position(M, Q - dq)
            fd = (p1 - p0) / (2 * eps)
            np.testing.assert_allclose(J[:3, j], fd, atol=1e-5)

    def test_point_velocity(self):
        qd = np.array([0.1, -0.2, 0.3, 0.0, 0.1, 0.2])
        v, w = point_velocity(M, Q, qd, 2, [0.1, 0, 0])
        self.assertEqual(v.shape, (3,))
        self.assertEqual(w.shape, (3,))
        self.assertGreater(np.linalg.norm(v), 0)

    def test_point_position(self):
        p = point_position(M, Q, 5, [0, 0, 0])
        np.testing.assert_allclose(p, end_effector_position(M, Q), atol=1e-9)


if __name__ == "__main__":
    unittest.main()
