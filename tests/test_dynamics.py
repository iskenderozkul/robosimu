import unittest

import numpy as np

from robosimu.dynamics import (forward_dynamics, gravity_torque, mass_matrix,
                               numeric_gravity_torque, rnea, total_energy)
from robosimu.model import load_model

M = load_model("models/six_axis_arm.json")
Q = np.array([0.2, -0.4, 0.6, -0.3, 0.5, 0.1])


class TestDynamics(unittest.TestCase):
    def test_gravity_matches_energy_gradient(self):
        g = gravity_torque(M, Q)
        gn = numeric_gravity_torque(M, Q)
        np.testing.assert_allclose(g, gn, atol=1e-6)

    def test_mass_matrix_symmetric_positive(self):
        Mq = mass_matrix(M, Q)
        np.testing.assert_allclose(Mq, Mq.T, atol=1e-9)
        self.assertGreater(np.min(np.linalg.eigvalsh(Mq)), 0.0)

    def test_inverse_dynamics_linear_in_acceleration(self):
        qd = np.zeros(M.n)
        a = np.array([1.0, 0.5, -0.3, 0.2, 0.1, -0.2])
        b = np.array([-0.4, 0.2, 0.1, -0.1, 0.3, 0.2])
        lhs = rnea(M, Q, qd, a + b)
        rhs = rnea(M, Q, qd, a) + rnea(M, Q, qd, b) - rnea(M, Q, qd, np.zeros(M.n))
        np.testing.assert_allclose(lhs, rhs, atol=1e-8)

    def test_passive_energy_conservation(self):
        q = np.array([0.3, 0.2, 0.1, 0.0, 0.0, 0.0])
        qd = np.zeros(M.n)
        e0 = total_energy(M, q, qd)
        dt = 1e-4
        for _ in range(1500):
            qdd = forward_dynamics(M, q, qd, np.zeros(M.n), with_friction=False)
            qd = qd + dt * qdd
            q = q + dt * qd
        e1 = total_energy(M, q, qd)
        self.assertLess(abs(e1 - e0), 0.5)


if __name__ == "__main__":
    unittest.main()
