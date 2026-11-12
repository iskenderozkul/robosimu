import unittest

import numpy as np

from robosimu.controllers import FixedCartesian, GovernorCartesian
from robosimu.kinematics import end_effector_position
from robosimu.model import load_model
from robosimu.simulator import SimConfig, simulate_reach

M = load_model("models/six_axis_arm.json")
Q0 = np.array([0.0, -0.3, 0.6, 0.0, 0.6, 0.0])


class TestSimulation(unittest.TestCase):
    def test_short_reach_is_finite_and_improves(self):
        target = end_effector_position(M, Q0) + np.array([0.08, 0.0, 0.0])
        cfg = SimConfig(duration=0.5, delay=0.06, noise=0.0, eps=0.004)
        r = simulate_reach(M, Q0, target, GovernorCartesian(delay=0.06, fps=100), cfg)
        self.assertTrue(np.all(np.isfinite(r.q)))
        self.assertLess(r.error[-1], r.error[0])

    def test_governor_is_slower_than_fixed_at_high_delay(self):
        target = end_effector_position(M, Q0) + np.array([0.1, 0.0, 0.0])
        cfg = SimConfig(duration=0.6, delay=0.18, noise=0.0, eps=0.004)
        g = GovernorCartesian(delay=0.18, fps=100, eps=0.004)
        f = FixedCartesian(k=6.0, vmax=0.6, eps=0.004)
        rg = simulate_reach(M, Q0, target, g, cfg)
        rf = simulate_reach(M, Q0, target, f, cfg)
        self.assertLessEqual(rg.metrics["final_error_m"], rf.metrics["final_error_m"] + 0.01)
        self.assertTrue(np.isfinite(rg.metrics["energy_j"]))

    def test_dead_zone_stops(self):
        c = FixedCartesian(k=6.0, eps=0.01)
        np.testing.assert_allclose(c(np.array([0.005, 0.0, 0.0])), 0.0)


if __name__ == "__main__":
    unittest.main()
