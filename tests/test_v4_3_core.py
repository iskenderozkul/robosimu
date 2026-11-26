"""V4.3 core tests (Faz 1 + Faz 2), mandated by the locked decision list.

Covers the five required checks:

* ``test_strict_oracle_firewall``      — pixels + timestamps only, no ``set_delay``
* ``test_broyden_norm_boundedness``    — Frobenius trust set is respected
* ``test_temporal_alignment_indexing`` — linear interpolation at capture instants
* ``test_gaze_reafference_subtraction``— neck compensation before arm update
* ``test_exact_group_delay_flag``      — legacy vs V4.3 delay convention
"""
import unittest

import numpy as np

from robosimu.controllers import GovernorCartesian
from robosimu.kinematics import end_effector_position
from robosimu.model import load_model
from robosimu.simulator import SimConfig, simulate_reach
from robosimu.stereo import StereoCameraConfig
from robosimu.v43 import (CorrelationAwareWindow, EfferenceRingBuffer,
                          OrderPreservingBroyden, TimestampedObservation,
                          V43Controller)

M = load_model("models/six_axis_arm.json")
Q0 = np.array([0.0, -0.3, 0.6, 0.0, 0.6, 0.0])


def _packet(t=0.0, delay=0.06, hand=(340.0, 250.0), target=(330.0, 245.0)):
    h = np.array(hand, float)
    tgt = np.array(target, float)
    return TimestampedObservation(
        t_cap=t, t_arr=t + delay, t_now=t + delay,
        hand=np.array([[h[0] - 2.0, h[1]], [h[0] + 2.0, h[1]]]),
        target=np.array([[tgt[0] - 2.0, tgt[1]], [tgt[0] + 2.0, tgt[1]]]),
    )


class FirewallProbe(V43Controller):
    """Records every observation and raises if the plant leaks the delay."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.seen = []
        self.set_delay_called = False

    def set_delay(self, delay):  # pragma: no cover - must never be reached
        self.set_delay_called = True
        raise AssertionError("oracle leak: set_delay() reached a V4.3 controller")

    def step_obs(self, obs):
        self.seen.append(obs)
        return super().step_obs(obs)


class TestOracleFirewall(unittest.TestCase):
    def test_strict_oracle_firewall(self):
        # 1. the packet has no metric/oracle field at all
        fields = set(TimestampedObservation.__dataclass_fields__)
        self.assertEqual(fields, {"t_cap", "t_arr", "t_now", "hand", "target"})
        pkt = _packet()
        for banned in ("rel_base_gap_m", "rel_cam_gap_m", "depth_m", "q_true",
                       "J_true", "delay_s"):
            self.assertFalse(hasattr(pkt, banned), banned)

        # 2. a V4.3 controller exposes no delay-setting entry point
        ctl = V43Controller()
        self.assertFalse(hasattr(ctl, "set_delay"))
        self.assertTrue(ctl.zero_oracle)

        # 3. full plant loop: only TimestampedObservation arrives, no leak call
        probe = FirewallProbe(fps=100.0, dt=0.01)
        target = end_effector_position(M, Q0) + np.array([0.08, 0.0, 0.0])
        cfg = SimConfig(duration=0.30, delay=0.08, noise=0.0, eps=0.004,
                        stereo=StereoCameraConfig(), zero_oracle=True)
        r = simulate_reach(M, Q0, target, probe, cfg)

        self.assertFalse(probe.set_delay_called)
        self.assertGreater(len(probe.seen), 10)
        for obs in probe.seen:
            self.assertIsInstance(obs, TimestampedObservation)
            self.assertEqual(set(obs.__dict__),
                             {"t_cap", "t_arr", "t_now", "hand", "target"})
            self.assertEqual(np.asarray(obs.hand).shape, (2, 2))
            self.assertEqual(np.asarray(obs.target).shape, (2, 2))
            # delay is inferred from timestamps only (the very first frame is
            # clamped at t = 0, hence a shorter observed delay there)
            self.assertLessEqual(obs.delta_obs, 0.08 + 1e-9)
        self.assertAlmostEqual(max(o.delta_obs for o in probe.seen), 0.08, places=9)
        self.assertAlmostEqual(probe.seen[-1].delta_obs, 0.08, places=9)
        self.assertTrue(np.all(np.isfinite(r.q)))
        self.assertTrue(r.metrics["zero_oracle"])
        self.assertGreater(r.metrics["delta_obs_s"], 0.0)

    def test_retinal_error_is_pixel_only(self):
        pkt = _packet(hand=(340.0, 250.0), target=(330.0, 245.0))
        e = pkt.retinal_error
        self.assertAlmostEqual(e[0], 10.0)    # mean-u difference
        self.assertAlmostEqual(e[1], 5.0)     # mean-v difference
        self.assertAlmostEqual(e[2], 0.0)     # equal baselines -> no disparity diff


class TestTemporalAlignment(unittest.TestCase):
    def test_temporal_alignment_indexing(self):
        buf = EfferenceRingBuffer(dim=5)
        buf.push(0.00, np.zeros(5))
        buf.push(0.02, np.ones(5))
        # midpoint interpolation: the whole point of the capture-time buffer
        np.testing.assert_allclose(buf.state_at(0.01), 0.5 * np.ones(5))
        np.testing.assert_allclose(buf.state_at(0.00), np.zeros(5))
        np.testing.assert_allclose(buf.state_at(0.02), np.ones(5))
        # constant extrapolation outside the recorded span
        np.testing.assert_allclose(buf.state_at(-0.5), np.zeros(5))
        np.testing.assert_allclose(buf.state_at(5.0), np.ones(5))
        # delta between two capture instants that straddle an update
        np.testing.assert_allclose(buf.delta_between(0.00, 0.01), 0.5 * np.ones(5))
        # out-of-order push must not corrupt monotone time
        buf.push(0.005, 7.0 * np.ones(5))
        self.assertEqual(len(buf.t), 2)
        self.assertLess(buf.t[0], buf.t[1])
        # interpolation is exact at an off-grid sample between two updates
        buf2 = EfferenceRingBuffer(dim=2)
        buf2.push(0.0, np.array([0.0, 10.0]))
        buf2.push(0.1, np.array([1.0, 0.0]))
        np.testing.assert_allclose(buf2.state_at(0.03), np.array([0.3, 7.0]))


class TestBroyden(unittest.TestCase):
    def test_broyden_norm_boundedness(self):
        b = OrderPreservingBroyden(d_arm=5, d_neck=2, delta_j_max=0.5)
        b.set_initial_jacobians(J_r0=np.eye(3, 5) * 10.0,
                                J_neck0=np.ones((3, 2)))
        rng = np.random.default_rng(7)
        for _ in range(400):
            dq = rng.normal(scale=0.05, size=5)
            b.update_arm(rng.normal(scale=1e3, size=3), dq, qd_neck_norm=0.0)
            dqn = rng.normal(scale=0.05, size=2)
            b.update_neck(rng.normal(scale=1e3, size=3), dqn, np.zeros(5))
        # each matrix is clamped inside its own Frobenius ball
        self.assertLessEqual(float(np.linalg.norm(b.J_r - b.J_r0, "fro")),
                             0.5 + 1e-9)
        self.assertLessEqual(float(np.linalg.norm(b.J_neck - b.J_neck0, "fro")),
                             0.5 + 1e-9)
        self.assertLessEqual(b.frobenius_residual(), 1.0 + 1e-9)
        self.assertTrue(np.all(np.isfinite(b.J_r)))

    def test_arm_update_gated_when_neck_dominates(self):
        b = OrderPreservingBroyden(d_arm=5, d_neck=2, eta_dom=0.5)
        b.set_initial_jacobians(J_r0=np.eye(3, 5), J_neck0=np.zeros((3, 2)))
        before = b.J_r.copy()
        updated = b.update_arm(np.ones(3), np.array([0.01, 0, 0, 0, 0]),
                               qd_neck_norm=0.5)
        self.assertFalse(updated)
        np.testing.assert_allclose(b.J_r, before)


class TestGazeReafference(unittest.TestCase):
    def test_gaze_reafference_subtraction(self):
        b = OrderPreservingBroyden(d_arm=5, d_neck=2, delta_j_max=1e9)
        b.set_initial_jacobians(J_r0=np.zeros((3, 5)),
                                J_neck0=np.array([[10.0, 0.0],
                                                  [0.0, 20.0],
                                                  [0.0, 0.0]]))
        de = np.array([3.0, -4.0, 1.0])
        dq_neck = np.array([0.2, 0.1])
        expected = de - np.array([2.0, 2.0, 0.0])
        np.testing.assert_allclose(b.compensate(de, dq_neck), expected)
        # compensation uses the *previous* J_neck: it must not change it
        np.testing.assert_allclose(b.J_neck, b.J_neck0)

    def test_neck_updates_only_while_arm_still(self):
        b = OrderPreservingBroyden(d_arm=5, d_neck=2, delta_j_max=1e9)
        b.set_initial_jacobians(J_r0=np.zeros((3, 5)),
                                J_neck0=np.zeros((3, 2)))
        moved = b.update_neck(np.array([1.0, 0.0, 0.0]),
                              np.array([0.05, 0.0]), np.zeros(5))
        self.assertTrue(moved)
        frozen = b.J_neck.copy()
        moved_again = b.update_neck(np.array([1.0, 0.0, 0.0]),
                                    np.array([0.05, 0.0]),
                                    np.array([0.3, 0, 0, 0, 0]))
        self.assertFalse(moved_again)
        np.testing.assert_allclose(b.J_neck, frozen)

    def test_neck_command_foveates_target(self):
        ctl = V43Controller(gaze_comp=True)
        pkt = _packet(target=(400.0, 240.0))     # target right of the fovea
        dq = ctl.neck_command(pkt)
        self.assertGreater(dq[0], 0.0)          # pan towards the target
        self.assertAlmostEqual(dq[1], 0.0, places=12)
        ctl.gaze_comp = False
        np.testing.assert_allclose(ctl.neck_command(pkt), np.zeros(2))


class TestExactGroupDelay(unittest.TestCase):
    def _gov(self, exact):
        return GovernorCartesian(delay=0.12, fps=100.0, eps=0.003,
                                 sigma=0.0005, exact_group_delay=exact)

    def test_exact_group_delay_flag(self):
        legacy = self._gov(False)
        exact = self._gov(True)

        # window upper bound: floor(2 delta / T) vs floor(2 delta / T) + 1
        self.assertEqual(legacy.n_max_window, 24)
        self.assertEqual(exact.n_max_window, 25)

        # with a small required window the two conventions differ by T/2
        self.assertEqual(int(legacy.n[0]), 1)
        self.assertEqual(int(exact.n[0]), 1)
        self.assertAlmostEqual(float(legacy.delta_eff), 0.12 + 0.005)
        self.assertAlmostEqual(float(exact.delta_eff), 0.12)

        # no double counting -> the exact gain is the larger (faster) one
        self.assertGreater(float(exact.k), float(legacy.k))
        self.assertAlmostEqual(float(exact.k / legacy.k), 0.125 / 0.120, places=6)

        # the legacy convention remains the default
        default = GovernorCartesian(delay=0.12, fps=100.0, eps=0.003,
                                    sigma=0.0005)
        self.assertFalse(default.exact_group_delay)
        self.assertAlmostEqual(float(default.k), float(legacy.k))

    def test_v43_uses_exact_delay_internally(self):
        ctl = V43Controller(fps=100.0, delay_align=True)
        ctl.step_obs(_packet(t=0.0, delay=0.12))
        ctl.step_obs(_packet(t=0.01, delay=0.12))
        info = ctl.info
        self.assertAlmostEqual(info["delta_obs"], 0.12, places=9)
        self.assertEqual(info["n_max"], 25)              # floor(2*0.12/0.01)+1
        self.assertAlmostEqual(info["delta_eff"],
                               0.12 + (info["n_star"] - 1) * 0.005, places=12)
        self.assertAlmostEqual(info["k"], 1.0 / (np.e * info["delta_eff"]),
                               places=12)


class TestCorrelationWindow(unittest.TestCase):
    def test_ar1_variance_and_selection(self):
        w = CorrelationAwareWindow(alpha=0.05)
        # white noise: V(n) = sigma^2 / n
        self.assertAlmostEqual(w.variance(1, 2.0, 0.0), 4.0)
        self.assertAlmostEqual(w.variance(4, 2.0, 0.0), 1.0)
        # positive correlation inflates the variance
        self.assertGreater(w.variance(4, 2.0, 0.6), w.variance(4, 2.0, 0.0))

        # calibration recovers a known AR(1) correlation
        rng = np.random.default_rng(3)
        rho, x = 0.7, np.zeros((4000, 3))
        for k in range(1, 4000):
            x[k] = rho * x[k - 1] + rng.normal(scale=1.0, size=3)
        w.calibrate(x)
        self.assertTrue(w.calibrated)
        self.assertTrue(np.all(np.abs(w.rho - 0.7) < 0.06))

        # a 2 px tolerance forbids short windows but allows long ones
        n_star, ok = w.select(np.full(3, 2.0), n_max=25)
        self.assertTrue(ok)
        self.assertGreater(n_star, 1)
        self.assertLessEqual(n_star, 25)
        # an impossible tolerance degrades to n_max with is_admissible=False
        n_bad, ok_bad = w.select(np.full(3, 1e-6), n_max=9)
        self.assertFalse(ok_bad)
        self.assertEqual(n_bad, 9)

    def test_stale_flag(self):
        self.assertTrue(CorrelationAwareWindow.is_stale(0.5, 0.10, 0.05))
        self.assertFalse(CorrelationAwareWindow.is_stale(0.12, 0.10, 0.05))


if __name__ == "__main__":
    unittest.main()
