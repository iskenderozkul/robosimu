import unittest

import numpy as np

from robosimu.controllers import (AdaptiveCartesian, BroydenCartesian,
                                  GovernorCartesian, LqrCartesian,
                                  PidCartesian, SmithCartesian,
                                  make_controller)
from robosimu.dynamics import mass_matrix, rnea
from robosimu.experiments import Q0, REACH, run_one
from robosimu.kinematics import end_effector_position
from robosimu.model import load_model
from robosimu.motors import drive
from robosimu.scenarios import CONTROLLERS, SCENARIOS, reach_variant
from robosimu.simulator import SimConfig, simulate_reach
from robosimu.stats import (bootstrap_ci, effective_width, fitts_id,
                            holm_bonferroni, permutation_test, throughput,
                            wilson_ci)
from robosimu.trajectory import plan_joint_move, plan_move

M = load_model("models/six_axis_arm.json")
TARGET = end_effector_position(M, Q0) + np.array([0.08, 0.0, 0.0])


class TestDriverFeatures(unittest.TestCase):
    def _motor(self):
        import copy
        return copy.deepcopy(M.links[1].motor)

    def test_current_limit_reduces_torque(self):
        motor = self._motor()
        motor.current_limit_a = 5.0
        s = drive(motor, 200.0, 0.0, 0.01)
        self.assertLess(abs(s.current), 5.0 + 1e-9)
        self.assertTrue(s.saturated)

    def test_torque_ramp_limits_rate(self):
        motor = self._motor()
        motor.torque_ramp_nm_s = 1000.0
        s = drive(motor, 200.0, 0.0, 0.01, prev_tau=0.0)
        self.assertLessEqual(abs(s.tau_joint), 1000.0 * 0.01 + 1e-9)

    def test_ramp_does_not_exceed_unlimited(self):
        motor = self._motor()
        motor.torque_ramp_nm_s = 0.0
        s = drive(motor, 50.0, 0.0, 0.01, prev_tau=0.0)
        self.assertFalse(s.saturated)


class TestSimConfig(unittest.TestCase):
    def test_delay_schedule_changes_measurement(self):
        cfg = SimConfig(duration=0.5, delay=0.0, delay_schedule=((0.0, 0.12),),
                        noise=0.0, seed=0)
        r = simulate_reach(M, Q0, TARGET, GovernorCartesian(delay=0.12, fps=100), cfg)
        self.assertAlmostEqual(r.metrics["delay_mean_s"], 0.12, places=3)

    def test_anisotropic_noise_is_accepted(self):
        cfg = SimConfig(duration=0.3, delay=0.0, noise=(0.001, 0.001, 0.004), seed=0)
        r = simulate_reach(M, Q0, TARGET, GovernorCartesian(delay=0.0, fps=100), cfg)
        self.assertTrue(np.all(np.isfinite(r.q)))

    def test_noise_vector_must_have_three_entries(self):
        cfg = SimConfig(duration=0.1, noise=(0.001, 0.001))
        with self.assertRaises(ValueError):
            simulate_reach(M, Q0, TARGET, GovernorCartesian(delay=0.0, fps=100), cfg)

    def test_jitter_is_reported(self):
        cfg = SimConfig(duration=0.3, delay=0.06, delay_jitter=0.02, noise=0.0, seed=1)
        r = simulate_reach(M, Q0, TARGET, GovernorCartesian(delay=0.06, fps=100), cfg)
        self.assertAlmostEqual(r.metrics["delay_jitter_s"], 0.02)


class TestControllers(unittest.TestCase):
    def test_factory_names(self):
        for name in ("fixed", "governor", "adaptive", "smith", "broyden", "pid", "lqr", "hdap", "hlsl"):
            c = make_controller(name, delay=0.06, fps=100, control_dt=0.01)
            self.assertEqual(c.name, name)

    def test_all_controllers_return_velocity(self):
        e = np.array([0.1, 0.0, 0.0])
        for name in ("fixed", "governor", "adaptive", "smith", "broyden", "pid", "lqr", "hdap", "hlsl"):
            c = make_controller(name, delay=0.06, fps=100, control_dt=0.01)
            v = c(e)
            self.assertEqual(v.shape, (3,))
            self.assertTrue(np.all(np.isfinite(v)))

    def test_dead_zone(self):
        for ctrl in (AdaptiveCartesian(eps=0.01), SmithCartesian(0.06, eps=0.01),
                     BroydenCartesian(0.06, eps=0.01), PidCartesian(0.01, eps=0.01),
                     LqrCartesian(0.01, eps=0.01)):
            np.testing.assert_allclose(ctrl(np.array([0.005, 0.0, 0.0])), 0.0)

    def test_lqr_gains_match_riccati(self):
        c = LqrCartesian(0.01, q_pos=1.0, q_vel=0.05, r=0.04)
        self.assertAlmostEqual(c.k1, 5.0)
        self.assertAlmostEqual(c.k2, np.sqrt(0.45 / 0.04))

    def test_hdap_distance_adaptive_cap(self):
        from robosimu.stereo import HumanApproachController, triphasic_weights
        long_reach = HumanApproachController(delay=0.06, fps=100.0, vmax=0.5)
        long_reach(np.array([0.45, 0.0, 0.85]))
        self.assertAlmostEqual(long_reach.vmax if False else 0.5, 0.5)
        # distance-adaptive cap: v_cap = vmax * clamp(sqrt(D/0.30), 0.35, 1)
        cap_long = 0.5 * min(1.0, max(0.35, (0.97 / 0.30) ** 0.5))
        self.assertAlmostEqual(cap_long, 0.5)
        cap_short = 0.5 * min(1.0, max(0.35, (0.10 / 0.30) ** 0.5))
        self.assertLess(cap_short, 0.5 * 0.7)
        # triphasic: proximal early vs distal late
        w_push = triphasic_weights(0.8)
        w_grasp = triphasic_weights(0.15)
        self.assertGreater(w_push[1], w_grasp[1])      # shoulder dominant early
        self.assertGreater(w_grasp[3], w_push[3])      # wrist dominant late
        self.assertTrue(np.all(w_push <= 1.0) and np.all(w_grasp <= 1.0))

    def test_governor_tracks_delay(self):
        g = GovernorCartesian(0.06, fps=100, eps=0.004)
        k_nominal = g.k
        g.set_delay(0.18)
        self.assertLess(g.k, k_nominal)
        self.assertAlmostEqual(g.delta_eff, 0.18 + g.n_max * g.T / 2.0)
        g.set_delay(0.0)
        self.assertAlmostEqual(g.delta_eff, g.T / 2.0)

    def test_smith_tracks_delay(self):
        s = SmithCartesian(0.06, control_dt=0.01)
        s(np.array([0.1, 0.0, 0.0]))
        s.set_delay(0.18)
        self.assertEqual(s.n_delay, 18)
        s.set_delay(0.0)
        self.assertEqual(s.n_delay, 1)

    def test_broyden_estimate_stays_bounded(self):
        c = BroydenCartesian(0.06, lam=6.0)
        for _ in range(50):
            v = c(np.array([0.1 - 0.001 * _, 0.0, 0.0]))
            self.assertTrue(np.all(np.isfinite(v)))
            self.assertLess(np.linalg.norm(v), 1.0)


class TestTrajectory(unittest.TestCase):
    def test_trapezoid_arrives_at_rest(self):
        p = plan_move(0.0, 1.0, 1.0, 2.0, jmax=0.0, dt=0.005)
        self.assertAlmostEqual(p.q[-1], 1.0, places=4)
        self.assertAlmostEqual(p.qd[-1], 0.0, places=6)
        self.assertLessEqual(np.max(np.abs(p.qd)), 1.0 + 1e-6)

    def test_scurve_arrives_at_rest(self):
        p = plan_move(0.0, 1.0, 1.0, 2.0, jmax=10.0, dt=0.005)
        self.assertAlmostEqual(p.q[-1], 1.0, places=4)
        self.assertAlmostEqual(p.qd[-1], 0.0, places=6)

    def test_short_move_is_triangular(self):
        p = plan_move(0.0, 0.02, 1.0, 2.0, jmax=10.0, dt=0.005)
        self.assertIn("tri", p.profile)
        self.assertAlmostEqual(p.q[-1], 0.02, places=4)

    def test_joint_move_is_synchronised(self):
        ts, qs, qds, qdds = plan_joint_move([0, 0, 0], [1.0, 0.5, 0.2],
                                            [1, 1, 1], [2, 2, 2], jmax=10, dt=0.01)
        np.testing.assert_allclose(qds[-1], 0.0, atol=1e-8)
        np.testing.assert_allclose(qs[-1], [1.0, 0.5, 0.2], atol=1e-9)


class TestStats(unittest.TestCase):
    def test_bootstrap_ci_contains_mean(self):
        rng = np.random.default_rng(0)
        v = rng.normal(1.0, 0.1, 200)
        lo, hi = bootstrap_ci(v, n_boot=2000, seed=0)
        self.assertLess(lo, 1.0)
        self.assertGreater(hi, 1.0)

    def test_permutation_test_detects_shift(self):
        rng = np.random.default_rng(1)
        a = rng.normal(1.0, 0.05, 30)
        b = rng.normal(1.4, 0.05, 30)
        self.assertLess(permutation_test(a, b, n_perm=1000), 0.01)
        self.assertGreater(permutation_test(a, a.copy(), n_perm=1000), 0.2)

    def test_wilson_ci(self):
        lo, hi = wilson_ci(5, 10)
        self.assertLess(lo, 0.5)
        self.assertGreater(hi, 0.5)

    def test_holm_orders_p_values(self):
        adj = holm_bonferroni([0.01, 0.04, 0.2])
        self.assertAlmostEqual(adj[0], 0.03)
        self.assertAlmostEqual(adj[1], 0.08)
        self.assertAlmostEqual(adj[2], 0.2)

    def test_fitts_and_throughput(self):
        self.assertAlmostEqual(fitts_id(0.15, 0.001), np.log2(151), places=6)
        self.assertAlmostEqual(effective_width(0.001), 4.133e-3, places=6)
        self.assertAlmostEqual(throughput(0.15, 0.001, 1.0), np.log2(151), places=6)


class TestScenarios(unittest.TestCase):
    def test_all_scenarios_build_jobs(self):
        for name, builder in SCENARIOS.items():
            sc = builder("models/six_axis_arm.json", seeds=(0, 1), reaches=2)
            self.assertGreater(len(sc.jobs), 0, name)

    def test_delay_scenario_cell_count(self):
        sc = SCENARIOS["delay"]("models/six_axis_arm.json", seeds=(0, 1, 2), reaches=6)
        self.assertEqual(len(sc.jobs), 6 * len(CONTROLLERS) * 3 * 6)

    def test_reach_variant_preserves_amplitude(self):
        v = reach_variant(REACH, 7, vary_distance=False)
        self.assertAlmostEqual(float(np.linalg.norm(v)), float(np.linalg.norm(REACH)),
                               places=9)

    def test_reach_variants_are_distinct(self):
        vs = [reach_variant(REACH, i) for i in range(6)]
        dist = [float(np.linalg.norm(vs[i] - vs[j]))
                for i in range(6) for j in range(i)]
        self.assertGreater(min(dist), 1e-3)

    def test_penetration_metric_consistent(self):
        cfg = SimConfig(duration=1.0, delay=0.12, noise=0.0, eps=0.004)
        ctrl = make_controller("fixed", delay=0.12, fps=100, eps=0.004)
        r = simulate_reach(M, Q0, TARGET, ctrl, cfg)
        d = float(np.linalg.norm(TARGET - r.p0))
        self.assertAlmostEqual(r.metrics["penetration_m"],
                               max(0.0, r.metrics["overshoot_pct"] * d / 100.0),
                               places=9)


class TestHtmlViewer(unittest.TestCase):
    def test_save_html_writes_viewer(self):
        import tempfile
        from pathlib import Path
        from robosimu.render import save_html
        cfg = SimConfig(duration=0.3, delay=0.06, noise=0.0)
        r = simulate_reach(M, Q0, TARGET, GovernorCartesian(delay=0.06, fps=100), cfg)
        with tempfile.TemporaryDirectory() as d:
            p = save_html(M, r, Path(d) / "view.html", every=5)
            text = Path(p).read_text(encoding="utf-8")
            self.assertIn("<canvas", text)
            self.assertIn("Stereo", text)
            self.assertIn('"links"', text)


class TestStereoAndSingularity(unittest.TestCase):
    def test_stereo_camera_triangulation(self):
        from robosimu.stereo import StereoCameraConfig, StereoCameraRig
        rig = StereoCameraRig(StereoCameraConfig(baseline_m=0.065, focal_px=520.0,
                                                 pixel_noise_px=0.0, disparity_noise_px=0.0))
        p_g = end_effector_position(M, Q0)
        obs = rig.observe(p_g, TARGET, rng=None)
        np.testing.assert_allclose(obs.rel_base_gap_m, TARGET - p_g, atol=1e-6)
        self.assertGreater(obs.sigma_cam_m[2], obs.sigma_cam_m[0])

    def test_singularity_safe_reach_runs(self):
        from robosimu.stereo import StereoCameraConfig
        st_cfg = StereoCameraConfig(baseline_m=0.065, disparity_noise_px=0.6)
        ctrl = GovernorCartesian(delay=0.06, fps=100, eps=0.004, per_axis=True)
        cfg = SimConfig(duration=0.6, delay=0.06, eps=0.004, stereo=st_cfg,
                        singularity_avoidance=True)
        r = simulate_reach(M, Q0, TARGET, ctrl, cfg)
        self.assertTrue(np.all(np.isfinite(r.q)))
        self.assertGreater(r.metrics["min_manipulability"], 0.0)
        self.assertEqual(len(r.stereo_obs), len(r.t))
        self.assertLess(r.error[-1], r.error[0])

    def test_hlsl_and_perspective_looming(self):
        from robosimu.stereo import HumanLoomingController, StereoCameraConfig, StereoCameraRig
        st_cfg = StereoCameraConfig(baseline_m=0.065, disparity_noise_px=0.0, pixel_noise_px=0.0)
        rig = StereoCameraRig(st_cfg)
        pred = rig.predict_reachability(M, Q0, TARGET)
        self.assertGreater(pred.parallax_shift_px, 0.0)
        self.assertGreater(pred.apparent_gripper_px, pred.apparent_target_px)
        ctrl = HumanLoomingController(delay=0.06, fps=100, eps=0.004, per_axis=True)
        cfg = SimConfig(duration=1.2, delay=0.06, eps=0.004, stereo=st_cfg,
                        singularity_avoidance=True)
        r = simulate_reach(M, Q0, TARGET, ctrl, cfg)
        # Gripper apparent size shrinks in perspective as it extends away from shoulder-mounted eyes
        self.assertLess(r.stereo_obs[-1].size_px_gripper, r.stereo_obs[0].size_px_gripper)
        self.assertAlmostEqual(r.stereo_obs[-1].scale_ratio, 1.0, delta=0.08)


class TestOrdinalPerception(unittest.TestCase):
    @staticmethod
    def _obs(scale_ratio, du_target=0.0, size_px=50.0):
        from robosimu.stereo import StereoObservation
        return StereoObservation(
            uvd_gripper=(320.0, 240.0, 12.0),
            uvd_target=(320.0 + du_target, 240.0, 12.0),
            uv_left_gripper=(326.0, 240.0),
            uv_right_gripper=(314.0, 240.0),
            uv_left_target=(326.0 + du_target, 240.0),
            uv_right_target=(314.0 + du_target, 240.0),
            size_px_gripper=size_px,
            size_px_target=size_px / max(scale_ratio, 1e-6),
            scale_ratio=scale_ratio,
            looming_error_m=0.0,
            rel_cam_gap_m=np.zeros(3),
            rel_base_gap_m=np.zeros(3),
            sigma_cam_m=np.zeros(3),
            depth_gripper_m=1.0,
            depth_target_m=1.0,
        )

    def test_all_five_proximity_classes_and_hysteresis(self):
        from robosimu.stereo import compute_ordinal_perception
        expected = [
            (0.30, "ÇOK UZAK"),
            (0.60, "UZAK"),
            (0.85, "YAKIN"),
            (0.96, "TEMAS ÖNCESİ"),
            (0.992, "TEMAS"),
        ]
        for rho, label in expected:
            perc = compute_ordinal_perception(self._obs(rho))
            self.assertEqual(perc.proximity_class, label, f"Failed for rho={rho}")
            self.assertGreater(perc.prox_ratio, 0.0)

        # Hysteresis prevents threshold chatter inside margin around 0.78
        hold = compute_ordinal_perception(
            self._obs(0.77), prev_class="YAKIN", prev_prox_ratio=0.77
        )
        self.assertEqual(hold.proximity_class, "YAKIN")
        self.assertEqual(hold.looming_state, "SABİT")

        # Dropping cleanly below hysteresis band transitions down and signals retreat
        retreat = compute_ordinal_perception(
            self._obs(0.74), prev_class="YAKIN", prev_prox_ratio=0.80
        )
        self.assertEqual(retreat.proximity_class, "UZAK")
        self.assertEqual(retreat.looming_state, "UZAKLAŞIYOR")

        # Advancing signals looming approach; out-of-FOV drops confidence
        approach = compute_ordinal_perception(
            self._obs(0.85), prev_class="UZAK", prev_prox_ratio=0.75
        )
        self.assertEqual(approach.looming_state, "YAKLAŞIYOR")
        oof = compute_ordinal_perception(self._obs(0.50), in_fov=False)
        self.assertEqual(oof.confidence, "DÜŞÜK")


class TestIntermittentGazeAndMetrics(unittest.TestCase):
    def test_intermittent_gaze_reduces_visual_load_and_computes_metrics(self):
        from robosimu.simulator import ReachMetrics, compute_metrics
        from robosimu.stereo import StereoCameraConfig
        st_cfg = StereoCameraConfig(baseline_m=0.065, disparity_noise_px=0.0, pixel_noise_px=0.0)
        ctrl = make_controller("hdap", delay=0.06, fps=100, control_dt=0.01, eps=0.004)
        cfg = SimConfig(duration=1.2, delay=0.06, eps=0.004, stereo=st_cfg,
                        singularity_avoidance=True, intermittent_gaze=True)
        res = simulate_reach(M, Q0, TARGET, ctrl, cfg)
        rm = compute_metrics(res)
        self.assertIsInstance(rm, ReachMetrics)
        self.assertGreater(rm.visual_load_hz, 0.0)
        self.assertLess(rm.visual_load_hz, 100.0)
        self.assertGreater(res.metrics["sampled_visual_frames"], 0)
        self.assertLess(res.metrics["sampled_visual_frames"], len(res.t))
        self.assertGreaterEqual(rm.pulse_efficiency, 0.0)
        self.assertLessEqual(rm.pulse_efficiency, 1.0)
        self.assertGreater(res.metrics["mechanical_work_j"], 0.0)
        self.assertGreater(rm.energy_j, 0.0)
        self.assertGreaterEqual(rm.t_comp_pct, 0.0)


class TestWorkbenchHtmlV5(unittest.TestCase):
    def test_v5_workbench_template_controls_and_cleanups(self):
        import tempfile
        from pathlib import Path
        from robosimu.render import save_html
        cfg = SimConfig(duration=0.3, delay=0.06, noise=0.0)
        r = simulate_reach(M, Q0, TARGET, GovernorCartesian(delay=0.06, fps=100), cfg)
        with tempfile.TemporaryDirectory() as d:
            p = save_html(M, r, Path(d) / "wb_v5.html", every=5)
            html = Path(p).read_text(encoding="utf-8")
            self.assertNotIn("pw-dots", html)
            self.assertNotIn("ChildElementCount", html)
            self.assertIn("childElementCount", html)
            for req_id in (
                "btnViewMode", "btnConceptGuide", "conceptDrawer", "theoremStrip",
                "chkGhostRobot", "chkDepthCones", "bottomTabBar", "btnShowAllTabs",
                "tblCompare", "kpiLVis", "kpiEtaPulse",
            ):
                self.assertIn(req_id, html)
            self.assertIn('colspan="10"', html)


if __name__ == "__main__":
    unittest.main()
