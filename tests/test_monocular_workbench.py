from pathlib import Path
import unittest

import numpy as np

from robosimu import SimConfig, simulate_reach
from robosimu.model import load_model, load_postures
from robosimu.stereo import HumanApproachController, StereoCameraConfig


class TestExpandedPosturesAndMonocularParallax(unittest.TestCase):
    def test_nine_anatomical_postures_within_joint_limits(self):
        model = load_model("models/human_arm.json")
        postures = load_postures("models/human_arm.json")
        expected = {
            "hanging_natural",
            "hanging_vertical",
            "ready_desk",
            "reach_high",
            "side_guard",
            "low_table_pick",
            "shelf_pre_reach",
            "cross_body_rest",
            "retracted_compact",
        }
        self.assertTrue(expected.issubset(set(postures.keys())))
        for name in expected:
            q = np.radians(postures[name])
            self.assertEqual(len(q), 6)
            for j, link in enumerate(model.links):
                lo, hi = link.limits
                self.assertGreaterEqual(q[j], lo - 1e-6, msg=f"{name} joint {j} below limit")
                self.assertLessEqual(q[j], hi + 1e-6, msg=f"{name} joint {j} above limit")

    def test_monocular_active_neck_parallax_converges(self):
        model = load_model("models/human_arm.json")
        postures = load_postures("models/human_arm.json")
        q0 = np.radians(postures["hanging_natural"])
        target = np.array([0.45, 0.10, 0.95])
        for mode in ("mono_left", "mono_right"):
            st_cfg = StereoCameraConfig(
                baseline_m=0.064,
                focal_px=520.0,
                cam_pos=(-0.10, 0.0, 1.12),
                look_at=(0.35, 0.0, 0.85),
                pixel_noise_px=0.35,
                disparity_noise_px=0.35,
                vision_mode=mode,
                neck_parallax_amp_m=0.038,
                neck_parallax_freq_hz=2.2,
            )
            cfg = SimConfig(
                duration=4.5,
                control_dt=0.01,
                delay=0.06,
                eps=0.005,
                max_qd=1.8,
                max_qdd=6.0,
                noise=0.0,
                singularity_avoidance=True,
                intermittent_gaze=False,
                stereo=st_cfg,
                seed=42,
            )
            ctrl = HumanApproachController(
                delay=0.06, eps=0.004, vmax=0.60, per_axis=True, speed_mode="normal"
            )
            res = simulate_reach(model, q0, target, ctrl, cfg)
            self.assertLess(res.metrics["final_error_m"], 0.005)
            baselines = [o.neck_baseline_m for o in res.stereo_obs]
            parallax_px = [abs(o.motion_parallax_px) for o in res.stereo_obs]
            self.assertGreater(max(baselines), 0.05)
            self.assertGreater(max(parallax_px), 20.0)
            self.assertTrue(all(o.vision_mode == mode for o in res.stereo_obs))
            offsets_y = [float(o.head_offset_m[1]) for o in res.stereo_obs]
            reversals = sum(
                1 for i in range(2, len(offsets_y))
                if (offsets_y[i - 1] - offsets_y[i - 2]) * (offsets_y[i] - offsets_y[i - 1]) < -1e-9
            )
            self.assertEqual(reversals, 0)

    def test_static_monocular_has_zero_neck_baseline(self):
        model = load_model("models/human_arm.json")
        postures = load_postures("models/human_arm.json")
        st_cfg = StereoCameraConfig(vision_mode="mono_static_left")
        cfg = SimConfig(
            duration=2.5,
            control_dt=0.01,
            delay=0.06,
            stereo=st_cfg,
            seed=42,
        )
        ctrl = HumanApproachController(delay=0.06, eps=0.004, vmax=0.60, per_axis=True)
        res = simulate_reach(model, np.radians(postures["hanging_natural"]), np.array([0.45, 0.10, 0.95]), ctrl, cfg)
        self.assertEqual(max(o.neck_baseline_m for o in res.stereo_obs), 0.0)

    def test_workbench_html_contains_v51_features(self):
        html = Path("figures/human_arm_workbench.html").read_text(encoding="utf-8")
        for token in (
            "position: sticky",
            "flex-wrap: nowrap",
            "overflow-x: clip",
            "#tabPanels > .card[data-tab]",
            'id="selVisionMode"',
            'data-preset="mononeck"',
            "neckParallaxOffsetAt",
            "side_guard",
            "low_table_pick",
            "shelf_pre_reach",
            "cross_body_rest",
            "retracted_compact",
            "mono_reach",
            "connector_insert",
            "axial_pronation_reach",
            "axial_supination_grasp",
            'id="jointPanel"',
            "getJointWorldPositions",
            "axial_x_rotate",
            "pour_axial_rot",
            "svgControlDiagram",
            "svgDashPhase",
            "svgDashEInit",
            "svgDashComp",
            "svgDashPulse",
            "updateControlDiagramLive",
            'id="btnViewPersp"',
            'id="btnViewFront"',
            'id="btnViewSide"',
            'id="btnViewTop"',
            'id="chkReachSphere"',
            "setCameraView",
            "drawBaseFrameTriad",
            "BEL GÖSTERGECİ",
            'id="benchSuiteCard"',
            'id="btnRunAcademicBenchmark"',
            "F₀ (O₀-XYZ)",
            "Eklem Referans Ekseni (F_i) Hareket Modu",
            'id="kinSvgOrient"',
            'id="rngTargetRoll"',
            'id="rngTargetPitch"',
            'id="rngTargetYaw"',
            'id="telOrient"',
            "buildTargetRotationSO3",
            "extractHandRPYDeg",
            "computeOrientationErrorSO3",
        ):
            self.assertIn(token, html)

        self.assertNotIn("drawUCSGizmo", html)
        self.assertNotIn("Local UCS", html)

    def test_spatial_6dof_so3_orientation_and_manipulability(self):
        from robosimu.kinematics import (
            build_grasp_rotation_so3,
            end_effector_rotation,
            extract_rpy_deg,
            orientation_error_so3,
        )
        from robosimu.stereo import (
            compute_singularity_metrics,
            singularity_safe_joint_velocities,
        )

        R_target = build_grasp_rotation_so3(18.5, -14.2, 22.7)
        self.assertAlmostEqual(float(np.linalg.det(R_target)), 1.0, places=6)
        rpy_back = extract_rpy_deg(R_target)
        np.testing.assert_allclose(rpy_back, [18.5, -14.2, 22.7], atol=1e-5)

        model = load_model("models/human_arm.json")
        postures = load_postures("models/human_arm.json")
        q = np.radians(postures["ready_desk"]).copy()
        qd = np.zeros(model.n)
        _, status = compute_singularity_metrics(model, q, qd)
        self.assertGreater(status.rotational_manipulability, 0.0)

        R0 = end_effector_rotation(model, q)
        R_goal = build_grasp_rotation_so3(22.0, -15.0, 18.0)
        _, err0 = orientation_error_so3(R0, R_goal)
        dt = 0.02
        for _ in range(180):
            qd_cmd, _ = singularity_safe_joint_velocities(
                model, q, qd, np.zeros(3), R_desired=R_goal, k_orient=4.0
            )
            qd = qd_cmd
            q = np.array([
                float(np.clip(q[j] + qd[j] * dt, model.links[j].limits[0], model.links[j].limits[1]))
                for j in range(model.n)
            ])
        R1 = end_effector_rotation(model, q)
        _, err1 = orientation_error_so3(R1, R_goal)
        self.assertLess(err1, 5.0)
        self.assertLess(err1, err0 * 0.25)


class TestAcademicBenchmarkSuite(unittest.TestCase):
    def test_academic_benchmark_cli_and_contrasts(self):
        import tempfile
        from pathlib import Path
        from robosimu.benchmark import run_academic_benchmark

        model_path = Path(__file__).resolve().parents[1] / "models" / "human_arm.json"
        with tempfile.TemporaryDirectory() as tmp:
            summary = run_academic_benchmark(model_path, out_dir=tmp, seeds=(0,))
            out = Path(tmp)
            self.assertTrue((out / "academic_benchmark.csv").exists())
            self.assertTrue((out / "academic_benchmark.md").exists())
            self.assertTrue((out / "academic_benchmark.json").exists())

            rows = summary["rows"]
            self.assertEqual(len(rows), 32)
            by_key = {(r["scenario_id"], r["controller"]): r for r in rows}

            # S2 (120 ms delay): HDAP stays stable & low-overshoot compared to high-overshoot/unstable baselines
            s2_hdap = by_key[("S2_high_latency_120ms", "hdap")]
            s2_pid = by_key[("S2_high_latency_120ms", "pid")]
            self.assertLess(s2_hdap["final_error_mm_mean"], 5.0)
            self.assertLess(s2_hdap["overshoot_pct_mean"], s2_pid["overshoot_pct_mean"])

            # S4 (monocular parallax): active neck parallax strongly beats static monocular left eye
            s4_act = by_key[("S4_monocular_parallax", "hdap")]
            s4_stat = by_key[("S4_monocular_parallax", "hdap_mono_static")]
            self.assertGreater(s4_act["neck_baseline_mm"], 15.0)
            self.assertLess(s4_act["final_error_mm_mean"], 8.0)
            self.assertGreater(s4_stat["final_error_mm_mean"], 20.0)

            # S5 (out-of-sphere lateral reach): synchronized waist yaw reaches cleanly; fixed waist cannot
            s5_sync = by_key[("S5_lateral_waist_sync", "hdap")]
            s5_fixed = by_key[("S5_lateral_waist_sync", "hdap_no_waist")]
            self.assertGreater(abs(s5_sync["waist_deg"]), 15.0)
            self.assertLess(s5_sync["final_error_mm_mean"], 8.0)
            self.assertGreater(s5_fixed["final_error_mm_mean"], 40.0)


if __name__ == "__main__":
    unittest.main()
