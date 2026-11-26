# -*- coding: utf-8 -*-
"""T1–T8 — pre-registered tests for the A3 fair-simulation programme.

Definitions in ``docs/dogru_yol_Faz0.md`` §0.5; hypotheses H1–H5 in §0.4.
"""
import math
import unittest
from pathlib import Path

import numpy as np

from robosimu import a3, pareto
from robosimu.controllers import DelayScaledP, GovernorCartesian, make_controller
from robosimu.tasks import (tune_targets, test_targets, ood_targets,
                            assert_disjoint, ood_is_outside_envelope)

KAPPA = 1.0 / math.e


class T1_DelayScaledP(unittest.TestCase):
    def test_gain_is_c_over_delta(self):
        c = 0.2
        d = 0.10
        ctrl = DelayScaledP(delay=d, c=c)
        self.assertAlmostEqual(ctrl.k * d, c, places=9)

    def test_c_equals_inv_e_matches_governor(self):
        # governor k = kappa / (delta + n_max*T/2); with sigma=0 the boxcar
        # still adds one frame (n=1) -> delta_eff = delta + T/2.
        d = 0.06
        dps = DelayScaledP(delay=d, c=KAPPA)
        gov = GovernorCartesian(delay=d, fps=100.0, sigma=0.0)
        self.assertAlmostEqual(gov.k, KAPPA / (d + 0.005), places=9)
        self.assertAlmostEqual(dps.k, KAPPA / d, places=9)


class T2_T3_TargetSets(unittest.TestCase):
    def test_disjoint(self):
        assert_disjoint()

    def test_ood_outside_envelope(self):
        self.assertTrue(ood_is_outside_envelope())


class T4_ClipKappa(unittest.TestCase):
    def test_clip_enforces_bound(self):
        ctrl = DelayScaledP(delay=0.05, c=0.9, clip_kappa=True)
        self.assertLessEqual(ctrl.k * 0.05, KAPPA + 1e-12)

    def test_small_residual_overshoot_at_bound(self):
        # The eps dead-zone under delay leaves a small residual overshoot
        # (~0.4 % here); the bound guarantees monotone approach, not 0.0 %.
        out = a3.simulate_reach(0.20, 0.10,
                                make_controller("dps", delay=0.10, c=KAPPA))
        self.assertLessEqual(out.overshoot, 0.5)
        self.assertGreater(out.overshoot, 0.0)


class T5_DelayMatrix(unittest.TestCase):
    def test_dps_inv_e_safe_everywhere(self):
        for d in (0.03, 0.06, 0.10, 0.15, 0.20):
            out = a3.simulate_reach(0.20, d,
                                    make_controller("dps", delay=d, c=KAPPA))
            self.assertLessEqual(out.overshoot, 1.0, f"delta={d}")


class T6_TuneOnly(unittest.TestCase):
    def test_tuning_never_sees_test_or_ood(self):
        """Behavioural leakage guard: every amplitude the tuner evaluates
        must belong to the pre-registered TUNE set."""
        import importlib.util
        from unittest import mock
        spec = importlib.util.spec_from_file_location(
            "tune_per_condition",
            Path(__file__).resolve().parent.parent / "scripts" / "tune_per_condition.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        seen = []
        real = a3.run_condition

        def spy(name, delay, targets, seeds, **kw):
            seen.extend(targets.amplitudes)
            return real(name, delay, targets, seeds, **kw)

        cs = np.linspace(0.05, 0.60, 6)
        with mock.patch.object(a3, "run_condition", side_effect=spy):
            mod.tune_delay(0.06, cs, (0, 1, 2))
        tune_amps = set(tune_targets((0, 1, 2)).amplitudes)
        self.assertTrue(set(seen) <= tune_amps)
        self.assertFalse(set(seen) & set(test_targets((0, 1, 2)).amplitudes))
        self.assertFalse(set(seen) & set(ood_targets((0, 1, 2)).amplitudes))


class T7_Misreport(unittest.TestCase):
    def test_overestimate_is_safe_underestimate_degrades(self):
        # Safe direction: believe 120 ms, actual 60 ms -> tiny overshoot.
        over = a3.simulate_reach(0.20, 0.06,
                                 make_controller("dps", delay=0.12, c=KAPPA))
        self.assertLessEqual(over.overshoot, 2.0)
        # Unsafe direction: believe 60 ms, actual 120 ms -> the bound is
        # violated (k*delta_actual = 2c) and overshoot grows.
        under = a3.simulate_reach(0.20, 0.12,
                                  make_controller("dps", delay=0.06, c=KAPPA))
        self.assertGreater(under.overshoot, 5.0)


class T8_Cmax(unittest.TestCase):
    def test_c_max_near_inv_e(self):
        cs = np.linspace(0.10, 0.60, 26)   # 0.02 step: grid resolution << tol
        scan = pareto.scan_c("dps", 0.06, cs, test_targets((0, 1, 2)), (0, 1, 2))
        cm = pareto.c_max(scan)
        self.assertIsNotNone(cm)
        self.assertLessEqual(abs(cm - KAPPA), 0.05)


if __name__ == "__main__":
    unittest.main()
