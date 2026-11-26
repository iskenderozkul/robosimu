"""v6 theory tests: the 1/e law sits on the exact sampled monotone boundary."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robosimu import margin as mg  # noqa: E402


def test_closed_form_matches_recursion():
    for d in (1, 3, 6, 11, 20):
        g = mg.g_star_closed_form(d)
        assert mg.step_response(0.995 * g, d).min() >= -1e-12
        assert mg.step_response(1.03 * g, d).min() < 0.0


def test_closed_form_tends_to_inverse_e():
    # k* (dT + T/2) -> 1/e ; error below 0.5 % from d = 3 on
    for d in (3, 6, 10, 20, 40):
        c = mg.g_star_closed_form(d) * (d + 0.5)
        assert abs(c - 1 / math.e) / (1 / math.e) < 5e-3


def test_delta_eff_with_averaging_is_on_the_boundary():
    rows = mg.boundary_table([0.03, 0.06, 0.10, 0.15, 0.20])
    for r in rows:
        assert 0.97 < r.ratio < 1.03, r
        assert abs(r.c_exact - 1 / math.e) < 0.01, r


def test_mismatch_eigs_identity():
    rng = np.random.default_rng(0)
    J = rng.normal(size=(3, 5)) * 400
    ev = mg.mismatch_eigs(J, J, lam=0.3)
    assert np.allclose(ev, 1.0, atol=1e-5)


def test_overgain_predicts_overshoot_and_undergain_predicts_slowness():
    assert mg.predicted_overshoot_pct(1.0, 0.06) < 0.1
    assert mg.predicted_overshoot_pct(1.5, 0.06) > 5.0
    assert mg.predicted_settle_s(0.25, 0.06) > 3 * mg.predicted_settle_s(1.0, 0.06)


def test_removed_delay_term_predicts_ablation_magnitude():
    # paper ablation: delay term removed -> observed 11.6 % (60 ms), 41.9 % (150 ms)
    s60 = mg.gain_scale_from_delay_report(0.06, 0.001)
    s150 = mg.gain_scale_from_delay_report(0.15, 0.001)
    p60 = mg.predicted_overshoot_pct(s60, 0.06)
    p150 = mg.predicted_overshoot_pct(s150, 0.15)
    assert 8.0 < p60 < 16.0
    assert 35.0 < p150 < 55.0
