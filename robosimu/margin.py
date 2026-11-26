"""Exact monotone-approach margin of the Perceptual 1/e Governor (v6 theory).

This module is pure NumPy and does not import the simulator.  It answers three
questions a reviewer asks about the speed law  k = (1/e) / delta_eff:

1. Is ``delta_eff = delta + L + (n-1)T/2 + T/2`` justified, or ad hoc?
   For the sampled loop  e[j+1] = e[j] - g * mean(e[j-d-n+1 .. j-d]),  g = kT,
   the exact largest non-overshooting gain is computed here
   (``g_max_exact``).  For n = 1 it has the closed form
   g* = d^d / (d+1)^(d+1)  (classical non-oscillation bound of the linear
   delay difference equation), and k* (dT + T/2) -> 1/e as d grows.
   With averaging, k* * delta_eff stays within ~1.5 % of 1/e for d >= 3, so the
   law sits *on* the exact boundary, not below it.

2. What does a wrong body schema do?
   With a learned image Jacobian J_hat and the true one J, every closed-loop
   mode is scaled by an eigenvalue lambda_i of M = J J_hat^+ (damped pseudo-
   inverse).  The approach is monotone in every mode iff each lambda_i is real
   and lambda_i <= g*/g_law (~1 for this law).  lambda_max > 1 predicts the
   overshoot (``predicted_overshoot_pct``); lambda_min < 1 predicts the slow
   axis (movement time grows ~ 1/lambda_min).

3. What does a mis-reported or removed delay do?
   It is the same mechanism: the gain is multiplied by delta_eff_true /
   delta_eff_used, which acts exactly like lambda.

None of this is used by the controller at run time; it is analysis code.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

INV_E = 1.0 / math.e


# ---------------------------------------------------------------- scalar loop
def g_star_closed_form(d: int) -> float:
    """Largest non-oscillatory gain of e[j+1] = e[j] - g e[j-d] (d >= 0 samples)."""
    d = int(d)
    if d == 0:
        return 1.0
    return d ** d / (d + 1) ** (d + 1)


def step_response(g: float, d: int, nwin: int = 1, steps: int = 3000) -> np.ndarray:
    """Error sequence of the sampled delayed loop with an n-frame moving average.

    History is 1 (a unit step was just applied); returns e[0..steps].
    """
    d = int(d); nwin = max(1, int(nwin))
    h = [1.0] * (d + nwin + 1)
    for _ in range(steps):
        a = sum(h[len(h) - d - nwin:len(h) - d]) / nwin
        h.append(h[-1] - g * a)
    return np.asarray(h[d + nwin:], float)


def overshoot_pct(g: float, d: int, nwin: int = 1, steps: int = 3000) -> float:
    """Peak travel past the target (percent of the step) for the scalar loop."""
    e = step_response(g, d, nwin, steps)
    return float(max(0.0, -e.min()) * 100.0)


def g_max_exact(d: int, nwin: int = 1, tol: float = 1e-13, iters: int = 48) -> float:
    """Largest gain g with no zero crossing (bisection on the exact recursion)."""
    lo, hi = 0.0, 1.0
    for _ in range(iters):
        m = 0.5 * (lo + hi)
        if step_response(m, d, nwin, 2500).min() < -tol:
            hi = m
        else:
            lo = m
    return lo


def delta_eff(delta: float, L: float, nwin: int, T: float, hold: float | None = None) -> float:
    """Effective delay used by the Perceptual 1/e Governor."""
    hold = T if hold is None else hold
    return float(delta + L + (nwin - 1) * T / 2.0 + hold / 2.0)


def law_gain(delta: float, L: float, nwin: int, T: float, c: float = INV_E) -> float:
    """k = c / delta_eff (1/s)."""
    return c / delta_eff(delta, L, nwin, T)


@dataclass
class BoundaryRow:
    delay_s: float
    d_samples: int
    k_exact: float
    k_law: float
    ratio: float          # k_exact / k_law  (>= 1: law is conservative)
    c_exact: float        # k_exact * delta_eff  (1/e if delta_eff is exact)


def boundary_table(delays, L=0.05, nwin=8, T=0.01):
    rows = []
    for dl in delays:
        d = int(round((dl + L) / T))
        k_ex = g_max_exact(d, nwin) / T
        k_lw = law_gain(dl, L, nwin, T)
        de = delta_eff(dl, L, nwin, T)
        rows.append(BoundaryRow(dl, d, k_ex, k_lw, k_ex / k_lw, k_ex * de))
    return rows


# ------------------------------------------------------------- body schema
def dls_pinv(J: np.ndarray, lam: float = 0.3) -> np.ndarray:
    """Damped least-squares inverse used by the controller."""
    J = np.asarray(J, float)
    return J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(J.shape[0]))


def mismatch_eigs(J_true: np.ndarray, J_hat: np.ndarray, lam: float = 0.3) -> np.ndarray:
    """Eigenvalues of M = J_true J_hat^+ ; 1 everywhere for a perfect schema."""
    M = np.asarray(J_true, float) @ dls_pinv(J_hat, lam)
    return np.linalg.eigvals(M)


def predicted_overshoot_pct(lmbda: float, delay: float, L=0.05, nwin=8, T=0.01) -> float:
    """Overshoot of a single mode whose gain is scaled by lambda."""
    d = int(round((delay + L) / T))
    g = law_gain(delay, L, nwin, T) * T * float(lmbda)
    return overshoot_pct(g, d, nwin)


def predicted_settle_s(lmbda: float, delay: float, L=0.05, nwin=8, T=0.01, tol=0.05) -> float:
    """Time for a mode to enter a +-tol band (fraction of the step)."""
    d = int(round((delay + L) / T))
    g = law_gain(delay, L, nwin, T) * T * float(lmbda)
    e = np.abs(step_response(g, d, nwin, 4000))
    idx = np.where(e > tol)[0]
    return float((idx[-1] + 1) * T) if idx.size else 0.0


def gain_scale_from_delay_report(delay_true, delay_used, L=0.05, nwin=8, T=0.01) -> float:
    """Gain multiplier caused by using a wrong delay inside delta_eff."""
    return delta_eff(delay_true, L, nwin, T) / delta_eff(delay_used, L, nwin, T)


def certified_c(lambda_max: float, c: float = INV_E) -> float:
    """Gain constant that restores the monotone margin for a known schema error."""
    return c / max(1.0, float(lambda_max))
